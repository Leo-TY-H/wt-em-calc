"""Read-only x64 PE reconnaissance, with preferred (unrelocated) addresses.

The exception directory identifies runtime-function ranges, not every function
or the complete control-flow graph. String references are locators, not proof
of a recovered algorithm. No executable is launched, attached to or modified.

Format references:
https://learn.microsoft.com/en-us/windows/win32/debug/pe-format
https://learn.microsoft.com/en-us/cpp/build/exception-handling-x64
"""
import argparse
import bisect
import hashlib
import json
from pathlib import Path
import re
import struct


class PE:
    def __init__(self, path):
        self.path = Path(path)
        self.data = self.path.read_bytes()
        self.sha256 = hashlib.sha256(self.data).hexdigest()
        if self.data[:2] != b'MZ':
            raise ValueError('Expected a PE executable (MZ header)')
        pe = self.unpack('<I', 0x3c)[0]
        if self.data[pe:pe+4] != b'PE\0\0':
            raise ValueError('Invalid PE signature')
        machine, count, self.timestamp, _, _, opt_size, _ = self.unpack('<HHIIIHH', pe+4)
        opt = pe+24
        if machine != 0x8664 or self.unpack('<H', opt)[0] != 0x20b:
            raise ValueError('Expected an AMD64 PE32+ image')
        if opt_size < 112 or not 0 < count <= 96:
            raise ValueError('Invalid PE optional header or section count')
        self.base = self.unpack('<Q', opt+24)[0]
        self.image_size, self.headers_size = self.unpack('<II', opt+56)
        self.sections = []
        for i in range(count):
            row = opt+opt_size+i*40
            name, size, rva, raw_size, offset = self.unpack('<8sIIII', row)
            flags = self.unpack('<I', row+36)[0]
            if raw_size and offset+raw_size > len(self.data):
                raise ValueError('Truncated section')
            self.sections.append(dict(name=name.rstrip(b'\0').decode('ascii', 'replace'),
                va=self.base+rva, size=size, offset=offset, filesize=raw_size,
                executable=bool(flags & 0x20000000)))
        directories = self.unpack('<I', opt+108)[0]
        self.functions = []
        if directories > 3:
            if opt_size < 144:
                raise ValueError('Truncated exception-directory entry')
            rva, size = self.unpack('<II', opt+112+3*8)
            if size:
                if size % 12:
                    raise ValueError('Invalid x64 runtime-function table size')
                data = self.read(self.base+rva, size)
                for start, end, unwind in struct.iter_unpack('<III', data):
                    if start == end == unwind == 0:
                        continue
                    if not 0 < start < end <= self.image_size:
                        raise ValueError('Invalid runtime-function range')
                    self.functions.append((self.base+start, self.base+end, self.base+unwind))
        self.functions.sort()
        self.starts = [f[0] for f in self.functions]

    def unpack(self, fmt, offset):
        if offset < 0 or offset+struct.calcsize(fmt) > len(self.data):
            raise ValueError('Truncated PE structure')
        return struct.unpack_from(fmt, self.data, offset)

    def file_offset(self, va, size=1):
        if size < 0:
            raise ValueError('Negative read length')
        if self.base <= va and va+size <= self.base+self.headers_size:
            offset = va-self.base
            if offset+size <= len(self.data):
                return offset
        for section in self.sections:
            if section['va'] <= va and va+size <= section['va']+section['filesize']:
                return section['offset']+va-section['va']
        raise ValueError('Address is not file-backed: '+hex(va))

    def read(self, va, size):
        offset = self.file_offset(va, size)
        return self.data[offset:offset+size]

    def function(self, va):
        index = bisect.bisect_right(self.starts, va)-1
        if index >= 0 and va < self.functions[index][1]:
            return self.functions[index]
        return None

    def cstrings(self):
        for section in self.sections:
            if section['executable']:
                continue
            data = self.read(section['va'], section['filesize'])
            for match in re.finditer(rb'[\x20-\x7e]{4,}\x00', data):
                yield section['va']+match.start(), match.group()[:-1].decode('ascii')


def references(binary, targets, calls=False):
    """Validate candidate encodings against linear decoding of their pdata range.

    Embedded data / branch-skipped bytes still require manual assembly review.
    Leaf functions without pdata are deliberately not attributed to neighbors.
    """
    from capstone import Cs, CS_ARCH_X86, CS_MODE_64, CS_OP_MEM, CS_OP_IMM
    from capstone.x86 import X86_REG_RIP
    cs = Cs(CS_ARCH_X86, CS_MODE_64)
    cs.detail = True
    candidates = {}
    pattern = rb'\xe8....' if calls else rb'[\x48-\x4f]\x8d[\x05\x0d\x15\x1d\x25\x2d\x35\x3d]....'
    for section in binary.sections:
        if not section['executable']:
            continue
        data = binary.read(section['va'], section['filesize'])
        # Include overlapping byte candidates: an immediate/displacement can
        # contain another apparent opcode before the real next instruction.
        for match in re.finditer(b'(?=('+pattern+b'))', data, re.S):
            address = section['va']+match.start()
            encoding = match.group(1)
            target = address+len(encoding)+struct.unpack('<i', encoding[-4:])[0]
            if target not in targets:
                continue
            function = binary.function(address)
            if function:
                candidates.setdefault(function, {})[address] = target
    result = []
    for function, addresses in sorted(candidates.items()):
        for ins in cs.disasm(binary.read(function[0], function[1]-function[0]), function[0]):
            if ins.address not in addresses:
                continue
            target = addresses[ins.address]
            valid = any((op.type == CS_OP_MEM and op.mem.base == X86_REG_RIP
                         and ins.address+ins.size+op.mem.disp == target)
                        or (calls and ins.mnemonic == 'call' and op.type == CS_OP_IMM and op.imm == target)
                        for op in ins.operands)
            if valid:
                result.append(dict(address=hex(ins.address), function=hex(function[0]),
                    function_end=hex(function[1]), target=hex(target),
                    label=targets[target], instruction=ins.mnemonic+' '+ins.op_str))
    return result


def disassemble(binary, address, destination):
    from capstone import Cs, CS_ARCH_X86, CS_MODE_64, CS_OP_MEM
    from capstone.x86 import X86_REG_RIP
    function = binary.function(address)
    if not function:
        raise ValueError('No runtime-function range contains '+hex(address))
    strings = dict(binary.cstrings())
    cs = Cs(CS_ARCH_X86, CS_MODE_64)
    cs.detail = True
    lines = []
    for ins in cs.disasm(binary.read(function[0], function[1]-function[0]), function[0]):
        notes = []
        for op in ins.operands:
            if op.type == CS_OP_MEM and op.mem.base == X86_REG_RIP:
                target = ins.address+ins.size+op.mem.disp
                notes.append(hex(target))
                if target in strings:
                    notes.append(repr(strings[target]))
        lines.append(f'{ins.address:016x}: {ins.mnemonic:10s} {ins.op_str}'+(' ; '+' '.join(notes) if notes else ''))
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text('\n'.join(lines)+'\n', encoding='utf-8')
    return len(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--pattern', default=r'instructor|overloadTime|critMult|autoTrim|constPitch|limitLoadfactor')
    parser.add_argument('--function', type=lambda value: int(value, 0))
    parser.add_argument('--callers', type=lambda value: int(value, 0))
    parser.add_argument('--out', type=Path, default=Path('analysis/windows-instructor'))
    args = parser.parse_args()
    binary = PE(args.binary)
    args.out.mkdir(parents=True, exist_ok=True)
    if args.function is not None:
        function = binary.function(args.function)
        if function is None:
            parser.error('Address has no runtime-function entry')
        dest = args.out/'disassembly'/f'{function[0]:016x}.asm'
        print(dest, disassemble(binary, args.function, dest), 'instructions')
    elif args.callers is not None:
        rows = references(binary, {args.callers:hex(args.callers)}, calls=True)
        dest = args.out/f'callers-{args.callers:x}.json'
        dest.write_text(json.dumps(rows, indent=2)+'\n', encoding='utf-8')
        print(json.dumps(rows, indent=2))
    else:
        pattern = re.compile(args.pattern, re.I)
        strings = {va:text for va,text in binary.cstrings() if pattern.search(text)}
        rows = references(binary, strings)
        report = dict(binary=str(binary.path.resolve()), sha256=binary.sha256,
            image_base=hex(binary.base), image_size=binary.image_size, sections=binary.sections,
            runtime_function_count=len(binary.functions),
            strings={hex(va):text for va,text in strings.items()}, references=rows,
            scope='Read-only candidate string references; not semantic or runtime validation.')
        (args.out/'pe-reconnaissance.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
        print(binary.sha256, len(binary.functions), 'runtime functions;', len(strings), 'strings;', len(rows), 'references')
        for row in rows:
            print(row['function'], row['address'], row['label'])


if __name__ == '__main__':
    main()
