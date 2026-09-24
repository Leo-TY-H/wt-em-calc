"""Read-only Mach-O function/string reconnaissance; addresses are unslid VAs."""
import argparse
import bisect
import hashlib
import json
from pathlib import Path
import re
import struct

DEFAULT_BINARY = str(Path(__file__).resolve().parents[1] / 'references/native/aces-820fee4a55601ffa459e2635da4ce1436cfc7adcc703a7c799380c95d136ebd5')


class MachO:
    def __init__(self, path=DEFAULT_BINARY):
        self.path = Path(path)
        self.data = self.path.read_bytes()
        magic, _, _, _, ncmds, _, _, _ = struct.unpack_from('<8I', self.data)
        if magic != 0xfeedfacf:
            raise ValueError('Expected a little-endian 64-bit Mach-O')
        self.sections, self.segments = [], []
        off, function_data = 32, None
        self.uuid = None
        for _ in range(ncmds):
            cmd, size = struct.unpack_from('<II', self.data, off)
            if cmd == 0x19:
                _, _, name, va, vs, fo, fs, _, _, ns, _ = struct.unpack_from('<II16sQQQQiiII', self.data, off)
                self.segments.append(dict(name=name.rstrip(b'\0').decode(), va=va, size=vs, offset=fo, filesize=fs))
                for i in range(ns):
                    sn, sg, addr, sz, offset = struct.unpack_from('<16s16sQQI', self.data, off + 72 + i * 80)
                    self.sections.append(dict(name=sn.rstrip(b'\0').decode(), segment=sg.rstrip(b'\0').decode(), va=addr, size=sz, offset=offset))
            elif cmd == 0x26:
                function_data = struct.unpack_from('<II', self.data, off + 8)
            elif cmd == 0x1b:
                self.uuid = self.data[off+8:off+24].hex()
            off += size
        self.base = next(s['va'] for s in self.segments if s['name'] == '__TEXT')
        self.starts = []
        if function_data:
            off, size = function_data
            end, addr = off + size, self.base
            while off < end:
                delta, shift = 0, 0
                while True:
                    b = self.data[off]; off += 1
                    delta |= (b & 127) << shift
                    if not b & 128: break
                    shift += 7
                if not delta: break
                addr += delta
                self.starts.append(addr)

    def file_offset(self, va):
        for s in self.segments:
            if s['va'] <= va < s['va'] + s['filesize']:
                return s['offset'] + va - s['va']
        raise ValueError(hex(va))

    def read(self, va, size):
        off = self.file_offset(va)
        return self.data[off:off+size]

    def function(self, va):
        idx = bisect.bisect_right(self.starts, va) - 1
        if idx < 0: return None
        return self.starts[idx]

    def function_end(self, va):
        idx = bisect.bisect_right(self.starts, va)
        return self.starts[idx] if idx < len(self.starts) else next(s['va']+s['size'] for s in self.sections if s['name']=='__text')

    def cstrings(self):
        sec = next(s for s in self.sections if s['name'] == '__cstring')
        data = self.data[sec['offset']:sec['offset']+sec['size']]
        for match in re.finditer(rb'[^\x00]+', data):
            yield sec['va']+match.start(), match.group().decode('utf-8', 'replace')


def scan(macho, out):
    pattern = re.compile(r'flightmodel|polares|summary forces|Downwash|ClToCm|Oswald|WingPlane|FuselagePlane|HorStabPlane|VerStabPlane|ClCrit|CdMin|lineClCoeff', re.I)
    strings = {va:s for va,s in macho.cstrings() if pattern.search(s)}
    # LEA reg,[RIP+disp32]. Validate hits by disassembling containing functions below.
    sec = next(s for s in macho.sections if s['name'] == '__text')
    data = macho.read(sec['va'], sec['size'])
    refs=[]
    for m in re.finditer(rb'[\x48-\x4f]\x8d[\x05\x0d\x15\x1d\x25\x2d\x35\x3d]....', data, re.S):
        va = sec['va'] + m.start()
        target = va + 7 + struct.unpack_from('<i',m.group(),3)[0]
        if target in strings:
            refs.append(dict(address=hex(va), function=hex(macho.function(va)), target=hex(target), text=strings[target]))
    out.mkdir(parents=True,exist_ok=True)
    (out/'string-xrefs.json').write_text(json.dumps(refs,indent=2))
    (out/'macho-layout.json').write_text(json.dumps(dict(sha256=hashlib.sha256(macho.data).hexdigest(),uuid=macho.uuid,base=hex(macho.base),sections=macho.sections,function_count=len(macho.starts)),indent=2))
    print(f'{len(macho.starts)} function starts; {len(refs)} candidate string xrefs')
    for ref in refs:
        if not ref['text'].startswith(('das_', 'das::', ' ::')):
            print(ref['function'],ref['address'],ref['text'])


def disassemble(macho, address, out):
    from capstone import Cs, CS_ARCH_X86, CS_MODE_64, CS_OP_IMM, CS_OP_MEM
    from capstone.x86 import X86_REG_RIP
    start=macho.function(address);end=macho.function_end(start)
    cs=Cs(CS_ARCH_X86,CS_MODE_64);cs.detail=True
    strings=dict(macho.cstrings())
    lines=[]
    for ins in cs.disasm(macho.read(start,end-start),start):
        notes=[]
        for op in ins.operands:
            target=None
            if op.type==CS_OP_MEM and op.mem.base==X86_REG_RIP:target=ins.address+ins.size+op.mem.disp
            elif op.type==CS_OP_IMM and ins.mnemonic in ('call','jmp'):target=op.imm
            if target is not None:
                notes.append(hex(target))
                if target in strings:notes.append(repr(strings[target]))
                elif ins.mnemonic.startswith(('movss','mulss','addss','subss','divss','minss','maxss','ucomiss')):
                    try: notes.append('f32='+repr(struct.unpack('<f',macho.read(target,4))[0]))
                    except ValueError:pass
        lines.append(f'{ins.address:016x}: {ins.mnemonic:10s} {ins.op_str}' + (' ; '+' '.join(notes) if notes else ''))
    out.mkdir(parents=True,exist_ok=True)
    dest=out/f'{start:016x}.asm';dest.write_text('\n'.join(lines)+'\n')
    print(dest,len(lines),'instructions')


if __name__ == '__main__':
    p=argparse.ArgumentParser();p.add_argument('--binary',default=DEFAULT_BINARY);p.add_argument('--function',type=lambda s:int(s,0));p.add_argument('--out',type=Path,default=Path('analysis'))
    a=p.parse_args();m=MachO(a.binary)
    if a.function:disassemble(m,a.function,a.out/'disassembly')
    else:scan(m,a.out)
