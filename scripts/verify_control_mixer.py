"""Execute the complete control mixer, including actual 1D/2D lookup code.

No hooks, stubs or mocked helpers. Prepared runtime tables are synthetic;
this test does not validate the BLK loader or upstream pilot/instructor logic.
"""
import json
import random
import struct
from pathlib import Path
from unicorn.x86_const import *
from component_assembly import f32
from control_mixer import mix, prepare_rows
from macho_scan import MachO
from verify_component_assembly import AssemblySlice, BASE, FRAME


class Mixer(AssemblySlice):
    def __init__(self):
        super().__init__()
        m = MachO()
        for va, size in [(0x1019e5000, 0x2000), (0x1071e4000, 0x30000)]:
            self.u.mem_map(va, size)
            self.u.mem_write(va, m.read(va, size))

    def table(self, descriptor, rows, nested=False):
        address = self.cursor
        stride = 32 if nested else 8 + 4 * (len(rows[0][2]) if rows else 3)
        self.cursor += max(16, stride * len(rows))
        self.u.mem_write(descriptor, struct.pack('<Q', address))
        self.u.mem_write(descriptor + 16, struct.pack('<II', len(rows), len(rows)))
        for i, (x, inv, values) in enumerate(rows):
            self.floats(address + stride*i, [x, inv])
            if nested:
                self.table(address + stride*i + 8, values)
            else:
                self.floats(address + stride*i + 8, values)

    def call(self, p, commands, inverted, bias, density, speed, mach, arcade):
        self.u.mem_write(BASE, bytes(0x30000))
        self.cursor = BASE + 0x1000
        for i in range(3):
            self.floats(BASE + 8*i, p['angles'][i])
            self.table(BASE + 0x18 + 24*i, p['tables_1d'][i])
            self.table(BASE + 0x60 + 24*i, p['tables_2d'][i], True)
        self.floats(BASE + 0xa8, [p['sensitivity']])
        self.table(BASE + 0xb0, p['sensitivity_curve'])
        self.table(BASE + 0xc8, p['arcade_curve'])
        self.floats(BASE + 0xe0, p['cl'] + p['cd'] + [p['wing_aoa']])
        inp, out, stop = BASE+0x800, BASE+0x900, BASE+0x2f000
        self.floats(inp, commands)
        self.u.mem_write(inp+12, bytes(inverted))
        self.floats(inp+16, [bias, density, speed, mach])
        self.u.mem_write(inp+32, bytes([arcade]))
        sp = FRAME-8
        self.u.mem_write(sp, struct.pack('<Q', stop))
        for reg, value in [(UC_X86_REG_RSP,sp),(UC_X86_REG_RDI,inp),
                           (UC_X86_REG_RSI,BASE),(UC_X86_REG_RDX,out)]:
            self.u.reg_write(reg, value)
        for i in range(16):
            self.xmm(i, [0])
        self.u.emu_start(0x1019e60c0, stop, count=20000)
        if self.u.reg_read(UC_X86_REG_RIP) != stop:
            raise RuntimeError('Mixer failed to return')
        return list(struct.unpack('<5f', self.u.mem_read(out, 20)))


def main():
    machine = Mixer()
    rng = random.Random(163960)
    failures, count, max_error = [], 0, 0.0
    def vals(n, a=-2, b=2):
        return [f32(rng.uniform(a,b)) for _ in range(n)]
    def rows(width):
        return prepare_rows([(x, vals(width)) for x in [0,80,180,300,450]])
    for i in range(1600):
        p = dict(angles=[vals(2,0,30) for _ in range(3)],
                 tables_1d=[rows(3) for _ in range(3)], tables_2d=[[],[],[]],
                 sensitivity=vals(1)[0], sensitivity_curve=prepare_rows([(x,vals(1)) for x in [0,.6,1,2]]),
                 arcade_curve=prepare_rows([(x,vals(1)) for x in [0,.7,1.3,2]]),
                 cl=vals(2), cd=vals(2), wing_aoa=vals(1)[0])
        for axis in range(3):
            if i % 4:
                p['tables_2d'][axis] = [(f32(x), f32(inv), rows(3))
                                          for x, inv in [(0.4,2.0),(.9,2.5),(1.3,0)]]
                if i % 4 == 2:
                    p['tables_2d'][axis][1] = (f32(.9),f32(2.5),[])
            if i % 11 == 0:
                p['tables_1d'][axis] = []
            elif i % 11 == 1:
                p['tables_1d'][axis] = p['tables_1d'][axis][:1]
        if i % 17 == 0: p['sensitivity_curve'] = []
        if i % 19 == 0: p['arcade_curve'] = []
        commands = vals(3,-1.5,1.5)
        if i % 5 == 0: commands = [f32(x) for x in [-1,0,1]]
        args = (p, commands, [bool(i & (1<<j)) for j in range(3)], vals(1)[0],
                rng.choice([f32(.4),f32(.9),f32(1.3),*vals(2,0,2)]),
                rng.choice([0.,80.,450.,*vals(2,-10,550)]),
                rng.choice([0.,f32(.6),1.,2.,*vals(2,-.1,3)]), bool(i&8))
        actual, expected = machine.call(*args), mix(*args)
        count += 1
        max_error = max(max_error, max(abs(a-b) for a,b in zip(actual,expected)))
        if actual != expected:
            failures.append(dict(case=i, actual=actual, expected=expected))
    report = dict(binary_sha256=machine.sha, complete_function_calls=count,
                  max_absolute_error=max_error, failures=failures,
                  limitations='Prepared synthetic runtime tables. All lookup helpers execute unmodified machine code without hooks. Does not verify raw BLK loading, density conversion, upstream commands or downstream force production.')
    Path('analysis/control-mixer-validation.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k!='failures'},indent=2))
    print('FAILURES',len(failures)); print(json.dumps(failures[:5],indent=2))
    if failures: raise SystemExit(1)


if __name__ == '__main__':
    main()
