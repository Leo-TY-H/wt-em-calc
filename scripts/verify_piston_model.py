"""Unmodified original piston consumers against independent float32 ports."""
import hashlib
import json
import random
import struct
from pathlib import Path
from unicorn import Uc, UC_ARCH_X86, UC_MODE_64, UC_HOOK_CODE
from unicorn.x86_const import *
from macho_scan import MachO
from verify_polar_machine_code import EXPECTED_BINARY_SHA256
from component_assembly import f32, mul
from piston_model import prepare, pressure_at_height, rpm_torque, mixture, compressor, inlet_pressure

DATA = 0x250000000
PROP = DATA
STATE = DATA + 0x1000
OUT = DATA + 0x2000
FM = DATA + 0x10000
STACK = DATA + 0x20000
STOP = DATA + 0x30000


class PistonMachine:
    def __init__(self):
        m = MachO()
        self.sha = hashlib.sha256(m.data).hexdigest()
        if self.sha != EXPECTED_BINARY_SHA256:
            raise ValueError('Binary changed')
        self.u = Uc(UC_ARCH_X86, UC_MODE_64)
        for a, n in [(0x1019f4000, 0x4000), (0x101988000, 0x1000),
                     (0x1071e4000, 0x40000), (0x107d6f000, 0x2000)]:
            self.u.mem_map(a, n)
            self.u.mem_write(a, m.read(a, n))
        self.u.mem_map(DATA, 0x31000)
        self.u.mem_map(0x106e61000, 0x1000)
        self.hooks = {}
        self.u.hook_add(UC_HOOK_CODE, self.powf, begin=0x106e6199f, end=0x106e6199f)

    def powf(self, u, address, size, data):
        import math
        self.hooks['powf'] = self.hooks.get('powf', 0)+1
        values = [struct.unpack('<f', (u.reg_read(UC_X86_REG_XMM0+i) & 0xffffffff).to_bytes(4,'little'))[0] for i in range(2)]
        self.xmm(0, f32(math.pow(*values)))
        sp = u.reg_read(UC_X86_REG_RSP)
        ret = struct.unpack('<Q', u.mem_read(sp,8))[0]
        u.reg_write(UC_X86_REG_RSP,sp+8); u.reg_write(UC_X86_REG_RIP,ret)

    def write(self, a, fmt, *values):
        self.u.mem_write(a, struct.pack('<'+fmt, *values))

    def read(self, a, n=1):
        return list(struct.unpack('<'+'f'*n, self.u.mem_read(a, n*4)))

    def xmm(self, i, value):
        self.u.reg_write(UC_X86_REG_XMM0+i, int.from_bytes(struct.pack('<f', value), 'little'))

    def result(self):
        return struct.unpack('<f', (self.u.reg_read(UC_X86_REG_XMM0) & 0xffffffff).to_bytes(4, 'little'))[0]

    def run(self, address, stack_args=()):
        sp = STACK+0xffd8
        self.write(sp, 'Q', STOP)
        for i, value in enumerate(stack_args):
            self.write(sp+8*(i+1), 'Q', value)
        self.u.reg_write(UC_X86_REG_RSP, sp)
        self.u.emu_start(address, STOP, count=30000)
        if self.u.reg_read(UC_X86_REG_RIP) != STOP:
            raise RuntimeError('Original consumer did not return')
        return self.result()

    def load(self, p):
        self.u.mem_write(DATA, bytes(0x20000))
        self.write(FM+0x8470, 'B', 1)
        self.write(FM+0x3658, 'B', 4)
        self.write(PROP, 'B', 0)
        for off, value in {0x14:p['min_throttle'], 0x18:p['max_throttle'],
                           0x144:p['max_omega'], 0x148:p['afterburner_omega'],
                           0x154:mul(p['max_omega'], 1.2), 0x15c:p['inverse_omega'],
                           0x164:p['compressor_reference'], 0x168:p['torque_base'],
                           0x190:p['mixer_scale'], 0x1b8:p['compressor_sq'],
                           0x1bc:p['max_ata'], 0x1c4:p['boost_ata_ratio'],
                           0x1c8:p['compressor_zero'], 0x1e8:p['throttle_boost'],
                           0x1ec:p['afterburner_boost']}.items():
            self.write(PROP+off, 'f', value)
        self.write(PROP+0x18c, 'I', p['mixer_type'])
        self.write(PROP+0x194, 'II', p['compressor_type'], len(p['stages'])-1)
        self.write(PROP+0x1e4, 'I', p['boost_type'])
        self.write(PROP+0x2b4, 'I', 1 if len(p['stages']) == 2 else 2)
        self.write(PROP+0x1c, 'B', 1)
        self.write(PROP+0x160, 'B', 1)
        self.write(PROP+0x1a0, 'Q', DATA+0x4000)
        self.write(PROP+0x1b0, 'I', len(p['ata']))
        for i, (x, inv, values) in enumerate(p['ata']):
            self.write(DATA+0x4000+12*i, '3f', x, inv, values[0])
        for i, stage in enumerate(p['stages']):
            for off, key in [(0x20,'critical'),(0x40,'power'),(0x60,'boost'),
                             (0x80,'flat'),(0xa0,'flat_power'),(0xc0,'curvature'),
                             (0xe0,'ceiling'),(0x100,'ceiling_factor'),(0x120,'pressure_boost')]:
                self.write(PROP+off+4*i, 'f', stage[key])
        self.write(STATE+0xc, 'B', 7)

    def pressure(self, height):
        self.xmm(0, height)
        return self.run(0x1019880b0)

    def torque(self, p, omega, throttle, torque_multiplier=1., afterburner=False, gear=0):
        self.load(p)
        self.write(STATE+4, 'f', throttle)
        self.write(STATE+0x18, 'f', omega)
        self.write(STATE+0xa4, 'I', gear)
        self.write(STATE+0xac, 'B', afterburner)
        for r, value in [(UC_X86_REG_RDI,FM),(UC_X86_REG_RSI,0),(UC_X86_REG_RDX,PROP),
                         (UC_X86_REG_RCX,STATE),(UC_X86_REG_R8,OUT)]:
            self.u.reg_write(r, value)
        self.xmm(0, torque_multiplier)
        return self.run(0x1019f6c60)

    def mixture(self, p, pressure, command, rich_accumulator=0.):
        self.load(p)
        self.write(STATE+0x9c, 'f', command)
        self.write(OUT+0x18, 'f', rich_accumulator)
        for r, value in [(UC_X86_REG_RDI,FM),(UC_X86_REG_RSI,0),(UC_X86_REG_RDX,PROP),
                         (UC_X86_REG_RCX,STATE),(UC_X86_REG_R8,OUT)]:
            self.u.reg_write(r, value)
        self.xmm(0, pressure)
        value = self.run(0x1019f71f0)
        return dict(multiplier=value, rich_accumulator=self.read(OUT+0x18)[0], requires_stop=False)

    def compressor(self, p, omega, throttle, inlet, dt, *, gear, old_gear=None,
                   regulator=-1., afterburner=False):
        self.load(p)
        self.write(STATE+4, 'f', throttle)
        self.write(STATE+0x18, 'f', omega)
        self.write(STATE+0xa4, 'I', (gear or 0) if old_gear is None else old_gear)
        self.write(STATE+0xac, 'B', afterburner)
        self.write(STATE+0x30, 'f', regulator)
        for r, value in [(UC_X86_REG_RDI,FM),(UC_X86_REG_RSI,DATA+0x3000),
                         (UC_X86_REG_RDX,0),(UC_X86_REG_RCX,0xffffffff if gear is None else gear),
                         (UC_X86_REG_R8,PROP),(UC_X86_REG_R9,STATE)]:
            self.u.reg_write(r, value)
        for i, value in enumerate([dt,1.,inlet,100.]): self.xmm(i,value)
        result = self.run(0x1019f4fd0, (DATA+0x5000,OUT))
        return dict(multiplier=result,gear=struct.unpack('<I',self.u.mem_read(STATE+0xa4,4))[0],
                    regulator=self.read(STATE+0x30)[0],throttle_ratio=self.read(STATE+0x2c)[0],
                    potential_manifold=self.read(OUT+8)[0],manifold=self.read(OUT+0xc)[0])


def main():
    rng = random.Random(19092026)
    machine = PistonMachine()
    counts = {}; failures = []
    def val(a, b): return f32(rng.uniform(a, b))
    def check(stage, actual, expected, **where):
        counts[stage] = counts.get(stage, 0)+1
        if actual != expected:
            failures.append(dict(stage=stage, actual=actual, expected=expected, **where))
            raise AssertionError(str(failures[-1]))
    try:
        for i in range(1000):
            h = val(-1000, 30000)
            check('pressure', machine.pressure(h), pressure_at_height(h), height=h)
        for name in ['yak-3', 'bf-109f-4']:
            fm = json.loads(Path('references/fm-2.59.0.13', name+'.blkx').read_text())
            p = prepare(fm)
            for i in range(2000):
                args = (p, mul(val(.3, 2.2), p['max_omega']), val(0, 1.1), val(.05, 1.5), bool(i%2), i%len(p['stages']))
                check('rpm_torque', machine.torque(*args), rpm_torque(*args), aircraft=name, inputs=args[1:])
                pressure = val(.08, 1.2); command = val(.1, 2.)
                rich = val(0, 100)
                check('mixture', machine.mixture(p, pressure, command, rich),
                      mixture(p, pressure, command, rich_accumulator=rich), aircraft=name)
            for i in range(2000):
                args = (p, mul(val(.6, 1.2),p['max_omega']), val(.05,1.1), val(.1,1.25), f32(1/48))
                opts = dict(gear=None if i%3==0 else i%len(p['stages']),
                            old_gear=(i+1)%len(p['stages']),regulator=-1. if i%4==0 else val(0,1),afterburner=bool(i%2))
                check('compressor', machine.compressor(*args, **opts), compressor(*args, **opts), aircraft=name,inputs=args[1:],options=opts)
            native_history = port_history = dict(gear=0, regulator=-1.)
            for i in range(600):
                h = f32(9000*(i%200)/199)
                args = (p, p['max_omega'], f32(1.1 if name=='bf-109f-4' else 1.),
                        inlet_pressure(h, f32(70+100*(i//200)/2), p['ram_recovery']), f32(1/48))
                options = dict(gear=None, afterburner=name=='bf-109f-4' and i>=200)
                actual = machine.compressor(*args, **options, old_gear=native_history['gear'], regulator=native_history['regulator'])
                expected = compressor(*args, **options, old_gear=port_history['gear'], regulator=port_history['regulator'])
                check('compressor_chained', actual, expected, aircraft=name,step=i)
                native_history, port_history = actual, expected
    except AssertionError as error:
        print(error)
    report = dict(binary_sha256=machine.sha, calls=counts, failures=failures,hooks=machine.hooks,
                  limitations='Complete original pressure, healthy running carburetor-type-1/2 torque, ExactAltitudes type-1/2 compressor and manual mixture consumers. Only compressor powf uses rounded host libm. Prepared properties use a statically traced adapter, not complete native BLK loading. Mixture shutdown cases, low-RPM manifold corrections, nonrunning states and damage are excluded. No complete propeller/aircraft-step accuracy claim.')
    Path('analysis/piston-model-validation.json').write_text(json.dumps(report, indent=2)+'\n')
    print(counts, 'FAILURES', len(failures))
    if failures:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
