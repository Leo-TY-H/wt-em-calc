"""Differential tests against isolated Instructor spans in Windows aces 2.59.0.31.

Uses offline Unicorn memory, never runs/attaches to the installed application.
These tests establish local stage parity, not a balanced-turn boundary.
"""
import argparse
import json
from pathlib import Path
import random
import struct

from unicorn import Uc, UC_ARCH_X86, UC_MODE_64
from unicorn.x86_const import (UC_X86_REG_RBP, UC_X86_REG_RSP, UC_X86_REG_RAX,
    UC_X86_REG_RIP, UC_X86_REG_XMM0)
from component_assembly import f32
from instructor_protection import protected_pitch_command
from pe_scan import PE

SHA256 = 'eaee1800caef7a2012cbe028d4830d0ffc529d282c9f7f57a3f84b9bc2973048'
ARENA = 0x200000000
OBJ, FM, STACK = ARENA, ARENA+0x1000, ARENA+0x20000


class WindowsInstructorSlices:
    def __init__(self, binary):
        self.binary = PE(binary)
        if self.binary.sha256 != SHA256:
            raise ValueError('Unrecognized executable SHA-256; rediscover and review addresses first')
        self.u = Uc(UC_ARCH_X86, UC_MODE_64)
        self.u.mem_map(self.binary.base, (self.binary.image_size+0xfff) & ~0xfff)
        for section in self.binary.sections:
            if section['filesize']:
                self.u.mem_write(section['va'], self.binary.read(section['va'], section['filesize']))
        self.u.mem_map(ARENA, 0x40000)

    def floats(self, address, values):
        self.u.mem_write(address, struct.pack('<'+'f'*len(values), *values))

    def read(self, address, count=1):
        return list(struct.unpack('<'+'f'*count, self.u.mem_read(address, count*4)))

    def xmm(self, index, value):
        self.u.reg_write(UC_X86_REG_XMM0+index, struct.unpack('<I', struct.pack('<f', value))[0])

    def reset(self):
        self.u.mem_write(ARENA, bytes(0x40000))
        self.u.mem_write(OBJ, struct.pack('<Q', FM))
        self.u.reg_write(UC_X86_REG_RBP, OBJ)
        self.u.reg_write(UC_X86_REG_RSP, STACK)
        for index in range(16):
            self.xmm(index, 0.)

    def run(self, start, end):
        self.u.emu_start(start, end, count=1000)
        if self.u.reg_read(UC_X86_REG_RIP) != end:
            raise RuntimeError('Instruction span did not reach its reviewed endpoint')

    def pitch_clamp(self, inputs):
        self.reset()
        self.floats(FM+0x87f8, [inputs['trim']])
        self.floats(FM+0x8518, [inputs['requested']])
        self.floats(OBJ+0x150, [inputs['authority_factor']])
        self.xmm(6, inputs['predictor_commands'][0])
        self.floats(STACK+0x2b0, [inputs['predictor_commands'][1]])
        self.xmm(15, inputs['authority_inverse'][0])
        self.floats(STACK+0xd4, [inputs['authority_inverse'][1]])
        self.floats(STACK+0x80, [inputs['predicted_adjusted'][0]])
        self.floats(STACK+0xd0, [inputs['predicted_adjusted'][1]])
        self.floats(STACK+0xc8, [inputs['adjustment_offsets'][0]])
        self.floats(STACK+0xcc, [inputs['adjustment_offsets'][1]])
        self.xmm(8, inputs['critical_high'])
        self.xmm(11, inputs['dt'])
        self.floats(STACK+0xc4, [inputs['recovery_reference']])
        self.run(0x14309635b, 0x143096560)
        scalar = lambda index: struct.unpack('<f', struct.pack('<I', self.u.reg_read(UC_X86_REG_XMM0+index) & 0xffffffff))[0]
        return dict(command_bounds=[scalar(15), scalar(13)],
            command=self.read(FM+0x8518)[0], authority_factor=self.read(OBJ+0x150)[0])

    def autotrim_writeback(self, success, requested, cached, actual, output):
        self.reset()
        self.floats(FM+0x87f4, requested)
        self.floats(FM+0x3a24, cached)
        self.floats(FM+0xa290, actual)
        self.floats(STACK+0x2e0, output)
        self.u.reg_write(UC_X86_REG_RAX, int(success))
        self.run(0x143093c49, 0x143093c8f)
        return dict(requested=self.read(FM+0x87f4, 3), cached=self.read(FM+0x3a24, 3),
                    actual=self.read(FM+0xa290, 3))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--out', type=Path, default=Path('analysis/windows-instructor/native-stage-validation.json'))
    args = parser.parse_args()
    native = WindowsInstructorSlices(args.binary)
    rng = random.Random(20260924)
    failures = []
    counts = dict(pitch_clamp=0, autotrim_success=0, autotrim_failure=0)
    rates = native.read(0x1471b2c28, 2)
    cases = []
    for i in range(2000):
        inputs = dict(predictor_commands=[f32(rng.uniform(-4, 4)) for _ in range(2)],
            trim=f32([-1., 0., 1., rng.uniform(-1, 1)][i % 4]),
            authority_inverse=[f32(-rng.uniform(1, 50)), f32(rng.uniform(1, 50))],
            authority_factor=f32(rng.uniform(.2, 1)), requested=f32([-1., 1., rng.uniform(-1, 1)][i % 3]),
            predicted_adjusted=[f32(rng.uniform(-50, 50)) for _ in range(2)],
            adjustment_offsets=[f32(rng.uniform(-10, 10)) for _ in range(2)],
            critical_high=f32(rng.uniform(10, 50)), recovery_reference=f32(rng.uniform(10, 50)),
            dt=f32([1/30, 1/48, 1/60, 1/120][i % 4]), adaptation_rates=rates)
        actual = native.pitch_clamp(inputs)
        expected = protected_pitch_command(**inputs)
        counts['pitch_clamp'] += 1
        if actual != expected:
            failures.append(dict(stage='pitch_clamp', index=i, inputs=inputs, actual=actual, expected=expected))
        if i < 4:
            cases.append(dict(inputs=inputs, result=actual))
    for i in range(400):
        vector = lambda: [f32(rng.uniform(-1, 1)) for _ in range(3)]
        requested, cached, actual_trim, output = vector(), vector(), vector(), vector()
        success = bool(i % 2)
        actual = native.autotrim_writeback(success, requested, cached, actual_trim, output)
        expected = dict(requested=list(requested), cached=list(cached), actual=actual_trim)
        if success:
            expected['requested'][:2] = [output[2], output[1]]
            expected['cached'][:2] = [output[2], output[1]]
        counts['autotrim_success' if success else 'autotrim_failure'] += 1
        if actual != expected:
            failures.append(dict(stage='autotrim_writeback', index=i, actual=actual, expected=expected))
    report = dict(binary=str(args.binary.resolve()), sha256=native.binary.sha256, counts=counts,
        failure_count=len(failures), failures=failures[:12], examples=cases,
        pitch_clamp_span=['0x14309635b', '0x143096560'],
        autotrim_writeback_span=['0x143093c49', '0x143093c8f'],
        scope='Prescribed finite source fields; original Windows pitch clamp and conditional auto-trim writeback only. No substituted calls in these spans. Does not execute the predictors, owner, aircraft trajectory or prove a boundary.')
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(dict(counts=counts, failure_count=len(failures), report=str(args.out))))
    if failures:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
