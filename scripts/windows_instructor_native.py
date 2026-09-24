"""Offline Windows reduced predictor with explicit prepared-property adapters.

Research harness, not a game-state initializer or production boundary model.
The bundled Mach-O fixture builder supplies documented source FM fields. The
Windows predictor, polar consumers and control consumers execute original code.
Sweep selection and flap packing are substituted property providers. Releasing
their fixture-owned arrays is a no-op; no arithmetic consumer is substituted.
"""
from unicorn import UC_HOOK_CODE
from unicorn.x86_const import *
from instructor_native import InstructorNative, BASE, INPUT, OUTPUT, STACK, STOP, HEAP
from pe_scan import PE
from verify_windows_instructor import SHA256
from polar_runtime import pack_runtime, flap_polar
from wing_sweep import select
from verify_wing_sweep import SweepMachine

TLS = 0x230000000


class WindowsInstructorNative(InstructorNative):
    def __init__(self, binary):
        super().__init__()
        self.windows = PE(binary)
        if self.windows.sha256 != SHA256:
            raise ValueError('Unreviewed Windows executable hash')
        u = self.u
        u.mem_map(self.windows.base, (self.windows.image_size+0xfff) & ~0xfff)
        for section in self.windows.sections:
            if section['filesize']:
                u.mem_write(section['va'], self.windows.read(section['va'], section['filesize']))
        u.mem_map(TLS, 0x10000)
        u.reg_write(UC_X86_REG_GS_BASE, TLS)
        self.qword(TLS+0x58, TLS+0x1000)
        index = self.read(0x1479c3424, 1, 'I')[0]
        if index >= 256:
            raise ValueError('Unexpected TLS index')
        self.qword(TLS+0x1000+index*8, TLS+0x2000)
        u.mem_write(TLS+0x2008, b'\1')
        self.qword(TLS+0x4000, TLS+0x5000)
        self.qword(TLS+0x5040, TLS+0x6000)
        u.mem_write(TLS+0x6000, b'\xc3')
        self.windows_calls = []
        for va in (0x142fdae80, 0x143140ce0, TLS+0x6000):
            u.hook_add(UC_HOOK_CODE, self.windows_provider, begin=va, end=va)

    def windows_provider(self, u, address, size, data):
        self.windows_calls.append(hex(address))
        if address == TLS+0x6000:
            self.return_call()
            return
        out = u.reg_read(UC_X86_REG_R8)
        wing = select(self.wings, self.read(self.windows_input+0x74, 1)[0])
        if address == 0x142fdae80:
            SweepMachine.pack(self, wing, HEAP+0x10000)
            u.mem_write(out, bytes(u.mem_read(HEAP+0x10000, 0xd8)))
            # Provider owns this fixture storage; there is no native allocation.
            for off in (0x98, 0xb0, 0xc8):
                self.qword(out+off, TLS+0x4000)
        elif address == 0x143140ce0:
            u.mem_write(out, pack_runtime(flap_polar(wing['polars'], self.read_xmm(1)[0]), out))
        else:
            raise RuntimeError('Unexpected Windows adapter')
        self.return_call()

    def predict(self, packed, history=(0., 0., False)):
        if len(packed) != 0xa0:
            raise ValueError('Expected the reviewed 0xa0-byte input record')
        u = self.u
        self.windows_input = INPUT
        self.windows_calls.clear()
        u.mem_write(INPUT, bytes(packed))
        hist = OUTPUT+0x100
        self.floats(hist, history[:2])
        u.mem_write(hist+8, bytes([bool(history[2])]))
        u.mem_write(OUTPUT, bytes(13*4))
        sp = STACK-8
        self.qword(sp, STOP)
        u.reg_write(UC_X86_REG_RSP, sp)
        for reg, value in zip((UC_X86_REG_RCX, UC_X86_REG_RDX, UC_X86_REG_R8, UC_X86_REG_R9),
                              (BASE, INPUT, OUTPUT, hist)):
            u.reg_write(reg, value)
        for i in range(16):
            self.xmm(i, [0.])
        try:
            u.emu_start(0x1430b06c0, STOP, count=5000000)
        except Exception as error:
            raise RuntimeError('Windows predictor stopped at '+hex(u.reg_read(UC_X86_REG_RIP))) from error
        if u.reg_read(UC_X86_REG_RIP) != STOP:
            raise RuntimeError('Windows predictor instruction budget exceeded')
        return dict(output=self.read(OUTPUT, 13), success=bool(u.reg_read(UC_X86_REG_RAX)&255),
                    history=self.read(hist, 2)+[bool(u.mem_read(hist+8, 1)[0])])
