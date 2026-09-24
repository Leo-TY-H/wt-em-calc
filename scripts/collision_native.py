"""Read-only native collision resource loader oracle for pinned aces.

Executes 10014d140 on the decompressed GRP collision stream. OS allocation,
stream I/O and material-name resolution are supplied; geometry decoding,
validation and resource-table construction execute original instructions.
"""
import struct
import math
from collections import Counter
from pathlib import Path
from unicorn import UC_HOOK_CODE
from unicorn.x86_const import *
from verify_aircraft_native import AircraftNative, BASE, FRAME, END

HEAP = 0x260000000
STUB = BASE + 0x70000

class CollisionNative(AircraftNative):
    def __init__(self):
        super().__init__()
        self.u.mem_map(HEAP, 0x2000000)
        self.cursor = HEAP
        self.sizes = {}
        self.services = Counter()
        self.stream = b''
        self.pos = 0
        self.resource = self.alloc(0x200)
        self.qword(self.resource + 0x90, self.resource)
        self.allocator = self.alloc(16)
        self.allocator_vtable = self.alloc(0x100)
        self.tls = self.alloc(16)
        self.stream_obj = self.alloc(16)
        self.stream_vtable = self.alloc(0x100)
        self.qword(self.allocator, self.allocator_vtable)
        self.qword(self.tls, self.allocator)
        self.qword(self.stream_obj, self.stream_vtable)
        for off in [0x18, 0x20, 0x28, 0x30, 0x38, 0x40]:
            self.qword(self.allocator_vtable + off, STUB + off)
        for off in [0x18, 0x20, 0x28, 0x30]:
            self.qword(self.stream_vtable + off, STUB + 0x100 + off)
        self.qword(0x107e2d0e8, self.allocator)
        self.qword(0x107e2d0d8, self.allocator)
        self.qword(0x107e2d0f0, self.allocator)
        self.qword(0x107615358, BASE + 0x6e000)
        self.u.mem_write(STUB, b'\xc3' * 0x1000)
        self.u.hook_add(UC_HOOK_CODE, self.service, begin=STUB, end=STUB+0xfff)
        for a in [0x10001de50, 0x101305e40, 0x10001d7e0, 0x10001d800, 0x10001e240, 0x10001ee10,
                  0x106e61351, 0x106e618c7, 0x106e618cd, 0x106e618d3, 0x106e6133f,
                  0x106e618c1, 0x106e61c9f, 0x106e61c75, 0x106e61465, 0x106e61549,
                  0x106e6199f, 0x106e61825, 0x106e613cf, 0x10027ef50, 0x10001a3b0]:
            self.u.hook_add(UC_HOOK_CODE, self.service, begin=a, end=a)

    def alloc(self, n):
        a = self.cursor
        self.cursor += (max(n, 1) + 63) & ~63
        if self.cursor > HEAP + 0x2000000:
            raise RuntimeError('Oracle heap exhausted')
        self.sizes[a] = n
        return a

    def cstr(self, a):
        out = bytearray()
        while True:
            b = self.u.mem_read(a + len(out), 1)[0]
            if not b: return bytes(out).decode()
            out.append(b)

    def ret(self, value=None):
        u = self.u
        if value is not None: u.reg_write(UC_X86_REG_RAX, value)
        sp = u.reg_read(UC_X86_REG_RSP)
        target = struct.unpack('<Q', u.mem_read(sp, 8))[0]
        u.reg_write(UC_X86_REG_RSP, sp + 8)
        u.reg_write(UC_X86_REG_RIP, target)

    def service(self, u, a, size, data):
        self.services[hex(a)] += 1
        rdi, rsi, rdx = [u.reg_read(r) for r in [UC_X86_REG_RDI, UC_X86_REG_RSI, UC_X86_REG_RDX]]
        result = None
        if a in [0x10001de50, 0x101305e40]: result = self.alloc(rdi)
        elif a == 0x10001d800: result = 0
        elif a == 0x10001d7e0:
            result = self.alloc(rsi)
            if rdi: u.mem_write(result, bytes(u.mem_read(rdi, min(rsi, self.sizes[rdi]))))
        elif a == 0x10001e240 or a == STUB + 0x40: pass
        elif a == 0x10001ee10: result = self.tls
        elif a in [STUB + 0x18, STUB + 0x20, STUB + 0x28]: result = self.alloc(rsi)
        elif a == STUB + 0x30: result = 0
        elif a == STUB + 0x38:
            result = self.alloc(rdx)
            if rsi: u.mem_write(result, bytes(u.mem_read(rsi, min(rdx, self.sizes[rsi]))))
        elif a in [STUB + 0x118, STUB + 0x120]:
            chunk = self.stream[self.pos:self.pos+rdx]
            if a == STUB + 0x118 and len(chunk) != rdx: raise ValueError('Stream overrun')
            if chunk: u.mem_write(rsi, chunk)
            self.pos += len(chunk)
            result = len(chunk)
        elif a == STUB + 0x128: result = self.pos
        elif a == STUB + 0x130: self.pos = rsi
        elif a == 0x106e61351: u.mem_write(rdi, bytes(rsi))
        elif a == 0x106e618cd: u.mem_write(rdi, bytes([rsi & 255])*rdx); result = rdi
        elif a == 0x106e618d3:
            pattern = bytes(u.mem_read(rsi, 16))
            u.mem_write(rdi, (pattern*((rdx+15)//16))[:rdx])
        elif a == 0x106e6133f: pass
        elif a in [0x106e618c7, 0x106e618c1]:
            u.mem_write(rdi, bytes(u.mem_read(rsi, rdx))); result = rdi
        elif a == 0x106e61c9f: result = len(self.cstr(rdi))
        elif a == 0x106e61c75:
            x, y = self.cstr(rdi), self.cstr(rsi)
            result = ((x > y) - (x < y)) & 0xffffffff
        elif a == 0x106e61465: self.xmm(0, [math.acos(self.read_xmm(0)[0])])
        elif a == 0x106e61549: self.xmm(0, [math.cos(self.read_xmm(0)[0])])
        elif a == 0x106e6199f: self.xmm(0, [math.pow(self.read_xmm(0)[0], self.read_xmm(1)[0])])
        elif a == 0x106e61825: self.xmm(0, [math.log(self.read_xmm(0)[0])])
        elif a == 0x106e613cf:
            x = struct.unpack('<d', (u.reg_read(UC_X86_REG_XMM0) & ((1<<64)-1)).to_bytes(8,'little'))[0]
            n = struct.unpack('<i', struct.pack('<I', rdi & 0xffffffff))[0]
            negate = n < 0; n = abs(n); value = 1.
            while n:
                if n & 1: value *= x
                n >>= 1
                if n: x *= x
            if negate: value = 1. / value
            u.reg_write(UC_X86_REG_XMM0, int.from_bytes(struct.pack('<d', value),'little'))
        elif a == 0x10027ef50: result = 0
        elif a == 0x10001a3b0: raise RuntimeError('Native loader error: ' + self.cstr(rsi))
        else: raise RuntimeError('Unknown service '+hex(a))
        self.ret(result)

    def run(self, address, args, limit=5000000):
        u = self.u
        for reg, value in zip([UC_X86_REG_RDI, UC_X86_REG_RSI, UC_X86_REG_RDX,
                               UC_X86_REG_RCX, UC_X86_REG_R8, UC_X86_REG_R9], args):
            u.reg_write(reg, value)
        self.qword(FRAME, END); u.reg_write(UC_X86_REG_RSP, FRAME)
        try: u.emu_start(address, END, count=limit)
        except Exception:
            print('Native PC', hex(u.reg_read(UC_X86_REG_RIP)), 'stream', self.pos, 'services', self.services)
            raise
        if u.reg_read(UC_X86_REG_RIP) != END: raise RuntimeError('Native call did not return at '+hex(u.reg_read(UC_X86_REG_RIP)))
        return u.reg_read(UC_X86_REG_RAX)

    def load(self, stream,limit=5000000):
        self.stream = stream; self.pos = 0
        name = self.alloc(16); self.u.mem_write(name, b'collision\0')
        result = self.run(0x10014d140, [self.resource, self.stream_obj, name, 0, 3, 0xffffffff],limit=limit)
        if result & 255 != 1: raise RuntimeError('Collision loader rejected stream')
        dump = self.read(self.resource+0x78, 1, 'Q')[0]
        # 1000f4ac0 reads the active transform array at resource+a0.
        active = self.read(self.resource+0xa0, 1, 'Q')[0]
        authored = dump + self.read(dump+0xc, 1, 'I')[0]
        count = self.read(dump+8, 1, 'I')[0]
        assert bytes(self.u.mem_read(active, count*48)) == bytes(self.u.mem_read(authored, count*48))
        return dump

if __name__ == '__main__':
    m = CollisionNative()
    p = Path('references/collision/bf_109f_4_collision.dump')
    dump = m.load(p.read_bytes())
    print('LOADED', hex(dump), 'consumed', m.pos, 'of', len(m.stream))
    nodeofs, count = m.read(dump+4, 2, 'I')
    names = dump + m.read(dump+0x4c, 1, 'I')[0]
    for i in range(count):
        node = dump + nodeofs + 64*i
        print(i, m.cstr(names+m.read(node+0x3c,1,'I')[0]), m.read(node+0x30,3,'I'))
