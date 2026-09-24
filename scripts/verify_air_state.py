"""Run the complete air-state producer with atmosphere helpers unchanged."""
import hashlib,json,math,random,struct
from pathlib import Path
from unicorn import Uc,UC_ARCH_X86,UC_MODE_64,UC_HOOK_CODE
from unicorn.x86_const import *
from macho_scan import MachO
from component_assembly import f32
from air_state import cache,world_to_body_air
from verify_polar_machine_code import EXPECTED_BINARY_SHA256


def main():
    m=MachO();sha=hashlib.sha256(m.data).hexdigest()
    if sha!=EXPECTED_BINARY_SHA256:raise ValueError('Binary changed')
    u=Uc(UC_ARCH_X86,UC_MODE_64)
    for a,n in [(0x101a33000,0x1000),(0x101988000,0x1000),(0x107d6f000,0x1000),(0x1071e4000,0x40000),(0x106e61000,0x1000)]:u.mem_map(a,n);u.mem_write(a,m.read(a,n))
    base=0x240000000;u.mem_map(base,0x20000);stop=base+0x1f000
    def atan_hook(u,a,size,data):
        y,x=[struct.unpack('<d',int(u.reg_read(r)).to_bytes(16,'little')[:8])[0] for r in [UC_X86_REG_XMM0,UC_X86_REG_XMM1]]
        u.reg_write(UC_X86_REG_XMM0,int.from_bytes(struct.pack('<d',math.atan2(y,x)),'little'))
        sp=u.reg_read(UC_X86_REG_RSP);u.reg_write(UC_X86_REG_RIP,struct.unpack('<Q',u.mem_read(sp,8))[0]);u.reg_write(UC_X86_REG_RSP,sp+8)
    u.hook_add(UC_HOOK_CODE,atan_hook,begin=0x106e614b3,end=0x106e614b3)
    rng=random.Random(163384);failures=[]
    for i in range(1000):
        q=[rng.uniform(-1,1) for _ in range(4)];norm=math.sqrt(sum(x*x for x in q));q=[f32(x/norm) for x in q]
        velocity=[rng.uniform(-700,700) for _ in range(3)];wind=[rng.uniform(-15,15) for _ in range(3)];additional=[rng.uniform(-5,5) for _ in range(3)];height=rng.uniform(-500,25000)
        rigid=base+0x1000;props=base+0x2000
        u.mem_write(base,struct.pack('<3d',*wind));u.mem_write(rigid+8,struct.pack('<d',height));u.mem_write(rigid+0x18,struct.pack('<4f',*q));u.mem_write(rigid+0x58,struct.pack('<3d',*velocity));u.mem_write(rigid+0xd8,struct.pack('<3d',*additional))
        u.reg_write(UC_X86_REG_RDI,base);u.reg_write(UC_X86_REG_RSI,rigid);u.reg_write(UC_X86_REG_RDX,props)
        sp=base+0x1eff8;u.mem_write(sp,struct.pack('<Q',stop));u.reg_write(UC_X86_REG_RSP,sp);u.emu_start(0x101a33880,stop,count=1000)
        actual_v=list(struct.unpack('<3d',u.mem_read(rigid+0xc0,24)));expected_v=world_to_body_air(q,velocity,wind,additional)
        actual=list(struct.unpack('<6f',u.mem_read(props+0x31c,24)));c=cache(expected_v,height);expected=[c[k] for k in ['alpha','beta','tas','speed_squared','mach','ias_u']]
        if actual_v!=expected_v or actual!=expected:failures.append(dict(actual_v=actual_v,expected_v=expected_v,actual=actual,expected=expected))
    report=dict(binary_sha256=sha,complete_air_state_calls=1000,failures=failures,
                limitations='Original producer and density/sound helpers execute unchanged; atan2 double import uses host libm. Default atmosphere globals, randomized attitudes/world winds/heights.')
    Path('analysis/air-state-validation.json').write_text(json.dumps(report,indent=2));print('FAILURES',len(failures));print(json.dumps(failures[:2],indent=2))
    if failures:raise SystemExit(1)


if __name__=='__main__':main()
