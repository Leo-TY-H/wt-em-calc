"""Native four-station blade integration, including original CL/CD and Mach.

Prepared static geometry is shared; this does not execute a complete loader or
the inflow/governor/transmission/aircraft caller. Only sinf/atan2f use host libm.
"""
import json
import math
import random
import struct
from pathlib import Path
from unicorn import UC_HOOK_CODE
from unicorn.x86_const import *
from macho_scan import MachO
from polar_runtime import PolarMachine, pack_runtime, DATA, STACK, STOP
from component_assembly import f32
from propeller_model import prepare, blade_forces


class BladeMachine(PolarMachine):
    def __init__(self):
        super().__init__()
        m = MachO()
        self.u.mem_map(0x101a07000,0x1000)
        self.u.mem_write(0x101a07000,m.read(0x101a07000,0x1000))
        self.hooks = {}
        for address in [0x106e614b9,0x106e61c1b]:
            self.u.hook_add(UC_HOOK_CODE,self.libm,begin=address,end=address)

    def libm(self,u,address,size,data):
        values = [struct.unpack('<f',(u.reg_read(UC_X86_REG_XMM0+i)&0xffffffff).to_bytes(4,'little'))[0] for i in range(2)]
        name = 'atan2f' if address == 0x106e614b9 else 'sinf'
        self.hooks[name] = self.hooks.get(name,0)+1
        value = math.atan2(*values) if name == 'atan2f' else math.sin(values[0])
        u.reg_write(UC_X86_REG_XMM0,int.from_bytes(struct.pack('<f',value),'little'))
        sp=u.reg_read(UC_X86_REG_RSP);ret=struct.unpack('<Q',u.mem_read(sp,8))[0]
        u.reg_write(UC_X86_REG_RSP,sp+8);u.reg_write(UC_X86_REG_RIP,ret)

    def blade(self,p,args):
        u=self.u
        u.mem_write(DATA,bytes(0x1000));u.mem_write(DATA,pack_runtime(p['polar'],DATA))
        u.mem_write(DATA+0x1e0,struct.pack('<fI',p['radius'],p['blades']))
        u.mem_write(DATA+0x1e8,struct.pack('<4f',*p['twist']))
        u.mem_write(DATA+0x1fc,struct.pack('<4f',*p['width']))
        for reg,value in [(UC_X86_REG_RDI,DATA),(UC_X86_REG_RSI,0),
                          (UC_X86_REG_RDX,DATA+0x800),(UC_X86_REG_RCX,DATA+0x804)]:
            u.reg_write(reg,value)
        for i,x in enumerate(args[:8]):
            u.reg_write(UC_X86_REG_XMM0+i,int.from_bytes(struct.pack('<f',x),'little'))
        sp=STACK+0xffd8
        u.mem_write(sp,struct.pack('<Qf',STOP,args[8]));u.reg_write(UC_X86_REG_RSP,sp)
        u.emu_start(0x101a07790,STOP,count=30000)
        if u.reg_read(UC_X86_REG_RIP)!=STOP:raise RuntimeError('Blade kernel did not return')
        return dict(zip(['thrust','torque'],struct.unpack('<2f',u.mem_read(DATA+0x800,8))))


def main():
    machine=BladeMachine();rng=random.Random(109003);failures=[];count=0;max_error=0.;exact=0
    for name in ['yak-3','bf-109f-4']:
        p=prepare(json.loads(Path('references/fm-2.59.0.13/'+name+'.blkx').read_text()))
        cases=[[w,f32(math.radians(b)),v,0.,0.,0.,1.225,340.,0.]
               for w in [0.,1e-6,50.,160.,210.,-100.] for b in [5.,20.,45.,75.] for v in [0.,50.,180.,-30.]]
        cases += [[rng.uniform(0.,300.),rng.uniform(-.2,1.6),rng.uniform(-70,250),
                   rng.uniform(-30,30),rng.uniform(0,60),rng.uniform(-10,10),
                   rng.uniform(.2,1.3),rng.uniform(280,350),rng.uniform(-5,5)] for _ in range(1000)]
        for args in cases:
            args=list(map(f32,args));expected=blade_forces(p,*args);actual=machine.blade(p,args);count+=1
            errors=[abs(expected[k]-actual[k]) for k in actual];max_error=max(max_error,*errors)
            exact+=expected==actual
            if not all(math.isclose(expected[k],actual[k],rel_tol=2e-6,abs_tol=.001) for k in actual):
                failures.append(dict(aircraft=name,args=args,expected=expected,actual=actual))
    report=dict(status='FAIL' if failures else 'PASS',binary_sha256=machine.sha,
                calls=count,exact_returns=exact,max_absolute_error=max_error,hooks=machine.hooks,
                scope=__doc__,failures=failures)
    Path('analysis/propeller-blade-validation.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k!='failures'},indent=2));print('Failures',len(failures));print(json.dumps(failures[:2],indent=2))
    if failures:raise SystemExit(1)


if __name__=='__main__':main()
