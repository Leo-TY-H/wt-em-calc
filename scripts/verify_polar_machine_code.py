"""Compare scalar reconstruction to isolated local x86 machine-code kernels.

Unicorn executes only copied kernel bytes, never launches/attaches to the game.
sin/sincos imports are hooked with host math functions rounded to float32.
This validates the kernel, not the aircraft caller, Mach loader or libm itself.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
from unicorn import Uc, UC_ARCH_X86, UC_MODE_64, UC_HOOK_CODE
from unicorn.x86_const import *
from macho_scan import MachO, DEFAULT_BINARY
from polar_model import make_polar, pack_polar, round_polar, calc_cl, calc_cd, calc_c

DATA=0x200000000;STACK=0x200100000;STOP=0x200200000
EXPECTED_BINARY_SHA256='820fee4a55601ffa459e2635da4ce1436cfc7adcc703a7c799380c95d136ebd5'


def f32(x):return struct.unpack('<f',struct.pack('<f',x))[0]
def fbits(x):return struct.unpack('<I',struct.pack('<f',x))[0]
def from_bits(x):return struct.unpack('<f',struct.pack('<I',x&0xffffffff))[0]


class Kernel:
    def __init__(self,macho):
        self.u=Uc(UC_ARCH_X86,UC_MODE_64)
        for address,size in [(0x10198c000,0x1000),(0x1071e4000,0x30000),
                             (0x106e61000,0x1000),(0x100233000,0x1000)]:
            self.u.mem_map(address,size);self.u.mem_write(address,macho.read(address,size))
        for address,size in [(DATA,0x1000),(STACK,0x10000),(STOP,0x1000)]:self.u.mem_map(address,size)
        self.u.hook_add(UC_HOOK_CODE,self.hook,begin=0x106e61c1b,end=0x106e61c1b)
        self.u.hook_add(UC_HOOK_CODE,self.hook,begin=0x100233d00,end=0x100233d00)

    def hook(self,u,address,size,data):
        angle=from_bits(u.reg_read(UC_X86_REG_XMM0))
        if address==0x106e61c1b:
            u.reg_write(UC_X86_REG_XMM0,fbits(math.sin(angle)))
        else:
            u.mem_write(u.reg_read(UC_X86_REG_RDI),struct.pack('<f',math.sin(angle)))
            u.mem_write(u.reg_read(UC_X86_REG_RSI),struct.pack('<f',math.cos(angle)))
        sp=u.reg_read(UC_X86_REG_RSP);ret=struct.unpack('<Q',u.mem_read(sp,8))[0]
        u.reg_write(UC_X86_REG_RSP,sp+8);u.reg_write(UC_X86_REG_RIP,ret)

    def call(self,address,p,args):
        u=self.u;sp=STACK+0xfff8
        u.mem_write(DATA,pack_polar(p));u.mem_write(sp,struct.pack('<Q',STOP))
        u.reg_write(UC_X86_REG_RSP,sp);u.reg_write(UC_X86_REG_RBP,0)
        u.reg_write(UC_X86_REG_RDI,DATA)
        for i,x in enumerate(args):u.reg_write(UC_X86_REG_XMM0+i,fbits(x))
        u.emu_start(address,STOP,count=10000)
        if u.reg_read(UC_X86_REG_RIP)!=STOP:raise RuntimeError('Kernel did not return')
        value=u.reg_read(UC_X86_REG_XMM0)
        return [from_bits(value),from_bits(value>>32)] if address==0x10198c320 else [from_bits(value)]


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--binary',default=DEFAULT_BINARY);args=parser.parse_args()
    m=MachO(args.binary)
    binary_hash=hashlib.sha256(m.data).hexdigest()
    if binary_hash != EXPECTED_BINARY_SHA256:
        parser.error('Executable differs from the analyzed binary. Remap code/constants and verify the ABI before using these addresses.')
    kernel=Kernel(m);failures=[];count=0;max_error=0;configs=0
    for name in ['f_16a_block_15_adf','saab_jas39c']:
        fm=json.loads(Path(f'references/fm-2.59.0.13/{name}.blkx').read_text())
        for comp in ['WingPlane','HorStabPlane','VerStabPlane','FuselagePlane']:
            plane=fm['Aerodynamics'][comp]
            for key,props in plane.items():
                if not (key=='Polar' or key.startswith('FlapsPolar')):continue
                areas=plane['Areas'];area=sum(areas[k] for k in ['LeftIn','LeftMid','LeftOut','RightIn','RightMid','RightOut']) if comp=='WingPlane' else sum(areas.values())
                for mach in [0,0.6,0.9,1.0,1.2,1.8]:
                    p=round_polar(make_polar(props,plane['Span'],area,mach));configs+=1
                    angles=sorted(set([-180,-150,-90,-45,-20,-5,0,5,15,25,45,90,150,180]+[f32(p[k]) for k in ['aoaLineL','aoaLineH','aoaCritL','aoaCritH']]))
                    for a in angles:
                        trials=[(0x10198c4d0,[a],[calc_cl(p,a)]),(0x10198c440,[a],[calc_cd(p,a)])]
                        # Negative multiplier detects incorrect abs/sign recovery.
                        for cd_mult in [1.0,-0.8]:
                            inputs=[a,f32(a-1),f32(0.03),f32(cd_mult)]
                            trials.append((0x10198c320,inputs,list(calc_c(p,*inputs))))
                        for address,inputs,expected in trials:
                            actual=kernel.call(address,p,inputs);count+=1
                            error=max(abs(x-y) for x,y in zip(actual,expected));max_error=max(max_error,error)
                            if not all(math.isclose(x,y,rel_tol=3e-5,abs_tol=3e-6) for x,y in zip(actual,expected)):
                                failures.append(dict(aircraft=name,component=comp,polar=key,mach=mach,aoa=a,address=hex(address),expected=expected,actual=actual))
    report=dict(binary_sha256=binary_hash,configurations=configs,kernel_calls=count,max_absolute_error=max_error,failures=failures,limitations='Tests use prepared synthetic Polares inputs. Mach preparation, aircraft area selection, full component flow and summation are NOT validated by this test. sin/sincos use host math rounded to float32.')
    Path('analysis/kernel-validation.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k!='failures'},indent=2));print('FAILURES',len(failures));print(json.dumps(failures[:3],indent=2))
    if failures:raise SystemExit(1)


if __name__=='__main__':main()
