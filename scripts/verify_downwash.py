"""Check the extracted downstream scalar wake stage; geometry supplied explicitly."""
import json
import math
import random
import struct
from pathlib import Path
from unicorn import UC_HOOK_CODE
from unicorn.x86_const import *
from component_assembly import f32, mul, sub
from downwash import scalar_stage
from macho_scan import MachO
from verify_component_stages import Stages
from verify_component_assembly import BASE, FRAME


class Wake(Stages):
    def __init__(self):
        super().__init__()
        m=MachO()
        for va,size in [(0x107d6f000,0x1000),(0x106e61000,0x1000),(0x100233000,0x1000)]:
            self.u.mem_map(va,size);self.u.mem_write(va,m.read(va,size))
        for va in [0x106e615f1,0x100233d00]:
            self.u.hook_add(UC_HOOK_CODE,self.hook,begin=va,end=va)

    def hook(self,u,address,size,data):
        value=self.read_xmm(0)[0]
        if address==0x106e615f1:
            self.xmm(0,[f32(math.exp(value))])
        else:
            self.floats(u.reg_read(UC_X86_REG_RDI),[f32(math.sin(value))])
            self.floats(u.reg_read(UC_X86_REG_RSI),[f32(math.cos(value))])
        sp=u.reg_read(UC_X86_REG_RSP)
        ret=struct.unpack('<Q',u.mem_read(sp,8))[0]
        u.reg_write(UC_X86_REG_RSP,sp+8);u.reg_write(UC_X86_REG_RIP,ret)

    def call(self,x,transverse,inverse_speed2,travel_cap,roll_rate,current_cl,
             cl_history_rate,inverse_halfspan,longitudinal_exponent,amplitude,
             coefficient,inverse_span2):
        self.reset()
        self.u.reg_write(UC_X86_REG_RSP,FRAME-0x2000)
        self.u.reg_write(UC_X86_REG_R15,BASE+0x10000)
        self.u.mem_write(BASE+0x10000,struct.pack('<Q',BASE+0x10010))
        self.u.mem_write(BASE+0x15e0,struct.pack('<d',roll_rate))
        self.floats(BASE+0x81b0,[coefficient]);self.floats(0x107d6fc54,[travel_cap])
        for off,value in [(0x790,inverse_speed2),(0x4f0,cl_history_rate),
                          (0x580,inverse_halfspan),(0x5d0,longitudinal_exponent),
                          (0x530,amplitude),(0x560,inverse_span2)]:
            self.floats(FRAME-off,[value])
        self.u.mem_write(FRAME-0x900,struct.pack('<4I',0xffffffff,0,0,0))
        self.floats(FRAME-0x480,list(transverse)+[0,0])
        self.xmm(3,transverse);self.xmm(0,[x]);self.xmm(14,[x]);self.xmm(13,[current_cl])
        self.xmm(11,[x]);self.floats(FRAME-0x500,[x,0,0,0])
        self.u.emu_start(0x106c615a1,0x106c616d6,count=1000)
        return self.read3(BASE+0x10010)[0]


def main():
    m=Wake();rng=random.Random(21639);failures=[];maximum=0
    for i in range(1000):
        def val(a,b):return f32(rng.uniform(a,b))
        span=val(6,16);x=val(.01,20);speed=val(10,600);dt=val(.005,.1)
        current,previous=val(-2,2),val(-2,2)
        # Density/wing geometry have already supplied amplitude; test its range.
        args=(x,[val(-10,10),val(-10,10)],f32(1/mul(speed,speed)),val(.001,2),
              rng.uniform(-4,4),current,mul(sub(previous,current),f32(1/dt)),
              f32(2/span),mul(-2.5,f32(2/span)),val(-4000,-10),
              [0.,f32(.8),1.][i%3],f32(1/mul(span,span)))
        actual,expected=m.call(*args),scalar_stage(*args)
        maximum=max(maximum,abs(actual-expected))
        if actual!=expected:failures.append(dict(case=i,actual=actual,expected=expected))
    report=dict(binary_sha256=m.sha,slice_executions=1000,max_absolute_error=maximum,
                failures=failures,limitations='Positive downstream branch and finite prepared geometry. Geometry rotation, history writes and amplitude setup are outside the tested slice. exp/sincos imports use host math rounded to float32.')
    Path('analysis/downwash-validation.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k!='failures'},indent=2))
    print('FAILURES',len(failures));print(json.dumps(failures[:5],indent=2))
    if failures:raise SystemExit(1)


if __name__=='__main__':main()
