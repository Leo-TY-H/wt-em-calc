"""Check control-sensitivity and radiator/spin leaf providers against aces."""
import json,random,struct
from pathlib import Path
from unicorn.x86_const import *
from macho_scan import MachO
from verify_primary_controls import Controls,BASE,END
from component_assembly import f32,mul
from primary_controls import sensitivity_parameters
from runtime_providers import automatic_radiator,spin_engine_factor
from wing_model import quantized_fraction

class Providers(Controls):
    def __init__(self):
        super().__init__();m=MachO()
        for va,size in [(0x101aab000,0x2000),(0x101a17000,0x1000),(0x1019f7000,0x5000)]:
            self.u.mem_map(va,size);self.u.mem_write(va,m.read(va,size))
    def sensitivity(self,values):
        self.reset();self.u.mem_write(BASE+0x55a0,struct.pack('<Q',BASE))
        # Selected jets have zero propeller count, so the first sensitivity triplet applies.
        self.floats(BASE+0x10000,values+[-100.,-100.,-100.]);self.u.reg_write(UC_X86_REG_RDI,BASE+0x10000);self.u.reg_write(UC_X86_REG_RSI,BASE)
        self.call(0x101aabe30)
        return dict(time_constants=self.read3(BASE+0x84fc),linear_rates=self.read3(BASE+0x8508))
    def radiator(self,etype,current,oil,wr,orr,dt,common):
        self.reset();self.u.mem_write(BASE,struct.pack('<Q',BASE+0x10000))
        self.u.mem_write(BASE+0x10000,bytes([etype]));self.u.mem_write(BASE+0x1024e,bytes([common]))
        self.floats(BASE+0x9c,[current,oil]);self.floats(BASE+0x308,[wr,orr])
        self.u.reg_write(UC_X86_REG_RDI,BASE);self.xmm(0,[dt]);self.call(0x1019fb860)
        water=self.read_xmm(0)[0]
        self.u.reg_write(UC_X86_REG_RDI,BASE);self.xmm(0,[dt]);self.call(0x1019fb860 if common else 0x1019fb8b0)
        return [water,self.read_xmm(0)[0]]
    def radiator_setters(self,requested):
        # Selected properties: no manual radiator control, automatic true, common true.
        self.u.mem_write(BASE+0x1024c,b'\x00\x00\x01');self.u.mem_write(BASE+0x10253,b'\x01\x01')
        self.u.reg_write(UC_X86_REG_RDI,BASE);self.xmm(0,[quantized_fraction(requested[0])]);self.call(0x1019fa7d0)
        self.u.reg_write(UC_X86_REG_RDI,BASE);self.xmm(0,[quantized_fraction(requested[1])]);self.call(0x1019fa830)
        return list(struct.unpack('<2f',self.u.mem_read(BASE+0x9c,8)))
    def spin(self,state,health,coefficient):
        self.reset();self.u.mem_write(BASE+0x1c,bytes([state]));self.floats(BASE+0x58,[health]);self.floats(BASE+0xa4,[coefficient])
        self.u.reg_write(UC_X86_REG_RDI,BASE);self.call(0x1019f7780)
        return self.read_xmm(0)[0]

def main():
    m=Providers();rng=random.Random(163907);fails=[];counts=dict(sensitivity=0,automatic_radiator=0,selected_radiator_setters=0,spin_engine_factor=0)
    def check(stage,actual,expected,args):
        counts[stage]+=1
        if actual!=expected:fails.append(dict(stage=stage,args=args,actual=actual,expected=expected))
    def val(a,b):return f32(rng.uniform(a,b))
    for i in range(1000):
        sensitivity=[val(-.2,1.2) for _ in range(3)] if i else [0.,.5,1.]
        check('sensitivity',m.sensitivity(sensitivity),sensitivity_parameters(sensitivity),sensitivity)
        args=[i%8,val(0,1),val(0,1),val(-10,10),val(-10,10),val(.001,.2),bool(i%2)]
        check('automatic_radiator',m.radiator(*args),automatic_radiator(*args),args)
        args[0]=2;args[-1]=True
        requested=m.radiator(*args)
        check('selected_radiator_setters',m.radiator_setters(requested),[0.,0.],args)
        args=[i%256,val(0,1),val(0,2)]
        check('spin_engine_factor',m.spin(*args),spin_engine_factor(*args),args)
    report=dict(binary_sha256=m.sha,counts=counts,cases=sum(counts.values()),failures=fails,
                limitations='Complete leaf functions, including original no-propeller sensitivity selector. Radiator tests chain automatic providers to setters with prepared properties, not an uninterrupted command snapshot update. Selected automatic common-radiator jet path only; manual override/network snapshots remain explicit states.')
    Path('analysis/runtime-providers-validation.json').write_text(json.dumps(report,indent=2))
    print(counts,'FAILURES',len(fails));print(json.dumps(fails[:5],indent=2))
    if fails:raise SystemExit(1)
if __name__=='__main__':main()
