"""Differential execution of authority, trim actuator and delivered commands.

Runs original functions. Only imported powf/expf and memset_pattern16 are
replaced; both independent and native sides use the same rounded host libm.
"""
import copy,json,math,random,struct
from pathlib import Path
from unicorn import UC_HOOK_CODE
from unicorn.x86_const import *
from macho_scan import MachO
from verify_component_stages import Stages
from verify_component_assembly import BASE,FRAME
from component_assembly import f32,mul
from primary_controls import selected_properties,authority_ranges,actuator_step,delivered_commands

OUT=BASE+0x11000
END=BASE+0x12000

class Controls(Stages):
    def __init__(self):
        super().__init__();m=MachO()
        for va,size in [(0x101a4a000,0x5000),(0x106e61000,0x1000),(0x107d6f000,0x1000)]:
            self.u.mem_map(va,size);self.u.mem_write(va,m.read(va,size))
        for va in [0x106e615f1,0x106e6199f,0x106e618d3]:
            self.u.hook_add(UC_HOOK_CODE,self.import_hook,begin=va,end=va)
    def import_hook(self,u,address,size,data):
        x=self.read_xmm(0)[0]
        if address==0x106e615f1:self.xmm(0,[math.exp(x)])
        elif address==0x106e6199f:self.xmm(0,[math.pow(x,self.read_xmm(1)[0])])
        else:
            n=u.reg_read(UC_X86_REG_RDX);pattern=bytes(u.mem_read(u.reg_read(UC_X86_REG_RSI),16))
            u.mem_write(u.reg_read(UC_X86_REG_RDI),(pattern*((n+15)//16))[:n])
        sp=u.reg_read(UC_X86_REG_RSP);ret=struct.unpack('<Q',u.mem_read(sp,8))[0]
        u.reg_write(UC_X86_REG_RSP,sp+8);u.reg_write(UC_X86_REG_RIP,ret)
    def call(self,address):
        self.u.reg_write(UC_X86_REG_RSP,FRAME-8);self.u.mem_write(FRAME-8,struct.pack('<Q',END))
        self.u.emu_start(address,END,count=10000)
        if self.u.reg_read(UC_X86_REG_RIP)!=END:raise RuntimeError('Did not return')
    def setup(self,p,enabled,scale):
        self.reset();self.u.reg_write(UC_X86_REG_RDI,BASE)
        e=p['effective_speed'];pw=p['power'];mn=p['minimum']
        self.floats(BASE+0x7c18,[e[0][0],e[2][0],*e[1]])
        self.floats(BASE+0x7c28,[pw[0][0],pw[2][0],*pw[1]])
        self.floats(BASE+0x7c38,[mn[0],mn[2],mn[1]])
        self.floats(BASE+0x7c48,p['negative']);self.floats(BASE+0x7c0c,p['max_rate'])
        self.floats(BASE+0x8058,[p['trim_rate'][i] for i in [0,2,1]])
        self.u.mem_write(BASE+0x7c44,bytes([p['strong']]))
        self.u.mem_write(BASE+0x7c54,bytes([p['invert_elevator']]))
        self.u.mem_write(BASE+0x8520,bytes(enabled));self.floats(BASE+0x16a8,[scale])
        self.u.mem_write(BASE+0x8470,b'\x01');self.u.mem_write(BASE+0x7faa,b'\x01\x01\x01')
    def ranges(self,p,speed,enabled,scale,full_loss,asymmetric,elevator,dt,times):
        self.setup(p,enabled,scale);self.floats(BASE+0x39f8,[elevator])
        self.floats(BASE+0x84fc,times)
        self.u.mem_write(0x107d6fbf5,bytes([full_loss]));self.u.mem_write(0x107d6fc2d,bytes([asymmetric]))
        self.u.reg_write(UC_X86_REG_RDX,OUT);self.xmm(0,[dt]);self.xmm(1,[speed])
        self.call(0x101a4acd0)
        data=list(struct.unpack('<10f',self.u.mem_read(OUT,40)))
        return dict(factors=data[:4],ranges=[data[4:6],data[8:10],data[6:8]])
    def actuator(self,p,sticks,trim_requested,trim_actual,old,ranges,dt,times,rates,enabled,scale):
        self.setup(p,enabled,scale)
        self.floats(BASE+0x8514,sticks);self.floats(BASE+0x87f4,trim_requested);self.floats(BASE+0xa290,trim_actual)
        for off,x in zip([0x39f4,0x3a1c,0x3a20],old):self.floats(BASE+off,[x])
        self.floats(BASE+0x8508,rates)
        self.floats(OUT,[f32(math.exp(f32(-dt/t))) for t in times]+[f32(math.exp(mul(-5.,dt)))]+ranges[0]+ranges[2]+ranges[1])
        self.u.reg_write(UC_X86_REG_RSI,OUT);self.xmm(0,[dt]);self.call(0x101a4b230)
        return dict(trim=self.read3(BASE+0xa290),state=[struct.unpack('<f',self.u.mem_read(BASE+off,4))[0] for off in [0x39f4,0x3a1c,0x3a20]],commands=self.read3(BASE+0x39f4))
    def delivery(self,p,desired,previous,ranges,dt):
        self.setup(p,[True]*3,1.);self.floats(BASE+0x2b14,desired);self.floats(BASE+0x1694,previous)
        self.floats(OUT,ranges[0]+ranges[2]+ranges[1]);self.u.reg_write(UC_X86_REG_RSI,OUT)
        self.xmm(0,[dt]);self.call(0x101a4e450)
        return self.read3(BASE+0x1694)

def main():
    m=Controls();rng=random.Random(163906);failures=[];counts=dict(authority_wrapper=0,trim_actuator=0,delivery=0)
    def uniform(lo,hi):return f32(rng.uniform(lo,hi))
    def vec(lo,hi):return [uniform(lo,hi) for _ in range(3)]
    def check(stage,actual,expected,case):
        counts[stage]+=1
        if actual!=expected:failures.append(dict(stage=stage,case=case,actual=actual,expected=expected))
    for name in ['f_16a_block_15_adf','saab_jas39c']:
        fm=json.loads(Path('references/fm-2.59.0.13/'+name+'.blkx').read_text());base=selected_properties(fm)
        for j in range(1000):
            p=copy.deepcopy(base)
            if j%2:
                p.update(negative=vec(.2,2.),strong=bool(j%3),invert_elevator=bool(j%5))
                p['effective_speed'][1]=[uniform(50,300),uniform(50,300)]
                p['power'][1]=[uniform(.5,4),uniform(.5,4)]
            enabled=[bool(rng.randrange(2)) for _ in range(3)] if j%4 else [True]*3
            speed=uniform(-100,700);scale=uniform(.3,1.);dt=uniform(.001,.2);times=vec(.01,.5)
            full=bool(j%3);asym=bool(j%4);elevator=uniform(-1,1)
            args=(p,speed,enabled,scale,full,asym,elevator)
            ranges=authority_ranges(*args)
            actual=m.ranges(*args,dt,times)
            expected=dict(ranges=ranges,factors=[f32(math.exp(f32(-dt/t))) for t in times]+[f32(math.exp(mul(-5.,dt)))])
            check('authority_wrapper',actual,expected,[name,j])
            sticks=vec(-1.5,1.5);tr=vec(-1,1);ta=vec(-1,1);old=vec(-1.5,1.5);rates=vec(0,5)
            # Independently exercise the actuator with nonsymmetric prepared limits.
            if j%2:ranges=[[-uniform(.1,1),uniform(.1,1)] for _ in range(3)]
            args=(p,sticks,tr,ta,old,ranges,dt,times,rates,enabled,scale)
            result=actuator_step(*args);expected={k:result[k] for k in ['trim','state','commands']}
            check('trim_actuator',m.actuator(*args),expected,[name,j])
            args=(p,result['commands'],old,ranges,dt)
            check('delivery',m.delivery(*args),delivered_commands(*args),[name,j])
    report=dict(binary_sha256=m.sha,counts=counts,cases=sum(counts.values()),failures=failures,
                limitations='Original complete authority-wrapper, primary-actuator and delivery calls with prepared properties/flags. Imports expf/powf use rounded host math; memset_pattern16 is byte-exact. All three control axes present, FM8470=1; finite inputs and positive time constants. Crew/autopilot flags, timestep snapshot provider and property loader are not validated by these calls.')
    Path('analysis/primary-controls-validation.json').write_text(json.dumps(report,indent=2))
    print('COUNTS',counts,'FAILURES',len(failures));print(json.dumps(failures[:8],indent=2))
    if failures:raise SystemExit(1)

if __name__=='__main__':main()
