"""Original jet table/mode kernels and contiguous scalar engine execution."""
import hashlib,json,random,struct
from pathlib import Path
from unicorn import Uc,UC_ARCH_X86,UC_MODE_64
from unicorn.x86_const import *
from macho_scan import MachO
from component_assembly import f32,mul
from control_mixer import density_at_height
from verify_polar_machine_code import EXPECTED_BINARY_SHA256
from jet_model import prepare,table,mode,scalar_update,steady

DATA=0x230000000;STACK=DATA+0x10000;STOP=DATA+0x20000


class JetMachine:
    def __init__(self):
        m=MachO();self.sha=hashlib.sha256(m.data).hexdigest()
        if self.sha!=EXPECTED_BINARY_SHA256:raise ValueError('Binary changed')
        self.u=Uc(UC_ARCH_X86,UC_MODE_64)
        for a,n in [(0x1019f0000,0x3000),(0x1071e4000,0x40000)]:
            self.u.mem_map(a,n);self.u.mem_write(a,m.read(a,n))
        self.u.mem_map(DATA,0x21000)
    def write(self,a,fmt,*values):self.u.mem_write(a,struct.pack('<'+fmt,*values))
    def read(self,a,n):return list(struct.unpack('<'+'f'*n,self.u.mem_read(a,4*n)))
    def xmm(self,i,x):self.u.reg_write(UC_X86_REG_XMM0+i,int.from_bytes(struct.pack('<f',x),'little'))
    def load(self,p):
        self.u.mem_write(DATA,bytes(0x10000))
        self.write(DATA,'Q',DATA+0x1000);self.write(DATA+0x10,'I',len(p['density']))
        self.write(DATA+0x20,'Q',DATA+0x1800);self.write(DATA+0x30,'I',len(p['speed']))
        self.write(DATA+0x38,'4f',*p['bases']);self.write(DATA+0x50,'Q',DATA+0x2000)
        cells=[x for row in p['cells'] for cell in row for x in cell]
        self.write(DATA+0x60,'I',len(cells)//4)
        self.write(DATA+0x1000,'f'*len(p['density']),*p['density'])
        self.write(DATA+0x1800,'f'*len(p['speed']),*p['speed'])
        self.write(DATA+0x2000,'f'*len(cells),*cells)
        self.write(DATA+0x68,'6f',p['tau'],0.,p['max_omega'],p['throttle_scale'],p['consumption'],p['torque_zero'])
        self.write(DATA+0x80,'Q',DATA+0x4000);self.write(DATA+0x90,'I',len(p['throttle']))
        self.write(DATA+0x98,'Q',DATA+0x4800);self.write(DATA+0xa8,'I',len(p['modes']))
        self.write(DATA+0xb0,'I',len(p['modes']))
        for i,(x,inv,_) in enumerate(p['throttle']):self.write(DATA+0x4000+i*12,'ffI',x,inv,i)
        for i,(x,inv,v) in enumerate(p['modes']):
            self.write(DATA+0x4800+i*12,'ffI',x,inv,i);self.write(DATA+0xb4+i*20,'5f',*v)
    def run(self,entry,stop=STOP):
        sp=STACK+0xfff8;self.write(sp,'Q',STOP);self.u.reg_write(UC_X86_REG_RSP,sp)
        self.u.emu_start(entry,stop,count=30000)
        if self.u.reg_read(UC_X86_REG_RIP)!=stop:raise RuntimeError('Did not reach stop')
    def table(self,p,density,speed):
        self.load(p)
        for reg,a in [(UC_X86_REG_RDI,DATA),(UC_X86_REG_RSI,DATA+0x6000),
                      (UC_X86_REG_RDX,DATA+0x6100),(UC_X86_REG_RCX,DATA+0x6104),
                      (UC_X86_REG_R8,DATA+0x6108),(UC_X86_REG_R9,DATA+0x610c)]:self.u.reg_write(reg,a)
        self.xmm(0,density);self.xmm(1,speed);self.run(0x1019f05d0)
        return self.read(DATA+0x6100,4)
    def mode(self,p,rpm):
        self.load(p)
        for reg,a in [(UC_X86_REG_RDI,DATA),(UC_X86_REG_RSI,DATA+0x6100),(UC_X86_REG_RDX,DATA+0x6104),
                      (UC_X86_REG_RCX,DATA+0x610c),(UC_X86_REG_R8,DATA+0x6110)]:self.u.reg_write(reg,a)
        self.xmm(0,rpm);self.run(0x1019f0d30);return self.read(DATA+0x6100,5)
    def scalar(self,p,density,speed,omega,throttle,dt,afterburner=False,fuel_available=1e6,health=1.,shaft_fraction=1.,running=2):
        self.load(p);engine=DATA+0x7000;state=DATA+0x8000
        self.write(engine,'2Q',DATA,DATA+0x9000)
        self.write(state,'2f',density,speed);self.write(state+0x10,'2fIf',omega,shaft_fraction,running,throttle)
        self.write(state+0x20,'B',afterburner);self.write(state+0x44,'2f',health,fuel_available)
        self.u.reg_write(UC_X86_REG_RDI,engine);self.u.reg_write(UC_X86_REG_RSI,state);self.xmm(0,dt)
        self.run(0x1019f19e0,0x1019f1e90)
        frame=self.u.reg_read(UC_X86_REG_RBP)
        return dict(thrust=self.read(frame-0x38,1)[0],torque=self.read(frame-0x30,1)[0],consumption=self.read(frame-0x34,1)[0],
                    next_omega=self.read(engine+0x18,1)[0],target_omega=self.read(engine+0x1c,1)[0],active=bool(self.u.mem_read(engine+0x180,1)[0]))


def main():
    rng=random.Random(191605);machine=JetMachine();counts=dict(table=0,mode=0,scalar=0);failures=[];examples=[]
    for name in ['f_16a_block_15_adf','saab_jas39c']:
        fm=json.loads(Path(f'references/fm-2.59.0.13/{name}.blkx').read_text());p=prepare(fm['EngineType0']['Main'])
        for i in range(600):
            h=f32(rng.uniform(-1000,25000));rho=density_at_height(h);speed=f32(rng.uniform(-50,850))
            actual=machine.table(p,rho,speed);expected=table(p,rho,speed);counts['table']+=1
            if actual!=expected:failures.append(dict(stage='table',aircraft=name,h=h,speed=speed,actual=actual,expected=expected))
            rpm=f32(rng.uniform(0,1.2));actual=machine.mode(p,rpm);expected=mode(p,rpm);counts['mode']+=1
            if actual!=expected:failures.append(dict(stage='mode',actual=actual,expected=expected))
            args=(p,rho,speed,mul(rpm,p['max_omega']),f32(rng.uniform(0,1)),f32(rng.uniform(.001,.05)))
            options=dict(afterburner=bool(i&1),fuel_available=f32(10**rng.uniform(-7,1)),health=f32(rng.uniform(.1,1)),
                         shaft_fraction=f32(rng.uniform(0,1.2)),running=i%3)
            actual=machine.scalar(*args,**options);expected=scalar_update(*args,**options);counts['scalar']+=1
            if actual!=expected:failures.append(dict(stage='scalar',aircraft=name,args=args[1:],options=options,actual=actual,expected=expected))
        for h in [0,4500,9000]:
            for speed in [0,250,500]:
                examples.append(dict(aircraft=name,height=h,body_u=speed,dry=steady(p,h,speed,1.),wet=steady(p,h,speed,1.1)))
    report=dict(binary_sha256=machine.sha,calls=counts,failures=failures,examples=examples,
                limitations='Static property adapter. Actual unmodified whole table/mode kernels and contiguous scalar-engine prefix run without hooks. Nozzle stage and upstream state/fuel providers are separately traced; this is not a full timestep replay.')
    Path('analysis/jet-model-validation.json').write_text(json.dumps(report,indent=2));print('CALLS',counts,'FAILURES',len(failures));print(json.dumps(failures[:3],indent=2))
    if failures:raise SystemExit(1)


if __name__=='__main__':main()
