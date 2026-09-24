"""Native wing normalization and contiguous two-wing overload/stress branch."""
import json,random,struct
from pathlib import Path
from unicorn import UC_HOOK_CODE
from unicorn.x86_const import *
from macho_scan import MachO
from verify_primary_controls import Controls,BASE
from verify_component_assembly import FRAME
from component_assembly import f32
from structural_limits import (wing_load_ratios,overload_step,random_fraction,position_tick_seed,
                               wing_ias_step,mach_step,control_ias_step,tail_ias_step,flutter_commands)

UNIT=BASE+0x10000

class StrengthMachine(Controls):
    def __init__(self):
        super().__init__();m=MachO()
        self.u.mem_map(0x104f94000,0x4000);self.u.mem_write(0x104f94000,m.read(0x104f94000,0x4000))
        self.u.mem_map(0x1050c7000,0x1000);self.u.mem_write(0x1050c7000,m.read(0x1050c7000,0x1000))
        for va,size in [(0x101a3e000,0x1000),(0x107521000,0x2000),(0x107f5f000,0x1000),
                        (0x10001a000,0x1000),(0x101988000,0x1000)]:
            self.u.mem_map(va,size)
            if va!=0x107f5f000:self.u.mem_write(va,m.read(va,size))
        for va in [0x104f97d40,0x104f97e90,0x1050c77e0]:self.u.hook_add(UC_HOOK_CODE,self.event,begin=va,end=va)
        for va in [0x10001a3b0,0x1019881d0]:self.u.hook_add(UC_HOOK_CODE,self.log_only,begin=va,end=va)
        self.draw_regs={0x104f95564:(1,0),0x104f9569a:(1,0),0x104f957b4:(1,0),0x104f967e9:(0,4),0x104f968c7:(0,4),
                        0x104f969cf:(0,2),0x104f96a2f:(0,2),0x104f96abb:(0,2),0x104f95abb:(0,2),
                        0x104f95b1b:(0,2),0x104f95b77:(0,2),0x104f95bd3:(0,2),0x104f95cdf:(0,2)}
        for va in self.draw_regs:self.u.hook_add(UC_HOOK_CODE,self.draw,begin=va,end=va)
        self.stop_address=None
        for va in [0x104f95742,0x104f95a5a,0x104f95d11,0x104f96667,0x104f96af5]:
            self.u.hook_add(UC_HOOK_CODE,self.stop,begin=va,end=va)
    def stop(self,u,address,size,data):
        if address==self.stop_address:u.emu_stop()
    def run_boundary(self,start,end):
        # An explicit hook also stops inside a translated block cached by a
        # previous, longer overlapping slice.
        self.stop_address=end;self.u.emu_start(start,end,count=5000)
        if self.u.reg_read(UC_X86_REG_RIP)!=end:raise RuntimeError('Boundary not reached')
        self.stop_address=None
    def draw(self,u,address,size,data):
        self.draws.append([self.read_xmm(n)[0] for n in self.draw_regs[address]])
    def log_only(self,u,address,size,data):
        # Atmosphere at104f9590a is used only by the following diagnostic log.
        if address==0x1019881d0:self.xmm(0,[0.])
        self.return_hook(u)
    def return_hook(self,u):
        sp=u.reg_read(UC_X86_REG_RSP);ret=struct.unpack('<Q',u.mem_read(sp,8))[0]
        u.reg_write(UC_X86_REG_RSP,sp+8);u.reg_write(UC_X86_REG_RIP,ret)
    def event(self,u,address,size,data):
        # Calls are event boundaries, not substituted force/stress calculations.
        if address==0x104f97d40:self.events.append(u.reg_read(UC_X86_REG_ESI))
        elif address==0x104f97e90:
            self.parts.append(u.reg_read(UC_X86_REG_ESI));u.reg_write(UC_X86_REG_EAX,0)
        else:self.notifications.append(bytes(u.mem_read(u.reg_read(UC_X86_REG_RSI),14)).hex())
        self.return_hook(u)
    def ratios(self,force,strength):
        self.reset();self.floats(BASE+0x81b4,strength);self.floats(FRAME-0x480,[force[0]])
        self.xmm(13,[force[1]]);self.xmm(7,[f32(4e-19)])
        self.u.emu_start(0x106c62e1b,0x106c62e9d,count=300)
        return list(struct.unpack('<2f',self.u.mem_read(BASE+0x8970,8)))
    def overload(self,ratios,health,stress,dt,seed):
        self.reset();self.events=[];self.notifications=[]
        self.floats(BASE+0x8970,ratios);self.floats(UNIT+0x5524,stress)
        self.u.mem_write(UNIT+0x2ef0,struct.pack('<Q',BASE));self.u.mem_write(FRAME-0x370,struct.pack('<Q',UNIT))
        self.floats(FRAME-0x374,[dt]);self.u.mem_write(FRAME-0x364,struct.pack('<I',seed))
        self.floats(0x107d6fc18+0x22c,[6.,-3.,-10.,3.,10.]);self.u.reg_write(UC_X86_REG_RDI,BASE)
        self.u.reg_write(UC_X86_REG_R11,UNIT);self.u.reg_write(UC_X86_REG_RSP,FRAME-0x800)
        self.xmm(13,[f32(f32(1.6)-f32(f32(.6)*health[0]))]);self.xmm(12,[f32(f32(1.6)-f32(f32(.6)*health[1]))])
        self.u.emu_start(0x104f94e86,0x104f952d9,count=5000)
        return dict(stress=list(struct.unpack('<2f',self.u.mem_read(UNIT+0x5524,8))),
                    seed=struct.unpack('<I',self.u.mem_read(FRAME-0x364,4))[0],break_events=self.events)
    def seed(self,position,tick):
        self.reset();self.u.mem_write(BASE+0x1558,struct.pack('<3d',*position))
        self.u.reg_write(UC_X86_REG_RDI,BASE);self.u.reg_write(UC_X86_REG_ESI,tick&0xffffffff)
        self.call(0x101a3e5e0);return self.u.reg_read(UC_X86_REG_EAX)
    def boundary_setup(self,dt,seed):
        self.reset();self.events=[];self.parts=[];self.draws=[];self.notifications=[]
        for address,value in [(UNIT+0x2ef0,BASE),(FRAME-0x370,UNIT),(0x107f5fda8,BASE+0x18000)]:
            self.u.mem_write(address,struct.pack('<Q',value))
        self.u.mem_write(BASE+0x1802f,b'\x01');self.floats(FRAME-0x374,[dt])
        self.u.mem_write(FRAME-0x364,struct.pack('<I',seed))
        self.u.reg_write(UC_X86_REG_RDI,BASE);self.u.reg_write(UC_X86_REG_R11,UNIT)
        self.u.reg_write(UC_X86_REG_RSP,FRAME-0x800);self.xmm(1,[dt])
        self.floats(0x107d6fc18+0x240,[.03,10.,.05,40.,.05])
    def boundary_result(self):
        return dict(seed=struct.unpack('<I',self.u.mem_read(FRAME-0x364,4))[0],
                    break_events=self.events,part_events=self.parts,draw_values=self.draws)
    def wing_ias(self,ias_u,vne,health,dt,seed,speed_multiplier=1.):
        self.boundary_setup(dt,seed);self.xmm(10,[ias_u]);self.floats(FRAME-0x378,[ias_u])
        self.xmm(4,[f32(vne*speed_multiplier)]);self.xmm(11,[speed_multiplier])
        for n,h in zip([13,12],health):self.xmm(n,[f32(f32(1.6)-f32(f32(.6)*h))])
        self.run_boundary(0x104f954f2,0x104f95742)
        return self.boundary_result()
    def mach(self,mach,mne,dt,seed,additional_mne=()):
        self.boundary_setup(dt,seed);self.floats(BASE+0x8464,[mach]);self.floats(BASE+0x81c0,[mne])
        self.u.mem_write(BASE+0x7550,struct.pack('<I',len(additional_mne)))
        for n,value in enumerate(additional_mne):self.floats(BASE+0x75e0+n*0xd8,[value])
        self.run_boundary(0x104f96671,0x104f96af5)
        return self.boundary_result()
    def control_ias(self,ias_excess,dt,seed):
        self.boundary_setup(dt,seed);self.xmm(2,[ias_excess]);self.xmm(3,[0.])
        self.u.reg_write(UC_X86_REG_RBX,0x107d6fc18)
        self.run_boundary(0x104f95a5a,0x104f95d11)
        return self.boundary_result()
    def tail_ias(self,ias_u,maximum_vne,dt,seed):
        self.boundary_setup(dt,seed);self.xmm(10,[ias_u]);self.xmm(3,[maximum_vne])
        self.floats(FRAME-0x378,[ias_u])
        self.run_boundary(0x104f95742,0x104f95a5a)
        return self.boundary_result()
    def flutter(self,commands,ias_excess,tick):
        self.boundary_setup(f32(1/48),0);self.floats(BASE+0x1694,commands)
        self.floats(FRAME-0x38c,[ias_excess]);self.u.mem_write(FRAME-0x380,struct.pack('<I',tick&0xffffffff))
        self.run_boundary(0x104f964f1,0x104f96667)
        return list(struct.unpack('<3f',self.u.mem_read(BASE+0x1694,12)))


def main():
    rng=random.Random(193189);m=StrengthMachine();fails=[];counts=dict(wing_normalization=0,overload=0,chained_stress=0);event_count=0
    def val(a,b):return f32(rng.uniform(a,b))
    for i in range(2000):
        forces=[val(-1e6,2e6) for _ in range(2)];strength=[-val(1e4,8e5),val(1e4,1e6)]
        a=m.ratios(forces,strength);e=wing_load_ratios(forces,strength);counts['wing_normalization']+=1
        if a!=e:fails.append(dict(stage='ratios',actual=a,expected=e))
        args=([val(0.,5.) for _ in range(2)],[val(.1,1.) for _ in range(2)],
              [val(0.,1.) for _ in range(2)],f32(rng.choice([1/30,1/48,1/60,1/120])),rng.getrandbits(32))
        a=m.overload(*args);e=overload_step(*args);counts['overload']+=1;event_count+=len(a['break_events'])
        if a!={k:e[k] for k in a}:fails.append(dict(stage='overload',args=args,actual=a,expected=e))
    native_stress=port_stress=[0.,0.]
    for i in range(600):
        ratio=[1.1,1.2] if i<300 else [.8,.95];seed=rng.getrandbits(32)
        a=m.overload(ratio,[1.,1.],native_stress,f32(1/48),seed)
        e=overload_step(ratio,[1.,1.],port_stress,f32(1/48),seed);counts['chained_stress']+=1
        native_stress=a['stress'];port_stress=e['stress']
        if a!={k:e[k] for k in a}:fails.append(dict(stage='chained',i=i,actual=a,expected=e))
    for i in range(2000):
        pos=[rng.uniform(-1e7,1e7) for _ in range(3)];tick=rng.getrandbits(32)
        if i%10==0:pos[i%3]=rng.choice([1e15,-1e15,214748364.7,-214748364.8])
        a=m.seed(pos,tick);e=position_tick_seed(pos,tick);counts['seed']=counts.get('seed',0)+1
        if a!=e:fails.append(dict(stage='seed',actual=a,expected=e))
        dt=f32(rng.choice([1/30,1/48,1/60,1/120]));seed=rng.getrandbits(32)
        cases=[('wing_ias',m.wing_ias,wing_ias_step,(val(200,650),val(300,450),[val(.1,1) for _ in range(2)],dt,seed,val(.8,1.2))),
               ('mach',m.mach,mach_step,(val(1.,4.),val(1.2,2.5),dt,seed,[val(1.,3.) for _ in range(i%13)])),
               ('tail_ias',m.tail_ias,tail_ias_step,(val(250,700),val(300,450),dt,seed)),
               ('control_ias',m.control_ias,control_ias_step,(val(-100,150),dt,seed))]
        for stage,native,port,args in cases:
            a=native(*args);p=port(*args)
            e={k:p.get(k,[]) for k in ['seed','break_events','part_events']}
            e['draw_values']=[[d['value'],d['threshold']] for d in p['draws']]
            counts[stage]=counts.get(stage,0)+1;event_count+=len(a['break_events'])+len(a['part_events'])
            if a!=e:fails.append(dict(stage=stage,args=args,actual=a,expected=e))
        args=([val(-1.,1.) for _ in range(3)],val(-10,100),tick)
        a=m.flutter(*args);e=flutter_commands(*args);counts['flutter']=counts.get('flutter',0)+1
        if a!=e:fails.append(dict(stage='flutter',args=args,actual=a,expected=e))
    report=dict(binary_sha256=m.sha,counts=counts,observed_random_break_requests=event_count,failures=fails,
                limitations='Original normalization, overload, wing/tail/control IAS, complete wing/control Mach, flutter branches and complete position/tick seed helper. Damage calls observed as events; their mesh/health mutation lies outside intact-domain boundary. Atmosphere call used only for tail-break diagnostic log is stubbed; log is omitted. Prepared health/load ratios/stress/command states. Deployed devices, pre-branch gates and full owner execution are not covered here. Finite numerical equality, not signed-zero/NaN bit parity.')
    Path('analysis/structural-limits-validation.json').write_text(json.dumps(report,indent=2));print(counts,'EVENTS',event_count,'FAILURES',len(fails));print(json.dumps(fails[:3],indent=2))
    if fails:raise SystemExit(1)

if __name__=='__main__':main()
