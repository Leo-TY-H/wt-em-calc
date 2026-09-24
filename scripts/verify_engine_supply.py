"""Original engine supply helpers and complete selected wrapper1019f3b80.

Only the expf import is substituted with rounded host libm. Original atmosphere,
fuel availability, mechanical random modulation, scalar engine and nozzle run.
"""
import json,math,random,struct
from pathlib import Path
from unicorn import UC_HOOK_CODE
from unicorn.x86_const import *
from macho_scan import MachO
from verify_jet_nozzle import JetNozzleMachine
from verify_jet_model import DATA,STOP
from jet_model import prepare,prepare_nozzle
from engine_supply import (fuel_properties,available_fuel,amplitude_coefficients,
                           selected_properties,mechanical_multiplier,wrapper_step)
from component_assembly import f32,mul

FM=DATA+0x30000;ENGINE=DATA+0xb000;PROP=DATA+0xc000;OWN=DATA+0xd000;SEED=DATA+0xe000


class EngineSupplyMachine(JetNozzleMachine):
    def __init__(self):
        super().__init__();m=MachO()
        for a,n in [(0x1019f3000,0xd000),(0x101994000,0x1000),(0x101988000,0x1000),
                    (0x101a51000,0x1000),(0x101a31000,0x2000),(0x107d6f000,0x1000),(0x106e61000,0x1000)]:
            self.u.mem_map(a,n);self.u.mem_write(a,m.read(a,n))
        self.u.mem_map(FM,0xc000)
        self.u.hook_add(UC_HOOK_CODE,self.exp_hook,begin=0x106e615f1,end=0x106e615f1)
        self.u.hook_add(UC_HOOK_CODE,self.exp_hook,begin=0x106e618d3,end=0x106e618d3)
        self.u.hook_add(UC_HOOK_CODE,lambda u,a,n,d:u.emu_stop(),begin=0x1019ff3a9,end=0x1019ff3a9)
    def exp_hook(self,u,a,n,data):
        if a==0x106e615f1:self.xmm(0,math.exp(self.read_xmm()))
        else:
            n=u.reg_read(UC_X86_REG_RDX);pattern=bytes(u.mem_read(u.reg_read(UC_X86_REG_RSI),16))
            u.mem_write(u.reg_read(UC_X86_REG_RDI),(pattern*((n+15)//16))[:n])
        sp=u.reg_read(UC_X86_REG_RSP);ret=struct.unpack('<Q',u.mem_read(sp,8))[0]
        u.reg_write(UC_X86_REG_RSP,sp+8);u.reg_write(UC_X86_REG_RIP,ret)
    def read_xmm(self,index=0):
        return struct.unpack('<f',int(self.u.reg_read(UC_X86_REG_XMM0+index)).to_bytes(16,'little')[:4])[0]
    def load_fuel(self,p,fuel,accumulator,index=0):
        mass=FM+0x4f48
        self.write(mass+0x210+28*index,'3f',p['minimal_load'],p['accumulator_flow'],p['engine_flow'])
        self.write(mass+0x49c+12*index,'3f',fuel,0.,accumulator)
    def available(self,p,fuel,accumulator,load,dt,index=0):
        self.load_fuel(p,fuel,accumulator,index)
        self.u.reg_write(UC_X86_REG_RDI,FM+0x4f48);self.u.reg_write(UC_X86_REG_ESI,index)
        self.xmm(0,load);self.xmm(1,dt);self.run(0x1019948c0)
        return self.read_xmm()
    def amplitude(self,p0,p1):
        self.write(PROP+0x170,'2f',mul(f32(p0[0]),f32(.10471975803375244)),f32(p0[1]))
        self.u.reg_write(UC_X86_REG_RBX,PROP);self.xmm(0,p1[0]);self.xmm(1,p1[1])
        self.u.emu_start(0x1019ff328,0x1019ff3a9,count=100)
        return self.read(PROP+0x180,3)
    def load_mechanical(self,p,omega,health,cylinders,previous,seed,disabled=False,enabled=True):
        self.write(PROP+0x15c,'f',p['inverse_omega']);self.write(PROP+0x180,'3f',*p['amplitude'])
        self.write(PROP+0x244,'i',p['cylinders']);self.write(ENGINE+0x28,'f',omega)
        self.write(ENGINE+0x58,'f',health);self.write(ENGINE+0x6c,'i',cylinders)
        self.write(ENGINE+0x38,'f',previous);self.write(SEED,'I',seed)
        self.write(0x107d6fc10,'2B',disabled,enabled)
    def mechanical(self,p,omega,health,cylinders,previous,extra,torque,friction,dt,seed,disabled=False,enabled=True):
        self.load_mechanical(p,omega,health,cylinders,previous,seed,disabled,enabled)
        for reg,a in [(UC_X86_REG_RDI,SEED),(UC_X86_REG_RSI,PROP),(UC_X86_REG_RDX,ENGINE+0x10)]:self.u.reg_write(reg,a)
        for i,x in enumerate([dt,torque,friction,extra]):self.xmm(i,x)
        self.run(0x1019f73f0)
        return self.read_xmm(),self.read(ENGINE+0x38,1)[0],struct.unpack('<I',self.u.mem_read(SEED,4))[0]
    def reset_drag(self):
        self.write(FM+0x18ec,'16f',*range(1,17));self.u.reg_write(UC_X86_REG_RDI,FM)
        self.run(0x101a31e80)
        return self.read(FM+0x18ec,16)
    def prepare_wrapper(self,p,jet,nozzle,fuel,state,velocity,height,cg,dt,seed,system_fuel,accumulator,
                load_factor=1.,ias_u=0.,flaps=0.,airbrake=0.,engine_drag_area=0.,fuel_g_effect=True,
                mechanical_disabled=False,mechanical_enabled=True):
        self.prepare_call(jet,nozzle,1.,0.,0.,0.,dt,cg)
        self.u.mem_write(FM,bytes(0xc000));self.load_fuel(fuel,system_fuel,accumulator)
        s=state;self.write(ENGINE,'2Q',PROP,OWN);self.write(ENGINE+0x320,'2Q',DATA,DATA+0x9000)
        self.write(PROP,'B',2);self.write(PROP+0x154,'f',p['omega_limit'])
        self.write(PROP+0x1e4,'I2f',3,p['throttle_boost'],1.1)
        self.write(PROP+0x268,'Q',OWN+0x100);self.write(PROP+0x278,'I',len(p['vtol_throttle']))
        for i,(x,inv,ys) in enumerate(p['vtol_throttle']):self.write(OWN+0x100+12*i,'3f',x,inv,*ys)
        self.write(PROP+0x284,'8f',*p['reverse_throttle'],*p['ias_vtol'])
        self.load_mechanical(p,s['omega'],s['health'],s['cylinders'],s['mechanical'],seed,mechanical_disabled,mechanical_enabled)
        self.write(ENGINE+0x1c,'B',s['running']);self.write(ENGINE+0x20,'2f',s['elapsed'],s['inactive_elapsed'])
        self.write(ENGINE+0x30,'f',s['rpm_limit_scale']);self.write(ENGINE+0xa4,'f',s['throttle'])
        self.write(ENGINE+0xb4,'i',-1);self.write(ENGINE+0xbc,'B',s['afterburner'])
        self.write(ENGINE+0xc4,'2f',s['vtol'],s['reverse']);self.write(ENGINE+0x120,'2f',s['torque'],s['friction'])
        self.write(ENGINE+0x134,'If',s['stop_reason'],s['extra_amplitude'])
        self.write(FM+0x8470,'B',1);self.write(0x107d6fbea,'B',fuel_g_effect)
        self.write(FM+0x8468,'f',ias_u);self.write(FM+0x5320,'3f',*cg)
        self.write(FM+0x2b58,'f',airbrake);self.write(FM+0x2b60,'f',flaps)
        self.write(SEED+0x10,'3f',*velocity)
        for reg,a in [(UC_X86_REG_RDI,ENGINE),(UC_X86_REG_RSI,FM),(UC_X86_REG_RDX,SEED+0x10),
                      (UC_X86_REG_RCX,SEED),(UC_X86_REG_R8,1),(UC_X86_REG_R9,ENGINE+0x94)]:self.u.reg_write(reg,a)
        for i,x in enumerate([dt,height,0.,0.,0.,engine_drag_area,load_factor,0.]):self.xmm(i,x)
    def wrapper(self,*args,**options):
        self.prepare_wrapper(*args,**options)
        self.run(0x1019f3b80)
        return self.wrapper_result(args[4])
    def wrapper_result(self,state):
        result=dict(state=dict(state),seed=struct.unpack('<I',self.u.mem_read(SEED,4))[0],
                    force=self.read(ENGINE+0x27c,3),moment=self.read(ENGINE+0x288,3),
                    consumption=self.read(ENGINE+0x94,1)[0],target_omega=self.read(ENGINE+0x33c,1)[0],
                    active=bool(self.u.mem_read(ENGINE+0x4a0,1)[0]))
        for field,off in [('omega',0x28),('mechanical',0x38),('torque',0x120),('friction',0x124),
                          ('effective_vtol',0x18),('effective_throttle',0x14),('elapsed',0x20),('inactive_elapsed',0x24)]:
            result['state'][field]=self.read(ENGINE+off,1)[0]
        result['state']['running']=self.u.mem_read(ENGINE+0x1c,1)[0]
        result['state']['stop_reason']=struct.unpack('<I',self.u.mem_read(ENGINE+0x134,4))[0]
        return result


def main():
    m=EngineSupplyMachine();rng=random.Random(193191);failures=[];counts={};events=0
    def val(a,b):return f32(rng.uniform(a,b))
    def check(stage,a,e,**info):
        counts[stage]=counts.get(stage,0)+1
        if a!=e:failures.append(dict(stage=stage,actual=a,expected=e,**info))
    check('intact_drag_reset',m.reset_drag(),[0.]*16)
    examples=[]
    for name in ['f_16a_block_15_adf','saab_jas39c']:
        fm=json.loads(Path('references/fm-2.59.0.13/'+name+'.blkx').read_text())
        p=selected_properties(fm['EngineType0']);fuel=fuel_properties(fm['Mass'])
        jet=prepare(fm['EngineType0']['Main']);nozzle=prepare_nozzle(fm['Engine0']['Nozzle0'])
        for i in range(1000):
            a0=[val(9000,15000),val(0,1)];a1=[0.,val(1,60)]
            check('amplitude_loader',m.amplitude(a0,a1),amplitude_coefficients(a0,a1))
            total=val(0,4000);acc=val(0,min(fuel['capacity'],total));dt=f32(rng.choice([1/48,1/60,1/30,.1]))
            fp=dict(fuel)
            if i%3==0:fp['engine_flow']=val(0,100.)
            args=(fp,total,acc,val(-5,5),dt)
            check('available_fuel',m.available(*args,index=i%16),available_fuel(*args))
            args=(p,mul(val(0,1.25),jet['max_omega']),val(.01,1.),rng.randrange(1,26),val(-1.,1.1),
                  val(0.,3.),val(-10,10),val(-1,1),dt,rng.getrandbits(32))
            options=dict(disabled=i%13==0,enabled=i%11!=0)
            actual=m.mechanical(*args,**options);expected=mechanical_multiplier(*args,**options)
            events+=actual[2]!=args[-1];check('mechanical',actual,expected)
        for i in range(600):
            s=dict(omega=mul(val(0,1.25),jet['max_omega']),health=val(.1,1.),cylinders=rng.randrange(1,26),
                   mechanical=val(-1,1.1),extra_amplitude=val(0,2),torque=val(-10,10),friction=val(-1,1),
                   throttle=val(0,1.2),running=rng.choice([0,6,7,7,7,8]),afterburner=bool(i%2),
                   vtol=0. if i%2 else val(0,1),reverse=0. if i%2 else val(0,1),rpm_limit_scale=val(.5,1.1),
                   elapsed=val(0,50),inactive_elapsed=val(0,3),stop_reason=0)
            total=val(0,4000) if i%5 else f32(10**rng.uniform(-8,-1));acc=val(0,min(fuel['capacity'],total))
            args=(p,jet,nozzle,fuel,s,[val(-80,700),val(-100,100),val(-100,100)],val(-500,25000),
                  [val(-1,1) for _ in range(3)],f32(1/48),rng.getrandbits(32),total,acc)
            opts=dict(load_factor=val(-5,5),ias_u=val(-100,500),flaps=val(0,1),airbrake=val(0,1),
                      engine_drag_area=0. if i%2 else val(0,.4))
            check('complete_wrapper',m.wrapper(*args,**opts),wrapper_step(*args,**opts),aircraft=name,case=i)
        ns=ps=dict(omega=mul(.6,jet['max_omega']),health=1.,cylinders=25,mechanical=1.,extra_amplitude=0.,
                   torque=0.,friction=0.,throttle=0.,running=7,afterburner=False,vtol=0.,reverse=0.,
                   rpm_limit_scale=1.,elapsed=100.,inactive_elapsed=0.,stop_reason=0)
        nseed=pseed=1357911
        for i in range(600):
            command=f32(.1 if i<100 else 1.1 if i<400 else .2)
            ns=dict(ns,throttle=command,afterburner=command>1.);ps=dict(ps,throttle=command,afterburner=command>1.)
            args=(p,jet,nozzle,fuel);tail=([250.,0.,0.],4500.,fm['Mass']['CenterOfGravity'],f32(1/48))
            a=m.wrapper(*args,ns,*tail,nseed,1000.,fuel['capacity'])
            e=wrapper_step(*args,ps,*tail,pseed,1000.,fuel['capacity'])
            ns,nseed=a['state'],a['seed'];ps,pseed=e['state'],e['seed']
            check('chained_wrapper',a,e,aircraft=name,case=i)
        examples.append(dict(aircraft=name,fuel=fuel,mechanical_properties=p))
    report=dict(binary_sha256=m.sha,counts=counts,random_stream_updates=events,failures=failures,selected_properties=examples,
                limitations='Complete selected1019f3b80 returns, with original fuel, atmosphere, mechanical and scalar/nozzle callees. Only expf uses rounded host libm. Static property adapter and prescribed fuel/health/command inputs; outer lifecycle, tank depletion and thermal state are not included. Chained states independently propagate RPM, random seed, modulation and running status. Finite equality, not exceptional-float or original-libm bit identity.')
    Path('analysis/engine-supply-validation.json').write_text(json.dumps(report,indent=2))
    print(counts,'FAILURES',len(failures));print(json.dumps(failures[:3],indent=2))
    if failures:raise SystemExit(1)

if __name__=='__main__':main()
