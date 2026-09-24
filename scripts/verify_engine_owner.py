"""Original jet-owner aggregation with explicitly frozen fuel and health.

The thermal, fuel-bookkeeping and damage functions are bypassed to implement
the user's stated scope. Engine input construction, complete force wrapper,
running lifecycle and owner force/moment accumulation execute original code.
"""
import json,random,struct
from pathlib import Path
from unicorn import UC_HOOK_CODE
from unicorn.x86_const import *
from macho_scan import MachO
from verify_engine_supply import EngineSupplyMachine,FM,ENGINE,PROP,SEED
from verify_jet_model import DATA
from engine_supply import selected_properties,fuel_properties,healthy_running_owner_step
from jet_model import prepare,prepare_nozzle
from component_assembly import f32,add,mul

OWNER=DATA+0x40000


class EngineOwnerMachine(EngineSupplyMachine):
    def __init__(self):
        super().__init__();m=MachO();self.hooks={}
        for a,n in [(0x101a15000,0x2000),(0x101a56000,0x1000),(0x107615000,0x1000)]:
            self.u.mem_map(a,n);self.u.mem_write(a,m.read(a,n))
        self.u.mem_map(OWNER,0x30000)
        self.write(0x107615358,'Q',SEED+0x300)
        for entry in [0x106e61351,0x1019f7b50,0x1019f77a0,0x1019f7fc0]:
            self.u.hook_add(UC_HOOK_CODE,self.owner_hook,begin=entry,end=entry)
    def owner_hook(self,u,a,n,data):
        self.hooks[hex(a)]=self.hooks.get(hex(a),0)+1
        if a==0x106e61351:
            u.mem_write(u.reg_read(UC_X86_REG_RDI),bytes(u.reg_read(UC_X86_REG_RSI)))
        sp=u.reg_read(UC_X86_REG_RSP);ret=struct.unpack('<Q',u.mem_read(sp,8))[0]
        u.reg_write(UC_X86_REG_RSP,sp+8);u.reg_write(UC_X86_REG_RIP,ret)
    def owner(self,*args,**options):
        self.prepare_wrapper(*args,**options)
        p,jet,nozzle,fuel,state,velocity,height,cg,dt=args[:9]
        self.u.mem_write(OWNER,bytes(0x30000))
        self.write(OWNER+0x25a78,'I',1)
        self.write(OWNER+0x25f78,'I',1);self.write(OWNER+0x25f80,'Q',ENGINE)
        self.write(OWNER+0x25934,'i',-1)
        # Poison all aggregates: correct reset is part of the comparison.
        self.write(OWNER+0x25a48,'6d',31.,-47.,12.,111.,333.,-222.)
        self.write(OWNER+0x25b40,'3d',71.,29.,11.)
        self.write(OWNER+0x25a7c,'3f',13.,27.,-33.)
        self.write(ENGINE+0x12,'B',1)
        self.write(ENGINE+0x138,'f',17.)  # outer wrapper must clear it
        self.write(FM+0x1560,'d',height);self.write(FM+0x1618,'3d',*velocity)
        self.write(FM+0x16b8,'d',options.get('load_factor',1.))
        self.write(FM+0x55a0,'Q',OWNER);self.write(FM+0x8254,'f',40.)
        self.write(FM+0x18ec,'f',0.)  # intact engine drag
        for reg,a in [(UC_X86_REG_RDI,OWNER),(UC_X86_REG_RSI,FM),(UC_X86_REG_RDX,SEED),
                      (UC_X86_REG_RCX,1),(UC_X86_REG_R8,SEED+0x400)]:self.u.reg_write(reg,a)
        self.xmm(0,dt);self.run(0x101a155c0)
        result=self.wrapper_result(state)
        result['aggregate_force']=list(struct.unpack('<3d',self.u.mem_read(OWNER+0x25a48,24)))
        result['aggregate_moment']=list(struct.unpack('<3d',self.u.mem_read(OWNER+0x25a60,24)))
        result['engine_angular_momentum']=list(struct.unpack('<3d',self.u.mem_read(OWNER+0x25b40,24)))
        result['propeller_force']=self.read(OWNER+0x25a7c,3)
        result['extra_amplitude']=self.read(ENGINE+0x138,1)[0]
        result['health']=self.read(ENGINE+0x58,1)[0]
        result['fuel_state']=self.read(FM+0x4f48+0x49c,3)
        return result


def expected_owner(*args,**options):
    result=healthy_running_owner_step(*args,**options)
    result.update(extra_amplitude=0.,health=1.,
                  fuel_state=[f32(args[10]),0.,f32(args[11])])
    return result


def main():
    machine=EngineOwnerMachine();rng=random.Random(193199);failures=[];counts={}
    def val(a,b):return f32(rng.uniform(a,b))
    def check(stage,a,e,**where):
        counts[stage]=counts.get(stage,0)+1
        if a!=e:
            failures.append(dict(stage=stage,actual=a,expected=e,**where))
            raise AssertionError(stage+' '+str(where))
    try:
        for name in ['f_16a_block_15_adf','saab_jas39c']:
            fm=json.loads(Path('references/fm-2.59.0.13/'+name+'.blkx').read_text())
            p=selected_properties(fm['EngineType0']);fuel=fuel_properties(fm['Mass'])
            jet=prepare(fm['EngineType0']['Main']);nozzle=prepare_nozzle(fm['Engine0']['Nozzle0'])
            initial=dict(omega=mul(.6,jet['max_omega']),health=1.,cylinders=25,mechanical=1.,extra_amplitude=0.,
                         torque=0.,friction=0.,throttle=.2,running=7,afterburner=False,vtol=0.,reverse=0.,
                         rpm_limit_scale=1.,elapsed=100.,inactive_elapsed=0.,stop_reason=0)
            for i in range(400):
                state=dict(initial,omega=mul(val(.55,1.04),jet['max_omega']),throttle=val(0.,1.1),
                           afterburner=bool(i%2),mechanical=val(.96,1.))
                args=(p,jet,nozzle,fuel,state,[val(-40.,750.),val(-75,75),val(-75,75)],val(-300,23000),
                      [val(-1,1) for _ in range(3)],f32(rng.choice([1/30,1/48,1/60,1/120])),rng.getrandbits(32),
                      2000.,fuel['capacity'])
                opts=dict(load_factor=val(-8,12),ias_u=val(0,650),flaps=val(0,1),airbrake=val(0,1))
                check('complete_owner',machine.owner(*args,**opts),expected_owner(*args,**opts),aircraft=name,case=i)
            ns=ps=initial;nseed=pseed=1899119
            for i in range(240):
                command=f32(.2 if i<40 else 1.1 if i<180 else .75)
                ns=dict(ns,throttle=command,afterburner=command>1.)
                ps=dict(ps,throttle=command,afterburner=command>1.)
                tail=([250.,3.,-2.],4500.,fm['Mass']['CenterOfGravity'],f32(1/48))
                a=machine.owner(p,jet,nozzle,fuel,ns,*tail,nseed,2000.,fuel['capacity'])
                e=expected_owner(p,jet,nozzle,fuel,ps,*tail,pseed,2000.,fuel['capacity'])
                check('chained_owner',a,e,aircraft=name,case=i)
                ns,nseed=a['state'],a['seed'];ps,pseed=e['state'],e['seed']
    except AssertionError as error:print(error)
    report=dict(binary_sha256=machine.sha,counts=counts,failures=failures,hooks=machine.hooks,
        limitations='Original101a155c0 reaches normal return with one jet and no propellers/transmissions. Original outer1019fa1b0, force wrapper1019f3b80, running lifecycle1019f3290, scalar engine and nozzles execute. Thermal1019f7b50, fuel bookkeeping1019f77a0 and damage1019f7fc0 are explicitly bypassed to freeze fuel and health as requested; this is not a test of those deferred routines. bzero is byte-equivalent and expf uses rounded host libm. Prepared owner/property inputs and seed; no game-owner scheduling or live-flight capture. Float32 engine outputs are independently checked through double owner accumulation and cleared propeller/gyro aggregates.')
    Path('analysis/engine-owner-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(counts,'FAILURES',len(failures));print(json.dumps(failures[:1],indent=2))
    if failures:raise SystemExit(1)


if __name__=='__main__':main()
