"""Run the complete original scalar/nozzle engine function, without hooks."""
import copy,json,random,struct
from pathlib import Path
from unicorn.x86_const import *
from verify_jet_model import JetMachine,DATA,STOP
from jet_model import prepare,prepare_nozzle,scalar_update,selected_nozzle
from component_assembly import f32,mul
from control_mixer import density_at_height


class JetNozzleMachine(JetMachine):
    def prepare_call(self,p,nozzle,density,speed,omega,throttle,dt,cg,afterburner=False,
                 fuel_available=1e6,health=1.,shaft_fraction=1.,running=2,
                 ias=0.,flaps=0.,airbrake=0.,vtol=0.,reverse=0.,sticks=(0.,0.,0.)):
        self.load(p);engine=DATA+0x7000;state=DATA+0x8000;prop=DATA+0x9008
        self.write(engine,'2Q',DATA,prop-8);self.write(prop-8,'I',1)
        # B0 is exact for both selected directions. B1/B2 have no contribution
        # because every configured deflection is zero. No sincos import executes.
        self.write(prop,'9f',1.,0.,0.,0.,1.,0.,0.,0.,1.)
        self.write(prop+0x24,'5f',*nozzle['position'],nozzle['ratio'],nozzle['maximum'])
        self.write(prop+0xb8,'4f',-f32(3.141592741),f32(3.141592741),-f32(3.141592741),f32(3.141592741))
        self.write(prop+0xc8,'10f',1.,0.,1.,0.,0.,0.,1.,0.,1.,1.)
        self.write(prop+0x138,'4f',*nozzle['flaps'])
        address=DATA+0xa000
        # Preserve the real one-row zero tables, rather than empty-table bypass.
        tables={0x40:[(0.,0.,[0.,0.,0.])],0x58:[(0.,0.,[0.,0.,0.])],
                0x70:[(0.,0.,[0.,0.,0.])],0x88:[(0.,0.,[0.,0.])],0xa0:[(0.,0.,[0.,0.])],
                0xf0:[(0.,0.,[0.,0.,0.])],0x108:[(0.,0.,[0.,0.,0.])],0x120:[(0.,0.,[0.,0.,0.])],
                0x148:nozzle['airbrake'],0x160:nozzle['vtol'],0x178:nozzle['reverse']}
        for offset,rows in tables.items():
            self.write(prop+offset,'Q',address);self.write(prop+offset+0x10,'I',len(rows))
            for x,inv,values in rows:
                self.write(address,'f'*(2+len(values)),x,inv,*values);address+=4*(2+len(values))
        self.write(state,'3f',density,speed,ias);self.write(state+0x10,'2fIf',omega,shaft_fraction,running,throttle)
        self.write(state+0x20,'B',afterburner);self.write(state+0x24,'7f',*sticks,flaps,airbrake,vtol,reverse)
        self.write(state+0x44,'5f',health,fuel_available,*cg)
        self.u.reg_write(UC_X86_REG_RDI,engine);self.u.reg_write(UC_X86_REG_RSI,state);self.xmm(0,dt)
        return engine

    def complete(self,*args,**options):
        engine=self.prepare_call(*args,**options)
        self.run(0x1019f19e0)
        return dict(force=self.read(engine+0x160,3),moment=self.read(engine+0x16c,3),angles=self.read(engine+0xe0,2),
                    next_omega=self.read(engine+0x18,1)[0],target_omega=self.read(engine+0x1c,1)[0],
                    torque=self.read(engine+0x178,1)[0],consumption=self.read(engine+0x17c,1)[0],
                    active=bool(self.u.mem_read(engine+0x180,1)[0]))


def expected(p,nozzle,density,speed,omega,throttle,dt,cg,ias=0.,flaps=0.,airbrake=0.,vtol=0.,reverse=0.,sticks=(0.,0.,0.),**options):
    scalar=scalar_update(p,density,speed,omega,throttle,dt,**options)
    return dict(selected_nozzle(nozzle,scalar['thrust'],cg,ias,flaps,airbrake,vtol,reverse),
                **{k:scalar[k] for k in ['next_omega','target_omega','torque','consumption','active']})


def main():
    m=JetNozzleMachine();rng=random.Random(193187);fails=[];counts=dict(random=0,auxiliary_states=0,chained_rpm=0);caps=0
    def val(a,b):return f32(rng.uniform(a,b))
    for name in ['f_16a_block_15_adf','saab_jas39c']:
        fm=json.loads(Path('references/fm-2.59.0.13/'+name+'.blkx').read_text());p=prepare(fm['EngineType0']['Main']);base=prepare_nozzle(fm['Engine0']['Nozzle0'])
        for i in range(800):
            noz=copy.deepcopy(base)
            if i%7==0:noz['maximum']=val(500,20000);caps+=1
            rho=density_at_height(val(-1000,25000));speed=val(-150,800)
            args=(p,noz,rho,speed,mul(val(0.,1.2),p['max_omega']),val(0.,1.),f32(1/48),[val(-2,2) for _ in range(3)])
            options=dict(afterburner=bool(i&1),fuel_available=f32(10**rng.uniform(-7,1)),health=val(.1,1.),
                         shaft_fraction=val(0,1.2),running=i%3,ias=val(-200,700),flaps=val(0,1),airbrake=val(0,1),sticks=[val(-1,1) for _ in range(3)])
            stage='random'
            if i%2:options.update(vtol=val(0,1),reverse=val(0,1));stage='auxiliary_states'
            a=m.complete(*args,**options);e=expected(*args,**options);counts[stage]+=1
            if a!=e:fails.append(dict(stage=stage,aircraft=name,i=i,actual=a,expected=e))
        native_omega=port_omega=mul(f32(.7),p['max_omega'])
        for i in range(400):
            command=f32(.5 if i<100 else 1. if i<300 else .8)
            args=(p,base,density_at_height(4500.),250.)
            tail=(command,f32(1/48),fm['Mass']['CenterOfGravity']);options=dict(afterburner=command==1.)
            a=m.complete(*args,native_omega,*tail,**options);e=expected(*args,port_omega,*tail,**options)
            counts['chained_rpm']+=1;native_omega=a['next_omega'];port_omega=e['next_omega']
            if a!=e:fails.append(dict(stage='chained_rpm',aircraft=name,i=i,actual=a,expected=e))
    report=dict(binary_sha256=m.sha,entry='0x1019f19e0',stop='normal return',counts=counts,synthetic_nozzle_caps=caps,failures=fails,
                limitations='Complete function without hooks. Selected forward nozzles and statically prepared properties; no upstream fuel, health, thermal or command owner. Auxiliary VTOL/reverse states are explicit test injections; configured controls are absent on both aircraft. Chained cases independently propagate native and port RPM.')
    Path('analysis/jet-nozzle-validation.json').write_text(json.dumps(report,indent=2));print(counts,'FAILURES',len(fails));print(json.dumps(fails[:3],indent=2))
    if fails:raise SystemExit(1)

if __name__=='__main__':main()
