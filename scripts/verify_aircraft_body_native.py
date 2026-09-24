"""Extend the contiguous aero execution through prepared engine/body assembly.

Engine aggregate vectors are explicit boundary inputs. No engine owner update,
contact response, attitude integration or kinematic feedback is claimed here.
"""
import json,random,math
from pathlib import Path
from verify_aircraft_native import AircraftNative,BASE,FRAME,OWNER,PARAM
from unicorn.x86_const import UC_X86_REG_RIP
from aircraft_model import prepare,evaluate
from mass_model import aircraft_properties,evaluate as mass_eval
from component_assembly import f32
from body_dynamics import gravity_body,compose_force,compose_moment,limit_total_moment,realistic_engine_scale,airborne_linear_acceleration

class AircraftBodyNative(AircraftNative):
    def extend(self,engine_force,engine_moment,external_force,external_moment,ext_thrust,ext_base,dt,engine_angular_momentum=(0.,0.,0.)):
        # The original aircraft function reads previously produced owner vectors.
        self.doubles(OWNER+0x25a48,engine_force);self.doubles(OWNER+0x25a60,engine_moment)
        self.doubles(OWNER+0x25b40,engine_angular_momentum)
        self.floats(BASE+0x1648,external_force);self.floats(BASE+0x1654,external_moment)
        self.floats(BASE+0x84e4,[ext_thrust]);self.floats(BASE+0x7dfc,[ext_base]);self.floats(BASE+0x4ea8,[dt])
        self.u.mem_write(BASE+0x846c,b'\x01')  # Already initialized collision state.
        self.qword(BASE+0x6ef0,BASE+0x6d000)  # Airborne interface, contact flag clear.
        self.u.emu_start(0x106c6331e,0x106c63e7f,count=200000)
        if self.u.reg_read(UC_X86_REG_RIP)!=0x106c63e7f:raise RuntimeError('Body boundary not reached')
        return dict(force=self.read(FRAME-0x4e0,2,'d')+self.read(FRAME-0x4d0,1,'d'),
                    with_gravity=self.read(BASE+0x6cc0,3,'d'),
                    raw_moment=self.read(FRAME-0x490,2,'d')+self.read(FRAME-0x480,1,'d'),
                    moment=self.read(FRAME-0x4f0,2,'d')+self.read(FRAME-0x510,1,'d'),
                    acceleration=self.read(BASE+0x8868,3,'d'))

def main():
    machine=AircraftBodyNative();rng=random.Random(16003920);failures=[];errors={};cases=0
    for name in ['f_16a_block_15_adf','saab_jas39c']:
        fm=json.loads(Path('references/fm-2.59.0.13/'+name+'.blkx').read_text());model=prepare(fm)
        for i in range(200):
            mass=mass_eval(aircraft_properties(fm),fuel_by_system=[f32(rng.uniform(100,2000))])
            v=[f32(rng.uniform(40,900)),f32(rng.uniform(-120,50)),f32(rng.uniform(-50,50))];w=[rng.uniform(-1,1) for _ in range(3)]
            commands=[f32(rng.uniform(-1,1)) for _ in range(3)];height=f32(rng.uniform(100,15000));dt=f32(1/60)
            hist=dict(wing_aoa=[0.,0.],body_angles=[0.,0.],wing_cl=[0.,0.],spin=0.)
            external=[f32(rng.uniform(-1e4,1e4)) for _ in range(3)];torque=[f32(rng.uniform(-1e5,1e5)) for _ in range(3)];ext=f32(rng.uniform(.5,1.5))
            args=[model,v,w,mass,commands,height,dt,hist]
            machine.call(*args);candidate=evaluate(*args,oil_radiator=0.,height_agl=height,external_force=external,external_moment=torque,ext_thrust_mult=ext)
            actual=machine.extend(candidate['engine_force'],candidate['engine_moment'],external,torque,ext,fm.get('ExtThrustBaseMult',1.),dt)
            gravity=gravity_body([0.,0.,0.,1.],mass['mass'])
            expected=dict(force=candidate['force'],with_gravity=[x+y for x,y in zip(candidate['force'],gravity)],
                          raw_moment=compose_moment(candidate['raw_aero_moment'],torque,candidate['engine_moment']),moment=candidate['stored_moment'])
            expected['acceleration']=airborne_linear_acceleration(candidate['force'],gravity,mass['mass'])
            bad=[]
            for key,x in actual.items():
                y=expected[key];errors[key]=max(errors.get(key,0.),max(abs(a-b) for a,b in zip(x,y)))
                if x!=y:bad.append(dict(field=key,actual=x,expected=y))
            if bad:failures.append(dict(aircraft=name,index=i,mismatches=bad))
            cases+=1
    report=dict(binary_sha256=machine.sha,cases=cases,span=['0x106c5c7d0','0x106c63e7f'],max_absolute_errors=errors,failing_cases=len(failures),failures=failures[:10],limitations='Original aerodynamic prologue through total moment cap with a pause at raw aero moment boundary to supply engine aggregate vectors and external state. Engine vectors come from separately verified equilibrium/nozzle port; this check does not independently verify engine production. Identity attitude, no contacts, intact full-real selected jets. No integration or complete live timestep.')
    Path('analysis/aircraft-body-native-validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k!='failures'},indent=2))
    if failures:print(json.dumps(failures[:2],indent=2));raise SystemExit(1)

if __name__=='__main__':main()
