"""Original jet engine/aero/body replay at selected altitude-SEP states."""
import altitude_runtime
import json
import math
from pathlib import Path
import numpy as np
from altitude_envelope import LevelSampler, settings
from verify_coupled_airborne import CoupledAirborne
from verify_engine_owner import EngineOwnerMachine
from engine_supply import healthy_running_owner_step
from kinematics import airborne_step, altitude_velocity_correction
from body_dynamics import G

ROOT=Path(__file__).resolve().parents[1]


def main():
    from macho_scan import MachO
    from verify_polar_machine_code import EXPECTED_BINARY_SHA256
    # Old harness subclasses do not forward a binary argument. Select the
    # saved, hash-checked research executable in this test process only.
    binary=ROOT/'references/native'/('aces-'+EXPECTED_BINARY_SHA256)
    if binary.exists():MachO.__init__.__defaults__=(str(binary),)
    machine=CoupledAirborne();engine=EngineOwnerMachine()
    records=[];failures=[];comparisons=0
    def check(actual,expected,label):
        nonlocal comparisons
        comparisons+=1
        if actual!=expected:failures.append(dict(label=label,actual=actual,expected=expected))
    for name in ['f_16a_block_15_adf','saab_jas39c']:
        sampler=LevelSampler(settings(dict(aircraft=name,conditions={'instructor':False})))
        for height,speed in [(0.,900.),(8000.,1200.),(16000.,1200.),(18000.,1200.)]:
            point=sampler(speed,height)
            assert point['valid'],(name,height,point['reasons'])
            s=sampler.solver(height);v=s.point_value(point);geom=v['geometry'];aero=v['result'];powers=[]
            check(v['ps'],point['ps_mps'],'saved SEP replay')
            for state in s.engine.phase_states(v['velocity'][0]):
                e=s.engine
                args=(e.ep,e.jet,e.nozzle,e.fp,state,v['velocity'],height,s.mass['cog'],s.dt,
                      state['_seed'],s.mass['fuel_mass'],e.accumulator)
                actual_engine=engine.owner(*args,ias_u=aero['air']['ias_u'])
                expected_engine=healthy_running_owner_step(*args,ias_u=aero['air']['ias_u'])
                check(actual_engine['aggregate_force'],expected_engine['aggregate_force'],'original engine force')
                check(actual_engine['aggregate_moment'],expected_engine['aggregate_moment'],'original engine moment')
                machine.initial_position=[0.,height,0.];machine.initial_quaternion=geom['quaternion']
                machine.world_velocity=[speed/3.6,0.,0.];machine.collision_radius=10.;machine.tick=10;machine.flex=[[0.,0.],[0.,0.]]
                original=machine.call(s.model,v['velocity'],geom['omega'].tolist(),s.mass,v['allocation']['commands'],height,s.dt,
                                      v['history_input'],throttle=s.config['throttle'],full_update=True,
                                      ground_height=height-1e6,quaternion=geom['quaternion'])
                check(original['forces'],{k:aero['component_forces'][k] for k in original['forces']},'original component forces')
                check(original['points'],{k:aero['component_points'][k] for k in original['points']},'original application points')
                check(original['moment'],aero['raw_aero_moment'],'original aero moment')
                assembly=machine.extend(actual_engine['aggregate_force'],actual_engine['aggregate_moment'],[0.]*3,[0.]*3,
                                        1.,s.fm.get('ExtThrustBaseMult',1.),s.dt)
                machine.integrate();native=machine.finish()
                predicted=airborne_step(machine.initial_position,machine.world_velocity,geom['quaternion'],aero['omega_for_flow'],
                                        assembly['force'],assembly['moment'],s.mass['mass'],s.mass['inertia'],s.dt)
                predicted['velocity'][1]=altitude_velocity_correction(predicted['position'][1],predicted['velocity'][1],s.dt)
                for field in ['position','velocity','omega','quaternion','angular_acceleration','world_acceleration']:
                    check(native[field],predicted[field],'original body '+field)
                ps=((native['position'][1]-height)+(sum(x*x for x in native['velocity'])-(speed/3.6)**2)/(2*float(G)))/s.dt
                powers.append(ps)
            delta=float(np.mean(powers))-point['ps_mps']
            assert abs(delta)<.001,(name,height,delta)
            records.append(dict(aircraft=name,altitude_m=height,speed_kmh=speed,ps_mps=point['ps_mps'],native_sep_difference_mps=delta,
                                altitude_recovery=point.get('altitude_recovery'),phases=len(powers)))
            print(name,height,point['ps_mps'],delta,flush=True)
    report=dict(status='FAIL' if failures else 'PASS',comparisons=comparisons,records=records,failures=failures,
                binary_sha256=machine.sha,scope='Selected original engine/component/body steps from 0 to 18 km. Frozen fuel/health and existing scoped native harness hooks. Not live-flight validation or a catalog-wide guarantee.')
    out=ROOT/'analysis/altitude-envelope';out.mkdir(parents=True,exist_ok=True)
    (out/'native-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(report['status'],comparisons,flush=True)
    assert not failures,failures[:2]


if __name__=='__main__':main()
