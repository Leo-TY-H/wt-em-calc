"""Bounded reduced-aircraft physical checks and Python/native replay comparisons."""
import os
os.environ['WT_MISSILE_ENGINE']='python'
os.environ['WT_MISSILE_BACKEND']='reference'
import json
import math
from copy import deepcopy
from pathlib import Path
import time
from benchmark_output import OutputBudget
from missile_aircraft_profile import profile
from missile_aircraft_motion import forces, validate_profile
from missile_worker import simulate, STEP, add
from missile_live_motion import steered_motion, steered_advance, steered_point, dot
from verify_missile_live import Engine


def main():
    began=time.perf_counter(); out=Path('outputs/live-missile'); budget=OutputBudget(out,'aircraft-trajectory')
    report=dict(checks=[],cases=0,max_target_position_error_m=0.,max_target_speed_error_mps=0.,failures=[])
    engine=None
    try:
        profiles=[profile(name) for name in ['f_16a_block_15_adf','saab_jas39c']]
        assert profiles[0]['mass_kg']!=profiles[1]['mass_kg'] and profiles[0]['thrust']!=profiles[1]['thrust']
        for p in profiles:validate_profile(p)
        report['checks'].append('Distinct aircraft source data, mass and thrust; bounded profiles')
        p=deepcopy(profiles[0]); target=dict(position=[0.,5000.,0.],velocity=[250.,0.,0.],angles=[0.,0.,0.])
        control=dict(load_g=9.,aircraft_profile=p)
        m=steered_motion(target,control); straight=forces(m,m['p'],m['dynamic_velocity'],m['u'],1.)
        pulled=forces(m,m['p'],m['dynamic_velocity'],m['u'],9.)
        assert pulled['drag']>straight['drag'] and pulled['load_g']>straight['load_g']
        low=forces(m,m['p'],[30.,0.,0.],m['u'],9.)
        assert low['load_g']<pulled['load_g'] and low['load_g']<9.
        assert forces(m,m['p'],[250.,0.,0.],m['u'],-99.)['load_g']>=p['negative_g']-1e-10
        report['checks'].append('Lift saturation, induced drag and asymmetric structural limits')
        # With zero propulsion/drag/lift, the same integrator must follow ballistics
        # and conserve specific mechanical energy; no artificial speed retention.
        ballistic=deepcopy(p);ballistic['thrust']=[[0. for _ in row] for row in p['thrust']]
        ballistic['positive_g']=ballistic['negative_g']=0.
        ballistic['aero']=[[r[0],0.,0.,-1.,1.] for r in p['aero']]
        b=steered_motion(target,dict(load_g=9.,aircraft_profile=ballistic)); t=0.
        for _ in range(240):
            new=add(t,STEP);steered_advance(b,t,new,0,0,0);t=new
        energy0=.5*250.**2+9.81*5000.;energy=.5*dot(b['dynamic_velocity'],b['dynamic_velocity'])+9.81*b['p'][1]
        assert abs(energy-energy0)<1e-6
        assert abs(b['p'][1]-(5000.-.5*9.81*t*t))<1e-6
        report['checks'].append('Ballistic trajectory and energy conservation without forces')
        # Aircraft response, banked turn, numerical frame integrity and history interpolation.
        m=steered_motion(target,control);t=0.
        for i in range(480):
            new=add(t,STEP);steered_advance(m,t,new,0,1,1 if i<48 else 0)
            mid=steered_point(m,(t+new)/2)
            assert all(math.isfinite(x) for x in mid['position']+mid['velocity']+mid['quaternion'])
            assert abs(dot(m['f'],m['u']))<1e-10 and abs(dot(m['u'],m['u'])-1)<1e-10
            t=new
        assert abs(m['p'][2])>10. and abs(math.sqrt(dot(m['dynamic_velocity'],m['dynamic_velocity']))-250.)>1.
        report['checks'].append('Banked trajectory, changing speed, orthonormal frame and interpolated state')
        bad=deepcopy(p);bad['thrust'][0][0]=float('nan')
        try:validate_profile(bad);raise AssertionError('Nonfinite profile accepted')
        except ValueError:pass
        bad=deepcopy(p);bad['aero'][1][0]=bad['aero'][0][0]
        try:validate_profile(bad);raise AssertionError('Nonmonotonic profile accepted')
        except ValueError:pass
        report['checks'].append('Malformed profile rejection')
        for velocity,angles in [([0.,250.,0.],[0.,90.,0.]),([0.,-250.,0.],[0.,-90.,0.]),([250.,0.,0.],[0.,0.,180.])]:
            m=steered_motion(dict(target,velocity=velocity,angles=angles),control);t=0.
            for _ in range(240):
                new=add(t,STEP);steered_advance(m,t,new,0,1,1);t=new
                assert all(math.isfinite(x) for x in m['p']+m['dynamic_velocity']+m['u'])
                assert abs(dot(m['f'],m['u']))<1e-10 and abs(dot(m['u'],m['u'])-1)<1e-10
        report['checks'].append('Vertical up/down and inverted starts: 720 finite orthonormal steps')
        if os.environ.get('AIRCRAFT_PHYSICS_ONLY')=='1': return
        engine=Engine([str(Path('.native_missile_engine/missile-engine.exe').resolve()),'--mode','simulator','--batch'],
                      dict(os.environ,WT_MISSILE_TYPED='compare',WT_MISSILE_TYPED_LOG='1'))
        patterns=[[[240,0,0,0]],[[240,0,1,0]],[[96,0,0,1],[144,0,1,0]],[[96,0,-1,-1],[144,0,1,0]]]
        for p in profiles:
            for missile in ['us_aim9l_sidewinder','us_aim_120c_5']:
                for inputs in patterns:
                    request=dict(missile=missile,duration=5.,early_exit=False,
                                 launcher=dict(position=[0,5000,0],velocity=[300,0,0],angles=[0,0,0]),
                                 target=dict(target,position=[12000,5000,0]),
                                 target_control=dict(load_g=9.,aircraft_profile=p,inputs=inputs))
                    native=engine.request(request); reference=simulate(request,lambda p:None)
                    assert native['outcome']['code']==reference['outcome']['code']
                    assert len(native['trajectory'])==len(reference['trajectory'])
                    for a,b in zip(native['trajectory'],reference['trajectory']):
                        assert a['time_s']==b['time_s']
                        dp=math.dist(a['target'],b['target']);dv=abs(a['target_speed_mps']-b['target_speed_mps'])
                        report['max_target_position_error_m']=max(report['max_target_position_error_m'],dp)
                        report['max_target_speed_error_mps']=max(report['max_target_speed_error_mps'],dv)
                        assert dp<=.0001 and dv<=.0001
                    report['cases']+=1
                    print(json.dumps(dict(completed=report['cases'],aircraft=p['aircraft_id'],missile=missile)),flush=True)
        report['source_profiles']=[dict(id=p['aircraft_id'],sha256=p['source_sha256']) for p in profiles]
        edge=dict(request,target=dict(target,position=[12000.,5000.,0.],velocity=[0.,25.1,0.],angles=[0.,90.,0.]),
                  target_control=dict(load_g=9.,aircraft_profile=profiles[0],inputs=[]))
        # No propulsion makes the low-speed boundary unambiguous during a climb.
        edge['target_control']['aircraft_profile']=deepcopy(profiles[0])
        edge['target_control']['aircraft_profile']['thrust']=[[0. for _ in row] for row in profiles[0]['thrust']]
        result=engine.request(edge)
        assert result['outcome']['code']=='target_envelope'
        report['checks'].append('Native low-speed envelope termination')
        engine.close()
        report['typed_compare_count']=engine.compare_count;report['typed_compare_failures']=engine.compare_failures
        engine=None
        assert report['typed_compare_count']==17 and report['typed_compare_failures']==0,'Typed path differs from reference or fell back'
    except Exception as error:
        report['failures'].append(str(error));raise
    finally:
        if engine:engine.close()
        report['elapsed_s']=time.perf_counter()-began
        budget.json(out/'aircraft-validation.json',report);budget.finish()
        print(json.dumps(report))


if __name__=='__main__':main()
