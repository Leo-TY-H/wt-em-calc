"""Check stationary proposals against native frames and perturbed continuations."""
import copy,json,math
from pathlib import Path
import numpy as np
from em_solver import TrimSolver,settings,ROOT
from prop_steady import initial_state,settled_cycle
from prop_fixedpoint import propose
from propulsion_general import step
from propulsion_general_native import PropulsionGeneralNative
from verify_propulsion_general import differences
from verify_polar_machine_code import EXPECTED_BINARY_SHA256
from aircraft_upgrades_native import apply as apply_native_upgrades
import verify_aircraft_native
from macho_scan import MachO


def main():
    binary=ROOT/'references/native'/('aces-'+EXPECTED_BINARY_SHA256)
    verify_aircraft_native.MachO=lambda path=None:MachO(path or binary)
    cases=[('a7m2',111.69057615447734,0.),('a7m2',190.,0.),('a7m2',300.,0.),('p-38g',600.,0.),
           ('p-63a-10',790.,0.),('b_25j_1',450.,0.),('saab_j21a_1',450.,0.),
           ('he51b1',200.,0.),('p-47d-28',600.,6500.),('tu_4',450.,3000.)]
    rows=[]
    for name,speed,height in cases:
        solver=TrimSolver(name,settings(dict(aircraft=[name],altitude_m=height,instructor=False,torque_gyro=True)))
        p=solver.engine.properties;v=[speed/3.6,-speed/120.,0.];w=[0.,.06,-.10]
        if name=='a7m2' and speed<120.:
            data=json.loads((ROOT/'analysis/em-upstream-september23/phase-root-trial/a7m_sb/data.json').read_text())
            points=[p for p in data['aircraft'][0]['points'] if p['valid'] and p['load_g']==1.]
            alpha=math.radians(min(points,key=lambda p:abs(p['speed_kmh']-speed))['solution'][0])
            v=[speed/3.6*math.cos(alpha),-speed/3.6*math.sin(alpha),0.];w=[0.,0.,0.]
        controls=solver.engine.automatic_controls;cg=solver.mass['cog'];dt=solver.dt;nitro=solver.mass['nitro_mass']
        def advance(s):return step(p,s,v,height,w,cg,dt,s['seed'],nitro,torque_gyro=True)
        state=initial_state(p,v,height,nitro=nitro,**controls)
        proposal=None;age=0
        for end in (0,16,960,2400):
            for frame in range(age,end):state=advance(state)
            age=end;proposal=propose(p,state,v,height,w,cg,dt,nitro,True)
            if proposal:break
        if proposal is None:
            rows.append(dict(aircraft=name,speed_kmh=speed,proposal=False));print(rows[-1],flush=True);continue
        candidate,diagnostic=proposal
        settled=settled_cycle(p,v,height,w,cg,dt,nitro,controls,state=candidate,require_cycle=True,
            allow_stationary=True,allow_stationary_mean=True,torque_gyro=True,
            aircraft_residual_scales=(solver.weight,solver.mass['inertia'],True))
        assert settled['converged'],(name,'Proposed initializer failed ordinary convergence')
        native=PropulsionGeneralNative()
        native.configure(json.loads((ROOT/'references/jet-catalog/fm'/(name+'.blkx')).read_text()))
        apply_native_upgrades(native,name)
        assert json.loads(json.dumps(native.config))==p
        current=copy.deepcopy(candidate);history=[];checked=0
        for frame in range(768):
            expected=advance(current)
            if frame in (0,1,2,7,31,127,255,511,767):
                actual=native.step(current,velocity=v,height=height,body_omega=w,cg=cg,dt=dt,
                    seed=current['seed'],nitro=nitro,torque_gyro=True)
                diff=differences(actual,expected);assert not diff,(name,frame,diff);checked+=1
            current=expected
            if frame>=512:history.append(current['aggregate_force']+current['aggregate_moment']+current['engine_wash'])
        mean=np.mean(history,axis=0);perturbations=[]
        for sign in (-1.,1.):
            perturbed=copy.deepcopy(candidate)
            for t in perturbed['transmissions']:
                t['omega']*=1.+sign*1e-5;t['previous_omega']*=1.+sign*1e-5
            for prop in perturbed['propellers']:
                prop['pitch']+=sign*1e-5;prop['governor_pitch']+=sign*1e-5
            values=[]
            for frame in range(768):
                perturbed=advance(perturbed)
                if frame>=512:values.append(perturbed['aggregate_force']+perturbed['aggregate_moment']+perturbed['engine_wash'])
            delta=abs(np.mean(values,axis=0)-mean)
            force=float(np.linalg.norm(delta[:3])/solver.weight)
            angular=(delta[3:6]/solver.mass['inertia']).tolist()
            assert force<1e-4 and max(angular)<2.5e-5,(name,sign,force,angular)
            assert max(delta[6:])<.02,(name,sign,'wash',delta[6:])
            perturbations.append(dict(sign=sign,force_difference_g=force,angular_difference_rad_s2=angular,wash_difference_mps=delta[6:].tolist()))
        row=dict(aircraft=name,speed_kmh=speed,proposal=True,seed_age_frames=age,initializer=diagnostic,
            ordinary_certificate_seconds=settled['simulated_seconds'],native_frames_checked=checked,
            propagated_frames=768,perturbations=perturbations)
        rows.append(row);print(name,'PASS',checked,'native frames; two 768-frame perturbation continuations',flush=True)
    assert any(r['proposal'] and r['aircraft']=='a7m2' and r['speed_kmh']<120. for r in rows),'Slow stable governor was not exercised'
    report=dict(status='PASS',scope=__doc__,cases=rows,binary_sha256=EXPECTED_BINARY_SHA256)
    (ROOT/'analysis/em-upstream-september23/fixedpoint-validation.json').write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':main()
