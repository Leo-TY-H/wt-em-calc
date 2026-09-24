"""Native-code checks at solved EM points, plus independent closure checks."""
import json
import math
from pathlib import Path

import numpy as np
from scipy.optimize import brentq
from em_solver import TrimSolver, settings, ROOT
from em_plot import enrich
from verify_coupled_airborne import CoupledAirborne
from verify_engine_owner import EngineOwnerMachine
from verify_primary_controls import Controls
from engine_supply import healthy_running_owner_step
from primary_controls import sensitivity_parameters
from air_state import world_to_body_air
from kinematics import airborne_step, altitude_velocity_correction
from component_assembly import f32
from body_dynamics import G


def main():
    machine=CoupledAirborne();engine=EngineOwnerMachine();controls=Controls()
    failures=[];counts={};examples=[]
    def check(stage,actual,expected,**where):
        counts[stage]=counts.get(stage,0)+1
        if actual!=expected:failures.append(dict(stage=stage,actual=actual,expected=expected,**where))
    def bound(stage,value,limit,**where):
        counts[stage]=counts.get(stage,0)+1
        if abs(value)>limit:failures.append(dict(stage=stage,value=value,limit=limit,**where))
    for name in ['f_16a_block_15_adf','saab_jas39c']:
        solver=TrimSolver(name,settings(dict(altitude_m=1000.,fuel_percent=50.)));root_cache={}
        def at_load(n):
            if n not in root_cache:root_cache[n]=solver.solve(900.,n,detailed=True)
            p=root_cache[n]
            if not p['valid']:raise AssertionError('Required root fixture failed trim: '+str(p['reasons']))
            return p['ps_mps']
        root=brentq(at_load,6.,10.,xtol=1e-5)
        cases=[solver.solve(900.,1.,detailed=True),solver.solve(900.,6.,detailed=True),root_cache[root],
               solver.solve(1158.3333333333333,3.4444444444444446,detailed=True)]
        if name=='f_16a_block_15_adf':
            # Saved failed continuation starts exercise the rounding-branch
            # refinement, followed by independent original-code comparison.
            for load,initial in [
                (7.722222222222223,[4.7324582896904515,82.5972657973602,.0010466734190749552,.27521763581131947,-.003756699750830731]),
                (8.944444444444445,[5.756101329149784,83.61254456942248,.0012489731777903757,.35089483776078495,-.0037584337983110434])]:
                cases.append(solver.solve(1158.3333333333333,load,initial,detailed=True))
        for index,p in enumerate(cases):
            detail=p['_detail'];geom=detail['geometry'];aero=detail['result'];allocation=detail['allocation']
            where=dict(aircraft=name,case=index,load=p['load_g'])
            speed=p['speed_kmh']/3.6
            av,ac=machine.air(dict(position=[0.,solver.config['altitude_m'],0.],quaternion=geom['quaternion'],velocity=[speed,0.,0.]),[0.]*3,[0.]*3)
            check('native_air_velocity',av,detail['velocity'],**where)
            check('native_air_cache',ac,[aero['air'][k] for k in ['alpha','beta','tas','speed_squared','mach','ias_u']],**where)
            check('trim_valid',p['valid'],True,**where)
            bound('angular_closure',p['angular_error_rad_s2'],5e-5,**where)
            bound('force_closure',p['force_error_g'],2e-4,**where)
            bound('history_closure',p['history_error'],2e-4,**where)
            bound('turn_heading',p['actual_turn_dps']-p['turn_dps'],.005,**where)
            if index==2:bound('sustained_energy',p['ps_mps'],.02,**where)
            raw=list(allocation['commands'])
            if solver.controls['invert_elevator']:raw[1]=-raw[1]
            response=sensitivity_parameters([.5]*3)
            c=controls.actuator(solver.controls,allocation['sticks'],allocation['trim'],allocation['trim'],raw,
                allocation['ranges'],solver.dt,response['time_constants'],response['linear_rates'],[True]*3,1.)
            delivered=controls.delivery(solver.controls,c['commands'],allocation['commands'],allocation['ranges'],solver.dt)
            check('native_control_fixed_point',delivered,allocation['commands'],**where)
            forces=[];moments=[];powers=[]
            for phase,state in enumerate(solver.engine.states):
                e=solver.engine
                args=(e.ep,e.jet,e.nozzle,e.fp,state,detail['velocity'],solver.config['altitude_m'],solver.mass['cog'],
                      solver.dt,183193,solver.mass['fuel_mass'],e.accumulator)
                native_engine=engine.owner(*args,ias_u=aero['air']['ias_u'])
                predicted_engine=healthy_running_owner_step(*args,ias_u=aero['air']['ias_u'])
                check('native_engine_force',native_engine['aggregate_force'],predicted_engine['aggregate_force'],**where)
                forces.append(native_engine['aggregate_force']);moments.append(native_engine['aggregate_moment'])
                machine.initial_position=[0.,solver.config['altitude_m'],0.];machine.initial_quaternion=geom['quaternion']
                machine.world_velocity=[speed,0.,0.];machine.collision_radius=10.;machine.tick=10;machine.flex=[[0.,0.],[0.,0.]]
                an=machine.call(solver.model,detail['velocity'],geom['omega'].tolist(),solver.mass,allocation['commands'],
                    solver.config['altitude_m'],solver.dt,detail['history_input'],throttle=solver.config['throttle'],full_update=True)
                check('native_component_forces',an['forces'],{k:aero['component_forces'][k] for k in an['forces']},**where)
                check('native_component_points',an['points'],{k:aero['component_points'][k] for k in an['points']},**where)
                check('native_aero_moment',an['moment'],aero['raw_aero_moment'],**where)
                assembly=machine.extend(native_engine['aggregate_force'],native_engine['aggregate_moment'],[0.]*3,[0.]*3,
                                        1.,solver.fm.get('ExtThrustBaseMult',1.),solver.dt)
                machine.integrate();native=machine.finish()
                expected=airborne_step(machine.initial_position,machine.world_velocity,geom['quaternion'],aero['omega_for_flow'],
                    assembly['force'],assembly['moment'],solver.mass['mass'],solver.mass['inertia'],solver.dt)
                expected['velocity'][1]=altitude_velocity_correction(expected['position'][1],expected['velocity'][1],solver.dt)
                for field in ['position','velocity','omega','quaternion','angular_acceleration','world_acceleration']:
                    check('native_'+field,native[field],expected[field],**where)
                before_up=world_to_body_air(geom['quaternion'],[0.,1.,0.]);after_up=world_to_body_air(native['quaternion'],[0.,1.,0.])
                bound('native_bank_attitude',max(abs(a-b) for a,b in zip(before_up,after_up)),5e-6,**where)
                bound('native_rate_closure',max(abs(a-b) for a,b in zip(native['omega'],geom['omega']))/solver.dt,6e-5,**where)
                ps=((native['position'][1]-solver.config['altitude_m'])+
                    (sum(v*v for v in native['velocity'])-speed**2)/(2*float(G)))/solver.dt
                powers.append(ps)
            check('native_mean_engine_force',np.mean(forces,axis=0).tolist(),aero['engine_force'],**where)
            check('native_mean_engine_moment',np.mean(moments,axis=0).tolist(),aero['engine_moment'],**where)
            bound('native_mean_energy',float(np.mean(powers))-p['ps_mps'],.001,**where)
            if index==2:bound('native_sustained_energy',float(np.mean(powers)),.03,**where)
            examples.append({**where,**{k:p[k] for k in ['speed_kmh','turn_dps','ps_mps','ps_continuous_mps','alpha_deg','trim','force_error_g','angular_error_rad_s2']}})
        fixed=TrimSolver(name,settings(dict(altitude_m=4500.,fuel_percent=50.,throttle=1.,afterburner=False,trim_mode='fixed')))
        point=fixed.solve(1100.,3.)
        check('fixed_trim_valid',point['valid'],True,aircraft=name)
        check('fixed_trim_preserved',point['trim'],[0.,0.,0.],aircraft=name)
    # Validation must reject NaN, unknown aircraft, backwards ranges and invalid
    # array sizes before they reach a numerical worker.
    for bad in [dict(altitude_m=float('nan')),dict(aircraft=['../../aces']),dict(speed_min_kmh=1800.,speed_max_kmh=900.),
                dict(speed_samples=7.5),dict(fixed_trim=[0.,0.]),dict(afterburner='yes')]:
        try:settings(bad);failures.append(dict(stage='bad_input_accepted',input=str(bad)))
        except ValueError:counts['input_rejected']=counts.get('input_rejected',0)+1
    report=dict(binary_sha256=machine.sha,counts=counts,examples=examples,failures=failures,
                limitations='Ten solved points including independently refined Ps=0 for both aircraft at1000m/50%fuel/AB, transonic derivative regressions and two F-16 rounding-branch continuation fixtures. Complete original air-cache producer, control, engine parent and aerodynamic/body normal return checked phase by phase. Frozen fuel/health and documented existing native harness hooks. Fixed-trim/dry checks use reconstructed equations. No live-flight comparison, no claim of full envelope stability or exhaustive branch coverage. Zero-Ps refers to native one-step energy, averaged across engine phases.')
    (ROOT/'analysis/em-solver-validation.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print('COUNTS',counts,'FAILURES',len(failures));print(json.dumps(failures[:4],indent=2))
    if failures:raise SystemExit(1)


if __name__=='__main__':main()
