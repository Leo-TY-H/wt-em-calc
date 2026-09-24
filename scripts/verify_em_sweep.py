"""Fixed sweep integration, physical speed edges and catalog propulsion checks."""
import json,math,time
from pathlib import Path
import numpy as np
from em_solver import AIRCRAFT,TrimSolver,settings
from em_sampling import sweep_speed_intervals
from verify_general_nozzles import Nozzles
from engine_supply import available_fuel,mechanical_multiplier
from component_assembly import f32,mul
from wing_sweep import available,schedule
from jet_catalog import load
from air_state import speed_of_sound


def main():
    start=time.monotonic();failures=[];points=[];edges=[];engines=[];native=Nozzles()
    for name in ['f_14a_early','mig_23m','f_111a','tornado_f3']:
        for sweep in [25.,50.,75.]:
            s=TrimSolver(name,dict(aircraft=[name],sweep_percent=sweep));p=s.solve(850.,3.)
            points.append(dict(id=name,sweep=sweep,ps=p['ps_mps'],valid=p['valid'],reasons=p['reasons']))
            if not p['valid']:failures.append(dict(stage='sweep_point',**points[-1]))
    s=TrimSolver('f_16a_block_15_adf',dict(sweep_percent=0.));p=s.solve(850.,3.)
    other=TrimSolver(s.name,dict(sweep_percent=100.));q=other.solve(850.,3.,p['solution'])
    if (p['force_n'],p['moment_nm'],p['ps_mps'])!=(q['force_n'],q['moment_nm'],q['ps_mps']):
        failures.append(dict(stage='fixed_wing_changed'))
    for name,metadata in AIRCRAFT.items():
        if not metadata['supported'] or not metadata['has_sweep']:continue
        fm=load(name);rows=schedule(fm);scale=float(speed_of_sound(0.))*3.6
        for sweep in [0.,25.,50.,75.,100.]:
            cfg=settings(dict(aircraft=[name],sweep_percent=sweep));spans=sweep_speed_intervals(name,cfg)
            for a,b in spans:
                for speed in [a-.02,a+.02,b-.02,b+.02]:
                    if not cfg['speed_min_kmh']<speed<cfg['speed_max_kmh']:continue
                    lo,hi=available(fm,rows,speed/scale);reachable=lo-1e-7<=sweep/100.<=hi+1e-7
                    excluded=any(x<speed<y for x,y in spans)
                    if reachable==excluded:failures.append(dict(stage='schedule_edge',id=name,sweep=sweep,speed=speed))
            edges.append(dict(id=name,sweep=sweep,excluded=spans))
    # Angled/implicit nozzles, multiple instances, old dry jets, lift-engine
    # throttle gating and afterburning swing wings; compare complete native
    # scalar+nozzle execution for every retained settled-engine phase.
    for name in ['f_16a_block_15_adf','saab_jas39c','f_14a_early','mig_23m','f_111a',
                 'tornado_f3','f-4e','arado-234c-3','f-80','me-262a-1a','yak_141','javelin_fmk9']:
        for ab in [False,True]:
            solver=TrimSolver(name,dict(aircraft=[name],afterburner=ab,sweep_percent=50.))
            for speed in [150.,350.]:
                expected_force=[0.]*3;expected_moment=[0.]*3;phases=0
                for unit in solver.engine.units:
                    forces=[];moments=[];ias=mul(speed,f32(math.sqrt(f32(unit.rho/f32(1.225)))))
                    for state in unit.phase_states(speed):
                        health,_,_=mechanical_multiplier(unit.ep,state['omega'],state['health'],state['cylinders'],
                            state['mechanical'],0.,state['torque'],state['friction'],unit.dt,state['_seed'])
                        thrust=state.get('effective_throttle',solver.config['throttle'])
                        r=native.call(unit.jet,unit.nozzles,unit.rho,speed,state['omega'],
                            min(f32(thrust/unit.jet['throttle_scale']),1.),unit.dt,solver.mass['cog'],ias,
                            0.,0.,0.,0.,[0.]*3,False,[False]*3,afterburner=ab,health=health,
                            fuel_available=available_fuel(unit.fp,unit.system_fuel,unit.accumulator,1.,unit.dt),
                            shaft_fraction=mul(f32(state['omega']),unit.ep['inverse_omega']))
                        forces.append(r['force']);moments.append(r['moment']);phases+=1
                    for i in range(3):
                        expected_force[i]+=float(np.mean(forces,axis=0)[i]);expected_moment[i]+=float(np.mean(moments,axis=0)[i])
                actual=solver.engine.vectors(speed)
                if actual!=(expected_force,expected_moment):failures.append(dict(stage='engine_mean',id=name,ab=ab,speed=speed,actual=actual,expected=[expected_force,expected_moment]))
                engines.append(dict(id=name,ab=ab,body_speed_mps=speed,phases=phases))
    for value in [-1,101,float('nan'),'50',True]:
        try:settings(dict(sweep_percent=value));failures.append(dict(stage='invalid_sweep_accepted',value=str(value)))
        except ValueError:pass
    report=dict(points=points,sweep_intervals=edges,engines=engines,failures=failures,elapsed_s=time.monotonic()-start,
        limitations='Native engine checks share prepared properties and frozen-fuel/mechanical phase providers. Sweep knot interpolation has separate native evidence. This checks fixed sweep integration, not live flight or exhaustive equilibria.')
    Path('analysis/em-sweep-validation.json').write_text(json.dumps(report,indent=2));print(json.dumps(dict(points=len(points),schedules=len(edges),engine_cases=len(engines),phases=sum(r['phases'] for r in engines),failures=failures,elapsed_s=report['elapsed_s']),indent=2))
    if failures:raise SystemExit(1)


if __name__=='__main__':main()
