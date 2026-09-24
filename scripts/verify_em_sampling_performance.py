"""Independent surface holdouts and unchanged-equation performance checks."""
import json,math,random,time
from pathlib import Path
import numpy as np
from em_solver import TrimSolver,settings,BACKEND
from em_sampling import sample_column,column_at_load,coordinate_load,load_coordinate
from instructor_protection import authority_factor_step,authority_factor_run,history_repeats

OUT=Path(__file__).resolve().parents[1]/'analysis/performance-pass'


def main():
    failures=[];recurrences=[];rows=[];rng=random.Random(2049)
    cases=[(factor,peak,reference,dt,budget)
           for factor in [.2,.30000001192092896,1.]
           for peak,reference in [(20.,20.),(20.0001,20.),(19.9999,20.),(22.,20.),(18.,20.)]
           for dt,budget in [(1/48.,0),(1/48.,1),(1/120.,137),(1/30.,10000)]]
    cases += [(rng.uniform(.2,1.),rng.uniform(17.,23.),20.,1/48.,2000000) for _ in range(12)]
    cases += [(.3,20.0002,20.,1/48.,2000000),(.3,19.9998,20.,1/48.,2000000)]
    for factor,peak,reference,dt,budget in cases:
        current=factor;previous=factor;count=0
        # Production values are native float32, including the initial factor.
        from component_assembly import f32
        current=previous=f32(factor)
        while count<budget:
            value=authority_factor_step(current,peak,21.,reference,dt,[f32(.0125),f32(.05)])
            if value==current:break
            previous,current=current,value;count+=1
        actual=authority_factor_run(f32(factor),peak,21.,reference,dt,[f32(.0125),f32(.05)],budget)
        if actual!=(current,previous,count):failures.append(dict(stage='rounded recurrence',expected=[current,previous,count],actual=actual))
        recurrences.append(dict(budget=budget,ticks=count,identical=actual==(current,previous,count)))
    repeats=[]
    for period in [2,3,4,8]:
        seed=[[rng.random() for _ in range(40)] for _ in range(period)]
        for perturbation in [0.,1e-7,1.9999e-6,2e-6,2.0001e-6,.1]:
            records=[(list(seed[i%period]),None,None) for i in range(3*period)]
            records[-period-1][0][-1]+=perturbation
            original=max(abs(a-b) for k in range(2*period) for a,b in zip(records[-1-k][0],records[-1-k-period][0]))<2e-6
            actual=history_repeats(records,period,2e-6)
            repeats.append(dict(period=period,perturbation=perturbation,identical=actual==original))
            if original!=actual:failures.append(dict(stage='cycle predicate',period=period,perturbation=perturbation))
    for mode,speed in [(False,700.),(True,700.),(False,1200.),(True,1200.)]:
        cfg=settings(dict(aircraft=['fa_18e_block_2'],instructor=mode))
        start=time.monotonic();column=sample_column(('fa_18e_block_2',json.dumps(cfg),speed,None))
        seconds=time.monotonic()-start;baseline=json.loads((OUT/f'baseline-{int(mode)}-{speed:g}.json').read_text())
        solver=TrimSolver('fa_18e_block_2',cfg)
        boundary_deltas={key:abs(column['boundary'][key]-baseline['boundary'][key])
                         for key in ['load_g','ps_mps','alpha_deg','turn_dps']}
        # The same equations may stop at a different float32-balanced point.
        # Demand changes well below the existing plot interpolation tolerance.
        for key,limit in dict(load_g=.001,ps_mps=.05,alpha_deg=.005,turn_dps=.005).items():
            if boundary_deltas[key]>limit:failures.append(dict(stage='boundary changed',mode=mode,speed=speed,field=key,delta=boundary_deltas[key]))
        unchanged=[]
        for p in baseline['points'][::max(1,len(baseline['points'])//12)]:
            v=solver.point_value(p)
            same=v['ps']==p['ps_mps'] and v['result']['force']==p['force_n'] and v['result']['stored_moment']==p['moment_nm']
            if not same:failures.append(dict(stage='equations changed',mode=mode,speed=speed,load=p['load_g']))
            unchanged.append(same)
        knots=[p for p in column['points'] if p['valid'] and p['surface_sample'] and p['load_g']<=column['boundary']['load_g']]
        holdouts=[];top=column['boundary']['load_g']
        for lo,hi in zip(knots,knots[1:]):
            for fraction in [.23,.73]:
                u=float(load_coordinate(lo['load_g'],top))*(1-fraction)+float(load_coordinate(hi['load_g'],top))*fraction
                n=float(coordinate_load(u,top))
                if not lo['load_g']<n<hi['load_g']:continue
                actual=solver.solve(speed,n,lo['solution'])
                predicted=float(column_at_load(column,[n])[0]);error=abs(actual['ps_mps']-predicted)
                if not actual['valid'] or not math.isfinite(error) or error>cfg['sep_tolerance_mps']:
                    failures.append(dict(stage='independent holdout',mode=mode,speed=speed,load=n,error=error,reasons=actual['reasons']))
                holdouts.append(dict(load_g=n,error_mps=error,valid=actual['valid']))
        if column['interior_failures'] or column['boundary_status']!='verified limit':
            failures.append(dict(stage='new gap',mode=mode,speed=speed))
        rows.append(dict(mode=mode,speed_kmh=speed,seconds=seconds,
            previous_points=len(baseline['points']),points=len(column['points']),surface_knots=len(knots),
            boundary_identical=column['boundary']['solution']==baseline['boundary']['solution'],
            boundary_deltas=boundary_deltas,
            original_equations_identical=all(unchanged),holdouts=holdouts,
            unresolved_load_intervals=column['unresolved_load_intervals']))
        print(mode,speed,'seconds',round(seconds,3),'points',len(column['points']),'holdouts',len(holdouts),'failures',len(failures),flush=True)
    report=dict(backend=BACKEND,recurrences=recurrences,cycle_predicates=repeats,rows=rows,failures=failures)
    (OUT/'sampling-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    assert not failures,failures[:10]


if __name__=='__main__':main()
