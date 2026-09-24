"""Performance invariants, branch checks and independent surface holdouts.

Tests exercise different authority assumptions, equilibrium starts and withheld
conditions. Existing verify_em_solver.py supplies the original-code oracle.
"""
import argparse,json,math
from pathlib import Path
import numpy as np
from scipy.interpolate import PchipInterpolator
from em_solver import TrimSolver,settings,ROOT
from em_sampling import column_values,speed_interpolate


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--surface');args=parser.parse_args()
    failures=[];invariance=[];branches=[];holdouts=[];near_limit_holdouts=[];boundaries=[];boundary_holdouts=[];gap_regressions=[]
    for name in ['f_16a_block_15_adf','saab_jas39c']:
        full=TrimSolver(name,settings(dict(aircraft=[name],altitude_m=1000.,fuel_percent=30.,max_load_g=15.)))
        reduced=TrimSolver(name,settings(dict(aircraft=[name],altitude_m=1000.,fuel_percent=30.,max_load_g=15.,trim_mode='fixed',fixed_trim=[0.,0.,0.])))
        for speed,load in [(600.,2.),(900.,4.),(1200.,6.),(1500.,8.)]:
            point=full.solve(speed,load,detailed=True)
            if not point['valid']:
                failures.append(dict(stage='allocation_fixture_invalid',aircraft=name,speed=speed,load=load,reasons=point['reasons']))
                continue
            original=point['_detail'];other=reduced.operating_point(speed/3.6,load,point['solution'])
            for key in ['force','stored_moment','component_forces','component_points']:
                if original['result'][key]!=other['result'][key]:failures.append(dict(stage='allocation_changed_physics',aircraft=name,key=key,speed=speed,load=load))
            if original['ps']!=other['ps']:failures.append(dict(stage='allocation_changed_ps',aircraft=name,speed=speed,load=load))
            different=reduced.solve(speed,load,point['solution'])
            invariance.append(dict(aircraft=name,speed_kmh=speed,load_g=load,exact_same_physics=original['ps']==other['ps'],
                                   full_valid=point['valid'],restricted_valid=different['valid'],restricted_reasons=different['reasons'],
                                   sep_difference_mps=different['ps_mps']-point['ps_mps']))
            if abs(different['ps_mps']-point['ps_mps'])>1e-10:failures.append(dict(stage='authority_changed_equilibrium',aircraft=name,speed=speed,load=load))
        branch_cases=[(700.,4.),(1000.,6.),(1158.33333333333,4.)]
        for speed,load in branch_cases:
            candidates=[]
            for alpha in [2.,10.,20.,28.]:
                p=full.solve(speed,load,[alpha,math.degrees(math.acos(1/load)),0.,0.,0.])
                if p['valid']:candidates.append(p)
            spread=max((p['ps_mps'] for p in candidates),default=0)-min((p['ps_mps'] for p in candidates),default=0)
            branches.append(dict(aircraft=name,speed_kmh=speed,load_g=load,valid_starts=len(candidates),sep_spread_mps=spread))
            if len(candidates)!=4 or spread>.08:failures.append(dict(stage='branch_dependence',aircraft=name,speed=speed,load=load,valid_starts=len(candidates),sep_spread_mps=spread))
    # This 50%-fuel case previously selected a balanced post-stall root when
    # continuation started at the neighboring stall boundary.
    regression=TrimSolver('f_16a_block_15_adf',settings(dict(altitude_m=1000.,fuel_percent=50.)))
    speed,load=678.255859375,9.68165778162856
    candidates=[regression.solve(speed,load,[alpha,math.degrees(math.acos(1/load)),0.,0.,0.])
                for alpha in [2.,20.,27.94,30.]]
    valid=[p for p in candidates if p['valid']]
    spread=max((p['ps_mps'] for p in valid),default=0)-min((p['ps_mps'] for p in valid),default=0)
    branches.append(dict(aircraft=regression.name,speed_kmh=speed,load_g=load,fuel_percent=50.,valid_starts=len(valid),sep_spread_mps=spread))
    if len(valid)!=4 or spread>.08:failures.append(dict(stage='prestall_branch_regression',valid_starts=len(valid),sep_spread_mps=spread,reasons=[p['reasons'] for p in candidates]))
    # A real high-speed control-power constraint: remove manual trim while
    # retaining the same aerodynamic equilibrium and isolate authority from
    # optional structural bounds. Forces/Ps stay identical; feasibility changes.
    cfg=dict(aircraft=['f_16a_block_15_adf'],altitude_m=1000.,fuel_percent=30.,max_load_g=20.,structural_limits=False)
    a=TrimSolver('f_16a_block_15_adf',settings(cfg));b=TrimSolver('f_16a_block_15_adf',settings(dict(cfg,trim_limit=0.)))
    p=a.solve(1400.,15.);q=b.solve(1400.,15.,p['solution'])
    if not p['valid'] or q['valid'] or 'control authority' not in q['reasons'] or p['ps_mps']!=q['ps_mps']:
        failures.append(dict(stage='authority_feasibility',full=p['reasons'],restricted=q['reasons'],sep_difference=q['ps_mps']-p['ps_mps']))
    invariance.append(dict(aircraft=a.name,speed_kmh=1400.,load_g=15.,exact_same_physics=p['force_n']==q['force_n'] and p['moment_nm']==q['moment_nm'],
                           full_valid=p['valid'],restricted_valid=q['valid'],restricted_reasons=q['reasons'],sep_difference_mps=q['ps_mps']-p['ps_mps']))
    if args.surface:
        data=json.loads(Path(args.surface).read_text());fractions=np.linspace(0.,1.,257)
        for aircraft in data['aircraft']:
            solver=TrimSolver(aircraft['id'],data['settings']);columns=aircraft['columns'];good=[c for c in columns if column_values(c,fractions) is not None]
            # Persist conditions that exposed reopened, unchecked intervals
            # and a maximum-lift fallback, independently of changing grids.
            if data['settings']['altitude_m']==0. and data['settings']['fuel_percent']==30. and data['settings']['afterburner'] and data['settings']['throttle']==1.1:
                cases={'f_16a_block_15_adf':[(264.4609375,2.105633751),(623.8359375,9.666018727515313),(623.8359375,9.740700129009651)],
                       'saab_jas39c':[(260.05859375,2.3),(260.05859375,2.38424779),(260.8896484375,2.4020596707),(265.3818359375,2.4595852897),
                                      (298.2421875,3.175),(298.2421875,3.18)]}
                for v,n in cases.get(aircraft['id'],[]):
                    predicted=float(speed_interpolate(columns,[v],[n])[0,0]);actual=solver.solve(v,n)
                    error=abs(actual['ps_mps']-predicted)
                    row=dict(aircraft=aircraft['id'],speed_kmh=v,load_g=n,valid=actual['valid'],error_mps=error if math.isfinite(error) else None)
                    gap_regressions.append(row)
                    if not actual['valid'] or not math.isfinite(error) or error>data['settings']['sep_tolerance_mps']:
                        failures.append(dict(stage='recovered_interval_regression',**row))
            # Off-knot deterministic points, kept entirely out of adaptive tests.
            selected=np.linspace(1,len(good)-3,min(20,len(good)-3),dtype=int)
            for k in selected:
                left,right=good[k:k+2];v=left['speed_kmh']*.63+right['speed_kmh']*.37
                speeds=[c['speed_kmh'] for c in good]
                caps=[c['boundary']['load_g'] for c in good];cap=float(PchipInterpolator(speeds,caps)(v))
                for fraction in [.23,.57,.86,.97]:
                    n=1.+fraction*(cap-1.);predicted=float(speed_interpolate(columns,[v],[n])[0,0])
                    if not math.isfinite(predicted):continue
                    candidates=[p for p in left['points'] if p['valid']];start=min(candidates,key=lambda p:abs(p['load_g']-n))['solution']
                    actual=solver.solve(v,n,start)
                    error=abs(actual['ps_mps']-predicted)
                    row=dict(aircraft=aircraft['id'],speed_kmh=v,load_g=n,fraction=fraction,valid=actual['valid'],error_mps=error)
                    holdouts.append(row)
                    if not actual['valid'] or error>max(.1,data['settings']['sep_tolerance_mps']):failures.append(dict(stage='surface_holdout',**row))
                # At the lower of the neighboring caps, fixed-load cubics used
                # to miss the steep maximum-lift slope by more than 10 m/s.
                shared_cap=min(left['boundary']['load_g'],right['boundary']['load_g'])
                for fraction in [.995,.9995,.99995,1.]:
                    n=1.+fraction*(shared_cap-1.);predicted=float(speed_interpolate(columns,[v],[n])[0,0])
                    if not math.isfinite(predicted):continue
                    actual=solver.solve(v,n,left['boundary']['solution'])
                    error=abs(actual['ps_mps']-predicted)
                    row=dict(aircraft=aircraft['id'],speed_kmh=v,load_g=n,fraction=fraction,valid=actual['valid'],error_mps=error)
                    near_limit_holdouts.append(row)
                    if not actual['valid'] or error>max(.1,data['settings']['sep_tolerance_mps']):failures.append(dict(stage='near_limit_holdout',**row))
            for c in good[::max(1,len(good)//10)]:
                b=c['boundary']
                if not b.get('envelope_limit'):continue
                n=b['load_g'];guess=list(b['solution']);guess[0]-=.4
                inside=solver.solve(c['speed_kmh'],max(1.,n-.01),guess)
                outside=solver.solve(c['speed_kmh'],n+.05,b['solution'])
                row=dict(aircraft=aircraft['id'],speed_kmh=c['speed_kmh'],limit=b['envelope_limit']['kind'],load_g=n,
                         inside_valid=inside['valid'],outside_valid=outside['valid'],outside_reasons=outside['reasons'])
                boundaries.append(row)
                if not inside['valid'] or outside['valid']:failures.append(dict(stage='boundary_probe',**row))
            # Independent speeds test the displayed limit between its solved
            # knots, including changes between active physical constraints.
            cap_curve=PchipInterpolator([c['speed_kmh'] for c in good],[c['boundary']['load_g'] for c in good])
            pairs=[(left,right) for left,right in zip(columns,columns[1:])
                   if left['boundary_status']=='verified limit' and right['boundary_status']=='verified limit']
            for k in np.linspace(0,len(pairs)-1,min(12,len(pairs)),dtype=int):
                left,right=pairs[k];v=.41*left['speed_kmh']+.59*right['speed_kmh'];predicted=float(cap_curve(v))
                guess=left['boundary']['solution']
                inside=solver.solve(v,max(1.,predicted-.1),guess)
                outside=solver.solve(v,predicted+.1,guess)
                exact=solver.boundary(v,inside,outside) if inside['valid'] else None
                error=abs(exact['load_g']-predicted) if exact else None
                row=dict(aircraft=aircraft['id'],speed_kmh=v,predicted_load_g=predicted,
                         solved_load_g=exact['load_g'] if exact else None,error_g=error,
                         limit=exact['envelope_limit']['kind'] if exact else None)
                boundary_holdouts.append(row)
                if exact is None or error>.025:failures.append(dict(stage='boundary_interpolation',**row))
    report=dict(allocation_invariance=invariance,multiple_starts=branches,surface_holdouts=holdouts,near_limit_holdouts=near_limit_holdouts,recovered_interval_regressions=gap_regressions,boundary_probes=boundaries,boundary_holdouts=boundary_holdouts,failures=failures)
    (ROOT/'analysis/em-performance-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print('INVARIANCE',len(invariance),'BRANCHES',len(branches),'HOLDOUTS',len(holdouts),'NEAR_LIMIT_HOLDOUTS',len(near_limit_holdouts),'BOUNDARIES',len(boundaries),'BOUNDARY_HOLDOUTS',len(boundary_holdouts),'FAILURES',len(failures))
    print(json.dumps(failures[:8],indent=2))
    if failures:raise SystemExit(1)
if __name__=='__main__':main()
