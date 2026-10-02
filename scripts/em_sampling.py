import math
import os
import time
from em_data import clone
from em_workers import process_pool,WORKERS
from collections import OrderedDict
from em_ordered_work import OrderedWork
from functools import lru_cache

import numpy as np
from scipy.interpolate import PchipInterpolator
from scipy.optimize import brentq
from em_roots import checked_root
from em_recovery import recover_equilibrium
from em_pitch_response import PHYSICAL_REASONS
from em_sideslip import recover_sideslip,recover_sideslip_boundary,needs_sideslip_search,recover_fixed_alpha,recover_scalar_sideslip,MAX_SIDESLIP_DEG


_COLUMN_CACHE=OrderedDict()
_AIRCRAFT_CACHE=OrderedDict()


def compute_cached(config,progress=None,cancelled=None,preview=None):
    import json
    from em_solver import aircraft_settings
    start=time.monotonic()
    if cancelled and cancelled():raise InterruptedError('Calculation cancelled')
    keys={name:json.dumps(aircraft_settings(config,name),sort_keys=True) for name in config['aircraft']}
    cached={name:_AIRCRAFT_CACHE[key] for name,key in keys.items() if key in _AIRCRAFT_CACHE}
    for name in cached:_AIRCRAFT_CACHE.move_to_end(keys[name])
    needed=[name for name in config['aircraft'] if name not in cached]
    def combine(data):
        rows={a['id']:a for a in data['aircraft']}
        rows.update({name:clone(entry[0]) for name,entry in cached.items()})
        output=dict(data,settings=config,aircraft=[rows[name] for name in config['aircraft']])
        for i,a in enumerate(output['aircraft']):a['color']=['#38c9d7','#ffa66b'][i]
        output['speeds_kmh']=sorted({c['speed_kmh'] for a in output['aircraft'] for c in a['columns']})
        output['elapsed_s']=time.monotonic()-start
        output['cached_aircraft']=list(cached)
        return output
    if needed:
        work=dict(config,aircraft=needed,aircraft_settings={n:config.get('aircraft_settings',{}).get(n,{}) for n in needed})
        fresh=compute_adaptive(work,progress,cancelled,
            (lambda data:preview(combine(data))) if preview else None)
        if cancelled and cancelled():raise InterruptedError('Calculation cancelled')
        metadata={k:v for k,v in fresh.items() if k not in ('aircraft','settings','speeds_kmh')}
        for a in fresh['aircraft']:
            _AIRCRAFT_CACHE[keys[a['id']]]=(clone(a),clone(metadata))
            if len(_AIRCRAFT_CACHE)>8:_AIRCRAFT_CACHE.popitem(last=False)
        return combine(fresh)
    data=clone(next(iter(cached.values()))[1])
    data.update(aircraft=[],workers=0,cached_columns=sum(len(entry[0]['columns']) for entry in cached.values()))
    if progress:progress(dict(done=len(cached),total=len(cached),phase='Reusing completed aircraft calculations',elapsed_s=time.monotonic()-start))
    return combine(data)


def outline_column(column):
    keys=('speed_kmh','boundary','lower_boundary','boundary_status','boundary_reason','load_search_limit','aircraft_search_region','mach_branch','discontinuity')
    return dict({k:column.get(k) for k in keys},
                points=[p for p in column['points'] if p['load_g']==1.])


def load_coordinate(load,top):


    u=np.minimum(1.,(np.asarray(load)-1.)/(top-1.))
    return u/(1.+np.sqrt(np.maximum(0.,1.-u)))


def coordinate_load(value,top):
    return 1.+(top-1.)*(2*np.asarray(value)-np.asarray(value)**2)


def speed_check_position(low,high):
    if low<=0. or high<=1.25*low:return (low+high)*.5
    speed=low*math.sqrt(2./(1.+(low/high)**2)) if low>0. else (low+high)*.5
    return min(low+.7*(high-low),max(low+.3*(high-low),speed))


def interpolation_knots(x,y):
    x=np.asarray(x);y=np.asarray(y)
    if np.any(np.diff(x)<0):raise ValueError('Interpolation knots are out of order')
    keep=np.r_[np.diff(x)>0,True]
    return x[keep],y[keep]


def sweep_speed_intervals(name,config):
    from aircraft_catalog import load
    from wing_sweep import prepare,schedule,available
    from air_state import speed_of_sound
    fm=load(name)
    if len(prepare(fm))<2:return []
    rows=schedule(fm);position=config['sweep_percent']/100.
    scale=float(speed_of_sound(config['altitude_m']))*3.6
    low,high=config['speed_min_kmh']/scale,config['speed_max_kmh']/scale
    knots=[low,high]+[row[0] for row in rows if low<row[0]<high]
    for (xa,_,va),(xb,_,vb) in zip(rows,rows[1:]):
        for a,b in zip(va,vb):
            if a!=b:
                x=xa+(position-a)*(xb-xa)/(b-a)
                if max(low,xa)<x<min(high,xb):knots.append(x)
    knots=sorted(set(knots));excluded=[]
    for a,b in zip(knots,knots[1:]):
        lo,hi=available(fm,rows,(a+b)*.5)
        if lo-1e-7<=position<=hi+1e-7:continue
        if excluded and abs(excluded[-1][1]-a*scale)<1e-6:excluded[-1][1]=b*scale
        else:excluded.append([a*scale,b*scale])
    return excluded


@lru_cache(maxsize=8)
def worker_solver(name, config_json):
    import json
    from em_solver import TrimSolver
    solver=TrimSolver(name,json.loads(config_json));solver.chart_search=True
    return solver


def sample_with_history(worker,task,history):
    from em_workers import prepare_job
    prepare_job()
    solver=worker_solver(task[0],task[1])
    solver.chart_load_anchors=[p['_low_speed_cap_anchor'] for p in task[3]
        if isinstance(p,dict) and p.get('_low_speed_cap_anchor')] if isinstance(task[3],list) else []
    solver._sideslip_solvers={}
    solver.instructor_boundaries={}
    solver.instructor_trim_entries=dict(history) if solver.config['instructor'] else {}
    for key in ('_trim_predictor','_minimum_instructor_speed','_static_instructor_trim_cache'):
        solver.__dict__.pop(key,None)
    from numbers import Real
    from em_speed_limits import excluded_column
    if isinstance(task[2],Real):
        excluded=excluded_column(solver,task[2])
        if excluded is not None:return excluded,{}
    solver.__dict__.pop('_interior_curve',None)
    if solver.is_prop and solver.engine.automatic:
        candidates=[p for p in task[3] if p.get('_propulsion_seed')] if isinstance(task[3],list) else []
        solver.engine._task_seed=(min(candidates,key=lambda p:(abs(p['speed_kmh']-task[2]),p['load_g']))['_propulsion_seed']
                                  if candidates else None)
        solver.engine.reset_search()
        solver.__dict__.pop('_trim_predictor',None)


    from numbers import Real
    from em_column_work import bounded_column
    from em_speed_seam import check_interior
    from em_boundary_seam import check_boundary
    certificate=worker in (check_interior,check_boundary)
    column=(bounded_column(task,lambda:worker(task),certificate=certificate)
            if isinstance(task[2],Real) else worker(task))
    from em_load_limits import annotate
    from em_mach_events import annotate as annotate_mach
    return annotate_mach(solver,annotate(solver,column)),solver.instructor_trim_entries if solver.config['instructor'] else {}


def boundary_prediction(columns,speed):
    run=[]
    for column in sorted(columns.values(),key=lambda c:c['speed_kmh'])+[None]:
        if column is not None and column['boundary_status'] in ('verified limit','plot ceiling'):
            run.append(column);continue
        if len(run)>1 and run[0]['speed_kmh']<speed<run[-1]['speed_kmh']:
            n=float(PchipInterpolator([c['speed_kmh'] for c in run],[c['boundary']['load_g'] for c in run])(speed))
            return math.degrees(9.8100004196167*math.sqrt(max(0.,n*n-1.))/(speed/3.6))
        run=[]
    return None


def sample_level_endpoint(task):
    name,config_json,left,right=task
    solver=worker_solver(name,config_json)
    levels={c['speed_kmh']:next(p for p in c['points'] if p['load_g']==1. and p['valid'])
            for c in (left,right)}
    class EndpointClosed(Exception):
        def __init__(self,speed,point):self.speed=speed;self.point=point
    def residual(speed):
        fresh=speed not in levels
        if speed not in levels:
            near=min(levels.values(),key=lambda p:abs(p['speed_kmh']-speed))
            point=solver.solve(speed,1.,near['solution'],exhaustive=False)
            if not point['valid']:raise ValueError('Unresolved level trim inside endpoint bracket')
            levels[speed]=point
        value=levels[speed]['ps_mps']


        if fresh and abs(value)<.002:raise EndpointClosed(speed,levels[speed])
        return value
    try:
        speed=brentq(residual,left['speed_kmh'],right['speed_kmh'],xtol=1e-4,maxiter=24)
        point=levels[speed]
    except EndpointClosed as closed:
        speed,point=closed.speed,closed.point
    except (ValueError,RuntimeError):return None
    if abs(point['ps_mps'])>.002:return None
    near=min((left,right),key=lambda c:abs(c['speed_kmh']-speed))
    column=sample_column((name,config_json,speed,
        [point]+[p for p in near['points'] if p['valid']]))
    point['surface_sample']=True
    point['sustained_endpoint']='level flight'
    column['points']=[point]+[p for p in column['points'] if p['load_g']!=1.]


    column['sustained']=[point]+[p for p in column['sustained'] if p['load_g']>1.0001]
    column['level_endpoint_bracket_kmh']=[left['speed_kmh'],right['speed_kmh']]
    return column


def sample_stall_endpoint(task):
    from em_level_limit import level_stall_edge
    started=time.monotonic();name,config_json,left,right=task[:4]
    kind=task[4] if len(task)>4 else None
    solver=worker_solver(name,config_json)
    point=level_stall_edge(solver,left['speed_kmh'],right,next((p for p in left['points'] if p['load_g']==1.),None),kind=kind)
    if point is None and (kind or right['boundary'].get('envelope_limit',{}).get('kind'))=='Instructor pitch':


        point=level_stall_edge(solver,left['speed_kmh'],right,
            next((p for p in left['points'] if p['load_g']==1.),None),kind='stall')
    if point is None:return None
    return dict(speed_kmh=point['speed_kmh'],points=[point],boundary=point,lower_boundary=point,
        sustained=[point] if abs(point['ps_mps'])<.002 else [],load_checks=[],boundary_bracket_g=None,
        boundary_reason=point['envelope_limit']['kind'],boundary_status='verified limit',numerical_gap_brackets=[],interior_failures=[],
        unresolved_load_intervals=[],elapsed_s=time.monotonic()-started,mass=solver.mass,engine=solver.engine.summary,
        level_stall_endpoint=True,level_stall_bracket_kmh=[left['speed_kmh'],right['speed_kmh']])


def complete_level_edge(task):
    started=time.monotonic()
    name,config_json,speed,seed,left,right=task
    low=next((p for p in left['points'] if p['load_g']==1.),None)
    kind=None
    if low and low['converged'] and low['reasons']:
        reasons=set(low['reasons'])
        kind=('Instructor pitch' if reasons=={'Instructor pitch limit'} else
              'stall' if reasons=={'post-stall'} else None)
    if kind is None and right['boundary']['authority_margin']<.02:kind='control'
    if kind is None and right['boundary']['stall_margin_deg']<.15:kind='stall'
    if kind:
        endpoint=sample_stall_endpoint((name,config_json,left,right,kind))
        if endpoint:return endpoint
    if low and (not low['converged'] or low['reasons']==['reversed pitch response']):
        from em_level_fold import level_fold
        solver=worker_solver(name,config_json)
        point=level_fold(solver,left,right)
        if point:
            return dict(speed_kmh=point['speed_kmh'],points=[point],boundary=point,lower_boundary=point,
                sustained=[point] if abs(point['ps_mps'])<.002 else [],load_checks=[],boundary_bracket_g=None,
                boundary_reason=point['envelope_limit']['kind'],boundary_status='verified limit',numerical_gap_brackets=[],
                interior_failures=[],unresolved_load_intervals=[],elapsed_s=time.monotonic()-started,mass=solver.mass,
                engine=solver.engine.summary,level_stall_endpoint=True,
                level_stall_bracket_kmh=[left['speed_kmh'],right['speed_kmh']])


    return sample_column(task[:4])


def sample_edge(task):
    if task[-1]=='corner':
        from em_corner import sample_corner
        return sample_corner(task[:-1])
    return (sample_stall_endpoint if task[-1]=='stall' else sample_level_endpoint)(task[:-1])


def sample_level_edge_probe(task):
    name,config_json,speed,seed=task;started=time.monotonic()
    solver=worker_solver(name,config_json)
    from em_speed_limits import excluded_column
    excluded=excluded_column(solver,speed)
    if excluded is not None:return excluded
    initial=next((p['solution'] for p in seed or [] if p['valid'] and p['load_g']==1.),None)
    point=solve_level(solver,speed,initial)
    return dict(speed_kmh=speed,points=[point],boundary=point if point['valid'] else None,
        lower_boundary=point if point['valid'] else None,sustained=[],load_checks=[],
        boundary_bracket_g=None,boundary_reason=None,
        boundary_status='level probe feasible' if point['valid'] else 'no feasible samples',
        numerical_gap_brackets=[],interior_failures=[],unresolved_load_intervals=[],
        level_edge_probe=True,elapsed_s=time.monotonic()-started,mass=solver.mass,engine=solver.engine.summary)


def recheck_level_probe(task):
    column=sample_column(task[:4])
    column['earlier_level_probe']=task[4]
    return column


def solve_level(solver,speed,initial=None,exhaustive=False):
    point=solver.solve(speed,1.,initial,exhaustive=exhaustive)
    if point['converged'] or point.get('bounded_search_stationary'):return point


    guess=solver.initial_guess(speed/3.6,1.)
    guess[0]*=.5
    retry=solver.solve(speed,1.,guess,exhaustive=False)
    if retry['valid'] or retry['converged'] and not point['converged']:return retry
    return min((point,retry),key=lambda p:p['force_error_g']+p['angular_error_rad_s2'])


def continue_sideslip(solver,speed,load,neighbors):
    candidates=sorted((p for p in neighbors if p['valid'] and p.get('sideslip_attitude_deg',0.)),
                      key=lambda p:abs(p['load_g']-load))
    tried=set()
    for seed in candidates:
        angle=seed['sideslip_attitude_deg']
        if angle in tried:continue
        tried.add(angle)
        point=solver.at_sideslip(angle).solve(speed,load,seed['solution'],exhaustive=False)
        if point['valid'] and abs(point['sideslip_deg'])<=MAX_SIDESLIP_DEG:
            point['recovery_method']='balanced nearby-sideslip continuation'
            point['sideslip_recovery']=dict(max_abs_deg=MAX_SIDESLIP_DEG,
                policy='zero sideslip preferred; checked nearby flight-condition continuation')
            return point
        if len(tried)==2:break
    return None


def sample_initial_column(task):
    return sample_column(task)


def sample_level_started_column(task,provisional=False):
    name,config_json,speed,seed=task;solver=worker_solver(name,config_json)
    from em_speed_limits import excluded_column
    excluded=excluded_column(solver,speed)
    if excluded is not None:return excluded
    started=time.monotonic()


    point=(solver.solve(speed,1.,exhaustive=False,quick=True) if provisional else
           solve_level(solver,speed))
    if point['valid']:return sample_column((name,config_json,speed,[point]+(seed or [])))
    if provisional:
        return dict(speed_kmh=speed,points=[point],boundary=None,lower_boundary=None,sustained=[],load_checks=[],
            boundary_bracket_g=None,boundary_reason='Provisional level-flight search',boundary_status='no feasible samples',
            numerical_gap_brackets=[],interior_failures=[],unresolved_load_intervals=[],level_probe_only=True,
            elapsed_s=time.monotonic()-started,mass=solver.mass,engine=solver.engine.summary)

    return sample_column(task)


def sample_boundary_column(task):
    from em_column_work import bounded_column
    return bounded_column(task,lambda:_sample_checked_boundary_column(task))


def _sample_checked_boundary_column(task):
    started=time.monotonic()
    from em_aircraft_region import annotate
    solver=worker_solver(task[0],task[1])
    column=_sample_boundary_column(task)
    column=_check_roll_leveling_boundary(task,column,boundary_only=True)
    annotate(solver,column)
    column['elapsed_s']=time.monotonic()-started
    return column


def _check_roll_leveling_boundary(task,column,boundary_only):
    if column.get('_roll_leveling_switch_checked'):return column
    column['_roll_leveling_switch_checked']=True
    candidate=column.get('boundary')
    if (not candidate or not candidate.get('envelope_limit') or
            -12.<candidate['alpha_deg']<12.):return column
    name,config_json,speed,seed=task
    solver=worker_solver(name,config_json)
    if not solver.fm.get('RollLeveling',True):return column
    level=next((p for p in column.get('points',[]) if p['valid'] and p['load_g']==1.
                and p['speed_kmh']==speed),None)
    if level is None:
        level=next((p for p in seed or [] if p['valid'] and p['load_g']==1.
                    and p['speed_kmh']==speed),None)
    if level is None:level=solve_level(solver,speed)
    from em_branch_limit import first_roll_leveling_limit
    first=first_roll_leveling_limit(solver,speed,level,candidate)
    if first is None:return column
    first['_checked_probe_boundary']=True
    corrected=_sample_column((name,config_json,speed,[level,first]),boundary_only=boundary_only)
    corrected['_roll_leveling_switch_checked']=True
    corrected.setdefault('branch_search_points',[]).append(candidate)
    return corrected


def _sample_boundary_column(task):
    name,config_json,speed,seed=task;started=time.monotonic()
    from em_speed_limits import excluded_column
    excluded=excluded_column(worker_solver(name,config_json),speed)
    if excluded is not None:return excluded
    if os.environ.get('WT_EM_BOUNDARY_CONTINUE','1')=='1' and seed:
        from em_boundary_predictor import continue_limit,coupled_limit
        solver=worker_solver(name,config_json)
        continued=coupled_limit(solver,speed,seed) or continue_limit(solver,speed,seed)
        if continued:
            low,point,observations=continued
            return dict(speed_kmh=speed,points=[low,point],boundary=point,lower_boundary=low,
                sustained=[],load_checks=[],boundary_bracket_g=None,
                boundary_reason=point['envelope_limit']['kind'],boundary_status='verified limit',
                numerical_gap_brackets=[],interior_failures=[],unresolved_load_intervals=[],
                elapsed_s=time.monotonic()-started,mass=solver.mass,engine=solver.engine.summary,boundary_probe=True)
    if any(p.get('native_branch_search') for p in seed or []):
        from em_branch_limit import continue_endpoint
        solver=worker_solver(name,config_json)
        levels=[p for p in seed if p['valid'] and p['load_g']==1.]
        level=solver.solve(speed,1.,min(levels,key=lambda p:abs(p['speed_kmh']-speed))['solution'] if levels else None,exhaustive=False)
        point=continue_endpoint(solver,speed,seed,level)
        if point:
            return dict(speed_kmh=speed,points=[level,point],boundary=point,lower_boundary=level,
                sustained=[],load_checks=[],boundary_bracket_g=None,
                boundary_reason=point.get('envelope_limit',{}).get('kind'),
                boundary_status='verified limit' if point.get('envelope_limit') else 'unresolved numerical boundary',
                numerical_gap_brackets=[],interior_failures=[],unresolved_load_intervals=[],
                elapsed_s=time.monotonic()-started,mass=solver.mass,engine=solver.engine.summary,boundary_probe=True)
    limits=sorted((p for p in (seed or []) if p.get('envelope_limit')),key=lambda p:p['speed_kmh'])
    if len(limits)==2 and limits[0]['speed_kmh']<speed<limits[1]['speed_kmh']:
        left,right=limits
        if left['envelope_limit']['kind']==right['envelope_limit']['kind'] and left['envelope_limit']['kind']!='trim fold':
            solver=worker_solver(name,config_json)
            t=(speed-left['speed_kmh'])/(right['speed_kmh']-left['speed_kmh'])
            n=left['load_g']*(1-t)+right['load_g']*t
            guess=(np.array(left['solution'])*(1-t)+np.array(right['solution'])*t).tolist()
            inside=guess.copy();inside[0]-=.2
            low=solver.solve(speed,max(1.,n-max(.01,n*.01)),inside,exhaustive=False,quick=True)
            if not low['valid']:


                groups=[[q for q in seed if q['valid'] and q['speed_kmh']==v] for v in (left['speed_kmh'],right['speed_kmh'])]
                if all(groups):
                    floor=max(min(q['load_g'] for q in group) for group in groups)
                    load=max(1.,floor+.75*(n-floor))
                    states=[]
                    for group in groups:
                        group.sort(key=lambda q:q['load_g'])
                        states.append([float(np.interp(load,[q['load_g'] for q in group],[q['solution'][j] for q in group])) for j in range(5)])
                    inside=[a*(1-t)+b*t for a,b in zip(*states)]
                    inside[1]=math.degrees(math.acos(1./load))
                    low=solver.solve(speed,load,inside,exhaustive=False,quick=True)
            if low['valid']:
                hint=dict(left,load_g=n+max(.01,n*.02),solution=guess,valid=False,converged=False,reasons=[],
                    continuation_limit_kind=left['envelope_limit']['kind'],continuation_seed=dict(left,solution=guess,load_g=n,speed_kmh=speed))
                if left['envelope_limit']['kind'] in ('Instructor pitch','wing force'):


                    high=solver.solve(speed,hint['load_g'],guess,exhaustive=False,quick=True)
                    if high['converged'] and high['reasons'] and set(high['reasons'])<=PHYSICAL_REASONS:
                        hint=high
                point=solver.boundary(speed,low,hint,continuation=False,scalar_fallback=False,
                    candidate_kinds={left['envelope_limit']['kind']},max_iterations=8,trial_cycle_seconds=20.)
                if point and (solver.is_prop or solver.config['instructor'] or abs(point['solution'][3]-guess[3])<.3):
                    return dict(speed_kmh=speed,points=[low,point],boundary=point,lower_boundary=low,
                        sustained=[],load_checks=[],boundary_bracket_g=None,
                        boundary_reason=point['envelope_limit']['kind'],boundary_status='verified limit',
                        numerical_gap_brackets=[],interior_failures=[],unresolved_load_intervals=[],
                        elapsed_s=time.monotonic()-started,mass=solver.mass,engine=solver.engine.summary,boundary_probe=True)
    return sample_column(task,boundary_only=True)


def predict_limit(solver,speed,level,seed):
    if any(p.get('native_branch_search') for p in seed or []):
        from em_branch_limit import continue_endpoint
        point=continue_endpoint(solver,speed,seed,level)
        if point:return point
    from aircraft_model import condition_properties
    from air_state import cache
    from component_assembly import f32
    limits=[p for p in (seed or []) if p.get('envelope_limit')]
    if limits:
        near=min(limits,key=lambda p:abs(p['speed_kmh']-speed))
        estimate=near['load_g']*(speed/near['speed_kmh'])**2
        guess=list(near['solution']);kind=near['envelope_limit']['kind']
        below=[p for p in limits if p['speed_kmh']<speed and p['envelope_limit']['kind']==kind]
        above=[p for p in limits if p['speed_kmh']>speed and p['envelope_limit']['kind']==kind]
        if below and above:
            left=max(below,key=lambda p:p['speed_kmh']);right=min(above,key=lambda p:p['speed_kmh'])
            fraction=(speed-left['speed_kmh'])/(right['speed_kmh']-left['speed_kmh'])
            estimate=left['load_g']*(1-fraction)+right['load_g']*fraction
            guess=(np.asarray(left['solution'])*(1-fraction)+np.asarray(right['solution'])*fraction).tolist()
        elif kind=='wing force':estimate=near['load_g']

    else:
        air=cache([f32(speed/3.6),0.,0.],solver.config['altitude_m'])
        flaps=solver.flaps
        polar=condition_properties(solver.model,air['mach'],flaps)[1]
        estimate=.5*air['density']*(speed/3.6)**2*solver.model['geometry']['area']*polar['cyCritH']/solver.weight
        guess=list(level['solution']);guess[0]+=level['stall_margin_deg']-.002
        kind='Instructor pitch' if solver.config['instructor'] else 'stall'
    estimate=min(63.,max(1.001,estimate))
    near_level_prediction=estimate<1.8
    if not limits or solver.is_prop or solver.config['instructor']:
        guess[1]=math.degrees(math.acos(1/estimate))


    wing_cap=(1./max(level['wing_load_ratios']) if solver.is_prop and solver.engine.automatic and solver.config['structural_limits']
              and max(level['wing_load_ratios'])>0. else float('inf'))
    if not limits and estimate>wing_cap:kind='wing force'
    target=min(estimate,wing_cap) if kind=='wing force' and not limits else estimate
    high_load=min(64.,max(2.,min(estimate*1.2,wing_cap*1.05) if kind=='wing force' else estimate*1.2))
    if limits and kind=='wing force':high_load=min(64.,estimate+max(.02,estimate*.02))
    low=level
    if estimate>1.8:
        fractions=(.8,.65,.9) if solver.is_prop and solver.engine.automatic else (.65,)
        for fraction in fractions:
            interior=solver.solve(speed,max(1.1,target*fraction),exhaustive=False)
            if interior['valid']:
                low=interior;break
        else:return None
    if not limits and kind=='wing force' and low['load_g']>level['load_g']:


        guess=list(low['solution'])
        slope=(low['alpha_deg']-level['alpha_deg'])/(low['load_g']-level['load_g'])
        guess[0]+=slope*(high_load-low['load_g'])
        guess[1]=math.degrees(math.acos(1/high_load))
    high=dict(level,load_g=high_load,valid=False,converged=False,
        reasons=[],solution=guess,continuation_limit_kind=kind,
        wing_load_ratios=[v*high_load for v in level['wing_load_ratios']],
        continuation_seed=dict(level,load_g=target,solution=guess))
    if near_level_prediction:


        if solver.config['instructor'] or kind in ('control','wing force'):
            measured=solver.solve(speed,min(high_load,max(1.01,estimate)),guess,
                exhaustive=False,quick=True)
            if measured['converged'] and measured['reasons'] and set(measured['reasons'])<=PHYSICAL_REASONS:
                from em_constraint_bracket import refine
                limit=refine(solver,speed,level,measured)
                if limit:return limit


        from em_level_limit import near_level_boundary
        limit=near_level_boundary(solver,speed,level,high,max_nfev=12)
        if limit:return limit


        interior=solver.solve(speed,max(1.1,estimate*.85),exhaustive=False)
        if interior['valid']:
            return solver.boundary(speed,interior,high,continuation=False,
                candidate_kinds={'stall','control'},max_iterations=12,trial_cycle_seconds=20.)
        return None
    if kind=='wing force':


        measured_high=solver.solve(speed,high_load,guess,exhaustive=False)
        if measured_high['converged'] and set(measured_high['reasons'])=={'wing force limit'}:
            measured_high['continuation_limit_kind']='wing force'
            high=measured_high


    limit=solver.boundary(speed,low,high,continuation=False,
                          candidate_kinds={kind,'control'},max_iterations=8,trial_cycle_seconds=20.,scalar_fallback=False)
    if limit or kind!='wing force':return limit


    stall_guess=list(level['solution'])
    stall_guess[0]+=level['stall_margin_deg']-.002
    stall_guess[1]=math.degrees(math.acos(1/estimate))
    stall_high=dict(high,load_g=min(64.,max(2.,estimate*1.2)),solution=stall_guess,
                    continuation_limit_kind='stall')
    return solver.boundary(speed,low,stall_high,continuation=False,
                           candidate_kinds={'stall'},max_iterations=8,trial_cycle_seconds=20.)


def sample_column(task,boundary_only=False):
    from em_column_work import bounded_column
    return bounded_column(task,lambda:_sample_checked_column(task,boundary_only))


def _sample_checked_column(task,boundary_only=False):
    started=time.monotonic()
    from em_aircraft_region import annotate
    solver=worker_solver(task[0],task[1])
    column=_sample_column(task,boundary_only)
    column=_check_roll_leveling_boundary(task,column,boundary_only)
    annotate(solver,column)
    column['elapsed_s']=time.monotonic()-started
    return column


def _sample_column(task,boundary_only=False):
    started=time.monotonic()
    name,config_json,speed,seed=task
    solver=worker_solver(name,config_json);cfg=solver.config
    if solver.is_prop and solver.engine.automatic:


        solver.engine.reset_search()
    from em_speed_limits import speed_limits
    if not hasattr(solver,'plot_speed_limits'):
        solver.plot_speed_limits=speed_limits(solver.fm,cfg)
    redline=solver.plot_speed_limits
    if redline['enforced'] and speed>redline['sample_speed_kmh']:
        return dict(speed_kmh=float(speed),points=[],boundary=None,sustained=[],load_checks=[],
            boundary_bracket_g=None,boundary_reason=redline['kind'],boundary_status='speed limit',
            lower_boundary=None,numerical_gap_brackets=[],interior_failures=[],unresolved_load_intervals=[],
            elapsed_s=time.monotonic()-started,mass=solver.mass,engine=solver.engine.summary)
    samples={};tol=cfg['sep_tolerance_mps'];checks=[];recovery_attempted=set();discovery=None


    from em_load_limits import ceiling,description
    load_ceiling=ceiling(solver,speed)
    for point in seed or []:
        if (point.get('valid') and point.get('speed_kmh')==speed and point['load_g']<=load_ceiling+1e-10):
            samples[point['load_g']]=dict(point)
    def solve(n,initial=None,exhaustive=False,local=False):
        n=float(n)
        if n not in samples:
            local=local or n in recovery_attempted
            exhaustive=exhaustive and not local
            searched_exhaustively=exhaustive
            if initial is None:
                good=sorted([p for p in samples.values() if p['valid']],key=lambda p:p['load_g'])
                below=[p for p in good if p['load_g']<n];above=[p for p in good if p['load_g']>n]
                if below and above:
                    lo,hi=below[-1],above[0];t=(n-lo['load_g'])/(hi['load_g']-lo['load_g'])
                    if (boundary and boundary.get('envelope_limit',{}).get('kind') in ('stall','trim fold')
                            and 1.<=lo['load_g']<hi['load_g']<=boundary['load_g'] and boundary['load_g']>1.):


                        coords=load_coordinate(np.array([lo['load_g'],n,hi['load_g']]),boundary['load_g'])
                        if coords[2]>coords[0]:t=float((coords[1]-coords[0])/(coords[2]-coords[0]))
                    initial=(np.asarray(lo['solution'])*(1-t)+np.asarray(hi['solution'])*t).tolist()


                    nominal=lambda load:math.degrees(math.acos(1./max(1.,load)))
                    initial[1]=nominal(n)+(lo['solution'][1]-nominal(lo['load_g']))*(1-t)+(hi['solution'][1]-nominal(hi['load_g']))*t
                elif good:initial=min(good,key=lambda p:abs(p['load_g']-n))['solution']
                elif seed:
                    nearby=min(seed,key=lambda p:abs(p['load_g']-n))


                    if not (n==1. and nearby['stall_margin_deg']<.5
                            and abs(nearby['speed_kmh']-speed)>.005*speed):
                        initial=nearby['solution']
            p=(solve_level(solver,speed,initial,exhaustive=exhaustive) if n==1. else
               solver.solve(speed,n,initial,exhaustive=exhaustive,
                            quick=local and solver.is_prop and solver.engine.automatic))
            if n==1. and initial is not None and not p['valid']:


                independent=solve_level(solver,speed)
                if independent['valid'] or independent['force_error_g']<p['force_error_g']:p=independent
            if boundary and bottom and bottom['load_g']<n<boundary['load_g'] and not p['converged']:


                from em_parameter import recover_fixed_load_point
                recovered=recover_fixed_load_point(solver,speed,n,samples.values())
                if recovered is not None:p=recovered
            if not local and not p['converged']:
                from em_sideslip import recover_local_sideslip
                recovered=recover_local_sideslip(solver,speed,n,p)
                if recovered is not None:p=recovered
            if (solver.is_prop and solver.engine.automatic and not p['converged']
                    and p['force_error_g']<.2 and p['stall_margin_deg']>1.
                    and p['authority_margin']>.02 and max(p['wing_load_ratios'])<.98):


                for alpha_offset in (.1,.5,1.,-.1):
                    guess=list(p['solution']);guess[0]+=alpha_offset
                    trial=solver.solve(speed,n,guess,exhaustive=False)
                    if trial['valid']:
                        trial['recovery_method']='adjacent alpha-branch restart'
                        p=trial;break
                    if (trial['force_error_g']+trial['angular_error_rad_s2']<
                            p['force_error_g']+p['angular_error_rad_s2']):p=trial
            if boundary and bottom and bottom['load_g']<n<boundary['load_g'] and not p['converged']:
                correction=(continue_sideslip(solver,speed,n,[*samples.values(),*(seed or [])]) or
                            (recover_scalar_sideslip(solver,speed,n,p) if not local else None) or
                            (recover_fixed_alpha(solver,speed,n,p) if not local else None))
                if correction:p=correction


            if (cfg['instructor'] and boundary and bottom and bottom['load_g']<n<boundary['load_g']
                    and p['reasons'] and set(p['reasons'])<={'control authority','Instructor pitch limit'}
                    and p['authority_margin']>-.001 and (p.get('instructor') or {}).get('envelope_margin',0.)>-.001):
                p=solver.solve(speed,n,p['solution'],exhaustive=True,refine=True)
                searched_exhaustively=True


            if (not solver.is_prop and not local and not p['converged'] and
                    p['force_error_g']<.015 and p['angular_error_rad_s2']<.008 and
                    max(abs(x) for x in p['solution'][2:])<.98):
                p=solver.solve(speed,n,p['solution'],exhaustive=True)
                searched_exhaustively=True
            samples[n]=p
            if not local and needs_sideslip_search(p) and boundary and bottom and bottom['load_g']<n<boundary['load_g']:
                if not searched_exhaustively:
                    p=solver.solve(speed,n,p['solution'],exhaustive=True);samples[n]=p
                if needs_sideslip_search(p):
                    replacement=recover_sideslip(solver,speed,n,p,list(samples.values()))
                    if replacement:samples[n]=replacement
                if not samples[n]['valid']:recovery_attempted.add(n)
            if local and not samples[n]['valid']:
                samples[n]['recovery_search']='local continuation inside an unresolved interval'
        return samples[n]


    previous=None;boundary=None;bracket=None;bottom=None;previous_invalid=None
    certified=next((p for p in samples.values() if p.get('_checked_probe_boundary')),None)


    known_boundary=certified or next((p for p in samples.values() if p.get('envelope_limit')),None)
    known_floor=next((p for p in samples.values() if p.get('_checked_probe_lower_boundary')),None)
    if cfg['max_load_g'] is None:


        search_loads=[1.]
        while search_loads[-1]<load_ceiling:
            search_loads.append(min(load_ceiling,search_loads[-1]+max(1.,search_loads[-1]*.35)))
    else:search_loads=np.linspace(1.,load_ceiling,max(5,(cfg['load_samples']+1)//2))
    if known_boundary and known_floor and known_floor['load_g']<=known_boundary['load_g']:
        bottom=known_floor;boundary=known_boundary;search_loads=[]
    search_sequence=list(search_loads)
    for n in search_sequence:
        p=solve(n)
        if (not p['valid'] and previous is not None and solver.is_prop and
                set(p['reasons'])<= {'propulsion did not settle','propulsion cycle unresolved'} and
                p['stall_margin_deg']>1. and p['authority_margin']>.02):


            local=previous
            for _ in range(5):
                middle=(local['load_g']+n)*.5
                if middle-local['load_g']<.005:break
                step=solve(middle,local['solution'],local=True)
                if not step['valid']:break
                local=step
                del samples[float(n)]
                p=solve(n,local['solution'],local=True)
                if p['valid'] or not set(p['reasons'])<= {'propulsion did not settle','propulsion cycle unresolved'}:break
            if local['load_g']>previous['load_g']:previous=local
        if n==1. and not p['valid'] and 'sweep unavailable' not in p['reasons']:
            from em_discovery import interior_seeds
            discovery=interior_seeds(solver,speed,seed)
            for interior in discovery['points']:
                samples[interior['load_g']]=interior
            if discovery['points']:


                extra=[q['load_g'] for q in discovery['points']]


                search_sequence[1:]=sorted(set(search_sequence[1:]+extra))
            elif discovery['attempts'] and p['stall_margin_deg']<.15 and p['force_error_g']<.15 and p['angular_error_rad_s2']<.01:
                return dict(speed_kmh=float(speed),points=[p],boundary=None,lower_boundary=None,
                    sustained=[],load_checks=[],boundary_bracket_g=None,
                    boundary_reason='No feasible equilibrium found by the bounded independent searches',
                    boundary_status='no feasible samples',numerical_gap_brackets=[],interior_failures=[],
                    unresolved_load_intervals=[],elapsed_s=time.monotonic()-started,
                    mass=solver.mass,engine=solver.engine.summary,branch_discovery=discovery)
        if n==1. and p['valid'] and known_boundary:
            boundary=known_boundary;bottom=p;break
        if (n==1. and p['valid'] and cfg['max_load_g'] is None and
                (solver.is_prop and solver.engine.automatic or
                 not solver.is_prop and any(p.get('envelope_limit') for p in (seed or [])))):
            direct=predict_limit(solver,speed,p,seed)
            if direct is None and seed:
                direct=predict_limit(solver,speed,p,None)
            if direct:
                samples[direct['load_g']]=direct;bottom=p;boundary=direct
                break
        if (n==1. and p['valid'] and solver.is_prop and solver.engine.automatic and not cfg['instructor']
                and seed and max(q['load_g'] for q in seed)<1.8
                and any(q.get('envelope_limit',{}).get('kind')=='stall' for q in seed)):


            from em_level_limit import near_level_boundary
            previous_limit=max((q for q in seed if q.get('envelope_limit',{}).get('kind')=='stall'),key=lambda q:q['load_g'])
            hint=dict(p,load_g=2.,valid=False,converged=False,reasons=[],continuation_seed=previous_limit)
            direct=near_level_boundary(solver,speed,p,hint)
            if direct:
                samples[direct['load_g']]=direct;bottom=p;boundary=direct
                break


        if 'sweep unavailable' in p['reasons']:break
        if 'Instructor boundary unresolved' in p['reasons'] and not (discovery and discovery['points']):break
        if (not solver.is_prop and not p['converged'] and p['stall_margin_deg']>2. and
                max(abs(v) for v in p['commands'])<.95):


            retry=solver.solve(speed,float(n),exhaustive=True)
            if retry['converged'] or retry['force_error_g']<p['force_error_g']:
                samples[float(n)]=p=retry
        if not p['valid']:
            if n==1. and previous is None and seed:
                floors=sorted((q for q in seed if q.get('_checked_probe_lower_boundary')),key=lambda q:q['speed_kmh'])
                caps=sorted((q for q in seed if q.get('envelope_limit')),key=lambda q:q['speed_kmh'])
                if floors and caps and min(q['load_g'] for q in floors)>1.:
                    floor=float(np.interp(speed,[q['speed_kmh'] for q in floors],[q['load_g'] for q in floors]))
                    cap=float(np.interp(speed,[q['speed_kmh'] for q in caps],[q['load_g'] for q in caps]))
                    guess=[float(np.interp(speed,[q['speed_kmh'] for q in floors],[q['solution'][j] for q in floors])) for j in range(5)]
                    near=solve(min(cap,floor+max(.01,.01*(cap-floor))),guess,local=True)
                    if near['valid']:


                        left=solve(max(1.,floor-max(.01,.01*(cap-floor))),near['solution'],local=True)
                        if not left['valid'] and (left['converged'] or left.get('bounded_search_stationary')):
                            right=near
                            for _ in range(11):
                                if right['load_g']-left['load_g']<=.004:break
                                mid=solve((right['load_g']+left['load_g'])*.5,right['solution'],local=True)
                                if mid['valid']:right=mid
                                else:left=mid
                            bottom=right
                            direct=known_boundary or predict_limit(solver,speed,bottom,seed)
                            if direct and direct['load_g']>=bottom['load_g']:
                                samples[direct['load_g']]=direct;boundary=direct;break
                            previous=bottom;continue
            if previous is not None:
                if ((not p['converged'] or set(p['reasons'])<= {'propulsion did not settle','propulsion cycle unresolved'}) and
                    p['stall_margin_deg']>1. and
                    p['authority_margin']>.02 and max(p['wing_load_ratios'])<.98):


                    continue
                bracket=(previous,p);break
            previous_invalid=p
            continue
        if bottom is None:
            bottom=p
            if previous_invalid is not None:
                left,right=previous_invalid,p
                for _ in range(11):
                    if right['load_g']-left['load_g']<=.004:break
                    middle=solve((left['load_g']+right['load_g'])*.5,right['solution'])
                    if middle['valid']:right=middle
                    else:left=middle
                bottom=right
            if known_boundary and known_boundary['load_g']>=bottom['load_g']:
                boundary=known_boundary;break
        previous=boundary=p
    if bracket:
        low,high=bracket
        physical_reasons=PHYSICAL_REASONS
        def numerical_failure(point):
            return ((not point['converged'] or set(point['reasons'])<=
                     {'propulsion did not settle','propulsion cycle unresolved'}) and
                    point['stall_margin_deg']>1. and point['authority_margin']>.02 and
                    max(point['wing_load_ratios'])<.98)


        physical_rejection=high['converged'] and bool(high['reasons']) and set(high['reasons'])<=physical_reasons
        direct=(None if cfg['instructor'] and low['load_g']<1.05 and not high['converged'] else
                solver.boundary(speed,low,high)) if not solver.is_prop else (
                solver.boundary(speed,low,high) if physical_rejection else None)
        if direct:
            samples[direct['load_g']]=direct;low=direct
        if not direct:
            from em_branch_limit import branch_limit
            continued=branch_limit(solver,speed,low,high)
            if continued:
                samples[continued['load_g']]=continued;low=continued;direct=continued
        if not direct and not physical_rejection:
            from em_branch_limit import turning_limit
            continued=turning_limit(solver,speed,low,high)
            if continued:
                samples[continued['load_g']]=continued;low=continued;direct=continued
        if not direct and not solver.is_prop and not high['converged']:
            # Follow balanced turns in fine load steps before attempting a
            # limit from a distant seed. Elevator command can turn around on
            # this branch, so command-only continuation can miss the endpoint.
            step=min(.02,(high['load_g']-low['load_g'])*.5)
            for _ in range(32):
                if step<.0005 or high['load_g']-low['load_g']<=.0005:break
                n=min(low['load_g']+step,(low['load_g']+high['load_g'])*.5)
                guess=list(low['solution'])
                guess[1]+=math.degrees(math.acos(1./n)-math.acos(1./low['load_g']))
                # Revisit failed coarse probes with the nearby balanced seed.
                if n in samples and not samples[n]['valid']:del samples[n]
                p=solve(n,guess,local=True)
                if p['valid']:
                    low=p
                    step=min(.02,step*1.5)
                else:
                    high=p;step*=.5
            from em_pitch_limit import limit as pitch_limit
            direct=pitch_limit(solver,speed,low)
            if direct:samples[direct['load_g']]=direct;low=direct
        for _ in range(0 if direct else 11):
            if high['load_g']-low['load_g']<=.004:break
            p=solve((low['load_g']+high['load_g'])/2,low['solution'])
            if numerical_failure(p):
                if not solver.is_prop:
                    # Contract toward the balanced trim instead of skipping the
                    # valid interval below a failed midpoint. This is only a
                    # search bracket; failure does not certify a physical limit.
                    high=p
                    continue


                for probe_load in ((low['load_g']+p['load_g'])*.5,
                                   (p['load_g']+high['load_g'])*.5):
                    probe=solve(probe_load,low['solution'],local=True)
                    if probe['valid']:
                        low=probe;break
                    if not numerical_failure(probe):
                        high=probe;break
                else:
                    break
                continue
            if p['valid']:low=p
            else:high=p
        if not high['converged']:


            certified=[p for p in samples.values() if p['load_g']>low['load_g']
                and p['converged'] and p['reasons'] and set(p['reasons'])<=physical_reasons]
            if certified:high=min(certified,key=lambda p:p['load_g'])
        physical_rejection=high['converged'] and bool(high['reasons']) and set(high['reasons'])<=physical_reasons
        if not direct and (not solver.is_prop or physical_rejection):
            direct=solver.boundary(speed,low,high)
            if direct:samples[direct['load_g']]=direct;low=direct
        if not direct and solver.is_prop:


            nearby_kinds=set()
            if min(low['authority_margin'],high['authority_margin'])<.05:nearby_kinds.add('control')
            if min(low['stall_margin_deg'],high['stall_margin_deg'])<1.:nearby_kinds.add('stall')
            if cfg['structural_limits'] and max(max(low['wing_load_ratios']),max(high['wing_load_ratios']))>.98:
                nearby_kinds.add('wing force')
            if nearby_kinds:
                direct=solver.boundary(speed,low,high,continuation=False,
                    candidate_kinds=nearby_kinds,max_iterations=12)
                if direct:samples[direct['load_g']]=direct;low=direct
            if not direct:
                from em_branch_limit import turning_limit
                direct=turning_limit(solver,speed,low,high)
                if direct:samples[direct['load_g']]=direct;low=direct
        if not direct and solver.is_prop:


            nearby=[p for p in samples.values() if p['valid'] and
                    p['stall_margin_deg']<3. and p['authority_margin']>.02 and
                    (not cfg['structural_limits'] or max(p['wing_load_ratios'])<.98)]
            if nearby:
                near=max(nearby,key=lambda p:p['load_g'])
                rejected=sorted((p for p in samples.values() if not p['valid'] and
                                 p['load_g']>near['load_g']),key=lambda p:p['load_g'])
                if rejected:
                    direct=solver.boundary(speed,near,rejected[0],continuation=False,
                                           candidate_kinds={'stall'},max_iterations=8)
            if direct:samples[direct['load_g']]=direct;low=direct
        if not direct:
            from em_branch_limit import branch_limit
            known=max((p for p in samples.values() if p['valid']),key=lambda p:p['load_g'],default=low)
            continued=branch_limit(solver,speed,known,high)
            if continued:
                samples[continued['load_g']]=continued;low=continued;direct=continued
        if not direct and (not solver.is_prop or physical_rejection) and not low.get('envelope_limit'):
            direct=recover_sideslip_boundary(solver,speed,low,high)
            if direct:samples[direct['load_g']]=direct;low=direct
        boundary=low
        bracket=(low,high) if low['load_g']<high['load_g'] else None
    if (boundary and not boundary.get('envelope_limit') and not solver.is_prop and
            any(not p['valid'] and p['load_g']>boundary['load_g'] for p in samples.values())):


        from em_pitch_limit import limit as pitch_limit
        direct=pitch_limit(solver,speed,boundary)
        if direct:
            samples[direct['load_g']]=direct;boundary=direct
            if bracket:bracket=(direct,bracket[1]) if direct['load_g']<bracket[1]['load_g'] else None
    branch_selection=None;branch_search_points=[]
    if boundary and boundary['load_g']==1. and not boundary.get('envelope_limit'):
        from em_level_limit import level_constraint_point
        endpoint=level_constraint_point(boundary)
        if endpoint:
            boundary=endpoint;samples[1.]=endpoint;bottom=endpoint;bracket=None


    def outline_result():
        if bottom:bottom['_checked_probe_lower_boundary']=True


        return dict(speed_kmh=float(speed),points=sorted(samples.values(),key=lambda p:p['load_g']),
            boundary=boundary,lower_boundary=bottom,lower_boundary_searched=True,sustained=[],load_checks=[],
            boundary_bracket_g=[p['load_g'] for p in bracket] if bracket else None,
            boundary_reason=boundary.get('envelope_limit',{}).get('kind') if boundary else None,
            boundary_status='no feasible samples' if boundary is None else
                'verified limit' if boundary.get('envelope_limit') else 'unresolved numerical boundary',
            numerical_gap_brackets=[],interior_failures=[],unresolved_load_intervals=[],
            elapsed_s=time.monotonic()-started,mass=solver.mass,engine=solver.engine.summary,boundary_probe=True,
            branch_selection=branch_selection,branch_search_points=branch_search_points,branch_discovery=discovery)
    surface_loads=set();holdout_loads=set();pending=[]
    if boundary_only:


        return outline_result()
    coordinated_retries=set();alpha_holdouts={}


    from aircraft_model import condition_properties
    guide=boundary or bottom
    polar=condition_properties(solver.model,guide['mach'],guide['flaps_percent']/100.)[1] if guide else {}
    polar_angles=[polar[k]-solver.model['geometry']['incidence'] for k in ('aoaLineL','aoaLineH') if k in polar]
    from em_parameter import interior_point as solve_interior_point
    def alpha_point(solver,speed,left,right):
        lo,hi=left['load_g'],right['load_g']
        candidates=[p for n,p in samples.items() if p['valid'] and
                    lo+.25*(hi-lo)<=n<=lo+.75*(hi-lo)]
        if candidates:return min(candidates,key=lambda p:abs(p['load_g']-(lo+hi)*.5))
        return solve_interior_point(solver,speed,left,right)
    def restore_coordinated_neighbors():


        good=sorted((p for p in samples.values() if p['valid'] and
            abs(p.get('sideslip_attitude_deg',0.))<1e-8),key=lambda p:p['load_g'])
        for n,p in list(samples.items()):
            if not p['valid'] or abs(p.get('sideslip_attitude_deg',0.))<1e-8:continue
            lo=next((q for q in reversed(good) if q['load_g']<n),None)
            hi=next((q for q in good if q['load_g']>n),None)
            if lo is None or hi is None:continue
            key=(n,lo['load_g'],hi['load_g'])
            if key in coordinated_retries:continue
            coordinated_retries.add(key)
            t=(n-lo['load_g'])/(hi['load_g']-lo['load_g'])
            guess=[a*(1-t)+b*t for a,b in zip(lo['solution'],hi['solution'])]
            retry=solver.solve(speed,n,guess,exhaustive=False,refine=True)
            if retry['valid']:
                retry['recovery_method']='coordinated trim restored from verified neighbors'
                samples[n]=retry

    def refine_surface(phase):
        top=boundary['load_g'];remaining=[]
        for depth in range(7):
            restore_coordinated_neighbors()
            runs=[];run=[]
            for n in sorted(surface_loads):
                p=samples[n]
                if n>top:break
                if p['valid']:run.append(p)
                else:
                    if len(run)>1:runs.append(run)
                    run=[]
            if len(run)>1:runs.append(run)
            if not runs:return []
            insert=[];remaining=[]
            for good in runs:
                ux,uy=interpolation_knots(load_coordinate([p['load_g'] for p in good],top),
                    [[p['ps_mps'],p['alpha_deg']] for p in good])
                if len(ux)<2:continue
                from em_surface import prepare_curve
                curve=prepare_curve(good,top)


                coordinates=np.unique(np.concatenate([np.linspace(a,b,9) for a,b in zip(ux,ux[1:])]))
                values=curve(coordinates)
                target_loads=[]
                for level in cfg['sep_contour_levels_mps']:
                    for i in range(len(coordinates)-1):
                        a,b=values[i:i+2]
                        if not np.isfinite([a,b]).all() or not min(a,b)<=level<=max(a,b) or a==b:continue
                        target=brentq(lambda u:float(curve(u))-level,coordinates[i],coordinates[i+1],xtol=1e-12)
                        n=float(coordinate_load(target,top))
                        target_loads.append(n)
                        if n in surface_loads:continue
                        solve(n);holdout_loads.add(n)
                for left,right in zip(good,good[1:]):
                    lo,hi=left['load_g'],right['load_g']
                    u=float((load_coordinate(lo,top)+load_coordinate(hi,top))*.5)
                    mid=float(coordinate_load(u,top))
                    if not lo<mid<hi:continue


                    if (left.get('roll_leveling_branch')==right.get('roll_leveling_branch') and
                            any(lo<n<hi for n in target_loads)):continue
                    if left.get('roll_leveling_branch')==right.get('roll_leveling_branch'):
                        from em_accuracy import visible_error
                        values=curve(np.linspace(float(load_coordinate(lo,top)),float(load_coordinate(hi,top)),9))
                        if not visible_error(np.min(values),np.max(values),cfg):continue
                    key=(left['load_g'],right['load_g'])
                    if key not in alpha_holdouts:
                        alpha_holdouts[key]=alpha_point(solver,speed,left,right)
                    p=alpha_holdouts[key]
                    if p is not None:
                        mid=p['load_g'];u=float(load_coordinate(mid,top));samples[mid]=p
                    else:p=solve(mid)
                    holdout_loads.add(mid)
                    if not p['valid']:
                        continue


                    p['alpha_sample']=True
                    error=abs(p['ps_mps']-float(curve(u)))
                    alpha_error=0.
                    checks.append(dict(load_g=mid,error_mps=error,alpha_equivalent_error_mps=alpha_error,
                                       depth=depth,phase=phase))
                    transition=left.get('roll_leveling_branch')!=right.get('roll_leveling_branch')


                    extra_fractions=[]
                    angle_span=right['alpha_deg']-left['alpha_deg']
                    if angle_span>1e-5:
                        extra_fractions.extend((angle-left['alpha_deg'])/angle_span for angle in polar_angles
                            if left['alpha_deg']+.1*angle_span<angle<right['alpha_deg']-.1*angle_span)
                    delta_ps=right['ps_mps']-left['ps_mps']
                    if transition:extra_fractions.extend((.25,.75))
                    if extra_fractions:


                        from em_parameter import _alpha_point
                        for fraction in extra_fractions:
                            branch_key=(*key,fraction)
                            if branch_key not in alpha_holdouts:
                                alpha_holdouts[branch_key]=_alpha_point(solver,speed,left,right,fraction)
                            probe=alpha_holdouts[branch_key]
                            if probe is not None:
                                samples[probe['load_g']]=probe;holdout_loads.add(probe['load_g'])
                                probe['alpha_sample']=True


                    from em_accuracy import neighboring_loads,turn_tolerance,within_contour_band,visible_error
                    query=neighboring_loads(speed,np.array([mid]),turn_tolerance(tol))[0]
                    query=np.clip(query,good[0]['load_g'],good[-1]['load_g'])
                    geometric=bool(within_contour_band(p['ps_mps'],curve(load_coordinate(query,top)),tol))
                    refine=error>tol and not geometric and hi-lo>.0002 and visible_error(p['ps_mps'],float(curve(u)),cfg)
                    checks[-1]['within_contour_tolerance']=geometric
                    checks[-1]['native_transition']=transition
                    if refine:
                        insert.append(mid);remaining.append((lo,hi))


                from em_accuracy import neighboring_loads,turn_tolerance,within_contour_band,visible_error
                evidence=[p for n,p in samples.items() if n not in surface_loads and p['valid'] and
                          good[0]['load_g']<n<good[-1]['load_g']]
                if evidence:
                    ns=np.array([p['load_g'] for p in evidence]);actual=np.array([p['ps_mps'] for p in evidence])
                    error=abs(actual-curve(load_coordinate(ns,top)))
                    query=np.clip(neighboring_loads(speed,ns,turn_tolerance(tol)),good[0]['load_g'],good[-1]['load_g'])
                    geometry=within_contour_band(actual,curve(load_coordinate(query,top)),tol)
                    for i in np.flatnonzero((error>tol)&~geometry&visible_error(actual,curve(load_coordinate(ns,top)),cfg)):
                        n=float(ns[i]);insert.append(n)
                        k=int(np.searchsorted([p['load_g'] for p in good],n))
                        remaining.append((good[k-1]['load_g'],good[k]['load_g']))
            if not insert:return []


            if depth<6:surface_loads.update(insert)
        return remaining
    if boundary and boundary['load_g']>1.:
        top=boundary['load_g'];start=float(load_coordinate(bottom['load_g'],top))


        surface_loads.update((bottom['load_g'],top))
        middle=alpha_point(solver,speed,bottom,boundary)
        if middle is not None:
            samples[middle['load_g']]=middle;surface_loads.add(middle['load_g'])
            for left,right in ((bottom,middle),(middle,boundary)):
                point=alpha_point(solver,speed,left,right)
                if point is None:point=solve((left['load_g']+right['load_g'])*.5)
                samples[point['load_g']]=point;surface_loads.add(point['load_g'])
        else:
            for n in coordinate_load(np.linspace(start,1.,5),top):
                point=solve(float(n));surface_loads.add(point['load_g'])


        if top-bottom['load_g']>.05:
            from em_accuracy import visible_error
            ordered=sorted((samples[n] for n in surface_loads if samples[n]['valid']),key=lambda p:p['load_g'])
            for n in (bottom['load_g']+min(.1,.01*(top-bottom['load_g'])),top-.005*(top-bottom['load_g'])):
                pair=next(((a,b) for a,b in zip(ordered,ordered[1:]) if a['load_g']<=n<=b['load_g']),None)
                if pair and visible_error(pair[0]['ps_mps'],pair[1]['ps_mps'],cfg):solve(float(n))


        ordered=sorted(samples.values(),key=lambda p:p['load_g'])
        for i,point in enumerate(ordered):
            if not point['valid']:
                surface_loads.update(p['load_g'] for p in ordered[max(0,i-1):i+2])
        holdout_loads.update(set(samples)-surface_loads)
        pending=refine_surface('independent midpoint checks')


    recovery_candidates={n for n,p in samples.items() if not p['valid']}
    for n,p in list(samples.items()):
        if p.get('native_seam_marker'):continue
        if (boundary and bottom and bottom['load_g']<n<boundary['load_g'] and not p['valid']
                and (not p['converged'] or set(p['reasons'])<=
                     {'propulsion did not settle','propulsion cycle unresolved'})):
            from em_parameter import recover_fixed_load_point
            recovered=recover_fixed_load_point(solver,speed,n,samples.values())
            if recovered is not None:
                samples[n]=recovered
                continue
        if (boundary and bottom and bottom['load_g']<n<boundary['load_g'] and
                not p['valid'] and (not p['converged'] or 'post-stall' in p['reasons'])):
            del samples[n];solve(n,exhaustive=not solver.is_prop,local=solver.is_prop)
            if not samples[n]['valid'] and n not in recovery_attempted:
                retry=solver.solve(speed,n,exhaustive=not solver.is_prop)
                if retry['valid']:samples[n]=retry
                elif needs_sideslip_search(retry):
                    replacement=recover_sideslip(solver,speed,n,retry,list(samples.values()))
                    if replacement:samples[n]=replacement
                    else:recovery_attempted.add(n)


    for n,p in list(samples.items()):
        if p.get('native_seam_marker'):continue
        if (not solver.is_prop and boundary and bottom and
                bottom['load_g']<n<boundary['load_g'] and not p['converged']):
            recovered=recover_equilibrium(solver,speed,n,list(samples.values()))
            if recovered:
                recovered['recovery_method']='bank/control balance followed by alpha force closure'
                samples[n]=recovered


    if boundary and boundary['load_g']>1.:
        recovered={n for n in recovery_candidates if samples[n]['valid'] and n<=boundary['load_g']}
        if recovered:
            surface_loads.update(recovered)
            pending=refine_surface('recovered intervals')


    def failed_runs():
        runs=[];left=None;bad=[]
        for p in sorted(samples.values(),key=lambda p:p['load_g']):
            if not boundary or p['load_g']>boundary['load_g']:break
            if p['valid']:
                if left and bad:runs.append((left,bad,p))
                left=p;bad=[]
            elif left:bad.append(p)
        return runs
    gap_brackets=[]


    gap_edge_tolerance=(max(.003,(boundary['load_g']-bottom['load_g'])/1000.)
                        if solver.is_prop and boundary and bottom else .0002)
    gap_edge_steps=6 if solver.is_prop else 11
    for left,bad,right in failed_runs():
        if all(p.get('native_seam_marker') for p in bad):
            gap_brackets.append(dict(load_g=bad[0]['load_g'],valid_side_loads=[left['load_g'],right['load_g']],
                                     width_g=right['load_g']-left['load_g'],native_seam=True))
            continue
        edges=[]
        for good,failed in ((left,bad[0]),(right,bad[-1])):
            valid_load=good['load_g'];bad_load=failed['load_g']
            for _ in range(gap_edge_steps):
                if abs(valid_load-bad_load)<gap_edge_tolerance:break
                mid=(valid_load+bad_load)*.5;q=solve(mid,samples[valid_load]['solution'],local=True)
                if q['valid']:valid_load=mid
                else:bad_load=mid
            edges.append(valid_load)
        gap_brackets.append(dict(load_g=bad[len(bad)//2]['load_g'],valid_side_loads=edges,
                                 width_g=edges[1]-edges[0]))


    island_found=False
    for gap in gap_brackets:
        if gap['width_g']>.005 and not gap.get('native_seam'):
            for n in np.linspace(*gap['valid_side_loads'],5 if solver.is_prop else 17)[1:-1]:
                fresh=float(n) not in samples
                if solve(n,local=True)['valid'] and fresh:island_found=True
    for left,bad,right in (failed_runs() if island_found or not solver.is_prop else []):
        if all(p.get('native_seam_marker') for p in bad):continue
        for good,failed in [(left,bad[0]),(right,bad[-1])]:
            for _ in range(gap_edge_steps):
                if abs(good['load_g']-failed['load_g'])<gap_edge_tolerance:break
                p=solve((good['load_g']+failed['load_g'])*.5,good['solution'],local=True)
                if p['valid']:good=p
                else:failed=p
    gap_brackets=[dict(load_g=bad[len(bad)//2]['load_g'],
        valid_side_loads=[left['load_g'],right['load_g']],width_g=right['load_g']-left['load_g'],
        edge_brackets_g=[[left['load_g'],bad[0]['load_g']],[bad[-1]['load_g'],right['load_g']]],
        reasons=sorted(set(reason for p in bad for reason in p['reasons'])))
        for left,bad,right in failed_runs()]
    for left,bad,right in failed_runs():surface_loads.update((left['load_g'],right['load_g']))


    for n,p in samples.items():
        p['surface_sample']=n in surface_loads or not p['valid']
    valid=sorted((p for p in samples.values() if p['valid']),key=lambda p:p['load_g'])
    roots=[]
    for low,high in zip(valid,valid[1:]) if 0. in cfg['sep_contour_levels_mps'] else []:
        if low['ps_mps']*high['ps_mps']>0:continue
        def residual(n):
            t=(n-low['load_g'])/(high['load_g']-low['load_g'])
            guess=(np.asarray(low['solution'])*(1-t)+np.asarray(high['solution'])*t).tolist()
            p=solve(n,guess)
            if not p['valid']:raise ValueError('Root outside feasible branch')
            return p['ps_mps']
        try:
            n=checked_root(residual,low['load_g'],high['load_g'],residual_tolerance=.002,xtol=2e-5,maxiter=18)
            p=samples[n]
            if abs(p['ps_mps'])<.02:roots.append(p)
        except (ValueError,RuntimeError):pass


        if not any(low['load_g']<=p['load_g']<=high['load_g'] for p in roots):
            candidates=[p for p in samples.values() if p['valid'] and
                low['load_g']<=p['load_g']<=high['load_g'] and abs(p['ps_mps'])<.02]
            if candidates:roots.append(min(candidates,key=lambda p:abs(p['ps_mps'])))
    if bottom:bottom['_checked_probe_lower_boundary']=True
    for p in samples.values():p.setdefault('surface_sample',False)
    surface_points=sorted((p for p in samples.values() if boundary and p['valid'] and
        p['surface_sample'] and p['load_g']<=boundary['load_g']),key=lambda p:p['load_g'])
    return dict(speed_kmh=float(speed),points=sorted(samples.values(),key=lambda p:p['load_g']),
                boundary=boundary,sustained=roots,load_checks=checks,
                boundary_bracket_g=[p['load_g'] for p in bracket] if bracket else None,
                boundary_reason=boundary.get('envelope_limit',{}).get('kind') if boundary else
                                'sweep unavailable' if any('sweep unavailable' in p['reasons'] for p in samples.values()) else None,
                boundary_status='sweep unavailable' if any('sweep unavailable' in p['reasons'] for p in samples.values()) else
                                'Instructor boundary unresolved' if any('Instructor boundary unresolved' in p['reasons'] for p in samples.values()) else
                                'no feasible samples' if boundary is None else
                                'verified limit' if boundary.get('envelope_limit') else
                                'plot ceiling' if not bracket and abs(boundary['load_g']-load_ceiling)<1e-8 and (cfg['max_load_g'] is not None or description(solver,speed)) else 'unresolved numerical boundary',
                lower_boundary=bottom,lower_boundary_searched=True,
                native_transitions=[dict(load_bracket_g=[a['load_g'],b['load_g']],branches=[a.get('roll_leveling_branch'),b.get('roll_leveling_branch')])
                    for a,b in zip(surface_points,surface_points[1:])
                    if a.get('roll_leveling_branch')!=b.get('roll_leveling_branch')] if boundary else [],
                numerical_gap_brackets=gap_brackets,
                interior_failures=[p['load_g'] for p in samples.values() if p['surface_sample'] and boundary and bottom and bottom['load_g']<p['load_g']<boundary['load_g'] and not p['valid']],
                unresolved_load_intervals=pending if boundary and boundary['load_g']>1. else [],
                elapsed_s=time.monotonic()-started,mass=solver.mass,engine=solver.engine.summary,
                branch_selection=branch_selection,branch_search_points=branch_search_points,branch_discovery=discovery)


def column_curve(column):
    if '_curve' in column:return column['_curve']
    top=column['boundary']['load_g'] if column.get('boundary') else float('inf')
    points=[p for p in column['points'] if p.get('surface_sample',True) and p['load_g']<=top]
    good=[p for p in points if p['valid']]
    if len(good)<2:return None
    ns=np.array([p['load_g'] for p in good]);runs=[];run=[]
    for point in points+[None]:
        if point is not None and point['valid']:run.append(point)
        else:
            if len(run)>=2:
                x=np.array([p['load_g'] for p in run]);y=np.array([p['ps_mps'] for p in run])
                ux,uy=interpolation_knots(load_coordinate(x,ns[-1]),y)
                if len(ux)>1:
                    from em_surface import prepare_curve
                    runs.append((x[0],x[-1],prepare_curve(run,ns[-1])))
            run=[]
    column['_curve']=(ns,runs)
    return column['_curve']


def column_values(column,fractions):
    prepared=column_curve(column)
    if prepared is None:return None
    ns,_=prepared
    return column_at_load(column,ns[0]+fractions*(ns[-1]-ns[0]))


def column_at_load(column,loads):
    loads=np.atleast_1d(loads);out=np.full(loads.shape,np.nan)
    prepared=column_curve(column)
    if prepared is None:

        for point in column['points']:
            if point['valid'] and point.get('surface_sample',True):
                out[loads==point['load_g']]=point['ps_mps']
        return out
    ns,runs=prepared
    for low,high,curve in runs:
        mask=(loads>=low)&(loads<=high)
        out[mask]=curve(load_coordinate(loads[mask],ns[-1]))
    return out


def fixed_load_interpolate(columns,speeds,loads):
    from em_fixed_load import interpolate
    return interpolate(columns,speeds,loads)


def alpha_curve(column):
    if '_alpha_curve' in column:return column['_alpha_curve']
    runs=[];run=[]
    top=column['boundary']['load_g'] if column.get('boundary_reason') in ('stall','trim fold') else None
    cap=column['boundary']['load_g'] if column.get('boundary') else float('inf')
    for p in [p for p in column['points'] if (p.get('surface_sample',True) or p.get('alpha_sample')) and p['load_g']<=cap]+[None]:
        same_branch=not run or p is None or p.get('roll_leveling_branch')==run[-1].get('roll_leveling_branch')
        if p is not None and p['valid'] and run and same_branch and p['alpha_deg']<=run[-1]['alpha_deg']:
            previous=run[-1]


            uncertainty=p['force_error_g']+previous['force_error_g']
            if p['load_g']-previous['load_g']<=uncertainty and abs(p['ps_mps']-previous['ps_mps'])<=.1:
                continue
        if p is not None and p['valid'] and same_branch and (not run or p['alpha_deg']>run[-1]['alpha_deg']):
            run.append(p)
        else:
            if len(run)>=2:
                angles=np.array([p['alpha_deg'] for p in run])
                values=np.array([[p['load_g'],p['ps_mps']] for p in run])
                if top:values[:,0]=load_coordinate(values[:,0],top)
                runs.append((angles,PchipInterpolator(angles,values,axis=0),top))
            run=[p] if p is not None and p['valid'] else []
    column['_alpha_curve']=runs
    return runs


def collapsed_load_range(column):
    low,high=column.get('lower_boundary'),column.get('boundary')
    return bool(low and high and high['load_g']-low['load_g']<=.0002)


def endpoint_surface(core,speed,loads,support=None):
    result=np.full(len(loads),np.nan)
    if not any(collapsed_load_range(c) for c in core):return result
    if any(c['boundary_status']!='verified limit' or c.get('numerical_gap_brackets') or
           c.get('unresolved_load_intervals') or c.get('interior_failures') for c in core):return result
    if not all(c.get(k) and c[k]['valid'] for c in core for k in ('lower_boundary','boundary')):return result
    support=core if support is None else support
    xs=[c['speed_kmh'] for c in support]
    lower,upper=[PchipInterpolator(xs,[[c[k]['load_g'],c[k]['ps_mps']] for c in support],axis=0)(speed)
                 for k in ('lower_boundary','boundary')]
    if upper[0]<=lower[0]:return result
    coordinate=(lambda n:load_coordinate(n,upper[0])) if any(c.get('boundary_reason') in ('stall','trim fold') for c in core) else np.asarray
    lo,hi=coordinate(np.array([lower[0],upper[0]]))
    mask=(loads>=lower[0])&(loads<=upper[0])
    fraction=(coordinate(loads[mask])-lo)/(hi-lo)
    result[mask]=lower[1]+(upper[1]-lower[1])*fraction


    t=(speed-core[0]['speed_kmh'])/(core[1]['speed_kmh']-core[0]['speed_kmh'])
    for column,weight in zip(core,(1-t,t)):
        if collapsed_load_range(column):continue
        bottom,top=column['lower_boundary'],column['boundary']
        stall=any(c.get('boundary_reason') in ('stall','trim fold') for c in core)
        low_n,high_n=bottom['load_g'],top['load_g']
        if stall:
            low_u,high_u=load_coordinate(np.array([low_n,high_n]),high_n)
            query=coordinate_load(low_u+fraction*(high_u-low_u),high_n)
        else:query=low_n+fraction*(high_n-low_n)
        values=column_at_load(column,np.clip(query,low_n,high_n))
        straight=bottom['ps_mps']+fraction*(top['ps_mps']-bottom['ps_mps'])
        result[mask]+=weight*(values-straight)
    return result


def segmented_speed_interpolate(columns,speeds,loads,fixed_base=True):
    speeds=np.atleast_1d(speeds);loads=np.atleast_1d(loads)
    out=(fixed_load_interpolate(columns,speeds,loads) if fixed_base else np.full((len(loads),len(speeds)),np.nan))
    xs=np.array([c['speed_kmh'] for c in columns])
    endpoint_curves={}
    def load_curve(branch,edges,stall):
        branch=branch[np.isfinite(branch).all(axis=1)]
        increasing=[]
        for row in branch:
            if not increasing or row[0]>increasing[-1][0]+1e-10:increasing.append(row)
        branch=np.asarray(increasing)
        if len(branch)<2:return None
        branch=branch[(branch[:,0]>edges[0][0]+1e-10)&(branch[:,0]<edges[1][0]-1e-10)]
        branch=np.array([edges[0],*branch,edges[1]])
        bottom,top=branch[0,0],branch[-1,0]
        if top<=1. or top<=bottom:return None
        coordinate=load_coordinate(branch[:,0],top) if stall else branch[:,0]
        ux,uy=interpolation_knots(coordinate,branch[:,1])
        if len(ux)<2:return None
        return bottom,top,PchipInterpolator(ux,uy)
    for j,speed in enumerate(speeds):
        right=int(np.searchsorted(xs,speed));left=right-1
        if right==len(xs) or right==0 or xs[right]==speed:continue
        core=columns[left:right+1]
        support=list(core)


        for i,prepend in [(left-1,True),(right+1,False)]:
            if not 0<=i<len(columns):continue
            c=columns[i]
            if (c['boundary_status']=='verified limit' and
                    all(c.get(k) and c[k]['valid'] for k in ('lower_boundary','boundary')) and
                    not any(c.get(k) for k in ('numerical_gap_brackets','unresolved_load_intervals','interior_failures'))):
                if prepend:support.insert(0,c)
                else:support.append(c)
        wedge=endpoint_surface(core,speed,loads,support)
        missing=~np.isfinite(out[:,j])
        out[missing,j]=wedge[missing]
        if all(c.get('boundary_status')=='plot ceiling' for c in core):continue


        run_count=len(alpha_curve(core[0]))
        if not run_count or len(alpha_curve(core[1]))!=run_count:continue
        segmented=np.full(len(loads),np.nan);complete=True
        for segment in range(run_count):
            indices=[left,right]
            for i,prepend in ((left-1,True),(right+1,False)):
                if 0<=i<len(columns) and len(alpha_curve(columns[i]))==run_count:
                    if prepend:indices.insert(0,i)
                    else:indices.append(i)
            runs=[alpha_curve(columns[i])[segment] for i in indices]
            low=max(alpha_curve(c)[segment][0][0] for c in core)
            high=min(alpha_curve(c)[segment][0][-1] for c in core)
            if high<=low:complete=False;break
            knots=np.unique(np.concatenate([alpha_curve(c)[segment][0] for c in core]))
            angles=np.unique(np.r_[knots[(knots>=low)&(knots<=high)],np.linspace(low,high,97)])


            angles=angles[(angles>low)&(angles<high)]
            values=np.full((len(indices),len(angles),2),np.nan)
            for row,(knots,curve,top) in enumerate(runs):
                mask=(angles>=knots[0])&(angles<=knots[-1])
                values[row,mask]=curve(angles[mask])
                if top:values[row,mask,0]=coordinate_load(values[row,mask,0],top)
            branch=np.full((len(angles),2),np.nan)
            masks=np.isfinite(values[:,:,0]).T
            for support in np.unique(masks,axis=0):
                selected=np.all(masks==support,axis=1);ii=np.where(support)[0]
                if len(ii)<2 or not np.all(np.diff(ii)==1):continue
                branch[selected]=PchipInterpolator(xs[np.array(indices)[ii]],values[ii][:,selected],axis=0)(speed)


            edges=[];endpoint_pairs=[]
            for edge in (0,-1):
                pairs=[]
                for knots,curve,top in runs:
                    pair=np.array(curve(knots[edge]),dtype=float)
                    if top:pair[0]=coordinate_load(pair[0],top)
                    pairs.append(pair)
                endpoint_pairs.append(pairs)
                edges.append(PchipInterpolator(xs[indices],pairs,axis=0)(speed))
            stall=any(c.get('boundary_reason') in ('stall','trim fold') for c in core)
            prepared=load_curve(branch,edges,stall)
            if prepared is None:complete=False;break
            bottom,top,curve=prepared
            mask=(loads>=bottom)&(loads<=top)
            query=load_coordinate(loads[mask],top) if stall else loads[mask]
            prediction=curve(query)


            low_u=load_coordinate(bottom,top) if stall else bottom
            high_u=1. if stall else top
            fraction=(query-low_u)/(high_u-low_u)
            t=(speed-xs[left])/(xs[right]-xs[left])
            blend=t*t*(3.-2.*t)
            for index,weight in ((left,1.-blend),(right,blend)):
                key=(left,segment,index)
                if key not in endpoint_curves:
                    row=indices.index(index)
                    endpoint_curves[key]=load_curve(values[row],
                        [endpoint_pairs[0][row],endpoint_pairs[1][row]],stall)
                endpoint=endpoint_curves[key]
                if endpoint is None:complete=False;break
                low_n,high_n,projected=endpoint
                low_u=load_coordinate(low_n,high_n) if stall else low_n
                high_u=1. if stall else high_n
                endpoint_u=low_u+fraction*(high_u-low_u)
                endpoint_load=(coordinate_load(endpoint_u,high_n) if stall else endpoint_u)
                canonical=column_at_load(columns[index],np.clip(endpoint_load,low_n,high_n))
                prediction+=weight*(canonical-projected(endpoint_u))
            if not complete:break
            segmented[mask]=prediction
        if complete:
            if run_count==1:
                mask=np.isfinite(segmented)
                out[mask,j]=segmented[mask]
            else:out[:,j]=segmented
    return out


def speed_interpolate(columns,speeds,loads):
    from em_surface import coordinate_regions
    speeds=np.atleast_1d(speeds);loads=np.asarray(loads)
    if not any(c.get('constraint_corner') for c in columns) or any(c.get('_coordinate_mapped') for c in columns):
        return _speed_interpolate(columns,speeds,loads)
    result=np.full((len(loads),len(speeds)),np.nan)
    for chosen,support,query in coordinate_regions(columns,speeds):
        result[:,chosen]=_speed_interpolate(support,query,loads if loads.ndim==1 else loads[:,chosen])
    return result


def _speed_interpolate(columns,speeds,loads):
    from em_surface import fitted_interpolate,eligible,envelope_values
    speeds=np.atleast_1d(speeds);loads=np.asarray(loads)
    out,covered=fitted_interpolate(columns,speeds,loads)
    xs=np.array([c['speed_kmh'] for c in columns])
    for index,speed in enumerate(speeds):
        right=int(np.searchsorted(xs,speed));left=right-1
        if not 0<right<len(xs) or xs[right]==speed:continue
        local=columns[max(0,left-1):min(len(columns),right+2)]
        query=loads if loads.ndim==1 else loads[:,index]
        valid=all(eligible(c) for c in columns[left:right+1])
        if valid:continue
        physical=segmented_speed_interpolate(local,[speed],query,fixed_base=True)[:,0]
        if not covered[index]:out[:,index]=physical
    limits=envelope_values(columns,speeds)
    grid=loads[:,None] if loads.ndim==1 else loads
    for side in (0,1):
        edge=np.isclose(grid,limits[:,side][None,:],rtol=0.,atol=1e-10)
        out=np.where(edge,limits[:,side+2][None,:],out)
    return out


def sample_speed_probe(task):
    name,config_json,speed,seed=task[:4];started=time.monotonic()
    column=sample_boundary_column(task[:4])
    column['speed_probe']=True
    if column.get('boundary_status')!='verified limit' or not column.get('boundary'):return column
    solver=worker_solver(name,config_json);top=column['boundary']['load_g']
    samples={p['load_g']:p for p in column['points']}
    def initial(n):
        groups={}
        for point in seed or []:
            if point['valid']:groups.setdefault(point['speed_kmh'],[]).append(point)
        guesses=[]
        for v,points in sorted(groups.items()):
            points.sort(key=lambda p:p['load_g']);cap=points[-1]['load_g']
            query=1.+(n-1.)*(cap-1.)/max(1e-12,top-1.)
            guess=[float(np.interp(query,[p['load_g'] for p in points],[p['solution'][j] for p in points])) for j in range(5)]
            guesses.append((v,guess))
        if not guesses:return None
        guess=[float(np.interp(speed,[v for v,_ in guesses],[g[j] for _,g in guesses])) for j in range(5)]
        guess[1]=math.degrees(math.acos(1./max(1.,n)))
        return guess
    if 1. not in samples:samples[1.]=solver.solve(speed,1.,initial(1.),exhaustive=False,quick=True)
    if samples[1.]['valid']:
        column['lower_boundary']=samples[1.]
    elif not column.get('lower_boundary_searched'):


        column=sample_column((name,config_json,speed,[p for p in samples.values() if p['valid']]),boundary_only=True)
        column['speed_probe']=True
        samples={p['load_g']:p for p in column['points']}
        if not column.get('boundary') or not column.get('lower_boundary'):return column
        top=column['boundary']['load_g']
    bottom=column.get('lower_boundary')
    if not bottom or not bottom['valid']:return column
    floor=bottom['load_g']
    if top>floor:
        fractions=np.linspace(0.,1.,3)


        start=float(load_coordinate(floor,top))
        loads=(floor+fractions*(top-floor) if column.get('boundary_reason') in ('wing force','control')
               else coordinate_load(start+fractions*(1.-start),top))
        for n in loads[1:-1]:
            n=float(n)
            if n not in samples:samples[n]=solver.solve(speed,n,initial(n),exhaustive=False,quick=True)
        for n in task[4] if len(task)>4 else []:
            if floor<=n<=top and n not in samples:
                samples[n]=solver.solve(speed,n,initial(n),exhaustive=False,quick=True)
    if top-floor>.05 and len(task)<=4:


        groups={}
        for p in seed or []:
            if p['valid']:groups.setdefault(p['speed_kmh'],[]).append(p['load_g'])
        common_top=min([top]+[max(ns) for ns in groups.values()])
        near_top=common_top-.005*(common_top-floor) if common_top-floor>.05 else top-.005*(top-floor)
        for n in (floor+min(.1,.01*(top-floor)),near_top):
            if n not in samples:samples[n]=solver.solve(speed,n,initial(n),exhaustive=False,quick=True)
    column['points']=sorted(samples.values(),key=lambda p:p['load_g'])
    if not solver.is_prop:
        column['longitudinal_speed_knots_kmh']=sorted({float(v)*3.6
            for unit in solver.engine.units if hasattr(unit,'jet') for v in unit.jet['speed']})
    column['elapsed_s']=time.monotonic()-started
    if all(p['valid'] for p in column['points'] if floor<=p['load_g']<=top):
        column['boundary']['_checked_probe_boundary']=True
    return column


def sample_interior_speed_probe(task):
    name,config_json,speed,seed,load=task;started=time.monotonic()
    solver=worker_solver(name,config_json);groups={}
    from em_speed_limits import excluded_column
    excluded=excluded_column(solver,speed)
    if excluded is not None:return excluded
    for point in seed:
        if point['valid']:groups.setdefault(point['speed_kmh'],[]).append(point)
    guesses=[]
    for v,points in sorted(groups.items()):
        points.sort(key=lambda p:p['load_g'])
        guesses.append((v,[float(np.interp(load,[p['load_g'] for p in points],
            [p['solution'][axis] for p in points])) for axis in range(5)]))
    initial=[float(np.interp(speed,[v for v,_ in guesses],[g[axis] for _,g in guesses])) for axis in range(5)]
    initial[1]=math.degrees(math.acos(1./load))
    point=solver.solve(speed,load,initial,exhaustive=False,quick=True)
    return dict(speed_kmh=speed,points=[point],boundary=point,lower_boundary=point,
        boundary_status='interior speed probe',boundary_reason=None,sustained=[],load_checks=[],
        numerical_gap_brackets=[],interior_failures=[],unresolved_load_intervals=[],
        speed_point_probe=True,elapsed_s=time.monotonic()-started,mass=solver.mass,engine=solver.engine.summary)


def compute_adaptive(config,progress=None,cancelled=None,preview=None):
    import json
    from em_accuracy import turn_tolerance,speed_tolerance,coverage
    from em_solver import AIRCRAFT,BACKEND,aircraft_settings
    from instructor_envelope import profile
    from aircraft_catalog import load
    from em_speed_limits import speed_limits
    start=time.monotonic();cfg_json=json.dumps(config,sort_keys=True)
    conditions={name:aircraft_settings(config,name) for name in config['aircraft']}
    redlines={name:speed_limits(load(name),cfg) for name,cfg in conditions.items()}
    from em_mach_events import aircraft_catalog as mach_catalog,speed_knots,crossings,transition_width,discontinuity_regions,overlaps
    mach_info={name:mach_catalog(load(name),cfg) for name,cfg in conditions.items()}
    discontinuities={name:discontinuity_regions(mach_info[name],cfg) for name,cfg in conditions.items()}
    mach_intervals={name:[] for name in conditions}
    workers=WORKERS;results={name:{} for name in config['aircraft']}
    output=dict(settings=config,aircraft=[],sampling='adaptive',backend=BACKEND,workers=workers,
        loads_g=[],method='near-coordinated trim below positive stall; per-aircraft flight model and Ps definition; adaptive cubic surface',
        assumptions=['Full-real manual aerodynamic trim','Positive-AoA stall enforced; negative-AoA stall not an exclusion; aircraft trim model selected per entry','Full pilot authority; aerodynamic control-power loss retained',
                     'Constant fuel and intact components','Propeller torque/gyro selected per aircraft: off for RB by default, on for SB; axial propwash retained','Still air; out of ground effect; retractable gear and airbrakes stowed; fixed propeller-aircraft gear retains its drag',
                     'Propellers: automatic or idealized manual engine management selected per aircraft; closed radiators and frozen boost supply; complete aircraft phase outputs averaged; manual global optimum and periodic flight trajectory not certified',
                     'Zero sideslip preferred; failed interior equilibria may use solved sideslip up to 2 degrees, with unchanged force/moment closure; not a minimum-drag sideslip optimization',
                     'Fixed flap extension; speed domain ends at the selected extension’s automatic IAS/Mach limit or intact-flap damage threshold; flap travel and damage transients omitted',
                     'Steady Instructor AoA schedule approximation: native Mach/flap/sweep angle targets, settled wing-angle adjustments, native rate feedback and reduced moment balance, with full physical trim and control-power loss. Same constraint at boundary and interior. Transient overshoot, delay, retained trim and overload reserve/release are omitted',
                     'Manual fixed sweep; native reachability limits retained; 0% forward, 100% aft',
                     'VTOL, reverse and thrust-vectoring commands zero; auxiliary rocket boosters off',
                     'Normal controllable flight: native pitch response must retain the normal elevator direction',
                     'Independently balanced pre-stall turns; failed intervals remain unresolved; extra mass at configured CG'],
        validation='Reconstructed kernels have native-code comparisons; this EM solver has not been validated against live flight.')
    if config.get('reference_load_cap',False):
        output['assumptions'].append('Chart search ceiling is a frozen lookup table: maximum of saved I-153 M-62, BI and F-16XL SB flutter-on boundaries plus the smaller of 2 deg/s or 1 g. References: sea level, zero fuel mass, clean, supplied maximum power, torque/gyro on, structural load limits on, each clipped at its speed redline. Linear TAS lookup with endpoint holds; never recalibrated during chart calculation. This is a search-domain assumption, not a proved physical limit.')
    output['assumptions'].append(f"Additional global search ceiling: {config['global_load_cap_g']:g} g; not a proved physical limit.")
    if config.get('aircraft_search_region',False) and not config.get('reference_load_cap',False):
        output['assumptions'].append('Predetermined per-aircraft search region: Mach/flap-dependent wing, tail and fuselage lift, selected mass, wing strength and algebraic propulsion estimates; padded by 25 percent plus 0.25 g. No engine settling, nested solves or trim-driven cap expansion. A valid edge at the cap is a search ceiling, not a physical limit. The estimated domain is not a proved physical exclusion.')
    if any(c['turn_response_mode']=='local_acceleration' for c in conditions.values()):
        output['assumptions'].append('Local acceleration selected per aircraft: first-order constant-load, level coordinated turn; fixed sideslip; settled aerodynamic memory; no entry history. Large local changes are flagged; unavailable derivatives remain unresolved.')
    if any(AIRCRAFT[name]['propulsion']=='rocket' for name in results):
        output['assumptions'].append('Rocket aircraft: supplied stationary thrust at selected throttle and fixed propellant mass; fuel depletion, ignition and burnout trajectories are not simulated. Authored thrust direction and mount moments retained.')
    if config['low_speed_load_cap'] and not config.get('reference_load_cap',False):
        output['assumptions'].append('Chart search limited to 3.6 g at or below 200 km/h TAS and 8 g above 200 and below 300 km/h TAS; verified higher-speed rising-branch endpoints may tighten this under a monotonic-envelope assumption. This is a search-domain assumption, not a proved physical limit.')
    speed_checks={name:[] for name in results};fractions=np.linspace(0.,1.,17)
    boundary_checks={name:[] for name in results}
    boundary_probes={name:{} for name in results}
    entry_histories={name:{} for name in results}
    cache_hits=0;preview_time=0.;preview_version=0;published_version=-1;speed_grids={}
    def publish_preview(force=False):
        nonlocal preview_time,published_version
        now=time.monotonic()
        if preview is None or preview_version==published_version or (not force and now-preview_time<3.):return
        if not any(sum(bool(c['boundary']) for c in cols.values())>=2 for cols in results.values()):return
        snapshot=dict(output,preview=True,elapsed_s=now-start,aircraft=[],speeds_kmh=[])
        for index,(name,cols) in enumerate(results.items()):
            ordered=[cols.get(v,dict(speed_kmh=v,points=[],boundary=None,lower_boundary=None,
                sustained=[],boundary_status='pending',boundary_reason=None,numerical_gap_brackets=[]))
                for v in sorted(set(speed_grids[name])|set(cols))]
            points=[p for c in ordered for p in c['points']]
            first=next(iter(cols.values()),None)
            snapshot['aircraft'].append(dict(AIRCRAFT[name],id=name,color=['#38c9d7','#ffa66b'][index],
                settings=conditions[name],columns=ordered,points=points,
                interpolation=dict(coverage(config),native_discontinuities=discontinuities[name],mach_approximations=mach_info[name].get('approximations',[])),
                instructor_approximation=profile(load(name),conditions[name]) if conditions[name]['instructor'] else None,
                boundary_columns=[outline_column(c) for c in sorted(
                    {**boundary_probes[name],**{c['speed_kmh']:c for c in ordered}}.values(),key=lambda c:c['speed_kmh'])],
                sustained=[p for c in ordered for p in c['sustained']],
                speed_limit=redlines[name],sweep_excluded_speeds_kmh=exclusions[name],
                mass=first['mass'] if first else {},engine=first['engine'] if first else {'policy':'Waiting for solved samples'},
                valid_points=sum(p['valid'] for p in points),converged_points=sum(p['converged'] for p in points)))


        preview(clone(snapshot));preview_time=now;published_version=preview_version
    def check_cancel():
        if cancelled and cancelled():raise InterruptedError('Calculation cancelled')
    with process_pool() as (pool,worker_cancel):
        def run(tasks,phase,worker=sample_column):
            nonlocal cache_hits,preview_version
            work=OrderedWork()
            for task in tasks:
                name=task[0]
                local=(name,json.dumps(conditions[name],sort_keys=True),*task[2:])
                key=None
                if worker in (sample_column,sample_initial_column,sample_boundary_column):
                    seed=[(p['load_g'],p['solution'],p.get('sideslip_attitude_deg',0.),
                           p.get('envelope_limit'),p.get('_propulsion_seed'),p.get('_low_speed_cap_anchor')) for p in task[3]] if task[3] else None
                    key=(worker.__name__,name,local[1],task[2],json.dumps(seed,separators=(',',':')))
                if key is not None and key in _COLUMN_CACHE:
                    _COLUMN_CACHE.move_to_end(key);cache_hits+=1
                    work.submit((task,key),value=(clone(_COLUMN_CACHE[key]),{}))
                else:
                    future=pool.submit(sample_with_history,worker,local,dict(entry_histories[name]))
                    work.submit((task,key),future=future)
            done=0;reported=0.
            destination=boundary_probes if worker is sample_boundary_column else results
            publish_preview()
            try:
                while work:
                    check_cancel()
                    for (task,key),(column,history) in work.take():
                        done+=1
                        for speed,entry in history.items():entry_histories[task[0]].setdefault(speed,entry)
                        if column is not None:
                            destination[task[0]][column['speed_kmh']]=column
                            preview_version+=1
                            if key is not None:
                                _COLUMN_CACHE[key]=clone(column)
                                if len(_COLUMN_CACHE)>256:_COLUMN_CACHE.popitem(last=False)
                        if progress:progress(dict(done=done,total=len(tasks),aircraft=task[0],speed_kmh=column['speed_kmh'] if column else None,phase=phase,elapsed_s=time.monotonic()-start))
                    publish_preview()
                    if progress and time.monotonic()-reported>1.:
                        progress(dict(done=done,total=len(tasks),phase=phase,elapsed_s=time.monotonic()-start));reported=time.monotonic()
            except BaseException:
                worker_cancel.set()
                work.cancel()
                raise
        exclusions={name:sweep_speed_intervals(name,conditions[name]) for name in results}
        speed_grids={}
        for name in results:
            upper=min(config['speed_max_kmh'],redlines[name]['sample_speed_kmh']) if redlines[name]['enforced'] else config['speed_max_kmh']
            speeds=(np.linspace(config['speed_min_kmh'],upper,config['speed_samples']).tolist()
                    if upper>config['speed_min_kmh'] else [config['speed_min_kmh']])
            speeds.extend(v for v in speed_knots(mach_info[name],conditions[name])
                          if config['speed_min_kmh']<v<upper)
            for interval in exclusions[name]:
                for edge in interval:


                    speeds.extend(edge+offset for offset in [-.01,.01]
                                  if config['speed_min_kmh']<edge+offset<upper)
            speed_grids[name]=sorted(set(speeds))
        if config.get('reference_load_cap',False) or not config['low_speed_load_cap']:
            run([(name,cfg_json,v,None) for name,speeds in speed_grids.items() for v in speeds],'Solving feasible speed columns',sample_initial_column)
        else:
            from em_load_limits import anchor_from_columns,LOWER_SPEED_KMH,LOWER_LOAD_G
            run([(name,cfg_json,v,None) for name,speeds in speed_grids.items() for v in speeds if v>=300.],
                'Solving reference speed columns',sample_initial_column)
            threshold_tasks=[]
            for name,speeds in speed_grids.items():
                nearby=min((c for c in results[name].values() if c.get('boundary') and c['boundary']['valid']),
                    key=lambda c:c['speed_kmh'],default=None)
                for threshold,cap,knots in ((300.,8.,(299.99,300.)),
                        (LOWER_SPEED_KMH,LOWER_LOAD_G,(LOWER_SPEED_KMH,LOWER_SPEED_KMH+.01))):
                    if not min(speeds)<=threshold<max(speeds):continue
                    if nearby is not None and nearby['boundary']['load_g']<cap:continue
                    for v in knots:
                        v=max(config['speed_min_kmh'],min(max(speeds),v))
                        if v not in speeds:speeds.append(v)
                        if v>=300. and v not in results[name]:threshold_tasks.append((name,cfg_json,v,None))
            run(threshold_tasks,'Resolving the low-speed search ceiling',sample_initial_column)
            pending_low={name:sorted((v for v in speeds if v<300.),reverse=True) for name,speeds in speed_grids.items()}
            while any(pending_low.values()):
                tasks=[]
                for name,speeds in pending_low.items():
                    if not speeds:continue
                    for _ in range(min(2,len(speeds))):
                        speed=speeds.pop(0);anchor=anchor_from_columns(results[name].values(),speed)
                        nearest=min((c for c in results[name].values() if c.get('boundary') and c['boundary']['valid']),
                            key=lambda c:abs(c['speed_kmh']-speed),default=None)
                        seed=[dict(p) for p in nearest['points'] if p['valid']] if nearest else None
                        if anchor and seed:seed[0]['_low_speed_cap_anchor']=anchor
                        tasks.append((name,cfg_json,speed,seed))
                run(tasks,'Solving low-speed columns with monotonic load caps',sample_initial_column)
        edge_tasks=[]
        for name,columns in results.items():
            ordered=[columns[v] for v in sorted(columns)]
            first=next((i for i,c in enumerate(ordered) if c['boundary_status']=='verified limit'),None)
            if (first is not None and first>0 and ordered[first-1]['boundary_status'] in ('no feasible samples','level probe unresolved')
                    and ordered[first]['boundary_reason'] in ('stall','Instructor pitch','control')):
                edge_tasks.append((name,cfg_json,ordered[first-1],ordered[first],'stall'))
            for left,right in zip(ordered,ordered[1:]):
                if overlaps(discontinuities[name],left['speed_kmh'],right['speed_kmh']):continue
                kinds={c['boundary_reason'] for c in (left,right)}
                if (all(c['boundary_status']=='verified limit' for c in (left,right)) and
                        (kinds=={'stall','wing force'} or
                         'Instructor pitch' in kinds and bool(kinds & {'wing force','control'}))):
                    edge_tasks.append((name,cfg_json,left,right,'corner'))
                levels=[next((p for p in c['points'] if p['valid'] and p['load_g']==1.),None)
                        for c in (left,right)]
                if all(levels) and levels[0]['ps_mps']*levels[1]['ps_mps']<0:
                    edge_tasks.append((name,cfg_json,left,right,'level'))


        run(edge_tasks,'Solving level-flight endpoints',sample_edge)
        stall_edges={name:next((c['speed_kmh'] for c in cols.values() if c.get('level_stall_endpoint')),None)
                     for name,cols in results.items()}
        fallback=[]
        for name,cols in results.items():
            for speed,column in cols.items():
                if not column.get('level_probe_only'):continue
                fallback.append((name,cfg_json,speed,None))
        run(fallback,'Rechecking unresolved level flight',sample_level_started_column)
        def refine_low_speed_edge():
            for _ in range(10):
                tasks=[]
                for name,columns in results.items():
                    ordered=sorted(columns)
                    first=next((i for i,v in enumerate(ordered) if columns[v]['boundary_status'] in ('verified limit','level probe feasible')),None)
                    if first is None or first==0:continue
                    if columns[ordered[first]].get('level_stall_endpoint'):continue
                    if (columns[ordered[first]].get('lower_boundary') or {}).get('load_g')!=1.:continue
                    lo,hi=ordered[first-1:first+1]
                    if columns[lo]['boundary_status']!='no feasible samples' or hi-lo<=.05:continue


                    seed=[p for p in columns[hi]['points'] if p['valid']]
                    tasks.extend((name,cfg_json,lo+(hi-lo)*fraction,seed)
                        for fraction in (.25,.5,.75))
                if not tasks:break
                run(tasks,'Refining the low-speed edge',sample_level_edge_probe)


            tasks=[]
            for name,columns in results.items():
                probes=[c for c in columns.values() if c.get('level_edge_probe')]
                good=[c for c in probes if c['boundary_status']=='level probe feasible']
                if good:
                    edge=min(good,key=lambda c:c['speed_kmh'])
                    neighbors=sorted((c for c in columns.values() if c['boundary_status']=='verified limit'),key=lambda c:abs(c['speed_kmh']-edge['speed_kmh']))[:1]
                    left=max((c for c in columns.values() if c['speed_kmh']<edge['speed_kmh']),key=lambda c:c['speed_kmh'])
                    tasks.append((name,cfg_json,edge['speed_kmh'],edge['points']+[p for c in neighbors for p in c['points'] if p['valid']],left,edge))
                for c in good:del columns[c['speed_kmh']]
            run(tasks,'Completing the low-speed edge',complete_level_edge)


        refine_low_speed_edge()
        exact_edges=[]
        for name,columns in results.items():
            ordered=sorted(columns.values(),key=lambda c:c['speed_kmh'])
            i=next((i for i,c in enumerate(ordered) if c['boundary_status']=='verified limit'),None)
            if i is None or i==0:continue
            left,right=ordered[i-1:i+1]
            if (right.get('level_stall_endpoint') or right['boundary_reason'] not in ('stall','Instructor pitch','control')
                    or right['boundary']['load_g']>=1.1):continue
            exact_edges.append((name,cfg_json,left,right))
        run(exact_edges,'Closing the level-flight speed bracket',sample_stall_endpoint)
        stall_edges={name:next((c['speed_kmh'] for c in cols.values() if c.get('level_stall_endpoint')),None)
                     for name,cols in results.items()}


        retries=[]
        for name,cols in results.items():
            edge=stall_edges[name]
            if edge is None:continue
            for speed,column in cols.items():
                if (speed<=edge or not column.get('level_edge_probe') or
                        any(p['converged'] for p in column['points'])):continue
                neighbors=sorted((c for c in cols.values() if c.get('boundary') and c['boundary']['valid']),
                                 key=lambda c:abs(c['speed_kmh']-speed))[:2]
                retries.append((name,cfg_json,speed,[p for c in neighbors for p in c['points'] if p['valid']],column['points']))
        run(retries,'Rechecking provisional level-flight failures',recheck_level_probe)
        intervals={}
        for name,cols in results.items():
            speeds=sorted(v for v in cols if stall_edges[name] is None or v>=stall_edges[name])
            intervals[name]=list(zip(speeds,speeds[1:]))
        terminal_speed_intervals={name:[] for name in results}


        speed_stop=.25*speed_tolerance(config)
        initial_intervals=intervals
        intervals={name:[] for name in results}
        physical_stall_speed_brackets={name:[] for name in results}
        speed_work=OrderedWork();scheduled_speed=0;finished_speed=0
        checked_probes={name:{} for name in results};point_check_requests=set()
        def schedule_speed(name,lo,hi,depth,point_load=None,point_speed=None):
            nonlocal cache_hits,scheduled_speed
            if overlaps(discontinuities[name],lo,hi):return
            columns=results[name];prior_speeds=tuple(sorted(columns))
            mid=(lo+hi)*.5 if point_load is None else speed_check_position(lo,hi)
            if point_speed is not None:mid=point_speed
            left,right=columns[lo],columns[hi]
            if (hi-lo<=transition_width(config) and
                    all(c.get('boundary_status')=='verified limit' for c in (left,right)) and
                    crossings(mach_info[name]['events'],left.get('boundary'),right.get('boundary'))):
                if (lo,hi) not in mach_intervals[name]:mach_intervals[name].append((lo,hi))
                return
            cap=min(c['boundary']['load_g'] if c['boundary'] else 1. for c in (left,right))
            floor=max(c['lower_boundary']['load_g'] if c.get('lower_boundary') else 1. for c in (left,right))
            loads=floor+fractions*max(0.,cap-floor)


            pred=speed_interpolate([columns[v] for v in prior_speeds],[mid],loads)[:,0]
            edge_rate=boundary_prediction(columns,mid)
            edge_load=math.hypot(1.,math.radians(edge_rate)*(mid/3.6)/9.8100004196167) if edge_rate is not None else None
            context=(name,lo,hi,mid,depth,pred,loads,edge_load,prior_speeds)
            seed=[p for c in (left,right) for p in c['points'] if p['valid']]
            local=(name,json.dumps(conditions[name],sort_keys=True),mid,seed)
            from em_surface import eligible
            worker=sample_speed_probe if all(eligible(c) for c in (left,right)) else sample_column
            level_speeds=[v for v,c in columns.items() if any(p['valid'] and p['load_g']==1. for p in c['points'])]
            if level_speeds and mid<min(level_speeds):worker=sample_level_started_column
            if point_load is not None:
                worker=sample_interior_speed_probe;local=(*local,point_load)
            elif worker is sample_speed_probe:
                from em_accuracy import surface_contour_loads
                local=(*local,surface_contour_loads([columns[v] for v in prior_speeds],mid,config))
            seed_key=[(p['load_g'],p['solution'],p.get('sideslip_attitude_deg',0.),p.get('envelope_limit'),p.get('_propulsion_seed')) for p in seed]
            key=(worker.__name__,name,local[1],mid,json.dumps(seed_key,separators=(',',':')))
            if point_load is not None:key=(*key,point_load)
            elif len(local)>4:key=(*key,tuple(local[4]))
            scheduled_speed+=1
            if key in _COLUMN_CACHE:
                _COLUMN_CACHE.move_to_end(key);cache_hits+=1
                speed_work.submit((context,None),value=(clone(_COLUMN_CACHE[key]),{}))
            else:
                future=pool.submit(sample_with_history,worker,local,dict(entry_histories[name]))
                speed_work.submit((context,key),future=future)
        def accept_speed(context,column,history,key):
            nonlocal preview_version,finished_speed,scheduled_speed,cache_hits
            name,lo,hi,mid,depth,pred,loads,predicted_cap,prior_speeds=context
            for speed,entry in history.items():entry_histories[name].setdefault(speed,entry)
            if key is not None:
                _COLUMN_CACHE[key]=clone(column)
                if len(_COLUMN_CACHE)>256:_COLUMN_CACHE.popitem(last=False)


            left,right=results[name][lo],results[name][hi]
            from em_surface import envelope_limits
            source=[results[name][v] for v in prior_speeds]
            predicted_limits=envelope_limits(source,[mid])[0]
            holdouts=[p for p in column['points'] if p['valid'] and column.get('boundary') and
                column.get('lower_boundary') and column['lower_boundary']['load_g']<=p['load_g']<=column['boundary']['load_g']]
            query=loads.copy()
            if holdouts:
                loads=np.array([p['load_g'] for p in holdouts]);query=loads.copy()


                for k,field in enumerate(('lower_boundary','boundary')):
                    if np.isfinite(predicted_limits[k]) and (column['boundary_status'] in ('verified limit','plot ceiling') or
                            k==0 and column[field]['load_g']==1.):
                        query[loads==column[field]['load_g']]=predicted_limits[k]
                pred=speed_interpolate(source,[mid],query)[:,0]
                actual=np.array([p['ps_mps'] for p in holdouts])
            else:actual=column_at_load(column,loads)
            finite=np.isfinite(actual)&np.isfinite(pred)
            error=float(np.max(abs(actual[finite]-pred[finite]))) if finite.any() else None
            from em_accuracy import neighboring_loads,turn_tolerance,within_contour_band,visible_error
            band=neighboring_loads(mid,loads,turn_tolerance(config['sep_tolerance_mps']))
            if np.isfinite(predicted_limits).all():band=np.clip(band,*predicted_limits)
            nearby=speed_interpolate(source,[mid],band.ravel())[:,0].reshape((-1,2))
            geometric=within_contour_band(actual,nearby,config['sep_tolerance_mps'])
            from em_accuracy import surface_band_values
            candidates=np.flatnonzero(finite & (abs(actual-pred)>config['sep_tolerance_mps']) & ~geometric & visible_error(actual,pred,config))
            if len(candidates):
                box=surface_band_values(source,mid,query[candidates],config)
                geometric[candidates]|=within_contour_band(actual[candidates],box,config['sep_tolerance_mps'])
            failed=finite&(abs(actual-pred)>config['sep_tolerance_mps'])&~geometric&visible_error(actual,pred,config)


            if hi-lo>4.*speed_tolerance(config):
                failed|=finite&(abs(actual-pred)>2.*config['sep_tolerance_mps'])&visible_error(actual,pred,config)
            caps=[c['boundary']['load_g'] if c['boundary'] else 0. for c in (left,column,right)]
            if np.isfinite(predicted_limits[1]):predicted_cap=predicted_limits[1]


            verified=lambda c:c['boundary_status'] in ('verified limit','plot ceiling')
            boundary_error=(abs(caps[1]-predicted_limits[1])
                if verified(column) and np.isfinite(predicted_limits[1]) else None)
            boundary_turn_error=(abs(column['boundary']['turn_dps']-
                math.degrees(9.8100004196167*math.sqrt(max(0.,predicted_limits[1]**2-1.))/(mid/3.6)))
                if boundary_error is not None else None)
            boundary_inaccurate=(boundary_turn_error is not None and
                boundary_turn_error>turn_tolerance(config['sep_tolerance_mps']))
            near_index=int(np.searchsorted(prior_speeds,mid))
            outline=source[max(0,near_index-2):near_index+2]
            turning_outline=any(all(c.get('boundary') for c in triple) and
                (triple[1]['boundary']['turn_dps']-triple[0]['boundary']['turn_dps'])*
                (triple[2]['boundary']['turn_dps']-triple[1]['boundary']['turn_dps'])<0.
                for triple in zip(outline,outline[1:],outline[2:]))
            if boundary_inaccurate and not turning_outline:


                dv=speed_tolerance(config)
                nearby_speeds=np.clip(np.array([mid-dv,mid,mid+dv]),lo,hi)
                caps_near=envelope_limits(source,nearby_speeds)[:,1]
                rates_near=np.degrees(9.8100004196167*np.sqrt(np.maximum(0.,caps_near**2-1.))/(nearby_speeds/3.6))
                if np.isfinite(rates_near).all():
                    rate=column['boundary']['turn_dps'];dw=turn_tolerance(config['sep_tolerance_mps'])
                    boundary_inaccurate=not (min(rates_near)-dw<=rate<=max(rates_near)+dw)
            feasibility_change=len({bool(c.get('boundary')) for c in (left,column,right)})>1
            speed_checks[name].append(dict(speed_kmh=mid,error_mps=error,boundary_error_g=boundary_error,depth=depth,
                                          probe=bool(column.get('speed_probe')),point_probe=bool(column.get('speed_point_probe')),
                                          holdouts=int(np.count_nonzero(finite)),
                                          failed_holdouts=int(np.count_nonzero(failed)),boundary_error_dps=boundary_turn_error,
                                          boundary_outside_plot_tolerance=boundary_inaccurate))
            overlap=loads[-1]-loads[0]>.0002
            unresolved_boundary=column['boundary_status'] in ('unresolved numerical boundary','Instructor boundary unresolved')
            inaccurate=(np.any(failed) or boundary_inaccurate or feasibility_change or unresolved_boundary or
                        column.get('speed_point_probe') and (not column['points'][0]['valid'] or not finite.all()) or
                        overlap and np.isfinite(actual).any()!=np.isfinite(pred).any() or
                        column.get('speed_probe') and (column.get('boundary_status') not in ('verified limit','plot ceiling') or
                        any(not p['valid'] for p in column['points'] if column.get('lower_boundary') and column.get('boundary') and
                            column['lower_boundary']['load_g']<=p['load_g']<=column['boundary']['load_g'])))
            if column.get('speed_probe') or column.get('speed_point_probe'):
                if inaccurate:


                    seed=[p for p in column['points'] if p['valid']]
                    local=(name,json.dumps(conditions[name],sort_keys=True),mid,seed)
                    future=pool.submit(sample_with_history,sample_column,local,dict(entry_histories[name]))
                    speed_work.submit((context,None),future=future)
                    scheduled_speed+=1
                else:
                    checked_probes[name][mid]=(context,column)
                    risk_request=(name,lo,hi,'contour curvature',prior_speeds)
                    if column.get('speed_probe') and risk_request not in point_check_requests:
                        from em_accuracy import contour_check_points
                        extra=contour_check_points(source,lo,hi,mid,query,actual,pred,config,
                            column.get('longitudinal_speed_knots_kmh',()))
                        if extra:
                            point_check_requests.add(risk_request)
                            for probe_speed,probe_load in extra:
                                if probe_speed not in results[name]:
                                    schedule_speed(name,lo,hi,depth,point_load=probe_load,point_speed=probe_speed)


                    request=(name,lo,hi)
                    if column.get('speed_probe') and hi>1.25*lo and request not in point_check_requests:
                        cap=min(c['boundary']['load_g'] for c in (left,right))
                        floor=max(c['lower_boundary']['load_g'] for c in (left,right))
                        if cap-floor>.05:
                            point_check_requests.add(request)
                            schedule_speed(name,lo,hi,depth,point_load=cap-.005*(cap-floor))
                finished_speed+=1
                if progress:progress(dict(done=finished_speed,total=scheduled_speed,aircraft=name,speed_kmh=mid,
                    phase='Checking selected interior and boundary trims',elapsed_s=time.monotonic()-start))
                return
            finished_speed+=1
            results[name][mid]=column;preview_version+=1
            if inaccurate:
                children=[(lo,mid),(mid,hi)]
                for a,b in children:
                    ca,cb=results[name][a],results[name][b]
                    pair=(ca,cb) if ca['speed_kmh']<cb['speed_kmh'] else (cb,ca)
                    physical_stall_edge=(pair[0]['boundary_status']=='no feasible samples'
                        and pair[1]['boundary_status']=='verified limit'
                        and pair[1].get('boundary_reason')=='stall' and pair[1].get('boundary')
                        and pair[1]['boundary']['load_g']<1.1)
                    if physical_stall_edge:


                        physical_stall_speed_brackets[name].append((a,b));continue
                    verified_pair=all(c['boundary_status'] in ('verified limit','plot ceiling') for c in pair)
                    if hi-lo<=speed_stop or b-a<=speed_stop and not verified_pair:
                        terminal_speed_intervals[name].append((a,b))
                    elif depth>=10:intervals[name].append((a,b))
                    else:schedule_speed(name,a,b,depth+1)
            if progress:progress(dict(done=finished_speed,total=scheduled_speed,aircraft=name,
                speed_kmh=mid,phase='Checking SEP interpolation' if depth==0 else 'Refining SEP curvature',
                elapsed_s=time.monotonic()-start))
            publish_preview()
        for name,spans in initial_intervals.items():
            for lo,hi in spans:schedule_speed(name,lo,hi,0)
        def drain_speed_checks():
            nonlocal scheduled_speed
            try:
                while True:
                    while speed_work:
                        check_cancel()
                        for (context,key),(column,history) in speed_work.take():
                            accept_speed(context,column,history,key)
                    for name,probes in checked_probes.items():
                        for mid,(context,column) in list(probes.items()):
                            prior=context[-1];current=tuple(sorted(results[name]))
                            if prior==current:continue


                            del probes[mid]
                            context=(*context[:-1],current)
                            scheduled_speed+=1
                            accept_speed(context,column,{},None)
                    if not speed_work:break
            except BaseException:
                worker_cancel.set()
                speed_work.cancel()
                raise
        drain_speed_checks()


        endpoint_probes={name:[] for name in results}
        while True:
            from em_accuracy import boundary_contour_speeds
            scheduled=False
            for name,columns in results.items():
                ordered=[columns[v] for v in sorted(columns)]
                for lo,hi,speed in boundary_contour_speeds(ordered,config):
                    if any(abs(speed-v)<=speed_tolerance(config) for v in endpoint_probes[name]):continue
                    endpoint_probes[name].append(speed)
                    schedule_speed(name,lo,hi,0,point_speed=speed)
                    scheduled=True
            if not scheduled:break
            drain_speed_checks()


        spans={name:list(intervals[name]) for name in results}
        for depth in range(9):
            tasks=[];pending_edges={}
            for name,pairs in spans.items():
                edge_columns={**boundary_probes[name],**results[name]}
                for lo,hi in pairs:
                    if overlaps(discontinuities[name],lo,hi):continue
                    if hi-lo<=speed_stop:continue
                    left,right=edge_columns[lo],edge_columns[hi]
                    if not any(c.get('boundary') for c in (left,right)):continue
                    mid=(lo+hi)*.5
                    pending_edges[(name,mid)]=(lo,hi,boundary_prediction(edge_columns,mid))
                    seed=[p for p in left['points'] if p['valid']]
                    if right.get('boundary') and right['boundary'].get('envelope_limit'):seed.append(right['boundary'])
                    tasks.append((name,cfg_json,mid,seed))
            if not tasks:break
            run(tasks,'Refining boundary transitions',sample_boundary_column)
            spans={name:[] for name in results}
            for (name,mid),(lo,hi,predicted_rate) in pending_edges.items():
                edge_columns={**boundary_probes[name],**results[name]}
                cols=[edge_columns[v] for v in (lo,mid,hi)]
                verified=[c['boundary_status']=='verified limit' for c in cols]
                rates=[c['boundary']['turn_dps'] if c.get('boundary') else None for c in cols]
                error=abs(rates[1]-predicted_rate) if all(verified) and predicted_rate is not None else None
                kinds=[c.get('boundary_reason') for c in cols]


                jump=(all(verified) and len(set(kinds))>1 and
                      abs(rates[1]-(rates[0]+rates[2])*.5)>.075)
                boundary_checks[name].append(dict(speed_kmh=mid,bracket_kmh=[lo,hi],
                    error_dps=error,depth=depth,verified=verified))
                if (error is not None and error>.025) or jump or len(set(verified))>1:
                    spans[name].extend([(lo,mid),(mid,hi)])


        retried_limits={}
        for _ in range(2):
            tasks=[]
            for name,columns in results.items():
                good=[c for c in columns.values() if c['boundary_status']=='verified limit']
                for speed,column in columns.items():
                    if column['boundary_status'] not in ('Instructor boundary unresolved','unresolved numerical boundary') or not good:continue
                    if (column.get('boundary') or {}).get('native_branch_search'):continue
                    nearby=sorted(good,key=lambda c:abs(c['speed_kmh']-speed))[:2]
                    if column['boundary_status']=='unresolved numerical boundary' and (
                            len(nearby)<2 or max(abs(c['speed_kmh']-speed) for c in nearby)>max(2.,speed*.02)):continue
                    key=(name,speed);evidence=tuple((c['speed_kmh'],c['boundary']['load_g']) for c in nearby)
                    if retried_limits.get(key)==evidence:continue
                    retried_limits[key]=evidence
                    tasks.append((name,cfg_json,speed,[c['boundary'] for c in nearby]+
                                  [p for p in column['points'] if p['valid']]))
            if not tasks:break
            run(tasks,'Rechecking limits from verified neighboring states')
        def resolve_visible_seams():


            raster=np.linspace(config['speed_min_kmh'],config['speed_max_kmh'],
                2*config['surface_resolution']-1)
            raster_step=(config['speed_max_kmh']-config['speed_min_kmh'])/(len(raster)-1)
            tasks=[];scheduled_raster=set()
            for name,columns in results.items():
                for lo,hi in intervals[name]+terminal_speed_intervals[name]:
                    if overlaps(discontinuities[name],lo,hi):continue
                    if hi-lo>raster_step:continue
                    for speed in raster[(raster>lo)&(raster<hi)]:
                        speed=float(speed)
                        if speed in columns or (name,speed) in scheduled_raster:continue
                        scheduled_raster.add((name,speed))
                        seed=[p for p in columns[lo]['points'] if p['valid']]
                        tasks.append((name,cfg_json,speed,seed))
            if tasks:
                run(tasks,'Solving visible pixels inside narrow numerical seams')
                for name in results:
                    def split_visible(spans):
                        pieces=[]
                        for lo,hi in spans:
                            verified=sorted(v for v,c in results[name].items()
                                if lo<v<hi and any(p['valid'] for p in c['points']))
                            cuts=[lo,*verified,hi]
                            pieces.extend(zip(cuts,cuts[1:]))
                        return pieces
                    intervals[name]=split_visible(intervals[name])
                    terminal_speed_intervals[name]=split_visible(terminal_speed_intervals[name])
            return bool(tasks)


        late_stall_tasks=[]
        for name,columns in results.items():
            if any(c.get('level_stall_endpoint') for c in columns.values()):continue
            verified=sorted((c for c in columns.values()
                if c['boundary_status']=='verified limit' and c.get('boundary_reason')=='stall'
                and c.get('boundary') and c['boundary']['load_g']<1.1),key=lambda c:c['speed_kmh'])
            if not verified:continue
            right=verified[0]
            lefts=[c for c in columns.values() if c['speed_kmh']<right['speed_kmh']
                   and c['boundary_status']=='no feasible samples']
            if lefts:late_stall_tasks.append((name,cfg_json,max(lefts,key=lambda c:c['speed_kmh']),right))
        if late_stall_tasks:
            run(late_stall_tasks,'Solving stall-speed edges',sample_stall_endpoint)
        for name,spans_at_edge in physical_stall_speed_brackets.items():
            solved_edges=[c['speed_kmh'] for c in results[name].values() if c.get('level_stall_endpoint')]
            for lo,hi in spans_at_edge:
                if not any(lo<=speed<=hi for speed in solved_edges):intervals[name].append((lo,hi))


        low_speed_edges={}
        refine_low_speed_edge()
        for name,columns in results.items():
            ordered=sorted(columns)
            first=next((i for i,v in enumerate(ordered) if columns[v]['boundary_status']=='verified limit'),None)
            if first is None:continue
            speed=ordered[first];column=columns[speed]
            previous=columns[ordered[first-1]] if first else None
            clipped=abs(speed-config['speed_min_kmh'])<1e-8
            kind='speed-range edge' if clipped else 'low-speed feasibility edge'
            if (previous and previous['boundary_status']=='no feasible samples' and column['boundary_reason']=='stall'
                and column['boundary']['load_g']<=1.01):
                kind='stall-speed edge'
            if previous and any(p.get('instructor',{}).get('minimum_level_speed_kmh') for p in previous['points'] if p.get('instructor')):
                kind='Instructor minimum-speed edge'
            if previous and column['boundary_reason']=='Instructor pitch' and column['boundary']['load_g']<=1.01:
                kind='Instructor pitch minimum-speed edge'
            low_speed_edges[name]=dict(speed_kmh=speed,kind=kind,
                bracket_kmh=[previous['speed_kmh'],speed] if previous else None,
                refined=bool(column.get('level_stall_endpoint') or previous and speed-previous['speed_kmh']<=.05),
                method='balanced level-flight limit root' if column.get('level_stall_endpoint') else 'speed bracket',
                turn_dps=column['boundary']['turn_dps'],
                lower_load_g=column['lower_boundary']['load_g'] if column['lower_boundary'] else None,
                note='Vertical outline only; Ps exists only where a balanced equilibrium was solved')
        endpoint_tasks=[]
        for name,columns in results.items():
            ordered=[columns[v] for v in sorted(columns)]
            existing=[c['level_endpoint_bracket_kmh'] for c in ordered if c.get('level_endpoint_bracket_kmh')]
            for left,right in zip(ordered,ordered[1:]):
                if overlaps(discontinuities[name],left['speed_kmh'],right['speed_kmh']):continue
                if any(lo<=left['speed_kmh'] and right['speed_kmh']<=hi for lo,hi in existing):continue
                levels=[next((p for p in c['points'] if p['valid'] and p['load_g']==1.),None)
                        for c in (left,right)]
                if all(levels) and levels[0]['ps_mps']*levels[1]['ps_mps']<0:
                    endpoint_tasks.append((name,cfg_json,left,right))
        run(endpoint_tasks,'Solving level-flight Ps=0 endpoints',sample_level_endpoint)


        for name,columns in results.items():
            for collection in (intervals,terminal_speed_intervals):
                remaining=[]
                for lo,hi in collection[name]:
                    cuts=[lo,*sorted(v for v in columns if lo<v<hi),hi]
                    if len(cuts)==2:
                        remaining.append((lo,hi));continue
                    for a,b in zip(cuts,cuts[1:]):
                        if all(columns[v]['boundary_status'] in ('verified limit','plot ceiling') for v in (a,b)):
                            schedule_speed(name,a,b,0)
                        else:remaining.append((a,b))
                collection[name]=remaining


        drain_speed_checks()
        while resolve_visible_seams():drain_speed_checks()


        from em_speed_seam import check_interior
        from em_boundary_seam import check_boundary,boundary_intervals
        seam_interiors={name:[] for name in results};seam_boundaries={name:[] for name in results};seam_work=OrderedWork()
        for name,columns in results.items():
            for lo,hi in sorted(set(intervals[name]+terminal_speed_intervals[name]+mach_intervals[name])):
                if overlaps(discontinuities[name],lo,hi):continue
                if hi-lo>2.*speed_tolerance(config):continue
                pair=[columns[v] for v in (lo,hi)]
                if not all(c.get('lower_boundary') and c.get('boundary') and
                    c['boundary_status'] in ('verified limit','plot ceiling') for c in pair):continue
                support=[{k:c[k] for k in ('speed_kmh','points','boundary','lower_boundary','boundary_status','boundary_reason')} for c in pair]
                task=(name,json.dumps(conditions[name],sort_keys=True),(lo+hi)*.5,support)
                future=pool.submit(sample_with_history,check_interior,task,dict(entry_histories[name]))
                seam_work.submit((name,False),future=future)
            for lo,hi in boundary_intervals(intervals[name]+terminal_speed_intervals[name]+mach_intervals[name],2.*speed_tolerance(config)):
                if overlaps(discontinuities[name],lo,hi):continue
                pair=[columns[v] for v in (lo,hi)]
                support=[{k:c[k] for k in ('speed_kmh','points','boundary','lower_boundary','boundary_status','boundary_reason')} for c in pair]
                task=(name,json.dumps(conditions[name],sort_keys=True),(lo+hi)*.5,support)
                future=pool.submit(sample_with_history,check_boundary,task,dict(entry_histories[name]))
                seam_work.submit((name,True),future=future)
        if seam_work and progress:progress(dict(phase='Checking interiors below uncertain boundary transitions',
            done=0,total=len(seam_work),elapsed_s=time.monotonic()-start))
        while seam_work:
            check_cancel()
            for (name,is_boundary),(certificate,history) in seam_work.take():
                if certificate:(seam_boundaries if is_boundary else seam_interiors)[name].append(certificate)
                for speed,entry in history.items():entry_histories[name].setdefault(speed,entry)
        for index,(name,columns) in enumerate(results.items()):
            ordered=[columns[v] for v in sorted(columns)]
            for column in ordered:
                column.pop('_curve',None);column.pop('_alpha_curve',None);column.pop('_angle_map',None)
            points=[p for col in ordered for p in col['points']]
            probes=list(boundary_probes[name].values())
            points.extend(p for col in probes for p in col['points'])
            points.extend(p for _,col in checked_probes[name].values() for p in col['points'])
            for certificate in seam_interiors[name]:points.extend(certificate.pop('points'))
            for certificate in seam_boundaries[name]:points.extend(certificate['points'])
            checks=speed_checks[name]
            metadata=dict(AIRCRAFT[name],color=['#38c9d7','#ffa66b'][index])
            from aircraft_description import description
            output['aircraft'].append(dict(id=name,**metadata,settings=conditions[name],aircraft_model=description(conditions[name]),points=points,columns=ordered,
                boundary_columns=[outline_column(c) for c in sorted(
                    {**boundary_probes[name],**columns}.values(),key=lambda c:c['speed_kmh'])],
                instructor_approximation=profile(load(name),conditions[name]) if conditions[name]['instructor'] else None,
                instructor_unresolved_speeds_kmh=[c['speed_kmh'] for c in ordered if c['boundary_status']=='Instructor boundary unresolved'],
                sweep_excluded_speeds_kmh=exclusions[name],
                low_speed_edge=low_speed_edges.get(name),
                speed_limit=redlines[name],
                sustained=[p for col in ordered for p in col['sustained']],mass=ordered[0]['mass'],engine=ordered[0]['engine'],
                valid_points=sum(p['valid'] for p in points),converged_points=sum(p['converged'] for p in points),
                interpolation=dict(target_mps=config['sep_tolerance_mps'],target_contour_dps=turn_tolerance(config['sep_tolerance_mps']),target_speed_kmh=speed_tolerance(config),**coverage(config),speed_checks=checks,
                                   unresolved_speed_intervals=intervals[name]+terminal_speed_intervals[name]+[
                                       span for span in mach_intervals[name] if not any(
                                           c['speed_interval_kmh'][0]<=span[0] and c['speed_interval_kmh'][1]>=span[1]
                                           for c in seam_boundaries[name])],boundary_checks=boundary_checks[name],
                                   mach_transition_intervals=mach_intervals[name],mach_events=mach_info[name]['events'],
                                   native_discontinuities=discontinuities[name],
                                   mach_approximations=mach_info[name].get('approximations',[]),
                                   certified_speed_interiors=seam_interiors[name],
                                   certified_boundary_intervals=seam_boundaries[name],
                                   boundary_refinement_intervals=spans[name],load_checks=sum(len(c['load_checks']) for c in ordered))))
    output['speeds_kmh']=sorted(set(v for cols in results.values() for v in cols))
    output['elapsed_s']=time.monotonic()-start
    output['cached_columns']=cache_hits
    return output
