"""Adaptive, directly checked sampling of the connected trim branch below positive stall.

Independent speed columns run in processes. The interpolant never replaces a
force/moment evaluation used for trim, limits, or a Ps=0 root. Rejected samples
remain in the exported evidence. The dense surface uses shape-preserving cubics
inside contiguous feasible columns; no spline is drawn across an empty column.
"""
import math
import os
import time
from em_data import clone
from em_workers import process_pool,WORKERS
from collections import OrderedDict,deque
from concurrent.futures import wait,FIRST_COMPLETED
from functools import lru_cache

import numpy as np
from scipy.interpolate import PchipInterpolator
from scipy.optimize import brentq
from em_roots import checked_root
from em_recovery import recover_equilibrium
from em_pitch_response import PHYSICAL_REASONS
from em_sideslip import recover_sideslip,recover_sideslip_boundary,needs_sideslip_search,recover_fixed_alpha,recover_scalar_sideslip,MAX_SIDESLIP_DEG

# Reuse exact work between interactive requests. Conditions and search seeds
# are part of the key; never reuse a nearby speed, fuel load, or engine mode.
# This process-local cache disappears when the equation server is restarted.
_COLUMN_CACHE=OrderedDict()
_AIRCRAFT_CACHE=OrderedDict()


def compute_cached(config,progress=None,cancelled=None,preview=None):
    """Reuse a completed aircraft when only its comparison partner changes."""
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
    """Only the evidence needed to draw and inspect a verified outline."""
    keys=('speed_kmh','boundary','lower_boundary','boundary_status','boundary_reason')
    return dict({k:column.get(k) for k in keys},
                points=[p for p in column['points'] if p['load_g']==1.])


def load_coordinate(load,top):
    """Resolve the square-root slope approaching maximum lift.

    This is an interpolation coordinate only, not an aerodynamic assumption:
    every accepted knot is still a full force/moment equilibrium. It avoids
    requiring hundreds of equally spaced load points next to a lift maximum.
    """
    # Rationalize 1-sqrt(1-u). The subtractive form maps distinct loads very
    # close to 1 g onto the identical zero coordinate, which makes PCHIP fail.
    # This is the same coordinate, evaluated without cancellation at its floor.
    u=np.minimum(1.,(np.asarray(load)-1.)/(top-1.))
    return u/(1.+np.sqrt(np.maximum(0.,1.-u)))


def coordinate_load(value,top):
    return 1.+(top-1.)*(2*np.asarray(value)-np.asarray(value)**2)


def speed_check_position(low,high):
    """Balance an interval in inverse dynamic pressure for its trim check.

    At fixed load, required lift coefficient varies with 1/V². A check at
    the arithmetic speed midpoint under-samples the more curved, lower-speed
    side of a broad interval. Use the ordinary centered check once the speed
    ratio is at most 1.25; biasing already local native-branch checks adds no
    conditioning benefit. This chooses a probe only; actual native trim,
    interpolation coordinates and error tolerances are unchanged. Bound the
    split away from either end so unusually broad requests still make progress.
    """
    if low<=0. or high<=1.25*low:return (low+high)*.5
    speed=low*math.sqrt(2./(1.+(low/high)**2)) if low>0. else (low+high)*.5
    return min(low+.7*(high-low),max(low+.3*(high-low),speed))


def interpolation_knots(x,y):
    """Keep the last representative of equal *display* coordinates.

    Distinct adjacent float64 loads can map to the same normalized coordinate
    after a load->coordinate->load round trip. PCHIP requires strict increase.
    Raw equilibrium samples remain untouched; never perturb a load or average
    forces to make the interpolator accept it. Reversed knots are still errors.
    """
    x=np.asarray(x);y=np.asarray(y)
    if np.any(np.diff(x)<0):raise ValueError('Interpolation knots are out of order')
    keep=np.r_[np.diff(x)>0,True]
    return x[keep],y[keep]


def sweep_speed_intervals(name,config):
    """Physical speed exclusions for a FIXED requested sweep, in true km/h.

    The native schedule is piecewise linear in Mach. Include every breakpoint
    and every intersection with the requested position; no coarse speed sample
    defines these edges. Actual equilibrium points still check native float32
    Mach and reachability independently.
    """
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
    return TrimSolver(name,json.loads(config_json))


def sample_with_history(worker,task,history):
    """Reuse exact entry-path nodes across matching aircraft worker tasks."""
    solver=worker_solver(task[0],task[1])
    if solver.is_prop and solver.engine.automatic:
        candidates=[p for p in task[3] if p.get('_propulsion_seed')] if isinstance(task[3],list) else []
        solver.engine._task_seed=(min(candidates,key=lambda p:(abs(p['speed_kmh']-task[2]),p['load_g']))['_propulsion_seed']
                                  if candidates else None)
        solver.engine.reset_search()
        solver.__dict__.pop('_trim_predictor',None)
    if solver.config['instructor']:
        for speed,entry in history.items():solver.instructor_trim_entries.setdefault(speed,entry)
    column=worker(task)
    return column,solver.instructor_trim_entries if solver.config['instructor'] else {}


def boundary_prediction(columns,speed):
    """Check the load-space interpolant that is actually drawn on the chart."""
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
    """Close a sustained-turn branch with a solved Ps=0, n=1 speed.

    Only adjacent columns with valid level equilibria provide a bracket. A
    failed trim or excluded speed is not a sign change and cannot be bridged.
    """
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
        # This is the same Ps acceptance used below for the published root.
        # Stop once an independently balanced interior sample meets it.
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
    # Discard only numerically duplicate roots at the floor, preserving any
    # separate higher-load root found in the fully sampled column.
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
        # The active limit can change between the first turning column and
        # 1 g. Try the competing native stall equation before recursively
        # computing whole surfaces at speeds below the actual level edge.
        point=level_stall_edge(solver,left['speed_kmh'],right,
            next((p for p in left['points'] if p['load_g']==1.),None),kind='stall')
    if point is None:return None
    return dict(speed_kmh=point['speed_kmh'],points=[point],boundary=point,lower_boundary=point,
        sustained=[point] if abs(point['ps_mps'])<.002 else [],load_checks=[],boundary_bracket_g=None,
        boundary_reason=point['envelope_limit']['kind'],boundary_status='verified limit',numerical_gap_brackets=[],interior_failures=[],
        unresolved_load_intervals=[],elapsed_s=time.monotonic()-started,mass=solver.mass,engine=solver.engine.summary,
        level_stall_endpoint=True,level_stall_bracket_kmh=[left['speed_kmh'],right['speed_kmh']])


def complete_level_edge(task):
    """Close a measured 1-g constraint before solving a vanishing turn column."""
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
    # An unbalanced level probe is not a physical constraint sign. Keep the
    # original full-column recovery when this direct endpoint cannot close.
    return sample_column(task[:4])


def sample_edge(task):
    """Batch independent physical edge roots into one worker wave."""
    if task[-1]=='corner':
        from em_corner import sample_corner
        return sample_corner(task[:-1])
    return (sample_stall_endpoint if task[-1]=='stall' else sample_level_endpoint)(task[:-1])


def sample_level_edge_probe(task):
    """Bracket the connected low-speed edge using only its defining 1 g trim."""
    name,config_json,speed,seed=task;started=time.monotonic()
    solver=worker_solver(name,config_json)
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
    """Check a second pre-stall start before declaring level flight missing."""
    point=solver.solve(speed,1.,initial,exhaustive=exhaustive)
    if point['converged'] or point.get('bounded_search_stationary'):return point
    # The lift-only linear estimate can start above a tail/polar fold even
    # with an ordinary lower-alpha aircraft equilibrium. Retry halfway from
    # zero lift toward that independent estimate; only the full solver accepts.
    guess=solver.initial_guess(speed/3.6,1.)
    guess[0]*=.5
    retry=solver.solve(speed,1.,guess,exhaustive=False)
    if retry['valid'] or retry['converged'] and not point['converged']:return retry
    return min((point,retry),key=lambda p:p['force_error_g']+p['angular_error_rad_s2'])


def continue_sideslip(solver,speed,load,neighbors):
    """Retain the flight-condition coordinate of a recovered nearby state."""
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
    """Defer a load sweep below level flight until the low edge is known."""
    name,config_json,speed,seed=task;solver=worker_solver(name,config_json)
    if speed!=solver.config['speed_min_kmh']:
        return sample_column(task)
    return sample_level_started_column(task,provisional=True)


def sample_level_started_column(task,provisional=False):
    name,config_json,speed,seed=task;solver=worker_solver(name,config_json)
    started=time.monotonic()
    # The first requested speed is often below the connected 1-g branch.
    # A bounded attempt is enough to seed its separate active-limit solve.
    # It is not evidence of infeasibility: if that limit cannot be verified,
    # compute_adaptive repeats this column with the complete level search.
    point=(solver.solve(speed,1.,exhaustive=False,quick=True) if provisional else
           solve_level(solver,speed))
    if point['valid']:return sample_column((name,config_json,speed,[point]+(seed or [])))
    if provisional:
        return dict(speed_kmh=speed,points=[point],boundary=None,lower_boundary=None,sustained=[],load_checks=[],
            boundary_bracket_g=None,boundary_reason='Provisional level-flight search',boundary_status='no feasible samples',
            numerical_gap_brackets=[],interior_failures=[],unresolved_load_intervals=[],level_probe_only=True,
            elapsed_s=time.monotonic()-started,mass=solver.mass,engine=solver.engine.summary)
    if point['converged'] and not set(point['reasons'])&{'post-stall','Instructor pitch limit'}:
        return sample_column(task)
    if (point['stall_margin_deg']<.15 and point['force_error_g']<.15
            and point['angular_error_rad_s2']<.01):
        return dict(speed_kmh=float(speed),points=[point],boundary=None,lower_boundary=None,
            sustained=[],load_checks=[],boundary_bracket_g=None,
            boundary_reason='Outside the connected branch from level flight',
            boundary_status='no feasible samples',numerical_gap_brackets=[],interior_failures=[],
            unresolved_load_intervals=[],elapsed_s=time.monotonic()-started,
            mass=solver.mass,engine=solver.engine.summary,
            branch_selection=dict(method='level-flight starts unresolved near the lift maximum',limit_verified=False))
    return dict(speed_kmh=speed,points=[point],boundary=None,lower_boundary=None,sustained=[],load_checks=[],
        boundary_bracket_g=None,boundary_reason='Unresolved level-flight equilibrium',boundary_status='no feasible samples',
        numerical_gap_brackets=[],interior_failures=[],unresolved_load_intervals=[],level_probe_only=True,
        elapsed_s=time.monotonic()-started,mass=solver.mass,engine=solver.engine.summary)


def sample_boundary_column(task):
    return _sample_boundary_column(task)


def _sample_boundary_column(task):
    name,config_json,speed,seed=task;started=time.monotonic()
    if os.environ.get('WT_EM_BOUNDARY_CONTINUE','1')=='1' and seed:
        from em_boundary_predictor import continue_limit
        solver=worker_solver(name,config_json)
        continued=continue_limit(solver,speed,seed)
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
                # A nearly saturated cap is a poor Newton start. Interpolate
                # a safely interior state from the two measured branches.
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
                    # A measured rejection makes the shared balanced bracket
                    # available immediately. Otherwise a synthetic endpoint
                    # forces an expensive coupled constraint search first.
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
    """Start the existing active-constraint solve inside the feasible branch.

    Lift/neighbor estimates choose a start only. The returned boundary must
    still close every aircraft equation and satisfy all competing limits.
    Failed predictions fall back to the original load continuation.
    """
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
    # This nearby structural seed helps coupled propeller trim. For jet
    # surfaces retain the established lift predictor: wing loading at 1 g
    # alone is too crude and can trigger costly distant constraint searches.
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
        # The stall angle is a poor seed for a structural limit reached first.
        # Extrapolate the verified low-load trim toward the predicted wing-load
        # cap; the active constraint and final equilibrium remain exact.
        guess=list(low['solution'])
        slope=(low['alpha_deg']-level['alpha_deg'])/(low['load_g']-level['load_g'])
        guess[0]+=slope*(high_load-low['load_g'])
        guess[1]=math.degrees(math.acos(1/high_load))
    high=dict(level,load_g=high_load,valid=False,converged=False,
        reasons=[],solution=guess,continuation_limit_kind=kind,
        wing_load_ratios=[v*high_load for v in level['wing_load_ratios']],
        continuation_seed=dict(level,load_g=target,solution=guess))
    if not solver.is_prop or solver.engine.automatic:
        if near_level_prediction:
            # A balanced rejected sample supplies a scalar physical bracket.
            # Check that inexpensive possibility before a speculative coupled
            # constraint search near the singular level-flight endpoint.
            if solver.config['instructor'] or kind in ('control','wing force'):
                measured=solver.solve(speed,min(high_load,max(1.01,estimate)),guess,
                    exhaustive=False,quick=True)
                if measured['converged'] and measured['reasons'] and set(measured['reasons'])<=PHYSICAL_REASONS:
                    from em_constraint_bracket import refine
                    limit=refine(solver,speed,level,measured)
                    if limit:return limit
            # Solve the competing stall/control equations in turn-rate
            # coordinates. Returning None here used to trigger repeated 2 g
            # searches above an estimated ~1 g boundary on some aircraft.
            from em_level_limit import near_level_boundary
            limit=near_level_boundary(solver,speed,level,high,max_nfev=12)
            if limit:return limit
            # A float32 polar seam can defeat the singularity-free root at one
            # exact speed. Establish one balanced turning state, then solve the
            # same competing active equations in ordinary load coordinates.
            interior=solver.solve(speed,max(1.1,estimate*.85),exhaustive=False)
            if interior['valid']:
                return solver.boundary(speed,interior,high,continuation=False,
                    candidate_kinds={'stall','control'},max_iterations=12,trial_cycle_seconds=20.)
            return None
        if kind=='wing force':
            # A structural cap is almost linear in load after trim. Establish
            # a real balanced outer endpoint before invoking the scalar limit
            # refinement; a synthetic endpoint forces the noisy six-variable
            # active-constraint solver to rediscover the same bracket.
            measured_high=solver.solve(speed,high_load,guess,exhaustive=False)
            if measured_high['converged'] and set(measured_high['reasons'])=={'wing force limit'}:
                measured_high['continuation_limit_kind']='wing force'
                high=measured_high
        # This is a speculative active-limit prediction, not a measured
        # bracket. Solve only the predicted constraint with bounded Newton
        # effort; the ordinary balanced load sweep remains the fallback.
        # The polar/wing estimates do not predict a tail-control stop. Solve
        # that competing physical equation in the same bounded attempt so a
        # missed control cap does not fall through to dozens of load trials.
        limit=solver.boundary(speed,low,high,continuation=False,
                              candidate_kinds={kind,'control'},max_iterations=8,trial_cycle_seconds=20.,scalar_fallback=False)
        if limit or kind!='wing force':return limit
        # The level-flight wing-load ratio is only a linear estimate. Around
        # competing lift and structural limits it can predict wing force first
        # even when the checked stall limit occurs at a slightly lower load.
        # Try that independent active equation before a long load sweep.
        stall_guess=list(level['solution'])
        stall_guess[0]+=level['stall_margin_deg']-.002
        stall_guess[1]=math.degrees(math.acos(1/estimate))
        stall_high=dict(high,load_g=min(64.,max(2.,estimate*1.2)),solution=stall_guess,
                        continuation_limit_kind='stall')
        return solver.boundary(speed,low,stall_high,continuation=False,
                               candidate_kinds={'stall'},max_iterations=8,trial_cycle_seconds=20.)
    return solver.boundary(speed,low,high)


def sample_column(task,boundary_only=False):
    return _sample_column(task,boundary_only)


def _sample_column(task,boundary_only=False):
    started=time.monotonic()
    name,config_json,speed,seed=task
    solver=worker_solver(name,config_json);cfg=solver.config
    if solver.is_prop and solver.engine.automatic:
        # Process workers receive speed tasks in nondeterministic order. A
        # drivetrain state left by an unrelated, distant column can take the
        # short warm-start budget down the wrong transient and make runtime or
        # completeness depend on which worker happened to receive the task.
        # Reset to this task's explicit neighboring seed (or a cold start),
        # independently of worker scheduling. The full native convergence
        # certificate is still required at the new speed and every load.
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
    samples={};tol=cfg['sep_tolerance_mps'];checks=[];recovery_attempted=set()
    # A few callers (low-speed and Ps=0 edge solves) already hold an
    # independently certified point at this exact speed. Reuse that evidence
    # instead of immediately solving the identical level condition again.
    for point in seed or []:
        if (point.get('valid') and point.get('speed_kmh')==speed):
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
                    initial=(np.asarray(lo['solution'])*(1-t)+np.asarray(hi['solution'])*t).tolist()
                    # Bank is strongly curved near 1 g. Interpolate its small
                    # departure from the coordinated-turn estimate instead of
                    # blending a level bank with an 85-degree boundary.
                    nominal=lambda load:math.degrees(math.acos(1./max(1.,load)))
                    initial[1]=nominal(n)+(lo['solution'][1]-nominal(lo['load_g']))*(1-t)+(hi['solution'][1]-nominal(hi['load_g']))*t
                elif good:initial=min(good,key=lambda p:abs(p['load_g']-n))['solution']
                elif seed:
                    nearby=min(seed,key=lambda p:abs(p['load_g']-n))
                    # A neighboring stall endpoint has an almost singular
                    # lift derivative. It is a poor level-flight start once
                    # speed has moved appreciably; use the independent lift
                    # estimate in that case instead of exhausting Newton.
                    if not (n==1. and nearby['stall_margin_deg']<.5
                            and abs(nearby['speed_kmh']-speed)>.005*speed):
                        initial=nearby['solution']
            p=(solve_level(solver,speed,initial,exhaustive=exhaustive) if n==1. else
               solver.solve(speed,n,initial,exhaustive=exhaustive,
                            quick=local and solver.is_prop and solver.engine.automatic))
            if n==1. and initial is not None and not p['valid']:
                # At a neighboring stall limit dL/dalpha is nearly zero.
                # That continuation state can trap the 1 g search at maximum
                # lift even though an ordinary pre-stall equilibrium exists.
                # Retry the aircraft's independent lift-based start before a
                # bad seed creates a false hole and many edge refinements.
                independent=solve_level(solver,speed)
                if independent['valid'] or independent['force_error_g']<p['force_error_g']:p=independent
            if (solver.is_prop and solver.engine.automatic and not p['converged']
                    and p['force_error_g']<.2 and p['stall_margin_deg']>1.
                    and p['authority_margin']>.02 and max(p['wing_load_ratios'])<.98):
                # Native float32 polar/controller branches can leave Newton on
                # the low-alpha side of a small coordinate seam even though a
                # nearby zero-sideslip equilibrium exists. Probe that adjacent
                # branch before launching the much costlier sideslip search.
                # Every accepted trial uses the unchanged full trim and final
                # propulsion certificate.
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
            # Native trim/authority can amplify the last few float32 bits of
            # a barely closed state. Resolve the aircraft more accurately
            # before declaring a small authority deficit an interior hole.
            if (cfg['instructor'] and boundary and bottom and bottom['load_g']<n<boundary['load_g']
                    and p['reasons'] and set(p['reasons'])<={'control authority','Instructor pitch limit'}
                    and p['authority_margin']>-.001 and (p.get('instructor') or {}).get('envelope_margin',0.)>-.001):
                p=solver.solve(speed,n,p['solution'],exhaustive=True,refine=True)
                searched_exhaustively=True
            # Search failures near closure deserve the complete rounding-branch
            # recovery before they can remove a visible piece of the envelope.
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
    # Follow the branch from level flight until its first rejected point. Above
    # this boundary is outside the connected pre-stall envelope being plotted.
    previous=None;boundary=None;bracket=None;bottom=None;previous_invalid=None;connected_trace=None
    certified=next((p for p in samples.values() if p.get('_checked_probe_boundary')),None)
    # A failed interior holdout invalidates the surface approximation, not an
    # independently balanced active boundary at this exact speed. Preserve
    # that physical evidence when upgrading a probe to a complete column.
    # The ordinary interior checks and connected-branch selection still run.
    known_boundary=certified or next((p for p in samples.values() if p.get('envelope_limit')),None)
    known_floor=next((p for p in samples.values() if p.get('_checked_probe_lower_boundary')),None)
    if cfg['max_load_g'] is None:
        # Grow the search until a real constraint is bracketed. The numerical
        # guard is never reported as a physical aircraft boundary.
        search_loads=[1.]
        while search_loads[-1]<64.:
            search_loads.append(min(64.,search_loads[-1]+max(1.,search_loads[-1]*.35)))
    else:search_loads=np.linspace(1.,cfg['max_load_g'],max(5,(cfg['load_samples']+1)//2))
    if known_boundary and known_floor and known_floor['load_g']<=known_boundary['load_g']:
        bottom=known_floor;boundary=known_boundary;search_loads=[]
    for n in search_loads:
        p=solve(n)
        if (not p['valid'] and previous is not None and solver.is_prop and
                set(p['reasons'])<= {'propulsion did not settle','propulsion cycle unresolved'} and
                p['stall_margin_deg']>1. and p['authority_margin']>.02):
            # Engine-cycle convergence can depend strongly on the incoming
            # state. A rejected coarse jump is not a physical load limit:
            # approach the same target through balanced intermediate states.
            # Keep each failed point as evidence if shorter continuation fails.
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
        # A balanced Instructor rejection at 1 g is a possible lower
        # boundary, not evidence that the entire speed column is infeasible.
        # Continue the ordinary physical load search and bracket the floor
        # when higher-load equilibria become permitted.
        if (n==1. and not p['valid'] and p['stall_margin_deg']<.15
                and p['force_error_g']<.15 and p['angular_error_rad_s2']<.01):
            # solve_level has already tried an independent pre-stall start.
            # When both end at the level-flight lift maximum, higher-load
            # roots are disconnected from the plotted level-flight envelope.
            # Avoid fourteen full trim solves at every speed probe below the
            # stall-speed edge.
            return dict(speed_kmh=float(speed),points=[p],boundary=None,lower_boundary=None,
                sustained=[],load_checks=[],boundary_bracket_g=None,
                boundary_reason='Outside the connected branch from level flight',
                boundary_status='no feasible samples',numerical_gap_brackets=[],interior_failures=[],
                unresolved_load_intervals=[],elapsed_s=time.monotonic()-started,
                mass=solver.mass,engine=solver.engine.summary,
                branch_selection=dict(method='level-flight starts unresolved near the lift maximum',limit_verified=False))
        if n==1. and p['valid'] and known_boundary:
            boundary=known_boundary;bottom=p;break
        if (n==1. and p['valid'] and not solver.is_prop and cfg['max_load_g'] is None
                and any(q.get('envelope_limit',{}).get('kind') in ('trim fold','pitch response') for q in seed or [])):
            # A neighboring fold identifies a singular load coordinate.
            # Continue the balanced branch before attempting distant active
            # constraints and sideslip restarts that cannot remove that fold.
            from em_branch import connected_limit
            traced=connected_limit(solver,speed,p,seed)
            if traced:
                connected_trace=traced
                samples={q['load_g']:q for q in traced['points']}
                boundary=traced['boundary'];bottom=p
                for seam in traced.get('seams',[]):samples[seam['failed']['load_g']]=seam['failed']
                break
        if (n==1. and p['valid'] and cfg['max_load_g'] is None and
                (solver.is_prop and solver.engine.automatic or
                 not solver.is_prop and any(p.get('envelope_limit') for p in (seed or [])))):
            direct=predict_limit(solver,speed,p,seed)
            if direct is None and seed:
                # A neighbor can supply the wrong active constraint across a
                # stall/structural transition. Retry from this aircraft's
                # current lift and wing-load estimates before sweeping loads.
                direct=predict_limit(solver,speed,p,None)
            if direct:
                samples[direct['load_g']]=direct;bottom=p;boundary=direct
                break
        if (n==1. and p['valid'] and solver.is_prop and solver.engine.automatic and not cfg['instructor']
                and seed and max(q['load_g'] for q in seed)<1.8
                and any(q.get('envelope_limit',{}).get('kind')=='stall' for q in seed)):
            # A neighboring verified stall edge supplies a continuation seed.
            # Solve that active constraint directly before requesting 2 g,
            # which can be far above this column's ~1 g lift capability. That
            # rejected 2 g optimizer otherwise spends minutes seeking a root
            # which is unnecessary once a nearer physical limit is verified.
            from em_level_limit import near_level_boundary
            previous_limit=max((q for q in seed if q.get('envelope_limit',{}).get('kind')=='stall'),key=lambda q:q['load_g'])
            hint=dict(p,load_g=2.,valid=False,converged=False,reasons=[],continuation_seed=previous_limit)
            direct=near_level_boundary(solver,speed,p,hint)
            if direct:
                samples[direct['load_g']]=direct;bottom=p;boundary=direct
                break
        # Sweep availability depends on Mach, not trim/load. An unavailable
        # fixed sweep excludes this speed column for a physical reason.
        if 'sweep unavailable' in p['reasons']:break
        if 'Instructor boundary unresolved' in p['reasons']:break
        if (not solver.is_prop and not p['converged'] and p['stall_margin_deg']>2. and
                max(abs(v) for v in p['commands'])<.95):
            # A failed search away from every active limit is not an envelope
            # edge. Retry independently before ending continuation at it.
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
                        # Predictions are starts only. Each endpoint and the
                        # lower bracket are measured with the full equations.
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
                    # An isolated interior numerical failure does not prove an
                    # upper envelope. Keep searching and localize the gap later.
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
        # An unbalanced coarse endpoint provides no evidence of which physical
        # constraint was crossed. Localize with balanced continuation first;
        # this also gives the active-limit solve a nearby state rather than a
        # distant post-stall state that can consume hundreds of engine cycles.
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
        for _ in range(0 if direct else 11):
            if high['load_g']-low['load_g']<=.004:break
            p=solve((low['load_g']+high['load_g'])/2,low['solution'])
            if numerical_failure(p):
                if not solver.is_prop:
                    probe=solve((p['load_g']+high['load_g'])*.5,low['solution'],exhaustive=True)
                    if probe['valid']:low=probe;continue
                    if (probe['converged'] or probe['stall_margin_deg']<=1. or
                            probe['authority_margin']<=.02 or max(probe['wing_load_ratios'])>=.98):
                        high=probe;continue
                    break
                # A failed midpoint cannot lower a physical upper bracket.
                # Probe both sides: a valid lower-quarter state often steps
                # past an isolated engine or native-polar rounding failure.
                # Every accepted advance is a fresh full-aircraft equilibrium.
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
            # Numerical bisection can replace a balanced physical rejection
            # with a failed interior solve. Keep the measured constraint as
            # the upper endpoint: the failure is still in samples and cannot
            # establish a smaller flight envelope.
            certified=[p for p in samples.values() if p['load_g']>low['load_g']
                and p['converged'] and p['reasons'] and set(p['reasons'])<=physical_reasons]
            if certified:high=min(certified,key=lambda p:p['load_g'])
        physical_rejection=high['converged'] and bool(high['reasons']) and set(high['reasons'])<=physical_reasons
        if not direct and (not solver.is_prop or physical_rejection):
            direct=solver.boundary(speed,low,high)
            if direct:samples[direct['load_g']]=direct;low=direct
        if not direct and solver.is_prop:
            # The rejected side of a tight bracket can remain a few force
            # ulps from closure even as the valid side reaches a real control,
            # stall, or structural limit. Its lack of convergence is not a
            # reason to discard the active constraint visible on the balanced
            # side. The coupled boundary solve must still close all aircraft
            # equations and land inside this measured bracket.
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
            # A failed intermediate trim is not an upper envelope. Other
            # verified samples from this same column may already lie above
            # it. Start the stall equation from the highest nearby verified
            # pre-stall state, retaining failed samples as separate gap
            # evidence instead of truncating the plotted branch at the first
            # search failure.
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
    branch_selection=None;branch_search_points=[]
    if connected_trace:
        branch_search_points=connected_trace['rejected']
        branch_selection=dict(method=connected_trace['method'],
            trace_loads_g=[p['load_g'] for p in connected_trace['points']],
            limit_verified=not connected_trace.get('unresolved',False),
            selection='continuation from level flight; no ranking by SEP')
    if boundary and boundary['load_g']==1. and not boundary.get('envelope_limit'):
        from em_level_limit import level_constraint_point
        endpoint=level_constraint_point(boundary)
        if endpoint:
            boundary=endpoint;samples[1.]=endpoint;bottom=endpoint;bracket=None
    if (not connected_trace and not certified and (not solver.is_prop or cfg['instructor']) and bottom and bottom['load_g']==1.
            and boundary and boundary.get('envelope_limit',{}).get('method')!='balanced level-flight active constraint'
            and boundary and boundary.get('envelope_limit',{}).get('kind') in (None,'stall','control')):
        from em_branch import connected_limit
        traced=connected_limit(solver,speed,bottom,seed)
        if traced:
            branch_search_points=list(samples.values())+traced['rejected']
            samples={p['load_g']:p for p in traced['points']}
            for seam in traced.get('seams',[]):
                marker=seam['failed']
                samples[marker['load_g']]=marker
            boundary=traced['boundary'];bottom=traced['points'][0];bracket=None
            branch_selection=dict(method=traced['method'],trace_loads_g=[p['load_g'] for p in traced['points']],
                                  limit_verified=not traced.get('unresolved',False),
                                  selection='continuation from level flight; no ranking by SEP',
                                  narrow_numerical_seams=[dict(valid_side_loads=[s['lower']['load_g'],s['upper']['load_g']],
                                                                failed_load_g=s['failed']['load_g']) for s in traced.get('seams',[])])
    def outline_result():
        if bottom:bottom['_checked_probe_lower_boundary']=True
        # These points verify the outline only. They are deliberately kept
        # separate from SEP columns, so they cannot create interpolation holes
        # or masquerade as an independently checked performance surface.
        return dict(speed_kmh=float(speed),points=sorted(samples.values(),key=lambda p:p['load_g']),
            boundary=boundary,lower_boundary=bottom,lower_boundary_searched=True,sustained=[],load_checks=[],
            boundary_bracket_g=[p['load_g'] for p in bracket] if bracket else None,
            boundary_reason=boundary.get('envelope_limit',{}).get('kind') if boundary else None,
            boundary_status='no feasible samples' if boundary is None else
                'verified limit' if boundary.get('envelope_limit') else 'unresolved numerical boundary',
            numerical_gap_brackets=[],interior_failures=[],unresolved_load_intervals=[],
            elapsed_s=time.monotonic()-started,mass=solver.mass,engine=solver.engine.summary,boundary_probe=True,
            branch_selection=branch_selection,branch_search_points=branch_search_points)
    surface_loads=set();holdout_loads=set();pending=[]
    def close_observed_pitch_limit():
        from em_branch import first_pitch_response_limit
        limit=first_pitch_response_limit(solver,speed,bottom,boundary,samples.values())
        if limit is None:return None
        limit['_checked_probe_boundary']=True
        retained=[p for p in samples.values() if p['load_g']<limit['load_g']]+[limit]
        result=sample_column((name,config_json,speed,retained),boundary_only=boundary_only)
        result['branch_search_points'].extend(p for p in samples.values() if p['load_g']>limit['load_g'])
        return result
    closed_pitch=close_observed_pitch_limit()
    if closed_pitch is not None:return closed_pitch
    if boundary_only:
        # The active boundary has already passed full force/moment closure.
        # A speed probe supplies its own independent interior checks below;
        # an outline-only task never uses an interior curve. Solving another
        # nine-point load grid here repeated those checks (at different loads
        # for control/structural edges), without changing either endpoint.
        return outline_result()
    coordinated_retries=set();alpha_holdouts={}
    # The native wing polar changes from its linear section to its curved
    # section at these known angles. A single generic midpoint can lie on
    # the linear side and miss the curvature in the rest of the interval.
    # These angles guide extra physical holdouts only; local wing flow and
    # complete aircraft balance are still evaluated by the ordinary solver.
    from aircraft_model import condition_properties
    guide=boundary or bottom
    polar=condition_properties(solver.model,guide['mach'],guide['flaps_percent']/100.)[1] if guide else {}
    polar_angles=[polar[k]-solver.model['geometry']['incidence'] for k in ('aoaLineL','aoaLineH') if k in polar]
    from em_parameter import interior_point as solve_interior_point
    def alpha_point(solver,speed,left,right):
        """Use existing independent trim evidence before solving a new point.

        A failed speed probe already supplies balanced interior samples. Its
        upgrade formerly solved another interlaced load grid, even when those
        samples could check the very same interpolation interval. Select only
        points in the middle half of this interval; sparse edge iterates cannot
        stand in for an interior observation. No interpolated state is reused
        as physical evidence, and all remaining samples are checked below.
        """
        lo,hi=left['load_g'],right['load_g']
        candidates=[p for n,p in samples.items() if p['valid'] and
                    lo+.25*(hi-lo)<=n<=lo+.75*(hi-lo)]
        if candidates:return min(candidates,key=lambda p:abs(p['load_g']-(lo+hi)*.5))
        return solve_interior_point(solver,speed,left,right)
    def restore_coordinated_neighbors():
        # Sideslip is a numerical fallback, not a different flight condition
        # to alternate with adjacent zero-sideslip samples. Once both nearby
        # coordinated states exist, try their interpolated full trim again.
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
        """Check the actual cubic without inserting successful check points.

        A midpoint which confirms the curve remains independent evidence. It
        becomes a knot only when the error needs refinement. Recheck *all*
        intervals after any insertion, including changed neighboring slopes;
        unchanged midpoints reuse their already solved aircraft equilibrium.
        This avoids repeatedly doubling a grid whose checks already pass.
        """
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
                for left,right in zip(good,good[1:]):
                    lo,hi=left['load_g'],right['load_g']
                    u=float((load_coordinate(lo,top)+load_coordinate(hi,top))*.5)
                    mid=float(coordinate_load(u,top))
                    if not lo<mid<hi:continue
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
                    # A passed midpoint is also an independently solved knot
                    # for the alpha-parameterized speed interpolant.
                    p['alpha_sample']=True
                    error=abs(p['ps_mps']-float(curve(u)))
                    alpha_error=0. # Speed interpolation uses this same checked load curve.
                    checks.append(dict(load_g=mid,error_mps=error,alpha_equivalent_error_mps=alpha_error,
                                       depth=depth,phase=phase))
                    transition=left.get('roll_leveling_branch')!=right.get('roll_leveling_branch')
                    # If only part of an interval is visible, its ordinary
                    # midpoint can be outside the saturated color/contour
                    # range. Check inside the visible part too; ignoring that
                    # midpoint alone leaves the last visible contour unchecked.
                    extra_fractions=[]
                    angle_span=right['alpha_deg']-left['alpha_deg']
                    if angle_span>1e-5:
                        extra_fractions.extend((angle-left['alpha_deg'])/angle_span for angle in polar_angles
                            if left['alpha_deg']+.1*angle_span<angle<right['alpha_deg']-.1*angle_span)
                    delta_ps=right['ps_mps']-left['ps_mps']
                    if delta_ps:
                        from em_accuracy import visible_range
                        low_ps,high_ps=visible_range(cfg)
                        visible=np.clip(sorted(((low_ps-left['ps_mps'])/delta_ps,
                                                (high_ps-left['ps_mps'])/delta_ps)),0.,1.)
                        if 0.<visible[1]-visible[0]<1.:
                            extra_fractions.extend(float(visible[0]+t*(visible[1]-visible[0]))
                                                   for t in (.25,.75))
                    if transition:extra_fractions.extend((.25,.75))
                    if extra_fractions:
                        # A midpoint can agree by accident across a native
                        # branch change while both sides are under-resolved.
                        # Check each side of that interval independently; the
                        # ordinary evidence pass below inserts only failures.
                        from em_parameter import _alpha_point
                        for fraction in extra_fractions:
                            branch_key=(*key,fraction)
                            if branch_key not in alpha_holdouts:
                                alpha_holdouts[branch_key]=_alpha_point(solver,speed,left,right,fraction)
                            probe=alpha_holdouts[branch_key]
                            if probe is not None:
                                samples[probe['load_g']]=probe;holdout_loads.add(probe['load_g'])
                                probe['alpha_sample']=True
                    # A native switch is fitted piecewise. Judge its remaining
                    # bracket by the same SEP/contour-position checks as every
                    # other interval, rather than forcing unconditional load
                    # bisections even when the jump is below the SEP target.
                    from em_accuracy import neighboring_loads,turn_tolerance,within_contour_band,visible_error
                    query=neighboring_loads(speed,np.array([mid]),turn_tolerance(tol))[0]
                    query=np.clip(query,good[0]['load_g'],good[-1]['load_g'])
                    geometric=bool(within_contour_band(p['ps_mps'],curve(load_coordinate(query,top)),tol))
                    refine=error>tol and not geometric and hi-lo>.0002 and visible_error(p['ps_mps'],float(curve(u)),cfg)
                    checks[-1]['within_contour_tolerance']=geometric
                    checks[-1]['native_transition']=transition
                    if refine:
                        insert.append(mid);remaining.append((lo,hi))
                # Reuse all already balanced search/probe points as holdouts.
                # Only a failed check inserts one into the fitted stencil.
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
            # Do not export a newly changed curve without checking it. If the
            # budget expires, retain its checked predecessor and report the
            # unresolved intervals, just as numerical failures are retained.
            if depth<6:surface_loads.update(insert)
        return remaining
    if boundary and boundary['load_g']>1.:
        top=boundary['load_g'];start=float(load_coordinate(bottom['load_g'],top))
        # Boundary-search iterates are independent evidence, not interpolation
        # knots. Nearly identical iterates have float32 closure noise and can
        # destroy an otherwise smooth cubic if every one is inserted.
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
        # Native holdouts near both ends cover the turn-coordinate floor
        # and the steep maximum-lift neighborhood independently.
        if top-bottom['load_g']>.05:
            for n in (bottom['load_g']+min(.1,.01*(top-bottom['load_g'])),top-.005*(top-bottom['load_g'])):
                solve(float(n))
        # Failed points always break the surface. The nearest valid states on
        # either side retain the measured extent of each branch.
        ordered=sorted(samples.values(),key=lambda p:p['load_g'])
        for i,point in enumerate(ordered):
            if not point['valid']:
                surface_loads.update(p['load_g'] for p in ordered[max(0,i-1):i+2])
        holdout_loads.update(set(samples)-surface_loads)
        pending=refine_surface('independent midpoint checks')
    closed_pitch=close_observed_pitch_limit()
    if closed_pitch is not None:return closed_pitch
    # If an interior search failed, repeat it using the now-established nearby
    # equilibria before classifying it as a numerical hole. Points below the
    # measured lower boundary are outside this interval, not interior gaps.
    # Retrying them at sixteen sideslips used to dominate fixed-flap columns.
    recovery_candidates={n for n,p in samples.items() if not p['valid']}
    for n,p in list(samples.items()):
        if p.get('native_seam_marker'):continue
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
    # Use another set of numerical coordinates for failures bracketed by real
    # interior equilibria. No force model or acceptance tolerance changes, and
    # a scalar sign change across a native discontinuity is never accepted.
    for n,p in list(samples.items()):
        if p.get('native_seam_marker'):continue
        if (not solver.is_prop and boundary and bottom and
                bottom['load_g']<n<boundary['load_g'] and not p['converged']):
            recovered=recover_equilibrium(solver,speed,n,list(samples.values()))
            if recovered:
                recovered['recovery_method']='bank/control balance followed by alpha force closure'
                samples[n]=recovered
    # Only recovery changes the checked knot set at this stage. Successful
    # midpoint checks above were not inserted, so an unchanged curve does not
    # need a second, redundant grid of new aircraft solutions.
    if boundary and boundary['load_g']>1.:
        recovered={n for n in recovery_candidates if samples[n]['valid'] and n<=boundary['load_g']}
        if recovered:
            surface_loads.update(recovered)
            pending=refine_surface('recovered intervals')
    # A discontinuity or rounding floor may prevent an exact equilibrium at
    # one load. Bound it with new valid solves on both sides instead of blanking
    # the entire coarse interval (or inventing data across the failure).
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
    # Engine-cycle evaluations dominate automatic-prop trim. Resolve masks to
    # a fraction of the displayed load spacing, then spend extra work only if
    # an interior probe discovers another balanced island.
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
    # Recheck the interior for valid islands once per unresolved run. A narrow
    # island can split a run, so the final grouping/edge checks below remain.
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
    closed_pitch=close_observed_pitch_limit()
    if closed_pitch is not None:return closed_pitch
    gap_brackets=[dict(load_g=bad[len(bad)//2]['load_g'],
        valid_side_loads=[left['load_g'],right['load_g']],width_g=right['load_g']-left['load_g'],
        edge_brackets_g=[[left['load_g'],bad[0]['load_g']],[bad[-1]['load_g'],right['load_g']]],
        reasons=sorted(set(reason for p in bad for reason in p['reasons'])))
        for left,bad,right in failed_runs()]
    for left,bad,right in failed_runs():surface_loads.update((left['load_g'],right['load_g']))
    # Root-finder iterates can differ by tiny loads beneath the force-closure
    # tolerance. Their noisy secant slopes must not alter the surface cubic.
    # Keep them as root evidence, separate from the checked surface knots.
    for n,p in samples.items():
        p['surface_sample']=n in surface_loads or not p['valid']
    valid=sorted((p for p in samples.values() if p['valid']),key=lambda p:p['load_g'])
    roots=[]
    for low,high in zip(valid,valid[1:]):
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
        # Native float32 branch noise can spoil the last root-finder iterate
        # after an earlier one already satisfied the physical Ps tolerance.
        # Retain that independently balanced point; do not invent a zero or
        # discard it merely because a later numerical step was unsuccessful.
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
                                'unresolved numerical boundary' if branch_selection and not branch_selection['limit_verified'] else
                                'verified limit' if boundary.get('envelope_limit') else
                                'plot ceiling' if not bracket and cfg['max_load_g'] is not None else 'unresolved numerical boundary',
                lower_boundary=bottom,lower_boundary_searched=True,
                native_transitions=[dict(load_bracket_g=[a['load_g'],b['load_g']],branches=[a.get('roll_leveling_branch'),b.get('roll_leveling_branch')])
                    for a,b in zip(surface_points,surface_points[1:])
                    if a.get('roll_leveling_branch')!=b.get('roll_leveling_branch')] if boundary else [],
                numerical_gap_brackets=gap_brackets,
                interior_failures=[p['load_g'] for p in samples.values() if p['surface_sample'] and boundary and bottom and bottom['load_g']<p['load_g']<boundary['load_g'] and not p['valid']],
                unresolved_load_intervals=pending if boundary and boundary['load_g']>1. else [],
                elapsed_s=time.monotonic()-started,mass=solver.mass,engine=solver.engine.summary,
                branch_selection=branch_selection,branch_search_points=branch_search_points)


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
        # A solved n=1 endpoint is a value, although it has no load curve.
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
    """Interpolate contiguous valid runs with batched, identical PCHIP rows."""
    from em_fixed_load import interpolate
    return interpolate(columns,speeds,loads)


def alpha_curve(column):
    """Load and power parameterized by the solved physical angle of attack.

    No envelope normalization enters these coordinates. Near maximum lift,
    power versus load has a nearly vertical slope, whereas both load and power
    remain well behaved versus alpha. Only monotone, valid solution intervals
    can contribute; a numerical gap still separates intervals.
    """
    if '_alpha_curve' in column:return column['_alpha_curve']
    runs=[];run=[]
    top=column['boundary']['load_g'] if column.get('boundary_reason') in ('stall','trim fold') else None
    cap=column['boundary']['load_g'] if column.get('boundary') else float('inf')
    for p in [p for p in column['points'] if (p.get('surface_sample',True) or p.get('alpha_sample')) and p['load_g']<=cap]+[None]:
        same_branch=not run or p is None or p.get('roll_leveling_branch')==run[-1].get('roll_leveling_branch')
        if p is not None and p['valid'] and run and same_branch and p['alpha_deg']<=run[-1]['alpha_deg']:
            previous=run[-1]
            # At a nearly flat lift maximum, samples separated by less than
            # their force-closure uncertainty can reverse angle order. A
            # redundant knot with <0.1m/s power difference must not force the
            # entire useful column back to a singular fixed-load interpolant.
            # Keep all raw evidence; omit only this indistinguishable knot.
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
    """Close a vanishing load interval with its solved physical endpoints.

    No interior equilibrium is invented at the zero-width endpoint. The
    limiting upper/lower Ps values define a provisional surface, independently
    checked by adaptive speed holdouts before use. Never bridge rejected or
    numerically unresolved columns.
    """
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
    # Carry the resolved column's interior shape into the shrinking interval.
    # Its contribution vanishes at the collapsed endpoint and matches the
    # full neighboring load curve exactly, avoiding a jump at that column.
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
    """Interpolate physical equilibrium branches, then evaluate at fixed load.

    The base interpolation holds physical alpha fixed. An endpoint correction
    makes it agree with each column's checked load curve at its sampled speed.
    Inversion to the requested load handles the steep slope at maximum lift.
    """
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
        # Use the same local cubic endpoint slopes as the regular surface,
        # retaining continuity at level flight and the limiting boundary.
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
        # A native piecewise seam can split an otherwise usable column into
        # several valid alpha runs. Match the same run on both sides of this
        # speed cell; the gap between runs remains masked. Falling back to
        # fixed-load interpolation for the entire column is inaccurate near
        # the steep maximum-lift slope and causes futile speed refinement.
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
            # Endpoints are inserted separately below. Keeping a common-alpha
            # endpoint as well creates a knot infinitesimally beside it as
            # speed approaches a solved column, changing PCHIP's endpoint
            # slope by a finite amount. Use strictly interior branch knots.
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
            # Each run has its own directly solved endpoints. Interpolating
            # those endpoints separately avoids stretching across a seam.
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
            # Alpha interpolation and the checked load interpolant are two
            # different representations of each column. Switching between
            # them at an exact speed knot used to leave finite SEP jumps and
            # needle-shaped contours. Make the alpha surface interpolate the
            # checked column curves, by blending its endpoint representation
            # errors across this speed cell. Each correction stays inside its
            # matching valid run; no raw equilibrium value is modified.
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
    """Continue physical angle branches and retain the solved edge vertices."""
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
    """Independent boundary and interior checks, without an unused full curve."""
    name,config_json,speed,seed=task;started=time.monotonic()
    column=sample_boundary_column(task)
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
        # Some fixed-flap envelopes start above 1 g. An upper-edge probe has
        # not located that floor. Search it once, carrying this already solved
        # cap; never overwrite a valid floor with the rejected level state.
        column=sample_column((name,config_json,speed,[p for p in samples.values() if p['valid']]),boundary_only=True)
        column['speed_probe']=True
        samples={p['load_g']:p for p in column['points']}
        if not column.get('boundary') or not column.get('lower_boundary'):return column
        top=column['boundary']['load_g']
    bottom=column.get('lower_boundary')
    if not bottom or not bottom['valid']:return column
    floor=bottom['load_g']
    if top>floor:
        fractions=np.linspace(0.,1.,9 if solver.config['instructor'] else 7)
        # Both limits define the sampling interval. A raised lower control
        # limit is as real as the upper stall/structural limit.
        start=float(load_coordinate(floor,top))
        loads=(floor+fractions*(top-floor) if column.get('boundary_reason') in ('wing force','control')
               else coordinate_load(start+fractions*(1.-start),top))
        for n in loads[1:-1]:
            n=float(n)
            if n not in samples:samples[n]=solver.solve(speed,n,initial(n),exhaustive=False,quick=True)
    if top-floor>.05:
        # A probe tied only to this new column's cap can move with an
        # erroneous envelope interpolant and miss the neighboring fixed-load
        # strip. Check the top of the shared feasible load range as well as
        # the independently solved actual boundary. This avoids refining an
        # entire speed column merely to test that same narrow strip later.
        groups={}
        for p in seed or []:
            if p['valid']:groups.setdefault(p['speed_kmh'],[]).append(p['load_g'])
        common_top=min([top]+[max(ns) for ns in groups.values()])
        near_top=common_top-.005*(common_top-floor) if common_top-floor>.05 else top-.005*(top-floor)
        for n in (floor+min(.1,.01*(top-floor)),near_top):
            if n not in samples:samples[n]=solver.solve(speed,n,initial(n),exhaustive=False,quick=True)
    # A checked point outside the plotted SEP range cannot validate the last
    # visible contour before it. Include a physical trim inside the visible
    # part of every interval crossing a color/contour saturation edge, just as
    # full-column refinement does. No work is added to wholly visible spans.
    ordered=sorted((p for p in samples.values() if p['valid'] and floor<=p['load_g']<=top),
                   key=lambda p:p['load_g'])
    for left,right in zip(ordered,ordered[1:]):
        delta=right['ps_mps']-left['ps_mps']
        if not delta:continue
        from em_accuracy import visible_range
        low_ps,high_ps=visible_range(solver.config)
        visible=np.clip(sorted(((low_ps-left['ps_mps'])/delta,(high_ps-left['ps_mps'])/delta)),0.,1.)
        if not 0.<visible[1]-visible[0]<1.:continue
        for t in (.25,.75):
            fraction=float(visible[0]+t*(visible[1]-visible[0]))
            n=left['load_g']*(1-fraction)+right['load_g']*fraction
            if n in samples:continue
            guess=[a*(1-fraction)+b*fraction for a,b in zip(left['solution'],right['solution'])]
            guess[1]=math.degrees(math.acos(1./n))
            samples[n]=solver.solve(speed,n,guess,exhaustive=False,quick=True)
        # The last visible contour is itself a required holdout. Fractional
        # samples can all lie well inside the color range while the largest
        # interpolation error sits just before saturation. Locate that native
        # SEP value with a few safeguarded secants, rather than densifying the
        # entire speed/load grid or certifying it from invisible samples.
        for edge in (low_ps,high_ps):
            target=edge-math.copysign(.1*solver.config['sep_tolerance_mps'],edge)
            if not min(left['ps_mps'],right['ps_mps'])<target<max(left['ps_mps'],right['ps_mps']):continue
            lo,hi=left,right
            for _ in range(6):
                fraction=(target-lo['ps_mps'])/(hi['ps_mps']-lo['ps_mps'])
                fraction=max(.02,min(.98,fraction))
                n=lo['load_g']*(1-fraction)+hi['load_g']*fraction
                guess=[a*(1-fraction)+b*fraction for a,b in zip(lo['solution'],hi['solution'])]
                guess[1]=math.degrees(math.acos(1./n))
                point=solver.solve(speed,n,guess,exhaustive=False,quick=True);samples[n]=point
                if not point['valid'] or abs(point['ps_mps']-target)<.002:break
                if (point['ps_mps']-target)*(lo['ps_mps']-target)>0.:lo=point
                else:hi=point
    column['points']=sorted(samples.values(),key=lambda p:p['load_g'])
    if not solver.is_prop:
        column['longitudinal_speed_knots_kmh']=sorted({float(v)*3.6
            for unit in solver.engine.units for v in unit.jet['speed']})
    column['elapsed_s']=time.monotonic()-started
    if all(p['valid'] for p in column['points'] if floor<=p['load_g']<=top):
        column['boundary']['_checked_probe_boundary']=True
    return column


def sample_interior_speed_probe(task):
    """One additional fixed-load check across a broad speed interval."""
    name,config_json,speed,seed,load=task;started=time.monotonic()
    solver=worker_solver(name,config_json);groups={}
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
    from em_accuracy import turn_tolerance,speed_tolerance,visible_range
    from em_solver import AIRCRAFT,BACKEND,aircraft_settings
    from instructor_envelope import profile
    from aircraft_catalog import load
    from em_speed_limits import speed_limits
    start=time.monotonic();cfg_json=json.dumps(config,sort_keys=True)
    conditions={name:aircraft_settings(config,name) for name in config['aircraft']}
    redlines={name:speed_limits(load(name),cfg) for name,cfg in conditions.items()}
    workers=WORKERS;results={name:{} for name in config['aircraft']}
    output=dict(settings=config,aircraft=[],sampling='adaptive',backend=BACKEND,workers=workers,
        loads_g=[],method='near-coordinated trim below positive stall; native-step Ps; mean settled engine; adaptive cubic surface',
        assumptions=['Full-real manual aerodynamic trim','Positive-AoA stall enforced; negative-AoA stall not an exclusion; native aerodynamics unchanged','Full pilot authority; aerodynamic control-power loss retained',
                     'Constant fuel and intact components','Propeller torque/gyro selected per aircraft: off for RB by default, on for SB; axial propwash retained','Still air; out of ground effect; retractable gear and airbrakes stowed; fixed propeller-aircraft gear retains its drag',
                     'Propellers: automatic or idealized manual engine management selected per aircraft; closed radiators and frozen boost supply; complete aircraft phase outputs averaged; manual global optimum and periodic flight trajectory not certified',
                     'Zero sideslip preferred; failed interior equilibria may use solved sideslip up to 2 degrees, with unchanged force/moment closure; not a minimum-drag sideslip optimization',
                     'Requested flap percentage held at every operating point, assumed achievable; intact flaps, no travel time or damage',
                     'Steady Instructor AoA schedule approximation: native Mach/flap/sweep angle targets, settled wing-angle adjustments, native rate feedback and reduced moment balance, with full physical trim and control-power loss. Same constraint at boundary and interior. Transient overshoot, delay, retained trim and overload reserve/release are omitted',
                     'Manual fixed sweep; native reachability limits retained; 0% forward, 100% aft',
                     'VTOL, reverse and thrust-vectoring commands zero; rocket boosters off',
                     'Normal controllable flight: native pitch response must retain the normal elevator direction',
                     'Connected feasible branch from level flight at each speed; extra mass at configured CG'],
        validation='Reconstructed kernels have native-code comparisons; this EM solver has not been validated against live flight.')
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
                instructor_approximation=profile(load(name)) if conditions[name]['instructor'] else None,
                boundary_columns=[outline_column(c) for c in sorted(
                    {**boundary_probes[name],**{c['speed_kmh']:c for c in ordered}}.values(),key=lambda c:c['speed_kmh'])],
                sustained=[p for c in ordered for p in c['sustained']],
                speed_limit=redlines[name],sweep_excluded_speeds_kmh=exclusions[name],
                mass=first['mass'] if first else {},engine=first['engine'] if first else {'policy':'Waiting for solved samples'},
                valid_points=sum(p['valid'] for p in points),converged_points=sum(p['converged'] for p in points)))
        # Rendering runs on another thread; it must not mutate the sampler's
        # live interpolation knots or race with subsequent recovery.
        preview(clone(snapshot));preview_time=now;published_version=preview_version
    def check_cancel():
        if cancelled and cancelled():raise InterruptedError('Calculation cancelled')
    with process_pool() as (pool,worker_cancel):
        prefetched={};prefetch_seen=set();prefetch_used=0
        def prefetch_initial_checks():
            # As initial neighbors finish, use idle workers for the same
            # independent midpoint job that the error checker will request.
            # Defer intervals needing a new physical corner or Ps=0 endpoint;
            # those endpoints must split the interval first. No prefetched
            # value enters the surface before the normal holdout acceptance.
            from em_surface import eligible
            for name,grid in speed_grids.items():
                columns=results[name]
                for lo,hi in zip(grid,grid[1:]):
                    if lo not in columns or hi not in columns:continue
                    left,right=columns[lo],columns[hi]
                    if not all(eligible(c) for c in (left,right)):continue
                    if left['boundary_reason']!=right['boundary_reason']:continue
                    levels=[next((p for p in c['points'] if p['load_g']==1. and p['valid']),None) for c in (left,right)]
                    if not all(levels) or levels[0]['ps_mps']*levels[1]['ps_mps']<0.:continue
                    mid=(lo+hi)*.5
                    seed=[p for c in (left,right) for p in c['points'] if p['valid']]
                    local=(name,json.dumps(conditions[name],sort_keys=True),mid,seed)
                    seed_key=[(p['load_g'],p['solution'],p.get('sideslip_attitude_deg',0.),p.get('envelope_limit'),p.get('_propulsion_seed')) for p in seed]
                    key=('sample_speed_probe',name,local[1],mid,json.dumps(seed_key,separators=(',',':')))
                    if key in prefetch_seen or key in _COLUMN_CACHE:continue
                    prefetch_seen.add(key)
                    prefetched[key]=pool.submit(sample_with_history,sample_speed_probe,local,dict(entry_histories[name]))
        def run(tasks,phase,worker=sample_column):
            nonlocal cache_hits,preview_version
            futures={};keys={};cached=[]
            for task in tasks:
                name=task[0]
                local=(name,json.dumps(conditions[name],sort_keys=True),*task[2:])
                key=None
                if worker in (sample_column,sample_initial_column,sample_boundary_column):
                    seed=[(p['load_g'],p['solution'],p.get('sideslip_attitude_deg',0.),
                           p.get('envelope_limit'),p.get('_propulsion_seed')) for p in task[3]] if task[3] else None
                    key=(worker.__name__,name,local[1],task[2],json.dumps(seed,separators=(',',':')))
                if key is not None and key in _COLUMN_CACHE:
                    _COLUMN_CACHE.move_to_end(key);cached.append((task,clone(_COLUMN_CACHE[key])));cache_hits+=1
                else:
                    future=pool.submit(sample_with_history,worker,local,dict(entry_histories[name]));futures[future]=task;keys[future]=key
            done=0;pending=set(futures);reported=0.
            destination=boundary_probes if worker is sample_boundary_column else results
            for task,column in cached:
                destination[task[0]][column['speed_kmh']]=column;done+=1
                preview_version+=1
            publish_preview()
            try:
                while pending:
                    check_cancel()
                    ready,pending=wait(pending,timeout=.25,return_when=FIRST_COMPLETED)
                    for future in ready:
                        task=futures[future];column,history=future.result();done+=1
                        for speed,entry in history.items():entry_histories[task[0]].setdefault(speed,entry)
                        if column is not None:
                            destination[task[0]][column['speed_kmh']]=column
                            preview_version+=1
                            key=keys[future]
                            if key is not None:
                                _COLUMN_CACHE[key]=clone(column)
                                if len(_COLUMN_CACHE)>256:_COLUMN_CACHE.popitem(last=False)
                        if progress:progress(dict(done=done,total=len(tasks),aircraft=task[0],speed_kmh=column['speed_kmh'] if column else None,phase=phase,elapsed_s=time.monotonic()-start))
                    if worker is sample_initial_column:prefetch_initial_checks()
                    publish_preview()
                    if progress and time.monotonic()-reported>1.:
                        progress(dict(done=done,total=len(tasks),phase=phase,elapsed_s=time.monotonic()-start));reported=time.monotonic()
            except BaseException:
                worker_cancel.set()
                for f in futures:f.cancel()
                raise
        exclusions={name:sweep_speed_intervals(name,conditions[name]) for name in results}
        speed_grids={}
        for name in results:
            upper=min(config['speed_max_kmh'],redlines[name]['sample_speed_kmh']) if redlines[name]['enforced'] else config['speed_max_kmh']
            speeds=(np.linspace(config['speed_min_kmh'],upper,config['speed_samples']).tolist()
                    if upper>config['speed_min_kmh'] else [config['speed_min_kmh']])
            for interval in exclusions[name]:
                for edge in interval:
                    # Bracket the float32 Mach edge to .02km/h, independently
                    # of the display grid and its finite refinement budget.
                    speeds.extend(edge+offset for offset in [-.01,.01]
                                  if config['speed_min_kmh']<edge+offset<upper)
            speed_grids[name]=sorted(set(speeds))
        run([(name,cfg_json,v,None) for name,speeds in speed_grids.items() for v in speeds],'Solving feasible speed columns',sample_initial_column)
        edge_tasks=[]
        for name,columns in results.items():
            ordered=[columns[v] for v in sorted(columns)]
            first=next((i for i,c in enumerate(ordered) if c['boundary_status']=='verified limit'),None)
            if (first is not None and first>0 and ordered[first-1]['boundary_status'] in ('no feasible samples','level probe unresolved')
                    and ordered[first]['boundary_reason'] in ('stall','Instructor pitch','control')):
                edge_tasks.append((name,cfg_json,ordered[first-1],ordered[first],'stall'))
            for left,right in zip(ordered,ordered[1:]):
                kinds={c['boundary_reason'] for c in (left,right)}
                if (all(c['boundary_status']=='verified limit' for c in (left,right)) and
                        (kinds=={'stall','wing force'} or
                         'Instructor pitch' in kinds and bool(kinds & {'wing force','control'}))):
                    edge_tasks.append((name,cfg_json,left,right,'corner'))
                levels=[next((p for p in c['points'] if p['valid'] and p['load_g']==1.),None)
                        for c in (left,right)]
                if all(levels) and levels[0]['ps_mps']*levels[1]['ps_mps']<0:
                    edge_tasks.append((name,cfg_json,left,right,'level'))
        # Insert these columns before checking speed interpolation. Late
        # endpoint insertion otherwise changes cubic slopes after holdouts
        # have passed, and serializes another complete column at the end.
        run(edge_tasks,'Solving level-flight endpoints',sample_edge)
        stall_edges={name:next((c['speed_kmh'] for c in cols.values() if c.get('level_stall_endpoint')),None)
                     for name,cols in results.items()}
        fallback=[]
        for name,cols in results.items():
            for speed,column in cols.items():
                if not column.get('level_probe_only'):continue
                if stall_edges[name] is None or speed>=stall_edges[name]:fallback.append((name,cfg_json,speed,None))
                else:
                    column['boundary_status']='below level-flight stall edge'
                    column['boundary_reason']='Outside the connected branch from level flight; higher-load islands not searched'
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
                    # Three independent level trims reduce this bracket fourfold
                    # per worker wave, avoiding a long serial bisection tail.
                    seed=[p for p in columns[hi]['points'] if p['valid']]
                    tasks.extend((name,cfg_json,lo+(hi-lo)*fraction,seed)
                        for fraction in (.25,.5,.75))
                if not tasks:break
                run(tasks,'Refining the low-speed edge',sample_level_edge_probe)
            # Only the closest feasible level probe needs a complete turn column.
            # Interior probe states remain holdout evidence, not surface knots.
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
        # Locate the first level equilibrium before asking for full turn
        # columns below it. A failed direct endpoint root must not trigger
        # serial high-load searches during this one-dimensional speed bracket.
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
        # Level-only probes belong to the endpoint search. An unsuccessful
        # temporary Newton solve above the now-certified endpoint is not a
        # physical exclusion and must not survive as an empty speed column.
        # Recheck with the complete column recovery and the new neighboring
        # equilibria, retaining the original failed probe as diagnostic data.
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
        # A narrow interval can still straddle a continuous control-limit
        # corner. Refine below the horizontal position allowance before
        # retaining an unresolved seam, rather than stopping a full pixel out.
        speed_stop=.25*speed_tolerance(config)
        initial_intervals=intervals
        intervals={name:[] for name in results}
        physical_stall_speed_brackets={name:[] for name in results}
        pending_speed={};ready_speed=deque();scheduled_speed=0;finished_speed=0
        checked_probes={name:{} for name in results};point_check_requests=set()
        def schedule_speed(name,lo,hi,depth,point_load=None,point_speed=None):
            nonlocal cache_hits,scheduled_speed,prefetch_used
            columns=results[name];prior_speeds=tuple(sorted(columns))
            mid=(lo+hi)*.5 if point_load is None else speed_check_position(lo,hi)
            if point_speed is not None:mid=point_speed
            left,right=columns[lo],columns[hi]
            cap=min(c['boundary']['load_g'] if c['boundary'] else 1. for c in (left,right))
            floor=max(c['lower_boundary']['load_g'] if c.get('lower_boundary') else 1. for c in (left,right))
            loads=floor+fractions*max(0.,cap-floor)
            # Capture the predictor before submitting the independent midpoint
            # solve. Completed siblings may improve later predictors but a
            # point can never validate an interpolant that already contains it.
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
            seed_key=[(p['load_g'],p['solution'],p.get('sideslip_attitude_deg',0.),p.get('envelope_limit'),p.get('_propulsion_seed')) for p in seed]
            key=(worker.__name__,name,local[1],mid,json.dumps(seed_key,separators=(',',':')))
            if point_load is not None:key=(*key,point_load)
            scheduled_speed+=1
            if key in _COLUMN_CACHE:
                _COLUMN_CACHE.move_to_end(key);cache_hits+=1
                ready_speed.append((context,clone(_COLUMN_CACHE[key]),{},None))
            else:
                future=prefetched.pop(key,None)
                if future is None:future=pool.submit(sample_with_history,worker,local,dict(entry_histories[name]))
                else:prefetch_used+=1
                pending_speed[future]=(context,key)
        def accept_speed(context,column,history,key):
            nonlocal preview_version,finished_speed,scheduled_speed,cache_hits
            name,lo,hi,mid,depth,pred,loads,predicted_cap,prior_speeds=context
            for speed,entry in history.items():entry_histories[name].setdefault(speed,entry)
            if key is not None:
                _COLUMN_CACHE[key]=clone(column)
                if len(_COLUMN_CACHE)>256:_COLUMN_CACHE.popitem(last=False)
            # Compare directly with solved interior holdouts, not a second
            # interpolant. The predictor snapshot excludes this midpoint.
            left,right=results[name][lo],results[name][hi]
            from em_surface import envelope_limits
            source=[results[name][v] for v in prior_speeds]
            predicted_limits=envelope_limits(source,[mid])[0]
            holdouts=[p for p in column['points'] if p['valid'] and column.get('boundary') and
                column.get('lower_boundary') and column['lower_boundary']['load_g']<=p['load_g']<=column['boundary']['load_g']]
            query=loads.copy()
            if holdouts:
                loads=np.array([p['load_g'] for p in holdouts]);query=loads.copy()
                # The active boundary is a separately checked coordinate.
                # Evaluate its SEP at the predicted boundary, not at an offset
                # load on a nearly vertical Ps(n) curve. Interior samples stay
                # at their original physical load.
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
            candidates=np.flatnonzero(finite & (abs(actual-pred)>config['sep_tolerance_mps']) & ~geometric)
            if len(candidates):
                box=surface_band_values(source,mid,query[candidates],config)
                geometric[candidates]|=within_contour_band(actual[candidates],box,config['sep_tolerance_mps'])
            failed=finite&(abs(actual-pred)>config['sep_tolerance_mps'])&~geometric&visible_error(actual,pred,config)
            # Do not let a broad speed cell spend its entire position budget
            # on a large SEP error. Refine until the two interpolation stages
            # meet their combined scalar budget, or the remaining speed cell
            # is itself only a few display cells wide. At that scale retain
            # the position test: inserting speeds cannot cure an endpoint
            # load curve's already accepted geometric interpolation error.
            if hi-lo>4.*speed_tolerance(config):
                failed|=finite&(abs(actual-pred)>2.*config['sep_tolerance_mps'])&visible_error(actual,pred,config)
            caps=[c['boundary']['load_g'] if c['boundary'] else 0. for c in (left,column,right)]
            if np.isfinite(predicted_limits[1]):predicted_cap=predicted_limits[1]
            # A numerical stopping load is not a physical envelope value.
            # Comparing such failed searches as boundary curvature drives
            # repeated full columns toward a limit that was never measured.
            # Keep its unknown outline, and test the balanced interior itself.
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
                # Apply the same two plot-coordinate error budgets as SEP.
                # A vertical boundary should not need sub-ulp load precision
                # just because its speed position differs within one pixel.
                # At a local extremum the PCHIP slope is clamped to zero;
                # retain the stricter vertical check there. A nearby speed
                # match can otherwise conceal the misplaced turning point
                # and distort SEP just inside an apparently accurate edge.
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
            inaccurate=(np.any(failed) or boundary_inaccurate or feasibility_change or
                        column.get('speed_point_probe') and (not column['points'][0]['valid'] or not finite.all()) or
                        overlap and np.isfinite(actual).any()!=np.isfinite(pred).any() or
                        column.get('speed_probe') and (column.get('boundary_status') not in ('verified limit','plot ceiling') or
                        any(not p['valid'] for p in column['points'] if column.get('lower_boundary') and column.get('boundary') and
                            column['lower_boundary']['load_g']<=p['load_g']<=column['boundary']['load_g'])))
            if column.get('speed_probe') or column.get('speed_point_probe'):
                if inaccurate:
                    # Reuse all actual same-speed evidence when upgrading a
                    # failed check. No interpolated value becomes a trim knot.
                    seed=[p for p in column['points'] if p['valid']]
                    local=(name,json.dumps(conditions[name],sort_keys=True),mid,seed)
                    future=pool.submit(sample_with_history,sample_column,local,dict(entry_histories[name]))
                    pending_speed[future]=(context,None)
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
                    # A single central check can miss a curved fixed-load
                    # strip close to the lower-speed column's lift limit.
                    # Across a broad interval, check that strip at a second
                    # independent speed. It costs one trim, not another full
                    # column; only a failed check requests surface refinement.
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
                        # One coupled n=1 speed root resolves this physical
                        # edge. Recursive full load surfaces inside the same
                        # bracket add no independent envelope information.
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
                    while pending_speed or ready_speed:
                        check_cancel()
                        if not ready_speed:
                            completed,_=wait(set(pending_speed),timeout=.25,return_when=FIRST_COMPLETED)
                            for future in completed:
                                context,key=pending_speed.pop(future)
                                column,history=future.result()
                                ready_speed.append((context,column,history,key))
                        while ready_speed:
                            accept_speed(*ready_speed.popleft())
                    for name,probes in checked_probes.items():
                        for mid,(context,column) in list(probes.items()):
                            prior=context[-1];current=tuple(sorted(results[name]))
                            if prior==current:continue
                            # No new solves for a passing recheck: use the exact
                            # held-out states against the final neighboring slopes.
                            del probes[mid]
                            context=(*context[:-1],current)
                            scheduled_speed+=1
                            accept_speed(context,column,{},None)
                    if not pending_speed and not ready_speed:break
            except BaseException:
                worker_cancel.set()
                for future in pending_speed:future.cancel()
                raise
        drain_speed_checks()
        # Boundary accuracy has its own budget: interior Ps convergence does
        # not establish a corner or a discontinuous controller transition.
        # Continue troublesome upper-edge intervals down to display scale, checking
        # actual equilibria at every new speed. Nothing is smoothed or averaged.
        spans={name:list(intervals[name]) for name in results}
        for depth in range(9):
            tasks=[];pending_edges={}
            for name,pairs in spans.items():
                edge_columns={**boundary_probes[name],**results[name]}
                for lo,hi in pairs:
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
                # A change of active constraint on a sloping but continuous
                # curve is not a jump. Test midpoint departure, not total rise.
                jump=(all(verified) and len(set(kinds))>1 and
                      abs(rates[1]-(rates[0]+rates[2])*.5)>.075)
                boundary_checks[name].append(dict(speed_kmh=mid,bracket_kmh=[lo,hi],
                    error_dps=error,depth=depth,verified=verified))
                if (error is not None and error>.025) or jump or len(set(verified))>1:
                    spans[name].extend([(lo,mid),(mid,hi)])
        # Refined columns can expose a failed controller start between two
        # fully verified neighbors. Revisit those columns with the actual
        # neighboring limit states rather than declaring a speed interval lost.
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
            # A subpixel interval may happen to contain an exact display-raster
            # speed. Masking the open interval would then leave a visible blank
            # slice even though both sides have verified equilibria. Solve those
            # few raster speeds directly, and retain the unverified open pieces
            # on either side. No interpolated point is promoted to evidence.
            raster=np.linspace(config['speed_min_kmh'],config['speed_max_kmh'],
                2*config['surface_resolution']-1)
            raster_step=(config['speed_max_kmh']-config['speed_min_kmh'])/(len(raster)-1)
            tasks=[];scheduled_raster=set()
            for name,columns in results.items():
                for lo,hi in intervals[name]+terminal_speed_intervals[name]:
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
        # Once adaptive checks expose a no-level-flight/stall bracket, solve
        # the coupled n=1 stall-speed endpoint directly. The earlier pass can
        # only see coarse initial columns; without this late pass the generic
        # edge loop computes several complete load surfaces to locate one
        # physical endpoint.
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

        # The first drawable speed is a separate edge, not a coarse-grid
        # accident. Refine the empty/feasible bracket to 0.05 km/h. Retain every
        # rejected column as evidence; only solved columns enter the surface.
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
                if any(lo<=left['speed_kmh'] and right['speed_kmh']<=hi for lo,hi in existing):continue
                levels=[next((p for p in c['points'] if p['valid'] and p['load_g']==1.),None)
                        for c in (left,right)]
                if all(levels) and levels[0]['ps_mps']*levels[1]['ps_mps']<0:
                    endpoint_tasks.append((name,cfg_json,left,right))
        run(endpoint_tasks,'Solving level-flight Ps=0 endpoints',sample_level_endpoint)
        # A recovered edge can split an older, broad unresolved interval.
        # Validate its newly feasible pieces before retiring that old mask;
        # keeping the original interval hides columns whose physical edge is
        # now known. Unknown pieces retain their original diagnostic masks.
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
        # Late physical edges and directly solved display speeds are new
        # knots too. Reuse the independent holdouts until every changed
        # neighboring stencil has been checked against the final surface.
        drain_speed_checks()
        while resolve_visible_seams():drain_speed_checks()
        # A difficult upper edge must not erase the independently balanced
        # interior of an entire speed column. Check the common load range in
        # fixed physical coordinates, without the uncertain cap's slopes.
        from em_speed_seam import check_interior
        seam_interiors={name:[] for name in results};seam_futures={}
        for name,columns in results.items():
            for lo,hi in sorted(set(intervals[name]+terminal_speed_intervals[name])):
                if hi-lo>2.*speed_tolerance(config):continue
                pair=[columns[v] for v in (lo,hi)]
                if not all(c.get('lower_boundary') and c.get('boundary') and
                    c['boundary_status'] in ('verified limit','plot ceiling') for c in pair):continue
                support=[{k:c[k] for k in ('speed_kmh','points','boundary','lower_boundary','boundary_status','boundary_reason')} for c in pair]
                task=(name,json.dumps(conditions[name],sort_keys=True),(lo+hi)*.5,support)
                future=pool.submit(sample_with_history,check_interior,task,dict(entry_histories[name]))
                seam_futures[future]=name
        if seam_futures and progress:progress(dict(phase='Checking interiors below uncertain boundary transitions',
            done=0,total=len(seam_futures),elapsed_s=time.monotonic()-start))
        while seam_futures:
            check_cancel()
            done,_=wait(set(seam_futures),timeout=.25,return_when=FIRST_COMPLETED)
            for future in done:
                name=seam_futures.pop(future);certificate,history=future.result()
                if certificate:seam_interiors[name].append(certificate)
                for speed,entry in history.items():entry_histories[name].setdefault(speed,entry)
        # End every speculative task inside this request's clock. Usually
        # all were adopted above; unused queued jobs can be cancelled safely.
        try:
            for future in prefetched.values():
                if future.cancel():continue
                while not future.done():
                    check_cancel();wait([future],timeout=.25)
                future.result()
        except BaseException:
            worker_cancel.set()
            for future in prefetched.values():future.cancel()
            raise
        output['prefetched_speed_checks']=dict(submitted=len(prefetch_seen),used=prefetch_used)
        for index,(name,columns) in enumerate(results.items()):
            ordered=[columns[v] for v in sorted(columns)]
            for column in ordered:
                column.pop('_curve',None);column.pop('_alpha_curve',None);column.pop('_angle_map',None)
            points=[p for col in ordered for p in col['points']]
            probes=list(boundary_probes[name].values())
            points.extend(p for col in probes for p in col['points'])
            points.extend(p for _,col in checked_probes[name].values() for p in col['points'])
            for certificate in seam_interiors[name]:points.extend(certificate.pop('points'))
            checks=speed_checks[name]
            metadata=dict(AIRCRAFT[name],color=['#38c9d7','#ffa66b'][index])
            output['aircraft'].append(dict(id=name,**metadata,settings=conditions[name],points=points,columns=ordered,
                boundary_columns=[outline_column(c) for c in sorted(
                    {**boundary_probes[name],**columns}.values(),key=lambda c:c['speed_kmh'])],
                instructor_approximation=profile(load(name)) if conditions[name]['instructor'] else None,
                instructor_unresolved_speeds_kmh=[c['speed_kmh'] for c in ordered if c['boundary_status']=='Instructor boundary unresolved'],
                sweep_excluded_speeds_kmh=exclusions[name],
                low_speed_edge=low_speed_edges.get(name),
                speed_limit=redlines[name],
                sustained=[p for col in ordered for p in col['sustained']],mass=ordered[0]['mass'],engine=ordered[0]['engine'],
                valid_points=sum(p['valid'] for p in points),converged_points=sum(p['converged'] for p in points),
                interpolation=dict(target_mps=config['sep_tolerance_mps'],target_contour_dps=turn_tolerance(config['sep_tolerance_mps']),target_speed_kmh=speed_tolerance(config),checked_sep_range_mps=list(visible_range(config)),speed_checks=checks,
                                   unresolved_speed_intervals=intervals[name]+terminal_speed_intervals[name],boundary_checks=boundary_checks[name],
                                   certified_speed_interiors=seam_interiors[name],
                                   boundary_refinement_intervals=spans[name],load_checks=sum(len(c['load_checks']) for c in ordered))))
    output['speeds_kmh']=sorted(set(v for cols in results.values() for v in cols))
    output['elapsed_s']=time.monotonic()-start
    output['cached_columns']=cache_hits
    return output
