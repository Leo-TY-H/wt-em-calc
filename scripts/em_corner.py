import time
import math
import numpy as np


def sample_corner(task):
    from em_sampling import worker_solver, sample_column
    name, config_json, left, right = task
    started = time.monotonic()
    solver = worker_solver(name, config_json)
    if solver.config.get('trim_solver_mode') == 'stationary' and hasattr(solver,'core'):
        point=stationary_intersection(solver,left,right)
        if point is None:return None
        column=sample_column((name,config_json,point['speed_kmh'],[point]))
        column['constraint_corner']=dict(point['envelope_limit'],
            structural_side=('right' if right['boundary_reason']=='wing force' else 'left')
                if 'Instructor pitch' not in point['envelope_limit']['competing_constraints'] else None)
        column['elapsed_s']=time.monotonic()-started
        return column
    if solver.is_prop and solver.engine.automatic:


        solver = solver.with_prop_controls(solver.engine.fixed_controls or solver.engine.automatic_controls)
        solver.engine.force_canonical = True
    points = [c['boundary'] for c in (left, right)]
    kinds={p['envelope_limit']['kind'] for p in points}
    if 'Instructor pitch' in kinds and kinds & {'wing force','control'}:
        return instructor_corner(task)
    if {p['envelope_limit']['kind'] for p in points} != {'stall', 'wing force'}:
        return None
    if any(p.get('sideslip_attitude_deg', 0.) for p in points):
        return None
    memo = {}

    def evaluate(z, frozen=None):
        key = tuple(z)
        if frozen is not None or key not in memo:
            value = solver.operating_point(z[6] / 3.6, z[5], z[:5],
                                           propulsion_override=frozen, cycle_seconds=20.)
            residual = np.r_[value['residual'], (value['stall_margin'] - .002) / 10.,
                             max(value['maximum_wing_load_ratios']) - .99995]
            pair = value, residual
            if frozen is not None:
                return pair
            memo[key] = pair
        return memo[key]

    lower = np.r_[solver.trim_bounds[0], 1., left['speed_kmh']]
    from em_load_limits import ceiling
    upper = np.r_[solver.trim_bounds[1], ceiling(solver,right['speed_kmh']), right['speed_kmh']]
    stall = next(p for p in points if p['envelope_limit']['kind']=='stall')
    if solver.is_prop and solver.engine.automatic and stall.get('_propulsion_seed') is not None:
        from em_data import clone
        solver.engine._reference=clone(stall['_propulsion_seed'])
    speed = float(np.clip(stall['speed_kmh']/math.sqrt(max(stall['wing_load_ratios'])), lower[6], upper[6]))
    n = stall['load_g']*(speed/stall['speed_kmh'])**2
    z = np.r_[stall['solution'], n, speed]
    z[1] += math.degrees(math.acos(1./n)-math.acos(1./stall['load_g']))
    z = np.clip(z, lower, upper)
    value, residual = evaluate(z)
    accepted = None
    for iteration in range(12):
        if ((not solver.is_prop or solver.engine.automatic) and
                value['force_error_g'] <= 2e-4 and max(abs(value['rate_residual'])) <= 5e-5
                and abs(residual[-2]) <= 2e-5 and abs(residual[-1]) <= 5e-6):
            candidate = solver.certify(float(z[6]),float(z[5]),z[:5],value,evaluations=len(memo))
            if candidate['valid']:
                accepted = candidate
                break
        if (value['force_error_g'] < 1e-5 and max(abs(value['rate_residual'])) < 5e-6
                and abs(residual[-2]) < 1e-5 and abs(residual[-1]) < 5e-6):
            break
        frozen = value['propulsion'] if solver.is_prop and value['propulsion']['converged'] else None
        base, base_residual = evaluate(z, frozen)
        columns = []
        for axis, step in enumerate([.002, .002, .0002, .0002, .0002, .001, .02]):
            for factor in (1., .5, -.5, 2., -1., .1, -.1):
                trial = z.copy()
                trial[axis] += step * factor
                v, r = evaluate(trial, frozen)
                if solver.derivative_branch(v) == solver.derivative_branch(base):
                    break
            columns.append((r - base_residual) / (step * factor))
        matrix = np.column_stack(columns)
        if not np.isfinite(matrix).all() or not np.isfinite(residual).all():return None
        try:delta = np.linalg.lstsq(matrix, -residual, rcond=None)[0]
        except np.linalg.LinAlgError:return None
        delta /= max(1., float(np.max(abs(delta) / [3., 10., .3, .3, .3, 3., 30.])))
        for factor in (1., .5, .25, .1):
            trial = np.clip(z + delta * factor, lower, upper)
            v, r = evaluate(trial)
            if np.linalg.norm(r) < np.linalg.norm(residual):
                z, value, residual = trial, v, r
                break
        else:
            return None
    if (value['force_error_g'] > 2e-4 or max(abs(value['rate_residual'])) > 5e-5
            or abs(residual[-2]) > 2e-5 or abs(residual[-1]) > 5e-6):
        return None


    point = accepted or ((solver.certify(float(z[6]),float(z[5]),z[:5],value,evaluations=len(memo))))
    if (not point['valid'] or abs(point['stall_margin_deg'] - .002) > .0003
            or abs(max(point['wing_load_ratios']) - .99995) > 1e-5):
        return None
    point['envelope_limit'] = dict(kind='stall', axis=None, limiting_load_g=point['load_g'],
        method='simultaneous stall and wing-force intersection',
        competing_constraints=['stall', 'wing force'],
        constraint_residual=point['stall_margin_deg'] - .002,
        wing_constraint_residual=max(point['wing_load_ratios']) - .99995,
        speed_bracket_kmh=[left['speed_kmh'], right['speed_kmh']])
    point['_checked_probe_boundary'] = True
    column = sample_column((name, config_json, point['speed_kmh'], [point]))
    column['constraint_corner'] = dict(point['envelope_limit'], structural_side=('right' if right['boundary_reason']=='wing force' else 'left'))
    column['elapsed_s'] = time.monotonic() - started
    return column


def stationary_intersection(solver,left,right):
    """Two physical faces, with speed/load and all joint states corrected together."""
    from em_stationary_boundary import Curve, mark
    points=[column['boundary'] for column in (left,right)]
    if any(not p['valid'] or p.get('sideslip_attitude_deg',0.) for p in points):return None
    kinds={p['envelope_limit']['kind'] for p in points}
    if kinds!={'stall','wing force'} and not (
            'Instructor pitch' in kinds and len(kinds)==2 and kinds & {'wing force','control'}):return None
    speed_interval=(left['speed_kmh'],right['speed_kmh'])
    if speed_interval[0]>=speed_interval[1]:return None
    seed=dict(points[0],solution=np.mean([p['solution'] for p in points],axis=0).tolist(),
        speed_kmh=sum(speed_interval)/2.,load_g=sum(p['load_g'] for p in points)/2.)
    curve=Curve(solver,seed['speed_kmh'],speed_interval=speed_interval,free_load=True)
    events=[]
    ordered=['Instructor pitch' if 'Instructor pitch' in kinds else 'stall',
             'wing force' if 'wing force' in kinds else 'control']
    for kind in ordered:
        if kind=='stall':
            events.append(lambda v:(v['stall_margin']-.002)/10.)
        elif kind=='wing force':
            events.append(lambda v:max(v['maximum_wing_load_ratios'])-.99995)
        elif kind=='Instructor pitch':
            def instructor(v):
                control=solver.instructor_at(v,v['speed'],v['equilibrium_load_g'])
                return control['envelope_margin']-2e-6 if control['converged'] else float('nan')
            events.append(instructor)
        else:
            edge=next(p for p in points if p['envelope_limit']['kind']=='control')
            _,axis,side=min((abs(command-bound),axis,side)
                for axis,(command,bounds) in enumerate(zip(edge['commands'],edge['control_bounds']))
                for side,bound in enumerate(bounds))
            def mechanical(v,axis=axis,side=side):
                control=solver.instructor_at(v,v['speed'],v['equilibrium_load_g'])
                if not control['converged']:return float('nan')
                return (1. if side==0 else -1.)*(v['allocation']['commands'][axis]-
                    control['control_bounds'][axis][side])-3e-5
            events.append(mechanical)
    point=curve.correct(seed,events=events,max_nfev=48,event_tolerance=5e-6)
    if curve.unverified_candidates:
        solver.__dict__.setdefault('stationary_boundary_evidence',[]).extend(curve.unverified_candidates)
    if point is None or not point['valid']:return None
    point=mark(point,ordered[0],method='shared stationary balance at intersecting physical constraints',
        competing_constraints=ordered,speed_bracket_kmh=list(speed_interval))
    point['_checked_probe_boundary']=True
    return point


def instructor_corner(task):
    from scipy.optimize import brentq
    from em_sampling import sample_boundary_column,sample_column,worker_solver
    name,config_json,left,right=task
    physical=next(c['boundary_reason'] for c in (left,right) if c['boundary_reason']!='Instructor pitch')
    columns={c['speed_kmh']:c for c in (left,right)}
    def margins(point):
        mechanical=(1.-max(point['wing_load_ratios']) if physical=='wing force' else point['authority_margin'])
        return point['instructor']['envelope_margin'],mechanical


    from em_instructor_corner import solve_corner
    point=solve_corner(worker_solver(name,config_json),left,right)
    if point is not None:
        column=sample_column((name,config_json,point['speed_kmh'],[point]))
        if (column['boundary_status']=='verified limit'
                and max(abs(x) for x in margins(column['boundary']))<=2e-4):
            column['constraint_corner']=dict(method=point['envelope_limit']['method'],
                competing_constraints=['Instructor pitch',physical],structural_side=None,
                speed_bracket_kmh=[left['speed_kmh'],right['speed_kmh']],residuals=list(margins(column['boundary'])))
            return column
    def residual(speed):
        if speed not in columns:
            near=min(columns.values(),key=lambda c:abs(c['speed_kmh']-speed))
            columns[speed]=sample_boundary_column((name,config_json,speed,[near['boundary']]))
        column=columns[speed]
        if column['boundary_status']!='verified limit':raise ValueError('Unresolved corner trim')
        aoa,mechanical=margins(column['boundary'])
        return aoa-mechanical
    try:
        speed=brentq(residual,left['speed_kmh'],right['speed_kmh'],xtol=.001,maxiter=24)
    except (ValueError,RuntimeError):return None
    point=columns[speed]['boundary']
    if max(abs(x) for x in margins(point))>2e-4:return None
    column=sample_column((name,config_json,speed,[point]))
    if column['boundary_status']!='verified limit':return None
    column['constraint_corner']=dict(method='balanced AoA and physical-limit intersection',
        competing_constraints=['Instructor pitch',physical],structural_side=None,
        speed_bracket_kmh=[left['speed_kmh'],right['speed_kmh']],residuals=list(margins(column['boundary'])))
    return column
