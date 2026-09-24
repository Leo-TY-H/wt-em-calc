"""Solve intersections of physical envelope constraints before mesh refinement."""
import time
import math
import numpy as np


def sample_corner(task):
    from em_sampling import worker_solver, sample_column
    name, config_json, left, right = task
    started = time.monotonic()
    solver = worker_solver(name, config_json)
    if solver.is_prop and solver.engine.automatic:
        # This coupled root fixes both active constraints simultaneously.
        # A mean-wash search followed by complete-phase re-trimming can move
        # the aircraft off their intersection and discard a valid corner.
        # Use the same complete phase residual as final validation, retaining
        # frozen-propulsion Jacobians only for the Newton search direction.
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
    upper = np.r_[solver.trim_bounds[1], solver.config['max_load_g'] or 64., right['speed_kmh']]
    stall = next(p for p in points if p['envelope_limit']['kind']=='stall')
    speed = float(np.clip(stall['speed_kmh']/math.sqrt(max(stall['wing_load_ratios'])), lower[6], upper[6]))
    n = stall['load_g']*(speed/stall['speed_kmh'])**2
    z = np.r_[stall['solution'], n, speed]
    z[1] += math.degrees(math.acos(1./n)-math.acos(1./stall['load_g']))
    z = np.clip(z, lower, upper)
    value, residual = evaluate(z)
    for iteration in range(12):
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
        delta = np.linalg.lstsq(np.column_stack(columns), -residual, rcond=None)[0]
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
    point = solver.solve(float(z[6]), float(z[5]), z[:5].tolist(), exhaustive=False)
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


def instructor_corner(task):
    """Locate a continuous AoA/physical-limit intersection in speed.

    Each root evaluation is an independently balanced boundary. Splitting the
    interpolation support at this measured corner avoids rounding the maximum
    with a cubic fitted across two different active constraints.
    """
    from scipy.optimize import brentq
    from em_sampling import sample_boundary_column,sample_column
    name,config_json,left,right=task
    physical=next(c['boundary_reason'] for c in (left,right) if c['boundary_reason']!='Instructor pitch')
    columns={c['speed_kmh']:c for c in (left,right)}
    def margins(point):
        mechanical=(1.-max(point['wing_load_ratios']) if physical=='wing force' else point['authority_margin'])
        return point['instructor']['envelope_margin'],mechanical
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
