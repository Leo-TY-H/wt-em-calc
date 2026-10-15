import numpy as np


def solve_corner(solver, left, right):
    points = [column['boundary'] for column in (left, right)]
    if any(not point['valid'] or point.get('sideslip_attitude_deg', 0.) for point in points):
        return None
    kinds = {point['envelope_limit']['kind'] for point in points}
    if 'Instructor pitch' not in kinds or not kinds & {'wing force', 'control'}:
        return None
    if solver.config.get('trim_solver_mode') == 'stationary' and hasattr(solver,'core'):
        from em_corner import stationary_intersection
        return stationary_intersection(solver,left,right)
    physical = next(kind for kind in kinds if kind != 'Instructor pitch')
    if physical == 'control':
        edge = next(point for point in points if point['envelope_limit']['kind'] == physical)


        _, axis, side = min((abs(command - bound), axis, side)
            for axis, (command, bounds) in enumerate(zip(edge['commands'], edge['control_bounds']))
            for side, bound in enumerate(bounds))
        direction = 1. if side else -1.

    if solver.is_prop and solver.engine.automatic:
        solver = solver.with_prop_controls(solver.engine.fixed_controls or solver.engine.automatic_controls)
        from em_data import clone
        seed = next((point['_propulsion_seed'] for point in points if point.get('_propulsion_seed')), None)
        if seed is not None:
            solver.engine._reference = clone(seed)
        solver.engine.force_canonical = True

    memo = {}
    def evaluate(z, frozen=None):
        key = tuple(z)
        if frozen is None and key in memo:
            return memo[key]
        value = solver.operating_point(z[6] / 3.6, z[5], z[:5],
            propulsion_override=frozen, cycle_seconds=20.)
        instructor = solver.instructor_at(value, z[6] / 3.6, z[5])
        if physical == 'wing force':
            mechanical = max(value['maximum_wing_load_ratios']) - .99995
        else:
            bound = instructor['control_bounds'][axis][side]
            mechanical = direction * (value['allocation']['commands'][axis] - bound) + 3e-5
        residual = np.r_[value['residual'], instructor['envelope_margin'] - 2e-6, mechanical]
        pair = value, residual
        if frozen is None:
            memo[key] = pair
        return pair

    lower = np.r_[solver.trim_bounds[0], 1., left['speed_kmh']]
    from em_load_limits import ceiling
    upper = np.r_[solver.trim_bounds[1], ceiling(solver,right['speed_kmh']), right['speed_kmh']]
    z = np.mean([point['solution'] + [point['load_g'], point['speed_kmh']] for point in points], axis=0)
    z = np.clip(z, lower, upper)
    value, residual = evaluate(z)
    accepted = None
    for _ in range(12):


        if ((not solver.is_prop or solver.engine.automatic) and
                value['force_error_g'] <= 2e-4 and max(abs(value['rate_residual'])) <= 5e-5
                and max(abs(residual[-2:])) <= 5e-6):
            candidate = solver.certify(float(z[6]),float(z[5]),z[:5],value,evaluations=len(memo))
            if candidate['valid']:
                accepted = candidate
                break
        if (value['force_error_g'] < 1e-5 and max(abs(value['rate_residual'])) < 5e-6
                and max(abs(residual[-2:])) < 5e-7):
            break
        frozen = value['propulsion'] if solver.is_prop and value['propulsion']['converged'] else None
        base, base_residual = evaluate(z, frozen)
        columns = []
        for axis_index, step in enumerate([.002, .002, .0002, .0002, .0002, .001, .02]):
            for factor in (1., .5, -.5, 2., -1., .1, -.1):
                trial = z.copy()
                trial[axis_index] += step * factor
                v, r = evaluate(trial, frozen)
                if solver.derivative_branch(v) == solver.derivative_branch(base):
                    break
            columns.append((r - base_residual) / (step * factor))
        matrix = np.column_stack(columns)
        if not np.isfinite(matrix).all() or not np.isfinite(residual).all():
            return None
        try:
            delta = np.linalg.lstsq(matrix, -residual, rcond=None)[0]
        except np.linalg.LinAlgError:
            return None
        delta /= max(1., float(np.max(abs(delta) / [3., 10., .3, .3, .3, 3., 30.])))
        for factor in (1., .5, .25, .1):
            trial = np.clip(z + factor * delta, lower, upper)
            v, r = evaluate(trial)
            if np.linalg.norm(r) < np.linalg.norm(residual):
                z, value, residual = trial, v, r
                break
        else:
            return None
    if (value['force_error_g'] > 2e-4 or max(abs(value['rate_residual'])) > 5e-5
            or max(abs(residual[-2:])) > 5e-6):
        return None


    point = accepted or ((solver.certify(float(z[6]),float(z[5]),z[:5],value,evaluations=len(memo))))
    if not point['valid']:
        return None
    margins = [point['instructor']['envelope_margin'],
        1. - max(point['wing_load_ratios']) if physical == 'wing force' else point['authority_margin']]
    if max(abs(margin) for margin in margins) > 2e-4:
        return None
    point['envelope_limit'] = dict(kind='Instructor pitch', axis=None, limiting_load_g=point['load_g'],
        method='simultaneous Instructor and physical-limit intersection',
        competing_constraints=['Instructor pitch', physical],
        constraint_residual=point['instructor']['envelope_margin'] - 2e-6,
        speed_bracket_kmh=[left['speed_kmh'], right['speed_kmh']])
    return point
