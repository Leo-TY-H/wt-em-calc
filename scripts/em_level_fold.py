"""Locate the level-flight endpoint when the trim branch folds in speed."""
import numpy as np
import copy
from scipy.optimize import least_squares, minimize_scalar


def level_fold(solver, left, right):
    """Certify a local speed minimum using balanced states on both sides.

    Fixed-speed trim is singular here. Keep pitch command fixed instead and
    solve the same five aircraft balances with speed free. A failed trim is
    never a certificate: the minimum and both neighboring command states
    must independently pass the ordinary aircraft and Instructor checks.
    """
    point = right.get('boundary')
    if not point or not point['valid'] or point['load_g'] != 1.:
        return None
    from em_level_bracket import level_response_edge
    response_edge=level_response_edge(solver,left,right)
    if response_edge:return response_edge
    if point['stall_margin_deg'] < .1 or point['authority_margin'] < .05:
        return None
    if solver.config['instructor'] and point['instructor']['envelope_margin'] < .04:
        return None
    # Automatic propeller phase continuation needs its separate certificate;
    # this coordinate solver currently covers the deterministic jet equations.
    if solver.is_prop:
        return None
    speed = right['speed_kmh']
    center = point['solution'][3]
    memo = {}
    # Fixed-command probes must retain their commanded branch even when it is
    # physically rejected; recovering another root would destroy this regular
    # coordinate and conceal a zero-response endpoint.
    solver=copy.copy(solver);solver._pitch_response_recovery=True

    def at(command):
        if command in memo:
            return memo[command]
        near = min(memo.values(), key=lambda p: abs(p['solution'][3]-command)) if memo else point
        x = near['solution']
        initial = np.array([x[0], x[1], x[2], x[4], near['speed_kmh']])
        cache = {}

        def value(z):
            key = tuple(z)
            if key not in cache:
                cache[key] = solver.operating_point(z[4]/3.6, 1., [z[0], z[1], z[2], command, z[3]])
            return cache[key]

        def jac(z):
            base = value(z)
            columns = []
            for i, h in enumerate([.002, .002, .0002, .0002, .002]):
                for factor in (1., .5, -.5, 2., -1., .1, -.1):
                    trial = z.copy()
                    trial[i] += h*factor
                    v = value(trial)
                    step = h*factor
                    if solver.derivative_branch(v) == solver.derivative_branch(base):
                        break
                columns.append((v['residual']-base['residual'])/step)
            return np.column_stack(columns)

        bounds = tuple([b[0], b[1], b[2], b[4], v] for b, v in
                       zip(solver.trim_bounds, (max(1., left['speed_kmh']-1.), speed+3.)))
        fit = least_squares(lambda z: value(z)['residual'], initial, jac=jac,
                            bounds=bounds, x_scale=[10., 30., .2, .2, 10.],
                            max_nfev=25, gtol=1e-10, xtol=1e-9, ftol=1e-10)
        z = fit.x
        v = value(z)
        if v['force_error_g'] > 1e-6 or max(abs(v['rate_residual'])) > 2e-6:
            raise ValueError('Unbalanced level-fold probe')
        p = solver.solve(z[4], 1., [z[0], z[1], z[2], command, z[3]], exhaustive=False, quick=True)
        if (not p['converged'] or set(p['reasons'])-{'reversed pitch response','pitch response unresolved'}
                or abs(p['solution'][3]-command) > 1e-5
                or abs(p['alpha_deg']-point['alpha_deg']) > 2.):
            raise ValueError('Disconnected level-fold probe')
        memo[command] = p
        return p

    for radius in (.05, .015, .005):
        bounds = (max(-.999, center-radius), min(.999, center+radius))
        separation = radius*.2
        try:
            fit = minimize_scalar(lambda e: at(e)['speed_kmh'], bounds=bounds, method='bounded',
                                  options=dict(xatol=1e-5, maxiter=25))
            p = at(fit.x)
            sides = [at(fit.x-separation), at(fit.x+separation)]
        except (ValueError, RuntimeError):
            continue
        if not bounds[0]+separation < fit.x < bounds[1]-separation:
            continue
        # The normal-control domain can end just before the algebraic speed
        # fold. Close its physical response sign in the regular command
        # coordinate. Both endpoints must balance; no failed trim is a sign.
        try:anchor=at(center)
        except (ValueError,RuntimeError):continue
        normal=[q for q in [p,*sides,anchor] if q['valid']]
        reversed_points=[q for q in [p,*sides,anchor] if q['reasons']==['reversed pitch response']]
        if normal and reversed_points:
            good=min(normal,key=lambda q:abs(q['solution'][3]-fit.x))
            bad=min(reversed_points,key=lambda q:abs(q['solution'][3]-good['solution'][3]))
            def closed_response():
                return (abs(good['speed_kmh']-bad['speed_kmh'])<.0005 and
                        abs(good['ps_mps']-bad['ps_mps'])<.002)
            try:
                for _ in range(24):
                    if closed_response():break
                    trial=at((good['solution'][3]+bad['solution'][3])*.5)
                    if trial['valid']:good=trial
                    elif trial['reasons']==['reversed pitch response']:bad=trial
                    else:break
            except (ValueError,RuntimeError):continue
            from em_accuracy import speed_tolerance
            # A near-singular coarse trim can pass the ordinary balance
            # tolerance just below the tightly balanced physical endpoint.
            # The coarse speed bracket is an initializer, not a game limit;
            # allow that correction within the chart's requested accuracy.
            if (closed_response() and left['speed_kmh']<=good['speed_kmh'] and
                    good['speed_kmh']<=right['speed_kmh']+.25*speed_tolerance(solver.config)):
                good['envelope_limit']=dict(kind='pitch response',axis=None,limiting_load_g=1.,
                    method='Balanced normal/reversed pitch-response bracket in level-flight command coordinates',
                    command_bracket=[good['solution'][3],bad['solution'][3]],
                    limiting_speed_interval_kmh=sorted([good['speed_kmh'],bad['speed_kmh']]),
                    constraint_residual=good['pitch_response']['margin'],
                    rejected_constraint_residual=bad['pitch_response']['margin'],evaluations=len(memo),
                    rejected_endpoint={k:bad[k] for k in ('speed_kmh','load_g','solution','reasons','force_error_g','angular_error_rad_s2')})
                return good
        if not all(q['valid'] for q in [p,*sides]):continue
        if any(q['speed_kmh']-p['speed_kmh'] < .002 for q in sides):
            continue
        if not left['speed_kmh'] <= p['speed_kmh'] <= right['speed_kmh']:
            continue
        if any(q['force_error_g'] > 1e-6 or q['angular_error_rad_s2'] > 2e-6 for q in [p, *sides]):
            continue
        p['envelope_limit'] = dict(kind='trim fold', axis=3, limiting_load_g=1.,
            method='balanced level-speed minimum in pitch-command coordinates',
            command_bracket=[q['solution'][3] for q in sides],
            bracket_speeds_kmh=[q['speed_kmh'] for q in sides], evaluations=len(memo),
            probes=[{k: q[k] for k in ('speed_kmh', 'load_g', 'solution', 'force_error_g',
                                      'angular_error_rad_s2', 'valid')} for q in sides])
        return p
    return None
