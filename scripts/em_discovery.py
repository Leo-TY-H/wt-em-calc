import math

from em_continuation import TrimCurve


def interior_seeds(solver, speed, neighbors=None):
    from aircraft_model import condition_properties
    from air_state import cache
    from component_assembly import f32
    pass
    air = cache([f32(speed / 3.6), 0., 0.], solver.config['altitude_m'])
    polar = condition_properties(solver.model, air['mach'], solver.flaps)[1]
    estimate = (.5 * air['density'] * (speed / 3.6) ** 2 *
        solver.model['geometry']['area'] * polar['cyCritH'] / solver.weight)
    from em_load_limits import ceiling
    cap = ceiling(solver,speed)
    candidates = []
    nearby = sorted((p for p in neighbors or [] if p.get('valid')),
        key=lambda p: abs(p['speed_kmh'] - speed))
    if nearby:


        point = min(nearby[:9], key=lambda p: abs(p['load_g'] - max(1., .5 * estimate)))
        load = min(cap, max(1., point['load_g'] * (speed / point['speed_kmh']) ** 2))
        guess = list(point['solution'])
        guess[1] = math.degrees(math.acos(1. / load))
        candidates.append((load, guess, point))
    for fraction in (.5, .8, .2):
        load = min(cap, max(1. + .1 * fraction, estimate * fraction))
        candidates.append((load, None, None))
    points, attempts, searched = [], [], set()
    for load, guess, seed in candidates:
        key = (load, tuple(guess) if guess is not None else None)
        if key in searched:
            continue
        searched.add(key)
        curve = TrimCurve(solver, speed, seed)
        point = curve.at_load(load, guess, max_iterations=6, max_refreshes=1,
            max_evaluations=10, extend_settling=False)
        attempts.append(dict(load_g=load, converged=bool(point and point['converged']),
            valid=bool(point and point['valid']),
            reasons=point['reasons'] if point else ['bounded trim search unresolved']))
        if point is not None and point['valid']:
            point['branch_seed'] = 'independent interior equilibrium; no 1-g connectivity prerequisite'
            points.append(point)
    return dict(points=points, attempts=attempts, complete=False, estimated_aerodynamic_load_g=estimate,
        method='independent interior starts; original force, moment and feasibility checks')
