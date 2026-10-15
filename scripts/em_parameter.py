import math
import numpy as np


def _curve(solver, speed, seed):
    from em_continuation import TrimCurve
    cached = getattr(solver, '_interior_curve', None)
    if cached is None or cached[0] != id(solver) or cached[1] != speed:
        cached = (id(solver), speed, TrimCurve(solver, speed, seed))
        solver._interior_curve = cached
    return cached[2]


def recover_fixed_load_point(solver, speed, load, neighbors):
    if (not solver.is_prop or not solver.engine.automatic or
            abs(solver.sideslip_attitude_deg) > 1e-8):
        return None
    candidates = [p for p in neighbors if p.get('valid') and p['speed_kmh'] == speed and
        abs(p.get('sideslip_attitude_deg', 0.)) < 1e-8]
    below = [p for p in candidates if p['load_g'] < load]
    above = [p for p in candidates if p['load_g'] > load]
    if not below or not above:
        return None
    left = max(below, key=lambda p: p['load_g'])
    right = min(above, key=lambda p: p['load_g'])
    fraction = (load - left['load_g']) / (right['load_g'] - left['load_g'])
    guess = np.asarray(left['solution']) * (1. - fraction) + np.asarray(right['solution']) * fraction
    nominal = lambda n: math.degrees(math.acos(1. / n))
    guess[1] = (nominal(load) + (left['bank_deg'] - nominal(left['load_g'])) * (1. - fraction)
        + (right['bank_deg'] - nominal(right['load_g'])) * fraction)
    from em_continuation import TrimCurve
    for seed in sorted((left, right), key=lambda p: abs(p['load_g'] - load)):
        curve = TrimCurve(solver, speed, seed)
        point = curve.at_load(load, guess, max_iterations=8, max_refreshes=2)
        if point is not None and point['valid']:
            point['recovery_method'] = 'bounded fixed-load correction from certified neighbors'
            return point
    return None


def _alpha_point(solver,speed,left,right,fraction):
    if not (left['valid'] and right['valid'] and left['load_g']<right['load_g']
            and left['alpha_deg']+1e-5<right['alpha_deg']):return None
    if any(abs(p.get('sideslip_attitude_deg',0.))>1e-8 for p in (left,right)):return None
    curve=_curve(solver,speed,left)
    a,b=curve.state(left),curve.state(right)
    point=curve.correct(a+fraction*(b-a),np.eye(6)[0])
    if (point is None or not point['valid'] or not left['load_g']<point['load_g']<right['load_g']
            or not left['alpha_deg']<point['alpha_deg']<right['alpha_deg']):return None
    point['sampling_coordinate']='angle continuation; independently certified full aircraft'
    return point


def alpha_point(solver,speed,left,right):


    for fraction in (.5,.4375,.5625):
        point=_alpha_point(solver,speed,left,right,fraction)
        if point is not None:return point
    return None


def interior_point(solver,speed,left,right):
    if not (left['valid'] and right['valid'] and left['load_g']<right['load_g']):return None
    if (solver.is_prop and not solver.engine.automatic or
            any(abs(p.get('sideslip_attitude_deg',0.))>1e-8 for p in (left,right))):
        return alpha_point(solver,speed,left,right)
    curve=_curve(solver,speed,left)
    for fraction in (.5,.4375,.5625):
        point=curve.between(left,right,fraction)
        if point is not None and point['valid']:
            point['sampling_coordinate']='transverse plane across the full trim curve'
            return point
    return alpha_point(solver,speed,left,right)
