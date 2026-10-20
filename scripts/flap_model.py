"""Fixed flap positions, intact-flap speeds, and automatic IAS/Mach limits.

The chart holds an actual extension, rather than a lever request that retracts
with speed. Its domain ends when that extension can no longer be maintained.

S8 (aces.exe traced 2026-10-19; WT_FLAP_LOCK=0 restores the earlier model): the
flap actuator target is min(mechanism bounds, flapsLimByIas, flapsLimByMach).
The bounds collapse to zero above AvailableControls.flapsLimits.mechLockIas
(forward IAS, km/h) or mechLockMachNumber (total Mach), and to
secondaryMechDependentRange while the secondary mechanism (wing sweep) lies
outside secondaryMechRange (F-14A: no flaps above 60% sweep). Flap damage is
checked only above 5% extension.
"""
import os

from component_assembly import f32, add, sub, mul
from structural_limits import interval

LOCK = os.environ.get('WT_FLAP_LOCK', '1') != '0'
DAMAGE_FRACTION = f32(.05)   # the damage check runs only above this extension


def first(value):
    # Repeated BLK blocks/parameters use the first occurrence, as in fm_loader.
    return value[0] if isinstance(value, list) and value and isinstance(value[0], (list, dict)) else value


def profile(fm):
    controls = fm.get('AvailableControls', {})
    available = controls.get('hasFlapsControl', False)
    if isinstance(available, list):available = available[0]
    axis = first(fm.get('Aerodynamics', {}).get('FlapsAxis', {}))
    modes = {}
    for name, default in (('Combat', .2), ('Takeoff', .33), ('Landing', 1.)):
        mode = first(axis.get(name, {}))
        present = mode.get('Presents', True)
        if isinstance(present, list):present = present[0]
        value = mode.get('Flaps', default)
        if isinstance(value, list):value = value[0]
        modes[name.lower()] = float(value)*100. if available and present else None

    mass = fm.get('Mass', {})
    points = []
    # Numbered Point2 parameters take precedence over the legacy Point4/scalar.
    for i in range(10):
        row = first(mass.get('FlapsDestructionIndSpeedP'+str(i)))
        if isinstance(row, list) and len(row) == 2:
            x, y = map(f32, row)
            if not points or sub(x, points[-1][0]) > f32(4e-19):
                points.append((x, mul(y, f32(1/3.6))))
    if not points:
        legacy = first(mass.get('FlapsDestructionIndSpeedP'))
        if isinstance(legacy, list) and len(legacy) == 4:
            points = [(f32(legacy[i]), mul(legacy[i+1], f32(1/3.6))) for i in (0, 2)]
        elif isinstance(legacy, (int, float)):
            points = [(0., mul(legacy, f32(1/3.6)))]
        else:
            speed = mass.get('FlapsDestructionIndSpeed', 320.4)
            if isinstance(speed, list):speed = speed[0]
            speed = mul(speed, f32(1/3.6))
            points = [(f32(.1), add(speed, f32(200/3.6))), (1., speed)]

    lock = first(controls.get('flapsLimits', {})) if LOCK else {}
    lock = lock if isinstance(lock, dict) else {}

    def scalar(key, default):
        value = lock.get(key, default)
        return value[0] if isinstance(value, list) and value else value
    lock_ias = float(scalar('mechLockIas', 2147440000.))
    lock_mach = float(scalar('mechLockMachNumber', 100.))
    secondary = scalar('secondaryMech', None) if LOCK else None
    return dict(available=bool(available), modes=modes, destruction_mps=points,
        deployment_rate_curve=first(fm.get('dvFlapsOut')),
        ias_curve=list(map(f32, first(fm.get('flapsLimByIas', [0., 3000., 1., 1.])))),
        mach_curve=list(map(f32, first(fm.get('flapsLimByMach', [.5, .7, 1., 1.])))),
        lock_ias_kmh=f32(lock_ias) if lock_ias < 1e9 else None, lock_mach=f32(lock_mach) if lock_mach < 50. else None,
        secondary=secondary if isinstance(secondary, str) else None,
        secondary_range=[float(x) for x in first(lock.get('secondaryMechRange', [0., 1.]))],
        secondary_dependent=[float(x) for x in first(lock.get('secondaryMechDependentRange', [0., 1.]))])


def destruction_speed(profile, fraction):
    """Native endpoint-held piecewise linear table, returned in m/s IAS."""
    points = profile['destruction_mps']; fraction = f32(fraction)
    if fraction <= points[0][0]:return points[0][1]
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        if fraction <= x1:
            t = mul(sub(fraction, x0), f32(1/sub(x1, x0)))
            return add(mul(sub(y1, y0), t), y0)
    return points[-1][1]


def automatic_extension(profile, ias_kmh, mach):
    def value(row, x):
        x0, x1, y0, y1 = row
        return interval(x, x0, y0, x1, y1)
    return min(value(profile['ias_curve'], ias_kmh), value(profile['mach_curve'], mach))


def curve_speed_limit(row, fraction):
    """First speed where a fixed extension becomes unavailable; no cap = None."""
    x0, x1, y0, y1 = row
    if x1 < x0:x0, x1, y0, y1 = x1, x0, y1, y0
    if fraction > y0:return 0.
    if fraction <= y1:return None
    if x0 == x1:return max(0., x0)
    return max(0., x0+(y0-fraction)*(x1-x0)/(y0-y1))


def secondary_allows(profile, fraction, sweep):
    """S8: whether the secondary-mechanism dependency lets the flaps hold ``fraction`` at this wing sweep (outside
    secondaryMechRange the flap bounds become secondaryMechDependentRange)."""
    if profile.get('secondary') != 'sweep' or sweep is None:return True
    lo, hi = profile['secondary_range'][:2]
    if lo-1e-7 <= sweep <= hi+1e-7:return True
    return fraction <= profile['secondary_dependent'][1]+2e-7


def locked(profile, ias_mps, mach):
    """S8: the flap mechanism lock (forward IAS km/h over mechLockIas, or total Mach over mechLockMachNumber)."""
    lock_ias, lock_mach = profile.get('lock_ias_kmh'), profile.get('lock_mach')
    return (lock_ias is not None and mul(ias_mps, 3.6) > lock_ias) or (lock_mach is not None and mach > lock_mach)


def limits(profile, percent, sweep=None):
    if not profile['available'] or percent <= 0:return None
    fraction = f32(percent/100.)
    # The automatic schedule clamps the requested target BEFORE actuator travel.
    # A takeoff request can therefore deploy a smaller extension from clean;
    # it does not require the full takeoff detent to be maintainable first.
    automatic_ias = curve_speed_limit(profile['ias_curve'], fraction)
    automatic_mach = curve_speed_limit(profile['mach_curve'], fraction)
    # S8: the mechanism lock retracts every extension above its speed; the sweep dependency at any speed
    if profile.get('lock_ias_kmh') is not None:
        automatic_ias = min(v for v in (automatic_ias, float(profile['lock_ias_kmh'])) if v is not None)
    if profile.get('lock_mach') is not None:
        automatic_mach = min(v for v in (automatic_mach, float(profile['lock_mach'])) if v is not None)
    sweep_locked = not secondary_allows(profile, fraction, sweep)
    if sweep_locked:automatic_ias = 0.
    deployment_ias = [deployment_speed_limit(profile.get('deployment_rate_curve')), automatic_ias]
    return dict(percent=float(percent),
        destruction_ias_kmh=destruction_speed(profile, fraction)*3.6,
        damage=bool(fraction > DAMAGE_FRACTION) if LOCK else True,
        deployment_ias_kmh=min((v for v in deployment_ias if v is not None), default=None),
        deployment_mach=automatic_mach,
        automatic_ias_kmh=automatic_ias,
        automatic_mach=automatic_mach, sweep_locked=sweep_locked)


def deployment_speed_limit(row):
    """Speed where outward travel stops; already-deployed flaps can stay out."""
    if row is None:return None
    x0, x1, y0, y1 = map(f32, row)
    if x1 < x0:x0, x1, y0, y1 = x1, x0, y1, y0
    if y1 > 0:return None
    if y0 <= 0:return 0.
    return max(0., x0+y0*(x1-x0)/(y0-y1))


def violations(profile, percent, ias_mps, mach, structural=True, sweep=None):
    if not profile['available'] or percent <= 0:return []
    fraction = f32(percent/100.); reasons = []
    if (fraction > automatic_extension(profile, mul(ias_mps, 3.6), mach)+2e-7 or locked(profile, ias_mps, mach)
            or not secondary_allows(profile, fraction, sweep)):
        reasons.append('flap automatic retraction')
    if structural and (fraction > DAMAGE_FRACTION or not LOCK) and ias_mps > destruction_speed(profile, fraction):
        reasons.append('flap IAS limit')
    return reasons
