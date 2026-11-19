"""Plane info page (Leo, 2026-11-19): key mass, aerodynamic and propulsion figures of one aircraft at its settings, its
flap speed curves and a flap efficiency score (report only; no solver changes).

Figures: the flight model's own values as the solver loads them (fully upgraded, as the catalog applies every
modification); masses and inertia are the solver's mass state at the settings; polar values are the wing polar the
solver evaluates at Mach 0 (flaps up and fully down); a jet's static thrust is the chart's engine force at 0 m and 0 km/h;
a propeller engine's power is the FM's Main.Power (the solver's shaft power at 100% throttle at sea level, checked on
the Bf 109 F-4, P-51D-5, Fw 190 A-5, La-7 and A2D).

Flap curves, in indicated airspeed at the page's altitude:
- schedule: the largest extension the game's flap actuator holds (flap_model: flapsLimByIas, flapsLimByMach and the
  mechanism lock); only aircraft with such a schedule have one;
- critical: the speed where flaps at each extension break (Mass.FlapsDestructionIndSpeedP*), checked by the game above
  5% extension.

Flap efficiency (Leo: "percent gain in lift divided by percent gain in drag at the instructor aoa and the
corresponding speed at each deployment percentage"; later the same day: only the flap settings the aircraft has,
combat, takeoff and landing, one value each). For each setting:
- speed: the EM chart's own speed limit with those flaps at the altitude (em_speed_limits.speed_limits: the lowest of
  the schedule, the lock and the critical speed, or the aircraft's own limit where no flap limit lies below it), a
  hair inside it;
- state: the EM chart's steady level turn at that speed at the Instructor's full pull (Mouse aim): where the march's
  'Instructor pitch' margin or its control authority margin with autotrim reaches zero, whichever comes first, found
  by the chart's own prescribed-alpha correction, once with the flaps and once with flaps up at the same speed. The
  wing's breaking load does not end the pull and the Instructor's wing protection is left out (the solver's Flutter
  off), so the comparison is at the Instructor's AoA limit itself also where the wing would break first (B-17G);
- lift and drag: the aircraft's aerodynamic force (every component, no engine force) across and along the airflow;
- score = (lift / lift flaps up - 1) / (drag / drag flaps up - 1).
"""
import json
import math
import time
from functools import lru_cache

from component_assembly import f32
from em_solver import AIRCRAFT, MAX_ALTITUDE_M, settings

KGF = 9.80665
MASS = ('fuel_percent', 'extra_mass_kg', 'loadout', 'stores_released', 'default_ammunition', 'ammunition_vehicle', 'boosters',
        'booster_fuel_percent')
MARGIN = 1e-5        # Instructor margin (tenths of a degree, or stick travel) that counts as the limit
END_MARGIN = .01     # the last balanced state counts as the limit when its margin is below this,
END_AHEAD = .05      # or when its margin's slope puts the limit within this many degrees of AoA
BRACKET = .05        # widest AoA bracket (deg) of the limit that still gives a state
PROPS = ('piston', 'turboprop', 'mixed')


def solver(name, altitude, throttle, mass='{}', flaps=0., structural=True):
    """The EM chart's solver at these settings (Mouse aim: Instructor and autotrim on), as engine_performance.solver
    builds it, with the compiled evaluator and the march chart's soft admission (states past a soft limit stay
    solvable). structural False: the EM page's Flutter off: the wing's breaking load and the structural speeds reject no
    state, and the Instructor's limit is its AoA limit without the wing-protection cap."""
    import em_march_native
    from em_stationary_baseline import BaselineSolver
    extra = dict(json.loads(mass), altitude_m=float(altitude), throttle=throttle / 100., afterburner=throttle > 100.,
                 flaps_percent=float(flaps), structural_limits=bool(structural))
    chart = BaselineSolver(name, settings(dict(aircraft=[name], sideslip_recovery=False, **extra)))
    em_march_native.attach(chart)
    chart.soft_admission = True
    return chart


def scalar(value, default=None):
    if isinstance(value, list):value = value[0] if value else default
    return default if value is None else value


# ---- Flap curves ----------------------------------------------------------------------------

def ias_factor(altitude):
    """Indicated over true airspeed at an altitude (the game's forward IAS of level flow)."""
    from air_state import cache
    return float(cache([1., 0., 0.], altitude)['ias_u'])


def flap_curves(profile, altitude):
    """(schedule, critical, detents) for the charts: schedule [[ias_kmh, percent], ...] or None without one; critical
    [[ias_kmh, percent], ...] from 5% extension up; detents {combat|takeoff|landing: percent}."""
    from air_state import speed_of_sound
    from flap_model import DAMAGE_FRACTION, automatic_extension, curve_speed_limit, destruction_speed
    if not profile['available']:return None, None, {}
    critical = [(float(destruction_speed(profile, x)) * 3.6, x * 100.) for x in
                sorted({float(DAMAGE_FRACTION), 1.} | {float(x) for x, _ in profile['destruction_mps'] if DAMAGE_FRACTION < x < 1.})]
    mach_to_ias = float(speed_of_sound(altitude)) * 3.6 * ias_factor(altitude)
    lock = min((v for v in (profile['lock_ias_kmh'], profile['lock_mach'] and profile['lock_mach'] * mach_to_ias) if v), default=None)
    scheduled = (curve_speed_limit(profile['ias_curve'], 1.) is not None or curve_speed_limit(profile['mach_curve'], 1.) is not None
                 or lock is not None)
    schedule = None
    if scheduled:
        top = 1.1 * max([v for v, _ in critical] + [lock or 0.] + [x for x in profile['ias_curve'][:2] if x < 2900.])
        knots = {0., top} | {float(x) for x in profile['ias_curve'][:2]} | {float(x) * mach_to_ias for x in profile['mach_curve'][:2]}
        schedule = []
        for v in sorted(k for k in knots if 0. <= k <= top):
            if lock is not None and v > lock:
                if not schedule or schedule[-1][0] < lock:
                    schedule += [(lock, float(automatic_extension(profile, lock, lock / mach_to_ias)) * 100.), (lock, 0.)]
                schedule.append((v, 0.))
            else:schedule.append((v, float(automatic_extension(profile, v, v / mach_to_ias)) * 100.))
    return schedule, critical, {k: v for k, v in profile['modes'].items() if v is not None}


# ---- Flap efficiency ------------------------------------------------------------------------

def instructor_state(chart, speed_kmh, seed=None, why=None):
    """The steady level turn at this speed at the Instructor's full pull: its AoA limit or the end of its control
    authority with autotrim (em_march.margins: 'Instructor pitch', 'control'), with the balanced state the search
    started from; (None, start) where there is none (no balanced flight inside the limit there, or the turn stops
    balancing short of it; why: a list that gets the reason). seed: a balanced state of the same aircraft in another
    configuration at this speed, tried as the start where the unseeded level-flight solve does not close."""
    import em_sampling
    from em_stationary_boundary import Curve
    def margin(p):
        ctl = p.get('instructor') if p else None
        return ctl['margin'] if ctl and ctl.get('converged') and math.isfinite(ctl['margin']) else None
    def fail(reason):
        if why is not None:why.append(reason)
        return None
    curve = Curve(chart, speed_kmh)
    low = None
    try:
        low = em_sampling.solve_level(chart, speed_kmh, None)
        if not low.get('converged') and seed is not None:
            low = chart.solve(speed_kmh, 1., em_sampling.continuation_initial(chart, seed, list(seed['solution'])))
    except ValueError:low = None
    if (low is None or not low.get('converged')) and seed is not None:
        # The same attitude as the seed, load free: any balanced state inside the limit starts the search.
        low = curve.correct(seed, alpha=float(seed['alpha_deg']), max_nfev=40, max_evaluations=400)
    if low is None or not low.get('converged'):return fail('no balanced start'), None
    start, m_low = low, margin(low)
    if m_low is None:return fail('Instructor unresolved at the start'), start
    if m_low <= 0.:return fail('start outside the limit'), start
    at = lambda alpha: curve.correct(low, alpha=float(alpha), max_nfev=40, max_evaluations=400)
    high = m_high = None
    step, slope = 4., None
    for _ in range(80):
        # Aim a little past where the margin's slope puts the limit, at most one step up.
        p = at(low['alpha_deg'] + step)
        m = margin(p) if p is not None else None
        if m is None:
            step *= .5
            if step < .002:
                # No balanced state above: the stick is at its stop, or the margin is falling to zero within END_AHEAD.
                if m_low < END_MARGIN or slope is not None and m_low / slope < END_AHEAD:return low, start
                return fail('no balanced state up to the limit (margin %.3f at %.2f deg)' % (m_low, low['alpha_deg'])), start
        elif m > 0.:
            slope = (m_low - m) / (p['alpha_deg'] - low['alpha_deg']) if m_low > m else None
            ahead = m / slope if slope else step
            low, m_low, step = p, m, min(4., max(.004, 1.05 * ahead))
        else:
            high, m_high = p, m
            break
    if high is None:return fail('limit not reached'), start
    # Safeguarded secant (Illinois): an end kept twice in a row counts half.
    w_low = w_high = 1.
    for _ in range(60):
        if m_low <= MARGIN or high['alpha_deg'] - low['alpha_deg'] < 1e-5:break
        a = low['alpha_deg'] + (high['alpha_deg'] - low['alpha_deg']) * w_low * m_low / (w_low * m_low - w_high * m_high)
        p = at(a)
        m = margin(p) if p is not None else None
        if m is None:
            p = at(.5 * (low['alpha_deg'] + high['alpha_deg']))
            m = margin(p) if p is not None else None
            if m is None:break   # the admitted side found so far is within the bracket of the limit
        if m > 0.:low, m_low, w_low, w_high = p, m, 1., .5 * w_high
        else:high, m_high, w_high, w_low = p, m, 1., .5 * w_low
    if high['alpha_deg'] - low['alpha_deg'] > BRACKET and m_low > END_MARGIN:
        return fail('limit bracketed only to %.2f deg' % (high['alpha_deg'] - low['alpha_deg'])), start
    return low, start


def aerodynamic(point):
    """(lift, drag) in N: the aerodynamic force of every component across and along the airflow (zero sideslip)."""
    fx = sum(float(f[0]) for f in point['component_forces'].values())
    fy = sum(float(f[1]) for f in point['component_forces'].values())
    a = math.radians(point['alpha_deg'])
    return fx * math.sin(a) + fy * math.cos(a), -(fx * math.cos(a) - fy * math.sin(a))


FLAP_LIMITS = {'Flap IAS limit': 'critical', 'Flap automatic IAS limit': 'schedule', 'Flap automatic Mach limit': 'schedule',
               'VNE': 'VNE', 'Mach limit': 'Mach limit', 'Wing sweep limit': 'wing sweep'}
MODES = ('combat', 'takeoff', 'landing')


def efficiency(name, altitude, throttle, mass, detents):
    from em_speed_limits import speed_limits
    """One row per flap setting the aircraft has (flap_model.profile modes), in the game's order."""
    clean = solver(name, altitude, throttle, mass, 0., structural=False)
    factor = ias_factor(altitude)
    rows = []
    for mode in MODES:
        percent = detents.get(mode)
        if percent is None or percent <= 0.:continue
        percent = float(percent)
        chart = solver(name, altitude, throttle, mass, percent, structural=False)
        limit = speed_limits(chart.fm, dict(chart.config, structural_limits=True))
        row = dict(mode=mode, percent=percent, limit=FLAP_LIMITS.get(limit['kind'], limit['kind']))
        speed = limit['sample_speed_kmh']
        row.update(speed_kmh=speed, ias_kmh=speed * factor)
        why = []
        reference, level = instructor_state(clean, speed, why=why)
        flapped, _ = instructor_state(chart, speed, seed=level, why=why)
        if why:row['unsolved'] = '; '.join(why)
        if flapped is not None and reference is not None:
            (lift, drag), (lift0, drag0) = aerodynamic(flapped), aerodynamic(reference)
            if lift0 > 0. and drag0 > 0.:
                lift_gain, drag_gain = (lift / lift0 - 1.) * 100., (drag / drag0 - 1.) * 100.
                limiting = lambda p: 'aoa' if p['instructor']['limiting'] == 'Instructor effective AoA' else 'control'
                row.update(mach=float(flapped['mach']), lift_gain_percent=lift_gain, drag_gain_percent=drag_gain,
                           limiting=limiting(flapped), clean_limiting=limiting(reference),
                           score=lift_gain / drag_gain if abs(drag_gain) > 1e-9 else None,
                           alpha_deg=flapped['alpha_deg'], clean_alpha_deg=reference['alpha_deg'],
                           load_g=flapped['load_g'], clean_load_g=reference['load_g'],
                           lift_n=lift, drag_n=drag, clean_lift_n=lift0, clean_drag_n=drag0)
        rows.append(row)
    return rows


# ---- Figures --------------------------------------------------------------------------------

def figure(label, value, unit='', digits=0, note=None):
    """One table row: value a number, a list of numbers (shown a / b) or text; None leaves the row out. Labels are the
    same for every aircraft (the page lines rows up by label); note: a short remark shown after the value."""
    if value is None or isinstance(value, list) and all(v is None for v in value):return None
    if isinstance(value, list):value = [None if v is None else float(v) for v in value]
    elif not isinstance(value, str):value = float(value)
    return dict(label=label, value=value, unit=unit, digits=digits, **({'note': note} if note else {}))


def section(title, rows):
    return dict(title=title, rows=[r for r in rows if r is not None])


def mass_figures(chart, entry):
    m, fm_mass, area = chart.mass, chart.fm['Mass'], float(chart.model['geometry']['area'])
    total, fuel, payload, extra = (float(m[k]) for k in ('mass', 'fuel_mass', 'payload_mass', 'additional_mass_kg'))
    empty = float(scalar(fm_mass.get('EmptyMass'), 0.))
    rest = total - empty - fuel - payload - extra
    return section('MASS', [
        figure('Empty', empty, 'kg'),
        figure('Crew and oil', rest if rest > .5 else None, 'kg'),
        figure('Fuel', fuel, 'kg', note=f"{chart.config['fuel_percent']:g}%"),
        figure('Fuel capacity', entry.get('fuel_capacity'), 'kg'),
        figure('Weapons and ammunition', payload if payload > 0. else None, 'kg'),
        figure('Additional', extra if extra > 0. else None, 'kg'),
        figure('Total', total, 'kg'),
        figure('Maximum takeoff', scalar(fm_mass.get('Takeoff')), 'kg'),
        figure('Wing loading', total / area if area > 0. else None, 'kg/m²', 1),
        figure('Inertia · roll / yaw / pitch', [float(v) for v in m['inertia']], 'kg·m²')])


def polar_at(chart, flaps):
    from aircraft_model import condition_properties
    return condition_properties(chart.model, f32(0.), flaps)[1]


def instructor_multiplier(chart):
    """The Instructor's share of the critical AoA in Mouse aim (instructor_protection.angle_targets, mode lane)."""
    from instructor_chart_inputs import GAMEPLAY
    props = dict(GAMEPLAY['propsDefault']);props.update(chart.fm.get('Instructor', {}))
    critical = [float(v) for v in props.get('critMult', [-1., -1.])]
    return .8 if critical[0] < 0. else critical[0]


def aerodynamic_figures(chart, entry):
    g, ad = chart.model['geometry'], chart.fm['Aerodynamics']
    span, area = float(g['span']), float(g['area'])
    clean, full = polar_at(chart, 0.), polar_at(chart, 1.) if entry.get('has_flaps') else None
    pair = lambda f: [f(clean)] + ([f(full)] if full is not None else [])
    suffix = ' · clean / full flaps'
    share = instructor_multiplier(chart)
    wing = ad.get('WingPlane') or ad.get('WingPlaneSweep0') or {}
    oswald = scalar((wing.get('FlapsPolar0') or {}).get('OswaldsEfficiencyNumber'), scalar(ad.get('OswaldsEfficiencyNumber')))
    drag = lambda key: (lambda v: v if v and v > 0. else None)(scalar(ad.get(key)))
    return section('AERODYNAMICS', [
        figure('Wing area', area, 'm²', 2),
        figure('Wingspan', span, 'm', 2),
        figure('Aspect ratio', span * span / area if area > 0. else None, '', 2),
        figure('Sweep', float(g['sweep']) if g.get('sweep') else None, '°', 1),
        figure('Lift slope', float(clean['clLineCoeff']) * float(clean['cyMult']), '/°', 4),
        figure('Oswald efficiency', oswald, '', 2),
        figure('CL max' + suffix, pair(lambda p: float(p['cyCritH']) * float(p['clKq'])), '', 2),
        figure('Critical AoA' + suffix, pair(lambda p: float(p['aoaCritH'])), '°', 1),
        figure('Instructor AoA' + suffix, pair(lambda p: share * float(p['aoaCritH'])), '°', 1),
        figure('CD min' + suffix, pair(lambda p: float(p['cd0'])), '', 4),
        figure('Gear CD', drag('GearCd'), '', 4),
        figure('Airbrake CD', drag('AirbrakeCd'), '', 4)])


def limit_figures(chart, entry, profile):
    from flap_model import destruction_speed
    strength, fm_mass = chart.model['geometry']['strength'], chart.fm['Mass']
    props = chart.fm.get('Instructor') or {}
    gear = scalar(fm_mass.get('GearDestructionIndSpeed'))
    rows = [figure('VNE', float(strength['ias']) * 3.6, 'km/h IAS'),
            figure('Mach limit', float(strength['mach']), '', 2),
            figure('Wing strength · + / −', [float(strength['force'][1]) / 1000., -float(strength['force'][0]) / 1000.], 'kN per wing', 1),
            figure('Instructor load limit · + / −', [float(props['loadFactorLimit'][1]), -float(props['loadFactorLimit'][0])]
                   if props.get('limitLoadfactor') and props.get('loadFactorLimit') else None, 'g', 1),
            figure('Gear limit', gear if gear and gear > 0. else None, 'km/h IAS')]
    if profile['available']:
        for mode in ('combat', 'takeoff', 'landing'):
            percent = profile['modes'].get(mode)
            if percent is not None:
                rows.append(figure(f'Flaps critical · {mode}', float(destruction_speed(profile, percent / 100.)) * 3.6, 'km/h IAS', note=f'{percent:g}%'))
    return section('LIMITS', rows)


ENGINE_NAMES = {'Inline': 'inline piston', 'Radial': 'radial piston', 'Jet': 'jet', 'TurboProp': 'turboprop', 'Rocket': 'rocket'}


def propulsion_figures(chart, entry, name, mass):
    from jet_catalog import engines
    installed = [e for _, e in engines(chart.fm)]
    kinds = {}
    for e in installed:
        kind = ENGINE_NAMES.get(e.get('Main', {}).get('Type'), str(e.get('Main', {}).get('Type')))
        kinds[kind] = kinds.get(kind, 0) + 1
    rows = [figure('Engines', ' + '.join(f'{n} × {k}' for k, n in kinds.items()))]
    total = float(chart.mass['mass'])
    if entry['propulsion'] in PROPS:
        mains = [e for e in installed if e.get('Main', {}).get('Type') in ('Inline', 'Radial', 'TurboProp')]
        e = mains[0] if mains else {}
        main, prop, compressor = e.get('Main', {}), e.get('Propellor', {}), e.get('Compressor', {})
        power = scalar(main.get('Power'))
        rows += [figure('Power per engine · 100% at 0 m', power, 'hp'),
                 figure('Power / weight', power * len(mains) / total if power else None, 'hp/kg', 3)]
        if main.get('Type') != 'TurboProp':
            for i in range(int(scalar(compressor.get('NumSteps'), 0))):
                stage, height = scalar(compressor.get(f'Power{i}')), scalar(compressor.get(f'Altitude{i}'))
                if stage and height is not None:
                    rows.append(figure(f'Supercharger gear {i + 1}', f'{stage:,.0f} hp at {height:,.0f} m'))
        rows += [figure('RPM · 100% / WEP / limit', [scalar(main.get('RPMMax')), scalar(main.get('RPMAfterburner')), scalar(main.get('RPMMaxAllowed'))], 'rpm'),
                 figure('Propeller', f"{scalar(prop.get('Diameter'), 0.):g} m · {int(scalar(prop.get('NumBlades'), 0))} blades" if prop.get('Diameter') else None),
                 figure('Blade angle', [scalar(prop.get('PhiMin')), scalar(prop.get('PhiMax'))] if prop.get('PhiMax') is not None else None, '°', 1),
                 figure('Reduction', scalar(prop.get('Reductor')), '', 3)]
    else:
        # Static thrust at sea level from the chart's own engine force, at 100% and (where the throttle goes there) 110%.
        from engine_performance import without_wep
        tops = [100.] if name in without_wep() else [100., 110.]
        thrust = [float(solver(name, 0., t, mass).engine.vectors(0.)[0][0]) / KGF for t in tops]
        main = installed[0].get('Main', {}) if installed else {}
        rows += [figure('Static thrust at 0 m · 100% / 110%', thrust, 'kgf'),
                 figure('Thrust / weight · 100% / 110%', [t / total for t in thrust], '', 2)]
        if main.get('Type') == 'Jet':
            rows.append(figure('RPM · 100% / WEP / limit', [scalar(main.get('RPMMax')), scalar(main.get('RPMAfterburner')), scalar(main.get('RPMMaxAllowed'))], 'rpm'))
    return section('PROPULSION', rows)


@lru_cache(maxsize=64)
def compute(name, altitude, throttle, mass='{}'):
    from engine_performance import LOCK
    entry = AIRCRAFT[name]
    if not entry.get('supported'):raise ValueError(entry.get('reason') or 'Aircraft not supported')
    started = time.perf_counter()
    with LOCK:
        chart = solver(name, altitude, throttle, mass)
        profile = entry['flaps']
        schedule, critical, detents = flap_curves(profile, altitude)
        return dict(aircraft_id=name, name=entry['name'], propulsion=entry['propulsion'], altitude_m=altitude, throttle_percent=throttle,
                    mass_kg=float(chart.mass['mass']),
                    sections=[mass_figures(chart, entry), aerodynamic_figures(chart, entry), limit_figures(chart, entry, profile),
                              propulsion_figures(chart, entry, name, mass)],
                    flaps=dict(available=bool(profile['available']), schedule=schedule, critical=critical, detents=detents,
                               efficiency=efficiency(name, altitude, throttle, mass, detents) if profile['available'] else []),
                    elapsed_s=time.perf_counter() - started)


def request(body):
    """POST /api/plane: {aircraft, altitude_m, throttle, mass: {fuel_percent, extra_mass_kg, loadout, stores_released,
    default_ammunition, ammunition_vehicle, boosters, booster_fuel_percent}}."""
    known = {'aircraft', 'altitude_m', 'throttle', 'mass'}
    if set(body) - known:raise ValueError('Unknown field: ' + ', '.join(sorted(set(body) - known)))
    name = body.get('aircraft')
    if not isinstance(name, str) or name not in AIRCRAFT:raise ValueError('Unknown aircraft')
    def number(key, default, low, high):
        v = body.get(key, default)
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or not low <= v <= high:
            raise ValueError(key + ' out of range')
        return float(v)
    altitude, throttle = number('altitude_m', 0., 0., float(MAX_ALTITUDE_M)), number('throttle', 110., 0., 110.)
    from engine_performance import without_wep
    if throttle > 100. and name in without_wep():raise ValueError('This aircraft has no WEP: throttle up to 100%')
    mass = body.get('mass', {})
    if not isinstance(mass, dict) or set(mass) - set(MASS):raise ValueError('Unknown mass setting')
    # settings() checks every value when the solver is built; the text is canonical so equal settings share a result.
    return compute(name, altitude, throttle, json.dumps(mass, sort_keys=True))
