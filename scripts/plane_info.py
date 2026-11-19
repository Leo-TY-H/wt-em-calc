"""Plane info page (Leo, 2026-11-19): key mass, aerodynamic and propulsion figures of one aircraft at its settings, its
flap speed curves (report only; no solver changes).

Figures: the flight model's own values as the solver loads them (fully upgraded, as the catalog applies every
modification); masses and inertia are the solver's mass state at the settings; polar values are the wing polar the
solver evaluates at Mach 0 (flaps up and fully down); a jet's static thrust is the chart's engine force at 0 m and 0 km/h;
a propeller engine's power is the FM's Main.Power (the solver's shaft power at 100% throttle at sea level, checked on
the Bf 109 F-4, P-51D-5, Fw 190 A-5, La-7 and A2D).

Flap speeds, in indicated airspeed at the page's altitude; only the limits that apply are drawn (Leo, 2026-11-19: "use
whatever limits that apply. for example on adf only the auto flap schedule apply"; "extension schedule and retraction
schedule are not necessarily the same"):
- holding an extension: the lower of the automatic retraction (flap_model: the actuator target capped by flapsLimByIas,
  flapsLimByMach and the mechanism lock) and the critical speed where the flaps break (Mass.FlapsDestructionIndSpeedP*,
  checked by the game above 5% extension); each drawn only where it is the lower one;
- extending: the speed where the flaps' outward travel stops (dvFlapsOut reaching zero, flap_model.deployment_speed_limit;
  flaps already out stay out), drawn below the holding limit.

A flap efficiency score (lift gain over drag gain at the Instructor's full pull) was built and removed the same day
(Leo, 2026-11-19: "just remove this feature"); it is in the branch history before this commit.
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
PROPS = ('piston', 'turboprop', 'mixed')


def solver(name, altitude, throttle, mass='{}'):
    """The EM chart's solver at these settings (Mouse aim: Instructor and autotrim on), as engine_performance.solver
    builds it, with the compiled evaluator."""
    import em_march_native
    from em_stationary_baseline import BaselineSolver
    extra = dict(json.loads(mass), altitude_m=float(altitude), throttle=throttle / 100., afterburner=throttle > 100.,
                 flaps_percent=0.)
    chart = BaselineSolver(name, settings(dict(aircraft=[name], sideslip_recovery=False, **extra)))
    em_march_native.attach(chart)
    return chart


def scalar(value, default=None):
    if isinstance(value, list):value = value[0] if value else default
    return default if value is None else value


# ---- Flap curves ----------------------------------------------------------------------------

def ias_factor(altitude):
    """Indicated over true airspeed at an altitude (the game's forward IAS of level flow)."""
    from air_state import cache
    return float(cache([1., 0., 0.], altitude)['ias_u'])


def holding_limit(profile, percent, altitude):
    """(IAS km/h, kind) of the lowest limit on holding an extension at this altitude: kind 'retraction' (automatic, by
    forward IAS: flapsLimByIas or mechLockIas), 'retraction_mach' (automatic, by the Mach of the total airspeed:
    flapsLimByMach or mechLockMachNumber, placed at the IAS of that Mach in level flow at this altitude) or 'critical'
    (the flaps break, by forward IAS); (None, None) where none applies. The game's own quantities (flap_model,
    em_speed_limits: aces.exe traced 2026-10-19)."""
    from air_state import speed_of_sound
    from flap_model import limits
    row = limits(profile, percent)
    if row is None:return None, None
    mach_to_ias = float(speed_of_sound(altitude)) * 3.6 * ias_factor(altitude)
    candidates = (([(float(row['automatic_ias_kmh']), 'retraction')] if row['automatic_ias_kmh'] is not None else [])
                  + ([(float(row['automatic_mach']) * mach_to_ias, 'retraction_mach')] if row['automatic_mach'] is not None else [])
                  + ([(float(row['destruction_ias_kmh']), 'critical')] if row['damage'] else []))
    return min(candidates) if candidates else (None, None)


def flap_limits(profile, altitude):
    """(segments, detents) for the flap speeds chart: segments [{kind, points: [[ias_kmh, percent], ...]}] of the limits
    that apply (kind 'retraction' | 'retraction_mach' | 'critical' | 'extension'); detents {combat|takeoff|landing:
    percent}."""
    from flap_model import deployment_speed_limit
    if not profile['available']:return [], {}
    segments = []
    # Every half percent from fully out down to (nearly) in; a change of the lower limit starts a new segment at the
    # same point so the lines meet.
    for i in range(200, -1, -1):
        percent = max(.5 * i, 1e-3)
        speed, kind = holding_limit(profile, percent, altitude)
        if kind is None:continue
        point = [float(speed), .5 * i]
        if segments and segments[-1]['kind'] != kind and abs(segments[-1]['points'][-1][1] - point[1]) <= .5:
            segments[-1]['points'].append(point)
        if not segments or segments[-1]['kind'] != kind:segments.append(dict(kind=kind, points=[]))
        segments[-1]['points'].append(point)
    for segment in segments:
        # Straight runs keep only their ends.
        pts, kept = segment['points'], []
        for j, q in enumerate(pts):
            if 0 < j < len(pts) - 1:
                (x0, y0), (x1, y1) = kept[-1], pts[j + 1]
                # within 0.02 km/h or percent of the straight line between its neighbours (the speeds are binary32)
                if abs((q[0] - x0) * (y1 - y0) - (q[1] - y0) * (x1 - x0)) <= .02 * math.hypot(x1 - x0, y1 - y0):continue
            kept.append(q)
        segment['points'] = kept
    stop = deployment_speed_limit(profile.get('deployment_rate_curve'))
    if stop is not None:
        # Extensions that can be held at the speed where outward travel stops: the line runs up to the highest.
        held = [.5 * i for i in range(1, 201) if (lambda v: v is None or v > stop)(holding_limit(profile, .5 * i, altitude)[0])]
        if held:segments.append(dict(kind='extension', points=[[float(stop), 0.], [float(stop), max(held)]]))
    return segments, {k: v for k, v in profile['modes'].items() if v is not None}


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
        limits, detents = flap_limits(profile, altitude)
        from air_state import speed_of_sound
        return dict(aircraft_id=name, name=entry['name'], propulsion=entry['propulsion'], altitude_m=altitude, throttle_percent=throttle,
                    mass_kg=float(chart.mass['mass']),
                    sections=[mass_figures(chart, entry), aerodynamic_figures(chart, entry), limit_figures(chart, entry, profile),
                              propulsion_figures(chart, entry, name, mass)],
                    flaps=dict(available=bool(profile['available']), limits=limits, detents=detents,
                               ias_per_mach=float(speed_of_sound(altitude)) * 3.6 * ias_factor(altitude)),
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
