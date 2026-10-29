"""Engine and propeller performance for the website's Engine performance page (report only; no solver changes).

The EM plotter's own logic throughout (Leo, 2026-10-28): each point is calculated by a solver built from the same
settings as an EM chart at that altitude (the EM page's throttle slider: setting / 100, boost requested above 100%;
its manual propeller control; every other setting at the EM defaults, fully upgraded as the catalog applies every
modification). Propeller aircraft: the EM plotter's axial-flow propulsion state (em_stationary_baseline.AxialReference:
BaselineSolver.operating_point at 1 g, angle of attack, bank and controls 0), kept when converged and admitted by the
chart's propeller admission. Jets: the chart's engine force (EngineEnsemble.vectors). Two sweeps: power and the rest
over altitude at one true airspeed, and over true airspeed at one altitude.
"""
import json
import math
import threading
import time
from collections import OrderedDict
from functools import lru_cache

import numpy as np

from component_assembly import f32
from em_solver import AIRCRAFT, settings

METRIC_HP = 735.49875
KGF = 9.80665
LOCK = threading.Lock()


# Default ranges (Leo, 2026-10-28): true airspeed 0-800 km/h for propeller aircraft, 0-1,300 for jets; altitude 0-10 km.
SPEEDS = {'jet': (0., 1300.)}
ALTITUDES = (0., 10000.)


def sweeps(propulsion, altitudes=None, speeds=None):
    """Altitudes (m) and true airspeeds (km/h) of the two sweeps' base grids (Leo, 2026-10-29: high quality): every 50 m
    and 2 km/h for propeller aircraft, 100 m and 5 km/h for jets, at most 800 steps over the range. A propeller needs
    inflow: no speed below 5 km/h."""
    jet = propulsion in ('jet', 'rocket')
    def grid(lo, hi, step):
        n = min(800, max(1, math.ceil((hi - lo) / step - 1e-9)))
        return [lo + (hi - lo) * i / n for i in range(n + 1)]
    altitudes = grid(*(altitudes or ALTITUDES), 100. if jet else 50.)
    speeds = grid(*(speeds or SPEEDS.get(propulsion, (0., 800.))), 5. if jet else 2.)
    if not jet:speeds = sorted({max(v, 5.) for v in speeds})
    return altitudes, speeds


@lru_cache(maxsize=1)
def without_wep():
    """Aircraft whose throttle stops at 100% in the game (Leo, 2026-10-28; aces.exe pilot throttle 0x1430afc00: each
    engine's throttle is capped at 110% when max(override, ThrottleBoost) > 1, else at 100%). The pages cap their
    throttle sliders at 100% for these; the solver itself is unchanged. (The game's WEP boost flag, 0x142fa0960, is the
    jet rule engine_commands.afterburner_command; the solver gives props the raw flag, set above 100%. With this cap that
    changes no result, checked 2026-10-28 on the only props concerned: Wyvern S4 at 110% with the flag on or off, and the
    Pe-2-205/359, whose WEP boost the game engages at 100%, with the boost on or off.)"""
    from aircraft_catalog import load
    from jet_catalog import engines
    out = []
    for name in AIRCRAFT:
        row = AIRCRAFT[name]
        if not row.get('supported') or row['propulsion'] == 'rocket':continue
        try:mains = [e for _, e in engines(load(name)) if e.get('Main', {}).get('Type') != 'Rocket']
        except (OSError, ValueError, KeyError):continue
        if mains and not any(e.get('Main', {}).get('ThrottleBoost', 1.) > 1. for e in mains):out.append(name)
    return sorted(out)


# The EM page's mass settings the popup carries (Leo, 2026-10-28), as JSON text so results can be cached.
MASS = ('fuel_percent', 'extra_mass_kg', 'flaps_percent', 'loadout', 'stores_released', 'default_ammunition', 'ammunition_vehicle')


def solver(name, altitude, throttle, pitch, mass='{}'):
    """The EM chart's solver for these settings (built per altitude: the catalog caches the aircraft, so this is cheap)."""
    from em_stationary_baseline import BaselineSolver
    extra = dict(json.loads(mass), altitude_m=float(altitude), throttle=throttle / 100., afterburner=throttle > 100.)
    if pitch is not None:extra.update(manual_engine_controls=True, propeller_pitch_auto=False, propeller_pitch_percent=pitch)
    return BaselineSolver(name, settings(dict(aircraft=[name], sideslip_recovery=False, **extra)))


def propeller_row(chart, speed_kmh, report):
    from em_operating import flight_condition
    from prop_admission import evaluate as admission
    from prop_quasisteady import propeller_condition
    from propulsion_general import fixed_air
    attitude = [0.] * 5;v = speed_kmh / 3.6
    prop = chart.operating_point(v, 1., attitude)['propulsion']
    if not prop.get('converged'):return None
    p, state = chart.engine.properties, prop['state']
    # Over-rev states are not results (Leo, 2026-10-29): an engine above RPMMaxAllowed, the chart's 'engine RPM limit' test.
    if any(e['omega'] > p['engines'][i]['properties']['omega_limit'] * (1 + 1e-6) for i, e in enumerate(state['engines'])):
        return None
    velocity, _, geometry, _ = flight_condition(chart, v, 1., attitude)
    velocity, rates = tuple(map(f32, velocity)), tuple(map(f32, geometry['omega']))
    air, sound = fixed_air(velocity, chart.config['altitude_m'])
    tips = [None] * len(p['propellers'])
    for i, shaft in enumerate(p['transmissions']):
        for link in shaft['propellers']:
            pp = p['propellers'][link['index']]['properties'];rotor = state['propellers'][link['index']]
            row = admission(pp, rotor['flow'], rotor['pitch'], state['transmissions'][i]['omega'] * link['ratio'],
                            air['density'], sound, propeller_condition(pp, velocity, rates))
            if not row['admitted']:return None
            tips[link['index']] = max((s['mach'] for r in row['rotors'] for s in r['sections']), default=None)
    power = sum(e.get('torque', 0.) * e['omega'] for e in state['engines'])
    pistons = [i for i, e in enumerate(p['engines']) if e['family'] in (0, 1)]
    # The clicked-point report's pitch % (em_server.pitch_percent) from the same fields the chart's points carry.
    point = dict(propulsion=dict(controls=prop['controls'], stationarity=prop['stationarity'],
                                 engine_rpm=[e['omega'] * 30 / math.pi for e in state['engines']],
                                 propeller_pitch_deg=[math.degrees(q['pitch']) for q in state['propellers']]))
    if report is not None:report(point, chart)
    thrust = float(prop['force'][0])
    return dict(thrust_kgf=thrust / KGF, power_hp=power / METRIC_HP if power > 0. or pistons else None,
                engine_power_hp=[e.get('torque', 0.) * e['omega'] / METRIC_HP for e in state['engines']],
                efficiency=thrust * v / power * 100. if power > 1. and not any(e['family'] == 2 for e in p['engines']) else None,
                rpm=point['propulsion']['engine_rpm'], pitch_deg=point['propulsion']['propeller_pitch_deg'],
                pitch_percent=point['propulsion'].get('propeller_pitch_percent'),
                manifold_ata=[float(state['engines'][i].get('manifold', math.nan)) for i in pistons],
                gear=[int(state['engines'][i].get('gear', 0)) + 1 for i in pistons],
                governor=list(prop['stationarity'].get('governor_status') or ()), tip_mach=tips,
                mach=float(air['mach']), ias_kmh=float(air['ias_u']) * 3.6)


def jet_row(chart, speed_kmh):
    from air_state import cache
    v = speed_kmh / 3.6
    air = cache([f32(v), 0., 0.], chart.config['altitude_m'])
    return dict(thrust_kgf=float(chart.engine.vectors(v)[0][0]) / KGF, rpm=[u.nominal(v)['omega'] * 30 / math.pi for u in chart.engine.units],
                mach=float(air['mach']), ias_kmh=float(air['ias_u']) * 3.6)


def level_solver(name, altitude, throttle, pitch, mass='{}'):
    """The EM plotter's point solver (em_server.point_solver for a march chart): soft admission, compiled loop."""
    import em_march_native
    chart = solver(name, altitude, throttle, pitch, mass)
    chart.soft_admission = True;em_march_native.attach(chart)
    return chart


def level_row(chart, speed_kmh, near):
    """SEP in 1 g level flight (Leo, 2026-10-28): the EM plotter's point solve at 1 g, seeded from a nearby solved point
    as em_server.solve_point seeds a clicked one; kept when the chart admits it and no engine over-revs (Leo, 2026-10-29).
    Returns (row, seed): seed holds what em_sampling.continuation_initial reads from the point."""
    from em_sampling import continuation_initial
    try:p = chart.solve(speed_kmh, 1., continuation_initial(chart, near, list(near['solution'])) if near is not None else None)
    except ValueError:return None, None
    prop = p.get('propulsion') or {}
    if not p.get('valid') or any(r > l * (1 + 1e-6) for r, l in zip(prop.get('engine_rpm') or (), prop.get('engine_rpm_threshold') or ())):
        return None, None
    v, a, f = speed_kmh / 3.6, math.radians(p['alpha_deg']), p['engine_force_n']
    return dict(ps_mps=p['ps_mps'], alpha_deg=p['alpha_deg'],
                ias_kmh=p['ias_kmh'], mach=p['mach'], thrust_kgf=(f[0] * math.cos(a) - f[1] * math.sin(a)) / KGF,
                power_hp=sum(prop['shaft_power_kw']) * 1000. / METRIC_HP if prop.get('shaft_power_kw') else None,
                rpm=prop.get('engine_rpm')), \
        {k:p[k] for k in ('solution', 'stationary_coordinate_names', 'stationary_coordinates', 'stationary_request_indices') if k in p}


GIVE_UP = 25   # consecutive unsolved base points that end a march (50 km/h or 1,250 m for props)


POINTS = OrderedDict()   # sweep identity -> {x: (row, seed)}


def points(key):
    """The points already solved for one sweep (aircraft, settings, axis, fixed value), kept across requests: another plot
    of the same sweep, another range or a repeat solves only the points it lacks (Leo, 2026-10-29: no redundant
    calculations)."""
    store = POINTS.setdefault(key, {});POINTS.move_to_end(key)
    while len(POINTS) > 32:POINTS.popitem(last=False)
    return store


def march(xs, at, starts, blank):
    """A sweep's base grid: solve at a start that converges, then march outward (a level-flight at(x) seeds each point
    from the nearest solved one). A march stops after GIVE_UP unsolved points in a row (over-rev, below the stall, past
    the ceiling): the rest of that side is left unsolved (blank(x)) without being solved."""
    rows = {}
    start = 0
    for s in starts:
        start = min(range(len(xs)), key=lambda i:abs(xs[i] - s))
        if xs[start] not in rows:rows[xs[start]] = at(xs[start])
        if rows[xs[start]]['solved']:break
    for side in (range(start + 1, len(xs)), range(start - 1, -1, -1)):
        misses = 0
        for i in side:
            if xs[i] not in rows:rows[xs[i]] = at(xs[i]) if misses < GIVE_UP else blank(xs[i])
            misses = 0 if rows[xs[i]]['solved'] else misses + 1
    return [rows[x] for x in xs]


# The plotted quantities (Leo, 2026-10-28: power, thrust, efficiency, SEP; RPM, blade pitch and manifold only in details).
CURVES = {'power':lambda r:r.get('power_hp'), 'thrust':lambda r:r.get('thrust_kgf'), 'efficiency':lambda r:r.get('efficiency'),
          'sep':lambda r:r.get('ps_mps')}
TOLERANCE = .0005  # of each plotted quantity's range over the sweep: the largest straight-line midpoint error kept
EXTRA = 1500       # most refinement points added to one sweep


def refined(rows, at, x, step, curves):
    """The sweep with points added where a straight line between neighbours misrepresents the plotted curves (Leo,
    2026-10-28: the fixed grid cut corners; 2026-10-29: only for the plot asked for). An interval is split where one end
    is solved and the other not, or where the base grid bends there (second difference above the tolerance); a split
    keeps halving while the midpoint is off the chord or the solved state changes inside, down to `step`."""
    def kind(r):return r['solved']
    values = [[f(r) for r in rows] for f in curves]
    # A flat quantity (RPM held by the governor, manifold below critical altitude) varies only by float32 noise: its
    # scale is at least 1% of its size.
    scale = [TOLERANCE * max(max(f) - min(f), .01 * max(map(abs, f))) if len(f := [v for v in vs if v is not None and math.isfinite(v)]) > 1
             else 0. for vs in values]
    def bent(a, m, b):
        for f, s in zip(curves, scale):
            ya, ym, yb = f(a), f(m), f(b)
            if s > 0. and None not in (ya, ym, yb) and abs(ym - .5 * (ya + yb)) > s:return True
        return False
    def candidate(i):
        if kind(rows[i]) != kind(rows[i + 1]):return True
        return any(0 < j < len(rows) - 1 and bent(rows[j - 1], rows[j], rows[j + 1]) for j in (i, i + 1))
    budget = [EXTRA]
    def fill(a, b):
        if x(b) - x(a) <= step or budget[0] <= 0:return []
        budget[0] -= 1
        m = at(.5 * (x(a) + x(b)))
        if kind(a) == kind(m) == kind(b) and not bent(a, m, b):return [m]
        return fill(a, m) + [m] + fill(m, b)
    out = [rows[0]]
    for i in range(len(rows) - 1):
        if candidate(i):out += fill(rows[i], rows[i + 1])
        out.append(rows[i + 1])
    return out


@lru_cache(maxsize=32)
def compute(name, altitude_m, speed_kmh, throttle=110., pitch=None, report=None, sweep=None, level=False, mass='{}',
            altitudes=None, speeds=None, quantity=None):
    """The sweeps at a throttle setting (%): sweep 'altitude' (at speed_kmh), 'speed' (at altitude_m) or None for both.
    pitch: manual propeller control (%) or None for automatic control. level: SEP in 1 g level flight instead of the
    engine at angle of attack 0. mass: JSON of the MASS settings (EM defaults where left out). altitudes, speeds: the
    sweep ranges (min, max), defaults SPEEDS / ALTITUDES. quantity: the plotted CURVES key the refinement follows (None:
    all of them)."""
    entry = AIRCRAFT[name]
    if not entry['supported']:raise ValueError(entry['name'] + ': ' + entry['reason'])
    if entry['propulsion'] == 'rocket':raise ValueError('Rocket aircraft have no engine performance page')
    prop = entry['propulsion'] not in ('jet', 'rocket')
    if pitch is not None and not (prop and entry.get('manual_controls', {}).get('propeller_pitch')):
        raise ValueError('Manual propeller pitch is unavailable for ' + entry['name'])
    if throttle > 100. and name in without_wep():raise ValueError(entry['name'] + ' has no WEP: its throttle stops at 100%')
    started = time.perf_counter()
    altitudes, speeds = sweeps(entry['propulsion'], altitudes, speeds)
    def row(chart, height, speed):
        try:value = propeller_row(chart, speed, report) if prop else jet_row(chart, speed)
        except (ValueError, ArithmeticError, np.linalg.LinAlgError):value = None
        return dict(value or {}, altitude_m=height, speed_kmh=speed, solved=value is not None)
    def flight(chart, height, speed, near):
        try:value, seed = level_row(chart, speed, near)
        except (ArithmeticError, np.linalg.LinAlgError):value = seed = None
        return dict(value or {}, altitude_m=height, speed_kmh=speed, solved=value is not None), seed
    curves = [CURVES[quantity]] if quantity else list(CURVES.values())
    with LOCK:
        chart = solver(name, altitude_m, throttle, pitch, mass)
        mode = dict(key='throttle', label=f'{throttle:g}%', level=bool(level))
        for axis, xs, fixed in (('altitude', altitudes, speed_kmh), ('speed', speeds, altitude_m)):
            if sweep not in (None, axis):continue
            store = points((name, throttle, pitch, mass, bool(level), axis, fixed))
            # Each point is solved once per sweep, however many plots and requests use it.
            if level:
                along = level_solver(name, altitude_m, throttle, pitch, mass) if axis == 'speed' else None
                def solve(x):
                    near = min(((abs(s - x), seed) for s, (r, seed) in store.items() if seed), key=lambda t:t[0], default=(0, None))[1]
                    return (flight(level_solver(name, x, throttle, pitch, mass), x, speed_kmh, near) if axis == 'altitude' else
                            flight(along, altitude_m, x, near))
            else:
                def solve(x):
                    return (row(solver(name, x, throttle, pitch, mass), x, speed_kmh) if axis == 'altitude' else
                            row(chart, altitude_m, x)), None
            def at(x, store=store, solve=solve):
                if x not in store:store[x] = solve(x)
                return store[x][0]
            blank = (lambda h:dict(altitude_m=h, speed_kmh=speed_kmh, solved=False)) if axis == 'altitude' else \
                (lambda v:dict(altitude_m=altitude_m, speed_kmh=v, solved=False))
            base = march(xs, at, (0., 3000., 6000.) if axis == 'altitude' else (400., 300., 500., 250.) if prop else (800., 600., 1000., 450.), blank)
            mode[axis] = refined(base, at, (lambda r:r['altitude_m']) if axis == 'altitude' else (lambda r:r['speed_kmh']),
                                 2. if axis == 'altitude' else .25 if prop else .5, curves)
        modes = [mode]
    engines = chart.engine.properties['engines'] if prop else []
    return dict(aircraft_id=name, name=entry['name'], propulsion=entry['propulsion'], altitude_m=altitude_m,
                speed_kmh=speed_kmh, throttle_percent=throttle, pitch_percent=pitch, modes=modes,
                engine_count=len(engines) if prop else len(chart.engine.units),
                stages=[len(e['properties']['stages']) for e in engines if e['family'] in (0, 1)],
                rpm_limit=([e['properties']['omega_limit'] * 30 / math.pi for e in engines] if prop else
                           [u.ep['omega_limit'] * 30 / math.pi for u in chart.engine.units]),
                elapsed_s=time.perf_counter() - started)


def request(body, report=None):
    """POST /api/engine: {aircraft, sweep: altitude|speed, level, altitude_m, speed_kmh, throttle, pitch: 'auto'|%,
    mass: {fuel_percent, extra_mass_kg, flaps_percent, loadout, stores_released, default_ammunition, ammunition_vehicle}}."""
    known = {'aircraft', 'sweep', 'level', 'altitude_m', 'speed_kmh', 'throttle', 'pitch', 'mass', 'altitude_range', 'speed_range', 'quantity'}
    if set(body) - known:raise ValueError('Unknown field: ' + ', '.join(sorted(set(body) - known)))
    name = body.get('aircraft')
    if not isinstance(name, str) or name not in AIRCRAFT:raise ValueError('Unknown aircraft')
    def number(key, default, low, high):
        v = body.get(key, default)
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or not low <= v <= high:
            raise ValueError(key + ' out of range')
        return v
    altitude, speed, throttle = number('altitude_m', 0., 0., 18000.), number('speed_kmh', 300., 5., 1600.), number('throttle', 110., 0., 110.)
    pitch = body.get('pitch', 'auto');pitch = None if pitch == 'auto' else number('pitch', None, 0., 100.)
    sweep = body.get('sweep', 'both')
    if sweep not in ('altitude', 'speed', 'both'):raise ValueError('Unknown sweep')
    if not isinstance(body.get('level', False), bool):raise ValueError('level must be true or false')
    def span(key, high):
        r = body.get(key)
        if r is None:return None
        if (not isinstance(r, list) or len(r) != 2 or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in r)
                or not 0 <= r[0] < r[1] <= high):raise ValueError(key + ' must be two increasing values from 0 to ' + str(int(high)))
        return float(r[0]), float(r[1])
    altitudes, speeds = span('altitude_range', 18000.), span('speed_range', 1600.)
    quantity = body.get('quantity')
    if quantity is not None and quantity not in CURVES:raise ValueError('Unknown quantity')
    mass = body.get('mass', {})
    if not isinstance(mass, dict) or set(mass) - set(MASS):raise ValueError('Unknown mass setting')
    # settings() checks every value when the solver is built; the text is canonical so equal settings share a result.
    return compute(name, float(altitude), float(speed), float(throttle), None if pitch is None else float(pitch), report,
                   None if sweep == 'both' else sweep, body.get('level', False), json.dumps(mass, sort_keys=True), altitudes, speeds, quantity)
