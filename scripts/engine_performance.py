"""Engine and propeller performance for the website's Engine performance page (report only; no solver changes).

The EM plotter's own logic throughout (Leo, 2026-10-28): each point is calculated by a solver built from the same
settings as an EM chart at that altitude (the EM page's throttle slider: setting / 100, boost requested above 100%;
its manual propeller control; every other setting at the EM defaults, fully upgraded as the catalog applies every
modification). Propeller aircraft: the EM plotter's axial-flow propulsion state (em_stationary_baseline.AxialReference:
BaselineSolver.operating_point at 1 g, angle of attack, bank and controls 0), kept when converged and admitted by the
chart's propeller admission. Jets: the chart's engine force (EngineEnsemble.vectors). Fuel flow (Leo, 2026-10-29) is
the game's consumption of each engine in that same state (FUEL-CONSUMPTION-MODEL.md: the piston power-ratio law, the
turbine scalar of jets and turboprops), summed. Sweeps over altitude at one true airspeed, over true airspeed at one
altitude, or over both (the 3D view).
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
# The game engine model's own units, which its fuel laws use: horsepower = 746 W (piston torque_base = base_hp x 746 /
# max_omega; the fuel law's power factor 0.00134048 = 1/746), kgf = 9.81 N (jets' k = ConsumptionOmegaMax / 3600 / 9.81).
GAME_HP = 746.
GAME_KGF = 9.81
LOCK = threading.Lock()


# Default ranges (Leo, 2026-10-28): true airspeed 0-800 km/h for propeller aircraft, 0-1,300 for jets; altitude 0-10 km.
SPEEDS = {'jet': (0., 1300.), 'rocket': (0., 1300.)}
ALTITUDES = (0., 10000.)


def sweeps(propulsion, altitudes=None, speeds=None, coarse=None):
    """Altitudes (m) and true airspeeds (km/h) of the two sweeps' base grids (Leo, 2026-10-29: high quality): every 50 m
    and 2 km/h for propeller aircraft, 100 m and 5 km/h for jets, at most 800 steps over the range. coarse: (altitude,
    speed) spacing wanted instead; the grid takes every k-th of those points, k chosen for the spacing nearest it, so a
    coarse grid's points are sweep points too. A propeller needs inflow: no speed below 5 km/h."""
    jet = propulsion in ('jet', 'rocket')
    def grid(lo, hi, step, wanted):
        n = min(800, max(1, math.ceil((hi - lo) / step - 1e-9)))
        k = 1 if wanted is None else n // min((d for d in range(1, n + 1) if n % d == 0), key=lambda d:abs(math.log(d * wanted / (hi - lo))))
        return [lo + (hi - lo) * i / n for i in range(0, n + 1, k)]
    altitudes = grid(*(altitudes or ALTITUDES), 100. if jet else 50., coarse and coarse[0])
    speeds = grid(*(speeds or SPEEDS.get(propulsion, (0., 800.))), 5. if jet else 2., coarse and coarse[1])
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
    """The EM chart's solver for these settings (built per altitude: the catalog caches the aircraft, so this is cheap),
    with the compiled evaluator (em_march_native: the same values; the axial points take a third less time)."""
    import em_march_native
    from em_stationary_baseline import BaselineSolver
    extra = dict(json.loads(mass), altitude_m=float(altitude), throttle=throttle / 100., afterburner=throttle > 100.)
    if pitch is not None:extra.update(manual_engine_controls=True, propeller_pitch_auto=False, propeller_pitch_percent=pitch)
    chart = BaselineSolver(name, settings(dict(aircraft=[name], sideslip_recovery=False, **extra)))
    em_march_native.attach(chart)
    return chart


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
    fuel = [3600. * e['consumption'] for e in state['engines']]   # kg/h; mixed aircraft include their jets
    # SFC (Leo, 2026-10-29), in the game's units: pistons' fuel per their shaft power (the torque x omega the fuel law
    # takes; kg per GAME_HP hour), turbines' fuel per the turbine scalar thrust it multiplies (kg per GAME_KGF hour),
    # from turbine_scalar, kept only where that re-run reproduces the solver's own fuel to the bit.
    families = [e['family'] for e in p['engines']];turbines = [i for i, f in enumerate(families) if f in (2, 5)]
    piston_w = sum(e.get('torque', 0.) * e['omega'] for e, f in zip(state['engines'], families) if f in (0, 1))
    scalars = [turbine_scalar(p['engines'][i], state['engines'][i], velocity, chart.config['altitude_m'], chart.mass['nitro_mass'])
               for i in turbines]
    turbine_n = sum(x['thrust'] for x in scalars)
    exact = all(x['consumption'] == state['engines'][i]['consumption'] for x, i in zip(scalars, turbines))
    return dict(thrust_kgf=thrust / KGF, power_hp=power / METRIC_HP if power > 0. or pistons else None,
                engine_power_hp=[e.get('torque', 0.) * e['omega'] / METRIC_HP for e in state['engines']],
                fuel_kgh=sum(fuel), engine_fuel_kgh=fuel,
                piston_power_game_hp=piston_w / GAME_HP if pistons else None,
                bsfc=sum(fuel[i] for i in pistons) / (piston_w / GAME_HP) if pistons and piston_w > 0. else None,
                turbine_thrust_kgf=turbine_n / KGF if turbines and exact else None,
                tsfc=sum(fuel[i] for i in turbines) / (turbine_n / GAME_KGF) if turbines and exact and turbine_n > 0. else None,
                efficiency=thrust * v / power * 100. if power > 1. and not any(e['family'] == 2 for e in p['engines']) else None,
                rpm=point['propulsion']['engine_rpm'], pitch_deg=point['propulsion']['propeller_pitch_deg'],
                pitch_percent=point['propulsion'].get('propeller_pitch_percent'),
                manifold_ata=[float(state['engines'][i].get('manifold', math.nan)) for i in pistons],
                gear=[int(state['engines'][i].get('gear', 0)) + 1 for i in pistons],
                governor=list(prop['stationarity'].get('governor_status') or ()), tip_mach=tips,
                mach=float(air['mach']), ias_kmh=float(air['ias_u']) * 3.6)


def turbine_scalar(engine, s, velocity, height, nitro):
    """A solved jet or turboprop engine's turbine scalar (thrust, consumption): turbine_general.step's own
    scalar_update call at the state's inputs, with dt 0 as prop_quasisteady.engine_output evaluates it (the gas
    generator holds its speed). Its thrust is the one the game's fuel law multiplies: fuel = ConsumptionMult x thrust
    x ConsumptionOmegaMax / 3600 / 9.81."""
    from component_assembly import mul
    from control_mixer import density_at_height
    from engine_supply import mechanical_multiplier
    from jet_model import scalar_update
    from piston_model import boost_active, div
    p = engine['properties'];omega = f32(s['omega']);dt = f32(0.)
    throttle = min(f32(s.get('throttle', 1.)), f32(1.1))
    health = mechanical_multiplier(p, omega, 1., p['cylinders'], s.get('mechanical', 1.), 0., s.get('torque', 0.), s.get('friction', 0.),
                                   dt, 12345)[0]
    gas = f32(s.get('turbo', 0.)) if p['family'] == 5 else omega
    return scalar_update(engine['turbine'], density_at_height(f32(height)), velocity[0], gas,
                         min(div(throttle, f32(1.1 if p['throttle_boost'] > 1. else 1.)), 1.), dt,
                         afterburner=boost_active(p, throttle, s.get('afterburner', False), s.get('gear', 0), nitro), health=health,
                         shaft_fraction=mul(omega, p['inverse_omega']), running=2)


def jet_row(chart, speed_kmh):
    from air_state import cache
    v = speed_kmh / 3.6
    air = cache([f32(v), 0., 0.], chart.config['altitude_m'])
    # The chart's nominal engine state (jet_nominal.evaluate): RPM, the scalar thrust and the fuel demand it sets, kg/s.
    nominal = [u.nominal(v) for u in chart.engine.units];fuel = [3600. * n['scalar']['consumption'] for n in nominal]
    engine = sum(n['scalar']['thrust'] for n in nominal)
    return dict(thrust_kgf=float(chart.engine.vectors(v)[0][0]) / KGF, rpm=[n['omega'] * 30 / math.pi for n in nominal],
                fuel_kgh=sum(fuel), engine_fuel_kgh=fuel, turbine_thrust_kgf=engine / KGF,
                tsfc=sum(fuel) / (engine / GAME_KGF) if engine > 0. else None, mach=float(air['mach']), ias_kmh=float(air['ias_u']) * 3.6)


def rocket_row(chart, speed_kmh):
    """A rocket aircraft's motors (rocket_em.RocketUnit: the scalar rocket law at the chart's throttle; no RPM)."""
    from air_state import cache
    v = speed_kmh / 3.6
    air = cache([f32(v), 0., 0.], chart.config['altitude_m'])
    units = chart.engine.units;fuel = [3600. * u.consumption for u in units];engine = sum(u.thrust for u in units)
    return dict(thrust_kgf=float(chart.engine.vectors(v)[0][0]) / KGF, fuel_kgh=sum(fuel), engine_fuel_kgh=fuel,
                turbine_thrust_kgf=engine / KGF, tsfc=sum(fuel) / (engine / GAME_KGF) if engine > 0. else None,
                mach=float(air['mach']), ias_kmh=float(air['ias_u']) * 3.6)


def level_solver(name, altitude, throttle, pitch, mass='{}'):
    """The EM plotter's point solver (em_server.point_solver for a march chart): soft admission, compiled loop."""
    chart = solver(name, altitude, throttle, pitch, mass)
    chart.soft_admission = True
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
    calculations). The 3D view's grid lines are sweeps too. The oldest sweeps go beyond 256 sweeps or 30,000 points."""
    store = POINTS.setdefault(key, {});POINTS.move_to_end(key)
    while len(POINTS) > 1 and (len(POINTS) > 256 or sum(map(len, POINTS.values())) > 30000):POINTS.popitem(last=False)
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


# The plotted quantities (Leo, 2026-10-28: power, thrust, efficiency, SEP; RPM, blade pitch and manifold only in details;
# 2026-10-29: fuel flow; SFC, over throttle).
CURVES = {'power':lambda r:r.get('power_hp'), 'thrust':lambda r:r.get('thrust_kgf'), 'efficiency':lambda r:r.get('efficiency'),
          'fuel':lambda r:r.get('fuel_kgh'), 'sep':lambda r:r.get('ps_mps')}
SFC = [lambda r:r.get('bsfc'), lambda r:r.get('tsfc')]   # an aircraft with pistons and jets has both


def curves_for(quantity):
    """The curves a sweep's refinement follows: the plot asked for, or all of them."""
    return SFC if quantity == 'sfc' else [CURVES[quantity]] if quantity else list(CURVES.values())
TOLERANCE = .0005  # of each plotted quantity's range over the sweep: the largest straight-line midpoint error kept
EXTRA = 1500       # most refinement points added to one sweep


def scales(rows, curves, tolerance):
    """The largest midpoint error kept for each curve: `tolerance` of its range over the rows. A flat quantity varies
    only by float32 noise: its scale is at least 1% of its size."""
    values = [[f(r) for r in rows] for f in curves]
    return [tolerance * max(max(f) - min(f), .01 * max(map(abs, f))) if len(f := [v for v in vs if v is not None and math.isfinite(v)]) > 1
            else 0. for vs in values]


def refined(rows, at, x, step, curves, scale=None, budget=None):
    """The sweep with points added where a straight line between neighbours misrepresents the plotted curves (Leo,
    2026-10-28: the fixed grid cut corners; 2026-10-29: only for the plot asked for). An interval is split where one end
    is solved and the other not, or where the base grid bends there (second difference above the tolerance); a split
    keeps halving while the midpoint is off the chord or the solved state changes inside, down to `step`. scale: each
    curve's tolerance (default scales(rows, curves, TOLERANCE)); budget: [points left] when shared between sweeps."""
    def kind(r):return r['solved']
    if scale is None:scale = scales(rows, curves, TOLERANCE)
    def bent(a, m, b):
        for f, s in zip(curves, scale):
            ya, ym, yb = f(a), f(m), f(b)
            if s > 0. and None not in (ya, ym, yb) and abs(ym - .5 * (ya + yb)) > s:return True
        return False
    def candidate(i):
        if kind(rows[i]) != kind(rows[i + 1]):return True
        return any(0 < j < len(rows) - 1 and bent(rows[j - 1], rows[j], rows[j + 1]) for j in (i, i + 1))
    if budget is None:budget = [EXTRA]
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


def axial_point(chart, height, speed, propulsion, report):
    """An engine sweep row: the axial state at angle of attack 0, unsolved where the EM plotter's reference is."""
    try:value = (jet_row(chart, speed) if propulsion == 'jet' else rocket_row(chart, speed) if propulsion == 'rocket' else
                 propeller_row(chart, speed, report))
    except (ValueError, ArithmeticError, np.linalg.LinAlgError):value = None
    return dict(value or {}, altitude_m=height, speed_kmh=speed, solved=value is not None)


def level_point(chart, height, speed, near):
    """A level-flight sweep row and its continuation seed."""
    try:value, seed = level_row(chart, speed, near)
    except (ArithmeticError, np.linalg.LinAlgError):value = seed = None
    return dict(value or {}, altitude_m=height, speed_kmh=speed, solved=value is not None), seed


def sweep_at(spec, axis, fixed, store):
    """at(x) of one sweep: over altitude at speed `fixed`, or over speed at altitude `fixed`. Each point is solved once and
    kept in `store` {x: (row, seed)}, however many plots and requests use it. spec: (name, throttle, pitch, mass, level,
    report)."""
    name, throttle, pitch, mass, level, report = spec
    propulsion, charts = AIRCRAFT[name]['propulsion'], []
    def along():   # the speed sweep's chart, built at its first unsolved point
        if not charts:charts.append((level_solver if level else solver)(name, fixed, throttle, pitch, mass))
        return charts[0]
    if axis == 'throttle':   # at altitude and true airspeed `fixed`, one chart per throttle setting (%)
        def solve(x):
            return dict(axial_point(solver(name, fixed[0], x, pitch, mass), *fixed, propulsion, report), throttle_percent=x), None
    elif level:
        def solve(x):
            near = min(((abs(s - x), seed) for s, (r, seed) in store.items() if seed), key=lambda t:t[0], default=(0, None))[1]
            return (level_point(level_solver(name, x, throttle, pitch, mass), x, fixed, near) if axis == 'altitude' else
                    level_point(along(), fixed, x, near))
    else:
        def solve(x):
            return (axial_point(solver(name, x, throttle, pitch, mass), x, fixed, propulsion, report) if axis == 'altitude' else
                    axial_point(along(), fixed, x, propulsion, report)), None
    def at(x):
        if x not in store:store[x] = solve(x)
        return store[x][0]
    return at


def blank(axis, fixed):
    """An unsolved row of a sweep (past a march's give-up)."""
    if axis == 'throttle':return lambda t:dict(altitude_m=fixed[0], speed_kmh=fixed[1], throttle_percent=t, solved=False)
    return (lambda h:dict(altitude_m=h, speed_kmh=fixed, solved=False)) if axis == 'altitude' else \
        (lambda v:dict(altitude_m=fixed, speed_kmh=v, solved=False))


def starts(axis, prop):
    """Where a march starts: a point that converges cold."""
    if axis == 'throttle':return (100., 80., 50.)
    return (0., 3000., 6000.) if axis == 'altitude' else (400., 300., 500., 250.) if prop else (800., 600., 1000., 450.)


PLACE = dict(altitude=lambda r:r['altitude_m'], speed=lambda r:r['speed_kmh'], throttle=lambda r:r['throttle_percent'])


# The 3D view (Leo, 2026-10-29: f(speed, altitude)). A point costs 6-12 ms (30 ms for four engines), so the grid is
# coarse where the plotted quantity is smooth and fine only where it bends or a solved edge crosses.
SURFACE_STEPS = 250., 40.      # base grid spacing wanted: altitude (m), true airspeed (km/h); speed curves are the smoother
SURFACE_TOLERANCE = .002       # of the plotted quantity's range over the base grid
SURFACE_SPLITS = 3             # halvings of a base grid step at most (31 m, 5 km/h)
LINE_EXTRA = 150               # most refinement points added to one grid line


def surface_line(task):
    """One grid line of the 3D view (Leo, 2026-10-29: speed it up): march its base grid (a row: xs given) or refine it
    (base and scale given). Runs on the EM plotter's worker processes, or here. task: (spec, axis, fixed, known points,
    xs, base, scale, step, quantity); spec's report True means em_server.pitch_percent. Returns (points solved here,
    base, refined line)."""
    spec, axis, fixed, known, xs, base, scale, step, quantity = task
    if spec[5] is True:
        from em_server import pitch_percent
        spec = spec[:5] + (pitch_percent,)
    store = dict(known);at = sweep_at(spec, axis, fixed, store)
    if base is None:base = march(xs, at, starts(axis, AIRCRAFT[spec[0]]['propulsion'] not in ('jet', 'rocket')), blank(axis, fixed))
    line = None if scale is None else refined(base, at, PLACE[axis], step, curves_for(quantity), scale, [LINE_EXTRA])
    return {x:v for x, v in store.items() if x not in known}, base, line


def lines(tasks, parallel):
    """surface_line of every task, in order: on the EM plotter's worker processes (em_workers, as its chart columns)
    when there are several and `parallel` (a jet's engine points take well under a millisecond: here), else here.
    Every line is the same calculation either way."""
    from em_workers import WORKERS, process_pool
    if not parallel or WORKERS < 2 or len(tasks) < 2:return [surface_line(t) for t in tasks]
    shipped = [(t[0][:5] + (True if t[0][5] is not None else None,),) + t[1:] for t in tasks]
    with process_pool() as (pool, cancel):
        return [f.result() for f in [pool.submit(surface_line, t) for t in shipped]]


def surface(propulsion, spec, ranges, quantity):
    """The 3D view's points, triangles and grid lines. Rows (speed sweeps about every SURFACE_STEPS[0]) march as speed
    sweeps do; columns (altitude sweeps about every SURFACE_STEPS[1]) share the rows' points. Every row and column is
    then refined as a sweep is, with one tolerance for all of them; a refinement point lies on its grid line and is that
    sweep's point, kept for later plots. The triangulation (Delaunay over the range scaled to a unit square) takes every
    point, solved or not, so a triangle with an unsolved corner, and only that one, is left out: gaps stay open."""
    from scipy.spatial import Delaunay, QhullError
    heights, speeds = sweeps(propulsion, *ranges, SURFACE_STEPS)
    parallel = spec[4] or propulsion not in ('jet', 'rocket')
    step = [(xs[-1] - xs[-2]) / 2 ** SURFACE_SPLITS if len(xs) > 1 else math.inf for xs in (heights, speeds)]
    rows = {h:points(spec[:5] + ('speed', h)) for h in heights}
    base = {}
    for h, (new, line, _) in zip(heights, lines([(spec, 'speed', h, rows[h], speeds, None, None, None, quantity) for h in heights], parallel)):
        rows[h].update(new);base[h] = line
    columns = {v:points(spec[:5] + ('altitude', v)) for v in speeds}
    for v in speeds:
        for h in heights:
            if v in rows[h]:columns[v].setdefault(h, rows[h][v])
    scale = scales([r for h in heights for r in base[h]], curves_for(quantity), SURFACE_TOLERANCE)
    tasks = ([(spec, 'speed', h, rows[h], None, base[h], scale, step[1], quantity) for h in heights] +
             [(spec, 'altitude', v, columns[v], None, [base[h][j] for h in heights], scale, step[0], quantity) for j, v in enumerate(speeds)])
    out = {}
    for (_, axis, fixed, *_), (new, _, line) in zip(tasks, lines(tasks, parallel)):
        (rows if axis == 'speed' else columns)[fixed].update(new)
        for r in line:out.setdefault((r['altitude_m'], r['speed_kmh']), r)
    keys = sorted(out);vertices = [out[k] for k in keys]
    (h0, h1), (v0, v1) = ranges
    try:corners = Delaunay(np.array([[(v - v0) / (v1 - v0), (h - h0) / (h1 - h0)] for h, v in keys])).simplices.tolist()
    except (QhullError, ValueError):corners = []   # one row or one column: nothing to span
    return vertices, [t for t in corners if all(vertices[i]['solved'] for i in t)], dict(altitudes=heights, speeds=speeds)


@lru_cache(maxsize=32)
def compute(name, altitude_m, speed_kmh, throttle=110., pitch=None, report=None, sweep=None, level=False, mass='{}',
            altitudes=None, speeds=None, quantity=None):
    """The sweeps at a throttle setting (%): sweep 'altitude' (at speed_kmh), 'speed' (at altitude_m), 'surface' (over
    both ranges), 'throttle' (every setting, at altitude_m and speed_kmh) or None for altitude and speed. pitch: manual propeller control (%) or None for automatic control.
    level: SEP in 1 g level flight instead of the engine at angle of attack 0. mass: JSON of the MASS settings (EM
    defaults where left out). altitudes, speeds: the sweep ranges (min, max), defaults SPEEDS / ALTITUDES. quantity: the
    plotted CURVES key the refinement follows (None: all of them)."""
    entry = AIRCRAFT[name]
    if not entry['supported']:raise ValueError(entry['name'] + ': ' + entry['reason'])
    prop = entry['propulsion'] not in ('jet', 'rocket')
    if sweep == 'throttle' and level:raise ValueError('The throttle sweep is the engine at angle of attack 0')
    if pitch is not None and not (prop and entry.get('manual_controls', {}).get('propeller_pitch')):
        raise ValueError('Manual propeller pitch is unavailable for ' + entry['name'])
    if throttle > 100. and name in without_wep():raise ValueError(entry['name'] + ' has no WEP: its throttle stops at 100%')
    started = time.perf_counter()
    ranges = altitudes or ALTITUDES, speeds or SPEEDS.get(entry['propulsion'], (0., 800.))
    altitudes, speeds = sweeps(entry['propulsion'], altitudes, speeds)
    curves = curves_for(quantity)
    spec = (name, throttle, pitch, mass, bool(level), report)
    with LOCK:
        chart = solver(name, altitude_m, throttle, pitch, mass)
        mode = dict(key='throttle', label=f'{throttle:g}%', level=bool(level))
        for axis, xs, fixed in (('altitude', altitudes, speed_kmh), ('speed', speeds, altitude_m)):
            if sweep not in (None, axis):continue
            at = sweep_at(spec, axis, fixed, points(spec[:5] + (axis, fixed)))
            base = march(xs, at, starts(axis, prop), blank(axis, fixed))
            mode[axis] = refined(base, at, PLACE[axis], 2. if axis == 'altitude' else .25 if prop else .5, curves)
        if sweep == 'surface':mode['surface'], mode['triangles'], mode['grid'] = surface(entry['propulsion'], spec, ranges, quantity)
        if sweep == 'throttle':
            # SFC (Leo, 2026-10-29): every throttle setting from 0 to the game's cap, 0.5% apart, refined to 0.05%; the
            # entry's own throttle plays no part.
            fixed, top = (altitude_m, speed_kmh), 100. if name in without_wep() else 110.
            tspec = (name, None, pitch, mass, False, report);n = int(top / .5)
            at = sweep_at(tspec, 'throttle', fixed, points(tspec[:5] + ('throttle', fixed)))
            base = march([top * i / n for i in range(n + 1)], at, starts('throttle', prop), blank('throttle', fixed))
            mode['throttle'] = refined(base, at, PLACE['throttle'], .05, curves)
        modes = [mode]
    engines = chart.engine.properties['engines'] if prop else []
    return dict(aircraft_id=name, name=entry['name'], propulsion=entry['propulsion'], altitude_m=altitude_m,
                speed_kmh=speed_kmh, throttle_percent=throttle, pitch_percent=pitch, modes=modes,
                engine_count=len(engines) if prop else len(chart.engine.units),
                stages=[len(e['properties']['stages']) for e in engines if e['family'] in (0, 1)],
                rpm_limit=([e['properties']['omega_limit'] * 30 / math.pi for e in engines] if prop else
                           [u.ep['omega_limit'] * 30 / math.pi for u in chart.engine.units] if entry['propulsion'] == 'jet' else []),
                elapsed_s=time.perf_counter() - started)


def request(body, report=None):
    """POST /api/engine: {aircraft, sweep: altitude|speed|surface|throttle, level, altitude_m, speed_kmh, throttle, pitch: 'auto'|%,
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
    if sweep not in ('altitude', 'speed', 'surface', 'throttle', 'both'):raise ValueError('Unknown sweep')
    if not isinstance(body.get('level', False), bool):raise ValueError('level must be true or false')
    def span(key, high):
        r = body.get(key)
        if r is None:return None
        if (not isinstance(r, list) or len(r) != 2 or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in r)
                or not 0 <= r[0] < r[1] <= high):raise ValueError(key + ' must be two increasing values from 0 to ' + str(int(high)))
        return float(r[0]), float(r[1])
    altitudes, speeds = span('altitude_range', 18000.), span('speed_range', 1600.)
    quantity = body.get('quantity')
    if quantity is not None and quantity not in CURVES and quantity != 'sfc':raise ValueError('Unknown quantity')
    mass = body.get('mass', {})
    if not isinstance(mass, dict) or set(mass) - set(MASS):raise ValueError('Unknown mass setting')
    # settings() checks every value when the solver is built; the text is canonical so equal settings share a result.
    return compute(name, float(altitude), float(speed), float(throttle), None if pitch is None else float(pitch), report,
                   None if sweep == 'both' else sweep, body.get('level', False), json.dumps(mass, sort_keys=True), altitudes, speeds, quantity)
