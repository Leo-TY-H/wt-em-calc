"""Engine and propeller performance for the website's Engine performance page (report only; no solver changes).

The EM plotter's own logic throughout (Leo, 2026-10-28): each point is calculated by a solver built from the same
settings as an EM chart at that altitude (the EM page's throttle slider: setting / 100, boost requested above 100%;
its manual propeller control; every other setting at the EM defaults, fully upgraded as the catalog applies every
modification). Propeller aircraft: the EM plotter's axial-flow propulsion state (em_stationary_baseline.AxialReference:
BaselineSolver.operating_point at 1 g, angle of attack, bank and controls 0), kept when converged and admitted by the
chart's propeller admission. Jets: the chart's engine force (EngineEnsemble.vectors). Fuel flow (Leo, 2026-10-29) is
the game's consumption of each engine in that same state (FUEL-CONSUMPTION-MODEL.md: the piston power-ratio law, the
turbine scalar of jets and turboprops), summed. Sweeps over altitude at one true airspeed or over true airspeed at one
altitude; SFC as the fuel laws' own curves.
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

KGF = 9.80665
# The game engine model's own horsepower, used for every hp on the site (Leo, 2026-10-29): 746 W (piston torque_base =
# base_hp x 746 / max_omega; the fuel law's power factor 0.00134048 = 1/746). Its jet fuel law's kgf is 9.81 N
# (k = ConsumptionOmegaMax / 3600 / 9.81): TSFC uses it so it reads as the FM's coefficients.
GAME_HP = 746.
GAME_KGF = 9.81
LOCK = threading.Lock()


# Default ranges (Leo, 2026-10-28): true airspeed 0-800 km/h for propeller aircraft, 0-1,300 for jets; altitude 0-10 km.
SPEEDS = {'jet': (0., 1300.), 'rocket': (0., 1300.)}
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
        if not row.get('supported'):continue
        # Rocket aircraft by their motors; others by their main engines (not auxiliary rockets).
        try:mains = [e for _, e in engines(load(name)) if (e.get('Main', {}).get('Type') == 'Rocket') == (row['propulsion'] == 'rocket')]
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
    return dict(thrust_kgf=thrust / KGF, power_hp=power / GAME_HP if power > 0. or pistons else None,
                engine_power_hp=[e.get('torque', 0.) * e['omega'] / GAME_HP for e in state['engines']],
                fuel_kgh=sum(fuel), engine_fuel_kgh=fuel,
                piston_power_hp=piston_w / GAME_HP if pistons else None,
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
                power_hp=sum(prop['shaft_power_kw']) * 1000. / GAME_HP if prop.get('shaft_power_kw') else None,
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


# The plotted quantities (Leo, 2026-10-28: power, thrust, efficiency, SEP; RPM, blade pitch and manifold only in details;
# 2026-10-29: fuel flow; SFC, over throttle).
CURVES = {'power':lambda r:r.get('power_hp'), 'thrust':lambda r:r.get('thrust_kgf'), 'efficiency':lambda r:r.get('efficiency'),
          'fuel':lambda r:r.get('fuel_kgh'), 'sep':lambda r:r.get('ps_mps')}


def curves_for(quantity):
    """The curves a sweep's refinement follows: the plot asked for, or all of them (SFC has no sweep)."""
    return [] if quantity == 'sfc' else [CURVES[quantity]] if quantity else list(CURVES.values())


def piston_fuel(p, shaft_hp):
    """piston_general.step's fuel demand (kg/s) of a running piston engine delivering `shaft_hp` (game hp), its own
    float32 operations line for line: the power law takes at least 5% of base_hp and interpolates the FM's four
    FuelConsumptionOn... coefficients (kg/hp/h) in power / base_hp. Nothing else enters, so BSFC is a function of shaft
    power alone, at any altitude and speed."""
    from component_assembly import add, mul, sub
    from piston_model import div
    power = max(mul(p['base_hp'], f32(.05)), f32(shaft_hp))
    x = max(0., div(power, p['base_hp']));y = p['consumption']
    if x <= .5:rate = y[0] if x <= 0 else y[1] if x >= .5 else add(y[0], mul(x, add(sub(y[1], y[0]), sub(y[1], y[0]))))
    elif x <= 1.:rate = y[2] if x >= 1 else add(mul(add(add(x, x), -1.), sub(y[2], y[1])), y[1])
    else:rate = y[3] if x >= f32(1.1) else add(mul(add(mul(x, f32(9.999998092651367)), f32(-9.999998092651367)), sub(y[3], y[2])), y[2])
    return mul(mul(power, f32(.00027777778450399637)), rate)


def bsfc_curves(chart):
    """BSFC (kg/hp/h) against shaft power (hp) of each distinct piston engine of a chart (its upgraded properties), from
    0.2% to 125% of base_hp (past 110% the coefficient holds), with the law's corners at 5%, 50%, 100% and 110%."""
    out = {}
    for i, e in enumerate(chart.engine.properties['engines']):
        if e['family'] not in (0, 1):continue
        p = e['properties'];key = (p['base_hp'], tuple(p['consumption']))
        if key in out:out[key]['engines'].append(i);continue
        xs = sorted({k / 500 for k in range(1, 626)} | {.05, .5, 1., 1.1})
        out[key] = dict(engines=[i], base_hp=p['base_hp'], coefficients=list(p['consumption']),
                        rows=[dict(shaft_hp=x * p['base_hp'], bsfc=3600. * piston_fuel(p, x * p['base_hp']) / (x * p['base_hp'])) for x in xs])
    return list(out.values())


def turbine_tsfc(t, boost, limit, throttle):
    """TSFC (kg/kgf/h, kgf of 9.81 N) of a jet or turboprop turbine at a throttle setting (%), from the game's scalar
    law alone (jet_model.scalar_update, as jet_nominal.evaluate and turbine_general.step run it): the command is the
    throttle over 1.1 when ThrottleBoost > 1 (else 1), at most 1; the gas generator runs at its throttle curve's speed,
    capped at `limit` (rad/s); there fuel = ConsumptionMult x thrust x k, so TSFC = ConsumptionMult x k at any thrust,
    altitude and speed. t: the prepared turbine (jet_model.prepare); boost: the engine's ThrottleBoost."""
    from component_assembly import mul
    from control_mixer import curve
    from jet_model import mode
    from piston_model import div
    command = min(div(min(f32(throttle / 100.), f32(1.1)), f32(1.1 if boost > 1. else 1.)), 1.)
    omega = min(max(mul(curve(t['throttle'], command, 1)[0], t['max_omega']), 0.), limit)
    return mul(mode(t, f32(omega / t['max_omega']))[3], t['consumption']) * 3600. * GAME_KGF


def sfc_curves(name, chart, propulsion):
    """SFC from the game's fuel laws alone, at no flight condition (Leo, 2026-10-29: "simple equations you can plot"):
    bsfc: BSFC against shaft power per distinct piston engine (piston_fuel); tsfc: TSFC against throttle per distinct
    jet or turbine (turbine_tsfc); rocket: fuel flow (kg/s) against throttle per motor (rocket_general.step at the
    motor's MaxThrMult cap, as rocket_em.RocketUnit runs it). Every 0.05% of throttle from 0 to the engine's own stop in
    the game (aces.exe pilot throttle cap, see without_wep: 110% when its ThrottleBoost > 1, else 100%); equal curves of
    several engines are one, with every engine listed."""
    from rocket_general import step
    def throttles(boost):
        top = 110. if boost > 1. else 100.
        return [top * i / int(top / .05) for i in range(int(top / .05) + 1)]
    out = dict(bsfc=bsfc_curves(chart) if propulsion not in ('jet', 'rocket') else [], tsfc=[], rocket=[])
    def add(kind, index, rows):
        same = next((c for c in out[kind] if c['rows'] == rows), None)
        if same:same['engines'].append(index)
        else:out[kind].append(dict(engines=[index], rows=rows))
    if propulsion == 'rocket':
        from aircraft_catalog import load
        from jet_catalog import engines
        caps = [f32(min(1.1, e['Main'].get('MaxThrMult', 1.1))) for _, e in engines(load(name)) if e['Main']['Type'] == 'Rocket']
        for i, (u, cap) in enumerate(zip(chart.engine.units, caps)):
            add('rocket', i, [dict(throttle_percent=x, fuel_kgs=step(u.properties, dict(throttle=min(f32(x / 100.), cap), running=7 if x > 0. else 0))['consumption'])
                              for x in throttles(u.properties['properties']['throttle_boost'])])
    elif propulsion == 'jet':
        for i, u in enumerate(chart.engine.units):
            limit = u.ep['omega_limit']
            add('tsfc', i, [dict(throttle_percent=x, tsfc=turbine_tsfc(u.jet, u.ep['throttle_boost'], limit, x)) for x in throttles(u.ep['throttle_boost'])])
    else:
        for i, e in enumerate(chart.engine.properties['engines']):
            if e['family'] not in (2, 5):continue
            p = e['properties'];limit = p['turbo_allowed'] if e['family'] == 5 else p['omega_limit']
            add('tsfc', i, [dict(throttle_percent=x, tsfc=turbine_tsfc(e['turbine'], p['throttle_boost'], limit, x)) for x in throttles(p['throttle_boost'])])
    return out
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
    if level:
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
    return (lambda h:dict(altitude_m=h, speed_kmh=fixed, solved=False)) if axis == 'altitude' else \
        (lambda v:dict(altitude_m=fixed, speed_kmh=v, solved=False))


def starts(axis, prop):
    """Where a march starts: a point that converges cold."""
    return (0., 3000., 6000.) if axis == 'altitude' else (400., 300., 500., 250.) if prop else (800., 600., 1000., 450.)


PLACE = dict(altitude=lambda r:r['altitude_m'], speed=lambda r:r['speed_kmh'])


@lru_cache(maxsize=32)
def compute(name, altitude_m, speed_kmh, throttle=110., pitch=None, report=None, sweep=None, level=False, mass='{}',
            altitudes=None, speeds=None, quantity=None):
    """The sweeps at a throttle setting (%): sweep 'altitude' (at speed_kmh), 'speed' (at altitude_m), 'sfc' (the fuel
    laws' SFC curves, no flight condition) or None for altitude and speed. pitch: manual propeller control (%) or None
    for automatic control.
    level: SEP in 1 g level flight instead of the engine at angle of attack 0. mass: JSON of the MASS settings (EM
    defaults where left out). altitudes, speeds: the sweep ranges (min, max), defaults SPEEDS / ALTITUDES. quantity: the
    plotted CURVES key the refinement follows (None: all of them)."""
    entry = AIRCRAFT[name]
    if not entry['supported']:raise ValueError(entry['name'] + ': ' + entry['reason'])
    prop = entry['propulsion'] not in ('jet', 'rocket')
    if pitch is not None and not (prop and entry.get('manual_controls', {}).get('propeller_pitch')):
        raise ValueError('Manual propeller pitch is unavailable for ' + entry['name'])
    if throttle > 100. and name in without_wep():raise ValueError(entry['name'] + ' has no WEP: its throttle stops at 100%')
    started = time.perf_counter()
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
        if sweep == 'sfc':mode['sfc'] = sfc_curves(name, chart, entry['propulsion'])
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
    """POST /api/engine: {aircraft, sweep: altitude|speed|sfc, level, altitude_m, speed_kmh, throttle, pitch: 'auto'|%,
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
    if sweep not in ('altitude', 'speed', 'sfc', 'both'):raise ValueError('Unknown sweep')
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
