"""Engine and propeller performance for the website's Engine performance page (report only; no solver changes).

The EM plotter's own logic throughout (Leo, 2026-10-28): each point is calculated by a solver built from the same
settings as an EM chart at that altitude (the EM page's throttle slider: setting / 100, boost requested above 100%;
its manual propeller control; every other setting at the EM defaults, fully upgraded as the catalog applies every
modification), except that level-flight SEP runs with the Instructor off, so it ends at the real stall (Leo,
2026-11-15). Propeller aircraft: the EM plotter's axial-flow propulsion state (em_stationary_baseline.AxialReference:
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
from bisect import bisect_left, bisect_right, insort
from collections import OrderedDict
from functools import lru_cache

import numpy as np

from component_assembly import f32
from em_solver import AIRCRAFT, MAX_ALTITUDE_M, settings
from em_speed_limits import speed_limits, sweep_excluded_kmh

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


# The EM page's mass settings the popup carries (Leo, 2026-10-28), as JSON text so results can be cached; with them the
# 'Booster on' setting (rocket boosters lit, Leo 2026-11-15), which goes with the loadout that fits the boosters.
MASS = ('fuel_percent', 'extra_mass_kg', 'flaps_percent', 'loadout', 'stores_released', 'default_ammunition', 'ammunition_vehicle', 'boosters',
        'booster_fuel_percent')


def solver(name, altitude, throttle, pitch, mass='{}', instructor=True):
    """The EM chart's solver for these settings (built per altitude: the catalog caches the aircraft, so this is cheap),
    with the compiled evaluator (em_march_native: the same values; the axial points take a third less time).
    instructor False: the EM page's Realistic control mode (autotrim, no Instructor AoA cap)."""
    import em_march_native
    from em_stationary_baseline import BaselineSolver
    extra = dict(json.loads(mass), throttle=throttle / 100., afterburner=throttle > 100.)
    if not instructor:extra.update(instructor=False, autotrim=True)
    if pitch is not None:extra.update(manual_engine_controls=True, propeller_pitch_auto=False, propeller_pitch_percent=pitch)
    if not math.isfinite(altitude):raise ValueError('Altitude must be finite')
    config = settings(dict(aircraft=[name], sideslip_recovery=False, **extra))
    # Engine plots have an independent altitude range from the EM form.
    config['altitude_m'] = min(MAX_ALTITUDE_M, max(0., float(altitude)))
    chart = BaselineSolver(name, config)
    if altitude != chart.config['altitude_m']:
        chart.config['altitude_m'] = chart.settings_echo['altitude_m'] = float(altitude)
        if chart.is_prop:
            chart.engine = type(chart.engine)(chart.name, chart.model, chart.mass, chart.config)
        else:
            chart.engine = type(chart.engine)(chart.model, chart.mass, chart.config)
    em_march_native.attach(chart)
    return chart


# The mixture lever's top and step (aces.exe mixture axis 0x141604EE0: each engine's command is max(0, min(1.2,
# setting)), 0x14714D9CC, held in steps of 0.005, 0x14714E8C8; WT-FM-MOUSE-AIM scripts/native_engine_axis_handlers.py).
MIXTURE_LEVER_MAX, MIXTURE_LEVER_STEP = 1.2, .005


def mixture_band(ep, lever, body_u, height):
    """Where the mixture lever gives full power (Leo, 2026-11-08), from the game's mixture law (piston_model.mixture) at
    this inlet pressure P (ambient plus ram on the forward speed, before the supercharger, so neither throttle nor gear
    enters). The engine's supply is lever x AltitudePressureToP0: rich of P, power falls as P / supply; lean of P it stays
    full while supply > P / 4, and the engine stops below 0.03. So with b = P / AltitudePressureToP0, type 2 is at full
    power for a lever in (max(b / 4, 0.03 / scale), b]. Type 1 reads the lever as at least 100%: full power up to 100%,
    and above it on the same interval, which can leave a gap above 100% (a small AltitudePressureToP0, such as the
    Mosquito's 0.118). Returns the full-power settings as (lean, rich) of the piece holding the lowest setting, the power
    at 100% (percent of full), and (lean, rich) of a piece above a gap or None; or None without a lever (Mixer
    IsControllable false, or type 0) or with no full-power setting."""
    from piston_model import inlet_pressure, mixture
    if not lever or ep['mixer_type'] not in (1, 2):return None
    inlet = inlet_pressure(height, body_u, ep['ram_recovery'])
    scale = float(ep['mixer_scale'])
    balanced, top = float(inlet) / scale, MIXTURE_LEVER_MAX
    full = (max(.25 * balanced, .03 / scale), min(balanced, top))
    if ep['mixer_type'] == 1:
        # A gap above 100% narrower than one lever step holds no setting the game can take.
        full = (max(full[0], 1.), full[1])
        pieces = [(0., full[1])] if full[0] < 1. + MIXTURE_LEVER_STEP and full[1] > 1. else [(0., 1.)] + ([full] if full[0] < full[1] else [])
    else:
        pieces = [full] if full[0] < full[1] else []
    if not pieces:return None
    percent = lambda piece:(piece[0] * 100., piece[1] * 100.)
    return (*percent(pieces[0]), float(mixture(ep, inlet, 1.)['multiplier']) * 100., percent(pieces[1]) if len(pieces) > 1 else None)


def mixture_levers(chart):
    """Whether each engine has a mixture lever (the FM's Mixer IsControllable), in engine order."""
    from jet_catalog import engines
    return [bool(e.get('Mixer', {}).get('IsControllable')) for _, e in engines(chart.fm)]


def polar_branch(p, a):
    """The branch of the game's lift and drag polar (polar_model.calc_cl, calc_cd) an angle of attack a (degrees) is on,
    as one number: 0 below the critical angles (the line and the parabola up to them join smoothly), then the pieces
    past them (to AfterCritParabAngle, to AfterCritMaxDistanceAngle, to 40, 90, 140 degrees, beyond), negative on the
    low side; plus a half where the drag is on its cap."""
    side = 1 if a >= 0 else -1
    linear = p['cl0'] + p['clLineCoeff'] * a
    capped = (p['cd0'] + linear * linear * p['indCoeff'] + p['cdAfterCoeff'] * max(side * (a - p['aoaCritH' if side > 0 else 'aoaCritL']), 0)
              > .15 + p['cyCritH'] * abs(math.sin(math.radians(a))))
    branch = 0
    if not p['aoaLineL'] <= a <= p['aoaLineH']:
        s = 1 if a - p['aoaLineH'] + .01 >= 0 else -1
        crit = p['aoaCritH' if s > 0 else 'aoaCritL']
        if s * (a - crit) > 0:
            top = max(40, p['maxDistAng'])
            if s * a > 140:branch = 6 * s
            elif s * a > top:branch = (5 if s * a > 90 else 4) * s
            elif s * a > p['maxDistAng']:branch = 3 * s
            else:branch = (1 if s * (a - crit) < p['parabAngle'] else 2) * s
    return branch + .5 * capped


# Whether a surface's pieces also end where a blade section changes polar branch. Off: the many section lines cross
# each other and the supercharger's, and rows either side of a crossing have different layouts, which engine_surface
# cannot blend yet (it halves such intervals down to neighbouring metres): the corners stay inside pieces and cost rows.
BLADE_REGIMES = False


def blade_regime(chart, state, velocity, rates):
    """Every blade section's polar branch in a solved propeller state (the sections and flow of
    prop_quasisteady.blade_forces): thrust and torque have a corner where a section passes its critical angle (the
    drag's after-critical term starts there), strongest at part throttle, where the blades run stalled at low speed."""
    if not BLADE_REGIMES:return ()
    from polar_model import evaluate_polar
    from prop_quasisteady import propeller_condition
    from propulsion_general import fixed_air
    p, out = chart.engine.properties, []
    try:
        _, sound = fixed_air(velocity, chart.config['altitude_m'])
        for i, shaft in enumerate(p['transmissions']):
            for link in shaft['propellers']:
                pp = p['propellers'][link['index']]['properties'];rotor = state['propellers'][link['index']]
                local, _, _, transverse, _, _, _, prepared = propeller_condition(pp, velocity, rates)
                sections, _, polar_data, polar, factors, legacy = prepared
                omega, flow, pitch = state['transmissions'][i]['omega'] * link['ratio'], rotor['flow'], rotor['pitch']
                axial = local[0] + .5 * (flow[0] + flow[1])
                for angular in ((.25 * flow[2], -.25 * flow[2]) if pp['coaxial'] else (.25 * flow[2],)):
                    for radius, twist, width in sections:
                        if width == 0.:continue
                        tangential = radius * (omega - angular)
                        speed = math.hypot(tangential + transverse, axial)
                        if speed == 0.:
                            out.append(0)
                            continue
                        alpha = math.degrees(pitch - (math.atan2(axial, tangential) if abs(omega) >= 1e-5 else math.pi / 2) + twist)
                        evaluate_polar(polar_data, speed / sound, polar, factors, False)
                        if legacy is not None:polar.update(legacy)
                        out.append(polar_branch(polar, alpha))
    except (ValueError, ArithmeticError, KeyError):return ()
    return tuple(out)


def regime(engines, states, governors, blades=(), fuel=True):
    """A solved propulsion state's regime, for the slice surface's pieces (engine_surface): where it changes, a curve
    over speed or altitude has a corner or a step, so a piece of one regime is smooth. Per piston engine (the game's
    supercharger law, piston_model.compressor): the gear, the branch of the gear's power shape the inlet pressure is on
    (up to the critical pressure, up to the flat-rated one, above it) and whether the regulator holds the manifold
    pressure back (below the critical altitude); the inlet pressure is read back from the state's potential manifold
    pressure, so it is the one the solver used; and the branch of the fuel law the shaft power is on (piston_fuel: its
    5% floor and the corners at 50%, 100% and 110% of the engine's base power). Per governor: its state (holding RPM,
    on a pitch stop). blades: `blade_regime`. fuel False: without the fuel law's branch (SEP does not turn with it)."""
    out = []
    for e, s in zip(engines, states):
        p, gear = e['properties'], int(s.get('gear', 0))
        if 'regime' in s:
            # The compiled engine model's own branches (march_native prop.pxi, EngState.regime): every one its law has.
            out.append((gear, s['regime'], s['fuel_regime'] if fuel else 0))
            continue
        if e['family'] not in (0, 1) or 'potential_manifold' not in s or not p.get('stages'):
            out.append((gear,))
            continue
        stage = p['stages'][min(gear, len(p['stages']) - 1)];critical, flat = stage['critical'], stage['flat']
        speed = p['inverse_omega'] * s['omega']
        speed = ((speed * speed - speed) * p['compressor_sq'] + speed) * (1. - p['compressor_zero']) + p['compressor_zero']
        flow = s['potential_manifold'] * p['compressor_reference'] * critical / (speed * p['max_ata']) if speed > 0. else 0.
        branch = 0 if flow <= critical else 1 if critical >= flat - .0001 else 2 if flow <= flat else 3
        load = s.get('torque', 0.) * s['omega'] / GAME_HP / p['base_hp'] if p.get('base_hp') else 0.
        out.append((gear, branch, s.get('regulator', 1.) < 1., (0 if load <= .05 else 1 if load <= .5 else 2 if load <= 1. else 3 if load < 1.1 else 4) if fuel else 0))
    return tuple(out), tuple(governors or ()), blades


def _propeller_fields(chart, v, prop, report, levers=None):
    from em_operating import flight_condition
    from prop_admission import evaluate as admission
    from prop_quasisteady import propeller_condition
    from propulsion_general import fixed_air
    if not prop or not prop.get('converged'):return None
    p, state = chart.engine.properties, prop['state']
    # Over-rev states are not results (Leo, 2026-10-29): an engine above RPMMaxAllowed, the chart's 'engine RPM limit' test.
    if any(e['omega'] > p['engines'][i]['properties']['omega_limit'] * (1 + 1e-6) for i, e in enumerate(state['engines'])):
        return None
    attitude = [0.] * 5
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
    # Lit rocket boosters (family 3): in the total thrust, not in the propeller efficiency or the engines' fuel.
    rockets = [i for i, e in enumerate(p['engines']) if e['family'] == 3]
    booster = sum(float(state['engines'][i].get('force', (0., 0., 0.))[0]) for i in rockets)
    # SFC (Leo, 2026-10-29), in the game's units: pistons' fuel per their shaft power (the torque x omega the fuel law
    # takes; kg per GAME_HP hour), turbines' fuel per the turbine scalar thrust it multiplies (kg per GAME_KGF hour),
    # from turbine_scalar, kept only where that re-run reproduces the solver's own fuel to the bit.
    families = [e['family'] for e in p['engines']];turbines = [i for i, f in enumerate(families) if f in (2, 5)]
    piston_w = sum(e.get('torque', 0.) * e['omega'] for e, f in zip(state['engines'], families) if f in (0, 1))
    scalars = [turbine_scalar(p['engines'][i], state['engines'][i], velocity, chart.config['altitude_m'], chart.mass['nitro_mass'])
               for i in turbines]
    turbine_n = sum(x['thrust'] for x in scalars)
    exact = all(x['consumption'] == state['engines'][i]['consumption'] for x, i in zip(scalars, turbines))
    if levers is None:levers = mixture_levers(chart)
    mixtures = [mixture_band(p['engines'][i]['properties'], levers[i] if i < len(levers) else False, velocity[0], chart.config['altitude_m'])
                for i in pistons]
    return dict(thrust_kgf=thrust / KGF, power_hp=power / GAME_HP if power > 0. or pistons else None,
                engine_power_hp=[e.get('torque', 0.) * e['omega'] / GAME_HP for e in state['engines']],
                fuel_kgh=sum(f for i, f in enumerate(fuel) if i not in rockets), engine_fuel_kgh=fuel,
                **(dict(booster_thrust_kgf=booster / KGF, booster_fuel_kgh=sum(fuel[i] for i in rockets)) if booster else {}),
                piston_power_hp=piston_w / GAME_HP if pistons else None,
                bsfc=sum(fuel[i] for i in pistons) / (piston_w / GAME_HP) if pistons and piston_w > 0. else None,
                turbine_thrust_kgf=turbine_n / KGF if turbines and exact else None,
                tsfc=sum(fuel[i] for i in turbines) / (turbine_n / GAME_KGF) if turbines and exact and turbine_n > 0. else None,
                efficiency=(thrust - booster) * v / power * 100. if power > 1. and not any(e['family'] == 2 for e in p['engines']) else None,
                rpm=point['propulsion']['engine_rpm'], pitch_deg=point['propulsion']['propeller_pitch_deg'],
                pitch_percent=point['propulsion'].get('propeller_pitch_percent'),
                manifold_ata=[float(state['engines'][i].get('manifold', math.nan)) for i in pistons],
                gear=[int(state['engines'][i].get('gear', 0)) + 1 for i in pistons],
                governor=list(prop['stationarity'].get('governor_status') or ()), tip_mach=tips,
                regime=regime(p['engines'], state['engines'], prop['stationarity'].get('governor_status'), blade_regime(chart, state, velocity, rates)),
                mixture_lean_percent=[m and m[0] for m in mixtures], mixture_rich_percent=[m and m[1] for m in mixtures],
                mixture_power_percent=[m and m[2] for m in mixtures],
                mixture_upper_lean_percent=[m and m[3] and m[3][0] for m in mixtures],
                mixture_upper_rich_percent=[m and m[3] and m[3][1] for m in mixtures],
                mach=float(air['mach']), ias_kmh=float(air['ias_u']) * 3.6)


def propeller_row(chart, speed_kmh, report):
    v = speed_kmh / 3.6
    prop = chart.operating_point(v, 1., [0.] * 5)['propulsion']
    if _needs_fine_pitch_check(prop):
        setup = _prop_fast_setup(chart)
        if setup is not None and setup['fine_seed'] is not None:
            from body_dynamics import G
            pm = _prop_model(setup, chart)
            candidate = _fine_pitch_record(pm, v, float(chart.dt), float(G), setup)
            if candidate is not None and 'controls' not in candidate:
                # The compiled loop's short record: the clicked point's report reads the full one of the same state.
                br, q = candidate['solved_at']
                full = pm.test_record(v, 1., 0., 0., float(G), float(chart.dt), q, br)
                candidate = full[0] if full is not None else None
            if candidate is not None:
                checked = _propeller_fields(chart, v, candidate, report)
                if checked is not None:
                    return checked
    return _propeller_fields(chart, v, prop, report)


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


def main_units(chart):
    """The aircraft's own direct-thrust engines, without lit rocket boosters (rocket_em.booster_units)."""
    return [u for u in chart.engine.units if not getattr(u, 'booster', False)]


def booster_fields(chart, v):
    """Lit rocket boosters' share of a row: their thrust (in the row's total) and propellant flow (not in fuel_kgh)."""
    lit = [u for u in getattr(chart.engine, 'units', ()) if getattr(u, 'booster', False)]
    if not lit:return {}
    force = [0.] * 3
    for u in lit:
        f, _ = u.vectors(v)
        force = [a + b for a, b in zip(force, f)]
    return dict(booster_thrust_kgf=float(force[0]) / KGF, booster_fuel_kgh=sum(3600. * u.consumption for u in lit))


def jet_row(chart, speed_kmh):
    from air_state import cache
    v = speed_kmh / 3.6
    air = cache([f32(v), 0., 0.], chart.config['altitude_m'])
    # The chart's nominal engine state (jet_nominal.evaluate): RPM, the scalar thrust and the fuel demand it sets, kg/s.
    nominal = [u.nominal(v) for u in main_units(chart)];fuel = [3600. * n['scalar']['consumption'] for n in nominal]
    engine = sum(n['scalar']['thrust'] for n in nominal)
    return dict(thrust_kgf=float(chart.engine.vectors(v)[0][0]) / KGF, rpm=[n['omega'] * 30 / math.pi for n in nominal],
                fuel_kgh=sum(fuel), engine_fuel_kgh=fuel, turbine_thrust_kgf=engine / KGF,
                tsfc=sum(fuel) / (engine / GAME_KGF) if engine > 0. else None, mach=float(air['mach']), ias_kmh=float(air['ias_u']) * 3.6,
                **booster_fields(chart, v))


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
    """The EM plotter's point solver (em_server.point_solver for a march chart): soft admission, compiled loop. The
    Instructor is off (Leo, 2026-11-15: "for the engine performance always use real stall"): level flight ends at the
    wing's stall (or another hard limit), not at the Instructor's AoA cap. Autotrim stays on (the EM page's Realistic
    control mode)."""
    chart = solver(name, altitude, throttle, pitch, mass, instructor=False)
    chart.soft_admission = True
    return chart


def level_regime(chart, p, state, prop):
    """A level-flight point's propulsion regime: `regime` on its solved propulsion state at its flight condition."""
    blades = ()
    if BLADE_REGIMES:
        from em_operating import flight_condition
        velocity, _, geometry, _ = flight_condition(chart, p['speed_kmh'] / 3.6, 1., list(p['solution']))
        blades = blade_regime(chart, state, tuple(map(f32, velocity)), tuple(map(f32, geometry['omega'])))
    return regime(chart.engine.properties['engines'], state['engines'], (prop.get('stationarity') or {}).get('governor_status'), blades, fuel=False)


def level_row(chart, speed_kmh, near, solve=None, regime_of=None):
    """SEP in 1 g level flight (Leo, 2026-10-28): the EM plotter's point solve at 1 g, seeded from a nearby solved point
    as em_server.solve_point seeds a clicked one; kept when the chart admits it and no engine over-revs (Leo, 2026-10-29).
    Returns (row, seed): seed holds what em_sampling.continuation_initial reads from the point. regime_of(point, state,
    propulsion): the row's regime instead of `level_regime`."""
    from altitude_chart import settle
    from em_sampling import continuation_initial
    # Above 9,900 m a state ending its step a hair upward is rejected; the altitude envelope's nudge lands it on the other side.
    if solve is None:solve = chart.solve
    try:p = settle(chart, solve, speed_kmh, solve(speed_kmh, 1., continuation_initial(chart, near, list(near['solution'])) if near is not None else None))
    except ValueError:return None, None
    prop = p.get('propulsion') or {}
    if not p.get('valid') or any(r > l * (1 + 1e-6) for r, l in zip(prop.get('engine_rpm') or (), prop.get('engine_rpm_threshold') or ())):
        return None, None
    v, a, f = speed_kmh / 3.6, math.radians(p['alpha_deg']), p['engine_force_n']
    state = p.get('_propulsion_seed')
    return dict(ps_mps=p['ps_mps'], alpha_deg=p['alpha_deg'],
                regime=() if not (chart.is_prop and isinstance(state, dict)) else
                level_regime(chart, p, state, prop) if regime_of is None else regime_of(p, state, prop),
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
    else:
        # None of the usual starts converges, as high up, where 1 g level flight only begins above 1,000 km/h or the
        # VNE lies below a fixed speed up to 14 km (MiG-25PD): try points spread over the requested range, halving it.
        # When none of these converges either, the march leaves from the last usual start as before.
        for f in (.5, .25, .75, .125, .375, .625, .875):
            i = round(f * (len(xs) - 1))
            if xs[i] not in rows:rows[xs[i]] = at(xs[i])
            if rows[xs[i]]['solved']:
                start = i
                break
    for side in (range(start + 1, len(xs)), range(start - 1, -1, -1)):
        misses = 0
        for i in side:
            if xs[i] not in rows:rows[xs[i]] = at(xs[i]) if misses < GIVE_UP else blank(xs[i])
            misses = 0 if rows[xs[i]]['solved'] else misses + 1
    return [rows[x] for x in xs]


# The plotted quantities (Leo, 2026-10-28: power, thrust, efficiency, SEP; RPM, blade pitch and manifold only in details;
# 2026-10-29: fuel flow; SFC, over throttle).
CURVES = {'power':lambda r:r.get('power_hp'), 'thrust':lambda r:r.get('thrust_kgf'), 'efficiency':lambda r:r.get('efficiency'),
          'fuel':lambda r:r.get('fuel_kgh'), 'sep':lambda r:r.get('ps_mps'),
          'mixture':lambda r:(r.get('mixture_rich_percent') or [None])[0]}


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
    motor's MaxThrMult cap, as rocket_em.RocketUnit runs it). Every 1% of throttle from 0 to the engine's own stop in
    the game (aces.exe pilot throttle cap, see without_wep: 110% when its ThrottleBoost > 1, else 100%); equal curves of
    several engines are one, with every engine listed."""
    from rocket_general import step
    def throttles(boost):
        top = 110. if boost > 1. else 100.
        return range(int(top) + 1)
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
        for i, u in enumerate(main_units(chart)):
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


def speed_limit_check(chart):
    """Whether a true airspeed is past the aircraft's limits at the chart's altitude: states there are not results (Leo,
    2026-10-31: the range has no cap, the aircraft's limits still apply). The EM chart's own: VNE, Mach, flap limits for
    the selected flaps, and the speeds where the selected wing sweep is unavailable."""
    limit = speed_limits(chart.fm, chart.config)
    maximum = limit['speed_kmh'] if limit['enforced'] else math.inf
    excluded = sweep_excluded_kmh(chart.fm, chart.config)
    if not excluded:
        return lambda speed: speed > maximum
    return lambda speed: speed > maximum or any(a < speed < b for a, b in excluded)


def past_limit(chart, speed_kmh):
    """`speed_limit_check` of the chart at a speed. The check is kept with the chart for its altitude: working the
    limits out takes 0.5 ms, and a sweep asks at every point (it was all of a jet curve's time and a sixth of an SEP
    curve's)."""
    kept = getattr(chart, '_speed_limit_check', None)
    if kept is None or kept[0] != chart.config['altitude_m']:
        kept = (chart.config['altitude_m'], speed_limit_check(chart))
        try:chart._speed_limit_check = kept
        except AttributeError:pass
    return kept[1](speed_kmh)


def within_limit(chart, speeds):
    """A speed sweep's range (min, max), ending just inside the aircraft's lowest limit, so its points stay as dense
    whatever range was asked for."""
    limit, (low, high) = speed_limits(chart.fm, chart.config), speeds
    return (low, min(high, limit['sample_speed_kmh'])) if limit['enforced'] and limit['sample_speed_kmh'] > low else speeds


def axial_point(chart, height, speed, propulsion, report):
    """An engine sweep row: the axial state at angle of attack 0, unsolved where the EM plotter's reference is."""
    if past_limit(chart, speed):return dict(altitude_m=height, speed_kmh=speed, solved=False)
    try:value = (jet_row(chart, speed) if propulsion == 'jet' else rocket_row(chart, speed) if propulsion == 'rocket' else
                 propeller_row(chart, speed, report))
    except (ValueError, ArithmeticError, np.linalg.LinAlgError):value = None
    return dict(value or {}, altitude_m=height, speed_kmh=speed, solved=value is not None)


# Whether a level-flight sweep's points are solved by the compiled point solver (the EM plotter's march engine,
# em_march_native: the same equations, 1 to 3 ms a point) instead of the chart's Python solve (5 to 8 ms).
FAST_LEVEL = True
# A level-flight point without a state is solved once more from the engine curve's state there (`engine_seed`), except
# within this distance (km/h along speed, m along altitude) of a point where that found nothing either.
ENGINE_SEED_EVERY = dict(speed=16., altitude=160.)


def level_point(chart, height, speed, near, solve=None):
    """A level-flight sweep row and its continuation seed. solve: the point solve (default: the chart's own)."""
    if past_limit(chart, speed):return dict(altitude_m=height, speed_kmh=speed, solved=False), None
    try:value, seed = level_row(chart, speed, near, solve)
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
        fast, at_altitude = [], {}
        def charted(h):
            """The level chart of altitude h and its point solve (None: the chart's own)."""
            if not FAST_LEVEL:return (level_solver(name, h, throttle, pitch, mass) if axis == 'altitude' else along()), None
            if not fast:fast.append(_level_charts(name, throttle, pitch, mass, h, lean=True)[1])
            if h not in at_altitude:
                if len(at_altitude) > 64:at_altitude.clear()
                at_altitude[h] = fast[0](h)
            return at_altitude[h]
        failed = set()   # coordinates where the solve itself found no state (not a limit)
        engine, unseeded = {}, []   # the engine row solvers by altitude; coordinates where the engine's state did not help
        def engine_seed(chart, height, speed):
            """A start for the level-flight solve at a point where it found no state: the chart's own attitude guess
            with the propulsion in the state the engine's row solver finds there (angle of attack 0). A level-flight solve started at rest on the governor's range
            finds nothing where the propeller's state is far from it: on a pitch stop with the RPM free (windmilling at
            low throttle: the Bf 109 at 20% had no SEP below 371 km/h, its states reach down to 209), or everywhere for
            a propeller whose two stops coincide (Ba.65 K.14 L)."""
            from em_stationary import JointProblem
            if propulsion in ('jet', 'rocket'):return None
            try:
                # the row solver's own state there (from its solved neighbours, or from rest and the alternate starts)
                if 'rows' not in engine:engine['rows'] = _axial_rows(name, AIRCRAFT[name], True, throttle, pitch, mass, height)[0]
                if height not in engine:
                    if len(engine) > 40:
                        for key in [k for k in engine if k not in ('rows', 'layout')]:del engine[key]
                    engine[height] = engine['rows'](height)
                state = engine[height](speed)
                if not state['solved'] or 'solved_at' not in state:return None
                guess = list(chart.initial_guess(speed, 1.))
                if 'layout' not in engine:
                    problem = JointProblem(chart.core(), speed, 1., guess)
                    engine['layout'] = (list(problem.coordinate_names), list(problem.attitude_indices), problem.prop_start - problem.history_start)
                names, indices, history = engine['layout']
                br, q = state['solved_at']
                coords = [guess[i] for i in indices] + [guess[0]] * history + [float(c) for c in q]
                if len(coords) != len(names):return None
                return dict(solution=guess, stationary_coordinate_names=names, stationary_coordinates=coords, stationary_request_indices=list(br))
            except (ValueError, ArithmeticError, np.linalg.LinAlgError, AttributeError, KeyError):return None
        def solve(x):
            at_near, near = min(((abs(s - x), s, seed) for s, (r, seed) in store.items() if seed), key=lambda t:t[0], default=(0, None, None))[1:]
            chart, point_solve = charted(x if axis == 'altitude' else fixed)
            # Past a point where the solve found no state (beyond the stall, on the way out from the solved part): only
            # the solve continued from the nearest solved point. The point solver's further starts (its kept seeds, the
            # trim guess) take about three times as long to fail there, and a march tries 25 such points before it stops.
            quick = point_solve is not None and near is not None and any(min(x, at_near) < u < max(x, at_near) for u in failed)
            if quick:
                getattr(chart, '_stationary_seeds', {}).clear()
                chart.initial_guess = _no_guess
            try:out = level_point(chart, x, fixed, near, point_solve) if axis == 'altitude' else level_point(chart, fixed, x, near, point_solve)
            except ValueError:
                if not quick:raise
                out = (dict(altitude_m=x if axis == 'altitude' else fixed, speed_kmh=fixed if axis == 'altitude' else x, solved=False), None)
            finally:
                if quick:del chart.initial_guess
            height, speed = (x, fixed) if axis == 'altitude' else (fixed, x)
            if not out[0]['solved'] and not past_limit(chart, speed):
                # once more from the engine curve's state there; not within ENGINE_SEED_EVERY of a point where that failed
                if point_solve is not None and not any(abs(x - u) < ENGINE_SEED_EVERY[axis] for u in unseeded):
                    seed = engine_seed(chart, height, speed)
                    again = level_point(chart, height, speed, seed, point_solve) if seed is not None else None
                    if again is not None and again[0]['solved']:out = again
                    else:unseeded.append(x)
                if not out[0]['solved']:failed.add(x)
            return out
    else:
        def solve(x):
            return (axial_point(solver(name, x, throttle, pitch, mass), x, fixed, propulsion, report) if axis == 'altitude' else
                    axial_point(along(), fixed, x, propulsion, report)), None
    def at(x):
        if x not in store:store[x] = solve(x)
        return store[x][0]
    def carry(x, u):
        """The level-flight state at the sweep's solved point u continued to its point x: kept (True) where x has no
        state or one of less thrust (`_more_thrust`: two propeller states coexist over a band, and a march keeps the
        one it came in on)."""
        if not level or u not in store or not store[u][1] or x not in store:return False
        height, speed = (x, fixed) if axis == 'altitude' else (fixed, x)
        chart, point_solve = charted(height)
        if past_limit(chart, speed):return False
        try:out = level_point(chart, height, speed, store[u][1], point_solve)
        except ValueError:return False
        if not out[0]['solved'] or not _more_thrust(out[0], store[x][0] if store[x][0]['solved'] else None):return False
        store[x] = out
        return True
    at.carry = carry
    return at


def settle_level(rows, at, place):
    """A level-flight sweep's rows after the more-thrust rule: where the thrust steps between two neighbouring solved
    points (by more than 4 times what it changes per km/h or metre on either side), each side's state is continued
    across the step, point by point, for as long as it has more thrust than what is there."""
    xs = [place(r) for r in rows]
    def slope(i, j):
        if not (0 <= i < len(rows) and 0 <= j < len(rows)) or not (rows[i]['solved'] and rows[j]['solved']) or xs[i] == xs[j]:return None
        return abs(rows[i]['thrust_kgf'] - rows[j]['thrust_kgf']) / abs(xs[i] - xs[j])
    def steps(i, j):
        here = slope(i, j)
        if here is None or abs(rows[i]['thrust_kgf'] - rows[j]['thrust_kgf']) <= 10. * PREFER * max(abs(rows[i]['thrust_kgf']), abs(rows[j]['thrust_kgf']), 1.):return False
        beside = [s for s in (slope(min(i, j) - 1, min(i, j)), slope(max(i, j), max(i, j) + 1)) if s is not None]
        return bool(beside) and here > 4. * max(beside)
    for _ in range(4):
        changed = False
        for step in (1, -1):
            order = range(len(rows) - 2, -1, -1) if step == 1 else range(1, len(rows))
            for i in order:
                j = i + step
                if rows[j]['solved'] and rows[i]['solved'] and steps(i, j) and at.carry(xs[i], xs[j]):
                    rows[i] = at(xs[i]);changed = True
        if not changed:break
    return rows


def blank(axis, fixed):
    """An unsolved row of a sweep (past a march's give-up)."""
    return (lambda h:dict(altitude_m=h, speed_kmh=fixed, solved=False)) if axis == 'altitude' else \
        (lambda v:dict(altitude_m=fixed, speed_kmh=v, solved=False))


def starts(axis, prop):
    """Where a march starts: a point that converges cold."""
    return (0., 3000., 6000.) if axis == 'altitude' else (400., 300., 500., 250.) if prop else (800., 600., 1000., 450.)


PLACE = dict(altitude=lambda r:r['altitude_m'], speed=lambda r:r['speed_kmh'])


def admitted(name, throttle, pitch):
    """The aircraft's catalog entry and whether it has propellers, or ValueError for settings the page cannot calculate."""
    entry = AIRCRAFT[name]
    if not entry['supported']:raise ValueError(entry['name'] + ': ' + entry['reason'])
    prop = entry['propulsion'] not in ('jet', 'rocket')
    if pitch is not None and not (prop and entry.get('manual_controls', {}).get('propeller_pitch')):
        raise ValueError('Manual propeller pitch is unavailable for ' + entry['name'])
    if throttle > 100. and name in without_wep():raise ValueError(entry['name'] + ' has no WEP: its throttle stops at 100%')
    return entry, prop


SURFACE_SEEDS = OrderedDict()   # (name, throttle, pitch, mass) -> [(altitude_m, speed_kmh, seed), ...]


def _remember_seeds(key, seeds):
    if not seeds:return
    SURFACE_SEEDS[key] = seeds;SURFACE_SEEDS.move_to_end(key)
    while len(SURFACE_SEEDS) > 16:SURFACE_SEEDS.popitem(last=False)


def _nearest_surface_seed(key, altitude_m, speed_kmh):
    seeds = SURFACE_SEEDS.get(key)
    if not seeds:return None
    return min(seeds, key=lambda item:((item[0] - altitude_m) / 1000.) ** 2 + ((item[1] - speed_kmh) / 100.) ** 2)[2]


def _prop_fast_setup(base_chart):
    import em_march_native
    from em_stationary import JointProblem
    from prop_equilibrium_blocks import variable_blocks
    from propeller_general import inverse_basis
    from prop_quasisteady import table as prop_table
    s2, _ = em_march_native.load(build=False)
    if s2 is None:return None
    try:
        pm = s2.PropModel(base_chart)
        layout = JointProblem(base_chart.core(), 300., 1., [0., 0., 0., 0., 0.])
        pm.set_joint_layout(layout.active, layout.rows)
    except Exception:
        return None
    ps = layout.prop_start
    p = base_chart.engine.properties
    blocks = variable_blocks(p, getattr(layout, 'paths', ()))
    # Check row blocks match column blocks when square (standard shaft+wake layout).
    n_act = len(layout.active)
    use_blocks = (len(layout.rows) == n_act and sum(len(b) for b in blocks) == n_act)
    prop_checks = []
    for shaft_idx, shaft in enumerate(p['transmissions']):
        for link in shaft['propellers']:
            idx = link['index'];pp = p['propellers'][idx]['properties']
            inv0 = float(inverse_basis(tuple(pp['basis']))[0])
            has_width = any(w > 0. and r > 0. for r, w in zip((.35 * pp['radius'], .55 * pp['radius'], .75 * pp['radius'], .95 * pp['radius']), pp['width']))
            prop_checks.append((shaft_idx, idx, float(link['ratio']), inv0, pp['airflow'], bool(pp['coaxial']), has_width))
    families = [e['family'] for e in p['engines']]
    blocks = blocks if use_blocks else (tuple(range(n_act)),)
    rscale = np.asarray(layout.correction_residual_scales[ps:], dtype=float)
    initial = np.asarray(layout.initial[ps:], dtype=float)
    fine_seed = initial.copy() if any(path[0] == 'pitch' for path in layout.paths) else None
    if fine_seed is not None:
        for j, path in enumerate(layout.paths):
            if path[0] == 'pitch':
                fine_seed[j] = 0.
            elif path[0] == 'flow' and path[2] == 2:
                fine_seed[j] = 0.
    # The EM plotter's alternate starts (em_stationary: a fine and a coarse blade with a weak and a strong wake).
    alternates = []
    for fraction, axial in ((.05, -.5), (.85, 2.)):
        other = initial.copy()
        for j, path in enumerate(layout.paths):
            if path[0] == 'pitch':other[j] = fraction
            elif path[0] == 'flow' and path[2] < 2:other[j] = axial
        alternates.append(np.clip(other, layout.lower[ps:], layout.upper[ps:]))
    # A governed propeller whose two pitch stops coincide (Ba.65 K.14 L: 24 degrees both) has no state on the governor's
    # own range: the coordinate changes nothing between 0 and 1, so a solve started there never leaves it. Its states
    # are on the stops, with the RPM free (below 0: under the governor's RPM, above 1: over it): two starts there.
    if any(link['index'] is not None and p['propellers'][link['index']]['properties'].get('governor')
           and p['propellers'][link['index']]['properties']['pitch_max'] <= p['propellers'][link['index']]['properties']['pitch_min']
           for shaft in p['transmissions'] for link in shaft['propellers']):
        for stop in (-.05, 1.05):
            other = initial.copy()
            for j, path in enumerate(layout.paths):
                if path[0] == 'pitch':other[j] = stop
            alternates.append(np.clip(other, layout.lower[ps:], layout.upper[ps:]))
    return dict(s2=s2, active=layout.active, rows=layout.rows, nt=len(p['transmissions']), engines=p['engines'], alternates=alternates,
                flows={}, table_speeds=[e['turbine']['speed'] for e in p['engines'] if e.get('turbine') and e['turbine'].get('speed')],
                blocks=blocks, max_color=max(map(len, blocks), default=0),
                prop_checks=prop_checks, prop_table=prop_table,
                pistons=[i for i, f in enumerate(families) if f in (0, 1)],
                rockets={i for i, f in enumerate(families) if f == 3},
                has_eff=not any(f == 2 for f in families),
                lower=np.asarray(layout.lower[ps:], dtype=float), upper=np.asarray(layout.upper[ps:], dtype=float),
                initial=initial, fine_seed=fine_seed, steps=np.asarray(layout.steps[ps:], dtype=float),
                slim=np.asarray(layout.correction_step_limits[ps:], dtype=float),
                rscale=rscale, closed_tol=rscale * .1, axial=hasattr(s2.PropModel, 'axial'),
                omega_limits=[e['properties']['omega_limit'] for e in p['engines']],
                plain=dict(lower=[float(v) for v in layout.lower[ps:]], upper=[float(v) for v in layout.upper[ps:]],
                           steps=[float(v) for v in layout.steps[ps:]], slim=[float(v) for v in layout.correction_step_limits[ps:]],
                           rscale=[float(v) for v in rscale], closed_tol=[float(v) * .1 for v in rscale]))


def _prop_model(setup, chart):
    """The compiled propulsion model of a chart (one altitude), laid out as JointProblem's propulsion block and, where
    the build has it, set for its own Newton loop (PropModel.axial)."""
    pm = setup['s2'].PropModel(chart)
    pm.set_joint_layout(setup['active'], setup['rows'])
    if setup['axial']:
        plain = setup['plain']
        pm.set_axial(plain['lower'], plain['upper'], plain['steps'], plain['slim'], plain['rscale'], setup['blocks'])
        if hasattr(pm, 'set_axial_row'):
            inv0 = [0.] * (1 + max((c[1] for c in setup['prop_checks']), default=0))
            for c in setup['prop_checks']:inv0[c[1]] = c[3]
            pm.set_axial_row(setup['omega_limits'], inv0, [float(x) for x in setup['initial']], [[float(x) for x in a] for a in setup['alternates']],
                             None if setup['fine_seed'] is None else [float(x) for x in setup['fine_seed']])
    return pm


def _prop_record(pm, v, dt, g_f, q, br, summary, setup):
    """The propulsion record of a solved state as the engine rows read it: from the compiled loop's summary (the
    fields `_fast_surface_prop_fields`, `regime` and the fine-pitch check use, without the full record's other
    entries), else PropModel.test_record's."""
    if summary is None:
        rec = pm.test_record(v, 1., 0., 0., g_f, dt, q, br)
        return None if rec is None else rec[0]
    converged, force, status, engines, props, shafts, *shafts_thrust = summary[:7]
    limits = setup['omega_limits']
    return dict(converged=converged, force=list(force), stationarity=dict(governor_status=status), solved_at=(tuple(br), list(q)),
                overspeed_engines=[k for k, e in enumerate(engines) if e[0] > limits[k] * (1 + 1e-6)],
                shaft_thrust=shafts_thrust[0] if shafts_thrust else None,
                state=dict(engines=[dict(omega=e[0], torque=e[1], consumption=e[2], gear=e[3], regulator=e[4], potential_manifold=e[5],
                                         force=(e[6], 0., 0.), manifold=e[7], **(dict(regime=e[8], fuel_regime=e[9]) if len(e) > 8 else {}))
                                    for e in engines],
                           propellers=[dict(pitch=p[0], flow=(p[1], p[2], p[3]), equilibrium_flow=(p[4], 0., 0.)) for p in props],
                           transmissions=[dict(omega=w) for w in shafts]))


def _fast_surface_prop_fields(prop, v, setup, view=None):
    if not prop or not prop.get('converged') or prop.get('overspeed_engines'):return None
    state = prop['state']
    trans = state['transmissions'];rotors = state['propellers'];engines = state['engines']
    vf = float(f32(v))
    prop_table = setup['prop_table']
    for shaft_idx, idx, ratio, inv0, airflow_rows, coaxial, has_width in setup['prop_checks']:
        local0 = float(f32(vf * inv0))
        upstream = local0 + prop_table(airflow_rows, local0)[0]
        rotor = rotors[idx];flow = rotor['flow']
        axial = local0 + .5 * (flow[0] + flow[1])
        if not (math.isfinite(upstream) and math.isfinite(axial) and upstream > 0. and axial > 0.):return None
        if coaxial and not (upstream + .5 * rotor['equilibrium_flow'][0] > 0.):return None
        omega_p = trans[shaft_idx]['omega'] * ratio
        if has_width:
            if omega_p - .25 * flow[2] <= 0.:return None
            if coaxial and omega_p + .25 * flow[2] <= 0.:return None
    power = sum(e.get('torque', 0.) * e['omega'] for e in engines)
    thrust = float(prop['force'][0])
    rockets = setup['rockets'];pistons = setup['pistons']
    fuel_kgh = sum(3600. * e['consumption'] for i, e in enumerate(engines) if i not in rockets)
    booster = sum(float(engines[i].get('force', (0., 0., 0.))[0]) for i in rockets) if rockets else 0.
    return dict(thrust_kgf=thrust / KGF,
                power_hp=power / GAME_HP if power > 0. or pistons else None,
                fuel_kgh=fuel_kgh,
                efficiency=(thrust - booster) * v / power * 100. if power > 1. and setup['has_eff'] else None,
                gear=[int(engines[i].get('gear', 0)) + 1 for i in pistons],
                regime=regime(setup['engines'], engines, prop['stationarity'].get('governor_status'),
                              blade_regime(view, state, (f32(v), 0., 0.), (0., 0., 0.)) if view is not None else ()))


def _summary_fields(summary, v, setup):
    """`_fast_surface_prop_fields` of `_prop_record`, straight from the compiled loop's summary (the same tests and
    numbers, without the record between): the row fields, None where the state is no result, False where an engine
    over-revs."""
    converged, force, status, engines, props, shafts = summary[:6]
    if not converged:return None
    limits = setup['omega_limits']
    for k, e in enumerate(engines):
        if e[0] > limits[k] * (1 + 1e-6):return False
    flows = setup['flows'].get(v)
    if flows is None:
        # Each propeller's own speed and the flow ahead of it: they follow from the flight speed alone.
        vf, prop_table, flows = float(f32(v)), setup['prop_table'], []
        for shaft_idx, idx, ratio, inv0, airflow_rows, coaxial, has_width in setup['prop_checks']:
            local0 = float(f32(vf * inv0))
            flows.append((local0, local0 + prop_table(airflow_rows, local0)[0]))
        setup['flows'][v] = flows
    for (shaft_idx, idx, ratio, inv0, airflow_rows, coaxial, has_width), (local0, upstream) in zip(setup['prop_checks'], flows):
        rotor = props[idx]
        axial = local0 + .5 * (rotor[1] + rotor[2])
        if not (math.isfinite(upstream) and math.isfinite(axial) and upstream > 0. and axial > 0.):return None
        if coaxial and not (upstream + .5 * rotor[4] > 0.):return None
        omega_p = shafts[shaft_idx] * ratio
        if has_width:
            if omega_p - .25 * rotor[3] <= 0.:return None
            if coaxial and omega_p + .25 * rotor[3] <= 0.:return None
    power = sum(e[1] * e[0] for e in engines)
    thrust = float(force[0])
    rockets, pistons = setup['rockets'], setup['pistons']
    fuel_kgh = sum(3600. * e[2] for i, e in enumerate(engines) if i not in rockets)
    booster = sum(float(engines[i][6]) for i in rockets) if rockets else 0.
    return dict(thrust_kgf=thrust / KGF,
                power_hp=power / GAME_HP if power > 0. or pistons else None,
                fuel_kgh=fuel_kgh,
                efficiency=(thrust - booster) * v / power * 100. if power > 1. and setup['has_eff'] else None,
                gear=[int(engines[i][3]) + 1 for i in pistons],
                regime=(tuple((int(e[3]), e[8], e[9]) for e in engines), tuple(status or ()), ()),
                # Each propeller's blade sections' polar branches (PropState.branches) and each turbine's cell of its
                # thrust table along speed: the curves have a corner where one changes, and a row's knots close in on
                # it (engine_surface.SOFT).
                **(dict(soft=tuple(p[5] for p in props) + tuple(bisect_right(nodes, float(f32(v))) for nodes in setup['table_speeds']))
                   if props and len(props[0]) > 5 else {}))


def _solved(matrix, right):
    """x with matrix x = right (a few unknowns: elimination with row pivoting), or None for a singular matrix."""
    n = len(right)
    m = [row[:] + [right[i]] for i, row in enumerate(matrix)]
    for c in range(n):
        pivot = max(range(c, n), key=lambda r:abs(m[r][c]))
        if m[pivot][c] == 0.:return None
        if pivot != c:m[c], m[pivot] = m[pivot], m[c]
        top = m[c];d = top[c]
        for r in range(c + 1, n):
            row = m[r];f = row[c] / d
            if f:
                for k in range(c, n + 1):row[k] -= f * top[k]
    x = [0.] * n
    for c in range(n - 1, -1, -1):
        row = m[c];total = row[n]
        for k in range(c + 1, n):total -= row[k] * x[k]
        x[c] = total / row[c]
    return x


def _newton_prop(pm, v, dt, g_f, br, q0, jac_warm, setup):
    """A propeller state at alpha 0: Newton on the compiled propulsion equations (PropModel.test_sample), the Jacobian
    by differences (one sample per unknown of a shaft, the shafts together) and kept between steps and points by
    Broyden's update. Plain floats and lists: with three unknowns per shaft, numpy's call overhead was most of a
    point's time. (q, residual, Jacobian, True, summary) or None; summary: the solved state from the compiled loop
    (PropModel.axial, the same loop in the native module, used where the build has it), else None."""
    axial = getattr(pm, 'axial', None) if setup.get('axial') else None
    if axial is not None:
        out = axial(v, g_f, dt, q0, br, jac_warm)
        if out is None:return None
        if out is not False:return out[0], out[1], out[2], True, out[3]
    plain = setup['plain']
    lo, hi, steps, slim, rscale, closed_tol = (plain[k] for k in ('lower', 'upper', 'steps', 'slim', 'rscale', 'closed_tol'))
    blocks, sqrt = setup['blocks'], math.sqrt
    n = len(q0)
    q = [min(max(float(x), a), b) for x, a, b in zip(q0, lo, hi)]
    res = pm.test_sample(v, 1., 0., 0., g_f, dt, q, br)
    if res is None:return None
    r = res[0];m = len(r)
    if m != n:
        out = _newton_prop_general(pm, v, dt, g_f, br, q0, None if jac_warm is None else np.asarray(jac_warm, dtype=float), setup)
        return None if out is None else (*out, None)
    def closed(sample):
        if not sample[2]:return False
        for x, t in zip(sample[0], closed_tol):
            if abs(x) > t:return False
        return True
    def size(residual):
        total = 0.
        for x, scale in zip(residual, rscale):total += (x / scale) ** 2
        return sqrt(total)
    def update(jac, residual, old, step, denom):
        for row, new, was in zip(jac, residual, old):
            e = new - was
            for a, b in zip(row, step):e -= a * b
            e /= denom
            for j in range(n):row[j] += e * step[j]
    if closed(res):return q, r, jac_warm, True, None
    jac = [list(row) for row in jac_warm] if jac_warm is not None and len(jac_warm) == m and len(jac_warm[0]) == n else None
    fresh, norm0 = False, size(r)
    for _ in range(8):
        if jac is None:
            jac = [[0.] * n for _ in range(m)]
            ok = True
            if len(blocks) > 1:
                for color in range(setup['max_color']):
                    qp, deltas = q[:], {}
                    for b in blocks:
                        if color < len(b):
                            c = b[color];h = steps[c] if q[c] + steps[c] <= hi[c] else -steps[c]
                            qp[c] += h;deltas[c] = h
                    rp = pm.test_sample(v, 1., 0., 0., g_f, dt, qp, br)
                    if rp is None:ok = False;break
                    rp = rp[0]
                    for b in blocks:
                        if color < len(b):
                            c = b[color];d = deltas[c]
                            for row in b:jac[row][c] = (rp[row] - r[row]) / d
            else:
                for j in range(n):
                    h = steps[j] if q[j] + steps[j] <= hi[j] else -steps[j]
                    qp = q[:];qp[j] += h
                    rp = pm.test_sample(v, 1., 0., 0., g_f, dt, qp, br)
                    if rp is None:ok = False;break
                    rp = rp[0]
                    for row in range(m):jac[row][j] = (rp[row] - r[row]) / h
            if not ok:break
            fresh = True
        dq = _solved(jac, [-x for x in r])
        if dq is None:
            if not fresh:jac = None;continue
            break
        ratio = max(abs(d) / limit for d, limit in zip(dq, slim)) if n else 0.
        if ratio > 1.:dq = [d / ratio for d in dq]
        accepted, alpha = False, 1.
        for _ in range(3):
            qn = [min(max(x + alpha * d, a), b) for x, d, a, b in zip(q, dq, lo, hi)]
            rn = pm.test_sample(v, 1., 0., 0., g_f, dt, qn, br)
            if rn is not None:
                rv = rn[0];norm1 = size(rv)
                step = [a - b for a, b in zip(qn, q)]
                denom = 0.
                for d in step:denom += d * d
                if closed(rn):
                    if denom > 1e-24:update(jac, rv, r, step, denom)
                    return qn, rv, jac, True, None
                if norm1 < (.75 if not fresh else .98) * norm0:
                    if alpha == 1. and denom > 1e-24 and norm1 < .65 * norm0:update(jac, rv, r, step, denom)
                    else:jac = None
                    q, r, norm0, fresh, accepted = qn, rv, norm1, False, True
                    break
            alpha *= .5
        if not accepted:
            if not fresh:jac = None;continue
            break
    res = pm.test_sample(v, 1., 0., 0., g_f, dt, q, br)
    if res is not None and res[2] and all(abs(x) <= scale for x, scale in zip(res[0], rscale)):return q, res[0], jac, True, None
    return None


def _newton_prop_general(pm, v, dt, g_f, br, q0, jac_warm, setup):
    """`_newton_prop` with numpy, for layouts with another number of equations than unknowns (least squares)."""
    lo, hi, steps, slim, rscale = setup['lower'], setup['upper'], setup['steps'], setup['slim'], setup['rscale']
    blocks = setup['blocks']
    closed_tol = setup['closed_tol']
    n = len(q0)
    q = np.clip(q0, lo, hi)
    res = pm.test_sample(v, 1., 0., 0., g_f, dt, q, br)
    if res is None:return None
    r = np.asarray(res[0], dtype=float)
    if res[2] and np.all(np.abs(r) <= closed_tol):return q, r, jac_warm, True
    jac = jac_warm.copy() if jac_warm is not None and jac_warm.shape == (len(r), n) else None
    fresh = False
    norm0 = float(np.linalg.norm(r / rscale))
    for _ in range(8):
        if jac is None:
            jac = np.zeros((len(r), n), dtype=float)
            ok = True
            if len(blocks) > 1 and len(r) == n:
                for color in range(setup['max_color']):
                    cols = [b[color] for b in blocks if color < len(b)]
                    qp = q.copy();deltas = {}
                    for c in cols:
                        h = steps[c] if q[c] + steps[c] <= hi[c] else -steps[c]
                        qp[c] += h;deltas[c] = h
                    rp = pm.test_sample(v, 1., 0., 0., g_f, dt, qp, br)
                    if rp is None:ok = False;break
                    diff = np.asarray(rp[0], dtype=float) - r
                    for b in blocks:
                        if color < len(b):
                            c = b[color];rows_b = list(b)
                            jac[rows_b, c] = diff[rows_b] / deltas[c]
            else:
                for j in range(n):
                    h = steps[j] if q[j] + steps[j] <= hi[j] else -steps[j]
                    qp = q.copy();qp[j] += h
                    rp = pm.test_sample(v, 1., 0., 0., g_f, dt, qp, br)
                    if rp is None:ok = False;break
                    jac[:, j] = (np.asarray(rp[0], dtype=float) - r) / h
            if not ok:break
            fresh = True
        try:dq = np.linalg.solve(jac, -r) if jac.shape[0] == jac.shape[1] else np.linalg.lstsq(jac, -r, rcond=None)[0]
        except np.linalg.LinAlgError:
            if not fresh:jac = None;continue
            break
        ratio = float(np.max(np.abs(dq) / slim)) if n else 0.
        if ratio > 1.:dq = dq / ratio
        accepted = False
        alpha = 1.
        for _ in range(3):
            qn = np.clip(q + alpha * dq, lo, hi)
            rn = pm.test_sample(v, 1., 0., 0., g_f, dt, qn, br)
            if rn is not None:
                rv = np.asarray(rn[0], dtype=float)
                norm1 = float(np.linalg.norm(rv / rscale))
                if rn[2] and np.all(np.abs(rv) <= closed_tol):
                    step_vec = qn - q;denom = float(np.dot(step_vec, step_vec))
                    if denom > 1e-24:jac = jac + np.outer(rv - r - jac @ step_vec, step_vec) / denom
                    return qn, rv, jac, True
                if norm1 < (.75 if not fresh else .98) * norm0:
                    step_vec = qn - q;denom = float(np.dot(step_vec, step_vec))
                    if alpha == 1. and denom > 1e-24 and norm1 < .65 * norm0:
                        jac = jac + np.outer(rv - r - jac @ step_vec, step_vec) / denom
                    else:jac = None
                    q, r, norm0, fresh, accepted = qn, rv, norm1, False, True
                    break
            alpha *= .5
        if not accepted:
            if not fresh:jac = None;continue
            break
    res = pm.test_sample(v, 1., 0., 0., g_f, dt, q, br)
    if res is not None and res[2] and np.all(np.abs(res[0]) <= rscale):
        return q, np.asarray(res[0], dtype=float), jac, True
    return None


def _needs_fine_pitch_check(prop):
    if prop and prop.get('overspeed_engines'):
        return False
    if not prop or not prop.get('converged'):
        return True
    return (prop['force'][0] < 0. and
            'target RPM' in prop['stationarity'].get('governor_status', ()))


def _fine_pitch_record(pm, v, dt, g_f, setup):
    # At the fine stop the shaft may windmill below the governor request. Only
    # accept a closed fine-stop solution, never the initial guess itself.
    import itertools
    seed = setup['fine_seed']
    probe = pm.test_sample(v, 1., 0., 0., g_f, dt, seed, [0] * setup['nt'])
    if probe is None:
        return None
    best, best_norm = None, math.inf
    for br in itertools.product(*(range(max(1, len(c))) for c in probe[3])):
        out = _newton_prop(pm, v, dt, g_f, br, seed, None, setup)
        if out is None:
            continue
        prop = _prop_record(pm, v, dt, g_f, out[0], br, out[4], setup)
        if prop is None:
            continue
        if (not prop.get('converged') or prop.get('overspeed_engines') or prop['force'][0] >= 0. or
                'minimum pitch' not in prop['stationarity']['governor_status']):
            continue
        norm = float(np.linalg.norm(out[1]))
        if norm < best_norm:
            best, best_norm = prop, norm
    return best


# Two states that coexist at one point (a propeller with its blades stalled or not): the one with more thrust is the
# result (Leo, 2026-10-21, for the EM plotter). A state replaces another when its thrust is higher by more than this
# share of it (two solves of one state differ by a few millionths).
PREFER = 1e-4
DISCOVER_M = 300.      # a row farther than this from every solved row is also started from rest and the alternate seeds


def _more_thrust(new, old):
    """Whether the row `new` is preferred to `old` (None: no result)."""
    return old is None or new['thrust_kgf'] > old['thrust_kgf'] + PREFER * max(abs(old['thrust_kgf']), 1.)


SLOW_START_KMH = 35.   # below it a propeller point gets the EM plotter's (slow) solve only on request ('always')
# Whether a surface row asks the EM plotter's solve for a speed next to a solved one that the compiled loop could not
# continue to. Off: it cost 40 ms (idle) to 380 ms (B-17, four engines) a time, most of such a surface, and a row is
# continued 1 km/h at a time from its solved neighbour anyway, which is what found the states it used to find.
SURFACE_THOROUGH = False


def _solve_prop_fast(view, pm, setup, speed_kmh, warm, allow_fallback=True, fallback_chart=None, seed=None, cold=True):
    """A propeller state at alpha 0: (row fields or None, the solved coordinates per request branch to start a
    neighbouring point from). warm: such coordinates of a nearby point; seed: coordinates to start from where warm has
    none (default: the layout's initial ones); cold False: only from warm (no second solve from rest)."""
    import itertools
    from body_dynamics import G
    g_f, v, dt = float(G), speed_kmh / 3.6, float(view.dt)
    if setup.get('axial') and hasattr(pm, 'candidates'):found = pm.candidates(v, g_f, dt)
    else:
        probe = pm.test_sample(v, 1., 0., 0., g_f, dt, setup['initial'], [0] * setup['nt'])
        found = probe[3] if probe is not None else None
    cands = [range(len(c)) if len(c) else [0] for c in found] if found is not None else [[0] for _ in range(setup['nt'])]
    branches = list(itertools.product(*cands))
    trials, next_warm, used_warm = [], {}, False
    for br in branches:
        prev = warm.get(br) if warm else None
        out = _newton_prop(pm, v, dt, g_f, br, prev[0], prev[1], setup) if prev is not None else None
        if out is not None:used_warm = True
        elif cold or prev is None:out = _newton_prop(pm, v, dt, g_f, br, setup['initial'] if seed is None else seed, None, setup)
        if out is not None:
            q, r, jac, closed, summary = out
            next_warm[br] = (q, jac)
            trials.append((not closed, math.sqrt(sum(float(x) * float(x) for x in r)), br, q, summary))
    def _fallback():
        if not allow_fallback or (speed_kmh < SLOW_START_KMH and allow_fallback != 'always'):return None, {}
        try:val = propeller_row(fallback_chart() if fallback_chart is not None else view, speed_kmh, None)
        except (ValueError, ArithmeticError, np.linalg.LinAlgError):val = None
        return val, {}
    if not trials or not any(not t[0] for t in trials):
        fine = _fine_pitch_record(pm, v, dt, g_f, setup) if setup['fine_seed'] is not None else None
        if fine is not None:
            fields = _fast_surface_prop_fields(fine, v, setup, view)
            if fields is not None:
                if 'solved_at' in fine:fields['solved_at'] = fine['solved_at']
                return fields, {}
        return _fallback()
    _, _, br, q, summary = min(trials, key=lambda t:(t[0], t[1], t[2]))
    if summary is not None and len(summary) > 6 and summary[0] and summary[1][0] >= 0. and not BLADE_REGIMES:
        # The usual state (converged, pulling): its fields straight from the compiled loop's summary. The record, the
        # second solve from rest and the fine-pitch search below are for the others (negative thrust, not converged).
        val = _summary_fields(summary, v, setup)
        if val is False:return _fallback() if allow_fallback == 'always' else (None, {})
        if val is None:return _fallback()
        val['solved_at'] = (br, q);val['summary'] = summary
        return val, next_warm
    prop_rec = _prop_record(pm, v, dt, g_f, q, br, summary, setup)
    if prop_rec is None:return _fallback()
    if used_warm and float(prop_rec['force'][0]) < 0.:
        cold = _newton_prop(pm, v, dt, g_f, br, setup['initial'], None, setup)
        if cold is not None and cold[3]:
            crec = _prop_record(pm, v, dt, g_f, cold[0], br, cold[4], setup)
            if crec is not None and crec.get('converged') and float(crec['force'][0]) > float(prop_rec['force'][0]):
                prop_rec = crec;next_warm[br] = (cold[0], cold[2])
    val = _fast_surface_prop_fields(prop_rec, v, setup, view)
    if setup['fine_seed'] is not None and _needs_fine_pitch_check(prop_rec):
        fine = _fine_pitch_record(pm, v, dt, g_f, setup)
        if fine is not None:
            fields = _fast_surface_prop_fields(fine, v, setup, view)
            if fields is not None:
                if 'solved_at' in fine:fields['solved_at'] = fine['solved_at']
                return fields, {}
    if val is None and (allow_fallback == 'always' or not prop_rec.get('overspeed_engines')):
        return _fallback()
    if val is not None and br in next_warm:val['solved_at'] = (br, next_warm[br][0])
    return val, next_warm if val is not None else {}


def _pack_axial_pieces(rows, prop, has_efficiency):
    pieces, current = [], []
    def flush():
        if not current:return
        piece = dict(gear=current[0].get('gear') or [],
                     x=[round(r['speed_kmh'], 2) for r in current],
                     thrust=[round(r['thrust_kgf'], 2) for r in current],
                     fuel=[round(r['fuel_kgh'], 2) for r in current])
        if prop:
            piece['power'] = [round(r['power_hp'], 2) if r.get('power_hp') is not None else None for r in current]
            if has_efficiency:
                piece['efficiency'] = [round(r['efficiency'], 2) if r.get('efficiency') is not None else None for r in current]
        pieces.append(piece);current.clear()
    for r in rows:
        if not r.get('solved'):flush();continue
        if current and (r.get('gear') or []) != (current[-1].get('gear') or []):flush()
        current.append(r)
    flush()
    return pieces


def _axial_rows(name, entry, prop, throttle, pitch, mass, height):
    """The engine rows of one aircraft and setting: (row(height) -> at(speed_kmh, ordered=False) -> a solved row dict
    (thrust_kgf, fuel_kgh, power_hp, efficiency, gear, regime) or an unsolved one, the chart, mixer, has_efficiency).
    Propeller rows: the compiled propeller equations solved from the nearest solved speed of the row (ordered: speeds
    come in increasing order, as an exact slice asks for them), else the EM plotter's axial solve (18 ms and more
    against 0.05): an exact slice tries it at once, a surface row not at all (SURFACE_THOROUGH): it continues from its
    solved neighbour when asked `again`. Jets and rockets: the chart's engine force.
    Altitude and speed limits as the sweeps apply them."""
    import copy
    from control_mixer import density_at_height
    propulsion = entry['propulsion']
    base_chart = solver(name, height, throttle, pitch, mass)
    levers = mixture_levers(base_chart) if prop else []
    engines = base_chart.engine.properties['engines'] if prop else []
    has_efficiency = prop and not any(e['family'] == 2 for e in engines)
    pistons = [i for i, e in enumerate(engines) if e['family'] in (0, 1)]
    mixer = None
    if pistons:
        i0 = pistons[0];ep0 = engines[i0]['properties']
        if i0 < len(levers) and levers[i0] and ep0.get('mixer_type') in (1, 2):
            mixer = dict(type=int(ep0['mixer_type']), ram=float(ep0['ram_recovery']), scale=float(ep0['mixer_scale']))
    setup = _prop_fast_setup(base_chart) if prop else None
    direct_mains = [] if prop else main_units(base_chart)
    solved_rows = {}   # altitude -> {speed: the solved propeller states of that row, to start a neighbouring one from}

    def propeller_row_at(height):
        view = copy.copy(base_chart)
        view.config = dict(base_chart.config, altitude_m=float(height))
        view.engine = copy.copy(base_chart.engine);view.engine.config = view.config
        exceeds_limit = speed_limit_check(view)
        pm = _prop_model(setup, view) if setup is not None else None
        seen, warm_by_v, fb_chart = {}, solved_rows.setdefault(height, {}), []
        nearest, order, orders = [], sorted(warm_by_v), {}
        def beside(speeds, v_kmh):
            """The speed of a sorted list nearest to v_kmh within 32 km/h (the lower of two as near), or None."""
            j = bisect_left(speeds, v_kmh);best = None
            for u in speeds[max(0, j - 1):j + 1]:
                if abs(u - v_kmh) <= 32. and (best is None or abs(u - v_kmh) < abs(best - v_kmh)):best = u
            return best
        def from_row(v_kmh):
            """The solved state nearest in speed (within 32 km/h) of the nearest altitude that has one: a row is the
            row next to it, shifted a little, so its states start this one's far better than a state at rest does."""
            if not nearest:nearest.append(sorted((h for h in solved_rows if h != height and solved_rows[h]), key=lambda h:abs(h - height))[:2])
            for h in nearest[0]:
                if h not in orders:orders[h] = sorted(solved_rows[h])
                near = beside(orders[h], v_kmh)
                if near is not None:return solved_rows[h][near]
            return None
        def get_fb_chart():
            if not fb_chart:
                import em_march_native
                fb = copy.copy(view)
                fb.engine = view.engine.with_controls(None);fb.engine.fixed_controls = None
                fb._stationary_seeds = OrderedDict();fb._stationary_predictors = OrderedDict()
                em_march_native.detach(fb);em_march_native.attach(fb)
                fb_chart.append(fb)
            return fb_chart[0]
        tried = {}   # speed without a result -> the solved speeds its solves were started from (None: the nearest row, or rest)
        thorough = set()   # speeds without a result that also had the EM plotter's solve
        def far():
            """Whether no solved row is near this one (its first states then come from several starts)."""
            if not nearest:from_row(0.)
            return not nearest[0] or abs(nearest[0][0] - height) > DISCOVER_M
        def at(v_kmh, ordered=False, again=False, start=None):
            if start is not None:
                # The state at the solved speed `start` continued to this one: the result here (returned) when this
                # speed has none, or has a coexisting state of less thrust; else None.
                # (start may also be the states of another altitude's point at this speed: continued along altitude)
                warm = start if isinstance(start, dict) else warm_by_v.get(start)
                if pm is None or not warm or exceeds_limit(v_kmh):return None
                val, w_out = _solve_prop_fast(view, pm, setup, v_kmh, warm, allow_fallback=False, cold=False)
                present = seen.get(v_kmh)
                if val is None or not w_out or not _more_thrust(val, present if present and present['solved'] else None):return None
                if v_kmh not in warm_by_v:insort(order, v_kmh)
                warm_by_v[v_kmh] = w_out
                seen[v_kmh] = dict(val, altitude_m=height, speed_kmh=v_kmh, solved=True)
                return seen[v_kmh]
            starts = None
            if v_kmh in seen:
                # again: an unsolved speed is solved once more from the solved speeds near it (nearest first, two at
                # most) that have not started it yet: its other neighbour, once that is solved, reaches a state the
                # first could not. With SURFACE_THOROUGH also by the EM plotter's solve where it has not had that.
                if not again or seen[v_kmh]['solved'] or pm is None or exceeds_limit(v_kmh):return seen[v_kmh]
                starts = sorted((u for u in warm_by_v if abs(u - v_kmh) <= 32. and u not in tried[v_kmh]), key=lambda u:(abs(u - v_kmh), u))[:2]
                if not starts and (v_kmh in thorough or not SURFACE_THOROUGH):return seen[v_kmh]
            if exceeds_limit(v_kmh):
                seen[v_kmh] = dict(altitude_m=height, speed_kmh=v_kmh, solved=False);return seen[v_kmh]
            if pm is not None:
                if starts is None:
                    if ordered:
                        near_v = next(reversed(warm_by_v), None)
                        if near_v is not None and v_kmh - near_v > 32.:near_v = None
                    else:near_v = beside(order, v_kmh)
                    starts = [near_v]
                val, w_out = None, {}
                for n, near_v in enumerate(starts or [None]):
                    tried.setdefault(v_kmh, set()).add(near_v)
                    val, w_out = _solve_prop_fast(view, pm, setup, v_kmh, warm_by_v[near_v] if near_v is not None else None if ordered else from_row(v_kmh),
                                                  allow_fallback=('always' if again and SURFACE_THOROUGH and n == len(starts or [None]) - 1 else ordered),
                                                  fallback_chart=get_fb_chart)
                    if val is not None:break
                if starts == [None] and not ordered and not again and far():
                    # No solved speed near in this row and no solved row near: the states other starts lead to as well
                    # (from rest, where the start above was another row's state, and the EM plotter's alternate seeds);
                    # of those found, the one with more thrust.
                    for other in ([None] if from_row(v_kmh) is not None else []) + list(setup['alternates']):
                        val2, w2 = _solve_prop_fast(view, pm, setup, v_kmh, {}, allow_fallback=False, seed=other)
                        if val2 is not None and w2 and _more_thrust(val2, val):val, w_out = val2, w2
                if (again and SURFACE_THOROUGH) or (ordered and v_kmh >= SLOW_START_KMH):thorough.add(v_kmh)
                if w_out:
                    if v_kmh not in warm_by_v:insort(order, v_kmh)
                    warm_by_v[v_kmh] = w_out
            else:
                try:val = propeller_row(get_fb_chart(), v_kmh, None)
                except (ValueError, ArithmeticError, np.linalg.LinAlgError):val = None
            seen[v_kmh] = dict(val or {}, altitude_m=height, speed_kmh=v_kmh, solved=val is not None)
            return seen[v_kmh]
        at.model, at.view, at.states, at.exceeds, at.setup = pm, view, warm_by_v, exceeds_limit, setup   # for the exact curves and clicked points
        return at

    table_speeds = [u.jet['speed'] for u in direct_mains if isinstance(getattr(u, 'jet', None), dict) and u.jet.get('speed')]

    def direct_row_at(height):
        exceeds_limit, seen = [None], {}
        def set_altitude():
            # One chart serves every altitude: its air density and caches are set for the row being asked.
            if base_chart.config['altitude_m'] == float(height) and exceeds_limit[0] is not None:return
            base_chart.config['altitude_m'] = float(height)
            rho = density_at_height(f32(height))
            for u in base_chart.engine.units:
                u.rho = rho;u.vectors.cache_clear()
            base_chart.engine.vectors.cache_clear()
            exceeds_limit[0] = speed_limit_check(base_chart)
        def at(v_kmh, ordered=False, again=False, start=None):
            if start is not None:return None   # one state per point: nothing to prefer
            if v_kmh in seen:return seen[v_kmh]
            set_altitude()
            if exceeds_limit[0](v_kmh):
                seen[v_kmh] = dict(altitude_m=height, speed_kmh=v_kmh, solved=False);return seen[v_kmh]
            v = v_kmh / 3.6
            try:
                if propulsion == 'jet':fuel = sum(3600. * u.nominal(v)['scalar']['consumption'] for u in direct_mains)
                else:fuel = sum(3600. * u.consumption for u in base_chart.engine.units)
                val = dict(thrust_kgf=float(base_chart.engine.vectors(v)[0][0]) / KGF, fuel_kgh=fuel, regime=(),
                           # each engine's cell of its thrust table along speed: a corner where it changes
                           soft=tuple(bisect_right(nodes, float(f32(v))) for nodes in table_speeds))
            except (ValueError, ArithmeticError, np.linalg.LinAlgError):val = None
            seen[v_kmh] = dict(val or {}, altitude_m=height, speed_kmh=v_kmh, solved=val is not None)
            return seen[v_kmh]
        return at

    return propeller_row_at if prop else direct_row_at, base_chart, mixer, has_efficiency


def _axial_slice(name, entry, prop, throttle, pitch, mass, heights, speeds):
    """Exact engine rows at every given altitude and speed (each point solved)."""
    row_at, base_chart, mixer, has_efficiency = _axial_rows(name, entry, prop, throttle, pitch, mass, heights[0])
    rows = []
    for h in heights:
        at = row_at(h)
        rows.append(dict(h=float(h), pieces=_pack_axial_pieces((at(v, True) for v in speeds if v >= (5. if prop else 0.)), prop, has_efficiency)))
    return dict(rows=rows, mixer=mixer), base_chart


def _level_charts(name, throttle, pitch, mass, height, lean=False):
    """(base chart, chart_at(height) -> (chart, solve)): the level-flight point solver of one aircraft and setting at
    any altitude. The charts share the aircraft data; each altitude has its own atmosphere, engine, compiled objects
    and solver caches. lean: the point's row is left as the solver made it (the surface reads four of its fields;
    normalizing every field for export took most of a point's time)."""
    import copy
    import em_march_native
    base = level_solver(name, height, throttle, pitch, mass)
    def chart_at(h):
        if float(h) == base.config['altitude_m']:chart = base
        else:
            chart = copy.copy(base)
            chart.config = dict(base.config, altitude_m=float(h))
            chart.settings_echo = dict(base.settings_echo, altitude_m=float(h))
            chart.instructor_boundaries = {}
            chart.instructor_trim_entries = {}
            chart._sideslip_solvers = {}
            if chart.is_prop:
                chart.engine = base.engine.with_controls(None)
                chart.engine.config = chart.config
            else:
                chart.engine = type(base.engine)(chart.model, chart.mass, chart.config)
            em_march_native.detach(chart)
            em_march_native.attach(chart)
        if lean:
            from em_stationary_surface import chart_point
            def sample(result, tolerance=chart.config['sep_tolerance_mps']):
                row = chart_point(result, tolerance)
                row['valid'] = row['chart_usable']
                return row
            chart.sample = sample
        native = em_march_native.engine(chart)
        return chart, native.get_point_solver().solve_row if native is not None else chart.solve
    return base, chart_at


def _level_slice(name, throttle, pitch, mass, heights, speeds):
    """Exact level-flight rows at every given altitude and speed (each point solved, seeded from the one before)."""
    rows, near = [], None
    chart, chart_at = _level_charts(name, throttle, pitch, mass, heights[0])
    for height in heights:
        chart, solve = chart_at(height)
        exceeds_limit = speed_limit_check(chart)
        pieces, piece = [], None
        for speed in speeds:
            row, seed = None, None
            if speed > 0 and not exceeds_limit(speed):
                try:row, seed = level_row(chart, speed, near, solve)
                except (ArithmeticError, np.linalg.LinAlgError):pass
            if row is None:
                piece = None
                continue
            near = seed
            if piece is None:
                piece = dict(x=[], sep=[], gear=[])
                pieces.append(piece)
            piece['x'].append(speed)
            piece['sep'].append(round(row['ps_mps'], 3))
        rows.append(dict(h=height, pieces=pieces))
    return dict(rows=rows, mixer=None), chart


LEVEL_QUICK_END = True
# Whether level rows name their blade corners for the row's knots to close in on, as the engine rows do. Off: SEP showed
# no error at those corners that it removed, and every extra knot is a level-flight solve (1 to 3 ms).
LEVEL_SOFT = False


def _no_guess(*args, **kwargs):
    raise ValueError('no trim guess for the step past a curve end')


# The curves of a slice surface (engine_surface), with the decimals the page gets them in.
SURFACE_SECONDS = 30.   # after this long a surface's remaining intervals stay as they are (stats: unchecked)
ENGINE_CURVES = (('thrust', 'thrust_kgf', 3), ('fuel', 'fuel_kgh', 3), ('power', 'power_hp', 3), ('efficiency', 'efficiency', 4))
LEVEL_CURVES = (('sep', 'ps_mps', 4),)


def _surface_solver(name, entry, prop, throttle, pitch, mass, level, h_lo):
    """The point solver of a slice surface's rows: (solver_at(h) -> f(x, again=False, start=None) -> None or (regime,
    the curves' values) at altitude h and speed x km/h (engine_surface.RowBuilder), curves, chart, mixer, the level
    rows' seeds and thrusts by altitude). Engine curves at angle of attack 0, or SEP in 1 g level flight."""
    seeds, thrusts = {}, {}
    if level:
        chart, chart_at = _level_charts(name, throttle, pitch, mass, h_lo, lean=True)
        curves, mixer = LEVEL_CURVES, None
        setup = _prop_fast_setup(chart) if prop else None
        if setup is not None and not hasattr(setup['s2'].PropModel, 'test_regime'):setup = None
        def solver_at(h):
            row_chart, solve = chart_at(h)
            pm = _prop_model(setup, row_chart) if setup is not None else None
            def regime_of(p, state, propulsion):
                # The engines' branches from the compiled model at the solved state (as the engine rows have them).
                from body_dynamics import G
                words, coords = None, p.get('stationary_coordinates')
                if pm is not None and coords is not None and len(coords) >= len(setup['active']):
                    words = pm.test_regime(p['speed_kmh'] / 3.6, 1., p['solution'][0], p['solution'][1], float(G), float(row_chart.dt),
                                           list(coords)[len(coords) - len(setup['active']):], list(p.get('stationary_request_indices') or ()))
                engines = state['engines'] if words is None else [dict(s, regime=w[0], fuel_regime=w[1]) for s, w in zip(state['engines'], words[0])]
                soft[0] = None if words is None else tuple(words[1])
                return regime(row_chart.engine.properties['engines'], engines, (propulsion.get('stationarity') or {}).get('governor_status'), (), fuel=False)
            soft = [None]   # the blade sections' polar branches of the point `regime_of` was last asked for
            def result(row):
                """What the row builder gets of a solved level point: its regime, its SEP and, where known, its corners' names."""
                found, soft[0] = soft[0], None
                return (row['regime'], (row['ps_mps'],)) if found is None or not LEVEL_SOFT else (row['regime'], (row['ps_mps'],), found)
            exceeds_limit, own = speed_limit_check(row_chart), seeds.setdefault(h, {})
            others = sorted((u for u in seeds if u != h and seeds[u]), key=lambda u:abs(u - h))[:2]
            def near(x):
                # The solved point to start from: the nearest of this row, or of the two nearest rows where that one is
                # nearer in state (a row 100 m away counts as 10 km/h away: its state at the same speed is almost
                # this row's, and the solve from it takes fewer steps than from a point 50 km/h along the row).
                best, cost = None, math.inf
                if own:
                    u = min(own, key=lambda u:abs(u - x))
                    best, cost = own[u], abs(u - x)
                for other in others:
                    if abs(other - h) * .1 >= cost:continue
                    pool = seeds[other];u = min(pool, key=lambda u:abs(u - x))
                    if abs(u - x) + abs(other - h) * .1 < cost:best, cost = pool[u], abs(u - x) + abs(other - h) * .1
                return best
            started = {}   # speed without a result -> how far away in this row the solve that failed was started
            present = thrusts.setdefault(h, {})
            def f(x, again=False, start=None):
                if exceeds_limit(x):return None
                if start is not None:
                    # The state at the solved speed `start` continued to this one: see _axial_rows.
                    if start not in own:return None
                    try:row, seed = level_row(row_chart, x, own[start], solve, regime_of)
                    except (ArithmeticError, np.linalg.LinAlgError):return None
                    if row is None or not _more_thrust(row, dict(thrust_kgf=present[x]) if x in present and x in own else None):return None
                    own[x] = seed;present[x] = row['thrust_kgf']
                    return result(row)
                distance = min((abs(u - x) for u in own), default=math.inf)
                if again and distance >= started.get(x, math.inf):return None
                if not own and not others and seeds:others.extend(sorted((u for u in seeds if u != h and seeds[u]), key=lambda u:abs(u - h))[:2])
                # Below every solved speed of the row and next to one (the step past the stall end): only the solve
                # continued from that neighbour. The point solver's further starts (its kept seeds, the trim guess)
                # take 6 ms to fail there against 2, and a state they found would not continue the curve.
                last_step = LEVEL_QUICK_END and bool(own) and x < min(own) and distance <= 2
                if last_step:
                    getattr(row_chart, '_stationary_seeds', {}).clear()
                    row_chart.initial_guess = _no_guess
                try:row, seed = level_row(row_chart, x, near(x), solve, regime_of)
                except (ArithmeticError, np.linalg.LinAlgError):row = None
                finally:
                    if last_step:del row_chart.initial_guess
                if row is None:
                    started[x] = distance
                    return None
                own[x] = seed;present[x] = row['thrust_kgf']
                return result(row)
            return f
    else:
        row_at, chart, mixer, has_efficiency = _axial_rows(name, entry, prop, throttle, pitch, mass, h_lo)
        curves = ENGINE_CURVES[:4 if has_efficiency else 3 if prop else 2]
        def solver_at(h):
            at = row_at(h)
            def f(x, again=False, start=None):
                r = at(x, again=again, start=start)
                if r is None or not r['solved']:return None
                if 'soft' in r:return r['regime'], tuple(r.get(key) for _, key, _ in curves), r['soft']
                return r['regime'], tuple(r.get(key) for _, key, _ in curves)
            return f
    # Corners along altitude: a turbine's thrust table is straight between its air densities, and the game's atmosphere
    # changes law at 18,300 m. A row's regime names its cells, so rows either side of such an altitude are never blended
    # (the surface finds the altitude as it finds any change of layout).
    from control_mixer import density_at_height
    if prop:tables = [e['turbine']['density'] for e in chart.engine.properties['engines'] if e.get('turbine') and e['turbine'].get('density')]
    else:tables = [u.jet['density'] for u in main_units(chart) if isinstance(getattr(u, 'jet', None), dict) and u.jet.get('density')]
    rows_at = solver_at
    def cells_at(h):
        rho = float(density_at_height(f32(h)))
        return (h > 18300,) + tuple(sum(rho < d for d in nodes) for nodes in tables)
    def solver_at(h):
        f, cells = rows_at(h), cells_at(h)
        def g(x, again=False, start=None):
            r = f(x, again=again, start=start)
            return None if r is None else ((r[0], cells),) + tuple(r[1:])
        return g
    solver_at.cells = cells_at
    return solver_at, curves, chart, mixer, seeds, thrusts


def _checked_surface(name, entry, prop, throttle, pitch, mass, level, h_lo, h_hi, v_lo, v_hi):
    """The slice surface over whole metres and whole km/h of the ranges (engine_surface: solved rows, checked against
    solved middle rows until the curves drawn between them are within the tolerance): engine curves at angle of attack
    0, or SEP in 1 g level flight. Returns (surface for the page, chart)."""
    import engine_surface
    h_lo, h_hi = math.ceil(h_lo), math.floor(h_hi)
    x_lo, x_hi = math.ceil(max(v_lo, 5. if prop else 1. if level else 0.)), math.floor(v_hi)
    solver_at, curves, chart, mixer, seeds, thrusts = _surface_solver(name, entry, prop, throttle, pitch, mass, level, h_lo)
    started = time.perf_counter()
    stats, rows = dict(rows=0, points=0), []
    if x_hi >= x_lo and h_hi >= h_lo:
        # Each curve's tolerance: TOLERANCE of its range over a first lattice of the surface (at least 2% of its size).
        lattice = [r[1] for h in sorted({h_lo + round((h_hi - h_lo) * i / 4) for i in range(5)})
                   for f in [solver_at(h)] for x in sorted({x_lo + round((x_hi - x_lo) * i / 12) for i in range(13)})
                   for r in [f(x)] if r is not None]
        # A propeller aircraft's engine curves are not held finer than 2% of what its engines are rated for: at idle
        # the curves' own ranges are next to nothing (a fraction of a horsepower), far below what the point solver's
        # convergence settles, and no number of rows would reproduce that. The efficiency, a quotient, takes its range
        # from 0-100% however far its values run where the power is next to nothing, and a point's efficiency may be
        # off besides by what those floors make of it there (`allow`).
        labels, floors, allow = [label for label, _, _ in curves], {}, None
        if prop and not level:
            engines = chart.engine.properties['engines']
            rated = sum(e['properties'].get('base_hp') or 0. for e in engines if e['family'] in (0, 1))
            if not rated > 0.:rated = max((abs(v[labels.index('power')]) for v in lattice if v[labels.index('power')] is not None), default=0.)
            floors = dict(thrust=.02 * rated * GAME_HP / 50. / KGF, fuel=.02 * rated * .25, power=.02 * rated, efficiency=2.)
        scales = []
        for k, label in enumerate(labels):
            g = [v[k] for v in lattice if v[k] is not None and math.isfinite(v[k])]
            size = max(max(g) - min(g), .02 * max(map(abs, g)), floors.get(label, 0.)) if g else 0.
            scales.append(engine_surface.TOLERANCE * (min(size, 100.) if label == 'efficiency' else size))
        if 'efficiency' in labels and floors:
            kt, kp, ke = labels.index('thrust'), labels.index('power'), labels.index('efficiency')
            ft, fp = engine_surface.TOLERANCE * floors['thrust'], engine_surface.TOLERANCE * floors['power']
            def allow(values):
                t, p, e = values[kt], values[kp], values[ke]
                if e is None or not t or not p:return None
                out = [0.] * len(values);out[ke] = abs(e) * (ft / abs(t) + fp / abs(p))
                return out
        seeds.clear();thrusts.clear()
        # The two metres either side of every altitude where the cells change (found on the cells alone, no solving).
        also, spans = [], [(h_lo, h_hi)]
        while spans:
            a, b = spans.pop()
            if solver_at.cells(a) == solver_at.cells(b):continue
            if b - a == 1:also += [a, b]
            else:spans += [(a, (a + b) // 2), ((a + b) // 2, b)]
        rows, stats = engine_surface.build_surface(solver_at, h_lo, h_hi, x_lo, x_hi, scales,
                                                   cancelled=lambda:time.perf_counter() - started > SURFACE_SECONDS,
                                                   soft=SLOW_START_KMH if prop and not level else None, allow=allow, also=also)
        stats['tolerance'] = scales
    regimes, packed = {}, []
    pistons = [i for i, e in enumerate(chart.engine.properties['engines']) if e['family'] in (0, 1)] if prop else []
    for row in rows:
        pieces = []
        for p in row.pieces:
            piece = dict(s=regimes.setdefault(p.sig, len(regimes)), x=p.x,
                         gear=[p.sig[0][0][i][0] + 1 for i in pistons] if prop and p.sig and p.sig[0] else [])
            for (label, _, digits), ys in zip(curves, p.y):
                piece[label] = [None if y is None else round(float(y), digits) for y in ys]
            pieces.append(piece)
        packed.append(dict(h=row.h, pieces=pieces, **(dict(m=row.modes) if row.modes and any(map(any, row.modes)) else {})))
    if level:
        _remember_seeds((name, throttle, pitch, mass), [(float(h), float(x), seed) for h, own in seeds.items()
                                                        for x, seed in list(own.items())[::4]])
        chart = solver(name, h_lo, throttle, pitch, mass)
    stats['seconds'] = time.perf_counter() - started
    return dict(altitude_range=[h_lo, h_hi], speed_range=[x_lo, x_hi], curves=[label for label, _, _ in curves], rows=packed,
                mixer=mixer, mixture_status=('ok' if mixer else 'no_lever') if prop and not level else 'none', checked=True,
                stats=stats), chart


@lru_cache(maxsize=96)
def compute_surface(name, throttle=110., pitch=None, level=False, mass='{}', altitudes=None, speeds=None, slice_spec=None):
    """The slice surface over altitude and true airspeed (engine curves, or 1 g level-flight SEP) the page draws its
    slider curves from; slice_spec (axis, fixed, start, count): instead the exact curve of one slice, every point
    solved (the reference the surface is checked against)."""
    entry, prop = admitted(name, throttle, pitch)
    started = time.perf_counter()
    h_lo, h_hi = altitudes or ALTITUDES
    v_lo, v_hi = speeds or SPEEDS.get(entry['propulsion'], (0., 800.))
    with LOCK:
        if slice_spec is not None:
            axis, fixed, start, count = slice_spec
            xs = range(start, start + count)
            heights, velocities = ((fixed,), xs) if axis == 'speed' else (xs, (fixed,))
            if level:surface, chart = _level_slice(name, throttle, pitch, mass, heights, velocities)
            else:surface, chart = _axial_slice(name, entry, prop, throttle, pitch, mass, heights, velocities)
            surface.update(axis=axis, fixed=fixed, step=1, altitude_range=[h_lo, h_hi], speed_range=[v_lo, v_hi])
        else:surface, chart = _checked_surface(name, entry, prop, throttle, pitch, mass, level, h_lo, h_hi, v_lo, v_hi)
    engines = chart.engine.properties['engines'] if prop else []
    mode = dict(key='throttle', label=f'{throttle:g}%', level=bool(level), surface=surface)
    return dict(aircraft_id=name, name=entry['name'], propulsion=entry['propulsion'], altitude_m=h_lo,
                speed_kmh=v_lo, throttle_percent=throttle, pitch_percent=pitch, modes=[mode],
                engine_count=len(engines) if prop else len(main_units(chart)), mass_kg=float(chart.mass['mass']),
                stages=[len(e['properties']['stages']) for e in engines if e['family'] in (0, 1)],
                rpm_limit=([e['properties']['omega_limit'] * 30 / math.pi for e in engines] if prop else
                           [u.ep['omega_limit'] * 30 / math.pi for u in main_units(chart)] if entry['propulsion'] == 'jet' else []),
                elapsed_s=time.perf_counter() - started)


# Whether a propeller aircraft's engine curve over speed is solved at every whole km/h by the compiled propulsion loop
# (`_exact_speed_rows`) instead of the EM plotter's solve at a base grid and refinement points (Leo, 2026-11-19: the
# deployed curves accurate in every case and not solved in Python).
EXACT_CURVES = True


def _summary_row(at, r, v, report, levers):
    """`_propeller_fields` of a state's full record, from the compiled loop's summary of the state instead (the same
    expressions on the same numbers; the propeller tests are the loop's own). Not for aircraft with turbines: their
    turbine thrust and TSFC need the full record (None). What does not change along a row (the aircraft's engines and
    propellers, the altitude) is worked out once and kept with the row solver."""
    from propulsion_general import fixed_air
    summary = r['summary']
    if len(summary) < 8:return None
    ctx = getattr(at, 'pack', None)
    if ctx is None:
        chart, p = at.view, at.view.engine.properties
        families = [e['family'] for e in p['engines']]
        ctx = at.pack = dict(chart=chart, turbines=any(f in (2, 5) for f in families), altitude=chart.config['altitude_m'],
                             pistons=[i for i, f in enumerate(families) if f in (0, 1)], rockets=[i for i, f in enumerate(families) if f == 3],
                             properties=[e['properties'] for e in p['engines']], propellers=len(p['propellers']),
                             linked=[link['index'] for shaft in p['transmissions'] for link in shaft['propellers']])
    if ctx['turbines']:return None
    converged, force, status, engines, props, shafts = summary[:6]
    chart, pistons, rockets, altitude = ctx['chart'], ctx['pistons'], ctx['rockets'], ctx['altitude']
    # At angle of attack 0 the flight velocity is along the body axis (em_operating.flight_condition gives (v, 0, 0)).
    velocity = (f32(v), 0., 0.)
    air, sound = fixed_air(velocity, altitude)
    if getattr(at, 'controls', None) is None:
        from body_dynamics import G
        br, q = r['solved_at']
        at.controls = at.model.test_record(v, 1., 0., 0., float(G), float(chart.dt), q, br)[0]['controls']
    powers = [e[1] * e[0] for e in engines]
    power = sum(powers)
    rpm = [e[0] * 30 / math.pi for e in engines];pitch_deg = [math.degrees(q[0]) for q in props]
    point = dict(propulsion=dict(controls=at.controls, stationarity=dict(governor_status=status, shaft_target_rpm=[t * 60 / (2 * math.pi) for t in summary[7]]),
                                 engine_rpm=rpm, propeller_pitch_deg=pitch_deg))
    if report is not None:report(point, chart)
    thrust = float(force[0])
    fuel = [3600. * e[2] for e in engines]
    booster = sum(float(engines[i][6]) for i in rockets) if rockets else 0
    piston_w = sum(powers[i] for i in pistons)
    properties = ctx['properties']
    mixtures = [mixture_band(properties[i], levers[i] if i < len(levers) else False, velocity[0], altitude) for i in pistons]
    tips = [None] * ctx['propellers']
    for index in ctx['linked']:tips[index] = props[index][6]
    return dict(thrust_kgf=thrust / KGF, power_hp=power / GAME_HP if power > 0. or pistons else None,
                engine_power_hp=[w / GAME_HP for w in powers],
                fuel_kgh=sum(f for i, f in enumerate(fuel) if i not in rockets) if rockets else sum(fuel), engine_fuel_kgh=fuel,
                **(dict(booster_thrust_kgf=booster / KGF, booster_fuel_kgh=sum(fuel[i] for i in rockets)) if booster else {}),
                piston_power_hp=piston_w / GAME_HP if pistons else None,
                bsfc=sum(fuel[i] for i in pistons) / (piston_w / GAME_HP) if pistons and piston_w > 0. else None,
                turbine_thrust_kgf=None, tsfc=None,
                efficiency=(thrust - booster) * v / power * 100. if power > 1. else None,
                rpm=rpm, pitch_deg=pitch_deg,
                pitch_percent=point['propulsion'].get('propeller_pitch_percent'),
                manifold_ata=[float(engines[i][7]) for i in pistons],
                gear=[int(engines[i][3]) + 1 for i in pistons],
                governor=list(status or ()), tip_mach=tips,
                regime=(tuple((int(e[3]), e[8], e[9]) for e in engines), tuple(status or ()), ()),
                mixture_lean_percent=[m and m[0] for m in mixtures], mixture_rich_percent=[m and m[1] for m in mixtures],
                mixture_power_percent=[m and m[2] for m in mixtures],
                mixture_upper_lean_percent=[m and m[3] and m[3][0] for m in mixtures],
                mixture_upper_rich_percent=[m and m[3] and m[3][1] for m in mixtures],
                mach=float(air['mach']), ias_kmh=float(air['ias_u']) * 3.6)


SUMMARY_ROWS = True   # whether a settled row's page rows come from the compiled loop's summaries (`_summary_row`)


def _full_row(at, height, speed_kmh, report, levers):
    """The page's row of a solved point of `at` (an altitude's row solver): the full record of its state, through
    `_propeller_fields`, as `propeller_row` gives it; an unsolved row where there is no state."""
    from body_dynamics import G
    r, value = at(speed_kmh), None
    if SUMMARY_ROWS and r['solved'] and 'summary' in r and 'solved_at' in r:
        value = _summary_row(at, r, speed_kmh / 3.6, report, levers)
        if value is not None:return dict(value, altitude_m=float(height), speed_kmh=float(speed_kmh), solved=True)
    if r['solved'] and 'solved_at' in r:
        br, q = r['solved_at']
        full = at.model.test_record(speed_kmh / 3.6, 1., 0., 0., float(G), float(at.view.dt), q, br)
        value = _propeller_fields(at.view, speed_kmh / 3.6, full[0], report, levers) if full is not None else None
    return dict(value or {}, altitude_m=float(height), speed_kmh=float(speed_kmh), solved=value is not None)


ANCHORS, ANCHOR_REACH = 5, 150   # altitudes of an altitude curve whose rows are settled around its speed; how far (km/h)
# Whether a settled row comes from the compiled module's own loop (PropModel.axial_row: the same solves in the same
# order, without Python between them) where the build has it and every shaft has one request.
NATIVE_ROWS = True


class _SettledRow:
    """An altitude's settled row from the compiled loop, offering what the Python row solver's `at` offers the curves
    and the clicked points: at(speed) -> dict(solved, solved_at, thrust_kgf), model, view, states."""

    def __init__(self, at, lo, results):
        self.at, self.model, self.view, self.lo = at, at.model, at.view, lo
        nothing = (0,) * len(at.view.engine.properties['transmissions'])
        self.rows, self.states = [], {}
        for i, found in enumerate(results):
            if found is None:
                self.rows.append(dict(solved=False))
                continue
            kind, (q, r, jac, summary), br, states = found
            self.rows.append(dict(solved=True, solved_at=(br or nothing, q), thrust_kgf=summary[1][0] / KGF, summary=summary))
            if kind == 2:self.states[lo + i] = {b:(coords, j) for b, coords, j in states}

    def __call__(self, x, again=False, start=None):
        i = x - self.lo
        if start is None and float(i).is_integer() and 0 <= i < len(self.rows):return self.rows[int(i)]
        return self.at(x, again=again, start=start)


EXACT_ROWS = OrderedDict()   # (aircraft, throttle, pitch, mass, altitude, lo, hi) -> that altitude's solved row


def _exact_row(name, entry, throttle, pitch, mass, height, lo, hi):
    """An altitude's row solver with every whole km/h lo..hi solved and settled: the compiled loop solves each speed
    from its solved neighbour; then every state is continued to the next speed down and up, and replaces what that
    speed has where it is the state with more thrust (two propeller states coexist over bands of speed, and a solve
    finds the one its start leads to), until nothing changes. None where the compiled loop is not available."""
    key = (name, throttle, pitch, mass, height, lo, hi)
    at = EXACT_ROWS.get(key)
    if at is None:
        at = _axial_rows(name, entry, True, throttle, pitch, mass, height)[0](height)
        if getattr(at, 'model', None) is None:return None
        results = None
        if NATIVE_ROWS and hasattr(at.model, 'axial_row') and not BLADE_REGIMES:
            from body_dynamics import G
            results = at.model.axial_row(float(G), float(at.view.dt), lo, hi, [not at.exceeds(x) for x in range(lo, hi + 1)])
        if results is not None:
            at = _SettledRow(at, lo, results)
            EXACT_ROWS[key] = at
            while len(EXACT_ROWS) > 8:EXACT_ROWS.popitem(last=False)
            return at
        for x in range(lo, hi + 1):at(x)
        for _ in range(8):
            changed = False
            for order, step in ((range(hi - 1, lo - 1, -1), 1), (range(lo + 1, hi + 1), -1)):
                for x in order:
                    if not at(x + step)['solved']:continue
                    if not at(x)['solved'] and at(x, again=True)['solved']:changed = True
                    if at(x, start=x + step) is not None:changed = True
            if not changed:break
        EXACT_ROWS[key] = at
        while len(EXACT_ROWS) > 8:EXACT_ROWS.popitem(last=False)
    EXACT_ROWS.move_to_end(key)
    return at


def _exact_speed_rows(name, entry, throttle, pitch, mass, height, lo, hi, report):
    """A propeller aircraft's engine curve at one altitude: a row for every whole km/h lo..hi, each solved (`_exact_row`;
    nothing between two speeds is interpolated). None where the compiled loop is not available: the caller solves the
    curve as before."""
    at = _exact_row(name, entry, throttle, pitch, mass, height, lo, hi)
    if at is None:return None
    levers = mixture_levers(at.view)
    return [_full_row(at, height, x, report, levers) for x in range(lo, hi + 1)]


def _exact_path_py(at, altitudes, speeds, allowed, anchors=(), retry=1):
    """PropModel.axial_path in Python, the loop it was written from (for checks, and for a build without it): the same
    solves in the same order through `_solve_prop_fast`. Returns for each point None or (2 or 3, the point's fields,
    its states for a neighbour to start from or None)."""
    pm, view, setup, count = at.model, at.view, at.setup, len(speeds)
    home = float(view.config['altitude_m'])
    val, warm, tried = [None] * count, [None] * count, [set() for _ in range(count)]
    last_cold, skipped = -1000000, []
    def solve(i, states, cold=True, seed=None):
        pm.set_altitude(float(altitudes[i]))
        return _solve_prop_fast(view, pm, setup, speeds[i], states or {}, allow_fallback=False, seed=seed, cold=cold)
    def near(i, skip=()):
        for d in range(1, 33):
            for j in (i - d, i + d):
                if 0 <= j < count and warm[j] and j not in skip:yield j
    try:
        for i in range(count):
            if not allowed[i]:continue
            u = next(near(i), None);tried[i].add(u)
            if u is None and i - last_cold < retry:
                skipped.append(i)
                continue
            v, w = solve(i, warm[u] if u is not None else None)
            if u is None:
                for other in setup['alternates'][:4]:
                    v2, w2 = solve(i, None, seed=other)
                    if v2 is not None and w2 and _more_thrust(v2, v):v, w = v2, w2
                if v is None:last_cold = i
            val[i], warm[i] = v, w or None
        for i, states in anchors:
            if not (0 <= i < count) or not allowed[i] or not states:continue
            v, w = solve(i, {b:(q, j) for b, q, j in states}, cold=False)
            if v is not None and w and _more_thrust(v, val[i]):val[i], warm[i] = v, w
        for outer in range(2):
            for _ in range(8):
                changed = False
                for step in (1, -1):
                    i = count - 2 if step == 1 else 1
                    while 0 <= i < count:
                        u = i + step
                        if val[u] is not None and allowed[i]:
                            if val[i] is None:
                                picks = []
                                for j in near(i, tried[i]):
                                    if len(picks) < 2:picks.append(j);tried[i].add(j)
                                for j in picks:
                                    if val[i] is not None:break
                                    v, w = solve(i, warm[j])
                                    if v is not None:
                                        val[i], warm[i] = v, w or None;changed = True
                            if warm[u]:
                                v, w = solve(i, warm[u], cold=False)
                                if v is not None and w and _more_thrust(v, val[i]):
                                    val[i], warm[i] = v, w;changed = True
                        i -= step
                if not changed:break
            if outer or retry <= 1:break
            found_more = False
            for i in [i for i in skipped if val[i] is None][:2000]:   # the compiled loop's AX_RESCUE
                v, w = solve(i, None)
                for other in setup['alternates'][:4]:
                    v2, w2 = solve(i, None, seed=other)
                    if v2 is not None and w2 and _more_thrust(v2, v):v, w = v2, w2
                if v is not None:
                    val[i], warm[i] = v, w or None;found_more = True
            skipped = []
            if not found_more:break
    finally:pm.set_altitude(home)
    return [None if v is None else (2 if w else 3, v, w) for v, w in zip(val, warm)]


def _allowed_altitudes(base_chart, speed_kmh, heights):
    """For each of the consecutive whole altitudes `heights`: whether speed_kmh is within the aircraft's limits there
    (`speed_limit_check`, 0.7 ms an altitude). Asked every LIMIT_STEP metres and, between two such altitudes with
    different answers, halved to the metre where it changes; between two with the same answer nothing changes (the
    limits are an IAS and a Mach number, which move by a fraction of a km/h in 100 m)."""
    import copy
    known = {}
    def at(i):
        if i not in known:
            view = copy.copy(base_chart);view.config = dict(base_chart.config, altitude_m=float(heights[i]))
            known[i] = not speed_limit_check(view)(speed_kmh)
        return known[i]
    count = len(heights);out = [False] * count
    marks = sorted(set(range(0, count, LIMIT_STEP)) | {count - 1})
    for a, b in zip(marks, marks[1:]):
        lo, hi = a, b
        while hi - lo > 1 and at(lo) != at(hi):
            mid = (lo + hi) // 2
            if at(mid) == at(a):lo = mid
            else:hi = mid
        if at(a) == at(b):out[a:b + 1] = [at(a)] * (b + 1 - a)
        else:
            out[a:lo + 1] = [at(a)] * (lo + 1 - a);out[hi:b + 1] = [at(b)] * (b + 1 - hi)
    if count == 1:out[0] = at(0)
    return out


LIMIT_STEP = 100
COLD_EVERY_M = 32   # along altitude, a start from rest that found no state is not repeated for this many metres (axial_path's retry)
# An altitude curve is solved at every whole metre (Leo, 2026-11-19). The page gets the metres needed to draw every
# solved one within this share of each plotted curve's range by straight lines (a tenth of the sweeps' tolerance), and
# every metre where the curve starts, stops or the engines change gear; a clicked point is solved on its own.
ALTITUDE_SHARE = .00005


def _kept(values, solved, tolerances, breaks):
    """The indices of an every-metre curve the page needs: each solved run's ends, the given breaks, and between them
    the points without which a straight line would miss a solved value by more than its curve's tolerance (checked at
    every metre). values: one list per curve (None where it has no value)."""
    keep = set()
    count = len(solved)
    runs, start = [], None
    for i in range(count + 1):
        if i < count and solved[i]:
            if start is None:start = i
        elif start is not None:
            runs.append((start, i - 1));start = None
    arrays = []
    for column, tol in zip(values, tolerances):
        has = np.array([v is not None for v in column])
        arrays.append((np.array([0. if v is None else v for v in column], dtype=float), has, tol))
    for first, last in runs:
        cuts = sorted({first, last} | {i for i in breaks if first <= i <= last})
        keep.update(cuts)
        stack = list(zip(cuts, cuts[1:]))
        while stack:
            a, b = stack.pop()
            if b - a < 2:continue
            worst, at_worst = 0., None
            t = (np.arange(a + 1, b) - a) / float(b - a)
            for y, has, tol in arrays:
                if tol <= 0. or not (has[a] and has[b]):
                    # a curve without a value at an end of the stretch: where its values begin or stop is a break
                    inner = np.nonzero(has[a + 1:b] != has[a])[0]
                    if len(inner):worst, at_worst = np.inf, a + 1 + int(inner[0])
                    continue
                err = np.abs(y[a + 1:b] - (y[a] + (y[b] - y[a]) * t)) / tol
                err[~has[a + 1:b]] = np.inf
                k = int(np.argmax(err))
                if err[k] > worst:worst, at_worst = float(err[k]), a + 1 + k
            if at_worst is not None and worst > 1.:
                keep.add(at_worst);stack += [(a, at_worst), (at_worst, b)]
    return sorted(keep)


def _exact_altitude_rows(name, entry, throttle, pitch, mass, speed_kmh, xs, report, curves):
    """A propeller aircraft's engine curve at one speed over altitude, solved at every whole metre of the range by
    the compiled loop (PropModel.axial_path: each metre from the one before; at ANCHORS altitudes the state of that
    altitude's settled speed row is given too, since some states are only reached along speed; then states are
    continued down and up under the more-thrust rule until nothing changes). Returns the rows the page needs to draw
    every solved metre (`_kept`). None where the compiled loop is not available: the caller solves the curve as before."""
    import copy
    from body_dynamics import G
    lo, hi = math.ceil(min(xs)), math.floor(max(xs))
    if hi < lo:return None
    heights = list(range(lo, hi + 1))
    row_at, base_chart = _axial_rows(name, entry, True, throttle, pitch, mass, lo)[:2]
    at = row_at(lo)
    if getattr(at, 'model', None) is None or BLADE_REGIMES:return None
    allowed = _allowed_altitudes(base_chart, speed_kmh, heights)
    whole, anchors = int(round(speed_kmh)), []
    for i in sorted({round((len(heights) - 1) * k / (ANCHORS - 1)) for k in range(ANCHORS)} if len(heights) > 1 else {0}):
        row = _exact_row(name, entry, throttle, pitch, mass, heights[i], max(5, whole - ANCHOR_REACH), whole + ANCHOR_REACH)
        states = row.states.get(whole) if row is not None else None
        if states:anchors.append((i, [(b, q, j) for b, (q, j) in states.items()]))
    speeds = [float(speed_kmh)] * len(heights)
    setup = at.setup
    rockets, has_eff, v = setup['rockets'], setup['has_eff'], speed_kmh / 3.6
    found = None
    if NATIVE_ROWS and hasattr(at.model, 'axial_path'):
        # six numbers a metre (state or not, thrust, power, fuel, rocket thrust, gears); a metre's full state on request
        found = at.model.axial_path(float(G), float(at.view.dt), [float(h) for h in heights], speeds, allowed, anchors, False, 1, COLD_EVERY_M)
    if found is not None:
        light, store = np.frombuffer(found[0], dtype=float).reshape(-1, 6), found[1]
        solved = [bool(k) for k in (light[:, 0] >= 2.)]
        thrust = [t / KGF if ok else None for t, ok in zip(light[:, 1], solved)]
        power = [w / GAME_HP if ok else None for w, ok in zip(light[:, 2], solved)]
        fuel = [3600. * f if ok else None for f, ok in zip(light[:, 3], solved)]
        efficiency = [(t - r) * v / w * 100. if ok and w > 1. and has_eff else None for t, r, w, ok in zip(light[:, 1], light[:, 4], light[:, 2], solved)]
        gears = [g if ok else None for g, ok in zip(light[:, 5], solved)]
        def point(i):
            f = store.entry(i)
            return None if f is None else dict(solved=True, solved_at=(f[2] or (0,) * setup['nt'], f[1][0]), thrust_kgf=f[1][3][1][0] / KGF, summary=f[1][3])
    else:
        points = [None if f is None else dict(f[1], solved=True) for f in _exact_path_py(at, heights, speeds, allowed, anchors, COLD_EVERY_M)]
        thrust, power, fuel, efficiency, gears = [], [], [], [], []
        for p in points:
            if p is None or 'summary' not in p:
                for column in (thrust, power, fuel, efficiency, gears):column.append(None)
                continue
            force, engines = p['summary'][1], p['summary'][3]
            w = sum(e[1] * e[0] for e in engines);t = float(force[0])
            booster = sum(float(engines[i][6]) for i in rockets) if rockets else 0.
            thrust.append(t / KGF);power.append(w / GAME_HP)
            fuel.append(sum(3600. * e[2] for i, e in enumerate(engines) if i not in rockets))
            efficiency.append((t - booster) * v / w * 100. if w > 1. and has_eff else None)
            gears.append(tuple(int(e[3]) for e in engines))
        solved = [g is not None for g in gears]
        def point(i):return points[i]
    columns = [thrust, power, fuel, efficiency]
    tolerances = []
    for column in columns:
        g = [x for x in column if x is not None]
        tolerances.append(ALTITUDE_SHARE * max(max(g) - min(g), .02 * max(map(abs, g))) if g else 0.)
    breaks = {i for i in range(1, len(heights)) if solved[i] and solved[i - 1] and gears[i] != gears[i - 1]}
    breaks |= {i - 1 for i in breaks}
    levers, view = mixture_levers(at.view), None
    class _Point:   # what `_full_row` reads of a row solver, for one altitude of the curve
        def __init__(self, i):
            self.i = i;self.model = at.model
            self.view = copy.copy(base_chart);self.view.config = dict(base_chart.config, altitude_m=float(heights[i]))
            self.view.engine = copy.copy(base_chart.engine);self.view.engine.config = self.view.config
            self.controls = controls[0]
            self.found = point(i) or dict(solved=False)
        def __call__(self, x):return self.found
    controls, rows = [None], []
    for i in _kept(columns, solved, tolerances, breaks) if any(solved) else []:
        p = _Point(i)
        at.model.set_altitude(float(heights[i]))
        try:rows.append(_full_row(p, heights[i], speed_kmh, report, levers))
        finally:at.model.set_altitude(float(lo))
        controls[0] = p.controls
    # the unsolved stretches' ends, so the page breaks the line there
    for i in range(len(heights)):
        if not solved[i] and ((i > 0 and solved[i - 1]) or (i + 1 < len(heights) and solved[i + 1]) or i in (0, len(heights) - 1)):
            rows.append(dict(altitude_m=float(heights[i]), speed_kmh=float(speed_kmh), solved=False))
    rows.sort(key=lambda r:r['altitude_m'])
    return rows


def _row_point(name, entry, throttle, pitch, mass, height, speed_kmh, report):
    """A clicked propeller engine point in the state its speed curve shows: the point of its altitude's settled row
    (`_exact_row` over the default speed range). None where that is not available or the row has no state there: the
    caller then solves the point on its own."""
    at = _exact_row(name, entry, throttle, pitch, mass, height, 5, max(800, int(speed_kmh)))
    if at is None:return None
    row = _full_row(at, height, speed_kmh, report, mixture_levers(at.view))
    return row if row['solved'] else None


def point(name, altitude_m, speed_kmh, throttle, pitch, report, axis, level, mass):
    """One point of a sweep, solved exactly as the sweep solves its points and kept in the same store (a level-flight
    point is seeded from the nearest solved one): the page hovers every 1 km/h or 1 m and a click gives the state there
    (Leo, 2026-10-29). axis: the sweep's ('altitude' at speed_kmh, or 'speed' at altitude_m)."""
    entry, prop = admitted(name, throttle, pitch)
    spec = (name, throttle, pitch, mass, bool(level), report)
    fixed, x = (speed_kmh, altitude_m) if axis == 'altitude' else (altitude_m, speed_kmh)
    if prop and not level and float(speed_kmh).is_integer() and float(altitude_m).is_integer() and speed_kmh >= 5:
        with LOCK:
            try:row = _row_point(name, entry, throttle, pitch, mass, int(altitude_m), int(speed_kmh), report)
            except (ValueError, ArithmeticError, np.linalg.LinAlgError):row = None
        if row is not None:return row
    with LOCK:
        store = points(spec[:5] + (axis, fixed))
        if level and not any(seed for _, (_, seed) in store.items()):
            near = _nearest_surface_seed((name, throttle, pitch, mass), altitude_m, speed_kmh)
            if near is not None and x not in store:
                chart = level_solver(name, altitude_m, throttle, pitch, mass)
                store[x] = level_point(chart, altitude_m, speed_kmh, near)
                return store[x][0]
        return sweep_at(spec, axis, fixed, store)(x)


@lru_cache(maxsize=32)
def compute(name, altitude_m, speed_kmh, throttle=110., pitch=None, report=None, sweep=None, level=False, mass='{}',
            altitudes=None, speeds=None, quantity=None):
    """The sweeps at a throttle setting (%): sweep 'altitude' (at speed_kmh), 'speed' (at altitude_m), 'sfc' (the fuel
    laws' SFC curves, no flight condition) or None for altitude and speed. pitch: manual propeller control (%) or None
    for automatic control.
    level: SEP in 1 g level flight instead of the engine at angle of attack 0. mass: JSON of the MASS settings (EM
    defaults where left out). altitudes, speeds: the sweep ranges (min, max), defaults SPEEDS / ALTITUDES. quantity: the
    plotted CURVES key the refinement follows (None: all of them)."""
    entry, prop = admitted(name, throttle, pitch)
    started = time.perf_counter()
    curves = curves_for(quantity)
    spec = (name, throttle, pitch, mass, bool(level), report)
    with LOCK:
        chart = solver(name, altitude_m, throttle, pitch, mass)
        altitudes, speeds = sweeps(entry['propulsion'], altitudes, within_limit(chart, speeds or SPEEDS.get(entry['propulsion'], (0., 800.))))
        mode = dict(key='throttle', label=f'{throttle:g}%', level=bool(level))
        for axis, xs, fixed in (('altitude', altitudes, speed_kmh), ('speed', speeds, altitude_m)):
            if sweep not in (None, axis):continue
            if prop and not level and EXACT_CURVES and len(xs):
                exact = (_exact_speed_rows(name, entry, throttle, pitch, mass, fixed, max(0, math.ceil(min(xs))), math.floor(max(xs)), report)
                         if axis == 'speed' else _exact_altitude_rows(name, entry, throttle, pitch, mass, fixed, list(xs), report, curves))
                if exact is not None:
                    mode[axis] = exact
                    continue
            at = sweep_at(spec, axis, fixed, points(spec[:5] + (axis, fixed)))
            base = march(xs, at, starts(axis, prop), blank(axis, fixed))
            mode[axis] = refined(base, at, PLACE[axis], 2. if axis == 'altitude' else .25 if prop else .5, curves)
            if level and prop and FAST_LEVEL:mode[axis] = settle_level(mode[axis], at, PLACE[axis])
        if sweep == 'sfc':mode['sfc'] = sfc_curves(name, chart, entry['propulsion'])
        modes = [mode]
    engines = chart.engine.properties['engines'] if prop else []
    return dict(aircraft_id=name, name=entry['name'], propulsion=entry['propulsion'], altitude_m=altitude_m,
                speed_kmh=speed_kmh, throttle_percent=throttle, pitch_percent=pitch, modes=modes,
                engine_count=len(engines) if prop else len(main_units(chart)), mass_kg=float(chart.mass['mass']),
                stages=[len(e['properties']['stages']) for e in engines if e['family'] in (0, 1)],
                rpm_limit=([e['properties']['omega_limit'] * 30 / math.pi for e in engines] if prop else
                           [u.ep['omega_limit'] * 30 / math.pi for u in main_units(chart)] if entry['propulsion'] == 'jet' else []),
                elapsed_s=time.perf_counter() - started)


def request(body, report=None):
    """POST /api/engine: {aircraft, sweep: altitude|speed|sfc, level, altitude_m, speed_kmh, throttle, pitch: 'auto'|%,
    mass: {fuel_percent, extra_mass_kg, flaps_percent, loadout, stores_released, default_ammunition, ammunition_vehicle, boosters, booster_fuel_percent},
    point: true for the one point of the sweep at altitude_m and speed_kmh ({row})}."""
    known = {'aircraft', 'sweep', 'level', 'altitude_m', 'speed_kmh', 'throttle', 'pitch', 'mass', 'altitude_range', 'speed_range', 'quantity', 'point', 'slice_start', 'slice_count'}
    if set(body) - known:raise ValueError('Unknown field: ' + ', '.join(sorted(set(body) - known)))
    name = body.get('aircraft')
    if not isinstance(name, str) or name not in AIRCRAFT:raise ValueError('Unknown aircraft')
    def number(key, default, low, high):
        v = body.get(key, default)
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or not low <= v <= high:
            raise ValueError(key + ' out of range')
        return v
    altitude, speed, throttle = number('altitude_m', 0., -math.inf, math.inf), number('speed_kmh', 300., 0., math.inf), number('throttle', 110., 0., 110.)
    pitch = body.get('pitch', 'auto');pitch = None if pitch == 'auto' else number('pitch', None, 0., 100.)
    sweep = body.get('sweep', 'both')
    if sweep not in ('altitude', 'speed', 'sfc', 'both', 'surface', 'slice_speed', 'slice_altitude'):raise ValueError('Unknown sweep')
    if not isinstance(body.get('level', False), bool):raise ValueError('level must be true or false')
    def span(key, low):
        r = body.get(key)
        if r is None:return None
        if (not isinstance(r, list) or len(r) != 2 or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in r)
                or r[0] < low or r[0] > r[1] or (r[0] == r[1] and not sweep.startswith('slice_'))):raise ValueError(key + ' must contain two increasing finite values in range')
        return float(r[0]), float(r[1])
    # The page's true airspeed range has no cap (Leo, 2026-10-30): a sweep stays at most 800 steps and ends at the aircraft's limits.
    altitudes, speeds = span('altitude_range', -math.inf), span('speed_range', 0.)
    quantity = body.get('quantity')
    if quantity is not None and quantity not in CURVES and quantity != 'sfc':raise ValueError('Unknown quantity')
    mass = body.get('mass', {})
    if not isinstance(mass, dict) or set(mass) - set(MASS):raise ValueError('Unknown mass setting')
    # settings() checks every value when the solver is built; the text is canonical so equal settings share a result.
    canonical_mass = json.dumps(mass, sort_keys=True)
    if body.get('point', False) is not False:
        if body['point'] is not True or sweep not in ('altitude', 'speed'):raise ValueError('point needs true and an altitude or speed sweep')
        return dict(row=point(name, float(altitude), float(speed), float(throttle), None if pitch is None else float(pitch), report,
                              sweep, body.get('level', False), canonical_mass))
    if sweep in ('slice_speed','slice_altitude'):
        axis = sweep[6:]
        start, count = body.get('slice_start'), body.get('slice_count')
        if type(start) is not int or abs(start) > 2**53 - 1 or type(count) is not int or not 1 <= count <= 256:
            raise ValueError('Slice requests need an integer start and 1 to 256 consecutive samples')
        if axis == 'speed' and start < 0:raise ValueError('Speed must be nonnegative')
        fixed = altitude if axis == 'speed' else speed
        return compute_surface(name, float(throttle), None if pitch is None else float(pitch),
                               body.get('level', False), canonical_mass, altitudes, speeds, (axis, fixed, start, count))
    if sweep == 'surface':
        return compute_surface(name, float(throttle), None if pitch is None else float(pitch), body.get('level', False),
                               canonical_mass, altitudes, speeds)
    return compute(name, float(altitude), float(speed), float(throttle), None if pitch is None else float(pitch), report,
                   None if sweep == 'both' else sweep, body.get('level', False), canonical_mass, altitudes, speeds, quantity)
