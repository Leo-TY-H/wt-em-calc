"""Engine and propeller performance for the website's Engine performance page (report only; no solver changes).

Straight flight with the body x axis on the flight path (angle of attack 0, no rotation) at a true airspeed and
altitude, fully upgraded (the catalog applies every modification), at 100% throttle and at WEP (110% with the boost).
Propeller aircraft: the chart's own shaft/governor/wake closure (em_stationary.JointProblem.seed_propulsion, the cold
start of a chart point) on every governed-request branch, the first closed and admitted state kept. Jets: the chart's
nominal jet thrust (EngineUnit.nominal) at the altitude's density, through the nozzles. Two sweeps per throttle: power
and the rest over altitude at one speed, and over speed at one altitude.
"""
import copy
import itertools
import math
import threading
import time
from functools import lru_cache

import numpy as np

from component_assembly import f32, mul
from control_mixer import density_at_height
from em_solver import AIRCRAFT, TrimSolver, settings

METRIC_HP = 735.49875
KGF = 9.80665
MODES = (('wep', 'WEP', dict(throttle=1.1, afterburner=True)), ('military', '100%', dict(throttle=1., afterburner=False)))
LOCK = threading.Lock()


def sweeps(propulsion):
    """Altitudes (m) and true airspeeds (km/h) of the two sweeps. A propeller needs inflow: its first speed is 5 km/h."""
    if propulsion in ('jet', 'rocket'):
        return [250. * i for i in range(65)], [20. * i for i in range(81)]
    return [200. * i for i in range(61)], [5.] + [10. * i for i in range(1, 91)]


@lru_cache(maxsize=24)
def solver(name, mode, pitch):
    extra = dict(next(c for k, _, c in MODES if k == mode))
    if pitch is not None:extra.update(manual_engine_controls=True, propeller_pitch_auto=False, propeller_pitch_percent=pitch)
    return TrimSolver(name, settings(dict(aircraft=[name], **extra)))


def at_altitude(base, height):
    """The solver at another altitude. Propulsion reads the altitude from the configuration at every call."""
    view = copy.copy(base);view.config = dict(base.config, altitude_m=float(height))
    return view


def propeller_state(view, speed_kmh, warm):
    """The closed, admitted propulsion state of the first governed-request branch that has one (None if none). Each
    branch starts cold, as a chart point does; where that fails, from the sweep's previous state on the branch (warm);
    last, the chart's axial RPM reference (BaselineSolver.operating_point at alpha 0: the whole joint problem with its
    propeller-branch recovery), slower but closing states the propulsion block alone can miss."""
    from em_stationary import JointProblem, prepare
    from em_stationary_baseline import BaselineSolver
    from prop_admission import evaluate as admission
    from prop_equilibrium import RESIDUAL_TOLERANCE
    from prop_quasisteady import propeller_condition
    from propulsion_general import fixed_air
    attitude = [0.] * 5
    _, requests, velocity, rates = prepare(view, speed_kmh / 3.6, 1., attitude)
    p = view.engine.properties
    air, sound = fixed_air(velocity, view.config['altitude_m'])
    def admitted(prop):
        """Blade-tip Mach of each propeller when every propeller passes the chart's admission, else None."""
        state, checks = prop['state'], []
        for i, shaft in enumerate(p['transmissions']):
            for link in shaft['propellers']:
                pp = p['propellers'][link['index']]['properties'];rotor = state['propellers'][link['index']]
                checks.append((link['index'], admission(pp, rotor['flow'], rotor['pitch'], state['transmissions'][i]['omega'] * link['ratio'],
                                                        air['density'], sound, propeller_condition(pp, velocity, rates))))
        if not all(row['admitted'] for _, row in checks):return None
        tips = dict((index, max((s['mach'] for r in row['rotors'] for s in r['sections']), default=None)) for index, row in checks)
        return [tips.get(i) for i in range(len(p['propellers']))]
    for branch in itertools.product(*[range(len(r['candidates'])) if r['governed'] else [0] for r in requests]):
        problem = JointProblem(view, speed_kmh, 1., attitude, branch)
        starts = [problem.initial]
        if len(warm.get(branch, ())) == len(problem.initial):
            start = problem.initial.copy();start[problem.prop_start:] = warm[branch][problem.prop_start:];starts.append(start)
        for start in starts:
            q = problem.seed_propulsion(start)
            residual, prop = problem.propulsion_at(q)
            if not (prop['converged'] and np.all(np.isfinite(residual)) and np.max(abs(residual), initial=0.) <= RESIDUAL_TOLERANCE):
                continue
            tips = admitted(prop)
            if tips is not None:
                warm[branch] = q
                return prop, tips, air
    chart = copy.copy(view);chart.__class__ = BaselineSolver
    prop = chart.operating_point(speed_kmh / 3.6, 1., attitude)['propulsion']
    tips = admitted(prop) if prop.get('converged') else None
    return None if tips is None else (prop, tips, air)


def propeller_row(view, speed_kmh, report, warm):
    found = propeller_state(view, speed_kmh, warm)
    if found is None:return None
    prop, tips, air = found
    p, state = view.engine.properties, prop['state']
    v = speed_kmh / 3.6;power = sum(e.get('torque', 0.) * e['omega'] for e in state['engines'])
    pistons = [i for i, e in enumerate(p['engines']) if e['family'] in (0, 1)]
    # The clicked-point report's pitch % (em_server.pitch_percent) from the same fields the chart's points carry.
    point = dict(propulsion=dict(controls=prop['controls'], stationarity=prop['stationarity'],
                                 engine_rpm=[e['omega'] * 30 / math.pi for e in state['engines']],
                                 propeller_pitch_deg=[math.degrees(q['pitch']) for q in state['propellers']]))
    if report is not None:report(point, view)
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


def jet_row(base, height, speed_kmh):
    from air_state import cache
    from jet_nozzle import evaluate as evaluate_nozzles
    rho = density_at_height(f32(height));v = speed_kmh / 3.6;thrust, rpm = 0., []
    for unit in base.engine.units:
        # EngineUnit.vectors at another altitude: its density is fixed when the solver is built.
        view = copy.copy(unit);view.rho = rho
        nominal = view.nominal(v)
        ias = mul(f32(v), f32(math.sqrt(f32(rho / f32(1.225)))))
        thrust += evaluate_nozzles(view.nozzles, nominal['scalar']['thrust'], base.mass['cog'], ias, flaps=0.)['force'][0]
        rpm.append(nominal['omega'] * 30 / math.pi)
    air = cache([f32(v), 0., 0.], height)
    return dict(thrust_kgf=float(thrust) / KGF, rpm=rpm, mach=float(air['mach']), ias_kmh=float(air['ias_u']) * 3.6)


@lru_cache(maxsize=32)
def compute(name, altitude_m, speed_kmh, pitch=None, report=None):
    """Both sweeps at 100% and WEP. pitch: manual propeller control (%) or None for the chart's automatic control."""
    entry = AIRCRAFT[name]
    if not entry['supported']:raise ValueError(entry['name'] + ': ' + entry['reason'])
    if entry['propulsion'] == 'rocket':raise ValueError('Rocket aircraft have no engine performance page')
    prop = entry['propulsion'] not in ('jet', 'rocket')
    if pitch is not None and not (prop and entry.get('manual_controls', {}).get('propeller_pitch')):
        raise ValueError('Manual propeller pitch is unavailable for ' + entry['name'])
    started = time.perf_counter()
    altitudes, speeds = sweeps(entry['propulsion'])
    modes = []
    with LOCK:
        for key, label, _ in MODES:
            base = solver(name, key, pitch)
            def row(height, speed, warm):
                try:
                    value = (propeller_row(at_altitude(base, height), speed, report, warm) if prop else
                             jet_row(base, height, speed))
                except (ValueError, ArithmeticError, np.linalg.LinAlgError):value = None
                return dict(value or {}, altitude_m=height, speed_kmh=speed, solved=value is not None)
            up, along = {}, {}
            modes.append(dict(key=key, label=label, altitude=[row(h, speed_kmh, up) for h in altitudes],
                              speed=[row(altitude_m, v, along) for v in speeds]))
        properties = base.engine.properties if prop else None
    # Without a boost (no throttle boost, no afterburner) WEP is the 100% state: show it once.
    same = lambda a, b:all(x.get('thrust_kgf') == y.get('thrust_kgf') and x.get('rpm') == y.get('rpm') for x, y in zip(a, b))
    if same(modes[0]['altitude'], modes[1]['altitude']) and same(modes[0]['speed'], modes[1]['speed']):
        modes = [dict(modes[1], key='military', label='100%')]
    engines = properties['engines'] if prop else []
    return dict(aircraft_id=name, name=entry['name'], propulsion=entry['propulsion'], altitude_m=altitude_m,
                speed_kmh=speed_kmh, pitch_percent=pitch, modes=modes,
                engine_count=len(engines) if prop else len(base.engine.units),
                stages=[len(e['properties']['stages']) for e in engines if e['family'] in (0, 1)],
                rpm_limit=([e['properties']['omega_limit'] * 30 / math.pi for e in engines] if prop else
                           [u.ep['omega_limit'] * 30 / math.pi for u in base.engine.units]),
                elapsed_s=time.perf_counter() - started)
