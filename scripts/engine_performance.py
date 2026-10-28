"""Engine and propeller performance for the website's Engine performance page (report only; no solver changes).

The EM plotter's own logic throughout (Leo, 2026-10-28): each point is calculated by a solver built from the same
settings as an EM chart at that altitude (the EM page's throttle slider: setting / 100, boost requested above 100%;
its manual propeller control; every other setting at the EM defaults, fully upgraded as the catalog applies every
modification). Propeller aircraft: the EM plotter's axial-flow propulsion state (em_stationary_baseline.AxialReference:
BaselineSolver.operating_point at 1 g, angle of attack, bank and controls 0), kept when converged and admitted by the
chart's propeller admission. Jets: the chart's engine force (EngineEnsemble.vectors). Two sweeps: power and the rest
over altitude at one true airspeed, and over true airspeed at one altitude.
"""
import math
import threading
import time
from functools import lru_cache

import numpy as np

from component_assembly import f32
from em_solver import AIRCRAFT, settings

METRIC_HP = 735.49875
KGF = 9.80665
LOCK = threading.Lock()


def sweeps(propulsion):
    """Altitudes (m) and true airspeeds (km/h) of the two sweeps. A propeller needs inflow: its first speed is 5 km/h."""
    if propulsion in ('jet', 'rocket'):
        return [250. * i for i in range(65)], [20. * i for i in range(81)]
    return [200. * i for i in range(61)], [5.] + [10. * i for i in range(1, 91)]


@lru_cache(maxsize=1)
def without_wep():
    """Aircraft whose engines have no boost above 100% throttle, so the game's throttle stops at 100% (Leo, 2026-10-28):
    no WEP the pilot or the engine can engage (the chart's jet rule, engine_commands.afterburner_command) and no
    ThrottleBoost. The pages cap their throttle sliders at 100% for these; the solver itself is unchanged."""
    from aircraft_catalog import load
    from engine_commands import afterburner_command
    from jet_catalog import engines
    out = []
    for name in AIRCRAFT:
        row = AIRCRAFT[name]
        if not row.get('supported') or row['propulsion'] == 'rocket':continue
        try:mains = [e for _, e in engines(load(name)) if e.get('Main', {}).get('Type') != 'Rocket']
        except (OSError, ValueError, KeyError):continue
        boost = lambda e:(afterburner_command(True, e.get('Afterburner', {}).get('Type', 0), e.get('Afterburner', {}).get('IsControllable', True))
                          or e.get('Main', {}).get('ThrottleBoost', 1.) > 1.)
        if mains and not any(map(boost, mains)):out.append(name)
    return sorted(out)


def solver(name, altitude, throttle, pitch):
    """The EM chart's solver for these settings (built per altitude: the catalog caches the aircraft, so this is cheap)."""
    from em_stationary_baseline import BaselineSolver
    extra = dict(altitude_m=float(altitude), throttle=throttle / 100., afterburner=throttle > 100.)
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
    velocity, _, geometry, _ = flight_condition(chart, v, 1., attitude)
    velocity, rates = tuple(map(f32, velocity)), tuple(map(f32, geometry['omega']))
    air, sound = fixed_air(velocity, chart.config['altitude_m'])
    p, state = chart.engine.properties, prop['state']
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


@lru_cache(maxsize=32)
def compute(name, altitude_m, speed_kmh, throttle=110., pitch=None, report=None):
    """Both sweeps at a throttle setting (%). pitch: manual propeller control (%) or None for automatic control."""
    entry = AIRCRAFT[name]
    if not entry['supported']:raise ValueError(entry['name'] + ': ' + entry['reason'])
    if entry['propulsion'] == 'rocket':raise ValueError('Rocket aircraft have no engine performance page')
    prop = entry['propulsion'] not in ('jet', 'rocket')
    if pitch is not None and not (prop and entry.get('manual_controls', {}).get('propeller_pitch')):
        raise ValueError('Manual propeller pitch is unavailable for ' + entry['name'])
    if throttle > 100. and name in without_wep():raise ValueError(entry['name'] + ' has no WEP: its throttle stops at 100%')
    started = time.perf_counter()
    altitudes, speeds = sweeps(entry['propulsion'])
    def row(chart, height, speed):
        try:value = propeller_row(chart, speed, report) if prop else jet_row(chart, speed)
        except (ValueError, ArithmeticError, np.linalg.LinAlgError):value = None
        return dict(value or {}, altitude_m=height, speed_kmh=speed, solved=value is not None)
    with LOCK:
        level = solver(name, altitude_m, throttle, pitch)
        modes = [dict(key='throttle', label=f'{throttle:g}%',
                      altitude=[row(solver(name, h, throttle, pitch), h, speed_kmh) for h in altitudes],
                      speed=[row(level, altitude_m, v) for v in speeds])]
    engines = level.engine.properties['engines'] if prop else []
    return dict(aircraft_id=name, name=entry['name'], propulsion=entry['propulsion'], altitude_m=altitude_m,
                speed_kmh=speed_kmh, throttle_percent=throttle, pitch_percent=pitch, modes=modes,
                engine_count=len(engines) if prop else len(level.engine.units),
                stages=[len(e['properties']['stages']) for e in engines if e['family'] in (0, 1)],
                rpm_limit=([e['properties']['omega_limit'] * 30 / math.pi for e in engines] if prop else
                           [u.ep['omega_limit'] * 30 / math.pi for u in level.engine.units]),
                elapsed_s=time.perf_counter() - started)
