"""Reproduce Spitfire telemetry and saved before/after upgrade diagnostics."""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np

from analyze_prop_flights import read, summarize
from body_dynamics import G
from em_solver import BACKEND, TrimSolver, settings
from prop_catalog import mass_state

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'analysis/spitfire-flight-comparison'
NAME = 'spitfire_ix_ussr'
# User-confirmed configuration; filenames have since been corrected.
FLAPS_BY_FILE = {
    'spitfire_ix_ussr-2026_09_21_01_11_39_.csv': 0.,
    'spitfire_ix_ussr-2026_09_21_01_14_25 (landing flaps).csv': 100.,
}


def recorded_runs():
    rows = []
    for path in sorted((ROOT / 'FlightTestData').glob('spitfire*.csv')):
        row = summarize(path)
        row['instructor'] = True  # Confirmed by user for both recordings.
        row['flaps_percent'] = FLAPS_BY_FILE[path.name]
        row['configuration_source'] = 'User confirmed slower run full landing flaps and corrected filenames. Both Instructor, WEP, automatic controls, 30% fuel, some ammunition.'
        row['sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
        data = read(path)
        t = data['Time, s']
        heading = np.degrees(np.unwrap(np.radians(data['Compass'])))
        progress = np.sign(heading[-1] - heading[0]) * (heading - heading[0])
        assert np.all(np.diff(progress) >= 0.)
        row['complete_revolutions'] = []
        for angle in np.arange(0., progress[-1] - 360. + 1e-8, 360.):
            a, b = np.interp([angle, angle + 360.], progress, t)
            ts = np.r_[a, t[(t > a) & (t < b)], b]
            speed = np.interp(ts, t, data['TAS, km/h'])
            row['complete_revolutions'].append(dict(start_s=float(a), end_s=float(b),
                turn_dps=float(360. / (b - a)),
                average_tas_kmh=float(np.trapz(speed, ts) / (b - a))))
        rows.append(row)
    return rows


def solver_for(row, flaps, fuel_percent, match_mass=True, octane=False, production=False):
    cfg = settings(dict(aircraft=[NAME], altitude_m=row['means']['Altitude, m'],
        fuel_percent=fuel_percent, instructor=True, flaps_percent=flaps))
    solver = TrimSolver(NAME, cfg)
    if not production:
        # Preserve the historical comparison after production upgrades ship.
        solver.engine.properties = json.loads((ROOT /
            'references/prop-propulsion/spitfire_ix.json').read_text())['properties']
    baseline = solver.mass['mass']
    extra = row['means']['Total mass, kg'] - baseline if match_mass else 0.
    if extra < 0.:
        raise ValueError('Selected fuel mass already exceeds recorded total mass')
    if match_mass:
        replacement = mass_state(NAME, fuel_percent, extra)
        solver.mass.clear()
        solver.mass.update(replacement)
        solver.weight = solver.mass['mass'] * float(G)
    if octane and not production:
        proof = json.loads((OUT / 'octane-native.json').read_text())
        assert proof['passes']
        solver.engine.properties = copy.deepcopy(solver.engine.properties)
        for engine in solver.engine.properties['engines']:
            assert all(engine['properties'][key] == value for key, value in
                       proof['native_before'].items())
            engine['properties'].update(proof['native_after'])
    return solver, dict(baseline_mass_kg=baseline, added_mass_kg=extra,
        actual_mass_kg=solver.mass['mass'], fuel_percent=fuel_percent,
        policy='Diagnostic mass match at nominal CG; ammunition/fuel split and actual ammunition inertia are not established.')


def save(name, value):
    OUT.mkdir(exist_ok=True)
    (OUT / name).write_text(json.dumps(value, indent=2) + '\n')


def boundaries(rows, fuel_percent, octane=False):
    """Independently solve the graph's allowed upper rate at each mean TAS."""
    suffix = '-octane' if octane else ''
    target = OUT / ('boundaries' + suffix + '.json')
    results = json.loads(target.read_text()) if target.exists() else []
    matched = json.loads((OUT / ('matched-points' + suffix + '.json')).read_text())
    for row in rows:
        if any(r['file'] == row['file'] and r.get('boundary') for r in results):
            continue
        record = next((r for r in matched if r['file'] == row['file'] and
            r['flaps_percent'] == row['flaps_percent']), None)
        if record is None:
            continue
        solver, mass = solver_for(row, row['flaps_percent'], fuel_percent, octane=octane)
        p = record['matched_rate_point']
        speed = p['speed_kmh']
        low = p if p['valid'] else solver.solve(speed, max(1., p['load_g'] * .98),
            p['solution'], exhaustive=False)
        high = p if p['converged'] and not p['valid'] else solver.solve(
            speed, p['load_g'] * 1.02, p['solution'], exhaustive=False)
        for _ in range(8):
            if not high['valid']:
                break
            low = high
            high = solver.solve(speed, high['load_g'] * 1.02, high['solution'], exhaustive=False)
        edge = solver.boundary(speed, low, high, continuation=False)
        result = dict(file=row['file'], flaps_percent=row['flaps_percent'],
            mass=mass, low=low, high=high, boundary=edge)
        results.append(result)
        save('boundaries' + suffix + '.json', results)
        print(row['file'], 'boundary', None if edge is None else
            {k:edge[k] for k in ('turn_dps', 'alpha_deg', 'ps_mps', 'valid', 'envelope_limit')}, flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fuel-percent', type=float, default=30.)
    parser.add_argument('--telemetry-only', action='store_true')
    parser.add_argument('--alternate-flaps', action='store_true')
    parser.add_argument('--boundaries', action='store_true')
    parser.add_argument('--octane', action='store_true', help='Isolated native-verified 150-octane diagnostic')
    args = parser.parse_args()
    rows = recorded_runs()
    save('telemetry.json', rows)
    if args.telemetry_only:
        return
    if args.boundaries:
        boundaries(rows, args.fuel_percent, args.octane)
        return
    results = []
    for row in rows:
        flap_cases = [row['flaps_percent']]
        if args.alternate_flaps:
            flap_cases.append(100. - row['flaps_percent'])
        for flaps in flap_cases:
            start = time.monotonic()
            solver, mass = solver_for(row, flaps, args.fuel_percent, octane=args.octane)
            # Rate is prescribed here to diagnose required lift and energy;
            # agreement in this column cannot validate predicted turn rate.
            speed = row['means']['TAS, km/h']
            load = math.hypot(1., math.radians(row['turn_dps']) * speed / 3.6 / float(G))
            initial = None
            if args.octane:
                baseline = json.loads((OUT / 'matched-points.json').read_text())
                initial = next(r['matched_rate_point']['solution'] for r in baseline
                    if r['file'] == row['file'] and r['flaps_percent'] == flaps)
            point = solver.solve(speed, load, initial=initial, exhaustive=False)
            record = dict(file=row['file'], flaps_percent=flaps, settings=solver.config,
                mass=mass, matched_rate_point=point, backend=BACKEND,
                elapsed_s=time.monotonic() - start)
            results.append(record)
            save('matched-points' + ('-octane' if args.octane else '') + '.json', results)
            print(row['file'], 'flaps', flaps, 'valid', point['valid'],
                'alpha', point['alpha_deg'], 'Ps', point['ps_mps'],
                'thrust kgf', point['engine_force_n'][0] / float(G),
                'reasons', point['reasons'], 'seconds', record['elapsed_s'], flush=True)


if __name__ == '__main__':
    main()
