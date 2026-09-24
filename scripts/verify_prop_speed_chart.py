"""Independent holdouts, saved-state replay and boundary checks after optimization."""
import argparse
import json
import math
from pathlib import Path
from em_solver import TrimSolver
from em_sampling import column_at_load, coordinate_load, load_coordinate

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'analysis/prop-speed'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--chart', default='deployed-chart')
    args = ap.parse_args()
    data = json.loads((OUT/(args.chart+'.json')).read_text())
    failures, replay, holdouts, native_points, comparisons = [], [], [], [], []
    for aircraft in data['aircraft']:
        name = aircraft['id']
        solver = TrimSolver(name, aircraft['settings'])
        native_settings = {k:v for k,v in aircraft['settings'].items() if k!='aircraft'}
        columns = aircraft['columns']
        # Fixed targets select representative low, middle and high speed columns.
        chosen = [min((c for c in columns if c['boundary']), key=lambda c:abs(c['speed_kmh']-v))
                  for v in [200., 350., 550.]]
        for c in chosen:
            native_points.append(dict(c['boundary'], aircraft=name, settings=native_settings))
            if c['sustained']:
                native_points.append(dict(c['sustained'][0], aircraft=name, settings=native_settings))
            top = c['boundary']['load_g']
            bottom = c['lower_boundary']['load_g']
            for fraction in [.173, .417, .783]:
                u = float(load_coordinate(bottom, top))*(1-fraction)+fraction
                load = float(coordinate_load(u, top))
                predicted = float(column_at_load(c, [load])[0])
                if not math.isfinite(predicted):
                    failures.append(dict(stage='holdout masked', aircraft=name, speed=c['speed_kmh'], load=load))
                    continue
                seed = min((p for p in c['points'] if p['valid']), key=lambda p:abs(p['load_g']-load))
                point = solver.solve(c['speed_kmh'], load, seed['solution'])
                row = dict(aircraft=name, speed=c['speed_kmh'], load=load, valid=point['valid'],
                           error_mps=abs(predicted-point['ps_mps']))
                holdouts.append(row)
                if not point['valid'] or row['error_mps']>data['settings']['sep_tolerance_mps']:
                    failures.append(dict(stage='holdout', **row))
        print(name, 'holdouts complete', flush=True)
        if name != 'yak-3':
            continue
        baseline = json.loads((OUT/'before-chart.json').read_text())['aircraft'][0]
        old_columns = {c['speed_kmh']:c for c in baseline['columns']}
        for c in columns:
            old = old_columns.get(c['speed_kmh'])
            if not old or not c['boundary'] or not old['boundary']:
                continue
            row = dict(speed_kmh=c['speed_kmh'], **{k:abs(c['boundary'][k]-old['boundary'][k])
                       for k in ['load_g', 'ps_mps', 'turn_dps']})
            comparisons.append(row)
            if row['load_g']>.001 or row['ps_mps']>.1:
                failures.append(dict(stage='boundary changed', **row))
        for c in baseline['columns'][::5]:
            for p in c['points'][::max(1, len(c['points'])//3)]:
                value = solver.point_value(p)
                exact = (value['ps']==p['ps_mps'] and value['result']['force']==p['force_n']
                         and value['result']['stored_moment']==p['moment_nm'])
                replay.append(dict(speed=p['speed_kmh'], load=p['load_g'], exact=exact))
                if not exact:
                    failures.append(dict(stage='saved equations changed', speed=p['speed_kmh'], load=p['load_g']))
        assert aircraft['mass']['mass']==2464.
    report = dict(status='FAIL' if failures else 'PASS', replay=replay, holdouts=holdouts,
                  boundaries=comparisons, failures=failures)
    (OUT/'chart-validation.json').write_text(json.dumps(report, indent=2)+'\n')
    (OUT/'boundary-points.json').write_text(json.dumps(native_points, indent=2)+'\n')
    print(report['status'], len(replay), 'exact replays', len(holdouts), 'holdouts', flush=True)
    assert not failures, failures[:5]


if __name__ == '__main__':
    main()
