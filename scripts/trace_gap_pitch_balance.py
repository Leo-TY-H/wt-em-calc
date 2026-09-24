"""Diagnostic only: trace force/roll/yaw closure as pitch command varies.

Uses the production operating-point equations without modification. A trace
does not prove global nonexistence; it exposes pitch-moment shortfalls on the
branches reached from the saved equilibria. It never writes plot results.
"""
import argparse
import json
import time
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

from em_solver import TrimSolver


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('index', type=int)
    parser.add_argument('--count', type=int, default=161)
    args = parser.parse_args()
    root = Path('analysis/equilibrium-gap-recovery')
    fixture = json.loads((root / 'fixtures.json').read_text())['fixtures'][args.index]
    solver = TrimSolver(fixture['aircraft'], fixture['settings'])
    point = fixture['bad']
    speed, load = point['speed_kmh'] / 3.6, point['load_g']
    started = time.monotonic()
    rows, recovered = [], []
    scale = np.array([2e-4, 2e-4, 5e-4, 5e-4])
    for seed in ['lower', 'upper']:
        for direction in [1, -1]:
            z = np.array(fixture[seed]['solution'])[[0, 1, 2, 4]]
            for pitch in np.linspace(-1., 1., args.count)[::direction]:
                def expand(z):
                    return [z[0], z[1], z[2], float(pitch), z[3]]

                def detail(z):
                    return solver.operating_point(speed, load, expand(z))

                def fun(z):
                    return detail(z)['residual'][:4] / scale

                def jac(z):
                    base = fun(z)
                    return np.column_stack([
                        (fun(z + np.eye(4)[i] * step) - base) / step
                        for i, step in enumerate([.002, .002, .0001, .0001])
                    ])

                answer = least_squares(
                    fun, z, jac=jac, bounds=([-6., -15., -1., -1.], [42., 89.7, 1., 1.]),
                    x_scale=[10., 30., .2, .2], max_nfev=60,
                    xtol=1e-10, ftol=1e-10, gtol=1e-10,
                )
                z = answer.x
                d = detail(z)
                closed = bool(max(abs(d['residual'][:4] / scale)) <= 1.)
                admissible = bool(closed and d['history_error'] <= 2e-4 and
                                  d['stall_margin'] >= 0. and d['allocation']['reachable'])
                row = dict(seed=seed, direction=direction, solution=expand(z),
                           residual=d['residual'].tolist(), closed_other=closed,
                           admissible_other=admissible, history_error=d['history_error'],
                           stall_margin=d['stall_margin'],
                           tail_angles=d['result']['tail']['effective_angles'],
                           tail_coefficients=d['result']['tail']['coefficients'],
                           ps=d['ps'])
                rows.append(row)
                if admissible and abs(d['rate_residual'][2]) <= 5e-5:
                    verified = solver.solve(point['speed_kmh'], load, expand(z), exhaustive=False)
                    if verified['valid']:
                        recovered.append(verified)
    admissible = [r for r in rows if r['admissible_other']]
    report = dict(fixture=args.index, aircraft=solver.name,
                  speed_kmh=point['speed_kmh'], load_g=load, rows=rows,
                  recoveries=recovered, elapsed_s=time.monotonic() - started)
    if admissible:
        report['pitch_residual_range_rad_s2'] = [
            min(r['residual'][4] for r in admissible) * .1,
            max(r['residual'][4] for r in admissible) * .1,
        ]
    (root / f'pitch-trace-{args.index}.json').write_text(json.dumps(report, indent=2))
    print(args.index, 'rows', len(rows), 'admissible four-equation closures', len(admissible),
          'pitch range', report.get('pitch_residual_range_rad_s2'),
          'recoveries', len(recovered), 'seconds', report['elapsed_s'], flush=True)


if __name__ == '__main__':
    main()
