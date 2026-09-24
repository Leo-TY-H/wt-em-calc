"""Offline six-coordinate feasibility search; never used by the EM plotter.

Sideslip is an additional attitude variable. All five production force/moment
equations and all acceptance constraints remain in force. An underdetermined
least-squares search is not an optimum-performance or minimum-sideslip solve.
"""
import json
import multiprocessing
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from probe_sideslip_equilibria import SlipSolver


def one(pair):
    index, fixture = pair
    solver = SlipSolver(fixture['aircraft'], fixture['settings'])
    bad = fixture['bad']
    memo = {}

    def evaluate(z):
        key = tuple(z)
        if key not in memo:
            if len(memo) > 512:
                memo.clear()
            solver.beta = float(z[5])
            memo[key] = solver.operating_point(bad['speed_kmh'] / 3.6, bad['load_g'], z[:5])
        return memo[key]

    def fun(z):
        return evaluate(z)['residual']

    def jac(z):
        base = evaluate(z)
        columns = []
        for axis, step in enumerate([.002, .002, .0002, .0002, .0002, .002]):
            for factor in [1., .5, 1.5, 2., -1., -.5, .25, -.25, 4., -4.]:
                delta = np.eye(6)[axis] * step * factor
                value = evaluate(z + delta)
                if solver.derivative_branch(base) == solver.derivative_branch(value):
                    break
            columns.append((value['residual'] - base['residual']) / (step * factor))
        return np.column_stack(columns)

    rows = []
    solution = None
    for beta in [0., 1., -1., 5., -5., 15., -15., 30., -30.]:
        for seed in ['bad', 'upper', 'lower']:
            start = np.array([*fixture[seed]['solution'], beta])
            fit = least_squares(fun, start, jac=jac,
                                bounds=([-6., -15., -1., -1., -1., -45.],
                                        [42., 89.7, 1., 1., 1., 45.]),
                                x_scale=[10., 30., .2, .2, .2, 10.],
                                max_nfev=100, ftol=1e-10, xtol=1e-9, gtol=1e-9)
            solver.beta = float(fit.x[5])
            # Recheck through the ordinary acceptance/authority path at the
            # resulting fixed beta, without certifying by residual norm alone.
            point = solver.solve(bad['speed_kmh'], bad['load_g'], fit.x[:5].tolist(),
                                 detailed=True, exhaustive=False)
            rows.append(dict(seed=seed, beta_seed=beta, beta_input=solver.beta,
                             solution=point['solution'], valid=point['valid'],
                             reasons=point['reasons'], force_error_g=point['force_error_g'],
                             angular_error_rad_s2=point['angular_error_rad_s2'],
                             stall_margin_deg=point['stall_margin_deg'], nfev=fit.nfev))
            if point['valid']:
                solution = {k: v for k, v in point.items() if not k.startswith('_')}
                solution.update(beta_input=solver.beta,
                                sideslip_deg=point['_detail']['result']['air']['beta'])
                break
        if solution:
            break
    return dict(index=index, aircraft=solver.name, speed=bad['speed_kmh'],
                load=bad['load_g'], solution=solution, trials=rows)


def main():
    fixtures = json.loads(Path('analysis/equilibrium-gap-recovery/fixtures.json').read_text())['fixtures']
    rows = []
    start = time.monotonic()
    with ProcessPoolExecutor(max_workers=4, mp_context=multiprocessing.get_context('spawn')) as pool:
        futures = [pool.submit(one, (index, fixtures[index])) for index in range(3, 8)]
        for future in as_completed(futures):
            row = future.result()
            rows.append(row)
            Path('analysis/gap-resolution/sideslip-joint.json').write_text(
                json.dumps(dict(rows=rows, seconds=time.monotonic() - start), indent=2) + '\n')
            print(row['index'], 'solved', bool(row['solution']), 'attempts', len(row['trials']), flush=True)


if __name__ == '__main__':
    main()
