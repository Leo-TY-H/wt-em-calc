"""Check relocated data, spawned workers, compiled solves and native aero replay.

Run after installation on each device. This is an installation smoke check,
not validation of the experimental Instructor boundary or of a full chart.
"""
import json
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def solve_case(case):
    from em_solver import BACKEND, TrimSolver, settings
    name, instructor, speed, load = case
    solver = TrimSolver(name, settings(dict(aircraft=[name], instructor=instructor,
                                          torque_gyro=not instructor)))
    point = solver.solve(speed, load, exhaustive=False)
    assert BACKEND == 'compiled', BACKEND
    assert point['valid'], (case, point['reasons'])
    value = solver.point_value(point)
    assert value['force_error_g'] <= 2e-4
    assert max(abs(value['rate_residual'])) <= 5e-5
    assert value['history_error'] <= 2e-4
    if not solver.is_prop and not instructor:
        from verify_aircraft_native import AircraftNative
        native = AircraftNative()
        got = native.call(solver.model, value['velocity'], value['geometry']['omega'].tolist(),
                          solver.mass, value['allocation']['commands'], solver.config['altitude_m'],
                          solver.dt, value['history_input'], flaps=value['flaps'],
                          throttle=solver.config['throttle'],
                          ground_height=solver.config['altitude_m']-1e6,
                          quaternion=value['geometry']['quaternion'])
        aero = value['result']
        for key, field in [('forces', 'component_forces'), ('points', 'component_points')]:
            assert got[key] == {k: aero[field][k] for k in got[key]}, key
        assert got['moment'] == aero['raw_aero_moment'], 'native aero moment'
    return dict(aircraft=name, instructor=instructor, speed_kmh=speed, load_g=load,
                alpha_deg=point['alpha_deg'], ps_mps=point['ps_mps'], backend=BACKEND)


def main():
    os.chdir(ROOT)
    os.environ['WT_EM_PROCESS_START'] = 'spawn'
    os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
    os.environ.setdefault('OMP_NUM_THREADS', '1')
    from em_backend import activate
    assert activate() == 'compiled'
    from em_workers import process_pool, shutdown
    try:
        with process_pool() as (pool, _):
            # Reuse one spawned worker, covering both jet and prop input catalogs.
            for case in [('f_16xl', False, 600., 2.), ('f_16xl', True, 600., 2.),
                         ('j6k1', False, 400., 2.)]:
                print(json.dumps(pool.submit(solve_case, case).result(timeout=180)), flush=True)
    finally:
        shutdown()
    from verify_em_stall_side import main as check_stall_policy
    check_stall_policy()
    print('PASS: portable runtime smoke checks (experimental Instructor boundary remains unvalidated).')


if __name__ == '__main__':
    main()
