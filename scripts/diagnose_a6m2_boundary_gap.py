"""Offline reproduction of seed-dependent limits near the roll-leveling switch.

Does not modify production equations or select a new production boundary.
The event-local bracket demonstrates a proposed connectivity check only.
"""
import argparse
import json
from pathlib import Path

from em_solver import TrimSolver, settings
from em_sampling import sample_column
from em_branch_limit import fixed_alpha
from em_constraint_bracket import refine


def brief(point):
    fields = ('speed_kmh', 'load_g', 'alpha_deg', 'turn_dps', 'valid', 'reasons',
              'roll_leveling_branch', 'force_error_g', 'angular_error_rad_s2')
    result = {key: point.get(key) for key in fields}
    result['instructor_margin'] = (point.get('instructor') or {}).get('envelope_margin')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, default=Path('outputs/a6m2-gap-investigation.json'))
    args = parser.parse_args()
    name = 'a6m2_zero_china'
    cfg = settings(dict(aircraft=[name], sep_tolerance_mps=.15, load_samples=13, heatmap=False))
    encoded = json.dumps(cfg, sort_keys=True)
    speeds = (490.0561117382764, 491.0814958385348, 491.1156753085434,
              491.149854778552, 492.)
    rows = []
    for speed in speeds:
        cold = sample_column((name, encoded, speed, None), boundary_only=True)
        solver = TrimSolver(name, cfg)
        # Follow ordinary low-load equilibria up to a state safely below the
        # event, then inspect the actual one-sided native equations.
        low = solver.solve(speed, 1.)
        for load in range(2, 13):
            low = solver.solve(speed, float(load), low['solution'], exhaustive=False)
            assert low['valid'], brief(low)
        near = solver.solve(speed, 12.7, low['solution'], exhaustive=False)
        assert near['valid'], brief(near)
        sides = []
        seed = near
        for alpha in (11.975, 11.99, 11.999, 12.001, 12.02):
            point = (fixed_alpha(solver, speed, alpha, seed) or
                     fixed_alpha(solver, speed, alpha, near, canonical=True))
            sides.append(point)
            if point is not None:
                seed = point
        assert all(p is not None and p['converged'] for p in sides)
        below, above = sides[2:4]
        assert below['reasons'] == ['Instructor pitch limit'] and above['valid']
        assert below['roll_leveling_branch'] == 0 and above['roll_leveling_branch'] == 1
        first = refine(solver, speed, low, below)
        assert first and first['valid'] and first['alpha_deg'] < 12.
        repeated = sample_column((name, encoded, speed, [first]), boundary_only=True)
        row = dict(speed_kmh=speed, cold_boundary=brief(cold['boundary']),
                   event_checked_boundary=brief(first), lower_seed_boundary=brief(repeated['boundary']),
                   one_sided_points=[brief(p) for p in sides])
        rows.append(row)
        print(speed, 'cold', row['cold_boundary']['turn_dps'], 'event-checked', first['turn_dps'], flush=True)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(dict(settings=cfg, rows=rows, scope=__doc__), indent=2)+'\n', encoding='utf-8')


if __name__ == '__main__':
    main()
