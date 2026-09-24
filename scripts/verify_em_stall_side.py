"""Regression: only positive stall excludes EM points, in both flight modes."""
from em_solver import TrimSolver, settings


def main():
    for name, speed, flaps, upper in [('f_16xl', 100., 0., True),
                                     ('j6k1', 771.5, 30., False)]:
        for instructor in (False, True):
            config = settings(dict(aircraft=[name], instructor=instructor,
                                   torque_gyro=not instructor, flaps_percent=flaps))
            solver = TrimSolver(name, config)
            point = solver.solve(speed, 1., exhaustive=False)
            value = solver.point_value(point)
            if not instructor:
                assert value['phase_results'] == [], 'SB must exercise the empty-history path'
            if upper:
                assert point['stall_margin_deg'] < .15, (name, instructor, point['stall_margin_deg'])
                assert ('post-stall' in point['reasons']) == (point['stall_margin_deg'] < 0.)
                beyond = list(point['solution']); beyond[0] += 1.
                trial = solver.operating_point(speed/3.6, 1., beyond)
                assert trial['stall_margin'] < 0., trial['stall_margin']
            else:
                assert point['negative_stall_margin_deg'] < .15, (name, instructor, point['alpha_deg'])
                assert point['stall_margin_deg'] > 20., (name, instructor, point['stall_margin_deg'])
                assert 'post-stall' not in point['reasons'], point['reasons']
                # Removing an exclusion cannot make an unbalanced iterate valid.
                if point['force_error_g'] > 2e-4:
                    assert not point['valid'] and 'trim did not converge' in point['reasons']
            print(name, 'RB' if instructor else 'SB', 'positive stall retained' if upper else 'negative stall ignored', 'PASS', flush=True)


if __name__ == '__main__':
    main()
