"""Canard sign conventions and trim-independent low-speed reachability."""
import json
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from em_solver import TrimSolver, settings
from em_sampling import sample_column
from instructor_chart_inputs import source_state
from instructor_steady_aoa import controller_limits
from instructor_static_trim import trim_independent_authority
from instructor_predictor_inputs import pack_pitch_inputs
from instructor_pitch_predictor import unpack_inputs, pitch_predictor
from primary_controls import steady_commands
from windows_instructor_source import fixed_source


class CanardInstructorTests(unittest.TestCase):
    def test_forward_balance_inverts_predictor_in_delivered_coordinates(self):
        # The predictor returns a simulation command; delivery inverts it on
        # these canards. Compare after delivery, including a conventional tail.
        for name in ('j_10a', 'saab_jas39c', 'ef_2000_block_10', 'f_16xl'):
            with self.subTest(aircraft=name):
                solver = TrimSolver(name, settings(dict(aircraft=[name], instructor=False)))
                point = solver.solve(1000., 10., detailed=True)
                self.assertTrue(point['converged'])
                value = point['_detail']
                result = controller_limits(solver, value)
                state = source_state(solver, value)
                fixed = fixed_source(solver.model, state)
                demand = dict(result['rate_demands'][1],
                              target_acceleration=result['reduced_balances'][1]['acceleration'])
                packed = pack_pitch_inputs(**state['wrapper'], **demand,
                    body_pitch_rate=state['pitch_rate'], angle_bounds=fixed['tail_bounds'],
                    axis_weights=[0., 0., 0.], quaternion=state['quaternion'],
                    world_velocity=state['world_velocity'])
                inverse = pitch_predictor(solver.model, unpack_inputs(packed), state['predictor'])
                delivered = inverse['output'][1]
                if solver.controls['invert_elevator']:
                    delivered = -delivered
                self.assertAlmostEqual(delivered, result['delivered_pitch'], delta=.002)

    def test_full_authority_endpoints_are_independent_of_trim(self):
        # Exercise the actual nonlinear delivery, not a copy of the test for
        # full authority. Both inverted pitch and unavailable trim are covered.
        solver = TrimSolver('j_10a', settings(dict(aircraft=['j_10a'])))
        for inverted in (False, True):
            controls = dict(solver.controls, invert_elevator=inverted,
                            trim_available=[False, True, True])
            ranges = [[-.4, .4], [-1., 1.], [-1., 1.]]
            self.assertTrue(trim_independent_authority(controls, ranges))
            baseline = [steady_commands(controls, [stick]*3, [0.]*3, ranges)
                        for stick in (-1., 1.)]
            for trim in (-1., -.75, -.1, 0., .3, .9, 1.):
                for stick, expected in zip((-1., 1.), baseline):
                    actual = steady_commands(controls, [stick]*3, [trim]*3, ranges)
                    for a, b in zip(actual, expected):
                        self.assertAlmostEqual(a, b, delta=1e-7)
            self.assertFalse(trim_independent_authority(controls,
                             [[-.4, .4], [-.999, 1.], [-1., 1.]]))

    def test_low_speed_columns_remain_balanced_across_auto_trim_gaps(self):
        for name, speeds in (('j_10a', (110., 140., 150.)),
                             ('saab_jas39c', (130., 140.)),
                             ('ef_2000_block_10', (140., 180., 200.))):
            config = settings(dict(aircraft=[name]))
            for speed in speeds:
                with self.subTest(aircraft=name, speed=speed):
                    column = sample_column((name, json.dumps(config, sort_keys=True), speed, None),
                                           boundary_only=True)
                    self.assertEqual(column['boundary_status'], 'verified limit')
                    point = column['boundary']
                    self.assertTrue(point['valid'])
                    self.assertLess(point['force_error_g'], 2e-4)
                    self.assertLess(point['angular_error_rad_s2'], 2e-4)
                    result = point['instructor']
                    self.assertTrue(result['converged'])
                    self.assertTrue(result['control_authority_trim_independent'])
                    # Direct lift closure now recovers some formerly failed
                    # inverse-CL roots. The remaining gaps must still use the
                    # trim-independent authority certificate.
                    recovered = ((name == 'j_10a' and speed == 150.) or
                                 (name == 'ef_2000_block_10' and speed in (140., 180., 200.)))
                    auto = result['static_trim']
                    self.assertEqual(auto['success'], recovered)
                    if recovered:
                        self.assertLess(abs(auto['equilibrium']['lift_error_n']), 20.)
                        self.assertLess(abs(auto['equilibrium']['moment_error_nm']), 20.)
                        self.assertIsNotNone(point['trim'])
                        self.assertIsNotNone(point['sticks'])
                    else:
                        self.assertIsNone(point['trim'])
                        self.assertIsNone(point['sticks'])

    def test_high_speed_ten_g_is_not_rejected_by_reversed_canard(self):
        for name in ('j_10a', 'saab_jas39c'):
            with self.subTest(aircraft=name):
                solver = TrimSolver(name, settings(dict(aircraft=[name])))
                point = solver.solve(1000., 10.)
                self.assertTrue(point['valid'], point['reasons'])
                self.assertTrue(point['instructor']['static_trim']['success'])


if __name__ == '__main__':
    unittest.main()
