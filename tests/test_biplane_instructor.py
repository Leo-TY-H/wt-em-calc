"""High-speed one-g trim must close the rotated lift balance."""
import json
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from em_solver import TrimSolver, settings
from em_sampling import sample_column
from instructor_chart_inputs import source_state
from windows_instructor_source import fixed_source
from instructor_pitch_predictor import unpack_inputs
from instructor_autotrim import autotrim_predictor
from instructor_static_trim import equilibrium_valid


class BiplaneInstructorTests(unittest.TestCase):
    def test_high_speed_columns_have_verified_balanced_limits(self):
        for name in ('gladiator_mk1_china', 'hs-123a-1_china'):
            config = json.dumps(settings(dict(aircraft=[name])), sort_keys=True)
            for speed in (450., 500., 550.):
                with self.subTest(aircraft=name, speed=speed):
                    column = sample_column((name, config, speed, None), boundary_only=True)
                    self.assertEqual(column['boundary_status'], 'verified limit')
                    point = column['boundary']
                    self.assertTrue(point['valid'], point['reasons'])
                    self.assertLess(point['force_error_g'], 2e-4)
                    self.assertLess(point['angular_error_rad_s2'], 2e-4)
                    auto = point['instructor']['static_trim']
                    self.assertTrue(auto['success'])
                    self.assertLess(abs(auto['equilibrium']['lift_error_n']), 20.)
                    self.assertLess(abs(auto['equilibrium']['moment_error_nm']), 20.)

    def test_fallback_closes_residual_without_relaxing_tolerance(self):
        for name in ('gladiator_mk1_china', 'hs-123a-1_china'):
            with self.subTest(aircraft=name):
                solver = TrimSolver(name, settings(dict(aircraft=[name])))
                point = solver.solve(500., 2., detailed=True)
                self.assertTrue(point['valid'], point['reasons'])
                state = source_state(solver, point['_detail'])
                ip = unpack_inputs(fixed_source(solver.model, state)['auto_inputs'])
                native = autotrim_predictor(solver.model, ip, state['predictor'])
                self.assertFalse(equilibrium_valid(native))
                solved = autotrim_predictor(solver.model, ip, state['predictor'],
                                           lift_iterations=64, lift_bisection=True)
                self.assertTrue(equilibrium_valid(solved))
                self.assertEqual(point['instructor']['static_trim']['method'],
                                 'bracketed algebraic lift solve')
                # An impossible one-g lift demand must remain unresolved.
                impossible = dict(ip)
                impossible[0x2c] *= 1000.
                failed = autotrim_predictor(solver.model, impossible, state['predictor'],
                                           lift_iterations=64, lift_bisection=True)
                self.assertFalse(equilibrium_valid(failed))


if __name__ == '__main__':
    unittest.main()
