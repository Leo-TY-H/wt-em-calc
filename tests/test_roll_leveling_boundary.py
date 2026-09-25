"""The first permitted turn limit survives a native roll-helper switch."""
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1] / 'scripts'))
from em_sampling import _sample_column, sample_boundary_column, sample_column
from em_solver import TrimSolver, settings
from aircraft_catalog import load as load_aircraft
from em_branch_limit import first_roll_leveling_limit
from em_boundary_seam import check_boundary, apply_outline, boundary_intervals


class RollLevelingBoundaryTests(unittest.TestCase):
    def test_default_disables_helper_without_changing_catalog_aircraft(self):
        name='a6m2_zero_china'
        original=load_aircraft(name).get('RollLeveling',True)
        disabled=TrimSolver(name,settings(dict(aircraft=[name])))
        enabled=TrimSolver(name,settings(dict(aircraft=[name],roll_leveling=True)))
        self.assertFalse(disabled.fm['RollLeveling'])
        self.assertFalse(disabled.model['fm']['RollLeveling'])
        self.assertTrue(enabled.model['fm']['RollLeveling'])
        self.assertEqual(load_aircraft(name).get('RollLeveling',True),original)

    @classmethod
    def setUpClass(cls):
        cls.name='a6m2_zero_china'
        cls.speed=491.1156753085434
        cls.config=json.dumps(settings(dict(aircraft=[cls.name],sep_tolerance_mps=.15,
                                            load_samples=13,heatmap=False,roll_leveling=True)),sort_keys=True)
        cls.task=(cls.name,cls.config,cls.speed,None)
        # This is the locally valid, but disconnected, upper root the fast
        # active-constraint search used to publish at this speed.
        cls.upper=_sample_column(cls.task,boundary_only=True)

    def assert_first_limit(self,column):
        point=column['boundary']
        self.assertEqual(column['boundary_status'],'verified limit')
        self.assertTrue(point['valid'])
        self.assertLess(point['alpha_deg'],12.)
        self.assertLess(point['turn_dps'],52.7)
        self.assertEqual(point['envelope_limit']['kind'],'Instructor pitch')
        self.assertEqual(point['envelope_limit']['roll_leveling_switch_deg'],12.)
        self.assertLess(point['force_error_g'],2e-4)
        self.assertLess(point['angular_error_rad_s2'],2e-4)

    def test_cold_and_reused_upper_root_select_first_limit(self):
        self.assertGreater(self.upper['boundary']['alpha_deg'],12.)
        cold=sample_column(self.task,boundary_only=True)
        reused=sample_column((self.name,self.config,self.speed,self.upper['points']),
                             boundary_only=True)
        for column in (cold,reused):
            self.assert_first_limit(column)

    def test_fast_boundary_probe_checks_its_candidate(self):
        with patch('em_sampling._sample_boundary_column',return_value=dict(self.upper)):
            column=sample_boundary_column(self.task)
        self.assert_first_limit(column)

    def test_neighboring_speeds_keep_the_same_first_component(self):
        rates=[]
        for speed in (491.0814958385348,self.speed,491.149854778552):
            column=sample_column((self.name,self.config,speed,None),boundary_only=True)
            self.assert_first_limit(column)
            rates.append(column['boundary']['turn_dps'])
        self.assertLess(max(rates)-min(rates),.02)

    def test_permitted_switch_crossing_preserves_another_aircraft_boundary(self):
        name='ef_2000_block_10';speed=600.
        config=json.dumps(settings(dict(aircraft=[name],sep_tolerance_mps=.15,
                                        load_samples=13,heatmap=False,roll_leveling=True)),sort_keys=True)
        task=(name,config,speed,None)
        raw=_sample_column(task,boundary_only=True)
        checked=sample_column(task,boundary_only=True)
        self.assertGreater(raw['boundary']['alpha_deg'],12.)
        self.assertEqual(checked['boundary_status'],'verified limit')
        self.assertAlmostEqual(checked['boundary']['turn_dps'],
                               raw['boundary']['turn_dps'],delta=.001)

    def test_disabled_helper_does_not_change_the_candidate(self):
        solver=TrimSolver(self.name,json.loads(self.config))
        with patch.dict(solver.fm,{'RollLeveling':False}):
            self.assertIsNone(first_roll_leveling_limit(solver,self.speed,
                              self.upper['lower_boundary'],self.upper['boundary']))

    def test_sideslip_recovery_is_not_reclassified_by_zero_sideslip_probe(self):
        solver=TrimSolver(self.name,json.loads(self.config))
        candidate=dict(self.upper['boundary'],sideslip_attitude_deg=2.)
        self.assertIsNone(first_roll_leveling_limit(solver,self.speed,
                          self.upper['lower_boundary'],candidate))

    def test_rejection_thinner_than_old_probe_offset_is_found(self):
        column=sample_column((self.name,self.config,487.9,None),boundary_only=True)
        self.assert_first_limit(column)
        self.assertGreater(column['boundary']['alpha_deg'],11.998)

    def test_native_speed_jump_has_measured_sides_and_a_drawable_edge(self):
        pair=[sample_column((self.name,self.config,speed,None),boundary_only=True)
              for speed in (487.82,487.88)]
        task=(self.name,self.config,487.85,pair)
        certificate=check_boundary(task)
        self.assertIsNotNone(certificate)
        lo,hi=certificate['transition_speed_interval_kmh']
        self.assertLessEqual(hi-lo,.01)
        self.assertNotEqual(*[p['valid'] for p in certificate['incoming_checks']])
        self.assertTrue(certificate['outgoing_check']['valid'])
        outline=[dict(speed_kmh=speed,turn_dps=None) for speed in (480.,487.85,500.)]
        drawn=apply_outline(outline,[certificate])
        self.assertTrue(all(p['turn_dps'] is not None for p in drawn if 487.82<=p['speed_kmh']<=487.88))
        self.assertIsNone(drawn[0]['turn_dps'])
        self.assertIsNone(drawn[-1]['turn_dps'])
        vertical=[p for p in drawn if p.get('vertical_edge')]
        self.assertEqual(len(vertical),2)
        self.assertEqual(vertical[0]['speed_kmh'],vertical[1]['speed_kmh'])
        self.assertGreater(abs(vertical[0]['turn_dps']-vertical[1]['turn_dps']),.15)
        # No balance evidence means no certificate and no permission to draw
        # through an existing numerical gap.
        with patch('em_branch_limit.fixed_alpha',return_value=None):
            self.assertIsNone(check_boundary(task))
        self.assertEqual(apply_outline(outline,[]),outline)

    def test_boundary_check_can_span_an_ambiguous_shared_knot(self):
        spans=[(487.80026671770804,487.83444618771665),
               (487.83444618771665,487.86862565772526)]
        self.assertEqual(boundary_intervals(spans,1.),[(spans[0][0],spans[1][1])])
        separated=[spans[0],(488.,488.1)]
        self.assertEqual(boundary_intervals(separated,1.),separated)


if __name__=='__main__':
    unittest.main()
