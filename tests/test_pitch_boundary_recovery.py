"""Recover narrow pitch-response limits without relaxing aircraft admission."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from em_pitch_limit import limit


def point(command):
    normal = command < .501
    return dict(speed_kmh=855., load_g=6.-10.*(command-.501)**2,
        alpha_deg=3., solution=[3., 80., 0., command, 0.],
        valid=normal, converged=True, reasons=[] if normal else ['reversed pitch response'],
        pitch_response=dict(margin=.501-command), turn_dps=12., ps_mps=100.*command,
        force_error_g=1e-5, angular_error_rad_s2=1e-6)


class PitchBoundaryProbeTests(unittest.TestCase):
    def solver(self):
        return SimpleNamespace(config=dict(trim_solver_mode='stationary',sep_tolerance_mps=.5),
            is_prop=False)

    def test_contracts_probe_that_jumps_past_narrow_maximum(self):
        commands=[]
        class Curve:
            def __init__(self,*args):pass
            def correct(self,seed,*,elevator,max_nfev):
                commands.append(elevator)
                return point(elevator)
        with patch('em_stationary_boundary.Curve',Curve):
            edge=limit(self.solver(),855.,point(.5))
        self.assertIsNotNone(edge)
        self.assertIn(.51,commands)
        self.assertTrue(any(.5<c<.502 for c in commands))
        self.assertTrue(edge['valid'])
        self.assertGreaterEqual(edge['load_g'],point(.5)['load_g'])
        proof=edge['envelope_limit']
        self.assertGreater(proof['constraint_residual'],0.)
        self.assertLess(proof['rejected_constraint_residual'],0.)
        self.assertLessEqual(abs(proof['ps_interval_mps'][1]-proof['ps_interval_mps'][0]),.1)

    def test_unclosed_probes_cannot_form_physical_rejection(self):
        class Curve:
            def __init__(self,*args):pass
            def correct(self,seed,*,elevator,max_nfev):
                return point(elevator) if elevator<.501 else None
        with patch('em_stationary_boundary.Curve',Curve):
            self.assertIsNone(limit(self.solver(),855.,point(.5)))

    def test_other_physical_rejection_is_not_a_pitch_response_bracket(self):
        class Curve:
            def __init__(self,*args):pass
            def correct(self,seed,*,elevator,max_nfev):
                p=point(elevator)
                if not p['valid']:p['reasons'].append('wing force limit')
                return p
        with patch('em_stationary_boundary.Curve',Curve):
            self.assertIsNone(limit(self.solver(),855.,point(.5)))

    def test_cancellation_propagates(self):
        with patch('em_stationary_boundary.Curve') as curve:
            curve.return_value.correct.side_effect=InterruptedError('cancelled')
            with self.assertRaises(InterruptedError):limit(self.solver(),855.,point(.5))


class EarlyPitchRecognitionTests(unittest.TestCase):
    def recognize(self,*,progress=True,margin=.01,nearest='pitch response',
                  candidate_kinds=None,edge_change=None,verified=True,found=True,high_converged=False):
        from em_stationary_boundary import from_bracket
        low=dict(point(.5),load_g=5.,pitch_response=dict(margin=.1))
        trial=dict(low,load_g=6.,alpha_deg=3.2,pitch_response=dict(margin=margin))
        high=dict(low,load_g=8.,valid=False,converged=high_converged,reasons=[])
        edge=dict(trial,load_g=6.1,alpha_deg=3.3,envelope_limit=dict(kind='pitch response'))
        edge.update(edge_change or {})
        fallback=dict(trial,load_g=6.2,envelope_limit=dict(kind='trim fold'))
        solver=SimpleNamespace(solve=Mock(return_value=trial),config={},sideslip_attitude_deg=.3)
        curve=Mock(cap=20.)
        curve._solver_at.return_value=solver
        curve.nearest_events.return_value=[(nearest,None,lambda v:0.)]
        with patch('em_stationary_boundary.Curve',return_value=curve), \
             patch('em_accuracy.load_interval_resolved',side_effect=[not progress,True]), \
             patch('em_sampling.continuation_initial',return_value=low), \
             patch('em_pitch_limit.limit',return_value=edge if found else None) as pitch, \
             patch('em_stationary_boundary.finish_edge',return_value=edge if verified else None) as finish, \
             patch('em_stationary_boundary.follow_branch',return_value=fallback) as follow:
            result=from_bracket(solver,855.,low,high,candidate_kinds=candidate_kinds)
        return result,pitch,finish,follow,solver

    def test_approaching_response_limit_uses_balanced_command_bracket(self):
        result,pitch,finish,follow,solver=self.recognize()
        self.assertEqual(result['envelope_limit']['kind'],'pitch response')
        pitch.assert_called_once()
        self.assertIs(pitch.call_args.args[0],solver)
        self.assertEqual(pitch.call_args.args[0].sideslip_attitude_deg,.3)
        finish.assert_called_once()
        follow.assert_not_called()

    def test_search_hint_requires_progress_toward_nearest_response_face(self):
        for settings in (dict(progress=False),dict(margin=.2),dict(margin=0.),
                         dict(nearest='control'),dict(candidate_kinds={'stall'}),
                         dict(high_converged=True)):
            with self.subTest(settings=settings):
                result,pitch,finish,follow,_=self.recognize(**settings)
                pitch.assert_not_called()
                finish.assert_not_called()
                follow.assert_called_once()
                self.assertEqual(result['envelope_limit']['kind'],'trim fold')

    def test_candidate_must_remain_admitted_inside_forward_bracket(self):
        for change in (dict(valid=False),dict(load_g=5.9),dict(load_g=8.1),dict(alpha_deg=3.1)):
            with self.subTest(change=change):
                result,pitch,finish,follow,_=self.recognize(edge_change=change)
                pitch.assert_called_once()
                finish.assert_not_called()
                follow.assert_called_once()
                self.assertEqual(result['envelope_limit']['kind'],'trim fold')

    def test_unverified_intervening_branch_keeps_general_recovery(self):
        result,pitch,finish,follow,_=self.recognize(verified=False)
        pitch.assert_called_once()
        finish.assert_called_once()
        follow.assert_called_once()
        self.assertEqual(result['envelope_limit']['kind'],'trim fold')

    def test_inconclusive_command_search_keeps_general_recovery(self):
        result,pitch,finish,follow,_=self.recognize(found=False)
        pitch.assert_called_once()
        finish.assert_not_called()
        follow.assert_called_once()
        self.assertEqual(result['envelope_limit']['kind'],'trim fold')


class BIPitchBoundaryTests(unittest.TestCase):
    def test_response_limits_are_verified_before_broad_sideslip_recovery(self):
        from em_solver import settings
        from em_sampling import sample_column,sample_with_history,worker_solver,recover_sideslip_boundary
        self.addCleanup(worker_solver.cache_clear)
        for name,speed,expected in (('bi',863.4891914603496,5.4574),('f_16xl',150.,1.2475)):
            with self.subTest(aircraft=name,speed=speed):
                config=settings(dict(aircraft=[name],instructor=False,torque_gyro=True,
                    engine_control_mode='automatic'))
                worker_solver.cache_clear()
                with patch('em_sampling.recover_sideslip_boundary',wraps=recover_sideslip_boundary) as recovery:
                    column,_=sample_with_history(sample_column,
                        (name,json.dumps(config,sort_keys=True),speed,None),{})
                recovery.assert_not_called()
                self.assertEqual(column['boundary_status'],'verified limit')
                p=column['boundary'];proof=p['envelope_limit']
                self.assertTrue(p['valid'])
                self.assertEqual(proof['kind'],'pitch response')
                self.assertAlmostEqual(p['load_g'],expected,delta=.001)
                self.assertEqual(proof['rejected_endpoint']['reasons'],['reversed pitch response'])
                self.assertLessEqual(p['force_error_g'],2e-4)
                self.assertLessEqual(p['angular_error_rad_s2'],5e-5)
                self.assertLessEqual(p['history_error'],2e-4)
                self.assertLess(column['search_work']['aircraft_evaluations'],4000)
                self.assertFalse(column['unresolved_load_intervals'])

    def test_control_stop_does_not_bypass_intervening_maximum_check(self):
        from em_stationary_boundary import finish_edge
        seed=point(.5)
        edge=dict(seed,envelope_limit=dict(kind='control',axis=3))
        middle=dict(seed,load_g=seed['load_g']+.1)
        curve=SimpleNamespace(correct=Mock(return_value=middle),retain=Mock())
        with patch('em_stationary_boundary.peak',return_value=None) as peak:
            self.assertIsNone(finish_edge(curve,[seed],edge))
        peak.assert_called_once()
        curve.retain.assert_called_once()

    def test_reached_control_stop_is_not_shifted_into_interior(self):
        from em_stationary_boundary import Curve
        curve=object.__new__(Curve)
        curve.solver=SimpleNamespace(config=dict(instructor=False))
        curve.nearest_events=Mock(return_value=[('control',3,lambda v:0.)])
        curve.correct=Mock(return_value=None)
        seed=dict(point(.5),commands=[0.,1.,0.],control_bounds=[[-1.,1.]]*3)
        edge=curve.active(seed,.1)
        self.assertIsNotNone(edge)
        self.assertEqual(edge['load_g'],seed['load_g'])
        self.assertEqual(edge['envelope_limit']['control_command'],1.)
        curve.correct.assert_not_called()
        for change in (dict(valid=False),dict(converged=False),dict(commands=[0.,.99999,0.])):
            self.assertIsNone(curve.active(dict(seed,**change),.1))

    def test_default_sb_control_stop_avoids_broad_recovery(self):
        from em_solver import settings
        from em_sampling import sample_column,sample_with_history,worker_solver
        config=settings(dict(aircraft=['bi'],instructor=False,torque_gyro=True,
            engine_control_mode='automatic'))
        worker_solver.cache_clear()
        self.addCleanup(worker_solver.cache_clear)
        column,_=sample_with_history(sample_column,
            ('bi',json.dumps(config,sort_keys=True),919.9964610351562,None),{})
        self.assertEqual(column['boundary_status'],'verified limit')
        p=column['boundary']
        self.assertTrue(p['valid'])
        self.assertEqual(p['envelope_limit']['kind'],'control')
        self.assertEqual(p['envelope_limit']['axis'],3)
        self.assertEqual(p['commands'][1],p['control_bounds'][1][1])
        self.assertAlmostEqual(p['load_g'],5.4675,delta=.001)
        self.assertLessEqual(p['force_error_g'],2e-4)
        self.assertLessEqual(p['angular_error_rad_s2'],5e-5)
        self.assertLessEqual(p['history_error'],2e-4)
        self.assertLess(column['search_work']['aircraft_evaluations'],3000)

    def test_default_sb_narrow_response_limits(self):
        from em_solver import settings
        from em_sampling import sample_column,sample_with_history,worker_solver
        config=settings(dict(aircraft=['bi'],instructor=False,torque_gyro=True,
            engine_control_mode='automatic'))
        for speed in (855.1269572686064,877.825406650178):
            with self.subTest(speed=speed):
                worker_solver.cache_clear()
                column,_=sample_with_history(sample_column,
                    ('bi',json.dumps(config,sort_keys=True),speed,None),{})
                self.assertEqual(column['boundary_status'],'verified limit')
                p=column['boundary'];proof=p['envelope_limit']
                self.assertTrue(p['valid'])
                self.assertEqual(proof['kind'],'pitch response')
                self.assertGreater(p['load_g'],5.46)
                self.assertLess(p['load_g'],5.49)
                self.assertLessEqual(p['force_error_g'],2e-4)
                self.assertLessEqual(p['angular_error_rad_s2'],5e-5)
                self.assertLessEqual(p['history_error'],2e-4)
                self.assertEqual(proof['rejected_endpoint']['reasons'],['reversed pitch response'])
                self.assertLessEqual(proof['rejected_endpoint']['force_error_g'],2e-4)
                self.assertLessEqual(proof['rejected_endpoint']['angular_error_rad_s2'],5e-5)
                self.assertLessEqual(max(proof['turn_interval_dps'])-min(proof['turn_interval_dps']),.02)
                self.assertFalse(column['unresolved_load_intervals'])
        worker_solver.cache_clear()


if __name__=='__main__':unittest.main()
