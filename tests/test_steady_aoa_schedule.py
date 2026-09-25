"""The app's explicitly steady approximation and Windows export contract."""
import copy
import csv
import io
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from em_solver import TrimSolver,settings
from em_plot import write_exports
from instructor_steady_aoa import controller_limits,REVISION


class SteadyScheduleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.solver=TrimSolver('f_16xl',settings(dict(aircraft=['f_16xl'],instructor=True)))
        cls.point=cls.solver.solve(500.,2.,detailed=True)

    def value(self):
        value=copy.deepcopy(self.point['_detail']);value['instructor']=None
        return value

    def test_steady_schedule_has_no_maneuver_dependency(self):
        self.assertTrue(self.point['valid'])
        with patch.dict(sys.modules,{'instructor_continuous':None,'instructor_settle':None}):
            result=controller_limits(self.solver,self.value())
        self.assertEqual(result['model_revision'],REVISION)
        self.assertTrue(result['history_independent'])
        self.assertFalse(result['exact_native_controller'])
        self.assertFalse(result['pitch_predictor_recheck'])

    def test_worst_propulsion_phase_constrains_schedule(self):
        value=self.value();baseline=controller_limits(self.solver,value)
        value=self.value();phase=copy.deepcopy(value['result'])
        phase['history']['wing_aoa']=[a+2. for a in phase['history']['wing_aoa']]
        value['phase_results']=[value['result'],phase]
        result=controller_limits(self.solver,value)
        self.assertAlmostEqual(result['adjusted_wing_angles_deg'][1]-baseline['adjusted_wing_angles_deg'][1],2.,places=4)
        self.assertLess(result['angle_margin'],baseline['angle_margin'])

    def test_low_speed_targets_match_native_critical_multiplier(self):
        from aircraft_model import condition_properties
        value=self.value();result=controller_limits(self.solver,value)
        polar=condition_properties(self.solver.model,value['result']['air']['mach'],value['flaps'])[1]
        from instructor_chart_inputs import source_state
        props=source_state(self.solver,value)['properties']
        factor=props.get('critMult',[-1.,-1.])[0]
        if factor<0:factor=.8
        self.assertFalse(props.get('limitLoadfactor',False))
        self.assertAlmostEqual(result['angle_limits_deg'][1],polar['aoaCritH']*factor,places=5)

    def test_balanced_turn_retains_native_rate_feedback(self):
        import json
        from em_sampling import sample_column
        # Fixed numerical regressions from the recovered PD/forward-balance
        # equations. Observations are a separate cross-check, never cap inputs.
        for name,speed,expected in [('f_16xl',300.,26.1593),('fa_18e_block_2',450.,28.1154)]:
            with self.subTest(aircraft=name):
                config=settings(dict(aircraft=[name],instructor=True))
                column=sample_column((name,json.dumps(config,sort_keys=True),speed,None),boundary_only=True)
                self.assertEqual(column['boundary_status'],'verified limit')
                point=column['boundary'];result=point['instructor']
                self.assertAlmostEqual(point['alpha_deg'],expected,delta=.005)
                self.assertGreater(abs(result['native_angle_rate_rad_s']),.03)
                self.assertLess(result['effective_angle_limits_deg'][1],result['angle_limits_deg'][1]-.9)
                self.assertLess(abs(result['rate_demands'][1]['target_acceleration']-
                                    result['reduced_balances'][1]['acceleration']),.0002)

    def test_selected_upper_speed_does_not_establish_history(self):
        solver=TrimSolver('f_16xl',dict(self.solver.config,speed_max_kmh=1600.))
        self.assertEqual(controller_limits(solver,self.value()),controller_limits(self.solver,self.value()))

    def test_default_is_static(self):
        self.assertEqual(settings()['instructor_model'],'steady')

    def test_low_speed_autotrim_gap_is_solved_with_closed_balance(self):
        solver=TrimSolver('fa_18e_block_2',settings(dict(aircraft=['fa_18e_block_2'],instructor=True)))
        for speed in (150.,155.,159.,160.,165.,170.,171.,175.,180.):
            with self.subTest(speed=speed):
                point=solver.solve(speed,1.,detailed=True)
                self.assertTrue(point['valid'],point['reasons'])
                auto=point['instructor']['static_trim']
                self.assertTrue(auto['success'])
                self.assertLess(abs(auto['equilibrium']['lift_error_n']),20.)
                self.assertLess(abs(auto['equilibrium']['moment_error_nm']),20.)
                self.assertLess(abs(auto['trim'][1]),.5)

    def test_native_success_without_lift_closure_is_rejected(self):
        from instructor_static_trim import equilibrium_valid
        self.assertFalse(equilibrium_valid(dict(success=True,
            equilibrium=dict(lift_error_n=45000.,moment_error_nm=0.))))

    def test_high_speed_cannot_use_free_turn_trim(self):
        from primary_controls import steady_commands
        from instructor_chart_inputs import source_state
        from windows_instructor_source import fixed_source
        from instructor_settings import trim_retained
        solver=TrimSolver('f_16xl',dict(self.solver.config,instructor=False))
        point=solver.solve(1000.,16.,detailed=True)
        self.assertTrue(point['converged'])
        value=point['_detail'];result=controller_limits(solver,value)
        self.assertTrue(result['converged'])
        self.assertLess(result['auto_trim'][1],0.)
        self.assertLess(result['margins']['control authority with auto trim'],0.)
        state=source_state(solver,value);fixed=fixed_source(solver.model,state)
        ground=[solver.fm['AvailableControls'].get('has'+axis+'TrimGroundControl',False)
                for axis in ('Aileron','Elevator','Rudder')]
        controls=dict(solver.controls,trim_available=trim_retained(solver.controls['trim_available'],ground,1,True))
        full=steady_commands(controls,[1.]*3,result['auto_trim'],fixed['ranges'])
        self.assertAlmostEqual(result['control_bounds'][1][1],full[1],places=7)

    def test_failed_auto_trim_is_unresolved_with_compressed_authority(self):
        value=self.solver.solve(1000.,2.,detailed=True)['_detail']
        value['instructor']=None
        with patch('instructor_aoa.static_trim',return_value=dict(success=False,trim=[0.,0.,0.],reason='test')):
            result=controller_limits(self.solver,value)
        self.assertFalse(result['converged'])
        self.assertFalse(result['control_authority_trim_independent'])

    def test_failed_auto_trim_does_not_hide_trim_independent_authority(self):
        with patch('instructor_aoa.static_trim',return_value=dict(success=False,trim=[0.,.7,0.],reason='test')):
            result=controller_limits(self.solver,self.value())
        self.assertTrue(result['converged'])
        self.assertTrue(result['control_authority_trim_independent'])
        self.assertFalse(result['static_trim']['success'])
        self.assertIsNone(result['auto_trim'])
        self.assertIsNone(result['sticks'])

    def test_trim_has_no_speed_history(self):
        from instructor_chart_inputs import source_state
        from windows_instructor_source import fixed_source
        from instructor_static_trim import static_trim
        values=[self.solver.solve(speed,2.,detailed=True)['_detail'] for speed in (500.,900.,1000.)]
        def calculate(value):
            state=source_state(self.solver,value)
            return static_trim(self.solver,state,fixed_source(self.solver.model,state))
        forward=[calculate(value) for value in values]
        self.solver._static_instructor_trim_cache.clear()
        reverse=[calculate(value) for value in reversed(values)]
        self.assertEqual(forward,list(reversed(reverse)))

    def test_cached_preparation_matches_fresh_and_is_not_mutated(self):
        from instructor_chart_inputs import source_state
        cache={}
        for flap in (0.,.3,1.,0.):
            value=self.value();value['flaps']=flap
            fresh=source_state(self.solver,value)
            prepared=source_state(self.solver,value,constant_cache=cache)
            self.assertEqual(prepared,fresh)
            self.solver._effective_aoa_constants=cache
            controller_limits(self.solver,value)
            self.assertEqual(prepared,fresh)


class WindowsExportTests(unittest.TestCase):
    def test_unicode_aircraft_exports_with_legacy_windows_default(self):
        point=dict(speed_kmh=500.,load_g=2.,turn_dps=7.,valid=True,ps_mps=1.,
                   reasons=[],flaps_percent=0.,flaps_requested_percent=0.)
        data=dict(settings=settings(),plot_max_turn=10.,aircraft=[dict(
            id='test',name='\u2417F-16V',points=[point],boundary=[])])
        real_open=Path.open
        def legacy_open(path,mode='r',buffering=-1,encoding=None,errors=None,newline=None):
            if 'b' not in mode and encoding is None:encoding='cp1252'
            return real_open(path,mode,buffering,encoding,errors,newline)
        with tempfile.TemporaryDirectory() as directory:
            with patch.object(Path,'open',legacy_open):
                write_exports(data,Path(directory),figures=False)
            rows=list(csv.DictReader(io.StringIO((Path(directory)/'samples.csv').read_text(encoding='utf-8'))))
            self.assertEqual(rows[0]['aircraft_name'],'\u2417F-16V')
            self.assertTrue((Path(directory)/'ready.json').exists())


if __name__=='__main__':unittest.main()
