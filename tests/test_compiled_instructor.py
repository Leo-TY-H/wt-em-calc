"""Compare new Cython helpers with their Python sources on identical states."""
import copy
import importlib.util
from pathlib import Path
import sys
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from em_solver import TrimSolver,settings,BACKEND
import instructor_aoa as compiled


def reference(name):
    spec=importlib.util.spec_from_file_location('reference_'+name,ROOT/'scripts'/(name+'.py'))
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@unittest.skipUnless(BACKEND=='compiled','requires compiled backend')
class CompiledInstructorTests(unittest.TestCase):
    def test_identical_phase_states_and_control_stops(self):
        python=reference('instructor_aoa')
        python.source_state=reference('instructor_chart_inputs').source_state
        python.fixed_source=reference('windows_instructor_source').fixed_source
        python.required_acceleration=reference('instructor_aoa_balance').required_acceleration
        import instructor_aoa_balance as balance
        from instructor_predictor_inputs import pack_pitch_inputs
        from instructor_pitch_predictor import unpack_inputs
        self.assertFalse(compiled.__file__.endswith('.py'))
        cases=[('a6m2_zero_china',310.,3.),('a6m2_zero_china',488.,10.),
               ('saab_jas39c',140.,1.),('saab_jas39c',1000.,10.),
               ('f_16a_block_15_adf',600.,3.)]
        for name,speed,load in cases:
            with self.subTest(aircraft=name,speed=speed):
                solver=TrimSolver(name,settings(dict(aircraft=[name],instructor=False,roll_leveling=False)))
                point=solver.solve(speed,load,detailed=True)
                self.assertTrue(point['converged'])
                value=point['_detail']
                for phase in value.get('phase_results') or [value['result']]:
                    fresh=dict(value,result=phase,instructor=None)
                    actual=compiled.controller_limits(solver,copy.deepcopy(fresh))
                    expected=python.controller_limits(solver,copy.deepcopy(fresh))
                    self.assertEqual(actual,expected)
                    state=python.source_state(solver,fresh)
                    self.assertEqual(compiled.source_state(solver,fresh),state)
                    fixed=python.fixed_source(solver.model,state)
                    self.assertEqual(compiled.fixed_source(solver.model,state),fixed)
                    demand=actual['rate_demands'][1]
                    packed=pack_pitch_inputs(**state['wrapper'],**demand,body_pitch_rate=state['pitch_rate'],
                        angle_bounds=fixed['tail_bounds'],axis_weights=[0.,0.,0.],
                        quaternion=state['quaternion'],world_velocity=state['world_velocity'])
                    ip=unpack_inputs(packed)
                    for pitch in (-1.,0.,1.,state['delivered'][1]):
                        self.assertEqual(balance.required_acceleration(solver.model,ip,state['predictor'],pitch),
                            python.required_acceleration(solver.model,ip,state['predictor'],pitch))
                    for angle in (-12.00001,-12.,-11.99999,11.99999,12.,12.00001):
                        edge=dict(ip)
                        edge[4]=angle
                        self.assertEqual(balance.required_acceleration(solver.model,edge,state['predictor'],1.),
                            python.required_acceleration(solver.model,edge,state['predictor'],1.))


if __name__=='__main__':unittest.main()
