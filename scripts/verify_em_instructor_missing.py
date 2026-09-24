"""A physically rejected equilibrium may have no Instructor evaluation."""
import json

from em_sampling import sample_column, worker_solver
from em_solver import settings


def main():
    # Real low-speed refinement reached by the Hunter F9 / CA27 comparison.
    # The aircraft equations converge, but the post-stall state is rejected
    # before the controller is evaluated. This used to crash sample_column.
    name='ca_27_mk_32';speed=197.81233639984129
    config=json.dumps(settings(dict(aircraft=[name],instructor=True)),sort_keys=True)
    solver=worker_solver(name,config)
    point=solver.solve(speed,1.)
    assert point['converged'] and not point['valid'],point
    assert 'post-stall' in point['reasons'] and point['instructor'] is None,point
    column=sample_column((name,config,speed,None))
    level=next(p for p in column['points'] if p['load_g']==1.)
    assert level['converged'] and not level['valid']
    assert level['instructor'] is None and level['reasons']==point['reasons']
    assert column['boundary'] is None and column['lower_boundary'] is None
    assert column['boundary_status']=='no feasible samples',column['boundary_status']
    assert not column['sustained'] and not any(p['valid'] for p in column['points'])
    print('Missing Instructor evaluation: rejected CA27 column retained without a crash or fabricated envelope')


if __name__=='__main__':main()
