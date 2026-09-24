"""Production small-sideslip recovery, state replay and mode invariance."""
import json
import multiprocessing
import time
from concurrent.futures import ProcessPoolExecutor,as_completed
from pathlib import Path
import numpy as np
from em_solver import TrimSolver
from em_sideslip import recover_sideslip


def one(pair):
    index,fixture=pair
    solver=TrimSolver(fixture['aircraft'],fixture['settings']);bad=fixture['bad']
    point=recover_sideslip(solver,bad['speed_kmh'],bad['load_g'],bad,[fixture['lower'],fixture['upper']])
    row=dict(index=index,aircraft=solver.name,point=point)
    if point:
        value=solver.point_value(point)
        assert value['result']['force']==point['force_n']
        assert value['result']['stored_moment']==point['moment_nm']
        assert value['ps']==point['ps_mps']
        assert value['force_error_g']<=2e-4 and max(abs(value['rate_residual']))<=5e-5
        assert abs(value['kinematic']['velocity'][1])<=.005
        assert abs(point['sideslip_deg'])<=2.
        on=TrimSolver(solver.name,dict(fixture['settings'],instructor=True)).at_sideslip(point['sideslip_attitude_deg'])
        replay=on.operating_point(bad['speed_kmh']/3.6,bad['load_g'],point['solution'])
        assert np.array_equal(value['result']['force'],replay['result']['force'])
        assert np.array_equal(value['result']['stored_moment'],replay['result']['stored_moment'])
        assert value['ps']==replay['ps']
        row['replay_and_mode_invariance']=True
        # Full Instructor solve for representative wing-rate/canard recoveries.
        if index in [0,8,11]:
            classified=on.solve(bad['speed_kmh'],bad['load_g'],point['solution'],exhaustive=False)
            row['instructor']=classified
            assert classified['converged']
            assert classified['instructor'] is not None and classified['instructor']['converged']
    return row


def main():
    fixtures=json.loads(Path('analysis/equilibrium-gap-recovery/fixtures.json').read_text())['fixtures']
    start=time.monotonic();rows=[]
    with ProcessPoolExecutor(max_workers=4,mp_context=multiprocessing.get_context('spawn')) as pool:
        for future in as_completed([pool.submit(one,item) for item in enumerate(fixtures)]):
            row=future.result();rows.append(row)
            Path('analysis/sideslip-integration/recovery-validation.json').write_text(json.dumps(dict(rows=rows,seconds=time.monotonic()-start),indent=2)+'\n')
            print(row['index'],row['aircraft'],'recovered',bool(row['point']),flush=True)
    assert all(row['point'] for row in rows if row['index'] not in range(3,8))
    print('All 21 previously recoverable fixtures pass production recovery/replay checks',flush=True)


if __name__=='__main__':main()
