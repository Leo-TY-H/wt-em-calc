"""Low-speed initialization, bounded recovery work and retained gap evidence."""
import json
from collections import Counter
from pathlib import Path
from unittest.mock import patch

import numpy as np
import em_sampling
from em_solver import TrimSolver, settings

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'analysis/f16xl-low-speed'


def replay(solver,point):
    value=solver.point_value(point)
    assert value['result']['force']==point['force_n']
    assert value['result']['stored_moment']==point['moment_nm']
    assert abs(value['ps']-point['ps_mps'])<1e-9
    assert value['force_error_g']<=2e-4
    assert max(abs(value['rate_residual']))<=5e-5
    assert value['history_error']<=2e-4


def main():
    cfg=settings(dict(aircraft=['f_16xl'],instructor=False,speed_samples=17,
                      load_samples=9,sep_tolerance_mps=.5))
    solver=TrimSolver('f_16xl',cfg)
    level=em_sampling.solve_level(solver,150.)
    assert level['valid'] and 19.<level['alpha_deg']<22.
    replay(solver,level)
    calls=Counter();recover=em_sampling.recover_sideslip
    def counted(solver,speed,load,*args,**kwargs):
        calls[load]+=1
        return recover(solver,speed,load,*args,**kwargs)
    with patch.object(em_sampling,'recover_sideslip',counted):
        column=em_sampling.sample_column(('f_16xl',json.dumps(cfg,sort_keys=True),176.953125,None))
    assert column['boundary_status']=='verified limit'
    assert abs(column['boundary']['load_g']-1.845013704984517)<.0002
    # Regression for the former 26 repeated full searches in this column.
    assert max(calls.values(),default=0)<=1,calls
    assert sum(calls.values())<=8,calls
    assert len(column['numerical_gap_brackets'])==1
    gap=column['numerical_gap_brackets'][0]
    assert abs(gap['valid_side_loads'][0]-1.752706375974749)<=.0002
    assert abs(gap['valid_side_loads'][1]-1.768759263894453)<=.0002
    assert all(abs(a-b)<=.0002 for a,b in gap['edge_brackets_g'])
    assert np.isnan(em_sampling.column_at_load(column,[sum(gap['valid_side_loads'])/2.])[0])
    valid=[p for p in column['points'] if p['valid']]
    for point in valid:replay(solver,point)
    # Recovered sideslip is part of the physical seed, with the same bound.
    continued=[p for p in valid if p.get('recovery_method')=='balanced nearby-sideslip continuation']
    assert continued and all(abs(p['sideslip_deg'])<=2. for p in continued)
    report=dict(status='PASS',level_alpha_deg=level['alpha_deg'],level=level,
        column_s=column['elapsed_s'],full_recovery_calls=sum(calls.values()),
        replayed_states=1+len(valid),boundary_load_g=column['boundary']['load_g'],gap=gap,
        checks=['150 km/h level equilibrium', 'one broad recovery per failed load',
                'unchanged verified upper limit','gap edges within existing 0.0002 g resolution',
                'unresolved gap remains masked','all accepted states replay at original tolerances',
                'bounded sideslip continuation'])
    OUT.mkdir(exist_ok=True)
    (OUT/'regression-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    column.pop('_curve',None)
    (OUT/'column-176-final.json').write_text(json.dumps(column)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='level'},indent=2))


if __name__=='__main__':main()
