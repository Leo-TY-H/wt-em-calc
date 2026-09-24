"""Regression for duplicate interpolation knots and general jet redlines."""
import json
from pathlib import Path
import numpy as np
from scipy.interpolate import PchipInterpolator
from em_solver import AIRCRAFT,settings
from em_sampling import interpolation_knots,load_coordinate,sample_column,worker_solver
from em_speed_limits import speed_limits
from jet_catalog import load

OUT=Path('analysis/speed-boundary-fixes')

def main():
    f=json.loads((OUT/'interpolation-failure.json').read_text())
    x,y=interpolation_knots(f['x'],[p['ps_mps'] for p in f['points']])
    assert len(x)==len(f['x'])-1 and np.all(np.diff(x)>0)
    curve=PchipInterpolator(x,y)
    assert np.max(abs(curve(x)-y))<1e-8
    # Re-run the actual failed column with its ORIGINAL native pointwise
    # limits, bypassing the new plot cutoff in this diagnostic only. This
    # proves the interpolation fix independently of merely hiding the speed.
    cfg=settings(dict(aircraft=['a_4b'],speed_samples=9,load_samples=7,sep_tolerance_mps=1.))
    encoded=json.dumps(cfg,sort_keys=True);solver=worker_solver('a_4b',encoded)
    solver.plot_speed_limits=dict(speed_limits(solver.fm,cfg),enforced=False)
    column=sample_column(('a_4b',encoded,f['speed'],None))
    assert column['boundary_status']=='verified limit'
    (OUT/'a4-original-failure-column-fixed.json').write_text(json.dumps(column,indent=2)+'\n')
    # Exercise every enabled family at three atmospheric conditions, including
    # swept-wing interpolation. Inversion must respect both reference limits.
    rows=[]
    for name,info in AIRCRAFT.items():
        if not info['supported']:continue
        fm=load(name)
        for height in [0.,5000.,10000.]:
            for sweep in ([0.,50.,100.] if info['has_sweep'] else [0.]):
                c=settings(dict(aircraft=[name],altitude_m=height,sweep_percent=sweep))
                limit=speed_limits(fm,c)
                assert 0<limit['sample_speed_kmh']<limit['speed_kmh']
                assert limit['speed_kmh']==min(limit['vne_tas_kmh'],limit['mne_tas_kmh'])
                rows.append(dict(aircraft=name,altitude_m=height,sweep=sweep,**limit))
    # An excluded column must never ask the equilibrium solver for a point.
    solver.plot_speed_limits=speed_limits(solver.fm,cfg)
    old=solver.solve
    def forbidden(*args,**kwargs):raise AssertionError('Equilibrium solve past redline')
    solver.solve=forbidden
    try:
        excluded=sample_column(('a_4b',encoded,1300.,None))
        assert excluded['boundary_status']=='speed limit' and not excluded['points']
    finally:solver.solve=old
    report=dict(duplicate_knots_removed=len(f['x'])-len(x),reproduced_speed_kmh=f['speed'],
        recovered_column=column['boundary_status'],catalog_cases=len(rows),catalog=rows,
        no_solve_above_redline=True,failures=[])
    (OUT/'speed-limit-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print('Original A-4 failure column passes; catalog redlines',len(rows),'cases; excluded speed performs no solve',flush=True)

if __name__=='__main__':main()
