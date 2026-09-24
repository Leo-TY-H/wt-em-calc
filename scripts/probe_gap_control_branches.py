"""Explore separated trim branches with unchanged equilibrium criteria."""
import json,time,math
from pathlib import Path
import numpy as np
from em_solver import TrimSolver
root=Path('analysis/equilibrium-gap-recovery');fixtures=json.loads((root/'fixtures.json').read_text())['fixtures'];rows=[];start=time.monotonic()
for index in [3,4,5,6,7,0,9,11,20]:
    f=fixtures[index];s=TrimSolver(f['aircraft'],f['settings']);p=f['bad'];success=[];best=p;attempts=0
    for alpha in [p['alpha_deg'],f['upper']['alpha_deg'],15.,22.,30.,34.,37.]:
        for pitch in [-.98,-.8,-.6,-.4,-.2,0.,.2,.4,.6,.8,.98]:
            for zero_lateral in [False,True]:
                x=list(p['solution']);x[0]=alpha;x[3]=pitch
                if zero_lateral:x[2]=x[4]=0.
                q=s.solve(p['speed_kmh'],p['load_g'],x,exhaustive=False);attempts+=1
                if q['valid']:success.append(q)
                if q['force_error_g']+10*q['angular_error_rad_s2']<best['force_error_g']+10*best['angular_error_rad_s2']:best=q
    rows.append(dict(index=index,aircraft=s.name,attempts=attempts,solutions=success,best=best))
    (root/'control-branch-audit.json').write_text(json.dumps(dict(rows=rows,elapsed_s=time.monotonic()-start),indent=2))
    print(index,s.name,'valid',len(success),'best F',best['force_error_g'],'reasons',best['reasons'],flush=True)
