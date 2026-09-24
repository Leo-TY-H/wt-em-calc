"""Optimize existing closure tolerances directly; no relaxed force model."""
import json,time
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from em_solver import TrimSolver

root=Path('analysis/equilibrium-gap-recovery');fixtures=json.loads((root/'fixtures.json').read_text())['fixtures']
rows=[];start=time.monotonic()
for index,f in enumerate(fixtures):
    s=TrimSolver(f['aircraft'],f['settings']);p=f['bad'];speed=p['speed_kmh']/3.6;load=p['load_g'];scale=np.array([2e-4,2e-4,5e-4,5e-4,5e-4]);memo={}
    def detail(x):
        k=tuple(x)
        if k not in memo:
            if len(memo)>512:memo.clear()
            memo[k]=s.operating_point(speed,load,x)
        return memo[k]
    best=None;solutions=[]
    for hinge in [False,True]:
        def fun(x):
            r=detail(x)['residual']/scale
            return np.sign(r)*np.maximum(0.,abs(r)-.85) if hinge else r
        def jac(x):
            r=fun(x);columns=[]
            for i,h in enumerate([.0002,.0002,.00002,.00002,.00002]):
                dx=np.eye(5)[i]*h;columns.append((fun(x+dx)-r)/h)
            return np.column_stack(columns)
        for key in ['bad','lower','upper']:
            x=f[key]['solution'];r=least_squares(fun,x,jac=jac,bounds=([-6.,-15.,-1.,-1.,-1.],[42.,89.7,1.,1.,1.]),
                x_scale=[10.,30.,.2,.2,.2],max_nfev=100,ftol=1e-12,xtol=1e-12,gtol=1e-10)
            d=detail(r.x);error=float(max(abs(d['residual']/scale)));q=dict(solution=r.x.tolist(),error=error,residual=d['residual'].tolist(),ps=d['ps'],hinge=hinge,seed=key)
            if best is None or error<best['error']:best=q
            if error<=1 and d['history_error']<=2e-4 and d['stall_margin']>=0 and d['allocation']['reachable'] and abs(d['kinematic']['velocity'][1])<=.005:solutions.append(q)
    row=dict(index=index,aircraft=s.name,solutions=solutions,best=best);rows.append(row)
    (root/'feasibility-probe.json').write_text(json.dumps(dict(rows=rows,elapsed_s=time.monotonic()-start),indent=2))
    print(index,s.name,'solutions',len(solutions),'best normalized residual',best['error'],flush=True)
