"""Alternative numerical coordinates for the unchanged five balance equations.

No aircraft-specific thresholds, altered forces or relaxed closure criteria.
The scalar search may bracket a discontinuity; only a subsequent complete
force/moment solve can accept a point. A sign change alone is never evidence.
"""
import numpy as np
from scipy.optimize import least_squares,brentq


def recover_equilibrium(solver,speed_kmh,load,neighbors):
    """Resolve bank/controls at fixed alpha, then close the remaining force.

    Called only after regular continuation/restarts fail, with independently
    valid solutions bracketing the requested load. Solving the four transverse
    and moment constraints first avoids an ill-conditioned coupled Newton step.
    """
    neighbors=[p for p in neighbors if p['valid']]
    below=[p for p in neighbors if p['load_g']<load];above=[p for p in neighbors if p['load_g']>load]
    if not below or not above:return None
    left=max(below,key=lambda p:p['load_g']);right=min(above,key=lambda p:p['load_g'])
    amin=max(solver.trim_bounds[0][0],min(left['alpha_deg'],right['alpha_deg'])-.05)
    amax=min(solver.trim_bounds[1][0],max(left['alpha_deg'],right['alpha_deg'])+.05)
    cache={};best=None
    def at(alpha):
        nonlocal best
        alpha=float(alpha)
        if alpha in cache:return cache[alpha]
        good=[(abs(a-alpha),z) for a,(z,v) in cache.items() if v['history_error']<=2e-4 and max(abs(v['residual'][1:]))<.0002]
        seed=min(good,key=lambda x:x[0])[1] if good else min([left,right],key=lambda p:abs(p['alpha_deg']-alpha))['solution'][1:]
        memo={}
        def value(z):
            key=tuple(z)
            if key not in memo:memo[key]=solver.operating_point(speed_kmh/3.6,load,[alpha,*z])
            return memo[key]
        def fun(z):return value(z)['residual'][1:]
        def jac(z):
            base=value(z);cols=[]
            for i,h in enumerate([.002,.0002,.0002,.0002]):
                for factor in [1.,.5,-.5,2.,-1.,.1,-.1,.01,-.01]:
                    t=z.copy();t[i]+=h*factor;v=value(t)
                    if solver.derivative_branch(v)==solver.derivative_branch(base):break
                cols.append((v['residual'][1:]-base['residual'][1:])/(h*factor))
            return np.column_stack(cols)
        bounds=(np.array([-15.,-1.,-1.,-1.]),np.array([89.7,1.,1.,1.]))
        z=np.clip(seed,bounds[0]+1e-8,bounds[1]-1e-8)
        fit=least_squares(fun,z,jac=jac,bounds=bounds,x_scale=[30.,.2,.2,.2],max_nfev=25,ftol=1e-10,xtol=1e-9,gtol=1e-10)
        v=value(fit.x);cache[alpha]=(fit.x,v)
        if v['force_error_g']<=2e-4 and max(abs(v['rate_residual']))<=5e-5 and v['history_error']<=2e-4:
            p=solver.solve(speed_kmh,load,[alpha,*fit.x],exhaustive=False)
            if p['valid']:best=p
        return cache[alpha]
    def scalar(alpha):
        _,v=at(alpha)
        if abs(v['residual'][1])>2e-4 or max(abs(v['rate_residual']))>5e-5 or v['history_error']>2e-4:
            raise ValueError('Unbalanced bank/control subproblem')
        return float(v['residual'][0])
    probes=sorted(set(np.linspace(amin,amax,9).tolist()+[left['alpha_deg'],right['alpha_deg']]))
    previous=None
    for alpha in probes:
        try:r=scalar(alpha)
        except ValueError:previous=None;continue
        if best:return best
        if previous and previous[1]*r<=0:
            try:brentq(scalar,previous[0],alpha,xtol=1e-10,maxiter=32)
            except (ValueError,RuntimeError):pass
            if best:return best
        previous=(alpha,r)
    return best
