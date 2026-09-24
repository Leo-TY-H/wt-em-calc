"""Close a local load maximum on a continuously traced pitch-trim branch."""
import math
import numpy as np
from scipy.optimize import least_squares, minimize_scalar


def trim_fold(solver,speed,last):
    """Use pitch command as the coordinate when fixed-load trim becomes singular.

    All five original balances are solved with load as an unknown. A local
    maximum must be bracketed by valid, lower-load equilibria on both sides.
    Failure of the ordinary solver alone is never evidence of a fold.
    """
    if last['stall_margin_deg']<.1 or last['authority_margin']<.05:return None
    memo={};center=last['solution'][3]
    def at(command):
        if command in memo:return memo[command]
        near=min(memo.values(),key=lambda p:abs(p['solution'][3]-command)) if memo else last
        x=near['solution'];n=near['load_g'];initial=np.array([x[0],x[1],x[2],x[4],math.sqrt(max(0.,(n-1.)*(n+1.)))])
        def value(z):return solver.operating_point(speed/3.6,math.hypot(1.,z[4]),[z[0],z[1],z[2],command,z[3]])
        def fun(z):return value(z)['residual']
        def jac(z):
            base=value(z);columns=[]
            for i,h in enumerate([.002,.002,.0002,.0002,.0002]):
                for factor in (1.,.5,-.5,2.,-1.,.1,-.1):
                    q=z.copy();q[i]+=h*factor
                    if i==4 and q[i]<0.:continue
                    trial=value(q);used_step=h*factor
                    if solver.derivative_branch(trial)==solver.derivative_branch(base):break
                columns.append((trial['residual']-base['residual'])/used_step)
            return np.column_stack(columns)
        bounds=tuple([b[0],b[1],b[2],b[4],u] for b,u in zip(solver.trim_bounds,(0.,math.sqrt(64.**2-1.))))
        fit=least_squares(fun,initial,jac=jac,bounds=bounds,
            x_scale=[10.,30.,.2,.2,1.],max_nfev=30,gtol=1e-10,xtol=1e-9,ftol=1e-10)
        z=fit.x;v=value(z)
        if v['force_error_g']>1e-5 or max(abs(v['rate_residual']))>5e-6:raise ValueError('Unbalanced fold probe')
        point=solver.solve(speed,math.hypot(1.,z[4]),[z[0],z[1],z[2],command,z[3]],exhaustive=False)
        if (not point['valid'] or abs(point['solution'][3]-command)>1e-5
                or abs(point['alpha_deg']-last['alpha_deg'])>2.):raise ValueError('Disconnected fold probe')
        memo[command]=point
        return point
    # A fixed command interval can jump out of the local trim branch before
    # the maximizer has even sampled its center. Contract the search region
    # on failed probes, retaining the same balanced two-sided fold evidence.
    for radius in (.15,.05,.015,.005):
        bounds=(max(-.999,center-radius),min(.999,center+radius))
        separation=radius*(.4 if radius<.015 else .2)
        try:
            fit=minimize_scalar(lambda command:-at(command)['load_g'],bounds=bounds,method='bounded',
                                options=dict(xatol=1e-5,maxiter=25))
            if not bounds[0]+separation<fit.x<bounds[1]-separation:continue
            point=at(fit.x)
        except (ValueError,RuntimeError):continue
        if abs(point['load_g']-last['load_g'])>.005:continue
        # A narrow command interval can have a real maximum but too little
        # load drop to certify it above the numerical noise floor. Probe farther
        # out inside the SAME searched interval before abandoning the maximum.
        # Preserve the original probes first and every acceptance threshold.
        wider=min(radius*.4,.8*min(fit.x-bounds[0],bounds[1]-fit.x))
        for distance in dict.fromkeys((separation,max(separation,wider))):
            try:sides=[at(fit.x-distance),at(fit.x+distance)]
            except (ValueError,RuntimeError):continue
            if any(p['force_error_g']>5e-6 or p['angular_error_rad_s2']>5e-6 for p in [point,*sides]):continue
            if any(point['load_g']-p['load_g']<.0003 for p in sides):continue
            point['envelope_limit']=dict(kind='trim fold',axis=3,limiting_load_g=point['load_g'],
                method='balanced local load maximum in pitch-command coordinates',
                command_bracket=[p['solution'][3] for p in sides],bracket_loads_g=[p['load_g'] for p in sides],
                evaluations=len(memo))
            return point
    return None
