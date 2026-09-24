"""Interior continuation with angle fixed and load solved from the full balance."""
import math
import numpy as np
from scipy.optimize import least_squares


def _alpha_point(solver,speed,left,right,fraction):
    """Return a directly certified midpoint of one continuous trim segment.

    This changes numerical unknowns only. Both endpoints must be balanced and
    ordered in load and angle. The ordinary solver independently checks the
    resulting load, including the full propulsion phase certificate.
    """
    if not (left['valid'] and right['valid'] and left['load_g']<right['load_g'] and
            left['alpha_deg']+1e-5<right['alpha_deg']):return None
    if any(abs(p.get('sideslip_attitude_deg',0.))>1e-8 for p in (left,right)):return None
    alpha=left['alpha_deg']*(1-fraction)+right['alpha_deg']*fraction
    guess=np.array(left['solution'])*(1-fraction)+np.array(right['solution'])*fraction
    turn=lambda n:math.sqrt(max(0.,(n-1.)*(n+1.)))
    initial=np.r_[guess[1:],turn(left['load_g']*(1-fraction)+right['load_g']*fraction)]
    initial[0]=math.degrees(math.atan(initial[4]))
    bounds=tuple([*b[1:],turn(n)] for b,n in zip(solver.trim_bounds,(left['load_g'],right['load_g'])))
    memo={}
    def value(z,frozen=None):
        key=tuple(z)
        # u = sqrt(n^2-1) is proportional to turn rate at this speed. A fixed
        # difference in n crosses a singular derivative at 1 g; u stays finite.
        load=math.hypot(1.,z[4])
        if frozen is not None:return solver.operating_point(speed/3.6,load,[alpha,*z[:4]],propulsion_override=frozen)
        if key not in memo:memo[key]=solver.operating_point(speed/3.6,load,[alpha,*z[:4]])
        return memo[key]
    class Closed(Exception):
        pass
    def fun(z):
        v=value(z)
        if v['force_error_g']<2e-5 and max(abs(v['rate_residual']))<5e-6:raise Closed(z)
        return v['residual']
    def jac(z):
        base=value(z);frozen=base['propulsion'] if solver.is_prop and base['propulsion']['converged'] else None
        if frozen is not None:base=value(z,frozen)
        cols=[]
        for i,h in enumerate([.002,.0002,.0002,.0002,.0002]):
            for factor in (1.,.5,-.5,2.,-1.,.1,-.1):
                q=z.copy();q[i]+=h*factor
                if i==4 and q[i]<0.:continue
                trial=value(q,frozen);used_step=h*factor
                if solver.derivative_branch(trial)==solver.derivative_branch(base):break
            cols.append((trial['residual']-base['residual'])/used_step)
        return np.column_stack(cols)
    try:
        fit=least_squares(fun,initial,jac=jac,bounds=bounds,max_nfev=12,
            x_scale=[30.,.2,.2,.2,max(1.,initial[4])],ftol=1e-9,xtol=2e-7,gtol=1e-8)
        z=fit.x
    except Closed as closed:z=closed.args[0]
    v=value(z)
    if v['force_error_g']>2e-4 or max(abs(v['rate_residual']))>5e-5:return None
    point=solver.solve(speed,math.hypot(1.,z[4]),[alpha,*z[:4]],exhaustive=False,quick=True)
    if not point['valid'] or not left['load_g']<point['load_g']<right['load_g']:return None
    if not left['alpha_deg']<point['alpha_deg']<right['alpha_deg']:return None
    point['sampling_coordinate']='angle continuation; original load/force/moment equations'
    return point


def alpha_point(solver,speed,left,right):
    # The location of an adaptive holdout is free. Nearby interior coordinates
    # can avoid native rounding seams while still checking the same cell.
    for fraction in (.5,.4375,.5625):
        point=_alpha_point(solver,speed,left,right,fraction)
        if point is not None:return point
    return None


def interior_point(solver,speed,left,right):
    """Choose a balanced interior check without exhaustively fixing its load.

    The sampler owns this coordinate. Native rounding seams can leave one
    arbitrarily chosen load without a root while adjacent coordinates close.
    Use nearby, independently certified samples before escalating to the
    angle coordinate; the interpolation error is checked at the actual point.
    """
    if not (left['valid'] and right['valid'] and left['load_g']<right['load_g']):return None
    if any(abs(p.get('sideslip_attitude_deg',0.))>1e-8 for p in (left,right)):
        return alpha_point(solver,speed,left,right)
    for fraction in (.5,.375,.625):
        load=left['load_g']*(1-fraction)+right['load_g']*fraction
        guess=[a*(1-fraction)+b*fraction for a,b in zip(left['solution'],right['solution'])]
        bank=lambda n:math.degrees(math.acos(1./n))
        guess[1]=bank(load)+(left['solution'][1]-bank(left['load_g']))*(1-fraction)+(right['solution'][1]-bank(right['load_g']))*fraction
        point=solver.solve(speed,load,guess,exhaustive=False,quick=True)
        if point['valid']:
            point['sampling_coordinate']='adaptive interior load; independently balanced and certified'
            return point
    return alpha_point(solver,speed,left,right)
