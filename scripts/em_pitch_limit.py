"""Follow balanced pitch-command states to the first response reversal."""
import copy,math
import numpy as np
from scipy.optimize import least_squares
from em_accuracy import turn_tolerance


def limit(solver,speed,seed):
    if not seed['valid'] or not seed.get('pitch_response'):return None
    view=copy.copy(solver);view._pitch_response_recovery=True
    center=seed['solution'][3];memo={center:seed}
    def at(command,near):
        if command in memo:return memo[command]
        x=near['solution'];n=near['load_g']
        initial=np.array([x[0],x[1],x[2],x[4],math.sqrt(max(0.,n*n-1.))]);cache={}
        def value(z):
            key=tuple(z)
            if key not in cache:cache[key]=view.operating_point(speed/3.6,math.hypot(1.,z[4]),[z[0],z[1],z[2],command,z[3]])
            return cache[key]
        class Closed(Exception):pass
        def fun(z):
            v=value(z)
            if v['force_error_g']<1e-6 and max(abs(v['rate_residual']))<2e-6:raise Closed(z.copy())
            return v['residual']
        def jac(z):
            base=value(z);columns=[]
            for i,h in enumerate((.002,.002,.0002,.0002,.0002)):
                for factor in (1.,.5,-.5,2.,-1.,.1,-.1):
                    q=z.copy();q[i]+=h*factor
                    if i==4 and q[i]<0.:continue
                    trial=value(q);step=h*factor
                    if view.derivative_branch(trial)==view.derivative_branch(base):break
                columns.append((trial['residual']-base['residual'])/step)
            return np.column_stack(columns)
        bounds=tuple([b[0],b[1],b[2],b[4],u] for b,u in zip(view.trim_bounds,(0.,math.sqrt(64.**2-1.))))
        try:
            fit=least_squares(fun,initial,jac=jac,bounds=bounds,max_nfev=18,
                x_scale=[10.,30.,.2,.2,1.],ftol=1e-10,xtol=1e-9,gtol=1e-10)
            z=fit.x
        except Closed as closed:z=closed.args[0]
        v=value(z)
        if v['force_error_g']>1e-6 or max(abs(v['rate_residual']))>2e-6:return None
        p=view.solve(speed,math.hypot(1.,z[4]),[z[0],z[1],z[2],command,z[3]],exhaustive=False,quick=True)
        if (not p['converged'] or abs(p['solution'][3]-command)>1e-5 or
                not p.get('pitch_response') or set(p['reasons'])-{'reversed pitch response'}):return None
        if abs(p['alpha_deg']-near['alpha_deg'])>.75:return None
        memo[command]=p
        return p
    # Pick a direction using independently balanced local control responses,
    # then keep that regular coordinate through the loss of pitch authority.
    probes=[]
    for sign in (-1.,1.):
        command=max(-1.,min(1.,center+sign*.01))
        if command==center:continue
        p=at(command,seed)
        if (p and p['load_g']>=seed['load_g'] and
                p['pitch_response']['margin']<seed['pitch_response']['margin']):probes.append(p)
    if not probes:return None
    probe=min(probes,key=lambda p:p['pitch_response']['margin'])
    direction=1. if probe['solution'][3]>center else -1.
    good=seed;bad=probe if not probe['valid'] else None
    if probe['valid']:good=probe
    step=.015
    for _ in range(24):
        if bad is not None:break
        command=max(-1.,min(1.,good['solution'][3]+direction*step))
        if command==good['solution'][3]:return None
        p=at(command,good)
        if p is None:
            step*=.5
            if step<1e-5:return None
            continue
        if not p['valid']:bad=p;break
        if p['load_g']<good['load_g']-2e-4:return None
        good=p;step=min(.04,step*1.5)
    if bad is None:return None
    def closed():
        return (abs(good['turn_dps']-bad['turn_dps'])<=.25*turn_tolerance(solver.config['sep_tolerance_mps']) and
                abs(good['ps_mps']-bad['ps_mps'])<=.2*solver.config['sep_tolerance_mps'])
    for _ in range(24):
        if closed():break
        p=at((good['solution'][3]+bad['solution'][3])*.5,good)
        if p is None:return None
        if p['valid']:good=p
        else:bad=p
    if not closed() or good['load_g']<seed['load_g']-2e-4:return None
    good=dict(good)
    good['envelope_limit']=dict(kind='pitch response',axis=None,limiting_load_g=good['load_g'],
        method='First balanced pitch-response crossing in continuous command coordinates',
        constraint_residual=good['pitch_response']['margin'],rejected_constraint_residual=bad['pitch_response']['margin'],
        command_bracket=[good['solution'][3],bad['solution'][3]],
        limiting_load_interval_g=sorted([good['load_g'],bad['load_g']]),
        turn_interval_dps=sorted([good['turn_dps'],bad['turn_dps']]),
        ps_interval_mps=[good['ps_mps'],bad['ps_mps']],evaluations=len(memo),
        rejected_endpoint={k:bad[k] for k in ('speed_kmh','load_g','solution','reasons','force_error_g','angular_error_rad_s2')})
    return good
