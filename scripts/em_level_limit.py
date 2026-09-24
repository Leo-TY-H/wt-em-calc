"""Condition the original active-limit solve near straight-and-level flight."""
import math
import numpy as np
from scipy.optimize import least_squares,brentq
from instructor_envelope import controller_limits as instructor_limits
from em_pitch_response import response as pitch_response


def level_constraint_point(point,kind=None):
    """Recognize an already certified 1-g root on an active physical limit.

    A level point at its actual control stop, or the ordinary stall inset,
    already solves the endpoint equations. Re-solving speed in a bracket only
    nanometres/second wide can lose that root to native rounding.
    """
    if not point or not point['valid'] or point['load_g']!=1.:return None
    axis=None;constraint=None;active=None
    if kind in (None,'control'):
        choices=[(abs(command-bound),i+2,command-bound) for i,(command,bounds) in
                 enumerate(zip(point['commands'],point['control_bounds'])) for bound in bounds]
        distance,axis,margin=min(choices)
        if distance<=2e-7:active='control';constraint=margin
    if active is None and kind in (None,'stall') and abs(point['stall_margin_deg']-.002)<=.0002:
        active='stall';constraint=point['stall_margin_deg']-.002;axis=None
    if active is None:return None
    result=dict(point)
    result['envelope_limit']=dict(kind=active,axis=axis,limiting_load_g=1.,constraint_residual=constraint,
        evaluations=0,method='balanced level-flight active constraint')
    return result


def level_stall_edge(solver,low_speed,column,low_point=None,kind=None):
    """Solve n=1 equilibrium and the stall constraint with speed unknown.

    Used only at an already bracketed low-speed stall edge. This removes
    dozens of entire speed/load grids from what is a single endpoint solve.
    A failed or differently constrained root falls back to normal sampling.
    """
    high=column['boundary']
    if high is None:return None
    if solver.is_prop and solver.engine.automatic:
        # The endpoint must close the same nonlinear phase average as the
        # published aircraft point. Solving against mean propwash and then
        # re-trimming against all phases moves alpha off the active limit,
        # discards a nearly solved endpoint and launches whole speed grids.
        # Keep frozen-mean Jacobians as inexpensive search directions only.
        import copy
        solver=solver.with_prop_controls(solver.engine.fixed_controls or solver.engine.automatic_controls)
        if high.get('_propulsion_seed') is not None:
            solver.engine._reference=copy.deepcopy(high['_propulsion_seed'])
        solver.engine.force_canonical=True
    kind=kind or high.get('envelope_limit',{}).get('kind')
    if kind not in ('stall','Instructor pitch','control','pitch response'):return None
    known=level_constraint_point(high,kind)
    if known is not None:return known
    control_axis=None;control_sign=None
    if kind=='control':
        choices=[(abs(p-bound),axis,side) for axis,(p,bounds) in enumerate(zip(high['commands'],high['control_bounds']))
                 for side,bound in enumerate(bounds)]
        _,control_axis,side=min(choices);control_sign=1. if side else -1.
    def control_margin(v):
        bounds=(solver.instructor_at(v,v['speed'],1.)['control_bounds'] if solver.config['instructor']
                else v['allocation']['bounds'])
        # The minimum-speed endpoint is the actual control stop. An inward
        # command inset can move its root beyond this already tight speed
        # bracket and make a valid endpoint appear unresolved.
        target=bounds[control_axis][1 if control_sign>0 else 0]
        return v['equilibrium_coordinates'][control_axis+2]-target
    upper_speed=column['speed_kmh'];memo={}
    if kind=='Instructor pitch':
        from em_accuracy import speed_tolerance
        speed_target=.25*speed_tolerance(solver.config)
        ps_target=.2*solver.config['sep_tolerance_mps']
        def small_bracket(a,b):
            return (b[0]-a[0]<=speed_target and
                    abs(levels[b[0]]['ps_mps']-levels[a[0]]['ps_mps'])<=ps_target)
        # At n=1 only speed remains unknown after balancing the aircraft.
        # Reuse the already observed sign bracket instead of a six-variable
        # search whose Jacobian repeatedly rebuilds speed-specific histories.
        levels={p['speed_kmh']:p for p in [low_point,*column['points']]
                if p and p['load_g']==1. and p['converged']}
        def pitch_at(speed):
            if speed not in levels:
                near=min(levels.values(),key=lambda p:abs(p['speed_kmh']-speed)) if levels else high
                levels[speed]=solver.solve(speed,1.,near['solution'],exhaustive=False)
            point=levels[speed];ctl=point.get('instructor')
            if (not point['converged'] or not ctl or not ctl['converged'] or
                    set(point['reasons'])-{'Instructor pitch limit'}):
                raise ValueError('No balanced Instructor level bracket')
            return ctl['envelope_margin']-2e-6
        # Establish the scalar bracket near the lift-scaled estimate. The
        # requested minimum speed is often far below any 1-g equilibrium;
        # attempting a controller root at that unbalanced endpoint used to
        # discard this cheap solve and trigger repeated full speed columns.
        # Failed aircraft solves are never assigned a constraint sign.
        estimate=min(upper_speed,max(low_speed,upper_speed/math.sqrt(high['load_g'])))
        candidate=estimate;blocked=low_speed;solved_speed=None;solved_bracket=None
        for attempt in range(10):
            try:
                error=pitch_at(candidate)
                if levels[candidate]['valid'] and abs(error)<2e-5:
                    solved_speed=candidate;break
            except ValueError:
                blocked=max(blocked,candidate)
            observed=[]
            for v,p in levels.items():
                ctl=p.get('instructor')
                if (p['converged'] and ctl and ctl['converged'] and
                        not set(p['reasons'])-{'Instructor pitch limit'}):
                    observed.append((v,ctl['envelope_margin']-2e-6))
            negative=[pair for pair in observed if pair[1]<0.]
            positive=[pair for pair in observed if pair[1]>=0.]
            brackets=[(a,b) for a in negative for b in positive if a[0]<b[0]]
            if brackets:
                a,b=min(brackets,key=lambda pair:pair[1][0]-pair[0][0])
                # Rounded controller states need not contain a continuous
                # zero. Retain a balanced rejected/accepted speed bracket,
                # just as for a load boundary, rather than asking a coupled
                # fallback to find a zero that may not exist.
                for iteration in range(24):
                    if small_bracket(a,b):
                        solved_speed=b[0];solved_bracket=[a[0],b[0]];break
                    fraction=-a[1]/(b[1]-a[1]) if iteration%3!=2 else .5
                    speed=a[0]+max(.1,min(.9,fraction))*(b[0]-a[0])
                    try:error=pitch_at(speed)
                    except ValueError:break
                    if error>=0.:
                        b=(speed,error)
                        if error<2e-5:solved_speed=speed;solved_bracket=[a[0],b[0]];break
                    else:a=(speed,error)
                break
            if not positive:break
            positive.sort();v0,f0=positive[0]
            candidate=v0-max(.5,.01*v0)
            if len(positive)>1:
                v1,f1=positive[1]
                if f1>f0+1e-8:candidate=v0-f0*(v1-v0)/(f1-f0)
            candidate=max(v0-max(.5,.05*v0),blocked+(v0-blocked)*.5,min(v0-.0001,candidate))
            if candidate<=blocked or v0-blocked<.001:break
        if solved_speed is not None:
            point=levels[solved_speed]
            if point['valid'] and (abs(pitch_at(solved_speed))<2e-5 or
                                  solved_bracket and small_bracket((solved_bracket[0],0.),(solved_bracket[1],0.))):
                point['surface_sample']=True
                point['envelope_limit']=dict(kind=kind,axis=None,limiting_load_g=1.,
                    constraint_residual=pitch_at(solved_speed),evaluations=len(levels),
                    limiting_speed_interval_kmh=solved_bracket,speed_target_kmh=speed_target,
                    ps_target_mps=ps_target,method='balanced level-flight speed bracket')
                return point
    def evaluate(z,frozen=None):
        key=tuple(z)
        if frozen is not None or key not in memo:
            # Coupled root iterations continue the native drivetrain between
            # nearby speeds. The accepted endpoint is independently certified
            # by solver.solve below; cold-restarting every Jacobian sample is
            # redundant and dominates low-speed edge time for multi-prop craft.
            budget=60. if solver.is_prop and solver.engine._warm is None else 20.
            v=solver.operating_point(z[5]/3.6,1.,z[:5],propulsion_override=frozen,cycle_seconds=budget)
            margin=((v['stall_margin']-.002)/10. if kind=='stall' else control_margin(v) if kind=='control'
                    else pitch_response(solver,v)['margin']-2e-6 if kind=='pitch response'
                    else instructor_limits(solver,v)['envelope_margin']-2e-6)
            pair=(v,np.append(v['residual'],margin))
            if frozen is not None:return pair
            memo[key]=pair
        return memo[key]
    def jac(z,coupled=False):
        base,res=evaluate(z)
        frozen=base['propulsion'] if solver.is_prop and solver.engine.automatic and not coupled else None
        if frozen is not None:base,res=evaluate(z,frozen)
        cols=[]
        for i,h in enumerate([.002,.002,.0002,.0002,.0002,.02]):
            for factor in [1.,.5,2.,-.5,-1.,.1,-.1,.01,-.01]:
                step=h*factor;q=z.copy();q[i]+=step;v,r=evaluate(q,frozen)
                if solver.derivative_branch(v)==solver.derivative_branch(base):break
            cols.append((r-res)/step)
        return np.column_stack(cols)
    lower=np.array([*solver.trim_bounds[0],low_speed]);upper=np.array([*solver.trim_bounds[1],upper_speed])
    z=np.array(high['solution']+[upper_speed/math.sqrt(high['load_g'])]);z[1]=0.
    z=np.clip(z,lower+1e-8,upper-1e-8);v,r=evaluate(z)
    use_coupled=False
    for iteration in range(12):
        if v['force_error_g']<1e-6 and max(abs(v['rate_residual']))<2e-6 and abs(r[-1])<2e-5:break
        delta=np.linalg.lstsq(jac(z,coupled=use_coupled or iteration>=8),-r,rcond=None)[0]
        delta/=max(1.,float(np.max(abs(delta)/[5.,10.,.4,.4,.4,30.])))
        accepted=False
        for factor in [1.,.5,.25,.1]:
            q=np.clip(z+factor*delta,lower,upper);vv,rr=evaluate(q)
            if np.linalg.norm(rr)<np.linalg.norm(r):z,v,r=q,vv,rr;accepted=True;break
        if not accepted:
            # A frozen engine derivative is an inexact search direction.
            # Failure of that direction is not failure of the endpoint root.
            if not use_coupled:use_coupled=True;continue
            break
    if v['force_error_g']>2e-4 or max(abs(v['rate_residual']))>5e-5 or abs(r[-1])>2e-5:return None
    point=solver.solve(float(z[5]),1.,z[:5].tolist(),exhaustive=False)
    constraint=(point['stall_margin_deg']-.002 if kind=='stall' else
                point['solution'][control_axis+2]-point['control_bounds'][control_axis][1 if control_sign>0 else 0]
                if kind=='control' else (point.get('pitch_response') or {}).get('margin',1.)-2e-6
                if kind=='pitch response' else (point.get('instructor') or {}).get('envelope_margin',1.)-2e-6)
    if not point['valid'] or abs(constraint)>(.0003 if kind=='stall' else 2e-5):return None
    point['surface_sample']=True
    point['envelope_limit']=dict(kind=kind,axis=control_axis+2 if kind=='control' else None,limiting_load_g=1.,
        constraint_residual=constraint,evaluations=len(memo),method='level-flight speed coordinate')
    return point


def near_level_boundary(solver,speed_kmh,low,high,candidate_kinds=None,max_nfev=60):
    """Use sqrt(n²-1), rather than n, as the final unknown near n=1.

    The physical model still receives the original n. This removes the
    singular derivative of turn rate versus load at the floor of the diagram.
    """
    if low['load_g']>1.05:return None
    kinds=(['Instructor pitch'] if solver.config['instructor'] else [])+['stall','control']
    if ('reversed pitch response' in high['reasons'] or high.get('continuation_limit_kind')=='pitch response' or
            candidate_kinds and 'pitch response' in candidate_kinds):kinds.insert(0,'pitch response')
    if candidate_kinds is not None:kinds=[kind for kind in kinds if kind in candidate_kinds]
    answers=[]
    for kind in kinds:
        memo={}
        target=1. if high['solution'][3]>=0 else -1.
        def value(z):
            key=tuple(z)
            if key not in memo:
                load=math.hypot(1.,z[5])
                v=solver.operating_point(speed_kmh/3.6,load,z[:5])
                if kind=='stall':margin=(v['stall_margin']-.002)/10.
                elif kind=='Instructor pitch':margin=instructor_limits(solver,v)['envelope_margin']-2e-6
                elif kind=='pitch response':margin=pitch_response(solver,v)['margin']-2e-6
                else:
                    bounds=solver.instructor_at(v,speed_kmh/3.6,load)['control_bounds'] if solver.config['instructor'] else v['allocation']['bounds']
                    margin=z[3]-(bounds[1][1 if target>0 else 0]-target*3e-5)
                memo[key]=(v,np.append(v['residual'],margin))
            return memo[key]
        class Closed(Exception):pass
        def fun(z):
            v,r=value(z)
            if (v['force_error_g']<=2e-4 and max(abs(v['rate_residual']))<=5e-5 and
                    v['history_error']<=2e-4 and abs(r[5])<=(2e-5 if kind=='stall' else 5e-6)):
                raise Closed(np.array(z))
            return r
        def jac(z):
            base,res=value(z);cols=[]
            frozen=(base['propulsion'] if solver.is_prop and solver.engine.automatic and
                    base['propulsion']['converged'] else None)
            if frozen is not None:
                # An inexact derivative is only a search direction. Residuals
                # and the independently validated final point remain coupled.
                load=math.hypot(1.,z[5])
                base=solver.operating_point(speed_kmh/3.6,load,z[:5],propulsion_override=frozen)
                def proxy(q):
                    n=math.hypot(1.,q[5])
                    v=solver.operating_point(speed_kmh/3.6,n,q[:5],propulsion_override=frozen)
                    if kind=='stall':margin=(v['stall_margin']-.002)/10.
                    elif kind=='Instructor pitch':margin=instructor_limits(solver,v)['envelope_margin']-2e-6
                    elif kind=='pitch response':margin=pitch_response(solver,v)['margin']-2e-6
                    else:margin=q[3]-(v['allocation']['bounds'][1][1 if target>0 else 0]-target*3e-5)
                    return v,np.append(v['residual'],margin)
                _,res=proxy(z)
            for i,h in enumerate([.002,.002,.0002,.0002,.0002,.0002]):
                for factor in [1.,.5,2.,-.5,-1.,.1,-.1,.01,-.01]:
                    trial=z.copy();trial[i]+=h*factor
                    if trial[5]<0:continue
                    v,r=proxy(trial) if frozen is not None else value(trial);step=h*factor
                    if solver.derivative_branch(v)==solver.derivative_branch(base):break
                cols.append((r-res)/step)
            return np.column_stack(cols)
        seed=np.array(low['solution']+[math.sqrt(max(0.,low['load_g']**2-1.))])
        # Starting essentially at u=0 also makes dn/du vanish numerically.
        # A modest turn seed lets the coupled load correction be identified;
        # this changes only initialization, never the accepted load/limit.
        seed[5]=max(.15,seed[5])
        if kind=='stall':seed[0]=low['alpha_deg']+low['stall_margin_deg']-.002
        elif kind=='Instructor pitch':seed[0]=low['alpha_deg']+max(0.,low.get('instructor',{}).get('envelope_margin',0.))
        nearby=high.get('continuation_seed')
        if nearby is not None:
            seed[:5]=nearby['solution']
            guess=min(high['load_g']-.001,max(1.001,nearby['load_g']*(speed_kmh/nearby['speed_kmh'])**2))
            seed[5]=math.sqrt(guess*guess-1.)
        lower=[*solver.trim_bounds[0],0.];upper=[*solver.trim_bounds[1],math.sqrt(max(1.1,high['load_g'])**2-1.)]
        from types import SimpleNamespace
        try:
            fit=least_squares(fun,np.clip(seed,np.array(lower)+1e-9,np.array(upper)-1e-9),jac=jac,
                              bounds=(lower,upper),x_scale=[10.,10.,.2,.2,.2,.1],max_nfev=max_nfev,
                              ftol=1e-10,xtol=1e-9,gtol=1e-10)
        except Closed as closed:fit=SimpleNamespace(x=closed.args[0])
        v,r=value(fit.x)
        if v['force_error_g']>2e-4 or max(abs(v['rate_residual']))>5e-5 or abs(r[5])>(2e-5 if kind=='stall' else 5e-6):continue
        point=solver.solve(speed_kmh,math.hypot(1.,fit.x[5]),fit.x[:5].tolist(),exhaustive=False)
        if not point['valid'] or point['load_g']<low['load_g']-1e-7:continue
        if kind=='pitch response' and abs(point['pitch_response']['margin']-2e-6)>5e-6:continue
        point['envelope_limit']=dict(kind=kind,axis=3 if kind=='control' else None,
                                    limiting_load_g=point['load_g'],constraint_residual=float(r[5]),
                                    evaluations=len(memo),method='near-level turn-rate coordinate')
        answers.append(point)
        if ((kind=='Instructor pitch' and point['stall_margin_deg']>.004 or kind=='stall' and not solver.config['instructor']) and point['authority_margin']>6e-5
                and (not solver.config['structural_limits'] or max(point['wing_load_ratios'])<.99990)
                and low['load_g']<=point['load_g']<=high['load_g']):return point
    return min(answers,key=lambda p:p['load_g']) if answers else None


def near_level_held_pitch(held,speed_kmh,initial):
    """The same well-conditioned turn coordinate for the coupled controller."""
    solver=held.aircraft;memo={}
    def unpack(z):return np.r_[z[:5],math.hypot(1.,z[5])]
    def value(z):
        key=tuple(z)
        if key not in memo:
            x=unpack(z);v=solver.operating_point(speed_kmh/3.6,x[5],x[:5])
            c=held.controller(v,speed_kmh/3.6)
            memo[key]=(v,c,np.r_[v['residual'],c['margin']*10.])
        return memo[key]
    def jac(z):
        base,_,res=value(z);cols=[]
        for i,h in enumerate([.003,.003,.0003,.0003,.0003,.0002]):
            for factor in [1.,.5,2.,-.5,-1.,.1,-.1,.01,-.01]:
                trial=z.copy();trial[i]+=h*factor
                if trial[5]<0:continue
                v,_,r=value(trial);step=h*factor
                if solver.derivative_branch(v)==solver.derivative_branch(base):break
            cols.append((r-res)/step)
        return np.column_stack(cols)
    z=np.r_[initial[:5],math.sqrt(max(0.,initial[5]**2-1.))]
    lower=np.array([*solver.trim_bounds[0],0.]);upper=np.array([*solver.trim_bounds[1],1.])
    fit=least_squares(lambda z:value(z)[2],np.clip(z,lower+1e-8,upper-1e-8),jac=jac,bounds=(lower,upper),
                      x_scale=[10.,20.,.2,.2,.2,.1],max_nfev=70,ftol=1e-10,xtol=1e-9,gtol=1e-9)
    v,c,res=value(fit.x)
    if v['force_error_g']<=2e-4 and max(abs(v['rate_residual']))<=5e-5 and abs(c['margin'])<2e-5 and c['converged']:
        return unpack(fit.x)
    return None
