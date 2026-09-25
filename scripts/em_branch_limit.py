"""Certify discontinuous physical limits on both sides of a native branch."""
import math
import numpy as np
from scipy.optimize import least_squares
from em_pitch_response import PHYSICAL_REASONS, KINDS

# Stay on the incoming float32 branch without omitting a resolvable band of
# rejected angles immediately below the native +/-12 degree switch.
ROLL_LEVELING_PROBE_OFFSET=1e-5


def fixed_alpha(solver,speed,alpha,seed,canonical=False):
    if canonical and solver.is_prop and solver.engine.automatic:
        solver=solver.with_prop_controls(solver.engine.fixed_controls or solver.engine.automatic_controls)
        solver.engine.force_canonical=True
    guess=np.array(seed['solution']);guess[0]=alpha
    n=max(1.000001,float(seed['load_g']));turn=math.sqrt((n-1.)*(n+1.));z=np.r_[guess[1:],turn]
    z[0]=math.degrees(math.atan(turn))
    bounds=tuple([*b[1:],u] for b,u in zip(solver.trim_bounds,(0.,math.sqrt(64.**2-1.))))
    z=np.clip(z,np.asarray(bounds[0])+1e-9,np.asarray(bounds[1])-1e-9)
    memo={}
    def value(x,frozen=None):
        key=tuple(x)
        load=math.hypot(1.,x[4])
        if frozen is not None:return solver.operating_point(speed/3.6,load,[alpha,*x[:4]],propulsion_override=frozen)
        if key not in memo:memo[key]=solver.operating_point(speed/3.6,load,[alpha,*x[:4]])
        return memo[key]
    class Closed(Exception):pass
    def fun(x):
        v=value(x)
        if v['force_error_g']<2e-5 and max(abs(v['rate_residual']))<5e-6:raise Closed(x)
        return v['residual']
    def jac(x):
        base=value(x);frozen=base['propulsion'] if solver.is_prop and base['propulsion']['converged'] else None
        if frozen is not None:base=value(x,frozen)
        cols=[]
        for i,h in enumerate([.002,.0002,.0002,.0002,.0002]):
            for factor in (1.,.5,-.5,2.,-1.,.1,-.1):
                q=x.copy();q[i]+=h*factor
                if i==4 and q[i]<0.:continue
                trial=value(q,frozen);used_step=h*factor
                if solver.derivative_branch(trial)==solver.derivative_branch(base):break
            cols.append((trial['residual']-base['residual'])/used_step)
        return np.column_stack(cols)
    try:
        fit=least_squares(fun,z,jac=jac,bounds=bounds,max_nfev=12,x_scale=[30.,.2,.2,.2,max(1.,n)],ftol=1e-9,xtol=2e-7,gtol=1e-8)
        z=fit.x
    except Closed as closed:z=closed.args[0]
    v=value(z)
    if v['force_error_g']>2e-4 or max(abs(v['rate_residual']))>5e-5:return None
    p=solver.solve(speed,math.hypot(1.,z[4]),[alpha,*z[:4]],exhaustive=False,quick=True)
    if not p['converged'] or abs(p['alpha_deg']-alpha)>.001:return None
    return p


def first_roll_leveling_limit(solver,speed,level,candidate):
    """Find a physical rejection hidden before a crossed roll-helper switch.

    An active-constraint root beyond the switch is locally valid, but does not
    prove that the level-flight component reaches it. Test the incoming side
    with a separately balanced state before accepting that root. This applies
    to every aircraft with the native helper, regardless of its wing layout.
    """
    if (not solver.fm.get('RollLeveling',True) or not level or not level['valid']
            or not candidate or not candidate.get('envelope_limit')
            or candidate['load_g']<=level['load_g']):return None
    # These fixed-angle probes use the column's ordinary zero-sideslip trim.
    # A recovered sideslip root needs its own continuation certificate.
    if (level.get('sideslip_attitude_deg',0.)!=0. or
            candidate.get('sideslip_attitude_deg',0.)!=0.):return None
    start,end=level['alpha_deg'],candidate['alpha_deg']
    if start<12.<=end:incoming=12.-ROLL_LEVELING_PROBE_OFFSET
    elif end<=-12.<start:incoming=-12.+ROLL_LEVELING_PROBE_OFFSET
    else:return None
    if not min(start,end)<incoming<max(start,end):return None

    # A post-switch boundary is a good numerical seed, but only a fresh
    # pre-switch equilibrium can establish the sign of the physical margins.
    probe=(fixed_alpha(solver,speed,incoming,candidate) or
           fixed_alpha(solver,speed,incoming,level) or
           fixed_alpha(solver,speed,incoming,candidate,canonical=True))
    if (probe is None or not probe['converged'] or
            probe.get('roll_leveling_branch')!=0 or
            not level['load_g']<probe['load_g']<candidate['load_g']):return None
    if probe['valid'] or not probe['reasons'] or not set(probe['reasons'])<=PHYSICAL_REASONS:return None

    from em_constraint_bracket import refine
    first=refine(solver,speed,level,probe)
    if first is None or first['load_g']>=candidate['load_g']:return None
    first['envelope_limit']['selection']='First balanced rejection before native roll-leveling switch'
    first['envelope_limit']['roll_leveling_switch_deg']=12. if incoming>0. else -12.
    return first


def turning_limit(solver,speed,low,high):
    """Bracket a physical limit in angle when load folds near maximum lift.

    A load-coordinate solve becomes singular as d(load)/d(alpha) approaches
    zero. Keep load free and follow balanced angle steps instead. Only a
    balanced physical rejection can close the bracket; failure stays unknown.
    """
    if not low['valid'] or not 0.<low['stall_margin_deg']<2.:return None
    allowed=PHYSICAL_REASONS
    from em_constraint_bracket import refine
    point=low
    for attempt in range(6):
        step=min(.5,max(.025,.65*point['stall_margin_deg']))
        alpha=point['alpha_deg']+step
        if alpha>=solver.trim_bounds[1][0]:return None
        trial=fixed_alpha(solver,speed,alpha,point)
        if trial is None:return None
        if trial['load_g']<=point['load_g']:return None
        if trial['valid']:
            point=trial
        elif trial['reasons'] and set(trial['reasons'])<=allowed:
            result=refine(solver,speed,point,trial)
            if result:
                result['envelope_limit']['seed_method']='balanced angle bracket near lift fold'
                result['envelope_limit']['continuation_steps']=attempt+1
            return result
        else:return None
    return None


def branch_limit(solver,speed,low,high):
    """Advance verified interior to a switch; certify only a real rejection.

    Failure beyond the switch is still numerical uncertainty. A last valid
    state carries no envelope_limit unless the opposite state also balances.
    """
    if not solver.fm.get('RollLeveling',True) or not low['valid']:return None
    allowed=PHYSICAL_REASONS
    def at(alpha,seed):
        return fixed_alpha(solver,speed,alpha,seed) or fixed_alpha(solver,speed,alpha,seed,True)
    for alpha in (-12.,12.):
        if not low['alpha_deg']<alpha<max(low['alpha_deg']+low['stall_margin_deg'],high['alpha_deg'])+.25:continue
        if alpha-low['alpha_deg']>4.:continue
        if abs(low['alpha_deg']+low['stall_margin_deg']-alpha)>.75:continue
        target=alpha-.002;left=low
        for attempt in range(5):
            point=at(target,left)
            if point and point['valid']:left=point;break
            closer=at((left['alpha_deg']+target)*.5,left)
            if not closer or not closer['valid']:break
            left=closer
        if abs(left['alpha_deg']-target)>.001 or left['load_g']<low['load_g']-2e-4:continue
        if max(abs(np.array(left['solution'])[2:]-np.array(low['solution'])[2:]))>.2:continue
        if left['stall_margin_deg']>.6:continue
        right=at(alpha+.002,left)
        if right is None:right=at(alpha+.002,low)
        if right and right['valid']:continue
        left['native_branch_search']=dict(alpha_switch_deg=alpha,solved_alpha_deg=left['alpha_deg'],
            upper_limit_verified=False,method='fixed-angle continuation to native branch; full independent force/moment/propulsion checks')
        left['_checked_probe_boundary']=True
        if right and not right['valid'] and right['reasons'] and set(right['reasons'])<=allowed and left.get('roll_leveling_branch')!=right.get('roll_leveling_branch'):
            kind=next(KINDS[reason] for reason in right['reasons'] if reason in KINDS)
            left['envelope_limit']=dict(kind=kind,axis=None,limiting_load_g=left['load_g'],
                method='balanced physical rejection across native roll-leveling branch',
                angle_bracket_deg=[left['alpha_deg'],right['alpha_deg']],
                opposite={k:right[k] for k in ('speed_kmh','load_g','alpha_deg','solution','converged','valid','reasons','force_error_g','angular_error_rad_s2','stall_margin_deg','wing_load_ratios','authority_margin')})
            left['native_branch_search']['upper_limit_verified']=True
        return left
    return None


def continue_endpoint(solver,speed,neighbors,level):
    """Continue a measured native branch edge across speed before root search."""
    known=sorted((p for p in neighbors or [] if p.get('native_branch_search')),key=lambda p:abs(p['speed_kmh']-speed))
    if not known or not level['valid']:return None
    seed=dict(known[0]);alpha=seed['native_branch_search']['alpha_switch_deg']
    if len(known)>1 and known[1]['native_branch_search']['alpha_switch_deg']==alpha:
        a,b=sorted(known[:2],key=lambda p:p['speed_kmh'])
        if a['speed_kmh']<speed<b['speed_kmh']:
            t=(speed-a['speed_kmh'])/(b['speed_kmh']-a['speed_kmh'])
            seed['solution']=(np.array(a['solution'])*(1-t)+np.array(b['solution'])*t).tolist()
            seed['load_g']=a['load_g']*(1-t)+b['load_g']*t
    left=fixed_alpha(solver,speed,alpha-.002,seed) or fixed_alpha(solver,speed,alpha-.002,seed,True)
    if not left or not left['valid'] or left['stall_margin_deg']>.6:return None
    opposite=next((p['envelope_limit']['opposite'] for p in known if p.get('envelope_limit',{}).get('opposite')),None)
    right_seed=opposite or left
    right=fixed_alpha(solver,speed,alpha+.002,right_seed) or fixed_alpha(solver,speed,alpha+.002,right_seed,True)
    if right and right['valid']:return None
    left['native_branch_search']=dict(alpha_switch_deg=alpha,solved_alpha_deg=left['alpha_deg'],
        upper_limit_verified=False,method='fixed-angle continuation of measured native branch edge across speed')
    left['_checked_probe_boundary']=True
    allowed=PHYSICAL_REASONS
    if right and right['reasons'] and set(right['reasons'])<=allowed and left.get('roll_leveling_branch')!=right.get('roll_leveling_branch'):
        kind=next(KINDS[reason] for reason in right['reasons'] if reason in KINDS)
        left['envelope_limit']=dict(kind=kind,axis=None,limiting_load_g=left['load_g'],
            method='balanced physical rejection across native roll-leveling branch',
            angle_bracket_deg=[left['alpha_deg'],right['alpha_deg']],
            opposite={k:right[k] for k in ('speed_kmh','load_g','alpha_deg','solution','converged','valid','reasons','force_error_g','angular_error_rad_s2','stall_margin_deg','wing_load_ratios','authority_margin')})
        left['native_branch_search']['upper_limit_verified']=True
    return left
