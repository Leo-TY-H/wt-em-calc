"""Recovered Instructor angle-target stage (not the entire controller).

Original 101a936c3..101a939a4, including the noncontiguous overload branches.
Inputs are prepared native state. In the game these targets feed the pitch
predictor. A static permission envelope can constrain the corresponding wing
angles under an explicit protection-history assumption; it does not reproduce
the complete controller's tracking response or equate this target to body AoA.
"""
import math
from component_assembly import f32,add,sub,mul
from body_dynamics import G


def divide(a,b):
    return f32(a/b) if abs(b)>f32(4e-19) else 0.


def gain_reference_speed(takeoff_mass,clean_runtime,rho0=f32(1.225)):
    """FM7f8c from 101a3d67e and 101a3d961..9c1, normal aircraft branch.

    Uses configured Mass.Takeoff and the zero-sweep, clean prepared polar.
    It is not the actual fuel-dependent operating-point stall speed.
    The buoyancy-dominated loader early exit is outside this helper.
    """
    b=clean_runtime['base']
    weight=mul(mul(f32(takeoff_mass),f32(1.1)),G)
    denominator=mul(mul(mul(b[11],.5),b[15]),f32(rho0))
    return f32(math.sqrt(divide(weight,denominator)))


def inverse_cl(p,target):
    """10198c7f0: pre-stall inverse; invalid targets return the critical angle."""
    c=f32(f32(target)/p['clKq'])
    if c>p['cyCritH']:return p['aoaCritH'],False
    if c<p['cyCritL']:return p['aoaCritL'],False
    linear=divide(sub(c,p['cl0']),mul(p['cyMult'],p['clLineCoeff']))
    if p['aoaLineL']<=linear<=p['aoaLineH']:return linear,True
    positive=add(linear,f32(.01))>=p['aoaLineH'];suffix='H' if positive else 'L'
    square=divide(abs(sub(p['cyCrit'+suffix],c)),p['parabCyCoeff'+suffix])
    root=f32(math.sqrt(square)) if square>0 else 0.
    return sub(p['aoaCrit'+suffix],root if positive else -root),True


def remap(x,lo,hi,a,b):
    """Native clamped two-point interpolation, including reversed knots."""
    if lo>hi:lo,hi,a,b=hi,lo,b,a
    if x<=lo:return a
    if x>=hi:return b
    return add(a,divide(mul(sub(x,lo),sub(b,a)),sub(hi,lo)))


def recovery_filter(commands,history,command_bounds,*,peak,reference,critical_high,
                    current_wing_peak,stored_yaw_rate,dt,max_cvt_angle=1.5,smoothness=.29,
                    current_commands=None):
    """Active branch 101a94df1..101a9517e, after protective pitch clamp.

    Call only if global recovery is enabled, peak > reference, and input+9 is
    clear. stored_yaw_rate is FM+15e8, not longitudinal roll (FM+15e0).
    Commands/history and bounds use roll,pitch,yaw order. The caller
    selects current FM commands or the preceding MouseAim-dispatch output
    cache by input+0x29; the cache is not the current pilot input.
    current_commands is the destination FM vector, which can differ from the
    selected source cache. At 101a95172 roll/yaw writes are skipped when the
    axis mix is zero; their filter histories still advance.
    This is a controller stage, never a static EM limit.
    """
    commands=list(map(f32,commands));history=list(map(f32,history))
    current_commands=commands if current_commands is None else list(map(f32,current_commands))
    edge=mul(f32(max_cvt_angle),f32(critical_high))
    pitch_mix=remap(f32(peak),f32(reference),edge,0.,1.)
    axis_mix=remap(f32(current_wing_peak),f32(critical_high),edge,0.,1.)
    roll,pitch,yaw=commands
    target=[sub(roll,mul(axis_mix,roll)),
            add(pitch,mul(sub(-1.,pitch),axis_mix)),
            add(yaw,mul(axis_mix,sub(float((stored_yaw_rate>0)-(stored_yaw_rate<0)),yaw)))]
    smoothness=f32(smoothness)
    if smoothness>=f32(1e-9):
        weight=sub(1.,f32(math.exp(f32(-f32(dt)/smoothness))))
        filtered=[add(old,mul(sub(new,old),weight)) for old,new in zip(history,target)]
    else:filtered=target
    bounds=[]
    for i,(lo,hi) in enumerate(command_bounds):
        mix=pitch_mix if i==1 else axis_mix
        bounds.append([max(-1.,add(f32(lo),mul(sub(filtered[i],f32(lo)),mix))),
                       min(1.,add(f32(hi),mul(sub(filtered[i],f32(hi)),mix)))])
    result=[filtered[0] if axis_mix>0 else current_commands[0],filtered[1],
            filtered[2] if axis_mix>0 else current_commands[2]]
    return dict(commands=result,history=filtered,command_bounds=bounds,
                pitch_mix=pitch_mix,axis_mix=axis_mix)


def angle_targets(p,props,predicted_angles,*,rho,speed_squared,area,dihedral,
                  strength,mass,tail_area_pair,timer,dt,mode_lane=False,
                  overload_enabled=True):
    critical=list(map(f32,props.get('critMult',[-1.,-1.])))
    # The native fallback checks the first lane only.
    if critical[0]<0:critical=list(map(f32,[.8,.9]))
    factor=critical[0 if mode_lane else 1]
    low,high=mul(p['aoaCritL'],factor),mul(p['aoaCritH'],factor)
    base=[low,high];timer=f32(timer);dt=f32(dt)
    cos_dihedral=f32(math.cos(mul(f32(dihedral),f32(.01745329238474369))))
    q=mul(mul(.5,f32(rho)),f32(speed_squared))
    inverse=None;trigger=False
    if overload_enabled and props.get('limitOverload',True):
        scale=f32(2./max(1.,mul(mul(q,f32(area)),cos_dihedral)))
        low_inverse,low_valid=inverse_cl(p,mul(f32(strength[0]),scale))
        high_inverse,high_valid=inverse_cl(p,mul(f32(strength[1]),scale))
        factors=list(map(f32,props.get('overloadMult',[.85,.92,.85,.92])))
        onset,reserve=factors[:2] if mode_lane else factors[2:]
        trigger=(low_valid and low_inverse>low and min(predicted_angles)<mul(low_inverse,onset)) or (
                 high_valid and high_inverse<high and max(predicted_angles)>mul(high_inverse,onset))
        timer=min(1.,add(timer,mul(f32(props.get('overloadTimeRate',.45)),dt))) if trigger else max(0.,sub(timer,dt))
        lo,hi=map(f32,props.get('overloadTimeRange',[.9,1.]))
        # Increasing accumulated overload releases the reserved-angle restriction
        # towards the critical-angle targets, not the other way around.
        low=remap(timer,lo,hi,max(low,mul(low_inverse,reserve)),low)
        high=remap(timer,lo,hi,min(high,mul(high_inverse,reserve)),high)
        inverse=[low_inverse,high_inverse]
    else:timer=0.
    if props.get('limitLoadfactor',False):
        effective_area=sub(mul(cos_dihedral,f32(area)),mul(add(*map(f32,tail_area_pair)),.5))
        scale=f32(f32(mass)/max(1.,mul(q,effective_area)))
        for i,g in enumerate(props.get('loadFactorLimit',[-5.,12.])):
            value,valid=inverse_cl(p,mul(mul(f32(g),G),scale))
            if valid:
                if i:high=min(high,value)
                else:low=max(low,value)
    return dict(angle_limits=[low,high],overload_timer=timer,overload_trigger=bool(trigger),
                inverse_strength_angles=inverse,base_angle_limits=base)


def predicted_wing_angles(angles,previous,roll,pitch,roll_sensitivity,elevon_factor,
                          ail_prediction=14.,elevon_prediction=-18.,prediction=5.):
    """101a932ba..101a9336f, after the Mach-dependent roll sensitivity lookup."""
    a=mul(mul(f32(roll),f32(roll_sensitivity)),f32(ail_prediction))
    e=mul(mul(f32(elevon_factor),f32(pitch)),f32(elevon_prediction))
    adjusted=[sub(f32(angles[0]),add(e,a)),add(f32(angles[1]),sub(a,e))]
    predicted=[add(mul(sub(x,f32(old)),f32(prediction)),x) for x,old in zip(adjusted,previous)]
    return dict(adjusted=adjusted,predicted=predicted)


def recovery_reference(critical_high, raw_wing_angles, critical_multipliers,
                       *, mode_lane=False, mode_scalar=1., force_advanced=False,
                       critical_blend=.2):
    """101a92f36..92f91 and 101a933b8..93426, BEFORE overload targets.

    forceAdvanced and scalar < .1 use a unit multiplier for recovery only.
    They do not replace the independently selected critical-angle target
    multiplier. raw_wing_angles are neither adjusted nor extrapolated.
    """
    lanes=list(map(f32,critical_multipliers))
    if lanes[0]<0:lanes=list(map(f32,[.8,.9]))
    factor=1. if force_advanced or f32(mode_scalar)<f32(.1) else lanes[0 if mode_lane else 1]
    limit=mul(f32(critical_high),factor)
    return add(limit,mul(sub(min(max(map(f32,raw_wing_angles)),limit),limit),f32(critical_blend)))


def angle_rate(velocity,acceleration,quaternion,pitch_rate):
    """101a939a4..101a93b3d: acceleration normal to the flight direction.

    World velocity/acceleration and body pitch rate are native doubles; the
    projection uses float32, then adds the double body rate before PD rounding.
    """
    a,b,c=map(f32,velocity)
    recip=f32(1./max(1.,f32(math.sqrt(add(mul(c,c),add(mul(b,b),mul(a,a)))))))
    a,b,c=[mul(x,recip) for x in [a,b,c]]
    x,y,z,w=map(f32,quaternion)
    rx=sub(mul(y,x),mul(z,w));rx=add(rx,rx)
    ry=add(mul(y,y),mul(w,w));ry=add(add(ry,ry),-1.)
    rz=add(mul(w,x),mul(z,y));rz=add(rz,rz)
    p=sub(mul(b,rz),mul(c,ry));q=sub(mul(c,rx),mul(rz,a));r=sub(mul(ry,a),mul(rx,b))
    normal=[sub(mul(q,c),mul(r,b)),sub(mul(r,a),mul(c,p)),sub(mul(p,b),mul(q,a))]
    ax,ay,az=map(f32,acceleration)
    projected=add(mul(az,normal[2]),add(mul(ay,normal[1]),mul(ax,normal[0])))
    return float(mul(projected,recip))+float(pitch_rate)


def pitch_demands(p,props,predicted,limits,*,reference_speed,tas,ias,rate,
                  proportional_base=1.,derivative_base=12.):
    """Constant-gain path -> the two ORIGINAL reduced pitch predictor calls.

    These outputs are desired pitch accelerations and clamped working angles,
    not elevator commands. The reduced predictor remains a separate algorithm.
    """
    kp=f32(mul(f32(proportional_base),f32(reference_speed))/max(5.,f32(tas)))
    kd=f32(f32(reference_speed)/max(5.,f32(ias)));kd=mul(mul(kd,kd),f32(derivative_base))
    lower,upper=min(predicted),max(predicted);low,high=limits
    normalized=divide(max(sub(upper,p['aoaLineH']),sub(p['aoaLineL'],lower)),mul(sub(p['aoaLineH'],p['aoaLineL']),.5))
    x0,y0,x1,y1=map(f32,props.get('constPitchDerrCoeffMultByClLinearRatio',[0.,1.,.01,2.]))
    damping=mul(mul(kd,f32(rate)),remap(normalized,x0,x1,y0,y1))
    return [dict(target_angle=max(lower,low),target_acceleration=sub(mul(-max(sub(low,lower),-10.),kp),damping)),
            dict(target_angle=min(upper,high),target_acceleration=sub(mul(-min(sub(high,upper),10.),kp),damping))]


def authority_factor_step(authority_factor,peak,critical_high,recovery_reference,dt,adaptation_rates):
    rate=f32(adaptation_rates[int(peak>f32(critical_high))])
    change=min(1.,max(-1.,mul(sub(f32(recovery_reference),peak),rate)))
    return min(1.,max(f32(.2),add(mul(change,f32(dt)),f32(authority_factor))))


def authority_factor_run(authority_factor,peak,critical_high,recovery_reference,dt,adaptation_rates,max_steps):
    """Repeat the original rounded scalar update with invariant observations.

    Every addition still rounds to float32; multiplying the increment by the
    tick count is NOT equivalent. Return the last value, its predecessor and
    the number of changing ticks. The full controller replays the final tick.
    The optional compiled backend types only this loop's scalar temporaries.
    """
    rate=f32(adaptation_rates[int(peak>f32(critical_high))])
    change=min(1.,max(-1.,mul(sub(f32(recovery_reference),peak),rate)))
    increment=mul(change,f32(dt));value=f32(authority_factor);previous=value;steps=0
    while steps<max_steps:
        updated=min(1.,max(f32(.2),add(increment,f32(value))))
        if updated==value:break
        previous=value;value=updated;steps+=1
    return value,previous,steps


def history_repeats(records,period,tolerance):
    """Exact convergence predicate over two complete history repetitions."""
    for k in range(2*period):
        left=records[-1-k][0];right=records[-1-k-period][0]
        for i in range(len(left)):
            delta=left[i]-right[i]
            if not -tolerance<delta<tolerance:return False
    return True


def protected_pitch_command(predictor_commands,trim,authority_inverse,authority_factor,
                            requested,predicted_adjusted,adjustment_offsets,
                            critical_high,recovery_reference,dt,adaptation_rates):
    """101a94b54..101a94d8b: predictor output -> protective command clamp.

    This is an intermediate controller stage, BEFORE stall recovery and the
    mode-dependent output filters. The authority factor only scales the upper
    signed predictor command here. Preserve that native asymmetry and order.
    `authority_inverse` is the signed inverse pair computed at 101a9312d.
    """
    trim=f32(trim);negative,positive=map(f32,predictor_commands)
    def undo(command,lower):
        product=mul(sub(command,trim),trim)
        sign=1. if product>0 else -1. if product<0 else 0.
        numerator=sub(min(command,trim) if lower else max(command,trim),trim)
        return divide(numerator,sub(1.,mul(abs(trim),sign)))
    low=mul(-f32(authority_inverse[0]),max(-1.,undo(negative,True)))
    high=mul(f32(authority_inverse[1]),min(1.,mul(undo(positive,False),f32(authority_factor))))
    command=min(max(f32(requested),low),high)
    # Native prediction combines the adjusted histories with the opposite
    # side's command offsets at this stage; do not substitute raw max AoA.
    peak=max(add(f32(adjustment_offsets[1]),f32(predicted_adjusted[0])),
             sub(f32(predicted_adjusted[1]),f32(adjustment_offsets[0])))
    updated=authority_factor_step(authority_factor,peak,critical_high,recovery_reference,dt,adaptation_rates)
    return dict(command_bounds=[low,high],command=command,authority_factor=updated)
