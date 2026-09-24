"""Recovered pieces of the Instructor's reduced aircraft predictor.

These research stages retain the native iteration rules and float32 operation
order. instructor_pitch_predictor.py and instructor_autotrim.py compose them
with independent geometry, control and wake preparation. They are the game's
reduced controller model, not an aircraft EM model.
"""
import math
from component_assembly import f32,add,sub,mul
from polar_f32 import calc_cd,calc_cl
from instructor_protection import divide
from tail_model import inline_half_sincos


def inverse_rotated_cl(p,target,angle,convert_aoa=False,initial_output=0.):
    """10198c900 + its finite-difference Newton loop 10198c9b0.

    The caller initializes its output separately. Saturated targets write the
    critical angle with false status; a failed Newton solve leaves that output
    unchanged. A successful solve terminates on a <.01-degree Newton step,
    not a coefficient residual. It tries at most 15 updates from zero degrees.
    """
    target=f32(target);angle=f32(angle)
    if target>add(mul(p['cyCritH'],p['clKq']),f32(-.001)):
        return p['aoaCritH'],False
    if target<add(mul(p['clKq'],p['cyCritL']),f32(.001)):
        return p['aoaCritL'],False
    def projected(a):
        r=mul(angle if convert_aoa else sub(a,angle),f32(.01745329238474369))
        sn=f32(math.sin(r));cs=f32(math.cos(r))
        return mul(add(mul(calc_cd(p,a),sn),mul(calc_cl(p,a),cs)),p['clKq'])
    x=0.;delta=f32(.01);inv_delta=f32(1./delta)
    for _ in range(15):
        residual=sub(projected(x),target)
        # The original has a zero residual tolerance, so its residual shortcut
        # cannot fire for finite values. Preserve its derivative grouping.
        derivative=mul(add(sub(-residual,target),projected(add(x,delta))),inv_delta)
        if abs(derivative)<f32(4e-19):return f32(initial_output),False
        change=f32(-residual/derivative)
        x=add(x,change)
        if abs(change)<delta:return x,True
    return f32(initial_output),False


def pitch_balance(*, cd,cl,control_cl,extra_cd,fuselage_cd,q_area,negative_q_area,
                  cos_dihedral,cm0,cm1,polar_area_over_span,wing_x,wing_y,
                  other_moment,engine_pitch_moment,pitch_inertia,target_acceleration,
                  weight,flight_path_cos,reference_x,tail_lever,working_alpha,
                  legacy_balance_shift=False):
    """Mode-1 balance 101a5f4cf..101a5f7f3, prepared two-wing geometry.

    The reduced predictor solves the required horizontal-tail force from wing
    drag/lift moments, intrinsic moments, engine moment and desired angular
    acceleration. wing_x/wing_y retain the native signed, prepared levers.
    other_moment includes its separately prepared fuselage/vertical-tail drag.
    This stage does not by itself establish that the tail can supply the force.
    """
    lift_coeff=[add(control_cl,c) for c in cl]
    drag=[mul(add(add(c,fuselage_cd),e),q) for c,e,q in zip(cd,extra_cd,negative_q_area)]
    lift=[mul(mul(cos_dihedral,q),c) for q,c in zip(q_area,lift_coeff)]
    intrinsic=[]
    for d,c,c0,c1 in zip(cd,lift_coeff,cm0,cm1):
        intrinsic.append(f32(mul(add(mul(c1,c),c0),polar_area_over_span)/add(mul(c,c),mul(d,d))))
    lift_moment=mul(sub(mul(intrinsic[0],lift_coeff[0]),wing_x[0]),lift[0])
    lift_moment=sub(lift_moment,mul(f32(pitch_inertia),target_acceleration))
    lift_moment=sub(lift_moment,mul(mul(weight,flight_path_cos),reference_x))
    lift_moment=add(lift_moment,mul(sub(mul(intrinsic[1],lift_coeff[1]),wing_x[1]),lift[1]))
    moment=add(other_moment,engine_pitch_moment)
    for i in range(2):moment=add(moment,mul(sub(wing_y[i],mul(intrinsic[i],cd[i])),drag[i]))
    moment=add(moment,lift_moment)
    summed_drag=add(drag[1],drag[0]);summed_lift=add(lift[1],lift[0])
    if legacy_balance_shift:
        shift=mul(f32(math.sin(mul(abs(working_alpha),f32(.01745329238474369)))),.5)
        moment=add(moment,mul(summed_lift,shift))
        denominator=sub(tail_lever,add(shift,reference_x))
    else:denominator=sub(tail_lever,reference_x)
    return dict(wing_force=[summed_drag,summed_lift],tail_force=divide(moment,denominator))


def tail_command(*,flow_angle,required_angle,flap_incidence,tail_incidence,
                 area_sensitivity_scale,has_sensitivity,center,positive_delta,
                 negative_delta,inverted):
    """101a60402..101a604e3: tail incidence -> bounded elevator command.

    The signed deltas are the predictor's prepared deflection limits relative
    to center. Endpoint equality counts as saturation in the native code.
    """
    incidence=add(flap_incidence,tail_incidence)
    requested=mul(add(sub(flow_angle,required_angle),incidence),area_sensitivity_scale)
    if not has_sensitivity:requested=0.
    lo=add(negative_delta,center);hi=add(positive_delta,center)
    saturated=requested<=lo or requested>=hi
    deflection=min(max(requested,lo),hi)
    delta=negative_delta if deflection<=center else positive_delta
    denominator=-delta if deflection<=center else delta
    command=divide(sub(deflection,center),denominator)
    if inverted:command=-command
    return dict(command=command,saturated=saturated,deflection=deflection)


def control_iteration(index,current,proposed,*,saturated,aileron_effect_range,
                      elevon_effect_range,direct_lift_authority):
    """101a6059e..6062a and 101a5e140..e15e, including tenth-pass damping."""
    tiny=f32(.001)
    no_effect=(aileron_effect_range<=tiny and elevon_effect_range<=tiny and direct_lift_authority<=tiny)
    if no_effect or (index>3 and (saturated or abs(sub(proposed,current))<tiny)):
        return dict(done=True,command=proposed)
    relaxed=add(current,mul(sub(proposed,current),.5))
    return dict(done=index+1==10,command=relaxed)


def legacy_wake_factor(wash_attenuation, axial_speed, aspect):
    """101a5dd54/e48e: attenuation/pi * axial speed / prepared polar aspect."""
    return f32(mul(mul(f32(1./math.pi),wash_attenuation),axial_speed)/aspect)


def tail_flow(*,downwash_type=2,legacy_factor=0.,**inputs):
    """Original reduced wake dispatch, including legacy modes 0 and 1.

    Mode 0 ignores propwash. Mode 1 uses the LEFT raw-angle CL, rather than
    the deflected/rotated lift in the moment balance. Its swirl sign does not
    use ClockWiseAOA. Native spans 101a5fa2e..fb38 and 101a60370..60402.
    """
    if downwash_type==2:return type2_tail_flow(**inputs)
    if downwash_type not in (0,1):raise ValueError('Unknown reduced downwash type')
    sn,cs,speed=inputs['sin_angle'],inputs['cos_angle'],inputs['speed']
    vx=mul(cs,speed)
    vy=sub(mul(sn,-speed),mul(inputs['pitch_rate'],inputs['tail_lever']))
    if downwash_type==1:
        induced=mul(calc_cl(inputs['wing_polars'][0],inputs['wing_angle']),legacy_factor)
        vy=add(mul(cs,induced),vy)
        vx=add(mul(induced,sn),vx)
        if inputs['engine_count']==1:
            swirl=(.36000001430511475*float(inputs['tail_area_delta'])
                   *float(inputs['swirl_wash'])*float(inputs['inverse_tail_area']))
            vy=float(vy)-swirl
            vx=float(vx)+float(inputs['axial_wash'])
    return mul(f32(math.atan2(vy,vx)),f32(-57.2957763671875))


def type2_tail_flow(*,span,area,sweep,taper,dihedral,working_alpha,wing_angle,
                   wing_polars,wing_x,wing_y,wing_z,tail_point,tail_polar_offset,
                   coefficient,pitch_rate,tail_lever,speed,sin_angle,cos_angle,
                   engine_count,clockwise,axial_wash,swirl_wash,tail_area_delta,inverse_tail_area):
    """Reduced type-2 wake 101a5fb8c..101a60402; distinct from full FM wake.

    It sums the two wing contributions at a single tail reference, with no
    convected wing-CL history. It uses the same inline half-angle polynomial
    as the detailed FM, but its projection and scaling operation order differ.
    wing_x/y/z are the reduced predictor's prepared geometry, not EM points.
    """
    rad=f32(.01745329238474369)
    sa,ca=inline_half_sincos(f32(math.pi));sb,cb=inline_half_sincos(mul(working_alpha,rad))
    invhalf=f32(2./span) if abs(mul(span,.5))>f32(4e-19) else 0.
    contributions=[]
    for i in range(2):
        sc,cc=inline_half_sincos(mul(dihedral if i==0 else -dihedral,rad))
        # Match the reduced predictor's product grouping, not the full FM's
        # algebraically equivalent quaternion helper.
        x=add(mul(mul(cc,sa),sb),mul(mul(sc,ca),cb))
        y=add(mul(mul(cc,sa),cb),mul(mul(sc,ca),sb))
        z=sub(mul(mul(cc,ca),sb),mul(mul(sc,sa),cb))
        w=sub(mul(mul(cc,ca),cb),mul(mul(sc,sa),sb))
        base=add(add(mul(w,w),mul(w,w)),-1.)
        dx=add(add(tail_polar_offset,tail_point[0]),wing_x[i])
        dy=sub(tail_point[1],wing_y[i])
        dz2=mul(sub(tail_point[2],wing_z[i]),2.) if i==0 else mul(add(tail_point[2],wing_z[i]),-2.)
        xy=mul(add(x,x),y);zw=mul(sub(-z,z),w)
        px=mul(add(add(mul(sub(mul(x,z),mul(y,w)),dz2),mul(sub(xy,zw),dy)),
                   mul(add(add(mul(x,x),mul(x,x)),base),dx)),invhalf)
        if px>0.:
            py=mul(invhalf,add(add(mul(dz2,add(mul(x,w),mul(z,y))),mul(dx,add(xy,zw))),
                              mul(dy,add(base,add(mul(y,y),mul(y,y))))))
            along=add(f32(math.exp(mul(px,-2.5))),f32(.9))
            across=f32(math.exp(mul(abs(py),f32(-1.409678))))
            contributions.append(mul(mul(calc_cl(wing_polars[i],wing_angle),along),across))
        else:contributions.append(0.)
    shape=sub(f32(1.56),mul(f32(math.exp(mul(taper,f32(-.35)))),f32(1.2)))
    shape=mul(shape,mul(area,f32(-23.1)))
    numerator=mul(mul(mul(add(f32(math.sin(mul(sweep,rad))),1.),add(*contributions)),shape),coefficient)
    vx=mul(cos_angle,speed);vy=sub(mul(sin_angle,-speed),mul(pitch_rate,tail_lever))
    if engine_count==1:
        swirl=float(swirl_wash)*float(tail_area_delta)*float(inverse_tail_area)*(.36000001430511475 if clockwise else -.36000001430511475)
        angle=f32(math.atan2(float(vy)+swirl,float(vx)+float(axial_wash)))
    else:angle=f32(math.atan2(vy,vx))
    return add(mul(angle,f32(-57.2957763671875)),f32(numerator/mul(span,span)))
