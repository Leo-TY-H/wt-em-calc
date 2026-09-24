"""Recovered post-aerodynamic stages, using the executable's stored frame.

Forces/positions are x forward, y up, z right. The stored angular rates and
moments have opposite sign to right-hand angular pseudovectors in this basis.
All functions explicitly state whether they preserve target float grouping.
"""
import math
from component_assembly import f32,add,sub,mul

TINY = f32(4e-19)
G = f32(9.81)


def detailed_path(state_flags, player_controlled):
    """0x101a3f94f: realisticAI || (player_controlled && playerEnabled)."""
    return bool(state_flags & 0x1000000 or player_controlled and state_flags & 0x200000)


def preprocess_angular_rate(stored_omega, longitudinal_ias):
    """106c5ce04..5cea1: unconditional per-update damping, then 5 rad/s cap.

    longitudinal_ias is FM+8468, float32(V_body.x * sqrtf(rho/rho0)).
    There is no dt in this stage; do not substitute a continuous torque.
    """
    scale=max(1.0+float(f32(longitudinal_ias))*(-1e-5),.8)
    x,y,z=[w*scale for w in stored_omega]
    norm2=z*z+(x*x+y*y)
    if norm2>25.0:
        cap=5.0/math.sqrt(norm2)
        x,y,z=x*cap,y*cap,cap*z
    return [x,y,z]


def parasite_force(body_velocity, density, cockpit_door_cd, door_fraction,
                   attachment_drag_area=0., cockpit_intact=True):
    """106c6051d..60575 and 106c626a7..627a8, drag applied at CG.

    This diagnostic 'parasite' term does not include FuseCd: that coefficient
    is added inside each wing's drag-device calculation. Body velocity is
    rounded to float32 before the pressure and direction calculations.
    """
    x,y,z=map(f32,body_velocity)
    norm2=add(mul(z,z),add(mul(x,x),mul(y,y)))
    speed=f32(math.sqrt(norm2))
    if speed>1e-9:
        reciprocal=f32(1./speed)
        direction=[mul(v,reciprocal) for v in (x,y,z)]
    else:
        direction=[0.,0.,0.]
    q=mul(norm2,mul(f32(density),.5))
    door=f32(door_fraction) if cockpit_intact else 1.
    area=add(mul(door,f32(cockpit_door_cd)),f32(attachment_drag_area))
    xy_scale=mul(area,-q)
    return [mul(xy_scale,direction[0]),mul(xy_scale,direction[1]),
            mul(mul(-q,direction[2]),area)]


def gravity_body(quaternion, mass, gravity=G):
    """0x106c6383d..638e2, float32 body gravity then double promotion.

    quaternion is (x,y,z,w), representing body-to-world orientation.
    The caller calculates weight in float32 at 0x106c62d5c..62d61.
    """
    x,y,z,w=map(f32,quaternion); weight=mul(gravity,mass)
    negative_twice_weight=mul(-2,weight)
    return [mul(add(mul(y,x),mul(z,w)),negative_twice_weight),
            mul(add(-1,mul(2,add(mul(y,y),mul(w,w)))),-weight),
            mul(sub(mul(z,y),mul(x,w)),negative_twice_weight)]


def rotate_acceleration_to_world(quaternion, acceleration):
    """0x106c641ea..64348: round acceleration to f32, rotate, promote to f64."""
    x,y,z,w=map(f32,quaternion);ax,ay,az=map(f32,acceleration)
    wx2,xx2,yx2=add(w,w),add(x,x),add(y,y)
    xy2=mul(y,xx2);yz2=mul(z,yx2);wy2=mul(wx2,y)
    wz2=mul(wx2,z);xw2=mul(xx2,w);xz2=mul(z,xx2)
    yy2=mul(2,mul(y,y));ww=mul(w,w);ww2m1=add(add(ww,ww),-1)
    r00=add(add(mul(x,x),mul(x,x)),ww2m1)
    r01=sub(xy2,wz2);r02=add(xz2,wy2)
    r10=add(wz2,xy2);r11=add(yy2,ww2m1);r12=sub(yz2,xw2)
    r20=sub(xz2,wy2);r21=add(xw2,yz2);r22=add(add(mul(z,z),mul(z,z)),ww2m1)
    return [add(add(mul(r02,az),mul(ay,r01)),mul(r00,ax)),
            add(mul(az,r12),add(mul(r10,ax),mul(ay,r11))),
            add(mul(az,r22),add(mul(r21,ay),mul(ax,r20)))]


def limit_total_moment(moment, mass, gravity=G):
    """0x106c63e09..63e7f, finite inputs: norm cap = float32(150*weight)."""
    x,y,z=moment; length=math.sqrt((x*x+y*y)+z*z)
    limit=float(mul(mul(gravity,mass),150))
    scale=limit/length if length>limit else 1.0
    return [scale*x,scale*y,scale*z]


def limit_aerodynamic_force(force, mass, gravity=G):
    """106c62dca..62ea1, finite inputs: aero-only norm cap = float32(50*weight).

    This occurs before external forces, propulsion and gravity. Its SSE norm
    adds z squared, x squared, then y squared, in that order.
    """
    x,y,z=force;length=math.sqrt((z*z+x*x)+y*y)
    limit=float(mul(mul(f32(gravity),f32(mass)),50))
    scale=limit/length if length>limit else 1.
    return [scale*x,scale*y,scale*z]


def angular_acceleration(omega, inertia, applied_moment, contact_moment=(0.,0.,0.)):
    """0x106c63ffd..640fc: diagonal Euler equations in stored angular signs.

    All arguments and arithmetic here are float64. Near-zero inertia returns
    zero after a masked divide; negative nonzero inertia is retained by code.
    applied_moment is the total after the 150*weight cap. Contact is added later.
    """
    wx,wy,wz=omega;ix,iy,iz=inertia
    tx,ty,tz=[contact_moment[i]+applied_moment[i] for i in range(3)]
    numerators=[(wy*wz)*(iz-iy)+tx,(wz*wx)*(ix-iz)+ty,(wy*wx)*(iy-ix)+tz]
    return [n/d if abs(d)>TINY else 0. for n,d in zip(numerators,inertia)]


def integrate_translation(position,velocity,world_acceleration,dt):
    """0x106c64411..644be; dt is the prepared double substep duration."""
    halfdt2=(dt*dt)*.5
    return ([position[i]+(velocity[i]*dt+world_acceleration[i]*halfdt2) for i in range(3)],
            [world_acceleration[i]*dt+velocity[i] for i in range(3)])


def compose_force(aerodynamic_force,external_force,engine_force,engine_scale=1.):
    """0x106c6374a..637fb; engine-scale input was rounded to float32 upstream.

    External force was float32 before promotion, whereas aerodynamic/engine
    accumulators are double. Gravity is intentionally supplied separately.
    """
    scale=float(f32(engine_scale))
    return [(aerodynamic_force[i]+float(f32(external_force[i])))+engine_force[i]*scale for i in range(3)]


def compose_moment(aerodynamic_moment,external_moment,engine_moment,gyro=(0.,0.,0.)):
    """634da..634ea,63643..6365f,63708..63728; selected helpers are zero.

    The engine force modifier never scales the engine moment. All terms are
    double accumulators except the external float32 moment, promoted here.
    """
    return [((aerodynamic_moment[i]+float(f32(external_moment[i])))+
             engine_moment[i])+gyro[i] for i in range(3)]


def airborne_linear_acceleration(body_force,body_gravity,mass):
    """63bbc..63c2a and641c6..641ea, before float32 body-to-world rotation.

    Assumes no contacts. The inverse mass is float32 before double promotion.
    The contact/gravity limiter cannot bind when contact forces are zero:
    its bound is at least twice the gravity vector's norm.
    """
    mass=f32(mass)
    inverse_mass=float(f32(1./mass)) if abs(mass)>TINY else 0.
    return [(body_gravity[i]+body_force[i])*inverse_mass for i in range(3)]


def physical_angular_vector(stored_vector):
    """Negate stored angular components, retaining the x-forward/y-up/z-right basis."""
    return [-v for v in stored_vector]


def realistic_engine_scale(ext_thrust_mult=1., ext_thrust_base_mult=1.):
    """106c6347e..63616 with arcadeBoost=false, wepOverspeed=1.

    ext_thrust_mult is the unit's named network field, default 1. It is an
    external mission/effect multiplier, separate from engine RPM or throttle.
    """
    return add(mul(sub(f32(ext_thrust_mult),1),f32(ext_thrust_base_mult)),1)


def nozzle_direction_from_sincos(basis, sin_a, cos_a, sin_b, cos_b):
    """1019f1606..164f after sincosf, preserving float32 grouping.

    basis is (B0, B1, B2), initialized from Direction/Direction2. This routine
    accepts the target sincosf results, so it makes no claim about host libm.
    All deflection tables are zero for the two selected jets; their normal
    in-flight direction is exactly B0 = (1,0,0).
    """
    b0,b1,b2=[list(map(f32,b)) for b in basis]
    sin_a,cos_a,sin_b,cos_b=map(f32,(sin_a,cos_a,sin_b,cos_b))
    s=[add(mul(b2[i],sin_a),mul(b0[i],cos_a)) for i in range(3)]
    return [add(mul(s[i],cos_b),mul(b1[i],-sin_b)) for i in range(2)]+[
        add(mul(b1[2],-sin_b),mul(s[2],cos_b))]


def accumulate_nozzle(direction, nozzle_position, cg, capped_thrust,
                      control_multiplier=1., force=(0.,0.,0.), moment=(0.,0.,0.)):
    """1019f1f2b..1fe2, one nozzle's exact float32 F and stored-M accumulation.

    capped_thrust is min(flapMultiplier * scalarThrust * ThrustRatio, ThrustMax).
    The caller performs nozzle accumulation in float32, then promotes each
    engine's result to float64. Moment uses F cross (nozzlePosition - CG).
    """
    thrust=mul(f32(control_multiplier),f32(capped_thrust))
    fx,fy,fz=[mul(f32(x),thrust) for x in direction]
    rx,ry,rz=[sub(f32(p),f32(c)) for p,c in zip(nozzle_position,cg)]
    force=list(map(f32,force));moment=list(map(f32,moment))
    accumulated_force=[add(old,new) for old,new in zip(force,(fx,fy,fz))]
    accumulated_moment=[add(mul(fy,rz),sub(moment[0],mul(fz,ry))),
                        add(moment[1],sub(mul(fz,rx),mul(fx,rz))),
                        add(sub(mul(fx,ry),mul(fy,rx)),moment[2])]
    return accumulated_force,accumulated_moment


def gyroscopic_moment(engine_angular_momentum, stored_omega, scale=1.):
    """101a17a40 plus 106c636f6..63728: H_engine cross stored omega.

    The engine owner resets H_engine each tick and only the propeller loop
    contributes to it. Both selected jets have no propellers and H_engine=0.
    scale is the wing/stall stage's double rbp-0xb80, normal-branch value 1.
    """
    hx,hy,hz=engine_angular_momentum;wx,wy,wz=stored_omega
    return [scale*(wz*hy-hz*wy),scale*(wx*hz-hx*wz),scale*(hx*wy-wx*hy)]
