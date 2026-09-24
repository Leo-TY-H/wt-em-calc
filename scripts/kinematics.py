"""Recovered finite-input aircraft attitude and airborne integration algorithms."""
import math
from component_assembly import f32,add,sub,mul
from tail_model import inline_half_sincos
from body_dynamics import angular_acceleration,gravity_body,airborne_linear_acceleration,rotate_acceleration_to_world,integrate_translation

DEG=f32(57.2957763671875)

def altitude_velocity_correction(height,upward_velocity,dt,*,arcade_boost=False):
    """106c64da5..64fac: post-integration world-y velocity correction.

    The outer height/velocity guards use binary64; ramp uses rounded binary32
    height. It changes velocity only, with no clamp when the correction crosses
    zero. RB/SB use arcade_boost=False (aerodynamic context+10).
    """
    if height<=9900. or upward_velocity<=0.:return upward_velocity
    h=f32(height)
    if h<=9900.:rate=0.
    elif h>=16000.:rate=-35. if arcade_boost else -3.
    elif arcade_boost:rate=add(mul(h,f32(-.005737705156207085)),f32(56.803279876708984))
    else:rate=add(mul(h,f32(-.0004918032791465521)),f32(4.868852615356445))
    return float(upward_velocity)+float(mul(rate,f32(dt)))

def restore_small_pose_change(old_position,old_quaternion,position,quaternion,
                              move_min_squared=1e-6,rotation_min_squared=1e-8):
    """106c64a1c..64d89: restore pose when both changes are below tolerance.

    Independent of skipUpdateOnSleep; velocity and angular rate are retained.
    This adapter assumes the intervening contact step produces no impulses.
    """
    dp=[float(a)-float(b) for a,b in zip(old_position,position)]
    small_move=(dp[0]*dp[0]+dp[1]*dp[1])+dp[2]*dp[2]<float(f32(move_min_squared))
    dq=[sub(f32(a),f32(b)) for a,b in zip(old_quaternion,quaternion)]
    d2=[mul(x,x) for x in dq]
    small_rotation=add(add(d2[0],d2[1]),add(d2[2],d2[3]))<f32(rotation_min_squared)
    restore=small_move and small_rotation
    return dict(position=list(old_position if restore else position),
                quaternion=list(old_quaternion if restore else quaternion),restored=restore)

def orientation_increment(q,angles_degrees):
    """Complete 10198bd30, quaternion right-product then float32 normalization.

    Inputs are the original yaw, pitch, roll parameters in degrees, not a
    conventional rotation vector. Polynomial trig and Euler product are kept.
    """
    sa,ca=inline_half_sincos(mul(f32(angles_degrees[0]),f32(math.pi/180)))
    sb,cb=inline_half_sincos(mul(f32(angles_degrees[1]),f32(math.pi/180)))
    sc,cc=inline_half_sincos(mul(f32(angles_degrees[2]),f32(-math.pi/180)))
    a=add(mul(mul(cc,sa),sb),mul(mul(sc,ca),cb))
    b=add(mul(mul(cc,sa),cb),mul(mul(sc,ca),sb))
    c=sub(mul(mul(cc,ca),sb),mul(mul(sc,sa),cb))
    d=sub(mul(mul(cc,ca),cb),mul(mul(sc,sa),sb))
    x,y,z,w=map(f32,q)
    out=[sub(add(add(mul(d,x),mul(w,a)),mul(c,y)),mul(b,z)),
         sub(add(add(mul(d,y),mul(w,b)),mul(a,z)),mul(c,x)),
         sub(add(mul(b,x),add(mul(d,z),mul(c,w))),mul(y,a)),
         sub(mul(d,w),add(mul(b,y),add(mul(c,z),mul(a,x))))]
    x,y,z,w=out
    norm=f32(math.sqrt(add(add(mul(w,w),mul(y,y)),add(mul(z,z),mul(x,x)))))
    return [mul(x,f32(1./norm)) for x in out] if norm else [0.,0.,0.,0.]

def airborne_step(position,velocity,quaternion,omega,body_force,moment,mass,inertia,dt,
                  *,outer_dt=None,sanity_factor=1.):
    """One native airborne kinematic substep; aerodynamic force is held fixed.

    Angular damping is performed by the aerodynamic prologue before this call.
    Contact substeps are excluded. Sanity factor is caller-settings +14.
    """
    dt=float(dt);outer_dt=f32(dt if outer_dt is None else outer_dt);factor=f32(sanity_factor)
    acceleration=angular_acceleration(omega,inertia,moment)
    new_omega=[omega[i]+acceleration[i]*dt for i in range(3)]
    limit=f32(mul(factor,f32(math.pi))/outer_dt)
    angular_reset=any(abs(x)>limit for x in new_omega)
    if angular_reset:new_omega=[0.,0.,0.]
    body_accel=airborne_linear_acceleration(body_force,gravity_body(quaternion,mass),mass)
    world_accel=rotate_acceleration_to_world(quaternion,body_accel)
    limit=mul(mul(f32(f32(686.4000244140625)/outer_dt),f32(f32(686.4000244140625)/outer_dt)),factor)
    acceleration_reset=sum(x*x for x in world_accel)>float(limit)
    if acceleration_reset:world_accel=[0.,0.,0.];velocity=[0.,0.,0.]
    p,v=integrate_translation(position,velocity,world_accel,dt)
    velocity_reset=sum(x*x for x in v)>float(mul(factor,2944656.))
    if velocity_reset:v=[0.,0.,0.];world_accel=[0.,0.,0.]
    midpoint=[omega[i]+acceleration[i]*(.5*dt) for i in range(3)]
    angles=[mul(f32(midpoint[1]*dt),-DEG),mul(f32(midpoint[2]*dt),-DEG),mul(f32(midpoint[0]*dt),DEG)]
    q=orientation_increment(quaternion,angles)
    return dict(position=p,velocity=v,quaternion=q,omega=new_omega,angular_acceleration=acceleration,
                world_acceleration=world_accel,reset=angular_reset or acceleration_reset or velocity_reset)
