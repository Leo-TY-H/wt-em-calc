"""Atmosphere and cached air state, 0x101988170 / 0x101a33880."""
import math
from component_assembly import f32,add,sub,mul
from control_mixer import density_at_height


def speed_of_sound(height,temperature0=f32(288.16),ceiling=f32(18300.)):
    h=min(f32(height),f32(ceiling));p=f32(3.9730601514748866e-18)
    for c in [-5.711039686951899e-14,2.1806899341836328e-10,-2.277120074722916e-05,1.]:p=add(mul(p,h),f32(c))
    return mul(f32(math.sqrt(mul(p,f32(temperature0)))),f32(20.1))


def world_to_body_air(quaternion,world_velocity,wind=(0.,0.,0.),additional_wind=(0.,0.,0.)):
    """Exact float32 rotation after double world velocity/wind subtraction."""
    vx,vy,vz=[f32(world_velocity[i]-(additional_wind[i]+wind[i])) for i in range(3)]
    x,y,z,w=map(f32,quaternion)
    base=add(add(mul(w,w),mul(w,w)),-1.)
    r00=add(add(mul(x,x),mul(x,x)),base);r11=add(add(mul(y,y),mul(y,y)),base);r22=add(add(mul(z,z),mul(z,z)),base)
    x2,y2=add(x,x),add(y,y);wn=mul(w,-2.)
    xy,yz,xz,wy,wz,wx=mul(x2,y),mul(y2,z),mul(x2,z),mul(y,wn),mul(wn,z),mul(wn,x)
    u=add(add(mul(sub(xy,wz),vy),mul(add(wy,xz),vz)),mul(vx,r00))
    v=add(add(mul(sub(yz,wx),vz),mul(add(xy,wz),vx)),mul(vy,r11))
    ww=add(mul(r22,vz),add(mul(sub(xz,wy),vx),mul(add(yz,wx),vy)))
    return [u,v,ww]


def cache(velocity,height,rho0=f32(1.225),temperature0=f32(288.16),ceiling=f32(18300.)):
    """FM+8454 alpha, +8458 beta, +845c TAS, +8460 TAS², +8464 M,
    +8468 longitudinal IAS. Velocity has already been rounded by the rotation.
    """
    u,v,w=velocity
    square=f32((w*w+u*u)+v*v);tas=f32(math.sqrt(square))
    rho=density_at_height(f32(height),f32(rho0),f32(ceiling))
    alpha=mul(f32(math.atan2(v,u)),f32(-57.2957763671875)) if u or v else -0.
    beta=mul(f32(math.atan2(w,u)),f32(57.2957763671875)) if u or w else 0.
    ias=f32(u*f32(math.sqrt(f32(rho/f32(rho0)))))
    mach=f32(tas/speed_of_sound(height,temperature0,ceiling))
    return dict(alpha=alpha,beta=beta,tas=tas,speed_squared=square,mach=mach,ias_u=ias,density=rho)
