"""Post-step wing-flex and overspeed observables, separate from wing-break stress."""
import math
from component_assembly import f32,add,sub,mul
from structural_limits import interval


def spring_step(stiffness,damping,limit,position,velocity,forcing,dt):
    """Complete101a907a0. Retain the native split integration and loop bounds.

    h=1/480, at most10 iterations. Even dt=0 executes once; the guard is
    remaining_dt>-1e-5. Limit crossing sets position to sign(position), not
    sign(position)*limit. All arithmetic below is explicitly binary32.
    """
    k,c,limit,x,v,forcing,remaining=map(f32,(stiffness,damping,limit,position,velocity,forcing,dt))
    h=f32(.0020833334419876337);half_h2=f32(2.170139168811147e-6)
    hf=mul(h,forcing);hhf=mul(half_h2,forcing);iterations=0
    while remaining>f32(-1e-5) and iterations<10:
        vh=add(v,hf);xp=add(add(mul(vh,h),hhf),x)
        a=sub(mul(xp,-k),mul(c,vh));v=add(mul(a,h),vh)
        x=add(add(mul(a,half_h2),xp),mul(v,h))
        remaining=add(remaining,-h);iterations+=1
    if abs(x)>limit:x=1. if x>0. else -1. if x<0. else 0.;v=0.
    return dict(position=x,velocity=v,iterations=iterations)


def wing_observables(quaternion,empty_mass,wing_mass_fraction,spring_multipliers,strength,
                     wing_y,reaction_y,reaction_moment_x,inertia_x,wing_arm_z,mass,ias_u,vne,dt,
                     flex_state,*,spring_enabled=True,flutter_enabled=True,gravity=9.81,
                     spring_arm_multiplier=1.,wave_range=(.88,1.),fake_wave_range=(.9,1.15)):
    """Complete101a2e350 for no ground contact; selected spring mode is enabled.

    wing_mass_fraction is FM7c9c: loader uses half of FM WingWaveMassRel.
    flex_state has two [position,velocity] pairs (FMa2f8/fc, a30c/10).
    reaction_y and reaction_moment_x are the caller's gravity/contact force and
    contact moment, NOT the complete aerodynamic+engine force and moment.
    Each output retains the spring state needed by the next call. Ground rolling
    animation when spring mode is disabled is outside this airborne adapter.
    """
    def inv(v):return f32(1./v) if abs(v)>f32(4e-19) else 0.
    x,y,z,w=map(f32,quaternion)
    nx=mul(sub(mul(y,x),mul(w,z)),2.)
    nz=mul(add(mul(w,x),mul(z,y)),2.)
    ny=add(mul(add(mul(w,w),mul(y,y)),2.),-1.)
    norm=f32(math.sqrt(add(mul(ny,ny),add(mul(nx,nx),mul(nz,nz)))))
    wing_mass=mul(f32(empty_mass),f32(wing_mass_fraction))
    wing_gravity=mul(mul(mul(f32(gravity),wing_mass),inv(norm)),ny)
    largest=max(-f32(strength[0]),f32(strength[1]));dt=f32(dt)
    next_state=[]
    if spring_enabled:
        inv_wing_mass=inv(wing_mass);scale=mul(largest,inv_wing_mass)
        k,c=[mul(scale,f32(p)) for p in spring_multipliers]
        roll=mul(mul(mul(mul(f32(mass),inv_wing_mass),
                             f32(f32(reaction_moment_x)/f32(inertia_x)) if abs(f32(inertia_x))>f32(4e-19) else 0.),
                         f32(wing_arm_z)),f32(spring_arm_multiplier))
        linear=f32(f32(reaction_y)/f32(mass)) if abs(f32(mass))>f32(4e-19) else 0.
        forces=[add(sub(mul(sub(f32(wing_y[0]),wing_gravity),inv_wing_mass),roll),linear),
                add(add(mul(sub(f32(wing_y[1]),wing_gravity),inv_wing_mass),linear),roll)]
        for state,force in zip(flex_state,forces):
            out=spring_step(k,c,1.,*state,force,dt);next_state.append([out['position'],out['velocity']])
        wing_wave=[s[0] for s in next_state]
    else:
        next_state=[list(map(f32,s)) for s in flex_state]
        wing_wave=[max(-1.,min(1.,mul(inv(largest),sub(f32(y),wing_gravity)))) for y in wing_y]
    low,high=wave_range if flutter_enabled else fake_wave_range
    lo=mul(f32(low),f32(vne));hi=mul(f32(high),f32(vne));speed=f32(ias_u)
    wave=interval(speed,lo,0.,hi,1.) if speed>lo else 0.
    return dict(wing_wave=wing_wave,flex_state=next_state,overspeed_wave=mul(wave,wave))
