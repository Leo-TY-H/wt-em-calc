"""Intact connected fixed-wing propellers, with original float32 ordering.

Airborne use with terrain augmentation disabled. Properties are explicit;
configuration loading and engine/transmission integration are separate stages.
"""
import math
from functools import lru_cache
from component_assembly import f32,add,sub,mul
from piston_model import div
from propeller_model import blade_forces
from propeller_step import sign,clamp,tiny
from structural_limits import interval


IDENTITY=[1.,0.,0.,0.,1.,0.,0.,0.,1.]


def transform(b,v):
    return [add(add(mul(v[2],b[i+6]),mul(v[1],b[i+3])),mul(v[0],b[i])) for i in range(3)]


def inverse(b):
    a,d,g,c,e,h,k,f,i=b
    # Columns of the inverse; determinant terms retain separate products.
    det=sub(add(add(mul(mul(a,e),i),mul(mul(d,h),k)),mul(mul(g,c),f)),
            add(add(mul(mul(g,e),k),mul(mul(d,c),i)),mul(mul(a,h),f)))
    return [div(x,det) for x in [sub(mul(e,i),mul(h,f)),sub(mul(g,f),mul(d,i)),sub(mul(d,h),mul(g,e)),
            sub(mul(h,k),mul(c,i)),sub(mul(a,i),mul(g,k)),sub(mul(g,c),mul(a,h)),
            sub(mul(c,f),mul(e,k)),sub(mul(d,k),mul(a,f)),sub(mul(a,e),mul(d,c))]]


@lru_cache(maxsize=128)
def inverse_basis(b):
    return tuple(inverse(b))


@lru_cache(maxsize=512)
def local_flow(basis,r,v,w):
    """Fixed inflow geometry across a shaft/governor settling sequence."""
    local=[add(sub(mul(r[1],w[2]),mul(r[2],w[1])),v[0]),
           add(sub(mul(r[2],w[0]),mul(r[0],w[2])),v[1]),
           add(sub(mul(r[0],w[1]),mul(r[1],w[0])),v[2])]
    inv=inverse_basis(basis)
    local=[add(add(mul(local[0],inv[i]),mul(local[1],inv[i+3])),mul(local[2],inv[i+6])) for i in range(3)]
    local_w=[add(add(mul(w[2],inv[i+6]),mul(w[1],inv[i+3])),mul(w[0],inv[i])) for i in range(3)]
    transverse_sq=add(mul(local[2],local[2]),mul(local[1],local[1]))
    return local,local_w,transverse_sq,f32(math.sqrt(transverse_sq))


def curve3(rows,x):
    if not rows:return [0.,0.,0.]
    if x<=rows[0][0]:return rows[0][2:]
    for lo,hi in zip(rows,rows[1:]):
        if x<=hi[0]:
            t=mul(sub(x,lo[0]),lo[1])
            return [add(mul(sub(b,a),t),a) for a,b in zip(lo[2:],hi[2:])]
    return rows[-1][2:]


def governor(p,pitch,gp,omega,previous,target,command,auto,boost,neutral,dt):
    kind=p['governor'];lo,hi=p['pitch_min'],p['pitch_max'];speed=p['governor_speed']
    if kind==0:return lo,gp,0.
    if kind not in (1,2,6,8):raise ValueError('Unimplemented fixed-wing governor '+str(kind))
    if kind in (1,2) or auto:
        if kind in (1,2):
            maximum=p['boost_omega'] if boost else p['max_omega']
            delivered=clamp(div(sub(target,p['min_omega']),sub(maximum,p['min_omega'])),0.,1.) if auto else command
            target=add(mul(sub(maximum,p['min_omega']),delivered),p['min_omega'])
        gain=mul(f32(.0001),p['reduction']);derivative=mul(p['reduction'],f32(.004))
        if p['governor_fast']:
            gain=div(gain,speed);derivative=div(derivative,mul(speed,speed))
        now=mul(omega,omega)
        error=sub(mul(gain,sub(mul(target,target),now)),mul(derivative,sub(now,mul(previous,previous))))
        if p['governor_fast']:
            pitch=sub(pitch,mul(mul(speed,dt),clamp(error,-.35,.35)))
            # Native fast governor 6 uses the neutral-inflow floor, not pitch_min.
            minimum=add(neutral,f32(-.12)) if kind==6 else max(lo,add(neutral,f32(-.12)))
            if kind==8:minimum=max(minimum,add(lo,f32(.05235988)))
            pitch=clamp(pitch,minimum,hi)
        else:
            limit=.35 if kind in (1,2) else 1.
            gp=clamp(sub(gp,mul(clamp(error,-limit,limit),dt)),add(neutral,f32(-.12)),hi)
            if kind==8:gp=clamp(gp,add(lo,f32(.05235988)),hi)
            pitch=clamp(add(pitch,mul(mul(speed,dt),clamp(mul(sub(gp,pitch),f32(.6)),-1.,1.))),lo,hi)
        if kind in (1,2):
            reported=command if not auto or p['pitch_command_report'] else delivered
        else:reported=command if auto and p['pitch_command_report'] else clamp(div(sub(hi,pitch),sub(hi,lo)),0.,1.)
    else:
        gp=add(mul(sub(lo,hi),command),hi)
        delta=sub(gp,pitch);rate=mul(mul(speed,dt),sign(delta))
        pitch=clamp(add(pitch,min(rate,delta) if delta>=0. else max(rate,delta)),lo,hi)
        reported=clamp(div(sub(hi,pitch),sub(hi,lo)),0.,1.)
    return pitch,gp,reported


def step(p,state,velocity=(100.,0.,0.),body_omega=(0.,0.,0.),cg=(0.,0.,0.),
         omega=180.,previous_omega=None,target_omega=270.,command=1.,auto=False,
         density=1.225,sound_speed=340.,dt=1/60,afterburner=False,torque_gyro=True):
    if p['cyclic'] or p['differential_pitch'] or p['active_pitch_2d'] or p['pitch_1d_count']:
        raise ValueError('Unsupported cyclic or scheduled-pitch geometry')
    omega,previous,target,command,density,sound_speed,dt=map(f32,
        [omega,omega if previous_omega is None else previous_omega,target_omega,command,density,sound_speed,dt])
    v=list(map(f32,velocity));w=list(map(f32,body_omega));r=p['position'];basis=p['basis']
    local,local_w,transverse_sq,transverse=local_flow(tuple(basis),tuple(r),tuple(v),tuple(w))
    flow=list(map(f32,state.get('flow',[0.,0.,0.])))
    pitch=f32(state.get('pitch',p['pitch_min']));gp=f32(state.get('governor_pitch',pitch))
    disc=mul(mul(p['diameter'],p['diameter']),f32(.7853981852531433));rho_disc=mul(density,disc)
    airflow=curve3(p['airflow'],local[0]);ambient=add(airflow[0],local[0])
    blade_inputs=[];last_delta=[0.,0.];relax=f32(.2)
    for iteration in range(10 if p['iterative'] else 1):
        axial=add(mul(add(flow[1],flow[0]),.5),local[0])
        args=[omega,pitch,axial,transverse,0.,0.,density,sound_speed,mul(flow[2],.25)]
        blade_inputs.append(args);blade=blade_forces(p,*args);thrust=blade['thrust'];torque=blade['torque'];first=thrust
        if p['coaxial']:
            args=args[:-1]+[mul(flow[2],-.25)]
            blade_inputs.append(args);second=blade_forces(p,*args)
            thrust=add(thrust,second['thrust']);torque=add(torque,second['torque'])
        def axial_flow(t,u):
            value=add(div(add(t,t),rho_disc),mul(sign(u),mul(u,u)))
            value=tiny(sub(mul(sign(value),f32(math.sqrt(abs(value)))),u))
            return clamp(tiny(mul(mul(sign(t),airflow[2]),value)),-150.,150.)
        x=axial_flow(first,ambient)
        if p['coaxial']:
            y=axial_flow(thrust,add(mul(x,.5),ambient));z=0.
        else:
            y=0.;u=add(mul(x,.5),local[0])
            flux=mul(mul(f32(math.sqrt(add(mul(u,u),transverse_sq))),disc),density)
            half=mul(p['diameter'],.5);divisor=mul(mul(half,half),mul(flux,.5))
            z=clamp(tiny(div(torque,divisor)) if abs(divisor)>f32(4e-19) else 0.,0.,20.)
        delta=[clamp(sub(a,b),-10.,10.) for a,b in zip([x,y],flow[:2])];dz=sub(z,flow[2])
        if not p['iterative'] or max(abs(a) for a in delta)<f32(.05) and abs(dz)<f32(.05):
            flow=[x,y,z];break
        flow=[add(mul(relax,a),b) for a,b in zip(delta+[dz],flow)]
        if any(mul(a,b)<0. for a,b in zip(delta,last_delta)):relax=mul(relax,.5)
        last_delta=delta
    deflection=clamp(mul(p['thrust_deflection'],thrust),-p['max_deflection'],p['max_deflection'])
    axial_force=mul(f32(math.sqrt(max(0.,sub(1.,mul(deflection,deflection))))),thrust)
    force=transform(basis,[axial_force,0.,0.])
    direction=1. if p['direction']==0 else -1.
    reaction=0. if p['coaxial'] else mul(add(torque,mul(div(sub(omega,previous),dt),p['inertia'])),direction)
    engine_omega=mul(div(1.,p['reduction']),omega);old_engine=mul(div(1.,p['reduction']),previous)
    normalized=mul(p['inv_max_omega'],engine_omega);normalized=mul(normalized,normalized)
    damp=interval(local[0],*p['damping_speed'])
    moment=transform(basis,[0.,mul(mul(mul(-sign(local_w[1]),damp),normalized),interval(abs(local_w[1]),*p['pitch_damping'])),
                            mul(mul(mul(-sign(local_w[2]),damp),normalized),interval(abs(local_w[2]),*p['yaw_damping']))])
    arm=[sub(x,y) for x,y in zip(r,map(f32,cg))]
    moment=[add(sub(mul(force[1],arm[2]),mul(arm[1],force[2])),moment[0]),
            add(sub(mul(force[2],arm[0]),mul(arm[2],force[0])),moment[1]),
            add(sub(mul(force[0],arm[1]),mul(arm[0],force[1])),moment[2])]
    momentum=0. if p['coaxial'] else mul(mul(mul(omega,p['inertia']),direction),p['momentum_scale'])
    neutral=(f32(math.pi/2) if abs(omega)<f32(1e-5) else
             add(sub(f32(math.atan2(local[0],mul(p['neutral_radius'],omega))),p['mean_twist']),
                 mul(div(p['polar']['base'][8],p['polar']['base'][1]),f32(-.01745329238474369))))
    pitch,gp,reported=governor(p,pitch,gp,engine_omega,old_engine,target,command,auto,afterburner,neutral,dt)
    out=[0.]*40;out[:3]=force;out[18]=torque;out[23:26]=moment
    if torque_gyro or p['torque_gyro_always']:
        out[20:23]=transform(basis,[reaction,0.,0.]);out[26:29]=[mul(momentum,c) for c in basis[:3]]
    a,b,c,d=p['shake']
    shake=interval(transverse,a,0.,b,1.) if transverse<b else 1. if transverse<c else interval(transverse,c,1.,d,0.)
    out[29]=mul(shake,normalized);out[30]=p['inertia'];out[31]=reported
    out[32]=add(flow[1],flow[0]);out[33]=flow[2]
    out[35]=sub(div(axial_force,thrust) if abs(thrust)>f32(4e-19) else 0.,1.)
    return dict(pitch=pitch,governor_pitch=gp,flow=flow,outputs=out,blade_inputs=blade_inputs)
