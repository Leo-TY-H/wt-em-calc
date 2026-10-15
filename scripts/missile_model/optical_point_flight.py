"""Independent pure-air optical flight composition for pinned client 2.59.0.34.

Research comparison boundary: supplied released body/guidance state, immutable
properties, fixed caller interval and already-enumerated clear point targets.
No aircraft provider, collisions, network corrections or geometric-only policy.
"""
from copy import deepcopy
import math
from kernels import f32,add,sub,mul,div
from motor_vector import propulsion_adapter
from aero_vectors import forces
from body_integration import integrate
from optical_point_controls import update as guidance_update


def packet(body):
    return body['position']+body['quaternion']+body['velocity']+body['omega']+[body['time']]


def cap_velocity(velocity,end_speed):
    v=list(map(f32,velocity));c=f32(end_speed)
    squared=add(add(mul(v[0],v[0]),mul(v[1],v[1])),mul(v[2],v[2]))
    if abs(c)>=2**-23 and squared>mul(c,c):
        factor=div(c,f32(math.sqrt(squared)))
        v=[mul(x,factor) for x in v]
    return v


def update(p,state,targets,dt,*,guidance_step=guidance_update):
    """Advance body first, evaluate guidance, store next controls, then cap speed.

    state: body, guidance, controls (fins2/orientation2/factors4).
    p: rocket, body properties, guidance properties, precision, end_speed.
    Optional state.launch_time defaults to zero for older snapshots. Body time
    remains in the caller's clock; guidance age is its binary32 difference from
    launch_time. Flat dry environment, zero wind and optional
    environment terms; native gravity enabled. Primary no-provider guidance.
    Optional p.matrix_velocity_frame selects the controller's game setting;
    omitted values retain the earlier matrix=true evidence convention.
    """
    dt=f32(dt)
    if not math.isfinite(dt) or dt<=0.:raise ValueError('Positive finite caller interval required')
    if p['precision']<=0:raise ValueError('Positive body substep count required')
    matrix_velocity_frame=p.get('matrix_velocity_frame',True)
    if type(matrix_velocity_frame) is not bool:raise ValueError('matrix_velocity_frame must be an explicit boolean')
    s=deepcopy(state);b=s['body'];old=packet(b);controls=s['controls']
    if b['water'] or b['immersion']!=0.:raise NotImplementedError('Pure-air composition only')
    target_time=add(b['time'],dt)
    # 14142ac75..acec passes rounded target time minus stored body time to
    # propulsion. Body substeps separately use the caller's nominal interval.
    motor_dt=sub(target_time,b['time'])
    propulsion=propulsion_adapter(p['rocket'],length=p['body']['length'],clocks=b['clocks'],
        factors=controls[4:],dt=motor_dt,quaternion=b['quaternion'],height=b['position'][1],
        fins=controls[:2],orientation=controls[2:4],seed=b['seed'],
        _motor_properties=p.get('_motor_properties'))
    b['seed']=propulsion['seed'];sub_dt=div(dt,p['precision']);substeps=[]
    # 14143b721..737 computes the flat-surface depth before body substeps.
    b['surface_depth']=max(sub(0.,b['position'][1]),f32(-2147440000.))
    for _ in range(p['precision']):
        sub_time=add(b['time'],sub_dt)
        force=forces(p['body'],b['position'][1],b['velocity'],b['quaternion'],b['omega'],
            fins=controls[:2],dt=sub_dt,force=propulsion['world_force'],torque=propulsion['body_torque'],
            mass_lost=propulsion['mass_lost'],body_random=b['body_random'])
        integrated=integrate(b,force['acceleration'],force['angular_acceleration'],sub_dt,
            sub_time,propulsion['clock_rates'])
        b.update(integrated['state'])
        b['velocity']=[min(2000.,max(-2000.,v)) for v in b['velocity']]
        b['omega']=[min(15.,max(-15.,v)) for v in b['omega']]
        substeps.append(dict(state=deepcopy(b),acceleration=force['acceleration'],
            angular_acceleration=force['angular_acceleration']))
    remaining=sub(target_time,b['time'])
    b['clocks']=[add(c,mul(rate,remaining)) for c,rate in zip(b['clocks'],propulsion['clock_rates'])]
    b['time']=target_time;pre_cap=packet(b)
    elapsed=sub(target_time,s.get('launch_time',0.))
    guidance=guidance_step(p['guidance'],s['guidance'],old,pre_cap,elapsed,b['clocks'],targets,
        context_value=b['surface_depth'],matrix_velocity_frame=matrix_velocity_frame)
    s['guidance']=guidance['state'];manager=s['guidance']['manager']
    s['controls']=manager['fins']+manager['orientation_auxiliary']+manager['factors']
    b['velocity']=cap_velocity(b['velocity'],p['end_speed'])
    return dict(state=s,propulsion=propulsion,substeps=substeps,pre_cap=pre_cap,guidance=guidance)
