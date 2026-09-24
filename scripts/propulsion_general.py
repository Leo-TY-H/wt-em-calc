"""Intact connected fixed-wing drivetrain and owner aggregation.

Runtime equations use portable prepared properties. All retained engine, shaft,
governor and wake states are explicit. Thermal damage/fuel depletion are frozen.
Unsupported engine consumers raise rather than silently omit their contribution.
"""
from component_assembly import f32,add,sub,mul
from functools import lru_cache
from piston_model import div,boost_active
from piston_general import step as piston_step
from propeller_general import step as propeller_step
from structural_limits import interval
from air_state import cache,speed_of_sound
from turbine_general import step as turbine_step
from rocket_general import step as rocket_step


def delivered_boost(p,s,nitro=0.):
    # Target/governor selection precedes this step's effective-throttle update.
    return boost_active(p,s.get('effective_throttle',0.),s.get('afterburner',False),s.get('gear',0),nitro)


def target_omega(p,s,nitro=0.):
    throttle=f32(s.get('throttle',1.))
    if p['boost_type']>0 and p['boost_controllable']:
        throttle=f32(1.1) if delivered_boost(p,s,nitro) else min(throttle,1.)
    rows=p['rpm_targets']
    if not rows:return 0.
    if throttle<=rows[0][0]:return rows[0][1]
    for lo,hi in zip(rows,rows[1:]):
        if throttle<hi[0]:return interval(throttle,*lo,*hi)
    return rows[-1][1]


def sum_vector(a,b):return [add(x,y) for x,y in zip(a,b)]


@lru_cache(maxsize=512)
def fixed_air(velocity,height):
    return cache(velocity,height),speed_of_sound(height)


def transmission_step(t,engines,props,state,engine_states,prop_states,velocity,height,body_omega,cg,dt,seed,nitro,torque_gyro):
    omega=f32(state['omega']);previous=f32(state.get('previous_omega',omega))
    target=0.;boost=False
    for link in t['engines']:
        i=link['index'];p=engines[i]['properties'];s=engine_states[i]
        target=max(target,target_omega(p,s,nitro))
        boost=boost or (p['boost_controllable'] and delivered_boost(p,s,nitro))
    air,sound=fixed_air(tuple(velocity),height)
    force=[0.]*3;moment=[0.]*3;reaction=[0.]*3;momentum=[0.]*3
    load=prop_inertia=prop_friction=0.;wash=0.;swirl=[0.,0.]
    new_props={};new_engines={}
    for link in t['propellers']:
        i=link['index'];p=props[i]['properties'];s=prop_states[i];ratio=link['ratio']
        r=propeller_step(p,s,velocity=velocity,body_omega=body_omega,cg=cg,
            omega=mul(omega,ratio),previous_omega=mul(previous,ratio),target_omega=target,
            command=s.get('command',1.),auto=s.get('auto',False),density=air['density'],sound_speed=sound,
            dt=dt,afterburner=boost,torque_gyro=torque_gyro)
        new_props[i]=dict(s,**r);o=r['outputs']
        force=sum_vector(force,o[:3]);moment=sum_vector(moment,o[23:26]);momentum=sum_vector(momentum,o[26:29])
        reaction=sum_vector(reaction,o[20:23] if t['correct_link'] else [mul(ratio,x) for x in o[20:23]])
        load=add(load,mul(o[18],ratio));prop_friction=add(prop_friction,mul(o[19],ratio))
        inertia=o[30]
        if t['auto_inertia']:inertia=mul(inertia,mul(ratio,ratio) if t['correct_link'] else ratio)
        prop_inertia=add(prop_inertia,inertia)
        axial=mul(o[32],p['basis'][0])
        if abs(axial)>abs(wash):wash=axial
        direction=0 if p['direction']==0 else 1
        swirl[direction]=max(swirl[direction],mul(o[33],p['basis'][0]))
    moment=sum_vector(moment,reaction)
    torque=engine_inertia=friction=0.;limit=f32(2147440000.)
    engine_force=[0.]*3;engine_moment=[0.]*3
    for link in t['engines']:
        i=link['index'];p=engines[i]['properties'];ratio=link['ratio']
        s=dict(engine_states[i],omega=mul(omega,ratio),extra_amplitude=0.)
        if p['family'] in (0,1):r=piston_step(p,s,velocity,height,dt,seed,nitro=nitro)
        elif p['family']==5:r=turbine_step(engines[i],s,velocity,height,dt,seed,nitro=nitro,cg=cg)
        else:raise ValueError('Unexpected connected engine family: '+str(p['family']))
        seed=r['seed']
        new_engines[i]=dict(s,**r,elapsed=add(s.get('elapsed',0.),dt))
        torque=add(torque,mul(r['torque'],ratio));friction=sub(friction,mul(r['friction'],ratio))
        # Original 101a118d6 replaces (does not sum) the engine inertia register.
        engine_inertia=mul(mul(ratio,ratio) if t['correct_link'] else ratio,p['engine_inertia'])
        limit=min(limit,mul(link['inverse_ratio'],p['omega_limit']))
        engine_force=sum_vector(engine_force,r['force']);engine_moment=sum_vector(engine_moment,[mul(ratio,x) for x in r['moment']])
    force=sum_vector(force,engine_force);moment=sum_vector(moment,engine_moment)
    inertia=add(engine_inertia,prop_inertia);inverse=div(1.,inertia) if abs(inertia)>f32(4e-19) else 0.
    accel=mul(sub(torque,load),inverse)
    if not t['auto_inertia']:accel=mul(accel,t['acceleration'])
    proposed=add(mul(accel,dt),omega)
    drag=mul(add(prop_friction,mul(inverse,friction)),dt)
    proposed=add(proposed,min(-min(drag,proposed),drag))
    next_omega=min(max(proposed,0.),add(limit,limit))
    outputs=force+moment+momentum+[wash,*swirl,0.]
    return dict(omega=next_omega,previous_omega=omega,outputs=outputs,engines=new_engines,propellers=new_props,seed=seed)


def step(config,state,velocity=(100.,0.,0.),height=0.,body_omega=(0.,0.,0.),cg=(0.,0.,0.),dt=1/48,seed=12345,nitro=0.,torque_gyro=True):
    for key in ['engines','propellers','transmissions']:
        if len(state[key])!=len(config[key]):raise ValueError('Incomplete '+key+' state')
    dt=f32(dt);es=[dict(s) for s in state['engines']];ps=[dict(s) for s in state['propellers']]
    force=[0.]*3;moment=[0.]*3;momentum=[0.]*3;wash=0.;swirl=[0.,0.]
    ts=[];linked=set();per_engine_force=[[0.]*3 for _ in es]
    for t,s in zip(config['transmissions'],state['transmissions']):
        r=transmission_step(t,config['engines'],config['propellers'],s,es,ps,velocity,height,body_omega,cg,dt,seed,nitro,torque_gyro)
        seed=r['seed'];o=r['outputs'];ts.append({k:r[k] for k in ['omega','previous_omega','outputs']})
        force=[a+b for a,b in zip(force,o[:3])];moment=[a+b for a,b in zip(moment,o[3:6])];momentum=[a+b for a,b in zip(momentum,o[6:9])]
        if abs(o[9])>abs(wash):wash=o[9]
        swirl=[max(a,b) for a,b in zip(swirl,o[10:12])]
        for link in t['engines']:
            i=link['index'];linked.add(i);per_engine_force[i]=sum_vector(per_engine_force[i],o[:3])
        for i,v in r['engines'].items():es[i]=v
        for i,v in r['propellers'].items():ps[i]=v
    for i,engine in enumerate(config['engines']):
        if i in linked:continue
        family=engine['family'];s=es[i]
        if family in (2,5):r=turbine_step(engine,s,velocity,height,dt,seed,nitro=nitro,cg=cg)
        elif family==3:r=rocket_step(engine,s,velocity,height,dt,seed,nitro=nitro,cg=cg)
        else:raise ValueError('Unexpected unconnected engine family: '+str(family))
        seed=r['seed'];es[i]=dict(s,**r,elapsed=add(s.get('elapsed',0.),dt))
        force=[a+b for a,b in zip(force,r['force'])];moment=[a+b for a,b in zip(moment,r['moment'])]
    return dict(transmissions=ts,engines=es,propellers=ps,seed=seed,aggregate_force=force,aggregate_moment=moment,
        engine_angular_momentum=momentum,engine_wash=[wash,sub(*swirl)],per_engine_force=per_engine_force)
