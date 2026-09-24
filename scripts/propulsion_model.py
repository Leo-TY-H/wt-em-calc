"""Selected intact propeller drivetrain, independently reconstructed.

One inline engine and one connected propeller. Original update order and legacy
transmission inertia semantics are retained. No thermal damage or fuel burn.
"""
from component_assembly import f32,add,mul,sub
from piston_model import div,source_blocks,boost_active
from piston_wrapper import properties as engine_properties,step as engine_step
from propeller_model import properties as prop_properties
from propeller_step import step as prop_step
from air_state import cache,speed_of_sound
from structural_limits import interval


def prepare(fm):
    pp=prop_properties(fm);e,g=source_blocks(fm);modern='Transmission0' in fm
    t=fm['Transmission0'] if modern else g
    return dict(prop=pp,engine=engine_properties(fm),inverse_reduction=div(1.,pp['reduction']),
                auto_inertia=t.get('UseAutoPropInertia',False),correct_link=t.get('CorrectPropellerToTransmissionLink',False) if modern else False,
                acceleration=f32(t.get('EngineAcceleration',1.) if modern else e['Main'].get('EngineAcceleration',1.)))


def target_omega(p,s):
    throttle=f32(s.get('throttle',1.))
    if p['boost_type']>0 and p['boost_controllable']:
        throttle=f32(1.1) if boost_active(p,throttle,s.get('afterburner',False),s.get('gear',0)) else min(throttle,1.)
    table=p['rpm_targets']
    if throttle<=table[0][0]:return table[0][1]
    for lo,hi in zip(table,table[1:]):
        if throttle<hi[0]:return interval(throttle,*lo,*hi)
    return table[-1][1]


def step(p,s,velocity=(100.,0.,0.),height=0.,body_omega=(0.,0.,0.),cg=(0.,0.,0.),dt=1/60,seed=12345,torque_gyro=True):
    dt=f32(dt);omega=f32(s['omega']);previous=f32(s.get('previous_omega',omega));ratio=p['prop']['reduction']
    es=dict(s.get('engine',{}),omega=omega,extra_amplitude=0.)
    air=cache(velocity,height)
    prop=prop_step(p['prop'],s.get('prop',{}),velocity=velocity,body_omega=body_omega,cg=cg,omega=mul(omega,ratio),previous_omega=mul(previous,ratio),
                   target_omega=target_omega(p['engine'],es),command=s.get('command',1.),auto=s.get('auto',False),density=air['density'],sound_speed=speed_of_sound(height),
                   dt=dt,afterburner=p['engine']['boost_controllable'] and boost_active(p['engine'],es.get('throttle',1.),es.get('afterburner',False),es.get('gear',0)),torque_gyro=torque_gyro)
    engine=engine_step(p['engine'],es,velocity,height,dt,seed)
    inertia=p['prop']['inertia']
    if p['auto_inertia']:inertia=mul(inertia,mul(ratio,ratio) if p['correct_link'] else ratio)
    inverse=div(1.,add(p['engine']['engine_inertia'],inertia))
    acceleration=mul(sub(engine['torque'],mul(prop['outputs'][18],ratio)),inverse)
    if not p['auto_inertia']:acceleration=mul(acceleration,p['acceleration'])
    new_omega=max(0.,min(add(mul(acceleration,dt),omega),add(p['engine']['omega_limit'],p['engine']['omega_limit'])))
    out=prop['outputs'][:3]+list(prop['outputs'][23:26])+prop['outputs'][26:29]+[prop['outputs'][32],0.,0.,0.]
    reaction=prop['outputs'][20]
    if not p['correct_link']:reaction=mul(ratio,reaction)
    out[3]=add(out[3],reaction);out[10 if p['prop']['direction']==0 else 11]=prop['outputs'][33]
    return dict(omega=new_omega,previous_omega=omega,outputs=out,engine=engine,prop=prop,elapsed=dt)
