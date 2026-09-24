"""Healthy supplied auxiliary rocket branch1019f4574, neutral fixed mount."""
from component_assembly import f32,add,sub,mul
from piston_model import div


def step(engine,s,velocity=(100.,0.,0.),height=0.,dt=1/48,seed=12345,nitro=0.,cg=(0.,0.,0.)):
    p=engine['properties'];throttle=min(f32(s.get('throttle',1.)),f32(1.1))
    normalized=min(div(throttle,f32(1.1 if p['throttle_boost']>1. else 1.)),1.)
    omega=add(mul(sub(p['max_omega'],p['shaft_min']),normalized),p['shaft_min'])
    thrust=div(mul(omega,engine['maximum_direct_thrust']),p['max_omega'])
    running=s.get('running',7)
    if running!=7:omega=thrust=normalized=throttle=0.
    force=[mul(thrust,x) for x in engine['axis']];r=[sub(x,f32(y)) for x,y in zip(engine['position'],cg)]
    moment=[sub(mul(force[1],r[2]),mul(force[2],r[1])),sub(mul(force[2],r[0]),mul(force[0],r[2])),sub(mul(force[0],r[1]),mul(force[1],r[0]))]
    return dict(omega=omega,torque=s.get('torque',0.),friction=s.get('friction',0.),regulator=s.get('regulator',-1.),gear=s.get('gear',0),
        throttle_ratio=s.get('throttle_ratio',0.),potential_manifold=s.get('potential_manifold',0.),manifold=s.get('manifold',0.),
        mechanical=f32(s.get('mechanical',1.)),seed=seed,extra_amplitude=0.,consumption=mul(mul(normalized,p['consumption'][3]),f32(s.get('throttle',1.))),
        effective_throttle=throttle,force=force,moment=moment,running=running,turbo=f32(s.get('turbo',0.)),turbo_command=f32(s.get('turbo_command',1.)))
