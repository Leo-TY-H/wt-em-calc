"""Healthy neutral-control type-2/5 turbine wrapper, including both shafts."""
import math
from component_assembly import f32,add,mul
from piston_model import div,boost_active
from engine_supply import mechanical_multiplier
from jet_model import scalar_update
from jet_nozzle import evaluate as nozzle_vectors
from control_mixer import density_at_height


def step(engine,s,velocity=(100.,0.,0.),height=0.,dt=1/48,seed=12345,nitro=0.,cg=(0.,0.,0.)):
    p=engine['properties'];t=engine['turbine'];dt=f32(dt);omega=f32(s['omega'])
    if p['family'] not in (2,5):raise ValueError('Unsupported turbine family')
    throttle=min(f32(s.get('throttle',1.)),f32(1.1));rho=density_at_height(f32(height))
    mod,mechanical,seed=mechanical_multiplier(p,omega,1.,p['cylinders'],s.get('mechanical',1.),0.,s.get('torque',0.),s.get('friction',0.),dt,seed)
    gas=f32(s.get('turbo',0.)) if p['family']==5 else omega
    scalar=scalar_update(t,rho,velocity[0],gas,min(div(throttle,f32(1.1 if p['throttle_boost']>1. else 1.)),1.),dt,
        afterburner=boost_active(p,throttle,s.get('afterburner',False),s.get('gear',0),nitro),health=mod,
        shaft_fraction=mul(omega,p['inverse_omega']),running=2)
    if not scalar['active']:raise ValueError('Running turbine requires adequate fuel')
    ias=mul(f32(velocity[0]),f32(math.sqrt(div(rho,f32(1.225)))))
    vectors=nozzle_vectors(engine['nozzles'],scalar['thrust'],cg,ias=ias)
    result=dict(torque=0.,friction=0.,regulator=s.get('regulator',-1.),gear=s.get('gear',0),
        throttle_ratio=s.get('throttle_ratio',0.),potential_manifold=s.get('potential_manifold',0.),manifold=s.get('manifold',0.),
        mechanical=mechanical,seed=seed,extra_amplitude=0.,consumption=scalar['consumption'],effective_throttle=throttle,
        force=vectors['force'],moment=vectors['moment'],running=7)
    if p['family']==5:
        mod,mechanical,seed=mechanical_multiplier(p,omega,1.,p['cylinders'],mechanical,0.,scalar['torque'],s.get('friction',0.),dt,seed)
        result.update(torque=mul(mod,scalar['torque']),mechanical=mechanical,seed=seed,
            turbo=min(max(scalar['next_omega'],0.),p['turbo_allowed']),turbo_command=s.get('turbo_command',1.))
    else:
        result['omega']=min(max(scalar['next_omega'],0.),p['omega_limit'])
        if p['compressor_type']==3:result.update(turbo=f32(s.get('turbo',0.)),turbo_command=f32(s.get('turbo_command',1.)))
    return result
