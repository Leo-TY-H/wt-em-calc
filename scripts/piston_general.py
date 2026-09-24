"""Healthy running piston wrapper with all fixed-wing carburetor families."""
from component_assembly import f32,add,sub,mul
from piston_model import div,inlet_pressure,rpm_torque,mixture,boost_active
from piston_compressor import step as compressor
from engine_supply import mechanical_multiplier


def reservoir_torque(p,omega,throttle,multiplier,reservoir,afterburner,gear,nitro):
    effective=add(mul(sub(1.,p['min_throttle']),min(p['max_throttle'],throttle)),p['min_throttle'])
    x=div(omega,p['max_omega']);shape=sub(mul(x,3.),div(add(mul(x,x),mul(x,x)),effective))
    half=mul(p['reservoir_capacity'],.5)
    if reservoir<=half:fuel=f32(.6)
    elif reservoir>=p['reservoir_capacity']:fuel=1.
    else:fuel=add(f32(.6),div(mul(sub(reservoir,half),sub(1.,f32(.6))),sub(p['reservoir_capacity'],half)))
    tq=mul(mul(mul(mul(p['throttle_boost'] if throttle>1. else 1.,multiplier),p['torque_base']),fuel),shape)
    if boost_active(p,throttle,afterburner,gear,nitro):tq=mul(tq,mul(p['stages'][gear]['boost'],p['afterburner_boost']))
    if tq<0.:tq=max(tq,mul(mul(mul(mul(multiplier,f32(-.8)),omega),p['inverse_omega']),p['torque_base']))
    return tq


def step(p,s,velocity=(100.,0.,0.),height=0.,dt=1/48,seed=12345,torque_multiplier=1.,nitro=0.):
    if p['family'] not in (0,1):raise ValueError('Piston engine required')
    s=dict(s);dt=f32(dt);omega=f32(s['omega']);throttle=min(f32(s.get('throttle',1.)),f32(1.1))
    inlet=inlet_pressure(height,velocity[0],p['ram_recovery'])
    c=compressor(p,omega,throttle,inlet,dt,
        gear=s.get('gear',0) if p['manual_compressor'] and not s.get('automatic_compressor',False) else None,old_gear=s.get('gear',0),
        regulator=s.get('regulator',-1.),afterburner=s.get('afterburner',False),nitro=nitro,
        turbo=s.get('turbo',0.),turbo_command=s.get('turbo_command',1.),automatic_turbo=s.get('automatic_turbo',True),height=height)
    if p['carburetor']==3:
        tq=reservoir_torque(p,omega,throttle,torque_multiplier,s.get('reservoir',p['reservoir_capacity']),s.get('afterburner',False),c['gear'],nitro)
    elif p['carburetor'] in (1,2):tq=rpm_torque(p,omega,throttle,torque_multiplier,s.get('afterburner',False),c['gear'],nitro)
    else:raise ValueError('Unknown running piston carburetor')
    tq=mul(tq,c['multiplier'])
    mix=mixture(p,inlet,s.get('mixture',.5),rich_accumulator=s.get('extra_amplitude',0.),automatic=s.get('automatic_mixture',False))
    if mix['requires_stop']:raise ValueError('Running branch requires adequate mixture')
    tq=mul(mix['multiplier'],tq)
    modulation,mechanical,seed=mechanical_multiplier(p,omega,1.,p['cylinders'],s.get('mechanical',1.),mix['rich_accumulator'],tq,s.get('friction',0.),dt,seed)
    tq=mul(modulation,tq)
    power=max(mul(p['base_hp'],f32(.05)),mul(mul(tq,f32(.00134048261602968)),omega))
    x=max(0.,div(power,p['base_hp']));y=p['consumption']
    if x<=.5:rate=y[0] if x<=0 else y[1] if x>=.5 else add(y[0],mul(x,add(sub(y[1],y[0]),sub(y[1],y[0]))))
    elif x<=1.:rate=y[2] if x>=1 else add(mul(add(add(x,x),-1.),sub(y[2],y[1])),y[1])
    else:rate=y[3] if x>=f32(1.1) else add(mul(add(mul(x,f32(9.999998092651367)),f32(-9.999998092651367)),sub(y[3],y[2])),y[2])
    consumption=mul(mul(power,f32(.00027777778450399637)),rate)
    manifold=c['manifold']
    if p['compressor_type']==1 and p['mixer_type']==1:
        manifold=mul(mixture(p,inlet,s.get('mixture',.5),automatic=s.get('automatic_mixture',False))['multiplier'],manifold)
    result=dict(torque=tq,friction=0.,regulator=c['regulator'],gear=c['gear'],throttle_ratio=c['throttle_ratio'],
        potential_manifold=c['potential_manifold'],manifold=manifold,mechanical=mechanical,seed=seed,
        extra_amplitude=mix['rich_accumulator'],consumption=consumption,effective_throttle=throttle,
        force=[0.,0.,0.],moment=[0.,0.,0.],running=7)
    if p['compressor_type']==3:result.update(turbo=c['turbo'],turbo_command=c['turbo_command'])
    return result
