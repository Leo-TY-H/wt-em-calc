"""Intact running piston wrapper 1019f3b80; fuel and health prescribed."""
from component_assembly import f32,add,mul,sub
from piston_model import prepare,source_blocks,div,inlet_pressure,compressor,rpm_torque,mixture
from engine_supply import amplitude_coefficients,mechanical_multiplier,fuel_properties


def properties(fm):
    p=prepare(fm);e,g=source_blocks(fm);main=e['Main']
    p.update(amplitude=amplitude_coefficients(main['RPMAmplitude0'],main['RPMAmplitude1']),
             cylinders=main['Cylinders'],omega_limit=mul(f32(main['RPMMaxAllowed']),f32(.10471975803375244)),
             engine_inertia=f32(main.get('EngineInertiaMoment',1.)),
             consumption=[f32(main.get('FuelConsumptionOn'+key,.25)) for key in ['Idle','HalfThr','FullThr','WEP']],
             manual_compressor=e['Compressor'].get('IsControllable',False),fuel=fuel_properties(fm['Mass']))
    rpm_source=main if 'ThrottleRPMAuto0' in main else g
    p['rpm_targets']=[[f32(rpm_source['ThrottleRPMAuto'+str(i)][0]),mul(f32(rpm_source['ThrottleRPMAuto'+str(i)][1]),f32(.10471975803375244))] for i in range(5) if 'ThrottleRPMAuto'+str(i) in rpm_source]
    p['boost_controllable']=e['Afterburner'].get('IsControllable',False)
    return p


def step(p,s,velocity=(100.,0.,0.),height=0.,dt=1/60,seed=12345,torque_multiplier=1.):
    s=dict(s);dt=f32(dt);throttle=min(f32(s.get('throttle',1.)),f32(1.1))
    inlet=inlet_pressure(height,velocity[0],p['ram_recovery'])
    c=compressor(p,s['omega'],throttle,inlet,dt,gear=s.get('gear',0) if p['manual_compressor'] else None,
                 old_gear=s.get('gear',0),regulator=s.get('regulator',-1.),afterburner=s.get('afterburner',False))
    tq=mul(rpm_torque(p,s['omega'],throttle,torque_multiplier,s.get('afterburner',False),c['gear']),c['multiplier'])
    mix=mixture(p,inlet,s.get('mixture',.5),rich_accumulator=s.get('extra_amplitude',0.))
    if mix['requires_stop']:raise ValueError('Running branch requires adequate mixture')
    tq=mul(mix['multiplier'],tq)
    modulation,mechanical,seed=mechanical_multiplier(p,s['omega'],1.,p['cylinders'],s.get('mechanical',1.),mix['rich_accumulator'],tq,s.get('friction',0.),dt,seed)
    tq=mul(modulation,tq)
    power=max(mul(p['base_hp'],f32(.05)),mul(mul(tq,f32(.00134048261602968)),s['omega']))
    x=max(0.,div(power,p['base_hp']));y=p['consumption']
    if x<=.5:rate=y[0] if x<=0 else y[1] if x>=.5 else add(y[0],mul(x,add(sub(y[1],y[0]),sub(y[1],y[0]))))
    elif x<=1.:rate=y[2] if x>=1 else add(mul(add(add(x,x),-1.),sub(y[2],y[1])),y[1])
    else:rate=y[3] if x>=f32(1.1) else add(mul(add(mul(x,f32(9.999998092651367)),f32(-9.999998092651367)),sub(y[3],y[2])),y[2])
    consumption=mul(mul(power,f32(.00027777778450399637)),rate)
    return dict(torque=tq,friction=0.,regulator=c['regulator'],gear=c['gear'],throttle_ratio=c['throttle_ratio'],
                potential_manifold=c['potential_manifold'],manifold=c['manifold'],mechanical=mechanical,seed=seed,
                extra_amplitude=mix['rich_accumulator'],consumption=consumption,effective_throttle=throttle,force=[0.,0.,0.],moment=[0.,0.,0.],running=7)
