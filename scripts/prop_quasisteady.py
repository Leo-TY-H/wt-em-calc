import math
from collections import OrderedDict
from functools import lru_cache

from polar_model import make_polar, calc_cl, calc_cd
from polar_runtime import CONST_KEYS
from propeller_general import local_flow
from propulsion_general import fixed_air
from piston_general import step as piston_step
from piston_compressor import step as compressor
from piston_model import inlet_pressure
from turbine_general import step as turbine_step
from rocket_general import step as rocket_step


REVISION = 'instant-governor-steady-v2'
_polars = OrderedDict()


def polar_properties(runtime):
    key = id(runtime)
    if key not in _polars:
        if len(_polars) >= 128:_polars.popitem(last=False)
        props = dict(zip(CONST_KEYS, runtime['base'][1:14]))


        props.update(OswaldsEfficiencyNumber=runtime['base'][0],
                     MachFactor=runtime['mode'], CombinedCl=runtime.get('combined', True))
        for i, row in enumerate(runtime.get('mach', ()), 1):
            props.update({k+str(i):v for k,v in zip(
                ('MachCrit','MachMax','MultMachMax','MultLineCoeff','MultLimit'), row[:5])})
        for i, (mach, _, values) in enumerate(runtime.get('cm', ())):
            props['ClToCmByMach'+str(i)] = [mach, *values]
        _polars[key] = runtime, props
    _polars.move_to_end(key)
    return _polars[key][1]


@lru_cache(maxsize=1)
def legacy_stall_shape():
    props = dict(zip(CONST_KEYS, (.075,3.,.02222222089767456,36.,.01,-1.09,1.09,
                                  .4,17.,-17.,1.4,-.9,.012)))
    props.update(OswaldsEfficiencyNumber=55.5, MachFactor=0)
    polar = make_polar(props, 1., 1.)
    return {k:polar[k] for k in ('aoaLineH','aoaLineL','parabCyCoeffH','parabCyCoeffL')}


def blade_polar(runtime, mach):
    polar = make_polar(polar_properties(runtime), 1., 1., mach)
    if runtime['mode'] == 2:polar.update(legacy_stall_shape())
    return polar


def blade_forces(p, omega, pitch, axial, transverse, density, sound, angular_flow=0.):
    thrust = torque = 0.
    area_unit = p['radius']**2 * .2
    for station, twist, width in zip((.35,.55,.75,.95), p['twist'], p['width']):
        radius = station*p['radius']
        tangential = radius*(omega-angular_flow)
        phi = math.atan2(axial,tangential) if abs(omega)>=1e-5 else math.pi/2
        alpha = math.degrees(pitch-phi+twist)
        speed = math.hypot(tangential+transverse,axial)
        if speed == 0.:continue
        polar = blade_polar(p['polar'],speed/sound)
        force_unit = speed*speed*.5*density*width*area_unit
        lift = calc_cl(polar,alpha)*force_unit*polar['clKq']
        drag = calc_cd(polar,alpha)*force_unit*polar['kq']
        thrust += (lift*tangential-drag*axial)/speed
        torque += (lift*axial+drag*tangential)/speed*radius
    return dict(thrust=thrust*p['blades'],torque=torque*p['blades'])


def clamp(x, lo, hi):return min(max(x,lo),hi)
def sign(x):return 1. if x>0. else -1. if x<0. else 0.
def transform(b, v):return [sum(v[j]*b[i+3*j] for j in range(3)) for i in range(3)]


def table(rows, x):
    if not rows:return [0.,0.,0.]
    if x<=rows[0][0]:return rows[0][2:]
    for lo,hi in zip(rows,rows[1:]):
        if x<=hi[0]:
            t=(x-lo[0])/(hi[0]-lo[0])
            return [a+(b-a)*t for a,b in zip(lo[2:],hi[2:])]
    return rows[-1][2:]


def interval(x, x0, y0, x1, y1):
    if x<=x0:return y0
    if x>=x1:return y1
    return y0+(y1-y0)*(x-x0)/(x1-x0)


def propeller(p, state, velocity, body_omega, cg, omega, density, sound, torque_gyro):
    if p['cyclic'] or p['differential_pitch'] or p['active_pitch_2d'] or p['pitch_1d_count']:
        raise ValueError('Unsupported cyclic or scheduled-pitch geometry')
    local,local_w,transverse_sq,transverse = local_flow(
        tuple(p['basis']),tuple(p['position']),tuple(velocity),tuple(body_omega))
    flow=list(state['flow']);pitch=state['pitch']
    axial=local[0]+.5*(flow[0]+flow[1])
    first=blade_forces(p,omega,pitch,axial,transverse,density,sound,.25*flow[2])
    thrust=first['thrust'];torque=first['torque']
    if p['coaxial']:
        second=blade_forces(p,omega,pitch,axial,transverse,density,sound,-.25*flow[2])
        thrust+=second['thrust'];torque+=second['torque']
    disc=math.pi*.25*p['diameter']**2
    airflow=table(p['airflow'],local[0]);ambient=local[0]+airflow[0]
    def induced(thrust, upstream):
        value=2*thrust/(density*disc)+sign(upstream)*upstream**2

        return clamp(sign(thrust)*airflow[2]*(sign(value)*math.sqrt(abs(value))-upstream),-150.,150.)
    x=induced(first['thrust'],ambient)
    if p['coaxial']:target=[x,induced(thrust,ambient+.5*x),0.]
    else:
        flux=math.sqrt((local[0]+.5*x)**2+transverse_sq)*disc*density
        divisor=(p['diameter']*.5)**2*flux*.5
        target=[x,0.,clamp(torque/divisor if abs(divisor)>4e-19 else 0.,0.,20.)]
    deflection=clamp(p['thrust_deflection']*thrust,-p['max_deflection'],p['max_deflection'])
    axial_force=math.sqrt(max(0.,1.-deflection**2))*thrust
    force=transform(p['basis'],[axial_force,0.,0.])
    direction=1. if p['direction']==0 else -1.
    reaction=0. if p['coaxial'] else torque*direction
    normalized=(omega/p['reduction']*p['inv_max_omega'])**2
    damp=interval(local[0],*p['damping_speed'])
    moment=transform(p['basis'],[0.,-sign(local_w[1])*damp*normalized*interval(abs(local_w[1]),*p['pitch_damping']),
                                   -sign(local_w[2])*damp*normalized*interval(abs(local_w[2]),*p['yaw_damping'])])
    arm=[a-b for a,b in zip(p['position'],cg)]
    moment=[moment[0]+force[1]*arm[2]-arm[1]*force[2],
            moment[1]+force[2]*arm[0]-arm[2]*force[0],
            moment[2]+force[0]*arm[1]-arm[0]*force[1]]
    momentum=0. if p['coaxial'] else omega*p['inertia']*direction*p['momentum_scale']
    outputs=[0.]*40;outputs[:3]=force;outputs[18]=torque;outputs[23:26]=moment
    if torque_gyro or p['torque_gyro_always']:
        outputs[20:23]=transform(p['basis'],[reaction,0.,0.])
        outputs[26:29]=[momentum*c for c in p['basis'][:3]]
    a,b,c,d=p['shake']
    outputs[29]=normalized*(interval(transverse,a,0.,b,1.) if transverse<b else
                            1. if transverse<c else interval(transverse,c,1.,d,0.))
    outputs[30]=p['inertia'];outputs[31]=state.get('command',1.)
    outputs[32]=flow[0]+flow[1];outputs[33]=flow[2]
    outputs[35]=axial_force/thrust-1. if abs(thrust)>4e-19 else 0.
    return dict(state,pitch=pitch,governor_pitch=pitch,flow=flow,equilibrium_flow=target,outputs=outputs)


def engine_output(engine, state, velocity, height, nitro, cg):
    p=engine['properties'];s=dict(state,regulator=-1.,extra_amplitude=0.)
    if engine['family'] in (0,1):
        if p['compressor_type']==3:
            c=compressor(p,s['omega'],s['throttle'],inlet_pressure(height,velocity[0],p['ram_recovery']),
                         0.,gear=None,afterburner=s['afterburner'],nitro=nitro,assisted=True,height=height)
            s['turbo']=c['turbo']
        result=piston_step(p,s,velocity,height,0.,12345,nitro=nitro)
    elif engine['family'] in (2,5):
        result=turbine_step(engine,s,velocity,height,0.,12345,nitro=nitro,cg=cg)
    elif engine['family']==3:result=rocket_step(engine,s,velocity,height,0.,12345,nitro=nitro,cg=cg)
    else:raise ValueError('Unsupported steady engine family')
    if result['seed']!=12345:raise ValueError('Steady engine requires nominal deterministic torque')
    return dict(s,**result)


def frame(p, state, velocity, height, body_omega, cg, dt, seed, nitro, torque_gyro=True):
    engines=[dict(e) for e in state['engines']];props=[dict(q) for q in state['propellers']]
    air,sound=fixed_air(tuple(velocity),height)
    linked=set();transmissions=[];force=[0.]*3;moment=[0.]*3;momentum=[0.]*3;wash=0.;swirl=[0.,0.]
    per_engine=[[0.]*3 for _ in engines]
    def add_vectors(a,b):return [x+y for x,y in zip(a,b)]
    for t,shaft in zip(p['transmissions'],state['transmissions']):
        tf=[0.]*3;tm=[0.]*3;th=[0.]*3;tw=0.;ts=[0.,0.]
        for link in t['propellers']:
            i=link['index'];pp=p['propellers'][i]['properties']
            q=propeller(pp,props[i],velocity,body_omega,cg,shaft['omega']*link['ratio'],air['density'],sound,torque_gyro)
            props[i]=q;o=q['outputs'];tf=add_vectors(tf,o[:3]);tm=add_vectors(tm,o[23:26]);th=add_vectors(th,o[26:29])
            tm=add_vectors(tm,[v*(1. if t['correct_link'] else link['ratio']) for v in o[20:23]])
            axial=o[32]*pp['basis'][0]
            if abs(axial)>abs(tw):tw=axial
            direction=0 if pp['direction']==0 else 1
            ts[direction]=max(ts[direction],o[33]*pp['basis'][0])
        for link in t['engines']:
            i=link['index'];linked.add(i)
            engines[i]=engine_output(p['engines'][i],dict(engines[i],omega=shaft['omega']*link['ratio']),velocity,height,nitro,cg)
            tf=add_vectors(tf,engines[i]['force']);tm=add_vectors(tm,[link['ratio']*v for v in engines[i]['moment']])
        transmissions.append(dict(shaft,previous_omega=shaft['omega'],outputs=tf+tm+th+[tw,*ts,0.]))
        force=add_vectors(force,tf);moment=add_vectors(moment,tm);momentum=add_vectors(momentum,th)
        if abs(tw)>abs(wash):wash=tw
        swirl=[max(a,b) for a,b in zip(swirl,ts)]
        for link in t['engines']:per_engine[link['index']]=add_vectors(per_engine[link['index']],tf)
    for i,e in enumerate(p['engines']):
        if i not in linked:
            engines[i]=engine_output(e,engines[i],velocity,height,nitro,cg)
            force=add_vectors(force,engines[i]['force']);moment=add_vectors(moment,engines[i]['moment'])
    return dict(transmissions=transmissions,engines=engines,propellers=props,seed=seed,
                aggregate_force=force,aggregate_moment=moment,engine_angular_momentum=momentum,
                engine_wash=[wash,swirl[0]-swirl[1]],per_engine_force=per_engine)
