"""Recovered selected-jet fuel supply and mechanical RPM modulation.

Native sites: fuel availability1019948c0, amplitude construction1019ff328,
mechanical multiplier1019f73f0, selected wrapper1019f3b80. These are stateful
upstream inputs to jet_model; thermal damage and fuel-tank updates are separate.
"""
import math
from component_assembly import f32,add,sub,mul
from structural_limits import random_fraction,interval
from control_mixer import prepare_rows,curve,density_at_height
from jet_model import scalar_update,selected_nozzle


def fuel_properties(mass,index=0):
    def field(key,default):return f32(mass.get(key+str(index),mass.get(key,default)))
    return dict(capacity=field('FuelAccumulatorCapacity',0.),
                minimal_load=field('MinimalLoadFactor',-1e6),
                accumulator_flow=field('FuelAccumulatorFlowRate',1e6),
                engine_flow=field('FuelEngineFlowRate',1e6))


def available_fuel(p,system_fuel,accumulator,load_factor,dt):
    """Read-only mass available in this step, including permitted feed-through."""
    system_fuel,accumulator,load_factor,dt=map(f32,(system_fuel,accumulator,load_factor,dt))
    cap=mul(p['engine_flow'],dt);missing=sub(cap,accumulator)
    if missing<0:return cap
    if load_factor<p['minimal_load']:return accumulator
    return add(accumulator,min(missing,min(sub(system_fuel,accumulator),mul(p['accumulator_flow'],dt))))


def amplitude_coefficients(point0,point1):
    """Loader's float32 polynomial, including rounded reciprocal then multiply."""
    x0=mul(f32(point0[0]),f32(.10471975803375244));y0=mul(f32(point0[1]),f32(.01))
    x1=mul(f32(point1[0]),f32(.10471975803375244));y1=mul(f32(point1[1]),f32(.01))
    delta=sub(y1,y0);d=sub(x1,x0);reciprocal=f32(1./mul(d,d))
    return [mul(delta,reciprocal),mul(mul(mul(delta,x0),-2.),reciprocal),
            add(mul(mul(mul(x0,x0),delta),reciprocal),y0)]


def selected_properties(engine_type):
    main=engine_type['Main'];controls=engine_type['Controls']
    if main['Type']!='Jet' or engine_type.get('AutoThrottle',{}).get('HasContorller',False):
        raise ValueError('Requires selected jet without automatic throttle PID')
    if main['CarbueretorType']!=0:raise ValueError('Selected carburetor type required')
    rows=[]
    for i in range(4):
        if 'vtolToThrottleLim'+str(i) in controls:
            x,y=controls['vtolToThrottleLim'+str(i)];rows.append((x,[y]))
    if not rows:
        v=controls.get('vtolToThrottleLim',[0.,1.1,1.,1.1])
        if isinstance(v,(int,float)):rows=[(0.,[v])]
        else:rows=[(v[0],[v[1]]),(v[2],[v[3]])]
    ias=list(map(f32,controls.get('iasToVtolLim',[0.,1.,0.,1.])))
    for i in [0,2]:ias[i]=mul(ias[i],f32(1/3.6))
    # Native inverse maximum omega is 9.549296379/RPMMax, not 1/rounded omega.
    return dict(amplitude=amplitude_coefficients(main['RPMAmplitude0'],main['RPMAmplitude1']),
                inverse_omega=f32(f32(9.549296379089355)/f32(main['RPMMax'])),
                omega_limit=mul(f32(main['RPMMaxAllowed']),f32(.10471975803375244)),
                cylinders=int(main['Cylinders']),throttle_boost=f32(main['ThrottleBoost']),
                ias_vtol=ias,vtol_throttle=prepare_rows(rows),
                reverse_throttle=list(map(f32,controls.get('reverseToThrottleLim',[0.,1.1,1.,1.1]))))


def mechanical_multiplier(p,omega,health,cylinders,previous,extra,torque,friction,dt,seed,
                          disabled=False,enabled=True):
    """Returns (current multiplier, retained multiplier state, updated seed).

    A bypass returns1 without resetting the old state. A small amplitude resets
    the old state to1. Otherwise it is held until an exponentially timed event.
    """
    omega,health,previous,extra,torque,friction,dt=map(f32,(omega,health,previous,extra,torque,friction,dt))
    if add(torque,friction)<0 or disabled or not enabled:return 1.,previous,seed
    rate=f32(3.33)
    if cylinders<3:amplitude=f32(11.25)
    else:
        loss=sub(1.,f32(f32(cylinders)/f32(p['cylinders'])))
        damage=max(mul(min(sub(f32(1.470588207244873),mul(health,f32(1.470588207244873))),1.),3.),
                   mul(mul(loss,loss),f32(9.25)))
        shaft=mul(p['inverse_omega'],omega)
        a,b,c=p['amplitude']
        amplitude=add(add(add(extra,c),mul(omega,add(mul(a,omega),b))),mul(damage,mul(shaft,shaft)))
        if amplitude<=f32(.01):return 1.,1.,seed
        if amplitude<2.:rate=add(mul(f32(-8.335),amplitude),20.)
    seed,u=random_fraction(seed)
    threshold=sub(1.,f32(math.exp(mul(-dt,rate))))
    if u>=threshold:return previous,previous,seed
    seed,u=random_fraction(seed)
    value=add(mul(add(mul(add(u,1.),f32(1.1)),f32(-2.1)),amplitude),1.)
    return value,value,seed


def wrapper_step(p,jet,nozzle,fuel,state,velocity,height,cg,dt,seed,
                 system_fuel,accumulator,load_factor=1.,ias_u=0.,flaps=0.,airbrake=0.,
                 engine_drag_area=0.,fuel_g_effect=True,mechanical_disabled=False,mechanical_enabled=True):
    """1019f3b80 selected path. Owner lifecycle/fuel depletion run afterward.

    State has delivered throttle/AB/auxiliary flags, running enum, health and
    RPM modulation history. Flags here describe delivered engine commands.
    No control/controller availability is invented by this function.
    """
    velocity=list(map(f32,velocity));height,dt=map(f32,(height,dt));s=dict(state)
    s['effective_vtol']=min(interval(ias_u,*p['ias_vtol']),f32(s['vtol']))
    s['effective_throttle']=min(curve(p['vtol_throttle'],s['effective_vtol'],1)[0],
                                interval(s['reverse'],*p['reverse_throttle']),f32(s['throttle']))
    supplied=available_fuel(fuel,system_fuel,accumulator,load_factor if fuel_g_effect else 1.,dt)
    multiplier,s['mechanical'],seed=mechanical_multiplier(p,s['omega'],s['health'],s['cylinders'],
        s['mechanical'],s['extra_amplitude'],s['torque'],s['friction'],dt,seed,mechanical_disabled,mechanical_enabled)
    health=mul(multiplier,f32(s['health'])) if s['running']==7 else 0.
    normalized=min(f32(s['effective_throttle']/f32(1.1 if p['throttle_boost']>1. else 1.)),1.)
    rho=density_at_height(height)
    scalar=scalar_update(jet,rho,velocity[0],s['omega'],normalized,dt,
        afterburner=s['afterburner'],fuel_available=supplied,health=health,
        shaft_fraction=mul(f32(s['omega']),p['inverse_omega']),running=1 if s['running']==6 else 2 if s['running']==7 else 0)
    ias=mul(velocity[0],f32(math.sqrt(f32(rho/f32(1.225)))))
    brake=mul(f32(max(0,min(255,int(mul(f32(airbrake),255.))))),f32(1/255))
    result=selected_nozzle(nozzle,scalar['thrust'],cg,ias,flaps,brake,s['effective_vtol'],s['reverse'])
    s['omega']=min(max(0.,scalar['next_omega']),mul(p['omega_limit'],f32(s['rpm_limit_scale'])))
    s['torque']=s['friction']=0.
    if scalar['active']:
        if s['running']==6:s['inactive_elapsed']=-1.
    elif s['running']==7:
        s['stop_reason']=1 if mul(dt,scalar['consumption'])<f32(system_fuel) else 2
        s['running']=6;s['elapsed']=0.;s['inactive_elapsed']=1.
    speed=f32(math.sqrt(add(mul(velocity[2],velocity[2]),add(mul(velocity[0],velocity[0]),mul(velocity[1],velocity[1])))))
    scale=mul(mul(mul(f32(engine_drag_area),.5),rho),speed)
    result['force']=[sub(x,mul(scale,v)) for x,v in zip(result['force'],velocity)]
    return dict(state=s,seed=seed,force=result['force'],moment=result['moment'],
                consumption=scalar['consumption'],target_omega=scalar['target_omega'],active=scalar['active'])


def healthy_running_owner_step(p,jet,nozzle,fuel,state,velocity,height,cg,dt,seed,
                               system_fuel,accumulator,**options):
    """Selected101a155c0/1019fa1b0 with fixed fuel and intact running engine.

    No propellers/transmissions, engine drag, thermal damage or fuel depletion.
    Consumption remains a diagnostic scalar output; fuel/mass are not changed.
    Ordinary running lifecycle advances elapsed time after the force update.
    Raises on loss of the running state rather than silently omitting a start
    or stop transition. Both selected aircraft have exactly one engine.
    """
    if state['health']!=1. or state['cylinders']!=p['cylinders'] or state['running']!=7:
        raise ValueError('Requires intact running engine and full cylinder count')
    if options.get('engine_drag_area',0.)!=0.:
        raise ValueError('Intact selected engine has zero damage drag')
    state=dict(state,extra_amplitude=0.)  # outer1019fa269 resets before forces
    result=wrapper_step(p,jet,nozzle,fuel,state,velocity,height,cg,dt,seed,system_fuel,accumulator,**options)
    if result['state']['running']!=7:
        raise ValueError('Start/stop lifecycle lies outside the running-engine adapter')
    result['state']['elapsed']=add(f32(result['state']['elapsed']),f32(dt))
    # Actual owner clears these double accumulators before the one-jet loop.
    result.update(aggregate_force=[0.+float(x) for x in result['force']],
                  aggregate_moment=[0.+float(x) for x in result['moment']],
                  engine_angular_momentum=[0.]*3,propeller_force=[0.]*3)
    return result
