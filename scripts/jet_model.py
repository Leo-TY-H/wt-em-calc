"""Recovered scalar jet engine: density/speed grid and RPM-mode interpolation.

Runtime preparation is statically traced to 0x1019eee00. The independent scalar
ports correspond to 0x1019f05d0, 0x1019f0d30 and the scalar part of 0x1019f19e0.
Nozzle vectors and the aircraft's external thrust multiplier are separate stages.
"""
import math
from component_assembly import f32,add,sub,mul
from control_mixer import density_at_height,prepare_rows,curve

G=f32(9.81)


def prepare(main):
    t=main['ThrustMax']
    # 1019eee00 default107d6ff20 -> string106f1c625, "TAS".
    if t.get('VelocityType','TAS')!='TAS':raise ValueError('Jet thrust table requires the configured speed-axis adapter')
    heights=[t[f'Altitude_{i}'] for i in range(sum(k.startswith('Altitude_') for k in t))]
    speeds=[mul(f32(t[f'Velocity_{i}']),f32(1/3.6)) for i in range(sum(k.startswith('Velocity_') for k in t))]
    density=[density_at_height(f32(h)) for h in heights]
    for i in range(1,len(density)):density[i]=min(density[i],sub(density[i-1],f32(.001)))
    for i in range(1,len(speeds)):speeds[i]=max(speeds[i],add(speeds[i-1],f32(.001)))
    cells=[]
    for i in range(len(density)):
        row=[]
        for j in range(len(speeds)):
            thrust=f32(t.get(f'ThrustMaxCoeff_{i}_{j}',1.))
            row.append([thrust,f32(t.get(f'ThrAftMaxCoeff_{i}_{j}',1.)),
                        f32(t.get(f'TorqueMaxCoeff_{i}_{j}',thrust)),f32(t.get(f'TrqAftMaxCoeff_{i}_{j}',1.))])
        cells.append(row)
    modes=[]
    for i in range(16):
        if f'Mode{i}' not in main:break
        x=dict(Throttle=1.,RPM=1.);x.update(main[f'Mode{i}'])
        if not modes or (f32(x['Throttle'])>f32(modes[-1]['Throttle']) and f32(x['RPM'])>f32(modes[-1]['RPM'])):modes.append(x)
    scale=f32(1.1 if main.get('ThrottleBoost',1.)>1. else 1.)
    rpm_rows=prepare_rows([(x['RPM'],[x.get(k,main.get('TorqueZeroOmegaMult',3.) if k=='TorqueMultMinRpm' else 1.)
                         for k in ['ThrustMult','TorqueMultMinRpm','TorqueMultMaxRpm','ConsumptionMult','TurbineTimeConstantMult']]) for x in modes])
    return dict(density=density,speed=speeds,cells=cells,
                bases=[mul(f32(t['ThrustMax0']),G),f32(main.get('AfterburnerBoost',1.)),0.,f32(main.get('AfterburnerBoostTorque',1.))],
                modes=rpm_rows,throttle=prepare_rows([(mul(f32(x['Throttle']),f32(1/scale)),[x['RPM']]) for x in modes]),
                throttle_scale=scale,max_omega=mul(f32(main['RPMMax']),f32(.10471975803375244)),
                consumption=f32(mul(f32(main['ConsumptionOmegaMax']),f32(1/3600))/G),
                tau=max(f32(main['TurbineTimeConstant']),f32(.1)),torque_zero=f32(main.get('TorqueZeroOmegaMult',3.)))


def bracket(grid,value,descending=False):
    if len(grid)==1:return 0,0.
    i=0
    while i+2<len(grid) and (value<grid[i+1] if descending else value>grid[i+1]):i+=1
    weight=min(1.,max(0.,f32(sub(value,grid[i])/sub(grid[i+1],grid[i]))))
    return i,weight


def table(p,density,speed):
    """Four outputs: base thrust N, AB multiplier, base torque, AB torque mult.

    Inputs clamp to the grid edges. The density axis is descending. Actual jet
    caller supplies longitudinal body airspeed, not the 3D velocity magnitude.
    """
    if not p['cells']:return list(p['bases'])
    i,u=bracket(p['density'],f32(density),True);j,v=bracket(p['speed'],f32(speed))
    i1=min(i+1,len(p['density'])-1);j1=min(j+1,len(p['speed'])-1)
    out=[]
    for k,base in enumerate(p['bases']):
        a,b=p['cells'][i][j][k],p['cells'][i][j1][k]
        c,d=p['cells'][i1][j][k],p['cells'][i1][j1][k]
        lo=add(mul(sub(b,a),v),a)
        # Original grouping differs from lerp(lerp(a,b),lerp(c,d)).
        value=add(mul(add(sub(c,lo),mul(sub(d,c),v)),u),lo) if i1!=i else lo
        out.append(mul(value,base))
    return out


def mode(p,rpm_fraction):
    return curve(p['modes'],f32(rpm_fraction),5)


def scalar_update(p,density,speed,omega,throttle,dt,afterburner=False,
                  fuel_available=1e6,health=1.,shaft_fraction=1.,running=2):
    """Original scalar engine state machine with prepared control/health inputs.

    throttle is normalized by the loader's 1.1 WEP scale; omega is radians/sec.
    running 0=off,1=starting,2=running. Finite inputs, positive timestep.
    Fuel availability is mass available this timestep, from the fuel-system
    provider. Health includes upstream engine-state/temperature factors.
    """
    density,speed,omega,throttle,dt,fuel_available,health,shaft_fraction=map(f32,
        (density,speed,omega,throttle,dt,fuel_available,health,shaft_fraction))
    thrust,ab,torque,abtorque=table(p,density,speed)
    target=0.;out_thrust=out_torque=consumption=0.;tau_mult=1.;active=False
    if running and p['modes']:
        target=curve(p['throttle'],throttle,1)[0]
        selected=-1;fraction=0.;candidate_thrust=thrust
        for i in range(len(p['modes'])-1,-1,-1):
            # Each selected mode has a matching throttle knot for these jets.
            # 1019f1c20..1c61: enabled boost always applies to the highest
            # mode, including legacy jets whose final throttle knot is1.0.
            ab_mode=mul(p['throttle_scale'],p['throttle'][i][0])>1. or (afterburner and i==len(p['modes'])-1)
            candidate_thrust=mul(ab if ab_mode else 1.,thrust)
            fraction=f32(fuel_available/max(mul(candidate_thrust,mul(p['consumption'],dt)),f32(1e-6)))
            props=p['modes'][i][2]
            if mul(props[0],props[3])<fraction:selected=i;break
        if selected>=0:
            active=True
            if afterburner:thrust=candidate_thrust;torque=mul(torque,abtorque)
            if selected<len(p['modes'])-1:
                a,b=p['modes'][selected:selected+2]
                x1,x2=mul(a[2][0],a[2][3]),mul(b[2][0],b[2][3]);y1,y2=a[0],b[0]
                if x2<x1:x1,x2,y1,y2=x2,x1,y2,y1
                cap=y1 if fraction<=x1 else y2 if fraction>=x2 else add(y1,f32(mul(sub(fraction,x1),sub(y2,y1))/sub(x2,x1)))
                target=min(target,cap)
            if running==2:
                thrust_mult,tmin,tmax,cons_mult,tau_mult=mode(p,f32(omega/p['max_omega']))
                out_thrust=mul(thrust,thrust_mult)
                out_torque=mul(add(mul(sub(tmax,tmin),shaft_fraction),tmin),torque)
                consumption=mul(mul(cons_mult,out_thrust),p['consumption'])
        else:target=0.
    sign=-1. if health<0 else 1. if health>0 else 0.
    target=mul(mul(mul(target,p['max_omega']),sign),f32(math.sqrt(abs(health))))
    next_omega=add(f32(mul(sub(target,omega),dt)/mul(tau_mult,p['tau'])),omega)
    return dict(thrust=out_thrust,torque=out_torque,consumption=consumption,
                target_omega=target,next_omega=next_omega,active=active)


def steady(p,height,longitudinal_speed,throttle=1.1):
    """Nominal target-RPM evaluation of the fuel-supplied scalar engine.

    This convenience value is not the exact fixed point of rounded RPM steps.
    Full-health mechanical modulation can also remain active at low throttle;
    use engine_supply.wrapper_step for that state/history-dependent behavior.
    The nozzle and whole-aircraft modifier have not yet been applied.
    """
    command=min(1.,f32(f32(throttle)/p['throttle_scale']))
    rpm=curve(p['throttle'],command,1)[0]
    omega=mul(rpm,p['max_omega'])
    return scalar_update(p,density_at_height(f32(height)),f32(longitudinal_speed),omega,command,
                         f32(1/48),afterburner=throttle>1.)


def prepare_nozzle(config):
    """1019ec630 selected, unvectored jet nozzle property adapter.

    Both aircraft have zero direction deflections and zero primary-axis thrust
    modulation. Numbered auxiliary tables use IAS in km/h, converted by loader
    float32(1/3.6). Their values are additive to the unit thrust multiplier.
    This deliberately rejects other aircraft's vectoring configurations.
    """
    if any(config['Direction']):raise ValueError('Only forward selected nozzles')
    for key in ['AileronsToThrustDeflection','ElevatorToThrustDeflection',
                'RudderToThrustDeflection','VtolToThrustDeflection',
                'ReverseToThrustDeflection','AileronsToThrust','ElevatorToThrust','RudderToThrust']:
        if any(config[key]):raise ValueError('Nonzero '+key+' is outside this adapter')
    def rows(key):
        values=[]
        for i in range(8):
            if key+str(i) in config:
                x,*v=config[key+str(i)];values.append((mul(f32(x),f32(1/3.6)),v))
        if not values:values=[(0.,config.get(key,[0.,0.]))]
        return prepare_rows(values)
    return dict(position=list(map(f32,config['Position'])),ratio=f32(config['ThrustRatio']),
                maximum=f32(config['ThrustMax']),flaps=list(map(f32,config['FlapsToThrust'])),
                airbrake=rows('AirbrakeToThrust'),vtol=rows('VtolToThrust'),reverse=rows('ReverseToThrust'))


def selected_nozzle(nozzle,thrust,cg,ias=0.,flaps=0.,airbrake=0.,vtol=0.,reverse=0.):
    """Entire selected-nozzle numerical result of 1019f19e0 after scalar state.

    Inputs are delivered positions. Configured VTOL/reverse controls are absent
    on these aircraft, so their normal-flight delivered values are zero. The
    auxiliary equations are retained for checking explicitly supplied states.
    """
    from body_dynamics import accumulate_nozzle
    thrust,ias,flaps,airbrake,vtol,reverse=map(f32,(thrust,ias,flaps,airbrake,vtol,reverse))
    x0,y0,x1,y1=nozzle['flaps']
    if x1<x0:x0,x1,y0,y1=x1,x0,y1,y0
    blend=y0 if flaps<=x0 else y1 if flaps>=x1 else add(y0,f32(mul(sub(flaps,x0),sub(y1,y0))/sub(x1,x0)))
    capped=min(mul(mul(blend,thrust),nozzle['ratio']),nozzle['maximum'])
    a0,a1=curve(nozzle['airbrake'],ias,2)
    v0,v1=curve(nozzle['vtol'],ias,2)
    r0,r1=curve(nozzle['reverse'],ias,2)
    multiplier=add(add(add(add(add(add(a0,1.),mul(sub(a1,a0),airbrake)),v0),mul(sub(v1,v0),vtol)),r0),mul(sub(r1,r0),reverse))
    force,moment=accumulate_nozzle([1.,0.,0.],nozzle['position'],cg,capped,multiplier)
    return dict(force=force,moment=moment,angles=[0.,0.])
