"""Selected-jet flap actuator from 0x101a4cbdb..cfd6.

IAS_U is the signed longitudinal indicated airspeed in m/s, FM8468.
Mechanism bounds default to the two jets' unrestricted clean configuration.
Other mechanism configurations must supply bounds from 0x101a4df10.
"""
from component_assembly import f32,add,sub,mul


def ramp(values,x):
    a,b,u,v=map(f32,values);x=f32(x)
    if a>b:a,b,u,v=b,a,v,u
    if x<=a:return u
    if x>=b:return v
    width=sub(b,a)
    d=f32(mul(sub(x,a),sub(v,u))/width) if abs(width)>f32(4e-19) else 0.
    return add(u,d)


def flap_step(fm,requested,actual,mach,ias_u,dt,mechanism_bounds=(0.,1.)):
    """Return next requested/actuated states (FM87dc/FM3a40 respectively).

    Rate is dvFlapsOut while extending and dvFlapsIn while retracting.
    The executable also imposes an independent maximum rate of one per second.
    """
    if not fm['AvailableControls']['hasFlapsControl']:return dict(requested=0.,actual=0.)
    requested,actual,mach,ias_u,dt=map(f32,[requested,actual,mach,ias_u,dt])
    speed=mul(ias_u,f32(3.6))
    limit=min(ramp(fm.get('flapsLimByMach',[.5,.7,1.,1.]),mach),
              ramp(fm.get('flapsLimByIas',[0.,3000.,1.,1.]),speed))
    lo,hi=map(f32,mechanism_bounds)
    if requested<lo or requested>hi:
        axis=fm['Aerodynamics']['FlapsAxis']
        positions=[f32(axis[k]['Flaps']) for k in ['Retracted','Combat','Takeoff','Landing'] if axis[k]['Presents']]
        low=next((p for p in positions if p>=lo),0.)
        high=next((p for p in reversed(positions) if p<=hi),0.)
        requested=low if abs(sub(requested,low))<abs(sub(requested,high)) else high
    requested=min(limit,requested)
    rate=ramp(fm['dvFlapsOut' if requested>actual else 'dvFlapsIn'],speed)
    step=mul(rate,dt)
    moved=min(add(actual,step),requested) if requested>actual else max(sub(actual,step),requested)
    moved=min(max(sub(actual,dt),moved),add(actual,dt))
    return dict(requested=requested,actual=min(1.,max(0.,moved)))


def flap_mechanism_bounds(fm,mach,ias_u,sweep=0.):
    """Intact airborne flap range, original helper 101a4df10 (mechanism 2).

    Gear, airbrake, bay door, chute and VTOL are retracted. Sweep is held fixed.
    Ground and damage ranges are outside this settled airborne slice.
    """
    p=fm.get('AvailableControls',{}).get('flapsLimits',{})
    low,high=0.,1.
    if f32(mach)>f32(p.get('mechLockMachNumber',100.)) or mul(f32(ias_u),f32(3.6))>f32(p.get('mechLockIas',2147440000.)):
        high=0.
    if p.get('secondaryMech') and f32(p.get('forcedSecondaryMechValue',-1.))<0.:
        secondary=f32(sweep) if p['secondaryMech']=='sweep' else 0.
        lo,hi=map(f32,p.get('secondaryMechRange',[0.,1.]))
        if not lo<=secondary<=hi:
            a,b=map(f32,p.get('secondaryMechDependentRange',[0.,1.]))
            low,high=max(low,a),min(high,b)
    return low,high


def settled_flaps(fm,requested,mach,ias_u,sweep=0.):
    """Held request after native mechanism/Mach/IAS caps have settled.

    The diagram assumes prior deployment where necessary; actuator travel time
    is not an EM output. Passing actual=requested does not bypass availability:
    the returned target is re-evaluated by the same native actuator procedure.
    """
    if requested==0. or not fm.get('AvailableControls',{}).get('hasFlapsControl',False):return 0.
    bounds=flap_mechanism_bounds(fm,mach,ias_u,sweep)
    return max(0.,min(1.,flap_step(fm,requested,requested,mach,ias_u,0.,bounds)['requested']))
