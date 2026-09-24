"""Healthy discrete and turbo-supercharger consumers of loaded piston data."""
import math
from component_assembly import f32,add,sub,mul
from piston_model import div,boost_active,pressure_at_height
from control_mixer import curve
from structural_limits import interval


def speed_factor(p,omega):
    x=mul(p['inverse_omega'],omega)
    return add(mul(add(mul(sub(mul(x,x),x),p['compressor_sq']),x),sub(1.,p['compressor_zero'])),p['compressor_zero'])


def requested_pressure(p,throttle,height):
    if not p['ata_enabled']:return mul(add(mul(min(throttle,1.),f32(.45)),f32(.55)),p['max_ata'])
    if throttle<f32(.01):return div(pressure_at_height(height),101300.)
    return curve(p['ata'],throttle,1)[0]


def low_rpm(manifold,omega,inlet):
    if 20.<=omega<150.:return min(manifold,mul(add(mul(omega,f32(.04)),f32(-.39999995)),inlet))
    if omega<20.:return mul(add(mul(omega,f32(-.03)),1.),inlet)
    return manifold


def candidate(p,s,i,speed,inlet,requested,throttle,afterburner,nitro):
    boost=boost_active(p,throttle,afterburner,i,nitro)
    pb=p['boost_ata_ratio'] if boost else 1.;flow=mul(inlet,s['pressure_boost']) if boost else inlet
    critical,flat,ceiling=s['critical'],s['flat'],s['ceiling']
    flat_present=critical<add(flat,f32(-.0001));ceiling_present=add(ceiling,f32(.0001))<critical
    exact=p['exact_altitudes'];line=s['ceiling_factor'] if exact and ceiling_present else 1.
    if not exact and (flat_present or ceiling_present):
        scale=mul(pb,div(1.,speed));critical=mul(scale,critical);flat=mul(flat,scale);ceiling=mul(ceiling,scale)
    baseline=1.
    for _ in range(i):baseline=mul(mul(baseline,max(s['power'],1.)),f32(.8))
    if flow<=critical:
        if exact:shape=s['power']
        elif flow<=ceiling:shape=s['ceiling_factor'] if ceiling_present else s['power']
        else:shape=add(div(mul(sub(s['ceiling_factor'],s['power']),sub(critical,flow)),max(sub(critical,ceiling),f32(.0001))),s['power'])
    elif not flat_present:
        shape=add(div(mul(sub(s['power'],baseline),sub(1.,flow)),max(sub(1.,critical),f32(.0001))),baseline)
    elif flow<=flat:
        x=(min(1.,max(0.,add(1.,div(mul(sub(flow,critical),-1.),sub(flat,critical))))) if exact
           else interval(flow,critical,1.,flat,0.))
        shape=add(mul(f32(math.pow(x,s['curvature'])),sub(s['power'],s['flat_power'])),s['flat_power'])
    else:shape=add(div(mul(sub(s['flat_power'],baseline),sub(1.,flow)),max(sub(1.,flat),f32(.0001))),baseline)
    potential=mul(mul(flow,speed),div(p['max_ata'],mul(p['compressor_reference'],s['critical'])))
    score=min(potential,mul(p['max_ata'],pb));offset=sub(1.,line)
    if exact:score=add(div(mul(score,line),mul(pb,requested)),offset)
    score=mul(score,shape)
    if boost:score=mul(score,mul(s['boost'],p['afterburner_boost']))
    return dict(gear=i,shape=shape,flow=flow,line=line,offset=offset,pb=pb,score=score)


def step(p,omega,throttle,inlet,dt,*,gear=0,old_gear=0,regulator=-1.,afterburner=False,nitro=0.,
         turbo=0.,turbo_command=1.,automatic_turbo=True,assisted=False,height=0.):
    omega,throttle,inlet,dt,regulator=map(f32,[omega,throttle,inlet,dt,regulator])
    requested=requested_pressure(p,throttle,height);speed=speed_factor(p,omega)
    if p['compressor_type']==3:
        if not p['exact_altitudes']:raise ValueError('Unvalidated alternate turbo-supercharger curve')
        s=p['stages'][0]
        if inlet<=s['critical']:shape=s['power']
        else:shape=add(div(mul(sub(s['power'],1.),sub(1.,inlet)),max(sub(1.,s['critical']),f32(.0001))),1.)
        base=mul(mul(speed,inlet),div(p['max_ata'],mul(p['compressor_reference'],s['critical'])))
        if boost_active(p,throttle,afterburner,0,nitro):requested=mul(requested,p['boost_ata_ratio'])
        upper=interval(omega,p['shaft_min'],p['turbo_min'],p['max_omega'],p['turbo_max']);allowed=p['turbo_allowed']
        if automatic_turbo or assisted:
            target=min(mul(div(allowed,base),requested),allowed);turbo_command=div(target,sub(upper,p['turbo_min']))
        else:target=add(mul(sub(upper,p['turbo_min']),turbo_command),p['turbo_min'])
        turbo=f32(turbo)
        turbo=target if assisted else add(mul(mul(sub(target,turbo),dt),p['inverse_time']),turbo)
        if turbo<p['turbo_min']:
            fraction=div(turbo,p['turbo_min']);potential=mul(inlet,.75)
            if fraction>0.:
                top=mul(div(p['turbo_min'],allowed),base)
                potential=add(potential,mul(sub(top,potential),fraction)) if fraction<1. else top
        else:potential=mul(div(turbo,allowed),base)
        ratio=min(div(requested,potential),1.);manifold=mul(ratio,potential)
        line=s['ceiling_factor'] if add(s['ceiling'],f32(.0001))<s['critical'] else 1.
        factor=mul(add(div(mul(line,manifold),requested),sub(1.,line)),shape)
        return dict(multiplier=factor,gear=0,regulator=1.,throttle_ratio=ratio,potential_manifold=potential,
                    manifold=low_rpm(manifold,omega,inlet),turbo=turbo,turbo_command=turbo_command)
    if assisted:raise ValueError('Assisted discrete compressor bypass is outside manual-performance scope')
    if gear is not None and not 0<=gear<len(p['stages']):raise ValueError('Compressor stage unavailable')
    selected=None;best=-1.
    for i in range(len(p['stages'])) if gear is None else [gear]:
        row=candidate(p,p['stages'][i],i,speed,inlet,requested,throttle,afterburner,nitro)
        if row['score']>best:selected=row;best=row['score']
    if selected is None:
        selected=dict(gear=0,shape=-1.,flow=inlet,line=1.,offset=0.,
                      pb=p['boost_ata_ratio'] if boost_active(p,throttle,afterburner,0,nitro) else 1.)
    i=selected['gear'];s=p['stages'][i]
    potential=mul(mul(speed,selected['flow']),div(p['max_ata'],mul(p['compressor_reference'],s['critical'])))
    target=mul(div(p['max_ata'],potential),selected['pb'])
    if i==old_gear and regulator>=0.:
        gain=f32(.1) if p['compressor_type']==2 else mul(dt,3.)
        target=add(mul(sub(target,regulator),gain),regulator)
    regulator=min(1.,max(0.,target));ratio=min(div(requested,p['max_ata']),1.)
    manifold=mul(mul(potential,ratio),regulator)
    factor=mul(add(div(mul(manifold,selected['line']),mul(requested,selected['pb'])),selected['offset']),selected['shape'])
    return dict(multiplier=factor,gear=i,regulator=regulator,throttle_ratio=ratio,
                potential_manifold=potential,manifold=low_rpm(manifold,omega,selected['flow']))
