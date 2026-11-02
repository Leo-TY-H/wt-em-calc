"""Compact, replayable performance tables from the existing clean jet model.

This intentionally excludes trim drag, sideslip, propulsion transients and damage.
Only fixed-wing, fixed-sweep pure jets supported by the existing adapters are listed.
"""
from functools import lru_cache
import hashlib
import math
import re

import jet_catalog
from aircraft_model import prepare, condition_properties
from air_state import speed_of_sound
from control_mixer import density_at_height
from jet_model import prepare as prepare_jet, steady, prepare_nozzle, selected_nozzle
from mass_model import aircraft_properties


@lru_cache(maxsize=1)
def catalog():
    return [dict(id=key, name=row['name']) for key,row in jet_catalog.catalog().items()
            if row['supported'] and row['propulsion']=='jet' and not row['has_sweep']]


@lru_cache(maxsize=32)
def profile(aircraft_id, throttle=1.1):
    choices={r['id']:r for r in catalog()}
    if aircraft_id not in choices: raise ValueError('Select a supported fixed-wing jet target')
    if throttle not in (1.,1.1): raise ValueError('Target throttle must be military power or afterburner')
    fm=jet_catalog.load(aircraft_id); model=prepare(fm); props=aircraft_properties(fm)
    mass=props['empty']+props['oil']+props['crew_mass']+.5*sum(jet_catalog.fuel_capacities(fm))
    area=model['geometry']['area']; ad=fm['Aerodynamics']; strength=ad['WingPlane']['Strength']
    machs=sorted(set([i/40 for i in range(121)]+[float(strength['MNE'])]))
    aero=[]
    for mach in machs:
        _,wing,secondary=condition_properties(model,mach,0.)
        # Secondary clean zero-lift drag, expressed on the wing reference area.
        cd0=(wing['cd0']+ad.get('FuseCd',0.))*wing['kq']
        for key in ('HorStabPlane','VerStabPlane','FuselagePlane'):
            from polar_runtime import evaluate
            p=evaluate(model['polars'][key][0][1],mach)
            cd0+=p['cd0']*p['kq']*sum(ad[key]['Areas'].values())/area
        aero.append([mach,cd0,wing['indCoeff']*wing['kq']/(wing['clKq']**2),
                     wing['cyCritL']*wing['clKq'],wing['cyCritH']*wing['clKq']])
    heights=list(range(0,30001,1000)); speeds=list(range(0,1001,50))
    installed=[]
    for _,engine in jet_catalog.engines(fm):
        if engine['Main']['Type']!='Jet': raise ValueError('Mixed propulsion is not supported for live targets')
        installed.append((prepare_jet(engine['Main']),[prepare_nozzle(v) for k,v in engine.items() if re.fullmatch(r'Nozzle\d+',k)]))
    thrust=[]; atmosphere=[]
    for height in heights:
        rho=float(density_at_height(height)); atmosphere.append([height,rho,float(speed_of_sound(height))])
        row=[]
        for speed in speeds:
            total=0.
            for engine,nozzles in installed:
                t=steady(engine,height,speed,throttle)['thrust']
                total+=sum(selected_nozzle(n,t,[0.,0.,0.],ias=speed*math.sqrt(rho/1.225))['force'][0] for n in nozzles)
            row.append(total)
        thrust.append(row)
    out=dict(version=1,aircraft_id=aircraft_id,name=choices[aircraft_id]['name'],
             source_sha256=hashlib.sha256(jet_catalog.source(aircraft_id).read_bytes()).hexdigest(),
             mass_kg=mass,area_m2=area,fuel_fraction=.5,throttle=throttle,
             positive_g=2.*strength['CritOverload'][1]/(mass*9.81),
             negative_g=2.*strength['CritOverload'][0]/(mass*9.81),
             vne_mps=strength['VNE']/3.6,mne=strength['MNE'],
             roll_rate_deg_s=180.,roll_reference_mps=fm['AileronEffectiveSpeed']/3.6,
             response_s=.25,atmosphere=atmosphere,aero=aero,speeds=speeds,thrust=thrust)
    from missile_aircraft_motion import validate_profile
    return validate_profile(out)
