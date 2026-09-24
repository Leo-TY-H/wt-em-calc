"""Pinned jet records, engine-instance resolution and explicit capability checks."""
import copy,json,re,math
from functools import lru_cache
from pathlib import Path
from component_assembly import f32,add,mul
from mass_model import tank_configuration
from fm_loader import normalize

ROOT=Path(__file__).resolve().parents[1]
DIRECTORY=ROOT/'references/jet-catalog/fm'
REFERENCE=['f_16a_block_15_adf','saab_jas39c']
NAMES={'f_16a_block_15_adf':'F-16A ADF','saab_jas39c':'JAS39C',
       'f_14a_early':'F-14A Early','f_14a_iriaf':'F-14A IRIAF','f_14b':'F-14B',
       'mig_23m':'MiG-23M','mig_23ml':'MiG-23ML','mig_23mld':'MiG-23MLD',
       'mig_27k':'MiG-27K','mig_27m':'MiG-27M','f_111a':'F-111A','f_111f':'F-111F'}


def source(name):
    # Historical oracle fixtures remain separate from the current runtime data.
    return DIRECTORY/f'{name}.blkx'


def merge(base,override):
    out=copy.deepcopy(base)
    for key,value in override.items():
        out[key]=merge(out[key],value) if isinstance(value,dict) and isinstance(out.get(key),dict) else copy.deepcopy(value)
    return out


def engines(fm):
    """Resolve each installed engine's type and instance overrides in index order."""
    found=[]
    for key in sorted((k for k in fm if re.fullmatch(r'Engine\d+',k)),key=lambda k:int(k[6:])):
        instance=fm[key];base=fm.get('EngineType'+str(instance.get('Type',0)),{})
        resolved=merge(base,instance)
        if resolved.get('Main',{}).get('Type')=='Jet' and not any(re.fullmatch(r'Nozzle\d+',k) for k in resolved):
            # Legacy fallback in1019ec630: one engine-root nozzle, ratio1.
            # Only neutral controls are used here; the old secondary basis
            # is not equivalent to modern Direction2 for vectoring commands.
            x,y,z=map(f32,resolved.get('Vector',[1.,0.,0.]))
            degrees=f32(57.2957763671875)
            direction=[mul(f32(math.atan2(-z,x)),degrees),
                       mul(f32(math.atan2(y,f32(math.sqrt(add(mul(z,z),mul(x,x)))))),degrees)]
            resolved['Nozzle0']=dict(Position=resolved.get('Position',[0.,0.,0.]),
                Direction=resolved.get('Direction',direction),ThrustRatio=1.,ThrustMax=2147440000.,
                FlapsToThrust=resolved.get('FlapsToThrust',[0.,1.,1.,1.]))
        found.append((key,resolved))
    return found


def fuel_capacities(fm):
    tanks=tank_configuration(fm);capacities={}
    for tank in tanks:
        if not tank['external']:
            i=int(tank['system']);capacities[i]=add(capacities.get(i,0.),tank['capacity'])
    if not capacities:
        mass=fm['Mass']
        capacities={int(k[len('MaxFuelMass'):]):f32(v) for k,v in mass.items() if re.fullmatch(r'MaxFuelMass\d+',k)}
        if not capacities and 'MaxFuelMass' in mass:capacities={0:f32(mass['MaxFuelMass'])}
    if not capacities or sum(capacities.values())<=0:raise ValueError('No internal fuel capacity')
    return [capacities.get(i,0.) for i in range(max(capacities)+1)]


@lru_cache(maxsize=64)
def load(name):return normalize(json.loads(source(name).read_text()))


@lru_cache(maxsize=1)
def catalog():
    from aircraft_model import prepare,condition_properties
    from primary_controls import selected_properties
    from engine_supply import selected_properties as engine_properties
    from jet_model import prepare as prepare_jet
    from jet_nozzle import prepare as prepare_nozzle
    out={}
    for path in sorted(DIRECTORY.glob('*.blkx')):
        raw=json.loads(path.read_text());installed=engines(raw)
        if not any(v.get('Main',{}).get('Type')=='Jet' for _,v in installed):continue
        name=path.stem;ad=raw.get('Aerodynamics',{})
        entry=dict(name=NAMES.get(name,name.replace('_',' ').replace('-','-')),color='#38c9d7',
                   has_sweep=any(k.startswith('WingPlaneSweep') for k in ad),supported=False,
                   has_flaps=bool(raw.get('AvailableControls',{}).get('hasFlapsControl',False)),
                   engine_count=sum(e.get('Main',{}).get('Type')=='Jet' for _,e in installed))
        try:
            fm=load(name);ad=fm['Aerodynamics']
            entry['has_flaps']=bool(fm['AvailableControls'].get('hasFlapsControl',False))
            if fm['Mass'].get('AdvancedMass',False):raise ValueError('Advanced mass requires component mesh geometry')
            if 'WingPlane' not in ad and not entry['has_sweep']:raise ValueError('Legacy aerodynamic loader integration pending')
            if any(e.get('Main',{}).get('Type') not in ('Jet','Rocket') for _,e in installed):raise ValueError('Mixed propulsion requires the propeller model')
            model=prepare(fm);condition_properties(model,.6,0.);selected_properties(fm)
            entry['fuel_capacity']=sum(fuel_capacities(fm))
            for _,e in engines(fm):
                if e['Main']['Type']=='Rocket':continue  # clean normal flight, boosters off
                jet=prepare_jet(e['Main']);engine_properties(e)
                if not jet['modes']:raise ValueError('Engine record has no RPM modes; running lifecycle unavailable')
                nozzles=[v for k,v in e.items() if re.fullmatch(r'Nozzle\d+',k)]
                if not nozzles:raise ValueError('Implicit nozzle loader integration pending')
                for n in nozzles:prepare_nozzle(n)
            entry['supported']=True
        except Exception as error:
            entry['reason']=str(error)
        out[name]=entry
    for i,name in enumerate(REFERENCE):
        if name in out:out[name]['color']=['#38c9d7','#ffa66b'][i]
    return out
