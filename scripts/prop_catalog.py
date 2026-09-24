"""Pinned fixed-wing prop vehicles, installation assets and mass preparation."""
import hashlib,json
from functools import lru_cache
from pathlib import Path
from component_assembly import f32
from fm_loader import normalize
from fuel_loading import initial
from mass_model import aircraft_properties,evaluate as legacy_mass
from advanced_mass import evaluate as advanced_mass
ROOT=Path(__file__).resolve().parents[1]
_worker_snapshot=None
NAMES={'yak-3':'Yak-3','bf-109f-4':'Bf 109 F-4','a2d':'A2D-1','a5m4':'A5M4','fw_200c_1':'Fw 200 C-1'}


def asset_mismatch(raw, propulsion, mass):
    expected=hashlib.sha256(raw).hexdigest()
    if propulsion.get('fm_sha256')!=expected or mass.get('fm_sha256')!=expected:
        return 'Updated FM needs regenerated propulsion/collision assets; cached assets are from an older FM'
    if propulsion.get('binary_sha256')!=mass.get('binary_sha256'):
        return 'Propeller assets use different executable versions'
    return None


def fm_source(name):return ROOT/'references/jet-catalog/fm'/(name+'.blkx')


@lru_cache(maxsize=1)
def catalog():
    if _worker_snapshot is not None:return _worker_snapshot
    census=json.loads((ROOT/'references/prop-native-config.json').read_text())
    names={r['aircraft'] for r in census['aircraft']}
    # Include newly added propeller FMs even before native assets are prepared.
    for path in (ROOT/'references/jet-catalog/fm').glob('*.blkx'):
        raw=json.loads(path.read_bytes())
        if any(k.startswith('Propeller') and isinstance(v,dict) for k,v in raw.items()):names.add(path.stem)
    vehicles=json.loads((ROOT/'references/prop-vehicles/manifest.json').read_text())['records']
    out={}
    for vehicle in vehicles:
        fm_id=Path(vehicle['fm']).stem if vehicle['fm'] else vehicle['vehicle']
        if fm_id not in names:continue
        name=vehicle['vehicle']
        if not fm_source(fm_id).exists():continue
        raw=fm_source(fm_id).read_bytes();fm=normalize(json.loads(raw))
        propulsion_path=ROOT/'references/prop-propulsion'/(fm_id+'.json')
        resource=(vehicle['model'] or '')+'_collision'
        mass_path=ROOT/'references/prop-mass'/(fm_id+'--'+resource+'.json')
        entry=dict(name=NAMES.get(name,name.replace('_',' ')),fm_id=fm_id,vehicle_id=name,resource=resource,
            color='#edb45e',has_sweep=False,has_flaps=bool(fm.get('AvailableControls',{}).get('hasFlapsControl',False)),
            propulsion='propeller',supported=False,reason='Propeller equilibrium integration in progress')
        if not propulsion_path.exists():entry['reason']='Pinned propulsion properties are unavailable'
        elif not mass_path.exists():entry['reason']='Matching collision geometry is unavailable'
        else:
            propulsion=json.loads(propulsion_path.read_text());m=json.loads(mass_path.read_text())
            mismatch=asset_mismatch(raw,propulsion,m)
            if mismatch:
                entry['reason']=mismatch
                out[name]=entry
                continue
            p=propulsion['properties']
            entry.update(engine_count=sum(e['family']!=3 for e in p['engines']),propeller_count=len(p['propellers']),
                fuel_capacity=sum(s['capacity']-s['external_capacity'] for s in m['fuel_systems']),
                propulsion='mixed' if any(e['family']==2 for e in p['engines']) else 'turboprop' if any(e['family']==5 for e in p['engines']) else 'piston',
                advanced_mass=bool(fm['Mass'].get('AdvancedMass',False)))
            entry.update(supported=True,experimental=True,reason='Experimental propeller EM: native equations checked; global optimum and live-flight accuracy not certified')
        out[name]=entry
    return out


@lru_cache(maxsize=64)
def load(name):return normalize(json.loads(fm_source(catalog()[name]['fm_id']).read_text()))


@lru_cache(maxsize=64)
def assets(name):
    row=catalog()[name];fm_id=row['fm_id']
    propulsion=json.loads((ROOT/'references/prop-propulsion'/(fm_id+'.json')).read_text())
    mass=json.loads((ROOT/'references/prop-mass'/(fm_id+'--'+row['resource']+'.json')).read_text())
    expected=hashlib.sha256(fm_source(fm_id).read_bytes()).hexdigest()
    if propulsion['fm_sha256']!=expected or mass['fm_sha256']!=expected:raise ValueError('Propeller assets do not match the pinned FM')
    if propulsion['binary_sha256']!=mass['binary_sha256']:raise ValueError('Propeller assets use different executable versions')
    from aircraft_upgrades import apply_propulsion
    return apply_propulsion(name,propulsion['properties']),mass


def mass_state(name,fuel_percent,extra_mass=0.):
    fm=load(name);_,asset=assets(name);p=aircraft_properties(fm)
    fill=initial(asset['tanks'],asset['fuel_systems'],fuel_percent/100.)
    payloads=[dict(mass=extra_mass,position=fm['Mass']['CenterOfGravity'])] if extra_mass else []
    nitro=f32(fm['Mass'].get('MaxNitro',0.))
    kw=dict(payloads=payloads,fuel_by_tank=fill['fuel_by_tank'],nitro=nitro)
    if asset['damage_records']:raise ValueError('Explicit mass parts require separate validation')
    if p['advanced_mass']:kw['mass_parts']=asset['records']
    result=(advanced_mass if p['advanced_mass'] else legacy_mass)(p,fill['fuel_by_system'],**kw)
    result.update(fill,nitro_mass=nitro,mass_geometry=asset['resource'],mass_policy='Native internal-fuel placement and running reservoir-inclusive totals; healthy intact aircraft; no weapon ammunition')
    return result
