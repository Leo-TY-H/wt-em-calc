"""Pinned fixed-wing propeller mechanism census; no capability claims.

Combine declared type/instance fields for this source inventory. This is not
the native loading procedure: typed shared properties and mounting are loaded
separately by prop_config_native.py, whose catalog census is authoritative for
effective defaults and legacy aliases. Require a shaft
engine (Inline/Radial/TurboProp): legacy jet files can retain dummy Propellor
blocks. Exclude cyclic-pitch rotorcraft, including compound helicopters.
The catalog includes unused/event/AI records and aliases, not just playable
aircraft. No absent field is silently treated as a verified native default.
"""
import hashlib
import json
import re
from collections import Counter, defaultdict
from pathlib import Path
from jet_catalog import engines, merge

ROOT=Path(__file__).resolve().parents[1]
SHAFT={'Inline','Radial','TurboProp'}
AIRSHIPS={'zeppelin':'https://warthunder.com/en/devblog/current/9438'}

def propellers(fm, installed):
    out=[]
    for k,v in fm.items():
        if re.fullmatch(r'Propeller\d+',k):
            out.append((k,merge(fm.get('PropellerType'+str(v.get('Type',0)),{}),v)))
    if not out:
        for k,e in installed:
            if e['Main']['Type'] in SHAFT and 'Propellor' in e:
                raw=e['Propellor']
                out.append((k+'.Propellor',dict(Geometry=raw,Governor=raw,
                                              Pos=e.get('PropPos',e.get('Position')))))
    return out

def describe(fm,installed,props):
    features=set(); summary={}
    def add(k,v):
        v='absent' if v is None else str(v)
        features.add(k+'='+v);summary.setdefault(k,[]).append(v)
    add('schema','typed' if 'PropellerType0' in fm else 'inline' if 'Propeller0' in fm else 'legacy')
    add('engine_count',len(installed));add('prop_count',len(props))
    add('shaft_engine_count',sum(e['Main']['Type'] in SHAFT for _,e in installed))
    add('advanced_mass',fm.get('Mass',{}).get('AdvancedMass'))
    add('airframe_schema','plane_blocks' if 'WingPlane' in fm.get('Aerodynamics',{}) else 'legacy')
    add('mixed_propulsion',any(e['Main']['Type'] not in SHAFT for _,e in installed))
    for k,e in installed:
        main=e['Main'];add('engine_type',main['Type'])
        if main['Type'] not in SHAFT:continue
        for group,key in [('Main','CarbueretorType'),('Compressor','Type'),('Compressor','NumSteps'),
                          ('Compressor','ExactAltitudes'),('Compressor','IsControllable'),('Mixer','Type'),
                          ('Afterburner','Type'),('AutoThrottle','HasContorller')]:
            add(group+'.'+key,e.get(group,{}).get(key))
        add('nozzle_records_present',any(re.fullmatch(r'Nozzle\d+',key) for key in e))
    for k,p in props:
        for group,key in [('Governor','GovernorType'),('Governor','GovernorFast'),
                          ('Geometry','AirFlowSolver'),('Geometry','Coaxial'),
                          ('Geometry','RotationDirection'),('Controls','HasManualPitchControl'),
                          ('Controls','HasAutoPitchControl'),('Controls','HasFeatheringControl')]:
            add(group+'.'+key,p.get(group,{}).get(key))
        add('nonzero_axis_direction',any(v!=0 for v in p.get('AxisDirection',[0.,0.])))
        pos=p.get('Pos');add('prop_behind_origin',pos[0]<0 if pos else None)
        add('off_center_prop',pos[2]!=0 if pos else None)
    transmissions=[]
    for k,t in fm.items():
        if not re.fullmatch(r'Transmission\d+',k):continue
        es=[(key,v) for key,v in t.items() if re.fullmatch(r'Engine\d+',key)]
        ps=[(key,v) for key,v in t.items() if re.fullmatch(r'Propeller\d+',key)]
        add('transmission_topology',str(len(es))+' engines/'+str(len(ps))+' props')
        for key in ['UseAutoPropInertia','CorrectPropellerToTransmissionLink']:
            add('Transmission.'+key,t.get(key))
        for key,v in t.items():
            if re.fullmatch(r'PropellerPitchSource\d+',key):add('pitch_source',v)
        transmissions.append(dict(name=k,source=t))
    return dict(features=sorted(features),categories=summary,transmissions=transmissions)

def main():
    manifest_path=ROOT/'references/jet-catalog/fm-manifest.json'
    manifest=json.loads(manifest_path.read_text())
    expected={Path(x['path']).name:x['sha'] for x in manifest['files']}
    rows=[];excluded=[];hash_failures=[];distribution=defaultdict(Counter); examples=defaultdict(lambda:defaultdict(list))
    paths=sorted((ROOT/'references/jet-catalog/fm').glob('*.blkx'))
    for path in paths:
        data=path.read_bytes();blobsha=hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest()
        if blobsha!=expected[path.name]:hash_failures.append(path.name)
        fm=json.loads(data);installed=engines(fm);props=propellers(fm,installed)
        if path.stem in AIRSHIPS:
            excluded.append(dict(aircraft=path.stem,reason='airship',source=AIRSHIPS[path.stem]));continue
        if any(p.get('Controls',{}).get('HasCyclicPitchControl',False) for _,p in props):
            excluded.append(dict(aircraft=path.stem,reason='cyclic_pitch_rotorcraft'));continue
        if not any(e['Main']['Type'] in SHAFT for _,e in installed):
            excluded.append(dict(aircraft=path.stem,reason='no_shaft_engine'));continue
        assert props,path.stem
        row=dict(aircraft=path.stem,sha256=hashlib.sha256(data).hexdigest(),
                 engines=[dict(name=k,type=e['Main']['Type']) for k,e in installed],
                 propeller_names=[k for k,_ in props],**describe(fm,installed,props))
        rows.append(row)
        for k,vs in row['categories'].items():
            for v in set(vs):
                distribution[k][v]+=1;examples[k][v].append(path.stem)
    # Greedy feature cover proposes test cases, not a claim of validation.
    selected=['yak-3','bf-109f-4'];by_name={r['aircraft']:r for r in rows}
    remaining=set().union(*(set(r['features']) for r in rows))
    for name in selected:remaining-=set(by_name[name]['features'])
    while remaining:
        candidate=max(rows,key=lambda r:len(remaining&set(r['features'])))
        assert remaining&set(candidate['features'])
        selected.append(candidate['aircraft']);remaining-=set(candidate['features'])
    report=dict(scope=__doc__,commit=manifest['commit'],files_scanned=len(paths),
                fixed_wing_prop_records=len(rows),hash_failures=hash_failures,
                excluded_counts=dict(Counter(r['reason'] for r in excluded)),
                counting='Each category counts aircraft records containing that value, once per value; multi-engine records can occur under multiple engine types. Counts are not unique designs or playable variants.',
                categories={k:dict(sorted(v.items())) for k,v in sorted(distribution.items())},
                category_examples={k:dict(v) for k,v in examples.items()},
                proposed_feature_cover=selected,aircraft=rows,excluded=excluded)
    (ROOT/'analysis/prop-catalog-mechanisms.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:report[k] for k in ['files_scanned','fixed_wing_prop_records','hash_failures','excluded_counts','categories','proposed_feature_cover']},indent=2))
    if hash_failures:raise SystemExit(1)

if __name__=='__main__':main()
