"""Actual installed collision resources -> portable native mass records.

Each geometry alternative is retained explicitly. No surrogate meshes are used.
Consumer checks prescribe tank contents; they do not validate initial fuel fill.
"""
import argparse,hashlib,json
from pathlib import Path
from prop_config_native import PropConfigNative
from propulsion_config import decode
from extract_prop_collision import extract
from fm_loader import normalize
from mass_model import aircraft_properties,evaluate as legacy_evaluate
from advanced_mass import evaluate as advanced_evaluate
from fuel_loading import initial
from verify_mass_model import MassMachine
from component_assembly import f32

ROOT=Path(__file__).resolve().parents[1]


def main():
    ap=argparse.ArgumentParser();ap.add_argument('names',nargs='*');args=ap.parse_args()
    mappings=json.loads((ROOT/'analysis/prop-integration/geometry-mapping.json').read_text())
    index=json.loads((ROOT/'analysis/prop-integration/collision-index.json').read_text())['resources']
    out=ROOT/'references/prop-mass';out.mkdir(exist_ok=True)
    reports=[];failures=[];native=MassMachine()
    for mapping in mappings:
        name=mapping['aircraft']
        if args.names and name not in args.names:continue
        source=ROOT/'references/jet-catalog/fm'/(name+'.blkx');raw=source.read_bytes();fm=normalize(json.loads(raw))
        for resource in mapping['assets']:
            try:
                variants=[extract(Path(path),resource) for path in index[resource]]
                if len({p['stream_sha256'] for _,_,p in variants})!=1:raise ValueError('Installed packages contain conflicting collision resources')
                _,stream,provenance=variants[0]
                m=PropConfigNative();m.load(stream,limit=50000000);assert m.pos==len(stream)
                cfg=decode(m,m.load_config(json.loads(raw)))
                extra=dict(masses={'engine%d_dm'%(e['index']+1):e['mass'] for e in cfg['engines']},
                    parts={'prop%d_dm'%p['index']:dict(mass=p['properties']['mass'],pos=p['position']) for p in cfg['propellers']})
                result=m.produce(fm,extra=extra)
                p=aircraft_properties(fm);count=result['tank_count'];capacities=[]
                # Producer stores the complete generated capacity array in the
                # allocated mass properties; retain it as an explicit artifact.
                capacities=[t['capacity'] for t in result['tanks']]
                for fraction in [0.,.3,.7,1.]:
                    fill=initial(result['tanks'],result['systems'],fraction)
                    fuel=fill['fuel_by_system']
                    kw=dict(fuel_by_tank=fill['fuel_by_tank'])
                    if result['damage_records']:raise ValueError('Explicit mass parts need separate consumer validation')
                    if p['advanced_mass']:kw['mass_parts']=result['records']
                    evaluate=advanced_evaluate if p['advanced_mass'] else legacy_evaluate
                    a=native.evaluate(p,fuel,**kw);e=evaluate(p,fuel,**kw)
                    diff={k:[a[k],e[k]] for k in a if a[k]!=e[k]}
                    if diff:raise ValueError('Mass consumer discrepancy: '+str(diff))
                artifact=dict(aircraft=name,resource=resource,binary_sha256=m.sha,fm_sha256=hashlib.sha256(raw).hexdigest(),
                    collision_provenance=provenance,vehicle_mapping=mapping,extra_masses=extra,tank_capacities=capacities,
                    tanks=result['tanks'],fuel_systems=result['systems'],
                    records=result['records'],damage_records=result['damage_records'],consumer_checks=4,
                    scope='Original collision decoder, complete mass-property loader and geometry producer; independent native-priority fuel allocation and intact mass consumer. Four initial internal fuel selections. Default authored intact pose. No weapon ammunition or payload. Installed geometry and FM patch pins remain separate.')
                path=out/(name+'--'+resource+'.json');path.write_text(json.dumps(artifact,indent=2)+'\n')
                reports.append(dict(aircraft=name,resource=resource,records=len(result['records']),tank_count=count,artifact=str(path.relative_to(ROOT))))
                print(name,resource,len(result['records']),'PASS',flush=True)
            except Exception as error:
                failures.append(dict(aircraft=name,resource=resource,error=repr(error)));print(name,repr(error),'FAIL',flush=True)
    report=dict(status='FAIL' if failures else 'PASS',rows=reports,failures=failures,
        unresolved=[r for r in mappings if not r['assets']])
    (ROOT/'analysis/prop-integration/geometry-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(report['status'],len(reports),'geometries',len(failures),'failures',len(report['unresolved']),'unresolved',flush=True)
    if failures:raise SystemExit(1)


if __name__=='__main__':main()
