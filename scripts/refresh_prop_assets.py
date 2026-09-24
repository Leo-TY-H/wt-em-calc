"""Regenerate one vehicle's derived assets from current FM and installed geometry.

Requires the local pinned native loader. Source and geometry versions remain
independent. Both generated artifacts are published together after mass checks;
the source hash is never simply relabeled to make an old asset pass its guard.
"""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import tempfile

from sync_game_data import ROOT, json_bytes, publish, read_json, sync_lock


def collision_packages(game, resource):
    matches = []
    for path in sorted((game/'content/base/res/aircrafts').glob('*logic.grp')):
        data = path.read_bytes()
        if data[:4] != b'GRP2':
            continue
        no, nc = struct.unpack_from('<II', data, 16)
        ro, rc = struct.unpack_from('<II', data, 32)
        names = []
        for i in range(nc):
            offset = struct.unpack_from('<I', data, no+4*i)[0]
            names.append(data[offset:data.index(0, offset)].decode())
        for cls, _, index, _ in struct.iter_unpack('<IIHH', data[ro:ro+12*rc]):
            if cls == 0xace50000 and names[index] == resource:
                matches.append(path)
    if not matches:
        raise ValueError('No exact installed collision resource: '+resource)
    return matches


def refresh(vehicle, game, report_path):
    from prop_config_native import PropConfigNative
    from propulsion_config import decode
    from extract_prop_collision import extract
    from fm_loader import normalize
    from mass_model import aircraft_properties, evaluate as legacy_evaluate
    from advanced_mass import evaluate as advanced_evaluate
    from fuel_loading import initial
    from verify_mass_model import MassMachine
    with sync_lock(ROOT) as work:
        version = read_json(ROOT/'references/data-version.json')
        rows = read_json(ROOT/'references/prop-vehicles/manifest.json')['records']
        row = next(r for r in rows if r['vehicle']==vehicle)
        name = Path(row['fm']).stem if row['fm'] else vehicle
        resource = row['model']+'_collision'
        raw = (ROOT/'references/jet-catalog/fm'/(name+'.blkx')).read_bytes()
        fm = normalize(json.loads(raw)); sha = hashlib.sha256(raw).hexdigest()
        variants = [extract(path, resource) for path in collision_packages(game, resource)]
        if len({v[2]['stream_sha256'] for v in variants}) != 1:
            raise ValueError('Conflicting installed collision resources')
        _, stream, provenance = variants[0]
        provenance.update(fm_commit=version['commit'],fm_version=version['version'],
            version_boundary='Installed geometry and FM data are independently pinned; version equivalence is not assumed.')
        loader = PropConfigNative()
        loader.load(stream, limit=50000000)
        if loader.pos != len(stream):
            raise ValueError('Collision decoder did not consume the complete stream')
        properties = decode(loader, loader.load_config(json.loads(raw)))
        extra = dict(masses={'engine%d_dm'%(e['index']+1):e['mass'] for e in properties['engines']},
            parts={'prop%d_dm'%p['index']:dict(mass=p['properties']['mass'],pos=p['position']) for p in properties['propellers']})
        result = loader.produce(fm, extra=extra)
        if result['damage_records']:
            raise ValueError('Explicit mass parts need separate validation')
        p = aircraft_properties(fm); native = MassMachine(); checks = []
        for fraction in (0., .3, .7, 1.):
            fill = initial(result['tanks'], result['systems'], fraction)
            kw = dict(fuel_by_tank=fill['fuel_by_tank'])
            if p['advanced_mass']:kw['mass_parts']=result['records']
            actual = native.evaluate(p, fill['fuel_by_system'], **kw)
            expected = (advanced_evaluate if p['advanced_mass'] else legacy_evaluate)(p, fill['fuel_by_system'], **kw)
            differences = {k:[v,expected[k]] for k,v in actual.items() if v!=expected[k]}
            if differences:
                raise ValueError('Native mass consumer differs at fuel fraction '+str(fraction)+': '+str(differences))
            checks.append(dict(fuel_fraction=fraction,mass=actual['mass']))
        propulsion = dict(schema=1,aircraft=name,commit=version['commit'],binary_sha256=loader.sha,
            fm_sha256=sha,properties=properties,
            scope='Current FM, pinned original property loader, value-only decoding; dynamics remain independent ports.')
        mass = dict(aircraft=name,resource=resource,binary_sha256=loader.sha,fm_sha256=sha,
            collision_provenance=provenance,vehicle_mapping=dict(aircraft=name,vehicles=[row],assets=[resource],referenced=True),
            extra_masses=extra,tank_capacities=[t['capacity'] for t in result['tanks']],tanks=result['tanks'],
            fuel_systems=result['systems'],records=result['records'],damage_records=result['damage_records'],consumer_checks=4,
            scope='Current FM and exact installed collision resource; native producer and independent intact mass consumer at four fuel selections. Geometry and executable pins are separate from FM version; no ammunition or payload.')
        names = {'references/prop-propulsion/'+name+'.json':propulsion,
                 'references/prop-mass/'+name+'--'+resource+'.json':mass}
        manifest_name = 'references/prop-propulsion/manifest.json'
        manifest = read_json(ROOT/manifest_name)
        records = {r['aircraft']:r for r in manifest['records']}
        records[name] = dict(aircraft=name,sha256=hashlib.sha256(json_bytes(propulsion)).hexdigest())
        manifest['records'] = sorted(records.values(),key=lambda r:r['aircraft'])
        # The manifest can contain assets from several input commits. Each
        # artifact's own commit/hash remains the authoritative provenance.
        manifest['scope'] = 'Mixed input generations; inspect each asset commit and FM hash.'
        names[manifest_name] = manifest
        with tempfile.TemporaryDirectory(prefix='assets-',dir=work) as tmp:
            stage = Path(tmp)
            for name, value in names.items():
                path = stage/name; path.parent.mkdir(parents=True,exist_ok=True); path.write_bytes(json_bytes(value))
            publish(ROOT, stage, names, [])
        report = dict(vehicle=vehicle,data_commit=version['commit'],data_version=version['version'],fm_sha256=sha,
            binary_sha256=loader.sha,resource=resource,checks=checks,files=list(names),status='PASS')
        report_path.parent.mkdir(parents=True,exist_ok=True)
        report_path.write_bytes(json_bytes(report))
        return report


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('aircraft')
    p.add_argument('--game',type=Path,required=True)
    p.add_argument('--report',type=Path,default=Path('analysis/windows-instructor/asset-refresh.json'))
    args = p.parse_args()
    print(json.dumps(refresh(args.aircraft,args.game,args.report)),flush=True)


if __name__=='__main__':main()
