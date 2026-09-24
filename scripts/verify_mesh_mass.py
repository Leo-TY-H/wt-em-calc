"""Installed collision -> original mass producer -> independent/native consumer."""
import hashlib
import json
import random
from pathlib import Path
from mass_parts_native import MassPartsNative
from mass_model import aircraft_properties
from advanced_mass import evaluate
from verify_mass_model import MassMachine
from component_assembly import f32

def main():
    source=Path('references/fm-2.59.0.13/bf-109f-4.blkx');fm=json.loads(source.read_text())
    stream=Path('references/collision/bf_109f_4_collision.dump').read_bytes()
    m=MassPartsNative();m.load(stream);assert m.pos==len(stream)
    result=m.produce(fm)
    Path('analysis/bf-109f-4-native-mass-parts.json').write_text(json.dumps(result,indent=2)+'\n')
    native=MassMachine();p=aircraft_properties(fm);rng=random.Random(0x990300)
    failures=[];examples=[]
    for i in range(1003):
        fuel=[0.,148.,296.][i] if i<3 else f32(rng.uniform(0,296))
        payloads=[] if i<3 else [dict(mass=f32(rng.uniform(0,250)),position=[f32(rng.uniform(-3,3)) for _ in range(3)]) for _ in range(i%5)]
        kw=dict(mass_parts=result['records'],fuel_by_tank=[fuel],payloads=payloads)
        actual=native.evaluate(p,[fuel],**kw);expected=evaluate(p,[fuel],**kw)
        diff={k:[v,expected[k]] for k,v in actual.items() if v!=expected[k]}
        if diff: failures.append(dict(case=i,diff=diff))
        if i<3:examples.append(dict(fuel=fuel,**actual))
    report=dict(status='FAIL' if failures else 'PASS',binary_sha256=m.sha,
                fm_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                collision_provenance=json.loads(Path('references/collision/bf_109f_4_collision-provenance.json').read_text()),
                stream_consumed=m.pos,record_count=len(result['records']),sum_record_mass=sum(r['mass'] for r in result['records']),
                cases=1003,examples=examples,failures=failures,
                scope='Original version-3 collision decoder10014d140, default authored transforms and complete advanced-mass record producer101990300 on actual installed Bf109F4 geometry; original wrapper101998ad0/consumer101998bf0 versus independent advanced_mass.evaluate. Source/default BLK, name-map, allocator and material services adapted. Engine/propeller extra masses follow traced intact caller; one fuel tank, no ammunition or selected weapon loadout in examples. Mass producer is a native oracle, not an independently ported mesh algorithm. Default intact resource pose, not a live game capture. Record mass sum is not aircraft total mass: preserve original consumer arithmetic.')
    Path('analysis/mesh-mass-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    if failures:raise SystemExit(1)

if __name__=='__main__':main()
