"""Native configuration and complete healthy compressor consumer comparison."""
import argparse,json,random,struct
from pathlib import Path
from prop_config_native import PropConfigNative
from piston_config import loaded_properties
from piston_compressor import step
from verify_aircraft_native import FRAME
from component_assembly import f32

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser();parser.add_argument('names',nargs='*');parser.add_argument('--cases',type=int,default=100)
    args=parser.parse_args();rng=random.Random(101954)
    census=json.loads((ROOT/'references/prop-native-config.json').read_text())
    names=args.names or [r['aircraft'] for r in census['aircraft'] if any(e['family']<2 for e in r['engines'])]
    reports=[];failures=[]
    for name in names:
        m=PropConfigNative();cfg=m.load_config(json.loads((ROOT/'references/jet-catalog/fm'/(name+'.blkx')).read_text()))
        engines=m.describe(cfg)['engines'];types=sorted(set(e['type_id'] for e in engines if e['family']<2));cases=0
        for index in types:
            address=cfg+8+index*0x770;p=loaded_properties(m,address)
            fm=m.alloc(0x8500);state=m.alloc(0x100);out=m.alloc(0x40);dummy=m.alloc(0x80)
            m.u.mem_write(fm+0x8470,b'\x01');m.u.mem_write(fm+0x3658,b'\x04')
            m.u.mem_write(fm+0x2f5c,b'\x10') # request native automatic search when stage=-1
            m.u.mem_write(state+0xc,b'\x07');m.floats(state+0x9c,[1.]);m.floats(state+0x38,[1.])
            ns=ps=dict(regulator=-1.,turbo=f32(.7*p['turbo_allowed']))
            for i in range(args.cases):
                def v(lo,hi):return f32(rng.uniform(lo,hi))
                kw=dict(omega=v(0.,1.5*p['max_omega']),throttle=v(.02,1.1),inlet=v(.05,1.3),dt=f32(1/48),
                    gear=None if i%3==0 else i%len(p['stages']),old_gear=i%len(p['stages']),afterburner=bool(i%2),nitro=10. if i%3 else 0.,
                    turbo_command=v(0.,1.),automatic_turbo=bool(i%2),assisted=False)
                if i<args.cases//2:ns=ps=dict(regulator=-1. if i%4==0 else v(0.,1.),turbo=v(0.,p['turbo_allowed']))
                m.floats(state+4,[kw['throttle']]);m.floats(state+0x18,[kw['omega']]);m.floats(state+0x30,[ns['regulator'],ns['turbo']])
                m.floats(state+0xb0,[kw['turbo_command']]);m.u.mem_write(state+0xbc,bytes([kw['automatic_turbo']]))
                m.u.mem_write(state+0xa4,struct.pack('<I',kw['old_gear']));m.u.mem_write(state+0xac,bytes([kw['afterburner']]))
                m.floats(fm+0x531c,[kw['nitro']]);m.qword(FRAME+8,dummy);m.qword(FRAME+16,out)
                for j,value in enumerate([kw['dt'],0.,kw['inlet'],100.]):m.xmm(j,[value])
                m.run(0x1019f4fd0,[fm,dummy,0,0xffffffff if kw['gear'] is None else kw['gear'],address,state])
                actual=dict(multiplier=m.read_xmm(0)[0],gear=m.read(state+0xa4,1,'I')[0],regulator=m.read(state+0x30,1)[0],
                    throttle_ratio=m.read(state+0x2c,1)[0],potential_manifold=m.read(out+8,1)[0],manifold=m.read(out+12,1)[0])
                if p['compressor_type']==3:actual.update(turbo=m.read(state+0x34,1)[0],turbo_command=m.read(state+0xb0,1)[0])
                try:expected=step(p,**dict(kw,**ps))
                except Exception as error:failures.append(dict(aircraft=name,type_id=index,case=i,error=repr(error)));break
                cases+=1
                if actual!=expected:
                    failures.append(dict(aircraft=name,type_id=index,case=i,properties=p,inputs=kw,state=ps,
                        diff={k:[actual[k],expected[k]] for k in actual if actual[k]!=expected[k]}));break
                ns=dict(regulator=actual['regulator'],turbo=actual.get('turbo',ns['turbo']))
                ps=dict(regulator=expected['regulator'],turbo=expected.get('turbo',ps['turbo']))
        reports.append(dict(aircraft=name,steps=cases))
        print(name,cases,'FAIL' if any(f['aircraft']==name for f in failures) else 'PASS',flush=True)
    report=dict(status='FAIL' if failures else 'PASS',aircraft=reports,steps=sum(r['steps'] for r in reports),failures=failures,
        scope='All installed piston types; original configuration and complete compressor consumer. Random and carried regulator/turbo state; manual and automatic stage requests, healthy turbo, positive RPM including startup corrections, boost with and without consumable. Adequate mixture; no assisted discrete bypass or whole-engine claim.',binary_sha256=m.sha)
    (ROOT/'analysis/prop-integration/compressor-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(report['status'],report['steps'],'steps',len(failures),'failures')
    if failures:raise SystemExit(1)


if __name__=='__main__':main()
