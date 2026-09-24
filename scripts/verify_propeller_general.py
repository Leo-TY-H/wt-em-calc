"""Differential full propeller steps after original type/instance loading."""
import argparse,json,random
from pathlib import Path
from unicorn import UC_HOOK_CODE
from prop_config_native import PropConfigNative
from propeller_native import PropellerNative,STATE
from propeller_config import loaded_properties
from propeller_general import step
from component_assembly import f32,mul

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser();parser.add_argument('names',nargs='*')
    parser.add_argument('--cases',type=int,default=100)
    args=parser.parse_args()
    census=json.loads((ROOT/'references/prop-native-config.json').read_text())
    names=args.names or [r['aircraft'] for r in census['aircraft']]
    reports=[];failures=[];rng=random.Random(101080)
    for name in names:
        m=PropConfigNative();a=m.load_config(json.loads((ROOT/'references/jet-catalog/fm'/(name+'.blkx')).read_text()))
        cfg=m.describe(a);cases=0
        # The real propeller constructor initializes the shared legacy polar.
        # Property loading alone does not initialize this process-wide value.
        m.run(0x101a08760,[STATE])
        m.u.hook_add(UC_HOOK_CODE,lambda u,a,n,d:PropellerNative.blade_capture(m,u,a,n,d),begin=0x101a07790,end=0x101a07790)
        for description in cfg['propellers']:
            idx=description['index'];inst=a+0x2491c+idx*0x38;address=a+0x21218+description['type_id']*0x370
            link=next(l for t in cfg['transmissions'] for l in t['propellers'] if l['index']==idx)
            p=loaded_properties(m,address,inst,link['ratio']);m.p=p
            m.qword(STATE,inst);m.qword(STATE+8,address)
            ns={};ps={}
            for i in range(args.cases):
                def v(lo,hi):return f32(rng.uniform(lo,hi))
                speed=v(30,200);omega=mul(p['max_omega'],link['ratio'])
                kw=dict(velocity=[speed,v(-15,15),v(-15,15)],body_omega=[v(-.2,.2) for _ in range(3)],cg=[v(-.5,.5) for _ in range(3)],
                    omega=omega,previous_omega=f32(omega+v(-.3,.3)),target_omega=p['max_omega'],command=v(0,1),
                    auto=bool(i%2) if p['auto_allowed'] else False,density=v(.4,1.3),sound_speed=v(295,345),dt=f32(1/48),afterburner=bool(i%3))
                if i<args.cases//2:ns=ps=dict(pitch=v(p['pitch_min'],p['pitch_max']),governor_pitch=v(p['pitch_min'],p['pitch_max']),flow=[v(-10,30),v(-10,30) if p['coaxial'] else 0.,v(0,20)])
                if i==args.cases//2:ns={};ps={}
                actual=PropellerNative.step(m,ns,**kw)
                try:expected=step(p,ps,**kw)
                except Exception as error:
                    failures.append(dict(aircraft=name,propeller=idx,case=i,error=repr(error)));break
                cases+=1
                diff={k:dict(native=actual[k],python=expected[k]) for k in expected if actual[k]!=expected[k]}
                if diff:
                    failures.append(dict(aircraft=name,propeller=idx,case=i,diff=diff,properties=p,inputs=kw,state=ps));break
                ns=actual['state'];ps=expected
        reports.append(dict(aircraft=name,steps=cases))
        print(name,cases,'FAIL' if any(f['aircraft']==name for f in failures) else 'PASS',flush=True)
    report=dict(status='FAIL' if failures else 'PASS',aircraft=reports,steps=sum(r['steps'] for r in reports),failures=failures,
                scope='Exact full propeller outputs, governor and wake, original configuration loading including actual mounts; random inputs and independently carried states. Healthy connected airborne props; no terrain augmentation. No engine/airframe-equilibrium claim.',binary_sha256=m.sha)
    directory=ROOT/'analysis/prop-integration';directory.mkdir(exist_ok=True)
    (directory/'propeller-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(report['status'],report['steps'],'steps',len(failures),'failures',flush=True)
    if failures:raise SystemExit(1)


if __name__=='__main__':main()
