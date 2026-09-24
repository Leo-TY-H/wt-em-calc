"""Compare complete generalized piston wrapper with original configured engine."""
import argparse,json,random
from pathlib import Path
from piston_general_native import PistonGeneralNative
from piston_general import step
from component_assembly import f32

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser();parser.add_argument('names',nargs='*');parser.add_argument('--cases',type=int,default=100)
    args=parser.parse_args();rng=random.Random(1019380);reports=[];failures=[]
    census=json.loads((ROOT/'references/prop-native-config.json').read_text())['aircraft']
    rows=[r for r in census if not args.names or r['aircraft'] in args.names]
    for row in rows:
        name=row['aircraft'];fm=json.loads((ROOT/'references/jet-catalog/fm'/(name+'.blkx')).read_text());cases=0
        for descriptor in row['engines']:
            if descriptor['family']>=2:continue
            m=PistonGeneralNative();m.configure_engine(fm,descriptor['index']);p=m.p
            ns=ps={};nseed=pseed=12345
            for i in range(args.cases):
                def v(lo,hi):return f32(rng.uniform(lo,hi))
                inputs=dict(velocity=[v(20.,250.),0.,0.],height=v(0.,10000.),dt=f32(1/48),nitro=10. if i%3 else 0.)
                controls=dict(omega=v(150.,1.2*p['max_omega']),throttle=v(.55,1.1),mixture=.3,afterburner=bool(i%2),
                    gear=i%len(p['stages']),automatic_turbo=bool(i%2),turbo_command=v(0.,1.))
                if i<args.cases//2:
                    ns=ps=dict(regulator=-1. if i%3 else v(0.,1.),mechanical=v(.8,1.1),reservoir=v(0.,p['reservoir_capacity']),turbo=v(0.,p['turbo_allowed']))
                    nseed=pseed=rng.getrandbits(32)
                elif i==args.cases//2:ns=ps={};nseed=pseed=12345
                ns=dict(ns,**controls);ps=dict(ps,**controls)
                actual=m.piston(ns,seed=nseed,**inputs)
                try:expected=step(p,ps,seed=pseed,**inputs)
                except Exception as error:failures.append(dict(aircraft=name,engine=descriptor['index'],case=i,error=repr(error)));break
                cases+=1
                if actual!=expected:
                    failures.append(dict(aircraft=name,engine=descriptor['index'],case=i,properties=p,inputs=inputs,state=ps,
                        diff={k:[actual[k],expected[k]] for k in actual if actual[k]!=expected[k]}));break
                ns=dict(ns,**actual);ps=dict(ps,**expected);nseed=actual['seed'];pseed=expected['seed']
        if cases:
            reports.append(dict(aircraft=name,steps=cases));print(name,cases,'FAIL' if any(f['aircraft']==name for f in failures) else 'PASS',flush=True)
    report=dict(status='FAIL' if failures else 'PASS',aircraft=reports,steps=sum(r['steps'] for r in reports),failures=failures,
        scope='Original complete configuration and running wrapper; every installed piston instance, random and independently carried compressor/mechanical states. Healthy running, adequate positive-load fixed fuel, closed radiators, manual delivered controls, ordinary shaft RPM. No transmission/propeller/aircraft equilibrium claim.',binary_sha256=m.sha)
    (ROOT/'analysis/prop-integration/piston-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(report['status'],report['steps'],'steps',len(failures),'failures')
    if failures:raise SystemExit(1)


if __name__=='__main__':main()
