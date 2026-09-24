"""Full original propulsion owner versus independent installation graph."""
import argparse,copy,json,random
from pathlib import Path
from component_assembly import f32
from propulsion_general_native import PropulsionGeneralNative
from propulsion_general import step

ROOT=Path(__file__).resolve().parents[1]


def differences(a,e,path=''):
    if isinstance(a,dict):
        return {k:v for key,value in a.items() for k,v in differences(value,e.get(key),path+'.'+key).items()}
    if isinstance(a,list) and isinstance(e,list) and len(a)==len(e):
        return {k:v for i,(x,y) in enumerate(zip(a,e)) for k,v in differences(x,y,path+'['+str(i)+']').items()}
    return {} if a==e else {path:[a,e]}


def main():
    ap=argparse.ArgumentParser();ap.add_argument('names',nargs='*');ap.add_argument('--cases',type=int,default=100)
    args=ap.parse_args();rng=random.Random(101155);reports=[];failures=[]
    census=json.loads((ROOT/'references/prop-native-config.json').read_text())['aircraft']
    for row in census:
        name=row['aircraft']
        if args.names and name not in args.names:continue
        m=PropulsionGeneralNative();m.configure(json.loads((ROOT/'references/jet-catalog/fm'/(name+'.blkx')).read_text()));p=m.config
        def v(a,b):return f32(rng.uniform(a,b))
        def fresh():
            return dict(engines=[dict(throttle=v(.7,1.1),mixture=.3,gear=rng.randrange(len(e['properties']['stages'])),
                    afterburner=True,mechanical=v(.9,1.1),automatic_turbo=True,omega=e['properties']['max_omega'],
                    turbo=e['properties']['turbo_max']*.8) for e in p['engines']],
                propellers=[dict(command=v(0.,1.),auto=bool(rng.randrange(2)) and pp['automatic'],
                    pitch=v(pp['pitch_min'],pp['pitch_max']),flow=[v(-5,30),v(-5,30) if pp['coaxial'] else 0.,v(0,15)]) for pp in p['propellers']],
                transmissions=[dict(omega=v(170.,300.)) for _ in p['transmissions']])
        ns=ps=fresh();nseed=pseed=12345;count=0
        for i in range(args.cases):
            kw=dict(velocity=[v(30,200),v(-10,10),v(-10,10)],height=v(0,9000),body_omega=[v(-.2,.2) for _ in range(3)],
                cg=[v(-.5,.5) for _ in range(3)],dt=f32(1/[30,48,60,120][i%4]),nitro=10. if i%2 else 0.,torque_gyro=bool(i%3))
            if i<args.cases//2:ns=ps=fresh();nseed=pseed=rng.getrandbits(32)
            actual=m.step(ns,seed=nseed,**kw)
            try:expected=step(p,ps,seed=pseed,**kw)
            except Exception as error:failures.append(dict(aircraft=name,case=i,error=repr(error)));break
            count+=1;diff=differences(actual,expected)
            if diff:
                failures.append(dict(aircraft=name,case=i,inputs=kw,state=ps,seed=pseed,diff=diff));break
            ns=dict(transmissions=actual['transmissions'],engines=[dict(s,**r) for s,r in zip(ns['engines'],actual['engines'])],
                propellers=[dict(s,**r) for s,r in zip(ns['propellers'],actual['propellers'])])
            ps=expected;nseed=actual['seed'];pseed=expected['seed']
        reports.append(dict(aircraft=name,steps=count));print(name,count,'FAIL' if failures and failures[-1]['aircraft']==name else 'PASS',flush=True)
    report=dict(status='FAIL' if failures else 'PASS',aircraft=reports,steps=sum(r['steps'] for r in reports),failures=failures,
        binary_sha256=m.sha,scope='Actual native property loading, complete original 101a155c0 owner/transmissions/piston/propeller consumers. Independently carried states and seed; intact connected airborne graph with fixed positive-load fuel and thermal state. No aircraft equilibrium or control optimization claim.')
    (ROOT/'analysis/prop-integration/propulsion-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(report['status'],report['steps'],'steps',len(failures),'failures',flush=True)
    if failures:raise SystemExit(1)


if __name__=='__main__':main()
