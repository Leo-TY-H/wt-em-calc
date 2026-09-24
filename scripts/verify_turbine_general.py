"""Complete original turbine wrappers on every fixed-wing prop installation."""
import json,random
from pathlib import Path
from piston_general_native import PistonGeneralNative
from turbine_general import step
from component_assembly import f32

ROOT=Path(__file__).resolve().parents[1]


def main():
    rng=random.Random(195321);failures=[];reports=[]
    def v(a,b):return f32(rng.uniform(a,b))
    for row in json.loads((ROOT/'references/prop-native-config.json').read_text())['aircraft']:
        for description in row['engines']:
            if description['family'] not in (2,5):continue
            name=row['aircraft'];m=PistonGeneralNative();m.configure_engine(json.loads((ROOT/'references/jet-catalog/fm'/(name+'.blkx')).read_text()),description['index'])
            p=m.description;ns=ps={};nseed=pseed=12345;count=0
            for i in range(1000):
                if i<500:ns=ps=dict(mechanical=v(.8,1.1),torque=v(-50.,10000.),friction=0.);nseed=pseed=rng.getrandbits(32)
                controls=dict(throttle=v(.1,1.1),afterburner=bool(i%2),gear=0)
                if i<500 or p['family']==5:controls['omega']=v(.5,1.15)*p['properties']['max_omega']
                if i<500:controls['turbo']=v(.5,1.1)*p['turbine']['max_omega']
                ns=dict(ns,**controls);ps=dict(ps,**controls)
                kw=dict(velocity=[v(0.,250.),v(-20,20),v(-20,20)],height=v(0,12000),cg=[v(-1,1) for _ in range(3)],dt=f32(1/[30,48,60,120][i%4]),nitro=10. if i%2 else 0.)
                a=m.piston(ns,seed=nseed,**kw);e=step(p,ps,seed=pseed,**kw);count+=1
                diff={k:[a[k],e.get(k)] for k in a if a[k]!=e.get(k)}
                if diff:failures.append(dict(aircraft=name,engine=description['index'],case=i,inputs=kw,state=ps,seed=pseed,diff=diff));break
                ns=dict(ns,**a);ps=dict(ps,**e);nseed=a['seed'];pseed=e['seed']
            reports.append(dict(aircraft=name,engine=description['index'],steps=count));print(name,description['index'],count,'FAIL' if failures and failures[-1]['aircraft']==name else 'PASS',flush=True)
    report=dict(status='FAIL' if failures else 'PASS',rows=reports,failures=failures,steps=sum(r['steps'] for r in reports),binary_sha256=m.sha,
        scope='Full original 1019f3b80 turbine wrapper after actual complete property loading; installed nozzles and both turbine/shaft speeds. Random and independently carried state/seed, healthy running ample fuel, neutral nozzle controls. No transmission or aircraft equilibrium claim.')
    (ROOT/'analysis/prop-integration/turbine-validation.json').write_text(json.dumps(report,indent=2)+'\n');print(report['status'],report['steps'],'steps',len(failures),'failures',flush=True)
    if failures:raise SystemExit(1)


if __name__=='__main__':main()
