"""Automatic/MEC native owner parity and default-mode EM performance regression."""
import argparse,json,random,time
from pathlib import Path
from em_backend import activate
activate()
from prop_steady import initial_state
from propulsion_general import step
from propulsion_general_native import PropulsionGeneralNative
from verify_propulsion_general import differences
from component_assembly import f32
from em_solver import TrimSolver,settings

ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'analysis/prop-performance'

def native(names,cases):
    rng=random.Random(101943);failures=[];records=[]
    for name in names:
        m=PropulsionGeneralNative();m.configure(json.loads((ROOT/'references/jet-catalog/fm'/(name+'.blkx')).read_text()));p=m.config
        count=0
        for control_mode in ['automatic','optimized']:
            s=initial_state(p,[100.,0.,0.],3000.,automatic=[True]*len(p['propellers']) if control_mode=='automatic' else None,engine_control_mode=control_mode)
            for i in range(cases):
                kw=dict(velocity=[f32(rng.uniform(40,200)),f32(rng.uniform(-5,5)),0.],height=f32(rng.uniform(0,12000)),
                    dt=f32(1/[30,48,60,120][i%4]),nitro=10.,body_omega=[0.,.1,-.1])
                # Deliberately rich/lean commands distinguish the mixture branches.
                for e in s['engines']:e['mixture']=[.2,.5,1.][i%3]
                actual=m.step(s,seed=s['seed'],**kw);expected=step(p,s,seed=s['seed'],**kw);count+=1
                diff=differences(actual,expected)
                if diff:failures.append(dict(aircraft=name,mode=control_mode,case=i,differences=diff));break
                s=expected
        records.append(dict(aircraft=name,steps=count));print(name,count,'FAIL' if any(f['aircraft']==name for f in failures) else 'PASS',flush=True)
    result=dict(status='FAIL' if failures else 'PASS',records=records,steps=sum(x['steps'] for x in records),failures=failures,
        scope='Original owner and independently advanced port; automatic and manual compressor/mixture branches, full-real flight model, frozen healthy supply, prescribed flight condition.',binary_sha256=m.sha)
    (OUT/'automatic-native.json').write_text(json.dumps(result,indent=2)+'\n');assert not failures,failures[:1]

def points():
    cases=[('yak-3',360,2,{}),('bf-109f-4',360,2,{}),('a5m4',300,2,{}),('a2d',450,2,{}),
        ('wyvern_s4',450,2,{}),('fr_1_fireball',450,2,{}),('fw_200c_1',330,1.5,{}),('tu_4',450,1.5,{}),
        ('b-17g',350,1.5,{}),('p-47d-28',450,2,dict(altitude_m=6500)),
        ('yak-3',360,2,dict(instructor=True)),('bf-109f-4',360,2,dict(instructor=True))]
    rows=[]
    for name,speed,load,extra in cases:
        s=TrimSolver(name,dict(aircraft=[name],**extra));t=time.monotonic();p=s.solve(speed,load,exhaustive=False);elapsed=time.monotonic()-t
        assert p['valid'],(name,p['reasons'])
        assert p['propulsion']['engine_control_mode']=='automatic'
        assert p['propulsion']['optimization']['evaluated']==0
        assert abs(s.point_value(p)['ps']-p['ps_mps'])<1e-9
        rows.append(dict(p,aircraft=name,settings=dict(extra,engine_control_mode='automatic'),wall_seconds=elapsed))
        print(name,extra,round(elapsed,3),p['ps_mps'],flush=True)
        (OUT/'automatic-points.json').write_text(json.dumps(rows,indent=2)+'\n')
    (OUT/'automatic-em.json').write_text(json.dumps(dict(status='PASS',cases=len(rows),checks=['exact replay','unchanged force/angular/history closure','zero manual search','default automatic']),indent=2)+'\n')

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('names',nargs='*');ap.add_argument('--cases',type=int,default=8);ap.add_argument('--points',action='store_true');args=ap.parse_args()
    if args.points:points()
    else:native(args.names or [r['aircraft'] for r in json.loads((ROOT/'references/prop-native-config.json').read_text())['aircraft']],args.cases)
