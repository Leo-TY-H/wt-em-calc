"""Complete original engine/propeller/transmission compared to independent port."""
import json,random,math
from pathlib import Path
from component_assembly import f32
from propulsion_model import prepare
from prop_owner import owner_step as step
from prop_owner_native import PropOwnerNative

def main():
    m=PropOwnerNative();rng=random.Random(300420);counts={};failures=[];hooks=set()
    def val(a,b):return f32(rng.uniform(a,b))
    def check(n,i,a,e):
        counts[n]=counts.get(n,0)+1;hooks.update(m.calls)
        if a!=e:failures.append(dict(aircraft=n,case=i,diff={k:dict(actual=a[k],expected=e[k]) for k in e if a[k]!=e[k]}))
    for n in ['yak-3','bf-109f-4']:
        p=prepare(json.loads(Path('references/fm-2.59.0.13/'+n+'.blkx').read_text()))
        for i in range(1000):
            s=dict(omega=val(160,320),previous_omega=val(160,320),command=val(0,1),auto=bool(i%2) if n=='bf-109f-4' else False,
                   prop=dict(pitch=val(p['prop']['pitch_min'],p['prop']['pitch_max']),governor_pitch=val(p['prop']['pitch_min'],p['prop']['pitch_max']),flow=[val(0,50),0.,val(0,20)]),
                   engine=dict(throttle=val(.6,1. if n=='yak-3' else 1.1),afterburner=bool(i%3) if n=='bf-109f-4' else False,mixture=.25,gear=i%len(p['engine']['stages']),regulator=val(.1,1.),mechanical=1.))
            kw=dict(velocity=[val(30,220),val(-20,20),val(-20,20)],body_omega=[val(-.4,.4) for _ in range(3)],height=val(0,10000),cg=[val(-.2,.2) for _ in range(3)],dt=f32(rng.choice([1/30,1/60,1/120])),seed=rng.getrandbits(32),torque_gyro=bool(i%4))
            a=m.owner(p,s,**kw);e=step(p,s,**kw);check(n,i,a,e)
            if failures:break
        if failures:break
        ns=ps=dict(omega=270.,prop=dict(pitch=.5),engine=dict(throttle=1.,mixture=.3),command=1.,auto=n=='bf-109f-4')
        nseed=pseed=12345
        for i in range(1200):
            kw=dict(velocity=[f32(80.+40*math.sin(i*.001)),0.,0.],height=3000.,dt=f32(1/60))
            a=m.owner(p,ns,seed=nseed,**kw);e=step(p,ps,seed=pseed,**kw);check(n+'_chained',i,a,e)
            if failures:break
            ns=dict(ns,**{k:a[k] for k in ['omega','previous_omega','prop']},engine=dict(ns['engine'],**a['engine']));nseed=a['engine']['seed']
            ps=dict(ps,**{k:e[k] for k in ['omega','previous_omega','prop']},engine=dict(ps['engine'],**e['engine']));pseed=e['engine']['seed']
        if failures:break
    report=dict(status='FAIL' if failures else 'PASS',binary_sha256=m.sha,counts=counts,substituted_calls=sorted(hooks),failures=failures[:2],
                scope='Complete owner101a155c0 and original101a10db0 return with original101a08de0 propeller,1019fa1b0 engine outer wrapper,1019f3b80 piston and1019f3290 running lifecycle. Prepared intact connected one-engine/one-propeller drivetrain. Thermal/fuel bookkeeping/damage bypassed for frozen fuel/health. All aggregate force, moment, angular momentum, wake, RPM, blade/governor/wake and engine fields compared; independent chained shaft, governor, regulator, modulation and random-stream histories. Includes original double owner force/moment/H aggregation and signed swirl; standalone owner test.')
    Path('analysis/prop-owner-validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
    if failures:raise SystemExit(1)

if __name__=='__main__':main()
