"""Full native fighter propeller return versus independent update and history."""
import json,random,math
from pathlib import Path
from component_assembly import f32
from propeller_native import PropellerNative,properties
from propeller_step import step


def main():
    m=PropellerNative();rng=random.Random(3004109);counts={};failures=[];errors={};hooks=set()
    def uniform(a,b):return f32(rng.uniform(a,b))
    def check(n,i,a,e):
        counts[n]=counts.get(n,0)+1
        diff={}
        for k in e:
            x=a[k];y=e[k]
            if isinstance(x,list):
                if x and isinstance(x[0],list):x=sum(x,[]);y=sum(y,[])
                error=max(abs(s-t) for s,t in zip(x,y))
            else:error=abs(x-y)
            errors[k]=max(errors.get(k,0.),error)
            if x!=y:diff[k]=dict(actual=x,expected=y)
        if diff:failures.append(dict(aircraft=n,case=i,diff=diff))
        hooks.update(m.calls)
    for n in ['yak-3','bf-109f-4']:
        p=properties(json.loads(Path('references/fm-2.59.0.13/'+n+'.blkx').read_text()));m.configure(p)
        for i in range(1000):
            s=dict(pitch=uniform(p['pitch_min'],p['pitch_max']),governor_pitch=uniform(p['pitch_min']-.3,p['pitch_max']),flow=[uniform(-20,70),0.,uniform(0,20)])
            omega=uniform(20,260)
            kw=dict(velocity=[uniform(-80,250),uniform(-40,40),uniform(-40,40)],body_omega=[uniform(-.7,.7) for _ in range(3)],cg=[uniform(-.5,.5) for _ in range(3)],
                    omega=omega,previous_omega=f32(omega+uniform(-.3,.3)),target_omega=uniform(100,320),command=uniform(0,1),auto=bool(i%2) if n=='bf-109f-4' else False,
                    density=uniform(.2,1.3),sound_speed=uniform(280,345),dt=f32(rng.choice([1/30,1/60,1/120])),afterburner=bool(i%3),torque_gyro=bool(i%4))
            a=m.step(s,**kw);e=step(p,s,**kw);check(n,i,a,e)
            if failures:break
        if failures:break
        ns={};ps={}
        for i in range(500):
            kw=dict(velocity=[f32(50+100*i/500),f32(5*math.sin(i*.04)),0.],omega=f32(170+10*math.sin(i*.03)),previous_omega=f32(170+10*math.sin((i-1)*.03)),
                    command=f32(.6 if i<150 else 1. if i<350 else .3),auto=n=='bf-109f-4',afterburner=i>=150)
            a=m.step(ns,**kw);e=step(p,ps,**kw);check(n+'_chained',i,a,e);ns=a['state'];ps=e
            if failures:break
        if failures:break
    report=dict(status='FAIL' if failures else 'PASS',binary_sha256=m.sha,counts=counts,max_absolute_errors=errors,substituted_calls=sorted(hooks),failures=failures[:3],
                scope='Original complete 101a08de0 through return; intact connected selected noncyclic noncoaxial type-2/type-8 governors with native blade, polar and Mach functions. Prepared properties and running context; ground-screen disabled, shaft RPM prescribed. All 40 output floats, blade call inputs, pitch, governor accumulator and three wake state floats compared exactly. Chained cases propagate independently computed governor and inflow states. Does not validate engine/transmission or aerodynamic coupling.')
    Path('analysis/propeller-step-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    if failures:raise SystemExit(1)


if __name__=='__main__':main()
