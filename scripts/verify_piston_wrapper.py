"""Complete original running piston wrapper versus independent composition."""
import json,random
from pathlib import Path
from component_assembly import f32
from piston_native import PistonNative
from piston_wrapper import properties,step

def main():
    m=PistonNative();r=random.Random(300419);counts={};failures=[];hooks=set()
    def val(a,b):return f32(r.uniform(a,b))
    for n in ['yak-3','bf-109f-4']:
        p=properties(json.loads(Path('references/fm-2.59.0.13/'+n+'.blkx').read_text()));m.configure_piston(p)
        for i in range(1200):
            s=dict(omega=val(150,340),throttle=val(.55,1.1),mixture=val(.1,1.),gear=i%len(p['stages']),regulator=-1. if i%3 else val(.1,1.),mechanical=val(.8,1.1),extra_amplitude=0.,afterburner=bool(i%2))
            kw=dict(velocity=[val(-50,250),0.,0.],height=val(0,12000),dt=f32(r.choice([1/30,1/60,1/120])),seed=r.getrandbits(32),torque_multiplier=val(.8,1.1))
            a=m.piston(s,**kw);e=step(p,s,**kw);counts[n]=counts.get(n,0)+1;hooks.update(m.calls)
            if a!=e:
                failures.append(dict(aircraft=n,case=i,state=s,inputs=kw,diff={k:[a[k],e[k]] for k in e if a[k]!=e[k]}));break
        if failures:break
        ns=ps=dict(omega=270.,throttle=1.,mixture=.5);nseed=pseed=12345
        for i in range(600):
            controls=dict(omega=f32(230.+i*.08),throttle=f32(1. if i<300 else 1.1),mixture=.3,afterburner=i>=300)
            ns.update(controls);ps.update(controls)
            kw=dict(velocity=[120.,0.,0.],height=f32(i*12.),dt=f32(1/60))
            a=m.piston(ns,seed=nseed,**kw);e=step(p,ps,seed=pseed,**kw);counts[n+'_chained']=counts.get(n+'_chained',0)+1;hooks.update(m.calls)
            if a!=e:
                failures.append(dict(aircraft=n,case=i,diff={k:[a[k],e[k]] for k in e if a[k]!=e[k]}));break
            ns.update(a);ps.update(e);nseed=a['seed'];pseed=e['seed']
        if failures:break
    report=dict(status='FAIL' if failures else 'PASS',binary_sha256=m.sha,counts=counts,substituted_calls=sorted(hooks),failures=failures,
                scope='Complete original1019f3b80 return with inline engine, original compressor, mixture, torque, fuel availability, mechanical modulation and consumption. Prepared intact running engine and inlet; ample prescribed fuel. RPM >=150rad/s, torque multiplier explicit. Chained tests independently propagate compressor regulator, selected gear, random stream and mechanical multiplier. Outer wrapper/transmission not covered here.')
    Path('analysis/piston-wrapper-validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
    if failures:raise SystemExit(1)

if __name__=='__main__':main()
