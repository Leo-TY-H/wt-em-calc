"""Propeller aircraft aero/body execution with prescribed mass and engine state."""
import json,random
from pathlib import Path
from component_assembly import f32
from fm_loader import normalize
from aircraft_model import prepare,evaluate
from verify_aircraft_body_native import AircraftBodyNative

def main():
    m=AircraftBodyNative();r=random.Random(300430);counts={};failures=[];hooks=set()
    def val(a,b):return f32(r.uniform(a,b))
    for n in ['yak-3','bf-109f-4']:
        fm=normalize(json.loads(Path('references/fm-2.59.0.13/'+n+'.blkx').read_text()));model=prepare(fm)
        for i in range(1000):
            mass=dict(mass=val(2500,3100),cog=[val(-.3,.3),val(-.3,.3),0.],inertia=list(map(f32,fm['MomentOfInertia'])))
            v=[val(20,240),val(-100,30),val(-35,35)];w=[val(-.7,.7) for _ in range(3)];cmd=[val(-1,1) for _ in range(3)];h=val(0,12000);dt=f32(r.choice([1/30,1/60,1/120]))
            hist=dict(wing_aoa=[val(-15,50),val(-15,50)],body_angles=[val(-15,50),val(-25,25)],wing_cl=[val(-1,1.5),val(-1,1.5)],spin=val(0,1.))
            kw=dict(flaps=val(0,1),gear=val(0,1),throttle=val(.5,1.1),engine_wash=[val(-20,70),val(-20,20)],torque_gyro=bool(i%4))
            ef=[val(-500,6500),0.,0.];em=[val(-6000,6000),val(-1000,1000),val(-1000,1000)];angular=[val(-30000,30000),0.,0.]
            args=[model,v,w,mass,cmd,h,dt,hist]
            a=m.call(*args,**kw);e=evaluate(*args,**kw,oil_radiator=0.,height_agl=h,engine_vectors=(ef,em),engine_spin_factor=kw['throttle'],engine_angular_momentum=angular)
            body=m.extend(ef,em,[0.]*3,[0.]*3,1.,fm.get('ExtThrustBaseMult',1.),dt,engine_angular_momentum=angular)
            comparisons=dict(forces=(a['forces'],{k:e['component_forces'][k] for k in a['forces']}),points=(a['points'],{k:e['component_points'][k] for k in a['points']}),history=(a['history'],e['history']),raw_moment=(a['moment'],e['raw_aero_moment']),force=(body['force'],e['force']),moment=(body['moment'],e['stored_moment']))
            diff={k:dict(actual=x,expected=y) for k,(x,y) in comparisons.items() if x!=y};counts[n]=counts.get(n,0)+1;hooks.update(m.calls)
            if diff:failures.append(dict(aircraft=n,case=i,inputs=kw,diff=diff));break
        if failures:break
    report=dict(status='FAIL' if failures else 'PASS',binary_sha256=m.sha,counts=counts,substituted_calls=sorted(hooks),failures=failures,
        scope='Original aircraft detailed aero through body force/moment assembly. Modern Yak and normalized legacy Bf. Native wing, controls, tail, downwash, axial propwash and signed swirl, spin, helpers, gyroscopic reaction, gravity/force/moment caps. Prepared mass/CG/inertia and engine/wake/H are test inputs; Bf AdvancedMass asset producer is not replaced with a jet mass model. No live-game capture.')
    Path('analysis/prop-aircraft-validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
    if failures:raise SystemExit(1)

if __name__=='__main__':main()
