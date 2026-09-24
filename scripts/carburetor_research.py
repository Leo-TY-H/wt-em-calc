"""Independent type-3 running torque consumer after original property loading.

All catalog records containing a type-3 piston engine. Fuel-reservoir content,
shaft speed and torque multiplier are prescribed, with healthy running state;
fuel production, mixture/compressor and the complete wrapper are not tested.
"""
import json
import random
import struct
from pathlib import Path
from component_assembly import f32,add,sub,mul
from piston_model import div
from prop_config_native import PropConfigNative

ROOT=Path(__file__).resolve().parents[1]

def torque(p,omega,throttle,multiplier,reservoir,afterburner,gear,nitro):
    effective=add(mul(sub(1.,p['min_throttle']),min(p['max_throttle'],throttle)),p['min_throttle'])
    x=div(omega,p['max_omega'])
    shape=sub(mul(x,3.),div(add(mul(x,x),mul(x,x)),effective))
    half=mul(p['capacity'],.5)
    if reservoir<=half:fuel=f32(.6)
    elif reservoir>=p['capacity']:fuel=1.
    else:fuel=add(f32(.6),div(mul(sub(reservoir,half),sub(1.,f32(.6))),sub(p['capacity'],half)))
    value=mul(mul(mul(mul(p['throttle_boost'] if throttle>1. else 1.,multiplier),p['torque_base']),fuel),shape)
    boost=afterburner
    if p['boost_type'] in [1,2,4,5,9]:
        if nitro<f32(.001) or (p['boost_type']==1 and throttle<=1.):boost=False
    stage_boost=mul(p['stages'][gear],p['boost'])
    if boost and stage_boost>f32(.99):value=mul(value,stage_boost)
    if value<0.:
        value=max(value,mul(mul(mul(mul(multiplier,f32(-.8)),omega),p['inverse_omega']),p['torque_base']))
    return value

def main():
    census=json.loads((ROOT/'references/prop-native-config.json').read_text())
    rows=[x for x in census['aircraft'] if any(e['family']<2 and e['carburetor']==3 for e in x['engines'])]
    rng=random.Random(0x1019f6fbc);failures=[];reports=[];m=None
    for row in rows:
        name=row['aircraft'];m=PropConfigNative()
        a=m.load_config(json.loads((ROOT/'references/jet-catalog/fm'/(name+'.blkx')).read_text()))
        engine=next(e for e in m.describe(a)['engines'] if e['family']<2 and e['carburetor']==3)
        t=a+8+engine['type_id']*0x770
        p={k:m.read(t+o,1)[0] for k,o in dict(min_throttle=0x14,max_throttle=0x18,max_omega=0x144,
             inverse_omega=0x15c,torque_base=0x168,throttle_boost=0x1e8,boost=0x1ec,capacity=0x2b8).items()}
        p['boost_type']=engine['boost_type'];p['stages']=m.read(t+0x60,engine['compressor_stages'])
        assert p['capacity']>0.,name
        fm=m.alloc(0x8500);state=m.alloc(0x100);m.u.mem_write(state+0xc,b'\x07')
        cases=0
        for i in range(100):
            def v(lo,hi):return f32(rng.uniform(lo,hi))
            kw=dict(omega=v(5.,2*p['max_omega']),throttle=v(.05,1.1),multiplier=v(.2,1.5),
                    reservoir=v(0.,1.2*p['capacity']),afterburner=bool(i%2),gear=i%len(p['stages']),
                    nitro=0. if i%3==0 else 10.)
            m.floats(state+4,[kw['throttle']]);m.floats(state+0x18,[kw['omega']]);m.floats(state+0x88,[kw['reservoir']])
            m.u.mem_write(state+0xa4,struct.pack('<I',kw['gear']));m.u.mem_write(state+0xac,bytes([kw['afterburner']]))
            m.floats(fm+0x531c,[kw['nitro']]);m.xmm(0,[kw['multiplier']])
            m.run(0x1019f6c60,[fm,0,t,state,0]);actual=m.read_xmm(0)[0]
            expected=torque(p,**kw);cases+=1
            if actual!=expected:
                failures.append(dict(aircraft=name,case=i,inputs=kw,properties=p,native=actual,python=expected));break
        reports.append(dict(aircraft=name,comparisons=cases))
        if failures:break
        if len(reports)%20==0:print(len(reports),'records checked',flush=True)
    report=dict(status='FAIL' if failures else 'PASS',binary_sha256=m.sha if m else None,scope=__doc__,
                records_requested=len(rows),comparisons=sum(r['comparisons'] for r in reports),aircraft=reports,failures=failures)
    (ROOT/'analysis/carburetor-type3-research.json').write_text(json.dumps(report,indent=2)+'\n')
    print(report['status'],len(reports),report['comparisons'],failures,flush=True)
    if failures:raise SystemExit(1)

if __name__=='__main__':main()
