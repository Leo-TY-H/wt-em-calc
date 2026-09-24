"""Independent non-ExactAltitudes type-1/2 compressor research.

Original complete configuration and compressor consumer. Explicit prescribed
stages, healthy running RPM>=150rad/s; no automatic gear-search validation or
whole engine coupling. Independent regulator, power factor and manifold math.
"""
import json
import math
import random
import struct
from pathlib import Path
from component_assembly import f32,add,sub,mul
from piston_model import div
from control_mixer import curve
from structural_limits import interval
from prop_config_native import PropConfigNative
from verify_aircraft_native import FRAME

ROOT=Path(__file__).resolve().parents[1]

def step(p,omega,throttle,inlet,dt,gear,old_gear,regulator,afterburner,nitro):
    s=p['stages'][gear]
    requested=curve(p['ata'],throttle,1)[0] if p['ata_enabled'] else mul(add(mul(min(throttle,1.),f32(.45)),f32(.55)),p['max_ata'])
    boost=afterburner
    if p['boost_type'] in [1,2,4,5,9]:
        if nitro<f32(.001) or (p['boost_type']==1 and throttle<=1.):boost=False
    boost=boost and mul(s['boost'],p['boost'])>f32(.99)
    pb=p['boost_ata'] if boost else 1.;flow=mul(s['pressure_boost'],inlet) if boost else inlet
    x=mul(p['inverse_omega'],omega)
    speed=add(mul(add(mul(sub(mul(x,x),x),p['speed_sq']),x),sub(1.,p['speed_zero'])),p['speed_zero'])
    critical,flat,ceiling=s['critical'],s['flat'],s['ceiling']
    flat_present=critical<add(flat,f32(-.0001));ceiling_present=add(ceiling,f32(.0001))<critical
    if flat_present or ceiling_present:
        scale=mul(pb,div(1.,speed));flat=mul(flat,scale);ceiling=mul(ceiling,scale);critical=mul(scale,critical)
    baseline=1.
    for _ in range(gear):baseline=mul(mul(baseline,max(s['power'],1.)),f32(.8))
    if flow<=critical:
        if flow<=ceiling:shape=s['ceiling_power'] if ceiling_present else s['power']
        else:shape=add(div(mul(sub(s['ceiling_power'],s['power']),sub(critical,flow)),max(sub(critical,ceiling),f32(.0001))),s['power'])
    elif flat_present:
        if flow<=flat:
            x=interval(flow,critical,1.,flat,0.)
            shape=add(mul(f32(math.pow(x,s['curvature'])),sub(s['power'],s['flat_power'])),s['flat_power'])
        else:shape=add(div(mul(sub(s['flat_power'],baseline),sub(1.,flow)),max(sub(1.,flat),f32(.0001))),baseline)
    else:shape=add(div(mul(sub(s['power'],baseline),sub(1.,flow)),max(sub(1.,critical),f32(.0001))),baseline)
    score=mul(min(mul(mul(flow,speed),div(p['max_ata'],mul(p['reference'],s['critical']))),mul(p['max_ata'],pb)),shape)
    if boost:score=mul(score,mul(s['boost'],p['boost']))
    if score<=-1.:
        # Candidate selection starts at score -1 and index -1, even for an
        # explicit stage request. A rejected request falls back to index0
        # while retaining default shape=-1 and unboosted inlet flow.
        gear=0;s=p['stages'][0];shape=-1.;flow=inlet
        boost=afterburner
        if p['boost_type'] in [1,2,4,5,9]:
            if nitro<f32(.001) or (p['boost_type']==1 and throttle<=1.):boost=False
        boost=boost and mul(s['boost'],p['boost'])>f32(.99)
        pb=p['boost_ata'] if boost else 1.
    potential=mul(mul(speed,flow),div(p['max_ata'],mul(p['reference'],s['critical'])))
    target=div(p['max_ata'],potential)
    if boost:target=mul(target,pb)
    if old_gear==gear and regulator>=0.:
        gain=f32(.1) if p['compressor_type']==2 else mul(dt,3.)
        target=add(mul(sub(target,regulator),gain),regulator)
    regulator=min(max(target,0.),1.);throttle_ratio=min(div(requested,p['max_ata']),1.)
    manifold=mul(mul(potential,throttle_ratio),regulator)
    factor=mul(div(manifold,mul(pb,requested)),shape)
    return dict(multiplier=factor,gear=gear,regulator=regulator,throttle_ratio=throttle_ratio,
                potential_manifold=potential,manifold=manifold)

def main():
    census=json.loads((ROOT/'references/prop-native-config.json').read_text())
    match=lambda e:e['family']<2 and e['compressor'] in [1,2] and not e['exact_altitudes']
    rows=[r for r in census['aircraft'] if any(match(e) for e in r['engines'])]
    reports=[];failures=[];rng=random.Random(0x1019f6111)
    for row in rows:
        name=row['aircraft'];m=PropConfigNative()
        a=m.load_config(json.loads((ROOT/'references/jet-catalog/fm'/(name+'.blkx')).read_text()))
        engine=next(e for e in m.describe(a)['engines'] if match(e));t=a+8+engine['type_id']*0x770
        p={k:m.read(t+o,1)[0] for k,o in dict(inverse_omega=0x15c,reference=0x164,max_ata=0x1bc,
             speed_sq=0x1b8,speed_zero=0x1c8,boost_ata=0x1c4,boost=0x1ec).items()}
        p['boost_type']=engine['boost_type'];p['compressor_type']=engine['compressor']
        p['stages']=[{k:m.read(t+o+4*i,1)[0] for k,o in dict(critical=0x20,power=0x40,boost=0x60,flat=0x80,
                      flat_power=0xa0,curvature=0xc0,ceiling=0xe0,ceiling_power=0x100,pressure_boost=0x120).items()} for i in range(engine['compressor_stages'])]
        p['ata_enabled']=bool(m.u.mem_read(t+0x160,1)[0]);count=m.read(t+0x1b0,1,'I')[0];ptr=m.read(t+0x1a0,1,'Q')[0]
        p['ata']=[(m.read(ptr+12*i,1)[0],m.read(ptr+12*i+4,1)[0],m.read(ptr+12*i+8,1)) for i in range(count)]
        fm=m.alloc(0x8500);state=m.alloc(0x100);out=m.alloc(0x40);dummy=m.alloc(0x80)
        m.u.mem_write(fm+0x8470,b'\x01');m.u.mem_write(state+0xc,b'\x07');m.floats(state+0x9c,[1.])
        cases=0;fallbacks=0;ns=ps=-1.
        for i in range(300):
            def v(lo,hi):return f32(rng.uniform(lo,hi))
            kw=dict(omega=v(150.,450.),throttle=v(.05,1.1),inlet=v(.05,1.3),dt=f32(1/48),gear=i%len(p['stages']),
                    old_gear=(i//2)%len(p['stages']),afterburner=bool(i%2),nitro=0. if i%3==0 else 10.)
            if i<200:ns=ps=-1. if i%4==0 else v(0.,1.)
            m.floats(state+4,[kw['throttle']]);m.floats(state+0x18,[kw['omega']]);m.floats(state+0x30,[ns])
            m.u.mem_write(state+0xa4,struct.pack('<I',kw['old_gear']));m.u.mem_write(state+0xac,bytes([kw['afterburner']]))
            m.floats(fm+0x531c,[kw['nitro']]);m.qword(FRAME+8,dummy);m.qword(FRAME+16,out)
            for j,value in enumerate([kw['dt'],1.,kw['inlet'],100.]):m.xmm(j,[value])
            m.run(0x1019f4fd0,[fm,dummy,0,kw['gear'],t,state])
            actual=dict(multiplier=m.read_xmm(0)[0],gear=m.read(state+0xa4,1,'I')[0],regulator=m.read(state+0x30,1)[0],
                        throttle_ratio=m.read(state+0x2c,1)[0],potential_manifold=m.read(out+8,1)[0],manifold=m.read(out+12,1)[0])
            expected=step(p,regulator=ps,**kw);cases+=1
            fallbacks+=actual['gear']!=kw['gear']
            if actual!=expected:
                failures.append(dict(aircraft=name,case=i,inputs=kw,old_native=ns,old_python=ps,properties=p,
                                     diff={k:[actual[k],expected[k]] for k in actual if actual[k]!=expected[k]}));break
            ns=actual['regulator'];ps=expected['regulator']
        reports.append(dict(aircraft=name,comparisons=cases,stage_fallbacks=fallbacks))
        if failures:break
        print(name,cases,'PASS',flush=True)
    report=dict(status='FAIL' if failures else 'PASS',binary_sha256=m.sha,scope=__doc__,records_requested=len(rows),
                comparisons=sum(r['comparisons'] for r in reports),aircraft=reports,failures=failures)
    (ROOT/'analysis/compressor-alternate-research.json').write_text(json.dumps(report,indent=2)+'\n')
    print(report['status'],report['comparisons'],failures,flush=True)
    if failures:raise SystemExit(1)

if __name__=='__main__':main()
