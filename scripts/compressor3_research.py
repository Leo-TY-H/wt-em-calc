"""Independent healthy type-3 piston compressor state/manifold research.

Original complete propulsion configuration loading, then original 1019f4fd0
versus independent manifold, throttle and turbo-speed updates. Runtime manual/
automatic gate and turbo health are explicit inputs; damage production and
whole engine/aircraft coupling are outside this test.
"""
import json
import random
from pathlib import Path
from component_assembly import f32,add,sub,mul
from piston_model import div
from control_mixer import curve
from structural_limits import interval
from prop_config_native import PropConfigNative
from verify_aircraft_native import FRAME

ROOT=Path(__file__).resolve().parents[1]

def step(p,omega,throttle,inlet,dt,old_turbo,command,automatic,assisted,afterburner,nitro):
    requested=curve(p['ata'],throttle,1)[0] if p['ata_enabled'] else mul(add(mul(min(throttle,1.),f32(.45)),f32(.55)),p['max_ata'])
    boost=afterburner
    if p['boost_type'] in [1,2,4,5,9]:
        if nitro<f32(.001) or (p['boost_type']==1 and throttle<=1.):boost=False
    if boost and p['stage_boost']>f32(.99):requested=mul(requested,p['boost_ata'])
    x=mul(p['inverse_omega'],omega)
    speed=add(mul(add(mul(sub(mul(x,x),x),p['speed_sq']),x),sub(1.,p['speed_zero'])),p['speed_zero'])
    base=mul(mul(speed,inlet),div(p['max_ata'],mul(p['reference'],p['critical'])))
    upper=interval(omega,p['shaft_min'],p['turbo_min'],p['shaft_max'],p['turbo_max'])
    allowed=p['turbo_allowed']
    if automatic or assisted:
        target=min(mul(div(allowed,base),requested),allowed)
        command=div(target,sub(upper,p['turbo_min']))
    else:target=add(mul(sub(upper,p['turbo_min']),command),p['turbo_min'])
    turbo=target if assisted else add(mul(mul(sub(target,old_turbo),dt),p['inverse_time']),old_turbo)
    if turbo<p['turbo_min']:
        fraction=div(turbo,p['turbo_min']);potential=mul(inlet,.75)
        if fraction>0.:
            top=mul(div(p['turbo_min'],allowed),base)
            potential=add(potential,mul(sub(top,potential),fraction)) if fraction<1. else top
    else:potential=mul(div(turbo,allowed),base)
    throttle_ratio=min(div(requested,potential),1.)
    manifold=mul(throttle_ratio,potential)
    if 20.<=omega<150.:manifold=min(manifold,mul(add(mul(omega,f32(.04)),f32(-.39999995)),inlet))
    elif omega<20.:manifold=mul(add(mul(omega,f32(-.03)),1.),inlet)
    return dict(turbo=turbo,command=command,regulator=1.,throttle_ratio=throttle_ratio,
                potential_manifold=potential,manifold=manifold,gear=0)

def main():
    census=json.loads((ROOT/'references/prop-native-config.json').read_text())
    rows=[x for x in census['aircraft'] if any(e['family']<2 and e['compressor']==3 for e in x['engines'])]
    reports=[];failures=[];rng=random.Random(0x1019f4fd0)
    for row in rows:
        name=row['aircraft'];m=PropConfigNative()
        a=m.load_config(json.loads((ROOT/'references/jet-catalog/fm'/(name+'.blkx')).read_text()))
        engine=next(e for e in m.describe(a)['engines'] if e['family']<2 and e['compressor']==3)
        t=a+8+engine['type_id']*0x770
        p={k:m.read(t+o,1)[0] for k,o in dict(shaft_min=0x140,shaft_max=0x144,inverse_omega=0x15c,
             reference=0x164,critical=0x20,max_ata=0x1bc,speed_sq=0x1b8,speed_zero=0x1c8,
             boost_ata=0x1c4,turbo_min=0x1cc,turbo_max=0x1d0,turbo_allowed=0x1d8,inverse_time=0x1e0).items()}
        p['boost_type']=engine['boost_type'];p['stage_boost']=mul(m.read(t+0x60,1)[0],m.read(t+0x1ec,1)[0])
        p['ata_enabled']=bool(m.u.mem_read(t+0x160,1)[0])
        count=m.read(t+0x1b0,1,'I')[0];ptr=m.read(t+0x1a0,1,'Q')[0]
        p['ata']=[(m.read(ptr+12*i,1)[0],m.read(ptr+12*i+4,1)[0],m.read(ptr+12*i+8,1)) for i in range(count)]
        fm=m.alloc(0x8500);state=m.alloc(0x100);out=m.alloc(0x40);dummy=m.alloc(0x80)
        m.u.mem_write(fm+0x3658,b'\x04');m.u.mem_write(state+0xc,b'\x07');m.floats(state+0x38,[1.])
        cases=0;ns=ps=f32(.7*p['turbo_allowed'])
        for i in range(300):
            def v(lo,hi):return f32(rng.uniform(lo,hi))
            kw=dict(omega=v(0.,1.3*p['shaft_max']),throttle=v(.05,1.1),inlet=v(.2,1.2),dt=f32(1/48),
                    command=v(0.,1.),automatic=bool(i%2),assisted=bool((i//2)%2),afterburner=bool((i//4)%2),nitro=0. if i%3==0 else 10.)
            if i<200:ns=ps=v(0.,p['turbo_allowed'])
            m.floats(state+4,[kw['throttle']]);m.floats(state+0x18,[kw['omega']]);m.floats(state+0x34,[ns])
            m.floats(state+0xb0,[kw['command']]);m.u.mem_write(state+0xbc,bytes([kw['automatic']]))
            m.u.mem_write(state+0x78,bytes([kw['assisted']]));m.u.mem_write(state+0xac,bytes([kw['afterburner']]))
            m.floats(fm+0x531c,[kw['nitro']]);m.qword(FRAME+8,dummy);m.qword(FRAME+16,out)
            for j,value in enumerate([kw['dt'],1.,kw['inlet'],100.]):m.xmm(j,[value])
            m.run(0x1019f4fd0,[fm,dummy,0,0,t,state])
            actual=dict(turbo=m.read(state+0x34,1)[0],command=m.read(state+0xb0,1)[0],regulator=m.read(state+0x30,1)[0],
                        throttle_ratio=m.read(state+0x2c,1)[0],potential_manifold=m.read(out+8,1)[0],manifold=m.read(out+12,1)[0],gear=m.read(state+0xa4,1,'I')[0])
            expected=step(p,old_turbo=ps,**kw);cases+=1
            if actual!=expected:
                failures.append(dict(aircraft=name,case=i,inputs=kw,old_native=ns,old_python=ps,properties=p,
                                     diff={k:[actual[k],expected[k]] for k in actual if actual[k]!=expected[k]}));break
            ns=actual['turbo'];ps=expected['turbo']
        reports.append(dict(aircraft=name,comparisons=cases))
        if failures:break
        print(name,cases,'PASS',flush=True)
    report=dict(status='FAIL' if failures else 'PASS',binary_sha256=m.sha,scope=__doc__,records_requested=len(rows),
                comparisons=sum(r['comparisons'] for r in reports),aircraft=reports,failures=failures)
    (ROOT/'analysis/compressor-type3-research.json').write_text(json.dumps(report,indent=2)+'\n')
    print(report['status'],report['comparisons'],failures,flush=True)
    if failures:raise SystemExit(1)

if __name__=='__main__':main()
