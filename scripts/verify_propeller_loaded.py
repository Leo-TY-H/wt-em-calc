"""Close the shared-property-adapter gap with original BLK property loaders.

Constructor101a02db0, modern101a03700 and legacy101a05a40 execute natively.
The JSON DataBlock access adapter delivers source values or native defaults;
no prepared Python propeller constants are fed to the loader.
"""
import json
import random
import struct
from pathlib import Path
from collision_native import HEAP
from mass_parts_native import MassPartsNative
from propeller_native import PropellerNative, PROPS
from propeller_model import properties
from propeller_step import step
from component_assembly import f32

def load_properties(fm):
    m=MassPartsNative(); address=m.alloc(0x500)
    for off in [0x268,0x2e8,0x328]:m.qword(address+off,m.allocator)
    m.qword(address+0x148,address+0x160)
    m.run(0x101a02db0,[address])
    modern='PropellerType0' in fm
    block=m.block(fm['PropellerType0'] if modern else fm['Engine0'])
    m.run(0x101a03700 if modern else 0x101a05a40,[address,block,1])
    return m,address

def install_loaded(native,loader,address):
    native.u.mem_map(HEAP,0x2000000)
    native.u.mem_write(HEAP,bytes(loader.u.mem_read(HEAP,loader.cursor-HEAP)))
    native.u.mem_write(PROPS,bytes(loader.u.mem_read(address,0x370)))

def main():
    rng=random.Random(1990300); failures=[]; reports={}
    scalar_ranges=[(0,0x144),(0x1e0,0x260),(0x278,0x2e0),(0x2f8,0x320),(0x330,0x370)]
    for name in ['yak-3','bf-109f-4']:
        fm=json.loads(Path('references/fm-2.59.0.13/'+name+'.blkx').read_text())
        loader,address=load_properties(fm);native=PropellerNative();p=properties(fm);native.configure(p)
        a=bytes(loader.u.mem_read(address,0x370));b=bytes(native.u.mem_read(PROPS,0x370))
        scalar_diff=[]
        for lo,hi in scalar_ranges:
            for off in range(lo,hi,4):
                if a[off:off+4]!=b[off:off+4]:scalar_diff.append(dict(offset=hex(off),native=a[off:off+4].hex(),python=b[off:off+4].hex()))
        if scalar_diff:failures.append(dict(aircraft=name,scalar_diff=scalar_diff))
        install_loaded(native,loader,address)
        cases=0;ns={};ps={}
        for i in range(1200):
            def v(lo,hi):return f32(rng.uniform(lo,hi))
            omega=v(25,260)
            kw=dict(velocity=[v(-70,240),v(-35,35),v(-35,35)],body_omega=[v(-.5,.5) for _ in range(3)],cg=[v(-.5,.5) for _ in range(3)],omega=omega,previous_omega=f32(omega+v(-.3,.3)),target_omega=v(130,310),command=v(0,1),auto=bool(i%2),density=v(.25,1.3),sound_speed=v(285,345),dt=f32(1/60),afterburner=bool(i%3))
            if i<1000:
                ns=ps=dict(pitch=v(p['pitch_min'],p['pitch_max']),governor_pitch=v(p['pitch_min']-.15,p['pitch_max']),flow=[v(-20,50),0.,v(0,20)])
            if i==1000:ns={};ps={}
            actual=native.step(ns,**kw);expected=step(p,ps,**kw);cases+=1
            diff={k:dict(native=actual[k],python=expected[k]) for k in expected if actual[k]!=expected[k]}
            if diff:failures.append(dict(aircraft=name,case=i,diff=diff));break
            ns=actual['state'];ps=expected
        reports[name]=dict(scalar_bytes=sum(hi-lo for lo,hi in scalar_ranges),steps=cases,getter_count=len(loader.getter_log),native_loader_services=dict(loader.services))
    report=dict(status='FAIL' if failures else 'PASS',binary_sha256=loader.sha,aircraft=reports,failures=failures,
                scope='Original propeller constructor and complete modern/legacy BLK loaders, then complete propeller timestep on those loaded properties. Source-value/default DataBlock adapters, temporary allocation/string formatting and rounded libm; original native curve creation and mesh-independent property math. 1000 random and 200 independently chained steps per aircraft. Engine/transmission instance and pilot/runtime inputs remain prepared; no game file parser or live flight.')
    Path('analysis/propeller-loaded-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    if failures:raise SystemExit(1)

if __name__=='__main__':main()
