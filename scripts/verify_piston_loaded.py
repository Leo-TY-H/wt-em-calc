"""Original engine property initialization/loading followed by running consumers."""
import json
import random
import struct
from pathlib import Path
from mass_parts_native import MassPartsNative
from collision_native import HEAP
from piston_native import PistonNative
from piston_wrapper import properties, step
from verify_piston_model import PROP
from component_assembly import f32

def load_engine_properties(engine):
    m = MassPartsNative(); address = m.alloc(0x770)
    m.run(0x1019fe1f0, [address, m.allocator])
    m.run(0x1019fe230, [address])
    block = m.block(engine)
    m.run(0x1019fea70, [address, block, 1])
    m.run(0x1019f3010, [address])
    return m, address

def load_properties(fm):
    return load_engine_properties(fm['EngineType0'] if 'EngineType0' in fm else fm['Engine0'])

class LoadedPiston(PistonNative):
    def __init__(self, loader, address):
        super().__init__()
        self.loaded = bytes(loader.u.mem_read(address, 0x770))
        self.u.mem_map(HEAP, 0x2000000)
        self.u.mem_write(HEAP, bytes(loader.u.mem_read(HEAP, loader.cursor-HEAP)))

    def configure_piston(self, p):
        super().configure_piston(p)
        self.u.mem_write(PROP, self.loaded)

def main():
    failures=[]; reports={}; rng=random.Random(0x19fea70)
    def v(lo,hi): return f32(rng.uniform(lo,hi))
    for name in ['yak-3', 'bf-109f-4']:
        fm = json.loads(Path('references/fm-2.59.0.13/'+name+'.blkx').read_text())
        loader,address = load_properties(fm); p=properties(fm)
        m=LoadedPiston(loader,address); m.configure_piston(p)
        prepared=PistonNative(); prepared.configure_piston(p)
        offsets=[0xc,0x14,0x18,0x144,0x148,0x154,0x15c,0x164,0x168,0x16c,
                 0x180,0x184,0x188,0x18c,0x190,0x194,0x198,0x1b8,0x1bc,0x1c0,
                 0x1c4,0x1c8,0x1e4,0x1e8,0x1ec,0x1f8,0x244,0x2a4,0x2a8,0x2ac,0x2b0,0x2b4]
        offsets += [off+4*i for off in range(0x20,0x140,0x20) for i in range(len(p['stages']))]
        offsets += [0x1fc+4*i for i in range(2*len(p['rpm_targets']))]
        diffs=[]
        for off in offsets:
            actual=bytes(m.u.mem_read(PROP+off,4)); expected=bytes(prepared.u.mem_read(PROP+off,4))
            if actual!=expected:diffs.append(dict(offset=hex(off),native=actual.hex(),prepared=expected.hex()))
        if diffs:failures.append(dict(aircraft=name,property_diff=diffs))
        count=0; ns=ps={}; nseed=pseed=12345
        for i in range(1800):
            if i<1200:
                ns=ps=dict(omega=v(150,340),throttle=v(.55,1.1),mixture=v(.1,1.),gear=i%len(p['stages']),regulator=-1. if i%3 else v(.1,1.),mechanical=v(.8,1.1),extra_amplitude=0.,afterburner=bool(i%2))
                nseed=pseed=rng.getrandbits(32)
                kw=dict(velocity=[v(-50,250),0.,0.],height=v(0,12000),dt=f32(rng.choice([1/30,1/48,1/120])),torque_multiplier=v(.8,1.1))
            else:
                if i==1200: ns={}; ps={}; nseed=pseed=12345
                j=i-1200
                controls=dict(omega=f32(230+j*.08),throttle=f32(1. if j<300 else 1.1),mixture=.3,afterburner=j>=300)
                ns.update(controls);ps.update(controls)
                kw=dict(velocity=[120.,0.,0.],height=f32(j*12.),dt=f32(1/48))
            actual=m.piston(ns,seed=nseed,**kw);expected=step(p,ps,seed=pseed,**kw);count+=1
            if actual!=expected:
                failures.append(dict(aircraft=name,case=i,diff={k:[actual[k],expected[k]] for k in expected if actual[k]!=expected[k]}));break
            ns=dict(ns,**actual);ps=dict(ps,**expected);nseed=actual['seed'];pseed=expected['seed']
        reports[name]=dict(scalar_bytes=len(offsets)*4,steps=count,source_getters=len(loader.getter_log),loader_services=dict(loader.services))
    report=dict(status='FAIL' if failures else 'PASS',binary_sha256=m.sha,aircraft=reports,failures=failures,
                scope='Original allocator setup1019fe1f0, constructor1019fe230, complete engine property loader1019fea70 (including table and thermal property creation), finalizer1019f3010 and complete running wrapper1019f3b80. Raw-source/default DataBlock adapters and OS/math services; no prepared Python engine properties supplied to native property loaders. 1200 randomized and 600 independently chained running steps per aircraft. Frozen fuel/health and prescribed running state, RPM>=150rad/s; thermal timestepping and start/stop are not validated.')
    Path('analysis/piston-loaded-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    if failures:raise SystemExit(1)

if __name__ == '__main__':main()
