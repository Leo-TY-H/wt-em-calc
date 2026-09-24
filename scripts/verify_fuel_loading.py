"""Original complete 101990060 versus independent allocation on prepared tanks."""
import json,random,struct
from pathlib import Path
from component_assembly import f32,add
from collision_native import CollisionNative
from fuel_loading import allocate


def main():
    m=CollisionNative();props=m.alloc(0x400);runtime=m.alloc(0x200);rng=random.Random(1990060)
    cases=0;failures=[]
    for i in range(10000):
        def v(a,b):return f32(rng.uniform(a,b))
        count=rng.randrange(1,17);ns=rng.randrange(1,4)
        tanks=[dict(system=rng.randrange(ns),priority=rng.randrange(4),capacity=v(1,500),external=bool(rng.randrange(2))) for _ in range(count)]
        systems=[]
        for system in range(ns):
            matching=[t for t in tanks if t['system']==system]
            total=external=0.
            for t in matching:
                total=add(total,t['capacity'])
                if t['external']:external=add(external,t['capacity'])
            systems.append(dict(capacity=total,external_capacity=external,reservoir_capacity=v(0,20),priority_count=max([t['priority']+1 for t in matching]+[1])))
        if i%5==0:
            for t in tanks:t['priority']=0
            for p in systems:p['priority_count']=1
        state=dict(tanks=[v(0,t['capacity']) for t in tanks],systems=[[v(0,500),v(0,100),v(0,10)] for _ in systems],present=[rng.random()>.15 for _ in tanks])
        system=rng.randrange(ns);amount=v(-10,systems[system]['capacity']*1.2+1)
        flags=dict(internal=bool(i%3),external=bool(i%4),clamp=bool(i%5))
        m.u.mem_write(props,bytes(0x400));m.u.mem_write(runtime,bytes(0x200))
        m.u.mem_write(props+0x10,struct.pack('<I',count))
        for j,t in enumerate(tanks):
            m.floats(props+0xb0+16*j,[t['capacity']]);m.u.mem_write(props+0xb4+16*j,struct.pack('<IIB',t['system'],t['priority'],t['external']))
            m.floats(runtime+0x5c+8*j,[state['tanks'][j]]);m.u.mem_write(runtime+0x60+8*j,bytes([state['present'][j]]))
        for j,p in enumerate(systems):
            m.floats(props+0x1b0+28*j,[p['capacity'],p['external_capacity'],p['reservoir_capacity']]);m.u.mem_write(props+0x1bc+28*j,struct.pack('<I',p['priority_count']))
            m.floats(runtime+0xdc+12*j,state['systems'][j])
        m.xmm(0,[amount]);m.run(0x101990060,[props,runtime,int(flags['internal']),int(flags['external']),system,int(flags['clamp'])])
        actual=dict(tanks=[m.read(runtime+0x5c+8*j,1)[0] for j in range(count)],systems=[m.read(runtime+0xdc+12*j,3) for j in range(ns)],present=state['present'])
        expected=allocate(tanks,systems,state,system,amount,**flags);cases+=1
        if actual!=expected:
            failures.append(dict(case=i,tanks=tanks,systems=systems,state=state,system=system,amount=amount,flags=flags,actual=actual,expected=expected));break
    report=dict(status='FAIL' if failures else 'PASS',cases=cases,binary_sha256=m.sha,failures=failures,scope=__doc__)
    Path('analysis/prop-integration/fuel-loading-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(report['status'],cases,'cases',len(failures),'failures')
    if failures:raise SystemExit(1)

if __name__=='__main__':main()
