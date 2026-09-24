"""Isolated original transmission aggregation with controlled consumer returns.

The complete 101a10db0 executes; engine/propeller outputs and RPM target are
test inputs supplied at their call boundaries. This identifies aggregation,
reduction and inertia semantics; it does not validate those consumers or a
coupled aircraft. Healthy, connected links; zero friction and no damage.
"""
import json
import random
import struct
from pathlib import Path
from unicorn import UC_HOOK_CODE
from unicorn.x86_const import UC_X86_REG_RDI
from prop_config_native import PropConfigNative
from component_assembly import f32,add,sub,mul
from piston_model import div


class AggregationProbe(PropConfigNative):
    def __init__(self):
        super().__init__()
        for a in [0x1019f4e30,0x1019fa1b0,0x101a08de0]:
            self.u.hook_add(UC_HOOK_CODE,self.consumer,begin=a,end=a)
        self.tp=self.alloc(0xc8);self.state=self.alloc(0x100);self.context=self.alloc(0x100)
        self.engine_list=self.alloc(32);self.prop_list=self.alloc(32)
        self.ground_list=self.alloc(64);self.engine_args=self.alloc(32)
        self.engine=[];self.prop=[]
        for i in range(4):
            e=self.alloc(0x300);ep=self.alloc(0x770);p=self.alloc(0x100);pi=self.alloc(0x40);pp=self.alloc(0x370)
            self.qword(e,ep);self.qword(p,pi);self.qword(p+8,pp)
            self.qword(self.engine_list+8*i,e);self.qword(self.prop_list+8*i,p)
            self.floats(pi+4,[1.]);self.floats(ep+0x154,[1000.])
            self.u.mem_write(e+0x1c,b'\x07');self.floats(e+0x30,[1.])
            self.engine.append((e,ep));self.prop.append(p)
        self.qword(self.state,self.tp)
        self.qword(self.context+0x60,self.engine_list);self.qword(self.context+0x70,self.prop_list)
        self.qword(self.context+0x80,self.engine_args);self.qword(self.context+0x90,self.ground_list)

    def consumer(self,u,a,n,data):
        self.services[hex(a)]+=1
        if a==0x1019f4e30:self.xmm(0,[300.]);self.ret()
        elif a==0x1019fa1b0:self.ret()
        else:self.ret(u.reg_read(UC_X86_REG_RDI)+0x60)

    def evaluate(self,engines,props,auto_inertia,correct_link,acceleration=1.,omega=100.,dt=1/48):
        def ui(a,v):self.u.mem_write(a,struct.pack('<I',v))
        self.u.mem_write(self.tp,bytes(0xc8));self.u.mem_write(self.state+8,bytes(0xf8))
        self.floats(self.state+8,[omega,omega,1.,0.,1.])
        ui(self.tp,len(engines));ui(self.tp+0x38,len(props))
        self.u.mem_write(self.tp+0xb0,bytes([auto_inertia]));self.floats(self.tp+0xb4,[acceleration])
        self.u.mem_write(self.tp+0xb8,bytes([correct_link]))
        for i,(ratio,torque,inertia) in enumerate(engines):
            e,ep=self.engine[i];ui(self.tp+4+12*i,i)
            self.floats(self.tp+8+12*i,[ratio,div(1.,ratio)])
            self.floats(e+0x120,[torque,0.]);self.floats(ep+0x16c,[inertia])
        for i,(ratio,torque,inertia) in enumerate(props):
            ui(self.tp+0x3c+28*i,i);self.floats(self.tp+0x40+28*i,[ratio,div(1.,ratio)])
            self.u.mem_write(self.prop[i]+0x60,bytes(160))
            self.floats(self.prop[i]+0x60+18*4,[torque]);self.floats(self.prop[i]+0x60+30*4,[inertia])
        self.xmm(0,[dt]);self.run(0x101a10db0,[self.state,self.context,0,0])
        return self.read(self.state+8,1)[0]


def reconstructed(engines,props,auto_inertia,correct_link,acceleration=1.,omega=100.,dt=1/48):
    torque=0.;load=0.;prop_inertia=0.;engine_inertia=0.
    for ratio,t,i in engines:
        torque=add(torque,mul(t,ratio))
        # 101a118d6..101a11914 replaces this register for each connected
        # engine. It is not an accumulated sum in this pinned executable.
        engine_inertia=mul(i,mul(ratio,ratio) if correct_link else ratio)
    for ratio,t,i in props:
        load=add(load,mul(t,ratio))
        if auto_inertia:i=mul(i,mul(ratio,ratio) if correct_link else ratio)
        prop_inertia=add(prop_inertia,i)
    accel=mul(sub(torque,load),div(1.,add(engine_inertia,prop_inertia)))
    if not auto_inertia:accel=mul(accel,acceleration)
    limit=min(mul(1000.,div(1.,e[0])) for e in engines)
    return min(max(add(omega,mul(accel,dt)),0.),add(limit,limit))


def main():
    m=AggregationProbe();rng=random.Random(0x101a10db0);failures=[];examples=[]
    for e in [[(1.,100.,10.),(1.,300.,30.)],[(1.,300.,30.),(1.,100.,10.)]]:
        p=[(1.,40.,20.)];actual=m.evaluate(e,p,True,True,dt=.125)
        examples.append(dict(engines=e,props=p,omega_before=100.,dt=.125,omega_after=actual,
                             reconstructed=reconstructed(e,p,True,True,dt=.125)))
    cases=0
    def v(lo,hi):return f32(rng.uniform(lo,hi))
    for i in range(1000):
        engines=[(v(.3,1.5),v(-500,4000),v(1,200)) for _ in range(1+i%4)]
        props=[(v(.05,.9),v(-500,4000),v(1,400)) for _ in range(1+(i//4)%4)]
        kw=dict(auto_inertia=bool((i//16)%2),correct_link=bool((i//32)%2),acceleration=v(.5,8.),omega=v(50,400),dt=f32(1/48))
        actual=m.evaluate(engines,props,**kw);expected=reconstructed(engines,props,**kw);cases+=1
        if actual!=expected:
            failures.append(dict(case=i,engines=engines,props=props,inputs=kw,native=actual,python=expected));break
    report=dict(status='FAIL' if failures else 'PASS',binary_sha256=m.sha,scope=__doc__,
                comparisons=cases,examples=examples,failures=failures)
    Path('analysis/transmission-aggregation-research.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    if failures:raise SystemExit(1)

if __name__=='__main__':main()
