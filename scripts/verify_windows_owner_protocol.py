"""Original Windows look-ahead count cap and controller/physics call protocol.

Tick/clock inputs are prescribed, not recovered network/spawn state. Loop child
calls are recording doubles; their arithmetic and physics are not tested here.
The separate moving-controller and owner-restore tests execute those kernels.
"""
import argparse
import json
import math
from pathlib import Path
import random
import struct

from unicorn import UC_HOOK_CODE
from unicorn.x86_const import *
from windows_instructor_controller import WindowsInstructorController,ACTOR
from instructor_native import BASE,ARENA
from instructor_owner import prediction_count,prediction_calls
from component_assembly import f32,add,mul


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--binary',type=Path,required=True)
    p.add_argument('--out',type=Path,default=Path('analysis/windows-instructor/windows-owner-protocol-validation.json'))
    a=p.parse_args();n=WindowsInstructorController(a.binary);u=n.u
    frame=ARENA+0xb0000;rng=random.Random(9262026);failures=[];events=[];capped=0
    n.qword(ACTOR+0x2ef0,BASE)
    def run(start,end):
        u.reg_write(UC_X86_REG_RSP,frame)
        u.emu_start(start,end,count=100000)
        if u.reg_read(UC_X86_REG_RIP)!=end:raise RuntimeError('Owner span did not finish')
    for case in range(2000):
        dt=f32(rng.choice([1/30,1/48,1/60,1/120,1/240]));tick=rng.randrange(100,100000)
        now=tick+rng.randrange(-5,300);previous=f32(rng.choice([0.,dt,2*dt,rng.uniform(0,.7)]))
        scale=f32(rng.uniform(.1,3.));frequency=f32(1./dt)
        threshold=math.ceil(mul(f32(.7),frequency));u.mem_write(BASE+0x15a8,struct.pack('<i',tick))
        u.reg_write(UC_X86_REG_RBX,BASE);u.reg_write(UC_X86_REG_R12,now);u.reg_write(UC_X86_REG_RBP,threshold)
        n.xmm(8,[frequency]);n.xmm(7,[mul(previous,frequency)]);n.xmm(9,[max(scale,1.)])
        run(0x140e5399f,0x140e53a0c)
        actual=n.read(frame+0x100,1,'Q')[0]
        expected=prediction_count(world_time=dt*(now+.25),physics_dt=dt,current_tick=tick,
            previous_interval=previous,time_scale=scale)['count']
        capped+=max(now-tick,0)>=threshold
        if actual!=expected:failures.append(dict(stage='count',case=case,actual=actual,expected=expected))
    pilot=[]
    def record(u,address,size,data):
        if address==0x1430b01e0:
            assert u.reg_read(UC_X86_REG_R8)==u.reg_read(UC_X86_REG_R9)==0
            assert u.mem_read(u.reg_read(UC_X86_REG_RSP)+0x28,1)==b'\0'
            events.append(dict(stage='select_snapshot',tick=u.reg_read(UC_X86_REG_EDX)))
            n.floats(BASE+0x8514,[9.,9.,9.])
        elif address==0x1430933b0:
            assert n.read(BASE+0x8514)==pilot
            assert u.reg_read(UC_X86_REG_RCX)==ACTOR+0x3088
            inp=u.reg_read(UC_X86_REG_RDX)
            events.append(dict(stage='controller',dt=n.read_xmm(2)[0],dispatch=bool(u.mem_read(inp+0x29,1)[0])))
            n.floats(BASE+0x8514,[add(v,f32(.125)) for v in pilot])
        else:
            assert n.read(BASE+0x8514)==[add(v,f32(.125)) for v in pilot]
            assert u.reg_read(UC_X86_REG_R9)==0
            events.append(dict(stage='physics',tick=u.reg_read(UC_X86_REG_EDX),dt=n.read_xmm(2)[0],produce_primary=False))
        n.return_call()
    handles=[u.hook_add(UC_HOOK_CODE,record,begin=x,end=x) for x in (0x1430b01e0,0x1430933b0,0x142fe2200)]
    for case in range(120):
        count=case%21;tick=rng.randrange(100,10000);dt=f32(rng.choice([1/48,1/60,1/120]))
        pilot=[f32(rng.uniform(-1,1)) for _ in range(3)];events.clear()
        n.qword(frame+0x100,count);n.floats(BASE+0x4ea8,[dt]);u.mem_write(frame+0x2c80,bytes(0x50))
        for reg,value in ((UC_X86_REG_RSI,ACTOR),(UC_X86_REG_RDI,tick),(UC_X86_REG_RAX,tick+count)):
            u.reg_write(reg,value)
        n.xmm(6,[pilot[2]]);n.xmm(7,[pilot[1]]);n.xmm(8,[pilot[0]])
        run(0x140e53a8f,0x140e53b5d)
        u.reg_write(UC_X86_REG_R14,count);u.reg_write(UC_X86_REG_RSI,ACTOR)
        run(0x140e53e3f,0x140e53e70)
        events.append(dict(stage='final_controller',dt=n.read_xmm(2)[0]))
        expected=prediction_calls(tick,count,dt)
        if events!=expected:failures.append(dict(stage='protocol',case=case,actual=list(events),expected=expected))
    for handle in handles:u.hook_del(handle)
    report=dict(scope=__doc__,windows_sha256=n.windows.sha256,counts=dict(caps=2000,protocols=120),
        capped_cases=capped,failures=failures,global_boundary_validated=False)
    a.out.parent.mkdir(parents=True,exist_ok=True);a.out.write_text(json.dumps(report,indent=2)+'\n')
    print(report['counts'],'failures',len(failures),flush=True)
    if failures:raise SystemExit(1)


if __name__=='__main__':main()
