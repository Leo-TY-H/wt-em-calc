"""Original owner arithmetic, dispatch gates and call-order protocol.

Loop child calls are explicit recording doubles: these check their invocation
order/arguments, not aircraft physics. Whole controller/physics kernel parity
has separate reports. No whole-owner or network initialization claim.
"""
import json,random,struct
from pathlib import Path
from unicorn import UC_HOOK_CODE
from unicorn.x86_const import *
from instructor_native import InstructorNative,BASE,ARENA,STACK
from instructor_owner import prediction_count,recovery_suppressed,dispatch_enabled,prediction_calls,produces_primary_snapshot
from component_assembly import f32,add


def main():
    n=InstructorNative();u=n.u;rng=random.Random(9202631);failures=[]
    actor=ARENA+0x80000;world=ARENA+0x90000;difficulty=ARENA+0xa0000;bp=STACK-0x1000
    n.qword(actor+0x2ef0,BASE);n.qword(BASE+0x6ed8,ARENA+0xb0000)
    n.qword(0x107f5fdf8,world);n.qword(0x107f5fda8,difficulty)
    counts=dict(schedules=0,gates=0,protocols=0,publication_gates=0);branches=dict(capped=0,dispatch=0,suppressed=0)
    def i32(p,v):u.mem_write(p,struct.pack('<i',v))
    def run(start,end):
        u.reg_write(UC_X86_REG_RBP,bp);u.reg_write(UC_X86_REG_RSP,bp-0x4000)
        u.emu_start(start,end,count=20000)
        if u.reg_read(UC_X86_REG_RIP)!=end:raise RuntimeError(hex(u.reg_read(UC_X86_REG_RIP)))
    for i in range(2000):
        dt=f32(rng.choice([1/30,1/48,1/60,1/120,1/240]));tick=rng.randrange(100,100000)
        source=dict(world_time=float(dt)*(tick+rng.uniform(-5,300)),physics_dt=dt,current_tick=tick,
                    previous_interval=f32(rng.choice([0.,dt,dt*2,rng.uniform(0,.7)])),time_scale=f32(rng.uniform(.1,3)))
        n.floats(BASE+0x4ea8,[dt,source['previous_interval']]);i32(BASE+0x15a8,tick);n.floats(world+0x1c4,[source['time_scale']])
        u.reg_write(UC_X86_REG_RAX,BASE);u.reg_write(UC_X86_REG_XMM0,int.from_bytes(struct.pack('<d',source['world_time']),'little'))
        run(0x104f6b0da,0x104f6b0f4)
        now_tick=n.read(bp-0x24b0,1,'i')[0]
        u.reg_write(UC_X86_REG_R14,0x107f5fdf8)
        run(0x104f6b10a,0x104f6b14c)
        n.xmm(0,[f32(.7)])
        run(0x104f701d8,0x104f70202)
        # The intervening boolean producer does not alter XMM0.
        run(0x104f70239,0x104f70247)
        threshold=int(n.read(bp-0x2530,1)[0]);u.reg_write(UC_X86_REG_R15,threshold);u.reg_write(UC_X86_REG_R14,BASE)
        run(0x104f703f3,0x104f70466)
        count=n.read(bp-0x24f0,1,'i')[0]
        u.reg_write(UC_X86_REG_R12,actor)
        run(0x104f708b4,0x104f708db)
        actual=dict(count=count,now_tick=now_tick,threshold=threshold,capped=max(now_tick-tick,0)>=threshold,final_dt=n.read_xmm(0)[0])
        expected=prediction_count(**source);counts['schedules']+=1;branches['capped']+=actual['capped']
        if actual!=expected:failures.append(dict(stage='schedule',source=source,actual=actual,expected=expected))
    for i in range(2000):
        source=dict(selector=i%4,difficulty309=bool(i%3),unit_flags108=rng.randrange(8),config332=bool(i%5),config330=bool(i%7),
                    unit4314=bool(i%2),longitudinal_speed=rng.choice([0.,10.,100.,rng.uniform(-50,300)]),speed_threshold=rng.choice([0.,10.,100.]),has_fm6ef0=bool(i%2))
        u.mem_write(actor+0x108,bytes([source['unit_flags108']]));u.mem_write(actor+0x4314,bytes([source['unit4314']]))
        u.mem_write(ARENA+0xb0330,bytes([source['config330'],0,source['config332']]))
        n.qword(BASE+0x6ef0,ARENA+0xc0000 if source['has_fm6ef0'] else 0)
        u.mem_write(difficulty+0x309,bytes([source['difficulty309']]))
        n.doubles(BASE+0x1618,[source['longitudinal_speed']]);n.floats(0x107e49914,[source['speed_threshold']])
        u.reg_write(UC_X86_REG_R12,actor);n.xmm(0,[0.]);run(0x104f70202,0x104f7024d)
        actual_suppression=bool(u.mem_read(bp-0x2587,1)[0])
        expected_suppression=recovery_suppressed(**{k:source[k] for k in ['unit_flags108','config332','config330','unit4314']})
        # Original provider getter runs and returns true; skip only unrelated
        # view-mode UI side effects, without modifying the tested decision.
        u.mem_write(0x107f60844,b'\1');u.reg_write(UC_X86_REG_EDX,source['selector'])
        commands=[f32(rng.uniform(-1,1)) for _ in range(3)]
        n.floats(bp-0x2490,commands[:2]);n.xmm(1,[commands[2]]);n.floats(BASE+0x8514,[9.,9.,9.])
        run(0x104f7070a,0x104f708b4)
        actual=dict(dispatch=bool(u.mem_read(bp-0x2567,1)[0]),suppressed=actual_suppression,commands=n.read(BASE+0x8514))
        expected=dict(dispatch=dispatch_enabled(**source),suppressed=expected_suppression,commands=commands)
        counts['gates']+=1;branches['dispatch']+=actual['dispatch'];branches['suppressed']+=actual_suppression
        if actual!=expected:failures.append(dict(stage='gate',source=source,actual=actual,expected=expected))
    events=[];active=False
    def child(u,address,size,data):
        if not active:return
        if address==0x101a51a10:
            events.append(dict(stage='select_snapshot',tick=u.reg_read(UC_X86_REG_ESI)))
            assert u.reg_read(UC_X86_REG_EDX)==0 and u.reg_read(UC_X86_REG_ECX)==0
            n.floats(BASE+0x8514,[9.,9.,9.])
        elif address==0x101a92670:
            assert n.read(BASE+0x8514)==pilot
            assert u.reg_read(UC_X86_REG_RDI)==actor+0x3088
            inp=u.reg_read(UC_X86_REG_RSI)
            events.append(dict(stage='controller',dt=n.read_xmm(0)[0],dispatch=bool(u.mem_read(inp+0x29,1)[0])))
            n.floats(BASE+0x8514,[add(v,f32(.125)) for v in pilot])
        else:
            assert n.read(BASE+0x8514)==[add(v,f32(.125)) for v in pilot]
            assert u.reg_read(UC_X86_REG_EDX)==0
            events.append(dict(stage='physics',tick=u.reg_read(UC_X86_REG_ESI),dt=n.read_xmm(0)[0],produce_primary=False))
        n.return_call()
    handles=[u.hook_add(UC_HOOK_CODE,child,begin=a,end=a) for a in [0x101a51a10,0x101a92670,0x101a3e630]]
    for i in range(120):
        dt=f32(rng.choice([1/48,1/60,1/120]));tick=rng.randrange(100,10000);count=i%21
        pilot=[f32(rng.uniform(-1,1)) for _ in range(3)];events.clear()
        i32(BASE+0x15a8,tick);n.floats(BASE+0x4ea8,[dt]);n.floats(bp-0x2490,pilot[:2]);n.floats(bp-0x24c0,pilot[2:])
        u.mem_write(bp-0x2590,bytes(0x50));u.reg_write(UC_X86_REG_R12,actor);u.reg_write(UC_X86_REG_EBX,tick+count)
        active=True;run(0x104f70504,0x104f705bc);active=False
        i32(bp-0x24f0,count);u.reg_write(UC_X86_REG_R12,actor);run(0x104f708b4,0x104f708e9)
        events.append(dict(stage='final_controller',dt=n.read_xmm(0)[0]))
        expected=prediction_calls(tick,count,dt);counts['protocols']+=1
        if events!=expected:failures.append(dict(stage='protocol',actual=list(events),expected=expected))
    for h in handles:u.hook_del(h)
    invoked=[]
    def producer(u,address,size,data):
        invoked.append(address);n.return_call()
    handle=u.hook_add(UC_HOOK_CODE,producer,begin=0x101a4b9a0,end=0x101a4b9a0)
    for phase in [False,True]:
        for has_owner in [False,True]:
            for flags in range(8):
                invoked.clear();n.qword(BASE+8,(BASE-actor-0x100)&((1<<64)-1) if has_owner else 0)
                u.mem_write(actor+0x108,bytes([flags]));u.reg_write(UC_X86_REG_RBX,BASE);u.reg_write(UC_X86_REG_R15,1-int(phase))
                run(0x101a3ea7e,0x101a3eab4)
                actual=bool(invoked);expected=produces_primary_snapshot(phase=phase,has_owner=has_owner,owner_flags108=flags)
                counts['publication_gates']+=1
                if actual!=expected:failures.append(dict(stage='publication',phase=phase,has_owner=has_owner,flags=flags,actual=actual,expected=expected))
    u.hook_del(handle)
    report=dict(binary_sha256=n.sha,counts=counts,branches=branches,failure_count=len(failures),failures=failures[:12],
        scope='Original owner instruction spans for source timing arithmetic, prediction count, input9/input29 gates, request restoration and child call protocol; physics-parent phase/owner primary-publication gate. Loop children and eligible primary producer are explicit recording doubles. Final command dt inspected before its call. No whole-owner physics, network clock, spawn, GUI or snapshot-restoration parity claimed.')
    Path('analysis/instructor-full/owner-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(counts,branches,'failures',len(failures),failures[:1])
    if failures:raise SystemExit(1)


if __name__=='__main__':main()
