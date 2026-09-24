"""Carry independent controller history through prescribed changing flight inputs.

This is not a flight trajectory: external kinematics/engine fields are varied
explicitly. It tests controller recurrence without importing native histories.
"""
import json,random,math,sys
from pathlib import Path
from instructor_native import InstructorNative,BASE,OWNER,OBJ,INPUT
from instructor_keyboard import keyboard_step
from verify_instructor_keyboard import extract_state,extract_history,extract_result
from aircraft_model import prepare,at_sweep
from jet_catalog import catalog,fuel_capacities
from instructor_source import load
from mass_model import aircraft_properties,evaluate
from component_assembly import f32


def main(names=None):
    n=InstructorNative();rng=random.Random(921260);failures=[];counts=dict(aircraft=0,updates=0,failures=0,active_recovery=0,autotrim_success=0,autotrim_failure=0)
    for name,item in catalog().items():
        if not item['supported'] or names and name not in names:continue
        fm=load(name);base=prepare(fm);mass=evaluate(aircraft_properties(fm),fuel_by_system=[f32(v*.3) for v in fuel_capacities(fm)])
        flaps=f32(rng.random());sweep=f32(rng.random());model=at_sweep(base,sweep)
        n.setup(model,mass,speed=rng.uniform(100.,450.),alpha=5.,flaps=flaps,sweep=sweep,height=rng.uniform(0,15000),mode_lane=True)
        actor=n.empty_payload_actor();n.qword(actor+0x2ef0,BASE)
        n.u.mem_write(INPUT+2,b'\1');n.u.mem_write(INPUT+0x29,b'\1');n.floats(INPUT+0x10,[0.]);n.floats(INPUT+0x44,[1.])
        n.u.mem_write(0x107d6fbc0,b'\1\1');n.u.mem_write(BASE+0x3658,b'\3');n.u.mem_write(0x107d6fbf5,b'\1')
        n.u.mem_write(0x107d6fc2d,bytes([counts['aircraft']%2]))
        n.floats(OBJ+0x154,[rng.uniform(.85,1.)]);n.floats(OBJ+0x150,[rng.uniform(.2,1.)]);n.floats(OBJ+0x130,[rng.uniform(-.4,.4) for _ in range(3)])
        history=extract_history(n);previous=None
        for step in range(6):
            n.dt=f32([1/120,1/48,1/20,1/10,1/60,1/240][step])
            commands=list(map(f32,[rng.uniform(-.3,.3),-1. if step%2 else 1.,rng.uniform(-.3,.3)]))
            n.floats(BASE+0x8514,commands)
            n.invoke(0x104f70ef0,[actor,INPUT+0x40,INPUT+0x41,INPUT+0x42])
            n.u.mem_write(INPUT+0x29,bytes([step%2]));n.u.mem_write(OBJ+0x48,bytes([step%2]));n.u.mem_write(INPUT+2,bytes([step not in (2,3)]))
            alpha=rng.uniform(-20.,38.);n.floats(BASE+0x1678,[alpha+rng.uniform(-1,1),alpha+rng.uniform(-1,1)])
            n.floats(BASE+0x1694,[rng.uniform(-.3,.3),rng.uniform(-.8,.8),rng.uniform(-.1,.1)])
            n.doubles(BASE+0x15e8,[rng.uniform(-.3,.3),rng.uniform(-1,1),rng.uniform(-.2,.2)])
            n.doubles(BASE+0x15b0,[rng.uniform(-400.,400.) for _ in range(3)])
            n.doubles(BASE+0x15c8,[rng.uniform(-80.,80.) for _ in range(3)])
            q=[rng.uniform(-1,1) for _ in range(4)];norm=math.sqrt(sum(v*v for v in q));n.floats(BASE+0x1570,[v/norm for v in q])
            n.doubles(OWNER+0x25a48,[rng.uniform(0,200000),rng.uniform(-1000,1000),0.]);n.doubles(OWNER+0x25a60,[0.,0.,rng.uniform(-5000,5000)])
            state=extract_state(n,model,flaps)
            if previous:
                for key in ['trim_requested','trim_actual','trim_cache','time_constants','linear_rates']:state[key]=previous[key]
            expected=keyboard_step(model,state,history,n.dt);a=n.step();actual=extract_result(n)
            counts['updates']+=1;counts['active_recovery']+=expected['diagnostics']['recovery_active']
            auto=expected['diagnostics']['autotrim']
            if auto:counts['autotrim_success' if auto['success'] else 'autotrim_failure']+=1
            bad={k:dict(actual=v,expected=expected[k]) for k,v in actual.items() if v!=expected[k]}
            if bad:
                counts['failures']+=1
                if len(failures)<12:failures.append(dict(case=[name,step],fields=bad,native_diagnostics=a,port_diagnostics=expected['diagnostics']))
            history=expected['history'];previous=expected
        counts['aircraft']+=1
        if counts['aircraft']%25==0:print(counts,flush=True)
    report=dict(binary_sha256=n.sha,counts=counts,failures=failures,
        scope='Independent controller history, prior dispatch-command cache, trim and response parameters propagated across 6 changing prescribed steps per jet. Random frames, accelerations, body rates, engine force/pitch moment, flaps/sweep, both MouseAim selectors, RB settings and high-speed authority. No closed-loop flight or spawned state claimed.')
    Path('analysis/instructor-full/keyboard-history-validation.json').write_text(json.dumps(report,indent=2)+'\n');print(counts,flush=True)
    if counts['failures']:raise SystemExit(1)

if __name__=='__main__':main(sys.argv[1].split(',') if len(sys.argv)>1 else None)
