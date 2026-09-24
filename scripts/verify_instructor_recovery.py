"""Original active Instructor recovery/filter stage versus independent port.

Prepared slice inputs; this is not closed-loop Instructor boundary validation.
"""
import json,random
from pathlib import Path
from unicorn import UC_HOOK_CODE
from unicorn.x86_const import *
from instructor_native import InstructorNative,OBJ,BASE,STACK,INPUT,OUTPUT
from instructor_protection import recovery_filter
from component_assembly import f32
from aircraft_model import prepare
from mass_model import aircraft_properties,evaluate
from jet_catalog import catalog,fuel_capacities
from instructor_source import load


def main():
    n=InstructorNative();rng=random.Random(202609202);failures=[]
    stop=0x101a9517e
    hook=n.u.hook_add(UC_HOOK_CODE,lambda u,a,size,data:u.emu_stop(),begin=stop,end=stop)
    for i in range(3000):
        inputs=dict(commands=[f32(rng.uniform(-1,1)) for _ in range(3)],
            current_commands=[f32(rng.uniform(-1,1)) for _ in range(3)],
            history=[f32(rng.uniform(-1,1)) for _ in range(3)],
            command_bounds=[[f32(rng.uniform(-1,0)),f32(rng.uniform(0,1))] for _ in range(3)],
            peak=f32(rng.uniform(-20,90)),reference=f32(rng.uniform(-10,60)),critical_high=f32(rng.uniform(10,50)),
            current_wing_peak=f32(rng.uniform(-20,90)),stored_yaw_rate=rng.choice([-1.,0.,1.])*rng.random(),
            dt=f32(rng.choice([0.,1/30,1/48,1/60,1/120])),max_cvt_angle=f32(rng.choice([1.,1.5,.7,2.])),
            smoothness=f32(rng.choice([0.,1e-10,1e-9,.29,.5])))
        # Execute the active branch directly, including reversed interpolation
        # knots, zero smoothing, signed yaw rates, clamped endpoints and
        # distinct source-cache/destination-FM commands (conditional writes).
        registers={UC_X86_REG_RBP:STACK,UC_X86_REG_RBX:OBJ,UC_X86_REG_RAX:BASE,
            UC_X86_REG_RSI:OUTPUT,UC_X86_REG_RCX:OUTPUT+4,UC_X86_REG_RDX:OUTPUT+8,
            UC_X86_REG_R12:BASE+0x8518,UC_X86_REG_R14:BASE+0x851c,UC_X86_REG_R15:BASE+0x8514,
            UC_X86_REG_R13:INPUT}
        for reg,value in registers.items():n.u.reg_write(reg,value)
        n.floats(OUTPUT,inputs['commands']);n.floats(BASE+0x8514,inputs['current_commands'])
        n.doubles(BASE+0x15e8,[inputs['stored_yaw_rate']])
        n.floats(OBJ+0x130,[inputs['history'][1],inputs['history'][0],inputs['history'][2]])
        n.floats(0x107e4992c,[inputs['max_cvt_angle'],inputs['smoothness']])
        for reg,val in [(7,inputs['peak']),(8,inputs['critical_high']),(9,inputs['reference']),
                        (6,inputs['command_bounds'][1][0]),(11,inputs['command_bounds'][1][1]),(12,-1.),(13,1.)]:n.xmm(reg,[val])
        for offset,val in [(0x34a0,inputs['current_wing_peak']),(0x3400,inputs['dt']),
                           (0x3550,inputs['command_bounds'][0][0]),(0x33bc,inputs['command_bounds'][0][1]),
                           (0x3510,inputs['command_bounds'][2][0]),(0x3470,inputs['command_bounds'][2][1])]:n.floats(STACK-offset,[val])
        n.invoke(0x101a94df1,stop=stop)
        actual=dict(commands=n.read(BASE+0x8514),history=[n.read(OBJ+0x134,1)[0],n.read(OBJ+0x130,1)[0],n.read(OBJ+0x138,1)[0]],
            command_bounds=[n.read(STACK-0x3408,2),n.read(STACK-0x33a0,2),[n.read(STACK-0x3510,1)[0],n.read_xmm(3)[0]]],
            pitch_mix=n.read_xmm(10)[0],axis_mix=n.read_xmm(9)[0])
        expected=recovery_filter(**inputs)
        if actual!=expected:failures.append(dict(index=i,inputs=inputs,actual=actual,expected=expected))
    n.u.hook_del(hook)
    active=0;inactive=0;probes=[]
    for name,item in catalog().items():
        if not item['supported']:continue
        fm=load(name);model=prepare(fm)
        mass=evaluate(aircraft_properties(fm),fuel_by_system=[f32(v*.3) for v in fuel_capacities(fm)])
        for alpha in [10.,45.]:
            n.setup(model,mass,speed=220.,alpha=alpha)
            # Stop before the active-stage dispatch, then continue on the same
            # native stack. This preserves real predictor outputs and histories.
            result=n.step(stop=0x101a94d8b)
            inputs=result['recovery_inputs']
            expected_gate=inputs['peak']>inputs['reference']
            # These prepared cases enable recovery and keep input+9 clear.
            n.capture_stop=None
            from instructor_native import STOP
            n.u.emu_start(0x101a94d8b,STOP,count=1000000)
            if n.u.reg_read(UC_X86_REG_RIP)!=STOP:raise RuntimeError('Recovery controller continuation did not finish')
            output=n.captured.get('recovery_output')
            if bool(output)!=expected_gate:failures.append(dict(aircraft=name,alpha=alpha,kind='gate'))
            if output:
                active+=1;expected=recovery_filter(**inputs)
                if output!=expected:failures.append(dict(aircraft=name,alpha=alpha,actual=output,expected=expected))
            else:inactive+=1
            if name in ['f_16a_block_15_adf','saab_jas39c','f_14a_early']:
                probes.append(dict(aircraft=name,alpha=alpha,active=bool(output),inputs=inputs,output=output))
    report=dict(cases=3000,controller_cases=active+inactive,active_controller_comparisons=active,inactive_controller_gates=inactive,
        failure_count=len(failures),failures=failures[:8],probes=probes,
        scope='Native active recovery/filter instruction span plus full-controller stage captures; explicit libm adapter. Prepared caller state and whole-controller reachability remain unvalidated.')
    Path('analysis/instructor-recovery-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2));assert not failures
if __name__=='__main__':main()
