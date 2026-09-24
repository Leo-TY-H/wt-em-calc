"""Differential validation of the native post-predictor pitch clamp.

Checks a controller stage, not closed-loop Instructor feasibility. Synthetic
slice fixtures and complete-controller stage captures are reported separately.
"""
import json,random
from pathlib import Path
from unicorn import UC_HOOK_CODE
from unicorn.x86_const import *
from instructor_native import InstructorNative,OBJ,BASE,STACK,INPUT
from instructor_protection import protected_pitch_command
from component_assembly import f32
from aircraft_model import prepare
from mass_model import aircraft_properties,evaluate
from jet_catalog import catalog,load,fuel_capacities


def main():
    n=InstructorNative();rng=random.Random(20260920);failures=[];counts={'isolated_clamp':0,'controller_clamp':0};probes=[]
    def check(kind,a,e,case):
        counts[kind]+=1
        if a!=e:failures.append(dict(kind=kind,case=case,actual=a,expected=e))
    for i in range(2000):
        inputs=dict(predictor_commands=[f32(rng.uniform(-4,4)) for _ in range(2)],
            trim=f32([-1,0,1,rng.uniform(-1,1)][i%4]),authority_inverse=[f32(-rng.uniform(1,50)),f32(rng.uniform(1,50))],
            authority_factor=f32(rng.uniform(.2,1)),requested=f32([-1,1,rng.uniform(-1,1)][i%3]),
            predicted_adjusted=[f32(rng.uniform(-50,50)) for _ in range(2)],
            adjustment_offsets=[f32(rng.uniform(-10,10)) for _ in range(2)],
            critical_high=f32(rng.uniform(10,50)),recovery_reference=f32(rng.uniform(10,50)),
            dt=f32(rng.choice([1/30,1/48,1/60,1/120])),adaptation_rates=n.read(0x107209df0,2))
        n.qword(OBJ,BASE);n.u.reg_write(UC_X86_REG_RBX,OBJ);n.u.reg_write(UC_X86_REG_RBP,STACK)
        for off,v in [(0x36b4,inputs['predictor_commands'][0]),(0x3680,inputs['predictor_commands'][1]),
                      (0x3370,inputs['predictor_commands'][0]),(0x3540,inputs['authority_inverse'][0]),(0x3460,inputs['authority_inverse'][1]),
                      (0x33c0,inputs['predicted_adjusted'][0]),(0x33c4,inputs['predicted_adjusted'][1]),
                      (0x3454,inputs['adjustment_offsets'][0]),(0x345c,inputs['adjustment_offsets'][1]),
                      (0x33e4,inputs['critical_high']),(0x3458,inputs['recovery_reference']),(0x3400,inputs['dt'])]:n.floats(STACK-off,[v])
        n.floats(BASE+0x87f8,[inputs['trim']]);n.floats(BASE+0x8518,[inputs['requested']]);n.floats(OBJ+0x150,[inputs['authority_factor']])
        a=n.invoke(0x101a94b54,stop=0x101a94d8b)
        check('isolated_clamp',a['pitch_clamp'],protected_pitch_command(**inputs),i)
    for name,item in catalog().items():
        if not item['supported']:continue
        fm=load(name);model=prepare(fm);mass=evaluate(aircraft_properties(fm),fuel_by_system=[f32(x*.3) for x in fuel_capacities(fm)])
        for case in range(2):
            n.setup(model,mass,speed=150+case*150,alpha=8+case*12,commands=(.1,-.2,.05))
            n.floats(BASE+0x87f8,[f32(rng.uniform(-.6,.6))]);n.floats(BASE+0x8518,[-1 if case else 1]);n.floats(OBJ+0x150,[.2 if case else 1.])
            a=n.step(stop=0x101a94d8b)
            check('controller_clamp',a['pitch_clamp'],protected_pitch_command(**a['pitch_clamp_inputs']),[name,case])
            if name in ['f_16a_block_15_adf','saab_jas39c','f_14a_early']:probes.append(dict(aircraft=name,inputs=a['pitch_clamp_inputs'],outputs=a['pitch_clamp']))
        if counts['controller_clamp']%100==0:print(counts,'failures',len(failures),flush=True)
    # Full keyboard override clears all three axis-assist flags. Audit both
    # MouseAim output gates: the advanced path restores saved commands, while
    # the simple path skips the final assisted writes. This does not validate
    # their upstream trajectory predictors or every side effect of the helpers.
    for kind,start,stop in [('advanced_keyboard_output',0x101a9b5e5,0x101a9b660),
                            ('simple_keyboard_output',0x101a9d65b,0x101a9d684)]:
        counts[kind]=0
        hook=n.u.hook_add(UC_HOOK_CODE,lambda u,a,size,data:u.emu_stop(),begin=stop,end=stop)
        for i in range(1000):
            original=[f32(rng.uniform(-1,1)) for _ in range(3)]
            n.u.reg_write(UC_X86_REG_RBP,STACK);n.u.reg_write(UC_X86_REG_RAX,BASE)
            n.u.reg_write(UC_X86_REG_R13,INPUT);n.u.mem_write(INPUT+0x38,b'\0\0\0')
            for offset,x in zip([0x33dc,0x33e0,0x33e4],original):n.floats(STACK-offset,[x])
            n.floats(BASE+0x8514,[f32(rng.uniform(-3,3)) for _ in range(3)] if kind.startswith('advanced') else original)
            n.u.emu_start(start,stop+1,count=100)
            if n.u.reg_read(UC_X86_REG_RIP)!=stop:raise RuntimeError('Keyboard output gate did not finish')
            check(kind,n.read(BASE+0x8514),original,i)
        n.u.hook_del(hook)
    out=dict(counts=counts,failures=failures,probes=probes,
             scope='post-predictor clamp and disabled MouseAim final output gates only; no closed-loop boundary validation')
    Path('analysis/instructor-command-validation.json').write_text(json.dumps(out,indent=2))
    print(counts,'failures',len(failures));assert not failures,failures[:3]
if __name__=='__main__':main()
