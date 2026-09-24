"""Compare independently packed predictor inputs with the original wrapper.

Prescribed states, not captured spawned aircraft. This validates the wrapper
and recovery-reference arithmetic, not the reduced force/moment predictor.
"""
import json
import math
import random
import struct
from pathlib import Path
from unicorn import UC_HOOK_CODE
from unicorn.x86_const import UC_X86_REG_RSI
from instructor_native import InstructorNative, BASE, OWNER, INPUT, OUTPUT, HEAP
from instructor_predictor_inputs import pack_pitch_inputs, pack_autotrim_inputs, defined_bytes, DEFINED_SPANS
from instructor_protection import recovery_reference
from component_assembly import f32
from body_dynamics import realistic_engine_scale
from control_mixer import curve
from aircraft_model import prepare
from mass_model import aircraft_properties, evaluate
from instructor_source import load


def main():
    n=InstructorNative();rng=random.Random(202609201);failures=[];count=0;autotrim_count=0
    def stop(u,a,size,data):u.emu_stop()
    handle=n.u.hook_add(UC_HOOK_CODE,stop,begin=0x101a5cac0,end=0x101a5cac0)
    for name in ['f_16a_block_15_adf','saab_jas39c','f_14a_early','mig_23m','harrier_gr3']:
        fm=load(name);model=prepare(fm);mass=evaluate(aircraft_properties(fm),fuel_by_system=[1000.])
        n.setup(model,mass)
        for i in range(200):
            def rand(lo,hi):return f32(rng.uniform(lo,hi))
            q=[rng.uniform(-1,1) for _ in range(4)];norm=math.sqrt(sum(x*x for x in q));q=[f32(x/norm) for x in q]
            v=[rng.uniform(-500,500) for _ in range(3)] if i%10 else [0.,0.,0.]
            flap=rand(0,1);mapped=curve(model['flaps'],flap,4) if model['flaps'] else [flap,flap,0.,0.]
            ext,ext_base=rand(.5,1.5),rand(.1,1.5)
            args=dict(target_angle=rand(-30,40),target_acceleration=rand(-5,5),body_pitch_rate=rng.uniform(-2,2),
                angle_bounds=[rand(-50,-5),rand(5,50)],axis_weights=[rand(.2,2) for _ in range(3)],
                mass=rand(1000,50000),quaternion=q,world_velocity=v,tas=rand(0,600),speed_squared=rand(1,360000),
                mach=rand(0,2.5),height=rng.uniform(-500,18000),engine_wash=[rand(0,100) for _ in range(2)],
                engine_force=[rng.uniform(-100000,100000) for _ in range(3)],engine_moment=[rng.uniform(-10000,10000) for _ in range(3)],
                engine_scale=realistic_engine_scale(ext,ext_base),torque_gyro=bool(i%2),
                flap_blend=mapped[0],flap_health=[rand(0,1),rand(0,1)],flap_incidence=mapped[2],sweep=rand(0,1),
                gear_fraction=rand(-.1,1.1),gear_available=[bool(i%2),bool(i%3)],airbrake_fraction=rand(-.1,1.1),
                airbrake_health=[rand(0,1),rand(0,1)],oil_radiator=rand(0,1),water_radiator=rand(0,1),cockpit_door=rand(0,1),
                parameter_pointer=0x107d6fba0,gameplay_pointer=HEAP+0x2000+0x2b8)
            n.floats(INPUT,args['angle_bounds']);n.floats(INPUT+16,args['axis_weights'])
            for off,key in [(0x5308,'mass'),(0x845c,'tas'),(0x8460,'speed_squared'),(0x8464,'mach'),
                            (0x16a4,'sweep'),(0x2b50,'gear_fraction'),(0x2b58,'airbrake_fraction'),(0x16a0,'cockpit_door')]:n.floats(BASE+off,[args[key]])
            n.doubles(BASE+0x15f0,[args['body_pitch_rate']]);n.doubles(BASE+0x15b0,v);n.floats(BASE+0x1570,q)
            n.doubles(BASE+0x1560,[args['height']]);n.floats(BASE+0x8494,[max(0.,args['height'])])
            n.floats(BASE+0x18dc,args['flap_health']);n.floats(BASE+0x18e4,args['airbrake_health'])
            n.floats(BASE+0x2b60,[flap]);n.floats(BASE+0x7c58,[0.])
            n.qword(BASE+0x1860,sum(int(x)<<(28+j) for j,x in enumerate(args['gear_available'])))
            n.floats(BASE+0x84e4,[ext]);n.floats(BASE+0x7dfc,[ext_base])
            n.u.mem_write(BASE+0x3658,bytes([2 if args['torque_gyro'] else 0]));n.u.mem_write(0x107d6fbf7,b'\0')
            n.floats(OWNER+0x25b58,args['engine_wash']);n.doubles(OWNER+0x25a48,args['engine_force']);n.doubles(OWNER+0x25a60,args['engine_moment'])
            n.floats(BASE+0x5809c,[args['oil_radiator'],args['water_radiator']])
            n.invoke(0x101a60b50,[BASE,args['parameter_pointer'],INPUT,INPUT+16,OUTPUT,OUTPUT+0x100],
                     [args['target_angle'],args['target_acceleration']],stop=0x101a5cac0)
            actual=bytes(n.u.mem_read(n.u.reg_read(UC_X86_REG_RSI),0xa0));expected=pack_pitch_inputs(**args)
            if defined_bytes(actual)!=defined_bytes(expected):
                diffs=[]
                for start,size in DEFINED_SPANS:
                    for off in range(start,start+size):
                        if actual[off]!=expected[off]:diffs.append(hex(off))
                failures.append(dict(aircraft=name,index=i,bytes=diffs,actual=actual.hex(),expected=expected.hex()))
            count+=1
            shared={key:value for key,value in args.items() if key not in [
                'target_angle','target_acceleration','body_pitch_rate','angle_bounds',
                'axis_weights','quaternion','world_velocity']}
            target_load=rand(-2.,5.)
            n.invoke(0x101a60830,[BASE,args['parameter_pointer'],OUTPUT,OUTPUT+0x100],
                     [target_load],stop=0x101a5cac0)
            actual=bytes(n.u.mem_read(n.u.reg_read(UC_X86_REG_RSI),0xa0))
            expected=pack_autotrim_inputs(target_load=target_load,**shared)
            if defined_bytes(actual)!=defined_bytes(expected):
                failures.append(dict(aircraft=name,index=i,stage='autotrim_wrapper',actual=actual.hex(),expected=expected.hex()))
            autotrim_count+=1
    n.u.hook_del(handle)
    report=dict(binary_sha256=n.sha,wrapper_cases=count,autotrim_wrapper_cases=autotrim_count,failures=failures[:10],failure_count=len(failures),
                scope='Defined bytes of 101a60b50 and 101a60830 input preparation; randomized prescribed state. Predictor not executed by this test.',
                adapters=sorted(set(n.calls+n.extra_calls)))
    Path('analysis/instructor-full/predictor-input-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print('wrapper cases',count,'autotrim',autotrim_count,'failures',len(failures),failures[:1]);assert not failures


if __name__=='__main__':main()
