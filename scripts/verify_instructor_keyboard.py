"""Independent complete keyboard command path from entry source state.

This compares observable aircraft commands, trim and relevant persistent
histories. No native controller local/intermediate is an independent input.
Disabled-output MouseAim internals execute natively but are not ported.
"""
import json,random,sys
from pathlib import Path
from instructor_native import InstructorNative,BASE,OWNER,OBJ,INPUT,HEAP
from instructor_keyboard import keyboard_step
from verify_instructor_pitch_predictor import source_state
from aircraft_model import prepare,at_sweep
from jet_catalog import catalog,fuel_capacities
from instructor_source import load
from mass_model import aircraft_properties,evaluate
from body_dynamics import realistic_engine_scale
from component_assembly import f32
from control_mixer import curve
from polar_runtime import flap_polar


def extract_history(n):
    def predictor(off):return n.read(OBJ+off,2)+[bool(n.u.mem_read(OBJ+off+8,1)[0])]
    ring=n.read(OBJ+8,1,'Q')[0];buf=n.read(ring,1,'Q')[0]
    return dict(last_commands=[n.read(OBJ+o,1)[0] for o in [0xe4,0xe0,0xe8]],autotrim=predictor(0x15c),predictors=[predictor(0x188),predictor(0x1b4)],angles=n.read(OBJ+0x148,2),
        overload_timer=n.read(OBJ+0x154,1)[0],authority_factor=n.read(OBJ+0x150,1)[0],recovery=[n.read(OBJ+o,1)[0] for o in [0x134,0x130,0x138]],
        dt_samples=n.read(buf,n.read(ring+8,1,'I')[0]),dt_index=n.read(ring+0xc,1,'I')[0],dt_sum=n.read(ring+0x10,1)[0])


def extract_result(n):
    return dict(commands=n.read(BASE+0x8514,3),trim_requested=n.read(BASE+0x87f4,3),trim_actual=n.read(BASE+0xa290,3),trim_cache=n.read(BASE+0x3a24,3),
        time_constants=n.read(BASE+0x84fc,3),linear_rates=n.read(BASE+0x8508,3),history=extract_history(n))


def extract_state(n,model,raw_flaps):
    def f(o):return n.read(BASE+o,1)[0]
    def flag(o):return bool(n.u.mem_read(BASE+o,1)[0])
    # Temporary source pointer supplies only the documented parameter address;
    # source_state reads raw FM scalar properties, not a controller stack.
    p=INPUT+0x800;n.qword(p+0x90,0x107d6fba0);predictor=source_state(n,p)
    predictor['f'].update({o:f(o) for o in [0x5328,0x79b8,0x79bc]})
    flap=f(0x2b60);mapped=curve(model['flaps'],flap,4) if model['flaps'] else [flap,flap,0.,0.]
    wrapper=dict(mass=f(0x5308),tas=f(0x845c),speed_squared=f(0x8460),mach=f(0x8464),height=n.read(BASE+0x1560,1,'d')[0],
        engine_wash=n.read(OWNER+0x25b58,2),engine_force=n.read(OWNER+0x25a48,3,'d'),engine_moment=n.read(OWNER+0x25a60,3,'d'),
        engine_scale=realistic_engine_scale(f(0x84e4),f(0x7dfc)),torque_gyro=bool(((n.u.mem_read(BASE+0x3658,1)[0]>>1) | n.u.mem_read(0x107d6fbf7,1)[0]) & 1),
        flap_blend=mapped[0],flap_health=n.read(BASE+0x18dc,2),flap_incidence=mapped[2],sweep=f(0x16a4),
        gear_fraction=f(0x2b50),gear_available=[bool(n.read(BASE+0x1860,1,'Q')[0]&(1<<j)) for j in [28,29]],
        airbrake_fraction=f(0x2b58),airbrake_health=n.read(BASE+0x18e4,2),oil_radiator=f(0x5809c),water_radiator=f(0x580a0),
        cockpit_door=f(0x16a0),parameter_pointer=0x107d6fba0,gameplay_pointer=HEAP+0x22b8)
    return dict(predictor=predictor,wrapper=wrapper,properties=n.properties,
        time_constants=n.read(BASE+0x84fc,3),linear_rates=n.read(BASE+0x8508,3),
        trim_requested=n.read(BASE+0x87f4,3),trim_actual=n.read(BASE+0xa290,3),trim_cache=n.read(BASE+0x3a24,3),
        autotrim_enabled=bool(n.u.mem_read(INPUT+2,1)[0]),default_autotrim=bool(n.u.mem_read(0x107d6fbc1,1)[0]),
        indicated_airspeed=f(0x8468),longitudinal_speed=n.read(BASE+0x1618,1,'d')[0],rho0=n.read(0x107d6f790,1)[0],
        axis_enabled=list(map(bool,n.u.mem_read(BASE+0x8520,3))),authority_scale=f(0x16a8),full_control_loss=bool(n.u.mem_read(0x107d6fbf5,1)[0]),
        asymmetric_authority=bool(n.u.mem_read(0x107d6fc2d,1)[0]),elevator_state=f(0x39f8),delivered=n.read(BASE+0x1694,3),wing_angles=n.read(BASE+0x1678,2),
        wing_runtime=flap_polar(model['polars']['WingPlane'],raw_flaps),wing_area=f(0x8254),dihedral=f(0x8154),strength=n.read(BASE+0x81b4,2),
        tail_area_pair=n.read(BASE+0x6f2c,2),overload_enabled=bool(n.u.mem_read(0x107d6fbc2,1)[0]),
        force_advanced=bool(n.u.mem_read(OBJ+0x48,1)[0]),world_velocity=n.read(BASE+0x15b0,3,'d'),world_acceleration=n.read(BASE+0x15c8,3,'d'),
        quaternion=n.read(BASE+0x1570,4),pitch_rate=n.read(BASE+0x15f0,1,'d')[0],stored_yaw_rate=n.read(BASE+0x15e8,1,'d')[0],reference_speed=f(0x7f8c),
        requested=n.read(BASE+0x8514,3),
        command_cache_enabled=bool(n.u.mem_read(INPUT+0x29,1)[0]),recovery_enabled=bool(n.u.mem_read(0x107d6fbb8,1)[0]),recovery_suppressed=bool(n.u.mem_read(INPUT+9,1)[0]))


def main(names=None,report_path=None):
    n=InstructorNative();rng=random.Random(260921);failures=[];counts={'updates':0,'failures':0,'active_recovery':0,'autotrim_success':0,'autotrim_failure':0}
    for name,item in catalog().items():
        if not item['supported'] or names and name not in names:continue
        fm=load(name);base=prepare(fm);mass=evaluate(aircraft_properties(fm),fuel_by_system=[f32(v*.3) for v in fuel_capacities(fm)])
        for case in range(4):
            sweep=[0.,0.,.4,1.][case];flaps=[0.,0.,.37,1.][case];model=at_sweep(base,sweep)
            n.setup(model,mass,speed=[100.,250.,350.,180.][case],alpha=[4.,30.,-12.,18.][case],flaps=flaps,sweep=sweep,height=[0.,6000.,12000.,1000.][case],mode_lane=True)
            actor=n.empty_payload_actor();n.qword(actor+0x2ef0,BASE)
            n.u.mem_write(INPUT+2,bytes([case!=2]));n.u.mem_write(INPUT+0x29,bytes([case%2]));n.floats(INPUT+0x10,[0.]);n.floats(INPUT+0x44,[1.])
            n.u.mem_write(0x107d6fbc0,b'\1\1');n.u.mem_write(BASE+0x3658,b'\3');n.u.mem_write(OBJ+0x48,bytes([case%2]))
            n.u.mem_write(0x107d6fbf5,b'\1')  # difficulty.highSpeedControlEffects
            commands=list(map(f32,[.15,-1. if case<2 else 1.,-.05]));n.floats(BASE+0x8514,commands);n.floats(OBJ+0xe0,[commands[1],commands[0],commands[2]])
            n.invoke(0x104f70ef0,[actor,INPUT+0x40,INPUT+0x41,INPUT+0x42])
            n.floats(BASE+0x84fc,[.2,.3,.4]);n.floats(BASE+0x8508,[.03,.02,.01])
            n.floats(BASE+0x87f4,[.12,-.14,.03]);n.floats(BASE+0xa290,[-.04,.08,-.01]);n.floats(BASE+0x3a24,[.07,-.06,.02])
            state=extract_state(n,model,flaps);history=extract_history(n)
            expected=keyboard_step(model,state,history,n.dt);a=n.step()
            actual=extract_result(n)
            counts['updates']+=1;counts['active_recovery']+=expected['diagnostics']['recovery_active']
            auto=expected['diagnostics']['autotrim']
            if auto:counts['autotrim_success' if auto['success'] else 'autotrim_failure']+=1
            bad={k:dict(actual=v,expected=expected[k]) for k,v in actual.items() if v!=expected[k]}
            if bad:
                counts['failures']+=1
                if len(failures)<12:failures.append(dict(case=[name,case],fields=bad,native_diagnostics=a,port_diagnostics=expected['diagnostics']))
        if counts['updates']%100==0:print(counts,flush=True)
    report=dict(binary_sha256=n.sha,counts=counts,failures=failures,
        scope='Whole independent full-pitch keyboard commands, auto trim, relevant histories and response filters from entry source state. Intact symmetric free-air RB constant-gain path, orbiting disabled, gyro enabled; MouseAim disabled-output internals are not ported. Prepared providers, not live aircraft.')
    path=Path(report_path or 'analysis/instructor-full/keyboard-port-validation.json')
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(report,indent=2)+'\n');print(counts,flush=True)
    if counts['failures']:raise SystemExit(1)

if __name__=='__main__':main(sys.argv[1].split(',') if len(sys.argv)>1 else None)
