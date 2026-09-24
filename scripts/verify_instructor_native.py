"""Native differential tests for the recovered Instructor protection stage.

The full update smoke tests exercise original predictors but are NOT evidence
of closed-loop boundary accuracy. See the report's explicit validation scope.
"""
import json,math,random,struct
from pathlib import Path
from unicorn.x86_const import *
from instructor_native import InstructorNative,OBJ,INPUT,OUTPUT,ARENA,BASE,STACK
from instructor_protection import inverse_cl,angle_targets,predicted_wing_angles,angle_rate,pitch_demands,gain_reference_speed
from component_assembly import f32,mul
from body_dynamics import G
from polar_model import FIELDS,pack_polar
from polar_runtime import evaluate as mach_polar,flap_polar,pack_runtime
from wing_sweep import select as select_wing
from aircraft_model import prepare,at_sweep
from mass_model import aircraft_properties,evaluate as mass_evaluate
from control_mixer import density_at_height,curve
from jet_catalog import catalog,fuel_capacities
from instructor_source import load


def main():
    n=InstructorNative();rng=random.Random(20260919);failures=[]
    counts=dict(inverse=0,reference_speed=0,angle_targets=0,wing_prediction=0,angle_rate=0,pitch_demands=0,
                keyboard_disable=0,full_update_smoke=0,history_steps=0)
    branches=dict(timer_increase=0,timer_decrease=0,load_factor=0,no_overload=0,first_lane=0,second_lane=0)
    probes=[];history_probes=[];providers=set();maximum_angle_error=0.
    def check(kind,actual,expected,case):
        counts[kind]+=1
        if actual!=expected:failures.append(dict(stage=kind,case=case,actual=actual,expected=expected))
    supported=[name for name,p in catalog().items() if p['supported']]
    for index,name in enumerate(supported):
        fm=load(name);model=prepare(fm)
        # Isolate the actual loader arithmetic, after prepared clean-wing
        # selection and the nonbuoyant-aircraft branch. No current fuel input.
        runtime=flap_polar(select_wing(model['wing_family'],0.)['polars'],0.)
        n.u.mem_write(STACK-0x300,pack_runtime(runtime,STACK-0x300))
        n.floats(STACK-0x4f0,[mul(mul(f32(fm['Mass'].get('Takeoff',0.)),f32(1.1)),G)])
        n.qword(STACK-0x4e8,BASE);n.u.reg_write(UC_X86_REG_RBP,STACK)
        n.u.reg_write(UC_X86_REG_RBX,0x107d6f7a0)
        n.u.emu_start(0x101a3d961,0x101a3d9c1,count=100)
        if n.u.reg_read(UC_X86_REG_RIP)!=0x101a3d9c1:raise RuntimeError('Reference-speed slice did not finish')
        check('reference_speed',n.read(BASE+0x7f8c,1)[0],gain_reference_speed(fm['Mass'].get('Takeoff',0.),runtime),name)
        for component in ['WingPlane','HorStabPlane','VerStabPlane','FuselagePlane']:
            for mach in [0.,.95,1.25]:
                p=mach_polar(model['polars'][component][0][1],mach)
                targets=[mul(p['clKq'],v) for v in [p['cyCritL']-.01,p['cyCritL'],p['cl0'],p['cyCritH'],p['cyCritH']+.01]]
                targets += [f32(rng.uniform(p['cyCritL'],p['cyCritH'])) for _ in range(3)]
                for target in targets:
                    n.u.mem_write(ARENA+0x8000,pack_polar(p))
                    n.invoke(0x10198c7f0,[ARENA+0x8000,OUTPUT],[target])
                    actual=[n.read(OUTPUT,1)[0],bool(n.u.reg_read(UC_X86_REG_RAX)&255)]
                    check('inverse',actual,list(inverse_cl(p,target)),[name,component,mach,target])
        mass=mass_evaluate(aircraft_properties(fm),fuel_by_system=[f32(v*.3) for v in fuel_capacities(fm)])
        for i,(speed,alpha,timer) in enumerate([(80.,3.,0.),(200.,18.,.89),(350.,25.,.95),(280.,-22.,1.)]):
            sweep=.5 if len(model['wing_family'])>1 else 0.
            selected=at_sweep(model,sweep);dt=f32([1/30,1/48,1/60,1/120][i])
            n.setup(selected,mass,speed=speed,alpha=alpha,flaps=0.,dt=dt,sweep=sweep,mode_lane=bool(i%2))
            if i==0:
                n.u.mem_write(OBJ+0xf4,b'\x00');n.properties['limitOverload']=False
            if i==3:
                n.properties['limitLoadfactor']=True;n.properties['loadFactorLimit']=[-3.,7.]
                n.u.mem_write(OBJ+0x114,b'\x01');n.floats(OBJ+0x118,[mul(-3.,G),mul(7.,G)])
            n.floats(OBJ+0x154,[timer])
            old=n.read(OBJ+0x148,2);angles=n.read(BASE+0x1678,2)
            commands=n.read(BASE+0x1694);mach=n.read(BASE+0x8464,1)[0]
            ail=selected['controls']['Ailerons']
            sensitivity=mul(curve(ail['sensitivity_curve'],mach,1)[0],ail['sensitivity'])
            prediction=predicted_wing_angles(angles,old,commands[0],commands[1],sensitivity,n.read(BASE+0x7ab8,1)[0])
            actual=n.step(stop=0x101a939a4)
            check('wing_prediction',[n.read(OBJ+0x148,2),actual['predicted_wing_angles']],
                  [prediction['adjusted'],sorted(prediction['predicted'])],[name,i])
            p=dict(zip(FIELDS,actual['wing_polar']))
            expected=angle_targets(p,n.properties,actual['predicted_wing_angles'],
                rho=density_at_height(0.),speed_squared=n.read(BASE+0x8460,1)[0],
                area=n.read(BASE+0x8254,1)[0],dihedral=n.read(BASE+0x8154,1)[0],
                strength=n.read(BASE+0x81b4,2),mass=mass['mass'],
                tail_area_pair=n.read(BASE+0x6f2c,2),timer=timer,dt=dt,mode_lane=bool(i%2))
            check('angle_targets',[list(actual['angle_limits']),actual['overload_timer']],
                [expected['angle_limits'],expected['overload_timer']],[name,i])
            maximum_angle_error=max(maximum_angle_error,max(abs(x-y) for x,y in zip(actual['angle_limits'],expected['angle_limits'])))
            branches['timer_increase']+=actual['overload_timer']>timer
            branches['timer_decrease']+=actual['overload_timer']<timer
            branches['load_factor']+=i==3;branches['no_overload']+=i==0
            branches['first_lane' if i%2 else 'second_lane']+=1
            providers.update(n.extra_calls)
        if index%50==0:print('aircraft',index+1,'/',len(supported),'failures',len(failures),flush=True)
    # Full-pitch axis-assist disable: execute the whole native owner helper.
    # Mouse-aim threshold and profile overrides are prepared, not guessed.
    owner=ARENA+0x70000;n.qword(owner+0x2ef0,BASE)
    for pitch in [-1.,1.]:
        for roll in [-1.,0.,.5,1.]:
            for yaw in [-1.,0.,1.]:
                n.floats(BASE+0x8514,[roll,pitch,yaw]);n.u.mem_write(OUTPUT,b'\xff'*3)
                n.invoke(0x104f70ef0,[owner,OUTPUT,OUTPUT+1,OUTPUT+2])
                check('keyboard_disable',list(n.u.mem_read(OUTPUT,3)),[0,0,0],[roll,pitch,yaw])
    # Independent full update runs, not resuming partially executed frames.
    for name in ['f_16a_block_15_adf','saab_jas39c','f_14a_early']:
        fm=load(name);model=prepare(fm);mass=mass_evaluate(aircraft_properties(fm),fuel_by_system=[f32(v*.3) for v in fuel_capacities(fm)])
        for speed in [150.,250.,350.]:
            for alpha in [5.,15.,25.]:
                n.setup(model,mass,speed=speed,alpha=alpha)
                actual=n.step();commands=n.read(BASE+0x8514)
                ok=all(math.isfinite(x) and abs(x)<=1.000001 for x in commands) and len(actual.get('predictor_inputs',[]))==2
                check('full_update_smoke',ok,True,[name,speed,alpha]);providers.update(n.extra_calls)
                probes.append(dict(aircraft=name,speed_mps=speed,alpha_deg=alpha,commands=commands,
                                   angle_targets=actual['angle_limits'],predictor_inputs=actual['predictor_inputs']))
    for name in ['f_16a_block_15_adf','saab_jas39c']:
        fm=load(name);model=prepare(fm);mass=mass_evaluate(aircraft_properties(fm),fuel_by_system=[f32(v*.3) for v in fuel_capacities(fm)])
        for i in range(150):
            speed=rng.uniform(50.,500.);alpha=rng.uniform(-25.,35.);height=rng.uniform(0.,15000.)
            n.setup(model,mass,speed=speed,alpha=alpha,height=height,reference_speed_override=rng.uniform(50.,150.))
            velocity=[rng.uniform(-400.,400.) for _ in range(3)];acceleration=[rng.uniform(-100.,100.) for _ in range(3)]
            q=[rng.uniform(-1.,1.) for _ in range(4)];norm=math.sqrt(sum(x*x for x in q));q=[f32(x/norm) for x in q]
            pitch_rate=rng.uniform(-1.,1.)
            n.doubles(BASE+0x15b0,velocity);n.doubles(BASE+0x15c8,acceleration);n.floats(BASE+0x1570,q)
            n.doubles(BASE+0x15f0,[pitch_rate]);n.floats(OBJ+0x154,[rng.random()])
            a=n.step();p=dict(zip(FIELDS,a['wing_polar']))
            rate=angle_rate(velocity,acceleration,q,pitch_rate)
            check('angle_rate',a['angle_rate'],rate,[name,i])
            expected=pitch_demands(p,n.properties,a['predicted_wing_angles'],a['angle_limits'],
                reference_speed=n.read(BASE+0x7f8c,1)[0],tas=n.read(BASE+0x845c,1)[0],
                ias=n.read(BASE+0x1618,1,'d')[0]*f32(math.sqrt(f32(density_at_height(f32(height))/f32(1.225)))),rate=rate)
            check('pitch_demands',a['predictor_inputs'],expected,[name,i])
        # Carry genuine native timer history through accumulation and recovery;
        # keep the aerodynamic state prescribed to isolate this feedback stage.
        n.setup(model,mass,speed=350.,alpha=25.,commands=(0.,0.,0.));timer=0.
        for i in range(1300):
            if i==1200:n.floats(BASE+0x1678,[0.,0.])
            a=n.step(stop=0x101a939a4);p=dict(zip(FIELDS,a['wing_polar']))
            expected=angle_targets(p,n.properties,a['predicted_wing_angles'],
                rho=density_at_height(0.),speed_squared=n.read(BASE+0x8460,1)[0],
                area=n.read(BASE+0x8254,1)[0],dihedral=n.read(BASE+0x8154,1)[0],
                strength=n.read(BASE+0x81b4,2),mass=mass['mass'],tail_area_pair=n.read(BASE+0x6f2c,2),
                timer=timer,dt=n.dt)
            check('history_steps',[list(a['angle_limits']),a['overload_timer']],
                [expected['angle_limits'],expected['overload_timer']],[name,i])
            timer=expected['overload_timer']
            if i in [0,94,95,96,105,106,950,959,960,961,1065,1066,1067,1199,1200,1210,1248,1299]:
                history_probes.append(dict(aircraft=name,step=i+1,time_s=(i+1)*n.dt,timer=timer,angles=a['angle_limits']))
    report=dict(binary_sha256=n.sha,counts=counts,aircraft=len(supported),branches=branches,
        max_angle_error_deg=maximum_angle_error,failures=failures[:30],failure_count=len(failures),
        native_full_update_probes=probes,native_history_probes=history_probes,extra_adapters=sorted(providers),
        validated_scope='Exact pre-stall inverse polar; wing-angle prediction, angle targets/timer, acceleration projection and constant-gain pitch demands from prepared native state; full-pitch owner axis-assist disable. Full-update runs are smoke tests only.',
        not_validated=['Live caller mode-lane identity and all client settings','Complete prepared FM state and frame conventions',
                       'Independent port of reduced pitch predictor and final command filtering','Closed-loop reachability, stability and settling',
                       'Production static Instructor envelope (known non-equivalent)'])
    Path('analysis/instructor-native-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ['native_full_update_probes','native_history_probes','failures']},indent=2))
    if failures:raise SystemExit(1)


if __name__=='__main__':main()
