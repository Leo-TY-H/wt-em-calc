"""Independent Instructor -> actuator -> aircraft feedback with native kernels.

Research only. Controller observes the preceding air cache/engine outputs and
current integrated pose/rates; primary commands use an explicit one-step-held
snapshot. This chosen publication convention is NOT a replay of the game owner
or a validated EM boundary. No production force or equilibrium code is changed.
"""
import copy,json,math,sys
from pathlib import Path
from instructor_native import InstructorNative,BASE,OWNER,OBJ,INPUT
from instructor_keyboard import keyboard_step
from verify_instructor_keyboard import extract_state,extract_history,extract_result
from instructor_settings import deliver_keyboard_commands,restored_wing_normalization
from verify_coupled_airborne import CoupledAirborne
from verify_primary_controls import Controls
from verify_snapshot_delivery import DeliveryMachine
from verify_engine_owner import EngineOwnerMachine,expected_owner
from primary_controls import selected_properties,authority_ranges,actuator_step
from engine_supply import selected_properties as engine_properties,fuel_properties
from jet_model import prepare_nozzle
from aircraft_model import prepare,evaluate
from mass_model import aircraft_properties,evaluate as mass_evaluate
from jet_catalog import fuel_capacities
from instructor_source import load
from component_assembly import f32,mul
from air_state import world_to_body_air,cache
from structural_limits import position_tick_seed,wing_load_ratios
from body_dynamics import gravity_body,limit_aerodynamic_force
from kinematics import airborne_step,orientation_increment,restore_small_pose_change,altitude_velocity_correction
from wing_observables import wing_observables


def write_controller(n,s,requested):
    n.doubles(BASE+0x1558,s['position']);n.floats(BASE+0x1570,s['quaternion'])
    n.doubles(BASE+0x15b0,s['velocity']);n.doubles(BASE+0x15c8,s['world_acceleration']);n.doubles(BASE+0x15e0,s['omega'])
    n.doubles(BASE+0x1618,s['cached_velocity']);n.floats(BASE+0x8454,[s['air'][k] for k in ['alpha','beta','tas','speed_squared','mach','ias_u']])
    n.floats(BASE+0x1678,s['history']['wing_aoa']);n.floats(BASE+0x1694,s['delivered'])
    n.floats(BASE+0x1680,s['history']['body_angles']);n.floats(BASE+0x1688,s['history']['wing_cl'])
    n.floats(BASE+0x1690,[s['history']['spin']])
    n.floats(BASE+0x39f4,s['snapshot']);n.floats(BASE+0x87f4,s['trim_requested']);n.floats(BASE+0xa290,s['trim'])
    if 'control_state' in s:n.floats(BASE+0x3a1c,s['control_state'][1:])
    n.doubles(OWNER+0x25a48,s['engine_force']);n.doubles(OWNER+0x25a60,s['engine_moment'])
    n.floats(BASE+0x8514,requested)


def port_source(template,s,requested):
    p=copy.deepcopy(template)
    p['predictor']['f'].update(s['wing_normalization'])
    p['wrapper'].update(tas=s['air']['tas'],speed_squared=s['air']['speed_squared'],mach=s['air']['mach'],height=s['position'][1],engine_force=s['engine_force'],engine_moment=s['engine_moment'])
    p.update(time_constants=s['time_constants'],linear_rates=s['linear_rates'],trim_requested=s['trim_requested'],trim_actual=s['trim'],trim_cache=s['trim_cache'],
        indicated_airspeed=s['air']['ias_u'],longitudinal_speed=s['cached_velocity'][0],elevator_state=s['snapshot'][1],delivered=s['delivered'],wing_angles=s['history']['wing_aoa'],
        world_velocity=s['velocity'],world_acceleration=s['world_acceleration'],quaternion=s['quaternion'],pitch_rate=s['omega'][2],stored_yaw_rate=s['omega'][1],requested=requested)
    return p


def main(steps=240,scenario_count=6):
    n=InstructorNative();machine=CoupledAirborne();controls=Controls();delivery=DeliveryMachine();owner=EngineOwnerMachine()
    failures=[];counts={};scenarios=[];active=0
    def check(stage,a,e,where):
        counts[stage]=counts.get(stage,0)+1
        if a!=e:
            failures.append(dict(stage=stage,actual=a,expected=e,**where));raise AssertionError(stage+' '+str(where))
    try:
      for name in ['f_16a_block_15_adf','saab_jas39c']:
        fm=load(name);model=prepare(fm);cp=selected_properties(fm);ep=engine_properties(fm['EngineType0']);fp=fuel_properties(fm['Mass']);nozzle=prepare_nozzle(fm['Engine0']['Nozzle0'])
        fuel=[f32(v*.3) for v in fuel_capacities(fm)];mass=mass_evaluate(aircraft_properties(fm),fuel_by_system=fuel);strength=fm['Aerodynamics']['WingPlane']['Strength']
        ground_trim=[fm['AvailableControls'].get('has'+k+'TrimGroundControl',False) for k in ['Aileron','Elevator','Rudder']]
        for scenario in range(scenario_count):
            dt=f32([1/48,1/60,1/120][scenario%3]);speed=[120.,220.,320.][scenario%3];alpha=[2.,15.,28.][scenario%3];height=3000.
            n.setup(model,mass,speed=speed,alpha=alpha,height=height,dt=dt,mode_lane=True)
            actor=n.empty_payload_actor();n.qword(actor+0x2ef0,BASE)
            n.u.mem_write(INPUT+2,b'\1');n.u.mem_write(INPUT+0x29,b'\1');n.floats(INPUT+0x10,[0.]);n.floats(INPUT+0x44,[1.])
            n.u.mem_write(0x107d6fbc0,b'\1\1');n.u.mem_write(BASE+0x3658,b'\3');n.u.mem_write(0x107d6fbf5,b'\1')
            n.u.mem_write(OBJ+0x48,bytes([scenario%2]));n.u.mem_write(0x107d6fc2d,b'\0')
            n.floats(BASE+0x8514,[0.,-1.,0.]);n.invoke(0x104f70ef0,[actor,INPUT+0x40,INPUT+0x41,INPUT+0x42])
            template=extract_state(n,model,0.);ch=extract_history(n)
            q=orientation_increment([0.,0.,0.,1.],[0.,alpha,0.]);v=[speed,0.,0.];wind=[0.,0.,0.];extra=[0.,0.,0.]
            bv=world_to_body_air(q,v,wind,extra);air=cache(bv,height)
            initial=dict(position=[100.,height,300.],quaternion=q,velocity=v,omega=[0.,0.,0.],world_acceleration=[0.,0.,0.],cached_velocity=bv,air=air,
                history=dict(wing_aoa=[air['alpha']]*2,body_angles=[air['alpha'],0.],wing_cl=[0.,0.],spin=0.),flex=[[0.,0.],[0.,0.]],
                control_state=[0.,0.,0.],trim=[0.,0.,0.],trim_requested=[0.,0.,0.],trim_cache=[0.,0.,0.],snapshot=[0.,0.,0.],delivered=[0.,0.,0.],
                wing_normalization={o:template['predictor']['f'][o] for o in [0x8438,0x843c,0x8440]},time_constants=template['time_constants'],linear_rates=template['linear_rates'],controller_history=ch,engine_force=[0.,0.,0.],engine_moment=[0.,0.,0.],
                engine=dict(omega=mul(.95,model['engine']['max_omega']),health=1.,cylinders=25,mechanical=1.,extra_amplitude=0.,torque=0.,friction=0.,throttle=1.1,running=7,afterburner=True,vtol=0.,reverse=0.,rpm_limit_scale=1.,elapsed=100.,inactive_elapsed=0.,stop_reason=0))
            ns=copy.deepcopy(initial);ps=copy.deepcopy(initial);samples=[]
            for tick in range(1,steps+1):
                where=dict(aircraft=name,scenario=scenario,tick=tick)
                requested=[0.,-1. if scenario<3 else 1.,0.]
                write_controller(n,ns,requested);source=port_source(template,ps,requested)
                check('controller_source',extract_state(n,model,0.),source,where)
                pc=keyboard_step(model,source,ps['controller_history'],dt)
                captured=n.step();nc=extract_result(n)
                if nc!={k:pc[k] for k in nc}:
                    Path('analysis/instructor-full/coupled-controller-diagnostic.json').write_text(json.dumps(dict(source=source,entry_history=ps['controller_history'],native=captured,port=pc['diagnostics'],where=where),indent=2)+'\n')
                check('controller',nc,{k:pc[k] for k in nc},where);active+=pc['diagnostics']['recovery_active']
                nv,ncache=machine.air(ns,wind,extra);pv=world_to_body_air(ps['quaternion'],ps['velocity'],wind,extra);pa=cache(pv,ps['position'][1])
                check('air_state',[nv,ncache],[pv,[pa[k] for k in ['alpha','beta','tas','speed_squared','mach','ias_u']]],where)
                nr=controls.ranges(cp,ncache[-1],[True]*3,1.,True,False,ns['snapshot'][1],dt,nc['time_constants'])['ranges'];pr=authority_ranges(cp,pa['ias_u'],elevator_state=ps['snapshot'][1]);check('authority',nr,pr,where)
                throttle=f32(1.1)
                ni=dict(ns['engine'],delivered=ns['delivered'],trim_actual=nc['trim_actual'],trim_requested=nc['trim_requested']);pi=dict(ps['engine'],delivered=ps['delivered'],trim_actual=pc['trim_actual'],trim_requested=pc['trim_requested'])
                flags=dict(autotrim=True,ground_trim=ground_trim)
                nd=delivery.delivery(cp,dict(commands=ns['snapshot'],throttle=throttle,afterburner=True),ni,nr,dt,**flags)
                pd=deliver_keyboard_commands(cp,dict(commands=ps['snapshot'],throttle=throttle,afterburner=True),pi,pr,dt,**flags);check('snapshot_delivery',nd,pd,where)
                ne=dict(ns['engine'],throttle=nd['throttle'],afterburner=nd['afterburner']);pe=dict(ps['engine'],throttle=pd['throttle'],afterburner=pd['afterburner'])
                ea=(ep,model['engine'],nozzle,fp)
                en=owner.owner(*ea,ne,nv,ns['position'][1],mass['cog'],dt,position_tick_seed(ns['position'],tick+0x2a85e),fuel[0],fp['capacity'],ias_u=ncache[-1])
                ex=expected_owner(*ea,pe,pv,ps['position'][1],mass['cog'],dt,position_tick_seed(ps['position'],tick+0x2a85e),fuel[0],fp['capacity'],ias_u=pa['ias_u']);check('engine',en,ex,where)
                na=controls.actuator(cp,nc['commands'],nd['trim_requested'],nd['trim_actual'],ns['control_state'],nr,dt,nc['time_constants'],nc['linear_rates'],[True]*3,1.)
                ac=actuator_step(cp,pc['commands'],pd['trim_requested'],pd['trim_actual'],ps['control_state'],pr,dt,pc['time_constants'],pc['linear_rates']);check('actuator',na,{k:ac[k] for k in na},where)
                machine.initial_position=ns['position'];machine.initial_quaternion=ns['quaternion'];machine.world_velocity=ns['velocity'];machine.collision_radius=10.;machine.tick=tick;machine.flex=ns['flex']
                an=machine.call(model,nv,ns['omega'],mass,nd['delivered'],ns['position'][1],dt,ns['history'],throttle=throttle,full_update=True,quaternion=ns['quaternion'])
                ae=evaluate(model,pv,ps['omega'],mass,pd['delivered'],ps['position'][1],dt,ps['history'],oil_radiator=0.,throttle=throttle,height_agl=f32(ps['position'][1]),engine_vectors=[ex['aggregate_force'],ex['aggregate_moment']],engine_spin_factor=throttle,quaternion=ps['quaternion'])
                for key,ek in [('forces','component_forces'),('points','component_points')]:check('aero_'+key,an[key],{k:ae[ek][k] for k in an[key]},where)
                check('aero_history',an['history'],ae['history'],where);check('aero_moment',an['moment'],ae['raw_aero_moment'],where)
                assembly=machine.extend(en['aggregate_force'],en['aggregate_moment'],[0.]*3,[0.]*3,1.,fm.get('ExtThrustBaseMult',1.),dt);check('total_force_moment',[assembly['force'],assembly['moment']],[ae['force'],ae['stored_moment']],where)
                machine.integrate();actual=machine.finish()
                k=airborne_step(ps['position'],ps['velocity'],ps['quaternion'],ae['omega_for_flow'],ae['force'],ae['stored_moment'],mass['mass'],mass['inertia'],dt)
                pose=restore_small_pose_change(ps['position'],ps['quaternion'],k['position'],k['quaternion']);k.update(position=pose['position'],quaternion=pose['quaternion']);k['velocity'][1]=altitude_velocity_correction(k['position'][1],k['velocity'][1],dt)
                gravity=gravity_body(ps['quaternion'],mass['mass']);wy=[ae['component_forces'][w][1] for w in ['left_wing','right_wing']]
                wave=wing_observables(k['quaternion'],fm['Mass']['EmptyMass'],mul(f32(fm['WingWaveMassRel']),.5),fm['WingSpringDampJointMult'],strength['CritOverload'],wy,gravity[1],0.,mass['inertia'][0],model['geometry']['arm'][2],mass['mass'],pa['ias_u'],mul(f32(strength['VNE']),f32(1/3.6)),dt,ps['flex'])
                expected={key:k[key] for key in ['position','velocity','quaternion','omega','angular_acceleration','world_acceleration']};expected.update(wave);expected.update(wing_load=wing_load_ratios(wy,strength['CritOverload']),reaction_force=gravity,reaction_moment=[0.]*3)
                if limit_aerodynamic_force(ae['raw_aero_force'],mass['mass'])!=ae['raw_aero_force']:expected['wing_load']=[-1.,-1.]
                check('airborne_return',actual,expected,where)
                for s,r,a,c,d,e,ctl,bvel,air_result in [(ns,actual,an,na,nd,en,nc,nv,dict(zip(['alpha','beta','tas','speed_squared','mach','ias_u'],ncache))),(ps,expected,dict(history=ae['history']),ac,pd,ex,pc,pv,pa)]:
                    s.update({key:r[key] for key in ['position','velocity','quaternion','omega','world_acceleration']});s.update(time_constants=ctl['time_constants'],linear_rates=ctl['linear_rates'],wing_normalization=restored_wing_normalization(model['geometry']['areas']),history=a['history'],flex=r['flex_state'],control_state=c['state'],trim=c['trim'],trim_requested=d['trim_requested'],trim_cache=ctl['trim_cache'],snapshot=c['commands'],delivered=d['delivered'],engine=e['state'],engine_force=e['aggregate_force'],engine_moment=e['aggregate_moment'],controller_history=ctl['history'],cached_velocity=bvel,air=air_result)
                if tick%24==0 or tick==1:samples.append(dict(tick=tick,time=tick*dt,alpha=pa['alpha'],tas=pa['tas'],pitch_command=pc['commands'][1],pitch_delivered=pd['delivered'][1],pitch_rate=ps['omega'][2],recovery=pc['diagnostics']['recovery_active'],autotrim_success=pc['diagnostics']['autotrim']['success']))
            scenarios.append(dict(aircraft=name,scenario=scenario,steps=steps,dt=dt,initial_speed=speed,initial_alpha=alpha,pitch_request=requested[1],samples=samples,final_position=ps['position']))
            print(name,scenario,steps,'steps pass',flush=True)
    except AssertionError as error:print(str(error),flush=True)
    report=dict(binary_sha256=n.sha,counts=counts,active_recovery=active,scenarios=scenarios,failures=failures,
        scope='Independent controller history including prior dispatch-command cache, actuator/trim, primary delivery, engine RPM and vectors, component forces/moments, aerodynamic history, world kinematics/acceleration and wing observables chained against original kernels. Empty payload, fixed fuel/health, full pilot authority, manual clean devices, free air, two reference jets. Explicit one-step-held command snapshot and preceding air-cache convention; owner scheduling/spawn not replayed. Both original MouseAim calls execute with output axes disabled. This is closed-loop kernel-composition parity, NOT a validated game EM boundary or proof of settled turn performance.')
    out='analysis/instructor-full/coupled-validation'+('-smoke' if steps<120 else '')+'.json'
    Path(out).write_text(json.dumps(report,indent=2)+'\n');print(counts,'failures',len(failures),flush=True)
    if failures:raise SystemExit(1)

if __name__=='__main__':main(int(sys.argv[1]) if len(sys.argv)>1 else 240,int(sys.argv[2]) if len(sys.argv)>2 else 6)
