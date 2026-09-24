"""Independently chained native kernels versus recovered coupled aircraft math.

This is a verification fixture, not an EM solver. It explicitly supplies a
one-step-held primary command snapshot, nominal pilot strength and fixed fuel.
It does not pretend to execute the network/game-owner scheduler.
"""
import copy,json,math
from pathlib import Path
from unicorn.x86_const import *
from verify_airborne_return import AirborneReturn
from verify_aircraft_native import BASE,FRAME,END
from verify_primary_controls import Controls
from verify_snapshot_delivery import DeliveryMachine
from verify_engine_owner import EngineOwnerMachine,expected_owner
from control_snapshots import deliver_selected_jet_commands
from primary_controls import (selected_properties as controls_properties,authority_ranges,
                              actuator_step,delivered_commands,sensitivity_parameters,steady_commands)
from engine_supply import selected_properties as engine_properties,fuel_properties,wrapper_step
from jet_model import prepare_nozzle
from aircraft_model import prepare,evaluate
from mass_model import aircraft_properties,evaluate as mass_evaluate
from air_state import world_to_body_air,cache
from structural_limits import position_tick_seed,wing_load_ratios
from body_dynamics import gravity_body,limit_aerodynamic_force
from kinematics import airborne_step,orientation_increment,restore_small_pose_change,altitude_velocity_correction
from wing_observables import wing_observables
from component_assembly import f32,mul


class CoupledAirborne(AirborneReturn):
    def configure(self,u,a,size,data):
        super().configure(u,a,size,data)
        self.doubles(BASE+0x15b0,self.world_velocity)
    def air(self,state,wind,extra):
        self.doubles(BASE+0x1558,state['position']);self.floats(BASE+0x1570,state['quaternion'])
        self.doubles(BASE+0x15b0,state['velocity']);self.doubles(BASE+0x8478,wind);self.doubles(BASE+0x1630,extra)
        self.u.reg_write(UC_X86_REG_RDI,BASE+0x8478);self.u.reg_write(UC_X86_REG_RSI,BASE+0x1558)
        self.u.reg_write(UC_X86_REG_RDX,BASE+0x8138);self.u.reg_write(UC_X86_REG_RSP,FRAME)
        self.qword(FRAME,END);self.u.emu_start(0x101a33880,END,count=2000)
        return self.read(BASE+0x1618,3,'d'),self.read(BASE+0x8454,6)


def main():
    machine=CoupledAirborne();controls=Controls();engine=DeliveryMachine();owner=EngineOwnerMachine()
    failures=[];counts={};scenarios=[];dt=f32(1/48)
    def check(stage,a,e,**where):
        counts[stage]=counts.get(stage,0)+1
        if a!=e:
            failures.append(dict(stage=stage,actual=a,expected=e,**where))
            # Do not let a discrepancy turn subsequent history comparisons into
            # thousands of consequences of a single initial error.
            raise AssertionError(stage+' '+str(where))
    try:
        for name in ['f_16a_block_15_adf','saab_jas39c']:
            fm=json.loads(Path('references/fm-2.59.0.13/'+name+'.blkx').read_text());model=prepare(fm)
            cp=controls_properties(fm);ep=engine_properties(fm['EngineType0']);fp=fuel_properties(fm['Mass'])
            nozzle=prepare_nozzle(fm['Engine0']['Nozzle0']);strength=fm['Aerodynamics']['WingPlane']['Strength']
            for scenario in range(6):
                fuel_mass=f32(500+250*scenario)
                mass=mass_evaluate(aircraft_properties(fm),fuel_by_system=[fuel_mass])
                h=[1000.,4500.,8000.,10500.,1500.,12000.][scenario]
                q=orientation_increment([0.,0.,0.,1.],[0.,f32(2+scenario*2),f32(scenario*3)])
                initial=dict(position=[100.,h,300.],quaternion=q,velocity=[float(160+scenario*40),2.,-3.],omega=[0.,0.,0.],
                    history=dict(wing_aoa=[0.,0.],body_angles=[0.,0.],wing_cl=[0.,0.],spin=0.),
                    flex=[[0.,0.],[0.,0.]],control_state=[0.,0.,0.],trim=[0.,0.,0.],snapshot=[0.,0.,0.],delivered=[0.,0.,0.],
                    engine=dict(omega=mul(.6,model['engine']['max_omega']),health=1.,cylinders=25,mechanical=1.,extra_amplitude=0.,
                        torque=0.,friction=0.,throttle=.2,running=7,afterburner=False,vtol=0.,reverse=0.,rpm_limit_scale=1.,
                        elapsed=100.,inactive_elapsed=0.,stop_reason=0))
                ns=copy.deepcopy(initial);ps=copy.deepcopy(initial)
                response=sensitivity_parameters([f32(.3+scenario*.1)]*3);times=response['time_constants'];rates=response['linear_rates']
                wind=[-4.,2.,3.];extra=[.1,-.2,.3]
                for tick in range(1,121):
                    where=dict(aircraft=name,scenario=scenario,tick=tick)
                    nv,ncache=machine.air(ns,wind,extra);pv=world_to_body_air(ps['quaternion'],ps['velocity'],wind,extra)
                    pcache=cache(pv,ps['position'][1]);check('air_state',[nv,ncache],[pv,[pcache[k] for k in ['alpha','beta','tas','speed_squared','mach','ias_u']]],**where)
                    nr=controls.ranges(cp,ncache[-1],[True]*3,1.,True,False,ns['snapshot'][1],dt,times)['ranges']
                    pr=authority_ranges(cp,pcache['ias_u'],elevator_state=ps['snapshot'][1]);check('authority',nr,pr,**where)
                    throttle=f32(.2 if tick<25 else 1.1 if tick<90 else .75)
                    requested=[f32(.015) if cp['trim_available'][0] else 0.,f32(-.04),f32(.01)]
                    ninput=dict(ns['engine'],delivered=ns['delivered'],trim_actual=ns['trim'],trim_requested=requested)
                    pinput=dict(ps['engine'],delivered=ps['delivered'],trim_actual=ps['trim'],trim_requested=requested)
                    ndel=engine.delivery(cp,dict(commands=ns['snapshot'],throttle=throttle,afterburner=throttle>1.),ninput,nr,dt)
                    pdel=deliver_selected_jet_commands(cp,dict(commands=ps['snapshot'],throttle=throttle,afterburner=throttle>1.),pinput,pr,dt)
                    check('complete_snapshot_delivery',ndel,pdel,**where);nd,pd=ndel['delivered'],pdel['delivered']
                    ne=dict(ns['engine'],**{k:ndel[k] for k in ['throttle','afterburner','vtol','reverse']})
                    pe=dict(ps['engine'],**{k:pdel[k] for k in ['throttle','afterburner','vtol','reverse']})
                    nseed=position_tick_seed(ns['position'],tick+0x2a85e);pseed=position_tick_seed(ps['position'],tick+0x2a85e)
                    ea=(ep,model['engine'],nozzle,fp)
                    en=owner.owner(*ea,ne,nv,ns['position'][1],mass['cog'],dt,nseed,fuel_mass,fp['capacity'],ias_u=ncache[-1])
                    ex=expected_owner(*ea,pe,pv,ps['position'][1],mass['cog'],dt,pseed,fuel_mass,fp['capacity'],ias_u=pcache['ias_u'])
                    check('engine_owner',en,ex,**where)
                    sticks=[f32(.1*math.sin(tick*.035)) if scenario%2 else 0.,f32(-.06 if tick<60 else .06),f32(.03*math.sin(tick*.02))]
                    nc=controls.actuator(cp,sticks,ndel['trim_requested'],ndel['trim_actual'],ns['control_state'],nr,dt,times,rates,[True]*3,1.)
                    pc=actuator_step(cp,sticks,pdel['trim_requested'],pdel['trim_actual'],ps['control_state'],pr,dt,times,rates)
                    check('actuator',nc,{k:pc[k] for k in nc},**where)
                    machine.initial_position=ns['position'];machine.initial_quaternion=ns['quaternion'];machine.world_velocity=ns['velocity']
                    machine.collision_radius=10.;machine.tick=tick;machine.flex=ns['flex']
                    an=machine.call(model,nv,ns['omega'],mass,nd,ns['position'][1],dt,ns['history'],throttle=throttle,full_update=True)
                    ae=evaluate(model,pv,ps['omega'],mass,pd,ps['position'][1],dt,ps['history'],oil_radiator=0.,throttle=throttle,
                                height_agl=f32(ps['position'][1]),scalar_thrust=ex['force'][0],engine_spin_factor=throttle)
                    for key,expected in [('forces',ae['component_forces']),('points',ae['component_points'])]:
                        check('aerodynamic_'+key,an[key],{k:expected[k] for k in an[key]},**where)
                    check('aerodynamic_history',an['history'],ae['history'],**where)
                    check('raw_aero_moment',an['moment'],ae['raw_aero_moment'],**where)
                    assembly=machine.extend(en['aggregate_force'],en['aggregate_moment'],[0.]*3,[0.]*3,1.,fm.get('ExtThrustBaseMult',1.),dt)
                    check('total_force_moment',[assembly['force'],assembly['moment']],[ae['force'],ae['stored_moment']],**where)
                    machine.integrate();actual=machine.finish()
                    k=airborne_step(ps['position'],ps['velocity'],ps['quaternion'],ae['omega_for_flow'],ae['force'],ae['stored_moment'],mass['mass'],mass['inertia'],dt)
                    pose=restore_small_pose_change(ps['position'],ps['quaternion'],k['position'],k['quaternion'])
                    k.update(position=pose['position'],quaternion=pose['quaternion']);k['velocity'][1]=altitude_velocity_correction(k['position'][1],k['velocity'][1],dt)
                    gravity=gravity_body(ps['quaternion'],mass['mass']);wy=[ae['component_forces'][n][1] for n in ['left_wing','right_wing']]
                    wave=wing_observables(k['quaternion'],fm['Mass']['EmptyMass'],mul(f32(fm['WingWaveMassRel']),.5),fm['WingSpringDampJointMult'],strength['CritOverload'],wy,gravity[1],0.,mass['inertia'][0],model['geometry']['arm'][2],mass['mass'],pcache['ias_u'],mul(f32(strength['VNE']),f32(1/3.6)),dt,ps['flex'])
                    expected={key:k[key] for key in ['position','velocity','quaternion','omega','angular_acceleration','world_acceleration']}
                    expected.update(wave);expected.update(wing_load=wing_load_ratios(wy,strength['CritOverload']),reaction_force=gravity,reaction_moment=[0.]*3)
                    if limit_aerodynamic_force(ae['raw_aero_force'],mass['mass'])!=ae['raw_aero_force']:expected['wing_load']=[-1.,-1.]
                    check('complete_airborne_return',actual,expected,**where)
                    for s,result,aero,control,delivered,eng in [(ns,actual,an,nc,nd,en),(ps,expected,dict(history=ae['history']),pc,pd,ex)]:
                        s.update({key:result[key] for key in ['position','velocity','quaternion','omega']})
                        s.update(history=aero['history'],flex=result['flex_state'],control_state=control['state'],trim=control['trim'],
                                 snapshot=control['commands'],delivered=delivered,engine=eng['state'])
                scenarios.append(dict(aircraft=name,scenario=scenario,steps=120,fuel_mass=fuel_mass,mass=mass['mass'],
                                      final_position=ns['position'],final_omega=ns['omega']))
    except AssertionError as error:print(str(error))
    report=dict(binary_sha256=machine.sha,dt=dt,counts=counts,scenarios=scenarios,failures=failures,
        limitations='Coupled native kernels with independently propagated world pose, velocity, angular rate, aerodynamic histories, trim/actuator/delivery, RPM/mechanical state and wing flex. Fixed mass/full health and full pilot authority as requested. Complete primary/engine snapshot consumer101a4e5f0 executes; explicit held primary snapshot, supplied engine commands and six response profiles. Network/owner publication is not executed. Complete engine owner101a155c0 and outer1019fa1b0 run, with thermal/fuel-bookkeeping/damage routines explicitly bypassed for the fixed-fuel/intact scope; original force wrapper, running lifecycle, scalar/nozzle code and double owner aggregation produce engine vectors used by native body assembly. Air cache, original aero/body normal return and contact empty path execute. Existing math/property/TLS hooks remain. This is kernel composition/replay evidence, not a live-game flight or solved EM equilibrium.')
    Path('analysis/coupled-airborne-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(counts,'FAILURES',len(failures));print(json.dumps(failures[:1],indent=2))
    if failures:raise SystemExit(1)

if __name__=='__main__':main()
