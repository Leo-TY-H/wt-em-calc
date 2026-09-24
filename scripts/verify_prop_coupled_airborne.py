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
from primary_controls import (selected_properties as controls_properties,authority_ranges,
                              actuator_step,sensitivity_parameters)
from aircraft_model import prepare,evaluate
from mass_model import aircraft_properties,evaluate as mass_evaluate
from advanced_mass import evaluate as advanced_mass_evaluate
from air_state import world_to_body_air,cache
from structural_limits import position_tick_seed,wing_load_ratios
from body_dynamics import gravity_body,limit_aerodynamic_force
from kinematics import airborne_step,orientation_increment,restore_small_pose_change,altitude_velocity_correction
from wing_observables import wing_observables
from fm_loader import normalize
from propulsion_model import prepare as prop_prepare
from prop_owner import owner_step
from prop_owner_native import PropOwnerNative
from prop_commands_native import PropCommandsNative
from prop_commands import deliver
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
    machine=CoupledAirborne();controls=Controls();owner=PropOwnerNative();delivery=PropCommandsNative()
    failures=[];counts={};scenarios=[];dt=f32(1/48)
    bf_parts=json.loads(Path('analysis/bf-109f-4-native-mass-parts.json').read_text())['records']
    def check(stage,a,e,**where):
        counts[stage]=counts.get(stage,0)+1
        if a!=e:
            failures.append(dict(stage=stage,actual=a,expected=e,**where))
            # Do not let a discrepancy turn subsequent history comparisons into
            # thousands of consequences of a single initial error.
            raise AssertionError(stage+' '+str(where))
    try:
        for name in ['yak-3','bf-109f-4']:
            fm=normalize(json.loads(Path('references/fm-2.59.0.13/'+name+'.blkx').read_text()));model=prepare(fm)
            cp=controls_properties(fm);pp=prop_prepare(fm)
            strength=fm['Aerodynamics']['WingPlane']['Strength']
            for scenario in range(6):
                fuel_mass=f32(100+25*scenario)
                mass=(mass_evaluate(aircraft_properties(fm),fuel_by_system=[fuel_mass]) if name=='yak-3' else
                      advanced_mass_evaluate(aircraft_properties(fm),fuel_by_system=[fuel_mass],
                                             mass_parts=bf_parts,fuel_by_tank=[fuel_mass]))
                h=[1000.,2500.,4000.,6500.,1500.,8000.][scenario]
                q=orientation_increment([0.,0.,0.,1.],[0.,f32(2+scenario*2),f32(scenario*3)])
                initial=dict(position=[100.,h,300.],quaternion=q,velocity=[float(75+scenario*15),2.,-3.],omega=[0.,0.,0.],
                    history=dict(wing_aoa=[0.,0.],body_angles=[0.,0.],wing_cl=[0.,0.],spin=0.),
                    flex=[[0.,0.],[0.,0.]],control_state=[0.,0.,0.],trim=[0.,0.,0.],snapshot=[0.,0.,0.],delivered=[0.,0.,0.],
                    engine=dict(omega=270.,previous_omega=270.,command=1.,auto=name=='bf-109f-4',prop=dict(pitch=.5),engine=dict(throttle=1.,mixture=.3)))
                ns=copy.deepcopy(initial);ps=copy.deepcopy(initial)
                response=sensitivity_parameters([f32(.3+scenario*.1)]*3);times=response['time_constants'];rates=response['linear_rates']
                wind=[-4.,2.,3.];extra=[.1,-.2,.3]
                for tick in range(1,121):
                    where=dict(aircraft=name,scenario=scenario,tick=tick)
                    nv,ncache=machine.air(ns,wind,extra);pv=world_to_body_air(ps['quaternion'],ps['velocity'],wind,extra)
                    pcache=cache(pv,ps['position'][1]);check('air_state',[nv,ncache],[pv,[pcache[k] for k in ['alpha','beta','tas','speed_squared','mach','ias_u']]],**where)
                    nr=controls.ranges(cp,ncache[-1],[True]*3,1.,True,False,ns['snapshot'][1],dt,times)['ranges']
                    pr=authority_ranges(cp,pcache['ias_u'],elevator_state=ps['snapshot'][1]);check('authority',nr,pr,**where)
                    throttle=f32(.8 if tick<25 else (1. if name=='yak-3' else 1.1) if tick<90 else .9)
                    requested=[0.,f32(-.04),0.]
                    command=dict(throttle=throttle,afterburner=throttle>1.,auto=name=='bf-109f-4',command=1.,mixture=.3,gear=int(name=='yak-3' and scenario>=3),radiator=0.,oil_radiator=0.)
                    nin=dict(engine=ns['engine']['engine'],command=ns['engine']['command'],delivered=ns['delivered'],trim_actual=ns['trim'],trim_requested=requested)
                    pin=dict(engine=ps['engine']['engine'],command=ps['engine']['command'],delivered=ps['delivered'],trim_actual=ps['trim'],trim_requested=requested)
                    ndel=delivery.delivery(cp,pp,dict(command,commands=ns['snapshot']),nin,nr,dt)
                    pdel=deliver(cp,pp,dict(command,commands=ps['snapshot']),pin,pr,dt)
                    check('complete_snapshot_delivery',ndel,pdel,**where);nd,pd=ndel['delivered'],pdel['delivered']
                    ne=dict(ns['engine'],engine=ndel['engine'],command=ndel['command'],auto=ndel['auto'])
                    pe=dict(ps['engine'],engine=pdel['engine'],command=pdel['command'],auto=pdel['auto'])
                    nseed=position_tick_seed(ns['position'],tick+0x2a85e);pseed=position_tick_seed(ps['position'],tick+0x2a85e)
                    en=owner.owner(pp,ne,nv,ns['position'][1],ns['omega'],mass['cog'],dt,nseed)
                    ex=owner_step(pp,pe,pv,ps['position'][1],ps['omega'],mass['cog'],dt,pseed)
                    check('propulsion_owner',en,ex,**where)
                    sticks=[f32(.1*math.sin(tick*.035)) if scenario%2 else 0.,f32(-.06 if tick<60 else .06),f32(.03*math.sin(tick*.02))]
                    nc=controls.actuator(cp,sticks,ndel['trim_requested'],ndel['trim_actual'],ns['control_state'],nr,dt,times,rates,[True]*3,1.)
                    pc=actuator_step(cp,sticks,pdel['trim_requested'],pdel['trim_actual'],ps['control_state'],pr,dt,times,rates)
                    check('actuator',nc,{k:pc[k] for k in nc},**where)
                    machine.initial_position=ns['position'];machine.initial_quaternion=ns['quaternion'];machine.world_velocity=ns['velocity']
                    machine.collision_radius=10.;machine.tick=tick;machine.flex=ns['flex']
                    an=machine.call(model,nv,ns['omega'],mass,nd,ns['position'][1],dt,ns['history'],throttle=throttle,full_update=True,engine_wash=en['engine_wash'])
                    ae=evaluate(model,pv,ps['omega'],mass,pd,ps['position'][1],dt,ps['history'],oil_radiator=0.,throttle=throttle,quaternion=ps['quaternion'],
                                height_agl=f32(ps['position'][1]),engine_vectors=(ex['aggregate_force'],ex['aggregate_moment']),engine_wash=ex['engine_wash'],engine_angular_momentum=ex['engine_angular_momentum'],engine_spin_factor=throttle)
                    for key,expected in [('forces',ae['component_forces']),('points',ae['component_points'])]:
                        check('aerodynamic_'+key,an[key],{k:expected[k] for k in an[key]},**where)
                    check('aerodynamic_history',an['history'],ae['history'],**where)
                    check('raw_aero_moment',an['moment'],ae['raw_aero_moment'],**where)
                    assembly=machine.extend(en['aggregate_force'],en['aggregate_moment'],[0.]*3,[0.]*3,1.,fm.get('ExtThrustBaseMult',1.),dt,engine_angular_momentum=en['engine_angular_momentum'])
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
                                 snapshot=control['commands'],delivered=delivered,engine=dict(s['engine'],omega=eng['omega'],previous_omega=eng['previous_omega'],prop=eng['prop'],engine=dict(s['engine']['engine'],**eng['engine'])))
                scenarios.append(dict(aircraft=name,scenario=scenario,steps=120,fuel_mass=fuel_mass,mass=mass['mass'],
                                      cog=mass['cog'],inertia=mass['inertia'],
                                      final_position=ns['position'],final_omega=ns['omega']))
    except AssertionError as error:print(str(error))
    report=dict(binary_sha256=machine.sha,dt=dt,counts=counts,scenarios=scenarios,failures=failures,
        limitations='Independently propagated native and Python world pose, velocity, angular rate, primary controls, shaft RPM, governor, regulator, wake, aero history and flex. Original complete prop owner, transmission, piston and propeller; original air cache, aero/body normal return, empty airborne contact path. Complete selected manual-engine snapshot delivery with prop auto/manual, quantization, availability and compressor reset; held snapshots; no network scheduler. Fixed fuel/health, closed radiators, nominal pilot authority. Yak nonadvanced mass calculated; Bf advanced mass calculated from installed collision geometry and original mass producer, checked in mesh-mass-validation.json; default intact pose without ammunition/payload. Rounded libm and existing property/TLS hooks; native property loading independently checked in separate reports; no live-game trajectory or solved flight equilibrium.')
    Path('analysis/prop-coupled-airborne-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(counts,'FAILURES',len(failures));print(json.dumps(failures[:1],indent=2))
    if failures:raise SystemExit(1)

if __name__=='__main__':main()
