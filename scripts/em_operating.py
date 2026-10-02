import math
import numpy as np
from em_cancellation import check as check_cancel
from aircraft_model import evaluate,condition_properties,replay_propulsion
from air_state import cache,world_to_body_air
from body_dynamics import angular_acceleration,G
from component_assembly import f32
from primary_controls import authority_ranges
from kinematics import airborne_step,altitude_velocity_correction
from em_numeric_core import closure


def flight_condition(self, speed, load, x):
    from em_solver import turn_geometry
    alpha,bank=x[:2]; a=math.radians(alpha)
    steady=self.config.get('aircraft_trim_mode','discrete')=='quasi_steady'
    evaluation_dt=self.dt
    velocity=[f32(speed*math.cos(a)),f32(-speed*math.sin(a)),0.]
    air=cache(velocity,self.config['altitude_m'])
    geometry=turn_geometry(alpha,bank,speed,load,self.dt,air['ias_u'],self.sideslip_attitude_deg)

    velocity=world_to_body_air(geometry['quaternion'],[speed,0.,0.])
    air=cache(velocity,self.config['altitude_m'])
    geometry=turn_geometry(alpha,bank,speed,load,self.dt,air['ias_u'],self.sideslip_attitude_deg)
    return velocity,air,geometry,evaluation_dt


def operating_point(self, speed, load, x, propulsion_override=None,canonical_propulsion=False,cycle_seconds=60.,certify_stationary=False,phase_certificate=False,propulsion_sample=None,*,_search=False,_steady_history=False):
    from em_solver import turn_geometry,command_allocation,DEFAULTS
    check_cancel()
    velocity,air,geometry,evaluation_dt=flight_condition(self,speed,load,x)
    steady=self.config.get('aircraft_trim_mode','discrete')=='quasi_steady'
    ranges=authority_ranges(self.controls,air['ias_u'])
    flaps=self.flaps


    canonical=command_allocation(self.controls,x[2:],ranges,DEFAULTS)
    commands=canonical['allocated_commands'] if canonical['reachable'] else list(map(f32,x[2:]))
    allocation=command_allocation(self.controls,commands,ranges,self.config)
    propulsion=(propulsion_override if propulsion_override is not None else propulsion_sample if propulsion_sample is not None else
                self.engine.condition(tuple(velocity),tuple(geometry['omega']),flaps,speed,canonical_propulsion,
                    cycle_seconds,certify_stationary,phase_certificate)) if self.is_prop else None
    vectors=(propulsion['force'],propulsion['moment']) if propulsion is not None else self.engine.vectors(velocity[0],flaps)
    engine_flow=propulsion['wash'] if propulsion is not None else (0.,0.)
    engine_momentum=propulsion['angular_momentum'] if propulsion is not None else (0.,0.,0.)
    history=dict(wing_aoa=[air['alpha']]*2,wing_cl=[0.,0.],body_angles=[air['alpha'],air['beta']],spin=0.)
    history_error=math.inf
    phases=propulsion.get('cycle_samples') if propulsion is not None and propulsion_override is None else None
    phases=phases or [None]


    phase_aero={};aircraft_condition={}
    def evaluate_phase(phase,history,seed_only=False):


        angles=(0.,0.) if constant_geometry and finite_memory(history) else tuple(history['wing_aoa'])
        phase_key=((tuple(phase[9:]) if self.config['torque_gyro'] else (phase[9],0.))+angles+tuple(history['wing_cl'])+
                   tuple(history['body_angles'])+(history['spin'],)) if phase is not None else None
        pass
        if not seed_only and phase_key is not None and phase_key in phase_aero:
            r=replay_propulsion(self.model,phase_aero[phase_key],self.mass,(phase[:3],phase[3:6]),phase[6:9],torque_gyro=self.config['torque_gyro'])
        else:
            r=evaluate(self.model,velocity,geometry['omega'].tolist(),self.mass,allocation['commands'],
                self.config['altitude_m'],evaluation_dt,history,oil_radiator=0.,flaps=flaps,gear=self.gear,throttle=self.config['throttle'],
                height_agl=1e6,engine_vectors=(phase[:3],phase[3:6]) if phase is not None else vectors,
                engine_spin_factor=f32(self.config['throttle']),quaternion=geometry['quaternion'],torque_gyro=self.config['torque_gyro'],
                engine_wash=phase[9:] if phase is not None else engine_flow,
                engine_angular_momentum=phase[6:9] if phase is not None else engine_momentum,
                _condition_cache=aircraft_condition,_history_seed_only=seed_only,_air_state=(None))
            if not seed_only and phase_key is not None:
                if len(phase_aero)>=max(256,len(phases)):phase_aero.pop(next(iter(phase_aero)))
                phase_aero[phase_key]=r
        return r


    polar=condition_properties(self.model,air['mach'],flaps)[1]
    shifts=self.model['geometry']['aoa_shift_add']
    def finite_memory(h):
        return h['spin']==0. and all(polar['aoaCritL']<=angle<=polar['aoaCritH'] for angle in h['wing_aoa'])
    constant_geometry=not shifts or all(y==shifts[0][1] for _,y in shifts)
    direct_history=not steady and len(phases)>16 and constant_geometry and finite_memory(history)
    initial_history=history
    steady_attempt=not steady and _steady_history and len(phases)==1 and constant_geometry and finite_memory(history)
    steady_used=False
    if steady_attempt:
        seed=evaluate_phase(phases[0],history,seed_only=True)['history']
        if finite_memory(seed):
            replay=evaluate_phase(phases[0],seed)


            if replay['history']==seed:
                history=seed;steady_used=True
    if direct_history:
        seed=evaluate_phase(phases[-1],history)['history']
        if finite_memory(seed):history=seed
        else:direct_history=False
    iteration=0
    while iteration<12:
        cycle_input=history;phase_results=[];branch_ok=True
        for phase in phases:
            r=replay if steady_used else evaluate_phase(phase,history)
            phase_results.append(r);history=r['history']
            if direct_history and not finite_memory(history):
                branch_ok=False;break
        if direct_history and not branch_ok:
            history=initial_history;direct_history=False
            continue
        new=r['history']
        pass
        history_error=max(abs(new[k][i]-cycle_input[k][i]) for k in ['wing_aoa','wing_cl','body_angles'] for i in range(2))
        history_error=max(history_error,abs(new['spin']-cycle_input['spin']))
        if history_error<1e-6:break


        if new['spin']>0 and iteration>=3:break
        iteration+=1
    if len(phase_results)>1:


        r=dict(r)
        for key in ['force','stored_moment','raw_aero_force','raw_aero_moment','engine_force','engine_moment']:
            r[key]=[sum(v[key][i] for v in phase_results)/len(phase_results) for i in range(3)]
        for key in ['component_forces','component_points']:
            r[key]={k:[sum(v[key][k][i] for v in phase_results)/len(phase_results) for i in range(3)] for k in r[key]}
    force=np.asarray(r['force']); processed=np.asarray(r['omega_for_flow'])
    pass
    residual,rate_residual=closure(self.numeric_parameters,speed,force,processed,r['stored_moment'],
        geometry['omega'],geometry['forward'],geometry['up'],geometry['lateral'],
        geometry['normal'],geometry['side'],geometry['turn_rate'])
    polar=condition_properties(self.model,air['mach'],flaps)[1]
    phase_angles=[a for phase in phase_results for a in phase['history']['wing_aoa']]


    stall_margin=polar['aoaCritH']-max(phase_angles)
    negative_stall_margin=min(phase_angles)-polar['aoaCritL']
    strength=self.model['geometry']['strength']['force']
    wing_ratios=[max(max(phase['component_forces'][side][1]/strength[0],phase['component_forces'][side][1]/strength[1])
                    for phase in phase_results) for side in ['left_wing','right_wing']]
    certificate=(propulsion or {}).get('stationarity') or {}
    force_uncertainty=certificate.get('force_mean_uncertainty_g',0.)
    angular_uncertainty=np.asarray(certificate.get('angular_mean_uncertainty_rad_s2',[0.]*3))
    if force_uncertainty or np.any(angular_uncertainty):


        if len(phase_results)>1:
            split=len(phase_results)//2
            blocks=(phase_results[:split],phase_results[split:])
            means=[np.mean([p['force'] for p in block],axis=0) for block in blocks]
            force_uncertainty=max(force_uncertainty,float(np.linalg.norm(means[1]-means[0]))/self.weight)
            moments=[np.mean([p['stored_moment'] for p in block],axis=0) for block in blocks]
            angular_uncertainty=np.maximum(angular_uncertainty,abs(moments[1]-moments[0])/self.mass['inertia'])
        force_uncertainty*=1.+abs(math.tan(geometry['turn_rate']*self.dt))
    value=dict(speed=float(speed),residual=residual,rate_residual=rate_residual,force_error_g=max(abs(residual[0]),abs(residual[1])),
                force_mean_uncertainty_g=force_uncertainty,angular_mean_uncertainty_rad_s2=angular_uncertainty,
                equilibrium_coordinates=list(x),equilibrium_load_g=float(load),
                history_error=history_error,history_input=cycle_input,result=r,geometry=geometry,
                propulsion_phase_frames=len(phases) if phases[0] is not None else 0,


                phase_results=[dict(air=p['air'],history=p['history']) for p in phase_results] if self.config['instructor'] else [],
                maximum_wing_load_ratios=wing_ratios,
                allocation=allocation,velocity=velocity,stall_margin=stall_margin,
                negative_stall_margin=negative_stall_margin,flaps=flaps,gear=self.gear,instructor=None,propulsion=propulsion,
                ps=None,ps_continuous=None,
                kinematic=None,altitude_correction=None,history_iterations=min(iteration+1,12),
                search_only=_search,steady_history_attempted=steady_attempt,steady_history_used=steady_used)
    return value if _search else complete_reporting(self,value)


def complete_reporting(self,value):
    if value['kinematic'] is not None:return value
    speed=value['speed'];geometry=value['geometry'];r=value['result']
    force=np.asarray(r['force'])
    ps_continuous=float(force.dot(geometry['forward'])*speed/self.weight)
    kinematic=airborne_step([0.,self.config['altitude_m'],0.],[speed,0.,0.],geometry['quaternion'],
                        r['omega_for_flow'],r['force'],r['stored_moment'],self.mass['mass'],self.mass['inertia'],self.dt)
    vy=kinematic['velocity'][1]
    kinematic['velocity'][1]=altitude_velocity_correction(kinematic['position'][1],vy,self.dt)
    altitude_correction=kinematic['velocity'][1]!=vy
    ps=((kinematic['position'][1]-self.config['altitude_m'])+
           (sum(v*v for v in kinematic['velocity'])-speed*speed)/(2*float(G)))/self.dt
    value.update(ps=ps,ps_continuous=ps_continuous,kinematic=kinematic,
                 altitude_correction=altitude_correction,search_only=False)
    return value


def search_point(self,speed,load,x,**kwargs):
    return operating_point(self,speed,load,x,_search=True,**kwargs)
