"""Full aircraft residual and native phase consumer for numerical EM solves.

Kept separately so the existing exact backend can compile the repeated phase
and history loops. Trim search and physical acceptance stay in em_solver.
"""
import math
import numpy as np
from em_cancellation import check as check_cancel
from aircraft_model import evaluate,condition_properties,replay_propulsion
from air_state import cache,world_to_body_air
from body_dynamics import angular_acceleration,G
from component_assembly import f32
from primary_controls import authority_ranges
from kinematics import airborne_step,altitude_velocity_correction


def operating_point(self, speed, load, x, propulsion_override=None,canonical_propulsion=False,cycle_seconds=60.,certify_stationary=False,phase_certificate=False,propulsion_sample=None):
    from em_solver import turn_geometry,command_allocation,DEFAULTS
    check_cancel()
    alpha,bank=x[:2]; a=math.radians(alpha)
    velocity=[f32(speed*math.cos(a)),f32(-speed*math.sin(a)),0.]
    air=cache(velocity,self.config['altitude_m'])
    geometry=turn_geometry(alpha,bank,speed,load,self.dt,air['ias_u'],self.sideslip_attitude_deg)
    # Produce exactly the body cache which this world velocity and stored
    # quaternion yield. Analytic sin/cos velocity can differ by an ulp;
    # some recovered transonic Cm polynomials amplify that difference.
    velocity=world_to_body_air(geometry['quaternion'],[speed,0.,0.])
    air=cache(velocity,self.config['altitude_m'])
    geometry=turn_geometry(alpha,bank,speed,load,self.dt,air['ias_u'],self.sideslip_attitude_deg)
    ranges=authority_ranges(self.controls,air['ias_u'])
    flaps=self.flaps
    # Canonical native command rounding uses the same full available trim
    # convention for every authority experiment. Restricted trim changes
    # the reachability test, never the force-model inputs at a given state.
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
    # Scope this exact cache to a single immutable flight/control condition.
    # Different retained engine states often give the same wash/history.
    # Reuse only their aerodynamic work; assemble every propulsion phase.
    phase_aero={}
    for iteration in range(12):
        cycle_input=history;phase_results=[]
        for phase in phases:
            phase_key=((tuple(phase[9:]) if self.config['torque_gyro'] else (phase[9],0.))+tuple(history['wing_aoa'])+tuple(history['wing_cl'])+
                       tuple(history['body_angles'])+(history['spin'],)) if phase is not None else None
            if phase_key is not None and phase_key in phase_aero:
                r=replay_propulsion(self.model,phase_aero[phase_key],self.mass,(phase[:3],phase[3:6]),phase[6:9],torque_gyro=self.config['torque_gyro'])
            else:
                r=evaluate(self.model,velocity,geometry['omega'].tolist(),self.mass,allocation['commands'],
                    self.config['altitude_m'],self.dt,history,oil_radiator=0.,flaps=flaps,gear=self.gear,throttle=self.config['throttle'],
                    height_agl=1e6,engine_vectors=(phase[:3],phase[3:6]) if phase is not None else vectors,
                    engine_spin_factor=f32(self.config['throttle']),quaternion=geometry['quaternion'],torque_gyro=self.config['torque_gyro'],
                    engine_wash=phase[9:] if phase is not None else engine_flow,
                    engine_angular_momentum=phase[6:9] if phase is not None else engine_momentum)
                if phase_key is not None:
                    if len(phase_aero)>=max(256,len(phases)):phase_aero.pop(next(iter(phase_aero)))
                    phase_aero[phase_key]=r
            phase_results.append(r);history=r['history']
        new=r['history']
        history_error=max(abs(new[k][i]-cycle_input[k][i]) for k in ['wing_aoa','wing_cl','body_angles'] for i in range(2))
        history_error=max(history_error,abs(new['spin']-cycle_input['spin']))
        if history_error<1e-6:break
        # Post-stall states are excluded, but retain a bounded smooth-ish
        # residual during the optimizer's exploratory steps.
        if new['spin']>0 and iteration>=3:break
    if len(phase_results)>1:
        # Average AFTER the complete nonlinear aircraft consumer. In
        # particular, mean propwash is not substituted into the tail.
        r=dict(r)
        for key in ['force','stored_moment','raw_aero_force','raw_aero_moment','engine_force','engine_moment']:
            r[key]=[sum(v[key][i] for v in phase_results)/len(phase_results) for i in range(3)]
        for key in ['component_forces','component_points']:
            r[key]={k:[sum(v[key][k][i] for v in phase_results)/len(phase_results) for i in range(3)] for k in r[key]}
    force=np.asarray(r['force']); processed=np.asarray(r['omega_for_flow'])
    acceleration=np.asarray(angular_acceleration(processed,self.mass['inertia'],r['stored_moment']))
    rate_residual=(processed+acceleration*self.dt-geometry['omega'])/self.dt
    # Enforce the requested *finite-step velocity heading*, including its
    # change in speed. At Ps=0 this includes the tangential centripetal
    # correction that forward Euler needs to hold speed exactly.
    tangential_acceleration=force.dot(geometry['forward'])/self.mass['mass']
    lateral_acceleration=(speed/self.dt+tangential_acceleration)*math.tan(geometry['turn_rate']*self.dt)
    required=self.weight*geometry['up']+self.mass['mass']*lateral_acceleration*geometry['lateral']
    f_error=(force-required)/self.weight
    residual=np.array([f_error.dot(geometry['normal']),f_error.dot(geometry['side']),*(rate_residual/.1)])
    polar=condition_properties(self.model,air['mach'],flaps)[1]
    phase_angles=[a for phase in phase_results for a in phase['history']['wing_aoa']]
    # Chart policy excludes only positive-AoA stall. Keep the native polar,
    # negative-angle forces and history evolution above unchanged; ignoring
    # the lower critical angle is not a linear extension of the aerodynamics.
    stall_margin=polar['aoaCritH']-max(phase_angles)
    negative_stall_margin=min(phase_angles)-polar['aoaCritL']
    strength=self.model['geometry']['strength']['force']
    wing_ratios=[max(max(phase['component_forces'][side][1]/strength[0],phase['component_forces'][side][1]/strength[1])
                    for phase in phase_results) for side in ['left_wing','right_wing']]
    kinematic=airborne_step([0.,self.config['altitude_m'],0.],[speed,0.,0.],geometry['quaternion'],
                            r['omega_for_flow'],r['force'],r['stored_moment'],self.mass['mass'],self.mass['inertia'],self.dt)
    vy=kinematic['velocity'][1]
    kinematic['velocity'][1]=altitude_velocity_correction(kinematic['position'][1],vy,self.dt)
    ps_native=((kinematic['position'][1]-self.config['altitude_m'])+
               (sum(v*v for v in kinematic['velocity'])-speed*speed)/(2*float(G)))/self.dt
    certificate=(propulsion or {}).get('stationarity') or {}
    force_uncertainty=certificate.get('force_mean_uncertainty_g',0.)
    angular_uncertainty=np.asarray(certificate.get('angular_mean_uncertainty_rad_s2',[0.]*3))
    if force_uncertainty or np.any(angular_uncertainty):
        # Include the nonlinear aircraft consumer, not just engine output,
        # in the uncertainty budget of an aperiodic native mean.
        if len(phase_results)>1:
            split=len(phase_results)//2
            blocks=(phase_results[:split],phase_results[split:])
            means=[np.mean([p['force'] for p in block],axis=0) for block in blocks]
            force_uncertainty=max(force_uncertainty,float(np.linalg.norm(means[1]-means[0]))/self.weight)
            moments=[np.mean([p['stored_moment'] for p in block],axis=0) for block in blocks]
            angular_uncertainty=np.maximum(angular_uncertainty,abs(moments[1]-moments[0])/self.mass['inertia'])
        force_uncertainty*=1.+abs(math.tan(geometry['turn_rate']*self.dt))
    return dict(speed=float(speed),residual=residual,rate_residual=rate_residual,force_error_g=max(abs(residual[0]),abs(residual[1])),
                force_mean_uncertainty_g=force_uncertainty,angular_mean_uncertainty_rad_s2=angular_uncertainty,
                equilibrium_coordinates=list(x),equilibrium_load_g=float(load),
                history_error=history_error,history_input=cycle_input,result=r,geometry=geometry,
                # Instructor permission reads only air and local angles.
                # Do not retain thousands of complete phase diagnostics in
                # every numerical-Jacobian sample for long native cycles.
                phase_results=[dict(air=p['air'],history=p['history']) for p in phase_results] if self.config['instructor'] else [],
                maximum_wing_load_ratios=wing_ratios,
                allocation=allocation,velocity=velocity,stall_margin=stall_margin,
                negative_stall_margin=negative_stall_margin,flaps=flaps,gear=self.gear,instructor=None,propulsion=propulsion,
                ps=ps_native,ps_continuous=float(force.dot(geometry['forward'])*speed/self.weight),
                kinematic=kinematic,altitude_correction=kinematic['velocity'][1]!=vy,history_iterations=iteration+1)
