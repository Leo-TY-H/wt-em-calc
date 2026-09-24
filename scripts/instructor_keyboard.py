"""Independent full-pitch keyboard Instructor command path, research only.

Intact free-air RB settings; constant-gain protection; orbiting disabled;
full-pitch input disables all MouseAim output axes; selected torque/gyro mode; manual
flap/gear; no near-ground assistance. The one-g auto-trim model is symmetric.
The controller's approximation is preserved independently of the aircraft FM.
MouseAim's disabled-output internal trajectory state is not reproduced here.
"""
import math
from component_assembly import f32,add,sub,mul
from control_mixer import curve,density_at_height,axis_limits
from polar_runtime import evaluate as mach_polar
from primary_controls import selected_properties,authority_ranges
from instructor_predictor_inputs import pack_pitch_inputs,pack_autotrim_inputs
from instructor_pitch_predictor import unpack_inputs,pitch_predictor
from instructor_autotrim import autotrim_predictor
from instructor_protection import (divide,predicted_wing_angles,angle_targets,
    recovery_reference,angle_rate,pitch_demands,protected_pitch_command,recovery_filter)



def rb_rudder_trim(model,state):
    """101a92d05/101a9412c..9433e: intact fin, non-arcade neutral trim.

    Native uses the fin polar at Mach zero, current-Mach rudder sensitivity,
    and the altitude/IAS control-angle table. As in the wing-bias caller,
    native passes altitude directly to the outer table axis, not air density.
    """
    wrapper=state['wrapper'];rudder=model['controls']['Rudder']
    polar=mach_polar(model['polars']['VerStabPlane'][0][1],0.)
    sensitivity=mul(curve(rudder['sensitivity_curve'],wrapper['mach'],1)[0],rudder['sensitivity'])
    sensitivity=mul(sensitivity,state['predictor']['f'][0x18d8])
    target=0.
    if abs(sensitivity)>f32(4e-19):
        incidence=f32(model['fm']['Aerodynamics']['VerStabPlane'].get('Angle',0.))
        target=f32(sub(incidence,f32(polar['cl0']/polar['clLineCoeff']))/sensitivity)
    positive,center,negative=axis_limits(rudder,[False]*3,f32(wrapper['height']),state['indicated_airspeed'])[2]
    denominator=sub(positive if target>0 else negative,center)
    numerator=target if target>0 else -target
    trim=f32(numerator/denominator) if abs(denominator)>f32(4e-19) else 0.
    return min(1.,max(-1.,trim))

def fixed_source(model,state):
    """Prepare only history-independent inputs for one held-source replay."""
    wrapper=state['wrapper'];props=state['properties']
    rho=density_at_height(f32(wrapper['height']))
    ias=f32(state['longitudinal_speed']*float(f32(math.sqrt(f32(rho/state['rho0'])))))
    ranges=authority_ranges(selected_properties(model['fm']),ias,axis_enabled=state['axis_enabled'],
        authority_scale=state['authority_scale'],full_loss=state['full_control_loss'],
        asymmetric=state['asymmetric_authority'],elevator_state=state['elevator_state'])
    ail=model['controls']['Ailerons'];sensitivity=mul(curve(ail['sensitivity_curve'],wrapper['mach'],1)[0],ail['sensitivity'])
    p=mach_polar(state['wing_runtime'],wrapper['mach'],1.)
    ref=recovery_reference(p['aoaCritH'],state['wing_angles'],props['critMult'],mode_lane=True,mode_scalar=0.,force_advanced=state['force_advanced'])
    rate=angle_rate(state['world_velocity'],state['world_acceleration'],state['quaternion'],state['pitch_rate'])
    tail=mach_polar(model['polars']['HorStabPlane'][0][1],0.)
    tail_bounds=[mul(tail['aoaCritL'],f32(.97)),mul(tail['aoaCritH'],f32(.97))]
    return dict(rho=rho,ias=ias,ranges=ranges,sensitivity=sensitivity,polar=p,reference=ref,rate=rate,tail_bounds=tail_bounds,
        auto_inputs=pack_autotrim_inputs(**wrapper),rudder_trim=rb_rudder_trim(model,state) if not wrapper['torque_gyro'] and state['autotrim_enabled'] else 0.)


def keyboard_step(model,state,history,dt,*,predictor_cache=None):
    """Source state in; command/trims and relevant persistent histories out.

    Provider results (engine, flap and current wing runtime) are explicit source
    inputs. No output or intermediate of the native controller is consumed.
    `history` is returned independently and is not mutated in place.
    """
    dt=f32(dt);pstate=state['predictor'];wrapper=dict(state['wrapper']);props=state['properties']
    trim=list(state['trim_requested']);actual_trim=list(state['trim_actual']);cache=list(state['trim_cache'])
    auto_history=list(history['autotrim']);predictor_history=[list(x) for x in history['predictors']]
    autotrim=None
    if predictor_cache is None:constant=fixed_source(model,state)
    else:
        if 'fixed_source' not in predictor_cache:predictor_cache['fixed_source']=fixed_source(model,state)
        constant=predictor_cache['fixed_source']
    def predict(kind,packed,predictor,previous):
        # This optional cache belongs to ONE fixed-source controller replay.
        # Native input bytes and the full predictor history identify a repeat.
        # It never skips a controller tick or changes predictor arithmetic.
        if predictor_cache is None:return predictor(model,unpack_inputs(packed),pstate,previous)
        key=(kind,packed,tuple(previous))
        if key not in predictor_cache:
            if len(predictor_cache)>2048:predictor_cache.clear()
            predictor_cache[key]=predictor(model,unpack_inputs(packed),pstate,previous)
        return predictor_cache[key]
    if state['autotrim_enabled']:
        autotrim=predict('auto',constant['auto_inputs'],autotrim_predictor,auto_history)
        auto_history=list(autotrim['history'])
        if autotrim['success']:
            trim[0],trim[1]=autotrim['output'][2],autotrim['output'][1]
            cache[0],cache[1]=trim[:2]
        if not wrapper['torque_gyro']:
            trim[2]=cache[2]=constant['rudder_trim']
    else:
        if state['default_autotrim']:trim=[0.,0.,0.];actual_trim=[0.,0.,0.]
        cache=list(trim)
    samples=list(history['dt_samples']);slot=history['dt_index'];total=history['dt_sum']
    total=add(sub(dt,samples[slot]),total);samples[slot]=dt;slot=(slot+1)%len(samples)
    mean=f32(total/len(samples))
    response=f32(.05) if mean<=f32(.02) else f32(.75) if mean>=f32(.1) else add(mul(mean,8.75),f32(-.125))
    rho=constant['rho'];ias=constant['ias'];ranges=constant['ranges'];sensitivity=constant['sensitivity']
    delivered=state['delivered'];wing_angles=state['wing_angles']
    pred=predicted_wing_angles(wing_angles,history['angles'],delivered[0],delivered[1],sensitivity,
        model['controls']['Elevator']['wing_aoa'])
    a=mul(mul(delivered[0],sensitivity),14.);e=mul(mul(model['controls']['Elevator']['wing_aoa'],delivered[1]),-18.)
    offsets=[add(e,a),sub(a,e)]
    p=constant['polar']
    limits=angle_targets(p,props,sorted(pred['predicted']),rho=rho,speed_squared=wrapper['speed_squared'],
        area=state['wing_area'],dihedral=state['dihedral'],strength=state['strength'],mass=wrapper['mass'],
        tail_area_pair=state['tail_area_pair'],timer=history['overload_timer'],dt=dt,
        mode_lane=True,overload_enabled=state['overload_enabled'])
    ref=constant['reference'];rate=constant['rate']
    demands=pitch_demands(p,props,pred['predicted'],limits['angle_limits'],reference_speed=state['reference_speed'],
        tas=wrapper['tas'],ias=ias,rate=rate)
    tail_bounds=constant['tail_bounds']
    predictions=[]
    for i,demand in enumerate(demands):
        packed=pack_pitch_inputs(**wrapper,**demand,body_pitch_rate=state['pitch_rate'],
            angle_bounds=tail_bounds,axis_weights=[0.,0.,0.],quaternion=state['quaternion'],world_velocity=state['world_velocity'])
        result=predict('pitch',packed,pitch_predictor,predictor_history[i]);predictions.append(result);predictor_history[i]=list(result['history'])
    clamp=protected_pitch_command([v['output'][1] for v in predictions],trim[1],
        [divide(1.,v) for v in ranges[1]],history['authority_factor'],state['requested'][1],
        pred['predicted'],offsets,p['aoaCritH'],ref,dt,[f32(.0125),f32(.05)])
    commands=list(state['requested']);commands[1]=clamp['command']
    peak=max(add(offsets[1],pred['predicted'][0]),sub(pred['predicted'][1],offsets[0]))
    active=state['recovery_enabled'] and peak>ref and not state['recovery_suppressed']
    if active:
        recovery=recovery_filter(history['last_commands'] if state['command_cache_enabled'] else commands,history['recovery'],[[-1.,1.],clamp['command_bounds'],[-1.,1.]],
            peak=peak,reference=ref,critical_high=p['aoaCritH'],current_wing_peak=max(wing_angles),stored_yaw_rate=state['stored_yaw_rate'],dt=dt,
            current_commands=commands)
        commands=recovery['commands'];recovery_history=recovery['history']
    else:recovery_history=list(commands)
    new_history=dict(autotrim=auto_history,predictors=predictor_history,angles=pred['adjusted'],
        overload_timer=limits['overload_timer'],authority_factor=clamp['authority_factor'],recovery=recovery_history,
        dt_samples=samples,dt_index=slot,dt_sum=total,
        last_commands=list(commands) if state['command_cache_enabled'] else list(history['last_commands']))
    return dict(commands=commands,trim_requested=trim,trim_actual=actual_trim,trim_cache=cache,
        time_constants=[response]*3 if state['command_cache_enabled'] else list(state['time_constants']),
        linear_rates=[f32(.03)]*3 if state['command_cache_enabled'] else list(state['linear_rates']),history=new_history,
        diagnostics=dict(autotrim=autotrim,angle_limits=limits['angle_limits'],predicted=pred['predicted'],demands=demands,
            predictors=predictions,clamp=clamp,recovery_active=active,reference=ref,angle_rate=rate,
            peak=peak,critical_high=p['aoaCritH']))
