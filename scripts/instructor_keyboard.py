"""History-independent full-pitch Instructor inputs and rudder trim."""
import math
from component_assembly import f32,sub,mul
from control_mixer import curve,density_at_height,axis_limits
from polar_runtime import evaluate as mach_polar
from primary_controls import selected_properties,authority_ranges
from instructor_predictor_inputs import pack_autotrim_inputs
from instructor_protection import recovery_reference,angle_rate



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
