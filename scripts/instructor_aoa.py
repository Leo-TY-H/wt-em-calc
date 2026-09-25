"""EXPERIMENTAL chart approximation; not a validated native permission boundary.

Static effective AoA constraint shared by the EM boundary and interior.

This approximation equates native PD demand to acceleration predicted at the aircraft's
balanced delivered elevator. The reduced predictor and full aircraft use
slightly different force/moment representations, so that acceleration is not
zero even in a steady turn. Evaluate its forward balance algebraically; do not
solve a controller history or fit an aircraft-specific angle correction.

Native critical-angle/Mach/flap/sweep targets, rate feedback and available
automatic trim authority are retained with independently solved one-g auto trim.
No freely optimized turn trim is permitted. Transient overload reserve/release is omitted;
the aircraft solver independently retains stall, strength and actuator limits.
This static chart model does not simulate the live controller's transients.
"""
from instructor_chart_inputs import source_state
from windows_instructor_source import fixed_source
from instructor_protection import angle_targets, predicted_wing_angles, pitch_demands
from instructor_settings import trim_retained
from instructor_predictor_inputs import pack_pitch_inputs
from instructor_pitch_predictor import unpack_inputs
from instructor_aoa_balance import required_acceleration
from instructor_static_trim import static_trim, trim_independent_authority


def controller_limits(solver, value, speed=None):
    from em_solver import command_allocation
    if value.get('instructor') is not None:
        return value['instructor']
    if not hasattr(solver, '_effective_aoa_constants'):
        solver._effective_aoa_constants = {}
    state = source_state(solver, value, constant_cache=solver._effective_aoa_constants)
    fixed = fixed_source(solver.model, state)
    adjusted = predicted_wing_angles(state['wing_angles'], [0., 0.],
        state['delivered'][0], state['delivered'][1], fixed['sensitivity'],
        solver.model['controls']['Elevator']['wing_aoa'])['adjusted']
    targets = angle_targets(fixed['polar'], state['properties'], adjusted,
        rho=fixed['rho'], speed_squared=state['wrapper']['speed_squared'],
        area=state['wing_area'], dihedral=state['dihedral'], strength=state['strength'],
        mass=state['wrapper']['mass'], tail_area_pair=state['tail_area_pair'],
        timer=0., dt=0., mode_lane=True, overload_enabled=False)
    demands = pitch_demands(fixed['polar'], state['properties'], adjusted,
        targets['angle_limits'], reference_speed=state['reference_speed'],
        tas=state['wrapper']['tas'], ias=fixed['ias'], rate=fixed['rate'])
    kp = state['reference_speed'] / max(5., state['wrapper']['tas'])
    balances = []
    for demand in demands:
        packed = pack_pitch_inputs(**state['wrapper'], **demand, body_pitch_rate=state['pitch_rate'],
            angle_bounds=fixed['tail_bounds'], axis_weights=[0.,0.,0.],
            quaternion=state['quaternion'], world_velocity=state['world_velocity'])
        balances.append(required_acceleration(solver.model, unpack_inputs(packed),
                                              state['predictor'], state['delivered'][1]))
    margins_angle = [(demands[0]['target_acceleration']-balances[0]['acceleration']) / kp,
                    (balances[1]['acceleration']-demands[1]['target_acceleration']) / kp]
    ground = [solver.fm['AvailableControls'].get('has'+axis+'TrimGroundControl', False)
              for axis in ('Aileron', 'Elevator', 'Rudder')]
    controls = dict(solver.controls, trim_available=trim_retained(
        solver.controls['trim_available'], ground, 1, True))
    auto = static_trim(solver, state, fixed)
    independent = trim_independent_authority(controls, fixed['ranges'])
    # A failed one-g inverse solve cannot remove a point when every possible
    # bounded trim gives exactly the same control interval. Use neutral trim
    # only to evaluate that invariant interval, never as a reported trim root.
    allocation_trim = auto['trim'] if auto['success'] else [0., 0., 0.]
    allocation = command_allocation(controls, state['delivered'], fixed['ranges'],
        dict(solver.config, trim_mode='fixed', fixed_trim=allocation_trim))
    mechanical = min(min(x-lo, hi-x) for x, (lo, hi) in zip(state['delivered'], allocation['bounds']))
    angle_margin = min(margins_angle) / 10.
    margins = {'Instructor effective AoA': angle_margin,
               'control authority with auto trim': mechanical}
    effective = [min(adjusted)-margins_angle[0], max(adjusted)+margins_angle[1]]
    out = dict(converged=auto['success'] or independent, margin=min(margins.values()), margins=margins,
        envelope_margin=angle_margin, pitch_margin=angle_margin, angle_margin=angle_margin,
        limiting=min(margins, key=margins.get), control_bounds=allocation['bounds'],
        auto_trim=allocation['trim'] if auto['success'] else None,
        sticks=allocation['sticks'] if auto['success'] else None, static_trim=auto,
        control_authority_trim_independent=independent,
        delivered_pitch=state['delivered'][1],
        required_pitch=state['delivered'][1], angle_limits_deg=targets['angle_limits'],
        effective_angle_limits_deg=effective, adjusted_wing_angles_deg=adjusted,
        native_angle_rate_rad_s=fixed['rate'], model='effective AoA schedule with reduced moment balance',
        rate_demands=demands, reduced_balances=balances, history_independent=True, exact_native_controller=False,
        overload_policy='transient reserve omitted; physical strength retained')
    value['instructor'] = out
    return out
