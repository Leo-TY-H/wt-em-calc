from instructor_chart_inputs import source_state
from windows_instructor_source import fixed_source
from instructor_protection import angle_targets, predicted_wing_angles, pitch_demands
from instructor_settings import trim_retained
from instructor_predictor_inputs import pack_pitch_inputs
from instructor_pitch_predictor import unpack_inputs
import instructor_aoa_balance
from instructor_aoa_balance import required_acceleration
from instructor_static_trim import static_trim, trim_independent_authority

# AoA margin (deg) without the Instructor: finite, since chart records pass through JSON.
NO_INSTRUCTOR_MARGIN = 1e30


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
    # Autotrim without the Instructor (realistic controls): the fixed one-g
    # trim bounds the controls, and there is no AoA limit.
    instructor = solver.config.get('instructor', True)
    # Chart policy: short-release aircraft use the released pull unconditionally.
    # Only the held state applies the overload reserve. Ordinary AoA, load-factor,
    # tailplane and control-authority limits remain active in either state.
    protection = getattr(solver, 'instructor_protection', None)
    targets = demands = limits = None
    balances = []
    margins_angle = [NO_INSTRUCTOR_MARGIN, NO_INSTRUCTOR_MARGIN]
    # the upper acceleration margin alone (deg of wing AoA: the readout effective_angle_limits_deg), the tailplane cap
    # margin (deg of tail angle) and whether the cap is the smaller term on the upper side
    upper_margin, tail_cap, tail_limiting = NO_INSTRUCTOR_MARGIN, instructor_aoa_balance.NO_TAIL_CAP, False
    if instructor:
        common = dict(rho=fixed['rho'], speed_squared=state['wrapper']['speed_squared'],
            area=state['wing_area'], dihedral=state['dihedral'], strength=state['strength'],
            mass=state['wrapper']['mass'], tail_area_pair=state['tail_area_pair'],
            timer=0., dt=0., mode_lane=True)
        targets = angle_targets(fixed['polar'], state['properties'], adjusted,
                                overload_enabled=protection == 'held', **common)
        limits = targets['angle_limits']
        demands = pitch_demands(fixed['polar'], state['properties'], adjusted,
            limits, reference_speed=state['reference_speed'],
            tas=state['wrapper']['tas'], ias=fixed['ias'], rate=fixed['rate'])
        kp = state['reference_speed'] / max(5., state['wrapper']['tas'])
        for demand in demands:
            packed = pack_pitch_inputs(**state['wrapper'], **demand, body_pitch_rate=state['pitch_rate'],
                angle_bounds=fixed['tail_bounds'], axis_weights=[0.,0.,0.],
                quaternion=state['quaternion'], world_velocity=state['world_velocity'])
            balances.append(required_acceleration(solver.model, unpack_inputs(packed),
                                                  state['predictor'], state['delivered'][1]))
        margins_angle = [(demands[0]['target_acceleration']-balances[0]['acceleration']) / kp,
                        (balances[1]['acceleration']-demands[1]['target_acceleration']) / kp]
        upper_margin, tail_cap = margins_angle[1], balances[1]['tail_cap_margin_deg']
        # the tailplane cap of the stick solve (instructor_aoa_balance.TAIL_CLAMP): upper side only, deg of tail angle
        if instructor_aoa_balance.TAIL_CLAMP:
            tail_limiting = tail_cap < margins_angle[1]
            margins_angle[1] = min(margins_angle[1], tail_cap)
    ground = [solver.fm['AvailableControls'].get('has'+axis+'TrimGroundControl', False)
              for axis in ('Aileron', 'Elevator', 'Rudder')]
    controls = dict(solver.controls, trim_available=trim_retained(
        solver.controls['trim_available'], ground, 1, True))
    independent = trim_independent_authority(controls, fixed['ranges'])
    if independent and solver.config.get('instructor_authority_mode') == 'direct':


        auto = dict(success=False, skipped=True, trim=[0., 0., 0.],
                    method='trim-independent authority',
                    reason='Automatic trim is unnecessary for control bounds')
    else:
        auto = static_trim(solver, state, fixed)


    allocation_trim = auto['trim'] if auto['success'] else [0., 0., 0.]
    allocation = command_allocation(controls, state['delivered'], fixed['ranges'],
        dict(solver.config, trim_mode='fixed', fixed_trim=allocation_trim))
    mechanical = min(min(x-lo, hi-x) for x, (lo, hi) in zip(state['delivered'], allocation['bounds']))
    angle_margin = min(margins_angle) / 10.
    margins = {'Instructor effective AoA': angle_margin,
               'control authority with auto trim': mechanical}
    # the wing-AoA limits of the acceleration margins; the tailplane cap (a tail angle) is reported beside them
    effective = [min(adjusted)-margins_angle[0], max(adjusted)+upper_margin]
    out = dict(converged=auto['success'] or independent, margin=min(margins.values()), margins=margins,
        authority_mode=solver.config.get('instructor_authority_mode','native'),
        envelope_margin=angle_margin, pitch_margin=angle_margin, angle_margin=angle_margin,
        limiting=min(margins, key=margins.get), control_bounds=allocation['bounds'],
        auto_trim=allocation['trim'] if auto['success'] else None,
        sticks=allocation['sticks'] if auto['success'] else None, static_trim=auto,
        control_authority_trim_independent=independent,
        delivered_pitch=state['delivered'][1],
        required_pitch=state['delivered'][1], angle_limits_deg=limits if instructor else None,
        effective_angle_limits_deg=effective, adjusted_wing_angles_deg=adjusted,
        tail_cap_margin_deg=tail_cap, tail_cap_limiting=tail_limiting,
        native_angle_rate_rad_s=fixed['rate'],
        model='effective AoA schedule with reduced moment balance' if instructor else 'autotrim authority (Instructor off)',
        rate_demands=demands, reduced_balances=balances, history_independent=True, exact_native_controller=False,
        overload_policy='wing protection held (overload timer 0)' if protection == 'held'
                        else 'wing protection released or off; wing strength charted')
    value['instructor'] = out
    return out
