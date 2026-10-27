"""Read-only flight diagnostics, kept outside the hash-pinned model bundle.

Re-evaluate the final substep's pure force function with the exact inputs used
by the completed update. Never advance propulsion again or commit any state.
Controller outputs are read back from the step's own controller calls
(record_controls), never recomputed.
"""
import math

from acceleration_control import lateral_axes
from aero_vectors import forces
from kernels import atmosphere, div, f32, add, mul, motor_scalar


FORCE_FIELDS = ('force_time_s', 'thrust_n', 'mass_kg', 'propellant_used_kg',
                'drag_n', 'lift_n', 'drag_coefficient', 'aoa_deg',
                'dynamic_pressure_pa', 'acceleration_mps2')
CONTROL_FIELDS = ('fin_h', 'fin_v', 'accel_cmd_h_mps2', 'accel_cmd_v_mps2', 'accel_h_mps2', 'accel_v_mps2',
                  'pid_p_h', 'pid_i_h', 'pid_d_h', 'pid_p_v', 'pid_i_v', 'pid_d_v',
                  'tvc_h_deg', 'tvc_v_deg', 'cruise_pid')

_step = {}   # this simulator step's last acceleration- and propulsion-control calls


def initial(properties):
    # No force integration has occurred at release. Do not invent a zero thrust.
    return dict.fromkeys(FORCE_FIELDS) | dict(mass_kg=properties['body']['mass']) | dict.fromkeys(CONTROL_FIELDS)


def record_controls():
    """Let each simulator step read back its controller results.

    The guidance managers call the acceleration and propulsion controllers through
    optical_manager_controls; these wrappers keep the last call of each and return
    its result unchanged. Installed after the backend has bound its controllers.
    """
    import optical_manager_controls as services
    if getattr(services, 'telemetry_recorded', False): return
    acceleration, propulsion = services.acceleration_update, services.propulsion_update

    def acceleration_update(p, aero, motor, state, request, measured, q, velocity, height, time, dt, *,
                            environment=None, matrix_velocity_frame=True):
        result = acceleration(p, aero, motor, state, request, measured, q, velocity, height, time, dt,
                              environment=environment, matrix_velocity_frame=matrix_velocity_frame)
        _step['acceleration'] = (result, measured)
        return result

    def propulsion_update(p, state, time, position, velocity, tgo, closure, dt, *, environment=None):
        stages = list(state['stages'])
        result = propulsion(p, state, time, position, velocity, tgo, closure, dt, environment=environment)
        _step['propulsion'] = (stages, result['factors'], len(p))
        return result

    services.acceleration_update = acceleration_update
    services.propulsion_update = propulsion_update
    services.telemetry_recorded = True


def begin_step():
    _step.clear()


def controls(properties, applied, body):
    """Fin commands, lateral accelerations and PID terms of the step's last
    acceleration-control call (horizontal, vertical axes of its control frame),
    thrust-vector angles of the fins the step flew with while a vectoring motor
    burns, and the cruise-speed PID's motor factor; None where they do not apply."""
    out = dict.fromkeys(CONTROL_FIELDS)
    record = _step.get('acceleration')
    if record is not None and record[0]['fins'] is not None:
        result, measured = record
        h, v = lateral_axes(result['frame'])
        mx, my, mz = map(f32, measured)
        # acceleration_control.update's projection of the measured acceleration
        actual = [add(mul(h[1], my), add(mul(h[2], mz), mul(h[0], mx))),
                  add(mul(v[2], mz), add(mul(v[1], my), mul(v[0], mx)))]
        h_state, v_state = result['state']
        out.update(fin_h=result['fins'][0], fin_v=result['fins'][1],
                   accel_cmd_h_mps2=result['limited_request'][0], accel_cmd_v_mps2=result['limited_request'][1],
                   accel_h_mps2=actual[0], accel_v_mps2=actual[1],
                   pid_p_h=mul(h_state[0], h_state[6]), pid_i_h=h_state[3], pid_d_h=mul(h_state[7], h_state[4]),
                   pid_p_v=mul(v_state[0], v_state[6]), pid_i_v=v_state[3], pid_d_v=mul(v_state[7], v_state[4]))
    motors = properties['guidance']['motors']
    scalar = motor_scalar(motors, body['clocks'][:len(motors)])
    if scalar['thrust'] > 0 and scalar['vectoring'] != 0:
        out['tvc_h_deg'] = math.degrees(math.asin(max(-1., min(1., mul(applied[0], scalar['vectoring'])))))
        out['tvc_v_deg'] = math.degrees(math.asin(max(-1., min(1., mul(applied[1], scalar['vectoring'])))))
    record = _step.get('propulsion')
    if record is not None:
        stages, factors, count = record
        for index in range(count):
            if stages[index] == 2:
                out['cruise_pid'] = factors[index]
                break
    return out


def completed(properties, before, controls, update, dt):
    substeps = update['substeps']
    body = substeps[-2]['state'] if len(substeps) > 1 else before
    propulsion = update['propulsion']
    aero = forces(properties['body'], body['position'][1], body['velocity'],
                  body['quaternion'], body['omega'], fins=controls[:2],
                  dt=div(dt, properties['precision']),
                  force=propulsion['world_force'], torque=propulsion['body_torque'],
                  mass_lost=propulsion['mass_lost'], body_random=body['body_random'])
    scale = aero['perturbation']['force_scale']
    return dict(force_time_s=body['time'],
                thrust_n=math.hypot(*propulsion['world_force']),
                mass_kg=aero['effective_mass'], propellant_used_kg=propulsion['mass_lost'],
                drag_n=math.hypot(*aero['baseline']['drag']) * abs(scale),
                lift_n=math.hypot(*aero['baseline']['lift']) * abs(scale),
                drag_coefficient=aero['baseline']['cd'],
                aoa_deg=math.degrees(math.acos(max(-1., min(1., aero['baseline']['cosine'])))),
                dynamic_pressure_pa=aero['pressure'],
                acceleration_mps2=math.hypot(*substeps[-1]['acceleration']))


def mach(body):
    return math.hypot(*body['velocity']) / atmosphere(body['position'][1])['sound_speed']
