"""Independent reconstruction of the pitch predictor's input preparation.

Original wrapper: 101a60b50..101a60f65. This is NOT the reduced predictor
101a5cac0 itself. Keeping the boundary explicit makes omitted aircraft state
visible before a predictor port or closed-loop comparison is attempted.
"""
import math
import struct
from component_assembly import f32, add, mul
from control_mixer import density_at_height
from body_dynamics import G


# Defined bytes only; the original wrapper leaves alignment padding unwritten.
DEFINED_SPANS = [(0, 12), (12, 1), (16, 24), (40, 1), (44, 60),
                 (104, 1), (108, 32), (144, 16)]


def pack_pitch_inputs(*, target_angle, target_acceleration, body_pitch_rate,
                      angle_bounds, axis_weights, mass, quaternion,
                      world_velocity, tas, speed_squared, mach, height,
                      engine_wash, engine_force, engine_moment, engine_scale,
                      torque_gyro, flap_blend, flap_health, flap_incidence,
                      sweep, gear_fraction, gear_available, airbrake_fraction,
                      airbrake_health, oil_radiator, water_radiator, cockpit_door,
                      parameter_pointer, gameplay_pointer):
    """Pack the 0xa0-byte input with the original float/double ordering.

    Vector conventions are the caller's native conventions, not converted EM
    axes. Engine force is double precision before applying the float32 scale;
    moment is copied without that multiplier. Density uses rounded altitude.
    The flap lookup result and radiator-provider results are explicit inputs.
    """
    out = bytearray(0xa0)
    def fs(off, values):
        struct.pack_into('<' + 'f'*len(values), out, off, *map(f32, values))
    struct.pack_into('<I', out, 0, 1)
    fs(4, [target_angle, target_acceleration])
    out[12] = 0
    fs(16, [body_pitch_rate, *angle_bounds, *axis_weights])
    out[40] = 0
    fs(44, [mul(G, f32(mass))])
    _, y, _, w = map(f32, quaternion)
    up = add(mul(2., add(mul(y, y), mul(w, w))), -1.)
    vx, vy, vz = world_velocity
    speed = math.sqrt(vz*vz + (vx*vx + vy*vy))
    vertical = f32(vy/speed) if speed > float(f32(4e-19)) else 0.
    fs(48, [min(1., max(-1., up)), min(1., max(-1., vertical)),
            tas, speed_squared, mach, density_at_height(f32(height)), *engine_wash])
    fs(80, [x*float(f32(engine_scale)) for x in engine_force])
    fs(92, engine_moment)
    out[104] = bool(torque_gyro)
    flap = mul(mul(f32(flap_blend), .5), add(*map(f32, flap_health)))
    def quantized(value):
        return min(255, max(0, int(mul(f32(value), 255.))))
    gear = mul(mul(float(quantized(gear_fraction)), f32(.0019607844)),
               float(sum(map(bool, gear_available))))
    airbrake = mul(mul(add(*map(f32, airbrake_health)), f32(.0019607844)),
                   float(quantized(airbrake_fraction)))
    fs(108, [flap, flap_incidence, sweep, gear, airbrake,
             oil_radiator, water_radiator, cockpit_door])
    struct.pack_into('<QQ', out, 144, parameter_pointer, gameplay_pointer)
    return bytes(out)


def defined_bytes(buffer):
    return b''.join(buffer[start:start+size] for start,size in DEFINED_SPANS)


def pack_autotrim_inputs(*, target_load=1., **state):
    """101a60830: level-flight equilibrium input, independent of current attitude.

    The native wrapper shares propulsion/deployment preparation with 101a60b50,
    but requests mode 0, zero pitch rate/reference point, and a horizontal
    one-g condition. The load multiplier is explicit; Instructor supplies 1.
    Current attitude, velocity direction and body rate are deliberately absent.
    """
    out = bytearray(pack_pitch_inputs(
        target_angle=target_load, target_acceleration=0., body_pitch_rate=0.,
        angle_bounds=[0., 0.], axis_weights=[0., 0., 0.],
        quaternion=[0., 0., 0., 1.], world_velocity=[1., 0., 0.], **state))
    struct.pack_into('<I', out, 0, 0)
    return bytes(out)
