"""Type-2 downwash scalar stage after geometric projection.

The caller's rotation into the wake frame remains outside this port. Inputs x,
transverse, amplitude, inverse_span2, inverse_halfspan, longitudinal_exponent
are explicitly prepared intermediates. Output is an angle addition in degrees.
"""
import math
from component_assembly import f32, add, mul


def scalar_stage(x, transverse, inverse_speed2, travel_cap, roll_rate,
                 current_cl, cl_history_rate, inverse_halfspan,
                 longitudinal_exponent, amplitude, coefficient, inverse_span2):
    """Port 0x106c615a1..0x106c616d6 for the positive downstream branch.

    Caller bypasses this entire stage for projected downstream x <= 0.
    exp/sincos are host math rounded to float32, as in the verification hooks.
    """
    a, b = transverse
    distance2 = add(add(mul(a,a), mul(b,b)), mul(x,x))
    travel = min(f32(math.sqrt(mul(distance2,inverse_speed2))), travel_cap)
    angle = f32(float(travel) * roll_rate)
    lateral = mul(add(mul(f32(math.cos(angle)),a), mul(f32(math.sin(angle)),b)), inverse_halfspan)
    delayed_cl = add(mul(travel,cl_history_rate),current_cl)
    longitudinal = add(f32(math.exp(mul(x,longitudinal_exponent))),f32(.9))
    strength = mul(mul(longitudinal,amplitude),delayed_cl)
    attenuation = f32(math.exp(mul(abs(lateral),f32(-1.4096779823303223))))
    return mul(mul(attenuation,inverse_span2),mul(strength,coefficient))
