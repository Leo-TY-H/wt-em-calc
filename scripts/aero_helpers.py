"""Native detailed-update helper moment, 106c62ece..6300b."""
import math
from component_assembly import f32,add,sub,mul


def roll_leveling(alpha,velocity_x,quaternion,mass):
    if abs(f32(alpha))>=12.:return 0.
    x,y,z,w=map(f32,quaternion)
    test=add(mul(w,z),mul(y,x))
    if test>=f32(.49999) or test<=f32(-.49999):roll=0.
    else:
        numerator=sub(mul(w,x),mul(z,y));denominator=add(mul(-z,z),sub(.5,mul(x,x)))
        roll=f32(math.atan2(numerator,denominator)) if numerator or denominator else 0.
    angle=min(30.,max(-30.,mul(roll,f32(-57.2957763671875))))
    gain=min(float(velocity_x)-50.,50.)*float(f32(.003))
    command=f32(float(angle)*gain)
    weight=mul(f32(9.8100004196167),f32(mass))
    return float(mul(mul(f32(-.01),weight),command))
