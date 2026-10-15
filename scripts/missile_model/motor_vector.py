"""Independent propulsion component for pinned game build 2.59.0.34.

Ports 143af2fb0 and 14142a3c0 for the catalog's factorIndex=-1 engines.
This is an isolated arithmetic reconstruction, not a trajectory simulator.
"""
import hashlib
import json
import math
from pathlib import Path
import struct

from kernels import add, f32, mul, sub, motor_properties, motor_scalar, pressure_multiplier


def load_table():
    data = json.loads(Path(__file__).with_name('motor-deviation-table.json').read_text())
    raw = b''.join(struct.pack('<2f', *row) for row in data['rows'])
    expected = '9bbbcddf2da7cbcc3f6b2b1b17f6c4060c3e92764a8619a3651f292d7136fe32'
    if len(raw) != 2048 or hashlib.sha256(raw).hexdigest() != expected:
        raise ValueError('Motor deviation table differs from pinned executable')
    return tuple(tuple(row) for row in data['rows'])


TABLE = load_table()


def deviation_sample(word):
    slope, intercept = TABLE[(word >> 23) & 255]
    return add(mul((word >> 16) & 127, slope), intercept)


def motor_vector(rocket, clocks, *, height=0., fins=(0., 0.),
                 orientation=(0., 0.), seed=1, _motor_properties=None):
    props = motor_properties(rocket) if _motor_properties is None else _motor_properties
    scalar = motor_scalar(props, clocks)
    mapping = [f32(v) for v in rocket.get('extPressureToThrustMult', [0., 1., 0., 1.])]
    # Native bypass retains the FIRST authored ordinate, before endpoint sorting.
    factor = mapping[1]
    if scalar['thrust'] > 0 and mapping[1] != mapping[3]:
        factor = pressure_multiplier(rocket, height)
    scale = mul(factor, scalar['thrust'])
    z = mul(-f32(fins[0]), scalar['vectoring'])
    y = mul(-f32(fins[1]), scalar['vectoring'])
    x = f32(math.sqrt(max(sub(1., add(mul(z, z), mul(y, y))), 0.)))
    first = (int(seed) * 0x41c64e6d + 0x3039) & 0xffffffff
    second = (first * 0x41c64e6d + 0x3039) & 0xffffffff
    deviation = f32(rocket.get('thrustDeviation', 0.))
    noisy_z = add(mul(deviation_sample(first), deviation), z)
    noisy_y = add(mul(deviation_sample(second), deviation), y)
    torque = f32(rocket.get('orientationTorque', 0.))
    return dict(thrust=[mul(x, scale), mul(noisy_y, scale), mul(noisy_z, scale)],
                noiseless_thrust=[mul(x, scale), mul(y, scale), mul(z, scale)],
                orientation_torque=[0., mul(-f32(orientation[0]), torque),
                                    mul(-f32(orientation[1]), torque)],
                mass_lost=scalar['mass_lost'], seed=second)


def rotate_thrust(quaternion, thrust):
    """Preserve native float32 expression order; do not normalize quaternion."""
    x, y, z, w = map(f32, quaternion)
    tx, ty, tz = thrust
    a = mul(add(z, z), x)
    b = mul(y, add(z, z))
    c = mul(add(z, z), w)
    d = mul(add(w, w), y)
    e = mul(w, add(x, x))
    f = mul(add(x, x), y)
    ww = sub(add(mul(w, w), mul(w, w)), 1.)
    xx = add(add(mul(x, x), mul(x, x)), ww)
    yy = add(add(mul(y, y), mul(y, y)), ww)
    zz = add(add(mul(z, z), mul(z, z)), ww)
    return [add(add(mul(add(d, a), tz), mul(sub(f, c), ty)), mul(xx, tx)),
            add(add(mul(sub(b, e), tz), mul(add(c, f), tx)), mul(yy, ty)),
            add(mul(zz, tz), add(mul(add(e, b), ty), mul(sub(a, d), tx)))]


def propulsion_adapter(rocket, *, length, clocks, factors, dt, quaternion,
                       height=0., fins=(0., 0.), orientation=(0., 0.), seed=1,
                       _motor_properties=None):
    """Compute force at prospective clocks, returning rates but not committing clocks.

    Speed is immaterial for supported engines. Factor-table engines are rejected
    by motor_properties. Caller supplies all four stored clocks and factors.
    """
    # Owned website flights may supply their validated immutable motor layout.
    # Other callers prepare from the current rocket; no identity/global cache.
    props = motor_properties(rocket) if _motor_properties is None else _motor_properties
    if len(clocks) != 4 or len(factors) != 4:
        raise ValueError('Adapter requires four stored clocks and four factors')
    rates = [f32(v) for v in factors[:len(props)]] + [0.] * (4-len(props))
    future = [add(t, mul(k, dt)) for t, k in zip(clocks, rates)]
    body = motor_vector(rocket, future[:len(props)], height=height,
                        fins=fins, orientation=orientation, seed=seed, _motor_properties=props)
    lever = mul(length, -.5)
    nt, ot = body['noiseless_thrust'], body['orientation_torque']
    return dict(world_force=rotate_thrust(quaternion, body['thrust']),
                body_torque=[ot[0], add(mul(lever, nt[2]), ot[1]),
                             sub(ot[2], mul(lever, nt[1]))],
                mass_lost=body['mass_lost'], clock_rates=rates, seed=body['seed'])
