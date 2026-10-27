"""Build the projectile properties omitted by the original flight fixtures.

This is a build-time adapter for decoded authored JSON, not a flight-time patch.
The scalar aero and motor ports remain unchanged. The outer rocket loader then
scales the fin-pressure cap by dry/initial mass. Native 2.59.0.87 instructions at
14143dac6..14143dc2d independently established that operation and its rounding.
"""
from copy import deepcopy
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import sys

MODEL = Path(__file__).resolve().parent / 'missile_model'
if str(MODEL) not in sys.path:
    sys.path.insert(0, str(MODEL))
from kernels import aero_properties, motor_properties, f32, add, sub, mul, div, safe_div
from body_integration import half_angle_trig

SCHEMA = 'wt-projectile-properties-v1'
NATIVE_BUILD = '2.59.0.87'
NATIVE_SHA256 = 'b56178905baedba7717f97202976ff8df2afa4c7819d4a6a2815cd8010598402'


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


def axis_quaternion(angles):
    """Native aerodynamic-axis degrees, half-angle polynomial and operation order."""
    if not isinstance(angles, list) or len(angles) != 3:
        raise ValueError('aerodynamicAxisAngles must contain three finite angles')
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in angles):
        raise ValueError('aerodynamicAxisAngles must contain three finite angles')
    trig = [half_angle_trig(mul(v, k)) for v, k in zip(
        angles, (.01745329238474369, .01745329238474369, -.01745329238474369))]
    sx, sy, sz = [v['sine'] for v in trig]
    cx, cy, cz = [v['cosine'] for v in trig]
    a = [mul(mul(sx, cz), sy), mul(mul(sx, cz), cy),
         mul(mul(cx, cz), sy), mul(mul(cx, cz), cy)]
    b = [mul(mul(cx, sz), cy), mul(mul(cx, sz), sy),
         mul(mul(sx, sz), cy), mul(mul(sx, sz), sy)]
    return [add(a[0], b[0]), add(a[1], b[1]), sub(a[2], b[2]), sub(a[3], b[3])]


def flat_curve(block, name='CyMachMult'):
    """The native curve loader searches numeric parameters in this block only.

    A same-name child is not a numeric parameter and is not traversed. Numbered
    point2 parameters take precedence over the point4/scalar fallback. Absent
    curves mean multiplier 1 in the existing body and controller consumers.
    """
    rows = []

    def append(pair):
        x, y = map(f32, pair)
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError('Curve points must be finite')
        if rows:
            dx = sub(x, rows[-1][0])
            if not dx > f32(4e-19):
                return
            rows[-1][1] = div(1., dx)
        rows.append([x, 0., y])

    for index in range(10):
        value = block.get(name + str(index))
        if value is not None:
            point2 = (isinstance(value, list) and len(value) == 2
                      and all(type(v) in (int, float) for v in value))
            append(value if point2 else (0., 0.))
    if not rows:
        value = block.get(name)
        if (isinstance(value, list) and len(value) == 4
                and all(type(v) in (int, float) for v in value)):
            append(value[:2])
            append(value[2:])
        elif type(value) in (int, float):
            append((0., value))
    return rows


def projectile_properties(rocket, globals_):
    """Derive body, motor and substep properties from unmodified authored input.

    Always start with a newly loaded raw cap; never scale a previously generated
    value. Summation and multiplication order match the outer native loader.
    The current motor port deliberately rejects unsupported factor tables.
    """
    body = aero_properties(rocket, globals_)
    motors = motor_properties(rocket)
    mass_lost = 0.
    for propulsion in motors:
        for impulse in propulsion.impulses:
            mass_lost = add(mass_lost, impulse.mass_lost)
    body['fin_pressure_limit'] = mul(
        body['fin_pressure_limit'], safe_div(sub(body['mass'], mass_lost), body['mass']))
    body['axis_quaternion'] = axis_quaternion(rocket.get('aerodynamicAxisAngles', [0., 0., 0.]))
    body['cy_table'] = flat_curve(rocket)
    precision = rocket.get('precision', 1)
    if type(precision) is not int or not 1 <= precision < 2**31:
        raise ValueError('The flight runtime requires a positive authored int32 precision')
    # JSON round-trip gives the same list representation used in stored profiles.
    serialized_motors = json.loads(json.dumps([asdict(m) for m in motors], allow_nan=False))
    return body, serialized_motors, precision


def regenerate_profile(profile, globals_):
    """Replace derived physical fields while preserving authored data and state.

    The saved constructor histories and non-aero guidance properties remain
    their independently captured inputs. Unsupported changes in those inputs
    fail instead of being silently recertified by a partial property generator.
    """
    if profile.get('properties_sha256') != digest(profile['properties']):
        raise ValueError('Input profile properties digest is invalid: ' + profile.get('asset', '?'))
    result = deepcopy(profile)
    properties = result['properties']
    rocket = properties['rocket']
    body, motors, precision = projectile_properties(rocket, globals_)
    old_body = properties['body']
    unaffected = set(body) - {'fin_pressure_limit', 'cy_table'}
    changed = [key for key in sorted(unaffected) if old_body.get(key) != body[key]]
    if changed or set(old_body) != set(body):
        raise ValueError('Body fields outside this generator revision changed: ' + repr(changed))
    if properties['guidance']['motors'] != motors:
        raise ValueError('Saved guidance motor properties disagree with authored motor data')
    properties['body'] = {key: body[key] for key in old_body}
    properties['precision'] = precision
    aero = properties['guidance']['aero']
    for key in ('mass', 'cy', 'cy_limit', 'side_area', 'cy_table'):
        aero[key] = deepcopy(body[key])
    result['properties_sha256'] = digest(properties)
    result['property_loading'] = dict(schema=SCHEMA, validated_build=NATIVE_BUILD,
                                     rocket_sha256=digest(rocket), globals_sha256=digest(globals_))
    return result
