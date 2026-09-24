"""Recovered non-AdvancedMass mass/CG/inertia consumer for the selected jets.

Coordinates use the executable's forward/up/right frame. Payload and detachable
part positions are prepared runtime records; their mesh/weapon-data providers
are deliberately not inferred from the flight-model JSON. Arithmetic ordering
matches 0x101998bf0 and the final mass overwrite in 0x101998ad0.
"""
import re
from component_assembly import f32, add, sub, mul


def _reduce4(v):
    return add(add(v[3], v[1]), add(v[2], v[0]))


def fuel_sum(values):
    """Fuel-system sum, including the executable's eight-wide reduction."""
    values = list(map(f32, values))
    n = len(values)
    stop = n - ((n & 7) or 8) if n >= 9 else 0
    lo, hi = [0.] * 4, [0.] * 4
    for start in range(0, stop, 8):
        lo = [add(a, b) for a, b in zip(lo, values[start:start + 4])]
        hi = [add(a, b) for a, b in zip(hi, values[start + 4:start + 8])]
    result = _reduce4([add(a, b) for a, b in zip(hi, lo)]) if stop else 0.
    for value in values[stop:]:
        result = add(result, value)
    return result


def payload_sums(records):
    """Mass, first moment and point-mass inertia; payload intrinsic I is unused."""
    rows = []
    for record in records:
        mass = f32(record['mass'])
        x, y, z = map(f32, record['position'])
        xx, yy, zz = mul(x, x), mul(y, y), mul(z, z)
        rows.append([mass, mul(x, mass), mul(y, mass), mul(z, mass),
                     mul(add(zz, yy), mass), mul(add(zz, xx), mass),
                     mul(add(xx, yy), mass)])
    stop = len(rows) & ~3
    accum = [[0.] * 4 for _ in range(7)]
    for start in range(0, stop, 4):
        for lane, row in enumerate(rows[start:start + 4]):
            for col in range(7):
                accum[col][lane] = add(accum[col][lane], row[col])
    result = [_reduce4(v) for v in accum]
    for row in rows[stop:]:
        result = [add(a, b) for a, b in zip(result, row)]
    return result[0], result[1:4], result[4:]


def evaluate(properties, fuel_by_system, payloads=(), damage_parts=(),
             fuel_by_tank=(), payload_cg_scale=None, payload_inertia_scale=None,
             inertia_modifiers=(1., 1., 1.), presence_mask=None, nitro=0.):
    """Evaluate the complete selected nonadvanced consumer and mass overwrite.

    properties: empty, oil, crew_mass, configured_cog, normalized_inertia,
      optional payload_affects_cog and cog_y_limits (prepared runtime clamps).
    records: mass, position, and for damage_parts specific_inertia and optional
      fuel_tank index. Bits in presence_mask indicate attached/present parts.
    normalized_inertia and inertia_modifiers are doubles; other inputs float32.
    """
    if properties.get('advanced_mass', False):
        raise ValueError('AdvancedMass is outside the two selected jet paths')
    if payload_cg_scale is None:
        payload_cg_scale = properties.get('payload_cg_scale', 1.)
    if payload_inertia_scale is None:
        payload_inertia_scale = properties.get('payload_inertia_scale', 1.)
    empty, oil, crew, nitro = map(f32, [properties['empty'], properties['oil'],
                                       properties['crew_mass'], nitro])
    configured_cog = list(map(f32, properties['configured_cog']))
    fuel = fuel_sum(fuel_by_system)
    tanks = list(map(f32, fuel_by_tank))
    payload_mass, payload_first, payload_inertia = payload_sums(payloads)
    core = add(add(add(add(oil, empty), crew), nitro), fuel)
    consumer_mass = add(core, payload_mass)
    base_cg_mass = add(add(oil, empty), add(add(fuel, crew), nitro))
    if properties.get('payload_affects_cog', False):
        weight = [add(mul(base_cg_mass, x), y)
                  for x, y in zip(configured_cog, payload_first)]
        denominator = consumer_mass
    else:
        scale = f32(payload_cg_scale)
        weight = [add(mul(base_cg_mass, x), mul(y, scale))
                  for x, y in zip(configured_cog, payload_first)]
        denominator = add(mul(payload_mass, add(scale, -1.)), consumer_mass)
    reciprocal = f32(1. / denominator) if abs(denominator) > f32(4e-19) else 0.
    cog = [mul(x, reciprocal) for x in weight]
    norm = properties['normalized_inertia']
    scale2 = mul(payload_inertia_scale, payload_inertia_scale)
    inertia = [add(f32((norm[i] * inertia_modifiers[i]) * core),
                   mul(scale2, payload_inertia[i])) for i in range(2)]
    inertia.append(add(f32((core * norm[2]) * inertia_modifiers[2]),
                       mul(payload_inertia[2], scale2)))
    if presence_mask is None:
        presence_mask = (1 << len(damage_parts)) - 1
    # AdvancedMass=false leaves every fuel-tank 'already accounted for' byte 0.
    if damage_parts and not (presence_mask >= ((1 << (len(damage_parts) & 63)) - 1)
                             and not tanks):
        removed_mass = 0.
        removed_first, fuel_first = [0.] * 3, [0.] * 3
        for index, part in enumerate(damage_parts):
            tank_index = part.get('fuel_tank')
            tank_mass = 0. if tank_index is None else tanks[tank_index]
            position = list(map(f32, part['position']))
            if not (presence_mask >> (index & 63)) & 1:
                mass = add(tank_mass, part['mass'])
                proposed = add(mass, removed_mass)
                if proposed > empty:
                    break
                removed_first = [add(a, mul(mass, b))
                                 for a, b in zip(removed_first, position)]
                removed_mass = proposed
            elif tank_index is not None:
                fuel_first = [add(a, mul(tank_mass, b))
                              for a, b in zip(fuel_first, position)]
        updated_mass = sub(consumer_mass, removed_mass)
        reciprocal = f32(1. / updated_mass)
        updated_cog = [mul(reciprocal, add(mul(consumer_mass, old), sub(f, r)))
                       for old, f, r in zip(cog, fuel_first, removed_first)]
        delta2 = [mul(sub(a, b), sub(a, b)) for a, b in zip(updated_cog, cog)]
        inertia = [f32(inertia[i] + mul(updated_mass, add(delta2[a], delta2[b])))
                   for i, (a, b) in enumerate([(2, 1), (2, 0), (1, 0)])]
        removed = 0.
        for index, part in enumerate(damage_parts):
            tank_index = part.get('fuel_tank')
            tank_mass = 0. if tank_index is None else tanks[tank_index]
            present = (presence_mask >> (index & 63)) & 1
            if present and tank_index is None:
                continue
            mass = add(part['mass'], tank_mass)
            if not present:
                removed = add(removed, mass)
                if removed > empty:
                    break
            point = list(map(f32, part['position']))
            reference = updated_cog if present else cog
            xx, yy, zz = [mul(sub(a, b), sub(a, b)) for a, b in zip(point, reference)]
            ix, iy, iz = map(f32, part.get('specific_inertia', (0., 0., 0.)))
            if present:
                contribution = [mul(mass, add(add(zz, yy), ix)),
                                mul(mass, add(add(iy, xx), zz)),
                                mul(add(add(yy, xx), iz), mass)]
                inertia = [a + b for a, b in zip(inertia, contribution)]
            else:
                contribution = [mul(mass, add(zz, add(ix, yy))),
                                mul(mass, add(iy, add(xx, zz))),
                                mul(add(add(yy, xx), iz), mass)]
                if any(a < b for a, b in zip(inertia, contribution)):
                    break
                inertia = [a - b for a, b in zip(inertia, contribution)]
        cog, consumer_mass = updated_cog, updated_mass
    lower, upper = properties.get('cog_y_limits', (-2147440000., 2147440000.))
    cog[1] = min(max(cog[1], f32(lower)), f32(upper))
    # Both the initialization wrapper and timestep fuel wrapper overwrite mass.
    total_mass = add(add(add(add(add(fuel, empty), oil), crew), nitro), payload_mass)
    return dict(mass=total_mass, consumer_mass=consumer_mass, cog=cog,
                inertia=inertia, payload_mass=payload_mass,
                payload_inertia=payload_inertia, fuel_mass=fuel)


def aircraft_properties(fm, payload_scale=.2):
    """Selected full-real RB/SB defaults, from gameparams medium/hard settings.

    Payload scale affects CG and (squared) inertia; it never scales payload mass.
    The supplied jet files have no explicit Mass.Part records.
    """
    mass = fm['Mass']
    empty = min(max(f32(mass['EmptyMass']), f32(.1)), f32(1e8))
    normalized = [float(f32(x)) * (1. / empty) for x in fm['MomentOfInertia']]
    return dict(empty=empty, oil=f32(mass.get('OilMass', 0.)),
                crew_mass=mul(fm.get('Crew', 0), 90.),
                configured_cog=list(map(f32, mass['CenterOfGravity'])),
                normalized_inertia=normalized,
                advanced_mass=bool(mass.get('AdvancedMass', False)),
                payload_affects_cog=bool(mass.get('doesPayloadAffectCOG', False)),
                cog_y_limits=(-2147440000., 2147440000.),
                payload_cg_scale=f32(payload_scale),
                payload_inertia_scale=f32(payload_scale),
                explicit_parts=mass.get('Part', []))


def tank_configuration(fm):
    """Declared tank capacities and priorities, excluding no tanks implicitly."""
    parts = fm['Mass'].get('Parts', {})
    indices = sorted(int(re.fullmatch(r'tank(\d+)_capacity', key)[1])
                     for key in parts if re.fullmatch(r'tank(\d+)_capacity', key))
    return [dict(index=i - 1, capacity=f32(parts[f'tank{i}_capacity']),
                 system=parts.get(f'tank{i}_system', 0),
                 external=parts.get(f'tank{i}_external', False),
                 priority=parts.get(f'tank{i}_priority', 0)) for i in indices]
