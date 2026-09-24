"""Selected Yak-3/Bf 109 F-4 piston kernels, with explicit evidence boundaries.

This module does not supply propeller thrust or a complete aircraft evaluator.
The property adapter follows 1019fea70; consumers are independently executable-
checked by verify_piston_model.py. Fuel and health remain fixed.
"""
import math
from component_assembly import f32, add, sub, mul
from control_mixer import prepare_rows, curve, density_at_height

RAD_PER_RPM = f32(.10471975803375244)
RPM_PER_RAD = f32(9.549296379089355)
EPS = f32(4e-19)


def div(a, b):
    return f32(a / b) if abs(b) > EPS else 0.


def pressure_at_height(height, pressure0=101300., ceiling=18300.):
    """1019880b0, including the high-altitude extension, in Pa."""
    height, pressure0, ceiling = map(f32, (height, pressure0, ceiling))
    h = min(height, ceiling)
    p = f32(1.6037300167995674e-18)
    for c in [-1.3738000266463185e-13, 5.676299874579627e-9,
              -.00011844099935842678, 1.]:
        p = add(mul(p, h), f32(c))
    return div(mul(mul(ceiling, pressure0), p), max(height, ceiling))


def inlet_pressure(height, body_u, recovery):
    """1019fa22c..265: longitudinal ram pressure, normalized by sea-level Pa."""
    rho = density_at_height(f32(height))
    ram = mul(mul(mul(f32(body_u), f32(body_u)), mul(rho, .5)), f32(recovery))
    return div(add(ram, pressure_at_height(height)), f32(101300.))


def source_blocks(fm):
    if 'PropellerType0' in fm:
        return fm['EngineType0'], fm['PropellerType0']['Governor']
    return fm['Engine0'], fm['Engine0']['Propellor']


def prepare(fm):
    """Only these two inline, ExactAltitudes, compressor-type-1/2 records.

    Legacy ShaftRPMMin/Max fall back to Propellor.GovernorMin/MaxParam.
    The resulting runtime power normalization is NOT raw Main.Power.
    verify_piston_loaded.py independently executes the original constructor,
    property loader and finalizer before comparing the running wrapper.
    """
    e, governor = source_blocks(fm)
    main, c = e['Main'], e['Compressor']
    if main['Type'] != 'Inline' or not c['ExactAltitudes'] or c['Type'] not in (1, 2):
        raise ValueError('Outside the selected piston adapter')
    wmax = mul(f32(main['RPMMax']), RAD_PER_RPM)
    invw = div(RPM_PER_RAD, f32(main['RPMMax']))
    wshaft = [mul(f32(main.get('ShaftRPM'+k, governor['Governor'+k+'Param'])), RAD_PER_RPM)
              for k in ('Min', 'Max')]
    ratio = mul(invw, wshaft[1])
    power_scale = mul(sub(mul(3., ratio), add(mul(ratio, ratio), mul(ratio, ratio))), ratio)
    base_hp = div(f32(main['Power']), power_scale)
    knots = []
    for i in range(16):
        if 'RPM'+str(i) in c:
            x = div(sub(mul(f32(c['RPM'+str(i)]), RAD_PER_RPM), wshaft[0]), sub(wshaft[1], wshaft[0]))
            knots.append((x, [c['ATA'+str(i)]]))
    ata = prepare_rows(knots)
    max_ata = curve(ata, 1., 1)[0]
    sq = f32(c['CompressorOmegaFactorSq']); zero = f32(c['CompressorPressureAtRPM0'])
    compressor_reference = add(mul(add(mul(sub(mul(ratio, ratio), ratio), sq), ratio), sub(1., zero)), zero)
    stage = []
    inv_pressure = div(1., f32(101300.))
    inv_hp = div(1., mul(power_scale, base_hp))
    for i in range(c['NumSteps']):
        critical, flat, ceiling = [mul(pressure_at_height(c[k+str(i)]), inv_pressure)
                                   for k in ['Altitude', 'AltitudeConstRPM', 'Ceiling']]
        power = mul(f32(c['Power'+str(i)]), inv_hp)
        ceiling_factor = 1.
        if add(ceiling, f32(.0001)) < critical:
            ceiling_factor = div(sub(div(f32(c['PowerAtCeiling'+str(i)]), f32(c['Power'+str(i)])), 1.),
                                 sub(div(ceiling, critical), 1.))
        stage.append(dict(critical=critical, power=power, flat=flat,
                          flat_power=mul(f32(c['PowerConstRPM'+str(i)]), inv_hp),
                          curvature=f32(c['PowerConstRPMCurvature'+str(i)]),
                          ceiling=ceiling, ceiling_factor=ceiling_factor,
                          boost=f32(c.get('AfterburnerBoostMul'+str(i), 1.)),
                          pressure_boost=f32(c.get('AfterburnerPressureBoost'+str(i), 1.))))
    return dict(max_omega=wmax, inverse_omega=invw,
                afterburner_omega=mul(f32(main['RPMAfterburner']), RAD_PER_RPM),
                torque_base=div(mul(base_hp, 746.), wmax), base_hp=base_hp,
                min_throttle=f32(main['MinThrMult']), max_throttle=f32(main['MaxThrMult']),
                throttle_boost=f32(main['ThrottleBoost']), afterburner_boost=f32(main['AfterburnerBoost']),
                boost_type=e['Afterburner']['Type'], compressor_type=c['Type'], stages=stage,
                compressor_zero=zero, compressor_sq=sq, compressor_reference=compressor_reference,
                ata=ata, max_ata=max_ata, boost_ata_ratio=div(f32(c['AfterburnerManifoldPressure']), max_ata),
                ram_recovery=f32(c['SpeedManifoldMultiplier']),
                mixer_type=e['Mixer']['Type'], mixer_scale=f32(e['Mixer']['AltitudePressureToP0']))


def boost_active(p, throttle, afterburner, gear, nitro=0.):
    if not afterburner:
        return False
    kind = p['boost_type']
    if kind in (2, 4, 5, 9) or kind == 1:
        if kind == 1 and throttle <= 1.:
            return False
        if nitro < f32(.001):
            return False
    return not (0 <= gear < len(p['stages'])) or mul(p['stages'][gear]['boost'], p['afterburner_boost']) > f32(.99)


def rpm_torque(p, omega, throttle, torque_multiplier=1., afterburner=False, gear=0, nitro=0.):
    """Entire healthy running 1019f6c60 type-1/2 carburetor result.

    The caller supplies its torque multiplier (wrapper XMM7), separately from
    the compressor output. Overspeed damage is frozen;
    RPM itself is neither forced nor clipped by this torque evaluation.
    """
    omega, throttle, torque_multiplier = map(f32, (omega, throttle, torque_multiplier))
    effective = add(mul(sub(1., p['min_throttle']), min(p['max_throttle'], throttle)), p['min_throttle'])
    boost = boost_active(p, throttle, afterburner, gear, nitro)
    wref = p['afterburner_omega'] if boost else p['max_omega']
    x = div(omega, mul(effective, wref))
    shape = add(add(.5, effective), mul(mul(effective, effective), -.5))
    scale = mul(mul(p['torque_base'], shape), torque_multiplier)
    torque = mul(sub(mul(3., x), mul(mul(2., x), x)), scale)
    if throttle > 1.:
        tboost = p['throttle_boost']
        if throttle < f32(1.1):
            tboost = add(mul(add(mul(tboost, f32(9.999998092651367)), f32(-9.999998092651367)), sub(throttle, 1.)), 1.)
        torque = mul(torque, tboost)
    if boost:
        torque = mul(torque, mul(p['afterburner_boost'], p['stages'][gear]['boost']))
    if torque < 0.:
        torque = max(torque, mul(mul(mul(mul(torque_multiplier, f32(-.8)), omega), p['inverse_omega']), p['torque_base']))
    return torque


def mixture(p, inlet, command, *, rich_accumulator=0., automatic=False):
    """1019f71f0 manual RB/SB numerical branch, no engine-stop transition.

    The command is the delivered engine value; UI percentage conversion is
    separate. Below the .03 shutdown threshold, report requires_stop rather
    than silently preserving a running engine. Mixture type zero bypasses.
    """
    pressure, command, accum = map(f32, (inlet, command, rich_accumulator))
    kind = p['mixer_type']
    if kind == 0 or kind not in (1, 2):
        return dict(multiplier=1., rich_accumulator=accum, requires_stop=False)
    if kind == 1:
        command = max(1., command)
        if command == 1.:
            return dict(multiplier=1., rich_accumulator=accum, requires_stop=False)
    supplied = mul(command, p['mixer_scale'])
    # 1019f71f0, RB/SB running branch with automatic-mixture byte set.
    if automatic:supplied=min(supplied,add(pressure,f32(-.01)))
    stop = supplied < pressure and supplied < f32(.03)
    if supplied < pressure:
        limit = mul(pressure, .25)
        factor = 1. if supplied > limit else div(supplied, limit)
    elif supplied == pressure:
        factor = 1.
    else:
        inv = div(1., supplied)
        factor = mul(pressure, inv)
        accum = add(add(mul(mul(pressure, -35.), inv), 35.), accum)
    return dict(multiplier=factor, rich_accumulator=accum, requires_stop=stop)


def compressor(p, omega, throttle, inlet, dt, *, gear, old_gear=None,
               regulator=-1., afterburner=False):
    """Selected ExactAltitudes 1019f4fd0, running manual-state branch.

    gear is an explicit permitted stage; None requests the original best-stage
    search. No caller is required to trust automatic stage selection. Pressure
    regulator history is retained; a negative prior value invokes native reset.
    Restricted to omega >=150 rad/s; startup corrections remain outside scope.
    """
    omega, throttle, inlet, dt, regulator = map(f32, (omega, throttle, inlet, dt, regulator))
    if omega < 150.:
        raise ValueError('Low-RPM startup manifold corrections not ported')
    requested = curve(p['ata'], throttle, 1)[0]
    speed = mul(p['inverse_omega'], omega)
    speed = add(mul(add(mul(sub(mul(speed, speed), speed), p['compressor_sq']), speed),
                    sub(1., p['compressor_zero'])), p['compressor_zero'])
    candidates = []
    for i in range(len(p['stages'])) if gear is None else [gear]:
        s = p['stages'][i]
        boost = boost_active(p, throttle, afterburner, i)
        pb = p['boost_ata_ratio'] if boost else 1.
        flow = mul(inlet, s['pressure_boost']) if boost else inlet
        critical, flat = s['critical'], s['flat']
        line = s['ceiling_factor'] if add(s['ceiling'], f32(.0001)) < critical else 1.
        offset = sub(1., line)
        if flow <= critical:
            shape = s['power']
        elif critical >= add(flat, f32(-.0001)):
            baseline = 1.
            for _ in range(i): baseline = mul(mul(baseline, max(s['power'], 1.)), f32(.8))
            shape = add(div(mul(sub(s['power'], baseline), sub(1., flow)), max(sub(1., critical), f32(.0001))), baseline)
        elif flow <= flat:
            x = add(1., div(mul(sub(flow, critical), -1.), sub(flat, critical)))
            x = min(1., max(0., x))
            shape = add(mul(f32(math.pow(x, s['curvature'])), sub(s['power'], s['flat_power'])), s['flat_power'])
        else:
            baseline = 1.
            for _ in range(i): baseline = mul(mul(baseline, max(s['power'], 1.)), f32(.8))
            shape = add(div(mul(sub(s['flat_power'], baseline), sub(1., flow)), max(sub(1., flat), f32(.0001))), baseline)
        potential = mul(mul(flow, speed), div(p['max_ata'], mul(p['compressor_reference'], critical)))
        capped = min(potential, mul(p['max_ata'], pb))
        value = mul(add(div(mul(capped, line), mul(pb, requested)), offset), shape)
        if boost: value = mul(value, mul(s['boost'], p['afterburner_boost']))
        candidates.append(dict(gear=i, shape=shape, flow=flow, line=line, offset=offset,
                               boost_pressure=pb, value=value))
    selected = max(candidates, key=lambda x:x['value'])
    i = selected['gear']; s = p['stages'][i]
    potential = mul(mul(speed, selected['flow']), div(p['max_ata'], mul(p['compressor_reference'], s['critical'])))
    target = mul(div(p['max_ata'], potential), selected['boost_pressure'])
    if i == (gear if old_gear is None else old_gear) and regulator >= 0.:
        gain = f32(.1) if p['compressor_type'] == 2 else mul(dt, 3.)
        target = add(mul(sub(target, regulator), gain), regulator)
    regulator = min(1., max(0., target))
    throttle_ratio = min(div(requested, p['max_ata']), 1.)
    manifold = mul(mul(potential, throttle_ratio), regulator)
    factor = mul(add(div(mul(manifold, selected['line']), mul(requested, selected['boost_pressure'])), selected['offset']), selected['shape'])
    return dict(multiplier=factor, gear=i, regulator=regulator, throttle_ratio=throttle_ratio,
                potential_manifold=potential, manifold=manifold)
