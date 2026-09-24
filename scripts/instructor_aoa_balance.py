"""Forward moment balance of the native mode-1 predictor at delivered pitch.

Invert its moment equation, not its iterative elevator/CL search: evaluate the
tail at the given command and return the acceleration that makes that reduced
aircraft balance. No controller history or empirical aircraft offset enters.
"""
import math
from component_assembly import f32, add, sub, mul
from polar_f32 import calc_c
from instructor_pitch_predictor import prepare_mode1, deflection, RAD
from instructor_protection import divide
from instructor_reduced import pitch_balance, tail_flow, legacy_wake_factor


def required_acceleration(model, ip, state, delivered_pitch):
    c = prepare_mode1(model, ip, state)
    f, flags, g = state['f'], state['flags'], c['geometry']
    convert, invert = flags[0x7c0a], flags[0x7c54]
    ail, elev = model['controls']['Ailerons'], model['controls']['Elevator']
    working = sub(ip[4], g['incidence'])
    sn, cs = f32(math.sin(mul(working, RAD))), f32(math.cos(mul(working, RAD)))
    vx, vy = mul(cs, c['speed']), mul(sn, -c['speed'])
    axial = swirl = 0.
    qvx = vx
    if ip[0x48] != 0.:
        wash = mul(ip[0x48], c['wash_attenuation'])
        k = max(add(f32(mul(abs(vy), -1.5) / max(add(wash, vx), f32(.2))), 1.), 0.)
        axial = mul(wash, k)
        if abs(axial) < f32(1.1920928955078125e-7):axial = 0.
        qvx = add(vx, ip[0x48])
        if ip[0x68] and not flags[0x8471]:
            spin = min(mul(mul(qvx, qvx), f32(.0011)), 1.) if qvx >= 0. else 0.
            swirl = mul(mul(mul(k, c['wash_attenuation']), spin), ip[0x4c])
            if abs(swirl) < f32(1.1920928955078125e-7):swirl = 0.
    dynamic = mul(mul(add(mul(vy, vy), mul(qvx, qvx)), .5), ip[0x44])
    taq = mul(dynamic, c['tail_area'])
    rates = c['limits']['Elevator'][1]
    center, positive, negative = rates[1], sub(rates[0], rates[1]), sub(rates[2], rates[1])
    command = -f32(delivered_pitch) if invert else f32(delivered_pitch)
    a = deflection([0., command, 0.], [True, False, False], c['limits']['Ailerons'])
    e = deflection([0., command, 0.], [True, False, False], c['limits']['Elevator'])
    cladd = mul(mul(a, c['ail_sens']), ail['cl'][int(a < 0.)])
    bias = sub(mul(e, -elev['wing_aoa']), mul(a, c['ail_gain']))
    angle = add(bias, ip[4])
    left = calc_c(c['polars'][0], angle, working if convert else sub(angle, g['incidence']))
    right = calc_c(c['polars'][1], angle, sub(angle, g['incidence']))
    # The effect-free native predictor stops on pass zero, before the updated
    # fin pressure is consumed. Otherwise its repeated balance uses dynamic.
    direct = (mul(abs(sub(c['limits']['Ailerons'][1][0], c['limits']['Ailerons'][1][2])), c['ail_gain']) <= f32(.001)
              and mul(abs(sub(rates[0], rates[2])), elev['wing_aoa']) <= f32(.001)
              and max(c['tail_cl']) <= f32(.001))
    vstab_drag = c['vstab_drag'] if direct else mul(dynamic, c['vstab_drag_factor'])
    other = add(mul(sub(c['fuse_y'], ip[0x20]), c['fuse_drag']), mul(vstab_drag, c['vstab_y']))
    products = [mul(ip[0x78] if flags[0x7fa3] else 1., f[0x7ca8]),
                mul(ip[0x80], f[0x7cb4]), mul(ip[0x84], f[0x7cb8]), mul(ip[0x7c], f[0x7cb0])]
    extra = add(add(products[3], products[1]), add(products[2], products[0]))
    inputs = dict(cd=[left[0], right[0]], cl=[left[1], right[1]], control_cl=cladd,
        extra_cd=[extra, extra], fuselage_cd=f[0x7cc0], q_area=c['q_area'], negative_q_area=c['negative_q_area'],
        cos_dihedral=c['cos_dihedral'], cm0=[p['clToCm0'] for p in c['polars']],
        cm1=[p['clToCm1'] for p in c['polars']], polar_area_over_span=c['polar_area_over_span'],
        wing_x=c['moment_wing_x'], wing_y=c['moment_wing_y'], other_moment=other,
        engine_pitch_moment=ip[0x64], pitch_inertia=state['pitch_inertia'], target_acceleration=0.,
        weight=ip[0x2c], flight_path_cos=c['flight_path_cos'], reference_x=ip[0x1c],
        tail_lever=c['tail_lever'], working_alpha=working,
        legacy_balance_shift=not state['new_balance'] and flags[0x7c08])
    flow = tail_flow(downwash_type=g['downwash_type'],
        legacy_factor=legacy_wake_factor(c['wash_attenuation'],qvx,c['legacy_aspect']) if g['downwash_type']==1 else 0.,
        span=g['span'], area=add(*reversed([add(add(s[1], s[0]), s[2]) for s in g['areas']])),
        sweep=g['sweep'], taper=g['taper'], dihedral=g['dihedral'], working_alpha=working, wing_angle=ip[4],
        wing_polars=c['polars'], wing_x=c['wing_x'], wing_y=c['wing_y'], wing_z=c['wing_z'],
        tail_point=[f[o] for o in [0x6f20, 0x6f24, 0x6f28]], tail_polar_offset=c['tail']['aerCenterOffset'],
        coefficient=g['downwash_coefficient'], pitch_rate=ip[0x10], tail_lever=c['tail_flow_lever'],
        speed=c['speed'], sin_angle=sn, cos_angle=cs, engine_count=state['engine_count'],
        clockwise=flags[0x6f3c], axial_wash=axial, swirl_wash=swirl,
        tail_area_delta=sub(*c['tail_areas']), inverse_tail_area=1./c['tail_area'])
    td = add(mul(positive if command >= 0. else -negative, command), center)
    required = sub(add(flow, add(ip[0x70], f[0x6f1c])), divide(td, c['area_sensitivity_scale']))
    tail_cl = calc_c(c['tail'], required, working if convert else required)[1]
    tail_force = mul(taq, sub(tail_cl, mul(c['tail_cl'][int(td < 0.)], td)))
    balance = pitch_balance(**inputs)
    lever = sub(c['tail_lever'], ip[0x1c])
    if inputs['legacy_balance_shift']:
        lever = sub(c['tail_lever'], add(mul(f32(math.sin(mul(abs(working), RAD))), .5), ip[0x1c]))
    acceleration = (balance['tail_force'] - tail_force) * lever / f32(state['pitch_inertia'])
    return dict(acceleration=acceleration, tail_force=tail_force,
                zero_acceleration_tail_force=balance['tail_force'], tail_angle_deg=required,
                tail_angle_bounds_deg=[ip[0x14], ip[0x18]], flow_angle_deg=flow,
                direct_predictor=direct)
