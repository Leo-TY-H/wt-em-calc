"""Source-value admission for the declared propeller operating regime.

This checks one candidate. It is neither root coverage nor an interval/error
certificate, and rejection never establishes absence at an aircraft condition.
"""
import math

from prop_quasisteady import blade_forces, blade_polar, propeller_load

REVISION = 'forward-any-stall-any-power-v3'


def evaluate(properties, flow, pitch, omega, density, sound, condition):
    p = properties
    local, _, _, transverse, _, _, upstream, prepared = condition
    sections, area_unit = prepared[:2]
    if (not all(math.isfinite(v) for v in (*flow, pitch, omega, density, sound, upstream))
            or sound <= 0. or density <= 0. or area_unit <= 0. or p['blades'] <= 0
            or any(r <= 0. or width < 0. or not all(map(math.isfinite, (r, twist, width)))
                   for r, twist, width in sections)):
        return dict(available=False, admitted=False, revision=REVISION,
                    reason='Invalid source geometry or nonfinite propeller inputs')
    axial = local[0] + .5 * (flow[0] + flow[1])
    _, _, wake = propeller_load(p, flow, pitch, omega, density, sound, condition)
    margins = [dict(name='upstream', value=upstream, strict=True),
               dict(name='blade axial flow', value=axial, strict=True)]
    if p['coaxial']:
        margins.append(dict(name='second rotor upstream', value=upstream + .5 * wake[0], strict=True))
    rotors = []
    for rotor, sign in enumerate((1., -1.) if p['coaxial'] else (1.,)):
        angular = sign * .25 * flow[2]
        load = blade_forces(p, omega, pitch, axial, transverse, density, sound, angular, prepared)
        power = omega * load['torque']
        # Power direction is diagnostic, not an admission inequality.
        rotor_margins = []
        rows = []
        for index, (radius, twist, width) in enumerate(sections):
            if width == 0.:continue
            tangential = radius * (omega - angular)
            phi = math.atan2(axial, tangential) if abs(omega) >= 1e-5 else math.pi / 2
            alpha = math.degrees(pitch - phi + twist)
            mach = math.hypot(tangential + transverse, axial) / sound
            polar = blade_polar(p['polar'], mach, moments=False)
            # The retained polar also models stalled sections. These signed
            # distances are diagnostics, never physical admission inequalities.
            stall_margins = dict(lower_deg=alpha - polar['aoaCritL'],
                                 upper_deg=polar['aoaCritH'] - alpha)
            section = [dict(name='tangential flow', value=tangential, strict=True)]
            rows.append(dict(section=index, alpha_deg=alpha, mach=mach,
                             critical_deg=[polar['aoaCritL'], polar['aoaCritH']],
                             stall_margins_deg=stall_margins, margins=section))
            rotor_margins.extend(dict(m, section=index) for m in section)
        rotors.append(dict(rotor=rotor, torque=load['torque'], power=power,
                           sections=rows, margins=rotor_margins))
        margins.extend(dict(m, rotor=rotor) for m in rotor_margins)
    finite_power = all(math.isfinite(r['power']) and math.isfinite(r['torque']) for r in rotors)
    available = finite_power and all(math.isfinite(m['value']) for m in margins)
    failures = [m for m in margins if not math.isfinite(m['value'])
                or (m['value'] <= 0. if m['strict'] else m['value'] < 0.)]
    failures.extend(dict(name='nonfinite rotor torque or power', rotor=r['rotor'],
                         value=r['power']) for r in rotors
                    if not math.isfinite(r['power']) or not math.isfinite(r['torque']))
    return dict(available=available, admitted=available and not failures,
                margins=margins, violations=failures, rotors=rotors,
                revision=REVISION, interval_certified=False)
