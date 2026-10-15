from instructor_aoa import controller_limits as effective_limits
from struct import pack

REVISION = 'steady-effective-aoa-fixed-autotrim-v9'


def controller_limits(solver, value, speed=None):
    if value.get('instructor') is not None:
        return value['instructor']
    phases = value.get('phase_results') or [value['result']]


    sources = {}
    results = []
    for phase in phases:
        air = phase['air']; angles = phase['history']['wing_aoa']
        key = pack('=4d', *(air[k] for k in ('tas', 'speed_squared', 'mach', 'ias_u')))
        key += pack('=' + 'd' * len(angles), *angles)
        if key not in sources:
            sources[key] = effective_limits(solver, dict(value, result=phase, instructor=None), speed)
        results.append(sources[key])


    result = dict(min(results, key=lambda item: item['margin']))
    result['converged'] = all(item['converged'] for item in results)
    result['adjusted_wing_angles_deg'] = [
        min(min(item['adjusted_wing_angles_deg']) for item in results),
        max(max(item['adjusted_wing_angles_deg']) for item in results)]
    result.update(model='Steady effective AoA schedule (approximation)',
                  model_revision=REVISION, pitch_predictor_recheck=False)
    from instructor_trim_equilibrium import REVISION as authority_revision
    result['authority_revision'] = authority_revision
    value['instructor'] = result
    return result
