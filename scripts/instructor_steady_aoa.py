"""Algebraic steady Instructor limit, without maneuver/controller integration.

The native rate proxy is nonzero in a balanced turn. Retain its PD feedback
and equate demand to the reduced predictor's forward moment balance at the
delivered elevator. Raw critical-angle targets alone overpredict allowed AoA.
One-g auto trim bounds control authority independently at every point.
Transient history remains outside this model.
"""
from instructor_aoa import controller_limits as effective_limits

REVISION = 'steady-effective-aoa-fixed-autotrim-v6'


def controller_limits(solver, value, speed=None):
    if value.get('instructor') is not None:
        return value['instructor']
    phases = value.get('phase_results') or [value['result']]
    results = [effective_limits(solver, dict(value, result=phase, instructor=None), speed)
               for phase in phases]
    # Phases share the balanced aircraft state. Check each wing-angle/air source;
    # this does not certify a periodic aircraft trajectory.
    result = dict(min(results, key=lambda item: item['margin']))
    result['converged'] = all(item['converged'] for item in results)
    result['adjusted_wing_angles_deg'] = [
        min(min(item['adjusted_wing_angles_deg']) for item in results),
        max(max(item['adjusted_wing_angles_deg']) for item in results)]
    result.update(model='Steady effective AoA schedule (approximation)',
                  model_revision=REVISION, pitch_predictor_recheck=False)
    value['instructor'] = result
    return result
