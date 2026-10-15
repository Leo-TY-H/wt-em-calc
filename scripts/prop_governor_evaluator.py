"""Exact fixed-RPM residuals without repeating unrelated engine/frame work."""
import numpy as np

from em_cancellation import check as check_cancel
from em_trim_work import charge
from prop_quasisteady import propeller_condition, propeller_load
from propulsion_general import fixed_air
from em_manual_controls import manual_rpm_governor


def prepare(properties, frame, paths, targets, spans, torque_scales,
            velocity, height, body_omega, cg, pitch_limits):
    """Reuse only constants for this exact flight condition and target RPM.

    The ordinary complete residual must validate the selected root afterwards.
    These evaluators neither update its mutable snapshot nor cache across
    flight conditions. Each shaft samples only its own independent propellers.
    """
    air, sound = fixed_air(tuple(velocity), height)
    evaluators = {}
    counts = dict(samples=0, propeller_evaluations=0)
    for shaft in spans:
        transmission = properties['transmissions'][shaft]
        si = paths.index(('shaft', shaft))
        pi = paths.index(('pitch', shaft))
        torque = sum(frame['engines'][link['index']]['torque'] * link['ratio']
                     for link in transmission['engines'])
        props = []
        for link in transmission['propellers']:
            index = link['index']
            pp = properties['propellers'][index]['properties']
            state = frame['propellers'][index]
            omega = targets[shaft] * link['ratio']
            lo, hi = pitch_limits(pp, velocity, body_omega, omega)
            manual = pp['governor'] and not state['auto'] and not manual_rpm_governor(pp)
            fixed_pitch = (pp['pitch_max'] + state['command']*(pp['pitch_min']-pp['pitch_max'])
                           if manual else None)
            flows = [(j, path[2]) for j, path in enumerate(paths)
                     if path[:2] == ('flow', index)]
            condition = propeller_condition(pp, velocity, body_omega)
            props.append((pp, list(state['flow']), omega, lo, hi-lo, fixed_pitch, flows, link['ratio'], condition))

        def make_sample(si, pi, torque, scale, props):
            def evaluate_into(x, residual):
                check_cancel()
                charge('governor_samples')
                counts['samples'] += 1
                fraction = min(1., max(0., float(x[pi])))
                total_load = 0.
                for pp, flow, omega, lo, span, fixed_pitch, flows, ratio, condition in props:
                    for j, axis in flows:flow[axis] = float(x[j]) * 20.
                    pitch = lo + fraction*span if fixed_pitch is None else fixed_pitch
                    charge('governor_propeller_evaluations')
                    counts['propeller_evaluations'] += 1
                    _, load, target = propeller_load(pp, flow, pitch, omega,
                                                    air['density'], sound, condition)
                    total_load += load * ratio
                    for j, axis in flows:
                        residual[j] = (target[axis]-flow[axis]) / 20.
                residual[si] = (torque-total_load) / scale

            def sample(x):
                residual = np.zeros(len(paths))
                evaluate_into(x, residual)
                return residual
            sample.evaluate_into = evaluate_into
            return sample
        evaluators[shaft] = make_sample(si, pi, torque, torque_scales[shaft], props)
    return evaluators, counts
