"""Select the first torque-balanced governor branch reached from fine pitch.

This is a branch convention for the ideal, quasi-steady governor, not a claim
that a transient governor cannot retain another stalled-blade equilibrium.
Aircraft-search warm starts must not choose that convention implicitly.
"""
import math

import numpy as np
from scipy.optimize import brentq, minimize_scalar
from prop_governor_wake import settle


REVISION = 'fine-pitch-connected-v1'
PITCH_STEP_RAD = math.radians(.5)
FLOW_TOLERANCE = 2e-8


def select(evaluate, initial, bounds, paths, blocks, targets, pitch_spans,
           *, pitch_step=PITCH_STEP_RAD, evaluators=None, accelerate=True):
    """Bracket the first restoring torque crossing at target shaft speed.

    Wake closure starts at zero flow at minimum pitch, then follows the settled
    wake from the nearest lower sampled pitch. Previous aircraft states never
    seed this traversal. Every traversed pitch must close; a failed sample is
    not skipped.
    Independent shafts are searched separately. Coupled governors require a
    joint branch search and are explicitly unresolved here.
    """
    x = np.array(initial, copy=True)
    modes = {}
    details = []
    diagnostic = dict(revision=REVISION, resolved=True,
                      policy='First restoring torque balance reached from minimum blade pitch',
                      pitch_step_deg=math.degrees(pitch_step), shafts=details)
    for shaft, span in pitch_spans.items():
        x[paths.index(('shaft', shaft))] = targets[shaft] / 300.
        x[paths.index(('pitch', shaft))] = 0.
    for shaft, span in pitch_spans.items():
        si = paths.index(('shaft', shaft))
        pi = paths.index(('pitch', shaft))
        block = next(b for b in blocks if si in b)
        if sum(paths[j][0] == 'shaft' for j in block) != 1:
            diagnostic.update(resolved=False, reason='Coupled governor branch search is unresolved')
            return x, modes, diagnostic
        flows = [j for j in block if paths[j][0] == 'flow']
        sample = (evaluators or {}).get(shaft, evaluate)
        samples = {}

        def at(fraction):
            if fraction in samples:
                return samples[fraction]
            q = x.copy()
            q[pi] = fraction
            lower = max((f for f in samples if f < fraction), default=None)
            q[flows] = 0. if lower is None else samples[lower][1][flows]
            residual = settle(sample, q, flows, bounds, FLOW_TOLERANCE, accelerate)
            samples[fraction] = float(residual[si]), q.copy()
            return samples[fraction]

        try:
            left = 0.
            low, chosen = at(left)
            mode = 0
            bracket = [0., 0.]
            if low > 0.:
                count = max(1, math.ceil(span / pitch_step))
                previous = None
                for right in np.linspace(0., 1., count + 1)[1:]:
                    high, chosen = at(float(right))
                    # A restoring/repelling root pair is born at a torque
                    # extremum. It can be much narrower than the pitch mesh:
                    # positive endpoints alone cannot rule out that pair.
                    if previous is not None and low < previous[1] and low < high:
                        minimum = minimize_scalar(lambda f: at(f)[0],
                            bounds=(previous[0], float(right)), method='bounded',
                            options=dict(xatol=2e-10))
                        if not minimum.success:
                            raise ValueError('Governor torque extremum is unresolved')
                        if minimum.fun <= 0.:
                            root = brentq(lambda f: at(f)[0], previous[0], float(minimum.x),
                                          xtol=2e-10, rtol=1e-12)
                            _, chosen = at(root)
                            mode = 'target'
                            bracket = [previous[0], float(minimum.x)]
                            break
                    if high <= 0.:
                        root = brentq(lambda f: at(f)[0], left, float(right),
                                      xtol=2e-10, rtol=1e-12)
                        _, chosen = at(root)
                        mode = 'target'
                        bracket = [left, float(right)]
                        break
                    previous = (left, low)
                    left = float(right)
                    low = high
                else:
                    mode = 1
                    bracket = [1., 1.]
            x[list(block)] = chosen[list(block)]
            modes[shaft] = mode
            details.append(dict(shaft=shaft, mode=mode, pitch_fraction=float(x[pi]),
                                bracket_fraction=bracket, pitch_samples=len(samples)))
        except (ValueError, RuntimeError) as error:
            diagnostic.update(resolved=False, reason=str(error))
            return x, modes, diagnostic
    return x, modes, diagnostic
