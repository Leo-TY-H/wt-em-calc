"""Wake closure shared by the compiled backend and Python reference path."""
import math
import numpy as np


def settle(evaluate, q, flows, bounds, tolerance, accelerate):
    """Mutate only this trial's coordinates; keep all scratch storage private."""
    initial = q.copy()
    residual = np.zeros(len(q))
    into = getattr(evaluate, 'evaluate_into', None)
    previous_flow = [0.] * len(flows)
    previous_residual = [0.] * len(flows)
    previous_valid = False
    previous_error = math.inf
    relaxation = 1.
    for iteration in range(80):
        if into is None:residual[:] = evaluate(q)
        else:into(q, residual)
        for raw in residual:
            if not math.isfinite(raw):
                raise ValueError('Nonfinite governor branch sample')
        error = 0.
        for j in flows:error = max(error, abs(residual[j]))
        if error <= tolerance:return residual
        if error > previous_error * 1.05:
            relaxation = max(.125, relaxation * .5)
        previous_error = error
        for k, j in enumerate(flows):
            current = float(q[j])
            value = float(residual[j])
            update = relaxation * value
            if accelerate and previous_valid:
                difference = value - previous_residual[k]
                if abs(difference) > 1e-14:
                    secant = -value*(current-previous_flow[k])/difference
                    if abs(secant) <= 2.*abs(update):update = secant
            q[j] = min(bounds[1][j], max(bounds[0][j], current+update))
            previous_flow[k] = current
            previous_residual[k] = value
        previous_valid = True
    from prop_wake_recovery import recover
    return recover(evaluate, initial, q, flows, bounds, tolerance)
