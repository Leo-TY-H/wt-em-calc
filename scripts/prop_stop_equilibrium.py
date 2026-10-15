"""Bracket stopped-pitch shaft balance when the simultaneous fit stagnates."""
import numpy as np
from scipy.optimize import brentq
from prop_governor_wake import settle


def recover(evaluate, initial, paths, bounds, blocks, targets, modes, tolerance):
    x = initial.copy()
    for shaft, mode in modes.items():
        if mode == 'target':continue
        si = paths.index(('shaft', shaft))
        pi = paths.index(('pitch', shaft))
        block = next(b for b in blocks if si in b)
        if sum(paths[j][0] == 'shaft' for j in block) != 1:return None
        flows = [j for j in block if paths[j][0] == 'flow']
        start = targets[shaft]/300.
        end = bounds[0][si] if mode == 0 else bounds[1][si]
        samples = {}

        def at(speed):
            if speed not in samples:
                q = x.copy();q[si] = speed;q[pi] = float(mode)
                if samples:
                    near = min(samples, key=lambda v:abs(v-speed))
                    q[flows] = samples[near][1][flows]
                residual = settle(evaluate,q,flows,bounds,2e-8,True)
                samples[speed] = (float(residual[si]),q.copy())
            return samples[speed][0]

        try:
            left = start;low = at(left)
            if abs(low) <= tolerance:
                x[list(block)] = samples[left][1][list(block)]
                continue
            for right in np.linspace(start,end,33)[1:]:
                high = at(float(right))
                if low*high <= 0.:
                    root = brentq(at,min(left,right),max(left,right),xtol=2e-12,rtol=1e-12)
                    at(root)
                    x[list(block)] = samples[root][1][list(block)]
                    break
                left,low = float(right),high
            else:return None
        except (ValueError, RuntimeError):
            return None
    return x if np.max(abs(evaluate(x)),initial=0.) <= tolerance else None
