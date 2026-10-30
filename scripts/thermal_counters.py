"""Exact float32 load-counter evolution at a constant rate, without stepping.

Recurrence (research/thermal-fatigue-2026-10-29/thermal_core.py:42-45, endurance_step):
    d = f(rate*dt);  v = f(c + d);  overflow if v > 1 and rate > 0;  c' = clamp(v,0,1)
Inside one binade [2^e, 2^(e+1)) every float32 is a multiple of u = 2^(e-23),
so f(c+d) = c + k*u with k = round(d/u) (constant) unless d/u is a half-integer
(ties-to-even: the increment becomes constant after at most one step).
The loops jump whole runs of constant increments and do the exact float32
step only at binade boundaries: O(number of binades) per counter.
k = 0 means the counter stalls for ever (increment below half an ulp).

ticks_from_zero is the integer form used by the engine ratings (counters start at 0); evolve takes any start
value and is its reference.
"""
import math
from thermal_core import f, add, mul

TINY = 2.0 ** -126
UNIT_EXP = 149                 # every float32 is a whole number of 2^-149 units
ONE = 1 << UNIT_EXP


def ulp_of(c):
    c = abs(c)
    if c < TINY:
        return 2.0 ** -149
    e = math.frexp(c)[1] - 1          # c in [2^e, 2^(e+1))
    return 2.0 ** (e - 23)


def evolve(c, rate, dt, horizon=None):
    """Return (ticks, state, value) where state is 'overflow' (tick of the overflow update),
    'zero' (tick at which counter first reaches 0), 'stall' (ticks = None, value
    stalls below 1 / above 0 or at 1 without overflow) or 'horizon'."""
    c = f(c); d = mul(rate, dt); n = 0
    if d == 0.:
        return None, 'stall', c
    while True:
        if horizon is not None and n >= horizon:
            return n, 'horizon', c
        if d > 0 and c >= 1.:
            v = add(c, d)
            return (n + 1, 'overflow', 1.) if v > 1. else (None, 'stall', c)
        if d < 0 and c <= 0.:
            return n, 'zero', c
        u = ulp_of(c) if c != 0. else 2.0 ** -149
        # exact increments for up to 2 steps to detect the per-binade pattern
        c1 = add(c, d); c2 = add(c1, d)
        i1, i2 = c1 - c, c2 - c1
        if c1 == c:
            return None, 'stall', c
        if c == 0. or ulp_of(c1) != u or ulp_of(c2) != u or (d > 0 and c2 > 1.) or (d < 0 and c2 <= 0.):
            # boundary region: take one exact step (clamp as the game does)
            v = c1; n += 1
            if d > 0 and v > 1.:
                return n, 'overflow', 1.
            c = min(1., max(0., v))
            continue
        # Non-tie: increment constant. Tie (d/u half-integer): ties-to-even makes
        # the increment constant after at most one transitional step, so a
        # stable configuration is recognised by two equal consecutive increments.
        if i1 != i2:
            v = c1; n += 1
            c = min(1., max(0., v))
            continue
        per, step2 = i1, 1
        # binade limits
        if d > 0:
            hi = min(2.0 * u * 2 ** 23, 1.)                  # 2^(e+1) or 1
            room = hi - c - abs(d) - 2 * u
        else:
            lo = u * 2 ** 23 if c >= TINY else 0.            # 2^e
            room = c - lo - abs(d) - 2 * u
        jumps = int(room // abs(per)) if room > 0 else 0
        if horizon is not None:
            jumps = min(jumps, (horizon - n) // step2)
        if jumps <= 0:
            v = c1; n += 1
            if d > 0 and v > 1.:
                return n, 'overflow', 1.
            c = min(1., max(0., v))
            continue
        c = c + jumps * per                                 # exact: multiple of u inside binade
        assert f(c) == c
        n += jumps * step2


def _round32(x):
    """Nearest float32 (ties to even) of the positive integer x (units 2^-149), below float32 overflow."""
    s = x.bit_length() - 24
    if s <= 0:
        return x
    q, r = divmod(x, 1 << s)
    if 2 * r > (1 << s) or (2 * r == (1 << s) and q & 1):
        q += 1
    return q << s


def ticks_from_zero(rate, dt):
    """Tick (1-based) of the first overflow of a counter starting at 0, or None if it never overflows.

    Integers in units of 2^-149 make every float32 rounding an integer round-half-even to the binade's ulp; the
    increment inside a binade is a constant multiple of that ulp (after at most one step on a tie), so whole runs
    are jumped. Equal to evolve(0, rate, dt) and to brute-force stepping."""
    if not rate > 0.:
        return None
    d = mul(rate, dt)
    if not d > 0.:
        return None
    D = int(d * 2.0 ** UNIT_EXP)
    C = 0; n = 0
    while True:
        if C >= ONE:                                   # clamped at exactly 1: overflows next tick or never
            return n + 1 if _round32(ONE + D) > ONE else None
        e = C.bit_length() - 1 if C else 0
        s = e - 23
        if s <= 0:                                     # exact region (ulp = 1 unit)
            U = 1; inc = D; B = 1 << 24
        else:
            U = 1 << s; B = 1 << (e + 1)
            q, r = divmod(D, U)
            if 2 * r > U: k = q + 1
            elif 2 * r < U: k = q
            else:                                      # tie: settles on the even one of q, q+1 after one step
                if (C >> s) & 1: k = None
                else: k = q if q % 2 == 0 else q + 1
            inc = None if k is None else k * U
            if inc == 0:                               # increment rounds away: stalls below 1
                return None
        if inc is not None and C:
            room = B - U - D - C - 1
            if room >= 0:
                j = room // inc + 1
                C += j * inc; n += j
                if C >= ONE: C = ONE                   # cannot happen (B <= ONE keeps C < ONE - U); defensive
                continue
        v = _round32(C + D); n += 1                    # exact single step at a binade boundary
        if v > ONE:
            return n
        if v == C:
            return None
        C = v
