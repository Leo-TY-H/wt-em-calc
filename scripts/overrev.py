"""Engine over-rev on the chart (Leo 2026-10-26, option C): an engine RPM above RPMMaxAllowed is charted, not a limit.

The game caps no piston engine RPM. In aces.exe the piston torque function (0x142f9fa90, its branch at 0x142f9fd8e)
runs this rule on every 48 Hz main step of a player aircraft when the battle's Engine_Overheat difficulty switch is on
(read at 0x140c9b8e1): while the engine RPM is above RPMMaxAllowed x s (s starts at 1)

- s <- 0.999965 s (the allowed RPM falls while the engine over-revs);
- counter <- counter + 0.01 + 0.05 x excess / (RPMMaxAllowed x s), excess = RPM - RPMMaxAllowed x s;
- once the counter is above 1, health <- health - 0.005 (counter - 1); where health falls below 0.2 the engine stops
  ('engineOverreved');
- the engine torque is multiplied by 1 + 0.01 x excess (rad/s);

and on every main step the counter then falls by 0.01 (not below 0). Jet engines are clamped at RPMMaxAllowed x s and
turboprops at RPMMaxAllowed, so neither over-revs.

The chart keeps a fresh engine (s = 1): an over-rev state is admitted like any other (no 'engine RPM limit'
rejection, under the march chart's admission rule), solved with the torque of the first over-rev step (piston_model,
the native prop.pxi), and carries the seconds the game's rule gives that engine before it stops at a steady RPM
(``seconds_to_stop``). The chart draws the iso-line RPM = RPMMaxAllowed where over-rev starts (NTracer.overrev_trace,
em_march_chart.overrev_lines).

WT_OVERREV=0 restores the deployed chart exactly (RPMMaxAllowed a soft chart limit: S8's transient over-rev, S16's
leaning engine RPM edge; an intact engine's torque). WT_OVERREV_TORQUE=0 keeps the rule without the over-rev torque.
"""
import os

import numpy as np

RULE = os.environ.get('WT_OVERREV', '1') != '0'
TORQUE = RULE and os.environ.get('WT_OVERREV_TORQUE', '1') != '0'

# aces.exe constants (float32)
LIMIT_DECAY = .9999650120735168     # s per over-rev step (0x1471b791c)
TORQUE_GAIN = .009999999776482582   # torque gain per rad/s of excess
STEP_HZ = 48.                       # the game's flight-model step
REPORT_PERCENT = (0., 1., 2., 5., 10., 20.)   # the over-rev line's hover table: percent over RPMMaxAllowed


def rule(solver):
    """Over-rev is no chart limit (option C) under the march chart's admission rule (solver.soft_admission); elsewhere
    (the baseline sampler, the Python march) the deployed rule stays."""
    return RULE and bool(getattr(solver, 'soft_admission', False))


def _simulate(ratios, max_steps=4096):
    """Seconds to the engine stop for each RPM / RPMMaxAllowed in ``ratios`` (> 1), the game's rule above in float32
    with the RPM held, a fresh engine (s = 1, counter 0, health 1); NaN where it does not stop within ``max_steps``."""
    f = np.float32
    omega = np.asarray(ratios, dtype=f)
    s = np.ones_like(omega);acc = np.zeros_like(omega);health = np.ones_like(omega)
    stop = np.full(omega.shape, np.nan)
    live = np.ones(omega.shape, dtype=bool)
    decay, k05, k01, k005, k02, one = f(LIMIT_DECAY), f(.05), f(.01), f(.005), f(.2), f(1.)
    for n in range(1, max_steps+1):
        over = live & (omega > s)
        s = np.where(over, (s*decay).astype(f), s)
        excess = (omega-s).astype(f)
        rise = np.where(s > f(4e-19), ((k05*excess).astype(f)/s).astype(f), f(0.))
        acc = np.where(over, ((rise+k01).astype(f)+acc).astype(f), acc)
        hurt = over & (acc > one)
        after = ((k005-(acc*k005).astype(f)).astype(f)+health).astype(f)
        ends = hurt & (health >= k02) & (after < k02)
        health = np.where(hurt, after, health)
        stop[ends] = n/STEP_HZ
        live &= ~ends
        acc = np.maximum(f(0.), (acc-k01).astype(f))
        if not live.any():break
    return stop


_TABLE = None


def _table():
    """(RPM / RPMMaxAllowed, seconds to the stop): a grid fine near the limit, from just over it to 4 x RPMMaxAllowed."""
    global _TABLE
    if _TABLE is None:
        excess = np.unique(np.r_[1e-6, np.linspace(1e-4, .05, 500), np.linspace(.05, .5, 451), np.linspace(.5, 3., 126)])
        ratio = 1.+excess
        _TABLE = (ratio, _simulate(ratio))
    return _TABLE


def seconds_to_stop(ratio):
    """Seconds of steady over-rev at RPM / RPMMaxAllowed = ``ratio`` until the game stops the engine (a fresh engine,
    Engine_Overheat on), from the simulated table (linear in the ratio); None at or below the limit or where unknown."""
    if ratio is None:return None
    r = float(ratio)
    if not (r > 1.):return None
    x, t = _table()
    value = float(np.interp(min(r, float(x[-1])), x, t))
    return round(value, 1) if np.isfinite(value) else None


def report_table():
    """[[percent over RPMMaxAllowed, seconds to the stop]] for the over-rev line's hover (0%: just over the limit)."""
    return [[p, seconds_to_stop(1.+max(p, 1e-4)/100.)] for p in REPORT_PERCENT]


def rpm_ratio(row):
    """The highest engine RPM against its RPMMaxAllowed of a chart row (its 'propulsion' record), or None."""
    if row is None:return None
    if row.get('engine_rpm_ratio') is not None:return float(row['engine_rpm_ratio'])
    p = row.get('propulsion') or {}
    rpm, limit = p.get('engine_rpm') or [], p.get('engine_rpm_threshold') or []
    pairs = [(float(a), float(b)) for a, b in zip(rpm, limit) if a is not None and b]
    return max((a/b for a, b in pairs), default=None)


def annotate(row):
    """Add the over-rev record to a chart row whose engine over-revs: {'rpm', 'rpm_limit', 'percent_over',
    'seconds_to_stop'} of its engine with the highest RPM against RPMMaxAllowed (rpm), and return the row."""
    if not RULE or not isinstance(row, dict):return row
    p = row.get('propulsion') or {}
    rpm, limit = p.get('engine_rpm') or [], p.get('engine_rpm_threshold') or []
    best = None
    for a, b in zip(rpm, limit):
        if a is None or not b:continue
        if best is None or a/b > best[0]/best[1]:best = (float(a), float(b))
    if best is None or not best[0] > best[1]*(1+1e-6):
        row.pop('overrev', None)
        return row
    ratio = best[0]/best[1]
    row['overrev'] = dict(rpm=best[0], rpm_limit=best[1], percent_over=100.*(ratio-1.), seconds_to_stop=seconds_to_stop(ratio))
    return row
