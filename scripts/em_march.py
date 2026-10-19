"""Alpha-march envelope solver (the default chart solver since S4, 2026-10-18).

Moved from research/speed_overhaul (s1_march, s3_march) and
research/output_solver_v1 (tracer) without changing the search; see
SPEED-PLAN.md and outputs/speed-s1/s3 reports for its validation.

At one speed the march starts from admitted level flight and increases alpha
with the production stationary corrections at prescribed alpha
(``Curve.correct(alpha=...)``: joint balance equations, load free, every
returned state replayed through production admission). The first limit
reached is the boundary (Leo, 2026-10-17):

- a physical margin (stall, wing force, control, Instructor pitch, pitch
  response, engine RPM) that crosses zero between two samples: the crossing is
  localized in alpha by a safeguarded secant (Illinois) on the smallest margin,
  each trial again a prescribed-alpha correction; the admitted side within the
  turn tolerance is the boundary state (as production ``bracket`` /
  ``em_constraint_bracket.refine`` mark it);
- a load maximum (trim fold): localized by parabolic steps in alpha;
- the chart load cap (plot ceiling).

The step is the smallest of: the maximum alpha step, the neighbouring speed's
boundary alpha (prediction), the linear and quadratic extrapolation of every
margin to zero, and the parabolic extrapolation of the load maximum. A sample
is always placed just below the native roll-leveling switch at 12 deg, so the
production roll-leveling first-limit check is part of the march.

Where the march gives up ('march correction unresolved': trials above the last
admitted sample do not close), ``RescueMarch`` tries the nearest physical faces
and then an alpha bisection from the admitted side (S3). Two guards are
relaxed by at most the march tolerance (0.05 deg/s) and flagged per column in
``s3_flags``: a face up to 0.5 deg behind the last admitted sample whose turn
rate is within the tolerance, and a load maximum less than the tolerance above
a face. Anything still unresolved uses the production boundary column search
at that speed (``production_column``); every column records how it was
resolved in ``s3_path``.

The column record has the production column shape, so the tracer's sweep,
corner/edge localization and withheld checks and the curve solver's column
mode consume it unchanged: the SEP knots come from the same march samples.

Policy: zero prescribed sideslip. The march is only run on solver configs with
``sideslip_recovery`` False (see em_march_chart); nothing here changes physics
or admission. Unresolved searches are never absence proofs: the tracer keeps
them as explicit gaps or edges.
"""
from __future__ import annotations

import json
import math
import time

import numpy as np

G = 9.8100004196167
MAX_STEP_DEG = 8.
APPROACH_DEG = 2.  # a predicted margin crossing farther than this is approached, not jumped to
OVERSHOOT_DEG = .1  # past a predicted crossing: balanced rejections close just past a limit, rarely far past it
TURN_TOLERANCE = .05  # localization target, deg/s (a third of the 0.15 envelope tolerance)
SWITCH_DEG = 12.
MAX_NFEV = 12
MAX_EVALUATIONS = 40  # all evaluations of one march trial, derivative probes included
HARD_FACES = ('control', 'propeller pitch', 'engine RPM')
STATS = dict(columns=0, fallback=0, reasons={}, predictors_offered=0)
# Cross-speed Jacobian hints: per model (config JSON) and prescribed-alpha
# predictor key, the corrector predictors left by earlier columns with the
# speed and alpha they were formed at. A hint only; Corrector rejects one whose
# anchor or derivative branch does not match and refreshes a poor prediction.
PREDICTORS = {}
REUSE_WINDOW_KMH = 60.
REUSE = True


def turn_rate(speed, load):
    return math.degrees(G*math.sqrt(max(0., load*load-1.))/(speed/3.6))


def physical(p):
    from em_pitch_response import PHYSICAL_REASONS
    return bool(p is not None and p.get('converged') and p.get('reasons') and set(p['reasons']) <= PHYSICAL_REASONS)


def margins(solver, p):
    """Physical margins of a balanced state (positive = admitted side).

    The same quantities production uses for its constraint bracket
    (em_constraint_bracket.refine); pitch response and Instructor margins exist
    only where production evaluated them (no earlier rejection)."""
    out = {}
    if p.get('stall_margin_deg') is not None:out['stall'] = p['stall_margin_deg']/10.
    if p.get('authority_margin') is not None and not solver.config['instructor']:out['control'] = p['authority_margin']
    if solver.config['structural_limits'] and p.get('wing_load_ratios'):out['wing force'] = 1.-max(p['wing_load_ratios'])
    pr = p.get('pitch_response')
    if pr is not None and pr.get('margin') is not None and math.isfinite(pr['margin']):out['pitch response'] = pr['margin']
    if p.get('stationary_governor_mode') == 'pitch_stops':
        prop = p.get('propulsion') or {}
        rpm = [1.-v/(lim*(1.+1e-6)) for v, lim in zip(prop.get('engine_rpm', ()), prop.get('engine_rpm_threshold', ()))]
        if rpm:out['engine RPM'] = min(rpm)
    if solver.config['instructor']:
        ctl = p.get('instructor')
        if ctl is not None and ctl.get('converged'):
            out['Instructor pitch'] = ctl['envelope_margin']
            out['control'] = ctl['margins']['control authority with auto trim']
    return out


def combined(solver, p):
    m = margins(solver, p)
    return min(m.values()) if m else None


def limit_kind(solver, high, low=None):
    from em_pitch_response import KINDS
    crossed = sorted({KINDS[r] for r in high['reasons'] if r in KINDS})
    if 'Instructor pitch' in crossed:return 'Instructor pitch'
    if len(crossed) == 1:return crossed[0]
    m = margins(solver, high)
    return min((k for k in crossed if k in m), key=lambda k: m[k], default=crossed[0] if crossed else None)


def control_axis(point):
    bounds = (point.get('instructor') or {}).get('control_bounds', point.get('control_bounds'))
    commands = point.get('commands')
    if commands is None or bounds is None:return None
    return 2+min(range(3), key=lambda i: min(commands[i]-bounds[i][0], bounds[i][1]-commands[i]))


class March:
    """One speed. ``curve`` is a production Curve on the model being marched."""

    def __init__(self, solver, speed, *, max_step=MAX_STEP_DEG, turn_tol=TURN_TOLERANCE, record=None, max_nfev=None,
                 model=None, reuse=True):
        from em_stationary_boundary import Curve
        self.solver, self.speed = solver, float(speed)
        self.curve = Curve(solver, speed)
        self.max_step, self.turn_tol = max_step, turn_tol
        # A well-seeded march trial closes in a few iterations; an unclosed one
        # is retried smaller or replaced by an active-face solve (never absence).
        self.max_nfev = max_nfev or MAX_NFEV
        self.samples = []      # every admitted state at this speed (march + localization)
        self.trials = record if record is not None else []
        self.alpha_upper = float(solver.trim_bounds[1][0])
        self.store = PREDICTORS.setdefault(model, {}) if reuse and model is not None else None
        self.key = (self.store or {}).get('_alpha_key')
        self.hints = []

    # ----- primitives ------------------------------------------------------
    # ----- cross-speed warm starts -------------------------------------------
    # Each closed march trial is stored with its speed, alpha and corrector
    # predictor. A trial at another speed and a nearby alpha starts from that
    # state (alpha replaced) with its predictor: the predictor's anchor is then
    # within the corrector's trust box, so the correction usually closes with
    # the reused derivative (about 5 evaluations instead of about 13).
    def neighbours(self):
        if self.store is None:return ()
        return [r for r in self.store.get('rows', ()) if r[0] != self.speed and abs(r[0]-self.speed) <= REUSE_WINDOW_KMH]

    def warm(self, seed, alpha, base_alpha):
        best, score = None, abs(base_alpha-alpha) if base_alpha is not None else float('inf')
        side = lambda x: (x >= SWITCH_DEG)-(x <= -SWITCH_DEG)
        request = tuple(seed.get('stationary_request_indices', ()))
        for v, a, state, pred in self.neighbours():
            # Same native roll-leveling side and the same requested-RPM branch.
            if side(a) != side(alpha) or tuple(state.get('stationary_request_indices', ())) != request:continue
            d = abs(a-alpha)+abs(v-self.speed)/25.
            if d < score:best, score = (state, pred), d
        if best is None:return seed, None
        state, pred = best
        x = list(state['solution']);x[0] = float(alpha)
        STATS['predictors_offered'] += 1
        return dict(state, solution=x, speed_kmh=self.speed), pred

    def keep(self, alpha, p):
        if self.store is None:return
        if self.key is None:
            keys = [k for k in self.curve.predictors if k is not None and k[0] == 0]
            if not keys:return
            self.key = self.store['_alpha_key'] = keys[0]
        pred = self.curve.predictors.get(self.key)
        # Only admitted states warm-start other speeds: a rejected state can
        # sit on another branch (e.g. past a pitch-response reversal).
        if pred is None or p is None or not p.get('valid'):return
        rows = self.store.setdefault('rows', [])
        rows.append((self.speed, float(p['alpha_deg']), p, pred))
        if len(rows) > 600:del rows[:200]

    def level_from(self, neighbour):
        """Level flight by the same stationary correction at prescribed load
        (1 g) from a neighbouring speed's level state and its predictor."""
        from em_trim_work import Work, use, current
        rows = [r for r in (self.store or {}).get('level_rows', ()) if r[0] != self.speed]
        key = (self.store or {}).get('_level_key')
        if rows and key is not None:
            v, state, pred = min(rows, key=lambda r: abs(r[0]-self.speed))
            if abs(v-self.speed) <= REUSE_WINDOW_KMH:
                neighbour = state;self.curve.predictors[key] = pred
        w = Work({}, parent=current())
        with use(w):p = self.curve.correct(neighbour, load=1., max_nfev=self.max_nfev, max_evaluations=MAX_EVALUATIONS)
        self.trials.append(dict(why='level correction', ok=None if p is None else bool(p['valid']),
                                evals=w.counts.get('aircraft_evaluations', 0), fresh=w.counts.get('derivative_samples', 0)))
        if self.store is not None and p is not None and p['valid']:
            keys = [k for k in self.curve.predictors if k is not None and k[0] != 0 and k is not self.key]
            if keys:
                self.store['_level_key'] = keys[0]
                rows = self.store.setdefault('level_rows', []);rows.append((self.speed, p, self.curve.predictors[keys[0]]))
                if len(rows) > 200:del rows[:50]
        return p if p is not None and p['valid'] else None

    def snap(self, low, target):
        """A march target moved down onto a neighbouring column's sample alpha
        (within the last 30% of the step), so that sample warm-starts it."""
        lo = low['alpha_deg']+.7*(target-low['alpha_deg'])
        near = [a for v, a, _, _ in self.neighbours() if lo <= a <= target and abs(v-self.speed) <= REUSE_WINDOW_KMH/2]
        return max(near) if near else target

    def at(self, seed, alpha, why, max_nfev=None, base_alpha=None):
        from em_trim_work import Work, use, current
        if base_alpha is None:base_alpha = seed.get('alpha_deg')
        # Cross-speed warm starts for march steps only: localization and fold
        # trials stay on this column's own bracket states (a neighbouring
        # speed's state can sit on another branch next to a reversal).
        seed, pred = self.warm(seed, float(alpha), base_alpha) if self.store is not None and why == 'march' else (seed, None)
        if pred is not None and self.key is not None:self.curve.predictors[self.key] = pred
        w = Work({}, parent=current())
        with use(w):p = self.curve.correct(seed, alpha=float(alpha), max_nfev=max_nfev or self.max_nfev,
                                          max_evaluations=MAX_EVALUATIONS)
        self.keep(alpha, p)
        self.trials.append(dict(why=why, alpha=round(float(alpha), 4), ok=None if p is None else bool(p['valid']),
                                evals=w.counts.get('aircraft_evaluations', 0), fresh=w.counts.get('derivative_samples', 0),
                                warm=pred is not None, load=None if p is None else round(p['load_g'], 5),
                                reasons=None if p is None else list(p['reasons'])))
        if p is not None and p['valid']:self.samples.append(p)
        return p

    def nearest(self, alpha):
        return min(self.samples, key=lambda p: abs(p['alpha_deg']-alpha))

    @staticmethod
    def extrapolated(history, target, cap):
        """Seed for the next march sample: secant in alpha of attitude and load
        from the last two admitted samples (internal coordinates from the last)."""
        low = history[-1]
        if len(history) < 2:return low
        prev = history[-2];h = low['alpha_deg']-prev['alpha_deg']
        if h <= 1e-9:return low
        r = (target-low['alpha_deg'])/h
        x = np.asarray(low['solution'])+r*(np.asarray(low['solution'])-np.asarray(prev['solution']))
        x[0] = target
        n = float(np.clip(low['load_g']+r*(low['load_g']-prev['load_g']), 1.001, cap))
        return dict(low, solution=x.tolist(), load_g=n)

    # ----- step control ----------------------------------------------------
    def margin_root(self, history, kind):
        """Predicted zero of one margin ahead of the last sample: the smaller of
        the linear (last two samples) and quadratic (last three) extrapolations,
        so an accelerating margin (pitch response near its reversal) is not
        stepped over."""
        pts = [(h['alpha_deg'], margins(self.solver, h).get(kind)) for h in history[-3:]]
        pts = [(x, y) for x, y in pts if y is not None]
        a = history[-1]['alpha_deg'];roots = []
        if len(pts) >= 2:
            (x0, y0), (x1, y1) = pts[-2], pts[-1]
            if y1 < y0 and x1 > x0:roots.append(x1+y1*(x1-x0)/(y0-y1))
        if len(pts) == 3:
            try:
                c = np.polyfit([x for x, _ in pts], [y for _, y in pts], 2)
                roots += [r.real for r in np.roots(c) if abs(r.imag) < 1e-12 and r.real > a]
            except (np.linalg.LinAlgError, ValueError):pass
        roots = [r for r in roots if r > a]
        return min(roots) if roots else None

    def next_alpha(self, history, step, predicted):
        low = history[-1];a = low['alpha_deg']
        target = a+step
        if predicted is not None and predicted > a+.05:target = min(target, predicted+max(OVERSHOOT_DEG, .02*(predicted-a)))
        if len(history) >= 2:
            for k in margins(self.solver, low):
                root = self.margin_root(history, k)
                if root is None:continue
                if root-a > APPROACH_DEG:
                    # Approach a distant predicted crossing in steps, so the
                    # margin's curvature is resampled before it is crossed.
                    target = min(target, a+.75*(root-a))
                else:target = min(target, root+max(OVERSHOOT_DEG, .03*(root-a)))
        if len(history) >= 3:
            # Parabolic load maximum ahead of the last sample.
            xs = np.array([h['alpha_deg'] for h in history[-3:]]);ys = np.array([h['load_g'] for h in history[-3:]])
            try:c2, c1, _ = np.polyfit(xs, ys, 2)
            except (np.linalg.LinAlgError, ValueError):c2 = 0.
            if c2 < 0.:
                vertex = -c1/(2*c2)
                if vertex > a:target = min(target, vertex+max(OVERSHOOT_DEG, .25*(vertex-a)))
        if a < SWITCH_DEG-1e-3 < target:
            from em_branch_limit import ROLL_LEVELING_PROBE_OFFSET
            target = SWITCH_DEG-ROLL_LEVELING_PROBE_OFFSET
        return min(target, self.alpha_upper)

    # ----- localization ----------------------------------------------------
    def closed(self, lo, hi):
        # The chart quantity is the turn rate at the boundary (envelope
        # tolerance 0.15 deg/s); localization closes at a third of it.
        return abs(turn_rate(self.speed, hi['load_g'])-turn_rate(self.speed, lo['load_g'])) <= self.turn_tol

    def margin_edge(self, lo, hi):
        """Illinois in alpha on the smallest margin between an admitted and a
        balanced physically rejected state; returns the admitted side."""
        from em_stationary_boundary import mark
        flo, fhi = combined(self.solver, lo), combined(self.solver, hi)
        side = 0
        for it in range(16):
            if self.closed(lo, hi):break
            w = hi['alpha_deg']-lo['alpha_deg']
            if w <= 1e-6:break
            if flo is not None and fhi is not None and flo > 0. > fhi:
                t = flo/(flo-fhi)
            else:t = .5
            t = min(.9, max(.1, t)) if it % 4 != 3 else .5
            # Straddle the estimated crossing by just under half the closing
            # width (turn slope from the bracket), alternating sides, so an
            # accurate estimate closes the bracket in two corrections.
            slope = abs(hi['turn_dps']-lo['turn_dps'])/w
            if it % 4 != 3 and slope > 0.:
                d = .4*self.turn_tol/slope/w
                t = t-d if side >= 0 else t+d
                t = min(.95, max(.05, t))
            # Always continued from the admitted side, so a rejected trial is on
            # the marched branch (seeding from a rejected state can stay on another root).
            p = self.at(lo, lo['alpha_deg']+t*w, 'localize')
            if p is None or (not p['valid'] and not physical(p)):
                p = self.at(lo, lo['alpha_deg']+.5*w, 'localize bisect')
                if p is None or (not p['valid'] and not physical(p)):
                    self.last_lo = lo
                    return None, 'localization correction unresolved'
            if p['valid']:
                lo, flo = p, combined(self.solver, p)
                if side == 1 and fhi is not None:fhi *= .5
                side = 1
                if hi['alpha_deg']-lo['alpha_deg'] <= 1e-3*max(1., w):
                    # Admitted next to the rejected end: that rejection was
                    # another root, not the end of this branch.
                    self.last_lo = lo
                    return None, 'false bracket'
            else:
                hi, fhi = p, combined(self.solver, p)
                if side == -1 and flo is not None:flo *= .5
                side = -1
        else:
            self.last_lo = lo
            return None, 'localization budget'
        if not self.closed(lo, hi):
            self.last_lo = lo
            return None, 'localization did not close'
        kind = limit_kind(self.solver, hi, lo)
        edge = mark(lo, kind, axis=control_axis(lo) if kind == 'control' else None,
                    method='S1 alpha-march margin bracket', alpha_bracket_deg=[lo['alpha_deg'], hi['alpha_deg']],
                    limiting_load_interval_g=[lo['load_g'], hi['load_g']],
                    turn_interval_dps=[lo['turn_dps'], hi['turn_dps']],
                    rejected_endpoint={k: hi[k] for k in ('load_g', 'alpha_deg', 'reasons', 'force_error_g', 'angular_error_rad_s2')})
        return edge, None

    def fold_edge(self, points):
        """Load maximum in alpha from admitted samples around it (parabolic steps)."""
        from em_stationary_boundary import mark
        pts = sorted({p['alpha_deg']: p for p in points}.values(), key=lambda p: p['alpha_deg'])
        for it in range(14):
            i = max(range(len(pts)), key=lambda j: pts[j]['load_g'])
            if i == 0 or i == len(pts)-1:return None, 'fold maximum not bracketed', None
            a, b, c = pts[i-1], pts[i], pts[i+1]
            tb = b['turn_dps']
            if (tb-a['turn_dps'] <= self.turn_tol and tb-c['turn_dps'] <= self.turn_tol and
                    abs(a['ps_mps']-b['ps_mps']) <= self.solver.config['sep_tolerance_mps'] and
                    abs(c['ps_mps']-b['ps_mps']) <= self.solver.config['sep_tolerance_mps']):
                return mark(b, 'trim fold', method='S1 alpha-march load maximum',
                            alpha_bracket_deg=[a['alpha_deg'], c['alpha_deg']],
                            neighboring_loads_g=[a['load_g'], c['load_g']]), None, None
            xs = np.array([a['alpha_deg'], b['alpha_deg'], c['alpha_deg']]);ys = np.array([a['load_g'], b['load_g'], c['load_g']])
            c2, c1, _ = np.polyfit(xs, ys, 2)
            v = -c1/(2*c2) if c2 < 0. else .5*(xs[0]+xs[2])
            lo_a, hi_a = xs[0]+.1*(xs[2]-xs[0]), xs[2]-.1*(xs[2]-xs[0])
            if it % 3 == 2 or not lo_a < v < hi_a or abs(v-xs[1]) < 1e-4:
                # Bisect the wider side of the current maximum.
                v = .5*(xs[0]+xs[1]) if xs[1]-xs[0] > xs[2]-xs[1] else .5*(xs[1]+xs[2])
            p = self.at(b, v, 'fold')
            if p is None:return None, 'fold correction unresolved', None
            if not p['valid']:
                # A physical rejection inside the fold bracket: that limit comes first
                # on the low side, otherwise the face is after the maximum.
                if physical(p) and v < xs[1]:return None, 'rejection before load maximum', (a, p)
                return None, 'fold bracket inconsistent' if physical(p) else 'fold correction unresolved', None
            pts.append(p);pts.sort(key=lambda q: q['alpha_deg'])
        return None, 'fold budget', None

    # ----- the march -------------------------------------------------------
    def seed_for(self, history, target, cap):
        """Secant extrapolation from the march, or a hint state (e.g. a mapped
        reduced-model sample) when it is nearer in alpha. Hints are Newton
        starting points only."""
        seed = self.extrapolated(history, target, cap)
        if self.hints:
            hint = min(self.hints, key=lambda h: abs(h['alpha_deg']-target))
            if abs(hint['alpha_deg']-target) < abs(history[-1]['alpha_deg']-target):
                x = list(hint['solution']);x[0] = float(target)
                seed = dict(hint, solution=x)
        return seed

    def run(self, level, predicted=None, history=None):
        """Return (boundary, status, reason); boundary has envelope_limit when verified.

        ``history``: admitted states at this speed with increasing alpha that
        the march continues from (default: level flight only)."""
        from em_load_limits import ceiling
        history = list(history) if history else [level]
        for h in history:
            if all(h is not q for q in self.samples):self.samples.append(h)
        span = (predicted-level['alpha_deg']) if predicted is not None else None
        step = self.max_step if span is None or span <= 0 else span/max(1, math.ceil(span/self.max_step))
        cap = ceiling(self.solver, self.speed)
        tried_active = None
        for _ in range(60):
            low = history[-1]
            target = self.next_alpha(history, step, predicted)
            if target <= low['alpha_deg']+1e-6:return None, 'unresolved numerical boundary', 'alpha search bound reached'
            if self.store is not None:target = self.snap(low, target)
            seed = self.seed_for(history, target, cap)
            p = self.at(seed, target, 'march', base_alpha=low['alpha_deg'])
            if p is not None and not p['valid'] and physical(p) and (seed is not low or self.trials[-1].get('warm')):
                # A rejection is evidence only on the marched branch: the
                # extrapolated or neighbouring-speed seed can converge onto
                # another root at this alpha. Confirm from the last sample.
                q = self.at(low, target, 'march confirm')
                if q is not None and q['valid']:
                    # The two seeds disagree: neither is continuity evidence
                    # over this step. Approach with half the step instead.
                    step = .5*(target-low['alpha_deg']);predicted = None
                    if step < .05:return None, 'unresolved numerical boundary', 'march roots disagree'
                    continue
                if q is not None and physical(q):p = q
            if p is None or (not p['valid'] and not physical(p)):
                if low is not tried_active:
                    tried_active = low
                    edge = self.hard_face(history, target)
                    if edge is not None:return self.finish_face(history, edge)
                if low['load_g'] >= cap-1e-6 or (p is not None and 'chart load search limit' in p.get('reasons', ())):
                    q = self.curve.correct(low, load=cap)
                    if q is not None and q['valid'] and q['alpha_deg'] >= low['alpha_deg']-1e-6:
                        self.samples.append(q)
                        return q, 'plot ceiling', None
                step = .5*(target-low['alpha_deg'])
                if step < .05:return None, 'unresolved numerical boundary', 'march correction unresolved'
                predicted = None
                continue
            if not p['valid']:
                self.last_lo = None
                edge, why = self.margin_edge(low, p)
                if edge is None and why == 'false bracket':
                    history.append(self.last_lo);predicted = None
                    continue
                if edge is None:
                    # The bracket did not close (e.g. the prescribed-alpha
                    # equations do not close next to a pitch-response
                    # reversal): solve the closing face from the last admitted
                    # localization state.
                    lo = self.last_lo if self.last_lo is not None and self.last_lo['alpha_deg'] > low['alpha_deg'] else None
                    edge = self.hard_face(history+([lo] if lo is not None else []), p['alpha_deg'])
                    if edge is None:return None, 'unresolved numerical boundary', why
                    if lo is not None:history = history+[lo]
                return self.finish_face(history, edge)
            if p['load_g'] < low['load_g']:
                if len(history) < 2:
                    # Decreasing from level flight: refine below this sample first.
                    q = self.at(low, .5*(low['alpha_deg']+p['alpha_deg']), 'march')
                    if q is None or not q['valid'] or q['load_g'] <= low['load_g']:
                        return None, 'unresolved numerical boundary', 'load decreases from level flight'
                    history.append(q)
                return self.finish_fold(history+[p], None)
            history.append(p)
            step = min(self.max_step, max(step, p['alpha_deg']-low['alpha_deg'])*1.5)
        return None, 'unresolved numerical boundary', 'march budget'

    def finish_face(self, history, edge):
        # A load maximum inside the last interval hides behind the face.
        top = max(history, key=lambda h: h['load_g'])
        if top['load_g'] > edge['load_g']+1e-9:return self.finish_fold(history, edge)
        return edge, 'verified limit', None

    def crossing(self, history, kind):
        return self.margin_root(history, kind) if len(history) >= 2 else None

    def hard_face(self, history, target):
        """An unclosed trial next to a face: solve that face directly.

        Only faces whose margin is closing along the march are tried (linear
        extrapolation of the margin crosses zero before the failed trial, or
        the margin is already nearly zero). Most balanced rejections close just
        past a limit, but some faces make the prescribed-alpha equations
        singular or saturated there: the pitch-response reversal (production
        follows elevator command instead: em_pitch_limit.limit), hard control
        or propeller-pitch stops, and post-stall alpha with no equilibrium.
        Those use the production active-event equations (Curve.correct(event=))
        from the last admitted sample, as follow_branch does."""
        low = history[-1];m = margins(self.solver, low);m0 = margins(self.solver, history[0])
        closing = []
        for kind in m:
            root = self.crossing(history, kind)
            small = m[kind] <= .02*max(abs(m0.get(kind, 1.)), 1e-6)
            if (root is not None and root < target+.5) or small:closing.append((root if root is not None else low['alpha_deg'], kind))
        for _, kind in sorted(closing):
            if kind == 'pitch response':
                if m0.get(kind, 0.) > 0. and m[kind] < .7*m0[kind]:
                    edge = self.pitch_face(low)
                    if edge is not None:return edge
                continue
            edge = self.active_face(low, target, kinds=(kind,))
            if edge is not None:return edge
        if 'propeller pitch' in self.hard_kinds(low):
            return self.active_face(low, target, kinds=('propeller pitch',))
        return None

    def hard_kinds(self, low):
        return {e[0] for e in self.curve.nearest_events(low)[:3]}

    def pitch_face(self, low):
        from em_trim_work import Work, use, current
        from em_pitch_limit import limit
        w = Work({}, parent=current())
        with use(w):
            try:p = limit(self.solver, self.speed, low)
            except Exception as exc:p = None
        self.trials.append(dict(why='pitch face', ok=None if p is None else bool(p['valid']), evals=w.counts.get('aircraft_evaluations', 0),
                                fresh=w.counts.get('derivative_samples', 0), alpha=None if p is None else round(p['alpha_deg'], 4)))
        if (p is None or not p.get('valid') or not p.get('envelope_limit') or p['alpha_deg'] < low['alpha_deg']-1e-4
                or p['load_g'] < low['load_g']-1e-3):return None
        self.samples.append(p)
        return p

    def active_face(self, low, target, kinds=HARD_FACES):
        """Hard bounds (control stops, propeller pitch stops) can prevent a
        balanced rejected trial: solve the nearest such face directly with the
        production active-event equations (as follow_branch does)."""
        from em_stationary_boundary import mark, event_details
        from em_trim_work import Work, use, current
        events = [e for e in self.curve.nearest_events(low) if e[0] in kinds][:1]
        for kind, axis, event in events:
            w = Work({}, parent=current())
            with use(w):p = self.curve.correct(low, event=event, max_nfev=16)
            self.trials.append(dict(why='active '+kind, axis=axis, ok=None if p is None else bool(p['valid']),
                                    evals=w.counts.get('aircraft_evaluations', 0), fresh=w.counts.get('derivative_samples', 0),
                                    alpha=None if p is None else round(p['alpha_deg'], 4)))
            if p is None or not p['valid']:continue
            if low['alpha_deg']-1e-4 <= p['alpha_deg'] <= target+1e-9 and p['load_g'] >= low['load_g']-1e-3:
                self.samples.append(p)
                return mark(p, kind, axis=axis, method='S1 alpha-march active face', **event_details(event))
        return None

    def finish_fold(self, history, face):
        edge, why, bracket = self.fold_edge(history if face is None else history+[face])
        if edge is not None:return edge, 'verified limit', None
        if bracket is not None:
            e, w = self.margin_edge(*bracket)
            if e is not None:return e, 'verified limit', None
            why = w
        return None, 'unresolved numerical boundary', why


def predicted_alpha(speed, seeds):
    """Boundary alpha at ``speed`` from the nearest verified neighbouring boundaries."""
    anchors = {}
    for p in seeds or ():
        if p.get('valid') and p.get('envelope_limit') and p['speed_kmh'] != speed:
            anchors.setdefault(p['speed_kmh'], p)
    near = sorted(anchors.values(), key=lambda p: abs(p['speed_kmh']-speed))[:2]
    if not near:return None
    if len(near) == 1:return near[0]['alpha_deg']
    a, b = near
    t = (speed-a['speed_kmh'])/(b['speed_kmh']-a['speed_kmh'])
    value = a['alpha_deg']+t*(b['alpha_deg']-a['alpha_deg'])
    lo, hi = sorted((a['alpha_deg'], b['alpha_deg']))
    return float(np.clip(value, lo-3., hi+3.))


def column_record(solver, speed, level, march, boundary, status, reason, started):
    keep = [p for p in march.samples if p['valid'] and abs(p['speed_kmh']-speed) <= 1e-9]
    if boundary is not None:
        keep = [p for p in keep if p['alpha_deg'] <= boundary['alpha_deg']+1e-9 and p is not boundary
                and abs(p['load_g']-boundary['load_g']) > 1e-12]+[boundary]
    points = sorted({id(p): p for p in keep}.values(), key=lambda p: p['load_g'])
    for p in points:p.setdefault('surface_sample', False)
    col = dict(speed_kmh=float(speed), points=points, boundary=boundary, lower_boundary=level,
               boundary_status=status, boundary_reason=(boundary or {}).get('envelope_limit', {}).get('kind') if status == 'verified limit'
               else reason if boundary is None else None,
               boundary_bracket_g=None, sustained=[], load_checks=[], numerical_gap_brackets=[], interior_failures=[],
               unresolved_load_intervals=[], mass=solver.mass, engine=solver.engine.summary, boundary_probe=True,
               lower_boundary_searched=False, elapsed_s=time.monotonic()-started,
               s1_march=dict(trials=len(march.trials), reason=reason,
                             trial_log=[[t.get('why'), t.get('alpha'), t.get('ok'), t.get('evals'), t.get('fresh'), t.get('warm')] for t in march.trials],
                             samples=[[round(p['alpha_deg'], 4), round(p['load_g'], 5), round(p['ps_mps'], 3)] for p in points]))
    col['s3_path'] = getattr(march, 's3_path', None) or ('march' if boundary is not None else 'none')
    if getattr(march, 's3_flags', None):col['s3_flags'] = sorted(march.s3_flags)
    RESCUE_STATS['s3_paths'][col['s3_path']] = RESCUE_STATS['s3_paths'].get(col['s3_path'], 0)+1
    return col


def march_worker(task, *, max_step=MAX_STEP_DEG, turn_tol=TURN_TOLERANCE, predicted=None, level_seed=None, reuse=None,
                 fallback=True):
    """Column worker (run through em_sampling.sample_with_history)."""
    import em_sampling as S
    from em_speed_limits import excluded_column
    from em_aircraft_region import annotate
    started = time.monotonic()
    name, cj, speed, seeds = task
    solver = S.worker_solver(name, cj)
    excluded = excluded_column(solver, speed)
    if excluded is not None:return excluded
    STATS['columns'] += 1
    seeds = [p for p in (seeds or ()) if p.get('valid') and p.get('stationary_coordinates') is not None]
    levels = [p for p in seeds if abs(p['load_g']-1.) <= 1e-12]
    if level_seed is None and levels:
        level_seed = S.continuation_initial(solver, min(levels, key=lambda p: abs(p['speed_kmh']-speed)))
    from em_trim_work import Work, use, current
    march = RescueMarch(solver, speed, max_step=max_step, turn_tol=turn_tol, model=cj, reuse=REUSE if reuse is None else reuse)
    level = None
    nearest_level = min(levels, key=lambda p: abs(p['speed_kmh']-speed)) if levels else None
    if nearest_level is not None and march.store is not None:
        level = march.level_from(nearest_level)
    if level is None:
        if solver.is_prop and hasattr(solver.engine, 'reset_search'):solver.engine.reset_search()
        w = Work({}, parent=current())
        with use(w):level = S.solve_level(solver, speed, level_seed)
        march.trials.append(dict(why='level', evals=w.counts.get('aircraft_evaluations', 0), ok=bool(level['valid']),
                                 reasons=list(level['reasons'])))
        if (not level['valid'] and not physical(level) and nearest_level is not None and
                speed > nearest_level['speed_kmh']):
            # An unclosed level-flight solve above an admitted neighbouring
            # speed (not the low-speed edge) is a numerical failure: one
            # unseeded production retry before the production column search.
            w = Work({}, parent=current())
            with use(w):retry = S.solve_level(solver, speed, None)
            march.trials.append(dict(why='level retry', evals=w.counts.get('aircraft_evaluations', 0), ok=bool(retry['valid']),
                                     reasons=list(retry['reasons'])))
            if retry['valid'] or physical(retry):level = retry
    # A level-flight failure above the neighbouring admitted speed is
    # numerical (high-speed propulsion), below it the low-speed edge.
    healthy = nearest_level is not None and speed > nearest_level['speed_kmh']
    if not level['valid']:
        if not physical(level) and healthy and fallback:
            return production_column(task, 'level flight unresolved: '+', '.join(level['reasons']))
        # As the production column: no admitted sample at this speed is
        # 'no feasible samples' (an explicit edge for the sweep, never an
        # absence proof).
        status = 'no feasible samples'
        col = column_record(solver, speed, None, march, None, status, 'level flight: '+', '.join(level['reasons']), started)
        col['points'] = [level]
        return col
    if predicted is None:predicted = predicted_alpha(speed, seeds)
    boundary, status, reason = march.run(level, predicted)
    if boundary is None and fallback:
        return production_column(task, 'march unresolved: '+str(reason))
    col = column_record(solver, speed, level, march, boundary, status, reason, started)
    annotate(solver, col)
    return col


def production_column(task, why):
    """Where the march is numerically unresolved, the production boundary
    column search at that speed (same model, same seeds) replaces it."""
    import em_sampling as S
    STATS['fallback'] += 1;STATS['reasons'][why[:40]] = STATS['reasons'].get(why[:40], 0)+1
    col = S._sample_checked_boundary_column(task)
    col['s1_fallback'] = why
    col['s3_path'] = 'production fallback'
    RESCUE_STATS['s3_paths']['production fallback'] = RESCUE_STATS['s3_paths'].get('production fallback', 0)+1
    reasons = RESCUE_STATS.setdefault('fallback_reasons', {});reasons[why[:60]] = reasons.get(why[:60], 0)+1
    RESCUE_STATS.setdefault('fallback_speeds', []).append(round(float(task[2]), 3))
    return col


def run_column(name, cj, speed, seeds, **kwargs):
    """One march column with production solver preparation and annotation."""
    import em_sampling as S
    worker = (lambda task: march_worker(task, **kwargs)) if kwargs else march_worker
    col, _ = S.sample_with_history(worker, (name, cj, float(speed), list(seeds or [])), {})
    return col




# ---- rescue where the march gives up (S3) ----
RESCUE_REASONS = ('march correction unresolved', 'march roots disagree')
RESCUE_STATS = dict(s3_face=0, s3_bisect=0, s3_unresolved=0, s3_paths={}, bisect_trials=0, face_trials=0, behind_face=0,
             fold_within_tolerance=0)


def parabola(pts):
    """(c2, c1, c0) of the parabola through three points (as np.polyfit(x, y, 2)), or None."""
    (x0, y0), (x1, y1), (x2, y2) = pts
    if x0 == x1 or x1 == x2 or x0 == x2:return None
    f01 = (y1-y0)/(x1-x0);f12 = (y2-y1)/(x2-x1)
    c2 = (f12-f01)/(x2-x0)
    c1 = f01-c2*(x0+x1)
    c0 = y0-x0*(f01-c2*x1)
    return c2, c1, c0


def quadratic_roots(c2, c1, c0):
    """Real roots as np.roots([c2, c1, c0]) with |imag| < 1e-12 counted as real."""
    if c2 == 0.:
        return [-c0/c1] if c1 != 0. else []
    disc = c1*c1-4.*c2*c0
    if disc >= 0.:
        sq = math.sqrt(disc)
        q = -.5*(c1+math.copysign(sq, c1))
        if q == 0.:return [0., 0.]
        return [q/c2, c0/q]
    if math.sqrt(-disc)/(2.*abs(c2)) < 1e-12:
        r = -c1/(2.*c2)
        return [r, r]
    return []


class RescueMarch(March):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.s3_path = None
        self.s3_flags = set()
        self._margins = {}
        self._neighbours = None

    # ----- bookkeeping -----------------------------------------------------
    def margins_of(self, p):
        hit = self._margins.get(id(p))
        if hit is not None and hit[0] is p:return hit[1]
        m = margins(self.solver, p)
        self._margins[id(p)] = (p, m)
        return m

    def neighbours(self):
        if self.store is None:return ()
        rows = self.store.get('rows', ())
        first = rows[0] if rows else None
        # Rows of this speed are excluded and no other column runs during this
        # one, so the filtered rows only change when the store is trimmed
        # (oldest rows deleted, so the first row changes).
        if self._neighbours is not None and self._neighbours[0] is first:return self._neighbours[1]
        out = [r for r in rows if r[0] != self.speed and abs(r[0]-self.speed) <= REUSE_WINDOW_KMH]
        self._neighbours = (first, out)
        return out

    def margin_root(self, history, kind):
        pts = [(h['alpha_deg'], self.margins_of(h).get(kind)) for h in history[-3:]]
        pts = [(x, y) for x, y in pts if y is not None]
        a = history[-1]['alpha_deg'];roots = []
        if len(pts) >= 2:
            (x0, y0), (x1, y1) = pts[-2], pts[-1]
            if y1 < y0 and x1 > x0:roots.append(x1+y1*(x1-x0)/(y0-y1))
        if len(pts) == 3:
            c = parabola(pts)
            if c is None:
                try:
                    c = np.polyfit([x for x, _ in pts], [y for _, y in pts], 2)
                    roots += [r.real for r in np.roots(c) if abs(r.imag) < 1e-12 and r.real > a]
                except (np.linalg.LinAlgError, ValueError):pass
            else:roots += [r for r in quadratic_roots(*c) if r > a]
        roots = [r for r in roots if r > a]
        return min(roots) if roots else None

    def next_alpha(self, history, step, predicted):
        low = history[-1];a = low['alpha_deg']
        target = a+step
        if predicted is not None and predicted > a+.05:target = min(target, predicted+max(OVERSHOOT_DEG, .02*(predicted-a)))
        if len(history) >= 2:
            for k in self.margins_of(low):
                root = self.margin_root(history, k)
                if root is None:continue
                if root-a > APPROACH_DEG:target = min(target, a+.75*(root-a))
                else:target = min(target, root+max(OVERSHOOT_DEG, .03*(root-a)))
        if len(history) >= 3:
            pts = [(h['alpha_deg'], h['load_g']) for h in history[-3:]]
            c = parabola(pts)
            if c is None:
                try:c2, c1, _ = np.polyfit([x for x, _ in pts], [y for _, y in pts], 2)
                except (np.linalg.LinAlgError, ValueError):c2 = 0.
            else:c2, c1, _ = c
            if c2 < 0.:
                vertex = -c1/(2*c2)
                if vertex > a:target = min(target, vertex+max(OVERSHOOT_DEG, .25*(vertex-a)))
        if a < SWITCH_DEG-1e-3 < target:
            from em_branch_limit import ROLL_LEVELING_PROBE_OFFSET
            target = SWITCH_DEG-ROLL_LEVELING_PROBE_OFFSET
        return min(target, self.alpha_upper)

    # ----- rescue of an unresolved march -----------------------------------
    def run(self, level, predicted=None, history=None):
        out = super().run(level, predicted, history)
        if out[0] is not None or out[2] not in RESCUE_REASONS:return out
        try:edge = self.rescue(level)
        except (ValueError, ArithmeticError, RuntimeError, np.linalg.LinAlgError) as error:
            # Never fatal: the column goes to the production column search as in S1/S2.
            errors = RESCUE_STATS.setdefault('rescue_errors', {});key = type(error).__name__+': '+str(error)[:60]
            errors[key] = errors.get(key, 0)+1;edge = None
        if edge is None:
            RESCUE_STATS['s3_unresolved'] += 1
            return out
        return edge

    def rescue(self, level):
        """Where S1 gives up ('march correction unresolved': trials above the last admitted
        sample do not close, step below 0.05 deg): nearest faces, then bisection, from the
        highest admitted sample at this speed (every sample was continued from the admitted side)."""
        admitted = sorted((p for p in self.samples if p.get('valid') and abs(p['speed_kmh']-self.speed) <= 1e-9
                           and p['alpha_deg'] >= level['alpha_deg']-1e-9), key=lambda p: p['alpha_deg'])
        if not admitted:return None
        low = admitted[-1]
        failed = [t['alpha'] for t in self.trials if t.get('ok') is None and t.get('alpha') is not None
                  and t['why'] in ('march', 'march confirm') and t['alpha'] > low['alpha_deg']]
        if not failed:return None
        hi = min(failed)
        edge = self.nearest_faces(low, hi)
        if edge is not None:
            self.s3_path = 's3 face';RESCUE_STATS['s3_face'] += 1
            return self.finish_face(admitted, edge)
        edge, lows = self.bisect(admitted, hi)
        if edge is not None:
            self.s3_path = 's3 bisect';RESCUE_STATS['s3_bisect'] += 1
            return self.finish_face(admitted+lows, edge)
        return None

    def nearest_faces(self, low, target, tried=(), count=4):
        """The nearest physical faces at ``low`` (production order), each solved once."""
        seen = set()
        for rank, (kind, axis, event) in enumerate(self.curve.nearest_events(low)[:count]):
            if (kind, axis) in seen or (kind, axis) in tried:continue
            seen.add((kind, axis))
            RESCUE_STATS['face_trials'] += 1
            if kind == 'pitch response':
                # em_pitch_limit costs hundreds of evaluations on propeller aircraft: only
                # when the pitch response is one of the two nearest faces.
                if low.get('pitch_response') is None or rank > 1:continue
                edge = self.pitch_face(low)
            else:
                edge = self.event_face(low, target, kind, axis, event)
            if edge is not None:return edge
        return None

    def accepted_face(self, low, target, p):
        """S1's active-face guard (alpha within [last admitted, failed trial], load not below
        the last admitted sample), plus: a face state at most 0.5 deg behind the last admitted
        sample is accepted when its turn rate is within the march tolerance of that sample.
        That is an admitted sample already on (or inside the inset of) the stop, as production
        Curve.active treats a state exactly on a control stop."""
        if p is None or not p['valid'] or p['load_g'] < low['load_g']-1e-3 or p['alpha_deg'] > target+1e-9:return False
        if p['alpha_deg'] >= low['alpha_deg']-1e-4:return True
        if p['alpha_deg'] >= low['alpha_deg']-.5 and abs(p['turn_dps']-low['turn_dps']) <= self.turn_tol:
            RESCUE_STATS['behind_face'] += 1;self.s3_flags.add('face behind last sample')
            return True
        return False

    def active_face(self, low, target, kinds=HARD_FACES):
        """S1 active_face with accepted_face as the guard."""
        events = [e for e in self.curve.nearest_events(low) if e[0] in kinds][:1]
        for kind, axis, event in events:
            edge = self.event_face(low, target, kind, axis, event, why='active ')
            if edge is not None:return edge
        return None

    def event_face(self, low, target, kind, axis, event, why='s3 active '):
        from em_stationary_boundary import mark, event_details
        from em_trim_work import Work, use, current
        w = Work({}, parent=current())
        with use(w):p = self.curve.correct(low, event=event, max_nfev=16)
        self.trials.append(dict(why=why+kind, axis=axis, ok=None if p is None else bool(p['valid']),
                                evals=w.counts.get('aircraft_evaluations', 0), fresh=w.counts.get('derivative_samples', 0),
                                alpha=None if p is None else round(p['alpha_deg'], 4)))
        if not self.accepted_face(low, target, p):return None
        self.samples.append(p)
        return mark(p, kind, axis=axis, method='S3 alpha-march active face' if why.startswith('s3') else 'S1 alpha-march active face',
                    **event_details(event))

    def finish_face(self, history, edge):
        """S1 finish_face; a load maximum less than the march tolerance (in turn rate) above
        the face is not a separate fold (production Curve.active: solving the inset event
        from a state already on the stop manufactures a tiny apparent maximum)."""
        top = max(history, key=lambda h: h['load_g'])
        if top['load_g'] > edge['load_g']+1e-9:
            if top['turn_dps']-edge['turn_dps'] > self.turn_tol:return self.finish_fold(history, edge)
            RESCUE_STATS['fold_within_tolerance'] += 1;self.s3_flags.add('load maximum within tolerance of face')
        return edge, 'verified limit', None

    def turn_slope(self, points):
        """Largest secant d(turn)/d(alpha) over consecutive admitted samples (deg/s per deg), or None."""
        pts = sorted(points, key=lambda p: p['alpha_deg'])
        best = None
        for a, b in zip(pts, pts[1:]):
            h = b['alpha_deg']-a['alpha_deg']
            if h > 1e-6:
                s = abs(b['turn_dps']-a['turn_dps'])/h
                best = s if best is None else max(best, s)
        return best

    def bisect(self, history, target, budget=8):
        """Alpha bisection of [last admitted, failed trial]; returns (edge, new admitted samples)."""
        lo = history[-1];hi = float(target);lows = []
        for _ in range(budget):
            width = hi-lo['alpha_deg']
            if width <= 1e-3:break
            slope = self.turn_slope(history+lows)
            if slope is not None and slope*width <= self.turn_tol:break
            RESCUE_STATS['bisect_trials'] += 1
            p = self.at(lo, lo['alpha_deg']+.5*width, 's3 bisect')
            if p is not None and p['valid']:
                lo = p;lows.append(p)
            elif p is not None and physical(p):
                self.last_lo = None
                edge, why = self.margin_edge(lo, p)
                if edge is not None:
                    if self.last_lo is not None:lows.append(self.last_lo)
                    return edge, lows
                if why == 'false bracket' and self.last_lo is not None:
                    lo = self.last_lo;lows.append(lo);continue
                break
            else:hi = lo['alpha_deg']+.5*width
        if lo is history[-1]:
            return None, lows
        return self.nearest_faces(lo, hi), lows


# ---- envelope tracer (speed placement) ----
class MarchTracer:
    """Direct envelope tracing across speed with alpha-march columns.

    The upper boundary is followed speed by speed (research/output_solver_v1
    tracer, unchanged speed placement):
    - segments are cut at native Mach seams and the enforced speed limit;
    - the step adapts to the predictor's turn error (linear extrapolation in load);
    - a change of limiting face between neighbours is localized to the chart
      speed tolerance (a corner) and the sweep continues on the new face;
    - loss of the boundary (no feasible state: below the stall speed, or an
      unresolved search) is localized to the speed tolerance and ends the sweep in
      that direction, recorded as an explicit edge (never as an absence proof);
    - an unresolved column is dropped and recorded as an 'unresolved' event (the
      chart never draws a smooth outline across it);
    - afterwards every interval between neighbouring points on the same face gets a
      withheld midpoint boundary solve; failures are inserted and re-checked.
    ``on_column(tracer, phase, speed, why)`` (optional) is called before and after
    every column, for progress and cancellation; it may raise to stop the trace.
    """
    def __init__(self, aircraft, config, *, turn_tolerance=None, max_step_kmh=50., max_depth=12, on_column=None):
        import em_sampling as S
        from em_solver import aircraft_settings
        from em_accuracy import turn_tolerance as chart_turn, speed_tolerance
        self.S = S
        self.aircraft, self.config = aircraft, config
        self.cj = json.dumps(aircraft_settings(config, aircraft), sort_keys=True)
        self.ttol = chart_turn(config['sep_tolerance_mps']) if turn_tolerance is None else float(turn_tolerance)
        self.vtol = speed_tolerance(config)
        # Corner/edge placement tolerance scales with an approved looser turn
        # tolerance (same ratio); at the chart tolerance it is unchanged.
        self.place_tol = self.vtol*max(1., self.ttol/chart_turn(config['sep_tolerance_mps']))
        self.max_step = max_step_kmh
        self.max_depth = max_depth
        self.columns = {}
        self.witnesses = []
        self.events = []
        self.work = {}
        self.on_column = on_column
        # March localization closes at a third of the envelope tolerance
        # (0.05 deg/s at the 0.15 deg/s speed-first envelope tolerance).
        self.march_tol = TURN_TOLERANCE*(self.ttol/.15)

    # ----- primitives --------------------------------------------------------
    def column(self, speed, neighbours, why, keep=True):
        """One alpha-march column (production column record shape)."""
        from em_trim_work import Work, use, current
        if self.on_column is not None:self.on_column(self, 'start', speed, why)
        seed = [p for c in neighbours if c is not None for p in c['points'] if p.get('valid')]
        w = Work({}, parent=current())
        with use(w):col = run_column(self.aircraft, self.cj, speed, seed, turn_tol=self.march_tol)
        if col.get('lower_boundary') is None and col.get('points'):
            col['lower_boundary'] = next((p for p in col['points'] if abs(p['load_g']-1.) <= 1e-9 and p.get('valid')), None)
        col['s1_role'] = why;col['s1_evaluations'] = w.counts.get('aircraft_evaluations', 0)
        row = self.work.setdefault(why, dict(columns=0, evaluations=0))
        row['columns'] += 1;row['evaluations'] += w.counts.get('aircraft_evaluations', 0)
        if keep:self.columns[float(speed)] = col
        if self.on_column is not None:self.on_column(self, 'done', speed, why)
        return col

    @staticmethod
    def ok(col):
        return col is not None and col.get('boundary_status') in ('verified limit', 'plot ceiling') and bool(col.get('boundary'))

    @staticmethod
    def kind(col):
        return col.get('boundary_reason')

    # ----- segments ----------------------------------------------------------
    def segments(self):
        from em_speed_limits import speed_limits
        from em_mach_events import solver_regions
        solver = self.S.worker_solver(self.aircraft, self.cj)
        lo, hi = float(self.config['speed_min_kmh']), float(self.config['speed_max_kmh'])
        limit = speed_limits(solver.fm, solver.config)
        end = 'range_end'
        if limit['enforced'] and limit['sample_speed_kmh'] < hi:hi, end = limit['sample_speed_kmh'], 'speed_limit'
        cuts = sorted((a, b) for a, b in (r['speed_interval_kmh'] for r in solver_regions(solver)) if b > lo and a < hi)
        out, start, kind = [], lo, 'range_end'
        for a, b in cuts:
            if a > start:out.append(dict(speed_kmh=[start, a], ends=[kind, 'mach_seam']))
            start, kind = max(start, b), 'mach_seam'
        if hi > start:out.append(dict(speed_kmh=[start, hi], ends=[kind, end]))
        return out, limit, cuts

    # ----- sweep -------------------------------------------------------------
    def localize(self, good, bad, reason):
        """Bisect in speed between a verified column and a failed/different one."""
        while abs(bad['speed_kmh']-good['speed_kmh']) > self.place_tol:
            mid = .5*(good['speed_kmh']+bad['speed_kmh'])
            # Both neighbouring faces are nominated (the first admissible limit wins).
            col = self.column(mid, [good, bad] if reason == 'corner' else [good], reason)
            if self.ok(col) and (reason == 'edge' or self.kind(col) == self.kind(good)):good = col
            else:bad = col
        return good, bad

    def sweep(self, segment, start):
        lo, hi = segment['speed_kmh']
        for side in (-1., 1.):
            bound = lo if side < 0 else hi
            chain = [start];step = self.max_step;skipped = 0
            while abs(chain[-1]['speed_kmh']-bound) > 1e-9:
                last = chain[-1];prev = chain[-2] if len(chain) > 1 else None
                speed = last['speed_kmh']+side*min(step, abs(bound-last['speed_kmh']))
                col = self.column(speed, [last, prev], 'sweep')
                if not self.ok(col) and col.get('boundary_status') != 'no feasible samples' and                         abs(bound-speed) > 1e-9 and skipped < 3:
                    # An unresolved search is not an envelope edge: keep it as an
                    # explicit gap and try the next speed from the last good state.
                    self.events.append(dict(kind='unresolved', speed_kmh=speed, status=col.get('boundary_status')))
                    skipped += 1;step = abs(speed-last['speed_kmh'])*2.
                    self.columns.pop(float(speed), None)
                    continue
                skipped = 0 if self.ok(col) else skipped
                if not self.ok(col):
                    good, bad = self.localize(last, col, 'edge')
                    self.events.append(dict(kind='edge', speed_kmh=[good['speed_kmh'], bad['speed_kmh']],
                                            status=bad.get('boundary_status'), reason=bad.get('boundary_reason')))
                    break
                if self.kind(col) != self.kind(last):
                    # Face change: not localized here. If interpolation across
                    # it fails a withheld check, check() localizes the corner.
                    chain.append(col);step = max(self.max_step/4., 2*self.vtol)
                    continue
                if prev is not None and self.kind(prev) == self.kind(last):
                    r = (speed-last['speed_kmh'])/(last['speed_kmh']-prev['speed_kmh'])
                    n = last['boundary']['load_g']+r*(last['boundary']['load_g']-prev['boundary']['load_g'])
                    error = abs(turn_rate(speed, n)-col['boundary']['turn_dps'])
                    h = abs(speed-last['speed_kmh'])
                    if error > 8*self.ttol:step = max(.5*h, 2*self.vtol)
                    elif error < 2*self.ttol:step = min(self.max_step, 1.5*max(step, h))
                chain.append(col)

    def check(self):
        """Withheld midpoint boundary solves on every interval.

        The interpolant is PCHIP in limiting load over the neighbouring verified
        points within one segment. A failed check inserts the midpoint; when
        the interval's endpoints are on different faces the corner is localized
        instead (once per interval). Faces that change while the boundary stays
        smooth (e.g. stall / fold / pitch response near-coincident) pass.
        """
        from scipy.interpolate import PchipInterpolator
        checks, tried, passed_set = [], set(), set()
        seam = lambda a, b: any(a < x < b or a < y < b for x, y in self.seams)
        for depth in range(self.max_depth):
            good = [c for c in sorted(self.columns.values(), key=lambda c: c['speed_kmh']) if self.ok(c)]
            added = 0
            for i, (a, b) in enumerate(zip(good, good[1:])):
                va, vb = a['speed_kmh'], b['speed_kmh']
                key = (va, vb)
                if key in passed_set or key in tried or vb-va <= 2*self.vtol or seam(va, vb):continue
                if self.kind(a) != self.kind(b) and vb-va > 4*self.place_tol:
                    # A wide interval spanning a face change: localize the corner
                    # directly (a midpoint on one face cannot see the kink).
                    tried.add(key);added += 1
                    g, h = self.localize(a, b, 'corner')
                    self.events.append(dict(kind='corner', speed_kmh=[g['speed_kmh'], h['speed_kmh']], faces=[self.kind(g), self.kind(h)]))
                    continue
                window = [c for c in good if va-150 <= c['speed_kmh'] <= vb+150 and not seam(min(c['speed_kmh'], va), max(c['speed_kmh'], vb))]
                xs = [c['speed_kmh'] for c in window];ns = [c['boundary']['load_g'] for c in window]
                f = PchipInterpolator(xs, ns) if len(xs) > 1 else None
                v = .5*(va+vb)
                col = self.column(v, [a, b], 'check', keep=False)
                ok = self.ok(col) and f is not None
                if ok:
                    box = [turn_rate(x, float(f(x))) for x in np.clip([v-self.vtol, v, v+self.vtol], va, vb)]
                    ok = min(box)-self.ttol <= col['boundary']['turn_dps'] <= max(box)+self.ttol
                checks.append(dict(interval_kmh=[va, vb], speed_kmh=v, passed=bool(ok), faces=[self.kind(a), self.kind(b)],
                                   midpoint_face=self.kind(col) if self.ok(col) else col.get('boundary_status')))
                if ok:
                    passed_set.add(key);self.witnesses.append(col);continue
                added += 1
                if self.kind(a) != self.kind(b) and vb-va > self.place_tol:
                    tried.add(key)
                    g, h = self.localize(a, b, 'corner')
                    self.events.append(dict(kind='corner', speed_kmh=[g['speed_kmh'], h['speed_kmh']], faces=[self.kind(g), self.kind(h)]))
                else:
                    self.columns[v] = col
                    if not self.ok(col):tried.add(key)
            if not added:break
        return checks

    def run(self):
        started = time.perf_counter()
        segments, limit, cuts = self.segments()
        self.seams = cuts
        previous = None
        for segment in sorted(segments, key=lambda s: -s['speed_kmh'][1]):
            lo, hi = segment['speed_kmh']
            start = self.column(hi, [previous] if previous is not None else [], 'start')
            if not self.ok(start):
                start = self.column(.5*(lo+hi), [previous] if previous is not None else [], 'start')
            if self.ok(start):
                self.sweep(segment, start)
                previous = min((c for c in self.columns.values() if lo-1e-9 <= c['speed_kmh'] <= hi+1e-9 and self.ok(c)),
                               key=lambda c: c['speed_kmh'], default=previous)
            else:
                self.events.append(dict(kind='segment unresolved', speed_kmh=segment['speed_kmh'], status=start.get('boundary_status')))
        checks = self.check()
        # Passing withheld checks are exact verified states: keep them as
        # extra outline points (they were not used to accept their interval).
        for col in self.witnesses:
            col['withheld_check'] = True
            self.columns.setdefault(float(col['speed_kmh']), col)
        cols = sorted(self.columns.values(), key=lambda c: c['speed_kmh'])
        return dict(columns=cols, segments=segments, mach_seams_kmh=cuts, events=self.events, checks=checks,
                    work_by_role=self.work, elapsed_s=time.perf_counter()-started,
                    speed_limit=dict(kind=limit['kind'], speed_kmh=limit['speed_kmh'], enforced=limit['enforced']))


# ---- per-chart state ------------------------------------------------------
def reset():
    """Forget every cross-column hint and counter (call before each aircraft).

    The cross-speed warm-start rows (PREDICTORS) are hints only, but they make
    later columns depend on earlier ones; a chart must not depend on what a
    previous request in the same process solved."""
    PREDICTORS.clear()
    STATS.clear();STATS.update(columns=0, fallback=0, reasons={}, predictors_offered=0)
    RESCUE_STATS.clear()
    RESCUE_STATS.update(s3_face=0, s3_bisect=0, s3_unresolved=0, s3_paths={}, bisect_trials=0, face_trials=0,
                        behind_face=0, fold_within_tolerance=0)


def stats():
    return json.loads(json.dumps(dict(march=STATS, rescue=RESCUE_STATS), default=str))
