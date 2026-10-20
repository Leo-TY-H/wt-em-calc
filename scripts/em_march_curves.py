"""SEP curves for the alpha-march chart (default chart solver since S4, 2026-10-18).

Moved from research/sep_curves/curves.py (``CurveSolver``) and
research/speed_overhaul/s1_curves.py (``MarchCurveSolver``) without changing
the method; see SPEED-PLAN.md and the S1-S3 reports for its validation.

Solves requested discrete-SEP levels directly with the existing joint
stationary equations plus ``discrete_SEP(state, load) - target = 0`` and the
existing physical admission (``Curve.correct`` -> ``JointProblem.finish``).
In column mode (``columns=``, the march chart's use) every envelope column's
admitted march samples bracket every level; roots at neighbouring columns are
linked into components and continued in speed only to locate the endpoints.

Coverage semantics:
- A failed correction is *unresolved*, never absence.
- A corrected, balanced state that fails a physical gate is a *physical
  exclusion* with its reasons; an unclosed state is a numerical failure.
- Components are never joined across a Mach-formula seam, a speed limit, a
  branch change, or an unresolved interval; ``polyline`` never draws across an
  unresolved knot interval.
- ``coverage_complete`` is always False: sampled loads and speeds can miss
  narrow components between samples.
"""
from __future__ import annotations

from dataclasses import dataclass, asdict
import hashlib
import json
import math
import time

import numpy as np

SCHEMA = 'sep-curves/1'
G = 9.8100004196167
# Fields needed to replay an admitted state through JointProblem.finish and
# to report it. Diagnostics and candidate lists stay in memory only.
STATE_FIELDS = ('speed_kmh', 'load_g', 'turn_dps', 'ps_mps', 'alpha_deg', 'bank_deg', 'solution',
                'stationary_coordinates', 'stationary_coordinate_names', 'stationary_request_indices',
                'sideslip_attitude_deg', 'sideslip_deg', 'stationary_governor_mode', 'model_experiments',
                'mach_curve_mode', 'mach_curve_revision', 'roll_leveling_branch', 'valid', 'reasons',
                'force_error_g', 'angular_error_rad_s2', 'history_error', 'converged')


@dataclass(frozen=True)
class CurvePolicy:
    seed_spacing_kmh: float = 100.
    max_step_kmh: float = 100.
    ladder_loads: tuple = (1., 1.5, 2., 2.5, 3., 3.5, 4., 4.5, 5., 6., 7., 8., 9., 10., 11., 12., 14., 16.)
    max_nfev: int = 40
    refinement_passes: int = 10
    seconds: float = 900.
    skip_covered_seeds: bool = True
    # Envelope-proximity seeds: margin as a fraction of the smallest level gap.
    envelope_margin_fraction: float = .25
    envelope_refinement_depth: int = 2
    min_seed_spacing_kmh: float = 12.5
    check_band_fraction: float = .5
    # Exhaustive ordinary-search retries for unclosed discovery samples. Off by
    # default: an unclosed sample stays an explicit unresolved record (one
    # exhaustive 1 g retry below the stall speed cost >5,000 evaluations).
    exhaustive_retry: bool = False
    edge_unclosed_probes: int = 2
    # Envelope-column mode: minimum speed spacing of root-solved columns.
    column_min_spacing_kmh: float = 40.
    tolerance_scale: float = 1.
    # Step control from the secant predictor's turn error (multiples of the
    # chart turn tolerance): shrink above, grow below.
    predictor_shrink: float = 8.
    predictor_grow: float = 2.


def turn_rate(speed, load):
    return math.degrees(G*math.sqrt(max(0., load*load-1.))/(speed/3.6))


def load_of(speed, turn):
    return math.hypot(1., math.radians(turn)*(speed/3.6)/G)


def compact_state(point):
    return {k: point[k] for k in STATE_FIELDS if k in point}


def compatible(a, b):
    """Conservative curve-local branch identity (no candidate-only modules)."""
    keys = ('stationary_coordinate_names', 'stationary_request_indices')
    return (all(tuple(a.get(k, ())) == tuple(b.get(k, ())) for k in keys) and
            a.get('sideslip_attitude_deg', 0.) == b.get('sideslip_attitude_deg', 0.) and
            a.get('roll_leveling_branch') == b.get('roll_leveling_branch') and
            a.get('stationary_governor_mode') == b.get('stationary_governor_mode'))


def balanced(point):
    return bool(point is not None and point.get('converged') and
                point.get('force_error_g', 1.) <= 2e-4 and
                point.get('angular_error_rad_s2', 1.) <= 5e-5 and
                point.get('history_error', 1.) <= 2e-4)


class Ledger:
    """Exact work by phase, including failed and discarded attempts."""

    def __init__(self, total):
        self.total = total
        self.phases = {}
        self.attempts = []
        self.outcomes = {}

    def snapshot(self):
        return dict(self.total.counts)

    def summary(self):
        """Attempts and aircraft evaluations by phase, kind and outcome."""
        rows = {}
        for a in self.attempts:
            key = '/'.join((a['phase'], a.get('kind', '?'), a['outcome'], 'exhaustive' if a.get('exhaustive') else ''))
            row = rows.setdefault(key.rstrip('/'), dict(attempts=0, aircraft_evaluations=0))
            row['attempts'] += 1;row['aircraft_evaluations'] += a['work'].get('aircraft_evaluations', 0)
        return rows

    def charge(self, phase, before, outcome, **details):
        after = self.total.counts
        delta = {k: after.get(k, 0)-before.get(k, 0) for k in set(after) | set(before)}
        delta = {k: v for k, v in delta.items() if v}
        bucket = self.phases.setdefault(phase, {})
        for k, v in delta.items():bucket[k] = bucket.get(k, 0)+v
        key = phase+':'+outcome
        self.outcomes[key] = self.outcomes.get(key, 0)+1
        self.attempts.append(dict(phase=phase, outcome=outcome, work=delta,
                                  **{k: v for k, v in details.items() if v is not None}))


class CurveSolver:
    def __init__(self, aircraft, config, levels=None, *, policy=CurvePolicy(), solver=None, columns=None):
        from em_stationary_baseline import BaselineSolver
        from em_accuracy_policy import ALLOWANCE, comparison_tolerance
        from em_accuracy import turn_tolerance, speed_tolerance
        self.aircraft, self.config, self.policy = aircraft, config, policy
        self.levels = sorted(float(v) for v in (config['sep_contour_levels_mps'] if levels is None else levels))
        self.solver = solver or BaselineSolver(aircraft, config)
        # The chart's own load-cap semantics (low-speed caps, plot ceiling).
        self.solver.chart_search = True
        self.allowance = float(ALLOWANCE)
        self.band = comparison_tolerance(config)
        self.turn_tol = turn_tolerance(config['sep_tolerance_mps'])
        self.speed_tol = speed_tolerance(config)
        # Optional accuracy relaxation (speed-first mode): scales the curve
        # interpolation tolerances only; physical admission is unchanged.
        self.turn_tol *= policy.tolerance_scale;self.band *= policy.tolerance_scale
        # Optional envelope columns (baseline column records: admitted samples,
        # level-flight and verified boundary states). When given they replace
        # seed discovery: every column brackets every level exactly.
        self.columns = sorted(columns, key=lambda c: c['speed_kmh']) if columns else None
        # Optional callable run before every solve (progress/cancellation); it may raise.
        self.check = None

    # ----- request identity and speed segments ---------------------------
    def identity(self):
        from em_solver import BACKEND
        text = json.dumps(self.config, sort_keys=True, default=str)
        revision = getattr(__import__('em_stationary'), 'REVISION', None)
        return dict(schema=SCHEMA, aircraft=self.aircraft, levels=self.levels,
                    config_sha256=hashlib.sha256(text.encode()).hexdigest(),
                    backend=BACKEND, model_revision=revision, policy=asdict(self.policy))

    def segments(self):
        """(segments, speed limit, Mach seams): em_march_chart.chart_segments (S8: also split where the selected wing
        sweep is unavailable)."""
        from em_march_chart import chart_segments
        segments, limit, seams, _ = chart_segments(self.solver)
        return segments, limit, seams

    # ----- primitive solves ----------------------------------------------
    def event(self, target):
        solver = self.solver

        def event(value):
            # Identical production discrete energy-step reporting.
            solver.complete_reporting(value)
            return value['ps'] - float(target)
        event.stationary_identity = ('direct discrete SEP', float(target))
        return event

    def fixed(self, speed, load, anchor=None, phase='seed_discovery', exhaustive=False):
        from em_sampling import continuation_initial
        if self.check is not None:self.check()
        before = self.ledger.snapshot()
        initial = None
        if anchor is not None:
            attitude = list(anchor['solution'])
            attitude[1] += math.degrees(math.acos(1./load)-math.acos(1./max(1., anchor['load_g'])))
            initial = continuation_initial(self.solver, anchor, attitude)
        point = self.solver.solve(speed, load, initial=initial, exhaustive=exhaustive)
        self.ledger.charge(phase, before, 'admitted' if point['valid'] else
                           'rejected' if balanced(point) else 'unresolved',
                           kind='fixed_load', speed_kmh=speed, load_g=load, exhaustive=exhaustive or None)
        return point

    def seed_from(self, speed, load, anchors, attitude=None):
        from em_sampling import continuation_initial
        anchor = min(anchors, key=lambda p: (abs(p['speed_kmh']-speed), abs(p['load_g']-load)))
        if attitude is None:attitude = list(anchor['solution'])
        attitude = list(attitude)
        nominal = lambda n: math.degrees(math.acos(1./max(1., n)))
        attitude[1] = anchor['bank_deg'] if 'bank_deg' in anchor else attitude[1]
        attitude[1] += nominal(load)-nominal(anchor['load_g'])
        hint = continuation_initial(self.solver, anchor, attitude)
        if isinstance(hint, dict):
            return {**anchor, **hint, 'solution': attitude, 'load_g': load, 'speed_kmh': speed}
        return {**anchor, 'solution': attitude, 'load_g': load, 'speed_kmh': speed}

    def direct(self, speed, target, seed, phase):
        """One bounded corrected solve of the SEP-constrained equations."""
        from em_stationary_boundary import Curve
        if self.check is not None:self.check()
        before = self.ledger.snapshot()
        curve = Curve(self.solver, speed)
        point = curve.correct(seed, event=self.event(target), event_tolerance=self.allowance,
                              max_nfev=self.policy.max_nfev)
        if point is None:outcome, reason = 'unresolved', 'correction failed'
        elif point['valid'] and abs(point['ps_mps']-target) <= self.allowance:outcome, reason = 'admitted', None
        elif not point['valid'] and balanced(point):outcome, reason = 'rejected', point['reasons']
        elif not point['valid']:outcome, reason = 'unresolved', ['unclosed']+list(point['reasons'])
        else:outcome, reason = 'unresolved', 'SEP constraint did not close'
        self.ledger.charge(phase, before, outcome, kind='direct', speed_kmh=speed, target_mps=target,
                           seed_load_g=seed['load_g'], reason=reason)
        return outcome, point, reason

    # ----- discovery -------------------------------------------------------
    def ladder(self, speed, phase='seed_discovery'):
        """Fixed-load samples from 1 g up to the first non-admitted load."""
        from em_load_limits import ceiling
        cap = ceiling(self.solver, speed)
        loads = [n for n in self.policy.ladder_loads if n < cap-1e-9]+[cap]
        points, stop = [], None
        for n in loads:
            p = self.fixed(speed, n, points[-1] if points else None, phase)
            if not p['valid'] and not balanced(p) and self.policy.exhaustive_retry:
                # Optional retry with the exhaustive ordinary search before
                # treating the rung as a numerical gap.
                p = self.fixed(speed, n, points[-1] if points else None, phase, exhaustive=True)
            if not p['valid']:
                stop = dict(load_g=n, kind='rejected' if balanced(p) else 'unresolved',
                            reasons=p['reasons'])
                break
            points.append(p)
        return dict(speed_kmh=speed, points=points, stop=stop, cap_g=cap)

    def ladder_roots(self, ladder, target, phase='seed_discovery'):
        """Every observed bracket for one level, plus the top interval."""
        pts = ladder['points'];speed = ladder['speed_kmh'];found = []
        for a, b in zip(pts, pts[1:]):
            if (a['ps_mps']-target)*(b['ps_mps']-target) > 0:continue
            for p in (a, b):
                if abs(p['ps_mps']-target) <= self.allowance:
                    found.append(dict(outcome='admitted', point=p, bracket=[a['load_g'], b['load_g']]))
                    break
            else:
                t = (target-a['ps_mps'])/(b['ps_mps']-a['ps_mps'])
                w = (a['load_g']**2-1.)+t*((b['load_g']**2-1.)-(a['load_g']**2-1.))
                n = math.sqrt(1.+w)
                f = (n-a['load_g'])/(b['load_g']-a['load_g'])
                attitude = (np.asarray(a['solution'])*(1-f)+np.asarray(b['solution'])*f).tolist()
                outcome, point, reason = self.direct(speed, target, self.seed_from(speed, n, [a, b], attitude), phase)
                if outcome == 'admitted' and not a['load_g']-1e-9 <= point['load_g'] <= b['load_g']+1e-9:
                    outcome, reason = 'unresolved', 'root left observed bracket'
                if outcome == 'admitted' and not (compatible(point, a) or compatible(point, b)):
                    outcome, reason = 'unresolved', 'corrected branch changed'
                found.append(dict(outcome=outcome, point=point, reason=reason, bracket=[a['load_g'], b['load_g']]))
        # The top admitted rung may be above the target while the next rung is
        # excluded: the root may lie in that interval or beyond the envelope.
        if len(pts) >= 2 and ladder['stop'] is not None and pts[-1]['ps_mps'] > target+self.allowance:
            a, b = pts[-2], pts[-1]
            slope = (b['ps_mps']-a['ps_mps'])/((b['load_g']**2)-(a['load_g']**2))
            if slope < 0:
                w = b['load_g']**2-1.+(target-b['ps_mps'])/slope
                n = min(math.sqrt(1.+max(w, 0.)), ladder['stop']['load_g'])
                outcome, point, reason = self.direct(speed, target, self.seed_from(speed, n, [b]), phase)
                if outcome == 'admitted' and not (b['load_g'] <= point['load_g'] <= ladder['stop']['load_g']+1e-9
                                                  and compatible(point, b)):
                    outcome, reason = 'unresolved', 'root outside top interval or branch changed'
                found.append(dict(outcome=outcome, point=point, reason=reason, top_interval=True,
                                  bracket=[b['load_g'], ladder['stop']['load_g']]))
        return found

    def local_edge(self, speed, target, low, high_load, slope, phase='seed_discovery'):
        """Find a root above ``low`` or bound the target beyond the local edge.

        Fixed-load probes: extend while SEP stays above the target, otherwise
        bisect towards the first non-admitted load. Stops once the remaining
        interval cannot reach the target even at three times the observed
        SEP-versus-load-squared slope. A balanced rejection bounds the edge as
        'rejected'; unclosed probes leave it 'unresolved'.
        """
        from em_load_limits import ceiling
        cap = ceiling(self.solver, speed)
        hi = min(max(high_load, low['load_g']+.05), cap)
        hi_kind, hi_reasons = None, None
        for _ in range(6):
            if hi <= low['load_g']+1e-9:
                return dict(outcome='beyond_load_cap', point=None, reason='target above the chart load cap',
                            bracket=[low['load_g'], cap], edge_g=cap)
            p = self.fixed(speed, hi, low, phase)
            if p['valid'] and compatible(p, low):
                if p['ps_mps'] <= target+self.allowance:return self.bracketed(speed, target, low, p, phase)
                slope = (p['ps_mps']-low['ps_mps'])/(p['load_g']**2-low['load_g']**2)
                low = p
                if slope >= 0.:
                    return dict(outcome='unresolved', point=None, reason='non-decreasing SEP near edge',
                                bracket=[low['load_g'], None])
                hi = min(math.sqrt(max(1., low['load_g']**2+1.25*(target-low['ps_mps'])/slope)), cap)
                continue
            hi_kind = 'rejected' if balanced(p) else 'unresolved'
            hi_reasons = p['reasons'];unclosed = int(hi_kind == 'unresolved')
            break
        else:
            return dict(outcome='unresolved', point=None, reason='edge extension budget', bracket=[low['load_g'], hi])
        for _ in range(8):
            dw = hi**2-low['load_g']**2
            if slope is not None and slope < 0. and low['ps_mps']-target-self.allowance > -3.*slope*dw:
                return dict(outcome=hi_kind, point=None, reason=hi_reasons, bracket=[low['load_g'], hi],
                            edge_g=hi, top=low, evidence='target unreachable before the first non-admitted probe '
                                                 '(3x observed SEP slope bound)')
            m = math.sqrt(.5*(low['load_g']**2+hi**2))
            q = self.fixed(speed, m, low, phase)
            if q['valid'] and compatible(q, low):
                if q['ps_mps'] <= target+self.allowance:return self.bracketed(speed, target, low, q, phase)
                slope = (q['ps_mps']-low['ps_mps'])/(q['load_g']**2-low['load_g']**2)
                low = q
            else:
                hi = m;hi_kind = 'rejected' if balanced(q) else 'unresolved';hi_reasons = q['reasons']
                # Unclosed probes are expensive and carry no exclusion
                # evidence; stop after a bounded number.
                unclosed += hi_kind == 'unresolved'
                if unclosed >= self.policy.edge_unclosed_probes:
                    return dict(outcome='unresolved', point=None, reason='unclosed probes near edge: '+str(hi_reasons),
                                bracket=[low['load_g'], hi], top=low)
        return dict(outcome='unresolved', point=None, reason='edge bisection budget', bracket=[low['load_g'], hi], top=low)

    def bracketed(self, speed, target, a, b, phase):
        if abs(b['ps_mps']-target) <= self.allowance:
            return dict(outcome='admitted', point=b, bracket=[a['load_g'], b['load_g']])
        t = (target-a['ps_mps'])/(b['ps_mps']-a['ps_mps'])
        n = math.sqrt(1.+(a['load_g']**2-1.)+t*(b['load_g']**2-a['load_g']**2))
        f = (n-a['load_g'])/(b['load_g']-a['load_g'])
        attitude = (np.asarray(a['solution'])*(1-f)+np.asarray(b['solution'])*f).tolist()
        outcome, point, reason = self.direct(speed, target, self.seed_from(speed, n, [a, b], attitude), phase)
        if outcome == 'admitted' and not (a['load_g']-1e-9 <= point['load_g'] <= b['load_g']+1e-9
                                          and compatible(point, a)):
            outcome, reason = 'unresolved', 'root left observed bracket or branch changed'
        return dict(outcome=outcome, point=point, reason=reason, bracket=[a['load_g'], b['load_g']])

    def one_g(self, speed, phase='seed_discovery', anchor=None):
        one = self.fixed(speed, 1., anchor, phase)
        if not one['valid'] and not balanced(one) and self.policy.exhaustive_retry:
            one = self.fixed(speed, 1., anchor, phase, exhaustive=True)
        return one

    def locate_one_g(self, target, above, below, phase='one_g_discovery'):
        """Bisection/secant on SEP(1 g) between two admitted 1 g states."""
        a, b = above, below
        while abs(b['speed_kmh']-a['speed_kmh']) > self.speed_tol:
            fa, fb = a['ps_mps']-target, b['ps_mps']-target
            lo, hi = a['speed_kmh'], b['speed_kmh']
            mid = lo+(hi-lo)*fa/(fa-fb)
            if not min(lo, hi)+.1*abs(hi-lo) < mid < max(lo, hi)-.1*abs(hi-lo):mid = .5*(lo+hi)
            p = self.fixed(mid, 1., a, phase)
            if not p['valid'] or not compatible(p, a):
                return None
            if abs(p['ps_mps']-target) <= self.allowance:return p
            if p['ps_mps'] > target:a = p
            else:b = p
        return a

    def level_ladder(self, speed, phase='seed_discovery', one=None, levels=None):
        """Seed discovery that uses the requested levels as the rungs.

        1 g and 2 g fixed-load solves, then each lower level solved directly
        from the previous admitted states. A rejected or unresolved corrected
        root is followed by local edge probing (``local_edge``); a rejection
        stops the descent with reasons, and lower levels inherit it as implied.
        Non-monotone or branch-changing results fall back to the complete
        fixed-load ladder at this speed. Roots use the ``ladder_roots`` form.
        """
        from em_load_limits import ceiling
        cap = ceiling(self.solver, speed)
        out = dict(speed_kmh=speed, roots=[], fallback=None, stop=None)
        if one is None:one = self.one_g(speed, phase)
        if not one['valid']:
            out['stop'] = dict(load_g=1., kind='rejected' if balanced(one) else 'unresolved', reasons=one['reasons'])
            return out
        states = [one]
        wanted = sorted(self.levels if levels is None else levels, reverse=True)
        for target in wanted:
            if abs(one['ps_mps']-target) <= self.allowance:
                out['roots'].append(dict(outcome='admitted', point=one, bracket=[1., 1.], target=target))
        pending = [t for t in wanted if t < one['ps_mps']-self.allowance]
        if not pending:return out
        failed_two = None
        if 2. < cap:
            two = self.fixed(speed, 2., one, phase)
            if two['valid'] and compatible(two, one):states.append(two)
            else:failed_two = two
        for index, target in enumerate(pending):
            a, b = sorted(states, key=lambda p: p['load_g'])[-2:] if len(states) > 1 else (None, states[-1])
            if a is not None and a['ps_mps'] != b['ps_mps']:
                slope = (b['ps_mps']-a['ps_mps'])/(b['load_g']**2-a['load_g']**2)
            else:slope = None
            if slope is not None and slope >= 0.:
                out['fallback'] = 'non-decreasing SEP predictor'
                break
            if slope is None:
                # 2 g was not admitted (or is above the cap): locate the edge.
                root = self.local_edge(speed, target, b, 2. if failed_two is not None else cap, None, phase)
            else:
                w = b['load_g']**2-1.+(target-b['ps_mps'])/slope
                n = min(math.sqrt(1.+max(w, 0.)), cap)
                if a is not None and min(a['load_g'], b['load_g']) <= n <= max(a['load_g'], b['load_g']):
                    f = (n-a['load_g'])/(b['load_g']-a['load_g'])
                    attitude = (np.asarray(a['solution'])*(1-f)+np.asarray(b['solution'])*f).tolist()
                    seed = self.seed_from(speed, n, [a, b], attitude)
                else:seed = self.seed_from(speed, n, [b])
                outcome, point, reason = self.direct(speed, target, seed, phase)
                root = dict(outcome=outcome, point=point, reason=reason, bracket=[b['load_g'], n])
                if outcome == 'rejected':
                    edge = self.local_edge(speed, target, b, max(min(point['load_g'], cap), b['load_g']+.05), slope, phase)
                    if edge['outcome'] == 'admitted':root = edge
                    else:root = dict(root, top=edge.get('top'), edge_g=edge.get('edge_g'), edge_outcome=edge['outcome'])
                if outcome == 'unresolved':
                    # Local retained discovery between the last admitted state
                    # and the predicted load, instead of a full ladder.
                    root = self.local_edge(speed, target, b, max(n, b['load_g']+.25), slope, phase)
            outcome, point = root['outcome'], root.get('point')
            if outcome == 'admitted' and not (compatible(point, b) and point['load_g'] >= one['load_g']):
                out['fallback'] = 'branch changed in level descent'
                break
            if outcome == 'admitted':
                # Admitted roots must be ordered: SEP falls with load along the
                # observed branch. Otherwise multiple roots are possible.
                previous = [r['point']['load_g'] for r in out['roots'] if r['outcome'] == 'admitted']
                if previous and point['load_g'] <= max(previous):
                    out['fallback'] = 'non-monotone level roots'
                    break
                out['roots'].append(dict(root, target=target))
                states.append(point)
                continue
            # Rejected or unresolved: lower levels inherit the same evidence
            # (implied), because they would lie beyond the same edge.
            for lower in pending[index:]:
                out['roots'].append(dict(root, target=lower, implied=lower != target))
            out['stop'] = dict(load_g=root.get('edge_g', point['load_g'] if point else None),
                               kind=outcome, reasons=root.get('reason'))
            out['top'] = root.get('top') or max(states, key=lambda p: p['load_g'])
            return out
        if out['fallback']:
            # Retained discovery: complete fixed-load ladder; keep the admitted
            # level roots already found, and add any new brackets.
            ladder = self.ladder(speed, 'fallback_discovery')
            have = [r for r in out['roots'] if r['outcome'] == 'admitted']
            for target in self.levels:
                for root in self.ladder_roots(ladder, target, 'fallback_discovery'):
                    if root['outcome'] == 'admitted' and any(
                            abs(h['point']['load_g']-root['point']['load_g']) < 1e-3 and
                            abs(h['point']['ps_mps']-target) <= self.allowance for h in have):continue
                    out['roots'].append(dict(root, target=target))
            out['stop'] = ladder['stop']
        return out

    # ----- continuation ----------------------------------------------------
    def predict(self, comp, speed, side):
        """Secant predictor in (w = n^2 - 1, attitude) from the nearest knots."""
        knots = comp['knots'] if side > 0 else comp['knots'][::-1]
        a = knots[-1];b = knots[-2] if len(knots) > 1 else None
        wa = a['load_g']**2-1.
        if b is None or b['speed_kmh'] == a['speed_kmh']:
            return wa+comp.get('w_slope', 0.)*(speed-a['speed_kmh']), list(a['solution'])
        r = (speed-a['speed_kmh'])/(a['speed_kmh']-b['speed_kmh'])
        wb = b['load_g']**2-1.
        attitude = (np.asarray(a['solution'])+r*(np.asarray(a['solution'])-np.asarray(b['solution']))).tolist()
        return wa+r*(wa-wb), attitude

    def one_g_endpoint(self, comp, target, inside, outside, side):
        """Locate SEP(1 g) = target between an admitted root and a 1 g sample below target."""
        lo, hi = inside['speed_kmh'], outside['speed_kmh']
        a_in, a_out = inside, outside
        while abs(hi-lo) > self.speed_tol:
            # Secant on the 1 g SEP, safeguarded by bisection.
            f_in = a_in['ps_mps']-target if a_in['load_g'] == 1. else None
            f_out = a_out['ps_mps']-target
            mid = .5*(lo+hi)
            if f_in is not None and f_in != f_out:
                guess = lo+(hi-lo)*f_in/(f_in-f_out)
                if min(lo, hi)+.1*abs(hi-lo) < guess < max(lo, hi)-.1*abs(hi-lo):mid = guess
            p = self.fixed(mid, 1., a_out, 'endpoint')
            if not p['valid']:
                return dict(kind='unresolved', speed_kmh=[lo, hi], reason='1 g sample not admitted',
                            reasons=p['reasons'])
            if abs(p['ps_mps']-target) <= self.allowance:
                return dict(kind='one_g', speed_kmh=mid, state=p)
            if p['ps_mps'] > target:lo, a_in = mid, p
            else:hi, a_out = mid, p
        p = a_in if a_in['load_g'] == 1. else a_out
        return dict(kind='one_g', speed_kmh=p['speed_kmh'], state=p, located_to_kmh=abs(hi-lo),
                    sep_residual_mps=p['ps_mps']-target)

    def march(self, comp, segment, side):
        """Continue a component in one speed direction to an explicit endpoint."""
        target = comp['level'];lo, hi = segment['speed_kmh']
        bound = hi if side > 0 else lo
        bound_kind = segment['ends'][1 if side > 0 else 0]
        step = self.policy.max_step_kmh
        failure = None
        while True:
            last = comp['knots'][-1] if side > 0 else comp['knots'][0]
            if abs(bound-last['speed_kmh']) <= 1e-9:
                return dict(kind=bound_kind, speed_kmh=bound)
            speed = last['speed_kmh']+side*min(step, abs(bound-last['speed_kmh']))
            between = [v for v in self.knot_speeds if side*(v-last['speed_kmh']) > self.speed_tol and
                       side*(speed-v) > 1e-9]
            if between:speed = min(between, key=lambda v: abs(v-last['speed_kmh']))
            w, attitude = self.predict(comp, speed, side)
            if w <= 0. and last['load_g'] == 1. and abs(last['ps_mps']-target) <= self.allowance:
                return dict(kind='one_g', speed_kmh=last['speed_kmh'], state=last)
            if w <= 0.:
                # The curve approaches level flight; confirm with a 1 g sample.
                one = self.fixed(speed, 1., last, 'endpoint')
                if one['valid'] and one['ps_mps'] < target:
                    end = self.one_g_endpoint(comp, target, last, one, side)
                    if end.get('state') is not None and end['kind'] == 'one_g':
                        self.add_knot(comp, end['state'], side)
                    return end
                w = max(w, (last['load_g']**2-1.)*.25)
            n = math.sqrt(1.+max(w, 1e-6))
            outcome, point, reason = self.direct(speed, target, self.seed_from(speed, n, [last], attitude), 'continuation')
            if outcome == 'admitted' and compatible(point, last):
                self.add_knot(comp, point, side)
                error = abs(turn_rate(speed, n)-point['turn_dps'])
                h = abs(speed-last['speed_kmh'])
                if error > self.policy.predictor_shrink*self.turn_tol:step = max(.5*h, self.speed_tol)
                elif error <= self.policy.predictor_grow*self.turn_tol:step = min(self.policy.max_step_kmh, max(step, h)*1.5)
                else:step = min(self.policy.max_step_kmh, max(step, h))
                failure = None
                continue
            if outcome == 'admitted':
                outcome, reason = 'incompatible', 'branch changed during continuation'
            failure = dict(speed_kmh=speed, outcome=outcome, reason=reason, point=point)
            if abs(speed-last['speed_kmh']) <= self.speed_tol:
                return self.classify_end(comp, last, failure, side)
            step = .5*abs(speed-last['speed_kmh'])

    def classify_end(self, comp, last, failure, side):
        """Endpoint between an admitted knot and a failed attempt within speed tolerance."""
        target = comp['level'];speed = failure['speed_kmh']
        one = self.fixed(speed, 1., last, 'endpoint')
        if one['valid'] and one['ps_mps'] < target-self.allowance:
            end = self.one_g_endpoint(comp, target, last, one, side)
            if end.get('state') is not None and end['kind'] == 'one_g':self.add_knot(comp, end['state'], side)
            return end
        if failure['outcome'] == 'rejected':
            return dict(kind='physically_excluded', speed_kmh=[last['speed_kmh'], speed],
                        turn_range_dps=sorted([last['turn_dps'], failure['point']['turn_dps']]),
                        reasons=failure['reason'], state=compact_state(failure['point']))
        # Retained discovery before declaring a numerical end: a ladder at the
        # failed speed. A compatible root continues; anything else is explicit.
        ladder = self.ladder(speed, 'fallback_discovery')
        roots = [r for r in self.ladder_roots(ladder, target, 'fallback_discovery') if r['outcome'] == 'admitted']
        near = [r for r in roots if compatible(r['point'], last)]
        if near:
            r = min(near, key=lambda r: abs(r['point']['turn_dps']-last['turn_dps']))
            if side*(r['point']['speed_kmh']-last['speed_kmh']) > 1e-9:
                self.add_knot(comp, r['point'], side)
                return None  # continue marching
            return dict(kind='unresolved_numerical', speed_kmh=[last['speed_kmh'], speed],
                        reason='fallback root made no progress in speed')
        if roots:
            return dict(kind='branch_change', speed_kmh=[last['speed_kmh'], speed],
                        other_roots=[compact_state(r['point']) for r in roots])
        rejected = [r for r in self.ladder_roots(ladder, target, 'fallback_discovery') if r['outcome'] == 'rejected']
        if ladder['stop'] and ladder['points'] and ladder['points'][-1]['ps_mps'] > target and ladder['stop']['kind'] == 'rejected':
            return dict(kind='physically_excluded', speed_kmh=[last['speed_kmh'], speed],
                        reasons=ladder['stop']['reasons'], evidence='ladder stopped by physical rejection above the target')
        return dict(kind='unresolved_numerical', speed_kmh=[last['speed_kmh'], speed],
                    reason=failure['reason'], rejected_roots=len(rejected))

    def add_knot(self, comp, point, side):
        if any(abs(k['speed_kmh']-point['speed_kmh']) <= 1e-9 for k in comp['knots']):return
        if side > 0:comp['knots'].append(point)
        else:comp['knots'].insert(0, point)

    def model_knot_speeds(self, lo, hi):
        """Speeds where the model's speed dependence has kinks: jet thrust-table
        speeds and stable-Mach polynomial piece ends/clamp onsets. Continuation
        lands on these so interpolation never spans a known kink."""
        return model_knot_speeds(self.solver, self.config, lo, hi, self.speed_tol)

    def follow(self, comp, segment):
        for side, key in ((1, 'end'), (-1, 'start')):
            while True:
                end = self.march(comp, segment, side)
                if end is not None:break
            comp[key] = end

    # ----- interpolation validation -----------------------------------------
    def interpolant(self, knots):
        from scipy.interpolate import PchipInterpolator
        v = np.array([p['speed_kmh'] for p in knots]);w = np.array([p['load_g']**2-1. for p in knots])
        if len(v) < 2:return None
        return PchipInterpolator(v, w)

    def check_intervals(self, comp):
        """Withheld exact solves at interval midpoints; insert knots on failure."""
        target = comp['level'];checks = comp.setdefault('checks', [])
        unresolved = comp.setdefault('unresolved_intervals', [])
        for _ in range(self.policy.refinement_passes):
            knots = comp['knots'];f = self.interpolant(knots)
            if f is None:return
            done = {(c['interval_kmh'][0], c['interval_kmh'][1]) for c in checks if c['passed']}
            blocked = {tuple(u['interval_kmh']) for u in unresolved}
            additions = []
            for a, b in zip(knots, knots[1:]):
                key = (a['speed_kmh'], b['speed_kmh'])
                if key in done or key in blocked:continue
                if b['speed_kmh']-a['speed_kmh'] <= self.speed_tol:
                    continue  # within horizontal tolerance; no withheld speed exists
                speed = .5*(a['speed_kmh']+b['speed_kmh'])
                w = float(f(speed));n = math.sqrt(1.+max(w, 0.))
                t = .5
                attitude = (np.asarray(a['solution'])*(1-t)+np.asarray(b['solution'])*t).tolist()
                outcome, truth, reason = self.direct(speed, target, self.seed_from(speed, n, [a, b], attitude),
                                                     'interpolation_checks')
                if outcome == 'admitted' and not (compatible(truth, a) and compatible(truth, b)):
                    outcome, reason = 'incompatible', 'midpoint branch differs from knots'
                if outcome != 'admitted':
                    # Retry by continuation from each end knot alone (the
                    # interpolated attitude can be a poor start on a steep
                    # curve). A success becomes a knot, not a withheld check.
                    for anchor in (a, b):
                        w0 = anchor['load_g']**2-1.
                        retry_outcome, retry, _ = self.direct(speed, target, self.seed_from(speed, math.sqrt(1.+max(w0, 1e-6)), [anchor]),
                                                              'interpolation_checks')
                        if retry_outcome == 'admitted' and compatible(retry, a) and compatible(retry, b):
                            break
                    else:retry = None
                    if retry is not None:
                        additions.append(retry);continue
                    unresolved.append(dict(interval_kmh=list(key), reason=reason, outcome=outcome))
                    continue
                interp_turn = turn_rate(speed, n);turn_error = abs(interp_turn-truth['turn_dps'])
                record = dict(interval_kmh=list(key), speed_kmh=speed, interpolated_turn_dps=interp_turn,
                              exact_turn_dps=truth['turn_dps'], turn_error_dps=turn_error,
                              holdout=compact_state(truth))
                passed = turn_error <= self.turn_tol
                if not passed:
                    # Combined band: exact SEP at the interpolated point.
                    p = self.fixed(speed, max(1., n), truth, 'interpolation_checks')
                    record.update(interpolated_point_valid=p['valid'], sep_error_mps=abs(p['ps_mps']-target),
                                  interpolated_point_reasons=p['reasons'])
                    # Midpoints are not the worst case: accept the SEP route only
                    # with margin (fraction of the comparison band).
                    passed = bool(p['valid'] and abs(p['ps_mps']-target) <= self.policy.check_band_fraction*self.band)
                record['passed'] = passed
                checks.append(record)
                if not passed:additions.append(truth)
            if not additions:return
            for p in additions:
                comp['knots'].append(p)
            comp['knots'].sort(key=lambda p: p['speed_kmh'])
            # Holdouts that became knots are no longer withheld evidence.
            speeds = {p['speed_kmh'] for p in additions}
            for c in checks:
                if c['holdout']['speed_kmh'] in speeds:c['became_knot'] = True
        comp['refinement_exhausted'] = True

    # ----- top level ---------------------------------------------------------
    def column_samples(self, column):
        """(admitted samples by load up to the verified boundary, boundary state or None)."""
        speed = column['speed_kmh']
        top = column.get('boundary') if column.get('boundary_status') in ('verified limit', 'plot ceiling') else None
        cap = top['load_g'] if top else max((p['load_g'] for p in column['points'] if p.get('valid')), default=None)
        pts = sorted((p for p in column['points'] if p.get('valid') and p.get('stationary_coordinates') is not None
                      and abs(p['speed_kmh']-speed) <= 1e-6 and (cap is None or p['load_g'] <= cap+1e-12)),
                     key=lambda p: p['load_g'])
        if top is not None and top.get('valid') and top.get('stationary_coordinates') is not None and \
                all(abs(p['load_g']-top['load_g']) > 1e-12 for p in pts):
            pts.append(top)
        return pts, top

    def column_levels(self, column):
        """Levels ``column_roots`` would find a root for at this column, from
        the samples alone (no solves); None for an uninformative column."""
        pts, _ = self.column_samples(column)
        if not pts or abs(pts[0]['load_g']-1.) > 1e-9:return None
        return frozenset(t for t in self.levels if abs(pts[0]['ps_mps']-t) <= self.allowance or any(
            (a['ps_mps']-t)*(b['ps_mps']-t) <= 0. and compatible(a, b) for a, b in zip(pts, pts[1:])))

    def column_roots(self, column, levels):
        """Level roots bracketed by one envelope column's admitted states.

        Brackets come from adjacent admitted, branch-compatible samples between
        the level-flight state and the verified boundary. A level above the
        top admitted state while the boundary is verified is reported as beyond
        the envelope at this speed (the boundary state is the evidence)."""
        speed = column['speed_kmh'];out = dict(speed_kmh=speed, roots=[], fallback=None, stop=None)
        pts, top = self.column_samples(column)
        if not pts or abs(pts[0]['load_g']-1.) > 1e-9:
            # Without the admitted level-flight state the column cannot bracket
            # levels from below; it carries no curve evidence (not exclusion).
            out['stop'] = dict(load_g=None, kind='unresolved', reasons=['no admitted level-flight state in column'])
            out['uninformative'] = True
            return out
        for target in levels:
            found = False
            for a, b in zip(pts, pts[1:]):
                if (a['ps_mps']-target)*(b['ps_mps']-target) > 0. or not compatible(a, b):continue
                root = self.bracketed(speed, target, a, b, 'column_roots')
                out['roots'].append(dict(root, target=target));found = True
            if found:continue
            if abs(pts[0]['ps_mps']-target) <= self.allowance:
                out['roots'].append(dict(outcome='admitted', point=pts[0], bracket=[pts[0]['load_g']]*2, target=target))
            elif top is not None and pts[-1] is top and top['ps_mps'] > target:
                out['roots'].append(dict(outcome='rejected', point=None, target=target, edge_g=top['load_g'],
                                         reason=['beyond verified envelope: '+str(column.get('boundary_reason'))],
                                         evidence='verified boundary SEP above the level'))
        return out

    def seed(self, speed, one, segment, components, seeds_report, excluded, unresolved_targets,
             only=None, provenance='seed', levels=None, column=None):
        seed = self.column_roots(column, levels if levels is not None else self.levels) if column is not None \
            else self.level_ladder(speed, one=one, levels=levels)
        self.ladders.append(dict(speed_kmh=speed, segment=segment['speed_kmh'], ladder=seed))
        seeds_report.append(dict(speed_kmh=speed, provenance=provenance,
                                 roots=sum(r['outcome'] == 'admitted' for r in seed['roots']),
                                 stop=seed['stop'], fallback=seed['fallback']))
        for root in seed['roots']:
            target = root['target']
            if only is not None and target != only:continue
            if root['outcome'] in ('rejected', 'beyond_load_cap'):
                excluded.append(dict(target_mps=target, speed_kmh=speed, implied=root.get('implied', False),
                                     kind=root['outcome'], reasons=root.get('reason'),
                                     edge_g=root.get('edge_g'), evidence=root.get('evidence')))
                continue
            if root['outcome'] != 'admitted':
                unresolved_targets.append(dict(target_mps=target, speed_kmh=speed,
                    bracket_g=root['bracket'], reason=root['reason']))
                continue
            p = root['point']
            if self.covered(components, target, p):continue
            comp = dict(level=target, segment=segment['speed_kmh'], knots=[p],
                        discovered_at_kmh=speed, provenance=provenance)
            components.append(comp)
            self.follow(comp, segment)
        return seed

    def link_columns(self, segment, by_speed, components, seeds_report, excluded, unresolved_targets, ones):
        """Envelope-column mode: every column's bracketed roots become knots.

        Roots at adjacent columns are linked when they have the same level and
        a compatible branch (closest turn rate when several qualify); a level
        absent at a column closes its component there. Continuation is then
        used only beyond each component's first and last knots to locate the
        endpoints. Interpolation checks follow as in the default mode.

        Columns denser than ``column_min_spacing_kmh`` are not root-solved,
        except for a level whose presence at a skipped column (read from its
        samples, no solves) differs from both neighbouring solved columns: that
        column solves that level only. Otherwise a component narrower than the
        spacing that touches only the envelope, such as Ps = 0 at a corner,
        would be missed although the envelope columns bracket it."""
        new, open_ = [], {}
        ordered = sorted(by_speed);last_used = None
        solved, presence = [], {}
        for speed in ordered:
            # Columns denser than the curve needs are not root-solved (the
            # interpolation checks still validate every knot interval).
            if (last_used is not None and speed-last_used < self.policy.column_min_spacing_kmh
                    and speed != ordered[-1]):continue
            presence[speed] = self.column_levels(by_speed[speed])
            if presence[speed] is None:continue  # uninformative: keeps components open across it
            solved.append(speed);last_used = speed
        extra = {}
        for a, b in zip(solved, solved[1:]):
            for speed in ordered:
                if not a < speed < b or speed in presence:continue
                here = self.column_levels(by_speed[speed])
                if here is None:continue
                levels = [L for L in self.levels if (L in here) != (L in presence[a]) and (L in here) != (L in presence[b])]
                if levels:extra[speed] = levels
        for speed in ordered:
            if speed not in presence and speed not in extra:continue
            column = by_speed[speed]
            levels = extra.get(speed, self.levels)
            if speed in presence:
                one = next((p for p in column['points'] if abs(p['load_g']-1.) <= 1e-9 and p.get('valid')
                            and p.get('stationary_coordinates') is not None), None)
                ones[speed] = self.ones[speed] = one if one is not None else dict(
                    valid=False, reasons=['no admitted level-flight state in column'], speed_kmh=speed)
            seed = self.column_roots(column, levels)
            self.ladders.append(dict(speed_kmh=speed, segment=segment['speed_kmh'], ladder=seed))
            seeds_report.append(dict(speed_kmh=speed, provenance='envelope column' if speed in presence else
                                     'envelope column (level presence change)',
                                     roots=sum(r['outcome'] == 'admitted' for r in seed['roots']), stop=seed['stop']))
            if seed.get('uninformative'):continue  # keep components open across it
            # Levels not solved at this column keep their open components.
            current = {L: c for L, c in open_.items() if L not in levels}
            for root in seed['roots']:
                target = root['target']
                if root['outcome'] in ('rejected', 'beyond_load_cap'):
                    excluded.append(dict(target_mps=target, speed_kmh=speed, kind=root['outcome'],
                                         reasons=root.get('reason'), evidence=root.get('evidence')))
                    continue
                if root['outcome'] != 'admitted':
                    unresolved_targets.append(dict(target_mps=target, speed_kmh=speed,
                                                   bracket_g=root.get('bracket'), reason=root.get('reason')))
                    continue
                p = root['point']
                taken = current.get(target, [])
                choices = [c for c in open_.get(target, []) if c not in taken and compatible(c['knots'][-1], p)]
                if choices:
                    comp = min(choices, key=lambda c: abs(c['knots'][-1]['turn_dps']-p['turn_dps']))
                    self.add_knot(comp, p, 1)
                else:
                    comp = dict(level=target, segment=segment['speed_kmh'], knots=[p], discovered_at_kmh=speed,
                                provenance='envelope column')
                    new.append(comp)
                current.setdefault(target, []).append(comp)
            open_ = current
        components.extend(new)
        for comp in new:self.follow(comp, segment)

    def envelope_refinement(self, segment, components, seeds_report, excluded, unresolved_targets):
        """Extra seeds where a level is excluded at two neighbouring seeds but the
        highest admitted SEP below the edge is within ``envelope_margin`` of it:
        a component touching only the upper envelope may lie between them."""
        margin = self.policy.envelope_margin_fraction*min(
            [b-a for a, b in zip(self.levels, self.levels[1:])] or [100.])
        for _ in range(self.policy.envelope_refinement_depth):
            rows = sorted((r for r in self.ladders if r['segment'] == segment['speed_kmh']),
                          key=lambda r: r['speed_kmh'])
            added = False
            for a, b in zip(rows, rows[1:]):
                if b['speed_kmh']-a['speed_kmh'] < self.policy.min_seed_spacing_kmh:continue
                ta, tb = a['ladder'].get('top'), b['ladder'].get('top')
                if ta is None or tb is None:continue
                excluded_a = {r['target'] for r in a['ladder']['roots'] if r['outcome'] in ('rejected', 'beyond_load_cap')}
                excluded_b = {r['target'] for r in b['ladder']['roots'] if r['outcome'] in ('rejected', 'beyond_load_cap')}
                near = [L for L in excluded_a & excluded_b if min(ta['ps_mps'], tb['ps_mps'])-L < margin]
                if not near:continue
                mid = .5*(a['speed_kmh']+b['speed_kmh'])
                self.seed(mid, None, segment, components, seeds_report, excluded, unresolved_targets, levels=near,
                          provenance='envelope proximity')
                added = True
            if not added:return

    def run(self):
        from em_trim_work import Work, WorkLimit, use
        started = time.perf_counter()
        total = Work(dict(seconds=self.policy.seconds))
        self.ledger = Ledger(total)
        segments, limit, cuts = self.segments()
        self.ladders = [];self.ones = {}
        self.knot_speeds = self.model_knot_speeds(float(self.config['speed_min_kmh']), float(self.config['speed_max_kmh']))
        components, seeds_report, unresolved_targets, excluded = [], [], [], []
        status = 'completed';junctions = []
        try:
            with use(total):
                for segment in segments:
                    lo, hi = segment['speed_kmh']
                    count = max(1, math.ceil((hi-lo)/self.policy.seed_spacing_kmh))
                    # Segment ends (one-sided at seams/limits) plus interior seeds.
                    speeds = sorted(set([lo, hi]+[lo+i*(hi-lo)/count for i in range(1, count)]))
                    by_speed = {}
                    if self.columns is not None:
                        by_speed = {c['speed_kmh']: c for c in self.columns if lo-1e-9 <= c['speed_kmh'] <= hi+1e-9}
                        speeds = sorted(by_speed)
                    # Centre-out order: the first components cover most of the
                    # segment, so later seeds where every level is already
                    # spanned by a component are skipped (recorded as such).
                    speeds.sort(key=lambda v: (abs(v-.5*(lo+hi)), v))
                    ones = {}
                    if self.columns is not None:
                        self.link_columns(segment, by_speed, components, seeds_report, excluded, unresolved_targets, ones)
                        speeds = []
                    for speed in speeds:
                        # The 1 g backbone is always sampled: it is cheap and
                        # brackets every curve end on the level-flight line.
                        if self.columns is not None:
                            column = by_speed[speed]
                            one = next((p for p in column['points'] if p['load_g'] == 1. and p.get('valid')
                                        and p.get('stationary_coordinates') is not None), None)
                            if one is None:
                                one = dict(valid=False, reasons=['no admitted level-flight state in column'], speed_kmh=speed)
                            ones[speed] = self.ones[speed] = one
                        else:
                            ones[speed] = self.ones[speed] = one = self.one_g(speed)
                        spanned = {c['level'] for c in components if c['segment'] == segment['speed_kmh'] and
                                   c['knots'][0]['speed_kmh']-1e-9 <= speed <= c['knots'][-1]['speed_kmh']+1e-9}
                        if self.policy.skip_covered_seeds and spanned >= set(self.levels):
                            seeds_report.append(dict(speed_kmh=speed, skipped='all levels spanned by components',
                                                     one_g_valid=one['valid']))
                            continue
                        # Levels already spanned here by a component are not re-solved.
                        self.seed(speed, one, segment, components, seeds_report, excluded, unresolved_targets,
                                  levels=[L for L in self.levels if L not in spanned], column=by_speed.get(speed))
                    if self.columns is None:
                        self.envelope_refinement(segment, components, seeds_report, excluded, unresolved_targets)
                    # Odd numbers of 1 g crossings between adjacent seeds that no
                    # component end explains start a new component inside.
                    ordered = sorted(ones.items())
                    for (va, pa), (vb, pb) in zip(ordered, ordered[1:]):
                        if not (pa['valid'] and pb['valid'] and compatible(pa, pb)):continue
                        for target in self.levels:
                            if (pa['ps_mps']-target)*(pb['ps_mps']-target) >= 0.:continue
                            explained = any(c['level'] == target and any(
                                e and e.get('kind') == 'one_g' and va-self.speed_tol <= (
                                    e['speed_kmh'] if not isinstance(e['speed_kmh'], list) else e['speed_kmh'][0]) <= vb+self.speed_tol
                                for e in (c.get('start'), c.get('end'))) for c in components)
                            if explained:continue
                            above, below = (pa, pb) if pa['ps_mps'] > target else (pb, pa)
                            crossing = self.locate_one_g(target, above, below)
                            if crossing is None:
                                unresolved_targets.append(dict(target_mps=target, speed_kmh=[va, vb],
                                    reason='1 g crossing bracketed but not localized'))
                                continue
                            if self.covered(components, target, crossing):continue
                            comp = self.from_one_g(crossing, above, below, target, segment)
                            if comp is None:
                                unresolved_targets.append(dict(target_mps=target, speed_kmh=crossing['speed_kmh'],
                                    reason='1 g crossing found; level slope unavailable'))
                                continue
                            components.append(comp)
                            self.follow(comp, segment)
                for comp in components:
                    self.check_intervals(comp)
                junctions = self.seam_junctions(segments, components)
        except WorkLimit as error:
            status = 'exhausted: '+str(error)
        elapsed = time.perf_counter()-started
        work = dict(total.counts)
        return dict(identity=self.identity(), status=status, elapsed_s=elapsed,
                    segments=segments, speed_limit=dict(kind=limit['kind'], speed_kmh=limit['speed_kmh'],
                                                       enforced=limit['enforced']),
                    mach_seams_kmh=cuts, model_knot_speeds_kmh=self.knot_speeds, seeds=seeds_report, seam_junctions=junctions,
                    components=[self.report(c) for c in components],
                    unresolved_targets=unresolved_targets, excluded_targets=excluded,
                    work=work, work_by_phase=self.ledger.phases, outcomes=self.ledger.outcomes,
                    attempt_count=len(self.ledger.attempts), attempt_work=self.ledger.summary(),
                    coverage_complete=False,
                    coverage_scope='sampled fixed-load ladders at seed speeds plus speed continuation; '
                                   'not an absence proof between samples')

    def from_one_g(self, crossing, above, below, target, segment):
        """Component seeded by an admitted 1 g state with SEP = target.

        dSEP(1 g)/dV comes from the bracketing 1 g states, dSEP/dw from one
        2 g solve at the crossing speed; their ratio predicts w(V) for the
        first continuation step. Every later step is corrected as usual."""
        v = crossing['speed_kmh']
        dv = above['speed_kmh']-below['speed_kmh']
        if dv == 0.:return None
        d1 = (above['ps_mps']-below['ps_mps'])/dv
        two = self.fixed(v, 2., crossing, 'one_g_discovery')
        if not (two['valid'] and compatible(two, crossing)):return None
        dw = (two['ps_mps']-crossing['ps_mps'])/3.
        if dw >= 0.:return None
        return dict(level=target, segment=segment['speed_kmh'], knots=[crossing], discovered_at_kmh=v,
                    provenance='1 g crossing', w_slope=-d1/dw)

    def seam_junctions(self, segments, components):
        """Vertical chart connections along a Mach seam, from one-sided evidence.

        For each seam and level: the one-sided curve-end turn rates (or None)
        and one-sided 1 g SEP values. The connecting span is reported only
        where both sides are established; otherwise it is explicitly open."""
        out = []
        for left, right in zip(segments, segments[1:]):
            if left['ends'][1] != 'mach_seam':continue
            vl, vr = left['speed_kmh'][1], right['speed_kmh'][0]
            ol, orr = self.ones.get(vl), self.ones.get(vr)
            for L in self.levels:
                def end_turn(segment, side):
                    for c in components:
                        if c['level'] == L and c['segment'] == segment['speed_kmh']:
                            e = c.get('end' if side > 0 else 'start')
                            if e and e.get('kind') == 'mach_seam':
                                return (c['knots'][-1] if side > 0 else c['knots'][0])['turn_dps']
                    return None
                tl, tr = end_turn(left, 1), end_turn(right, -1)
                below = lambda one: None if not (one and one['valid']) else one['ps_mps'] < L
                bl, br = below(ol), below(orr)
                lower = [t for t in (tl, tr) if t is not None]
                if tl is not None and tr is not None:span, status = [min(tl, tr), max(tl, tr)], 'established'
                elif lower and ((tl is None and br) or (tr is None and bl)):
                    span, status = [0., lower[0]], 'established (other side below level at 1 g)'
                elif lower:span, status = [lower[0], None], 'open: other side not reached below the envelope'
                elif bl is not None and br is not None and bl != br:
                    span, status = [0., None], 'open: 1 g SEP straddles level; envelope not computed'
                else:continue
                out.append(dict(seam_kmh=[vl, vr], level=L, left_turn_dps=tl, right_turn_dps=tr,
                                left_one_g_sep=ol and ol.get('ps_mps'), right_one_g_sep=orr and orr.get('ps_mps'),
                                span_turn_dps=span, status=status))
        return out

    def covered(self, components, target, point):
        for comp in components:
            if comp['level'] != target:continue
            v = [p['speed_kmh'] for p in comp['knots']]
            if not v[0]-1e-9 <= point['speed_kmh'] <= v[-1]+1e-9:continue
            f = self.interpolant(comp['knots'])
            if f is None:
                if (compatible(point, comp['knots'][0]) and
                        abs(comp['knots'][0]['turn_dps']-point['turn_dps']) <= self.turn_tol):return True
                continue
            if not compatible(point, comp['knots'][0]):continue
            n = math.sqrt(1.+max(float(f(point['speed_kmh'])), 0.))
            if abs(turn_rate(point['speed_kmh'], n)-point['turn_dps']) <= 4*self.turn_tol:
                return True
            # The unrefined interpolant disagrees: decide identity with an
            # exact continuation solve from the component at this speed.
            i = max(1, min(len(v)-1, int(np.searchsorted(v, point['speed_kmh']))))
            a, b = comp['knots'][i-1], comp['knots'][i]
            t = (point['speed_kmh']-a['speed_kmh'])/max(b['speed_kmh']-a['speed_kmh'], 1e-12)
            attitude = (np.asarray(a['solution'])*(1-t)+np.asarray(b['solution'])*t).tolist()
            outcome, same, _ = self.direct(point['speed_kmh'], target,
                                           self.seed_from(point['speed_kmh'], n, [a, b], attitude), 'seed_identity')
            if (outcome == 'admitted' and compatible(same, a) and
                    abs(same['turn_dps']-point['turn_dps']) <= self.turn_tol):
                # Same root: keep the exact seed state as a knot.
                if all(abs(k['speed_kmh']-point['speed_kmh']) > 1e-9 for k in comp['knots']):
                    comp['knots'].insert(i, point)
                return True
        return False

    def report(self, comp):
        end = lambda e: None if e is None else {k: (compact_state(v) if k == 'state' and isinstance(v, dict) and 'solution' in v else v)
                                               for k, v in e.items()}
        return dict(level=comp['level'], segment_kmh=comp['segment'], discovered_at_kmh=comp['discovered_at_kmh'],
                    provenance=comp.get('provenance'),
                    knots=[compact_state(p) for p in comp['knots']], start=end(comp.get('start')), end=end(comp.get('end')),
                    checks=comp.get('checks', []), unresolved_intervals=comp.get('unresolved_intervals', []),
                    refinement_exhausted=comp.get('refinement_exhausted', False))


def polyline(component, samples=200, gaps=False):
    """Dense (speed, turn) drawing of one component from its knots.

    Unresolved knot intervals are never drawn: their interior samples are
    omitted, and with ``gaps=True`` a ``None`` separator marks each break."""
    from scipy.interpolate import PchipInterpolator
    knots = component['knots']
    if len(knots) < 2:
        return [(knots[0]['speed_kmh'], knots[0]['turn_dps'])] if knots else []
    v = np.array([p['speed_kmh'] for p in knots]);w = np.array([p['load_g']**2-1. for p in knots])
    f = PchipInterpolator(v, w);xs = np.unique(np.r_[np.linspace(v[0], v[-1], samples), v])
    open_ = [tuple(u['interval_kmh']) for u in component.get('unresolved_intervals', [])]
    out = []
    for x in xs:
        inside = any(lo < x < hi for lo, hi in open_)
        if inside:
            if gaps and out and out[-1] is not None:out.append(None)
            continue
        out.append((float(x), turn_rate(float(x), math.sqrt(1.+max(float(f(x)), 0.)))))
    return out


def model_knot_speeds(solver, config, lo, hi, speed_tol):
    """CurveSolver.model_knot_speeds for any solver of the aircraft (S7: the envelope tracer places its withheld
    checks there too)."""
    from em_mach_events import catalog
    from air_state import speed_of_sound
    scale = 3.6*speed_of_sound(config['altitude_m'])
    speeds = {float(v)*3.6 for u in getattr(solver.engine, 'units', ())
              if hasattr(u, 'jet') for v in u.jet['speed']}
    try:info = catalog(solver, force=True)
    except Exception:info = {}
    for a, b, high, slope, limit in info.get('polynomial_pieces', ()):
        speeds.update((a*scale, b*scale))
        if slope:
            m = b+(limit-high)/slope
            if m > b:speeds.add(m*scale)
    for r in info.get('approximations', []):speeds.update(m*scale for m in r['mach_knots'])
    return sorted(v for v in speeds if lo+speed_tol < v < hi-speed_tol)


class MarchCurveSolver(CurveSolver):
    """CurveSolver with one change to the withheld interpolation checks: when a
    knot interval contains a model knot speed (jet thrust-table speed or
    stable-Mach polynomial piece end, where the curve has a kink), the withheld
    exact solve is placed at that speed instead of the midpoint, because PCHIP
    across a kink is worst next to the kink. That exact state is always kept as
    a knot. Everything else is CurveSolver unchanged."""
    def check_speed(self, a, b):
        mid = .5*(a+b)
        inside = [v for v in getattr(self, 'knot_speeds', ()) if a+self.speed_tol < v < b-self.speed_tol]
        return min(inside, key=lambda v: abs(v-mid)) if inside else mid

    def check_intervals(self, comp):
        """CurveSolver.check_intervals with kink-placed withheld checks."""
        target = comp['level'];checks = comp.setdefault('checks', [])
        unresolved = comp.setdefault('unresolved_intervals', [])
        for _ in range(self.policy.refinement_passes):
            knots = comp['knots'];f = self.interpolant(knots)
            if f is None:return
            done = {(c['interval_kmh'][0], c['interval_kmh'][1]) for c in checks if c['passed']}
            blocked = {tuple(u['interval_kmh']) for u in unresolved}
            additions = []
            for a, b in zip(knots, knots[1:]):
                key = (a['speed_kmh'], b['speed_kmh'])
                if key in done or key in blocked:continue
                if b['speed_kmh']-a['speed_kmh'] <= self.speed_tol:continue
                speed = self.check_speed(a['speed_kmh'], b['speed_kmh'])
                w = float(f(speed));n = math.sqrt(1.+max(w, 0.))
                t = (speed-a['speed_kmh'])/(b['speed_kmh']-a['speed_kmh'])
                attitude = (np.asarray(a['solution'])*(1-t)+np.asarray(b['solution'])*t).tolist()
                outcome, truth, reason = self.direct(speed, target, self.seed_from(speed, n, [a, b], attitude),
                                                     'interpolation_checks')
                if outcome == 'admitted' and not (compatible(truth, a) and compatible(truth, b)):
                    outcome, reason = 'incompatible', 'midpoint branch differs from knots'
                if outcome != 'admitted':
                    for anchor in (a, b):
                        w0 = anchor['load_g']**2-1.
                        retry_outcome, retry, _ = self.direct(speed, target, self.seed_from(
                            speed, math.sqrt(1.+max(w0, 1e-6)), [anchor]), 'interpolation_checks')
                        if retry_outcome == 'admitted' and compatible(retry, a) and compatible(retry, b):
                            break
                    else:retry = None
                    if retry is not None:
                        additions.append(retry);continue
                    unresolved.append(dict(interval_kmh=list(key), reason=reason, outcome=outcome))
                    continue
                interp_turn = turn_rate(speed, n);turn_error = abs(interp_turn-truth['turn_dps'])
                record = dict(interval_kmh=list(key), speed_kmh=speed, interpolated_turn_dps=interp_turn,
                              exact_turn_dps=truth['turn_dps'], turn_error_dps=turn_error,
                              holdout=compact_state(truth), at_model_knot=abs(speed-.5*sum(key)) > 1e-9)
                passed = turn_error <= self.turn_tol
                if record['at_model_knot']:
                    # The exact state at a kink is always kept as a knot, so
                    # the interpolant never spans the kink.
                    record['passed'] = False;record['kink_knot'] = True
                    checks.append(record);additions.append(truth)
                    continue
                if not passed:
                    p = self.fixed(speed, max(1., n), truth, 'interpolation_checks')
                    record.update(interpolated_point_valid=p['valid'], sep_error_mps=abs(p['ps_mps']-target),
                                  interpolated_point_reasons=p['reasons'])
                    passed = bool(p['valid'] and abs(p['ps_mps']-target) <= self.policy.check_band_fraction*self.band)
                record['passed'] = passed
                checks.append(record)
                if not passed:additions.append(truth)
            if not additions:return
            for p in additions:comp['knots'].append(p)
            comp['knots'].sort(key=lambda p: p['speed_kmh'])
            speeds = {p['speed_kmh'] for p in additions}
            for c in checks:
                if c['holdout']['speed_kmh'] in speeds:c['became_knot'] = True
        comp['refinement_exhausted'] = True
