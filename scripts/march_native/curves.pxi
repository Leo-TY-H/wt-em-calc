# S5: the SEP curve solver of the march chart (em_march_curves.MarchCurveSolver in
# envelope-column mode) on native trial states, line by line. Every direct SEP solve is
# the native correction and lean finish; knots stay native states and the component
# report is built from their fields (compact_state). The work ledger is the Python
# Ledger, so the curve work record is unchanged.
#
# S7 (WT_CURVE_S7=0 restores S5/S6 exactly): every knot carries the exact slope dw/dV of its SEP level set (MarchEngine
# .tangent, both one-sided speed derivatives), and the withheld checks judge each interval by position: the exact
# midpoint turn rate within the curve tolerance of the drawn line, allowing the chart's 1 km/h horizontal slack, on the
# piecewise cubic Hermite through values and slopes that the chart draws (em_march_chart.polyline). The SEP band no
# longer passes an interval; it stays the acceptance check of the chart against the reference.
#
# S8 (WT_MARCH_S8=0 restores S7): the envelope's split rule on every contour piece (check_intervals, piece_bound): a
# piece is finished when the bound of its error from its two knots' values and slopes alone (band_load in w) is within
# the curve tolerance, allowing the horizontal slack, and it is at most the maximum step wide; otherwise the level is
# solved where that bound is largest and becomes a knot. No withheld solve; a piece narrower than the speed tolerance
# whose bound still fails is a discontinuity (the component's 'steps', drawn vertical).

STATE_FIELDS = ('speed_kmh', 'load_g', 'turn_dps', 'ps_mps', 'alpha_deg', 'bank_deg', 'solution',
                'stationary_coordinates', 'stationary_coordinate_names', 'stationary_request_indices',
                'sideslip_attitude_deg', 'sideslip_deg', 'stationary_governor_mode', 'model_experiments',
                'mach_curve_mode', 'mach_curve_revision', 'roll_leveling_branch', 'valid', 'reasons',
                'force_error_g', 'angular_error_rad_s2', 'history_error', 'converged', 'chart_rule')


DEF S7_CURVE_CHECK = .6666666666666666   # S7: a midpoint within 2/3 of the curve tolerance (the drawn line's worst point
                                          # can lie off the midpoint, so the interval stays within the tolerance)
DEF S9_SECANT_STEPS = 8                   # S9: prescribed-alpha corrections of one level solve by alpha (alpha_secant)
DEF S9_END_SEP = .1                       # S9: a contour's 1 g end: within this x the SEP allowance ...
DEF S9_END_SPEED = .25                    # ... or bracketed to this x the speed tolerance
DEF S9_END_STEPS = 10                     # ... in at most this many level-flight solves
DEF S9_END_DW = .05                       # S9: the level-turn solve (w = n^2-1) for a 1 g end's slope
DEF S9_END_VERTICAL = 50.                 # S9: the steepest end slope dw/dV (per km/h): a vertical drop to 1 g
DEF S10_FOLD_KNOTS = 10                   # S10: the most knots of one fold piece (the curve's turn in speed)
DEF S10_ROOT_STEPS = 8                    # S10: secant steps in speed of one fold knot (the level at a fixed load)


cdef inline bint numerical_code(int r) noexcept nogil:
    # S9: a rejection reason that reports a sub-solve that did not close (trim, propulsion, vertical step, pitch-response
    # or Instructor evaluation), not a physical limit; an interned reason text of unknown meaning counts as one
    return (r == R_TRIM or r == R_PROP_SETTLE or r == R_CYCLE or r == R_VSTEP or r == R_PITCH_UNRES or r == R_INSTR_UNRES
            or r >= R_FIXED)


cdef double py_sqrt(double x) except? -1.:
    # math.sqrt: ValueError on a negative argument
    if x < 0.:raise ValueError('math domain error')
    return sqrt(x)


cdef inline double nominal_bank(double n) noexcept nogil:
    # math.degrees(math.acos(1./max(1., n)))
    return acos(1./pmax(1., n))*RAD_TO_DEG


cdef inline double curve_turn(double speed, double load) noexcept nogil:
    # em_march_curves.turn_rate
    return G_TURN*sqrt(pmax(0., load*load-1.))/(speed/3.6)*RAD_TO_DEG


cdef class NCurves:
    """MarchCurveSolver.run in column mode."""
    cdef MarchEngine eng
    cdef public object py, ledger, check, total
    cdef public list levels, knot_speeds, ladders
    cdef public dict ones
    cdef double allowance, band, turn_tol, speed_tol
    cdef object policy
    cdef int max_nfev
    cdef double max_step, predictor_shrink, predictor_grow, check_band_fraction, column_min_spacing
    cdef int refinement_passes
    cdef list ladder_loads
    cdef dict column_ids
    # S5: the Jacobian of the latest direct SEP correction (per request branch), the next one's predictor
    cdef PredMap preds
    cdef public bint reuse
    # the Jacobian each admitted direct solve ended with, by its state (a bounded pool): the predictor of
    # a later solve anchored at that state
    cdef Predictor* pool
    cdef int npool, next_pool
    cdef dict pred_of
    cdef list owners
    # S7: exact knot slopes (state id -> (left, right) dw/dV per km/h, NaN where unavailable) and the switch
    cdef public dict slopes
    cdef public bint s7, s8
    cdef double tangent_delta
    cdef public list segs
    # S8: the envelope boundary (verified columns: speed, limiting load) for the transient over-rev rule
    cdef public list env_v, env_n, env_ps
    cdef public list raw_components      # S9 diagnostics: the components with their knot state ids
    cdef public list trace               # S9 diagnostics: every direct solve (when a list)
    cdef public bint s9                  # S9 (WT_MARCH_S9=0 restores S8 exactly)
    cdef double root_dsdv                # S9: dSEP/dV along level flight over one_g_root's last bracket
    cdef public bint exit_seen           # S9: the last level solve met a physical rejection at the level (exit evidence)
    cdef public bint edge_exit           # S9: a step of the last edge_root met one
    cdef public int exit_state           # S9: that physically rejected state
    # S10 (WT_MARCH_S10=0 restores S9 exactly): contours that turn back in speed (Leo, 2026-10-20: "keep the true
    # contour"). Where SEP rises with load from level flight, a level can have two loads at one speed: the branch where
    # SEP falls with load (the usual one) and the one where it rises ('below': level flight lies below the level there);
    # they join where the level's two roots meet, the fold, where the contour turns back in speed. A component followed
    # to a fold ends there ('fold' end) with the fold piece (comp['fold']: knots from its end knot to the other branch's
    # end knot, drawn as speed against w = n^2-1), and the other branch is a component of its own, followed away from it.
    cdef public bint s10
    cdef list pending                    # S10: components found at folds, still to follow
    cdef dict grads                      # S10: (dSEP/dV, dSEP/dw) of curve states by their base state

    @cython.boundscheck(True)
    @cython.wraparound(True)
    def __cinit__(self, *args, **kwargs):
        self.pool = NULL

    def __dealloc__(self):
        if self.pool != NULL:cfree(self.pool)

    def __init__(self, MarchEngine eng, py):
        import os
        self.preds.n = 0
        self.reuse = os.environ.get('WT_CURVE_PREDICTORS', '1') == '1'
        self.npool = 256;self.next_pool = 0;self.pred_of = {};self.owners = [None]*256
        self.pool = <Predictor*>malloc(self.npool*sizeof(Predictor))
        if self.pool == NULL:raise MemoryError()
        self.eng = eng;self.py = py
        self.levels = list(py.levels);self.allowance = py.allowance;self.band = py.band
        self.turn_tol = py.turn_tol;self.speed_tol = py.speed_tol
        p = py.policy;self.policy = p
        self.max_nfev = p.max_nfev;self.max_step = p.max_step_kmh;self.predictor_shrink = p.predictor_shrink
        self.predictor_grow = p.predictor_grow;self.check_band_fraction = p.check_band_fraction
        self.column_min_spacing = p.column_min_spacing_kmh;self.refinement_passes = p.refinement_passes
        self.ladder_loads = list(p.ladder_loads)
        if p.exhaustive_retry:raise NativeMarchUnsupported('exhaustive curve retries')
        self.check = None
        self.column_ids = {}
        self.slopes = {}
        self.s7 = os.environ.get('WT_CURVE_S7', '1') != '0'
        # S8: WT_MARCH_S8=0 restores the S7 curve solver (no rescue of failing stretches)
        self.s8 = os.environ.get('WT_MARCH_S8', '1') != '0' and self.s7
        self.s9 = self.s8 and os.environ.get('WT_MARCH_S9', '1') != '0'
        self.s10 = self.s9 and os.environ.get('WT_MARCH_S10', '1') != '0'
        self.pending = [];self.grads = {}
        self.exit_seen = False
        self.tangent_delta = .5
        self.trace = None;self.raw_components = None

    # ------------------------------------------------------------------ states
    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int state(self, row) except -1:
        """The curve engine's state of an envelope row (imported once per row object)."""
        key = id(row)
        sid = self.column_ids.get(key)
        if sid is None:
            sid = self.eng.import_row(row);self.column_ids[key] = (sid, row)
            return sid
        return sid[0]

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef inline TState* S(self, int sid) noexcept:
        return &self.eng.st[sid]

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int admit_transient(self, int sid) except -1:
        """S8: the engine RPM rule on a curve state: rejected for engine RPM only (its hard checks passed) below the
        envelope boundary at its speed (the boundary of the neighbouring verified columns, the lower of the two), it is
        a transient over-rev, admitted like the envelope's (the boundary is the first hard limit there, or the start of
        the final over-rev stretch above it)."""
        import bisect
        cdef double v, n
        cdef int i
        if sid < 0 or not self.eng.soft or self.eng.st[sid].valid or not self.env_v:return 0
        if not self.eng.only_reasons(sid, (<unsigned long long>1) << R_RPM):return 0
        v = self.eng.st[sid].speed
        i = bisect.bisect_left(self.env_v, v)
        if i < len(self.env_v) and fabs(self.env_v[i]-v) <= 1e-9:n = self.env_n[i]
        elif i > 0 and fabs(self.env_v[i-1]-v) <= 1e-9:n = self.env_n[i-1]
        elif 0 < i < len(self.env_v):n = pmin(self.env_n[i-1], self.env_n[i])
        else:return 0
        if self.eng.st[sid].load < n-1e-9:
            self.eng.st[sid].valid = True;self.eng.st[sid].rule = 1
            self.eng.stats['s8_transient_over_rev'] = self.eng.stats.get('s8_transient_over_rev', 0)+1
        return 0

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef bint balanced(self, int sid) noexcept:
        cdef TState* t = &self.eng.st[sid]
        return (t.converged and t.force_error_g <= 2e-4 and t.angular_error <= 5e-5 and t.history_error <= 2e-4)

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef object compact(self, int sid):
        """em_march_curves.compact_state(point)."""
        cdef TState* t = &self.eng.st[sid]
        row = self.eng.rows[sid] if t.base == sid else None
        if row is not None:
            return {k: row[k] for k in STATE_FIELDS if k in row}
        if t.base != sid:
            out = self.compact(t.base)
            return out
        return self.eng.compact_native(sid)

    # ------------------------------------------------------------------ primitive solves
    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef object initial_dict(self, int anchor, list attitude):
        """em_sampling.continuation_initial(solver, anchor, attitude)."""
        cdef TState* t = &self.eng.st[anchor]
        cdef int i
        names = self.eng.names_of(anchor)
        if not t.has_coords or not names or len(names) != t.nx:return attitude
        predicted = dict(zip(('alpha_deg', 'bank_deg', 'aileron', 'elevator', 'rudder'), attitude))
        return dict(solution=attitude, stationary_coordinate_names=list(names),
                    stationary_coordinates=[predicted.get(name, t.coords[i]) for i, name in enumerate(names)],
                    stationary_request_indices=[t.req[i] for i in range(t.nreq)])

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int fixed(self, double speed, double load, int anchor, phase, bint exhaustive=False) except -2:
        """CurveSolver.fixed: the production fixed-load point solve (em_stationary.solve)."""
        global g_sink
        cdef int sid
        if self.check is not None:self.check()
        self.eng.flush_counts()
        before = self.ledger.snapshot()
        initial = None
        if anchor >= 0:
            attitude = [self.eng.st[anchor].sol[i] for i in range(5)]
            attitude[1] += (acos(1./load)-acos(1./pmax(1., self.eng.st[anchor].load)))*RAD_TO_DEG
            initial = self.eng.initial_dict(anchor, attitude)
        sid = self.eng.points_().solve(speed, load, initial)
        self.admit_transient(sid)
        self.eng.flush_counts()
        self.ledger.charge(phase, before, 'admitted' if self.eng.st[sid].valid else
                           'rejected' if self.balanced(sid) else 'unresolved',
                           kind='fixed_load', speed_kmh=speed, load_g=load, exhaustive=exhaustive or None)
        return sid

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef tuple seed_from(self, double speed, double load, list anchors, list attitude):
        """CurveSolver.seed_from: (anchor state, solution) of the seed dict (its load is ``load``)."""
        cdef int anchor = anchors[0], a
        cdef double best0 = fabs(self.eng.st[anchor].speed-speed), best1 = fabs(self.eng.st[anchor].load-load), d0, d1
        for a in anchors[1:]:
            d0 = fabs(self.eng.st[a].speed-speed);d1 = fabs(self.eng.st[a].load-load)
            if d0 < best0 or (d0 == best0 and d1 < best1):
                anchor = a;best0 = d0;best1 = d1
        if attitude is None:attitude = [self.eng.st[anchor].sol[i] for i in range(5)]
        attitude = list(attitude)
        attitude[1] = self.eng.st[anchor].bank
        attitude[1] += nominal_bank(load)-nominal_bank(self.eng.st[anchor].load)
        return anchor, attitude

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef tuple direct(self, double speed, double target, int anchor, list attitude, double load, phase):
        """CurveSolver.direct: (outcome, point, reason) of one SEP-constrained correction."""
        cdef double sol[5]
        cdef int i, p
        if self.check is not None:self.check()
        cdef double t0 = s5_now()
        self.eng.flush_counts()
        before = self.ledger.snapshot()
        self.eng.t_ledger += s5_now()-t0
        for i in range(5):sol[i] = attitude[i]
        cdef PKey key
        cdef TState* t = &self.eng.st[anchor]
        cdef Predictor* got
        key.fixed = self.eng.cc.fixed_index(FIX_SEP);key.nreq = t.nreq
        for i in range(t.nreq):key.req[i] = t.req[i]
        if self.reuse:
            slot = self.pred_of.get(self.eng.st[anchor].base)
            if slot is not None:pm_set(&self.preds, &key, &self.pool[<int>slot])
        p = self.eng.correct(anchor, sol, load, True, FIX_SEP, target, 0, 0, 0, self.max_nfev, 0, self.allowance, speed,
                             &self.preds if self.reuse else NULL, NULL)
        if self.reuse and phase in ('interpolation_checks', 'column_roots') and (p < 0 or (not self.eng.st[p].valid and not self.balanced(p))
                           or (self.eng.st[p].valid and fabs(self.eng.st[p].ps-target) > self.allowance)):
            # a predicted linearization that does not close: once more from a fresh Jacobian
            for i in range(5):sol[i] = attitude[i]
            p = self.eng.correct(anchor, sol, load, True, FIX_SEP, target, 0, 0, 0, self.max_nfev, 0, self.allowance,
                                 speed, NULL, NULL)
            self.eng.stats['curve_predictor_retries'] = self.eng.stats.get('curve_predictor_retries', 0)+1
        if p >= 0:self.admit_transient(p)
        if self.reuse and p >= 0:
            got = pm_get(&self.preds, &key)
            if got != NULL and got.valid:
                if self.owners[self.next_pool] is not None:self.pred_of.pop(self.owners[self.next_pool], None)
                self.pool[self.next_pool] = got[0];self.pred_of[self.eng.st[p].base] = self.next_pool
                self.owners[self.next_pool] = self.eng.st[p].base
                self.next_pool = (self.next_pool+1) % self.npool
        self.eng.flush_counts()
        if p < 0:outcome, reason = 'unresolved', 'correction failed'
        elif self.eng.st[p].valid and fabs(self.eng.st[p].ps-target) <= self.allowance:outcome, reason = 'admitted', None
        elif not self.eng.st[p].valid and self.balanced(p) and self.s9 and not self.exit_evidence(p):
            # S9: rejected only because a sub-solve of the admission did not close (Instructor static trim, pitch
            # response, propulsion): numerical, no evidence that the level leaves the envelope here
            outcome, reason = 'unresolved', ['numerical rejection']+self.eng.reason_list(p)
        elif not self.eng.st[p].valid and self.balanced(p):outcome, reason = 'rejected', self.eng.reason_list(p)
        elif not self.eng.st[p].valid:outcome, reason = 'unresolved', ['unclosed']+self.eng.reason_list(p)
        else:outcome, reason = 'unresolved', 'SEP constraint did not close'
        t0 = s5_now()
        self.ledger.charge(phase, before, outcome, kind='direct', speed_kmh=speed, target_mps=target,
                           seed_load_g=load, reason=reason)
        self.eng.t_ledger += s5_now()-t0
        if self.trace is not None:
            self.trace.append((phase, speed, target, anchor, self.eng.st[anchor].speed, load, outcome, p, reason))
        return outcome, (None if p < 0 else p), reason

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int rescue_direct(self, double speed, double target, int anchor, list attitude, double load, phase) except -2:
        """S8 (item 3): a stretch of a curve that does not solve: the sliding state where the anchor's blade polar jump
        blocks its torque balance at the target, then the S6 corrections (governor coordinates on their regime boundary,
        a larger budget). An admitted state at the level compatible with the anchor, or -1."""
        cdef SlideSpec spec
        cdef double sol[5]
        cdef int p = -1, i, rs
        cdef double offset
        cdef NMarch m
        if not self.s8:return -1
        for i in range(5):sol[i] = attitude[i]
        if self.eng.sliding and self.eng.is_prop:
            self.eng.slide_candidate(anchor, &spec, sol[0], sol[1], load, True, speed, True)
            if spec.n and self.eng.st[anchor].slide.n > 0 and self.eng.same_slide(&spec, &self.eng.st[anchor].slide):spec.n = 0
            # S9: from a sliding anchor whose jump no longer blocks a shaft at the target, the regular correction (the
            # shafts leave the jump on the side their theta is nearer)
            if spec.n or (self.s9 and self.eng.st[anchor].slide.n > 0):
                self.eng.slide_override = spec;self.eng.use_override = True
                try:
                    p = self.eng.correct(anchor, sol, load, True, FIX_SEP, target, 0, 0, 0, ROBUST_NFEV, 0, self.allowance, speed,
                                         NULL, NULL)
                finally:
                    self.eng.use_override = False
                if self.ok_root(p, anchor, target):
                    self.eng.stats['s8_curve_slides'] = self.eng.stats.get('s8_curve_slides', 0)+1
                    return p
        m = NMarch(self.eng, speed, self.turn_tol)
        for offset in (0., .01):
            rs = m.regime_seed(anchor, offset)
            if rs < 0:break
            p = self.eng.correct(rs, sol, load, True, FIX_SEP, target, 0, 0, 0, ROBUST_NFEV, 0, self.allowance, speed, NULL, NULL)
            if self.ok_root(p, anchor, target):
                self.eng.stats['s8_curve_regime'] = self.eng.stats.get('s8_curve_regime', 0)+1
                return p
        p = self.eng.correct(anchor, sol, load, True, FIX_SEP, target, 0, 0, 0, ROBUST_NFEV, 0, self.allowance, speed, NULL, NULL)
        if self.ok_root(p, anchor, target):
            self.eng.stats['s8_curve_budget'] = self.eng.stats.get('s8_curve_budget', 0)+1
            return p
        self.eng.flush_counts()
        return -1

    def debug_direct(self, double speed, double target, int anchor, double load, attitude=None):
        """Diagnostics (S9): CurveSolver.direct from ``anchor`` after run(): (outcome, state, reason)."""
        cdef int i
        if attitude is None:attitude = [self.eng.st[anchor].sol[i] for i in range(5)]
        return self.direct(speed, target, anchor, list(attitude), load, 'debug')

    def debug_rescue(self, double speed, double target, int anchor, double load):
        cdef int i
        attitude = [self.eng.st[anchor].sol[i] for i in range(5)]
        return self.rescue_direct(speed, target, anchor, attitude, load, 'debug')

    cdef int edge_root(self, int k, double v_out, double target, bint full=False) except -2:
        """S8: where a curve leaves the envelope between knot ``k`` and the rejected speed ``v_out``: bisection in speed
        (SEP-fixed corrections from the last admitted state) to the speed tolerance; the last admitted state, or -1.
        S9 (``full``): each step solves the level as the split does (solve_level: the S8 rescue and the alpha secant too),
        and a step that meets a physical rejection at the level sets edge_exit."""
        cdef int good = k, q
        cdef double lo = self.eng.st[k].speed, hi = v_out, mid, w
        while fabs(hi-lo) > self.speed_tol:
            mid = .5*(lo+hi)
            if full:
                q = self.solve_level(mid, target, good, -1, 'interpolation_checks')
                if q >= 0:
                    lo = mid;good = q
                else:
                    if self.exit_seen:self.edge_exit = True
                    hi = mid
                continue
            w = pow(self.eng.st[good].load, 2.)-1.
            outcome, q2, reason = self.direct_seed(mid, target, sqrt(1.+pmax(w, 1e-6)), [good], None, 'interpolation_checks')
            if outcome == 'admitted' and q2 is not None and self.eng.compatible(q2, good):
                lo = mid;good = q2
            else:hi = mid
        return good if good != k else -1

    cdef bint ok_root(self, int p, int anchor, double target) except -1:
        if p < 0:return False
        self.admit_transient(p)
        return self.eng.st[p].valid and fabs(self.eng.st[p].ps-target) <= self.allowance and self.eng.compatible(p, anchor)

    # ------------------------------------------------------------------ S9: level solves that do not close
    cdef bint exit_evidence(self, int p) except -1:
        """S9: a state showing that the level is not reached inside the envelope there: a balanced rejection by physical
        limits only (no sub-solve of the admission that did not close)."""
        cdef TState* t
        cdef int i
        if p < 0:return False
        t = &self.eng.st[p]
        if t.valid or t.nreasons == 0 or not self.balanced(p):return False
        for i in range(t.nreasons):
            if numerical_code(t.reasons[i]):return False
        return True

    cdef int alpha_correct(self, int seed, double speed, double alpha) except -2:
        """S9: one prescribed-alpha correction at ``speed`` from ``seed`` (with the shafts sliding where the jump check at
        the target confirms a blade-polar jump and the regular correction does not close; from a sliding seed, the
        shafts the check releases regular)."""
        cdef double sol[5]
        cdef int p, i
        cdef SlideSpec spec
        for i in range(5):sol[i] = self.eng.st[seed].sol[i]
        sol[0] = alpha
        p = self.eng.correct(seed, sol, self.eng.st[seed].load, True, FIX_ALPHA, alpha, 0, 0, 0, self.max_nfev, 0, 2e-6, speed,
                             NULL, NULL)
        if (p < 0 or not (self.eng.st[p].valid or self.balanced(p))) and self.eng.sliding and self.eng.is_prop:
            self.eng.slide_candidate(seed, &spec, alpha, sol[1], self.eng.st[seed].load, True, speed, True)
            if spec.n and self.eng.st[seed].slide.n > 0 and self.eng.same_slide(&spec, &self.eng.st[seed].slide):spec.n = 0
            if spec.n or self.eng.st[seed].slide.n > 0:
                self.eng.slide_override = spec;self.eng.use_override = True
                try:
                    p = self.eng.correct(seed, sol, self.eng.st[seed].load, True, FIX_ALPHA, alpha, 0, 0, 0, ROBUST_NFEV, 0, 2e-6,
                                         speed, NULL, NULL)
                finally:
                    self.eng.use_override = False
        if p >= 0:self.admit_transient(p)
        return p

    cdef int alpha_secant(self, double speed, double target, int a, int b, phase) except -2:
        """S9: the level at ``speed`` where its SEP-fixed correction does not close (next to a discontinuity of the model
        such as a polar's jump, the corrector's steps cross it): prescribed-alpha corrections at ``speed`` continued from
        the knot nearer in speed (alpha first interpolated between the knots), alpha moved by the secant of their SEP on
        the level until one is admitted within the SEP allowance of it (the acceptance of every curve solve); a correction
        that does not close is moved 0.005 deg once. The admitted state compatible with the knots, or -1; exit_seen when
        the level lies where a physical limit rejects the state (beyond the envelope)."""
        cdef int seed, p, k, anchor
        cdef double al, al0 = NAN, ps0 = NAN, ps, step, t
        anchor = a if b < 0 or fabs(self.eng.st[a].speed-speed) <= fabs(self.eng.st[b].speed-speed) else b
        if b >= 0 and self.eng.st[b].speed != self.eng.st[a].speed:
            t = (speed-self.eng.st[a].speed)/(self.eng.st[b].speed-self.eng.st[a].speed)
            al = self.eng.st[a].alpha+t*(self.eng.st[b].alpha-self.eng.st[a].alpha)
        else:
            al = self.eng.st[anchor].alpha
        self.eng.flush_counts();before = self.ledger.snapshot()
        seed = anchor;out = -1
        for k in range(S9_SECANT_STEPS):
            p = self.alpha_correct(seed, speed, al)
            if p < 0 or not self.eng.st[p].valid:
                if p >= 0 and self.exit_evidence(p) and isfinite(ps0) and ps0 > target:
                    self.exit_seen = True;self.exit_state = p;break
                al += .005
                p = self.alpha_correct(seed, speed, al)
                if p < 0 or not self.eng.st[p].valid:
                    if p >= 0 and self.exit_evidence(p) and isfinite(ps0) and ps0 > target:
                        self.exit_seen = True;self.exit_state = p
                    break
            if not (self.eng.compatible(p, a) and (b < 0 or self.eng.compatible(p, b))):break
            ps = self.eng.st[p].ps
            if not isfinite(ps):break
            if fabs(ps-target) <= self.allowance:
                out = p;break
            if isfinite(ps0) and ps != ps0:step = -(ps-target)*(al-al0)/(ps-ps0)
            else:step = .05 if ps > target else -.05
            step = pmin(3., pmax(-3., step))
            al0 = al;ps0 = ps;seed = p;al = al+step
        self.eng.flush_counts()
        self.ledger.charge(phase, before, 'admitted' if out >= 0 else 'unresolved', kind='alpha secant', speed_kmh=speed,
                           target_mps=target)
        key = 's9_alpha_secant_solved' if out >= 0 else 's9_alpha_secant_failed'
        self.eng.stats[key] = self.eng.stats.get(key, 0)+1
        return out

    cdef int solve_level(self, double speed, double target, int a, int b, phase) except -2:
        """S9: one admitted state of the level at ``speed`` next to knot ``a`` (between knots a and b where b >= 0): the
        SEP-fixed correction from the attitude interpolated between the knots, then from each knot alone, then the S8
        rescue (sliding, regime seeds, budget), then the alpha secant. The state compatible with the knots, or -1;
        exit_seen when a physical rejection lies at the level."""
        cdef double t, w, wa, n
        cdef int q, i
        self.exit_seen = False
        anchors = [a] if b < 0 else [a, b]
        wa = pow(self.eng.st[a].load, 2.)-1.
        if b >= 0 and self.eng.st[b].speed != self.eng.st[a].speed:
            t = (speed-self.eng.st[a].speed)/(self.eng.st[b].speed-self.eng.st[a].speed)
            w = wa+t*((pow(self.eng.st[b].load, 2.)-1.)-wa)
            attitude = [self.eng.st[a].sol[i]*(1-t)+self.eng.st[b].sol[i]*t for i in range(5)]
        else:
            w = wa;attitude = None
        n = sqrt(1.+pmax(w, 1e-6))
        outcome, q2, reason = self.direct_seed(speed, target, n, anchors, attitude, phase)
        if outcome == 'admitted' and all(self.eng.compatible(q2, x) for x in anchors):return q2
        if outcome == 'rejected' and q2 is not None and self.exit_evidence(q2):
            self.exit_seen = True;self.exit_state = q2
        if b >= 0:
            for anchor in anchors:
                w = pow(self.eng.st[anchor].load, 2.)-1.
                o2, q2, _ = self.direct_seed(speed, target, sqrt(1.+pmax(w, 1e-6)), [anchor], None, phase)
                if o2 == 'admitted' and all(self.eng.compatible(q2, x) for x in anchors):return q2
                if o2 == 'rejected' and q2 is not None and self.exit_evidence(q2):
                    self.exit_seen = True;self.exit_state = q2
        if self.exit_seen:return -1
        self.eng.flush_counts();before = self.ledger.snapshot()
        q = self.rescue_direct(speed, target, a, attitude if attitude is not None else [self.eng.st[a].sol[i] for i in range(5)],
                               n, phase)
        if q >= 0 and b >= 0 and not self.eng.compatible(q, b):q = -1
        self.eng.flush_counts()
        self.ledger.charge(phase, before, 'admitted' if q >= 0 else 'unresolved', kind='rescue', speed_kmh=speed, target_mps=target)
        if q >= 0:return q
        return self.alpha_secant(speed, target, a, b, phase)

    cdef bint envelope_exit(self, double lo_v, double hi_v, double target, int anchor) except -1:
        """S9: whether the level lies beyond the envelope boundary in the stretch (lo_v, hi_v) between two located curve
        ends: every verified envelope column inside it, away from the ends' speed-tolerance brackets (where the exit
        lies, so a column there can still show the level), has its boundary SEP above the level (the level is not
        reached up to the boundary there); without such a column, the boundary of a column computed at the middle of the
        stretch (the alpha march from level flight on the curve engine) decides."""
        cdef int one, boundary
        cdef double mid = .5*(lo_v+hi_v)
        cdef NMarch m
        inside = [ps for v, ps in zip(self.env_v, self.env_ps) if lo_v+self.speed_tol < v < hi_v-self.speed_tol and ps is not None]
        if inside:return all(ps > target for ps in inside)
        self.eng.flush_counts();before = self.ledger.snapshot()
        out = False
        one = self.fixed(mid, 1., anchor, 'interpolation_checks')
        if one >= 0 and self.eng.st[one].valid:
            m = NMarch(self.eng, mid, self.turn_tol)
            boundary, status, reason = m.run(one, False, 0.)
            if boundary >= 0:boundary, status, reason = m.settle((boundary, status, reason))
            if boundary >= 0 and status in ('verified limit', 'plot ceiling') and isfinite(self.eng.st[boundary].ps):
                out = self.eng.st[boundary].ps > target+self.allowance
        self.eng.flush_counts()
        self.ledger.charge('interpolation_checks', before, 'admitted' if out else 'unresolved', kind='exit column', speed_kmh=mid,
                           target_mps=target)
        self.eng.stats['s9_exit_columns'] = self.eng.stats.get('s9_exit_columns', 0)+1
        return out

    cdef int failed_piece(self, dict comp, int a, int b, double speed, bint exit, reason, list unresolved, list additions) except -1:
        """S9: a piece whose split solve found no admitted state (every S8 retry failed). Where the level lies beyond the
        envelope there (a physical rejection at the level, or the verified envelope columns either side of the split
        speed have their boundary SEP above the level), the curve leaves the envelope: both ends located as S8 does.
        Otherwise the failure is numerical: the alpha secant at the split speed, then other split speeds inside the
        piece (a failure at one speed only: the Instructor's static trim does not close at isolated states), each solved
        as the split is (solve_level); then the failing stretch narrowed from both knots to the speed tolerance with the
        same solves (edge_root). The stretch is filed as an exit where a physical rejection lies at the level or the
        envelope's boundary SEP is above it (envelope_exit), otherwise as numerical; one narrower than the speed
        tolerance between compatible ends is a piece like any other (the split rule finishes it; S8's step rule draws it
        if its bound fails)."""
        cdef double target = comp['level'], va = self.eng.st[a].speed, vb = self.eng.st[b].speed, h = vb-va, x, f, lo_v, hi_v
        cdef int q, ea, eb
        cdef bint numerical
        if not exit:
            import bisect
            i = bisect.bisect_left(self.env_v, speed)
            near = [self.env_ps[k] for k in (i-1, i) if 0 <= k < len(self.env_v)]
            exit = len(near) == 2 and all(ps is not None and ps > target for ps in near)
        numerical = not exit
        if numerical:
            self.exit_seen = False
            q = self.alpha_secant(speed, target, a, b, 'interpolation_checks')
            if q >= 0:
                additions.append(q);return 0
            if self.exit_seen:exit = True
        if not exit:
            for f in (.3, .7, .15, .85):
                x = va+f*h
                if fabs(x-speed) <= .05*h:continue
                q = self.solve_level(x, target, a, b, 'interpolation_checks')
                if q >= 0:
                    additions.append(q)
                    self.eng.stats['s9_curve_nudges'] = self.eng.stats.get('s9_curve_nudges', 0)+1
                    return 0
                if self.exit_seen:
                    exit = True;speed = x;break
        self.edge_exit = False
        # an exit is located with the S8 bisection (plain solves); a numerical stretch with the full solves
        ea = self.edge_root(a, speed, target, numerical and not exit);eb = self.edge_root(b, speed, target, numerical and not exit)
        exit = exit or self.edge_exit
        lo_v = self.eng.st[ea].speed if ea >= 0 else va
        hi_v = self.eng.st[eb].speed if eb >= 0 else vb
        if ea >= 0:additions.append(ea)
        if eb >= 0:additions.append(eb)
        if not exit:exit = self.envelope_exit(lo_v, hi_v, target, ea if ea >= 0 else a)
        if not exit and hi_v-lo_v <= self.speed_tol*(1.+1e-9) and self.eng.compatible(ea if ea >= 0 else a, eb if eb >= 0 else b):
            self.eng.stats['s9_curve_narrow'] = self.eng.stats.get('s9_curve_narrow', 0)+1
            if ea < 0 and eb < 0:
                # no knot added: the piece itself is narrower than the speed tolerance (finished as a narrow piece)
                comp.setdefault('checks', []).append(dict(interval_kmh=[va, vb], speed_kmh=None, passed=True, route='narrow'))
            return 0
        unresolved.append(dict(interval_kmh=[lo_v, hi_v], reason='level beyond the envelope boundary' if exit else reason,
                               outcome='excluded' if exit else 'unresolved'))
        key = 's8_curve_exits' if exit else 's8_curve_narrowed'
        self.eng.stats[key] = self.eng.stats.get(key, 0)+1
        return 0

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef tuple direct_seed(self, double speed, double target, double n, list anchors, list attitude, phase):
        anchor, attitude = self.seed_from(speed, n, anchors, attitude)
        return self.direct(speed, target, anchor, attitude, n, phase)

    # ------------------------------------------------------------------ discovery
    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef dict ladder(self, double speed, phase='seed_discovery'):
        cdef double cap = self.eng.cap(speed)
        cdef int p
        loads = [n for n in self.ladder_loads if n < cap-1e-9]+[cap]
        points, stop = [], None
        for n in loads:
            p = self.fixed(speed, n, points[-1] if points else -1, phase)
            if not self.eng.st[p].valid:
                # S9: a rejection is physical only without a numerical reason (exit_evidence)
                stop = dict(load_g=n, kind='rejected' if (self.exit_evidence(p) if self.s9 else self.balanced(p)) else 'unresolved',
                            reasons=self.eng.reason_list(p))
                break
            points.append(p)
        return dict(speed_kmh=speed, points=points, stop=stop, cap_g=cap)

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef list ladder_roots(self, dict ladder, double target, phase='seed_discovery'):
        cdef int a, b, i
        cdef double t, w, n, f, slope
        pts = ladder['points'];speed = ladder['speed_kmh'];found = []
        for i in range(len(pts)-1):
            a = pts[i];b = pts[i+1]
            if (self.eng.st[a].ps-target)*(self.eng.st[b].ps-target) > 0:continue
            hit = None
            for q in (a, b):
                if fabs(self.eng.st[q].ps-target) <= self.allowance:
                    hit = q;break
            if hit is not None:
                found.append(dict(outcome='admitted', point=hit, bracket=[self.eng.st[a].load, self.eng.st[b].load]))
                continue
            t = (target-self.eng.st[a].ps)/(self.eng.st[b].ps-self.eng.st[a].ps)
            w = (pow(self.eng.st[a].load, 2.)-1.)+t*((pow(self.eng.st[b].load, 2.)-1.)-(pow(self.eng.st[a].load, 2.)-1.))
            n = py_sqrt(1.+w)
            f = (n-self.eng.st[a].load)/(self.eng.st[b].load-self.eng.st[a].load)
            attitude = [self.eng.st[a].sol[k]*(1-f)+self.eng.st[b].sol[k]*f for k in range(5)]
            outcome, point, reason = self.direct_seed(speed, target, n, [a, b], attitude, phase)
            if outcome == 'admitted' and not (self.eng.st[a].load-1e-9 <= self.eng.st[point].load <= self.eng.st[b].load+1e-9):
                outcome, reason = 'unresolved', 'root left observed bracket'
            if outcome == 'admitted' and not (self.eng.compatible(point, a) or self.eng.compatible(point, b)):
                outcome, reason = 'unresolved', 'corrected branch changed'
            found.append(dict(outcome=outcome, point=point, reason=reason, bracket=[self.eng.st[a].load, self.eng.st[b].load]))
        if len(pts) >= 2 and ladder['stop'] is not None and self.eng.st[pts[-1]].ps > target+self.allowance:
            a, b = pts[-2], pts[-1]
            slope = (self.eng.st[b].ps-self.eng.st[a].ps)/(pow(self.eng.st[b].load, 2.)-pow(self.eng.st[a].load, 2.))
            if slope < 0:
                w = pow(self.eng.st[b].load, 2.)-1.+(target-self.eng.st[b].ps)/slope
                n = pmin(py_sqrt(1.+pmax(w, 0.)), ladder['stop']['load_g'])
                outcome, point, reason = self.direct_seed(speed, target, n, [b], None, phase)
                if outcome == 'admitted' and not (self.eng.st[b].load <= self.eng.st[point].load <= ladder['stop']['load_g']+1e-9
                                                  and self.eng.compatible(point, b)):
                    outcome, reason = 'unresolved', 'root outside top interval or branch changed'
                found.append(dict(outcome=outcome, point=point, reason=reason, top_interval=True,
                                  bracket=[self.eng.st[b].load, ladder['stop']['load_g']]))
        return found

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef dict bracketed(self, double speed, double target, int a, int b, phase):
        cdef double t, n, f
        if fabs(self.eng.st[b].ps-target) <= self.allowance:
            return dict(outcome='admitted', point=b, bracket=[self.eng.st[a].load, self.eng.st[b].load])
        t = (target-self.eng.st[a].ps)/(self.eng.st[b].ps-self.eng.st[a].ps)
        n = py_sqrt(1.+(pow(self.eng.st[a].load, 2.)-1.)+t*(pow(self.eng.st[b].load, 2.)-pow(self.eng.st[a].load, 2.)))
        f = (n-self.eng.st[a].load)/(self.eng.st[b].load-self.eng.st[a].load)
        attitude = [self.eng.st[a].sol[k]*(1-f)+self.eng.st[b].sol[k]*f for k in range(5)]
        outcome, point, reason = self.direct_seed(speed, target, n, [a, b], attitude, phase)
        if outcome == 'admitted' and not (self.eng.st[a].load-1e-9 <= self.eng.st[point].load <= self.eng.st[b].load+1e-9
                                          and self.eng.compatible(point, a)):
            outcome, reason = 'unresolved', 'root left observed bracket or branch changed'
        return dict(outcome=outcome, point=point, reason=reason, bracket=[self.eng.st[a].load, self.eng.st[b].load])

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int one_g(self, double speed, phase='seed_discovery', int anchor=-1) except -2:
        return self.fixed(speed, 1., anchor, phase)

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef object locate_one_g(self, double target, int above, int below, phase='one_g_discovery'):
        cdef int a = above, b = below, p
        cdef double fa, fb, lo, hi, mid
        if self.s9:
            # S9: the 1 g end of a contour to a quarter of the speed tolerance (or a tenth of the SEP allowance): where
            # SEP is flat along level flight the allowance alone places it km/h off (Stirling SB +10 m/s: 3.9 km/h)
            return self.one_g_root(target, above, below, phase)
        while fabs(self.eng.st[b].speed-self.eng.st[a].speed) > self.speed_tol:
            fa = self.eng.st[a].ps-target;fb = self.eng.st[b].ps-target
            lo = self.eng.st[a].speed;hi = self.eng.st[b].speed
            mid = lo+(hi-lo)*fa/(fa-fb)
            if not (pmin(lo, hi)+.1*fabs(hi-lo) < mid and mid < pmax(lo, hi)-.1*fabs(hi-lo)):mid = .5*(lo+hi)
            p = self.fixed(mid, 1., a, phase)
            if not self.eng.st[p].valid or not self.eng.compatible(p, a):
                return None
            if fabs(self.eng.st[p].ps-target) <= self.allowance:return p
            if self.eng.st[p].ps > target:a = p
            else:b = p
        return a

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef object one_g_root(self, double target, int above, int below, phase, int avoid=-1):
        """S9: the level-flight speed where SEP equals ``target`` between the 1 g states ``above`` (SEP over it) and
        ``below``: Illinois regula falsi to S9_END_SEP x the allowance in SEP or S9_END_SPEED x the speed tolerance in
        speed; the state nearer the target. None where a sample is not admitted or changes branch. S10: never the state
        ``avoid`` while the bracket is wider (level flight at a contour's last knot: an end there leaves no knot to draw
        it by)."""
        cdef int a = above, b = below, p, it, last_side = 0
        cdef double fa = self.eng.st[above].ps-target, fb = self.eng.st[below].ps-target, lo, hi, mid
        for it in range(S9_END_STEPS):
            lo = self.eng.st[a].speed;hi = self.eng.st[b].speed
            self.root_dsdv = (self.eng.st[b].ps-self.eng.st[a].ps)/(hi-lo) if hi != lo else NAN
            if fabs(fa) <= S9_END_SEP*self.allowance and a != avoid:return a
            if fabs(fb) <= S9_END_SEP*self.allowance and b != avoid:return b
            if fabs(hi-lo) <= S9_END_SPEED*self.speed_tol:break
            mid = lo+(hi-lo)*fa/(fa-fb) if fa != fb else .5*(lo+hi)
            if not (pmin(lo, hi) < mid < pmax(lo, hi)):mid = .5*(lo+hi)
            p = self.fixed(mid, 1., a, phase)
            if not self.eng.st[p].valid or not self.eng.compatible(p, a):
                return None
            self.eng.stats['s9_end_solves'] = self.eng.stats.get('s9_end_solves', 0)+1
            if self.eng.st[p].ps > target:
                a = p;fa = self.eng.st[p].ps-target
                if last_side > 0:fb *= .5
                last_side = 1
            else:
                b = p;fb = self.eng.st[p].ps-target
                if last_side < 0:fa *= .5
                last_side = -1
        if a == avoid:return b
        if b == avoid:return a
        return a if fabs(self.eng.st[a].ps-target) <= fabs(self.eng.st[b].ps-target) else b

    # ------------------------------------------------------------------ continuation
    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef tuple predict(self, dict comp, double speed, int side):
        cdef int a, b
        cdef double wa, wb, r
        knots = comp['knots'] if side > 0 else comp['knots'][::-1]
        a = knots[-1];b = knots[-2] if len(knots) > 1 else -1
        wa = pow(self.eng.st[a].load, 2.)-1.
        if b < 0 or self.eng.st[b].speed == self.eng.st[a].speed:
            return wa+comp.get('w_slope', 0.)*(speed-self.eng.st[a].speed), [self.eng.st[a].sol[i] for i in range(5)]
        r = (speed-self.eng.st[a].speed)/(self.eng.st[a].speed-self.eng.st[b].speed)
        wb = pow(self.eng.st[b].load, 2.)-1.
        attitude = [self.eng.st[a].sol[i]+r*(self.eng.st[a].sol[i]-self.eng.st[b].sol[i]) for i in range(5)]
        return wa+r*(wa-wb), attitude

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef dict one_g_endpoint(self, dict comp, double target, int inside, int outside, int side):
        cdef double lo = self.eng.st[inside].speed, hi = self.eng.st[outside].speed, mid, guess, f_out
        cdef int a_in = inside, a_out = outside, p
        cdef bint has_in
        cdef double f_in = 0., sgn = self.sense(comp)
        if self.s9:
            # S9: the 1 g end to a quarter of the speed tolerance (one_g_root), from a 1 g state at the last knot
            # (S10: on a 'below' branch level flight lies below the level inside the contour: the comparisons turn round)
            if self.eng.st[a_in].load != 1.:
                p = self.fixed(lo, 1., a_out, 'endpoint')
                if self.eng.st[p].valid and self.eng.compatible(p, a_out) and sgn*(self.eng.st[p].ps-target) > 0.:a_in = p
                elif (self.s10 and self.eng.st[p].valid and self.eng.compatible(p, a_out) and self.eng.compatible(p, inside)
                      and sgn > 0.):
                    # S10: level flight at the last knot's speed is outside the level too, so no 1 g end lies between:
                    # where the level has a second root below that knot (SEP rises above the level between level flight
                    # and the knot), the contour turns back in speed here (fold_turn)
                    end = self.fold_turn(comp, inside, side, p)
                    if end is not None:return end
            if self.eng.st[a_in].load == 1.:
                # (S10: not at level flight at the last knot's own speed while the bracket is wider: Sunderland Mk5 SB
                # +10 m/s, where SEP along level flight is flat, the S9 SEP test took it 0.12 km/h from the end)
                avoid = a_in if self.s10 and a_in != inside else -1
                r_ = (self.one_g_root(target, a_in, a_out, 'endpoint', avoid) if sgn > 0. else
                      self.one_g_root(target, a_out, a_in, 'endpoint', avoid))
                if r_ is not None:
                    p = r_
                    self.end_slope(p, side)
                    return dict(kind='one_g', speed_kmh=self.eng.st[p].speed, state=p,
                                sep_residual_mps=self.eng.st[p].ps-target)
            a_in = inside
        while fabs(hi-lo) > self.speed_tol:
            has_in = self.eng.st[a_in].load == 1.
            if has_in:f_in = self.eng.st[a_in].ps-target
            f_out = self.eng.st[a_out].ps-target
            mid = .5*(lo+hi)
            if has_in and f_in != f_out:
                guess = lo+(hi-lo)*f_in/(f_in-f_out)
                if pmin(lo, hi)+.1*fabs(hi-lo) < guess and guess < pmax(lo, hi)-.1*fabs(hi-lo):mid = guess
            p = self.fixed(mid, 1., a_out, 'endpoint')
            if not self.eng.st[p].valid:
                return dict(kind='unresolved', speed_kmh=[lo, hi], reason='1 g sample not admitted',
                            reasons=self.eng.reason_list(p))
            if fabs(self.eng.st[p].ps-target) <= self.allowance:
                return dict(kind='one_g', speed_kmh=mid, state=p)
            if sgn*(self.eng.st[p].ps-target) > 0.:
                lo = mid;a_in = p
            else:
                hi = mid;a_out = p
        p = a_in if self.eng.st[a_in].load == 1. else a_out
        return dict(kind='one_g', speed_kmh=self.eng.st[p].speed, state=p, located_to_kmh=abs(hi-lo),
                    sep_residual_mps=self.eng.st[p].ps-target)

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef object march(self, dict comp, dict segment, int side):
        cdef double target = comp['level'], bound, step, speed, w, n, error, h
        cdef int last, one
        lo, hi = segment['speed_kmh']
        bound = hi if side > 0 else lo
        bound_kind = segment['ends'][1 if side > 0 else 0]
        step = self.max_step
        # S10: a branch where SEP rises with load (a contour turning back) is short and steep next to its fold: it is
        # followed from there in steps of at most twice the speed tolerance at first, and every knot must stay on it
        below = self.s10 and bool(comp.get('below'))
        if below:step = pmin(self.max_step, 2.*self.speed_tol)
        failure = None
        while True:
            last = comp['knots'][-1] if side > 0 else comp['knots'][0]
            if abs(bound-self.eng.st[last].speed) <= 1e-9:
                return dict(kind=bound_kind, speed_kmh=bound)
            speed = self.eng.st[last].speed+side*min(step, abs(bound-self.eng.st[last].speed))
            between = [v for v in self.knot_speeds if side*(v-self.eng.st[last].speed) > self.speed_tol and
                       side*(speed-v) > 1e-9]
            if between:speed = min(between, key=lambda v: abs(v-self.eng.st[last].speed))
            w, attitude = self.predict(comp, speed, side)
            if below and w > 0.:
                # S10: level flight at every step of a 'below' branch (the slope found next to its fold is a poor guide to
                # where it meets 1 g): outside the level there, the 1 g end lies within the step
                one = self.fixed(speed, 1., last, 'endpoint')
                if self.eng.st[one].valid and self.sense(comp)*(self.eng.st[one].ps-target) < 0.:
                    end = self.one_g_endpoint(comp, target, last, one, side)
                    if end.get('state') is not None and end['kind'] == 'one_g':
                        self.add_knot(comp, end['state'], side)
                    return end
            if w <= 0. and self.eng.st[last].load == 1. and abs(self.eng.st[last].ps-target) <= self.allowance:
                if self.s9 and abs(self.eng.st[last].ps-target) > S9_END_SEP*self.allowance:
                    # S9: a 1 g knot within the allowance is not yet the end: the end to a quarter of the speed tolerance
                    end = self.refine_end(comp, target, last, speed, side)
                    if end is not None:return end
                return dict(kind='one_g', speed_kmh=self.eng.st[last].speed, state=last)
            if w <= 0.:
                one = self.fixed(speed, 1., last, 'endpoint')
                if self.eng.st[one].valid and self.sense(comp)*(self.eng.st[one].ps-target) < 0.:
                    end = self.one_g_endpoint(comp, target, last, one, side)
                    if end.get('state') is not None and end['kind'] == 'one_g':
                        self.add_knot(comp, end['state'], side)
                    return end
                w = max(w, (pow(self.eng.st[last].load, 2.)-1.)*.25)
            n = py_sqrt(1.+max(w, 1e-6))
            outcome, point, reason = self.direct_seed(speed, target, n, [last], attitude, 'continuation')
            if outcome == 'admitted' and self.eng.compatible(point, last) and (not below or self.rising(point)):
                self.add_knot(comp, point, side)
                error = abs(curve_turn(speed, n)-self.eng.st[point].turn)
                h = abs(speed-self.eng.st[last].speed)
                if error > self.predictor_shrink*self.turn_tol:step = max(.5*h, self.speed_tol)
                elif error <= self.predictor_grow*self.turn_tol:step = min(self.max_step, max(step, h)*1.5)
                else:step = min(self.max_step, max(step, h))
                failure = None
                continue
            if outcome == 'admitted':
                outcome, reason = 'incompatible', 'branch changed during continuation'
            failure = dict(speed_kmh=speed, outcome=outcome, reason=reason, point=point)
            if abs(speed-self.eng.st[last].speed) <= self.speed_tol:
                if self.s8 and outcome != 'rejected':
                    # S8 (item 3): before the curve ends here, sliding on a confirmed blade-polar jump and the S6 retries
                    self.eng.flush_counts();before = self.ledger.snapshot()
                    rescued = self.rescue_direct(speed, target, last, attitude, n, 'continuation')
                    self.eng.flush_counts()
                    self.ledger.charge('continuation', before, 'admitted' if rescued >= 0 else 'unresolved', kind='rescue',
                                       speed_kmh=speed, target_mps=target)
                    if rescued < 0 and self.s9:
                        # S9: the level by prescribed-alpha corrections (a model discontinuity next to the level); a
                        # physical rejection where the level would be ends the curve as an exit
                        self.exit_seen = False
                        rescued = self.alpha_secant(speed, target, last, -1, 'continuation')
                        if rescued < 0 and self.exit_seen:
                            failure['outcome'] = 'rejected';failure['point'] = self.exit_state
                            failure['reason'] = self.eng.reason_list(self.exit_state)
                    if rescued >= 0 and below and not self.rising(rescued):rescued = -1
                    if rescued >= 0:
                        self.add_knot(comp, rescued, side)
                        failure = None
                        continue
                return self.classify_end(comp, last, failure, side)
            step = .5*abs(speed-self.eng.st[last].speed)

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef object classify_end(self, dict comp, int last, dict failure, int side):
        cdef double target = comp['level'], speed = failure['speed_kmh']
        cdef int one
        one = self.fixed(speed, 1., last, 'endpoint')
        if self.eng.st[one].valid and self.sense(comp)*(self.eng.st[one].ps-target) < -self.allowance:
            end = self.one_g_endpoint(comp, target, last, one, side)
            if end.get('state') is not None and end['kind'] == 'one_g':self.add_knot(comp, end['state'], side)
            return end
        if failure['outcome'] == 'rejected':
            return dict(kind='physically_excluded', speed_kmh=[self.eng.st[last].speed, speed],
                        turn_range_dps=sorted([self.eng.st[last].turn, self.eng.st[failure['point']].turn]),
                        reasons=failure['reason'], state=self.compact(failure['point']))
        ladder = self.ladder(speed, 'fallback_discovery')
        roots = [r for r in self.ladder_roots(ladder, target, 'fallback_discovery') if r['outcome'] == 'admitted']
        near = [r for r in roots if self.eng.compatible(r['point'], last)]
        if near:
            r = min(near, key=lambda r: abs(self.eng.st[r['point']].turn-self.eng.st[last].turn))
            if side*(self.eng.st[r['point']].speed-self.eng.st[last].speed) > 1e-9:
                self.add_knot(comp, r['point'], side)
                return None
            return dict(kind='unresolved_numerical', speed_kmh=[self.eng.st[last].speed, speed],
                        reason='fallback root made no progress in speed')
        if roots:
            return dict(kind='branch_change', speed_kmh=[self.eng.st[last].speed, speed],
                        other_roots=[self.compact(r['point']) for r in roots])
        rejected = [r for r in self.ladder_roots(ladder, target, 'fallback_discovery') if r['outcome'] == 'rejected']
        if (ladder['stop'] and ladder['points'] and self.eng.st[ladder['points'][-1]].ps > target
                and ladder['stop']['kind'] == 'rejected'):
            return dict(kind='physically_excluded', speed_kmh=[self.eng.st[last].speed, speed],
                        reasons=ladder['stop']['reasons'], evidence='ladder stopped by physical rejection above the target')
        return dict(kind='unresolved_numerical', speed_kmh=[self.eng.st[last].speed, speed],
                    reason=failure['reason'], rejected_roots=len(rejected))

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef void end_slope(self, int p, int side=0) except *:
        """S9: the exact slope dw/dV of a contour's 1 g end (the knot's tangent is singular there): -(dSEP/dV along
        level flight, one_g_root's bracket) / (dSEP/dw at the end speed, a level-turn solve at w = S9_END_DW). Where SEP
        is far from linear in w near 1 g (Mirage Milan SB +100 m/s near Mach 1: w falls 2.7 in the last 0.7 km/h) the
        split rule then refines the end."""
        cdef int q, base = self.eng.st[p].base
        cdef double dsdw, slope
        if not isfinite(self.root_dsdv) or self.slopes.get(base) is not None and isfinite(self.slopes[base][0]):return
        q = self.fixed(self.eng.st[p].speed, sqrt(1.+S9_END_DW), p, 'endpoint')
        if not (self.eng.st[q].valid and self.eng.compatible(q, p)):return
        dsdw = (self.eng.st[q].ps-self.eng.st[p].ps)/S9_END_DW
        # SEP rising with load at 1 g (a cambered polar near Mach 1: Mirage Milan SB +100 m/s): the level set is no graph
        # over speed next to its 1 g end (it turns back beyond it); the chart draws the end as a vertical drop
        slope = -self.root_dsdv/dsdw if dsdw < 0. else copysign(S9_END_VERTICAL, self.root_dsdv)
        if self.s10 and side != 0 and dsdw != 0. and (-self.root_dsdv/dsdw)*side < 0.:
            # S10: the level set's own slope wherever it agrees with the side the contour lies on (left of a high-speed
            # end it falls to 1 g, right of a low-speed end it rises from it): the 1 g end of a branch where SEP rises
            # with load (a contour that turns back: Do 17 Z-7 RB -40 m/s at 567.8 km/h) is drawn as it is
            slope = -self.root_dsdv/dsdw
        if fabs(slope) > S9_END_VERTICAL:slope = copysign(S9_END_VERTICAL, slope)
        self.slopes[base] = (slope, slope)
        self.eng.stats['s9_end_slopes'] = self.eng.stats.get('s9_end_slopes', 0)+1

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef object refine_end(self, dict comp, double target, int last, double speed, int side):
        """S9: the 1 g end of a contour whose last knot ``last`` is a 1 g state within the allowance: bracketed by a 1 g
        state at the next speed (``last`` over the level) or by level flight at the knot before it (``last`` under it),
        then one_g_root."""
        cdef int one, prev, r
        cdef double sgn = self.sense(comp)
        knots = comp['knots']
        # (S10: on a 'below' branch the sides of the level turn round: sense)
        if sgn*(self.eng.st[last].ps-target) > 0.:
            one = self.fixed(speed, 1., last, 'endpoint')
            if not (self.eng.st[one].valid and self.eng.compatible(one, last) and sgn*(self.eng.st[one].ps-target) < 0.):
                return None
            r_ = self.one_g_root(target, last, one, 'endpoint') if sgn > 0. else self.one_g_root(target, one, last, 'endpoint')
        else:
            if len(knots) < 2:return None
            prev = knots[-2] if side > 0 else knots[1]
            if self.eng.st[prev].load == 1.:
                if sgn*(self.eng.st[prev].ps-target) < 0.:return None
                one = prev
            else:
                one = self.fixed(self.eng.st[prev].speed, 1., last, 'endpoint')
                if not (self.eng.st[one].valid and self.eng.compatible(one, last) and sgn*(self.eng.st[one].ps-target) > 0.):
                    return None
            r_ = self.one_g_root(target, one, last, 'endpoint') if sgn > 0. else self.one_g_root(target, last, one, 'endpoint')
            if r_ is not None:
                if side > 0:knots.pop()
                else:knots.pop(0)
        if r_ is None:return None
        r = r_
        self.eng.stats['s9_refined_ends'] = self.eng.stats.get('s9_refined_ends', 0)+1
        self.end_slope(r, side)
        self.add_knot(comp, r, side)
        return dict(kind='one_g', speed_kmh=self.eng.st[r].speed, state=r, sep_residual_mps=self.eng.st[r].ps-target)

    # ------------------------------------------------------------------ S10: contours that turn back in speed
    cdef double sense(self, dict comp) except? -2.:
        """S10: +1 where SEP falls with load along the component (level flight lies above the level inside the contour, the
        usual branch), -1 on a 'below' branch (SEP rises with load: level flight lies below the level inside it)."""
        return -1. if self.s10 and comp.get('below') else 1.

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef bint rising(self, int p) except -1:
        """S10: whether SEP rises with load at the curve state ``p`` (a fixed-load solve from it at 3% less w, at least 0.005;
        True where none is possible or admitted: no evidence)."""
        cdef double w = self.eng.st[p].load**2-1., dw = pmax(.03*w, .005)
        cdef int q
        if w-dw <= 0.:return True
        q = self.fixed(self.eng.st[p].speed, sqrt(1.+w-dw), p, 'fold')
        if not (self.eng.st[q].valid and self.eng.compatible(q, p)):return True
        if self.eng.st[q].ps < self.eng.st[p].ps:return True
        self.eng.stats['s10_below_left'] = self.eng.stats.get('s10_below_left', 0)+1
        return False

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef tuple fold_grad(self, int p):
        """S10: (dSEP/dV per km/h, dSEP/dw) at a curve state: fixed-load solves from it at its load and V + half the speed
        tolerance, and at its speed and w + 0.02 max(1, w) (NaN where one is not admitted or changes branch). The level
        set's slope there is dV/dw = -(dSEP/dw)/(dSEP/dV), finite through a fold (where dSEP/dw = 0)."""
        cdef int base = self.eng.st[p].base, q
        cdef double v = self.eng.st[p].speed, n = self.eng.st[p].load, w = n*n-1., dv = .5*self.speed_tol, dw, a = NAN, b = NAN
        got = self.grads.get(base)
        if got is not None:return got
        # (next to 1 g a fold can lie within a few thousandths of w: Sunderland Mk5 SB +10 m/s at 229 km/h; the step in w
        # stays within a quarter of the state's own w there)
        dw = pmin(.02*pmax(1., w), pmax(.25*w, 1e-4))
        q = self.fixed(v+dv, n, p, 'fold')
        if self.eng.st[q].valid and self.eng.compatible(q, p):a = (self.eng.st[q].ps-self.eng.st[p].ps)/dv
        q = self.fixed(v, sqrt(1.+w+dw), p, 'fold')
        if self.eng.st[q].valid and self.eng.compatible(q, p):b = (self.eng.st[q].ps-self.eng.st[p].ps)/dw
        got = (a, b);self.grads[base] = got
        return got

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef double fold_dvdw(self, int p) except? -1e300:
        """S10: the level set's dV/dw at a curve state (fold_grad), NaN where unavailable."""
        a, b = self.fold_grad(p)
        if not (isfinite(a) and isfinite(b)) or a == 0.:return NAN
        return -b/a

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int speed_root(self, double target, double n, double v0, int anchor) except -2:
        """S10: the level at the fixed load ``n``: its speed by secant steps of fixed-load solves (from ``v0``, the first
        step along dSEP/dV of ``anchor``), each seeded from the previous; the state within the SEP allowance of the level,
        or -1 (a solve not admitted or on another branch, or no closure in S10_ROOT_STEPS)."""
        cdef int p, q, k
        cdef double v1, f0, f1, d
        p = self.fixed(v0, n, anchor, 'fold')
        if not (self.eng.st[p].valid and self.eng.compatible(p, anchor)):return -1
        f0 = self.eng.st[p].ps-target
        if fabs(f0) <= self.allowance:return p
        d, _ = self.fold_grad(anchor)
        if not isfinite(d) or d == 0.:return -1
        v1 = v0-f0/d
        v1 = pmin(pmax(v1, v0-2.*self.speed_tol), v0+2.*self.speed_tol)
        for k in range(S10_ROOT_STEPS):
            q = self.fixed(v1, n, p, 'fold')
            if not (self.eng.st[q].valid and self.eng.compatible(q, anchor)):return -1
            f1 = self.eng.st[q].ps-target
            if fabs(f1) <= self.allowance:return q
            if f1 == f0 or self.eng.st[q].speed == self.eng.st[p].speed:return -1
            d = (f1-f0)/(self.eng.st[q].speed-self.eng.st[p].speed)
            if d == 0.:return -1
            p = q;f0 = f1;v0 = self.eng.st[q].speed
            v1 = v0-f0/d
            v1 = pmin(pmax(v1, v0-2.*self.speed_tol), v0+2.*self.speed_tol)
        return -1

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef object fold_turn(self, dict comp, int last, int side, int one):
        """S10: the contour turns back in speed beyond its knot ``last`` (on a branch where SEP falls with load), where level
        flight at that knot's speed (``one``) lies outside the level too: the level's second root at that speed, between
        level flight and the knot (fold_sample, then the bracketed solve). Where the knot itself lies just past the turn
        (within the SEP allowance of the level, below it: no second root at its speed), at the speed of one of the three
        knots before it instead, the first whose level flight lies outside the level too. Found, the component ends at the
        fold ('fold' end; its fold piece joins ``last`` to that root, refined by check_fold) and the other branch (a 'below'
        component: SEP rises with load there) is followed away from the fold. None where no second root is found (the S9
        ends then)."""
        cdef double target = comp['level'], vk = self.eng.st[last].speed, v = vk, fo, fh, sv
        cdef int hit = -1, other_root = -1, top = last, k, q, i
        if not self.s10 or self.eng.st[last].load <= 1.+1e-6:return None
        hit = self.fold_sample(v, target, last, (.8, .6, .4, .2))
        if hit < 0 and self.eng.st[last].ps < target:
            # the knot lies past the turn (Do 17Z-2 SB -20 m/s: 491.87 km/h, 0.047 m/s under the level): the turn lies
            # between it and the knots before it, whose speeds have both roots (samples from next to the knot down)
            knots = comp['knots'] if side > 0 else comp['knots'][::-1]
            for i in range(2, 5):
                if i > len(knots):break
                k = knots[-i]
                if self.eng.st[k].load <= 1.+1e-6:break
                q = self.fixed(self.eng.st[k].speed, 1., k, 'fold')
                if not (self.eng.st[q].valid and self.eng.compatible(q, k)):break
                if self.eng.st[q].ps >= target:
                    if i == 2:
                        # level flight at the knot before lies inside the level: the turn and the 1 g end of the other
                        # branch both lie between the two knots, closer than the march steps (XP-55 SB +10 m/s, 461-462
                        # km/h): the fold piece runs from the knot down to that 1 g end
                        r_ = self.one_g_root(target, q, one, 'fold')
                        if r_ is not None:
                            k = r_
                            if (self.eng.compatible(k, last) and self.eng.st[k].load == 1.
                                    and pmin(self.eng.st[q].speed, vk) < self.eng.st[k].speed < pmax(self.eng.st[q].speed, vk)):
                                comp['fold'] = dict(knots=[last, k], side=side)
                                self.eng.stats['s10_fold_ends'] = self.eng.stats.get('s10_fold_ends', 0)+1
                                return dict(kind='fold', speed_kmh=vk, other_load_g=1.)
                    break
                hit = self.fold_sample(self.eng.st[k].speed, target, k, (.95, .8, .6, .4, .2))
                if hit >= 0:
                    one = q;top = k;v = self.eng.st[k].speed
                    self.eng.stats['s10_fold_retreats'] = self.eng.stats.get('s10_fold_retreats', 0)+1
                    break
        if hit < 0:
            self.eng.stats['s10_fold_misses'] = self.eng.stats.get('s10_fold_misses', 0)+1
            return None
        fo = self.eng.st[one].ps-target;fh = self.eng.st[hit].ps-target
        if fh > self.allowance:
            r = self.bracketed(v, target, one, hit, 'fold')
            other_root = r['point'] if r['outcome'] == 'admitted' and r['point'] is not None else -1
            if other_root >= 0 and not (self.eng.compatible(other_root, last)
                                        and self.eng.st[one].load-1e-9 <= self.eng.st[other_root].load <= self.eng.st[hit].load+1e-9):
                other_root = -1
        if other_root < 0:
            # the SEP-fixed correction left the bracket (it can converge to the knot's own root), or the whole bracket lies
            # within the SEP allowance of the level (a fold a few thousandths of a m/s deep next to 1 g: Sunderland Mk5 SB
            # +10 m/s at 229 km/h): fixed-load solves, to a tenth of the bracket's own SEP spread
            other_root = self.load_root(v, target, one, hit, pmin(self.allowance, .1*pmin(-fo, fh)))
        if other_root < 0:return None
        if not (self.eng.compatible(other_root, last) and 1.+1e-9 < self.eng.st[other_root].load < self.eng.st[top].load-1e-9):
            return None
        other = dict(level=target, segment=comp['segment'], knots=[other_root], discovered_at_kmh=v, provenance='fold',
                     below=True)
        other['end' if side > 0 else 'start'] = dict(kind='fold', speed_kmh=vk)
        sv = self.fold_dvdw(other_root)
        # (away from the fold w falls on that branch: a slope of the other sign, from a root next to the turn, is not used)
        if isfinite(sv) and sv != 0. and side/sv > 0.:other['w_slope'] = 1./sv
        comp['fold'] = dict(knots=[last, other_root], side=side)
        self.pending.append(other)
        self.eng.stats['s10_folds'] = self.eng.stats.get('s10_folds', 0)+1
        return dict(kind='fold', speed_kmh=vk, other_load_g=self.eng.st[other_root].load)

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int fold_sample(self, double v, double target, int knot, tuple fractions) except -2:
        """S10: a state above the level at speed ``v`` between level flight and the curve state ``knot`` (fixed-load solves at
        the given fractions of the knot's w, each from the one before), or -1."""
        cdef double wk = self.eng.st[knot].load**2-1., f
        cdef int prev = knot, p
        for f in fractions:
            p = self.fixed(v, sqrt(1.+f*wk), prev, 'fold')
            if not (self.eng.st[p].valid and self.eng.compatible(p, knot)):return -1
            if self.eng.st[p].ps > target:return p
            prev = p
        return -1

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int load_root(self, double v, double target, int lo, int hi, double tol=-1.) except -2:
        """S10: the level at speed ``v`` between two states there (``lo`` below the level, ``hi`` above it): Illinois steps
        in w of fixed-load solves, each from the bracket end nearer in w; the state within the SEP allowance of the level,
        or -1 (a solve not admitted or on another branch, or 16 steps without closure). ``tol``: the SEP closure where
        given (the allowance otherwise)."""
        cdef double wl = self.eng.st[lo].load**2-1., wh = self.eng.st[hi].load**2-1., fl = self.eng.st[lo].ps-target
        cdef double fh = self.eng.st[hi].ps-target, w, f
        cdef int p, k, last_side = 0
        if not (fl < 0. < fh):return -1
        if tol < 0.:tol = self.allowance
        for k in range(16):
            w = wl-fl*(wh-wl)/(fh-fl)
            if not (pmin(wl, wh) < w < pmax(wl, wh)):w = .5*(wl+wh)
            p = self.fixed(v, sqrt(1.+pmax(w, 0.)), lo if fabs(w-wl) <= fabs(w-wh) else hi, 'fold')
            if not (self.eng.st[p].valid and self.eng.compatible(p, lo)):return -1
            f = self.eng.st[p].ps-target
            if fabs(f) <= tol:return p
            if f < 0.:
                wl = w;fl = f;lo = p
                if last_side < 0:fh *= .5
                last_side = -1
            else:
                wh = w;fh = f;hi = p
                if last_side > 0:fl *= .5
                last_side = 1
        return -1

    cdef follow_pending(self, dict segment, list components):
        """S10: the components found at folds, followed away from their fold."""
        while self.pending:
            other = self.pending.pop(0)
            components.append(other)
            self.follow(other, segment)

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef check_fold(self, dict comp):
        """S10: the split rule on a fold piece. Through the fold the level's speed is a smooth function of w (dV/dw = 0
        at the turn, where w(V) is singular), so the piece is drawn as the cubic Hermite V(w) through its knots in w with
        their exact dV/dw (fold_dvdw). A piece is finished when the band its end values and slopes allow (band_load in w)
        lies within S8_BOUND of the speed tolerance of the drawn line (the chart's horizontal slack covers it); otherwise
        the level is solved at the fixed load where the band is widest (speed_root) and becomes a knot."""
        cdef dict fold = comp.get('fold')
        cdef int n, k, j, a, b, q, added
        cdef double wa, wb, va, vb, w, vd, lo, hi, e, best, at
        cdef double* ws
        cdef double* vs
        cdef double* dl
        cdef double* dr
        if fold is None:return
        done = fold.setdefault('done', set())
        for _ in range(S10_FOLD_KNOTS):
            knots = sorted(fold['knots'], key=lambda x: self.eng.st[x].load)
            n = len(knots)
            if n >= S10_FOLD_KNOTS:break
            ws = <double*>malloc(n*sizeof(double));vs = <double*>malloc(n*sizeof(double))
            dl = <double*>malloc(n*sizeof(double));dr = <double*>malloc(n*sizeof(double))
            if ws == NULL or vs == NULL or dl == NULL or dr == NULL:
                if ws != NULL:cfree(ws)
                if vs != NULL:cfree(vs)
                if dl != NULL:cfree(dl)
                if dr != NULL:cfree(dr)
                raise MemoryError()
            added = 0
            try:
                for k in range(n):
                    ws[k] = self.eng.st[knots[k]].load**2-1.;vs[k] = self.eng.st[knots[k]].speed
                    dl[k] = self.fold_dvdw(knots[k]);dr[k] = dl[k]
                fill_slopes(n, ws, vs, dl, dr)
                for k in range(n-1):
                    a = knots[k];b = knots[k+1]
                    if (a, b) in done:continue
                    wa, wb = ws[k], ws[k+1];va, vb = vs[k], vs[k+1]
                    best = 0.;at = .5*(wa+wb)
                    for j in range(1, S8_SAMPLES):
                        w = wa+(wb-wa)*j/S8_SAMPLES
                        vd = hermite_eval(n, ws, vs, dl, dr, w)
                        band_load(wa, va, dr[k], wb, vb, dl[k+1], w, &lo, &hi)
                        e = pmax(hi-vd, vd-lo)
                        if not isfinite(e):e = INFINITY
                        if e > best:
                            best = e;at = w
                    if best <= S8_BOUND*self.speed_tol:
                        done.add((a, b))
                        continue
                    at = pmin(pmax(at, wa+.1*(wb-wa)), wb-.1*(wb-wa))
                    q = self.speed_root(comp['level'], sqrt(1.+at), hermite_eval(n, ws, vs, dl, dr, at),
                                        a if fabs(at-wa) <= fabs(at-wb) else b)
                    self.eng.stats['s10_fold_splits'] = self.eng.stats.get('s10_fold_splits', 0)+1
                    if q < 0:
                        done.add((a, b));fold['unresolved'] = True
                        continue
                    fold['knots'].append(q);added += 1
            finally:
                cfree(ws);cfree(vs);cfree(dl);cfree(dr)
            if not added:break

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef add_knot(self, dict comp, int point, int side):
        for k in comp['knots']:
            if abs(self.eng.st[k].speed-self.eng.st[point].speed) <= 1e-9:return
        if side > 0:comp['knots'].append(point)
        else:comp['knots'].insert(0, point)

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef follow(self, dict comp, dict segment):
        for side, key in ((1, 'end'), (-1, 'start')):
            # S10: a side already ended (a component found at a fold) is not followed again
            if self.s10 and key in comp:continue
            while True:
                end = self.march(comp, segment, side)
                if end is not None:break
            comp[key] = end

    # ------------------------------------------------------------------ S7: exact slopes
    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int slope_sides(self, double speed) except -1:
        """S7: both one-sided slopes next to a model kink, else the one towards the inside of the knot's segment."""
        cdef double v
        for v in self.knot_speeds:
            if fabs(v-speed) <= 2*self.speed_tol:return 3
        for seg in self.segs or ():
            if seg['speed_kmh'][0]-1e-9 <= speed <= seg['speed_kmh'][1]+1e-9:
                return 1 if speed+self.tangent_delta > seg['speed_kmh'][1] else 2
        return 2

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef tuple knot_slope(self, int sid):
        """(left, right) dw/dV of the knot's SEP level set (MarchEngine.tangent, mode 0): one value for both sides where
        the one-sided derivatives agree within 5% (their mean, a central difference), NaN where unavailable."""
        cdef double l = NAN, r = NAN, m
        cdef int base = self.eng.st[sid].base
        got = self.slopes.get(base)
        if got is not None:return got
        if not self.eng.tangent(base, 0, 0, 0, 0, self.eng.st[base].speed, self.tangent_delta, self.slope_sides(self.eng.st[base].speed),
                                &l, &r):
            l = NAN;r = NAN
        elif isfinite(l) and isfinite(r):
            m = pmax(fabs(l), fabs(r))
            if fabs(l-r) <= .05*m or m < 1e-9:
                l = .5*(l+r);r = l
        if self.s9:
            # S9: a tangent next to level flight (its turn coordinate is singular there) can come out far too steep
            # (T-2 SB +100 m/s: -160 per km/h at 1.05 g, the level set's own slope there about -0.5): no slope beyond
            # the steepest a drawn piece takes (S9_END_VERTICAL; a vertical end has its own, end_slope)
            if fabs(l) > S9_END_VERTICAL:l = NAN
            if fabs(r) > S9_END_VERTICAL:r = NAN
        got = (l, r);self.slopes[base] = got
        return got

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef object hermite(self, list knots):
        """The chart's contour interpolant: cubic Hermite in w through the knots with their exact slopes (PCHIP slopes
        where none is available)."""
        cdef int n = len(knots), i
        cdef HermiteCurve f
        if n < 2:return None
        f = HermiteCurve(n)
        for i in range(n):
            f.x[i] = self.eng.st[knots[i]].speed;f.y[i] = pow(self.eng.st[knots[i]].load, 2.)-1.
            l, r = self.knot_slope(knots[i])
            f.dl[i] = l;f.dr[i] = r
        f.build()
        return f

    # ------------------------------------------------------------------ interpolation validation
    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef object interpolant(self, list knots):
        cdef int n = len(knots), i
        if n < 2:return None
        f = PchipCurve(n)
        for i in range(n):
            f.x[i] = self.eng.st[knots[i]].speed;f.y[i] = pow(self.eng.st[knots[i]].load, 2.)-1.
        f.build()
        return f

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef double check_speed(self, double a, double b):
        mid = .5*(a+b)
        inside = [v for v in self.knot_speeds if a+self.speed_tol < v < b-self.speed_tol]
        return min(inside, key=lambda v: abs(v-mid)) if inside else mid

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef double piece_bound(self, HermiteCurve hf, int i, double* at) noexcept:
        """S8: the error bound of contour piece i (knots i, i+1 of ``hf``, the drawn Hermite in w = n^2-1) from its end
        values and slopes alone: the largest turn-rate distance from the drawn line (over x +- the speed tolerance) to
        the band the level set lies in (band_load in w), at S8_SAMPLES points; ``at``: where it is largest."""
        cdef double va = hf.x[i], vb = hf.x[i+1], x, xl, xr, lo, hi, t, tl, tr, e, best = 0.
        cdef int j
        at[0] = .5*(va+vb)
        for j in range(1, S8_SAMPLES):
            x = va+(vb-va)*j/S8_SAMPLES
            xl = pmax(va, x-self.speed_tol);xr = pmin(vb, x+self.speed_tol)
            t = curve_turn(x, sqrt(1.+pmax(hf.eval(x), 0.)))
            tl = curve_turn(xl, sqrt(1.+pmax(hf.eval(xl), 0.)));tr = curve_turn(xr, sqrt(1.+pmax(hf.eval(xr), 0.)))
            band_load(va, hf.y[i], hf.dr[i], vb, hf.y[i+1], hf.dl[i+1], x, &lo, &hi)
            e = pmax(curve_turn(x, sqrt(1.+pmax(hi, 0.)))-pmax(t, pmax(tl, tr)),
                     pmin(t, pmin(tl, tr))-curve_turn(x, sqrt(1.+pmax(lo, 0.))))
            if not isfinite(e):e = INFINITY
            if e > best:
                best = e;at[0] = x
        return best

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef check_intervals(self, dict comp):
        """MarchCurveSolver.check_intervals: withheld exact checks, kink-placed (S8: the split rule, piece_bound)."""
        cdef double target = comp['level'], speed, w, n, t, w0, interp_turn, turn_error, e = 0., xm = 0., h
        cdef int a, b, i, p
        cdef PchipCurve f
        cdef HermiteCurve hf
        cdef double x, tx, lo_t, hi_t, box
        checks = comp.setdefault('checks', [])
        unresolved = comp.setdefault('unresolved_intervals', [])
        for _ in range(self.refinement_passes):
            knots = comp['knots']
            if self.s7:
                hf = self.hermite(knots)
                if hf is None:return
            else:
                f = self.interpolant(knots)
                if f is None:return
            done = {(c['interval_kmh'][0], c['interval_kmh'][1]) for c in checks if c['passed']}
            blocked = {tuple(u['interval_kmh']) for u in unresolved}
            additions = []
            for i in range(len(knots)-1):
                a = knots[i];b = knots[i+1]
                key = (self.eng.st[a].speed, self.eng.st[b].speed)
                if key in done or key in blocked:continue
                if self.s8:
                    # S8: the split rule: no solve where the piece's error bound is within the tolerance
                    h = key[1]-key[0]
                    e = self.piece_bound(hf, i, &xm) if h > 0. else 0.
                    bound = e if isfinite(e) else None
                    if h <= self.speed_tol or (e <= self.turn_tol and h <= pmin(self.max_step, S8_MAX_WIDTH)):
                        if e > self.turn_tol:
                            comp.setdefault('steps', []).append(dict(interval_kmh=list(key), bound_dps=bound))
                            self.eng.stats['s8_curve_steps'] = self.eng.stats.get('s8_curve_steps', 0)+1
                        checks.append(dict(interval_kmh=list(key), speed_kmh=None, passed=True,
                                           route='step' if e > self.turn_tol else 'bound', bound_dps=bound))
                        continue
                    if e <= self.turn_tol:xm = .5*(key[0]+key[1])
                    speed = pmin(pmax(xm, key[0]+.1*h), key[1]-.1*h)
                    self.eng.stats['s8_curve_splits'] = self.eng.stats.get('s8_curve_splits', 0)+1
                else:
                    if self.eng.st[b].speed-self.eng.st[a].speed <= self.speed_tol:continue
                    speed = self.check_speed(self.eng.st[a].speed, self.eng.st[b].speed)
                w = hf.eval(speed) if self.s7 else f.eval(speed);n = sqrt(1.+pmax(w, 0.))
                t = (speed-self.eng.st[a].speed)/(self.eng.st[b].speed-self.eng.st[a].speed)
                attitude = [self.eng.st[a].sol[k]*(1-t)+self.eng.st[b].sol[k]*t for k in range(5)]
                outcome, truth, reason = self.direct_seed(speed, target, n, [a, b], attitude, 'interpolation_checks')
                if outcome == 'admitted' and not (self.eng.compatible(truth, a) and self.eng.compatible(truth, b)):
                    outcome, reason = 'incompatible', 'midpoint branch differs from knots'
                if outcome != 'admitted' and self.s7:
                    # S7: the S5 seed (the PCHIP load) before the single-anchor retries; the check still compares the
                    # exact state with the drawn (Hermite) line
                    f = self.interpolant(knots)
                    w0 = sqrt(1.+pmax(f.eval(speed), 0.))
                    if w0 != n:
                        o2, t2, r2 = self.direct_seed(speed, target, w0, [a, b], attitude, 'interpolation_checks')
                        if o2 == 'admitted' and self.eng.compatible(t2, a) and self.eng.compatible(t2, b):
                            outcome, truth, reason = o2, t2, r2
                if outcome != 'admitted':
                    retry = None
                    for anchor in (a, b):
                        w0 = pow(self.eng.st[anchor].load, 2.)-1.
                        retry_outcome, retry_p, _ = self.direct_seed(speed, target, sqrt(1.+pmax(w0, 1e-6)), [anchor], None,
                                                                     'interpolation_checks')
                        if retry_outcome == 'admitted' and self.eng.compatible(retry_p, a) and self.eng.compatible(retry_p, b):
                            retry = retry_p;break
                    if retry is None and self.s8:
                        # S8 (item 3): a failing stretch: sliding on a confirmed blade-polar jump, then the S6 retries
                        self.eng.flush_counts();before = self.ledger.snapshot()
                        retry = self.rescue_direct(speed, target, a, attitude, n, 'interpolation_checks')
                        if retry < 0 or not self.eng.compatible(retry, b):retry = None
                        self.eng.flush_counts()
                        self.ledger.charge('interpolation_checks', before, 'admitted' if retry is not None else 'unresolved',
                                           kind='rescue', speed_kmh=speed, target_mps=target)
                    if retry is not None:
                        additions.append(retry);continue
                    if self.s9:
                        self.failed_piece(comp, a, b, speed, outcome == 'rejected' and truth is not None and self.exit_evidence(truth),
                                          reason, unresolved, additions)
                        continue
                    if self.s8:
                        # S8 (item 3): the curve leaves the envelope between these knots (its exact state at the midpoint
                        # is physically rejected): it ends at the envelope on both sides, each end located to the speed
                        # tolerance, and the stretch between is excluded (a break with a physical cause, not a gap); a
                        # stretch that stays numerically unresolved is narrowed the same way (an unresolved break)
                        ea = self.edge_root(a, speed, target);eb = self.edge_root(b, speed, target)
                        lo_v = self.eng.st[ea].speed if ea >= 0 else self.eng.st[a].speed
                        hi_v = self.eng.st[eb].speed if eb >= 0 else self.eng.st[b].speed
                        if ea >= 0:additions.append(ea)
                        if eb >= 0:additions.append(eb)
                        if outcome != 'rejected':
                            # a stretch where the envelope boundary's own SEP is above the level throughout: the level
                            # lies past the boundary there (the curve is outside the envelope), whatever the solve did
                            inside = [ps for v, ps in zip(self.env_v, self.env_ps) if lo_v+1e-9 < v < hi_v-1e-9 and ps is not None]
                            if inside and all(ps > target for ps in inside):
                                outcome, reason = 'rejected', 'level above the envelope boundary SEP'
                        unresolved.append(dict(interval_kmh=[lo_v, hi_v], reason=reason,
                                               outcome='excluded' if outcome == 'rejected' else outcome))
                        key2 = 's8_curve_exits' if outcome == 'rejected' else 's8_curve_narrowed'
                        self.eng.stats[key2] = self.eng.stats.get(key2, 0)+1
                        continue
                    unresolved.append(dict(interval_kmh=list(key), reason=reason, outcome=outcome))
                    continue
                if self.s8:
                    # S8: the solved state is a knot (the split), not a withheld check
                    checks.append(dict(interval_kmh=list(key), speed_kmh=speed, passed=False, route='split', bound_dps=bound,
                                       holdout=self.compact(truth)))
                    additions.append(truth)
                    continue
                interp_turn = curve_turn(speed, n);turn_error = abs(interp_turn-self.eng.st[truth].turn)
                record = dict(interval_kmh=list(key), speed_kmh=speed, interpolated_turn_dps=interp_turn,
                              exact_turn_dps=self.eng.st[truth].turn, turn_error_dps=turn_error,
                              holdout=self.compact(truth), at_model_knot=abs(speed-.5*sum(key)) > 1e-9)
                passed = turn_error <= self.turn_tol
                if record['at_model_knot']:
                    record['passed'] = False;record['kink_knot'] = True
                    checks.append(record);additions.append(truth)
                    continue
                if self.s7:
                    # S7: position with the chart's horizontal slack (the drawn line over speed +- speed tolerance)
                    lo_t = interp_turn;hi_t = interp_turn
                    for x in (speed-self.speed_tol, speed+self.speed_tol):
                        x = pmin(pmax(x, self.eng.st[a].speed), self.eng.st[b].speed)
                        tx = curve_turn(x, sqrt(1.+pmax(hf.eval(x), 0.)))
                        lo_t = pmin(lo_t, tx);hi_t = pmax(hi_t, tx)
                    tx = self.eng.st[truth].turn
                    box = 0. if lo_t <= tx <= hi_t else (tx-hi_t if tx > hi_t else lo_t-tx)
                    passed = box <= S7_CURVE_CHECK*self.turn_tol
                    record.update(position_error_dps=box, route='position')
                    record['passed'] = passed
                    checks.append(record)
                    if not passed:additions.append(truth)
                    continue
                if not passed:
                    p = self.fixed(speed, pmax(1., n), truth, 'interpolation_checks')
                    record.update(interpolated_point_valid=self.eng.st[p].valid, sep_error_mps=abs(self.eng.st[p].ps-target),
                                  interpolated_point_reasons=self.eng.reason_list(p))
                    passed = bool(self.eng.st[p].valid and abs(self.eng.st[p].ps-target) <= self.check_band_fraction*self.band)
                record['passed'] = passed
                checks.append(record)
                if not passed:additions.append(truth)
            if not additions:return
            for p in additions:comp['knots'].append(p)
            comp['knots'].sort(key=lambda q: self.eng.st[q].speed)
            speeds = {self.eng.st[p].speed for p in additions}
            for c in checks:
                if c.get('holdout') is not None and c['holdout']['speed_kmh'] in speeds:c['became_knot'] = True
        comp['refinement_exhausted'] = True

    # ------------------------------------------------------------------ top level
    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef tuple column_samples(self, column):
        """CurveSolver.column_samples: (admitted samples by load up to the verified boundary, boundary or -1)."""
        cdef double speed = column['speed_kmh'], cap = 0.
        cdef bint has_cap = False
        cdef int top = -1, sid
        if column.get('boundary_status') in ('verified limit', 'plot ceiling') and column.get('boundary'):
            top = self.state(column['boundary'])
        if top >= 0:
            cap = self.eng.st[top].load;has_cap = True
        else:
            for p in column['points']:
                if p.get('valid'):
                    v = self.eng.st[self.state(p)].load
                    cap = v if not has_cap else max(cap, v);has_cap = True
        pts = []
        for p in column['points']:
            sid = self.state(p)
            if (self.eng.st[sid].valid and self.eng.st[sid].has_coords and abs(self.eng.st[sid].speed-speed) <= 1e-6
                    and (not has_cap or self.eng.st[sid].load <= cap+1e-12)):pts.append(sid)
        pts.sort(key=lambda q: self.eng.st[q].load)
        if top >= 0 and self.eng.st[top].valid and self.eng.st[top].has_coords and \
                all(abs(self.eng.st[q].load-self.eng.st[top].load) > 1e-12 for q in pts):
            pts.append(top)
        return pts, top

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef object column_levels(self, column):
        """CurveSolver.column_levels: the levels column_roots would find at this column (no solves), or None."""
        cdef int i
        pts, _ = self.column_samples(column)
        if not pts or abs(self.eng.st[pts[0]].load-1.) > 1e-9:return None
        out = []
        for t in self.levels:
            if abs(self.eng.st[pts[0]].ps-t) <= self.allowance:
                out.append(t);continue
            for i in range(len(pts)-1):
                if ((self.eng.st[pts[i]].ps-t)*(self.eng.st[pts[i+1]].ps-t) <= 0. and
                        self.eng.compatible(pts[i], pts[i+1])):
                    out.append(t);break
        return frozenset(out)

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef dict column_roots(self, column, list levels):
        cdef double speed = column['speed_kmh'], target
        cdef bint found
        cdef int top, a, b, i
        out = dict(speed_kmh=speed, roots=[], fallback=None, stop=None)
        pts, top = self.column_samples(column)
        if not pts or abs(self.eng.st[pts[0]].load-1.) > 1e-9:
            out['stop'] = dict(load_g=None, kind='unresolved', reasons=['no admitted level-flight state in column'])
            out['uninformative'] = True
            return out
        for target in levels:
            found = False
            for i in range(len(pts)-1):
                a = pts[i];b = pts[i+1]
                if (self.eng.st[a].ps-target)*(self.eng.st[b].ps-target) > 0. or not self.eng.compatible(a, b):continue
                root = self.bracketed(speed, target, a, b, 'column_roots')
                if self.s10:root['rising'] = self.eng.st[b].ps > self.eng.st[a].ps
                out['roots'].append(dict(root, target=target));found = True
            if found:continue
            if abs(self.eng.st[pts[0]].ps-target) <= self.allowance:
                out['roots'].append(dict(outcome='admitted', point=pts[0], bracket=[self.eng.st[pts[0]].load]*2, target=target))
            elif top >= 0 and pts[-1] == top and self.eng.st[top].ps > target:
                out['roots'].append(dict(outcome='rejected', point=None, target=target, edge_g=self.eng.st[top].load,
                                         reason=['beyond verified envelope: '+str(column.get('boundary_reason'))],
                                         evidence='verified boundary SEP above the level'))
        return out

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef link_columns(self, dict segment, dict by_speed, list components, list seeds_report, list excluded,
                      list unresolved_targets, dict ones):
        """CurveSolver.link_columns (with the level-presence columns between thinned solved columns)."""
        new, open_ = [], {}
        ordered = sorted(by_speed);last_used = None
        solved, presence = [], {}
        for speed in ordered:
            if (last_used is not None and speed-last_used < self.column_min_spacing and speed != ordered[-1]):continue
            presence[speed] = self.column_levels(by_speed[speed])
            if presence[speed] is None:continue
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
                one = None
                for p in column['points']:
                    if abs(p['load_g']-1.) <= 1e-9 and p.get('valid') and p.get('stationary_coordinates') is not None:
                        one = self.state(p);break
                ones[speed] = self.ones[speed] = one if one is not None else dict(
                    valid=False, reasons=['no admitted level-flight state in column'], speed_kmh=speed)
            seed = self.column_roots(column, levels)
            self.ladders.append(dict(speed_kmh=speed, segment=segment['speed_kmh'], ladder=seed))
            seeds_report.append(dict(speed_kmh=speed, provenance='envelope column' if speed in presence else
                                     'envelope column (level presence change)',
                                     roots=sum(r['outcome'] == 'admitted' for r in seed['roots']), stop=seed['stop']))
            if seed.get('uninformative'):continue
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
                # (S10: a root on a branch where SEP rises with load joins only such a component, and the usual ones only
                # the usual ones: the two branches of a contour that turns back are two components)
                choices = [c for c in open_.get(target, []) if c not in taken and self.eng.compatible(c['knots'][-1], p)
                           and (not self.s10 or bool(c.get('below')) == bool(root.get('rising')))]
                if choices:
                    comp = min(choices, key=lambda c: abs(self.eng.st[c['knots'][-1]].turn-self.eng.st[p].turn))
                    self.add_knot(comp, p, 1)
                else:
                    comp = dict(level=target, segment=segment['speed_kmh'], knots=[p], discovered_at_kmh=speed,
                                provenance='envelope column')
                    if self.s10 and root.get('rising'):comp['below'] = True
                    new.append(comp)
                current.setdefault(target, []).append(comp)
            open_ = current
        components.extend(new)
        for comp in new:
            if self.s9 and any(other is not comp and 'end' in other and other['level'] == comp['level']
                               and other['segment'] == comp['segment'] and all(self.on_line(other, k) for k in comp['knots'])
                               for other in components):
                # S9: every knot of this component lies on a component already followed (the same contour reached from
                # another column across a column where the level was absent): no second copy
                components.remove(comp)
                self.eng.stats['s9_duplicates_skipped'] = self.eng.stats.get('s9_duplicates_skipped', 0)+1
                continue
            self.follow(comp, segment)
            if self.s10:self.follow_pending(segment, components)

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef bint on_line(self, dict comp, int point) except -1:
        """S9: whether the state ``point`` lies on component ``comp``'s contour: inside its knot span, compatible with its
        nearest knot, and within four curve tolerances in turn rate of its interpolant at that speed (PCHIP; where that is
        off by up to twenty, the level solved there from the component's neighbouring knots decides, within one)."""
        cdef double v = self.eng.st[point].speed, t, n, d
        cdef PchipCurve f
        cdef int i, a, b
        knots = comp['knots']
        if not knots or not (self.eng.st[knots[0]].speed-1e-9 <= v <= self.eng.st[knots[-1]].speed+1e-9):return False
        near = min(knots, key=lambda k: fabs(self.eng.st[k].speed-v))
        if not self.eng.compatible(point, near):return False
        if len(knots) == 1:return fabs(self.eng.st[near].turn-self.eng.st[point].turn) <= 4*self.turn_tol
        f = self.interpolant(knots)
        n = sqrt(1.+pmax(f.eval(v), 0.))
        d = fabs(curve_turn(v, n)-self.eng.st[point].turn)
        if d <= 4*self.turn_tol:return True
        if d > 20*self.turn_tol:return False
        i = 1
        while i < len(knots)-1 and self.eng.st[knots[i]].speed < v:i += 1
        a, b = knots[i-1], knots[i]
        t = (v-self.eng.st[a].speed)/pmax(self.eng.st[b].speed-self.eng.st[a].speed, 1e-12)
        attitude = [self.eng.st[a].sol[k]*(1-t)+self.eng.st[b].sol[k]*t for k in range(5)]
        outcome, same, _ = self.direct_seed(v, comp['level'], n, [a, b], attitude, 'seed_identity')
        return (outcome == 'admitted' and self.eng.compatible(same, a)
                and fabs(self.eng.st[same].turn-self.eng.st[point].turn) <= self.turn_tol)

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef list merge_duplicates(self, list components):
        """S9: components of one level and segment whose spans overlap and whose knots there lie on each other's contour
        (on_line) are the same contour found from two seeds: joined into one (the knots of both, a knot within 0.001 km/h
        of one already there dropped, the outer ends kept), so no stretch is drawn or counted twice."""
        out = []
        for comp in components:
            target = None
            v0, v1 = self.eng.st[comp['knots'][0]].speed, self.eng.st[comp['knots'][-1]].speed
            for other in out:
                if other['level'] != comp['level'] or other['segment'] != comp['segment']:continue
                # S10: a branch where SEP rises with load and one where it falls are two halves of a contour that turns back,
                # never one contour found twice (next to 1 g they can lie within the turn-rate test: Sunderland Mk5 SB)
                if self.s10 and bool(other.get('below')) != bool(comp.get('below')):continue
                w0, w1 = self.eng.st[other['knots'][0]].speed, self.eng.st[other['knots'][-1]].speed
                lo, hi = max(v0, w0), min(v1, w1)
                if hi < lo-1e-9:continue
                inner = [k for k in comp['knots'] if lo-1e-9 <= self.eng.st[k].speed <= hi+1e-9]
                inner2 = [k for k in other['knots'] if lo-1e-9 <= self.eng.st[k].speed <= hi+1e-9]
                if (inner and all(self.on_line(other, k) for k in inner)) or (inner2 and all(self.on_line(comp, k) for k in inner2)):
                    target = other;break
            if target is None:
                out.append(comp);continue
            w0, w1 = self.eng.st[target['knots'][0]].speed, self.eng.st[target['knots'][-1]].speed
            have = [self.eng.st[k].speed for k in target['knots']]
            for k in comp['knots']:
                if all(fabs(self.eng.st[k].speed-x) > 1e-3 for x in have):
                    target['knots'].append(k);have.append(self.eng.st[k].speed)
            target['knots'].sort(key=lambda q: self.eng.st[q].speed)
            if v0 < w0-1e-9:target['start'] = comp.get('start')
            if v1 > w1+1e-9:target['end'] = comp.get('end')
            if self.s10 and comp.get('fold') is not None:
                # S10: the fold piece goes with its knots (one per component: a second one is the same turn found twice)
                if target.get('fold') is None:target['fold'] = comp['fold']
                else:self.eng.stats['s10_fold_duplicates'] = self.eng.stats.get('s10_fold_duplicates', 0)+1
            if self.s10 and comp.get('below'):target['below'] = True
            self.eng.stats['s9_merged_components'] = self.eng.stats.get('s9_merged_components', 0)+1
        return out

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef object covered(self, list components, double target, int point):
        cdef PchipCurve f
        cdef int i, a, b
        cdef double t, n
        for comp in components:
            if comp['level'] != target:continue
            v = [self.eng.st[p].speed for p in comp['knots']]
            if not v[0]-1e-9 <= self.eng.st[point].speed <= v[-1]+1e-9:continue
            f = self.interpolant(comp['knots'])
            if f is None:
                if (self.eng.compatible(point, comp['knots'][0]) and
                        abs(self.eng.st[comp['knots'][0]].turn-self.eng.st[point].turn) <= self.turn_tol):return True
                continue
            if not self.eng.compatible(point, comp['knots'][0]):continue
            n = sqrt(1.+pmax(f.eval(self.eng.st[point].speed), 0.))
            if abs(curve_turn(self.eng.st[point].speed, n)-self.eng.st[point].turn) <= 4*self.turn_tol:
                return True
            import numpy as np
            i = max(1, min(len(v)-1, int(np.searchsorted(v, self.eng.st[point].speed))))
            a, b = comp['knots'][i-1], comp['knots'][i]
            t = (self.eng.st[point].speed-self.eng.st[a].speed)/pmax(self.eng.st[b].speed-self.eng.st[a].speed, 1e-12)
            attitude = [self.eng.st[a].sol[k]*(1-t)+self.eng.st[b].sol[k]*t for k in range(5)]
            outcome, same, _ = self.direct_seed(self.eng.st[point].speed, target, n, [a, b], attitude, 'seed_identity')
            if (outcome == 'admitted' and self.eng.compatible(same, a) and
                    abs(self.eng.st[same].turn-self.eng.st[point].turn) <= self.turn_tol):
                if all(abs(self.eng.st[k].speed-self.eng.st[point].speed) > 1e-9 for k in comp['knots']):
                    comp['knots'].insert(i, point)
                return True
        return False

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef object from_one_g(self, int crossing, int above, int below, double target, dict segment):
        cdef double v = self.eng.st[crossing].speed, dv, d1, dw
        cdef int two
        dv = self.eng.st[above].speed-self.eng.st[below].speed
        if dv == 0.:return None
        d1 = (self.eng.st[above].ps-self.eng.st[below].ps)/dv
        if self.s10:
            # S10: SEP rising with load at the crossing (a level-turn solve at w = S9_END_DW): the lower half of a contour that
            # turns back meets 1 g here (Do 17 Z-7 RB -40 m/s at 567.8 km/h, the plot ending at 570 before the turn): a
            # 'below' component
            two = self.fixed(v, sqrt(1.+S9_END_DW), crossing, 'one_g_discovery')
            if (self.eng.st[two].valid and self.eng.compatible(two, crossing) and self.eng.st[two].ps > self.eng.st[crossing].ps
                    and d1 != 0.):
                dw = (self.eng.st[two].ps-self.eng.st[crossing].ps)/S9_END_DW
                self.eng.stats['s10_below_from_one_g'] = self.eng.stats.get('s10_below_from_one_g', 0)+1
                return dict(level=target, segment=segment['speed_kmh'], knots=[crossing], discovered_at_kmh=v,
                            provenance='1 g crossing', w_slope=-d1/dw, below=True)
        two = self.fixed(v, 2., crossing, 'one_g_discovery')
        if not (self.eng.st[two].valid and self.eng.compatible(two, crossing)):return None
        dw = (self.eng.st[two].ps-self.eng.st[crossing].ps)/3.
        if dw >= 0.:return None
        return dict(level=target, segment=segment['speed_kmh'], knots=[crossing], discovered_at_kmh=v,
                    provenance='1 g crossing', w_slope=-d1/dw)

    @cython.boundscheck(True)
    @cython.wraparound(True)
    def run(self, columns):
        """MarchCurveSolver.run() with ``columns`` (the verified envelope column records)."""
        import time, math
        from em_trim_work import Work, WorkLimit, use
        import em_march_curves as C
        started = time.perf_counter()
        total = Work(dict(seconds=self.policy.seconds))
        self.total = total
        self.ledger = C.Ledger(total)
        segments, limit, cuts = self.py.segments()
        self.segs = segments
        self.ladders = [];self.ones = {}
        self.knot_speeds = self.py.model_knot_speeds(float(self.py.config['speed_min_kmh']), float(self.py.config['speed_max_kmh']))
        columns = sorted(columns, key=lambda c: c['speed_kmh'])
        self.env_v = [float(c['speed_kmh']) for c in columns if c.get('boundary')]
        self.env_n = [float(c['boundary']['load_g']) for c in columns if c.get('boundary')]
        self.env_ps = [c['boundary'].get('ps_mps') for c in columns if c.get('boundary')]
        components, seeds_report, unresolved_targets, excluded = [], [], [], []
        status = 'completed';junctions = []
        global g_sink
        saved = <size_t>g_sink
        try:
            with use(total):
                g_sink = self.eng.counts
                try:
                    for segment in segments:
                        lo, hi = segment['speed_kmh']
                        by_speed = {c['speed_kmh']: c for c in columns if lo-1e-9 <= c['speed_kmh'] <= hi+1e-9}
                        ones = {}
                        self.link_columns(segment, by_speed, components, seeds_report, excluded, unresolved_targets, ones)
                        ordered = sorted(ones.items())
                        for (va, pa), (vb, pb) in zip(ordered, ordered[1:]):
                            if not (isinstance(pa, int) and isinstance(pb, int)):continue
                            if not (self.eng.st[pa].valid and self.eng.st[pb].valid and self.eng.compatible(pa, pb)):continue
                            for target in self.levels:
                                if (self.eng.st[pa].ps-target)*(self.eng.st[pb].ps-target) >= 0.:continue
                                explained = any(c['level'] == target and any(
                                    e and e.get('kind') == 'one_g' and va-self.speed_tol <= (
                                        e['speed_kmh'] if not isinstance(e['speed_kmh'], list) else e['speed_kmh'][0]) <= vb+self.speed_tol
                                    for e in (c.get('start'), c.get('end'))) for c in components)
                                if self.s10 and not explained:
                                    # S10: or a fold piece that runs down to 1 g there (NCurves.fold_turn)
                                    explained = any(c['level'] == target and c.get('fold') is not None and any(
                                        self.eng.st[q].load == 1. and va-self.speed_tol <= self.eng.st[q].speed <= vb+self.speed_tol
                                        for q in c['fold']['knots']) for c in components)
                                if explained:continue
                                above, below = (pa, pb) if self.eng.st[pa].ps > target else (pb, pa)
                                crossing = self.locate_one_g(target, above, below)
                                if crossing is None:
                                    unresolved_targets.append(dict(target_mps=target, speed_kmh=[va, vb],
                                        reason='1 g crossing bracketed but not localized'))
                                    continue
                                if self.covered(components, target, crossing):continue
                                if self.s9:self.end_slope(crossing)
                                comp = self.from_one_g(crossing, above, below, target, segment)
                                if comp is None:
                                    unresolved_targets.append(dict(target_mps=target, speed_kmh=self.eng.st[crossing].speed,
                                        reason='1 g crossing found; level slope unavailable'))
                                    continue
                                components.append(comp)
                                self.follow(comp, segment)
                                if self.s10:self.follow_pending(segment, components)
                    if self.s9:components[:] = self.merge_duplicates(components)
                    for comp in components:
                        self.check_intervals(comp)
                    if self.s10:
                        for comp in components:self.check_fold(comp)
                    junctions = self.seam_junctions(segments, components)
                    self.raw_components = components
                finally:
                    g_sink = <long*>saved
                    self.eng.flush_counts()
        except WorkLimit as error:
            status = 'exhausted: '+str(error)
        elapsed = time.perf_counter()-started
        work = dict(total.counts)
        return dict(identity=self.py.identity(), status=status, elapsed_s=elapsed,
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

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef list seam_junctions(self, list segments, list components):
        out = []
        for left, right in zip(segments, segments[1:]):
            if left['ends'][1] != 'mach_seam':continue
            vl, vr = left['speed_kmh'][1], right['speed_kmh'][0]
            ol, orr = self.ones.get(vl), self.ones.get(vr)
            for L in self.levels:
                tl, tr = self.end_turn(components, L, left, 1), self.end_turn(components, L, right, -1)
                bl, br = self.below(ol, L), self.below(orr, L)
                lower = [t for t in (tl, tr) if t is not None]
                if tl is not None and tr is not None:span, status = [min(tl, tr), max(tl, tr)], 'established'
                elif lower and ((tl is None and br) or (tr is None and bl)):
                    span, status = [0., lower[0]], 'established (other side below level at 1 g)'
                elif lower:span, status = [lower[0], None], 'open: other side not reached below the envelope'
                elif bl is not None and br is not None and bl != br:
                    span, status = [0., None], 'open: 1 g SEP straddles level; envelope not computed'
                else:continue
                out.append(dict(seam_kmh=[vl, vr], level=L, left_turn_dps=tl, right_turn_dps=tr,
                                left_one_g_sep=self.one_ps(ol), right_one_g_sep=self.one_ps(orr),
                                span_turn_dps=span, status=status))
        return out

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef object end_turn(self, list components, double L, dict segment, int side):
        for c in components:
            if c['level'] == L and c['segment'] == segment['speed_kmh']:
                e = c.get('end' if side > 0 else 'start')
                if e and e.get('kind') == 'mach_seam':
                    return self.eng.st[c['knots'][-1] if side > 0 else c['knots'][0]].turn
        return None

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef object below(self, one, double L):
        if one is None:return None
        if isinstance(one, dict):
            return None if not one['valid'] else one['ps_mps'] < L
        if not self.eng.st[one].valid:return None
        return self.eng.st[one].ps < L

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef object one_ps(self, one):
        # ol and ol.get('ps_mps'): None for a missing state, False/the row value otherwise
        if one is None:return None
        if isinstance(one, dict):return one.get('ps_mps')
        return self.eng.st[one].ps if isfinite(self.eng.st[one].ps) else None

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef dict report(self, dict comp):
        def end(e):
            if e is None:return None
            out = {}
            for k, v in e.items():
                if k == 'state' and isinstance(v, int):out[k] = self.compact(v)
                else:out[k] = v
            return out
        knots = [self.compact(p) for p in comp['knots']]
        if self.s7:
            for p, k in zip(comp['knots'], knots):
                l, r = self.knot_slope(p)
                k['s7_w_slope'] = [l if isfinite(l) else None, r if isfinite(r) else None]
        out = dict(level=comp['level'], segment_kmh=comp['segment'], discovered_at_kmh=comp['discovered_at_kmh'],
                   provenance=comp.get('provenance'),
                   knots=knots, start=end(comp.get('start')), end=end(comp.get('end')),
                   checks=comp.get('checks', []), unresolved_intervals=comp.get('unresolved_intervals', []),
                   steps=comp.get('steps', []), refinement_exhausted=comp.get('refinement_exhausted', False))
        if self.s10:
            if comp.get('below'):out['below'] = True
            fold = comp.get('fold')
            if fold is not None:
                # S10: the fold piece, its knots by load (each with the level set's dV/dw there)
                fk = []
                for q in sorted(fold['knots'], key=lambda q: self.eng.st[q].load):
                    k = self.compact(q);d = self.fold_dvdw(q)
                    k['fold_dv_dw'] = d if isfinite(d) else None
                    fk.append(k)
                out['fold'] = dict(knots=fk, unresolved=bool(fold.get('unresolved')))
        return out


cdef class PchipCurve:
    """PchipInterpolator(v, w) of curve knots (native, same arithmetic as scipy)."""
    cdef int n
    cdef double* x
    cdef double* y
    cdef double* c

    @cython.boundscheck(True)
    @cython.wraparound(True)
    def __cinit__(self, int n):
        self.n = n
        self.x = <double*>malloc(n*sizeof(double));self.y = <double*>malloc(n*sizeof(double))
        self.c = <double*>malloc(4*n*sizeof(double))
        if self.x == NULL or self.y == NULL or self.c == NULL:raise MemoryError()

    @cython.boundscheck(True)
    @cython.wraparound(True)
    def __dealloc__(self):
        if self.x != NULL:cfree(self.x)
        if self.y != NULL:cfree(self.y)
        if self.c != NULL:cfree(self.c)

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int build(self) except -1:
        return pchip_build(self.n, self.x, self.y, self.c)

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef double eval(self, double v) noexcept:
        return pchip_eval(self.n, self.x, self.c, v)


cdef class HermiteCurve:
    """S7: the piecewise cubic Hermite of curve knots (values, exact slopes, PCHIP slopes where missing)."""
    cdef int n
    cdef double* x
    cdef double* y
    cdef double* dl
    cdef double* dr

    @cython.boundscheck(True)
    @cython.wraparound(True)
    def __cinit__(self, int n):
        self.n = n
        self.x = <double*>malloc(n*sizeof(double));self.y = <double*>malloc(n*sizeof(double))
        self.dl = <double*>malloc(n*sizeof(double));self.dr = <double*>malloc(n*sizeof(double))
        if self.x == NULL or self.y == NULL or self.dl == NULL or self.dr == NULL:raise MemoryError()

    @cython.boundscheck(True)
    @cython.wraparound(True)
    def __dealloc__(self):
        if self.x != NULL:cfree(self.x)
        if self.y != NULL:cfree(self.y)
        if self.dl != NULL:cfree(self.dl)
        if self.dr != NULL:cfree(self.dr)

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int build(self) except -1:
        fill_slopes(self.n, self.x, self.y, self.dl, self.dr)
        return 0

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef double eval(self, double v) noexcept:
        return hermite_eval(self.n, self.x, self.y, self.dl, self.dr, v)
