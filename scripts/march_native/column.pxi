# S5: one alpha-march column on native trial states (em_march.March + RescueMarch,
# march_worker, column_record, em_pitch_limit.limit), line by line.
# S6: robust corrections (regime_seed, retry) and the S6 rescue (robust_march), native only.

DEF STATUS_VERIFIED = 0     # 'verified limit'
DEF STATUS_CEILING = 1      # 'plot ceiling'
DEF STATUS_UNRESOLVED = 2   # 'unresolved numerical boundary'
DEF STATUS_NOFEASIBLE = 3   # 'no feasible samples'
DEF STATUS_OTHER = 4        # any other status (excluded / discontinuity / fallback columns): text in NCol.status_text
# S6 robust corrections: governor coordinates this close to a regime boundary get a boundary seed; the budget retry
DEF REGIME_REACH = .35
DEF ROBUST_NFEV = 30
DEF ROBUST_EVALS = 120
DEF ROBUST_RETRIES = 16     # retried corrections per S6 rescue of one column (the resolving rescues needed at most 13)
DEF S9_TRIM_DEV = .02       # S9: elevator-command distance (of its [-1, 1] travel) from the step's linear prediction ...
DEF S9_TRIM_SLOPE = .02     # ... with a secant-slope change (per deg) above the previous secant's, or this, splits the step
DEF S9_TRIM_PITCH = .8      # ... where the pitch-response margin fell below this fraction over the step before
DEF S9_PROP_JUMP = .05      # S9: a propulsion coordinate change (of its size, at least 1) that marks another propeller branch

STATUS_TEXT = ('verified limit', 'plot ceiling', 'unresolved numerical boundary', 'no feasible samples', None)
RESCUE_REASONS = ('march correction unresolved', 'march roots disagree')
# S5: the S3 rescue also follows an unresolved localization (the Python march sends it to the production column)
NATIVE_RESCUE_REASONS = RESCUE_REASONS+('localization correction unresolved',)


cdef inline int side12(double x) noexcept nogil:
    # (x >= 12) - (x <= -12)
    return (x >= SWITCH_DEG)-(x <= -SWITCH_DEG)


cdef bint parabola(double x0, double y0, double x1, double y1, double x2, double y2, double* c) noexcept:
    """em_march.parabola: (c2, c1, c0) of the parabola through three points; False when two x coincide."""
    cdef double f01, f12
    if x0 == x1 or x1 == x2 or x0 == x2:return False
    f01 = (y1-y0)/(x1-x0);f12 = (y2-y1)/(x2-x1)
    c[0] = (f12-f01)/(x2-x0)
    c[1] = f01-c[0]*(x0+x1)
    c[2] = y0-x0*(f01-c[0]*x1)
    return True


cdef int quadratic_roots(double c2, double c1, double c0, double* r) noexcept:
    """em_march.quadratic_roots: real roots (np.roots with |imag| < 1e-12); returns the count."""
    cdef double disc, sq, q
    if c2 == 0.:
        if c1 != 0.:
            r[0] = -c0/c1;return 1
        return 0
    disc = c1*c1-4.*c2*c0
    if disc >= 0.:
        sq = sqrt(disc)
        q = -.5*(c1+copysign(sq, c1))
        if q == 0.:
            r[0] = 0.;r[1] = 0.;return 2
        r[0] = q/c2;r[1] = c0/q;return 2
    if sqrt(-disc)/(2.*fabs(c2)) < 1e-12:
        r[0] = -c1/(2.*c2);r[1] = r[0];return 2
    return 0


cdef class NColumn:
    """One envelope column of the native trace (the column record is built after the trace)."""
    cdef public double speed
    cdef public int status
    cdef public object status_text, reason, march_reason
    cdef public int boundary, level
    cdef public list points           # state ids (sorted by load) for march columns
    cdef public object record         # the Python column dict (excluded / discontinuity / fallback columns, or built)
    cdef public list trial_log, evidence
    cdef public int ntrials
    cdef public object s3_path, s3_flags
    cdef public double elapsed
    cdef public dict search_work
    cdef public object role
    cdef public long evaluations
    cdef public bint withheld, march, region
    cdef public list sample_points, evidence_list
    cdef public object raw_status, raw_reason
    cdef public object nudged_from     # S6: the requested speed when this column was placed within the speed tolerance
    cdef public int lower              # S8: the lower edge of a column above a forward-IAS limit at level flight (state), or -1
    cdef public double lower_slope     # S8: dw/dV (w = n^2-1, per km/h) of that edge along the forward-IAS limit (NaN: none)
    cdef public bint jump             # S8: the march met a blade-polar jump (no S6 nudges or robust retries here)
    # S7: the exact slope of the boundary, dn/dV (per km/h) from below / above the column speed (NaN: none), and its kind
    cdef public double slope_l, slope_r
    cdef public object slope_kind

    @cython.boundscheck(True)
    @cython.wraparound(True)
    def __init__(self, double speed):
        self.nudged_from = None
        self.slope_l = NAN;self.slope_r = NAN;self.slope_kind = None
        self.lower = -1;self.jump = False;self.lower_slope = NAN
        self.speed = speed;self.status = STATUS_OTHER;self.status_text = None;self.reason = None;self.march_reason = None
        self.boundary = -1;self.level = -1;self.points = [];self.record = None;self.trial_log = None;self.evidence = []
        self.ntrials = 0;self.s3_path = None;self.s3_flags = None;self.elapsed = 0.;self.search_work = None
        self.role = None;self.evaluations = 0;self.withheld = False;self.march = False;self.region = False
        self.sample_points = [];self.raw_status = None;self.raw_reason = None;self.evidence_list = []

    @cython.boundscheck(True)
    @cython.wraparound(True)
    def status_name(self):
        return STATUS_TEXT[self.status] if self.status != STATUS_OTHER else self.status_text


cdef class NMarch:
    """em_march.RescueMarch at one speed."""
    cdef MarchEngine eng
    cdef public double speed
    cdef double turn_tol, alpha_upper, max_step
    cdef int max_nfev
    cdef PredMap preds
    cdef list samples
    cdef Trial* trials
    cdef int ntrials, cap_trials
    cdef int last_lo
    cdef public object s3_path
    cdef public set s3_flags
    cdef bint store
    cdef public list evidence
    # S6: robust corrections (regime seeds and a larger budget after a failed correction), used only by the
    # S6 rescue of a column that every earlier rescue left unresolved
    cdef public bint robust
    cdef int retries_left
    # S6: a face found by the S5 face rescues must lie on the branch the march follows (face_branch)
    cdef public bint strict_faces
    # S7: no S3 rescue (the edge refinement column: where its plain march fails, the edge stays at the localized column)
    cdef public bint no_rescue
    # S8: a blade-polar jump within reach of a sample's shaft speed was found here (item 5: no S6 probing at such speeds)
    cdef public bint jump_seen
    # S9: the next march step approaches a hard limit's predicted root (no warm start from another speed then)
    cdef bint approach
    # S9: the side (+1 higher speeds, -1 lower, 0 none) of the column's segment start: level flight is continued from
    # that side first (the branch the sweep follows), the cross-speed level store on that side first
    cdef public double pref_side

    @cython.boundscheck(True)
    @cython.wraparound(True)
    def __cinit__(self):
        self.trials = NULL

    @cython.boundscheck(True)
    @cython.wraparound(True)
    def __dealloc__(self):
        if self.trials != NULL:cfree(self.trials)

    @cython.boundscheck(True)
    @cython.wraparound(True)
    def __init__(self, MarchEngine eng, double speed, double turn_tol):
        self.eng = eng;self.speed = speed;self.turn_tol = turn_tol;self.max_step = S8_MAX_STEP_DEG if eng.soft else MAX_STEP_DEG
        if eng.alpha_max_step > 0.:self.max_step = eng.alpha_max_step   # S8: the dense truth's finer alpha march
        self.max_nfev = MARCH_NFEV
        self.alpha_upper = float(eng.solver.trim_bounds[1][0])
        self.preds.n = 0
        self.samples = []
        self.cap_trials = 64;self.trials = <Trial*>malloc(self.cap_trials*sizeof(Trial));self.ntrials = 0
        if self.trials == NULL:raise MemoryError()
        self.last_lo = -1;self.s3_path = None;self.s3_flags = set()
        self.store = True
        self.evidence = []
        self.robust = False
        self.retries_left = 0
        self.strict_faces = False
        self.no_rescue = False
        self.jump_seen = False
        self.approach = False

    # ------------------------------------------------------------------ bookkeeping
    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef Trial* trial(self, why) except NULL:
        cdef Trial* grown
        if self.ntrials >= self.cap_trials:
            grown = <Trial*>realloc(self.trials, 2*self.cap_trials*sizeof(Trial))
            if grown == NULL:raise MemoryError()
            self.trials = grown;self.cap_trials *= 2
        self.ntrials += 1
        t = &self.trials[self.ntrials-1]
        t.why = self.eng.why(why);t.alpha = NAN;t.ok = -1;t.evals = 0;t.fresh = 0;t.warm = -1
        return t

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef inline TState* S(self, int sid) noexcept:
        return &self.eng.st[sid]

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef bint has_sample(self, int sid) except -1:
        cdef int q
        for q in self.samples:
            if q == sid:return True
        return False

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef double margin(self, int sid, int k, bint* present) noexcept:
        cdef TState* t = &self.eng.st[sid]
        present[0] = (t.mset >> k) & 1
        return t.m[k] if present[0] else 0.

    # ------------------------------------------------------------------ the correction of a trial
    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int correct_at(self, int seed, const double* sol, double load, int fixed_kind, double value, int ev_kind,
                        int ev_axis, int ev_side, int max_nfev, int max_evaluations, double tolerance,
                        PredMap* preds, long* evals, long* fresh, int* evidence) except -2:
        cdef long e0 = self.eng.counts[K_EVAL], d0 = self.eng.counts[K_DERIV]
        cdef int sid = self.eng.correct(seed, sol, load, True, fixed_kind, value, ev_kind, ev_axis, ev_side, max_nfev,
                                        max_evaluations, tolerance, self.speed, preds, evidence)
        evals[0] = self.eng.counts[K_EVAL]-e0;fresh[0] = self.eng.counts[K_DERIV]-d0
        return sid

    # ------------------------------------------------------------------ S6: robust corrections
    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int regime_seed(self, int seed, double offset) except -2:
        """A copy of ``seed`` with every propeller governor coordinate moved onto its nearest regime boundary
        (0: minimum pitch, 1: maximum pitch), ``offset`` beyond it; -1 when the seed has none within reach.
        One governor coordinate spans the three governor regimes, so a correction started on the governed
        side of a pitch stop can stall at the kink the state has to cross; started on the boundary it closes."""
        cdef TState* s = &self.eng.st[seed]
        cdef int j, cid
        cdef double g, b
        if not s.has_coords or not s.layout_names:return -1
        moved = []
        for j in range(s.nx):
            if j >= len(self.eng.layout_names) or not self.eng.layout_names[j].startswith('governor:'):continue
            g = s.coords[j]
            b = 0. if g < .5 else 1.
            if fabs(g-b) > REGIME_REACH:continue
            moved.append((j, b-offset if b == 0. else b+offset))
        if not moved:return -1
        cid = self.eng.new_state()
        self.eng.st[cid] = self.eng.st[seed]
        self.eng.st[cid].src = -1;self.eng.st[cid].row = -1;self.eng.st[cid].base = cid;self.eng.st[cid].limit = -1
        self.eng.st[cid].valid = False
        for j, b in moved:self.eng.st[cid].coords[j] = b
        return cid

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int retry(self, int seed, const double* sol, double load, int fixed_kind, double value, int ev_kind,
                   int ev_axis, int ev_side, int max_nfev, int max_evaluations) except -2:
        """After a correction that did not close (robust mode): the same correction from regime-boundary seeds,
        then with a larger budget, each with fresh derivatives. A closed state still passes every admission check."""
        cdef int p = -1, rs
        cdef long evals, fresh
        cdef double offset
        cdef Trial* t
        if self.retries_left <= 0:return -1
        self.retries_left -= 1
        for offset in (0., .01):
            rs = self.regime_seed(seed, offset)
            if rs < 0:break
            p = self.correct_at(rs, sol, load, fixed_kind, value, ev_kind, ev_axis, ev_side, max_nfev, max_evaluations,
                                2e-6, NULL, &evals, &fresh, NULL)
            t = self.trial('s6 regime seed');t.ok = -1 if p < 0 else int(self.eng.st[p].valid);t.evals = evals;t.fresh = fresh
            t.alpha = value if fixed_kind == FIX_ALPHA else (self.eng.st[p].alpha if p >= 0 else NAN)
            self.eng.stats['s6_regime_seeds'] = self.eng.stats.get('s6_regime_seeds', 0)+1
            if p >= 0:
                self.eng.stats['s6_regime_closed'] = self.eng.stats.get('s6_regime_closed', 0)+1
                return p
        p = self.correct_at(seed, sol, load, fixed_kind, value, ev_kind, ev_axis, ev_side, ROBUST_NFEV,
                            ROBUST_EVALS if max_evaluations > 0 else 0, 2e-6, NULL, &evals, &fresh, NULL)
        t = self.trial('s6 larger budget');t.ok = -1 if p < 0 else int(self.eng.st[p].valid);t.evals = evals;t.fresh = fresh
        t.alpha = value if fixed_kind == FIX_ALPHA else (self.eng.st[p].alpha if p >= 0 else NAN)
        self.eng.stats['s6_budget_retries'] = self.eng.stats.get('s6_budget_retries', 0)+1
        if p >= 0:self.eng.stats['s6_budget_closed'] = self.eng.stats.get('s6_budget_closed', 0)+1
        return p

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef object robust_march(self, int level, bint has_predicted, double predicted):
        """S6: where the march and every earlier rescue left this speed unresolved, the march again from the
        highest admitted samples of this speed with robust corrections (regime seeds, larger budget), then the
        S3 rescue with them. Same step control, localization, face guards and first-limit rule; the columns
        that resolve without it never reach it. (edge, status, reason) or None."""
        cdef int lo
        admitted = sorted((p for p in self.samples if self.eng.passable(p) and fabs(self.eng.st[p].speed-self.speed) <= 1e-9
                           and self.eng.st[p].alpha >= self.eng.st[level].alpha-1e-9), key=lambda p: self.eng.st[p].alpha)
        if not admitted:
            if not self.eng.passable(level):return None
            admitted = [level]
        start = [admitted[0]]
        for sid in admitted[1:]:
            # an increasing-alpha, non-decreasing-load history (the march's own invariant)
            if self.eng.st[sid].load >= self.eng.st[start[-1]].load:start.append(sid)
        self.robust = True;self.retries_left = ROBUST_RETRIES
        self.eng.stats['s6_robust_marches'] = self.eng.stats.get('s6_robust_marches', 0)+1
        import numpy as np
        try:
            out = self.run_march(level, has_predicted, predicted, start[-3:])
            if out[0] < 0:
                edge = self.rescue(level)
                if edge is not None and edge[0] >= 0:out = edge
        except (ValueError, ArithmeticError, RuntimeError, np.linalg.LinAlgError) as error:
            errors = self.eng.stats.setdefault('rescue_errors', {});key = type(error).__name__+': '+str(error)[:60]
            errors[key] = errors.get(key, 0)+1;out = (-1, 'unresolved numerical boundary', 'S6 rescue error')
        finally:
            self.robust = False
        if out[0] < 0:return None
        self.s3_path = 's6 robust march'
        self.eng.stats['s6_robust_resolved'] = self.eng.stats.get('s6_robust_resolved', 0)+1
        return out

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int at(self, int seed, const double* sol, double load, double seed_alpha, double alpha, why,
                bint has_base, double base_alpha) except -2:
        """March.at(seed, alpha, why, base_alpha=...). ``seed``/``sol``/``load``/``seed_alpha``: the seed dict
        (a state with its solution and load replaced; seed_alpha its 'alpha_deg')."""
        cdef int p, i, q, warm_state = -1
        cdef double wsol[5]
        cdef double wload
        cdef long evals, fresh, evals2 = 0, fresh2 = 0
        cdef Trial* t
        cdef Predictor* warm_pred = NULL
        cdef StoreRow* best = NULL
        if not has_base:base_alpha = seed_alpha
        if self.store and why == 'march':
            best = self.warm(seed, alpha, base_alpha)
            if best != NULL:
                self.eng.stats['predictors_offered'] += 1
                warm_state = best.state
                for i in range(5):wsol[i] = self.eng.st[warm_state].sol[i]
                wsol[0] = alpha
                wload = self.eng.st[warm_state].load
                if best.has_pred and self.eng.has_alpha_key:
                    pm_set(&self.preds, &self.eng.alpha_key, &best.pred)
        if warm_state >= 0:
            p = self.correct_at(warm_state, wsol, wload, FIX_ALPHA, alpha, 0, 0, 0, self.max_nfev, MARCH_EVALS, 2e-6,
                                &self.preds, &evals, &fresh, NULL)
            if self.eng.s9 and p >= 0 and self.eng.is_prop and self.prop_jump(seed, p):
                # S9: the warm start from another speed landed on another propeller branch (wash / governor state) than
                # the column's own samples (Ki-43-III SB 137-143 km/h): the step from the column's own sample instead,
                # where that closes on its own branch
                q = self.correct_at(seed, sol, load, FIX_ALPHA, alpha, 0, 0, 0, self.max_nfev, MARCH_EVALS, 2e-6,
                                    NULL, &evals2, &fresh2, NULL)
                evals += evals2;fresh += fresh2
                if q >= 0 and not self.prop_jump(seed, q):
                    self.eng.stats['s9_warm_branches'] = self.eng.stats.get('s9_warm_branches', 0)+1
                    p = q
        else:
            p = self.correct_at(seed, sol, load, FIX_ALPHA, alpha, 0, 0, 0, self.max_nfev, MARCH_EVALS, 2e-6,
                                &self.preds, &evals, &fresh, NULL)
        if p < 0 and self.robust:
            p = self.retry(seed, sol, load, FIX_ALPHA, alpha, 0, 0, 0, self.max_nfev, MARCH_EVALS)
        if (p < 0 or (not self.eng.passable(p) and not self.eng.hard_rejected(p))) and self.eng.soft and self.eng.is_prop:
            # S8: next to a blade-polar jump, the sliding state (from a sliding seed, the regular one)
            q = self.slide_at(seed, sol, load, FIX_ALPHA, alpha, 's8 sliding '+str(why))
            if q >= 0:p = q
        if self.store:self.eng.store_keep(p, self.speed, &self.preds)
        t = self.trial(why)
        t.alpha = alpha;t.ok = -1 if p < 0 else int(self.eng.st[p].valid);t.evals = evals;t.fresh = fresh
        t.warm = 1 if (best != NULL and best.has_pred) else 0
        if p >= 0 and self.eng.passable(p):self.samples.append(p)
        return p

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int slide_at(self, int low, const double* sol, double load, int fixed_kind, double value, why) except -2:
        """S8 (items 2-3): a correction that did not close next to a blade-polar jump. From a regular propeller seed whose
        blade polar jumps within reach of its shaft speed (MarchEngine.slide_candidate), the same correction with the
        shafts sliding on the jump (the Filippov state: theta in [0, 1]); from a sliding seed, the same correction regular
        (the band may end there). The state, or -1."""
        cdef SlideSpec spec
        cdef int p
        cdef long evals, fresh
        cdef Trial* t
        if not self.eng.sliding or not self.eng.is_prop:return -1
        if self.eng.st[low].slide.n > 0:
            spec.n = 0
            if self.eng.s9:
                # S9: from a sliding seed, the jump check again at the target: the shafts still blocked keep sliding, the
                # others are regular (a set equal to the seed's is the correction that failed: then all regular, as S8)
                if fixed_kind == FIX_LOAD:self.eng.slide_candidate(low, &spec, sol[0], sol[1], value, True, self.speed, True)
                else:self.eng.slide_candidate(low, &spec, value, sol[1], load, True, self.speed, True)
                if spec.n > 0 and self.eng.same_slide(&spec, &self.eng.st[low].slide):spec.n = 0
        else:
            # the jump check at the correction's own target (alpha, bank and load of its seed)
            if fixed_kind == FIX_LOAD:self.eng.slide_candidate(low, &spec, sol[0], sol[1], value, True, self.speed)
            else:self.eng.slide_candidate(low, &spec, value, sol[1], load, True, self.speed)

            if spec.n == 0:return -1
        self.eng.slide_override = spec;self.eng.use_override = True
        try:
            p = self.correct_at(low, sol, load, fixed_kind, value, 0, 0, 0, ROBUST_NFEV, ROBUST_EVALS, 2e-6, NULL, &evals,
                                &fresh, NULL)
        finally:
            self.eng.use_override = False
        t = self.trial(why)
        t.alpha = value if fixed_kind == FIX_ALPHA else (self.eng.st[p].alpha if p >= 0 else NAN)
        t.ok = -1 if p < 0 else int(self.eng.st[p].valid);t.evals = evals;t.fresh = fresh
        key = 's8_slide_closed' if spec.n else 's8_unslide_closed'
        if p >= 0 and self.eng.passable(p):
            self.eng.stats[key] = self.eng.stats.get(key, 0)+1
            # the jump is confirmed as the cause where its sliding state closes (item 5: no S6 probing here)
            if spec.n:self.jump_seen = True
            return p
        return -1

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef bint prop_jump(self, int a, int b) noexcept:
        """S9: the propulsion coordinates of state ``b`` (shaft, governor and wash coordinates, after the seven attitude and
        wing-history ones) differ from those of ``a`` by more than S9_PROP_JUMP of their size (at least 1): another
        propeller branch. False where the layouts differ or either state slides on a blade-polar jump."""
        cdef TState* s = &self.eng.st[a]
        cdef TState* t = &self.eng.st[b]
        cdef int i
        if not (s.has_coords and t.has_coords) or s.nx != t.nx or s.nx <= 7 or s.slide.n > 0 or t.slide.n > 0:return False
        for i in range(7, s.nx):
            if fabs(t.coords[i]-s.coords[i]) > S9_PROP_JUMP*pmax(1., fabs(s.coords[i])):return True
        return False

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int at_state(self, int seed, double alpha, why) except -2:
        """at(state, alpha, why) with the state itself as the seed. S8: the solution is copied (the state pool can
        move when a correction adds states, and the robust retry reads the seed's solution again)."""
        cdef double sol[5]
        cdef int i
        for i in range(5):sol[i] = self.eng.st[seed].sol[i]
        return self.at(seed, sol, self.eng.st[seed].load, self.eng.st[seed].alpha, alpha, why, False, 0.)

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef StoreRow* warm(self, int seed, double alpha, double base_alpha) noexcept:
        """March.warm: the nearest neighbouring-speed state (same roll-leveling side and request branch)."""
        cdef StoreRow* best = NULL
        cdef StoreRow* r
        cdef double score = fabs(base_alpha-alpha), d
        cdef int i, k
        cdef TState* s = &self.eng.st[seed]
        cdef TState* q
        cdef bint same
        for i in range(self.eng.nstore):
            r = &self.eng.store[i]
            if r.speed == self.speed or fabs(r.speed-self.speed) > REUSE_WINDOW:continue
            if self.eng.ncut and self.eng.across(r.speed, self.speed):continue
            if side12(r.alpha) != side12(alpha):continue
            q = &self.eng.st[r.state]
            if q.nreq != s.nreq:continue
            same = True
            for k in range(q.nreq):
                if q.req[k] != s.req[k]:same = False
            if not same:continue
            d = fabs(r.alpha-alpha)+fabs(r.speed-self.speed)/25.
            if d < score:
                best = r;score = d
        return best

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef double snap(self, int low, double target) noexcept:
        cdef double lo = self.eng.st[low].alpha+.7*(target-self.eng.st[low].alpha), best = 0.
        cdef bint found = False
        cdef StoreRow* r
        cdef int i
        for i in range(self.eng.nstore):
            r = &self.eng.store[i]
            if r.speed == self.speed or fabs(r.speed-self.speed) > REUSE_WINDOW:continue
            if self.eng.ncut and self.eng.across(r.speed, self.speed):continue
            if lo <= r.alpha and r.alpha <= target and fabs(r.speed-self.speed) <= REUSE_WINDOW/2:
                if not found or r.alpha > best:best = r.alpha
                found = True
        return best if found else target

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int level_from(self, int neighbour) except -2:
        """March.level_from: level flight by the prescribed-load correction from a neighbouring level state."""
        cdef StoreRow* r
        cdef StoreRow* best = NULL
        cdef int i, p, k, q
        cdef long evals, fresh
        cdef Trial* t
        cdef double d, bd = 0.
        cdef PKey lk
        if self.eng.has_level_key:
            for i in range(self.eng.nlevel):
                r = &self.eng.level_store[i]
                if r.speed == self.speed or (self.eng.dive_seed and r.speed > self.speed):continue
                if self.eng.ncut and self.eng.across(r.speed, self.speed):continue
                d = fabs(r.speed-self.speed)
                if best == NULL or d < bd:
                    best = r;bd = d
            if self.pref_side != 0.:
                # S9: the nearest stored level flight on the segment start's side (within the reuse window), if any
                for i in range(self.eng.nlevel):
                    r = &self.eng.level_store[i]
                    if r.speed == self.speed or (self.eng.dive_seed and r.speed > self.speed):continue
                    if self.eng.ncut and self.eng.across(r.speed, self.speed):continue
                    if (r.speed-self.speed)*self.pref_side <= 0. or fabs(r.speed-self.speed) > REUSE_WINDOW:continue
                    d = fabs(r.speed-self.speed)
                    if (best.speed-self.speed)*self.pref_side <= 0. or d < bd:
                        best = r;bd = d
            if best != NULL and fabs(best.speed-self.speed) <= REUSE_WINDOW:
                neighbour = best.state
                pm_set(&self.preds, &self.eng.level_key, &best.pred)
        p = self.correct_at(neighbour, self.eng.st[neighbour].sol, self.eng.st[neighbour].load, FIX_LOAD, 1., 0, 0, 0,
                            self.max_nfev, MARCH_EVALS, 2e-6, &self.preds, &evals, &fresh, NULL)
        if (p < 0 or not (self.eng.st[p].valid or self.eng.ias_soft(p))) and self.eng.soft and self.eng.is_prop:
            # S8: level flight next to a blade-polar jump: the sliding state (from a sliding level flight, the regular one)
            q = self.slide_at(neighbour, self.eng.st[neighbour].sol, self.eng.st[neighbour].load, FIX_LOAD, 1., 's8 sliding level')
            if q >= 0:p = q
        t = self.trial('level correction')
        t.ok = -1 if p < 0 else int(self.eng.st[p].valid);t.evals = evals;t.fresh = fresh
        if p >= 0 and (self.eng.st[p].valid or self.eng.ias_soft(p)):
            k = -1
            for i in range(self.preds.n):
                if self.preds.key[i].fixed != 0:
                    k = i;break
            if k >= 0:
                self.eng.level_key = self.preds.key[k];self.eng.has_level_key = True
                if self.eng.nlevel < 201:
                    self.eng.level_store[self.eng.nlevel].speed = self.speed;self.eng.level_store[self.eng.nlevel].alpha = 0.
                    self.eng.level_store[self.eng.nlevel].state = p;self.eng.level_store[self.eng.nlevel].has_pred = True
                    self.eng.level_store[self.eng.nlevel].pred = self.preds.pred[k]
                    self.eng.nlevel += 1
                if self.eng.nlevel > 200:
                    for i in range(self.eng.nlevel-50):self.eng.level_store[i] = self.eng.level_store[i+50]
                    self.eng.nlevel -= 50
        # S8: a level flight over a forward-IAS limit (only) starts the march too (its states at higher alpha can be inside)
        return p if p >= 0 and (self.eng.st[p].valid or self.eng.ias_soft(p)) else -1

    # ------------------------------------------------------------------ step control
    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef bint margin_root(self, list history, int kind, double* root) except -1:
        """RescueMarch.margin_root."""
        cdef double xs[3]
        cdef double ys[3]
        cdef int n = 0, i, nr = 0, k
        cdef double a, c[3]
        cdef double roots[4]
        cdef double qr[2]
        cdef bint present, found = False
        cdef double best = 0., v
        for sid in history[-3:]:
            v = self.margin(sid, kind, &present)
            if present:
                xs[n] = self.eng.st[sid].alpha;ys[n] = v;n += 1
        a = self.eng.st[history[-1]].alpha
        if n >= 2:
            if ys[n-1] < ys[n-2] and xs[n-1] > xs[n-2]:
                roots[nr] = xs[n-1]+ys[n-1]*(xs[n-1]-xs[n-2])/(ys[n-2]-ys[n-1]);nr += 1
        if n == 3:
            if parabola(xs[0], ys[0], xs[1], ys[1], xs[2], ys[2], c):
                k = quadratic_roots(c[0], c[1], c[2], qr)
                for i in range(k):
                    if qr[i] > a:
                        roots[nr] = qr[i];nr += 1
            else:
                for value in polyfit_roots([xs[0], xs[1], xs[2]], [ys[0], ys[1], ys[2]], a):
                    roots[nr] = value;nr += 1
                    if nr >= 4:break
        for i in range(nr):
            if roots[i] > a:
                if not found or roots[i] < best:best = roots[i]
                found = True
        root[0] = best
        return found

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef double margin_dip(self, int h0, int h1, int h2) noexcept:
        """S8: the split rule along alpha on the march step [h1, h2] (h0 the sample before): a hard-limit margin
        positive at both ends whose band (band_load from its values and slopes at both ends; the slopes of the parabola
        through the three samples' margins) reaches zero inside the step could hide a limit between the samples: the
        alpha where the lowest such band is lowest, NaN where none."""
        cdef double x0 = self.eng.st[h0].alpha, x1 = self.eng.st[h1].alpha, x2 = self.eng.st[h2].alpha
        cdef double y0, y1, y2, c[3], s1, s2, x, lo, hi, best = 0., at = NAN
        cdef bint p0, p1, p2
        cdef int k, j
        if not (x0 < x1 < x2):return NAN
        for k in range(NKD):
            if self.eng.soft_kind(k):continue
            y0 = self.margin(h0, k, &p0);y1 = self.margin(h1, k, &p1);y2 = self.margin(h2, k, &p2)
            if not (p0 and p1 and p2) or y1 <= 0. or y2 <= 0.:continue
            if not parabola(x0, y0, x1, y1, x2, y2, c):continue
            s1 = 2*c[0]*x1+c[1];s2 = 2*c[0]*x2+c[1]
            for j in range(1, S8_SAMPLES):
                x = x1+(x2-x1)*j/S8_SAMPLES
                band_load(x1, y1, s1, x2, y2, s2, x, &lo, &hi)
                if lo < best:
                    best = lo;at = x
        return at

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef bint trim_jump(self, int h0, int h1, int h2) noexcept:
        """S9: the elevator command over the march step [h1, h2] (h0 the sample before) does not continue the previous
        step: off the linear prediction by more than S9_TRIM_DEV, with a secant slope that differs from the previous one
        by more than that slope's size (or S9_TRIM_SLOPE). A smooth trim branch passes once the steps are short; a fold
        or a jump to another branch does not."""
        cdef double x0 = self.eng.st[h0].alpha, x1 = self.eng.st[h1].alpha, x2 = self.eng.st[h2].alpha
        cdef double e0 = self.eng.st[h0].sol[3], e1 = self.eng.st[h1].sol[3], e2 = self.eng.st[h2].sol[3], d1, d2
        if not (x0 < x1-1e-9 and x1 < x2-1e-9):return False
        # a trim fold is where the pitch response vanishes: only while its margin is falling (by S9_TRIM_PITCH over the
        # step before; the elevator's own bend towards the stall leaves it flat)
        if not ((self.eng.st[h0].mset >> KD_PITCH) & 1 and (self.eng.st[h1].mset >> KD_PITCH) & 1
                and self.eng.st[h1].m[KD_PITCH] < S9_TRIM_PITCH*self.eng.st[h0].m[KD_PITCH]):return False
        d1 = (e1-e0)/(x1-x0);d2 = (e2-e1)/(x2-x1)
        if fabs(e2-e1-d1*(x2-x1)) <= S9_TRIM_DEV:return False
        return fabs(d2-d1) > pmax(fabs(d1), S9_TRIM_SLOPE)

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef double next_alpha(self, list history, double step, bint has_predicted, double predicted) except? -1e300:
        """RescueMarch.next_alpha."""
        cdef int low = history[-1], k
        cdef double a = self.eng.st[low].alpha, target, root, c[3], vertex, c2, c1
        cdef TState* t = &self.eng.st[low]
        target = a+step
        self.approach = False
        if has_predicted and predicted > a+.05:target = pmin(target, predicted+pmax(OVERSHOOT_DEG, .02*(predicted-a)))
        if len(history) >= 2:
            for k in range(NKD):
                if not (t.mset >> k) & 1 or self.eng.soft_kind(k):continue
                if not self.margin_root(history, k, &root):continue
                if root-a <= 2*self.max_step:self.approach = True
                if root-a > APPROACH_DEG:target = pmin(target, a+.75*(root-a))
                else:target = pmin(target, root+pmax(OVERSHOOT_DEG, .03*(root-a)))
        if len(history) >= 3:
            h0, h1, h2 = history[-3], history[-2], history[-1]
            if parabola(self.eng.st[h0].alpha, self.eng.st[h0].load, self.eng.st[h1].alpha, self.eng.st[h1].load,
                        self.eng.st[h2].alpha, self.eng.st[h2].load, c):
                c2 = c[0];c1 = c[1]
            else:
                c2, c1 = polyfit2([self.eng.st[h0].alpha, self.eng.st[h1].alpha, self.eng.st[h2].alpha],
                                  [self.eng.st[h0].load, self.eng.st[h1].load, self.eng.st[h2].load])
            if c2 < 0.:
                vertex = -c1/(2*c2)
                if vertex > a:target = pmin(target, vertex+pmax(OVERSHOOT_DEG, .25*(vertex-a)))
        if a < SWITCH_DEG-1e-3 and SWITCH_DEG-1e-3 < target:
            target = SWITCH_DEG-ROLL_PROBE
        return pmin(target, self.alpha_upper)

    # ------------------------------------------------------------------ localization
    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef bint closed(self, int lo, int hi) noexcept:
        if fabs(turn_rate(self.speed, self.eng.st[hi].load)-turn_rate(self.speed, self.eng.st[lo].load)) > self.turn_tol:return False
        # S9: the turn rates of the two states bound the face's only where the rejected state continues the admitted
        # one's branch; past a stall it lies on another (the load drops back, J35A SB 152.6 km/h), so the bracket
        # closes in alpha too (0.1 deg)
        if self.eng.s9 and fabs(self.eng.st[hi].alpha-self.eng.st[lo].alpha) > .1:return False
        return True

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef tuple margin_edge(self, int lo, int hi):
        """March.margin_edge: (edge state or -1, reason or None)."""
        cdef double flo = 0., fhi = 0., w, t, slope, d
        cdef bint has_flo, has_fhi
        cdef int side = 0, it, p, kind, axis
        cdef bint done = False
        has_flo = self.eng.combined(lo, &flo);has_fhi = self.eng.combined(hi, &fhi)
        for it in range(24 if self.eng.s9 else 16):
            if self.closed(lo, hi):
                done = True;break
            w = self.eng.st[hi].alpha-self.eng.st[lo].alpha
            if w <= 1e-6:
                done = True;break
            if has_flo and has_fhi and flo > 0. and 0. > fhi:
                t = flo/(flo-fhi)
            else:t = .5
            t = pmin(.9, pmax(.1, t)) if it % 4 != 3 else .5
            slope = fabs(self.eng.st[hi].turn-self.eng.st[lo].turn)/w
            if it % 4 != 3 and slope > 0.:
                d = .4*self.turn_tol/slope/w
                t = t-d if side >= 0 else t+d
                t = pmin(.95, pmax(.05, t))
            p = self.at_state(lo, self.eng.st[lo].alpha+t*w, 'localize')
            if p < 0 or (not self.eng.passable(p) and not self.eng.hard_rejected(p)):
                p = self.at_state(lo, self.eng.st[lo].alpha+.5*w, 'localize bisect')
                if p < 0 or (not self.eng.passable(p) and not self.eng.hard_rejected(p)):
                    self.last_lo = lo
                    return -1, 'localization correction unresolved'
            if self.eng.passable(p):
                lo = p;has_flo = self.eng.combined(p, &flo)
                if side == 1 and has_fhi:fhi *= .5
                side = 1
                if self.eng.st[hi].alpha-self.eng.st[lo].alpha <= 1e-3*pmax(1., w):
                    self.last_lo = lo
                    return -1, 'false bracket'
            else:
                hi = p;has_fhi = self.eng.combined(p, &fhi)
                if side == -1 and has_flo:flo *= .5
                side = -1
        if not done:
            self.last_lo = lo
            return -1, 'localization budget'
        if not self.closed(lo, hi):
            self.last_lo = lo
            return -1, 'localization did not close'
        kind = self.eng.limit_kind(hi)
        axis = self.eng.control_axis(lo) if kind == KD_CONTROL else -1
        details = dict(method='S1 alpha-march margin bracket',
                       alpha_bracket_deg=[self.eng.st[lo].alpha, self.eng.st[hi].alpha],
                       limiting_load_interval_g=[self.eng.st[lo].load, self.eng.st[hi].load],
                       turn_interval_dps=[self.eng.st[lo].turn, self.eng.st[hi].turn],
                       rejected_endpoint=self.endpoint(hi, False))
        return self.eng.mark(lo, kind, axis, details), None

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef dict endpoint(self, int sid, bint with_speed):
        cdef TState* t = &self.eng.st[sid]
        reasons = [REASON_TEXT[t.reasons[i]] for i in range(t.nreasons)]
        fe = t.force_error_g if isfinite(t.force_error_g) else None
        ae = t.angular_error if isfinite(t.angular_error) else None
        if t.row >= 0 and self.eng.rows[sid] is not None:
            row = self.eng.rows[sid]
            reasons = row['reasons'];fe = row['force_error_g'];ae = row['angular_error_rad_s2']
        if with_speed:
            return dict(speed_kmh=t.speed, load_g=t.load, solution=[t.sol[i] for i in range(5)], reasons=reasons,
                        force_error_g=fe, angular_error_rad_s2=ae)
        return dict(load_g=t.load, alpha_deg=t.alpha, reasons=reasons, force_error_g=fe, angular_error_rad_s2=ae)

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef tuple fold_edge(self, list points, int budget=14):
        """March.fold_edge: (edge or -1, reason or None, bracket (a, p) or None)."""
        import numpy as np
        cdef int it, i, j, n, a, b, c, p
        cdef double tb, v, lo_a, hi_a
        pts = []
        alphas = []
        for sid in points:
            v = self.eng.st[sid].alpha
            if v in alphas:pts[alphas.index(v)] = sid
            else:
                alphas.append(v);pts.append(sid)
        pts.sort(key=lambda q: self.eng.st[q].alpha)
        for it in range(budget):
            n = len(pts);i = 0
            for j in range(1, n):
                if self.eng.st[pts[j]].load > self.eng.st[pts[i]].load:i = j
            if i == 0 or i == n-1:return -1, 'fold maximum not bracketed', None
            a = pts[i-1];b = pts[i];c = pts[i+1]
            tb = self.eng.st[b].turn
            if (tb-self.eng.st[a].turn <= self.turn_tol and tb-self.eng.st[c].turn <= self.turn_tol and
                    fabs(self.eng.st[a].ps-self.eng.st[b].ps) <= self.eng.sep_tol and
                    fabs(self.eng.st[c].ps-self.eng.st[b].ps) <= self.eng.sep_tol):
                details = dict(method='S1 alpha-march load maximum',
                               alpha_bracket_deg=[self.eng.st[a].alpha, self.eng.st[c].alpha],
                               neighboring_loads_g=[self.eng.st[a].load, self.eng.st[c].load])
                return self.eng.mark(b, KD_FOLD, -1, details), None, None
            xs = np.array([self.eng.st[a].alpha, self.eng.st[b].alpha, self.eng.st[c].alpha])
            ys = np.array([self.eng.st[a].load, self.eng.st[b].load, self.eng.st[c].load])
            c2, c1, _ = np.polyfit(xs, ys, 2)
            vv = -c1/(2*c2) if c2 < 0. else .5*(xs[0]+xs[2])
            lo_v, hi_v = xs[0]+.1*(xs[2]-xs[0]), xs[2]-.1*(xs[2]-xs[0])
            if it % 3 == 2 or not lo_v < vv < hi_v or abs(vv-xs[1]) < 1e-4:
                vv = .5*(xs[0]+xs[1]) if xs[1]-xs[0] > xs[2]-xs[1] else .5*(xs[1]+xs[2])
            p = self.at_state(b, float(vv), 'fold')
            if p < 0:return -1, 'fold correction unresolved', None
            if not self.eng.passable(p):
                if self.eng.hard_rejected(p) and vv < xs[1]:return -1, 'rejection before load maximum', (a, p)
                return -1, ('fold bracket inconsistent' if self.eng.hard_rejected(p) else 'fold correction unresolved'), None
            pts.append(p);pts.sort(key=lambda q: self.eng.st[q].alpha)
        return -1, 'fold budget', None

    # ------------------------------------------------------------------ the march
    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef tuple extrapolated(self, list history, double target, double cap):
        """March.extrapolated: (state, solution, load) of the seed for the next sample."""
        cdef int low = history[-1], prev
        cdef double h, r, n
        cdef list x
        if len(history) < 2:return low, None, 0.
        prev = history[-2]
        h = self.eng.st[low].alpha-self.eng.st[prev].alpha
        if h <= 1e-9:return low, None, 0.
        r = (target-self.eng.st[low].alpha)/h
        x = [self.eng.st[low].sol[i]+r*(self.eng.st[low].sol[i]-self.eng.st[prev].sol[i]) for i in range(5)]
        x[0] = target
        n = pmin(pmax(self.eng.st[low].load+r*(self.eng.st[low].load-self.eng.st[prev].load), 1.001), cap)
        return low, x, n

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef tuple run(self, int level, bint has_predicted, double predicted):
        """RescueMarch.run(level, predicted): (boundary state or -1, status, reason)."""
        out = self.run_march(level, has_predicted, predicted)
        if out[0] >= 0 or out[2] not in NATIVE_RESCUE_REASONS or self.no_rescue:return out
        import numpy as np
        try:
            edge = self.rescue(level)
        except (ValueError, ArithmeticError, RuntimeError, np.linalg.LinAlgError) as error:
            errors = self.eng.stats.setdefault('rescue_errors', {});key = type(error).__name__+': '+str(error)[:60]
            errors[key] = errors.get(key, 0)+1;edge = None
        if edge is None:
            self.eng.stats['s3_unresolved'] = self.eng.stats.get('s3_unresolved', 0)+1
            return out
        return edge

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef tuple run_march(self, int level, bint has_predicted, double predicted, list start=None):
        """March.run(level, predicted, history=start)."""
        cdef double span, step, cap, target, dip
        cdef int low, p, q, tried_active = -1, lo, edge, seed_state
        cdef double sol[5]
        cdef double load
        cdef bint extrapolated_seed, at_cap = False
        cdef int it, i
        history = list(start) if start else [level]
        for h in history:
            if not self.has_sample(h):self.samples.append(h)
        if has_predicted:
            span = predicted-self.eng.st[history[-1]].alpha if start else predicted-self.eng.st[level].alpha
            step = self.max_step if span <= 0 else span/pmax(1., ceil(span/self.max_step))
        else:
            step = self.max_step
        cap = self.eng.cap(self.speed)
        for it in range(60):
            low = history[-1]
            target = self.next_alpha(history, step, has_predicted, predicted)
            if target <= self.eng.st[low].alpha+1e-6:return -1, 'unresolved numerical boundary', 'alpha search bound reached'
            if self.store:target = self.snap(low, target)
            seed_state, x, load = self.extrapolated(history, target, cap)
            extrapolated_seed = x is not None
            if extrapolated_seed:
                for i in range(5):sol[i] = x[i]
            else:
                for i in range(5):sol[i] = self.eng.st[low].sol[i]
                load = self.eng.st[low].load
            # S9: approaching a hard limit, the step is corrected from this column's own samples only: a warm start from
            # another speed's state at the same alpha can land on another equilibrium branch and pass the limit the
            # branch continued from level flight meets (F-16XL SB 174 km/h: the pitch-response reversal)
            p = self.at(seed_state, sol, load, self.eng.st[seed_state].alpha, target,
                        'march near' if self.eng.s9 and self.approach else 'march', True, self.eng.st[low].alpha)
            # S8: a prescribed-alpha correction that ends on the load cap (its turn coordinate is bounded there) shows
            # that the cap lies before this alpha
            at_cap = p < 0 and isfinite(self.eng.last_load) and self.eng.last_load >= self.eng.last_cap*(1.-1e-6)
            if p >= 0 and self.eng.hard_rejected(p) and (extrapolated_seed or self.trials[self.ntrials-1].warm == 1):
                q = self.at_state(low, target, 'march confirm')
                if q >= 0 and self.eng.passable(q):
                    step = .5*(target-self.eng.st[low].alpha);has_predicted = False
                    if step < .05:return -1, 'unresolved numerical boundary', 'march roots disagree'
                    continue
                if q >= 0 and self.eng.hard_rejected(q):p = q
            if p < 0 or (not self.eng.passable(p) and not self.eng.hard_rejected(p)):
                if low != tried_active:
                    tried_active = low
                    edge = self.hard_face(history, target)
                    if edge >= 0:return self.finish_face(history, edge)
                if self.eng.st[low].load >= cap-1e-6 or (p >= 0 and self.has_reason(p, R_LOAD_CAP)) or at_cap:
                    q = self.correct_plain(low, FIX_LOAD, cap)
                    if q >= 0 and self.eng.passable(q) and self.eng.st[q].alpha >= self.eng.st[low].alpha-1e-6:
                        self.samples.append(q)
                        return q, 'plot ceiling', None
                step = .5*(target-self.eng.st[low].alpha)
                if step < .05:return -1, 'unresolved numerical boundary', 'march correction unresolved'
                has_predicted = False
                continue
            if self.eng.s9 and self.eng.passable(p) and len(history) >= 2 and self.trim_jump(history[-2], low, p):
                # S9: the elevator command leaves the branch continued from level flight between the samples (its secant
                # slope more than doubles and the step lands off its linear prediction): a trim fold (pitch-response
                # reversal) or a jump to another trim branch can lie between them (Hunter SB 1110-1124 km/h: the fold at
                # 7.03 deg, no trim to 7.2 deg, a second branch above): the step is halved; a correction past a fold
                # fails there and the pitch face is found from the last sample (hard_face)
                self.eng.stats['s9_trim_splits'] = self.eng.stats.get('s9_trim_splits', 0)+1
                step = .5*(self.eng.st[p].alpha-self.eng.st[low].alpha);has_predicted = False
                if step >= .01:continue
                edge = self.hard_face(history, self.eng.st[p].alpha)
                if edge >= 0:return self.finish_face(history, edge)
            if self.eng.soft and self.eng.passable(p) and len(history) >= 2:
                # S8: the split rule along alpha: where a hard limit could lie between the samples, a sample there
                dip = self.margin_dip(history[-2], low, p)
                if isfinite(dip):
                    self.eng.stats['s8_alpha_splits'] = self.eng.stats.get('s8_alpha_splits', 0)+1
                    q = self.at_state(low, dip, 'march split')
                    if q >= 0 and self.eng.hard_rejected(q):
                        self.eng.stats['s8_alpha_split_limits'] = self.eng.stats.get('s8_alpha_split_limits', 0)+1
                        p = q
                    elif q >= 0 and self.eng.passable(q) and self.eng.st[q].load >= self.eng.st[low].load:
                        history.append(q);low = q
            if not self.eng.passable(p):
                self.last_lo = -1
                edge, why = self.margin_edge(low, p)
                if edge < 0 and why == 'false bracket':
                    history.append(self.last_lo);has_predicted = False
                    continue
                if edge < 0:
                    lo = self.last_lo if self.last_lo >= 0 and self.eng.st[self.last_lo].alpha > self.eng.st[low].alpha else -1
                    edge = self.hard_face(history+([lo] if lo >= 0 else []), self.eng.st[p].alpha)
                    if edge < 0:return -1, 'unresolved numerical boundary', why
                    if lo >= 0:history = history+[lo]
                return self.finish_face(history, edge)
            if self.eng.st[p].load < self.eng.st[low].load:
                if len(history) < 2:
                    q = self.at_state(low, .5*(self.eng.st[low].alpha+self.eng.st[p].alpha), 'march')
                    if q < 0 or not self.eng.passable(q) or self.eng.st[q].load <= self.eng.st[low].load:
                        return -1, 'unresolved numerical boundary', 'load decreases from level flight'
                    history.append(q)
                return self.finish_fold(history+[p], -1)
            history.append(p)
            step = pmin(self.max_step, pmax(step, self.eng.st[p].alpha-self.eng.st[low].alpha)*1.5)
        return -1, 'unresolved numerical boundary', 'march budget'

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef bint has_reason(self, int sid, int code) noexcept:
        cdef int i
        for i in range(self.eng.st[sid].nreasons):
            if self.eng.st[sid].reasons[i] == code:return True
        return False

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int correct_plain(self, int seed, int fixed_kind, double value) except -2:
        """self.curve.correct(low, load=cap): defaults (max_nfev 40, no evaluation budget), no trial record."""
        cdef long evals, fresh
        return self.correct_at(seed, self.eng.st[seed].sol, self.eng.st[seed].load, fixed_kind, value, 0, 0, 0, 40, 0,
                               2e-6, &self.preds, &evals, &fresh, NULL)

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef tuple finish_face(self, list history, int edge):
        """RescueMarch.finish_face."""
        cdef int top = history[0]
        for h in history:
            if self.eng.st[h].load > self.eng.st[top].load:top = h
        if self.eng.st[top].load > self.eng.st[edge].load+1e-9:
            if self.eng.st[top].turn-self.eng.st[edge].turn > self.turn_tol:return self.finish_fold(history, edge)
            self.eng.stats['fold_within_tolerance'] = self.eng.stats.get('fold_within_tolerance', 0)+1
            self.s3_flags.add('load maximum within tolerance of face')
        return edge, 'verified limit', None

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef tuple finish_fold(self, list history, int face):
        edge, why, bracket = self.fold_edge(history if face < 0 else history+[face])
        if edge >= 0:return edge, 'verified limit', None
        if bracket is not None:
            e, w = self.margin_edge(bracket[0], bracket[1])
            if e >= 0:return e, 'verified limit', None
            why = w
        return -1, 'unresolved numerical boundary', why

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef bint over(self, int sid, unsigned long long mask) noexcept:
        """S8: rejected (among others) for a reason in ``mask``."""
        cdef int i, r
        for i in range(self.eng.st[sid].nreasons):
            r = self.eng.st[sid].reasons[i]
            if r < 64 and (mask >> r) & 1:return True
        return False

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef tuple settle(self, tuple out):
        """S8: the soft limits at the march's stop. The hard limits stop the march where they first appear; the engine
        RPM limit (Leo, 2026-10-19) and the forward-IAS limits are checked only at that stop. Within them there, an
        over-rev stretch passed on the way is transient (drawn; a forward-IAS stretch is not drawn: its states stand on
        their own forward IAS); over one there, the boundary is where its final stretch begins (the soft face between the
        last sample within it and the first of that stretch), or there is nothing to draw above level flight (a column
        past the leaning forward-IAS edge). Level flight over the RPM limit stays an edge (the column is infeasible)."""
        cdef int edge = out[0], j, w, o, face, kind
        cdef unsigned long long mask
        if edge < 0 or not self.eng.soft or self.eng.st[edge].valid:return out
        if self.eng.has_code(edge, R_RPM):
            mask = (<unsigned long long>1) << R_RPM;kind = KD_RPM
        elif self.over(edge, self.eng.ias_mask):
            mask = self.eng.ias_mask;kind = KD_IAS
        else:return out
        pts = sorted((p for p in self.samples if self.eng.passable(p) and fabs(self.eng.st[p].speed-self.speed) <= 1e-9
                      and self.eng.st[p].alpha < self.eng.st[edge].alpha-1e-9), key=lambda p: self.eng.st[p].alpha)
        j = len(pts)-1
        while j >= 0 and self.over(pts[j], mask):j -= 1
        if j < 0:
            if kind == KD_IAS:return -1, 'no feasible samples', 'forward-IAS limit up to the first hard limit'
            return -1, 'unresolved numerical boundary', 'over-rev from the lowest sample'
        w = pts[j];o = pts[j+1] if j+1 < len(pts) else edge
        face = self.soft_edge(w, o, mask, kind)
        key = 's8_rpm_final_stretch' if kind == KD_RPM else 's8_ias_final_stretch'
        self.eng.stats[key] = self.eng.stats.get(key, 0)+1
        if face < 0:return -1, 'unresolved numerical boundary', 'final soft-limit stretch unresolved'
        return face, 'verified limit', None

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int soft_edge(self, int lo, int hi, unsigned long long mask, int kind) except -2:
        """S8: a soft-limit face between ``lo`` (within it) and ``hi`` (over it): for a forward-IAS limit the event
        correction from ``lo`` (native event 7, exact), else (and where that fails) prescribed-alpha bisection, the secant
        of the limit's margin first, to the march tolerance in turn rate; the face is the last state within the limit
        (a hard rejection found inside is that hard limit's own edge). Marked with ``kind``."""
        cdef int it, p, k, engine = 0, q
        cdef double w, t, flo, fhi
        cdef bint has_lo, has_hi
        cdef long evals, fresh
        if kind == KD_IAS and isfinite(self.eng.ias_limit):
            # S9: from the sample over the limit too (an event correction from level flight starts where the turn
            # coordinate is singular)
            for seed in ([lo, hi] if self.eng.s9 else [lo]):
                q = self.correct_at(seed, self.eng.st[seed].sol, self.eng.st[seed].load, FIX_EVENT, 0., 7, 0, 0, 16, 0, 2e-6,
                                    &self.preds, &evals, &fresh, NULL)
                if (q >= 0 and self.eng.st[q].valid and pmin(self.eng.st[lo].alpha, self.eng.st[hi].alpha)-.5 <= self.eng.st[q].alpha
                        and self.eng.st[q].alpha <= pmax(self.eng.st[lo].alpha, self.eng.st[hi].alpha)+.5):
                    self.samples.append(q)
                    return self.eng.mark(q, KD_IAS, -1, dict(method='S8 forward-IAS face (event correction)',
                                                             ias_limit_kmh=self.eng.ias_limit*3.6))
        for it in range(24):
            if self.closed(lo, hi):break
            w = self.eng.st[hi].alpha-self.eng.st[lo].alpha
            if fabs(w) <= 1e-6:break
            has_lo = (self.eng.st[lo].mset >> kind) & 1;has_hi = (self.eng.st[hi].mset >> kind) & 1
            flo = self.eng.st[lo].m[kind];fhi = self.eng.st[hi].m[kind]
            t = flo/(flo-fhi) if has_lo and has_hi and flo > 0. and 0. > fhi and it % 3 != 2 else .5
            t = pmin(.9, pmax(.1, t))
            p = self.at_state(lo, self.eng.st[lo].alpha+t*w, 's8 soft-limit stretch')
            if p < 0 and self.eng.s9:
                # S9: a correction that does not close over a long alpha step: the same alpha from the sample over the
                # limit, then a quarter of the bracket from the one within it
                p = self.at_state(hi, self.eng.st[lo].alpha+t*w, 's9 soft-limit stretch')
                if p < 0:
                    t = .25
                    p = self.at_state(lo, self.eng.st[lo].alpha+t*w, 's9 soft-limit stretch')
            if p < 0:return -1
            if self.eng.passable(p) and not self.over(p, mask):lo = p
            elif self.eng.passable(p):hi = p
            elif self.eng.hard_rejected(p):
                e, _ = self.margin_edge(lo, p)
                return e
            else:return -1
        if not self.closed(lo, hi):return -1
        details = dict(method='S8 final soft-limit stretch', alpha_bracket_deg=[self.eng.st[lo].alpha, self.eng.st[hi].alpha],
                       limiting_load_interval_g=[self.eng.st[lo].load, self.eng.st[hi].load],
                       turn_interval_dps=[self.eng.st[lo].turn, self.eng.st[hi].turn],
                       rejected_endpoint=self.endpoint(hi, False))
        if kind == KD_RPM:
            for k in range(self.eng.st[hi].ne):
                if self.eng.st[hi].rpm[k] > self.eng.st[hi].rpm_limit[k]*(1.+1e-6):
                    engine = k;break
            details['engine_rpm_limit'] = dict(engine=engine, rpm_limit=self.eng.st[hi].rpm_limit[engine])
        return self.eng.mark(lo, kind, -1, details)

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int lower_edge(self, int level, int boundary) except -2:
        """S8: the lower edge of a column whose level flight is over a forward-IAS limit (only): the forward-IAS face
        between the last such sample of the march and the first admitted one above it (soft_edge from the admitted side).
        -1 where no admitted sample lies at or below the boundary."""
        cdef int j, u, v, face
        pts = sorted((p for p in self.samples if self.eng.passable(p) and fabs(self.eng.st[p].speed-self.speed) <= 1e-9
                      and self.eng.st[p].alpha <= self.eng.st[boundary].alpha+1e-9 and p != boundary),
                     key=lambda p: self.eng.st[p].alpha)+[boundary]
        j = 0
        while j < len(pts) and not self.eng.st[pts[j]].valid:j += 1
        if j >= len(pts):return -1
        v = pts[j];u = pts[j-1] if j >= 1 else level
        if not self.over(u, self.eng.ias_mask):return v
        face = self.soft_edge(v, u, self.eng.ias_mask, KD_IAS)
        if face < 0:return v
        return face

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef bint crossing(self, list history, int kind, double* root) except -1:
        if len(history) < 2:return False
        return self.margin_root(history, kind, root)

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int hard_face(self, list history, double target) except -2:
        """March.hard_face."""
        cdef int low = history[-1], first = history[0], k, edge, i
        cdef double root, m0k
        cdef bint has_root, small
        cdef Event ev[32]
        cdef int nev
        # S8: the states are read by index (a face correction below adds states, and the state pool can move)
        closing = []
        for k in range(NKD):
            if not (self.eng.st[low].mset >> k) & 1 or self.eng.soft_kind(k):continue
            has_root = self.crossing(history, k, &root)
            m0k = self.eng.st[first].m[k] if (self.eng.st[first].mset >> k) & 1 else 1.
            small = self.eng.st[low].m[k] <= .02*pmax(fabs(m0k), 1e-6)
            if (has_root and root < target+.5) or small:closing.append((root if has_root else self.eng.st[low].alpha, k))
        for _, k in sorted(closing):
            if k == KD_PITCH:
                if (((self.eng.st[first].mset >> k) & 1) and self.eng.st[first].m[k] > 0.
                        and self.eng.st[low].m[k] < .7*self.eng.st[first].m[k]):
                    edge = self.pitch_face(low)
                    if edge >= 0:return edge
                continue
            edge = self.active_face(low, target, k)
            if edge >= 0 and self.eng.s9 and len(history) >= 2 and self.trim_jump(history[-2], low, edge):
                # S9: a face corrected from the last sample onto another trim branch (past a fold the march failed at:
                # Hunter SB 1121-1124 km/h, the wing-force face above the pitch-response fold) is not this column's face
                self.eng.stats['s9_trim_faces'] = self.eng.stats.get('s9_trim_faces', 0)+1
                continue
            if edge >= 0:return edge
        nev = self.eng.nearest_events(low, ev)
        for i in range(min(3, nev)):
            if ev[i].kind == KD_PROPPITCH:return self.active_face(low, target, KD_PROPPITCH)
        return -1

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int active_face(self, int low, double target, int kind) except -2:
        """RescueMarch.active_face(low, target, kinds=(kind,))."""
        cdef Event ev[32]
        cdef int nev = self.eng.nearest_events(low, ev), i
        for i in range(nev):
            if ev[i].kind == kind:
                return self.event_face(low, target, &ev[i], 'active ')
        return -1

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int event_face(self, int low, double target, Event* ev, prefix) except -2:
        """RescueMarch.event_face: the active-event correction of one face from the last admitted sample."""
        cdef int p, edge
        cdef long evals = 0, fresh = 0
        cdef Trial* t
        if ev.code > 0:
            p = self.correct_at(low, self.eng.st[low].sol, self.eng.st[low].load, FIX_EVENT, 0., ev.code,
                                ev.engine if ev.code == 6 else (ev.axis-2 if ev.axis >= 0 else 0), ev.side, 16, 0, 2e-6,
                                &self.preds, &evals, &fresh, NULL)
            if p < 0 and self.robust:
                p = self.retry(low, self.eng.st[low].sol, self.eng.st[low].load, FIX_EVENT, 0., ev.code,
                               ev.engine if ev.code == 6 else (ev.axis-2 if ev.axis >= 0 else 0), ev.side, 16, 0)
        else:
            p = self.python_event(low, ev, &evals, &fresh)
        t = self.trial(prefix+KIND_TEXT[ev.kind])
        t.ok = -1 if p < 0 else int(self.eng.st[p].valid);t.evals = evals;t.fresh = fresh
        t.alpha = self.eng.st[p].alpha if p >= 0 else NAN
        if not self.accepted_face(low, target, p):return -1
        self.samples.append(p)
        details = dict(method='S3 alpha-march active face' if prefix.startswith('s3') else 'S1 alpha-march active face')
        if ev.kind == KD_RPM:
            details['engine_rpm_limit'] = dict(engine=ev.engine, rpm_limit=self.eng.st[low].rpm_limit[ev.engine])
        return self.eng.mark(p, ev.kind, ev.axis, details)

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int python_event(self, int low, Event* ev, long* evals, long* fresh) except -2:
        """An event the native corrector does not cover: the production Curve.correct on the state's row."""
        global g_sink
        from em_trim_work import Work, use, current
        from em_stationary_boundary import Curve
        row = self.eng.row_of(low)
        curve = Curve(self.eng.solver, self.speed)
        event = None
        for kind, axis, fn in curve.nearest_events(row):
            if kind != KIND_TEXT[ev.kind] or (axis if axis is not None else -1) != ev.axis:continue
            if ev.kind == KD_RPM and getattr(fn, 'stationary_engine_limit', {}).get('engine') != ev.engine:continue
            event = fn;break
        if event is None:return -1
        self.eng.stats['python_events'] += 1
        w = Work({}, parent=current())
        saved = <size_t>g_sink;g_sink = NULL
        try:
            with use(w):p = curve.correct(row, event=event, max_nfev=16)
        finally:
            g_sink = <long*>saved
        evals[0] = w.counts.get('aircraft_evaluations', 0);fresh[0] = w.counts.get('derivative_samples', 0)
        return -1 if p is None else self.eng.import_row(p)

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef bint accepted_face(self, int low, double target, int p) except -1:
        cdef TState* l = &self.eng.st[low]
        cdef TState* s
        if p < 0:return False
        s = &self.eng.st[p]
        if not self.eng.passable(p) or s.load < l.load-1e-3 or s.alpha > target+1e-9:return False
        if s.alpha >= l.alpha-1e-4:return True
        if s.alpha >= l.alpha-.5 and fabs(s.turn-l.turn) <= self.turn_tol:
            self.eng.stats['behind_face'] = self.eng.stats.get('behind_face', 0)+1
            self.s3_flags.add('face behind last sample')
            return True
        return False

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int pitch_face(self, int low) except -2:
        """March.pitch_face: em_pitch_limit.limit from the last admitted sample (native port, pitch_limit)."""
        cdef long e0 = self.eng.counts[K_EVAL], d0 = self.eng.counts[K_DERIV]
        cdef int p
        cdef Trial* t
        try:
            p = self.pitch_limit(low)
        except Exception:
            p = -1
        t = self.trial('pitch face')
        t.ok = -1 if p < 0 else int(self.eng.st[p].valid)
        t.evals = self.eng.counts[K_EVAL]-e0;t.fresh = self.eng.counts[K_DERIV]-d0
        t.alpha = self.eng.st[p].alpha if p >= 0 else NAN
        if (p < 0 or not self.eng.passable(p) or self.eng.st[p].limit < 0
                or self.eng.st[p].alpha < self.eng.st[low].alpha-1e-4 or self.eng.st[p].load < self.eng.st[low].load-1e-3):
            return -1
        self.samples.append(p)
        return p

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int pitch_limit(self, int seed) except -2:
        """em_pitch_limit.limit(solver, speed, seed) for the stationary chart solver."""
        cdef PredMap preds
        cdef double center, command, step, ps_tol, turn_tol, sign
        cdef int contraction, p, probe, good, bad, it, i
        # S8: the seed's fields are read by index (limit_at adds states, and the state pool can move)
        if not self.eng.passable(seed) or not self.eng.st[seed].has_pitch_rec:return -1
        preds.n = 0
        memo_c = [];memo_s = []
        center = self.eng.st[seed].sol[3]
        memo_c.append(center);memo_s.append(seed)
        ps_tol = self.eng.sep_tol
        from em_accuracy import turn_tolerance
        turn_tol = turn_tolerance(ps_tol)
        probes = []
        for sign in (-1., 1.):
            for contraction in range(8):
                command = pmax(-1., pmin(1., center+sign*.01*pow(.5, contraction)))
                if command == center:break
                p = self.limit_at(command, seed, memo_c, memo_s, &preds)
                if p >= 0:
                    if self.eng.st[p].pitch_margin >= self.eng.st[seed].pitch_margin:break
                    if self.eng.st[p].load >= self.eng.st[seed].load:
                        probes.append(p);break
        if not probes:return -1
        probe = probes[0]
        for q in probes[1:]:
            if self.eng.st[q].pitch_margin < self.eng.st[probe].pitch_margin:probe = q
        direction = 1. if self.eng.st[probe].sol[3] > center else -1.
        good = seed;bad = probe if not self.eng.passable(probe) else -1
        if self.eng.passable(probe):good = probe
        step = .015
        for it in range(24):
            if bad >= 0:break
            command = pmax(-1., pmin(1., self.eng.st[good].sol[3]+direction*step))
            if command == self.eng.st[good].sol[3]:return -1
            p = self.limit_at(command, good, memo_c, memo_s, &preds)
            if p < 0:
                step *= .5
                if step < 1e-5:return -1
                continue
            if not self.eng.passable(p):
                bad = p;break
            if self.eng.st[p].load < self.eng.st[good].load-2e-4:return -1
            good = p;step = pmin(.04, step*1.5)
        if bad < 0:return -1
        for it in range(24):
            if self.limit_closed(good, bad, turn_tol, ps_tol):break
            p = self.limit_at((self.eng.st[good].sol[3]+self.eng.st[bad].sol[3])*.5, good, memo_c, memo_s, &preds)
            if p < 0:return -1
            if self.eng.passable(p):good = p
            else:bad = p
        if not self.limit_closed(good, bad, turn_tol, ps_tol) or self.eng.st[good].load < self.eng.st[seed].load-2e-4:return -1
        g = &self.eng.st[good];b = &self.eng.st[bad]
        limit = dict(kind='pitch response', axis=None, limiting_load_g=g.load,
            method='First balanced pitch-response crossing in continuous command coordinates',
            constraint_residual=g.pitch_margin, rejected_constraint_residual=b.pitch_margin,
            command_bracket=[g.sol[3], b.sol[3]],
            limiting_load_interval_g=sorted([g.load, b.load]),
            turn_interval_dps=sorted([g.turn, b.turn]),
            ps_interval_mps=[g.ps, b.ps], evaluations=len(memo_c),
            rejected_endpoint=self.endpoint(bad, True))
        cid = self.eng.new_state()
        self.eng.st[cid] = self.eng.st[good]
        self.eng.st[cid].base = good;self.eng.st[cid].row = -1;self.eng.rows[cid] = None
        self.eng.st[cid].limit = len(self.eng.limits);self.eng.limits.append(limit)
        return cid

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef bint limit_closed(self, int good, int bad, double turn_tol, double ps_tol) noexcept:
        return (fabs(self.eng.st[good].turn-self.eng.st[bad].turn) <= .25*turn_tol and
                fabs(self.eng.st[good].ps-self.eng.st[bad].ps) <= .2*ps_tol)

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int limit_at(self, double command, int near, list memo_c, list memo_s, PredMap* preds) except -2:
        """em_pitch_limit original_at (stationary): elevator-prescribed correction with its admission guards."""
        cdef int p, i, ev = -1
        cdef long evals, fresh
        cdef TState* t
        for i in range(len(memo_c)):
            if memo_c[i] == command:return memo_s[i]
        p = self.correct_at(near, self.eng.st[near].sol, self.eng.st[near].load, FIX_ELEVATOR, command, 0, 0, 0, 18, 0,
                            2e-6, preds, &evals, &fresh, &ev)
        if ev >= 0:
            # Curve.retain into the solver's boundary evidence (em_pitch_limit's curve shares that list)
            entry = dict(point=None, boundary_verified=False,
                         missing_requirement='Boundary correction did not meet its balance or constraint tolerance',
                         coverage_complete=False, correction=self.eng.last_correction())
            self.eng.solver.__dict__.setdefault('stationary_boundary_evidence', []).append(entry)
            self.evidence.append((entry, ev))
        if p < 0:return -1
        t = &self.eng.st[p]
        if not t.converged or fabs(t.sol[3]-command) > 1e-5 or not t.has_pitch_rec:return -1
        for i in range(t.nreasons):
            if t.reasons[i] != R_PITCH_REV and not (self.eng.soft and soft_code(t.reasons[i])):return -1
        if fabs(t.alpha-self.eng.st[near].alpha) > .75:return -1
        memo_c.append(command);memo_s.append(p)
        return p

    # ------------------------------------------------------------------ the S3 rescue
    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef object rescue(self, int level):
        """RescueMarch.rescue: nearest faces, then bisection, from the highest admitted sample."""
        cdef int low, edge
        cdef double hi
        cdef int i
        admitted = sorted((p for p in self.samples if self.eng.passable(p) and fabs(self.eng.st[p].speed-self.speed) <= 1e-9
                           and self.eng.st[p].alpha >= self.eng.st[level].alpha-1e-9), key=lambda p: self.eng.st[p].alpha)
        if not admitted:return None
        low = admitted[-1]
        failed = []
        for i in range(self.ntrials):
            t = &self.trials[i]
            if t.ok != -1 or not isfinite(t.alpha):continue
            why = self.eng.why_text[t.why]
            if why not in ('march', 'march confirm', 'march near'):continue
            a = round(t.alpha, 4)
            if a > self.eng.st[low].alpha:failed.append(a)
        if not failed:return None
        hi = min(failed)
        edge = self.nearest_faces(low, hi, 4)
        if edge >= 0:
            self.s3_path = 's3 face';self.eng.stats['s3_face'] = self.eng.stats.get('s3_face', 0)+1
            return self.finish_face(admitted, edge)
        edge, lows = self.bisect(admitted, hi)
        if edge >= 0:
            self.s3_path = 's3 bisect';self.eng.stats['s3_bisect'] = self.eng.stats.get('s3_bisect', 0)+1
            return self.finish_face(admitted+lows, edge)
        return None

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int nearest_faces(self, int low, double target, int count) except -2:
        cdef Event ev[32]
        cdef int nev = self.eng.nearest_events(low, ev), rank, edge
        seen = set()
        for rank in range(min(count, nev)):
            key = (ev[rank].kind, ev[rank].axis)
            if key in seen:continue
            seen.add(key)
            self.eng.stats['face_trials'] = self.eng.stats.get('face_trials', 0)+1
            if self.eng.soft_kind(ev[rank].kind):continue
            if ev[rank].kind == KD_PITCH:
                if not self.eng.st[low].has_pitch_rec or rank > 1:continue
                edge = self.pitch_face(low)
            else:
                edge = self.event_face(low, target, &ev[rank], 's3 active ')
            if edge >= 0:return edge
        return -1

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef object turn_slope(self, list points):
        pts = sorted(points, key=lambda p: self.eng.st[p].alpha)
        best = None
        for a, b in zip(pts, pts[1:]):
            h = self.eng.st[b].alpha-self.eng.st[a].alpha
            if h > 1e-6:
                s = abs(self.eng.st[b].turn-self.eng.st[a].turn)/h
                best = s if best is None else max(best, s)
        return best

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef tuple bisect(self, list history, double target):
        cdef int lo = history[-1], p, it, edge
        cdef double hi = target, width
        lows = []
        for it in range(8):
            width = hi-self.eng.st[lo].alpha
            if width <= 1e-3:break
            slope = self.turn_slope(history+lows)
            if slope is not None and slope*width <= self.turn_tol:break
            self.eng.stats['bisect_trials'] = self.eng.stats.get('bisect_trials', 0)+1
            p = self.at_state(lo, self.eng.st[lo].alpha+.5*width, 's3 bisect')
            if p >= 0 and self.eng.passable(p):
                lo = p;lows.append(p)
            elif p >= 0 and self.eng.hard_rejected(p):
                self.last_lo = -1
                edge, why = self.margin_edge(lo, p)
                if edge >= 0:
                    if self.last_lo >= 0:lows.append(self.last_lo)
                    return edge, lows
                if why == 'false bracket' and self.last_lo >= 0:
                    lo = self.last_lo;lows.append(lo);continue
                break
            else:hi = self.eng.st[lo].alpha+.5*width
        if lo == history[-1]:
            return -1, lows
        return self.nearest_faces(lo, hi, 4), lows

    # ------------------------------------------------------------------ the S5 neighbour-face rescue
    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef object neighbour_face(self, list seeds, int level):
        """Where the march and its S3 rescue stay unresolved: the limit face of the nearest neighbouring
        columns' boundaries (same kind, axis and engine), corrected at this speed from that boundary state
        (the native counterpart of the production column's neighbouring-boundary continuation). The face
        must lie at or beyond the highest admitted sample of this speed, and the alpha gap between them is
        sampled so that no physical rejection lies before it. (edge, status, reason) or None."""
        cdef int lo, b, edge, i, nev, kind, axis, engine
        cdef double d
        cdef Event ev[32]
        admitted = sorted((p for p in self.samples if self.eng.passable(p) and fabs(self.eng.st[p].speed-self.speed) <= 1e-9
                           and self.eng.st[p].alpha >= self.eng.st[level].alpha-1e-9), key=lambda p: self.eng.st[p].alpha)
        if not admitted:
            if not self.eng.passable(level):return None
            admitted = [level]
        lo = admitted[-1]
        sides = {}
        for b in seeds:
            if not self.eng.st[b].valid or self.eng.st[b].limit < 0 or self.eng.st[b].speed == self.speed:continue
            side = self.eng.st[b].speed > self.speed
            d = fabs(self.eng.st[b].speed-self.speed)
            if side not in sides or d < sides[side][0]:sides[side] = (d, b)
        for _, b in sorted(sides.values()):
            limit = self.eng.limits[self.eng.st[b].limit]
            kind = KIND_CODE.get(limit['kind'], -1)
            axis = -1 if limit.get('axis') is None else int(limit['axis'])
            edge = -1
            if self.eng.soft_kind(kind):continue    # S8: an RPM face is no hard limit to rescue with
            if kind == KD_FOLD:
                edge = self.neighbour_fold(admitted, b)
            elif kind == KD_PITCH:
                edge = self.neighbour_pitch(lo, b)
            elif kind >= 0:
                engine = int((limit.get('engine_rpm_limit') or {}).get('engine', -1))
                nev = self.eng.nearest_events(b, ev)
                for i in range(nev):
                    if ev[i].kind != kind or ev[i].code <= 0:continue
                    if kind == KD_CONTROL and ev[i].axis != axis:continue
                    if kind == KD_RPM and ev[i].engine != engine:continue
                    edge = self.seeded_face(lo, b, &ev[i])
                    break
            gap = []
            if edge >= 0:edge = self.verify_gap(lo, edge, gap)
            if edge >= 0:
                self.s3_path = 's5 neighbour face'
                self.eng.stats['s5_neighbour_face'] = self.eng.stats.get('s5_neighbour_face', 0)+1
                return self.finish_face(admitted+gap, edge)
        # this speed's nearest faces from the highest admitted sample, also beyond trials that failed only
        # numerically (the S3 rescue stops at the first failed march trial)
        edge = self.nearest_faces(lo, INFINITY, 4)
        gap = []
        if edge >= 0:edge = self.verify_gap(lo, edge, gap)
        if edge >= 0:
            self.s3_path = 's5 face'
            self.s3_flags.add('face beyond numerically unresolved trials')
            self.eng.stats['s5_face'] = self.eng.stats.get('s5_face', 0)+1
            return self.finish_face(admitted+gap, edge)
        return None

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int seeded_face(self, int lo, int b, Event* ev) except -2:
        """event_face from a neighbouring boundary state: its active event corrected at this speed."""
        cdef int p
        cdef long evals = 0, fresh = 0
        cdef Trial* t
        p = self.correct_at(b, self.eng.st[b].sol, self.eng.st[b].load, FIX_EVENT, 0., ev.code,
                            ev.engine if ev.code == 6 else (ev.axis-2 if ev.axis >= 0 else 0), ev.side, 16, 0, 2e-6,
                            &self.preds, &evals, &fresh, NULL)
        if p < 0 and self.robust:
            p = self.retry(b, self.eng.st[b].sol, self.eng.st[b].load, FIX_EVENT, 0., ev.code,
                           ev.engine if ev.code == 6 else (ev.axis-2 if ev.axis >= 0 else 0), ev.side, 16, 0)
        t = self.trial('s5 neighbour '+KIND_TEXT[ev.kind])
        t.ok = -1 if p < 0 else int(self.eng.st[p].valid);t.evals = evals;t.fresh = fresh
        t.alpha = self.eng.st[p].alpha if p >= 0 else NAN
        if not self.accepted_face(lo, INFINITY, p):return -1
        self.samples.append(p)
        details = dict(method='S5 alpha-march face from a neighbouring boundary')
        if ev.kind == KD_RPM:
            details['engine_rpm_limit'] = dict(engine=ev.engine, rpm_limit=self.eng.st[lo].rpm_limit[ev.engine])
        return self.eng.mark(p, ev.kind, ev.axis, details)

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int neighbour_pitch(self, int lo, int b) except -2:
        """The pitch-response limit from this speed's state at the neighbouring boundary's alpha."""
        cdef int p, edge
        p = self.at_state(b, self.eng.st[b].alpha, 's5 neighbour pitch')
        if p < 0:return -1
        if self.eng.passable(p):
            if self.eng.st[p].alpha < self.eng.st[lo].alpha-1e-4:return -1
            return self.pitch_face(p)
        if self.eng.hard_rejected(p) and self.eng.st[p].alpha > self.eng.st[lo].alpha:
            self.last_lo = -1
            edge, _ = self.margin_edge(lo, p)
            return edge
        return -1

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int neighbour_fold(self, list admitted, int b) except -2:
        """The load maximum of this speed around the neighbouring fold's alpha (fold_edge, a larger budget)."""
        cdef int p
        cdef double base = self.eng.st[b].alpha, d
        points = [q for q in admitted]
        for d in (-.8, -.4, 0., .4):
            p = self.at_state(b, base+d, 's5 neighbour fold')
            if p >= 0 and self.eng.passable(p):points.append(p)
        edge, why, bracket = self.fold_edge(points, 28)
        if edge >= 0:return edge
        if bracket is not None:
            self.last_lo = -1
            e, _ = self.margin_edge(bracket[0], bracket[1])
            return e
        return -1

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int verify_gap(self, int lo, int edge, list gap) except -2:
        """Samples between the highest admitted sample and a face found beyond it (at least three, at most one
        degree apart): the face stands unless a physical rejection lies before it (then that rejection's own
        edge, or none) or the load falls on the way (then the load maximum, fold_edge, or none). The admitted
        gap samples join the history (finish_face)."""
        cdef double a0 = self.eng.st[lo].alpha, a1 = self.eng.st[edge].alpha
        cdef int n, j, p, prev = lo, e
        if a1-a0 <= .25:return self.face_branch(lo, edge) if self.strict_faces else edge
        n = <int>pmin(12., pmax(3., ceil(a1-a0)))
        for j in range(1, n):
            p = self.at_state(prev, a0+j*(a1-a0)/n, 's5 gap')
            if p >= 0 and self.eng.passable(p):
                if self.eng.st[p].load < self.eng.st[prev].load:
                    e, _, bracket = self.fold_edge([lo]+gap+[p], 28)
                    if e >= 0:self.s3_flags.add('load maximum before the face')
                    return e
                gap.append(p);prev = p;continue
            if p >= 0 and self.eng.hard_rejected(p):
                self.last_lo = -1
                e, _ = self.margin_edge(prev, p)
                return e
        return self.face_branch(prev, edge) if self.strict_faces else edge

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int face_branch(self, int prev, int edge) except -2:
        """S6: a face from the S5 face rescues (a neighbouring boundary's face corrected at this speed, or this
        speed's faces beyond numerically failed trials) can lie on another equilibrium branch than the march's.
        The state at the face's alpha corrected from the last admitted sample of the march branch decides: the face
        itself (turn rate within the march tolerance) stands; a physical rejection there means this branch's first
        limit lies before it (margin_edge, as the march localizes); a different admitted state rejects the face.
        A correction that does not close leaves the S5 result."""
        cdef double a1 = self.eng.st[edge].alpha
        cdef int q, e
        if a1-self.eng.st[prev].alpha <= 1e-6:return edge
        q = self.at_state(prev, a1, 's6 face branch')
        self.eng.stats['s6_face_checks'] = self.eng.stats.get('s6_face_checks', 0)+1
        if q < 0:return edge
        if self.eng.passable(q):
            if fabs(self.eng.st[q].turn-self.eng.st[edge].turn) <= self.turn_tol:return edge
            self.s3_flags.add('face on another branch rejected')
            self.eng.stats['s6_face_rejected'] = self.eng.stats.get('s6_face_rejected', 0)+1
            return -1
        if self.eng.hard_rejected(q):
            self.last_lo = -1
            e, _ = self.margin_edge(prev, q)
            self.eng.stats['s6_face_replaced'] = self.eng.stats.get('s6_face_replaced', 0)+1
            if e >= 0:self.s3_flags.add('face replaced by the march branch limit')
            return e
        return edge

    # ------------------------------------------------------------------ the column record
    @cython.boundscheck(True)
    @cython.wraparound(True)
    def trial_log(self):
        out = []
        for i in range(self.ntrials):
            t = &self.trials[i]
            out.append([self.eng.why_text[t.why], round(t.alpha, 4) if isfinite(t.alpha) else None,
                        None if t.ok < 0 else bool(t.ok), t.evals, t.fresh if t.fresh >= 0 else None,
                        None if t.warm < 0 else bool(t.warm)])
        return out

    @cython.boundscheck(True)
    @cython.wraparound(True)
    def trial_count(self):
        return self.ntrials

    @cython.boundscheck(True)
    @cython.wraparound(True)
    def sample_list(self):
        return list(self.samples)


cdef object polyfit2(xs, ys):
    """np.polyfit(xs, ys, 2) leading coefficients (0. on numpy failure, as em_march)."""
    import numpy as np
    try:
        c2, c1, _ = np.polyfit(xs, ys, 2)
    except (np.linalg.LinAlgError, ValueError):
        return 0., 0.
    return float(c2), float(c1)


cdef list polyfit_roots(xs, ys, double a):
    import numpy as np
    try:
        c = np.polyfit(xs, ys, 2)
        return [r.real for r in np.roots(c) if abs(r.imag) < 1e-12 and r.real > a]
    except (np.linalg.LinAlgError, ValueError):
        return []
