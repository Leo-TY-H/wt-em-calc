# S5: one alpha-march column on native trial states (em_march.March + RescueMarch,
# march_worker, column_record, em_pitch_limit.limit), line by line.

DEF STATUS_VERIFIED = 0     # 'verified limit'
DEF STATUS_CEILING = 1      # 'plot ceiling'
DEF STATUS_UNRESOLVED = 2   # 'unresolved numerical boundary'
DEF STATUS_NOFEASIBLE = 3   # 'no feasible samples'
DEF STATUS_OTHER = 4        # any other status (excluded / discontinuity / fallback columns): text in NCol.status_text

STATUS_TEXT = ('verified limit', 'plot ceiling', 'unresolved numerical boundary', 'no feasible samples', None)
RESCUE_REASONS = ('march correction unresolved', 'march roots disagree')


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

    @cython.boundscheck(True)
    @cython.wraparound(True)
    def __init__(self, double speed):
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
        self.eng = eng;self.speed = speed;self.turn_tol = turn_tol;self.max_step = MAX_STEP_DEG
        self.max_nfev = MARCH_NFEV
        self.alpha_upper = float(eng.solver.trim_bounds[1][0])
        self.preds.n = 0
        self.samples = []
        self.cap_trials = 64;self.trials = <Trial*>malloc(self.cap_trials*sizeof(Trial));self.ntrials = 0
        if self.trials == NULL:raise MemoryError()
        self.last_lo = -1;self.s3_path = None;self.s3_flags = set()
        self.store = True
        self.evidence = []

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

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int at(self, int seed, const double* sol, double load, double seed_alpha, double alpha, why,
                bint has_base, double base_alpha) except -2:
        """March.at(seed, alpha, why, base_alpha=...). ``seed``/``sol``/``load``/``seed_alpha``: the seed dict
        (a state with its solution and load replaced; seed_alpha its 'alpha_deg')."""
        cdef int p, i, warm_state = -1
        cdef double wsol[5]
        cdef double wload
        cdef long evals, fresh
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
        else:
            p = self.correct_at(seed, sol, load, FIX_ALPHA, alpha, 0, 0, 0, self.max_nfev, MARCH_EVALS, 2e-6,
                                &self.preds, &evals, &fresh, NULL)
        if self.store:self.eng.store_keep(p, self.speed, &self.preds)
        t = self.trial(why)
        t.alpha = alpha;t.ok = -1 if p < 0 else int(self.eng.st[p].valid);t.evals = evals;t.fresh = fresh
        t.warm = 1 if (best != NULL and best.has_pred) else 0
        if p >= 0 and self.eng.st[p].valid:self.samples.append(p)
        return p

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int at_state(self, int seed, double alpha, why) except -2:
        """at(state, alpha, why) with the state itself as the seed."""
        cdef TState* t = &self.eng.st[seed]
        return self.at(seed, t.sol, t.load, t.alpha, alpha, why, False, 0.)

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
        cdef int i, p, k
        cdef long evals, fresh
        cdef Trial* t
        cdef double d, bd = 0.
        cdef PKey lk
        if self.eng.has_level_key:
            for i in range(self.eng.nlevel):
                r = &self.eng.level_store[i]
                if r.speed == self.speed:continue
                d = fabs(r.speed-self.speed)
                if best == NULL or d < bd:
                    best = r;bd = d
            if best != NULL and fabs(best.speed-self.speed) <= REUSE_WINDOW:
                neighbour = best.state
                pm_set(&self.preds, &self.eng.level_key, &best.pred)
        p = self.correct_at(neighbour, self.eng.st[neighbour].sol, self.eng.st[neighbour].load, FIX_LOAD, 1., 0, 0, 0,
                            self.max_nfev, MARCH_EVALS, 2e-6, &self.preds, &evals, &fresh, NULL)
        t = self.trial('level correction')
        t.ok = -1 if p < 0 else int(self.eng.st[p].valid);t.evals = evals;t.fresh = fresh
        if p >= 0 and self.eng.st[p].valid:
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
        return p if p >= 0 and self.eng.st[p].valid else -1

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
    cdef double next_alpha(self, list history, double step, bint has_predicted, double predicted) except? -1e300:
        """RescueMarch.next_alpha."""
        cdef int low = history[-1], k
        cdef double a = self.eng.st[low].alpha, target, root, c[3], vertex, c2, c1
        cdef TState* t = &self.eng.st[low]
        target = a+step
        if has_predicted and predicted > a+.05:target = pmin(target, predicted+pmax(OVERSHOOT_DEG, .02*(predicted-a)))
        if len(history) >= 2:
            for k in range(NKD):
                if not (t.mset >> k) & 1:continue
                if not self.margin_root(history, k, &root):continue
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
        return fabs(turn_rate(self.speed, self.eng.st[hi].load)-turn_rate(self.speed, self.eng.st[lo].load)) <= self.turn_tol

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef tuple margin_edge(self, int lo, int hi):
        """March.margin_edge: (edge state or -1, reason or None)."""
        cdef double flo = 0., fhi = 0., w, t, slope, d
        cdef bint has_flo, has_fhi
        cdef int side = 0, it, p, kind, axis
        cdef bint done = False
        has_flo = self.eng.combined(lo, &flo);has_fhi = self.eng.combined(hi, &fhi)
        for it in range(16):
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
            if p < 0 or (not self.eng.st[p].valid and not self.eng.physical(p)):
                p = self.at_state(lo, self.eng.st[lo].alpha+.5*w, 'localize bisect')
                if p < 0 or (not self.eng.st[p].valid and not self.eng.physical(p)):
                    self.last_lo = lo
                    return -1, 'localization correction unresolved'
            if self.eng.st[p].valid:
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
    cdef tuple fold_edge(self, list points):
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
        for it in range(14):
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
            if not self.eng.st[p].valid:
                if self.eng.physical(p) and vv < xs[1]:return -1, 'rejection before load maximum', (a, p)
                return -1, ('fold bracket inconsistent' if self.eng.physical(p) else 'fold correction unresolved'), None
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
        if out[0] >= 0 or out[2] not in RESCUE_REASONS:return out
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
    cdef tuple run_march(self, int level, bint has_predicted, double predicted):
        """March.run(level, predicted)."""
        cdef double span, step, cap, target
        cdef int low, p, q, tried_active = -1, lo, edge, seed_state
        cdef double sol[5]
        cdef double load
        cdef bint extrapolated_seed
        cdef int it, i
        history = [level]
        for h in history:
            if not self.has_sample(h):self.samples.append(h)
        if has_predicted:
            span = predicted-self.eng.st[level].alpha
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
            p = self.at(seed_state, sol, load, self.eng.st[seed_state].alpha, target, 'march', True, self.eng.st[low].alpha)
            if p >= 0 and not self.eng.st[p].valid and self.eng.physical(p) and (extrapolated_seed or self.trials[self.ntrials-1].warm == 1):
                q = self.at_state(low, target, 'march confirm')
                if q >= 0 and self.eng.st[q].valid:
                    step = .5*(target-self.eng.st[low].alpha);has_predicted = False
                    if step < .05:return -1, 'unresolved numerical boundary', 'march roots disagree'
                    continue
                if q >= 0 and self.eng.physical(q):p = q
            if p < 0 or (not self.eng.st[p].valid and not self.eng.physical(p)):
                if low != tried_active:
                    tried_active = low
                    edge = self.hard_face(history, target)
                    if edge >= 0:return self.finish_face(history, edge)
                if self.eng.st[low].load >= cap-1e-6 or (p >= 0 and self.has_reason(p, R_LOAD_CAP)):
                    q = self.correct_plain(low, FIX_LOAD, cap)
                    if q >= 0 and self.eng.st[q].valid and self.eng.st[q].alpha >= self.eng.st[low].alpha-1e-6:
                        self.samples.append(q)
                        return q, 'plot ceiling', None
                step = .5*(target-self.eng.st[low].alpha)
                if step < .05:return -1, 'unresolved numerical boundary', 'march correction unresolved'
                has_predicted = False
                continue
            if not self.eng.st[p].valid:
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
                    if q < 0 or not self.eng.st[q].valid or self.eng.st[q].load <= self.eng.st[low].load:
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
    cdef bint crossing(self, list history, int kind, double* root) except -1:
        if len(history) < 2:return False
        return self.margin_root(history, kind, root)

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int hard_face(self, list history, double target) except -2:
        """March.hard_face."""
        cdef int low = history[-1], first = history[0], k, edge, i
        cdef TState* t = &self.eng.st[low]
        cdef TState* t0 = &self.eng.st[first]
        cdef double root, m0k
        cdef bint has_root, small
        cdef Event ev[32]
        cdef int nev
        closing = []
        for k in range(NKD):
            if not (t.mset >> k) & 1:continue
            has_root = self.crossing(history, k, &root)
            m0k = t0.m[k] if (t0.mset >> k) & 1 else 1.
            small = t.m[k] <= .02*pmax(fabs(m0k), 1e-6)
            if (has_root and root < target+.5) or small:closing.append((root if has_root else t.alpha, k))
        for _, k in sorted(closing):
            if k == KD_PITCH:
                if ((t0.mset >> k) & 1) and t0.m[k] > 0. and t.m[k] < .7*t0.m[k]:
                    edge = self.pitch_face(low)
                    if edge >= 0:return edge
                continue
            edge = self.active_face(low, target, k)
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
                                ev.axis-2 if ev.axis >= 0 else 0, ev.side, 16, 0, 2e-6, &self.preds, &evals, &fresh, NULL)
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
        if not s.valid or s.load < l.load-1e-3 or s.alpha > target+1e-9:return False
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
        if (p < 0 or not self.eng.st[p].valid or self.eng.st[p].limit < 0
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
        cdef TState* s = &self.eng.st[seed]
        if not s.valid or not s.has_pitch_rec:return -1
        preds.n = 0
        memo_c = [];memo_s = []
        center = s.sol[3]
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
                    if self.eng.st[p].pitch_margin >= s.pitch_margin:break
                    if self.eng.st[p].load >= s.load:
                        probes.append(p);break
        if not probes:return -1
        probe = probes[0]
        for q in probes[1:]:
            if self.eng.st[q].pitch_margin < self.eng.st[probe].pitch_margin:probe = q
        direction = 1. if self.eng.st[probe].sol[3] > center else -1.
        good = seed;bad = probe if not self.eng.st[probe].valid else -1
        if self.eng.st[probe].valid:good = probe
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
            if not self.eng.st[p].valid:
                bad = p;break
            if self.eng.st[p].load < self.eng.st[good].load-2e-4:return -1
            good = p;step = pmin(.04, step*1.5)
        if bad < 0:return -1
        for it in range(24):
            if self.limit_closed(good, bad, turn_tol, ps_tol):break
            p = self.limit_at((self.eng.st[good].sol[3]+self.eng.st[bad].sol[3])*.5, good, memo_c, memo_s, &preds)
            if p < 0:return -1
            if self.eng.st[p].valid:good = p
            else:bad = p
        if not self.limit_closed(good, bad, turn_tol, ps_tol) or self.eng.st[good].load < s.load-2e-4:return -1
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
            if t.reasons[i] != R_PITCH_REV:return -1
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
        admitted = sorted((p for p in self.samples if self.eng.st[p].valid and fabs(self.eng.st[p].speed-self.speed) <= 1e-9
                           and self.eng.st[p].alpha >= self.eng.st[level].alpha-1e-9), key=lambda p: self.eng.st[p].alpha)
        if not admitted:return None
        low = admitted[-1]
        failed = []
        for i in range(self.ntrials):
            t = &self.trials[i]
            if t.ok != -1 or not isfinite(t.alpha):continue
            why = self.eng.why_text[t.why]
            if why not in ('march', 'march confirm'):continue
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
            if p >= 0 and self.eng.st[p].valid:
                lo = p;lows.append(p)
            elif p >= 0 and self.eng.physical(p):
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
