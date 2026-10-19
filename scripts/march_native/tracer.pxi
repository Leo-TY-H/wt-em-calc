# S5: the march column worker (em_march.march_worker / column_record) and the speed
# tracer (em_march.MarchTracer) on native columns. Column records (Python dicts with
# chart rows) are built once, after the trace (NTracer.records).
#
# S6 (native only; the Python march is unchanged): a column every rescue leaves unresolved
# gets the robust march (NMarch.robust_march), then the nearest verified speed within the
# chart's speed tolerance (column); interior numerical gaps get columns on both sides
# (resolve_gaps), and an unresolved column within twice the speed tolerance of verified
# columns, or past an explicit edge, is dropped with a trace event (within_tolerance).
# WT_MARCH_S6=0 restores the S5 tracer exactly.


cdef int pchip_build(int n, const double* x, const double* y, double* c) except -1:
    """scipy.interpolate.PchipInterpolator(x, y) coefficients (PPoly c[4][n-1], row-major), same arithmetic."""
    cdef double* h = <double*>malloc(n*sizeof(double))
    cdef double* mk = <double*>malloc(n*sizeof(double))
    cdef double* dk = <double*>malloc(n*sizeof(double))
    cdef int k
    cdef double w1, w2, whmean, t, slope
    if h == NULL or mk == NULL or dk == NULL:
        if h != NULL:cfree(h)
        if mk != NULL:cfree(mk)
        if dk != NULL:cfree(dk)
        raise MemoryError()
    try:
        for k in range(n-1):
            h[k] = x[k+1]-x[k];mk[k] = (y[k+1]-y[k])/h[k]
        if n == 2:
            dk[0] = mk[0];dk[1] = mk[0]
        else:
            for k in range(1, n-1):
                if (np_sign(mk[k]) != np_sign(mk[k-1])) or mk[k] == 0. or mk[k-1] == 0.:
                    dk[k] = 0.
                else:
                    w1 = 2*h[k]+h[k-1];w2 = h[k]+2*h[k-1]
                    whmean = (w1/mk[k-1]+w2/mk[k])/(w1+w2)
                    dk[k] = 1.0/whmean
            dk[0] = pchip_edge(h[0], h[1], mk[0], mk[1])
            dk[n-1] = pchip_edge(h[n-2], h[n-3], mk[n-2], mk[n-3])
        for k in range(n-1):
            slope = (y[k+1]-y[k])/h[k]
            t = (dk[k]+dk[k+1]-2*slope)/h[k]
            c[k] = t/h[k]
            c[(n-1)+k] = (slope-dk[k])/h[k]-t
            c[2*(n-1)+k] = dk[k]
            c[3*(n-1)+k] = y[k]
    finally:
        cfree(h);cfree(mk);cfree(dk)
    return 0


cdef inline double np_sign(double v) noexcept nogil:
    if v > 0.:return 1.
    if v < 0.:return -1.
    if v == 0.:return 0.
    return v


cdef double pchip_edge(double h0, double h1, double m0, double m1) noexcept nogil:
    # PchipInterpolator._edge_case
    cdef double d = ((2*h0+h1)*m0-h0*m1)/(h0+h1)
    cdef bint mask = np_sign(d) != np_sign(m0)
    cdef bint mask2 = (np_sign(m0) != np_sign(m1)) and (fabs(d) > 3.*fabs(m0))
    if mask:return 0.
    if mask2:return 3.*m0
    return d


cdef double pchip_eval(int n, const double* x, const double* c, double xv) noexcept nogil:
    """PPoly.__call__ (extrapolating, ascending x): interval search and evaluate_poly1."""
    cdef int i = 0, lo, hi, mid
    cdef double s, res, z
    if not (x[0] <= xv and xv <= x[n-1]):
        if xv < x[0]:i = 0
        elif xv > x[n-1]:i = n-2
        else:return NAN
    elif xv == x[n-1]:
        i = n-2
    else:
        lo = 0;hi = n-2
        while lo < hi:
            mid = (lo+hi)//2
            if xv < x[mid]:hi = mid
            elif xv >= x[mid+1]:lo = mid+1
            else:
                lo = mid;break
        i = lo
    s = xv-x[i]
    res = 0.;z = 1.
    res = res+c[3*(n-1)+i]*z;z = z*s
    res = res+c[2*(n-1)+i]*z;z = z*s
    res = res+c[(n-1)+i]*z;z = z*s
    res = res+c[i]*z
    return res


cdef class NTracer:
    """em_march.MarchTracer on native columns."""
    cdef MarchEngine eng
    cdef public object aircraft, config, cj, on_column, segments_, limit, seams
    cdef public double ttol, vtol, place_tol, max_step, march_tol
    cdef public int max_depth
    cdef public dict columns, work
    cdef public list witnesses, events
    cdef object N, M, S
    cdef bint production_fallback, nudge, s6, probing
    cdef set check_tried, check_passed

    @cython.boundscheck(True)
    @cython.wraparound(True)
    def __init__(self, MarchEngine eng, aircraft, config, cj, double ttol, double vtol, double place_tol, double max_step,
                 int max_depth, double march_tol, on_column):
        import em_march_native as N
        import em_march as M
        import em_sampling as S
        self.eng = eng;self.aircraft = aircraft;self.config = config;self.cj = cj
        self.ttol = ttol;self.vtol = vtol;self.place_tol = place_tol;self.max_step = max_step;self.max_depth = max_depth
        self.march_tol = march_tol;self.on_column = on_column
        self.columns = {};self.witnesses = [];self.events = [];self.work = {}
        self.check_tried = set();self.check_passed = set()
        self.N = N;self.M = M;self.S = S
        import os
        self.production_fallback = os.environ.get('WT_MARCH_PRODUCTION_FALLBACK') == '1'
        # S6: WT_MARCH_S6=0 restores the S5 tracer and column rescues exactly (rollback / comparison);
        # WT_MARCH_NUDGE=0 keeps unresolved columns at their requested speed (diagnostics)
        self.s6 = os.environ.get('WT_MARCH_S6', '1') != '0' and not self.production_fallback
        self.nudge = self.s6 and os.environ.get('WT_MARCH_NUDGE', '1') != '0'
        self.probing = False   # nudge and gap probes run without the S6 robust march (they look for speeds outside a band)

    # ------------------------------------------------------------------ one column
    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef NColumn column(self, double speed, list neighbours, why, bint keep=True, double lo=-INFINITY,
                        double hi=INFINITY):
        """MarchTracer.column -> run_column -> sample_with_history -> bounded_column -> march_worker.

        S6: a column whose boundary stays unresolved is replaced by the nearest column within the chart's speed
        tolerance (inside the open interval (lo, hi) the caller allows) whose boundary is verified; the tracer
        already resolves corners, edges and check intervals only to that tolerance. The column keeps its own
        speed and records the requested one (nudged_from); a 'nudged' trace event names the reason."""
        global g_sink
        from em_trim_work import Work, use, current
        cdef NColumn col, c, alt
        cdef int sid
        cdef double s, q
        if self.on_column is not None:self.on_column(self, 'start', speed, why)
        seeds = []
        for c in neighbours:
            if c is None:continue
            for sid in c.points:
                if self.eng.st[sid].valid:seeds.append(sid)
        w = Work({}, parent=current())
        with use(w):
            col = self.run_column(speed, seeds)
            if col.status == STATUS_UNRESOLVED and col.march and self.nudge:
                q = .25*self.vtol
                for k in (1., -1., 2., -2., 4., -4.):
                    s = speed+k*q
                    if not (lo < s < hi):continue
                    self.probing = True
                    try:alt = self.run_column(s, seeds)
                    finally:self.probing = False
                    if NTracer.ok(alt):
                        alt.nudged_from = speed
                        self.events.append(dict(kind='nudged', speed_kmh=speed, to_kmh=s, role=why,
                                                reason=str(col.march_reason)[:80]))
                        self.eng.stats['s6_nudged'] = self.eng.stats.get('s6_nudged', 0)+1
                        col = alt;break
        col.role = why;col.evaluations = w.counts.get('aircraft_evaluations', 0)
        row = self.work.setdefault(why, dict(columns=0, evaluations=0))
        row['columns'] += 1;row['evaluations'] += col.evaluations
        if keep:self.columns[float(col.speed)] = col
        if self.on_column is not None:self.on_column(self, 'done', speed, why)
        return col

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef NColumn run_column(self, double speed, list seeds):
        global g_sink
        from em_trim_work import Work, use, current
        import time
        cdef NColumn col = NColumn(speed)
        solver = self.eng.solver
        excluded = self.N.column_preamble(solver, speed)
        if excluded is not None:
            self.adopt(col, excluded)
            return col
        disc = self.N.discontinuous(solver, speed)
        if disc is not None:
            self.adopt(col, disc)
            return col
        work = Work({}, parent=current(), collect=False)
        started = time.monotonic()
        saved = <size_t>g_sink
        with use(work):
            g_sink = self.eng.counts
            try:
                self.march_worker(col, speed, seeds, started)
            finally:
                g_sink = <long*>saved
                self.eng.flush_counts()
        if col.record is not None:
            # production fallback column: bounded_column / sample_with_history post-processing
            col.record['search_work'] = dict(work.counts)
            self.N.post_column(solver, col.record)
            self.adopt(col, col.record)
        else:
            col.search_work = dict(work.counts)
        return col

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef void adopt(self, NColumn col, dict record) except *:
        """A column record built by the production code: its states are imported."""
        cdef int sid
        col.record = record;col.march = False
        col.status_text = record.get('boundary_status');col.reason = record.get('boundary_reason')
        col.status = STATUS_OTHER
        for k, text in enumerate(STATUS_TEXT[:4]):
            if col.status_text == text:col.status = k
        ids = {}
        col.points = []
        for p in record.get('points', ()):
            if id(p) not in ids:ids[id(p)] = self.eng.import_row(p)
            col.points.append(ids[id(p)])
        b = record.get('boundary')
        col.boundary = -1
        if b is not None:
            if id(b) not in ids:ids[id(b)] = self.eng.import_row(b)
            col.boundary = ids[id(b)]
            if b.get('envelope_limit') is not None:
                self.eng.st[col.boundary].limit = len(self.eng.limits);self.eng.limits.append(b['envelope_limit'])
        lb = record.get('lower_boundary')
        col.level = -1
        if lb is not None:
            if id(lb) not in ids:ids[id(lb)] = self.eng.import_row(lb)
            col.level = ids[id(lb)]

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int march_worker(self, NColumn col, double speed, list seeds, double started) except -1:
        """em_march.march_worker on native states."""
        global g_sink
        from em_trim_work import Work, use, current
        import time
        cdef NMarch march
        cdef int level = -1, nearest = -1, sid, boundary
        cdef double d, bd = 0., predicted
        cdef bint has_predicted, healthy
        self.eng.stats['columns'] += 1
        seeds = [sid for sid in seeds if self.eng.st[sid].valid and self.eng.st[sid].has_coords]
        for sid in seeds:
            if fabs(self.eng.st[sid].load-1.) <= 1e-12:
                d = fabs(self.eng.st[sid].speed-speed)
                if nearest < 0 or d < bd:
                    nearest = sid;bd = d
        march = NMarch(self.eng, speed, self.march_tol)
        march.strict_faces = self.s6
        col.march = True
        if nearest >= 0 and march.store:
            level = march.level_from(nearest)
        if level < 0:
            level = self.python_level(march, speed, nearest)
        healthy = nearest >= 0 and speed > self.eng.st[nearest].speed
        if not self.eng.st[level].valid:
            if not self.eng.physical(level) and healthy:
                reasons = ', '.join(REASON_TEXT[self.eng.st[level].reasons[i]] for i in range(self.eng.st[level].nreasons))
                # S5: march from an admitted state of this speed on the neighbouring columns' alpha path
                # (their march from level flight reached it without a limit) instead of the production column
                if self.interior_march(col, march, speed, seeds, 'level flight unresolved: '+reasons, started):return 0
                if self.production_fallback:
                    self.production(col, speed, seeds, 'level flight unresolved: '+reasons)
                    return 0
                # S5: no production column: the explicit edge of an unresolved level flight (as below)
                self.eng.stats['withheld_production'] = self.eng.stats.get('withheld_production', 0)+1
            reasons = ', '.join(REASON_TEXT[self.eng.st[level].reasons[i]] for i in range(self.eng.st[level].nreasons))
            self.column_record(col, march, -1, -1, STATUS_NOFEASIBLE, 'level flight: '+reasons, started)
            col.points = [level]
            col.region = False
            col.evidence = march.evidence;col.evidence_list = list(getattr(self.eng.solver, 'stationary_boundary_evidence', []))
            return 0
        has_predicted = self.predicted_alpha(speed, seeds, &predicted)
        boundary, status, reason = march.run(level, has_predicted, predicted)
        if boundary < 0:
            out = march.neighbour_face(seeds, level)
            if out is not None and out[0] >= 0:
                march.s3_flags.add('march unresolved: '+str(reason)[:60])
                boundary, status, reason = out
        if boundary < 0 and self.s6 and not self.probing:
            # S6: the march again with robust corrections (regime seeds, larger budget)
            out = march.robust_march(level, has_predicted, predicted)
            if out is not None:
                march.s3_flags.add('march unresolved: '+str(reason)[:60])
                boundary, status, reason = out
        if boundary < 0:
            if not self.production_fallback:
                # S5: the unresolved march column itself (a numerical gap in the chart, never an absence proof);
                # WT_MARCH_PRODUCTION_FALLBACK=1 restores the Python production column search (rollback)
                self.column_record(col, march, level, -1, STATUS_UNRESOLVED, reason, started)
                col.region = True
                col.evidence = march.evidence;col.evidence_list = list(getattr(self.eng.solver, 'stationary_boundary_evidence', []))
                self.eng.stats['withheld_production'] = self.eng.stats.get('withheld_production', 0)+1
                return 0
            self.production(col, speed, seeds, 'march unresolved: '+str(reason))
            return 0
        self.column_record(col, march, level, boundary,
                           STATUS_VERIFIED if status == 'verified limit' else STATUS_CEILING if status == 'plot ceiling' else STATUS_UNRESOLVED,
                           reason, started)
        col.region = True
        col.evidence = march.evidence;col.evidence_list = list(getattr(self.eng.solver, 'stationary_boundary_evidence', []))
        return 0

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef bint interior_march(self, NColumn col, NMarch march, double speed, list seeds, why, double started) except -1:
        """Where level flight at this speed is numerically unresolved above an admitted neighbouring speed: the
        march from the lowest-alpha admitted state of this speed that the nearest neighbouring columns' marches
        passed (their samples above level flight, corrected at the same alpha here). The boundary is the first
        limit along alpha from that state; the neighbours' paths from level flight to it had none. False when
        no such state closes or the march from it is unresolved (then the production column, as before)."""
        cdef int sid, start = -1, p, boundary
        cdef double d, best = INFINITY, predicted
        cdef bint has_predicted
        above = [sid for sid in seeds if self.eng.st[sid].valid and self.eng.st[sid].has_coords
                 and self.eng.st[sid].load > 1.+1e-9 and self.eng.st[sid].speed != speed]
        if not above:return False
        for sid in above:
            d = fabs(self.eng.st[sid].speed-speed)
            if d < best:best = d
        # the nearest neighbouring speed on each side
        sides = {}
        for sid in above:
            side = self.eng.st[sid].speed > speed
            d = fabs(self.eng.st[sid].speed-speed)
            if side not in sides or d < sides[side]:sides[side] = d
        near = [sid for sid in above if fabs(self.eng.st[sid].speed-speed) == sides[self.eng.st[sid].speed > speed]]
        near.sort(key=lambda q: (self.eng.st[q].alpha, fabs(self.eng.st[q].speed-speed)))
        for sid in near[:4]:
            p = march.at_state(sid, self.eng.st[sid].alpha, 'interior start')
            if p >= 0 and self.eng.st[p].valid:
                start = p;break
        if start < 0:return False
        has_predicted = self.predicted_alpha(speed, seeds, &predicted)
        boundary, status, reason = march.run(start, has_predicted, predicted)
        if boundary < 0 and self.s6 and not self.probing:
            out = march.robust_march(start, has_predicted, predicted)
            if out is not None:
                boundary, status, reason = out
                march.s3_flags.add('s6 robust march')
        if boundary < 0:return False
        march.s3_path = 'interior start'
        march.s3_flags.add(why[:80])
        self.column_record(col, march, start, boundary,
                           STATUS_VERIFIED if status == 'verified limit' else STATUS_CEILING if status == 'plot ceiling' else STATUS_UNRESOLVED,
                           reason, started)
        col.region = True
        col.evidence = march.evidence;col.evidence_list = list(getattr(self.eng.solver, 'stationary_boundary_evidence', []))
        self.eng.stats['interior_starts'] = self.eng.stats.get('interior_starts', 0)+1
        return True

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int python_level(self, NMarch march, double speed, int nearest) except -1:
        """The production level-flight solve (em_sampling.solve_level), with the S1 unseeded retry."""
        global g_sink
        from em_trim_work import Work, use, current
        cdef int level, retry
        cdef Trial* t
        cdef LevelSolver ls = LevelSolver(self.eng.points_())
        cdef long e0
        solver = self.eng.solver
        seed = None
        if nearest >= 0:seed = self.eng.initial_dict(nearest, None)
        if solver.is_prop and hasattr(solver.engine, 'reset_search'):solver.engine.reset_search()
        # trial work: native charges plus the Python ones (propeller branch recovery) of the solve
        e0 = self.eng.counts[K_EVAL]
        w = Work({}, parent=current())
        with use(w):level = ls.solve(speed, seed)
        t = march.trial('level');t.ok = int(self.eng.st[level].valid)
        t.evals = self.eng.counts[K_EVAL]-e0+w.counts.get('aircraft_evaluations', 0)
        t.fresh = -1
        if (not self.eng.st[level].valid and not self.eng.physical(level) and nearest >= 0
                and speed > self.eng.st[nearest].speed):
            e0 = self.eng.counts[K_EVAL]
            w = Work({}, parent=current())
            with use(w):retry = ls.solve(speed, None)
            t = march.trial('level retry');t.ok = int(self.eng.st[retry].valid)
            t.evals = self.eng.counts[K_EVAL]-e0+w.counts.get('aircraft_evaluations', 0);t.fresh = -1
            if self.eng.st[retry].valid or self.eng.physical(retry):level = retry
        return level

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int production(self, NColumn col, double speed, list seeds, why) except -1:
        """em_march.production_column: the production boundary column search at this speed."""
        global g_sink
        rows = [self.eng.row_of(sid) for sid in seeds]
        cdef double t0 = s5_now()
        saved = <size_t>g_sink;g_sink = NULL
        try:
            self.eng.flush_counts()
            record = self.M.production_column((self.aircraft, self.cj, float(speed), rows), why)
        finally:
            g_sink = <long*>saved
            self.eng.t_python += s5_now()-t0
        self.eng.stats['python_calls'] += 1
        col.record = record
        return 0

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef bint predicted_alpha(self, double speed, list seeds, double* out) except -1:
        """em_march.predicted_alpha(speed, seeds)."""
        cdef int a, b, sid
        cdef double t, value, lo, hi
        anchors = {}
        for sid in seeds:
            if self.eng.st[sid].valid and self.eng.st[sid].limit >= 0 and self.eng.st[sid].speed != speed:
                anchors.setdefault(self.eng.st[sid].speed, sid)
        near = sorted(anchors.values(), key=lambda p: abs(self.eng.st[p].speed-speed))[:2]
        if not near:return False
        if len(near) == 1:
            out[0] = self.eng.st[near[0]].alpha;return True
        a, b = near
        t = (speed-self.eng.st[a].speed)/(self.eng.st[b].speed-self.eng.st[a].speed)
        value = self.eng.st[a].alpha+t*(self.eng.st[b].alpha-self.eng.st[a].alpha)
        lo = pmin(self.eng.st[a].alpha, self.eng.st[b].alpha);hi = pmax(self.eng.st[a].alpha, self.eng.st[b].alpha)
        if self.eng.st[b].alpha < self.eng.st[a].alpha:
            lo, hi = self.eng.st[b].alpha, self.eng.st[a].alpha
        else:
            lo, hi = self.eng.st[a].alpha, self.eng.st[b].alpha
        out[0] = pmin_np(pmax_np(value, lo-3.), hi+3.)
        return True

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int column_record(self, NColumn col, NMarch march, int level, int boundary, int status, reason,
                           double started) except -1:
        """em_march.column_record (the dict itself is built after the trace, NTracer.record)."""
        import time
        cdef int sid
        cdef double ba = 0., bl = 0.
        keep = [sid for sid in march.samples if self.eng.st[sid].valid and fabs(self.eng.st[sid].speed-col.speed) <= 1e-9]
        if boundary >= 0:
            ba = self.eng.st[boundary].alpha;bl = self.eng.st[boundary].load
            keep = [sid for sid in keep if self.eng.st[sid].alpha <= ba+1e-9 and sid != boundary
                    and fabs(self.eng.st[sid].load-bl) > 1e-12]+[boundary]
        seen = set();points = []
        for sid in keep:
            if sid not in seen:
                seen.add(sid);points.append(sid)
        points.sort(key=lambda p: self.eng.st[p].load)
        col.points = points;col.sample_points = list(points);col.boundary = boundary;col.level = level
        col.status = status;col.march_reason = reason
        col.raw_status = STATUS_TEXT[status]
        if status == STATUS_VERIFIED:
            col.raw_reason = self.eng.limits[self.eng.st[boundary].limit]['kind'] if boundary >= 0 else None
        elif boundary < 0:
            col.raw_reason = reason
        else:
            col.raw_reason = None
        col.reason = col.raw_reason
        col.trial_log = march.trial_log();col.ntrials = march.trial_count()
        col.s3_path = march.s3_path or ('march' if boundary >= 0 else 'none')
        col.s3_flags = sorted(march.s3_flags) if march.s3_flags else None
        col.elapsed = time.monotonic()-started
        paths = self.eng.stats.setdefault('s3_paths', {});paths[col.s3_path] = paths.get(col.s3_path, 0)+1
        # em_load_limits.annotate: a valid boundary at the chart load cap is the plot ceiling
        if boundary >= 0 and self.eng.st[boundary].valid and fabs(self.eng.st[boundary].load-self.eng.cap(col.speed)) < 1e-8:
            col.status = STATUS_CEILING;col.reason = 'chart load search limit'
        return 0

    # ------------------------------------------------------------------ the trace
    @staticmethod
    def ok(NColumn col):
        return col is not None and col.status in (STATUS_VERIFIED, STATUS_CEILING) and col.boundary >= 0

    @cython.boundscheck(True)
    @cython.wraparound(True)
    def kind(self, NColumn col):
        return col.reason

    @cython.boundscheck(True)
    @cython.wraparound(True)
    def segments(self):
        from em_speed_limits import speed_limits
        from em_mach_events import solver_regions
        solver = self.eng.solver
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

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef tuple localize(self, NColumn good, NColumn bad, reason, double tol=-1.):
        cdef NColumn col
        cdef double mid
        if tol <= 0.:tol = self.place_tol
        while abs(bad.speed-good.speed) > tol:
            mid = .5*(good.speed+bad.speed)
            col = self.column(mid, [good, bad] if reason == 'corner' else [good], reason, True, pmin(good.speed, bad.speed),
                              pmax(good.speed, bad.speed))
            if NTracer.ok(col) and (reason == 'edge' or col.reason == good.reason):good = col
            else:bad = col
        return good, bad

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef sweep(self, dict segment, NColumn start):
        cdef NColumn last, prev, col, good, bad
        cdef double speed, step, n, error, h, r, bound
        cdef int skipped
        cdef bint ended
        lo, hi = segment['speed_kmh']
        for side in (-1., 1.):
            bound = lo if side < 0 else hi
            chain = [start];step = self.max_step;skipped = 0;ended = False
            while abs(chain[-1].speed-bound) > 1e-9 and not ended:
                last = chain[-1];prev = chain[-2] if len(chain) > 1 else None
                speed = last.speed+side*min(step, abs(bound-last.speed))
                if side > 0:col = self.column(speed, [last, prev], 'sweep', True, last.speed, bound+1e-9)
                else:col = self.column(speed, [last, prev], 'sweep', True, bound-1e-9, last.speed)
                # S6: a bound column placed inside the speed tolerance ends this direction
                ended = col.nudged_from is not None and abs(bound-speed) <= 1e-9
                if (not NTracer.ok(col) and col.status_name() != 'no feasible samples' and abs(bound-speed) > 1e-9
                        and skipped < 3):
                    self.events.append(dict(kind='unresolved', speed_kmh=speed, status=col.status_name()))
                    skipped += 1;step = abs(speed-last.speed)*2.
                    self.columns.pop(float(speed), None)
                    continue
                skipped = 0 if NTracer.ok(col) else skipped
                if not NTracer.ok(col):
                    # S6: an unresolved (not infeasible) column ends the sweep only to the speed tolerance
                    good, bad = self.localize(last, col, 'edge', self.vtol if col.status == STATUS_UNRESOLVED and self.s6 else -1.)
                    self.events.append(dict(kind='edge', speed_kmh=[good.speed, bad.speed],
                                            status=bad.status_name(), reason=bad.reason))
                    break
                if col.reason != last.reason:
                    chain.append(col);step = max(self.max_step/4., 2*self.vtol)
                    continue
                if prev is not None and prev.reason == last.reason:
                    r = (speed-last.speed)/(last.speed-prev.speed)
                    n = self.eng.st[last.boundary].load+r*(self.eng.st[last.boundary].load-self.eng.st[prev.boundary].load)
                    error = abs(self.M.turn_rate(speed, n)-self.eng.st[col.boundary].turn)
                    h = abs(speed-last.speed)
                    if error > 8*self.ttol:step = max(.5*h, 2*self.vtol)
                    elif error < 2*self.ttol:step = min(self.max_step, 1.5*max(step, h))
                chain.append(col)

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef list check(self):
        """MarchTracer.check: withheld midpoint boundary solves on every interval."""
        cdef NColumn a, b, col, g, h
        cdef double va, vb, v, x
        cdef int nw, i
        cdef double* xs
        cdef double* ns
        cdef double* coef
        # S6: the tried / passed intervals persist, so a later call checks only new intervals
        checks, tried, passed_set = [], self.check_tried, self.check_passed
        seams = self.seams

        def seam(lo, hi):
            return any(lo < x < hi or lo < y < hi for x, y in seams)
        for depth in range(self.max_depth):
            good = [c for c in sorted(self.columns.values(), key=lambda c: c.speed) if NTracer.ok(c)]
            added = 0
            for i in range(len(good)-1):
                a = good[i];b = good[i+1]
                va, vb = a.speed, b.speed
                key = (va, vb)
                if key in passed_set or key in tried or vb-va <= 2*self.vtol or seam(va, vb):continue
                if a.reason != b.reason and vb-va > 4*self.place_tol:
                    tried.add(key);added += 1
                    g, h = self.localize(a, b, 'corner')
                    self.events.append(dict(kind='corner', speed_kmh=[g.speed, h.speed], faces=[g.reason, h.reason]))
                    continue
                window = [c for c in good if va-150 <= c.speed <= vb+150 and not seam(min(c.speed, va), max(c.speed, vb))]
                nw = len(window)
                v = .5*(va+vb)
                col = self.column(v, [a, b], 'check', False, va, vb)
                v = col.speed
                ok = NTracer.ok(col) and nw > 1
                if ok:
                    xs = <double*>malloc(nw*sizeof(double));ns = <double*>malloc(nw*sizeof(double))
                    coef = <double*>malloc(4*nw*sizeof(double))
                    try:
                        for j, c in enumerate(window):
                            xs[j] = (<NColumn>c).speed;ns[j] = self.eng.st[(<NColumn>c).boundary].load
                        pchip_build(nw, xs, ns, coef)
                        box = []
                        for x in (v-self.vtol, v, v+self.vtol):
                            x = pmin_np(pmax_np(x, va), vb)
                            box.append(self.M.turn_rate(x, float(pchip_eval(nw, xs, coef, x))))
                    finally:
                        cfree(xs);cfree(ns);cfree(coef)
                    ok = min(box)-self.ttol <= self.eng.st[col.boundary].turn <= max(box)+self.ttol
                checks.append(dict(interval_kmh=[va, vb], speed_kmh=v, passed=bool(ok), faces=[a.reason, b.reason],
                                   midpoint_face=col.reason if NTracer.ok(col) else col.status_name()))
                if ok:
                    passed_set.add(key);self.witnesses.append(col);continue
                added += 1
                if a.reason != b.reason and vb-va > self.place_tol:
                    tried.add(key)
                    g, h = self.localize(a, b, 'corner')
                    self.events.append(dict(kind='corner', speed_kmh=[g.speed, h.speed], faces=[g.reason, h.reason]))
                else:
                    self.columns[v] = col
                    if not NTracer.ok(col):tried.add(key)
            if not added:break
        return checks

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef bint numerical_gap(self, NColumn c) except -1:
        """A march column without a verified boundary for a numerical reason: an unresolved boundary, or a level
        flight that did not close for a reason that is not a physical rejection."""
        if not c.march:return False
        if c.status == STATUS_UNRESOLVED:return True
        return c.status == STATUS_NOFEASIBLE and len(c.points) > 0 and not self.eng.physical(c.points[0])

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef list resolve_gaps(self, list segments):
        """S6: around every column that is a numerical gap in the drawn outline (an unresolved boundary anywhere, or
        a numerically unresolved level flight between verified columns of its segment) and wider than twice the speed
        tolerance, columns at up to the tolerance on both sides inside the verified bracket; the verified ones join
        the chart and the new intervals get the usual withheld checks. Returns those check records."""
        cdef NColumn c, left, right, alt
        cdef double q = .25*self.vtol, s, lo_b, hi_b
        cdef int rnd, added
        checks = []
        tried = set()
        for rnd in range(2):
            added = 0
            good = sorted((c for c in self.columns.values() if NTracer.ok(c)), key=lambda c: c.speed)
            for key in sorted(self.columns):
                c = self.columns.get(key)
                if c is None or not self.numerical_gap(c):continue
                seg = next((g['speed_kmh'] for g in segments if g['speed_kmh'][0]-1e-9 <= c.speed <= g['speed_kmh'][1]+1e-9), None)
                if seg is None:continue
                lefts = [g for g in good if seg[0]-1e-9 <= g.speed < c.speed]
                rights = [g for g in good if c.speed < g.speed <= seg[1]+1e-9]
                left = lefts[-1] if lefts else None;right = rights[0] if rights else None
                if c.status == STATUS_NOFEASIBLE and (left is None or right is None):continue  # an envelope end
                if left is not None and right is not None:
                    if right.speed-left.speed <= 2*self.vtol+1e-9:continue
                elif left is not None:
                    if c.speed-left.speed <= 2*self.vtol+1e-9:continue
                elif right is not None:
                    if right.speed-c.speed <= 2*self.vtol+1e-9:continue
                else:continue
                lo_b = left.speed if left is not None else seg[0]-1e-9
                hi_b = right.speed if right is not None else seg[1]+1e-9
                for side in (1., -1.):
                    # the nearest verified column on this side within the tolerance (stop at the first one)
                    if side > 0 and right is not None and right.speed-c.speed <= self.vtol+1e-9:continue
                    if side < 0 and left is not None and c.speed-left.speed <= self.vtol+1e-9:continue
                    for k in (1., 2., 4.):
                        s = c.speed+side*k*q
                        if not (lo_b < s < hi_b) or round(s, 6) in tried:continue
                        tried.add(round(s, 6))
                        self.probing = True
                        try:alt = self.column(s, [left, right], 'gap', False, lo_b, hi_b)
                        finally:self.probing = False
                        if NTracer.ok(alt):
                            self.columns[float(alt.speed)] = alt;added += 1
                            self.eng.stats['s6_gap_columns'] = self.eng.stats.get('s6_gap_columns', 0)+1
                            break
            if not added:break
            checks += self.check()
        return checks

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef within_tolerance(self, list segments):
        """S6: unresolved march columns, and unresolved sweep speeds, within twice the chart's speed tolerance of
        verified columns (between verified columns of one segment at most twice the tolerance apart, which the check
        accepts without a midpoint; or towards a segment end or edge, at most twice the tolerance beyond the last
        verified column) are resolved by those columns; the column is dropped and a 'within tolerance' event keeps its
        speed and reason. An unresolved column separated from every verified column of its segment by a 'no feasible
        samples' column lies beyond an explicit edge ('beyond edge'; the tracer never charts islands past an edge).
        Anything else stays an unresolved gap."""
        cdef NColumn c
        good = sorted(float(c.speed) for c in self.columns.values() if NTracer.ok(c))
        vtol = self.vtol

        def bracket(v):
            for seg in segments:
                lo, hi = seg['speed_kmh']
                if lo-1e-9 <= v <= hi+1e-9:break
            else:return None, None, False
            left = [x for x in good if lo-1e-9 <= x < v]
            right = [x for x in good if v < x <= hi+1e-9]
            l = left[-1] if left else None;r = right[0] if right else None
            if l is not None and r is not None:return l, r, r-l <= 2*vtol+1e-9
            if l is not None:return l, None, v-l <= 2*vtol+1e-9
            if r is not None:return None, r, r-v <= 2*vtol+1e-9
            return None, None, False
        ordered = sorted(self.columns.values(), key=lambda c: c.speed)
        status = [(float(c.speed), NTracer.ok(c), c.status == STATUS_NOFEASIBLE) for c in ordered]

        def beyond_edge(v):
            # a 'no feasible samples' column lies between v and every verified column of its segment on each side
            for seg in segments:
                lo, hi = seg['speed_kmh']
                if lo-1e-9 <= v <= hi+1e-9:break
            else:return False
            blocked = False
            for side in (-1, 1):
                seq = [x for x in status if lo-1e-9 <= x[0] <= hi+1e-9 and (x[0] < v if side < 0 else x[0] > v)]
                if side < 0:seq.reverse()
                for speed, ok, infeasible in seq:
                    if ok:return False
                    if infeasible:
                        blocked = True;break
            return blocked
        for key in list(self.columns):
            c = self.columns[key]
            if not self.numerical_gap(c):continue
            l, r, inside = bracket(c.speed)
            if c.status == STATUS_NOFEASIBLE:
                # only an interior one (verified columns on both sides) is a gap; at an envelope end it is the edge
                if l is None or r is None or not inside:continue
            kind = 'within tolerance' if inside else 'beyond edge' if beyond_edge(c.speed) else None
            if kind is None:continue
            del self.columns[key]
            self.events.append(dict(kind=kind, speed_kmh=c.speed, bracket_kmh=[l, r], role=c.role, status=c.status_name(),
                                    reason=str(c.march_reason if c.status == STATUS_UNRESOLVED else c.raw_reason)[:80]))
            self.eng.stats['s6_'+kind.replace(' ', '_')] = self.eng.stats.get('s6_'+kind.replace(' ', '_'), 0)+1
        for e in self.events:
            if e['kind'] != 'unresolved':continue
            l, r, inside = bracket(e['speed_kmh'])
            if inside:
                e['kind'] = 'within tolerance';e['bracket_kmh'] = [l, r]
                self.eng.stats['s6_within_tolerance'] = self.eng.stats.get('s6_within_tolerance', 0)+1

    @cython.boundscheck(True)
    @cython.wraparound(True)
    def run(self):
        import time
        cdef NColumn start, c, previous = None
        started = time.perf_counter()
        segments, limit, cuts = self.segments()
        self.seams = cuts
        for segment in sorted(segments, key=lambda s: -s['speed_kmh'][1]):
            lo, hi = segment['speed_kmh']
            start = self.column(hi, [previous] if previous is not None else [], 'start', True, lo-1e-9, hi+1e-9)
            if not NTracer.ok(start):
                start = self.column(.5*(lo+hi), [previous] if previous is not None else [], 'start', True, lo-1e-9, hi+1e-9)
            # S5: where neither start closes (level flight unresolved there), more start speeds across the
            # segment before it is given up (the Python march would leave the chart to the old sampler)
            for fraction in (.25, .75, .125, .375, .625, .875):
                if NTracer.ok(start):break
                start = self.column(lo+fraction*(hi-lo), [previous] if previous is not None else [], 'start', True, lo-1e-9,
                                    hi+1e-9)
            if NTracer.ok(start):
                self.sweep(segment, start)
                inside = [c for c in self.columns.values() if lo-1e-9 <= c.speed <= hi+1e-9 and NTracer.ok(c)]
                if inside:previous = min(inside, key=lambda c: c.speed)
            else:
                self.events.append(dict(kind='segment unresolved', speed_kmh=segment['speed_kmh'], status=start.status_name()))
        checks = self.check()
        if self.nudge:checks += self.resolve_gaps(segments)
        for c in self.witnesses:
            c.withheld = True
            self.columns.setdefault(float(c.speed), c)
        if self.nudge:self.within_tolerance(segments)
        cols = sorted(self.columns.values(), key=lambda c: c.speed)
        return dict(columns=cols, segments=segments, mach_seams_kmh=cuts, events=self.events, checks=checks,
                    work_by_role=self.work, elapsed_s=time.perf_counter()-started,
                    speed_limit=dict(kind=limit['kind'], speed_kmh=limit['speed_kmh'], enforced=limit['enforced']))

    # ------------------------------------------------------------------ column records
    @cython.boundscheck(True)
    @cython.wraparound(True)
    def record(self, NColumn col):
        """The column dict of the production column shape (as march_worker + sample_with_history built it)."""
        cdef int sid
        if not col.march:
            out = col.record
        else:
            for entry, sid in col.evidence:entry['point'] = self.eng.row_of(sid)
            points = [self.eng.row_of(sid) for sid in col.points]
            samples = [self.eng.row_of(sid) for sid in col.sample_points]
            if col.raw_status != 'no feasible samples':
                for p in points:p.setdefault('surface_sample', False)
            boundary = self.eng.row_of(col.boundary) if col.boundary >= 0 else None
            level = self.eng.row_of(col.level) if col.level >= 0 else None
            out = self.N.march_record(self.eng.solver, col, points, samples, boundary, level)
            if out.get('boundary_status') != col.status_name() or out.get('boundary_reason') != col.reason:
                raise RuntimeError('native march column status differs from its record: %r %r / %r %r' % (
                    out.get('boundary_status'), out.get('boundary_reason'), col.status_name(), col.reason))
        if col.role is not None:
            if out.get('lower_boundary') is None and out.get('points'):
                out['lower_boundary'] = next((p for p in out['points'] if abs(p['load_g']-1.) <= 1e-9 and p.get('valid')), None)
            out['s1_role'] = col.role;out['s1_evaluations'] = col.evaluations
        if col.withheld:out['withheld_check'] = True
        if col.nudged_from is not None:out['s6_requested_speed_kmh'] = float(col.nudged_from)
        return out

    @cython.boundscheck(True)
    @cython.wraparound(True)
    def column_state(self, NColumn col):
        return dict(points=list(col.points), boundary=col.boundary, level=col.level, status=col.status_name(),
                    reason=col.reason, march_reason=col.march_reason)
