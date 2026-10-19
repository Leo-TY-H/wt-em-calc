# S5: the march column worker (em_march.march_worker / column_record) and the speed
# tracer (em_march.MarchTracer) on native columns. Column records (Python dicts with
# chart rows) are built once, after the trace (NTracer.records).


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
        self.N = N;self.M = M;self.S = S

    # ------------------------------------------------------------------ one column
    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef NColumn column(self, double speed, list neighbours, why, bint keep=True):
        """MarchTracer.column -> run_column -> sample_with_history -> bounded_column -> march_worker."""
        global g_sink
        from em_trim_work import Work, use, current
        cdef NColumn col, c
        cdef int sid
        if self.on_column is not None:self.on_column(self, 'start', speed, why)
        seeds = []
        for c in neighbours:
            if c is None:continue
            for sid in c.points:
                if self.eng.st[sid].valid:seeds.append(sid)
        w = Work({}, parent=current())
        with use(w):col = self.run_column(speed, seeds)
        col.role = why;col.evaluations = w.counts.get('aircraft_evaluations', 0)
        row = self.work.setdefault(why, dict(columns=0, evaluations=0))
        row['columns'] += 1;row['evaluations'] += col.evaluations
        if keep:self.columns[float(speed)] = col
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
        col.march = True
        if nearest >= 0 and march.store:
            level = march.level_from(nearest)
        if level < 0:
            level = self.python_level(march, speed, nearest)
        healthy = nearest >= 0 and speed > self.eng.st[nearest].speed
        if not self.eng.st[level].valid:
            if not self.eng.physical(level) and healthy:
                reasons = ', '.join(REASON_TEXT[self.eng.st[level].reasons[i]] for i in range(self.eng.st[level].nreasons))
                self.production(col, speed, seeds, 'level flight unresolved: '+reasons)
                return 0
            reasons = ', '.join(REASON_TEXT[self.eng.st[level].reasons[i]] for i in range(self.eng.st[level].nreasons))
            self.column_record(col, march, -1, -1, STATUS_NOFEASIBLE, 'level flight: '+reasons, started)
            col.points = [level]
            col.region = False
            col.evidence = march.evidence;col.evidence_list = list(getattr(self.eng.solver, 'stationary_boundary_evidence', []))
            return 0
        has_predicted = self.predicted_alpha(speed, seeds, &predicted)
        boundary, status, reason = march.run(level, has_predicted, predicted)
        if boundary < 0:
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
    cdef int python_level(self, NMarch march, double speed, int nearest) except -1:
        """The production level-flight solve (em_sampling.solve_level), with the S1 unseeded retry."""
        global g_sink
        from em_trim_work import Work, use, current
        cdef int level, retry
        cdef Trial* t
        solver = self.eng.solver
        seed = None
        if nearest >= 0:seed = self.S.continuation_initial(solver, self.eng.row_of(nearest))
        if solver.is_prop and hasattr(solver.engine, 'reset_search'):solver.engine.reset_search()
        saved = <size_t>g_sink;g_sink = NULL
        try:
            w = Work({}, parent=current())
            with use(w):row = self.S.solve_level(solver, speed, seed)
            self.eng.stats['python_calls'] += 1
            level = self.eng.import_row(row)
            t = march.trial('level');t.ok = int(self.eng.st[level].valid);t.evals = w.counts.get('aircraft_evaluations', 0)
            t.fresh = -1
            if (not self.eng.st[level].valid and not self.eng.physical(level) and nearest >= 0
                    and speed > self.eng.st[nearest].speed):
                w = Work({}, parent=current())
                with use(w):row = self.S.solve_level(solver, speed, None)
                self.eng.stats['python_calls'] += 1
                retry = self.eng.import_row(row)
                t = march.trial('level retry');t.ok = int(self.eng.st[retry].valid)
                t.evals = w.counts.get('aircraft_evaluations', 0);t.fresh = -1
                if self.eng.st[retry].valid or self.eng.physical(retry):level = retry
        finally:
            g_sink = <long*>saved
        return level

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int production(self, NColumn col, double speed, list seeds, why) except -1:
        """em_march.production_column: the production boundary column search at this speed."""
        global g_sink
        rows = [self.eng.row_of(sid) for sid in seeds]
        saved = <size_t>g_sink;g_sink = NULL
        try:
            self.eng.flush_counts()
            record = self.M.production_column((self.aircraft, self.cj, float(speed), rows), why)
        finally:
            g_sink = <long*>saved
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
    cdef tuple localize(self, NColumn good, NColumn bad, reason):
        cdef NColumn col
        cdef double mid
        while abs(bad.speed-good.speed) > self.place_tol:
            mid = .5*(good.speed+bad.speed)
            col = self.column(mid, [good, bad] if reason == 'corner' else [good], reason)
            if NTracer.ok(col) and (reason == 'edge' or col.reason == good.reason):good = col
            else:bad = col
        return good, bad

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef sweep(self, dict segment, NColumn start):
        cdef NColumn last, prev, col, good, bad
        cdef double speed, step, n, error, h, r, bound
        cdef int skipped
        lo, hi = segment['speed_kmh']
        for side in (-1., 1.):
            bound = lo if side < 0 else hi
            chain = [start];step = self.max_step;skipped = 0
            while abs(chain[-1].speed-bound) > 1e-9:
                last = chain[-1];prev = chain[-2] if len(chain) > 1 else None
                speed = last.speed+side*min(step, abs(bound-last.speed))
                col = self.column(speed, [last, prev], 'sweep')
                if (not NTracer.ok(col) and col.status_name() != 'no feasible samples' and abs(bound-speed) > 1e-9
                        and skipped < 3):
                    self.events.append(dict(kind='unresolved', speed_kmh=speed, status=col.status_name()))
                    skipped += 1;step = abs(speed-last.speed)*2.
                    self.columns.pop(float(speed), None)
                    continue
                skipped = 0 if NTracer.ok(col) else skipped
                if not NTracer.ok(col):
                    good, bad = self.localize(last, col, 'edge')
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
        checks, tried, passed_set = [], set(), set()
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
                col = self.column(v, [a, b], 'check', keep=False)
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
    def run(self):
        import time
        cdef NColumn start, c, previous = None
        started = time.perf_counter()
        segments, limit, cuts = self.segments()
        self.seams = cuts
        for segment in sorted(segments, key=lambda s: -s['speed_kmh'][1]):
            lo, hi = segment['speed_kmh']
            start = self.column(hi, [previous] if previous is not None else [], 'start')
            if not NTracer.ok(start):
                start = self.column(.5*(lo+hi), [previous] if previous is not None else [], 'start')
            if NTracer.ok(start):
                self.sweep(segment, start)
                inside = [c for c in self.columns.values() if lo-1e-9 <= c.speed <= hi+1e-9 and NTracer.ok(c)]
                if inside:previous = min(inside, key=lambda c: c.speed)
            else:
                self.events.append(dict(kind='segment unresolved', speed_kmh=segment['speed_kmh'], status=start.status_name()))
        checks = self.check()
        for c in self.witnesses:
            c.withheld = True
            self.columns.setdefault(float(c.speed), c)
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
        return out

    @cython.boundscheck(True)
    @cython.wraparound(True)
    def column_state(self, NColumn col):
        return dict(points=list(col.points), boundary=col.boundary, level=col.level, status=col.status_name(),
                    reason=col.reason, march_reason=col.march_reason)
