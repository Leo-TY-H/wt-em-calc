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
#
# S7 (WT_MARCH_S7=0 restores S6 exactly): every verified column carries the exact slope of its boundary (column_slope:
# MarchEngine.tangent on the limiting face's own equations; one-sided towards the inside of its segment, both sides next
# to a model kink; the slope of the load cap on the plot ceiling). The outline between two columns on one limiting face
# (reason, control axis, RPM engine: face_of) is the cubic Hermite through their loads and slopes; between columns on
# two faces the two faces continued from their columns (slope and curvature) up to where they meet (corner_vertex).
# Withheld checks (check_s7) compare that interpolant, as em_march_chart.outline draws it, within half the envelope
# tolerance: at the midpoint, where the column's own exact slope must agree too (a hidden kink fails it), or at the
# predicted corner, which converges until both faces are drawn from nearby columns (a steep stretch of the face the
# corner column is not on gets its own withheld column: stretch_check); narrow intervals across a jump are halved to a
# quarter of the speed tolerance. The sweep predicts its next column with the face's slope and
# curvature and stops a step past the predicted low-speed edge; infeasible edges are refined to the speed tolerance
# with level-flight probes (edge_probe).
#
# S8 (WT_MARCH_S8=0 restores S7): one split rule replaces the withheld checks (check_s8). A drawn piece between two
# verified columns is finished when the bound of its error from its two exact end loads and slopes alone (band_load:
# the true boundary lies between the chord and the two tangents where its slope changes monotonically, a kink
# anywhere included; across an inflection within half the larger slope deviation times the width of the chord) is
# within 2/3 of the envelope turn tolerance, allowing the chart's horizontal slack, and the piece is at most the maximum
# step wide; otherwise a column is solved where that bound is largest. A piece narrower than the speed tolerance
# whose bound still fails is a discontinuity, drawn as a step.


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


cdef void pchip_slopes(int n, const double* x, const double* y, double* d) noexcept nogil:
    """S7: the PCHIP derivatives at the knots (PchipInterpolator(x, y).derivative()(x), the arithmetic of pchip_build)."""
    cdef int k
    cdef double w1, w2, whmean, h0, h1, m0, m1
    if n < 2:
        if n == 1:d[0] = 0.
        return
    if n == 2:
        d[0] = (y[1]-y[0])/(x[1]-x[0]);d[1] = d[0]
        return
    for k in range(1, n-1):
        h0 = x[k]-x[k-1];h1 = x[k+1]-x[k]
        m0 = (y[k]-y[k-1])/h0;m1 = (y[k+1]-y[k])/h1
        if (np_sign(m1) != np_sign(m0)) or m1 == 0. or m0 == 0.:
            d[k] = 0.
        else:
            w1 = 2*h1+h0;w2 = h1+2*h0
            whmean = (w1/m0+w2/m1)/(w1+w2)
            d[k] = 1.0/whmean
    d[0] = pchip_edge(x[1]-x[0], x[2]-x[1], (y[1]-y[0])/(x[1]-x[0]), (y[2]-y[1])/(x[2]-x[1]))
    d[n-1] = pchip_edge(x[n-1]-x[n-2], x[n-2]-x[n-3], (y[n-1]-y[n-2])/(x[n-1]-x[n-2]), (y[n-2]-y[n-3])/(x[n-2]-x[n-3]))


cdef double hermite_eval(int n, const double* x, const double* y, const double* dl, const double* dr, double v) noexcept nogil:
    """S7: piecewise cubic Hermite through (x, y): on [x[k], x[k+1]] the slope dr[k] (right of knot k) and dl[k+1]
    (left of knot k+1); the end cubics extrapolate. Same formula as em_march_chart.hermite."""
    cdef int i = 0, lo, hi, mid
    cdef double h, t, t2, t3
    if n == 1:return y[0]
    if v <= x[0]:i = 0
    elif v >= x[n-1]:i = n-2
    else:
        lo = 0;hi = n-2
        while lo < hi:
            mid = (lo+hi)//2
            if v < x[mid]:hi = mid
            elif v >= x[mid+1]:lo = mid+1
            else:
                lo = mid;break
        i = lo
    h = x[i+1]-x[i];t = (v-x[i])/h;t2 = t*t;t3 = t2*t
    return ((2*t3-3*t2+1)*y[i]+(t3-2*t2+t)*h*dr[i]+(-2*t3+3*t2)*y[i+1]+(t3-t2)*h*dl[i+1])


cdef void fill_slopes(int n, const double* x, const double* y, double* dl, double* dr) noexcept nogil:
    """S7: knots without an exact slope (NaN) take the PCHIP derivative there; an end knot in level flight (y = w = 0,
    where no exact slope exists) takes the slope of the parabola through both ends of its interval with the
    neighbour's slope (w is smooth and crosses zero at a contour's level-flight end). Same rule as
    em_march_chart.hermite_slopes."""
    cdef double d[256]
    cdef double* buf = d
    cdef int k, j
    cdef bint missing = False, lend, rend
    for k in range(n):
        if not (isfinite(dl[k]) and isfinite(dr[k])):missing = True
    if not missing:return
    if n > 256:
        buf = <double*>malloc(n*sizeof(double))
        if buf == NULL:return
    pchip_slopes(n, x, y, buf)
    lend = n >= 2 and y[0] == 0. and not isfinite(dl[0]) and not isfinite(dr[0])
    rend = n >= 2 and y[n-1] == 0. and not isfinite(dl[n-1]) and not isfinite(dr[n-1])
    for k in range(n):
        if not isfinite(dl[k]) and not isfinite(dr[k]):
            dl[k] = buf[k];dr[k] = buf[k]
        elif not isfinite(dl[k]):dl[k] = dr[k]
        elif not isfinite(dr[k]):dr[k] = dl[k]
    if lend:
        dl[0] = 2.*(y[1]-y[0])/(x[1]-x[0])-dl[1];dr[0] = dl[0]
    if rend:
        dr[n-1] = 2.*(y[n-1]-y[n-2])/(x[n-1]-x[n-2])-dr[n-2];dl[n-1] = dr[n-1]
    if buf != d:cfree(buf)


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


DEF S7_CHECK = .5   # S7: withheld envelope checks pass within half the envelope turn tolerance (the outline aims at it)
DEF S8_MAX_WIDTH = 25.   # S8: the widest outline piece (km/h; the validation's general-rule setting, 50 left features unseen)
DEF S8_BOUND = .6666666666666666   # S8: an outline piece is finished when its error bound is within 2/3 of the envelope
                                   # turn tolerance (the headroom of S7's curve checks)
DEF S14_TIE = .05    # S14: a margin of another hard limit at a column's boundary below which it can take over nearby


cdef inline double hermite_dload(double va, double na, double sa, double vb, double nb, double sb, double v) noexcept nogil:
    """S7: d(load)/dV of the cubic Hermite through (na, sa) at va and (nb, sb) at vb."""
    cdef double h = vb-va, t = (v-va)/h, t2 = t*t
    return ((6*t2-6*t)*na+(3*t2-4*t+1)*h*sa+(-6*t2+6*t)*nb+(3*t2-2*t)*h*sb)/h


cdef inline double face_load(double v0, double n0, double s0, double k0, double v) noexcept nogil:
    """S7: one limiting face beyond its column: load, slope and curvature there (a quadratic)."""
    return n0+(v-v0)*(s0+.5*k0*(v-v0))


cdef inline bint corner_vertex(double va, double na, double sa, double ka, double vb, double nb, double sb, double kb,
                               double* vc) noexcept nogil:
    """S7: the corner of an interval whose ends lie on different limiting faces: where the two faces, continued from
    their columns (face_load: load, exact slope and the curvature from the face's neighbouring column), meet. The
    boundary is the lower face (the first limit along alpha), so face a must lie below face b at va and above it at vb
    (within 0.1% in load: then the corner is that end); False otherwise (no corner: a jump or a face the columns do not
    describe). Bisection, the same 60 steps as em_march_chart.corner_vertex."""
    cdef double lo = va, hi = vb, mid, f, fa, fb, tol = 1e-3*pmax(fabs(na), fabs(nb))
    cdef int it
    if not (isfinite(sa) and isfinite(sb) and isfinite(ka) and isfinite(kb)):return False
    fa = na-face_load(vb, nb, sb, kb, va);fb = face_load(va, na, sa, ka, vb)-nb
    if fa > tol or fb < -tol:return False
    # within 0.1% in load of an end the corner is that end (the column there is on the other face's edge)
    if fa > 0.:
        vc[0] = va;return True
    if fb < 0.:
        vc[0] = vb;return True
    for it in range(60):
        mid = .5*(lo+hi)
        f = face_load(va, na, sa, ka, mid)-face_load(vb, nb, sb, kb, mid)
        if f > 0.:hi = mid
        else:lo = mid
    vc[0] = .5*(lo+hi)
    return True


cdef inline double interval_load(double va, double na, double sa, double ka, double pa, double vb, double nb, double sb,
                                 double kb, double pb, bint corner, bint exact, double v) noexcept nogil:
    """S7: the outline's load at v on [va, vb] (em_march_chart.boundary_interval): on one limiting face the cubic Hermite
    through loads and slopes (sa right of va, sb left of vb); across a face change the two faces up to their corner
    (corner_vertex) where both slopes are exact (``exact``) and the faces meet, else the Hermite with the PCHIP slopes
    (pa, pb; no overshoot)."""
    cdef double vc, h, t, t2, t3
    if corner:
        if exact and corner_vertex(va, na, sa, ka, vb, nb, sb, kb, &vc):
            return face_load(va, na, sa, ka, v) if v <= vc else face_load(vb, nb, sb, kb, v)
        sa = pa;sb = pb
    h = vb-va;t = (v-va)/h;t2 = t*t;t3 = t2*t
    return (2*t3-3*t2+1)*na+(t3-2*t2+t)*h*sa+(-2*t3+3*t2)*nb+(t3-t2)*h*sb


cdef class NTracer:
    """em_march.MarchTracer on native columns."""
    cdef MarchEngine eng
    cdef public object aircraft, config, cj, on_column, segments_, limit, seams, mach_seams, sweep_gaps
    cdef public double ttol, vtol, place_tol, max_step, march_tol
    cdef public int max_depth
    cdef public dict columns, work
    cdef public list witnesses, events
    cdef object N, M, S
    cdef bint production_fallback, nudge, s6, probing, s7, cheap, s8
    cdef set check_tried, check_passed
    cdef public list knot_speeds
    cdef public list seg_starts     # S9: (lo, hi, start speed) of every swept segment
    cdef double tangent_delta
    cdef bint exact_a, exact_b      # S7: interval_slopes found face slopes (exact or the face neighbour's) at both ends
    cdef int forced_level           # S9: the level flight the next column marches from (the dive path's walked state), or -1
    cdef public list ias_knots      # S9: forward-IAS breakpoints of the control authority (m/s): effective speeds, floors
    cdef double sigma               # S9: sqrt(density ratio) at the chart altitude (forward IAS = u sigma)
    cdef public list s9_knots       # S9: the model breakpoints in speed the outline is checked at (model knots, Mach tables)
    cdef int brake_level            # S10: a braking level flight found from an interior start (interior_march), or -1

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
        # S7: WT_MARCH_S7=0 restores the S6 tracer exactly (PCHIP checks, corners bisected to the placement tolerance)
        self.s7 = os.environ.get('WT_MARCH_S7', '1') != '0' and self.s6
        # S8: WT_MARCH_S8=0 restores the S7 edge search (no rescue columns at the edge)
        self.s8 = os.environ.get('WT_MARCH_S8', '1') != '0' and self.s7
        self.knot_speeds = []
        self.seg_starts = []
        self.tangent_delta = 1.
        self.cheap = False     # S7: a column without rescues or nudges (the edge refinement column)
        self.forced_level = -1
        self.ias_knots = [];self.sigma = 1.;self.s9_knots = []
        self.brake_level = -1

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
            if self.eng.ncut and self.eng.across(c.speed, speed):continue
            for sid in c.points:
                if self.eng.st[sid].valid:seeds.append(sid)
            # S8: a level flight over a forward-IAS limit seeds the next column's level flight
            if c.level >= 0 and self.eng.ias_soft(c.level):seeds.append(c.level)
        w = Work({}, parent=current())
        with use(w):
            col = self.run_column(speed, seeds)
            if col.status == STATUS_UNRESOLVED and col.march and self.nudge and not self.cheap and not (self.s8 and col.jump):
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
            if self.s7 and NTracer.ok(col):self.column_slope(col)
            if self.s8 and NTracer.ok(col) and col.lower >= 0:self.lower_slope(col)
        col.role = why;col.evaluations = w.counts.get('aircraft_evaluations', 0)
        row = self.work.setdefault(why, dict(columns=0, evaluations=0))
        row['columns'] += 1;row['evaluations'] += col.evaluations
        if keep:self.columns[float(col.speed)] = col
        if self.on_column is not None:self.on_column(self, 'done', speed, why)
        return col

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int slope_sides(self, double speed) except -1:
        """S7: which one-sided slopes a column needs: both next to a model kink (two pieces meet there), else one, towards
        the inside of its segment (a one-sided difference over the slope step is exact to well below the tolerances)."""
        cdef double v
        for v in self.knot_speeds:
            if fabs(v-speed) <= 2*self.vtol:return 3
        for seg in self.segments_ or ():
            if seg['speed_kmh'][0]-1e-9 <= speed <= seg['speed_kmh'][1]+1e-9:
                return 1 if speed+self.tangent_delta > seg['speed_kmh'][1] else 2
        return 2

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int lower_slope(self, NColumn col) except -1:
        """S8: the exact slope dw/dV of a column's lower edge (the leaning forward-IAS edge): MarchEngine.tangent on the
        forward-IAS event residual (native event 7) through the edge state; NaN where unavailable."""
        cdef double l = NAN, r = NAN
        cdef int base = self.eng.st[col.lower].base
        self.eng.flush_counts()
        if self.eng.tangent(base, 1, 7, 0, 0, self.eng.st[base].speed, self.tangent_delta, self.slope_sides(col.speed), &l, &r):
            if isfinite(l) and isfinite(r):col.lower_slope = .5*(l+r)
            elif isfinite(l):col.lower_slope = l
            elif isfinite(r):col.lower_slope = r
        return 0

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int column_slope(self, NColumn col) except -1:
        """S7: the exact slope dn/dV of the column's boundary (both sides): the load cap's on the plot ceiling; the
        limiting face's own equations through the boundary state (MarchEngine.tangent: the event residual for stall,
        wing force, control, Instructor and engine RPM faces, fixed alpha at a load maximum). Pitch-response faces and
        events without a native residual have none (the outline takes the PCHIP slope there)."""
        cdef int b = col.boundary, base, i, nev, mode = -1, code = 0, axis = 0, side = 0, kind = -1, laxis = -1
        cdef double wl = NAN, wr = NAN, n, d = self.tangent_delta, c0, cp, cm
        cdef Event ev[32]
        if b < 0:return 0
        base = self.eng.st[b].base
        n = self.eng.st[base].load
        if col.status == STATUS_CEILING:
            c0 = self.eng.cap(col.speed);cp = self.eng.cap(col.speed+d);cm = self.eng.cap(col.speed-d)
            col.slope_l = (c0-cm)/d;col.slope_r = (cp-c0)/d;col.slope_kind = 'load cap'
            return 0
        limit = self.eng.limits[self.eng.st[b].limit] if self.eng.st[b].limit >= 0 else None
        if limit is None:return 0
        kind = KIND_CODE.get(limit.get('kind'), -1)
        if limit.get('axis') is not None:laxis = int(limit['axis'])
        if kind == KD_FOLD:
            mode = 2
        elif kind in (KD_STALL, KD_WING, KD_CONTROL, KD_INSTR, KD_RPM, KD_BRAKE):
            engine = int((limit.get('engine_rpm_limit') or {}).get('engine', -1))
            nev = self.eng.nearest_events(base, ev)
            for i in range(nev):
                if ev[i].kind != kind or ev[i].code <= 0:continue
                if kind == KD_CONTROL and laxis >= 0 and ev[i].axis != laxis:continue
                if kind == KD_RPM and engine >= 0 and ev[i].engine != engine:continue
                mode = 1;code = ev[i].code;side = ev[i].side
                axis = ev[i].engine if code == 6 else (ev[i].axis-2 if ev[i].axis >= 0 else 0)
                break
        if mode < 0:
            col.slope_kind = 'none: '+str(limit.get('kind'))
            return 0
        self.eng.flush_counts()
        if not self.eng.tangent(base, mode, code, axis, side, self.eng.st[base].speed, d, self.slope_sides(col.speed), &wl, &wr):
            col.slope_kind = 'unavailable: '+str(limit.get('kind'))
            return 0
        if isfinite(wl) and isfinite(wr) and fabs(wl-wr) <= .05*pmax(fabs(wl), fabs(wr)):
            wl = .5*(wl+wr);wr = wl
        col.slope_l = wl/(2.*n);col.slope_r = wr/(2.*n)
        if not isfinite(col.slope_l):col.slope_l = col.slope_r
        if not isfinite(col.slope_r):col.slope_r = col.slope_l
        col.slope_kind = ('fixed alpha' if mode == 2 else 'face')+': '+str(limit.get('kind'))
        return 0

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
        level_seeds = [sid for sid in seeds if self.eng.ias_soft(sid) and self.eng.st[sid].has_coords]
        seeds = [sid for sid in seeds if self.eng.st[sid].valid and self.eng.st[sid].has_coords]
        for sid in seeds+level_seeds:
            if fabs(self.eng.st[sid].load-1.) <= 1e-12:
                d = fabs(self.eng.st[sid].speed-speed)
                if nearest < 0 or d < bd:
                    nearest = sid;bd = d
        march = NMarch(self.eng, speed, self.march_tol)
        if self.eng.s9:
            # S9: where two level-flight branches overlap, the column follows the branch its sweep follows (the one
            # continued from the segment's start; the dense truth's own continuation): level flight from the nearest
            # seed on the start's side, the other side only where that side has none (P1Y1 SB 194-196 km/h)
            march.pref_side = self.start_side(speed)
            if march.pref_side != 0.:
                for sid in seeds+level_seeds:
                    if (fabs(self.eng.st[sid].load-1.) <= 1e-12 and (self.eng.st[sid].speed-speed)*march.pref_side > 0.
                            and (nearest < 0 or (self.eng.st[nearest].speed-speed)*march.pref_side <= 0.
                                 or fabs(self.eng.st[sid].speed-speed) < fabs(self.eng.st[nearest].speed-speed))):
                        nearest = sid
        march.strict_faces = self.s6
        march.no_rescue = self.cheap
        col.march = True
        if self.forced_level >= 0 and fabs(self.eng.st[self.forced_level].speed-speed) <= 1e-9:
            level = self.forced_level
        elif nearest >= 0 and march.store:
            level = march.level_from(nearest)
        if level < 0:
            level = self.python_level(march, speed, nearest)
        if (self.eng.s9 and not self.cheap and not self.probing
                and (level < 0 or not (self.eng.st[level].valid or self.eng.ias_soft(level) or self.eng.level_physical(level)))):
            # S9: level flight that does not close from the seeds (a far neighbour across a blade-polar jump the jump check
            # cannot see from there): continued from the nearest admitted level flight below, as the dive path reaches it
            below = -1
            for sid in seeds:
                if (fabs(self.eng.st[sid].load-1.) <= 1e-12 and self.eng.st[sid].valid and self.eng.st[sid].speed < speed-1e-9
                        and not (self.eng.ncut and self.eng.across(self.eng.st[sid].speed, speed))
                        and (below < 0 or self.eng.st[sid].speed > self.eng.st[below].speed)):below = sid
            if below >= 0:
                sid = self.level_walk(below, speed)
                if sid >= 0:level = sid
        if level < 0:
            # S8: no level-flight candidate at all
            self.column_record(col, march, -1, -1, STATUS_NOFEASIBLE, 'level flight: no established candidate', started)
            col.points = [];col.region = False
            return 0
        healthy = nearest >= 0 and speed > self.eng.st[nearest].speed
        # S8: a level flight over a forward-IAS limit only is no edge: the march starts from it (states at higher
        # alpha can lie inside the limit; the column's lower edge is where they begin)
        if not (self.eng.st[level].valid or self.eng.ias_soft(level)):
            if not self.eng.level_physical(level) and healthy:
                reasons = ', '.join(REASON_TEXT[self.eng.st[level].reasons[i]] for i in range(self.eng.st[level].nreasons))
                # S5: march from an admitted state of this speed on the neighbouring columns' alpha path
                # (their march from level flight reached it without a limit) instead of the production column
                self.brake_level = -1
                if not self.cheap and self.interior_march(col, march, speed, seeds, 'level flight unresolved: '+reasons, started):return 0
                if self.brake_level >= 0:
                    # S10: level flight at this speed brakes (found from the interior start): an edge, not a gap
                    level = self.brake_level
                    reasons = ', '.join(REASON_TEXT[self.eng.st[level].reasons[i]] for i in range(self.eng.st[level].nreasons))
                    self.column_record(col, march, -1, -1, STATUS_NOFEASIBLE, 'level flight: '+reasons, started)
                    col.points = [level];col.region = False
                    return 0
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
        if boundary < 0 and not self.cheap:
            out = march.neighbour_face(seeds, level)
            if out is not None and out[0] >= 0:
                march.s3_flags.add('march unresolved: '+str(reason)[:60])
                boundary, status, reason = out
        col.jump = march.jump_seen
        # S8 (item 5): no S6 robust march where the march met a blade-polar jump (the sliding state is the answer there)
        if boundary < 0 and self.s6 and not self.probing and not self.cheap and not (self.s8 and march.jump_seen):
            # S6: the march again with robust corrections (regime seeds, larger budget)
            out = march.robust_march(level, has_predicted, predicted)
            if out is not None:
                march.s3_flags.add('march unresolved: '+str(reason)[:60])
                boundary, status, reason = out
        # S8: the soft limits at the stop (NMarch.settle)
        if boundary >= 0:boundary, status, reason = march.settle((boundary, status, reason))
        if boundary < 0 and status == 'no feasible samples':
            # S8: nothing inside the forward-IAS limit up to the first hard limit: past the leaning edge
            self.column_record(col, march, -1, -1, STATUS_NOFEASIBLE, 'level flight: '+str(reason), started)
            col.points = [level]
            col.region = False
            return 0
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
        if self.eng.ias_soft(level):
            col.lower = march.lower_edge(level, boundary)
            if col.lower < 0:
                self.column_record(col, march, -1, -1, STATUS_NOFEASIBLE, 'level flight: forward-IAS limit', started)
                col.points = [level];col.region = False
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
        cdef double d, best = INFINITY, predicted, w, f
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
        if start < 0 and self.eng.s12:
            # S12: the neighbouring columns' march from level flight met no limit anywhere along its alpha path, not only
            # at the samples it kept (AM-1 RB 804.8 km/h: the 804.1 km/h column kept only its level flight and its
            # boundary, and that boundary's alpha lies past this speed's control face): starts a quarter, half and three
            # quarters of the way from the nearest neighbour's level flight to its lowest sample above it, seeded from
            # that sample (each side's nearest column, the nearer first)
            firsts = {}
            for sid in near:
                firsts.setdefault(self.eng.st[sid].speed > speed, sid)
            for sid in sorted(firsts.values(), key=lambda q: fabs(self.eng.st[q].speed-speed)):
                lv = -1
                for p in seeds:
                    if (fabs(self.eng.st[p].load-1.) <= 1e-12 and self.eng.st[p].valid
                            and self.eng.st[p].speed == self.eng.st[sid].speed):
                        lv = p;break
                if lv < 0 or not self.eng.st[sid].alpha > self.eng.st[lv].alpha:continue
                for f in (.25, .5, .75):
                    p = march.at_state(sid, self.eng.st[lv].alpha+f*(self.eng.st[sid].alpha-self.eng.st[lv].alpha),
                                       's12 interior start')
                    if p >= 0 and self.eng.st[p].valid:
                        start = p;break
                if start >= 0:
                    self.eng.stats['s12_path_starts'] = self.eng.stats.get('s12_path_starts', 0)+1
                    break
        if start < 0:return False
        if self.eng.brake_limit:
            # S10 (not S11, where braking is no limit): the boundary is the first limit along alpha from level flight:
            # where level flight at this speed, continued in load from the interior start (w halved, then to a quarter, a tenth and level flight), brakes (and no pulling
            # level flight is found: MarchEngine.brake_reseed), level flight is physically rejected here, whatever the
            # states at higher alpha do (F2G-1 RB 786 km/h: braking at 1 g, pulling at 3.8 g), so the column is an edge
            p = start
            w = self.eng.st[start].load**2-1.
            for f in (.5, .25, .1, 0.):
                p = self.eng.correct(p, self.eng.st[p].sol, self.eng.st[p].load, True, FIX_LOAD, sqrt(1.+f*w), 0, 0, 0, ROBUST_NFEV,
                                     ROBUST_EVALS, 2e-6, speed, NULL, NULL)
                if p < 0 or not (self.eng.st[p].valid or self.eng.has_code(p, R_PROP_BRAKE)):break
            self.eng.flush_counts()
            if p >= 0 and fabs(self.eng.st[p].load-1.) <= 1e-9 and self.eng.has_code(p, R_PROP_BRAKE) and self.eng.level_physical(p):
                self.brake_level = p
                self.eng.stats['s10_interior_brake'] = self.eng.stats.get('s10_interior_brake', 0)+1
                return False
        has_predicted = self.predicted_alpha(speed, seeds, &predicted)
        boundary, status, reason = march.run(start, has_predicted, predicted)
        if boundary < 0 and self.s6 and not self.probing:
            out = march.robust_march(start, has_predicted, predicted)
            if out is not None:
                boundary, status, reason = out
                march.s3_flags.add('s6 robust march')
        if boundary >= 0:boundary, status, reason = march.settle((boundary, status, reason))
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
        from em_trim_work import Work, use, current, WorkLimit
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
        try:
            with use(w):level = ls.solve(speed, seed)
        except WorkLimit as error:
            # S8: no established candidate at all (a speed far from any level flight): no level state, an edge
            if 'no established candidate' not in str(error):raise
            self.eng.stats['s8_level_no_candidate'] = self.eng.stats.get('s8_level_no_candidate', 0)+1
            return -1
        t = march.trial('level');t.ok = int(self.eng.st[level].valid)
        t.evals = self.eng.counts[K_EVAL]-e0+w.counts.get('aircraft_evaluations', 0)
        t.fresh = -1
        if (not self.eng.st[level].valid and not self.eng.level_physical(level) and not self.eng.ias_soft(level) and nearest >= 0
                and speed > self.eng.st[nearest].speed):
            e0 = self.eng.counts[K_EVAL]
            w = Work({}, parent=current())
            try:
                with use(w):retry = ls.solve(speed, None)
            except WorkLimit as error:
                if 'no established candidate' not in str(error):raise
                return level
            t = march.trial('level retry');t.ok = int(self.eng.st[retry].valid)
            t.evals = self.eng.counts[K_EVAL]-e0+w.counts.get('aircraft_evaluations', 0);t.fresh = -1
            if self.eng.st[retry].valid or self.eng.level_physical(retry):level = retry
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
        if boundary >= 0 and self.eng.soft:
            # S8: below a verified stop, an over-rev state (rejected for engine RPM only, its hard checks passed) is a
            # transient over-rev: admitted by the chart rule, drawn and seeded like any admitted state
            ba = self.eng.st[boundary].alpha
            for sid in march.samples:
                if (not self.eng.st[sid].valid and fabs(self.eng.st[sid].speed-col.speed) <= 1e-9
                        and self.eng.st[sid].alpha <= ba+1e-9 and self.eng.only_reasons(sid, (<unsigned long long>1) << R_RPM)):
                    self.eng.st[sid].valid = True;self.eng.st[sid].rule = 1
                    self.eng.stats['s8_transient_over_rev'] = self.eng.stats.get('s8_transient_over_rev', 0)+1
            if self.eng.st[boundary].base != boundary and self.eng.st[self.eng.st[boundary].base].rule == 1:
                self.eng.st[boundary].valid = True;self.eng.st[boundary].rule = 1
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
        """(segments, speed limit, cuts): em_march_chart.chart_segments; the cuts (no interpolation or withheld check
        across them) are the Mach seams and, S8, the stretches where the selected wing sweep is unavailable."""
        import em_march_chart as K
        out, limit, seams, gaps = K.chart_segments(self.eng.solver)
        self.mach_seams = seams;self.sweep_gaps = gaps
        return out, limit, sorted([tuple(s) for s in seams]+[tuple(g) for g in gaps])

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
    cdef int level_probe(self, int level, double speed, PredMap* preds) except -2:
        """S7: level flight at ``speed`` continued from a level-flight state (the march's own level correction; once more
        with fresh derivatives and the S6 robust budget when it does not close). The admitted state, -1 for a balanced
        physical rejection, -3 where the correction did not close (no evidence either way)."""
        cdef int p = self.eng.correct(level, self.eng.st[level].sol, 1., True, FIX_LOAD, 1., 0, 0, 0, MARCH_NFEV, MARCH_EVALS,
                                      2e-6, speed, preds, NULL)
        if p < 0 or (not self.eng.st[p].valid and not self.eng.level_physical(p)):
            p = self.eng.correct(level, self.eng.st[level].sol, 1., True, FIX_LOAD, 1., 0, 0, 0, ROBUST_NFEV, ROBUST_EVALS, 2e-6,
                                 speed, NULL, NULL)
        self.eng.flush_counts()
        if p >= 0 and self.eng.st[p].valid:return p
        if self.s8 and self.eng.sliding and self.eng.is_prop:
            # S8 (item 2): next to a blade-polar jump, level flight with the shafts sliding on it (NMarch.slide_at's rule)
            q = self.slide_probe(level, speed)
            if q >= 0:return q
        return -1 if p >= 0 and self.eng.level_physical(p) else -3

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef double start_side(self, double speed):
        """S9: +1 / -1 where the segment holding ``speed`` started its sweep above / below it, 0 elsewhere."""
        for lo, hi, s0 in self.seg_starts:
            if lo-1e-9 <= speed <= hi+1e-9:
                return 1. if s0 > speed+1e-9 else -1. if s0 < speed-1e-9 else 0.
        return 0.

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef bint healthy(self, int sid) noexcept:
        """S9: every hard-limit margin of the state is at least S9_FOLD_MARGIN."""
        cdef int k
        for k in range(NKD):
            if (self.eng.st[sid].mset >> k) & 1 and not self.eng.soft_kind(k) and self.eng.st[sid].m[k] < S9_FOLD_MARGIN:return False
        return True

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int level_walk(self, int level, double speed) except -2:
        """S9: level flight at ``speed`` continued from the level-flight state ``level`` of a lower speed, as the aircraft
        reaches it by speeding up at 1 g (the dive path): steps of at most the placement tolerance, a step that does not
        close halved down to the speed tolerance (level_probe: the sliding state where the jump check at the step's own
        speed confirms a blade-polar jump). The admitted state at ``speed``; -1 where level flight is physically
        rejected on the way (an edge lies before it) or a step of the speed tolerance does not close."""
        from em_trim_work import Work, use, current
        cdef double v = self.eng.st[level].speed, step = self.place_tol, h, nxt
        cdef int p
        cdef PredMap preds
        preds.n = 0
        w = Work({}, parent=current())
        try:
            with use(w):
                while speed-v > 1e-9:
                    h = pmin(step, speed-v)
                    nxt = speed if speed-(v+h) < .25*self.vtol else v+h
                    p = self.level_probe(level, nxt, &preds)
                    if p >= 0:
                        v = nxt;level = p;step = pmin(self.place_tol, 2.*step)
                        continue
                    if p == -1 or h <= self.vtol*(1.+1e-9):return -1
                    step = pmax(self.vtol, .5*h)
        finally:
            row = self.work.setdefault('level walk', dict(columns=0, evaluations=0))
            row['columns'] += 1;row['evaluations'] += w.counts.get('aircraft_evaluations', 0)
        self.eng.stats['s9_level_walks'] = self.eng.stats.get('s9_level_walks', 0)+1
        return level

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int slide_probe(self, int level, double speed) except -2:
        """S8: the level-flight probe from ``level`` at ``speed`` with the shafts sliding on a blade-polar jump the
        jump check confirms at the seed (from a sliding seed: the regular correction). The admitted state, or -1."""
        cdef SlideSpec spec
        cdef int p
        if self.eng.st[level].slide.n > 0:
            spec.n = 0
        else:
            self.eng.slide_candidate(level, &spec, self.eng.st[level].sol[0], self.eng.st[level].sol[1], 1., True, speed)
            if spec.n == 0:return -1
        self.eng.slide_override = spec;self.eng.use_override = True
        try:
            p = self.eng.correct(level, self.eng.st[level].sol, 1., True, FIX_LOAD, 1., 0, 0, 0, ROBUST_NFEV, ROBUST_EVALS, 2e-6,
                                 speed, NULL, NULL)
        finally:
            self.eng.use_override = False
        self.eng.flush_counts()
        if p >= 0 and self.eng.st[p].valid:
            self.eng.stats['s8_slide_probes'] = self.eng.stats.get('s8_slide_probes', 0)+1
            return p
        return -1

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef tuple edge_probe(self, NColumn good, NColumn bad):
        """S7: an edge beyond which level flight is infeasible, to the speed tolerance, with level-flight probes
        (level_probe: corrections continued from the verified column's own level-flight state; no march, no production
        level search): from the verified column towards the infeasible one in steps of the placement tolerance (bands
        without an equilibrium are sampled as finely as the S6 column bisection did); after a step that fails, on in
        steps of the speed tolerance from the last admitted level flight (a correction that does not close over a longer
        continuation step near the edge is no evidence of it), back to the placement tolerance once past the failed
        speed; the first step of the speed tolerance that fails, or the infeasible column, is halved once more. Then one
        column without rescues at the last probed speed whose level flight was admitted; where it does not verify, at the
        two admitted speeds before it, and from the first of those that verifies (the march's predictors from a
        neighbouring column) once more at the last admitted speed. Returns (the edge's verified column, the nearest
        probed speed beyond it). Where no refinement column verifies, the S6 column bisection to the placement tolerance
        follows."""
        from em_trim_work import Work, use, current
        cdef double out = bad.speed, inside = good.speed, nxt, h, step, failed = bad.speed, v, leapt = NAN
        cdef int k
        cdef double direction = 1. if bad.speed > good.speed else -1.
        cdef int level = good.level, p
        cdef PredMap preds
        cdef NColumn col
        cdef bint branched = False
        cdef double halved_from = NAN
        preds.n = 0
        if level < 0 or not self.eng.st[level].valid or fabs(self.eng.st[level].load-1.) > 1e-9:
            g, b = self.localize(good, bad, 'edge')
            return g, b.speed
        admitted = []
        w = Work({}, parent=current())
        with use(w):
            step = self.place_tol
            while True:
                h = pmin(step, fabs(bad.speed-inside))
                nxt = inside+direction*h
                if fabs(bad.speed-nxt) <= 1e-9:
                    p = -1          # the infeasible column itself
                else:
                    p = self.level_probe(level, nxt, &preds)
                if p >= 0:
                    inside = nxt;level = p;admitted.append(nxt)
                    if step < self.place_tol and (inside-failed)*direction >= 0.:
                        step = self.place_tol;halved_from = NAN
                    continue
                out = nxt
                if self.eng.s9 and p == -3 and self.vtol*.25*(1.+1e-9) < h <= self.vtol*(1.+1e-9):
                    # S9: a level flight that does not close over the speed tolerance is continued in halved steps (to a
                    # quarter of it) before it counts as non-closing: a stiff stretch (the Instructor's level flight on
                    # J29A RB closes in steps of 0.2 km/h, not 0.5) walks on to its own edge
                    self.eng.stats['s9_probe_halvings'] = self.eng.stats.get('s9_probe_halvings', 0)+1
                    if not isfinite(halved_from):halved_from = inside
                    failed = nxt;step = .5*h
                    continue
                if h <= self.vtol*(1.+1e-9):
                    if self.s8 and p == -3 and inside != leapt:
                        # S8: a level flight that does not close is no evidence of the edge (a band without a steady
                        # state at a blade-polar jump, or a stiff stretch): probes leaping 2, 4 and 8 speed tolerances
                        # (at least 1 km/h each) past the last admitted level flight; one that is admitted, with no
                        # physical rejection of level flight halfway back to the failed probe, continues the probes there
                        leapt = inside
                        for k in range(1, 4):
                            v = inside+direction*pmin((2 << (k-1))*pmax(self.vtol, 1.), fabs(bad.speed-inside))
                            if fabs(bad.speed-v) <= 1e-9:break
                            p = self.level_probe(level, v, &preds)
                            if p >= 0:break
                            if p == -1:
                                out = v;break
                        if p >= 0 and self.level_probe(p, .5*(nxt+v), &preds) == -1:
                            # a physical rejection of level flight inside the leapt band (an engine over-rev at 1 g, a
                            # stall): the edge is there, whatever lies beyond it
                            out = .5*(nxt+v);p = -1
                        if p >= 0:
                            self.eng.stats['s8_probe_leaps'] = self.eng.stats.get('s8_probe_leaps', 0)+1
                            inside = v;level = p;admitted.append(v);failed = v;step = self.vtol
                            continue
                        if self.eng.s9 and p == -3 and not branched:
                            # S9: no leap closes from this branch's level flight and none is physically rejected: the
                            # branch may end at a fold where level flight continues on another one (N1K1 SB: a second
                            # propeller-wash branch below 156.5 km/h, down to 145.4). The level-flight search (its own
                            # seeds) at the first leap; where it finds admitted level flight, the probes continue from it.
                            # The followed branch keeps the speeds where it was admitted: its last admitted probe speed
                            # (within the speed tolerance of its end) becomes a branch cut (no state is carried across it,
                            # so every column on its side starts from it, every column beyond from the other branch;
                            # columns right at the fold of the followed branch would need the S6 rescues)
                            # (only where nothing explains the stop: every hard-limit margin of that level flight at least
                            # S9_FOLD_MARGIN; a level flight closing in on its stall or another limit ends there)
                            branched = True
                            v = inside+direction*pmin(2*pmax(self.vtol, 1.), fabs(bad.speed-inside))
                            # S14: also where a limit is close (Hurricane Mk I/B SB 136.9 km/h: level flight near its stall
                            # stops closing, the other branch's level flight continues to 134.0 km/h; the chart ended 2.9
                            # km/h early): the stop is only explained where the probes met a physical rejection, and none did
                            if fabs(bad.speed-v) > 1e-9 and (self.healthy(level) or self.eng.s14):
                                self.eng.stats['s9_probe_searches'] = self.eng.stats.get('s9_probe_searches', 0)+1
                                if not self.healthy(level):
                                    self.eng.stats['s14_probe_searches'] = self.eng.stats.get('s14_probe_searches', 0)+1
                                q = self.python_level(NMarch(self.eng, v, self.march_tol), v, -1)
                                if (q >= 0 and self.eng.st[q].valid and fabs(self.eng.st[q].load-1.) <= 1e-9
                                        and self.level_probe(q, .5*(nxt+v), &preds) != -1):
                                    self.eng.stats['s9_probe_branches'] = self.eng.stats.get('s9_probe_branches', 0)+1
                                    # (before the halved steps: the followed branch right at its fold needs the S6
                                    # rescues in every column there)
                                    if isfinite(halved_from):inside = halved_from
                                    self.eng.add_cut(inside)
                                    self.events.append(dict(kind='branch cut', speed_kmh=[inside, nxt], branch_level_kmh=v,
                                                            reason='level flight continues on another branch past the fold'))
                                    admitted = [x for x in admitted if (x-inside)*direction <= 1e-9]
                                    inside = v;level = q;admitted.append(v);failed = v;step = self.vtol;halved_from = NAN
                                    continue
                    break
                failed = nxt;step = self.vtol
            if fabs(out-inside) > .5*self.vtol:
                nxt = .5*(inside+out)
                p = self.level_probe(level, nxt, &preds)
                if p >= 0:
                    inside = nxt;level = p;admitted.append(nxt)
                else:
                    out = nxt
            if self.eng.s9 and isfinite(halved_from) and fabs(inside-halved_from) < self.vtol*(1.-1e-9):
                # S9: halved steps that walked less than the speed tolerance past the last full step place the edge no
                # better than S8's one halving did: the edge columns start where S8's would (the last full step, or
                # half a speed tolerance past it where that was admitted); the bracket stays the halved one
                v = halved_from+.5*self.vtol*direction
                inside = v if any(fabs(x-v) <= 1e-6 for x in admitted) else halved_from
                admitted = [x for x in admitted if (x-inside)*direction <= 1e-9]
        row = self.work.setdefault('edge probe', dict(columns=0, evaluations=0))
        row['columns'] += 1;row['evaluations'] += w.counts.get('aircraft_evaluations', 0)
        self.eng.stats['s7_edge_probes'] = self.eng.stats.get('s7_edge_probes', 0)+1
        if inside == good.speed:return good, out
        seeds = [good]
        for k in range(len(admitted)-1, max(-1, len(admitted)-4), -1):
            v = admitted[k]
            self.cheap = True
            try:col = self.column(v, seeds, 'edge', False, pmin(v, out), pmax(v, good.speed))
            finally:self.cheap = False
            if NTracer.ok(col):
                self.columns[float(col.speed)] = col
                if k == len(admitted)-1:return col, out
                v = admitted[len(admitted)-1]
                self.cheap = True
                try:near = self.column(v, [col]+seeds, 'edge', False, pmin(v, out), pmax(v, col.speed))
                finally:self.cheap = False
                if NTracer.ok(near):
                    self.columns[float(near.speed)] = near
                    return near, out
                if self.s8 and fabs(out-col.speed) > self.vtol*(1.+1e-9):
                    # S8 (item 1): the bracket is still wider than the speed tolerance: the same column with the S6
                    # rescue steps (only here, so charts whose edge closes without them never pay for it)
                    near = self.column(v, [col]+seeds, 'edge', False, pmin(v, out), pmax(v, col.speed))
                    self.eng.stats['s8_edge_rescues'] = self.eng.stats.get('s8_edge_rescues', 0)+1
                    if NTracer.ok(near):
                        self.columns[float(near.speed)] = near
                        return near, out
                    if self.eng.s9:
                        # S9: the column at the last admitted speed does not verify, even with the rescues (PBM-3 SB
                        # 121.9 km/h): columns halfway between it and the verified one, while that is still farther
                        # than the speed tolerance from the edge's failing probe
                        lo_v = v
                        while fabs(out-col.speed) > self.vtol*(1.+1e-9) and fabs(col.speed-lo_v) > .25*self.vtol:
                            v = .5*(col.speed+lo_v)
                            self.eng.stats['s9_edge_halves'] = self.eng.stats.get('s9_edge_halves', 0)+1
                            self.cheap = True
                            try:near = self.column(v, [col]+seeds, 'edge', False, pmin(v, out), pmax(v, col.speed))
                            finally:self.cheap = False
                            if NTracer.ok(near):
                                self.columns[float(near.speed)] = near
                                col = near
                            else:lo_v = v
                return col, out
            self.eng.stats['s7_edge_column_misses'] = self.eng.stats.get('s7_edge_column_misses', 0)+1
        if self.s8:
            # S8 (item 1): no rescue-free column next to the edge verifies: columns with the S6 rescue steps at the
            # admitted speeds nearest the edge, until one verifies (the S6 bisection to the placement tolerance only
            # where none does)
            for k in range(len(admitted)-1, max(-1, len(admitted)-4), -1):
                v = admitted[k]
                col = self.column(v, seeds, 'edge', False, pmin(v, out), pmax(v, good.speed))
                self.eng.stats['s8_edge_rescues'] = self.eng.stats.get('s8_edge_rescues', 0)+1
                if NTracer.ok(col):
                    self.columns[float(col.speed)] = col
                    return col, out
        self.eng.stats['s7_edge_probe_misses'] = self.eng.stats.get('s7_edge_probe_misses', 0)+1
        g, b = self.localize(good, bad, 'edge')
        return g, b.speed

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef object gap_probe(self, NColumn upper, double hi):
        """S8: level flight continued from the lowest verified column of the segment above down through a stretch where
        the selected wing sweep is unavailable (the selected sweep's aerodynamics are the same there; 'sweep unavailable'
        is the only extra reason, so such states carry the continuation) to the segment below, whose top is ``hi``, in
        steps of the placement tolerance. ('infeasible', speed) where level flight is physically rejected above that
        segment (no level flight below a low-speed edge, the tracer's edge rule), ('seed', state) with the admitted level
        flight at its top, or None (a correction that does not close: no evidence)."""
        from em_trim_work import Work, use, current
        cdef double v, h, step, last
        cdef int level = upper.level, p
        cdef PredMap preds
        cdef unsigned long long soft = (<unsigned long long>1) << R_SWEEP
        preds.n = 0
        if level < 0 or not self.eng.st[level].valid or fabs(self.eng.st[level].load-1.) > 1e-9:return None
        v = upper.speed;step = self.place_tol
        w = Work({}, parent=current())
        try:
            with use(w):
                while v > hi+1e-9:
                    last = v
                    h = pmin(step, v-hi);v = v-h
                    if v-hi < .25*self.vtol:v = hi
                    p = self.eng.correct(level, self.eng.st[level].sol, 1., True, FIX_LOAD, 1., 0, 0, 0, MARCH_NFEV, MARCH_EVALS,
                                         2e-6, v, &preds, NULL)
                    if p < 0 or not (self.eng.st[p].valid or self.eng.only_reasons(p, soft)):
                        p = self.eng.correct(level, self.eng.st[level].sol, 1., True, FIX_LOAD, 1., 0, 0, 0, ROBUST_NFEV,
                                             ROBUST_EVALS, 2e-6, v, NULL, NULL)
                    self.eng.flush_counts()
                    if p >= 0 and (self.eng.st[p].valid or self.eng.only_reasons(p, soft)):
                        level = p;continue
                    if step > self.vtol*(1.+1e-9) and not (p >= 0 and self.eng.physical_except(p, soft)):
                        # as the edge probes: a correction that does not close over a long step is no evidence;
                        # on from the last admitted level flight in steps of the speed tolerance
                        v = last;step = self.vtol;continue
                    self.events.append(dict(kind='sweep gap probe', speed_kmh=v, from_kmh=upper.speed,
                                            reasons=self.eng.reason_list(p) if p >= 0 else 'correction unresolved'))
                    # the low-speed edge of level flight (the edge probes' rule: a failure at the speed tolerance's
                    # step, physical or not, is the edge; the tracer never charts past an edge)
                    return ('infeasible', v) if v > hi+1e-9 else None
        finally:
            row = self.work.setdefault('sweep gap probe', dict(columns=0, evaluations=0))
            row['columns'] += 1;row['evaluations'] += w.counts.get('aircraft_evaluations', 0)
        return ('seed', level) if self.eng.st[level].valid else None

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int dive_path(self, list segments) except -1:
        """S8 (item 4): where two stable 1 g states exist at one speed (either side of a blade-polar jump), the chart
        takes the one reached by speeding up at 1 g, as in a dive. Ascending through each segment, a verified march
        column whose level flight has a blade-polar jump within the jump check's width gets level flight continued
        from its lower verified neighbour's; where that state differs (SEP by more than 0.01 m/s), the column is traced
        again from level flight continued from below and replaces the first ('dive path' event)."""
        cdef NColumn c, low, col
        cdef SlideSpec spec
        cdef int p, k
        if not self.eng.is_prop:return 0
        for seg in segments:
            lo, hi = seg['speed_kmh']
            cols = sorted([c for c in self.columns.values() if NTracer.ok(c) and c.march and c.level >= 0
                           and lo-1e-9 <= c.speed <= hi+1e-9], key=lambda c: c.speed)
            for k in range(1, len(cols)):
                c = cols[k];low = cols[k-1]
                self.eng.slide_candidate(c.level, &spec, NAN, NAN, NAN, False)
                if spec.n == 0:continue
                self.eng.stats['s8_dive_checks'] = self.eng.stats.get('s8_dive_checks', 0)+1
                if self.eng.s9 and self.eng.st[low.level].valid:
                    # S9: level flight continued from below in steps of at most the placement tolerance (one correction
                    # over a whole column interval can land on the other branch, P-47D-16 RE RB 785 -> 810 km/h)
                    p = self.level_walk(low.level, c.speed)
                else:
                    p = self.eng.correct(low.level, self.eng.st[low.level].sol, 1., True, FIX_LOAD, 1., 0, 0, 0, 12, 40, 2e-6,
                                         c.speed, NULL, NULL)
                if p < 0 or not self.eng.st[p].valid or fabs(self.eng.st[p].ps-self.eng.st[c.level].ps) <= .01:continue
                self.eng.dive_seed = True
                if self.eng.s9:self.forced_level = p     # S9: the column marches from the walked level flight itself
                try:
                    col = self.column(c.speed, [low], 'dive path', False, low.speed, hi+1e-9)
                finally:
                    self.eng.dive_seed = False;self.forced_level = -1
                if not (NTracer.ok(col) and col.speed == c.speed and col.level >= 0
                        and fabs(self.eng.st[col.level].ps-self.eng.st[p].ps) <= .01):continue
                self.events.append(dict(kind='dive path', speed_kmh=c.speed, level_ps_mps=[self.eng.st[c.level].ps,
                                                                                         self.eng.st[col.level].ps]))
                self.eng.stats['s8_dive_paths'] = self.eng.stats.get('s8_dive_paths', 0)+1
                self.columns[float(c.speed)] = col;cols[k] = col
        return 0

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int upper_segment(self, dict segment) except -1:
        """S8: a segment wholly past the zero-alpha speed of a forward-IAS limit: started at its low end from the
        nearest verified column below (across the seam) and swept; past the high-speed edge the trace found below it
        (or where that start is infeasible) it is beyond the edge (nothing to draw, no gap)."""
        cdef NColumn start, prev
        lo, hi = segment['speed_kmh']
        good = [c for c in self.columns.values() if NTracer.ok(c) and c.speed < lo]
        ended = any(e['kind'] == 'edge' and e['speed_kmh'][0] < e['speed_kmh'][1] and e['speed_kmh'][1] <= lo+1e-9
                    for e in self.events)
        if not good or ended:
            self.events.append(dict(kind='segment beyond edge', speed_kmh=segment['speed_kmh'], status='no feasible samples'))
            return 0
        prev = max(good, key=lambda c: c.speed)
        start = self.column(lo, [prev], 'start', True, lo-1e-9, hi+1e-9)
        if NTracer.ok(start):
            self.sweep(segment, start)
        elif start.status == STATUS_NOFEASIBLE:
            self.events.append(dict(kind='edge', speed_kmh=[prev.speed, start.speed], status=start.status_name(),
                                    reason=start.reason, located_by='segment start past the seam'))
            self.events.append(dict(kind='segment beyond edge', speed_kmh=segment['speed_kmh'], status='no feasible samples'))
        else:
            self.events.append(dict(kind='segment unresolved', speed_kmh=segment['speed_kmh'], status=start.status_name()))
        return 0

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int seam_restarts(self, list segments) except -1:
        """S8: a segment whose starts stayed unresolved (its trace began without a neighbouring level flight: the top
        segment, or one above another unresolved one) is started again at its end next to a traced segment, seeded by that
        segment's nearest verified column (level flight continued across the seam), then swept."""
        cdef NColumn start, nb
        cdef int k
        for k in range(len(self.events)-1, -1, -1):
            e = self.events[k]
            if e['kind'] != 'segment unresolved':continue
            lo, hi = e['speed_kmh']
            segment = next((s for s in segments if s['speed_kmh'][0] == lo and s['speed_kmh'][1] == hi), None)
            if segment is None:continue
            good = [c for c in self.columns.values() if NTracer.ok(c) and c.level >= 0 and self.eng.st[c.level].valid]
            below = max((c for c in good if c.speed < lo), key=lambda c: c.speed, default=None)
            above = min((c for c in good if c.speed > hi), key=lambda c: c.speed, default=None)
            for nb, v in ((below, lo), (above, hi)):
                if nb is None:continue
                start = self.column(v, [nb], 'start', True, lo-1e-9, hi+1e-9)
                self.eng.stats['s8_seam_restarts'] = self.eng.stats.get('s8_seam_restarts', 0)+1
                if NTracer.ok(start):
                    del self.events[k]
                    self.sweep(segment, start)
                    break
        return 0

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int ias_one_g(self) except -1:
        """S8: where level flight first passes the lowest forward-IAS limit: between neighbouring verified columns, the
        lower with its level flight inside the limit and the upper with it over (a lower edge), level flight continued
        from the lower in steps of the placement tolerance, then halved to 0.05 km/h ('ias edge' event; the leaning
        edge's foot, em_march_chart.ias_edge)."""
        from em_trim_work import Work, use, current
        cdef NColumn a, b
        cdef double lo, hi, v
        cdef int level, p
        cdef PredMap preds
        good = sorted((c for c in self.columns.values() if NTracer.ok(c)), key=lambda c: c.speed)
        for a, b in zip(good, good[1:]):
            if a.lower >= 0 or b.lower < 0 or a.level < 0 or not self.eng.st[a.level].valid:continue
            preds.n = 0
            level = a.level;lo = a.speed;hi = b.speed
            w = Work({}, parent=current())
            with use(w):
                while hi-lo > .05:
                    v = pmin(lo+self.place_tol, .5*(lo+hi)) if hi-lo > 2*self.place_tol else .5*(lo+hi)
                    p = self.eng.correct(level, self.eng.st[level].sol, 1., True, FIX_LOAD, 1., 0, 0, 0, MARCH_NFEV,
                                         MARCH_EVALS, 2e-6, v, &preds, NULL)
                    if p < 0 or not (self.eng.st[p].valid or self.eng.ias_soft(p)):
                        p = self.eng.correct(level, self.eng.st[level].sol, 1., True, FIX_LOAD, 1., 0, 0, 0, ROBUST_NFEV,
                                             ROBUST_EVALS, 2e-6, v, NULL, NULL)
                    self.eng.flush_counts()
                    if p >= 0 and self.eng.st[p].valid:
                        lo = v;level = p
                    elif p >= 0 and self.eng.ias_soft(p):hi = v
                    else:break
            row = self.work.setdefault('ias edge probe', dict(columns=0, evaluations=0))
            row['columns'] += 1;row['evaluations'] += w.counts.get('aircraft_evaluations', 0)
            self.events.append(dict(kind='ias edge', speed_kmh=[lo, hi], located_by='level-flight probes'))
        return 0

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef sweep(self, dict segment, NColumn start):
        cdef NColumn last, prev, col, good, bad
        cdef double speed, step, n, error, h, r, bound
        cdef int skipped
        cdef bint ended
        lo, hi = segment['speed_kmh']
        self.seg_starts.append((float(lo), float(hi), float(start.speed)))
        for side in (-1., 1.):
            bound = lo if side < 0 else hi
            chain = [start];step = self.max_step;skipped = 0;ended = False
            while abs(chain[-1].speed-bound) > 1e-9 and not ended:
                last = chain[-1];prev = chain[-2] if len(chain) > 1 else None
                speed = last.speed+side*min(step, abs(bound-last.speed))
                if self.s7 and side < 0 and isfinite(last.slope_l) and last.slope_l > 0.:
                    # S7: towards the low-speed edge the boundary falls to level flight (n = 1) where its face, continued
                    # with its exact slope, reaches it; a column far below that is an expensive infeasible one, so the
                    # sweep steps at most the placement tolerance past the predicted edge
                    n = self.eng.st[last.boundary].load
                    h = last.speed-(n-1.)/last.slope_l-self.place_tol
                    if speed < h < last.speed:speed = h
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
                    if col.status == STATUS_NOFEASIBLE and (last.lower >= 0 or (col.points and (self.eng.ias_soft(col.points[0])
                                                                                      or self.eng.ias_any(col.points[0])))):
                        # S8: past the leaning forward-IAS edge (no state inside the limit up to the first hard
                        # limit; or a level flight over the limit that also did not close): the top corner of that edge,
                        # by columns to the speed tolerance (their marches reach the states inside the limit)
                        good, bad = self.localize(last, col, 'edge', self.vtol)
                        self.events.append(dict(kind='edge', speed_kmh=[good.speed, bad.speed], status=bad.status_name(),
                                                reason=bad.reason, located_by='columns (forward-IAS edge)'))
                        break
                    # S6: an unresolved (not infeasible) column ends the sweep only to the speed tolerance
                    if self.s7 and col.status == STATUS_NOFEASIBLE:
                        # S7: an infeasible edge to the speed tolerance (it was drawn up to the placement tolerance inside)
                        good, edge = self.edge_probe(last, col)
                        if self.eng.s9:
                            # S9: two level-flight branches near an edge: the probe chain followed one that ends here;
                            # where the edge column's own level flight closes at the probe's first failing speed, the
                            # edge lies further on: the probes again from that column (at most three times)
                            for k in range(3):
                                if (good is last or good.level < 0 or not self.eng.st[good.level].valid
                                        or fabs(edge-good.speed) <= 1e-9 or fabs(col.speed-edge) <= 1e-9):break
                                preds_n = self.debug_probe(good.level, edge)
                                if preds_n < 0:break
                                self.eng.stats['s9_edge_reprobes'] = self.eng.stats.get('s9_edge_reprobes', 0)+1
                                good, edge = self.edge_probe(good, col)
                        self.events.append(dict(kind='edge', speed_kmh=[good.speed, edge], status=col.status_name(),
                                                reason=col.reason, located_by='level-flight probes'))
                        break
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
                    if (self.s7 and isfinite(last.slope_l) and isfinite(last.slope_r) and isfinite(prev.slope_l)
                            and isfinite(prev.slope_r) and self.face_of(last) == self.face_of(prev)):
                        # S7: the face continued with its exact slope and curvature (the outline's own face_load)
                        if side > 0:
                            n = face_load(last.speed, self.eng.st[last.boundary].load, last.slope_r,
                                          (last.slope_l-prev.slope_r)/(last.speed-prev.speed), speed)
                        else:
                            n = face_load(last.speed, self.eng.st[last.boundary].load, last.slope_l,
                                          (prev.slope_l-last.slope_r)/(prev.speed-last.speed), speed)
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
        if self.s8:return self.check_s8()
        if self.s7:return self.check_s7()
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
    cdef object face_of(self, NColumn c):
        """S7: the limiting face of a verified column: its boundary reason, with the control axis or the engine
        (an aileron and an elevator authority limit are two faces; em_march_chart.face_of)."""
        if c.boundary < 0 or self.eng.st[c.boundary].limit < 0:return c.reason
        limit = self.eng.limits[self.eng.st[c.boundary].limit]
        if limit.get('axis') is not None:return '%s:%s' % (c.reason, limit['axis'])
        engine = (limit.get('engine_rpm_limit') or {}).get('engine')
        if engine is not None:return '%s:%s' % (c.reason, engine)
        return c.reason

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef tuple interval_slopes(self, list good, int i):
        """S7: the interval good[i]..good[i+1] as em_march_chart.run_loads draws it: (sa, sb, ka, kb, pa, pb): the right
        slope of good[i] and the left slope of good[i+1] (the PCHIP slope through the neighbouring verified columns
        with no seam between where none is available), each face's curvature from its neighbouring column on the same
        face (0 without one), and the PCHIP slopes at both ends."""
        cdef NColumn a = good[i], b = good[i+1], c, p, q
        cdef double sa = a.slope_r, sb = b.slope_l, ka = 0., kb = 0., pa, pb, s0, s1
        cdef int lo, hi, j, n
        cdef double xs[8]
        cdef double ns[8]
        cdef double ds[8]
        seams = self.seams
        lo = i;hi = i+1
        while lo > 0 and i-lo < 2 and not any((<NColumn>good[lo-1]).speed < x < (<NColumn>good[lo]).speed or
                                               (<NColumn>good[lo-1]).speed < y < (<NColumn>good[lo]).speed for x, y in seams):lo -= 1
        while hi < len(good)-1 and hi-(i+1) < 2 and not any((<NColumn>good[hi]).speed < x < (<NColumn>good[hi+1]).speed or
                                                           (<NColumn>good[hi]).speed < y < (<NColumn>good[hi+1]).speed for x, y in seams):hi += 1
        n = hi-lo+1
        for j in range(n):
            c = good[lo+j];xs[j] = c.speed;ns[j] = self.eng.st[c.boundary].load
        pchip_slopes(n, xs, ns, ds)
        pa = ds[i-lo];pb = ds[i+1-lo]
        # a missing exact slope: the same face's slope at its neighbouring column (a face's slope changes little over a
        # column interval; a PCHIP slope through another face's column says nothing about it), else PCHIP
        if not isfinite(sa) and lo <= i-1:
            p = good[i-1]
            if self.face_of(p) == self.face_of(a) and isfinite(p.slope_r):sa = p.slope_r
        if not isfinite(sb) and hi >= i+2:
            q = good[i+2]
            if self.face_of(q) == self.face_of(b) and isfinite(q.slope_l):sb = q.slope_l
        self.exact_a = isfinite(sa);self.exact_b = isfinite(sb)
        if not isfinite(sa):sa = pa
        if not isfinite(sb):sb = pb
        if lo <= i-1:
            p = good[i-1]
            if self.face_of(p) == self.face_of(a):
                s0 = p.slope_r if isfinite(p.slope_r) else ds[i-1-lo]
                s1 = a.slope_l if isfinite(a.slope_l) else pa
                ka = (s1-s0)/(a.speed-p.speed)
        if hi >= i+2:
            q = good[i+2]
            if self.face_of(q) == self.face_of(b):
                s0 = b.slope_r if isfinite(b.slope_r) else pb
                s1 = q.slope_l if isfinite(q.slope_l) else ds[i+2-lo]
                kb = (s1-s0)/(q.speed-b.speed)
        return sa, sb, ka, kb, pa, pb

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef bint stretch_check(self, list good, NColumn col, NColumn other, list checks) except -1:
        """S7: whether a passing corner column ``col`` also validates the other face, drawn from its column ``other`` up
        to the corner. A stretch within twice the speed tolerance, or whose drawn turn rate changes by at most twice the
        turn tolerance, is validated by the corner column; a steeper one (an engine RPM limit near the speed limit can
        stand almost vertical and curve away from its tangent within a few km/h) gets a withheld column at its midpoint,
        which passes when it lies on that face within half the turn tolerance of the outline drawn with ``col`` (the
        chart draws the corner column as a witness). A passing one is a witness too; a failing one joins the columns, and
        the caller splits the interval at ``col``."""
        cdef NColumn mid, a, b
        cdef double va, vb, vm, x, t, lo_t, hi_t, sa, sb, ka, kb, pa, pb, na, nb
        cdef bint ok
        cdef int j
        if fabs(other.speed-col.speed) <= 2*self.vtol:return True
        if fabs(self.eng.st[other.boundary].turn-self.eng.st[col.boundary].turn) <= 2*self.ttol:return True
        a, b = (col, other) if col.speed < other.speed else (other, col)
        va, vb = a.speed, b.speed;vm = .5*(va+vb)
        mid = self.column(vm, [col, other], 'corner check', False, va, vb)
        vm = mid.speed
        ok = NTracer.ok(mid) and self.face_of(mid) == self.face_of(other)
        if ok:
            drawn = sorted([c for c in good if c is not col]+[col], key=lambda c: c.speed)
            j = drawn.index(a)
            sa, sb, ka, kb, pa, pb = self.interval_slopes(drawn, j)
            na = self.eng.st[a.boundary].load;nb = self.eng.st[b.boundary].load
            lo_t = INFINITY;hi_t = -INFINITY
            for x in (vm-self.vtol, vm, vm+self.vtol):
                x = pmin_np(pmax_np(x, va), vb)
                t = self.M.turn_rate(x, float(interval_load(va, na, sa, ka, pa, vb, nb, sb, kb, pb, self.face_of(a) != self.face_of(b),
                                                             self.exact_a and self.exact_b, x)))
                lo_t = pmin(lo_t, t);hi_t = pmax(hi_t, t)
            ok = lo_t-S7_CHECK*self.ttol <= self.eng.st[mid.boundary].turn <= hi_t+S7_CHECK*self.ttol
        checks.append(dict(interval_kmh=[va, vb], speed_kmh=vm, passed=bool(ok), faces=[a.reason, b.reason],
                           midpoint_face=mid.reason if NTracer.ok(mid) else mid.status_name(), kind='corner stretch'))
        self.eng.stats['s7_stretch_checks'] = self.eng.stats.get('s7_stretch_checks', 0)+1
        if ok:self.witnesses.append(mid)
        else:
            self.eng.stats['s7_stretch_failed'] = self.eng.stats.get('s7_stretch_failed', 0)+1
            if NTracer.ok(mid):self.columns[vm] = mid
        return ok

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef double piece_error(self, double va, double na, double sa, double ka, double pa, double vb, double nb, double sb,
                            double kb, double pb, bint corner, bint exact, double* at) noexcept:
        """S8: the error bound of the outline piece [va, vb] (interval_load, as the chart draws it) from its end loads
        and slopes alone: at S8_SAMPLES points the largest turn-rate distance from the drawn line (allowing the chart's
        horizontal slack: the drawn line over x +- the speed tolerance) to the band the true boundary lies in
        (band_load, from the slopes the piece is drawn with: exact, or a missing one from the same face's neighbouring
        column or PCHIP on one face). ``at``: where it is largest."""
        cdef double x, xl, xr, lo, hi, w, wl, wr, e, best = 0., vc
        cdef int j
        at[0] = .5*(va+vb)
        # a face change is drawn as its two faces meeting; without both faces' exact slopes, or where they do not meet
        # (a jump), nothing bounds it (it is split down to the speed tolerance)
        if corner and not (exact and corner_vertex(va, na, sa, ka, vb, nb, sb, kb, &vc)):return INFINITY
        # an end in level flight (n = 1, the low-speed edge) has no exact slope (the turn coordinate is singular there),
        # and turn rate there is singular in load: nothing bounds the piece (it is split down to the speed tolerance)
        if (not self.exact_a and fabs(na-1.) <= 1e-9) or (not self.exact_b and fabs(nb-1.) <= 1e-9):return INFINITY
        for j in range(1, S8_SAMPLES):
            x = va+(vb-va)*j/S8_SAMPLES
            xl = pmax(va, x-self.vtol);xr = pmin(vb, x+self.vtol)
            w = turn_rate(x, interval_load(va, na, sa, ka, pa, vb, nb, sb, kb, pb, corner, exact, x))
            wl = turn_rate(xl, interval_load(va, na, sa, ka, pa, vb, nb, sb, kb, pb, corner, exact, xl))
            wr = turn_rate(xr, interval_load(va, na, sa, ka, pa, vb, nb, sb, kb, pb, corner, exact, xr))
            band_load(va, na, sa, vb, nb, sb, x, &lo, &hi)
            e = pmax(turn_rate(x, hi)-pmax(w, pmax(wl, wr)), pmin(w, pmin(wl, wr))-turn_rate(x, lo))
            if not isfinite(e):e = INFINITY
            if e > best:
                best = e;at[0] = x
        return best

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int lower_split(self, list checks, double tol) except -1:
        """S8: the split rule on the leaning forward-IAS edge (em_march_chart.ias_edge). Along the edge the speed is a
        smooth function of w = n^2-1 (at level flight the edge leaves the 1 g line vertically: dV/dw = 0 there, while
        w(V) is singular), so the edge is drawn as the cubic Hermite V(w) through the columns' lower edges with their
        exact slopes (dV/dw = 1/lower_slope) and level flight at the located 'ias edge' speed with slope 0. Each piece is
        judged by its error bound from its end values and slopes (band_load on V(w)): it is finished when the band lies
        within S8_BOUND of the speed tolerance of the drawn edge (the chart's horizontal slack covers it) or the piece is
        narrower than the speed tolerance; otherwise a column is solved at the speed where the band is widest. Where
        the edge is not monotone in w, nothing here (the edge is drawn through the columns as recorded). Returns the
        columns added."""
        cdef NColumn c, col
        cdef int n, k, added = 0, j
        cdef double va, vb, wa, wb, e, x, w, vd, lo, hi, best, htol = S8_BOUND*self.vtol
        cdef double* ws
        cdef double* vs
        cdef double* dl
        cdef double* dr
        tried, passed_set = self.check_tried, self.check_passed
        lows = [c for c in sorted(self.columns.values(), key=lambda c: c.speed) if NTracer.ok(c) and c.lower >= 0]
        if not lows:return 0
        seg = next((g['speed_kmh'] for g in self.segments_ or () if g['speed_kmh'][0]-1e-9 <= (<NColumn>lows[0]).speed
                    <= g['speed_kmh'][1]+1e-9), None)
        if seg is not None:lows = [c for c in lows if seg[0]-1e-9 <= c.speed <= seg[1]+1e-9]
        pts = [(pow(self.eng.st[c.lower].load, 2.)-1., c.speed, c,
                1./c.lower_slope if isfinite(c.lower_slope) and c.lower_slope > 0. else NAN) for c in lows]
        foot = next((max(e['speed_kmh']) for e in self.events if e['kind'] == 'ias edge'
                     and min(e['speed_kmh']) <= pts[0][1]+1e-9), None)
        if foot is not None and foot < pts[0][1]-1e-9:pts.insert(0, (0., foot, None, 0.))
        n = len(pts)
        if self.eng.s14:
            # S14: an edge that is not monotone in w (where it meets the upper boundary, the lower edge's load can fall
            # again: J6K1 SB 798-802 km/h) is split on its monotone stretch from level flight; S8 split nothing then, so
            # the stretch from the foot to the first column (J6K1 SB 781.4-792.5 km/h, where the edge rises almost
            # vertically from 1 g to 6.5 g: level-flight body alpha is negative, pulling first raises forward IAS) was
            # drawn as recorded, up to 13 deg/s low
            k = 0
            while k+1 < n and pts[k+1][0] > pts[k][0]:k += 1
            if k+1 < n:
                pts = pts[:k+1];n = len(pts)
                self.eng.stats['s14_lower_prefix'] = self.eng.stats.get('s14_lower_prefix', 0)+1
        if n < 2 or any(pts[k+1][0] <= pts[k][0] for k in range(n-1)):return 0
        ws = <double*>malloc(n*sizeof(double));vs = <double*>malloc(n*sizeof(double))
        dl = <double*>malloc(n*sizeof(double));dr = <double*>malloc(n*sizeof(double))
        if ws == NULL or vs == NULL or dl == NULL or dr == NULL:
            if ws != NULL:cfree(ws)
            if vs != NULL:cfree(vs)
            if dl != NULL:cfree(dl)
            if dr != NULL:cfree(dr)
            raise MemoryError()
        try:
            for k in range(n):
                ws[k] = pts[k][0];vs[k] = pts[k][1];dl[k] = pts[k][3];dr[k] = pts[k][3]
            fill_slopes(n, ws, vs, dl, dr)
            for k in range(n-1):
                wa, wb = ws[k], ws[k+1];va, vb = vs[k], vs[k+1]
                key = ('lower edge', va, vb)
                if key in passed_set or key in tried:continue
                best = 0.;x = .5*(va+vb)
                for j in range(1, S8_SAMPLES):
                    w = wa+(wb-wa)*j/S8_SAMPLES
                    vd = hermite_eval(n, ws, vs, dl, dr, w)
                    band_load(wa, va, dr[k], wb, vb, dl[k+1], w, &lo, &hi)
                    e = pmax(hi-vd, vd-lo)
                    if not isfinite(e):e = INFINITY
                    if e > best:
                        best = e;x = vd
                e = best
                if vb-va <= self.vtol+1e-9 or (e <= htol and vb-va <= self.max_step+1e-9):
                    passed_set.add(key)
                    checks.append(dict(interval_kmh=[va, vb], passed=True, kind='lower edge bound',
                                       bound_kmh=e if isfinite(e) else None))
                    continue
                if e <= htol:x = .5*(va+vb)
                x = pmin(pmax(x, va+.1*(vb-va)), vb-.1*(vb-va))
                near = [q[2] for q in (pts[k], pts[k+1]) if q[2] is not None]
                col = self.column(x, near, 'split', False, va, vb)
                checks.append(dict(interval_kmh=[va, vb], speed_kmh=col.speed, passed=False, kind='lower edge split',
                                   bound_kmh=e if isfinite(e) else None))
                self.columns[col.speed] = col;added += 1
                if not NTracer.ok(col) or col.lower < 0:tried.add(key)
        finally:
            cfree(ws);cfree(vs);cfree(dl);cfree(dr)
        return added

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef bint face_event(self, NColumn col, Event* out) except -1:
        """S9: the native event of a verified column's limiting face (column_slope's choice: stall, wing force, control
        with its axis, Instructor, engine RPM with its engine), or False."""
        cdef int b = col.boundary, base, i, nev, kind, laxis = -1, engine
        cdef Event ev[32]
        if b < 0 or self.eng.st[b].limit < 0 or col.status != STATUS_VERIFIED:return False
        base = self.eng.st[b].base
        limit = self.eng.limits[self.eng.st[b].limit]
        kind = KIND_CODE.get(limit.get('kind'), -1)
        if kind not in (KD_STALL, KD_WING, KD_CONTROL, KD_INSTR, KD_RPM, KD_BRAKE):return False
        if limit.get('axis') is not None:laxis = int(limit['axis'])
        engine = int((limit.get('engine_rpm_limit') or {}).get('engine', -1))
        nev = self.eng.nearest_events(base, ev)
        for i in range(nev):
            if ev[i].kind != kind or ev[i].code <= 0:continue
            if kind == KD_CONTROL and laxis >= 0 and ev[i].axis != laxis:continue
            if kind == KD_RPM and engine >= 0 and ev[i].engine != engine:continue
            out[0] = ev[i]
            return True
        return False

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef bint knot_check(self, list good, int i, double k, double sa, double sb, double ka, double kb, double pa, double pb,
                         bint corner, bint exact, list checks) except -1:
        """S9: whether the outline piece good[i]..good[i+1], drawn as interval_load, holds at the model breakpoint ``k``
        inside it: the limiting face of the nearer column corrected at ``k`` (its own event equations from its boundary
        state) is admitted and lies within S8_BOUND of the turn tolerance of the drawn line there (allowing the chart's
        horizontal slack). A face change, or a face without a native event (pitch response, load maximum), has no such
        check: the piece stands on S8's rule there (splitting such pieces at every knot doubled their cost)."""
        cdef NColumn a = good[i], b = good[i+1], c
        cdef Event ev
        cdef int base, p
        cdef double x, t, lo_t = INFINITY, hi_t = -INFINITY, box, na, nb
        if corner:return True
        c = a if k-a.speed <= b.speed-k else b
        if not self.face_event(c, &ev):return True
        base = self.eng.st[c.boundary].base
        p = self.eng.correct(base, self.eng.st[base].sol, self.eng.st[base].load, True, FIX_EVENT, 0., ev.code,
                             ev.engine if ev.code == 6 else (ev.axis-2 if ev.axis >= 0 else 0), ev.side, 16, 0, 2e-6, k, NULL, NULL)
        self.eng.flush_counts()
        self.eng.stats['s9_knot_checks'] = self.eng.stats.get('s9_knot_checks', 0)+1
        if p < 0 or not self.eng.st[p].valid:return False
        na = self.eng.st[a.boundary].load;nb = self.eng.st[b.boundary].load
        for x in (k-self.vtol, k, k+self.vtol):
            x = pmin(pmax(x, a.speed), b.speed)
            t = turn_rate(x, interval_load(a.speed, na, sa, ka, pa, b.speed, nb, sb, kb, pb, corner, exact, x))
            lo_t = pmin(lo_t, t);hi_t = pmax(hi_t, t)
        t = self.eng.st[p].turn
        box = 0. if lo_t <= t <= hi_t else (t-hi_t if t > hi_t else lo_t-t)
        checks.append(dict(interval_kmh=[a.speed, b.speed], speed_kmh=k, passed=bool(box <= S8_BOUND*self.ttol), kind='knot',
                           error_dps=box, faces=[a.reason, b.reason]))
        if box > S8_BOUND*self.ttol:
            self.eng.stats['s9_knot_splits'] = self.eng.stats.get('s9_knot_splits', 0)+1
            return False
        return True

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef bint wake_check(self, list good, int i, int side, double sa, double sb, double ka, double kb, double pa, double pb,
                         bint exact, list checks, double* at) except -1:
        """S12: whether the outline piece good[i]..good[i+1] holds where its face crosses the tail-wake switch ``side``
        (MarchEngine.wake_margins; its end states lie on either side): the crossing is located on the face itself (the
        nearer column's limiting face corrected at the speed where the switch quantity, interpolated between the bracketing
        states, is zero: Illinois steps, at most S12_WAKE_STEPS corrections, until the bracket is narrower than the speed
        tolerance), and there the face must lie within S8_BOUND of the turn tolerance of the drawn line (allowing the
        chart's horizontal slack). ``at``: the crossing. A face without a native event has no such check (True)."""
        cdef NColumn a = good[i], b = good[i+1], c
        cdef Event ev
        cdef int seed, p = -1, q, it, kept = 0
        cdef double lv = a.speed, lm, hv = b.speed, hm, k = NAN, m, x, t, lo_t = INFINITY, hi_t = -INFINITY, box, na, nb
        lm = self.eng.st[a.boundary].wake[side];hm = self.eng.st[b.boundary].wake[side]
        at[0] = lv+lm/(lm-hm)*(hv-lv)
        for it in range(S12_WAKE_STEPS):
            k = lv+lm/(lm-hm)*(hv-lv)
            if p < 0:
                c = a if k-a.speed <= b.speed-k else b
                if not self.face_event(c, &ev):return True
                seed = self.eng.st[c.boundary].base
            else:seed = p
            q = self.eng.correct(seed, self.eng.st[seed].sol, self.eng.st[seed].load, True, FIX_EVENT, 0., ev.code,
                                 ev.engine if ev.code == 6 else (ev.axis-2 if ev.axis >= 0 else 0), ev.side, 16, 0, 2e-6, k,
                                 NULL, NULL)
            self.eng.flush_counts()
            self.eng.stats['s12_wake_corrections'] = self.eng.stats.get('s12_wake_corrections', 0)+1
            if q < 0 or not self.eng.st[q].valid:break
            p = q;at[0] = k
            m = self.eng.st[p].wake[side]
            if not isfinite(m) or hv-lv <= self.vtol:break
            # Illinois: the bracket end kept twice has its value halved
            if (m >= 0.) == (lm >= 0.):
                lv = k;lm = m
                if kept == 1:hm *= .5
                kept = 1
            else:
                hv = k;hm = m
                if kept == -1:lm *= .5
                kept = -1
            if hv-lv <= self.vtol:break
        if p < 0:return False
        k = self.eng.st[p].speed
        na = self.eng.st[a.boundary].load;nb = self.eng.st[b.boundary].load
        for x in (k-self.vtol, k, k+self.vtol):
            x = pmin(pmax(x, a.speed), b.speed)
            t = turn_rate(x, interval_load(a.speed, na, sa, ka, pa, b.speed, nb, sb, kb, pb, False, exact, x))
            lo_t = pmin(lo_t, t);hi_t = pmax(hi_t, t)
        t = self.eng.st[p].turn
        box = 0. if lo_t <= t <= hi_t else (t-hi_t if t > hi_t else lo_t-t)
        checks.append(dict(interval_kmh=[a.speed, b.speed], speed_kmh=k, passed=bool(box <= S8_BOUND*self.ttol), kind='tail wake',
                           error_dps=box, faces=[a.reason, b.reason]))
        return box <= S8_BOUND*self.ttol

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int tie_kind(self, list good, int j, int o) except -2:
        """S14: a hard limit other than column good[j]'s own face whose margin at its boundary is below S14_TIE while at
        the other end good[o] it is larger (more than twice it plus half S14_TIE): the limit that can take over inside
        the piece; -1 for none, or where the verified column beyond good[j] (outside the piece) is limited by it."""
        cdef NColumn c = good[j], d = good[o]
        cdef TState* s = &self.eng.st[c.boundary]
        cdef TState* t = &self.eng.st[d.boundary]
        cdef int k, own = KIND_CODE.get(c.reason, -1), nb = j-1 if o > j else j+1
        for k in (KD_CONTROL, KD_STALL, KD_WING, KD_INSTR):
            if k == own or not (s.mset >> k) & 1 or not (t.mset >> k) & 1:continue
            if not (s.m[k] < S14_TIE and t.m[k] > 2.*s.m[k]+.5*S14_TIE):continue
            if 0 <= nb < len(good) and KIND_CODE.get((<NColumn>good[nb]).reason, -1) == k:continue
            return k
        return -1

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef list check_s8(self):
        """S8: the split rule on every outline piece between verified columns of one segment (no Mach seam or segment
        end inside). A piece whose error bound (piece_error) is within S8_BOUND of the envelope turn tolerance and which
        is at most the maximum step wide is finished; otherwise a column is solved where the bound is largest (at the
        middle of a piece that is only too wide), at least a tenth of the piece from its ends, and both halves are
        judged again. A piece narrower than the speed tolerance is finished; if its bound still fails and each end's
        tangent misses the other end by more than the turn tolerance, the boundary jumps there (a 'step' event: the
        chart draws it vertical). A column that is not verified ends the splitting of its piece (the S6 gap rules take
        it from there)."""
        cdef NColumn a, b, col
        cdef double va, vb, x, sa, sb, ka, kb, pa, pb, na, nb, e, vc, tc
        cdef bint corner, exact
        cdef int i
        cdef long splits = 0, steps = 0
        cdef double tol = S8_BOUND*self.ttol
        cdef double width = pmin(self.max_step, S8_MAX_WIDTH)
        checks, tried, passed_set = [], self.check_tried, self.check_passed
        seams = self.seams

        def seam(lo, hi):
            return any(lo < x < hi or lo < y < hi for x, y in seams)
        for depth in range(self.max_depth+8):
            good = [c for c in sorted(self.columns.values(), key=lambda c: c.speed) if NTracer.ok(c)]
            added = 0
            for i in range(len(good)-1):
                a = good[i];b = good[i+1]
                va, vb = a.speed, b.speed
                key = (va, vb)
                if key in passed_set or key in tried or seam(va, vb):continue
                sa, sb, ka, kb, pa, pb = self.interval_slopes(good, i)
                na = self.eng.st[a.boundary].load;nb = self.eng.st[b.boundary].load
                corner = self.face_of(a) != self.face_of(b);exact = self.exact_a and self.exact_b
                e = self.piece_error(va, na, sa, ka, pa, vb, nb, sb, kb, pb, corner, exact, &x)
                bound = e if isfinite(e) else None
                knot_split = False
                if self.eng.s9 and vb-va > self.vtol+1e-9 and e <= tol and vb-va <= width+1e-9:
                    # S9: the bound assumes the boundary's slope changes monotonically inside the piece; where a polar's
                    # Mach curve bends back between its breakpoints it can rise and fall inside one piece (A-6E RB,
                    # 1180-1200 km/h): the face at those bends from the nearer column (its event correction) must lie on
                    # the drawn line, else the piece is split there
                    for kx in self.s9_knots:
                        if not (va+self.vtol < kx < vb-self.vtol):continue
                        if self.knot_check(good, i, kx, sa, sb, ka, kb, pa, pb, corner, exact, checks):continue
                        e = INFINITY;x = kx;knot_split = True
                        break
                    if not knot_split and self.ias_knots:
                        # the control authority kinks where the boundary's forward IAS passes an effective speed (or the
                        # authority floor) of a control axis (A-6E RB 1188 km/h): checked where the end states' forward
                        # IAS brackets it, interpolated in speed
                        ua = self.eng.st[a.boundary].speed/3.6*cos(self.eng.st[a.boundary].alpha*DEG_TO_RAD)*self.sigma
                        ub = self.eng.st[b.boundary].speed/3.6*cos(self.eng.st[b.boundary].alpha*DEG_TO_RAD)*self.sigma
                        for kn in self.ias_knots:
                            if not (ua-kn)*(ub-kn) < 0.:continue
                            kx = va+(kn-ua)/(ub-ua)*(vb-va)
                            if not (va+self.vtol < kx < vb-self.vtol):continue
                            if self.knot_check(good, i, kx, sa, sb, ka, kb, pa, pb, corner, exact, checks):continue
                            e = INFINITY;x = kx;knot_split = True
                            break
                if (self.eng.s12 and not corner and not knot_split and e <= tol and vb-va > self.vtol+1e-9
                        and vb-va <= width+1e-9):
                    # S12: the bound assumes the face's slope changes monotonically inside the piece. The game's wing-wake
                    # term of the tail's downwash (type-2 downwash) has a cusp where a tail half passes through the wake
                    # sheet (its attenuation exp(-1.41 |lateral offset|)) and a kink where the wake's travel time reaches its
                    # cap, so the faces kink there; against the face's curvature the slope then overshoots inside the piece
                    # (MB.151 C.1 RB 525 km/h: the control face's slope from -0.026 to -0.097 g per km/h there, -0.080 at the
                    # piece's end 530 km/h; drawn 0.46 deg/s off). Where the end states lie on either side of such a switch,
                    # the face at the crossing (interpolated in speed) must lie on the drawn line, else the piece is split
                    # there (both one-sided slopes at that column)
                    done = []
                    for side in range(4):
                        ma = self.eng.st[a.boundary].wake[side];mb = self.eng.st[b.boundary].wake[side]
                        if not (isfinite(ma) and isfinite(mb)) or (ma >= 0.) == (mb >= 0.):continue
                        kx = va+ma/(ma-mb)*(vb-va)
                        # (both tail halves cross the wake sheet within the speed tolerance of each other: one check)
                        if any(fabs(kx-v) <= self.vtol for v in done):continue
                        self.eng.stats['s12_wake_checks'] = self.eng.stats.get('s12_wake_checks', 0)+1
                        if self.wake_check(good, i, side, sa, sb, ka, kb, pa, pb, exact, checks, &vc):
                            done.append(vc);continue
                        if not (va+.5*self.vtol < vc < vb-.5*self.vtol):continue
                        e = INFINITY;x = vc;knot_split = True
                        self.knot_speeds.append(vc)
                        self.eng.stats['s12_wake_splits'] = self.eng.stats.get('s12_wake_splits', 0)+1
                        break
                if self.eng.s14 and not corner and not knot_split and e <= tol and vb-va > 2.*self.vtol+1e-9:
                    # S14: another hard limit nearly active at one end's boundary (its margin below S14_TIE there, larger
                    # at the other end) can become the first limit inside the piece, which neither end sees (I-16 type 5
                    # RB 120.5-124.5 km/h: the elevator saturates before the Instructor pitch limit, drawn 1.5 deg/s
                    # high): a column a quarter of the piece from that end (a tie explained by the neighbouring corner
                    # beyond that end is not one)
                    for j, o in ((i, i+1), (i+1, i)):
                        if self.tie_kind(good, j, o) < 0:continue
                        x = va+.25*(vb-va) if j == i else vb-.25*(vb-va)
                        e = INFINITY;knot_split = True;bound = None
                        self.eng.stats['s14_tie_splits'] = self.eng.stats.get('s14_tie_splits', 0)+1
                        break
                if (self.eng.s10 and corner and exact and not knot_split and e <= tol and vb-va > self.vtol+1e-9
                        and corner_vertex(va, na, sa, ka, vb, nb, sb, kb, &vc)):
                    # S10: the bound takes both faces as continued from their columns up to the corner. A long steep stretch
                    # of one (longer than the speed tolerance, the chart's horizontal slack, its turn rate changing by more
                    # than twice the turn tolerance) can hide a jump the columns do not see (D.520 RB 608-610.6 km/h: the
                    # control face continued 2.3 km/h and 9 deg/s up to the Instructor pitch face, which in fact holds to
                    # 610.5 and drops there): a column at the middle of that stretch
                    vc = pmin(pmax(vc, va), vb)
                    tc = turn_rate(vc, interval_load(va, na, sa, ka, pa, vb, nb, sb, kb, pb, corner, exact, vc))
                    if vc-va > self.vtol and fabs(tc-turn_rate(va, na)) > 2.*self.ttol:
                        e = INFINITY;x = .5*(va+vc)
                    elif vb-vc > self.vtol and fabs(tc-turn_rate(vb, nb)) > 2.*self.ttol:
                        e = INFINITY;x = .5*(vc+vb)
                    if not isfinite(e):
                        bound = None
                        self.eng.stats['s10_corner_stretch'] = self.eng.stats.get('s10_corner_stretch', 0)+1
                if self.eng.s14 and not knot_split and e > tol and vb-va > self.vtol+1e-9:
                    # S14: a failing piece with a model breakpoint inside: the failure can be a jump there (P-61C-1 SB
                    # 675.6 km/h: the wing-force limit steps from 7.49 to 7.19 g), which the split at the largest bound
                    # (at least a tenth of the piece from its ends) only creeps towards (14 columns from 662 km/h, the
                    # last piece 672.8-676.0 left smooth, 0.25 deg/s off): columns a quarter of the speed tolerance
                    # either side of the breakpoint
                    for kx in self.knot_speeds:
                        if not (va+.05*self.vtol < kx < vb-.05*self.vtol):continue
                        if kx-.25*self.vtol > va+.05*self.vtol:x = kx-.25*self.vtol
                        elif kx+.25*self.vtol < vb-.05*self.vtol:x = kx+.25*self.vtol
                        else:continue
                        knot_split = True
                        self.eng.stats['s14_knot_splits'] = self.eng.stats.get('s14_knot_splits', 0)+1
                        break
                if vb-va <= self.vtol+1e-9 or (e <= tol and vb-va <= width+1e-9):
                    passed_set.add(key)
                    kind = 'bound' if e <= tol else 'narrow'
                    # narrower than the speed tolerance and still failing: where each end's tangent misses the other end
                    # by more than the turn tolerance, the difference did not shrink with the piece: a discontinuity
                    if e > tol and na > 1.+1e-9 and nb > 1.+1e-9 and pmin(fabs(turn_rate(vb, nb)-turn_rate(vb, na+sa*(vb-va))),
                                                                          fabs(turn_rate(va, na)-turn_rate(va, nb-sb*(vb-va)))) > self.ttol:
                        kind = 'step';steps += 1
                        self.events.append(dict(kind='step', speed_kmh=[va, vb], faces=[a.reason, b.reason], bound_dps=bound))
                    checks.append(dict(interval_kmh=[va, vb], passed=True, kind=kind, bound_dps=bound, faces=[a.reason, b.reason]))
                    if corner:self.events.append(dict(kind='corner', speed_kmh=[va, vb], faces=[a.reason, b.reason]))
                    continue
                if e <= tol:x = .5*(va+vb)
                if not knot_split:x = pmin(pmax(x, va+.1*(vb-va)), vb-.1*(vb-va))
                col = self.column(x, [a, b], 'split', False, va, vb)
                splits += 1
                checks.append(dict(interval_kmh=[va, vb], speed_kmh=col.speed, passed=False, kind='split', bound_dps=bound,
                                   faces=[a.reason, b.reason], split_face=col.reason if NTracer.ok(col) else col.status_name()))
                added += 1
                self.columns[col.speed] = col
                if not NTracer.ok(col):tried.add(key)
            n = self.lower_split(checks, tol)
            added += n;splits += n
            if not added:break
        self.eng.stats['s8_splits'] = self.eng.stats.get('s8_splits', 0)+splits
        self.eng.stats['s8_steps'] = self.eng.stats.get('s8_steps', 0)+steps
        return checks

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef list check_s7(self):
        """S7: MarchTracer.check on the S7 outline. Every interval between verified columns (wider than twice the speed
        tolerance, no Mach seam inside) gets a withheld column: at the predicted corner where its ends lie on different
        limiting faces (corner_vertex), else at its midpoint. The column passes when its turn rate lies within half the
        turn tolerance of the outline over the speed tolerance around it (interval_load, as the chart draws it) and, at a
        midpoint, its own exact slope agrees with the drawn line's there; at a corner it passes only if the face it is
        not on is drawn from its column over at most twice the placement tolerance, and that stretch passes its own
        withheld column where it is steep (stretch_check); a face change without both faces' slopes is halved down to the
        horizontal tolerance. A passing column is drawn as a witness, a failing one splits
        the interval, an unresolved one is kept as before. Narrower intervals across a jump of the boundary are halved
        down to a quarter of the speed tolerance."""
        cdef NColumn a, b, col
        cdef double va, vb, v, x, sa, sb, ka, kb, pa, pb, na, nb, vc, lo_t, hi_t, t
        cdef bint corner, ok, has_vertex
        cdef int i
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
                if key in passed_set or key in tried or seam(va, vb):continue
                sa, sb, ka, kb, pa, pb = self.interval_slopes(good, i)
                na = self.eng.st[a.boundary].load;nb = self.eng.st[b.boundary].load
                if vb-va <= 2*self.vtol:
                    # within the horizontal tolerance no withheld check exists; a jump of the boundary (the load at b
                    # more than the turn tolerance from a's continued with the slopes) is bracketed to a quarter of it
                    t = .5*(sa+sb)*(vb-va)
                    if not (vb-va > .25*self.vtol and isfinite(t) and fabs(self.M.turn_rate(vb, nb)-self.M.turn_rate(vb, na+t)) > self.ttol):
                        continue
                    col = self.column(.5*(va+vb), [a, b], 'jump', False, va, vb)
                    checks.append(dict(interval_kmh=[va, vb], speed_kmh=col.speed, passed=False, faces=[a.reason, b.reason],
                                       midpoint_face=col.reason if NTracer.ok(col) else col.status_name(), kind='jump'))
                    if NTracer.ok(col):
                        added += 1;self.columns[col.speed] = col
                    else:
                        tried.add(key)     # no column inside the horizontal tolerance that is not verified
                    continue
                corner = self.face_of(a) != self.face_of(b)
                # a corner needs both faces' exact slopes (a PCHIP slope through another face's column says nothing)
                has_vertex = (corner and self.exact_a and self.exact_b
                              and corner_vertex(va, na, sa, ka, vb, nb, sb, kb, &vc))
                if has_vertex:
                    # a corner at least a tenth of the interval from its ends (each failed check then shrinks it)
                    v = pmin(pmax(vc, va+.1*(vb-va)), vb-.1*(vb-va))
                else:
                    # the midpoint: a kink inside (model or not) fails the column's exact-slope test there
                    v = .5*(va+vb)
                col = self.column(v, [a, b], 'corner check' if corner else 'check', False, va, vb)
                v = col.speed
                ok = NTracer.ok(col)
                if ok:
                    lo_t = INFINITY;hi_t = -INFINITY
                    for x in (v-self.vtol, v, v+self.vtol):
                        x = pmin_np(pmax_np(x, va), vb)
                        t = self.M.turn_rate(x, float(interval_load(va, na, sa, ka, pa, vb, nb, sb, kb, pb, corner,
                                                                     self.exact_a and self.exact_b, x)))
                        lo_t = pmin(lo_t, t);hi_t = pmax(hi_t, t)
                    ok = lo_t-S7_CHECK*self.ttol <= self.eng.st[col.boundary].turn <= hi_t+S7_CHECK*self.ttol
                    if ok and corner:
                        # a face change: without both faces' slopes the interval is halved down to the horizontal
                        # tolerance; at a predicted corner the column passes only if the face it is not on is drawn
                        # from its column over at most twice the placement tolerance (a face continued further is not
                        # validated by this column); otherwise it splits the interval and the corner is predicted again
                        if not has_vertex:ok = False
                        elif self.face_of(col) == self.face_of(a):ok = vb-v <= 2*self.place_tol and self.stretch_check(good, col, b, checks)
                        elif self.face_of(col) == self.face_of(b):ok = v-va <= 2*self.place_tol and self.stretch_check(good, col, a, checks)
                        else:ok = False
                    elif ok and isfinite(col.slope_l) and isfinite(col.slope_r):
                        # the withheld column's own exact slope must agree with the drawn line's there: a kink of the
                        # face inside the interval (the drawn cubic is wrong away from the midpoint) fails the check
                        t = .5*(col.slope_l+col.slope_r)-hermite_dload(va, na, sa, vb, nb, sb, v)
                        x = self.eng.st[col.boundary].load
                        ok = fabs(self.M.turn_rate(v, x+fabs(t)*.25*(vb-va))-self.M.turn_rate(v, x)) <= S7_CHECK*self.ttol
                checks.append(dict(interval_kmh=[va, vb], speed_kmh=v, passed=bool(ok), faces=[a.reason, b.reason],
                                   midpoint_face=col.reason if NTracer.ok(col) else col.status_name(),
                                   kind='corner' if has_vertex else 'kink' if v != .5*(va+vb) and not corner else 'midpoint'))
                if ok:
                    passed_set.add(key);self.witnesses.append(col)
                    if corner:self.events.append(dict(kind='corner', speed_kmh=[va, vb], faces=[a.reason, b.reason],
                                                      vertex_kmh=vc if has_vertex else None, verified_at_kmh=v))
                    continue
                added += 1
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
        # S8: level flight over a forward-IAS limit with nothing inside it up to the first hard limit is an edge
        return (c.status == STATUS_NOFEASIBLE and len(c.points) > 0 and not self.eng.level_physical(c.points[0])
                and not self.eng.ias_soft(c.points[0]))

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
                if c is None or not self.numerical_gap(c) or (self.s8 and c.jump):continue
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
        self.seams = cuts;self.segments_ = segments
        if self.s7:
            import em_march_curves as C
            self.knot_speeds = C.model_knot_speeds(self.eng.solver, self.config, float(self.config['speed_min_kmh']),
                                                   float(self.config['speed_max_kmh']), self.vtol)
        if self.eng.s9:
            # S9: the forward-IAS breakpoints of the control authority (primary_controls.authority_ranges: each axis's
            # effective speed, and where its power loss reaches the authority floor)
            from control_mixer import density_at_height
            p = getattr(self.eng.solver, 'controls', None)
            knots = set()
            if p:
                for axis in range(3):
                    for side in range(2):
                        eff = float(p['effective_speed'][axis][side]);power = float(p['power'][axis][side])
                        floor = float(p['minimum'][axis])
                        if eff > 0.:
                            knots.add(eff)
                            if power > 0. and 0. < floor < 1.:knots.add(eff*pow(floor, -1./power))
            # the control mixers' deflection tables by forward IAS (AnglesMultiplier rows, the density families' rows
            # too) and their sensitivity curve by Mach (control_mixer.mix): piecewise linear, so the control authority
            # kinks at their breakpoints
            mach_knots = set()
            model = getattr(self.eng.solver, 'model', None)
            controls = model.get('controls') if isinstance(model, dict) else None
            for name in ('Ailerons', 'Elevator', 'Rudder'):
                props = (controls or {}).get(name)
                if not props:continue
                for rows in props.get('tables_1d') or ():
                    if len(rows) > 1:knots.update(float(r[0]) for r in rows)
                for family in props.get('tables_2d') or ():
                    for entry in family:
                        if len(entry[2]) > 1:knots.update(float(r[0]) for r in entry[2])
                rows = props.get('sensitivity_curve') or ()
                if len(rows) > 1:mach_knots.update(float(r[0]) for r in rows)
            # the polars' Mach curves (polar_model.mach_factor): a cubic Hermite from MachCrit to MachMax whose end slope
            # turns it back inside (A-6E RB, horizontal tail curve 7: up to x3 and down at -200 per Mach) bends the
            # chart strongly between those knots (the control face peaks at Mach 0.967): its middle and its extremum
            # are checked as knots too (its quarter points where it overshoots strongly)
            def polars(d):
                if isinstance(d, dict):
                    if 'MachCrit1' in d:yield d
                    for v in d.values():yield from polars(v)
                elif isinstance(d, list):
                    for v in d:yield from polars(v)
            bends = set()
            for props in polars(getattr(self.eng.solver, 'fm', None)):
                for i in range(1, 10):
                    try:
                        a = float(props['MachCrit%d' % i]);b = float(props['MachMax%d' % i])
                        hh = float(props['MultMachMax%d' % i])-(0. if i == 6 else 1.);ss = (b-a)*float(props['MultLineCoeff%d' % i])
                    except (KeyError, TypeError, ValueError):continue
                    if not b > a or 6*hh-3*ss == 0.:continue
                    ts = (6*hh-2*ss)/(6*hh-3*ss)
                    if not 0. < ts < 1.:continue
                    bends.update([a+(b-a)*ts, a+.5*(b-a)])
                    # a curve that overshoots its end value by more than S9_BEND of it (A-6E: 3.46 against 3) is checked
                    # at its quarter points too
                    lo0 = 0. if i == 6 else 1.
                    peak = lo0+hh*(-2*ts**3+3*ts**2)+ss*(ts**3-ts**2)
                    if abs(peak-(lo0+hh)) > S9_BEND*max(abs(lo0), abs(lo0+hh), 1e-9):
                        bends.update([a+.25*(b-a), a+.75*(b-a)])
            # only those bends are checked: a single breakpoint (thrust tables, Mach curve ends, control tables by IAS)
            # is a kink the split rule's band already covers, and checking every breakpoint cost about 30 face
            # corrections per chart for two splits in 384 charts (the IAS breakpoints stay listed, unchecked)
            self.ias_knots = []
            self.sigma = sqrt(float(density_at_height(float(self.config['altitude_m'])))/1.225)
            from air_state import speed_of_sound
            sound = float(speed_of_sound(float(self.config['altitude_m'])))*3.6
            lo_v, hi_v = float(self.config['speed_min_kmh']), float(self.config['speed_max_kmh'])
            self.s9_knots = sorted({m*sound for m in bends if lo_v < m*sound < hi_v})
        # S8: past the zero-alpha speed of a forward-IAS limit (the top segment's start hint) the segments are traced
        # upwards from the one below (they hold only states inside the limit at higher alpha), the rest top-down as before
        hint = next((s['start_kmh'] for s in segments if 'start_kmh' in s), None) if self.s8 else None
        lower = sorted([s for s in segments if hint is None or s['speed_kmh'][0] < hint], key=lambda s: -s['speed_kmh'][1])
        upper = sorted([s for s in segments if hint is not None and s['speed_kmh'][0] >= hint], key=lambda s: s['speed_kmh'][0])
        for segment in lower:
            lo, hi = segment['speed_kmh']
            neighbours = [previous] if previous is not None else []
            if previous is not None and segment['ends'][1] == 'sweep_limit':
                # S8: the segment below a stretch of unavailable wing sweep: level flight continued down through it
                probe = self.gap_probe(previous, hi)
                if probe is not None and probe[0] == 'infeasible':
                    self.events.append(dict(kind='segment infeasible', speed_kmh=segment['speed_kmh'], status='no feasible samples',
                                            reason='low-speed edge of level flight at %.2f km/h (continued through the '
                                                   'unavailable wing sweep)' % probe[1]))
                    self.eng.stats['s8_infeasible_segments'] = self.eng.stats.get('s8_infeasible_segments', 0)+1
                    continue
                if probe is not None:
                    seed = NColumn(hi);seed.points = [probe[1]]
                    neighbours = [seed]
            start = self.column(segment.get('start_kmh', hi), neighbours, 'start', True, lo-1e-9, hi+1e-9)
            starts = [start]
            # S8: past the zero-alpha speed of a forward-IAS limit (the start hint) the chart is traced upwards from below
            # only (the dive path: a state there that the sweep from below does not reach is not drawn), so the other
            # start speeds lie below the hint
            top = segment['start_kmh'] if self.s8 and 'start_kmh' in segment else hi
            if not NTracer.ok(start):
                start = self.column(.5*(lo+top), neighbours, 'start', True, lo-1e-9, hi+1e-9)
                starts.append(start)
            # S5: where neither start closes (level flight unresolved there), more start speeds across the
            # segment before it is given up (the Python march would leave the chart to the old sampler)
            for fraction in (.25, .75, .125, .375, .625, .875):
                if NTracer.ok(start):break
                start = self.column(lo+fraction*(top-lo), neighbours, 'start', True, lo-1e-9, hi+1e-9)
                starts.append(start)
            if NTracer.ok(start):
                self.sweep(segment, start)
                inside = [c for c in self.columns.values() if lo-1e-9 <= c.speed <= hi+1e-9 and NTracer.ok(c)]
                if inside:previous = min(inside, key=lambda c: c.speed)
            elif all(c.status == STATUS_NOFEASIBLE and not self.numerical_gap(c) for c in starts):
                # S8: level flight physically rejected at every start (e.g. a sliver of available wing sweep below the
                # stall speed): nothing to draw there, no gap
                self.events.append(dict(kind='segment infeasible', speed_kmh=segment['speed_kmh'], status=start.status_name(),
                                        reason=str(start.raw_reason)[:80]))
            else:
                self.events.append(dict(kind='segment unresolved', speed_kmh=segment['speed_kmh'], status=start.status_name()))
        if self.s8:self.seam_restarts(segments)
        for segment in upper:self.upper_segment(segment)
        if self.s8:self.dive_path(segments)
        self.ias_one_g()
        checks = self.check()
        if self.nudge:checks += self.resolve_gaps(segments)
        for c in self.witnesses:
            c.withheld = True
            self.columns.setdefault(float(c.speed), c)
        if self.nudge:self.within_tolerance(segments)
        cols = sorted(self.columns.values(), key=lambda c: c.speed)
        return dict(columns=cols, segments=segments, mach_seams_kmh=self.mach_seams, sweep_gaps_kmh=self.sweep_gaps,
                    events=self.events, checks=checks, work_by_role=self.work, elapsed_s=time.perf_counter()-started,
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
        if col.lower >= 0:
            # S8: the column's lower edge (level flight over a forward-IAS limit): the chart's leaning edge
            out['lower_edge'] = self.eng.row_of(col.lower);out['lower_edge_kind'] = 'forward IAS limit'
            if isfinite(col.lower_slope):out['lower_edge_slope'] = col.lower_slope
        if col.nudged_from is not None:out['s6_requested_speed_kmh'] = float(col.nudged_from)
        if col.slope_kind is not None:
            out['boundary_slope'] = [col.slope_l if isfinite(col.slope_l) else None, col.slope_r if isfinite(col.slope_r) else None]
            out['boundary_slope_kind'] = col.slope_kind
        return out

    @cython.boundscheck(True)
    @cython.wraparound(True)
    def debug_column(self, double speed, list near, bint cheap=False):
        """Diagnostics (S8): one column at ``speed`` seeded from the trace's columns at the ``near`` speeds (not kept)."""
        cdef NColumn col
        seeds = [self.columns[v] for v in near if v in self.columns]
        self.cheap = cheap
        try:
            col = self.column(speed, seeds, 'debug', False, -INFINITY, INFINITY)
        finally:
            self.cheap = False
        return dict(speed=col.speed, status=col.status_name(), reason=col.reason, raw=str(col.raw_reason)[:300],
                    march_reason=col.march_reason, level=col.level, trials=col.trial_log,
                    points=[(round(self.eng.st[q].alpha, 3), round(self.eng.st[q].load, 3), bool(self.eng.st[q].valid),
                             self.eng.reason_list(q)) for q in col.points],
                    turn=self.eng.st[col.boundary].turn if col.boundary >= 0 else None,
                    load=self.eng.st[col.boundary].load if col.boundary >= 0 else None, boundary=col.boundary)

    def debug_probe(self, int level, double speed):
        """Diagnostics (S8): level_probe from state ``level`` at ``speed`` (state id, -1 physical rejection, -3 open)."""
        cdef PredMap preds
        preds.n = 0
        return self.level_probe(level, speed, &preds)

    @cython.boundscheck(True)
    @cython.wraparound(True)
    def column_state(self, NColumn col):
        return dict(points=list(col.points), boundary=col.boundary, level=col.level, status=col.status_name(),
                    reason=col.reason, march_reason=col.march_reason)
