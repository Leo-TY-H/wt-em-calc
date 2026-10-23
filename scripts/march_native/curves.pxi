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
                'force_error_g', 'angular_error_rad_s2', 'history_error', 'converged', 'chart_rule',
                'propeller_braking')


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
DEF S13_ROOT_SEP = .001                   # S13: a level root on the other propeller branch: lower load by more than this x
DEF S16_OUTLINE_SAMPLES = 16              # S16: points across a contour piece checked against the drawn outline
DEF S16_BRANCH_DV = 1.                   # S16: km/h, a level-flight step of branch_crossing
DEF S16_BRANCH_STEPS = 40                # S16: ... at most this many (within the two level flights)
DEF S16_BUBBLE_SEP = 1.                  # S16: m/s, a level this far beyond both neighbouring level flights is looked for between them
DEF S16_BUBBLE_STEPS = 6                 # S16: parabolic steps to level flight's SEP extremum between two columns
DEF S16_BRANCH_STEP = .25                # S16: x the speed tolerance, the shortest continuation step that may cross a propeller branch switch
DEF S16_PATH_DA = .5                      # S16: deg, the largest alpha step of a level-flight path march (path_state)
DEF S16_PATH_SEP = 5.                     # S16: x the SEP allowance, a curve state's SEP apart from the path state's at its load


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
    cdef int root_a, root_b              # S16: the two level-flight states of that bracket
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
    # S13 (WT_MARCH_S13=0 restores S12 exactly): a contour piece whose two end states lie on different propeller branches
    # (S9's propulsion-coordinate test) is not bounded by its end values and slopes, since the branch switch can lie
    # anywhere inside it: it is split until it is narrower than the speed tolerance (a step). Where both branches have
    # a root of the level at a split speed, the knot is the one at the lower load: the first crossing of the level along
    # alpha from level flight (the envelope's first-limit rule, Leo 2026-10-17). F4U-1D RB -40 m/s at 746-752 km/h.
    cdef public bint s13
    # S14: a contour piece whose end states lie in different regimes of a propeller governor (its coordinate on either
    # side of 0, the fine-pitch stop, or of 1: the RPM governed or not) is split at the crossing, interpolated in speed:
    # the regime switch kinks SEP, against the curvature the piece bound assumes (PBY-5 SB +10 m/s at 154-155 km/h,
    # where each engine reaches its governed 2,700 rpm: drawn 0.15 deg/s off)
    cdef public bint s14
    # S15 (WT_MARCH_S15=0: S14): the last column with level flight closes the solved columns (link_columns)
    cdef public bint s15
    # S16 (WT_MARCH_S16=0: S15): every level present at an envelope column is drawn there (Leo 2026-10-23: "contour not
    # plotted when they exist within the envelope"): columns above a lower edge (level flight over a soft limit) are
    # looked at too; a level met at a column no followed contour passes is followed from there; a level met at level
    # flight is a 1 g crossing (never a one-point contour stuck at 1 g); no over-rev state below a lower edge is transient
    cdef public bint s16
    cdef public list env_lo               # S16: the lower-edge load of each envelope column (1 without one)
    cdef public list pcol_v               # S16: speeds of the verified columns whose alpha paths carry propulsion states
    cdef public list pcol                 # S16: each such column's admitted path samples (state ids by load)
    cdef public list edge_knots           # S16: the rows of each traced leaning edge's knots (NTracer.edge_rows), or None
    cdef public list rpm_feet             # S16: the speeds where level flight first over-revs (the 'rpm edge' events)
    # S16: the drawn outline's closed regions (em_march_chart.curve_outline): their points (speed, turn) concatenated,
    # the start of each region, and the units a point outside is measured in (speed and envelope tolerances)
    cdef double* opts
    cdef int* ooff
    cdef int nreg
    cdef double ounit_v, ounit_t
    cdef list gov_idx
    cdef list pending                    # S10: components found at folds, still to follow
    cdef dict grads                      # S10: (dSEP/dV, dSEP/dw) of curve states by their base state

    @cython.boundscheck(True)
    @cython.wraparound(True)
    def __cinit__(self, *args, **kwargs):
        self.pool = NULL
        self.opts = NULL;self.ooff = NULL;self.nreg = 0

    def __dealloc__(self):
        if self.pool != NULL:cfree(self.pool)
        if self.opts != NULL:cfree(self.opts)
        if self.ooff != NULL:cfree(self.ooff)

    @cython.boundscheck(True)
    @cython.wraparound(True)
    def set_outline(self, list regions, double unit_v, double unit_t):
        """S16: the drawn outline the contours must stay inside: closed regions, each a list of (speed km/h, turn deg/s)
        points (em_march_chart.curve_outline); a point outside is measured in units of (unit_v km/h, unit_t deg/s)."""
        cdef int total = 0, k = 0, r
        if self.opts != NULL:cfree(self.opts)
        if self.ooff != NULL:cfree(self.ooff)
        self.opts = NULL;self.ooff = NULL;self.nreg = 0
        regions = [reg for reg in regions if len(reg) >= 3]
        for reg in regions:total += len(reg)
        if not regions:return
        self.opts = <double*>malloc(2*total*sizeof(double));self.ooff = <int*>malloc((len(regions)+1)*sizeof(int))
        if self.opts == NULL or self.ooff == NULL:raise MemoryError()
        for r, reg in enumerate(regions):
            self.ooff[r] = k
            for x, y in reg:
                self.opts[2*k] = float(x);self.opts[2*k+1] = float(y);k += 1
        self.ooff[len(regions)] = k
        self.nreg = len(regions);self.ounit_v = unit_v;self.ounit_t = unit_t

    cdef double outline_outside(self, double x, double t) noexcept:
        """S16: how far the point (speed ``x``, turn ``t``) lies outside the drawn outline, in its units (0 inside any
        region: even-odd crossings; else the distance to the nearest region edge)."""
        cdef int r, i, j, a, b, cross
        cdef double x0, y0, x1, y1, xc, best = INFINITY, px, py, qx, qy, ex, ey, l, u, d
        for r in range(self.nreg):
            a = self.ooff[r];b = self.ooff[r+1];cross = 0
            for i in range(a, b):
                j = i+1 if i+1 < b else a
                x0 = self.opts[2*i];y0 = self.opts[2*i+1];x1 = self.opts[2*j];y1 = self.opts[2*j+1]
                if (y0 > t) != (y1 > t):
                    xc = x0+(t-y0)*(x1-x0)/(y1-y0)
                    if x < xc:cross += 1
            if cross % 2:return 0.
        px = x/self.ounit_v;py = t/self.ounit_t
        for r in range(self.nreg):
            a = self.ooff[r];b = self.ooff[r+1]
            for i in range(a, b):
                j = i+1 if i+1 < b else a
                qx = self.opts[2*i]/self.ounit_v;qy = self.opts[2*i+1]/self.ounit_t
                ex = self.opts[2*j]/self.ounit_v-qx;ey = self.opts[2*j+1]/self.ounit_t-qy
                l = ex*ex+ey*ey
                u = ((px-qx)*ex+(py-qy)*ey)/l if l > 0. else 0.
                u = pmin(1., pmax(0., u))
                d = sqrt((qx+u*ex-px)*(qx+u*ex-px)+(qy+u*ey-py)*(qy+u*ey-py))
                if d < best:best = d
        return best

    cdef bint outside(self, int p) noexcept:
        """S16: an admitted curve state that lies more than one unit outside the drawn outline: beyond the envelope by the
        chart's own rule (the first limit along alpha from level flight: a trim fold or a pitch-response reversal stops the
        march where the state's own admission still passes, Hunter F.58A SB -200 m/s at 1,096-1,112 km/h), so no contour
        knot."""
        if not self.s16 or self.nreg == 0 or p < 0 or not self.eng.st[p].valid:return False
        return self.outline_outside(self.eng.st[p].speed, self.eng.st[p].turn) > 1.

    cdef double outline_exit(self, HermiteCurve hf, double va, double vb) noexcept:
        """S16: the speed inside (va, vb) where the drawn contour piece lies farthest outside the drawn outline, when that
        is more than one unit (S16_OUTLINE_SAMPLES points across it); NaN where it stays inside."""
        cdef int j
        cdef double x, w, d, best = 1., at = NAN
        if self.nreg == 0:return NAN
        for j in range(1, S16_OUTLINE_SAMPLES):
            x = va+(vb-va)*j/S16_OUTLINE_SAMPLES
            w = hf.eval(x)
            d = self.outline_outside(x, curve_turn(x, sqrt(1.+pmax(w, 0.))))
            if d > best:
                best = d;at = x
        return at

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
        self.s13 = self.eng.s12 and os.environ.get('WT_MARCH_S13', '1') != '0'
        self.s14 = self.eng.s14
        self.s15 = self.s14 and os.environ.get('WT_MARCH_S15', '1') != '0'
        self.s16 = self.s15 and self.eng.s16
        self.edge_knots = None;self.rpm_feet = []
        self.gov_idx = [j for j, name in enumerate(self.eng.layout_names) if str(name).startswith('governor:')]
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
        cdef double v, n, lo
        cdef int i
        if sid < 0 or not self.eng.soft or self.eng.st[sid].valid or not self.env_v:return 0
        if not self.eng.only_reasons(sid, (<unsigned long long>1) << R_RPM):return 0
        v = self.eng.st[sid].speed
        i = bisect.bisect_left(self.env_v, v)
        if i < len(self.env_v) and fabs(self.env_v[i]-v) <= 1e-9:n = self.env_n[i]
        elif i > 0 and fabs(self.env_v[i-1]-v) <= 1e-9:n = self.env_n[i-1]
        elif 0 < i < len(self.env_v):n = pmin(self.env_n[i-1], self.env_n[i])
        else:return 0
        if self.s16 and self.env_lo and not (self.rpm_feet and all(v <= f+1e-9 for f in self.rpm_feet)):
            # S16: past where level flight over-revs (the engine RPM edge's foot), not below the lower edge of the
            # columns either side (the over-rev stretch from level flight up to that edge lies outside the outline);
            # below the foot level flight is admitted and an over-rev stretch on the way is transient
            if i < len(self.env_v) and fabs(self.env_v[i]-v) <= 1e-9:lo = self.env_lo[i]
            elif i > 0 and fabs(self.env_v[i-1]-v) <= 1e-9:lo = self.env_lo[i-1]
            else:lo = pmax(self.env_lo[i-1], self.env_lo[i])
            if self.eng.st[sid].load <= lo+1e-9:return 0
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
        from em_trim_work import WorkLimit
        cdef int sid
        if self.check is not None:self.check()
        self.eng.flush_counts()
        before = self.ledger.snapshot()
        initial = None
        if anchor >= 0:
            attitude = [self.eng.st[anchor].sol[i] for i in range(5)]
            attitude[1] += (acos(1./load)-acos(1./pmax(1., self.eng.st[anchor].load)))*RAD_TO_DEG
            initial = self.eng.initial_dict(anchor, attitude)
        try:
            sid = self.eng.points_().solve(speed, load, initial)
        except WorkLimit as error:
            # S16: a point solve that establishes no candidate (a speed and load outside every equilibrium: a fixed solve
            # past the envelope that S16's outline checks reach, Do 17 Z-7 SB) is a failed sample, not the end of the
            # whole curve solve (it raised through run() and the chart fell back to the old sampler)
            if not (self.s16 and anchor >= 0 and 'no established candidate' in str(error.kind)):raise
            sid = self.eng.new_state()
            self.eng.st[sid] = self.eng.st[anchor]
            self.eng.st[sid].src = -1;self.eng.st[sid].row = -1;self.eng.st[sid].base = sid;self.eng.st[sid].limit = -1
            self.eng.st[sid].valid = False;self.eng.st[sid].converged = False;self.eng.st[sid].nreasons = 0
            self.eng.st[sid].speed = speed;self.eng.st[sid].load = load
            self.eng.stats['s16_fixed_unresolved'] = self.eng.stats.get('s16_fixed_unresolved', 0)+1
            self.eng.flush_counts()
            self.ledger.charge(phase, before, 'unresolved', kind='fixed_load', speed_kmh=speed, load_g=load,
                               exhaustive=exhaustive or None)
            return sid
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
        elif self.outside(p) and fabs(self.eng.st[p].ps-target) <= self.allowance:
            # S16: the level's state lies beyond the drawn envelope: an exit, as a physical rejection there
            outcome, reason = 'rejected', ['beyond the envelope outline']
            self.eng.stats['s16_outside_outline'] = self.eng.stats.get('s16_outside_outline', 0)+1
        elif (self.s16 and self.eng.st[p].valid and fabs(self.eng.st[p].ps-target) <= self.allowance
              and self.path_check(p, anchor) == 1):
            # S16 (Leo 2026-10-23): the level's state on another propeller branch than the one reached by pulling from
            # level flight at its speed (N1K1 RB at 3,000 m: +10 m/s at 280 km/h jumped from the path's governing state,
            # 2.14 g, to one on its pitch stop at 1.75 g): not this chart's state
            outcome, reason = 'unresolved', 'another propeller branch than the level-flight path'
            self.eng.stats['s16_off_path'] = self.eng.stats.get('s16_off_path', 0)+1
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

    def debug_path(self, double v, double n):
        """Diagnostics (S16): path_state at (v, n) from the nearer neighbouring column: its (speed, load, turn, SEP,
        propulsion coordinates), or None."""
        import bisect
        cdef int i, k, lo = -1, hi = -1
        if not self.pcol_v:return None
        i = bisect.bisect_left(self.pcol_v, v)
        k = min(max(i-1, 0), len(self.pcol_v)-1)
        if 0 < i < len(self.pcol_v) and self.pcol_v[i]-v < v-self.pcol_v[i-1]:k = i
        if not self.path_bracket(v, n, k, &lo, &hi):return None
        return [(self.eng.st[q].speed, self.eng.st[q].load, self.eng.st[q].turn, self.eng.st[q].ps,
                 [self.eng.st[q].coords[i] for i in range(7, self.eng.st[q].nx)]) for q in (lo, hi)]

    def debug_fixed(self, double v, double n, int anchor):
        """Diagnostics (S16): fixed() at (v, n) from ``anchor``: (state, valid, converged, ps, compatible with anchor,
        reasons)."""
        cdef int q = self.fixed(v, n, anchor, 'debug')
        return (q, self.eng.st[q].valid, self.eng.st[q].converged, self.eng.st[q].ps, self.eng.compatible(q, anchor),
                self.eng.reason_list(q))

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
            elif self.s16 and outcome != 'rejected':
                # S16: a plain solve that does not close far from the last state (a steep level near its 1 g end: the
                # load seeded from that state is far off) is no evidence of the exit: the level solved as the split does
                # (A6M6c RB at 65% pitch: +10 m/s was excluded over the whole 162-185 km/h piece)
                q = self.solve_level(mid, target, good, -1, 'interpolation_checks')
                if q >= 0:
                    lo = mid;good = q
                else:hi = mid
            else:hi = mid
        return good if good != k else -1

    cdef bint ok_root(self, int p, int anchor, double target) except -1:
        if p < 0:return False
        self.admit_transient(p)
        return (self.eng.st[p].valid and fabs(self.eng.st[p].ps-target) <= self.allowance and self.eng.compatible(p, anchor)
                and not self.outside(p) and not (self.s16 and self.path_check(p, anchor) == 1))

    # ------------------------------------------------------------------ S9: level solves that do not close
    cdef bint exit_evidence(self, int p) except -1:
        """S9: a state showing that the level is not reached inside the envelope there: a balanced rejection by physical
        limits only (no sub-solve of the admission that did not close)."""
        cdef TState* t
        cdef int i
        if p < 0:return False
        if self.outside(p):return True     # S16: beyond the drawn envelope
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
                if self.outside(p):
                    # S16: the level lies beyond the drawn envelope here
                    self.exit_seen = True;self.exit_state = p;break
                if self.s16 and self.path_check(p, a) == 1:
                    # S16: on another propeller branch than the level-flight path (direct)
                    self.eng.stats['s16_off_path'] = self.eng.stats.get('s16_off_path', 0)+1
                    break
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
        cdef double t, n, f, slack
        if fabs(self.eng.st[b].ps-target) <= self.allowance:
            return dict(outcome='admitted', point=b, bracket=[self.eng.st[a].load, self.eng.st[b].load])
        t = (target-self.eng.st[a].ps)/(self.eng.st[b].ps-self.eng.st[a].ps)
        n = py_sqrt(1.+(pow(self.eng.st[a].load, 2.)-1.)+t*(pow(self.eng.st[b].load, 2.)-pow(self.eng.st[a].load, 2.)))
        f = (n-self.eng.st[a].load)/(self.eng.st[b].load-self.eng.st[a].load)
        attitude = [self.eng.st[a].sol[k]*(1-f)+self.eng.st[b].sol[k]*f for k in range(5)]
        outcome, point, reason = self.direct_seed(speed, target, n, [a, b], attitude, phase)
        # S16: a bracket narrower than the curve tolerance in turn rate (a level next to the boundary: Su-25T SB -200 m/s
        # at 834.8 km/h, 9.5593-9.5601 g) is widened by that tolerance (the same root, not another crossing)
        slack = 1e-9
        if self.s16 and outcome == 'admitted' and fabs(self.eng.st[b].turn-self.eng.st[a].turn) < self.turn_tol:
            slack = self.turn_tol*RAD_TO_DEG**-1*(speed/3.6)*sqrt(pmax(n*n-1., 1e-12))/(G_TURN*n)
        if outcome == 'admitted' and not (self.eng.st[a].load-slack <= self.eng.st[point].load <= self.eng.st[b].load+slack
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
            self.root_a = a;self.root_b = b
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
                if (self.eng.st[one].valid and self.sense(comp)*(self.eng.st[one].ps-target) < 0.
                        and not (self.s16 and not below and self.eng.st[last].load > 1.+1e-9 and self.rising_at(one))):
                    # (S16: not where SEP rises with load from level flight at this speed: the contour can lie above 1 g
                    # here and turn back to it in speed (Do 17 E-1 SB -40 m/s: from 505.3 km/h out to 509.5 between 3.5
                    # and 7.9 deg/s, back to 1 g at 506.4); the solve above 1 g first, S10's fold_turn at its tip)
                    end = self.one_g_endpoint(comp, target, last, one, side)
                    if end.get('state') is not None and end['kind'] == 'one_g':
                        self.add_knot(comp, end['state'], side)
                    return end
                w = max(w, (pow(self.eng.st[last].load, 2.)-1.)*.25)
            n = py_sqrt(1.+max(w, 1e-6))
            outcome, point, reason = self.direct_seed(speed, target, n, [last], attitude, 'continuation')
            if (outcome == 'admitted' and self.s16 and abs(speed-self.eng.st[last].speed) > S16_BRANCH_STEP*self.speed_tol
                    and self.step_jump(comp, last, point, side)):
                # S16: a continuation step that lands on another propeller branch than the last knot is no step of the
                # contour (A6M2-N RB at 65% pitch: from its 1 g crossing at 162.7 km/h on the branch that ends at 163.5,
                # one step reached the other branch's +10 m/s at 193 km/h): shorter steps keep the branch to where it
                # ends (down to S16_BRANCH_STEP of the speed tolerance: a branch's piece can be narrower than it), a
                # switch is crossed within that only
                outcome, reason = 'incompatible', 'propeller branch changed during continuation'
                self.eng.stats['s16_branch_steps'] = self.eng.stats.get('s16_branch_steps', 0)+1
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
            # (S16: a branch change, or a contour of one knot so far, which draws nothing, halves on to S16_BRANCH_STEP of
            # the speed tolerance: A6M2-N RB at 65% pitch, +10 m/s from its 1 g crossing at 162.7 km/h on a branch that
            # ends at 163.5)
            if abs(speed-self.eng.st[last].speed) <= self.speed_tol and not (
                    self.s16 and (reason == 'propeller branch changed during continuation' or len(comp['knots']) == 1)
                    and abs(speed-self.eng.st[last].speed) > S16_BRANCH_STEP*self.speed_tol):
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
        step along dSEP/dV of ``anchor``), each seeded from the previous; the state within S9_END_SEP of the SEP allowance
        of the level (next to a fold SEP changes slowly along speed: within the allowance alone a knot can sit 0.3 km/h
        off, XP-55 SB +10 m/s), else the nearest one within the allowance, or -1 (a solve not admitted or on another
        branch, or none within the allowance in S10_ROOT_STEPS)."""
        cdef int p, q, k, best = -1
        cdef double v1, f0, f1, d, tol = S9_END_SEP*self.allowance
        p = self.fixed(v0, n, anchor, 'fold')
        if not (self.eng.st[p].valid and self.eng.compatible(p, anchor)):return -1
        f0 = self.eng.st[p].ps-target
        if fabs(f0) <= tol:return p
        if fabs(f0) <= self.allowance:best = p
        d, _ = self.fold_grad(anchor)
        if not isfinite(d) or d == 0.:return best
        v1 = v0-f0/d
        v1 = pmin(pmax(v1, v0-2.*self.speed_tol), v0+2.*self.speed_tol)
        for k in range(S10_ROOT_STEPS):
            q = self.fixed(v1, n, p, 'fold')
            if not (self.eng.st[q].valid and self.eng.compatible(q, anchor)):return best
            f1 = self.eng.st[q].ps-target
            if fabs(f1) <= tol:return q
            if fabs(f1) <= self.allowance and (best < 0 or fabs(f1) < fabs(self.eng.st[best].ps-target)):best = q
            if f1 == f0 or self.eng.st[q].speed == self.eng.st[p].speed:return best
            d = (f1-f0)/(self.eng.st[q].speed-self.eng.st[p].speed)
            if d == 0.:return best
            p = q;f0 = f1;v0 = self.eng.st[q].speed
            v1 = v0-f0/d
            v1 = pmin(pmax(v1, v0-2.*self.speed_tol), v0+2.*self.speed_tol)
        return best

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef object fold_turn(self, dict comp, int last, int side, int one):
        """S10: the contour turns back in speed beyond its knot ``last`` (on a branch where SEP falls with load), where level
        flight at that knot's speed (``one``) lies outside the level too: the level's second root at that speed, between
        level flight and the knot (fold_sample, then the bracketed solve). Where the knot itself lies on or just past the turn
        (within the SEP allowance of the level, below it: no second root at its speed), at the speed of one of the three
        knots before it instead, the first whose level flight lies outside the level too. Found, the component ends at the
        fold ('fold' end; its fold piece joins ``last`` to that root, refined by check_fold) and the other branch (a 'below'
        component: SEP rises with load there) is followed away from the fold. None where no second root is found (the S9
        ends then)."""
        cdef double target = comp['level'], vk = self.eng.st[last].speed, v = vk, fo, fh, sv
        cdef int hit = -1, other_root = -1, top = last, k, q, i
        if not self.s10 or self.eng.st[last].load <= 1.+1e-6:return None
        # (from next to the knot down: where the turn is close, the other root lies just below the knot's load, XF5U-1 RB
        # -10 m/s at 683 km/h)
        hit = self.fold_sample(v, target, last, (.95, .8, .6, .4, .2))
        if hit < 0:
            # the knot lies past the turn (Do 17Z-2 SB -20 m/s: 491.87 km/h, 0.047 m/s under the level) or on its tip
            # (XF5U-1 RB -10 m/s at 683 km/h, where SEP peaks sharply in load): the turn lies between it and the knots
            # before it, whose speeds have both roots (samples from next to the knot down)
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
                        # (not level flight at the knot's own speed, even within S9_END_SEP: the end would leave no knot
                        # at 1 g to draw it by; DB-3B RB +10 m/s at 250.7 km/h, the knot at 1.001 g)
                        r_ = self.one_g_root(target, q, one, 'fold', one)
                        if r_ is not None:
                            k = r_
                            if (self.eng.compatible(k, last) and self.eng.st[k].load == 1.
                                    and pmin(self.eng.st[q].speed, vk) < self.eng.st[k].speed < pmax(self.eng.st[q].speed, vk)):
                                comp['fold'] = dict(knots=[last, k], side=side)
                                self.fold_slope(last)
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
        self.fold_slope(last);self.fold_slope(other_root)
        self.pending.append(other)
        self.eng.stats['s10_folds'] = self.eng.stats.get('s10_folds', 0)+1
        return dict(kind='fold', speed_kmh=vk, other_load_g=self.eng.st[other_root].load)

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef fold_slope(self, int p):
        """S10: the drawn slope dw/dV of a component's knot at its fold from the level set's own gradient there (fold_dvdw:
        fixed-load solves, which exist across the turn), not from level solves at V +- delta, which straddle the turn
        there (Do 17 Z-7 RB -40 m/s: the lower half drawn with an S-bend 0.4 deg/s low at 570 km/h)."""
        cdef double d = self.fold_dvdw(p), sl
        if not isfinite(d) or d == 0.:return
        sl = 1./d
        if fabs(sl) > S9_END_VERTICAL:sl = copysign(S9_END_VERTICAL, sl)
        self.slopes[self.eng.st[p].base] = (sl, sl)
        self.eng.stats['s10_fold_slopes'] = self.eng.stats.get('s10_fold_slopes', 0)+1

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
    cdef bint fold_absorb(self, dict comp, int i) except -1:
        """S10: the knot piece ``i`` of a component next to its fold (between the fold's knot and the one before it) joins
        the fold piece: the knot before becomes a fold knot and the fold's knot leaves the component. Where the split rule
        cannot finish that piece within the speed tolerance, the level is steep in speed there because the turn is near
        (XP-55 SB +10 m/s, 460.9-461.9 km/h), and V(w) draws it smoothly where w(V) would need a step."""
        fold = comp.get('fold')
        if fold is None:return False
        knots = comp['knots']
        if fold['side'] > 0 and i == len(knots)-2 and knots[-1] in fold['knots']:
            fold['knots'].append(knots[i]);knots.pop()
        elif fold['side'] < 0 and i == 0 and len(knots) >= 2 and knots[0] in fold['knots']:
            fold['knots'].append(knots[1]);knots.pop(0)
        else:
            return False
        self.eng.stats['s10_fold_absorbed'] = self.eng.stats.get('s10_fold_absorbed', 0)+1
        return True

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
    cdef int prop_branch(self, list knots, int i) except -1:
        """S13: contour piece i (knots i, i+1) crosses a propeller branch switch: a propulsion coordinate (after the seven
        attitude and wing-history ones) of each end differs from its value continued from the other side by more than
        S9_PROP_JUMP of its size (at least 1). Each side is continued linearly through its own neighbouring knot (both
        sides must disagree, so a smooth drift over a long piece is not a switch); a side without a neighbour is held
        constant (S9's NTracer.prop_jump test). False where the layouts differ or a state slides on a blade-polar jump."""
        cdef int n = len(knots), a = knots[i], b = knots[i+1], j, pa = -1, pb = -1
        cdef TState* s = &self.eng.st[a]
        cdef TState* t = &self.eng.st[b]
        cdef TState* u
        cdef double xa, xb, ca, cb, tol
        if not (s.has_coords and t.has_coords) or s.nx != t.nx or s.nx <= 7 or s.slide.n > 0 or t.slide.n > 0:return False
        xa = s.speed;xb = t.speed
        if i > 0:
            pa = knots[i-1];u = &self.eng.st[pa]
            if not u.has_coords or u.nx != s.nx or u.slide.n > 0 or not u.speed < xa:pa = -1
        if i+2 < n:
            pb = knots[i+2];u = &self.eng.st[pb]
            if not u.has_coords or u.nx != s.nx or u.slide.n > 0 or not u.speed > xb:pb = -1
        for j in range(7, s.nx):
            tol = S9_PROP_JUMP*pmax(1., pmax(fabs(s.coords[j]), fabs(t.coords[j])))
            ca = s.coords[j]
            if pa >= 0:ca += (s.coords[j]-self.eng.st[pa].coords[j])*(xb-xa)/(xa-self.eng.st[pa].speed)
            cb = t.coords[j]
            if pb >= 0:cb += (t.coords[j]-self.eng.st[pb].coords[j])*(xa-xb)/(xb-self.eng.st[pb].speed)
            if fabs(t.coords[j]-ca) > tol and fabs(s.coords[j]-cb) > tol:return True
        return False

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef bint regime_cross(self, int a, int b, double* at) except -1:
        """S14: curve states ``a`` and ``b`` have a propeller governor coordinate on either side of 0 or of 1 (its regime
        kinks); ``at``: the first such crossing, interpolated linearly in speed. False where a state slides on a jump."""
        cdef TState* s = &self.eng.st[a]
        cdef TState* t = &self.eng.st[b]
        cdef double ca, cb, k, x, best = INFINITY
        if not (s.has_coords and t.has_coords) or s.nx != t.nx or s.slide.n > 0 or t.slide.n > 0:return False
        for j in self.gov_idx:
            if j >= s.nx:continue
            ca = s.coords[j];cb = t.coords[j]
            for k in (0., 1.):
                if (ca-k)*(cb-k) < 0.:
                    x = s.speed+(k-ca)/(cb-ca)*(t.speed-s.speed)
                    if x < best:best = x
        if not isfinite(best):return False
        at[0] = best
        return True

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
        cdef double x, tx, lo_t, hi_t, box, xc = 0., xo
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
            additions = [];absorbed = False
            for i in range(len(knots)-1):
                a = knots[i];b = knots[i+1]
                key = (self.eng.st[a].speed, self.eng.st[b].speed)
                if key in done or key in blocked:continue
                branch_piece = False
                if self.s8:
                    # S8: the split rule: no solve where the piece's error bound is within the tolerance
                    h = key[1]-key[0]
                    e = self.piece_bound(hf, i, &xm) if h > 0. else 0.
                    if self.s13 and h > self.speed_tol and self.sense(comp) > 0. and self.prop_branch(knots, i):
                        # S13: ends on different propeller branches: the switch can lie anywhere inside, so no bound
                        branch_piece = True;e = INFINITY;xm = .5*(key[0]+key[1])
                        self.eng.stats['s13_branch_splits'] = self.eng.stats.get('s13_branch_splits', 0)+1
                    elif self.s14 and h > 2.*self.speed_tol and self.regime_cross(a, b, &xc):
                        # S14: a governor regime switch inside: split at its interpolated crossing
                        e = INFINITY;xm = xc
                        self.eng.stats['s14_regime_splits'] = self.eng.stats.get('s14_regime_splits', 0)+1
                    if self.s16 and h > self.speed_tol and not branch_piece:
                        # S16: a piece drawn outside the drawn outline (Saab J35A RB -400 m/s across the boundary's dip at
                        # 610-630 km/h): no bound passes it; the level is solved where it lies farthest out (an exit there
                        # ends the contour at the boundary on both sides: failed_piece)
                        xo = self.outline_exit(hf, key[0], key[1])
                        if isfinite(xo):
                            e = INFINITY;xm = xo
                            self.eng.stats['s16_outline_splits'] = self.eng.stats.get('s16_outline_splits', 0)+1
                    bound = e if isfinite(e) else None
                    if h <= self.speed_tol or (e <= self.turn_tol and h <= pmin(self.max_step, S8_MAX_WIDTH)):
                        if e > self.turn_tol and self.s10 and self.fold_absorb(comp, i):
                            # S10: next to the fold the level is steep in speed, not broken: drawn in V(w) with the fold
                            absorbed = True
                            break
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
                if branch_piece:
                    # S13: the level's root on each end's branch (seeded from that end alone); the knot is the root at the
                    # lower load (the first crossing along alpha from level flight)
                    best = truth if outcome == 'admitted' else -1
                    for anchor in (a, b):
                        o2, p2, r2 = self.direct_seed(speed, target, self.eng.st[anchor].load, [anchor], None, 'interpolation_checks')
                        # another root only: a load lower by more than S13_ROOT_SEP of it (not the same root converged
                        # to a slightly different load)
                        if (o2 == 'admitted' and self.eng.compatible(p2, a) and self.eng.compatible(p2, b)
                                and (best < 0 or self.eng.st[p2].load < self.eng.st[best].load*(1.-S13_ROOT_SEP))):
                            best = p2
                    if best >= 0:
                        if best != truth:
                            self.eng.stats['s13_lower_roots'] = self.eng.stats.get('s13_lower_roots', 0)+1
                        outcome, truth, reason = 'admitted', best, None
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
            if not additions and not absorbed:return
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
    cdef bint column_start(self, column, list pts) except -1:
        """Whether the column's lowest admitted sample starts its region: level flight, or (S16) the column's lower edge
        (level flight over a soft limit: forward IAS, engine RPM)."""
        if not pts:return False
        if abs(self.eng.st[pts[0]].load-1.) <= 1e-9:return True
        if not self.s16 or not column.get('lower_edge'):return False
        return abs(self.eng.st[pts[0]].load-float(column['lower_edge']['load_g'])) <= 1e-9

    cdef object column_levels(self, column):
        """CurveSolver.column_levels: the levels column_roots would find at this column (no solves), or None."""
        cdef int i
        pts, _ = self.column_samples(column)
        if not self.column_start(column, pts):return None
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
        if not self.column_start(column, pts):
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
                # S16: a level met at level flight itself is a 1 g crossing: it is followed from where it is located
                # between the columns' level flights (from_one_g, with the level set's slope there); a one-point
                # component at 1 g has no slope to leave it by (Ki-21-I RB -20 m/s at 490 km/h was never drawn)
                if self.s16 and abs(self.eng.st[pts[0]].load-1.) <= 1e-9:
                    self.eng.stats['s16_one_g_roots'] = self.eng.stats.get('s16_one_g_roots', 0)+1
                    continue
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
        if self.s15:
            # S15: the last column with level flight ends the solved columns where the last column itself has none
            # (past a forward-IAS limit): a level that appears only between them was never looked for there (Yak-3
            # landing flaps, Ps = 0 from the top boundary into the flap-limit edge)
            for speed in reversed(ordered):
                if solved and speed <= solved[-1]:break
                here = self.column_levels(by_speed[speed])
                if here is None:continue
                presence[speed] = here;solved.append(speed)
                self.eng.stats['s15_last_level_columns'] = self.eng.stats.get('s15_last_level_columns', 0)+1
                break
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
        self.follow_new(new, components, segment)
        if self.s16:
            # S16: every column's level flight is a 1 g sample of the level crossings along level flight (run(): a level
            # met at level flight between two thinned columns and left again before the next one)
            for speed in ordered:
                if speed in ones:continue
                for p in by_speed[speed]['points']:
                    if abs(p['load_g']-1.) <= 1e-9 and p.get('valid') and p.get('stationary_coordinates') is not None:
                        ones[speed] = self.state(p);break
            # S16: every other column of the segment: a level present there that no followed contour of it passes (a
            # second contour of the level between two solved columns, e.g. an island against the boundary next to a
            # corner) is followed from that column (He 111 H-16 RB -20 m/s at 316 km/h, Hampden RB -10 at 399)
            for speed in ordered:
                if speed in presence or speed in extra:continue
                column = by_speed[speed]
                here = self.column_levels(column)
                if not here:continue
                pts, _ = self.column_samples(column)
                levels = [L for L in self.levels if L in here and not self.level_covered(components, segment, L, pts)]
                if not levels:continue
                seed = self.column_roots(column, levels)
                self.ladders.append(dict(speed_kmh=speed, segment=segment['speed_kmh'], ladder=seed))
                seeds_report.append(dict(speed_kmh=speed, provenance='envelope column (level not followed there)',
                                         roots=sum(r['outcome'] == 'admitted' for r in seed['roots']), stop=seed['stop']))
                more = []
                for root in seed['roots']:
                    if root['outcome'] != 'admitted':continue
                    p = root['point']
                    if self.covered(components, root['target'], p):continue
                    comp = dict(level=root['target'], segment=segment['speed_kmh'], knots=[p], discovered_at_kmh=speed,
                                provenance='envelope column (level not followed there)')
                    if self.s10 and root.get('rising'):comp['below'] = True
                    more.append(comp)
                if more:
                    self.eng.stats['s16_column_seeds'] = self.eng.stats.get('s16_column_seeds', 0)+len(more)
                    components.extend(more)
                    self.follow_new(more, components, segment)
                    self.drop_copies(more, components)

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef drop_copies(self, list new, list components):
        """S16: a component seeded by S16 (a level at a column no followed contour passed, or crossing a leaning edge)
        that, followed, turns out to be a copy of one already there: its span inside that one's (two speed tolerances
        wider) and three quarters of its knots within four curve tolerances of that one's interpolant (next to a model
        jump the S9 merge, which needs every knot on the line, keeps both: P-63A-10 SB at 3,000 m +10 m/s)."""
        cdef PchipCurve f
        cdef int k, hits
        cdef double v, n
        fresh = {id(c) for c in new}
        for comp in new:
            if not any(c is comp for c in components) or len(comp['knots']) < 2:continue
            ks = comp['knots']
            v0, v1 = self.eng.st[ks[0]].speed, self.eng.st[ks[-1]].speed
            for other in components:
                if id(other) in fresh or other['level'] != comp['level'] or other['segment'] != comp['segment']:continue
                if len(other['knots']) < 2 or (self.s10 and bool(other.get('below')) != bool(comp.get('below'))):continue
                w0, w1 = self.eng.st[other['knots'][0]].speed, self.eng.st[other['knots'][-1]].speed
                if not (w0-2.*self.speed_tol <= v0 and v1 <= w1+2.*self.speed_tol):continue
                f = self.interpolant(other['knots'])
                hits = 0
                for k in ks:
                    v = pmin(pmax(self.eng.st[k].speed, w0), w1)
                    n = sqrt(1.+pmax(f.eval(v), 0.))
                    if fabs(curve_turn(v, n)-self.eng.st[k].turn) <= 4.*self.turn_tol:hits += 1
                if hits >= .75*len(ks):
                    components[:] = [c for c in components if c is not comp]
                    self.eng.stats['s16_copies_dropped'] = self.eng.stats.get('s16_copies_dropped', 0)+1
                    break

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef follow_new(self, list new, list components, dict segment):
        """Follow the components ``new`` (already in ``components``) in speed."""
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
    cdef edge_seeds(self, list segments, list components):
        """S16: levels crossing a traced leaning edge (forward IAS or engine RPM) between its knots, where no followed
        contour of the level ends: past the speed where level flight reaches the limit no column needs to lie in the band
        above the edge (Bf 110 F-2 RB at 65% pitch: +10 m/s meets the engine RPM edge between its foot at 241.8 km/h and
        the first column past it at 255.6). At the crossing interpolated between two edge knots the level is solved, at
        the crossing's speed or up to two speed tolerances lower (inside the limit at that load), and followed both ways
        from there with its exact slope."""
        cdef int i, a, b, p, k, anchor
        cdef double pa, pb, f, v, n, w, t, vv
        for rows in self.edge_knots:
            if not rows or len(rows) < 2:continue
            sids = [self.state(r) for r in rows]
            for i in range(len(sids)-1):
                a = sids[i];b = sids[i+1]
                for L in self.levels:
                    pa = self.eng.st[a].ps-L;pb = self.eng.st[b].ps-L
                    if not (isfinite(pa) and isfinite(pb)) or pa*pb > 0. or pa == pb:continue
                    f = pa/(pa-pb)
                    v = self.eng.st[a].speed+f*(self.eng.st[b].speed-self.eng.st[a].speed)
                    w = (pow(self.eng.st[a].load, 2.)-1.)+f*((pow(self.eng.st[b].load, 2.)-1.)-(pow(self.eng.st[a].load, 2.)-1.))
                    n = sqrt(1.+pmax(w, 0.));t = curve_turn(v, n)
                    segment = next((s for s in segments if s['speed_kmh'][0]-1e-9 <= v <= s['speed_kmh'][1]+1e-9), None)
                    if segment is None or self.near_component(components, segment, L, v, t):continue
                    anchor = a if f <= .5 else b
                    attitude = [self.eng.st[a].sol[k]*(1.-f)+self.eng.st[b].sol[k]*f for k in range(5)]
                    p = -1
                    for k in range(5):
                        vv = v-.5*k*self.speed_tol
                        outcome, q, _ = self.direct_seed(vv, L, pmax(n, 1.+1e-6), [anchor], attitude, 'seed_discovery')
                        if outcome == 'admitted' and q is not None and self.eng.compatible(q, anchor):
                            p = q;break
                    if p < 0 or self.covered(components, L, p):continue
                    comp = dict(level=L, segment=segment['speed_kmh'], knots=[p], discovered_at_kmh=self.eng.st[p].speed,
                                provenance='leaning edge crossing')
                    sl, sr = self.knot_slope(p)
                    if isfinite(sl) or isfinite(sr):comp['w_slope'] = sl if isfinite(sl) else sr
                    self.eng.stats['s16_edge_seeds'] = self.eng.stats.get('s16_edge_seeds', 0)+1
                    components.append(comp)
                    self.follow_new([comp], components, segment)
                    self.drop_copies([comp], components)

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef bint near_component(self, list components, dict segment, double target, double v, double t) except -1:
        """S16: a followed component of the level passes (v, t): a knot within two speed tolerances in speed and half a
        deg/s in turn, or its interpolant within that turn at v inside its knots' span."""
        cdef PchipCurve f
        cdef double n
        for comp in components:
            if comp['level'] != target or comp['segment'] != segment['speed_kmh'] or not comp['knots']:continue
            ks = comp['knots']
            for q in ks:
                if fabs(self.eng.st[q].speed-v) <= 2.*self.speed_tol and fabs(self.eng.st[q].turn-t) <= .5:return True
            if len(ks) >= 2 and self.eng.st[ks[0]].speed <= v <= self.eng.st[ks[-1]].speed:
                f = self.interpolant(ks)
                n = sqrt(1.+pmax(f.eval(v), 0.))
                if fabs(curve_turn(v, n)-t) <= .5:return True
        return False

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef bint level_covered(self, list components, dict segment, double target, list pts) except -1:
        """S16: whether every crossing of the level ``target`` between the column samples ``pts`` (by load) lies on a
        followed component of that level: one whose knots span the column's speed with its interpolant's turn rate
        inside the crossing's turn interval (four curve tolerances wider), or a fold piece spanning that speed. A level
        met at level flight only (a 1 g crossing) counts as covered (the 1 g crossings are followed from level flight)."""
        cdef int i, a, b
        cdef double v, lo, hi, n, t
        cdef PchipCurve f
        if not pts:return True
        v = self.eng.st[pts[0]].speed
        brackets = []
        for i in range(len(pts)-1):
            a = pts[i];b = pts[i+1]
            if (self.eng.st[a].ps-target)*(self.eng.st[b].ps-target) > 0. or not self.eng.compatible(a, b):continue
            brackets.append((pmin(self.eng.st[a].turn, self.eng.st[b].turn), pmax(self.eng.st[a].turn, self.eng.st[b].turn)))
        for lo, hi in brackets:
            ok = False
            for comp in components:
                if comp['level'] != target or comp['segment'] != segment['speed_kmh'] or not comp['knots']:continue
                ks = comp['knots']
                fold = comp.get('fold')
                if fold is not None and fold.get('knots'):
                    fv = [self.eng.st[q].speed for q in fold['knots']]
                    if min(fv)-1e-9 <= v <= max(fv)+1e-9:
                        ok = True;break
                if not (self.eng.st[ks[0]].speed-1e-9 <= v <= self.eng.st[ks[-1]].speed+1e-9):continue
                if len(ks) == 1:
                    t = self.eng.st[ks[0]].turn
                else:
                    f = self.interpolant(ks)
                    n = sqrt(1.+pmax(f.eval(v), 0.))
                    t = curve_turn(v, n)
                if lo-4*self.turn_tol <= t <= hi+4*self.turn_tol:
                    ok = True;break
            if not ok:return False
        return True

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
    cdef bint path_coords(self, int k, double n, double* out) except -1:
        """S16: the propulsion coordinates (after the seven attitude and wing-history ones) of column k's level-flight
        alpha path at load ``n``: linear in load between its samples, held beyond them; out[0] its SEP there (linear in
        load). False where the layouts differ."""
        cdef int j, a, b, nx
        cdef double f = 0.
        pts = self.pcol[k]
        nx = self.eng.st[pts[0]].nx
        a = pts[0];b = pts[0]
        if n >= self.eng.st[pts[-1]].load:
            a = pts[-1];b = pts[-1]
        elif n > self.eng.st[pts[0]].load:
            for j in range(len(pts)-1):
                if self.eng.st[pts[j]].load <= n <= self.eng.st[pts[j+1]].load:
                    a = pts[j];b = pts[j+1]
                    if self.eng.st[b].load > self.eng.st[a].load:
                        f = (n-self.eng.st[a].load)/(self.eng.st[b].load-self.eng.st[a].load)
                    break
        if self.eng.st[a].nx != nx or self.eng.st[b].nx != nx:return False
        for j in range(7, nx):out[j] = self.eng.st[a].coords[j]*(1.-f)+self.eng.st[b].coords[j]*f
        out[0] = self.eng.st[a].ps*(1.-f)+self.eng.st[b].ps*f
        return True

    cdef bint branch_apart(self, const double* x, const double* y, int nx) noexcept:
        """S16: two propulsion coordinate sets on different propeller branches: some coordinate apart by more than
        twice S9_PROP_JUMP of its size (at least 1; twice for the interpolation between path samples and columns)."""
        cdef int j
        for j in range(7, nx):
            if fabs(x[j]-y[j]) > 2.*S9_PROP_JUMP*pmax(1., pmax(fabs(x[j]), fabs(y[j]))):return True
        return False

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int path_check(self, int p, int anchor) except -2:
        """S16 (Leo 2026-10-23: the chart follows the propeller state reached by pulling from steady level flight): 1
        where curve state ``p`` (solved from ``anchor``) lies on another propeller branch than the level-flight alpha
        path at its speed and load, 0 on it, -1 unknown. Decided on the path itself (path_bracket: its coordinates, and
        its SEP at p's load) where p jumped from its anchor's branch (prop_jump) or where the neighbouring verified
        columns' paths lie on different branches at p's load (the path switches between them); else by the columns'
        samples, linear in speed."""
        import bisect
        cdef double ra[NMAX]
        cdef double rb[NMAX]
        cdef double v, n, f
        cdef int i, j, nx, q, k, lo = -1, hi = -1
        cdef bint exact
        cdef TState* t
        if not self.pcol_v or p < 0:return -1
        t = &self.eng.st[p]
        if not t.has_coords or t.nx <= 7 or t.slide.n > 0:return -1
        v = t.speed;n = t.load;nx = t.nx
        i = bisect.bisect_left(self.pcol_v, v)
        # the columns' path at p's load: at a column its own samples; past the first or last column that one; between two
        # columns on the same branch there, linear in speed (between two that are not, the path switches: exact)
        exact = False
        k = -1
        if i < len(self.pcol_v) and fabs(self.pcol_v[i]-v) <= 1e-6:k = i
        elif i == 0:k = 0
        elif i == len(self.pcol_v):k = i-1
        if k >= 0:
            if self.eng.st[self.pcol[k][0]].nx != nx or not self.path_coords(k, n, ra):return -1
        else:
            if (self.eng.st[self.pcol[i-1][0]].nx != nx or self.eng.st[self.pcol[i][0]].nx != nx
                    or not self.path_coords(i-1, n, ra) or not self.path_coords(i, n, rb)):return -1
            if self.columns_apart(i-1, i, n, ra, rb, nx):
                exact = True
                self.eng.stats['s16_path_column_trigger'] = self.eng.stats.get('s16_path_column_trigger', 0)+1
            else:
                f = (v-self.pcol_v[i-1])/(self.pcol_v[i]-self.pcol_v[i-1])
                for j in range(7, nx):ra[j] = ra[j]*(1.-f)+rb[j]*f
        if not exact:
            if self.branch_apart(ra, t.coords, nx):return 1
            # a state off the columns' path by more than S9_PROP_JUMP but not twice that is decided on the path itself: two
            # branches can lie close in their coordinates, and part smoothly (Ki-21-I hei SB at 65% pitch: from 155.9 km/h
            # on, +10 m/s follows a branch whose governor drifts to 0.44 against the path's 0.36 at 161.5, SEP 0.6 m/s
            # apart at the same load)
            if not self.near_apart(ra, t.coords, nx):return 0
            self.eng.stats['s16_path_near_trigger'] = self.eng.stats.get('s16_path_near_trigger', 0)+1
        if i == 0 or i == len(self.pcol_v):k = min(i, len(self.pcol_v)-1)
        else:k = i-1 if v-self.pcol_v[i-1] <= self.pcol_v[i]-v else i
        self.eng.stats['s16_path_marches'] = self.eng.stats.get('s16_path_marches', 0)+1
        if not self.path_bracket(v, n, k, &lo, &hi):return -1
        # (the path switching branch between its two states either side of p's load: undecided)
        if self.branch_apart(self.eng.st[lo].coords, self.eng.st[hi].coords, nx):return -1
        # the path's state at p's load (fixed-load from the one below, on its branch; else its SEP linear in load between
        # the two): the two branches can lie close in their coordinates and apart in SEP (Hurricane Mk IV RB at 40%
        # pitch, 273.8 km/h, 2.53 g: governor 1.09 against 1.20, SEP 0 against 3.2 m/s)
        if hi != lo:
            q = self.fixed(v, n, lo, 'path_check')
            if q >= 0 and self.eng.st[q].converged and not self.prop_jump(lo, q):
                return 1 if (self.branch_apart(self.eng.st[q].coords, t.coords, nx)
                             or fabs(self.eng.st[q].ps-t.ps) > S16_PATH_SEP*self.allowance) else 0
        f = 0. if hi == lo else (n-self.eng.st[lo].load)/(self.eng.st[hi].load-self.eng.st[lo].load)
        q = lo if f <= .5 else hi
        return 1 if (self.branch_apart(self.eng.st[q].coords, t.coords, nx)
                     or fabs(self.eng.st[lo].ps+f*(self.eng.st[hi].ps-self.eng.st[lo].ps)-t.ps) > S16_PATH_SEP*self.allowance) else 0

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef bint step_jump(self, dict comp, int last, int point, int side) except -1:
        """S16: a continuation state ``point`` on another propeller branch than the contour's ``last`` knot: S13's
        prop_branch test on the piece, ``last`` continued through the knot before it (a drift of the coordinates along
        the contour is no switch); prop_jump where the contour has one knot."""
        knots = comp['knots']
        if len(knots) < 2:return self.prop_jump(last, point)
        prev = knots[-2] if side > 0 else knots[1]
        if side > 0:return self.prop_branch([prev, last, point], 1)
        return self.prop_branch([point, last, prev], 0)

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef bint columns_apart(self, int i, int j, double n, const double* ra, const double* rb, int nx) except -1:
        """S16: the paths of neighbouring columns i < j at load ``n`` (ra, rb) on different branches: some coordinate of
        each apart by more than twice S9_PROP_JUMP of its size from the other continued linearly through its own outer
        neighbouring column (i-1, j+1; held where there is none), so that a drift with speed is no switch."""
        cdef double rp[NMAX]
        cdef double rn[NMAX]
        cdef double vi = self.pcol_v[i], vj = self.pcol_v[j], ca, cb, tol
        cdef bint hp = False, hn = False
        cdef int k
        if i > 0 and self.eng.st[self.pcol[i-1][0]].nx == nx:hp = self.path_coords(i-1, n, rp)
        if j+1 < len(self.pcol_v) and self.eng.st[self.pcol[j+1][0]].nx == nx:hn = self.path_coords(j+1, n, rn)
        for k in range(7, nx):
            tol = 2.*S9_PROP_JUMP*pmax(1., pmax(fabs(ra[k]), fabs(rb[k])))
            ca = ra[k]
            if hp and vi > self.pcol_v[i-1]:ca += (ra[k]-rp[k])*(vj-vi)/(vi-self.pcol_v[i-1])
            cb = rb[k]
            if hn and self.pcol_v[j+1] > vj:cb += (rb[k]-rn[k])*(vi-vj)/(self.pcol_v[j+1]-vj)
            if fabs(rb[k]-ca) > tol and fabs(ra[k]-cb) > tol:return True
        return False

    cdef bint near_apart(self, const double* x, const double* y, int nx) noexcept:
        """S16: some propulsion coordinate apart by more than S9_PROP_JUMP of its size (at least 1)."""
        cdef int j
        for j in range(7, nx):
            if fabs(x[j]-y[j]) > S9_PROP_JUMP*pmax(1., pmax(fabs(x[j]), fabs(y[j]))):return True
        return False

    cdef bint prop_jump(self, int a, int b) noexcept:
        """S16: NMarch.prop_jump on curve states: a propulsion coordinate of ``b`` apart from ``a``'s by more than
        S9_PROP_JUMP of its size (at least 1); False where the layouts differ or either state slides."""
        cdef TState* s = &self.eng.st[a]
        cdef TState* t = &self.eng.st[b]
        cdef int i
        if not (s.has_coords and t.has_coords) or s.nx != t.nx or s.nx <= 7 or s.slide.n > 0 or t.slide.n > 0:return False
        for i in range(7, s.nx):
            if fabs(t.coords[i]-s.coords[i]) > S9_PROP_JUMP*pmax(1., fabs(s.coords[i])):return True
        return False

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef bint path_bracket(self, double v, double n, int k, int* lo, int* hi) except -1:
        """S16: the level-flight alpha path at speed ``v`` around load ``n``: level flight at v from column k's first
        sample (the other neighbouring column's where that lands on another branch), then prescribed-alpha corrections at
        the alphas of that column's own march samples and in steps of S16_PATH_DA past them (path_alpha: each on the
        branch of the one before), until the load passes n: ``lo`` and ``hi`` the path states either side of n (both
        ``lo`` where n is level flight's load or below). False where a solve does not close or the path ends before n."""
        cdef int cur = -1, r, c, kk, it
        cdef double a1
        for kk in (k, k+1 if k+1 < len(self.pcol_v) and self.pcol_v[k] < v else k-1):
            if kk < 0 or kk >= len(self.pcol_v):continue
            c = self.pcol[kk][0]
            r = self.fixed(v, self.eng.st[c].load, c, 'path_check')
            if r >= 0 and self.eng.st[r].converged and not self.branch_apart(self.eng.st[c].coords, self.eng.st[r].coords,
                                                                              self.eng.st[r].nx):
                cur = r;break
        if cur < 0:return False
        if n <= self.eng.st[cur].load:
            lo[0] = cur;hi[0] = cur
            return True
        # the alphas of column k's own march samples (the march's adaptive steps), then steps of S16_PATH_DA past them;
        # a step whose state lands on another branch is taken again in 2, 4 and 8 sub-steps (one none keeps is the
        # path's own switch)
        alphas = [self.eng.st[q].alpha for q in self.pcol[kk] if self.eng.st[q].alpha > self.eng.st[cur].alpha+1e-6]
        for it in range(200):
            if it < len(alphas):a1 = alphas[it]
            else:a1 = self.eng.st[cur].alpha+S16_PATH_DA
            r = self.path_alpha(cur, v, a1)
            if r < 0 or not self.eng.st[r].converged or self.eng.st[r].load <= self.eng.st[cur].load:return False
            if self.eng.st[r].load >= n:
                lo[0] = cur;hi[0] = r
                return True
            if not (self.eng.st[r].valid or self.balanced(r)):return False
            cur = r
        return False

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int path_alpha(self, int cur, double v, double a1) except -2:
        """S16: the path step from ``cur`` to alpha ``a1`` at speed ``v``: one prescribed-alpha correction, or 2, 4 or 8
        sub-steps each on the branch of the one before where the single step lands on another branch (the single step
        where none keeps it: a switch of the path itself). -1 where a correction does not close."""
        cdef int r, n, j, c, q
        cdef double a0 = self.eng.st[cur].alpha
        r = self.alpha_correct(cur, v, a1)
        if r >= 0 and self.eng.st[r].converged and not self.prop_jump(cur, r):return r
        for n in (2, 4, 8):
            c = cur
            for j in range(1, n+1):
                q = self.alpha_correct(c, v, a0+(a1-a0)*j/n)
                if q < 0 or not self.eng.st[q].converged or self.prop_jump(c, q):
                    c = -1;break
                c = q
            if c >= 0:return c
        return r if r >= 0 and self.eng.st[r].converged else -1

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef double half_w(self, double v) except? -1.:
        """S16: half of w = n^2-1 at the envelope boundary at speed ``v`` (the verified columns' limiting loads, linear
        between the two either side), at least S9_END_DW; 0 outside the columns' span."""
        import bisect
        cdef int i
        cdef double n
        if not self.env_v:return 0.
        i = bisect.bisect_left(self.env_v, v)
        if i < len(self.env_v) and fabs(self.env_v[i]-v) <= 1e-9:n = self.env_n[i]
        elif 0 < i < len(self.env_v):
            n = self.env_n[i-1]+(self.env_n[i]-self.env_n[i-1])*(v-self.env_v[i-1])/(self.env_v[i]-self.env_v[i-1])
        else:return 0.
        return pmax(S9_END_DW, .5*(n*n-1.))

    cdef object one_g_start(self, double target, int above, int below, double va, double vb, dict segment, list components,
                            list unresolved_targets):
        """A contour of ``target`` met at level flight between the 1 g states ``above`` (SEP over it) and ``below`` (speeds
        va..vb): located (locate_one_g) and followed from there, unless a followed contour of it already ends at 1 g in
        the span (or, S10, a fold piece runs down to 1 g there)."""
        cdef int crossing
        explained = any(c['level'] == target and any(
            e and e.get('kind') == 'one_g' and va-self.speed_tol <= (
                e['speed_kmh'] if not isinstance(e['speed_kmh'], list) else e['speed_kmh'][0]) <= vb+self.speed_tol
            for e in (c.get('start'), c.get('end'))) for c in components)
        if self.s10 and not explained:
            # S10: or a fold piece that runs down to 1 g there (NCurves.fold_turn)
            explained = any(c['level'] == target and c.get('fold') is not None and any(
                self.eng.st[q].load == 1. and va-self.speed_tol <= self.eng.st[q].speed <= vb+self.speed_tol
                for q in c['fold']['knots']) for c in components)
        if explained:return None
        got = self.locate_one_g(target, above, below)
        if got is None:
            unresolved_targets.append(dict(target_mps=target, speed_kmh=[va, vb],
                reason='1 g crossing bracketed but not localized'))
            return None
        crossing = got
        if self.covered(components, target, crossing):return None
        if (self.s16 and fabs(self.eng.st[crossing].ps-target) > self.allowance
                and self.prop_branch([self.root_a, self.root_b], 0)):
            # S16: the last bracket of one_g_root straddles a propeller branch switch. Where one of its two branches
            # meets the level on its own nearby (A6M2-N RB at 65% pitch: the lower branch's level flight crosses +10 m/s
            # at 162.7 km/h and ends at 163.5, the other runs at 14 m/s from 156.6 km/h on) that is the crossing, if it
            # lies on the chart's level-flight path there (path_check); else level flight's SEP jumps across the level
            # (Pyorremyrsky RB at 40% pitch, 7.7 to 18.3 m/s at 216.3-216.5 km/h across +10 m/s): no contour of it meets
            # 1 g there (a steep SEP along level flight without a switch is a crossing: jets near their top speed)
            got = self.branch_crossing(target, self.root_a, self.root_b, va, vb)
            if got is None:
                unresolved_targets.append(dict(target_mps=target, speed_kmh=self.eng.st[crossing].speed,
                    reason='level-flight SEP jumps across the level'))
                self.eng.stats['s16_one_g_jumps'] = self.eng.stats.get('s16_one_g_jumps', 0)+1
                return None
            crossing = got[0];above = got[1];below = got[2]
            if self.covered(components, target, crossing):return None
            self.eng.stats['s16_one_g_branch_crossings'] = self.eng.stats.get('s16_one_g_branch_crossings', 0)+1
        if self.s9:self.end_slope(crossing)
        comp = self.from_one_g(crossing, above, below, target, segment)
        if comp is None:
            unresolved_targets.append(dict(target_mps=target, speed_kmh=self.eng.st[crossing].speed,
                reason='1 g crossing found; level slope unavailable'))
            return None
        components.append(comp)
        self.follow(comp, segment)
        if self.s10:self.follow_pending(segment, components)
        return comp

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef object branch_crossing(self, double target, int ra, int rb, double va, double vb):
        """S16: the 1 g crossing of ``target`` on the branch of one end of a bracket that straddles a propeller branch
        switch (ra, rb): each end's level flight continued in steps of S16_BRANCH_DV towards and past the other end (at
        most S16_BRANCH_STEPS, within va..vb, on its own branch) until its SEP crosses the level, then one_g_root on that
        branch alone. (crossing, above, below) of a crossing within the SEP allowance on the chart's level-flight path,
        or None."""
        cdef int e, other, prev, q, k, r, hi_, lo_
        cdef double v, d, f0
        for e, other in ((ra, rb), (rb, ra)):
            d = 1. if self.eng.st[other].speed > self.eng.st[e].speed else -1.
            f0 = self.eng.st[e].ps-target
            prev = e;v = self.eng.st[e].speed
            for k in range(S16_BRANCH_STEPS):
                v += d*S16_BRANCH_DV
                if not (pmin(va, vb)-1e-9 <= v <= pmax(va, vb)+1e-9):break
                q = self.fixed(v, 1., prev, 'one_g_discovery')
                if q < 0 or not self.eng.st[q].converged or not self.eng.st[q].valid or self.prop_jump(prev, q):break
                if (self.eng.st[q].ps-target)*f0 < 0.:
                    if self.eng.st[prev].ps > target:
                        hi_ = prev;lo_ = q
                    else:
                        hi_ = q;lo_ = prev
                    got = self.one_g_root(target, hi_, lo_, 'one_g_discovery')
                    if got is None:break
                    r = got
                    if (fabs(self.eng.st[r].ps-target) > self.allowance or self.prop_jump(e, r)
                            or self.path_check(r, -1) == 1):break
                    return (r, hi_, lo_)
                prev = q
        return None

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int one_g_bubbles(self, int pa, int pb, double va, double vb, dict segment, list components,
                           list unresolved_targets) except -1:
        """S16: level flight's SEP with an extremum between two neighbouring level flights (pa, pb, on one branch) that
        reaches a level neither of them does: a contour bubble against 1 g between the columns (P-43A-1 RB at 40% pitch:
        -0.29 and -0.78 m/s at 430 and 455 km/h, +0.25 at 440). Looked for only where a level lies within S16_BUBBLE_SEP
        beyond both: level flight at the middle, then parabolic steps to the extremum (at most S16_BUBBLE_STEPS) until
        one crosses the level; each side's crossing then started as any other (one_g_start)."""
        cdef double fa = self.eng.st[pa].ps, fb = self.eng.st[pb].ps, hi = pmax(fa, fb), lo = pmin(fa, fb), x, den
        cdef double x0, x1, x2, y0, y1, y2, sgn, vm
        cdef int m, q, k, best, j
        if self.prop_jump(pa, pb) or vb-va <= 2.*self.speed_tol:return 0
        ups = [t for t in self.levels if hi < t <= hi+S16_BUBBLE_SEP]
        downs = [t for t in self.levels if lo-S16_BUBBLE_SEP <= t < lo]
        if not ups and not downs:return 0
        vm = .5*(va+vb)
        m = self.fixed(vm, 1., pa, 'one_g_discovery')
        if m < 0 or not self.eng.st[m].converged or not self.eng.st[m].valid or self.prop_jump(pa, m):return 0
        self.eng.stats['s16_bubble_probes'] = self.eng.stats.get('s16_bubble_probes', 0)+1
        for targets, sgn in ((ups, 1.), (downs, -1.)):
            if not targets or not sgn*(self.eng.st[m].ps-(hi if sgn > 0. else lo)) > 0.:continue
            pts = [pa, m, pb];best = m
            for k in range(S16_BUBBLE_STEPS):
                if all(sgn*(self.eng.st[best].ps-t) > 0. for t in targets):break
                pts.sort(key=lambda r: self.eng.st[r].speed)
                j = max(range(len(pts)), key=lambda i: sgn*self.eng.st[pts[i]].ps)
                if j == 0 or j == len(pts)-1:break
                x0 = self.eng.st[pts[j-1]].speed;x1 = self.eng.st[pts[j]].speed;x2 = self.eng.st[pts[j+1]].speed
                y0 = self.eng.st[pts[j-1]].ps;y1 = self.eng.st[pts[j]].ps;y2 = self.eng.st[pts[j+1]].ps
                den = (x1-x0)*(y1-y2)-(x1-x2)*(y1-y0)
                if den == 0.:break
                x = x1-.5*((x1-x0)*(x1-x0)*(y1-y2)-(x1-x2)*(x1-x2)*(y1-y0))/den
                if not x0 < x < x2 or pmin(fabs(x-x0), pmin(fabs(x-x1), fabs(x-x2))) < .25*self.speed_tol:break
                q = self.fixed(x, 1., pts[j], 'one_g_discovery')
                if q < 0 or not self.eng.st[q].converged or not self.eng.st[q].valid or self.prop_jump(pts[j], q):break
                pts.append(q)
                if sgn*self.eng.st[q].ps > sgn*self.eng.st[best].ps:best = q
            for t in targets:
                if not sgn*(self.eng.st[best].ps-t) > 0.:continue
                self.eng.stats['s16_bubbles'] = self.eng.stats.get('s16_bubbles', 0)+1
                for side in (pa, pb):
                    above, below = (best, side) if sgn > 0. else (side, best)
                    v0, v1 = sorted((self.eng.st[best].speed, self.eng.st[side].speed))
                    self.one_g_start(t, above, below, v0, v1, segment, components, unresolved_targets)
        return 0

    cdef bint rising_at(self, int one) except -1:
        """S16: SEP rising with load from the level flight ``one`` (a level-turn solve at w = S9_END_DW, as S10's 'below'
        test), on its branch."""
        cdef int two = self.fixed(self.eng.st[one].speed, sqrt(1.+S9_END_DW), one, 'endpoint')
        return (self.eng.st[two].valid and self.eng.compatible(two, one) and not self.prop_jump(one, two)
                and self.eng.st[two].ps > self.eng.st[one].ps)

    cdef bint slope_state(self, int two, int crossing) except -1:
        """An admitted state compatible with the 1 g crossing (S16: on its propeller branch) for from_one_g's slope."""
        return (self.eng.st[two].valid and self.eng.compatible(two, crossing)
                and not (self.s16 and self.prop_jump(crossing, two)))

    cdef object from_one_g(self, int crossing, int above, int below, double target, dict segment):
        cdef double v = self.eng.st[crossing].speed, dv, d1, dw, dw2
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
        dw2 = 3.
        # (S16: on the crossing's own propeller branch)
        if not self.slope_state(two, crossing) and self.s16:
            # S16: where the envelope ends below 2 g at this speed (flying boats and bombers near their top speed: H6K4
            # RB -10 m/s at 403.5 km/h, the control limit at 1.9 g) the level set's slope from halfway to the boundary
            dw2 = self.half_w(v)
            if dw2 > 0.:two = self.fixed(v, sqrt(1.+dw2), crossing, 'one_g_discovery')
            if not self.slope_state(two, crossing):
                # (where that lands on another propeller branch or the boundary dips between the columns: A6M2-N RB at
                # 65% pitch, 163 km/h, 1.1 g between 1.9 g columns; the end's own step, as end_slope)
                dw2 = S9_END_DW
                two = self.fixed(v, sqrt(1.+dw2), crossing, 'one_g_discovery')
        if not self.slope_state(two, crossing):return None
        dw = (self.eng.st[two].ps-self.eng.st[crossing].ps)/dw2
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
        self.env_lo = [float((c.get('lower_edge') or {}).get('load_g', 1.)) for c in columns if c.get('boundary')]
        self.pcol_v = [];self.pcol = []
        if self.s16 and self.eng.is_prop:
            # S16: the level-flight alpha paths of the verified columns (path_check)
            for c in columns:
                if not c.get('boundary'):continue
                pts, _ = self.column_samples(c)
                if pts and self.eng.st[pts[0]].nx > 7:
                    self.pcol_v.append(float(c['speed_kmh']));self.pcol.append(pts)
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
                                above, below = (pa, pb) if self.eng.st[pa].ps > target else (pb, pa)
                                self.one_g_start(target, above, below, va, vb, segment, components, unresolved_targets)
                            if self.s16:self.one_g_bubbles(pa, pb, va, vb, segment, components, unresolved_targets)
                    if self.s16 and self.edge_knots:self.edge_seeds(segments, components)
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
