# S5: the SEP curve solver of the march chart (em_march_curves.MarchCurveSolver in
# envelope-column mode) on native trial states, line by line. Every direct SEP solve is
# the native correction and lean finish; knots stay native states and the component
# report is built from their fields (compact_state). The work ledger is the Python
# Ledger, so the curve work record is unchanged.

STATE_FIELDS = ('speed_kmh', 'load_g', 'turn_dps', 'ps_mps', 'alpha_deg', 'bank_deg', 'solution',
                'stationary_coordinates', 'stationary_coordinate_names', 'stationary_request_indices',
                'sideslip_attitude_deg', 'sideslip_deg', 'stationary_governor_mode', 'model_experiments',
                'mach_curve_mode', 'mach_curve_revision', 'roll_leveling_branch', 'valid', 'reasons',
                'force_error_g', 'angular_error_rad_s2', 'history_error', 'converged')


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
    cdef double max_step, predictor_shrink, predictor_grow, check_band_fraction, column_min_spacing, check_flag
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
        self.check_flag = getattr(p, 'check_flag_dps', INFINITY)
        self.column_min_spacing = p.column_min_spacing_kmh;self.refinement_passes = p.refinement_passes
        self.ladder_loads = list(p.ladder_loads)
        if p.exhaustive_retry:raise NativeMarchUnsupported('exhaustive curve retries')
        self.check = None
        self.column_ids = {}

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
        elif not self.eng.st[p].valid and self.balanced(p):outcome, reason = 'rejected', self.eng.reason_list(p)
        elif not self.eng.st[p].valid:outcome, reason = 'unresolved', ['unclosed']+self.eng.reason_list(p)
        else:outcome, reason = 'unresolved', 'SEP constraint did not close'
        t0 = s5_now()
        self.ledger.charge(phase, before, outcome, kind='direct', speed_kmh=speed, target_mps=target,
                           seed_load_g=load, reason=reason)
        self.eng.t_ledger += s5_now()-t0
        return outcome, (None if p < 0 else p), reason

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
                stop = dict(load_g=n, kind='rejected' if self.balanced(p) else 'unresolved', reasons=self.eng.reason_list(p))
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
        cdef double f_in = 0.
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
            if self.eng.st[p].ps > target:
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
            if w <= 0. and self.eng.st[last].load == 1. and abs(self.eng.st[last].ps-target) <= self.allowance:
                return dict(kind='one_g', speed_kmh=self.eng.st[last].speed, state=last)
            if w <= 0.:
                one = self.fixed(speed, 1., last, 'endpoint')
                if self.eng.st[one].valid and self.eng.st[one].ps < target:
                    end = self.one_g_endpoint(comp, target, last, one, side)
                    if end.get('state') is not None and end['kind'] == 'one_g':
                        self.add_knot(comp, end['state'], side)
                    return end
                w = max(w, (pow(self.eng.st[last].load, 2.)-1.)*.25)
            n = py_sqrt(1.+max(w, 1e-6))
            outcome, point, reason = self.direct_seed(speed, target, n, [last], attitude, 'continuation')
            if outcome == 'admitted' and self.eng.compatible(point, last):
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
                return self.classify_end(comp, last, failure, side)
            step = .5*abs(speed-self.eng.st[last].speed)

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef object classify_end(self, dict comp, int last, dict failure, int side):
        cdef double target = comp['level'], speed = failure['speed_kmh']
        cdef int one
        one = self.fixed(speed, 1., last, 'endpoint')
        if self.eng.st[one].valid and self.eng.st[one].ps < target-self.allowance:
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
    cdef add_knot(self, dict comp, int point, int side):
        for k in comp['knots']:
            if abs(self.eng.st[k].speed-self.eng.st[point].speed) <= 1e-9:return
        if side > 0:comp['knots'].append(point)
        else:comp['knots'].insert(0, point)

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef follow(self, dict comp, dict segment):
        for side, key in ((1, 'end'), (-1, 'start')):
            while True:
                end = self.march(comp, segment, side)
                if end is not None:break
            comp[key] = end

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
    cdef check_intervals(self, dict comp):
        """MarchCurveSolver.check_intervals: withheld exact checks, kink-placed."""
        cdef double target = comp['level'], speed, w, n, t, w0, interp_turn, turn_error
        cdef int a, b, i, p
        cdef PchipCurve f
        checks = comp.setdefault('checks', [])
        unresolved = comp.setdefault('unresolved_intervals', [])
        for _ in range(self.refinement_passes):
            knots = comp['knots'];f = self.interpolant(knots)
            if f is None:return
            done = {(c['interval_kmh'][0], c['interval_kmh'][1]) for c in checks if c['passed']}
            blocked = {tuple(u['interval_kmh']) for u in unresolved}
            additions = []
            for i in range(len(knots)-1):
                a = knots[i];b = knots[i+1]
                key = (self.eng.st[a].speed, self.eng.st[b].speed)
                if key in done or key in blocked:continue
                if self.eng.st[b].speed-self.eng.st[a].speed <= self.speed_tol:continue
                speed = self.check_speed(self.eng.st[a].speed, self.eng.st[b].speed)
                w = f.eval(speed);n = sqrt(1.+pmax(w, 0.))
                t = (speed-self.eng.st[a].speed)/(self.eng.st[b].speed-self.eng.st[a].speed)
                attitude = [self.eng.st[a].sol[k]*(1-t)+self.eng.st[b].sol[k]*t for k in range(5)]
                outcome, truth, reason = self.direct_seed(speed, target, n, [a, b], attitude, 'interpolation_checks')
                if outcome == 'admitted' and not (self.eng.compatible(truth, a) and self.eng.compatible(truth, b)):
                    outcome, reason = 'incompatible', 'midpoint branch differs from knots'
                if outcome != 'admitted':
                    retry = None
                    for anchor in (a, b):
                        w0 = pow(self.eng.st[anchor].load, 2.)-1.
                        retry_outcome, retry_p, _ = self.direct_seed(speed, target, sqrt(1.+pmax(w0, 1e-6)), [anchor], None,
                                                                     'interpolation_checks')
                        if retry_outcome == 'admitted' and self.eng.compatible(retry_p, a) and self.eng.compatible(retry_p, b):
                            retry = retry_p;break
                    if retry is not None:
                        additions.append(retry);continue
                    unresolved.append(dict(interval_kmh=list(key), reason=reason, outcome=outcome))
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
                if not passed:
                    p = self.fixed(speed, pmax(1., n), truth, 'interpolation_checks')
                    record.update(interpolated_point_valid=self.eng.st[p].valid, sep_error_mps=abs(self.eng.st[p].ps-target),
                                  interpolated_point_reasons=self.eng.reason_list(p))
                    passed = bool(self.eng.st[p].valid and abs(self.eng.st[p].ps-target) <= self.check_band_fraction*self.band)
                    if passed and turn_error > self.check_flag:
                        passed = False;record['flag_exceeded'] = True
                record['passed'] = passed
                checks.append(record)
                if not passed:additions.append(truth)
            if not additions:return
            for p in additions:comp['knots'].append(p)
            comp['knots'].sort(key=lambda q: self.eng.st[q].speed)
            speeds = {self.eng.st[p].speed for p in additions}
            for c in checks:
                if c['holdout']['speed_kmh'] in speeds:c['became_knot'] = True
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
                choices = [c for c in open_.get(target, []) if c not in taken and self.eng.compatible(c['knots'][-1], p)]
                if choices:
                    comp = min(choices, key=lambda c: abs(self.eng.st[c['knots'][-1]].turn-self.eng.st[p].turn))
                    self.add_knot(comp, p, 1)
                else:
                    comp = dict(level=target, segment=segment['speed_kmh'], knots=[p], discovered_at_kmh=speed,
                                provenance='envelope column')
                    new.append(comp)
                current.setdefault(target, []).append(comp)
            open_ = current
        components.extend(new)
        for comp in new:self.follow(comp, segment)

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
        self.ladders = [];self.ones = {}
        self.knot_speeds = self.py.model_knot_speeds(float(self.py.config['speed_min_kmh']), float(self.py.config['speed_max_kmh']))
        columns = sorted(columns, key=lambda c: c['speed_kmh'])
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
                                if explained:continue
                                above, below = (pa, pb) if self.eng.st[pa].ps > target else (pb, pa)
                                crossing = self.locate_one_g(target, above, below)
                                if crossing is None:
                                    unresolved_targets.append(dict(target_mps=target, speed_kmh=[va, vb],
                                        reason='1 g crossing bracketed but not localized'))
                                    continue
                                if self.covered(components, target, crossing):continue
                                comp = self.from_one_g(crossing, above, below, target, segment)
                                if comp is None:
                                    unresolved_targets.append(dict(target_mps=target, speed_kmh=self.eng.st[crossing].speed,
                                        reason='1 g crossing found; level slope unavailable'))
                                    continue
                                components.append(comp)
                                self.follow(comp, segment)
                    for comp in components:
                        self.check_intervals(comp)
                    junctions = self.seam_junctions(segments, components)
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
        return dict(level=comp['level'], segment_kmh=comp['segment'], discovered_at_kmh=comp['discovered_at_kmh'],
                    provenance=comp.get('provenance'),
                    knots=[self.compact(p) for p in comp['knots']], start=end(comp.get('start')), end=end(comp.get('end')),
                    checks=comp.get('checks', []), unresolved_intervals=comp.get('unresolved_intervals', []),
                    refinement_exhausted=comp.get('refinement_exhausted', False))


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
