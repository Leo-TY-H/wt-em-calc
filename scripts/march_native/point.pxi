# S5: the production fixed-load point solve (em_stationary.solve for the march chart's
# solvers: zero sideslip, ordinary starts, exhaustive=False, quick=False) on native trial
# states, and em_sampling.solve_level on it. JointProblem, its propulsion seed correction
# (seed_propulsion), the joint correction (JointProblem.correct, native since S2) and the
# finish run natively; the chart row of a solve (BaselineSolver.sample of the candidate
# record) is built only when the state is kept. The bounded propeller branch recovery
# (em_prop_branch_recovery) runs natively too (recovery.pxi) on the same corrected state.

DEF PROPCACHE = 32


cdef class PropBlock(NativeCorrector):
    """JointProblem.seed_propulsion: the propulsion block alone (aircraft coordinates fixed)."""
    cdef CurveProblem P
    cdef PointProblem owner
    cdef double q0[NMAX]
    cdef int off
    cdef double rsample[MAXPATH]

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int evaluate(self, const double* y, Sample* out) except -1:
        cdef double q[NMAX]
        cdef int i, code
        for i in range(self.owner.nx):q[i] = self.q0[i]
        for i in range(self.n):q[self.off+i] = y[i]
        code = self.owner.propulsion_rows(q, self.rsample)
        if code:return UNEVALUABLE
        for i in range(self.m):out.r[i] = self.rsample[i]
        for i in range(BMAX):out.br[i] = 0
        out.slot = -1
        return OK


cdef class PointProblem:
    """One em_stationary.JointProblem (speed, load, attitude, request branch) of a point solve."""
    cdef MarchEngine eng
    cdef public double speed_kmh, load
    cdef public int nx, nprows
    cdef public long calls
    cdef double initial[NMAX]
    cdef int req[NP]
    cdef public int nreq
    # JointProblem.prop_cache: propulsion samples of this problem (engine_frames are charged on a miss)
    cdef int ncache
    cdef double cache_key[PROPCACHE][MAXPATH+2]

    @cython.boundscheck(True)
    @cython.wraparound(True)
    def __init__(self, MarchEngine eng, double speed_kmh, double load):
        self.eng = eng;self.speed_kmh = speed_kmh;self.load = load
        self.nx = eng.layout_nx;self.nprows = eng.P.nprows;self.calls = 0;self.ncache = 0;self.nreq = 0

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int propulsion_rows(self, const double* q, double* inner) except -1:
        """JointProblem.propulsion_at(q)[0] (the inner rows), charging an engine frame per new sample."""
        cdef CurveProblem P = self.eng.P
        cdef PropPrep* prep
        cdef PropSample ps
        cdef double qq[MAXPATH]
        cdef int i, k, code
        cdef bint same
        prep = P.prepared(self.load, q[0], q[1])
        P.pm.use_slide(NULL)
        code = P.pm.propulsion_q(prep, &q[7], self.req, self.nreq, inner, &ps, qq)
        if code:return 1
        for k in range(self.ncache):
            same = self.cache_key[k][0] == q[0] and self.cache_key[k][1] == q[1]
            for i in range(P.pm.npaths):
                if self.cache_key[k][2+i] != qq[i]:same = False
            if same:return 0
        if self.ncache >= PROPCACHE:self.ncache = 0
        self.cache_key[self.ncache][0] = q[0];self.cache_key[self.ncache][1] = q[1]
        for i in range(P.pm.npaths):self.cache_key[self.ncache][2+i] = qq[i]
        self.ncache += 1
        charge(K_FRAMES, 1)
        return 0

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int warm(self, const double* q) except -1:
        """propulsion_at(q) into the sample cache without a charge (a fresh JointProblem warmed in a
        throwaway work record): the corrected state's own propulsion sample."""
        global g_sink
        cdef long scratch[NCOUNT]
        cdef long* sink = g_sink
        cdef double inner[MAXPATH]
        g_sink = scratch
        try:
            if self.propulsion_rows(q, inner):
                from em_stationary import BranchUnavailable
                raise BranchUnavailable('Governed request branch changed or is inconsistent')
        finally:
            g_sink = sink
        return 0


cdef class PointSolver:
    """em_stationary.solve on native trial states (the march chart's BaselineSolver.solve)."""
    cdef MarchEngine eng
    cdef public object solver, ST
    cdef public list solves
    cdef double steps[NMAX]
    cdef double scales[NMAX]
    cdef double limits[NMAX]
    cdef double rscales[NMAX]
    cdef bint dep[NMAX*NMAX]
    cdef public dict stats
    cdef public bint python_recovery

    @cython.boundscheck(True)
    @cython.wraparound(True)
    def __init__(self, MarchEngine eng):
        import em_stationary as ST
        cdef int i, k, nx
        self.eng = eng;self.solver = eng.solver;self.ST = ST
        nx = eng.layout_nx
        cdef double[7] steps7 = [.002, .002, .0002, .0002, .0002, .002, .002]
        cdef double[7] scales7 = [10., 30., .2, .2, .2, 10., 10.]
        cdef double[7] limits7 = [8., 20., .4, .4, .4, 8., 8.]
        cdef double[7] rs7 = [2e-4, 2e-4, 5e-4, 5e-4, 5e-4, 2e-5, 2e-5]
        from prop_equilibrium import RESIDUAL_TOLERANCE
        for i in range(nx):
            if i < 7:
                self.steps[i] = steps7[i];self.scales[i] = scales7[i];self.limits[i] = limits7[i];self.rscales[i] = rs7[i]
            else:
                self.steps[i] = .00001;self.scales[i] = 1.;self.limits[i] = .3;self.rscales[i] = RESIDUAL_TOLERANCE
        for i in range(nx):
            for k in range(nx):
                self.dep[i*nx+k] = not (i >= 7 and 2 <= k < 7)
        self.solves = []
        self.stats = dict(solves=0, python_recoveries=0)
        import os
        self.python_recovery = os.environ.get('WT_NATIVE_RECOVERY', '1') == '0'
        # BaselineSolver.core() creates the shared seed and predictor caches
        self.solver.core()

    # ------------------------------------------------------------------ JointProblem pieces
    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int problem_initial(self, PointProblem pr, list attitude) except -1:
        """JointProblem.initial (clipped) for ``attitude``."""
        cdef int i
        cdef double v
        for i in range(5):pr.initial[i] = attitude[i]
        pr.initial[5] = attitude[0];pr.initial[6] = attitude[0]
        for i in range(7, pr.nx):pr.initial[i] = self.eng.cc.l_initial[i]
        for i in range(pr.nx):
            v = pr.initial[i]
            pr.initial[i] = pmin_np(pmax_np(v, self.eng.cc.l_lower[i]), self.eng.cc.l_upper[i])
        return 0

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef list branches(self, PointProblem pr, saved, list attitude):
        """The governed request branches at this attitude, the saved seed's branch first."""
        import itertools
        from numbers import Integral
        cdef PropPrep* prep
        cdef int s
        if not self.eng.is_prop:return [()]
        self.eng.P.speed_kmh = pr.speed_kmh;self.eng.P.nprep = 0;self.eng.P.prep_next = 0
        prep = self.eng.P.prepared(pr.load, attitude[0], attitude[1])
        ranges = []
        for s in range(self.eng.fin.pm.nt):
            ranges.append(range(prep.ncand[s]) if prep.governed_request[s] else [0])
        out = list(itertools.product(*ranges))
        preferred = tuple(saved.get('stationary_request_indices', ())) if isinstance(saved, dict) else ()
        if all(isinstance(index, Integral) and not isinstance(index, bool) for index in preferred) and preferred in out:
            preferred = tuple(map(int, preferred))
            out.remove(preferred);out.insert(0, preferred)
        return out

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int seed_propulsion(self, PointProblem pr, double* seed) except -1:
        """JointProblem.seed_propulsion: close the propulsion block once for a cold start."""
        cdef PropBlock pb = PropBlock()
        cdef Result res
        cdef int i, off = 7, n = pr.nx-7
        cdef Predictor pred
        self.eng.P.speed_kmh = pr.speed_kmh;self.eng.P.nprep = 0;self.eng.P.prep_next = 0
        pb.P = self.eng.P;pb.owner = pr;pb.off = off
        for i in range(pr.nx):pb.q0[i] = seed[i]
        pb.n = n;pb.m = pr.nprows
        for i in range(n):
            pb.lower[i] = self.eng.cc.l_lower[off+i];pb.upper[i] = self.eng.cc.l_upper[off+i]
            pb.steps[i] = self.steps[off+i];pb.scale[i] = self.scales[off+i];pb.step_limit[i] = self.limits[off+i]
            pb.rscale[i] = self.rscales[off+i]
        pb.has_project = False;pb.has_dep = False
        pb.evaluations = pb.derivative_samples = pb.nominal_comparisons = pb.nominal_restorations = pb.branch_secants = 0
        pred.valid = False
        pb.solve(&seed[off], 20, 0, &pred, False, &res)
        charge(K_DERIV, pb.derivative_samples);charge(K_NOMCMP, pb.nominal_comparisons)
        charge(K_NOMRES, pb.nominal_restorations);charge(K_SECANT, pb.branch_secants)
        if res.termination == -1:
            from em_stationary import BranchUnavailable
            raise BranchUnavailable('Governed request branch changed or is inconsistent')
        if res.success:
            for i in range(n):seed[off+i] = res.x[i]
        return 0

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef tuple correct(self, PointProblem pr, double* x0, int max_nfev, bint warm_start):
        """JointProblem.correct(initial, offset=0) natively: (slot, result dict, x) on the curve problem."""
        import numpy as np
        cdef CurveProblem P = self.eng.P
        cdef int i, nx = pr.nx, slot
        cdef Result res
        cdef Predictor pred
        cdef NativePredictor np_pred
        cdef double y[NMAX]
        solver = self.solver
        cache = getattr(solver, '_stationary_predictors', None)
        request = tuple(pr.req[i] for i in range(pr.nreq))
        key = (None if cache is None else (id(solver.engine), pr.speed_kmh/3.6, solver.sideslip_attitude_deg,
               tuple(self.eng.layout_names), request, 0))
        previous = cache.get(key) if cache is not None else None
        if previous is not None and warm_start:
            anchor = previous['coordinates']
            if len(anchor) == nx:
                v = 0.
                for i in range(nx):v = max(v, abs(anchor[i]-x0[i])/self.limits[i])
                if v <= 1.:
                    for i in range(5, nx):x0[i] = anchor[i]
        P.slide.n = 0     # S8: point solves are regular (no blade-polar sliding)
        P.nreq = pr.nreq
        for i in range(pr.nreq):P.req[i] = pr.req[i]
        P.speed_kmh = pr.speed_kmh;P.load = pr.load;P.fixed = -2
        P.nz = nx;P.n = nx;P.m = 7+P.nprows
        for i in range(nx):
            P.free_idx[i] = i;P.zlower[i] = self.eng.cc.l_lower[i];P.zupper[i] = self.eng.cc.l_upper[i]
            P.lower[i] = P.zlower[i];P.upper[i] = P.zupper[i];P.steps[i] = self.steps[i]
            P.scale[i] = self.scales[i];P.step_limit[i] = self.limits[i]
            P.z[i] = x0[i]
        for i in range(P.m):P.rscale[i] = self.rscales[i]
        P.has_project = True
        P.has_dep = True
        for i in range(P.m*nx):P.dep[i] = self.dep[i]
        P.nmemo = 0;P.aircraft_evaluations = 0;P.last_slot = -1;P.nprep = 0;P.prep_next = 0
        P.evaluations = 0;P.derivative_samples = 0;P.nominal_comparisons = 0;P.nominal_restorations = 0;P.branch_secants = 0
        for i in range(nx):y[i] = pmin_np(pmax_np(x0[i], P.lower[i]), P.upper[i])
        pred.valid = False
        if previous is not None and isinstance(previous['predictor'], NativePredictor):
            pred = (<NativePredictor>previous['predictor']).p
        try:
            P.solve(y, max_nfev, 0, &pred, True, &res)
        finally:
            P.has_dep = False
            self.eng.cc.charge_problem(P)
        if res.termination == -1:
            from em_trim_numerics import UnevaluableTrial
            raise UnevaluableTrial('Nonfinite stationary residual; correction unresolved')
        slot = res.value.slot
        q = [x0[i] for i in range(nx)]
        for i in range(nx):q[i] = res.x[i]
        pr.calls += P.aircraft_evaluations
        if res.predictor.valid:
            np_pred = NativePredictor();np_pred.p = res.predictor;np_pred.n = P.n;np_pred.m = P.m
            if cache is not None:
                cache[key] = dict(predictor=np_pred, coordinates=np.asarray(q, dtype=float))
                cache.move_to_end(key)
                while len(cache) > 64:cache.popitem(last=False)
        termination = TERMINATIONS[res.termination]
        messages = dict(numerical_target='Numerical correction target reached',
                        stagnation='No useful componentwise progress; correction unresolved',
                        evaluation_budget='Total correction evaluation budget exhausted',
                        iteration_budget='Correction iteration budget exhausted', objective_budget='Correction budget exhausted')
        fun = [P.memo_r[slot*NMAX+i] for i in range(P.m)]
        # Corrector.solve's jac: the exact Jacobian at the returned state when current, else NaN
        jac = None
        if res.jacobian_current and res.predictor.valid:
            jac = [[res.predictor.matrix[i*P.n+k] for k in range(P.n)] for i in range(P.m)]
        diagnostics = dict(termination=termination, success=bool(res.success), nfev=res.nfev, evaluations=res.evaluations,
                           derivative_evaluations=res.evaluations-res.nfev, iterations=res.iterations,
                           accepted_steps=res.accepted, contractions=res.contractions, coordinate_polls=res.polls,
                           trust_radius=res.radius, return_metric='scaled_linf_then_l2',
                           return_score=(res.closure_ratio, res.return_key1), closure_ratio=res.closure_ratio,
                           merit_norm=res.return_key1, numerical_target=None, jacobian_reuses=res.reuses,
                           jacobian_refreshes=res.refreshes)
        return slot, dict(message=messages[termination], diagnostics=diagnostics, fun=fun, jac=jac), q

    # ------------------------------------------------------------------ the solve
    @cython.boundscheck(True)
    @cython.wraparound(True)
    def solve_row(self, double speed_kmh, double load, initial=None):
        return self.eng.row_of(self.solve(speed_kmh, load, initial))

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int solve(self, double speed_kmh, double load, initial) except -2:
        """BaselineSolver.solve(speed, load, initial, exhaustive=False): the representative state (its row lazily)."""
        import time
        import numpy as np
        from em_pitch_response import PHYSICAL_REASONS
        cdef PointProblem pr
        cdef double seed[NMAX]
        cdef int i, slot, si, sid, nx = self.eng.layout_nx
        cdef bint governed, same
        started = time.monotonic()
        cdef double t0 = s5_now()
        self.stats['solves'] += 1
        solver = self.solver
        candidates, attempts, recovery_records = [], [], []
        pending_recovery, closed_requests, tried = {}, set(), set()
        guess = [None]

        def ordinary_guess():
            if guess[0] is None:guess[0] = list(solver.initial_guess(speed_kmh/3.6, load))
            return guess[0]

        def starts():
            if initial is not None:
                supplied = list(initial['solution']) if isinstance(initial, dict) else list(initial[:5])
                yield supplied, initial
            cached = self.ST.nearby_seed(solver, speed_kmh, load, 0.)
            if cached is not None:yield list(cached['solution']), cached
            yield ordinary_guess(), None

        cdef FrozenState chain
        chain.valid = False;chain.has_pred = False
        self.eng.fin.chain = &chain
        try:
            self.starts_loop(starts, speed_kmh, load, nx, candidates, attempts, recovery_records, pending_recovery,
                             closed_requests, tried)
        finally:
            self.eng.fin.chain = NULL
            self.eng.t_point += s5_now()-t0
        sid = self.result(speed_kmh, load, candidates, attempts, recovery_records, started)
        if self.eng.brake and self.eng.brakes(sid):
            # S10: the representative state brakes: the prescribed-load correction from it with coarser governor pitch
            # (MarchEngine.brake_reseed) for the pulling state of this speed and load (S11: else the braking state stays)
            q = self.eng.brake_reseed(sid, -1, FIX_LOAD, load, 0, 0, 0, ROBUST_NFEV, ROBUST_EVALS, 2e-6, speed_kmh,
                                      self.eng.st[sid].thrust+S11_THRUST_EPS)
            if q >= 0:
                if self.eng.s11:self.eng.st[q].alt = sid
                return q
        return sid

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int starts_loop(self, starts, double speed_kmh, double load, int nx, list candidates, list attempts,
                         list recovery_records, dict pending_recovery, set closed_requests, set tried) except -1:
        import numpy as np
        cdef PointProblem pr
        cdef double seed[NMAX]
        cdef int i, slot, si, sid
        cdef bint governed, same
        done = False
        for attitude, saved in starts():
            try:
                pr = PointProblem(self.eng, speed_kmh, load)
                self.problem_initial(pr, attitude)
                branches = self.branches(pr, saved, attitude)
                if not branches:
                    attempts.append(dict(initial=attitude, sideslip=0., error='No consistent request at this trial attitude; aircraft coverage unresolved'))
            except (ValueError, ArithmeticError) as error:
                attempts.append(dict(initial=attitude, sideslip=0., error=str(error)))
                continue
            for branch_index, branch in enumerate(branches):
                if branch_index:
                    pr = PointProblem(self.eng, speed_kmh, load)
                    self.problem_initial(pr, attitude)
                pr.nreq = len(branch)
                for i in range(pr.nreq):pr.req[i] = branch[i]
                if (isinstance(saved, dict) and tuple(saved.get('stationary_request_indices', ())) == branch
                        and len(saved.get('stationary_coordinates', [])) == nx
                        and saved.get('stationary_coordinate_names', self.eng.layout_names) == self.eng.layout_names):
                    coords = saved['stationary_coordinates']
                    for i in range(nx):seed[i] = pmin_np(pmax_np(float(coords[i]), self.eng.cc.l_lower[i]), self.eng.cc.l_upper[i])
                else:
                    for i in range(nx):seed[i] = pr.initial[i]
                identity = (branch, tuple(seed[i] for i in range(nx)))
                if identity in tried:continue
                tried.add(identity)
                try:
                    governed = self.eng.is_prop and any(n.startswith('governor:') for n in self.eng.layout_names)
                    same = True
                    for i in range(nx):
                        if seed[i] != pr.initial[i]:same = False
                    if self.eng.is_prop and saved is None and same:
                        self.seed_propulsion(pr, seed)
                    same = True
                    for i in range(nx):
                        if seed[i] != pr.initial[i]:same = False
                    attempt_seed = [seed[i] for i in range(nx)]
                    slot, fit, x = self.correct(pr, seed, 120, not isinstance(saved, dict) and same)
                    si = self.finish(pr, slot)
                    sid = self.eng.state_from_src(si)
                    if governed:
                        if self.eng.srcs[si].fin.prop_converged:closed_requests.add(branch)
                        else:
                            score = float(np.linalg.norm(fit['fun'][7:]))
                            previous = pending_recovery.get(branch)
                            if previous is None or score < previous[0]:
                                pending_recovery[branch] = (score, pr, fit, x, list(attitude))
                    candidates.append(('native', sid, fit))
                    attempts.append(dict(initial=attempt_seed, sideslip=0., requests=list(branch), calls=pr.calls,
                                         local_valid=bool(self.eng.srcs[si].fin.local_valid),
                                         reasons=self.eng.reason_list(sid)))
                    if self.eng.srcs[si].fin.local_valid:
                        done = True;break
                    if self.eng.physical(sid):
                        done = True;break
                except (ValueError, ArithmeticError) as error:
                    attempts.append(dict(initial=[seed[i] for i in range(nx)], sideslip=0., requests=list(branch), error=str(error)))
            if done:break
        if not done and pending_recovery:
            self.recover(speed_kmh, load, pending_recovery, closed_requests, candidates, recovery_records)
        return 0

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int finish(self, PointProblem pr, int slot) except -1:
        """JointProblem.finish(x) of the corrected state: the native finish with the problem's calls."""
        global g_sink
        cdef int si = self.eng.new_src()
        cdef long scratch[NCOUNT]
        cdef long* sink = g_sink
        cdef double q[NMAX]
        cdef double inner[MAXPATH]
        cdef int i
        for i in range(NCOUNT):scratch[i] = 0
        # the finish's own charges, except its propulsion frame: that is a JointProblem.propulsion_at
        # sample, charged on a cache miss of the problem only (whichever sink is current)
        g_sink = scratch
        try:
            self.eng.fin.compute(self.eng.P, slot, pr.speed_kmh, pr.load, pr.calls, &self.eng.srcs[si])
        except RuntimeError as error:
            self.eng.nsrc -= 1
            from em_stationary import BranchUnavailable
            raise BranchUnavailable('Governed request branch changed or is inconsistent') from error
        finally:
            g_sink = sink
        if self.eng.is_prop:scratch[K_FRAMES] = 0
        for i in range(NCOUNT):charge(i, scratch[i])
        if self.eng.is_prop:
            for i in range(pr.nx):q[i] = self.eng.P.memo_q[slot*NMAX+i]
            pr.propulsion_rows(q, inner)
        return si

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef object recover(self, double speed_kmh, double load, dict pending, set closed, list candidates, list records):
        """em_stationary.solve's bounded failure recovery: em_prop_branch_recovery for each unclosed request
        branch (natively; its candidates join the ordinary ones)."""
        import numpy as np
        cdef RecBudget budget = None
        cdef PointProblem pr
        if self.python_recovery:return self.recover_python(speed_kmh, load, pending, closed, candidates, records)
        cdef double t0 = s5_now()
        try:
            for branch in sorted(set(pending)-closed):
                _, pr, fit, x, attitude = pending[branch]
                if budget is None:budget = RecBudget()
                self.stats['native_recoveries'] = self.stats.get('native_recoveries', 0)+1
                try:
                    try:hints = self.recovery_warnings(pr, fit, x)
                    except (ValueError, ArithmeticError, np.linalg.LinAlgError) as error:
                        hints = [dict(warning='derivative unavailable', reason=str(error))]
                    extra, record = self.recover_branch(pr, x, budget)
                    for point in extra:candidates.append(('python', self.eng.import_row(point), point))
                    records.append(dict(record, sideslip=0., requests=list(branch), early_warnings=hints))
                    if any(p['local_valid'] for p in extra):return True
                except (ValueError, ArithmeticError, np.linalg.LinAlgError) as error:
                    records.append(dict(status='unresolved', reason=str(error), sideslip=0., requests=list(branch),
                                        coverage_complete=False))
        finally:
            self.eng.t_python += s5_now()-t0
        return False

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef object recover_python(self, double speed_kmh, double load, dict pending, set closed, list candidates, list records):
        """The Python em_prop_branch_recovery on a Python JointProblem of each unclosed request branch
        (WT_NATIVE_RECOVERY=0: the reference for the native recovery)."""
        global g_sink
        import numpy as np
        from types import SimpleNamespace
        from em_prop_branch_recovery import Budget, recover, warnings
        from em_trim_work import Work, use
        from em_stationary import JointProblem
        budget = None
        saved = <size_t>g_sink;g_sink = NULL
        try:
            for branch in sorted(set(pending)-closed):
                _, pr, fit, x, attitude = pending[branch]
                if budget is None:budget = Budget()
                problem = JointProblem(self.solver.core(), speed_kmh, load, attitude, branch)
                problem.calls = pr.calls
                xa = np.asarray(x, dtype=float)
                with use(Work({})):problem.propulsion_at(xa)
                py_fit = SimpleNamespace(x=xa, fun=np.asarray(fit['fun']), jac=np.full((len(fit['fun']), len(x)), np.nan),
                                         message=fit['message'], **fit['diagnostics'])
                self.stats['python_recoveries'] += 1
                try:
                    try:hints = warnings(problem, py_fit)
                    except (ValueError, ArithmeticError, np.linalg.LinAlgError) as error:
                        hints = [dict(warning='derivative unavailable', reason=str(error))]
                    extra, record = recover(problem, xa, detailed=False, budget=budget)
                    for point in extra:candidates.append(('python', self.eng.import_row(point), point))
                    records.append(dict(record, sideslip=0., requests=list(branch), early_warnings=hints))
                    if any(p['local_valid'] for p in extra):return True
                except (ValueError, ArithmeticError, np.linalg.LinAlgError) as error:
                    records.append(dict(status='unresolved', reason=str(error), sideslip=0., requests=list(branch),
                                        coverage_complete=False))
        finally:
            g_sink = <long*>saved
        return False


    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef list recovery_warnings(self, PointProblem pr, dict fit, list x):
        """em_prop_branch_recovery.warnings(problem, fit): local Schur-complement hints from the fit's Jacobian
        (NaN when the correction returned without a current one)."""
        import math
        import numpy as np
        cdef PropModel pm = self.eng.fin.pm
        cdef int j
        paths = pm.paths
        active = [paths[pm.active[j]] for j in range(pm.nactive)]
        rows = [pm.rows[j] for j in range(pm.nrows)]
        m = len(fit['fun'])
        matrix = np.asarray(fit['jac'], dtype=float) if fit.get('jac') is not None else np.full((m, len(x)), np.nan)
        fun = fit['fun']
        result = []
        for col, path in enumerate(active, 7):
            if path[0] != 'pitch':continue
            if pm.governor_saturation and not 0. <= x[col] <= 1.:
                result.append(dict(shaft=path[1], governor_coordinate=float(x[col]),
                                   warning='saturated governor; pitch-only slope is inapplicable'))
                continue
            shaft = path[1]
            indices = {pm.trans[shaft].props[j].index for j in range(pm.trans[shaft].np)}
            flows = [(c, q) for c, q in enumerate(active, 7) if q[0] == 'flow' and q[1] in indices]
            flowcols = [c for c, _ in flows]
            flowrows = [7+rows.index(paths.index(q)) for _, q in flows]
            torque = 7+rows.index(paths.index(('shaft', shaft)))
            row = dict(shaft=shaft, pitch_fraction=float(x[col]), net_torque_scaled=float(fun[torque]),
                       scope='local frozen-attitude derivative; not a fold certificate')
            try:
                block = matrix[np.ix_(flowrows, flowcols)]
                condition = float(np.linalg.cond(block)) if flows else 1.
                row['wake_jacobian_condition'] = condition if math.isfinite(condition) else None
                if condition > 1e10:raise np.linalg.LinAlgError('ill-conditioned wake')
                slope = float(matrix[torque, col]-(matrix[torque, flowcols] @
                              np.linalg.solve(block, matrix[flowrows, col]) if flows else 0.))
                row['net_torque_pitch_slope'] = slope
                if abs(slope) < 1e-12:row['warning'] = 'flat torque curve'
                else:
                    step = -row['net_torque_scaled']/slope
                    row['predicted_pitch_correction'] = step
                    if not 0. <= x[col]+step <= 1.:row['warning'] = 'linear prediction leaves pitch interval'
                    elif abs(step) > .1:row['warning'] = 'weak torque slope; large predicted pitch correction'
                if min(x[col], 1.-x[col]) < 1e-6:row['pitch_endpoint'] = True
            except np.linalg.LinAlgError:row['warning'] = 'wake branch derivative unavailable'
            result.append(row)
        return result

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef tuple discover(self, PointProblem pr, list x, RecBudget budget):
        """em_prop_branch_recovery.discover: joint seeds from the frozen pitch/wake branches of each governed shaft."""
        import itertools, math
        cdef CurveProblem P = self.eng.P
        cdef PropModel pm = self.eng.fin.pm
        cdef PropPrep* prep
        cdef PropPrep prep_copy
        cdef PropSample ps
        cdef double q[NMAX]
        cdef double qq[MAXPATH]
        cdef double inner[MAXPATH]
        cdef double lo, hi
        cdef int i, k, s, idx, nx = pr.nx
        cdef WakeBlock wb
        diagnostics = dict(revision=RECOVERY_REVISION, status='unresolved', trigger='propulsion nonclosure',
                           shafts=[], corrections=[], coverage_complete=False, governor_selection_certified=False)
        seeds = []
        for i in range(nx):q[i] = x[i]
        # problem.residual(x): the corrected state's propulsion sample (its frame and equilibrium coordinates)
        P.speed_kmh = pr.speed_kmh;P.nprep = 0;P.prep_next = 0;P.slide.n = 0
        prep = P.prepared(pr.load, q[0], q[1])
        prep_copy = prep[0]
        pm.use_slide(NULL)
        if pm.propulsion_q(&prep_copy, &q[7], pr.req, pr.nreq, inner, &ps, qq):
            from em_stationary import BranchUnavailable
            raise BranchUnavailable('Governed request branch changed or is inconsistent')
        paths = pm.paths
        targets = [ps.frame.trans_omega[s] for s in range(pm.nt)]
        spans, pitch_intervals = {}, {}
        for s in range(pm.nt):
            if ('pitch', s) not in paths or not prep_copy.governed_request[s]:continue
            intervals = []
            for k in range(pm.trans[s].np):
                idx = pm.trans[s].props[k].index
                if not pm.props[idx].governor:continue
                pm.pitch_limits(&pm.props[idx], prep_copy.velocity, prep_copy.rates, targets[s]*pm.trans[s].props[k].ratio,
                                &lo, &hi)
                intervals.append((lo, hi))
            spans[s] = max((b-a for a, b in intervals), default=0.)
            pitch_intervals[s] = [list(v) for v in intervals]
        if not spans:
            diagnostics['reason'] = 'No governed pitch unknown';return seeds, diagnostics
        qlist = [qq[i] for i in range(pm.npaths)]
        options = []
        for shaft in spans:
            check_cancel_py()
            si = paths.index(('shaft', shaft));pi = paths.index(('pitch', shaft))
            indices = {pm.trans[shaft].props[k].index for k in range(pm.trans[shaft].np)}
            flows = [j for j, path in enumerate(paths) if path[0] == 'flow' and path[1] in indices]
            wb = WakeBlock()
            wb.pm = pm;wb.prep = prep_copy;wb.si = si;wb.nshaft_props = pm.trans[shaft].np;wb.calls = 0
            for s in range(NP):
                wb.has_assigned[s] = False;wb.assigned[s] = 0.
            # the frame's shaft speeds (the request, or its saturated governor speed) stay fixed
            for s in range(pm.nt):
                if prep_copy.governed_request[s]:
                    wb.assigned[s] = ps.frame.trans_omega[s];wb.has_assigned[s] = True
            wb.n = wb.m = len(flows)
            for k, j in enumerate(flows):
                wb.flows[k] = j
                wb.lower[k] = pm.lower[j];wb.upper[k] = pm.upper[j]
                wb.steps[k] = 1e-5;wb.scale[k] = 1.;wb.step_limit[k] = .3;wb.rscale[k] = 1e-6
            wb.has_project = False;wb.has_dep = False;wb.target = 1e-7;wb.reuse = False
            row = Scan(wb, budget, qlist, si, pi, flows).run(spans[shaft] == 0.)
            row.update(shaft=shaft, request_index=pr.req[shaft], requested_omega=targets[shaft],
                       pitch_intervals_rad=pitch_intervals[shaft], torque_scale_nm=pm.torque_scales[shaft])
            for probe in row['probes']:probe['net_torque_nm'] = probe['net_torque_scaled']*pm.torque_scales[shaft]
            diagnostics['shafts'].append(row)
            options.append([(pi, flows, r['coordinates']) for r in row['roots']])
        total = math.prod(len(v) for v in options)
        diagnostics['candidate_combinations'] = total
        allowance = budget.corrections
        diagnostics['combination_budget_exhausted'] = total > allowance
        active = [pm.active[k] for k in range(pm.nactive)]
        for combination in itertools.islice(itertools.product(*options), allowance):
            check_cancel_py()
            seed = list(qlist)
            for pi, flows, state in combination:
                seed[pi] = state[pi]
                for j in flows:seed[j] = state[j]
            start = list(x)
            start[7:] = [seed[j] for j in active]
            seeds.append(start)
        if seeds:diagnostics['status'] = 'discovered_seeds'
        return seeds, diagnostics

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef tuple recover_branch(self, PointProblem pr, list x, RecBudget budget):
        """em_prop_branch_recovery.recover (detailed=False, unconditional): correct the nominated seeds with the
        joint equations and the common admission."""
        import numpy as np
        cdef PointProblem pc
        cdef double start[NMAX]
        cdef double xq[NMAX]
        cdef int i, slot, si, nx = pr.nx
        cdef RowSrc* src
        seeds, diagnostics = self.discover(pr, x, budget)
        diagnostics['conditional_aircraft'] = False
        candidates = []
        names = ('termination', 'success', 'nfev', 'evaluations', 'derivative_evaluations', 'iterations', 'accepted_steps',
                 'contractions', 'coordinate_polls', 'trust_radius', 'return_metric', 'return_score', 'closure_ratio',
                 'merit_norm')
        # the branch's fresh JointProblem (shared by its seed corrections): its calls, and the corrected
        # state's propulsion sample cached
        pc = PointProblem(self.eng, pr.speed_kmh, pr.load)
        pc.nreq = pr.nreq
        for i in range(pr.nreq):pc.req[i] = pr.req[i]
        pc.calls = pr.calls
        for i in range(nx):xq[i] = x[i]
        pc.warm(xq)
        for seed in seeds:
            check_cancel_py();charge(K_BCORR, 1)
            budget.corrections -= 1
            try:
                for i in range(nx):start[i] = seed[i]
                slot, fit, q = self.correct(pc, start, CORRECTION_NFEV, False)
                si = self.finish(pc, slot)
                src = &self.eng.srcs[si]
                point = self.eng.fin.point(src)
                residual = np.asarray([src.resid[i] for i in range(src.nres)])
                scales = np.asarray(point['stationary_correction_residual_scales'])
                point['internal_residual_norm'] = float(np.linalg.norm(residual[5:]))
                point['internal_closed'] = bool(src.fin.prop_converged and np.all(abs(residual[5:]) <= scales[5:]))
                point['numerical_message'] = str(fit['message'])
                point['correction_diagnostics'] = {name: fit['diagnostics'].get(name) for name in names}
                point['branch_recovery_candidate'] = True
                candidates.append(point)
                diagnostics['corrections'].append(dict(local_valid=point['local_valid'], internal_closed=point['internal_closed'],
                                                       reasons=point['reasons'], force_error_g=point['force_error_g'],
                                                       angular_error_rad_s2=point['angular_error_rad_s2']))
            except (ValueError, ArithmeticError, np.linalg.LinAlgError) as error:
                diagnostics['corrections'].append(dict(error=str(error)))
        if any(p['local_valid'] for p in candidates):diagnostics['status'] = 'recovered_candidates'
        return candidates, diagnostics

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int result(self, double speed_kmh, double load, list candidates, list attempts, list records, started) except -2:
        """em_stationary.solve's result(): the representative candidate (BaselineSolver.solve raises when none)."""
        import time
        cdef int sid, rep = -1, k, i
        cdef TState* t
        admitted = [c for c in candidates if self.eng.st[c[1]].valid]
        for c in admitted:self.remember(c)
        pool = admitted or candidates
        if not pool:
            from em_trim_work import WorkLimit, current
            if current() is not None:raise WorkLimit(current(), 'stationary sample has no established candidate')
            raise ValueError('Unresolved numerical calculation: stationary sample has no established candidate')
        if admitted:best = min(pool, key=lambda c: self.preference(c[1]))
        else:best = min(pool, key=lambda c: self.failed_preference(c[1]))
        rep = best[1]
        sid = self.eng.new_state()
        self.eng.st[sid] = self.eng.st[rep]
        self.eng.st[sid].base = sid;self.eng.st[sid].row = -1;self.eng.st[sid].limit = -1
        self.eng.st[sid].src = -1;self.eng.rows[sid] = None
        self.eng.solve_of[sid] = (self, candidates, best, attempts, records, len(admitted), time.monotonic()-started)
        return sid

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef tuple preference(self, int sid):
        cdef TState* t = &self.eng.st[sid]
        cdef int i
        return (0, abs(t.beta), -t.ps, tuple(t.coords[i] for i in range(t.nx)))

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef tuple failed_preference(self, int sid):
        import math
        cdef TState* t = &self.eng.st[sid]
        cdef RowSrc* s
        cdef int i
        if t.src >= 0:
            s = &self.eng.srcs[t.src]
            error = 0.
            for i in range(s.nres):
                if not isfinite(s.resid[i]):
                    error = float('inf');break
                error = max(error, abs(s.resid[i])/self.rscales[i])
        else:
            row = self.eng.rows[sid]
            residual = np.asarray(row.get('stationary_residual', [float('inf')]))
            scales = np.asarray(row.get('stationary_correction_residual_scales', np.ones(len(residual))))
            error = float(np.max(abs(residual)/scales, initial=0.))
        return (not t.converged, error if math.isfinite(error) else float('inf'), self.preference(sid))

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef remember(self, tuple candidate):
        """em_stationary.remember_seed of an admitted candidate."""
        cdef TState* t = &self.eng.st[candidate[1]]
        cdef int i
        if candidate[0] == 'python':
            self.ST.remember_seed(self.solver, candidate[2]);return
        point = dict(local_valid=True, sideslip_attitude_deg=0., stationary_coordinate_names=self.eng.layout_names,
                     stationary_request_indices=[t.req[i] for i in range(t.nreq)] if self.eng.is_prop else [],
                     speed_kmh=t.speed, load_g=t.load, solution=[t.sol[i] for i in range(5)],
                     stationary_coordinates=[t.coords[i] for i in range(t.nx)])
        self.ST.remember_seed(self.solver, point)

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef object row(self, int sid):
        """BaselineSolver.sample(result) of a point solve: the candidate records and the selection."""
        solver_, candidates, best, attempts, records, nadmitted, seconds = self.eng.solve_of[sid]
        points = []
        rep_point = None
        for c in candidates:
            if c[0] == 'native':
                point = self.eng.fin.point(&self.eng.srcs[self.eng.st[c[1]].src])
                fit = c[2]
                point['numerical_message'] = str(fit['message'])
                point['correction_diagnostics'] = dict(fit['diagnostics'])
            else:
                point = c[2]
            points.append(point)
            if c is best:rep_point = point
        output = dict(rep_point)
        admitted = [p for p in points if p['local_valid']]
        output.update(stationary_candidates=points, stationary_attempts=attempts, propeller_branch_recovery=records,
                      local_candidate_count=len(admitted), selected_candidates=[rep_point] if admitted else [],
                      selection_status='numerical_selected' if admitted else 'unresolved',
                      selection_scope='First admitted continuation or ordinary candidate; bounded failure recovery, without global selection certification',
                      seconds=seconds)
        if 'stationary_coordinates' not in output:
            raise RuntimeError('point solve without an established candidate')
        return self.solver.sample(output)


cdef class LevelSolver:
    """em_sampling.solve_level(solver, speed, initial) on the native point solve."""
    cdef PointSolver ps

    @cython.boundscheck(True)
    @cython.wraparound(True)
    def __init__(self, PointSolver ps):
        self.ps = ps

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int solve(self, double speed, initial) except -2:
        cdef int point, retry
        cdef TState* a
        cdef TState* b
        point = self.ps.solve(speed, 1., initial)
        if self.ps.eng.st[point].converged:return point
        guess = self.ps.solver.initial_guess(speed/3.6, 1.)
        guess[0] *= .5
        retry = self.ps.solve(speed, 1., guess)
        a = &self.ps.eng.st[point];b = &self.ps.eng.st[retry]
        if b.valid or (b.converged and not a.converged):return retry
        return point if not (b.force_error_g+b.angular_error < a.force_error_g+a.angular_error) else retry
