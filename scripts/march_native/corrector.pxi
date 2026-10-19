# Native port of em_stationary_corrector.Corrector.solve (production mode:
# componentwise residual-scale target, reuse_jacobian=True; S5: also a raw
# numerical target and reuse_jacobian=False, as the propeller wake probes) and
# em_trim_numerics.jacobian (actual_step, prefer_informative, nominal
# fallback stencils, branch-aware columns, projected stencils).
#
# Control flow, acceptance rules, budgets and counters follow the Python
# source line by line. Linear algebra (least squares, bounded least squares,
# stencil solve) is a small native implementation, so iterates agree with
# the Python corrector to rounding, not bit for bit.

DEF BMAX = 10

cdef double[18] FACTORS = [1., .5, 1.5, 2., .25, -.5, -1., -1.5, -2., .1, 3., 4., 5., -.1, .05, -.05, .01, -.01]
cdef double MERIT_WINDOW_N = 5
cdef double SUFFICIENT_DECREASE = 1e-4
cdef double MODEL_AGREEMENT = .1
cdef double MERIT_ROUNDOFF = 32.*2.220446049250313e-16
cdef int PROGRESS_WINDOW = 6
cdef int FAR_PROGRESS_WINDOW = 2
cdef double NEAR_CLOSURE_RATIO = 10.
cdef double PROGRESS_FRACTION = .001

# status codes inside the solve
DEF OK = 0
DEF UNEVALUABLE = 1
DEF CLOSED = 2
DEF EVAL_BUDGET = 3
DEF OBJ_BUDGET = 4
DEF ITER_BUDGET = 5
DEF DERIV_UNAVAILABLE = 6
DEF LINALG = 7

# termination codes in results
DEF T_TARGET = 1
DEF T_STAGNATION = 2
DEF T_EVAL_BUDGET = 3
DEF T_OBJ_BUDGET = 4
DEF T_ITER_BUDGET = 5


cdef struct Sample:
    double q[NMAX]
    double r[NMAX]
    int br[BMAX]
    int slot


cdef struct Predictor:
    bint valid
    double matrix[NMAX*NMAX]
    double x[NMAX]
    int br[BMAX]


cdef struct Result:
    double x[NMAX]
    Sample value
    int termination
    bint success
    double closure_ratio, return_key1, radius
    int nfev, evaluations, iterations, accepted, contractions, polls, reuses, refreshes, unavailable, nonmonotone
    bint jacobian_current
    Predictor predictor


cdef inline bint same_br(const int* a, const int* b) noexcept nogil:
    cdef int i
    for i in range(BMAX):
        if a[i] != b[i]:
            return False
    return True


cdef class NativeCorrector:
    cdef int n, m
    cdef double lower[NMAX]
    cdef double upper[NMAX]
    cdef double steps[NMAX]
    cdef double scale[NMAX]
    cdef double step_limit[NMAX]
    cdef double rscale[NMAX]
    cdef bint has_project
    cdef bint has_dep
    cdef bint dep[NMAX*NMAX]
    cdef public long evaluations, derivative_samples, nominal_comparisons, nominal_restorations, branch_secants
    # numerical_target (negative: None, the componentwise production target) and reuse_jacobian
    cdef double target
    cdef bint reuse

    def __cinit__(self, *args, **kwargs):
        self.target = -1.;self.reuse = True

    # ---- evaluator interface (subclasses) ------------------------------
    cdef int evaluate(self, const double* q, Sample* out) except -1:
        return UNEVALUABLE

    cdef int project(self, const double* q, const Sample* base, double* out) except -1:
        cdef int i
        for i in range(self.n):
            out[i] = q[i]
        return OK

    # ---- helpers --------------------------------------------------------
    cdef double merit(self, const double* r) noexcept:
        cdef double w[NMAX]
        cdef int i
        for i in range(self.m):
            w[i] = r[i]/self.rscale[i]
        return vector_norm(self.m, w)

    cdef bint inside(self, const double* q) noexcept:
        cdef int i
        for i in range(self.n):
            if not (q[i] >= self.lower[i]) or not (q[i] <= self.upper[i]):
                return False
        return True

    cdef int direction(self, const double* matrix, const double* residual, const double* x, double radius,
                       double tolerance, double* delta) noexcept:
        cdef double cap, t
        cdef double lo[NMAX]
        cdef double hi[NMAX]
        cdef double w[NMAX]
        cdef double A[NMAX*NMAX]
        cdef double As[NMAX*NMAX]
        cdef double rhs[NMAX]
        cdef double z[NMAX]
        cdef double Af[NMAX*NMAX]
        cdef double bf[NMAX]
        cdef double lbf[NMAX]
        cdef double ubf[NMAX]
        cdef double xl[NMAX]
        cdef double xb[NMAX]
        cdef int i, j, k, nf, n = self.n, m = self.m
        cdef int fidx[NMAX]
        cdef bint outside = False, inb
        for j in range(n):
            cap = radius*self.step_limit[j]
            lo[j] = pmax_np(self.lower[j]-x[j], -cap)
            hi[j] = pmin_np(self.upper[j]-x[j], cap)
        for i in range(m):
            w[i] = residual[i]/self.rscale[i]
            for j in range(n):
                A[i*n+j] = matrix[i*n+j]/self.rscale[i]
                As[i*n+j] = A[i*n+j]*self.scale[j]
            rhs[i] = -w[i]
        if svd_solve(m, n, As, rhs, z):
            return LINALG
        for j in range(n):
            delta[j] = self.scale[j]*z[j]
            if delta[j] < lo[j] or delta[j] > hi[j]:
                outside = True
        if not outside:
            return OK
        nf = 0
        for j in range(n):
            if hi[j] > lo[j]:
                fidx[nf] = j; nf += 1
            delta[j] = lo[j]
        if nf == 0:
            return OK
        for i in range(m):
            t = 0.
            for j in range(n):
                if not (hi[j] > lo[j]):
                    t += A[i*n+j]*delta[j]
            bf[i] = -w[i]-t
            for k in range(nf):
                Af[i*nf+k] = A[i*n+fidx[k]]*self.scale[fidx[k]]
        for k in range(nf):
            lbf[k] = lo[fidx[k]]/self.scale[fidx[k]]; ubf[k] = hi[fidx[k]]/self.scale[fidx[k]]
        if svd_solve(m, nf, Af, bf, xl):
            return LINALG
        inb = True
        for k in range(nf):
            if not (xl[k] >= lbf[k] and xl[k] <= ubf[k]):
                inb = False
        if inb:
            for k in range(nf):
                xb[k] = xl[k]
        elif bvls(m, nf, Af, bf, xl, lbf, ubf, tolerance, 20, xb):
            return LINALG
        for k in range(nf):
            delta[fidx[k]] = xb[k]*self.scale[fidx[k]]
        return OK

    # ---- the solve ------------------------------------------------------
    cdef int solve(self, const double* initial, int max_nfev, int max_evaluations, const Predictor* predictor,
                   bint fast_stagnation, Result* res) except -1:
        cdef SolveState st
        cdef int i, j, code
        st.c = <void*>self
        st.n = self.n; st.m = self.m
        st.max_nfev = max_nfev; st.max_iterations = max_nfev
        st.max_evaluations = max_evaluations if max_evaluations > 0 else max_nfev*(1+self.n*18)
        for i in range(self.n):
            st.x[i] = pmin_np(pmax_np(initial[i], self.lower[i]), self.upper[i])
            st.stencil[i] = self.steps[i]
        st.start = self.evaluations
        st.used = st.iterations = st.contractions = st.polls = st.accepted = st.unavailable = 0
        st.radius = 1.
        st.has_best = st.has_return = False
        st.best_norm = 1e308*10.
        st.has_matrix = False; st.matrix_exact = False
        st.reuses = st.refreshes = 0
        st.nrecent = 0
        st.nonmonotone = 0
        st.progress_ratio = 1e308*10.; st.progress_iteration = 0; st.progress_models = 0
        st.termination = T_OBJ_BUDGET
        st.fast_stagnation = fast_stagnation
        code = run_solve(&st, predictor)
        if code == -2:
            res.termination = -1
            res.success = False
            res.predictor.valid = False
            return 0
        if code == CLOSED:
            st.termination = T_TARGET
        elif code == EVAL_BUDGET:
            st.termination = T_EVAL_BUDGET
        elif code == OBJ_BUDGET:
            st.termination = T_OBJ_BUDGET
        elif code == ITER_BUDGET:
            st.termination = T_ITER_BUDGET
        elif code < 0:
            return -1
        for i in range(self.n):
            res.x[i] = st.return_value.q[i]
        res.value = st.return_value
        res.termination = st.termination
        res.success = st.termination == T_TARGET
        res.closure_ratio = st.return_key0
        res.nfev = st.used; res.evaluations = self.evaluations-st.start; res.iterations = st.iterations
        res.return_key1 = st.return_key1; res.radius = st.radius; res.accepted = st.accepted
        res.contractions = st.contractions; res.polls = st.polls; res.reuses = st.reuses; res.refreshes = st.refreshes
        res.unavailable = st.unavailable; res.nonmonotone = st.nonmonotone
        res.jacobian_current = st.matrix_exact and st.has_matrix
        if res.jacobian_current:
            for i in range(self.n):
                if st.matrix_x[i] != st.return_value.q[i]:
                    res.jacobian_current = False
        res.predictor.valid = False
        if st.has_matrix:
            res.predictor.valid = True
            for i in range(self.m*self.n):
                if not isfinite(st.matrix[i]):
                    res.predictor.valid = False
            if res.predictor.valid:
                for i in range(self.m*self.n):
                    res.predictor.matrix[i] = st.matrix[i]
                for i in range(self.n):
                    res.predictor.x[i] = st.matrix_x[i]
                for i in range(BMAX):
                    res.predictor.br[i] = st.matrix_value.br[i]
        return 0


cdef inline double pmax_np(double a, double b) noexcept nogil:
    # numpy.maximum / clip for finite inputs
    return a if a >= b else b

cdef inline double pmin_np(double a, double b) noexcept nogil:
    return a if a <= b else b


cdef struct Recent:
    double q[NMAX]
    double norm


cdef struct SolveState:
    void* c
    int n, m
    int max_nfev, max_iterations, max_evaluations
    long start
    int used, iterations, contractions, polls, accepted, unavailable, reuses, refreshes, nonmonotone
    double radius
    double x[NMAX]
    Sample value
    bint has_best, has_return
    Sample best_value
    double best_norm
    Sample return_value
    double return_key0, return_key1
    bint has_matrix, matrix_exact
    double matrix[NMAX*NMAX]
    double matrix_x[NMAX]
    Sample matrix_value
    double stencil[NMAX]
    Recent recent[5]
    int nrecent
    double progress_ratio
    int progress_iteration, progress_models
    int termination
    bint fast_stagnation
    # current-iteration probe best (sampled_candidate)
    bint has_sc
    Sample sc
    double sc_norm


cdef int st_sample(SolveState* st, const double* q, bint objective, Sample* out) except -1:
    """Corrector.solve's sample(): budgets, best/return bookkeeping, closure."""
    cdef NativeCorrector c = <NativeCorrector>st.c
    cdef int code, i
    cdef double w[NMAX]
    cdef double norm, k0
    if c.evaluations-st.start >= st.max_evaluations:
        return EVAL_BUDGET
    if objective:
        if st.used >= st.max_nfev:
            return OBJ_BUDGET
        st.used += 1
    c.evaluations += 1
    code = c.evaluate(q, out)
    if code == OK:
        for i in range(c.m):
            if not isfinite(out.r[i]):
                code = UNEVALUABLE
    if code != OK:
        st.unavailable += 1
        return UNEVALUABLE
    for i in range(c.n):
        out.q[i] = q[i]
    k0 = 0.
    for i in range(c.m):
        w[i] = out.r[i]/c.rscale[i]
        if fabs(w[i]) > k0 or i == 0:
            k0 = pmax(k0, fabs(w[i])) if i else fabs(w[i])
    norm = vector_norm(c.m, w)
    cdef bint met = True
    cdef double raw = 0.
    if c.target >= 0.:
        for i in range(c.m):
            raw = pmax(raw, fabs(out.r[i]))
        met = raw == 0. if c.target == 0. else raw < c.target
    if not st.has_best or norm < st.best_norm:
        st.best_value = out[0]; st.best_norm = norm; st.has_best = True
    if not st.has_return or (k0 < st.return_key0 or (k0 == st.return_key0 and norm < st.return_key1)):
        st.return_value = out[0]; st.return_key0 = k0; st.return_key1 = norm; st.has_return = True
    if met and k0 <= 1.:
        st.best_value = out[0]; st.best_norm = norm
        st.return_value = out[0]; st.return_key0 = k0; st.return_key1 = norm
        return CLOSED
    return OK


cdef void remember(SolveState* st, const double* q, double norm) noexcept nogil:
    cdef int i, j
    if st.nrecent == 5:
        for i in range(4):
            st.recent[i] = st.recent[i+1]
        st.nrecent = 4
    for j in range(st.n):
        st.recent[st.nrecent].q[j] = q[j]
    st.recent[st.nrecent].norm = norm
    st.nrecent += 1


cdef inline bint significant_decrease(double before, double after) noexcept nogil:
    return after < before and (not isfinite(before) or before-after > MERIT_ROUNDOFF*pmax(before, after))


cdef void adapt(SolveState* st, const double* displacement, bint contract) noexcept:
    cdef NativeCorrector c = <NativeCorrector>st.c
    cdef double floor_, size = 0., v
    cdef int i
    for i in range(st.n):
        v = fabs(displacement[i])/c.step_limit[i]
        if v > size:
            size = v
    if contract:
        st.radius = .5*pmin(st.radius, size if size > 0. else st.radius)
        st.contractions += 1
    if size > 0.:
        for i in range(st.n):
            floor_ = 8.*spacing(pmax_np(1., fabs(st.x[i])))
            if fabs(displacement[i]) >= floor_:
                st.stencil[i] = pmin_np(c.steps[i], pmax_np(.25*fabs(displacement[i]), floor_))


cdef void accept(SolveState* st, const Sample* trial, int kind, bint relaxed, bint empirical) noexcept:
    # kind: 0 newton, 1 sampled_best, 2 coordinate_model, 3 poll
    cdef NativeCorrector c = <NativeCorrector>st.c
    cdef double displacement[NMAX]
    cdef double scaled[NMAX]
    cdef double change[NMAX]
    cdef double length = 0., t, size = 0., v
    cdef int i, j
    for i in range(st.n):
        displacement[i] = trial.q[i]-st.x[i]
    if c.reuse and st.has_matrix:
        if kind == 0 and not relaxed and not empirical and same_br(st.value.br, trial.br):
            for j in range(st.n):
                scaled[j] = displacement[j]/c.scale[j]
                length += scaled[j]*scaled[j]
            if length > 0.:
                for i in range(st.m):
                    t = 0.
                    for j in range(st.n):
                        t += st.matrix[i*st.n+j]*displacement[j]
                    change[i] = trial.r[i]-st.value.r[i]-t
                for i in range(st.m):
                    for j in range(st.n):
                        st.matrix[i*st.n+j] = st.matrix[i*st.n+j]+change[i]*(scaled[j]/length)/c.scale[j]
                for j in range(st.n):
                    st.matrix_x[j] = trial.q[j]
                st.matrix_value = trial[0]
                st.matrix_exact = False
        else:
            st.has_matrix = False
            st.matrix_exact = False
    adapt(st, displacement, False)
    if empirical:
        for i in range(st.n):
            v = fabs(displacement[i])/c.step_limit[i]
            if v > size:
                size = v
        st.radius = pmin(1., pmax(st.radius, size))
    for i in range(st.n):
        st.x[i] = trial.q[i]
    st.value = trial[0]
    st.accepted += 1
    st.nonmonotone += relaxed
    remember(st, st.x, c.merit(st.value.r))


cdef int proposal(SolveState* st, const double* q_in, const Sample* base, double* q) except -1:
    """Returns 1 when a proposal exists (written to q), else 0."""
    cdef NativeCorrector c = <NativeCorrector>st.c
    cdef double t[NMAX]
    cdef int i, code
    cdef bint same = True
    for i in range(st.n):
        t[i] = pmin_np(pmax_np(q_in[i], c.lower[i]), c.upper[i])
    if c.has_project:
        code = c.project(t, base, q)
        if code != OK:
            st.unavailable += 1
            return 0
    else:
        for i in range(st.n):
            q[i] = t[i]
    if not c.inside(q):
        return 0
    for i in range(st.n):
        if not isfinite(q[i]):
            return 0
        if q[i] != st.x[i]:
            same = False
    return 0 if same else 1


cdef int jacobian_probe(SolveState* st, const Sample* base, int axis, double length, bint comparison,
                        double* seen, int* nseen, double* column, double* disp, bint* same_branch) except -1:
    """em_trim_numerics.jacobian probe(); returns OK, UNEVALUABLE (no candidate) or a solve code."""
    cdef NativeCorrector c = <NativeCorrector>st.c
    cdef double q[NMAX]
    cdef double qp[NMAX]
    cdef double step, norm
    cdef Sample trial
    cdef int i, k, code
    cdef bint dup
    for i in range(st.n):
        q[i] = st.x[i]
    q[axis] += length
    if c.has_project:
        code = c.project(q, base, qp)
        if code != OK:
            return UNEVALUABLE
        for i in range(st.n):
            q[i] = qp[i]
    if not c.inside(q):
        return UNEVALUABLE
    step = q[axis]-st.x[axis]
    if step == 0.:
        return UNEVALUABLE
    for k in range(nseen[0]):
        dup = True
        for i in range(st.n):
            if seen[k*NMAX+i] != q[i]:
                dup = False;break
        if dup:
            return UNEVALUABLE
    if nseen[0] < 64:
        for i in range(st.n):
            seen[nseen[0]*NMAX+i] = q[i]
        nseen[0] += 1
    if comparison:
        c.nominal_comparisons += 1
    c.derivative_samples += 1
    # The solve's probe(): sample(q) (non-objective), then sampled_candidate.
    code = st_sample(st, q, False, &trial)
    if code == UNEVALUABLE:
        return UNEVALUABLE
    if code != OK:
        return code
    norm = c.merit(trial.r)
    if not st.has_sc or norm < st.sc_norm:
        st.sc = trial; st.sc_norm = norm; st.has_sc = True
    for i in range(st.m):
        column[i] = (trial.r[i]-base.r[i])/step
        if not isfinite(column[i]):
            return UNEVALUABLE
    for i in range(st.n):
        disp[i] = (q[i]-st.x[i])/step
    same_branch[0] = same_br(trial.br, base.br)
    return OK


cdef int jacobian(SolveState* st, const Sample* base, double* matrix) except -1:
    cdef NativeCorrector c = <NativeCorrector>st.c
    cdef double lengths[40]
    cdef int nlen, idx, axis, i, k, code
    cdef double h, nominal, length
    cdef bint has_nominal, matched, nominal_prioritized, has_last, has_zero, informative, same, any_missing, cand_ok
    cdef int last_id = -1, zero_id = -2
    cdef bint dep_any
    cdef double seen[64*NMAX]
    cdef int nseen
    cdef double col[NMAX]
    cdef double disp[NMAX]
    cdef double last_col[NMAX]
    cdef double last_disp[NMAX]
    cdef double zero_col[NMAX]
    cdef double zero_disp[NMAX]
    cdef double cmp_col[NMAX]
    cdef double cmp_disp[NMAX]
    cdef double columns[NMAX*NMAX]
    cdef double displacements[NMAX*NMAX]
    cdef bint required[NMAX]
    cdef bint missing[NMAX]
    cdef bint cmp_same, identity
    cdef double tmp[NMAX*NMAX]
    for i in range(st.m):
        required[i] = fabs(base.r[i]/c.rscale[i]) > 1.
        if c.target >= 0.:
            required[i] = required[i] or ((fabs(base.r[i]) > 0.) if c.target == 0. else (fabs(base.r[i]) >= c.target))
    for axis in range(st.n):
        h = st.stencil[axis]
        matched = False; has_last = False; has_zero = False; nseen = 0
        has_nominal = False; nominal = 0.
        nlen = 0
        for k in range(18):
            lengths[nlen] = FACTORS[k]*h; nlen += 1
        if c.steps[axis] > h:
            nominal = c.steps[axis]; has_nominal = True
            for k in range(18):
                lengths[nlen] = FACTORS[k]*nominal; nlen += 1
        nominal_prioritized = False
        idx = 0
        while idx < nlen:
            length = lengths[idx]
            code = jacobian_probe(st, base, axis, length, False, seen, &nseen, col, disp, &same)
            if code == UNEVALUABLE:
                idx += 1;continue
            if code != OK:
                return code
            informative = False
            for i in range(st.m):
                if col[i] != 0.:
                    informative = True;break
            if not informative and has_nominal and not nominal_prioritized:
                for k in range(nlen, idx+1, -1):
                    lengths[k] = lengths[k-1]
                lengths[idx+1] = FACTORS[0]*nominal; nlen += 1
                nominal_prioritized = True
            if informative or not has_last:
                for i in range(st.m):last_col[i] = col[i]
                for i in range(st.n):last_disp[i] = disp[i]
                has_last = True; last_id = idx
            if not same:
                idx += 1;continue
            if not informative:
                for i in range(st.m):zero_col[i] = col[i]
                for i in range(st.n):zero_disp[i] = disp[i]
                has_zero = True; zero_id = idx
                idx += 1;continue
            if has_nominal and fabs(length) < nominal:
                any_missing = False
                for i in range(st.m):
                    missing[i] = required[i] and col[i] == 0.
                    if missing[i] and c.has_dep:
                        # Projection can move more coordinates: any of their dependencies count
                        dep_any = False
                        for k in range(st.n):
                            if disp[k] != 0. and c.dep[i*st.n+k]:
                                dep_any = True
                        missing[i] = dep_any
                    if missing[i]:any_missing = True
                if any_missing:
                    code = jacobian_probe(st, base, axis, FACTORS[0]*nominal, True, seen, &nseen, cmp_col, cmp_disp, &cmp_same)
                    if code != OK and code != UNEVALUABLE:
                        return code
                    if code == OK and cmp_same:
                        cand_ok = False
                        for i in range(st.m):
                            if missing[i] and cmp_col[i] != 0.:
                                cand_ok = True
                        if cand_ok:
                            for i in range(st.m):col[i] = cmp_col[i]
                            for i in range(st.n):disp[i] = cmp_disp[i]
                            c.nominal_restorations += 1
            for i in range(st.m):columns[axis*NMAX+i] = col[i]
            for i in range(st.n):displacements[axis*NMAX+i] = disp[i]
            matched = True
            break
        if not matched:
            if has_last:
                for i in range(st.m):columns[axis*NMAX+i] = last_col[i]
                for i in range(st.n):displacements[axis*NMAX+i] = last_disp[i]
                if not (has_zero and zero_id == last_id):
                    c.branch_secants += 1
            elif has_zero:
                for i in range(st.m):columns[axis*NMAX+i] = zero_col[i]
                for i in range(st.n):displacements[axis*NMAX+i] = zero_disp[i]
            else:
                return DERIV_UNAVAILABLE
    # matrix (m x n, row-major): column axis = columns[axis]
    for i in range(st.m):
        for axis in range(st.n):
            matrix[i*st.n+axis] = columns[axis*NMAX+i]
    if c.has_project:
        identity = True
        for axis in range(st.n):
            for i in range(st.n):
                if displacements[axis*NMAX+i] != (1. if i == axis else 0.):
                    identity = False
        if not identity:
            # matrix = solve(D.T, matrix.T).T with D[:, axis] = that stencil's displacement
            for i in range(st.n):
                for axis in range(st.n):
                    tmp[i*st.n+axis] = displacements[axis*NMAX+i]
            for i in range(st.m):
                for axis in range(st.n):
                    columns[axis*st.m+i] = matrix[i*st.n+axis]    # M^T row-major (n x m)
            if lu_solve_t(st.n, tmp, columns, st.m, displacements):
                return DERIV_UNAVAILABLE
            for axis in range(st.n):
                for i in range(st.m):
                    matrix[i*st.n+axis] = displacements[axis*st.m+i]
    for i in range(st.m*st.n):
        if not isfinite(matrix[i]):
            return DERIV_UNAVAILABLE
    return OK


cdef int run_solve(SolveState* st, const Predictor* predictor) except -1:
    cdef NativeCorrector c = <NativeCorrector>st.c
    cdef int code, i, j, axis, k, nprop, f
    cdef Sample trial
    cdef double norm, reference, trial_norm, predicted, ratio, size, v, floor_, cap, column_norm, candidate, length
    cdef double delta[NMAX]
    cdef double q[NMAX]
    cdef double qin[NMAX]
    cdef double mq[NMAX]
    cdef double w[NMAX]
    cdef double failed_disp[NMAX]
    cdef double failed_size
    cdef bint has_failed, moved, reused, has_relaxed, sampled_improves, has_pc, has_current, preferred, modeled, rep, dup, improves
    cdef Sample relaxed_trial
    cdef double relaxed_pred, relaxed_ratio
    cdef double tried[64*NMAX]
    cdef int ntried
    cdef double factors4[4]
    cdef double prop_q[2*NMAX*NMAX]
    cdef int prop_axis[2*NMAX]
    cdef int prop_kind[2*NMAX]
    cdef double prop_pred[2*NMAX]
    cdef bint prop_has_pred[2*NMAX]
    cdef int order[2*NMAX]
    cdef Sample pc
    cdef double pc_norm
    cdef int pc_kind
    cdef Sample cur
    cdef double cur_norm
    cdef int cur_kind
    cdef double col[NMAX]
    cdef double wm[NMAX*NMAX]
    cdef double radius_step
    factors4[0] = 1.; factors4[1] = .5; factors4[2] = .25; factors4[3] = .1
    code = st_sample(st, st.x, True, &st.value)
    if code == UNEVALUABLE:
        return -2
    if code != OK:
        return code
    remember(st, st.x, c.merit(st.value.r))
    if c.reuse and predictor != NULL and predictor.valid:
        v = 0.
        for i in range(st.n):
            v = pmax(v, fabs(predictor.x[i]-st.x[i])/c.step_limit[i])
        if v <= 1. and same_br(predictor.br, st.value.br):
            for i in range(st.m*st.n):
                st.matrix[i] = predictor.matrix[i]
            for i in range(st.n):
                st.matrix_x[i] = st.x[i]
            st.matrix_value = st.value
            st.has_matrix = True
    while st.used < st.max_nfev:
        if c.target >= 0.:
            pass
        elif st.return_key0 < st.progress_ratio*(1.-PROGRESS_FRACTION):
            st.progress_ratio = st.return_key0; st.progress_iteration = st.iterations
            st.progress_models = st.iterations-st.reuses
        elif (st.iterations-st.progress_iteration >= PROGRESS_WINDOW or
              (st.fast_stagnation and st.progress_ratio > NEAR_CLOSURE_RATIO
               and st.iterations-st.reuses-st.progress_models >= FAR_PROGRESS_WINDOW)):
            st.termination = T_STAGNATION
            return OK
        rep = False
        for i in range(st.n):
            if st.radius*c.step_limit[i] > 8.*spacing(pmax_np(1., fabs(st.x[i]))):
                rep = True
        if not rep:
            st.termination = T_STAGNATION
            return OK
        if st.iterations >= st.max_iterations:
            return ITER_BUDGET
        st.iterations += 1
        norm = c.merit(st.value.r)
        reference = st.recent[0].norm
        for i in range(1, st.nrecent):
            reference = pmax(reference, st.recent[i].norm)
        for i in range(st.n):
            st.stencil[i] = pmax_np(st.stencil[i], 8.*spacing(pmax_np(1., fabs(st.x[i]))))
        st.has_sc = False
        reused = c.reuse and st.has_matrix
        if reused:
            for i in range(st.n):
                if st.matrix_x[i] != st.x[i]:
                    reused = False
            for i in range(st.m*st.n):
                if not isfinite(st.matrix[i]):
                    reused = False
        code = OK
        if reused:
            st.reuses += 1
        else:
            code = jacobian(st, &st.value, st.matrix)
            if code == OK:
                for i in range(st.n):
                    st.matrix_x[i] = st.x[i]
                st.matrix_value = st.value; st.matrix_exact = True; st.has_matrix = True
                st.refreshes += 1
            elif code != DERIV_UNAVAILABLE:
                return code
        if code == OK:
            code = c.direction(st.matrix, st.value.r, st.x, st.radius, 1e-5 if c.target < 0. else 1e-10, delta)
        if code != OK:
            st.has_matrix = False; st.matrix_exact = False
            for i in range(st.n):
                delta[i] = 0.
        moved = False
        ntried = 0
        has_relaxed = False
        has_failed = False; failed_size = 1e308*10.
        if st.has_matrix:
            for f in range(4):
                for i in range(st.n):
                    qin[i] = st.x[i]+factors4[f]*delta[i]
                if not proposal(st, qin, &st.value, q):
                    continue
                dup = False
                for k in range(ntried):
                    dup = True
                    for i in range(st.n):
                        if tried[k*NMAX+i] != q[i]:
                            dup = False;break
                    if dup:break
                if dup:
                    continue
                if ntried < 64:
                    for i in range(st.n):tried[ntried*NMAX+i] = q[i]
                    ntried += 1
                code = st_sample(st, q, True, &trial)
                if code == UNEVALUABLE:
                    continue
                if code != OK:
                    return code
                trial_norm = c.merit(trial.r)
                for i in range(st.m):
                    v = 0.
                    for j in range(st.n):
                        v += st.matrix[i*st.n+j]*(q[j]-st.x[j])
                    w[i] = (st.value.r[i]+v)/c.rscale[i]
                predicted = norm-vector_norm(st.m, w)
                ratio = (norm-trial_norm)/predicted if predicted > 0. else 0.
                improves = significant_decrease(norm, trial_norm) and ratio >= MODEL_AGREEMENT
                if not improves:
                    size = 0.
                    for i in range(st.n):
                        size = pmax(size, fabs(q[i]-st.x[i])/c.step_limit[i])
                    if 0. < size and size < failed_size:
                        for i in range(st.n):failed_disp[i] = q[i]-st.x[i]
                        failed_size = size; has_failed = True
                    if not has_relaxed:
                        if (trial_norm > norm and predicted > 0. and significant_decrease(reference, trial_norm)
                                and trial_norm < reference-SUFFICIENT_DECREASE*predicted):
                            dup = False
                            for k in range(st.nrecent):
                                dup = True
                                for i in range(st.n):
                                    if st.recent[k].q[i] != q[i]:
                                        dup = False;break
                                if dup:break
                            if not dup:
                                relaxed_trial = trial; relaxed_pred = predicted; relaxed_ratio = ratio
                                has_relaxed = True
                    continue
                if st.has_sc and significant_decrease(trial_norm, st.sc_norm):
                    accept(st, &st.sc, 1, False, True)
                    moved = True
                    break
                if ratio > .75 and f == 0:
                    st.radius = pmin(1., 2.*st.radius)
                accept(st, &trial, 0, False, False)
                moved = True
                break
        if moved:
            continue
        if reused:
            st.has_matrix = False; st.matrix_exact = False
            continue
        sampled_improves = st.has_sc and significant_decrease(norm, st.sc_norm)
        st.polls += 1
        has_pc = False
        nprop = 0
        for i in range(st.m):
            w[i] = st.value.r[i]/c.rscale[i]
            if st.has_matrix:
                for j in range(st.n):
                    wm[i*st.n+j] = st.matrix[i*st.n+j]/c.rscale[i]
        for axis in range(st.n):
            radius_step = pmin_np(st.radius*c.step_limit[axis], st.stencil[axis])
            modeled = False
            length = 0.
            if st.has_matrix:
                for i in range(st.m):
                    col[i] = wm[i*st.n+axis]
                column_norm = vector_norm(st.m, col)
                if 0. < column_norm and column_norm < 1e308*10.:
                    v = 0.
                    for i in range(st.m):
                        v += (col[i]/column_norm)*w[i]
                    candidate = -v/column_norm
                    if candidate == candidate:
                        cap = st.radius*c.step_limit[axis]
                        length = pmin_np(pmax_np(candidate, pmax(c.lower[axis]-st.x[axis], -cap)),
                                         pmin(c.upper[axis]-st.x[axis], cap))
                        modeled = length != 0.
            for k in range(1 if modeled else 2):
                for i in range(st.n):
                    qin[i] = st.x[i]
                if modeled:
                    qin[axis] += length
                else:
                    qin[axis] += radius_step if k == 0 else -radius_step
                if not proposal(st, qin, &st.value, q):
                    continue
                dup = False
                for j in range(ntried):
                    dup = True
                    for i in range(st.n):
                        if tried[j*NMAX+i] != q[i]:
                            dup = False;break
                    if dup:break
                if dup:
                    continue
                if ntried < 64:
                    for i in range(st.n):tried[ntried*NMAX+i] = q[i]
                    ntried += 1
                for i in range(st.n):
                    prop_q[nprop*NMAX+i] = q[i]
                prop_axis[nprop] = axis
                prop_kind[nprop] = 2 if modeled else 3
                prop_has_pred[nprop] = modeled
                if modeled:
                    for i in range(st.m):
                        v = 0.
                        for j in range(st.n):
                            v += st.matrix[i*st.n+j]*(q[j]-st.x[j])
                        mq[i] = (st.value.r[i]+v)/c.rscale[i]
                    prop_pred[nprop] = norm-vector_norm(st.m, mq)
                else:
                    prop_pred[nprop] = 0.
                nprop += 1
        # stable sort by -predicted (absent / NaN rank as zero)
        for i in range(nprop):
            order[i] = i
        for i in range(1, nprop):
            k = order[i]; j = i-1
            while j >= 0 and sortkey(prop_pred[order[j]], prop_has_pred[order[j]]) > sortkey(prop_pred[k], prop_has_pred[k]):
                order[j+1] = order[j]; j -= 1
            order[j+1] = k
        for k in range(nprop):
            i = order[k]
            code = st_sample(st, &prop_q[i*NMAX], True, &trial)
            if code == UNEVALUABLE:
                continue
            if code != OK:
                return code
            trial_norm = c.merit(trial.r)
            if prop_has_pred[i]:
                ratio = (norm-trial_norm)/prop_pred[i] if prop_pred[i] > 0. else 0.
                improves = significant_decrease(norm, trial_norm) and ratio >= MODEL_AGREEMENT
            else:
                improves = significant_decrease(norm, trial_norm)
            if improves and (not has_pc or trial_norm < pc_norm):
                pc = trial; pc_norm = trial_norm; pc_kind = prop_kind[i]; has_pc = True
        has_current = False
        if sampled_improves:
            cur = st.sc; cur_norm = st.sc_norm; cur_kind = 1; has_current = True
        if has_pc and (not has_current or significant_decrease(cur_norm, pc_norm)):
            cur = pc; cur_norm = pc_norm; cur_kind = pc_kind; has_current = True
        preferred = has_current and (not has_relaxed or norm-cur_norm >= MODEL_AGREEMENT*relaxed_pred)
        if has_relaxed and not preferred:
            size = 0.
            for i in range(st.n):
                size = pmax(size, fabs(relaxed_trial.q[i]-st.x[i])/c.step_limit[i])
            st.radius = pmin(st.radius, size)
            accept(st, &relaxed_trial, 0, True, False)
            continue
        if not has_pc:
            adapt(st, delta, True)
        if has_failed:
            adapt(st, failed_disp, False)
        if has_current:
            accept(st, &cur, cur_kind, False, True)
            continue
    return OBJ_BUDGET


cdef inline double sortkey(double pred, bint has) noexcept nogil:
    if has and pred == pred:
        return -pred
    return 0.
