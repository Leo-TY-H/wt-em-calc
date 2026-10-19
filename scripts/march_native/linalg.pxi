# Small dense linear algebra for the native corrector, plus CPython 3.11's
# math.hypot (vector_norm), which production uses for every merit and for
# the prescribed load (hypot(1, turn)).

DEF NMAX = 24

cdef double vector_norm(int n, const double* values) noexcept nogil:
    # math.hypot(*values), CPython 3.11 Modules/mathmodule.c vector_norm
    cdef double vec[NMAX]
    cdef double T27 = 134217729.0
    cdef double x, scale, oldcsum, csum = 1.0, frac1 = 0.0, frac2 = 0.0, frac3 = 0.0
    cdef double t, hi, lo, h, mx = 0.0
    cdef int max_e, i
    cdef bint found_nan = False
    for i in range(n):
        x = fabs(values[i]); vec[i] = x
        if x != x:
            found_nan = True
        if x > mx:
            mx = x
    if mx == mx and mx > 1.7976931348623157e308:
        return mx
    if found_nan:
        return 0.0/0.0
    if mx == 0.0 or n <= 1:
        return mx
    frexp(mx, &max_e)
    if max_e >= -1023:
        scale = ldexp(1.0, -max_e)
        for i in range(n):
            x = vec[i]*scale
            t = x*T27
            hi = t-(t-x)
            lo = x-hi
            x = hi*hi
            oldcsum = csum; csum += x; frac1 += (oldcsum-csum)+x
            x = 2.0*hi*lo
            oldcsum = csum; csum += x; frac2 += (oldcsum-csum)+x
            frac3 += lo*lo
        h = sqrt(csum-1.0+(frac1+frac2+frac3))
        x = h
        t = x*T27
        hi = t-(t-x)
        lo = x-hi
        x = -hi*hi
        oldcsum = csum; csum += x; frac1 += (oldcsum-csum)+x
        x = -2.0*hi*lo
        oldcsum = csum; csum += x; frac2 += (oldcsum-csum)+x
        x = -lo*lo
        oldcsum = csum; csum += x; frac3 += (oldcsum-csum)+x
        x = csum-1.0+(frac1+frac2+frac3)
        return (h+x/(2.0*h))/scale
    for i in range(n):
        x = vec[i]/mx
        x = x*x
        oldcsum = csum; csum += x; frac1 += (oldcsum-csum)+x
    return mx*sqrt(csum-1.0+frac1)


cdef inline double hypot2(double a, double b) noexcept nogil:
    cdef double v[2]
    v[0] = a; v[1] = b
    return vector_norm(2, v)


cdef inline double spacing(double v) noexcept nogil:
    # numpy.spacing for positive finite v
    return nextafter(v, 1.7976931348623157e308*10.)-v

cdef extern from "math.h" nogil:
    double nextafter(double x, double y)


cdef int svd_solve(int m, int n, const double* Ain, const double* b, double* x) noexcept nogil:
    """Minimum-norm least squares x = pinv(A) b (numpy.linalg.lstsq, rcond=None).

    One-sided Jacobi SVD on the m x n row-major matrix A; singular values
    below eps*max(m, n)*s_max are treated as zero (numpy's default cutoff).
    Returns 0, or 1 when the iteration does not converge (LinAlgError).
    """
    cdef double U[NMAX*NMAX]
    cdef double V[NMAX*NMAX]
    cdef double s[NMAX]
    cdef double tmp[NMAX]
    cdef int i, j, k, sweep, rotated
    cdef double alpha, beta, gamma, zeta, t, c, sn, ui, uj, smax, cutoff, eps = 2.220446049250313e-16
    cdef int rows = m
    if m < n:
        rows = n
    # U (rows x n) = A padded with zero rows when m < n
    for i in range(rows):
        for j in range(n):
            U[i*n+j] = Ain[i*n+j] if i < m else 0.
    for i in range(n):
        for j in range(n):
            V[i*n+j] = 1. if i == j else 0.
    for sweep in range(60):
        rotated = 0
        for j in range(n-1):
            for k in range(j+1, n):
                alpha = 0.; beta = 0.; gamma = 0.
                for i in range(rows):
                    alpha += U[i*n+j]*U[i*n+j]
                    beta += U[i*n+k]*U[i*n+k]
                    gamma += U[i*n+j]*U[i*n+k]
                if gamma == 0. or fabs(gamma) <= eps*sqrt(alpha*beta):
                    continue
                rotated = 1
                zeta = (beta-alpha)/(2.*gamma)
                t = (1. if zeta >= 0 else -1.)/(fabs(zeta)+sqrt(1.+zeta*zeta))
                c = 1./sqrt(1.+t*t); sn = c*t
                for i in range(rows):
                    ui = U[i*n+j]; uj = U[i*n+k]
                    U[i*n+j] = c*ui-sn*uj; U[i*n+k] = sn*ui+c*uj
                for i in range(n):
                    ui = V[i*n+j]; uj = V[i*n+k]
                    V[i*n+j] = c*ui-sn*uj; V[i*n+k] = sn*ui+c*uj
        if not rotated:
            break
    else:
        return 1
    smax = 0.
    for j in range(n):
        t = 0.
        for i in range(rows):
            t += U[i*n+j]*U[i*n+j]
        s[j] = sqrt(t)
        if s[j] > smax:
            smax = s[j]
    cutoff = eps*(m if m > n else n)*smax
    # x = V diag(1/s) U^T b over the retained values (U columns are s*u)
    for j in range(n):
        tmp[j] = 0.
        if s[j] > cutoff and s[j] > 0.:
            t = 0.
            for i in range(m):
                t += U[i*n+j]*b[i]
            tmp[j] = t/(s[j]*s[j])
    for i in range(n):
        t = 0.
        for j in range(n):
            t += V[i*n+j]*tmp[j]
        x[i] = t
    for i in range(n):
        if not isfinite(x[i]):
            return 1
    return 0


cdef int lu_solve_t(int n, const double* D, const double* Mt, int ncol, double* out) noexcept nogil:
    """out = solve(D^T, Mt) where D, Mt are row-major (Gaussian elimination, partial pivoting)."""
    cdef double Aa[NMAX*NMAX]
    cdef double B[NMAX*NMAX]
    cdef int i, j, k, p
    cdef double f, best, t
    for i in range(n):
        for j in range(n):
            Aa[i*n+j] = D[j*n+i]
        for j in range(ncol):
            B[i*ncol+j] = Mt[i*ncol+j]
    for k in range(n):
        p = k; best = fabs(Aa[k*n+k])
        for i in range(k+1, n):
            if fabs(Aa[i*n+k]) > best:
                best = fabs(Aa[i*n+k]); p = i
        if best == 0.:
            return 1
        if p != k:
            for j in range(n):
                t = Aa[k*n+j]; Aa[k*n+j] = Aa[p*n+j]; Aa[p*n+j] = t
            for j in range(ncol):
                t = B[k*ncol+j]; B[k*ncol+j] = B[p*ncol+j]; B[p*ncol+j] = t
        for i in range(k+1, n):
            f = Aa[i*n+k]/Aa[k*n+k]
            for j in range(k, n):
                Aa[i*n+j] -= f*Aa[k*n+j]
            for j in range(ncol):
                B[i*ncol+j] -= f*B[k*ncol+j]
    for k in range(n-1, -1, -1):
        for j in range(ncol):
            t = B[k*ncol+j]
            for i in range(k+1, n):
                t -= Aa[k*n+i]*out[i*ncol+j]
            out[k*ncol+j] = t/Aa[k*n+k]
    return 0


cdef int bvls(int m, int n, const double* A, const double* b, const double* x_lsq, const double* lb,
              const double* ub, double tol, int max_iter, double* x) noexcept nogil:
    """scipy.optimize._lsq.bvls.bvls (method='bvls' of lsq_linear), small dense."""
    cdef double on_bound[NMAX]
    cdef int free_idx[NMAX]
    cdef int nfree, i, j, k, iteration, it_end, termination, move_to_free, nv, i_min
    cdef double r[NMAX]
    cdef double g[NMAX]
    cdef double Af[NMAX*NMAX]
    cdef double bf[NMAX]
    cdef double z[NMAX]
    cdef double xfree[NMAX]
    cdef double cost, cost_new, cost_change, optimality, t, alpha, amin, gmax
    cdef bint lbv[NMAX]
    cdef bint ubv[NMAX]
    cdef bint any_v
    for i in range(n):
        x[i] = x_lsq[i]; on_bound[i] = 0.
        if x[i] <= lb[i]:
            x[i] = lb[i]; on_bound[i] = -1.
        if x[i] >= ub[i]:
            x[i] = ub[i]; on_bound[i] = 1.
    nfree = 0
    for i in range(n):
        if on_bound[i] == 0.:
            free_idx[nfree] = i; nfree += 1
    bvls_residual(m, n, A, x, b, r, g, &cost)
    iteration = 0
    while nfree > 0:
        iteration += 1
        bvls_reduced(m, n, A, b, x, on_bound, free_idx, nfree, Af, bf, 0)
        if svd_solve(m, nfree, Af, bf, z):
            return 1
        any_v = False
        for k in range(nfree):
            i = free_idx[k]
            lbv[k] = z[k] < lb[i]; ubv[k] = z[k] > ub[i]
            if lbv[k]:
                x[i] = lb[i]; on_bound[i] = -1.; any_v = True
            elif ubv[k]:
                x[i] = ub[i]; on_bound[i] = 1.; any_v = True
            else:
                x[i] = z[k]
        bvls_residual(m, n, A, x, b, r, g, &cost_new)
        cost = cost_new
        if any_v:
            j = 0
            for k in range(nfree):
                if not (lbv[k] or ubv[k]):
                    free_idx[j] = free_idx[k]; j += 1
            nfree = j
        else:
            break
    if max_iter < 0:
        max_iter = n
    it_end = max_iter+iteration
    termination = -1
    optimality = kkt(n, g, on_bound)
    for iteration in range(iteration, it_end):
        if optimality < tol:
            termination = 1
        if termination != -1:
            break
        move_to_free = 0; gmax = g[0]*on_bound[0]
        for i in range(1, n):
            if g[i]*on_bound[i] > gmax:
                gmax = g[i]*on_bound[i]; move_to_free = i
        on_bound[move_to_free] = 0.
        while True:
            nfree = 0
            for i in range(n):
                if on_bound[i] == 0.:
                    free_idx[nfree] = i; nfree += 1
            for k in range(nfree):
                xfree[k] = x[free_idx[k]]
            bvls_reduced(m, n, A, b, x, on_bound, free_idx, nfree, Af, bf, 1)
            if svd_solve(m, nfree, Af, bf, z):
                return 1
            nv = 0; amin = 0.; i_min = -1
            # alphas: lower violations first, then upper (np.hstack order), argmin first
            for k in range(nfree):
                i = free_idx[k]
                if z[k] < lb[i]:
                    alpha = (lb[i]-xfree[k])/(z[k]-xfree[k])
                    if nv == 0 or alpha < amin:
                        amin = alpha; i_min = k
                        lbv[0] = True
                    nv += 1
            for k in range(nfree):
                i = free_idx[k]
                if z[k] > ub[i]:
                    alpha = (ub[i]-xfree[k])/(z[k]-xfree[k])
                    if nv == 0 or alpha < amin:
                        amin = alpha; i_min = k
                        lbv[0] = False
                    nv += 1
            if nv > 0:
                for k in range(nfree):
                    xfree[k] *= 1-amin
                    xfree[k] += amin*z[k]
                    x[free_idx[k]] = xfree[k]
                on_bound[free_idx[i_min]] = -1. if lbv[0] else 1.
            else:
                for k in range(nfree):
                    x[free_idx[k]] = z[k]
                break
        bvls_residual(m, n, A, x, b, r, g, &cost_new)
        cost_change = cost-cost_new
        if cost_change < tol*cost:
            termination = 2
        cost = cost_new
        optimality = kkt(n, g, on_bound)
    return 0


cdef void bvls_residual(int m, int n, const double* A, const double* x, const double* b, double* r, double* g,
                        double* cost) noexcept nogil:
    cdef int i, j
    cdef double t
    for i in range(m):
        t = 0.
        for j in range(n):
            t += A[i*n+j]*x[j]
        r[i] = t-b[i]
    t = 0.
    for i in range(m):
        t += r[i]*r[i]
    cost[0] = .5*t
    for j in range(n):
        t = 0.
        for i in range(m):
            t += A[i*n+j]*r[i]
        g[j] = t


cdef void bvls_reduced(int m, int n, const double* A, const double* b, const double* x, const double* on_bound,
                       const int* free_idx, int nfree, double* Af, double* bf, int unused) noexcept nogil:
    cdef int i, j, k
    cdef double t
    for i in range(m):
        for k in range(nfree):
            Af[i*nfree+k] = A[i*n+free_idx[k]]
        t = 0.
        for j in range(n):
            if on_bound[j] != 0.:
                t += A[i*n+j]*x[j]
        bf[i] = b[i]-t


cdef double kkt(int n, const double* g, const double* on_bound) noexcept nogil:
    cdef double out = 0., v
    cdef int i
    for i in range(n):
        v = g[i]*on_bound[i] if on_bound[i] != 0. else fabs(g[i])
        if i == 0 or v > out:
            out = v
    return out
