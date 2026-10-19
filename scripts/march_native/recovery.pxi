# S5: em_prop_branch_recovery natively (bounded pitch/wake seed discovery and the
# correction of the nominated seeds), for the point solves of the native march.
#
# The frozen-attitude pitch/wake probes of one governed shaft (prop_governor_evaluator
# rows) are the native propulsion sample at the corrected state with that shaft's
# pitch and wake changed; each wake is closed by the native corrector in the Python
# probe's mode (raw numerical target 1e-7, no Jacobian reuse). The scan's grid,
# crossings (scipy brentq) and extremum refinement (scipy bounded minimize_scalar)
# follow the Python source line by line; records and work charges are the same.
# The finite scan stays discovery only: it proves neither coverage nor absence.

from libc.math cimport signbit

DEF PITCH_SAMPLES = 65
DEF TOTAL_SAMPLES = 260
DEF CORRECTIONS = 8
DEF WAKE_NFEV = 24
RECOVERY_REVISION = 'bounded-pitch-wake-discovery-v2'
CORRECTION_NFEV = 40
_CANCEL = []


class ProbeUnavailable(ValueError):
    pass


cdef int check_cancel_py() except -1:
    if not _CANCEL:
        from em_cancellation import check
        _CANCEL.append(check)
    _CANCEL[0]()
    return 0


cdef class RecBudget:
    """em_prop_branch_recovery.Budget: one shared probe and correction allowance per point solve."""
    cdef public int remaining, corrections

    def __init__(self):
        self.remaining = TOTAL_SAMPLES;self.corrections = CORRECTIONS

    cdef int sample(self) except -1:
        check_cancel_py()
        if self.remaining <= 0:raise ProbeUnavailable('shared pitch probe budget')
        self.remaining -= 1
        charge(K_BPROBES, 1)
        return 0


cdef class WakeBlock(NativeCorrector):
    """One shaft's frozen-attitude governor rows (prop_governor_evaluator) at a fixed pitch fraction:
    the wake coordinates of its propellers are the unknowns."""
    cdef PropModel pm
    cdef PropPrep prep
    cdef double q[MAXPATH]
    cdef int flows[MAXPATH]
    cdef int si, nshaft_props
    cdef double assigned[NP]
    cdef bint has_assigned[NP]
    cdef public long calls
    cdef PropSample ps

    cdef int residual(self) except -1:
        """em_prop_branch_recovery.scan residual(z) after q[flows] = z (its kernel and governor charges)."""
        cdef int i
        charge(K_BKERNEL, 1);charge(K_GSAMPLES, 1);charge(K_GPROPS, self.nshaft_props)
        self.calls += 1
        self.pm.sample(&self.prep, self.q, self.assigned, self.has_assigned, &self.ps)
        if not isfinite(self.ps.residual[self.si]):raise ProbeUnavailable('nonfinite propeller probe')
        for i in range(self.m):
            if not isfinite(self.ps.residual[self.flows[i]]):raise ProbeUnavailable('nonfinite propeller probe')
        return 0

    cdef int evaluate(self, const double* y, Sample* out) except -1:
        cdef int i
        for i in range(self.n):self.q[self.flows[i]] = y[i]
        self.residual()
        for i in range(self.m):out.r[i] = self.ps.residual[self.flows[i]]
        for i in range(BMAX):out.br[i] = 0
        out.slot = -1
        return OK


cdef class Scan:
    """em_prop_branch_recovery.scan for one shaft (the cache, failures and records of one wake continuation)."""
    cdef WakeBlock wb
    cdef RecBudget budget
    cdef list initial, flows
    cdef int torque_row, pitch_col, count
    cdef dict cache, failures, row
    cdef list probes, roots, extrema

    def __init__(self, WakeBlock wb, RecBudget budget, list initial, int torque_row, int pitch_col, list flows):
        self.wb = wb;self.budget = budget;self.initial = initial
        self.torque_row = torque_row;self.pitch_col = pitch_col;self.flows = flows
        self.cache = {};self.failures = {};self.probes = [];self.roots = [];self.extrema = []
        self.count = 0
        self.row = dict(revision=RECOVERY_REVISION, probes=self.probes, roots=self.roots, extrema=self.extrema,
                        unresolved_intervals=[], coverage_complete=False)

    cdef tuple at(self, double fraction):
        import numpy as np
        cdef WakeBlock wb = self.wb
        cdef Result res
        cdef double init[MAXPATH]
        cdef double error
        cdef int i, n = len(self.flows)
        check_cancel_py()
        hit = self.cache.get(fraction)
        if hit is not None:return hit
        if fraction in self.failures:raise ProbeUnavailable(self.failures[fraction])
        if self.count >= PITCH_SAMPLES:raise ProbeUnavailable('shaft pitch probe budget')
        self.budget.sample();self.count += 1
        for i in range(wb.pm.npaths):wb.q[i] = self.initial[i]
        wb.q[self.pitch_col] = fraction
        try:
            if n:
                wb.evaluations = wb.derivative_samples = wb.nominal_comparisons = wb.nominal_restorations = wb.branch_secants = 0
                for i in range(n):init[i] = self.initial[self.flows[i]]
                try:
                    wb.solve(init, WAKE_NFEV, 0, NULL, False, &res)
                finally:
                    charge(K_DERIV, wb.derivative_samples);charge(K_NOMCMP, wb.nominal_comparisons)
                    charge(K_NOMRES, wb.nominal_restorations);charge(K_SECANT, wb.branch_secants)
                for i in range(n):wb.q[self.flows[i]] = res.x[i]
            wb.residual()
            error = 0.
            for i in range(n):error = pmax(error, fabs(wb.ps.residual[self.flows[i]]))
            if error > 1e-6:raise ProbeUnavailable('wake did not close')
            q = [wb.q[i] for i in range(wb.pm.npaths)]
            result = (float(wb.ps.residual[self.torque_row]), q)
            self.cache[fraction] = result
            self.probes.append(dict(pitch_fraction=fraction, net_torque_scaled=result[0], wake_residual=error,
                                    wake_coordinates=[q[j] for j in self.flows],
                                    wake_identity='fixed trial seed; uniqueness unproved'))
            return result
        except (ValueError, ArithmeticError, np.linalg.LinAlgError) as error_:
            self.failures[fraction] = str(error_)
            raise ProbeUnavailable(str(error_)) from error_

    cdef int nominate(self, double f, list bracket, kind) except -1:
        from prop_equilibrium import RESIDUAL_TOLERANCE
        r, q = self.at(f)
        if abs(r) > RESIDUAL_TOLERANCE:return 0
        for root in self.roots:
            if abs(root['pitch_fraction']-f) < 1e-8:return 0
        self.roots.append(dict(pitch_fraction=float(f), bracket=list(bracket), kind=kind, net_torque_scaled=r,
                               coordinates=list(q)))
        return 0

    cdef void interval_error(self, double lo, double hi, error):
        self.row['unresolved_intervals'].append(dict(interval=[lo, hi], reason=str(error)))

    cdef double torque(self, double f) except? -1e308:
        """brentq's f: at(f)[0], with scipy's NaN guard."""
        cdef double v = self.at(f)[0]
        if v != v:raise ValueError(f'The function value at x={f} is NaN; solver cannot continue.')
        return v

    cdef double brentq(self, double xa, double xb, double xtol, double rtol, int iters) except? -1e308:
        """scipy.optimize.brentq (Zeros/brentq.c, disp=True) on torque()."""
        cdef double xpre = xa, xcur = xb, xblk = 0., fpre, fcur, fblk = 0., spre = 0., scur = 0., sbis
        cdef double delta, stry, dpre, dblk
        cdef int i, iterations = 0
        fpre = self.torque(xpre)
        fcur = self.torque(xcur)
        if fpre == 0:return xpre
        if fcur == 0:return xcur
        if signbit(fpre) == signbit(fcur):raise ValueError('f(a) and f(b) must have different signs')
        for i in range(iters):
            iterations += 1
            if fpre != 0 and fcur != 0 and signbit(fpre) != signbit(fcur):
                xblk = xpre;fblk = fpre
                spre = scur = xcur-xpre
            if fabs(fblk) < fabs(fcur):
                xpre = xcur;xcur = xblk;xblk = xpre
                fpre = fcur;fcur = fblk;fblk = fpre
            delta = (xtol+rtol*fabs(xcur))/2
            sbis = (xblk-xcur)/2
            if fcur == 0 or fabs(sbis) < delta:return xcur
            if fabs(spre) > delta and fabs(fcur) < fabs(fpre):
                if xpre == xblk:
                    stry = -fcur*(xcur-xpre)/(fcur-fpre)
                else:
                    dpre = (fpre-fcur)/(xpre-xcur)
                    dblk = (fblk-fcur)/(xblk-xcur)
                    stry = -fcur*(fblk*dblk-fpre*dpre)/(dblk*dpre*(fblk-fpre))
                if 2*fabs(stry) < pmin(fabs(spre), 3*fabs(sbis)-delta):
                    spre = scur;scur = stry
                else:
                    spre = sbis;scur = sbis
            else:
                spre = sbis;scur = sbis
            xpre = xcur;fpre = fcur
            if fabs(scur) > delta:xcur += scur
            else:xcur += (delta if sbis > 0 else -delta)
            fcur = self.torque(xcur)
        raise RuntimeError('Failed to converge after %d iterations, value is %f' % (iterations, xcur))

    cdef tuple fminbound(self, double sign, double x1, double x2, double xatol, int maxfun):
        """scipy.optimize._minimize_scalar_bounded of sign*at(f)[0]: (x, success)."""
        import math
        cdef double sqrt_eps = math.sqrt(2.2e-16), golden_mean = .5*(3.-math.sqrt(5.))
        cdef double a = x1, b = x2, fulc, nfc, xf, rat = 0., e = 0., x, fx, fu, ffulc, fnfc, xm, tol1, tol2, r, q, p, si
        cdef int num, flag = 0
        cdef bint golden
        if not (isfinite(x1) and isfinite(x2)):raise ValueError('Optimization bounds must be finite scalars.')
        if x1 > x2:raise ValueError('The lower bound exceeds the upper bound.')
        fulc = a+golden_mean*(b-a)
        nfc = xf = fulc
        x = xf
        fx = sign*self.at(x)[0]
        num = 1
        fu = INFINITY
        ffulc = fnfc = fx
        xm = .5*(a+b)
        tol1 = sqrt_eps*fabs(xf)+xatol/3.
        tol2 = 2.*tol1
        while fabs(xf-xm) > (tol2-.5*(b-a)):
            golden = True
            if fabs(e) > tol1:
                golden = False
                r = (xf-nfc)*(fx-ffulc)
                q = (xf-fulc)*(fx-fnfc)
                p = (xf-fulc)*q-(xf-nfc)*r
                q = 2.*(q-r)
                if q > 0.:p = -p
                q = fabs(q)
                r = e
                e = rat
                if fabs(p) < fabs(.5*q*r) and p > q*(a-xf) and p < q*(b-xf):
                    rat = (p+0.)/q
                    x = xf+rat
                    if (x-a) < tol2 or (b-x) < tol2:
                        si = np_sign(xm-xf)+(1. if xm-xf == 0 else 0.)
                        rat = tol1*si
                else:
                    golden = True
            if golden:
                if xf >= xm:e = a-xf
                else:e = b-xf
                rat = golden_mean*e
            si = np_sign(rat)+(1. if rat == 0 else 0.)
            x = xf+si*np_maximum(fabs(rat), tol1)
            fu = sign*self.at(x)[0]
            num += 1
            if fu <= fx:
                if x >= xf:a = xf
                else:b = xf
                fulc = nfc;ffulc = fnfc
                nfc = xf;fnfc = fx
                xf = x;fx = fu
            else:
                if x < xf:a = x
                else:b = x
                if fu <= fnfc or nfc == xf:
                    fulc = nfc;ffulc = fnfc
                    nfc = x;fnfc = fu
                elif fu <= ffulc or fulc == xf or fulc == nfc:
                    fulc = x;ffulc = fu
            xm = .5*(a+b)
            tol1 = sqrt_eps*fabs(xf)+xatol/3.
            tol2 = 2.*tol1
            if num >= maxfun:
                flag = 1
                break
        if xf != xf or fx != fx or fu != fu:flag = 2
        return xf, flag == 0

    cdef int crossings(self, list nodes) except -1:
        cdef double lo, hi, a, b, f
        for k in range(len(nodes)-1):
            lo = nodes[k];hi = nodes[k+1]
            if lo not in self.cache or hi not in self.cache:continue
            if any(lo <= g <= hi for g in self.failures):continue
            a = self.cache[lo][0];b = self.cache[hi][0]
            if a*b >= 0.:continue
            if any(lo <= r['pitch_fraction'] <= hi for r in self.roots):continue
            try:
                f = self.brentq(lo, hi, 1e-8, 8.881784197001252e-16, 24)
                self.nominate(f, [lo, hi], 'restoring crossing hint' if a > b else 'repelling crossing hint')
                if not any(lo <= r['pitch_fraction'] <= hi for r in self.roots):
                    self.interval_error(lo, hi, 'crossing has no closed torque root; possible discontinuity')
            except (ValueError, ArithmeticError, RuntimeError) as error:
                self.interval_error(lo, hi, error)
        return 0

    cdef dict run(self, bint collapsed):
        from prop_equilibrium import RESIDUAL_TOLERANCE
        cdef double lo, mid, hi, a, b, c, sign, fx
        if collapsed:grid = [0.]
        else:grid = sorted(set([i/8. for i in range(9)]+[float(self.initial[self.pitch_col])]))
        for f in grid:
            try:
                r, _ = self.at(f)
                if abs(r) <= RESIDUAL_TOLERANCE:self.nominate(f, [f, f], 'near zero probe')
            except (ValueError, ArithmeticError) as error:
                self.interval_error(f, f, error)
        self.crossings(grid)
        for k in range(len(grid)-2):
            lo = grid[k];mid = grid[k+1];hi = grid[k+2]
            if lo not in self.cache or mid not in self.cache or hi not in self.cache:continue
            a = self.cache[lo][0];b = self.cache[mid][0];c = self.cache[hi][0]
            if (b-a)*(c-b) >= 0.:continue
            sign = 1. if b < a else -1.
            try:
                fx, success = self.fminbound(sign, lo, hi, 1e-5, 24)
                r, _ = self.at(fx)
                self.extrema.append(dict(pitch_fraction=float(fx), net_torque_scaled=r,
                                         kind='minimum' if sign > 0 else 'maximum'))
                if not success:self.interval_error(lo, hi, 'extremum refinement budget')
                self.nominate(fx, [lo, hi], 'near zero extremum')
                self.crossings(sorted(set([lo, float(fx), hi])))
            except (ValueError, ArithmeticError) as error:
                self.interval_error(lo, hi, error)
        self.row.update(pitch_samples=self.count, kernel_evaluations=self.wb.calls,
                        failed_probes=[dict(pitch_fraction=f, reason=r) for f, r in sorted(self.failures.items())])
        self.roots.sort(key=lambda r: r['pitch_fraction'])
        return self.row


cdef inline double np_maximum(double a, double b) noexcept:
    # numpy.maximum (NaN propagates)
    if a != a or b != b:return a+b
    return a if a >= b else b
