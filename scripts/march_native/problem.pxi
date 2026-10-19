# Native Curve._correct (em_stationary_boundary) for prescribed alpha or
# prescribed load on the full stationary model (JointProblem coordinates:
# alpha, bank, aileron, elevator, rudder, two wing histories; plus turn).

from libc.stdlib cimport malloc, free as cfree

DEF MEMO = 512


cdef int polar_branch(const Polar* p, double a) noexcept nogil:
    cdef double sign, critical, distance, sa
    if p.aoaLineL <= a and a <= p.aoaLineH:
        return 0
    sign = 1. if A(a, .01) >= p.aoaLineH else -1.
    critical = p.aoaCritH if sign > 0 else p.aoaCritL
    distance = M(a-critical, sign); sa = M(sign, a)
    if distance <= 0:
        return 10+(sign > 0)
    return (100+(sign > 0)*32+(distance < p.parabAngle)*16+(sa <= p.maxDistAng)*8
            +(sa <= pmax(40., p.maxDistAng))*4+(sa <= 90.)*2+(sa <= 140.))


cdef void branch_code(Model m, const Eval* e, int* br) noexcept:
    cdef const Cond* c = &m.conds[e.cond]
    cdef double blend = e.frame.wing.blend
    br[0] = c.piece
    br[1] = 0 if blend == 0 else (2 if blend == 1 else 1)
    br[2] = polar_branch(&c.wing_raw, e.frame.wing.coef[0].aoa)
    br[3] = polar_branch(&c.wing_raw, e.frame.wing.coef[1].aoa)
    br[4] = polar_branch(&c.tp.hstab, e.frame.tail.angles[0])
    br[5] = polar_branch(&c.tp.hstab, e.frame.tail.angles[1])
    br[6] = polar_branch(&c.tp.vstab, e.frame.tail.angles[2])
    br[7] = polar_branch(&c.tp.fuselage, e.frame.tail.angles[3])
    if m.roll_leveling:
        br[8] = 1+(-1 if e.air.alpha <= -12. else (1 if e.air.alpha >= 12. else 0))
    else:
        br[8] = 0
    br[9] = 0


def decode_branch(code):
    """Python derivative-branch tuple fragment for one polar code."""
    if code == 0:return ('linear',)
    if code < 100:return ('precrit', 1. if code == 11 else -1.)
    c = code-100
    return ('postcrit', 1. if c & 32 else -1., bool(c & 16), bool(c & 8), bool(c & 4), bool(c & 2), bool(c & 1))


cdef class CurveProblem(NativeCorrector):
    cdef Model model
    cdef double speed_kmh
    cdef int fixed           # 0: alpha; 3: elevator; nx: load (turn fixed); -1: discrete-SEP event; -3: generic event (S3)
    cdef int ev_kind, ev_axis, ev_side   # S3 generic event: 1 stall, 2 wing force, 3 control (axis 0-2, side 0/1)
    cdef double alpha, load, target
    cdef double z[NMAX]
    cdef double zlower[NMAX]
    cdef double zupper[NMAX]
    cdef int nz
    cdef int free_idx[NMAX]
    # memo of evaluated coordinates (Curve._correct value())
    cdef int nmemo
    cdef double memo_q[MEMO*NMAX]
    cdef double memo_r[MEMO*NMAX]
    cdef double memo_extra[MEMO]
    cdef int memo_br[MEMO*BMAX]
    cdef Eval* memo_eval
    cdef public long aircraft_evaluations
    cdef int last_slot
    # propeller aircraft: JointProblem propulsion coordinates and rows
    cdef int nx, nprop, nprows
    cdef bint prop
    cdef PropModel pm
    cdef int req[NP]
    cdef int nreq
    cdef PropPrep preps[8]
    cdef double prep_key[8][3]
    cdef int nprep, prep_next
    cdef double* memo_row
    cdef bint* memo_conv

    def __cinit__(self):
        self.memo_eval = <Eval*>malloc(MEMO*sizeof(Eval))
        self.memo_row = <double*>malloc(MEMO*11*sizeof(double))
        self.memo_conv = <bint*>malloc(MEMO*sizeof(bint))
        if self.memo_eval == NULL or self.memo_row == NULL or self.memo_conv == NULL:raise MemoryError()
        self.nx = 7;self.nprop = 0;self.nprows = 0;self.prop = False

    def __dealloc__(self):
        if self.memo_eval != NULL:cfree(self.memo_eval)
        if self.memo_row != NULL:cfree(self.memo_row)
        if self.memo_conv != NULL:cfree(self.memo_conv)

    cdef PropPrep* prepared(self, double load, double alpha, double bank) except NULL:
        cdef int i
        for i in range(self.nprep):
            if self.prep_key[i][0] == alpha and self.prep_key[i][1] == bank and self.prep_key[i][2] == load:
                return &self.preps[i]
        if self.nprep < 8:
            i = self.nprep;self.nprep += 1
        else:
            i = self.prep_next;self.prep_next = (self.prep_next+1) % 8
        self.pm.dt_aircraft = self.model.dt
        self.pm.prepare(self.speed_kmh/3.6, load, alpha, bank, self.model.G, &self.preps[i])
        self.prep_key[i][0] = alpha;self.prep_key[i][1] = bank;self.prep_key[i][2] = load
        return &self.preps[i]

    cdef void expand(self, const double* y, double* q) noexcept nogil:
        cdef int i
        for i in range(self.nz):
            q[i] = self.z[i]
        for i in range(self.n):
            q[self.free_idx[i]] = y[i]

    cdef inline double request(self, const double* q) noexcept nogil:
        if self.fixed == self.nx or self.fixed == -2:
            return self.load
        return hypot2(1., q[self.nx])

    cdef int value(self, const double* q) except -2:
        """Memoized value(q); returns the memo slot, or -1 when unavailable."""
        cdef int k, i, slot
        cdef bint same
        cdef Eval* e
        cdef double extra, load
        cdef PropSample ps
        cdef double inner[MAXPATH]
        cdef PropPrep* prep
        for k in range(self.nmemo):
            same = True
            for i in range(self.nz):
                if self.memo_q[k*NMAX+i] != q[i]:
                    same = False;break
            if same:
                if self.memo_ok(k):
                    return k
                # Production never memoizes an unavailable trial: it is evaluated again.
                self.aircraft_evaluations += 1
                self.last_slot = k
                return -1
        if self.nmemo >= MEMO:
            raise RuntimeError('native correction memo full')
        slot = self.nmemo
        e = &self.memo_eval[slot]
        load = self.request(q)
        if self.prop:
            prep = self.prepared(load, q[0], q[1])
            if self.pm.propulsion_at(prep, &q[7], self.req, self.nreq, inner, &ps):
                # BranchUnavailable before the aircraft evaluation: never memoized or charged
                return -1
            for i in range(11):
                self.model.cur_phase[i] = ps.row[i];self.memo_row[slot*11+i] = ps.row[i]
            self.memo_conv[slot] = ps.converged
            self.model.has_phase = True
            try:
                self.model.evaluate(self.speed_kmh/3.6, load, q, e)
            finally:
                self.model.has_phase = False
        else:
            self.model.evaluate(self.speed_kmh/3.6, load, q, e)
        self.aircraft_evaluations += 1
        self.last_slot = slot
        for i in range(self.nz):
            self.memo_q[slot*NMAX+i] = q[i]
        for i in range(5):
            self.memo_r[slot*NMAX+i] = e.residual[i]
        self.memo_r[slot*NMAX+5] = e.angle_residual[0]/10.
        self.memo_r[slot*NMAX+6] = e.angle_residual[1]/10.
        for i in range(self.nprows):
            self.memo_r[slot*NMAX+7+i] = inner[i]
        if self.fixed == -2:
            extra = 0.
        elif self.fixed == 0:
            extra = (q[0]-self.alpha)/10.
        elif self.fixed == 3:
            extra = q[3]-self.alpha
        elif self.fixed == self.nx:
            extra = (hypot2(1., q[self.nx])-self.load)/10.
        elif self.fixed == -3:
            extra = self.event_value(e)
        else:
            # CurveSolver.event: production discrete energy-step reporting, ps - level
            ensure_complete(self.model, e)
            extra = e.kin.ps-self.target
        self.memo_extra[slot] = extra
        branch_code(self.model, e, &self.memo_br[slot*BMAX])
        self.nmemo += 1
        return slot if self.memo_ok(slot) else -1

    cdef double event_value(self, const Eval* e) noexcept:
        """Curve.events (em_stationary_boundary) residuals on the value record, same arithmetic:
        stall (v['stall_margin']-.002)/10; wing force 1.-max(maximum_wing_load_ratios)-5e-5
        (Python max keeps the first of equal values); control (non-Instructor)
        (+-1.)*(allocation commands[axis]-allocation bounds[axis][side])-3e-5."""
        cdef double m
        if self.ev_kind == 1:
            return (e.stall_margin-.002)/10.
        if self.ev_kind == 2:
            m = e.wing_ratios[0]
            if e.wing_ratios[1] > m:m = e.wing_ratios[1]
            return 1.-m-5e-5
        return (1. if self.ev_side == 0 else -1.)*(e.alloc.commands[self.ev_axis]-e.alloc.bounds[self.ev_axis][self.ev_side])-3e-5

    cdef bint memo_ok(self, int slot) noexcept:
        cdef int i
        if not isfinite(self.memo_extra[slot]):
            return False
        for i in range(7+self.nprows):
            if not isfinite(self.memo_r[slot*NMAX+i]):
                return False
        return True

    cdef int evaluate(self, const double* y, Sample* out) except -1:
        cdef double q[NMAX]
        cdef int slot, i
        self.expand(y, q)
        slot = self.value(q)
        if slot < 0:
            return UNEVALUABLE
        for i in range(7+self.nprows):
            out.r[i] = self.memo_r[slot*NMAX+i]
        if self.fixed == -1 or self.fixed == -3:
            out.r[7+self.nprows] = self.memo_extra[slot]
        for i in range(BMAX):
            out.br[i] = self.memo_br[slot*BMAX+i]
        out.slot = slot
        return OK

    cdef double mach_at(self, const double* x) noexcept:
        cdef double quat[4]
        cdef double vel[3]
        cdef Air air
        turn_quaternion(x[0], x[1], quat)
        world_to_body_air(quat, self.speed_kmh/3.6, vel)
        air_cache(vel, self.model.altitude, &air)
        return air.mach

    cdef int project(self, const double* y, const Sample* base, double* out) except -1:
        cdef double q[NMAX]
        cdef double t[NMAX]
        cdef double mach0, step
        cdef const Eval* be = &self.memo_eval[base.slot]
        cdef int i, k
        cdef double[10] offsets = [1., -1., 2., -2., 4., -4., 8., -8., 16., -16.]
        for i in range(self.n):
            out[i] = y[i]
        if not self.model.conds[be.cond].rough:
            return OK
        self.expand(y, q)
        mach0 = be.air.mach
        if self.mach_at(q) == mach0:
            return OK
        step = pmax(fabs(spacing32(q[1])), 1e-7)
        for k in range(10):
            for i in range(self.nz):
                t[i] = q[i]
            t[1] += offsets[k]*step
            if not (-90. <= t[1] and t[1] <= 90.):
                continue
            if self.mach_at(t) == mach0:
                for i in range(self.n):
                    out[i] = t[self.free_idx[i]]
                return OK
        return OK


cdef double spacing32(double v) noexcept nogil:
    # numpy.spacing(numpy.float32(v)) as a float
    cdef float f = <float>v
    cdef float g
    if f >= 0:
        g = nextafterf(f, <float>3.4028234663852886e38*<float>10.)
    else:
        g = nextafterf(f, -<float>3.4028234663852886e38*<float>10.)
    return <double>g-<double>f

cdef extern from "math.h" nogil:
    float nextafterf(float x, float y)


cdef class NativePredictor:
    """A Curve.predictors entry produced by the native corrector."""
    cdef Predictor p
    cdef public int n, m

    def branch(self):
        return tuple(self.p.br[i] for i in range(BMAX))


def bulk_charge(kind, int count):
    from em_trim_work import _current
    work = _current.get()
    chain = []
    while work is not None:
        if work.counts[kind]+count > work.limits.get(kind, float('inf')):
            from em_trim_work import WorkLimit
            raise WorkLimit(work, kind)
        chain.append(work);work = work.parent
    for owner in chain:owner.counts[kind] += count


cdef class CurveCorrection:
    """Native replacement for Curve.correct(seed, alpha=...) and correct(seed, load=...)."""
    cdef public Model model
    cdef CurveProblem problem
    cdef public dict stats
    cdef public bint charge
    cdef public object finisher, layout
    cdef public int verify

    def __init__(self, Model model):
        self.model = model
        self.finisher = None
        self.verify = 0
        self.layout = dict(names=['alpha_deg', 'bank_deg', 'aileron', 'elevator', 'rudder', 'left_wing_history_deg',
                                  'right_wing_history_deg'], lower=[-180., -90., -1., -1., -1., -1000., -1000.],
                           upper=[180., 90., 1., 1., 1., 1000., 1000.], initial=[0.]*7)
        self.problem = CurveProblem()
        self.problem.model = model
        self.stats = dict(calls=0, evaluations=0, derivative_samples=0, closed=0, unresolved=0, finish_python=0)
        self.charge = True

    def set_prop_layout(self, PropModel pm, names, lower, upper, initial):
        """JointProblem coordinate layout of a propeller aircraft (attitude, histories, propulsion)."""
        cdef CurveProblem P = self.problem
        P.prop = True;P.pm = pm
        P.nprop = len(names)-7;P.nx = len(names);P.nprows = pm.nrows
        self.layout = dict(names=list(names), lower=list(map(float, lower)), upper=list(map(float, upper)),
                           initial=list(map(float, initial)))

    def solve(self, curve, seed, alpha=None, load=None, int max_nfev=40, max_evaluations=None, double event_tolerance=2e-6,
              event_target=None, elevator=None, event_kind=None):
        """Returns (x, load, closed, info) or None (production returns None before finishing)."""
        import math
        cdef CurveProblem P = self.problem
        cdef double x5[5]
        cdef double initial[NMAX]
        cdef double n, n0, turn, turn_upper, cap
        cdef int i, k, nx, nz
        cdef Result res
        cdef Predictor pred
        cdef NativePredictor np_pred
        cdef double y[NMAX]
        cdef double zf[NMAX]
        cdef int slot
        cdef Eval* e
        self.stats['calls'] += 1
        if (alpha is not None)+(load is not None)+(event_target is not None)+(elevator is not None)+(event_kind is not None) != 1:
            raise ValueError('native correction: exactly one of alpha / elevator / load / SEP event / generic event')
        if seed.get('sideslip_attitude_deg', 0.) != 0.:raise ValueError('zero sideslip only')
        L = self.layout
        nx = P.nx;nz = nx+1
        cap = curve.cap
        P.speed_kmh = curve.speed
        sol = seed['solution']
        for i in range(5):x5[i] = float(sol[i])
        n0 = float(seed['load_g']);n = n0
        if alpha is not None:
            n = min(cap, max(1.001, n+.15*(alpha-x5[0])))
            x5[0] = alpha
        if elevator is not None:
            x5[3] = elevator
        if load is None:
            n = min(cap, max(1.001, n))
        else:
            n = load
        x5[1] += math.degrees(math.acos(1./n)-math.acos(1./max(1., n0)))
        # JointProblem(core, speed, n, x, request_indices).initial, or the seed's own coordinates
        names = seed.get('stationary_coordinate_names')
        if names == L['names']:
            coords = seed['stationary_coordinates']
            for i in range(nx):initial[i] = float(coords[i])
            for i in range(5):initial[i] = x5[i]
        else:
            for i in range(5):initial[i] = x5[i]
            initial[5] = x5[0];initial[6] = x5[0]
            for i in range(7, nx):initial[i] = L['initial'][i]
            for i in range(nx):initial[i] = min(max(initial[i], L['lower'][i]), L['upper'][i])
        request = tuple(seed.get('stationary_request_indices', ()))
        P.nreq = len(request)
        if P.nreq > NP:raise ValueError('request indices')
        for i in range(P.nreq):
            if isinstance(request[i], bool) or int(request[i]) != request[i]:return None
            P.req[i] = int(request[i])
        turn = math.sqrt(max(0., n*n-1.));turn_upper = math.sqrt(cap**2-1.)
        P.nz = nz
        for i in range(nx):
            P.zlower[i] = L['lower'][i];P.zupper[i] = L['upper'][i]
        P.zlower[nx] = 0.;P.zupper[nx] = turn_upper
        for i in range(nz):
            if not P.zupper[i] > P.zlower[i]:return None
        for i in range(nx):P.z[i] = initial[i]
        P.z[nx] = turn
        for i in range(nz):P.z[i] = min(max(P.z[i], P.zlower[i]+1e-10), P.zupper[i]-1e-10)
        if alpha is not None:
            P.fixed = 0;P.alpha = alpha;P.z[0] = alpha
        elif elevator is not None:
            P.fixed = 3;P.alpha = elevator;P.z[3] = elevator
        elif load is not None:
            P.fixed = nx;P.load = load;P.z[nx] = math.sqrt(max(0., load*load-1.))
        elif event_kind is not None:
            P.fixed = -3;P.ev_kind = int(event_kind[0]);P.ev_axis = int(event_kind[1]);P.ev_side = int(event_kind[2])
            if not (1 <= P.ev_kind <= 3 and 0 <= P.ev_axis <= 2 and 0 <= P.ev_side <= 1):raise ValueError('native event kind')
        else:
            P.fixed = -1;P.target = float(event_target)
        for i in range(nz):
            if P.z[i] < P.zlower[i] or P.z[i] > P.zupper[i]:return None
        steps = [.002, .002, .0002, .0002, .0002, .002, .002]+[.00001]*(nx-7)+[.0002]
        scales = [10., 30., .2, .2, .2, 10., 10.]+[1.]*(nx-7)+[5.]
        limits = [8., 20., .4, .4, .4, 8., 8.]+[.3]*(nx-7)+[5.]
        rs = [2e-4, 2e-4, 5e-4, 5e-4, 5e-4, 2e-5, 2e-5]+[1e-5]*P.nprows
        k = 0
        for i in range(nz):
            if i == P.fixed:continue
            P.free_idx[k] = i
            P.lower[k] = P.zlower[i];P.upper[k] = P.zupper[i]
            P.steps[k] = steps[i];P.scale[k] = scales[i];P.step_limit[k] = limits[i]
            k += 1
        P.n = k;P.m = 7+P.nprows+(0 if P.fixed >= 0 else 1)
        for i in range(7+P.nprows):P.rscale[i] = rs[i]
        if P.fixed < 0:P.rscale[7+P.nprows] = event_tolerance
        P.has_project = True
        P.nmemo = 0;P.aircraft_evaluations = 0;P.last_slot = -1;P.nprep = 0;P.prep_next = 0
        P.evaluations = 0;P.derivative_samples = 0;P.nominal_comparisons = 0;P.nominal_restorations = 0;P.branch_secants = 0
        slot = P.value(P.z)
        if slot < 0:
            self._charge(P)
            return None
        for i in range(P.n):y[i] = P.z[P.free_idx[i]]
        key = (P.fixed, tuple(P.free_idx[i] for i in range(P.n)), request, 0.) if P.fixed >= 0 else None
        previous = curve.predictors.get(key) if key is not None else None
        pred.valid = False
        if isinstance(previous, NativePredictor):
            pred = (<NativePredictor>previous).p
        P.solve(y, max_nfev, 0 if max_evaluations is None else max_evaluations, &pred, False, &res)
        if res.termination == -1:
            self._charge(P)
            return None
        if key is not None and res.predictor.valid:
            np_pred = NativePredictor();np_pred.p = res.predictor;np_pred.n = P.n;np_pred.m = P.m
            curve.predictors[key] = np_pred
        P.expand(res.x, zf)
        slot = P.value(zf)
        e = &P.memo_eval[slot]
        nr = 7+P.nprows
        rmax = max(abs(P.memo_r[slot*NMAX+i]) for i in range(nr))
        angular = max(abs(e.rate_residual[i]) for i in range(3))
        constraint = abs(P.memo_extra[slot])
        curve.last_correction = dict(force_error_g=e.force_error_g, angular_error_rad_s2=angular,
                                     residual_max=rmax, constraint_residual=constraint)
        closed = (all(math.isfinite(P.memo_r[slot*NMAX+i]) for i in range(nr)) and math.isfinite(P.memo_extra[slot])
                  and e.force_error_g <= 2e-4 and angular <= 5e-5
                  and all(abs(P.memo_r[slot*NMAX+i]) <= rs[i] for i in range(nr)) and constraint <= event_tolerance)
        # Production finish replays the returned state unless it was the last evaluation.
        if P.last_slot != slot:P.aircraft_evaluations += 1
        self._charge(P)
        self.stats['closed' if closed else 'unresolved'] += 1
        x = [zf[i] for i in range(nx)]
        return x, (P.load if P.fixed == nx else math.hypot(1., zf[nx])), closed, dict(
            termination=res.termination, nfev=res.nfev, evaluations=res.evaluations, slot=slot)

    def _charge(self, CurveProblem P):
        self.stats['evaluations'] += P.aircraft_evaluations
        self.stats['derivative_samples'] += P.derivative_samples
        if not self.charge:return
        if P.aircraft_evaluations:bulk_charge('aircraft_evaluations', P.aircraft_evaluations)
        if P.derivative_samples:bulk_charge('derivative_samples', P.derivative_samples)
        if P.nominal_comparisons:bulk_charge('nominal_sensitivity_comparisons', P.nominal_comparisons)
        if P.nominal_restorations:bulk_charge('nominal_sensitivity_restorations', P.nominal_restorations)
        if P.branch_secants:bulk_charge('branch_secant_predictors', P.branch_secants)


# ---------------------------------------------------------------- frozen history
TERMINATIONS = {1: 'numerical_target', 2: 'stagnation', 3: 'evaluation_budget', 4: 'objective_budget', 5: 'iteration_budget'}


cdef class FrozenProblem(NativeCorrector):
    """em_stationary_baseline.stationary_frozen_point: wing histories only, aircraft fixed."""
    cdef Model model
    cdef double speed, load
    cdef double x5[5]
    cdef int nmemo
    cdef double memo_q[MEMO*2]
    cdef double memo_r[MEMO*2]
    cdef int memo_br[MEMO*BMAX]
    cdef bint memo_finite[MEMO]
    cdef Eval* memo_eval
    cdef public long aircraft_evaluations

    def __cinit__(self):
        self.memo_eval = <Eval*>malloc(MEMO*sizeof(Eval))
        if self.memo_eval == NULL:raise MemoryError()

    def __dealloc__(self):
        if self.memo_eval != NULL:cfree(self.memo_eval)

    cdef int evaluate(self, const double* y, Sample* out) except -1:
        cdef int k, i, slot = -1
        cdef double q[7]
        cdef Eval* e
        for k in range(self.nmemo):
            if self.memo_q[2*k] == y[0] and self.memo_q[2*k+1] == y[1]:
                slot = k;break
        if slot < 0:
            if self.nmemo >= MEMO:raise RuntimeError('native frozen memo full')
            slot = self.nmemo;self.nmemo += 1
            for i in range(5):q[i] = self.x5[i]
            q[5] = y[0];q[6] = y[1]
            e = &self.memo_eval[slot]
            self.model.evaluate(self.speed, self.load, q, e)
            self.aircraft_evaluations += 1
            self.memo_q[2*slot] = y[0];self.memo_q[2*slot+1] = y[1]
            self.memo_r[2*slot] = e.angle_residual[0]/10.;self.memo_r[2*slot+1] = e.angle_residual[1]/10.
            branch_code(self.model, e, &self.memo_br[slot*BMAX])
            self.memo_br[slot*BMAX] = e.cond
        out.r[0] = self.memo_r[2*slot];out.r[1] = self.memo_r[2*slot+1]
        for i in range(BMAX):out.br[i] = self.memo_br[slot*BMAX+i]
        out.slot = slot
        return OK


cdef class FrozenPoint:
    """Callable replacement for stationary_frozen_point on supported solvers."""
    cdef public Model model
    cdef FrozenProblem problem
    cdef public dict stats

    def __init__(self, Model model):
        self.model = model
        self.problem = FrozenProblem();self.problem.model = model
        self.problem.n = 2;self.problem.m = 2
        for i in range(2):
            self.problem.lower[i] = -1000.;self.problem.upper[i] = 1000.;self.problem.steps[i] = .002
            self.problem.scale[i] = 10.;self.problem.step_limit[i] = 8.;self.problem.rscale[i] = 2e-5
        self.problem.has_project = False
        self.stats = dict(calls=0, evaluations=0, derivative_samples=0)

    def solve(self, core, double speed, double load, x, propulsion, history_initial, bint charge=True):
        import numpy as np
        frames = self.model.set_propulsion(propulsion)
        try:
            value = self._solve(core, speed, load, x, propulsion, history_initial, charge)
        finally:
            self.model.has_phase = False
        if propulsion is not None:
            value['propulsion'] = propulsion;value['propulsion_phase_frames'] = frames
        return value

    def _solve(self, core, double speed, double load, x, propulsion, history_initial, bint charge=True):
        import numpy as np
        cdef FrozenProblem P = self.problem
        cdef Result res
        cdef Predictor pred
        cdef NativePredictor np_pred
        cdef double y[2]
        cdef int i
        self.stats['calls'] += 1
        key = (speed, load, getattr(core, 'sideslip_attitude_deg', 0.))
        previous = getattr(core, '_stationary_frozen_predictor', None)
        nearby = (previous is not None and previous['key'] == key and previous['propulsion'] is propulsion
                  and np.max(abs(np.asarray(x)-previous['attitude'])/np.array([8., 20., .4, .4, .4])) <= 1.)
        initial = previous['angles'] if nearby else history_initial if history_initial is not None else [x[0]]*2
        P.speed = speed;P.load = load
        for i in range(5):P.x5[i] = float(x[i])
        P.nmemo = 0;P.aircraft_evaluations = 0
        P.evaluations = 0;P.derivative_samples = 0;P.nominal_comparisons = 0;P.nominal_restorations = 0;P.branch_secants = 0
        y[0] = float(initial[0]);y[1] = float(initial[1])
        pred.valid = False
        if nearby and isinstance(previous['predictor'], NativePredictor):
            pred = (<NativePredictor>previous['predictor']).p
        P.solve(y, 120, 0, &pred, False, &res)
        self.stats['evaluations'] += P.aircraft_evaluations;self.stats['derivative_samples'] += P.derivative_samples
        if charge:
            if P.aircraft_evaluations:bulk_charge('aircraft_evaluations', P.aircraft_evaluations)
            if P.derivative_samples:bulk_charge('derivative_samples', P.derivative_samples)
            if P.nominal_comparisons:bulk_charge('nominal_sensitivity_comparisons', P.nominal_comparisons)
            if P.nominal_restorations:bulk_charge('nominal_sensitivity_restorations', P.nominal_restorations)
            if P.branch_secants:bulk_charge('branch_secant_predictors', P.branch_secants)
        if res.termination == -1:
            raise RuntimeError('native frozen history: initial state unevaluable')
        slot = res.value.slot
        value = value_record(self.model, &P.memo_eval[slot], True, True)
        np_pred = None
        if res.predictor.valid:
            np_pred = NativePredictor();np_pred.p = res.predictor;np_pred.n = 2;np_pred.m = 2
        fx = np.array([res.x[0], res.x[1]])
        core._stationary_frozen_predictor = dict(key=key, propulsion=propulsion, attitude=np.asarray(x).copy(),
                                                 angles=fx.copy(), predictor=np_pred)
        value['_stationary_frozen_history'] = dict(coordinates=fx.tolist(), residual=[res.value.r[0], res.value.r[1]],
            termination=TERMINATIONS[res.termination], nfev=res.nfev, evaluations=res.evaluations,
            numerical_target_reached=bool(res.success),
            scope='Conditional history closure at fixed aircraft coordinates and propulsion; no aircraft admission')
        return value


def correct(CurveCorrection cc, curve, seed, alpha=None, load=None, int max_nfev=40, max_evaluations=None,
            double event_tolerance=2e-6, event_target=None, elevator=None, event_kind=None):
    """Curve.correct(seed, alpha=...|load=...) with the native correction and the
    unchanged baseline finish (JointProblem.finish -> certify -> chart sample)."""
    import numpy as np
    from time import perf_counter
    from em_stationary import JointProblem
    t0 = perf_counter()
    out = cc.solve(curve, seed, alpha=alpha, load=load, max_nfev=max_nfev, max_evaluations=max_evaluations,
                   event_tolerance=event_tolerance, event_target=event_target, elevator=elevator, event_kind=event_kind)
    t1 = perf_counter()
    cc.stats['solve_s'] = cc.stats.get('solve_s', 0.)+(t1-t0)
    if out is None:
        return None
    x7, n, closed, info = out
    cdef CurveProblem P = cc.problem
    cdef int slot = info['slot']
    s = curve.solver
    if cc.finisher is not None:
        t2 = perf_counter()
        p = cc.finisher.finish(P, slot, curve.speed, n, x7, P.aircraft_evaluations)
        t3 = perf_counter()
        cc.stats['native_finish_s'] = cc.stats.get('native_finish_s', 0.)+(t3-t2)
        if cc.verify:
            q = python_finish(cc, P, slot, curve, seed, n, x7)
            diffs = row_diff(q, p)
            cc.stats['verified'] = cc.stats.get('verified', 0)+1
            if diffs:
                cc.stats['verify_mismatch'] = cc.stats.get('verify_mismatch', 0)+1
                bad = cc.stats.setdefault('verify_examples', [])
                if len(bad) < 5:bad.append(dict(speed=curve.speed, load=n, diffs=diffs[:8]))
            p = q
    else:
        p = python_finish(cc, P, slot, curve, seed, n, x7)
    if not closed:
        if p.get('valid'):
            curve.retain(p, 'Boundary correction did not meet its balance or constraint tolerance',
                         correction=dict(curve.last_correction))
        return None
    if not p['converged']:
        return None
    return p


def python_finish(CurveCorrection cc, CurveProblem P, int slot, curve, seed, double n, x7):
    import numpy as np
    from time import perf_counter
    from em_stationary import JointProblem
    t1 = perf_counter()
    value = value_record(cc.model, &P.memo_eval[slot], True, True)
    value['_stationary_pitch_fractions'] = {}
    s = curve.solver
    problem = JointProblem(s.core(), curve.speed, n, x7[:5], seed.get('stationary_request_indices', ()))
    problem.load = n;problem.speed = curve.speed/3.6
    xa = np.asarray(x7, dtype=float)
    if P.prop:
        # The baseline propulsion record of the corrected state (the same deterministic sample)
        inner, prop = problem.propulsion_at(xa)
        frames = cc.model.set_propulsion(prop);cc.model.has_phase = False
        value['propulsion'] = prop;value['propulsion_phase_frames'] = frames
        value['_stationary_pitch_fractions'] = {path[1]: min(1., max(0., float(xa[j])))
                                                for j, path in enumerate(problem.paths, problem.prop_start) if path[0] == 'pitch'}
        if [float(v) for v in inner] != [P.memo_r[slot*NMAX+7+i] for i in range(P.nprows)]:
            cc.stats['propulsion_mismatch'] = cc.stats.get('propulsion_mismatch', 0)+1
    problem.last = value
    problem._last_residual = np.r_[value['residual'], value['stationary_angle_residual']/10.,
                                   [P.memo_r[slot*NMAX+7+i] for i in range(P.nprows)]]
    problem._last_key = problem.sample_key(xa)
    problem.calls = P.aircraft_evaluations
    t2 = perf_counter()
    point = problem.finish(xa);point['selected_candidates'] = []
    t3 = perf_counter()
    p = s.sample(dict(point, stationary_candidates=[point], stationary_attempts=[]))
    t4 = perf_counter()
    cc.stats['setup_s'] = cc.stats.get('setup_s', 0.)+(t2-t1)
    cc.stats['finish_s'] = cc.stats.get('finish_s', 0.)+(t3-t2)
    cc.stats['sample_s'] = cc.stats.get('sample_s', 0.)+(t4-t3)
    return p


def row_diff(a, b, path=''):
    """Paths where two chart rows differ (exact values and types)."""
    out = []
    if isinstance(a, dict):
        if not isinstance(b, dict):return [path]
        if set(a) != set(b):return [path+'{keys:'+','.join(sorted(map(str, set(a) ^ set(b))))+'}']
        for k in a:out += row_diff(a[k], b[k], path+'/'+str(k))
        return out
    if isinstance(a, list):
        if not isinstance(b, list) or len(a) != len(b):return [path]
        for i, (u, v) in enumerate(zip(a, b)):out += row_diff(u, v, path+'[%d]' % i)
        return out
    if type(a) != type(b) or a != b:
        if not (isinstance(a, float) and isinstance(b, float) and a != a and b != b):return [path]
    return out


def joint_correct(CurveCorrection cc, problem, initial, int max_nfev=120, bint warm_start=False):
    """JointProblem.correct(initial, offset=0) for the production target: the native
    corrector at fixed speed and load (fast stagnation, predictor cache, warm start)."""
    import math
    import numpy as np
    from types import SimpleNamespace
    cdef CurveProblem P = cc.problem
    cdef int i, nx = P.nx, k, slot
    cdef Result res
    cdef Predictor pred
    cdef NativePredictor np_pred
    cdef double y[NMAX]
    cdef Eval* e
    L = cc.layout
    if list(problem.coordinate_names) != L['names']:
        raise ValueError('native point correction: coordinate layout')
    solver = problem.solver
    cache = getattr(solver, '_stationary_predictors', None)
    key = (None if cache is None else (id(solver.engine), problem.speed, solver.sideslip_attitude_deg,
           tuple(problem.coordinate_names), tuple(problem.request_indices), 0))
    previous = cache.get(key) if cache is not None else None
    x0 = np.asarray(initial, dtype=float).copy()
    limits = np.asarray(problem.correction_step_limits)
    if previous is not None and warm_start:
        anchor = previous['coordinates']
        if len(anchor) == len(x0) and np.max(abs(anchor-x0)/limits, initial=0.) <= 1.:
            x0[problem.history_start:] = anchor[problem.history_start:]
    request = tuple(problem.request_indices)
    P.nreq = len(request)
    for i in range(P.nreq):P.req[i] = int(request[i])
    P.speed_kmh = problem.speed*3.6;P.load = problem.load;P.fixed = -2
    P.nz = nx;P.n = nx;P.m = 7+P.nprows
    for i in range(nx):
        P.free_idx[i] = i;P.zlower[i] = problem.lower[i];P.zupper[i] = problem.upper[i]
        P.lower[i] = problem.lower[i];P.upper[i] = problem.upper[i];P.steps[i] = problem.steps[i]
        P.scale[i] = problem.correction_scales[i];P.step_limit[i] = problem.correction_step_limits[i]
        P.z[i] = x0[i]
    for i in range(P.m):P.rscale[i] = problem.correction_residual_scales[i]
    P.has_project = True
    dep = problem.derivative_dependencies
    P.has_dep = True
    for i in range(P.m):
        for k in range(nx):P.dep[i*nx+k] = bool(dep[i][k])
    P.nmemo = 0;P.aircraft_evaluations = 0;P.last_slot = -1;P.nprep = 0;P.prep_next = 0
    P.evaluations = 0;P.derivative_samples = 0;P.nominal_comparisons = 0;P.nominal_restorations = 0;P.branch_secants = 0
    for i in range(nx):y[i] = min(max(x0[i], P.lower[i]), P.upper[i])
    pred.valid = False
    if previous is not None and isinstance(previous['predictor'], NativePredictor):
        pred = (<NativePredictor>previous['predictor']).p
    try:
        P.solve(y, max_nfev, 0, &pred, True, &res)
    finally:
        P.has_dep = False
        cc._charge(P)
    if res.termination == -1:
        from em_trim_numerics import UnevaluableTrial
        raise UnevaluableTrial('Nonfinite stationary residual; correction unresolved')
    slot = res.value.slot
    e = &P.memo_eval[slot]
    q = x0.copy()
    for i in range(nx):q[i] = res.x[i]
    # The returned sample's physical record (production restores problem.last to it)
    value = value_record(cc.model, e, True, True)
    value['_stationary_pitch_fractions'] = {}
    if P.prop:
        inner, prop = problem.propulsion_at(q)
        frames = cc.model.set_propulsion(prop);cc.model.has_phase = False
        value['propulsion'] = prop;value['propulsion_phase_frames'] = frames
        value['_stationary_pitch_fractions'] = {path[1]: min(1., max(0., float(q[j])))
                                                for j, path in enumerate(problem.paths, problem.prop_start) if path[0] == 'pitch'}
    residual = np.asarray([P.memo_r[slot*NMAX+i] for i in range(P.m)])
    problem.last = value;problem._last_residual = residual;problem._last_key = problem.sample_key(q)
    problem.calls += P.aircraft_evaluations
    predictor = None
    if res.predictor.valid:
        np_pred = NativePredictor();np_pred.p = res.predictor;np_pred.n = P.n;np_pred.m = P.m
        predictor = np_pred
        if cache is not None:
            cache[key] = dict(predictor=np_pred, coordinates=q.copy())
            cache.move_to_end(key)
            while len(cache) > 64:cache.popitem(last=False)
    termination = TERMINATIONS[res.termination]
    messages = dict(numerical_target='Numerical correction target reached',
                    stagnation='No useful componentwise progress; correction unresolved',
                    evaluation_budget='Total correction evaluation budget exhausted',
                    iteration_budget='Correction iteration budget exhausted', objective_budget='Correction budget exhausted')
    return SimpleNamespace(x=q, fun=residual, value=value, predictor=predictor, termination=termination,
                           message=messages[termination], success=bool(res.success), status=1 if res.success else 0,
                           nfev=res.nfev, evaluations=res.evaluations, derivative_evaluations=res.evaluations-res.nfev,
                           iterations=res.iterations, iteration_limit=max_nfev, accepted_steps=res.accepted,
                           nonmonotone_steps=res.nonmonotone, contractions=res.contractions, coordinate_polls=res.polls,
                           unevaluable_trials=res.unavailable, trust_radius=res.radius, jacobian_reuses=res.reuses,
                           jacobian_refreshes=res.refreshes, numerical_target=None,
                           residual_scale=np.asarray(problem.correction_residual_scales), return_metric='scaled_linf_then_l2',
                           return_score=(res.closure_ratio, res.return_key1), closure_ratio=res.closure_ratio,
                           merit_norm=res.return_key1, jacobian_current=bool(res.jacobian_current),
                           jac=np.full((P.m, nx), np.nan))
