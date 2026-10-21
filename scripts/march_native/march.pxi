# S5: the native march chart engine (speed plan S5, 2026-10-19).
#
# The alpha march (em_march.March / RescueMarch), its column worker
# (em_march.march_worker / column_record) and the speed tracer
# (em_march.MarchTracer) transliterated onto native trial states. Every trial
# is the same native correction and finish as S2-S4 (CurveCorrection.solve_core
# + Finisher.compute); the decisions read the state's fields directly and no
# Python record is built for a trial. Only the states a column keeps become
# chart rows, after the trace (Finisher.row on the stored RowSrc).
#
# The Python march (em_march.py) is the reference this port follows line by
# line; with the same native corrections both take the same path.
#
# Python call-outs (unchanged production code, rare): level flight where the
# cross-speed level correction fails (em_sampling.solve_level), the production
# column search where the march is unresolved, and event corrections the native
# corrector does not cover (Instructor pitch / control bounds, engine RPM).

cimport cython
from libc.stdlib cimport realloc
from libc.math cimport ceil

DEF INFINITY = float('inf')

cdef extern from *:
    """
    #ifdef _WIN32
    #ifndef WIN32_LEAN_AND_MEAN
    #define WIN32_LEAN_AND_MEAN
    #endif
    #include <windows.h>
    static double s5_now(void) {
        static LARGE_INTEGER frequency;
        LARGE_INTEGER count;
        if (!frequency.QuadPart) QueryPerformanceFrequency(&frequency);
        QueryPerformanceCounter(&count);
        return (double)count.QuadPart/(double)frequency.QuadPart;
    }
    #else
    #include <time.h>
    static double s5_now(void) {
        struct timespec t;
        clock_gettime(CLOCK_MONOTONIC, &t);
        return (double)t.tv_sec+1e-9*(double)t.tv_nsec;
    }
    #endif
    """
    double s5_now() nogil

DEF KD_INSTR = 0
DEF KD_CONTROL = 1
DEF KD_RPM = 2
DEF KD_PITCH = 3
DEF KD_PROPPITCH = 4
DEF KD_STALL = 5
DEF KD_FOLD = 6
DEF KD_WING = 7
DEF KD_IAS = 8      # S8: a forward-IAS limit face (wing VNE, flap retraction or damage by IAS; native event 7)
DEF KD_BRAKE = 9    # S10: propeller braking (net propeller thrust negative under power) met along alpha
DEF NKD = 10
# Kind names in the lexical order of the Python strings (sorted() of (root, kind) ties)
KIND_TEXT = ('Instructor pitch', 'control', 'engine RPM', 'pitch response', 'propeller pitch', 'stall', 'trim fold',
             'wing force', 'IAS limit', 'propeller braking')
KIND_CODE = {k: i for i, k in enumerate(KIND_TEXT)}

DEF G_TURN = 9.8100004196167
DEF MAX_STEP_DEG = 8.
DEF S8_MAX_STEP_DEG = 4.   # S8: the alpha march's maximum step under the split rule (its maximum piece width along alpha)
DEF APPROACH_DEG = 2.
DEF OVERSHOOT_DEG = .1
DEF SWITCH_DEG = 12.
DEF ROLL_PROBE = 1e-5
DEF MARCH_NFEV = 12
DEF MARCH_EVALS = 40
DEF REUSE_WINDOW = 60.
DEF MAXPRED = 8
DEF MAXHIST = 256


cdef inline int reason_kind(int r) noexcept nogil:
    # em_pitch_response.KINDS
    if r == R_STALL:return KD_STALL
    if r == R_CONTROL:return KD_CONTROL
    if r == R_WING:return KD_WING
    if r == R_INSTR:return KD_INSTR
    if r == R_RPM:return KD_RPM
    if r == R_PITCH_REV:return KD_PITCH
    if r == R_PROP_BRAKE:return KD_BRAKE
    return -1


cdef inline double turn_rate(double speed, double load) noexcept nogil:
    # em_march.turn_rate
    return G_TURN*sqrt(pmax(0., load*load-1.))/(speed/3.6)*RAD_TO_DEG


DEF S8_INFLECTION = .5   # S8: band_load across an inflection: the chord +- this times the larger end-slope deviation x width
DEF S8_SAMPLES = 32      # S8: points across a piece at which its error bound is evaluated
DEF S9_FOLD_MARGIN = .1  # S9: a level flight with every hard-limit margin above this that stops closing ends at a fold
DEF S9_BEND = .05        # S9: a polar Mach curve overshooting its end value by more than this fraction is checked at quarters
DEF S10_BRAKE_MARGIN = .05   # S10: the braking margin (net propeller thrust in weights) is a march margin below this
DEF S11_NFEV = 30            # S11: the shadow check's prescribed-load corrections (column.pxi ROBUST_NFEV, ROBUST_EVALS)
DEF S11_EVALS = 120
DEF S11_THRUST_EPS = 1e-3     # S11: another state of the same equations has more net thrust by this (weights)


cdef inline void band_load(double va, double na, double sa, double vb, double nb, double sb, double x, double* lo,
                           double* hi) noexcept nogil:
    """S8: the values a function can take at x on [va, vb] given its values and slopes at both ends (sa at va, sb at
    vb). Where the chord slope lies between the end slopes (a slope that changes monotonically: smooth, or with a kink
    anywhere inside) the function lies between the chord and the nearer tangent line; otherwise (an inflection
    inside) within S8_INFLECTION times the larger end-slope deviation from the chord, times the width, of the chord."""
    cdef double h = vb-va, m = (nb-na)/h, chord = na+m*(x-va), ta = na+sa*(x-va), tb = nb+sb*(x-vb), b
    if (sa-m)*(m-sb) >= 0.:
        if sa >= m:
            lo[0] = chord;hi[0] = pmin(ta, tb)
        else:
            lo[0] = pmax(ta, tb);hi[0] = chord
    else:
        b = S8_INFLECTION*pmax(fabs(sa-m), fabs(sb-m))*h
        lo[0] = chord-b;hi[0] = chord+b


cdef struct TState:
    int src            # RowSrc index (native) or -1
    int row            # index into MarchEngine.rows (built or imported Python row) or -1
    int base           # the state this one copies (mark), itself otherwise
    int limit          # index into MarchEngine.limits (envelope_limit) or -1
    double speed, load, alpha, bank, turn, ps, mach, beta
    double sol[5]
    int nx
    double coords[NMAX]
    bint has_coords, layout_names
    int names_id
    int nreq
    int req[NP]
    bint req_ok
    bint converged, valid
    int nreasons
    int reasons[MAXREASONS]
    double force_error_g, angular_error, history_error
    unsigned int mset
    double m[NKD]
    bint has_stall_deg
    double stall_deg
    bint has_wing
    double wing_max
    bint has_instr_rec, instr_has_env
    double instr_env
    bint has_pitch_rec, pitch_margin_ok
    double pitch_margin
    bint has_authority
    double authority
    bint has_cb
    double commands[3]
    double bounds[3][2]
    int ne
    double rpm[NP]
    double rpm_limit[NP]
    bint regulated[NP]
    bint pitch_stops
    int roll_branch    # -1, 0, 1, or 9 for None
    long evaluations
    int rule           # S8: admitted by the chart rule only: 1 a transient over-rev (engine RPM over its limit on the way to a hard limit)
    SlideSpec slide    # S8: shafts sliding on a blade-polar jump (n = 0: a regular state)
    bint brakes        # S11: the propellers' net thrust is negative under power (Fin.brakes; S10: also its rejection reason)
    double thrust      # S11: the propellers' net thrust in weights (NaN: unknown)


cdef struct PKey:
    int fixed
    int nreq
    int req[NP]


cdef struct PredMap:
    # Curve.predictors of one march (insertion ordered)
    int n
    PKey key[MAXPRED]
    Predictor pred[MAXPRED]


cdef inline bint same_key(const PKey* a, const PKey* b) noexcept nogil:
    cdef int i
    if a.fixed != b.fixed or a.nreq != b.nreq:return False
    for i in range(a.nreq):
        if a.req[i] != b.req[i]:return False
    return True


cdef Predictor* pm_get(PredMap* m, const PKey* k) noexcept nogil:
    cdef int i
    for i in range(m.n):
        if same_key(&m.key[i], k):return &m.pred[i]
    return NULL


cdef void pm_set(PredMap* m, const PKey* k, const Predictor* p) noexcept nogil:
    cdef int i
    for i in range(m.n):
        if same_key(&m.key[i], k):
            m.pred[i] = p[0];return
    if m.n < MAXPRED:
        m.key[m.n] = k[0];m.pred[m.n] = p[0];m.n += 1


cdef struct StoreRow:
    double speed, alpha
    int state
    bint has_pred
    Predictor pred


cdef struct Trial:
    int why            # index into MarchEngine.why_text
    double alpha       # NaN for None
    int ok             # -1 None, 0 False, 1 True
    long evals, fresh
    int warm           # -1 None, 0, 1


cdef struct Event:
    int kind           # KD
    int axis           # -1 None, else 2..4 (control)
    int side
    int engine         # engine RPM
    double distance
    int code           # native event code (1 stall, 2 wing force, 3 control) or 0 for Python


class NativeMarchUnsupported(Exception):
    pass


DEF TANGENT_SCALE = 4.   # S7: tangent stencils are 4x the corrector's steps (binary32 rows)


cdef class MarchEngine:
    """The trial states, chart rows and cross-speed store of one march chart solver."""
    cdef public object solver, loop
    cdef CurveCorrection cc
    cdef CurveProblem P
    cdef Finisher fin
    cdef Model model
    cdef TState* st
    cdef int nst, cap_st
    cdef RowSrc* srcs
    cdef int nsrc, cap_src
    cdef public list rows, limits, why_text, names_table
    cdef public dict why_code, solve_of
    cdef public object points, point_solver
    cdef long counts[NCOUNT]
    cdef bint instructor, structural, is_prop, roll_leveling, native_instructor
    # S8: the soft chart limits (solver.soft_admission): engine RPM and the forward-IAS limits pass the march through
    cdef public bint soft
    cdef public bint dive_seed   # S8: level flight continued only from stored level states at lower speeds (dive path)
    cdef public double alpha_max_step   # S8 validation: the alpha march's largest step (0: MAX_STEP_DEG)
    cdef unsigned long long soft_mask, ias_mask
    cdef public double ias_limit      # the lowest enforced forward-IAS limit (m/s), NaN without one
    cdef public double sep_tol
    cdef public list layout_names
    cdef int layout_nx
    cdef StoreRow* store
    cdef int nstore
    cdef bint has_alpha_key, has_level_key
    cdef PKey alpha_key, level_key
    cdef StoreRow* level_store
    cdef int nlevel
    cdef public dict stats
    cdef public object ceilings
    # S5: the envelope engine's exact load caps (by speed), for the curve engine's cheap SEP-solve bound
    cdef public object known_caps
    cdef public list known_speeds
    cdef public double t_solve, t_finish, t_rows, t_point, t_python, t_ledger, t_srows, t_tangent
    # S8: the load and closure of the latest correction (also where it did not close: a fixed-alpha correction ending at
    # the load cap shows that the cap lies before its alpha)
    cdef public double last_load, last_cap
    # S8: an explicit sliding spec for the next corrections (the blade-polar jump check), instead of the seed's
    cdef SlideSpec slide_override
    cdef public bint use_override
    cdef public bint sliding         # S8: the sliding state at blade-polar jumps (WT_MARCH_SLIDE=0: off)
    cdef public bint s9              # S9 (WT_MARCH_S9=0 restores S8 exactly)
    # S9: speeds where the level-flight branch the sweep follows ends at a fold and level flight continues on another
    # branch beyond it (edge probes): no state is carried across them (column seeds, the cross-speed stores)
    cdef public int ncut
    cdef double cut[8]
    cdef public bint last_closed
    # S10 (WT_MARCH_S10=0 restores S9 exactly): a braking propeller state (net thrust negative under power) is rejected,
    # and a correction that lands on one is tried again from coarser governor pitch (brake_reseed)
    cdef public bint s10
    cdef public bint brake               # S10: the braking rule is on (Finisher.brake)
    # S11 (WT_MARCH_S11=0 restores S10 exactly): braking is a choice between states: a braking state is kept, as admitted
    # by its own checks, where no state of the same speed and load with more net thrust (pulling, or braking less: Leo
    # 2026-10-21) is found (brake_reseed); no braking limit
    cdef public bint s11
    cdef public bint brake_limit         # S10's braking limit (s10 rule without s11): the braking face and edge
    cdef bint brake_retry
    cdef int prop_engines
    cdef list engine_shafts

    @cython.boundscheck(True)
    @cython.wraparound(True)
    def __cinit__(self):
        self.st = NULL;self.srcs = NULL;self.store = NULL;self.level_store = NULL

    @cython.boundscheck(True)
    @cython.wraparound(True)
    def __dealloc__(self):
        if self.st != NULL:cfree(self.st)
        if self.srcs != NULL:cfree(self.srcs)
        if self.store != NULL:cfree(self.store)
        if self.level_store != NULL:cfree(self.level_store)

    @cython.boundscheck(True)
    @cython.wraparound(True)
    def __init__(self, solver, loop):
        cdef int i
        entry = loop.native(solver)
        if entry is None:raise NativeMarchUnsupported('native corrections unavailable: '+str(loop.unsupported))
        self.solver = solver;self.loop = loop
        self.cc = entry[0];self.P = self.cc.problem;self.model = self.cc.model
        if self.cc.finisher is None:raise NativeMarchUnsupported('native finish unavailable: '+str(loop.unsupported))
        self.fin = self.cc.finisher
        self.P.inst = self.fin.instructor
        self.P.instr_event = instructor_event
        if self.fin.instructor is not None:(<Instructor>self.fin.instructor).loop_stats = loop.stats
        cfg = solver.config
        self.instructor = bool(cfg['instructor']);self.structural = bool(cfg['structural_limits'])
        self.native_instructor = self.instructor and self.cc.finisher is not None and self.cc.finisher.instructor is not None
        self.is_prop = bool(solver.is_prop);self.roll_leveling = bool(solver.fm.get('RollLeveling', True))
        self.sep_tol = cfg['sep_tolerance_mps']
        self.layout_names = list(self.cc.layout['names']);self.layout_nx = len(self.layout_names)
        if any(name.startswith('pitch:') for name in self.layout_names):
            raise NativeMarchUnsupported('exact-RPM propeller pitch coordinates')
        if cfg.get('stationary_governor_mode', 'pitch_stops') != 'pitch_stops':
            raise NativeMarchUnsupported('governor mode')
        self.cap_st = 4096;self.st = <TState*>malloc(self.cap_st*sizeof(TState))
        self.cap_src = 1024;self.srcs = <RowSrc*>malloc(self.cap_src*sizeof(RowSrc))
        self.store = <StoreRow*>malloc(601*sizeof(StoreRow));self.level_store = <StoreRow*>malloc(201*sizeof(StoreRow))
        if self.st == NULL or self.srcs == NULL or self.store == NULL or self.level_store == NULL:raise MemoryError()
        self.nst = self.nsrc = self.nstore = self.nlevel = 0
        self.has_alpha_key = self.has_level_key = False
        self.rows = [];self.limits = [];self.names_table = [];self.why_text = [];self.why_code = {};self.solve_of = {}
        self.points = None
        self.point_solver = None
        for i in range(NCOUNT):self.counts[i] = 0
        self.stats = dict(columns=0, fallback=0, predictors_offered=0, python_events=0, native_trials=0, python_calls=0,
                          rows_built=0)
        self.ceilings = {}
        self.known_caps = None;self.known_speeds = None
        self.t_solve = self.t_finish = self.t_rows = self.t_point = self.t_python = self.t_ledger = self.t_srows = 0.
        self.t_tangent = 0.
        self.engine_shafts = []
        self.soft = bool(getattr(solver, "soft_admission", False))
        import os
        self.sliding = self.soft and os.environ.get("WT_MARCH_SLIDE", "1") != "0"
        self.s9 = self.soft and os.environ.get("WT_MARCH_S9", "1") != "0"
        self.ncut = 0
        if self.fin is not None and self.fin.pm is not None:self.fin.pm.s9 = self.s9
        self.s10 = self.s9 and os.environ.get("WT_MARCH_S10", "1") != "0"
        self.brake = self.s10 and self.fin is not None and self.fin.brake
        self.s11 = self.brake and self.fin.brake_choice
        self.brake_limit = self.brake and not self.s11
        self.brake_retry = False
        from em_speed_limits import speed_limits
        value = speed_limits(solver.fm, solver.config).get("forward_ias_limit_mps")
        self.ias_limit = NAN if value is None else float(value)
        self.ias_mask = ((<unsigned long long>1) << R_IAS) | ((<unsigned long long>1) << R_FLAP_AUTO) | ((<unsigned long long>1) << R_FLAP_IAS)
        self.soft_mask = ((<unsigned long long>1) << R_RPM) | ((<unsigned long long>1) << R_IAS) | ((<unsigned long long>1) << R_FLAP_AUTO) | ((<unsigned long long>1) << R_FLAP_IAS)
        if self.is_prop:
            p = solver.engine.properties
            for shaft in p['transmissions']:self.engine_shafts.append([link['index'] for link in shaft['engines']])

    # ------------------------------------------------------------------ pools
    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int new_state(self) except -1:
        cdef TState* grown
        if self.nst >= self.cap_st:
            grown = <TState*>realloc(self.st, 2*self.cap_st*sizeof(TState))
            if grown == NULL:raise MemoryError()
            self.st = grown;self.cap_st *= 2
        self.nst += 1
        self.rows.append(None)
        return self.nst-1

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int new_src(self) except -1:
        cdef RowSrc* grown
        if self.nsrc >= self.cap_src:
            grown = <RowSrc*>realloc(self.srcs, 2*self.cap_src*sizeof(RowSrc))
            if grown == NULL:raise MemoryError()
            self.srcs = grown;self.cap_src *= 2
        self.nsrc += 1
        return self.nsrc-1

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int why(self, text) except -1:
        code = self.why_code.get(text)
        if code is None:
            code = len(self.why_text);self.why_text.append(text);self.why_code[text] = code
        return code

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef double sep_cap(self, double speed, double* near) except? -1.:
        """A cheap load bound for an SEP-fixed correction: 1.25x the exact caps interpolated between the
        envelope's columns (``near``: that interpolation), or the exact cap where no columns bracket it."""
        import bisect
        from em_load_limits import configured_ceiling
        cdef double a, b, ca, cb, t
        value = self.ceilings.get(speed)
        if value is not None or self.known_caps is None:
            near[0] = -1.
            return self.cap(speed)
        if self.known_speeds is None or len(self.known_speeds) != len(self.known_caps):
            self.known_speeds = sorted(self.known_caps)
        i = bisect.bisect_left(self.known_speeds, speed)
        if i <= 0 or i >= len(self.known_speeds):
            near[0] = -1.
            return self.cap(speed)
        a = self.known_speeds[i-1];b = self.known_speeds[i]
        ca = self.known_caps[a];cb = self.known_caps[b]
        t = (speed-a)/(b-a) if b > a else 0.
        near[0] = ca+t*(cb-ca)
        return min(float(configured_ceiling(self.solver, speed)), 1.25*near[0])

    cdef double cap(self, double speed) except? -1.:
        value = self.ceilings.get(speed)
        if value is None:
            from em_load_limits import ceiling
            value = float(ceiling(self.solver, speed));self.ceilings[speed] = value
        return value

    # ------------------------------------------------------------------ states
    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int state_from_src(self, int si) except -1:
        """A trial state from its native finish (RowSrc)."""
        import math
        cdef int sid = self.new_state()
        cdef TState* t = &self.st[sid]
        cdef RowSrc* s = &self.srcs[si]
        cdef Eval* e = &s.e
        cdef Fin* f = &s.fin
        cdef int i, k
        cdef double v, best
        t.src = si;t.row = -1;t.base = sid;t.limit = -1;t.rule = 0
        t.slide = s.slide
        for k in range(NP if t.slide.n > 0 else 0):
            if t.slide.flag[k] and t.slide.lo[k] > 0.:t.slide.hint[k] = t.slide.lo[k]
        t.speed = s.speed_kmh/3.6*3.6;t.load = s.load;t.alpha = e.x[0];t.bank = e.x[1]
        t.turn = e.geom.turn_rate*RAD_TO_DEG
        t.ps = e.kin.ps if isfinite(e.kin.ps) else NAN
        t.mach = e.air.mach;t.beta = e.air.beta
        for i in range(5):t.sol[i] = e.x[i]
        t.nx = s.nx;t.has_coords = True;t.layout_names = True;t.names_id = 0
        for i in range(s.nx):t.coords[i] = s.q[i]
        t.nreq = s.nreq;t.req_ok = True
        for i in range(s.nreq):t.req[i] = s.req[i]
        t.converged = f.converged;t.valid = f.row_valid
        t.nreasons = f.nreasons
        for i in range(f.nreasons):t.reasons[i] = f.reasons[i]
        t.brakes = f.brakes
        t.thrust = net_thrust(self.fin.pm, &s.ps.frame)/self.model.weight if self.is_prop and self.fin.pm is not None else NAN
        t.force_error_g = e.force_error_g if isfinite(e.force_error_g) else NAN
        v = pmax(pmax(fabs(e.rate_residual[0]), fabs(e.rate_residual[1])), fabs(e.rate_residual[2]))
        t.angular_error = v if isfinite(v) else NAN
        t.history_error = e.history_error if isfinite(e.history_error) else NAN
        t.evaluations = f.calls
        # nearest_events inputs and margins (em_march.margins on the row)
        t.mset = 0
        t.has_stall_deg = isfinite(e.stall_margin);t.stall_deg = e.stall_margin
        if t.has_stall_deg:
            t.mset |= 1 << KD_STALL;t.m[KD_STALL] = e.stall_margin/10.
        t.has_authority = isfinite(f.authority);t.authority = f.authority
        if t.has_authority and not self.instructor:
            t.mset |= 1 << KD_CONTROL;t.m[KD_CONTROL] = f.authority
        t.has_wing = True;t.wing_max = pmax(e.wing_ratios[0], e.wing_ratios[1])
        if self.structural:
            t.mset |= 1 << KD_WING;t.m[KD_WING] = 1.-t.wing_max
        t.has_pitch_rec = f.has_pitch;t.pitch_margin_ok = False
        if f.has_pitch and not f.pitch.unresolved:
            v = f.pitch.margin[f.pitch.nsamples-1]
            if isfinite(v):
                t.pitch_margin_ok = True;t.pitch_margin = v
                t.mset |= 1 << KD_PITCH;t.m[KD_PITCH] = v
        t.pitch_stops = True
        t.ne = 0
        if self.is_prop:
            t.ne = self.fin.pm.ne
            best = 0.
            for k in range(t.ne):
                t.rpm[k] = s.ps.frame.engines[k].omega*60/(2*M_PI)
                t.rpm_limit[k] = self.fin.pm.engines[k].omega_limit*60/(2*M_PI)
                v = 1.-t.rpm[k]/(t.rpm_limit[k]*(1.+1e-6))
                best = v if k == 0 else pmin(best, v)
                t.regulated[k] = False
            if t.ne:
                t.mset |= 1 << KD_RPM;t.m[KD_RPM] = best
            self.mark_regulated(t)
            if self.brake_limit:
                # S10: the braking margin, where the propellers' net thrust is within S10_BRAKE_MARGIN weights of zero
                v = net_thrust(self.fin.pm, &s.ps.frame)/self.model.weight
                if v < S10_BRAKE_MARGIN:
                    t.mset |= 1 << KD_BRAKE;t.m[KD_BRAKE] = v
        t.has_instr_rec = f.instr != 0;t.instr_has_env = False
        if f.instr == 1:
            t.instr_has_env = True;t.instr_env = f.ires.angle_margin
            if f.ires.converged:
                t.mset |= (1 << KD_INSTR) | (1 << KD_CONTROL)
                t.m[KD_INSTR] = f.ires.angle_margin;t.m[KD_CONTROL] = f.ires.mechanical
        elif f.instr == 2:
            ctl = self.fin.python_instructors[f.instr_index]
            t.instr_has_env = 'envelope_margin' in ctl;t.instr_env = ctl.get('envelope_margin', NAN)
            if ctl.get('converged'):
                t.mset |= (1 << KD_INSTR) | (1 << KD_CONTROL)
                t.m[KD_INSTR] = ctl['envelope_margin'];t.m[KD_CONTROL] = ctl['margins']['control authority with auto trim']
        t.has_cb = True
        for i in range(3):
            t.commands[i] = e.alloc.commands[i];t.bounds[i][0] = f.bounds[i][0];t.bounds[i][1] = f.bounds[i][1]
        t.roll_branch = ((-1 if e.air.alpha <= -12. else 1 if e.air.alpha >= 12. else 0) if self.roll_leveling else 9)
        return sid

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int mark_regulated(self, TState* t) except -1:
        # Curve.events: engines on a shaft whose governor coordinate is strictly inside (0, 1)
        cdef int j, s
        for j in range(t.nx):
            if j < len(self.layout_names):
                name = self.layout_names[j]
                if name.startswith('governor:') and 0. < t.coords[j] < 1.:
                    s = int(name.split(':')[1])
                    for idx in self.engine_shafts[s]:
                        if idx < NP:t.regulated[idx] = True
        return 0

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int import_row(self, row) except -1:
        """A state from a Python chart row (production solves and fallbacks)."""
        import math
        import em_march as Mpy
        cdef int sid = self.new_state()
        cdef TState* t = &self.st[sid]
        cdef int i, k
        cdef double v
        self.rows[sid] = row
        t.src = -1;t.row = sid;t.base = sid;t.limit = -1
        t.rule = 1 if row.get('chart_rule') == 'transient over-rev' else 0
        slide_from_row(row.get('propeller_sliding'), &t.slide)
        t.speed = row['speed_kmh'];t.load = row['load_g']
        sol = row.get('solution')
        t.alpha = row.get('alpha_deg', sol[0] if sol else NAN);t.bank = row.get('bank_deg', sol[1] if sol else NAN)
        t.turn = row.get('turn_dps', NAN)
        ps = row.get('ps_mps');t.ps = ps if ps is not None else NAN
        mach = row.get('mach');t.mach = mach if mach is not None else NAN
        beta = row.get('sideslip_deg');t.beta = beta if beta is not None else NAN
        for i in range(5):t.sol[i] = float(sol[i]) if sol else NAN
        coords = row.get('stationary_coordinates');names = row.get('stationary_coordinate_names')
        t.has_coords = coords is not None
        t.nx = len(coords) if coords is not None else 0
        if t.nx > NMAX:raise NativeMarchUnsupported('coordinate count')
        for i in range(t.nx):t.coords[i] = float(coords[i])
        t.layout_names = names == self.layout_names
        t.names_id = 0
        if not t.layout_names:
            key = tuple(names) if names is not None else None
            if key in self.names_table:t.names_id = 1+self.names_table.index(key)
            else:
                self.names_table.append(key);t.names_id = len(self.names_table)
        request = tuple(row.get('stationary_request_indices', ()))
        if len(request) > NP:raise NativeMarchUnsupported('request indices')
        t.nreq = len(request);t.req_ok = True
        for i in range(t.nreq):
            if isinstance(request[i], bool) or int(request[i]) != request[i]:t.req_ok = False;t.req[i] = 0
            else:t.req[i] = int(request[i])
        t.converged = bool(row.get('converged'));t.valid = bool(row.get('valid'))
        reasons = row.get('reasons', ())
        t.nreasons = min(len(reasons), MAXREASONS)
        for i in range(t.nreasons):t.reasons[i] = reason_code(reasons[i])
        t.brakes = bool(row.get('propeller_braking')) or 'propeller braking' in reasons;t.thrust = NAN
        for name, default in (('force_error_g', 0), ('angular_error_rad_s2', 1), ('history_error', 2)):
            value = row.get(name)
            value = NAN if value is None else float(value)
            if default == 0:t.force_error_g = value
            elif default == 1:t.angular_error = value
            else:t.history_error = value
        t.evaluations = row.get('evaluations') or 0
        t.mset = 0
        for kind, value in Mpy.margins(self.solver, row).items():
            k = KIND_CODE[kind];t.mset |= 1 << k;t.m[k] = value
        stall = row.get('stall_margin_deg', math.inf)
        t.has_stall_deg = stall is not None;t.stall_deg = stall if stall is not None else NAN
        ratios = row.get('wing_load_ratios')
        t.has_wing = bool(ratios) and all(r is not None for r in ratios)
        t.wing_max = max(ratios) if t.has_wing else NAN
        ctl = row.get('instructor')
        t.has_instr_rec = ctl is not None
        t.instr_has_env = ctl is not None and ctl.get('envelope_margin') is not None
        t.instr_env = ctl['envelope_margin'] if t.instr_has_env else NAN
        pr = row.get('pitch_response')
        t.has_pitch_rec = pr is not None
        t.pitch_margin_ok = pr is not None and pr.get('margin') is not None
        t.pitch_margin = pr['margin'] if t.pitch_margin_ok else NAN
        authority = row.get('authority_margin')
        t.has_authority = authority is not None;t.authority = authority if authority is not None else NAN
        commands = row.get('commands');bounds = (ctl or {}).get('control_bounds', row.get('control_bounds'))
        t.has_cb = commands is not None and bounds is not None
        if t.has_cb:
            for i in range(3):
                t.commands[i] = commands[i];t.bounds[i][0] = bounds[i][0];t.bounds[i][1] = bounds[i][1]
        t.pitch_stops = row.get('stationary_governor_mode') == 'pitch_stops'
        prop = row.get('propulsion') or {}
        rpm = list(prop.get('engine_rpm', ()));limits = list(prop.get('engine_rpm_threshold', ()))
        t.ne = min(len(rpm), len(limits), NP)
        for k in range(t.ne):
            t.rpm[k] = rpm[k];t.rpm_limit[k] = limits[k];t.regulated[k] = False
        if self.is_prop and t.has_coords and names is not None:
            for j, name in enumerate(names):
                if name.startswith('governor:') and 0. < coords[j] < 1.:
                    for idx in self.engine_shafts[int(name.split(':')[1])]:
                        if idx < NP:t.regulated[idx] = True
        rb = row.get('roll_leveling_branch')
        t.roll_branch = 9 if rb is None else int(rb)
        return sid

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef object row_of(self, int sid):
        """The chart row of a state (built once; a marked copy is dict(base row) with its envelope_limit)."""
        cdef TState* t = &self.st[sid]
        row = self.rows[sid]
        if row is not None:return row
        cdef double t0 = s5_now()
        if t.base != sid:
            base = self.row_of(t.base)
            row = dict(base)
        elif sid in self.solve_of:
            row = (<PointSolver>self.solve_of[sid][0]).row(sid)
            self.stats['rows_built'] += 1
            self.t_srows += s5_now()-t0
        elif t.src >= 0:
            row = self.fin.row(&self.srcs[t.src])
            self.stats['rows_built'] += 1
        else:
            raise RuntimeError('state without a row source')
        if t.limit >= 0:row['envelope_limit'] = self.limits[t.limit]
        if t.rule == 1 and not row.get('valid'):
            # S8: admitted by the chart rule (a transient over-rev); its own admission record stays as it is
            row = dict(row, valid=True, chart_usable=True, chart_rule='transient over-rev')
        self.rows[sid] = row
        if t.base == sid:self.t_rows += s5_now()-t0
        return row

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int mark(self, int sid, int kind, int axis, dict details) except -1:
        """em_stationary_boundary.mark: a copy of the state with an envelope_limit (built now; its row later)."""
        cdef int cid = self.new_state()
        self.st[cid] = self.st[sid]
        self.st[cid].base = sid;self.st[cid].row = -1;self.rows[cid] = None
        limit = dict(kind=KIND_TEXT[kind], axis=None if axis < 0 else axis, limiting_load_g=self.st[sid].load,
                     certificate_scope='local numerical branch boundary; global coverage unresolved')
        limit.update(details)
        self.st[cid].limit = len(self.limits);self.limits.append(limit)
        return cid

    @cython.boundscheck(True)
    @cython.wraparound(True)
    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef list reason_list(self, int sid):
        cdef TState* t = &self.st[sid]
        if t.row >= 0 and self.rows[sid] is not None and t.base == sid and t.src < 0:
            return list(self.rows[sid]['reasons'])
        return [REASON_TEXT[t.reasons[i]] for i in range(t.nreasons)]

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef object names_of(self, int sid):
        cdef TState* t = &self.st[sid]
        if t.layout_names:return self.layout_names
        if t.names_id == 0:return None
        names = self.names_table[t.names_id-1]
        return None if names is None else list(names)

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef dict compact_native(self, int sid):
        """em_march_curves.compact_state of a native state's row, from its fields."""
        cdef TState* t = &self.st[sid]
        cdef int i
        c = self.fin.constants
        out = dict(speed_kmh=t.speed, load_g=t.load, turn_dps=t.turn, ps_mps=t.ps if isfinite(t.ps) else None,
                   alpha_deg=t.alpha, bank_deg=t.bank, solution=[t.sol[i] for i in range(5)],
                   stationary_coordinates=[t.coords[i] for i in range(t.nx)],
                   stationary_coordinate_names=list(self.layout_names),
                   stationary_request_indices=[t.req[i] for i in range(t.nreq)] if self.is_prop else [],
                   sideslip_attitude_deg=0., sideslip_deg=t.beta if isfinite(t.beta) else None,
                   stationary_governor_mode=c['governor'], model_experiments=dict(c['description']),
                   mach_curve_mode=c['mach_mode'])
        if c['mach_mode'] == 'stable':out['mach_curve_revision'] = c['stable_revision']
        out.update(roll_leveling_branch=None if t.roll_branch == 9 else t.roll_branch, valid=bool(t.valid),
                   reasons=[REASON_TEXT[t.reasons[i]] for i in range(t.nreasons)],
                   force_error_g=t.force_error_g if isfinite(t.force_error_g) else None,
                   angular_error_rad_s2=t.angular_error if isfinite(t.angular_error) else None,
                   history_error=t.history_error if isfinite(t.history_error) else None, converged=bool(t.converged))
        if t.rule == 1:out['chart_rule'] = 'transient over-rev'
        if self.s11 and t.brakes:out['propeller_braking'] = True   # S11: a braking state kept (none with more thrust found)
        return out

    cdef bint physical(self, int sid) noexcept:
        # em_march.physical: converged, reasons nonempty and all physical
        cdef TState* t = &self.st[sid]
        cdef int i
        if not t.converged or not t.nreasons:return False
        for i in range(t.nreasons):
            if not physical_code(t.reasons[i]):return False
        return True

    cdef bint only_reasons(self, int sid, unsigned long long mask) noexcept:
        """S8: a converged state rejected for reasons in ``mask`` only (bit r: reason code r)."""
        cdef TState* t = &self.st[sid]
        cdef int i, r
        if not t.converged or not t.nreasons:return False
        for i in range(t.nreasons):
            r = t.reasons[i]
            if r >= 64 or not (mask >> r) & 1:return False
        return True

    cdef bint passable(self, int sid) noexcept:
        """S8: a state the alpha march passes through: admitted, or (under the chart's soft-limit rule) rejected only for
        soft limits after its hard checks (pitch response, Instructor) passed."""
        if self.st[sid].valid:return True
        return self.soft and self.only_reasons(sid, self.soft_mask)

    cdef bint hard_rejected(self, int sid) noexcept:
        """em_march.physical under the soft-limit rule: a balanced physical rejection by at least one hard limit
        (stall, wing force, control, Instructor, pitch response; engine RPM too without the rule), every other reason
        physical or soft."""
        cdef TState* t = &self.st[sid]
        cdef int i, r
        cdef bint hard = False
        if not self.soft:return self.physical(sid)
        if not t.converged or not t.nreasons:return False
        for i in range(t.nreasons):
            r = t.reasons[i]
            if r < 64 and (self.soft_mask >> r) & 1:continue
            if not physical_code(r):return False
            hard = True
        return hard

    cdef int slide_candidate(self, int sid, SlideSpec* out, double alpha=NAN, double bank=NAN, double load=NAN,
                             bint straddle=True, double speed=NAN, bint transition=False) except -1:
        """S8 (the blade-polar jump check): the sliding spec of a regular propeller state whose shafts (on a pitch stop
        or with a fixed pitch) have a blade-polar jump within PropModel.slide_width of their speed that blocks their
        torque balance (net torque of opposite signs either side of it; any jump there without ``straddle``), at the
        state's flight condition or at the given one (the target of a correction from it); out.n = 0 without.
        S9: the target's flight condition includes its speed (``speed``): a correction continued from a neighbouring
        speed is checked where it is solved (a shaft approaching the jump with speed reaches it only there). With
        ``transition``, a sliding state is checked too, its sliding shafts at their jump speed: the shafts whose jump still
        blocks their torque balance at the target keep sliding (with the state's own sliding coordinates), the others are
        regular there (a turn makes the propellers' conditions differ, so their jumps part)."""
        cdef TState* t = &self.st[sid]
        cdef PropPrep* prep
        cdef double regular[NMAX]
        cdef int i, slot
        out.n = 0
        if not self.is_prop or self.fin is None or self.fin.pm is None:return 0
        if not t.has_coords or not t.layout_names or not t.req_ok:return 0
        if t.slide.n > 0 and not (self.s9 and transition):return 0
        self.P.speed_kmh = speed if self.s9 and isfinite(speed) else t.speed;self.P.nprep = 0;self.P.prep_next = 0
        prep = self.P.prepared(t.load if not isfinite(load) else load, t.coords[0] if not isfinite(alpha) else alpha,
                               t.coords[1] if not isfinite(bank) else bank)
        if t.slide.n > 0:
            for i in range(t.nx):regular[i] = t.coords[i]
            for i in range(NP):
                if t.slide.flag[i]:
                    slot = self.fin.pm.shaft_coordinate(i)
                    if slot >= 0:regular[7+slot] = t.slide.cw[i]+t.slide.lo[i]/300.
            self.fin.pm.jump_near(prep, &regular[7], t.req, t.nreq, out, straddle)
            for i in range(NP):
                if out.n > 0 and out.flag[i] and t.slide.flag[i]:
                    out.c0[i] = t.slide.c0[i];out.cw[i] = t.slide.cw[i];out.frac[i] = t.slide.frac[i]
                    out.hint[i] = t.slide.hint[i];out.theta[i] = t.coords[7+self.fin.pm.shaft_coordinate(i)]-t.slide.c0[i]
            self.stats['s9_transition_checks'] = self.stats.get('s9_transition_checks', 0)+1
        else:
            self.fin.pm.jump_near(prep, &t.coords[7], t.req, t.nreq, out, straddle)
        self.stats['s8_jump_checks'] = self.stats.get('s8_jump_checks', 0)+1
        if out.n:self.stats['s8_jumps_found'] = self.stats.get('s8_jumps_found', 0)+1
        return 0

    cdef bint across(self, double a, double b) noexcept:
        """S9: a branch cut lies between the speeds ``a`` and ``b``."""
        cdef int i
        for i in range(self.ncut):
            if (a-self.cut[i])*(b-self.cut[i]) < 0.:return True
        return False

    cdef void add_cut(self, double v) noexcept:
        if self.ncut < 8:
            self.cut[self.ncut] = v;self.ncut += 1

    cdef bint same_slide(self, const SlideSpec* a, const SlideSpec* b) noexcept:
        """S9: the same set of sliding shafts."""
        cdef int i
        if a.n != b.n:return False
        for i in range(NP):
            if (a.flag[i] if a.n > 0 else False) != (b.flag[i] if b.n > 0 else False):return False
        return True

    cdef bint level_physical(self, int sid) noexcept:
        """em_march.physical for a level-flight state; S9: under the soft-limit rule a level flight over the engine RPM
        limit (a physical limit at 1 g: an edge) that is also over a forward-IAS limit is physical too (S8 took the
        IAS reason for a numerical one, so the march went on from an interior start past the 1 g over-rev edge)."""
        if self.physical(sid):return True
        return self.s9 and self.soft and self.physical_except(sid, self.soft_mask)

    cdef bint ias_soft(self, int sid) noexcept:
        """S8: rejected only for forward-IAS limits (its hard checks passed): below a leaning forward-IAS edge."""
        return self.soft and not self.st[sid].valid and self.only_reasons(sid, self.ias_mask)

    cdef bint ias_any(self, int sid) noexcept:
        """S8: rejected for a forward-IAS limit among other reasons (a level flight past a forward-IAS limit that also
        did not close numerically: the leaning edge may still continue above it)."""
        cdef int i, r
        if not self.soft or self.st[sid].valid:return False
        for i in range(self.st[sid].nreasons):
            r = self.st[sid].reasons[i]
            if r < 64 and (self.ias_mask >> r) & 1:return True
        return False

    cdef bint has_code(self, int sid, int code) noexcept:
        cdef int i
        for i in range(self.st[sid].nreasons):
            if self.st[sid].reasons[i] == code:return True
        return False

    cdef bint brakes(self, int sid) noexcept:
        """S10/S11: the state's propellers brake (net thrust negative under power; S10 rejects it for that)."""
        return self.st[sid].brakes or self.has_code(sid, R_PROP_BRAKE)

    cdef bint soft_kind(self, int k) noexcept:
        """S8: a margin kind that is soft under the chart's rule (engine RPM): no march step, face or bracket on it."""
        return self.soft and k == KD_RPM

    cdef bint physical_except(self, int sid, unsigned long long soft) noexcept:
        """S8: a converged state with at least one physical reason, every other reason in ``soft``."""
        cdef TState* t = &self.st[sid]
        cdef int i, r
        cdef bint hard = False
        if not t.converged or not t.nreasons:return False
        for i in range(t.nreasons):
            r = t.reasons[i]
            if physical_code(r):hard = True
            elif r >= 64 or not (soft >> r) & 1:return False
        return hard

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef bint combined(self, int sid, double* out) noexcept:
        cdef TState* t = &self.st[sid]
        cdef int k
        cdef bint found = False
        for k in range(NKD):
            if t.mset & (1 << k) and not self.soft_kind(k):
                out[0] = t.m[k] if not found else pmin(out[0], t.m[k])
                found = True
        return found

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int limit_kind(self, int hi) noexcept:
        """em_march.limit_kind(solver, high)."""
        cdef TState* t = &self.st[hi]
        cdef int i, k, count = 0, best = -1, only = -1
        cdef unsigned int crossed = 0
        cdef double v = 0.
        for i in range(t.nreasons):
            k = reason_kind(t.reasons[i])
            if k >= 0 and not self.soft_kind(k):crossed |= 1 << k
        if crossed & (1 << KD_INSTR):return KD_INSTR
        for k in range(NKD):
            if crossed & (1 << k):
                count += 1
                if only < 0:only = k
        if count == 1:return only
        for k in range(NKD):
            if crossed & (1 << k) and t.mset & (1 << k):
                if best < 0 or t.m[k] < v:
                    best = k;v = t.m[k]
        if best >= 0:return best
        return only

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int control_axis(self, int sid) noexcept:
        cdef TState* t = &self.st[sid]
        cdef int i, best = 0
        cdef double v, bv = 0.
        if not t.has_cb:return -1
        for i in range(3):
            v = pmin(t.commands[i]-t.bounds[i][0], t.bounds[i][1]-t.commands[i])
            if i == 0 or v < bv:
                bv = v;best = i
        return 2+best

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef bint compatible(self, int a, int b) noexcept:
        """em_march_curves.compatible."""
        cdef TState* x = &self.st[a]
        cdef TState* y = &self.st[b]
        cdef int i
        if x.layout_names != y.layout_names or x.names_id != y.names_id:return False
        if not x.layout_names and x.names_id == 0:return False
        if x.nreq != y.nreq:return False
        for i in range(x.nreq):
            if x.req[i] != y.req[i]:return False
        return x.roll_branch == y.roll_branch and x.pitch_stops == y.pitch_stops

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int nearest_events(self, int sid, Event* out) noexcept:
        """Curve.nearest_events(point): the event list of Curve.events, stably sorted by distance."""
        cdef TState* t = &self.st[sid]
        cdef int n = 0, i, j, axis, side
        cdef Event tmp
        out[n].kind = KD_STALL;out[n].axis = -1;out[n].side = 0;out[n].engine = -1;out[n].code = 1
        out[n].distance = fabs(t.stall_deg)/10. if t.has_stall_deg else INFINITY;n += 1
        if t.pitch_stops:
            for i in range(t.ne):
                out[n].kind = KD_RPM;out[n].axis = -1;out[n].side = 0;out[n].engine = i;out[n].code = 6 if t.rpm_limit[i] > 0. else 0
                out[n].distance = INFINITY if t.regulated[i] else fabs(1.-t.rpm[i]/t.rpm_limit[i]);n += 1
        if self.structural:
            out[n].kind = KD_WING;out[n].axis = -1;out[n].side = 0;out[n].engine = -1;out[n].code = 2
            out[n].distance = fabs(1.-t.wing_max) if t.has_wing else INFINITY;n += 1
        if self.instructor:
            out[n].kind = KD_INSTR;out[n].axis = -1;out[n].side = 0;out[n].engine = -1
            out[n].code = 4 if self.native_instructor else 0
            out[n].distance = fabs(t.instr_env) if t.instr_has_env else INFINITY;n += 1
        if t.has_pitch_rec:
            out[n].kind = KD_PITCH;out[n].axis = -1;out[n].side = 0;out[n].engine = -1;out[n].code = 0
            out[n].distance = fabs(t.pitch_margin) if t.pitch_margin_ok else NAN;n += 1
        if self.brake_limit and (t.mset >> KD_BRAKE) & 1:
            # S10: the braking face (native event 8: the net propeller thrust)
            out[n].kind = KD_BRAKE;out[n].axis = -1;out[n].side = 0;out[n].engine = -1;out[n].code = 8
            out[n].distance = fabs(t.m[KD_BRAKE]);n += 1
        for axis in range(3):
            for side in range(2):
                out[n].kind = KD_CONTROL;out[n].axis = axis+2;out[n].side = side;out[n].engine = -1
                out[n].code = (5 if self.native_instructor else 0) if self.instructor else 3
                if t.has_cb:out[n].distance = fabs(t.commands[axis]-t.bounds[axis][side])
                else:out[n].distance = fabs(t.authority) if t.has_authority else INFINITY
                n += 1
        # stable insertion sort (Python sorted on the distance key)
        for i in range(1, n):
            tmp = out[i];j = i-1
            while j >= 0 and tmp.distance < out[j].distance:
                out[j+1] = out[j];j -= 1
            out[j+1] = tmp
        return n

    # ------------------------------------------------------------------ the correction
    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int correct(self, int seed, const double* sol, double load, bint use_coords, int fixed_kind, double value,
                     int ev_kind, int ev_axis, int ev_side, int max_nfev, int max_evaluations, double event_tolerance,
                     double speed, PredMap* preds, int* evidence) except -2:
        """Curve.correct(seed, ...) natively: a state id, or -1 for None. ``sol``/``load`` replace the seed's
        solution and load (dict(seed, solution=..., load_g=...)). ``evidence``: when not NULL and the
        correction does not close but its state is admitted, the stored state id (Curve.retain), else -1."""
        cdef Seed sd
        cdef TState* t = &self.st[seed]
        cdef int i, slot, code, si, sid, k
        cdef double n, floor, tp
        cdef bint closed
        cdef PKey key
        cdef Predictor pred_out
        cdef Predictor* pred_in = NULL
        cdef Fin* f
        if evidence != NULL:evidence[0] = -1
        for i in range(5):sd.sol[i] = sol[i]
        sd.load = load
        sd.has_coords = t.has_coords and t.layout_names
        if sd.has_coords:
            for i in range(t.nx):sd.coords[i] = t.coords[i]
        sd.nreq = t.nreq;sd.req_ok = t.req_ok
        # S8: the seed's sliding shafts (or the march's explicit choice: slide_override)
        sd.slide = self.slide_override if self.use_override else t.slide
        if self.s9 and sd.has_coords and self.use_override and self.fin is not None and self.fin.pm is not None:
            # S9: shaft by shaft: one leaving the jump gets its regular coordinate at the shaft speed on the side of the
            # jump its theta is nearer; one starting to slide gets its sliding coordinate (c0 plus the side it is on)
            for i in range(NP):
                if t.slide.n > 0 and t.slide.flag[i] and not (sd.slide.n > 0 and sd.slide.flag[i]):
                    slot = self.fin.pm.shaft_coordinate(i)
                    if slot >= 0:
                        sd.coords[7+slot] = t.slide.cw[i]+(t.slide.hi[i] if t.coords[7+slot]-t.slide.c0[i] >= .5 else t.slide.lo[i])/300.
                elif sd.slide.n > 0 and sd.slide.flag[i] and not (t.slide.n > 0 and t.slide.flag[i]):
                    slot = self.fin.pm.shaft_coordinate(i)
                    if slot >= 0:sd.coords[7+slot] = sd.slide.c0[i]+sd.slide.theta[i]
        elif sd.has_coords and t.slide.n > 0 and sd.slide.n == 0 and self.fin is not None and self.fin.pm is not None:
            # leaving the jump: the shaft coordinate back to its regular meaning at the shaft speed of the jump
            for i in range(NP):
                if t.slide.flag[i]:
                    slot = self.fin.pm.shaft_coordinate(i)
                    if slot >= 0:sd.coords[7+slot] = t.slide.cw[i]+t.slide.lo[i]/300.
        for i in range(t.nreq):sd.req[i] = t.req[i]
        if fixed_kind == FIX_EVENT and ev_kind == 6:
            # engine RPM: the event point's limit, rpm_limit*pi/30 (Curve.events)
            self.P.ev_limit = t.rpm_limit[ev_axis]*M_PI/30.
        elif fixed_kind == FIX_EVENT and ev_kind == 7:
            self.P.ev_limit = self.ias_limit
        # S5: SEP-fixed corrections also take a predictor (the curve solver passes its own map)
        if fixed_kind != FIX_EVENT and preds != NULL:
            key.fixed = self.cc.fixed_index(fixed_kind);key.nreq = t.nreq
            for i in range(t.nreq):key.req[i] = t.req[i]
            pred_in = pm_get(preds, &key)
        self.stats['native_trials'] += 1
        cdef double near = -1.
        cdef double cap = self.sep_cap(speed, &near) if fixed_kind == FIX_SEP else self.cap(speed)
        cdef double t0 = s5_now(), t1
        code = self.cc.solve_core(speed, cap, &sd, fixed_kind, value, ev_kind, ev_axis, ev_side, max_nfev,
                                  max_evaluations, event_tolerance, pred_in, &pred_out, &slot, &n, &closed)
        if near > 0. and code >= 0 and n > .8*near:
            # near the interpolated cap: the correction again with the exact cap
            self.stats['sep_cap_resolves'] = self.stats.get('sep_cap_resolves', 0)+1
            cap = self.cap(speed)
            code = self.cc.solve_core(speed, cap, &sd, fixed_kind, value, ev_kind, ev_axis, ev_side, max_nfev,
                                      max_evaluations, event_tolerance, pred_in, &pred_out, &slot, &n, &closed)
        if code < 0 and sd.slide.n > 0 and not self.use_override and sd.has_coords and self.fin.pm is not None:
            # S8: no jump within reach of a sliding seed here (no sliding sample at its start): the regular correction
            for i in range(NP):
                if t.slide.flag[i]:
                    k = self.fin.pm.shaft_coordinate(i)
                    if k >= 0:
                        sd.coords[7+k] = t.slide.cw[i]+(t.slide.hi[i] if self.s9 and t.coords[7+k]-t.slide.c0[i] >= .5
                                                        else t.slide.lo[i])/300.
            sd.slide.n = 0
            self.stats['s8_slide_dropped'] = self.stats.get('s8_slide_dropped', 0)+1
            code = self.cc.solve_core(speed, cap, &sd, fixed_kind, value, ev_kind, ev_axis, ev_side, max_nfev,
                                      max_evaluations, event_tolerance, pred_in, &pred_out, &slot, &n, &closed)
        t1 = s5_now();self.t_solve += t1-t0
        self.last_load = n if code >= 0 else NAN;self.last_cap = cap;self.last_closed = code >= 0 and closed
        if code < 0:return -1
        if pred_out.valid and preds != NULL and fixed_kind != FIX_EVENT:
            pm_set(preds, &key, &pred_out)
        si = self.new_src()
        self.fin.compute(self.P, slot, speed, n, self.P.aircraft_evaluations, &self.srcs[si])
        self.t_finish += s5_now()-t1
        f = &self.srcs[si].fin
        if not closed:
            if f.row_valid and evidence != NULL:
                evidence[0] = self.state_from_src(si)
            else:
                self.nsrc -= 1
            return -1
        if not f.converged:
            self.nsrc -= 1
            return -1
        sid = self.state_from_src(si)
        if not f.row_valid and not (self.soft and self.only_reasons(sid, self.soft_mask)):
            # Rejected trials never become rows: release the stored source (S8: a state rejected only for soft limits
            # keeps it: the chart rule can admit it, as a transient over-rev, after the march)
            if si == self.nsrc-1:
                self.nsrc -= 1;self.st[sid].src = -1
        if self.brake and not self.brake_retry and self.brakes(sid):
            floor = self.st[sid].thrust+S11_THRUST_EPS
            if self.s11 and fixed_kind == FIX_SEP:
                tp = self.brake_shadowed(sid, seed, speed)
                if not isfinite(tp):
                    # S11: no state of this speed and load with more thrust: the braking state is the chart's state here
                    # (a level's braking state can have a state of the same speed and load with more thrust and another SEP:
                    # the shadow check)
                    self.last_load = n;self.last_cap = cap;self.last_closed = True
                    return sid
                # shadowed: the level's state at this speed on the branch with more thrust (closer to its thrust than to
                # the shadowed state's: the same branch at another load)
                floor = .5*(self.st[sid].thrust+tp)
            # (S11: every braking result is searched, also along alpha: another state can appear part-way along alpha,
            # F2G-1 RB 798 km/h from 3.4 g)
            k = self.brake_reseed(sid, seed, fixed_kind, value, ev_kind, ev_axis, ev_side, max_nfev, max_evaluations,
                                  event_tolerance, speed, floor)
            if k >= 0:return k
            self.last_load = n;self.last_cap = cap;self.last_closed = True
        return sid

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef double brake_shadowed(self, int sid, int seed, double speed) except? -1e300:
        """S11: whether a braking state has a state of the same speed and load with more net thrust (the prescribed-load
        correction at its load from coarser governor pitch, brake_reseed: pulling, or braking less, Leo 2026-10-21 'more
        thrust'); then that one is the chart's state there and the braking state is rejected ('propeller braking'), also
        in its row source: that state's net thrust (in weights), else NaN. Used where the correction's own equations do
        not fix the load (an SEP-fixed knot: the state of the same speed and load with more thrust has another SEP)."""
        cdef int q, si
        cdef Fin* f
        q = self.brake_reseed(sid, seed, FIX_LOAD, self.st[sid].load, 0, 0, 0, S11_NFEV, S11_EVALS, 2e-6, speed,
                              self.st[sid].thrust+S11_THRUST_EPS)
        if q < 0:return NAN
        self.stats['s11_brake_shadowed'] = self.stats.get('s11_brake_shadowed', 0)+1
        if self.st[sid].nreasons < MAXREASONS:
            self.st[sid].reasons[self.st[sid].nreasons] = R_PROP_BRAKE;self.st[sid].nreasons += 1
        self.st[sid].valid = False
        si = self.st[sid].src
        if si >= 0:
            f = &self.srcs[si].fin
            self.fin.add_reason(f, R_PROP_BRAKE)
            f.local_valid = False;f.row_valid = False
        return self.st[q].thrust

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef int brake_reseed(self, int sid, int seed, int fixed_kind, double value, int ev_kind, int ev_axis, int ev_side,
                          int max_nfev, int max_evaluations, double event_tolerance, double speed, double floor) except -2:
        """S10: a correction that closed on a braking propeller state (its governed propellers near the fine pitch stop,
        their blade sections at negative incidence) is tried again with the same equations from coarser governor pitch,
        where the pulling state of the same shaft speed lies (ER-2 ACh-30B SB 515 km/h: 24.6 against 33.4 deg): first from
        the correction's own seed where that one pulls (its propulsion coordinates, every governor coordinate moved
        half its distance from the braking state's further away from it: the braking state's induced flows are no seed
        for the pulling one), then from the braking state with every governor coordinate moved halfway, then a quarter
        of the way towards the coarse stop (on the fine stop: to mid travel; S11: a quarter of the travel, and a fourth
        attempt three quarters of the way, or of the travel). The first such correction that closes on a
        state that does not brake, or -1 (the braking state is the only one found: pulling flight ends there; S11: the
        braking state stays the chart's state there). S11 (Leo 2026-10-21: of two states the one with more net thrust):
        the corrections run with at least the robust budget (S11_NFEV, S11_EVALS; the other state's correction from mid
        travel needs it), a seed the same as the previous attempt's is not tried again, the correction's own seed is used where it pulls or brakes less than the braking
        state, and a state that brakes too is taken where its net thrust (weights) exceeds ``floor`` (the braking state's
        plus S11_THRUST_EPS: another state; for a shadowed SEP knot the midpoint to the state found at its load): the first
        pulling state, else the one with the most thrust above ``floor``, else -1. S11: no attempt after one that closes
        back on the braking state itself (its own basin reaches that seed)."""
        cdef int j, cid, q, attempt, src, best = -1, nfev = max_nfev, evals = max_evaluations
        cdef double g, gs, top = floor
        cdef double sol[5]
        cdef double prev[NMAX]
        cdef bint moved, has_prev = False, same
        if not self.st[sid].has_coords or not self.st[sid].layout_names:return -1
        if self.s11:
            nfev = max(max_nfev, S11_NFEV)
            if max_evaluations > 0:evals = max(max_evaluations, S11_EVALS)
        self.stats['s10_brake_states'] = self.stats.get('s10_brake_states', 0)+1
        if self.s11:
            key = 's11_brake_kind_%d_%s' % (fixed_kind, 'seedbrakes' if seed >= 0 and self.brakes(seed) else 'seedpulls' if seed >= 0 else 'noseed')
            self.stats[key] = self.stats.get(key, 0)+1
        for attempt in range(4 if self.s11 else 3):
            if attempt == 0:
                # the seed: a pulling state with the same coordinate layout and request branch
                if (seed < 0 or not self.st[seed].has_coords or not self.st[seed].layout_names or self.st[seed].nx != self.st[sid].nx
                        or self.st[seed].nreq != self.st[sid].nreq):continue
                if self.brakes(seed) and not (self.s11 and self.st[seed].thrust > floor):continue
                src = seed
            else:src = sid
            cid = self.new_state()
            self.st[cid] = self.st[sid]
            self.st[cid].src = -1;self.st[cid].row = -1;self.st[cid].base = cid;self.st[cid].limit = -1
            self.st[cid].valid = False
            moved = False
            for j in range(7, self.st[cid].nx):self.st[cid].coords[j] = self.st[src].coords[j]
            for j in range(self.st[cid].nx):
                if j >= self.layout_nx or not self.layout_names[j].startswith('governor:'):continue
                g = self.st[sid].coords[j]
                if attempt == 0:
                    gs = self.st[seed].coords[j]
                    self.st[cid].coords[j] = pmin(1., pmax(0., gs+.5*(gs-g))) if 0. <= g < 1. else gs
                elif attempt == 3:
                    # S11: three quarters of the way towards the coarse stop (the third state of P-47D or Tu-2 near top
                    # speed: several governed states, the one with the most thrust at the coarsest pitch)
                    self.st[cid].coords[j] = .75 if g < 0. else (g+.75*(1.-g) if g < 1. else g)
                elif g < 0.:self.st[cid].coords[j] = .25 if self.s11 and attempt == 2 else .5
                elif g < 1.:self.st[cid].coords[j] = .5*(g+1.) if attempt == 1 else g+.25*(1.-g)
                moved = moved or self.st[cid].coords[j] != g
            if not moved:continue
            if self.s11:
                same = has_prev
                for j in range(self.st[cid].nx):
                    if has_prev and prev[j] != self.st[cid].coords[j]:same = False
                    prev[j] = self.st[cid].coords[j]
                has_prev = True
                if same:continue
            for j in range(5):sol[j] = self.st[sid].sol[j]
            self.brake_retry = True
            try:
                q = self.correct(cid, sol, self.st[sid].load, True, fixed_kind, value, ev_kind, ev_axis, ev_side, nfev,
                                 evals, event_tolerance, speed, NULL, NULL)
            finally:
                self.brake_retry = False
            if self.s11:
                key = 's11_reseed_%d_%s' % (attempt, 'failed' if q < 0 else 'pulling' if not self.brakes(q) else
                                            'more_thrust' if self.st[q].thrust > floor else 'braking')
                self.stats[key] = self.stats.get(key, 0)+1
                self.stats[key+'_evals'] = self.stats.get(key+'_evals', 0)+(self.st[q].evaluations if q >= 0 else self.P.aircraft_evaluations)
            if q >= 0 and not self.brakes(q):
                self.stats['s10_brake_reseeded'] = self.stats.get('s10_brake_reseeded', 0)+1
                return q
            if self.s11 and q >= 0 and self.st[q].thrust > top:
                best = q;top = self.st[q].thrust
            elif self.s11 and q >= 0 and attempt > 0 and fabs(self.st[q].thrust-self.st[sid].thrust) <= S11_THRUST_EPS:
                # S11: back on the braking state itself from coarser pitch: its own basin reaches there, no further seeds
                break
        if best >= 0:
            self.stats['s11_brake_more_thrust'] = self.stats.get('s11_brake_more_thrust', 0)+1
        return best

    # ------------------------------------------------------------------ S7: exact slopes
    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef bint tangent(self, int sid, int mode, int ev_code, int ev_axis, int ev_side, double speed, double delta,
                      int sides, double* left, double* right) except -1:
        """S7: dw/dV (w = n^2-1, V in km/h) along the solution set of a state's own equations through the state:
        mode 0 the SEP level set (rows: stationary residuals and SEP; unknowns: coordinates and w), mode 1 an event
        face (rows: stationary residuals and the event residual ``ev_code``/``ev_axis``/``ev_side``), mode 2 fixed
        alpha (rows: stationary residuals; unknowns without alpha: at a load maximum along alpha this is the slope
        of the maximum). A fresh forward-difference Jacobian at the state (4x the corrector's steps: the rows are binary32
        model values, and the corrector's own steps leave about 1% noise in a slope; stencils projected onto the state's
        Mach bin on rough Mach intervals, as the corrector's), then the state's own rows solved again at speed-delta
        (``left``) and speed+delta (``right``) by chord Newton steps with that Jacobian (NaN where unavailable): the
        slope is the secant of exact solutions of the same rows, so neither noise in the Jacobian (the Instructor's and
        the propulsion rows are not smooth at stencil scale) nor the state's closure residual enters it. Returns False
        when no slope is available (unevaluable state or stencil, a stencil that changes the polar branch, singular
        rows). ``sides``: 1 left, 2 right, 3 both; a single requested side that fails falls back to the other."""
        cdef CurveProblem P = self.P
        cdef TState* t = &self.st[sid]
        cdef int nx = P.nx, n, m, nr, i, j, k, slot0, slot, ci, side
        cdef double q0[NMAX]
        cdef double q[NMAX]
        cdef double y[NMAX]
        cdef double yp[NMAX]
        cdef double r0[NMAX]
        cdef double rv[NMAX]
        cdef double C[NMAX*NMAX]
        cdef double D[NMAX*NMAX]
        cdef double z[NMAX]
        cdef double du[NMAX]
        cdef double rs[NMAX]
        cdef double h, w0, wp, step, value
        cdef int br0[BMAX]
        cdef Sample base
        cdef bint ok = True, same, governor
        cdef double sgn, first, best, best_w
        cdef int oi
        cdef double[7] steps7 = [.002, .002, .0002, .0002, .0002, .002, .002]
        cdef double[7] rs7 = [2e-4, 2e-4, 5e-4, 5e-4, 5e-4, 2e-5, 2e-5]
        cdef double t0 = s5_now()
        left[0] = NAN;right[0] = NAN
        if not t.has_coords or not t.layout_names or t.nx != nx or not t.req_ok:return False
        # level flight (w = 0): bank and turn rate scale with sqrt(w) there, so no slope in these coordinates
        # (the interpolant's end rule takes over: fill_slopes)
        if not t.load*t.load-1. > 1e-6:return False
        self.stats['s7_tangents'] = self.stats.get('s7_tangents', 0)+1
        # the problem at the state (CurveCorrection.solve_core's layout, turn coordinate q[nx])
        P.slide = t.slide
        P.nreq = t.nreq
        for i in range(t.nreq):P.req[i] = t.req[i]
        P.nz = nx+1;P.speed_kmh = speed
        w0 = pmax(0., t.load*t.load-1.)
        for i in range(nx):q0[i] = t.coords[i]
        q0[nx] = sqrt(w0)
        for i in range(nx+1):P.z[i] = q0[i]
        if mode == 0:
            P.fixed = -1;P.target = t.ps if isfinite(t.ps) else 0.
        elif mode == 1:
            P.fixed = -3;P.ev_kind = ev_code;P.ev_axis = ev_axis;P.ev_side = ev_side
            if ev_code == 6:P.ev_limit = t.rpm_limit[ev_axis]*M_PI/30.
            elif ev_code == 7:P.ev_limit = self.ias_limit
        else:
            P.fixed = 0;P.alpha = q0[0]
        k = 0
        for i in range(nx+1):
            if mode == 2 and i == 0:continue
            P.free_idx[k] = i;k += 1
        n = k;P.n = n
        nr = 7+P.nprows;m = nr+(1 if mode != 2 else 0)
        if n != m:return False
        for i in range(7):rs[i] = rs7[i]
        for i in range(7, nr):rs[i] = 1e-5
        if mode == 0:rs[nr] = .05
        elif mode == 1:rs[nr] = 2e-6
        P.nmemo = 0;P.aircraft_evaluations = 0;P.last_slot = -1;P.nprep = 0;P.prep_next = 0
        P.evaluations = 0;P.derivative_samples = 0;P.nominal_comparisons = 0;P.nominal_restorations = 0;P.branch_secants = 0
        try:
            slot0 = P.value(q0)
            if slot0 < 0:
                self.stats['s10_tangent_fail_unevaluable'] = self.stats.get('s10_tangent_fail_unevaluable', 0)+1
                return False
            for i in range(nr):r0[i] = P.memo_r[slot0*NMAX+i]
            if m > nr:r0[nr] = P.memo_extra[slot0]
            for i in range(BMAX):br0[i] = P.memo_br[slot0*BMAX+i]
            # the coordinates must reproduce a closed state of these rows (the SEP and event rows by construction:
            # their level is the state's own)
            for i in range(nr):
                if not fabs(r0[i]) <= 20.*rs[i]:
                    self.stats['s10_tangent_fail_closure'] = self.stats.get('s10_tangent_fail_closure', 0)+1
                    return False
            base.slot = slot0
            for j in range(n):
                ci = P.free_idx[j]
                governor = False
                if ci < 7:h = steps7[ci]
                elif ci < nx:
                    governor = self.layout_names[ci].startswith('governor:')
                    h = 1e-4 if governor else 1e-4*pmax(1., fabs(q0[ci]))
                else:h = .0004*pmax(sqrt(w0), .25)
                h *= TANGENT_SCALE
                same = False
                # S10: where both stencils change the polar branch (a state between two branch points within the step: the
                # stall face's alpha), both again at an eighth of the step
                for side in range(4 if self.s10 else 2):
                    step = (h if side % 2 == 0 else -h)*(1. if side < 2 else .125)
                    if governor:
                        # one governor coordinate spans three regimes with kinks at 0 and 1: stay in the state's regime
                        if (q0[ci] < 0.) != (q0[ci]+step < 0.) or (q0[ci] > 1.) != (q0[ci]+step > 1.):continue
                    for i in range(nx+1):q[i] = q0[i]
                    if ci == nx:
                        wp = w0+step
                        if wp < 0.:continue
                        q[nx] = sqrt(wp)
                    else:
                        q[ci] = q0[ci]+step
                    for i in range(n):y[i] = q[P.free_idx[i]]
                    if P.model.conds[P.memo_eval[slot0].cond].rough:
                        P.project(y, &base, yp)
                        for i in range(n):q[P.free_idx[i]] = yp[i]
                    slot = P.value(q)
                    if slot < 0:continue
                    same = True
                    for i in range(BMAX):
                        if P.memo_br[slot*BMAX+i] != br0[i]:
                            same = False;break
                    if not same:continue
                    for i in range(nr):C[i*n+j] = (P.memo_r[slot*NMAX+i]-r0[i])/rs[i]
                    if m > nr:C[nr*n+j] = (P.memo_extra[slot]-r0[nr])/rs[nr]
                    for i in range(n):
                        ci = P.free_idx[i]
                        if ci == nx:D[i*n+j] = q[nx]*q[nx]-w0
                        else:D[i*n+j] = q[ci]-q0[ci]
                    break
                if not same:
                    # (S10 diagnostics: the coordinate whose stencil changes the polar branch on both sides)
                    key = 's10_tangent_fail_stencil_%d' % ci
                    self.stats[key] = self.stats.get(key, 0)+1
                    return False
            # each side: the state's own rows F = F(state) solved at speed +- delta by chord Newton steps with this
            # Jacobian from the state (the first step is the linear tangent); the slope is the secant of the two exact
            # solutions, so neither the Jacobian's noise nor the state's closure residual enters it
            for oi in range(2):
                side = (oi if sides != 2 else 1-oi)
                if sides != 3 and oi == 1 and (isfinite(left[0]) or isfinite(right[0])):break
                sgn = 1. if side == 1 else -1.
                P.speed_kmh = speed+sgn*delta
                P.nmemo = 0;P.nprep = 0;P.prep_next = 0
                for i in range(n):du[i] = 0.
                first = NAN;value = NAN;best = INFINITY;best_w = NAN
                for it in range(6):
                    for i in range(nx+1):q[i] = q0[i]
                    ok = True
                    for i in range(n):
                        ci = P.free_idx[i]
                        if ci == nx:
                            wp = w0+du[i]
                            if wp < 0.:
                                ok = False;break
                            q[nx] = sqrt(wp)
                        else:q[ci] = q0[ci]+du[i]
                    if not ok:break
                    slot = P.value(q)
                    if slot < 0:
                        self.stats['s7_chord_unevaluable'] = self.stats.get('s7_chord_unevaluable', 0)+1;break
                    same = True
                    # S8: the exact-Mach branch (native and continuous Mach curves) changes with the speed itself
                    for i in range(1 if self.model.exact_mach else 0, BMAX):
                        if P.memo_br[slot*BMAX+i] != br0[i]:
                            same = False;break
                    if not same:
                        self.stats['s7_chord_branch'] = self.stats.get('s7_chord_branch', 0)+1;break
                    step = 0.
                    for i in range(nr):
                        rv[i] = (P.memo_r[slot*NMAX+i]-r0[i])/rs[i];step = pmax(step, fabs(rv[i]))
                    if m > nr:
                        rv[nr] = (P.memo_extra[slot]-r0[nr])/rs[nr];step = pmax(step, fabs(rv[nr]))
                    if it > 0 and step < best:
                        best = step;best_w = du[n-1]
                    if (it > 0 and step <= .05) or (it >= 2 and step <= .5):break
                    if it == 5:
                        # the rows' own noise floor (Instructor and propulsion rows): the best iterate within half the
                        # corrector's closure tolerance stands
                        self.stats['s7_chord_budget'] = self.stats.get('s7_chord_budget', 0)+1
                        break
                    for i in range(m):rv[i] = -rv[i]
                    if svd_solve(m, n, C, rv, z):break
                    for i in range(n):
                        h = 0.
                        for k in range(n):h += D[i*n+k]*z[k]
                        du[i] += h
                    if it == 0:first = du[n-1]/(sgn*delta)
                if best <= .5:value = best_w/(sgn*delta)
                else:
                    value = first
                    self.stats['s7_tangent_linear'] = self.stats.get('s7_tangent_linear', 0)+1
                if not isfinite(value):continue
                if side == 1:right[0] = value
                else:left[0] = value
        finally:
            P.speed_kmh = speed
            self.stats['s7_tangent_evals'] = self.stats.get('s7_tangent_evals', 0)+P.aircraft_evaluations
            self.cc.charge_problem(P)
            self.t_tangent += s5_now()-t0
        if isfinite(left[0]) or isfinite(right[0]):
            self.stats['s7_tangents_ok'] = self.stats.get('s7_tangents_ok', 0)+1
            return True
        return False

    def debug_tangent(self, int sid, int mode, int code, int axis, int side, double speed, double delta):
        """Diagnostics: MarchEngine.tangent from Python ((left, right) dw/dV or None)."""
        cdef double l = NAN, r = NAN
        if not self.tangent(sid, mode, code, axis, side, speed, delta, 3, &l, &r):return None
        return l, r

    def debug_face(self, int sid, int code, int axis, int side, double speed):
        """Diagnostics: the event-face correction (FIX_EVENT) at ``speed`` from state ``sid``: (load, valid) or None."""
        cdef PredMap preds
        cdef int p
        preds.n = 0
        if code == 6:self.P.ev_limit = self.st[sid].rpm_limit[axis]*M_PI/30.
        p = self.correct(sid, self.st[sid].sol, self.st[sid].load, True, FIX_EVENT, 0., code, axis, side, 30, 0, 2e-6, speed,
                         &preds, NULL)
        if p < 0:return None
        return self.st[p].load, bool(self.st[p].valid), self.st[p].alpha

    def debug_level(self, int sid, double speed, int nfev=12, int evals=40):
        """Diagnostics: the level-flight correction (FIX_LOAD 1 g) at ``speed`` from state ``sid``: (state id, valid,
        physical, reasons) or None."""
        cdef int p = self.correct(sid, self.st[sid].sol, 1., True, FIX_LOAD, 1., 0, 0, 0, nfev, evals, 2e-6, speed, NULL, NULL)
        if p < 0:return None
        return p, bool(self.st[p].valid), bool(self.physical(p)), self.reason_list(p)

    def debug_jump(self, int sid):
        """Diagnostics (S8): the blade-polar jump check of a state (its sliding spec record, or None)."""
        cdef SlideSpec spec
        self.slide_candidate(sid, &spec)
        if spec.n == 0:return None
        for i in range(NP):
            if spec.flag[i]:
                spec.lo[i] = spec.hint[i];spec.hi[i] = spec.hint[i];spec.theta[i] = 0.
        return slide_record(&spec)

    def debug_slide_level(self, int sid, double speed):
        """Diagnostics (S8): level flight at ``speed`` from state ``sid`` sliding on its jump: (state, valid, reasons,
        sliding record) or None."""
        cdef SlideSpec spec
        cdef int p
        self.slide_candidate(sid, &spec)
        if spec.n == 0:return 'no jump'
        self.slide_override = spec;self.use_override = True
        try:
            p = self.correct(sid, self.st[sid].sol, 1., True, FIX_LOAD, 1., 0, 0, 0, 30, 120, 2e-6, speed, NULL, NULL)
        finally:
            self.use_override = False
        if p < 0:return None
        return p, bool(self.st[p].valid), self.reason_list(p), slide_record(&self.st[p].slide)

    def debug_slide_alpha(self, int sid, double alpha, int nfev=30, int evals=120, bint slide=True):
        """Diagnostics (S8): the prescribed-alpha correction at the state's speed from ``sid``, sliding on its jump
        (``slide``) or regular: (state or -1, valid, reasons, last correction, sliding record)."""
        cdef SlideSpec spec
        cdef int p
        cdef double sol[5]
        if slide:
            self.slide_candidate(sid, &spec)
            if spec.n == 0:return 'no jump'
        else:spec.n = 0
        for i in range(5):sol[i] = self.st[sid].sol[i]
        sol[0] = alpha
        self.slide_override = spec;self.use_override = True
        try:
            p = self.correct(sid, sol, self.st[sid].load, True, FIX_ALPHA, alpha, 0, 0, 0, nfev, evals, 2e-6, self.st[sid].speed,
                             NULL, NULL)
        finally:
            self.use_override = False
        lc = self.last_correction()
        if p < 0:return -1, None, None, lc, None
        return p, bool(self.st[p].valid), self.reason_list(p), lc, slide_record(&self.st[p].slide)

    def debug_torque_scan(self, int sid, int shaft, double alpha_shift, list deltas):
        """Diagnostics (S8): at the state's flight condition (alpha shifted by ``alpha_shift``, the rest held), the
        propulsion residual rows and the signature of shaft ``shaft`` with its coordinate moved by each delta."""
        cdef TState* t = &self.st[sid]
        cdef PropPrep* prep
        cdef PropSample ps
        cdef double c[NMAX]
        cdef double qq[MAXPATH]
        cdef double inner[MAXPATH]
        cdef int j, i, code
        cdef PropModel pm = self.fin.pm
        out = []
        self.P.speed_kmh = t.speed;self.P.nprep = 0;self.P.prep_next = 0
        prep = self.P.prepared(t.load, t.coords[0]+alpha_shift, t.coords[1])
        j = pm.shaft_coordinate(shaft)
        pm.use_slide(NULL)
        for d in deltas:
            for i in range(t.nx-7):c[i] = t.coords[7+i]
            c[j] += d
            code = pm.propulsion_q(prep, c, t.req, t.nreq, inner, &ps, qq)
            if code:
                out.append((d, None));continue
            out.append((d, ps.frame.trans_omega[shaft]*30./M_PI, [inner[i] for i in range(pm.nrows)], ps.torques[shaft],
                        pm.signature(shaft, prep, qq, ps.fractions[shaft], ps.frame.trans_omega[shaft])))
        return out

    def debug_torque_at(self, int sid, double speed, double load, double alpha, double bank, list shafts, list omegas,
                        list flows=None):
        """Diagnostics (S9): at the flight condition (speed, load, alpha, bank) with the state's other propulsion
        coordinates (the induced flows from ``flows`` where given: coordinate index -> value), every shaft in
        ``shafts`` at each shaft speed of ``omegas`` (rpm): (rpm, residual rows, shaft torques, signatures)."""
        cdef TState* t = &self.st[sid]
        cdef PropPrep* prep
        cdef PropSample ps
        cdef double c[NMAX]
        cdef double qq[MAXPATH]
        cdef double inner[MAXPATH]
        cdef int j, i, code, s
        cdef PropModel pm = self.fin.pm
        out = []
        self.P.speed_kmh = speed;self.P.nprep = 0;self.P.prep_next = 0
        prep = self.P.prepared(load, alpha, bank)
        pm.use_slide(NULL)
        for rpm in omegas:
            for i in range(t.nx-7):c[i] = t.coords[7+i]
            if flows:
                for k, v in flows.items():c[int(k)] = float(v)
            for s in shafts:
                j = pm.shaft_coordinate(s)
                # the state's coordinate offset (a governed shaft on a pitch stop: coordinate - shaft speed/300)
                c[j] = t.coords[7+j]-t.rpm[s]*M_PI/30./300.+rpm*M_PI/30./300.
            code = pm.propulsion_q(prep, c, t.req, t.nreq, inner, &ps, qq)
            if code:
                out.append((rpm, None));continue
            out.append((rpm, [inner[i] for i in range(pm.nrows)], [ps.torques[s] for s in shafts],
                        [pm.signature(s, prep, qq, ps.fractions[s], ps.frame.trans_omega[s]) for s in shafts],
                        [dict(torque=ps.frame.engines[k].torque, friction=ps.frame.engines[k].friction,
                              manifold=ps.frame.engines[k].manifold, potential=ps.frame.engines[k].potential_manifold,
                              ratio=ps.frame.engines[k].throttle_ratio, gear=ps.frame.engines[k].gear,
                              omega=ps.frame.engines[k].omega, mixture=ps.frame.engines[k].mixture,
                              regulator=ps.frame.engines[k].regulator, effective=ps.frame.engines[k].effective_throttle,
                              mechanical=ps.frame.engines[k].mechanical)
                         for k in range(pm.ne)],
                        [ps.frame.props[k].outputs[18] for k in range(pm.nprops)],
                        pm.debug_stations(shafts[0], prep, qq, ps.fractions[shafts[0]], ps.frame.trans_omega[shafts[0]])))
        return out

    def debug_rows(self, int sid, int index, list deltas):
        """Diagnostics (S9): the residual rows (scaled as the corrector's: force/moment rows, propulsion rows) of the
        state's own coordinates with coordinate ``index`` (nx: the turn coordinate sqrt(n^2-1)) moved by each delta."""
        cdef CurveProblem P = self.P
        cdef TState* t = &self.st[sid]
        cdef double q[NMAX]
        cdef int i, slot, nx = P.nx, nr = 7+P.nprows
        P.slide = t.slide
        P.nreq = t.nreq
        for i in range(t.nreq):P.req[i] = t.req[i]
        P.nz = nx+1;P.speed_kmh = t.speed;P.fixed = 0;P.alpha = t.coords[0]
        P.nmemo = 0;P.aircraft_evaluations = 0;P.last_slot = -1;P.nprep = 0;P.prep_next = 0
        out = []
        for d in deltas:
            for i in range(nx):q[i] = t.coords[i]
            q[nx] = sqrt(pmax(0., t.load*t.load-1.))
            q[index] += d
            P.nmemo = 0;P.nprep = 0;P.prep_next = 0
            slot = P.value(q)
            if slot < 0:
                out.append((d, None));continue
            out.append((d, [P.memo_r[slot*NMAX+i] for i in range(nr)], P.memo_eval[slot].force_error_g))
        return out

    def debug_record(self, int sid, int index, double delta):
        """Diagnostics (S9): the model's value record of the state with coordinate ``index`` moved by ``delta``."""
        cdef CurveProblem P = self.P
        cdef TState* t = &self.st[sid]
        cdef double q[NMAX]
        cdef int i, slot, nx = P.nx
        P.slide = t.slide
        P.nreq = t.nreq
        for i in range(t.nreq):P.req[i] = t.req[i]
        P.nz = nx+1;P.speed_kmh = t.speed;P.fixed = 0;P.alpha = t.coords[0]
        P.nmemo = 0;P.aircraft_evaluations = 0;P.last_slot = -1;P.nprep = 0;P.prep_next = 0
        for i in range(nx):q[i] = t.coords[i]
        q[nx] = sqrt(pmax(0., t.load*t.load-1.))
        q[index] += delta
        slot = P.value(q)
        if slot < 0:return None
        return value_record(self.model, &P.memo_eval[slot], True, True)

    def debug_slide_sample(self, int sid, double theta):
        """Diagnostics (S8): the sliding propulsion sample of the state (every jump shaft at ``theta``) against the regular
        samples just below and above the jump: (sliding row, inner, regular row below, regular row above)."""
        cdef TState* t = &self.st[sid]
        cdef SlideSpec spec
        cdef PropPrep* prep
        cdef PropSample ps
        cdef double c[NMAX]
        cdef double qq[MAXPATH]
        cdef double inner[MAXPATH]
        cdef int j, i, s, code
        cdef PropModel pm = self.fin.pm
        self.slide_candidate(sid, &spec)
        if spec.n == 0:return None
        self.P.speed_kmh = t.speed;self.P.nprep = 0;self.P.prep_next = 0
        prep = self.P.prepared(t.load, t.coords[0], t.coords[1])
        out = {}
        for i in range(t.nx-7):c[i] = t.coords[7+i]
        for s in range(NP):
            if spec.flag[s]:c[pm.shaft_coordinate(s)] = spec.c0[s]+theta
        pm.use_slide(&spec)
        code = pm.propulsion_q(prep, c, t.req, t.nreq, inner, &ps, qq)
        pm.slide_result(&spec)
        out['slide'] = None if code else ([ps.row[i] for i in range(11)], [inner[i] for i in range(pm.nrows)],
                                          [(s, spec.lo[s]*30/M_PI, spec.hi[s]*30/M_PI) for s in range(NP) if spec.flag[s]])
        pm.use_slide(NULL)
        for side in (0, 1):
            for i in range(t.nx-7):c[i] = t.coords[7+i]
            for s in range(NP):
                if spec.flag[s]:c[pm.shaft_coordinate(s)] = spec.cw[s]+(spec.lo[s] if side == 0 else spec.hi[s])/300.
            code = pm.propulsion_q(prep, c, t.req, t.nreq, inner, &ps, qq)
            out['below' if side == 0 else 'above'] = None if code else ([ps.row[i] for i in range(11)], [inner[i] for i in range(pm.nrows)])
        return out

    def debug_state_slide(self, int sid):
        return slide_record(&self.st[sid].slide)

    def debug_events(self, int sid):
        cdef Event ev[32]
        cdef int n = self.nearest_events(sid, ev), i
        return [(ev[i].kind, ev[i].axis, ev[i].side, ev[i].engine, ev[i].code, ev[i].distance) for i in range(n)]

    def debug_state(self, int sid):
        cdef TState* t = &self.st[sid]
        return dict(speed=t.speed, load=t.load, alpha=t.alpha, base=t.base, limit=t.limit, valid=bool(t.valid),
                    margins={KIND_TEXT[k]: t.m[k] for k in range(NKD) if (t.mset >> k) & 1})

    def debug_info(self, int sid):
        """Diagnostics (S9): the fields of a state."""
        cdef TState* t = &self.st[sid]
        cdef int i
        return dict(sid=sid, speed=t.speed, load=t.load, alpha=t.alpha, bank=t.bank, turn=t.turn, ps=t.ps, mach=t.mach,
                    valid=bool(t.valid), converged=bool(t.converged), reasons=self.reason_list(sid), rule=t.rule,
                    force_error_g=t.force_error_g, angular_error=t.angular_error, history_error=t.history_error,
                    sol=[t.sol[i] for i in range(5)], coords=[t.coords[i] for i in range(t.nx)], base=t.base, limit=t.limit,
                    slide=t.slide.n, physical=bool(self.physical(sid)), hard=bool(self.hard_rejected(sid)),
                    margins={KIND_TEXT[k]: t.m[k] for k in range(NKD) if (t.mset >> k) & 1},
                    rpm=[t.rpm[i] for i in range(t.ne)], rpm_limit=[t.rpm_limit[i] for i in range(t.ne)])

    def debug_correct(self, int seed, double speed, kind, double value, load=None, alpha=None, int nfev=30, int evals=120,
                      double tol=2e-6, slide=False):
        """Diagnostics (S9): one correction from state ``seed`` at ``speed``: kind 'alpha' / 'load' / 'sep' / 'elevator'
        with its value (the seed's solution with ``alpha`` and ``load`` replaced where given). (state or -1, last
        correction residuals, evidence state or -1)."""
        cdef double sol[5]
        cdef int i, p, ev = -1, fk
        cdef double n
        cdef SlideSpec spec
        for i in range(5):sol[i] = self.st[seed].sol[i]
        if alpha is not None:sol[0] = float(alpha)
        n = self.st[seed].load if load is None else float(load)
        fk = dict(alpha=FIX_ALPHA, load=FIX_LOAD, sep=FIX_SEP, elevator=FIX_ELEVATOR)[kind]
        if fk == FIX_ALPHA:sol[0] = value
        if slide == 'regular':
            spec.n = 0
            self.slide_override = spec;self.use_override = True
        elif slide:
            self.slide_candidate(seed, &spec, sol[0], sol[1], n, True, speed)
            if spec.n == 0:return 'no jump'
            self.slide_override = spec;self.use_override = True
        try:
            p = self.correct(seed, sol, n, True, fk, value, 0, 0, 0, nfev, evals, tol, speed, NULL, &ev)
        finally:
            self.use_override = False
        return p, self.last_correction(), ev, self.last_load, self.last_closed

    def debug_seed(self, int sid, dict coords, sol=None, load=None, speed=None, brakes=None):
        """Diagnostics (S11): a seed copy of a state with some coordinates replaced ({index: value}), and optionally its
        solution, load, speed and braking flag: its state id."""
        cdef int cid = self.new_state(), i
        self.st[cid] = self.st[sid]
        self.st[cid].src = -1;self.st[cid].row = -1;self.st[cid].base = cid;self.st[cid].limit = -1;self.st[cid].valid = False
        for j, v in coords.items():self.st[cid].coords[<int>j] = float(v)
        if sol is not None:
            for i in range(5):self.st[cid].sol[i] = float(sol[i])
        if load is not None:self.st[cid].load = float(load)
        if speed is not None:self.st[cid].speed = float(speed)
        if brakes is not None:self.st[cid].brakes = bool(brakes)
        return cid

    def debug_raw(self, bint raw):
        """Diagnostics (S11): corrections without the braking reseed (raw) or with it."""
        self.brake_retry = raw

    def debug_brakes(self, int sid):
        """Diagnostics (S11): whether a state's propellers brake, and its net propeller thrust in weights."""
        cdef double v = NAN
        if self.st[sid].src >= 0 and self.fin.pm is not None:
            v = net_thrust(self.fin.pm, &self.srcs[self.st[sid].src].ps.frame)/self.model.weight
        return bool(self.brakes(sid)), v

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef dict last_correction(self):
        return self.cc.last_correction()

    # ------------------------------------------------------------------ the cross-speed store
    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef void store_keep(self, int sid, double speed, PredMap* preds) noexcept:
        """March.keep: the corrected admitted state with the predictor under the store's alpha key."""
        cdef Predictor* p
        cdef int i
        if not self.has_alpha_key:
            for i in range(preds.n):
                if preds.key[i].fixed == 0:
                    self.alpha_key = preds.key[i];self.has_alpha_key = True;break
            if not self.has_alpha_key:return
        p = pm_get(preds, &self.alpha_key)
        if p == NULL or sid < 0 or not self.passable(sid):return
        if self.nstore >= 601:return
        self.store[self.nstore].speed = speed;self.store[self.nstore].alpha = self.st[sid].alpha
        self.store[self.nstore].state = sid;self.store[self.nstore].has_pred = True
        self.store[self.nstore].pred = p[0]
        self.nstore += 1
        if self.nstore > 600:
            for i in range(self.nstore-200):self.store[i] = self.store[i+200]
            self.nstore -= 200

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef bint store_used(self) noexcept:
        # (self.store or {}) truthiness: the store dict is non-empty
        return self.has_alpha_key or self.has_level_key or self.nstore > 0 or self.nlevel > 0

    @cython.boundscheck(True)
    @cython.wraparound(True)
    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef object initial_dict(self, int anchor, list attitude):
        """em_sampling.continuation_initial(solver, anchor, attitude) from the state's fields."""
        cdef TState* t = &self.st[anchor]
        cdef int i
        if attitude is None:attitude = [t.sol[i] for i in range(5)]
        names = self.names_of(anchor)
        if not t.has_coords or not names or len(names) != t.nx:return attitude
        predicted = dict(zip(('alpha_deg', 'bank_deg', 'aileron', 'elevator', 'rudder'), attitude))
        return dict(solution=attitude, stationary_coordinate_names=list(names),
                    stationary_coordinates=[predicted.get(name, t.coords[i]) for i, name in enumerate(names)],
                    stationary_request_indices=[t.req[i] for i in range(t.nreq)])

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef PointSolver points_(self):
        if self.point_solver is None:self.point_solver = PointSolver(self)
        return <PointSolver>self.point_solver

    def fin_timers(self):
        return dict(solve_rows_s=self.t_srows, record_s=self.fin.t_record, admission_s=self.fin.t_admit, report_s=self.fin.t_report,
                    clean_s=self.fin.t_clean)

    def get_point_solver(self):
        return self.points_()

    def python_counts(self):
        """Where this engine left native code: Python call-outs by kind (zero everywhere is fully native)."""
        out = dict(production_columns=self.stats.get('python_calls', 0), python_events=self.stats.get('python_events', 0),
                   python_instructor=len(self.fin.python_instructors) if self.fin is not None else 0,
                   python_admission=bool(self.fin.python_admission) if self.fin is not None else False)
        if self.point_solver is not None:out['prop_branch_recoveries'] = (<PointSolver>self.point_solver).stats['python_recoveries']
        return out

    def flush_counts(self):
        """Book the native work counters to the current Python work record (and its parents)."""
        cdef int k
        for k in range(NCOUNT):
            if self.counts[k]:
                bulk_charge(COUNT_NAMES[k], self.counts[k])
                self.counts[k] = 0

    @cython.boundscheck(True)
    @cython.wraparound(True)
    def state_count(self):
        return self.nst

    @cython.boundscheck(True)
    @cython.wraparound(True)
    def state_row(self, int sid):
        return self.row_of(sid)
