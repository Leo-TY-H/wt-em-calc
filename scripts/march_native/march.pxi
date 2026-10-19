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
DEF NKD = 8
# Kind names in the lexical order of the Python strings (sorted() of (root, kind) ties)
KIND_TEXT = ('Instructor pitch', 'control', 'engine RPM', 'pitch response', 'propeller pitch', 'stall', 'trim fold',
             'wing force')
KIND_CODE = {k: i for i, k in enumerate(KIND_TEXT)}

DEF G_TURN = 9.8100004196167
DEF MAX_STEP_DEG = 8.
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
    return -1


cdef inline double turn_rate(double speed, double load) noexcept nogil:
    # em_march.turn_rate
    return G_TURN*sqrt(pmax(0., load*load-1.))/(speed/3.6)*RAD_TO_DEG


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
    cdef public double t_solve, t_finish, t_rows, t_point, t_python, t_ledger
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
        self.t_solve = self.t_finish = self.t_rows = self.t_point = self.t_python = self.t_ledger = 0.
        self.engine_shafts = []
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
        t.src = si;t.row = -1;t.base = sid;t.limit = -1
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
        elif t.src >= 0:
            row = self.fin.row(&self.srcs[t.src])
            self.stats['rows_built'] += 1
        else:
            raise RuntimeError('state without a row source')
        if t.limit >= 0:row['envelope_limit'] = self.limits[t.limit]
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
        return out

    cdef bint physical(self, int sid) noexcept:
        # em_march.physical: converged, reasons nonempty and all physical
        cdef TState* t = &self.st[sid]
        cdef int i
        if not t.converged or not t.nreasons:return False
        for i in range(t.nreasons):
            if not physical_code(t.reasons[i]):return False
        return True

    @cython.boundscheck(True)
    @cython.wraparound(True)
    cdef bint combined(self, int sid, double* out) noexcept:
        cdef TState* t = &self.st[sid]
        cdef int k
        cdef bint found = False
        for k in range(NKD):
            if t.mset & (1 << k):
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
            if k >= 0:crossed |= 1 << k
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
        cdef int i, slot, code, si, sid
        cdef double n
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
        for i in range(t.nreq):sd.req[i] = t.req[i]
        if fixed_kind == FIX_EVENT and ev_kind == 6:
            # engine RPM: the event point's limit, rpm_limit*pi/30 (Curve.events)
            self.P.ev_limit = t.rpm_limit[ev_axis]*M_PI/30.
        if fixed_kind != FIX_EVENT and fixed_kind != FIX_SEP and preds != NULL:
            key.fixed = self.cc.fixed_index(fixed_kind);key.nreq = t.nreq
            for i in range(t.nreq):key.req[i] = t.req[i]
            pred_in = pm_get(preds, &key)
        self.stats['native_trials'] += 1
        cdef double cap = self.cap(speed)
        cdef double t0 = s5_now(), t1
        code = self.cc.solve_core(speed, cap, &sd, fixed_kind, value, ev_kind, ev_axis, ev_side, max_nfev,
                                  max_evaluations, event_tolerance, pred_in, &pred_out, &slot, &n, &closed)
        t1 = s5_now();self.t_solve += t1-t0
        if code < 0:return -1
        if pred_out.valid and preds != NULL and fixed_kind != FIX_EVENT and fixed_kind != FIX_SEP:
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
        if not f.row_valid:
            # Rejected trials never become rows: release the stored source.
            if si == self.nsrc-1:
                self.nsrc -= 1;self.st[sid].src = -1
        return sid

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
        if p == NULL or sid < 0 or not self.st[sid].valid:return
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
