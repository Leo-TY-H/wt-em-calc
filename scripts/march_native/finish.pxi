# Native finish of a stationary correction: JointProblem.finish -> TrimSolver.certify
# -> BaselineSolver.sample (chart_point, candidate summaries, clean). Direct-thrust
# and supported propeller aircraft (native propulsion record, governed requests,
# phase-fixed pitch response; the source-value propeller admission rows are the
# baseline prop_admission on the native record). The admission rules are
# transliterated line by line; verify mode compares every row with the Python
# finish.
#
# S5: the finish is split in two. ``compute`` makes every admission decision
# (all numerical work, every work charge) into a RowSrc without building Python
# records; ``row``/``point`` build the chart row from a RowSrc afterwards, only
# for states that are kept. ``finish`` (both together) is the S2-S4 entry.

DEF NAN = float('nan')
DEF MAXREASONS = 16

# Reason codes (JointProblem.finish / certify / sample reasons, in their text)
DEF R_BANK = 1
DEF R_TRIM = 2
DEF R_PROP_SETTLE = 3
DEF R_RPM = 4
DEF R_SHAFT = 5
DEF R_CYCLE = 6
DEF R_CONTROL = 7
DEF R_STALL = 8
DEF R_VSTEP = 9
DEF R_WING = 10
DEF R_IAS = 11
DEF R_MACH = 12
DEF R_SWEEP = 13
DEF R_FLAP_AUTO = 14
DEF R_FLAP_IAS = 15
DEF R_PITCH_REV = 16
DEF R_PITCH_UNRES = 17
DEF R_INSTR_UNRES = 18
DEF R_INSTR = 19
DEF R_SIDESLIP = 20
DEF R_PROP_ADM = 21
DEF R_LOAD_CAP = 22
DEF R_PROP_BRAKE = 23   # S10: the propellers' net thrust is negative under power (Leo, 2026-10-20)
DEF R_FIXED = 24    # first dynamically interned reason text

REASON_TEXT = [None, 'bank outside [-90, 90] degrees', 'trim did not converge', 'propulsion did not settle',
               'engine RPM limit', 'propeller shaft stopped', 'propulsion cycle unresolved', 'control authority',
               'post-stall', 'vertical step did not close', 'wing force limit', 'IAS limit', 'Mach limit',
               'sweep unavailable', 'flap automatic retraction', 'flap IAS limit', 'reversed pitch response',
               'pitch response unresolved', 'Instructor boundary unresolved', 'Instructor pitch limit', 'sideslip limit',
               'propeller admission', 'chart load search limit', 'propeller braking']
REASON_CODE = {text: i for i, text in enumerate(REASON_TEXT) if text is not None}


cdef int reason_code(text) except -1:
    code = REASON_CODE.get(text)
    if code is None:
        code = len(REASON_TEXT);REASON_TEXT.append(text);REASON_CODE[text] = code
    return code


cdef inline bint physical_code(int r) noexcept nogil:
    # em_pitch_response.PHYSICAL_REASONS; S10: propeller braking (a state's own physical rejection under the chart rule)
    return (r == R_STALL or r == R_CONTROL or r == R_WING or r == R_INSTR or r == R_RPM or r == R_PITCH_REV
            or r == R_PROP_BRAKE)


cdef inline bint soft_code(int r) noexcept nogil:
    # S8: the soft chart limits (em_solver.SOFT_REASONS): engine RPM (an over-rev passed on the way to a hard limit is
    # transient) and the forward-IAS limits (wing VNE, automatic flap retraction, flap damage: judged on each state's
    # own forward IAS, so the chart's high-speed edge leans with alpha)
    return r == R_RPM or r == R_IAS or r == R_FLAP_AUTO or r == R_FLAP_IAS


BRIEF_FIELDS = ('speed_kmh', 'load_g', 'solution', 'stationary_coordinates', 'stationary_coordinate_names',
                'stationary_request_indices', 'sideslip_attitude_deg', 'source_branch', 'local_valid', 'converged',
                'reasons', 'ps_mps', 'turn_dps', 'force_error_g', 'angular_error_rad_s2', 'history_error',
                'equilibrium_coverage_complete')
NAMES7 = ['alpha_deg', 'bank_deg', 'aileron', 'elevator', 'rudder', 'left_wing_history_deg', 'right_wing_history_deg']
SCALES7 = [.01, .01, .0001, .0001, .0001, .01, .01]
RSCALES7 = [2e-4, 2e-4, 5e-4, 5e-4, 5e-4, 2e-5, 2e-5]
LOWER7 = [-180., -90., -1., -1., -1., -1000., -1000.]
UPPER7 = [180., 90., 1., 1., 1., 1000., 1000.]


cdef inline object fin(double v):
    # em_stationary_chart.clean: nonfinite floats are exported as None
    return v if isfinite(v) else None


cdef struct FrozenState:
    bint valid
    double speed, load
    double attitude[5]
    double angles[2]
    bint has_pred
    Predictor pred


cdef struct PitchRes:
    bint unresolved, normal, reversed_
    int nsamples
    double step[2]
    double gain[2]
    double margin[2]
    double rounding[2]
    double failed_step


cdef struct Fin:
    # The admission of one corrected state: every decision and every number the
    # chart row takes from the finish (the rest comes from the Eval / propulsion sample).
    bint finite, history_finite, converged, local_valid, row_valid
    int nreasons
    int reasons[MAXREASONS]
    int nstructural
    int structural[3]
    bint has_pitch
    PitchRes pitch
    int instr          # 0 none, 1 native InstrRes, 2 Python dict (Finisher.python_instructors[instr_index])
    int instr_index
    InstrRes ires
    double authority
    double bounds[3][2]
    bint prop_converged
    int frames
    bint has_sweep_range
    double sweep_range[2]
    long calls
    bint brakes        # S11: the propellers' net thrust is negative under power (under the braking rule)


cdef struct RowSrc:
    # Everything ``Finisher.row`` needs to rebuild the chart row of one corrected state.
    Eval e
    int br[BMAX]
    double q[NMAX]
    double resid[NMAX]
    int nx, nres
    int req[NP]
    int nreq
    double speed_kmh, load
    PropPrep prep
    PropSample ps
    double qq[MAXPATH]
    SlideSpec slide      # S8: the shafts sliding on a blade-polar jump, with theta and shaft speeds
    Fin fin


cdef class Finisher:
    cdef Model model
    cdef FrozenProblem fp
    cdef public object solver
    cdef public dict constants
    cdef public long frozen_evaluations, frozen_derivative_samples
    cdef double length, inertia_z
    cdef bint invert_elevator, roll_leveling, structural, has_sweep, flaps_available, instructor_enabled
    # S8: the chart admission rule (solver.soft_admission, set by the march chart): a state whose only rejections are
    # soft limits (engine RPM, forward-IAS limits) still gets the hard checks (pitch response, Instructor), so the
    # march can tell a transient soft rejection from the first hard limit along alpha
    cdef public bint soft
    # S10 (WT_MARCH_S10=0 restores S9): under the chart rule a state whose propellers' net thrust is negative while the
    # engines deliver power (a braking propeller) is rejected (Leo, 2026-10-20: off the chart everywhere)
    cdef public bint brake
    # S11 (WT_MARCH_S11=0 restores S10): braking is a choice between states, not a rejection: a braking state is admitted
    # (flagged, Fin.brakes) and the march charts a pulling state of the same speed and load instead where one exists
    cdef public bint brake_choice
    cdef public bint is_prop, python_admission
    cdef object admission_revision
    cdef PropModel pm
    cdef public object layout
    cdef public object instructor        # native Instructor (lean admission) or None: Python solver.instructor_at
    cdef public dict loop_stats
    cdef public list python_instructors
    cdef public double t_record, t_admit, t_report, t_dict, t_clean
    cdef double flaps_percent
    cdef FrozenState* chain     # the frozen-history predictor chain of a point solve (its candidates share it), or NULL
    cdef dict request_cache

    def __init__(self, Model model, solver):
        from em_model_experiments import description
        from em_stationary import REVISION, STABLE_POLAR_REVISION
        from em_accuracy_policy import sep_error_budget
        self.model = model;self.solver = solver
        self.fp = FrozenProblem();self.fp.model = model;self.fp.n = 2;self.fp.m = 2
        for i in range(2):
            self.fp.lower[i] = -1000.;self.fp.upper[i] = 1000.;self.fp.steps[i] = .002
            self.fp.scale[i] = 10.;self.fp.step_limit[i] = 8.;self.fp.rscale[i] = 2e-5
        self.fp.has_project = False
        cfg = solver.config
        self.length = solver.fm['Length'];self.inertia_z = solver.mass['inertia'][2]
        self.invert_elevator = bool(solver.controls['invert_elevator'])
        self.roll_leveling = bool(solver.fm.get('RollLeveling', True))
        self.structural = bool(cfg['structural_limits']);self.has_sweep = bool(solver.has_sweep)
        self.instructor_enabled = bool(cfg['instructor'])
        self.flaps_available = bool(solver.flap_profile['available']) and cfg['flaps_percent'] > 0
        self.flaps_percent = cfg['flaps_percent']
        self.constants = dict(description=description(cfg), revision=REVISION, stable_revision=STABLE_POLAR_REVISION,
                              budget=float(sep_error_budget(cfg['sep_tolerance_mps'])), trim_mode=cfg['aircraft_trim_mode'],
                              governor='pitch_stops' if cfg.get('stationary_governor_mode', 'pitch_stops') == 'pitch_stops' else 'exact_rpm',
                              mach_mode=cfg.get('mach_curve_mode', 'native'), sweep_percent=cfg['sweep_percent'],
                              flaps_percent=cfg['flaps_percent'], instructor=cfg['instructor'],
                              gear_percent=solver.gear*100.)
        self.frozen_evaluations = self.frozen_derivative_samples = 0
        self.is_prop = bool(solver.is_prop);self.pm = None;self.layout = None
        import os
        self.python_admission = os.environ.get('S2_PYTHON_ADMISSION') == '1'
        from prop_admission import REVISION as admission_revision
        self.admission_revision = admission_revision
        self.instructor = None;self.loop_stats = None;self.python_instructors = []
        self.chain = NULL
        self.soft = bool(getattr(solver, "soft_admission", False))
        from em_stationary import braking_rule, braking_choice
        self.brake = self.is_prop and braking_rule(solver)
        self.brake_choice = self.brake and braking_choice(solver)
        self.request_cache = {}

    def set_prop(self, PropModel pm, layout):
        self.pm = pm;self.layout = dict(layout)
        if not self.pm.governor_saturation:raise PropUnsupported('exact-RPM governor finish')

    cdef int frozen(self, FrozenState* st, double speed, double load, const double* x, const double* history_initial,
                    double* moment_z, bint* closed) except -1:
        """stationary_frozen_point at (speed, load, x) for the pitch response; returns moment and closure."""
        cdef FrozenProblem P = self.fp
        cdef Result res
        cdef Predictor pred
        cdef double y[2]
        cdef double v
        cdef bint nearby = False
        cdef int i
        cdef double[5] lim = [8., 20., .4, .4, .4]
        cdef Eval* e
        if st.valid and st.speed == speed and st.load == load:
            v = 0.
            for i in range(5):
                v = pmax(v, fabs(x[i]-st.attitude[i])/lim[i])
            nearby = v <= 1.
        if nearby:
            y[0] = st.angles[0];y[1] = st.angles[1]
        else:
            y[0] = history_initial[0];y[1] = history_initial[1]
        P.speed = speed;P.load = load
        for i in range(5):P.x5[i] = x[i]
        P.nmemo = 0;P.aircraft_evaluations = 0
        P.evaluations = 0;P.derivative_samples = 0;P.nominal_comparisons = 0;P.nominal_restorations = 0;P.branch_secants = 0
        pred.valid = False
        if nearby and st.has_pred:
            pred = st.pred
        P.solve(y, 120, 0, &pred, False, &res)
        self.frozen_evaluations += P.aircraft_evaluations;self.frozen_derivative_samples += P.derivative_samples
        if P.aircraft_evaluations:charge(K_EVAL, P.aircraft_evaluations)
        if P.derivative_samples:charge(K_DERIV, P.derivative_samples)
        if P.nominal_comparisons:charge(K_NOMCMP, P.nominal_comparisons)
        if P.nominal_restorations:charge(K_NOMRES, P.nominal_restorations)
        if P.branch_secants:charge(K_SECANT, P.branch_secants)
        if res.termination == -1:
            raise RuntimeError('native frozen history: initial state unevaluable')
        e = &P.memo_eval[res.value.slot]
        st.valid = True;st.speed = speed;st.load = load
        for i in range(5):st.attitude[i] = x[i]
        st.angles[0] = res.x[0];st.angles[1] = res.x[1]
        st.has_pred = res.predictor.valid
        if res.predictor.valid:st.pred = res.predictor
        moment_z[0] = e.frame.moment[2]
        closed[0] = (isfinite(e.history_error) and 0. <= e.history_error and e.history_error <= 2e-4
                     and isfinite(e.angle_residual[0]) and isfinite(e.angle_residual[1])
                     and pmax(fabs(e.angle_residual[0]), fabs(e.angle_residual[1])) <= 2e-4)
        return 0

    cdef int pitch_core(self, Eval* e, bint jet, PitchRes* pr) except -1:
        """The native steady pitch response at the state (phase-fixed propulsion already set on the model)."""
        cdef FrozenState local
        cdef int i
        if self.chain != NULL and jet:
            return self.pitch_steps(e, self.chain, pr)
        previous = self.solver.__dict__.get('_stationary_frozen_predictor')
        local.valid = False;local.has_pred = False
        # The propulsion record of a native finish is a fresh object, so only a
        # direct-thrust predictor (propulsion None) can match ``previous`` (as S2-S4).
        if previous is not None and previous['key'][2] == 0. and jet and previous['propulsion'] is None:
            local.valid = True;local.speed = previous['key'][0];local.load = previous['key'][1]
            for i in range(5):local.attitude[i] = float(previous['attitude'][i])
            local.angles[0] = float(previous['angles'][0]);local.angles[1] = float(previous['angles'][1])
            if isinstance(previous['predictor'], NativePredictor):
                local.has_pred = True;local.pred = (<NativePredictor>previous['predictor']).p
        return self.pitch_steps(e, &local, pr)

    cdef int pitch_steps(self, Eval* e, FrozenState* st, PitchRes* pr) except -1:
        cdef double x[5]
        cdef double lo[5]
        cdef double hi[5]
        cdef double hist[2]
        cdef double sign, scale, step, width, ma, mb, slope, rounding
        cdef bint ca, cb
        cdef int i, k
        cdef double[2] steps_ = [.002, .0005]
        for i in range(5):x[i] = e.x[i]
        hist[0] = e.cycle_input.aoa[0];hist[1] = e.cycle_input.aoa[1]
        sign = 1. if self.invert_elevator else -1.
        scale = self.model.weight*self.length
        pr.nsamples = 0;pr.unresolved = False
        for k in range(2):
            step = steps_[k]
            for i in range(5):
                lo[i] = x[i];hi[i] = x[i]
            lo[3] = pmax(-1., x[3]-step);hi[3] = pmin(1., x[3]+step)
            self.frozen(st, e.speed, e.load, lo, hist, &ma, &ca)
            self.frozen(st, e.speed, e.load, hi, hist, &mb, &cb)
            if not ca or not cb:
                pr.unresolved = True;pr.normal = False;pr.reversed_ = False;pr.failed_step = step
                return 0
            width = hi[3]-lo[3]
            slope = sign*(mb-ma)/width
            rounding = 8.*(fabs(spacing32(ma))+fabs(spacing32(mb)))/width
            i = pr.nsamples
            pr.step[i] = step;pr.gain[i] = slope/self.inertia_z;pr.margin[i] = slope/scale;pr.rounding[i] = rounding/scale
            pr.nsamples += 1
            if fabs(slope) > 32.*rounding and fabs(slope/scale) > .002:
                break
        pr.normal = True;pr.reversed_ = True
        for i in range(pr.nsamples):
            if not (pr.margin[i] > pr.rounding[i]):pr.normal = False
            if not (pr.margin[i] < -pr.rounding[i]):pr.reversed_ = False
        return 0

    cdef dict pitch_dict(self, const PitchRes* pr):
        samples = [dict(step=pr.step[i], gain_rad_s2=pr.gain[i], margin=pr.margin[i], rounding=pr.rounding[i])
                   for i in range(pr.nsamples)]
        if pr.unresolved:
            return dict(normal=False, reversed=False, unresolved=True, reason='pitch response unresolved',
                        detail='Conditional wing history did not close', margin=NAN, gain_rad_s2=NAN,
                        samples=samples, failed_step=pr.failed_step, method='Native steady pitch response unresolved')
        i = pr.nsamples-1
        return dict(normal=pr.normal, reversed=pr.reversed_, margin=pr.margin[i], gain_rad_s2=pr.gain[i],
                    samples=samples, method='Native steady pitch moment derivative at fixed flight state; normal control sign')

    cdef dict pitch_response(self, Eval* e, object propulsion=None):
        # S2-S4 entry kept for reference: the dict of pitch_core
        cdef PitchRes pr
        self.pitch_core(e, propulsion is None, &pr)
        return self.pitch_dict(&pr)

    cdef object branch_tuple(self, const Eval* e, const int* br):
        key = [br[1]]
        if self.roll_leveling:key.append(-1 if e.air.alpha <= -12. else 1 if e.air.alpha >= 12. else 0)
        for i in range(2, 8):key.append(decode_branch(br[i]))
        # JointProblem.derivative_branch(value)[1:]: the stable mode ends with its Mach piece key; the native and
        # continuous modes lead with the exact Mach, which the source branch drops (S8)
        if not self.model.exact_mach:
            piece = [k for k, v in self.model.pieces.items() if v == br[0]][0]
            key.append(piece)
        return tuple(key)

    cdef void prop_flags(self, const PropSample* ps, Fin* f, int* overspeed, int* stopped) noexcept:
        """PropModel.record's converged / overspeed_engines / stopped_shafts, without the record."""
        cdef const PFrame* fr = &ps.frame
        cdef PropModel pm = self.pm
        cdef double error = 0., v
        cdef bint converged = True
        cdef int k, s
        for k in range(pm.npaths):
            v = ps.residual[k]
            if not isfinite(v):
                converged = False
            elif fabs(v) > error:
                error = fabs(v)
        converged = converged and error <= 1e-5
        for s in range(pm.nt):
            if pm.ngoverned[s] and not (ps.fractions[s] <= 2e-5 or ps.fractions[s] >= 1.-2e-5
                                        or fabs(fr.trans_omega[s]-ps.targets[s]) <= 1e-10*pmax(1., fabs(ps.targets[s]))):
                converged = False
        for k in range(3):
            if not (isfinite(fr.force[k]) and isfinite(fr.moment[k]) and isfinite(fr.momentum[k])):converged = False
        if not (isfinite(fr.wash[0]) and isfinite(fr.wash[1])):converged = False
        f.prop_converged = converged
        overspeed[0] = 0;stopped[0] = 0
        for k in range(pm.ne):
            if fr.engines[k].omega > pm.engines[k].omega_limit*(1+1e-6):overspeed[0] += 1
        for s in range(pm.nt):
            if fr.trans_omega[s] <= 1.:stopped[0] += 1

    cdef bint admitted_native(self, const PropSample* ps, const PropPrep* prep) noexcept:
        """all(row['admitted']) of native_admission_rows, without the rows."""
        cdef double density, sound, omega
        cdef Air air
        cdef int s, k, idx
        cdef const PFrame* fr = &ps.frame
        air_cache(prep.velocity, self.pm.altitude, &air)
        density = air.density
        sound = speed_of_sound(self.pm.altitude, F(288.16), F(18300.))
        for s in range(self.pm.nt):
            for k in range(self.pm.trans[s].np):
                idx = self.pm.trans[s].props[k].index
                omega = fr.trans_omega[s]*self.pm.trans[s].props[k].ratio
                if not self.admission_ok(&self.pm.props[idx], fr.props[idx].flow, fr.props[idx].pitch, omega, density, sound,
                                         prep):
                    return False
        return True

    cdef bint admission_ok(self, const PropT* p, const double* flow, double pitch, double omega, double density,
                           double sound, const PropPrep* prep) noexcept:
        # Finisher.admission's 'admitted' (prop_admission.evaluate)
        cdef double local[3]
        cdef double local_w[3]
        cdef double airflow[3]
        cdef double tsq, transverse, upstream, axial, angular, thrust, torque, power, tangential, t
        cdef int i, k
        local_flow32(p, prep.velocity, prep.rates, local, local_w, &tsq, &transverse)
        if local[0] <= p.airflow_x[0]:
            for i in range(3):airflow[i] = p.airflow_v[0][i]
        else:
            k = -1
            for i in range(p.nairflow-1):
                if local[0] <= p.airflow_x[i+1]:
                    k = i;break
            if k < 0:
                for i in range(3):airflow[i] = p.airflow_v[p.nairflow-1][i]
            else:
                t = (local[0]-p.airflow_x[k])/(p.airflow_x[k+1]-p.airflow_x[k])
                for i in range(3):airflow[i] = p.airflow_v[k][i]+(p.airflow_v[k+1][i]-p.airflow_v[k][i])*t
        upstream = local[0]+airflow[0]
        if not (isfinite(flow[0]) and isfinite(flow[1]) and isfinite(flow[2]) and isfinite(pitch) and isfinite(omega)
                and isfinite(density) and isfinite(sound) and isfinite(upstream)):return False
        if sound <= 0. or density <= 0. or p.area_unit <= 0. or p.blades <= 0:return False
        for k in range(4):
            if (p.station_radius[k] <= 0. or p.width[k] < 0. or not isfinite(p.station_radius[k])
                    or not isfinite(p.twist[k]) or not isfinite(p.width[k])):return False
        axial = local[0]+.5*(flow[0]+flow[1])
        if not (upstream > 0.) or not (axial > 0.):return False
        angular = 1.*.25*flow[2]
        blade_forces(p, omega, pitch, axial, transverse, density, sound, angular, &thrust, &torque)
        power = omega*torque
        for k in range(4):
            if p.width[k] == 0.:continue
            tangential = p.station_radius[k]*(omega-angular)
            if not (tangential > 0.):return False
        return isfinite(power) and isfinite(torque)

    cdef int compute(self, CurveProblem P, int slot, double speed_kmh, double load, long calls, RowSrc* s) except -1:
        """Every admission decision of the corrected state at memo ``slot`` into ``s`` (no Python records)."""
        cdef Eval* e
        cdef Fin* f = &s.fin
        cdef const Kinematic* k
        cdef const Frame* fr
        cdef int i, nover = 0, nstop = 0
        cdef bint converged
        cdef double rate_max, upright
        cdef PropPrep* prep
        cdef double inner[MAXPATH]
        s.e = P.memo_eval[slot]
        e = &s.e
        fr = &e.frame
        for i in range(BMAX):s.br[i] = P.memo_br[slot*BMAX+i]
        s.nx = P.nx;s.nres = 7+P.nprows
        for i in range(P.nz):s.q[i] = P.memo_q[slot*NMAX+i]
        for i in range(s.nres):s.resid[i] = P.memo_r[slot*NMAX+i]
        s.nreq = P.nreq
        for i in range(P.nreq):s.req[i] = P.req[i]
        s.speed_kmh = speed_kmh;s.load = load;f.calls = calls
        s.slide.n = 0
        ensure_complete(self.model, e)
        k = &e.kin
        if self.is_prop:
            prep = P.prepared(e.load, s.q[0], s.q[1])
            s.prep = prep[0]
            self.pm.use_slide(&P.slide)
            if self.pm.propulsion_q(prep, &s.q[7], P.req, P.nreq, inner, &s.ps, s.qq):
                raise RuntimeError('native finish: propulsion branch unavailable')
            charge(K_FRAMES, 1)
            self.prop_flags(&s.ps, f, &nover, &nstop)
            # S8: a sliding state lies on its jump only with theta in [0, 1] (else the regular state exists: no closure)
            s.slide = P.slide
            if s.slide.n > 0:
                self.pm.slide_result(&s.slide)
                for i in range(NP):
                    if s.slide.flag[i] and not (-1e-9 <= s.slide.theta[i] <= 1.+1e-9):f.prop_converged = False
            f.frames = 1 if f.prop_converged else 0
        f.finite = True
        for i in range(5):
            f.finite = f.finite and isfinite(e.x[i]) and isfinite(e.residual[i])
        for i in range(3):
            f.finite = f.finite and isfinite(e.rate_residual[i]) and isfinite(fr.force[i]) and isfinite(fr.moment[i]) and isfinite(k.velocity[i])
        f.finite = (f.finite and isfinite(e.history_error) and isfinite(e.stall_margin) and isfinite(k.ps)
                    and isfinite(e.wing_ratios[0]) and isfinite(e.wing_ratios[1]) and isfinite(e.air.ias_u) and isfinite(e.air.mach))
        f.history_finite = True
        for i in range(2):
            f.history_finite = (f.history_finite and isfinite(e.cycle_input.aoa[i]) and isfinite(e.cycle_input.cl[i])
                                and isfinite(e.cycle_input.body[i]) and isfinite(fr.history.aoa[i]) and isfinite(fr.history.cl[i])
                                and isfinite(fr.history.body[i]) and isfinite(e.angle_residual[i]))
        f.history_finite = f.history_finite and isfinite(e.cycle_input.spin) and isfinite(fr.history.spin)
        f.finite = f.finite and f.history_finite
        rate_max = pmax(pmax(fabs(e.rate_residual[0])+0., fabs(e.rate_residual[1])+0.), fabs(e.rate_residual[2])+0.)
        converged = f.finite and e.force_error_g+0. <= 2e-4 and rate_max <= 5e-5 and e.history_error <= 2e-4
        f.converged = converged
        f.nreasons = 0
        f.brakes = False
        upright = 90.-fabs(e.x[1]) if isfinite(e.x[1]) else -1e308*10.
        if upright < 0.:self.add_reason(f, R_BANK)
        if not converged:self.add_reason(f, R_TRIM)
        if self.is_prop:
            if not f.prop_converged:self.add_reason(f, R_PROP_SETTLE)
            if self.constants['governor'] == 'pitch_stops' and nover:self.add_reason(f, R_RPM)
            if nstop:self.add_reason(f, R_SHAFT)
            if not f.prop_converged:self.add_reason(f, R_CYCLE)
        if not self.instructor_enabled and not e.alloc.reachable:self.add_reason(f, R_CONTROL)
        if e.stall_margin < 0:self.add_reason(f, R_STALL)
        if fabs(k.velocity[1]) > .005:self.add_reason(f, R_VSTEP)
        f.nstructural = 0
        if pmax(e.wing_ratios[0], e.wing_ratios[1]) > 1:f.structural[f.nstructural] = R_WING;f.nstructural += 1
        if e.air.ias_u > self.model.strength_ias:f.structural[f.nstructural] = R_IAS;f.nstructural += 1
        if e.air.mach > self.model.strength_mach:f.structural[f.nstructural] = R_MACH;f.nstructural += 1
        f.has_sweep_range = False
        if self.has_sweep:
            from wing_sweep import available as available_sweep
            sweep_range = available_sweep(self.solver.fm, self.solver.sweep_rows, e.air.mach)
            f.has_sweep_range = True;f.sweep_range[0] = sweep_range[0];f.sweep_range[1] = sweep_range[1]
            if not sweep_range[0]-1e-7 <= self.solver.model['sweep'] <= sweep_range[1]+1e-7:self.add_reason(f, R_SWEEP)
        if self.structural:
            for i in range(f.nstructural):self.add_reason(f, f.structural[i])
        if self.flaps_available:
            from flap_model import violations as flap_violations
            for text in flap_violations(self.solver.flap_profile, self.flaps_percent, e.air.ias_u, e.air.mach, self.structural,
                                        self.solver.model['sweep'] if self.has_sweep else None):
                self.add_reason(f, reason_code(text))
        f.has_pitch = False
        if converged and e.stall_margin >= 0. and (not f.nreasons or (self.soft and self.only_soft(f))):
            f.has_pitch = True
            if self.is_prop:
                set_phase(self.model, &s.ps)
                try:
                    self.pitch_core(e, False, &f.pitch)
                finally:
                    self.model.has_phase = False
            else:
                self.pitch_core(e, True, &f.pitch)
            if not f.pitch.normal:
                self.add_reason(f, R_PITCH_REV if f.pitch.reversed_ else R_PITCH_UNRES)
        f.instr = 0
        if self.instructor_enabled and (not f.nreasons or (self.soft and self.only_soft(f))):
            self.admit_instructor(s)
            if f.instr == 1:
                if not f.ires.converged:self.add_reason(f, R_INSTR_UNRES)
                else:
                    if f.ires.angle_margin < -2e-7:self.add_reason(f, R_INSTR)
                    if f.ires.mechanical < -2e-7:self.add_reason(f, R_CONTROL)
            else:
                instructor = self.python_instructors[f.instr_index]
                if not instructor['converged']:self.add_reason(f, R_INSTR_UNRES)
                else:
                    if instructor['envelope_margin'] < -2e-7:self.add_reason(f, R_INSTR)
                    if instructor['margins']['control authority with auto trim'] < -2e-7:self.add_reason(f, R_CONTROL)
        # control bounds and authority margin
        if f.instr == 1:
            for i in range(3):
                f.bounds[i][0] = f.ires.bounds[i][0];f.bounds[i][1] = f.ires.bounds[i][1]
        elif f.instr == 2:
            b = self.python_instructors[f.instr_index]['control_bounds']
            for i in range(3):
                f.bounds[i][0] = b[i][0];f.bounds[i][1] = b[i][1]
        else:
            for i in range(3):
                f.bounds[i][0] = e.alloc.bounds[i][0];f.bounds[i][1] = e.alloc.bounds[i][1]
        f.authority = authority_margin(e.alloc.commands, f.bounds)
        if fabs(e.air.beta) > 2.:self.add_reason(f, R_SIDESLIP)
        if self.is_prop:
            if self.python_admission:
                record = self.pm.record(&s.ps, s.qq)
                rows = self.admission_rows(record, (s.prep.velocity[0], s.prep.velocity[1], s.prep.velocity[2]),
                                           (s.prep.rates[0], s.prep.rates[1], s.prep.rates[2]))
                if any(not row['admitted'] for row in rows):self.add_reason(f, R_PROP_ADM)
            elif not self.admitted_native(&s.ps, &s.prep):
                self.add_reason(f, R_PROP_ADM)
            if self.brake and self.braking(&s.ps):
                # S11: under the choice rule a braking state keeps its own admission (MarchEngine.correct looks for a
                # pulling state of the same speed and load); S10 rejects it
                f.brakes = True
                if not self.brake_choice:self.add_reason(f, R_PROP_BRAKE)
        f.local_valid = not f.nreasons
        f.row_valid = f.local_valid and isfinite(k.ps)
        return 0

    cdef bint braking(self, const PropSample* ps) noexcept:
        """S10: em_stationary.propeller_braking on the native sample: the net thrust of the shafts' propellers (each
        propeller's force along its axis) is negative while their engines deliver power (sum of torque x shaft speed)."""
        return net_thrust(self.pm, &ps.frame) < 0. and shaft_power(self.pm, &ps.frame) > 0.

    cdef void add_reason(self, Fin* f, int code) noexcept:
        if f.nreasons < MAXREASONS:
            f.reasons[f.nreasons] = code;f.nreasons += 1

    cdef bint only_soft(self, const Fin* f) noexcept:
        """S8: every reason so far is a soft chart limit (engine RPM, a forward-IAS limit)."""
        cdef int i
        for i in range(f.nreasons):
            if not soft_code(f.reasons[i]):return False
        return True

    cdef int admit_instructor(self, RowSrc* s) except -1:
        """Instructor admission of the state: native lean controller, else the Python path on the value record."""
        cdef Fin* f = &s.fin
        cdef Instructor inst = self.instructor
        if inst is not None:
            try:
                inst.eval_limits(&s.e, &s.ps if self.is_prop else NULL, &f.ires)
                f.instr = 1
                if self.loop_stats is not None:self.loop_stats['native_instructor'] += 1
                return 0
            except InstructorUnsupported as error:
                if self.loop_stats is not None:
                    reasons = self.loop_stats.setdefault('python_instructor_reasons', {})
                    reasons[str(error)] = reasons.get(str(error), 0)+1
        value = value_record(self.model, &s.e, True, True)
        if self.is_prop:
            record = self.pm.record(&s.ps, s.qq)
            value['propulsion'] = record;value['propulsion_phase_frames'] = s.fin.frames
        instructor = self.solver.instructor_at(value, s.speed_kmh/3.6, s.load)
        f.instr = 2;f.instr_index = len(self.python_instructors)
        self.python_instructors.append(instructor)
        return 0

    # ------------------------------------------------------------------ the chart row
    cdef dict point(self, RowSrc* s):
        """JointProblem.finish(x) of the state (the certify point before the chart sample)."""
        import math
        cdef Eval* e = &s.e
        cdef const Kinematic* k = &e.kin
        cdef const Frame* f = &e.frame
        cdef Fin* fi = &s.fin
        cdef double speed = s.speed_kmh/3.6
        cdef double speed_cert = speed*3.6
        cdef double upright
        cdef int i
        c = self.constants
        record = None;frames = 0;requests = [];velocity = rates = None
        cdef double tt0 = s5_now(), tt1
        if self.is_prop:
            record = self.pm.record(&s.ps, s.qq)
            self.t_record += s5_now()-tt0
            velocity = (s.prep.velocity[0], s.prep.velocity[1], s.prep.velocity[2])
            # prop_requests.enumerate_speeds depends on the flight condition only: computed once per velocity
            requests = self.request_cache.get(velocity)
            if requests is None:
                requests = self.pm.request_records(&s.prep)
                if len(self.request_cache) >= 256:self.request_cache.clear()
                self.request_cache[velocity] = requests
            rates = (s.prep.rates[0], s.prep.rates[1], s.prep.rates[2])
            frames = fi.frames
        converged = fi.converged
        upright = 90.-fabs(e.x[1]) if isfinite(e.x[1]) else -1e308*10.
        reasons = [REASON_TEXT[fi.reasons[i]] for i in range(fi.nreasons)]
        structural = [REASON_TEXT[fi.structural[i]] for i in range(fi.nstructural)]
        phase_check = None
        if self.is_prop:
            phase_check = dict(checked=False, force_error_g=None, angular_error_rad_s2=None, history_error=None)
            if record.get('cycle_samples') and frames == len(record['cycle_samples']) and converged:
                phase_check = dict(checked=True, model='Quasi-steady ideal-governor aircraft equilibrium',
                    force_error_g=e.force_error_g,
                    angular_error_rad_s2=float(pmax(pmax(fabs(e.rate_residual[0]), fabs(e.rate_residual[1])), fabs(e.rate_residual[2]))),
                    force_mean_uncertainty_g=0., angular_mean_uncertainty_rad_s2=[0., 0., 0.],
                    history_error=e.history_error, periodic_aircraft_trajectory_certified=False)
        sweep_range = (fi.sweep_range[0], fi.sweep_range[1]) if self.has_sweep else (0., 1.)
        tt1 = s5_now()
        pitch = self.pitch_dict(&fi.pitch) if fi.has_pitch else None
        self.t_admit += s5_now()-tt1
        instructor = None
        if fi.instr == 1:
            instructor = (<Instructor>self.instructor).limits_dict(&fi.ires, e.alloc.commands[1])
        elif fi.instr == 2:
            instructor = self.python_instructors[fi.instr_index]
        commands = L3(e.alloc.commands)
        bounds = [[fi.bounds[i][0], fi.bounds[i][1]] for i in range(3)]
        authority = fi.authority
        restrictions = []
        tt1 = s5_now()
        if self.is_prop:
            restrictions = (self.admission_rows(record, velocity, rates) if self.python_admission
                            else self.native_admission_rows(record, &s.prep))
        self.t_admit += s5_now()-tt1
        local_valid = fi.local_valid
        comp = {COMPONENT_NAMES[i]: L3(f.component_forces[i]) for i in range(8)}
        tt1 = s5_now()
        branch = self.branch_tuple(e, s.br)
        self.t_record += s5_now()-tt1
        source_branch = [[list(b) if isinstance(b, tuple) else b for b in branch], [], [s.req[i] for i in range(s.nreq)]
                         if self.is_prop else []]
        point = dict(speed_kmh=float(speed_cert), load_g=float(s.load), turn_dps=math.degrees(e.geom.turn_rate),
                     aircraft_trim_mode=c['trim_mode'], ps_definition='discrete energy step', ps_mps=fin(k.ps),
                     ps_continuous_mps=fin(k.ps_continuous), alpha_deg=e.x[0], bank_deg=e.x[1], sideslip_deg=e.air.beta,
                     sideslip_attitude_deg=0., upright_bank_margin_deg=fin(upright),
                     roll_leveling_branch=((-1 if e.air.alpha <= -12. else 1 if e.air.alpha >= 12. else 0)
                                           if self.roll_leveling else None),
                     converged=bool(converged), valid=local_valid, reasons=reasons, structural_flags=structural,
                     bounded_search_stationary=False, pitch_response=None if pitch is None else clean_pitch(pitch),
                     force_error_g=fin(e.force_error_g),
                     angular_error_rad_s2=fin(pmax(pmax(fabs(e.rate_residual[0]), fabs(e.rate_residual[1])), fabs(e.rate_residual[2]))),
                     history_error=fin(e.history_error), stall_margin_deg=fin(e.stall_margin),
                     history_components_finite=bool(fi.history_finite), negative_stall_margin_deg=fin(e.negative_stall_margin),
                     wing_load_ratios=[fin(e.wing_ratios[0]), fin(e.wing_ratios[1])], commands=commands,
                     control_bounds=bounds, authority_margin=fin(authority),
                     sticks=instructor.get('sticks', L3(e.alloc.sticks)) if instructor else L3(e.alloc.sticks),
                     trim=instructor['auto_trim'] if instructor else L3(e.alloc.trim),
                     ias_kmh=fin(e.air.ias_u*3.6), mach=fin(e.air.mach),
                     sweep_percent=c['sweep_percent'] if self.has_sweep else None,
                     flaps_percent=self.model.flaps*100., flaps_requested_percent=c['flaps_percent'],
                     gear_percent=c['gear_percent'], instructor=instructor, instructor_enabled=c['instructor'],
                     sweep_available_percent=[v*100 for v in sweep_range] if self.has_sweep else None,
                     force_n=L3(f.force), moment_nm=L3(f.moment), engine_force_n=L3(f.engine_force), component_forces=comp,
                     evaluations=fi.calls, solution=[e.x[0], e.x[1], e.x[2], e.x[3], e.x[4]],
                     vertical_step_velocity_mps=k.velocity[1], altitude_correction=bool(k.altitude_correction),
                     actual_turn_dps=math.degrees(math.atan2(k.velocity[2], k.velocity[0])/self.model.dt))
        tt1 = s5_now()
        if self.is_prop:
            self.propulsion_report(point, record, converged, phase_check)
        self.t_report += s5_now()-tt1
        if self.is_prop:
            L = self.layout;nprop = len(L['names'])-7
            names = list(L['names']);scales = list(SCALES7)+[.0001]*nprop;rscales = list(RSCALES7)+[1e-5]*nprop
            bounds = [list(L['lower']), list(L['upper'])];request_list = [s.req[i] for i in range(s.nreq)]
        else:
            names = list(NAMES7);scales = list(SCALES7);rscales = list(RSCALES7);bounds = [list(LOWER7), list(UPPER7)]
            request_list = []
        point.update(local_valid=local_valid, valid=local_valid, status='admitted' if local_valid else 'unresolved',
                     validity_scope='Numerical stationary point with original physical admission',
                     selection_status='numerical_candidate' if local_valid else 'unresolved',
                     model_revision=c['revision'], stationary_coordinates=[s.q[i] for i in range(s.nx)],
                     stationary_coordinate_names=names, stationary_coordinate_scales=scales,
                     stationary_correction_residual_scales=rscales, stationary_history_domain=None,
                     source_branch=source_branch, stationary_request_indices=request_list, stationary_governor_mode=c['governor'],
                     stationary_residual=[fin(s.resid[i]) for i in range(s.nres)],
                     propeller_admission=restrictions,
                     request_records=requests, equilibrium_coverage_complete=False, governor_selection_certified=False,
                     aircraft_selection_certified=False, numerical_error_certified=False, absence_certified=False,
                     search_unresolved=None if local_valid else 'No locally admitted stationary point',
                     search_bounds=bounds, model_experiments=dict(c['description']),
                     history_closure_required=True, symmetry_projection=None, mach_curve_mode=c['mach_mode'])
        if c['mach_mode'] == 'stable':point['mach_curve_revision'] = c['stable_revision']
        if s.slide.n > 0:point['propeller_sliding'] = slide_record(&s.slide)
        if fi.brakes and self.brake_choice:point['propeller_braking'] = True   # S11: admitted, no pulling state found here
        return point

    cdef dict row(self, RowSrc* s):
        """BaselineSolver.sample(dict(point, stationary_candidates=[point], stationary_attempts=[])) after
        point['selected_candidates'] = [] (chart_point, candidate summaries, clean): the Curve.correct row."""
        c = self.constants
        point = self.point(s)
        local_valid = point['local_valid']
        brief = {key: point[key] for key in BRIEF_FIELDS if key in point}
        row = dict(point)
        row.update(selected_candidates=[], stationary_candidates=[brief], stationary_attempts=[],
                   chart_usable=False, valid=False, status='unresolved', equilibrium_coverage_complete=False,
                   aircraft_selection_certified=False, governor_selection_certified=False,
                   chart_status='no locally admitted candidate', point_sep_error_budget_mps=c['budget'],
                   point_sep_error_certified=False, chart_grouping_sep_budget_mps=c['budget'])
        if local_valid:
            spread = point['ps_mps']-point['ps_mps']
            usable = bool(spread <= c['budget'])
            row.update(chart_sep_spread_mps=spread, chart_candidate=brief, chart_usable=usable, valid=usable,
                       status='admitted' if usable else 'unresolved',
                       chart_status='nearby local candidates' if usable else 'multiple numerical candidate branches')
        row['valid'] = row['chart_usable']
        row['validity_scope'] = 'numerical equilibrium and physical admission'
        cdef double tt0 = s5_now()
        out = fast_clean(row)
        self.t_clean += s5_now()-tt0
        return out

    def finish(self, CurveProblem P, int slot, double speed_kmh, double load, x7, long calls):
        """Cleaned chart row for the corrected state, as BaselineSolver.sample(JointProblem.finish(x)) (S2-S4 entry)."""
        cdef RowSrc* s = <RowSrc*>malloc(sizeof(RowSrc))
        if s == NULL:raise MemoryError()
        try:
            self.compute(P, slot, speed_kmh, load, calls, s)
            row = self.row(s)
        finally:
            cfree(s)
        from em_trim_work import record_point
        record_point(row)
        return row

    cdef list admission_rows(self, record, velocity, rates):
        """JointProblem.finish propeller restrictions: the baseline source-value admission
        (prop_admission.evaluate) on the native record's rotor states."""
        import em_stationary as ST
        air, sound = ST.fixed_air(velocity, self.solver.config['altitude_m'])
        p = self.solver.engine.properties
        state = record['state']
        out = []
        for i, shaft in enumerate(p['transmissions']):
            for link in shaft['propellers']:
                index = link['index']
                pp = p['propellers'][index]['properties']
                rotor = state['propellers'][index]
                row = ST.admission(pp, rotor['flow'], rotor['pitch'], state['transmissions'][i]['omega']*link['ratio'],
                                   air['density'], sound, ST.propeller_condition(pp, velocity, rates))
                out.append(dict(row, propeller=index, shaft=i))
        return out

    cdef list native_admission_rows(self, record, const PropPrep* prep):
        """prop_admission.evaluate for every propeller (non-coaxial supported subset), native."""
        cdef double density, sound
        cdef Air air
        cdef int s, k, idx
        cdef double omega
        air_cache(prep.velocity, self.pm.altitude, &air)
        density = air.density
        sound = speed_of_sound(self.pm.altitude, F(288.16), F(18300.))
        state = record['state']
        out = []
        for s in range(self.pm.nt):
            omega_shaft = state['transmissions'][s]['omega']
            for k in range(self.pm.trans[s].np):
                idx = self.pm.trans[s].props[k].index
                rotor = state['propellers'][idx]
                omega = omega_shaft*self.pm.trans[s].props[k].ratio
                row = self.admission(&self.pm.props[idx], rotor['flow'], rotor['pitch'], omega, density, sound, prep)
                out.append(dict(row, propeller=idx, shaft=s))
        return out

    cdef dict admission(self, const PropT* p, flow_in, double pitch, double omega, double density, double sound,
                        const PropPrep* prep):
        import math
        cdef double local[3]
        cdef double local_w[3]
        cdef double airflow[3]
        cdef double flow[3]
        cdef double tsq, transverse, upstream, axial, angular, thrust, torque, power, tangential, phi, alpha, mach, t
        cdef BPolar polar
        cdef int i, k, n
        cdef bint bad = False
        for i in range(3):flow[i] = flow_in[i]
        local_flow32(p, prep.velocity, prep.rates, local, local_w, &tsq, &transverse)
        if local[0] <= p.airflow_x[0]:
            for i in range(3):airflow[i] = p.airflow_v[0][i]
        else:
            k = -1
            for i in range(p.nairflow-1):
                if local[0] <= p.airflow_x[i+1]:
                    k = i;break
            if k < 0:
                for i in range(3):airflow[i] = p.airflow_v[p.nairflow-1][i]
            else:
                t = (local[0]-p.airflow_x[k])/(p.airflow_x[k+1]-p.airflow_x[k])
                for i in range(3):airflow[i] = p.airflow_v[k][i]+(p.airflow_v[k+1][i]-p.airflow_v[k][i])*t
        upstream = local[0]+airflow[0]
        for v in (flow[0], flow[1], flow[2], pitch, omega, density, sound, upstream):
            if not math.isfinite(v):bad = True
        if sound <= 0. or density <= 0. or p.area_unit <= 0. or p.blades <= 0:bad = True
        for k in range(4):
            if (p.station_radius[k] <= 0. or p.width[k] < 0. or not isfinite(p.station_radius[k])
                    or not isfinite(p.twist[k]) or not isfinite(p.width[k])):bad = True
        if bad:
            return dict(available=False, admitted=False, revision=self.admission_revision,
                        reason='Invalid source geometry or nonfinite propeller inputs')
        axial = local[0]+.5*(flow[0]+flow[1])
        margins = [dict(name='upstream', value=upstream, strict=True),
                   dict(name='blade axial flow', value=axial, strict=True)]
        if p.coaxial:
            # propeller_load's wake target[0]: the first rotor's induced flow
            blade_forces(p, omega, pitch, axial, transverse, density, sound, .25*flow[2], &thrust, &torque)
            wake = induced_flow(thrust, upstream, density, M_PI*.25*pow(p.diameter, 2.), airflow[2])
            margins.append(dict(name='second rotor upstream', value=upstream+.5*wake, strict=True))
        rotors = []
        signs = [1., -1.] if p.coaxial else [1.]
        for rotor, sgn in enumerate(signs):
            angular = sgn*.25*flow[2]
            blade_forces(p, omega, pitch, axial, transverse, density, sound, angular, &thrust, &torque)
            power = omega*torque
            rotor_margins = [];rows = []
            for k in range(4):
                if p.width[k] == 0.:continue
                tangential = p.station_radius[k]*(omega-angular)
                phi = atan2(axial, tangential) if fabs(omega) >= 1e-5 else M_PI/2
                alpha = (pitch-phi+p.twist[k])*RAD_TO_DEG
                mach = hypot2(tangential+transverse, axial)/sound
                evaluate_polar(&p.polar, mach, &polar)
                section = [dict(name='tangential flow', value=tangential, strict=True)]
                rows.append(dict(section=k, alpha_deg=alpha, mach=mach, critical_deg=[polar.aoaCritL, polar.aoaCritH],
                                 stall_margins_deg=dict(lower_deg=alpha-polar.aoaCritL, upper_deg=polar.aoaCritH-alpha),
                                 margins=section))
                rotor_margins.extend(dict(m, section=k) for m in section)
            rotors.append(dict(rotor=rotor, torque=torque, power=power, sections=rows, margins=rotor_margins))
            margins.extend(dict(m, rotor=rotor) for m in rotor_margins)
        finite_power = all(math.isfinite(r['power']) and math.isfinite(r['torque']) for r in rotors)
        available = finite_power and all(math.isfinite(m['value']) for m in margins)
        failures = [m for m in margins if not math.isfinite(m['value'])
                    or (m['value'] <= 0. if m['strict'] else m['value'] < 0.)]
        failures.extend(dict(name='nonfinite rotor torque or power', rotor=r['rotor'], value=r['power']) for r in rotors
                        if not math.isfinite(r['power']) or not math.isfinite(r['torque']))
        return dict(available=available, admitted=available and not failures, margins=margins, violations=failures,
                    rotors=rotors, revision=self.admission_revision, interval_certified=False)

    cdef void propulsion_report(self, dict point, record, bint converged, phase_check):
        # TrimSolver.certify: propeller report
        import math
        from em_data import clone
        engine = self.solver.engine
        p = record
        if converged and p['converged'] and phase_check['checked'] and engine.automatic:
            # the native record is built for this row alone: its state needs no defensive copy
            point['_propulsion_seed'] = p['state']
        point['propulsion'] = dict(controls=p['controls'], converged=p['converged'],
            canonical_initialization=p['canonical_initialization'],
            period_frames=p['period_frames'], window_relative_change=p['window_relative_change'],
            stationarity=p.get('stationarity'), sample_frames=len(p.get('cycle_samples') or []),
            phase_check=phase_check,
            engine_rpm=[e['omega']*60/(2*math.pi) for e in p['state']['engines']],
            radiator_percent=self.solver.radiator*100., oil_radiator_percent=self.solver.oil_radiator*100.,
            rpm_threshold_exceeded_engines=p['overspeed_engines'],
            engine_rpm_threshold=[e['properties']['omega_limit']*60/(2*math.pi) for e in engine.properties['engines']],
            compressor_stages=[e.get('gear', 0) for e in p['state']['engines']],
            engine_control_mode=p['controls'].get('engine_control_mode', 'optimized'),
            mixture=[e.get('mixture') for e in p['state']['engines']],
            shaft_power_kw=[e.get('torque', 0.)*e['omega']/1000. for e in p['state']['engines']],
            propeller_pitch_deg=[e['pitch']*180/math.pi for e in p['state']['propellers']],
            wash_mps=p['wash'], angular_momentum=p['angular_momentum'])
        if point['propulsion']['engine_control_mode'] == 'automatic':
            point['propulsion']['optimization'] = dict(method='Dynamic propulsion with automatic controls', evaluated=0,
                global_optimum_certified=False, radiators_closed=not (self.solver.radiator or self.solver.oil_radiator))
        elif engine.quasi_steady:
            point['propulsion']['optimization'] = dict(method='Quasi-steady selected manual pitch' if
                any(not auto for auto in p['controls']['automatic']) else 'Quasi-steady ideal RPM governor', evaluated=0,
                global_optimum_certified=False, radiators_closed=not (self.solver.radiator or self.solver.oil_radiator))


cdef double instructor_event(object inst, const Eval* e, const PropSample* ps, int kind, int axis, int side) except? -1e308:
    """Curve.events Instructor residuals on a search record (instructor_at completes it first):
    pitch: envelope_margin-2e-6 if converged else nan; control: (+-1.)*(commands[axis]-bounds[axis][side])-3e-5."""
    cdef InstrRes r
    cdef Instructor native = <Instructor>inst
    native.eval_limits(<Eval*>e, ps, &r)
    if native.loop_stats is not None:native.loop_stats['native_instructor'] += 1
    if kind == 4:
        return r.angle_margin-2e-6 if r.converged else NAN
    return (1. if side == 0 else -1.)*(e.alloc.commands[axis]-r.bounds[axis][side])-3e-5


cdef void set_phase(Model m, const PropSample* ps) noexcept:
    """Model.set_propulsion(record) of the native propulsion record (its cycle row)."""
    cdef int i
    for i in range(3):
        m.cur_phase[i] = ps.frame.force[i];m.cur_phase[3+i] = ps.frame.moment[i];m.cur_phase[6+i] = ps.frame.momentum[i]
    m.cur_phase[9] = ps.frame.wash[0];m.cur_phase[10] = ps.frame.wash[1]
    m.has_phase = True


cdef double authority_margin(const double* commands, double bounds[3][2]) noexcept:
    # min(min(d-lo, hi-d) for d, (lo, hi) in zip(commands, bounds)) with Python's min tie rules
    cdef double best = 0., v
    cdef int i
    for i in range(3):
        v = pmin(commands[i]-bounds[i][0], bounds[i][1]-commands[i])
        best = v if i == 0 else pmin(best, v)
    return best


cdef object fast_clean(object v):
    """em_stationary_chart.clean (export normalization): nonfinite floats to None, tuples and arrays to lists,
    no '_detail' keys. Copy on change: a container that is already clean is returned itself (S5)."""
    cdef type t = type(v)
    cdef Py_ssize_t i, n
    cdef bint changed
    if t is float:
        return v if isfinite(<double>v) else None
    if t is dict:
        changed = False
        for key, x in (<dict>v).items():
            if type(key) is not str or key == '_detail':
                changed = True;break
            y = fast_clean(x)
            if y is not x:
                changed = True;break
        if not changed:return v
        return {str(key): fast_clean(x) for key, x in (<dict>v).items() if key != '_detail'}
    if t is list:
        n = len(<list>v)
        for i in range(n):
            x = (<list>v)[i]
            y = fast_clean(x)
            if y is not x:
                out = list(<list>v)[:i]+[y]
                for j in range(i+1, n):out.append(fast_clean((<list>v)[j]))
                return out
        return v
    if t is tuple:
        return [fast_clean(x) for x in v]
    if t is str or t is int or t is bool or v is None:
        return v
    if isinstance(v, np.ndarray):
        return [fast_clean(x) for x in v.tolist()]
    if isinstance(v, np.generic):
        return fast_clean(v.item())
    if isinstance(v, dict):
        return {str(key): fast_clean(x) for key, x in v.items() if key != '_detail'}
    if isinstance(v, (list, tuple)):
        return [fast_clean(x) for x in v]
    if isinstance(v, float):
        return float(v) if isfinite(<double>v) else None
    return v


cdef dict clean_pitch(dict pitch):
    out = dict(pitch)
    for key in ('margin', 'gain_rad_s2'):
        v = out[key]
        if isinstance(v, float) and not math_isfinite(v):out[key] = None
    out['samples'] = [{k: (None if isinstance(v, float) and not math_isfinite(v) else v) for k, v in s.items()}
                      for s in pitch['samples']]
    return out


cdef inline bint math_isfinite(double v) noexcept:
    return isfinite(v)
