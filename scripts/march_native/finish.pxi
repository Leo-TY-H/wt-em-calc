# Native finish of a stationary correction: JointProblem.finish -> TrimSolver.certify
# -> BaselineSolver.sample (chart_point, candidate summaries, clean). Direct-thrust
# and supported propeller aircraft (native propulsion record, governed requests,
# phase-fixed pitch response; the source-value propeller admission rows are the
# baseline prop_admission on the native record). The admission rules are
# transliterated line by line; s2_native verify mode compares every row with the
# Python finish.

DEF NAN = float('nan')

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


cdef class Finisher:
    cdef Model model
    cdef FrozenProblem fp
    cdef public object solver
    cdef public dict constants
    cdef public long frozen_evaluations, frozen_derivative_samples
    cdef double length, inertia_z
    cdef bint invert_elevator, roll_leveling, structural, has_sweep
    cdef public bint is_prop, python_admission
    cdef object admission_revision
    cdef PropModel pm
    cdef public object layout

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
        if P.aircraft_evaluations:bulk_charge('aircraft_evaluations', P.aircraft_evaluations)
        if P.derivative_samples:bulk_charge('derivative_samples', P.derivative_samples)
        if P.nominal_comparisons:bulk_charge('nominal_sensitivity_comparisons', P.nominal_comparisons)
        if P.nominal_restorations:bulk_charge('nominal_sensitivity_restorations', P.nominal_restorations)
        if P.branch_secants:bulk_charge('branch_secant_predictors', P.branch_secants)
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

    cdef dict pitch_response(self, Eval* e, object propulsion=None) :
        cdef FrozenState st
        cdef double x[5]
        cdef double lo[5]
        cdef double hi[5]
        cdef double hist[2]
        cdef double sign, scale, step, width, ma, mb, slope, rounding
        cdef bint ca, cb
        cdef int i, k
        cdef double[2] steps_ = [.002, .0005]
        previous = self.solver.__dict__.get('_stationary_frozen_predictor')
        st.valid = False;st.has_pred = False
        if previous is not None and previous['key'][2] == 0. and previous['propulsion'] is propulsion:
            st.valid = True;st.speed = previous['key'][0];st.load = previous['key'][1]
            for i in range(5):st.attitude[i] = float(previous['attitude'][i])
            st.angles[0] = float(previous['angles'][0]);st.angles[1] = float(previous['angles'][1])
            if isinstance(previous['predictor'], NativePredictor):
                st.has_pred = True;st.pred = (<NativePredictor>previous['predictor']).p
        for i in range(5):x[i] = e.x[i]
        hist[0] = e.cycle_input.aoa[0];hist[1] = e.cycle_input.aoa[1]
        sign = 1. if self.invert_elevator else -1.
        scale = self.model.weight*self.length
        samples = []
        for k in range(2):
            step = steps_[k]
            for i in range(5):
                lo[i] = x[i];hi[i] = x[i]
            lo[3] = pmax(-1., x[3]-step);hi[3] = pmin(1., x[3]+step)
            self.frozen(&st, e.speed, e.load, lo, hist, &ma, &ca)
            self.frozen(&st, e.speed, e.load, hi, hist, &mb, &cb)
            if not ca or not cb:
                return dict(normal=False, reversed=False, unresolved=True, reason='pitch response unresolved',
                            detail='Conditional wing history did not close', margin=NAN, gain_rad_s2=NAN,
                            samples=samples, failed_step=step, method='Native steady pitch response unresolved')
            width = hi[3]-lo[3]
            slope = sign*(mb-ma)/width
            rounding = 8.*(fabs(spacing32(ma))+fabs(spacing32(mb)))/width
            samples.append(dict(step=step, gain_rad_s2=slope/self.inertia_z, margin=slope/scale, rounding=rounding/scale))
            if fabs(slope) > 32.*rounding and fabs(slope/scale) > .002:
                break
        last = samples[len(samples)-1]
        normal = all(p['margin'] > p['rounding'] for p in samples)
        reversed_response = all(p['margin'] < -p['rounding'] for p in samples)
        return dict(normal=normal, reversed=reversed_response, margin=last['margin'], gain_rad_s2=last['gain_rad_s2'],
                    samples=samples, method='Native steady pitch moment derivative at fixed flight state; normal control sign')

    cdef object branch_tuple(self, Eval* e, const int* br):
        key = [br[1]]
        if self.roll_leveling:key.append(-1 if e.air.alpha <= -12. else 1 if e.air.alpha >= 12. else 0)
        for i in range(2, 8):key.append(decode_branch(br[i]))
        piece = [k for k, v in self.model.pieces.items() if v == br[0]][0]
        key.append(piece)
        return tuple(key)

    def finish(self, CurveProblem P, int slot, double speed_kmh, double load, x7, long calls):
        """Cleaned chart row for the corrected state, as BaselineSolver.sample(JointProblem.finish(x))."""
        import math
        from flap_model import violations as flap_violations
        cdef Eval* e = &P.memo_eval[slot]
        cdef const Kinematic* k
        cdef const Frame* f = &e.frame
        cdef double speed = speed_kmh/3.6
        cdef double speed_cert = speed*3.6
        cdef int i
        cdef bint finite, history_finite, converged
        cdef double upright, rate_max
        cdef int* br = &P.memo_br[slot*BMAX]
        c = self.constants
        ensure_complete(self.model, e)
        k = &e.kin
        cdef PropPrep* prep
        cdef PropSample ps
        cdef double qq[MAXPATH]
        cdef double inner[MAXPATH]
        cdef double* q = &P.memo_q[slot*NMAX]
        record = None;frames = 0;requests = [];velocity = rates = None
        if self.is_prop:
            prep = P.prepared(e.load, q[0], q[1])
            if self.pm.propulsion_q(prep, &q[7], P.req, P.nreq, inner, &ps, qq):
                raise RuntimeError('native finish: propulsion branch unavailable')
            record = self.pm.record(&ps, qq)
            bulk_charge('engine_frames', 1)
            requests = self.pm.request_records(prep)
            velocity = (prep.velocity[0], prep.velocity[1], prep.velocity[2])
            rates = (prep.rates[0], prep.rates[1], prep.rates[2])
            frames = self.model.set_propulsion(record);self.model.has_phase = False
        finite = True
        for i in range(5):
            finite = finite and isfinite(e.x[i]) and isfinite(e.residual[i])
        for i in range(3):
            finite = finite and isfinite(e.rate_residual[i]) and isfinite(f.force[i]) and isfinite(f.moment[i]) and isfinite(k.velocity[i])
        finite = (finite and isfinite(e.history_error) and isfinite(e.stall_margin) and isfinite(k.ps)
                  and isfinite(e.wing_ratios[0]) and isfinite(e.wing_ratios[1]) and isfinite(e.air.ias_u) and isfinite(e.air.mach))
        history_finite = True
        for i in range(2):
            history_finite = (history_finite and isfinite(e.cycle_input.aoa[i]) and isfinite(e.cycle_input.cl[i])
                              and isfinite(e.cycle_input.body[i]) and isfinite(f.history.aoa[i]) and isfinite(f.history.cl[i])
                              and isfinite(f.history.body[i]) and isfinite(e.angle_residual[i]))
        history_finite = history_finite and isfinite(e.cycle_input.spin) and isfinite(f.history.spin)
        finite = finite and history_finite
        rate_max = pmax(pmax(fabs(e.rate_residual[0])+0., fabs(e.rate_residual[1])+0.), fabs(e.rate_residual[2])+0.)
        converged = finite and e.force_error_g+0. <= 2e-4 and rate_max <= 5e-5 and e.history_error <= 2e-4
        reasons = []
        upright = 90.-fabs(e.x[1]) if isfinite(e.x[1]) else -1e308*10.
        if upright < 0.:reasons.append('bank outside [-90, 90] degrees')
        if not converged:reasons.append('trim did not converge')
        phase_check = None
        if self.is_prop:
            if not record['converged']:reasons.append('propulsion did not settle')
            if c['governor'] == 'pitch_stops' and record['overspeed_engines']:reasons.append('engine RPM limit')
            if record['stopped_shafts']:reasons.append('propeller shaft stopped')
            phases = record.get('cycle_samples')
            phase_check = dict(checked=False, force_error_g=None, angular_error_rad_s2=None, history_error=None)
            if not phases or frames != len(phases):
                reasons.append('propulsion cycle unresolved')
            elif converged:
                phase_check = dict(checked=True, model='Quasi-steady ideal-governor aircraft equilibrium',
                    force_error_g=e.force_error_g,
                    angular_error_rad_s2=float(pmax(pmax(fabs(e.rate_residual[0]), fabs(e.rate_residual[1])), fabs(e.rate_residual[2]))),
                    force_mean_uncertainty_g=0., angular_mean_uncertainty_rad_s2=[0., 0., 0.],
                    history_error=e.history_error, periodic_aircraft_trajectory_certified=False)
        if not c['instructor'] and not e.alloc.reachable:reasons.append('control authority')
        if e.stall_margin < 0:reasons.append('post-stall')
        if fabs(k.velocity[1]) > .005:reasons.append('vertical step did not close')
        structural = []
        if pmax(e.wing_ratios[0], e.wing_ratios[1]) > 1:structural.append('wing force limit')
        if e.air.ias_u > self.model.strength_ias:structural.append('IAS limit')
        if e.air.mach > self.model.strength_mach:structural.append('Mach limit')
        if self.has_sweep:
            from wing_sweep import available as available_sweep
            sweep_range = available_sweep(self.solver.fm, self.solver.sweep_rows, e.air.mach)
            if not sweep_range[0]-1e-7 <= self.solver.model['sweep'] <= sweep_range[1]+1e-7:reasons.append('sweep unavailable')
        else:
            sweep_range = (0., 1.)
        if self.structural:reasons += structural
        reasons += flap_violations(self.solver.flap_profile, c['flaps_percent'], e.air.ias_u, e.air.mach, self.structural)
        pitch = None
        if converged and e.stall_margin >= 0. and not reasons:
            if self.is_prop:
                self.model.set_propulsion(record)
                try:
                    pitch = self.pitch_response(e, record)
                finally:
                    self.model.has_phase = False
            else:
                pitch = self.pitch_response(e)
            if not pitch['normal']:
                reasons.append('reversed pitch response' if pitch['reversed'] else 'pitch response unresolved')
        instructor = None
        if c['instructor'] and not reasons:
            # Instructor admission: the unchanged baseline controller on the native state record
            value = value_record(self.model, e, True, True)
            if self.is_prop:
                value['propulsion'] = record;value['propulsion_phase_frames'] = frames
            instructor = self.solver.instructor_at(value, speed, load)
            if not instructor['converged']:reasons.append('Instructor boundary unresolved')
            else:
                if instructor['envelope_margin'] < -2e-7:reasons.append('Instructor pitch limit')
                if instructor['margins']['control authority with auto trim'] < -2e-7:reasons.append('control authority')
        commands = L3(e.alloc.commands)
        if instructor is not None:
            bounds = instructor['control_bounds']
        else:
            bounds = [[e.alloc.bounds[i][0], e.alloc.bounds[i][1]] for i in range(3)]
        authority = min(min(d-lo, hi-d) for d, (lo, hi) in zip(commands, bounds))
        local_valid = not reasons
        restrictions = []
        if self.is_prop:
            restrictions = (self.admission_rows(record, velocity, rates) if self.python_admission
                            else self.native_admission_rows(record, prep))
        if abs(e.air.beta) > 2.:
            reasons.append('sideslip limit');local_valid = False
        if any(not row['admitted'] for row in restrictions):
            reasons.append('propeller admission')
        local_valid = not reasons
        comp = {COMPONENT_NAMES[i]: L3(f.component_forces[i]) for i in range(8)}
        branch = self.branch_tuple(e, br)
        source_branch = [[list(b) if isinstance(b, tuple) else b for b in branch], [], [P.req[i] for i in range(P.nreq)]
                         if self.is_prop else []]
        point = dict(speed_kmh=float(speed_cert), load_g=float(load), turn_dps=math.degrees(e.geom.turn_rate),
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
                     history_components_finite=bool(history_finite), negative_stall_margin_deg=fin(e.negative_stall_margin),
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
                     evaluations=calls, solution=[e.x[0], e.x[1], e.x[2], e.x[3], e.x[4]],
                     vertical_step_velocity_mps=k.velocity[1], altitude_correction=bool(k.altitude_correction),
                     actual_turn_dps=math.degrees(math.atan2(k.velocity[2], k.velocity[0])/self.model.dt))
        if self.is_prop:
            self.propulsion_report(point, record, converged, phase_check)
        if self.is_prop:
            L = self.layout;nprop = len(L['names'])-7
            names = list(L['names']);scales = list(SCALES7)+[.0001]*nprop;rscales = list(RSCALES7)+[1e-5]*nprop
            bounds = [list(L['lower']), list(L['upper'])];request_list = [P.req[i] for i in range(P.nreq)]
            residual_rows = 7+P.nprows
        else:
            names = list(NAMES7);scales = list(SCALES7);rscales = list(RSCALES7);bounds = [list(LOWER7), list(UPPER7)]
            request_list = [];residual_rows = 7
        point.update(local_valid=local_valid, valid=local_valid, status='admitted' if local_valid else 'unresolved',
                     validity_scope='Numerical stationary point with original physical admission',
                     selection_status='numerical_candidate' if local_valid else 'unresolved',
                     model_revision=c['revision'], stationary_coordinates=list(map(float, x7)),
                     stationary_coordinate_names=names, stationary_coordinate_scales=scales,
                     stationary_correction_residual_scales=rscales, stationary_history_domain=None,
                     source_branch=source_branch, stationary_request_indices=request_list, stationary_governor_mode=c['governor'],
                     stationary_residual=[fin(P.memo_r[slot*NMAX+i]) for i in range(residual_rows)],
                     propeller_admission=restrictions,
                     request_records=requests, equilibrium_coverage_complete=False, governor_selection_certified=False,
                     aircraft_selection_certified=False, numerical_error_certified=False, absence_certified=False,
                     search_unresolved=None if local_valid else 'No locally admitted stationary point',
                     search_bounds=bounds, model_experiments=dict(c['description']),
                     history_closure_required=True, symmetry_projection=None, mach_curve_mode=c['mach_mode'])
        if c['mach_mode'] == 'stable':point['mach_curve_revision'] = c['stable_revision']
        # BaselineSolver.sample(dict(point, stationary_candidates=[point], stationary_attempts=[])) after
        # point['selected_candidates'] = []: chart_point, candidate summaries, clean
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
        row = fast_clean(row)
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
        angular = 1.*.25*flow[2]
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
        rotors = [dict(rotor=0, torque=torque, power=power, sections=rows, margins=rotor_margins)]
        margins.extend(dict(m, rotor=0) for m in rotor_margins)
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
            point['_propulsion_seed'] = clone(p['state'])
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


cdef object fast_clean(object v):
    # em_stationary_chart.clean (export normalization) without its per-object memo
    cdef type t = type(v)
    if t is float:
        return v if isfinite(<double>v) else None
    if t is dict:
        return {str(key): fast_clean(x) for key, x in (<dict>v).items() if key != '_detail'}
    if t is list or t is tuple:
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
