# PropModel: the stationary propulsion block of JointProblem for supported
# propeller aircraft (prepare / enumerate_speeds / sample / propulsion_at).

from libc.stdlib cimport calloc, free as pfree

DEF MAXPATH = 40
DEF MAXCAND = 8

cdef struct PropPrep:
    # one flight condition (speed, load, alpha, bank)
    double velocity[3]
    double rates[3]
    int ncand[NP]
    double cand[NP][MAXCAND]
    bint governed_request[NP]
    # prop_equilibrium: the steady gas-generator (family 5) or spool (family 2) speed of each turbine seed
    double tseed[NP]


cdef unsigned long long cl_jump_sides(const BPolar* p, double alpha, unsigned long long code) noexcept nogil:
    """S9 (the blade-polar jump check): the side of ``alpha`` of every breakpoint of calc_cl64 where the lift jumps (the
    ends of the lift line, the +0.01 branch switch below its upper end, the critical angles and the post-stall
    breakpoints): a legacy polar keeps its stall-shape parabolas from the base polar while its lift line follows the
    Mach rows, so the lift can jump where a section leaves the lift line (Do 17 Z-7 propellers at 147-153 km/h, 1 g).
    A breakpoint where the lift is continuous is left out (no torque jump there)."""
    cdef double pts[12]
    cdef double b, d, lo, hi, mx = pmax(40., p.maxDistAng)
    cdef int k, n = 0
    pts[n] = p.aoaLineL;n += 1
    pts[n] = p.aoaLineH;n += 1
    pts[n] = p.aoaLineH-0.01;n += 1
    pts[n] = p.aoaCritH;n += 1
    pts[n] = p.aoaCritL;n += 1
    pts[n] = p.aoaCritH+p.parabAngle;n += 1
    pts[n] = p.aoaCritL-p.parabAngle;n += 1
    pts[n] = p.maxDistAng;n += 1
    pts[n] = -p.maxDistAng;n += 1
    pts[n] = mx;n += 1
    pts[n] = -mx;n += 1
    for k in range(n):
        b = pts[k]
        if not isfinite(b):continue
        d = 1e-7*(1.+fabs(b))
        lo = calc_cl64(p, b-d);hi = calc_cl64(p, b+d)
        if fabs(hi-lo) > 1e-4*(1.+fabs(lo)):
            code = (code ^ <unsigned long long>(32+2*k+(1 if alpha >= b else 0)))*1099511628211ULL
    return code


cdef struct SlideSpec:
    # S8: the shafts sliding on a blade-polar jump (PropModel.set_slide) and, for a state, where (theta, shaft speeds)
    int n
    bint flag[NP]
    double c0[NP]
    double frac[NP]
    double hint[NP]
    double cw[NP]          # the regular coordinate of the shaft at shaft speed w: cw + w/300 (to leave the jump)
    double theta[NP]
    double lo[NP]
    double hi[NP]


cdef int slide_from_row(object d, SlideSpec* out) except -1:
    """S8: the sliding spec of a chart row's propeller_sliding record (none: n = 0)."""
    cdef int s
    out.n = 0
    for s in range(NP):
        out.flag[s] = False;out.c0[s] = 0.;out.frac[s] = 0.;out.hint[s] = 0.;out.theta[s] = 0.;out.lo[s] = 0.;out.hi[s] = 0.
        out.cw[s] = 0.
    if not d:return 0
    for item in d.get('shafts', ()):
        s = int(item['shaft'])
        if s < 0 or s >= NP:continue
        out.flag[s] = True;out.c0[s] = item['theta_origin'];out.frac[s] = item['pitch_fraction']
        out.hint[s] = item['shaft_speed_rad_s'];out.theta[s] = item['theta'];out.lo[s] = item['shaft_speed_rad_s']
        out.hi[s] = item['shaft_speed_above_rad_s'];out.cw[s] = item['regular_offset'];out.n += 1
    return 0


cdef object slide_record(const SlideSpec* sp):
    """S8: the propeller_sliding record of a sliding state (None for a regular one)."""
    cdef int s
    if sp.n <= 0:return None
    return dict(method='Filippov sliding state on a blade-polar jump: the shaft at the jump, the two one-sided propeller '
                       'states mixed so that the shaft torque balances (S8)',
                shafts=[dict(shaft=s, theta=sp.theta[s], shaft_speed_rad_s=sp.lo[s], shaft_speed_above_rad_s=sp.hi[s],
                             shaft_rpm=sp.lo[s]*30./M_PI, theta_origin=sp.c0[s], pitch_fraction=sp.frac[s],
                             regular_offset=sp.cw[s])
                        for s in range(NP) if sp.flag[s]])


cdef struct PropSample:
    double residual[MAXPATH]     # per prep path (scaled)
    double row[11]               # aggregate force, moment, angular momentum, wash
    bint converged
    double fractions[NP]
    double targets[NP]
    double torques[NP]
    PFrame frame


class PropUnsupported(ValueError):
    pass


cdef class PropModel:
    cdef int ne, nprops, nt
    cdef EngineT engines[NP]
    cdef PropT props[NP]
    cdef TransT trans[NP]
    cdef EngState seed_engines[NP]
    cdef PropState seed_props[NP]
    cdef double seed_omega[NP]
    cdef double altitude, nitro, dt
    cdef double cg[3]
    cdef bint torque_gyro, governor_saturation
    cdef int npaths
    cdef int path_kind[MAXPATH]      # 0 shaft, 1 pitch, 2 flow
    cdef int path_i[MAXPATH]
    cdef int path_j[MAXPATH]
    cdef double scales[MAXPATH]
    cdef double lower[MAXPATH]
    cdef double upper[MAXPATH]
    cdef double initial[MAXPATH]
    cdef int ngoverned[NP]
    cdef LinkT governed[NP][NP]
    cdef double shaft_targets[NP]
    cdef double torque_scales[NP]
    cdef public int nactive, nrows
    cdef int active[MAXPATH]
    cdef int rows[MAXPATH]
    cdef public object paths, py_controls, py_seed, py_engines, py_props, py_trans
    cdef public object revision, early_stop
    cdef Unit* units
    cdef bint linked[NP]
    cdef public long samples
    cdef double dt_aircraft
    # S8: the sliding (Filippov) state of a shaft on a jump of its propeller's blade polar (Leo, 2026-10-19): where the
    # shaft torque jumps through zero as the shaft speed crosses a blade-polar discontinuity (a section Mach on a
    # degenerate Mach row, the degenerate lift-line switch), the shaft sits on the jump and the two one-sided propeller
    # states are mixed by theta in [0, 1] so that the shaft torque balances. The shaft's own coordinate carries theta
    # (theta = coordinate - slide_c0); its pitch fraction is held (slide_frac: the stop it is on, or a fixed pitch).
    cdef public int nslide
    cdef bint slide[NP]
    cdef double slide_c0[NP]
    cdef double slide_frac[NP]
    cdef double slide_hint[NP]
    cdef int slide_coord[NP]
    cdef public double slide_width
    # the last sliding evaluation (for records): theta and the shaft speeds either side of the jump
    cdef double slide_theta[NP]
    cdef double slide_lo[NP]
    cdef double slide_hi[NP]
    # S9: theta's coordinate range kept inside the shaft coordinate's bounds (set by MarchEngine; False: S8)
    cdef public bint s9

    def __cinit__(self, *args, **kwargs):
        self.units = NULL
        self.s9 = False

    def __dealloc__(self):
        if self.units != NULL:pfree(self.units)

    def __init__(self, solver):
        cdef EngineT* E
        cdef PropT* P
        cdef BPolar* Q
        cdef EngState* S_
        cdef TransT* T
        from prop_steady import initial_state
        from em_manual_controls import manual_rpm_governor
        from prop_quasisteady import prepare_blades
        from polar_model import mach_curve
        from component_assembly import f32
        engine = solver.engine
        p = engine.properties
        cfg = solver.config
        if len(p['engines']) > NP or len(p['propellers']) > NP or len(p['transmissions']) > NP:
            raise PropUnsupported('too many engines, propellers or shafts')
        self.ne = len(p['engines']);self.nprops = len(p['propellers']);self.nt = len(p['transmissions'])
        self.altitude = cfg['altitude_m'];self.nitro = solver.mass['nitro_mass'];self.dt = engine.dt
        for i in range(3):self.cg[i] = solver.mass['cog'][i]
        self.torque_gyro = bool(cfg['torque_gyro'])
        self.governor_saturation = cfg.get('stationary_governor_mode', 'pitch_stops') == 'pitch_stops'
        controls = engine.fixed_controls or engine.automatic_controls
        self.py_controls = controls
        for k, e in enumerate(p['engines']):
            ep = e['properties'];E = &self.engines[k]
            if e['family'] not in (0, 1, 2, 3, 5):
                raise PropUnsupported('engine family')
            if e['family'] in (0, 1) and ep.get('compressor_type', 0) not in (1, 2, 3):
                raise PropUnsupported('engine compressor')
            if e['family'] in (0, 1) and ep.get('compressor_type', 0) == 3 and not ep.get('exact_altitudes', 0):
                raise PropUnsupported('alternate turbo-supercharger curve')
            E.family = e['family'];E.compressor_type = ep.get('compressor_type', 0);E.carburetor = ep.get('carburetor', 0)
            E.mixer_type = ep.get('mixer_type', 0);E.boost_type = ep.get('boost_type', 0);E.cylinders = ep.get('cylinders', 0)
            E.exact_altitudes = bool(ep.get('exact_altitudes', 0));E.ata_enabled = bool(ep.get('ata_enabled', 0))
            E.manual_compressor = bool(ep.get('manual_compressor', 0));E.boost_controllable = bool(ep.get('boost_controllable', 0))
            for name in ('base_hp', 'min_throttle', 'max_throttle', 'max_omega', 'afterburner_omega', 'omega_limit',
                         'inverse_omega', 'compressor_reference', 'torque_base', 'mixer_scale', 'compressor_sq', 'max_ata',
                         'ram_recovery', 'boost_ata_ratio', 'compressor_zero', 'throttle_boost', 'afterburner_boost',
                         'reservoir_capacity'):
                pass
            E.base_hp = ep.get('base_hp', 0);E.min_throttle = ep.get('min_throttle', 0);E.max_throttle = ep.get('max_throttle', 0)
            E.max_omega = ep.get('max_omega', 0);E.afterburner_omega = ep.get('afterburner_omega', 0);E.omega_limit = ep.get('omega_limit', 0)
            E.inverse_omega = ep.get('inverse_omega', 0);E.compressor_reference = ep.get('compressor_reference', 0)
            E.torque_base = ep.get('torque_base', 0);E.mixer_scale = ep.get('mixer_scale', 0);E.compressor_sq = ep.get('compressor_sq', 0)
            E.max_ata = ep.get('max_ata', 0);E.ram_recovery = ep.get('ram_recovery', 0);E.boost_ata_ratio = ep.get('boost_ata_ratio', 0)
            E.compressor_zero = ep.get('compressor_zero', 0);E.throttle_boost = ep.get('throttle_boost', 0)
            E.afterburner_boost = ep.get('afterburner_boost', 0);E.reservoir_capacity = ep.get('reservoir_capacity', 0)
            for i in range(3):E.amplitude[i] = ep.get('amplitude', (0., 0., 0.))[i]
            for i in range(4):E.consumption[i] = ep.get('consumption', (0., 0., 0., 0.))[i]
            if len(ep.get('stages', ())) > 8:raise PropUnsupported('compressor stages')
            E.nstages = len(ep.get('stages', ()))
            for i, st in enumerate(ep.get('stages', ())):
                E.stages[i].critical = st['critical'];E.stages[i].power = st['power'];E.stages[i].flat = st['flat']
                E.stages[i].flat_power = st['flat_power'];E.stages[i].curvature = st['curvature']
                E.stages[i].ceiling = st['ceiling'];E.stages[i].ceiling_factor = st['ceiling_factor']
                E.stages[i].boost = st['boost'];E.stages[i].pressure_boost = st['pressure_boost']
            set_table(&E.ata, ep.get('ata', ()), 1)
            E.shaft_min = ep.get('shaft_min', 0);E.turbo_min = ep.get('turbo_min', 0);E.turbo_max = ep.get('turbo_max', 0)
            E.turbo_allowed = ep.get('turbo_allowed', 0);E.inverse_time = ep.get('inverse_time', 0)
            E.direct_thrust = e.get('maximum_direct_thrust', 0.)
            for i in range(3):
                E.axis[i] = (e.get('axis') or (1., 0., 0.))[i];E.position[i] = (e.get('position') or (0., 0., 0.))[i]
            if e['family'] in (2, 5):
                if self.units == NULL:
                    self.units = <Unit*>calloc(NP, sizeof(Unit))
                    if self.units == NULL:raise MemoryError()
                set_turbine(&self.units[k], e)
            if len(ep.get('rpm_targets', ())) > 8:raise PropUnsupported('rpm targets')
            E.ntargets = len(ep.get('rpm_targets', ()))
            for i, (x, y) in enumerate(ep.get('rpm_targets', ())):E.target_x[i] = x;E.target_y[i] = y
        for k, prop in enumerate(p['propellers']):
            pp = prop['properties'];P = &self.props[k]
            # 'iterative' only changes the game's dynamic rotor update: the steady model ignores it as the Python one
            if pp['cyclic'] or pp['differential_pitch'] or pp['active_pitch_2d'] or pp['pitch_1d_count']:
                raise PropUnsupported('propeller geometry')
            P.coaxial = bool(pp['coaxial'])
            sections, area_unit, prepared, _, _, legacy = prepare_blades(pp)
            props_, base, mode, mrows = prepared
            Q = &P.polar
            Q.cl0 = base['cl0'];Q.cd0 = base['cd0'];Q.indCoeff = base['indCoeff'];Q.clLineCoeff = base['clLineCoeff']
            Q.cyCritH = base['cyCritH'];Q.cyCritL = base['cyCritL'];Q.aoaCritH = base['aoaCritH'];Q.aoaCritL = base['aoaCritL']
            Q.aerCenterOffset = base['aerCenterOffset'];Q.clToCm0 = base['clToCm0'];Q.clToCm1 = base['clToCm1']
            Q.parabAngle = base['parabAngle'];Q.declineCoeff = base['declineCoeff'];Q.maxDistAng = base['maxDistAng']
            Q.cdAfterCoeff = base['cdAfterCoeff'];Q.clAfterCritL = base['clAfterCritL'];Q.clAfterCritH = base['clAfterCritH']
            Q.kq = base['kq'];Q.clKq = base['clKq'];Q.cyMult = base['cyMult']
            Q.aoaLineH = base.get('aoaLineH', 0.);Q.aoaLineL = base.get('aoaLineL', 0.)
            Q.parabCyCoeffH = base.get('parabCyCoeffH', 0.);Q.parabCyCoeffL = base.get('parabCyCoeffL', 0.)
            Q.mode = mode;Q.combined = bool(props_.get('CombinedCl', True))
            for i, row in enumerate(mrows):
                for j in range(6):Q.rows[i][j] = row[j]
            Q.legacy = legacy is not None
            if legacy is not None:
                Q.legacy_aoaLineH = legacy['aoaLineH'];Q.legacy_aoaLineL = legacy['aoaLineL']
                Q.legacy_parabH = legacy['parabCyCoeffH'];Q.legacy_parabL = legacy['parabCyCoeffL']
            for i, (r, tw, wd) in enumerate(sections):
                P.station_radius[i] = r;P.twist[i] = tw;P.width[i] = wd
            P.area_unit = area_unit;P.radius = pp['radius'];P.blades = pp['blades']
            P.mean_twist = pp['mean_twist'];P.thrust_deflection = pp['thrust_deflection'];P.max_deflection = pp['max_deflection']
            P.momentum_scale = pp['momentum_scale']
            for i in range(4):
                P.pitch_damping[i] = pp['pitch_damping'][i];P.yaw_damping[i] = pp['yaw_damping'][i]
                P.damping_speed[i] = pp['damping_speed'][i];P.shake[i] = pp['shake'][i]
            P.diameter = pp['diameter'];P.neutral_radius = pp['neutral_radius'];P.inertia = pp['inertia']
            P.pitch_min = pp['pitch_min'];P.pitch_max = pp['pitch_max'];P.governor = pp['governor']
            P.governor_fast = bool(pp['governor_fast']);P.torque_gyro_always = bool(pp['torque_gyro_always'])
            P.min_omega = pp['min_omega'];P.max_omega = pp['max_omega'];P.inv_max_omega = pp['inv_max_omega']
            P.boost_omega = pp['boost_omega'];P.reduction = pp['reduction']
            for i in range(9):P.basis[i] = pp['basis'][i]
            for i in range(3):P.position[i] = pp['position'][i]
            inverse_basis(P.basis, P.inv_basis)
            P.direction = pp['direction']
            if len(pp['airflow']) > 16:raise PropUnsupported('airflow table')
            P.nairflow = len(pp['airflow'])
            for i, row in enumerate(pp['airflow']):
                P.airflow_x[i] = row[0]
                for j in range(3):P.airflow_v[i][j] = row[2+j]
            P.polar_base1 = pp['polar']['base'][1];P.polar_base8 = pp['polar']['base'][8]
        for k in range(NP):self.linked[k] = False
        for k, t in enumerate(p['transmissions']):
            for l in t['engines']:self.linked[l['index']] = True
            T = &self.trans[k]
            if len(t['engines']) > NP or len(t['propellers']) > NP:raise PropUnsupported('shaft links')
            T.ne = len(t['engines']);T.np = len(t['propellers']);T.correct_link = bool(t['correct_link'])
            for i, l in enumerate(t['engines']):
                T.engines[i].index = l['index'];T.engines[i].ratio = l['ratio'];T.engines[i].inverse_ratio = l['inverse_ratio']
            for i, l in enumerate(t['propellers']):
                T.props[i].index = l['index'];T.props[i].ratio = l['ratio'];T.props[i].inverse_ratio = l['inverse_ratio']
        # Seed (prop_equilibrium.solve: native_controls engine_control_mode='automatic'), velocity independent
        native_controls = dict(controls, engine_control_mode='automatic')
        seed = initial_state(p, (100., 0., 0.), self.altitude, nitro=self.nitro, **native_controls)
        for k, es in enumerate(seed['engines']):
            S_ = &self.seed_engines[k]
            S_.throttle = es['throttle'];S_.effective_throttle = es['effective_throttle'];S_.omega = es['omega']
            S_.mechanical = es['mechanical'];S_.mixture = es['mixture'];S_.reservoir = es['reservoir']
            S_.regulator = es['regulator'];S_.turbo = es['turbo'];S_.afterburner = bool(es['afterburner'])
            S_.automatic_mixture = bool(es['automatic_mixture']);S_.automatic_compressor = bool(es['automatic_compressor'])
            S_.gear = es['gear'];S_.friction = 0.;S_.torque = 0.;S_.extra_amplitude = 0.;S_.seed = 12345
            S_.turbo_command = es.get('turbo_command', 1.)
            S_.throttle_ratio = 0.;S_.potential_manifold = 0.;S_.manifold = 0.;S_.consumption = 0.
            for j in range(3):
                S_.force[j] = 0.;S_.moment[j] = 0.
            S_.running = es.get('running', 7)
            if S_.running != 7 and p['engines'][k]['family'] != 3:raise PropUnsupported('engine not running')
        for k, ps in enumerate(seed['propellers']):
            self.seed_props[k].command = ps['command'];self.seed_props[k].auto = bool(ps['auto'])
            self.seed_props[k].pitch = ps['pitch']
            for j in range(3):self.seed_props[k].flow[j] = ps['flow'][j]
            # prop_equilibrium: the coordinate-free wake component starts at zero
            self.seed_props[k].flow[2 if self.props[k].coaxial else 1] = 0.
        for k, ts in enumerate(seed['transmissions']):self.seed_omega[k] = ts['omega']
        self.py_seed = seed;self.py_engines = p['engines'];self.py_props = p['propellers'];self.py_trans = p['transmissions']
        from prop_quasisteady import REVISION as prop_revision
        from prop_equilibrium import EARLY_STOP_ENABLED
        self.revision = prop_revision;self.early_stop = EARLY_STOP_ENABLED
        # Paths, scales, bounds, initial, governed links, shaft targets and torque scales
        paths = [];scales = [];lower = [];upper = [];initial = []
        def variable(path, value, scale, lo, hi):
            paths.append(path);scales.append(scale);lower.append(lo/scale);upper.append(hi/scale);initial.append(value/scale)
        for i, t in enumerate(p['transmissions']):
            links = [l for l in t['propellers'] if p['propellers'][l['index']]['properties']['governor']
                     and (seed['propellers'][l['index']]['auto'] or manual_rpm_governor(p['propellers'][l['index']]['properties']))]
            self.ngoverned[i] = len(links)
            for j, l in enumerate(links):
                self.governed[i][j].index = l['index'];self.governed[i][j].ratio = l['ratio']
                self.governed[i][j].inverse_ratio = l['inverse_ratio']
            from propulsion_general import target_omega as py_target
            target = max((py_target(p['engines'][l['index']]['properties'], seed['engines'][l['index']], self.nitro)
                          * l['inverse_ratio'] for l in t['engines']), default=300.)
            self.shaft_targets[i] = target
            limit = min((p['engines'][l['index']]['properties']['omega_limit']*l['inverse_ratio'] for l in t['engines']),
                        default=max(300., target))
            variable(('shaft', i), target, 300., 1., 2.*limit)
            self.torque_scales[i] = max(1000., sum(abs(p['engines'][l['index']]['properties'].get('torque_base', 1000.)*l['ratio'])
                                                   for l in t['engines']))
            if links:
                variable(('pitch', i), .5, 1., -.25, 1.25)
        for i, prop in enumerate(p['propellers']):
            for j in ((0, 1) if self.props[i].coaxial else (0, 2)):
                variable(('flow', i, j), 0., 20., -150. if j < 2 else 0., 150. if j < 2 else 20.)
        if len(paths) > MAXPATH:raise PropUnsupported('too many propulsion unknowns')
        self.npaths = len(paths);self.paths = paths
        import numpy as np
        lo = np.asarray(lower);hi = np.asarray(upper);ini = np.clip(initial, lo, hi)
        for k, path in enumerate(paths):
            self.path_kind[k] = 0 if path[0] == 'shaft' else 1 if path[0] == 'pitch' else 2
            self.path_i[k] = path[1];self.path_j[k] = path[2] if len(path) > 2 else 0
            self.scales[k] = scales[k];self.lower[k] = lo[k];self.upper[k] = hi[k];self.initial[k] = ini[k]
        self.samples = 0
        self.nslide = 0;self.slide_width = .02
        for k in range(NP):
            self.slide[k] = False;self.slide_coord[k] = -1


    cdef int stage_requests(self, const PropPrep* prep, int s, double* possible) except -1:
        """prop_requests.enumerate_speeds: the union of the shaft's requests over every product of its engines'
        eligible stages (automatic pistons: each authored stage; turbines and rockets: their seed gear)."""
        cdef EngState states[NP]
        cdef double desired[NP]
        cdef int counts[NP]
        cdef int gears[NP]
        cdef int ne = self.trans[s].ne, k, i, k2, nd, idx, npossible = 0
        cdef bint dup
        for k in range(ne):
            idx = self.trans[s].engines[k].index
            counts[k] = (self.engines[idx].nstages if self.engines[idx].nstages > 0 else 1) if self.engines[idx].family <= 1 else 1
            gears[k] = 0
        while True:
            self.seeded(prep, states)
            for k in range(ne):
                idx = self.trans[s].engines[k].index
                if self.engines[idx].family <= 1:states[idx].gear = gears[k]
            self.requests_for(s, states, desired, &nd)
            for i in range(nd):
                dup = False
                for k2 in range(npossible):
                    if possible[k2] == desired[i]:dup = True
                if not dup:
                    if npossible >= MAXCAND:raise PropUnsupported('request candidates')
                    possible[npossible] = desired[i];npossible += 1
            # itertools.product order: the last engine varies fastest
            k = ne-1
            while k >= 0:
                gears[k] += 1
                if gears[k] < counts[k]:break
                gears[k] = 0;k -= 1
            if k < 0:break
        return npossible

    # ------------------------------------------------------------------ turbines and rockets
    cdef inline const Unit* unit(self, int k) noexcept nogil:
        return &self.units[k] if self.units != NULL else NULL

    cdef void seeded(self, const PropPrep* prep, EngState* states) noexcept nogil:
        """The nominal seed engines of this flight condition (prop_equilibrium: turbine speeds at the condition)."""
        cdef int k
        for k in range(self.ne):
            states[k] = self.seed_engines[k]
            if self.engines[k].family == 5:
                states[k].turbo = prep.tseed[k]
            elif self.engines[k].family == 2:
                states[k].omega = prep.tseed[k]

    cdef void turbine_seeds(self, PropPrep* out) noexcept nogil:
        """prop_equilibrium: each turbine seed at the steady scalar target of this condition (velocity, altitude)."""
        cdef int k
        cdef const EngineT* E
        cdef const EngState* es
        cdef double normalized, thrust, torque, consumption, target, next_omega
        cdef bint active
        for k in range(self.ne):
            out.tseed[k] = 0.
            E = &self.engines[k]
            if E.family != 2 and E.family != 5:continue
            es = &self.seed_engines[k]
            normalized = pmin(pdiv(F(es.throttle), F(1.1) if E.throttle_boost > 1. else 1.), 1.)
            scalar_full(&self.units[k].jet, density_at_height(F(self.altitude), F(1.225), F(18300.)), out.velocity[0],
                        self.units[k].jet.max_omega, normalized, self.dt,
                        boost_active(E, es.effective_throttle, es.afterburner, es.gear, self.nitro), 1e6, 1., 1.,
                        &thrust, &torque, &consumption, &target, &next_omega, &active)
            out.tseed[k] = F(pmin(pmax(target, 0.), E.turbo_allowed if E.family == 5 else E.omega_limit))

    # ------------------------------------------------------------------ helpers
    cdef void pitch_limits(self, const PropT* p, const double* velocity, const double* body_omega, double shaft_omega,
                           double* lo_out, double* hi_out) noexcept nogil:
        cdef double lo = p.pitch_min, hi = p.pitch_max, neutral, floor_
        cdef double local[3]
        cdef double local_w[3]
        cdef double tsq, tr
        if not p.governor:
            lo_out[0] = lo; hi_out[0] = lo
            return
        local_flow32(p, velocity, body_omega, local, local_w, &tsq, &tr)
        if fabs(shaft_omega) < 1e-5:
            neutral = M_PI/2
        else:
            neutral = atan2(local[0], p.neutral_radius*shaft_omega)-p.mean_twist-p.polar_base8/p.polar_base1*M_PI/180
        floor_ = neutral-.12
        lo = floor_ if (p.governor_fast and p.governor == 6) else pmax(lo, floor_)
        if p.governor == 8:
            lo = pmax(lo, p.pitch_min+.05235988)
        lo_out[0] = pmin(lo, hi); hi_out[0] = hi

    cdef void requests_for(self, int shaft, const EngState* engines, double* desired, int* ndesired) noexcept nogil:
        """prop_requests.requests / the evaluate() governed-target loop for one shaft."""
        cdef const TransT* t = &self.trans[shaft]
        cdef double target = 0., value, maximum
        cdef bint boost = False, first = True
        cdef int k, idx
        cdef const PropT* pp
        for k in range(t.ne):
            idx = t.engines[k].index
            value = target_omega(&self.engines[idx], &engines[idx], self.nitro)
            if first or value > target:
                target = value
            first = False
            if self.engines[idx].boost_controllable and boost_active(&self.engines[idx], engines[idx].effective_throttle,
                                                                     engines[idx].afterburner, engines[idx].gear, self.nitro):
                boost = True
        if first:
            target = 0.
        ndesired[0] = 0
        for k in range(self.ngoverned[shaft]):
            idx = self.governed[shaft][k].index
            pp = &self.props[idx]
            value = target
            if pp.governor == 1 or pp.governor == 2:
                maximum = pp.boost_omega if boost else pp.max_omega
                if self.seed_props[idx].auto:
                    value = pmin(pmax(value, pp.min_omega), maximum)
                else:
                    value = pp.min_omega+self.seed_props[idx].command*(maximum-pp.min_omega)
            desired[ndesired[0]] = value*pp.reduction/self.governed[shaft][k].ratio
            ndesired[0] += 1

    cdef int prepare(self, double speed, double load, double alpha, double bank, double G, PropPrep* out) except -1:
        """JointProblem.prepared(attitude): flight condition and governed request candidates."""
        cdef double quat[4]
        cdef Air air
        cdef Geometry geom
        cdef double desired[NP]
        cdef double possible[MAXCAND]
        cdef int npossible = 0, nd, i, k, k2, s, idx, code, nc
        cdef EngState states[NP]
        cdef double v, tmp
        cdef bint ok, dup
        turn_quaternion(alpha, bank, quat)
        world_to_body_air(quat, speed, out.velocity)
        air_cache(out.velocity, self.altitude, &air)
        turn_geometry(alpha, bank, speed, load, self.dt_aircraft, air.ias_u, G, &geom)
        for i in range(3):
            out.velocity[i] = F(out.velocity[i]); out.rates[i] = F(geom.omega[i])
        self.turbine_seeds(out)
        for s in range(self.nt):
            out.governed_request[s] = self.ngoverned[s] > 0
            out.ncand[s] = 0
            if not self.ngoverned[s]:
                continue
            npossible = 0
            npossible = self.stage_requests(out, s, possible)
            # sorted(possible)
            for i in range(1, npossible):
                tmp = possible[i]; k = i-1
                while k >= 0 and possible[k] > tmp:
                    possible[k+1] = possible[k]; k -= 1
                possible[k+1] = tmp
            for i in range(npossible):
                if not isfinite(possible[i]):raise ValueError('Nonfinite governed request; source-model calculation unresolved')
            for i in range(npossible):
                self.seeded(out, states)
                for k in range(self.trans[s].ne):
                    idx = self.trans[s].engines[k].index
                    states[idx].omega = possible[i]*self.trans[s].engines[k].ratio
                    code = engine_output(&self.engines[idx], &states[idx], out.velocity[0], self.altitude, self.nitro,
                                         self.unit(idx), self.cg)
                    if code:
                        raise ValueError('native steady engine output unavailable (%d)' % code)
                self.requests_for(s, states, desired, &nd)
                ok = True
                for k in range(nd):
                    if desired[k] != possible[i]:
                        ok = False
                if ok:
                    out.cand[s][out.ncand[s]] = possible[i]; out.ncand[s] += 1
        return 0

    cdef int sample(self, const PropPrep* prep, const double* x, const double* assigned, const bint* has_assigned,
                    PropSample* out) except -1:
        """prop_equilibrium evaluate(x) (stationary, prepared-only): residual per path and aggregate row."""
        cdef PFrame* fr = &out.frame
        cdef int k, i, j, idx, s
        cdef double value, lo, hi, pitch, torque, load_, target, rpm_error, error, density, sound, best
        cdef double desired[NP]
        cdef int nd
        cdef double tf[3]
        cdef double tm[3]
        cdef double th[3]
        cdef double tw
        cdef double ts[2]
        cdef double o_axial, sw
        cdef Air air
        cdef const PropT* pp
        cdef PropState* q
        cdef int direction
        cdef bint gov_ok
        cdef bint has_fraction[NP]
        cdef double swirl[2]
        cdef double err = 0.
        cdef bint finite = True
        self.samples += 1
        self.seeded(prep, fr.engines)
        for k in range(self.ne):
            fr.engines[k].regulator = -1.
        for k in range(self.nprops):
            fr.props[k] = self.seed_props[k]
        for k in range(self.nt):
            fr.trans_omega[k] = self.seed_omega[k]
        for k in range(NP):
            out.fractions[k] = 0.
        for k in range(NP):
            has_fraction[k] = False
        for k in range(self.npaths):
            value = x[k]*self.scales[k]
            i = self.path_i[k]
            if self.path_kind[k] == 0:
                fr.trans_omega[i] = assigned[i] if has_assigned[i] else value
            elif self.path_kind[k] == 1:
                out.fractions[i] = value; has_fraction[i] = True
            else:
                fr.props[i].flow[self.path_j[k]] = value
        for s in range(self.nt):
            for k in range(self.trans[s].ne):
                idx = self.trans[s].engines[k].index
                fr.engines[idx].omega = fr.trans_omega[s]*self.trans[s].engines[k].ratio
            for k in range(self.trans[s].np):
                idx = self.trans[s].props[k].index; pp = &self.props[idx]
                self.pitch_limits(pp, prep.velocity, prep.rates, fr.trans_omega[s]*self.trans[s].props[k].ratio, &lo, &hi)
                pitch = lo+pmin(1., pmax(0., out.fractions[s] if has_fraction[s] else 0.))*(hi-lo)
                if pp.governor and not fr.props[idx].auto and not (pp.governor == 1 or pp.governor == 2):
                    pitch = pp.pitch_max+fr.props[idx].command*(pp.pitch_min-pp.pitch_max)
                fr.props[idx].pitch = pitch
        # prop_quasisteady.frame
        air_cache(prep.velocity, self.altitude, &air)
        density = air.density
        sound = speed_of_sound(self.altitude, F(288.16), F(18300.))
        for i in range(3):
            fr.force[i] = 0.; fr.moment[i] = 0.; fr.momentum[i] = 0.
        fr.wash[0] = 0.; sw = 0.
        swirl[0] = 0.; swirl[1] = 0.
        for s in range(self.nt):
            for i in range(3):
                tf[i] = 0.; tm[i] = 0.; th[i] = 0.
            tw = 0.; ts[0] = 0.; ts[1] = 0.
            for k in range(self.trans[s].np):
                idx = self.trans[s].props[k].index; pp = &self.props[idx]; q = &fr.props[idx]
                propeller(pp, q, prep.velocity, prep.rates, self.cg, fr.trans_omega[s]*self.trans[s].props[k].ratio,
                          density, sound, self.torque_gyro)
                for i in range(3):
                    tf[i] = tf[i]+q.outputs[i]
                for i in range(3):
                    tm[i] = tm[i]+q.outputs[23+i]
                for i in range(3):
                    th[i] = th[i]+q.outputs[26+i]
                for i in range(3):
                    tm[i] = tm[i]+q.outputs[20+i]*(1. if self.trans[s].correct_link else self.trans[s].props[k].ratio)
                o_axial = q.outputs[32]*pp.basis[0]
                if fabs(o_axial) > fabs(tw):
                    tw = o_axial
                direction = 0 if pp.direction == 0 else 1
                ts[direction] = pmax(ts[direction], q.outputs[33]*pp.basis[0])
            for k in range(self.trans[s].ne):
                idx = self.trans[s].engines[k].index
                fr.engines[idx].omega = fr.trans_omega[s]*self.trans[s].engines[k].ratio
                if engine_output(&self.engines[idx], &fr.engines[idx], prep.velocity[0], self.altitude, self.nitro,
                                 self.unit(idx), self.cg):
                    raise ValueError('Steady engine requires nominal deterministic torque')
                for i in range(3):
                    tf[i] = tf[i]+fr.engines[idx].force[i]
                for i in range(3):
                    tm[i] = tm[i]+self.trans[s].engines[k].ratio*fr.engines[idx].moment[i]
            for i in range(3):
                fr.trans_outputs[s][i] = tf[i]; fr.trans_outputs[s][3+i] = tm[i]; fr.trans_outputs[s][6+i] = th[i]
            fr.trans_outputs[s][9] = tw; fr.trans_outputs[s][10] = ts[0]; fr.trans_outputs[s][11] = ts[1]; fr.trans_outputs[s][12] = 0.
            for i in range(3):
                fr.force[i] = fr.force[i]+tf[i]; fr.moment[i] = fr.moment[i]+tm[i]; fr.momentum[i] = fr.momentum[i]+th[i]
            if fabs(tw) > fabs(fr.wash[0]):
                fr.wash[0] = tw
            swirl[0] = pmax(swirl[0], ts[0]); swirl[1] = pmax(swirl[1], ts[1])
        fr.wash[1] = swirl[0]-swirl[1]
        # engines on no shaft (turbojets, rockets): their direct force and moment
        for k in range(self.ne):
            if self.linked[k]:continue
            if engine_output(&self.engines[k], &fr.engines[k], prep.velocity[0], self.altitude, self.nitro, self.unit(k), self.cg):
                raise ValueError('Steady engine requires nominal deterministic torque')
            for i in range(3):
                fr.force[i] = fr.force[i]+fr.engines[k].force[i]; fr.moment[i] = fr.moment[i]+fr.engines[k].moment[i]
        # torques / targets / residual
        for s in range(self.nt):
            torque = 0
            for k in range(self.trans[s].ne):
                idx = self.trans[s].engines[k].index
                torque = torque+fr.engines[idx].torque*self.trans[s].engines[k].ratio
            load_ = 0
            for k in range(self.trans[s].np):
                idx = self.trans[s].props[k].index
                load_ = load_+fr.props[idx].outputs[18]*self.trans[s].props[k].ratio
            out.torques[s] = torque-load_
            self.requests_for(s, fr.engines, desired, &nd)
            for k in range(1, nd):
                if desired[k] != desired[0]:
                    raise ValueError('Conflicting ideal governor targets on a shared shaft')
            if nd == 0:
                out.targets[s] = self.shaft_targets[s]
            else:
                best = desired[0]
                for k in range(1, nd):
                    best = pmax(best, desired[k])
                out.targets[s] = best
        for k in range(self.npaths):
            value = x[k]*self.scales[k]
            i = self.path_i[k]
            if self.path_kind[k] == 0:
                error = out.torques[i]/self.torque_scales[i]
            elif self.path_kind[k] == 1:
                rpm_error = (fr.trans_omega[i]-out.targets[i])/300.
                if self.governor_saturation:
                    error = value-pmin(1., pmax(0., value+rpm_error))
                else:
                    error = rpm_error
            else:
                error = (fr.props[i].equilibrium_flow[self.path_j[k]]-fr.props[i].flow[self.path_j[k]])/self.scales[k]
            out.residual[k] = error
        for i in range(3):
            out.row[i] = fr.force[i]; out.row[3+i] = fr.moment[i]; out.row[6+i] = fr.momentum[i]
        out.row[9] = fr.wash[0]; out.row[10] = fr.wash[1]
        # _result convergence
        for k in range(self.npaths):
            if not isfinite(out.residual[k]):
                finite = False
            err = pmax(err, fabs(out.residual[k]))
        out.converged = finite and err <= 1e-5
        for s in range(self.nt):
            if self.ngoverned[s]:
                if not (out.fractions[s] <= 2e-5 or out.fractions[s] >= 1.-2e-5 or
                        fabs(fr.trans_omega[s]-out.targets[s]) <= 1e-10*pmax(1., fabs(out.targets[s]))):
                    out.converged = False
        for i in range(11):
            if not isfinite(out.row[i]):
                out.converged = False
        return 0

    # ------------------------------------------------------------------ JointProblem.propulsion_at
    cdef int propulsion_at(self, const PropPrep* prep, const double* coords, const int* request_indices, int nrequest,
                           double* inner, PropSample* out) except -1:
        cdef double q[MAXPATH]
        return self.propulsion_q(prep, coords, request_indices, nrequest, inner, out, q)

    cdef int propulsion_q(self, const PropPrep* prep, const double* coords, const int* request_indices, int nrequest,
                          double* inner, PropSample* out, double* q) except -1:
        """coords: the JointProblem propulsion coordinates (x[prop_start:]); returns 1 for BranchUnavailable.
        q: the assembled propulsion-sample coordinates (prop_equilibrium paths)."""
        cdef double assigned[NP]
        cdef bint has_assigned[NP]
        cdef int i, k, j, shaft, index, pitch_idx
        cdef double c, omega, target
        for k in range(self.npaths):
            q[k] = self.initial[k]
        for i in range(NP):
            has_assigned[i] = False; assigned[i] = 0.
        for i in range(self.nt):
            if prep.governed_request[i]:
                if i >= nrequest:
                    return 1
                index = request_indices[i]
                if index < 0 or index >= prep.ncand[i]:
                    return 1
                assigned[i] = prep.cand[i][index]; has_assigned[i] = True
                for k in range(self.npaths):
                    if self.path_kind[k] == 0 and self.path_i[k] == i:
                        q[k] = assigned[i]/300.
        for j in range(self.nactive):
            q[self.active[j]] = coords[j]
        if self.governor_saturation:
            for j in range(self.nactive):
                k = self.active[j]
                if self.path_kind[k] != 1:
                    continue
                shaft = self.path_i[k]; c = coords[j]; target = assigned[shaft]
                omega = target+300.*(c if c < 0. else (c-1. if c > 1. else 0.))
                index = -1
                for i in range(self.npaths):
                    if self.path_kind[i] == 0 and self.path_i[i] == shaft:
                        index = i
                if not (300.*self.lower[index] <= omega and omega <= 300.*self.upper[index]):
                    return 1
                assigned[shaft] = omega; q[index] = omega/300.
                q[k] = pmin(1., pmax(0., c))
        if self.nslide:
            # S8: shafts sliding on a blade-polar jump
            if self.propulsion_slide(prep, coords, out, q, assigned, has_assigned):return 1
        else:
            self.sample(prep, q, assigned, has_assigned, out)
        for j in range(self.nrows):
            inner[j] = out.residual[self.rows[j]]
        return 0

    # ------------------------------------------------------------------ S8: sliding on a blade-polar jump
    cdef int set_slide(self, int n, const bint* flags, const double* c0, const double* frac, const double* hint) except -1:
        """The sliding shafts of the following evaluations (n = 0: none)."""
        cdef int s, j, k
        self.nslide = 0
        for s in range(NP):
            self.slide[s] = False
        if n <= 0:return 0
        for s in range(self.nt):
            if not flags[s]:continue
            k = -1
            for j in range(self.nactive):
                if self.path_i[self.active[j]] == s and (self.path_kind[self.active[j]] == 1 or
                                                         (self.path_kind[self.active[j]] == 0 and not self.ngoverned[s])):
                    k = j
            if k < 0:continue
            self.slide[s] = True;self.slide_coord[s] = k;self.slide_c0[s] = c0[s];self.slide_frac[s] = frac[s]
            self.slide_hint[s] = hint[s];self.nslide += 1
        return 0

    cdef int shaft_coordinate(self, int s) noexcept:
        """The index (in the propulsion coordinates) of shaft ``s``'s own coordinate: its governor coordinate, or its
        speed for an ungoverned shaft; -1 without one."""
        cdef int j, k, out = -1
        for j in range(self.nactive):
            k = self.active[j]
            if self.path_i[k] == s and (self.path_kind[k] == 1 or (self.path_kind[k] == 0 and not self.ngoverned[s])):out = j
        return out

    cdef int use_slide(self, const SlideSpec* sp) except -1:
        if sp == NULL or sp.n <= 0:
            if self.nslide:self.set_slide(0, NULL, NULL, NULL, NULL)
            return 0
        return self.set_slide(sp.n, sp.flag, sp.c0, sp.frac, sp.hint)

    cdef void slide_result(self, SlideSpec* sp) noexcept:
        """theta and the shaft speeds either side of the jump of the last sliding evaluation into ``sp``."""
        cdef int s
        for s in range(NP):
            if self.slide[s]:
                sp.theta[s] = self.slide_theta[s];sp.lo[s] = self.slide_lo[s];sp.hi[s] = self.slide_hi[s]

    cdef int jump_near(self, const PropPrep* prep, const double* coords, const int* request_indices, int nrequest,
                       SlideSpec* out, bint straddle=True) except -1:
        """S8 (the blade-polar jump check): the shafts of a regular state (on a pitch stop or with a fixed pitch) whose
        propellers' blade polar jumps within slide_width of the shaft speed at this flight condition and induced flow
        (and, with ``straddle``, blocks its torque balance there); ``out`` is the sliding spec that starts on the
        state's side of the jump (theta 0 below it, 1 above)."""
        cdef double q[MAXPATH]
        cdef double assigned[NP]
        cdef double lo, hi, c, omega, frac
        cdef int s, j, k, kk, index
        out.n = 0
        for s in range(NP):
            out.flag[s] = False;out.theta[s] = 0.;out.lo[s] = 0.;out.hi[s] = 0.
        if self.nslide:self.set_slide(0, NULL, NULL, NULL, NULL)
        for k in range(self.npaths):
            q[k] = self.initial[k]
        for s in range(NP):assigned[s] = 0.
        for s in range(self.nt):
            if prep.governed_request[s]:
                if s >= nrequest:return 0
                index = request_indices[s]
                if index < 0 or index >= prep.ncand[s]:return 0
                assigned[s] = prep.cand[s][index]
        for j in range(self.nactive):
            q[self.active[j]] = coords[j]
        for s in range(self.nt):
            j = -1
            for kk in range(self.nactive):
                k = self.active[kk]
                if self.path_i[k] == s and (self.path_kind[k] == 1 or (self.path_kind[k] == 0 and not self.ngoverned[s])):j = kk
            if j < 0:continue
            c = coords[j]
            if self.path_kind[self.active[j]] == 1:
                if not self.governor_saturation or (0. < c and c < 1.):continue
                frac = 1. if c >= 1. else 0.
                omega = assigned[s]+300.*(c if c < 0. else c-1.)
                for k in range(self.npaths):
                    if self.path_kind[k] == 1 and self.path_i[k] == s:q[k] = frac
            else:
                frac = 0.;omega = c*300.
            if self.locate(s, prep, q, frac, omega, &lo, &hi):continue
            if straddle and not self.straddles(prep, coords, request_indices, nrequest, s, j, c-omega/300., lo, hi):continue
            out.flag[s] = True;out.frac[s] = frac;out.hint[s] = lo
            out.c0[s] = c-(0. if omega <= lo else 1.);out.cw[s] = c-omega/300.
            if self.s9:
                # S9: theta in [0, 1] is the coordinate minus c0, and the coordinate keeps its own bounds in the
                # correction (a shaft speed's are [min, max] shaft speed / 300): c0 is placed so that [c0, c0+1] lies
                # inside them (an S8 c0 at the shaft speed itself cut theta off well inside [0, 1] on fast shafts); the
                # correction starts the coordinate at c0 plus the side of the jump the state is on (theta here)
                out.theta[s] = 0. if omega <= lo else 1.
                k = self.active[j]
                if self.upper[k]-self.lower[k] > 1.+2e-9:
                    out.c0[s] = pmin(pmax(out.c0[s], self.lower[k]+1e-9), self.upper[k]-1.-1e-9)
            out.n += 1
        return 0

    cdef bint straddles(self, const PropPrep* prep, const double* coords, const int* request_indices, int nrequest, int s,
                        int j, double cw, double lo, double hi) except -1:
        """S8: the jump blocks shaft ``s``'s torque balance here: its net torque just below and just above the jump
        (the other coordinates held) have opposite signs, so no shaft speed balances it (the cause the sliding state
        resolves)."""
        cdef double c[MAXPATH]
        cdef double q[MAXPATH]
        cdef double inner[MAXPATH]
        cdef double t0, t1
        cdef PropSample ps
        cdef int i
        for i in range(self.nactive):c[i] = coords[i]
        c[j] = cw+lo/300.
        if self.propulsion_q(prep, c, request_indices, nrequest, inner, &ps, q):return False
        t0 = ps.torques[s]
        c[j] = cw+hi/300.
        if self.propulsion_q(prep, c, request_indices, nrequest, inner, &ps, q):return False
        t1 = ps.torques[s]
        return (t0 > 0.) != (t1 > 0.)

    cdef unsigned long long signature(self, int s, const PropPrep* prep, const double* q, double frac, double omega) noexcept:
        """The discrete branch of every blade section of the shaft's propellers at shaft speed ``omega``: the side of
        each degenerate Mach row (a jump of the polar at that section Mach) and the lift-curve branch (calc_cl64), the
        inputs of propeller()/blade_forces at this flight condition, pitch fraction and induced flow."""
        cdef unsigned long long code = 1469598103934665603ULL
        cdef int kk, idx, k, r, rotor, nrot
        cdef const PropT* p
        cdef PropState st
        cdef double local[3]
        cdef double local_w[3]
        cdef double tsq, transverse, axial, lo, hi, pitch, om, angular, radius, tangential, phi, alpha, speed, mach, crit
        cdef double sound = speed_of_sound(self.altitude, F(288.16), F(18300.)), sgn
        cdef BPolar polar
        cdef int c
        for kk in range(self.trans[s].np):
            idx = self.trans[s].props[kk].index;p = &self.props[idx]
            st = self.seed_props[idx]
            for k in range(self.npaths):
                if self.path_kind[k] == 2 and self.path_i[k] == idx:st.flow[self.path_j[k]] = q[k]*self.scales[k]
            om = omega*self.trans[s].props[kk].ratio
            self.pitch_limits(p, prep.velocity, prep.rates, om, &lo, &hi)
            pitch = lo+pmin(1., pmax(0., frac))*(hi-lo)
            if p.governor and not st.auto and not (p.governor == 1 or p.governor == 2):
                pitch = p.pitch_max+st.command*(p.pitch_min-p.pitch_max)
            local_flow32(p, prep.velocity, prep.rates, local, local_w, &tsq, &transverse)
            axial = local[0]+.5*(st.flow[0]+st.flow[1])
            nrot = 2 if p.coaxial else 1
            for rotor in range(nrot):
                angular = (.25 if rotor == 0 else -.25)*st.flow[2]
                for k in range(4):
                    radius = p.station_radius[k]
                    tangential = radius*(om-angular)
                    phi = atan2(axial, tangential) if fabs(om) >= 1e-5 else M_PI/2
                    alpha = (pitch-phi+p.twist[k])*RAD_TO_DEG
                    speed = hypot2(tangential+transverse, axial)
                    if speed == 0.:continue
                    mach = speed/sound
                    # only the branches whose change is a jump of the polar: the side of a degenerate Mach row
                    # (mach_factor: low below its knot, high above it, low != high) and, where the lift line has
                    # degenerated to a point (aoaLineL == aoaLineH), the side of calc_cl64's +0.01 switch
                    if p.polar.mode == 3:
                        for r in range(7):
                            if p.polar.rows[r][1] == p.polar.rows[r][2] and p.polar.rows[r][0] != p.polar.rows[r][3]:
                                c = 0 if mach < p.polar.rows[r][1] else (1 if mach == p.polar.rows[r][1] else 2)
                                code = (code ^ <unsigned long long>(c+1))*1099511628211ULL
                    evaluate_polar(&p.polar, mach, &polar)
                    if polar.aoaLineL == polar.aoaLineH:
                        c = 1 if alpha-polar.aoaLineH+0.01 >= 0 else 2
                        code = (code ^ <unsigned long long>(c+16))*1099511628211ULL
                    elif self.s9:
                        code = cl_jump_sides(&polar, alpha, code)
        return code

    cdef list debug_stations(self, int s, const PropPrep* prep, const double* q, double frac, double omega):
        """Diagnostics (S9): the blade sections of shaft ``s`` at shaft speed ``omega`` (signature()'s inputs and the
        polar there): (prop, rotor, station, mach, alpha, aoaLineL, aoaLineH, aoaCritL, aoaCritH, cl, cd)."""
        cdef int kk, idx, k, rotor, nrot
        cdef const PropT* p
        cdef PropState st
        cdef double local[3]
        cdef double local_w[3]
        cdef double tsq, transverse, axial, lo, hi, pitch, om, angular, radius, tangential, phi, alpha, speed, mach
        cdef double sound = speed_of_sound(self.altitude, F(288.16), F(18300.))
        cdef BPolar polar
        out = []
        for kk in range(self.trans[s].np):
            idx = self.trans[s].props[kk].index;p = &self.props[idx]
            st = self.seed_props[idx]
            for k in range(self.npaths):
                if self.path_kind[k] == 2 and self.path_i[k] == idx:st.flow[self.path_j[k]] = q[k]*self.scales[k]
            om = omega*self.trans[s].props[kk].ratio
            self.pitch_limits(p, prep.velocity, prep.rates, om, &lo, &hi)
            pitch = lo+pmin(1., pmax(0., frac))*(hi-lo)
            if p.governor and not st.auto and not (p.governor == 1 or p.governor == 2):
                pitch = p.pitch_max+st.command*(p.pitch_min-p.pitch_max)
            local_flow32(p, prep.velocity, prep.rates, local, local_w, &tsq, &transverse)
            axial = local[0]+.5*(st.flow[0]+st.flow[1])
            nrot = 2 if p.coaxial else 1
            for rotor in range(nrot):
                angular = (.25 if rotor == 0 else -.25)*st.flow[2]
                for k in range(4):
                    radius = p.station_radius[k]
                    tangential = radius*(om-angular)
                    phi = atan2(axial, tangential) if fabs(om) >= 1e-5 else M_PI/2
                    alpha = (pitch-phi+p.twist[k])*RAD_TO_DEG
                    speed = hypot2(tangential+transverse, axial)
                    mach = speed/sound
                    evaluate_polar(&p.polar, mach, &polar)
                    out.append((idx, rotor, k, mach, alpha, polar.aoaLineL, polar.aoaLineH, polar.aoaCritL, polar.aoaCritH,
                                calc_cl64(&polar, alpha), calc_cd64(&polar, alpha), pitch*RAD_TO_DEG, lo*RAD_TO_DEG, hi*RAD_TO_DEG))
        return out

    cdef int locate(self, int s, const PropPrep* prep, const double* q, double frac, double hint, double* lo_out,
                    double* hi_out) noexcept:
        """The blade-polar jump of shaft ``s`` nearest ``hint`` (within slide_width of it): (lo, hi) bracketing the change
        of signature() to 1e-11 relative; 1 where none."""
        cdef unsigned long long s0 = self.signature(s, prep, q, frac, hint), sa, sb
        cdef double step = hint*self.slide_width/16., a, b, m
        cdef int k, side, it
        cdef bint found = False
        a = b = hint
        for k in range(1, 17):
            for side in range(2):
                m = hint+(k*step if side == 0 else -k*step)
                if self.signature(s, prep, q, frac, m) != s0:
                    if side == 0:
                        a = hint+(k-1)*step;b = m
                    else:
                        a = m;b = hint-(k-1)*step
                    found = True;break
            if found:break
        if not found:return 1
        sa = self.signature(s, prep, q, frac, a)
        for it in range(80):
            if b-a <= 1e-11*fabs(hint):break
            m = .5*(a+b)
            if self.signature(s, prep, q, frac, m) == sa:a = m
            else:b = m
        lo_out[0] = a;hi_out[0] = b
        return 0

    cdef int propulsion_slide(self, const PropPrep* prep, const double* coords, PropSample* out, double* q,
                              double* assigned, bint* has_assigned) except -1:
        """S8: the propulsion sample with the sliding shafts on their jump: the shaft speed at the jump (located at this
        flight condition and induced flow), theta from the shaft's coordinate; every output of a sliding shaft (its
        propellers' forces, moments, angular momentum, wash, induced-flow equilibria and shaft torque) is the mix
        (1-theta) low side + theta high side; the aggregate is rebuilt from the shafts. 1 where a jump is not found."""
        cdef PropSample hs
        cdef double qlo[MAXPATH]
        cdef double qhi[MAXPATH]
        cdef double alo[NP]
        cdef double ahi[NP]
        cdef bint has[NP]
        cdef double theta[NP]
        cdef double lo, hi, t, error = 0., tw, ts0, ts1
        cdef int s, k, kk, idx, i, j, direction
        cdef bint finite = True
        cdef PFrame* fl = &out.frame
        cdef PFrame* fh = &hs.frame
        for k in range(self.npaths):
            qlo[k] = q[k];qhi[k] = q[k]
        for s in range(NP):
            alo[s] = assigned[s];ahi[s] = assigned[s];has[s] = has_assigned[s]
        for s in range(self.nt):
            if not self.slide[s]:continue
            theta[s] = coords[self.slide_coord[s]]-self.slide_c0[s]
            for k in range(self.npaths):
                if self.path_kind[k] == 1 and self.path_i[k] == s:
                    qlo[k] = self.slide_frac[s];qhi[k] = self.slide_frac[s];q[k] = self.slide_frac[s]
            if self.locate(s, prep, qlo, self.slide_frac[s], self.slide_hint[s], &lo, &hi):return 1
            self.slide_lo[s] = lo;self.slide_hi[s] = hi;self.slide_theta[s] = theta[s]
            alo[s] = lo;ahi[s] = hi;has[s] = True
            for k in range(self.npaths):
                if self.path_kind[k] == 0 and self.path_i[k] == s:
                    qlo[k] = lo/300.;qhi[k] = hi/300.;q[k] = lo/300.
        self.sample(prep, qlo, alo, has, out)
        self.sample(prep, qhi, ahi, has, &hs)
        for s in range(self.nt):
            if not self.slide[s]:continue
            t = theta[s]
            for i in range(13):
                fl.trans_outputs[s][i] = (1.-t)*fl.trans_outputs[s][i]+t*fh.trans_outputs[s][i]
            for kk in range(self.trans[s].np):
                idx = self.trans[s].props[kk].index
                for i in range(40):
                    fl.props[idx].outputs[i] = (1.-t)*fl.props[idx].outputs[i]+t*fh.props[idx].outputs[i]
                for i in range(3):
                    fl.props[idx].equilibrium_flow[i] = (1.-t)*fl.props[idx].equilibrium_flow[i]+t*fh.props[idx].equilibrium_flow[i]
            out.torques[s] = (1.-t)*out.torques[s]+t*hs.torques[s]
        # the aggregate (prop_quasisteady.frame) from the shafts, plus the engines on no shaft
        for i in range(3):
            fl.force[i] = 0.;fl.moment[i] = 0.;fl.momentum[i] = 0.
        fl.wash[0] = 0.;ts0 = 0.;ts1 = 0.
        for s in range(self.nt):
            for i in range(3):
                fl.force[i] += fl.trans_outputs[s][i];fl.moment[i] += fl.trans_outputs[s][3+i];fl.momentum[i] += fl.trans_outputs[s][6+i]
            tw = fl.trans_outputs[s][9]
            if fabs(tw) > fabs(fl.wash[0]):fl.wash[0] = tw
            ts0 = pmax(ts0, fl.trans_outputs[s][10]);ts1 = pmax(ts1, fl.trans_outputs[s][11])
        fl.wash[1] = ts0-ts1
        for k in range(self.ne):
            if self.linked[k]:continue
            for i in range(3):
                fl.force[i] += fl.engines[k].force[i];fl.moment[i] += fl.engines[k].moment[i]
        # the residuals of the sliding shafts' paths
        for k in range(self.npaths):
            s = self.path_i[k]
            if self.path_kind[k] == 0 and self.slide[s]:
                out.residual[k] = out.torques[s]/self.torque_scales[s]
            elif self.path_kind[k] == 2:
                idx = s
                out.residual[k] = (fl.props[idx].equilibrium_flow[self.path_j[k]]-fl.props[idx].flow[self.path_j[k]])/self.scales[k]
        for i in range(3):
            out.row[i] = fl.force[i];out.row[3+i] = fl.moment[i];out.row[6+i] = fl.momentum[i]
        out.row[9] = fl.wash[0];out.row[10] = fl.wash[1]
        for k in range(self.npaths):
            if not isfinite(out.residual[k]):finite = False
            error = pmax(error, fabs(out.residual[k]))
        out.converged = finite and error <= 1e-5
        for i in range(11):
            if not isfinite(out.row[i]):out.converged = False
        return 0

    def test_slide(self, double speed, double load, double alpha, double bank, double G, double dt, coords, request_indices,
                   shafts, c0, frac, hint):
        """Diagnostics: (inner residual, row, theta, lo, hi) of a sliding propulsion sample, or None."""
        cdef PropPrep prep
        cdef PropSample out
        cdef double c[MAXPATH]
        cdef double q[MAXPATH]
        cdef int r[NP]
        cdef double inner[MAXPATH]
        cdef bint f[NP]
        cdef double a[NP]
        cdef double b[NP]
        cdef double h[NP]
        self.dt_aircraft = dt
        self.prepare(speed, load, alpha, bank, G, &prep)
        for i in range(len(coords)):c[i] = float(coords[i])
        for i in range(len(request_indices)):r[i] = int(request_indices[i])
        for i in range(NP):
            f[i] = i in shafts;a[i] = c0[i] if i < len(c0) else 0.;b[i] = frac[i] if i < len(frac) else 0.
            h[i] = hint[i] if i < len(hint) else 0.
        self.set_slide(len(shafts), f, a, b, h)
        try:
            code = self.propulsion_q(&prep, c, r, len(request_indices), inner, &out, q)
        finally:
            self.set_slide(0, f, a, b, h)
        if code:return None
        return ([inner[i] for i in range(self.nrows)], [out.row[i] for i in range(11)],
                [self.slide_theta[i] for i in shafts], [self.slide_lo[i] for i in shafts], [self.slide_hi[i] for i in shafts])

    def test_signature(self, double speed, double load, double alpha, double bank, double G, double dt, coords, int shaft,
                       double frac, omegas):
        """Diagnostics: signature() of a shaft at the given shaft speeds."""
        cdef PropPrep prep
        cdef double q[MAXPATH]
        cdef int i
        self.dt_aircraft = dt
        self.prepare(speed, load, alpha, bank, G, &prep)
        for i in range(self.npaths):q[i] = self.initial[i]
        for j in range(self.nactive):q[self.active[j]] = float(coords[j])
        return [self.signature(shaft, &prep, q, frac, float(w)) for w in omegas]

    cdef dict engine_dict(self, int k, const EngState* es):
        # prop_quasisteady.engine_output(engine, dict(state, omega=...)) on the nominal seed state
        d = dict(self.py_seed['engines'][k])
        d['regulator'] = -1.
        d['omega'] = es.omega
        d['extra_amplitude'] = 0.
        family = self.engines[k].family
        d.update(torque=es.torque, friction=0., regulator=es.regulator, gear=es.gear, throttle_ratio=es.throttle_ratio,
                 potential_manifold=es.potential_manifold, manifold=es.manifold, mechanical=es.mechanical, seed=int(es.seed),
                 extra_amplitude=es.extra_amplitude, consumption=es.consumption, effective_throttle=es.effective_throttle,
                 force=[es.force[0], es.force[1], es.force[2]], moment=[es.moment[0], es.moment[1], es.moment[2]],
                 running=es.running)
        if family == 5 or family == 3 or self.engines[k].compressor_type == 3:
            d.update(turbo=es.turbo, turbo_command=es.turbo_command)
        return d

    cdef object record(self, const PropSample* ps, const double* q):
        """The baseline propulsion record of one stationary sample: prop_equilibrium
        _result(...) plus the prepared sampler's stationary update (pitch-stop governor)."""
        import math
        import numpy as np
        cdef const PFrame* fr = &ps.frame
        cdef const PropState* st
        cdef int k, i, s
        cdef double error = 0.
        cdef bint converged
        if not self.governor_saturation:
            raise PropUnsupported('exact-RPM governor record')
        engines = [self.engine_dict(k, &fr.engines[k]) for k in range(self.ne)]
        props = []
        for k in range(self.nprops):
            st = &fr.props[k]
            d = dict(self.py_seed['propellers'][k])
            d.update(pitch=st.pitch, governor_pitch=st.pitch, flow=[st.flow[0], st.flow[1], st.flow[2]],
                     equilibrium_flow=[st.equilibrium_flow[0], st.equilibrium_flow[1], st.equilibrium_flow[2]],
                     outputs=[st.outputs[i] for i in range(40)])
            props.append(d)
        trans = [dict(omega=fr.trans_omega[s], previous_omega=fr.trans_omega[s],
                      outputs=[fr.trans_outputs[s][i] for i in range(13)]) for s in range(self.nt)]
        per_engine = [[0., 0., 0.] for k in range(self.ne)]
        for s in range(self.nt):
            for k in range(self.trans[s].ne):
                i = self.trans[s].engines[k].index
                per_engine[i] = [per_engine[i][0]+fr.trans_outputs[s][0], per_engine[i][1]+fr.trans_outputs[s][1],
                                 per_engine[i][2]+fr.trans_outputs[s][2]]
        r = dict(transmissions=trans, engines=engines, propellers=props, seed=self.py_seed['seed'],
                 aggregate_force=[fr.force[0], fr.force[1], fr.force[2]],
                 aggregate_moment=[fr.moment[0], fr.moment[1], fr.moment[2]],
                 engine_angular_momentum=[fr.momentum[0], fr.momentum[1], fr.momentum[2]],
                 engine_wash=[fr.wash[0], fr.wash[1]], per_engine_force=per_engine)
        residual = np.asarray([ps.residual[k] for k in range(self.npaths)])
        error = float(np.max(np.abs(residual), initial=0.))
        converged = bool(np.isfinite(residual).all() and error <= 1e-5)
        rpm_errors = [fr.trans_omega[s]-ps.targets[s] for s in range(self.nt)]
        for s in range(self.nt):
            if self.ngoverned[s] and not (ps.fractions[s] <= 2e-5 or ps.fractions[s] >= 1.-2e-5
                                          or abs(rpm_errors[s]) <= 1e-10*max(1., abs(ps.targets[s]))):
                converged = False
        row = r['aggregate_force']+r['aggregate_moment']+r['engine_angular_momentum']+r['engine_wash']
        converged = converged and all(math.isfinite(v) for v in row)
        stopped = [s for s in range(self.nt) if fr.trans_omega[s] <= 1.]
        status = []
        for s in range(self.nt):
            if not self.ngoverned[s]:
                status.append('manual blade pitch' if any(self.props[self.trans[s].props[k].index].governor
                                                          for k in range(self.trans[s].np)) else 'fixed pitch')
            else:
                status.append('minimum pitch' if ps.fractions[s] <= 2e-5 else
                              'maximum pitch' if ps.fractions[s] >= 1-2e-5 else 'target RPM')
        diagnostics = dict(method='quasi-steady ideal governor equilibrium', model_revision=self.revision,
            force_model='continuous steady blade polar; normalized Hermite Mach curves', residual=error,
            residual_early_stop_enabled=self.early_stop, residual_early_stops=0,
            frame_evaluations=1, shaft_torque_residual_nm=[ps.torques[s] for s in range(self.nt)],
            shaft_target_rpm=[ps.targets[s]*60/(2*math.pi) for s in range(self.nt)],
            shaft_rpm_error=[v*60/(2*math.pi) for v in rpm_errors],
            induced_flow_residual_mps=[[a-b for a, b in zip(q_['equilibrium_flow'], q_['flow'])] for q_ in props],
            governor_status=status)
        result = dict(state=r, force=row[:3], moment=row[3:6], angular_momentum=row[6:9], wash=row[9:],
            converged=converged, feasible=converged and not stopped, stopped_shafts=stopped,
            overspeed_engines=[k for k, e in enumerate(engines)
                               if e['omega'] > self.py_engines[k]['properties']['omega_limit']*(1+1e-6)],
            period_frames=None, cycle_samples=[row] if converged else None, window_relative_change=None,
            simulated_seconds=0., controls=self.py_controls, canonical_initialization=True, stationarity=diagnostics)
        result['stationarity'].update(method='stationary bounded governor torque/wake equations',
            governor_model='pitch_stops', equilibrium_coverage_complete=False, governor_selection_certified=False)
        result['_equilibrium_coordinates'] = np.asarray([q[k] for k in range(self.npaths)])
        return result

    cdef object request_records(self, const PropPrep* prep):
        """prop_requests.enumerate_speeds for every shaft (the JointProblem prepared requests)."""
        cdef EngState states[NP]
        cdef double desired[NP]
        cdef double possible[MAXCAND]
        cdef int npossible, nd, i, k, k2, s, idx, code
        cdef bint dup, ok
        out = []
        for s in range(self.nt):
            if not self.ngoverned[s]:
                out.append(dict(governed=False, candidates=[], excluded=[], request_coverage_complete=True))
                continue
            npossible = self.stage_requests(prep, s, possible)
            values = sorted([possible[i] for i in range(npossible)])
            candidates = [];excluded = []
            for speed in values:
                self.seeded(prep, states)
                engines = {}
                for k in range(self.trans[s].ne):
                    i = self.trans[s].engines[k].index
                    states[i].omega = speed*self.trans[s].engines[k].ratio
                    code = engine_output(&self.engines[i], &states[i], prep.velocity[0], self.altitude, self.nitro,
                                         self.unit(i), self.cg)
                    if code:raise ValueError('native steady engine output unavailable (%d)' % code)
                    engines[i] = self.engine_dict(i, &states[i])
                self.requests_for(s, states, desired, &nd)
                actual = [desired[i] for i in range(nd)]
                record = dict(shaft_speed=speed, actual_requests=actual,
                              engine_gears=[states[self.trans[s].engines[k].index].gear for k in range(self.trans[s].ne)],
                              engines=engines)
                ok = True
                for i in range(nd):
                    if desired[i] != speed:ok = False
                if ok:candidates.append(record)
                else:excluded.append(record)
            out.append(dict(governed=True, candidates=candidates, excluded=excluded, request_coverage_complete=True,
                            equilibrium_coverage_complete=False))
        return out

    def test_record(self, double speed, double load, double alpha, double bank, double G, double dt, coords, request_indices):
        """(record, request records, velocity, rates) for a JointProblem state (verification)."""
        cdef PropPrep prep
        cdef PropSample out
        cdef double c[MAXPATH]
        cdef double q[MAXPATH]
        cdef int r[NP]
        cdef double inner[MAXPATH]
        self.dt_aircraft = dt
        self.prepare(speed, load, alpha, bank, G, &prep)
        for i in range(len(coords)):c[i] = float(coords[i])
        for i in range(len(request_indices)):r[i] = int(request_indices[i])
        code = self.propulsion_q(&prep, c, r, len(request_indices), inner, &out, q)
        if code:return None
        return (self.record(&out, q), self.request_records(&prep), tuple(prep.velocity[i] for i in range(3)),
                tuple(prep.rates[i] for i in range(3)))

    def set_joint_layout(self, active, rows):
        self.nactive = len(active);self.nrows = len(rows)
        for i, a in enumerate(active):self.active[i] = a
        for i, r in enumerate(rows):self.rows[i] = r

    def bench(self, double speed, double load, double alpha, double bank, double G, double dt, coords, request_indices,
              int repeats=200):
        """(seconds per prepare, seconds per propulsion sample) at one flight condition."""
        import time
        cdef PropPrep prep
        cdef PropSample out
        cdef double c[MAXPATH]
        cdef int r[NP]
        cdef double inner[MAXPATH]
        cdef int k, i
        self.dt_aircraft = dt
        for i in range(len(coords)):c[i] = float(coords[i])
        for i in range(len(request_indices)):r[i] = int(request_indices[i])
        t0 = time.perf_counter()
        for k in range(repeats):
            self.prepare(speed, load, alpha, bank, G, &prep)
        t1 = time.perf_counter()
        for k in range(repeats):
            c[0] += 1e-9
            self.propulsion_at(&prep, c, r, len(request_indices), inner, &out)
        t2 = time.perf_counter()
        return (t1-t0)/repeats, (t2-t1)/repeats

    def test_sample(self, double speed, double load, double alpha, double bank, double G, double dt, coords, request_indices):
        """(inner residual, row, converged, candidates) for a JointProblem state (verification)."""
        cdef PropPrep prep
        cdef PropSample out
        cdef double c[MAXPATH]
        cdef int r[NP]
        cdef double inner[MAXPATH]
        self.dt_aircraft = dt
        self.prepare(speed, load, alpha, bank, G, &prep)
        for i in range(len(coords)):c[i] = float(coords[i])
        for i in range(len(request_indices)):r[i] = int(request_indices[i])
        code = self.propulsion_at(&prep, c, r, len(request_indices), inner, &out)
        if code:return None
        return ([inner[i] for i in range(self.nrows)], [out.row[i] for i in range(11)], bool(out.converged),
                [[prep.cand[s][k] for k in range(prep.ncand[s])] for s in range(self.nt)])


cdef void set_turbine(Unit* u, dict e) except *:
    """A turbine engine's scalar table and nozzles (the jet engine Unit layout, set_unit)."""
    j = e['turbine']
    if len(j['density']) > 16 or len(j['speed']) > 32:raise PropUnsupported('turbine table too large')
    if len(e.get('nozzles') or ()) > 16:raise PropUnsupported('too many turbine nozzles')
    u.rocket = False
    u.jet.nd = len(j['density']);u.jet.ns = len(j['speed'])
    for i, d in enumerate(j['density']):u.jet.density[i] = d
    for i, s in enumerate(j['speed']):u.jet.speed[i] = s
    if not j['cells']:raise PropUnsupported('turbine table without cells')
    for i, row in enumerate(j['cells']):
        for k, cell in enumerate(row):
            for q in range(4):u.jet.cells[i][k][q] = cell[q]
    for q in range(4):u.jet.bases[q] = j['bases'][q]
    set_table(&u.jet.modes, j['modes'], 5);set_table(&u.jet.throttle, j['throttle'], 1)
    u.jet.throttle_scale = j['throttle_scale'];u.jet.max_omega = j['max_omega'];u.jet.consumption = j['consumption']
    u.jet.tau = j['tau']
    nozzles = e.get('nozzles') or ()
    u.nnoz = len(nozzles)
    for k, nz in enumerate(nozzles):set_nozzle(&u.noz[k], nz)
