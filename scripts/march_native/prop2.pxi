# PropModel: the stationary propulsion block of JointProblem for supported
# propeller aircraft (prepare / enumerate_speeds / sample / propulsion_at).

from libc.stdlib cimport calloc, free as pfree
from cpython.bytes cimport PyBytes_FromStringAndSize

DEF MAXPATH = 40
DEF AX_RESCUE = 2000   # most points a path starts from rest after its states were continued (axial_path's last pass)
DEF AXC = 8        # most request combinations (one request per shaft each) a row point keeps states of
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


cdef int cl_jump_points(const BPolar* p, double* out) noexcept nogil:
    """The breakpoints of calc_cl64 where the lift jumps (cl_jump_sides' own list and test), into ``out`` (12)."""
    cdef double pts[12]
    cdef double b, d, lo, hi, mx = pmax(40., p.maxDistAng)
    cdef int k, n = 0, m = 0
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
            out[m] = b;m += 1
    return m


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


DEF S16_ENGINE_SLIDE = .15   # S16: relative shaft-speed reach of the supercharger-edge search (locate)


class AxMore(Exception):
    """A path point has more request combinations than the path's arrays hold."""


cdef class AxStore:
    """The states of a solved path (PropModel.axial_path with full False): `entry(i)` gives point i as the path's
    full result would (`ax_entry`, without the states of other request combinations), by evaluating the model once
    more at the point's solved coordinates."""
    cdef object model
    cdef int count, n
    cdef double G, dt
    cdef char* kind
    cdef double* Q
    cdef long long* B
    cdef double* ALT
    cdef double* SPD

    def __dealloc__(self):
        pfree(self.kind);pfree(self.Q);pfree(self.B);pfree(self.ALT);pfree(self.SPD)

    def entry(self, int i):
        return self.model.path_entry(self, i)


cdef class PropModel:
    cdef int ne, nprops, nt
    cdef EngineT engines[NP]
    cdef PropT props[NP]
    cdef TransT trans[NP]
    cdef EngState seed_engines[NP]
    cdef PropState seed_props[NP]
    cdef double seed_omega[NP]
    cdef double altitude, nitro, dt, turn
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
    # Engine performance page (`axial`): the Newton settings of the propulsion coordinates (set_axial)
    cdef int ax_n, ax_nblocks, ax_colors
    cdef double ax_lo[MAXPATH]
    cdef double ax_hi[MAXPATH]
    cdef double ax_step[MAXPATH]
    cdef double ax_limit[MAXPATH]
    cdef double ax_scale[MAXPATH]
    cdef int ax_size[MAXPATH]
    cdef int ax_block[MAXPATH][MAXPATH]
    # `axial_row`: over-rev limits, each propeller's own-axis share of the flight speed, the starts
    cdef double ax_wlimit[NP]
    cdef double ax_inv0[NP]
    cdef double ax_initial[MAXPATH]
    cdef double ax_alt[4][MAXPATH]
    cdef double ax_fine[MAXPATH]
    cdef int ax_nalt
    cdef bint ax_has_fine, ax_row_ready
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
    # overrev follow-up: the jump check (straddle) takes the nearest jump within slide_width that blocks the torque
    # balance, not only the nearest jump (two jumps 1.3% apart: Bf 109 F-4 RB at 65% pitch, 478.5 km/h near 2.7 g, the
    # nearer at 3,432 rpm does not block, the one at 3,477 rpm does)
    cdef public bint blocking
    # Governed sliding (Leo, 2026-11-21): a shaft whose governor is holding its target speed (pitch between the stops)
    # can have no torque balance either, where a blade section sits on a jump of its polar at that speed: the shaft
    # torque steps through zero for a hair of pitch (Do 17 Z-2 at 3,950 m, 200-259 km/h: +141 / -202 N m). The game's
    # 48 Hz step (governor and shaft, recovered code) then flips the shaft speed across the jump every frame by a
    # revolution or so a minute, the pitch holds still, and thrust alternates between the two sides; its mean is their
    # mix that balances the shaft to 0.7%. So the same sliding state, with the jump located along the pitch fraction at
    # the shaft's target speed instead of along shaft speed at a held pitch. Such a shaft's spec has frac = -1, and
    # its hint, lo and hi are 300 x the pitch fraction at the jump (cw = 0), so cw + lo/300 is its regular coordinate.
    cdef public bint pitch_slide
    cdef double slide_last[NP]      # governed sliding: where the jump was at the last evaluation (the next search starts there)
    # the last sliding evaluation (for records): theta and the shaft speeds either side of the jump
    cdef double slide_theta[NP]
    cdef double slide_lo[NP]
    cdef double slide_hi[NP]
    # S9: theta's coordinate range kept inside the shaft coordinate's bounds (set by MarchEngine; False: S8)
    cdef public bint s9
    # S16: a shaft's engine supercharger with no valid stage on one side of a shaft speed is a jump too (signature())
    cdef public bint s16

    def __cinit__(self, *args, **kwargs):
        self.units = NULL
        self.s9 = False
        self.s16 = False

    def __dealloc__(self):
        if self.units != NULL:pfree(self.units)

    def __init__(self, solver):
        cdef EngineT* E
        cdef PropT* P
        cdef BPolar* Q
        cdef EngState* S_
        cdef TransT* T
        from prop_steady import initial_state
        from em_manual_controls import governs, manual_rpm_governor
        from prop_quasisteady import prepare_blades
        from polar_model import mach_curve
        from component_assembly import f32
        from overrev import TORQUE as OVERREV_TORQUE
        engine = solver.engine
        p = engine.properties
        cfg = solver.config
        if len(p['engines']) > NP or len(p['propellers']) > NP or len(p['transmissions']) > NP:
            raise PropUnsupported('too many engines, propellers or shafts')
        self.ne = len(p['engines']);self.nprops = len(p['propellers']);self.nt = len(p['transmissions'])
        self.altitude = cfg['altitude_m'];self.nitro = solver.mass['nitro_mass'];self.dt = engine.dt
        for i in range(3):self.cg[i] = solver.mass['cog'][i]
        self.torque_gyro = bool(cfg['torque_gyro'])
        self.turn = -1. if cfg.get('turn_direction', 'left') == 'right' else 1.
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
            E.overrev_torque = OVERREV_TORQUE and e['family'] in (0, 1)
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
        # Lit rocket boosters (controls['boosters']) run from the seed on (rocket_boosters.seed_state)
        from rocket_boosters import seed_state as booster_seed_state
        seed = booster_seed_state(p, (100., 0., 0.), self.altitude, self.nitro, controls)
        for k, es in enumerate(seed['engines']):
            S_ = &self.seed_engines[k]
            S_.throttle = es['throttle'];S_.effective_throttle = es['effective_throttle'];S_.omega = es['omega']
            S_.mechanical = es['mechanical'];S_.mixture = es['mixture'];S_.reservoir = es['reservoir']
            S_.regulator = es['regulator'];S_.turbo = es['turbo'];S_.afterburner = bool(es['afterburner'])
            S_.automatic_mixture = bool(es['automatic_mixture']);S_.automatic_compressor = bool(es['automatic_compressor'])
            S_.gear = es['gear'];S_.friction = 0.;S_.torque = 0.;S_.extra_amplitude = 0.;S_.seed = 12345
            S_.turbo_command = es.get('turbo_command', 1.)
            S_.throttle_ratio = 0.;S_.potential_manifold = 0.;S_.manifold = 0.;S_.consumption = 0.
            S_.regime = 0;S_.fuel_regime = 0
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
            links = [l for l in t['propellers'] if governs(p['propellers'][l['index']]['properties'])
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
        self.nslide = 0;self.slide_width = .02;self.blocking = False;self.pitch_slide = False
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
        turn_quaternion(alpha, self.turn*bank, quat)
        world_to_body_air(quat, speed, out.velocity)
        air_cache(out.velocity, self.altitude, &air)
        turn_geometry(alpha, bank, speed, load, self.dt_aircraft, air.ias_u, G, self.turn, &geom)
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
                if self.nslide and self.slide[shaft] and self.slide_frac[shaft] < 0.:
                    # governed sliding: the shaft holds its target speed, and its coordinate carries theta
                    q[k] = 0.
                    continue
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
            self.slide_last[s] = hint[s]/300.
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
                # governed sliding: the jump's pitch moves with the flight condition, so the state keeps where it is now
                if self.slide_frac[s] < 0.:sp.hint[s] = self.slide_lo[s]

    cdef int jump_near(self, const PropPrep* prep, const double* coords, const int* request_indices, int nrequest,
                       SlideSpec* out, bint straddle=True, double wide=0.) except -1:
        """S8 (the blade-polar jump check): the shafts of a regular state (on a pitch stop or with a fixed pitch) whose
        propellers' blade polar jumps within slide_width of the shaft speed at this flight condition and induced flow
        (and, with ``straddle``, blocks its torque balance there); ``out`` is the sliding spec that starts on the
        state's side of the jump (theta 0 below it, 1 above). With ``wide`` (a fraction of the shaft speed): the nearest
        jump within that range that blocks the torque balance (locate_blocking), for a correction from a neighbour whose
        shaft speed lies farther from the jump than slide_width."""
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
                if self.governor_saturation and 0. < c and c < 1.:
                    # a governing shaft: the jump along its pitch fraction at its target speed that blocks its torque
                    # balance (governed sliding)
                    if not self.pitch_slide or not straddle or wide > 0.:continue
                    if self.locate_pitch(s, prep, q, assigned[s], c, PITCH_WIDE, PITCH_STEPS, coords, request_indices, nrequest, j,
                                         &lo, &hi):continue
                    out.flag[s] = True;out.frac[s] = -1.;out.hint[s] = 300.*lo;out.cw[s] = 0.
                    out.lo[s] = 300.*lo;out.hi[s] = 300.*hi
                    out.theta[s] = 0. if c <= lo else 1.
                    out.c0[s] = c-out.theta[s]
                    k = self.active[j]
                    if self.upper[k]-self.lower[k] > 1.+2e-9:
                        out.c0[s] = pmin(pmax(out.c0[s], self.lower[k]+1e-9), self.upper[k]-1.-1e-9)
                    out.n += 1
                    continue
                if not self.governor_saturation or (0. < c and c < 1.):continue
                frac = 1. if c >= 1. else 0.
                omega = assigned[s]+300.*(c if c < 0. else c-1.)
                for k in range(self.npaths):
                    if self.path_kind[k] == 1 and self.path_i[k] == s:q[k] = frac
            else:
                frac = 0.;omega = c*300.
            if wide > 0.:
                if self.locate_blocking(s, prep, q, frac, omega, wide, coords, request_indices, nrequest, j, c-omega/300.,
                                        &lo, &hi):continue
            elif straddle and self.blocking:
                if self.locate_blocking(s, prep, q, frac, omega, self.slide_width, coords, request_indices, nrequest, j,
                                        c-omega/300., &lo, &hi):continue
            else:
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

    cdef int locate_pitch(self, int s, const PropPrep* prep, const double* q, double omega, double hint, double wide,
                          int steps, const double* coords, const int* request_indices, int nrequest, int j,
                          double* lo_out, double* hi_out) except -1:
        """Governed sliding: the blade-polar jump of shaft ``s`` along its pitch fraction nearest ``hint`` (within
        ``wide`` of it, inside the stops) at shaft speed ``omega``: (lo, hi) bracketing the change of signature() to
        1e-11; with ``j`` >= 0 (the shaft's coordinate in ``coords``) only one that blocks its torque balance
        (straddles). 1 where none."""
        cdef double step = wide/steps, a, b, m, prev
        cdef unsigned long long sm, sa
        cdef int k, side, it
        cdef double last[2]
        cdef unsigned long long sig[2]
        sig[0] = sig[1] = self.signature(s, prep, q, hint, omega);last[0] = last[1] = hint
        for k in range(1, steps+1):
            for side in range(2):
                m = hint+(k*step if side == 0 else -k*step)
                if m <= 0. or m >= 1.:continue
                sm = self.signature(s, prep, q, m, omega)
                if sm != sig[side]:
                    a = last[side] if side == 0 else m
                    b = m if side == 0 else last[side]
                    sa = self.signature(s, prep, q, a, omega)
                    for it in range(80):
                        if b-a <= 1e-11:break
                        prev = .5*(a+b)
                        if self.signature(s, prep, q, prev, omega) == sa:a = prev
                        else:b = prev
                    if j < 0 or self.straddles(prep, coords, request_indices, nrequest, s, j, 0., 300.*a, 300.*b):
                        lo_out[0] = a;hi_out[0] = b
                        return 0
                    sig[side] = sm
                last[side] = m
        return 1

    cdef int pitch_jump(self, int s, const PropPrep* prep, const double* q, double omega, double hint, double wide,
                        double* lo_out, double* hi_out) noexcept:
        """Governed sliding, for the evaluations of a sliding state: locate_pitch's nearest jump worked out instead of
        searched for. A blade section's angle of attack is linear in the pitch fraction (pitch - inflow angle + twist),
        so each breakpoint where its lift jumps is met at one fraction; the nearest of them to ``hint`` within ``wide``,
        bracketed by 1e-10 either side and confirmed by signature(). 1 where there is none (the caller then searches)."""
        cdef int kk, idx, k, rotor, nrot, i, n
        cdef const PropT* p
        cdef PropState st
        cdef double local[3]
        cdef double local_w[3]
        cdef double points[12]
        cdef double tsq, transverse, axial, lo, hi, om, angular, radius, tangential, phi, speed, mach, f, best = -1.
        cdef double sound = speed_of_sound(self.altitude, F(288.16), F(18300.))
        cdef BPolar polar
        if not self.s9:return 1
        for kk in range(self.trans[s].np):
            idx = self.trans[s].props[kk].index;p = &self.props[idx]
            st = self.seed_props[idx]
            for k in range(self.npaths):
                if self.path_kind[k] == 2 and self.path_i[k] == idx:st.flow[self.path_j[k]] = q[k]*self.scales[k]
            if p.governor and not st.auto and not (p.governor == 1 or p.governor == 2):continue
            om = omega*self.trans[s].props[kk].ratio
            self.pitch_limits(p, prep.velocity, prep.rates, om, &lo, &hi)
            if not hi-lo > 1e-12:continue
            local_flow32(p, prep.velocity, prep.rates, local, local_w, &tsq, &transverse)
            axial = local[0]+.5*(st.flow[0]+st.flow[1])
            nrot = 2 if p.coaxial else 1
            for rotor in range(nrot):
                angular = (.25 if rotor == 0 else -.25)*st.flow[2]
                for k in range(4):
                    radius = p.station_radius[k]
                    tangential = radius*(om-angular)
                    phi = atan2(axial, tangential) if fabs(om) >= 1e-5 else M_PI/2
                    speed = hypot2(tangential+transverse, axial)
                    if speed == 0.:continue
                    mach = speed/sound
                    evaluate_polar(&p.polar, mach, &polar)
                    if polar.aoaLineL == polar.aoaLineH:
                        points[0] = polar.aoaLineH-0.01;n = 1
                    else:
                        n = cl_jump_points(&polar, points)
                    for i in range(n):
                        f = (points[i]/RAD_TO_DEG+phi-p.twist[k]-lo)/(hi-lo)
                        if f > 0. and f < 1. and fabs(f-hint) <= wide and (best < 0. or fabs(f-hint) < fabs(best-hint)):
                            best = f
        if best < 0.:return 1
        lo = best-1e-10;hi = best+1e-10
        if self.signature(s, prep, q, lo, omega) == self.signature(s, prep, q, hi, omega):return 1
        lo_out[0] = lo;hi_out[0] = hi
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
        if self.s16:code = self.engine_signature(s, prep, omega, code)
        return code

    cdef unsigned long long engine_signature(self, int s, const PropPrep* prep, double omega,
                                             unsigned long long code) noexcept:
        """S16 (Leo 2026-10-24): ``code`` hashed with the supercharger branch of each piston engine of shaft ``s`` at
        shaft speed ``omega``: a stage selected, or none (no stage scores above the starting -1 and the factor falls back
        to -1, as in aces.exe 0x142fa2180). Engine torque jumps there, so a shaft on a pitch stop whose engine passes that
        edge with shaft speed can have no torque balance (Ki-43-I RB at 40%, 300-460 km/h: net torque +5,435 N m at 1,520
        rpm, -422 at 1,540), and it slides on it like on a blade-polar jump (the game's shaft speed chatters there)."""
        cdef const EngineT* E
        cdef const EngState* es
        cdef double mult, reg, ratio, pot, man
        cdef int gear, sel, kk, idx
        for kk in range(self.trans[s].ne):
            idx = self.trans[s].engines[kk].index
            E = &self.engines[idx]
            if (E.family != 0 and E.family != 1) or (E.compressor_type != 1 and E.compressor_type != 2):continue
            es = &self.seed_engines[idx]
            sel = 0
            if compressor_step(E, omega*self.trans[s].engines[kk].ratio, pmin(F(es.throttle), F(1.1)),
                               inlet_pressure(self.altitude, prep.velocity[0], E.ram_recovery), 0.,
                               es.gear if (E.manual_compressor and not es.automatic_compressor) else -1, es.gear, -1.,
                               es.afterburner, self.nitro, self.altitude, &mult, &gear, &reg, &ratio, &pot, &man, &sel):
                continue
            code = (code ^ <unsigned long long>(48 if sel < 0 else 49))*1099511628211ULL
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
        if not found and self.s16:
            # S16: a supercharger edge (engine_signature) is looked for up to S16_ENGINE_SLIDE of the shaft speed: the
            # engine torque jump is a whole stage factor (+1 to -1), so a seed state well off it is still on its slope
            # (Ki-43-I RB at 40%: from 1,553 rpm at 460 km/h, the edge at 450 km/h lies 3% lower)
            s0 = self.engine_signature(s, prep, hint, 0)
            step = hint*S16_ENGINE_SLIDE/32.
            for k in range(1, 33):
                for side in range(2):
                    m = hint+(k*step if side == 0 else -k*step)
                    if self.engine_signature(s, prep, m, 0) != s0:
                        if side == 0:
                            a = hint+(k-1)*step;b = m
                        else:
                            a = m;b = hint-(k-1)*step
                        found = True;break
                if found:break
            if not found:return 1
            sa = self.engine_signature(s, prep, a, 0)
            for it in range(80):
                if b-a <= 1e-11*fabs(hint):break
                m = .5*(a+b)
                if self.engine_signature(s, prep, m, 0) == sa:a = m
                else:b = m
            lo_out[0] = a;hi_out[0] = b
            return 0
        if not found:return 1
        sa = self.signature(s, prep, q, frac, a)
        for it in range(80):
            if b-a <= 1e-11*fabs(hint):break
            m = .5*(a+b)
            if self.signature(s, prep, q, frac, m) == sa:a = m
            else:b = m
        lo_out[0] = a;hi_out[0] = b
        return 0

    cdef int locate_blocking(self, int s, const PropPrep* prep, const double* q, double frac, double hint, double wide,
                             const double* coords, const int* request_indices, int nrequest, int j, double cw,
                             double* lo_out, double* hi_out) except -1:
        """The blade-polar jump of shaft ``s`` nearest ``hint`` within ``wide`` x hint of it that blocks its torque balance
        (straddles): the changes of signature() in SLIDE_WIDE_STEPS steps each side, nearest first, each bracketed to 1e-11
        relative as locate does; 1 where none blocks."""
        cdef double step = hint*wide/SLIDE_WIDE_STEPS, a, b, m, prev
        cdef unsigned long long sp, sm, sa
        cdef int k, side, it
        cdef double last[2]
        cdef unsigned long long sig[2]
        if not (hint > 0.):return 1
        sig[0] = sig[1] = self.signature(s, prep, q, frac, hint);last[0] = last[1] = hint
        for k in range(1, SLIDE_WIDE_STEPS+1):
            for side in range(2):
                m = hint+(k*step if side == 0 else -k*step)
                sm = self.signature(s, prep, q, frac, m)
                if sm != sig[side]:
                    a = last[side] if side == 0 else m
                    b = m if side == 0 else last[side]
                    sa = self.signature(s, prep, q, frac, a)
                    for it in range(80):
                        if b-a <= 1e-11*fabs(hint):break
                        prev = .5*(a+b)
                        if self.signature(s, prep, q, frac, prev) == sa:a = prev
                        else:b = prev
                    if self.straddles(prep, coords, request_indices, nrequest, s, j, cw, a, b):
                        lo_out[0] = a;hi_out[0] = b
                        return 0
                    sig[side] = sm
                last[side] = m
        return 1

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
            if self.slide_frac[s] < 0.:
                # governed sliding: the pitch fraction either side of the jump, at the shaft's target speed
                if self.pitch_jump(s, prep, qlo, assigned[s], self.slide_last[s], PITCH_WIDE, &lo, &hi):
                    if self.locate_pitch(s, prep, qlo, assigned[s], self.slide_last[s], PITCH_WIDE, PITCH_STEPS, NULL, NULL, 0, -1,
                                         &lo, &hi):return 1
                self.slide_last[s] = lo
                self.slide_lo[s] = 300.*lo;self.slide_hi[s] = 300.*hi;self.slide_theta[s] = theta[s]
                for k in range(self.npaths):
                    if self.path_kind[k] == 1 and self.path_i[k] == s:
                        qlo[k] = lo;qhi[k] = hi;q[k] = lo
                continue
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
                                                          and self.props[self.trans[s].props[k].index].pitch_max
                                                          > self.props[self.trans[s].props[k].index].pitch_min
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

    def test_regime(self, double speed, double load, double alpha, double bank, double G, double dt, coords, request_indices):
        """([(the branches of its law, of its fuel law) per engine], [its blade sections' polar branches per
        propeller]) at a JointProblem state (EngState.regime, PropState.branches), or None."""
        cdef PropPrep prep
        cdef PropSample out
        cdef double c[MAXPATH]
        cdef int r[NP]
        cdef double inner[MAXPATH]
        self.dt_aircraft = dt
        self.prepare(speed, load, alpha, bank, G, &prep)
        for i in range(len(coords)):c[i] = float(coords[i])
        for i in range(len(request_indices)):r[i] = int(request_indices[i])
        if self.propulsion_at(&prep, c, r, len(request_indices), inner, &out):return None
        return ([(out.frame.engines[k].regime, out.frame.engines[k].fuel_regime) for k in range(self.ne)],
                [out.frame.props[k].branches for k in range(self.nprops)])

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

    # ------------------------------------------------------------------ Engine performance page: propeller points at alpha 0
    def set_altitude(self, double altitude):
        """Another altitude for the same aircraft and settings. Nothing else this model keeps depends on it: the engines'
        start states are the automatic-control ones (prop_steady.initial_state with mixture 1), the same at every
        altitude; air, inlet pressure and the turbines' tables are read at self.altitude in every sample."""
        self.altitude = altitude

    def set_axial(self, lower, upper, steps, limits, scales, blocks):
        """The settings of `axial` (engine_performance._prop_fast_setup): bounds, difference steps, step limits and
        residual scales of the propulsion coordinates (JointProblem's, from prop_start on), and the coordinates of
        each shaft (blocks; one block: every coordinate is differenced on its own)."""
        cdef int i, b, c
        self.ax_n = len(lower)
        for i in range(self.ax_n):
            self.ax_lo[i] = lower[i];self.ax_hi[i] = upper[i];self.ax_step[i] = steps[i]
            self.ax_limit[i] = limits[i];self.ax_scale[i] = scales[i]
        self.ax_nblocks = len(blocks);self.ax_colors = 0
        for b in range(self.ax_nblocks):
            self.ax_size[b] = len(blocks[b])
            if self.ax_size[b] > self.ax_colors:self.ax_colors = self.ax_size[b]
            for c in range(self.ax_size[b]):self.ax_block[b][c] = blocks[b][c]

    def candidates(self, double speed, double G, double dt):
        """The governed request candidates of each shaft at alpha 0 and 1 g (test_sample's last item, without a sample)."""
        cdef PropPrep prep
        self.dt_aircraft = dt
        self.prepare(speed, 1., 0., 0., G, &prep)
        return [[prep.cand[s][k] for k in range(prep.ncand[s])] for s in range(self.nt)]

    cdef object axial_result(self, const double* q, const double* r, const double* J, bint have_jac, const PropSample* ps):
        """(coordinates, residual, Jacobian or None, summary of the solved state): the summary holds what `record`
        gives the engine page: (converged as record decides it, force, governor status, per engine (omega, torque,
        consumption, gear, regulator, potential manifold, direct force x, manifold, the branches of its law, of its fuel
        law), per propeller (pitch, flow 0, 1, 2, equilibrium flow 0, its blade sections' polar branches, its fastest
        section's Mach number), shaft speeds, each shaft's propeller thrust, each shaft's target speed)."""
        cdef double thrust
        cdef int idx
        cdef const PFrame* fr = &ps.frame
        cdef int n = self.ax_n, i, j, k, s
        cdef double error = 0.
        cdef bint converged = True, manual
        for k in range(self.npaths):
            if not isfinite(ps.residual[k]):converged = False
            elif fabs(ps.residual[k]) > error:error = fabs(ps.residual[k])
        if error > 1e-5:converged = False
        status = []
        for s in range(self.nt):
            if not self.ngoverned[s]:
                manual = False
                for k in range(self.trans[s].np):
                    if (self.props[self.trans[s].props[k].index].governor
                            and self.props[self.trans[s].props[k].index].pitch_max > self.props[self.trans[s].props[k].index].pitch_min):manual = True
                status.append('manual blade pitch' if manual else 'fixed pitch')
            else:
                if not (ps.fractions[s] <= 2e-5 or ps.fractions[s] >= 1.-2e-5
                        or fabs(fr.trans_omega[s]-ps.targets[s]) <= 1e-10*pmax(1., fabs(ps.targets[s]))):converged = False
                status.append('minimum pitch' if ps.fractions[s] <= 2e-5 else
                              'maximum pitch' if ps.fractions[s] >= 1-2e-5 else 'target RPM')
        for i in range(3):
            if not (isfinite(fr.force[i]) and isfinite(fr.moment[i]) and isfinite(fr.momentum[i])):converged = False
        if not (isfinite(fr.wash[0]) and isfinite(fr.wash[1])):converged = False
        shafts = []
        for s in range(self.nt):
            thrust = 0.
            for k in range(self.trans[s].np):
                idx = self.trans[s].props[k].index
                for i in range(3):thrust += fr.props[idx].outputs[i]*self.props[idx].basis[i]
            shafts.append(thrust)
        summary = (bool(converged), (fr.force[0], fr.force[1], fr.force[2]), status,
                   [(fr.engines[k].omega, fr.engines[k].torque, fr.engines[k].consumption, fr.engines[k].gear,
                     fr.engines[k].regulator, fr.engines[k].potential_manifold, fr.engines[k].force[0], fr.engines[k].manifold,
                     fr.engines[k].regime, fr.engines[k].fuel_regime)
                    for k in range(self.ne)],
                   [(fr.props[k].pitch, fr.props[k].flow[0], fr.props[k].flow[1], fr.props[k].flow[2],
                     fr.props[k].equilibrium_flow[0], fr.props[k].branches, fr.props[k].tip_mach) for k in range(self.nprops)],
                   [fr.trans_omega[s] for s in range(self.nt)], shafts, [ps.targets[s] for s in range(self.nt)])
        return ([q[i] for i in range(n)], [r[i] for i in range(n)],
                [[J[i*n+j] for j in range(n)] for i in range(n)] if have_jac else None, summary)

    cdef int axial_core(self, const PropPrep* prep, const int* req, int nreq, double* q, double* J, bint* have_jac_io,
                        double* r, PropSample* out) except -1:
        """The Newton loop of `axial` on a prepared flight condition. q: the start, then the solved coordinates; J and
        have_jac_io: a Jacobian to start with (or none), then the one to go on with; r, out: the solved residual and
        sample. Returns 0 with a closed state, 1 without one."""
        cdef double qp[MAXPATH]
        cdef double qn[MAXPATH]
        cdef double rv[MAXPATH]
        cdef double dq[MAXPATH]
        cdef double step[MAXPATH]
        cdef double delta[MAXPATH]
        cdef double M[MAXPATH*(MAXPATH+1)]
        cdef int n = self.ax_n, w = self.ax_n+1, i, j, k, b, c, color, it, ls, pivot, row
        cdef bint have_jac = have_jac_io[0], fresh = False, accepted, ok, closed, singular
        cdef double norm0, norm1, alpha, ratio, v, denom, e, d, f, h, total
        for i in range(n):q[i] = pmin(pmax(q[i], self.ax_lo[i]), self.ax_hi[i])
        if self.propulsion_at(prep, q, req, nreq, r, out):return 1
        closed = out.converged
        norm0 = 0.
        for i in range(n):
            if fabs(r[i]) > .1*self.ax_scale[i]:closed = False
            norm0 += (r[i]/self.ax_scale[i])*(r[i]/self.ax_scale[i])
        norm0 = sqrt(norm0)
        if closed:
            have_jac_io[0] = have_jac
            return 0
        for it in range(8):
            if not have_jac:
                for i in range(n*n):J[i] = 0.
                ok = True
                if self.ax_nblocks > 1:
                    for color in range(self.ax_colors):
                        for i in range(n):qp[i] = q[i]
                        for b in range(self.ax_nblocks):
                            if color < self.ax_size[b]:
                                c = self.ax_block[b][color]
                                h = self.ax_step[c] if q[c]+self.ax_step[c] <= self.ax_hi[c] else -self.ax_step[c]
                                qp[c] += h;delta[c] = h
                        if self.propulsion_at(prep, qp, req, nreq, rv, out):
                            ok = False
                            break
                        for b in range(self.ax_nblocks):
                            if color < self.ax_size[b]:
                                c = self.ax_block[b][color]
                                for k in range(self.ax_size[b]):
                                    row = self.ax_block[b][k]
                                    J[row*n+c] = (rv[row]-r[row])/delta[c]
                else:
                    for j in range(n):
                        h = self.ax_step[j] if q[j]+self.ax_step[j] <= self.ax_hi[j] else -self.ax_step[j]
                        for i in range(n):qp[i] = q[i]
                        qp[j] += h
                        if self.propulsion_at(prep, qp, req, nreq, rv, out):
                            ok = False
                            break
                        for row in range(n):J[row*n+j] = (rv[row]-r[row])/h
                if not ok:break
                have_jac = True;fresh = True
            # J dq = -r: elimination with row pivoting
            for i in range(n):
                for j in range(n):M[i*w+j] = J[i*n+j]
                M[i*w+n] = -r[i]
            singular = False
            for c in range(n):
                pivot = c
                for row in range(c+1, n):
                    if fabs(M[row*w+c]) > fabs(M[pivot*w+c]):pivot = row
                if M[pivot*w+c] == 0.:
                    singular = True
                    break
                if pivot != c:
                    for k in range(w):
                        e = M[c*w+k];M[c*w+k] = M[pivot*w+k];M[pivot*w+k] = e
                d = M[c*w+c]
                for row in range(c+1, n):
                    f = M[row*w+c]/d
                    if f != 0.:
                        for k in range(c, w):M[row*w+k] -= f*M[c*w+k]
            if singular:
                if not fresh:
                    have_jac = False
                    continue
                break
            for c in range(n-1, -1, -1):
                total = M[c*w+n]
                for k in range(c+1, n):total -= M[c*w+k]*dq[k]
                dq[c] = total/M[c*w+c]
            ratio = 0.
            for i in range(n):
                if fabs(dq[i])/self.ax_limit[i] > ratio:ratio = fabs(dq[i])/self.ax_limit[i]
            if ratio > 1.:
                for i in range(n):dq[i] = dq[i]/ratio
            accepted = False;alpha = 1.
            for ls in range(3):
                for i in range(n):qn[i] = pmin(pmax(q[i]+alpha*dq[i], self.ax_lo[i]), self.ax_hi[i])
                if not self.propulsion_at(prep, qn, req, nreq, rv, out):
                    closed = out.converged
                    norm1 = 0.;denom = 0.
                    for i in range(n):
                        if fabs(rv[i]) > .1*self.ax_scale[i]:closed = False
                        norm1 += (rv[i]/self.ax_scale[i])*(rv[i]/self.ax_scale[i])
                        step[i] = qn[i]-q[i];denom += step[i]*step[i]
                    norm1 = sqrt(norm1)
                    if closed or norm1 < (.98 if fresh else .75)*norm0:
                        if denom > 1e-24 and (closed or (alpha == 1. and norm1 < .65*norm0)):
                            for i in range(n):
                                e = rv[i]-r[i]
                                for j in range(n):e -= J[i*n+j]*step[j]
                                e = e/denom
                                for j in range(n):J[i*n+j] += e*step[j]
                        elif not closed:have_jac = False
                        if closed:
                            for i in range(n):
                                q[i] = qn[i];r[i] = rv[i]
                            have_jac_io[0] = have_jac
                            return 0
                        for i in range(n):
                            q[i] = qn[i];r[i] = rv[i]
                        norm0 = norm1;fresh = False;accepted = True
                        break
                alpha *= .5
            if not accepted:
                if not fresh:
                    have_jac = False
                    continue
                break
        if self.propulsion_at(prep, q, req, nreq, r, out):return 1
        if not out.converged:return 1
        for i in range(n):
            if fabs(r[i]) > self.ax_scale[i]:return 1
        have_jac_io[0] = have_jac
        return 0



    def axial(self, double speed, double G, double dt, coords, request_indices, jacobian):
        """engine_performance._newton_prop on native samples: Newton on the propulsion equations at alpha 0 and 1 g
        from `coords`, the Jacobian by differences (one sample per coordinate of a shaft, the shafts together) and kept
        between steps and points by Broyden's update (jacobian: the one to start with, or None). Returns None (no
        state on this request branch, or no convergence), False (another number of equations than coordinates: the
        Python loop solves those), or `axial_result`."""
        cdef PropPrep prep
        cdef PropSample out
        cdef double q[MAXPATH]
        cdef double r[MAXPATH]
        cdef double J[MAXPATH*MAXPATH]
        cdef int req[NP]
        cdef int n = self.ax_n, nreq, i, j
        cdef bint have_jac = False
        if self.nrows != self.nactive or n != self.nactive or n <= 0:return False
        self.dt_aircraft = dt
        self.prepare(speed, 1., 0., 0., G, &prep)
        nreq = len(request_indices)
        for i in range(nreq):req[i] = int(request_indices[i])
        for i in range(n):q[i] = float(coords[i])
        if jacobian is not None and len(jacobian) == n and len(jacobian[0]) == n:
            for i in range(n):
                for j in range(n):J[i*n+j] = float(jacobian[i][j])
            have_jac = True
        if self.axial_core(&prep, req, nreq, q, J, &have_jac, r, &out):return None
        return self.axial_result(q, r, J, have_jac, &out)

    # ------------------------------------------------------------------ a whole row of propeller points (one altitude)
    def set_axial_row(self, omega_limits, inv0, initial, alternates, fine_seed):
        """The settings of `axial_row` beside `set_axial`'s: each engine's over-rev limit (rad/s), each propeller's
        share of the flight speed along its own axis, the coordinates a solve starts from at rest, the alternate
        starts (up to 4) and the fine-pitch start (or None)."""
        cdef int i, a
        for i in range(self.ne):self.ax_wlimit[i] = omega_limits[i]
        for i in range(self.nprops):self.ax_inv0[i] = inv0[i]
        for i in range(self.ax_n):self.ax_initial[i] = initial[i]
        self.ax_nalt = min(len(alternates), 4)
        for a in range(self.ax_nalt):
            for i in range(self.ax_n):self.ax_alt[a][i] = alternates[a][i]
        self.ax_has_fine = fine_seed is not None
        if self.ax_has_fine:
            for i in range(self.ax_n):self.ax_fine[i] = fine_seed[i]
        self.ax_row_ready = True

    cdef bint ax_converged(self, const PropSample* ps) noexcept:
        """`axial_result`'s 'converged' (the record's)."""
        cdef const PFrame* fr = &ps.frame
        cdef double error = 0.
        cdef int k, s, i
        for k in range(self.npaths):
            if not isfinite(ps.residual[k]):return False
            elif fabs(ps.residual[k]) > error:error = fabs(ps.residual[k])
        if error > 1e-5:return False
        for s in range(self.nt):
            if self.ngoverned[s]:
                if not (ps.fractions[s] <= 2e-5 or ps.fractions[s] >= 1.-2e-5
                        or fabs(fr.trans_omega[s]-ps.targets[s]) <= 1e-10*pmax(1., fabs(ps.targets[s]))):return False
        for i in range(3):
            if not (isfinite(fr.force[i]) and isfinite(fr.moment[i]) and isfinite(fr.momentum[i])):return False
        if not (isfinite(fr.wash[0]) and isfinite(fr.wash[1])):return False
        return True

    cdef bint ax_overspeed(self, const PropSample* ps) noexcept:
        cdef int k
        for k in range(self.ne):
            if ps.frame.engines[k].omega > self.ax_wlimit[k]*(1+1e-6):return True
        return False

    cdef bint ax_flows(self, double v, const PropSample* ps) noexcept:
        """engine_performance._summary_fields' tests of each propeller's flow (prop_admission's margins)."""
        cdef const PFrame* fr = &ps.frame
        cdef const PropT* p
        cdef double vf = F(v), local0, upstream, axial, omega_p, t, air
        cdef int s, k, idx, i, cell
        cdef bint has_width
        for s in range(self.nt):
            for k in range(self.trans[s].np):
                idx = self.trans[s].props[k].index
                p = &self.props[idx]
                local0 = F(vf*self.ax_inv0[idx])
                if local0 <= p.airflow_x[0]:air = p.airflow_v[0][0]
                else:
                    cell = -1
                    for i in range(p.nairflow-1):
                        if local0 <= p.airflow_x[i+1]:
                            cell = i
                            break
                    if cell < 0:air = p.airflow_v[p.nairflow-1][0]
                    else:
                        t = (local0-p.airflow_x[cell])/(p.airflow_x[cell+1]-p.airflow_x[cell])
                        air = p.airflow_v[cell][0]+(p.airflow_v[cell+1][0]-p.airflow_v[cell][0])*t
                upstream = local0+air
                axial = local0+.5*(fr.props[idx].flow[0]+fr.props[idx].flow[1])
                if not (isfinite(upstream) and isfinite(axial) and upstream > 0. and axial > 0.):return False
                if p.coaxial and not (upstream+.5*fr.props[idx].equilibrium_flow[0] > 0.):return False
                omega_p = fr.trans_omega[s]*self.trans[s].props[k].ratio
                has_width = False
                for i in range(4):
                    if p.width[i] > 0. and p.station_radius[i] > 0.:has_width = True
                if has_width:
                    if omega_p-.25*fr.props[idx].flow[2] <= 0.:return False
                    if p.coaxial and omega_p+.25*fr.props[idx].flow[2] <= 0.:return False
        return True

    cdef bint ax_status_any(self, const PropSample* ps, int what) noexcept:
        """Whether a governed shaft is on its minimum pitch stop (what 1) or holds its target RPM (what 3)."""
        cdef int s
        for s in range(self.nt):
            if not self.ngoverned[s]:continue
            if ps.fractions[s] <= 2e-5:
                if what == 1:return True
            elif ps.fractions[s] >= 1-2e-5:pass
            elif what == 3:return True
        return False

    cdef int ax_solve(self, const PropPrep* prep, double v, const double* wQ, const double* wJ, const char* wH,
                      const long long* wB, int wn, bint cold, const double* seed, double* oQ, double* oJ, char* oH,
                      long long* oB, int* on, int* chosen, double* r, PropSample* ps, int cap=AXC) except -1:
        """engine_performance._solve_prop_fast. A shaft may have several governor requests; every combination of one
        request per shaft (at most AXC) is solved, from the neighbour's state of the same combination (wQ, wJ, wH, wB:
        wn states, each with its combination packed in wB) or, where it has none or `cold`, from rest (`seed`, default
        the layout's). The state of the smallest residual is the point's; then the tests of the Python function.
        Returns 0 with a result later points can start from (oQ, oJ, oH, oB: `on` states, `chosen` the point's; r,
        ps: its residual and sample), 3 with a fine-stop result (r, ps and state 0 of oQ; not started from), 1
        without a result, 9 with more than AXC combinations (the caller's Python loop solves those rows), 8 with
        more than `cap` (what the caller's arrays hold) but no more than AXC."""
        cdef int req[NP]
        cdef int digits[NP]
        cdef int n = self.ax_n, nreq = self.nt, i, s, c, total = 1, k, w, best = -1
        cdef bint got, used_warm = False, conv, over, valid, needs_fine, hj
        cdef double q[MAXPATH]
        cdef double rr[MAXPATH]
        cdef double J[MAXPATH*MAXPATH]
        cdef PropSample ps2
        cdef double force, norm, best_norm = 0.
        cdef long long code
        for s in range(self.nt):
            digits[s] = prep.ncand[s] if prep.ncand[s] > 1 else 1
            if digits[s] > 8:return 9
            total *= digits[s]
            if total > AXC:return 9
        if total > cap:return 8
        on[0] = 0
        for c in range(total):
            # combination c: the last shaft's request changes fastest (itertools.product)
            k = c;code = 0
            for s in range(self.nt-1, -1, -1):
                req[s] = k % digits[s];k //= digits[s]
            for s in range(self.nt):code = code*8+req[s]
            w = -1
            for i in range(wn):
                if wB[i] == code:
                    w = i
                    break
            got = False
            if w >= 0:
                for i in range(n):q[i] = wQ[w*n+i]
                hj = wH[w]
                if hj:
                    for i in range(n*n):J[i] = wJ[w*n*n+i]
                if not self.axial_core(prep, req, nreq, q, J, &hj, rr, &ps2):
                    got = True;used_warm = True
            if not got and (cold or w < 0):
                for i in range(n):q[i] = seed[i] if seed != NULL else self.ax_initial[i]
                hj = False
                if not self.axial_core(prep, req, nreq, q, J, &hj, rr, &ps2):got = True
            if not got:continue
            k = on[0];on[0] = k+1
            for i in range(n):oQ[k*n+i] = q[i]
            oH[k] = hj;oB[k] = code
            if hj:
                for i in range(n*n):oJ[k*n*n+i] = J[i]
            norm = 0.
            for i in range(n):norm += rr[i]*rr[i]
            norm = sqrt(norm)
            if best < 0 or norm < best_norm:
                best = k;best_norm = norm
                for i in range(n):r[i] = rr[i]
                ps[0] = ps2
        if best < 0:
            on[0] = 0
            return 3 if not self.ax_fine_state(prep, v, oQ, r, ps) else 1
        chosen[0] = best
        conv = self.ax_converged(ps);force = ps.frame.force[0]
        if conv and force >= 0.:
            if self.ax_overspeed(ps) or not self.ax_flows(v, ps):return 1
            return 0
        k = oB[best]
        for s in range(self.nt-1, -1, -1):
            req[s] = k % 8;k //= 8
        if used_warm and force < 0.:
            # a state of negative thrust reached from a neighbour: the one from rest where it has more thrust
            for i in range(n):q[i] = self.ax_initial[i]
            hj = False
            if not self.axial_core(prep, req, nreq, q, J, &hj, rr, &ps2):
                if self.ax_converged(&ps2) and ps2.frame.force[0] > force:
                    for i in range(n):
                        oQ[best*n+i] = q[i];r[i] = rr[i]
                    oH[best] = hj
                    if hj:
                        for i in range(n*n):oJ[best*n*n+i] = J[i]
                    ps[0] = ps2
                    conv = True;force = ps.frame.force[0]
        over = self.ax_overspeed(ps)
        valid = conv and not over and self.ax_flows(v, ps)
        needs_fine = (not over) and ((not conv) or (force < 0. and self.ax_status_any(ps, 3)))
        if self.ax_has_fine and needs_fine:
            if not self.ax_fine_state(prep, v, q, rr, &ps2):
                for i in range(n):
                    oQ[i] = q[i];r[i] = rr[i]
                ps[0] = ps2;on[0] = 0
                return 3
        return 0 if valid else 1

    cdef int ax_fine_state(self, const PropPrep* prep, double v, double* q_out, double* r_out, PropSample* ps_out) except -1:
        """engine_performance._fine_pitch_record and its acceptance: over every request combination, the closed
        fine-stop state of negative thrust with the smallest residual, where it is a result (0: q_out, r_out, ps_out);
        1 without one."""
        cdef int req[NP]
        cdef int digits[NP]
        cdef int n = self.ax_n, i, s, c, k, total = 1
        cdef double q[MAXPATH]
        cdef double rr[MAXPATH]
        cdef double J[MAXPATH*MAXPATH]
        cdef PropSample ps
        cdef bint hj, found = False
        cdef double norm, best = 0.
        if not self.ax_has_fine:return 1
        for s in range(self.nt):
            digits[s] = prep.ncand[s] if prep.ncand[s] > 1 else 1
            total *= digits[s]
        for c in range(total):
            k = c
            for s in range(self.nt-1, -1, -1):
                req[s] = k % digits[s];k //= digits[s]
            for i in range(n):q[i] = self.ax_fine[i]
            hj = False
            if self.axial_core(prep, req, self.nt, q, J, &hj, rr, &ps):continue
            if not self.ax_converged(&ps) or self.ax_overspeed(&ps) or not (ps.frame.force[0] < 0.) or not self.ax_status_any(&ps, 1):continue
            norm = 0.
            for i in range(n):norm += rr[i]*rr[i]
            norm = sqrt(norm)
            if not found or norm < best:
                found = True;best = norm
                for i in range(n):
                    q_out[i] = q[i];r_out[i] = rr[i]
                ps_out[0] = ps
        if not found or not self.ax_flows(v, ps_out):return 1
        return 0

    cdef object ax_entry(self, int kind, const double* oQ, const double* oJ, const char* oH, const long long* oB, int on,
                         int chosen, const double* r, const PropSample* ps):
        """A row point for the caller: (2 or 3, `axial_result` of its state, that state's request of each shaft, every
        kept state as (requests, coordinates, Jacobian or None))."""
        cdef int n = self.ax_n, k, i, j
        if kind == 3:return (3, self.axial_result(oQ, r, oJ, False, ps), None, [])
        states = []
        for k in range(on):
            states.append((self.ax_requests(oB[k]), [oQ[k*n+i] for i in range(n)],
                           [[oJ[k*n*n+i*n+j] for j in range(n)] for i in range(n)] if oH[k] else None))
        return (2, self.axial_result(&oQ[chosen*n], r, &oJ[chosen*n*n], oH[chosen], ps), self.ax_requests(oB[chosen]), states)

    def path_entry(self, AxStore store, int i):
        """Point i of a solved path (AxStore.entry)."""
        if not (0 <= i < store.count) or store.kind[i] < 2:return None
        return self.ax_again(store.G, store.dt, store.ALT[i], store.SPD[i], store.kind[i], &store.Q[i*store.n], store.B[i])

    cdef object ax_again(self, double G, double dt, double altitude, double speed, int kind, const double* q, long long packed):
        """A solved path point once more from its coordinates: its `ax_entry` (AxStore.entry)."""
        cdef PropPrep prep
        cdef PropSample ps
        cdef int req[NP]
        cdef double r[MAXPATH]
        cdef double home = self.altitude
        cdef long long k = packed
        cdef int s
        for s in range(self.nt-1, -1, -1):
            req[s] = <int>(k % 8);k //= 8
        self.dt_aircraft = dt;self.altitude = altitude
        try:
            self.prepare(speed/3.6, 1., 0., 0., G, &prep)
            if self.propulsion_at(&prep, q, req, self.nt, r, &ps):return None
            return (kind, self.axial_result(q, r, q, False, &ps), self.ax_requests(packed) if kind == 2 else None, [])
        finally:self.altitude = home

    cdef tuple ax_requests(self, long long packed):
        """A packed request combination as each shaft's request index."""
        cdef int s
        out = [0]*self.nt
        for s in range(self.nt-1, -1, -1):
            out[s] = int(packed % 8);packed //= 8
        return tuple(out)

    cdef void ax_keep(self, int i, int code, int cap, const double* tQ, const double* tJ, const char* tH, const long long* tB,
                      int on, int sel, const PropSample* ps, double* Q, double* JJ, char* H, long long* B, int* NS,
                      double* QC, long long* BC, double* L) noexcept:
        """A path point's new result into the path's arrays: its states for its neighbours to start from (code 0),
        its own state's coordinates and requests, and its six numbers for the caller."""
        cdef int n = self.ax_n, d, k
        cdef const PFrame* fr = &ps.frame
        cdef double power = 0., fuel = 0., rocket = 0., gears = 0., base = 1.
        if code == 0:
            for d in range(on*n):Q[i*cap*n+d] = tQ[d]
            for d in range(on*n*n):JJ[i*cap*n*n+d] = tJ[d]
            for d in range(on):
                H[i*cap+d] = tH[d];B[i*cap+d] = tB[d]
            NS[i] = on
            for d in range(n):QC[i*n+d] = tQ[sel*n+d]
            BC[i] = tB[sel]
        else:
            NS[i] = 0
            for d in range(n):QC[i*n+d] = tQ[d]
            BC[i] = 0
        for k in range(self.ne):
            power += fr.engines[k].torque*fr.engines[k].omega
            if self.engines[k].family == 3:rocket += fr.engines[k].force[0]
            else:fuel += fr.engines[k].consumption
            gears += base*fr.engines[k].gear;base *= 8.
        L[i*6] = 2. if code == 0 else 3.;L[i*6+1] = fr.force[0];L[i*6+2] = power;L[i*6+3] = fuel;L[i*6+4] = rocket;L[i*6+5] = gears

    def axial_row(self, double G, double dt, int lo, int hi, allowed):
        """A whole row at this model's altitude: every whole km/h lo..hi (`axial_path` along speed)."""
        return self.axial_path(G, dt, [self.altitude]*(hi-lo+1), [float(x) for x in range(lo, hi+1)], allowed, ())

    def axial_path(self, double G, double dt, altitudes, speeds, allowed, anchors, bint full=True, int cap=1, int retry=1):
        """Every point of a path solved (alpha 0, 1 g) and settled: point i at altitudes[i] (m) and speeds[i] (km/h),
        neighbours in the path one step apart (a row: one altitude, every whole km/h; an altitude curve: one speed,
        every whole metre). Each point is solved from the nearest solved one within 32 steps, else from rest and the
        alternate starts; then the states in `anchors` ((index, [(requests, coordinates, Jacobian or None), ...]) are
        continued at their points; then, until nothing changes, every state is continued to the next point down and
        up, where it replaces a state of less thrust (or none). allowed[i]: whether point i is within the aircraft's
        limits. Returns None where the layout is not this loop's or a point has more than AXC request combinations,
        else for each point None or `ax_entry`. The model's altitude is as before afterwards.

        full False (a long path, of which the caller needs few points in full): returns instead (the bytes of six
        doubles a point: 0 no state / 2 / 3, thrust N, shaft power W, fuel kg/s without rockets, rocket thrust N, the
        engines' gears packed in base 8; an AxStore giving `ax_entry` of any point on request).
        cap: how many request combinations a point's arrays hold (1 to start with; the path is solved again with AXC
        where a point has more).
        retry: after a point with no solved point near found no state from rest either, the points within this many
        steps after it are not started from rest again (1: every point is; along altitude the next metre is the same
        flight condition to a part in ten thousand, and what failed there fails here)."""
        cdef int n = self.ax_n, count = len(speeds), i, x, u, d, step, a, code, rounds, pick, tried_n, c2, on, sel, on2, sel2, m, sj
        cdef double* QC
        cdef long long* BC
        cdef double* L
        cdef AxStore store
        cdef bint again = False
        cdef int last_cold = -1000000, outer
        cdef bint found_more
        cdef PropPrep prep
        cdef PropSample ps
        cdef PropSample ps2
        cdef double r[MAXPATH]
        cdef double rb[MAXPATH]
        cdef bint changed
        cdef double v, kgf, old
        cdef char* kind
        cdef char* none_tried
        cdef char* H
        cdef char* tH
        cdef char* tH2
        cdef double* Q
        cdef double* JJ
        cdef double* T
        cdef double* tQ
        cdef double* tJ
        cdef double* tQ2
        cdef double* tJ2
        cdef long long* B
        cdef long long tB[AXC]
        cdef long long tB2[AXC]
        cdef int* NS
        cdef unsigned long long* lower_tried
        cdef unsigned long long* upper_tried
        # A state continued from a neighbour gives the same again while that neighbour's state is the same: ver counts
        # a speed's changes, from_lower / from_upper hold the neighbour's count when its state was last continued here.
        cdef int* ver
        cdef int* from_lower
        cdef int* from_upper
        cdef double* ALT
        cdef double* SPD
        cdef double home = self.altitude
        if not self.ax_row_ready or self.nrows != self.nactive or n != self.nactive or n <= 0 or count <= 0:return None
        if len(altitudes) != count or len(allowed) != count:raise ValueError('axial_path: one altitude and one flag for each speed')
        self.dt_aircraft = dt
        if cap < 1 or cap > AXC:cap = AXC
        m = cap*n;sj = cap*n*n
        kind = <char*>calloc(count, 1);none_tried = <char*>calloc(count, 1)
        H = <char*>calloc(count*cap, 1);tH = <char*>calloc(AXC, 1);tH2 = <char*>calloc(AXC, 1)
        Q = <double*>calloc(count*m, sizeof(double));JJ = <double*>calloc(count*sj, sizeof(double))
        tQ = <double*>calloc(AXC*n, sizeof(double));tJ = <double*>calloc(AXC*n*n, sizeof(double))
        tQ2 = <double*>calloc(AXC*n, sizeof(double));tJ2 = <double*>calloc(AXC*n*n, sizeof(double))
        T = <double*>calloc(count, sizeof(double))
        B = <long long*>calloc(count*cap, sizeof(long long));NS = <int*>calloc(count, sizeof(int))
        QC = <double*>calloc(count*n, sizeof(double));BC = <long long*>calloc(count, sizeof(long long))
        L = <double*>calloc(count*6, sizeof(double))
        lower_tried = <unsigned long long*>calloc(count, sizeof(unsigned long long))
        upper_tried = <unsigned long long*>calloc(count, sizeof(unsigned long long))
        ver = <int*>calloc(count, sizeof(int));from_lower = <int*>calloc(count, sizeof(int));from_upper = <int*>calloc(count, sizeof(int))
        for i in range(count):
            from_lower[i] = -1;from_upper[i] = -1
        ALT = <double*>calloc(count, sizeof(double));SPD = <double*>calloc(count, sizeof(double))
        for i in range(count):
            ALT[i] = altitudes[i];SPD[i] = speeds[i]
        res = [None]*count
        try:
            # ---- every speed once, in increasing order
            for i in range(count):
                if not allowed[i]:
                    kind[i] = 1
                    continue
                v = SPD[i]/3.6;self.altitude = ALT[i]
                self.prepare(v, 1., 0., 0., G, &prep)
                u = -1
                for d in range(1, 33):
                    if i-d >= 0 and kind[i-d] == 2:
                        u = i-d
                        break
                    if i+d < count and kind[i+d] == 2:
                        u = i+d
                        break
                if u >= 0:
                    if u < i:lower_tried[i] |= (<unsigned long long>1) << (i-u)
                    else:upper_tried[i] |= (<unsigned long long>1) << (u-i)
                    if u == i-1:from_lower[i] = ver[u]
                    elif u == i+1:from_upper[i] = ver[u]
                    code = self.ax_solve(&prep, v, &Q[u*m], &JJ[u*sj], &H[u*cap], &B[u*cap], NS[u], True, NULL, tQ, tJ, tH, tB, &on, &sel, r, &ps, cap)
                else:
                    if i-last_cold < retry:
                        kind[i] = 1;none_tried[i] = 1
                        continue
                    none_tried[i] = 2
                    code = self.ax_solve(&prep, v, NULL, NULL, NULL, NULL, 0, True, NULL, tQ, tJ, tH, tB, &on, &sel, r, &ps, cap)
                if code == 9:return None
                if code == 8:
                    again = True
                    raise AxMore()
                if u < 0:
                    # no solved speed near: the alternate starts too; of the states found, the one with more thrust
                    for a in range(self.ax_nalt):
                        c2 = self.ax_solve(&prep, v, NULL, NULL, NULL, NULL, 0, True, self.ax_alt[a], tQ2, tJ2, tH2, tB2, &on2, &sel2, rb, &ps2, cap)
                        if c2 == 9:return None
                        if c2 == 8:
                            again = True
                            raise AxMore()
                        if c2 != 0:continue
                        kgf = ps2.frame.force[0]/9.80665
                        old = ps.frame.force[0]/9.80665 if code != 1 else 0.
                        if code == 1 or kgf > old+1e-4*pmax(fabs(old), 1.):
                            for d in range(AXC*n):tQ[d] = tQ2[d]
                            for d in range(AXC*n*n):tJ[d] = tJ2[d]
                            for d in range(AXC):
                                tH[d] = tH2[d];tB[d] = tB2[d]
                            for d in range(n):r[d] = rb[d]
                            on = on2;sel = sel2;ps = ps2;code = 0
                if code == 1:
                    kind[i] = 1
                    if u < 0:last_cold = i
                    continue
                kind[i] = 2 if code == 0 else 3
                T[i] = ps.frame.force[0]/9.80665;ver[i] += 1
                self.ax_keep(i, code, cap, tQ, tJ, tH, tB, on, sel, &ps, Q, JJ, H, B, NS, QC, BC, L)
                if full:res[i] = self.ax_entry(kind[i], tQ, tJ, tH, tB, on, sel, r, &ps)
            # ---- states given from outside (another path's states at some of these points), continued here as a
            # neighbour's are: kept where the point has no state or one of less thrust
            for anchor in anchors:
                i = anchor[0]
                if not (0 <= i < count) or not allowed[i]:continue
                on2 = 0
                for requests, coords, jac in anchor[1]:
                    if len(coords) != n:continue
                    if on2 >= cap:raise AxMore()
                    tB2[on2] = 0
                    for d in range(self.nt):tB2[on2] = tB2[on2]*8+int(requests[d])
                    for d in range(n):tQ2[on2*n+d] = coords[d]
                    tH2[on2] = jac is not None
                    if jac is not None:
                        for d in range(n):
                            for c2 in range(n):tJ2[on2*n*n+d*n+c2] = jac[d][c2]
                    on2 += 1
                if not on2:continue
                v = SPD[i]/3.6;self.altitude = ALT[i]
                self.prepare(v, 1., 0., 0., G, &prep)
                code = self.ax_solve(&prep, v, tQ2, tJ2, tH2, tB2, on2, False, NULL, tQ, tJ, tH, tB, &on, &sel, r, &ps, cap)
                if code == 9:return None
                if code == 8:
                    again = True
                    raise AxMore()
                if code != 0:continue
                kgf = ps.frame.force[0]/9.80665
                if kind[i] < 2 or kgf > T[i]+1e-4*pmax(fabs(T[i]), 1.):
                    kind[i] = 2;T[i] = kgf;ver[i] += 1
                    self.ax_keep(i, 0, cap, tQ, tJ, tH, tB, on, sel, &ps, Q, JJ, H, B, NS, QC, BC, L)
                    if full:res[i] = self.ax_entry(2, tQ, tJ, tH, tB, on, sel, r, &ps)
            # ---- states continued down and up until nothing changes
            for outer in range(2):
                for rounds in range(8):
                    changed = False
                    for step in (1, -1):
                        i = count-2 if step == 1 else 1
                        while 0 <= i < count:
                            u = i+step
                            if kind[u] >= 2 and allowed[i]:
                                v = SPD[i]/3.6;self.altitude = ALT[i]
                                if kind[i] == 1:
                                    # no state here yet: once more from the solved speeds near it that have not started it
                                    self.prepare(v, 1., 0., 0., G, &prep)
                                    tried_n = 0
                                    for d in range(1, 33):
                                        for pick in (i-d, i+d):
                                            if tried_n >= 2 or pick < 0 or pick >= count or kind[pick] != 2:continue
                                            if pick < i:
                                                if lower_tried[i] & ((<unsigned long long>1) << d):continue
                                                lower_tried[i] |= (<unsigned long long>1) << d
                                            else:
                                                if upper_tried[i] & ((<unsigned long long>1) << d):continue
                                                upper_tried[i] |= (<unsigned long long>1) << d
                                            tried_n += 1
                                            if kind[i] != 1:continue
                                            if pick == i-1:from_lower[i] = ver[pick]
                                            elif pick == i+1:from_upper[i] = ver[pick]
                                            code = self.ax_solve(&prep, v, &Q[pick*m], &JJ[pick*sj], &H[pick*cap], &B[pick*cap], NS[pick], True, NULL,
                                                                 tQ, tJ, tH, tB, &on, &sel, r, &ps, cap)
                                            if code == 9:return None
                                            if code == 8:
                                                again = True
                                                raise AxMore()
                                            if code == 1:continue
                                            kind[i] = 2 if code == 0 else 3
                                            T[i] = ps.frame.force[0]/9.80665;ver[i] += 1
                                            self.ax_keep(i, code, cap, tQ, tJ, tH, tB, on, sel, &ps, Q, JJ, H, B, NS, QC, BC, L)
                                            if full:res[i] = self.ax_entry(kind[i], tQ, tJ, tH, tB, on, sel, r, &ps)
                                            changed = True
                                if kind[u] == 2 and (from_upper[i] if step == 1 else from_lower[i]) != ver[u]:
                                    # the neighbour's state continued here: kept where it has more thrust than what is here
                                    if step == 1:from_upper[i] = ver[u]
                                    else:from_lower[i] = ver[u]
                                    self.prepare(v, 1., 0., 0., G, &prep)
                                    code = self.ax_solve(&prep, v, &Q[u*m], &JJ[u*sj], &H[u*cap], &B[u*cap], NS[u], False, NULL,
                                                         tQ, tJ, tH, tB, &on, &sel, r, &ps, cap)
                                    if code == 9:return None
                                    if code == 8:
                                        again = True
                                        raise AxMore()
                                    if code == 0:
                                        kgf = ps.frame.force[0]/9.80665
                                        if kind[i] < 2 or kgf > T[i]+1e-4*pmax(fabs(T[i]), 1.):
                                            kind[i] = 2;T[i] = kgf;ver[i] += 1
                                            self.ax_keep(i, 0, cap, tQ, tJ, tH, tB, on, sel, &ps, Q, JJ, H, B, NS, QC, BC, L)
                                            if full:res[i] = self.ax_entry(2, tQ, tJ, tH, tB, on, sel, r, &ps)
                                            changed = True
                            i -= step
                    if not changed:break
                if outer or retry <= 1:break
                # the points left without a state that the first pass did not start from rest (retry): from rest and
                # the alternate starts now; where one finds a state, the states are continued once more
                found_more = False;tried_n = 0
                for i in range(count):
                    if kind[i] != 1 or not allowed[i] or none_tried[i] != 1:continue
                    if tried_n >= AX_RESCUE:break
                    tried_n += 1
                    v = SPD[i]/3.6;self.altitude = ALT[i]
                    self.prepare(v, 1., 0., 0., G, &prep)
                    code = self.ax_solve(&prep, v, NULL, NULL, NULL, NULL, 0, True, NULL, tQ, tJ, tH, tB, &on, &sel, r, &ps, cap)
                    if code == 9:return None
                    if code == 8:raise AxMore()
                    for a in range(self.ax_nalt):
                        c2 = self.ax_solve(&prep, v, NULL, NULL, NULL, NULL, 0, True, self.ax_alt[a], tQ2, tJ2, tH2, tB2, &on2, &sel2, rb, &ps2, cap)
                        if c2 == 9:return None
                        if c2 == 8:raise AxMore()
                        if c2 != 0:continue
                        kgf = ps2.frame.force[0]/9.80665
                        old = ps.frame.force[0]/9.80665 if code != 1 else 0.
                        if code == 1 or kgf > old+1e-4*pmax(fabs(old), 1.):
                            for d in range(AXC*n):tQ[d] = tQ2[d]
                            for d in range(AXC*n*n):tJ[d] = tJ2[d]
                            for d in range(AXC):
                                tH[d] = tH2[d];tB[d] = tB2[d]
                            for d in range(n):r[d] = rb[d]
                            on = on2;sel = sel2;ps = ps2;code = 0
                    none_tried[i] = 2
                    if code == 1:continue
                    kind[i] = 2 if code == 0 else 3
                    T[i] = ps.frame.force[0]/9.80665;ver[i] += 1
                    self.ax_keep(i, code, cap, tQ, tJ, tH, tB, on, sel, &ps, Q, JJ, H, B, NS, QC, BC, L)
                    if full:res[i] = self.ax_entry(kind[i], tQ, tJ, tH, tB, on, sel, r, &ps)
                    found_more = True
                if not found_more:break
            if full:return res
            store = AxStore.__new__(AxStore)
            store.model = self;store.count = count;store.n = n;store.G = G;store.dt = dt
            store.kind = kind;store.Q = QC;store.B = BC;store.ALT = ALT;store.SPD = SPD
            kind = NULL;QC = NULL;BC = NULL;ALT = NULL;SPD = NULL
            return (PyBytes_FromStringAndSize(<char*>L, count*6*sizeof(double)), store)
        except AxMore:
            again = True
        finally:
            pfree(kind);pfree(none_tried);pfree(H);pfree(tH);pfree(tH2);pfree(Q);pfree(JJ);pfree(tQ);pfree(tJ);pfree(tQ2);pfree(tJ2)
            pfree(T);pfree(B);pfree(NS);pfree(lower_tried);pfree(upper_tried);pfree(ver);pfree(from_lower);pfree(from_upper)
            pfree(ALT);pfree(SPD);pfree(QC);pfree(BC);pfree(L)
            self.altitude = home
        if again:return None if cap >= AXC else self.axial_path(G, dt, altitudes, speeds, allowed, anchors, full, AXC, retry)


cdef double net_thrust(PropModel pm, const PFrame* fr) noexcept:
    """S10: the net thrust (N) of the shafts' propellers in a propulsion frame: each propeller's force along its own axis
    (em_stationary.propeller_braking's sum)."""
    cdef double thrust = 0.
    cdef int s, k, idx, i
    for s in range(pm.nt):
        for k in range(pm.trans[s].np):
            idx = pm.trans[s].props[k].index
            for i in range(3):thrust += fr.props[idx].outputs[i]*pm.props[idx].basis[i]
    return thrust


cdef double shaft_power(PropModel pm, const PFrame* fr) noexcept:
    """S10: the power (W) the shafts' engines deliver in a propulsion frame (torque x shaft speed)."""
    cdef double power = 0.
    cdef int s, k, idx
    for s in range(pm.nt):
        for k in range(pm.trans[s].ne):
            idx = pm.trans[s].engines[k].index
            power += fr.engines[idx].torque*fr.engines[idx].omega
    return power


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
