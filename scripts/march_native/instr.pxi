# Native Instructor admission: instructor_steady_aoa / instructor_aoa
# controller_limits (steady effective AoA schedule with reduced moment balance
# and fixed auto trim), transliterated from the baseline runtime modules
# instructor_chart_inputs, windows_instructor_source, instructor_keyboard,
# instructor_protection, instructor_predictor_inputs, instructor_pitch_predictor
# (prepare_geometry), instructor_aoa_balance, instructor_reduced,
# instructor_autotrim (direct equilibrium path), instructor_trim_equilibrium
# and instructor_static_trim. Binary32 rounding and operation order follow the
# Python source; the one-g trim correction uses the native corrector (rounding-
# level agreement, like the curve corrections). Per-Mach polars come from the
# baseline polar_runtime and are cached by their exact Mach input.

DEF NIP = 40          # predictor input slots (0xa0 bytes / 4)
DEF NF = 8468         # predictor f table (offset/4)

class InstructorUnsupported(Exception):
    pass


cdef inline double DIVF(double a, double b) noexcept nogil:
    # instructor_protection.divide
    return F(a/b) if fabs(b) > F_TINY else 0.


cdef struct IWing:
    int downwash_type
    double incidence, dihedral, span, sweep, taper, downwash_coefficient, v_focus
    double areas[2][3]
    double flaps_shift[2]
    double gear_shift[2]
    double airbrakes_shift[2]
    double arm[3]


cdef struct IGeo:
    Polar polars[2]
    Polar tail
    double lim_ail[3][3]
    double lim_elev[3][3]
    double legacy_aspect, speed
    double wing_x[2]
    double wing_y[2]
    double wing_z[2]
    double tail_areas[2]
    double tail_gain, ail_gain, ail_sens, tail_area, tail_flow_lever, tail_lever
    double tail_cl[2]
    double area_sensitivity_scale, flight_path_cos, cos_dihedral, polar_area_over_span
    bint has_sensitivity
    double q_area[2]
    double negative_q_area[2]
    double moment_wing_x[2]
    double moment_wing_y[2]
    double fuse_drag, vstab_drag, vstab_drag_factor, fuse_y, vstab_y, parasite_drag, wash_attenuation


cdef struct MachG:
    double mach
    Polar wing
    Polar tail
    Polar fuselage
    Polar vstab


cdef struct MachW:
    double mach
    Polar wing


cdef struct Balance:
    double acceleration, tail_force, zero_force, tail_angle, flow
    bint direct


cdef struct Auto:
    double output[3]
    bint success, saturated
    double lift_error, moment_error, command_error


cdef struct IIn:
    # effective_limits inputs (one phase of the value record)
    double tas, speed_squared, mach, ias_u
    double wing_angles[2]
    double gear_fraction
    double delivered[3]
    double longitudinal
    double world_acc[3]
    double quaternion[4]
    double pitch_rate
    double speed
    bint has_prop
    double force[3]
    double moment[3]
    double wash_raw[2]


cdef struct InstrRes:
    # effective_limits outputs; the record dict is built from these (Instructor.limits_dict)
    bint converged, success, independent
    double angle_margin, mechanical
    double bounds[3][2]
    double trim[3]
    double sticks[3]
    double low, high
    double effective[2]
    double adjusted[2]
    double rate
    double demand_angle[2]
    double demand_acc[2]
    Balance bal[2]
    int auto_index


cdef void axis_limits_mix(const Mix* p, const bint* inverted, double density, double speed, double out[3][3]) noexcept nogil:
    # control_mixer.axis_limits: out[axis] = (positive, center, negative)
    cdef double lim[3]
    cdef int i
    for i in range(3):
        if p.use2[i]:
            surface_curve(p, i, density, speed, lim)
        else:
            curve(&p.t1[i], speed, 3, lim)
        if inverted[i]:
            out[i][0] = M(lim[2], p.angles[i][1])
            out[i][1] = M(lim[1], p.angles[i][1] if lim[1] >= 0 else p.angles[i][0])
            out[i][2] = M(lim[0], -p.angles[i][0])
        else:
            out[i][0] = M(lim[0], p.angles[i][0])
            out[i][1] = M(lim[1], p.angles[i][0] if lim[1] >= 0 else p.angles[i][1])
            out[i][2] = M(lim[2], -p.angles[i][1])


cdef double deflection_pitch(double command, const bint* inverted, double lim[3][3]) noexcept nogil:
    # instructor_pitch_predictor.deflection([0, command, 0], inverted, limits)
    cdef double values[3]
    cdef double cmd, slope, value
    cdef int i
    for i in range(3):
        cmd = command if i == 1 else 0.
        slope = S(lim[i][1], lim[i][0]) if cmd >= 0. else S(lim[i][2], lim[i][1])
        value = S(lim[i][1], M(slope, cmd))
        values[i] = -value if inverted[i] else value
    return A(values[2], A(values[1], values[0]))


cdef double inverse_cl(const Polar* p, double target, bint* valid) noexcept nogil:
    cdef double c, linear, square, root, crit, cy, parab
    cdef bint positive
    c = F(F(target)/p.clKq)
    if c > p.cyCritH:
        valid[0] = False;return p.aoaCritH
    if c < p.cyCritL:
        valid[0] = False;return p.aoaCritL
    linear = DIVF(S(c, p.cl0), M(p.cyMult, p.clLineCoeff))
    valid[0] = True
    if p.aoaLineL <= linear and linear <= p.aoaLineH:
        return linear
    positive = A(linear, F(.01)) >= p.aoaLineH
    crit = p.aoaCritH if positive else p.aoaCritL
    cy = p.cyCritH if positive else p.cyCritL
    parab = p.parabCyCoeffH if positive else p.parabCyCoeffL
    square = DIVF(fabs(S(cy, c)), parab)
    root = F(sqrt(square)) if square > 0 else 0.
    return S(crit, root if positive else -root)


cdef inline double remap(double x, double lo, double hi, double a, double b) noexcept nogil:
    cdef double t
    if lo > hi:
        t = lo;lo = hi;hi = t
        t = a;a = b;b = t
    if x <= lo:return a
    if x >= hi:return b
    return A(a, DIVF(M(S(x, lo), S(b, a)), S(hi, lo)))


cdef double projected_cl(const Polar* p, double a, double angle, bint convert) noexcept nogil:
    cdef double r, sn, cs
    r = M(angle if convert else S(a, angle), F(.01745329238474369))
    sn = F(sin(r));cs = F(cos(r))
    return M(A(M(calc_cd(p, a), sn), M(calc_cl(p, a), cs)), p.clKq)


cdef double inverse_rotated_cl(const Polar* p, double target, double angle, bint convert, bint* ok) noexcept nogil:
    cdef double x = 0., delta = F(.01), inv_delta, residual, derivative, change
    cdef int k
    target = F(target);angle = F(angle)
    inv_delta = F(1./delta)
    if target > A(M(p.cyCritH, p.clKq), F(-.001)):
        ok[0] = False;return p.aoaCritH
    if target < A(M(p.clKq, p.cyCritL), F(.001)):
        ok[0] = False;return p.aoaCritL
    for k in range(15):
        residual = S(projected_cl(p, x, angle, convert), target)
        derivative = M(A(S(-residual, target), projected_cl(p, A(x, delta), angle, convert)), inv_delta)
        if fabs(derivative) < F_TINY:
            ok[0] = False;return F(0.)
        change = F(-residual/derivative)
        x = A(x, change)
        if fabs(change) < delta:
            ok[0] = True;return x
    ok[0] = False
    return F(0.)


cdef struct FlowIn:
    int downwash_type
    double legacy_factor, span, area, sweep, taper, dihedral, working_alpha, wing_angle
    const Polar* wing_polars
    double wing_x[2]
    double wing_y[2]
    double wing_z[2]
    double tail_point[3]
    double tail_polar_offset, coefficient, pitch_rate, tail_lever, speed, sin_angle, cos_angle
    int engine_count
    bint clockwise
    double axial_wash, swirl_wash, tail_area_delta, inverse_tail_area


cdef double tail_flow(const FlowIn* q) noexcept nogil:
    cdef double sn, cs, speed, vx, vy, induced, swirl, rad, sa, ca, sb, cb, invhalf, sc, cc, x, y, z, w, base
    cdef double dx, dy, dz2, xy, zw, px, py, along, across, shape, numerator, angle
    cdef double contributions[2]
    cdef int i
    if q.downwash_type != 2:
        sn = q.sin_angle;cs = q.cos_angle;speed = q.speed
        vx = M(cs, speed)
        vy = S(M(sn, -speed), M(q.pitch_rate, q.tail_lever))
        if q.downwash_type == 1:
            induced = M(calc_cl(&q.wing_polars[0], q.wing_angle), q.legacy_factor)
            vy = A(M(cs, induced), vy)
            vx = A(M(induced, sn), vx)
            if q.engine_count == 1:
                swirl = (.36000001430511475*q.tail_area_delta)*q.swirl_wash*q.inverse_tail_area
                vy = vy-swirl
                vx = vx+q.axial_wash
        return M(F(atan2(vy, vx)), F(-57.2957763671875))
    rad = F(.01745329238474369)
    inline_half_sincos(F(M_PI), &sa, &ca)
    inline_half_sincos(M(q.working_alpha, rad), &sb, &cb)
    invhalf = F(2./q.span) if fabs(M(q.span, .5)) > F_TINY else 0.
    for i in range(2):
        inline_half_sincos(M(q.dihedral if i == 0 else -q.dihedral, rad), &sc, &cc)
        x = A(M(M(cc, sa), sb), M(M(sc, ca), cb))
        y = A(M(M(cc, sa), cb), M(M(sc, ca), sb))
        z = S(M(M(cc, ca), sb), M(M(sc, sa), cb))
        w = S(M(M(cc, ca), cb), M(M(sc, sa), sb))
        base = A(A(M(w, w), M(w, w)), -1.)
        dx = A(A(q.tail_polar_offset, q.tail_point[0]), q.wing_x[i])
        dy = S(q.tail_point[1], q.wing_y[i])
        dz2 = M(S(q.tail_point[2], q.wing_z[i]), 2.) if i == 0 else M(A(q.tail_point[2], q.wing_z[i]), -2.)
        xy = M(A(x, x), y);zw = M(S(-z, z), w)
        px = M(A(A(M(S(M(x, z), M(y, w)), dz2), M(S(xy, zw), dy)), M(A(A(M(x, x), M(x, x)), base), dx)), invhalf)
        if px > 0.:
            py = M(invhalf, A(A(M(dz2, A(M(x, w), M(z, y))), M(dx, A(xy, zw))), M(dy, A(base, A(M(y, y), M(y, y))))))
            along = A(F(exp(M(px, -2.5))), F(.9))
            across = F(exp(M(fabs(py), F(-1.409678))))
            contributions[i] = M(M(calc_cl(&q.wing_polars[i], q.wing_angle), along), across)
        else:
            contributions[i] = 0.
    shape = S(F(1.56), M(F(exp(M(q.taper, F(-.35)))), F(1.2)))
    shape = M(shape, M(q.area, F(-23.1)))
    numerator = M(M(M(A(F(sin(M(q.sweep, rad))), 1.), A(contributions[0], contributions[1])), shape), q.coefficient)
    vx = M(q.cos_angle, q.speed);vy = S(M(q.sin_angle, -q.speed), M(q.pitch_rate, q.tail_lever))
    if q.engine_count == 1:
        swirl = ((q.swirl_wash*q.tail_area_delta)*q.inverse_tail_area)*(.36000001430511475 if q.clockwise else -.36000001430511475)
        angle = F(atan2(vy+swirl, vx+q.axial_wash))
    else:
        angle = F(atan2(vy, vx))
    return A(M(angle, F(-57.2957763671875)), F(numerator/M(q.span, q.span)))


cdef struct BalanceIn:
    double cd[2]
    double cl[2]
    double control_cl, extra_cd, fuselage_cd
    double q_area[2]
    double negative_q_area[2]
    double cos_dihedral
    double cm0[2]
    double cm1[2]
    double polar_area_over_span
    double wing_x[2]
    double wing_y[2]
    double other_moment, engine_pitch_moment, pitch_inertia, target_acceleration, weight, flight_path_cos
    double reference_x, tail_lever, working_alpha
    bint legacy_balance_shift


cdef void pitch_balance(const BalanceIn* b, double* wing_force, double* tail_force) noexcept nogil:
    cdef double lift_coeff[2]
    cdef double drag[2]
    cdef double lift[2]
    cdef double intrinsic[2]
    cdef double lift_moment, moment, summed_drag, summed_lift, shift, denominator
    cdef int i
    for i in range(2):
        lift_coeff[i] = A(b.control_cl, b.cl[i])
        drag[i] = M(A(A(b.cd[i], b.fuselage_cd), b.extra_cd), b.negative_q_area[i])
        lift[i] = M(M(b.cos_dihedral, b.q_area[i]), lift_coeff[i])
    for i in range(2):
        intrinsic[i] = F(M(A(M(b.cm1[i], lift_coeff[i]), b.cm0[i]), b.polar_area_over_span)/A(M(lift_coeff[i], lift_coeff[i]), M(b.cd[i], b.cd[i])))
    lift_moment = M(S(M(intrinsic[0], lift_coeff[0]), b.wing_x[0]), lift[0])
    lift_moment = S(lift_moment, M(F(b.pitch_inertia), b.target_acceleration))
    lift_moment = S(lift_moment, M(M(b.weight, b.flight_path_cos), b.reference_x))
    lift_moment = A(lift_moment, M(S(M(intrinsic[1], lift_coeff[1]), b.wing_x[1]), lift[1]))
    moment = A(b.other_moment, b.engine_pitch_moment)
    for i in range(2):
        moment = A(moment, M(S(b.wing_y[i], M(intrinsic[i], b.cd[i])), drag[i]))
    moment = A(moment, lift_moment)
    summed_drag = A(drag[1], drag[0]);summed_lift = A(lift[1], lift[0])
    if b.legacy_balance_shift:
        shift = M(F(sin(M(fabs(b.working_alpha), F(.01745329238474369)))), .5)
        moment = A(moment, M(summed_lift, shift))
        denominator = S(b.tail_lever, A(shift, b.reference_x))
    else:
        denominator = S(b.tail_lever, b.reference_x)
    wing_force[0] = summed_drag;wing_force[1] = summed_lift
    tail_force[0] = DIVF(moment, denominator)


cdef void tail_command(double flow_angle, double required_angle, double flap_incidence, double tail_incidence,
                       double area_sensitivity_scale, bint has_sensitivity, double center, double positive_delta,
                       double negative_delta, bint inverted, double* command, bint* saturated, double* deflection) noexcept nogil:
    cdef double incidence, requested, lo, hi, d, delta, denominator
    incidence = A(flap_incidence, tail_incidence)
    requested = M(A(S(flow_angle, required_angle), incidence), area_sensitivity_scale)
    if not has_sensitivity:requested = 0.
    lo = A(negative_delta, center);hi = A(positive_delta, center)
    saturated[0] = requested <= lo or requested >= hi
    d = pmin(pmax(requested, lo), hi)
    delta = negative_delta if d <= center else positive_delta
    denominator = -delta if d <= center else delta
    command[0] = DIVF(S(d, center), denominator)
    if inverted:command[0] = -command[0]
    deflection[0] = d


cdef inline double legacy_wake_factor(double wash_attenuation, double axial_speed, double aspect) noexcept nogil:
    return F(M(M(F(1./M_PI), wash_attenuation), axial_speed)/aspect)


cdef class Instructor


cdef double angle_rate(const double* velocity, const double* acceleration, const double* quaternion, double pitch_rate) noexcept:
    cdef double a, b, c, recip, x, y, z, w, rx, ry, rz, p, q, r, ax, ay, az, projected
    cdef double normal[3]
    a = F(velocity[0]);b = F(velocity[1]);c = F(velocity[2])
    recip = F(1./pmax(1., F(sqrt(A(M(c, c), A(M(b, b), M(a, a)))))))
    a = M(a, recip);b = M(b, recip);c = M(c, recip)
    x = F(quaternion[0]);y = F(quaternion[1]);z = F(quaternion[2]);w = F(quaternion[3])
    rx = S(M(y, x), M(z, w));rx = A(rx, rx)
    ry = A(M(y, y), M(w, w));ry = A(A(ry, ry), -1.)
    rz = A(M(w, x), M(z, y));rz = A(rz, rz)
    p = S(M(b, rz), M(c, ry));q = S(M(c, rx), M(rz, a));r = S(M(ry, a), M(rx, b))
    normal[0] = S(M(q, c), M(r, b));normal[1] = S(M(r, a), M(c, p));normal[2] = S(M(p, b), M(q, a))
    ax = F(acceleration[0]);ay = F(acceleration[1]);az = F(acceleration[2])
    projected = A(M(az, normal[2]), A(M(ay, normal[1]), M(ax, normal[0])))
    return M(projected, recip)+pitch_rate


cdef struct TrimMemo:
    double x[3]
    double r[3]
    int br[4]
    Auto auto


cdef class TrimCorrector(NativeCorrector):
    cdef Instructor owner
    cdef IGeo c
    cdef double ip[NIP]
    cdef double weight, moment_scale
    cdef TrimMemo* memo
    cdef int nmemo, cap

    def __cinit__(self):
        self.memo = NULL;self.nmemo = 0;self.cap = 0

    def __dealloc__(self):
        if self.memo != NULL:free(self.memo)

    cdef int lookup(self, const double* x) except -1:
        # instructor_trim_equilibrium.solve.evaluate: memoized by the exact coordinates
        cdef int i
        cdef TrimMemo* t
        cdef Auto au
        for i in range(self.nmemo-1, -1, -1):
            t = &self.memo[i]
            if t.x[0] == x[0] and t.x[1] == x[1] and t.x[2] == x[2]:
                return i
        self.owner.autotrim(self.ip, &self.c, x[0], x[1]*self.weight, x[2], &au)
        if self.nmemo == self.cap:
            self.cap = 16 if self.cap == 0 else 2*self.cap
            t = <TrimMemo*>realloc(self.memo, self.cap*sizeof(TrimMemo))
            if t == NULL:raise MemoryError()
            self.memo = t
        t = &self.memo[self.nmemo]
        for i in range(3):t.x[i] = x[i]
        t.auto = au
        t.r[0] = au.lift_error/self.weight;t.r[1] = au.moment_error/self.moment_scale;t.r[2] = au.command_error
        t.br[0] = x[0] >= 0.;t.br[1] = x[1] >= 0.;t.br[2] = x[2] >= 0.;t.br[3] = au.saturated
        self.nmemo += 1
        return self.nmemo-1

    cdef int evaluate(self, const double* q, Sample* out) except -1:
        cdef int i, k
        k = self.lookup(q)
        for i in range(3):out.r[i] = self.memo[k].r[i]
        for i in range(BMAX):out.br[i] = 0
        for i in range(4):out.br[i] = self.memo[k].br[i]
        out.slot = k
        return OK


cdef class Instructor:
    """instructor_at(value, speed) for one solver (steady effective-AoA schedule)."""
    cdef Model model
    cdef public object solver
    cdef double f[NF]
    cdef bint fl_7fa6, fl_7fa3, fl_7c08, fl_7c0a, fl_7c54, fl_8471, fl_6f3c
    cdef double pitch_inertia, rho0, balance_multiplier
    cdef int engine_count
    cdef bint new_balance
    cdef IWing wing
    cdef object wing_runtime_geo, wing_runtime_fixed, polar_tail, polar_fuselage, polar_vstab
    cdef double legacy_aspect
    cdef MachG mg[32]
    cdef int nmg, mg_next
    cdef MachW mw[32]
    cdef int nmw, mw_next
    # wrapper constants
    cdef double w_mass, w_height, w_engine_scale, w_flap_blend, w_flap_incidence, w_sweep, w_gear
    cdef double w_oil, w_water, w_cockpit
    cdef bint w_torque_gyro
    cdef double flaps_value
    # source constants
    cdef double reference_speed, wing_area, dihedral, strength[2], tail_area_pair[2]
    cdef bint force_advanced
    cdef double crit_mult[2]
    cdef bint limit_load_factor
    cdef double load_factor_limit[2]
    cdef double pitch_coeff[4]
    cdef double tail_bounds[2]
    cdef double elevator_wing_aoa
    cdef Primary sel           # selected_properties(fm) for the Instructor authority ranges
    cdef Primary alloc_props   # solver.controls with the retained-trim availability
    cdef bint quasi_steady, direct_mode, rudder_trim_needed
    cdef double rudder_incidence
    cdef Polar vstab0
    cdef public dict trim_cache
    cdef object _wing_select
    cdef public object predictor
    cdef object _f_dict
    cdef public bint python_trim
    cdef public long calls, fallbacks, trim_solves, trim_evaluations
    cdef public object authority_mode, authority_revision
    cdef bint invert_elevator
    cdef double trim_limit
    cdef public list autos
    cdef dict auto_ids
    cdef public object loop_stats
    cdef object skipped_auto

    def __init__(self, Model model, solver):
        import json, math
        from instructor_chart_inputs import source_state, GAMEPLAY
        from tail_model import aircraft_secondary_properties
        from control_mixer import curve as py_curve
        from polar_runtime import flap_polar, evaluate as mach_polar
        from instructor_settings import restored_wing_normalization, trim_retained
        from instructor_protection import gain_reference_speed
        from jet_catalog import engines as installed_engines
        from body_dynamics import realistic_engine_scale
        from wing_sweep import select
        from primary_controls import selected_properties
        from instructor_trim_equilibrium import REVISION as authority_revision
        from component_assembly import f32
        self.model = model;self.solver = solver
        m = solver.model;fm = m['fm'];ad = fm['Aerodynamics'];g = m['geometry'];mass = solver.mass
        cfg = solver.config
        import instructor_steady_aoa
        if instructor_steady_aoa.REVISION != 'steady-effective-aoa-fixed-autotrim-v9':
            raise InstructorUnsupported('Instructor revision')
        self.authority_revision = authority_revision
        self.authority_mode = cfg.get('instructor_authority_mode', 'native')
        self.direct_mode = cfg.get('instructor_authority_mode') == 'direct'
        tp = aircraft_secondary_properties(fm, {})
        flaps = solver.flaps
        self.flaps_value = flaps
        mapped = py_curve(m['flaps'], f32(flaps), 4) if m['flaps'] else [f32(flaps), f32(flaps), 0., 0.]
        self.wing_runtime_fixed = flap_polar(m['polars']['WingPlane'], mapped[0])
        self.reference_speed = gain_reference_speed(fm['Mass'].get('Takeoff', 0.), flap_polar(m['wing_family'][0][1]['polars'], 0.))
        for i in range(NF):self.f[i] = 0.
        f = {0x18a0: 1., 0x18d4: 1., 0x18d8: 1., 0x8448: 0.}
        for off, v in zip([0x5320, 0x5324, 0x5328], mass['cog']):f[off] = f32(v)
        for off, v in zip([0x6f20, 0x6f24, 0x6f28], tp['arms']['hstab']):f[off] = f32(v)
        for off, v in [(0x7334, tp['arms']['fuselage'][1]), (0x712c, tp['arms']['vstab'][1]), (0x7140, tp['slipstream_distance']),
                       (0x6f1c, tp['incidence']['hstab'])]:f[off] = f32(v)
        for key, off in dict(fuselage=0x83fc, left_main=0x8420, left_elevator=0x8424, right_main=0x8428, right_elevator=0x842c,
                             v_main=0x8430, rudder=0x8434).items():f[off] = f32(tp['areas'][key])
        for off, key in [(0x7cc4, 'CockpitDoorCd'), (0x7ca8, 'GearCd'), (0x7cb4, 'RadiatorCd'), (0x7cb8, 'OilRadiatorCd'),
                         (0x7cb0, 'AirbrakeCd'), (0x7cc0, 'FuseCd')]:f[off] = f32(ad.get(key, 0.))
        f[0x79b8], f[0x79bc] = m['controls']['Ailerons']['cd'];f.update(restored_wing_normalization(g['areas']))
        for off, v in f.items():
            if off % 4 or off//4 >= NF:raise InstructorUnsupported('predictor table layout')
            self.f[off//4] = float(v)
        self._f_dict = f
        self.fl_7fa6 = bool(fm['AvailableControls']['hasFlapsControl']);self.fl_7fa3 = bool(fm['AvailableControls']['hasGearControl'])
        self.fl_7c08 = bool(fm.get('AllowModsToChangeLongidutialBalance', True));self.fl_7c0a = bool(fm.get('ConvertAoa', False))
        self.fl_7c54 = bool(fm['InvertElevator']);self.fl_8471 = False;self.fl_6f3c = bool(tp['clockwise'])
        if '_instructor_engine_count' not in m:
            m['_instructor_engine_count'] = len(installed_engines(fm))
        self.engine_count = m['_instructor_engine_count']
        props = dict(GAMEPLAY['propsDefault']);props.update(fm.get('Instructor', {}))
        self.tail_area_pair[0] = f32(ad['HorStabPlane']['Areas']['Main']);self.tail_area_pair[1] = f32(ad['HorStabPlane']['Areas']['Elevator'])
        self.pitch_inertia = mass['inertia'][2];self.balance_multiplier = 1.;self.new_balance = True;self.rho0 = f32(1.225)
        flags = {0x7fa6: self.fl_7fa6, 0x7fa3: self.fl_7fa3, 0x7c08: self.fl_7c08, 0x7c0a: self.fl_7c0a, 0x7c54: self.fl_7c54,
                 0x8471: False, 0x6f3c: self.fl_6f3c}
        self.predictor = dict(f=dict(self._f_dict), flags=flags, pitch_inertia=mass['inertia'][2], engine_count=self.engine_count,
                              balance_multiplier=1., new_balance=True, rho0=f32(1.225))
        self.python_trim = False
        # wrapper constants
        self.w_mass = f32(mass['mass']);self.w_height = float(cfg['altitude_m'])
        self.w_engine_scale = realistic_engine_scale(1., fm.get('ExtThrustBaseMult', 1.))
        self.w_torque_gyro = bool(cfg['torque_gyro']);self.w_flap_blend = mapped[0];self.w_flap_incidence = mapped[2]
        self.w_sweep = m['sweep'];self.w_gear = solver.gear
        self.w_oil = solver.oil_radiator;self.w_water = solver.radiator;self.w_cockpit = 0.
        self.wing_area = g['area'];self.dihedral = g['dihedral']
        self.strength[0] = g['strength']['force'][0];self.strength[1] = g['strength']['force'][1]
        self.force_advanced = bool(props.get('MouseAim', {}).get('forceAdvanced', False))
        crit = list(map(f32, props.get('critMult', [-1., -1.])))
        if crit[0] < 0:crit = list(map(f32, [.8, .9]))
        self.crit_mult[0] = crit[0];self.crit_mult[1] = crit[1]
        self.limit_load_factor = bool(props.get('limitLoadfactor', False))
        lf = list(props.get('loadFactorLimit', [-5., 12.]))
        if len(lf) != 2:raise InstructorUnsupported('load factor limit layout')
        self.load_factor_limit[0] = lf[0];self.load_factor_limit[1] = lf[1]
        pc = list(map(f32, props.get('constPitchDerrCoeffMultByClLinearRatio', [0., 1., .01, 2.])))
        for i in range(4):self.pitch_coeff[i] = pc[i]
        tail0 = mach_polar(m['polars']['HorStabPlane'][0][1], 0.)
        self.tail_bounds[0] = F(tail0['aoaCritL']*F(.97));self.tail_bounds[1] = F(tail0['aoaCritH']*F(.97))
        self.elevator_wing_aoa = m['controls']['Elevator']['wing_aoa']
        self.quasi_steady = m.get('aircraft_trim_mode') == 'quasi_steady'
        # rb_rudder_trim constants
        self.rudder_trim_needed = not self.w_torque_gyro
        set_polar(&self.vstab0, mach_polar(m['polars']['VerStabPlane'][0][1], 0.))
        self.rudder_incidence = f32(ad['VerStabPlane'].get('Angle', 0.))
        # authority properties
        sp = selected_properties(fm)
        for i in range(3):
            for j in range(2):
                self.sel.eff[i][j] = sp['effective_speed'][i][j];self.sel.power[i][j] = sp['power'][i][j]
            self.sel.minimum[i] = sp['minimum'][i];self.sel.negative[i] = sp['negative'][i]
            self.sel.trim_available[i] = False
        self.sel.strong = bool(sp['strong']);self.sel.invert_elevator = bool(sp['invert_elevator'])
        c = solver.controls
        ground = [fm['AvailableControls'].get('has'+axis+'TrimGroundControl', False) for axis in ('Aileron', 'Elevator', 'Rudder')]
        retained = trim_retained(c['trim_available'], ground, 1, True)
        for i in range(3):
            for j in range(2):
                self.alloc_props.eff[i][j] = c['effective_speed'][i][j];self.alloc_props.power[i][j] = c['power'][i][j]
            self.alloc_props.minimum[i] = c['minimum'][i];self.alloc_props.negative[i] = c['negative'][i]
            self.alloc_props.trim_available[i] = bool(retained[i])
        self.alloc_props.strong = bool(c['strong']);self.alloc_props.invert_elevator = bool(c['invert_elevator'])
        # predictor geometry: the selected wing at the (constant) sweep and flap input
        sweep32 = f32(self.w_sweep)
        wing = select(m['wing_family'], sweep32);gw = wing['geometry']
        if gw['downwash_type'] not in (0, 1, 2):raise InstructorUnsupported('downwash type')
        self.wing.downwash_type = gw['downwash_type'];self.wing.incidence = gw['incidence'];self.wing.dihedral = gw['dihedral']
        self.wing.span = gw['span'];self.wing.sweep = gw['sweep'];self.wing.taper = gw['taper']
        self.wing.downwash_coefficient = gw['downwash_coefficient'];self.wing.v_focus = gw['v_focus']
        for i in range(2):
            for j in range(3):self.wing.areas[i][j] = gw['areas'][i][j]
            self.wing.flaps_shift[i] = gw['shifts']['FlapsShift'][i];self.wing.gear_shift[i] = gw['shifts']['GearShift'][i]
            self.wing.airbrakes_shift[i] = gw['shifts']['AirbrakesShift'][i]
        for j in range(3):self.wing.arm[j] = gw['arm'][j]
        self._wing_select = (sweep32, wing)
        self.wing_runtime_geo = None
        self.polar_tail = m['polars']['HorStabPlane'][0][1];self.polar_fuselage = m['polars']['FuselagePlane'][0][1]
        self.polar_vstab = m['polars']['VerStabPlane'][0][1]
        self.nmg = self.mg_next = self.nmw = self.mw_next = 0
        self.trim_cache = {}
        self.calls = self.fallbacks = self.trim_solves = self.trim_evaluations = 0
        self.invert_elevator = bool(solver.controls['invert_elevator'])
        self.trim_limit = cfg['trim_limit']
        self.autos = [];self.auto_ids = {};self.loop_stats = None
        self.skipped_auto = dict(success=False, skipped=True, trim=[0., 0., 0.], method='trim-independent authority',
                                 reason='Automatic trim is unnecessary for control bounds')

    cdef int geo_mach(self, double mach, double flap) except -1:
        cdef int i
        for i in range(self.nmg):
            if self.mg[i].mach == mach:return i
        from polar_runtime import flap_polar, evaluate as mach_polar
        if self.wing_runtime_geo is None or self.wing_runtime_geo[0] != flap:
            runtime = flap_polar(self._wing_select[1]['polars'], flap)
            self.wing_runtime_geo = (flap, runtime)
            self.legacy_aspect = runtime['base'][0]
            self.nmg = 0;self.mg_next = 0
        runtime = self.wing_runtime_geo[1]
        if self.nmg < 32:
            i = self.nmg;self.nmg += 1
        else:
            i = self.mg_next;self.mg_next = (self.mg_next+1) % 32
        self.mg[i].mach = mach
        set_polar(&self.mg[i].wing, mach_polar(runtime, mach, 1.))
        set_polar(&self.mg[i].tail, mach_polar(self.polar_tail, mach))
        set_polar(&self.mg[i].fuselage, mach_polar(self.polar_fuselage, mach))
        set_polar(&self.mg[i].vstab, mach_polar(self.polar_vstab, mach))
        return i

    cdef int fixed_mach(self, double mach) except -1:
        cdef int i
        cdef Polar* p
        cdef double distance, denominator, slope, numerator
        for i in range(self.nmw):
            if self.mw[i].mach == mach:return i
        from polar_runtime import evaluate as mach_polar
        if self.nmw < 32:
            i = self.nmw;self.nmw += 1
        else:
            i = self.mw_next;self.mw_next = (self.mw_next+1) % 32
        self.mw[i].mach = mach
        p = &self.mw[i].wing
        set_polar(p, mach_polar(self.wing_runtime_fixed, mach, 1.))
        if not self.quasi_steady:
            # windows_instructor_source.fixed_source
            distance = S(p.aoaCritH, p.aoaLineH);denominator = M(distance, distance)
            slope = M(p.clLineCoeff, p.cyMult)
            numerator = S(p.cyCritH, A(M(p.aoaLineH, slope), p.cl0))
            p.parabCyCoeffH = F(numerator/denominator) if denominator > F_TINY else 0.
        return i

    # ---------------------------------------------------------------- predictor geometry
    cdef int geometry(self, const double* ip, IGeo* c) except -1:
        cdef double flap, flap_shift, x, y, dx, dy, lever, speed, speed2, q, wash_speed, washq, sn, cs, elevator_area
        cdef double vstab_area, tail_sens, ail_sens, sensitivity_den, ias, cd_out[2]
        cdef double areas[2]
        cdef double s1[1]
        cdef bint inv[3]
        cdef int i, k
        cdef const double* f = self.f
        cdef const IWing* g = &self.wing
        cdef MachG* mg
        if not (ip[0] == 0. or ip[0] == 1.) or ip[3] != 0. or ip[10] != 0.:
            raise ValueError('Only intact mode-0/1 predictor with prescribed pitch rate')
        if ip[0x74//4] != F(self.w_sweep):
            raise InstructorUnsupported('sweep input')
        flap = ip[0x6c//4] if self.fl_7fa6 else 0.
        k = self.geo_mach(ip[0x40//4], flap)
        mg = &self.mg[k]
        for i in range(2):
            c.polars[i] = mg.wing
        c.polars[0].indCoeff = M(DIVF(1., f[0x843c//4]), c.polars[0].indCoeff)
        c.polars[1].indCoeff = M(DIVF(1., f[0x8440//4]), c.polars[1].indCoeff)
        c.tail = mg.tail
        for i in range(2):
            areas[i] = A(A(g.areas[i][1], g.areas[i][0]), g.areas[i][2])
        c.tail_areas[0] = A(f[0x8424//4], f[0x8420//4]);c.tail_areas[1] = A(f[0x8428//4], f[0x842c//4])
        elevator_area = A(f[0x842c//4], f[0x8424//4])
        c.tail_area = A(c.tail_areas[1], c.tail_areas[0])
        sn = F(sin(M(g.dihedral, RAD_W)));cs = F(cos(M(g.dihedral, RAD_W)))
        flap_shift = pmin(M(ip[0x6c//4], 3.), 1.) if self.fl_7fa6 else 0.
        x = A(M(g.flaps_shift[0], flap_shift), g.arm[0])
        y = A(M(flap_shift, g.flaps_shift[1]), g.arm[1])
        dx = A(M(g.gear_shift[0], ip[0x78//4]), M(g.airbrakes_shift[0], ip[0x7c//4]))
        dy = A(M(ip[0x7c//4], g.airbrakes_shift[1]), M(ip[0x78//4], g.gear_shift[1]))
        c.wing_x[0] = A(A(c.polars[0].aerCenterOffset, x), dx);c.wing_x[1] = A(A(x, dx), c.polars[1].aerCenterOffset)
        c.wing_z[0] = M(g.arm[2], F(1.3));c.wing_z[1] = M(F(-1.3), g.arm[2])
        c.wing_y[0] = S(M(M(g.v_focus, sn), fabs(c.wing_z[0])), A(y, dy))
        c.wing_y[1] = S(M(fabs(c.wing_z[1]), M(g.v_focus, sn)), A(y, dy))
        lever = A(S(f[0x6f20//4], f[0x5320//4]), c.tail.aerCenterOffset)
        speed = ip[0x38//4]
        speed = pmax(speed, 1.) if speed >= 0. else pmin(speed, -1.)
        c.speed = speed
        speed2 = M(speed, speed);q = M(M(.5, speed2), ip[0x44//4])
        wash_speed = A(M(ip[0x48//4], F(.07)), ip[0x38//4])
        wash_speed = pmax(wash_speed, 1.) if wash_speed >= 0. else pmin(wash_speed, -1.)
        washq = M(M(wash_speed, wash_speed), M(ip[0x44//4], .5))
        calc_c(&mg.fuselage, 0., 0., 0., 1., cd_out)
        c.fuse_drag = M(M(M(M(ip[0x44//4], cd_out[0]), M(speed2, -.5)), f[0x18a0//4]), f[0x83fc//4])
        vstab_area = A(M(f[0x18d4//4], M(f[0x8430//4], 1.5)), F(.2))
        vstab_area = A(M(f[0x8434//4], f[0x18d8//4]), vstab_area)
        calc_c(&mg.vstab, 0., 0., 0., 1., cd_out)
        c.vstab_drag = M(q, M(vstab_area, -cd_out[0]))
        c.vstab_drag_factor = M(vstab_area, -cd_out[0])
        curve(&self.model.mixers[1].sens_curve, ip[0x40//4], 1, s1);tail_sens = s1[0]
        curve(&self.model.mixers[0].sens_curve, ip[0x40//4], 1, s1);ail_sens = s1[0]
        c.tail_gain = M(self.model.mixers[1].sensitivity, tail_sens)
        sensitivity_den = M(pmax(elevator_area, F(.01)), c.tail_gain)
        ias = M(F(sqrt(F(ip[0x44//4]/self.rho0))), ip[0x38//4])
        inv[0] = True;inv[1] = False;inv[2] = False
        axis_limits_mix(&self.model.mixers[0], inv, ip[0x44//4], ias, c.lim_ail)
        axis_limits_mix(&self.model.mixers[1], inv, ip[0x44//4], ias, c.lim_elev)
        c.legacy_aspect = self.legacy_aspect
        c.tail_cl[0] = M(self.model.mixers[1].cl[0], tail_sens);c.tail_cl[1] = M(self.model.mixers[1].cl[1], tail_sens)
        c.ail_gain = M(self.model.mixers[0].sensitivity, ail_sens);c.ail_sens = -ail_sens
        c.tail_flow_lever = lever;c.tail_lever = S(lever, c.tail.clToCm1)
        c.area_sensitivity_scale = M(elevator_area, F(1./sensitivity_den)) if sensitivity_den != 0. else float('inf')
        c.has_sensitivity = fabs(sensitivity_den) > F_TINY
        c.flight_path_cos = F(sqrt(S(1., M(ip[0x34//4], ip[0x34//4]))))
        for i in range(2):
            c.q_area[i] = M(washq, areas[i]);c.negative_q_area[i] = M(-areas[i], washq)
        c.cos_dihedral = cs;c.polar_area_over_span = F(f[0x8438//4]/g.span)
        for i in range(2):
            c.moment_wing_x[i] = -A(f[0x5320//4], c.wing_x[i]);c.moment_wing_y[i] = S(c.wing_y[i], f[0x5324//4])
        c.fuse_y = S(f[0x7334//4], f[0x5324//4]);c.vstab_y = S(f[0x712c//4], f[0x5324//4])
        c.parasite_drag = A(S(M(ip[0x88//4], f[0x7cc4//4]), M(f[0x8448//4], washq)), c.fuse_drag)
        c.wash_attenuation = F(1./M(A(M(f[0x7140//4], F(.04)), 1.), A(M(f[0x7140//4], F(.04)), 1.)))
        return 0

    cdef void flow_inputs(self, const IGeo* c, const double* ip, FlowIn* q, double working, double sn, double cs,
                          double wing_angle, double pitch_rate, double axial, double swirl, double legacy_factor) noexcept:
        cdef int i
        cdef const IWing* g = &self.wing
        q.downwash_type = g.downwash_type;q.legacy_factor = legacy_factor
        q.span = g.span
        q.area = A(A(A(g.areas[1][1], g.areas[1][0]), g.areas[1][2]), A(A(g.areas[0][1], g.areas[0][0]), g.areas[0][2]))
        q.sweep = g.sweep;q.taper = g.taper;q.dihedral = g.dihedral;q.working_alpha = working;q.wing_angle = wing_angle
        q.wing_polars = c.polars
        for i in range(2):
            q.wing_x[i] = c.wing_x[i];q.wing_y[i] = c.wing_y[i];q.wing_z[i] = c.wing_z[i]
        q.tail_point[0] = self.f[0x6f20//4];q.tail_point[1] = self.f[0x6f24//4];q.tail_point[2] = self.f[0x6f28//4]
        q.tail_polar_offset = c.tail.aerCenterOffset;q.coefficient = g.downwash_coefficient;q.pitch_rate = pitch_rate
        q.tail_lever = c.tail_flow_lever;q.speed = c.speed;q.sin_angle = sn;q.cos_angle = cs
        q.engine_count = self.engine_count;q.clockwise = self.fl_6f3c;q.axial_wash = axial;q.swirl_wash = swirl
        q.tail_area_delta = S(c.tail_areas[0], c.tail_areas[1]);q.inverse_tail_area = 1./c.tail_area

    # ---------------------------------------------------------------- reduced balance
    cdef int required_acceleration(self, const double* ip, double delivered_pitch, Balance* out) except -1:
        cdef IGeo c
        cdef FlowIn fq
        cdef BalanceIn b
        cdef double working, sn, cs, vx, vy, axial = 0., swirl = 0., qvx, wash, k, spin, dynamic, taq
        cdef double center, positive, negative, command, a, e, cladd, bias, angle, vstab_drag, other, extra, flow
        cdef double td, required, tail_cl, tail_force, lever, left[2], right[2], wing_force[2], zero_force, tail_out[2]
        cdef double products[4]
        cdef bint inv[3]
        cdef bint convert = self.fl_7c0a
        cdef const double* f = self.f
        if ip[0] != 1.:raise ValueError('Expected mode 1')
        self.geometry(ip, &c)
        inv[0] = True;inv[1] = False;inv[2] = False
        working = S(ip[1], self.wing.incidence)
        sn = F(sin(M(working, RAD_W)));cs = F(cos(M(working, RAD_W)))
        vx = M(cs, c.speed);vy = M(sn, -c.speed)
        qvx = vx
        if ip[0x48//4] != 0.:
            wash = M(ip[0x48//4], c.wash_attenuation)
            k = pmax(A(F(M(fabs(vy), -1.5)/pmax(A(wash, vx), F(.2))), 1.), 0.)
            axial = M(wash, k)
            if fabs(axial) < F(1.1920928955078125e-7):axial = 0.
            qvx = A(vx, ip[0x48//4])
            if ip[0x68//4] != 0. and not self.fl_8471:
                spin = pmin(M(M(qvx, qvx), F(.0011)), 1.) if qvx >= 0. else 0.
                swirl = M(M(M(k, c.wash_attenuation), spin), ip[0x4c//4])
                if fabs(swirl) < F(1.1920928955078125e-7):swirl = 0.
        dynamic = M(M(A(M(vy, vy), M(qvx, qvx)), .5), ip[0x44//4])
        taq = M(dynamic, c.tail_area)
        center = c.lim_elev[1][1];positive = S(c.lim_elev[1][0], center);negative = S(c.lim_elev[1][2], center)
        command = F(delivered_pitch)
        a = deflection_pitch(command, inv, c.lim_ail)
        e = deflection_pitch(command, inv, c.lim_elev)
        cladd = M(M(a, c.ail_sens), self.model.mixers[0].cl[1 if a < 0. else 0])
        bias = S(M(e, -self.elevator_wing_aoa), M(a, c.ail_gain))
        angle = A(bias, ip[1])
        calc_c(&c.polars[0], angle, working if convert else S(angle, self.wing.incidence), 0., 1., left)
        calc_c(&c.polars[1], angle, S(angle, self.wing.incidence), 0., 1., right)
        out.direct = (M(fabs(S(c.lim_ail[1][0], c.lim_ail[1][2])), c.ail_gain) <= F(.001)
                      and M(fabs(S(c.lim_elev[1][0], c.lim_elev[1][2])), self.elevator_wing_aoa) <= F(.001)
                      and pmax(c.tail_cl[0], c.tail_cl[1]) <= F(.001))
        vstab_drag = c.vstab_drag if out.direct else M(dynamic, c.vstab_drag_factor)
        other = A(M(S(c.fuse_y, ip[0x20//4]), c.fuse_drag), M(vstab_drag, c.vstab_y))
        products[0] = M(ip[0x78//4] if self.fl_7fa3 else 1., f[0x7ca8//4])
        products[1] = M(ip[0x80//4], f[0x7cb4//4]);products[2] = M(ip[0x84//4], f[0x7cb8//4]);products[3] = M(ip[0x7c//4], f[0x7cb0//4])
        extra = A(A(products[3], products[1]), A(products[2], products[0]))
        b.cd[0] = left[0];b.cd[1] = right[0];b.cl[0] = left[1];b.cl[1] = right[1];b.control_cl = cladd
        b.extra_cd = extra;b.fuselage_cd = f[0x7cc0//4]
        for i in range(2):
            b.q_area[i] = c.q_area[i];b.negative_q_area[i] = c.negative_q_area[i]
            b.cm0[i] = c.polars[i].clToCm0;b.cm1[i] = c.polars[i].clToCm1
            b.wing_x[i] = c.moment_wing_x[i];b.wing_y[i] = c.moment_wing_y[i]
        b.cos_dihedral = c.cos_dihedral;b.polar_area_over_span = c.polar_area_over_span
        b.other_moment = other;b.engine_pitch_moment = ip[0x64//4];b.pitch_inertia = self.pitch_inertia
        b.target_acceleration = 0.;b.weight = ip[0x2c//4];b.flight_path_cos = c.flight_path_cos
        b.reference_x = ip[0x1c//4];b.tail_lever = c.tail_lever;b.working_alpha = working
        b.legacy_balance_shift = not self.new_balance and self.fl_7c08
        self.flow_inputs(&c, ip, &fq, working, sn, cs, ip[1], ip[0x10//4], axial, swirl,
                         legacy_wake_factor(c.wash_attenuation, qvx, c.legacy_aspect) if self.wing.downwash_type == 1 else 0.)
        flow = tail_flow(&fq)
        td = A(M(positive if command >= 0. else -negative, command), center)
        required = S(A(flow, A(ip[0x70//4], f[0x6f1c//4])), DIVF(td, c.area_sensitivity_scale))
        calc_c(&c.tail, required, working if convert else required, 0., 1., tail_out)
        tail_cl = tail_out[1]
        tail_force = M(taq, S(tail_cl, M(c.tail_cl[1 if td < 0. else 0], td)))
        pitch_balance(&b, wing_force, &zero_force)
        lever = S(c.tail_lever, ip[0x1c//4])
        if b.legacy_balance_shift:
            lever = S(c.tail_lever, A(M(F(sin(M(fabs(working), RAD_W))), .5), ip[0x1c//4]))
        out.acceleration = (zero_force-tail_force)*lever/F(self.pitch_inertia)
        out.tail_force = tail_force;out.zero_force = zero_force;out.tail_angle = required;out.flow = flow
        return 0

    # ---------------------------------------------------------------- auto trim (direct equilibrium)
    cdef void roll_balance(self, const IGeo* c, const double* ip, double target, double wing_angle,
                           double plain_ail[3][3], double plain_elev[3][3], double* roll, double* pair) noexcept:
        cdef double left, right, tl, tr, cg, al, ar, el, er, sn, zl, zr, bl, br_, wl, wr, wing_sum, wing_lever, wing_control
        cdef double inverse_q, forcing, tail_zl, tail_zr, dl, dr, tail_ratio, wing_ratio, denominator, numerator
        cdef bint positive
        cdef const Polar* p = &c.polars[0]
        cdef const Polar* q = &c.polars[1]
        cdef const Polar* t = &c.tail
        cdef const double* f = self.f
        cdef const IWing* g = &self.wing
        left = A(A(g.areas[0][1], g.areas[0][0]), g.areas[0][2]);right = A(A(g.areas[1][1], g.areas[1][0]), g.areas[1][2])
        tl = c.tail_areas[0];tr = c.tail_areas[1];cg = f[0x5328//4]
        positive = left >= right
        # slope(limits): hi, center, lo = limits[0] (roll axis)
        al = M(c.ail_gain, S(c.lim_ail[0][0], c.lim_ail[0][1]) if positive else -S(c.lim_ail[0][2], c.lim_ail[0][1]))
        ar = M(c.ail_gain, S(plain_ail[0][0], plain_ail[0][1]) if positive else -S(plain_ail[0][2], plain_ail[0][1]))
        el = S(c.lim_elev[0][0], c.lim_elev[0][1]) if positive else -S(c.lim_elev[0][2], c.lim_elev[0][1])
        er = S(plain_elev[0][0], plain_elev[0][1]) if positive else -S(plain_elev[0][2], plain_elev[0][1])
        sn = F(sin(M(g.dihedral, RAD_W)))
        zl = S(M(c.wing_z[0], c.cos_dihedral), A(M(c.wing_y[0], sn), cg))
        zr = S(A(M(c.wing_y[1], sn), M(c.wing_z[1], c.cos_dihedral)), cg)
        bl = M(p.clLineCoeff, left);br_ = M(q.clLineCoeff, right)
        wl = M(bl, al);wr = M(br_, ar)
        wing_sum = A(bl, br_)
        wing_lever = pmax(A(M(br_, zr), M(bl, zl)), F(1e-6))
        wing_control = S(M(wr, zr), M(wl, zl))
        inverse_q = DIVF(1., DIVF(c.q_area[0], left))
        forcing = A(A(M(ip[0x5c//4], inverse_q), M(M(zr, right), q.cl0)), M(M(zl, left), p.cl0))
        tail_zl = S(f[0x6f28//4], cg);tail_zr = A(f[0x6f28//4], cg)
        dl = M(M(M(c.tail_gain, tl), t.clLineCoeff), el)
        dr = M(M(M(c.tail_gain, tr), t.clLineCoeff), er)
        tail_ratio = DIVF(A(M(t.clLineCoeff, tl), M(t.clLineCoeff, tr)),
                          pmax(S(M(M(t.clLineCoeff, tl), tail_zl), M(M(t.clLineCoeff, tr), tail_zr)), F(1e-6)))
        wing_ratio = DIVF(wing_sum, wing_lever)
        denominator = A(S(S(M(wing_control, wing_ratio), M(A(M(dr, tail_zr), M(dl, tail_zl)), tail_ratio)), A(wr, dr)), A(wl, dl))
        numerator = A(M(S(M(S(M(tail_zl, tl), M(tail_zr, tr)), tail_ratio), c.tail_area), t.cl0),
                      A(M(S(target, p.cl0), A(left, right)), A(M(wing_ratio, forcing), M(target, c.tail_area))))
        roll[0] = pmax(-1., pmin(1., DIVF(numerator, denominator)))
        pair[0] = A(M(al, roll[0]), wing_angle);pair[1] = S(wing_angle, M(ar, roll[0]))

    cdef int autotrim(self, const double* ip, const IGeo* c, double angle, double tail_force, double current,
                      Auto* out) except -1:
        # instructor_autotrim.autotrim_predictor(..., equilibrium_state=(angle, tail_force, current))
        cdef double areas[2]
        cdef double area, roll = 0., axial = 0., swirl = 0., center, positive, negative, level_scale, wash_speed, washq
        cdef double sum_q_area, command, a, e, cladd, bias, wing_angle, ad, drag_control, other_drag, working, sn, cs, dynamic
        cdef double taq, vstab_drag, required, eng, other_normal, required_wing, error, k0, inv0, shiftx0, shifty0, inv1, k1
        cdef double shiftx1, shifty1, mx, my, unmet, td, target, flow_angle, proposed, deflection, command_error
        cdef double effective, actual, residual, vx, vy, wash, k, spin
        cdef double pair[2]
        cdef double extra[4]
        cdef double tc[2]
        cdef double pol[2][2]
        cdef double cls[2]
        cdef double drag[2]
        cdef double lift[2]
        cdef double wings[2]
        cdef double plain_ail[3][3]
        cdef double plain_elev[3][3]
        cdef bint inv[3]
        cdef bint fl[3]
        cdef bint asymmetric, wing_ok, tail_ok, tail_final_ok, saturated
        cdef bint convert = self.fl_7c0a, invert = self.fl_7c54
        cdef int side
        cdef FlowIn fq
        cdef const double* f = self.f
        cdef const IWing* g = &self.wing
        if ip[0] != 0. or ip[1] != 1. or ip[0x30//4] != 1. or ip[0x34//4] != 0.:
            raise ValueError('Mode-0 one-g horizontal auto trim only')
        inv[0] = True;inv[1] = False;inv[2] = False
        fl[0] = False;fl[1] = False;fl[2] = False
        for side in range(2):
            areas[side] = A(A(g.areas[side][1], g.areas[side][0]), g.areas[side][2])
        area = A(areas[0], areas[1])
        asymmetric = (fabs(ip[0x5c//4]) > F(.01) or fabs(S(areas[0], areas[1])) > M(area, F(.01))
                      or fabs(S(c.tail_areas[0], c.tail_areas[1])) > M(c.tail_area, F(.01)) or fabs(f[0x5328//4]) > F(.1))
        if asymmetric:
            axis_limits_mix(&self.model.mixers[0], fl, ip[0x44//4], M(F(sqrt(F(ip[0x44//4]/self.rho0))), ip[0x38//4]), plain_ail)
            axis_limits_mix(&self.model.mixers[1], fl, ip[0x44//4], M(F(sqrt(F(ip[0x44//4]/self.rho0))), ip[0x38//4]), plain_elev)
        angle = F(angle);tail_force = F(tail_force);current = F(current)
        center = c.lim_elev[1][1];positive = S(c.lim_elev[1][0], center);negative = S(c.lim_elev[1][2], center)
        level_scale = 1. if self.fl_7c08 else DIVF(1., self.balance_multiplier)
        level_scale = M(level_scale, c.flight_path_cos)
        wash_speed = A(M(ip[0x48//4], F(.07)), ip[0x38//4])
        wash_speed = pmax(wash_speed, 1.) if wash_speed >= 0. else pmin(wash_speed, -1.)
        washq = M(M(wash_speed, wash_speed), M(ip[0x44//4], .5))
        sum_q_area = M(washq, area)
        command = -current if invert else current
        a = deflection_pitch(command, inv, c.lim_ail)
        e = deflection_pitch(command, inv, c.lim_elev)
        cladd = M(M(a, c.ail_sens), self.model.mixers[0].cl[1 if a < 0. else 0])
        bias = S(M(e, -self.elevator_wing_aoa), M(a, c.ail_gain))
        wing_angle = A(angle, bias)
        pair[0] = wing_angle;pair[1] = wing_angle
        if asymmetric:
            self.roll_balance(c, ip, M(calc_cl(&c.polars[0], wing_angle), c.polars[0].clKq), wing_angle, plain_ail, plain_elev,
                              &roll, pair)
        wing_ok = (c.polars[0].aoaCritL <= pair[0] and pair[0] <= c.polars[0].aoaCritH
                   and c.polars[1].aoaCritL <= pair[1] and pair[1] <= c.polars[1].aoaCritH)
        ad = deflection_pitch(current, inv, c.lim_ail)
        drag_control = M(M(ad, -c.ail_gain), A(M(ip[0x6c//4], F(-.1)), 1.))
        extra[0] = M(drag_control, f[0x79b8//4] if drag_control >= 0. else f[0x79bc//4])
        extra[1] = M(ip[0x78//4] if self.fl_7fa3 else 1., f[0x7ca8//4])
        extra[2] = M(ip[0x80//4], f[0x7cb4//4]);extra[3] = M(ip[0x84//4], f[0x7cb8//4])
        other_drag = A(A(extra[3], extra[1]), A(extra[2], extra[0]))
        # flow(angle)
        working = S(angle, g.incidence);sn = F(sin(M(working, RAD_W)));cs = F(cos(M(working, RAD_W)))
        vx = M(cs, c.speed);vy = M(sn, -c.speed)
        if ip[0x48//4] != 0.:
            wash = M(ip[0x48//4], c.wash_attenuation)
            k = pmax(A(F(M(fabs(vy), -1.5)/pmax(A(wash, vx), F(.2))), 1.), 0.)
            axial = M(wash, k)
            if fabs(axial) < F(1.1920928955078125e-7):axial = 0.
            vx = A(axial, vx)
            if ip[0x68//4] != 0. and not self.fl_8471:
                spin = pmin(M(M(vx, vx), F(.0011)), 1.) if vx >= 0. else 0.
                swirl = M(M(M(k, c.wash_attenuation), spin), ip[0x4c//4])
                if fabs(swirl) < F(1.1920928955078125e-7):swirl = 0.
        dynamic = M(M(A(M(vy, vy), M(vx, vx)), .5), ip[0x44//4])
        taq = M(dynamic, c.tail_area)
        vstab_drag = M(dynamic, c.vstab_drag_factor)
        required = inverse_rotated_cl(&c.tail, DIVF(tail_force, taq), 0., convert, &tail_ok)
        calc_c(&c.tail, required, 0., 0., 1., tc)
        eng = A(M(cs, ip[0x54//4]), M(sn, ip[0x50//4]))
        other_normal = A(M(A(c.parasite_drag, vstab_drag), sn), A(M(taq, S(M(cs, tc[1]), M(sn, tc[0]))), eng))
        required_wing = S(M(ip[1], M(level_scale, ip[0x2c//4])), M(other_normal, ip[0x30//4]))
        for side in range(2):
            calc_c(&c.polars[side], pair[side], working if convert else S(pair[side], g.incidence), 0., 1., pol[side])
            cls[side] = A(pol[side][1], cladd)
        for side in range(2):
            drag[side] = M(A(A(A(f[0x7cc0//4], pol[side][0]), M(ip[0x7c//4], f[0x7cb0//4])), other_drag), c.negative_q_area[side])
            lift[side] = M(M(c.q_area[side], cls[side]), c.cos_dihedral)
        wings[0] = A(drag[1], drag[0]);wings[1] = A(lift[1], lift[0])
        error = S(required_wing, M(A(M(sn, wings[0]), M(cs, wings[1])), ip[0x30//4]))
        k0 = M(A(M(c.polars[0].clToCm1, cls[0]), c.polars[0].clToCm0), c.polar_area_over_span)
        inv0 = F(1./A(M(cls[0], cls[0]), M(pol[0][0], pol[0][0])))
        shiftx0 = M(M(k0, cls[0]), inv0);shifty0 = M(inv0, M(k0, pol[0][0]))
        inv1 = F(1./A(M(cls[1], cls[1]), M(pol[1][0], pol[1][0])))
        k1 = M(inv1, M(A(M(c.polars[1].clToCm1, cls[1]), c.polars[1].clToCm0), c.polar_area_over_span))
        shiftx1 = M(k1, cls[1]);shifty1 = M(k1, pol[1][0])
        mx = M(S(c.moment_wing_x[0], A(shiftx0, ip[0x1c//4])), lift[0])
        mx = A(mx, M(c.tail_lever, tail_force))
        mx = A(mx, M(S(c.moment_wing_x[1], A(shiftx1, ip[0x1c//4])), lift[1]))
        my = A(ip[0x64//4], A(M(vstab_drag, c.vstab_y), M(c.fuse_drag, c.fuse_y)))
        my = A(my, M(S(c.moment_wing_y[0], A(ip[0x20//4], shifty0)), drag[0]))
        my = A(my, M(S(c.moment_wing_y[1], A(shifty1, ip[0x20//4])), drag[1]))
        unmet = S(my, mx)
        if not self.new_balance and self.fl_7c08:
            unmet = A(unmet, M(F(sin(M(fabs(working), RAD_W))), M(A(wings[1], tail_force), .5)))
        td = A(M(positive if command >= 0. else -negative, command), center)
        target = A(DIVF(tail_force, taq), M(c.tail_cl[1 if td < 0. else 0], td))
        required = inverse_rotated_cl(&c.tail, target, working if convert else 0., convert, &tail_final_ok)
        self.flow_inputs(c, ip, &fq, working, sn, cs, angle, 0., axial, swirl,
                         legacy_wake_factor(c.wash_attenuation, A(M(cs, c.speed), axial), c.legacy_aspect)
                         if g.downwash_type == 1 else 0.)
        fq.area = area
        flow_angle = tail_flow(&fq)
        tail_command(flow_angle, required, ip[0x70//4], f[0x6f1c//4], c.area_sensitivity_scale, c.has_sensitivity,
                     center, positive, negative, invert, &proposed, &saturated, &deflection)
        command_error = S(proposed, current)
        if saturated:
            effective = A(S(A(ip[0x70//4], f[0x6f1c//4]), M(deflection, c.tail_gain)), flow_angle)
            calc_c(&c.tail, effective, working if convert else effective, 0., 1., tc)
            actual = tc[1]
            residual = A(M(taq, S(M(c.tail_cl[1 if proposed < 0. else 0], proposed), actual)), tail_force)
            unmet = A(unmet, M(residual, c.tail_lever))
        out.output[0] = working;out.output[1] = current;out.output[2] = roll
        out.success = wing_ok and tail_ok and tail_final_ok and fabs(unmet) < 20. and not saturated
        out.lift_error = error;out.moment_error = unmet;out.command_error = command_error;out.saturated = saturated
        return 0

    # ---------------------------------------------------------------- one-g trim (instructor_trim_equilibrium.solve)
    cdef dict trim_solve(self, const double* ip):
        import numpy as np
        cdef IGeo c
        cdef TrimCorrector tc
        cdef double weight, moment_scale, cl, slope, angle, tail, lift_slope, adjusted, finite32
        cdef double initial[3]
        cdef Result res
        cdef Predictor pred
        cdef int slot, seed, seeded
        cdef const Polar* p
        self.geometry(ip, &c)
        weight = pmax(fabs(ip[0x2c//4]), 1.)
        moment_scale = weight*pmax(fabs(c.tail_lever), 1.)
        p = &c.polars[0]
        cl = weight/pmax(((0.+c.q_area[0])+c.q_area[1])*c.cos_dihedral, 1.)
        slope = p.clLineCoeff*p.clKq
        angle = (cl-p.cl0*p.clKq)/slope if fabs(slope) > 1e-9 else 0.
        angle = pmin(p.aoaCritH, pmax(p.aoaCritL, angle))
        tc = TrimCorrector()
        tc.owner = self;tc.c = c
        for i in range(NIP):tc.ip[i] = ip[i]
        tc.weight = weight;tc.moment_scale = moment_scale
        finite32 = float(np.nextafter(float(np.finfo(np.float32).max), 0.))
        tc.n = 3;tc.m = 3
        tc.lower[0] = -finite32;tc.lower[1] = -finite32/weight;tc.lower[2] = -1.
        tc.upper[0] = finite32;tc.upper[1] = finite32/weight;tc.upper[2] = 1.
        tc.steps[0] = .002;tc.steps[1] = 2e-5;tc.steps[2] = .0002
        tc.scale[0] = 10.;tc.scale[1] = 1.;tc.scale[2] = .2
        tc.step_limit[0] = 8.;tc.step_limit[1] = .3;tc.step_limit[2] = .4
        tc.rscale[0] = 20./weight;tc.rscale[1] = 20./moment_scale;tc.rscale[2] = 2e-6
        tc.has_project = False;tc.has_dep = False
        tc.evaluations = tc.derivative_samples = tc.nominal_comparisons = tc.nominal_restorations = tc.branch_secants = 0
        initial[0] = angle;initial[1] = 0.;initial[2] = 0.
        if fabs(c.tail_lever) > 4e-19:
            seed = tc.lookup(initial)
            tail = tc.memo[seed].auto.moment_error/c.tail_lever/weight
            if not tc.memo[seed].auto.saturated and isfinite(tail) and tc.lower[1] <= tail and tail <= tc.upper[1]:
                initial[1] = tail
                seeded = tc.lookup(initial)
                lift_slope = ((0.+c.q_area[0])+c.q_area[1])*c.cos_dihedral*slope*p.cyMult
                if not tc.memo[seeded].auto.saturated and fabs(lift_slope) > 1e-9:
                    adjusted = angle+tc.memo[seeded].auto.lift_error/lift_slope
                    if isfinite(adjusted):
                        initial[0] = pmin(p.aoaCritH, pmax(p.aoaCritL, adjusted))
        pred.valid = False
        tc.solve(initial, 120, 0, &pred, False, &res)
        if tc.derivative_samples:charge(K_DERIV, tc.derivative_samples)
        if tc.nominal_comparisons:charge(K_NOMCMP, tc.nominal_comparisons)
        if tc.nominal_restorations:charge(K_NOMRES, tc.nominal_restorations)
        if tc.branch_secants:charge(K_SECANT, tc.branch_secants)
        self.trim_solves += 1;self.trim_evaluations += tc.nmemo
        if res.termination == -1:
            from em_trim_numerics import UnevaluableTrial
            raise UnevaluableTrial('Nonfinite stationary residual; correction unresolved')
        slot = tc.lookup(res.x)
        cdef Auto* au = &tc.memo[slot].auto
        r = tc.memo[slot].r
        success = (au.success and abs(au.lift_error) < 20. and abs(au.moment_error) < 20. and abs(au.command_error) < 2e-6
                   and abs(au.output[1]) <= 1. and isfinite(res.x[0]) and isfinite(res.x[1]) and isfinite(res.x[2]))
        from instructor_trim_equilibrium import REVISION
        f32max = np.nextafter(float(np.finfo(np.float32).max), 0.)
        lower = [-f32max, -f32max/weight, -1.];upper = [f32max, f32max/weight, 1.]
        rmax = float(np.max(np.abs(np.asarray([r[0], r[1], r[2]])), initial=0.))
        return dict(success=bool(success), method='shared correction of simultaneous one-g relation',
            equilibrium=dict(lift_error_n=au.lift_error, moment_error_nm=au.moment_error, command_error=au.command_error),
            trim=[au.output[2], au.output[1], 0.],
            one_g_alpha_deg=au.output[0], evaluations=tc.nmemo,
            reason=None if success else 'One-g equilibrium correction unresolved',
            coordinates=[res.x[0], res.x[1], res.x[2]],
            coordinate_names=['predictor_angle_deg', 'tail_force_over_weight', 'elevator_command'],
            search_bounds=[lower, upper],
            search_bound_scope='Finite binary32 predictor states and retained command bound; no coverage proof',
            local_admission=bool(success), coverage_complete=False,
            correction=dict(termination=TERMINATIONS[res.termination], nfev=res.nfev,
                evaluations=res.evaluations, derivative_evaluations=res.evaluations-res.nfev,
                residual_max=rmax,
                return_metric='scaled_linf_then_l2', closure_ratio=res.closure_ratio,
                numerical_target_reached=bool(res.success)),
            revision=REVISION)

    # ---------------------------------------------------------------- the controller
    cdef void pack_inputs(self, double* ip, int mode, double target_angle, double target_acceleration, double body_pitch_rate,
                          const double* angle_bounds, const double* quaternion, const double* world_velocity, double tas,
                          double speed_squared, double mach, const double* wash, const double* force, const double* moment,
                          double gear_fraction) noexcept:
        # instructor_predictor_inputs.pack_pitch_inputs / pack_autotrim_inputs, then unpack_inputs
        cdef double y, w, up, vx, vy, vz, speed, vertical, flap, gear, airbrake, scale
        cdef int i, qg, qa
        for i in range(NIP):ip[i] = 0.
        ip[0] = mode
        ip[1] = F(target_angle);ip[2] = F(target_acceleration)
        ip[3] = 0.
        ip[4] = F(body_pitch_rate);ip[5] = F(angle_bounds[0]);ip[6] = F(angle_bounds[1])
        ip[7] = 0.;ip[8] = 0.;ip[9] = 0.
        ip[10] = 0.
        ip[11] = F(M(self.model.G, F(self.w_mass)))
        y = F(quaternion[1]);w = F(quaternion[3])
        up = A(M(2., A(M(y, y), M(w, w))), -1.)
        vx = world_velocity[0];vy = world_velocity[1];vz = world_velocity[2]
        speed = sqrt(vz*vz+(vx*vx+vy*vy))
        vertical = F(vy/speed) if speed > F_TINY else 0.
        ip[12] = F(pmin(1., pmax(-1., up)));ip[13] = F(pmin(1., pmax(-1., vertical)))
        ip[14] = F(tas);ip[15] = F(speed_squared);ip[16] = F(mach)
        ip[17] = F(density_at_height(F(self.w_height), F(1.225), F(18300.)))
        ip[18] = F(wash[0]);ip[19] = F(wash[1])
        scale = F(self.w_engine_scale)
        for i in range(3):ip[20+i] = F(force[i]*scale)
        for i in range(3):ip[23+i] = F(moment[i])
        ip[26] = 1. if self.w_torque_gyro else 0.
        flap = M(M(F(self.w_flap_blend), .5), A(F(1.), F(1.)))
        qg = <int>pmin(255., pmax(0., <double>(<int>M(F(gear_fraction), 255.))))
        qa = <int>pmin(255., pmax(0., <double>(<int>M(F(0.), 255.))))
        gear = M(M(<double>qg, F(.0019607844)), 2.)
        airbrake = M(M(A(F(1.), F(1.)), F(.0019607844)), <double>qa)
        ip[27] = F(flap);ip[28] = F(self.w_flap_incidence);ip[29] = F(self.w_sweep);ip[30] = F(gear);ip[31] = F(airbrake)
        ip[32] = F(self.w_oil);ip[33] = F(self.w_water);ip[34] = F(self.w_cockpit)

    cdef double rb_rudder_trim(self, double mach, double ias) noexcept:
        cdef double s1[1]
        cdef double lim[3][3]
        cdef bint fl[3]
        cdef double sensitivity, target, positive, center, negative, denominator, numerator, trim
        curve(&self.model.mixers[2].sens_curve, mach, 1, s1)
        sensitivity = M(s1[0], self.model.mixers[2].sensitivity)
        sensitivity = M(sensitivity, self.f[0x18d8//4])
        target = 0.
        if fabs(sensitivity) > F_TINY:
            target = F(S(self.rudder_incidence, F(self.vstab0.cl0/self.vstab0.clLineCoeff))/sensitivity)
        fl[0] = False;fl[1] = False;fl[2] = False
        axis_limits_mix(&self.model.mixers[2], fl, F(self.w_height), ias, lim)
        positive = lim[2][0];center = lim[2][1];negative = lim[2][2]
        denominator = S(positive if target > 0 else negative, center)
        numerator = target if target > 0 else -target
        trim = F(numerator/denominator) if fabs(denominator) > F_TINY else 0.
        return pmin(1., pmax(-1., trim))

    cdef void authority(self, double speed, double elevator_state, double* out) noexcept:
        # primary_controls.authority_ranges(selected_properties(fm), speed, full_loss=True, symmetric, unit scale)
        cdef const Primary* p = &self.sel
        cdef double roll, yaw, rr0, rr1, yr0, yr1, pr0, pr1, t
        speed = F(speed)
        roll = power_limit(p, speed, 0, 0);yaw = power_limit(p, speed, 2, 0)
        rr0 = pmax(-1., M(-roll, 1.));rr1 = roll
        yr0 = pmax(-1., M(-yaw, 1.));yr1 = yaw
        pr0 = -power_limit(p, speed, 1, 0);pr1 = power_limit(p, speed, 1, 1)
        if fabs(S(p.eff[1][0], p.eff[1][1])) < F(1e-6) and fabs(S(p.power[1][0], p.power[1][1])) < F(1e-6):
            pr0 = pmax(-1., M(-pr1, 1.))
        if not p.strong:
            if p.invert_elevator != (F(elevator_state) >= 0.):
                pr0 = yr0;pr1 = yr1
            t = A(M(pr0, .5), -.5);pr1 = A(M(pr1, .5), .5);pr0 = t
        out[0] = M(rr0, F(1.));out[1] = M(rr1, F(1.))
        out[2] = M(pr0, F(1.));out[3] = M(pr1, F(1.))
        out[4] = M(yr0, F(1.));out[5] = M(yr1, F(1.))

    cdef dict effective_limits(self, value, phase, double speed_unused):
        """The Python-record entry (verify mode, Python events): inputs from the value record."""
        cdef IIn inp
        cdef InstrRes r
        cdef int i
        air = phase['air']
        inp.tas = air['tas'];inp.speed_squared = air['speed_squared'];inp.mach = air['mach'];inp.ias_u = air['ias_u']
        wa = phase['history']['wing_aoa']
        inp.wing_angles[0] = wa[0];inp.wing_angles[1] = wa[1]
        if value['flaps'] != self.flaps_value:raise InstructorUnsupported('flap input')
        inp.gear_fraction = value.get('gear', 0.)
        commands = value['allocation']['commands']
        for i in range(3):inp.delivered[i] = commands[i]
        inp.longitudinal = value['velocity'][0]
        kin = value['kinematic']
        for i in range(3):inp.world_acc[i] = kin['world_acceleration'][i]
        geom = value['geometry']
        for i in range(4):inp.quaternion[i] = geom['quaternion'][i]
        inp.pitch_rate = float(geom['omega'][2])
        inp.speed = float(value['speed'])
        prop = value.get('propulsion')
        inp.has_prop = prop is not None
        if prop is not None:
            for i in range(3):inp.force[i] = prop['force'][i];inp.moment[i] = prop['moment'][i]
            inp.wash_raw[0] = prop['wash'][0];inp.wash_raw[1] = prop['wash'][1]
        elif self.model.prop:raise InstructorUnsupported('propeller without propulsion record')
        self.effective_core(&inp, &r)
        return self.effective_dict(&r, commands[1])

    cdef int eval_inputs(self, Eval* e, const PropSample* ps, IIn* inp) except -1:
        """Inputs of the Instructor admission of a native state (value_record(..., complete=True) fields)."""
        cdef int i
        ensure_complete(self.model, e)
        inp.tas = e.air.tas;inp.speed_squared = e.air.speed_squared;inp.mach = e.air.mach;inp.ias_u = e.air.ias_u
        inp.wing_angles[0] = e.frame.history.aoa[0];inp.wing_angles[1] = e.frame.history.aoa[1]
        if self.model.flaps != self.flaps_value:raise InstructorUnsupported('flap input')
        inp.gear_fraction = self.model.gear
        for i in range(3):inp.delivered[i] = e.alloc.commands[i]
        inp.longitudinal = e.velocity[0]
        for i in range(3):inp.world_acc[i] = e.kin.world_acceleration[i]
        for i in range(4):inp.quaternion[i] = e.geom.quaternion[i]
        inp.pitch_rate = e.geom.omega[2]
        inp.speed = e.speed
        inp.has_prop = ps != NULL
        if ps != NULL:
            for i in range(3):inp.force[i] = ps.frame.force[i];inp.moment[i] = ps.frame.moment[i]
            inp.wash_raw[0] = ps.frame.wash[0];inp.wash_raw[1] = ps.frame.wash[1]
        elif self.model.prop:raise InstructorUnsupported('propeller without propulsion record')
        return 0

    cdef int eval_limits(self, Eval* e, const PropSample* ps, InstrRes* r) except -1:
        """limits() of a native state without building the value record (lean admission)."""
        cdef IIn inp
        self.eval_inputs(e, ps, &inp)
        self.calls += 1
        self.effective_core(&inp, r)
        return 0

    cdef dict effective_dict(self, const InstrRes* r, double delivered_pitch):
        cdef int i
        angle_margin = r.angle_margin;mechanical = r.mechanical
        margins = {'Instructor effective AoA': angle_margin, 'control authority with auto trim': mechanical}
        bounds = [[r.bounds[i][0], r.bounds[i][1]] for i in range(3)]
        demands = [dict(target_angle=r.demand_angle[i], target_acceleration=r.demand_acc[i]) for i in range(2)]
        balances = [dict(acceleration=r.bal[i].acceleration, tail_force=r.bal[i].tail_force,
                         zero_acceleration_tail_force=r.bal[i].zero_force, tail_angle_deg=r.bal[i].tail_angle,
                         tail_angle_bounds_deg=[self.tail_bounds[0], self.tail_bounds[1]], flow_angle_deg=r.bal[i].flow,
                         direct_predictor=bool(r.bal[i].direct)) for i in range(2)]
        success = r.success
        return dict(converged=success or r.independent, margin=pmin(angle_margin, mechanical), margins=margins,
            authority_mode=self.authority_mode,
            envelope_margin=angle_margin, pitch_margin=angle_margin, angle_margin=angle_margin,
            limiting=min(margins, key=margins.get), control_bounds=bounds,
            auto_trim=[r.trim[0], r.trim[1], r.trim[2]] if success else None,
            sticks=[r.sticks[0], r.sticks[1], r.sticks[2]] if success else None, static_trim=self.autos[r.auto_index],
            control_authority_trim_independent=r.independent,
            delivered_pitch=delivered_pitch,
            required_pitch=delivered_pitch, angle_limits_deg=[r.low, r.high],
            effective_angle_limits_deg=[r.effective[0], r.effective[1]], adjusted_wing_angles_deg=[r.adjusted[0], r.adjusted[1]],
            native_angle_rate_rad_s=r.rate, model='effective AoA schedule with reduced moment balance',
            rate_demands=demands, reduced_balances=balances, history_independent=True, exact_native_controller=False,
            overload_policy='transient reserve omitted; physical strength retained')

    cdef dict limits_dict(self, const InstrRes* r, double delivered_pitch):
        """limits() result of one native state (one phase) from its stored admission."""
        result = self.effective_dict(r, delivered_pitch)
        a = result['adjusted_wing_angles_deg']
        result['adjusted_wing_angles_deg'] = [min(a), max(a)]
        result.update(model='Steady effective AoA schedule (approximation)',
                      model_revision='steady-effective-aoa-fixed-autotrim-v9', pitch_predictor_recheck=False)
        result['authority_revision'] = self.authority_revision
        return result

    cdef int effective_core(self, const IIn* inp, InstrRes* r) except -1:
        import math
        cdef double tas, speed_squared, mach, ias_u, longitudinal, rho, ias, sensitivity, rate, kp, mechanical
        cdef double wing_angles[2]
        cdef double delivered[3]
        cdef double quaternion[4]
        cdef double world_acc[3]
        cdef double world_velocity[3]
        cdef double force[3]
        cdef double moment[3]
        cdef double wash[2]
        cdef double adjusted[2]
        cdef double limits[2]
        cdef double ranges[6]
        cdef double ip[NIP]
        cdef double ipa[NIP]
        cdef double zero2[2]
        cdef double ident_q[4]
        cdef double unit_v[3]
        cdef double fixed_trim[3]
        cdef double demand_angle[2]
        cdef double demand_acc[2]
        cdef double margins_angle[2]
        cdef Balance bal[2]
        cdef Allocation alloc
        cdef double a_r, e_r, cos_dihedral, q, low, high, factor, scale, vv, valid_lo, lower, upper, normalized, kd, damping
        cdef double x0, y0, x1, y1, kpd, mass_scale, effective_area, rudder_trim, pitch_rate, gear_fraction
        cdef bint valid, independent
        cdef int i
        tas = inp.tas;speed_squared = inp.speed_squared;mach = inp.mach;ias_u = inp.ias_u
        wing_angles[0] = inp.wing_angles[0];wing_angles[1] = inp.wing_angles[1]
        gear_fraction = inp.gear_fraction
        for i in range(3):delivered[i] = inp.delivered[i]
        longitudinal = inp.longitudinal
        for i in range(3):world_acc[i] = inp.world_acc[i]
        for i in range(4):quaternion[i] = inp.quaternion[i]
        pitch_rate = inp.pitch_rate
        world_velocity[0] = inp.speed;world_velocity[1] = 0.;world_velocity[2] = 0.
        if inp.has_prop:
            for i in range(3):force[i] = inp.force[i];moment[i] = inp.moment[i]
            wash[0] = F(inp.wash_raw[0]);wash[1] = F(inp.wash_raw[1])
        else:
            self.model.vectors(longitudinal, force, moment)
            wash[0] = wash[1] = 0.
        snapshot1 = -delivered[1] if self.invert_elevator else delivered[1]
        # fixed_source
        rho = density_at_height(F(self.w_height), F(1.225), F(18300.))
        ias = F(longitudinal*F(sqrt(F(rho/self.rho0))))
        self.authority(ias, snapshot1, ranges)
        curve(&self.model.mixers[0].sens_curve, mach, 1, &sensitivity)
        sensitivity = M(sensitivity, self.model.mixers[0].sensitivity)
        cdef int wk = self.fixed_mach(mach)
        cdef Polar p = self.mw[wk].wing
        rate = angle_rate(world_velocity, world_acc, quaternion, pitch_rate)
        rudder_trim = self.rb_rudder_trim(mach, ias_u) if self.rudder_trim_needed else 0.
        # predicted_wing_angles(...)['adjusted']
        a_r = M(M(F(delivered[0]), F(sensitivity)), F(14.))
        e_r = M(M(F(self.elevator_wing_aoa), F(delivered[1])), F(-18.))
        adjusted[0] = S(F(wing_angles[0]), A(e_r, a_r));adjusted[1] = A(F(wing_angles[1]), S(a_r, e_r))
        # angle_targets(..., mode_lane=True, overload_enabled=False)
        factor = self.crit_mult[0]
        low = M(p.aoaCritL, factor);high = M(p.aoaCritH, factor)
        base_limits = [low, high]
        cos_dihedral = F(cos(M(F(self.dihedral), F(.01745329238474369))))
        q = M(M(.5, F(rho)), F(speed_squared))
        if self.limit_load_factor:
            effective_area = S(M(cos_dihedral, F(self.wing_area)), M(A(self.tail_area_pair[0], self.tail_area_pair[1]), .5))
            mass_scale = F(F(self.w_mass)/pmax(1., M(q, effective_area)))
            for i in range(2):
                vv = inverse_cl(&p, M(M(F(self.load_factor_limit[i]), self.model.G), mass_scale), &valid)
                if valid:
                    if i:high = pmin(high, vv)
                    else:low = pmax(low, vv)
        limits[0] = low;limits[1] = high
        # pitch_demands
        kpd = F(M(F(1.), F(self.reference_speed))/pmax(5., F(tas)))
        kd = F(F(self.reference_speed)/pmax(5., F(ias)));kd = M(M(kd, kd), F(12.))
        lower = pmin(adjusted[0], adjusted[1]);upper = pmax(adjusted[0], adjusted[1])
        normalized = DIVF(pmax(S(upper, p.aoaLineH), S(p.aoaLineL, lower)), M(S(p.aoaLineH, p.aoaLineL), .5))
        x0 = self.pitch_coeff[0];y0 = self.pitch_coeff[1];x1 = self.pitch_coeff[2];y1 = self.pitch_coeff[3]
        damping = M(M(kd, F(rate)), remap(normalized, x0, x1, y0, y1))
        demand_angle[0] = pmax(lower, low);demand_acc[0] = S(M(-pmax(S(low, lower), -10.), kpd), damping)
        demand_angle[1] = pmin(upper, high);demand_acc[1] = S(M(-pmin(S(high, upper), 10.), kpd), damping)
        kp = self.reference_speed/pmax(5., tas)
        zero2[0] = zero2[1] = 0.
        for i in range(2):
            self.pack_inputs(ip, 1, demand_angle[i], demand_acc[i], pitch_rate, self.tail_bounds, quaternion, world_velocity,
                             tas, speed_squared, mach, wash, force, moment, gear_fraction)
            self.required_acceleration(ip, delivered[1], &bal[i])
        margins_angle[0] = (demand_acc[0]-bal[0].acceleration)/kp
        margins_angle[1] = (bal[1].acceleration-demand_acc[1])/kp
        independent = True
        for i in range(3):
            if self.alloc_props.trim_available[i] and not (ranges[2*i] == -1. and ranges[2*i+1] == 1.):
                independent = False
        if independent and self.direct_mode:
            auto = self.skipped_auto
        else:
            ident_q[0] = ident_q[1] = ident_q[2] = 0.;ident_q[3] = 1.
            unit_v[0] = 1.;unit_v[1] = unit_v[2] = 0.
            self.pack_inputs(ipa, 0, 1., 0., 0., zero2, ident_q, unit_v, tas, speed_squared, mach, wash, force, moment,
                             gear_fraction)
            auto = self.static_trim(ipa, rudder_trim)
        if auto['success']:
            t = auto['trim']
            for i in range(3):fixed_trim[i] = t[i]
        else:
            fixed_trim[0] = fixed_trim[1] = fixed_trim[2] = 0.
        command_allocation(&self.alloc_props, delivered, ranges, False, self.trim_limit, fixed_trim, &alloc)
        mechanical = 0.
        for i in range(3):
            vv = pmin(delivered[i]-alloc.bounds[i][0], alloc.bounds[i][1]-delivered[i])
            mechanical = vv if i == 0 else pmin(mechanical, vv)
        r.angle_margin = pmin(margins_angle[0], margins_angle[1])/10.
        r.mechanical = mechanical
        r.effective[0] = pmin(adjusted[0], adjusted[1])-margins_angle[0];r.effective[1] = pmax(adjusted[0], adjusted[1])+margins_angle[1]
        for i in range(3):
            r.bounds[i][0] = alloc.bounds[i][0];r.bounds[i][1] = alloc.bounds[i][1]
            r.trim[i] = alloc.trim[i];r.sticks[i] = alloc.sticks[i]
        for i in range(2):
            r.demand_angle[i] = demand_angle[i];r.demand_acc[i] = demand_acc[i];r.bal[i] = bal[i]
            r.adjusted[i] = adjusted[i]
        r.success = bool(auto['success'])
        r.independent = independent
        r.converged = r.success or independent
        r.low = low;r.high = high;r.rate = rate
        r.auto_index = self.auto_index(auto)
        return 0

    cdef int auto_index(self, auto) except -1:
        key = id(auto)
        index = self.auto_ids.get(key)
        if index is None:
            index = len(self.autos);self.autos.append(auto);self.auto_ids[key] = index
        return index

    cdef object static_trim(self, double* ipa, double rudder_trim):
        cdef double key[NIP+1]
        cdef int i
        for i in range(NIP):key[i] = ipa[i]
        key[NIP] = rudder_trim
        k = (<char*>key)[:sizeof(double)*(NIP+1)]
        cached = self.trim_cache.get(k)
        if cached is not None:
            return cached
        if self.python_trim:
            corrected = self.py_static_trim(ipa)
        else:
            corrected = self.trim_solve(ipa)
        result = dict(corrected, trim=list(corrected['trim']))
        result['trim'][2] = rudder_trim
        if len(self.trim_cache) >= 2048:self.trim_cache.clear()
        self.trim_cache[k] = result
        return result

    cdef object py_static_trim(self, double* ipa):
        # verification: the baseline one-g correction on the same predictor inputs
        from instructor_trim_equilibrium import solve
        from instructor_pitch_predictor import unpack_inputs
        ip = {off: ipa[off//4] for off in range(4, 0x8c, 4)}
        for off in (0xc, 0x28, 0x68):ip[off] = bool(ipa[off//4])
        ip[0] = int(ipa[0])
        return solve(self.solver.model, ip, self.predictor)

    def limits(self, value, speed=None):
        """instructor_steady_aoa.controller_limits(solver, value, speed)."""
        if value.get('instructor') is not None:
            return value['instructor']
        if value.get('search_only', False):
            # Boundary events can inspect a residual-only trial before admission;
            # the baseline completes the record on demand (same mutation here).
            from em_operating import complete_reporting
            complete_reporting(self.solver, value)
        if value.get('kinematic') is None:
            raise InstructorUnsupported('incomplete record')
        phases = value.get('phase_results') or [value['result']]
        self.calls += 1
        sources = {}
        results = []
        for phase in phases:
            air = phase['air'];angles = phase['history']['wing_aoa']
            key = (air['tas'], air['speed_squared'], air['mach'], air['ias_u'], *angles)
            if key not in sources:
                sources[key] = self.effective_limits(value, phase, 0.)
            results.append(sources[key])
        result = dict(min(results, key=lambda item: item['margin']))
        result['converged'] = all(item['converged'] for item in results)
        result['adjusted_wing_angles_deg'] = [
            min(min(item['adjusted_wing_angles_deg']) for item in results),
            max(max(item['adjusted_wing_angles_deg']) for item in results)]
        result.update(model='Steady effective AoA schedule (approximation)',
                      model_revision='steady-effective-aoa-fixed-autotrim-v9', pitch_predictor_recheck=False)
        result['authority_revision'] = self.authority_revision
        value['instructor'] = result
        return result


