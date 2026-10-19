# S2 native frame model: typed transliteration of the baseline runtime's
# polar_f32, wing_model, wing_stages, tail_model, component_stages, downwash,
# component_assembly, body_dynamics, air_state, aero_helpers and
# control_mixer/primary_controls kernels (jet path, zero health damage).
#
# Every Python f32()/add()/sub()/mul() is a binary32 rounding here (F/A/S/M),
# every other operation is binary64 in the same order as the Python source,
# and min/max keep Python's first-argument tie rule (pmin/pmax). libm calls
# are the same CRT functions CPython's math module calls.

from libc.math cimport sin, cos, tan, sqrt, atan2, asin, acos, exp, fmod, pow, fabs, copysign, trunc, isfinite, frexp, ldexp, M_PI

cdef extern from *:
    """
    static double s2_f32(double x) { return (double)(float)x; }
    """
    double s2_f32(double x) nogil

cdef inline double F(double x) noexcept nogil:
    return s2_f32(x)

cdef inline double A(double a, double b) noexcept nogil:
    return s2_f32(a+b)

cdef inline double S(double a, double b) noexcept nogil:
    return s2_f32(a-b)

cdef inline double M(double a, double b) noexcept nogil:
    return s2_f32(a*b)

cdef inline double pmax(double a, double b) noexcept nogil:
    # Python max(a, b): the first argument wins ties and incomparable values.
    return b if b > a else a

cdef inline double pmin(double a, double b) noexcept nogil:
    return b if b < a else a

cdef double TINY = 4e-19  # f32(4e-19) is computed in init (binary32 constant)
cdef double F_TINY
cdef double DEG_W   # wing/tail DEG = f32(57.2957763671875)
cdef double RAD_W   # f32(0.01745329238474369)
cdef double PI_F    # f32(math.pi)
cdef double DEG_TO_RAD = M_PI/180.0
cdef double RAD_TO_DEG = 180.0/M_PI


cdef struct Polar:
    double cl0, cd0, indCoeff, clLineCoeff, cyCritH, cyCritL, aoaCritH, aoaCritL
    double aoaLineH, aoaLineL, parabCyCoeffH, parabCyCoeffL, aerCenterOffset
    double clToCm0, clToCm1, parabAngle, declineCoeff, maxDistAng, cdAfterCoeff
    double clAfterCritL, clAfterCritH, kq, clKq, cyMult


# ---------------------------------------------------------------- polar_f32
cdef inline double SINF(double x) noexcept nogil:
    return F(sin(x))

cdef double calc_cl(const Polar* p, double a) noexcept nogil:
    cdef double s, crit, cy, after, da, x, maxang, sa, pa, h, den, coeff, local_sign, local_angle, one, two, correction, wave, parab
    cdef bint positive
    a = F(a)
    if p.aoaLineL <= a and a <= p.aoaLineH:
        return A(M(M(a, p.clLineCoeff), p.cyMult), p.cl0)
    positive = A(a, F(.01)) >= p.aoaLineH
    s = 1. if positive else -1.
    if positive:
        crit = p.aoaCritH; cy = p.cyCritH; after = p.clAfterCritH; parab = p.parabCyCoeffH
    else:
        crit = p.aoaCritL; cy = p.cyCritL; after = p.clAfterCritL; parab = p.parabCyCoeffL
    da = S(a, crit)
    if M(da, s) <= 0.:
        x = S(crit, a)
        return S(cy, M(M(M(x, x), s), parab))
    maxang = pmax(40., p.maxDistAng)
    sa = M(s, a)
    if sa <= maxang:
        if sa <= p.maxDistAng:
            pa = p.parabAngle
            if M(da, s) < pa:
                return S(cy, M(M(M(da, da), s), p.declineCoeff))
            h = M(SINF(M(F(.039269909262657166), p.maxDistAng)), after)
            x = S(M(S(p.maxDistAng, pa), s), crit)
            den = M(x, x)
            coeff = F(S(S(cy, h), M(M(M(pa, pa), s), p.declineCoeff))/den) if den > F_TINY else 0.
            x = S(M(p.maxDistAng, s), a)
            return A(M(M(x, x), coeff), h)
        return M(after, SINF(M(M(a, F(.039269909262657166)), s)))
    if sa <= 140.:
        local_sign = -s if sa > 90. else s
        local_angle = S(180., sa) if sa > 90. else sa
        one = SINF(M(F(.039269909262657166), maxang))
        two = SINF(S(F(-1.5707963705062866), M(pmax(0., A(maxang, -40.)), F(.03141592815518379))))
        correction = A(two, one)
        correction = M(A(M(A(M(sa, F(-.01)), F(.3999999761581421)), correction), correction), s)
        wave = M(SINF(A(M(local_angle, F(.03141592815518379)), F(.3141592741012573))), local_sign)
        return M(A(wave, correction), M(after, s))
    return M(M(M(s, s), after), SINF(A(M(sa, F(.039269909262657166)), F(-7.0685834884643555))))


cdef double calc_cd(const Polar* p, double a) noexcept nogil:
    cdef double line, delta, cd, bound
    a = F(a)
    line = A(M(p.clLineCoeff, a), p.cl0)
    delta = S(a, p.aoaCritH if a >= 0. else p.aoaCritL)
    if a < 0.:
        delta = -delta
    cd = A(A(M(M(line, line), p.indCoeff), p.cd0), M(delta, p.cdAfterCoeff) if delta >= 0. else 0.)
    bound = A(M(fabs(SINF(M(a, F(.01745329238474369)))), p.cyCritH), F(.15))
    return pmin(cd, bound)


cdef void calc_c(const Polar* p, double a, double angle, double cl_add, double cd_coeff, double* out) noexcept nogil:
    cdef double cd, cl, radians, sn, cs
    cd = M(calc_cd(p, a), F(cd_coeff))
    cl = A(calc_cl(p, a), F(cl_add))
    radians = M(F(angle), F(.01745329238474369))
    sn = SINF(radians)
    cs = F(cos(radians))
    out[0] = M(S(M(cs, cd), M(cl, sn)), p.kq)
    out[1] = M(A(M(cs, cl), M(cd, sn)), p.clKq)


# ---------------------------------------------------------------- wing_model
cdef struct WingGeom:
    double span, area
    double areas[2][4]
    double arm[3]
    double flaps_shift[2]
    double gear_shift[2]
    double airbrakes_shift[2]
    double elevon_shift[2]
    double incidence, dihedral, sine_aos, v_focus, aoa_shift
    int n_shift_add
    double shift_add_x[10]
    double shift_add_y[10]
    bint shift_constant
    bint use_spin_loss
    double spin_loss[2]
    double strength_force[2]
    double strength_ias, strength_mach


cdef struct DeviceDrag:
    double gear, central_gear, radiator, oil_radiator, fuselage, airbrake, bomb_bay


cdef inline double wdiv(double a, double b) noexcept nogil:
    # wing_model.div / tail_model.guarded_div
    return F(a/b) if fabs(b) > F_TINY else 0.0


cdef double lookup(const WingGeom* g, double x) noexcept nogil:
    cdef int i
    if g.n_shift_add == 0:
        return 0.0
    if x <= g.shift_add_x[0]:
        return g.shift_add_y[0]
    for i in range(g.n_shift_add-1):
        if x <= g.shift_add_x[i+1]:
            return A(g.shift_add_y[i], M(S(g.shift_add_y[i+1], g.shift_add_y[i]),
                     M(S(x, g.shift_add_x[i]), wdiv(1., S(g.shift_add_x[i+1], g.shift_add_x[i])))))
    return g.shift_add_y[g.n_shift_add-1]


cdef void base_points(const WingGeom* g, const Polar* polars, const double* previous_aoa, double beta,
                      const double* flaps, const double* gear, const double* airbrake, double pitch,
                      double* out) noexcept nogil:
    # out: 2 x 3 (health = 1, damage_arm = 0, modifier_x = 0)
    cdef double sn, sv, old, lo, hi, excess, blend, flap, x, y, first, armz, z
    cdef int i
    sn = F(sin(M(F(beta), RAD_W)))
    sv = F(sin(M(g.dihedral, RAD_W)))
    for i in range(2):
        old = F(previous_aoa[i]); lo = polars[i].aoaCritL; hi = polars[i].aoaCritH
        excess = pmax(S(lo, old), S(old, hi))
        blend = pmin(1., pmax(0., pmin(excess*.3, fabs(old)*.0111)))
        flap = pmin(F(flaps[i])*3., 1.)
        x = A(polars[i].aerCenterOffset, g.arm[0]) + 0.
        x += (1.-blend)*g.flaps_shift[0]*flap
        x += M(g.airbrakes_shift[0], F(airbrake[i]))
        x += M(g.gear_shift[0], F(gear[i]))
        x += g.aoa_shift*blend
        x += M(fabs(F(pitch)), g.elevon_shift[0])
        x += lookup(g, excess)
        y = M(g.airbrakes_shift[1], F(airbrake[i])) + g.arm[1]
        y += M(g.gear_shift[1], F(gear[i]))
        y += M(F(pitch), g.elevon_shift[1])
        y += g.flaps_shift[1]*flap
        first = A(M(S(1., F(1.)), F(0.)), 1.)
        armz = M(A(M(S(F(1.), 1.), F(0.)), first), g.arm[2])
        z = M(A(M(g.sine_aos, sn), F(1.3 if i == 0 else -1.3)), armz)
        out[3*i] = -F(x)
        out[3*i+1] = S(M(M(fabs(z), sv), g.v_focus), F(y))
        out[3*i+2] = z


cdef struct Flow:
    double flow[3]
    double alpha, reference_angle, speed_squared


cdef void local_inputs(int side, const double* position, const double* cog, const double* velocity,
                       const double* omega, double dihedral, double engine_x, double engine_swirl,
                       Flow* out) noexcept nogil:
    cdef double v[3]
    cdef double w[3]
    cdef double r[3]
    cdef double sn, cs, y, z, uy, rz, uz, axial, rotation_x, rotation_y, k, ux, swirl, local_y, reference_y
    cdef int i
    for i in range(3):
        v[i] = F(velocity[i]); w[i] = F(omega[i]); r[i] = S(F(position[i]), F(cog[i]))
    sn = F(sin(M(F(dihedral), RAD_W))); cs = F(cos(M(F(dihedral), RAD_W)))
    # disturbance is zero: add(v, f32(0.)) keeps v (including its sign of zero rules)
    y = A(v[1], 0.); z = A(v[2], 0.)
    if side == 0:
        uy = S(M(cs, y), M(sn, z))
    else:
        uy = A(M(cs, y), M(sn, z))
    rz = S(M(r[0], w[1]), M(w[0], r[1]))
    if side == 0:
        uz = A(A(rz, M(sn, y)), M(cs, z))
    else:
        uz = A(rz, S(M(cs, z), M(sn, y)))
    axial = A(v[0], 0.)
    rotation_x = S(M(r[1], w[2]), M(r[2], w[1]))
    rotation_y = S(M(r[2], w[0]), M(r[0], w[2]))
    k = .07000000029802322 if side == 0 else .07
    ux = F(axial+engine_x*k)
    out.flow[0] = A(ux, rotation_x)
    out.flow[1] = A(uy, rotation_y)
    out.flow[2] = uz
    swirl = engine_swirl*k*(1. if side == 0 else -1.)
    local_y = F(out.flow[1]+swirl)
    reference_y = F(uy+engine_swirl*.07000000029802322*(1. if side == 0 else -1.))
    if out.flow[0] != 0. or local_y != 0.:
        out.alpha = M(F(atan2(local_y, out.flow[0])), -DEG_W)
    else:
        out.alpha = -0.
    if ux != 0. or reference_y != 0.:
        out.reference_angle = M(F(atan2(reference_y, ux)), DEG_W)
    else:
        out.reference_angle = 0.
    out.speed_squared = A(M(out.flow[1], out.flow[1]), A(M(out.flow[2], out.flow[2]), M(out.flow[0], out.flow[0])))


cdef struct Coef:
    double cx, cy, cy_reference, cy_add, aoa, reference_aoa, raw_cl


cdef void coefficients(const Polar* p, const Flow* flow, const double* control, double incidence, double flap,
                       double body_aoa, bint convert_aoa, const DeviceDrag* e, Coef* out) noexcept nogil:
    cdef double c[5]
    cdef double normal[2]
    cdef double ref[2]
    cdef double drag[2]
    cdef double health, shift, rotation, angle, reference, rot, dragrot, local, common, total, cdadd
    cdef int i
    for i in range(5):
        c[i] = F(control[i])
    health = F(1.)
    shift = M(M(A(c[4], c[1]), health), A(M(F(flap), F(-.1)), 1.))
    rotation = A(flow.alpha, shift)
    angle = A(rotation, F(incidence))
    reference = A(S(shift, flow.reference_angle), F(incidence))
    rot = F(body_aoa) if convert_aoa else rotation
    calc_c(p, angle, rot, 0., 1., normal)
    normal[0] = F(normal[0]); normal[1] = F(normal[1])
    calc_c(p, reference, S(reference, F(incidence)), 0., 1., ref)
    ref[0] = F(ref[0]); ref[1] = F(ref[1])
    dragrot = F(body_aoa) if convert_aoa else S(angle, F(incidence))
    calc_c(p, angle, dragrot, 0., 1., drag)
    drag[0] = F(drag[0]); drag[1] = F(drag[1])
    local = A(e.gear, M(c[3], health))
    common = A(e.radiator, e.central_gear)
    common = A(e.oil_radiator, common)
    total = A(local, common)
    total = A(total, e.fuselage)
    total = A(e.airbrake, total)
    total = A(total, e.bomb_bay)
    cdadd = M(total, p.kq)
    out.cx = A(drag[0], cdadd)
    out.cy = normal[1]
    out.cy_reference = ref[1]
    out.cy_add = M(c[2], health)
    out.aoa = angle
    out.reference_aoa = reference
    out.raw_cl = F(calc_cl(p, angle))


cdef struct Post:
    double xy[2][2]
    double spin, gyroscopic_scale, vertical_area_scale, vertical_control_scale


cdef void postprocess_xy(double xy[2][2], const double* aoa, const double* crit, double body_q, double dt,
                         double height, double ground_length, double spin, double spin_increment,
                         double spin_cap_input, double yaw_command, double yaw_rate, bint use_spin_loss,
                         double spin_agl, const double* spin_loss, Post* out) noexcept nogil:
    # ordering=False, rudder_health=1
    cdef double threshold, maxaoa, delta, growth, cap, d, upper, s, h, limit, gain
    cdef double positive[2]
    cdef double negative[2]
    cdef int high, low, i, j
    for i in range(2):
        for j in range(2):
            out.xy[i][j] = xy[i][j]
    spin = F(spin)
    out.gyroscopic_scale = 1.; out.vertical_area_scale = 1.; out.vertical_control_scale = 1.
    threshold = M(A(F(crit[0]), F(crit[1])), .5)
    maxaoa = pmax(aoa[0], aoa[1])
    if maxaoa <= threshold:
        spin = 0.
    else:
        delta = S(maxaoa, threshold)
        growth = pmin(1., pmax(0., M(delta, F(.1))))
        cap = A(M(F(spin_cap_input), F(.4)), F(1.2))
        spin = pmin(A(spin, M(M(F(spin_increment), F(dt)), growth)), cap)
        if fabs(yaw_command) > .5 and F(yaw_command)*yaw_rate > 0.:
            spin = pmax(0., A(spin, M(M(F(-.3), F(dt)), growth)))
        d = pmin(15., pmax(1., M(delta, F(.8))))
        out.gyroscopic_scale = pmax(1.-delta*.2, .2)/d
        out.vertical_control_scale = A(M(d, F(.035)), 1.)
        upper = M(A(F(crit[0]), F(crit[1])), .75)
        if maxaoa >= upper:
            out.vertical_area_scale = F(.3)
        else:
            out.vertical_area_scale = A(1., wdiv(M(S(maxaoa, threshold), S(F(.3), 1.)), S(upper, threshold)))
        if use_spin_loss:
            s = M(M(F(body_q), d), spin)*pmin(1., pmax(0., M(F(spin_agl), F(.1))))
            positive[0] = s*.019999999552965164*F(spin_loss[0])
            positive[1] = s*.019999999552965164*F(spin_loss[1])
            negative[0] = s*.25*F(spin_loss[0])
            negative[1] = s*.10000000149011612*F(spin_loss[1])
            if aoa[0] > aoa[1]:
                high = 0; low = 1
            else:
                high = 1; low = 0
            for j in range(2):
                out.xy[low][j] = F(out.xy[low][j]+positive[j])
                out.xy[high][j] = F(out.xy[high][j]-negative[j])
    out.spin = spin
    h = pmax(height, 0.)
    limit = F(ground_length)*.4
    if h < limit:
        gain = (h*-.2)/limit+1.2
        for i in range(2):
            for j in range(2):
                out.xy[i][j] = F(out.xy[i][j]*gain)


cdef struct Wing:
    double forces[2][3]
    double points[2][3]
    double base[2][3]
    Flow flow[2]
    Coef coef[2]
    double q[2]
    double areas[2]
    double blend
    double cy[2]
    Post post


cdef void wing_evaluate(const WingGeom* g, const Polar* polars, const double* velocity, const double* omega,
                        const double* cog, const double* wc, double density, double dt, const double* previous_aoa,
                        double beta, double body_aoa, double flap_blend, double gear_q, double airbrake_q,
                        double pitch, const DeviceDrag* extra_cd, double inertia_x, double height,
                        double ground_length, bint convert_aoa, double spin, double spin_increment,
                        double spin_cap_input, double yaw_command, double spin_agl, double engine_x,
                        double engine_swirl, Wing* out) noexcept nogil:
    # wc: 2 x 5 aileron mix outputs (left, right)
    cdef double flaps[2]
    cdef double gear[2]
    cdef double airbrake[2]
    cdef double sv, cv, sb, cb, delta, omega0, ratio, chord, denom, inv, k, x, y, body_q
    cdef double weights[2]
    cdef double drag[2]
    cdef double lift[2]
    cdef double xy[2][2]
    cdef double aoa[2]
    cdef double crit[2]
    cdef double z[2]
    cdef double bv[3]
    cdef int i, j
    flaps[0] = flaps[1] = flap_blend
    gear[0] = gear[1] = gear_q
    airbrake[0] = airbrake[1] = airbrake_q
    base_points(g, polars, previous_aoa, beta, flaps, gear, airbrake, pitch, &out.base[0][0])
    for i in range(2):
        local_inputs(i, &out.base[i][0], cog, velocity, omega, g.dihedral, engine_x, engine_swirl, &out.flow[i])
        coefficients(&polars[i], &out.flow[i], &wc[5*i], g.incidence, flaps[i], body_aoa, convert_aoa, &extra_cd[i], &out.coef[i])
        # effective_areas with full health
        out.areas[i] = A(A(M(g.areas[i][2], 1.), A(M(g.areas[i][1], 1.), M(g.areas[i][0], 1.))), M(A(1., -1.0), g.areas[i][3]))
        out.q[i] = M(out.flow[i].speed_squared, M(F(density), .5))
    sv = F(sin(M(g.dihedral, RAD_W))); cv = F(cos(M(g.dihedral, RAD_W)))
    sb = F(sin(M(F(beta), RAD_W))); cb = fabs(F(cos(M(F(beta), RAD_W))))
    for i in range(2):
        weights[i] = wdiv(M(M(M(out.q[i], out.areas[i]), cv), out.base[i][2]), F(inertia_x))
    # coupled_points
    delta = A(M(S(out.coef[0].cy, out.coef[0].cy_reference), weights[0]),
              M(S(out.coef[1].cy, out.coef[1].cy_reference), weights[1]))
    omega0 = F(omega[0])
    ratio = F(M(delta, F(dt))/-omega0) if fabs(omega0) > F_TINY else 0.0
    out.blend = pmin(1.0, pmax(0.0, A(ratio, -1.0)))
    for i in range(2):
        out.cy[i] = A(M(S(out.coef[i].cy_reference, out.coef[i].cy), out.blend), A(out.coef[i].cy, out.coef[i].cy_add))
    chord = F(g.area/g.span)
    for i in range(2):
        x = out.coef[i].cx; y = out.cy[i]
        denom = A(M(y, y), M(x, x))
        inv = F(1/denom) if denom > F_TINY else 0.0
        k = M(inv, M(A(M(polars[i].clToCm1, y), polars[i].clToCm0), chord))
        out.points[i][0] = S(out.base[i][0], M(y, k))
        out.points[i][1] = S(out.base[i][1], M(x, k))
        out.points[i][2] = out.base[i][2]
    for i in range(2):
        drag[i] = M(-out.q[i], out.coef[i].cx)
        lift[i] = M(out.cy[i], out.q[i])
        xy[i][0] = M(drag[i], cb); xy[i][1] = M(lift[i], cv)
        aoa[i] = out.coef[i].aoa
        crit[i] = polars[i].aoaCritH
    for i in range(3):
        bv[i] = F(velocity[i])
    body_q = M(A(A(M(bv[0], bv[0]), M(bv[1], bv[1])), M(bv[2], bv[2])), M(F(density), .5))
    postprocess_xy(xy, aoa, crit, body_q, dt, height, ground_length, spin, spin_increment, spin_cap_input,
                   yaw_command, omega[1], g.use_spin_loss, spin_agl, g.spin_loss, &out.post)
    z[0] = S(M(drag[0], sb), M(lift[0], sv))
    z[1] = A(M(lift[1], sv), M(drag[1], sb))
    for i in range(2):
        out.forces[i][0] = M(out.post.xy[i][0], out.areas[i])
        out.forces[i][1] = M(out.post.xy[i][1], out.areas[i])
        out.forces[i][2] = M(z[i], out.areas[i])
