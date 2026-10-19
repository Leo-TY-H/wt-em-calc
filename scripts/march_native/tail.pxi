# tail_model / component_stages / downwash (full health, zero disturbance).

cdef struct TailProps:
    Polar hstab, vstab, fuselage
    double span, area, sweep, taper, dihedral, downwash_coefficient, travel_cap
    int engine_count, downwash_type
    bint clockwise
    double arm_h[3]
    double arm_v[3]
    double arm_f[3]
    double inc_h, inc_v, inc_f
    double fi_h, fi_v, fi_f
    double a_fuselage, a_left_main, a_left_elevator, a_right_main, a_right_elevator, a_v_main, a_rudder
    double slipstream_distance, downwash_aspect


cdef struct Tail:
    double forces[4][3]   # left_hstab, right_hstab, vstab, fuselage
    double points[4][3]
    double angles[4]      # effective angles (hstab l/r, vstab, fuselage)
    double previous_cl[2]
    double previous_angles[2]
    double q_tail, q_fuselage
    double coeff[4][2]
    double flows[4][3]
    double projection[2][3]
    double correction[2]
    double angles_wake[2]
    bint type2


cdef inline double tdiv(double a, double b) noexcept nogil:
    return F(a/b)


cdef void inline_half_sincos(double angle, double* sn_out, double* cs_out) noexcept nogil:
    cdef double h, r, r2, c, s, sn, cs
    cdef long long n
    h = M(angle, .5)
    n = <long long>trunc(A(copysign(.5, h), M(angle, F(.31830987334251404))))
    r = A(M(F(<double>n), F(-1.5707963705062866)), h)
    r2 = M(r, r)
    c = A(M(A(M(A(M(r2, F(-.0013602249091491103)), F(.04165669530630112)), r2), F(-.4999990165233612)), r2), 1.)
    s = A(M(A(M(A(M(r2, F(-.0001950727018993348)), F(.00833207555115223)), r2), F(-.16666652262210846)), M(r2, r)), r)
    if (n & 1) == 0:
        sn = s; cs = c
    else:
        sn = c; cs = s
    sn_out[0] = -sn if (n & 2) else sn
    cs_out[0] = -cs if ((n+1) & 2) else cs


cdef void wake_projection(const double* wing_point, const double* tail_point, double alpha_degrees,
                          double beta_degrees, double dihedral_degrees, double* out) noexcept nogil:
    cdef double sa, ca, sb, cb, sc, cc, x, y, z, w, dx, dy, dz, xx, xy, zw, xz, yw, base, yz, xw
    inline_half_sincos(S(PI_F, M(beta_degrees, RAD_W)), &sa, &ca)
    inline_half_sincos(M(alpha_degrees, RAD_W), &sb, &cb)
    inline_half_sincos(M(dihedral_degrees, RAD_W), &sc, &cc)
    x = A(M(M(cc, sa), sb), M(M(sc, ca), cb))
    y = A(M(M(cc, sa), cb), M(M(sc, ca), sb))
    z = S(M(M(cc, ca), sb), M(M(sc, sa), cb))
    w = S(M(M(cc, ca), cb), M(M(sc, sa), sb))
    dx = S(tail_point[0], wing_point[0]); dy = S(tail_point[1], wing_point[1]); dz = S(tail_point[2], wing_point[2])
    xx = A(M(x, x), M(x, x)); xy = M(A(x, x), y)
    zw = M(M(z, -2.), w); xz = M(A(z, z), x)
    yw = M(M(y, -2.), w)
    base = A(A(M(w, w), M(w, w)), -1.)
    out[0] = A(A(M(A(xz, yw), dz), M(S(xy, zw), dy)), M(A(xx, base), dx))
    yz = M(A(z, z), y); xw = M(M(x, -2.), w)
    out[1] = A(M(dy, A(base, A(M(y, y), M(y, y)))), A(M(dx, A(zw, xy)), M(dz, S(yz, xw))))
    out[2] = A(M(dz, A(base, A(M(z, z), M(z, z)))), A(M(dy, A(xw, yz)), M(dx, S(xz, yw))))


cdef inline double wrap_degrees(double value) noexcept nogil:
    value = F(fmod(value, 360.))
    return A(value, -360. if value >= 180. else (360. if value < -180. else 0.))


cdef double scalar_stage(double x, double a, double b, double inverse_speed2, double travel_cap, double roll_rate,
                         double current_cl, double cl_history_rate, double inverse_halfspan,
                         double longitudinal_exponent, double amplitude, double coefficient,
                         double inverse_span2) noexcept nogil:
    cdef double distance2, travel, angle, lateral, delayed_cl, longitudinal, strength, attenuation
    distance2 = A(A(M(a, a), M(b, b)), M(x, x))
    travel = pmin(F(sqrt(M(distance2, inverse_speed2))), travel_cap)
    angle = F(travel*roll_rate)
    lateral = M(A(M(F(cos(angle)), a), M(F(sin(angle)), b)), inverse_halfspan)
    delayed_cl = A(M(travel, cl_history_rate), current_cl)
    longitudinal = A(F(exp(M(x, longitudinal_exponent))), F(.9))
    strength = M(M(longitudinal, amplitude), delayed_cl)
    attenuation = F(exp(M(fabs(lateral), F(-1.4096779823303223))))
    return M(M(attenuation, inverse_span2), M(strength, coefficient))


cdef void type2_downwash(const TailProps* p, const double* wing_points, const double* tail_l, const double* tail_r,
                         const double* velocity, double omega_x, double alpha_degrees, double beta_degrees,
                         const double* current_cl, const double* previous_cl, double dt,
                         const double* tail_flows, double axial, double swirl, double* angles,
                         double* previous_out, double* projection_out, double* correction_out) noexcept nogil:
    # tail_flows: 2 x 3; wing_points: 2 x 3 (wing base points)
    cdef double span, area, sweep, taper, dihedral, coefficient, speed2, invspeed, invhalf, shape, amp, invspan2, invdt
    cdef double projected[3]
    cdef double delta, local
    cdef int i, side
    span = F(p.span); area = F(p.area); sweep = F(p.sweep); taper = F(p.taper); dihedral = F(p.dihedral)
    dt = F(dt); coefficient = F(p.downwash_coefficient)
    speed2 = F(velocity[2]*velocity[2]+(velocity[0]*velocity[0]+velocity[1]*velocity[1]))
    invspeed = wdiv(1., speed2)
    invhalf = wdiv(2., span)
    shape = A(M(F(exp(M(taper, F(-.35)))), F(-1.2)), F(1.56))
    amp = M(M(M(area, A(F(sin(M(sweep, RAD_W))), 1.)), shape), F(-46.2))
    invspan2 = tdiv(1., M(span, span)); invdt = tdiv(1., dt)
    for i in range(2):
        wake_projection(&wing_points[3*i], tail_l if i == 0 else tail_r, alpha_degrees, beta_degrees,
                        dihedral if i == 0 else -dihedral, projected)
        delta = 0.
        if projected[0] > 0:
            delta = scalar_stage(projected[0], projected[1], projected[2], invspeed, F(p.travel_cap), omega_x,
                                 F(current_cl[i]), M(S(previous_cl[i], current_cl[i]), invdt), invhalf,
                                 M(F(-2.5), invhalf), amp, coefficient, invspan2)
        side = 1 if ((i == 0) == (p.clockwise != 0)) else -1
        local = M(F(atan2(tail_flows[3*i+1]+swirl*(side*.36000001430511475), tail_flows[3*i]+axial)), -DEG_W)
        angles[i] = wrap_degrees(A(delta, local))
        previous_out[i] = F(current_cl[i])
        projection_out[3*i] = projected[0]; projection_out[3*i+1] = projected[1]; projection_out[3*i+2] = projected[2]
        correction_out[i] = delta


cdef void legacy_downwash(int mode, const double* tail_flows, double vertical_axial, double aspect, double distance,
                          double alpha, const double* current_cl, const double* previous_cl, double axial,
                          double swirl, int engine_count, double* angles, double* previous_out) noexcept nogil:
    cdef double attenuation = 0., factor = 0., angle, sn = 0., cs = 0., x, y, induced
    cdef int i
    if mode == 1:
        attenuation = 1./pow(1.+F(distance)*.04, 2.)
        factor = F(attenuation*tdiv(M(vertical_axial, F(1/M_PI)), aspect))
        angle = M(F(alpha), RAD_W); sn = F(sin(angle)); cs = F(cos(angle))
    for i in range(2):
        x = tail_flows[3*i]; y = tail_flows[3*i+1]
        if mode == 1:
            induced = M(F(current_cl[i]), factor)
            x = A(M(sn, induced), x); y = A(M(cs, induced), y)
            if engine_count == 1:
                x += axial
                y += swirl*.36000001430511475*(-1. if i == 0 else 1.)
        angles[i] = M(F(atan2(y, x)), -DEG_W)
        previous_out[i] = previous_cl[i]


cdef inline double history_increment(double current_degrees, double previous_degrees, double scale,
                                     double flow_inertia, double dt, bint vertical) noexcept nogil:
    cdef double delta
    delta = F(fmod(S(current_degrees, previous_degrees), F(2*M_PI)))
    if delta > PI_F:
        delta = A(delta, F(-2*M_PI))
    elif delta < -PI_F:
        delta = A(delta, F(2*M_PI))
    if vertical:
        return M(F(flow_inertia), tdiv(M(F(scale), -delta), F(dt)))
    return tdiv(M(M(delta, F(scale)), F(flow_inertia)), F(dt))


cdef void horizontal_application_point(const double* base, double cx, double cy, double cd_add, double cm0,
                                       double cm1, double* out) noexcept nogil:
    cdef double b, den, inv, k
    cx = A(F(cx), F(cd_add)); cy = F(cy)
    b = A(M(F(cm1), cy), F(cm0))
    den = A(M(cy, cy), M(cx, cx))
    inv = F(1/den) if den > F_TINY else 0.0
    k = M(inv, b)
    out[0] = S(F(base[0]), M(cy, k))
    out[1] = S(F(base[1]), M(cx, k))
    out[2] = F(base[2])


cdef void propeller_tail_flow(const double* flow, double wash0, double wash1, double distance, bint torque_gyro,
                              double* out, double* axial_out, double* swirl_out) noexcept nogil:
    cdef double axial_source, swirl_source, attenuation, axial, cross, gain, gate, swirl
    axial_source = F(wash0); swirl_source = F(wash1)
    out[0] = flow[0]; out[1] = flow[1]; out[2] = flow[2]
    if axial_source == 0.:
        axial_out[0] = 0.; swirl_out[0] = 0.
        return
    attenuation = 1./pow(1.+F(distance)*.04, 2.)
    axial = axial_source*attenuation
    cross = F(sqrt(A(M(flow[2], flow[2]), M(flow[1], flow[1]))))
    gain = pmax(1.+cross*-1.5/pmax(flow[0]+axial, .2), 0.)
    axial *= gain
    if fabs(axial) < 1.1920928955078125e-7:
        axial = 0.
    out[0] = F(flow[0]+axial)
    swirl = 0.
    if torque_gyro and swirl_source != 0.:
        gate = pmin(M(out[0], out[0])*.0011, 1.) if out[0] >= 0 else 0.
        swirl = (gain*attenuation)*(gate*swirl_source)
        if fabs(swirl) < 1.1920928955078125e-7:
            swirl = 0.
    axial_out[0] = axial; swirl_out[0] = swirl


cdef void secondary_model(const TailProps* p, double horizontal_bias, double vertical_control_scale,
                          double vertical_area_scale, bint convert_aoa, const double* velocity, const double* omega,
                          const double* cog, double density, double dt, double body_alpha, double body_beta,
                          const double* previous_angles, const double* previous_wing_cl, double wash0, double wash1,
                          bint torque_gyro, const double* wing_points, const double* wing_cl,
                          const double* tc, Tail* out) noexcept nogil:
    # tc: 3 x 5 mix outputs (left_hstab, right_hstab, vstab)
    cdef double lh[3]
    cdef double rh[3]
    cdef double vs_arm[3]
    cdef double fu_arm[3]
    cdef double v[3]
    cdef double w[3]
    cdef double flows[4][3]
    cdef double tail_flows[6]
    cdef double washed[3]
    cdef double axial, swirl, hx, hy, hz, rx, ry, rz, hw0, hw1
    cdef double c[5]
    cdef double cxcy[2]
    cdef double coeff[4][2]
    cdef double local_angles[4]
    cdef double hinc, rate, vinc, finc, ib, a, hc, rhohalf, qt, qf, al, ar, av, fq, vsf
    cdef double pt[3]
    cdef int i, j
    for i in range(3):
        lh[i] = F(p.arm_h[i]); vs_arm[i] = F(p.arm_v[i]); fu_arm[i] = F(p.arm_f[i])
    lh[0] = A(lh[0], p.hstab.aerCenterOffset)
    rh[0] = lh[0]; rh[1] = lh[1]; rh[2] = -lh[2]
    vs_arm[0] = A(vs_arm[0], p.vstab.aerCenterOffset)
    fu_arm[0] = A(fu_arm[0], p.fuselage.aerCenterOffset)
    for i in range(3):
        v[i] = F(velocity[i]); w[i] = F(omega[i])
    # horizontal_flows (disturbance zero: f32(0.)=0.)
    for i in range(2):
        if i == 0:
            hx = S(lh[0], F(cog[0])); hy = S(lh[1], F(cog[1])); hz = S(lh[2], F(cog[2]))
        else:
            hx = S(rh[0], F(cog[0])); hy = S(rh[1], F(cog[1])); hz = S(rh[2], F(cog[2]))
        flows[i][0] = A(S(A(v[0], 0.), M(w[1], hz)), M(w[2], hy))
        flows[i][1] = S(A(M(w[0], hz), A(0., v[1])), M(w[2], hx))
        if i == 0:
            flows[i][2] = A(S(A(0., v[2]), M(w[0], hy)), M(w[1], hx))
        else:
            flows[i][2] = S(A(A(0., v[2]), M(w[1], hx)), M(w[0], hy))
    # local_flow for the vertical stabilizer
    rx = S(F(vs_arm[0]), F(cog[0])); ry = S(F(vs_arm[1]), F(cog[1])); rz = S(F(vs_arm[2]), F(cog[2]))
    flows[2][0] = A(A(S(0., M(w[1], rz)), M(w[2], ry)), v[0])
    flows[2][1] = A(S(A(M(w[0], rz), 0.), M(w[2], rx)), v[1])
    flows[2][2] = A(S(0., M(w[0], ry)), A(M(w[1], rx), v[2]))
    # fuselage_flow
    rx = S(fu_arm[0], F(cog[0])); ry = S(fu_arm[1], F(cog[1])); rz = S(fu_arm[2], F(cog[2]))
    flows[3][0] = A(v[0], S(M(ry, w[2]), M(rz, w[1])))
    flows[3][1] = A(v[1], S(M(rz, w[0]), M(rx, w[2])))
    flows[3][2] = A(S(M(rx, w[1]), M(ry, w[0])), v[2])
    propeller_tail_flow(flows[2], wash0, wash1, p.slipstream_distance, torque_gyro, washed, &axial, &swirl)
    flows[2][0] = washed[0]; flows[2][1] = washed[1]; flows[2][2] = washed[2]
    for i in range(3):
        tail_flows[i] = flows[0][i]; tail_flows[3+i] = flows[1][i]
    if p.downwash_type == 2:
        if p.engine_count == 1:
            hw0 = axial; hw1 = swirl
        else:
            hw0 = 0.; hw1 = 0.
        type2_downwash(p, wing_points, lh, rh, velocity, omega[0], body_alpha, body_beta, wing_cl,
                       previous_wing_cl, dt, tail_flows, hw0, hw1, local_angles, out.previous_cl,
                       &out.projection[0][0], out.correction)
        out.type2 = True
    else:
        legacy_downwash(p.downwash_type, tail_flows, washed[0], p.downwash_aspect, p.slipstream_distance,
                        body_alpha, wing_cl, previous_wing_cl, axial, swirl, p.engine_count, local_angles,
                        out.previous_cl)
        out.type2 = False
    for i in range(4):
        for j in range(3):
            out.flows[i][j] = flows[i][j]
    out.angles_wake[0] = local_angles[0]; out.angles_wake[1] = local_angles[1]
    local_angles[2] = M(F(atan2(A(flows[2][2], M(F(swirl), -.5)), flows[2][0])), -DEG_W)
    local_angles[3] = M(F(atan2(flows[3][2], flows[3][0])), -DEG_W)
    # secondary_coefficients (full health: damage multipliers are 1)
    hinc = history_increment(body_alpha, previous_angles[0], 1., p.fi_h, dt, False)
    rate = history_increment(body_beta, previous_angles[1], 1., 1., dt, True)
    vinc = M(F(p.fi_v), rate)
    finc = M(rate, F(p.fi_f))
    ib = A(F(horizontal_bias), F(p.inc_h))
    for i in range(2):
        for j in range(5):
            c[j] = F(tc[5*i+j])
        hc = F(1.)
        a = A(A(A(M(c[1], hc), ib), F(local_angles[i])), hinc)
        calc_c(&p.hstab, a, F(body_alpha) if convert_aoa else a, M(c[2], hc), 1., cxcy)
        coeff[i][0] = A(F(cxcy[0]), M(c[3], hc)); coeff[i][1] = F(cxcy[1])
        out.angles[i] = a
    for j in range(5):
        c[j] = F(tc[10+j])
    hc = F(1.)
    a = A(S(S(F(local_angles[2]), F(p.inc_v)), M(M(F(vertical_control_scale), c[1]), hc)), vinc)
    calc_c(&p.vstab, a, -F(body_beta) if convert_aoa else a, M(c[2], -hc), 1., cxcy)
    coeff[2][0] = A(F(cxcy[0]), M(c[3], hc)); coeff[2][1] = F(cxcy[1])
    out.angles[2] = a
    a = A(finc, F(local_angles[3]))
    calc_c(&p.fuselage, a, -F(body_beta) if convert_aoa else a, 0., 1., cxcy)
    coeff[3][0] = F(cxcy[0]); coeff[3][1] = F(cxcy[1])
    out.angles[3] = a
    out.previous_angles[0] = F(body_alpha); out.previous_angles[1] = F(body_beta)
    rhohalf = M(F(density), .5)
    qt = M(A(A(M(flows[2][1], flows[2][1]), M(flows[2][0], flows[2][0])), M(flows[2][2], flows[2][2])), rhohalf)
    qf = M(rhohalf, A(A(M(flows[3][2], flows[3][2]), M(flows[3][0], flows[3][0])), M(flows[3][1], flows[3][1])))
    out.q_tail = qt; out.q_fuselage = qf
    for i in range(4):
        out.coeff[i][0] = coeff[i][0]; out.coeff[i][1] = coeff[i][1]
    # secondary_points
    horizontal_application_point(lh, coeff[0][0], coeff[0][1], 0., p.hstab.clToCm0, p.hstab.clToCm1, out.points[0])
    horizontal_application_point(rh, coeff[1][0], coeff[1][1], 0., p.hstab.clToCm0, p.hstab.clToCm1, out.points[1])
    pt[0] = vs_arm[0]; pt[1] = vs_arm[2]; pt[2] = vs_arm[1]
    horizontal_application_point(pt, coeff[2][0], coeff[2][1], 0., p.vstab.clToCm0, p.vstab.clToCm1, out.points[2])
    a = out.points[2][1]; out.points[2][1] = out.points[2][2]; out.points[2][2] = a
    pt[0] = fu_arm[0]; pt[1] = fu_arm[2]; pt[2] = fu_arm[1]
    horizontal_application_point(pt, coeff[3][0], coeff[3][1], 0., p.fuselage.clToCm0, p.fuselage.clToCm1, out.points[3])
    a = out.points[3][1]; out.points[3][1] = out.points[3][2]; out.points[3][2] = a
    # secondary_forces (health 1, vertical_lift_scale 1, vertical_area_add f32(.2))
    for i in range(4):
        for j in range(2):
            coeff[i][j] = F(coeff[i][j])
    qt = F(qt); qf = F(qf)
    al = A(M(F(1.), F(p.a_left_main)), M(F(p.a_left_elevator), F(1.)))
    ar = A(M(F(1.), F(p.a_right_elevator)), M(F(p.a_right_main), F(1.)))
    av = M(A(M(F(1.), F(p.a_rudder)), A(F(F(.2)), M(M(F(1.), 1.5), F(p.a_v_main)))), F(vertical_area_scale))
    fq = M(M(-qf, F(1.)), F(p.a_fuselage))
    vsf = F(F(1.)*qt*coeff[2][1])
    out.forces[0][0] = M(M(-qt, al), coeff[0][0]); out.forces[0][1] = M(M(coeff[0][1], qt), al); out.forces[0][2] = 0.0
    out.forces[1][0] = M(M(-qt, ar), coeff[1][0]); out.forces[1][1] = M(M(coeff[1][1], qt), ar); out.forces[1][2] = 0.0
    out.forces[2][0] = M(M(coeff[2][0], -qt), av); out.forces[2][1] = 0.0; out.forces[2][2] = M(av, vsf)
    out.forces[3][0] = M(coeff[3][0], fq); out.forces[3][1] = 0.0
    out.forces[3][2] = M(M(coeff[3][1], qf), M(F(1.), F(p.a_fuselage)))
