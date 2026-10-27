# Controls, air state, aircraft assembly and the explicit-history operating
# point (em_operating.operating_point with stationary_angles, jet/rocket
# direct-thrust engines, one discrete propulsion phase).

cdef struct Table:
    int n
    double x[12]
    double inv[12]
    double v[12][5]


cdef struct Mix:
    double angles[3][2]
    Table t1[3]
    int n2[3]
    double d2[3][10]
    double i2[3][10]
    Table t2[3][10]
    bint use2[3]
    double sensitivity
    Table sens_curve
    double cl[2]
    double cd[2]
    double wing_aoa


cdef struct Primary:
    double eff[3][2]
    double power[3][2]
    double minimum[3]
    double negative[3]
    bint strong, invert_elevator
    bint trim_available[3]


cdef struct Allocation:
    double commands[3]
    double delivered[3]
    double sticks[3]
    double trim[3]
    double bounds[3][2]
    double ranges[3][2]
    bint reachable
    double allocation_error


cdef void curve(const Table* t, double x, int width, double* out) noexcept nogil:
    cdef int i, j
    if t.n == 0:
        for j in range(width):
            out[j] = 0.0
        return
    if t.n == 1 or x <= t.x[0]:
        for j in range(width):
            out[j] = t.v[0][j]
        return
    if x >= t.x[t.n-1]:
        for j in range(width):
            out[j] = t.v[t.n-1][j]
        return
    for i in range(t.n-1):
        if x <= t.x[i+1]:
            tt = M(S(x, t.x[i]), t.inv[i])
            for j in range(width):
                out[j] = A(M(S(t.v[i+1][j], t.v[i][j]), tt), t.v[i][j])
            return
    for j in range(width):
        out[j] = t.v[t.n-1][j]


cdef void surface_curve(const Mix* p, int axis, double density, double speed, double* out) noexcept nogil:
    cdef int n = p.n2[axis], i, j
    cdef double a[3]
    cdef double b[3]
    cdef double t
    if n == 0:
        out[0] = out[1] = out[2] = 0.0
        return
    if n == 1 or density <= p.d2[axis][0]:
        curve(&p.t2[axis][0], speed, 3, out)
        return
    if density >= p.d2[axis][n-1]:
        curve(&p.t2[axis][n-1], speed, 3, out)
        return
    for i in range(n-1):
        if density <= p.d2[axis][i+1]:
            t = M(S(density, p.d2[axis][i]), p.i2[axis][i])
            curve(&p.t2[axis][i], speed, 3, a)
            curve(&p.t2[axis][i+1], speed, 3, b)
            for j in range(3):
                out[j] = A(M(S(b[j], a[j]), t), a[j])
            return
    curve(&p.t2[axis][n-1], speed, 3, out)


cdef void mix(const Mix* p, const double* commands, const bint* inverted, double aoa_bias, double density,
              double speed, double mach, double* out) noexcept nogil:
    cdef double lim[3]
    cdef double contributions[3]
    cdef double positive, center, negative, slope, value, deflection, sensitivity, scaled, shift, cl, cd
    cdef double s1[1]
    cdef int i
    for i in range(3):
        if p.use2[i]:
            surface_curve(p, i, density, speed, lim)
        else:
            curve(&p.t1[i], speed, 3, lim)
        # lim = (a, center, b)
        if inverted[i]:
            positive = M(lim[2], p.angles[i][1])
            center = M(lim[1], p.angles[i][1] if lim[1] >= 0 else p.angles[i][0])
            negative = M(lim[0], -p.angles[i][0])
        else:
            positive = M(lim[0], p.angles[i][0])
            center = M(lim[1], p.angles[i][0] if lim[1] >= 0 else p.angles[i][1])
            negative = M(lim[2], -p.angles[i][1])
        slope = S(center, positive) if commands[i] >= 0 else S(negative, center)
        value = S(center, M(slope, commands[i]))
        contributions[i] = -value if inverted[i] else value
    deflection = A(A(contributions[1], contributions[0]), contributions[2])
    curve(&p.sens_curve, mach, 1, s1)
    sensitivity = s1[0]
    scaled = M(sensitivity, -deflection)
    shift = A(M(p.sensitivity, scaled), aoa_bias)
    cl = M(scaled, p.cl[0] if deflection >= 0 else p.cl[1])
    cd = M(shift, p.cd[0] if shift >= 0 else p.cd[1])
    out[0] = deflection; out[1] = shift; out[2] = cl; out[3] = cd; out[4] = M(p.wing_aoa, -deflection)


cdef double power_limit(const Primary* p, double speed, int axis, int side) noexcept nogil:
    cdef double effective = p.eff[axis][side], denominator, ratio
    if speed <= effective:
        return 1.
    denominator = speed
    ratio = F(effective/denominator) if fabs(denominator) > F_TINY else 0.
    return pmax(p.minimum[axis], F(pow(ratio, p.power[axis][side])))


cdef void authority_ranges(const Primary* p, double speed, double* out) noexcept nogil:
    # out: 3 x 2 (roll, pitch, yaw); full_loss, symmetric, unit authority, elevator_state 0
    cdef double roll, yaw, rr0, rr1, yr0, yr1, pr0, pr1, t
    speed = F(speed)
    roll = power_limit(p, speed, 0, 0); yaw = power_limit(p, speed, 2, 0)
    rr0 = pmax(-1., M(-roll, 1.)); rr1 = roll
    yr0 = pmax(-1., M(-yaw, 1.)); yr1 = yaw
    pr0 = -power_limit(p, speed, 1, 0); pr1 = power_limit(p, speed, 1, 1)
    if fabs(S(p.eff[1][0], p.eff[1][1])) < F(1e-6) and fabs(S(p.power[1][0], p.power[1][1])) < F(1e-6):
        pr0 = pmax(-1., M(-pr1, 1.))
    if not p.strong:
        if p.invert_elevator != (F(0.) >= 0.):
            pr0 = yr0; pr1 = yr1
        t = A(M(pr0, .5), -.5); pr1 = A(M(pr1, .5), .5); pr0 = t
    out[0] = M(rr0, F(1.)); out[1] = M(rr1, F(1.))
    out[2] = M(pr0, F(1.)); out[3] = M(pr1, F(1.))
    out[4] = M(yr0, F(1.)); out[5] = M(yr1, F(1.))


cdef void trimmed_targets(const double* sticks, const double* trim, const double* ranges, double* out) noexcept nogil:
    cdef int i
    cdef double command, t, lo, hi, value, product, sign
    for i in range(3):
        command = pmin(1., pmax(-1., F(sticks[i]))); t = F(trim[i])
        lo = ranges[2*i]; hi = ranges[2*i+1]
        if command <= -1.:
            value = lo
        elif command >= 1.:
            value = hi
        else:
            value = A(M(A(M(command, .5), .5), S(hi, lo)), lo)
        product = M(value, t)
        sign = 1. if product > 0. else (-1. if product < 0. else 0.)
        out[i] = A(M(S(1., M(fabs(t), sign)), value), t)


cdef void steady_commands(const Primary* p, const double* sticks, const double* trim, const double* ranges,
                          double* out) noexcept nogil:
    cdef double applied[3]
    cdef int i
    for i in range(3):
        applied[i] = F(trim[i]) if p.trim_available[i] else 0.
    trimmed_targets(sticks, applied, ranges, out)
    out[1] = pmin(1., pmax(-1., out[1])); out[2] = pmin(1., pmax(-1., out[2]))
    if p.invert_elevator:
        out[1] = -out[1]


cdef void command_allocation(const Primary* p, const double* coordinate, const double* ranges, bint optimized,
                             double trim_limit, const double* fixed_trim, Allocation* out) noexcept nogil:
    cdef double low_trim[3]
    cdef double high_trim[3]
    cdef double ones[3]
    cdef double minus[3]
    cdef double lo_end[3]
    cdef double hi_end[3]
    cdef double d, raw, t, delta, denominator, v, stick, lo, hi, product
    cdef int i
    for i in range(3):
        ones[i] = 1.; minus[i] = -1.
        if optimized:
            low_trim[i] = -trim_limit if p.trim_available[i] else 0.
            high_trim[i] = trim_limit if p.trim_available[i] else 0.
        else:
            low_trim[i] = high_trim[i] = fixed_trim[i] if p.trim_available[i] else 0.
    steady_commands(p, minus, low_trim, ranges, lo_end)
    steady_commands(p, ones, high_trim, ranges, hi_end)
    for i in range(3):
        if hi_end[i] < lo_end[i]:
            out.bounds[i][0] = hi_end[i]; out.bounds[i][1] = lo_end[i]
        else:
            out.bounds[i][0] = lo_end[i]; out.bounds[i][1] = hi_end[i]
        out.ranges[i][0] = ranges[2*i]; out.ranges[i][1] = ranges[2*i+1]
    for i in range(3):
        d = F(coordinate[i]); out.commands[i] = d
        lo = ranges[2*i]; hi = ranges[2*i+1]
        raw = -d if (i == 1 and p.invert_elevator) else d
        if optimized and p.trim_available[i]:
            t = pmax(-trim_limit, pmin(trim_limit, raw))
        else:
            t = low_trim[i]
        delta = raw-t
        product = delta*t
        denominator = 1.-fabs(t)*(1. if product > 0 else (-1. if product < 0 else 0.))
        v = delta/denominator if denominator != 0. else 0.
        stick = 2.*(v-lo)/(hi-lo)-1. if hi > lo else 0.
        out.sticks[i] = F(pmax(-1., pmin(1., stick))); out.trim[i] = F(t)
    steady_commands(p, out.sticks, out.trim, ranges, out.delivered)
    out.reachable = True
    out.allocation_error = 0.
    for i in range(3):
        if not (out.bounds[i][0]-2e-7 <= out.commands[i] and out.commands[i] <= out.bounds[i][1]+2e-7):
            out.reachable = False
        if i == 0:
            out.allocation_error = fabs(out.delivered[0]-out.commands[0])
        else:
            out.allocation_error = pmax(out.allocation_error, fabs(out.delivered[i]-out.commands[i]))


# ---------------------------------------------------------------- air state
cdef double density_at_height(double height, double rho0, double ceiling) noexcept nogil:
    cdef double h = pmin(height, ceiling), polynomial = F(2.287190065873038e-19)
    polynomial = A(M(polynomial, h), F(-5.83556030646186e-14))
    polynomial = A(M(polynomial, h), F(3.5311800150594763e-09))
    polynomial = A(M(polynomial, h), F(-9.593870345270261e-05))
    polynomial = A(M(polynomial, h), F(1.0))
    return F(M(M(ceiling, rho0), polynomial)/pmax(height, ceiling))


cdef double speed_of_sound(double height, double temperature0, double ceiling) noexcept nogil:
    cdef double h = pmin(F(height), F(ceiling)), p = F(3.9730601514748866e-18)
    p = A(M(p, h), F(-5.711039686951899e-14))
    p = A(M(p, h), F(2.1806899341836328e-10))
    p = A(M(p, h), F(-2.277120074722916e-05))
    p = A(M(p, h), F(1.))
    return M(F(sqrt(M(p, F(temperature0)))), F(20.1))


cdef struct Air:
    double alpha, beta, tas, speed_squared, mach, ias_u, density


cdef void air_cache(const double* velocity, double height, Air* out) noexcept nogil:
    cdef double u = velocity[0], v = velocity[1], w = velocity[2], rho0 = F(1.225), t0 = F(288.16), ceiling = F(18300.)
    out.speed_squared = F((w*w+u*u)+v*v)
    out.tas = F(sqrt(out.speed_squared))
    out.density = density_at_height(F(height), F(rho0), F(ceiling))
    out.alpha = M(F(atan2(v, u)), F(-57.2957763671875)) if (u != 0. or v != 0.) else -0.
    out.beta = M(F(atan2(w, u)), F(57.2957763671875)) if (u != 0. or w != 0.) else 0.
    out.ias_u = F(u*F(sqrt(F(out.density/F(rho0)))))
    out.mach = F(out.tas/speed_of_sound(height, t0, ceiling))


cdef void turn_quaternion(double alpha_deg, double bank_deg, double* q) noexcept nogil:
    cdef double alpha = alpha_deg*DEG_TO_RAD, bank = bank_deg*DEG_TO_RAD
    q[0] = F(sin(bank*.5)*cos(alpha*.5))
    q[1] = F(-sin(bank*.5)*sin(alpha*.5))
    q[2] = F(cos(bank*.5)*sin(alpha*.5))
    q[3] = F(cos(bank*.5)*cos(alpha*.5))


cdef void world_to_body_air(const double* q, double speed, double* out) noexcept nogil:
    cdef double vx = F(speed-(0.+0.)), vy = F(0.-(0.+0.)), vz = F(0.-(0.+0.))
    cdef double x = F(q[0]), y = F(q[1]), z = F(q[2]), w = F(q[3])
    cdef double base, r00, r11, r22, x2, y2, wn, xy, yz, xz, wy, wz, wx
    base = A(A(M(w, w), M(w, w)), -1.)
    r00 = A(A(M(x, x), M(x, x)), base); r11 = A(A(M(y, y), M(y, y)), base); r22 = A(A(M(z, z), M(z, z)), base)
    x2 = A(x, x); y2 = A(y, y); wn = M(w, -2.)
    xy = M(x2, y); yz = M(y2, z); xz = M(x2, z); wy = M(y, wn); wz = M(wn, z); wx = M(wn, x)
    out[0] = A(A(M(S(xy, wz), vy), M(A(wy, xz), vz)), M(vx, r00))
    out[1] = A(A(M(S(yz, wx), vz), M(A(xy, wz), vx)), M(vy, r11))
    out[2] = A(M(r22, vz), A(M(S(xz, wy), vx), M(A(yz, wx), vy)))


cdef struct Geometry:
    double forward[3]
    double normal[3]
    double side[3]
    double up[3]
    double lateral[3]
    double omega[3]
    double quaternion[4]
    double turn_rate


cdef void turn_geometry(double alpha_deg, double bank_deg, double speed, double load, double dt, double ias_u,
                        double G, double turn, Geometry* out) noexcept nogil:
    # em_solver.turn_geometry: bank into the turn; turn = 1 left (bit-identical to the left-only geometry), -1 right
    cdef double alpha = alpha_deg*DEG_TO_RAD, bank = (turn*bank_deg)*DEG_TO_RAD
    cdef double sa = sin(alpha), ca = cos(alpha), sb = sin(bank), cb = cos(bank)
    cdef double x, y, z, w, s, r00, r10, r20, r11, r12, yaw, pitch, roll, damping, k
    out.forward[0] = ca; out.forward[1] = -sa; out.forward[2] = 0.
    out.normal[0] = sa; out.normal[1] = ca; out.normal[2] = 0.
    out.side[0] = 0.; out.side[1] = 0.; out.side[2] = 1.
    out.up[0] = cb*sa-sb*0.; out.up[1] = cb*ca-sb*0.; out.up[2] = cb*0.-sb*1.
    out.lateral[0] = turn*(sb*sa+cb*0.); out.lateral[1] = turn*(sb*ca+cb*0.); out.lateral[2] = turn*(sb*0.+cb*1.)
    out.turn_rate = G*sqrt(pmax(0., load*load-1.))/speed
    s = sin(out.turn_rate*dt*.5)
    x = -turn*out.up[0]*s; y = -turn*out.up[1]*s; z = -turn*out.up[2]*s
    w = cos(out.turn_rate*dt*.5)
    r00 = 1-2*(y*y+z*z); r10 = 2*(x*y+w*z); r20 = 2*(x*z-w*y)
    r11 = 1-2*(x*x+z*z); r12 = 2*(y*z-w*x)
    yaw = atan2(-r20, r00); pitch = asin(pmax(-1., pmin(1., r10))); roll = atan2(-r12, r11)
    damping = pmax(1.-F(ias_u)*1e-5, .8)
    k = 2./(1.+damping)
    out.omega[0] = (-roll/dt)*k; out.omega[1] = (-yaw/dt)*k; out.omega[2] = (-pitch/dt)*k
    turn_quaternion(alpha_deg, turn*bank_deg, out.quaternion)
