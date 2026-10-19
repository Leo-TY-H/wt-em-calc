# em_operating.complete_reporting: discrete energy step (kinematics.airborne_step,
# altitude_velocity_correction) and the continuous SEP.

cdef struct Kinematic:
    double position[3]
    double velocity[3]
    double quaternion[4]
    double omega[3]
    double angular_acceleration[3]
    double world_acceleration[3]
    bint reset
    double ps, ps_continuous
    bint altitude_correction


cdef void orientation_increment(const double* q, const double* angles_degrees, double* out) noexcept nogil:
    cdef double sa, ca, sb, cb, sc, cc, a, b, c, d, x, y, z, w, norm, inv
    cdef double o[4]
    cdef int i
    inline_half_sincos(M(F(angles_degrees[0]), F(M_PI/180)), &sa, &ca)
    inline_half_sincos(M(F(angles_degrees[1]), F(M_PI/180)), &sb, &cb)
    inline_half_sincos(M(F(angles_degrees[2]), F(-M_PI/180)), &sc, &cc)
    a = A(M(M(cc, sa), sb), M(M(sc, ca), cb))
    b = A(M(M(cc, sa), cb), M(M(sc, ca), sb))
    c = S(M(M(cc, ca), sb), M(M(sc, sa), cb))
    d = S(M(M(cc, ca), cb), M(M(sc, sa), sb))
    x = F(q[0]); y = F(q[1]); z = F(q[2]); w = F(q[3])
    o[0] = S(A(A(M(d, x), M(w, a)), M(c, y)), M(b, z))
    o[1] = S(A(A(M(d, y), M(w, b)), M(a, z)), M(c, x))
    o[2] = S(A(M(b, x), A(M(d, z), M(c, w))), M(y, a))
    o[3] = S(M(d, w), A(M(b, y), A(M(c, z), M(a, x))))
    x = o[0]; y = o[1]; z = o[2]; w = o[3]
    norm = F(sqrt(A(A(M(w, w), M(y, y)), A(M(z, z), M(x, x)))))
    if norm != 0.:
        inv = F(1./norm)
        for i in range(4):
            out[i] = M(o[i], inv)
    else:
        for i in range(4):
            out[i] = 0.


cdef void airborne_step(const double* position, const double* velocity_in, const double* quaternion, const double* omega,
                        const double* body_force, const double* moment, double mass, const double* inertia, double dt,
                        Kinematic* k) noexcept nogil:
    cdef double outer_dt, factor, limit, ix = inertia[0], iy = inertia[1], iz = inertia[2], s, inverse_mass, halfdt2
    cdef double acc[3]
    cdef double num[3]
    cdef double bg[3]
    cdef double ba[3]
    cdef double wa[3]
    cdef double vel[3]
    cdef double mid[3]
    cdef double angles[3]
    cdef double x, y, z, w, weight, ntw, mf, ax, ay, az
    cdef double wx2, xx2, yx2, xy2, yz2, wy2, wz2, xw2, xz2, yy2, ww, ww2m1, r00, r01, r02, r10, r11, r12, r20, r21, r22
    cdef bint angular_reset, acceleration_reset, velocity_reset
    cdef int i
    dt = dt
    outer_dt = F(dt); factor = F(1.)
    # angular_acceleration (contact moment zero)
    num[0] = (omega[1]*omega[2])*(iz-iy)+(0.+moment[0])
    num[1] = (omega[2]*omega[0])*(ix-iz)+(0.+moment[1])
    num[2] = (omega[1]*omega[0])*(iy-ix)+(0.+moment[2])
    for i in range(3):
        acc[i] = num[i]/inertia[i] if fabs(inertia[i]) > F_TINY else 0.
        k.angular_acceleration[i] = acc[i]
        k.omega[i] = omega[i]+acc[i]*dt
    limit = F(M(factor, F(M_PI))/outer_dt)
    angular_reset = False
    for i in range(3):
        if fabs(k.omega[i]) > limit:
            angular_reset = True
    if angular_reset:
        k.omega[0] = k.omega[1] = k.omega[2] = 0.
    # gravity_body
    x = F(quaternion[0]); y = F(quaternion[1]); z = F(quaternion[2]); w = F(quaternion[3])
    weight = M(F(9.81), mass); ntw = M(-2, weight)
    bg[0] = M(A(M(y, x), M(z, w)), ntw)
    bg[1] = M(A(-1, M(2, A(M(y, y), M(w, w)))), -weight)
    bg[2] = M(S(M(z, y), M(x, w)), ntw)
    # airborne_linear_acceleration
    mf = F(mass)
    inverse_mass = F(1./mf) if fabs(mf) > F_TINY else 0.
    for i in range(3):
        ba[i] = (bg[i]+body_force[i])*inverse_mass
    # rotate_acceleration_to_world
    ax = F(ba[0]); ay = F(ba[1]); az = F(ba[2])
    wx2 = A(w, w); xx2 = A(x, x); yx2 = A(y, y)
    xy2 = M(y, xx2); yz2 = M(z, yx2); wy2 = M(wx2, y)
    wz2 = M(wx2, z); xw2 = M(xx2, w); xz2 = M(z, xx2)
    yy2 = M(2, M(y, y)); ww = M(w, w); ww2m1 = A(A(ww, ww), -1)
    r00 = A(A(M(x, x), M(x, x)), ww2m1)
    r01 = S(xy2, wz2); r02 = A(xz2, wy2)
    r10 = A(wz2, xy2); r11 = A(yy2, ww2m1); r12 = S(yz2, xw2)
    r20 = S(xz2, wy2); r21 = A(xw2, yz2); r22 = A(A(M(z, z), M(z, z)), ww2m1)
    wa[0] = A(A(M(r02, az), M(ay, r01)), M(r00, ax))
    wa[1] = A(M(az, r12), A(M(r10, ax), M(ay, r11)))
    wa[2] = A(M(az, r22), A(M(r21, ay), M(ax, r20)))
    limit = M(M(F(F(686.4000244140625)/outer_dt), F(F(686.4000244140625)/outer_dt)), factor)
    s = 0.
    for i in range(3):
        s = s+wa[i]*wa[i]
    acceleration_reset = s > limit
    for i in range(3):
        vel[i] = velocity_in[i]
    if acceleration_reset:
        wa[0] = wa[1] = wa[2] = 0.; vel[0] = vel[1] = vel[2] = 0.
    # integrate_translation
    halfdt2 = (dt*dt)*.5
    for i in range(3):
        k.position[i] = position[i]+(vel[i]*dt+wa[i]*halfdt2)
        k.velocity[i] = wa[i]*dt+vel[i]
    s = 0.
    for i in range(3):
        s = s+k.velocity[i]*k.velocity[i]
    velocity_reset = s > M(factor, 2944656.)
    if velocity_reset:
        k.velocity[0] = k.velocity[1] = k.velocity[2] = 0.
        wa[0] = wa[1] = wa[2] = 0.
    for i in range(3):
        k.world_acceleration[i] = wa[i]
        mid[i] = omega[i]+acc[i]*(.5*dt)
    angles[0] = M(F(mid[1]*dt), -DEG_W); angles[1] = M(F(mid[2]*dt), -DEG_W); angles[2] = M(F(mid[0]*dt), DEG_W)
    orientation_increment(quaternion, angles, k.quaternion)
    k.reset = angular_reset or acceleration_reset or velocity_reset


cdef double altitude_velocity_correction(double height, double upward_velocity, double dt) noexcept nogil:
    cdef double h, rate
    if height <= 9900. or upward_velocity <= 0.:
        return upward_velocity
    h = F(height)
    if h <= 9900.:
        rate = 0.
    elif h >= 16000.:
        rate = -3.
    else:
        rate = A(M(h, F(-.0004918032791465521)), F(4.868852615356445))
    return upward_velocity+M(rate, F(dt))


