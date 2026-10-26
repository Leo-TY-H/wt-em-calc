# Native stationary propeller propulsion: prop_equilibrium.solve(_prepare_only=True,
# _stationary=True) sample/evaluate, prop_quasisteady.frame (binary64 blade
# polars, f32 local flow), piston_general.step with piston_compressor.step
# (discrete exact-altitude stages), rpm/reservoir torque, mixture and the
# mechanical multiplier, prop_requests.enumerate_speeds and the JointProblem
# governor-coordinate mapping. Supported subset (anything else stays Python):
# piston engines (families 0/1, compressor types 1/2, exact altitudes),
# propellers without cyclic or scheduled pitch (S5: also coaxial rotor pairs).

DEF NP = 12   # propellers / engines / transmissions supported


cdef struct BPolar:
    # polar_model prepared base (binary64) plus the Mach rows
    double cl0, cd0, indCoeff, clLineCoeff, cyCritH, cyCritL, aoaCritH, aoaCritL
    double aoaLineH, aoaLineL, parabCyCoeffH, parabCyCoeffL, aerCenterOffset
    double clToCm0, clToCm1, parabAngle, declineCoeff, maxDistAng, cdAfterCoeff
    double clAfterCritL, clAfterCritH, kq, clKq, cyMult
    int mode
    bint combined
    double rows[7][6]
    bint legacy
    double legacy_aoaLineH, legacy_aoaLineL, legacy_parabH, legacy_parabL


cdef struct PropT:
    BPolar polar
    double radius, area_unit
    int blades
    double station_radius[4]
    double twist[4]
    double width[4]
    double mean_twist, thrust_deflection, max_deflection, momentum_scale
    double pitch_damping[4]
    double yaw_damping[4]
    double damping_speed[4]
    double shake[4]
    double diameter, neutral_radius, inertia, pitch_min, pitch_max
    int governor
    bint governor_fast, torque_gyro_always, coaxial
    double min_omega, max_omega, inv_max_omega, boost_omega, reduction
    double basis[9]
    double inv_basis[9]
    double position[3]
    int direction
    int nairflow
    double airflow_x[16]
    double airflow_v[16][3]
    double polar_base1, polar_base8


cdef struct StageT:
    double critical, power, flat, flat_power, curvature, ceiling, ceiling_factor, boost, pressure_boost


cdef struct EngineT:
    int family, compressor_type, carburetor, mixer_type, boost_type, cylinders, nstages
    bint exact_altitudes, ata_enabled, manual_compressor, boost_controllable
    double base_hp, min_throttle, max_throttle, max_omega, afterburner_omega, omega_limit, inverse_omega
    double compressor_reference, torque_base, mixer_scale, compressor_sq, max_ata, ram_recovery, boost_ata_ratio
    double compressor_zero, throttle_boost, afterburner_boost, reservoir_capacity
    bint overrev_torque   # the game's over-rev torque (overrev.py; WT_OVERREV=0 / WT_OVERREV_TORQUE=0: off)
    double amplitude[3]
    double consumption[4]
    StageT stages[8]
    Table ata
    int ntargets
    double target_x[8]
    double target_y[8]
    # turbo-supercharger (compressor_type 3); turbines and rockets
    double shaft_min, turbo_min, turbo_max, turbo_allowed, inverse_time
    double direct_thrust
    double axis[3]
    double position[3]


cdef struct LinkT:
    int index
    double ratio, inverse_ratio


cdef struct TransT:
    int ne, np
    LinkT engines[NP]
    LinkT props[NP]
    bint correct_link


cdef struct EngState:
    # seed / per-sample engine state (the fields the steady piston model reads)
    double throttle, effective_throttle, omega, mechanical, mixture, reservoir, regulator, turbo
    bint afterburner, automatic_mixture, automatic_compressor
    int gear
    # piston step outputs
    double torque, friction, throttle_ratio, potential_manifold, manifold, extra_amplitude, consumption
    double turbo_command
    long seed
    # direct force and moment of the engine (turbine nozzles, rockets; zero for pistons) and its running state
    double force[3]
    double moment[3]
    int running


cdef struct PropState:
    double command, pitch
    double flow[3]
    bint auto
    double equilibrium_flow[3]
    double outputs[40]


cdef struct PFrame:
    EngState engines[NP]
    PropState props[NP]
    double trans_omega[NP]
    double trans_outputs[NP][13]
    double force[3]
    double moment[3]
    double momentum[3]
    double wash[2]


# ---------------------------------------------------------------- binary64 polar
cdef inline double safe_div(double a, double b) noexcept nogil:
    return a/b if fabs(b) > 4e-19 else 0.0


cdef double mach_factor(const double* row, double mach) noexcept nogil:
    cdef double low = row[0], a = row[1], b = row[2], high = row[3], slope = row[4], limit = row[5], value, t
    if mach < a:
        return low
    if mach > b:
        value = high+slope*(mach-b)
        return (pmin(value, limit) if slope >= 0 else pmax(value, limit))
    if a == b:
        return low*pow(mach, 3.)
    t = (mach-a)/(b-a)
    return (2*pow(t, 3.)-3*pow(t, 2.)+1)*low+(-2*pow(t, 3.)+3*pow(t, 2.))*high+(pow(t, 3.)-pow(t, 2.))*(b-a)*slope


cdef void stall_shape(BPolar* p, double cy_mult) noexcept nogil:
    p.aoaLineH = 2*safe_div(p.cyCritH-p.cl0, p.clLineCoeff*cy_mult)-p.aoaCritH
    p.aoaLineL = 2*safe_div(p.cyCritL-p.cl0, p.clLineCoeff*cy_mult)-p.aoaCritL
    if p.aoaLineL > p.aoaLineH:
        p.aoaLineL = p.aoaLineH = 0.5*(p.aoaLineL+p.aoaLineH)
    p.parabCyCoeffH = safe_div(p.cyCritH-(p.cl0+p.aoaLineH*p.clLineCoeff*cy_mult), pow(p.aoaCritH-p.aoaLineH, 2.))
    p.parabCyCoeffL = safe_div(-p.cyCritL+(p.cl0+p.aoaLineL*p.clLineCoeff*cy_mult), pow(p.aoaCritL-p.aoaLineL, 2.))


cdef void evaluate_polar(const BPolar* base, double mach, BPolar* p) noexcept nogil:
    cdef double f[8]
    cdef double cl, ind, cy_mult, mm, x
    cdef int i
    p[0] = base[0]
    cl = base.cl0; ind = base.indCoeff; cy_mult = base.cyMult
    if base.mode == 3:
        for i in range(1, 8):
            f[i] = mach_factor(base.rows[i-1], mach)
        p.cd0 = base.cd0*f[1]
        p.clLineCoeff = base.clLineCoeff*(((1+f[1])-f[2]) if base.combined else f[2])
        p.cl0 = cl*f[7]
        p.cyCritH = cl+(base.cyCritH-cl*f[7])*f[3]
        p.cyCritL = cl+(base.cyCritL-cl*f[7])*f[3]
        p.aoaCritH = base.aoaCritH*f[4]
        p.aoaCritL = base.aoaCritL*f[4]
        p.indCoeff = ind*f[5]
        p.aerCenterOffset = f[6]
        stall_shape(p, cy_mult)
    elif base.mode == 1 or base.mode == 2:
        mm = pmin(mach, 0.9) if base.mode == 1 else mach
        x = fabs(1-mm*mm)
        if base.mode == 1:
            x = pmin(x, 1.0)
        p.kq = 1/pmax(pow(x, 0.3), 0.2); p.clKq = sqrt(p.kq)
    if base.legacy:
        p.aoaLineH = base.legacy_aoaLineH; p.aoaLineL = base.legacy_aoaLineL
        p.parabCyCoeffH = base.legacy_parabH; p.parabCyCoeffL = base.legacy_parabL


cdef double calc_cl64(const BPolar* p, double a) noexcept nogil:
    cdef double s, crit, after, cy, max_ang, da, h, max_da, need, local_sign, local_aoa, correction
    if p.aoaLineL <= a and a <= p.aoaLineH:
        return p.cl0+p.clLineCoeff*a*p.cyMult
    s = 1 if a-p.aoaLineH+0.01 >= 0 else -1
    crit = p.aoaCritH if s > 0 else p.aoaCritL
    after = p.clAfterCritH if s > 0 else p.clAfterCritL
    cy = p.cyCritH if s > 0 else p.cyCritL
    if s*(a-crit) <= 0:
        return cy-s*(p.parabCyCoeffH if s > 0 else p.parabCyCoeffL)*pow(crit-a, 2.)
    max_ang = pmax(40, p.maxDistAng)
    if s*a <= max_ang:
        if s*a <= p.maxDistAng:
            da = a-crit
            if s*da < p.parabAngle:
                return cy-s*p.declineCoeff*da*da
            h = after*sin(M_PI*0.0125*p.maxDistAng)
            max_da = s*(p.maxDistAng-p.parabAngle)-crit
            need = cy-s*p.declineCoeff*pow(p.parabAngle, 2.)-h
            return h+safe_div(need, pow(max_da, 2.))*pow(s*p.maxDistAng-a, 2.)
        return after*sin(M_PI*0.0125*s*a)
    if s*a <= 140:
        local_sign = s; local_aoa = s*a
        if s*a > 90:
            local_sign = -s; local_aoa = 180-local_aoa
        correction = sin(M_PI*0.0125*max_ang)-sin(M_PI*0.5+M_PI*0.01*pmax(max_ang-40, 0))
        return s*after*(s*correction*(1-(s*a-40)/100)+sin(M_PI*0.5+M_PI*0.01*(local_aoa-40))*local_sign)
    return s*after*sin(M_PI*0.0125*(180-s*a))*(-s)


cdef double calc_cd64(const BPolar* p, double a) noexcept nogil:
    cdef double linear, s, crit, cd
    linear = p.cl0+p.clLineCoeff*a
    s = 1 if a >= 0 else -1
    crit = p.aoaCritH if s > 0 else p.aoaCritL
    cd = p.cd0+linear*linear*p.indCoeff+p.cdAfterCoeff*pmax(s*(a-crit), 0)
    return pmin(cd, 0.15+p.cyCritH*fabs(sin(a*DEG_TO_RAD)))


# ---------------------------------------------------------------- propeller
cdef inline double sign64(double x) noexcept nogil:
    return 1. if x > 0. else (-1. if x < 0. else 0.)

cdef inline double clamp64(double x, double lo, double hi) noexcept nogil:
    return pmin(pmax(x, lo), hi)

cdef inline double interval64(double x, double x0, double y0, double x1, double y1) noexcept nogil:
    if x <= x0:
        return y0
    if x >= x1:
        return y1
    return y0+(y1-y0)*(x-x0)/(x1-x0)

cdef inline void transform64(const double* b, double v0, double v1, double v2, double* out) noexcept nogil:
    cdef int i
    for i in range(3):
        out[i] = ((0+v0*b[i])+v1*b[i+3])+v2*b[i+6]


cdef void local_flow32(const PropT* p, const double* v, const double* w, double* local, double* local_w,
                       double* transverse_sq, double* transverse) noexcept nogil:
    # propeller_general.local_flow (binary32)
    cdef const double* r = p.position
    cdef const double* inv = p.inv_basis
    cdef double l[3]
    cdef int i
    l[0] = A(S(M(r[1], w[2]), M(r[2], w[1])), v[0])
    l[1] = A(S(M(r[2], w[0]), M(r[0], w[2])), v[1])
    l[2] = A(S(M(r[0], w[1]), M(r[1], w[0])), v[2])
    for i in range(3):
        local[i] = A(A(M(l[0], inv[i]), M(l[1], inv[i+3])), M(l[2], inv[i+6]))
        local_w[i] = A(A(M(w[2], inv[i+6]), M(w[1], inv[i+3])), M(w[0], inv[i]))
    transverse_sq[0] = A(M(local[2], local[2]), M(local[1], local[1]))
    transverse[0] = F(sqrt(transverse_sq[0]))


cdef void inverse_basis(const double* b, double* out) noexcept nogil:
    cdef double a = b[0], d = b[1], g = b[2], c = b[3], e = b[4], h = b[5], k = b[6], f = b[7], i = b[8], det
    cdef double vals[9]
    cdef int n
    det = S(A(A(M(M(a, e), i), M(M(d, h), k)), M(M(g, c), f)),
            A(A(M(M(g, e), k), M(M(d, c), i)), M(M(a, h), f)))
    vals[0] = S(M(e, i), M(h, f)); vals[1] = S(M(g, f), M(d, i)); vals[2] = S(M(d, h), M(g, e))
    vals[3] = S(M(h, k), M(c, i)); vals[4] = S(M(a, i), M(g, k)); vals[5] = S(M(g, c), M(a, h))
    vals[6] = S(M(c, f), M(e, k)); vals[7] = S(M(d, k), M(a, f)); vals[8] = S(M(a, e), M(d, c))
    for n in range(9):
        out[n] = F(vals[n]/det) if fabs(det) > F_TINY else 0.


cdef void blade_forces(const PropT* p, double omega, double pitch, double axial, double transverse, double density,
                       double sound, double angular_flow, double* thrust_out, double* torque_out) noexcept nogil:
    cdef double thrust = 0., torque = 0., radius, twist, width, tangential, phi, alpha, speed, force_unit, lift, drag
    cdef BPolar polar
    cdef int k
    for k in range(4):
        radius = p.station_radius[k]; twist = p.twist[k]; width = p.width[k]
        tangential = radius*(omega-angular_flow)
        phi = atan2(axial, tangential) if fabs(omega) >= 1e-5 else M_PI/2
        alpha = (pitch-phi+twist)*RAD_TO_DEG
        speed = hypot2(tangential+transverse, axial)
        if speed == 0.:
            continue
        evaluate_polar(&p.polar, speed/sound, &polar)
        force_unit = speed*speed*.5*density*width*p.area_unit
        lift = calc_cl64(&polar, alpha)*force_unit*polar.clKq
        drag = calc_cd64(&polar, alpha)*force_unit*polar.kq
        thrust += (lift*tangential-drag*axial)/speed
        torque += (lift*axial+drag*tangential)/speed*radius
    thrust_out[0] = thrust*p.blades; torque_out[0] = torque*p.blades


cdef inline double induced_flow(double thrust, double upstream, double density, double disc, double gain) noexcept nogil:
    # prop_quasisteady.propeller_load induced(thrust, upstream)
    cdef double value = 2*thrust/(density*disc)+sign64(upstream)*pow(upstream, 2.)
    return clamp64(sign64(thrust)*gain*(sign64(value)*sqrt(fabs(value))-upstream), -150., 150.)


cdef void propeller(const PropT* p, PropState* st, const double* velocity, const double* body_omega, const double* cg,
                    double omega, double density, double sound, bint torque_gyro) noexcept nogil:
    cdef double local[3]
    cdef double local_w[3]
    cdef double transverse_sq, transverse, disc, ambient, axial, thrust, torque, value, x, flux, divisor
    cdef double airflow[3]
    cdef double force[3]
    cdef double moment[3]
    cdef double arm[3]
    cdef double tr[3]
    cdef double deflection, axial_force, direction, reaction, normalized, damp, momentum, a, b, c, d, t
    cdef double first, thrust2, torque2
    cdef int i, k
    local_flow32(p, velocity, body_omega, local, local_w, &transverse_sq, &transverse)
    disc = M_PI*.25*pow(p.diameter, 2.)
    # prop_quasisteady.table(airflow, local[0])
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
    ambient = local[0]+airflow[0]
    # propeller_load
    axial = local[0]+.5*(st.flow[0]+st.flow[1])
    blade_forces(p, omega, st.pitch, axial, transverse, density, sound, .25*st.flow[2], &thrust, &torque)
    first = thrust
    if p.coaxial:
        # the second rotor: identical blade inputs at zero swirl and a finite nonzero shaft speed
        if st.flow[2] == 0. and omega != 0. and isfinite(omega):
            thrust2 = thrust;torque2 = torque
        else:
            blade_forces(p, omega, st.pitch, axial, transverse, density, sound, -.25*st.flow[2], &thrust2, &torque2)
        thrust = thrust+thrust2;torque = torque+torque2
    x = induced_flow(first, ambient, density, disc, airflow[2])
    if p.coaxial:
        st.equilibrium_flow[0] = x;st.equilibrium_flow[1] = induced_flow(thrust, ambient+.5*x, density, disc, airflow[2])
        st.equilibrium_flow[2] = 0.
    else:
        flux = sqrt(pow(local[0]+.5*x, 2.)+transverse_sq)*disc*density
        divisor = pow(p.diameter*.5, 2.)*flux*.5
        st.equilibrium_flow[0] = x; st.equilibrium_flow[1] = 0.
        st.equilibrium_flow[2] = clamp64(torque/divisor if fabs(divisor) > 4e-19 else 0., 0., 20.)
    deflection = clamp64(p.thrust_deflection*thrust, -p.max_deflection, p.max_deflection)
    axial_force = sqrt(pmax(0., 1.-pow(deflection, 2.)))*thrust
    transform64(p.basis, axial_force, 0., 0., force)
    direction = 1. if p.direction == 0 else -1.
    reaction = 0. if p.coaxial else torque*direction
    normalized = pow(omega/p.reduction*p.inv_max_omega, 2.)
    damp = interval64(local[0], p.damping_speed[0], p.damping_speed[1], p.damping_speed[2], p.damping_speed[3])
    transform64(p.basis, 0.,
                -sign64(local_w[1])*damp*normalized*interval64(fabs(local_w[1]), p.pitch_damping[0], p.pitch_damping[1], p.pitch_damping[2], p.pitch_damping[3]),
                -sign64(local_w[2])*damp*normalized*interval64(fabs(local_w[2]), p.yaw_damping[0], p.yaw_damping[1], p.yaw_damping[2], p.yaw_damping[3]),
                moment)
    for i in range(3):
        arm[i] = p.position[i]-cg[i]
    a = moment[0]+force[1]*arm[2]-arm[1]*force[2]
    b = moment[1]+force[2]*arm[0]-arm[2]*force[0]
    c = moment[2]+force[0]*arm[1]-arm[0]*force[1]
    moment[0] = a; moment[1] = b; moment[2] = c
    momentum = 0. if p.coaxial else omega*p.inertia*direction*p.momentum_scale
    for i in range(40):
        st.outputs[i] = 0.
    for i in range(3):
        st.outputs[i] = force[i]; st.outputs[23+i] = moment[i]
    st.outputs[18] = torque
    if torque_gyro or p.torque_gyro_always:
        transform64(p.basis, reaction, 0., 0., tr)
        for i in range(3):
            st.outputs[20+i] = tr[i]; st.outputs[26+i] = momentum*p.basis[i]
    a = p.shake[0]; b = p.shake[1]; c = p.shake[2]; d = p.shake[3]
    st.outputs[29] = normalized*(interval64(transverse, a, 0., b, 1.) if transverse < b else
                                 (1. if transverse < c else interval64(transverse, c, 1., d, 0.)))
    st.outputs[30] = p.inertia; st.outputs[31] = st.command
    st.outputs[32] = st.flow[0]+st.flow[1]; st.outputs[33] = st.flow[2]
    st.outputs[35] = axial_force/thrust-1. if fabs(thrust) > 4e-19 else 0.


# ---------------------------------------------------------------- piston engine
cdef inline double pdiv(double a, double b) noexcept nogil:
    return F(a/b) if fabs(b) > F_TINY else 0.


cdef double pressure_at_height(double height) noexcept nogil:
    cdef double pressure0 = F(101300.), ceiling = F(18300.), h, p
    height = F(height)
    h = pmin(height, ceiling)
    p = F(1.6037300167995674e-18)
    p = A(M(p, h), F(-1.3738000266463185e-13))
    p = A(M(p, h), F(5.676299874579627e-9))
    p = A(M(p, h), F(-.00011844099935842678))
    p = A(M(p, h), F(1.))
    return pdiv(M(M(ceiling, pressure0), p), pmax(height, ceiling))


cdef double inlet_pressure(double height, double body_u, double recovery) noexcept nogil:
    cdef double rho = density_at_height(F(height), F(1.225), F(18300))
    cdef double ram = M(M(M(F(body_u), F(body_u)), M(rho, .5)), F(recovery))
    return pdiv(A(ram, pressure_at_height(height)), F(101300.))


cdef bint boost_active(const EngineT* p, double throttle, bint afterburner, int gear, double nitro) noexcept nogil:
    cdef int kind
    if not afterburner:
        return False
    kind = p.boost_type
    if kind == 2 or kind == 4 or kind == 5 or kind == 9 or kind == 1:
        if kind == 1 and throttle <= 1.:
            return False
        if nitro < F(.001):
            return False
    return not (0 <= gear and gear < p.nstages) or M(p.stages[gear].boost, p.afterburner_boost) > F(.99)


cdef double interval32(double x, double x0, double y0, double x1, double y1) noexcept nogil:
    cdef double t, delta, fraction
    x = F(x); x0 = F(x0); y0 = F(y0); x1 = F(x1); y1 = F(y1)
    if x1 < x0:
        t = x0; x0 = x1; x1 = t
        t = y0; y0 = y1; y1 = t
    if x <= x0:
        return y0
    if x >= x1:
        return y1
    delta = S(x1, x0)
    fraction = F(M(S(x, x0), S(y1, y0))/delta) if fabs(delta) > F_TINY else 0.
    return A(y0, fraction)


cdef double target_omega(const EngineT* p, const EngState* s, double nitro) noexcept nogil:
    cdef double throttle = F(s.throttle)
    cdef int i
    if p.boost_type > 0 and p.boost_controllable:
        throttle = F(1.1) if boost_active(p, s.effective_throttle, s.afterburner, s.gear, nitro) else pmin(throttle, 1.)
    if p.ntargets == 0:
        return 0.
    if throttle <= p.target_x[0]:
        return p.target_y[0]
    for i in range(p.ntargets-1):
        if throttle < p.target_x[i+1]:
            return interval32(throttle, p.target_x[i], p.target_y[i], p.target_x[i+1], p.target_y[i+1])
    return p.target_y[p.ntargets-1]


cdef inline double overrev_torque(const EngineT* p, double omega, double torque) noexcept nogil:
    """piston_model.overrev_torque: the game's torque of an engine over RPMMaxAllowed (the first over-rev step of a fresh
    engine), 1 + 0.01 x (omega - 0.999965 x limit) (rad/s)."""
    if not p.overrev_torque or not p.omega_limit > 0. or not omega > F(p.omega_limit):
        return torque
    return M(torque, A(M(S(omega, M(F(.9999650120735168), p.omega_limit)), F(.009999999776482582)), 1.))


cdef double rpm_torque(const EngineT* p, double omega, double throttle, double torque_multiplier, bint afterburner,
                       int gear, double nitro) noexcept nogil:
    cdef double effective, wref, x, shape, scale, torque, tboost
    cdef bint boost
    omega = F(omega); throttle = F(throttle); torque_multiplier = F(torque_multiplier)
    effective = A(M(S(1., p.min_throttle), pmin(p.max_throttle, throttle)), p.min_throttle)
    boost = boost_active(p, throttle, afterburner, gear, nitro)
    wref = p.afterburner_omega if boost else p.max_omega
    x = pdiv(omega, M(effective, wref))
    shape = A(A(.5, effective), M(M(effective, effective), -.5))
    scale = M(M(p.torque_base, shape), torque_multiplier)
    torque = M(S(M(3., x), M(M(2., x), x)), scale)
    if throttle > 1.:
        tboost = p.throttle_boost
        if throttle < F(1.1):
            tboost = A(M(A(M(tboost, F(9.999998092651367)), F(-9.999998092651367)), S(throttle, 1.)), 1.)
        torque = M(torque, tboost)
    torque = overrev_torque(p, omega, torque)
    if boost:
        torque = M(torque, M(p.afterburner_boost, p.stages[gear].boost))
    if torque < 0.:
        torque = pmax(torque, M(M(M(M(torque_multiplier, F(-.8)), omega), p.inverse_omega), p.torque_base))
    return torque


cdef double reservoir_torque(const EngineT* p, double omega, double throttle, double multiplier, double reservoir,
                             bint afterburner, int gear, double nitro) noexcept nogil:
    cdef double effective, x, shape, half, fuel, tq
    effective = A(M(S(1., p.min_throttle), pmin(p.max_throttle, throttle)), p.min_throttle)
    x = pdiv(omega, p.max_omega); shape = S(M(x, 3.), pdiv(A(M(x, x), M(x, x)), effective))
    half = M(p.reservoir_capacity, .5)
    if reservoir <= half:
        fuel = F(.6)
    elif reservoir >= p.reservoir_capacity:
        fuel = 1.
    else:
        fuel = A(F(.6), pdiv(M(S(reservoir, half), S(1., F(.6))), S(p.reservoir_capacity, half)))
    tq = overrev_torque(p, omega, M(M(M(M(p.throttle_boost if throttle > 1. else 1., multiplier), p.torque_base), fuel), shape))
    if boost_active(p, throttle, afterburner, gear, nitro):
        tq = M(tq, M(p.stages[gear].boost, p.afterburner_boost))
    if tq < 0.:
        tq = pmax(tq, M(M(M(M(multiplier, F(-.8)), omega), p.inverse_omega), p.torque_base))
    return tq


cdef int mixture(const EngineT* p, double inlet, double command, double rich, bint automatic, double* factor_out,
                 double* rich_out) noexcept nogil:
    """Returns 1 when the running branch requires a stop."""
    cdef double pressure = F(inlet), accum = F(rich), supplied, limit, factor, inv
    cdef bint stop
    cdef int kind = p.mixer_type
    command = F(command)
    if kind == 0 or not (kind == 1 or kind == 2):
        factor_out[0] = 1.; rich_out[0] = accum
        return 0
    if kind == 1:
        command = pmax(1., command)
        if command == 1.:
            factor_out[0] = 1.; rich_out[0] = accum
            return 0
    supplied = M(command, p.mixer_scale)
    if automatic:
        supplied = pmin(supplied, A(pressure, F(-.01)))
    stop = supplied < pressure and supplied < F(.03)
    if supplied < pressure:
        limit = M(pressure, .25)
        factor = 1. if supplied > limit else pdiv(supplied, limit)
    elif supplied == pressure:
        factor = 1.
    else:
        inv = pdiv(1., supplied)
        factor = M(pressure, inv)
        accum = A(A(M(M(pressure, -35.), inv), 35.), accum)
    factor_out[0] = factor; rich_out[0] = accum
    return 1 if stop else 0


cdef int compressor_step(const EngineT* p, double omega, double throttle, double inlet, double dt, int gear_in,
                         int old_gear, double regulator, bint afterburner, double nitro, double height,
                         double* multiplier, int* gear_out, double* regulator_out, double* ratio_out,
                         double* potential_out, double* manifold_out, int* selected_out=NULL) noexcept nogil:
    """piston_compressor.step for discrete stages (types 1/2); gear_in < 0 selects automatically. S16: ``selected_out``
    (where given) is the stage the scores selected, -1 where none scored above the starting -1 (the fallback factor -1
    that aces.exe 0x142fa2180 returns too)."""
    cdef double requested, x, speed, best, potential, target, gain, ratio, manifold, factor
    cdef double pb, flow, critical, flat, ceiling, line, baseline, shape, score, offset, xx, scale
    cdef bint boost, flat_present, ceiling_present, exact
    cdef int i, lo_i, hi_i, sel = -1, k
    cdef double sel_shape = -1., sel_flow = 0., sel_line = 1., sel_offset = 0., sel_pb = 1.
    cdef const StageT* s
    omega = F(omega); throttle = F(throttle); inlet = F(inlet); dt = F(dt); regulator = F(regulator)
    if not p.ata_enabled:
        requested = M(A(M(pmin(throttle, 1.), F(.45)), F(.55)), p.max_ata)
    elif throttle < F(.01):
        requested = pdiv(pressure_at_height(height), 101300.)
    else:
        curve(&p.ata, throttle, 1, &requested)
    x = M(p.inverse_omega, omega)
    speed = A(M(A(M(S(M(x, x), x), p.compressor_sq), x), S(1., p.compressor_zero)), p.compressor_zero)
    if gear_in >= 0 and not (0 <= gear_in and gear_in < p.nstages):
        return 1
    best = -1.
    lo_i = 0 if gear_in < 0 else gear_in
    hi_i = p.nstages if gear_in < 0 else gear_in+1
    exact = p.exact_altitudes
    for i in range(lo_i, hi_i):
        s = &p.stages[i]
        boost = boost_active(p, throttle, afterburner, i, nitro)
        pb = p.boost_ata_ratio if boost else 1.
        flow = M(inlet, s.pressure_boost) if boost else inlet
        critical = s.critical; flat = s.flat; ceiling = s.ceiling
        flat_present = critical < A(flat, F(-.0001)); ceiling_present = A(ceiling, F(.0001)) < critical
        line = s.ceiling_factor if exact and ceiling_present else 1.
        if not exact and (flat_present or ceiling_present):
            scale = M(pb, pdiv(1., speed)); critical = M(scale, critical); flat = M(flat, scale); ceiling = M(ceiling, scale)
        baseline = 1.
        for k in range(i):
            baseline = M(M(baseline, pmax(s.power, 1.)), F(.8))
        if flow <= critical:
            if exact:
                shape = s.power
            elif flow <= ceiling:
                shape = s.ceiling_factor if ceiling_present else s.power
            else:
                shape = A(pdiv(M(S(s.ceiling_factor, s.power), S(critical, flow)), pmax(S(critical, ceiling), F(.0001))), s.power)
        elif not flat_present:
            shape = A(pdiv(M(S(s.power, baseline), S(1., flow)), pmax(S(1., critical), F(.0001))), baseline)
        elif flow <= flat:
            if exact:
                xx = pmin(1., pmax(0., A(1., pdiv(M(S(flow, critical), -1.), S(flat, critical)))))
            else:
                xx = interval32(flow, critical, 1., flat, 0.)
            shape = A(M(F(pow(xx, s.curvature)), S(s.power, s.flat_power)), s.flat_power)
        else:
            shape = A(pdiv(M(S(s.flat_power, baseline), S(1., flow)), pmax(S(1., flat), F(.0001))), baseline)
        potential = M(M(flow, speed), pdiv(p.max_ata, M(p.compressor_reference, s.critical)))
        score = pmin(potential, M(p.max_ata, pb)); offset = S(1., line)
        if exact:
            score = A(pdiv(M(score, line), M(pb, requested)), offset)
        score = M(score, shape)
        if boost:
            score = M(score, M(s.boost, p.afterburner_boost))
        if score > best:
            sel = i; best = score
            sel_shape = shape; sel_flow = flow; sel_line = line; sel_offset = offset; sel_pb = pb
    if selected_out != NULL:selected_out[0] = sel
    if sel < 0:
        sel = 0; sel_shape = -1.; sel_flow = inlet; sel_line = 1.; sel_offset = 0.
        sel_pb = p.boost_ata_ratio if boost_active(p, throttle, afterburner, 0, nitro) else 1.
    s = &p.stages[sel]
    potential = M(M(speed, sel_flow), pdiv(p.max_ata, M(p.compressor_reference, s.critical)))
    target = M(pdiv(p.max_ata, potential), sel_pb)
    if sel == old_gear and regulator >= 0.:
        gain = F(.1) if p.compressor_type == 2 else M(dt, 3.)
        target = A(M(S(target, regulator), gain), regulator)
    regulator = pmin(1., pmax(0., target)); ratio = pmin(pdiv(requested, p.max_ata), 1.)
    manifold = M(M(potential, ratio), regulator)
    factor = M(A(pdiv(M(manifold, sel_line), M(requested, sel_pb)), sel_offset), sel_shape)
    # low_rpm(manifold, omega, flow)
    if 20. <= omega and omega < 150.:
        manifold = pmin(manifold, M(A(M(omega, F(.04)), F(-.39999995)), sel_flow))
    elif omega < 20.:
        manifold = M(A(M(omega, F(-.03)), 1.), sel_flow)
    multiplier[0] = factor; gear_out[0] = sel; regulator_out[0] = regulator; ratio_out[0] = ratio
    potential_out[0] = potential; manifold_out[0] = manifold
    return 0

cdef void turbo_compressor(const EngineT* p, double omega, double throttle, double inlet, double dt, bint afterburner,
                           double nitro, double height, double turbo, double turbo_command, bint automatic, bint assisted,
                           double* multiplier, double* ratio_out, double* potential_out, double* manifold_out,
                           double* turbo_out, double* command_out) noexcept nogil:
    """piston_compressor.step for the turbo-supercharger (compressor_type 3, exact altitudes)."""
    cdef double requested, x, speed, shape, base, upper, allowed, target, fraction, potential, top, ratio, manifold, line, factor
    cdef const StageT* s = &p.stages[0]
    omega = F(omega); throttle = F(throttle); inlet = F(inlet); dt = F(dt)
    if not p.ata_enabled:
        requested = M(A(M(pmin(throttle, 1.), F(.45)), F(.55)), p.max_ata)
    elif throttle < F(.01):
        requested = pdiv(pressure_at_height(height), 101300.)
    else:
        curve(&p.ata, throttle, 1, &requested)
    x = M(p.inverse_omega, omega)
    speed = A(M(A(M(S(M(x, x), x), p.compressor_sq), x), S(1., p.compressor_zero)), p.compressor_zero)
    if inlet <= s.critical:
        shape = s.power
    else:
        shape = A(pdiv(M(S(s.power, 1.), S(1., inlet)), pmax(S(1., s.critical), F(.0001))), 1.)
    base = M(M(speed, inlet), pdiv(p.max_ata, M(p.compressor_reference, s.critical)))
    if boost_active(p, throttle, afterburner, 0, nitro):
        requested = M(requested, p.boost_ata_ratio)
    upper = interval32(omega, p.shaft_min, p.turbo_min, p.max_omega, p.turbo_max)
    allowed = p.turbo_allowed
    if automatic or assisted:
        target = pmin(M(pdiv(allowed, base), requested), allowed)
        turbo_command = pdiv(target, S(upper, p.turbo_min))
    else:
        target = A(M(S(upper, p.turbo_min), turbo_command), p.turbo_min)
    turbo = F(turbo)
    turbo = target if assisted else A(M(M(S(target, turbo), dt), p.inverse_time), turbo)
    if turbo < p.turbo_min:
        fraction = pdiv(turbo, p.turbo_min);potential = M(inlet, .75)
        if fraction > 0.:
            top = M(pdiv(p.turbo_min, allowed), base)
            potential = A(potential, M(S(top, potential), fraction)) if fraction < 1. else top
    else:
        potential = M(pdiv(turbo, allowed), base)
    ratio = pmin(pdiv(requested, potential), 1.);manifold = M(ratio, potential)
    line = s.ceiling_factor if A(s.ceiling, F(.0001)) < s.critical else 1.
    factor = M(A(pdiv(M(line, manifold), requested), S(1., line)), shape)
    # low_rpm(manifold, omega, inlet)
    if 20. <= omega and omega < 150.:
        manifold = pmin(manifold, M(A(M(omega, F(.04)), F(-.39999995)), inlet))
    elif omega < 20.:
        manifold = M(A(M(omega, F(-.03)), 1.), inlet)
    multiplier[0] = factor;ratio_out[0] = ratio;potential_out[0] = potential;manifold_out[0] = manifold
    turbo_out[0] = turbo;command_out[0] = turbo_command


cdef int mechanical_multiplier(const EngineT* p, double omega, double previous, double extra, double torque,
                               double friction, double dt, long seed, double* value_out, double* mechanical_out,
                               long* seed_out) noexcept nogil:
    """engine_supply.mechanical_multiplier with health 1 and all cylinders; returns 1 if a random draw is needed."""
    cdef double rate, loss, damage, shaft, amplitude
    omega = F(omega); previous = F(previous); extra = F(extra); torque = F(torque); friction = F(friction); dt = F(dt)
    if A(torque, friction) < 0:
        value_out[0] = 1.; mechanical_out[0] = previous; seed_out[0] = seed
        return 0
    rate = F(3.33)
    if p.cylinders < 3:
        amplitude = F(11.25)
    else:
        loss = S(1., F(F(p.cylinders)/F(p.cylinders)))
        damage = pmax(M(pmin(S(F(1.470588207244873), M(F(1.), F(1.470588207244873))), 1.), 3.), M(M(loss, loss), F(9.25)))
        shaft = M(p.inverse_omega, omega)
        amplitude = A(A(A(extra, p.amplitude[2]), M(omega, A(M(p.amplitude[0], omega), p.amplitude[1]))), M(damage, M(shaft, shaft)))
        if amplitude <= F(.01):
            value_out[0] = 1.; mechanical_out[0] = 1.; seed_out[0] = seed
            return 0
    return 1


cdef void scalar_full(const JetT* p, double density, double speed, double omega, double throttle, double dt,
                      bint afterburner, double fuel_available, double health, double shaft_fraction,
                      double* thrust_out, double* torque_out, double* consumption_out, double* target_out,
                      double* next_out, bint* active_out) noexcept nogil:
    """jet_model.scalar_update(running=2) with its torque, consumption and next omega (turbine engines)."""
    cdef double tab[4]
    cdef double thrust, ab, torque, abtorque, target, out_thrust, out_torque, consumption, fraction, candidate_thrust
    cdef double x1, x2, y1, y2, t, cap, tau_mult, sgn
    cdef double r1[1]
    cdef double md[5]
    cdef int i, selected, nm = p.modes.n
    cdef bint ab_mode, active = False
    density = F(density); speed = F(speed); omega = F(omega); throttle = F(throttle); dt = F(dt)
    fuel_available = F(fuel_available); health = F(health); shaft_fraction = F(shaft_fraction)
    jet_table(p, density, speed, tab)
    thrust = tab[0]; ab = tab[1]; torque = tab[2]; abtorque = tab[3]
    target = 0.; out_thrust = 0.; out_torque = 0.; consumption = 0.; tau_mult = 1.
    if nm > 0:
        curve(&p.throttle, throttle, 1, r1); target = r1[0]
        selected = -1; fraction = 0.; candidate_thrust = thrust
        for i in range(nm-1, -1, -1):
            ab_mode = M(p.throttle_scale, p.throttle.x[i]) > 1. or (afterburner and i == nm-1)
            candidate_thrust = M(ab if ab_mode else 1., thrust)
            fraction = F(fuel_available/pmax(M(candidate_thrust, M(p.consumption, dt)), F(1e-6)))
            if M(p.modes.v[i][0], p.modes.v[i][3]) < fraction:
                selected = i
                break
        if selected >= 0:
            active = True
            if afterburner:
                thrust = candidate_thrust;torque = M(torque, abtorque)
            if selected < nm-1:
                x1 = M(p.modes.v[selected][0], p.modes.v[selected][3]); x2 = M(p.modes.v[selected+1][0], p.modes.v[selected+1][3])
                y1 = p.modes.x[selected]; y2 = p.modes.x[selected+1]
                if x2 < x1:
                    t = x1; x1 = x2; x2 = t
                    t = y1; y1 = y2; y2 = t
                if fraction <= x1:
                    cap = y1
                elif fraction >= x2:
                    cap = y2
                else:
                    cap = A(y1, F(M(S(fraction, x1), S(y2, y1))/S(x2, x1)))
                target = pmin(target, cap)
            curve(&p.modes, F(omega/p.max_omega), 5, md)
            out_thrust = M(thrust, md[0])
            out_torque = M(A(M(S(md[2], md[1]), shaft_fraction), md[1]), torque)
            consumption = M(M(md[3], out_thrust), p.consumption)
            tau_mult = md[4]
        else:
            target = 0.
    sgn = -1. if health < 0 else (1. if health > 0 else 0.)
    target = M(M(M(target, p.max_omega), sgn), F(sqrt(fabs(health))))
    thrust_out[0] = out_thrust; torque_out[0] = out_torque; consumption_out[0] = consumption; target_out[0] = target
    next_out[0] = A(F(M(S(target, omega), dt)/M(tau_mult, p.tau)), omega); active_out[0] = active


cdef int turbine_output(const EngineT* p, EngState* s, const Unit* u, double body_u, double height, double nitro,
                        const double* cg) noexcept nogil:
    """turbine_general.step (dt = 0, the nominal seed) for a turbojet (family 2) or turboprop (family 5)."""
    cdef double dt = F(0.), omega, throttle, rho, mod, mechanical, gas, thrust, torque, consumption, target, next_omega, ias
    cdef double mod2, mechanical2
    cdef bint active
    cdef long seed_out, seed2
    if u == NULL:
        return 9
    omega = F(s.omega); throttle = pmin(F(s.throttle), F(1.1))
    rho = density_at_height(F(height), F(1.225), F(18300.))
    if mechanical_multiplier(p, omega, s.mechanical, 0., s.torque, s.friction, dt, 12345, &mod, &mechanical, &seed_out):
        return 7
    gas = F(s.turbo) if p.family == 5 else omega
    scalar_full(&u.jet, rho, body_u, gas, pmin(pdiv(throttle, F(1.1) if p.throttle_boost > 1. else 1.), 1.), dt,
                boost_active(p, throttle, s.afterburner, s.gear, nitro), 1e6, mod, M(omega, p.inverse_omega),
                &thrust, &torque, &consumption, &target, &next_omega, &active)
    if not active:
        return 8
    ias = M(F(body_u), F(sqrt(pdiv(rho, F(1.225)))))
    nozzle_vectors(u, thrust, cg, ias, 0., s.force, s.moment)
    s.torque = 0.; s.friction = 0.; s.mechanical = mechanical; s.seed = seed_out; s.extra_amplitude = 0.
    s.consumption = consumption; s.effective_throttle = throttle; s.running = 7
    if p.family == 5:
        if mechanical_multiplier(p, omega, mechanical, 0., torque, s.friction, dt, seed_out, &mod2, &mechanical2, &seed2):
            return 7
        s.torque = M(mod2, torque); s.mechanical = mechanical2; s.seed = seed2
        s.turbo = pmin(pmax(next_omega, 0.), p.turbo_allowed)
    else:
        s.omega = pmin(pmax(next_omega, 0.), p.omega_limit)
        if p.compressor_type == 3:
            s.turbo = F(s.turbo); s.turbo_command = F(s.turbo_command)
    return 0


cdef int rocket_output(const EngineT* p, EngState* s, const double* cg) noexcept nogil:
    """rocket_general.step (an unlit rocket gives no thrust)."""
    cdef double throttle, normalized, omega, thrust, r[3]
    cdef int i
    throttle = pmin(F(s.throttle), F(1.1))
    normalized = pmin(pdiv(throttle, F(1.1) if p.throttle_boost > 1. else 1.), 1.)
    omega = A(M(S(p.max_omega, p.shaft_min), normalized), p.shaft_min)
    thrust = pdiv(M(omega, p.direct_thrust), p.max_omega)
    if s.running != 7:
        omega = 0.; thrust = 0.; normalized = 0.; throttle = 0.
    for i in range(3):
        s.force[i] = M(thrust, p.axis[i]); r[i] = S(p.position[i], F(cg[i]))
    s.moment[0] = S(M(s.force[1], r[2]), M(s.force[2], r[1]))
    s.moment[1] = S(M(s.force[2], r[0]), M(s.force[0], r[2]))
    s.moment[2] = S(M(s.force[0], r[1]), M(s.force[1], r[0]))
    s.consumption = M(M(normalized, p.consumption[3]), F(s.throttle))
    s.omega = omega; s.mechanical = F(s.mechanical); s.seed = 12345; s.extra_amplitude = 0.; s.effective_throttle = throttle
    s.turbo = F(s.turbo); s.turbo_command = F(s.turbo_command)
    return 0


cdef int engine_output(const EngineT* p, EngState* s, double body_u, double height, double nitro, const Unit* u,
                       const double* cg) noexcept nogil:
    """prop_quasisteady.engine_output: pistons, turbines (family 2/5) and rockets (family 3); nonzero: unavailable."""
    if p.family == 2 or p.family == 5:
        s.regulator = -1.; s.extra_amplitude = 0.
        return turbine_output(p, s, u, body_u, height, nitro, cg)
    if p.family == 3:
        s.regulator = -1.; s.extra_amplitude = 0.
        return rocket_output(p, s, cg)
    s.force[0] = s.force[1] = s.force[2] = 0.; s.moment[0] = s.moment[1] = s.moment[2] = 0.; s.running = 7
    return piston_output(p, s, body_u, height, nitro)


cdef int piston_output(const EngineT* p, EngState* s, double body_u, double height, double nitro) noexcept nogil:
    """prop_quasisteady.engine_output for a running piston engine; nonzero: Python must handle it."""
    cdef double dt = F(0.), omega, throttle, inlet, mult, regulator, ratio, potential, manifold, tq, mixf, rich, modulation
    cdef double mechanical, power, x, rate, mixf2, rich2
    cdef double* y
    cdef int gear, code
    cdef long seed_out
    cdef double turbo, command
    if p.family != 0 and p.family != 1:
        return 2
    s.regulator = -1.; s.extra_amplitude = 0.
    omega = F(s.omega); throttle = pmin(F(s.throttle), F(1.1))
    inlet = inlet_pressure(height, body_u, p.ram_recovery)
    if p.compressor_type == 3:
        # engine_output: the assisted turbo of the steady state, then piston_step's own (dt = 0) compressor step
        turbo_compressor(p, s.omega, s.throttle, inlet, 0., s.afterburner, nitro, height, 0., 1., True, True,
                         &mult, &ratio, &potential, &manifold, &turbo, &command)
        turbo_compressor(p, omega, throttle, inlet, dt, s.afterburner, nitro, height, turbo, s.turbo_command, True, False,
                         &mult, &ratio, &potential, &manifold, &turbo, &command)
        s.turbo = turbo;s.turbo_command = command
        gear = 0;regulator = 1.
    else:
        code = compressor_step(p, omega, throttle, inlet, dt,
                               s.gear if (p.manual_compressor and not s.automatic_compressor) else -1, s.gear,
                               s.regulator, s.afterburner, nitro, height,
                               &mult, &gear, &regulator, &ratio, &potential, &manifold)
        if code:
            return 4
    if p.carburetor == 3:
        tq = reservoir_torque(p, omega, throttle, 1., s.reservoir, s.afterburner, gear, nitro)
    elif p.carburetor == 1 or p.carburetor == 2:
        tq = rpm_torque(p, omega, throttle, 1., s.afterburner, gear, nitro)
    else:
        return 5
    tq = M(tq, mult)
    if mixture(p, inlet, s.mixture, s.extra_amplitude, s.automatic_mixture, &mixf, &rich):
        return 6
    tq = M(mixf, tq)
    if mechanical_multiplier(p, omega, s.mechanical, rich, tq, s.friction, dt, 12345, &modulation, &mechanical, &seed_out):
        return 7
    tq = M(modulation, tq)
    power = pmax(M(p.base_hp, F(.05)), M(M(tq, F(.00134048261602968)), omega))
    x = pmax(0., pdiv(power, p.base_hp))
    y = p.consumption
    if x <= .5:
        rate = y[0] if x <= 0 else (y[1] if x >= .5 else A(y[0], M(x, A(S(y[1], y[0]), S(y[1], y[0])))))
    elif x <= 1.:
        rate = y[2] if x >= 1 else A(M(A(A(x, x), -1.), S(y[2], y[1])), y[1])
    else:
        rate = y[3] if x >= F(1.1) else A(M(A(M(x, F(9.999998092651367)), F(-9.999998092651367)), S(y[3], y[2])), y[2])
    if p.compressor_type == 1 and p.mixer_type == 1:
        mixture(p, inlet, s.mixture, 0., s.automatic_mixture, &mixf2, &rich2)
        manifold = M(mixf2, manifold)
    s.torque = tq; s.friction = 0.; s.regulator = regulator; s.gear = gear; s.throttle_ratio = ratio
    s.potential_manifold = potential; s.manifold = manifold; s.mechanical = mechanical; s.seed = seed_out
    s.extra_amplitude = rich; s.consumption = M(M(power, F(.00027777778450399637)), rate)
    s.effective_throttle = throttle
    return 0
