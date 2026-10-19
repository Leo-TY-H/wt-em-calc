# Direct-thrust engine force vectors: em_solver.EngineEnsemble.vectors with
# EngineUnit (jet_nominal + jet_model.scalar_update + jet_nozzle.evaluate)
# and rocket_em.RocketUnit (constant thrust + jet_nozzle.evaluate). Normal
# forward flight: sticks, VTOL, reverse and airbrake commands are zero.

cdef struct NozzleT:
    double position[3]
    double ratio, maximum
    double basis[3][3]
    bint main_controls, tip
    double weights[2][5]
    double ranges[2][2]
    double flaps[4]
    Table roll_angle, pitch_angle, yaw_angle, vtol_angle, reverse_angle
    Table roll_thrust, pitch_thrust, yaw_thrust, airbrake, vtol, reverse


cdef struct JetT:
    int nd, ns
    double density[16]
    double speed[32]
    double cells[16][32][4]
    double bases[4]
    Table modes
    Table throttle
    double throttle_scale, max_omega, consumption, tau
    double normalized, supply, omega_limit, inverse_omega, dt, rho
    bint afterburner


cdef struct Unit:
    bint rocket
    double thrust
    double ias_factor
    JetT jet
    int nnoz
    NozzleT noz[4]


cdef void jet_bracket(const double* grid, int n, double value, bint descending, int* i_out, double* w_out) noexcept nogil:
    cdef int i = 0
    if n == 1:
        i_out[0] = 0; w_out[0] = 0.
        return
    while i+2 < n and ((value < grid[i+1]) if descending else (value > grid[i+1])):
        i += 1
    i_out[0] = i
    w_out[0] = pmin(1., pmax(0., F(S(value, grid[i])/S(grid[i+1], grid[i]))))


cdef void jet_table(const JetT* p, double density, double speed, double* out) noexcept nogil:
    cdef int i, j, i1, j1, k
    cdef double u, v, a, b, c, d, lo, value
    jet_bracket(p.density, p.nd, F(density), True, &i, &u)
    jet_bracket(p.speed, p.ns, F(speed), False, &j, &v)
    i1 = i+1 if i+1 < p.nd-1 else p.nd-1
    j1 = j+1 if j+1 < p.ns-1 else p.ns-1
    for k in range(4):
        a = p.cells[i][j][k]; b = p.cells[i][j1][k]
        c = p.cells[i1][j][k]; d = p.cells[i1][j1][k]
        lo = A(M(S(b, a), v), a)
        value = A(M(A(S(c, lo), M(S(d, c), v)), u), lo) if i1 != i else lo
        out[k] = M(value, p.bases[k])


cdef void scalar_update(const JetT* p, double density, double speed, double omega, double throttle, double dt,
                        bint afterburner, double fuel_available, double shaft_fraction,
                        double* thrust_out, double* target_out, bint* active_out) noexcept nogil:
    cdef double tab[4]
    cdef double thrust, ab, torque, abtorque, target, out_thrust, fraction, candidate_thrust, x1, x2, y1, y2, t, cap, health
    cdef double r1[1]
    cdef double md[5]
    cdef int i, selected, nm = p.modes.n
    cdef bint ab_mode, active = False
    density = F(density); speed = F(speed); omega = F(omega); throttle = F(throttle); dt = F(dt)
    fuel_available = F(fuel_available); health = F(1.); shaft_fraction = F(shaft_fraction)
    jet_table(p, density, speed, tab)
    thrust = tab[0]; ab = tab[1]; torque = tab[2]; abtorque = tab[3]
    target = 0.; out_thrust = 0.
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
                thrust = candidate_thrust
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
        else:
            target = 0.
    target = M(M(M(target, p.max_omega), 1.), F(sqrt(fabs(health))))
    thrust_out[0] = out_thrust; target_out[0] = target; active_out[0] = active


cdef inline double control_value(const double* values, double command) noexcept nogil:
    return A(M(S(values[2], values[1]) if command >= 0 else S(values[1], values[0]), command), values[1])


cdef void nozzle_vectors(const Unit* u, double thrust, const double* cg, double ias, double flaps,
                         double* force, double* moment) noexcept nogil:
    cdef double r, e, y, v, reverse_angle, a, sh, ch, sv, cv, x0, y0, x1, y1, t, blend, capped, multiplier
    cdef double rd, ed, yd, th, fx, fy, fz, rx, ry, rz, m0, m1, m2
    cdef double c3[3]
    cdef double c2[2]
    cdef double rl[3]
    cdef double el[3]
    cdef double yl[3]
    cdef double angle[2]
    cdef double direction[3]
    cdef int k, i
    cdef const NozzleT* p
    thrust = F(thrust); ias = F(ias); flaps = F(flaps)
    force[0] = force[1] = force[2] = 0.; moment[0] = moment[1] = moment[2] = 0.
    for k in range(u.nnoz):
        p = &u.noz[k]
        # sticks are zero: roll = pitch = yaw = f32(0.) (or 0. on tip nozzles)
        if not p.main_controls:
            curve(&p.roll_angle, ias, 3, c3); r = control_value(c3, 0.)
            curve(&p.pitch_angle, ias, 3, c3); e = -control_value(c3, 0.)
            curve(&p.yaw_angle, ias, 3, c3); y = control_value(c3, 0.)
        else:
            r = 0.; e = 0.; y = 0.
        curve(&p.vtol_angle, ias, 2, c2); v = A(M(S(c2[1], c2[0]), 0.), c2[0])
        curve(&p.reverse_angle, ias, 2, c2); reverse_angle = S(M(S(c2[0], c2[1]), 0.), c2[0])
        for i in range(2):
            a = S(M(p.weights[i][1], e), A(M(p.weights[i][2], y), M(p.weights[i][0], r)))
            a = A(S(M(p.weights[i][4], reverse_angle), M(p.weights[i][3], v)), a)
            angle[i] = pmin(p.ranges[i][1], pmax(p.ranges[i][0], a))
        sh = F(sin(angle[0])); ch = F(cos(angle[0])); sv = -F(sin(angle[1])); cv = F(cos(angle[1]))
        for i in range(2):
            direction[i] = A(M(cv, A(M(p.basis[2][i], sh), M(p.basis[0][i], ch))), M(sv, p.basis[1][i]))
        direction[2] = A(M(p.basis[1][2], sv), M(A(M(p.basis[2][2], sh), M(p.basis[0][2], ch)), cv))
        x0 = p.flaps[0]; y0 = p.flaps[1]; x1 = p.flaps[2]; y1 = p.flaps[3]
        if x1 < x0:
            t = x0; x0 = x1; x1 = t
            t = y0; y0 = y1; y1 = t
        if flaps <= x0:
            blend = y0
        elif flaps >= x1:
            blend = y1
        else:
            blend = A(y0, F(M(S(flaps, x0), S(y1, y0))/S(x1, x0)))
        capped = pmin(M(M(blend, thrust), p.ratio), p.maximum)
        multiplier = 1.
        if not p.main_controls:
            curve(&p.roll_thrust, ias, 3, rl); curve(&p.pitch_thrust, ias, 3, el); curve(&p.yaw_thrust, ias, 3, yl)
            rd = M(S(rl[2], rl[1]), 0.)
            ed = M(S(el[2], el[1]), 0.)
            yd = M(S(yl[1], yl[2]), 0.)
            multiplier = S(A(A(A(A(el[1], rd), ed), A(rl[1], 1.)), yl[1]), yd)
        curve(&p.airbrake, ias, 2, c2); multiplier = A(A(c2[0], multiplier), M(S(c2[1], c2[0]), 0.))
        curve(&p.vtol, ias, 2, c2); multiplier = A(A(c2[0], multiplier), M(S(c2[1], c2[0]), 0.))
        curve(&p.reverse, ias, 2, c2); multiplier = A(A(c2[0], multiplier), M(S(c2[1], c2[0]), 0.))
        # accumulate_nozzle
        th = M(F(multiplier), F(capped))
        fx = M(F(direction[0]), th); fy = M(F(direction[1]), th); fz = M(F(direction[2]), th)
        rx = S(F(p.position[0]), F(cg[0])); ry = S(F(p.position[1]), F(cg[1])); rz = S(F(p.position[2]), F(cg[2]))
        m0 = F(moment[0]); m1 = F(moment[1]); m2 = F(moment[2])
        force[0] = A(F(force[0]), fx); force[1] = A(F(force[1]), fy); force[2] = A(F(force[2]), fz)
        moment[0] = A(M(fy, rz), S(m0, M(fz, ry)))
        moment[1] = A(m1, S(M(fz, rx), M(fx, rz)))
        moment[2] = A(S(M(fx, ry), M(fy, rx)), m2)


cdef void unit_vectors(const Unit* u, double body_u, double flaps, const double* cg, double* force, double* moment) noexcept nogil:
    cdef double ias, thrust, target, omega, t2
    cdef bint active
    if u.rocket:
        ias = M(F(body_u), u.ias_factor)
        nozzle_vectors(u, u.thrust, cg, ias, flaps, force, moment)
        return
    ias = M(body_u, u.ias_factor)
    scalar_update(&u.jet, u.jet.rho, body_u, 0., u.jet.normalized, u.jet.dt, u.jet.afterburner, u.jet.supply, 1.,
                  &thrust, &target, &active)
    omega = pmin(pmax(target, 0.), M(u.jet.omega_limit, F(1.)))
    scalar_update(&u.jet, u.jet.rho, body_u, omega, u.jet.normalized, u.jet.dt, u.jet.afterburner, u.jet.supply,
                  M(F(omega), u.jet.inverse_omega), &thrust, &t2, &active)
    nozzle_vectors(u, thrust, cg, ias, flaps, force, moment)


cdef void set_nozzle(NozzleT* n, dict p) except *:
    for i in range(3):n.position[i] = p['position'][i]
    n.ratio = p['ratio'];n.maximum = p['maximum']
    for i in range(3):
        for j in range(3):n.basis[i][j] = p['basis'][i][j]
    n.main_controls = bool(p['main_controls']);n.tip = bool(p['tip'])
    for i in range(2):
        for j in range(5):n.weights[i][j] = p['weights'][i][j]
        n.ranges[i][0] = p['ranges'][i][0];n.ranges[i][1] = p['ranges'][i][1]
    for i in range(4):n.flaps[i] = p['flaps'][i]
    set_table(&n.roll_angle, p['roll_angle'], 3);set_table(&n.pitch_angle, p['pitch_angle'], 3)
    set_table(&n.yaw_angle, p['yaw_angle'], 3);set_table(&n.vtol_angle, p['vtol_angle'], 2)
    set_table(&n.reverse_angle, p['reverse_angle'], 2);set_table(&n.roll_thrust, p['roll_thrust'], 3)
    set_table(&n.pitch_thrust, p['pitch_thrust'], 3);set_table(&n.yaw_thrust, p['yaw_thrust'], 3)
    set_table(&n.airbrake, p['airbrake'], 2);set_table(&n.vtol, p['vtol'], 2);set_table(&n.reverse, p['reverse'], 2)


cdef void set_unit(Unit* u, unit, double throttle) except *:
    import math
    from component_assembly import f32, mul
    from structural_limits import interval
    from control_mixer import curve as py_curve
    from engine_supply import available_fuel
    if len(unit.nozzles) > 4:raise ValueError('too many nozzles')
    u.nnoz = len(unit.nozzles)
    for k, nz in enumerate(unit.nozzles):set_nozzle(&u.noz[k], nz)
    u.ias_factor = f32(math.sqrt(f32(unit.rho/f32(1.225))))
    if type(unit).__name__ == 'RocketUnit':
        u.rocket = True;u.thrust = float(unit.thrust)
        return
    u.rocket = False
    j = unit.jet
    if len(j['density']) > 16 or len(j['speed']) > 32:raise ValueError('jet table too large')
    u.jet.nd = len(j['density']);u.jet.ns = len(j['speed'])
    for i, d in enumerate(j['density']):u.jet.density[i] = d
    for i, s in enumerate(j['speed']):u.jet.speed[i] = s
    if not j['cells']:raise ValueError('jet table without cells')
    for i, row in enumerate(j['cells']):
        for k, cell in enumerate(row):
            for q in range(4):u.jet.cells[i][k][q] = cell[q]
    for q in range(4):u.jet.bases[q] = j['bases'][q]
    set_table(&u.jet.modes, j['modes'], 5);set_table(&u.jet.throttle, j['throttle'], 1)
    u.jet.throttle_scale = j['throttle_scale'];u.jet.max_omega = j['max_omega'];u.jet.consumption = j['consumption']
    u.jet.tau = j['tau']
    ep = unit.ep;fp = unit.fp
    vtol = min(interval(0., *ep['ias_vtol']), 0.)
    effective = min(py_curve(ep['vtol_throttle'], vtol, 1)[0], interval(0., *ep['reverse_throttle']), f32(throttle))
    u.jet.normalized = min(f32(effective/f32(1.1 if ep['throttle_boost'] > 1. else 1.)), 1.)
    u.jet.supply = available_fuel(fp, unit.system_fuel, unit.accumulator, 1., unit.dt)
    u.jet.omega_limit = ep['omega_limit'];u.jet.inverse_omega = ep['inverse_omega']
    u.jet.dt = unit.dt;u.jet.rho = unit.rho;u.jet.afterburner = bool(unit.afterburner)
