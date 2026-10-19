# Python value record of a native evaluation, with the fields and types of
# em_operating.operating_point(..., stationary_angles=..., _search=True)
# (plus JointProblem.residual's pitch fractions), so the unchanged baseline
# admission (certify, pitch response, Instructor, finish, chart sample) runs
# on it exactly as on a Python evaluation.

cdef void complete_reporting(Model m, const Eval* e, Kinematic* k) noexcept:
    cdef double position[3]
    cdef double velocity[3]
    cdef double vy, s
    cdef const Frame* f = &e.frame
    cdef int i
    k.ps_continuous = fma(f.force[2], e.geom.forward[2], fma(f.force[1], e.geom.forward[1], f.force[0]*e.geom.forward[0]))*e.speed/m.weight
    position[0] = 0.; position[1] = m.altitude; position[2] = 0.
    velocity[0] = e.speed; velocity[1] = 0.; velocity[2] = 0.
    airborne_step(position, velocity, e.geom.quaternion, f.omega, f.force, f.moment, m.mass, m.inertia, m.dt, k)
    vy = k.velocity[1]
    k.velocity[1] = altitude_velocity_correction(k.position[1], vy, m.dt)
    k.altitude_correction = k.velocity[1] != vy
    s = 0.
    for i in range(3):
        s = s+k.velocity[i]*k.velocity[i]
    k.ps = ((k.position[1]-m.altitude)+(s-e.speed*e.speed)/(2*m.G))/m.dt


cdef void ensure_complete(Model m, Eval* e) noexcept:
    if not e.completed:
        complete_reporting(m, e, &e.kin)
        e.completed = True


TAIL_NAMES = ('left_hstab', 'right_hstab', 'vstab', 'fuselage')
COMPONENT_NAMES = ('left_wing', 'right_wing', 'left_hstab', 'right_hstab', 'vstab', 'fuselage', 'chute', 'parasite')


cdef inline list L3(const double* v):
    return [v[0], v[1], v[2]]


cdef inline object A3(const double* v):
    cdef cnp.ndarray out = np.empty(3)
    out[0] = v[0]; out[1] = v[1]; out[2] = v[2]
    return out


cdef dict air_dict(const Air* a):
    return dict(alpha=a.alpha, beta=a.beta, tas=a.tas, speed_squared=a.speed_squared, mach=a.mach, ias_u=a.ias_u,
                density=a.density)


cdef dict history_dict(const History* h):
    return dict(wing_aoa=[h.aoa[0], h.aoa[1]], wing_cl=[h.cl[0], h.cl[1]], body_angles=[h.body[0], h.body[1]],
                spin=h.spin)


cdef dict wing_dict(const Wing* w):
    cdef int i
    flows = [];coefs = []
    for i in range(2):
        flows.append(dict(flow=L3(w.flow[i].flow), alpha=w.flow[i].alpha, reference_angle=w.flow[i].reference_angle,
                          speed_squared=w.flow[i].speed_squared))
        c = &w.coef[i]
        coefs.append(dict(cx=c.cx, cy=c.cy, cy_reference=c.cy_reference, cy_add=c.cy_add, aoa=c.aoa,
                          reference_aoa=c.reference_aoa, raw_cl=c.raw_cl))
    post = dict(xy=[[w.post.xy[0][0], w.post.xy[0][1]], [w.post.xy[1][0], w.post.xy[1][1]]], spin=w.post.spin,
                gyroscopic_scale=w.post.gyroscopic_scale, vertical_area_scale=w.post.vertical_area_scale,
                vertical_control_scale=w.post.vertical_control_scale)
    return dict(forces=[L3(w.forces[0]), L3(w.forces[1])], points=[L3(w.points[0]), L3(w.points[1])],
                base_points=[L3(w.base[0]), L3(w.base[1])], flow=flows, coefficients=coefs, geometry_error=0.,
                q=[w.q[0], w.q[1]], areas=[w.areas[0], w.areas[1]], blend=w.blend, cy=[w.cy[0], w.cy[1]], postprocess=post)


cdef dict tail_dict(const Tail* t):
    cdef int i
    coefficients = {};angles = {};points = {};forces = {};flows = {}
    for i in range(4):
        n = TAIL_NAMES[i]
        coefficients[n] = [t.coeff[i][0], t.coeff[i][1]]
        angles[n] = t.angles[i]
        points[n] = L3(t.points[i])
        forces[n] = L3(t.forces[i])
        flows[n] = L3(t.flows[i])
    if t.type2:
        wake = dict(angles=[t.angles_wake[0], t.angles_wake[1]], projection=[L3(t.projection[0]), L3(t.projection[1])],
                    correction=[t.correction[0], t.correction[1]], previous_cl=[t.previous_cl[0], t.previous_cl[1]])
    else:
        wake = dict(angles=[t.angles_wake[0], t.angles_wake[1]], projection=[], correction=[],
                    previous_cl=[t.previous_cl[0], t.previous_cl[1]])
    return dict(coefficients=coefficients, effective_angles=angles,
                drag_multipliers={n: 1. for n in TAIL_NAMES}, previous_angles=[t.previous_angles[0], t.previous_angles[1]],
                points=points, forces=forces, flows=flows, wake=wake, q_tail=t.q_tail, q_fuselage=t.q_fuselage)


cdef dict value_record(Model m, Eval* e, bint search, bint complete=False):
    cdef const Frame* f = &e.frame
    cdef const Cond* c = &m.conds[e.cond]
    cdef int i
    cdef cnp.ndarray residual = np.empty(5)
    cdef cnp.ndarray rate = np.empty(3)
    cdef cnp.ndarray angle_residual = np.empty(2)
    for i in range(5):residual[i] = e.residual[i]
    for i in range(3):rate[i] = e.rate_residual[i]
    angle_residual[0] = e.angle_residual[0];angle_residual[1] = e.angle_residual[1]
    air = air_dict(&e.air)
    comp_forces = {COMPONENT_NAMES[i]: L3(f.component_forces[i]) for i in range(8)}
    comp_points = {COMPONENT_NAMES[i]: L3(f.component_points[i]) for i in range(7)}
    tail = tail_dict(&f.tail)
    history = history_dict(&f.history)
    r = dict(force=L3(f.force), stored_moment=L3(f.moment), component_forces=comp_forces, component_points=comp_points,
             raw_aero_force=L3(f.raw_force), raw_aero_moment=L3(f.raw_moment), helper_moment=L3(f.helper),
             engine_force=L3(f.engine_force), engine_moment=L3(f.engine_moment), air=air, omega_for_flow=L3(f.omega),
             wing=wing_dict(&f.wing), tail=tail, history=history, engine=None,
             flap_mapping=[c.flap_values[0], c.flap_values[1], c.flap_values[2], c.flap_values[3]],
             controls=dict(wing=[[f.wc[0][i] for i in range(5)], [f.wc[1][i] for i in range(5)]],
                           tail=dict(left_hstab=[f.tc[0][i] for i in range(5)], right_hstab=[f.tc[1][i] for i in range(5)],
                                     vstab=[f.tc[2][i] for i in range(5)])))
    g = &e.geom
    geometry = dict(forward=A3(g.forward), normal=A3(g.normal), side=A3(g.side), up=A3(g.up), lateral=A3(g.lateral),
                    omega=A3(g.omega), quaternion=[g.quaternion[0], g.quaternion[1], g.quaternion[2], g.quaternion[3]],
                    turn_rate=g.turn_rate)
    a = &e.alloc
    allocation = dict(commands=L3(a.commands), allocated_commands=L3(a.delivered), sticks=L3(a.sticks), trim=L3(a.trim),
                      ranges=[[a.ranges[i][0], a.ranges[i][1]] for i in range(3)],
                      bounds=[[a.bounds[i][0], a.bounds[i][1]] for i in range(3)], reachable=bool(a.reachable),
                      allocation_error=a.allocation_error)
    value = dict(speed=float(e.speed), residual=residual, rate_residual=rate, force_error_g=e.force_error_g,
                 force_mean_uncertainty_g=0., angular_mean_uncertainty_rad_s2=np.asarray([0., 0., 0.]),
                 equilibrium_coordinates=[e.x[0], e.x[1], e.x[2], e.x[3], e.x[4]], equilibrium_load_g=float(e.load),
                 history_error=e.history_error, history_input=history_dict(&e.cycle_input), result=r, geometry=geometry,
                 propulsion_phase_frames=0,
                 phase_results=[dict(air=air, history=history)] if m.instructor else [],
                 maximum_wing_load_ratios=[e.wing_ratios[0], e.wing_ratios[1]],
                 allocation=allocation, velocity=L3(e.velocity), stall_margin=e.stall_margin,
                 negative_stall_margin=e.negative_stall_margin, flaps=m.flaps, gear=m.gear, instructor=None, propulsion=None,
                 ps=None, ps_continuous=None, kinematic=None, altitude_correction=None, history_iterations=1,
                 history_closure_required=True, search_only=bool(search), steady_history_attempted=False,
                 steady_history_used=False, stationary_angle_residual=angle_residual, explicit_stationary_history=True)
    if complete:
        ensure_complete(m, e)
        k = &e.kin
        value.update(ps=k.ps, ps_continuous=k.ps_continuous, altitude_correction=bool(k.altitude_correction), search_only=False,
                     kinematic=dict(position=L3(k.position), velocity=L3(k.velocity),
                                    quaternion=[k.quaternion[0], k.quaternion[1], k.quaternion[2], k.quaternion[3]],
                                    omega=L3(k.omega), angular_acceleration=L3(k.angular_acceleration),
                                    world_acceleration=L3(k.world_acceleration), reset=bool(k.reset)))
    return value
