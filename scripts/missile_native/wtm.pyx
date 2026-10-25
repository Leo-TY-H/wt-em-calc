# cython: language_level=3
"""Python boundary of the native missile flight model.

All numerical work happens in the C headers included below; this file only
converts between the reference model's Python dictionaries and C structs.
"""
from copy import deepcopy
import math
from libc.stdlib cimport malloc, free

cdef extern from "body.h":
    ctypedef struct v3:
        float x, y, z
    ctypedef struct q4:
        float x, y, z, w
    ctypedef struct wtm_air:
        float density, sound_speed, pressure
    ctypedef struct wtm_trig:
        float sine, cosine
        long long quadrant
        float reduced
    int wtm_error_code
    const char *wtm_error_text
    wtm_air atmosphere(float h)
    float drag_mach(float m)
    float atan2f_native(float y, float x)
    float tanf_native(float x)
    wtm_trig half_angle_trig(float increment)
    v3 rotate_thrust(q4 q, v3 t)

    ctypedef struct wtm_impulse:
        float duration, force, mass_lost, vectoring
    ctypedef struct wtm_propulsion:
        float delay
        int count
        wtm_impulse impulses[4]
    ctypedef struct wtm_motors:
        int count
        wtm_propulsion items[4]
    ctypedef struct wtm_rocket:
        float ext_pressure[4]
        float thrust_deviation, orientation_torque
    ctypedef struct wtm_row:
        float x, slope, y
    ctypedef struct wtm_body_props:
        float mass, front_area, side_area, stabilizer_arm, length, cx, cy, cy_limit, cx_aoa
        float fins_hor, fins_ver, fin_pressure_limit, damping_geometry
        float inertia[3]
        float angular_damping[3]
        q4 axis_quaternion
        int cy_rows
        wtm_row cy_table[64]
    ctypedef struct wtm_body:
        v3 position
        q4 quaternion
        v3 velocity
        v3 omega
        float time
        unsigned int seed
        float body_random
        float clocks[4]
        int water
        float distance, water_distance, surface_depth, immersion
    ctypedef struct wtm_aero_eval:
        v3 drag, lift, force
        float cosine, cd, cy
    ctypedef struct wtm_forces:
        v3 frame[3]
        v3 axes[3]
        float cm_speed, mach, pressure
        v3 local_flow, flow
        wtm_aero_eval baseline, steering
        int fin_active, fin_limited
        float deflection[2]
        v3 fin_flow
        v3 moment
        float damping[3]
        int damping_clipped[3]
        v3 angular_acceleration_before_environment, angular_acceleration
        double acceleration[3]
        float effective_mass
    ctypedef struct wtm_propulsion_out:
        v3 world_force, body_torque
        float mass_lost
        float clock_rates[4]
        unsigned int seed
    ctypedef struct wtm_flight_props:
        const wtm_body_props *body
        const wtm_rocket *rocket
        const wtm_motors *motors
        int precision
        float end_speed
    ctypedef struct wtm_body_step_out:
        wtm_propulsion_out propulsion
        float target_time
    ctypedef struct wtm_substep:
        wtm_body state
        double acceleration[3]
        v3 angular_acceleration
    void aero_forces(const wtm_body_props *p, float height, v3 velocity, q4 q, v3 omega, float fin0, float fin1,
        float dt, v3 force, v3 torque, float mass_lost, float perturbation, float body_random, wtm_forces *o)
    void body_advance(const wtm_flight_props *p, wtm_body *b, const float *controls, float dt, wtm_body_step_out *out,
        wtm_substep *substeps)
    v3 cap_velocity(v3 v, float c)


cdef extern from "controls.h":
    v3 boresight(q4 q, const float *angles)

    ctypedef struct wtm_orientation_props:
        float start, duration, final_speed, final_distance, depth_min
        float elevation[4]
        float heading[4]
        float angle_to_rate
        float coefficients[5]
        int use_fins
    ctypedef struct wtm_orientation_state:
        int enabled
        float pid[2][8]
    ctypedef struct wtm_orientation_out:
        int override
        float fins[2]
        float auxiliary[2]
        int active
        float angles[2]
        float elevation
        float errors[2]
    wtm_orientation_out orientation_update(const wtm_orientation_props *p, wtm_orientation_state *s, float time,
        float range_squared, q4 q, v3 omega, const float *angles_in, float depth, float dt)

    ctypedef struct wtm_table:
        int count
        wtm_row rows[32]
    ctypedef struct wtm_guidance_props:
        int loft_enabled
        float range_elevation[4]
        float loft_elevation, loft_rate_squared, loft_accel, hold_time, hold_elevation, hold_rate
        wtm_table time_gain
        wtm_table time_to_hit_gain
        float elevation_add, angle_gain, omega_gain, course_gain, course_final_angle
        int absolute_altitude
        wtm_table altitude_table
        float altitude_diff_gain, altitude_rate_gain, climb_accel_gain
        float climb_limits[2]
    ctypedef struct wtm_request:
        int mode, course
        float gain
        int has_acceleration
        v3 acceleration
        float target_pitch, target_yaw
        float command[3]
    wtm_request guidance_request(const wtm_guidance_props *p, int mode, int course, v3 los, v3 rate, v3 navigation_velocity,
        q4 quaternion, v3 velocity, v3 position, float time, float time_to_hit, float range_squared, int has_height, float height_query)

    q4 control_frame(q4 q, v3 velocity, int velocity_reference, int matrix)

    ctypedef struct wtm_acceleration_props:
        int velocity_frame
        float max_accel
        int limit_aoa
        float aoa_max, base_speed_squared
        int schedule_count
        float schedule[32][3]
        int coefficient_count
        float coefficients[16][4]
    ctypedef struct wtm_aero_props:
        float mass, cy, cy_limit, side_area
        int cy_rows
        wtm_row cy_table[64]
    ctypedef struct wtm_motor_scalar:
        float thrust, mass_lost, vectoring
    wtm_motor_scalar motor_scalar(const wtm_motors *motors, const float *clocks)
    ctypedef struct wtm_acceleration_out:
        q4 frame
        int has_fins
        float fins[2]
        float errors[2]
        float limited[2]
        int has_aoa_limit
        float aoa_limit, scale, scale_squared
        float coefficients[4]
    int acceleration_update(const wtm_acceleration_props *p, const wtm_aero_props *aero, wtm_motor_scalar motor,
        float *state, v3 request, v3 measured, q4 q, v3 velocity, float height, float time, float dt, int matrix,
        wtm_acceleration_out *o)

    ctypedef struct wtm_stage:
        float start
        float altitude[2]
        float speed_squared, closure_min, tgo_min
        int use_ias
        float cruise, cruise_error_limit, base_ias_squared
        float coefficients[4]
        float pursuit_closure[2]
        float terminal_tgo
    ctypedef struct wtm_propulsion_control_props:
        int count
        wtm_stage stages[4]
    ctypedef struct wtm_propulsion_control_state:
        int stages[4]
        float pid[8]
    void propulsion_control_update(const wtm_propulsion_control_props *p, wtm_propulsion_control_state *s, float time,
        v3 position, v3 velocity, float tgo, float closure, float dt, float *factors)


cdef inline v3 _v3(object v):
    cdef v3 out
    out.x = v[0]; out.y = v[1]; out.z = v[2]
    return out

cdef inline q4 _q4(object v):
    cdef q4 out
    out.x = v[0]; out.y = v[1]; out.z = v[2]; out.w = v[3]
    return out

cdef inline list _list3(v3 v):
    return [<double>v.x, <double>v.y, <double>v.z]


def _check():
    global wtm_error_code
    if wtm_error_code:
        code = wtm_error_code; text = (<bytes>wtm_error_text).decode() if wtm_error_text != NULL else ''
        wtm_error_code = 0
        if code == 1: raise ZeroDivisionError(text)
        if code == 3: raise NotImplementedError(text)
        raise ValueError(text)


# ---------------------------------------------------------------- core tests
def core_atmosphere(double h):
    cdef wtm_air a = atmosphere(<float>h)
    _check()
    return dict(density=<double>a.density, sound_speed=<double>a.sound_speed, pressure=<double>a.pressure)

def core_drag_mach(double m):
    cdef float r = drag_mach(<float>m)
    _check(); return <double>r

def core_atan2f(double y, double x):
    cdef float r = atan2f_native(<float>y, <float>x)
    _check(); return <double>r

def core_tanf(double x):
    cdef float r = tanf_native(<float>x)
    _check(); return <double>r

def core_half_angle_trig(double increment):
    cdef wtm_trig t = half_angle_trig(<float>increment)
    _check()
    return dict(sine=<double>t.sine, cosine=<double>t.cosine, quadrant=t.quadrant, reduced=<double>t.reduced)

def core_rotate_thrust(q, t):
    cdef v3 r = rotate_thrust(_q4(q), _v3(t))
    _check(); return _list3(r)


# ------------------------------------------------------------ conversions
cdef inline list _list4(q4 q):
    return [<double>q.x, <double>q.y, <double>q.z, <double>q.w]

cdef int _body_props(object d, wtm_body_props *p) except -1:
    p.mass = d['mass']; p.front_area = d['front_area']; p.side_area = d['side_area']; p.stabilizer_arm = d['stabilizer_arm']
    p.length = d['length']; p.cx = d['cx']; p.cy = d['cy']; p.cy_limit = d['cy_limit']; p.cx_aoa = d['cx_aoa']
    p.fins_hor = d['fins_hor']; p.fins_ver = d['fins_ver']; p.fin_pressure_limit = d['fin_pressure_limit']
    p.damping_geometry = d['damping_geometry']
    for i in range(3):
        p.inertia[i] = d['inertia'][i]; p.angular_damping[i] = d['angular_damping'][i]
    p.axis_quaternion = _q4(d['axis_quaternion'])
    rows = d['cy_table']
    if len(rows) > 64: raise ValueError('cy table too long')
    p.cy_rows = len(rows)
    for i, row in enumerate(rows):
        p.cy_table[i].x = row[0]; p.cy_table[i].slope = row[1]; p.cy_table[i].y = row[2]
    return 0

cdef int _rocket(object r, wtm_rocket *out) except -1:
    mapping = r.get('extPressureToThrustMult', [0., 1., 0., 1.])
    for i in range(4): out.ext_pressure[i] = mapping[i]
    out.thrust_deviation = r.get('thrustDeviation', 0.)
    out.orientation_torque = r.get('orientationTorque', 0.)
    return 0

def _field(item, key):
    return getattr(item, key) if hasattr(item, key) else item[key]

cdef int _motors(object motors, wtm_motors *out) except -1:
    if len(motors) > 4: raise ValueError('At most four propulsion units')
    out.count = len(motors)
    for i, m in enumerate(motors):
        impulses = _field(m, 'impulses')
        out.items[i].delay = _field(m, 'delay')
        if len(impulses) > 4: raise ValueError('At most four impulses')
        out.items[i].count = len(impulses)
        for j, item in enumerate(impulses):
            if _field(item, 'factor_index') >= 0: raise NotImplementedError('Propulsion factor tables need their native loader')
            out.items[i].impulses[j].duration = _field(item, 'duration'); out.items[i].impulses[j].force = _field(item, 'force')
            out.items[i].impulses[j].mass_lost = _field(item, 'mass_lost'); out.items[i].impulses[j].vectoring = _field(item, 'vectoring')
    return 0

cdef int _body(object d, wtm_body *b) except -1:
    b.position = _v3(d['position']); b.quaternion = _q4(d['quaternion']); b.velocity = _v3(d['velocity'])
    b.omega = _v3(d['omega']); b.time = d['time']; b.seed = d['seed']; b.body_random = d['body_random']
    for i in range(4): b.clocks[i] = d['clocks'][i]
    b.water = bool(d['water']); b.distance = d['distance']; b.water_distance = d['water_distance']
    b.surface_depth = d['surface_depth']; b.immersion = d['immersion']
    return 0

cdef dict _body_dict(wtm_body *b):
    return dict(position=_list3(b.position), quaternion=_list4(b.quaternion), velocity=_list3(b.velocity), omega=_list3(b.omega),
                time=<double>b.time, seed=b.seed, body_random=<double>b.body_random, clocks=[<double>b.clocks[i] for i in range(4)],
                water=bool(b.water), distance=<double>b.distance, water_distance=<double>b.water_distance,
                surface_depth=<double>b.surface_depth, immersion=<double>b.immersion)

cdef dict _eval_dict(wtm_aero_eval *e):
    return dict(drag=_list3(e.drag), lift=_list3(e.lift), force=_list3(e.force), cosine=<double>e.cosine, cd=<double>e.cd, cy=<double>e.cy)


# ------------------------------------------------------------ body tests
def body_forces(props, height, velocity, q, omega, fins, dt, force, torque, mass_lost, body_random, perturbation=0.):
    cdef wtm_body_props p
    cdef wtm_forces f
    _body_props(props, &p)
    aero_forces(&p, height, _v3(velocity), _q4(q), _v3(omega), fins[0], fins[1], dt, _v3(force), _v3(torque), mass_lost, perturbation, body_random, &f)
    _check()
    return dict(frame=[_list3(f.frame[i]) for i in range(3)], axes=[_list3(f.axes[i]) for i in range(3)], cm_speed=<double>f.cm_speed,
                mach=<double>f.mach, pressure=<double>f.pressure, local_flow=_list3(f.local_flow), flow=_list3(f.flow),
                baseline=_eval_dict(&f.baseline), fin_active=bool(f.fin_active), fin_limited=bool(f.fin_limited),
                deflection=[<double>f.deflection[0], <double>f.deflection[1]], fin_flow=_list3(f.fin_flow), steering=_eval_dict(&f.steering),
                moment=_list3(f.moment), damping=[<double>f.damping[i] for i in range(3)], damping_clipped=[bool(f.damping_clipped[i]) for i in range(3)],
                angular_acceleration_before_environment=_list3(f.angular_acceleration_before_environment),
                angular_acceleration=_list3(f.angular_acceleration), acceleration=[f.acceleration[i] for i in range(3)],
                effective_mass=<double>f.effective_mass)

def body_step(p, body, controls, dt):
    """Body half of optical_point_flight.update; returns (body, propulsion)."""
    cdef wtm_body_props bp
    cdef wtm_rocket rocket
    cdef wtm_motors motors
    cdef wtm_flight_props fp
    cdef wtm_body b
    cdef wtm_body_step_out out
    cdef float c[8]
    _body_props(p['body'], &bp); _rocket(p['rocket'], &rocket); _motors(p['guidance']['motors'], &motors)
    fp.body = &bp; fp.rocket = &rocket; fp.motors = &motors; fp.precision = p['precision']; fp.end_speed = p['end_speed']
    _body(body, &b)
    for i in range(8): c[i] = controls[i]
    body_advance(&fp, &b, c, dt, &out, NULL)
    _check()
    return _body_dict(&b), dict(world_force=_list3(out.propulsion.world_force), body_torque=_list3(out.propulsion.body_torque),
        mass_lost=<double>out.propulsion.mass_lost, clock_rates=[<double>out.propulsion.clock_rates[i] for i in range(4)], seed=out.propulsion.seed)

def body_cap(velocity, end_speed):
    cdef v3 result = cap_velocity(_v3(velocity), end_speed)
    _check()
    return _list3(result)


# ------------------------------------------------- controller conversions
cdef int _table(object rows, wtm_table *t) except -1:
    if len(rows) > 32: raise ValueError('guidance table too long')
    t.count = len(rows)
    for i, row in enumerate(rows):
        t.rows[i].x = row[0]; t.rows[i].slope = row[1]; t.rows[i].y = row[2]
    return 0

cdef int _orientation_props(object d, wtm_orientation_props *p) except -1:
    p.start = d['start']; p.duration = d['duration']; p.final_speed = d['final_speed']; p.final_distance = d['final_distance']
    p.depth_min = d['depth_min']; p.angle_to_rate = d['angle_to_rate']; p.use_fins = bool(d['use_fins'])
    for i in range(4):
        p.elevation[i] = d['elevation'][i]; p.heading[i] = d['heading'][i]
    for i in range(5): p.coefficients[i] = d['coefficients'][i]
    return 0

cdef int _guidance_props(object d, wtm_guidance_props *p) except -1:
    p.loft_enabled = bool(d['loft_enabled'])
    for i in range(4): p.range_elevation[i] = d['range_elevation'][i]
    p.loft_elevation = d['loft_elevation']; p.loft_rate_squared = d['loft_rate_squared']; p.loft_accel = d['loft_accel']
    p.hold_time = d['hold_time']; p.hold_elevation = d['hold_elevation']; p.hold_rate = d['hold_rate']
    _table(d['time_gain'], &p.time_gain); _table(d['time_to_hit_gain'], &p.time_to_hit_gain)
    p.elevation_add = d['elevation_add']; p.angle_gain = d['angle_gain']; p.omega_gain = d['omega_gain']
    p.course_gain = d['course_gain']; p.course_final_angle = d['course_final_angle']
    p.absolute_altitude = bool(d['absolute_altitude']); _table(d['altitude_table'], &p.altitude_table)
    p.altitude_diff_gain = d['altitude_diff_gain']; p.altitude_rate_gain = d['altitude_rate_gain']
    p.climb_accel_gain = d['climb_accel_gain']; p.climb_limits[0] = d['climb_limits'][0]; p.climb_limits[1] = d['climb_limits'][1]
    return 0

cdef int _acceleration_props(object d, wtm_acceleration_props *p) except -1:
    p.velocity_frame = bool(d['velocity_frame']); p.max_accel = d['max_accel']; p.limit_aoa = bool(d['limit_aoa'])
    p.aoa_max = d['aoa_max']; p.base_speed_squared = d['base_speed_squared']
    schedule = d['schedule']; coefficients = d['coefficients']
    if len(schedule) > 32 or len(coefficients) > 16: raise ValueError('acceleration schedule too long')
    p.schedule_count = len(schedule); p.coefficient_count = len(coefficients)
    for i, row in enumerate(coefficients):
        if len(row) != 4: raise ValueError('Four PID coefficients required')
        for k in range(4): p.coefficients[i][k] = row[k]
    for i, row in enumerate(schedule):
        # The reference indexes its coefficient list with this value.
        index = row[2]
        if type(index) is not int or not 0 <= index < len(coefficients): raise IndexError('Invalid PID coefficient index')
        p.schedule[i][0] = row[0]; p.schedule[i][1] = row[1]; p.schedule[i][2] = index
    return 0

cdef int _aero_props(object d, wtm_aero_props *p) except -1:
    p.mass = d['mass']; p.cy = d['cy']; p.cy_limit = d['cy_limit']; p.side_area = d['side_area']
    rows = d['cy_table']
    if len(rows) > 64: raise ValueError('cy table too long')
    p.cy_rows = len(rows)
    for i, row in enumerate(rows):
        p.cy_table[i].x = row[0]; p.cy_table[i].slope = row[1]; p.cy_table[i].y = row[2]
    return 0

cdef int _propulsion_props(object stages, wtm_propulsion_control_props *p) except -1:
    if len(stages) > 4: raise ValueError('Native controller supports at most four stages')
    p.count = len(stages)
    for i, c in enumerate(stages):
        p.stages[i].start = c['start']; p.stages[i].altitude[0] = c['altitude'][0]; p.stages[i].altitude[1] = c['altitude'][1]
        p.stages[i].speed_squared = c['speed_squared']; p.stages[i].closure_min = c['closure_min']; p.stages[i].tgo_min = c['tgo_min']
        p.stages[i].use_ias = bool(c['use_ias']); p.stages[i].cruise = c['cruise']
        p.stages[i].cruise_error_limit = c['cruise_error_limit']; p.stages[i].base_ias_squared = c['base_ias_squared']
        for k in range(4): p.stages[i].coefficients[k] = c['coefficients'][k]
        p.stages[i].pursuit_closure[0] = c['pursuit_closure'][0]; p.stages[i].pursuit_closure[1] = c['pursuit_closure'][1]
        p.stages[i].terminal_tgo = c['terminal_tgo']
    return 0


# ------------------------------------------------------- controller tests
# Each returns the same dictionary as its scripts/missile_model counterpart.
def controls_motor(propulsions, clocks):
    """kernels.motor_scalar, converting the motor layout on every call."""
    cdef wtm_motors motors
    if len(propulsions) != len(clocks): raise ValueError('One clock per propulsion is required')
    _motors(propulsions, &motors)
    return _motor_run(&motors, clocks)

cdef dict _motor_run(const wtm_motors *motors, object clocks):
    cdef float times[4]
    cdef wtm_motor_scalar result
    cdef int i
    if motors.count != len(clocks): raise ValueError('One clock per propulsion is required')
    for i in range(motors.count): times[i] = clocks[i]
    result = motor_scalar(motors, times)
    _check()
    return dict(thrust=<double>result.thrust, mass_lost=<double>result.mass_lost, vectoring=<double>result.vectoring)

def controls_boresight(q, angles):
    """optical_geometry.boresight"""
    cdef float a[2]
    a[0] = angles[0]; a[1] = angles[1]
    cdef v3 r = boresight(_q4(q), a)
    _check(); return _list3(r)

def controls_orientation(p, state, time, range_squared, q, omega, angles, depth, dt):
    """orientation_control.update"""
    cdef wtm_orientation_props op
    _orientation_props(p, &op)
    return _orientation_run(&op, state, time, range_squared, q, omega, angles, depth, dt)

cdef object _orientation_run(const wtm_orientation_props *op, state, time, range_squared, q, omega, angles, depth, dt):
    cdef wtm_orientation_state st
    cdef wtm_orientation_out r
    cdef float a[2]
    cdef int i, j
    if len(state['pid']) != 2: raise ValueError('Two PID records required')
    st.enabled = bool(state['enabled'])
    for i in range(2):
        for j in range(8): st.pid[i][j] = state['pid'][i][j]
    a[0] = angles[0]; a[1] = angles[1]
    r = orientation_update(op, &st, time, range_squared, _q4(q), _v3(omega), a, depth, dt)
    _check()
    s = deepcopy(state)
    if s['enabled'] and not st.enabled: s['enabled'] = False
    result = dict(state=s, override=bool(r.override), fins=[0., 0.], auxiliary=[0., 0.], active=False)
    if not r.active: return result
    pid = []
    for i in range(2):
        pid.append([<double>st.pid[i][j] for j in range(8)])
    s['pid'] = pid
    if op.use_fins: result['fins'] = [<double>r.fins[0], <double>r.fins[1]]
    else: result['auxiliary'] = [<double>r.auxiliary[0], <double>r.auxiliary[1]]
    result.update(active=True, angles=[<double>r.angles[0], <double>r.angles[1]], elevation=<double>r.elevation,
                  errors=[<double>r.errors[0], <double>r.errors[1]])
    return result

def controls_request(p, mode, course, los, rate, navigation_velocity, quaternion, velocity, position, time, time_to_hit,
                     range_squared, height_query=None):
    """guidance_request.request (height_query None = unsuccessful height query)"""
    cdef wtm_guidance_props gp
    _guidance_props(p, &gp)
    return _request_run(&gp, mode, course, los, rate, navigation_velocity, quaternion, velocity, position, time, time_to_hit,
                        range_squared, height_query)

cdef object _request_run(const wtm_guidance_props *gp, mode, course, los, rate, navigation_velocity, quaternion, velocity, position,
                         time, time_to_hit, range_squared, height_query):
    cdef wtm_request r
    cdef int has_height = height_query is not None
    cdef float height = 0.
    if has_height: height = height_query
    r = guidance_request(gp, mode, course, _v3(los), _v3(rate), _v3(navigation_velocity), _q4(quaternion), _v3(velocity),
                         _v3(position), time, time_to_hit, range_squared, has_height, height)
    _check()
    if not r.has_acceleration: return dict(mode=mode, course=course, gain=<double>r.gain, acceleration=[])
    return dict(mode=r.mode, course=r.course, gain=<double>r.gain, acceleration=_list3(r.acceleration),
                target_pitch=<double>r.target_pitch, target_yaw=<double>r.target_yaw,
                angular_command=[<double>r.command[0], <double>r.command[1], <double>r.command[2]])

def controls_frame(q, velocity, velocity_reference=True, matrix=True):
    """control_frame.frame (matrix=False: alternate_control_frame.frame)"""
    cdef q4 r = control_frame(_q4(q), _v3(velocity), bool(velocity_reference), bool(matrix))
    _check(); return _list4(r)

def controls_acceleration(p, aero, motor, state, request, measured, q, velocity, height, time, dt, matrix_velocity_frame=True):
    """acceleration_control.update with the default environment; motor is a
    kernels.motor_scalar dictionary or None."""
    cdef wtm_acceleration_props ap
    cdef wtm_aero_props bp
    cdef const wtm_aero_props *aero_pointer = NULL
    _acceleration_props(p, &ap)
    if aero is not None:
        _aero_props(aero, &bp); aero_pointer = &bp
    return _acceleration_run(&ap, aero_pointer, motor, state, request, measured, q, velocity, height, time, dt, matrix_velocity_frame)

cdef object _acceleration_run(const wtm_acceleration_props *ap, const wtm_aero_props *aero_pointer, motor, state, request, measured,
                              q, velocity, height, time, dt, matrix_velocity_frame):
    cdef wtm_motor_scalar m
    cdef wtm_acceleration_out o
    cdef float pid[16]
    cdef int i, k
    m.thrust = 0.; m.mass_lost = 0.; m.vectoring = 0.
    if motor:
        m.thrust = motor['thrust']; m.mass_lost = motor['mass_lost']
    if len(state) != 2: raise ValueError('Two PID records required')
    for i in range(2):
        for k in range(8): pid[8 * i + k] = state[i][k]
    acceleration_update(ap, aero_pointer, m, pid, _v3(request), _v3(measured), _q4(q), _v3(velocity), height, time, dt,
                        bool(matrix_velocity_frame), &o)
    _check()
    frame = _list4(o.frame)
    if not o.has_fins: return dict(frame=frame, fins=None, state=state)
    states = []
    for i in range(2):
        states.append([<double>pid[8 * i + k] for k in range(8)])
    aoa_limit = None
    if o.has_aoa_limit: aoa_limit = <double>o.aoa_limit
    return dict(frame=frame, fins=[<double>o.fins[0], <double>o.fins[1]], state=states,
                errors=[<double>o.errors[0], <double>o.errors[1]], limited_request=[<double>o.limited[0], <double>o.limited[1]],
                aoa_limit=aoa_limit, scale=<double>o.scale, scale_squared=<double>o.scale_squared,
                coefficients=[<double>o.coefficients[k] for k in range(4)])

def controls_propulsion(p, state, time, position, velocity, tgo, closure, dt):
    """propulsion_control.update with the default environment"""
    cdef wtm_propulsion_control_props pp
    if len(p) > 4: raise ValueError('Native controller supports at most four stages')
    if len(state['stages']) != 4 or len(state['pid']) != 8: raise ValueError('Four enums and one eight-float shared PID required')
    if not p: return dict(state=deepcopy(state), factors=[1., 1., 1., 1.])
    _propulsion_props(p, &pp)
    return _propulsion_run(&pp, p, state, time, position, velocity, tgo, closure, dt)

cdef object _propulsion_run(const wtm_propulsion_control_props *pp, p, state, time, position, velocity, tgo, closure, dt):
    """Stages already validated and non-empty."""
    cdef wtm_propulsion_control_state st
    cdef float factors[4]
    cdef int i, cruising = 0
    s = deepcopy(state)
    for i in range(4): st.stages[i] = state['stages'][i]
    for i in range(8): st.pid[i] = state['pid'][i]
    propulsion_control_update(pp, &st, time, _v3(position), _v3(velocity), tgo, closure, dt, factors)
    _check()
    # Only a stage entering in cruise (mode 2) rewrites the shared PID record.
    for i in range(len(p)):
        if state['stages'][i] == 2: cruising = 1
    if cruising: s['pid'] = [<double>st.pid[i] for i in range(8)]
    s['stages'] = [st.stages[i] for i in range(4)]
    return dict(state=s, factors=[<double>factors[i] for i in range(4)])


# ------------------------------------------------------------ flight path
# Converted property blocks of the website's owned flights, keyed by object
# identity. Each entry keeps its source objects alive, so a cached id is never
# reused by another object. The website treats flight properties as immutable
# within a flight (missile_fast); other callers use the controls_* entry points
# or the reference modules, which convert on every call.
cdef dict _prepared = {}

cdef int _remember(object key, object value) except -1:
    if len(_prepared) >= 64: _prepared.clear()
    _prepared[key] = value
    return 0

cdef class _Flight:
    cdef wtm_body_props body
    cdef wtm_rocket rocket
    cdef wtm_motors motors
    cdef object sources

cdef class _Motors:
    cdef wtm_motors p
    cdef object source

cdef class _Orientation:
    cdef wtm_orientation_props p
    cdef object source

cdef class _Guidance:
    cdef wtm_guidance_props p
    cdef object source

cdef class _Acceleration:
    cdef wtm_acceleration_props p
    cdef wtm_aero_props aero
    cdef int has_aero
    cdef object sources

cdef class _Propulsion:
    cdef wtm_propulsion_control_props p
    cdef object source

cdef _Flight _flight_for(object p):
    cdef _Flight r
    # optical_point_flight passes the owned motor layout when present; otherwise
    # the reference rebuilds it from the rocket, which FlightSession has already
    # checked equal to guidance.motors.
    motors = p.get('_motor_properties')
    if motors is None: motors = p['guidance']['motors']
    body = p['body']; rocket = p['rocket']
    key = ('flight', id(body), id(rocket), id(motors))
    r = _prepared.get(key)
    if r is None:
        r = _Flight.__new__(_Flight)
        _body_props(body, &r.body); _rocket(rocket, &r.rocket); _motors(motors, &r.motors)
        r.sources = (body, rocket, motors)
        _remember(key, r)
    return r

cdef _Orientation _orientation_for(object d):
    cdef _Orientation r
    key = ('orientation', id(d))
    r = _prepared.get(key)
    if r is None:
        r = _Orientation.__new__(_Orientation)
        _orientation_props(d, &r.p); r.source = d
        _remember(key, r)
    return r

cdef _Motors _motors_for(object propulsions):
    cdef _Motors r
    key = ('motors', id(propulsions))
    r = _prepared.get(key)
    if r is None:
        r = _Motors.__new__(_Motors)
        _motors(propulsions, &r.p); r.source = propulsions
        _remember(key, r)
    return r

cdef _Guidance _guidance_for(object d):
    cdef _Guidance r
    key = ('guidance', id(d))
    r = _prepared.get(key)
    if r is None:
        r = _Guidance.__new__(_Guidance)
        _guidance_props(d, &r.p); r.source = d
        _remember(key, r)
    return r

cdef _Acceleration _acceleration_for(object d, object aero):
    cdef _Acceleration r
    key = ('acceleration', id(d), id(aero))
    r = _prepared.get(key)
    if r is None:
        r = _Acceleration.__new__(_Acceleration)
        _acceleration_props(d, &r.p)
        r.has_aero = aero is not None
        if r.has_aero: _aero_props(aero, &r.aero)
        r.sources = (d, aero)
        _remember(key, r)
    return r

cdef _Propulsion _propulsion_for(object stages):
    cdef _Propulsion r
    key = ('propulsion', id(stages))
    r = _prepared.get(key)
    if r is None:
        r = _Propulsion.__new__(_Propulsion)
        _propulsion_props(stages, &r.p); r.source = stages
        _remember(key, r)
    return r

def clear_prepared():
    """Forget converted property blocks (tests that mutate properties in place)."""
    _prepared.clear()


# Same signatures and results as the reference controllers (default environment).
def flight_motor(propulsions, clocks):
    """kernels.motor_scalar with an immutable motor layout cached by identity."""
    cdef _Motors r
    if len(propulsions) != len(clocks): raise ValueError('One clock per propulsion is required')
    r = _motors_for(propulsions)
    return _motor_run(&r.p, clocks)

def flight_orientation(p, state, time, range_squared, q, omega, angles, depth, dt):
    """orientation_control.update"""
    cdef _Orientation r = _orientation_for(p)
    return _orientation_run(&r.p, state, time, range_squared, q, omega, angles, depth, dt)

def flight_request(p, mode, course, los, rate, navigation_velocity, quaternion, velocity, position, time, time_to_hit,
                   range_squared, height_query=None):
    """guidance_request.request"""
    cdef _Guidance r = _guidance_for(p)
    return _request_run(&r.p, mode, course, los, rate, navigation_velocity, quaternion, velocity, position, time, time_to_hit,
                        range_squared, height_query)

def flight_acceleration(p, aero, motor, state, request, measured, q, velocity, height, time, dt, matrix_velocity_frame=True):
    """acceleration_control.update"""
    cdef _Acceleration r = _acceleration_for(p, aero)
    cdef const wtm_aero_props *aero_pointer = NULL
    if r.has_aero: aero_pointer = &r.aero
    return _acceleration_run(&r.p, aero_pointer, motor, state, request, measured, q, velocity, height, time, dt, matrix_velocity_frame)

def flight_propulsion(p, state, time, position, velocity, tgo, closure, dt):
    """propulsion_control.update"""
    cdef _Propulsion r
    if len(p) > 4: raise ValueError('Native controller supports at most four stages')
    if len(state['stages']) != 4 or len(state['pid']) != 8: raise ValueError('Four enums and one eight-float shared PID required')
    if not p: return dict(state=deepcopy(state), factors=[1., 1., 1., 1.])
    r = _propulsion_for(p)
    return _propulsion_run(&r.p, p, state, time, position, velocity, tgo, closure, dt)


cdef list _packet(object b):
    return b['position'] + b['quaternion'] + b['velocity'] + b['omega'] + [b['time']]

cdef int _store_body(object d, wtm_body *b) except -1:
    """Fields body_integration.integrate replaces (pure air: water_distance kept)."""
    d['position'] = _list3(b.position); d['quaternion'] = _list4(b.quaternion); d['velocity'] = _list3(b.velocity)
    d['omega'] = _list3(b.omega); d['time'] = <double>b.time
    d['clocks'] = [<double>b.clocks[0], <double>b.clocks[1], <double>b.clocks[2], <double>b.clocks[3]]
    d['distance'] = <double>b.distance; d['immersion'] = <double>b.immersion
    return 0

def flight_update(p, state, targets, dt, guidance_step):
    """optical_point_flight.update with the native motor, body substeps and speed
    cap. guidance_step (seeker, manager and controllers) remains a callback and
    receives exactly the reference arguments. Same result dictionary."""
    cdef _Flight props
    cdef wtm_flight_props fp
    cdef wtm_body b
    cdef wtm_body_step_out out
    cdef wtm_substep *steps
    cdef float c[8]
    cdef float step = dt
    cdef float launch
    cdef int precision, i
    if not math.isfinite(step) or step <= 0.: raise ValueError('Positive finite caller interval required')
    if p['precision'] <= 0: raise ValueError('Positive body substep count required')
    matrix_velocity_frame = p.get('matrix_velocity_frame', True)
    if type(matrix_velocity_frame) is not bool: raise ValueError('matrix_velocity_frame must be an explicit boolean')
    s = deepcopy(state); body = s['body']; old = _packet(body); controls = s['controls']
    if body['water'] or body['immersion'] != 0.: raise NotImplementedError('Pure-air composition only')
    if len(body['clocks']) != 4 or len(controls) != 8: raise ValueError('Adapter requires four stored clocks and four factors')
    props = _flight_for(p)
    precision = p['precision']
    fp.body = &props.body; fp.rocket = &props.rocket; fp.motors = &props.motors; fp.precision = precision
    fp.end_speed = p['end_speed']
    _body(body, &b)
    for i in range(8): c[i] = controls[i]
    steps = <wtm_substep *>malloc(precision * sizeof(wtm_substep))
    if steps == NULL: raise MemoryError()
    try:
        body_advance(&fp, &b, c, step, &out, steps)
        _check()
        body['seed'] = b.seed; body['surface_depth'] = <double>b.surface_depth
        substeps = []
        for i in range(precision):
            entry = dict(body)
            _store_body(entry, &steps[i].state)
            substeps.append(dict(state=entry, acceleration=[steps[i].acceleration[0], steps[i].acceleration[1], steps[i].acceleration[2]],
                                 angular_acceleration=_list3(steps[i].angular_acceleration)))
    finally:
        free(steps)
    _store_body(body, &b)
    pre_cap = _packet(body)
    launch = s.get('launch_time', 0.)
    elapsed = <double>(out.target_time - launch)
    guidance = guidance_step(p['guidance'], s['guidance'], old, pre_cap, elapsed, body['clocks'], targets,
                             context_value=body['surface_depth'], matrix_velocity_frame=matrix_velocity_frame)
    s['guidance'] = guidance['state']; manager = s['guidance']['manager']
    s['controls'] = manager['fins'] + manager['orientation_auxiliary'] + manager['factors']
    body['velocity'] = _list3(cap_velocity(_v3(body['velocity']), p['end_speed']))
    _check()
    propulsion = dict(world_force=_list3(out.propulsion.world_force), body_torque=_list3(out.propulsion.body_torque),
                      mass_lost=<double>out.propulsion.mass_lost,
                      clock_rates=[<double>out.propulsion.clock_rates[i] for i in range(4)], seed=out.propulsion.seed)
    return dict(state=s, propulsion=propulsion, substeps=substeps, pre_cap=pre_cap, guidance=guidance)
