// Typed evaluation path for missile_envelope.evaluate.
//
// The translated engine still performs the per-sample setup (profile, release
// and seeker initialization). Its complete session state is then converted once
// into typed structs and the flight loop runs without dynamic values. Any state
// layout, property or numerical branch the typed port does not cover raises
// typed::Bail, and the sample is recomputed from scratch by the translated
// evaluate, so every result keeps the reference semantics.
#include "typed_engine.h"
#include <cmath>
#include <math.h>
namespace nt { double reference_pow(double,double); }
#ifdef _WIN32
#define pow nt::reference_pow
#endif
#include "controls.h"
#ifdef _WIN32
#undef pow
#endif
#include "typed/typed_prelaunch.h"
#include "typed/typed_launch.h"
#include "reference_norm.h"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>

namespace nt {
namespace {
using namespace typed;

// ------------------------------------------------------------ value access
V at_key(const V& d, const char* name) {
    if (d.kind() != Kind::Dict) bail("typed loader: dictionary expected");
    try { return item(d, V(name)); } catch (const PyError&) { bail("typed loader: missing key"); }
}
V field(const V& d, const char* name) {
    if (d.kind() == Kind::Dict) return at_key(d, name);
    try { return attr(d, name); } catch (const PyError&) { bail("typed loader: missing attribute"); }
}
bool has(const V& d, const char* name) { return d.kind() == Kind::Dict && contains(d, V(name)); }
V get_or(const V& d, const char* name, V fallback) { return has(d, name) ? at_key(d, name) : fallback; }
size_t length(const V& v) {
    Kind k = v.kind();
    if (k != Kind::List && k != Kind::Tuple && k != Kind::Dict) bail("typed loader: sequence expected");
    return iter(v).size();
}
double num(const V& v) {
    switch (v.kind()) {
        case Kind::Float: return v.as_double();
        case Kind::Int: return (double)v.as_int();
        case Kind::Bool: return truth(v) ? 1. : 0.;
        default: bail("typed loader: number expected");
    }
}
int64_t ival(const V& v) {
    switch (v.kind()) {
        case Kind::Int: return v.as_int();
        case Kind::Bool: return truth(v) ? 1 : 0;
        default: bail("typed loader: integer expected");
    }
}
bool flag(const V& v) {
    Kind k = v.kind();
    if (k != Kind::Bool && k != Kind::Int && k != Kind::Float) bail("typed loader: flag expected");
    return truth(v);
}
double num_at(const V& d, const char* name) { return num(field(d, name)); }
int64_t int_at(const V& d, const char* name) { return ival(field(d, name)); }
bool flag_at(const V& d, const char* name) { return flag(field(d, name)); }
template<size_t N> std::array<double,N> vec(const V& v) {
    Args items = iter(v);
    if (items.size() != N) bail("typed loader: vector length");
    std::array<double,N> out; for (size_t i = 0; i < N; ++i) out[i] = num(items[i]); return out;
}
template<size_t N> std::array<double,N> vec_at(const V& d, const char* name) { return vec<N>(field(d, name)); }
template<size_t N> void floats_at(const V& d, const char* name, float (&out)[N]) {
    auto v = vec_at<N>(d, name); for (size_t i = 0; i < N; ++i) out[i] = (float)v[i];
}
float fnum(const V& d, const char* name) { return (float)num_at(d, name); }
void expect_keys(const V& d, size_t count) {
    if (d.kind() != Kind::Dict) bail("typed loader: dictionary expected");
    if (iter(d).size() != count) bail("typed loader: unexpected state layout");
}

// ------------------------------------------------------- kernel properties
// Same conversions as engine_kernels' verified adapters.
void load_rows(const V& value, wtm_row* rows, int& count, int maximum) {
    Args values = iter(value);
    if (values.size() > (size_t)maximum) bail("typed loader: table too long");
    count = (int)values.size();
    for (size_t i = 0; i < values.size(); ++i) { auto r = vec<3>(values[i]); rows[i] = {(float)r[0], (float)r[1], (float)r[2]}; }
}
void load_motors(const V& value, wtm_motors& out) {
    Args motors = iter(value);
    if (motors.size() > 4) bail("typed loader: at most four propulsion units");
    out = {}; out.count = (int)motors.size();
    for (size_t i = 0; i < motors.size(); ++i) {
        wtm_propulsion& p = out.items[i];
        p.delay = (float)num(field(motors[i], "delay"));
        Args impulses = iter(field(motors[i], "impulses"));
        if (impulses.size() > 4) bail("typed loader: at most four impulses");
        p.count = (int)impulses.size();
        for (size_t j = 0; j < impulses.size(); ++j) {
            if (ival(field(impulses[j], "factor_index")) >= 0) bail("Propulsion factor tables need their native loader");
            wtm_impulse& m = p.impulses[j];
            m.duration = (float)num(field(impulses[j], "duration")); m.force = (float)num(field(impulses[j], "force"));
            m.mass_lost = (float)num(field(impulses[j], "mass_lost")); m.vectoring = (float)num(field(impulses[j], "vectoring"));
        }
    }
}
void load_rocket(const V& rocket, wtm_rocket& out) {
    auto ext = vec<4>(get_or(rocket, "extPressureToThrustMult", make_list({0., 1., 0., 1.})));
    for (int i = 0; i < 4; ++i) out.ext_pressure[i] = (float)ext[i];
    out.thrust_deviation = (float)num(get_or(rocket, "thrustDeviation", V(0.)));
    out.orientation_torque = (float)num(get_or(rocket, "orientationTorque", V(0.)));
}
void load_body_props(const V& v, wtm_body_props& o) {
    o.mass = fnum(v, "mass"); o.front_area = fnum(v, "front_area"); o.side_area = fnum(v, "side_area");
    o.stabilizer_arm = fnum(v, "stabilizer_arm"); o.length = fnum(v, "length"); o.cx = fnum(v, "cx"); o.cy = fnum(v, "cy");
    o.cy_limit = fnum(v, "cy_limit"); o.cx_aoa = fnum(v, "cx_aoa"); o.fins_hor = fnum(v, "fins_hor"); o.fins_ver = fnum(v, "fins_ver");
    o.fin_pressure_limit = fnum(v, "fin_pressure_limit"); o.damping_geometry = fnum(v, "damping_geometry");
    floats_at(v, "inertia", o.inertia); floats_at(v, "angular_damping", o.angular_damping);
    auto q = vec_at<4>(v, "axis_quaternion"); o.axis_quaternion = {(float)q[0], (float)q[1], (float)q[2], (float)q[3]};
    load_rows(field(v, "cy_table"), o.cy_table, o.cy_rows, 64);
}
void load_orientation(const V& v, wtm_orientation_props& o) {
    o.start = fnum(v, "start"); o.duration = fnum(v, "duration"); o.final_speed = fnum(v, "final_speed");
    o.final_distance = fnum(v, "final_distance"); o.depth_min = fnum(v, "depth_min"); o.angle_to_rate = fnum(v, "angle_to_rate");
    o.use_fins = flag_at(v, "use_fins");
    floats_at(v, "elevation", o.elevation); floats_at(v, "heading", o.heading); floats_at(v, "coefficients", o.coefficients);
}
void load_guidance_props(const V& v, wtm_guidance_props& o) {
    o.loft_enabled = flag_at(v, "loft_enabled"); o.absolute_altitude = flag_at(v, "absolute_altitude");
    o.loft_elevation = fnum(v, "loft_elevation"); o.loft_rate_squared = fnum(v, "loft_rate_squared"); o.loft_accel = fnum(v, "loft_accel");
    o.hold_time = fnum(v, "hold_time"); o.hold_elevation = fnum(v, "hold_elevation"); o.hold_rate = fnum(v, "hold_rate");
    o.elevation_add = fnum(v, "elevation_add"); o.angle_gain = fnum(v, "angle_gain"); o.omega_gain = fnum(v, "omega_gain");
    o.course_gain = fnum(v, "course_gain"); o.course_final_angle = fnum(v, "course_final_angle");
    o.altitude_diff_gain = fnum(v, "altitude_diff_gain"); o.altitude_rate_gain = fnum(v, "altitude_rate_gain");
    o.climb_accel_gain = fnum(v, "climb_accel_gain");
    floats_at(v, "range_elevation", o.range_elevation); floats_at(v, "climb_limits", o.climb_limits);
    load_rows(field(v, "time_gain"), o.time_gain.rows, o.time_gain.count, 32);
    load_rows(field(v, "time_to_hit_gain"), o.time_to_hit_gain.rows, o.time_to_hit_gain.count, 32);
    load_rows(field(v, "altitude_table"), o.altitude_table.rows, o.altitude_table.count, 32);
}
void load_acceleration(const V& v, wtm_acceleration_props& o) {
    o.velocity_frame = flag_at(v, "velocity_frame"); o.limit_aoa = flag_at(v, "limit_aoa");
    o.max_accel = fnum(v, "max_accel"); o.aoa_max = fnum(v, "aoa_max"); o.base_speed_squared = fnum(v, "base_speed_squared");
    Args schedule = iter(field(v, "schedule")), coefficients = iter(field(v, "coefficients"));
    if (schedule.size() > 32 || coefficients.size() > 16) bail("typed loader: acceleration schedule too long");
    o.schedule_count = (int)schedule.size(); o.coefficient_count = (int)coefficients.size();
    for (size_t i = 0; i < coefficients.size(); ++i) {
        auto c = vec<4>(coefficients[i]); for (int j = 0; j < 4; ++j) o.coefficients[i][j] = (float)c[j];
    }
    for (size_t i = 0; i < schedule.size(); ++i) {
        Args row = iter(schedule[i]);
        if (row.size() != 3) bail("typed loader: schedule row");
        if (row[2].kind() != Kind::Int || row[2].as_int() < 0 || row[2].as_int() >= (int64_t)coefficients.size()) bail("IndexError: Invalid PID coefficient index");
        for (int j = 0; j < 3; ++j) o.schedule[i][j] = (float)num(row[j]);
    }
}
void load_aero(const V& v, wtm_aero_props& o) {
    o.mass = fnum(v, "mass"); o.cy = fnum(v, "cy"); o.cy_limit = fnum(v, "cy_limit"); o.side_area = fnum(v, "side_area");
    load_rows(field(v, "cy_table"), o.cy_table, o.cy_rows, 64);
}
void load_propulsion_props(const V& value, wtm_propulsion_control_props& o) {
    Args stages = iter(value);
    if (stages.size() > 4) bail("ValueError: Native controller supports at most four stages");
    o = {}; o.count = (int)stages.size();
    for (size_t i = 0; i < stages.size(); ++i) {
        const V& v = stages[i]; wtm_stage& s = o.stages[i];
        s.start = fnum(v, "start"); s.speed_squared = fnum(v, "speed_squared"); s.closure_min = fnum(v, "closure_min");
        s.tgo_min = fnum(v, "tgo_min"); s.cruise = fnum(v, "cruise"); s.cruise_error_limit = fnum(v, "cruise_error_limit");
        s.base_ias_squared = fnum(v, "base_ias_squared"); s.terminal_tgo = fnum(v, "terminal_tgo"); s.use_ias = flag_at(v, "use_ias");
        floats_at(v, "altitude", s.altitude); floats_at(v, "coefficients", s.coefficients); floats_at(v, "pursuit_closure", s.pursuit_closure);
    }
}
void load_controls(const V& g, ControlProps& c) {
    load_orientation(field(g, "orientation"), c.orientation);
    load_guidance_props(field(g, "guidance"), c.guidance);
    load_acceleration(field(g, "acceleration"), c.acceleration);
    load_aero(field(g, "aero"), c.aero);
    load_motors(field(g, "motors"), c.motors);
    load_propulsion_props(field(g, "propulsion"), c.propulsion);
}
SharedProps load_shared(const V& v) {
    return {num_at(v, "angle_max"), num_at(v, "lock_angle_max"), num_at(v, "rate_max"), num_at(v, "alpha"), num_at(v, "beta"), num_at(v, "gate_rate")};
}

// ---------------------------------------------------- family properties
void load_optical_props(const V& g, OpticalProps& o) {
    V m = field(g, "manager");
    o.manager = {num_at(m, "lock_timeout"), flag_at(m, "inertial_navigation"), num_at(m, "lock_distance"), num_at(m, "lock_cone_threshold"),
                 flag_at(m, "reacquire"), num_at(m, "leading_gain"), vec_at<2>(m, "leading_times"), num_at(m, "scan_half_sector"),
                 num_at(m, "half_fov"), flag_at(m, "auxiliary_flag"), has(m, "designation_notification") ? flag_at(m, "designation_notification") : false};
    load_controls(g, o.controls);
    V s = field(g, "seeker");
    o.seeker.shared = load_shared(field(s, "shared"));
    o.seeker.width_max = num_at(s, "width_max"); o.seeker.width_min = num_at(s, "width_min");
    o.seeker.prolongation = num_at(s, "prolongation"); o.seeker.reset_time = num_at(s, "reset_time");
    o.seeker.designation_mask = int_at(s, "designation_mask"); o.seeker.constant_mask = int_at(s, "constant_mask");
    o.seeker.arret_angles = vec_at<2>(s, "arret_angles"); o.seeker.scan_half_sector = num_at(s, "scan_half_sector");
    o.seeker.surface_as_target = flag_at(s, "surface_as_target"); o.seeker.half_fov = num_at(s, "half_fov");
    V ob = field(g, "observation");
    o.observation = {num_at(ob, "cosine"), num_at(ob, "prolongation")};
    o.los_check_timeout = num_at(g, "los_check_timeout");
}
Axis load_axis(const V& v, bool designation) {
    Axis a;
    a.present = flag_at(v, "present"); a.minimum = num_at(v, "minimum"); a.maximum = num_at(v, "maximum"); a.width = num_at(v, "width");
    a.alpha = num_at(v, "alpha"); a.beta = num_at(v, "beta"); a.limit = vec_at<2>(v, "limit"); a.limit_timeout = num_at(v, "limit_timeout");
    if (designation) { a.unambiguous = num_at(v, "unambiguous"); a.signal_width_min = num_at(v, "signal_width_min"); }
    else a.initial = num_at(v, "initial");
    return a;
}
DesignationProps load_designation(const V& v) {
    DesignationProps d;
    d.shared = load_shared(field(v, "shared"));
    d.distance = load_axis(field(v, "distance"), true); d.doppler = load_axis(field(v, "doppler"), true);
    d.use_target_id = flag_at(v, "use_target_id"); d.doppler_to_distance = num_at(v, "doppler_to_distance");
    return d;
}
void load_radar_props(const V& g, RadarProps& o) {
    V m = field(g, "manager");
    RadarManagerProps& p = o.manager;
    p.lock_timeout = num_at(m, "lock_timeout"); p.lock_distance = num_at(m, "lock_distance"); p.lock_cone_threshold = num_at(m, "lock_cone_threshold");
    p.inertial_navigation = int_at(m, "inertial_navigation"); p.reacquire = int_at(m, "reacquire");
    p.use_target_velocity = int_at(m, "use_target_velocity"); p.semi_active = int_at(m, "semi_active");
    p.designation_notification = int_at(m, "designation_notification");
    auto limits = vec_at<3>(m, "search_limits"); p.search_limits = {limits[0], limits[1], limits[2]};
    p.search_values.clear(); for (V row : iter(field(m, "search_values"))) p.search_values.push_back(vec<2>(row));
    p.designation_mask = int_at(m, "designation_mask"); p.datalink_reconnect = int_at(m, "datalink_reconnect");
    p.datalink_limit = int_at(m, "datalink_limit"); p.datalink_counter = int_at(m, "datalink_counter");
    p.distance_limits.clear();
    for (V row : iter(field(m, "distance_limits"))) p.distance_limits.push_back({int_at(row, "present"), num_at(row, "minimum")});
    load_controls(g, o.controls);
    V seeker = field(g, "seeker"), modes = field(seeker, "modes");
    RadarModesProps& md = o.seeker.modes;
    md.shared = load_shared(field(modes, "shared"));
    md.distance = load_axis(field(modes, "distance"), false); md.doppler = load_axis(field(modes, "doppler"), false);
    md.prolongation = num_at(modes, "prolongation"); md.reset_time = num_at(modes, "reset_time");
    md.arret_angles = vec_at<2>(modes, "arret_angles"); md.use_target_id = flag_at(modes, "use_target_id");
    md.designation = load_designation(field(modes, "designation"));
    // missile_support.supported_radar_step prepares these designation axes.
    for (int k = 0; k < 2; ++k) {
        Axis& axis = k == 0 ? md.designation.distance : md.designation.doppler;
        axis.minimum = k == 0 ? 0. : -BOUND; axis.maximum = BOUND; axis.unambiguous = 2 * BOUND;
        axis.signal_width_min = mx(1., axis.signal_width_min);
    }
    md.use_source_frame = flag_at(modes, "use_source_frame"); md.beam_cosine = num_at(modes, "beam_cosine");
    md.scan_half_sector = num_at(modes, "scan_half_sector"); md.range_search_half = num_at(modes, "range_search_half");
    md.closure_search_half = num_at(modes, "closure_search_half");
    V frame = field(field(seeker, "observation"), "frame");
    o.seeker.frame = {flag_at(frame, "active"), flag_at(frame, "force_bistatic"), vec_at<4>(frame, "range_parameters")};
    V query = field(seeker, "query");
    o.seeker.query_maximum = num_at(query, "maximum"); o.seeker.query_cosine = num_at(query, "cosine");
    o.los_check_timeout = num_at(g, "los_check_timeout");
    V in = field(g, "inertial");
    o.inertial = {num_at(in, "drift_speed"), int_at(in, "gnss"), int_at(in, "datalink"), int_at(in, "reconnect"),
                  int_at(in, "channels_max"), int_at(in, "keep_channels_for_last")};
}
// missile_initialization.radar_reset's launcher properties, already prepared
// for the website policy (radar_activation and radar_prelaunch inputs).
void load_radar_launcher(const V& p, RadarLauncherProps& L) {
    L.permanent = flag_at(p, "permanent"); L.lock_after_launch = flag_at(p, "lock_after_launch");
    L.primary_mask = int_at(p, "primary_mask"); L.fallback_mask = int_at(p, "fallback_mask");
    L.seeker = load_designation(field(p, "seeker"));
    L.warm_up = num_at(p, "warm_up"); L.work_time = num_at(p, "work_time");
    L.uncage = flag_at(p, "uncage"); L.inertial = flag_at(p, "inertial"); L.designation_required = flag_at(p, "designation_required");
    V m = field(p, "modes");
    RadarModesProps& md = L.modes;
    md.shared = load_shared(field(m, "shared"));
    md.distance = load_axis(field(m, "distance"), false); md.doppler = load_axis(field(m, "doppler"), false);
    md.prolongation = num_at(m, "prolongation"); md.reset_time = num_at(m, "reset_time");
    md.arret_angles = vec_at<2>(m, "arret_angles"); md.use_target_id = flag_at(m, "use_target_id");
    md.designation = load_designation(field(m, "designation"));
    md.use_source_frame = false; md.beam_cosine = md.scan_half_sector = md.range_search_half = md.closure_search_half = 0.;   // mode 7 only
    L.los_check_timeout = num_at(p, "los_check_timeout");
    L.authored_rate = flag_at(p, "authored_rate"); L.include_gap_after_reject = flag_at(p, "include_gap_after_reject");
}
void load_flight_props(const V& p, const V& serialized_rocket, const V& session_props, FlightProps& F) {
    load_body_props(field(p, "body"), F.body);
    V rocket = field(p, "rocket");
    load_rocket(rocket, F.rocket);
    V motors = has(p, "_motor_properties") ? at_key(p, "_motor_properties") : field(field(p, "guidance"), "motors");
    load_motors(motors, F.motors);
    F.precision = (int)int_at(p, "precision"); F.end_speed = num_at(p, "end_speed");
    V matrix = get_or(p, "matrix_velocity_frame", V(true));
    if (matrix.kind() != Kind::Bool) bail("ValueError: matrix_velocity_frame must be an explicit boolean");
    F.matrix_velocity_frame = truth(matrix);
    V guidance_section = get_or(rocket, "guidance", rocket);
    F.break_lock = typed::f32(num(get_or(guidance_section, "breakLockMaxTime", V(1.))));
    F.has_fuse = truth(get_or(serialized_rocket, "hasProximityFuse", V(false)));
    F.proximity = {num_at(session_props, "timeOut"), num_at(session_props, "radius"), num_at(session_props, "minimalAltitude"),
                   num_at(session_props, "armDistance"), num_at(session_props, "armDistanceToTarget"),
                   flag_at(session_props, "detectAirUnits"), flag_at(session_props, "detectGroundUnits"), flag_at(session_props, "detectShells")};
    if (F.family == 0) load_optical_props(field(p, "guidance"), F.optical);
    else {
        V policy = get_or(p, "policy", V());
        if (policy.kind() != Kind::String || policy.as_string() != "geometric_radar_v1") bail("ValueError: Ideal radar support requires one geometric-policy target");
        load_radar_props(field(p, "guidance"), F.radar);
    }
}

// ------------------------------------------------------------ state loaders
void load_body(const V& v, wtm_body& b) {
    expect_keys(v, 13);
    auto pos = vec_at<3>(v, "position"), vel = vec_at<3>(v, "velocity"), om = vec_at<3>(v, "omega");
    auto q = vec_at<4>(v, "quaternion");
    b = {};
    b.position = V3((float)pos[0], (float)pos[1], (float)pos[2]); b.quaternion = {(float)q[0], (float)q[1], (float)q[2], (float)q[3]};
    b.velocity = V3((float)vel[0], (float)vel[1], (float)vel[2]); b.omega = V3((float)om[0], (float)om[1], (float)om[2]);
    b.time = fnum(v, "time"); b.seed = (uint32_t)int_at(v, "seed"); b.body_random = fnum(v, "body_random");
    floats_at(v, "clocks", b.clocks);
    b.water = flag_at(v, "water"); b.distance = fnum(v, "distance"); b.water_distance = fnum(v, "water_distance");
    b.surface_depth = fnum(v, "surface_depth"); b.immersion = fnum(v, "immersion");
}
void load_pid2(const V& v, float (&pid)[2][8]) {
    Args rows = iter(v);
    if (rows.size() != 2) bail("ValueError: Two PID records required");
    for (int i = 0; i < 2; ++i) { auto r = vec<8>(rows[i]); for (int j = 0; j < 8; ++j) pid[i][j] = (float)r[j]; }
}
void load_controllers(const V& g, Controllers& c) {
    V o = field(g, "orientation"); expect_keys(o, 2);
    c.orientation.enabled = flag_at(o, "enabled"); load_pid2(field(o, "pid"), c.orientation.pid);
    V gu = field(g, "guidance"); expect_keys(gu, 3);
    c.guidance.mode = int_at(gu, "mode"); c.guidance.course = int_at(gu, "course"); load_pid2(field(gu, "pid"), c.guidance.pid);
    V pr = field(g, "propulsion"); expect_keys(pr, 2);
    Args stages = iter(field(pr, "stages"));
    if (stages.size() != 4 || length(field(pr, "pid")) != 8) bail("ValueError: Four enums and one eight-float shared PID required");
    for (int i = 0; i < 4; ++i) c.propulsion.stages[i] = ival(stages[i]);
    auto pid = vec_at<8>(pr, "pid"); for (int i = 0; i < 8; ++i) c.propulsion.pid[i] = (float)pid[i];
}
void load_optical_seeker(const V& seeker, OpticalSeekerState& s, Opaque& opaque) {
    expect_keys(seeker, 2);
    V st = field(seeker, "state"); expect_keys(st, 8);
    s.angles = vec_at<2>(st, "angles"); s.direction = vec_at<3>(st, "direction"); s.angular_rate = vec_at<3>(st, "angular_rate");
    s.widths = vec_at<2>(st, "widths"); s.gap = num_at(st, "gap"); s.distance = num_at(st, "distance");
    s.los_check_time = num_at(st, "los_check_time"); s.los_cache_valid = flag_at(st, "los_cache_valid");
    Args raw = iter(field(seeker, "opaque_state"));
    if (raw.size() != 32) bail("ValueError: opaque_state requires 32 bytes");
    for (int i = 0; i < 32; ++i) {
        if (raw[i].kind() != Kind::Int || raw[i].as_int() < 0 || raw[i].as_int() > 255) bail("ValueError: opaque_state requires 32 bytes");
        opaque[i] = raw[i].as_int();
    }
}
void load_optical_state(const V& g, OpticalGuidance& o) {
    expect_keys(g, 5);
    V m = field(g, "manager"); expect_keys(m, 23);
    OpticalManagerState& s = o.manager;
    s.transition_time = num_at(m, "transition_time"); s.scan_phase = num_at(m, "scan_phase"); s.strength = num_at(m, "strength");
    s.distance = num_at(m, "distance"); s.seeker_distance = num_at(m, "seeker_distance");
    s.mode = int_at(m, "mode"); s.count = int_at(m, "count");
    s.active = flag_at(m, "active"); s.auxiliary_flag = flag_at(m, "auxiliary_flag"); s.engaged = flag_at(m, "engaged");
    s.tracking = flag_at(m, "tracking"); s.point_valid = flag_at(m, "point_valid"); s.seeker_point_valid = flag_at(m, "seeker_point_valid");
    s.inertial_valid = flag_at(m, "inertial_valid"); s.seeker_flag = int_at(m, "seeker_flag");
    s.fins = vec_at<2>(m, "fins"); s.orientation_auxiliary = vec_at<2>(m, "orientation_auxiliary"); s.factors = vec_at<4>(m, "factors");
    s.auxiliary = vec_at<3>(m, "auxiliary"); s.direction = vec_at<3>(m, "direction"); s.seeker_direction = vec_at<3>(m, "seeker_direction");
    s.inertial_point = vec_at<3>(m, "inertial_point"); s.inertial_offset = vec_at<3>(m, "inertial_offset");
    load_controllers(g, o.controllers);
    load_optical_seeker(field(g, "seeker"), o.seeker, o.opaque);
}
void load_radar_seeker(const V& ss, RadarSeekerState& r);
void load_radar_state(const V& g, RadarGuidance& o) {
    expect_keys(g, 5);
    V m = field(g, "manager"); expect_keys(m, 29);
    RadarManagerState& s = o.manager;
    s.transition_time = num_at(m, "transition_time"); s.strength = num_at(m, "strength"); s.distance = num_at(m, "distance");
    s.search_value = num_at(m, "search_value"); s.seeker_range = num_at(m, "seeker_range"); s.seeker_closure = num_at(m, "seeker_closure");
    s.mode = int_at(m, "mode"); s.seeker_tag = int_at(m, "seeker_tag"); s.count = int_at(m, "count"); s.channel_index = int_at(m, "channel_index");
    s.active = int_at(m, "active"); s.engaged = int_at(m, "engaged"); s.tracking = int_at(m, "tracking");
    s.range_valid = int_at(m, "range_valid"); s.closure_valid = int_at(m, "closure_valid"); s.inertial_valid = int_at(m, "inertial_valid");
    s.designation_indicator = int_at(m, "designation_indicator");
    s.fins = vec_at<2>(m, "fins"); s.orientation_auxiliary = vec_at<2>(m, "orientation_auxiliary"); s.factors = vec_at<4>(m, "factors");
    s.direction = vec_at<3>(m, "direction"); s.seeker_angles = vec_at<2>(m, "seeker_angles"); s.seeker_direction = vec_at<3>(m, "seeker_direction");
    s.inertial_point = vec_at<3>(m, "inertial_point"); s.inertial_offset = vec_at<3>(m, "inertial_offset");
    s.association_flag = int_at(m, "association_flag"); s.inertial_source_tag = int_at(m, "inertial_source_tag");
    Args padding = iter(field(m, "inertial_padding"));
    if (padding.size() != 6) bail("typed loader: inertial padding");
    for (int i = 0; i < 6; ++i) s.inertial_padding[i] = ival(padding[i]);
    s.inertial_velocity = vec_at<3>(m, "inertial_velocity");
    load_controllers(g, o.controllers);
    load_radar_seeker(field(g, "seeker"), o.seeker);
}
void load_radar_seeker(const V& ss, RadarSeekerState& r) {
    expect_keys(ss, 20);
    Args pad = iter(field(ss, "padding"));
    if (pad.size() != 7) bail("typed loader: seeker padding");
    for (int i = 0; i < 7; ++i) r.padding[i] = ival(pad[i]);
    r.gap = num_at(ss, "gap"); r.range = num_at(ss, "range"); r.range_rate = num_at(ss, "range_rate"); r.closure = num_at(ss, "closure");
    r.closure_rate = num_at(ss, "closure_rate"); r.target_value = num_at(ss, "target_value"); r.los_check_time = num_at(ss, "los_check_time");
    r.visibility_weight = num_at(ss, "visibility_weight");
    r.rejected = int_at(ss, "rejected"); r.range_valid = int_at(ss, "range_valid"); r.closure_valid = int_at(ss, "closure_valid");
    r.los_cache_valid = int_at(ss, "los_cache_valid"); r.visibility_clear = int_at(ss, "visibility_clear");
    r.associated_id = int_at(ss, "associated_id"); r.channel_index = int_at(ss, "channel_index"); r.target_id = int_at(ss, "target_id");
    r.angles = vec_at<2>(ss, "angles"); r.direction = vec_at<3>(ss, "direction"); r.angular_rate = vec_at<3>(ss, "angular_rate");
}
void load_session(const V& session, int family, Session& S) {
    V frame = attr(session, "frame"), flight = attr(frame, "flight"), state = attr(flight, "state");
    expect_keys(state, 3);
    load_body(field(state, "body"), S.body);
    S.controls = vec_at<8>(state, "controls");
    if (family == 0) load_optical_state(field(state, "guidance"), S.optical);
    else load_radar_state(field(state, "guidance"), S.radar);
    V limits = attr(frame, "limits");
    S.launch_time = num_at(limits, "launch_time"); S.lifetime = num_at(limits, "lifetime");
    S.distance_limit = num_at(limits, "distance_limit"); S.origin = vec_at<3>(limits, "origin");
    S.iteration_limit = ival(attr(frame, "iteration_limit")); S.expiry_flag = truth(attr(frame, "expiry_flag"));
    S.lost_tracking_clock = num(attr(frame, "lost_tracking_clock"));
    V proximity = attr(session, "proximity");
    S.proximity_clock = num_at(proximity, "clock"); S.proximity_origin = vec_at<3>(proximity, "origin");
    S.proximity_enabled = flag_at(proximity, "enabled"); S.guidance_flag = flag_at(proximity, "guidance_flag");
    S.mode = ival(attr(session, "mode")); S.delay = num(attr(session, "delay"));
    V event = attr(session, "event");
    S.has_event = !event.is_none();
    if (S.has_event) { S.event.position = vec_at<3>(event, "position"); S.event.direction = vec_at<3>(event, "direction"); S.event.time = num_at(event, "time"); }
    S.offsets = vec<2>(attr(session, "offsets"));
}

// --------------------------------------------------------------- snapshots
using Snapshot = std::vector<std::pair<std::string,double>>;
template<size_t N> void put(Snapshot& s, const char* name, const std::array<double,N>& v) {
    for (size_t i = 0; i < N; ++i) s.emplace_back(std::string(name) + "[" + std::to_string(i) + "]", v[i]);
}
void put(Snapshot& s, const char* name, double v) { s.emplace_back(name, v); }
Snapshot snapshot(const Session& S, int family) {
    Snapshot s;
    const wtm_body& b = S.body;
    put(s, "body.position", d3{b.position.x, b.position.y, b.position.z}); put(s, "body.quaternion", dq(b.quaternion));
    put(s, "body.velocity", d3{b.velocity.x, b.velocity.y, b.velocity.z}); put(s, "body.omega", d3{b.omega.x, b.omega.y, b.omega.z});
    put(s, "body.time", b.time); put(s, "body.seed", (double)b.seed); put(s, "body.body_random", b.body_random);
    put(s, "body.clocks", d4{b.clocks[0], b.clocks[1], b.clocks[2], b.clocks[3]}); put(s, "body.distance", b.distance);
    put(s, "body.surface_depth", b.surface_depth); put(s, "body.water", b.water);
    put(s, "body.water_distance", b.water_distance); put(s, "body.immersion", b.immersion);
    put(s, "controls", S.controls);
    auto pids = [&](const char* name, const float (&pid)[2][8]) {
        for (int i = 0; i < 2; ++i) for (int j = 0; j < 8; ++j) put(s, (std::string(name) + "." + std::to_string(i) + "." + std::to_string(j)).c_str(), pid[i][j]);
    };
    const Controllers& c = family == 0 ? S.optical.controllers : S.radar.controllers;
    put(s, "orientation.enabled", c.orientation.enabled); pids("orientation.pid", c.orientation.pid);
    put(s, "guidance.mode", (double)c.guidance.mode); put(s, "guidance.course", (double)c.guidance.course); pids("guidance.pid", c.guidance.pid);
    for (int i = 0; i < 4; ++i) put(s, ("propulsion.stages." + std::to_string(i)).c_str(), (double)c.propulsion.stages[i]);
    for (int i = 0; i < 8; ++i) put(s, ("propulsion.pid." + std::to_string(i)).c_str(), c.propulsion.pid[i]);
    if (family == 0) {
        const OpticalManagerState& m = S.optical.manager;
        put(s, "m.transition_time", m.transition_time); put(s, "m.scan_phase", m.scan_phase); put(s, "m.strength", m.strength);
        put(s, "m.distance", m.distance); put(s, "m.seeker_distance", m.seeker_distance); put(s, "m.mode", (double)m.mode);
        put(s, "m.count", (double)m.count); put(s, "m.active", m.active); put(s, "m.auxiliary_flag", m.auxiliary_flag);
        put(s, "m.engaged", m.engaged); put(s, "m.tracking", m.tracking); put(s, "m.point_valid", m.point_valid);
        put(s, "m.seeker_point_valid", m.seeker_point_valid); put(s, "m.inertial_valid", m.inertial_valid); put(s, "m.seeker_flag", (double)m.seeker_flag);
        put(s, "m.fins", m.fins); put(s, "m.orientation_auxiliary", m.orientation_auxiliary); put(s, "m.factors", m.factors);
        put(s, "m.auxiliary", m.auxiliary); put(s, "m.direction", m.direction); put(s, "m.seeker_direction", m.seeker_direction);
        put(s, "m.inertial_point", m.inertial_point); put(s, "m.inertial_offset", m.inertial_offset);
        const OpticalSeekerState& k = S.optical.seeker;
        put(s, "s.angles", k.angles); put(s, "s.direction", k.direction); put(s, "s.angular_rate", k.angular_rate); put(s, "s.widths", k.widths);
        put(s, "s.gap", k.gap); put(s, "s.distance", k.distance); put(s, "s.los_check_time", k.los_check_time); put(s, "s.los_cache_valid", k.los_cache_valid);
        for (int i = 0; i < 32; ++i) put(s, ("s.opaque." + std::to_string(i)).c_str(), (double)S.optical.opaque[i]);
    } else {
        const RadarManagerState& m = S.radar.manager;
        put(s, "m.transition_time", m.transition_time); put(s, "m.strength", m.strength); put(s, "m.distance", m.distance);
        put(s, "m.search_value", m.search_value); put(s, "m.seeker_range", m.seeker_range); put(s, "m.seeker_closure", m.seeker_closure);
        put(s, "m.mode", (double)m.mode); put(s, "m.seeker_tag", (double)m.seeker_tag); put(s, "m.count", (double)m.count);
        put(s, "m.channel_index", (double)m.channel_index); put(s, "m.active", (double)m.active); put(s, "m.engaged", (double)m.engaged);
        put(s, "m.tracking", (double)m.tracking); put(s, "m.range_valid", (double)m.range_valid); put(s, "m.closure_valid", (double)m.closure_valid);
        put(s, "m.inertial_valid", (double)m.inertial_valid); put(s, "m.designation_indicator", (double)m.designation_indicator);
        put(s, "m.fins", m.fins); put(s, "m.orientation_auxiliary", m.orientation_auxiliary); put(s, "m.factors", m.factors);
        put(s, "m.direction", m.direction); put(s, "m.seeker_angles", m.seeker_angles); put(s, "m.seeker_direction", m.seeker_direction);
        put(s, "m.inertial_point", m.inertial_point); put(s, "m.inertial_offset", m.inertial_offset);
        put(s, "m.association_flag", (double)m.association_flag); put(s, "m.inertial_source_tag", (double)m.inertial_source_tag);
        put(s, "m.inertial_velocity", m.inertial_velocity);
        for (int i = 0; i < 6; ++i) put(s, ("m.inertial_padding." + std::to_string(i)).c_str(), (double)m.inertial_padding[i]);
        const RadarSeekerState& k = S.radar.seeker;
        put(s, "s.gap", k.gap); put(s, "s.range", k.range); put(s, "s.range_rate", k.range_rate); put(s, "s.closure", k.closure);
        put(s, "s.closure_rate", k.closure_rate); put(s, "s.target_value", k.target_value); put(s, "s.los_check_time", k.los_check_time);
        put(s, "s.visibility_weight", k.visibility_weight); put(s, "s.rejected", (double)k.rejected); put(s, "s.range_valid", (double)k.range_valid);
        put(s, "s.closure_valid", (double)k.closure_valid); put(s, "s.los_cache_valid", (double)k.los_cache_valid);
        put(s, "s.visibility_clear", (double)k.visibility_clear); put(s, "s.associated_id", (double)k.associated_id);
        put(s, "s.channel_index", (double)k.channel_index); put(s, "s.target_id", (double)k.target_id);
        put(s, "s.angles", k.angles); put(s, "s.direction", k.direction); put(s, "s.angular_rate", k.angular_rate);
        for (int i = 0; i < 7; ++i) put(s, ("s.padding." + std::to_string(i)).c_str(), (double)k.padding[i]);
    }
    put(s, "frame.expiry_flag", S.expiry_flag); put(s, "frame.lost_tracking_clock", S.lost_tracking_clock);
    put(s, "frame.launch_time", S.launch_time); put(s, "frame.lifetime", S.lifetime);
    put(s, "frame.distance_limit", S.distance_limit); put(s, "frame.origin", S.origin);
    put(s, "frame.iteration_limit", (double)S.iteration_limit);
    put(s, "proximity.clock", S.proximity_clock); put(s, "interaction.mode", (double)S.mode); put(s, "interaction.has_event", S.has_event);
    put(s, "proximity.origin", S.proximity_origin); put(s, "proximity.enabled", S.proximity_enabled);
    put(s, "proximity.guidance_flag", S.guidance_flag); put(s, "interaction.delay", S.delay);
    put(s, "interaction.offsets", S.offsets);
    if (S.has_event) {
        put(s, "event.time", S.event.time); put(s, "event.position", S.event.position); put(s, "event.direction", S.event.direction);
    }
    return s;
}
uint64_t bits(double x) { uint64_t b; std::memcpy(&b, &x, 8); return b; }

// ---------------------------------------------------------- evaluate entry
enum class Mode { Off, On, Compare };
Mode typed_mode() {
    static const Mode mode = [] {
        const char* v = std::getenv("WT_MISSILE_TYPED");
        if (!v || !*v || std::string(v) == "1") return Mode::On;
        if (std::string(v) == "compare") return Mode::Compare;
        return Mode::Off;
    }();
    return mode;
}
bool typed_log() { static const bool on = [] { const char* v = std::getenv("WT_TYPED_LOG"); return v && *v && std::string(v) != "0"; }(); return on; }

V invoke_fn(const char* module, const char* function, Args args = {}, Kw kwargs = {}) {
    return call(attr(import_module(module), function), args, kwargs);
}

struct Prepared {
    FlightProps props; Session session; EvaluateSetup setup;
    V family, lifetime, distance_limit;
    bool cache_hit = false;
};
// Mirrors the preamble of missile_envelope.evaluate. PyError propagates to the
// caller, which then reruns the reference function from scratch.
void prepare_full(const V& missile, const V& launcher, const V& target, const V& early_exit, const V& reach_m, Prepared& out) {
    V w = invoke_fn("missile_envelope", "_missile_worker");
    V profile = invoke_fn("missile_envelope", "_profile", {missile});
    V frame = call(attr(w, "initial_frame"), {profile, launcher, target}, {{"launcher_aircraft", V(true)}});
    V release = attr(frame, "website_release");
    V session = call(attr(attr(w, "InteractionSession"), "from_frame"), {frame},
                     {{"clock", V(0.)}, {"origin", item(release, V("position"))}, {"enabled", V(reach_m.is_none())}, {"guidance_flag", V(false)}});
    V flight = attr(frame, "flight");
    out.family = attr(flight, "family");
    std::string family = out.family.as_string();
    out.props.family = family == "optical" ? 0 : family == "radar" ? 1 : -1;
    if (out.props.family < 0) bail("Unsupported guidance family");
    V limits = attr(frame, "limits");
    out.lifetime = item(limits, V("lifetime")); out.distance_limit = item(limits, V("distance_limit"));
    double launch_time = num(item(limits, V("launch_time"))), lifetime = num(out.lifetime);
    EvaluateSetup& E = out.setup;
    E.deadline = launch_time + lifetime;
    double count = std::ceil((lifetime + 1.) / STEP);
    if (!std::isfinite(count) || count < 0 || count > 1e9) bail("Unsupported step count");
    E.count = (int64_t)count;
    V session_props = attr(session, "props");
    if (!reach_m.is_none()) { E.has_reach = true; E.reach_m = num(reach_m); E.radius = E.reach_m + 30.; }
    else {
        V radius = dict_get(session_props, V("radius"), V(0.));
        E.radius = (truth(radius) ? num(radius) : 0.) + 30.;
    }
    E.early_exit = truth(early_exit);
    V motors = item(item(attr(flight, "properties"), V("guidance")), V("motors"));
    for (V end : iter(invoke_fn("missile_envelope", "_burn_end", {motors}))) E.burn_end.push_back(num(end));
    E.best_distance = num(call(attr(import_module("math"), "dist"), {item(release, V("position")), item(target, V("position"))}));
    V target_quaternion = call(attr(w, "quaternion"), {item(target, V("angles"))});
    E.motion = {vec<3>(item(target, V("position"))), vec<3>(item(target, V("velocity"))), vec<4>(target_quaternion)};
    for (double x : E.motion.position) if (!std::isfinite(x)) bail("ValueError: motion must be finite");
    for (double x : E.motion.velocity) if (!std::isfinite(x)) bail("ValueError: motion must be finite");
    E.target_velocity_raw = E.motion.velocity;
    E.launch = {vec<3>(item(launcher, V("position"))), vec<3>(item(launcher, V("velocity"))),
                vec<4>(call(attr(w, "quaternion"), {item(launcher, V("angles"))}))};
    // Typed state and properties from the constructed session.
    V session_frame = attr(session, "frame"), session_flight = attr(session_frame, "flight");
    load_flight_props(attr(session_flight, "properties"), item(attr(session_flight, "serialized_properties"), V("rocket")), session_props, out.props);
    load_session(session, out.props.family, out.session);
    validate_flight_numbers(out.session, out.props.family);
}

// One immutable profile per worker. All properties still pass the translated
// FlightSession constructor, digest/motor checks and radar configuration once.
// Release construction, launch spread, seeker reset, prelaunch acquisition and
// launch initialization are computed for every sample: on typed state once the
// typed setup has reproduced the translated one (see typed), otherwise with the
// translated native functions. No state is shared between flights. Keeping the
// source value alive makes the identity key unambiguous.
struct ProfileSetup {
    V source, profile, properties, rocket, release, launch_properties;
    V proximity, distance_limit, family;
    FlightProps typed_props;
    std::vector<double> burn_end;
    // Typed setup: per-profile inputs of the release, spread and reset, and the
    // first sample's launched guidance, whose sample-dependent fields are replaced
    // for every later sample. Enabled on this thread only after the typed setup
    // reproduced a complete translated setup bit for bit.
    int typed = 0;   // 0 not checked, 1 enabled, -1 disabled
    LaunchProps launch{};
    OpticalSeekerState optical_initial{}; Opaque optical_initial_raw{};
    RadarLauncherProps radar_launcher{}; RadarSeekerState radar_template{};
    OpticalGuidance launched_optical{}; RadarGuidance launched_radar{};
};
thread_local std::unique_ptr<ProfileSetup> profile_setup;
ProfileSetup& prepared_profile(const V& w, const V& source, bool& hit) {
    hit = profile_setup && profile_setup->source.node() == source.node();
    if (hit) return *profile_setup;
    auto c = std::make_unique<ProfileSetup>();
    c->source = source;
    c->profile = item(source, "family").as_string() == "radar"
        ? call(attr(w, "profile_with_support"), {source}) : source;
    V flight = call(attr(w, "FlightSession"), {c->profile});
    c->properties = attr(flight, "properties"); c->family = attr(flight, "family");
    c->rocket = item(c->properties, "rocket");
    c->release = item(c->profile, "release_inputs");
    c->launch_properties = call(attr(w, "launch_properties"), {c->rocket, item(c->properties, "guidance")});
    c->proximity = invoke_fn("proximity_request", "properties", {item(attr(flight, "serialized_properties"), "rocket")});
    c->distance_limit = call(attr(w, "distance_setting"), {c->rocket});
    c->typed_props.family = c->family.as_string() == "optical" ? 0 : 1;
    load_flight_props(c->properties, item(attr(flight, "serialized_properties"), "rocket"), c->proximity, c->typed_props);
    for (V end : iter(invoke_fn("missile_envelope", "_burn_end", {item(item(c->properties, "guidance"), "motors")})))
        c->burn_end.push_back(num(end));
    profile_setup = std::move(c);  // Publish only after complete validation.
    return *profile_setup;
}
// body_launch.initialize inputs for launch mode 2 (missile_worker.launch_body).
void load_launch_props(const ProfileSetup& c, LaunchProps& L) {
    const V& r = c.rocket;
    L.max_delta_angle = num_at(r, "maxDeltaAngle"); L.advanced_spread = num(get_or(r, "advancedSpread", V(0.)));
    L.start_speed = num_at(r, "startSpeed"); L.use_start_speed = truth(field(r, "useStartSpeed"));
    L.vel_spread = num(get_or(r, "velSpread", V(1.))); L.distance_fuse = truth(get_or(r, "distanceFuse", V(true)));
    L.time_life = num(get_or(r, "timeLife", V(-1.))); L.distance_limit = num(c.distance_limit);
    Args motors = iter(invoke_fn("kernels", "motor_properties", {r}));
    L.propulsion_count = (int64_t)motors.size(); L.first_delay = motors.empty() ? 0. : num(attr(motors[0], "delay"));
    L.table.clear();
    for (V row : iter(attr(import_module("body_launch"), "TABLE"))) L.table.push_back(vec<2>(row));
}
// The translated setup keeps the constructed and launched bodies and the seeker
// reset of the last launcher state (seeds are fixed). Each sample receives its
// own copy of the constructed body; the ready functions copy the reset state.
struct LaunchSetup {
    V source; d3 position{}, velocity{}, angles{};
    V quaternion, body, launched, reset;
};
thread_local std::unique_ptr<LaunchSetup> launch_setup;
bool same_bits(const d3& a, const d3& b) { return std::memcmp(a.data(), b.data(), sizeof(a)) == 0; }
bool same_bits(const d4& a, const d4& b) { return std::memcmp(a.data(), b.data(), sizeof(a)) == 0; }
LaunchSetup& launched_body(const V& w, const V& source, const ProfileSetup& c, const V& launcher) {
    d3 position = vec_at<3>(launcher, "position"), velocity = vec_at<3>(launcher, "velocity"), angles = vec_at<3>(launcher, "angles");
    if (launch_setup && launch_setup->source.node() == source.node() && same_bits(launch_setup->position, position)
        && same_bits(launch_setup->velocity, velocity) && same_bits(launch_setup->angles, angles)) return *launch_setup;
    auto s = std::make_unique<LaunchSetup>();
    s->source = source; s->position = position; s->velocity = velocity; s->angles = angles;
    s->quaternion = call(attr(w, "quaternion"), {item(launcher, "angles")});
    V record = make_dict({{"position", item(launcher, "position")}, {"velocity", item(launcher, "velocity")},
        {"quaternion", s->quaternion}, {"omega", make_list({0., 0., 0.})}, {"time", 0.}});
    s->body = call(attr(w, "construct"), {record});
    s->launched = call(attr(w, "launch_body"), {c.rocket, deepcopy(s->body), 123, 0x12345678},
        {{"launch_time", 0.}, {"clock_time", 0.}, {"launch_mode", 2}, {"distance_limit", c.distance_limit}});
    launch_setup = std::move(s);
    return *launch_setup;
}
// constructed_guidance and guidance_launch.initialize for an acquired seeker: only
// the seeker block and the manager fields that alias it depend on the sample.
OpticalGuidance launched_optical_guidance(const OpticalGuidance& launched, const OpticalAcquired& a) {
    OpticalGuidance g = launched;
    g.seeker = a.state; g.seeker.los_check_time = 0.; g.seeker.los_cache_valid = false;
    g.opaque = a.raw;
    for (int i = 16; i < 20; ++i) g.opaque[i] = 255;
    g.opaque[20] = 0;
    if (g.opaque[0] == 1) bail("Optical point launch uses the translated setup");
    g.manager.seeker_direction = g.seeker.direction; g.manager.seeker_distance = g.seeker.distance;
    g.manager.seeker_point_valid = g.opaque[0] != 0;
    return g;
}
// The whole per-sample setup on typed state: missile_worker.quaternion, construct,
// launch_body, optical_reset/radar_reset, optical_ready/radar_ready and
// launch_guidance. Writes the launched body and guidance into S.
struct TypedLaunch { double lifetime; d3 origin; d4 launcher_q; };
TypedLaunch typed_setup(const ProfileSetup& c, const FlightProps& P, const d3& position, const d3& velocity, const d3& angles,
                        const TargetMotion& motion, Session& S) {
    d4 launcher_q = angles_quaternion(angles);
    wtm_body body = construct_body(position, launcher_q, velocity);
    LaunchedBody launched = launch_body(c.launch, body);
    d3 body_position = dv(body.position), body_velocity = dv(body.velocity); d4 body_q = dq(body.quaternion);
    if (P.family == 0) {
        auto none = [](const OpticalSeekerState&) -> OpticalObservation { bail("Optical reset observes nothing"); };
        OpticalModesResult reset = optical_modes_update(P.optical.seeker, c.optical_initial, body_q, -2 * STEP, -STEP, none, 0,
                                                        c.optical_initial_raw, true, true, false, -1.);
        S.optical = launched_optical_guidance(c.launched_optical, optical_prelaunch_acquire(P.optical, reset.state, reset.opaque, body_q,
                                                                                            body_position, motion.position));
    } else {
        RadarSeekerState reset = radar_reset_state(c.radar_launcher, c.radar_template, body_position, body_q, body_velocity);
        S.radar = c.launched_radar;
        radar_handoff(S.radar, c.launched_radar.seeker, radar_prelaunch(c.radar_launcher, P.radar.seeker, reset, body_position, body_q,
                                                                        body_velocity, motion));
    }
    S.body = launched.body;
    return {launched.lifetime, body_position, launcher_q};
}
// math.dist as the translated runtime computes it.
double reference_dist(const d3& a, const d3& b) {
    double d[3], maximum = 0.; int nan = 0;
    for (int i = 0; i < 3; ++i) { d[i] = std::fabs(a[i] - b[i]); if (d[i] > maximum) maximum = d[i]; nan |= std::isnan(d[i]); }
    return vector_norm(3, d, maximum, nan);
}
bool same_state(const Session& a, const Session& b, int family) {
    Snapshot x = snapshot(a, family), y = snapshot(b, family);
    if (x.size() != y.size()) return false;
    for (size_t i = 0; i < x.size(); ++i) if (x[i].first != y[i].first || bits(x[i].second) != bits(y[i].second)) return false;
    return true;
}
bool typed_log();
void diagnostic(const V& value);
// Keep the first sample's translated setup and enable the typed setup only if
// it reproduces that sample bit for bit.
void enable_typed_setup(ProfileSetup& c, const LaunchSetup& ls, const FlightProps& P, const Session& S, double lifetime, const d3& origin,
                        const d4& launcher_q, const V& launcher, const V& target, const TargetMotion& motion) {
    c.typed = -1;
    try {
        load_launch_props(c, c.launch);
        if (P.family == 0) {
            load_optical_seeker(field(invoke_fn("optical_prelaunch", "initial"), "seeker"), c.optical_initial, c.optical_initial_raw);
            c.launched_optical = S.optical;
        } else {
            load_radar_launcher(item(ls.reset, 0), c.radar_launcher);
            load_radar_seeker(field(item(c.release, "constructed_guidance"), "seeker"), c.radar_template);
            c.launched_radar = S.radar;
        }
        Session check = S;
        TargetMotion typed_motion = motion;
        typed_motion.quaternion = angles_quaternion(vec_at<3>(target, "angles"));
        TypedLaunch t = typed_setup(c, P, vec_at<3>(launcher, "position"), vec_at<3>(launcher, "velocity"), vec_at<3>(launcher, "angles"),
                                    typed_motion, check);
        if (same_state(check, S, P.family) && bits(t.lifetime) == bits(lifetime) && same_bits(t.origin, origin)
            && same_bits(t.launcher_q, launcher_q) && same_bits(typed_motion.quaternion, motion.quaternion)) c.typed = 1;
    } catch (const Bail&) {} catch (const PyError&) {}
}
void prepare_cached(const V& missile, const V& launcher, const V& target, const V& early_exit, const V& reach_m, Prepared& out) {
    V w = invoke_fn("missile_envelope", "_missile_worker");
    V source = invoke_fn("missile_envelope", "_profile", {missile});
    auto& c = prepared_profile(w, source, out.cache_hit);
    out.props = c.typed_props; out.family = c.family; out.distance_limit = c.distance_limit;
    d3 position = vec_at<3>(launcher, "position"), velocity = vec_at<3>(launcher, "velocity");
    TargetMotion motion{vec_at<3>(target, "position"), vec_at<3>(target, "velocity"), {}};
    Session& S = out.session;
    double lifetime; d3 origin; d4 launcher_q;
    if (c.typed == 1) {
        motion.quaternion = angles_quaternion(vec_at<3>(target, "angles"));
        TypedLaunch t = typed_setup(c, out.props, position, velocity, vec_at<3>(launcher, "angles"), motion, S);
        lifetime = t.lifetime; origin = t.origin; launcher_q = t.launcher_q;
        out.lifetime = V(lifetime);
    } else {
        LaunchSetup& ls = launched_body(w, source, c, launcher);
        V target_quaternion = call(attr(w, "quaternion"), {item(target, "angles")});
        motion.quaternion = vec<4>(target_quaternion);
        V body = deepcopy(ls.body);
        V templ = item(c.release, "constructed_guidance"), initialized;
        if (out.props.family == 0) {
            if (ls.reset.is_none())
                ls.reset = invoke_fn("missile_initialization", "optical_reset", {c.rocket, item(c.properties, "guidance"), item(ls.body, "quaternion")});
            initialized = call(attr(w, "optical_ready"), {c.rocket, item(c.properties, "guidance"), templ, body, target}, {{"reset", ls.reset}});
        } else {
            if (ls.reset.is_none()) ls.reset = invoke_fn("missile_initialization", "radar_reset", {item(c.profile, "asset"), templ, ls.body});
            V target_body = deepcopy(target);
            dict_set(target_body, "quaternion", target_quaternion);
            initialized = call(attr(w, "radar_ready"), {item(c.profile, "asset"), item(c.properties, "guidance"), templ, body, target_body},
                               {{"reset", ls.reset}});
        }
        V guidance = call(attr(w, "launch_guidance"), {c.launch_properties, item(initialized, 0), body, 123});
        load_body(item(ls.launched, "body"), S.body);
        if (out.props.family == 0) load_optical_state(item(guidance, "state"), S.optical);
        else load_radar_state(item(guidance, "state"), S.radar);
        out.lifetime = item(ls.launched, "lifetime"); lifetime = num(out.lifetime);
        origin = vec_at<3>(ls.body, "position"); launcher_q = vec<4>(ls.quaternion);
        if (c.typed == 0) {
            enable_typed_setup(c, ls, out.props, S, lifetime, origin, launcher_q, launcher, target, motion);
            if (typed_log()) diagnostic(make_dict({{"type", "typed_setup"}, {"missile", missile}, {"enabled", V(c.typed == 1)}}));
        }
    }
    S.controls = vec<8>(item(c.release, "controls"));
    S.lifetime = lifetime; S.distance_limit = num(c.distance_limit);
    S.origin = origin;
    S.proximity_origin = {S.body.position.x, S.body.position.y, S.body.position.z};
    S.proximity_enabled = reach_m.is_none(); S.offsets = {typed::f32(-.05), typed::f32(0.)};
    validate_flight_numbers(S, out.props.family);
    finite_or_bail(S.lifetime); finite_or_bail(S.distance_limit); finite_or_bail(S.origin);

    EvaluateSetup& E = out.setup;
    E.deadline = S.launch_time + S.lifetime;
    double count = std::ceil((S.lifetime + 1.) / STEP);
    if (!std::isfinite(count) || count < 0 || count > 1e9) bail("Unsupported step count");
    E.count = (int64_t)count;
    E.has_reach = !reach_m.is_none(); E.early_exit = truth(early_exit);
    if (E.has_reach) { E.reach_m = num(reach_m); E.radius = E.reach_m + 30.; }
    else {
        V radius = dict_get(c.proximity, "radius", 0.);
        E.radius = (truth(radius) ? num(radius) : 0.) + 30.;
    }
    E.burn_end = c.burn_end;
    E.best_distance = reference_dist({S.body.position.x, S.body.position.y, S.body.position.z}, motion.position);
    E.motion = motion;
    finite_or_bail(E.motion.position); finite_or_bail(E.motion.velocity);
    E.target_velocity_raw = E.motion.velocity;
    E.launch = {position, velocity, launcher_q};
}
bool cached_setup() {
    static const bool enabled = [] { const char* v = std::getenv("WT_TYPED_SETUP"); return !v || std::string(v) != "0"; }();
    return enabled;
}
bool typed_metrics() {
    static const bool enabled = [] { const char* v = std::getenv("WT_TYPED_METRICS"); return v && std::string(v) == "1"; }();
    return enabled;
}
void diagnostic(const V& value) {
    static std::mutex mutex;
    std::lock_guard<std::mutex> lock(mutex);
    std::cerr << json_dump(value) << std::endl;
}

V result_value(const Prepared& p, const EvaluateResult& r) {
    return make_dict({{"hit", V(r.hit)}, {"outcome", V(r.outcome)}, {"time_s", V(r.time_s)},
                      {"event_time_s", r.has_event_time ? V(r.event_time_s) : V()},
                      {"closest_m", V(r.closest_m)}, {"closest_time_s", V(r.closest_time_s)}, {"steps", V(r.steps)},
                      {"family", p.family}, {"lifetime_s", p.lifetime}, {"distance_limit_m", p.distance_limit}});
}

// Compare in lockstep, retaining only the current states and first 24 differing
// fields. Memory is independent of flight duration; no cumulative trace dump.
struct Comparison {
    const Prepared* prepared;
    Session state;
    int64_t steps = 0, first = -1;
    bool initial_equal = true;
    Args fields;
    void compare(const V& reference, int64_t step) {
        if (first != -1) return;
        Session expected;
        load_session(reference, prepared->props.family, expected);
        Snapshot x = snapshot(expected, prepared->props.family), y = snapshot(state, prepared->props.family);
        if (x.size() != y.size()) {
            first = step; fields.push_back(V("snapshot size differs")); return;
        }
        for (size_t i = 0; i < x.size(); ++i) {
            if (x[i].first != y[i].first || bits(x[i].second) != bits(y[i].second)) {
                first = step;
                if (fields.size() < 24) fields.push_back(make_dict({{"field", V(x[i].first)},
                    {"reference", V(x[i].second)}, {"typed", V(y[i].second)}}));
            }
        }
    }
};
thread_local Comparison* reference_trace = nullptr;
thread_local bool trace_installed = false;
void install_reference_trace() {
    if (trace_installed) return;
    trace_installed = true;
    V cls = get(module_env("missile_fast"), "FastInteractionSession");
    V original = attr(cls, "advance");
    setattr(cls, "advance", builtin("FastInteractionSession.advance", [original](const Args& args, const Kw& kwargs) {
        Comparison* c = reference_trace;
        if (c && !args.empty() && c->steps == 0) {
            c->compare(args[0], -2);
            c->initial_equal = c->first == -1;
        }
        V result = call(original, args, kwargs);
        if (c && !args.empty()) {
            const auto& p = *c->prepared;
            double now = add(c->state.body.time, STEP);
            interaction_advance(p.props, c->state, p.setup.motion, p.setup.launch, now, STEP, now);
            c->compare(args[0], c->steps++);
        }
        return result;
    }));
}

// missile_worker.escape_motion as computed by the translated function, so the
// path constants are the reference's; typed target_at evaluates it per time.
void load_escape(const V& m, EvaluateSetup& E) {
    if (field(m, "kind").as_string() != "escape") bail("typed loader: escape motion expected");
    TargetMotion& t = E.motion;
    t.position = vec_at<3>(m, "position"); t.velocity = vec_at<3>(m, "velocity"); t.quaternion = vec_at<4>(m, "quaternion");
    t.escape = true;
    t.start = num_at(m, "start"); t.end = num_at(m, "end"); t.heading = num_at(m, "heading");
    t.sine = num_at(m, "sine"); t.cosine = num_at(m, "cosine"); t.rate = num_at(m, "rate"); t.radius = num_at(m, "radius");
    t.speed = num_at(m, "speed"); t.roll = num_at(m, "roll");
    t.turn_point = vec_at<3>(m, "turn_point"); t.exit_point = vec_at<3>(m, "exit_point"); t.run = vec_at<3>(m, "run");
    t.run_quaternion = vec_at<4>(m, "run_quaternion");
    for (double x : {t.start, t.end, t.heading, t.sine, t.cosine, t.rate, t.radius, t.speed, t.roll}) finite_or_bail(x);
    finite_or_bail(t.position); finite_or_bail(t.velocity); finite_or_bail(t.quaternion);
    finite_or_bail(t.turn_point); finite_or_bail(t.exit_point); finite_or_bail(t.run); finite_or_bail(t.run_quaternion);
    E.target_velocity_raw = t.run; E.straight_from = t.end;
}

// missile_worker.escape_motion itself: the same expressions in the same order,
// with the runtime's math functions (std::fmod, atan2, sin, cos, sqrt, acos;
// degrees as x * (180 / pi)). Input it does not accept bails, and the
// translated function then raises its error.
void escape_setup(const V& target, const V& escape, const V& origin_value, EvaluateSetup& E) {
    constexpr double PI = 3.141592653589793238462643383279502884, DEGREES = 180.0 / PI;
    V worker = import_module("missile_worker");
    double gravity = num(field(worker, "ESCAPE_GRAVITY"));
    auto load_range = vec<2>(field(worker, "ESCAPE_LOAD_G")), delay_range = vec<2>(field(worker, "ESCAPE_DELAY_S"));
    d3 position = vec_at<3>(target, "position"), velocity = vec_at<3>(target, "velocity"), origin = vec<3>(origin_value);
    d3 angles = vec_at<3>(target, "angles");
    double delay = num_at(escape, "delay_s"), load = num_at(escape, "load_g");
    finite_or_bail(position); finite_or_bail(velocity); finite_or_bail(origin); finite_or_bail(angles);
    if (velocity[1] != 0) bail("typed escape: target not level");
    if (!(delay_range[0] <= delay && delay <= delay_range[1]) || !(load_range[0] <= load && load <= load_range[1]))
        bail("typed escape: load or delay out of range");
    auto wrapped = [&](double angle) {   // (-pi, pi]
        angle = std::fmod(angle, 2 * PI);
        if (std::isnan(angle)) bail("typed escape: math domain");
        if (angle > PI) angle -= 2 * PI;
        if (angle <= -PI) angle += 2 * PI;
        return angle;
    };
    double speed = std::sqrt(velocity[0] * velocity[0] + velocity[2] * velocity[2]);
    d3 turn_point;
    for (int i = 0; i < 3; ++i) turn_point[i] = position[i] + velocity[i] * delay;
    double heading = std::atan2(velocity[2], velocity[0]);
    double turn = wrapped(std::atan2(turn_point[2] - origin[2], turn_point[0] - origin[0]) - heading);
    double sine = std::sin(heading), cosine = std::cos(heading);
    double rate = 0., radius = 0., end = delay, exit_heading = heading;
    d3 exit_point = turn_point, run = velocity;
    if (speed > 0 && turn != 0) {
        rate = std::copysign(gravity * std::sqrt(load * load - 1.) / speed, turn);
        radius = speed / rate;
        double sign = std::copysign(1., turn);
        auto ahead = [&](double tau) {
            double h = heading + rate * tau;
            double x = turn_point[0] + radius * (std::sin(h) - sine), z = turn_point[2] - radius * (std::cos(h) - cosine);
            return sign * wrapped(std::atan2(z - origin[2], x - origin[0]) - h);
        };
        double step = PI / 180. / std::fabs(rate);
        double duration = turn / rate, previous = ahead(0.);
        for (int k = 1; k < 361; ++k) {
            double value = ahead(k * step);
            if (value <= 0. && previous < PI / 2) {
                double low = (k - 1) * step, high = k * step;
                for (int i = 0; i < 60; ++i) {
                    double middle = (low + high) / 2;
                    if (ahead(middle) > 0.) low = middle; else high = middle;
                }
                duration = high;
                break;
            }
            previous = value;
        }
        end = delay + duration;
        exit_heading = heading + rate * (end - delay);
        exit_point = {turn_point[0] + radius * (std::sin(exit_heading) - sine), turn_point[1],
                      turn_point[2] - radius * (std::cos(exit_heading) - cosine)};
        run = {speed * std::cos(exit_heading), 0., speed * std::sin(exit_heading)};
    }
    TargetMotion& t = E.motion;
    t = TargetMotion{};
    t.position = position; t.velocity = velocity; t.quaternion = angles_quaternion(angles);
    t.escape = true;
    t.start = delay; t.end = end; t.heading = heading; t.sine = sine; t.cosine = cosine; t.rate = rate; t.radius = radius;
    t.speed = speed; t.roll = std::copysign(std::acos(1. / load) * DEGREES, turn);
    t.turn_point = turn_point; t.exit_point = exit_point; t.run = run;
    t.run_quaternion = angles_quaternion({exit_heading * DEGREES, 0., 0.});
    for (double x : {t.start, t.end, t.heading, t.sine, t.cosine, t.rate, t.radius, t.speed, t.roll}) finite_or_bail(x);
    finite_or_bail(t.turn_point); finite_or_bail(t.exit_point); finite_or_bail(t.run);
    finite_or_bail(t.quaternion); finite_or_bail(t.run_quaternion);
    E.target_velocity_raw = t.run; E.straight_from = t.end;
}
bool same_motion(const TargetMotion& a, const TargetMotion& b) {
    auto eq = [](double x, double y) { return bits(x) == bits(y); };
    auto eqv = [&](const auto& x, const auto& y) { for (size_t i = 0; i < x.size(); ++i) if (!eq(x[i], y[i])) return false; return true; };
    return a.escape == b.escape && eq(a.start, b.start) && eq(a.end, b.end) && eq(a.heading, b.heading) && eq(a.sine, b.sine)
        && eq(a.cosine, b.cosine) && eq(a.rate, b.rate) && eq(a.radius, b.radius) && eq(a.speed, b.speed) && eq(a.roll, b.roll)
        && eqv(a.position, b.position) && eqv(a.velocity, b.velocity) && eqv(a.quaternion, b.quaternion)
        && eqv(a.turn_point, b.turn_point) && eqv(a.exit_point, b.exit_point) && eqv(a.run, b.run) && eqv(a.run_quaternion, b.run_quaternion);
}

V typed_evaluate(const V& reference, const Args& args, const Kw& kwargs) {
    Mode mode = typed_mode();
    if (mode == Mode::Off) return call(reference, args, kwargs);
    V a[5] = {V(), V(), V(), V(true), V()};
    static const char* names[5] = {"missile", "launcher", "target", "early_exit", "reach_m"};
    if (args.size() > 5) return call(reference, args, kwargs);
    for (size_t i = 0; i < args.size(); ++i) a[i] = args[i];
    bool given[5] = {false, false, false, false, false};
    for (size_t i = 0; i < args.size(); ++i) given[i] = true;
    for (const auto& pair : kwargs) {
        int index = -1; for (int i = 0; i < 5; ++i) if (pair.first == names[i]) index = i;
        if (index < 0 || given[index]) return call(reference, args, kwargs);
        a[index] = pair.second; given[index] = true;
    }
    if (!given[0] || !given[1] || !given[2]) return call(reference, args, kwargs);
    Prepared p;
    Comparison comparison{&p};
    int64_t typed_steps = 0;
    const char* declined = nullptr;
    V typed_result, escape_equal;
    try {
        auto began = std::chrono::steady_clock::now();
        // An escaping target (no-escape zone) is set up from its release state,
        // as missile_envelope.evaluate does.
        V target = a[2], escape = dict_get(a[2], V("escape"), V());
        if (!escape.is_none())
            target = make_dict({{"position", item(a[2], V("position"))}, {"velocity", item(a[2], V("velocity"))},
                                {"angles", item(a[2], V("angles"))}});
        (cached_setup() ? prepare_cached : prepare_full)(a[0], a[1], target, a[3], a[4], p);
        if (!escape.is_none()) {
            escape_setup(target, escape, item(a[1], V("position")), p.setup);
            if (mode == Mode::Compare) {
                EvaluateSetup reference_escape;
                load_escape(invoke_fn("missile_worker", "escape_motion", {target, escape, item(a[1], V("position"))}), reference_escape);
                escape_equal = V(same_motion(p.setup.motion, reference_escape.motion));
            }
        }
        auto prepared = std::chrono::steady_clock::now();
        if (mode == Mode::Compare) comparison.state = p.session;
        EvaluateResult r = evaluate_loop(p.props, p.session, p.setup);
        auto completed = std::chrono::steady_clock::now();
        typed_steps = r.steps;
        typed_result = result_value(p, r);
        if (typed_metrics()) diagnostic(make_dict({{"type", "typed_metrics"}, {"missile", a[0]},
            {"setup_s", std::chrono::duration<double>(prepared - began).count()},
            {"flight_s", std::chrono::duration<double>(completed - prepared).count()},
            {"steps", r.steps}, {"cached_setup", cached_setup()}, {"cache_hit", p.cache_hit}}));
    } catch (const Bail& b) { declined = b.reason; }
    catch (const PyError&) { declined = "setup raised"; }
    if (declined && typed_log())
        diagnostic(make_dict({{"type", "typed_fallback"}, {"missile", a[0]}, {"reason", V(std::string(declined))}}));
    if (mode == Mode::On) return declined ? call(reference, args, kwargs) : typed_result;
    // Compare: the reference result is returned; differences go to stderr.
    install_reference_trace();
    reference_trace = declined ? nullptr : &comparison;
    V expected;
    try { expected = call(reference, args, kwargs); } catch (...) { reference_trace = nullptr; throw; }
    reference_trace = nullptr;
    Kw report{{"type", "typed_compare"}, {"missile", a[0]}, {"declined", declined ? V(std::string(declined)) : V()}};
    report.emplace_back("cache_hit", V(p.cache_hit));
    report.emplace_back("setup_cached", V(cached_setup()));
    report.emplace_back("escape_equal", escape_equal);
    bool same = !declined && json_dump(expected, true) == json_dump(typed_result, true);
    report.emplace_back("result_equal", V(same));
    if (!declined && !same) { report.emplace_back("expected", expected); report.emplace_back("typed", typed_result); }
    if (!declined) {
        if (comparison.steps != typed_steps && comparison.first == -1) {
            comparison.first = std::min(comparison.steps, typed_steps);
            comparison.fields.push_back(V("step count differs"));
        }
        report.emplace_back("initial_equal", V(comparison.initial_equal));
        report.emplace_back("steps_reference", V(comparison.steps));
        report.emplace_back("steps_typed", V(typed_steps));
        report.emplace_back("first_mismatch_step", V(comparison.first));
        report.emplace_back("fields", make_list(comparison.fields));
    }
    diagnostic(make_dict(report));
    return expected;
}

// ---------------------------------------------------------- simulate entry
// missile_worker.simulate on typed state. The translated simulation_setup and
// simulation_document supply the release, seeker initialization and the result
// description; the step loop, samples, tracking descriptions, telemetry, closest
// approach and early-miss test reproduce the reference expressions. Legacy
// post-release requests, and anything the typed port declines, run the
// translated function from scratch.

// math.hypot as the translated runtime computes it.
double runtime_hypot(const double* v, int n) {
    double d[4], maximum = 0.; int nan = 0;
    for (int i = 0; i < n; ++i) { d[i] = std::fabs(v[i]); if (d[i] > maximum) maximum = d[i]; nan |= std::isnan(d[i]); }
    return vector_norm((size_t)n, d, maximum, nan);
}

// missile_tracking.describe
struct TrackingStatus {
    bool tracking = false; int available = -1;   // observation_available: -1 None
    bool has_reason = false; std::string reason, state;
    bool operator==(const TrackingStatus& o) const {
        return tracking == o.tracking && available == o.available && has_reason == o.has_reason && reason == o.reason && state == o.state;
    }
};
const char* loss_name(int reason) {
    switch (reason) {
        case LOSS_NO_TARGET: return "no_target"; case LOSS_BEHIND: return "behind"; case LOSS_MINIMUM_DISTANCE: return "minimum_distance";
        case LOSS_HORIZON: return "horizon"; case LOSS_FIELD_OF_VIEW: return "field_of_view";
        case LOSS_OBSERVATION_CONE: return "observation_cone"; case LOSS_SENSOR_GEOMETRY: return "sensor_geometry";
        default: return nullptr;
    }
}
TrackingStatus describe_step(int family, const Session& S, bool inertial_enabled, const ObservationLog& log) {
    TrackingStatus t;
    bool inertial_valid; int64_t mode;
    if (family == 0) { t.tracking = S.optical.manager.tracking; inertial_valid = S.optical.manager.inertial_valid; mode = S.optical.manager.mode; }
    else { t.tracking = S.radar.manager.tracking != 0; inertial_valid = S.radar.manager.inertial_valid != 0; mode = S.radar.manager.mode; }
    if (log.count) {
        t.available = log.any ? 1 : 0;
        if (const char* name = loss_name(log.last)) { t.has_reason = true; t.reason = name; }
    }
    if (t.tracking) t.state = t.available == 1 ? "observed" : "coasting";
    else if (inertial_enabled && inertial_valid && (mode == 0 || mode == 4)) t.state = "inertial";
    else t.state = "acquiring";
    return t;
}
TrackingStatus tracking_from(const V& d) {
    TrackingStatus t;
    t.tracking = truth(at_key(d, "tracking"));
    V available = at_key(d, "observation_available"), reason = at_key(d, "loss_reason");
    t.available = available.is_none() ? -1 : truth(available) ? 1 : 0;
    if (!reason.is_none()) { t.has_reason = true; t.reason = reason.as_string(); }
    t.state = at_key(d, "tracking_state").as_string();
    return t;
}
void put_tracking(Kw& kw, const TrackingStatus& t) {
    kw.emplace_back("tracking", V(t.tracking));
    kw.emplace_back("observation_available", t.available < 0 ? V() : V(t.available == 1));
    kw.emplace_back("tracking_state", V(t.state));
    kw.emplace_back("loss_reason", t.has_reason ? V(t.reason) : V());
}

V list3(const d3& v) { return make_list({V(v[0]), V(v[1]), V(v[2])}); }
V list4(const d4& v) { return make_list({V(v[0]), V(v[1]), V(v[2]), V(v[3])}); }

// missile_telemetry.completed: the final substep's force evaluation again.
struct Telemetry { double force_time, thrust, mass, propellant, drag, lift, cd, aoa, pressure, acceleration; };
Telemetry completed_telemetry(const FlightProps& P, const StepCapture& k, const std::array<double,8>& controls) {
    const wtm_body& b = k.force_input;
    double sub_dt = div(STEP, (double)P.precision);
    wtm_forces f;
    aero_forces(&P.body, b.position.y, b.velocity, b.quaternion, b.omega, (float)controls[0], (float)controls[1], (float)sub_dt,
                k.propulsion.world_force, k.propulsion.body_torque, k.propulsion.mass_lost, 0.0f, b.body_random, &f);
    check_kernel();
    double scale = std::fabs((double)perturbation_coefficients(0.0f, b.body_random).force_scale);
    double world[3] = {k.propulsion.world_force.x, k.propulsion.world_force.y, k.propulsion.world_force.z};
    double drag[3] = {f.baseline.drag.x, f.baseline.drag.y, f.baseline.drag.z};
    double lift[3] = {f.baseline.lift.x, f.baseline.lift.y, f.baseline.lift.z};
    double cosine = mx(-1., mn(1., (double)f.baseline.cosine));
    return {(double)b.time, runtime_hypot(world, 3), (double)f.effective_mass, (double)k.propulsion.mass_lost,
            runtime_hypot(drag, 3) * scale, runtime_hypot(lift, 3) * scale, (double)f.baseline.cd,
            std::acos(cosine) * (180.0 / 3.141592653589793238462643383279502884), (double)f.pressure,
            runtime_hypot(k.acceleration, 3)};
}
V telemetry_value(const Telemetry& t) {
    return make_dict({{"force_time_s", V(t.force_time)}, {"thrust_n", V(t.thrust)}, {"mass_kg", V(t.mass)},
                      {"propellant_used_kg", V(t.propellant)}, {"drag_n", V(t.drag)}, {"lift_n", V(t.lift)},
                      {"drag_coefficient", V(t.cd)}, {"aoa_deg", V(t.aoa)}, {"dynamic_pressure_pa", V(t.pressure)},
                      {"acceleration_mps2", V(t.acceleration)}});
}

// missile_worker.simulate's sample()
struct Sample {
    double time = 0.; d3 missile{}, target{}, velocity{}; double speed = 0.; d4 quaternion{}; double mach = 0., traveled = 0.;
    TrackingStatus tracking; bool has_telemetry = false; Telemetry telemetry{};
};
Sample take_sample(const Session& S, const TargetMotion& motion, double traveled, const TrackingStatus& tracking) {
    Sample s; const wtm_body& b = S.body;
    s.time = b.time; s.missile = {b.position.x, b.position.y, b.position.z};
    s.target = target_at(motion, s.time).position;
    s.velocity = {b.velocity.x, b.velocity.y, b.velocity.z};
    double total = 0.; for (double v : s.velocity) total = total + v * v;
    s.speed = py_sqrt(total);
    s.quaternion = dq(b.quaternion);
    wtm_air air = ::atmosphere((float)s.missile[1]); check_kernel();
    s.mach = runtime_hypot(s.velocity.data(), 3) / (double)air.sound_speed;
    s.traveled = traveled; s.tracking = tracking;
    return s;
}
V sample_value(const Sample& s, const V* telemetry = nullptr) {
    Kw kw{{"time_s", V(s.time)}, {"missile", list3(s.missile)}, {"target", list3(s.target)}, {"velocity", list3(s.velocity)},
          {"speed_mps", V(s.speed)}, {"quaternion", list4(s.quaternion)}, {"mach", V(s.mach)}, {"traveled_distance_m", V(s.traveled)}};
    put_tracking(kw, s.tracking);
    if (telemetry) kw.emplace_back("telemetry", *telemetry);
    else if (s.has_telemetry) kw.emplace_back("telemetry", telemetry_value(s.telemetry));
    return make_dict(kw);
}
// missile_worker.proximity_endpoint
struct Endpoint { double time; d3 missile, target; double traveled; };
V endpoint_value(const Endpoint& e) {
    return make_dict({{"kind", V("proximity_event")}, {"time_s", V(e.time)}, {"missile", list3(e.missile)}, {"target", list3(e.target)},
                      {"velocity", V()}, {"speed_mps", V()}, {"quaternion", V()}, {"tracking", V()}, {"observation_available", V()},
                      {"tracking_state", V("event")}, {"loss_reason", V()}, {"mach", V()}, {"telemetry", V()},
                      {"traveled_distance_m", V(e.traveled)}});
}

V simulate_loop(const V& request, const V& progress) {
    V w = import_module("missile_worker");
    V config = call(attr(w, "validate"), {request});
    if (!has(config, "launcher")) bail("Legacy post-release launch uses the translated simulator");
    V setup = call(attr(w, "simulation_setup"), {config});
    V session = at_key(setup, "session"), frame = at_key(setup, "frame");
    V session_flight = attr(attr(session, "frame"), "flight");
    std::string family_name = attr(session_flight, "family").as_string();
    FlightProps P;
    P.family = family_name == "optical" ? 0 : family_name == "radar" ? 1 : -1;
    if (P.family < 0) bail("Unsupported guidance family");
    load_flight_props(attr(session_flight, "properties"), item(attr(session_flight, "serialized_properties"), V("rocket")),
                      attr(session, "props"), P);
    Session S;
    load_session(session, P.family, S);
    validate_flight_numbers(S, P.family);
    V motion_value = at_key(setup, "target_motion");
    TargetMotion motion;
    if (field(motion_value, "kind").as_string() == "escape") {
        EvaluateSetup E; load_escape(motion_value, E); motion = E.motion;
    } else {
        motion.position = vec_at<3>(motion_value, "position"); motion.velocity = vec_at<3>(motion_value, "velocity");
        motion.quaternion = vec_at<4>(motion_value, "quaternion");
        finite_or_bail(motion.position); finite_or_bail(motion.velocity);
    }
    V launch_value = at_key(setup, "launch");
    LaunchState launch{vec_at<3>(launch_value, "position"), vec_at<3>(launch_value, "velocity"),
                       vec<4>(call(attr(w, "quaternion"), {at_key(launch_value, "angles")}))};
    bool inertial_enabled = truth(at_key(setup, "inertial_enabled"));
    bool early_exit = truth(at_key(config, "early_exit"));
    double straight_from = num(at_key(setup, "straight_from"));
    d3 straight_velocity = vec_at<3>(setup, "straight_velocity");
    std::vector<double> burn_end; for (V end : iter(at_key(setup, "burn_end"))) burn_end.push_back(num(end));
    double deadline = num(at_key(setup, "deadline")), reach_radius = num(at_key(setup, "reach_radius"));
    double duration = num(at_key(config, "duration"));
    V initialization = at_key(setup, "initialization");
    V describe = attr(import_module("missile_tracking"), "describe");
    V session_state = attr(session_flight, "state");
    TrackingStatus tracking = tracking_from(call(describe, {V(family_name), item(item(session_state, V("guidance")), V("manager")),
                                                           V(inertial_enabled)}, {{"initialization", initialization}}));

    Args rows, transitions, seeker_transitions;
    double traveled = 0.;
    auto seeker_entry = [](double time, const TrackingStatus& t) {
        Kw kw{{"time_s", V(time)}}; put_tracking(kw, t); return make_dict(kw);
    };
    seeker_transitions.push_back(seeker_entry(0., tracking));
    V target_value = at_key(config, "target");
    double best_distance = reference_dist(vec_at<3>(attr(frame, "website_release"), "position"), vec_at<3>(target_value, "position"));
    double best_time = 0.;
    bool tracked_before = truth(at_key(initialization, "tracking"));
    if (tracked_before) transitions.push_back(make_dict({{"time_s", V(0.)}, {"tracking", V(true)}}));
    V initial_telemetry = call(attr(import_module("missile_telemetry"), "initial"), {attr(attr(frame, "flight"), "properties")});
    Sample first = take_sample(S, motion, traveled, tracking);
    rows.push_back(sample_value(first, &initial_telemetry));
    const char* code = "time_limit"; const char* label = "Time limit reached \xe2\x80\x94 unresolved";
    bool outcome_event = false; Event event_value{};
    double count_value = std::ceil(duration / STEP);
    if (!std::isfinite(count_value) || count_value < 0 || count_value > 1e9) bail("Unsupported step count");
    int64_t count = (int64_t)count_value;
    Sample last_new; bool have_new = false;
    for (int64_t i = 0; i < count; ++i) {
        Sample old = take_sample(S, motion, traveled, tracking);
        double now = add(old.time, STEP);
        std::array<double,8> controls = S.controls;
        ObservationLog log; StepCapture capture;
        observation_log = &log; step_capture = &capture;
        Expiry expiry;
        try { expiry = interaction_advance(P, S, motion, launch, now, STEP, now); }
        catch (...) { observation_log = nullptr; step_capture = nullptr; throw; }
        observation_log = nullptr; step_capture = nullptr;
        if (capture.updates != 1) bail("Simulator step made other than one flight update");
        TrackingStatus previous = tracking;
        tracking = describe_step(P.family, S, inertial_enabled, log);
        Sample now_sample = take_sample(S, motion, traveled, tracking);
        bool event = S.has_event;
        // endpoint: the proximity event, or the new sample itself (aliased)
        Endpoint end{};
        if (event) {
            if (!(old.time <= S.event.time && S.event.time <= now_sample.time)) bail("ValueError: proximity time outside the step");
            end.time = S.event.time; end.missile = S.event.position; end.target = target_at(motion, end.time).position;
        } else { end.time = now_sample.time; end.missile = now_sample.missile; end.target = now_sample.target; }
        traveled = traveled + reference_dist(old.missile, end.missile);
        if (event) end.traveled = traveled; else now_sample.traveled = traveled;
        auto seg = closest_segment(old.missile, end.missile, old.target, end.target);
        if (seg.first < best_distance) { best_distance = seg.first; best_time = old.time + seg.second * (end.time - old.time); }
        if (!event && now_sample.tracking.tracking != tracked_before) {
            transitions.push_back(make_dict({{"time_s", V(now_sample.time)}, {"tracking", V(now_sample.tracking.tracking)}}));
            tracked_before = now_sample.tracking.tracking;
        }
        bool terminal = false;
        if (event) { code = "proximity_event"; label = "Target proximity event"; outcome_event = true; event_value = S.event; terminal = true; }
        else if (now_sample.missile[1] <= 0) { code = "ground"; label = "Missile reached ground"; terminal = true; }
        else if (now_sample.target[1] <= 0) { code = "target_ground"; label = "Target reached ground \xe2\x80\x94 scenario ended"; terminal = true; }
        else if (S.expiry_flag) {
            if (expiry.time_expired) { code = "lifetime"; label = "Missile lifetime expired"; }
            else if (expiry.distance_expired) { code = "distance_limit"; label = "Missile distance limit reached"; }
            else { code = "tracking_timeout"; label = "Geometric tracking timeout"; }
            terminal = true;
        } else if (early_exit && i % 6 == 5 && now_sample.time >= straight_from) {
            bool burned = true;
            size_t n = burn_end.size() < 4 ? burn_end.size() : 4;
            for (size_t k = 0; k < n; ++k) if (!((double)S.body.clocks[k] >= burn_end[k] + 1e-3)) { burned = false; break; }
            if (burned && unreachable(now_sample.missile, now_sample.speed, now_sample.target, straight_velocity,
                                      deadline - now_sample.time + 2 * STEP, reach_radius, S.origin, S.distance_limit)) {
                code = "unreachable"; label = "Miss \xe2\x80\x94 target is unreachable"; terminal = true;
            }
        }
        if (event) {
            seeker_transitions.push_back(make_dict({{"time_s", V(end.time)}, {"tracking", V()}, {"observation_available", V()},
                                                    {"tracking_state", V("event")}, {"loss_reason", V()}}));
        } else if (!(tracking == previous)) seeker_transitions.push_back(seeker_entry(now_sample.time, tracking));
        if (i % 3 == 2 || terminal || i == count - 1) {
            now_sample.telemetry = completed_telemetry(P, capture, controls); now_sample.has_telemetry = true;
            if (event) {
                now_sample.traveled = old.traveled + reference_dist(old.missile, now_sample.missile);
                V last = rows.back();
                bool same_time = num(at_key(last, "time_s")) == end.time;
                if (same_time) rows.back() = endpoint_value(end); else rows.push_back(endpoint_value(end));
            } else rows.push_back(sample_value(now_sample));
        }
        if (i % 24 == 23) call(progress, {make_dict({{"time_s", V(now)}, {"duration_s", V(duration)}, {"fraction", V(mn(1., now / duration))}})});
        last_new = now_sample; have_new = true;
        if (terminal) break;
    }
    if (!have_new) bail("No simulator step");
    Kw outcome{{"code", V(code)}, {"label", V(label)}};
    if (outcome_event) outcome.emplace_back("event", make_dict({{"position", list3(event_value.position)},
                                                                {"direction", list3(event_value.direction)}, {"time", V(event_value.time)}}));
    V best = make_dict({{"distance_m", V(best_distance)}, {"time_s", V(best_time)}});
    return call(attr(w, "simulation_document"), {config, setup, make_list(rows), make_list(transitions), make_list(seeker_transitions),
                                                 best, make_dict(outcome), sample_value(last_new)});
}

V typed_simulate(const V& reference, const Args& args, const Kw& kwargs) {
    Mode mode = typed_mode();
    if (mode == Mode::Off || args.empty() || args.size() > 2 || !kwargs.empty()) return call(reference, args, kwargs);
    V progress = args.size() > 1 ? args[1] : builtin("progress", [](const Args&, const Kw&) { return V(); });
    const char* declined = nullptr; V typed_result;
    try { typed_result = simulate_loop(args[0], progress); }
    catch (const Bail& b) { declined = b.reason; }
    catch (const PyError&) { declined = "setup raised"; }
    if (declined && typed_log())
        diagnostic(make_dict({{"type", "typed_simulate_fallback"}, {"reason", V(std::string(declined))}}));
    if (mode == Mode::On) return declined ? call(reference, args, kwargs) : typed_result;
    V expected = call(reference, args, kwargs);
    bool same = !declined && json_dump(expected, true) == json_dump(typed_result, true);
    Kw report{{"type", "typed_simulate_compare"}, {"declined", declined ? V(std::string(declined)) : V()}, {"result_equal", V(same)}};
    if (!declined && !same) {
        // First differing trajectory row, for diagnosis.
        Args a = iter(item(expected, V("trajectory"))), b = iter(item(typed_result, V("trajectory")));
        report.emplace_back("rows_reference", V((int64_t)a.size())); report.emplace_back("rows_typed", V((int64_t)b.size()));
        for (size_t i = 0; i < a.size() && i < b.size(); ++i)
            if (json_dump(a[i], true) != json_dump(b[i], true)) {
                report.emplace_back("first_row", V((int64_t)i)); report.emplace_back("reference", a[i]); report.emplace_back("typed", b[i]); break;
            }
        for (const char* key : {"outcome", "closest_approach", "tracking_transitions", "seeker_transitions", "final_body_sample"})
            if (json_dump(item(expected, V(key)), true) != json_dump(item(typed_result, V(key)), true)) report.emplace_back(std::string("differs_") + key, V(true));
    }
    diagnostic(make_dict(report));
    return expected;
}
}  // namespace

void install_typed_evaluate(E env) {
    V reference = get(env, "evaluate");
    set(env, "evaluate", builtin("evaluate", [reference](const Args& args, const Kw& kwargs) {
        return typed_evaluate(reference, args, kwargs);
    }));
}

void install_typed_simulate(E env) {
    V reference = get(env, "simulate");
    set(env, "simulate", builtin("simulate", [reference](const Args& args, const Kw& kwargs) {
        return typed_simulate(reference, args, kwargs);
    }));
}
}  // namespace nt
