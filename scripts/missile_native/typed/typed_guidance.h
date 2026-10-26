// Typed ports of the guidance managers, their controller services, flight
// inertial navigation, SARH illumination and the website's ideal radar support.
#pragma once
#include "typed_radar.h"

namespace typed {

using Packet = std::array<double,14>;   // position, xyzw quaternion, velocity, omega, time

// ------------------------------------------------------ controller services
struct ControlProps {
    wtm_orientation_props orientation;
    wtm_guidance_props guidance;
    wtm_acceleration_props acceleration;
    wtm_aero_props aero;
    wtm_motors motors;
    wtm_propulsion_control_props propulsion;
};
struct OrientationCtl { bool enabled; float pid[2][8]; };
struct GuidanceCtl { int64_t mode, course; float pid[2][8]; };
struct PropulsionCtl { int64_t stages[4]; float pid[8]; };
struct Controllers { OrientationCtl orientation; GuidanceCtl guidance; PropulsionCtl propulsion; };

struct OrientationResult { bool override_; d2 fins, auxiliary; };
// optical_manager_controls.orientation through engine_kernels' verified adapter.
inline OrientationResult orientation_service(const ControlProps& p, OrientationCtl& s, double time, double range_squared,
        const d4& q, const d3& omega, const d2& angles, double context_value, double dt) {
    wtm_orientation_state st{}; st.enabled = s.enabled ? 1 : 0;
    std::memcpy(st.pid, s.pid, sizeof st.pid);
    float a[2] = {(float)angles[0], (float)angles[1]};
    wtm_orientation_out out = ::orientation_update(&p.orientation, &st, (float)time, (float)range_squared, fq(q), fv(omega), a,
                                                   (float)context_value, (float)dt);
    check_kernel();
    OrientationResult r{out.override != 0, {0., 0.}, {0., 0.}};
    if (s.enabled && !st.enabled) s.enabled = false;
    if (!out.active) return r;
    std::memcpy(s.pid, st.pid, sizeof st.pid);
    if (p.orientation.use_fins) r.fins = {out.fins[0], out.fins[1]};
    else r.auxiliary = {out.auxiliary[0], out.auxiliary[1]};
    return r;
}
// optical_manager_controls.guidance (+ motor_scalar and acceleration_control).
inline d2 guidance_service(const ControlProps& p, GuidanceCtl& gs, const float* clocks, double time, double time_to_hit, double range_squared,
        const d3& position, const d4& q, const d3& velocity, const d3& feedback, const d3& direction, const d3& angular_rate,
        const d3& navigation, double dt, const d2& fins_in, bool matrix_velocity_frame) {
    wtm_request command = ::guidance_request(&p.guidance, (int)gs.mode, (int)gs.course, fv(direction), fv(angular_rate), fv(navigation),
                                             fq(q), fv(velocity), fv(position), (float)time, (float)time_to_hit, (float)range_squared, 0, 0.0f);
    check_kernel();
    d2 fins = {0., 0.};
    if (!command.has_acceleration) return fins;
    gs.mode = command.mode; gs.course = command.course;
    float times[4] = {};
    for (int i = 0; i < p.motors.count; ++i) times[i] = clocks[i];
    wtm_motor_scalar scalar = ::motor_scalar(&p.motors, times);
    check_kernel();
    wtm_motor_scalar motor{}; motor.thrust = scalar.thrust; motor.mass_lost = scalar.mass_lost;
    float pid[16];
    for (int i = 0; i < 2; ++i) for (int j = 0; j < 8; ++j) pid[i * 8 + j] = gs.pid[i][j];
    wtm_acceleration_out out{};
    ::acceleration_update(&p.acceleration, &p.aero, motor, pid, command.acceleration, fv(feedback), fq(q), fv(velocity),
                          (float)position[1], (float)time, (float)dt, matrix_velocity_frame ? 1 : 0, &out);
    check_kernel();
    if (!out.has_fins) return fins_in;
    for (int i = 0; i < 2; ++i) for (int j = 0; j < 8; ++j) gs.pid[i][j] = pid[i * 8 + j];
    return {out.fins[0], out.fins[1]};
}
inline d4 propulsion_service(const ControlProps& p, PropulsionCtl& s, double time, const d3& position, const d3& velocity,
                             double time_to_hit, double closure, double dt) {
    if (!p.propulsion.count) return {1., 1., 1., 1.};
    wtm_propulsion_control_state st{}; bool cruising = false;
    for (int i = 0; i < 4; ++i) { st.stages[i] = (int)s.stages[i]; if (i < p.propulsion.count && st.stages[i] == 2) cruising = true; }
    std::memcpy(st.pid, s.pid, sizeof st.pid);
    float factors[4];
    ::propulsion_control_update(&p.propulsion, &st, (float)time, fv(position), fv(velocity), (float)time_to_hit, (float)closure, (float)dt, factors);
    check_kernel();
    if (cruising) std::memcpy(s.pid, st.pid, sizeof st.pid);
    for (int i = 0; i < 4; ++i) s.stages[i] = st.stages[i];
    return {factors[0], factors[1], factors[2], factors[3]};
}

inline d3 p3(const Packet& v, int i) { return {v[i], v[i + 1], v[i + 2]}; }
inline d4 p4(const Packet& v, int i) { return {v[i], v[i + 1], v[i + 2], v[i + 3]}; }

// ------------------------------------------------------------ optical manager
struct OpticalManagerProps {
    double lock_timeout; bool inertial_navigation; double lock_distance, lock_cone_threshold;
    bool reacquire; double leading_gain; d2 leading_times; double scan_half_sector, half_fov;
    bool auxiliary_flag, designation_notification;
};
struct OpticalManagerState {
    double transition_time, scan_phase, strength, distance, seeker_distance;
    int64_t mode, count;
    bool active, auxiliary_flag, engaged, tracking, point_valid, seeker_point_valid, inertial_valid;
    int64_t seeker_flag;
    d2 fins, orientation_auxiliary; d4 factors; d3 auxiliary, direction, seeker_direction, inertial_point, inertial_offset;
};
struct OpticalProps {
    OpticalManagerProps manager; ControlProps controls; OpticalSeekerProps seeker; OpticalObservationProps observation;
    double los_check_timeout;
};
struct OpticalGuidance { OpticalManagerState manager; Controllers controllers; OpticalSeekerState seeker; Opaque opaque; };
struct OpticalTarget { d3 position; d3 auxiliary; };

inline d2 leading_angles(const d2& angles, const d3& rate_in, const d4& q_in, double gain) {
    double x = f32(q_in[0]), y = f32(q_in[1]), z = f32(q_in[2]), w = f32(q_in[3]);
    double rx = f32(rate_in[0]), ry = f32(rate_in[1]), rz = f32(rate_in[2]);
    double xz = mul(x, z), yw = mul(y, w), tworx = add(rx, rx);
    double first = mul(add(yw, xz), tworx);
    double yz2 = mul(add(y, y), z), minus2wx = mul(mul(-2., w), x);
    first = add(mul(add(minus2wx, yz2), ry), first);
    double zz2 = add(mul(z, z), mul(z, z)), zw = mul(z, w);
    double wwminus1 = add(add(mul(w, w), mul(w, w)), -1.);
    first = add(mul(add(zz2, wwminus1), rz), first);
    double second = mul(sub(zw, mul(x, y)), tworx);
    second = sub(second, mul(add(add(mul(y, y), mul(y, y)), wwminus1), ry));
    second = add(mul(sub(minus2wx, yz2), rz), second);
    return {add(mul(second, gain), angles[0]), sub(angles[1], mul(first, gain))};
}

// optical_point_controls.update -> optical_manager_controls.update -> optical_manager.update
inline void optical_guidance_update(const OpticalProps& P, OpticalGuidance& g, const Packet& old_in, const Packet& new_in, double elapsed_in,
        const float* clocks, const OpticalTarget& target, double context_value, bool matrix_velocity_frame) {
    const OpticalManagerProps& p = P.manager;
    if (p.designation_notification) bail("Designation notification remains separate");
    OpticalManagerState s = g.manager;
    double elapsed = f32(elapsed_in);
    Packet old = f32s(old_in), nw = f32s(new_in);
    double dt = sub(nw[13], old[13]);
    bool tracking = false;
    if (!s.active) { s.strength = 0.; s.fins = {0., 0.}; s.count = 0; s.auxiliary_flag = false; }
    else {
        int64_t mode = s.mode;
        if ((mode & 0xffffffffLL) < 2) {
            bool begin = elapsed > p.lock_timeout;
            if (p.inertial_navigation && s.inertial_valid) {
                d3 d; for (int i = 0; i < 3; ++i) d[i] = sub(add(s.inertial_offset[i], s.inertial_point[i]), nw[i]);
                double distance = add(add(mul(d[2], d[2]), mul(d[1], d[1])), mul(d[0], d[0]));
                d3 terms; for (int i = 0; i < 3; ++i) terms[i] = mul(d[i], s.seeker_direction[i]);
                double projection = add(terms[2], add(terms[1], terms[0]));
                begin = begin && mul(std::fabs(projection), projection) > mul(p.lock_cone_threshold, distance) && distance < mul(p.lock_distance, p.lock_distance);
            }
            if (begin) { s.mode = 3; s.transition_time = nw[13]; }
        } else if (mode == 5 && sub(nw[13], (double)py_trunc(nw[13])) < sub(old[13], (double)py_trunc(old[13]))) {
            s.mode = 3; s.transition_time = nw[13];
        }
        mode = s.mode;
        static const int64_t seeker_modes[6] = {6, 6, 6, 7, 8, 6};
        int64_t seeker_mode = 0 <= mode && mode <= 5 ? seeker_modes[mode] : 0;
        // optical_point_controls.seeker
        OpticalSeekerState seeker_state = g.seeker;
        const Opaque raw = g.opaque;
        d4 q = p4(nw, 3); d3 origin = p3(nw, 0);
        double old_time = old[13], new_time = nw[13];
        bool collect = seeker_mode == 5 || seeker_mode == 8;
        auto observation = [&](const OpticalSeekerState& temporary) {
            return geometric_optical_observe(P.observation, temporary, raw, q, origin, target.position, target.auxiliary, collect);
        };
        OpticalModesResult result = optical_modes_update(P.seeker, seeker_state, q, old_time, new_time, observation, seeker_mode, raw,
                                                         true, true, false, P.los_check_timeout);
        g.seeker = result.state; g.opaque = result.opaque;
        const OpticalOutputs& obs = result.outputs;
        // child = outputs + scan_phase (unchanged), direction, distance, point_valid
        s.seeker_direction = g.seeker.direction; s.seeker_distance = g.seeker.distance; s.seeker_point_valid = g.opaque[0] != 0;
        tracking = obs.tracking;
        d3 rate = tracking ? obs.angular_rate : d3{0., 0., 0.};
        d2 angles = obs.angles;
        if (p.leading_gain > 0. && elapsed < add(p.leading_times[0], p.leading_times[1]))
            angles = leading_angles(angles, rate, p4(nw, 3), p.leading_gain);
        OrientationResult orientation = orientation_service(P.controls, g.controllers.orientation, elapsed, 0., p4(nw, 3), p3(nw, 10),
                                                            angles, f32(context_value), dt);
        s.fins = orientation.fins; s.orientation_auxiliary = orientation.auxiliary;
        if (!orientation.override_) {
            if (tracking || p.inertial_navigation) {
                d3 feedback; for (int i = 0; i < 3; ++i) feedback[i] = mul(sub(nw[7 + i], old[7 + i]), safe_div(1., dt));
                s.fins = guidance_service(P.controls, g.controllers.guidance, clocks, elapsed, 0., 0., p3(nw, 0), p4(nw, 3), p3(nw, 7),
                                          feedback, s.seeker_direction, rate, p3(nw, 7), dt, s.fins, matrix_velocity_frame);
                s.factors = propulsion_service(P.controls, g.controllers.propulsion, elapsed, p3(nw, 0), p3(nw, 7), 0., 0., dt);
            } else s.fins = {0., 0.};
        }
        s.strength = obs.strength; s.count = obs.count; s.auxiliary_flag = p.auxiliary_flag; s.auxiliary = obs.auxiliary;
    }
    if (tracking || (s.mode != 0 && !p.reacquire)) s.mode = 4;
    else if (s.mode == 4) { s.mode = 3; s.transition_time = nw[13]; }
    else if (s.mode == 3 && !(p.scan_half_sector > p.half_fov)) s.mode = 5;
    s.engaged = s.mode >= 3; s.tracking = tracking; s.direction = s.seeker_direction;
    s.distance = s.seeker_distance; s.point_valid = s.seeker_point_valid;
    g.manager = s;
}

// ------------------------------------------------------------ flight_inertial
struct InertialProps { double drift_speed; int64_t gnss, datalink, reconnect, channels_max, keep_channels_for_last; };
struct InertialState { int64_t valid; d3 point, velocity, offset; int64_t source_tag; std::array<int64_t,6> padding; };
struct Record {
    int64_t source, source_tag, target_id; double time;
    int64_t origin_override, point_valid, velocity_valid, flag48, priority;
    d3 origin, point, direction, relative_velocity;
};
struct NoiseTables { std::array<int64_t,514> permutation; std::array<double,514> gradients; };
// noise_tables.initialize() with its fixed startup seed (the draw order covers
// all four tables; only the permutation and 1-D gradients are read here).
inline const NoiseTables& default_noise_tables() {
    static const NoiseTables tables = [] {
        uint64_t seed = 0x42c5ffb1u;
        auto draw = [&]() { seed = (seed * 0x41c64e6dull + 0x3039ull) & 0xffffffffull; return mul((double)((int64_t)((seed >> 16) & 511) - 256), 1. / 256); };
        std::array<int64_t,256> permutation; std::array<double,256> gradients;
        for (int i = 0; i < 256; ++i) permutation[i] = i;
        for (int i = 0; i < 256; ++i) {
            gradients[i] = draw();
            draw(); draw();            // gradients2 draws
            draw(); draw(); draw();    // gradients3 draws
        }
        for (int i = 255; i > 0; --i) {
            seed = (seed * 0x41c64e6dull + 0x3039ull) & 0xffffffffull;
            int64_t j = (int64_t)((seed >> 16) & 255);
            std::swap(permutation[i], permutation[(size_t)j]);
        }
        NoiseTables t;
        for (int i = 0; i < 514; ++i) { t.permutation[i] = permutation[i % 256]; t.gradients[i] = gradients[i % 256]; }
        return t;
    }();
    return tables;
}
inline double square_zyx(const d3& v) { return add(add(mul(v[2], v[2]), mul(v[1], v[1])), mul(v[0], v[0])); }
inline d3 relative_rate(const d3& delta, const d3& velocity) {
    double x = delta[0], y = delta[1], z = delta[2], vx = velocity[0], vy = velocity[1], vz = velocity[2];
    double squared = square_zyx(delta);
    double inverse = squared > f32(4e-19) ? div(1., squared) : 0.;
    return {mul(sub(mul(vy, z), mul(vz, y)), inverse), mul(sub(mul(vz, x), mul(vx, z)), inverse), mul(sub(mul(vx, y), mul(vy, x)), inverse)};
}
inline d3 body_direction(const d4& quaternion, const d3& vector) {
    double x = f32(quaternion[0]), y = f32(quaternion[1]), z = f32(quaternion[2]), w = f32(quaternion[3]);
    double vx = vector[0], vy = vector[1], vz = vector[2];
    double z2 = add(z, z), xz = mul(x, z2), mw2 = mul(-2., w);
    double wy = mul(y, mw2), yz = mul(z2, y), wx = mul(x, mw2), wz = mul(mw2, z);
    double xy = mul(add(x, x), y), ww = add(add(mul(w, w), mul(w, w)), -1.);
    double xx = add(add(mul(x, x), mul(x, x)), ww), yy = add(add(mul(y, y), mul(y, y)), ww), zz = add(add(mul(z, z), mul(z, z)), ww);
    return {add(add(mul(add(wy, xz), vz), mul(sub(xy, wz), vy)), mul(xx, vx)),
            add(mul(sub(yz, wx), vz), add(mul(yy, vy), mul(add(xy, wz), vx))),
            add(add(mul(zz, vz), mul(add(wx, yz), vy)), mul(sub(xz, wy), vx))};
}
struct Projection { int64_t valid; d3 direction, angular_rate; double range, closure; };
inline Projection project(const InertialState& state, const d3& origin, const d3& velocity, const d3& provider_origin,
                          const d3& provider_velocity, double rate_limit) {
    int64_t valid = state.valid & 255;
    if (!valid) return {valid, {1., 0., 0.}, {0., 0., 0.}, 0., 0.};
    d3 point, delta, mv, pv, bv, provider_delta, baseline;
    for (int i = 0; i < 3; ++i) point[i] = add(state.offset[i], state.point[i]);
    for (int i = 0; i < 3; ++i) delta[i] = sub(point[i], origin[i]);
    double squared = square_zyx(delta);
    double inverse = squared > f32(4e-19) ? div(1., squared) : 0.;
    d3 rel; for (int i = 0; i < 3; ++i) rel[i] = sub(state.velocity[i], velocity[i]);
    d3 rate = relative_rate(delta, rel);
    double rate_squared = add(mul(rate[2], rate[2]), add(mul(rate[1], rate[1]), mul(rate[0], rate[0])));
    rate_limit = f32(rate_limit);
    if (rate_squared > mul(rate_limit, rate_limit)) {
        double scale = div(rate_limit, sqrt32(rate_squared));
        for (double& x : rate) x = mul(x, scale);
    }
    d3 direction; for (int i = 0; i < 3; ++i) direction[i] = mul(delta[i], sqrt32(inverse));
    for (int i = 0; i < 3; ++i) { provider_delta[i] = sub(point[i], provider_origin[i]); baseline[i] = sub(origin[i], provider_origin[i]); }
    double provider_range = sqrt32(square_zyx(provider_delta)), baseline_range = sqrt32(square_zyx(baseline));
    double provider_inverse = provider_range > f32(4e-19) ? div(1., provider_range) : 0.;
    double baseline_inverse = baseline_range > f32(4e-19) ? div(1., baseline_range) : 0.;
    double distance = mul(sub(add(provider_range, sqrt32(squared)), baseline_range), .5);
    for (int i = 0; i < 3; ++i) { pv[i] = sub(provider_velocity[i], state.velocity[i]); mv[i] = sub(velocity[i], state.velocity[i]); bv[i] = sub(velocity[i], provider_velocity[i]); }
    double provider_speed = mul(add(add(mul(pv[2], provider_delta[2]), mul(pv[1], provider_delta[1])), mul(pv[0], provider_delta[0])), provider_inverse);
    double missile_speed = add(mul(mv[2], direction[2]), add(mul(mv[1], direction[1]), mul(mv[0], direction[0])));
    double baseline_speed = add(mul(add(mul(bv[0], baseline[0]), mul(bv[1], baseline[1])), baseline_inverse), mul(mul(baseline[2], baseline_inverse), bv[2]));
    double closure = mul(add(baseline_speed, add(missile_speed, provider_speed)), .5);
    return {valid, direction, rate, distance, closure};
}
inline double noise(double coordinate, const NoiseTables& t) {
    double value = f32(coordinate);
    int64_t base = py_floor(value);
    double fraction = sub(value, f32((double)base));
    double smooth = f32((3. - 2. * fraction) * mul(fraction, fraction));
    double left = mul(fraction, t.gradients[(size_t)t.permutation[(size_t)(base & 255)]]);
    double right = mul(add(fraction, -1.), t.gradients[(size_t)t.permutation[(size_t)((base + 1) & 255)]]);
    return add(mul(sub(right, left), smooth), left);
}
// Returns the record index or -1 (select_record's None).
inline int select_record(const InertialProps& p, int64_t datalink, int64_t state_source_tag, const std::vector<Record>& records,
                         int64_t channel_first, int64_t channel_last, int64_t source_mask) {
    int64_t channel = p.keep_channels_for_last ? channel_last : channel_first;
    if ((datalink & 255) != 1 || channel >= p.channels_max) return -1;
    for (size_t index = 0; index < records.size(); ++index) {
        const Record& record = records[index];
        if (!(((uint64_t)(source_mask & 0xffffffffLL) >> (record.source & 31)) & 1)) continue;
        bool eligible = (p.reconnect & 1) ? (record.flag48 & 255) == 1 : (record.source_tag & 255) == (state_source_tag & 255);
        if (eligible && (record.point_valid & 255) != 0) return (int)index;
    }
    return -1;
}
struct InertialView { const std::vector<Record>* records; d3 point, velocity; };
struct InertialResult { InertialState state; d2 angles; d3 rate; };
inline InertialResult inertial_update(const InertialProps& p, const InertialState& state, const InertialView& view, const d3& origin,
        const d4& quaternion, const d3& velocity, bool use_seeker, const d3& direction, const d3& angular_rate, int64_t range_valid,
        double distance, double radial_speed, int64_t channel_first, int64_t channel_last, int64_t source_mask, double time, double dt,
        const d2& angles, const d3& rate, double lag_limit) {
    InertialResult result{state, angles, rate};
    InertialState& out = result.state;
    int selected = select_record(p, p.datalink, state.source_tag, *view.records, channel_first, channel_last, source_mask);
    const Record* record = selected >= 0 ? &(*view.records)[(size_t)selected] : nullptr;
    d3 fallback = record && (record->origin_override & 255) == 1 ? record->origin : view.point;
    d3 offsets = state.offset, old_point, delta;
    for (int i = 0; i < 3; ++i) old_point[i] = add(state.point[i], offsets[i]);
    for (int i = 0; i < 3; ++i) delta[i] = sub(old_point[i], origin[i]);
    double length = sqrt32(square_zyx(delta));
    d3 los;
    if (length > 1e-9) { double inv = div(1., length); for (int i = 0; i < 3; ++i) los[i] = mul(delta[i], inv); }
    else {
        double x = f32(quaternion[0]), y = f32(quaternion[1]), z = f32(quaternion[2]), w = f32(quaternion[3]);
        los = {add(add(add(mul(x, x), mul(w, w)), add(mul(x, x), mul(w, w))), -1.),
               add(add(mul(w, z), mul(y, x)), add(mul(w, z), mul(y, x))),
               add(sub(mul(x, z), mul(w, y)), sub(mul(x, z), mul(w, y)))};
    }
    if (use_seeker) {
        out.valid = 1;
        if (range_valid) {
            d3 displacement; for (int i = 0; i < 3; ++i) displacement[i] = mul(direction[i], distance);
            for (int i = 0; i < 3; ++i) out.point[i] = add(origin[i], displacement[i]);
            double vx = f32(velocity[0]), vy = f32(velocity[1]), vz = f32(velocity[2]);
            double dx = f32(direction[0]), dy = f32(direction[1]), dz = f32(direction[2]);
            double wx = f32(angular_rate[0]), wy = f32(angular_rate[1]), wz = f32(angular_rate[2]);
            double rx = displacement[0], ry = displacement[1], rz = displacement[2];
            double residual = sub(radial_speed, add(mul(dz, vz), add(mul(vy, dy), mul(vx, dx))));
            out.velocity = {add(mul(residual, dx), add(sub(mul(wz, ry), mul(wy, rz)), vx)),
                            add(mul(dy, residual), add(sub(mul(wx, rz), mul(wz, rx)), vy)),
                            add(sub(add(mul(wy, rx), vz), mul(ry, wx)), mul(dz, residual))};
        } else {
            double height = old_point[1];   // designation_valid is False on this path
            double scale = safe_div(sub(height, origin[1]), direction[1]);
            out.point = {add(mul(direction[0], scale), origin[0]), height, add(mul(direction[2], scale), origin[2])};
        }
        out.offset = {0., 0., 0.};
        return result;
    }
    if (record) {
        out.valid = 1;
        d3 target_velocity; for (int i = 0; i < 3; ++i) target_velocity[i] = add(record->relative_velocity[i], view.velocity[i]);
        if ((record->point_valid & 255) == 1) {
            double limit = f32(lag_limit);
            double lag = mn(mx(sub(time, record->time), -limit), limit);
            for (int i = 0; i < 3; ++i) out.point[i] = add(mul(target_velocity[i], lag), record->point[i]);
        } else out.point = f32s(fallback);
        out.velocity = target_velocity;
    } else if ((out.valid & 255) != 1) {
        result.angles = {0., 0.}; result.rate = {0., 0., 0.};
        return result;
    }
    if (!p.gnss) {
        const NoiseTables& tables = default_noise_tables();
        double drift_scale = mul(mul(f32(20.293798446655273), dt), p.drift_speed);
        for (int i = 0; i < 3; ++i) offsets[i] = add(offsets[i], mul(noise(mul(add(mul(los[i], 60.), time), .3), tables), drift_scale));
        out.offset = offsets;
    }
    for (int i = 0; i < 3; ++i) out.point[i] = add(mul(out.velocity[i], dt), out.point[i]);
    d3 displacement, rel;
    for (int i = 0; i < 3; ++i) { displacement[i] = sub(add(out.point[i], offsets[i]), origin[i]); rel[i] = sub(out.velocity[i], velocity[i]); }
    result.rate = relative_rate(displacement, rel);
    d3 body = body_direction(quaternion, los);
    result.angles = {atan2n(-body[2], body[0]), atan2n(body[1], sqrt32(add(mul(body[2], body[2]), mul(body[0], body[0]))))};
    return result;
}

// --------------------------------------------------------- radar_illumination
inline Illumination absent_illumination() {
    Illumination r; r.present = false; r.frame = {1., 0., 0., 0., 1., 0., 0., 0., 1., 0., 0., 0.}; r.velocity = {0., 0., 0.}; return r;
}
inline Frame12 direction_frame(const d3& direction, const d3& origin) {
    double x = f32(direction[0]), y = f32(direction[1]), z = f32(direction[2]);
    double horizontal = sqrt32(add(mul(x, x), mul(z, z))), right_x, right_z;
    if (horizontal > 1e-9) { double inverse = div(1., horizontal); right_x = mul(inverse, -z); right_z = mul(inverse, x); }
    else { right_x = 0.; right_z = 1.; }
    return {x, y, z, mul(-y, right_z), sub(mul(x, right_z), mul(z, right_x)), mul(right_x, y), right_x, 0., right_z,
            f32(origin[0]), f32(origin[1]), f32(origin[2])};
}
inline d3 designation_beam(const Packet& body, const Record& record, double lag_limit) {
    double limit = f32(lag_limit);
    double lag = mn(mx(sub(body[13], record.time), -limit), limit);
    d3 delta;
    if ((record.point_valid & 255) == 1) {
        d3 origin = (record.origin_override & 255) != 0 ? record.origin : d3{body[0], body[1], body[2]};
        for (int i = 0; i < 3; ++i) delta[i] = add(sub(record.point[i], origin[i]), mul(record.relative_velocity[i], lag));
    } else for (int i = 0; i < 3; ++i) delta[i] = add(mul(record.relative_velocity[i], lag), record.direction[i]);
    d3 square = {mul(delta[0], delta[0]), mul(delta[1], delta[1]), mul(delta[2], delta[2])};
    double inverse = safe_div(1., sqrt32(add(add(square[2], square[1]), square[0])));
    return {mul(delta[0], inverse), mul(delta[1], inverse), mul(delta[2], inverse)};
}
inline int select_source(const std::vector<Record>& records, int64_t source_tag, bool prediction, int64_t inertial, int64_t datalink, int64_t reconnect) {
    int64_t tag = source_tag & 255;
    if ((inertial & datalink & 255) == 0 || (reconnect & 1)) tag = 0;
    for (size_t index = 0; index < records.size(); ++index) {
        const Record& r = records[index];
        int64_t flag = r.flag48 & 255, priority = r.priority & 255, source = r.source & 255;
        bool eligible = prediction ? (flag != 0 || (source != 255 && (tag ? priority == 1 : priority != 0))) : (tag ? flag == 1 : flag != 0);
        if (eligible && (!tag || (r.source_tag & 255) == tag)) return (int)index;
    }
    return -1;
}

// ------------------------------------------------- missile_support ideal provider
struct IdealProvider { d3 position, velocity; d4 quaternion; };   // pose position, velocity; launch attitude
struct IdealContext { IdealProvider provider; std::vector<Record> records; double lag_limit; };
struct LaunchState { d3 position, velocity; d4 quaternion; };
inline void ideal_context(const LaunchState& launch, const RadarTarget& target, double time, int64_t source_mask, IdealContext& c) {
    for (int i = 0; i < 3; ++i) c.provider.position[i] = f32(launch.position[i] + launch.velocity[i] * time);
    c.provider.velocity = f32s(launch.velocity);
    c.provider.quaternion = launch.quaternion;
    static const int order[10] = {6, 3, 5, 2, 1, 4, 7, 8, 9, 0};
    int source = -1;
    for (int s : order) if (source_mask & (1LL << s)) { source = s; break; }
    c.records.clear();
    if (source >= 0) {
        Record r{};
        r.source = source; r.source_tag = 0; r.target_id = -1; r.time = f32(time);
        r.origin_override = 0; r.point_valid = 1; r.velocity_valid = 1; r.flag48 = 1; r.priority = 1;
        r.origin = c.provider.position; r.point = target.position; r.direction = {0., 0., 0.};
        for (int i = 0; i < 3; ++i) r.relative_velocity[i] = sub(target.velocity[i], c.provider.velocity[i]);
        c.records.push_back(r);
    }
    c.lag_limit = f32(.2);
}
inline Illumination build_illumination(const IdealContext& c, int64_t source_tag, double time, bool prediction,
                                       int64_t inertial, int64_t datalink, int64_t reconnect) {
    int index = select_source(c.records, source_tag, prediction, inertial, datalink, reconnect);
    if (index < 0) return absent_illumination();
    const Record& record = c.records[(size_t)index];
    d3 position = f32s(c.provider.position); d4 quaternion = f32s(c.provider.quaternion); d3 velocity = f32s(c.provider.velocity);
    time = f32(time);
    Packet body = {position[0], position[1], position[2], quaternion[0], quaternion[1], quaternion[2], quaternion[3],
                   velocity[0], velocity[1], velocity[2], 0., 0., 0., time};
    d3 beam = f32s(designation_beam(body, record, c.lag_limit));   // the ideal source_direction returns it unchanged
    d3 origin = (record.origin_override & 255) == 1 ? record.origin : c.provider.position;
    Illumination r; r.present = true; r.frame = direction_frame(beam, origin); r.velocity = f32s(c.provider.velocity);
    return r;
}

// ----------------------------------------------- radar_provider_manager + controls
struct DistanceLimit { int64_t present; double minimum; };
struct RadarManagerProps {
    double lock_timeout, lock_distance, lock_cone_threshold;
    int64_t inertial_navigation, reacquire, use_target_velocity, semi_active, designation_notification;
    std::array<double,3> search_limits; std::vector<d2> search_values;
    int64_t designation_mask, datalink_reconnect, datalink_limit, datalink_counter;
    std::vector<DistanceLimit> distance_limits;
};
struct RadarManagerState {
    double transition_time, strength, distance, search_value, seeker_range, seeker_closure;
    int64_t mode, seeker_tag, count, channel_index;
    int64_t active, engaged, tracking, range_valid, closure_valid, inertial_valid, designation_indicator;
    d2 fins, orientation_auxiliary; d4 factors; d3 direction; d2 seeker_angles; d3 seeker_direction;
    d3 inertial_point, inertial_offset;
    int64_t association_flag, inertial_source_tag; std::array<int64_t,6> inertial_padding; d3 inertial_velocity;
};
struct RadarProps {
    RadarManagerProps manager; ControlProps controls; RadarSeekerProps seeker; InertialProps inertial;
    double los_check_timeout;
};
struct RadarGuidance { RadarManagerState manager; Controllers controllers; RadarSeekerState seeker; };

inline InertialState manager_inertial(const RadarManagerState& s) {
    return {s.inertial_valid, s.inertial_point, s.inertial_velocity, s.inertial_offset, s.inertial_source_tag, s.inertial_padding};
}
inline d3 world_to_body(const d4& quaternion, const d3& vector) {
    double x = f32(quaternion[0]), y = f32(quaternion[1]), z = f32(quaternion[2]), w = f32(quaternion[3]);
    double vx = f32(vector[0]), vy = f32(vector[1]), vz = f32(vector[2]);
    double zz2 = add(z, z), xz = mul(x, zz2), minus_w2 = mul(-2., w), wy = mul(y, minus_w2), yz = mul(zz2, y);
    double wx = mul(x, minus_w2), wz = mul(minus_w2, z), xy = mul(add(x, x), y), ww = add(add(mul(w, w), mul(w, w)), -1.);
    double xx = add(add(mul(x, x), mul(x, x)), ww), yy = add(add(mul(y, y), mul(y, y)), ww), zz = add(add(mul(z, z), mul(z, z)), ww);
    return {add(add(mul(add(wy, xz), vz), mul(sub(xy, wz), vy)), mul(xx, vx)),
            add(add(mul(sub(yz, wx), vz), mul(add(xy, wz), vx)), mul(yy, vy)),
            add(mul(zz, vz), add(mul(add(wx, yz), vy), mul(sub(xz, wy), vx)))};
}
inline void navigation(const d3& direction, const d3& rate, const d3& velocity_in, double distance, double closure, bool use_target_velocity,
                       d3& nav, double& tgo, double& range_squared) {
    d3 velocity = velocity_in;
    if (use_target_velocity) {
        d3 relative = {mul(direction[0], distance), mul(direction[1], distance), mul(direction[2], distance)};
        d3 tangent = {sub(mul(rate[1], relative[2]), mul(rate[2], relative[1])), sub(mul(rate[2], relative[0]), mul(rate[0], relative[2])),
                      sub(mul(rate[0], relative[1]), mul(rate[1], relative[0]))};
        for (int i = 0; i < 3; ++i) velocity[i] = add(mul(direction[i], closure), tangent[i]);
    }
    nav = velocity; tgo = safe_div(distance, closure); range_squared = mul(distance, distance);
}
inline int64_t designation_indicator(const RadarManagerProps& p, int64_t source_tag, const std::vector<Record>& records) {
    const int64_t channel_counts[2] = {0, 0};
    if (p.datalink_counter < 0 || p.datalink_counter > 1) bail("IndexError: datalink counter");
    if (channel_counts[p.datalink_counter] >= p.datalink_limit) return 0;
    for (const Record& record : records) {
        int64_t source = record.source & 31;
        bool eligible = (p.datalink_reconnect & 1) ? (record.flag48 & 255) == 1 : (record.source_tag & 255) == source_tag;
        if ((p.designation_mask & (1LL << source)) && eligible && (record.point_valid & 255) != 0) return 8;
    }
    return 0;
}

// supported_radar_step guidance -> radar_provider_controls.update -> radar_provider_manager.update
inline void radar_guidance_update(const RadarProps& P, RadarGuidance& g, const Packet& old_in, const Packet& new_in, double elapsed_in,
        const float* clocks, const RadarTarget& target, const LaunchState& launch, double context_value, bool matrix_velocity_frame) {
    const RadarManagerProps& p = P.manager;
    static thread_local IdealContext context;   // rebuilt every step; kept only to reuse the record storage
    ideal_context(launch, target, new_in[13], p.designation_mask, context);
    const std::vector<Record>& records = context.records;
    const double lag = context.lag_limit;
    RadarManagerState s = g.manager;
    RadarSeekerState ss = g.seeker;
    Packet old = f32s(old_in), nw = f32s(new_in);
    double elapsed = f32(elapsed_in), dt = sub(nw[13], old[13]);
    s.orientation_auxiliary = {0., 0.}; s.count = 0;
    bool tracking = false;
    auto sync = [&]() {
        s.seeker_angles = ss.angles; s.seeker_direction = ss.direction; s.seeker_range = ss.range; s.seeker_closure = ss.closure;
        s.range_valid = ss.range_valid; s.closure_valid = ss.closure_valid; s.channel_index = ss.channel_index;
    };
    if (s.active) {
        int64_t mode = s.mode;
        if ((mode & 0xffffffffLL) < 2) {
            bool begin = elapsed > p.lock_timeout;
            if (p.inertial_navigation == 1 && s.inertial_valid) {
                d3 d; for (int i = 0; i < 3; ++i) d[i] = sub(add(s.inertial_point[i], s.inertial_offset[i]), nw[i]);
                double distance = add(add(mul(d[2], d[2]), mul(d[1], d[1])), mul(d[0], d[0]));
                d3 terms; for (int i = 0; i < 3; ++i) terms[i] = mul(d[i], ss.direction[i]);
                double projection = add(terms[2], add(terms[1], terms[0]));
                begin = begin && mul(std::fabs(projection), projection) > mul(p.lock_cone_threshold, distance) && distance < mul(p.lock_distance, p.lock_distance);
            }
            if (begin) { s.mode = 2; s.transition_time = nw[13]; }
        } else if (mode == 4 && sub(nw[13], (double)py_trunc(nw[13])) < sub(old[13], (double)py_trunc(old[13]))) {
            s.mode = 2; s.transition_time = nw[13];
        }
        const bool available = true;   // ideal provider
        mode = s.mode;
        static const int64_t seeker_modes[5] = {5, 6, 7, 8, 6};
        int64_t seeker_mode = 0 <= mode && mode <= 4 ? seeker_modes[mode] : 0;
        Illumination illumination;
        if (p.semi_active)
            illumination = build_illumination(context, s.inertial_source_tag, nw[13], false, P.manager.inertial_navigation, P.inertial.datalink, P.inertial.reconnect);
        else {
            illumination.present = true;
            illumination.frame = observation_frame(p4(nw, 3), ss.angles, p3(nw, 0));
            illumination.velocity = p3(nw, 7);
        }
        Designation designation{};   // empty_designation(): zeroed block
        bool have_initial_angles = false; d2 initial_angles{0., 0.};
        bool inertial_branch = available && p.inertial_navigation == 1 && s.inertial_valid == 1;
        if (inertial_branch && s.mode != 3) {
            bool have_source = false; d3 source{};
            if (p.designation_notification == 1) {
                int selected = select_record(P.inertial, 1, s.inertial_source_tag, records, 0, 0, p.designation_mask);
                if (selected >= 0 && !records.empty() && (records[(size_t)selected].origin_override & 255) == 1) {
                    source = f32s(records[(size_t)selected].origin); have_source = true;
                }
            }
            if (!have_source) source = f32s(context.provider.position);
            d3 velocity = p.semi_active ? f32s(context.provider.velocity) : p3(nw, 7);
            Projection projected = project(manager_inertial(s), p3(nw, 0), p3(nw, 7), p.semi_active ? source : p3(nw, 0), velocity, f32(2147440000.));
            initial_angles = {projected.angular_rate[0], projected.angular_rate[1]}; have_initial_angles = true;
            designation.present = projected.valid;
            designation.direction = world_to_body(p4(nw, 3), projected.direction);
            designation.angular_rate = world_to_body(p4(nw, 3), projected.angular_rate);
            designation.range_valid = 1; designation.range = projected.range;
            designation.closure_valid = 1; designation.closure = projected.closure;
            designation.target_id = ss.associated_id;
        }
        // radar_provider_controls.seeker
        RFrame ob{p3(old, 0), p4(old, 3), p3(old, 7), old[13]}, nb{p3(nw, 0), p4(nw, 3), p3(nw, 7), nw[13]};
        Illumination used = illumination.present ? illumination : absent_illumination();
        if (!have_initial_angles) {
            if (seeker_mode == 0 || sub(nw[13], old[13]) < f32(1e-5))
                bail("ValueError: This reset branch requires explicit caller_seeker_angles");
            initial_angles = {0., 0.};
        }
        RadarSeekerResult details = geometric_radar_update(P.seeker, ss, ob, nb, used, &target, elapsed, seeker_mode,
            designation.present != 0, designation, false, P.los_check_timeout, initial_angles, 0);
        ss = details.state;
        sync();
        s.count = details.outputs.extra_flags; tracking = details.outputs.tracking;
        d3 rate = details.outputs.angular_rate; d2 angles = details.outputs.angles;
        if (inertial_branch) {
            if (ss.channel_index < 0 || (size_t)ss.channel_index >= p.distance_limits.size()) bail("IndexError: distance limit channel");
            const DistanceLimit& channel = p.distance_limits[(size_t)ss.channel_index];
            bool skip = tracking && (ss.gap > f32(.001) || (channel.present == 1 && ss.range_valid && mul(channel.minimum, f32(1.1)) > ss.range));
            if (!skip) {
                d3 position = f32s(context.provider.position); d3 velocity = f32s(context.provider.velocity);
                InertialView view{&records, position, velocity};
                InertialResult estimated = inertial_update(P.inertial, manager_inertial(s), view, p3(nw, 0), p4(nw, 3), p3(nw, 7), tracking,
                    ss.direction, ss.angular_rate, ss.range_valid, ss.range, ss.range_rate, 0, 0, p.designation_mask, nw[13], dt,
                    angles, rate, lag);
                s.inertial_valid = estimated.state.valid; s.inertial_point = estimated.state.point;
                s.inertial_velocity = estimated.state.velocity; s.inertial_offset = estimated.state.offset;
                s.inertial_source_tag = estimated.state.source_tag; s.inertial_padding = estimated.state.padding;
                angles = estimated.angles; rate = estimated.rate;
            }
        }
        d3 nav{0., 0., 0.}; double tgo = 0., range_squared = 0.; bool use_range = false;
        if (tracking) { s.mode = 3; use_range = true; }
        else if (s.mode == 0) use_range = true;
        else if (p.reacquire != 1) s.mode = 3;
        else if (s.mode == 3) { s.mode = 2; s.transition_time = nw[13]; }
        else if (s.mode == 2) { if (mn(p.search_limits[0], p.search_limits[1]) >= p.search_limits[2]) { s.mode = 4; use_range = true; } }
        else if (s.mode == 4) use_range = true;
        if (use_range) {
            d3 direction; double distance, closure; bool velocity_valid;
            if (!tracking && (designation.present & 1) && designation.range_valid) {
                direction = rotate(p4(nw, 3), designation.direction);
                distance = designation.range; closure = designation.closure;
                velocity_valid = p.use_target_velocity == 1;
            } else {
                direction = ss.direction; distance = ss.range; closure = ss.closure;
                velocity_valid = p.use_target_velocity == 1 && ss.range_valid && ss.closure_valid;
            }
            navigation(direction, rate, p3(nw, 7), distance, closure, velocity_valid, nav, tgo, range_squared);
        }
        OrientationResult orientation = orientation_service(P.controls, g.controllers.orientation, elapsed, range_squared, p4(nw, 3),
                                                            p3(nw, 10), angles, f32(context_value), dt);
        s.fins = orientation.fins; s.orientation_auxiliary = orientation.auxiliary;
        if (!orientation.override_) {
            if (tracking || s.mode == 0 || s.mode == 4) {
                d3 feedback; for (int i = 0; i < 3; ++i) feedback[i] = mul(sub(nw[7 + i], old[7 + i]), safe_div(1., dt));
                s.fins = guidance_service(P.controls, g.controllers.guidance, clocks, elapsed, tgo, range_squared, p3(nw, 0), p4(nw, 3), p3(nw, 7),
                                          feedback, ss.direction, rate, nav, dt, s.fins, matrix_velocity_frame);
                d3 terms; for (int i = 0; i < 3; ++i) terms[i] = mul(ss.direction[i], nav[i]);
                s.factors = propulsion_service(P.controls, g.controllers.propulsion, elapsed, p3(nw, 0), p3(nw, 7), tgo,
                                               add(terms[2], add(terms[1], terms[0])), dt);
            } else s.fins = {0., 0.};
        }
        s.strength = details.outputs.strength;
    } else { s.strength = 0.; s.fins = {0., 0.}; }
    sync();
    s.engaged = s.mode >= 2; s.tracking = tracking; s.direction = ss.direction;
    s.distance = ss.range;   // observation_range is True for the ideal context
    if (ss.channel_index < 0 || (size_t)ss.channel_index >= p.search_values.size()) bail("IndexError: search value channel");
    const d2& sv = p.search_values[(size_t)ss.channel_index];
    s.search_value = mul(add(sv[1], sv[0]), .5);
    if (p.designation_notification == 1) s.designation_indicator = designation_indicator(p, s.inertial_source_tag, records);
    g.manager = s; g.seeker = ss;
}

// ---------------------------------------------- website optical prelaunch
// missile_initialization.optical_ready after its cached optical_reset: optical_modes.update
// in mode 4 with the authored designation source (seeker_designation.allowed/point)
// and the launcher's measurement callback (geometric_optical_observation.observe).
struct OpticalAcquired { OpticalSeekerState state; Opaque raw; bool tracking; };
inline OpticalAcquired optical_prelaunch_acquire(const OpticalProps& P, const OpticalSeekerState& reset, const Opaque& reset_raw,
                                                 const d4& body_q, const d3& body_position, const d3& target_position) {
    const OpticalSeekerProps& p = P.seeker;
    const double old_time = -(double)(float)(1.0 / 48.0), new_time = 0.;   // -frame_session.STEP, 0
    double dt = sub(new_time, old_time);
    Opaque raw = reset_raw;
    d4 q = f32s(body_q);
    OpticalSeekerState out = reset;
    out.angles = f32s(reset.angles); out.direction = f32s(reset.direction); out.angular_rate = f32s(reset.angular_rate);
    out.widths = f32s(reset.widths);
    out.gap = f32(reset.gap); out.distance = f32(reset.distance); out.los_check_time = f32(reset.los_check_time);
    if (!(dt >= f32(1e-5))) bail("Optical prelaunch interval");
    int source = -1;
    for (int s : {6, 3, 5, 2, 1, 4, 7, 8, 9, 0}) if (p.designation_mask & (1LL << s)) { source = s; break; }
    // seeker_designation.allowed(4, source, designation_mask, constant_mask)
    bool designated = source >= 0 && ((((p.designation_mask & 0xffff) | 0) & 0xffffffffLL) & (1LL << (source & 31))) != 0;
    if (designated) {
        if (p.surface_as_target) bail("NotImplementedError: Designated surface-target generation is not reconstructed here");
        d3 delta = {sub(target_position[0], body_position[0]), sub(target_position[1], body_position[1]), sub(target_position[2], body_position[2])};
        Shared shared = designation_point(p.shared, q, world_to_body(body_q, delta), {0., 0., 0.}, out.angles, true);
        out.widths = {p.width_max, p.width_max};
        out.angles = shared.angles; out.direction = shared.direction; out.angular_rate = shared.angular_rate;
        raw[0] = 0;
        if (shared.accepted == 1) for (int i = 4; i < 16; ++i) raw[i] = 0;
    } else {
        out.angles = p.arret_angles; out.direction = boresight_of(q, p.arret_angles);
        out.angular_rate = {0., 0., 0.}; out.widths = {p.width_max, p.width_max};
    }
    out.los_cache_valid = false;
    // The launcher's callback packs its result into a copy of the reset bytes,
    // which then replace the mode's working copy.
    OpticalObservation observed = geometric_optical_observe(P.observation, out, reset_raw, body_q, body_position, target_position, {0., 0., 0.}, false);
    if (observed.refresh_los_cache) { out.los_check_time = f32(new_time); out.los_cache_valid = true; }
    out.distance = f32(observed.distance); raw = observed.opaque;
    OpticalAcquired result{out, raw, observed.available};
    if (!observed.available) { result.state.gap = add(out.gap, dt); return result; }
    d3 measurement = f32s(observed.measurement);
    Shared shared = reseed(p.shared, q, measurement);
    shared = shared_update(p.shared, q, &measurement, shared, dt, true, true, false);
    out.angles = shared.angles; out.direction = shared.direction; out.angular_rate = shared.angular_rate;
    double inverse_distance = safe_div(1., out.distance);
    double aux[2] = {f32(observed.auxiliary[2]), f32(observed.auxiliary[1])};
    for (int i = 0; i < 2; ++i) out.widths[i] = mn(p.width_max, mx(p.width_min, mul(mul(aux[i], inverse_distance), .5)));
    out.gap = 0.;
    result.state = out;
    return result;
}

}  // namespace typed
