// Typed ports of the geometric radar seeker chain: scene admission, point
// return frame, range/Doppler filters, tracking, designation, mode-7 search,
// radar_modes, radar_point_modes and geometric_radar.update.
#pragma once
#include "typed_seekers.h"

namespace typed {

constexpr double BOUND = 2147440000.0;   // geometric_radar.BOUND == f32(2147440000.)

struct RFrame { d3 position; d4 quaternion; d3 velocity; double time; };
struct Illumination { bool present = false; Frame12 frame{}; d3 velocity{}; };
struct Designation {
    int64_t present = 0; d3 direction{}, angular_rate{};
    int64_t range_valid = 0; double range = 0.; int64_t closure_valid = 0; double closure = 0.; int64_t target_id = 0;
};
struct RadarTarget { int64_t id; d3 position, velocity; d4 quaternion; };   // scene.new_target

struct Axis {
    bool present = false; double minimum = 0., maximum = 0., width = 0., alpha = 1., beta = 1.;
    d2 limit{BOUND, BOUND}; double limit_timeout = BOUND, initial = 0.;
    double unambiguous = 1., signal_width_min = 0.;   // radar_designation axes only
};
struct DesignationProps { SharedProps shared; Axis distance, doppler; bool use_target_id; double doppler_to_distance; };
struct RadarModesProps {
    SharedProps shared; Axis distance, doppler; double prolongation, reset_time;
    d2 arret_angles; bool use_target_id; DesignationProps designation;
    bool use_source_frame; double beam_cosine, scan_half_sector, range_search_half, closure_search_half;
};
struct ReturnProps {   // p['observation']['frame'] fields read by the point specialization
    bool active, force_bistatic; d4 range_parameters;
};
struct RadarSeekerProps { RadarModesProps modes; double query_maximum, query_cosine; ReturnProps frame; };

struct RadarSeekerState {   // radar seeker state (radar_handoff seeker block)
    std::array<int64_t,7> padding{};
    double gap = 0., range = 0., range_rate = 0., closure = 0., closure_rate = 0., target_value = 0., los_check_time = 0., visibility_weight = 0.;
    int64_t rejected = 0, range_valid = 0, closure_valid = 0, los_cache_valid = 0, visibility_clear = 0;
    int64_t associated_id = -1, channel_index = 0, target_id = -1;
    d2 angles{}; d3 direction{}, angular_rate{};
};
inline Shared shared_of(const RadarSeekerState& s) { Shared x; x.angles = s.angles; x.direction = s.direction; x.angular_rate = s.angular_rate; return x; }
inline void set_shared(RadarSeekerState& s, const Shared& x) { s.angles = x.angles; s.direction = x.direction; s.angular_rate = x.angular_rate; }

// geometric measure: the observation result plus its explicit state effects.
struct RadarObservation {
    bool available = false; d3 measurement{0., 0., 0.}; double range = 0., closure = 0., strength = 0.;
    int64_t extra_flags = 0, target_id = -1;
    bool has_effects = false;               // dict(target_id=-1, visibility_clear=True, visibility_weight=1.)
    bool accepted_effects = false;          // + target_value, los_cache_valid, los_check_time
    double target_value = 0., los_check_time = 0.;
};
inline void apply_effects(RadarSeekerState& out, const RadarObservation& o) {
    if (!o.has_effects) return;
    out.target_id = -1; out.visibility_clear = 1; out.visibility_weight = 1.;
    if (o.accepted_effects) { out.target_value = o.target_value; out.los_cache_valid = 1; out.los_check_time = o.los_check_time; }
}
struct RadarOutputs { bool tracking = false; d2 angles{0., 0.}; d3 angular_rate{0., 0., 0.}; double strength = 0.; int64_t extra_flags = 0; };
struct RadarSeekerResult { RadarSeekerState state; RadarOutputs outputs; };

// ---------------------------------------------------- radar_scene_geometry
inline bool admitted(const d3& position, const d3& origin, const d3& axis, double cosine, double minimum, double maximum, double radius) {
    d3 p = f32s(position), o = f32s(origin), a = f32s(axis);
    d3 d = {sub(p[0], o[0]), sub(p[1], o[1]), sub(p[2], o[2])};
    auto norm = [](const d3& v) { return add(add(mul(v[2], v[2]), mul(v[0], v[0])), mul(v[1], v[1])); };
    double distance = norm(d);
    double low = mx(sub(minimum, 50.), 0.);
    double lower = mul(low, low);
    double upper = mul(std::fabs(add(maximum, 50.)), add(maximum, 50.));
    double dot = add(add(mul(d[2], a[2]), mul(d[0], a[0])), mul(d[1], a[1]));
    double threshold = mul(mn(mul(mul(std::fabs(f32(cosine)), cosine), distance),
                              sub(distance, mul(add(radius, 50.), add(radius, 50.)))), norm(a));
    return lower <= distance && distance <= upper && mul(std::fabs(dot), dot) >= threshold;
}

// --------------------------------------------- sensor_point_response (zero size)
using Lobes = std::array<double,13>;
inline double lobe(double squared, const double* values, bool side_lobes) {
    double cosine = values[0], exponent = values[1], side = values[2];
    if (sub(1., squared) > mul(std::fabs(cosine), cosine)) return 1.;
    double value = f32(py_exp(mul(-squared, exponent)));
    if (side_lobes && value < side) bail("Radar side-lobe response is outside the geometric policy");
    return value;
}
struct SensorOutput { bool accepted = false; std::array<double,13> out{}; };
inline SensorOutput sensor_point_response(const Frame12& frame_in, const Lobes& lobes_in, double earth_multiplier, const d3& effective_in,
        double target_height, double signature, double minimum_signal, const d4& range_parameters, bool range_scaling) {
    Frame12 frame = f32s(frame_in); Lobes lobes = f32s(lobes_in); d3 target = f32s(effective_in); d4 rp = f32s(range_parameters);
    double p26 = f32(earth_multiplier), p34 = f32(target_height), p41 = f32(signature), p42 = f32(minimum_signal);
    SensorOutput r;
    d3 local = local_point(frame, target);
    if (local[0] < 0.) return r;
    d3 delta = {sub(target[0], frame[9]), sub(target[1], frame[10]), sub(target[2], frame[11])};
    double squared = add(add(mul(delta[2], delta[2]), mul(delta[1], delta[1])), mul(delta[0], delta[0]));
    if (squared < 100.) return r;
    double earth = mul(p26, 6371000.), earth2 = mul(earth, earth);
    double h1 = add(mx(frame[10], 1.), earth), h2 = add(mx(p34, 1.), earth);
    double a = sub(mul(h1, h1), earth2), b = sub(mul(h2, h2), earth2);
    double root = sqrt32(mul(b, a)), horizon_squared = add(add(b, a), add(root, root));
    if (squared > horizon_squared) return r;
    double distance = sqrt32(squared), inverse = safe_div(1., distance);
    double base = mul(inverse, inverse);
    if (range_scaling && distance <= rp[1]) {
        if (distance > rp[0]) base = rp[3];
        else if (distance < rp[0]) base = mul(base, rp[2]);
        else base = rp[2];
    }
    if (mul(p41, base) < p42) return r;
    d3 unit_local = {mul(local[0], inverse), mul(local[1], inverse), mul(local[2], inverse)};
    double yy = mul(unit_local[1], unit_local[1]), zz = mul(unit_local[2], unit_local[2]);
    double radial = add(mul(mul(lobes[12], lobes[12]), yy), zz);
    double primary = lobe(radial, &lobes[0], false), secondary = lobe(yy, &lobes[6], false);
    double gain = mn(primary, secondary), response = mul(base, gain);
    if (mul(p41, response) < p42) return r;
    r.accepted = true;
    r.out = {mul(delta[0], inverse), mul(delta[1], inverse), mul(delta[2], inverse), distance, inverse,
             unit_local[0], unit_local[1], unit_local[2], 0., 0., 0., response, gain};
    return r;
}

// ------------------------------------------------------ radar_return_frame
struct ReturnValues { bool accepted = false; d3 direction{}; double range = 0., closure = 0.; };   // output[3:6], [11], [19]
inline ReturnValues return_response(const ReturnProps& p, const d3& target_in, const d3& target_velocity, const Frame12& tx, const Frame12& rx,
                                    const d3& transmitter_velocity, const d3& receiver_velocity) {
    // geometric_radar.return_inputs: flat lobes, unit strengths, no range scaling,
    // no offset/extent/multipath/side lobes, availability 1, zero minimum signal.
    static const Lobes flat = {0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1.};
    const double tx_strength = 1., tx_scale = 1., rx_gain = 1., availability = 1., minimum_signal = 0.;
    ReturnValues result;
    d3 target = f32s(target_in);
    d3 effective = target;
    auto sensor = [&](const Frame12& frame, double signature) {
        return sensor_point_response(frame, flat, f32(1.333), effective, target[1], signature, minimum_signal, p.range_parameters, false);
    };
    d3 baseline = {sub(rx[9], tx[9]), sub(rx[10], tx[10]), sub(rx[11], tx[11])};
    double baseline_squared = add(add(mul(baseline[2], baseline[2]), mul(baseline[1], baseline[1])), mul(baseline[0], baseline[0]));
    bool bistatic = p.active && (baseline_squared > 1. || p.force_bistatic);
    std::array<double,13> t, v;
    if (p.active) {
        double signature = mul(mul(mul(tx_scale, availability), tx_strength), rx_gain);
        SensorOutput first = sensor(tx, signature);
        if (!first.accepted) return result;
        t = first.out;
        if (bistatic) {
            SensorOutput second = sensor(rx, mul(signature, t[11]));
            if (!second.accepted) return result;
            v = second.out;
        } else {
            if (mul(mul(t[11], t[11]), signature) < f32(minimum_signal)) return result;
            v = t;
        }
    } else {
        SensorOutput second = sensor(rx, mul(rx_gain, availability));
        if (!second.accepted) return result;
        v = second.out; t = v;
    }
    result.accepted = true;
    result.direction = {v[0], v[1], v[2]};
    result.range = bistatic ? mul(add(sub(v[3], sqrt32(baseline_squared)), t[3]), .5) : v[3];
    d3 velocity = f32s(target_velocity);
    d3 rv = {mul(sub(receiver_velocity[0], velocity[0]), v[0]), mul(sub(receiver_velocity[1], velocity[1]), v[1]), mul(sub(receiver_velocity[2], velocity[2]), v[2])};
    if (bistatic) {
        d3 tv = {mul(sub(transmitter_velocity[0], velocity[0]), t[0]), mul(sub(transmitter_velocity[1], velocity[1]), t[1]), mul(sub(transmitter_velocity[2], velocity[2]), t[2])};
        d3 bv = {mul(sub(transmitter_velocity[0], receiver_velocity[0]), baseline[0]), mul(sub(transmitter_velocity[1], receiver_velocity[1]), baseline[1]),
                 mul(sub(transmitter_velocity[2], receiver_velocity[2]), baseline[2])};
        double base = mul(add(add(bv[2], bv[1]), bv[0]), safe_div(-1., sqrt32(baseline_squared)));
        result.closure = mul(add(add(add(rv[0], add(tv[2], add(tv[1], tv[0]))), base), add(rv[2], rv[1])), .5);
    } else result.closure = add(rv[2], add(rv[1], rv[0]));
    return result;
}

// geometric_radar.measure
inline RadarObservation geometric_measure(const RadarSeekerProps& p, const RadarSeekerState& state, const RFrame& nw,
                                          const Illumination& illumination, const RadarTarget* target) {
    Frame12 frame = observation_frame(nw.quaternion, state.angles, nw.position);
    if (p.frame.active && !illumination.present) bail("ValueError: Geometric radar policy requires an explicit illumination frame");
    RadarObservation out;
    out.measurement = {1., 0., 0.}; out.has_effects = true;
    if (!target) return out;
    d3 axis = {frame[0], frame[1], frame[2]};
    if (!admitted(target->position, nw.position, axis, p.query_cosine, 0., BOUND, 0.)) return out;
    const Frame12& transmitter = p.frame.active ? illumination.frame : frame;
    const d3& transmitter_velocity = p.frame.active ? illumination.velocity : nw.velocity;
    ReturnValues values = return_response(p.frame, target->position, target->velocity, transmitter, frame, transmitter_velocity, nw.velocity);
    if (!values.accepted) return out;
    d4 q = nw.quaternion; q[3] = -q[3];
    out.accepted_effects = true; out.target_value = values.range;
    out.los_check_time = state.los_cache_valid ? state.los_check_time : f32(nw.time);
    out.available = true; out.measurement = rotate(q, values.direction); out.range = values.range; out.closure = values.closure;
    out.strength = 2.; out.extra_flags = 1; out.target_id = target->id;
    return out;
}

// ------------------------------------------------- radar_tracking_filters
constexpr double UNBOUNDED = BOUND;
inline std::pair<double,double> coefficients(const Axis& p, double interval) {
    double n = mn(mul(48., interval), 10.);
    if (n <= 0.) return {1., 1.};
    // A tracking filter sees the same interval step after step; pow is a pure
    // function, so the last results for the same inputs are reused.
    struct Memo { double alpha, beta, n; std::pair<double,double> value; };
    static thread_local Memo memo[2] = {{NAN, NAN, NAN, {}}, {NAN, NAN, NAN, {}}};
    static thread_local int next = 0;
    for (const Memo& m : memo) if (m.alpha == p.alpha && m.beta == p.beta && m.n == n) return m.value;
    std::pair<double,double> value = {sub(1., f32(py_pow(sub(1., p.alpha), n))), sub(1., f32(py_pow(sub(1., p.beta), mul(n, n))))};
    memo[next] = {p.alpha, p.beta, n, value}; next ^= 1;
    return value;
}
inline double clamp_channel(double value, const Axis& p) {
    double half = mul(p.width, .5);
    return mn(sub(p.maximum, half), mx(add(half, p.minimum), value));
}
inline RadarSeekerState rounded_state(const RadarSeekerState& state) {
    RadarSeekerState out = state;
    out.angles = f32s(state.angles); out.direction = f32s(state.direction); out.angular_rate = f32s(state.angular_rate);
    out.gap = f32(state.gap); out.range = f32(state.range); out.range_rate = f32(state.range_rate);
    out.closure = f32(state.closure); out.closure_rate = f32(state.closure_rate);
    return out;
}
struct FilterResult { RadarSeekerState state; d2 angles{}; d3 angular_rate{}; bool accepted = false; };
inline FilterResult filter_update(const RadarModesProps& p, const RadarSeekerState& state, const d4& quaternion, const d3& measurement,
        double range_measurement, double closure_measurement, double dt, double limit_age, bool authored_rate, bool include_gap_after_reject,
        const d2& initial_angles, const d3& initial_rate) {
    dt = f32(dt); limit_age = f32(limit_age);
    if (!finite(dt) || dt <= 0.) bail("ValueError: Positive finite dt required");
    FilterResult result;
    result.state = rounded_state(state);
    RadarSeekerState& out = result.state;
    result.angles = f32s(initial_angles); result.angular_rate = f32s(initial_rate);
    double gap = f32(state.gap);
    bool rewind = !state.rejected || include_gap_after_reject;
    double interval = add(rewind ? gap : 0., dt);
    double values[4] = {0., 0., 0., 0.}; bool have[2] = {false, false};
    for (int k = 0; k < 2; ++k) {
        const Axis& axis = k == 0 ? p.distance : p.doppler;
        double value = f32(k == 0 ? state.range : state.closure), rate = f32(k == 0 ? state.range_rate : state.closure_rate);
        double observed = k == 0 ? range_measurement : closure_measurement;
        if (!axis.present) continue;
        if (rewind) value = sub(value, mul(rate, gap));
        auto ab = coefficients(axis, interval);
        observed = mn(axis.maximum, mx(f32(observed), axis.minimum));
        double predicted = add(value, mul(rate, interval)), residual = sub(observed, predicted);
        double change = mul(mul(ab.second, residual), safe_div(1., interval));
        double next_rate = add(change, rate);
        if (limit_age >= axis.limit_timeout) {
            double limit = add(mul(sub(axis.limit[1], axis.limit[0]), mn(1., gap)), axis.limit[0]);
            bool rejected = k == 0 ? (std::fabs(change) >= mul(limit, interval) || std::fabs(next_rate) >= UNBOUNDED)
                                   : (std::fabs(change) >= mul(UNBOUNDED, interval) || std::fabs(next_rate) >= limit);
            if (rejected) { out.rejected = 1; return result; }
        }
        values[2 * k] = clamp_channel(add(mul(ab.first, residual), predicted), axis);
        values[2 * k + 1] = next_rate; have[k] = true;
    }
    Shared shared = shared_update(p.shared, quaternion, &measurement, shared_of(state), dt, false, authored_rate, false);
    set_shared(out, shared);
    if (shared.accepted != 1) { out.rejected = 1; return result; }
    if (have[0]) { out.range = values[0]; out.range_rate = values[1]; }
    if (have[1]) { out.closure = values[2]; out.closure_rate = values[3]; }
    out.rejected = 0;
    if (p.distance.present) out.range_valid = 1;
    if (p.doppler.present) out.closure_valid = 1;
    result.angles = out.angles; result.angular_rate = out.angular_rate;
    result.accepted = true;
    return result;
}
inline RadarSeekerResult filter_coast(const RadarModesProps& p, const RadarSeekerState& state, const d4& q, double dt, bool authored_rate) {
    RadarSeekerResult r;
    r.state = rounded_state(state); dt = f32(dt);
    double range = add(mul(dt, state.range_rate), state.range);
    if (p.distance.present && state.range_valid) range = clamp_channel(range, p.distance);
    r.state.range = range;
    double closure = add(mul(dt, state.closure_rate), state.closure);
    if (p.doppler.present && state.closure_valid) closure = clamp_channel(closure, p.doppler);
    r.state.closure = closure;
    Shared shared = shared_update(p.shared, q, nullptr, shared_of(state), dt, false, authored_rate, true);
    set_shared(r.state, shared);
    r.outputs.angles = r.state.angles; r.outputs.angular_rate = r.state.angular_rate;
    return r;
}

// Accepted-measurement channel synthesis shared by radar_tracking and radar_search.
inline void absent_channels(const RadarModesProps& p, RadarSeekerState& out, const RFrame& old, const RFrame& nw,
                            double projected_position, double projected_velocity, double dt, bool guarded_division) {
    const d3& d = out.direction;
    if (!p.doppler.present) {
        d3 old_terms = {mul(old.velocity[0], d[0]), mul(old.velocity[1], d[1]), mul(old.velocity[2], d[2])};
        double previous_speed = add(old_terms[2], add(old_terms[1], old_terms[0]));
        double acceleration = 0.;
        if (!guarded_division || dt > f32(4e-19))
            acceleration = div(sub(add(out.closure, out.range_rate), add(projected_velocity, previous_speed)), dt);
        out.closure_rate = acceleration;
        d3 terms = {mul(nw.velocity[0], d[0]), mul(nw.velocity[1], d[1]), mul(nw.velocity[2], d[2])};
        out.closure = add(sub(terms[0], out.range_rate), add(terms[2], terms[1]));
    }
    if (!p.distance.present) {
        out.range = sub(add(projected_position, out.range), mul(dt, out.closure));
        d3 terms = {mul(nw.velocity[0], d[0]), mul(nw.velocity[1], d[1]), mul(nw.velocity[2], d[2])};
        out.range_rate = add(sub(terms[0], out.closure), add(terms[2], terms[1]));
    }
}

// ----------------------------------------------------------- radar_tracking
template<typename Observe>
RadarSeekerResult radar_tracking_update(const RadarModesProps& p, const RadarSeekerState& state, const RFrame& old, const RFrame& nw,
        Observe&& observe, double limit_age, bool suppress, bool authored_rate, bool include_gap_after_reject,
        double los_check_timeout, int64_t initial_extra_flags) {
    double dt = sub(nw.time, old.time);
    if (!finite(dt) || dt < f32(1e-5)) bail("Radar short-step reset is separate");
    RadarSeekerState out = state;
    d4 q = f32s(nw.quaternion);
    out.angles = f32s(out.angles); out.direction = f32s(out.direction); out.angular_rate = f32s(out.angular_rate);
    out.gap = f32(out.gap); out.range = f32(out.range); out.range_rate = f32(out.range_rate);
    out.closure = f32(out.closure); out.closure_rate = f32(out.closure_rate); out.los_check_time = f32(out.los_check_time);
    if (add(los_check_timeout, out.los_check_time) < f32(nw.time)) out.los_cache_valid = 0;
    d3 d = out.direction;
    d3 dp = {mul(sub(nw.position[0], old.position[0]), d[0]), mul(sub(nw.position[1], old.position[1]), d[1]), mul(sub(nw.position[2], old.position[2]), d[2])};
    double projected_position = add(dp[2], add(dp[1], dp[0]));
    out.range = sub(out.range, projected_position);
    d3 dvv = {mul(sub(nw.velocity[0], old.velocity[0]), d[0]), mul(sub(nw.velocity[1], old.velocity[1]), d[1]), mul(sub(nw.velocity[2], old.velocity[2]), d[2])};
    double projected_velocity = add(add(dvv[2], dvv[1]), dvv[0]);
    out.closure = add(out.closure, projected_velocity);
    out.angles = slew(p.shared, pre_slew_body(q, d), out.angles, dt, false, false);
    RadarObservation observed; observed.extra_flags = initial_extra_flags;
    if (!suppress) {
        observed = observe(out);
        apply_effects(out, observed);
        out.channel_index = 0;
    }
    bool have_filter = false;
    FilterResult filtered;
    if (observed.available) {
        if (out.gap > p.reset_time) set_shared(out, reseed(p.shared, q, observed.measurement));
        filtered = filter_update(p, out, q, observed.measurement, observed.range, observed.closure, dt, limit_age,
                                 authored_rate, include_gap_after_reject, {0., 0.}, {0., 0., 0.});
        out = filtered.state; have_filter = true;
    }
    bool accepted = have_filter && filtered.accepted;
    RadarSeekerResult r;
    bool tracking;
    if (accepted) {
        absent_channels(p, out, old, nw, projected_position, projected_velocity, dt, true);
        out.gap = 0.; r.outputs.angles = filtered.angles; r.outputs.angular_rate = filtered.angular_rate; tracking = true;
    } else {
        RadarSeekerResult coasted = filter_coast(p, out, q, dt, authored_rate);
        out = coasted.state; r.outputs = coasted.outputs;
        out.gap = add(out.gap, dt);
        tracking = out.gap < p.prolongation || suppress;
    }
    r.outputs.tracking = tracking; r.outputs.strength = f32(observed.strength); r.outputs.extra_flags = observed.extra_flags;
    r.state = out;
    return r;
}

// ------------------------------------------------- seeker_designation.point
inline Shared designation_point(const SharedProps& p, const d4& quaternion, const d3& direction, const d3& angular_rate,
                                const d2& previous_angles, bool gate_previous) {
    d4 q = f32s(quaternion); d3 dir = f32s(direction);
    double x = dir[0], y = dir[1], z = dir[2];
    d2 angles = {atan2n(-z, x), atan2n(y, sqrt32(add(mul(z, z), mul(x, x))))};
    double limit = p.lock_angle_max;
    d2 tested = gate_previous ? f32s(previous_angles) : angles;
    bool accepted = std::fabs(tested[0]) < limit && std::fabs(tested[1]) < limit;
    for (double& a : angles) a = mn(limit, mx(-limit, a));
    d3 rate = rotate(q, f32s(angular_rate));
    Shared out;
    out.angles = angles; out.direction = boresight_of(q, angles);
    for (int i = 0; i < 3; ++i) out.angular_rate[i] = mn(10., mx(-10., rate[i]));
    out.accepted = accepted ? 1 : 0;
    return out;
}

// --------------------------------------------------- radar_designation.apply
inline std::pair<RadarSeekerState,bool> apply_designation(const DesignationProps& p, const RadarSeekerState& state, const RFrame& frame,
                                                           const Designation& designation, double dt) {
    RadarSeekerState out = rounded_state(state);
    d4 q = f32s(frame.quaternion);
    d3 supplied_direction = f32s(designation.direction);
    Shared shared = designation_point(p.shared, q, supplied_direction, designation.angular_rate, out.angles, false);
    set_shared(out, shared);
    out.channel_index = 0;
    bool accepted = shared.accepted == 1;
    double inverse = safe_div(1., dt);
    auto scalar = [&](const Axis& axis, double measurement, double& key, double& rate_key, int64_t& valid_key) {
        double previous = key; valid_key = 1;
        double value;
        if (axis.present) {
            if (axis.unambiguous == 0.) bail("Zero radar ambiguity period is outside the finite supported domain");
            double wrapped = f32(py_fmod(measurement, axis.unambiguous));
            value = clamp_channel(wrapped, axis);
            accepted = accepted && std::fabs(sub(wrapped, value)) < mul(.5, axis.signal_width_min);
        } else value = measurement;
        key = value; rate_key = mul(sub(value, previous), inverse);
    };
    if (designation.range_valid) {
        double value = f32(designation.range);
        if (p.distance.present) value = add(mul(p.doppler_to_distance, designation.closure), value);
        scalar(p.distance, value, out.range, out.range_rate, out.range_valid);
        d3 direction = rotate(q, supplied_direction);
        d3 terms = {mul(direction[0], frame.velocity[0]), mul(direction[1], frame.velocity[1]), mul(direction[2], frame.velocity[2])};
        out.range_rate = add(add(terms[2], out.range_rate), add(terms[1], terms[0]));
    }
    if (designation.closure_valid) scalar(p.doppler, f32(designation.closure), out.closure, out.closure_rate, out.closure_valid);
    if (p.use_target_id && designation.target_id != -1) out.associated_id = designation.target_id;
    out.target_id = -1;
    return {out, accepted};
}

// ------------------------------------------------------- radar_search (mode 7)
inline double norm_squared(const d3& v) { return add(add(mul(v[2], v[2]), mul(v[1], v[1])), mul(v[0], v[0])); }
struct Candidate { double distance, closure, score; };
// candidates() + order_candidates() for the single enumerated target.
inline bool search_candidate(const RadarModesProps& p, const RadarSeekerState& state, const RFrame& frame, const d3& beam_in,
        const d3& ipos_in, const d3& ivel_in, const d3& iaxis_in, const RadarTarget& target, Candidate& row) {
    d3 position = f32s(frame.position), velocity = f32s(frame.velocity);
    d3 ipos = f32s(ipos_in), ivel = f32s(ivel_in), iaxis = f32s(iaxis_in), beam = f32s(beam_in);
    d3 baseline = {sub(position[0], ipos[0]), sub(position[1], ipos[1]), sub(position[2], ipos[2])};
    double baseline_length = sqrt32(norm_squared(baseline));
    d3 bt = {mul(sub(velocity[0], ivel[0]), baseline[0]), mul(sub(velocity[1], ivel[1]), baseline[1]), mul(sub(velocity[2], ivel[2]), baseline[2])};
    double baseline_rate = mul(add(add(bt[1], bt[2]), bt[0]), safe_div(1., baseline_length));
    d3 delta = {sub(target.position[0], position[0]), sub(target.position[1], position[1]), sub(target.position[2], position[2])};
    double squared = norm_squared(delta);
    double dot = add(mul(delta[2], beam[2]), add(mul(beam[1], delta[1]), mul(delta[0], beam[0])));
    if (mul(std::fabs(dot), dot) < mul(mul(p.beam_cosine, p.beam_cosine), squared)) return false;
    d3 other = {sub(target.position[0], ipos[0]), sub(target.position[1], ipos[1]), sub(target.position[2], ipos[2])};
    double other_squared = norm_squared(other);
    double distance = 0., closure = 0., score = 0.;
    if (p.distance.present) {
        distance = mul(add(sub(sqrt32(other_squared), baseline_length), sqrt32(squared)), .5);
        if (state.range_valid) {
            double residual = std::fabs(sub(distance, state.range));
            if (residual > p.range_search_half) return false;
            double ratio = div(residual, p.range_search_half); score = mul(ratio, ratio);
        }
    }
    if (p.doppler.present) {
        double other_length = sqrt32(other_squared), length = sqrt32(squared);
        d3 u, v;
        if (other_length > 1e-9) { for (int i = 0; i < 3; ++i) u[i] = mul(other[i], div(1., other_length)); } else u = iaxis;
        if (length > 1e-9) { for (int i = 0; i < 3; ++i) v[i] = mul(delta[i], div(1., length)); } else v = beam;
        d3 tv = f32s(target.velocity);
        d3 tx = {mul(sub(ivel[0], tv[0]), u[0]), mul(sub(ivel[1], tv[1]), u[1]), mul(sub(ivel[2], tv[2]), u[2])};
        d3 rx = {mul(sub(velocity[0], tv[0]), v[0]), mul(sub(velocity[1], tv[1]), v[1]), mul(sub(velocity[2], tv[2]), v[2])};
        double first = add(add(tx[0], tx[1]), add(tx[2], baseline_rate));
        double second = add(rx[0], add(rx[1], rx[2]));
        closure = mul(add(second, first), .5);
        if (state.closure_valid) {
            double residual = std::fabs(sub(closure, state.closure));
            if (residual > p.closure_search_half) return false;
            double ratio = div(residual, p.closure_search_half); score = add(score, mul(ratio, ratio));
        }
    }
    // Occupancy maps start empty for the single target: a present channel is
    // never already occupied; an absent channel counts as seen.
    bool seen_all = true;
    for (int i = 0; i < 2; ++i) {
        const Axis& axis = i == 0 ? p.distance : p.doppler;
        double value = i == 0 ? distance : closure;
        if (!axis.present) continue;
        double inverse = std::fabs(mul(axis.width, .5)) > f32(4e-19) ? div(2., axis.width) : 0.;
        (void)py_floor(add(mul(sub(value, axis.minimum), inverse), .5));
        seen_all = false;
    }
    if (seen_all) return false;
    row = {f32(distance), f32(closure), f32(score)};   // radar_candidate_order: finite float32 triples
    if (!finite(row.distance) || !finite(row.closure) || !finite(row.score)) bail("ValueError: Candidate ordering requires finite float32 triples");
    return true;
}
template<typename Observe>
RadarSeekerResult radar_search_update(const RadarModesProps& p, const RadarSeekerState& state, const RFrame& old, const RFrame& nw,
        const RadarTarget* target, const d3& source_position, const d3& source_velocity, const d3& source_direction,
        Observe&& observe, double limit_age, bool suppress, bool authored_rate, bool include_gap_after_reject, int64_t initial_extra_flags) {
    double dt = sub(nw.time, old.time);
    if (!finite(dt) || dt < f32(1e-5)) bail("Radar short-step reset is separate");
    if (!suppress && state.associated_id != -1) bail("Associated-ID scene resolution must be supplied separately");
    RadarSeekerState out = rounded_state(state);
    d4 q = f32s(nw.quaternion);
    RadarObservation observed; observed.extra_flags = initial_extra_flags;
    if (!suppress) {
        out.angles = slew(p.shared, pre_slew_body(q, out.direction), out.angles, dt, false, false);
        d2 saved_angles = out.angles; double saved_range = out.range, saved_closure = out.closure;
        d3 beam = boresight_of(q, out.angles);
        d3 ipos = p.use_source_frame ? source_position : nw.position;
        d3 ivel = p.use_source_frame ? source_velocity : nw.velocity;
        d3 iaxis = p.use_source_frame ? source_direction : beam;
        Candidate row{}; bool found = false;
        if (target && search_candidate(p, out, nw, beam, ipos, ivel, iaxis, *target, row)) {
            out.los_cache_valid = 0;
            if (p.distance.present) out.range = row.distance;
            if (p.doppler.present) out.closure = row.closure;
            observed = observe(out);
            apply_effects(out, observed);
            if (observed.available) {
                if (p.distance.present) out.range_valid = 1;
                if (p.doppler.present) out.closure_valid = 1;
                if (p.use_target_id) out.associated_id = observed.target_id;
                out.direction = beam;
                found = true;
            }
        }
        if (!found) { out.angles = saved_angles; out.range = saved_range; out.closure = saved_closure; }
    }
    d3 d = out.direction;
    d3 dp = {mul(sub(nw.position[0], old.position[0]), d[0]), mul(sub(nw.position[1], old.position[1]), d[1]), mul(sub(nw.position[2], old.position[2]), d[2])};
    double projected_position = add(dp[2], add(dp[1], dp[0]));
    out.range = sub(out.range, projected_position);
    d3 dvv = {mul(sub(nw.velocity[0], old.velocity[0]), d[0]), mul(sub(nw.velocity[1], old.velocity[1]), d[1]), mul(sub(nw.velocity[2], old.velocity[2]), d[2])};
    double projected_velocity = add(add(dvv[2], dvv[1]), dvv[0]);
    out.closure = add(out.closure, projected_velocity);
    bool have_filter = false;
    FilterResult filtered;
    if (observed.available) {
        d3 rate = rotate(q, out.angular_rate);
        set_shared(out, reseed(p.shared, q, observed.measurement));
        for (int i = 0; i < 3; ++i) out.angular_rate[i] = mn(10., mx(-10., rate[i]));
        out.channel_index = 0;
        filtered = filter_update(p, out, q, observed.measurement, observed.range, observed.closure, dt, limit_age,
                                 authored_rate, include_gap_after_reject, {0., 0.}, {0., 0., 0.});
        out = filtered.state; have_filter = true;
    }
    bool accepted = have_filter && filtered.accepted;
    RadarSeekerResult r;
    if (accepted) {
        absent_channels(p, out, old, nw, projected_position, projected_velocity, dt, false);
        out.gap = 0.; r.outputs.angles = filtered.angles; r.outputs.angular_rate = filtered.angular_rate;
    } else {
        RadarSeekerResult coasted = filter_coast(p, out, q, dt, authored_rate);
        out = coasted.state; r.outputs = coasted.outputs;
        out.gap = add(out.gap, dt);
        if (out.gap > p.prolongation) out.associated_id = -1;
    }
    r.outputs.tracking = accepted; r.outputs.strength = f32(observed.strength); r.outputs.extra_flags = observed.extra_flags;
    r.state = out;
    return r;
}

// ---------------------------------------------------------------- radar_modes
template<typename Observe>
RadarSeekerResult radar_modes_update(const RadarModesProps& p, const RadarSeekerState& state, const RFrame& old, const RFrame& nw,
        Observe&& observe, double limit_age, int64_t mode_in, bool designation_present, const Designation& designation,
        bool suppress, bool authored_rate, bool include_gap_after_reject, double los_check_timeout,
        const d2& initial_angles, int64_t initial_extra_flags) {
    double dt = sub(nw.time, old.time);
    int64_t mode = mode_in & 0xffffffffLL;
    if (!finite(dt)) bail("ValueError: Finite radar time interval required");
    RadarSeekerState out = rounded_state(state);
    d4 q = f32s(nw.quaternion);
    out.los_check_time = f32(out.los_check_time); out.target_value = f32(out.target_value); out.visibility_weight = f32(out.visibility_weight);
    RadarSeekerResult result;
    RadarOutputs& outputs = result.outputs;
    outputs.angles = f32s(initial_angles); outputs.extra_flags = initial_extra_flags;
    auto finish = [&]() { result.state = out; return result; };
    auto park = [&]() {
        out.angles = p.arret_angles; out.direction = boresight_of(q, p.arret_angles); out.angular_rate = {0., 0., 0.};
        out.channel_index = 0; out.range_valid = 0; out.range = p.distance.initial; out.range_rate = 0.;
        out.closure_valid = 0; out.closure = p.doppler.initial; out.closure_rate = 0.; out.associated_id = -1;
    };
    auto keep_visibility = [&](const RadarSeekerState& saved) {
        double lct = out.los_check_time; int64_t lcv = out.los_cache_valid, vc = out.visibility_clear; double vw = out.visibility_weight;
        out = saved; out.los_check_time = lct; out.los_cache_valid = lcv; out.visibility_clear = vc; out.visibility_weight = vw;
    };
    if (mode == 0 || dt < f32(1e-5)) { park(); out.gap = 0.; out.target_id = -1; out.target_value = -1.; return finish(); }
    if (!(mode == 1 || mode == 2 || mode == 3 || mode == 4 || mode == 5 || mode == 6 || mode == 8))
        bail("Radar scene-search/unknown mode remains separate");
    bool designated = designation_present && mode != 8;
    if (designated) {
        if (mode == 2) { out.los_cache_valid = 1; out.visibility_clear = 1; out.visibility_weight = 1.; }
        else if (mode == 3) out.los_cache_valid = 0;
        else if (mode == 4) {
            if (add(los_check_timeout, out.los_check_time) < f32(nw.time)) out.los_cache_valid = 0;
            out.angles = slew(p.shared, pre_slew_body(q, out.direction), out.angles, dt, true, authored_rate);
        }
        RadarSeekerState saved = out;
        auto applied = apply_designation(p.designation, out, nw, designation, dt);
        out = applied.first;
        bool designation_accepted = applied.second;
        if (mode == 1) { outputs.tracking = designation_accepted; return finish(); }
        if (mode == 5 || mode == 6) {
            out.gap = add(out.gap, dt);
            if (mode == 6 && out.gap > p.prolongation) out.associated_id = -1;
            return finish();
        }
        RadarObservation observed; observed.extra_flags = initial_extra_flags;
        if (!suppress && (mode == 2 || designation_accepted)) {
            observed = observe(out);
            apply_effects(out, observed);
            out.channel_index = 0;
        }
        outputs.strength = f32(observed.strength); outputs.extra_flags = observed.extra_flags;
        if (mode == 2) { keep_visibility(saved); outputs.tracking = designation_accepted; }
        else if (mode == 3) {
            outputs.tracking = observed.available;
            if (observed.available) { out.gap = 0.; if (p.use_target_id) out.associated_id = observed.target_id; }
            else { out.gap = add(out.gap, dt); if (out.gap >= p.prolongation && out.associated_id != -1) park(); }
        } else {
            out.gap = observed.available ? 0. : add(out.gap, dt);
            outputs.tracking = observed.available || out.gap < p.prolongation || suppress;
        }
        return finish();
    }
    if (mode == 1) { park(); outputs.tracking = true; return finish(); }
    if (mode == 4 || mode == 8) {
        if (mode == 4) {
            if (add(los_check_timeout, out.los_check_time) < f32(nw.time)) out.los_cache_valid = 0;
            out.angles = slew(p.shared, pre_slew_body(q, out.direction), out.angles, dt, true, authored_rate);
        }
        return radar_tracking_update(p, out, old, nw, observe, limit_age, suppress, authored_rate, include_gap_after_reject,
                                     los_check_timeout, initial_extra_flags);
    }
    if (mode == 5 || mode == 6) {
        d3 d = out.direction;
        out.angles = slew(p.shared, pre_slew_body(q, d), out.angles, dt, false, false);
        d3 dp = {mul(sub(old.position[0], nw.position[0]), d[0]), mul(sub(old.position[1], nw.position[1]), d[1]), mul(sub(old.position[2], nw.position[2]), d[2])};
        out.range = add(add(add(dp[2], dp[1]), out.range), dp[0]);
        d3 dvv = {mul(sub(nw.velocity[0], old.velocity[0]), d[0]), mul(sub(nw.velocity[1], old.velocity[1]), d[1]), mul(sub(nw.velocity[2], old.velocity[2]), d[2])};
        out.closure = add(add(add(dvv[2], dvv[1]), out.closure), dvv[0]);
        RadarSeekerResult coasted = filter_coast(p, out, q, dt, authored_rate);
        out = coasted.state;
        outputs.angles = coasted.outputs.angles; outputs.angular_rate = coasted.outputs.angular_rate;
        out.gap = add(out.gap, dt);
        if (mode == 6 && out.gap > p.prolongation) out.associated_id = -1;
        return finish();
    }
    // Modes 2 and 3: mode 2 restores the seeker block after observation.
    RadarSeekerState saved = out;
    if (mode == 2) { out.los_cache_valid = 1; out.visibility_clear = 1; out.visibility_weight = 1.; }
    else out.los_cache_valid = 0;
    park();
    RadarObservation observed; observed.extra_flags = initial_extra_flags;
    if (!suppress) { observed = observe(out); apply_effects(out, observed); }
    outputs.strength = f32(observed.strength); outputs.extra_flags = observed.extra_flags;
    if (mode == 2) { keep_visibility(saved); outputs.tracking = true; return finish(); }
    if (observed.available) {
        set_shared(out, reseed(p.shared, q, observed.measurement));
        FilterResult filtered = filter_update(p, out, q, observed.measurement, observed.range, observed.closure, dt, limit_age,
                                              authored_rate, include_gap_after_reject, initial_angles, {0., 0., 0.});
        out = filtered.state;
        outputs.angles = filtered.angles; outputs.angular_rate = filtered.angular_rate; outputs.tracking = true;
        out.gap = 0.;
        if (p.use_target_id) out.associated_id = observed.target_id;
    } else {
        out.gap = add(out.gap, dt);
        if (out.gap >= p.prolongation && out.associated_id != -1) park();
    }
    return finish();
}

// ------------------------------------------------- radar_point_modes + geometric_radar
inline void check_radar_state(const RadarSeekerState& s) {
    for (double v : {s.range, s.range_rate, s.closure, s.closure_rate, s.gap})
        if (!finite(v) || std::fabs(v) >= BOUND / 8) bail("ValueError: Geometric radar estimate outside supported finite envelope");
    double squared = sum_squares(s.direction);
    if (!(.99 < squared && squared < 1.01)) bail("ValueError: Geometric radar requires a normalized direction history");
}
inline RadarSeekerResult geometric_radar_update(const RadarSeekerProps& p, const RadarSeekerState& state, const RFrame& old, const RFrame& nw,
        const Illumination& illumination, const RadarTarget* target, double limit_age, int64_t mode_in,
        bool designation_present, const Designation& designation, bool suppress, double los_check_timeout,
        const d2& initial_angles, int64_t initial_extra_flags) {
    if (!(mode_in == 5 || mode_in == 6 || mode_in == 7 || mode_in == 8) || sub(nw.time, old.time) < f32(1e-5))
        bail("Geometric radar policy supports positive-step flight modes 5..8");
    if (!(sub(nw.time, old.time) <= 1.)) bail("ValueError: Geometric radar step exceeds supported interval");
    if (!(0. <= limit_age && limit_age < 1e6)) bail("ValueError: Geometric radar flight age outside supported envelope");
    check_radar_state(state);
    // !(|x| < 1e7) is true exactly for |x| >= 1e7 and for non-finite x.
    auto outside = [](const double* x, size_t n) { bool bad = false; for (size_t i = 0; i < n; ++i) bad |= !(std::fabs(x[i]) < 1e7); return bad; };
    for (const RFrame* frame : {&old, &nw}) {
        double s = sum_squares(frame->quaternion);
        if (!(.99 < s && s < 1.01)) bail("ValueError: Near-unit body quaternion required");
        if (outside(frame->position.data(), 3) || outside(frame->velocity.data(), 3)) bail("ValueError: Body state outside geometric radar envelope");
    }
    if (target && (outside(target->position.data(), 3) || outside(target->velocity.data(), 3)))
        bail("ValueError: Target outside geometric radar envelope");
    if (p.modes.shared.gate_rate != BOUND) bail("ValueError: Configure the geometric radar policy first");
    if (!illumination.present && p.frame.active) bail("ValueError: Semi-active geometric flight requires supplied illumination");
    if (outside(illumination.frame.data(), 12) || outside(illumination.velocity.data(), 3))
        bail("ValueError: Illumination outside geometric radar envelope");
    // radar_point_modes.update
    int64_t mode = mode_in & 0xffffffffLL;
    double dt = sub(nw.time, old.time);
    if (state.associated_id != -1 && !(mode == 0 || dt < f32(1e-5) || mode == 1 || mode == 5 || mode == 6))
        bail("Associated-unit lookup remains separate");
    auto measure = [&](const RadarSeekerState& s) { return geometric_measure(p, s, nw, illumination, target); };
    RadarSeekerResult result;
    if (mode == 7 && dt >= f32(1e-5)) {
        const RadarTarget* scene = nullptr;
        if (!suppress) {
            d2 angles = slew(p.modes.shared, pre_slew_body(nw.quaternion, state.direction), state.angles, dt, false, false);
            d3 axis = boresight_of(nw.quaternion, angles);
            double minimum = 0., maximum = p.query_maximum;
            if (state.range_valid && p.modes.distance.present) {
                double half = p.modes.range_search_half;
                minimum = mx(sub(add(state.range, 10.), half), 0.);
                maximum = mn(add(add(state.range, -10.), half), maximum);
            }
            if (target && admitted(target->position, nw.position, axis, p.modes.beam_cosine, minimum, maximum, 0.)) scene = target;
        }
        d3 source_position = {illumination.frame[9], illumination.frame[10], illumination.frame[11]};
        d3 source_direction = {illumination.frame[0], illumination.frame[1], illumination.frame[2]};
        result = radar_search_update(p.modes, state, old, nw, scene, source_position, illumination.velocity, source_direction,
                                     measure, limit_age, suppress, true, true, initial_extra_flags);
    } else {
        result = radar_modes_update(p.modes, state, old, nw, measure, limit_age, mode, designation_present, designation,
                                    suppress, true, true, los_check_timeout, initial_angles, initial_extra_flags);
    }
    check_radar_state(result.state);
    return result;
}

}  // namespace typed
