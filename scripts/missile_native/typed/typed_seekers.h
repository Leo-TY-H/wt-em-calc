// Typed ports of the shared seeker and the optical seeker chain. Each function
// mirrors its scripts/missile_model source line by line; module and function
// names are kept so the two can be read side by side.
#pragma once
#include "typed_core.h"
#include <vector>

namespace typed {

// ------------------------------------------------------------ shared_seeker
struct SharedProps { double angle_max, lock_angle_max, rate_max, alpha, beta, gate_rate; };
// state['angles'/'direction'/'angular_rate'] plus the filter gate result
// (accepted: -1 = None, 0 = False, 1 = True).
struct Shared { d2 angles; d3 direction; d3 angular_rate; int accepted = -1; };

inline d3 cross(const d3& a, const d3& b) {
    return {sub(mul(a[1], b[2]), mul(a[2], b[1])), sub(mul(a[2], b[0]), mul(a[0], b[2])),
            sub(mul(a[0], b[1]), mul(a[1], b[0]))};
}
inline d3 world_residual(const d4& q, const d3& m, const d3& predicted) {
    double x = q[0], y = q[1], z = q[2], w = q[3], mx_ = m[0], my = m[1], mz = m[2];
    double a = mul(add(z, z), x), b = mul(y, add(z, z)), c = mul(add(z, z), w);
    double d = mul(add(w, w), y), e = mul(w, add(x, x)), f = mul(add(x, x), y);
    double ww = sub(add(mul(w, w), mul(w, w)), 1.);
    double xx = add(add(mul(x, x), mul(x, x)), ww), yy = add(add(mul(y, y), mul(y, y)), ww), zz = add(add(mul(z, z), mul(z, z)), ww);
    return {add(sub(add(mul(add(d, a), mz), mul(sub(f, c), my)), predicted[0]), mul(xx, mx_)),
            add(sub(add(mul(sub(b, e), mz), mul(add(c, f), mx_)), predicted[1]), mul(yy, my)),
            add(sub(add(mul(add(e, b), my), mul(sub(a, d), mx_)), predicted[2]), mul(zz, mz))};
}
inline d3 coast_body(const d4& q, const d3& u) {
    double x = q[0], y = q[1], z = q[2], w = q[3], ux = u[0], uy = u[1], uz = u[2];
    double a = mul(x, add(z, z)), b = mul(add(z, z), y), nw = mul(-2., w);
    double d = mul(y, nw), e = mul(x, nw), c = mul(nw, z), f = mul(add(x, x), y);
    double ww = sub(add(mul(w, w), mul(w, w)), 1.);
    double xx = add(add(mul(x, x), mul(x, x)), ww), yy = add(add(mul(y, y), mul(y, y)), ww), zz = add(add(mul(z, z), mul(z, z)), ww);
    return {add(mul(add(d, a), uz), add(mul(sub(f, c), uy), mul(xx, ux))),
            add(mul(sub(b, e), uz), add(mul(yy, uy), mul(add(f, c), ux))),
            add(mul(zz, uz), add(mul(add(e, b), uy), mul(sub(a, d), ux)))};
}
inline d2 slew(const SharedProps& p, const d3& desired, const d2& angles, double dt, bool lock_limit, bool authored_rate) {
    double x = desired[0], y = desired[1], z = desired[2];
    double targets[2] = {atan2n(-z, x), atan2n(y, sqrt32(add(mul(z, z), mul(x, x))))};
    double step = mul(authored_rate ? p.rate_max : 10., dt);
    double limit = lock_limit ? p.lock_angle_max : p.angle_max;
    d2 out;
    for (int i = 0; i < 2; ++i) {
        double previous = angles[i];
        double change = mn(step, mx(-step, sub(targets[i], previous)));
        change = mn(sub(limit, previous), mx(sub(-limit, previous), change));
        out[i] = add(change, previous);
    }
    return out;
}
inline Shared shared_update(const SharedProps& p, const d4& quaternion, const d3* measurement, const Shared& state, double dt,
                            bool lock_limit, bool authored_rate, bool coast) {
    dt = f32(dt);
    if (!finite(dt) || dt <= 0.) bail("Shared seeker reconstruction requires finite positive dt");
    d4 q = f32s(quaternion);
    d2 angles = f32s(state.angles);
    d3 u = f32s(state.direction), omega = f32s(state.angular_rate);
    d3 c = cross(u, omega), raw;
    for (int i = 0; i < 3; ++i) raw[i] = add(mul(c[i], dt), u[i]);
    d3 predicted = normalize(raw, coast ? PHASE_COAST : PHASE_PREDICT);
    d3 desired = coast ? coast_body(q, predicted) : f32s(*measurement);
    d2 new_angles = slew(p, desired, angles, dt, lock_limit, authored_rate);
    Shared out;
    if (coast) { out.angles = new_angles; out.direction = predicted; out.angular_rate = omega; out.accepted = -1; return out; }
    d3 error = cross(world_residual(q, desired, predicted), predicted);
    double gate = mul(p.gate_rate, dt);
    bool accepted = std::fabs(error[0]) <= gate && std::fabs(error[1]) <= gate && std::fabs(error[2]) <= gate;
    d3 direction = predicted;
    if (accepted) {
        double step = mul(authored_rate ? p.rate_max : 10., dt);
        for (double& a : error) a = mn(step, mx(-step, a));
        d3 cp = cross(predicted, error), moved;
        for (int i = 0; i < 3; ++i) moved[i] = add(mul(cp[i], p.alpha), predicted[i]);
        direction = normalize(moved, PHASE_CORRECT);
        double gain = div(p.beta, dt);
        for (int i = 0; i < 3; ++i) omega[i] = mn(10., mx(-10., add(mul(error[i], gain), omega[i])));
    }
    out.angles = new_angles; out.direction = direction; out.angular_rate = omega; out.accepted = accepted ? 1 : 0;
    return out;
}

// optical_tracking.reseed (zero supplied body angular history).
inline Shared reseed(const SharedProps& p, const d4& q, const d3& m) {
    double x = m[0], y = m[1], z = m[2], limit = p.lock_angle_max;
    double a0 = atan2n(-z, x), a1 = atan2n(y, sqrt32(add(mul(z, z), mul(x, x))));
    Shared out;
    out.angles = {mn(limit, mx(-limit, a0)), mn(limit, mx(-limit, a1))};
    out.direction = rotate(q, m);
    out.angular_rate = rotate(q, {0., 0., 0.});
    return out;
}

// --------------------------------------------------------- optical_geometry
using Frame12 = std::array<double,12>;
inline Frame12 observation_frame(const d4& quaternion, const d2& angles, const d3& origin) {
    d3 forward = boresight_of(quaternion, angles);
    double x = forward[0], y = forward[1], z = forward[2];
    double norm = sqrt32(add(mul(x, x), mul(z, z))), rx, rz;
    if (norm > 1e-9) { double inverse = div(1., norm); rx = mul(inverse, -z); rz = mul(inverse, x); }
    else { rx = 0.; rz = 1.; }
    return {x, y, z, mul(-y, rz), sub(mul(x, rz), mul(z, rx)), mul(rx, y), rx, 0., rz, f32(origin[0]), f32(origin[1]), f32(origin[2])};
}
inline d3 local_point(const Frame12& frame, const d3& target_in) {
    double a = f32(frame[0]), d = f32(frame[1]), g = f32(frame[2]), b = f32(frame[3]), e = f32(frame[4]), h = f32(frame[5]);
    double c = f32(frame[6]), f = f32(frame[7]), i = f32(frame[8]), ox = f32(frame[9]), oy = f32(frame[10]), oz = f32(frame[11]);
    double tx = f32(target_in[0]), ty = f32(target_in[1]), tz = f32(target_in[2]);
    double det = sub(add(mul(mul(f, b), g), add(mul(mul(h, d), c), mul(mul(e, a), i))),
                     add(mul(mul(g, e), c), add(mul(mul(f, h), a), mul(mul(b, d), i))));
    if (det == 0.) bail("ValueError: Singular optical observation frame");
    double inverse = div(1., det);
    double rows[3][3] = {{sub(mul(i, e), mul(f, h)), sub(mul(c, h), mul(b, i)), sub(mul(f, b), mul(c, e))},
                         {sub(mul(g, f), mul(d, i)), sub(mul(i, a), mul(g, c)), sub(mul(c, d), mul(f, a))},
                         {sub(mul(h, d), mul(g, e)), sub(mul(g, b), mul(h, a)), sub(mul(e, a), mul(b, d))}};
    double dx = sub(tx, ox);
    d3 output;
    for (int k = 0; k < 3; ++k) {
        double r0 = mul(rows[k][0], inverse), r1 = mul(rows[k][1], inverse), r2 = mul(rows[k][2], inverse);
        output[k] = sub(add(mul(r0, dx), add(mul(r2, tz), mul(r1, ty))), add(mul(r2, oz), mul(r1, oy)));
    }
    return output;
}
struct OpticalGeometry { bool admitted = false; int reason = 0; d3 world_los{}; double distance = 0.; bool core_lobe = false; };
enum { REASON_NONE, REASON_BEHIND, REASON_MINIMUM, REASON_HORIZON };
// optical_geometry.point_response (AAM defaults; only the fields consumers read).
inline OpticalGeometry optical_point_response(const Frame12& frame_in, const d3& target_in, double cosine) {
    Frame12 frame = f32s(frame_in); d3 target = f32s(target_in); cosine = f32(cosine);
    OpticalGeometry out;
    d3 local = local_point(frame, target);
    d3 delta = {sub(target[0], frame[9]), sub(target[1], frame[10]), sub(target[2], frame[11])};
    double squared = add(add(mul(delta[2], delta[2]), mul(delta[1], delta[1])), mul(delta[0], delta[0]));
    if (local[0] < 0.) { out.reason = REASON_BEHIND; return out; }
    if (squared < 100.) { out.reason = REASON_MINIMUM; return out; }
    double earth = mul(1., 6371000.);
    double height = target[1];
    double h1 = add(mx(frame[10], 1.), earth), h2 = add(mx(height, 1.), earth);
    double earth2 = mul(earth, earth);
    double a = sub(mul(h1, h1), earth2), b = sub(mul(h2, h2), earth2);
    double root = sqrt32(mul(b, a));
    double horizon_squared = add(add(b, a), add(root, root));
    if (squared > horizon_squared) { out.reason = REASON_HORIZON; return out; }
    double distance = sqrt32(squared), inverse = safe_div(1., distance);
    d3 unit_local = {mul(local[0], inverse), mul(local[1], inverse), mul(local[2], inverse)};
    double yy = mul(unit_local[1], unit_local[1]), zz = mul(unit_local[2], unit_local[2]);
    double radial = add(yy, zz), cosine_squared = mul(std::fabs(cosine), cosine);
    bool in_primary = sub(1., radial) > cosine_squared;
    bool in_secondary = sub(1., yy) > cosine_squared;
    // The lobe values are unused by the geometric policy. The reference computes
    // exp(-x * 1e6) for x >= 0 there, which cannot overflow or raise, so it is skipped.
    out.admitted = true;
    out.world_los = {mul(delta[0], inverse), mul(delta[1], inverse), mul(delta[2], inverse)};
    out.distance = distance;
    out.core_lobe = in_primary && in_secondary;
    return out;
}

// ------------------------------------------------- optical seeker modes/tracking
struct OpticalSeekerProps {
    SharedProps shared;
    double width_max, width_min, prolongation, reset_time;
    int64_t designation_mask, constant_mask;
    d2 arret_angles;
    double scan_half_sector;
    bool surface_as_target;
    double half_fov;
};
struct OpticalSeekerState {   // seeker['state']
    d2 angles; d3 direction; d3 angular_rate; d2 widths;
    double gap, distance, los_check_time;
    bool los_cache_valid;
};
using Opaque = std::array<int64_t,32>;
// Result of optical_point_controls' observation callback.
struct OpticalObservation {
    bool available = false; d3 measurement{1., 0., 0.};
    double distance = 0., strength = 0.;
    bool flag = false; int64_t count = 0;
    d3 auxiliary{0., 0., 0.};
    Opaque opaque{};
    bool refresh_los_cache = false;
};
struct OpticalOutputs {
    bool tracking = false; d2 angles{0., 0.}; d3 angular_rate{0., 0., 0.};
    double strength = 0.; bool flag = false; int64_t count = 0; d3 auxiliary{0., 0., 0.};
};
struct OpticalModesResult { OpticalSeekerState state; OpticalOutputs outputs; Opaque opaque; };

inline Shared shared_of(const OpticalSeekerState& s) { Shared x; x.angles = s.angles; x.direction = s.direction; x.angular_rate = s.angular_rate; return x; }

// optical_tracking.update. observe receives the temporary seeker state that
// optical_modes.active_observe builds (dict copies are explicit here).
template<typename Observe>
OpticalModesResult optical_tracking_update(const OpticalSeekerProps& p, const OpticalSeekerState& state, const OpticalSeekerState& modes_out,
        Opaque& raw, const d4& quaternion, double old_time, double new_time, Observe&& observe,
        bool authored_rate, bool predict_before_observation, bool suppress, double los_check_timeout, int64_t mode) {
    double dt = sub(new_time, old_time);
    if (mode != 5 && mode != 8) bail("Active optical transition requires mode 5 or 8");
    bool lock_limit = mode == 5;
    if (!finite(dt) || dt < f32(1e-5)) bail("Mode-8 short-step reset must use its separate recovered path");
    d4 q = f32s(quaternion);
    OpticalSeekerState out = state;
    out.widths = f32s(state.widths);
    out.gap = f32(state.gap); out.distance = f32(state.distance); out.los_check_time = f32(state.los_check_time);
    const SharedProps& ps = p.shared;
    Shared shared; shared.angles = f32s(state.angles); shared.direction = f32s(state.direction); shared.angular_rate = f32s(state.angular_rate);
    if (add(los_check_timeout, state.los_check_time) < f32(new_time)) out.los_cache_valid = false;
    shared.angles = slew(ps, pre_slew_body(q, shared.direction), shared.angles, dt, lock_limit, lock_limit ? authored_rate : false);
    Shared temporary = shared;
    if (predict_before_observation) temporary = shared_update(ps, q, nullptr, shared, dt, lock_limit, authored_rate, true);
    OpticalObservation observed;
    observed.available = false; observed.strength = 0.; observed.flag = false; observed.count = 0; observed.auxiliary = {0., 0., 0.};
    bool has_observation = false;
    if (!suppress) {
        // optical_modes.active_observe: the modes-level state with these angles.
        OpticalSeekerState t = modes_out;
        t.angles = temporary.angles; t.direction = temporary.direction; t.angular_rate = temporary.angular_rate;
        observed = observe(t);
        raw = observed.opaque;
        has_observation = true;
    }
    if (has_observation) out.distance = f32(observed.distance);
    if (has_observation && observed.refresh_los_cache) { out.los_check_time = f32(new_time); out.los_cache_valid = true; }
    bool tracking;
    if (observed.available) {
        d3 measurement = f32s(observed.measurement);
        if (f32(state.gap) > p.reset_time) shared = reseed(ps, q, measurement);
        out.gap = 0.;
        shared = shared_update(ps, q, &measurement, shared, dt, lock_limit, authored_rate, false);
        double half_inverse_distance = mul(safe_div(1., out.distance), .5);
        double aux[2] = {observed.auxiliary[2], observed.auxiliary[1]};
        for (int i = 0; i < 2; ++i) out.widths[i] = mn(p.width_max, mx(p.width_min, mul(aux[i], half_inverse_distance)));
        if (lock_limit)
            for (int i = 0; i < 2; ++i) out.widths[i] = mn(p.width_max, mx(p.width_min, mul(mul(aux[i], safe_div(1., out.distance)), .5)));
        tracking = true;
    } else {
        out.gap = add(state.gap, dt);
        shared = shared_update(ps, q, nullptr, shared, dt, lock_limit, authored_rate, true);
        tracking = out.gap < p.prolongation || suppress;
    }
    out.angles = shared.angles; out.direction = shared.direction; out.angular_rate = shared.angular_rate;
    OpticalModesResult result;
    result.state = out;
    result.outputs.tracking = tracking; result.outputs.angles = out.angles; result.outputs.angular_rate = out.angular_rate;
    result.outputs.strength = f32(observed.strength); result.outputs.flag = observed.flag;
    result.outputs.count = observed.count; result.outputs.auxiliary = f32s(observed.auxiliary);
    result.opaque = raw;
    return result;
}

// optical_modes.update for the website's non-designated seeker modes.
template<typename Observe>
OpticalModesResult optical_modes_update(const OpticalSeekerProps& p, const OpticalSeekerState& state, const d4& quaternion,
        double old_time, double new_time, Observe&& observe, int64_t mode_in, const Opaque& opaque_state,
        bool authored_rate, bool predict_before_observation, bool suppress, double los_check_timeout) {
    double dt = sub(new_time, old_time);
    if (!finite(dt)) bail("Finite seeker time interval required");
    int64_t mode = mode_in & 0xffffffffLL;
    Opaque raw = opaque_state;
    d4 q = f32s(quaternion);
    OpticalSeekerState out = state;
    out.angles = f32s(state.angles); out.direction = f32s(state.direction); out.angular_rate = f32s(state.angular_rate);
    out.widths = f32s(state.widths);
    out.gap = f32(state.gap); out.distance = f32(state.distance); out.los_check_time = f32(state.los_check_time);
    OpticalModesResult result;
    auto finish = [&]() { result.state = out; result.opaque = raw; return result; };
    auto park = [&]() {
        out.angles = p.arret_angles; out.direction = boresight_of(q, p.arret_angles);
        out.angular_rate = {0., 0., 0.}; out.widths = {p.width_max, p.width_max};
    };
    if (mode == 0 || dt < f32(1e-5)) {
        park(); out.gap = 0.; out.distance = -1.;
        raw[0] = 0; for (int i = 28; i < 32; ++i) raw[i] = 255;
        return finish();
    }
    if (mode > 8) return finish();
    // No designation is supplied on the website path: designated is False.
    if (mode == 8 || mode == 5) {
        if (add(los_check_timeout, out.los_check_time) < f32(new_time)) out.los_cache_valid = false;
        OpticalModesResult tracked = optical_tracking_update(p, state, out, raw, q, old_time, new_time, observe,
            authored_rate, predict_before_observation, suppress, los_check_timeout, mode);
        tracked.opaque = raw;
        return tracked;
    }
    if (mode != 6 && mode != 7) bail("Optical modes 1..4 are unused on the website path");
    if (mode == 7) out.los_cache_valid = false;
    {
        out.angles = slew(p.shared, pre_slew_body(q, out.direction), out.angles, dt, false, false);
        Shared coast = shared_update(p.shared, q, nullptr, shared_of(out), dt, false, authored_rate, true);
        out.angles = coast.angles; out.direction = coast.direction; out.angular_rate = coast.angular_rate;
    }
    if (mode == 6) return finish();
    if (p.scan_half_sector > p.half_fov) bail("Optical spiral-scan acquisition is not reconstructed here");
    OpticalObservation observed;
    bool has_observation = false;
    if (!suppress) { observed = observe(out); has_observation = true; }
    if (has_observation && observed.refresh_los_cache) { out.los_check_time = f32(new_time); out.los_cache_valid = true; }
    if (has_observation) { out.distance = f32(observed.distance); raw = observed.opaque; }
    OpticalOutputs& outputs = result.outputs;
    outputs.strength = f32(observed.strength); outputs.flag = observed.flag; outputs.count = observed.count;
    outputs.auxiliary = f32s(observed.auxiliary); outputs.tracking = observed.available;
    if (!observed.available) { out.gap = add(out.gap, dt); return finish(); }
    d3 measurement = f32s(observed.measurement);
    Shared shared = reseed(p.shared, q, measurement);
    shared = shared_update(p.shared, q, &measurement, shared, dt, false, authored_rate, false);
    out.angles = shared.angles; out.direction = shared.direction; out.angular_rate = shared.angular_rate;
    double inverse_distance = safe_div(1., out.distance);
    double aux[2] = {observed.auxiliary[2], observed.auxiliary[1]};
    for (int i = 0; i < 2; ++i) out.widths[i] = mn(p.width_max, mx(p.width_min, mul(mul(aux[i], inverse_distance), .5)));
    out.gap = 0.;
    outputs.angles = out.angles; outputs.angular_rate = out.angular_rate;
    return finish();
}

// geometric_optical_observation.observe + optical_point_controls.observation.
struct OpticalObservationProps { double cosine, prolongation; };
inline OpticalObservation geometric_optical_observe(const OpticalObservationProps& p, const OpticalSeekerState& temporary,
        const Opaque& raw, const d4& q, const d3& origin, const d3& target_position, const d3& target_auxiliary, bool collect) {
    // optical_point_controls.observation: the target ID comes from the step's
    // original opaque bytes; the reject time is always reset to zero.
    int32_t target_id;
    {
        uint8_t b[4] = {(uint8_t)raw[28], (uint8_t)raw[29], (uint8_t)raw[30], (uint8_t)raw[31]};
        std::memcpy(&target_id, b, 4);
    }
    bool los_cache_valid = temporary.los_cache_valid;
    double distance = temporary.distance;
    int64_t out_target_id = target_id;
    Frame12 frame = observation_frame(q, temporary.angles, origin);
    OpticalGeometry geometry = optical_point_response(frame, target_position, p.cosine);
    bool accepted = geometry.admitted && geometry.core_lobe;
    OpticalObservation r;
    r.measurement = {1., 0., 0.}; r.auxiliary = {0., 0., 0.};
    if (accepted) {
        r.measurement = normalize(pre_slew_body(f32s(q), geometry.world_los), PHASE_PREDICT);
        r.auxiliary = f32s(target_auxiliary);
        distance = geometry.distance; out_target_id = -1; los_cache_valid = true;
    } else if (f32(temporary.gap) > p.prolongation) {
        distance = -1.; out_target_id = -1;
    }
    r.available = accepted;
    log_observation(accepted, accepted ? LOSS_NONE : !geometry.admitted
        ? (geometry.reason == REASON_BEHIND ? LOSS_BEHIND : geometry.reason == REASON_MINIMUM ? LOSS_MINIMUM_DISTANCE : LOSS_HORIZON)
        : LOSS_FIELD_OF_VIEW);
    r.distance = distance;
    r.strength = accepted ? 2. : 0.;
    r.flag = false;
    r.count = accepted && collect ? 1 : 0;   // reported_target_count
    // changed = raw[:]; reject_time and target_id packed back.
    r.opaque = raw;
    for (int i = 24; i < 28; ++i) r.opaque[i] = 0;
    if (out_target_id < INT32_MIN || out_target_id > INT32_MAX) bail("struct.error: target id outside int32");
    int32_t packed = (int32_t)out_target_id; uint8_t b[4]; std::memcpy(b, &packed, 4);
    for (int i = 0; i < 4; ++i) r.opaque[28 + i] = b[i];
    if (accepted) r.opaque[0] = 0;
    r.refresh_los_cache = !temporary.los_cache_valid && los_cache_valid;
    return r;
}

}  // namespace typed
