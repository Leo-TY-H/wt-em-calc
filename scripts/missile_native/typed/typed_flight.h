// Typed ports of the website evaluation loop: optical_point_flight.update with
// the verified body kernel, FlightSession.step, FastFrameSession.advance,
// FastInteractionSession.advance (proximity trace) and missile_envelope.evaluate.
#pragma once
#include "typed_guidance.h"
#include "typed_launch.h"

namespace typed {

constexpr double STEP = (double)(float)(1.0 / 48.0);   // frame_session.STEP

struct ProximityProps {
    double timeOut, radius, minimalAltitude, armDistance, armDistanceToTarget;
    bool detectAirUnits, detectGroundUnits, detectShells;
};
struct FlightProps {
    int family = 0;                     // 0 optical, 1 radar
    wtm_body_props body{}; wtm_rocket rocket{}; wtm_motors motors{};
    int precision = 0; double end_speed = 0.; bool matrix_velocity_frame = true;
    double break_lock = 1.;             // tracking_timeout.threshold(rocket)
    bool has_fuse = false;
    ProximityProps proximity{};
    OpticalProps optical{}; RadarProps radar{};
};
struct TargetMotion {
    d3 position, velocity; d4 quaternion;
    // missile_worker.escape_motion fields (no-escape zone); constant velocity when escape is false.
    bool escape = false;
    double start = 0., end = 0., heading = 0., sine = 0., cosine = 0., rate = 0., radius = 0., speed = 0., roll = 0.;
    d3 turn_point{}, exit_point{}, run{}; d4 run_quaternion{};
};
struct TargetPoint { d3 position, velocity; d4 quaternion; };
// flight_session.linear_point for the constant-velocity target (epoch 0), or
// missile_worker.target_point on an escape path.
inline TargetPoint target_at(const TargetMotion& m, double time) {
    TargetPoint p;
    if (!m.escape) {
        double delta = time - 0.;
        for (int i = 0; i < 3; ++i) { p.position[i] = f32(m.position[i] + m.velocity[i] * delta); p.velocity[i] = f32(m.velocity[i]); }
        p.quaternion = f32s(m.quaternion);
    } else if (time <= m.start) {
        for (int i = 0; i < 3; ++i) { p.position[i] = f32(m.position[i] + m.velocity[i] * time); p.velocity[i] = f32(m.velocity[i]); }
        p.quaternion = f32s(m.quaternion);
    } else if (time < m.end) {
        double heading = m.heading + m.rate * (time - m.start);
        p.position = {f32(m.turn_point[0] + m.radius * (std::sin(heading) - m.sine)), f32(m.turn_point[1]),
                      f32(m.turn_point[2] - m.radius * (std::cos(heading) - m.cosine))};
        p.velocity = {f32(m.speed * std::cos(heading)), f32(0.), f32(m.speed * std::sin(heading))};
        p.quaternion = f32s(angles_quaternion({heading * (180.0 / 3.141592653589793238462643383279502884), 0., m.roll}));   // math.degrees
    } else {
        double delta = time - m.end;
        for (int i = 0; i < 3; ++i) { p.position[i] = f32(m.exit_point[i] + m.run[i] * delta); p.velocity[i] = f32(m.run[i]); }
        p.quaternion = f32s(m.run_quaternion);
    }
    return p;
}
struct Event { d3 position, direction; double time; };
struct Session {
    // FlightSession state
    wtm_body body{}; std::array<double,8> controls{};
    OpticalGuidance optical{}; RadarGuidance radar{};
    // FrameSession
    double launch_time = 0., lifetime = 0., distance_limit = 0.; d3 origin{};
    int64_t iteration_limit = 15; bool expiry_flag = false; double lost_tracking_clock = 0.;
    // InteractionSession
    double proximity_clock = 0.; d3 proximity_origin{}; bool proximity_enabled = false, guidance_flag = false;
    int64_t mode = 1; double delay = 0.; bool has_event = false; Event event{}; d2 offsets{};
};
struct Expiry { bool returned, expiry_flag, time_expired, distance_expired; };

inline Packet packet(const wtm_body& b) {
    return {b.position.x, b.position.y, b.position.z, b.quaternion.x, b.quaternion.y, b.quaternion.z, b.quaternion.w,
            b.velocity.x, b.velocity.y, b.velocity.z, b.omega.x, b.omega.y, b.omega.z, b.time};
}

// state_binary32.validate_flight_numbers: everything finite except PID records.
inline void finite_or_bail(double x) { if (!finite(x)) bail("ValueError: state must be finite"); }
template<size_t N> void finite_or_bail(const std::array<double,N>& v) { for (double x : v) finite_or_bail(x); }
// x * 0 is NaN exactly when x is not finite, so one test covers a whole state
// (strict floating point keeps every product).
struct FiniteSum {
    double sum = 0.;
    void operator()(double x) { sum = sum + x * 0.; }
    template<size_t N> void operator()(const std::array<double,N>& v) { for (double x : v) sum = sum + x * 0.; }
};
inline void validate_flight_numbers(const Session& S, int family) {
    const wtm_body& b = S.body;
    FiniteSum c;
    for (float x : {b.position.x, b.position.y, b.position.z, b.quaternion.x, b.quaternion.y, b.quaternion.z, b.quaternion.w,
                    b.velocity.x, b.velocity.y, b.velocity.z, b.omega.x, b.omega.y, b.omega.z, b.time, b.body_random,
                    b.clocks[0], b.clocks[1], b.clocks[2], b.clocks[3], b.distance, b.water_distance, b.surface_depth, b.immersion})
        c((double)x);
    c(S.controls);
    if (family == 0) {
        const OpticalManagerState& m = S.optical.manager;
        for (double x : {m.transition_time, m.scan_phase, m.strength, m.distance, m.seeker_distance}) c(x);
        c(m.fins); c(m.orientation_auxiliary); c(m.factors); c(m.auxiliary);
        c(m.direction); c(m.seeker_direction); c(m.inertial_point); c(m.inertial_offset);
        const OpticalSeekerState& s = S.optical.seeker;
        c(s.angles); c(s.direction); c(s.angular_rate); c(s.widths);
        for (double x : {s.gap, s.distance, s.los_check_time}) c(x);
    } else {
        const RadarManagerState& m = S.radar.manager;
        for (double x : {m.transition_time, m.strength, m.distance, m.search_value, m.seeker_range, m.seeker_closure}) c(x);
        c(m.fins); c(m.orientation_auxiliary); c(m.factors); c(m.direction);
        c(m.seeker_angles); c(m.seeker_direction); c(m.inertial_point); c(m.inertial_offset);
        c(m.inertial_velocity);
        const RadarSeekerState& s = S.radar.seeker;
        for (double x : {s.gap, s.range, s.range_rate, s.closure, s.closure_rate, s.target_value, s.los_check_time, s.visibility_weight}) c(x);
        c(s.angles); c(s.direction); c(s.angular_rate);
    }
    finite_or_bail(c.sum);
}

// optical_point_flight.update (engine_kernels.flight_result) + family guidance.
inline void flight_update(const FlightProps& P, Session& S, const TargetPoint& new_target, const LaunchState& launch, double dt_in) {
    float dt = (float)dt_in;
    if (!std::isfinite(dt) || dt <= 0.f) bail("ValueError: Positive finite caller interval required");
    if (P.precision <= 0) bail("ValueError: Positive body substep count required");
    Packet old = packet(S.body);
    if (S.body.water || S.body.immersion != 0.f) bail("NotImplementedError: Pure-air composition only");
    float c[8]; for (int i = 0; i < 8; ++i) c[i] = (float)S.controls[i];
    wtm_flight_props fp{&P.body, &P.rocket, &P.motors, P.precision, (float)P.end_speed};
    wtm_body_step_out out{};
    body_advance(&fp, &S.body, c, dt, &out, nullptr);
    check_kernel();
    Packet pre_cap = packet(S.body);
    double elapsed = fsub(out.target_time, 0.0f);   // state carries no launch_time
    double context_value = S.body.surface_depth;
    if (P.family == 0) {
        OpticalTarget target{new_target.position, {0., 0., 0.}};
        optical_guidance_update(P.optical, S.optical, old, pre_cap, elapsed, S.body.clocks, target, context_value, P.matrix_velocity_frame);
        const OpticalManagerState& m = S.optical.manager;
        S.controls = {m.fins[0], m.fins[1], m.orientation_auxiliary[0], m.orientation_auxiliary[1], m.factors[0], m.factors[1], m.factors[2], m.factors[3]};
    } else {
        RadarTarget target{123, new_target.position, new_target.velocity, new_target.quaternion};
        radar_guidance_update(P.radar, S.radar, old, pre_cap, elapsed, S.body.clocks, target, launch, context_value, P.matrix_velocity_frame);
        const RadarManagerState& m = S.radar.manager;
        S.controls = {m.fins[0], m.fins[1], m.orientation_auxiliary[0], m.orientation_auxiliary[1], m.factors[0], m.factors[1], m.factors[2], m.factors[3]};
    }
    S.body.velocity = cap_velocity(S.body.velocity, (float)P.end_speed);
    check_kernel();
}

// FlightSession.step
inline void flight_step(const FlightProps& P, Session& S, const TargetPoint& old_target, const TargetPoint& new_target,
                        const LaunchState& launch, double dt) {
    dt = f32(dt);
    if (!finite(dt) || !(f32(1e-5) <= dt && dt <= 1.)) bail("ValueError: Step must lie between 1e-5 and 1 second");
    finite_or_bail(new_target.position);   // finite(targets)
    if (P.family == 1) {
        finite_or_bail(new_target.velocity); finite_or_bail(new_target.quaternion);
        finite_or_bail(old_target.position); finite_or_bail(old_target.velocity); finite_or_bail(old_target.quaternion);
    }
    // The reference also validates the state on entry. Setup validates the
    // initial state and every step validates its result, and nothing between
    // steps changes the validated fields, so that check always passes here.
    if (add(S.body.time, dt) <= S.body.time) bail("ValueError: Step does not advance the binary32 clock");
    flight_update(P, S, new_target, launch, dt);
    validate_flight_numbers(S, P.family);
}

// flight_limits.evaluate
inline Expiry limits_evaluate(const Session& S, double now, bool initial_flag) {
    d3 position = {S.body.position.x, S.body.position.y, S.body.position.z}, d;
    for (int i = 0; i < 3; ++i) d[i] = sub(S.origin[i], position[i]);
    double distance_squared = add(add(mul(d[2], d[2]), mul(d[1], d[1])), mul(d[0], d[0]));
    double deadline = add(S.launch_time, S.lifetime);
    bool time_expired = deadline < f32(now);
    bool distance_expired = f32(S.distance_limit) > 0. && distance_squared >= mul(S.distance_limit, S.distance_limit);
    bool expired = time_expired || distance_expired;
    return {!expired, initial_flag || expired, time_expired, distance_expired};
}

// FastFrameSession.advance
inline Expiry frame_advance(const FlightProps& P, Session& S, const TargetMotion& motion, const LaunchState& launch, double now) {
    now = f32(now);
    if (!finite(now)) bail("ValueError: Finite caller time required");
    if (S.expiry_flag) bail("Expiry dispatch is outside the evaluation loop");
    double start = S.body.time;
    double lost_clock = S.lost_tracking_clock; bool destroy = false;
    double target = mn(add(start, .5), now);
    int64_t counter = 0;
    while ((double)S.body.time < target) {
        ++counter;
        if (counter >= S.iteration_limit) break;
        double old_time = S.body.time, next_time = add(old_time, STEP);
        TargetPoint old_target = target_at(motion, old_time), new_target = target_at(motion, next_time);
        flight_step(P, S, old_target, new_target, launch, STEP);
        bool engaged, tracking;
        if (P.family == 0) { engaged = S.optical.manager.engaged; tracking = S.optical.manager.tracking; }
        else { engaged = S.radar.manager.engaged != 0; tracking = S.radar.manager.tracking != 0; }
        // tracking_timeout.advance
        double clock = f32(lost_clock), limit = f32(P.break_lock), dt = f32(STEP);
        if (!engaged || tracking) lost_clock = 0.;
        else if (clock > limit) { lost_clock = clock; destroy = true; }
        else lost_clock = add(clock, dt);
    }
    Expiry expiry = limits_evaluate(S, now, S.expiry_flag || destroy);
    S.expiry_flag = expiry.expiry_flag; S.lost_tracking_clock = lost_clock;
    return expiry;
}

inline double norm2(const d3& v) { return add(add(mul(v[2], v[2]), mul(v[1], v[1])), mul(v[0], v[0])); }

// FastInteractionSession.advance with proximity_request.build/prepare,
// trace_events.limit_pending/consume_no_contacts, proximity_motion.sample_segment
// and proximity_geometry.point_candidate (flat surface at height zero).
inline Expiry interaction_advance(const FlightProps& P, Session& S, const TargetMotion& motion, const LaunchState& launch,
                                  double now, double frame_dt, double collision_time) {
    if (S.has_event || S.expiry_flag) bail("Dispatch is outside the evaluation loop");
    now = f32(now); frame_dt = f32(frame_dt); collision_time = f32(collision_time);
    if (!finite(now) || !finite(frame_dt) || !finite(collision_time) || frame_dt < 0.)
        bail("ValueError: Finite caller clocks and nonnegative frame_dt required");
    d3 old_position = {S.body.position.x, S.body.position.y, S.body.position.z}; double old_time = S.body.time;
    Expiry expiry = frame_advance(P, S, motion, launch, now);
    // proximity_request.advance_clock
    double clock = P.has_fuse && expiry.returned ? add(S.proximity_clock, frame_dt) : f32(S.proximity_clock);
    d3 position = {S.body.position.x, S.body.position.y, S.body.position.z};
    d3 velocity_body = {S.body.velocity.x, S.body.velocity.y, S.body.velocity.z};
    double guidance_distance = P.family == 0 ? S.optical.manager.distance : S.radar.manager.distance;
    // proximity_request.build
    d3 start = f32s(old_position), end = f32s(position), origin = f32s(S.proximity_origin);
    for (const d3* v : {&start, &end, &origin}) for (double x : *v) if (!finite(x)) bail("ValueError: Three finite coordinates required");
    if (add(add(mul(start[0], start[0]), mul(start[1], start[1])), mul(start[2], start[2])) > f32(1e11))
        bail("NotImplementedError: Large-position trace-builder branch not recovered");
    d3 delta = {sub(end[0], start[0]), sub(end[1], start[1]), sub(end[2], start[2])};
    double length = sqrt32(norm2(delta));
    bool queued = !(length < f32(1e-5));
    // trace_events.limit_pending checks run before its queued test.
    if (!(S.mode == 1 || S.mode == 2)) bail("NotImplementedError: Only active modes 1 and 2 audited here");
    double delay = f32(S.delay);
    if (!finite(delay)) bail("ValueError: Finite stored delay required");
    bool consumed_any = false; int64_t new_mode = S.mode; bool new_event = false; Event event{};
    if (queued) {
        d3 direction; for (int i = 0; i < 3; ++i) direction[i] = mul(delta[i], div(1., length));
        double elapsed = f32(clock);
        bool properties_bound = S.proximity_enabled && P.has_fuse;
        double d_length = mn(length, 1000.), original_length = mn(length, 1000.);
        double guidance_scalar = f32(guidance_distance);
        double prior_body_time = f32(old_time), body_time = f32(S.body.time);
        bool forced = false;
        d3 v = f32s(velocity_body);
        if (S.mode == 2) {
            for (double x : v) if (!finite(x)) bail("ValueError: Three finite velocity components required");
            double speed = sqrt32(add(add(mul(v[1], v[1]), mul(v[0], v[0])), mul(v[2], v[2])));
            double distance = mx(0., mul(speed, delay));
            if (d_length > distance) { forced = true; d_length = distance; original_length = distance; }
        }
        // proximity_request.prepare
        d3 endpoint; for (int i = 0; i < 3; ++i) endpoint[i] = add(start[i], mul(direction[i], d_length));
        bool queried = properties_bound && elapsed >= P.proximity.timeOut;
        if (queried) {
            d3 e; for (int i = 0; i < 3; ++i) e[i] = sub(endpoint[i], origin[i]);
            queried = norm2(e) >= mul(P.proximity.armDistance, P.proximity.armDistance);
        }
        bool allowed = false;
        if (queried) {
            double height = f32(0.);
            allowed = !(P.proximity.minimalAltitude > 0 && sub(endpoint[1], P.proximity.minimalAltitude) <= height);
            allowed = allowed && !(P.proximity.armDistanceToTarget > 0 && guidance_scalar > 0 && guidance_scalar >= P.proximity.armDistanceToTarget);
        }
        int mask = allowed ? ((P.proximity.detectAirUnits ? 8 : 0) | (P.proximity.detectGroundUnits ? 4 : 0)) : 0;
        double candidate = -1.;
        if (mask & 8) {
            // proximity_motion.sample_segment
            double t0 = f32(prior_body_time), t1 = f32(body_time);
            double lower = f32(add(collision_time, S.offsets[0])), upper = f32(add(collision_time, S.offsets[1]));
            auto clamp = [&](double t) { return mn(mx(t, lower), upper); };
            double first = clamp(t0), last = clamp(t1);
            d3 a = target_at(motion, first).position, b = target_at(motion, last).position;
            d3 dd = {f32(b[0] - a[0]), f32(b[1] - a[1]), f32(b[2] - a[2])};
            d3 seg_start = f32s(start), seg_end_in = f32s(endpoint), seg_end, relative, to_target;
            for (int i = 0; i < 3; ++i) {
                relative[i] = sub(seg_end_in[i], add(seg_start[i], dd[i]));
                to_target[i] = sub(f32(a[i]), seg_start[i]);
                seg_end[i] = sub(seg_end_in[i], dd[i]);
            }
            auto dot = [](const d3& x, const d3& y) { return add(add(mul(x[2], y[2]), mul(x[1], y[1])), mul(x[0], y[0])); };
            double seg_length = sqrt32(dot(relative, relative));
            double reciprocal = div(1., mx(seg_length, f32(.01)));
            double offset = mul(mul(reciprocal, reciprocal), mul(sub(t1, t0), dot(to_target, relative)));
            double attitude_time = clamp(add(mx(offset, 0.), t0));
            d4 q = target_at(motion, attitude_time).quaternion;
            // finite(motion): the rotation from q, the end point and the sampled point.
            for (double x : q) if (!finite(x)) bail("ValueError: point_history must be finite");
            for (double x : a) if (!finite(x)) bail("ValueError: point_history must be finite");
            for (double x : seg_end) if (!finite(x)) bail("ValueError: point_history must be finite");
            for (double x : {first, last, attitude_time}) if (!finite(x)) bail("ValueError: point_history must be finite");
            // proximity_geometry.point_candidate(point=a, start, end=seg_end, origin, elapsed=clock, ...)
            if (!(f32(clock) < f32(P.proximity.timeOut))) {
                d3 so; for (int i = 0; i < 3; ++i) so[i] = sub(start[i], origin[i]);
                if (!(norm2(so) < mul(P.proximity.armDistance, P.proximity.armDistance))) {
                    // closest_point
                    d3 cp = f32s(a), cs = f32s(start), ce = f32s(seg_end);
                    d3 cd = {sub(ce[0], cs[0]), sub(ce[1], cs[1]), sub(ce[2], cs[2])};
                    double length2 = norm2(cd), clen = sqrt32(length2);
                    double inv = clen > f32(4e-19) ? div(1., clen) : 0.;
                    d3 cdir = {mul(cd[0], inv), mul(cd[1], inv), mul(cd[2], inv)};
                    d3 off = {sub(cp[0], cs[0]), sub(cp[1], cs[1]), sub(cp[2], cs[2])};
                    double projection = add(mul(off[2], cdir[2]), add(mul(off[1], cdir[1]), mul(off[0], cdir[0])));
                    d3 closest;
                    if (projection < 0) closest = cs;
                    else if (mul(projection, projection) > length2) closest = ce;
                    else for (int i = 0; i < 3; ++i) closest[i] = add(mul(cdir[i], projection), cs[i]);
                    d3 pc = {sub(cp[0], closest[0]), sub(cp[1], closest[1]), sub(cp[2], closest[2])};
                    bool detected = norm2(pc) <= mul(P.proximity.radius, P.proximity.radius);
                    if (detected) {
                        d3 sc = {sub(start[0], closest[0]), sub(start[1], closest[1]), sub(start[2], closest[2])};
                        double distance = sqrt32(norm2(sc));
                        if (clen > f32(4e-19)) distance = div(mul(distance, d_length), clen);
                        candidate = mx(distance, f32(4e-19));
                    }
                }
            }
        }
        // trace_events.consume_no_contacts
        if (!(S.mode >= 0 && S.mode <= 4)) bail("NotImplementedError: Unsupported projectile mode");
        double c_length = f32(d_length), original = f32(original_length);
        double auxiliary = f32(-1.), proximity = f32(candidate);
        double prior = f32(prior_body_time), body_t = f32(body_time), base = f32(collision_time), initial = f32(-1.);
        double low = f32(S.offsets[0]), high = f32(S.offsets[1]);
        for (double x : {c_length, original, delay, auxiliary, proximity, prior, body_t, base, initial, low, high,
                         start[0], start[1], start[2], direction[0], direction[1], direction[2]})
            if (!finite(x)) bail("ValueError: Finite trace coordinates and scalars required");
        double limit = c_length, delayed = -1.;
        if (S.mode == 2) {
            double speed = sqrt32(add(add(mul(v[2], v[2]), mul(v[0], v[0])), mul(v[1], v[1])));
            delayed = mx(mul(delay, speed), 0.); limit = mn(limit, delayed);
        }
        double distance = forced ? limit : -1.;
        if (delayed > 0. && delayed < limit) distance = delayed;
        if (proximity >= 0. && (distance < 0. || proximity < distance)) distance = proximity;
        bool valid = !(S.mode == 3 || S.mode == 4) && distance >= 0.;
        consumed_any = true;
        new_mode = valid ? 4 : S.mode;
        if (valid) {
            for (int i = 0; i < 3; ++i) event.position[i] = add(start[i], mul(direction[i], mn(limit, distance)));
            event.direction = direction;
            double stamp = initial;
            if (stamp <= 0.) {
                if (prior >= 0. && original > f32(4e-19)) {
                    d3 dl = {sub(start[0], event.position[0]), sub(start[1], event.position[1]), sub(start[2], event.position[2])};
                    double fraction = mn(div(sqrt32(norm2(dl)), original), 1.);
                    stamp = add(prior, mul(sub(body_t, prior), fraction));
                    stamp = mn(mx(add(base, low), stamp), add(base, high));
                } else stamp = base;
            }
            event.time = stamp;
            new_event = true;
        }
    }
    S.proximity_clock = clock;
    if (consumed_any) { S.mode = new_mode; S.has_event = new_event; if (new_event) S.event = event; }
    return expiry;
}

// missile_worker.closest_segment
inline std::pair<double,double> closest_segment(const d3& m0, const d3& m1, const d3& t0, const d3& t1) {
    d3 r, d;
    for (int i = 0; i < 3; ++i) { r[i] = m0[i] - t0[i]; d[i] = (m1[i] - m0[i]) - (t1[i] - t0[i]); }
    double squared = 0.; for (double x : d) squared = squared + x * x;
    double fraction = 0.;
    if (squared != 0.) {
        double dot = 0.; for (int i = 0; i < 3; ++i) dot = dot + r[i] * d[i];
        fraction = mn(1., mx(0., -dot / squared));
    }
    double total = 0.;
    for (int i = 0; i < 3; ++i) total = total + py_pow(r[i] + fraction * d[i], 2.);
    return {py_sqrt(total), fraction};
}

// missile_envelope._intervals / unreachable
struct Interval { double low, high; };
inline void intervals(double a, double b, double c, double low, double high, std::vector<Interval>& out) {
    out.clear();
    double roots[2] = {0., 0.}; int count = 0;
    if (std::fabs(a) > 1e-12) {
        double disc = b * b - 4 * a * c;
        if (disc >= 0) {
            double s = py_sqrt(disc);
            double r0 = (-b - s) / (2 * a), r1 = (-b + s) / (2 * a);
            if (std::isnan(r0) || std::isnan(r1)) bail("Unordered interval roots");
            if (r1 < r0) std::swap(r0, r1);
            roots[0] = r0; roots[1] = r1; count = 2;
        }
    } else if (std::fabs(b) > 1e-12) { roots[0] = -c / b; count = 1; if (std::isnan(roots[0])) bail("Unordered interval roots"); }
    double cuts[4]; int n = 0;
    cuts[n++] = low;
    for (int i = 0; i < count; ++i) if (low < roots[i] && roots[i] < high) cuts[n++] = roots[i];
    cuts[n++] = high;
    for (int i = 0; i + 1 < n; ++i) {
        double x = cuts[i], y = cuts[i + 1], m = (x + y) / 2;
        if (a * m * m + b * m + c <= 0) out.push_back({x, y});
    }
    for (int i = 0; i < count; ++i) if (low <= roots[i] && roots[i] <= high) out.push_back({roots[i], roots[i]});
}
inline bool unreachable(const d3& position, double speed, const d3& target_position, const d3& target_velocity, double remaining,
                        double radius, const d3& origin, double distance_limit) {
    double bound = py_sqrt(speed * speed + 2. * 9.81 * mx(position[1], 0.)) * 1.02 + 5.;
    d3 d; for (int i = 0; i < 3; ++i) d[i] = target_position[i] - position[i];
    double v2 = 0., dvv = 0., dd = 0.;
    for (int i = 0; i < 3; ++i) v2 = v2 + target_velocity[i] * target_velocity[i];
    for (int i = 0; i < 3; ++i) dvv = dvv + d[i] * target_velocity[i];
    for (int i = 0; i < 3; ++i) dd = dd + d[i] * d[i];
    std::vector<Interval> reach, inside;
    intervals(v2 - bound * bound, 2. * (dvv - bound * radius), dd - radius * radius, 0., remaining, reach);
    if (reach.empty()) return true;
    if (distance_limit <= 0) return false;
    d3 e; for (int i = 0; i < 3; ++i) e[i] = target_position[i] - origin[i];
    double limit = distance_limit + radius + bound * .05;
    double ev = 0., ee = 0.;
    for (int i = 0; i < 3; ++i) ev = ev + e[i] * target_velocity[i];
    for (int i = 0; i < 3; ++i) ee = ee + e[i] * e[i];
    intervals(v2, 2. * ev, ee - limit * limit, 0., remaining, inside);
    for (const Interval& r : reach) for (const Interval& s : inside) if (mx(r.low, s.low) <= mn(r.high, s.high)) return false;
    return true;
}

// missile_envelope.evaluate after its setup.
struct EvaluateSetup {
    TargetMotion motion; d3 target_velocity_raw; LaunchState launch;
    bool has_reach = false; double reach_m = 0.; bool early_exit = true;
    int64_t count = 0; double radius = 0., deadline = 0.; std::vector<double> burn_end;
    double best_distance = 0.;
    double straight_from = 0.;   // the target flies straight from then on (target_velocity_raw)
};
struct EvaluateResult {
    bool hit; const char* outcome; double time_s; bool has_event_time; double event_time_s;
    double closest_m, closest_time_s; int64_t steps;
};
// Optional per-step hook for development comparisons (nullptr in production).
using StepHook = void (*)(void*, const Session&, int64_t);
inline EvaluateResult evaluate_loop(const FlightProps& P, Session& S, const EvaluateSetup& E, StepHook hook = nullptr, void* hook_data = nullptr) {
    double best_d = E.best_distance, best_t = 0.;
    d3 prev_point = {S.body.position.x, S.body.position.y, S.body.position.z};
    d3 prev_target = target_at(E.motion, S.body.time).position;
    double prev_time = S.body.time;
    const char* outcome = "time_limit"; double end_time = S.body.time; bool has_event_time = false; double event_time = 0.;
    int64_t i = 0;
    for (i = 0; i < E.count; ++i) {
        double now = add(S.body.time, STEP);
        Expiry expiry = interaction_advance(P, S, E.motion, E.launch, now, STEP, now);
        if (hook) hook(hook_data, S, i);
        bool event = S.has_event;
        d3 point; double at;
        if (event) { point = S.event.position; at = S.event.time; }
        else { point = {S.body.position.x, S.body.position.y, S.body.position.z}; at = S.body.time; }
        d3 target_point = target_at(E.motion, at).position;
        auto seg = closest_segment(prev_point, point, prev_target, target_point);
        if (seg.first < best_d) { best_d = seg.first; best_t = prev_time + seg.second * (at - prev_time); }
        prev_point = point; prev_target = target_point; prev_time = at;
        end_time = at;
        if (E.has_reach && best_d <= E.reach_m) { outcome = "reached"; has_event_time = true; event_time = best_t; break; }
        if (event) { outcome = "proximity_event"; has_event_time = true; event_time = at; break; }
        if (S.body.position.y <= 0) { outcome = "ground"; break; }
        if (target_at(E.motion, S.body.time).position[1] <= 0) { outcome = "target_ground"; break; }
        if (S.expiry_flag) {
            outcome = expiry.time_expired ? "lifetime" : expiry.distance_expired ? "distance_limit" : "tracking_timeout";
            break;
        }
        if (E.early_exit && i % 6 == 5 && (double)S.body.time >= E.straight_from) {
            bool burned = true;   // all() over zip(clocks, burn_end)
            size_t n = E.burn_end.size() < 4 ? E.burn_end.size() : 4;
            for (size_t k = 0; k < n; ++k) if (!((double)S.body.clocks[k] >= E.burn_end[k] + 1e-3)) { burned = false; break; }
            if (burned) {
                d3 velocity = {S.body.velocity.x, S.body.velocity.y, S.body.velocity.z};
                double total = 0.; for (double v : velocity) total = total + v * v;
                double speed = py_sqrt(total);
                d3 position = {S.body.position.x, S.body.position.y, S.body.position.z};
                if (unreachable(position, speed, target_at(E.motion, S.body.time).position, E.target_velocity_raw,
                                E.deadline - (double)S.body.time + 2 * STEP, E.radius, S.origin, S.distance_limit)) {
                    outcome = "unreachable"; break;
                }
            }
        }
    }
    bool hit = std::strcmp(outcome, "proximity_event") == 0 || std::strcmp(outcome, "reached") == 0;
    int64_t last = i < E.count ? i : E.count - 1;   // Python's loop variable after a completed range()
    return {hit, outcome, has_event_time ? event_time : end_time, has_event_time, event_time, best_d, best_t, last + 1};
}

}  // namespace typed
