// Typed ports of the website radar prelaunch: missile_initialization.radar_ready
// after its cached radar_reset (radar_activation.activate, two
// radar_prelaunch.update steps with launcher_designation.select and
// launcher_inertial.update, radar_designation.reseed) and
// radar_handoff.constructed_guidance. Each function mirrors its
// scripts/missile_model source; anything outside the website path bails.
#pragma once
#include "typed_flight.h"
#include <cfloat>
#include <climits>

namespace typed {

// ------------------------------------------------------- launcher_designation
struct LauncherDesignation {   // select()/convert() output fields read downstream
    int64_t source = 255; d3 origin{}, direction{}, angular_rate{};
    bool range_valid = false; double range = 0., radial_velocity = 0.;
    bool closure_valid = false; double closure = 0.;
    d3 point{}, velocity{}; int64_t source_tag = 0; bool flag55 = false; int64_t target_id = 0;
};
inline LauncherDesignation launcher_convert(const RFrame& frame, const Record& record) {
    double limit = f32(.2);
    double lag = mn(mx(sub(frame.time, record.time), -limit), limit);
    const d3& base = record.origin_override ? record.origin : frame.position;
    d3 launcher_v = f32s(frame.velocity), relative_v = f32s(record.relative_velocity), origin, absolute_v;
    for (int i = 0; i < 3; ++i) { origin[i] = add(mul(launcher_v[i], lag), base[i]); absolute_v[i] = add(relative_v[i], launcher_v[i]); }
    if (!record.point_valid) bail("Direction-only launcher designation records are outside the website path");
    d3 delta, direction;
    for (int i = 0; i < 3; ++i) delta[i] = add(sub(mul(absolute_v[i], lag), origin[i]), record.point[i]);
    d3 square = {mul(delta[0], delta[0]), mul(delta[1], delta[1]), mul(delta[2], delta[2])};
    double distance = mx(sqrt32(add(add(square[2], square[1]), square[0])), 1.);
    double inverse = div(1., distance);
    for (int i = 0; i < 3; ++i) direction[i] = mul(delta[i], inverse);
    double radial_velocity = add(mul(relative_v[2], direction[2]), add(mul(relative_v[1], direction[1]), mul(relative_v[0], direction[0])));
    d3 rate = {sub(mul(relative_v[1], direction[2]), mul(relative_v[2], direction[1])),
               sub(mul(relative_v[2], direction[0]), mul(relative_v[0], direction[2])),
               sub(mul(relative_v[0], direction[1]), mul(relative_v[1], direction[0]))};
    for (double& a : rate) a = mul(a, inverse);
    double closure = record.velocity_valid
        ? -add(mul(relative_v[0], direction[0]), add(mul(relative_v[2], direction[2]), mul(relative_v[1], direction[1]))) : 0.;
    LauncherDesignation out;
    out.source = record.source & 255; out.origin = origin; out.direction = direction; out.angular_rate = rate;
    out.range_valid = true; out.range = distance; out.radial_velocity = radial_velocity;
    out.closure_valid = record.velocity_valid != 0; out.closure = closure;
    for (int i = 0; i < 3; ++i) out.point[i] = add(mul(direction[i], distance), origin[i]);
    out.velocity = absolute_v; out.source_tag = record.source_tag & 255; out.flag55 = record.flag48 != 0; out.target_id = record.target_id;
    return out;
}
inline LauncherDesignation launcher_select(const RFrame& frame, const Record* record, int64_t primary_mask, int64_t fallback_mask) {
    if (record)
        for (int primary = 1; primary >= 0; --primary) {
            int64_t mask = primary ? primary_mask : fallback_mask;
            if ((!primary || record->priority) && (((mask & 0xffffffffLL) >> (record->source & 31)) & 1)) return launcher_convert(frame, *record);
        }
    LauncherDesignation out;
    double x = f32(frame.quaternion[0]), y = f32(frame.quaternion[1]), z = f32(frame.quaternion[2]), w = f32(frame.quaternion[3]);
    d3 direction = {sub(add(add(mul(w, w), mul(x, x)), add(mul(w, w), mul(x, x))), 1.),
                    add(add(mul(y, x), mul(z, w)), add(mul(y, x), mul(z, w))),
                    add(sub(mul(x, z), mul(w, y)), sub(mul(x, z), mul(w, y)))};
    d3 squares = {mul(direction[0], direction[0]), mul(direction[1], direction[1]), mul(direction[2], direction[2])};
    double inverse = safe_div(1., sqrt32(add(add(squares[2], squares[1]), squares[0])));
    out.source = 255; out.origin = f32s(frame.position);
    for (int i = 0; i < 3; ++i) out.direction[i] = mul(direction[i], inverse);
    out.target_id = -1;
    return out;
}

// ------------------------------------------------------- website radar launcher
// missile_initialization.radar_reset's prepared launcher properties.
struct RadarLauncherProps {
    bool permanent = false, lock_after_launch = false; int64_t primary_mask = 0, fallback_mask = 0;
    DesignationProps seeker{};
    double warm_up = 0., work_time = 0.; bool uncage = false, inertial = false, designation_required = false;
    RadarModesProps modes{};
    double los_check_timeout = 0.; bool authored_rate = true, include_gap_after_reject = true;
};
struct LauncherState {
    int64_t phase = 0; double activation_time = 0.; bool association_flag = false;
    RadarSeekerState seeker{}; InertialState inertial{};
};

// radar_designation.reseed
inline RadarSeekerState designation_reseed(const DesignationProps& p, const RadarSeekerState& state, const RFrame& frame, const Designation& d) {
    RadarSeekerState out = rounded_state(state);
    Shared shared = designation_point(p.shared, frame.quaternion, d.direction, d.angular_rate, out.angles, true);
    set_shared(out, shared);
    out.channel_index = 0;
    for (int k = 0; k < 2; ++k) {
        const Axis& axis = k == 0 ? p.distance : p.doppler;
        int64_t valid = k == 0 ? d.range_valid : d.closure_valid;
        if (!valid || !axis.present) continue;
        double value = f32(k == 0 ? d.range : d.closure);
        if (k == 0) value = add(mul(p.doppler_to_distance, d.closure), value);
        if (axis.unambiguous == 0.) bail("NotImplementedError: Zero radar ambiguity period is outside the finite supported domain");
        double clamped = clamp_channel(f32(py_fmod(value, axis.unambiguous)), axis);
        if (k == 0) { out.range_valid = 1; out.range = clamped; out.range_rate = 0.; }
        else { out.closure_valid = 1; out.closure = clamped; out.closure_rate = 0.; }
    }
    if (p.use_target_id && d.target_id != -1) out.associated_id = d.target_id;
    return out;
}

// launcher_inertial.update
inline InertialState launcher_inertial_update(const InertialState& state, const d3& origin_in, const d3& velocity_in, const d3& direction_in,
        const d3& rate_in, bool use_seeker, bool range_valid, double distance_in, double radial_in, bool designation_valid,
        const d3& point_in, const d3& designated_in, int64_t source_tag) {
    InertialState out = state;
    d3 o = f32s(origin_in), v = f32s(velocity_in), d = f32s(direction_in), w = f32s(rate_in), point = f32s(point_in), dv = f32s(designated_in);
    double distance = f32(distance_in), radial = f32(radial_in);
    if (use_seeker) {
        out.valid = 1;
        if (range_valid) {
            for (int i = 0; i < 3; ++i) out.point[i] = add(mul(d[i], distance), o[i]);
            d3 cross = {sub(mul(w[2], d[1]), mul(w[1], d[2])), sub(mul(w[0], d[2]), mul(w[2], d[0])), sub(mul(w[1], d[0]), mul(w[0], d[1]))};
            for (double& a : cross) a = mul(a, distance);
            out.velocity = {add(add(mul(d[0], radial), cross[0]), v[0]), add(add(mul(d[1], radial), v[1]), cross[1]),
                            add(add(mul(d[2], radial), cross[2]), v[2])};
        } else {
            double ratio = safe_div(sub(point[1], o[1]), d[1]);
            out.point = {add(mul(d[0], ratio), o[0]), point[1], add(mul(d[2], ratio), o[2])};
            double dx = sub(out.point[0], o[0]), dy = sub(out.point[1], o[1]), dz = sub(out.point[2], o[2]);
            double projected = add(mul(d[2], v[2]), add(mul(d[1], v[1]), mul(d[0], v[0])));
            out.velocity = {sub(add(mul(w[2], dy), v[0]), add(mul(projected, d[0]), mul(dz, w[1]))),
                            sub(add(mul(w[0], dz), v[1]), add(mul(d[1], projected), mul(dx, w[2]))),
                            sub(add(mul(dx, w[1]), v[2]), add(mul(d[2], projected), mul(dy, w[0])))};
        }
    } else if (designation_valid) { out.valid = 1; out.point = point; out.velocity = dv; }
    else out.valid = 0;
    out.offset = {0., 0., 0.}; out.source_tag = source_tag & 255;
    return out;
}

inline RFrame body_frame(const Packet& body) {
    return {{f32(body[0]), f32(body[1]), f32(body[2])}, {f32(body[3]), f32(body[4]), f32(body[5]), f32(body[6])},
            {f32(body[7]), f32(body[8]), f32(body[9])}, f32(body[13])};
}
// missile_initialization's launcher record at time t: the constructed body moving
// at constant velocity and attitude.
inline Packet launcher_packet(const d3& body_position, const d4& body_q, const d3& body_velocity, double t) {
    Packet b{f32(body_position[0] + body_velocity[0] * t), f32(body_position[1] + body_velocity[1] * t),
             f32(body_position[2] + body_velocity[2] * t), body_q[0], body_q[1], body_q[2], body_q[3],
             body_velocity[0], body_velocity[1], body_velocity[2], 0., 0., 0., f32(t)};
    for (double x : b) if (!finite(x)) bail("ValueError: Finite 14-float launcher records required");
    return b;
}
// missile_initialization.radar_reset's seeker reset: radar_modes in mode 0 over
// the interval before the first prelaunch update.
inline RadarSeekerState radar_reset_state(const RadarLauncherProps& L, const RadarSeekerState& template_seeker,
                                          const d3& body_position, const d4& body_q, const d3& body_velocity) {
    auto none = [](const RadarSeekerState&) -> RadarObservation { bail("Radar reset observes nothing"); };
    RFrame old_frame = body_frame(launcher_packet(body_position, body_q, body_velocity, -3 * STEP));
    RFrame new_frame = body_frame(launcher_packet(body_position, body_q, body_velocity, -2 * STEP));
    return radar_modes_update(L.modes, template_seeker, old_frame, new_frame, none, 0., 0, false, Designation{}, false, true, true,
                              -1., {0., 0.}, 0).state;
}

// radar_prelaunch.update for one interval with the website's ideal seeker callback
// (missile_initialization.radar_ready.seeker): geometric_radar.measure against the
// constant-velocity target, identity-frame illumination at the launcher.
inline void radar_prelaunch_step(const RadarLauncherProps& L, const RadarSeekerProps& flight_seeker, LauncherState& out,
        const Packet& old, const Packet& nw, const Record* record, const TargetMotion& target, const d3& body_velocity) {
    if (out.seeker.channel_index != 0) bail("NotImplementedError: Only the recovered single authored channel is supported");
    LauncherDesignation d = launcher_select(body_frame(nw), record, L.primary_mask, L.fallback_mask);
    out.association_flag = false;
    if (d.target_id != -1) bail("NotImplementedError: Associated-unit eligibility service required");
    if (L.designation_required) {
        if (d.source == 255) { out.phase = 0; return; }
        if (L.permanent && out.phase == 0) out.phase = 0x30 + 0x20 * (int64_t)L.lock_after_launch;
    }
    int64_t phase = out.phase;
    if (L.inertial) {
        const RadarSeekerState& s = out.seeker;
        out.inertial = launcher_inertial_update(out.inertial, {nw[0], nw[1], nw[2]}, {nw[7], nw[8], nw[9]}, s.direction, s.angular_rate,
            phase == 0x40 && L.uncage, s.range_valid != 0, s.range, s.range_rate, d.source != 255 && d.range_valid, d.point, d.velocity, d.source_tag);
    }
    double age = sub(nw[13], out.activation_time);
    if (phase == 0x20 && age > L.warm_up) { phase = 0x30 + 0x20 * (int64_t)L.lock_after_launch; out.activation_time = f32(nw[13]); }
    else if (phase >= 0x50 && age > L.work_time) phase = L.permanent ? 0x30 + 0x20 * (int64_t)L.lock_after_launch : 0;
    out.phase = phase;
    int64_t mode = phase == 0x30 ? 3 : phase == 0x40 ? 3 + (int64_t)L.uncage : phase == 0x50 ? 1 : 0;
    // adapter_inputs with zero prediction offsets
    auto shifted = [](const Packet& b) {
        return RFrame{{add(0., b[0]), add(0., b[1]), add(0., b[2])}, {f32(b[3]), f32(b[4]), f32(b[5]), f32(b[6])},
                      {add(0., b[7]), add(0., b[8]), add(0., b[9])}, f32(b[13])};
    };
    RFrame old_frame = shifted(old), new_frame = shifted(nw);
    d4 nq = {nw[3], nw[4], nw[5], nw[6]};
    Designation designation{};
    designation.present = d.source != 255;
    designation.direction = world_to_body(nq, d.direction); designation.angular_rate = world_to_body(nq, d.angular_rate);
    designation.range_valid = d.range_valid; designation.range = add(d.range, 0.);
    designation.closure_valid = d.closure_valid || d.range_valid;
    designation.closure = add(d.closure_valid ? d.closure : -d.radial_velocity, 0.);
    designation.target_id = d.target_id;
    TargetPoint point = target_at(target, new_frame.time);
    RadarTarget scene{123, point.position, point.velocity, target.quaternion};
    Illumination light; light.present = true;
    light.frame = {1., 0., 0., 0., 1., 0., 0., 0., 1., new_frame.position[0], new_frame.position[1], new_frame.position[2]};
    light.velocity = body_velocity;
    auto measure = [&](const RadarSeekerState& s) { return geometric_measure(flight_seeker, s, new_frame, light, &scene); };
    RadarSeekerResult answer = radar_modes_update(L.modes, out.seeker, old_frame, new_frame, measure, out.activation_time, mode,
        designation.present != 0, designation, false, L.authored_rate, L.include_gap_after_reject, L.los_check_timeout, {0., 0.}, 0);
    out.seeker = answer.state;
    if (phase >= 0x30 && phase != 0x50) out.phase = 0x30 + 0x10 * (int64_t)answer.outputs.tracking;
}

// radar_ready from the cached reset through radar_handoff.constructed_guidance; the
// result replaces the sample-dependent fields of a launched guidance state.
struct RadarPrelaunch { RadarSeekerState seeker; InertialState inertial; bool association_flag; };
inline RadarPrelaunch radar_prelaunch(const RadarLauncherProps& L, const RadarSeekerProps& flight_seeker, const RadarSeekerState& seeker_reset,
        const d3& body_position, const d4& body_q, const d3& body_velocity, const TargetMotion& target) {
    const double step = STEP;
    auto packet = [&](double t) { return launcher_packet(body_position, body_q, body_velocity, t); };
    RFrame start = body_frame(packet(-2 * step));
    int source = -1;
    for (int s : {6, 3, 5, 2, 1, 4, 7, 8, 9, 0}) if (L.primary_mask & (1LL << s)) { source = s; break; }
    auto record_at = [&](double t, Record& r) {
        r = Record{};
        r.source = source; r.source_tag = 0; r.target_id = -1; r.time = f32(t);
        r.origin_override = 0; r.point_valid = 1; r.velocity_valid = 1; r.flag48 = 1; r.priority = 1;
        r.origin = {0., 0., 0.}; r.direction = {0., 0., 0.};
        for (int i = 0; i < 3; ++i) { r.point[i] = f32(target.position[i] + target.velocity[i] * t); r.relative_velocity[i] = sub(target.velocity[i], body_velocity[i]); }
    };
    Record record;
    LauncherState st;
    st.phase = 0; st.activation_time = -(double)FLT_MAX; st.association_flag = false; st.seeker = seeker_reset; st.inertial = InertialState{};
    // radar_activation.activate
    if (L.permanent) st.phase = 0x30 + 0x20 * (int64_t)L.lock_after_launch;
    else {
        if (source >= 0) record_at(-2 * step, record);
        LauncherDesignation sel = launcher_select(start, source >= 0 ? &record : nullptr, L.primary_mask, L.fallback_mask);
        Designation d{};
        d.present = 1; d.direction = world_to_body(start.quaternion, sel.direction); d.angular_rate = world_to_body(start.quaternion, sel.angular_rate);
        d.range_valid = sel.range_valid; d.range = sel.range; d.closure_valid = sel.closure_valid || sel.range_valid;
        d.closure = sel.closure_valid ? sel.closure : -sel.radial_velocity; d.target_id = 0;
        st.seeker = designation_reseed(L.seeker, st.seeker, start, d);
        st.seeker.los_cache_valid = 0;
        st.phase = 0x20; st.activation_time = f32(start.time);
    }
    st.phase = L.lock_after_launch ? 0x50 : 0x30; st.activation_time = f32(-2 * step);
    const double intervals[2][2] = {{-2 * step, -step}, {-step, 0.}};
    for (const auto& iv : intervals) {
        if (source >= 0) record_at(iv[1], record);
        radar_prelaunch_step(L, flight_seeker, st, packet(iv[0]), packet(iv[1]), source >= 0 ? &record : nullptr, target, body_velocity);
    }
    return {st.seeker, st.inertial, st.association_flag};
}

// radar_handoff.constructed_guidance onto a launched guidance state: the seeker
// prefix (+00..+4b), the inertial block and the association byte come from the
// launcher; the seeker cache tail stays the template's. Binary32 packing rounds
// floats and rejects non-finite values and integers outside int32.
inline void radar_handoff(RadarGuidance& g, const RadarSeekerState& template_seeker, const RadarPrelaunch& launcher) {
    const RadarSeekerState& s = launcher.seeker;
    auto packed = [](double x) { if (!finite(x) || std::fabs(x) > (double)FLT_MAX) bail("struct.error: float outside binary32"); return f32(x); };
    auto flag = [](int64_t x) { if (x != 0 && x != 1) bail("ValueError: canonical seeker flags required"); return x; };
    auto int32 = [](int64_t x) { if (x < INT32_MIN || x > INT32_MAX) bail("struct.error: int32 required"); return x; };
    RadarSeekerState out = template_seeker;   // tail: los_check_time, los_cache_valid, visibility_clear, padding 5..6, visibility_weight
    for (int i = 0; i < 2; ++i) out.angles[i] = packed(s.angles[i]);
    for (int i = 0; i < 3; ++i) { out.direction[i] = packed(s.direction[i]); out.angular_rate[i] = packed(s.angular_rate[i]); }
    out.gap = packed(s.gap); out.range = packed(s.range); out.range_rate = packed(s.range_rate);
    out.closure = packed(s.closure); out.closure_rate = packed(s.closure_rate); out.target_value = packed(s.target_value);
    out.rejected = flag(s.rejected); out.range_valid = flag(s.range_valid); out.closure_valid = flag(s.closure_valid);
    out.associated_id = int32(s.associated_id); out.channel_index = int32(s.channel_index); out.target_id = int32(s.target_id);
    for (int i = 0; i < 7; ++i) if (s.padding[i] < 0 || s.padding[i] > 255) bail("ValueError: Unsigned byte required");
    for (int i = 0; i < 5; ++i) out.padding[i] = s.padding[i];
    // The whole launcher block and the template both pass the codec.
    for (const RadarSeekerState* block : {&s, &template_seeker}) {
        (void)packed(block->los_check_time); (void)packed(block->visibility_weight);
        (void)flag(block->los_cache_valid); (void)flag(block->visibility_clear);
    }
    g.seeker = out;
    RadarManagerState& m = g.manager;
    m.association_flag = launcher.association_flag ? 1 : 0;
    m.seeker_angles = out.angles; m.seeker_direction = out.direction; m.seeker_range = out.range; m.seeker_closure = out.closure;
    m.range_valid = out.range_valid; m.closure_valid = out.closure_valid; m.channel_index = out.channel_index;
    const InertialState& in = launcher.inertial;
    m.inertial_valid = flag(in.valid);
    for (int i = 0; i < 3; ++i) { m.inertial_point[i] = packed(in.point[i]); m.inertial_velocity[i] = packed(in.velocity[i]); m.inertial_offset[i] = packed(in.offset[i]); }
    if (in.source_tag < 0 || in.source_tag > 255) bail("ValueError: Unsigned byte required");
    m.inertial_source_tag = in.source_tag;
    for (int i = 0; i < 6; ++i) { if (in.padding[i] < 0 || in.padding[i] > 255) bail("ValueError: Unsigned byte required"); m.inertial_padding[i] = in.padding[i]; }
}

}  // namespace typed
