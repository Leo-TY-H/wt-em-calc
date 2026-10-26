// Typed ports of the website release setup: missile_worker.quaternion,
// body_launch.position_seed/construct, launch_spread.sample/attitude/velocity and
// body_launch.initialize in launch mode 2. Each mirrors its scripts/missile_model
// source; scalar sin/cos/sqrt are the translated engine's library calls.
#pragma once
#include "typed_core.h"
#include <cstring>
#include <vector>

namespace typed {

// missile_worker.quaternion: heading, pitch and roll in degrees; binary64 products.
inline d4 angles_quaternion(const d3& angles) {
    const double degree = 3.141592653589793238462643383279502884 / 180.0;   // math.radians
    double heading = angles[0] * degree / 2, pitch = angles[1] * degree / 2, roll = angles[2] * degree / 2;
    auto product = [](const d4& a, const d4& b) {
        double x = a[0], y = a[1], z = a[2], w = a[3], X = b[0], Y = b[1], Z = b[2], W = b[3];
        return d4{w * X + x * W + y * Z - z * Y, w * Y - x * Z + y * W + z * X, w * Z + x * Y - y * X + z * W, w * W - x * X - y * Y - z * Z};
    };
    d4 q = product(product({0., -std::sin(heading), 0., std::cos(heading)}, {0., 0., std::sin(pitch), std::cos(pitch)}),
                   {std::sin(roll), 0., 0., std::cos(roll)});
    return f32s(q);
}

// ------------------------------------------------------------ launch_spread
constexpr uint64_t SPREAD_MASK = 0xffffffffull, SPREAD_STEP = 0x9e3779b9ull;
inline uint64_t spread_mix(uint64_t seed) {
    uint64_t product = (seed ^ (seed >> 16)) * 0x21f0aaadull;
    return ((product >> 32) ^ product) & SPREAD_MASK;
}
inline double float_bits(uint64_t bits) { uint32_t b = (uint32_t)bits; float f; std::memcpy(&f, &b, 4); return f; }
using SpreadTable = std::vector<std::array<double,2>>;
struct SpreadDraw { double magnitude, sine, cosine; uint64_t global_seed; };
inline SpreadDraw spread_sample(uint64_t seed, uint64_t global_seed, const SpreadTable& table) {
    seed &= SPREAD_MASK; global_seed &= SPREAD_MASK;
    if (seed == 0) { global_seed = (global_seed + SPREAD_STEP) & SPREAD_MASK; seed = spread_mix(global_seed) >> 1; }
    uint64_t word = (seed * 0x41c64e6dull + 0x3039ull) & SPREAD_MASK;
    size_t index = (word >> 23) & 255; uint64_t fraction = (word >> 16) & 127;
    if (index >= table.size()) bail("IndexError: launch spread table");
    double magnitude = add(mul((double)fraction, table[index][0]), table[index][1]);
    double uniform = float_bits((spread_mix((word + SPREAD_STEP) & SPREAD_MASK) >> 9) | 0x3f800000ull);
    double azimuth = add(mul(uniform, 6.2831854820251465), -6.2831854820251465);
    wtm_trig trig = half_angle_trig((float)mul(-2., azimuth));
    check_kernel();
    return {magnitude, trig.sine, trig.cosine, global_seed};
}

// ------------------------------------------------------------- body_launch
// Per-profile inputs of body_launch.initialize (rocket fields and motor layout).
struct LaunchProps {
    double max_delta_angle = 0., advanced_spread = 0., start_speed = 0., vel_spread = 1., time_life = -1., distance_limit = 0.;
    bool use_start_speed = false, distance_fuse = true;
    double first_delay = 0.; int64_t propulsion_count = 0;
    SpreadTable table;
};
// body_launch.construct for a release record with zero omega at time zero.
inline wtm_body construct_body(const d3& position_in, const d4& quaternion_in, const d3& velocity_in) {
    d3 position = f32s(position_in), velocity = f32s(velocity_in); d4 quaternion = f32s(quaternion_in);
    for (double x : position) if (!finite(x)) bail("ValueError: Invalid release position");
    for (double x : quaternion) if (!finite(x)) bail("ValueError: Invalid release quaternion");
    for (double x : velocity) if (!finite(x)) bail("ValueError: Invalid release velocity");
    // position_seed
    uint64_t words[3];
    for (int i = 0; i < 3; ++i) {
        double scaled = mul(position[i], 100.);
        words[i] = (-2147483648.0 <= scaled && scaled < 2147483648.0) ? (uint64_t)(int64_t)std::trunc(scaled) & SPREAD_MASK : 0x80000000ull;
    }
    uint64_t seed = ((words[0] ^ words[1] ^ words[2]) + SPREAD_STEP) & SPREAD_MASK;
    double fraction = sub(float_bits((spread_mix(seed) >> 9) | 0x3f800000ull), 1.);
    wtm_body b{};
    b.position = fv(position); b.quaternion = fq(quaternion); b.velocity = fv(velocity); b.omega = V3(0.f, 0.f, 0.f);
    b.time = 0.f; b.seed = (uint32_t)seed; b.body_random = (float)fraction;
    for (float& clock : b.clocks) clock = 0.f;
    b.water = 0; b.distance = 0.f; b.water_distance = 0.f; b.surface_depth = (float)sub(0., position[1]); b.immersion = 0.f;
    return b;
}
// body_launch.initialize(rocket, body, 123, 0x12345678, launch_time=0, clock_time=0,
// launch_mode=2, distance_limit) as missile_worker.launch_body calls it.
struct LaunchedBody { wtm_body body; double lifetime; };
inline LaunchedBody launch_body(const LaunchProps& p, const wtm_body& body) {
    const uint64_t launch_seed = 123, global_seed = 0x12345678;
    wtm_body b = body;
    b.omega = V3(0.f, 0.f, 0.f); b.time = 0.f;
    // launch_spread.attitude
    d4 q = dq(b.quaternion);
    double max_angle = f32(p.max_delta_angle), scale = mul(p.advanced_spread, 1.);
    uint64_t global = global_seed;
    if (!(max_angle < 0.)) {
        SpreadDraw draw = spread_sample(launch_seed, global, p.table);
        double norm = sqrt32(add(mul(draw.sine, draw.sine), mul(draw.cosine, draw.cosine)));
        double inverse = norm > f32(4e-19) ? div(1., norm) : 0.;
        double half = mul(mul(mul(scale, .008726646192371845), max_angle), draw.magnitude);
        double s = mul(f32(std::sin(half)), inverse);
        double dy = mul(s, draw.sine), dz = mul(s, draw.cosine), dw = f32(std::cos(half));
        double x = q[0], y = q[1], z = q[2], w = q[3];
        q = {sub(add(mul(dz, y), mul(dw, x)), mul(dy, z)), sub(add(mul(dw, y), mul(dy, w)), mul(dz, x)),
             add(mul(dy, x), add(mul(dw, z), mul(dz, w))), sub(mul(dw, w), add(mul(dz, z), mul(dy, y)))};
        global = draw.global_seed;
    }
    // launch_spread.velocity
    double vx = b.velocity.x, vy = b.velocity.y, vz = b.velocity.z, x = q[0], y = q[1], z = q[2], w = q[3];
    double twice_z = add(z, z), xz = mul(x, twice_z), minus_twice_w = mul(-2., w);
    double nyw = mul(y, minus_twice_w), nxw = mul(x, minus_twice_w), nzw = mul(minus_twice_w, z);
    double ww = sub(add(mul(w, w), mul(w, w)), 1.);
    double xx = add(add(mul(x, x), mul(x, x)), ww), yy = add(add(mul(y, y), mul(y, y)), ww), zz = add(add(mul(z, z), mul(z, z)), ww);
    double twice_y = add(y, y), xy = mul(twice_y, x), yz = mul(twice_z, y);
    double local_x = add(mul(vx, xx), add(mul(sub(xy, nzw), vy), mul(add(nyw, xz), vz)));
    double local_y = add(add(mul(add(xy, nzw), vx), mul(sub(yz, nxw), vz)), mul(vy, yy));
    double local_z = add(add(mul(sub(xz, nyw), vx), mul(add(nxw, yz), vy)), mul(vz, zz));
    double factor = p.propulsion_count && f32(p.first_delay) > 0. ? 1. : f32(.333);
    local_x = add(local_x, p.use_start_speed ? f32(p.start_speed) : 200.);
    local_y = mul(local_y, factor); local_z = mul(local_z, factor);
    if (f32(p.max_delta_angle) > 0.) {
        SpreadDraw draw = spread_sample(launch_seed, global, p.table);
        double angle = mul(mul(mul(mul(p.vel_spread, 1.), .01745329238474369), f32(p.max_delta_angle)), draw.magnitude);
        double spread = mul(f32(std::sin(angle)), local_x);
        local_y = add(local_y, mul(draw.sine, spread));
        local_z = add(local_z, mul(spread, draw.cosine));
    }
    double yw = mul(twice_y, w), twice_x = add(x, x), xw = mul(twice_x, w), xy2 = mul(twice_x, y), zw = mul(twice_z, w);
    d3 v = {add(mul(add(yw, xz), local_z), add(mul(sub(xy2, zw), local_y), mul(xx, local_x))),
            add(mul(sub(yz, xw), local_z), add(mul(yy, local_y), mul(add(xy2, zw), local_x))),
            add(mul(zz, local_z), add(mul(add(xw, yz), local_y), mul(sub(xz, yw), local_x)))};
    b.quaternion = fq(q); b.velocity = fv(v);
    double speed = sqrt32(add(add(mul(v[0], v[0]), mul(v[1], v[1])), mul(v[2], v[2])));
    double lifetime;
    if (p.distance_fuse && f32(p.distance_limit) > 0.) lifetime = div(p.distance_limit, add(speed, .001));
    else { double life = f32(p.time_life); lifetime = life > 0. ? mul(life, 1.) : 300.; }
    return {b, lifetime};
}

}  // namespace typed
