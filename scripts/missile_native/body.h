/* Body physics: motor_vector, aero_vectors, body_integration, body_perturbation
 * and the body half of optical_point_flight.update. */
#ifndef WTM_BODY_H
#define WTM_BODY_H
#include "core.h"

#define WTM_MAX_ROWS 64

typedef struct { float duration, force, mass_lost, vectoring; } wtm_impulse;
typedef struct { float delay; int count; wtm_impulse impulses[4]; } wtm_propulsion;
typedef struct { int count; wtm_propulsion items[4]; } wtm_motors;

/* Rocket fields consumed by the motor (kernels.pressure_multiplier, motor_vector). */
typedef struct {
    float ext_pressure[4];          /* extPressureToThrustMult, f32 */
    float thrust_deviation;          /* thrustDeviation */
    float orientation_torque;        /* orientationTorque */
} wtm_rocket;

typedef struct { float x, slope, y; } wtm_row;

/* flight properties p['body'] (kernels.aero_properties + loader extras). */
typedef struct {
    float mass, front_area, side_area, stabilizer_arm, length, cx, cy, cy_limit, cx_aoa;
    float fins_hor, fins_ver, fin_pressure_limit, damping_geometry;
    float inertia[3], angular_damping[3];
    q4 axis_quaternion;
    int cy_rows; wtm_row cy_table[WTM_MAX_ROWS];
} wtm_body_props;

typedef struct {
    v3 position; q4 quaternion; v3 velocity; v3 omega;
    float time; uint32_t seed; float body_random; float clocks[4];
    int water; float distance, water_distance, surface_depth, immersion;
} wtm_body;

/* ------------------------------------------------------------- vector ops */
static inline float vdot(v3 a, v3 b) { return fadd(fadd(fmul(a.x, b.x), fmul(a.y, b.y)), fmul(a.z, b.z)); }
static inline v3 vcross(v3 a, v3 b) {
    return V3(fsub(fmul(a.y, b.z), fmul(a.z, b.y)), fsub(fmul(a.z, b.x), fmul(a.x, b.z)), fsub(fmul(a.x, b.y), fmul(a.y, b.x)));
}
static inline v3 vscale(v3 a, float s) { return V3(fmul(a.x, s), fmul(a.y, s), fmul(a.z, s)); }
static inline v3 vplus(v3 a, v3 b) { return V3(fadd(a.x, b.x), fadd(a.y, b.y), fadd(a.z, b.z)); }
static inline float vlength(v3 a) { return fsqrt(vdot(a, a)); }
/* aero_vectors.unit(a, eps, fallback): eps compared in double. */
static inline v3 vunit(v3 a, double eps, v3 fallback) {
    float n = vlength(a);
    return (double)n > eps ? vscale(a, fdiv(F(1.0), n)) : fallback;
}
static inline float vget(v3 v, int i) { return i == 0 ? v.x : i == 1 ? v.y : v.z; }

/* --------------------------------------------------------------- motors */
/* kernels.motor_scalar */
typedef struct { float thrust, mass_lost, vectoring; } wtm_motor_scalar;
static inline wtm_motor_scalar motor_scalar(const wtm_motors *m, const float *clocks) {
    wtm_motor_scalar out = {0.0f, 0.0f, 0.0f};
    for (int p = 0; p < m->count; p++) {
        const wtm_propulsion *prop = &m->items[p];
        float t = fsub(clocks[p], prop->delay), elapsed = 0.0f, used = 0.0f, force = 0.0f, angle = 0.0f;
        for (int i = 0; i < prop->count; i++) {
            const wtm_impulse *item = &prop->impulses[i];
            float end = fadd(elapsed, item->duration);
            if (end > t) {
                float local = fsub(t, elapsed);
                if (local > 0.0f) {
                    force = item->force; angle = item->vectoring;
                    used = fadd(used, fmul(safe_div(local, item->duration), item->mass_lost));
                }
                break;
            }
            used = fadd(used, item->mass_lost); elapsed = end;
        }
        out.thrust = fadd(out.thrust, force); out.mass_lost = fadd(out.mass_lost, used); out.vectoring = fadd(out.vectoring, angle);
    }
    return out;
}

/* kernels.pressure_multiplier */
static inline float pressure_multiplier(const wtm_rocket *r, float height) {
    float a = r->ext_pressure[0], ya = r->ext_pressure[1], b = r->ext_pressure[2], yb = r->ext_pressure[3];
    if (a > b) { float t = a; a = b; b = t; t = ya; ya = yb; yb = t; }
    float p = atmosphere(height).pressure;
    if (p <= a) return ya;
    if (p >= b) return yb;
    return fadd(ya, safe_div(fmul(fsub(p, a), fsub(yb, ya)), fsub(b, a)));
}

/* motor_vector.deviation_sample */
static inline float deviation_sample(uint32_t word) {
    float slope = MOTOR_DEVIATION_TABLE[(word >> 23) & 255][0], intercept = MOTOR_DEVIATION_TABLE[(word >> 23) & 255][1];
    return fadd(fmul((float)((word >> 16) & 127), slope), intercept);
}

typedef struct { v3 world_force, body_torque; float mass_lost; float clock_rates[4]; uint32_t seed; } wtm_propulsion_out;

/* motor_vector.propulsion_adapter (motor_vector inlined) */
static inline wtm_propulsion_out propulsion_adapter(const wtm_rocket *rocket, const wtm_motors *motors, float length,
        const float *clocks, const float *factors, float dt, q4 quaternion, float height,
        float fin0, float fin1, float orient0, float orient1, uint32_t seed) {
    wtm_propulsion_out out;
    int n = motors->count;
    float future[4];
    for (int i = 0; i < 4; i++) {
        out.clock_rates[i] = i < n ? factors[i] : 0.0f;
        future[i] = fadd(clocks[i], fmul(out.clock_rates[i], dt));
    }
    wtm_motor_scalar scalar = motor_scalar(motors, future);
    float factor = rocket->ext_pressure[1];
    if (scalar.thrust > 0.0f && rocket->ext_pressure[1] != rocket->ext_pressure[3]) factor = pressure_multiplier(rocket, height);
    float scale = fmul(factor, scalar.thrust);
    float z = fmul(-fin0, scalar.vectoring), y = fmul(-fin1, scalar.vectoring);
    float x = fsqrt(pmax(fsub(F(1.0), fadd(fmul(z, z), fmul(y, y))), F(0.0)));
    uint32_t first = seed * 0x41c64e6du + 0x3039u;
    uint32_t second = first * 0x41c64e6du + 0x3039u;
    float deviation = rocket->thrust_deviation;
    float noisy_z = fadd(fmul(deviation_sample(first), deviation), z);
    float noisy_y = fadd(fmul(deviation_sample(second), deviation), y);
    float torque = rocket->orientation_torque;
    v3 thrust = V3(fmul(x, scale), fmul(noisy_y, scale), fmul(noisy_z, scale));
    v3 noiseless = V3(fmul(x, scale), fmul(y, scale), fmul(z, scale));
    v3 ot = V3(0.0f, fmul(-orient0, torque), fmul(-orient1, torque));
    float lever = fmul(length, F(-0.5));
    out.world_force = rotate_thrust(quaternion, thrust);
    out.body_torque = V3(ot.x, fadd(fmul(lever, noiseless.z), ot.y), fsub(ot.z, fmul(lever, noiseless.y)));
    out.mass_lost = scalar.mass_lost;
    out.seed = second;
    return out;
}

/* ------------------------------------------------------------------ aero */
typedef struct { float force_scale, angle, cosine, sine, lever_fraction; } wtm_perturb;
/* body_perturbation.coefficients (immersion 0) */
static inline wtm_perturb perturbation_coefficients(float amplitude, float body_random) {
    wtm_perturb out = {1.0f, 0.0f, 1.0f, 0.0f, 0.0f};
    float a = amplitude, s = body_random;
    if (a <= 0.0f) return out;
    float scale = a < 2.0f ? fmul(F(0.01), a) : F(0.02);
    float angle = fmul(a < 2.0f ? fmul(F(PI_D / 4), a) : F(PI_D / 2), s);
    out.force_scale = fadd(fmul(scale, s), 1.0f); out.angle = angle;
    out.cosine = (float)cos((double)angle); out.sine = (float)sin((double)angle);
    out.lever_fraction = a < F(0.5) ? fmul(a, F(1.6)) : F(0.8);
    return out;
}

static inline q4 composed_quaternion(q4 q, q4 p) {
    float x = q.x, y = q.y, z = q.z, w = q.w, a = p.x, b = p.y, c = p.z, d = p.w;
    q4 out;
    out.x = fadd(fsub(fadd(fmul(x, d), fmul(a, w)), fmul(b, z)), fmul(c, y));
    out.y = fsub(fadd(fadd(fmul(y, d), fmul(b, w)), fmul(a, z)), fmul(c, x));
    out.z = fadd(fsub(fmul(b, x), fmul(a, y)), fadd(fmul(z, d), fmul(c, w)));
    out.w = fsub(fmul(d, w), fadd(fmul(c, z), fadd(fmul(b, y), fmul(a, x))));
    return out;
}

/* aero_vectors.columns: col[i] is a column vector. */
static inline void quaternion_columns(q4 q, v3 col[3]) {
    float x = q.x, y = q.y, z = q.z, w = q.w;
#define TW(a) fadd((a), (a))
    col[0] = V3(fsub(TW(fadd(fmul(x, x), fmul(w, w))), 1.0f), TW(fadd(fmul(x, y), fmul(z, w))), TW(fsub(fmul(x, z), fmul(y, w))));
    col[1] = V3(TW(fsub(fmul(x, y), fmul(z, w))), fsub(TW(fadd(fmul(y, y), fmul(w, w))), 1.0f), TW(fadd(fmul(y, z), fmul(x, w))));
    col[2] = V3(TW(fadd(fmul(x, z), fmul(y, w))), TW(fsub(fmul(y, z), fmul(x, w))), fsub(TW(fadd(fmul(z, z), fmul(w, w))), 1.0f));
#undef TW
}

/* aero_vectors.table_value */
static inline float aero_table_value(const wtm_row *rows, int count, float x) {
    if (!count) return 1.0f;
    if (x <= rows[0].x) return rows[0].y;
    if (x >= rows[count - 1].x) return rows[count - 1].y;
    for (int i = 0; i + 1 < count; i++)
        if (x <= rows[i + 1].x) return fadd(rows[i].y, fmul(fsub(rows[i + 1].y, rows[i].y), fmul(fsub(x, rows[i].x), rows[i].slope)));
    wtm_fail(WTM_VALUE, "aero table lookup failed");
    return 1.0f;
}

typedef struct { v3 drag, lift, force; float cosine, cd, cy; } wtm_aero_eval;
typedef struct {
    v3 frame[3]; v3 axes[3]; float cm_speed, mach, pressure; v3 local_flow, flow;
    wtm_aero_eval baseline, steering; int fin_active, fin_limited; float deflection[2]; v3 fin_flow;
    v3 moment; float damping[3]; int damping_clipped[3];
    v3 angular_acceleration_before_environment, angular_acceleration;
    double acceleration[3]; float effective_mass;
} wtm_forces;

typedef struct {
    const wtm_body_props *props; v3 forward; float cx_mach, ki, cy, pressure;
} wtm_aero_ctx;

static inline wtm_aero_eval aero_evaluate(const wtm_aero_ctx *c, v3 direction) {
    wtm_aero_eval out;
    const wtm_body_props *p = c->props;
    float cosine = vdot(direction, c->forward);
    float sin2 = pmax(fsub(F(1.0), fmul(cosine, cosine)), F(0.0));
    float lift_coefficient = fmul(fsqrt(sin2), c->cy), cap = p->cy_limit;
    if (lift_coefficient > cap) lift_coefficient = pmax(fsub(fadd(cap, cap), lift_coefficient), F(0.0));
    else if (lift_coefficient < -cap) lift_coefficient = pmin(fsub(fmul(F(-2.0), cap), lift_coefficient), F(0.0));
    float cd = fadd(c->cx_mach, fmul(fmul(sin2, fmul(p->cx, p->cx_aoa)), c->ki));
    out.drag = vscale(direction, fmul(cd, fmul(p->front_area, -c->pressure)));
    v3 lift_direction = vunit(vcross(direction, vcross(direction, c->forward)), 1e-9, V3(0.0f, 0.0f, 0.0f));
    float lift_scalar = fmul(lift_coefficient, fmul(p->side_area, -c->pressure));
    if (cosine < 0) lift_scalar = -lift_scalar;
    out.lift = vscale(lift_direction, lift_scalar);
    out.force = vplus(out.drag, out.lift);
    out.cosine = cosine; out.cd = cd; out.cy = lift_coefficient;
    return out;
}

/* aero_vectors.forces with zero wind, additional cx/lever, mass term and
 * angular environment; gravity and Cxi enabled (the flight composition). */
static inline void aero_forces(const wtm_body_props *p, float height, v3 velocity, q4 q, v3 omega, float fin0, float fin1,
        float dt, v3 force, v3 torque, float mass_lost, float perturbation, float body_random, wtm_forces *o) {
    v3 relative = V3(fsub(velocity.x, 0.0f), fsub(velocity.y, 0.0f), fsub(velocity.z, 0.0f));
    double cm_speed64 = sqrt((((double)0 + (double)relative.x * (double)relative.x) + (double)relative.y * (double)relative.y) + (double)relative.z * (double)relative.z);
    float speed = (float)cm_speed64;
    q4 composed = composed_quaternion(q, p->axis_quaternion);
    quaternion_columns(composed, o->frame);
    static const int orders[3][3] = {{2, 1, 0}, {2, 0, 1}, {0, 1, 2}};
    for (int k = 0; k < 3; k++) {
        v3 col = o->frame[k];
        float a = fmul(vget(col, orders[k][0]), vget(col, orders[k][0]));
        float b = fmul(vget(col, orders[k][1]), vget(col, orders[k][1]));
        float c = fmul(vget(col, orders[k][2]), vget(col, orders[k][2]));
        float n = fsqrt(fadd(fadd(a, b), c));
        o->axes[k] = n > F(4e-19) ? vscale(col, fdiv(F(1.0), n)) : V3(0.0f, 0.0f, 0.0f);
    }
    float ww = fsub(fmul(F(2.0), fmul(composed.w, composed.w)), F(1.0));
    o->frame[0].x = fadd(fmul(F(2.0), fmul(composed.x, composed.x)), ww);
    o->frame[1].y = fadd(fmul(F(2.0), fmul(composed.y, composed.y)), ww);
    o->frame[2].z = fadd(fmul(F(2.0), fmul(composed.z, composed.z)), ww);
    v3 forward = o->axes[0];
    v3 fallback = speed > F(0.001) ? V3((float)((double)relative.x / cm_speed64), (float)((double)relative.y / cm_speed64), (float)((double)relative.z / cm_speed64)) : forward;
    float arm = fadd(p->stabilizer_arm, 0.0f);
    wtm_perturb perturb = perturbation_coefficients(perturbation, body_random);
    float offset = fmul(fmul(perturb.lever_fraction, body_random), p->stabilizer_arm);
    float cosine_offset = fmul(perturb.cosine, offset), sine_offset = fmul(offset, perturb.sine);
    float bx = fsub(fmul(omega.z, sine_offset), fmul(omega.y, cosine_offset));
    float by = fadd(fmul(omega.z, arm), fmul(omega.x, cosine_offset));
    float bz = fsub(fmul(omega.y, -arm), fmul(omega.x, sine_offset));
    v3 *f = o->frame;
    v3 local = V3(fadd(fadd(fmul(f[2].x, bz), relative.x), fmul(f[1].x, by)),
                  fadd(fadd(fmul(f[2].y, bz), relative.y), fmul(f[1].y, by)),
                  fadd(fadd(relative.z, fmul(f[1].z, by)), fmul(f[2].z, bz)));
    local = V3(fadd(local.x, fmul(f[0].x, bx)), fadd(local.y, fmul(f[0].y, bx)), fadd(local.z, fmul(f[0].z, bx)));
    o->local_flow = local;
    v3 flow = vunit(local, 1e-9, fallback);
    o->flow = flow;
    wtm_air air = atmosphere(height);
    float mach = fdiv(speed, air.sound_speed);
    float z = pmin(height, F(18300.0)), poly = F(2.28719e-19);
    poly = fadd(fmul(poly, z), F(-5.83556e-14)); poly = fadd(fmul(poly, z), F(3.53118e-9));
    poly = fadd(fmul(poly, z), F(-9.59387e-5)); poly = fadd(fmul(poly, z), F(1.0));
    float pressure = fdiv(fmul(fmul(speed, speed), fmul(fmul(fmul(F(1.225), F(0.5)), F(18300.0)), poly)), pmax(height, F(18300.0)));
    float k = drag_mach(mach);
    float ki = fadd(fmul(k, F(2.4352500438690186)), F(0.25001898407936096));
    float cx_total = fadd(p->cx, 0.0f);
    float cx_mach = (F(1.4) <= mach && (double)mach < 4.0) ? fdiv(fmul(mach, cx_total), fadd(fmul(fadd(fmul(F(0.356), mach), F(2.237)), mach), F(-1.4))) : fmul(cx_total, k);
    float cy = fmul(p->cy, aero_table_value(p->cy_table, p->cy_rows, mach));
    wtm_aero_ctx ctx = {p, forward, cx_mach, ki, cy, pressure};
    o->cm_speed = speed; o->mach = mach; o->pressure = pressure;
    o->baseline = aero_evaluate(&ctx, flow);
    float deflection[2] = {fmul(fin0, p->fins_hor), fmul(fin1, p->fins_ver)};
    int active = fmul(fabsf(fin0), p->fins_hor) > F(1e-5) || fmul(fabsf(fin1), p->fins_ver) > F(1e-5);
    int limited = 0; v3 fin_flow = flow; o->steering = o->baseline;
    if (active) {
        float size2 = fadd(fmul(deflection[0], deflection[0]), fmul(deflection[1], deflection[1]));
        float limit = p->fin_pressure_limit;
        if (fmul(fmul(pressure, pressure), size2) > fmul(limit, limit)) {
            float factor = fdiv(F(1.0), fmul(pressure, fsqrt(size2)));
            deflection[0] = fmul(fmul(deflection[0], limit), factor); deflection[1] = fmul(fmul(deflection[1], limit), factor);
            limited = 1;
        }
        fin_flow = vunit(vplus(vplus(flow, vscale(o->axes[2], deflection[0])), vscale(o->axes[1], deflection[1])), 1e-9, fallback);
        o->steering = aero_evaluate(&ctx, fin_flow);
    }
    o->fin_active = active; o->fin_limited = limited; o->deflection[0] = deflection[0]; o->deflection[1] = deflection[1]; o->fin_flow = fin_flow;
    float fx = vdot(f[0], o->steering.force), fy = vdot(f[1], o->steering.force), fz = vdot(f[2], o->steering.force);
    v3 moment = V3(fsub(fmul(fy, cosine_offset), fmul(fz, sine_offset)),
                   fsub(fmul(-arm, fz), fmul(fx, cosine_offset)),
                   fadd(fmul(arm, fy), fmul(fx, sine_offset)));
    o->moment = moment;
    v3 raw_torque = vplus(moment, torque);
    const float *inertia = p->inertia;
    float qgeom = fmul(pressure, p->damping_geometry);
    float om[3] = {omega.x, omega.y, omega.z}, rt[3] = {raw_torque.x, raw_torque.y, raw_torque.z}, tq[3] = {torque.x, torque.y, torque.z};
    float mo[3] = {moment.x, moment.y, moment.z}, damping[3];
    for (int i = 0; i < 3; i++) {
        float d = fmul(fmul(fmul(i == 0 ? F(-0.01) : F(-0.05), om[i]), qgeom), p->angular_damping[i]);
        float stopping = fadd(fmul(fmul(om[i], inertia[i]), fdiv(F(1.0), dt)), rt[i]);
        if (i == 0) stopping = fadd(fadd(fmul(fmul(om[i], inertia[i]), fdiv(F(1.0), dt)), tq[i]), mo[i]);
        int clipped = fabsf(stopping) < fabsf(d);
        if (clipped) d = -stopping;
        o->damping_clipped[i] = clipped; damping[i] = d; o->damping[i] = d;
    }
    float gyro[3] = {fmul(fmul(om[2], om[1]), fsub(inertia[2], inertia[1])),
                     fmul(fsub(inertia[0], inertia[2]), fmul(om[2], om[0])),
                     fmul(fmul(om[0], om[1]), fsub(inertia[1], inertia[0]))};
    float alpha[3];
    for (int i = 0; i < 3; i++) alpha[i] = fdiv(fadd(fadd(rt[i], gyro[i]), damping[i]), inertia[i]);
    alpha[0] = fdiv(fadd(fadd(fadd(gyro[0], tq[0]), mo[0]), damping[0]), inertia[0]);
    o->angular_acceleration_before_environment = V3(alpha[0], alpha[1], alpha[2]);
    float x = q.x, y = q.y, zq = q.z, w = q.w, ex = 0.0f, ey = 0.0f, ez = 0.0f;
    float zx = fmul(fadd(zq, zq), x), yw = fmul(fadd(y, y), w), ey2 = fadd(ey, ey);
    float ww2 = fsub(fadd(fmul(w, w), fmul(w, w)), F(1.0));
    float plus_x = fadd(fmul(fadd(fadd(fmul(zq, zq), fmul(zq, zq)), ww2), ez),
                        fadd(fmul(fadd(fmul(zq, y), fmul(x, w)), ey2), fmul(fsub(zx, yw), ex)));
    float plus_z = fsub(fsub(fmul(fsub(fmul(zq, w), fmul(y, x)), ey2),
                             fmul(fadd(fadd(fmul(x, x), fmul(x, x)), ww2), ex)), fmul(fadd(yw, zx), ez));
    o->angular_acceleration = V3(fadd(fmul(plus_x, F(9.81)), alpha[0]), alpha[1], fadd(fmul(plus_z, F(9.81)), alpha[2]));
    float mass = fsub(fadd(fmul(p->mass, 0.0f), p->mass), mass_lost);
    float inverse = safe_div(F(1.0), mass);
    double fs = (double)perturb.force_scale;
    o->acceleration[0] = (((double)o->baseline.drag.x + (double)o->baseline.lift.x) * fs + (double)force.x) * (double)inverse;
    o->acceleration[1] = (((double)o->baseline.drag.y + (double)o->baseline.lift.y) * fs + (double)force.y) * (double)inverse;
    o->acceleration[2] = (((double)o->baseline.drag.z + (double)o->baseline.lift.z) * fs + (double)force.z) * (double)inverse;
    o->acceleration[1] -= (double)fmul(mass, F(9.81)) * (double)inverse;
    double n2 = ((0.0 + o->acceleration[0] * o->acceleration[0]) + o->acceleration[1] * o->acceleration[1]) + o->acceleration[2] * o->acceleration[2];
    if (n2 >= 6000.0 * 6000.0) {
        double r = sqrt(6000.0 * 6000.0 / n2);
        for (int i = 0; i < 3; i++) o->acceleration[i] = o->acceleration[i] * r;
    }
    o->effective_mass = mass;
}

/* ------------------------------------------------------------ integration */
/* body_integration.orientation */
static inline q4 integrate_orientation(q4 quaternion, v3 increment) {
    wtm_trig tx = half_angle_trig(increment.x), ty = half_angle_trig(increment.y), tz = half_angle_trig(increment.z);
    float sx = tx.sine, sy = ty.sine, sz = tz.sine, cx = tx.cosine, cy = ty.cosine, cz = tz.cosine;
    float a0 = fmul(fmul(sy, cx), sz), a1 = fmul(fmul(sy, cx), cz), a2 = fmul(fmul(cy, cx), sz), a3 = fmul(fmul(cy, cx), cz);
    float b0 = fmul(fmul(cy, sx), cz), b1 = fmul(fmul(cy, sx), sz), b2 = fmul(fmul(sy, sx), cz), b3 = fmul(fmul(sy, sx), sz);
    float dx = fadd(b0, a0), dy = fadd(b1, a1), dz = fsub(a2, b2), dw = fsub(a3, b3);
    float x = quaternion.x, y = quaternion.y, z = quaternion.z, w = quaternion.w;
    float raw[4];
    raw[0] = fsub(fadd(fmul(dz, y), fadd(fmul(dx, w), fmul(dw, x))), fmul(dy, z));
    raw[1] = fsub(fadd(fmul(dx, z), fadd(fmul(dw, y), fmul(dy, w))), fmul(dz, x));
    raw[2] = fsub(fadd(fmul(dy, x), fadd(fmul(dw, z), fmul(dz, w))), fmul(dx, y));
    raw[3] = fsub(fmul(dw, w), fadd(fmul(dx, x), fadd(fmul(dz, z), fmul(dy, y))));
    float norm2 = fadd(fadd(fmul(raw[3], raw[3]), fmul(raw[2], raw[2])), fadd(fmul(raw[1], raw[1]), fmul(raw[0], raw[0])));
    q4 out = {0.0f, 0.0f, 0.0f, 0.0f};
    if (norm2 != 0.0f) {
        float inv = fdiv(F(1.0), fsqrt(norm2));
        out.x = fmul(raw[0], inv); out.y = fmul(raw[1], inv); out.z = fmul(raw[2], inv); out.w = fmul(raw[3], inv);
    }
    return out;
}

/* body_integration.integrate (pure air: immersion becomes 0) */
static inline void integrate_body(wtm_body *b, const double *acceleration, v3 angular_acceleration, float dt, float absolute_time, const float *clock_rates) {
    float half_dt2 = fmul(fmul(dt, dt), F(0.5));
    float vel[3] = {b->velocity.x, b->velocity.y, b->velocity.z}, pos[3] = {b->position.x, b->position.y, b->position.z};
    float om[3] = {b->omega.x, b->omega.y, b->omega.z}, aa[3] = {angular_acceleration.x, angular_acceleration.y, angular_acceleration.z};
    float npos[3], nvel[3], inc[3], nom[3];
    for (int i = 0; i < 3; i++) {
        float displacement = fadd((float)(acceleration[i] * (double)half_dt2), fmul(vel[i], dt));
        npos[i] = fadd(pos[i], displacement);
        nvel[i] = fadd((float)(acceleration[i] * (double)dt), vel[i]);
        inc[i] = fadd(fmul(aa[i], half_dt2), fmul(om[i], dt));
        nom[i] = fadd(fmul(aa[i], dt), om[i]);
    }
    q4 rotation = integrate_orientation(b->quaternion, V3(inc[0], inc[1], inc[2]));
    float distance_step = fmul(fsqrt(fadd(fadd(fmul(nvel[2], nvel[2]), fmul(nvel[1], nvel[1])), fmul(nvel[0], nvel[0]))), dt);
    float elapsed = fsub(absolute_time, b->time);
    for (int i = 0; i < 4; i++) b->clocks[i] = fadd(fmul(clock_rates[i], elapsed), b->clocks[i]);
    b->position = V3(npos[0], npos[1], npos[2]); b->velocity = V3(nvel[0], nvel[1], nvel[2]); b->omega = V3(nom[0], nom[1], nom[2]);
    b->quaternion = rotation; b->time = absolute_time;
    float new_distance = fadd(b->distance, distance_step);
    if (b->water) b->water_distance = fadd(distance_step, b->water_distance);
    b->distance = new_distance; b->immersion = 0.0f;
}

/* optical_point_flight.cap_velocity */
static inline v3 cap_velocity(v3 v, float c) {
    float squared = fadd(fadd(fmul(v.x, v.x), fmul(v.y, v.y)), fmul(v.z, v.z));
    if ((double)fabsf(c) >= 1.1920928955078125e-07 && squared > fmul(c, c)) {
        float factor = fdiv(c, fsqrt(squared));
        v = V3(fmul(v.x, factor), fmul(v.y, factor), fmul(v.z, factor));
    }
    return v;
}

static inline float clampf(float v, float limit) { return pmin(limit, pmax(-limit, v)); }

typedef struct {
    const wtm_body_props *body; const wtm_rocket *rocket; const wtm_motors *motors;
    int precision; float end_speed;
} wtm_flight_props;

typedef struct { wtm_propulsion_out propulsion; float target_time; } wtm_body_step_out;
/* One entry of optical_point_flight.update's substeps list. */
typedef struct { wtm_body state; double acceleration[3]; v3 angular_acceleration; } wtm_substep;

/* Body half of optical_point_flight.update: motor, substeps, clocks and time.
 * controls[0..1] fins, [2..3] orientation, [4..7] motor factors. substeps,
 * when not NULL, receives p->precision entries. */
static inline void body_advance(const wtm_flight_props *p, wtm_body *b, const float *controls, float dt, wtm_body_step_out *out,
        wtm_substep *substeps) {
    if (b->water || b->immersion != 0.0f) { wtm_fail(WTM_UNSUPPORTED, "Pure-air composition only"); return; }
    float target_time = fadd(b->time, dt);
    float motor_dt = fsub(target_time, b->time);
    wtm_propulsion_out propulsion = propulsion_adapter(p->rocket, p->motors, p->body->length, b->clocks, controls + 4, motor_dt,
        b->quaternion, b->position.y, controls[0], controls[1], controls[2], controls[3], b->seed);
    b->seed = propulsion.seed;
    float sub_dt = fdiv(dt, (float)p->precision);
    b->surface_depth = pmax(fsub(F(0.0), b->position.y), F(-2147440000.0));
    wtm_forces f;
    for (int i = 0; i < p->precision; i++) {
        float sub_time = fadd(b->time, sub_dt);
        aero_forces(p->body, b->position.y, b->velocity, b->quaternion, b->omega, controls[0], controls[1], sub_dt,
                    propulsion.world_force, propulsion.body_torque, propulsion.mass_lost, 0.0f, b->body_random, &f);
        integrate_body(b, f.acceleration, f.angular_acceleration, sub_dt, sub_time, propulsion.clock_rates);
        b->velocity = V3(clampf(b->velocity.x, F(2000.0)), clampf(b->velocity.y, F(2000.0)), clampf(b->velocity.z, F(2000.0)));
        b->omega = V3(clampf(b->omega.x, F(15.0)), clampf(b->omega.y, F(15.0)), clampf(b->omega.z, F(15.0)));
        if (substeps) {
            substeps[i].state = *b; substeps[i].angular_acceleration = f.angular_acceleration;
            for (int k = 0; k < 3; k++) substeps[i].acceleration[k] = f.acceleration[k];
        }
    }
    float remaining = fsub(target_time, b->time);
    for (int i = 0; i < 4; i++) b->clocks[i] = fadd(b->clocks[i], fmul(propulsion.clock_rates[i], remaining));
    b->time = target_time;
    out->propulsion = propulsion; out->target_time = target_time;
}

#endif
