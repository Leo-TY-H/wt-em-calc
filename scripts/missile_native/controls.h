/* Controllers shared by both guidance families: orientation_control,
 * guidance_request/guidance_modes, acceleration_control (+ control_frame,
 * alternate_control_frame) and propulsion_control. */
#ifndef WTM_CONTROLS_H
#define WTM_CONTROLS_H
#include "body.h"

/* ------------------------------------------------------------ geometry */
/* optical_geometry.boresight / shared_seeker.normalize are needed here too. */
#define NORM_PREDICT 0
#define NORM_CORRECT 1
#define NORM_COAST 2
#define NORM_OTHER 3
/* shared_seeker.normalize */
static inline v3 seeker_normalize(v3 v, int phase) {
    float x = fmul(v.x, v.x), y = fmul(v.y, v.y), z = fmul(v.z, v.z), squared;
    if (phase == NORM_PREDICT) squared = fadd(fadd(z, y), x);
    else if (phase == NORM_CORRECT) squared = fadd(z, fadd(y, x));
    else squared = fadd(fadd(z, x), y);
    float length = fsqrt(squared);
    if (phase == NORM_COAST && (double)length <= 1e-9) return V3(1.0f, 0.0f, 0.0f);
    float reciprocal = length > F(4e-19) ? fdiv(F(1.0), length) : F(0.0);
    return V3(fmul(v.x, reciprocal), fmul(v.y, reciprocal), fmul(v.z, reciprocal));
}

/* optical_geometry.boresight */
static inline v3 boresight(q4 q, const float *angles) {
    wtm_trig ty = half_angle_trig(-fmul(angles[0], F(0.9999999403953552)));
    wtm_trig tp = half_angle_trig(-fmul(angles[1], F(0.9999999403953552)));
    float sy = ty.sine, sp = tp.sine, cy = ty.cosine, cp = tp.cosine;
    float a = fmul(sy, sp), b = fmul(sy, cp), c = fmul(cy, sp), d = fmul(cy, cp);
    float z = fsub(fmul(a, c), fmul(d, b)); z = fadd(z, z);
    float y = fadd(fmul(c, d), fmul(b, a)); y = fadd(y, y);
    float x = fadd(fmul(d, d), fmul(a, a)); x = fsub(fadd(x, x), F(1.0));
    v3 v = seeker_normalize(V3(x, y, z), NORM_PREDICT);
    float qx = q.x, qy = q.y, qz = q.z, qw = q.w;
    float A = fmul(fadd(qz, qz), qx), B = fmul(qy, fadd(qz, qz)), C = fmul(fadd(qz, qz), qw);
    float D = fmul(fadd(qw, qw), qy), E = fmul(qw, fadd(qx, qx)), Fv = fmul(fadd(qx, qx), qy);
    float ww = fsub(fadd(fmul(qw, qw), fmul(qw, qw)), F(1.0));
    float xx = fadd(fadd(fmul(qx, qx), fmul(qx, qx)), ww);
    float yy = fadd(fadd(fmul(qy, qy), fmul(qy, qy)), ww);
    float zz = fadd(fadd(fmul(qz, qz), fmul(qz, qz)), ww);
    return V3(fadd(fadd(fmul(fadd(D, A), v.z), fmul(fsub(Fv, C), v.y)), fmul(xx, v.x)),
              fadd(fadd(fmul(fsub(B, E), v.z), fmul(yy, v.y)), fmul(fadd(C, Fv), v.x)),
              fadd(fadd(fmul(zz, v.z), fmul(fadd(E, B), v.y)), fmul(fsub(A, D), v.x)));
}

/* --------------------------------------------------- orientation_control */
typedef struct {
    float start, duration, final_speed, final_distance, depth_min, elevation[4], heading[4], angle_to_rate, coefficients[5];
    int use_fins;
} wtm_orientation_props;
typedef struct { int enabled; float pid[2][8]; } wtm_orientation_state;
/* angles/elevation/errors are the reference's diagnostics, valid when active. */
typedef struct { int override; float fins[2], auxiliary[2]; int active; float angles[2], elevation, errors[2]; } wtm_orientation_out;

/* orientation_control.interpolate */
static inline float orientation_interpolate(const float *points, float x) {
    float a = points[0], b = points[1], c = points[2], d = points[3];
    if (a > c) { float t = a; a = c; c = t; t = b; b = d; d = t; }
    if (x <= a) return b;
    if (x >= c) return d;
    return fadd(b, safe_div(fmul(fsub(x, a), fsub(d, b)), fsub(c, a)));
}

/* orientation_control.elevated_angles */
static inline void elevated_angles(q4 q, const float *angles, float elevation, float *out) {
    v3 v = boresight(q, angles);
    float vx = v.x, vy = v.y, vz = v.z;
    float horizontal = fsqrt(fadd(fmul(vx, vx), fmul(vz, vz)));
    float inverse = horizontal > F(4e-19) ? fdiv(F(1.0), horizontal) : F(0.0);
    float ux = fmul(inverse, vx), uz = fmul(inverse, -vz);
    float along = fsub(fmul(ux, vx), fmul(uz, vz)), tangent = tanf_native(elevation);
    float term = fmul(vy, tangent);
    vx = fsub(vx, fmul(ux, term)); vy = fadd(fmul(along, tangent), vy); vz = fadd(fmul(term, uz), vz);
    float x = q.x, y = q.y, z = q.z, w = q.w;
    float zz = fadd(z, z), a = fmul(x, zz), b = fmul(y, fmul(F(-2.0), w)), c = fmul(zz, y), d = fmul(x, fmul(F(-2.0), w));
    float e = fmul(fmul(F(-2.0), w), z), base = fadd(fadd(fmul(w, w), fmul(w, w)), F(-1.0));
    float bz = fadd(fadd(fmul(fadd(fadd(fmul(z, z), fmul(z, z)), base), vz), fmul(fsub(a, b), vx)), fmul(fadd(d, c), vy));
    float xy = fmul(fadd(x, x), y);
    float by = fadd(fadd(fmul(fsub(c, d), vz), fmul(fadd(xy, e), vx)), fmul(fadd(fadd(fmul(y, y), fmul(y, y)), base), vy));
    float bx = fadd(fadd(fmul(fadd(b, a), vz), fmul(fadd(fadd(fmul(x, x), fmul(x, x)), base), vx)), fmul(fsub(xy, e), vy));
    out[0] = (float)atan2((double)-bz, (double)bx);
    out[1] = (float)atan2((double)by, (double)fsqrt(fadd(fmul(bz, bz), fmul(bx, bx))));
}

/* orientation_control.update */
static inline wtm_orientation_out orientation_update(const wtm_orientation_props *p, wtm_orientation_state *s, float time,
        float range_squared, q4 q, v3 omega, const float *angles_in, float depth, float dt) {
    wtm_orientation_out r = {0, {0.0f, 0.0f}, {0.0f, 0.0f}, 0, {0.0f, 0.0f}, 0.0f, {0.0f, 0.0f}};
    if (s->enabled && (time > fadd(p->start, p->duration) || depth < p->depth_min)) s->enabled = 0;
    r.override = s->enabled != 0;
    if (!s->enabled || time < p->start) return r;
    float elevation = orientation_interpolate(p->elevation, range_squared);
    float angles[2];
    if ((double)fabsf(elevation) < 1.1920928955078125e-07) { angles[0] = angles_in[0]; angles[1] = angles_in[1]; }
    else elevated_angles(q, angles_in, elevation, angles);
    float heading = orientation_interpolate(p->heading, fabsf(angles[1]));
    float errors[2] = {fsub(fmul(fmul(heading, p->angle_to_rate), -angles[0]), omega.y), fsub(fmul(-angles[1], p->angle_to_rate), omega.z)};
    float kp = p->coefficients[0], ki = p->coefficients[1], limit = p->coefficients[2], kd = p->coefficients[3], tau = p->coefficients[4];
    float inverse = safe_div(F(1.0), tau), values[2];
    for (int i = 0; i < 2; i++) {
        float *prev = s->pid[i], error = errors[i];
        float integral = pmin(limit, pmax(-limit, fadd(fmul(fmul(error, dt), ki), prev[3])));
        float derivative = fadd(fmul(fsub(error, fadd(fmul(dt, prev[7]), prev[6])), inverse), prev[7]);
        float value = -fadd(fadd(integral, fmul(kp, error)), fmul(derivative, kd));
        values[i] = pmin(F(1.0), pmax(F(-1.0), value));
        float next[8] = {kp, ki, limit, integral, kd, inverse, error, derivative};
        memcpy(prev, next, sizeof next);
    }
    if (p->use_fins) { r.fins[0] = values[0]; r.fins[1] = values[1]; }
    else { r.auxiliary[0] = values[0]; r.auxiliary[1] = values[1]; }
    r.active = 1; r.elevation = elevation;
    r.angles[0] = angles[0]; r.angles[1] = angles[1]; r.errors[0] = errors[0]; r.errors[1] = errors[1];
    return r;
}

/* ------------------------------------------------------ guidance_request */
#define WTM_MAX_TABLE 32
typedef struct { int count; wtm_row rows[WTM_MAX_TABLE]; } wtm_table;
typedef struct {
    int loft_enabled; float range_elevation[4], loft_elevation, loft_rate_squared, loft_accel;
    float hold_time, hold_elevation, hold_rate;
    wtm_table time_gain, time_to_hit_gain;
    float elevation_add, angle_gain, omega_gain, course_gain, course_final_angle;
    int absolute_altitude; wtm_table altitude_table;
    float altitude_diff_gain, altitude_rate_gain, climb_accel_gain, climb_limits[2];
} wtm_guidance_props;
typedef struct { int mode, course; float pid[2][8]; } wtm_guidance_state;

/* guidance_request.table_value */
static inline float request_table_value(const wtm_table *t, float value) {
    if (!t->count) return 0.0f;
    if (t->count == 1 || value <= t->rows[0].x) return t->rows[0].y;
    if (value >= t->rows[t->count - 1].x) return t->rows[t->count - 1].y;
    for (int i = 0; i + 1 < t->count; i++)
        if (value <= t->rows[i + 1].x) {
            float fraction = fmul(fsub(value, t->rows[i].x), t->rows[i].slope);
            return fadd(fmul(fsub(t->rows[i + 1].y, t->rows[i].y), fraction), t->rows[i].y);
        }
    wtm_fail(WTM_VALUE, "Unordered or invalid native table");
    return 0.0f;
}

/* guidance_request.table_value over [x, slope, y] rows (acceleration control's cy table). */
static inline float request_rows_value(const wtm_row *rows, int count, float value) {
    if (!count) return 0.0f;
    if (count == 1 || value <= rows[0].x) return rows[0].y;
    if (value >= rows[count - 1].x) return rows[count - 1].y;
    for (int i = 0; i + 1 < count; i++)
        if (value <= rows[i + 1].x) {
            float fraction = fmul(fsub(value, rows[i].x), rows[i].slope);
            return fadd(fmul(fsub(rows[i + 1].y, rows[i].y), fraction), rows[i].y);
        }
    wtm_fail(WTM_VALUE, "Unordered or invalid native table");
    return 0.0f;
}

/* guidance_request.target_angles */
static inline void target_angles(v3 los, q4 q, float *pitch, float *yaw) {
    float lx = los.x, ly = los.y, lz = los.z, x = q.x, y = q.y, z = q.z, w = q.w;
    float twice_z = fadd(z, z), negative_twice_w = fmul(F(-2.0), w);
    float x_twice_z = fmul(x, twice_z), y_negative_twice_w = fmul(y, negative_twice_w);
    float y_twice_z = fmul(twice_z, y), x_negative_twice_w = fmul(x, negative_twice_w);
    float z_negative_twice_w = fmul(negative_twice_w, z);
    float twice_xy = fmul(fadd(x, x), y);
    float twice_ww_minus_one = fadd(fadd(fmul(w, w), fmul(w, w)), F(-1.0));
    float bz = fadd(fmul(fadd(fadd(fmul(z, z), fmul(z, z)), twice_ww_minus_one), lz),
        fadd(fmul(fadd(x_negative_twice_w, y_twice_z), ly), fmul(fsub(x_twice_z, y_negative_twice_w), lx)));
    float by = fadd(fadd(fmul(fsub(y_twice_z, x_negative_twice_w), lz), fmul(fadd(twice_xy, z_negative_twice_w), lx)),
        fmul(fadd(fadd(fmul(y, y), fmul(y, y)), twice_ww_minus_one), ly));
    float bx = fadd(fadd(fmul(fadd(y_negative_twice_w, x_twice_z), lz), fmul(fsub(twice_xy, z_negative_twice_w), ly)),
        fmul(fadd(fadd(fmul(x, x), fmul(x, x)), twice_ww_minus_one), lx));
    *pitch = (float)atan2((double)by, (double)fsqrt(fadd(fmul(bz, bz), fmul(bx, bx))));
    *yaw = (float)atan2((double)-bz, (double)bx);
}

/* guidance_modes.transition */
static inline int guidance_transition(int mode, const wtm_guidance_props *p, v3 los, v3 rate, q4 q, float time_to_hit, float gain) {
    if (gain < F(0.001)) return mode;
    float x = los.x, y = los.y, z = los.z, rx = rate.x, ry = rate.y, rz = rate.z;
    float qx = q.x, qy = q.y, qz = q.z, qw = q.w;
    float elevation = fmul(fabsf(y), y);
    float horizontal = fadd(fmul(z, z), fmul(x, x));
    if (mode == 2) {
        float a = fmul(fadd(fmul(qy, qw), fmul(qx, qz)), rx);
        float b = fmul(fsub(fmul(qy, qz), fmul(qx, qw)), ry);
        float c = fmul(fadd(fmul(F(2.0), fadd(fmul(qw, qw), fmul(qz, qz))), F(-1.0)), rz);
        float projected = fabsf(fadd(fmul(F(2.0), fadd(b, a)), c));
        if (elevation >= fmul(horizontal, p->hold_elevation) && projected <= p->hold_rate && time_to_hit >= p->hold_time) return 2;
        mode = p->loft_enabled ? 1 : 0;
    }
    if (mode == 1) {
        float norm = fadd(fmul(rz, rz), fadd(fmul(ry, ry), fmul(rx, rx)));
        if (elevation < fmul(horizontal, p->loft_elevation) || norm > p->loft_rate_squared) return 0;
    }
    return mode;
}

/* guidance_modes.loft_acceleration */
static inline float loft_acceleration(const wtm_guidance_props *p, q4 q, float range_squared, float ordinary_y) {
    float x0 = p->range_elevation[0], y0 = p->range_elevation[1], x1 = p->range_elevation[2], y1 = p->range_elevation[3];
    if (x0 > x1) { float t = x0; x0 = x1; x1 = t; t = y0; y0 = y1; y1 = t; }
    float distance = range_squared, angle;
    if (distance <= x0) angle = y0;
    else if (distance >= x1) angle = y1;
    else angle = fadd(y0, safe_div(fmul(fsub(distance, x0), fsub(y1, y0)), fsub(x1, x0)));
    float x = q.x, y = q.y, z = q.z, w = q.w;
    float a = fmul(F(2.0), fsub(fmul(x, z), fmul(w, y)));
    float b = fmul(F(2.0), fadd(fmul(z, w), fmul(y, x)));
    float c = fadd(fmul(F(2.0), fadd(fmul(w, w), fmul(x, x))), F(-1.0));
    float length = fsqrt(fadd(fadd(fmul(a, a), fmul(b, b)), fmul(c, c)));
    float inv = length > F(4e-19) ? safe_div(F(1.0), length) : F(0.0);
    a = fmul(a, inv); b = fmul(b, inv); c = fmul(c, inv);
    float pitch = (float)atan2((double)b, (double)fsqrt(fadd(fmul(a, a), fmul(c, c))));
    return pmax(fmul(fsub(angle, pitch), p->loft_accel), ordinary_y);
}

/* target_pitch/target_yaw/command are diagnostics, valid with has_acceleration. */
typedef struct { int mode, course; float gain; int has_acceleration; v3 acceleration; float target_pitch, target_yaw, command[3]; } wtm_request;

/* guidance_request.request; height_query None => altitude unavailable. */
static inline wtm_request guidance_request(const wtm_guidance_props *p, int mode, int course, v3 los, v3 rate, v3 navigation_velocity,
        q4 quaternion, v3 velocity, v3 position, float time, float time_to_hit, float range_squared, int has_height, float height_query) {
    wtm_request r;
    memset(&r, 0, sizeof r);
    float gain = fmul(request_table_value(&p->time_gain, time), request_table_value(&p->time_to_hit_gain, time_to_hit));
    r.mode = mode; r.course = course; r.gain = gain; r.has_acceleration = 0; r.acceleration = V3(0.0f, 0.0f, 0.0f);
    if (gain < F(0.001)) return r;
    float pitch, yaw;
    target_angles(los, quaternion, &pitch, &yaw);
    float rx = rate.x, ry = rate.y, rz = rate.z, omega = p->omega_gain, angle = p->angle_gain;
    float command[3] = {fmul(fmul(omega, gain), rx),
                        fmul(fsub(fmul(ry, omega), fmul(angle, yaw)), gain),
                        fmul(fsub(fmul(rz, omega), fmul(fadd(pitch, p->elevation_add), angle)), gain)};
    if (course == 1) {
        if (fabsf(yaw) > p->course_final_angle) command[1] = fmul(-yaw, p->course_gain);
        else course = 0;
    }
    float vx = navigation_velocity.x, vy = navigation_velocity.y, vz = navigation_velocity.z;
    float cx = command[0], cy = command[1], cz = command[2];
    v3 acc = V3(fsub(fmul(vy, cz), fmul(vz, cy)), fsub(fmul(cx, vz), fmul(vx, cz)), fsub(fmul(vx, cy), fmul(vy, cx)));
    mode = guidance_transition(mode, p, los, rate, quaternion, time_to_hit, gain);
    if (mode == 1) acc.y = loft_acceleration(p, quaternion, range_squared, acc.y);
    else if (mode == 2) {
        int available = p->absolute_altitude ? 1 : has_height;
        float altitude = p->absolute_altitude ? position.y : height_query;
        if (available) {
            float wvx = velocity.x, wvy = velocity.y, wvz = velocity.z;
            float target = request_table_value(&p->altitude_table, time_to_hit);
            float climb = fsub(fmul(fsub(target, altitude), p->altitude_diff_gain), fmul(p->altitude_rate_gain, wvy));
            climb = pmin(pmax(climb, p->climb_limits[0]), p->climb_limits[1]);
            float horizontal_squared = fadd(fmul(wvz, wvz), fmul(wvx, wvx));
            float pitch_velocity = (float)atan2((double)wvy, (double)fsqrt(horizontal_squared));
            float speed = fsqrt(fadd(fmul(wvy, wvy), horizontal_squared));
            acc.y = fmul(fmul(speed, p->climb_accel_gain), fsub(climb, pitch_velocity));
        }
    }
    r.mode = mode; r.course = course; r.has_acceleration = 1; r.acceleration = acc;
    r.target_pitch = pitch; r.target_yaw = yaw; r.command[0] = command[0]; r.command[1] = command[1]; r.command[2] = command[2];
    return r;
}

/* ------------------------------------------------------ control frames */
static inline float cf_length(v3 v, int a, int b, int c) {
    float x = fmul(vget(v, a), vget(v, a)), y = fmul(vget(v, b), vget(v, b)), z = fmul(vget(v, c), vget(v, c));
    return fsqrt(fadd(fadd(x, y), z));
}
static inline v3 cf_normalized(v3 v, int a, int b, int c) {
    float n = cf_length(v, a, b, c);
    float inv = n > F(4e-19) ? fdiv(F(1.0), n) : F(0.0);
    return V3(fmul(v.x, inv), fmul(v.y, inv), fmul(v.z, inv));
}

/* control_frame.matrix_quaternion */
static inline q4 matrix_quaternion(v3 forward, v3 up, v3 right) {
    float fx = forward.x, fy = forward.y, fz = forward.z, ux = up.x, uy = up.y, uz = up.z, rx = right.x, ry = right.y, rz = right.z;
    float trace_x = fadd(fx, F(1.0)), trace_yz = fadd(rz, uy);
    float squared[4] = {fsub(trace_x, trace_yz), fsub(fadd(uy, F(1.0)), fadd(fx, rz)), fadd(fsub(F(1.0), fadd(fx, uy)), rz), fadd(trace_yz, trace_x)};
    float magnitudes[4];
    for (int i = 0; i < 4; i++) magnitudes[i] = squared[i] > 0.0f ? fmul(fsqrt(squared[i]), F(0.5)) : 0.0f;
    float best = F(3.4028234663852886e38);
    q4 chosen = {0.0f, 0.0f, 0.0f, 0.0f};
    static const float signs[2] = {1.0f, -1.0f};
    static const float zw[4][2] = {{1.0f, 1.0f}, {1.0f, -1.0f}, {-1.0f, 1.0f}, {-1.0f, -1.0f}};
    for (int i = 0; i < 2; i++) for (int j = 0; j < 2; j++) for (int k = 0; k < 4; k++) {
        float sx = signs[i], sy = signs[j], sz = zw[k][0], sw = zw[k][1];
        float x = fmul(magnitudes[0], sx), y = fmul(magnitudes[1], sy), z = fmul(magnitudes[2], sz), w = fmul(magnitudes[3], sw);
        float xy = fmul(x, y), wz = fmul(w, z), wy = fmul(w, y), xz = fmul(x, z), yz = fmul(y, z), wx = fmul(w, x);
        float d0 = fsub(fmul(F(2.0), fsub(xy, wz)), ux), d1 = fsub(fmul(F(2.0), fadd(wy, xz)), rx);
        float d2 = fsub(fmul(F(2.0), fadd(xy, wz)), fy), d3 = fsub(fmul(F(2.0), fsub(yz, wx)), ry);
        float d4 = fsub(fmul(F(2.0), fsub(xz, wy)), fz), d5 = fsub(fmul(F(2.0), fadd(yz, wx)), uz);
        float a = fmul(d0, d0), b = fmul(d1, d1), c = fmul(d2, d2), d = fmul(d3, d3), e = fmul(d4, d4), f = fmul(d5, d5);
        float error = sz == sw ? fadd(fadd(fadd(d, b), fadd(c, a)), fadd(f, e)) : fadd(fadd(f, e), fadd(fadd(d, c), fadd(b, a)));
        if (error < best) { chosen.x = x; chosen.y = y; chosen.z = z; chosen.w = w; best = error; }
    }
    return chosen;
}

/* alternate_control_frame.norm */
static inline v3 acf_norm(v3 v, int a, int b, int c) {
    float x = fmul(vget(v, a), vget(v, a)), y = fmul(vget(v, b), vget(v, b)), z = fmul(vget(v, c), vget(v, c));
    float length = fsqrt(fadd(fadd(x, y), z));
    float inverse = length > F(4e-19) ? fdiv(F(1.0), length) : F(0.0);
    return V3(fmul(v.x, inverse), fmul(v.y, inverse), fmul(v.z, inverse));
}

/* alternate_control_frame.frame */
static inline q4 alternate_frame(q4 q, v3 velocity, int velocity_reference) {
    if (!velocity_reference) return q;
    float x = q.x, y = q.y, z = q.z, w = q.w;
    v3 forward = V3(fadd(fmul(F(2.0), fadd(fmul(x, x), fmul(w, w))), F(-1.0)), fmul(F(2.0), fadd(fmul(w, z), fmul(y, x))), fmul(F(2.0), fsub(fmul(z, x), fmul(w, y))));
    forward = acf_norm(forward, 2, 1, 0);
    float vx = velocity.x, vy = velocity.y, vz = velocity.z;
    float speed = fsqrt(fadd(fmul(vz, vz), fadd(fmul(vy, vy), fmul(vx, vx))));
    v3 desired = (double)speed > 1e-9 ? V3(fmul(vx, fdiv(F(1.0), speed)), fmul(vy, fdiv(F(1.0), speed)), fmul(vz, fdiv(F(1.0), speed))) : forward;
    v3 fv = acf_norm(forward, 2, 1, 0), tv = acf_norm(desired, 0, 1, 2);
    float fx = fv.x, fy = fv.y, fz = fv.z, tx = tv.x, ty = tv.y, tz = tv.z;
    float cosine = fadd(fmul(tz, fz), fadd(fmul(ty, fy), fmul(tx, fx)));
    float rx, ry, rz, rw;
    if (cosine < F(-0.9999)) {
        if ((double)fabsf(fz) > 0.7071067811865476) {
            float length = fsqrt(fadd(fmul(fy, fy), fmul(fz, fz)));
            float inverse = length > F(4e-19) ? fdiv(F(1.0), length) : F(0.0);
            rx = 0.0f; ry = fmul(-fz, inverse); rz = fmul(fy, inverse); rw = 0.0f;
        } else {
            float length = fsqrt(fadd(fmul(fx, fx), fmul(fy, fy)));
            float inverse = length > F(4e-19) ? fdiv(F(1.0), length) : F(0.0);
            rx = fmul(-fy, inverse); ry = fmul(fx, inverse); rz = 0.0f; rw = 0.0f;
        }
    } else {
        rx = fsub(fmul(tz, fy), fmul(ty, fz)); ry = fsub(fmul(fz, tx), fmul(tz, fx)); rz = fsub(fmul(ty, fx), fmul(tx, fy));
        float length = fsqrt(fadd(fadd(cosine, cosine), F(2.0)));
        float inverse = length > F(4e-19) ? fdiv(F(1.0), length) : F(0.0);
        rx = fmul(rx, inverse); ry = fmul(ry, inverse); rz = fmul(rz, inverse); rw = fmul(length, F(0.5));
    }
    q4 out;
    out.x = fadd(fsub(fadd(fmul(rz, y), fmul(rw, x)), fmul(z, ry)), fmul(rx, w));
    out.y = fadd(fadd(fsub(fmul(rw, y), fmul(x, rz)), fmul(ry, w)), fmul(rx, z));
    out.z = fsub(fadd(fmul(ry, x), fadd(fmul(rw, z), fmul(rz, w))), fmul(y, rx));
    out.w = fsub(fmul(rw, w), fadd(fmul(rx, x), fadd(fmul(ry, y), fmul(rz, z))));
    return out;
}

/* control_frame.frame */
static inline q4 control_frame(q4 q, v3 velocity, int velocity_reference, int matrix) {
    if (!matrix) return alternate_frame(q, velocity, velocity_reference);
    if (!velocity_reference) return q;
    float x = q.x, y = q.y, z = q.z, w = q.w;
    v3 forward = V3(fadd(fmul(F(2.0), fadd(fmul(x, x), fmul(w, w))), F(-1.0)), fmul(F(2.0), fadd(fmul(w, z), fmul(y, x))), fmul(F(2.0), fsub(fmul(z, x), fmul(w, y))));
    forward = cf_normalized(forward, 2, 1, 0);
    float speed = cf_length(velocity, 0, 1, 2);
    if ((double)speed > 1.e-9) { float inv = fdiv(F(1.0), speed); forward = V3(fmul(velocity.x, inv), fmul(velocity.y, inv), fmul(velocity.z, inv)); }
    v3 up = V3(fmul(F(2.0), fsub(fmul(y, x), fmul(w, z))), fadd(fmul(F(2.0), fadd(fmul(w, w), fmul(y, y))), F(-1.0)), fmul(F(2.0), fadd(fmul(w, x), fmul(z, y))));
    up = cf_normalized(up, 2, 0, 1);
    float fx = forward.x, fy = forward.y, fz = forward.z, ux = up.x, uy = up.y, uz = up.z;
    v3 right = V3(fsub(fmul(uz, fy), fmul(uy, fz)), fsub(fmul(ux, fz), fmul(fx, uz)), fsub(fmul(fx, uy), fmul(fy, ux)));
    right = cf_normalized(right, 1, 0, 2);
    float rx = right.x, ry = right.y, rz = right.z;
    v3 new_up = V3(fsub(fmul(ry, fz), fmul(rz, fy)), fsub(fmul(rz, fx), fmul(rx, fz)), fsub(fmul(rx, fy), fmul(ry, fx)));
    ux = new_up.x; uy = new_up.y; uz = new_up.z;
    float positive = fadd(fadd(fmul(fmul(rx, fy), uz), fmul(fmul(rz, fx), uy)), fmul(fmul(ry, fz), ux));
    float negative = fadd(fmul(fmul(ry, fx), uz), fadd(fmul(fmul(rz, fy), ux), fmul(fmul(rx, fz), uy)));
    if (positive < negative) right = V3(-right.x, -right.y, -right.z);
    return matrix_quaternion(forward, new_up, right);
}

/* ------------------------------------------------- acceleration_control */
#define WTM_MAX_COEFFS 16
typedef struct {
    int velocity_frame; float max_accel; int limit_aoa; float aoa_max, base_speed_squared;
    int schedule_count; float schedule[WTM_MAX_TABLE][3];   /* time, slope, coefficient index */
    int coefficient_count; float coefficients[WTM_MAX_COEFFS][4];
} wtm_acceleration_props;
typedef struct { float mass, cy, cy_limit, side_area; int cy_rows; wtm_row cy_table[WTM_MAX_ROWS]; } wtm_aero_props;

static inline void lateral_axes(q4 q, v3 *h, v3 *v) {
    float x = q.x, y = q.y, z = q.z, w = q.w;
    float hx = fmul(F(2.0), fadd(fmul(z, x), fmul(w, y)));
    float t = fmul(fadd(z, z), y), u = fmul(fmul(F(-2.0), x), w);
    float hy = fadd(u, t);
    float common = fadd(fmul(F(2.0), fmul(w, w)), F(-1.0));
    float hz = fadd(fmul(F(2.0), fmul(z, z)), common);
    float vx = fmul(F(2.0), fsub(fmul(x, y), fmul(w, z)));
    float vy = fadd(fmul(F(2.0), fmul(y, y)), common), vz = fsub(t, u);
    *h = V3(hx, hy, hz); *v = V3(vx, vy, vz);
}

static inline float density_polynomial(float height) {
    float z = pmin(height, F(18300.0)), a = F(2.28719e-19);
    a = fadd(fmul(a, z), F(-5.83556e-14)); a = fadd(fmul(a, z), F(3.53118e-9)); a = fadd(fmul(a, z), F(-9.59387e-5)); a = fadd(fmul(a, z), F(1.0));
    return a;
}
/* acceleration_control.atmosphere_values with the default environment. */
static inline void atmosphere_values(float height, float *density, float *sound) {
    float ceiling = F(18300.0), z = pmin(height, ceiling);
    *density = fdiv(fmul(fmul(ceiling, F(1.225)), density_polynomial(height)), pmax(height, ceiling));
    float poly = F(3.97306e-18);
    poly = fadd(fmul(poly, z), F(-5.71104e-14)); poly = fadd(fmul(poly, z), F(2.18069e-10)); poly = fadd(fmul(poly, z), F(-2.27712e-5)); poly = fadd(fmul(poly, z), F(1.0));
    *sound = fmul(fsqrt(fmul(poly, F(288.16))), F(20.1));
}

/* acceleration_control.pid_coefficients: returns 0 when None. */
static inline int pid_coefficients(const wtm_acceleration_props *p, float time, float *out) {
    if (!p->schedule_count) {
        if (p->coefficient_count == 1) { memcpy(out, p->coefficients[0], 4 * sizeof(float)); return 1; }
        return 0;
    }
    int n = p->schedule_count;
    if (n == 1 || time <= p->schedule[0][0]) { memcpy(out, p->coefficients[(int)p->schedule[0][2]], 16); return 1; }
    if (time >= p->schedule[n - 1][0]) { memcpy(out, p->coefficients[(int)p->schedule[n - 1][2]], 16); return 1; }
    for (int i = 0; i + 1 < n; i++)
        if (time <= p->schedule[i + 1][0]) {
            float fraction = fmul(fsub(time, p->schedule[i][0]), p->schedule[i][1]);
            const float *x = p->coefficients[(int)p->schedule[i][2]], *y = p->coefficients[(int)p->schedule[i + 1][2]];
            for (int k = 0; k < 4; k++) out[k] = fadd(fmul(fsub(y[k], x[k]), fraction), x[k]);
            return 1;
        }
    wtm_fail(WTM_VALUE, "Invalid schedule");
    return 0;
}

/* Output of acceleration_update; everything after has_fins is the reference's
 * diagnostics, valid only when has_fins. */
typedef struct {
    q4 frame; int has_fins; float fins[2], errors[2], limited[2], actual[2];
    int has_aoa_limit; float aoa_limit, scale, scale_squared, coefficients[4];
} wtm_acceleration_out;

/* acceleration_control.update with the default environment. Returns 0 when
 * there are no coefficients (fins None); state (two 8-float PIDs, axis-major)
 * is updated in place only when fins are produced. */
static inline int acceleration_update(const wtm_acceleration_props *p, const wtm_aero_props *aero, wtm_motor_scalar motor,
        float *state, v3 request, v3 measured, q4 q, v3 velocity, float height, float time, float dt, int matrix, wtm_acceleration_out *o) {
    memset(o, 0, sizeof *o);
    q4 reference = control_frame(q, velocity, p->velocity_frame, matrix);
    o->frame = reference;
    v3 h, v; lateral_axes(reference, &h, &v);
    float wanted[2] = {fadd(fadd(fmul(h.z, request.z), fmul(h.x, request.x)), fmul(h.y, request.y)),
                       fadd(fmul(v.z, request.z), fadd(fmul(v.y, request.y), fmul(v.x, request.x)))};
    float squared = fadd(fmul(wanted[1], wanted[1]), fmul(wanted[0], wanted[0]));
    float vx = velocity.x, vy = velocity.y, vz = velocity.z;
    float speed_squared = fadd(fmul(vz, vz), fadd(fmul(vy, vy), fmul(vx, vx)));
    float limit = p->max_accel;
    if (aero && p->limit_aoa) {
        float mass = fsub(aero->mass, motor.mass_lost);
        float cy_mult = 1.0f;
        if (aero->cy_rows) {
            float density, sound; atmosphere_values(height, &density, &sound);
            cy_mult = request_rows_value(aero->cy_table, aero->cy_rows, fdiv(speed_squared, sound));
        }
        float factor = fmul(F(1.225), F(0.5));
        factor = fmul(factor, speed_squared); factor = fmul(factor, F(18300.0)); factor = fmul(factor, density_polynomial(height));
        factor = fmul(factor, cy_mult); factor = fmul(factor, aero->cy); factor = fmul(factor, aero->side_area);
        factor = fdiv(factor, pmax(height, F(18300.0)));
        float aoa_limit = fdiv(fmul(fadd(factor, motor.thrust), pmin(p->aoa_max, aero->cy_limit)), mass);
        limit = pmin(limit, aoa_limit);
        o->has_aoa_limit = 1; o->aoa_limit = aoa_limit;
    }
    float limit_squared = fmul(limit, limit);
    if (squared > limit_squared) {
        float reduction = fsqrt(fdiv(limit_squared, squared));
        wanted[0] = fmul(wanted[0], reduction); wanted[1] = fmul(wanted[1], reduction);
    }
    float scale_squared = 1.0f;
    if (p->base_speed_squared > 0.0f) {
        float rho, sound; atmosphere_values(height, &rho, &sound);
        scale_squared = safe_div(fmul(fmul(F(1.225), F(1.225)), p->base_speed_squared), fmul(fmul(rho, rho), speed_squared));
    }
    float scale = fsqrt(scale_squared), coefficients[4];
    if (!pid_coefficients(p, time, coefficients)) return 0;
    float mx = measured.x, my = measured.y, mz = measured.z;
    float actual[2] = {fadd(fmul(h.y, my), fadd(fmul(h.z, mz), fmul(h.x, mx))), fadd(fmul(v.z, mz), fadd(fmul(v.y, my), fmul(v.x, mx)))};
    float errors[2] = {fsub(wanted[0], actual[0]), fsub(wanted[1], actual[1])};
    float kp = fmul(coefficients[0], scale), ki = fmul(coefficients[1], scale), kd = fmul(coefficients[2], scale_squared), ilim = coefficients[3];
    for (int index = 0; index < 2; index++) {
        float *previous = state + 8 * index, error = errors[index];
        float integral = pmin(ilim, pmax(-ilim, fadd(fmul(fmul(error, dt), ki), previous[3])));
        float derivative = fadd(fmul(fsub(error, fadd(fmul(dt, previous[7]), previous[6])), F(48.0)), previous[7]);
        float proportional = fmul(kp, error), differential = fmul(derivative, kd);
        float value = index == 0 ? fadd(fadd(differential, proportional), integral) : fadd(fadd(integral, proportional), differential);
        o->fins[index] = pmin(F(1.0), pmax(F(-1.0), value));
        float next[8] = {kp, ki, ilim, integral, kd, F(48.0), error, derivative};
        memcpy(previous, next, sizeof next);
    }
    o->has_fins = 1; o->scale = scale; o->scale_squared = scale_squared;
    for (int k = 0; k < 2; k++) { o->errors[k] = errors[k]; o->limited[k] = wanted[k]; o->actual[k] = actual[k]; }
    for (int k = 0; k < 4; k++) o->coefficients[k] = coefficients[k];
    return 1;
}

/* --------------------------------------------------- propulsion_control */
typedef struct {
    float start, altitude[2], speed_squared, closure_min, tgo_min; int use_ias;
    float cruise, cruise_error_limit, base_ias_squared, coefficients[4], pursuit_closure[2], terminal_tgo;
} wtm_stage;
typedef struct { int count; wtm_stage stages[4]; } wtm_propulsion_control_props;
typedef struct { int stages[4]; float pid[8]; } wtm_propulsion_control_state;

static inline void atmosphere_terms(float height, float *density, float *ias_factor) {
    float ceiling = F(18300.0), reference = F(1.225), sea = F(1.225);
    float z = pmin(height, ceiling), poly = F(2.28719e-19);
    poly = fadd(fmul(poly, z), F(-5.83556e-14)); poly = fadd(fmul(poly, z), F(3.53118e-9)); poly = fadd(fmul(poly, z), F(-9.59387e-5)); poly = fadd(fmul(poly, z), F(1.0));
    float numerator = fmul(poly, fmul(reference, ceiling)), denominator = pmax(height, ceiling);
    *density = fdiv(numerator, denominator);
    *ias_factor = fsqrt(fdiv(numerator, fmul(denominator, sea)));
}

/* propulsion_control.update with the default environment. */
static inline void propulsion_control_update(const wtm_propulsion_control_props *p, wtm_propulsion_control_state *s, float time,
        v3 position, v3 velocity, float tgo, float closure, float dt, float *factors) {
    for (int i = 0; i < 4; i++) factors[i] = 1.0f;
    if (!p->count) return;
    float height = position.y, vx = velocity.x, vy = velocity.y, vz = velocity.z;
    float speed_squared = fadd(fmul(vz, vz), fadd(fmul(vy, vy), fmul(vx, vx)));
    for (int index = 0; index < p->count; index++) {
        const wtm_stage *stage = &p->stages[index];
        int mode = s->stages[index];
        if (mode == 0) {
            int start = time > stage->start && (height < stage->altitude[0] || height > stage->altitude[1]
                || speed_squared < stage->speed_squared || closure < stage->closure_min || tgo < stage->tgo_min);
            if (start) s->stages[index] = 1; else factors[index] = 0.0f;
        } else if (mode == 1 || mode == 2) {
            float density = 0.0f, ias = 1.0f;
            if (stage->use_ias || (mode == 2 && stage->base_ias_squared > 0.0f)) atmosphere_terms(height, &density, &ias);
            float speed = fmul(fsqrt(speed_squared), stage->use_ias ? ias : F(1.0));
            if (mode == 1) { if (speed > stage->cruise) s->stages[index] = 2; continue; }
            float ratio = 1.0f, root = 1.0f;
            if (stage->base_ias_squared > 0.0f) {
                ratio = safe_div(fmul(stage->base_ias_squared, fmul(F(1.225), F(1.225))), fmul(fmul(density, density), speed_squared));
                root = fsqrt(ratio);
            }
            float prop = stage->coefficients[0], intg = stage->coefficients[1], diff = stage->coefficients[2], limit = stage->coefficients[3];
            float kp = fmul(root, prop), ki = fmul(root, intg), kd = fmul(ratio, diff);
            float error = pmin(stage->cruise_error_limit, pmax(-stage->cruise_error_limit, fsub(stage->cruise, speed)));
            float *previous = s->pid;
            float integral = pmin(limit, pmax(-limit, fadd(fmul(fmul(ki, dt), error), previous[3])));
            float derivative = fadd(fmul(fadd(fsub(fmul(previous[7], -dt), previous[6]), error), F(48.0)), previous[7]);
            factors[index] = pmin(F(1.0), pmax(F(0.0), fadd(fadd(integral, fmul(kp, error)), fmul(derivative, kd))));
            float next[8] = {kp, ki, limit, integral, kd, F(48.0), error, derivative};
            memcpy(s->pid, next, sizeof next);
            if (tgo < stage->terminal_tgo) s->stages[index] = 4;
            else if (closure < stage->pursuit_closure[0]) s->stages[index] = 3;
        } else if (mode == 3) {
            if (tgo < stage->terminal_tgo) s->stages[index] = 4;
            else if (closure > stage->pursuit_closure[1]) s->stages[index] = 2;
        }
    }
}

#endif
