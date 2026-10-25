/* Native missile model core: binary32 arithmetic and recovered math routines.
 *
 * Every function mirrors a Python reference in scripts/missile_model with the
 * same operation order. Python's f32()/add/sub/mul/div round both operands to
 * binary32 and the result; here float storage and float arithmetic do the same
 * (strict IEEE evaluation, no contraction, no excess precision). Where the
 * reference computes in double (host math.*, explicit float() expressions),
 * this code computes in double too.
 */
#ifndef WTM_CORE_H
#define WTM_CORE_H
#include <math.h>
#include <stdint.h>
#include <string.h>
#if defined(_MSC_VER)
#include <intrin.h>
#endif

/* ------------------------------------------------------------------ errors */
/* The reference raises exceptions for invalid states; the native engine records
 * the first failure and the caller reports it. */
#ifdef __cplusplus
static thread_local int wtm_error_code = 0;
static thread_local const char *wtm_error_text = 0;
#else
static int wtm_error_code = 0;
static const char *wtm_error_text = 0;
#endif
static inline void wtm_fail(int code, const char *text) {
    if (!wtm_error_code) { wtm_error_code = code; wtm_error_text = text; }
}
#define WTM_ZERO_DIVISION 1
#define WTM_DOMAIN 2
#define WTM_UNSUPPORTED 3
#define WTM_VALUE 4

/* -------------------------------------------------------------- arithmetic */
#define F(x) ((float)(x))
#define PI_D 3.141592653589793
static inline float f32(double x) { return (float)x; }
static inline float fadd(float a, float b) { return a + b; }
static inline float fsub(float a, float b) { return a - b; }
static inline float fmul(float a, float b) { return a * b; }
static inline float fdiv(float a, float b) {
    if (b == F(0.0)) { wtm_fail(WTM_ZERO_DIVISION, "float division by zero"); return F(0.0); }
    return a / b;
}
/* kernels.safe_div: div(a,b) if abs(f32(b))>f32(4e-19) else 0. */
static inline float safe_div(float a, float b) { return fabsf(b) > F(4e-19) ? a / b : F(0.0); }
/* Python min/max of two values: the first argument wins ties (and NaN rules). */
static inline float pmin(float a, float b) { return b < a ? b : a; }
static inline float pmax(float a, float b) { return b > a ? b : a; }
static inline double dmin(double a, double b) { return b < a ? b : a; }
static inline double dmax(double a, double b) { return b > a ? b : a; }
/* f32(math.sqrt(x)) for a binary32 argument. */
static inline float fsqrt(float x) {
    if (x < F(0.0)) { wtm_fail(WTM_DOMAIN, "math domain error (sqrt)"); return F(0.0); }
    return (float)sqrt((double)x);
}
static inline double dsqrt(double x) {
    if (x < 0.0) { wtm_fail(WTM_DOMAIN, "math domain error (sqrt)"); return 0.0; }
    return sqrt(x);
}
static inline float copysignf_(float magnitude, float sign) { return (float)copysign((double)magnitude, (double)sign); }

/* ------------------------------------------------------------------ vectors */
typedef struct { float x, y, z; } v3;
typedef struct { float x, y, z, w; } q4;
static inline v3 V3(float x, float y, float z) { v3 v = {x, y, z}; return v; }

/* ------------------------------------------------------------ tables (gen) */
#include "tables.h"

/* ------------------------------------------------------------- atmosphere */
typedef struct { float density, sound_speed, pressure; } wtm_air;

static inline float poly5(const float *c, float z) {
    float value = c[0];
    for (int i = 1; i < 5; i++) value = fadd(fmul(value, z), c[i]);
    return value;
}
static const float AIR_DENSITY[5] = {F(2.28719e-19), -F(5.83556e-14), F(3.53118e-9), -F(9.59387e-5), F(1.0)};
static const float AIR_SOUND[5] = {F(3.97306e-18), -F(5.71104e-14), F(2.18069e-10), -F(2.27712e-5), F(1.0)};
static const float AIR_PRESSURE[5] = {F(1.60373e-18), -F(1.3738e-13), F(5.6763e-9), -F(1.18441e-4), F(1.0)};

/* kernels.atmosphere */
static inline wtm_air atmosphere(float h) {
    wtm_air out;
    float z = pmin(h, F(18300.0));
    float top = pmax(h, F(18300.0));
    out.density = fdiv(fmul(fmul(F(1.225), F(18300.0)), poly5(AIR_DENSITY, z)), top);
    out.sound_speed = fmul(fsqrt(fmul(poly5(AIR_SOUND, z), F(288.16))), F(20.1));
    out.pressure = fdiv(fmul(fmul(F(101300.0), F(18300.0)), poly5(AIR_PRESSURE, z)), top);
    return out;
}

/* kernels.drag_mach */
static inline float drag_mach(float m) {
    if (m < F(0.61)) return F(0.308);
    if ((double)m < 1.0) return fadd(fmul(F(0.505), (float)pow((double)fsub(m, F(0.61)), (double)F(2.31))), F(0.308));
    if (m < F(1.4)) {
        float d = fsub(m, F(1.0));
        return fadd(fmul(fmul(F(0.4485), (float)pow((double)d, (double)F(0.505))), (float)exp((double)fmul(d, -F(5.68)))), F(0.551));
    }
    if ((double)m < 4.0) return fdiv(m, fadd(fmul(fadd(fmul(F(0.356), m), F(2.237)), m), -F(1.4)));
    return F(0.302);
}

/* ---------------------------------------------------------- native atan2f */
/* native_atan2f.atan2f: client CRT atan2f with double intermediates. */
static inline float atan2f_native(float y, float x) {
    if (!isfinite(y) || !isfinite(x)) { wtm_fail(WTM_UNSUPPORTED, "Only finite atan2f inputs are reconstructed"); return F(0.0); }
    int negative_x = signbit(x) != 0, negative_y = signbit(y) != 0;
    double ax = fabs((double)x), ay = fabs((double)y);
    if (ay == 0.0) return negative_x ? (float)copysign((double)(float)PI_D, (double)y) : y;
    if (ax == 0.0) return (float)copysign((double)(float)(PI_D / 2), (double)y);
    int ey, ex; frexp(ay, &ey); frexp(ax, &ex);
    int delta = ey - ex;
    if (delta > 26) return (float)copysign((double)(float)(PI_D / 2), (double)y);
    if (delta < -13 && !negative_x) return (float)((double)y / (double)x);
    if (delta < -26 && negative_x) return (float)copysign((double)(float)PI_D, (double)y);
    int swapped = ay > ax;
    if (swapped) { double t = ay; ay = ax; ax = t; }
    double ratio = ay / ax, angle;
    if (ratio > 0.0625) {
        long long index = (long long)trunc(ratio * 256.0 + 0.5);
        double residual = (ay * 256.0 - (double)index * ax) / ((double)index * ay + ax * 256.0);
        angle = residual + ATAN2F_TABLE[index - 16];
        angle -= ((residual * residual) * residual) * 0.33333333333224097;
    } else if (ratio >= 0.0001) {
        double squared = ratio * ratio;
        double coefficient = 0.19999999999393223 - squared * 0.1428571356180717;
        coefficient = 0.3333333333333317 - coefficient * squared;
        angle = ratio - coefficient * (squared * ratio);
    } else angle = ratio;
    if (swapped) angle = PI_D / 2 - angle;
    if (negative_x) angle = PI_D - angle;
    if (negative_y) angle = -angle;
    return (float)angle;
}

/* ------------------------------------------------------------ native tanf */
static inline double bits_to_double(uint64_t bits) { double d; memcpy(&d, &bits, 8); return d; }
static inline uint64_t double_to_bits(double d) { uint64_t b; memcpy(&b, &d, 8); return b; }
static inline uint64_t mul_64x64(uint64_t a, uint64_t b, uint64_t *high) {
#if defined(_MSC_VER) && defined(_M_X64)
    return _umul128(a, b, high);
#elif defined(__SIZEOF_INT128__)
    unsigned __int128 p = (unsigned __int128)a * b; *high = (uint64_t)(p >> 64); return (uint64_t)p;
#else
    uint64_t a0 = a & 0xffffffffu, a1 = a >> 32, b0 = b & 0xffffffffu, b1 = b >> 32;
    uint64_t p00 = a0 * b0, p01 = a0 * b1, p10 = a1 * b0, p11 = a1 * b1;
    uint64_t middle = (p00 >> 32) + (p01 & 0xffffffffu) + (p10 & 0xffffffffu);
    *high = p11 + (p01 >> 32) + (p10 >> 32) + (middle >> 32);
    return (middle << 32) | (p00 & 0xffffffffu);
#endif
}
static inline int bit_length64(uint64_t v) { int n = 0; while (v) { n++; v >>= 1; } return n; }

static inline double tanf_polynomial(double x) {
    double squared = x * x;
    double numerator = TANF_P1 * squared;
    numerator = numerator + TANF_P0;
    double denominator = TANF_Q2 * squared;
    denominator = denominator + TANF_Q1;
    denominator = denominator * squared;
    denominator = denominator + TANF_Q0;
    double ratio = numerator / denominator;
    double correction = squared * x;
    correction = correction * ratio;
    return x + correction;
}

/* native_tanf._large_reduce */
static inline double tanf_large_reduce(double magnitude, int *quadrant_out) {
    uint64_t word = double_to_bits(magnitude);
    long long exponent = (long long)(word >> 52) - 0x3ff;
    long long offset = 0x86 - (exponent >> 3);
    uint64_t mantissa = (word & ((1ULL << 52) - 1)) | (1ULL << 52);
    uint64_t first, second, third;
    memcpy(&first, TANF_TABLE + offset, 8); memcpy(&second, TANF_TABLE + offset + 8, 8); memcpy(&third, TANF_TABLE + offset + 16, 8);
    uint64_t ph, pl, qh, ql, rh, rl;
    pl = mul_64x64(first, mantissa, &ph);
    uint64_t low = pl;
    ql = mul_64x64(second, mantissa, &qh);
    /* product = second*mantissa + (first*mantissa >> 64) */
    uint64_t middle = ql + ph; uint64_t carry_add = middle < ql ? 1 : 0;
    uint64_t product_high = qh + carry_add;
    rl = mul_64x64(third, mantissa, &rh); (void)rh;
    uint64_t high = product_high + rl;
    int residual_exponent = (int)(exponent & 7);
    int shift = 0x36 - residual_exponent;
    uint64_t quadrant = high >> shift;
    uint64_t carry = (high >> (shift - 1)) & 1;
    uint64_t sign = 0;
    if (carry) { high = ~high; middle = ~middle; low = ~low; sign = 1ULL << 63; }
    quadrant = (quadrant + carry) & 3;
    shift = residual_exponent + 10;
    high = (high << shift) >> shift;
    long long result_exponent = shift - 64;
    if (high == 0) { high = middle; middle = low; low = 0; result_exponent -= 64; }
    if (high == 0) { wtm_fail(WTM_VALUE, "Unresolved zero significand in finite tanf reduction"); *quadrant_out = 0; return 0.0; }
    int leading = bit_length64(high) - 1;
    result_exponent += leading;
    shift = leading - 52;
    if (shift > 0) {
        uint64_t previous = high;
        high >>= shift;
        middle = (middle >> shift) | (previous << (64 - shift));
    } else if (shift < 0) {
        shift = -shift;
        uint64_t previous = middle;
        high = (high << shift) | (previous >> (64 - shift));
        middle = (middle << shift) | (low >> (64 - shift));
    }
    result_exponent += 0x3ff;
    high = (high & ~(1ULL << 52)) | sign | ((uint64_t)result_exponent << 52);
    double reduced = bits_to_double(high);
    *quadrant_out = (int)quadrant;
    return reduced * TANF_HALF_PI;
}

/* native_tanf.tanf */
static inline float tanf_native(float x) {
    if (!isfinite(x)) { wtm_fail(WTM_UNSUPPORTED, "Only finite tanf inputs are reconstructed"); return F(0.0); }
    double magnitude = fabs((double)x);
    if (magnitude <= TANF_QUARTER_PI) {
        if (magnitude < TANF_SMALL) return x;
        if (magnitude < TANF_CUBIC) {
            double correction = (double)x * (double)x;
            correction = correction * (double)x;
            correction = correction * TANF_THIRD;
            return (float)(correction + (double)x);
        }
        return (float)tanf_polynomial((double)x);
    }
    double reduced; int quadrant;
    if (magnitude < TANF_LARGE) {
        long long index = (long long)trunc(magnitude * TANF_TWO_OVER_PI + 0.5);
        double head = (double)index * TANF_PI2_HEAD;
        double residual = magnitude - head;
        double tail = (double)index * TANF_PI2_TAIL;
        reduced = residual - tail;
        quadrant = (int)(index & 3);
    } else reduced = tanf_large_reduce(magnitude, &quadrant);
    double result = tanf_polynomial(reduced);
    if (quadrant & 1) result = result != 0.0 ? -1.0 / result : copysign(INFINITY, -result);
    if (signbit(x)) result = -result;
    return (float)result;
}

/* --------------------------------------------- body_integration helpers */
typedef struct { float sine, cosine; long long quadrant; float reduced; } wtm_trig;

/* body_integration.half_angle_trig */
static inline wtm_trig half_angle_trig(float increment) {
    wtm_trig out;
    float h = fmul(-increment, F(0.5));
    float quadrant_input = fsub((float)copysign(0.5, (double)h), fmul(increment, F(0.31830987334251404)));
    double qt = trunc((double)quadrant_input);
    if (!(qt >= -2147483648.0 && qt < 2147483648.0)) { wtm_fail(WTM_VALUE, "Outside audited finite quadrant-conversion domain"); qt = 0.0; }
    long long quadrant = (long long)qt;
    float t = fadd(fmul((float)quadrant, -F(1.5707963705062866)), h);
    float t2 = fmul(t, t);
    float c = fadd(fmul(fadd(fmul(fadd(fmul(-F(0.0013602249091491103), t2), F(0.04165669530630112)), t2), -F(0.4999990165233612)), t2), F(1.0));
    float s = fadd(fmul(fadd(fmul(fadd(fmul(-F(0.0001950727018993348), t2), F(0.00833207555115223)), t2), -F(0.16666652262210846)), fmul(t2, t)), t);
    if (quadrant & 1) { float tmp = s; s = c; c = tmp; }
    if (quadrant & 2) s = -s;
    if ((quadrant + 1) & 2) c = -c;
    out.sine = s; out.cosine = c; out.quadrant = quadrant; out.reduced = t;
    return out;
}

/* motor_vector.rotate_thrust: native float32 expression order, unnormalized q. */
static inline v3 rotate_thrust(q4 q, v3 t) {
    float x = q.x, y = q.y, z = q.z, w = q.w;
    float a = fmul(fadd(z, z), x), b = fmul(y, fadd(z, z)), c = fmul(fadd(z, z), w);
    float d = fmul(fadd(w, w), y), e = fmul(w, fadd(x, x)), f = fmul(fadd(x, x), y);
    float ww = fsub(fadd(fmul(w, w), fmul(w, w)), F(1.0));
    float xx = fadd(fadd(fmul(x, x), fmul(x, x)), ww);
    float yy = fadd(fadd(fmul(y, y), fmul(y, y)), ww);
    float zz = fadd(fadd(fmul(z, z), fmul(z, z)), ww);
    v3 out;
    out.x = fadd(fadd(fmul(fadd(d, a), t.z), fmul(fsub(f, c), t.y)), fmul(xx, t.x));
    out.y = fadd(fadd(fmul(fsub(b, e), t.z), fmul(fadd(c, f), t.x)), fmul(yy, t.y));
    out.z = fadd(fmul(zz, t.z), fadd(fmul(fadd(e, b), t.y), fmul(fsub(a, d), t.x)));
    return out;
}

#endif
