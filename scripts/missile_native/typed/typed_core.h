// Typed missile engine: Python-exact scalar semantics without dynamic values.
//
// Every Python float is held as the identical binary64 value. kernels.f32/add/
// sub/mul/div round both operands and the result to binary32 exactly as the
// reference does (binary64 is wide enough that the double rounding is exact).
// Anything the reference would raise for, or that this port does not cover,
// throws Bail; the caller then reruns the whole sample on the translated engine.
// Include after controls.h (core.h kernels, wtm_error_code).
#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <initializer_list>
#include <utility>

namespace nt { double reference_pow(double,double); }

namespace typed {

struct Bail { const char* reason; };
[[noreturn]] inline void bail(const char* reason) { throw Bail{reason}; }

using d2 = std::array<double,2>;
using d3 = std::array<double,3>;
using d4 = std::array<double,4>;

inline double f32(double x) { return (double)(float)x; }
inline double add(double a, double b) { return (double)((float)a + (float)b); }
inline double sub(double a, double b) { return (double)((float)a - (float)b); }
inline double mul(double a, double b) { return (double)((float)a * (float)b); }
inline double div(double a, double b) {
    float y = (float)b;
    if (y == 0.0f) bail("ZeroDivisionError: float division by zero");
    return (double)((float)a / y);
}
inline double safe_div(double a, double b) {
    return std::fabs(f32(b)) > f32(4e-19) ? div(a, b) : 0.0;
}
// Python min(a, b)/max(a, b): the first argument wins unless the second is
// strictly smaller/larger (also the NaN rule).
inline double mn(double a, double b) { return b < a ? b : a; }
inline double mx(double a, double b) { return b > a ? b : a; }
inline double py_sqrt(double x) {
    if (x < 0.0) bail("ValueError: math domain error (sqrt)");
    return std::sqrt(x);
}
inline double sqrt32(double x) { return f32(py_sqrt(x)); }   // f32(math.sqrt(x))
inline double py_exp(double x) {
    double r = std::exp(x);
    if (std::isinf(r) && std::isfinite(x)) bail("OverflowError: math range error (exp)");
    return r;
}
// math.pow and float ** float through the engine's UCRT pow. The translated
// engine raises for NaN results; overflow is left to it as well.
inline double py_pow(double x, double y) {
    double r = nt::reference_pow(x, y);
    if (std::isnan(r) || (std::isinf(r) && std::isfinite(x) && std::isfinite(y))) bail("pow outside the finite domain");
    return r;
}
inline int64_t py_trunc(double x) {
    if (!std::isfinite(x) || std::fabs(x) >= 9.0e18) bail("trunc outside the integer domain");
    return (int64_t)std::trunc(x);
}
inline int64_t py_floor(double x) {
    if (!std::isfinite(x) || std::fabs(x) >= 9.0e18) bail("floor outside the integer domain");
    return (int64_t)std::floor(x);
}
inline double py_fmod(double x, double y) {
    double r = std::fmod(x, y);
    if (std::isnan(r)) bail("ValueError: math domain error (fmod)");
    return r;
}
inline bool finite(double x) { return std::isfinite(x); }

// Verified C kernels report failures through wtm_fail.
inline void check_kernel() {
    if (wtm_error_code) {
        const char* text = wtm_error_text;
        wtm_error_code = 0; wtm_error_text = nullptr;
        throw Bail{text ? text : "native kernel failure"};
    }
}
inline v3 fv(const d3& a) { return V3((float)a[0], (float)a[1], (float)a[2]); }
inline q4 fq(const d4& a) { q4 q = {(float)a[0], (float)a[1], (float)a[2], (float)a[3]}; return q; }
inline d3 dv(v3 a) { return {(double)a.x, (double)a.y, (double)a.z}; }
inline d4 dq(q4 a) { return {(double)a.x, (double)a.y, (double)a.z, (double)a.w}; }
template<size_t N> std::array<double,N> f32s(const std::array<double,N>& a) {
    std::array<double,N> out; for (size_t i = 0; i < N; ++i) out[i] = f32(a[i]); return out;
}

// native_atan2f.atan2f (named apart from the C library's atan2f).
inline double atan2n(double y, double x) { float r = atan2f_native((float)y, (float)x); check_kernel(); return r; }
inline d3 rotate(const d4& q, const d3& t) { return dv(rotate_thrust(fq(q), fv(t))); }
inline d3 boresight_of(const d4& q, const d2& angles) {
    float a[2] = {(float)angles[0], (float)angles[1]};
    v3 r = boresight(fq(q), a); check_kernel(); return dv(r);
}
enum { PHASE_PREDICT = NORM_PREDICT, PHASE_CORRECT = NORM_CORRECT, PHASE_COAST = NORM_COAST, PHASE_OTHER = NORM_OTHER };
inline d3 normalize(const d3& v, int phase) { v3 r = seeker_normalize(fv(v), phase); check_kernel(); return dv(r); }
// optical_tracking.pre_slew_body: inverse rotation with the force-adapter order.
inline d3 pre_slew_body(const d4& q, const d3& direction) { return rotate({-q[0], -q[1], -q[2], q[3]}, direction); }

// Python sum(x*x for x in v), starting from the integer zero.
template<size_t N> double sum_squares(const std::array<double,N>& v) {
    double s = 0.0; for (double x : v) s = s + x * x; return s;
}

}  // namespace typed
