// Adapted from CPython 3.11 Modules/mathmodule.c vector_norm.
// Copyright Python Software Foundation; see PYTHON-LICENSE.txt.
// Preserves reference hypot/dist rounding without a Python runtime.
#pragma once
#include <cassert>
#include <cmath>
#include <limits>
static inline double
vector_norm(size_t n, double *vec, double max, int found_nan)
{
    const double T27 = 134217729.0;     /* ldexp(1.0, 27) + 1.0) */
    double x, scale, oldcsum, csum = 1.0, frac1 = 0.0, frac2 = 0.0, frac3 = 0.0;
    double t, hi, lo, h;
    int max_e;
    size_t i;

    if (std::isinf(max)) {
        return max;
    }
    if (found_nan) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    if (max == 0.0 || n <= 1) {
        return max;
    }
    frexp(max, &max_e);
    if (max_e >= -1023) {
        scale = ldexp(1.0, -max_e);
        assert(max * scale >= 0.5);
        assert(max * scale < 1.0);
        for (i=0 ; i < n ; i++) {
            x = vec[i];
            assert(std::isfinite(x) && fabs(x) <= max);

            x *= scale;
            assert(fabs(x) < 1.0);

            t = x * T27;
            hi = t - (t - x);
            lo = x - hi;
            assert(hi + lo == x);

            x = hi * hi;
            assert(x <= 1.0);
            assert(fabs(csum) >= fabs(x));
            oldcsum = csum;
            csum += x;
            frac1 += (oldcsum - csum) + x;

            x = 2.0 * hi * lo;
            assert(fabs(csum) >= fabs(x));
            oldcsum = csum;
            csum += x;
            frac2 += (oldcsum - csum) + x;

            assert(csum + lo * lo == csum);
            frac3 += lo * lo;
        }
        h = sqrt(csum - 1.0 + (frac1 + frac2 + frac3));

        x = h;
        t = x * T27;
        hi = t - (t - x);
        lo = x - hi;
        assert (hi + lo == x);

        x = -hi * hi;
        assert(fabs(csum) >= fabs(x));
        oldcsum = csum;
        csum += x;
        frac1 += (oldcsum - csum) + x;

        x = -2.0 * hi * lo;
        assert(fabs(csum) >= fabs(x));
        oldcsum = csum;
        csum += x;
        frac2 += (oldcsum - csum) + x;

        x = -lo * lo;
        assert(fabs(csum) >= fabs(x));
        oldcsum = csum;
        csum += x;
        frac3 += (oldcsum - csum) + x;

        x = csum - 1.0 + (frac1 + frac2 + frac3);
        return (h + x / (2.0 * h)) / scale;
    }
    /* When max_e < -1023, ldexp(1.0, -max_e) overflows.
       So instead of multiplying by a scale, we just divide by *max*.
    */
    for (i=0 ; i < n ; i++) {
        x = vec[i];
        assert(std::isfinite(x) && fabs(x) <= max);
        x /= max;
        x = x*x;
        assert(x <= 1.0);
        assert(fabs(csum) >= fabs(x));
        oldcsum = csum;
        csum += x;
        frac1 += (oldcsum - csum) + x;
    }
    return max * sqrt(csum - 1.0 + frac1);
}
