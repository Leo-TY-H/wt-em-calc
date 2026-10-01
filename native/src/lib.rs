//! Aerodynamic polar kernels. Every helper rounds at the same boundary as Python.
//! Double intermediates are intentional: replacing them with f32 arithmetic can
//! introduce double-rounding differences from the reference implementation.
use std::cell::Cell;
mod aero;
mod python;
mod seeker;
thread_local! {static OVERFLOW: Cell<bool> = Cell::new(false);}
#[inline]
fn f(x: f64) -> f64 {
    let rounded = x as f32;
    if rounded.is_infinite() && x.is_finite() {
        OVERFLOW.with(|flag| flag.set(true));
    }
    rounded as f64
}
fn reset_overflow() {
    OVERFLOW.with(|flag| flag.set(false));
}
fn success() -> u32 {
    OVERFLOW.with(|flag| u32::from(!flag.get()))
}
#[inline]
fn add(a: f64, b: f64) -> f64 {
    f(a + b)
}
#[inline]
fn sub(a: f64, b: f64) -> f64 {
    f(a - b)
}
#[inline]
fn mul(a: f64, b: f64) -> f64 {
    f(a * b)
}
// ABI order matches polar_model.FIELDS; values retain their original f64 values.
#[inline(always)]
fn polar_f<const CHECK: bool>(x: f64) -> f64 {
    if CHECK {
        f(x)
    } else {
        (x as f32) as f64
    }
}
#[inline(always)]
fn polar_add<const CHECK: bool>(a: f64, b: f64) -> f64 {
    polar_f::<CHECK>(a + b)
}
#[inline(always)]
fn polar_sub<const CHECK: bool>(a: f64, b: f64) -> f64 {
    polar_f::<CHECK>(a - b)
}
#[inline(always)]
fn polar_mul<const CHECK: bool>(a: f64, b: f64) -> f64 {
    polar_f::<CHECK>(a * b)
}
#[inline(always)]
fn polar_div<const CHECK: bool>(a: f64, b: f64) -> f64 {
    polar_f::<CHECK>(a / b)
}
#[inline(always)]
fn polar_sin<const CHECK: bool>(a: f64) -> f64 {
    polar_f::<CHECK>(a.sin())
}
#[inline(always)]
fn cl<const CHECK: bool>(p: &[f64], angle: f64) -> f64 {
    let a = polar_f::<CHECK>(angle);
    if p[9] <= a && a <= p[8] {
        return polar_add::<CHECK>(polar_mul::<CHECK>(polar_mul::<CHECK>(a, p[3]), p[23]), p[0]);
    }
    let positive = polar_add::<CHECK>(a, polar_f::<CHECK>(0.01)) >= p[8];
    let s = if positive { 1. } else { -1. };
    let crit = p[if positive { 6 } else { 7 }];
    let cy = p[if positive { 4 } else { 5 }];
    let after = p[if positive { 20 } else { 19 }];
    let da = polar_sub::<CHECK>(a, crit);
    if polar_mul::<CHECK>(da, s) <= 0. {
        let x = polar_sub::<CHECK>(crit, a);
        return polar_sub::<CHECK>(
            cy,
            polar_mul::<CHECK>(
                polar_mul::<CHECK>(polar_mul::<CHECK>(x, x), s),
                p[if positive { 10 } else { 11 }],
            ),
        );
    }
    let maxang = 40_f64.max(p[17]);
    let sa = polar_mul::<CHECK>(s, a);
    if sa <= maxang {
        if sa <= p[17] {
            let pa = p[15];
            if polar_mul::<CHECK>(da, s) < pa {
                return polar_sub::<CHECK>(
                    cy,
                    polar_mul::<CHECK>(polar_mul::<CHECK>(polar_mul::<CHECK>(da, da), s), p[16]),
                );
            }
            let (h, coeff) = if CHECK {
                let h = polar_mul::<CHECK>(
                    if CHECK {
                        polar_sin::<CHECK>(polar_mul::<CHECK>(
                            polar_f::<CHECK>(0.039269909262657166),
                            p[17],
                        ))
                    } else {
                        p[13]
                    },
                    after,
                );
                let x =
                    polar_sub::<CHECK>(polar_mul::<CHECK>(polar_sub::<CHECK>(p[17], pa), s), crit);
                let den = polar_mul::<CHECK>(x, x);
                let coeff = if den > polar_f::<CHECK>(4e-19) {
                    polar_div::<CHECK>(
                        polar_sub::<CHECK>(
                            polar_sub::<CHECK>(cy, h),
                            polar_mul::<CHECK>(
                                polar_mul::<CHECK>(polar_mul::<CHECK>(pa, pa), s),
                                p[16],
                            ),
                        ),
                        den,
                    )
                } else {
                    0.
                };
                (h, coeff)
            } else {
                let side = if positive { 0 } else { 1 };
                (p[24 + side], p[26 + side])
            };
            let x = polar_sub::<CHECK>(polar_mul::<CHECK>(p[17], s), a);
            return polar_add::<CHECK>(polar_mul::<CHECK>(polar_mul::<CHECK>(x, x), coeff), h);
        }
        return polar_mul::<CHECK>(
            after,
            polar_sin::<CHECK>(polar_mul::<CHECK>(
                polar_mul::<CHECK>(a, polar_f::<CHECK>(0.039269909262657166)),
                s,
            )),
        );
    }
    if sa <= 140. {
        let local_sign = if sa > 90. { -s } else { s };
        let local_angle = if sa > 90. {
            polar_sub::<CHECK>(180., sa)
        } else {
            sa
        };
        let correction = if CHECK {
            let one = polar_sin::<CHECK>(polar_mul::<CHECK>(
                polar_f::<CHECK>(0.039269909262657166),
                maxang,
            ));
            let two = polar_sin::<CHECK>(polar_sub::<CHECK>(
                polar_f::<CHECK>(-1.5707963705062866),
                polar_mul::<CHECK>(
                    0_f64.max(polar_add::<CHECK>(maxang, -40.)),
                    polar_f::<CHECK>(0.03141592815518379),
                ),
            ));
            polar_add::<CHECK>(two, one)
        } else {
            p[14]
        };
        let correction = polar_mul::<CHECK>(
            polar_add::<CHECK>(
                polar_mul::<CHECK>(
                    polar_add::<CHECK>(
                        polar_mul::<CHECK>(sa, polar_f::<CHECK>(-0.01)),
                        polar_f::<CHECK>(0.3999999761581421),
                    ),
                    correction,
                ),
                correction,
            ),
            s,
        );
        let wave = polar_mul::<CHECK>(
            polar_sin::<CHECK>(polar_add::<CHECK>(
                polar_mul::<CHECK>(local_angle, polar_f::<CHECK>(0.03141592815518379)),
                polar_f::<CHECK>(0.3141592741012573),
            )),
            local_sign,
        );
        return polar_mul::<CHECK>(
            polar_add::<CHECK>(wave, correction),
            polar_mul::<CHECK>(after, s),
        );
    }
    polar_mul::<CHECK>(
        polar_mul::<CHECK>(polar_mul::<CHECK>(s, s), after),
        polar_sin::<CHECK>(polar_add::<CHECK>(
            polar_mul::<CHECK>(sa, polar_f::<CHECK>(0.039269909262657166)),
            polar_f::<CHECK>(-7.0685834884643555),
        )),
    )
}

#[inline(always)]
fn cd<const CHECK: bool>(p: &[f64], angle: f64) -> f64 {
    let a = polar_f::<CHECK>(angle);
    let line = polar_add::<CHECK>(polar_mul::<CHECK>(p[3], a), p[0]);
    let delta = polar_sub::<CHECK>(a, p[if a >= 0. { 6 } else { 7 }]);
    let delta = if a < 0. { -delta } else { delta };
    let cd = polar_add::<CHECK>(
        polar_add::<CHECK>(
            polar_mul::<CHECK>(polar_mul::<CHECK>(line, line), p[2]),
            p[1],
        ),
        if delta >= 0. {
            polar_mul::<CHECK>(delta, p[18])
        } else {
            0.
        },
    );
    // In the proven finite domain, |sin| is in [0,1]. Monotonic
    // rounding makes this a lower bound on the drag cap. If the uncapped
    // value is already below it, the sine cannot change the result.
    // The checked path still evaluates every original operation so an
    // otherwise unused intermediate overflow remains observable.
    if !CHECK {
        let lower = polar_add::<false>(polar_f::<false>(p[4].min(0.)), polar_f::<false>(0.15));
        if cd <= lower {
            return cd;
        }
    }
    let bound = polar_add::<CHECK>(
        polar_mul::<CHECK>(
            polar_sin::<CHECK>(polar_mul::<CHECK>(a, polar_f::<CHECK>(0.01745329238474369))).abs(),
            p[4],
        ),
        polar_f::<CHECK>(0.15),
    );
    cd.min(bound)
}

#[inline(always)]
fn coefficients<const CHECK: bool>(
    p: &[f64],
    a: f64,
    angle: f64,
    cl_add: f64,
    cd_coeff: f64,
) -> [f64; 2] {
    let cd = polar_mul::<CHECK>(cd::<CHECK>(p, a), polar_f::<CHECK>(cd_coeff));
    let cl = polar_add::<CHECK>(cl::<CHECK>(p, a), polar_f::<CHECK>(cl_add));
    let radians = polar_mul::<CHECK>(
        polar_f::<CHECK>(angle),
        polar_f::<CHECK>(0.01745329238474369),
    );
    let (sn, cs) = radians.sin_cos();
    let sn = polar_f::<CHECK>(sn);
    let cs = polar_f::<CHECK>(cs);
    [
        polar_mul::<CHECK>(
            polar_sub::<CHECK>(polar_mul::<CHECK>(cs, cd), polar_mul::<CHECK>(cl, sn)),
            p[21],
        ),
        polar_mul::<CHECK>(
            polar_add::<CHECK>(polar_mul::<CHECK>(cs, cl), polar_mul::<CHECK>(cd, sn)),
            p[22],
        ),
    ]
}

/// Caller supplies 24 readable doubles and output space for two doubles.
/// mode 0=lift, 1=drag, 2=rotated force coefficients. No pointer is retained.
#[no_mangle]
pub unsafe extern "C" fn wt_polar(
    p: *const f64,
    a: f64,
    angle: f64,
    cl_add: f64,
    cd_coeff: f64,
    mode: u32,
    out: *mut f64,
) -> u32 {
    reset_overflow();
    let p = std::slice::from_raw_parts(p, 24);
    let result = match mode {
        0 => [cl::<true>(p, a), 0.],
        1 => [cd::<true>(p, a), 0.],
        _ => coefficients::<true>(p, a, angle, cl_add, cd_coeff),
    };
    std::ptr::copy_nonoverlapping(result.as_ptr(), out, 2);
    success()
}

/// One FFI crossing for an entire angle sweep. Each row is [a, rotation,
/// added lift, drag multiplier]; outputs are consecutive coefficient pairs.
#[no_mangle]
pub unsafe extern "C" fn wt_polar_batch(
    p: *const f64,
    inputs: *const f64,
    count: usize,
    out: *mut f64,
) -> u32 {
    reset_overflow();
    let p = std::slice::from_raw_parts(p, 24);
    for i in 0..count {
        let row = std::slice::from_raw_parts(inputs.add(i * 4), 4);
        let result = coefficients::<true>(p, row[0], row[1], row[2], row[3]);
        std::ptr::copy_nonoverlapping(result.as_ptr(), out.add(i * 2), 2);
    }
    success()
}

/// Eight force vectors ordered as component_assembly.NAMES, then parasite.
#[no_mangle]
pub unsafe extern "C" fn wt_force(input: *const f64, out: *mut f64) -> u32 {
    reset_overflow();
    let v = std::slice::from_raw_parts(input, 24);
    let result = force_values::<true>(v);
    std::ptr::copy_nonoverlapping(result.as_ptr(), out, 3);
    success()
}

#[inline(always)]
fn force_values<const CHECK: bool>(v: &[f64]) -> [f64; 3] {
    let mut q = [0.; 24];
    for i in 0..24 {
        q[i] = polar_f::<CHECK>(v[i]);
    }
    let [l, r, h, j, v, b, c, p] = std::array::from_fn::<_, 8, _>(|i| &q[i * 3..i * 3 + 3]);
    [
        polar_add::<CHECK>(
            polar_add::<CHECK>(
                polar_add::<CHECK>(
                    v[0],
                    polar_add::<CHECK>(
                        j[0],
                        polar_add::<CHECK>(
                            h[0],
                            polar_add::<CHECK>(polar_add::<CHECK>(l[0], c[0]), r[0]),
                        ),
                    ),
                ),
                b[0],
            ),
            p[0],
        ),
        polar_add::<CHECK>(
            polar_add::<CHECK>(
                polar_add::<CHECK>(
                    polar_add::<CHECK>(
                        polar_add::<CHECK>(polar_add::<CHECK>(l[1], c[1]), r[1]),
                        j[1],
                    ),
                    polar_add::<CHECK>(v[1], h[1]),
                ),
                p[1],
            ),
            b[1],
        ),
        polar_add::<CHECK>(
            polar_add::<CHECK>(
                polar_add::<CHECK>(
                    b[2],
                    polar_add::<CHECK>(p[2], polar_add::<CHECK>(r[2], l[2])),
                ),
                c[2],
            ),
            v[2],
        ),
    ]
}
pub(crate) fn bounded_force(v: &[f64]) -> Option<[f64; 3]> {
    // At most eight terms per axis: finite inputs <=1e30 cannot overflow f32.
    if v.iter().all(|x| x.abs() <= 1e30) {
        Some(force_values::<false>(v))
    } else {
        None
    }
}

/// Seven forces, seven positions and a center of gravity (45 doubles).
#[no_mangle]
pub unsafe extern "C" fn wt_moment(input: *const f64, out: *mut f64) -> u32 {
    reset_overflow();
    let data = std::slice::from_raw_parts(input, 45);
    let mut forces = [0.; 21];
    let mut arms = [0.; 21];
    for i in 0..21 {
        forces[i] = f(data[i]);
        arms[i] = sub(f(data[21 + i]), f(data[42 + i % 3]));
    }
    let term = |n: usize, fi: usize, ri: usize| mul(forces[n * 3 + fi], arms[n * 3 + ri]);
    let paired = |fi, ri| {
        let wings = add(term(1, fi, ri), term(0, fi, ri));
        let tails = add(term(2, fi, ri), term(3, fi, ri));
        let main = add(wings, tails);
        let extra = add(term(5, fi, ri), term(4, fi, ri));
        add(add(main, extra), term(6, fi, ri))
    };
    let pos_w = add(term(1, 1, 2), term(0, 1, 2));
    let pos_h = add(term(2, 1, 2), term(3, 1, 2));
    let pos = add(
        add(term(6, 1, 2), add(term(5, 1, 2), term(4, 1, 2))),
        add(pos_h, pos_w),
    );
    let neg_w = add(term(1, 2, 1), term(0, 2, 1));
    let neg_h = add(term(2, 2, 1), term(3, 2, 1));
    let neg = add(
        term(6, 2, 1),
        add(add(term(5, 2, 1), term(4, 2, 1)), add(neg_h, neg_w)),
    );
    let result = [
        sub(pos, neg),
        sub(paired(2, 0), paired(0, 2)),
        sub(paired(0, 1), paired(1, 0)),
    ];
    std::ptr::copy_nonoverlapping(result.as_ptr(), out, 3);
    success()
}

#[no_mangle]
pub extern "C" fn wt_numeric_abi() -> u32 {
    2
}

#[no_mangle]
pub unsafe extern "C" fn wt_atmosphere(height: f64, out: *mut f64) {
    // Missile helpers round both operands before arithmetic, unlike EM helpers.
    let add = |a, b| f(f(a) + f(b));
    let mul = |a, b| f(f(a) * f(b));
    let div = |a, b| f(f(a) / f(b));
    let h = f(height);
    let z = h.min(f(18300.));
    let poly = |coefficients: &[f64]| {
        let mut value = f(coefficients[0]);
        for c in &coefficients[1..] {
            value = add(mul(value, z), *c);
        }
        value
    };
    let density = div(
        mul(
            mul(1.225, 18300.),
            poly(&[2.28719e-19, -5.83556e-14, 3.53118e-9, -9.59387e-5, 1.]),
        ),
        h.max(18300.),
    );
    let sound = mul(
        f(mul(
            poly(&[3.97306e-18, -5.71104e-14, 2.18069e-10, -2.27712e-5, 1.]),
            288.16,
        )
        .sqrt()),
        20.1,
    );
    let pressure = div(
        mul(
            mul(101300., 18300.),
            poly(&[1.60373e-18, -1.3738e-13, 5.6763e-9, -1.18441e-4, 1.]),
        ),
        h.max(18300.),
    );
    let result = [density, sound, pressure];
    std::ptr::copy_nonoverlapping(result.as_ptr(), out, 3);
}

#[inline]
fn ma(a: f64, b: f64) -> f64 {
    f(f(a) + f(b))
}
#[inline]
fn ms(a: f64, b: f64) -> f64 {
    f(f(a) - f(b))
}
#[inline]
fn mm(a: f64, b: f64) -> f64 {
    f(f(a) * f(b))
}
#[inline]
fn md(a: f64, b: f64) -> f64 {
    f(f(a) / f(b))
}

fn half_angle(increment: f64) -> Option<[f64; 4]> {
    let h = mm(-f(increment), 0.5);
    let quadrant_input = ms(0.5_f64.copysign(h), mm(increment, 0.31830987334251404));
    let quadrant = quadrant_input.trunc();
    if !quadrant.is_finite() || !(-2147483648. ..2147483648.).contains(&quadrant) {
        return None;
    }
    let k = quadrant as i64;
    let quadrant = k as f64; // Python math.trunc produces integer zero, never -0.
    let t = ma(mm(f(quadrant), -1.5707963705062866), h);
    let t2 = mm(t, t);
    let mut c = ma(
        mm(
            ma(
                mm(ma(mm(-0.0013602249091491103, t2), 0.04165669530630112), t2),
                -0.4999990165233612,
            ),
            t2,
        ),
        1.,
    );
    let mut s = ma(
        mm(
            ma(
                mm(ma(mm(-0.0001950727018993348, t2), 0.00833207555115223), t2),
                -0.16666652262210846,
            ),
            mm(t2, t),
        ),
        t,
    );
    if k & 1 != 0 {
        std::mem::swap(&mut s, &mut c);
    }
    if k & 2 != 0 {
        s = -s;
    }
    if (k + 1) & 2 != 0 {
        c = -c;
    }
    Some([s, c, quadrant, t])
}

/// Quaternion and Euler increment -> trig records, delta, raw and normalized
/// quaternion. Returns 0 outside the reference's audited conversion domain.
#[no_mangle]
pub unsafe extern "C" fn wt_orientation(input: *const f64, out: *mut f64) -> u32 {
    let v = std::slice::from_raw_parts(input, 7);
    let mut trig = [[0.; 4]; 3];
    for i in 0..3 {
        match half_angle(v[4 + i]) {
            Some(t) => trig[i] = t,
            None => return 0,
        }
    }
    let [sx, sy, sz] = std::array::from_fn::<_, 3, _>(|i| trig[i][0]);
    let [cx, cy, cz] = std::array::from_fn::<_, 3, _>(|i| trig[i][1]);
    let a = [
        mm(mm(sy, cx), sz),
        mm(mm(sy, cx), cz),
        mm(mm(cy, cx), sz),
        mm(mm(cy, cx), cz),
    ];
    let b = [
        mm(mm(cy, sx), cz),
        mm(mm(cy, sx), sz),
        mm(mm(sy, sx), cz),
        mm(mm(sy, sx), sz),
    ];
    let [dx, dy, dz, dw] = [
        ma(b[0], a[0]),
        ma(b[1], a[1]),
        ms(a[2], b[2]),
        ms(a[3], b[3]),
    ];
    let [x, y, z, w] = std::array::from_fn::<_, 4, _>(|i| f(v[i]));
    let raw = [
        ms(ma(mm(dz, y), ma(mm(dx, w), mm(dw, x))), mm(dy, z)),
        ms(ma(mm(dx, z), ma(mm(dw, y), mm(dy, w))), mm(dz, x)),
        ms(ma(mm(dy, x), ma(mm(dw, z), mm(dz, w))), mm(dx, y)),
        ms(mm(dw, w), ma(mm(dx, x), ma(mm(dz, z), mm(dy, y)))),
    ];
    let norm2 = ma(
        ma(mm(raw[3], raw[3]), mm(raw[2], raw[2])),
        ma(mm(raw[1], raw[1]), mm(raw[0], raw[0])),
    );
    let q = if norm2 != 0. {
        raw.map(|v| mm(v, md(1., f(norm2.sqrt()))))
    } else {
        [0.; 4]
    };
    for (i, t) in trig.iter().enumerate() {
        std::ptr::copy_nonoverlapping(t.as_ptr(), out.add(i * 4), 4);
    }
    for (i, row) in [[dx, dy, dz, dw], raw, q].iter().enumerate() {
        std::ptr::copy_nonoverlapping(row.as_ptr(), out.add(12 + i * 4), 4);
    }
    1
}

/// Conservative bound for a check-free binary32 polar specialization. With
/// |a|/|rotation| <=180, |parameters| <=1000, |maxDistAng| <=180,
/// |added lift|/|drag multiplier| <=1000 and |force scales| <=1e5:
/// the largest coefficient is bounded by 3.3e32 (the sole division uses a
/// denominator >4e-19); rotated/scaled results stay below 3.3e37 < f32::MAX.
/// All original rounding boundaries remain. Inputs outside this proof use the
/// checked implementation, including overflow/fallback cases.
pub(crate) fn prepare_polar_profile(p: &mut [f64]) {
    p[13] = sin_prepared(mul(0.039269909262657166, p[17]));
    let maxang = 40_f64.max(p[17]);
    let one = sin_prepared(mul(0.039269909262657166, maxang));
    let two = sin_prepared(sub(
        -1.5707963705062866,
        mul(0_f64.max(add(maxang, -40.)), 0.03141592815518379),
    ));
    p[14] = add(two, one);
    for side in 0..2 {
        let sign = if side == 0 { 1. } else { -1. };
        let cy = p[4 + side];
        let crit = p[6 + side];
        let after = p[if side == 0 { 20 } else { 19 }];
        let pa = p[15];
        let h = mul(p[13], after);
        let x = sub(mul(sub(p[17], pa), sign), crit);
        let den = mul(x, x);
        let coeff = if den > f(4e-19) {
            f(sub(sub(cy, h), mul(mul(mul(pa, pa), sign), p[16])) / den)
        } else {
            0.
        };
        p[24 + side] = h;
        p[26 + side] = coeff;
    }
}
#[inline]
fn sin_prepared(v: f64) -> f64 {
    f(v.sin())
}
pub(crate) fn bounded_polar_profile(p: &[f64]) -> bool {
    p[..24].iter().enumerate().all(|(i, v)| match i {
        12..=14 => true,
        21 | 22 => v.abs() <= 1e5,
        _ => v.abs() <= 1000.,
    }) && p[17].abs() <= 180.
}
#[inline(always)]
pub(crate) fn bounded_polar(
    p: &[f64],
    a: f64,
    angle: f64,
    cl_add: f64,
    cd_coeff: f64,
    mode: u32,
) -> Option<[f64; 2]> {
    if !(a.abs() <= 180. && angle.abs() <= 180. && cl_add.abs() <= 1000. && cd_coeff.abs() <= 1000.)
    {
        return None;
    }
    Some(match mode {
        0 => [cl::<false>(p, a), 0.],
        1 => [cd::<false>(p, a), 0.],
        _ => coefficients::<false>(p, a, angle, cl_add, cd_coeff),
    })
}
