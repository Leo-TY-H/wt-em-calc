//! Complete pure-air missile force block. Preserve every recovered sum order.
use super::{f, ma as add, md as div, mm as mul, ms as sub, reset_overflow, success};
type V = [f64; 3];
fn dot(a: V, b: V) -> f64 {
    add(add(mul(a[0], b[0]), mul(a[1], b[1])), mul(a[2], b[2]))
}
fn cross(a: V, b: V) -> V {
    [
        sub(mul(a[1], b[2]), mul(a[2], b[1])),
        sub(mul(a[2], b[0]), mul(a[0], b[2])),
        sub(mul(a[0], b[1]), mul(a[1], b[0])),
    ]
}
fn scale(a: V, s: f64) -> V {
    a.map(|x| mul(x, s))
}
fn plus(a: V, b: V) -> V {
    std::array::from_fn(|i| add(a[i], b[i]))
}
fn unit(a: V, eps: f64, fallback: V) -> V {
    let n = f(dot(a, a).sqrt());
    if n > eps {
        scale(a, div(1., n))
    } else {
        fallback
    }
}
fn quaternion(q: [f64; 4], p: [f64; 4]) -> [f64; 4] {
    let [x, y, z, w] = q;
    let [a, b, c, d] = p;
    [
        add(sub(add(mul(x, d), mul(a, w)), mul(b, z)), mul(c, y)),
        sub(add(add(mul(y, d), mul(b, w)), mul(a, z)), mul(c, x)),
        add(sub(mul(b, x), mul(a, y)), add(mul(z, d), mul(c, w))),
        sub(mul(d, w), add(mul(c, z), add(mul(b, y), mul(a, x)))),
    ]
}
fn columns(q: [f64; 4]) -> [V; 3] {
    let [x, y, z, w] = q;
    let twice = |a| add(a, a);
    [
        [
            sub(twice(add(mul(x, x), mul(w, w))), 1.),
            twice(add(mul(x, y), mul(z, w))),
            twice(sub(mul(x, z), mul(y, w))),
        ],
        [
            twice(sub(mul(x, y), mul(z, w))),
            sub(twice(add(mul(y, y), mul(w, w))), 1.),
            twice(add(mul(y, z), mul(x, w))),
        ],
        [
            twice(add(mul(x, z), mul(y, w))),
            twice(sub(mul(y, z), mul(x, w))),
            sub(twice(add(mul(z, z), mul(w, w))), 1.),
        ],
    ]
}
fn drag_mach(mach: f64) -> f64 {
    let m = f(mach);
    if m < f(0.61) {
        f(0.308)
    } else if m < 1. {
        add(mul(0.505, f(sub(m, 0.61).powf(f(2.31)))), 0.308)
    } else if m < f(1.4) {
        let d = sub(m, 1.);
        add(
            mul(mul(0.4485, f(d.powf(f(0.505)))), f(mul(d, -5.68).exp())),
            0.551,
        )
    } else if m < 4. {
        div(m, add(mul(add(mul(0.356, m), 2.237), m), -1.4))
    } else {
        f(0.302)
    }
}
fn table_value(rows: &[f64], x: f64) -> f64 {
    if rows.is_empty() {
        return 1.;
    }
    if x <= rows[0] {
        return rows[2];
    }
    let end = rows.len() - 3;
    if x >= rows[end] {
        return rows[end + 2];
    }
    for i in (0..end).step_by(3) {
        if x <= rows[i + 3] {
            return add(
                rows[i + 2],
                mul(
                    sub(rows[i + 5], rows[i + 2]),
                    mul(sub(x, rows[i]), rows[i + 1]),
                ),
            );
        }
    }
    f64::NAN
}
#[derive(Clone, Copy)]
struct Force {
    drag: V,
    lift: V,
    force: V,
    cosine: f64,
    cd: f64,
    cy: f64,
}
fn evaluate(
    direction: V,
    forward: V,
    cy: f64,
    cx_mach: f64,
    ki: f64,
    pressure: f64,
    p: &[f64],
) -> Force {
    let cosine = dot(direction, forward);
    let sin2 = sub(1., mul(cosine, cosine)).max(0.);
    let mut lift_coefficient = mul(f(sin2.sqrt()), cy);
    let cap = p[4];
    if lift_coefficient > cap {
        lift_coefficient = sub(add(cap, cap), lift_coefficient).max(0.);
    } else if lift_coefficient < -cap {
        lift_coefficient = sub(mul(-2., cap), lift_coefficient).min(0.);
    }
    let cd = add(cx_mach, mul(mul(sin2, mul(p[1], p[2])), ki));
    let drag = scale(direction, mul(cd, mul(p[5], -pressure)));
    let lift_direction = unit(cross(direction, cross(direction, forward)), 1e-9, [0.; 3]);
    let mut lift_scalar = mul(lift_coefficient, mul(p[6], -pressure));
    if cosine < 0. {
        lift_scalar = -lift_scalar;
    }
    let lift = scale(lift_direction, lift_scalar);
    Force {
        drag,
        lift,
        force: plus(drag, lift),
        cosine,
        cd,
        cy: lift_coefficient,
    }
}
fn force_row(a: Force) -> [f64; 12] {
    [
        a.drag[0], a.drag[1], a.drag[2], a.lift[0], a.lift[1], a.lift[2], a.force[0], a.force[1],
        a.force[2], a.cosine, a.cd, a.cy,
    ]
}

/// Fixed layouts documented by rust_backend.aero_function. Caller owns 22
/// property doubles, 34 inputs, rows*3 table doubles and 85 output doubles.
/// No pointer is retained; invalid/nonfinite arithmetic requests fall back.
/// Properties: 12 geometry scalars, axis quaternion, inertia, angular damping.
/// Inputs: height, velocity, quaternion, omega, wind, fins, drag/lever additions,
/// dt, torque, force, mass loss, gravity/Cxi switches, mass term, environment,
/// perturbation amplitude and random sample. Outputs follow aero_function's map.
/// Caller provides valid buffers of 22, 34, rows*3, and 85 doubles respectively.
/// No pointers are retained. Return zero requests the reference error path.
#[no_mangle]
pub unsafe extern "C" fn wt_aero(
    properties: *const f64,
    input: *const f64,
    table: *const f64,
    rows: usize,
    out: *mut f64,
) -> u32 {
    if properties.is_null()
        || input.is_null()
        || table.is_null()
        || out.is_null()
        || rows.checked_mul(3).is_none()
    {
        return 0;
    }
    reset_overflow();
    let p = std::slice::from_raw_parts(properties, 22);
    let v = std::slice::from_raw_parts(input, 34);
    let table = std::slice::from_raw_parts(table, rows * 3);
    let height = v[0];
    let vector = |start: usize| std::array::from_fn::<_, 3, _>(|i| f(v[start + i]));
    let velocity = vector(1);
    let q = std::array::from_fn::<_, 4, _>(|i| f(v[4 + i]));
    let omega = vector(8);
    let wind = vector(11);
    let fins = [f(v[14]), f(v[15])];
    let torque = vector(19);
    let force = vector(22);
    let relative = std::array::from_fn::<_, 3, _>(|i| sub(velocity[i], wind[i]));
    let cm_speed64 = ((relative[0] * relative[0] + relative[1] * relative[1])
        + relative[2] * relative[2])
        .sqrt();
    let speed = f(cm_speed64);
    let composed = quaternion(q, std::array::from_fn(|i| p[12 + i]));
    let mut frame = columns(composed);
    let mut axes = [[0.; 3]; 3];
    for (i, order) in [[2, 1, 0], [2, 0, 1], [0, 1, 2]].iter().enumerate() {
        let [a, b, c] = order.map(|j| mul(frame[i][j], frame[i][j]));
        let n = f(add(add(a, b), c).sqrt());
        axes[i] = if n > f(4e-19) {
            scale(frame[i], div(1., n))
        } else {
            [0.; 3]
        };
    }
    let ww = sub(mul(2., mul(composed[3], composed[3])), 1.);
    for i in 0..3 {
        frame[i][i] = add(mul(2., mul(composed[i], composed[i])), ww);
    }
    let forward = axes[0];
    let fallback = if speed > f(0.001) {
        relative.map(|x| f(x / cm_speed64))
    } else {
        forward
    };
    let arm = add(p[0], v[17]);
    let amplitude = f(v[32]);
    let random = f(v[33]);
    let perturb = if amplitude <= 0. {
        [1., 0., 1., 0., 0.]
    } else {
        let scale = if amplitude < 2. {
            mul(0.01, amplitude)
        } else {
            f(0.02)
        };
        let angle = mul(
            if amplitude < 2. {
                mul(f(std::f64::consts::PI / 4.), amplitude)
            } else {
                f(std::f64::consts::PI / 2.)
            },
            random,
        );
        [
            add(mul(scale, random), 1.),
            angle,
            f(angle.cos()),
            f(angle.sin()),
            if amplitude < f(0.5) {
                mul(amplitude, 1.6)
            } else {
                f(0.8)
            },
        ]
    };
    let offset = mul(mul(perturb[4], v[33]), p[0]);
    let cosine_offset = mul(perturb[2], offset);
    let sine_offset = mul(offset, perturb[3]);
    let bx = sub(mul(omega[2], sine_offset), mul(omega[1], cosine_offset));
    let by = add(mul(omega[2], arm), mul(omega[0], cosine_offset));
    let bz = sub(mul(omega[1], -arm), mul(omega[0], sine_offset));
    let local = [
        add(add(mul(frame[2][0], bz), relative[0]), mul(frame[1][0], by)),
        add(add(mul(frame[2][1], bz), relative[1]), mul(frame[1][1], by)),
        add(add(relative[2], mul(frame[1][2], by)), mul(frame[2][2], bz)),
    ];
    let local = std::array::from_fn(|i| add(local[i], mul(frame[0][i], bx)));
    let flow = unit(local, 1e-9, fallback);
    let mut air = [0.; 3];
    super::wt_atmosphere(height, air.as_mut_ptr());
    let mach = div(speed, air[1]);
    let z = f(height).min(18300.);
    let mut poly = f(2.28719e-19);
    for coefficient in [-5.83556e-14, 3.53118e-9, -9.59387e-5, 1.] {
        poly = add(mul(poly, z), coefficient);
    }
    let pressure = div(
        mul(mul(speed, speed), mul(mul(mul(1.225, 0.5), 18300.), poly)),
        f(height).max(18300.),
    );
    let k = drag_mach(mach);
    let ki = if v[27] != 0. {
        add(mul(k, 2.4352500438690186), 0.25001898407936096)
    } else {
        1.
    };
    let cx_total = add(p[1], v[16]);
    let cx_mach = if f(1.4) <= mach && mach < 4. {
        div(
            mul(mach, cx_total),
            add(mul(add(mul(0.356, mach), 2.237), mach), -1.4),
        )
    } else {
        mul(cx_total, k)
    };
    let cy = mul(p[3], table_value(table, mach));
    let baseline = evaluate(flow, forward, cy, cx_mach, ki, pressure, p);
    let mut deflection = [mul(fins[0], p[7]), mul(fins[1], p[8])];
    let active = mul(fins[0].abs(), p[7]) > f(1e-5) || mul(fins[1].abs(), p[8]) > f(1e-5);
    let mut limited = false;
    let mut fin_flow = flow;
    let mut steering = baseline;
    if active {
        let size2 = add(
            mul(deflection[0], deflection[0]),
            mul(deflection[1], deflection[1]),
        );
        let limit = p[9];
        if mul(mul(pressure, pressure), size2) > mul(limit, limit) {
            let factor = div(1., mul(pressure, f(size2.sqrt())));
            deflection = deflection.map(|x| mul(mul(x, limit), factor));
            limited = true;
        }
        fin_flow = unit(
            plus(
                plus(flow, scale(axes[2], deflection[0])),
                scale(axes[1], deflection[1]),
            ),
            1e-9,
            fallback,
        );
        steering = evaluate(fin_flow, forward, cy, cx_mach, ki, pressure, p);
    }
    let [fx, fy, fz] = frame.map(|c| dot(c, steering.force));
    let moment = [
        sub(mul(fy, cosine_offset), mul(fz, sine_offset)),
        sub(mul(-arm, fz), mul(fx, cosine_offset)),
        add(mul(arm, fy), mul(fx, sine_offset)),
    ];
    let raw_torque = plus(moment, torque);
    let inertia = std::array::from_fn::<_, 3, _>(|i| p[16 + i]);
    let dt = f(v[18]);
    let qgeom = mul(pressure, p[10]);
    let mut damping = [0.; 3];
    let mut clipped = [0.; 3];
    for i in 0..3 {
        let mut d = mul(
            mul(mul(if i == 0 { -0.01 } else { -0.05 }, omega[i]), qgeom),
            p[19 + i],
        );
        let stopping = if i == 0 {
            add(
                add(mul(mul(omega[i], inertia[i]), div(1., dt)), torque[i]),
                moment[i],
            )
        } else {
            add(mul(mul(omega[i], inertia[i]), div(1., dt)), raw_torque[i])
        };
        if stopping.abs() < d.abs() {
            d = -stopping;
            clipped[i] = 1.;
        }
        damping[i] = d;
    }
    let gyro = [
        mul(mul(omega[2], omega[1]), sub(inertia[2], inertia[1])),
        mul(sub(inertia[0], inertia[2]), mul(omega[2], omega[0])),
        mul(mul(omega[0], omega[1]), sub(inertia[1], inertia[0])),
    ];
    let mut alpha = std::array::from_fn::<_, 3, _>(|i| {
        div(add(add(raw_torque[i], gyro[i]), damping[i]), inertia[i])
    });
    alpha[0] = div(
        add(add(add(gyro[0], torque[0]), moment[0]), damping[0]),
        inertia[0],
    );
    let base_alpha = alpha;
    let [x, y, z, w] = q;
    let [ex, ey, ez] = vector(29);
    let zx = mul(add(z, z), x);
    let yw = mul(add(y, y), w);
    let ey2 = add(ey, ey);
    let ww = sub(add(mul(w, w), mul(w, w)), 1.);
    let plus_x = add(
        mul(add(add(mul(z, z), mul(z, z)), ww), ez),
        add(mul(add(mul(z, y), mul(x, w)), ey2), mul(sub(zx, yw), ex)),
    );
    let plus_z = sub(
        sub(
            mul(sub(mul(z, w), mul(y, x)), ey2),
            mul(add(add(mul(x, x), mul(x, x)), ww), ex),
        ),
        mul(add(yw, zx), ez),
    );
    alpha = [
        add(mul(plus_x, 9.81), alpha[0]),
        alpha[1],
        add(mul(plus_z, 9.81), alpha[2]),
    ];
    let mass = sub(add(mul(p[11], v[28]), p[11]), v[25]);
    let inverse = if f(mass).abs() > f(4e-19) {
        div(1., mass)
    } else {
        0.
    };
    let mut acceleration = std::array::from_fn::<_, 3, _>(|i| {
        ((baseline.drag[i] + baseline.lift[i]) * perturb[0] + force[i]) * inverse
    });
    if v[26] != 0. {
        acceleration[1] -= mul(mass, 9.81) * inverse;
    }
    let n2 = (acceleration[0] * acceleration[0] + acceleration[1] * acceleration[1])
        + acceleration[2] * acceleration[2];
    if n2 >= 6000_f64.powi(2) {
        acceleration = acceleration.map(|x| x * (6000_f64.powi(2) / n2).sqrt());
    }
    let output = std::slice::from_raw_parts_mut(out, 85);
    let mut index = 0;
    let mut append = |row: &[f64]| {
        output[index..index + row.len()].copy_from_slice(row);
        index += row.len();
    };
    for col in frame {
        append(&col);
    }
    for col in axes {
        append(&col);
    }
    append(&[speed, mach, pressure]);
    append(&local);
    append(&flow);
    append(&force_row(baseline));
    append(&[u32::from(active) as f64, u32::from(limited) as f64]);
    append(&deflection);
    append(&fin_flow);
    append(&force_row(steering));
    append(&moment);
    append(&damping);
    append(&clipped);
    append(&base_alpha);
    append(&alpha);
    append(&acceleration);
    append(&[mass]);
    append(&perturb);
    append(&[arm, -sine_offset, -cosine_offset]);
    u32::from(output.iter().all(|v| v.is_finite())) & success()
}

/// Inputs are forward/up/right (nine doubles); output is four doubles.
/// Caller owns valid, non-overlapping buffers for the duration of this call.
#[no_mangle]
pub unsafe extern "C" fn wt_matrix_quaternion(input: *const f64, out: *mut f64) -> u32 {
    if input.is_null() || out.is_null() {
        return 0;
    }
    reset_overflow();
    let v = std::slice::from_raw_parts(input, 9);
    let [fx, fy, fz, ux, uy, uz, rx, ry, rz] = std::array::from_fn::<_, 9, _>(|i| v[i]);
    let tx = add(fx, 1.);
    let tyz = add(rz, uy);
    let squared = [
        sub(tx, tyz),
        sub(add(uy, 1.), add(fx, rz)),
        add(sub(1., add(fx, uy)), rz),
        add(tyz, tx),
    ];
    let magnitudes = squared.map(|v| if v > 0. { mul(f(v.sqrt()), 0.5) } else { 0. });
    let mut best = f(3.4028234663852886e38);
    let mut chosen = [0.; 4];
    for sx in [1., -1.] {
        for sy in [1., -1.] {
            for (sz, sw) in [(1., 1.), (1., -1.), (-1., 1.), (-1., -1.)] {
                let [x, y, z, w] =
                    std::array::from_fn::<_, 4, _>(|i| mul(magnitudes[i], [sx, sy, sz, sw][i]));
                let xy = mul(x, y);
                let wz = mul(w, z);
                let wy = mul(w, y);
                let xz = mul(x, z);
                let yz = mul(y, z);
                let wx = mul(w, x);
                let differences = [
                    sub(mul(2., sub(xy, wz)), ux),
                    sub(mul(2., add(wy, xz)), rx),
                    sub(mul(2., add(xy, wz)), fy),
                    sub(mul(2., sub(yz, wx)), ry),
                    sub(mul(2., sub(xz, wy)), fz),
                    sub(mul(2., add(yz, wx)), uz),
                ];
                let [a, b, c, d, e, f] = differences.map(|v| mul(v, v));
                let error = if sz == sw {
                    add(add(add(d, b), add(c, a)), add(f, e))
                } else {
                    add(add(f, e), add(add(d, c), add(b, a)))
                };
                if error < best {
                    chosen = [x, y, z, w];
                    best = error;
                }
            }
        }
    }
    std::slice::from_raw_parts_mut(out, 4).copy_from_slice(&chosen);
    u32::from(chosen.iter().all(|v| v.is_finite())) & success()
}

/// Quaternion (4), vector (3), predicted vector (3). Three distinct sum orders.
/// Mode 0 rotates propulsion, 1 forms seeker residual, 2 transforms coast LOS.
#[no_mangle]
pub unsafe extern "C" fn wt_vector(input: *const f64, mode: u32, out: *mut f64) -> u32 {
    if input.is_null() || out.is_null() || mode > 2 {
        return 0;
    }
    reset_overflow();
    let v = std::slice::from_raw_parts(input, 10);
    let [x, y, z, w] = std::array::from_fn::<_, 4, _>(|i| f(v[i]));
    let [tx, ty, tz] = [v[4], v[5], v[6]];
    let (a, b, c, d, e) = if mode == 2 {
        let nw = mul(-2., w);
        (
            mul(x, add(z, z)),
            mul(add(z, z), y),
            mul(nw, z),
            mul(y, nw),
            mul(x, nw),
        )
    } else {
        (
            mul(add(z, z), x),
            mul(y, add(z, z)),
            mul(add(z, z), w),
            mul(add(w, w), y),
            mul(w, add(x, x)),
        )
    };
    let xy = mul(add(x, x), y);
    let ww = sub(add(mul(w, w), mul(w, w)), 1.);
    let xx = add(add(mul(x, x), mul(x, x)), ww);
    let yy = add(add(mul(y, y), mul(y, y)), ww);
    let zz = add(add(mul(z, z), mul(z, z)), ww);
    let r = match mode {
        0 => [
            add(add(mul(add(d, a), tz), mul(sub(xy, c), ty)), mul(xx, tx)),
            add(add(mul(sub(b, e), tz), mul(add(c, xy), tx)), mul(yy, ty)),
            add(mul(zz, tz), add(mul(add(e, b), ty), mul(sub(a, d), tx))),
        ],
        1 => [
            add(
                sub(add(mul(add(d, a), tz), mul(sub(xy, c), ty)), v[7]),
                mul(xx, tx),
            ),
            add(
                sub(add(mul(sub(b, e), tz), mul(add(c, xy), tx)), v[8]),
                mul(yy, ty),
            ),
            add(
                sub(add(mul(add(e, b), ty), mul(sub(a, d), tx)), v[9]),
                mul(zz, tz),
            ),
        ],
        _ => [
            add(mul(add(d, a), tz), add(mul(sub(xy, c), ty), mul(xx, tx))),
            add(mul(sub(b, e), tz), add(mul(yy, ty), mul(add(xy, c), tx))),
            add(mul(zz, tz), add(mul(add(e, b), ty), mul(sub(a, d), tx))),
        ],
    };
    std::slice::from_raw_parts_mut(out, 3).copy_from_slice(&r);
    u32::from(r.iter().all(|v| v.is_finite())) & success()
}

/// Packed body state (21), accelerations (6), dt/time (2), clock rates (4).
/// Output: updated state (21), displacement/increment (6), orientation (24).
#[no_mangle]
pub unsafe extern "C" fn wt_integrate(input: *const f64, out: *mut f64) -> u32 {
    if input.is_null() || out.is_null() {
        return 0;
    }
    reset_overflow();
    let v = std::slice::from_raw_parts(input, 33);
    let dt = f(v[27]);
    let half = mul(mul(dt, dt), 0.5);
    let displacement =
        std::array::from_fn::<_, 3, _>(|i| add(f(v[21 + i] * half), mul(v[3 + i], dt)));
    let position = std::array::from_fn::<_, 3, _>(|i| add(v[i], displacement[i]));
    let velocity = std::array::from_fn::<_, 3, _>(|i| add(f(v[21 + i] * dt), v[3 + i]));
    let increment =
        std::array::from_fn::<_, 3, _>(|i| add(mul(v[24 + i], half), mul(v[6 + i], dt)));
    let omega = std::array::from_fn::<_, 3, _>(|i| add(mul(v[24 + i], dt), v[6 + i]));
    let mut rotation_input = [0.; 7];
    rotation_input[..4].copy_from_slice(&v[9..13]);
    rotation_input[4..].copy_from_slice(&increment);
    let valid = success();
    let mut rotation = [0.; 24];
    if super::wt_orientation(rotation_input.as_ptr(), rotation.as_mut_ptr()) == 0 {
        return 0;
    }
    let [vx, vy, vz] = velocity;
    let distance_step = mul(
        f(add(add(mul(vz, vz), mul(vy, vy)), mul(vx, vx)).sqrt()),
        dt,
    );
    let elapsed = sub(v[28], v[13]);
    let clocks = std::array::from_fn::<_, 4, _>(|i| add(mul(v[29 + i], elapsed), v[14 + i]));
    let r = std::slice::from_raw_parts_mut(out, 51);
    r[..3].copy_from_slice(&position);
    r[3..6].copy_from_slice(&velocity);
    r[6..9].copy_from_slice(&omega);
    r[9..13].copy_from_slice(&rotation[20..24]);
    r[13] = f(v[28]);
    r[14..18].copy_from_slice(&clocks);
    r[18] = add(v[18], distance_step);
    r[19] = if v[20] != 0. {
        add(distance_step, v[19])
    } else {
        v[19]
    };
    r[20] = 0.;
    r[21..24].copy_from_slice(&displacement);
    r[24..27].copy_from_slice(&increment);
    r[27..].copy_from_slice(&rotation);
    valid & success() & u32::from(r.iter().all(|v| v.is_finite()))
}

/// Acceleration controller after frame/schedule selection. See Python packing.
#[no_mangle]
pub unsafe extern "C" fn wt_controller(
    input: *const f64,
    table: *const f64,
    rows: usize,
    out: *mut f64,
) -> u32 {
    if input.is_null() || table.is_null() || out.is_null() || rows.checked_mul(3).is_none() {
        return 0;
    }
    reset_overflow();
    let p = std::slice::from_raw_parts(input, 50);
    let table = std::slice::from_raw_parts(table, rows * 3);
    let [x, y, z, w] = std::array::from_fn::<_, 4, _>(|i| f(p[i]));
    let hx = mul(2., add(mul(z, x), mul(w, y)));
    let t = mul(add(z, z), y);
    let u = mul(mul(-2., x), w);
    let common = add(mul(2., mul(w, w)), -1.);
    let h = [hx, add(u, t), add(mul(2., mul(z, z)), common)];
    let v = [
        mul(2., sub(mul(x, y), mul(w, z))),
        add(mul(2., mul(y, y)), common),
        sub(t, u),
    ];
    let [rx, ry, rz] = [f(p[4]), f(p[5]), f(p[6])];
    let mut wanted = [
        add(add(mul(h[2], rz), mul(h[0], rx)), mul(h[1], ry)),
        add(mul(v[2], rz), add(mul(v[1], ry), mul(v[0], rx))),
    ];
    let squared = add(mul(wanted[1], wanted[1]), mul(wanted[0], wanted[0]));
    let [vx, vy, vz] = [f(p[10]), f(p[11]), f(p[12])];
    let speed2 = add(mul(vz, vz), add(mul(vy, vy), mul(vx, vx)));
    let height = f(p[13]);
    let ceiling = f(p[19]);
    let altitude = height.min(ceiling);
    let density_poly = || {
        let mut a = f(2.28719e-19);
        for c in [-5.83556e-14, 3.53118e-9, -9.59387e-5, 1.] {
            a = add(mul(a, altitude), c);
        }
        a
    };
    let sound = || {
        let mut a = f(3.97306e-18);
        for c in [-5.71104e-14, 2.18069e-10, -2.27712e-5, 1.] {
            a = add(mul(a, altitude), c);
        }
        mul(f(mul(a, p[22]).sqrt()), 20.1)
    };
    let mut limit = p[15];
    let mut aoa = 0.;
    if p[49] != 0. && p[16] != 0. {
        let mass = sub(p[23], p[28]);
        let cy_mult = if table.is_empty() {
            1.
        } else {
            table_value(table, div(speed2, sound()))
        };
        let mut factor = mul(p[20], 0.5);
        for x in [speed2, p[19], density_poly(), cy_mult, p[24], p[26]] {
            factor = mul(factor, x);
        }
        factor = div(factor, height.max(ceiling));
        aoa = div(mul(add(factor, p[27]), p[17].min(p[25])), mass);
        limit = limit.min(aoa);
    }
    let limit2 = mul(limit, limit);
    if squared > limit2 {
        let reduction = f(div(limit2, squared).sqrt());
        wanted = wanted.map(|x| mul(x, reduction));
    }
    let mut scale2 = 1.;
    if p[18] > 0. {
        let rho = div(
            mul(mul(ceiling, p[20]), density_poly()),
            height.max(ceiling),
        );
        let denom = mul(mul(rho, rho), speed2);
        scale2 = if f(denom).abs() > f(4e-19) {
            div(mul(mul(p[21], p[21]), p[18]), denom)
        } else {
            0.
        };
    }
    let scale = f(scale2.sqrt());
    let [mx, my, mz] = [f(p[7]), f(p[8]), f(p[9])];
    let actual = [
        add(mul(h[1], my), add(mul(h[2], mz), mul(h[0], mx))),
        add(mul(v[2], mz), add(mul(v[1], my), mul(v[0], mx))),
    ];
    let errors = [sub(wanted[0], actual[0]), sub(wanted[1], actual[1])];
    let kp = mul(p[45], scale);
    let ki = mul(p[46], scale);
    let kd = mul(p[47], scale2);
    let ilim = p[48];
    let dt = p[14];
    let r = std::slice::from_raw_parts_mut(out, 25);
    for i in 0..2 {
        let prev = &p[29 + i * 8..37 + i * 8];
        let error = errors[i];
        let integral = add(mul(mul(error, dt), ki), prev[3]).max(-ilim).min(ilim);
        let derivative = add(
            mul(sub(error, add(mul(dt, prev[7]), prev[6])), 48.),
            prev[7],
        );
        let proportional = mul(kp, error);
        let differential = mul(derivative, kd);
        let value = if i == 0 {
            add(add(differential, proportional), integral)
        } else {
            add(add(integral, proportional), differential)
        };
        r[i] = value.max(-1.).min(1.);
        r[2 + i * 8..10 + i * 8]
            .copy_from_slice(&[kp, ki, ilim, integral, kd, 48., error, derivative]);
    }
    r[18..20].copy_from_slice(&errors);
    r[20..22].copy_from_slice(&wanted);
    r[22] = aoa;
    r[23] = scale;
    r[24] = scale2;
    success() & u32::from(r.iter().all(|x| x.is_finite()))
}
