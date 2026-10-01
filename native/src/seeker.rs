//! Pinned CRT angle reduction and complete shared seeker filter.
use super::{f, ma as add, md as div, mm as mul, ms as sub, reset_overflow, success};
const TABLE: [f64; 241] = [
    0.06241880999595734,
    0.06630889491982347,
    0.0701969710718705,
    0.07408292254903373,
    0.0779666338315423,
    0.08184798980307655,
    0.08572687577074481,
    0.08960317748487173,
    0.09347678115858946,
    0.09734757348722367,
    0.10121544166746667,
    0.10508027341632953,
    0.10894195698986579,
    0.11280038120165939,
    0.11665543544106935,
    0.12050700969122455,
    0.12435499454676142,
    0.12819928123129812,
    0.13203976161463873,
    0.1358763282297013,
    0.13970887428916362,
    0.14353729370182122,
    0.14736148108865163,
    0.15118133179858004,
    0.15499674192394097,
    0.15880760831563107,
    0.16261382859794857,
    0.16641530118311493,
    0.17021192528547438,
    0.17400360093536768,
    0.17779022899267605,
    0.18157171116003215,
    0.18534794999569476,
    0.18911884892608397,
    0.19288431225797464,
    0.19664424519034499,
    0.2003985538258785,
    0.204147145182117,
    0.207889927202263,
    0.21162680876562975,
    0.21535769969773805,
    0.21908251078005775,
    0.2228011537593945,
    0.22651354135691962,
    0.23021958727684372,
    0.23391920621473342,
    0.23761231386547124,
    0.2412988269308588,
    0.24497866312686414,
    0.24865174119051325,
    0.25231798088642715,
    0.2559773030130055,
    0.2596296294082575,
    0.2632748829552824,
    0.2669129875874004,
    0.27054386829293653,
    0.2741674511196588,
    0.2777836631788732,
    0.2813924326491784,
    0.28499368877988124,
    0.28858736189407735,
    0.29217338339139876,
    0.29575168575043154,
    0.2993222025308074,
    0.30288486837497136,
    0.30643961900963007,
    0.30998639124688343,
    0.31352512298504387,
    0.317055753209147,
    0.320578221991157,
    0.32409247048987166,
    0.3275984409505308,
    0.33109607670413205,
    0.3345853221664589,
    0.33806612283682547,
    0.3415384252965417,
    0.3450021772071051,
    0.348457327308122,
    0.35190382541496473,
    0.3553416224161683,
    0.3587706702705722,
    0.36219092200421216,
    0.3656023317069668,
    0.3690048545289644,
    0.3723984466767542,
    0.3757830654092489,
    0.3791586690334418,
    0.3825252168999051,
    0.38588266939807375,
    0.3892309879513207,
    0.3925701350118286,
    0.3959000740552629,
    0.39922076957525254,
    0.4025321870776825,
    0.40583429307480406,
    0.4091270550791683,
    0.41241044159738727,
    0.4156844221237294,
    0.41894896713355284,
    0.42220404807658357,
    0.42544963737004227,
    0.42868570839162573,
    0.4319122354723482,
    0.4351291938892468,
    0.4383365598579578,
    0.4415343105251667,
    0.4447224239609393,
    0.4479008791509373,
    0.45106965598852344,
    0.4542287352667625,
    0.4573780986703208,
    0.46051772876727104,
    0.4636476090008061,
    0.4667677236808665,
    0.4698780579756869,
    0.4729785979032656,
    0.4760693303227612,
    0.47915024292582253,
    0.4822213242278537,
    0.4852825635592212,
    0.4883339510564055,
    0.4913754776531019,
    0.4944071350712753,
    0.49742891581217225,
    0.500440813147294,
    0.5034428211093364,
    0.5064349344830967,
    0.5094171487963562,
    0.5123894603107376,
    0.5153518660125433,
    0.5183043636035779,
    0.5212469514919582,
    0.5241796287829132,
    0.5271023952695795,
    0.5300152514237931,
    0.5329181983868821,
    0.5358112379604636,
    0.5386943725972466,
    0.5415676053918449,
    0.5444309400716031,
    0.5472843809874369,
    0.550127933104693,
    0.5529616019940282,
    0.5557853938223135,
    0.5585993153435623,
    0.5614033738898894,
    0.5641975773624975,
    0.5669819342227005,
    0.5697564534829784,
    0.5725211446980724,
    0.5752760179561178,
    0.5780210838698195,
    0.5807563535676703,
    0.5834818386852149,
    0.5861975513563605,
    0.588903504204738,
    0.5915997103351114,
    0.5942861833248412,
    0.5969629372154015,
    0.5996299865039514,
    0.6022873461349642,
    0.604935031491914,
    0.6075730583890223,
    0.6102014430630651,
    0.6128202021652412,
    0.615429352753105,
    0.6180289122825617,
    0.6206188985999295,
    0.6231993299340659,
    0.625770224888563,
    0.6283316024340097,
    0.6308834819003218,
    0.6334258829691445,
    0.6359588256663214,
    0.6384823303544375,
    0.640996417725432,
    0.6435011087932844,
    0.6459964248867716,
    0.6484823876423005,
    0.6509590189968124,
    0.6534263411807619,
    0.6558843767111708,
    0.658333148384756,
    0.6607726792711326,
    0.6632029927060932,
    0.665624112284961,
    0.6680360618560202,
    0.6704388655140213,
    0.6728325475937631,
    0.6752171326637498,
    0.6775926455199252,
    0.6799591111794818,
    0.6823165548747481,
    0.6846650020471489,
    0.6870044783412449,
    0.6893350095988457,
    0.6916566218531998,
    0.6939693413232598,
    0.6962731944080235,
    0.6985682076809498,
    0.7008544078844501,
    0.7031318219244537,
    0.705400476865049,
    0.707660399923198,
    0.7099116184635248,
    0.7121541599931787,
    0.7143880521567689,
    0.7166133227313746,
    0.7188299996216244,
    0.7210381108548516,
    0.7232376845763179,
    0.7254287490445107,
    0.7276113326265107,
    0.7297854637934291,
    0.7319511711159166,
    0.7341084832597397,
    0.7362574289814281,
    0.7383980371239895,
    0.7405303366126926,
    0.7426543564509179,
    0.7447701257160751,
    0.7468776735555874,
    0.7489770291829414,
    0.7510682218738023,
    0.7531512809621943,
    0.7552262358367449,
    0.7572931159369924,
    0.7593519507497579,
    0.7614027698055784,
    0.7634456026752018,
    0.7654804789661445,
    0.7675074283193082,
    0.7695264804056582,
    0.7715376649229595,
    0.7735410115925735,
    0.7755365501563116,
    0.7775243103733477,
    0.7795043220171863,
    0.7814766148726883,
    0.7834412187331518,
    0.7853981633974483,
];
fn atan(y: f64, x: f64) -> f64 {
    let (y, x) = (f(y), f(x));
    let (nx, ny) = (x.is_sign_negative(), y.is_sign_negative());
    let (mut ax, mut ay) = (x.abs(), y.abs());
    if ay == 0. {
        return if nx {
            f(std::f64::consts::PI).copysign(y)
        } else {
            y
        };
    }
    if ax == 0. {
        return f(std::f64::consts::FRAC_PI_2).copysign(y);
    }
    let delta = ((ay.to_bits() >> 52) & 2047) as i32 - ((ax.to_bits() >> 52) & 2047) as i32;
    if delta > 26 {
        return f(std::f64::consts::FRAC_PI_2).copysign(y);
    }
    if delta < -13 && !nx {
        return f(y / x);
    }
    if delta < -26 && nx {
        return f(std::f64::consts::PI).copysign(y);
    }
    let swapped = ay > ax;
    if swapped {
        std::mem::swap(&mut ay, &mut ax);
    }
    let ratio = ay / ax;
    let mut angle = if ratio > 0.0625 {
        let index = (ratio * 256. + 0.5).trunc() as usize;
        let residual = (ay * 256. - index as f64 * ax) / (index as f64 * ay + ax * 256.);
        let mut angle = residual + TABLE[index - 16];
        angle -= ((residual * residual) * residual) * 0.33333333333224097;
        angle
    } else if ratio >= 0.0001 {
        let squared = ratio * ratio;
        let coefficient = 0.19999999999393223 - squared * 0.1428571356180717;
        let coefficient = 0.3333333333333317 - coefficient * squared;
        ratio - coefficient * (squared * ratio)
    } else {
        ratio
    };
    if swapped {
        angle = std::f64::consts::FRAC_PI_2 - angle;
    }
    if nx {
        angle = std::f64::consts::PI - angle;
    }
    if ny {
        angle = -angle;
    }
    f(angle)
}
type V = [f64; 3];
fn cross(a: V, b: V) -> V {
    [
        sub(mul(a[1], b[2]), mul(a[2], b[1])),
        sub(mul(a[2], b[0]), mul(a[0], b[2])),
        sub(mul(a[0], b[1]), mul(a[1], b[0])),
    ]
}
fn normalize(v: V, phase: u32) -> V {
    let [x, y, z] = v.map(|a| mul(a, a));
    let squared = match phase {
        0 => add(add(z, y), x),
        1 => add(z, add(y, x)),
        _ => add(add(z, x), y),
    };
    let length = f(squared.sqrt());
    if phase == 2 && length <= 1e-9 {
        return [1., 0., 0.];
    }
    let reciprocal = if length > f(4e-19) {
        div(1., length)
    } else {
        0.
    };
    v.map(|a| mul(a, reciprocal))
}
fn slew(p: &[f64], desired: V, angles: [f64; 2], dt: f64, lock: bool, authored: bool) -> [f64; 2] {
    let [x, y, z] = desired;
    let targets = [atan(-z, x), atan(y, f(add(mul(z, z), mul(x, x)).sqrt()))];
    let step = mul(if authored { p[2] } else { 10. }, dt);
    let limit = if lock { p[1] } else { p[0] };
    std::array::from_fn(|i| {
        let previous = angles[i];
        let change = sub(targets[i], previous).max(-step).min(step);
        let change = change.max(sub(-limit, previous)).min(sub(limit, previous));
        add(change, previous)
    })
}
/// 25 doubles: six properties, quaternion, measurement, angles, direction,
/// angular rate, dt, lock/authored/coast switches. Output: 2+3+3, acceptance.
#[no_mangle]
pub unsafe extern "C" fn wt_seeker(input: *const f64, out: *mut f64) -> u32 {
    if input.is_null() || out.is_null() {
        return 0;
    }
    reset_overflow();
    let p = std::slice::from_raw_parts(input, 25);
    let dt = f(p[21]);
    if dt <= 0. {
        return 0;
    }
    let q = std::array::from_fn::<_, 4, _>(|i| f(p[6 + i]));
    let measurement = std::array::from_fn::<_, 3, _>(|i| f(p[10 + i]));
    let angles = [f(p[13]), f(p[14])];
    let u = std::array::from_fn::<_, 3, _>(|i| f(p[15 + i]));
    let mut omega = std::array::from_fn::<_, 3, _>(|i| f(p[18 + i]));
    let coast = p[24] != 0.;
    let lock = p[22] != 0.;
    let authored = p[23] != 0.;
    let v = cross(u, omega);
    let predicted = normalize(
        std::array::from_fn(|i| add(mul(v[i], dt), u[i])),
        if coast { 2 } else { 0 },
    );
    let mut vector_input = [0.; 10];
    vector_input[..4].copy_from_slice(&q);
    let mut transformed = [0.; 3];
    let valid = success();
    let desired = if coast {
        vector_input[4..7].copy_from_slice(&predicted);
        if super::aero::wt_vector(vector_input.as_ptr(), 2, transformed.as_mut_ptr()) == 0 {
            return 0;
        }
        transformed
    } else {
        measurement
    };
    let angles = slew(p, desired, angles, dt, lock, authored);
    let mut direction = predicted;
    let mut accepted = -1.;
    if !coast {
        vector_input[4..7].copy_from_slice(&desired);
        vector_input[7..].copy_from_slice(&predicted);
        let valid2 = success();
        if super::aero::wt_vector(vector_input.as_ptr(), 1, transformed.as_mut_ptr()) == 0
            || valid2 == 0
        {
            return 0;
        }
        let mut error = cross(transformed, predicted);
        let gate = mul(p[5], dt);
        let accept = error.iter().all(|a| a.abs() <= gate);
        accepted = if accept { 1. } else { 0. };
        if accept {
            let step = mul(if authored { p[2] } else { 10. }, dt);
            error = error.map(|a| a.max(-step).min(step));
            let correction = cross(predicted, error);
            direction = normalize(
                std::array::from_fn(|i| add(mul(correction[i], p[3]), predicted[i])),
                1,
            );
            let gain = div(p[4], dt);
            omega = std::array::from_fn(|i| add(mul(error[i], gain), omega[i]).max(-10.).min(10.));
        }
    }
    let r = std::slice::from_raw_parts_mut(out, 9);
    r[..2].copy_from_slice(&angles);
    r[2..5].copy_from_slice(&direction);
    r[5..8].copy_from_slice(&omega);
    r[8] = accepted;
    valid & success() & u32::from(r.iter().all(|x| x.is_finite()))
}
#[no_mangle]
pub unsafe extern "C" fn wt_slew(input: *const f64, out: *mut f64) -> u32 {
    if input.is_null() || out.is_null() {
        return 0;
    }
    reset_overflow();
    let p = std::slice::from_raw_parts(input, 14);
    let r = slew(
        p,
        [p[6], p[7], p[8]],
        [p[9], p[10]],
        p[11],
        p[12] != 0.,
        p[13] != 0.,
    );
    std::slice::from_raw_parts_mut(out, 2).copy_from_slice(&r);
    success() & u32::from(r.iter().all(|x| x.is_finite()))
}
