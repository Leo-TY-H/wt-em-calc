//! Direct CPython builtins over the same audited kernels. Public Stable ABI
//! functions provide the portable path. A probed CPython 3.11/3.12 path also reads
//! their documented object layouts; future ABIs never enable these reads.
//! Function addresses are supplied by the running interpreter, so the
//! portable cdylib does not link a particular libpython. Callbacks always retain
//! the GIL. Their self tuple owns all interned keys (no global Python objects).
//! Unsupported inputs return None for the existing reference fallback.
use std::cell::RefCell;
use std::ffi::{c_char, c_void};
use std::ptr;
use std::sync::atomic::{AtomicBool, AtomicPtr, Ordering};
use std::sync::OnceLock;
type O = *mut c_void;
static API: OnceLock<[usize; 28]> = OnceLock::new();
static API_BASE: AtomicPtr<usize> = AtomicPtr::new(ptr::null_mut());
static FAST_LAYOUT: AtomicBool = AtomicBool::new(false);
// Optional CPython 3.11/3.12 fast path, matching their public cpython headers.
// Python probes every layout before enabling it. Other runtimes use Stable ABI.
#[repr(C)]
struct Header {
    references: isize,
    kind: O,
}
#[repr(C)]
struct VarObject {
    header: Header,
    size: isize,
}
#[repr(C)]
struct FloatObject {
    header: Header,
    value: f64,
}
#[repr(C)]
struct ListObject {
    base: VarObject,
    items: *const O,
    allocated: isize,
}
#[repr(C)]
struct TupleObject {
    base: VarObject,
    items: [O; 0],
}
#[repr(C)]
struct DictObject {
    header: Header,
    used: isize,
    version: u64,
}
#[derive(Clone, Copy)]
struct ProfileCache {
    context: usize,
    identity: usize,
    version: u64,
    values: [f64; 28],
}
thread_local! {static PROFILE:RefCell<ProfileCache>=RefCell::new(ProfileCache{context:0,identity:0,version:0,values:[0.;28]});}
#[derive(Clone, Copy)]
struct ForceFields {
    context: usize,
    identity: usize,
    version: u64,
    items: [usize; 8],
}
thread_local! {static FORCE_FIELDS:RefCell<ForceFields>=RefCell::new(ForceFields{context:0,identity:0,version:0,items:[0;8]});}
#[inline]
fn fast_layout() -> bool {
    FAST_LAYOUT.load(Ordering::Relaxed)
}
macro_rules! api {
    ($i:expr, $t:ty) => {
        std::mem::transmute::<usize, $t>(*API_BASE.load(Ordering::Acquire).add($i))
    };
}
unsafe fn inc(o: O) {
    if fast_layout() {
        let h = &mut *o.cast::<Header>();
        // Only ordinary refcounts; immortal/overflow cases retain runtime API.
        if h.references > 0 && h.references < (u32::MAX as isize - 1) {
            h.references += 1;
            return;
        }
    }
    api!(0, unsafe extern "C" fn(O))(o);
}
unsafe fn dec(o: O) {
    if fast_layout() {
        let h = &mut *o.cast::<Header>();
        // Final destruction and immortal objects always use Py_DecRef.
        if h.references > 1 && h.references < u32::MAX as isize {
            h.references -= 1;
            return;
        }
    }
    api!(1, unsafe extern "C" fn(O))(o);
}
struct Owned(O);
impl Drop for Owned {
    fn drop(&mut self) {
        unsafe { dec(self.0) }
    }
}
impl Owned {
    fn take(self) -> O {
        let o = self.0;
        std::mem::forget(self);
        o
    }
}
unsafe fn owned(o: O) -> Option<Owned> {
    if o.is_null() {
        None
    } else {
        Some(Owned(o))
    }
}
#[inline(always)]
unsafe fn key(s: O, i: usize) -> O {
    if fast_layout() {
        return *(*s.cast::<TupleObject>()).items.as_ptr().add(i);
    }
    api!(2, unsafe extern "C" fn(O, isize) -> O)(s, i as isize)
}
#[inline(always)]
unsafe fn number(s: O, o: O) -> Option<f64> {
    if o.is_null() {
        return None;
    }
    let v = if fast_layout() && (*o.cast::<Header>()).kind == key(s, 53) {
        (*o.cast::<FloatObject>()).value
    } else {
        api!(3, unsafe extern "C" fn(O) -> f64)(o)
    };
    // Check conversion errors once at the callback boundary, rather than after
    // every scalar. A pending error always discards the computed result.
    if !v.is_finite() || v.abs() > f32::MAX as f64 {
        None
    } else {
        Some(v)
    }
}
#[inline(always)]
unsafe fn plain_number(s: O, o: O) -> Option<f64> {
    let typ = if fast_layout() {
        None
    } else {
        Some(owned(api!(18, unsafe extern "C" fn(O) -> O)(o))?)
    };
    let kind = if let Some(ref typ) = typ {
        typ.0
    } else {
        (*o.cast::<Header>()).kind
    };
    if kind != key(s, 53) && kind != key(s, 54) && kind != key(s, 56) {
        return None;
    }
    number(s, o)
}
#[inline(always)]
unsafe fn dictionary(s: O, d: O) -> Option<()> {
    if fast_layout() {
        return if !d.is_null() && (*d.cast::<Header>()).kind == key(s, 51) {
            Some(())
        } else {
            None
        };
    }
    // A dict subclass can override __getitem__; preserve its Python behavior.
    let typ = owned(api!(18, unsafe extern "C" fn(O) -> O)(d))?;
    if typ.0 != key(s, 51) {
        return None;
    }
    Some(())
}
#[inline(always)]
unsafe fn field(s: O, d: O, i: usize) -> Option<O> {
    let o = api!(5, unsafe extern "C" fn(O, O) -> O)(d, key(s, i));
    if o.is_null() {
        None
    } else {
        Some(o)
    }
}
#[inline(always)]
unsafe fn scalar(s: O, d: O, i: usize) -> Option<f64> {
    number(s, field(s, d, i)?)
}
#[inline(always)]
unsafe fn sequence(s: O, o: O, out: &mut [f64]) -> Option<()> {
    if o.is_null() {
        return None;
    }
    if fast_layout() {
        let kind = (*o.cast::<Header>()).kind;
        if kind != key(s, 49) && kind != key(s, 50) {
            return None;
        }
        if (*o.cast::<VarObject>()).size != out.len() as isize {
            return None;
        }
        let items = if kind == key(s, 49) {
            (*o.cast::<ListObject>()).items
        } else {
            (*o.cast::<TupleObject>()).items.as_ptr()
        };
        let float_type = key(s, 53);
        for (i, v) in out.iter_mut().enumerate() {
            let item = *items.add(i);
            let value = if (*item.cast::<Header>()).kind == float_type {
                (*item.cast::<FloatObject>()).value
            } else {
                api!(3, unsafe extern "C" fn(O) -> f64)(item)
            };
            if !value.is_finite() || value.abs() > f32::MAX as f64 {
                return None;
            }
            *v = value;
        }
        return Some(());
    }
    let typ = owned(api!(18, unsafe extern "C" fn(O) -> O)(o))?;
    if typ.0 != key(s, 49) && typ.0 != key(s, 50) {
        return None;
    }
    let list = if typ.0 == key(s, 49) {
        api!(6, unsafe extern "C" fn(O) -> isize)(o)
    } else {
        -1
    };
    let get: unsafe extern "C" fn(O, isize) -> O;
    let n;
    if list >= 0 {
        n = list;
        get = api!(7, unsafe extern "C" fn(O, isize) -> O)
    } else {
        api!(8, unsafe extern "C" fn())();
        n = api!(9, unsafe extern "C" fn(O) -> isize)(o);
        get = api!(2, unsafe extern "C" fn(O, isize) -> O)
    }
    if n != out.len() as isize {
        return None;
    }
    for (i, v) in out.iter_mut().enumerate() {
        *v = number(s, get(o, i as isize))?;
    }
    Some(())
}
#[inline(always)]
unsafe fn float(v: f64) -> Option<Owned> {
    owned(api!(10, unsafe extern "C" fn(f64) -> O)(v))
}
#[inline(always)]
unsafe fn list(v: &[f64]) -> Option<Owned> {
    let result = owned(api!(11, unsafe extern "C" fn(isize) -> O)(v.len() as isize))?;
    if fast_layout() {
        let items = (*result.0.cast::<ListObject>()).items.cast_mut();
        let make_float = api!(10, unsafe extern "C" fn(f64) -> O);
        for (i, x) in v.iter().enumerate() {
            let value = make_float(*x);
            if value.is_null() {
                return None;
            }
            items.add(i).write(value);
        }
    } else {
        for (i, x) in v.iter().enumerate() {
            set_new_item(result.0, i as isize, float(*x)?.take())?;
        }
    }
    Some(result)
}
#[inline(always)]
unsafe fn set_new_item(list: O, index: isize, value: O) -> Option<()> {
    // Only used for fresh, private lists with a null slot at this valid index.
    // Equivalent to CPython's PyList_SET_ITEM; ownership is stolen once.
    if fast_layout() {
        debug_assert!(index >= 0 && index < (*list.cast::<ListObject>()).base.size);
        (*list.cast::<ListObject>())
            .items
            .cast_mut()
            .add(index as usize)
            .write(value);
        Some(())
    } else if api!(12, unsafe extern "C" fn(O, isize, O) -> i32)(list, index, value) < 0 {
        None
    } else {
        Some(())
    }
}
unsafe fn dict() -> Option<Owned> {
    owned(api!(13, unsafe extern "C" fn() -> O)())
}
#[inline(always)]
unsafe fn put(s: O, d: &Owned, name: &'static [u8], v: Owned) -> Option<()> {
    if api!(23, unsafe extern "C" fn(O, O, O) -> i32)(d.0, key(s, output_key(name)), v.0) < 0 {
        None
    } else {
        Some(())
    }
}
#[inline(always)]
unsafe fn put_float(s: O, d: &Owned, name: &'static [u8], v: f64) -> Option<()> {
    put(s, d, name, float(v)?)
}
#[inline(always)]
unsafe fn put_list(s: O, d: &Owned, name: &'static [u8], v: &[f64]) -> Option<()> {
    put(s, d, name, list(v)?)
}
#[inline(always)]
unsafe fn finish(s: O, result: Option<Owned>) -> O {
    let error = api!(4, unsafe extern "C" fn() -> O)();
    if error.is_null() {
        if let Some(o) = result {
            return o.take();
        }
    }
    // Input errors are handled by the original Python implementation. Allocation
    // errors must propagate instead of becoming an apparently valid fallback.
    if !error.is_null() && api!(17, unsafe extern "C" fn(O) -> i32)(key(s, 48)) != 0 {
        return ptr::null_mut();
    }
    api!(8, unsafe extern "C" fn())();
    let none = key(s, 47);
    inc(none);
    none
}
unsafe fn args<'a>(values: *const O, n: isize, expected: usize) -> Option<&'a [O]> {
    if n != expected as isize {
        None
    } else {
        Some(std::slice::from_raw_parts(values, expected))
    }
}
unsafe fn all_args<'a>(values: *const O, n: isize) -> &'a [O] {
    // CPython permits a null argument pointer for a zero-argument FASTCALL.
    if n == 0 {
        &[]
    } else {
        std::slice::from_raw_parts(values, n as usize)
    }
}
unsafe fn polar_input(s: O, p: O, strict: bool) -> Option<[f64; 28]> {
    dictionary(s, p)?;
    let version = if fast_layout() {
        (*p.cast::<DictObject>()).version
    } else {
        0
    };
    if version != 0 {
        if let Some(values) = PROFILE.with(|cache| {
            let c = cache.borrow();
            if c.context == s as usize && c.identity == p as usize && c.version == version {
                Some(c.values)
            } else {
                None
            }
        }) {
            return Some(values);
        }
    }
    let mut out = [0.; 28];
    let mut cacheable = version != 0;
    for (i, v) in out[..24].iter_mut().enumerate() {
        // Aerodynamic-center and pitching-moment fields are unused by the
        // lift/drag kernels; the reference never reads them either.
        if (12..15).contains(&i) {
            continue;
        }
        let obj = field(s, p, i)?;
        if cacheable || strict {
            let typ = if fast_layout() {
                None
            } else {
                Some(owned(api!(18, unsafe extern "C" fn(O) -> O)(obj))?)
            };
            let kind = if let Some(ref typ) = typ {
                typ.0
            } else {
                (*obj.cast::<Header>()).kind
            };
            let plain = kind == key(s, 53) || kind == key(s, 54) || kind == key(s, 56);
            if strict && !plain {
                return None;
            }
            cacheable &= plain;
        }
        *v = number(s, obj)?;
    }
    out[12] = if super::bounded_polar_profile(&out) {
        1.
    } else {
        0.
    };
    if out[12] == 1. {
        super::prepare_polar_profile(&mut out);
    }
    if cacheable && api!(4, unsafe extern "C" fn() -> O)().is_null() {
        PROFILE.with(|cache| {
            *cache.borrow_mut() = ProfileCache {
                context: s as usize,
                identity: p as usize,
                version,
                values: out,
            }
        });
    }
    Some(out)
}
#[inline(always)]
unsafe fn cached_polar(
    s: O,
    profile: O,
    a: f64,
    rotation: f64,
    added: f64,
    drag: f64,
    mode: u32,
) -> Option<[f64; 2]> {
    if !fast_layout() {
        return None;
    }
    dictionary(s, profile)?;
    let version = (*profile.cast::<DictObject>()).version;
    PROFILE.with(|cache| {
        let c = cache.borrow();
        if c.context == s as usize
            && c.identity == profile as usize
            && c.version == version
            && c.values[12] == 1.
        {
            super::bounded_polar(&c.values, a, rotation, added, drag, mode)
        } else {
            None
        }
    })
}
#[inline(always)]
unsafe fn polar_values(
    p: &[f64; 28],
    a: f64,
    rotation: f64,
    added: f64,
    drag: f64,
    mode: u32,
    out: *mut f64,
) -> u32 {
    if p[12] == 1. {
        if let Some(v) = super::bounded_polar(p, a, rotation, added, drag, mode) {
            std::ptr::copy_nonoverlapping(v.as_ptr(), out, 2);
            return 1;
        }
    }
    super::wt_polar(p.as_ptr(), a, rotation, added, drag, mode, out)
}
unsafe extern "C" fn polar(s: O, a: *const O, n: isize) -> O {
    finish(
        s,
        (|| {
            let a = args(a, n, 6)?;
            let p = polar_input(s, a[0], false)?;
            let mode = number(s, a[5])?;
            if ![0., 1., 2.].contains(&mode) {
                return None;
            }
            let mut out = [0.; 2];
            if polar_values(
                &p,
                number(s, a[1])?,
                number(s, a[2])?,
                number(s, a[3])?,
                number(s, a[4])?,
                mode as u32,
                out.as_mut_ptr(),
            ) == 0
                || !out.iter().all(|v| v.is_finite())
            {
                return None;
            }
            list(&out)
        })(),
    )
}
unsafe extern "C" fn packed_assembly(s: O, a: *const O, n: isize) -> O {
    finish(
        s,
        (|| {
            let a = args(a, n, 2)?;
            let moment = number(s, a[1])? != 0.;
            let mut input = [0.; 45];
            sequence(s, a[0], &mut input[..if moment { 45 } else { 24 }])?;
            let mut out = [0.; 3];
            let valid = if moment {
                super::wt_moment(input.as_ptr(), out.as_mut_ptr())
            } else {
                super::wt_force(input.as_ptr(), out.as_mut_ptr())
            };
            if valid == 0 || !out.iter().all(|v| v.is_finite()) {
                None
            } else {
                list(&out)
            }
        })(),
    )
}
#[inline(always)]
unsafe fn cached_force_values(s: O, d: O) -> Option<[f64; 24]> {
    if !fast_layout() {
        return None;
    }
    let version = (*d.cast::<DictObject>()).version;
    let cached = FORCE_FIELDS.with(|cache| {
        let c = cache.borrow();
        if c.context == s as usize && c.identity == d as usize && c.version == version {
            Some(c.items)
        } else {
            None
        }
    })?;
    let list_type = key(s, 49);
    let tuple_type = key(s, 50);
    let float_type = key(s, 53);
    let mut values = [0.; 24];
    // No Python API calls, allocations or conversions until every borrowed
    // value has been copied. Thus no callback can invalidate these pointers.
    for (i, address) in cached.iter().enumerate() {
        let o = *address as O;
        let kind = (*o.cast::<Header>()).kind;
        if kind != list_type && kind != tuple_type {
            return None;
        }
        if (*o.cast::<VarObject>()).size != 3 {
            return None;
        }
        let items = if kind == list_type {
            (*o.cast::<ListObject>()).items
        } else {
            (*o.cast::<TupleObject>()).items.as_ptr()
        };
        for j in 0..3 {
            let item = *items.add(j);
            if (*item.cast::<Header>()).kind != float_type {
                return None;
            }
            let value = (*item.cast::<FloatObject>()).value;
            if !value.is_finite() || value.abs() > f32::MAX as f64 {
                return None;
            }
            values[i * 3 + j] = value;
        }
    }
    Some(values)
}
unsafe fn force_fields(s: O, d: O) -> Option<[Owned; 8]> {
    let version = if fast_layout() {
        (*d.cast::<DictObject>()).version
    } else {
        0
    };
    if version != 0 {
        if let Some(items) = FORCE_FIELDS.with(|cache| {
            let c = cache.borrow();
            if c.context == s as usize && c.identity == d as usize && c.version == version {
                Some(c.items)
            } else {
                None
            }
        }) {
            // Keep every value alive before parsing any user-convertible leaf.
            return Some(std::array::from_fn(|i| {
                let o = items[i] as O;
                inc(o);
                Owned(o)
            }));
        }
    }
    let mut held: [Option<Owned>; 8] = std::array::from_fn(|_| None);
    for (i, h) in held.iter_mut().enumerate() {
        let o = field(s, d, 24 + i)?;
        inc(o);
        *h = Some(Owned(o));
    }
    let held = held.map(|v| v.unwrap());
    if version != 0 && (*d.cast::<DictObject>()).version == version {
        FORCE_FIELDS.with(|cache| {
            *cache.borrow_mut() = ForceFields {
                context: s as usize,
                identity: d as usize,
                version,
                items: std::array::from_fn(|i| held[i].0 as usize),
            };
        });
    }
    Some(held)
}
unsafe extern "C" fn force(s: O, a: *const O, n: isize) -> O {
    finish(
        s,
        (|| {
            let a = args(a, n, 1)?;
            dictionary(s, a[0])?;
            let input = if let Some(v) = cached_force_values(s, a[0]) {
                v
            } else {
                let mut input = [0.; 24];
                let held = force_fields(s, a[0])?;
                for i in 0..8 {
                    sequence(s, held[i].0, &mut input[i * 3..i * 3 + 3])?;
                }
                input
            };
            let mut out = [0.; 3];
            if let Some(v) = super::bounded_force(&input) {
                list(&v)
            } else if super::wt_force(input.as_ptr(), out.as_mut_ptr()) == 0 {
                None
            } else {
                list(&out)
            }
        })(),
    )
}
unsafe extern "C" fn moment(s: O, a: *const O, n: isize) -> O {
    finish(
        s,
        (|| {
            let a = args(a, n, 3)?;
            dictionary(s, a[0])?;
            dictionary(s, a[1])?;
            let mut input = [0.; 45];
            for i in 0..7 {
                sequence(s, field(s, a[0], 24 + i)?, &mut input[i * 3..i * 3 + 3])?;
                sequence(
                    s,
                    field(s, a[1], 24 + i)?,
                    &mut input[21 + i * 3..24 + i * 3],
                )?
            }
            sequence(s, a[2], &mut input[42..45])?;
            let mut out = [0.; 3];
            if super::wt_moment(input.as_ptr(), out.as_mut_ptr()) == 0 {
                None
            } else {
                list(&out)
            }
        })(),
    )
}
unsafe extern "C" fn batch(s: O, a: *const O, n: isize) -> O {
    finish(
        s,
        (|| {
            let a = args(a, n, 2)?;
            let p = polar_input(s, a[0], false)?;
            let count = if fast_layout() {
                if (*a[1].cast::<Header>()).kind != key(s, 49) {
                    return None;
                }
                (*a[1].cast::<VarObject>()).size
            } else {
                let typ = owned(api!(18, unsafe extern "C" fn(O) -> O)(a[1]))?;
                if typ.0 != key(s, 49) {
                    return None;
                }
                api!(6, unsafe extern "C" fn(O) -> isize)(a[1])
            };
            if count < 0 {
                return None;
            }
            let result = owned(api!(11, unsafe extern "C" fn(isize) -> O)(count))?;
            for i in 0..count {
                let row = if fast_layout() {
                    *(*a[1].cast::<ListObject>()).items.add(i as usize)
                } else {
                    api!(7, unsafe extern "C" fn(O, isize) -> O)(a[1], i)
                };
                let mut input = [0.; 4];
                sequence(s, row, &mut input)?;
                let mut out = [0.; 2];
                if polar_values(
                    &p,
                    input[0],
                    input[1],
                    input[2],
                    input[3],
                    2,
                    out.as_mut_ptr(),
                ) == 0
                    || !out.iter().all(|v| v.is_finite())
                {
                    return None;
                }
                set_new_item(result.0, i, list(&out)?.take())?;
            }
            Some(result)
        })(),
    )
}
unsafe extern "C" fn atmosphere(s: O, a: *const O, n: isize) -> O {
    finish(
        s,
        (|| {
            let a = args(a, n, 1)?;
            let mut out = [0.; 3];
            super::wt_atmosphere(number(s, a[0])?, out.as_mut_ptr());
            if !out.iter().all(|v| v.is_finite()) {
                return None;
            }
            let d = dict()?;
            put_float(s, &d, b"density\0", out[0])?;
            put_float(s, &d, b"sound_speed\0", out[1])?;
            put_float(s, &d, b"pressure\0", out[2])?;
            Some(d)
        })(),
    )
}
#[inline(always)]
unsafe fn rotation(s: O, out: &[f64]) -> Option<Owned> {
    let result = dict()?;
    let trig = owned(api!(11, unsafe extern "C" fn(isize) -> O)(3))?;
    for i in 0..3 {
        let d = dict()?;
        put_float(s, &d, b"sine\0", out[i * 4])?;
        put_float(s, &d, b"cosine\0", out[i * 4 + 1])?;
        put(
            s,
            &d,
            b"quadrant\0",
            owned(api!(15, unsafe extern "C" fn(i64) -> O)(
                out[i * 4 + 2] as i64,
            ))?,
        )?;
        put_float(s, &d, b"reduced\0", out[i * 4 + 3])?;
        set_new_item(trig.0, i as isize, d.take())?;
    }
    put(s, &result, b"trig\0", trig)?;
    put_list(s, &result, b"delta\0", &out[12..16])?;
    put_list(s, &result, b"raw\0", &out[16..20])?;
    put_list(s, &result, b"quaternion\0", &out[20..24])?;
    Some(result)
}
unsafe extern "C" fn orientation(s: O, a: *const O, n: isize) -> O {
    finish(
        s,
        (|| {
            let a = args(a, n, 2)?;
            let mut input = [0.; 7];
            sequence(s, a[0], &mut input[..4])?;
            sequence(s, a[1], &mut input[4..])?;
            let mut out = [0.; 24];
            if super::wt_orientation(input.as_ptr(), out.as_mut_ptr()) == 0
                || !out.iter().all(|v| v.is_finite())
            {
                None
            } else {
                rotation(s, &out)
            }
        })(),
    )
}
unsafe extern "C" fn vector(s: O, a: *const O, n: isize) -> O {
    finish(
        s,
        (|| {
            let a = args(a, n, 4)?;
            let mode = number(s, a[3])?;
            if ![0., 1., 2.].contains(&mode) {
                return None;
            }
            let mut input = [0.; 10];
            sequence(s, a[0], &mut input[..4])?;
            sequence(s, a[1], &mut input[4..7])?;
            if mode == 1. {
                sequence(s, a[2], &mut input[7..])?;
            }
            let mut out = [0.; 3];
            if super::aero::wt_vector(input.as_ptr(), mode as u32, out.as_mut_ptr()) == 0 {
                None
            } else {
                list(&out)
            }
        })(),
    )
}
unsafe extern "C" fn matrix(s: O, a: *const O, n: isize) -> O {
    finish(
        s,
        (|| {
            let a = args(a, n, 3)?;
            let mut input = [0.; 9];
            for i in 0..3 {
                sequence(s, a[i], &mut input[i * 3..i * 3 + 3])?
            }
            let mut out = [0.; 4];
            if super::aero::wt_matrix_quaternion(input.as_ptr(), out.as_mut_ptr()) == 0 {
                None
            } else {
                list(&out)
            }
        })(),
    )
}
unsafe extern "C" fn integrate(s: O, a: *const O, n: isize) -> O {
    finish(
        s,
        (|| {
            let a = args(a, n, 6)?;
            dictionary(s, a[0])?;
            let mut input = [0.; 33];
            for (key, start, len) in [(32, 0, 3), (33, 3, 3), (34, 6, 3), (35, 9, 4), (37, 14, 4)] {
                sequence(s, field(s, a[0], key)?, &mut input[start..start + len])?;
            }
            for (key, i) in [(36, 13), (38, 18), (39, 19), (40, 20)] {
                input[i] = scalar(s, a[0], key)?;
            }
            sequence(s, a[1], &mut input[21..24])?;
            sequence(s, a[2], &mut input[24..27])?;
            input[27] = number(s, a[3])?;
            input[28] = number(s, a[4])?;
            sequence(s, a[5], &mut input[29..33])?;
            let mut out = [0.; 51];
            if super::aero::wt_integrate(input.as_ptr(), out.as_mut_ptr()) == 0 {
                return None;
            }
            let result = dict()?;
            let state = dict()?;
            let rot = rotation(s, &out[27..])?;
            for (name, start, len) in [
                (b"position\0".as_slice(), 0, 3),
                (b"velocity\0", 3, 3),
                (b"omega\0", 6, 3),
                (b"clocks\0", 14, 4),
            ] {
                put_list(s, &state, name, &out[start..start + len])?;
            }
            let q = api!(5, unsafe extern "C" fn(O, O) -> O)(rot.0, key(s, 35));
            if q.is_null() {
                return None;
            }
            inc(q);
            put(s, &state, b"quaternion\0", Owned(q))?;
            for (name, i) in [
                (b"time\0".as_slice(), 13),
                (b"distance\0", 18),
                (b"water_distance\0", 19),
                (b"immersion\0", 20),
            ] {
                put_float(s, &state, name, out[i])?;
            }
            put(s, &result, b"state\0", state)?;
            put_list(s, &result, b"displacement\0", &out[21..24])?;
            put_list(s, &result, b"increment\0", &out[24..27])?;
            put(s, &result, b"rotation\0", rot)?;
            Some(result)
        })(),
    )
}
#[repr(C)]
struct Method {
    name: *const c_char,
    function: unsafe extern "C" fn(O, *const O, isize) -> O,
    flags: i32,
    doc: *const c_char,
}
// Static definitions have process lifetime; CPython retains pointers to them.
unsafe impl Sync for Method {}
macro_rules! method {
    ($name:literal,$f:ident) => {
        Method {
            name: concat!($name, "\0").as_ptr().cast(),
            function: $f,
            flags: 0x80,
            doc: ptr::null(),
        }
    };
}
static METHODS: [Method; 13] = [
    method!("polar", polar),
    method!("assembly", packed_assembly),
    method!("force", force),
    method!("moment", moment),
    method!("batch", batch),
    method!("atmosphere", atmosphere),
    method!("orientation", orientation),
    method!("vector", vector),
    method!("matrix", matrix),
    method!("integrate", integrate),
    method!("deepcopy", deepcopy),
    method!("finite_graph", finite_graph),
    method!("aero", aero),
];

unsafe fn bool_obj(s: O, value: bool) -> Owned {
    let o = key(s, if value { 60 } else { 61 });
    inc(o);
    Owned(o)
}
unsafe fn alias(o: &Owned) -> Owned {
    inc(o.0);
    Owned(o.0)
}
unsafe fn evaluated(s: O, r: &[f64]) -> Option<Owned> {
    let d = dict()?;
    for (name, start) in [(b"drag\0".as_slice(), 0), (b"lift\0", 3), (b"force\0", 6)] {
        put_list(s, &d, name, &r[start..start + 3])?;
    }
    for (name, i) in [(b"cosine\0".as_slice(), 9), (b"cd\0", 10), (b"cy\0", 11)] {
        put_float(s, &d, name, r[i])?;
    }
    Some(d)
}
unsafe fn matrix_list(r: &[f64]) -> Option<Owned> {
    let out = owned(api!(11, unsafe extern "C" fn(isize) -> O)(3))?;
    for i in 0..3 {
        if api!(12, unsafe extern "C" fn(O, isize, O) -> i32)(
            out.0,
            i as isize,
            list(&r[i * 3..i * 3 + 3])?.take(),
        ) < 0
        {
            return None;
        }
    }
    Some(out)
}
unsafe extern "C" fn aero(s: O, a: *const O, n: isize) -> O {
    finish(
        s,
        (|| {
            let a = args(a, n, 6)?;
            dictionary(s, a[0])?;
            dictionary(s, a[5])?;
            let mut p = [0.; 22];
            let mut v = [0.; 34];
            for i in 0..12 {
                p[i] = scalar(s, a[0], AERO_INPUT + i)?;
            }
            sequence(s, field(s, a[0], AERO_INPUT + 12)?, &mut p[12..16])?;
            sequence(s, field(s, a[0], AERO_INPUT + 13)?, &mut p[16..19])?;
            sequence(s, field(s, a[0], AERO_INPUT + 14)?, &mut p[19..22])?;
            v[0] = number(s, a[1])?;
            sequence(s, a[2], &mut v[1..4])?;
            sequence(s, a[3], &mut v[4..8])?;
            sequence(s, a[4], &mut v[8..11])?;
            v[18] = 1. / 48.;
            v[26] = 1.;
            v[27] = 1.;
            let mut supplied = 0;
            for (k, start, len) in [
                (16, 11, 3),
                (17, 14, 2),
                (21, 19, 3),
                (22, 22, 3),
                (27, 29, 3),
            ] {
                if let Some(value) = field(s, a[5], AERO_INPUT + k) {
                    sequence(s, value, &mut v[start..start + len])?;
                    supplied += 1;
                }
            }
            for (k, i) in [
                (18, 16),
                (19, 17),
                (20, 18),
                (23, 25),
                (24, 26),
                (25, 27),
                (26, 28),
                (28, 32),
                (29, 33),
            ] {
                if let Some(value) = field(s, a[5], AERO_INPUT + k) {
                    v[i] = number(s, value)?;
                    supplied += 1;
                }
            }
            if supplied != api!(26, unsafe extern "C" fn(O) -> isize)(a[5]) {
                return None;
            }
            let rows = field(s, a[0], AERO_INPUT + 15)?;
            let typ = owned(api!(18, unsafe extern "C" fn(O) -> O)(rows))?;
            let count;
            let get: unsafe extern "C" fn(O, isize) -> O;
            if typ.0 == key(s, 49) {
                count = api!(6, unsafe extern "C" fn(O) -> isize)(rows);
                get = api!(7, unsafe extern "C" fn(O, isize) -> O)
            } else if typ.0 == key(s, 50) {
                count = api!(9, unsafe extern "C" fn(O) -> isize)(rows);
                get = api!(2, unsafe extern "C" fn(O, isize) -> O)
            } else {
                return None;
            }
            let mut table = Vec::new();
            table.try_reserve((count as usize).checked_mul(3)?).ok()?;
            for i in 0..count {
                let mut row = [0.; 3];
                sequence(s, get(rows, i), &mut row)?;
                table.extend(row);
            }
            let mut r = [0.; 85];
            if super::aero::wt_aero(
                p.as_ptr(),
                v.as_ptr(),
                table.as_ptr(),
                count as usize,
                r.as_mut_ptr(),
            ) == 0
            {
                return None;
            }
            let d = dict()?;
            let flow = list(&r[24..27])?;
            let baseline = evaluated(s, &r[27..39])?;
            let active = r[39] != 0.;
            put(s, &d, b"frame\0", matrix_list(&r[..9])?)?;
            put(s, &d, b"axes\0", matrix_list(&r[9..18])?)?;
            for (name, i) in [
                (b"cm_speed\0".as_slice(), 18),
                (b"mach\0", 19),
                (b"pressure\0", 20),
                (b"effective_mass\0", 76),
            ] {
                put_float(s, &d, name, r[i])?;
            }
            put_list(s, &d, b"local_flow\0", &r[21..24])?;
            put(s, &d, b"flow\0", alias(&flow))?;
            put(s, &d, b"baseline\0", alias(&baseline))?;
            put(s, &d, b"fin_active\0", bool_obj(s, active))?;
            put(s, &d, b"fin_limited\0", bool_obj(s, r[40] != 0.))?;
            put_list(s, &d, b"deflection\0", &r[41..43])?;
            put(
                s,
                &d,
                b"fin_flow\0",
                if active { list(&r[43..46])? } else { flow },
            )?;
            put(
                s,
                &d,
                b"steering\0",
                if active {
                    evaluated(s, &r[46..58])?
                } else {
                    baseline
                },
            )?;
            for (name, start) in [
                (b"moment\0".as_slice(), 58),
                (b"damping\0", 61),
                (b"angular_acceleration_before_environment\0", 67),
                (b"angular_acceleration\0", 70),
                (b"acceleration\0", 73),
                (b"perturbed_lever\0", 82),
            ] {
                put_list(s, &d, name, &r[start..start + 3])?;
            }
            let clipped = owned(api!(11, unsafe extern "C" fn(isize) -> O)(3))?;
            for i in 0..3 {
                if api!(12, unsafe extern "C" fn(O, isize, O) -> i32)(
                    clipped.0,
                    i as isize,
                    bool_obj(s, r[64 + i] != 0.).take(),
                ) < 0
                {
                    return None;
                }
            }
            put(s, &d, b"damping_clipped\0", clipped)?;
            let perturb = dict()?;
            for (name, i) in [
                (b"force_scale\0".as_slice(), 77),
                (b"angle\0", 78),
                (b"cosine\0", 79),
                (b"sine\0", 80),
                (b"lever_fraction\0", 81),
            ] {
                put_float(s, &perturb, name, r[i])?;
            }
            put(s, &d, b"perturbation\0", perturb)?;
            Some(d)
        })(),
    )
}

unsafe fn all_finite(s: O, o: O, depth: usize, string_keys: bool) -> Option<bool> {
    if depth > 100 {
        return Some(false);
    }
    let typ = owned(api!(18, unsafe extern "C" fn(O) -> O)(o))?;
    if typ.0 == key(s, 53) {
        return Some(api!(3, unsafe extern "C" fn(O) -> f64)(o).is_finite());
    }
    if atomic(s, typ.0) {
        return Some(true);
    }
    if typ.0 == key(s, 51) {
        let mut pos = 0;
        let mut k = ptr::null_mut();
        let mut v = ptr::null_mut();
        while api!(
            22,
            unsafe extern "C" fn(O, *mut isize, *mut O, *mut O) -> i32
        )(o, &mut pos, &mut k, &mut v)
            != 0
        {
            if string_keys && owned(api!(18, unsafe extern "C" fn(O) -> O)(k))?.0 != key(s, 55) {
                return Some(false);
            }
            if !all_finite(s, v, depth + 1, string_keys)? {
                return Some(false);
            }
        }
        return Some(true);
    }
    let n;
    let get: unsafe extern "C" fn(O, isize) -> O;
    if typ.0 == key(s, 49) {
        n = api!(6, unsafe extern "C" fn(O) -> isize)(o);
        get = api!(7, unsafe extern "C" fn(O, isize) -> O)
    } else if typ.0 == key(s, 50) {
        n = api!(9, unsafe extern "C" fn(O) -> isize)(o);
        get = api!(2, unsafe extern "C" fn(O, isize) -> O)
    } else {
        return Some(false);
    }
    for i in 0..n {
        if !all_finite(s, get(o, i), depth + 1, string_keys)? {
            return Some(false);
        }
    }
    Some(true)
}
unsafe extern "C" fn finite_graph(s: O, a: *const O, n: isize) -> O {
    finish(
        s,
        (|| {
            let a = args(a, n, 2)?;
            let value = all_finite(s, a[0], 0, number(s, a[1])? != 0.)?;
            let result = key(s, if value { 60 } else { 61 });
            inc(result);
            Some(Owned(result))
        })(),
    )
}

unsafe fn atomic(s: O, typ: O) -> bool {
    (53..59).any(|i| typ == key(s, i))
}
unsafe fn plain_graph(
    s: O,
    o: O,
    seen: &mut std::collections::HashSet<usize>,
    depth: usize,
) -> Option<bool> {
    if depth > 100 {
        return Some(false);
    }
    let typ = owned(api!(18, unsafe extern "C" fn(O) -> O)(o))?;
    if atomic(s, typ.0) {
        return Some(true);
    }
    if typ.0 != key(s, 49) && typ.0 != key(s, 50) && typ.0 != key(s, 51) {
        return Some(false);
    }
    if !seen.insert(o as usize) {
        return Some(true);
    }
    if typ.0 == key(s, 49) || typ.0 == key(s, 50) {
        let is_list = typ.0 == key(s, 49);
        let n = if is_list {
            api!(6, unsafe extern "C" fn(O) -> isize)(o)
        } else {
            api!(9, unsafe extern "C" fn(O) -> isize)(o)
        };
        let get: unsafe extern "C" fn(O, isize) -> O = if is_list {
            api!(7, unsafe extern "C" fn(O, isize) -> O)
        } else {
            api!(2, unsafe extern "C" fn(O, isize) -> O)
        };
        for i in 0..n {
            if !plain_graph(s, get(o, i), seen, depth + 1)? {
                return Some(false);
            }
        }
    } else {
        let mut pos = 0;
        let mut k = ptr::null_mut();
        let mut v = ptr::null_mut();
        while api!(
            22,
            unsafe extern "C" fn(O, *mut isize, *mut O, *mut O) -> i32
        )(o, &mut pos, &mut k, &mut v)
            != 0
        {
            if !plain_graph(s, k, seen, depth + 1)? || !plain_graph(s, v, seen, depth + 1)? {
                return Some(false);
            }
        }
    }
    Some(true)
}
unsafe fn identity(o: O) -> Option<Owned> {
    owned(api!(25, unsafe extern "C" fn(O) -> O)(o))
}
unsafe fn clone_graph(s: O, o: O, memo: O) -> Option<Owned> {
    let typ = owned(api!(18, unsafe extern "C" fn(O) -> O)(o))?;
    if atomic(s, typ.0) {
        inc(o);
        return Some(Owned(o));
    }
    let id = identity(o)?;
    let existing = api!(5, unsafe extern "C" fn(O, O) -> O)(memo, id.0);
    if !existing.is_null() {
        inc(existing);
        return Some(Owned(existing));
    }
    if typ.0 == key(s, 50) {
        let size = api!(9, unsafe extern "C" fn(O) -> isize)(o);
        let mut children = Vec::with_capacity(size as usize);
        let mut unchanged = true;
        for i in 0..size {
            let item = api!(2, unsafe extern "C" fn(O, isize) -> O)(o, i);
            let child = clone_graph(s, item, memo)?;
            unchanged &= child.0 == item;
            children.push(child);
        }
        // A tuple/list cycle can memoize this tuple while copying its children.
        let existing = api!(5, unsafe extern "C" fn(O, O) -> O)(memo, id.0);
        if !existing.is_null() {
            inc(existing);
            return Some(Owned(existing));
        }
        if unchanged {
            inc(o);
            return Some(Owned(o));
        }
        let result = owned(api!(19, unsafe extern "C" fn(isize) -> O)(size))?;
        for (i, child) in children.into_iter().enumerate() {
            if api!(20, unsafe extern "C" fn(O, isize, O) -> i32)(
                result.0,
                i as isize,
                child.take(),
            ) < 0
            {
                return None;
            }
        }
        if api!(23, unsafe extern "C" fn(O, O, O) -> i32)(memo, id.0, result.0) < 0 {
            return None;
        }
        keep_alive(o, memo)?;
        return Some(result);
    }
    let is_list = typ.0 == key(s, 49);
    let size = if is_list {
        api!(6, unsafe extern "C" fn(O) -> isize)(o)
    } else {
        0
    };
    let result = if is_list {
        owned(api!(11, unsafe extern "C" fn(isize) -> O)(0))?
    } else {
        dict()?
    };
    if api!(23, unsafe extern "C" fn(O, O, O) -> i32)(memo, id.0, result.0) < 0 {
        return None;
    }
    if is_list {
        for i in 0..size {
            let v = api!(7, unsafe extern "C" fn(O, isize) -> O)(o, i);
            let child = clone_graph(s, v, memo)?;
            // Keep memo-visible cyclic lists valid even during allocation/GC.
            if api!(24, unsafe extern "C" fn(O, O) -> i32)(result.0, child.0) < 0 {
                return None;
            }
        }
    } else {
        let mut pos = 0;
        let mut k = ptr::null_mut();
        let mut v = ptr::null_mut();
        while api!(
            22,
            unsafe extern "C" fn(O, *mut isize, *mut O, *mut O) -> i32
        )(o, &mut pos, &mut k, &mut v)
            != 0
        {
            let ck = clone_graph(s, k, memo)?;
            let cv = clone_graph(s, v, memo)?;
            if api!(23, unsafe extern "C" fn(O, O, O) -> i32)(result.0, ck.0, cv.0) < 0 {
                return None;
            }
        }
    }
    keep_alive(o, memo)?;
    Some(result)
}
unsafe fn keep_alive(o: O, memo: O) -> Option<()> {
    // Match copy._keep_alive: retain source containers while the memo survives.
    let memo_id = identity(memo)?;
    let mut keeper = api!(5, unsafe extern "C" fn(O, O) -> O)(memo, memo_id.0);
    let keepalive;
    if keeper.is_null() {
        keepalive = owned(api!(11, unsafe extern "C" fn(isize) -> O)(0))?;
        keeper = keepalive.0;
        if api!(23, unsafe extern "C" fn(O, O, O) -> i32)(memo, memo_id.0, keeper) < 0 {
            return None;
        }
    }
    if api!(24, unsafe extern "C" fn(O, O) -> i32)(keeper, o) < 0 {
        return None;
    }
    Some(())
}
unsafe extern "C" fn deepcopy(s: O, a: *const O, n: isize) -> O {
    let Some(a) = args(a, n, 2) else {
        return scalar_fallback(key(s, 59), all_args(a, n));
    };
    let result = (|| {
        // Custom objects and subclasses delegate as a whole graph. This
        // prevents custom callbacks from mutating a dictionary during PyDict_Next.
        let mut seen = std::collections::HashSet::new();
        if !plain_graph(s, a[0], &mut seen, 0)? || seen.contains(&(a[1] as usize)) {
            return None;
        }
        let memo;
        let m = if a[1] == key(s, 47) {
            memo = dict()?;
            memo.0
        } else {
            dictionary(s, a[1])?;
            a[1]
        };
        clone_graph(s, a[0], m)
    })();
    if let Some(result) = result {
        return result.take();
    }
    if !api!(4, unsafe extern "C" fn() -> O)().is_null() {
        return ptr::null_mut();
    }
    scalar_fallback(key(s, 59), a)
}

unsafe fn scalar_fallback(reference: O, values: &[O]) -> O {
    api!(8, unsafe extern "C" fn())();
    let Some(tuple) = owned(api!(19, unsafe extern "C" fn(isize) -> O)(
        values.len() as isize
    )) else {
        return ptr::null_mut();
    };
    for (i, o) in values.iter().enumerate() {
        inc(*o);
        if api!(20, unsafe extern "C" fn(O, isize, O) -> i32)(tuple.0, i as isize, *o) < 0 {
            return ptr::null_mut();
        }
    }
    api!(21, unsafe extern "C" fn(O, O) -> O)(reference, tuple.0)
}
unsafe fn scalar_call(s: O, a: *const O, n: isize, op: u32) -> O {
    let reference = key(s, 0);
    let values = all_args(a, n);
    if n != if op == 0 { 1 } else { 2 } {
        return scalar_fallback(reference, values);
    }
    let x = api!(3, unsafe extern "C" fn(O) -> f64)(values[0]);
    let x = (x as f32) as f64;
    let result = if op == 0 {
        x
    } else {
        let y = (api!(3, unsafe extern "C" fn(O) -> f64)(values[1]) as f32) as f64;
        match op {
            1 => x + y,
            2 => x - y,
            3 => x * y,
            _ => {
                if y == 0. {
                    return scalar_fallback(reference, values);
                }
                x / y
            }
        }
    };
    if !api!(4, unsafe extern "C" fn() -> O)().is_null() {
        return scalar_fallback(reference, values);
    }
    api!(10, unsafe extern "C" fn(f64) -> O)((result as f32) as f64)
}
macro_rules! scalar_method {
    ($f:ident,$op:literal) => {
        unsafe extern "C" fn $f(s: O, a: *const O, n: isize) -> O {
            scalar_call(s, a, n, $op)
        }
    };
}
scalar_method!(f32, 0);
scalar_method!(add, 1);
scalar_method!(sub, 2);
scalar_method!(mul, 3);
scalar_method!(div, 4);
static SCALARS: [Method; 5] = [
    method!("f32", f32),
    method!("add", add),
    method!("sub", sub),
    method!("mul", mul),
    method!("div", div),
];
/// context owns (reference callable, library), retaining both for every builtin.
#[no_mangle]
pub unsafe extern "C" fn wt_python_scalar(index: u32, context: O) -> O {
    if index >= 5 || API.get().is_none() || context.is_null() {
        return ptr::null_mut();
    }
    api!(16, unsafe extern "C" fn(*const Method, O, O) -> O)(
        &SCALARS[index as usize],
        context,
        ptr::null_mut(),
    )
}
#[no_mangle]
pub unsafe extern "C" fn wt_python_init(
    addresses: *const usize,
    len: usize,
    context: O,
    layout: u32,
) -> O {
    if len != 28 || context.is_null() {
        return ptr::null_mut();
    }
    let table: [usize; 28] = std::slice::from_raw_parts(addresses, 28)
        .try_into()
        .unwrap();
    if table.contains(&0) {
        return ptr::null_mut();
    }
    if let Some(current) = API.get() {
        if current != &table {
            return ptr::null_mut();
        }
    } else {
        let _ = API.set(table);
    }
    API_BASE.store(API.get().unwrap().as_ptr().cast_mut(), Ordering::Release);
    if api!(9, unsafe extern "C" fn(O) -> isize)(context) != 144 {
        return ptr::null_mut();
    }
    FAST_LAYOUT.store(layout == 1, Ordering::Relaxed);
    PROFILE.with(|cache| cache.borrow_mut().identity = 0);
    let result = (|| {
        let d = dict()?;
        for m in &METHODS {
            let f = owned(api!(16, unsafe extern "C" fn(*const Method, O, O) -> O)(
                m,
                context,
                ptr::null_mut(),
            ))?;
            if api!(14, unsafe extern "C" fn(O, *const c_char, O) -> i32)(d.0, m.name, f.0) < 0 {
                return None;
            }
        }
        if api!(14, unsafe extern "C" fn(O, *const c_char, O) -> i32)(
            d.0,
            b"_context\0".as_ptr().cast(),
            context,
        ) < 0
        {
            return None;
        }
        Some(d)
    })();
    result.map_or(ptr::null_mut(), Owned::take)
}

unsafe fn original_call(reference: O, a: &[O], keywords: O) -> O {
    let count = if keywords.is_null() {
        0
    } else {
        api!(9, unsafe extern "C" fn(O) -> isize)(keywords) as usize
    };
    let positional = a.len() - count;
    let Some(tuple) = owned(api!(19, unsafe extern "C" fn(isize) -> O)(
        positional as isize,
    )) else {
        return ptr::null_mut();
    };
    for (i, o) in a[..positional].iter().enumerate() {
        inc(*o);
        if api!(20, unsafe extern "C" fn(O, isize, O) -> i32)(tuple.0, i as isize, *o) < 0 {
            return ptr::null_mut();
        }
    }
    if count == 0 {
        return api!(21, unsafe extern "C" fn(O, O) -> O)(reference, tuple.0);
    }
    let Some(kwargs) = keyword_dict(a, positional, keywords) else {
        return ptr::null_mut();
    };
    api!(27, unsafe extern "C" fn(O, O, O) -> O)(reference, tuple.0, kwargs.0)
}
unsafe fn keyword_dict(a: &[O], n: usize, keywords: O) -> Option<Owned> {
    let d = dict()?;
    let count = if keywords.is_null() {
        0
    } else {
        api!(9, unsafe extern "C" fn(O) -> isize)(keywords)
    };
    for i in 0..count {
        let k = api!(2, unsafe extern "C" fn(O, isize) -> O)(keywords, i);
        if api!(23, unsafe extern "C" fn(O, O, O) -> i32)(d.0, k, a[n + i as usize]) < 0 {
            return None;
        }
    }
    Some(d)
}
#[inline(always)]
unsafe fn bound_call(self_: O, argv: *const O, n: isize, keywords: O, kind: u32) -> O {
    let s = key(self_, 0);
    let nk = if keywords.is_null() {
        0
    } else {
        api!(9, unsafe extern "C" fn(O) -> isize)(keywords)
    };
    let a = all_args(argv, n + nk);
    let result = if nk != 0 && kind != 12 {
        ptr::null_mut()
    } else {
        match kind {
            0 => force(s, argv, n),
            1 => moment(s, argv, n),
            2 | 3 | 4 => {
                let allowed = if kind == 4 { 3..=5 } else { 2..=2 };
                if !allowed.contains(&(n as usize)) {
                    ptr::null_mut()
                } else {
                    finish(
                        s,
                        (|| {
                            let angle = plain_number(s, a[1])?;
                            let rotation = if kind == 4 {
                                plain_number(s, a[2])?
                            } else {
                                0.
                            };
                            let added = if n > 3 { plain_number(s, a[3])? } else { 0. };
                            let drag = if n > 4 { plain_number(s, a[4])? } else { 1. };
                            let mut r = [0.; 2];
                            let mode = if kind == 2 {
                                0
                            } else if kind == 3 {
                                1
                            } else {
                                2
                            };
                            if let Some(v) =
                                cached_polar(s, a[0], angle, rotation, added, drag, mode)
                            {
                                r = v;
                            } else {
                                let p = polar_input(s, a[0], true)?;
                                if polar_values(
                                    &p,
                                    angle,
                                    rotation,
                                    added,
                                    drag,
                                    mode,
                                    r.as_mut_ptr(),
                                ) == 0
                                    || !r.iter().all(|v| v.is_finite())
                                {
                                    return None;
                                }
                            }
                            if mode == 2 {
                                list(&r)
                            } else {
                                float(r[0])
                            }
                        })(),
                    )
                }
            }
            5 | 6 | 7 => {
                let required = if kind == 6 { 3 } else { 2 };
                if n != required {
                    ptr::null_mut()
                } else {
                    // The mode is a native constant; avoid creating a temporary Python int.
                    finish(
                        s,
                        (|| {
                            let mut input = [0.; 10];
                            sequence(s, a[0], &mut input[..4])?;
                            sequence(s, a[1], &mut input[4..7])?;
                            if kind == 6 {
                                sequence(s, a[2], &mut input[7..])?
                            }
                            let mut r = [0.; 3];
                            if super::aero::wt_vector(input.as_ptr(), kind - 5, r.as_mut_ptr()) == 0
                            {
                                None
                            } else {
                                list(&r)
                            }
                        })(),
                    )
                }
            }
            8 => atmosphere(s, argv, n),
            9 => orientation(s, argv, n),
            10 => matrix(s, argv, n),
            11 => {
                if n == 5 {
                    let p = [a[0], a[1], a[2], a[3], a[4], key(self_, 3)];
                    integrate(s, p.as_ptr(), 6)
                } else {
                    integrate(s, argv, n)
                }
            }
            _ => {
                if n != 5 {
                    ptr::null_mut()
                } else {
                    let kwargs = keyword_dict(a, n as usize, keywords);
                    match kwargs {
                        None => return ptr::null_mut(),
                        Some(k) => {
                            let p = [a[0], a[1], a[2], a[3], a[4], k.0];
                            aero(s, p.as_ptr(), 6)
                        }
                    }
                }
            }
        }
    };
    if !result.is_null() {
        if result != key(s, 47) {
            return result;
        }
        dec(result)
    }
    if !api!(4, unsafe extern "C" fn() -> O)().is_null() {
        return ptr::null_mut();
    }
    original_call(key(self_, 1), a, keywords)
}
#[repr(C)]
struct BoundMethod {
    name: *const c_char,
    function: unsafe extern "C" fn(O, *const O, isize, O) -> O,
    flags: i32,
    doc: *const c_char,
}
unsafe impl Sync for BoundMethod {}
macro_rules! bound_method {
    ($f:ident,$kind:literal) => {
        unsafe extern "C" fn $f(s: O, a: *const O, n: isize, k: O) -> O {
            bound_call(s, a, n, k, $kind)
        }
    };
}
bound_method!(bound_force, 0);
bound_method!(bound_moment, 1);
bound_method!(bound_cl, 2);
bound_method!(bound_cd, 3);
unsafe extern "C" fn bound_polar(self_: O, argv: *const O, n: isize, keywords: O) -> O {
    if n == 5 && keywords.is_null() && fast_layout() {
        if let Some(result) = fast_public_polar(self_, argv) {
            return result;
        }
    }
    bound_call(self_, argv, n, keywords, 4)
}

#[inline(always)]
unsafe fn fast_public_polar(self_: O, argv: *const O) -> Option<O> {
    // This path performs no Python conversions or callbacks before copying
    // inputs. The generic binding retains defaults, keywords and fallback.
    let s = *(*self_.cast::<TupleObject>()).items.as_ptr();
    let context = (*s.cast::<TupleObject>()).items.as_ptr();
    let p = *argv;
    if (*p.cast::<Header>()).kind != *context.add(51) {
        return None;
    }
    let float_type = *context.add(53);
    let mut values = [0.; 4];
    for (i, value) in values.iter_mut().enumerate() {
        let obj = *argv.add(i + 1);
        if (*obj.cast::<Header>()).kind != float_type {
            return None;
        }
        *value = (*obj.cast::<FloatObject>()).value;
    }
    let version = (*p.cast::<DictObject>()).version;
    let result = PROFILE.with(|cache| {
        let c = cache.borrow();
        if c.context == s as usize
            && c.identity == p as usize
            && c.version == version
            && c.values[12] == 1.
        {
            super::bounded_polar(&c.values, values[0], values[1], values[2], values[3], 2)
        } else {
            None
        }
    })?;
    // Allocation failure must return NULL with the interpreter error intact.
    Some(list(&result).map_or(ptr::null_mut(), Owned::take))
}
bound_method!(bound_rotate, 5);
bound_method!(bound_residual, 6);
bound_method!(bound_coast, 7);
bound_method!(bound_atmosphere, 8);
bound_method!(bound_orientation, 9);
bound_method!(bound_matrix, 10);
bound_method!(bound_integrate, 11);
bound_method!(bound_aero, 12);
macro_rules! bound_definition {
    ($name:literal,$f:ident) => {
        BoundMethod {
            name: concat!($name, "\0").as_ptr().cast(),
            function: $f,
            flags: 0x82,
            doc: ptr::null(),
        }
    };
}
static BOUND: [BoundMethod; 13] = [
    bound_definition!("assemble_force", bound_force),
    bound_definition!("assemble_moment", bound_moment),
    bound_definition!("calc_cl", bound_cl),
    bound_definition!("calc_cd", bound_cd),
    bound_definition!("calc_c", bound_polar),
    bound_definition!("rotate_thrust", bound_rotate),
    bound_definition!("world_residual", bound_residual),
    bound_definition!("coast_body", bound_coast),
    bound_definition!("atmosphere", bound_atmosphere),
    bound_definition!("orientation", bound_orientation),
    bound_definition!("matrix_quaternion", bound_matrix),
    bound_definition!("integrate", bound_integrate),
    bound_definition!("forces", bound_aero),
];
#[no_mangle]
pub unsafe extern "C" fn wt_python_bound(index: u32, context: O) -> O {
    if index >= 13 || API.get().is_none() || context.is_null() {
        return ptr::null_mut();
    }
    api!(16, unsafe extern "C" fn(*const BoundMethod, O, O) -> O)(
        &BOUND[index as usize],
        context,
        ptr::null_mut(),
    )
}

#[inline(always)]
fn output_key(name: &[u8]) -> usize {
    match name {
        b"density\0" => 62,
        b"sound_speed\0" => 63,
        b"pressure\0" => 64,
        b"sine\0" => 65,
        b"cosine\0" => 66,
        b"quadrant\0" => 67,
        b"reduced\0" => 68,
        b"trig\0" => 69,
        b"delta\0" => 70,
        b"raw\0" => 71,
        b"quaternion\0" => 72,
        b"position\0" => 73,
        b"velocity\0" => 74,
        b"omega\0" => 75,
        b"clocks\0" => 76,
        b"time\0" => 77,
        b"distance\0" => 78,
        b"water_distance\0" => 79,
        b"immersion\0" => 80,
        b"state\0" => 81,
        b"displacement\0" => 82,
        b"increment\0" => 83,
        b"rotation\0" => 84,
        b"drag\0" => 85,
        b"lift\0" => 86,
        b"force\0" => 87,
        b"cd\0" => 88,
        b"cy\0" => 89,
        b"frame\0" => 90,
        b"axes\0" => 91,
        b"cm_speed\0" => 92,
        b"mach\0" => 93,
        b"local_flow\0" => 94,
        b"flow\0" => 95,
        b"baseline\0" => 96,
        b"fin_active\0" => 97,
        b"fin_limited\0" => 98,
        b"deflection\0" => 99,
        b"fin_flow\0" => 100,
        b"steering\0" => 101,
        b"moment\0" => 102,
        b"damping\0" => 103,
        b"damping_clipped\0" => 104,
        b"angular_acceleration_before_environment\0" => 105,
        b"angular_acceleration\0" => 106,
        b"acceleration\0" => 107,
        b"effective_mass\0" => 108,
        b"perturbation\0" => 109,
        b"force_scale\0" => 110,
        b"angle\0" => 111,
        b"lever_fraction\0" => 112,
        b"perturbed_lever\0" => 113,
        _ => unreachable!("Unknown native output field"),
    }
}
const AERO_INPUT: usize = 114;
#[no_mangle]
pub extern "C" fn wt_python_field_names() -> *const c_char {
    b"density sound_speed pressure sine cosine quadrant reduced trig delta raw quaternion position velocity omega clocks time distance water_distance immersion state displacement increment rotation drag lift force cd cy frame axes cm_speed mach local_flow flow baseline fin_active fin_limited deflection fin_flow steering moment damping damping_clipped angular_acceleration_before_environment angular_acceleration acceleration effective_mass perturbation force_scale angle lever_fraction perturbed_lever stabilizer_arm cx cx_aoa cy cy_limit front_area side_area fins_hor fins_ver fin_pressure_limit damping_geometry mass axis_quaternion inertia angular_damping cy_table wind fins additional_cx additional_lever dt torque force mass_lost gravity use_cxi mass_term angular_environment perturbation body_random\0".as_ptr().cast()
}
