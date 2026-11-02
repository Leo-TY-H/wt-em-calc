"""Engine ratings page (Leo, 2026-10-30): each engine's temperature limits under the game's own thermal fatigue rules.

Per distinct engine and temperature channel (water or cylinder head, and oil), with the temperature held constant:
the hottest temperature sustainable indefinitely (no load counter can ever overflow, whatever its history) and, for
each rating of R minutes, the hottest one at which fresh counters give no damage within R minutes; the automatic
radiator's resting temperatures and how long they last; by gauge reading (Leo, 2026-11-02: pilot-direct), the bands of
equal time to first damage and how cool and for how long to get each one's time back; and the time to first damage
against temperature for the chart. All of it is the float32 arithmetic of the research ports (R = research/thermal-fatigue-2026-10-29):
thermal_properties.construct loads the engine's Temperature block, thermal_core.rates gives each load counter's rate
at a temperature and thermal_counters.ticks_from_zero the exact tick its counter first overflows. No limit reads a
loader value that depends on expf/powf/logf (R/thermal_properties.py:68-71,122-133 use none), so the Python
stand-ins below do not change any of them. RB/SB (Engine_Overheat on), fresh counters, full health, standard
atmosphere. No modification carries a thermal block, so the limits do not depend on upgrades, throttle or mass.

Plots (Leo, 2026-11-01): each controllable radiator's factor on the cooling coefficient against opening, and the
heat index against throttle (heat(): the settled temperature at full cooling, from the engine state the Engine
performance page solves); from the same sweep, the tables by setting (throttle_tables: automatic and open radiators,
how long each lasts and how long until WEP is back). These read slots and use expf/powf, so they carry the
stand-ins' rounding (display only).
"""
import functools
import json
import math
import re
import struct
from pathlib import Path

import orjson

import thermal_validated
from thermal_core import f, add, sub, mul, div, rates, damage
from thermal_counters import evolve, ticks_from_zero
from thermal_properties import K, construct, density

DT = f(1 / 48)                      # the aircraft thermal update's step (R/ENGINE-THERMODYNAMICS.md:52-55)
RATINGS = (30., 10., 5., 1.)        # minutes
CAP_S = 36000.                      # chart: 10 h
ASSUMPTIONS = dict(game_mode='RB/SB (Engine_Overheat on)', counters='fresh', temperature='held constant', ambient_k=288.16, dt='1/48',
                   automatic_radiator='automatic control on; a radiator is available where Controls.hasRadiatorControl, '
                   'hasOilRadiatorControl or hasCommonRadiator gives it', wep='boost engaged, resource as loaded (Mass.MaxNitro)')
# Native family enum (R/evidence/installed-thermal-validation.json family_enum); the thermal model admits 0, 1, 2 and 5
# (R/validate_input_construction.py:45; R/ENGINE-THERMODYNAMICS.md:76-79).
FAMILIES = ('Inline', 'Radial', 'Jet', 'Rocket', 'Tow', 'TurboProp', 'PVRD', 'UNKNOWN')
ADMITTED = (0, 1, 2, 5)
SLOTS = 16
VECTORS = frozenset(('WaterThermalConductionCrossCoeffTable', 'OilThermalConductionCrossCoeffTable', 'DegradationTime_0_10', 'DegradationTime_10_5'))
# Values of a malformed block that the port's float32 helpers reject (native SSE code gives inf or nan instead).
DATA_ERRORS = (ValueError, OverflowError, ZeroDivisionError, TypeError, IndexError, KeyError, AttributeError, struct.error)
# The Zeppelin-like engines (no oil channel, nonzero water cross coupling) depend on stack memory in the temperature
# update (R/ENGINE-THERMODYNAMICS.md:591-598; R/REPORT.md:290-300), which no limit uses: every limit holds the
# temperature, and load counters read only their own channel (R/thermal_core.py:18-46). The heat index (settled) uses
# the computed oil coefficient there; the cross factor clamps, so any plausible stack value moves it by under 0.3 K.
# True excludes them anyway.
EXCLUDE_NO_OIL_CROSS = False


def bits(x): return struct.unpack('<I', struct.pack('<f', x))[0]
def unbits(b): return struct.unpack('<f', struct.pack('<I', b))[0]
def prev32(x): return unbits(bits(x) - 1)       # x > 0
def next32(x): return unbits(bits(x) + 1)
# The loader's own Celsius offset, f(273.15) (R/thermal_properties.py:8): config temperatures come back exactly.
def celsius(t): return None if t is None else t - K


def largest_true(pred, lo, hi):
    """Largest float32 x in [lo, hi] (0 < lo < hi) with pred(x), for pred true at lo, false at hi and true on a down-set
    (positive float32 bit patterns are in numeric order)."""
    a, b = bits(lo), bits(hi)
    while b - a > 1:
        m = (a + b) // 2
        if pred(unbits(m)): a = m
        else: b = m
    return unbits(a)


# Stand-ins for the game's CRT, used only by loader slots no limit reads, and by the display-only plots.
def expf(x):
    try: return f(math.exp(f(x)))
    except OverflowError: return math.inf
def logf(x):
    try: return f(math.log(f(x)))
    except (ValueError, OverflowError): return math.nan
def powf(x, y):
    try: return f(math.pow(f(x), f(y)))
    except (ValueError, OverflowError, ZeroDivisionError): return math.nan


# ---- Engines and their Temperature blocks, as the game selects them --------------------------------------------
def first(value):
    # A repeated BLK key decodes as a list; the native scalar getter takes its first value.
    return value[0] if isinstance(value, list) and value else value


def loader_block(block):
    """The loader's view of a raw Temperature block (R/validate_installed_thermal.py:39-56): equal repeated scalars
    collapse; unequal repeats are traced only for float Water/OilAirDensityPower (the first float is read)."""
    out = {}
    for k, v in block.items():
        if isinstance(v, dict):
            v = loader_block(v)
        elif isinstance(v, list) and k not in VECTORS:
            if not (v and all(type(x) == type(v[0]) and x == v[0] for x in v)) and not (
                    k in ('WaterAirDensityPower', 'OilAirDensityPower') and all(isinstance(x, float) for x in v)):
                raise ValueError('unverified repeated parameter ' + k)
            v = v[0]
        out[k] = v
    return out


def contiguous(fm, prefix):
    out = []
    for i in range(SLOTS):
        if not isinstance(fm.get(prefix + str(i)), dict):break
        out.append(prefix + str(i))
    return out


def flag(value):
    # A Controls switch, as the app reads it (em_manual_controls.flag): the first value, absent = False.
    return first(value) is True


def boost_delivered(block, kind, mass):
    """Whether WEP can deliver the boost coordinate 1.1 of a boost-controlled engine (R/validate_input_construction.py:
    48-54; R/REPORT.md:401-406): kinds 1/2/4/5/9 need the boost resource FM+0x531c >= f(0.001), as loaded
    Mass.MaxNitro (prop_catalog.py:113), and a compressor stage whose AfterburnerBoostMul_i x AfterburnerBoost
    (props+0x60+4i, props+0x1ec: R/validate_compressor_driver.py:42-47; piston_model.py:73,79,87-96) is above
    f(0.99); the stage gate covers every stage (0..props+0x198 = NumSteps-1) and none without stages."""
    if kind in (1, 2, 4, 5, 9) and not f(first(mass.get('MaxNitro', 0.))) >= f(.001): return False
    main = block.get('Main') if isinstance(block.get('Main'), dict) else {}
    compressor = block.get('Compressor') if isinstance(block.get('Compressor'), dict) else {}
    boost = f(first(main.get('AfterburnerBoost', 1.)))
    stages = range(int(first(compressor.get('NumSteps', 0))))
    return not stages or any(mul(f(first(compressor.get(f'AfterburnerBoostMul{i}', 1.))), boost) > f(.99) for i in stages)


def properties(block, mass):
    """The thermal fields of a selected engine property block."""
    main = block.get('Main') if isinstance(block.get('Main'), dict) else {}
    boost = block.get('Afterburner') if isinstance(block.get('Afterburner'), dict) else {}
    controls = block.get('Controls') if isinstance(block.get('Controls'), dict) else {}
    name = first(main.get('Type')); family = FAMILIES.index(name) if isinstance(name, str) and name in FAMILIES else -1
    water = bool(first(main.get('IsWaterCooled', False)))
    kind = first(boost.get('Type', 0)); controllable = bool(first(boost.get('IsControllable', False)))
    controlled = bool((controllable and kind > 0) or kind in (4, 8, 10))     # R/validate_input_construction.py:48
    # Radiator availability props+0x24c/0x24d/0x24e (water, oil, linked; R/validate_input_construction.py:66-67): the
    # same three switches as Controls (mapping inferred). Linked oil follows the water radiator.
    radiator = flag(controls.get('hasRadiatorControl'))
    oil = radiator if flag(controls.get('hasCommonRadiator')) else flag(controls.get('hasOilRadiatorControl'))
    return dict(family=name, family_index=family, water=water,
                temperature=block.get('Temperature') if isinstance(block.get('Temperature'), dict) else None,
                special=family in (2, 5),                           # loader special argument, inp[0]
                liquid0=water and family in (0, 1),                 # ch0 liquid flag inp[1] (R/validate_input_construction.py:68)
                boost_controlled=controlled,                        # target coordinate (engine_commands.py:2)
                wep=f(first(main.get('ThrottleBoost', 1.))) > 1.,   # pilot throttle reaches 110% (engine_performance.without_wep)
                throttle_boost=f(first(main.get('ThrottleBoost', 1.))),  # props+0x1e8, the WEP heat scale (heat_scale)
                boost_delivered=controlled and boost_delivered(block, kind, mass),
                radiator_control=None if family in (2, 5) else [radiator, oil])


def select(fm):
    """One record per distinct thermal engine of a raw FM tree, in engine order.

    The powerplant loader reads contiguous EngineType0..15 and Engine0..15. A valid instance Type index selects the
    shared type object, without merging the instance's own Temperature child; an absent or invalid Type loads a type
    from the instance block, within the 16-type capacity (R/UPSTREAM-THERMAL-TRACE.md:310-315;
    R/GEOMETRY-CONTINUATION.md:310-313; R/PREDAMAGE-MECHANICS-TRACE.md:431-434). Family (props+0), IsWaterCooled
    (props+0x23c) and the thermal properties (props+0x368) live in that one object (R/validate_upstream_boundaries.py:
    139-154), so all of them come from the selected block. Engines with the same Temperature block, family, water flag,
    boost (radiator target coordinate, WEP) and radiator switches share a record; excluded engines group by reason."""
    types = contiguous(fm, 'EngineType'); created = len(types); out = {}
    mass = fm.get('Mass') if isinstance(fm.get('Mass'), dict) else {}
    for index, key in enumerate(contiguous(fm, 'Engine')):
        t = first(fm[key].get('Type'))
        source = types[t] if isinstance(t, int) and not isinstance(t, bool) and 0 <= t < len(types) else key
        row = dict(index=index, key=key, family=None)
        if source == key:
            created += 1
        if created > SLOTS and source == key:
            row.update(admitted=False, reason='engine type capacity exceeded')
        else:
            try:
                row.update(properties(fm[source], mass), source=source)
            except DATA_ERRORS as error:                            # one malformed engine, not the whole aircraft
                main = fm[source].get('Main') if isinstance(fm[source].get('Main'), dict) else {}
                row.update(source=source, family=first(main.get('Type')), admitted=False, reason=f'invalid engine value: {error}')
            else:
                name, temperature = row['family'], row['temperature']
                if name is None: row.update(admitted=False, reason='no engine family')
                elif row['family_index'] not in ADMITTED: row.update(admitted=False, reason=f'{name}: no thermal model')
                elif temperature is None: row.update(admitted=False, reason='no Temperature block')
                else: row.update(admitted=True)
        fields = ('temperature', 'family', 'water', 'boost_controlled', 'wep', 'throttle_boost', 'boost_delivered', 'radiator_control')
        group = json.dumps([row['admitted'], *([row[k] for k in fields] if row['admitted'] else [row['family'], row['reason']])],
                           sort_keys=True, default=str)
        if group in out:
            out[group]['indices'].append(index); out[group]['keys'].append(key)
        else:
            out[group] = dict(row, indices=[index], keys=[key])
    return list(out.values())


# ---- Counters at a held temperature -----------------------------------------------------------------------------
def _rho_full():
    """Smallest float32 rate with f(1 + f(rate*dt)) > 1: below it no counter in any state can overflow; at dt = f(1/48)
    the float32 after 2.86102294921875e-6 (R/ENGINE-THERMODYNAMICS.md:544-550)."""
    return next32(largest_true(lambda r: add(1., mul(r, DT)) <= 1., f(1e-30), f(1.)))


RHO_FULL = _rho_full()


@functools.lru_cache(maxsize=256)          # ratings come from requests: bounded
def rho_star(ticks):
    """Smallest float32 rate whose counter, from 0, first overflows at a tick <= ticks. ticks_from_zero does not
    increase with the rate (float32 rounding is monotone), so the rates that overflow by then are [rho_star, inf).
    The first possible overflow is tick 1, so below one tick no rate does (inf); from one tick on, f(1e6) does."""
    if ticks < 1: return math.inf
    def slower(r):
        n = ticks_from_zero(r, DT)
        return n is None or n > ticks
    return next32(largest_true(slower, RHO_FULL, f(1e6)))


def peak(rows, work, recover, t):
    """Largest counter rate at float32 t (K). Full health: the effective temperature is t (R/thermal_core.py:40)."""
    return max(rates(rows, work, recover, t)[:len(work)], default=0.)    # no load rows: counters never move


def first_damage(rows, work, recover, t):
    """Tick of the first damage from fresh counters held at t, or None. Each counter's overflow tick does not increase
    with its rate, and any overflow is damage (R/thermal_core.py:42-46; R/REPORT.md:119-133), so it is the tick of the
    largest rate."""
    return ticks_from_zero(peak(rows, work, recover, t), DT)


def overflow_damages(r, work):
    """Every overflow lowers health: the damage proposal (R/thermal_core.py:48-51) and the caller's 0.68h+0.32
    (R/REPORT.md:176-182) stay below 1 at the smallest and largest rates that overflow (both linear in the rate)."""
    s = r['slots']; work = [w for w in work if w > 0.]
    for rate in (RHO_FULL, max(work)) if work else ():
        h = damage(1., rate, DT, s['0x1e8'], s['0x1ec'])
        if not (h < 1. and min(1., add(mul(h, .6800000071525574), .3199999928474426)) < 1.):return False
    return True


def channel(r, c, liquid, label, work, recover, ratings):
    """Limits of one channel. Rates do not decrease with temperature (R/thermal_core.py:18-34 with work and recover
    reciprocals >= 0), so each limit is a bisection over float32 temperatures. All counters recover at or below the
    first load row and run at full work at or above the last, so the search ends there, or just below boiling on a
    liquid channel: it is clamped at its boiling point and boils (R/thermal_temperature.py:81-82;
    R/ENGINE-THERMODYNAMICS.md:383-398). bound: 'load' (a load row binds), 'boiling' (it boils first), 'none'."""
    rows = r['water_rows'] if c == 0 else r['oil_rows']; s = r['slots']
    # Thermostat and boiling apply to liquid channels only (R/thermal_temperature.py:72-75,81).
    boil = s[hex(0x1d0 + 4 * c)] if liquid else None
    out = dict(label=label, thermostat_c=celsius(s[hex(0x1e0 + 4 * c)]) if liquid else None, boiling_c=celsius(boil))
    if len(rows) < 2:                                       # every rate <= 0 (R/thermal_core.py:21-23)
        none = dict(max_c=None, bound='none')
        return dict(out, continuous=none, ratings=[dict(minutes=m, **none) for m in ratings], curve=[], floor_s=None)
    lo, hi = rows[0][0], rows[-1][0]
    top = min(hi, prev32(boil)) if liquid else hi
    over = 'boiling' if liquid and boil <= hi else 'none'
    held = lambda t: peak(rows, work, recover, t)

    def limit(rho):
        if top < lo: return dict(max_c=None, bound='boiling'), None
        if held(top) < rho: return dict(max_c=None, bound=over), None
        t = largest_true(lambda x: held(x) < rho, lo, top)
        return dict(max_c=celsius(t), bound='load'), t

    continuous, start = limit(RHO_FULL)
    # Rating R: first damage after tick R*2880, i.e. none within R minutes.
    out.update(continuous=continuous, ratings=[dict(minutes=m, **limit(rho_star(m * 2880.))[0]) for m in ratings])
    n = first_damage(rows, work, recover, top) if top >= lo else None
    out.update(floor_s=None if n is None else n / 48., curve=[] if start is None else curve(rows, work, recover, start, top))
    return out


def curve(rows, work, recover, start, top, dln=.08, dk=.02):
    """[[C, s]] from the continuous limit to the top, s = exact ticks/48 to first damage, at most 10 h. Breakpoints:
    the ends, the 10 h crossing, both sides of every load row (steps) and every change of the dominant counter (the
    interpolated counter's rate passing the largest full-work rate). Time is monotone between them; segments are
    halved until neighbouring times differ by at most 8% or 0.02 K. A 0.1 C grid would need thousands of points on
    a jet's head channel."""
    held = lambda t: peak(rows, work, recover, t)
    def secs(t):
        n = first_damage(rows, work, recover, t)
        return CAP_S if n is None else min(CAP_S, n / 48.)
    if start >= top: return [[celsius(start), secs(start)]]
    points = {start, top}
    cap = rho_star(CAP_S * 48.)
    if held(top) >= cap > held(start):
        t = largest_true(lambda x: held(x) < cap, start, top); points.update((t, next32(t)))
    points.update(x for row in rows for x in (row[0], next32(row[0])) if start < x < top)
    for k in range(1, len(rows)):
        p = min(rows[k][2], len(work))
        if not 0 < p < len(work): continue
        plateau = max(work[:p]); a, b = max(rows[k - 1][0], start), min(prev32(rows[k][0]), top)
        rate = lambda x: rates(rows, work, recover, x)[p]
        if a < b and rate(a) < plateau <= rate(b):
            t = largest_true(lambda x: rate(x) < plateau, a, b); points.update((t, next32(t)))
    points = sorted(points); value = {t: secs(t) for t in points}; out = [points[0]]
    def split(a, b, depth=0):
        if depth < 12 and b - a > dk and abs(math.log(value[a] / value[b])) > dln:
            m = f((a + b) / 2.)
            if a < m < b:
                value[m] = secs(m); split(a, m, depth + 1); split(m, b, depth + 1); return
        out.append(b)
    for a, b in zip(points, points[1:]): split(a, b)
    return [[celsius(t), value[t]] for t in out]


# ---- Automatic radiator -----------------------------------------------------------------------------------------
def target(rows, x, c):
    """The automatic radiator's target temperature (K) at coordinate x: the RadiatorMode curve with clamped ends
    (R/thermal_temperature.py:16-21)."""
    if not rows: return 0.
    if len(rows) == 1 or x <= rows[0][0]: return rows[0][2 + c]
    if x >= rows[-1][0]: return rows[-1][2 + c]
    k = next(i for i, r in enumerate(rows) if r[0] >= x)
    return add(rows[k - 1][2 + c], mul(sub(rows[k][2 + c], rows[k - 1][2 + c]), mul(sub(x, rows[k - 1][0]), rows[k - 1][1])))


@functools.cache
def opening_step():
    """Smallest float32 demand that opens the radiator one level, the largest over levels: the command
    clamp(applied + demand*dt) is applied as trunc(command*255)/255 each update (R/validate_controls.py:29-30,55;
    R/ENGINE-THERMODYNAMICS.md:426-450), so a smaller positive demand is discarded: 0.18823530, about 48/255."""
    inv = f(1 / 255)
    def stays(q, d):
        command = min(1., max(0., add(mul(f(float(q)), inv), mul(d, DT))))
        return min(255, int(mul(command, 255.))) <= q
    return max(next32(largest_true(lambda d: stays(q, d), f(1e-30), f(.5))) for q in range(255))


def residual(t):
    """Largest float32 |dT/dt| that leaves t > 0 unchanged by the update t + dT/dt*dt (R/thermal_temperature.py:80): a
    temperature at rest can still carry a derivative that large, which the radiator demand reads. Whether t - e rounds
    back to t depends only on the gap below t (its ulp, half that at a power of two) and on t's parity (ties to even),
    so there is one search per binade and parity."""
    b = bits(t)
    return _residual(b >> 23, b & 0x7fffff == 0, b & 1)


@functools.cache
def _residual(exponent, power, odd):
    t = unbits(exponent << 23 | (0 if power else 2 + odd))
    return largest_true(lambda d: add(mul(-d, DT), t) == t, f(1e-30), f(1.))


def resting(t):
    """Top of the automatic resting band: the largest float32 T whose demand clamp(0.2*(T - target) + 3*dT/dt, +-0.5)
    (R/thermal_temperature.py:84-88, the same derivative as the update) stays below one opening step while T does not
    change, with the most negative such derivative: about target + 0.952 K (0.941 K from the step, 48/255/0.2,
    R/ENGINE-THERMODYNAMICS.md:426-450, and up to 3*ulp(T)/(2*dt)/0.2 from the derivative)."""
    step = opening_step()
    return largest_true(lambda x: min(.5, max(-.5, add(mul(sub(x, t), .2), mul(-residual(x), 3.)))) < step, t, f(t + 2.))


def radiator(r, rec, channels, work, recover):
    """Automatic radiator rows of an ordinary engine (RadiatorMode rows load only when not special,
    R/thermal_properties.py:128-132): each RadiatorMode throttle, 100% and WEP (110%) where the throttle reaches them,
    the target of each channel and the time to first damage with every channel held at the top of its resting band.
    Coordinate (R/validate_input_construction.py:47-57; R/REPORT.md:401-409): the throttle; for boost-controlled
    engines min(throttle, 1), or 1.1 while the boost is delivered. A channel whose radiator is not available keeps
    its opening at 1, not at its target (R/validate_input_construction.py:66-67; R/validate_controls.py:58): it has
    no target here and does not count in the time; with neither available there are no rows. A linked radiator follows
    the larger demand (R/REPORT.md:417-418), so each channel stays at or below its own band."""
    rows = r['radiator_rows']; auto = [c for c in range(channels) if rec['radiator_control'][c]]
    if not rows or not auto: return []
    one, wep = f(1.), f(1.1)
    reach = lambda x: x <= one or rec['wep'] and (x == wep and rec['boost_delivered'] if rec['boost_controlled'] else x <= wep)
    tables = (r['water_rows'], r['oil_rows'])
    boil = boiling(r, rec, channels)
    out = []
    for x in sorted({row[0] for row in rows} | {one} | ({wep} if rec['wep'] else set())):
        if not reach(x): continue
        targets = [target(rows, x, c) if c in auto else None for c in range(channels)]; held = [None] * channels; boils = False
        for c in auto:
            held[c] = resting(targets[c])
            held[c], hot = clamped(held[c], boil[c]); boils = boils or hot
        ticks = [n for n in (first_damage(tables[c], work, recover, held[c]) for c in auto) if n is not None]
        out.append(dict(coordinate=x, label='WEP' if x == wep and rec['wep'] else '%d%%' % round(x * 100),
                        targets_c=[celsius(t) for t in targets], held_c=[celsius(t) for t in held],
                        seconds=min(ticks) / 48. if ticks else None, boils=boils))
    return out


# ---- Pilot tables (Leo, 2026-11-02): by gauge reading and by setting ----------------------------------------------
# Exact float32 counter evolution (thermal_counters.evolve) at constant rates, phase by phase: a counter's value after
# m updates, how long it then lasts at another rate, and the fewest updates at one set of rates after which every
# counter lasts a given time again at another.
LIMIT = round(CAP_S * 48)           # the longest wait searched: 10 h of updates


def after(c, rate, m):
    """A counter's value after m updates at a constant rate from c (clamped to [0, 1] as the game does), or None if it
    overflows on the way."""
    if m <= 0: return c
    n, state, v = evolve(c, rate, DT, horizon=m)
    return None if state == 'overflow' else 0. if state == 'zero' else v


def lasts_from(c, rate):
    """Updates from counter value c to its overflow (the overflow update) at a constant rate, or None (never)."""
    if not rate > 0.: return None
    n, state, _ = evolve(c, rate, DT)
    return n if state == 'overflow' else None


@functools.lru_cache(maxsize=4096)       # channels often share rates at their band tops (oil = water - 10 K)
def cap(rate, need):
    """The largest float32 counter value from which a counter at `rate` lasts at least `need` updates (1 where it never
    overflows), or None where even an empty one does not."""
    ok = lambda c: (lambda n: n is None or n >= need)(lasts_from(c, rate))
    if ok(1.): return 1.
    if not ok(0.): return None
    tiny = unbits(1)
    return largest_true(ok, tiny, 1.) if ok(tiny) else 0.


def first_m(pred, limit=LIMIT):
    """Smallest m in [0, limit] with pred(m), for pred false and then true; None if pred(limit) is false."""
    if pred(0): return 0
    a, b = 0, 1
    while not pred(b):
        if b >= limit: return None
        a, b = b, min(2 * b, limit)
    while b - a > 1:
        m = (a + b) // 2
        if pred(m): b = m
        else: a = m
    return b


def fine(c, rate, top, m):
    """A counter (value c, constant rate) is at or below its cap after m updates, without overflowing."""
    v = after(c, rate, m)
    return v is not None and v <= top


def back(counters, limit=LIMIT):
    """Fewest updates m after which every counter (value, rate, cap) is at or below its cap, or None (not within the
    limit). The rates are constant within a phase, so a draining counter is fine from some m on and a filling one up to
    some m."""
    need, until = 0, limit
    for c, rate, top in counters:
        if top is None: return None
        if c <= top:
            if rate > 0.:
                passes = first_m(lambda m: not fine(c, rate, top, m), limit)
                if passes is not None: until = min(until, passes - 1)
            continue
        if not rate < 0.: return None
        m = first_m(lambda m: fine(c, rate, top, m), limit)
        if m is None: return None
        need = max(need, m)
    return need if need <= until else None


def counter_rates(rows, work, recover, t):
    return rates(rows, work, recover, t)[:len(work)]


def used(rows, work, recover, t, ticks):
    """Every counter of a channel after `ticks` updates held at t (float32 K) from fresh."""
    return [after(0., rate, ticks) for rate in counter_rates(rows, work, recover, t)]


def gauge(r, c, liquid, work, recover):
    """By gauge reading: the held temperatures of one channel in bands of equal time to first damage from a fresh
    engine and, for each band, how cool to get and for how long to have all of its time back.

    The time to first damage does not increase with temperature and sits at a counter's full-work fill time on each
    plateau, so after the continuous limit there is one band per distinct fill time n, longest first, up to the hottest
    temperature that still lasts n updates (its time is that temperature's: the band's minimum). Recovery: every
    counter after one update short of that time at the band's top (a full use), then the fewest updates at a cooler
    temperature after which the band lasts its time again (back). That is fastest at or below the first load row (every
    counter at its full recover rate); the hottest temperature with the same time is a bisection, as back does not
    decrease with temperature. Liquid channels end just below boiling (channel)."""
    rows = r['water_rows'] if c == 0 else r['oil_rows']; s = r['slots']
    if len(rows) < 2: return None
    lo, hi = rows[0][0], rows[-1][0]
    boil = s[hex(0x1d0 + 4 * c)] if liquid else None
    top = min(hi, prev32(boil)) if liquid else hi
    if top < lo: return None
    ticks = lambda t: first_damage(rows, work, recover, t)
    lasting = lambda n: lambda t: (lambda k: k is None or k >= n)(ticks(t))
    below = top if ticks(top) is None else largest_true(lambda t: ticks(t) is None, lo, top)
    bands = []
    for n in sorted({n for n in (ticks_from_zero(w, DT) for w in work if w > 1e-8) if n is not None}, reverse=True):
        if below >= top: break
        t = top if lasting(n)(top) else largest_true(lasting(n), below, top)
        if t <= below: continue
        k = ticks(t)
        state = used(rows, work, recover, t, k - 1)
        caps = [cap(rate, k) for rate in counter_rates(rows, work, recover, t)]
        phase = lambda tc: list(zip(state, counter_rates(rows, work, recover, tc), caps))
        wait = back(phase(lo))
        # Not slower than at the first load row: every counter is fine after exactly that many updates.
        same = lambda tc: all(fine(v, rate, q, wait) for v, rate, q in phase(tc))
        cool = None if wait is None else t if same(t) else largest_true(same, lo, t)
        bands.append(dict(above_c=celsius(below), to_c=None if t >= hi else celsius(t), lasts_s=k / 48.,
                          recover_c=celsius(cool), recover_s=None if wait is None else wait / 48.))
        below = t
    return dict(bands=bands, boils_c=celsius(boil) if liquid and top < hi else None)


# ---- Radiator and heat ------------------------------------------------------------------------------------------
OPENINGS = tuple(range(101))        # radiator opening, %
AMBIENT = f(288.1600036621094)      # the reference global B and the current global A (R/ENGINE-THERMODYNAMICS.md:286-290)


def radiator_factor(s, c, opening, extra=1.):
    """Channel c's radiator factor on the cooling coefficient at an opening in [0, 1] (R/thermal_temperature.py:43-45):
    b*exp((-0.9r - 0.1)*RadiatorEffectPower) + a*E, calibrated to MinRadiatorThermalConductionCoeff closed and 1 open
    with E = 1 (R/ENGINE-THERMODYNAMICS.md:201-202,316-323). E is EngineProperties+0x240, which only the modification
    parameter radiatorEffMul assigns (R/UPSTREAM-THERMAL-TRACE.md:330,347-350); new_radiator carries it with
    invertEnableLogic, so it applies only while the upgrade is missing: fully upgraded, E keeps its default, 1."""
    return add(mul(expf(mul(add(mul(f(opening), -.9), -.1), s[hex(0x180 + 4 * c)])), s[hex(0x18c + 8 * c)]), mul(s[hex(0x188 + 8 * c)], extra))


def radiator_curves(r, rec, channels):
    """Factor against opening of each channel whose radiator the pilot controls (ordinary engines only: special ones
    have no radiator term). A channel without control stays fully open (R/validate_input_construction.py:66-67)."""
    if rec['special']: return None
    shown = [c for c in range(channels) if rec['radiator_control'][c]]
    if not shown: return None
    return dict(opening_percent=list(OPENINGS), channels=shown,
                factor=[[radiator_factor(r['slots'], c, x / 100.) for x in OPENINGS] for c in shown])


def interval(x, x0, x1, y0, y1):
    """R/thermal_temperature.py:9-14."""
    if x0 > x1: x0, x1, y0, y1 = x1, x0, y1, y0
    if x <= x0: return y0
    if x >= x1: return y1
    d = sub(x1, x0)
    return add(y0, div(mul(sub(x, x0), sub(y1, y0)), d) if abs(d) > f(4e-19) else 0.)


def mode_value(r, x, index):
    """heat_inputs' lookup (R/thermal_temperature.py:23-33) of a mode output (index 2/3: water h/D, 4/5: oil h/D) at the
    driving coordinate x: linear between Mode rows, scaled by x over the end row's coordinate beyond them. Native code
    reads that coordinate at the mode ID as a row index; heat() admits only blocks where every ID is its row."""
    rows, modes = r['mode_rows'], r['modes']
    if not rows: return 0.
    if len(rows) == 1 or x <= rows[0][0]: row = rows[0]
    elif x >= rows[-1][0]: row = rows[-1]
    else:
        k = next(i for i, q in enumerate(rows) if q[0] >= x)
        lo, hi = modes[rows[k - 1][2]][index], modes[rows[k][2]][index]
        return add(mul(sub(hi, lo), mul(sub(x, rows[k - 1][0]), rows[k - 1][1])), lo)
    d = rows[row[2]][0]
    return mul(div(x, d) if abs(d) > f(4e-19) else 0., modes[row[2]][index])


def heat_scale(rec, throttle):
    """The caller's heat scale on the compressor coordinate of an ordinary engine (R/validate_input_construction.py:
    57-63; R/REPORT.md:407-409): 1 up to throttle 1, ThrottleBoost (props+0x1e8) from 1.1, linear between; 1 on a
    boost-controlled engine."""
    if rec['boost_controlled'] or not throttle > 1.: return 1.
    m = rec['throttle_boost']
    return add(mul(add(mul(m, 9.999998092651367), -9.999998092651367), sub(throttle, 1.)), 1.) if throttle < f(1.1) else m


def settled(r, x, altitude_m, channels, opening=(1., 1.)):
    """Heat index (C) of each channel at heat coordinate x: the temperature at which the update leaves it unchanged
    (R/thermal_temperature.py:47-80, dT/dt = 0) at full cooling: radiator open (E = 1; `opening` sets each channel's
    radiator for the closed bound of throttle_tables), thermostat open, full fluid,
    fast flight (the flow term decays to nothing: the sink is the ambient) and the ambient AMBIENT, with the air
    density at altitude_m. There dT/dt = [(Ta - T)k + (D - (T - Ta))h]/inertia, so T = Ta + hD/(h + k): float64 from
    the float32 terms. At a Mode row above the thermostat setpoint, or on an air-cooled head, this is that row's
    temperature less 0.5(700 - (T - Ta))/699.5 K, 0.3-0.5 K (its calibration takes the ambient B + 0.5 K,
    R/ENGINE-THERMODYNAMICS.md:146-163); a row below the setpoint is calibrated with the thermostat closed
    (R/thermal_properties.py:107), so the index reads far below it. None where the channel has no heat calibration
    (h = 0 in every row)."""
    s = r['slots']; rho = density(altitude_m); q = []
    for c in (0, 1):
        level = add(mul(sub(1., s[hex(0x1a0 + 4 * c)]), 1.), s[hex(0x1a0 + 4 * c)])
        thermostat = add(mul(sub(1., s[hex(0x198 + 4 * c)]), 1.), s[hex(0x198 + 4 * c)])
        q.append(mul(mul(thermostat, mul(level, powf(rho, s[hex(0x1c8 + 4 * c)]))), radiator_factor(s, c, opening[c])))
    out = []
    for c in range(channels):
        if not any(m[2 + 2 * c] for m in r['modes']): out.append(None); continue
        h, d = mode_value(r, x, 2 + 2 * c), mode_value(r, x, 3 + 2 * c)
        cross = interval(q[1 - c], s[hex(0x1a8 + 16 * c)], s[hex(0x1ac + 16 * c)], s[hex(0x1b0 + 16 * c)], s[hex(0x1b4 + 16 * c)])
        k = mul(s[hex(0x14c + 4 * c)], add(mul(sub(q[1 - c], q[c]), cross), q[c]))
        out.append(AMBIENT + h * d / (h + k) - K if h + k > 0. else None)
    return out


HEAT_ALTITUDE_M, HEAT_SPEED_KMH = 1000., 300.   # the Engine performance page's defaults (app/engine.js)
HEAT_STEP = 2.                                  # throttle %


def boiling(r, rec, channels):
    """Each channel's boiling point (K), None on an air-cooled head (channel 0 is liquid by liquid0, oil always)."""
    return [r['slots'][hex(0x1d0 + 4 * c)] if liquid else None for c, liquid in enumerate((rec['liquid0'], True)[:channels])]


def clamped(t, boil):
    """A held temperature (float32 K) and whether it boils: a liquid channel is clamped at its boiling point and boils
    (R/thermal_temperature.py:81-82), taken just below it as channel() and radiator() do."""
    return (prev32(boil), True) if boil is not None and t >= boil else (t, False)


def held_s(r, c, temperature_c, boil=None):
    """Seconds to first damage from a fresh engine with channel c held at temperature_c (clamped below boil), None
    where it never comes or there is no value."""
    if temperature_c is None: return None
    work = [w for w, _ in r['loads']]; recover = [x for _, x in r['loads']]
    n = first_damage((r['water_rows'], r['oil_rows'])[c], work, recover, clamped(f(temperature_c + K), boil)[0])
    return None if n is None else n / 48.


def throttle_tables(rec, r, channels, throttles, points, coordinates, altitude_m):
    """By setting, at the heat sweep's altitude and speed: what each channel settles at, how long to first damage from a
    fresh engine, and how long after a full run at the hottest setting (WEP, or 100% without it) it lasts that long
    again. Modes: the automatic radiator, and radiators fully open (one mode, 'fixed', where the pilot moves none).

    Radiator open is the heat index. Automatic: each controlled channel rests at the top of its target band
    (resting(target), as radiator()), unless even the open radiator runs hotter (then it is the heat index) or the
    closed one cooler (the heat index with that channel's radiator closed, both controlled channels closed: the
    hottest bound, as the cross term only adds cooling for an opener radiator); an uncontrolled channel stays open
    (R/validate_input_construction.py:66-67). A linked radiator follows the larger demand (R/REPORT.md:417-418), so
    its cooler channel only runs cooler than shown. Target coordinate as radiator(): the throttle up to 100%, 1.1 at WEP.

    Rows: for the no-damage level and then each counter's fill time, longest first, the highest swept throttle up to
    which every solved point lasts at least that long ('<= t%': its hottest temperatures and shortest time), then 100%
    and WEP. The wait is back(): every counter of each channel one update short of the hottest row's time, then the
    row's temperatures; it rounds up to whole seconds on the page. A hottest row that never damages has no wait. A
    liquid channel at or above its boiling point is held just below it and the row boils (clamped)."""
    work = [w for w, _ in r['loads']]; recover = [x for _, x in r['loads']]
    tables = (r['water_rows'], r['oil_rows'])[:channels]
    control = list(rec['radiator_control'] or (False, False))[:channels]
    if not all(len(rows) >= 2 for rows in tables): return None
    one, wep = f(1.), f(1.1)
    reach = lambda x: x <= one or rec['wep'] and (x == wep and rec['boost_delivered'] if rec['boost_controlled'] else x <= wep)
    auto = bool(r['radiator_rows']) and any(control)
    boil = boiling(r, rec, channels)
    ticks = lambda temps: min((n for n in (first_damage(rows, work, recover, t) for rows, t in zip(tables, temps)) if n is not None), default=None)
    sweep = {'auto': [], 'open': []}
    for t, p, x in zip(throttles, points, coordinates):
        x_t = f(t / 100.)
        if p is None or x is None or any(v is None for v in p['heat_c'][:channels]) or not reach(x_t): continue
        if t > 100. and not (rec['wep'] and x_t == wep): continue           # 100% and WEP: the pilot's settings
        opened = [f(v + K) for v in p['heat_c'][:channels]]
        sweep['open'].append((t, opened))
        if auto:   # no closed value (no heat calibration): no cooler bound
            closed = [math.inf if v is None else f(v + K) for v in settled(r, x, altitude_m, channels, [0. if on else 1. for on in control] + [1.] * (2 - channels))]
            temps = [min(max(resting(target(r['radiator_rows'], x_t, c)), o), k) if control[c] else o
                     for c, (o, k) in enumerate(zip(opened, closed))]
            sweep['auto'].append((t, temps))
    held = lambda temps: [clamped(t, b) for t, b in zip(temps, boil)]
    sweep = {mode: [(t, [h for h, _ in held(temps)], any(b for _, b in held(temps))) for t, temps in run] for mode, run in sweep.items()}
    levels = [None] + sorted({n for n in (ticks_from_zero(w, DT) for w in work if w > 1e-8) if n is not None}, reverse=True)
    lasting = lambda n, level: n is None if level is None else n is None or n >= level
    out = []
    for mode, run in sweep.items():
        if not run or mode == 'auto' and not auto: continue
        flat = [(t, temps, ticks(temps), boils) for t, temps, boils in run]
        grid = [q for q in flat if q[0] <= 100.]
        rows, reached = [], -1.
        for level in levels:
            prefix = []
            for q in grid:
                if not lasting(q[2], level): break
                prefix.append(q)
            if not prefix or prefix[-1][0] <= reached or prefix[-1][0] >= 100.: continue
            reached = prefix[-1][0]
            temps = [max(q[1][c] for q in prefix) for c in range(channels)]
            times = [q[2] for q in prefix]
            rows.append(dict(label='≤ %d%%' % round(reached), temps=temps, boils=any(q[3] for q in prefix),
                             n=None if all(n is None for n in times) else min(n for n in times if n is not None)))
        rows += [dict(label='%d%%' % round(t) if t <= 100. else 'WEP', temps=temps, n=n, boils=boils)
                 for t, temps, n, boils in flat if t == 100. or t > 100.]
        hot = rows[-1] if rows and rows[-1]['label'] in ('WEP', '100%') and rows[-1]['n'] is not None else None
        if hot:
            k = hot['n']
            state = [used(rows_c, work, recover, t, k - 1) for rows_c, t in zip(tables, hot['temps'])]
            caps = [[cap(rate, k) for rate in counter_rates(rows_c, work, recover, t)] for rows_c, t in zip(tables, hot['temps'])]
            for row in rows[:-1]:
                wait = back([(v, rate, q) for c, (rows_c, t) in enumerate(zip(tables, row['temps']))
                             for v, rate, q in zip(state[c], counter_rates(rows_c, work, recover, t), caps[c])])
                row['back_s'] = None if wait is None else wait / 48.
        out.append(dict(mode=mode if control and any(control) else 'fixed', hot=hot['label'] if hot else None,
                        rows=[dict(label=row['label'], temps_c=[celsius(t) for t in row['temps']], boils=row['boils'],
                                   lasts_s=None if row['n'] is None else row['n'] / 48., back_s=row.get('back_s')) for row in rows]))
    return out or None


def heat_bases(fm):
    """{engine index: (record, construct result, channels)} of the ordinary engines whose Mode rows load unfiltered."""
    out = {}
    for rec in select(fm):
        if not rec['admitted'] or rec['special']: continue
        try: r, skipped = load(rec['temperature'], False, rec['water'])
        except DATA_ERRORS: continue
        if skipped or not r['mode_rows'] or any(row[2] != k for k, row in enumerate(r['mode_rows'])): continue
        for i in rec['indices']: out[i] = (rec, r, 2 if r['has_oil'] else 1)
    return out


def alike(a, b, tol=.005):
    """Equal to the page's precision: the same shape, numbers within tol (C; seconds are whole updates, 1/48 s apart,
    so within tol they are equal)."""
    if isinstance(a, dict): return isinstance(b, dict) and a.keys() == b.keys() and all(alike(a[k], b[k], tol) for k in a)
    if isinstance(a, (list, tuple)): return isinstance(b, (list, tuple)) and len(a) == len(b) and all(alike(x, y, tol) for x, y in zip(a, b))
    if isinstance(a, float) and isinstance(b, (int, float)) and not isinstance(b, bool): return abs(a - b) <= tol
    return a == b


def same_curve(a, b):
    """Two engines' heat curves equal to the plot's precision: the same points solved, RPM within 0.05, manifold within
    1e-4 ata, heat index within 0.005 C and the same time to first damage. Twins with counter-rotating propellers differ
    only by solver rounding (0.001 rpm and 4e-5 C, 2026-11-01)."""
    close = lambda x, y, tol: (x is None) == (y is None) and (x is None or abs(x - y) <= tol)
    return len(a) == len(b) and all(
        (p is None) == (q is None) and (p is None or close(p['rpm'], q['rpm'], .05) and close(p['manifold_ata'], q['manifold_ata'], 1e-4)
                                        and alike(p['heat_c'], q['heat_c']) and alike(p['lasts_s'], q['lasts_s']))
        for p, q in zip(a, b))


def grouped(curves, tables):
    """[{engines, points, tables}]: engines in order, each joining the first group whose curve (same_curve) and tables
    are the same; a group shows its first engine's. Engines with different thermal blocks split even on one curve."""
    groups = []
    for i, points in sorted(curves.items()):
        group = next((g for g in groups if same_curve(g['points'], points) and alike(g['tables'], tables[i])), None)
        if group: group['engines'].append(i)
        else: groups.append(dict(engines=[i], points=points, tables=tables[i]))
    return groups


@functools.lru_cache(maxsize=32)
def _heat(path, stamp, aircraft_id, altitude_m, speed_kmh):
    import numpy as np
    import engine_performance as ep
    fm = orjson.loads(Path(path).read_bytes())
    bases = heat_bases(fm)
    _, prop = ep.admitted(aircraft_id, 100., None)
    top = 100. if aircraft_id in ep.without_wep() else 110.
    throttles = [top * i / round(top / HEAT_STEP) for i in range(round(top / HEAT_STEP) + 1)]
    curves = {i: [] for i in bases} if prop else {}
    coordinates = {i: [] for i in curves}
    count = len(contiguous(fm, 'Engine'))
    with ep.LOCK:
        for t in throttles if curves else ():
            chart = ep.solver(aircraft_id, altitude_m, t, None)    # outside the try, as engine_performance.compute: a
            try: row = ep.propeller_row(chart, speed_kmh, None)     # solver the page cannot build is a ValueError (400)
            except (ValueError, ArithmeticError, np.linalg.LinAlgError): row = None     # as engine_performance.axial_point
            engines = chart.engine.properties['engines'] if row else []
            pistons = [j for j, e in enumerate(engines) if e['family'] in (0, 1)]
            for i, (rec, r, channels) in bases.items():
                point = x = None
                # The solver lists every FM engine in order; the heat input is the engine's own compressor output
                # (Engine+0x12c, R/UPSTREAM-THERMAL-TRACE.md:58-60), the state's manifold, at its angular speed.
                if len(engines) == count and i in pistons:
                    rpm, manifold = row['rpm'][i], row['manifold_ata'][pistons.index(i)]
                    if math.isfinite(rpm) and math.isfinite(manifold):
                        x = mul(mul(heat_scale(rec, f(t / 100.)), f(manifold)), f(rpm * math.pi / 30.))
                        heat_c = settled(r, x, altitude_m, channels)
                        boil = boiling(r, rec, channels)
                        point = dict(rpm=rpm, manifold_ata=manifold, heat_c=heat_c, lasts_s=[held_s(r, c, v, boil[c]) for c, v in enumerate(heat_c)])
                curves[i].append(point); coordinates[i].append(x)
    tables = {}
    for i in curves:
        rec, r, channels = bases[i]
        try: tables[i] = throttle_tables(rec, r, channels, throttles, curves[i], coordinates[i], altitude_m)
        except DATA_ERRORS: tables[i] = None
    return dict(throttle_percent=throttles, altitude_m=altitude_m, speed_kmh=speed_kmh, engines=grouped(curves, tables))


def heat(aircraft_id, altitude_m=HEAT_ALTITUDE_M, speed_kmh=HEAT_SPEED_KMH):
    """Heat index against throttle (every HEAT_STEP % up to the engine page's throttle stop) of each ordinary engine, at
    the engine state the Engine performance page solves at altitude_m and speed_kmh (engine_performance.solver and
    propeller_row: auto pitch, angle of attack 0, 1 g; each point solved from cold, as the page's sweeps start: some
    do not converge, and are None). Engines with the same curve (same_curve) share one.
    ValueError where the engine page cannot calculate the aircraft. Cached on the FM file."""
    from aircraft_catalog import source
    path = source(aircraft_id); stat = path.stat()
    return dict(_heat(str(path), (stat.st_mtime_ns, stat.st_size), aircraft_id, float(altitude_m), float(speed_kmh)), aircraft_id=aircraft_id)


# ---- One aircraft -----------------------------------------------------------------------------------------------
def load(block, special, water):
    """construct() on the loader's view of the block; True when its Mode rows had to be left out. The port raises where
    native SSE arithmetic gives inf or nan (f() beyond the float32 range, a division by zero). Mode rows feed only the
    mode, heat and flow slots (R/thermal_properties.py:78-120); no limit reads them (lines 58-75 and 122-133), so a
    block whose Mode rows fail loads again without them, with every limit unchanged (heat_bases leaves such blocks out)."""
    data = loader_block(block)
    try: return construct(data, special, water, powf, expf, logf), False
    except DATA_ERRORS as error:
        try: return construct({k: v for k, v in data.items() if not re.fullmatch(r'Mode\d+', k)}, special, water, powf, expf, logf), True
        except DATA_ERRORS: raise error from None


def engine(rec, ratings):
    special, water = rec['special'], rec['water']
    out = dict(engines=rec['indices'], family=rec['family'], water_cooled=rec['liquid0'],
               # Whether this exact loader input is one of the 761 the research ran through the game's own loader.
               verified=thermal_validated.verified(rec['temperature'], special, water))
    try:   # water = Main.IsWaterCooled, as the research's native comparison passed it (R/validate_installed_thermal.py:31,57)
        r, skipped = load(rec['temperature'], special, water)
    except DATA_ERRORS as error:
        return dict(out, excluded=f'thermal loader: {error}')
    if skipped: out['mode_rows_skipped'] = True
    if EXCLUDE_NO_OIL_CROSS and not r['has_oil'] and any(r['slots'][hex(0x1a8 + 4 * j)] != 0. for j in range(4)):
        return dict(out, excluded='no oil channel with water cross coupling')
    work = [w for w, _ in r['loads']]; recover = [x for _, x in r['loads']]
    if any(x < 0. for x in work + recover):return dict(out, excluded='negative WorkTime or RecoverTime')
    if not overflow_damages(r, work):return dict(out, excluded='load counter overflow without damage')
    # Ch0 is water or cylinder head by props+0x23c (R/REPORT.md:202-206); ch1 (oil) exists only with HasOil: the
    # update processes properties[0x140]+1 channels (R/REPORT.md:80-83,290-300). Oil is always a liquid channel.
    liquid, labels = (rec['liquid0'], True), ('Water' if water else 'Head', 'Oil')
    channels = [channel(r, c, liquid[c], labels[c], work, recover, ratings) for c in ((0, 1) if r['has_oil'] else (0,))]
    return dict(out, channels=channels, gauge=[gauge(r, c, liquid[c], work, recover) for c in range(len(channels))],
                automatic=[] if special else radiator(r, rec, len(channels), work, recover),
                radiator=radiator_curves(r, rec, len(channels)))


def merged(engines):
    # Blocks that differ only in fields the limits don't read (DB-3A: CoolingEffectiveAirSpeed) give one result.
    out = {}
    for e in engines:
        key = json.dumps({k: v for k, v in e.items() if k != 'engines'}, sort_keys=True)
        if key in out: out[key]['engines'] = sorted(out[key]['engines'] + e['engines'])
        else: out[key] = e
    return list(out.values())


def radiator_cd(fm):
    """Aerodynamics.RadiatorCd and OilRadiatorCd: the drag coefficients the solver adds times each opening
    (wing_model.drag_terms)."""
    aero = fm.get('Aerodynamics') if isinstance(fm.get('Aerodynamics'), dict) else {}
    out = []
    for name in ('RadiatorCd', 'OilRadiatorCd'):
        try: out.append(f(first(aero.get(name, 0.))))
        except DATA_ERRORS: out.append(None)
    return out


def evaluate(fm, ratings_min=RATINGS):
    """Every engine of a raw FM tree (orjson.loads of its .blkx)."""
    ratings = tuple(float(m) for m in ratings_min)
    engines = [engine(rec, ratings) if rec['admitted'] else dict(engines=rec['indices'], family=rec['family'], excluded=rec['reason'], verified=None)
               for rec in select(fm)]
    return dict(engines=merged(engines), ratings_min=list(ratings), radiator_cd=radiator_cd(fm),
                assumptions=dict(ASSUMPTIONS, verified_against=thermal_validated.VERSION))


@functools.lru_cache(maxsize=64)
def _limits(path, stamp, ratings):
    return evaluate(orjson.loads(Path(path).read_bytes()), ratings)


def limits(aircraft_id, ratings_min=RATINGS):
    """The page's result for one catalog aircraft, from its raw FM; cached on the FM file and the ratings."""
    from aircraft_catalog import source
    path = source(aircraft_id); stat = path.stat()
    return dict(_limits(str(path), (stat.st_mtime_ns, stat.st_size), tuple(ratings_min)), aircraft_id=aircraft_id)


NUMBER = re.compile(r'(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?')


def parse_ratings(raw):
    """'30,10,5,1' -> (30., 10., 5., 1.): up to 8 distinct ratings in (0, 600] minutes, longest first; None: defaults;
    '' : none. Plain decimal numbers only (as JavaScript writes them)."""
    if raw is None: return RATINGS
    items = [x.strip() for x in raw.split(',')] if raw.strip() else []
    if not all(NUMBER.fullmatch(x) for x in items): raise ValueError('Invalid rating')
    values = set(map(float, items))
    if len(values) > 8: raise ValueError('At most 8 ratings')
    if not all(math.isfinite(v) and 0. < v <= 600. for v in values): raise ValueError('Ratings must be within 0-600 min')
    return tuple(sorted(values, reverse=True))
