"""Engine ratings page (Leo, 2026-10-30): each engine's temperature limits under the game's own thermal fatigue rules.

Per distinct engine and temperature channel (water or cylinder head, and oil), with the temperature held constant:
the hottest temperature sustainable indefinitely (no load counter can ever overflow, whatever its history) and, for
each rating of R minutes, the hottest one at which fresh counters give no damage within R minutes; the automatic
radiator's resting temperatures and how long they last; and the time to first damage against temperature for the
chart. All of it is the float32 arithmetic of the research ports (R = research/thermal-fatigue-2026-10-29):
thermal_properties.construct loads the engine's Temperature block, thermal_core.rates gives each load counter's rate
at a temperature and thermal_counters.ticks_from_zero the exact tick its counter first overflows. No result reads a
loader value that depends on expf/powf/logf (R/thermal_properties.py:68-71,122-133 use none), so the Python
stand-ins below do not change any of them. RB/SB (Engine_Overheat on), fresh counters, full health, standard
atmosphere. No modification carries a thermal block, so the limits do not depend on upgrades, throttle or mass.
"""
import functools
import json
import math
import re
import struct
from pathlib import Path

import orjson

import thermal_validated
from thermal_core import f, add, sub, mul, rates, damage
from thermal_counters import ticks_from_zero
from thermal_properties import K, construct

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
# update (R/ENGINE-THERMODYNAMICS.md:591-598; R/REPORT.md:290-300), which no result here uses: every result holds the
# temperature, and load counters read only their own channel (R/thermal_core.py:18-46). True excludes them anyway.
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


# Stand-ins for the game's CRT, used only by loader slots no result reads.
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
        fields = ('temperature', 'family', 'water', 'boost_controlled', 'wep', 'boost_delivered', 'radiator_control')
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
    boiling = [r['slots'][hex(0x1d0 + 4 * c)] if liquid else None for c, liquid in enumerate((rec['liquid0'], True))]
    out = []
    for x in sorted({row[0] for row in rows} | {one} | ({wep} if rec['wep'] else set())):
        if not reach(x): continue
        targets = [target(rows, x, c) if c in auto else None for c in range(channels)]; held = [None] * channels; boils = False
        for c in auto:
            held[c] = resting(targets[c])
            if boiling[c] is not None and held[c] >= boiling[c]: held[c] = prev32(boiling[c]); boils = True
        ticks = [n for n in (first_damage(tables[c], work, recover, held[c]) for c in auto) if n is not None]
        out.append(dict(coordinate=x, label='WEP' if x == wep and rec['wep'] else '%d%%' % round(x * 100),
                        targets_c=[celsius(t) for t in targets], held_c=[celsius(t) for t in held],
                        seconds=min(ticks) / 48. if ticks else None, boils=boils))
    return out


# ---- One aircraft -----------------------------------------------------------------------------------------------
def load(block, special, water):
    """construct() on the loader's view of the block; True when its Mode rows had to be left out. The port raises where
    native SSE arithmetic gives inf or nan (f() beyond the float32 range, a division by zero). Mode rows feed only the
    mode, heat and flow slots (R/thermal_properties.py:78-120); nothing used here reads them (lines 58-75 and 122-133),
    so a block whose Mode rows fail loads again without them, with every value used here unchanged."""
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
    return dict(out, channels=channels, automatic=[] if special else radiator(r, rec, len(channels), work, recover))


def merged(engines):
    # Blocks that differ only in fields the limits don't read (DB-3A: CoolingEffectiveAirSpeed) give one result.
    out = {}
    for e in engines:
        key = json.dumps({k: v for k, v in e.items() if k != 'engines'}, sort_keys=True)
        if key in out: out[key]['engines'] = sorted(out[key]['engines'] + e['engines'])
        else: out[key] = e
    return list(out.values())


def evaluate(fm, ratings_min=RATINGS):
    """Every engine of a raw FM tree (orjson.loads of its .blkx)."""
    ratings = tuple(float(m) for m in ratings_min)
    engines = [engine(rec, ratings) if rec['admitted'] else dict(engines=rec['indices'], family=rec['family'], excluded=rec['reason'], verified=None)
               for rec in select(fm)]
    return dict(engines=merged(engines), ratings_min=list(ratings), assumptions=dict(ASSUMPTIONS, verified_against=thermal_validated.VERSION))


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
