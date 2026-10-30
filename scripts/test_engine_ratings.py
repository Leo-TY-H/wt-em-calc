"""Engine ratings: ported files, exact counters, the game's engine selection, Bf 109 F-4 values, monotonicity, API.

Fleet checks read references/jet-catalog/fm when it is there and are skipped otherwise.
"""
import copy
import hashlib
import json
import math
import random
import sys
import time
import unittest
from pathlib import Path
from types import ModuleType
from unittest.mock import patch

import numpy as np
import orjson

import engine_ratings as er
import thermal_validated as tv
from thermal_core import f, mul, rates
from thermal_counters import evolve, ticks_from_zero
from thermal_properties import safe

HERE = Path(__file__).resolve().parent
FM = HERE.parent / 'references/jet-catalog/fm'
F4 = FM / 'bf-109f-4.blkx'
DT = f(1 / 48)


def fm(name):
    return orjson.loads((FM / f'{name}.blkx').read_bytes())


def brute(rates_, horizon):
    """First overflow tick of counters from 0 by plain float32 stepping (numpy), -1 beyond the horizon."""
    r = np.array(rates_, dtype=np.float32); d = (r * np.float32(DT)).astype(np.float32)
    c = np.zeros_like(d); first = np.full(len(r), -1, dtype=np.int64)
    for n in range(1, horizon + 1):
        v = (c + d).astype(np.float32)
        first[(v > 1) & (first < 0) & (r > 0)] = n
        c = np.minimum(np.float32(1), np.maximum(np.float32(0), v))
    return first


def engine(**main):
    temperature = dict(Load0=dict(WaterTemperature=70., OilTemperature=60.), Load1=dict(WaterTemperature=90., OilTemperature=80., WorkTime=600., RecoverTime=200.))
    return dict(Main=dict(Type='Inline', IsWaterCooled=True, **main), Temperature=temperature)


class Ports(unittest.TestCase):
    def test_ported_files_are_the_research_originals(self):
        # research/thermal-fatigue-2026-10-29: thermal_core.py and thermal_properties.py, byte for byte (LF).
        for name, digest in (('thermal_core.py', '5882fc58cf9390d81d34bb7d9522c2bdf4e33389fb14eae7c7f6133d6c54ab68'),
                             ('thermal_properties.py', '273072edeb0ee5b20193baf0737aa4ec34c50cc7d4db9fe8b7c03ede4deaff83')):
            with self.subTest(name=name):
                self.assertEqual(hashlib.sha256((HERE / name).read_bytes().replace(b'\r\n', b'\n')).hexdigest(), digest)

    def test_validated_set(self):
        self.assertEqual(len(tv.SIGNATURES), 761)
        self.assertEqual(hashlib.sha256('\n'.join(sorted(tv.SIGNATURES)).encode()).hexdigest(),
                         '16b0196413dfa731c5f56b4a07103fe66766331536dd64058024f137f8346297')


class Counters(unittest.TestCase):
    def test_constants(self):
        # A counter at 1 keeps 2.86102294921875e-6 (f(r*dt) = 2^-24 rounds away) and overflows at the next float32
        # (R/ENGINE-THERMODYNAMICS.md:544-550).
        self.assertEqual(er.RHO_FULL, er.next32(2.86102294921875e-6))
        self.assertEqual([er.rho_star(m * 2880.) for m in (30, 10, 5, 1)],
                         [5.557537660934031e-4, 1.6665459843352437e-3, 3.333807224407792e-3, 1.6666889190673828e-2])
        self.assertEqual(er.opening_step(), 0.1882352977991104)
        # Band top: 0.941 K from the opening step plus the largest derivative a resting temperature can carry.
        t = f(363.65); top = er.resting(t)
        self.assertEqual((top - t, er.residual(top)), (0.9521484375, 0.0007324218167923391))
        self.assertEqual(er.add(er.mul(-er.residual(top), DT), top), top)
        self.assertNotEqual(er.add(er.mul(-er.next32(er.residual(top)), DT), top), top)
        rng = random.Random(5)
        for x in [f(rng.uniform(250., 1300.)) for _ in range(300)] + [256., 512., 1024., er.next32(512.), er.prev32(512.)]:
            direct = er.largest_true(lambda d: er.add(er.mul(-d, DT), x) == x, f(1e-30), f(1.))
            self.assertEqual(er.residual(x), direct, x)
        self.assertEqual(er.rho_star(.5), math.inf)               # below one tick nothing overflows
        self.assertEqual(er.rho_star.cache_info().maxsize, 256)

    def test_worktime_ticks(self):
        # Float32 accumulation is not 48*W ticks.
        got = {w: ticks_from_zero(safe(1., f(w)), DT) for w in (60, 180, 300, 600, 1800, 3600)}
        self.assertEqual(got, {60: 2881, 180: 8640, 300: 14401, 600: 28790, 1800: 86458, 3600: 172939})

    def test_ticks_from_zero_matches_evolve_and_stepping(self):
        rng = random.Random(7)
        sample = [f(10 ** rng.uniform(-6.5, -0.5)) for _ in range(600)] + [f(rng.uniform(1e-4, 3e-2)) for _ in range(200)]
        sample += [f(2.86102294921875e-6), er.RHO_FULL, f(1e-9), 0., -1e-3]
        for e in range(-24, -6):                      # increments exactly on ties at some binade
            for q in range(1, 4): sample += [f((q + .5) * 2. ** (e - 23) / DT)]
        exact = [ticks_from_zero(r, DT) for r in sample]
        for r, n in zip(sample, exact):
            ticks, kind, _ = evolve(0., r, DT)
            self.assertEqual(n, ticks if kind == 'overflow' else None, r)
        horizon = 30000
        for r, n, m in zip(sample, exact, brute(sample, horizon)):
            if m >= 0: self.assertEqual(n, m, r)
            else: self.assertTrue(n is None or n > horizon, r)

    def test_ticks_do_not_increase_with_rate(self):
        rng = random.Random(3)
        sample = sorted({f(10 ** rng.uniform(-6, 0)) for _ in range(3000)})
        ticks = [ticks_from_zero(r, DT) for r in sample]
        ticks = [math.inf if n is None else n for n in ticks]
        self.assertTrue(all(a >= b for a, b in zip(ticks, ticks[1:])))


class Selection(unittest.TestCase):
    def test_type_index_selects_the_shared_type(self):
        a = engine(); b = engine(); b['Temperature']['Load1']['WaterTemperature'] = 95.
        tree = dict(EngineType0=a, Engine0=dict(Type=0, Temperature=b['Temperature']), Engine1=dict(Type=0))
        (rec,) = er.select(tree)
        self.assertEqual((rec['indices'], rec['source'], rec['temperature']), ([0, 1], 'EngineType0', a['Temperature']))

    def test_invalid_type_loads_the_instance_and_gaps_stop(self):
        tree = dict(EngineType0=engine(), Engine0=dict(engine(), Type=1), Engine2=engine())
        (rec,) = er.select(tree)
        self.assertEqual((rec['indices'], rec['source']), ([0], 'Engine0'))
        tree = dict(EngineType0=dict(Main=dict(Type='Inline')), Engine0=dict(engine(), Type=0))
        self.assertEqual(er.select(tree)[0]['reason'], 'no Temperature block')

    def test_type_capacity(self):
        tree = {f'EngineType{i}': engine() for i in range(10)}
        tree.update({f'Engine{i}': engine(ThrottleBoost=1. + i / 100) for i in range(7)})
        recs = er.select(tree)
        self.assertEqual([r['reason'] for r in recs if not r['admitted']], ['engine type capacity exceeded'])
        self.assertEqual(recs[-1]['indices'], [6])

    def test_families_and_flags(self):
        tree = dict(Engine0=engine(), Engine1=dict(Main=dict(Type='Rocket'), Temperature={}), Engine2=dict(Main=dict(Type='Rocket')),
                    Engine3=dict(Temperature={}), Engine4=engine())
        recs = er.select(tree)
        self.assertEqual([(r['indices'], r.get('reason')) for r in recs],
                         [([0, 4], None), ([1, 2], 'Rocket: no thermal model'), ([3], 'no engine family')])
        tree = dict(Engine0=dict(engine(), Afterburner=dict(Type=6, IsControllable=True)), Engine1=engine(ThrottleBoost=1.1))
        self.assertEqual([(r['boost_controlled'], r['wep']) for r in er.select(tree)], [(True, False), (False, True)])

    def test_boost_delivery(self):
        # Kinds 1/2/4/5/9 need the resource; every stage product AfterburnerBoostMul_i x AfterburnerBoost <= 0.99 rejects.
        def delivered(kind, nitro, stages, boost=1.2):
            block = dict(engine(ThrottleBoost=1.1, AfterburnerBoost=boost), Afterburner=dict(Type=kind, IsControllable=True),
                         Compressor=dict(NumSteps=len(stages), **{f'AfterburnerBoostMul{i}': m for i, m in enumerate(stages)}))
            return er.select(dict(Mass=dict(MaxNitro=nitro), Engine0=block))[0]['boost_delivered']
        self.assertEqual([delivered(4, 0., [1.]), delivered(4, 33., [1.]), delivered(6, 0., [1.]), delivered(1, .0009, [1.])],
                         [False, True, True, False])
        self.assertEqual([delivered(6, 0., [1., 0.]), delivered(6, 0., [0., 0.]), delivered(6, 0., [.825], 1.2), delivered(6, 0., [])],
                         [True, False, False, True])

    def test_malformed_engine_is_excluded(self):
        for main, boost in ((dict(ThrottleBoost='x'), {}), (dict(ThrottleBoost=1e39), {}), ({}, dict(Type='x', IsControllable=True))):
            tree = dict(Engine0=dict(engine(**main), Afterburner=boost), Engine1=dict(engine(), Main=dict(Type='Radial')))
            with self.subTest(main=main, boost=boost):
                first, second = er.evaluate(tree)['engines']
                self.assertTrue(first['excluded'].startswith('invalid engine value'))
                self.assertIn('channels', second)
        for key, value in (('DegradationTime_10_5', [1.]), ('WaterThermalConductionCrossCoeffTable', 'x')):
            tree = dict(Engine0=engine()); tree['Engine0']['Temperature'][key] = value
            with self.subTest(key=key):
                self.assertTrue(er.evaluate(tree)['engines'][0]['excluded'].startswith('thermal loader'))
        tree = dict(Engine0=engine()); tree['Engine0']['Temperature']['Load1']['WaterTemperature'] = 'hot'
        self.assertTrue(er.evaluate(tree)['engines'][0]['excluded'].startswith('thermal loader'))

    def test_loader_input(self):
        block = dict(WaterAirDensityPower=[-.015, 0.], RadiatorAuto=[True, True], DegradationTime_10_5=[180., 171.])
        self.assertEqual(er.loader_block(block), dict(WaterAirDensityPower=-.015, RadiatorAuto=True, DegradationTime_10_5=[180., 171.]))
        result = er.evaluate(dict(Engine0=dict(engine(), Temperature=dict(engine()['Temperature'], NoFlowMode=[0, 1]))))
        self.assertEqual(result['engines'][0]['excluded'], 'thermal loader: unverified repeated parameter NoFlowMode')

    def test_no_oil_engine_has_one_channel(self):
        tree = dict(Engine0=engine()); tree['Engine0']['Temperature']['HasOil'] = False
        (e,) = er.evaluate(tree)['engines']
        self.assertEqual([c['label'] for c in e['channels']], ['Water'])
        self.assertFalse(e['verified'])

    def test_equal_results_merge(self):
        tree = dict(Engine0=engine(), Engine1=engine(), Engine2=engine())
        tree['Engine1']['Temperature']['CoolingEffectiveAirSpeed'] = 290.
        tree['Engine2']['Temperature']['HasOil'] = False
        self.assertEqual(len(er.select(tree)), 3)
        self.assertEqual([e['engines'] for e in er.evaluate(tree)['engines']], [[0, 1], [2]])

    def test_no_oil_water_cross_switch(self):
        # Admitted (no result reads the cross coupling); EXCLUDE_NO_OIL_CROSS restores the exclusion.
        tree = dict(Engine0=engine()); tree['Engine0']['Temperature'].update(HasOil=False, WaterThermalConductionCrossCoeffTable=[.004, .002, .0002, .002])
        self.assertIn('channels', er.evaluate(tree)['engines'][0])
        with patch.object(er, 'EXCLUDE_NO_OIL_CROSS', True):
            self.assertEqual(er.evaluate(tree)['engines'][0]['excluded'], 'no oil channel with water cross coupling')
            jet = dict(Engine0=dict(Main=dict(Type='Jet'), Temperature=dict(engine()['Temperature'], HasOil=False)))
            self.assertIn('channels', er.evaluate(jet)['engines'][0])      # special engines keep zero cross slots

    def test_radiator_availability(self):
        rows = dict(RadiatorMode0=dict(Throttle=.8, WaterTemperature=80., OilTemperature=70.))
        def automatic(**controls):
            tree = dict(Engine0=dict(engine(), Controls=controls)); tree['Engine0']['Temperature'].update(rows)
            return [(a['label'], a['targets_c'], a['seconds'] is None) for a in er.evaluate(tree)['engines'][0]['automatic']]
        both = automatic(hasRadiatorControl=True, hasOilRadiatorControl=True)
        self.assertEqual([t for _, t, _ in both], [[80., 70.], [80., 70.]])
        self.assertEqual(automatic(hasRadiatorControl=True, hasCommonRadiator=True), both)
        self.assertEqual([t for _, t, _ in automatic(hasRadiatorControl=True)], [[80., None], [80., None]])
        self.assertEqual([t for _, t, _ in automatic(hasOilRadiatorControl=True)], [[None, 70.], [None, 70.]])
        self.assertEqual(automatic(hasRadiatorControl=False, hasOilRadiatorControl=False, hasCommonRadiator=True), [])

    def test_no_load_rows(self):
        tree = dict(Engine0=dict(engine(), Controls=dict(hasRadiatorControl=True, hasOilRadiatorControl=True)))
        tree['Engine0']['Temperature'] = dict(RadiatorMode0=dict(Throttle=.8, WaterTemperature=80., OilTemperature=70.))
        (e,) = er.evaluate(tree)['engines']
        self.assertEqual([c['continuous']['bound'] for c in e['channels']], ['none', 'none'])
        self.assertEqual([a['seconds'] for a in e['automatic']], [None, None])


@unittest.skipUnless(F4.is_file(), 'references/jet-catalog/fm is not here')
class BfF4(unittest.TestCase):
    def test_limits(self):
        (e,) = er.evaluate(fm('bf-109f-4'))['engines']
        self.assertEqual((e['engines'], e['family'], e['water_cooled'], e['verified']), ([0], 'Inline', True, True))
        water, oil = e['channels']
        self.assertEqual((water['label'], water['thermostat_c'], water['boiling_c']), ('Water', 60., 166.5))
        self.assertEqual((oil['label'], oil['thermostat_c'], oil['boiling_c']), ('Oil', 50., 360.0000305175781))
        self.assertEqual(water['continuous'], dict(max_c=86.70098876953125, bound='load'))
        self.assertEqual(oil['continuous'], dict(max_c=73.4019775390625, bound='load'))
        self.assertEqual([(r['minutes'], r['max_c'], r['bound']) for r in water['ratings']],
                         [(30., 97.08111572265625, 'load'), (10., 97.47836303710938, 'load'), (5., 103.8260498046875, 'load'), (1., None, 'none')])
        self.assertEqual([r['max_c'] for r in oil['ratings']], [87.08111572265625, 87.47836303710938, 93.8260498046875, None])
        self.assertEqual((water['floor_s'], oil['floor_s']), (2881 / 48, 2881 / 48))
        # Every limit is a float32 temperature and the next one up damages within the rating.
        rows = er.construct(er.loader_block(fm('bf-109f-4')['Engine0']['Temperature']), False, True, er.powf, er.expf, er.logf)
        work = [w for w, _ in rows['loads']]; recover = [r for _, r in rows['loads']]
        for r in water['ratings'][:3]:
            t = r['max_c'] + er.K
            self.assertEqual(f(t), t)
            self.assertGreater(er.first_damage(rows['water_rows'], work, recover, t), r['minutes'] * 2880)
            self.assertLessEqual(er.first_damage(rows['water_rows'], work, recover, er.next32(t)), r['minutes'] * 2880)

    def test_recovery(self):
        (e,) = er.evaluate(fm('bf-109f-4'))['engines']
        self.assertEqual([(b['fill_s'], b['empty_s']) for b in e['buckets']],
                         [(172939 / 48, 86458 / 48), (86458 / 48, 43218 / 48), (15362 / 48, 9120 / 48), (8640 / 48, 6240 / 48), (2881 / 48, 1440 / 48)])
        self.assertEqual([[round(t['full_c'], 4) for t in b['channels']] for b in e['buckets']],
                         [[80., 60.], [90., 80.], [95., 85.], [98., 88.], [105., 95.]])
        self.assertEqual([[round(t['drains_c'], 2) for t in b['channels']] for b in e['buckets']],
                         [[86.67, 73.33], [93.33, 83.33], [96.88, 86.88], [102.06, 92.06], [111.67, 101.67]])
        # Each is the last float32 temperature with the property: the next one up loses it.
        rows = er.construct(er.loader_block(fm('bf-109f-4')['Engine0']['Temperature']), False, True, er.powf, er.expf, er.logf)
        work = [w for w, _ in rows['loads']]; recover = [r for _, r in rows['loads']]
        for j, b in zip(range(1, 6), e['buckets']):
            for table, t in zip((rows['water_rows'], rows['oil_rows']), b['channels']):
                rate = lambda x: er.rates(table, work, recover, x)[j]
                drains, full = t['drains_c'] + er.K, t['full_c'] + er.K
                self.assertTrue(rate(drains) < 0. <= rate(er.next32(drains)))
                self.assertTrue(rate(full) == f(-recover[j]) < rate(er.next32(full)))

    def test_automatic_radiator(self):
        (e,) = er.evaluate(fm('bf-109f-4'))['engines']
        self.assertEqual([(a['label'], a['targets_c'], a['seconds'] * 48, a['boils']) for a in e['automatic']],
                         [('89%', [90.5, 80.5], 172939, False), ('100%', [95.5, 85.5], 86458, False), ('WEP', [105.5, 95.5], 8640, False)])
        self.assertEqual(e['automatic'][0]['held_c'], [91.4521484375, 81.4521484375])

    def test_mode_rows_fallback(self):
        # Mode rows the port cannot load (native gives inf) feed no result: the block loads without them, same limits.
        tree = fm('f-80a'); (base,) = er.evaluate(tree)['engines']
        for value in tree.values():
            if isinstance(value, dict) and isinstance(value.get('Temperature'), dict): value['Temperature']['OilThermostatSetPoint'] = 500.
        (e,) = er.evaluate(tree)['engines']
        self.assertEqual((e['verified'], e.get('mode_rows_skipped'), e['channels'][0]), (False, True, base['channels'][0]))
        self.assertNotIn('mode_rows_skipped', base)

    def test_research_signature(self):
        block = fm('bf-109f-4')['Engine0']['Temperature']
        self.assertEqual(hashlib.sha256(json.dumps(tv._tree(block), sort_keys=True, separators=(',', ':')).encode()).hexdigest(),
                         'be13351a73366cce9467d23653aadc4d20170f7832f33234c3c40599f6e232c7')
        self.assertEqual(tv.signature(block, False, True), '70711a0d46a71afba3cfee791b0505edb07cb8bb6a21fa6866ac92b65ae832e7')
        self.assertTrue(tv.verified(block, False, True))
        self.assertFalse(tv.verified(block, False, False)); self.assertFalse(tv.verified(block, True, True))
        changed = copy.deepcopy(block); changed['Load1']['WaterTemperature'] = er.next32(f(90.))
        self.assertFalse(tv.verified(changed, False, True))
        changed['Load1']['WaterTemperature'] = 90. + 1e-12
        self.assertTrue(tv.verified(changed, False, True))

    def test_cached_by_file(self):
        stub = ModuleType('aircraft_catalog'); stub.source = lambda name: F4
        with patch.dict(sys.modules, aircraft_catalog=stub):
            a = er.limits('bf-109f-4'); b = er.limits('bf-109f-4')
            self.assertEqual(a['aircraft_id'], 'bf-109f-4')
            self.assertIs(a['engines'], b['engines'])
            self.assertEqual(er.limits('bf-109f-4', (10.,))['ratings_min'], [10.])

    def test_parse_ratings(self):
        self.assertEqual(er.parse_ratings(None), (30., 10., 5., 1.))
        self.assertEqual(er.parse_ratings('5,30,5,2.5'), (30., 5., 2.5))
        self.assertEqual(er.parse_ratings(''), ())
        self.assertEqual(er.parse_ratings(','.join(['1'] * 9)), (1.,))
        self.assertEqual(er.parse_ratings('1e-7, .5'), (.5, 1e-7))
        for raw in ('0', '600.5', 'nan', 'inf', '-1', 'x', '1,,2', '1_0', '+1', '0x10', ','.join(str(i) for i in range(1, 10))):
            with self.subTest(raw=raw), self.assertRaises(ValueError): er.parse_ratings(raw)
        # Shorter than one update: nothing can damage within it.
        (e,) = er.evaluate(fm('bf-109f-4'), (1e-7,))['engines']
        self.assertEqual([c['ratings'][0] for c in e['channels']], [dict(minutes=1e-7, max_c=None, bound='none')] * 2)

    def test_pandora_wep_needs_nitro(self):
        # Afterburner Type 4 with Mass.MaxNitro 0: the boost never engages, 110% stays on the 100% target.
        (e,) = er.evaluate(fm('pandora_mk_1'))['engines']
        self.assertEqual([a['label'] for a in e['automatic']], ['91%', '100%'])


def admitted_blocks():
    """(name, record, construct result) of every distinct admitted engine of the catalog FMs."""
    for path in sorted(FM.glob('*.blkx')):
        for rec in er.select(orjson.loads(path.read_bytes())):
            if not rec['admitted']: continue
            try: r = er.construct(er.loader_block(rec['temperature']), rec['special'], rec['water'], er.powf, er.expf, er.logf)
            except (ValueError, OverflowError, ZeroDivisionError, TypeError): continue
            yield path.stem, rec, r


@unittest.skipUnless(FM.is_dir(), 'references/jet-catalog/fm is not here')
class Fleet(unittest.TestCase):
    def test_every_fm_evaluates(self):
        counts = dict(fms=0, engines=0, verified=0, excluded=0, port_only=[]); times = []
        for path in sorted(FM.glob('*.blkx')):
            tree = orjson.loads(path.read_bytes())
            start = time.perf_counter(); result = er.evaluate(tree); times.append(time.perf_counter() - start)
            orjson.dumps(result)
            counts['fms'] += 1
            for e in result['engines']:
                if 'excluded' in e: counts['excluded'] += 1; continue
                counts['engines'] += 1; counts['verified'] += e['verified']
                if not e['verified']: counts['port_only'].append(path.stem)
                for c in e['channels']:
                    with self.subTest(fm=path.stem, channel=c['label']):
                        limits = [c['continuous']] + c['ratings']
                        values = [x['max_c'] for x in limits if x['bound'] == 'load']
                        self.assertEqual(values, sorted(values))                       # longer ratings never hotter
                        seconds = [s for _, s in c['curve']]
                        self.assertTrue(all(a >= b for a, b in zip(seconds, seconds[1:])))
                        self.assertTrue(all(a[0] < b[0] for a, b in zip(c['curve'], c['curve'][1:])))
                for b in e['buckets']:
                    with self.subTest(fm=path.stem, bucket=b):
                        self.assertTrue(all(t['drains_c'] >= t['full_c'] for t in b['channels'] if t['full_c'] is not None))
        times.sort()
        print(f"\n{counts['fms']} FMs, {counts['engines']} engines ({counts['verified']} verified, {counts['excluded']} excluded); "
              f"median {times[len(times) // 2] * 1e3:.0f} ms, max {times[-1] * 1e3:.0f} ms; port only: {', '.join(sorted(set(counts['port_only']))) or 'none'}",
              file=sys.stderr)

    def test_rates_rise_with_temperature(self):
        """The premise of every bisection: each counter's rate does not fall as the temperature rises (dense scan,
        0.05 K over the rows +-2 K and 16 float32 steps around each row)."""
        for name, rec, r in admitted_blocks():
            work = [w for w, _ in r['loads']]; recover = [x for _, x in r['loads']]
            for c, rows in enumerate((r['water_rows'], r['oil_rows'])[:1 + r['has_oil']]):
                if len(rows) < 2: continue
                ts = {f(rows[0][0] - 2. + .05 * i) for i in range(int((rows[-1][0] - rows[0][0] + 4.) / .05) + 1)}
                for row in rows: ts.update(er.unbits(er.bits(row[0]) + d) for d in range(-16, 17))
                previous = None
                for t in sorted(ts):
                    now = rates(rows, work, recover, t)[:len(work)]
                    if previous is not None and any(a < b for a, b in zip(now, previous)):
                        self.fail(f'{name} channel {c}: a rate falls at {t} K')
                    previous = now

    def test_limits_equal_the_tick_bisection(self):
        """The fast search (largest rate below rho*) equals bisection on the first-damage tick itself."""
        for name, rec, r in admitted_blocks():
            if name not in ('bf-109f-4', 'b_24d', 'mig-9', 'a2d', 'la-7', 'zeppelin', 'uav_quadcopter', 'nt_tu_95m', 'f-80a'): continue
            work = [w for w, _ in r['loads']]; recover = [x for _, x in r['loads']]
            liquid = (rec['liquid0'], True)
            result = er.engine(rec, (30., 10., 5., 1.))
            for c, ch in enumerate(result['channels']):
                rows = (r['water_rows'], r['oil_rows'])[c]
                top = min(rows[-1][0], er.prev32(r['slots'][hex(0x1d0 + 4 * c)])) if liquid[c] else rows[-1][0]
                for limit in ch['ratings']:
                    ticks = limit['minutes'] * 2880
                    ok = lambda t: all(n is None or n > ticks for n in (ticks_from_zero(x, DT) for x in rates(rows, work, recover, t)[:len(work)]))
                    expected = None if ok(top) else er.largest_true(ok, rows[0][0], top)
                    with self.subTest(fm=name, channel=ch['label'], minutes=limit['minutes']):
                        self.assertEqual(None if limit['max_c'] is None else limit['max_c'] + er.K, expected)


class Page(unittest.TestCase):
    def test_page_files_are_served(self):
        # em_server.py serves only the app files its name table lists; index.html loads these two.
        server = (HERE / 'em_server.py').read_text(encoding='utf-8'); index = (HERE.parent / 'app/index.html').read_text(encoding='utf-8')
        for name in ('ratings.js', 'ratings.css'):
            with self.subTest(name=name):
                self.assertIn(f"'/{name}':'{name}'", server)
                self.assertIn(f'./{name}', index)


@unittest.skipUnless(F4.is_file(), 'references/jet-catalog/fm is not here')
class Api(unittest.TestCase):
    def test_route(self):
        try:
            import em_server
            from em_solver import AIRCRAFT
        except Exception as error:       # the full app (solver, catalog) is needed to serve
            self.skipTest(f'server not importable: {error}')
        from http.server import ThreadingHTTPServer
        from urllib.error import HTTPError
        from urllib.request import urlopen
        import threading
        server = ThreadingHTTPServer(('127.0.0.1', 0), em_server.Handler)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        base = f'http://127.0.0.1:{server.server_port}/api/engine-ratings/'
        try:
            name = 'bf-109f-4' if 'bf-109f-4' in AIRCRAFT else next(iter(AIRCRAFT))
            for page in ('ratings.js', 'ratings.css'):
                with self.subTest(page=page): self.assertEqual(urlopen(base.removesuffix('api/engine-ratings/') + page).status, 200)
            body = json.loads(urlopen(base + name + '?ratings=30,10').read())
            self.assertEqual((body['aircraft_id'], body['ratings_min']), (name, [30., 10.]))
            self.assertIn('game_data', body)
            for path, status in (('no_such_aircraft', 404), (name + '?ratings=0', 400), (name + '?ratings=1&ratings=2', 400),
                                 (name + '?ratings=' + ','.join(str(i) for i in range(1, 10)), 400), (name + '?ratings=nan', 400)):
                with self.subTest(path=path), self.assertRaises(HTTPError) as caught: urlopen(base + path)
                self.assertEqual(caught.exception.code, status)
        finally:
            server.shutdown(); server.server_close()


if __name__ == '__main__': unittest.main(verbosity=2)
