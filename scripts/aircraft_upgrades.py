"""Fully researched RB/SB aircraft: source-selected, native-checked upgrades.

Most standard upgrades remove an inverted stock penalty; the base FM already
represents their researched state. Positive engine upgrades are separate.
Native selectors: 1051614a0 (local/shared invert flag), 105163630 (whole effects
block fallback). Engine modifier: 1019f8c30. Do not recursively merge effects.
"""
import copy
import hashlib
import json
from functools import lru_cache
from pathlib import Path

from component_assembly import f32, add, sub, mul

ROOT = Path(__file__).resolve().parents[1]
VEHICLES = ROOT / 'references/prop-vehicles'
DEFINITIONS = ROOT / 'references/aircraft-modifications/modifications.blkx'
ENGINE_EFFECTS = frozenset(('afterburnerMult', 'afterburnerCompressorMult', 'addHorsePowers'))
# These effects do not change the existing intact, clean EM equations.
# Researching weapons does not install every mutually exclusive weapon/belt.
OUTSIDE_EM = frozenset(('bulletMod', 'weaponMod', 'additiveBulletMod', 'commonWeapons',
    'extinguisherNum', 'gForceTolerationMult', 'sensors', 'counterMeasures',
    'nightVision', 'enginesInfraRedBrightnessMult', 'effects'))
POLICY = 'Fully upgraded aircraft; RB/SB; all flight-performance upgrades installed'


@lru_cache(maxsize=1)
def definitions():
    manifest = json.loads(DEFINITIONS.with_name('manifest.json').read_text())
    raw = DEFINITIONS.read_bytes()
    if hashlib.sha256(raw).hexdigest() != manifest['sha256']:
        raise ValueError('Aircraft modification definitions do not match pinned source')
    vehicles = json.loads((VEHICLES / 'manifest.json').read_text())
    if vehicles['commit'] != manifest['commit']:
        raise ValueError('Vehicle and modification source versions differ')
    return json.loads(raw)['modifications'], vehicles['records']


def resolve_modifications(vehicle, shared):
    """Return researched effects in source order; an explicit empty block wins."""
    active, included, ignored = [], [], []
    for name, local in vehicle.get('modifications', {}).items():
        base = shared.get(name, {})
        inverted = local.get('invertEnableLogic', base.get('invertEnableLogic', False))
        if inverted:
            included.append(name)
            continue
        effects = local['effects'] if 'effects' in local else base.get('effects', {})
        unknown = set(effects) - ENGINE_EFFECTS - OUTSIDE_EM
        if unknown:
            raise ValueError('Unimplemented researched FM effect: ' + ', '.join(sorted(unknown)))
        physics = {key: value for key, value in effects.items() if key in ENGINE_EFFECTS}
        if physics:
            active.append(dict(name=name, effects=physics))
        if set(effects) & OUTSIDE_EM:
            ignored.append(dict(name=name, effects=sorted(set(effects) & OUTSIDE_EM)))
    return dict(applied=active, already_in_base_fm=included, outside_em=ignored)


@lru_cache(maxsize=2048)
def profile(name):
    shared, rows = definitions()
    by_name = {row['vehicle']: row for row in rows}
    candidates = [name] if name in by_name else sorted(row['vehicle'] for row in rows
        if Path(row.get('fm') or '').stem == name)
    if not candidates:
        raise ValueError('Fully upgraded configuration cannot be verified: no pinned vehicle maps to this FM')
    resolved = [resolve_modifications(json.loads((VEHICLES / (n + '.blkx')).read_text()), shared)
                for n in candidates]
    # FM-only jet entries can represent several variants. Only share them when
    # every mapped vehicle has the same effects on the supported EM equations.
    signatures = {json.dumps(r['applied'], sort_keys=True) for r in resolved}
    if len(signatures) != 1:
        raise ValueError('Mapped vehicle variants have different researched FM upgrades')
    return dict(policy=POLICY, fully_upgraded=True, vehicle_sources=candidates,
                **resolved[0])


@lru_cache(maxsize=1)
def effect_scales():
    params = json.loads((ROOT / 'references/body-gameparams.blkx').read_text())
    def find(value):
        if isinstance(value, dict):
            if isinstance(value.get('noArcadeBoost'), dict) and 'on' in value['noArcadeBoost']:
                return value['noArcadeBoost']['on']
            for child in value.values():
                match = find(child)
                if match is not None:
                    return match
        return None
    mode = find(params)
    if mode is None:
        raise ValueError('Missing pinned RB/SB modification scales')
    return f32(mode['modificationsEffect']), f32(mode['wholeModificationsEffect'])


def modify_engine(properties, effects, scales=None):
    """1019f8c30 operations on already-loaded piston properties, float32 order."""
    p = copy.deepcopy(properties)
    modification, whole = scales or effect_scales()
    power_scale = mul(modification, whole)
    for key, value in effects.items():
        if key == 'afterburnerMult':
            p['afterburner_boost'] = add(mul(sub(p['afterburner_boost'], 1.), f32(value)), 1.)
        elif key == 'afterburnerCompressorMult':
            p['boost_ata_ratio'] = add(mul(sub(p['boost_ata_ratio'], 1.), f32(value)), 1.)
        elif key == 'addHorsePowers':
            p['base_hp'] = add(p['base_hp'], mul(f32(value), power_scale))
        else:
            raise ValueError('Unimplemented engine upgrade: ' + key)
    # The original modifier finishes by refreshing torque from normalized hp.
    p['torque_base'] = f32(mul(p['base_hp'], 746.) / p['max_omega'])
    return p


def apply_propulsion(name, properties):
    upgrades = profile(name)
    if not upgrades['applied']:
        return properties
    result = copy.deepcopy(properties)
    for engine in result['engines']:
        if engine['family'] not in (0, 1):
            raise ValueError('Positive engine upgrade requires a validated piston consumer')
        for modification in upgrades['applied']:
            engine['properties'] = modify_engine(engine['properties'], modification['effects'])
        engine['torque_base'] = engine['properties']['torque_base']
    # 101a14130 visits propeller types too. Its modifier unconditionally
    # refreshes inertia in a different float32 multiplication order from the
    # initial loader (101a059eb..5a24), even for an engine-only upgrade.
    for propeller in result['propellers']:
        p = propeller['properties']
        inertia = mul(p['inertia_coefficient'], mul(mul(mul(p['diameter'],
            p['diameter']), f32(1. / 12.)), p['mass']))
        p['inertia'] = inertia
        propeller['inertia'] = inertia
    return result


def source_paths():
    return [DEFINITIONS, DEFINITIONS.with_name('manifest.json'),
        ROOT / 'references/body-gameparams.blkx', VEHICLES / 'manifest.json',
        *sorted(VEHICLES.glob('*.blkx'))]
