"""Catalog audit and original-machine-code checks for fully upgraded RB/SB FMs."""
import json
import copy
from pathlib import Path
from unicorn import UC_HOOK_CODE
from unicorn.x86_const import UC_X86_REG_RSI

from aircraft_upgrades import (ROOT, definitions, profile, modify_engine,
    effect_scales, resolve_modifications)
from aircraft_catalog import catalog
from jet_catalog import engines
from prop_catalog import load, catalog as prop_catalog
from mass_parts_native import MassPartsNative
from verify_piston_loaded import load_engine_properties

OUT = ROOT / 'analysis/fully-upgraded'
OFFSETS = dict(base_hp=0xc, torque_base=0x168, afterburner_boost=0x1ec, boost_ata_ratio=0x1c4)


def native_effect_selection():
    """Execute the original local-effects/shared-effects selector itself."""
    native = MassPartsNative()
    captured = []
    def clear(u, address, size, data):
        native.ret()
    def copy_block(u, address, size, data):
        captured.append(native.blocks[u.reg_read(UC_X86_REG_RSI)])
        native.ret()
    native.u.hook_add(UC_HOOK_CODE, clear, begin=0x102e6a740, end=0x102e6a740)
    native.u.hook_add(UC_HOOK_CODE, copy_block, begin=0x102e6a020, end=0x102e6a020)
    shared = {'test': {'effects': {'addHorsePowers': 50.}}}
    context = native.alloc(64)
    native.qword(context + 0x20, native.block({'modifications': shared}))
    name = native.alloc(8); native.u.mem_write(name, b'test\0')
    reports = []
    for local in ({}, {'effects': {}}, {'effects': {'afterburnerMult': 1.75}}):
        vehicle = {'modifications': {'test': local}}
        count = len(captured)
        native.run(0x105163630, [context, native.block(vehicle), name, native.alloc(32)])
        assert len(captured) == count + 1
        expected = local.get('effects', shared['test']['effects'])
        assert captured[-1] == expected
        resolved = resolve_modifications(vehicle, shared)['applied']
        assert (resolved[0]['effects'] if resolved else {}) == expected
        reports.append(dict(local=local, native_selected=captured[-1]))
    return reports


def native_running():
    """Independent owner propagation, including a shared-type twin installation."""
    from aircraft_upgrades_native import apply as apply_native
    from prop_catalog import assets
    from prop_steady import initial_state
    from propulsion_general_native import PropulsionGeneralNative
    from propulsion_general import step
    from verify_propulsion_general import differences
    from component_assembly import f32
    reports = []
    for name in ('spitfire_ix_ussr', 'yak-3_france', 'db_3b'):
        native = PropulsionGeneralNative()
        fm_id = prop_catalog()[name]['fm_id']
        native.configure(json.loads((ROOT / 'references/jet-catalog/fm' / (fm_id + '.blkx')).read_text()))
        actual_properties = apply_native(native, name)
        expected_properties = assets(name)[0]
        assert json.loads(json.dumps(actual_properties)) == expected_properties, (name, 'installation properties')
        state = initial_state(expected_properties, [90., -15., 1.], 30.,
            automatic=[True] * len(expected_properties['propellers']), engine_control_mode='automatic')
        ns = copy.deepcopy(state)
        count = 0
        for i in range(24):
            kw = dict(velocity=[f32(70. + i), f32(-15.), f32(.5)], height=f32(30. + i * 180.),
                dt=f32(1 / 48), body_omega=[f32(.03), f32(.1), f32(-.2)],
                cg=[f32(-.16), f32(.019), 0.], nitro=0.)
            actual = native.step(ns, seed=ns['seed'], **kw)
            predicted = step(expected_properties, state, seed=state['seed'], **kw)
            diff = differences(actual, predicted)
            assert not diff, (name, i, diff)
            ns = dict(ns, transmissions=actual['transmissions'], seed=actual['seed'],
                engines=[dict(s, **r) for s, r in zip(ns['engines'], actual['engines'])],
                propellers=[dict(s, **r) for s, r in zip(ns['propellers'], actual['propellers'])])
            state = predicted
            count += 1
        reports.append(dict(aircraft=name, independently_propagated_steps=count,
            engine_count=len(expected_properties['engines'])))
        print('PASS running', name, count, flush=True)
    return reports


def main():
    OUT.mkdir(exist_ok=True)
    rows = catalog()
    audit = []
    for name, row in rows.items():
        audit.append(dict(aircraft=name, supported=row['supported'],
            reason=row.get('reason'), upgrades=row.get('upgrades')))
    (OUT / 'catalog.json').write_text(json.dumps(audit, indent=2) + '\n')
    shared, _ = definitions()
    assert not resolve_modifications({'modifications': {'hydravlic_power': {}}}, shared)['applied']
    assert not resolve_modifications({'modifications': {'150_octan_fuel': {
        'invertEnableLogic': True, 'effects': {'afterburnerMult': 0.4167}}}}, shared)['applied']
    assert profile('spitfire_ix_ussr')['applied'][0]['effects'] == {
        'afterburnerMult': 1.75, 'afterburnerCompressorMult': 2.14}
    selection = native_effect_selection()
    checked = {}
    for name, row in prop_catalog().items():
        upgrades = profile(name)
        if not upgrades['applied'] or not row['supported']:
            continue
        fm_id = row['fm_id']
        key = json.dumps([fm_id, upgrades['applied']], sort_keys=True)
        if key in checked:
            checked[key]['vehicles'].append(name)
            continue
        fm = load(name)
        prepared = json.loads((ROOT / 'references/prop-propulsion' / (fm_id + '.json')).read_text())['properties']
        reports = []
        for index, (_, engine) in enumerate(engines(fm)):
            native, address = load_engine_properties(engine)
            before = bytes(native.u.mem_read(address, 0x770))
            p = prepared['engines'][index]['properties']
            for modification in upgrades['applied']:
                effects = modification['effects']
                a, b = effect_scales()
                native.xmm(0, [a]); native.xmm(1, [b])
                native.run(0x1019f8c30, [address, native.block(effects)])
                p = modify_engine(p, effects)
            actual = {k: native.read(address + off, 1)[0] for k, off in OFFSETS.items()}
            expected = {k: p[k] for k in OFFSETS}
            assert actual == expected, (name, index, actual, expected)
            after = bytes(native.u.mem_read(address, 0x770))
            changed = [i for i in range(0, len(before), 4) if before[i:i+4] != after[i:i+4]]
            assert set(changed) <= set(OFFSETS.values()), (name, changed)
            reports.append(dict(engine=index, actual=actual, expected=expected,
                                changed_offsets=[hex(x) for x in changed]))
        checked[key] = dict(fm_id=fm_id, vehicles=[name], engines=reports)
        print('PASS', name, fm_id, flush=True)
    running = native_running()
    report = dict(status='PASS', policy='Fully upgraded RB/SB', effect_scales=effect_scales(),
        native_effect_selection=selection, supported_aircraft=sum(r['supported'] for r in rows.values()),
        changed_aircraft=sum(bool(r.get('upgrades', {}).get('applied')) and r['supported'] for r in rows.values()),
        missing_vehicle=[n for n, r in rows.items() if 'no pinned vehicle' in r.get('reason', '')],
        native_configurations=list(checked.values()), running=running, binary_sha256=native.sha)
    (OUT / 'native-validation.json').write_text(json.dumps(report, indent=2) + '\n')
    print('PASS', report['supported_aircraft'], 'supported;', report['changed_aircraft'],
          'with positive performance upgrades;', len(checked), 'native engine configurations', flush=True)


if __name__ == '__main__':
    main()
