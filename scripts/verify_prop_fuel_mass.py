"""Native allocation -> frozen running fuel update -> mass, across prop assets."""
import json
import struct
from pathlib import Path
from collision_native import CollisionNative
from component_assembly import f32, mul, sub
from fuel_loading import initial
from prop_catalog import catalog, assets, load, mass_state
from mass_model import aircraft_properties
from verify_mass_model import MassMachine

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'analysis/prop-fuel-mass'


def main():
    native = CollisionNative()
    mass_native = MassMachine()
    base = native.alloc(0x1000)
    props, runtime = base + 0x50, base + 0x3c0
    checked = set()
    rows = []
    affected = []
    for name, entry in catalog().items():
        if not entry['supported']:
            continue
        _, asset = assets(name)
        if sum(initial(asset['tanks'], asset['fuel_systems'], .3)['reservoir']) > 0:
            affected.append(name)
        key = (entry['fm_id'], entry['resource'])
        if key in checked:
            continue
        checked.add(key)
        fm = load(name)
        p = aircraft_properties(fm)
        tanks, systems = asset['tanks'], asset['fuel_systems']
        for percent in [0., .1, 1., 30., 70., 100.]:
            native.u.mem_write(base, bytes(0x1000))
            native.u.mem_write(props + 8, struct.pack('<I', len(systems)))
            native.u.mem_write(props + 0x10, struct.pack('<I', len(tanks)))
            for j, t in enumerate(tanks):
                native.floats(props + 0xb0 + 16*j, [t['capacity']])
                native.u.mem_write(props + 0xb4 + 16*j,
                                   struct.pack('<IIB', t['system'], t['priority'], t['external']))
                native.u.mem_write(runtime + 0x60 + 8*j, bytes([not t['external']]))
            for j, s in enumerate(systems):
                native.floats(props + 0x1b0 + 28*j,
                              [s['capacity'], s['external_capacity'], s['reservoir_capacity']])
                native.u.mem_write(props + 0x1bc + 28*j, struct.pack('<I', s['priority_count']))
                amount = mul(sub(s['capacity'], s['external_capacity']), f32(percent / 100.))
                native.xmm(0, [amount])
                native.run(0x101990060, [props, runtime, 1, 0, j, 1])
            allocated = [native.read(runtime + 0xdc + 12*j, 1)[0] for j in range(len(systems))]
            for j in range(len(systems)):
                # Exact original fuel update, with consumption and dt zero:
                # preserve the selected load while establishing running totals.
                native.xmm(0, [0.]); native.xmm(1, [1.]); native.xmm(2, [0.])
                native.run(0x10199a210, [base, 1, j])
            actual = dict(
                fuel_by_tank=[native.read(runtime + 0x5c + 8*j, 1)[0] for j in range(len(tanks))],
                fuel_by_system=[native.read(runtime + 0xdc + 12*j, 1)[0] for j in range(len(systems))],
                reservoir=[native.read(runtime + 0xe4 + 12*j, 1)[0] for j in range(len(systems))])
            expected = initial(tanks, systems, percent / 100.)
            assert actual == expected, (name, percent, actual, expected)
            kw = dict(fuel_by_tank=actual['fuel_by_tank'], nitro=f32(fm['Mass'].get('MaxNitro', 0.)))
            if p['advanced_mass']:
                kw['mass_parts'] = asset['records']
            measured = mass_native.evaluate(p, actual['fuel_by_system'], **kw)
            website = mass_state(name, percent)
            assert all(website[k] == v for k, v in measured.items()), (name, percent, measured, website)
            rows.append(dict(aircraft=name, fm=entry['fm_id'], resource=entry['resource'],
                             fuel_percent=percent, allocator_fuel=sum(allocated),
                             running_fuel=website['fuel_mass'], reservoir=sum(actual['reservoir']),
                             mass=website['mass']))
        if len(checked) % 100 == 0:
            print(len(checked), 'assets PASS', flush=True)
    assert mass_state('yak-3', 30)['mass'] == 2464.
    assert mass_state('yak-3', 30)['fuel_mass'] == 81.
    report = dict(status='PASS', binary_sha256=native.sha, assets=len(checked), cases=len(rows),
                  affected_variants_at_30_percent=len(affected), affected_variants=affected,
                  scope=__doc__, rows=rows)
    OUT.mkdir(exist_ok=True)
    (OUT / 'native-validation.json').write_text(json.dumps(report, indent=2) + '\n')
    print('PASS', len(checked), 'assets', len(rows), 'cases;', len(affected), 'affected variants')


if __name__ == '__main__':
    main()
