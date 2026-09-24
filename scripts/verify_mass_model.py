"""Execute the complete mass wrapper/consumer with synthetic runtime records."""
import json, random, struct
from pathlib import Path
from unicorn.x86_const import *
from macho_scan import MachO
from component_assembly import f32
from verify_component_stages import Stages, BASE, FRAME
from mass_model import evaluate, aircraft_properties, tank_configuration


class MassMachine(Stages):
    def __init__(self):
        super().__init__()
        m = MachO()
        for va, size in [(0x101998000, 0x2000), (0x107615000, 0x1000)]:
            self.u.mem_map(va, size)
            self.u.mem_write(va, m.read(va, size))

    def records(self, address, records):
        for n, rec in enumerate(records):
            p = address + n * 36
            self.floats(p, [rec['mass']])
            self.floats(p + 8, rec.get('specific_inertia', [0., 0., 0.]))
            self.floats(p + 20, rec['position'])
            self.u.mem_write(p + 33, bytes([rec.get('fuel_tank') if rec.get('fuel_tank') is not None else 255]))

    def evaluate(self, properties, fuel_by_system, payloads=(), damage_parts=(),
                 fuel_by_tank=(), payload_cg_scale=None, payload_inertia_scale=None,
                 inertia_modifiers=(1., 1., 1.), presence_mask=None, nitro=0., mass_parts=()):
        self.reset()
        if payload_cg_scale is None:
            payload_cg_scale = properties.get('payload_cg_scale', 1.)
        if payload_inertia_scale is None:
            payload_inertia_scale = properties.get('payload_inertia_scale', 1.)
        if presence_mask is None:
            presence_mask = (1 << len(damage_parts)) - 1
        self.u.mem_write(0x107615358, struct.pack('<Q', BASE + 0xf000))
        self.floats(BASE + 0x50, [properties['empty']])
        self.floats(BASE + 0x64, [properties['crew_mass'], properties['oil']])
        self.floats(BASE + 0x3d4, [nitro])
        self.floats(BASE + 0x78, properties['configured_cog'])
        self.floats(BASE + 0x84, properties.get('cog_y_limits', [-2147440000., 2147440000.]))
        self.u.mem_write(BASE + 0x90, struct.pack('<3d', *properties['normalized_inertia']))
        self.u.mem_write(BASE + 0x3e8, struct.pack('<3d', *inertia_modifiers))
        self.u.mem_write(BASE + 0xc8, bytes([properties.get('payload_affects_cog', False)]))
        self.u.mem_write(BASE + 0x58, struct.pack('<I', len(fuel_by_system)))
        self.u.mem_write(BASE + 0x60, struct.pack('<I', len(fuel_by_tank)))
        for n, fuel in enumerate(fuel_by_system):
            self.floats(BASE + 0x49c + 12 * n, [fuel])
        for n, fuel in enumerate(fuel_by_tank):
            self.floats(BASE + 0x41c + 8 * n, [fuel])
        self.u.mem_write(BASE + 0xd0, struct.pack('<Q', BASE + 0x4000))
        self.u.mem_write(BASE + 0xe0, struct.pack('<I', len(damage_parts)))
        self.records(BASE + 0x4000, damage_parts)
        self.u.mem_write(BASE + 0xa8, bytes([properties.get('advanced_mass', False)]))
        self.u.mem_write(BASE + 0xb0, struct.pack('<Q', BASE + 0x2000))
        self.u.mem_write(BASE + 0xc0, struct.pack('<I', len(mass_parts)))
        self.records(BASE + 0x2000, mass_parts)
        self.records(BASE + 0x6000, payloads)
        ctx = BASE + 0x8000
        self.u.mem_write(ctx + 8, struct.pack('<Q', BASE + 0x6000))
        self.u.mem_write(ctx + 24, struct.pack('<I', len(payloads)))
        self.floats(ctx + 32, [payload_cg_scale, payload_inertia_scale])
        self.u.mem_write(ctx + 40, struct.pack('<Q', presence_mask))
        stack, stop = FRAME - 0x1000, BASE + 0x2f000
        self.u.reg_write(UC_X86_REG_RDI, BASE)
        self.u.reg_write(UC_X86_REG_RSI, ctx)
        self.u.reg_write(UC_X86_REG_RSP, stack)
        self.u.mem_write(stack, struct.pack('<Q', stop))
        self.u.emu_start(0x101998ad0, stop, count=30000)
        return dict(mass=self.read3(BASE + 0x3c0)[0],
                    payload_mass=self.read3(BASE + 0x3c4)[0],
                    payload_inertia=self.read3(BASE + 0x3c8),
                    cog=self.read3(BASE + 0x3d8),
                    inertia=list(struct.unpack('<3d', self.u.mem_read(BASE + 0x400, 24))))


def main():
    machine, rng = MassMachine(), random.Random(0x19998bf0)
    failures, cases = [], 0
    def vals(n, a, b):
        return [f32(rng.uniform(a, b)) for _ in range(n)]
    for k in range(1600):
        properties = dict(empty=f32(rng.uniform(100, 40000)), oil=f32(rng.uniform(0, 200)),
                          crew_mass=f32(rng.randint(1, 10) * 90), configured_cog=vals(3, -2, 2),
                          normalized_inertia=[rng.uniform(2, 50) for _ in range(3)],
                          payload_affects_cog=bool(k & 1), cog_y_limits=(-1.25, 1.25))
        fuel = vals(k % 17, 0, 400)
        payloads = [dict(mass=f32(rng.uniform(0, 300)), position=vals(3, -7, 7))
                    for _ in range(k % 19)]
        tanks = vals(k % 13, 0, 200)
        parts = [dict(mass=f32(rng.uniform(0, 700)), position=vals(3, -7, 7),
                      specific_inertia=vals(3, 0, 3),
                      fuel_tank=rng.randrange(len(tanks)) if tanks and rng.random() < .5 else None)
                 for _ in range(k % 14)]
        kwargs = dict(payloads=payloads, damage_parts=parts, fuel_by_tank=tanks,
                      payload_cg_scale=f32(rng.uniform(0, 1.5)),
                      payload_inertia_scale=f32(rng.uniform(0, 1.5)),
                      inertia_modifiers=[rng.uniform(.5, 2.) for _ in range(3)],
                      nitro=f32(rng.uniform(0, 20)),
                      presence_mask=None if k % 3 == 0 else rng.getrandbits(len(parts)))
        expected = evaluate(properties, fuel, **kwargs)
        actual = machine.evaluate(properties, fuel, **kwargs)
        cases += 1
        bad = {key: dict(actual=actual[key], expected=expected[key])
               for key in actual if actual[key] != expected[key]}
        if bad:
            failures.append(dict(case=k, differences=bad, properties=properties,
                                 fuel_by_system=fuel, inputs=kwargs))
    aircraft = {}
    for name in ['f_16a_block_15_adf', 'saab_jas39c']:
        fm = json.loads(Path('references/fm/' + name + '.blkx').read_text())
        p = aircraft_properties(fm)
        tanks = tank_configuration(fm)
        internal = sum(t['capacity'] for t in tanks if not t['external'])
        external = sum(t['capacity'] for t in tanks if t['external'])
        aircraft[name] = dict(properties=p, declared_internal_fuel=internal,
                              declared_external_fuel=external,
                              max_fuel_config=fm['Mass']['MaxFuelMass0'])
        for amount in [0., internal / 2, internal]:
            actual = machine.evaluate(p, [amount])
            expected = evaluate(p, [amount])
            cases += 1
            if any(actual[key] != expected[key] for key in actual):
                failures.append(dict(aircraft=name, fuel=amount, actual=actual, expected=expected))
    result = dict(binary_sha256=machine.sha, contiguous_executions=cases,
                  span='0x101998ad0 wrapper, complete 0x101998bf0 consumer, wrapper return',
                  comparison='Exact numeric equality for mass, CG, diagonal inertia and payload aggregates',
                  aircraft=aircraft, failures=failures,
                  limitations='Prepared payload/damage-part records and CG clamps. AdvancedMass=false only. '
                  'No live mesh/weapon-data record capture; no fuel depletion/allocation step.')
    Path('analysis/mass-model-validation.json').write_text(json.dumps(result, indent=2))
    print(json.dumps({key: value for key, value in result.items() if key not in ['failures', 'aircraft']}, indent=2))
    print('FAILURES', len(failures))
    print(json.dumps(failures[:3], indent=2))
    if failures:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
