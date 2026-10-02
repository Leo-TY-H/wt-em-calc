"""Default, unmodified gun and countermeasure ammunition from authored inputs.

Native weapon payload rules: first projectile mass times count, at each weapon
attachment transformed cmOffset. External launcher hardware is included only
where the native payload getter adds it; internal guns are in empty mass.
"""
import json
import math
from functools import lru_cache
from pathlib import Path, PurePosixPath
from component_assembly import f32, add, mul

ROOT = Path(__file__).resolve().parents[1]
PROFILE = ROOT / 'references/aircraft-ammunition.json'
WEAPONS = 'aces.vromfs.bin_u/gamedata/weapons'
PRESETS = 'aces.vromfs.bin_u/gamedata/flightmodels/weaponpresets'


def rows(value):
    return value if isinstance(value, list) else [value] if value else []


def source_name(value):
    path = PurePosixPath(value.lower())
    if path.is_absolute() or '..' in path.parts or '\\' in value or ':' in value:
        raise ValueError('Unsafe ammunition source path')
    return str(path.with_suffix('.blkx'))


def default_preset(vehicle):
    presets = rows(vehicle.get('weapon_presets', {}).get('preset'))
    return next((p['blk'] for p in presets if not p.get('reqModification')), None)


def resolve_weapons(vehicle, preset):
    slots = {s['index']: s for s in rows(vehicle.get('WeaponSlots', {}).get('WeaponSlot'))}

    def expand(weapons, visited=(), required=True):
        for weapon in rows(weapons):
            if weapon.get('reqModification'):
                continue
            if 'slot' not in weapon:
                yield weapon
                continue
            key = (weapon['slot'], weapon['preset'])
            if key in visited:
                raise ValueError('Cyclic default weapon slot')
            slot = slots.get(key[0], {})
            choices = [p for p in rows(slot.get('WeaponPreset')) if p.get('name') == key[1]]
            choices = list({json.dumps(p, sort_keys=True): p for p in choices}.values())
            if len(choices) != 1:
                if not choices and not required:
                    # Some shared external presets refer to hardpoints absent
                    # on this variant; those mounts do not exist on the aircraft.
                    continue
                raise ValueError('Missing or ambiguous default weapon slot: ' + str(key))
            choice = choices[0]
            if choice.get('reqModification'):
                continue
            yield from expand(choice.get('Weapon'), (*visited, key), required)

    return list(expand(vehicle.get('commonWeapons', {}).get('Weapon'))) + list(expand(preset.get('Weapon'), required=False))


def ammunition(vehicle, preset, read_weapon, nodes=None):
    weapons = []
    for mount in resolve_weapons(vehicle, preset):
        trigger = mount.get('trigger', '')
        if mount.get('dummy') or not (trigger in ('cannon', 'machine gun', 'countermeasures') or trigger.startswith('gunner')):
            continue
        document = read_weapon(mount['blk'])
        if document.get('rocketGun') or document.get('bombGun'):
            raise ValueError('Unsupported default ammunition weapon class: ' + mount['blk'])
        count = mount.get('bullets', document.get('bullets'))
        if isinstance(count, bool) or not isinstance(count, int) or count < 0:
            raise ValueError('Invalid default ammunition count: ' + mount['blk'])
        belt = rows(document.get('bullet'))
        if not belt:
            raise ValueError('Missing default ammunition belt: ' + mount['blk'])
        masses = [b.get('mass') for b in belt]
        if any(isinstance(m, bool) or not isinstance(m, (int, float)) or not math.isfinite(m) or m < 0 for m in masses):
            raise ValueError('Invalid ammunition mass: ' + mount['blk'])
        # 0x1414088e0: first resolved projectile, not the average of a belt.
        ammunition_mass = mul(count, masses[0])
        hardware = document.get('mass', 0.) if mount.get('external', False) else 0.
        if not isinstance(hardware, (int, float)) or not math.isfinite(hardware) or hardware < 0:
            raise ValueError('Invalid external launcher mass: ' + mount['blk'])
        mass = add(ammunition_mass, hardware)
        weapon = dict(source=source_name(mount['blk']), kind='countermeasure' if trigger == 'countermeasures' else 'gun',
                      count=count, mass_kg=mass, ammunition_mass_kg=ammunition_mass,
                      hardware_mass_kg=f32(hardware), emitter=mount.get('emitter'))
        if nodes is not None:
            emitter = mount.get('emitter')
            # Fixed Cannon instances copy the emitter transform (0x14147eca1).
            # Turret setup subsequently replaces it with the head transform
            # (0x1414cc89f), so using a gun muzzle for a turret is incorrect.
            turret = next((value for value in rows(mount.get('turret')) if isinstance(value, dict)), {})
            attachment = turret.get('head', emitter)
            # Node lookup in the aircraft loader is case-insensitive.
            attachment = next((n for n in nodes if n.casefold() == str(attachment).casefold()), attachment)
            if attachment not in nodes and turret:
                raise ValueError('Missing weapon attachment: ' + str(attachment))
            # The per-mount loader resets cmOffset to its own default (zero).
            # It does not inherit the weapon file's cmOffset at this stage.
            offset = mount.get('cmOffset', [0., 0., 0.])
            # With a decoded skeleton but no emitter match the native loader
            # leaves an identity matrix at the model origin (0x1414c04d3).
            # This is distinct from missing geometry, which remains unsupported.
            matrix = nodes.get(attachment, [1.,0.,0.,0.,1.,0.,0.,0.,1.,0.,0.,0.])
            # No current default mounts use these additional loader frames.
            # Keep future unverified frame changes out of published results.
            if mount.get('parentNode') or mount.get('emitterOffset'):
                raise ValueError('Unsupported weapon attachment modifier: ' + str(attachment))
            if (not isinstance(offset, (list, tuple)) or not isinstance(matrix, (list, tuple)) or
                    len(offset) != 3 or len(matrix) != 12 or
                    not all(isinstance(x, (int, float)) and math.isfinite(x) for x in [*offset, *matrix])):
                raise ValueError('Invalid weapon mass transform: ' + str(attachment))
            if any(offset) and not turret:
                # The native fixed-gun loader removes scale from its basis.
                # Zero offsets (all current stock mounts) use translation only.
                basis = [matrix[i:i+3] for i in (0,3,6)]
                if any(abs(sum(a*b for a,b in zip(basis[i],basis[j]))-int(i==j))>1e-4
                       for i in range(3) for j in range(3)):
                    raise ValueError('Unsupported scaled weapon cmOffset: ' + str(attachment))
            weapon['position'] = [add(add(add(mul(matrix[i], offset[0]), mul(matrix[3+i], offset[1])),
                                                mul(matrix[6+i], offset[2])), matrix[9+i]) for i in range(3)]
            weapon['cm_offset'] = offset
            weapon['mass_attachment'] = attachment
            if attachment not in nodes:
                weapon['attachment_fallback'] = 'Native missing-emitter identity transform at model origin'
        weapons.append(weapon)
    supported = nodes is not None or not weapons
    return dict(supported=supported, **({} if supported else {'reason': 'Weapon attachment inputs have not been compiled'}),
                gun_rounds=sum(w['count'] for w in weapons if w['kind'] == 'gun'),
                countermeasures=sum(w['count'] for w in weapons if w['kind'] == 'countermeasure'),
                mass_kg=math.fsum(w['mass_kg'] for w in weapons), weapons=weapons,
                policy='Native default ammunition and external launcher payloads at weapon attachments; no armament modifications')


def generate(stage, records):
    references = stage / 'references'

    @lru_cache(maxsize=None)
    def read_weapon(name):
        relative = source_name(name).removeprefix('gamedata/weapons/')
        path = (references / 'missile-sources' / PurePosixPath(relative).name if relative.startswith('rocketguns/')
                else references / 'weapon-sources' / relative)
        return json.loads(path.read_bytes())

    profiles = {}
    for record in records:
        name = record['vehicle']
        vehicle = json.loads((references / 'prop-vehicles' / (name + '.blkx')).read_bytes())
        if 'helicopter' in vehicle or not record.get('model'):
            continue
        try:
            selected = default_preset(vehicle)
            preset = json.loads((references / 'weapon-presets' / PurePosixPath(source_name(selected)).name).read_bytes()) if selected else {}
            geometry = references / 'aircraft-skeletons' / (record['model'] + '.json')
            skeleton = json.loads(geometry.read_bytes()) if geometry.exists() else None
            profiles[name] = ammunition(vehicle, preset, read_weapon, skeleton['nodes'] if skeleton else None)
            if skeleton:
                profiles[name]['geometry'] = skeleton['provenance']
        except (ValueError, KeyError, OSError) as error:
            profiles[name] = dict(supported=False, reason=str(error))
    return profiles


@lru_cache(maxsize=1)
def profiles():
    data = json.loads(PROFILE.read_bytes())
    vehicles = json.loads((ROOT / 'references/prop-vehicles/manifest.json').read_bytes())
    if data['commit'] != vehicles['commit']:
        raise ValueError('Default ammunition inputs do not match aircraft inputs')
    if data.get('schema') != 2:
        raise ValueError('Default ammunition positions need rebuilding')
    return data['aircraft']


def profile(name, vehicle=None):
    try:
        available = profiles()
        if name in available:
            if vehicle and vehicle != name:
                raise ValueError('Ammunition vehicle does not match the selected aircraft')
            return available[name]
        # Some catalog entries intentionally combine vehicles sharing one FM.
        # Expose those layouts to the interface so its selected variant is visible.
        from aircraft_upgrades import profile as upgrades
        candidates = {n: available[n] for n in upgrades(name)['vehicle_sources']}
        selected = vehicle or next(iter(candidates))
        if selected not in candidates:
            raise ValueError('Ammunition vehicle does not share the selected flight model')
        return dict(candidates[selected], vehicle_id=selected, variants=candidates)
    except (OSError, ValueError, KeyError) as error:
        return dict(supported=False, reason=str(error))


def selected_mass(name, vehicle=None):
    value = profile(name, vehicle)
    if not value['supported']:
        raise ValueError('Default ammunition is unavailable for ' + name + ': ' + value['reason'])
    return value['mass_kg']


def selected_payloads(name, vehicle=None):
    value = profile(name, vehicle)
    if not value['supported']:
        raise ValueError('Default ammunition is unavailable for ' + name + ': ' + value['reason'])
    return [dict(mass=w['mass_kg'], position=list(w['position'])) for w in value['weapons']]
