"""Aircraft weapon loadouts: authored presets, custom slot editing and the
native store mass and drag rules.

Native rules (aces.exe, pinned build; unchanged in 2.59.0.80):
- The payload routine (0x142fe5600) walks every weapon of the aircraft. Each
  weapon adds one payload record (mass, attachment position) to the existing
  mass routine and adds its drag area to FM+0x8448 in float32.
- Store launchers (bombs, rockets, missiles, torpedoes; vtable 0x146c6f748)
  and flight-model modules (drop tanks, boosters, pods; 0x146c6fce8):
  mass = rounds * round mass + rack mass (0x1413e6430), drag area =
  rounds * round dragCx + rack dragCx when the mount is external
  (0x1413e64f0). The rack terms stay when every round is released.
- Rack mass and dragCx are the container chain totals (0x1414b9c10: mass and
  dragCx scaled by the running count, then count *= bullets, default 0),
  replaced by the mount's own `mass` / `dragCx` (0x141473770). A missing
  round dragCx is 0.03. `external` defaults to false.
- Guns, gun pods and countermeasures keep the existing ammunition rule
  (aircraft_ammunition.gun_payload); an external gun adds its weapon file's
  dragCx (0x141408a90), independent of the rounds left.
- dragCx is a drag area in m^2: the aerodynamics step uses
  area = door * CockpitDoorCd + FM+0x8448 as pure drag at the CG (no moment,
  lift or flow change); the Instructor predictor reads the same field.
- A drop tank switches on flight-model fuel tank `itemIndex1`; the spawn path
  (0x1411204d0) fills every present external tank completely.

Custom loadouts follow the game's own editor (gui.vromfs scripts
weaponryPresetsParams / weaponryPresets / weaponryInfo): editable slots are
WeaponSlot blocks with a `tier`; banned and dependent slot presets are
resolved the same way; a loadout is allowed only within maxloadMass, the
left/right console limits and maxDisbalance, using the editor's mass rule.
"""
import json
import math
from functools import lru_cache
from pathlib import Path, PurePosixPath
from component_assembly import f32, add, mul
from aircraft_ammunition import rows, source_name, gun_payload, attachment_frame

ROOT = Path(__file__).resolve().parents[1]
PROFILE = ROOT / 'references/aircraft-loadouts.json'
SCHEMA = 1
# Weapon subdirectories referenced by aircraft presets (rocketguns is the
# missile-sources inventory). Read at the same upstream commit as the presets.
WEAPON_DIRS = ('bombguns', 'containers', 'drop_tank', 'equipment', 'payloadguns', 'torpedoes', 'mines')
MIN_TIERS_COUNT = 13  # weaponryPresets.nut default slot count
STORE_KINDS = (('bombGun', 'bomb'), ('rocketGun', 'rocket'), ('torpedoGun', 'torpedo'),
               ('fuelTankGun', 'payload'), ('airDropGun', 'payload'), ('undercarriageGun', 'payload'),
               ('targetingPodGun', 'payload'), ('boosterGun', 'payload'))
ZERO_KINDS = ('lightBeam', 'beamGun', 'sonicGun')
MODULE_KINDS = ('fuelTankGun', 'boosterGun', 'airDropGun', 'undercarriageGun')


def number(value, default=0.):
    value = default if value is None else value
    if isinstance(value, list) and value:
        value = value[0]  # a repeated BLK parameter: the loaders read the first one
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError('Invalid weapon number')
    return value


def short(value):
    """Shortest decimal that rounds to the same binary32 value (f32() restores it exactly)."""
    import numpy as np
    return float(repr(np.float32(value)).removeprefix('np.float32(').removesuffix(')'))


STORE_FIELDS = ('c', 'tr', 'w', 'n', 'm', 'M', 'd', 'D', 'x', 't', 'booster', 'p')
GUN_FIELDS = ('tr', 'w', 'n', 'mass', 'hw', 'd', 'x', 'p')


def pack(item):
    """Compact list form of one physics item (the profile holds ~50,000)."""
    fields = GUN_FIELDS if item['k'] == 'g' else STORE_FIELDS
    return [item['k'], *(item[f] for f in fields)]


def unpack(row):
    return dict(zip(('k', *(GUN_FIELDS if row[0] == 'g' else STORE_FIELDS)), row))


def prune(value):
    return {k: v for k, v in value.items()
            if not (v is None or v is False or (isinstance(v, (list, dict, str)) and not v))}


def weapon_key(blk):
    return PurePosixPath(source_name(blk)).stem


class Compiler:
    def __init__(self, references, labels):
        self.references = references
        # Weapon file names are matched case-insensitively (labels keep the authored case).
        self.labels = {key.lower(): value for key, value in labels.items()}

        @lru_cache(maxsize=None)
        def read(name):
            relative = source_name(name).removeprefix('gamedata/weapons/')
            path = (references / 'missile-sources' / PurePosixPath(relative).name if relative.startswith('rocketguns/')
                    else references / 'weapon-sources' / relative)
            return json.loads(path.read_bytes())
        self.read = read

    def label(self, key):
        key = key.lower()
        return self.labels.get(f'weapons/{key}/short') or self.labels.get(f'weapons/{key}') or key

    def chain(self, blk):
        """Native container chain: rack totals, round count and the final weapon file."""
        document = self.read(blk)
        count, rack_mass, rack_drag, container, final = 1, 0., 0., False, blk
        depth = 0
        # A `container` sub-block (rocket node layout) is not the container flag.
        while document.get('container') is True:
            depth += 1
            if depth > 8 or 'blk' not in document:
                raise ValueError('Invalid weapon container: ' + blk)
            container = True
            rack_mass = add(mul(count, f32(number(document.get('mass')))), rack_mass)
            rack_drag = add(mul(count, f32(number(document.get('dragCx')))), rack_drag)
            bullets = document.get('bullets', 0)
            if isinstance(bullets, bool) or not isinstance(bullets, int) or bullets < 0:
                raise ValueError('Invalid container count: ' + blk)
            count *= bullets
            final = document['blk']
            document = self.read(final)
        return document, final, count, rack_mass, rack_drag, container

    def item(self, mount, nodes):
        if mount.get('dummy') or not mount.get('blk'):
            return None
        document, final, count, rack_mass, rack_drag, container = self.chain(mount['blk'])
        key = weapon_key(final)
        if any(document.get(flag) for flag in ZERO_KINDS):
            return None  # placeholder weapon class: zero mass and drag
        kind = next((block for flag, block in STORE_KINDS if document.get(flag)), None)
        trigger = mount.get('trigger', '')
        if kind is None:
            gun = gun_payload(mount, document, nodes)
            external = bool(mount.get('external', False))
            drag = f32(number(document.get('dragCx'))) if external else 0.
            return dict(k='g', tr=trigger, w=key, n=gun['count'], mass=short(gun['mass_kg']),
                        hw=short(gun['hardware_mass_kg']), d=short(drag), x=external,
                        p=[short(v) for v in gun['position']])
        rounds = mount.get('bullets', count if container else document.get('bullets', 1))
        if isinstance(rounds, bool) or not isinstance(rounds, int) or rounds < 0:
            raise ValueError('Invalid store count: ' + mount['blk'])
        block = next(iter(rows(document.get(kind))), {})
        tank = None
        if kind == 'payload' and block.get('itemType1') == 'fuelTank':
            tank = block.get('itemIndex1')
            if isinstance(tank, bool) or not isinstance(tank, int) or tank < 0:
                raise ValueError('Invalid drop tank index: ' + mount['blk'])
        external = bool(mount.get('external', False))
        _, _, matrix = attachment_frame(mount, nodes, emitter_offset=True)
        # Store frames: emitter translation plus the mount's emitterOffset in aircraft
        # axes (weapon-set builder 0x1414c04d9). The key is case-sensitive.
        offset = mount.get('emitterOffset', [0., 0., 0.])
        if (not isinstance(offset, list) or len(offset) != 3 or
                not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in offset)):
            raise ValueError('Invalid emitterOffset: ' + str(mount.get('emitter')))
        position = [add(matrix[9 + i], f32(offset[i])) for i in range(3)]
        return dict(k='s', c=kind, tr=trigger, w=key, n=rounds,
                    m=short(number(block.get('mass'))), M=short(number(mount.get('mass', rack_mass))),
                    d=short(number(block.get('dragCx'), .03)), D=short(number(mount.get('dragCx', rack_drag))),
                    x=external, t=tank, booster=bool(document.get('boosterGun')), p=[short(v) for v in position])

    def editor_mass(self, preset):
        """Tier mass of one slot preset by the game's editor rule (weaponryInfo.addWeaponsFromBlk
        and weaponryPresetsParams.getPresetWeightRestrictionText)."""
        groups = {}
        for weapon in rows(preset.get('Weapon')):
            if weapon.get('dummy') or (weapon.get('triggerGroup') == 'commander' and weapon.get('bullets') is None):
                continue
            if not weapon.get('blk'):
                continue
            # getWeaponBlkParams: counts include every level's `bullets` (default 1)
            bullets, containers, container_mass, path = 1, 1, 0., weapon['blk']
            document = self.read(path)
            while True:
                bullets *= number(document.get('bullets'), 1)
                if document.get('container') is True and 'blk' in document:
                    container_mass += number(document.get('mass')) * containers
                    containers = number(document.get('bullets'), 1)
                    path = document['blk']
                    document = self.read(path)
                    continue
                break
            trigger = weapon.get('trigger', '')
            tag = None
            if document.get('rocketGun'):
                tag = 'rocket'
            elif document.get('bombGun'):
                tag = 'bomb'
            elif document.get('torpedoGun'):
                tag = 'torpedo'
            elif trigger in ('machine gun', 'cannon', 'additional gun', 'rockets', 'agm', 'aam', 'guided bombs',
                             'bombs', 'torpedoes', 'smoke', 'flares', 'chaffs', 'countermeasures'):
                tag = 'bullet'
            elif not preset.get('showInWeaponMenu', False) and any(document.get(k) for k in MODULE_KINDS):
                continue  # drop tanks, boosters and air drops are not counted by the editor
            countermeasure = trigger in ('flares', 'chaffs', 'countermeasures') or weapon.get('triggerGroup') == 'smoke'
            mass = 0.
            if 'payload' in document:
                mass = number(next(iter(rows(document['payload'])), {}).get('mass'), mass)
            if countermeasure:
                mass = number(document.get('mass'), mass)
            elif tag and tag in document:
                mass = number(next(iter(rows(document[tag])), {}).get('mass'), mass)
            name = weapon_key(path)
            group = groups.setdefault(name, dict(mass=mass, amount=0., additional=0.))
            group['amount'] += number(weapon.get('bullets'), bullets)
            group['additional'] += number(weapon.get('mass'), number(document.get('mass'))) + container_mass
        # One weapon per tier in practice; extra weapon types are summed the same way.
        return sum(g['mass'] * g['amount'] + g['additional'] for g in groups.values())

    def vehicle(self, vehicle, nodes):
        items, index = [], {}

        def add_item(mount):
            value = self.item(mount, nodes)
            if value is None:
                return None
            key = json.dumps(value, sort_keys=True)
            if key not in index:
                index[key] = len(items)
                items.append(value)
            return index[key]

        slots = {s['index']: s for s in rows(vehicle.get('WeaponSlots', {}).get('WeaponSlot'))}

        def expand(weapons, visited=()):
            for weapon in rows(weapons):
                if 'slot' not in weapon:
                    yield weapon
                    continue
                key = (weapon['slot'], weapon['preset'])
                if key in visited:
                    raise ValueError('Cyclic weapon slot preset')
                choice = next((p for p in rows(slots.get(key[0], {}).get('WeaponPreset')) if p.get('name') == key[1]), None)
                if choice is None:
                    continue  # shared presets may name hardpoints absent on this variant
                yield from expand(choice.get('Weapon'), (*visited, key))

        def ids(weapons):
            return [i for i in (add_item(m) for m in expand(weapons)) if i is not None]

        def guarded(weapons):
            # An unverified mount (for example an attachment frame modifier) withholds only
            # the preset or pylon option that uses it, not the whole aircraft.
            try:
                return dict(items=ids(weapons))
            except (ValueError, KeyError, OSError) as error:
                return dict(items=[], error=str(error))

        common = ids(vehicle.get('commonWeapons', {}).get('Weapon'))
        settings = vehicle.get('WeaponSlots', {})
        count = settings.get('weaponsSlotCount', MIN_TIERS_COUNT)
        tier_of = {s['index']: s['tier'] if 'tier' in s else count - 1 - s['index'] for s in slots.values()}
        presets = []
        for preset in rows(vehicle.get('weapon_presets', {}).get('preset')):
            # (pruned on output: empty tiers/items and false flags are omitted)
            try:
                document = self.read_preset(preset['blk'])
            except (OSError, ValueError) as error:
                presets.append(dict(id=preset['name'], req=preset.get('reqModification'), items=[],
                                    error='Preset file unavailable: ' + str(error).split(':')[0]))
                continue
            tiers = {}
            for weapon in rows(document.get('Weapon')):
                if 'slot' in weapon and 'tier' in slots.get(weapon['slot'], {}):
                    tiers[str(tier_of[weapon['slot']])] = weapon['preset']
            presets.append(dict(id=preset['name'], req=preset.get('reqModification'), tiers=tiers,
                                hidden=not preset.get('showInWeaponMenu', True), **guarded(document.get('Weapon'))))
        editor = None
        editable = [s for s in slots.values() if 'tier' in s]
        if editable:
            tiers = []
            for slot in sorted(editable, key=lambda s: (s['tier'], s['index'])):
                options = []
                for option in rows(slot.get('WeaponPreset')):
                    options.append(dict(
                        id=option['name'], req=option.get('reqModification'), icon=option.get('iconType'),
                        mass=short(self.editor_mass(option)),
                        banned=[[b['slot'], b['preset']] for b in rows(option.get('BannedWeaponPreset'))],
                        dependent=[[d['slot'], d['preset']] for d in rows(option.get('DependentWeaponPreset'))],
                        **guarded(option.get('Weapon'))))
                tiers.append(dict(tier=slot['tier'], slot=slot['index'], options=options,
                                  center=bool(slot.get('notUseforDisbalanceCalculation', False))))
            limit = lambda key: number(settings.get(key), -1)
            editor = dict(count=count, tiers=tiers,
                          limits=dict(total=limit('maxloadMass'), left=limit('maxloadMassLeftConsoles'),
                                      right=limit('maxloadMassRightConsoles'), disbalance=limit('maxDisbalance')))
        labels = {}
        for item in items:
            labels.setdefault(item['w'], self.label(item['w']))
        return dict(supported=True, items=[pack(i) for i in items], labels=labels, common=common,
                    presets=[prune(p) for p in presets], editor=editor and dict(editor, tiers=[
                        dict(t, options=[prune(o) for o in t['options']]) for t in editor['tiers']]))

    def read_preset(self, blk):
        return json.loads((self.references / 'weapon-presets' / PurePosixPath(source_name(blk)).name).read_bytes())


def generate(stage, records, geometry_root=None, labels=None):
    references = stage / 'references'
    geometry = Path(geometry_root or stage) / 'references/aircraft-skeletons'
    if labels is None:
        names = references / 'missile-names.json'
        labels = json.loads(names.read_bytes())['labels'] if names.exists() else {}
    compiler = Compiler(references, labels)
    profiles = {}
    for record in records:
        name = record['vehicle']
        vehicle = json.loads((references / 'prop-vehicles' / (name + '.blkx')).read_bytes())
        if 'helicopter' in vehicle or not record.get('model'):
            continue
        try:
            model = record['model']
            if PurePosixPath(model).name != model or '\\' in model or ':' in model:
                raise ValueError('Unsafe aircraft geometry name')
            path = geometry / (model + '.json')
            if not path.exists():
                raise ValueError('Weapon attachment inputs have not been compiled')
            profiles[name] = compiler.vehicle(vehicle, json.loads(path.read_bytes())['nodes'])
        except (ValueError, KeyError, OSError) as error:
            profiles[name] = dict(supported=False, reason=str(error))
    return profiles


# ---------------------------------------------------------------- runtime

@lru_cache(maxsize=1)
def profiles():
    data = json.loads(PROFILE.read_bytes())
    vehicles = json.loads((ROOT / 'references/prop-vehicles/manifest.json').read_bytes())
    if data['commit'] != vehicles['commit']:
        raise ValueError('Loadout inputs do not match aircraft inputs')
    if data.get('schema') != SCHEMA:
        raise ValueError('Loadout inputs need rebuilding')
    return data['aircraft']


def vehicle_profile(name, vehicle=''):
    """Loadouts of the selected variant (catalog entries may share one FM)."""
    from aircraft_ammunition import profile as ammunition_profile
    available = profiles()
    ammo = ammunition_profile(name, vehicle or None)
    selected = vehicle or ammo.get('vehicle_id') or name
    value = available.get(selected)
    if value is None:
        return dict(supported=False, reason='No loadout data for ' + selected)
    if value.get('supported') and value['items'] and isinstance(value['items'][0], list):
        value['items'] = [unpack(row) for row in value['items']]
        for item in value['items']:
            item['name'] = value['labels'].get(item['w'], item['w'])
        for preset in value['presets']:
            preset.setdefault('tiers', {});preset.setdefault('items', []);preset.setdefault('req', None)
        for tier in (value['editor'] or {}).get('tiers', []):
            for option in tier['options']:
                for key in ('banned', 'dependent', 'items'):option.setdefault(key, [])
                option.setdefault('req', None);option.setdefault('icon', None)
    return value


def normalize(loadout):
    """Request form: '' (clean), a preset id, or {'tiers': {tier: slot preset}}."""
    if loadout in (None, '', 'clean'):
        return ''
    if isinstance(loadout, str):
        if len(loadout) > 120:
            raise ValueError('Invalid loadout preset')
        return loadout
    if isinstance(loadout, dict) and set(loadout) <= {'tiers'} and isinstance(loadout.get('tiers'), dict):
        tiers = loadout['tiers']
        if len(tiers) > 64 or any(not isinstance(k, str) or not k.isdigit() or not isinstance(v, str) or len(v) > 120
                                  for k, v in tiers.items()):
            raise ValueError('Invalid custom loadout')
        return dict(tiers={str(int(k)): v for k, v in sorted(tiers.items(), key=lambda kv: int(kv[0])) if v})
    raise ValueError('Invalid loadout')


def option_lookup(editor):
    by_tier = {t['tier']: t for t in editor['tiers']}
    by_slot = {}
    for t in editor['tiers']:
        for option in t['options']:
            by_slot[(t['slot'], option['id'])] = (t, option)
    return by_tier, by_slot


def restrictions(editor, tiers):
    """getPresetWeightRestrictionText: [] when the editor would save the loadout."""
    limits = editor['limits']
    if all(v < 0 for v in limits.values()):
        return [], dict(left=0., right=0., center=0., total=0.)
    by_tier, _ = option_lookup(editor)
    center_index = editor['count'] // 2
    left = right = center = 0.
    for tier, preset in tiers.items():
        t = by_tier[int(tier)]
        option = next(o for o in t['options'] if o['id'] == preset)
        if t['center']:
            center += option['mass']
        elif int(tier) < center_index:
            left += option['mass']
        else:
            right += option['mass']
    problems = []
    if abs(left - right) > limits['disbalance']:
        problems.append(f"Left/right mass difference {abs(left - right):.0f} kg exceeds {limits['disbalance']:.0f} kg")
    total = left + right + center
    for value, key, text in ((total, 'total', 'Total'), (left, 'left', 'Left'), (right, 'right', 'Right')):
        if limits[key] >= 0 and value > limits[key]:
            problems.append(f'{text} pylon mass {value:.0f} kg exceeds {limits[key]:.0f} kg')
    return problems, dict(left=left, right=right, center=center, total=total)


def check_custom(editor, tiers):
    """Validate a custom loadout: options exist, no banned pair, dependents present, weight limits."""
    by_tier, by_slot = option_lookup(editor)
    installed = {}
    for tier, preset in tiers.items():
        t = by_tier.get(int(tier))
        option = next((o for o in t['options'] if o['id'] == preset), None) if t else None
        if option is None:
            raise ValueError(f'Pylon {int(tier) + 1} cannot carry {preset}')
        if option.get('error'):
            raise ValueError(f'{preset} is unavailable: ' + option['error'])
        installed[t['slot']] = preset
    for slot, preset in installed.items():
        option = by_slot[(slot, preset)][1]
        for banned_slot, banned in option['banned']:
            if installed.get(banned_slot) == banned:
                raise ValueError(f'{preset} cannot be carried together with {banned}')
        for need_slot, need in option['dependent']:
            if (need_slot, need) in by_slot and installed.get(need_slot) != need:
                raise ValueError(f'{preset} needs {need} on its paired pylon')
    problems, masses = restrictions(editor, tiers)
    if problems:
        raise ValueError('; '.join(problems))
    return masses


def resolve(name, vehicle, loadout, released=False, ammunition=True):
    """Ordered payload items of a loadout, or None for the clean default."""
    loadout = normalize(loadout)
    if loadout == '':
        return None
    value = vehicle_profile(name, vehicle)
    if not value['supported']:
        raise ValueError('Loadouts are unavailable for ' + name + ': ' + value['reason'])
    order = list(value['common'])
    if isinstance(loadout, str):
        preset = next((p for p in value['presets'] if p['id'] == loadout), None)
        if preset is None:
            raise ValueError('Unknown loadout preset: ' + loadout)
        if preset.get('error'):
            raise ValueError(f'Loadout {loadout} is unavailable: ' + preset['error'])
        order += preset['items']
        label = loadout
    else:
        editor = value['editor']
        if not editor:
            raise ValueError('This aircraft has no custom pylons')
        check_custom(editor, loadout['tiers'])
        by_tier, _ = option_lookup(editor)
        for tier, preset in loadout['tiers'].items():
            order += next(o for o in by_tier[int(tier)]['options'] if o['id'] == preset)['items']
        label = 'custom'
    items = [value['items'][i] for i in order]
    return dict(state(items, released=released, ammunition=ammunition), label=label)


def state(items, released=False, ammunition=True):
    """Payload records, float32 drag area (game summation order) and present drop tanks."""
    payloads, tanks, area = [], [], 0.
    store_mass = gun_mass = 0.
    for item in items:
        if item['k'] == 'g':
            mass = item['mass'] if ammunition else item['hw']
            drag = item['d']
            gun_mass += mass
        else:
            rounds = 0 if released else item['n']
            mass = add(mul(rounds, item['m']), item['M'])
            drag = add(mul(rounds, item['d']), item['D']) if item['x'] else 0.
            store_mass += mass
            if item['t'] is not None and rounds > 0:
                tanks.append(item['t'])
        payloads.append(dict(mass=mass, position=list(item['p'])))
        area = add(drag, area)
    return dict(payloads=payloads, drag_area=area, external_tanks=sorted(set(tanks)),
                store_mass_kg=store_mass, gun_mass_kg=gun_mass, items=items, released=released)


def availability(name):
    """Small catalog flag; the full description comes from /api/loadouts/<aircraft>."""
    try:
        from aircraft_ammunition import profile as ammunition_profile
        from aircraft_upgrades import profile as upgrades
        variants = upgrades(name)['vehicle_sources'] if name not in profiles() else [name]
        rows_ = {v: profiles().get(v, dict(supported=False)) for v in variants}
        supported = {v: r for v, r in rows_.items() if r.get('supported')}
        if not supported:
            return dict(supported=False, reason=next(iter(rows_.values()), {}).get('reason', 'No loadout data'))
        return dict(supported=True, presets=max(sum(not p.get('hidden') for p in r['presets']) for r in supported.values()),
                    custom=any(r['editor'] for r in supported.values()))
    except (OSError, ValueError, KeyError) as error:
        return dict(supported=False, reason=str(error))


@lru_cache(maxsize=64)
def cached_summary(name, vehicle=''):
    return summary(name, vehicle)


def display_label(name, vehicle, loadout, released=False):
    """Short legend text for a loadout."""
    loadout = normalize(loadout)
    if loadout == '':
        return 'Clean'
    try:
        info = cached_summary(name, vehicle)
        if isinstance(loadout, str):
            text = next((p['label'] for p in info['presets'] if p['id'] == loadout), loadout)
        else:
            tiers = {t['tier']: t for t in (info['editor'] or {}).get('tiers', [])}
            counts = {}
            for tier, option in loadout['tiers'].items():
                label = next((o['label'] for o in tiers.get(int(tier), {}).get('options', []) if o['id'] == option), option)
                counts[label] = counts.get(label, 0) + 1
            text = 'Custom: ' + (', '.join(f'{n}× ({k})' if n > 1 else k for k, n in counts.items()) or 'empty')
    except (OSError, ValueError, KeyError):
        text = loadout if isinstance(loadout, str) else 'Custom'
    if len(text) > 48:
        text = text[:47] + '…'
    return text + (' (released)' if released else '')


def summary(name, vehicle=''):
    """Interface description of an aircraft's presets and pylons (no physics)."""
    value = vehicle_profile(name, vehicle)
    if not value['supported']:
        return dict(supported=False, reason=value['reason'])

    common = set(value['common'])

    def describe(item_ids):
        """Label, mass and drag of the weapons a preset or pylon adds (common guns excluded)."""
        own = [i for i in item_ids if i not in common]
        if not own:
            return dict(label='Empty', mass_kg=0., drag_area_m2=0.)
        stores, guns = {}, {}
        for i in own:
            item = value['items'][i]
            target = stores if item['k'] == 's' else guns
            target[item['name']] = target.get(item['name'], 0) + (item['n'] if item['k'] == 's' else 1)
        parts = stores or guns
        s = state([value['items'][i] for i in own])
        label = ', '.join(f'{n}× {k}' if n > 1 else k for k, n in parts.items())
        return dict(label=label, mass_kg=round(s['store_mass_kg'] + s['gun_mass_kg'], 1), drag_area_m2=round(s['drag_area'], 5))

    presets = [dict(id=p['id'], req=p.get('req'), tiers=p['tiers'], error=p.get('error'), **describe(p['items']))
               for p in value['presets'] if not p.get('hidden')]
    editor = None
    if value['editor']:
        e = value['editor']
        editor = dict(count=e['count'], limits=e['limits'],
                      tiers=[dict(tier=t['tier'], slot=t['slot'], center=t['center'],
                                  options=[dict(id=o['id'], req=o['req'], icon=o['icon'], editor_mass_kg=o['mass'], error=o.get('error'),
                                                banned=o['banned'], dependent=o['dependent'], **describe(o['items']))
                                           for o in t['options']]) for t in e['tiers']])
    return dict(supported=True, presets=presets, editor=editor)
