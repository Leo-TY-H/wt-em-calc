"""Compile carrier compatibility from pinned aircraft/preset/weapon sources.

Sensor extracts come from the same commit, retrieved through the GitHub tool.
No whole data tree is copied. The output is a compact runtime lookup.
"""
import argparse
from functools import lru_cache
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]


def rows(value):
    return value if isinstance(value, list) else [value] if isinstance(value, dict) else []


def key(value):
    return Path(value.replace('\\', '/')).stem.lower()


def mounts(value):
    if isinstance(value, dict):
        count = value.get('bullets', 1)
        if isinstance(count, list): count = count[0] if count else 0
        if isinstance(value.get('blk'), str) and not value.get('dummy') and isinstance(count, (int, float)) and count > 0:
            yield value
        for item in value.values():
            yield from mounts(item)
    elif isinstance(value, list):
        for item in value:
            yield from mounts(item)


def reviewed_assets(ident, base_sha, review, resolve):
    """Apply only mounted weapons evidenced by a newer, pinned vehicle source.

    A base hash mismatch forces review when refreshing the main dataset; a
    previously valid addition must not silently survive a later loadout removal.
    The runtime catalog retains the newer source for each added pairing.
    """
    update = review.get('aircraft_updates', {}).get(ident)
    if update is None: return set()
    if update['base_sha'] != base_sha:
        raise ValueError(f'Reviewed launcher update is stale for {ident}')
    found = set()
    for mount in mounts(update['mounts']):
        if not mount['blk'].lower().replace('\\', '/').startswith('gamedata/weapons/rocketguns/'):
            raise ValueError(f'Reviewed launcher update needs a direct missile asset: {ident}')
        asset = resolve(key(mount['blk']))
        if asset is None: raise ValueError(f'Missing missile profile in reviewed update for {ident}')
        found.add(asset)
    if not found: raise ValueError(f'Empty reviewed launcher update for {ident}')
    return found


def inventory(references, loadouts=None, weapon_inputs=None):
    from missile_names import display_name
    profiles = {}
    for path in (ROOT / 'scripts/missile_model/launch-profiles').glob('*.json'):
        if path.name == 'index.json': continue
        p = json.loads(path.read_bytes())
        profiles[path.stem] = dict(family=p['family'], name=display_name(path.stem, p['properties']['rocket']))
    groups = {}
    for asset, p in profiles.items(): groups.setdefault(p['name'].casefold(), []).append(asset)
    sources = {}
    @lru_cache(maxsize=None)
    def read(path):
        raw = path.read_bytes()
        sources[path.relative_to(references).as_posix()] = hashlib.sha256(raw).hexdigest()
        return json.loads(raw)
    weapons = {p.stem.lower(): p for p in (references / 'missile-sources').glob('*.blkx')}
    weapons.update({p.stem.lower(): p for p in (references / 'weapon-sources/containers').glob('*.blkx')})
    presets = {p.stem.lower(): p for p in (references / 'weapon-presets').glob('*.blkx')}
    containers = {}
    if weapon_inputs:
        raw = weapon_inputs.read_bytes()
        containers = json.loads(raw)
        sources[str(weapon_inputs.resolve())] = hashlib.sha256(raw).hexdigest()
    @lru_cache(maxsize=None)
    def resolve(asset):
        seen = set()
        while (asset in weapons or asset in containers) and asset not in seen:
            seen.add(asset); d = read(weapons[asset]) if asset in weapons else containers[asset]
            if d.get('container') is True:
                asset = key(d['blk']); continue
            return asset if asset in profiles else None
        return None
    names = {}
    for filename in ('wiki.json', 'game.json'):
        path = references / 'vehicle-names' / filename
        if path.exists(): names.update(read(path).get('names', {}))
    exclusions = json.loads((ROOT / 'references/aircraft-exclusions.json').read_bytes())
    excluded = set(exclusions.get('vehicle_variants', {})) | set(exclusions.get('aircraft', {}))
    aircraft = {}; carriers = {asset: [] for asset in profiles}; required = set()
    records = read(references / 'prop-vehicles/manifest.json')
    review_path = ROOT / 'references/missile-launcher-audit.json'
    review = json.loads(review_path.read_bytes())
    sources[str(review_path)] = hashlib.sha256(review_path.read_bytes()).hexdigest()
    if review['base_commit'] != records['commit']:
        raise ValueError('Launcher audit must be refreshed for the new base dataset')
    compatibility_updates = {}
    compiled = {}
    if loadouts:
        raw = loadouts.read_bytes()
        payload = json.loads(raw)
        assert payload['commit'] == records['commit']
        sources[str(loadouts.resolve())] = hashlib.sha256(raw).hexdigest()
        compiled = payload['aircraft']
    by_vehicle = {r['vehicle']: r for r in records['records']}
    for path in sorted((references / 'prop-vehicles').glob('*.blkx')):
        if path.stem in excluded or '_killstreak' in path.stem: continue
        d = read(path)
        # Check the actual aircraft data matches its pinned manifest.
        raw = path.read_bytes()
        git_sha = hashlib.sha1(b'blob ' + str(len(raw)).encode() + b'\0' + raw).hexdigest()
        assert git_sha == by_vehicle[path.stem]['sha'], path
        found = set()
        # The pinned loadout compiler resolves nested weapon containers (SRAAM,
        # for example). Retain the exact input location/hash, without copying it.
        for item in compiled.get(path.stem, {}).get('items', []):
            if len(item) > 4 and item[:3] == ['s', 'rocket', 'aam'] and item[4] > 0 and item[3] in profiles:
                found.add(item[3])
        blocks = [d.get('WeaponSlots', {}), d.get('commonWeapons', {})]
        for preset in rows(d.get('weapon_presets', {}).get('preset')):
            pk = key(preset.get('blk', ''))
            if pk in presets: blocks.append(read(presets[pk]))
        for block in blocks:
            for w in mounts(block):
                asset = resolve(key(w['blk']))
                if asset: found.add(asset)
        added = reviewed_assets(path.stem, by_vehicle[path.stem]['sha'], review, resolve)
        found.update(added)
        if not found: continue
        # Follow the existing missile picker’s display-name grouping, including
        # mounting/default aliases which resolve to the same displayed missile.
        compatible = set().union(*(groups[profiles[a]['name'].casefold()] for a in found))
        for asset in compatible: carriers[asset].append(path.stem)
        for asset in added:
            for alias in groups[profiles[asset]['name'].casefold()]:
                compatibility_updates.setdefault(alias, {})[path.stem] = review['aircraft_updates'][path.stem]
        radar_missiles = any(profiles[a]['family'] == 'radar' for a in compatible)
        sensors = [dict(s) for s in rows(d.get('sensors', {}).get('sensor'))
                   if s.get('blk') and not s.get('external') and s.get('dmPart', '').startswith('radar')]
        if radar_missiles: required.update(key(s['blk']) + '.blkx' for s in sensors)
        aircraft[path.stem] = dict(name=names.get(path.stem, path.stem), sensors=sensors,
                                   radar_missiles=radar_missiles, weapon_assets=sorted(found))
    return dict(commit=records['commit'], aircraft=aircraft, carriers=carriers,
                profiles=profiles, needed_sensors=sorted(required), source_hashes=sources,
                checked_commit=review['checked_commit'], availability=review['findings'],
                compatibility_updates=compatibility_updates)


def build(data, sensors):
    aircraft = {}; unknown = []
    for ident, a in data['aircraft'].items():
        radars = []
        for mount in a['sensors']:
            name = key(mount['blk']) + '.blkx'
            sensor = sensors.get(name)
            if not sensor or sensor.get('type') != 'radar': continue
            patterns = sensor.get('scanPatterns') or {}
            pattern = next((p for p in ('radarTrack', 'trackRadar', 'track') if p in patterns), None)
            if pattern:
                p = patterns[pattern]
                az, el = p.get('azimuthLimits'), p.get('elevationLimits')
                if isinstance(az, list) and len(az) == 2 and isinstance(el, list) and len(el) == 2:
                    radars.append(dict(id=name[:-5], name=sensor.get('name', name[:-5]),
                        azimuth_deg=az, elevation_deg=el, pattern=pattern, source_sha=sensor['sha']))
        if a['radar_missiles'] and not radars: unknown.append(ident)
        aircraft[ident] = dict(name=a['name'], radars=radars)
    for updates in data.get('compatibility_updates', {}).values():
        for ident, update in updates.items():
            actual = {r['id']: r['source_sha'] for r in aircraft[ident]['radars']}
            if actual != update['radar_sources']:
                raise ValueError(f'Reviewed launcher update needs refreshed radar data for {ident}')
    return dict(schema=1, source_commit=data['commit'], radar_policy='radar_guided_only_launch_time',
                aircraft=aircraft, carriers=data['carriers'],
                families={k: p['family'] for k, p in data['profiles'].items()},
                checked_commit=data['checked_commit'], availability=data['availability'],
                compatibility_updates=data['compatibility_updates']), unknown


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--references', type=Path, required=True)
    parser.add_argument('--workstream', type=Path, required=True)
    parser.add_argument('--sensors', type=Path)
    parser.add_argument('--loadouts', type=Path)
    parser.add_argument('--weapon-inputs', type=Path)
    args = parser.parse_args()
    from benchmark_output import OutputBudget
    out = args.workstream / 'aircraft-launch'
    budget = OutputBudget(args.workstream, 'aircraft-launch-catalog')
    manifest = dict(owner='/root', status='active', task='Per-missile compatible launcher aircraft and radar launch limits',
        worktree=str(ROOT), source_revision='5205482a8ff5c9d6c445af0afb36b233eaf11b83',
        references=str(args.references.resolve()), budget=dict(summary=1000000, diagnostic=25000000, total=100000000),
        cleanup_condition='Retain compact catalog, source evidence and checks; no copied reference tree')
    assert budget.json(out / 'manifest.json', manifest)
    if args.sensors:
        import gzip
        data = json.loads(gzip.decompress((out / 'inventory.json.gz').read_bytes()))
        result, unknown = build(data, json.loads(args.sensors.read_bytes()))
        if unknown: raise ValueError('Missing radar track limits for: ' + ', '.join(unknown))
        raw = json.dumps(result, separators=(',', ':'), ensure_ascii=False).encode('utf-8')
        assert len(raw) < 1000000
        path = ROOT / 'references/missile-launchers.json'
        path.write_bytes(raw)
        assert budget.json(out / 'catalog-report.json', dict(bytes=len(raw), sha256=hashlib.sha256(raw).hexdigest(),
            path=str(path), aircraft=len(result['aircraft']), unknown_radars=unknown))
        print(json.dumps(dict(catalog_bytes=len(raw), unknown_radars=unknown)))
    else:
        data = inventory(args.references, args.loadouts, args.weapon_inputs)
        assert budget.json(out / 'inventory.json.gz', data, kind='diagnostic')
        assert budget.json(out / 'pilot.json', dict(inventory_bytes=(out/'inventory.json.gz').stat().st_size,
            sensor_count=len(data['needed_sensors']), estimated_sensor_extract_bytes=500000,
            sensor_extract_ceiling_bytes=1000000, full_traces=False))
        print(json.dumps(dict(aircraft=len(data['aircraft']), missiles=sum(bool(v) for v in data['carriers'].values()),
                              needed_sensors=data['needed_sensors'])))
    raise SystemExit(budget.finish())
