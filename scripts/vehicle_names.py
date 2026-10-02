import json
from functools import lru_cache
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'references/vehicle-names/wiki.json'
GAME_SOURCE = ROOT / 'references/vehicle-names/game.json'


@lru_cache(maxsize=1)
def naming_sources():
    names = json.loads(SOURCE.read_text(encoding='utf-8')) if SOURCE.exists() else dict(names={}, sources={})
    if GAME_SOURCE.exists():
        game = json.loads(GAME_SOURCE.read_text(encoding='utf-8'))
        names = dict(names={**names['names'], **game['names']},
                    sources={**names.get('sources', {}), **game.get('sources', {})})
        for key in game.get('fallback_ids', []):
            names['sources'].pop(key, None)
    records = json.loads((ROOT / 'references/prop-vehicles/manifest.json').read_text())['records']
    return names, records


def apply_names(catalog):
    names, records = naming_sources()
    vehicles = {r['vehicle'] for r in records}
    for key, row in catalog.items():
        ids = [key] if key in vehicles else sorted(r['vehicle'] for r in records
            if row['propulsion'] in ('jet','rocket') and Path(r['fm'] or '').stem == key)
        labels = []
        for vehicle in ids or [key]:
            name = names['names'].get(vehicle)
            labels.append(dict(vehicle_id=vehicle if ids else None,
                               display_name=name,
                               source=names['sources'].get(vehicle)))

        visible = [v for v in labels if v['display_name']]
        if not visible:
            # A newly added FM/vehicle must not depend on a manual Wiki edit.
            visible = [dict(vehicle_id=vehicle if ids else None,
                            display_name=vehicle, source=None) for vehicle in ids or [key]]
        row['vehicle_names'] = visible
        row['name_verified'] = all(v['source'] is not None for v in visible)
        row['name'] = ' / '.join(
            f"{v['display_name']} [{v['vehicle_id'] or key}]" for v in visible)


def source_paths():
    return [path for path in (SOURCE, GAME_SOURCE) if path.exists()]


def refresh_result_names(data, catalog):
    for row in data['aircraft']:
        key = row.get('aircraft_id', row['id'])
        if key not in catalog:
            key = key.removesuffix('__instructor_on').removesuffix('__instructor_off')
        if key not in catalog:
            continue

        suffix = row['name'].partition(' · ')[2]
        row['name'] = catalog[key]['name'] + (' · ' + suffix if suffix else '')
    return data
