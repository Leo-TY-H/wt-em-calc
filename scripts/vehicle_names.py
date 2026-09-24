"""Offline presentation names, independent of solver and comparison identities."""
import json
from functools import lru_cache
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'references/vehicle-names/wiki.json'


@lru_cache(maxsize=1)
def naming_sources():
    wiki = json.loads(SOURCE.read_text(encoding='utf-8'))
    records = json.loads((ROOT / 'references/prop-vehicles/manifest.json').read_text())['records']
    return wiki, records


def apply_names(catalog):
    wiki, records = naming_sources()
    vehicles = {r['vehicle'] for r in records}
    for key, row in catalog.items():
        # A real vehicle always wins over other vehicles sharing its FM. Jets
        # lacking a vehicle record retain ALL mapped identities, not a guessed
        # country/variant. Prop catalog keys already identify exact vehicles.
        ids = [key] if key in vehicles else sorted(r['vehicle'] for r in records
            if row['propulsion'] == 'jet' and Path(r['fm'] or '').stem == key)
        labels = []
        for vehicle in ids or [key]:
            name = wiki['names'].get(vehicle)
            labels.append(dict(vehicle_id=vehicle if ids else None,
                               display_name=name,
                               source=wiki['sources'].get(vehicle)))
        row['vehicle_names'] = labels
        # Hidden/test siblings must not clutter a public vehicle's label. Keep
        # their complete identities in metadata for source inspection.
        visible = [v for v in labels if v['display_name']] or labels
        row['name_verified'] = all(v['display_name'] for v in visible)
        row['name'] = ' / '.join(
            f"{v['display_name'] or row['name']} [{v['vehicle_id'] or key}]"
            + ('' if v['display_name'] else ' (wiki name unavailable)') for v in visible)


def refresh_result_names(data, catalog):
    """Relabel saved chart metadata without recomputing physical samples."""
    for row in data['aircraft']:
        key = row.get('aircraft_id', row['id'])
        if key not in catalog:
            key = key.removesuffix('__instructor_on').removesuffix('__instructor_off')
        if key not in catalog:
            continue
        # Entry number and Instructor suffixes are presentation-specific and
        # remain useful when comparing two copies of the same vehicle.
        suffix = row['name'].partition(' · ')[2]
        row['name'] = catalog[key]['name'] + (' · ' + suffix if suffix else '')
    return data
