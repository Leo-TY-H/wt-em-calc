"""Authored input discovery and localization; no simulation approximations.

Every file in an update is read at the same upstream commit. Rocket inputs
remain separate from compiled flight profiles: downloading new source data
must never label an old profile as current.
"""
import csv
import io
import json
from pathlib import Path

ROCKETS = 'aces.vromfs.bin_u/gamedata/weapons/rocketguns'
LOCALIZATION = {
    'lang.vromfs.bin_u/lang/units.csv': 'references/localization/units.csv',
    'lang.vromfs.bin_u/lang/units_weaponry.csv': 'references/localization/units_weaponry.csv',
}


def localized_names(content):
    """Read the authored English column, including quoted semicolons/newlines."""
    rows = csv.reader(io.StringIO(content.decode('utf-8-sig'), newline=''),
                      delimiter=';', strict=True)
    header = next(rows, [])
    if not header or header[0] != '<ID|readonly|noverify>' or '<English>' not in header:
        raise ValueError('Game localization is missing the ID/English header')
    english = header.index('<English>')
    names = {}
    for row in rows:
        if not row or not any(row):
            continue
        if len(row) <= english:
            raise ValueError('Incomplete game localization row')
        key, value = row[0], row[english].strip()
        if key and value:
            if key in names and names[key] != value:
                raise ValueError('Conflicting game localization ID: ' + key)
            names[key] = value
    if not names:
        raise ValueError('Game localization is empty')
    return names


def validate_source(name, content):
    if name in LOCALIZATION.values():
        localized_names(content)
    elif not isinstance(json.loads(content), dict):
        raise ValueError('Expected a game data object: ' + name)


def generated_inputs(stage, inventory, records, repository, commit, version):
    """Derive display catalogs and a raw missile inventory from staged inputs."""
    generated = {}
    provenance = dict(repository=repository, commit=commit, version=version)
    for source, destination in LOCALIZATION.items():
        if destination not in inventory:
            continue
        labels = localized_names((stage / destination).read_bytes())
        url = f'https://github.com/{repository}/blob/{commit}/{source}'
        if destination.endswith('/units.csv'):
            names = {}
            for record in records:
                key = record['vehicle']
                # Keep the stable game ID visible if localization trails a new
                # aircraft. Never remove a valid aircraft for a missing label.
                names[key] = next((labels[k] for k in (key + '_shop', key + '_0', key)
                                   if k in labels), key)
            generated['references/vehicle-names/game.json'] = dict(
                provenance, names=names,
                sources={key: url for key in names if names[key] != key},
                fallback_ids=sorted(key for key in names if names[key] == key))
        else:
            generated['references/missile-names.json'] = dict(provenance, source=url, labels=labels)

    missiles = []
    for name, row in sorted(inventory.items()):
        if not name.startswith('references/missile-sources/'):
            continue
        document = json.loads((stage / name).read_bytes())
        rockets = document.get('rocket', {})
        rockets = rockets if isinstance(rockets, list) else [rockets]
        if any(isinstance(rocket, dict) and rocket.get('bulletType') == 'aam' for rocket in rockets):
            missiles.append(dict(id=Path(name).stem, path=Path(name).name, sha=row['sha'],
                                 source=row['path']))
    if any(name.startswith('references/missile-sources/') for name in inventory):
        if not missiles:
            raise ValueError('Upstream air-to-air missile inventory is empty')
        generated['references/missile-sources/manifest.json'] = dict(
            provenance, schema=1, missiles=missiles, kind='authored_source_inputs')
    if any(name.startswith('references/weapon-sources/') for name in inventory):
        from aircraft_ammunition import generate
        generated['references/aircraft-ammunition.json'] = dict(
            provenance, schema=2, aircraft=generate(stage, records))
    return generated
