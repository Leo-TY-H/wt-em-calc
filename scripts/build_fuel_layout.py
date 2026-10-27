"""Pin collision fuel tank parts for vehicles without a game-produced prop-mass asset.

Needs an installed War Thunder (and the zstandard package). Reads the collision
resources of the installed aircraft packs, checks fuel_layout.layout against
every game-produced asset whose FM and collision stream are unchanged, then
writes references/collision-fuel-tanks.json. Nothing is written if any check
disagrees.
"""
import argparse
import hashlib
import json
import re
import struct
from pathlib import Path
from fm_loader import normalize
from fuel_layout import TANK_PARTS, layout
import prop_catalog
from sync_game_data import ROOT, json_bytes

COLLISION = 0xace50000


class Collisions:
    def __init__(self, game):
        self.entries, self.packs = {}, {}
        for path in sorted((game/'content/base/res/aircrafts').glob('*.grp')):
            data = path.read_bytes()
            if data[:4] != b'GRP2':
                continue
            names_at, name_count = struct.unpack_from('<2I', data, 16)
            rows_at, row_count = struct.unpack_from('<2I', data, 32)
            names = []
            for i in range(name_count):
                offset = struct.unpack_from('<I', data, names_at+4*i)[0]
                names.append(data[offset:data.index(0, offset)].decode())
            rows = [struct.unpack_from('<IIHH', data, rows_at+12*i) for i in range(row_count)]
            starts = sorted({r[1] for r in rows} | {len(data)})
            for cls, start, index, _ in rows:
                if cls == COLLISION:
                    end = next(s for s in starts if s > start)
                    self.entries.setdefault(names[index], []).append((path, start, end))
                    self.packs[path] = data

    def stream(self, resource):
        import zstandard
        found = set()
        for path, start, end in self.entries.get(resource, ()):
            packed = self.packs[path][start:end]
            magic, tag = struct.unpack_from('<II', packed)
            if magic != 0xace50003 or tag >> 30 != 1:
                raise ValueError('Unknown collision format: '+resource)
            found.add((path.name, zstandard.ZstdDecompressor().decompress(packed[8:8+(tag & 0x3fffffff)])))
        if len({s for _, s in found}) > 1:
            raise ValueError('Installed packs disagree on '+resource)
        return min(found) if found else None


def tank_parts(stream):
    """tankN_dm numbers in node order (collResStream.h version 3: 108-byte header,
    8-byte bind and 48-byte node records, names and material names last)."""
    counts = struct.unpack_from('<11I', stream, 64)
    nodes, names_size, materials_size = counts[0], counts[9], counts[10]
    end = len(stream)-materials_size
    names = stream[end-names_size:end]
    offsets = [struct.unpack_from('<I', stream, 108+8*nodes+48*i)[0] for i in range(nodes)]
    if not names.endswith(b'\0') or not stream.endswith(b'\0') or \
            any(o >= names_size or o and names[o-1] for o in offsets):
        raise ValueError('Unrecognized collision stream layout')
    parts = []
    for o in offsets:
        match = re.fullmatch(rb'tank(\d+)_dm', names[o:names.index(0, o)])
        if match and int(match[1]) not in parts:
            parts.append(int(match[1]))
    return parts


def fm(name):
    raw = prop_catalog.fm_source(name).read_bytes()
    return hashlib.sha256(raw).hexdigest(), normalize(json.loads(raw))


def check(collisions):
    matched, refused, skipped = 0, [], []
    for path in sorted((ROOT/'references/prop-mass').glob('*--*.json')):
        asset = json.loads(path.read_text())
        sha, model = fm(asset['aircraft'])
        found = collisions.stream(asset['resource'])
        if sha != asset['fm_sha256'] or not found or \
                hashlib.sha256(found[1]).hexdigest() != asset['collision_provenance']['stream_sha256']:
            skipped.append(path.stem)
            continue
        try:
            tanks, systems = layout(model, tank_parts(found[1]))
        except ValueError:
            refused.append(path.stem)
            continue
        if tanks != asset['tanks'] or systems != asset['fuel_systems']:
            raise SystemExit('Fuel layout rule disagrees with game-produced '+path.name)
        matched += 1
    return dict(matched=matched, refused=refused, skipped=skipped)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--game', type=Path, required=True, help='War Thunder installation folder')
    args = parser.parse_args()
    collisions = Collisions(args.game)
    result = check(collisions)
    resources, unavailable = {}, set()
    for row in prop_catalog.catalog().values():
        if prop_catalog.mass_path(row['fm_id'], row['resource']).exists() or row['resource'] in resources:
            continue
        found = collisions.stream(row['resource'])
        if not found:
            unavailable.add(row['resource'])
            continue
        resources[row['resource']] = dict(tank_parts=tank_parts(found[1]), pack=found[0],
                                          stream_sha256=hashlib.sha256(found[1]).hexdigest())
    version = args.game/'content/pkg_main.ver'
    TANK_PARTS.write_bytes(json_bytes(dict(
        schema=1, game_version=version.read_text().strip() if version.exists() else None,
        scope='tankN_dm part numbers, in node order, of installed collision models without a game-produced prop-mass asset. '
              'fuel_layout.layout reproduced every unchanged game-produced asset it did not refuse.',
        check=result, unavailable=sorted(unavailable), resources=dict(sorted(resources.items())))))
    print(json.dumps(dict(matched=result['matched'], refused=len(result['refused']), skipped=len(result['skipped']),
                          pinned=sorted(resources), unavailable=len(unavailable))))


if __name__ == '__main__':
    main()
