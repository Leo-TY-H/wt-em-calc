"""Regenerate scoped physical profile properties from embedded authored rockets.

No game executable, research directory or network access is needed. Run --check
to inspect freshness, or --write to update the packaged profiles, index and
their manifest hashes. Existing unrelated manifest entries are preserved.
"""
import argparse
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import stat
import tempfile

from missile_profile_loading import regenerate_profile, digest, SCHEMA, NATIVE_BUILD, NATIVE_SHA256

ROOT = Path(__file__).resolve().parents[1]
MAX_PROFILE_BYTES = 1_000_000


def encoded(value):
    # Packaged profiles use CRLF and no terminal newline. Keep that stable even
    # when the generator runs on another platform so hashes and diffs agree.
    return json.dumps(value, indent=2, ensure_ascii=True, allow_nan=False).replace('\n', '\r\n').encode()


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def regular_file(path):
    info = path.lstat()
    if (path.is_symlink() or not stat.S_ISREG(info.st_mode)
            or getattr(info, 'st_file_attributes', 0) & getattr(stat, 'FILE_ATTRIBUTE_REPARSE_POINT', 1024)):
        raise ValueError('Expected an ordinary owned file: ' + str(path))


def plan(model, globals_):
    """Validate every input before changing any file; retain only bounded payloads."""
    model = Path(model)
    if model.is_symlink() or (model / 'launch-profiles').is_symlink():
        raise ValueError('Profile generation does not write through directory links')
    index_path = model / 'launch-profiles/index.json'
    manifest_path = model / 'manifest.json'
    regular_file(index_path)
    regular_file(manifest_path)
    index = json.loads(index_path.read_bytes())
    manifest = json.loads(manifest_path.read_bytes())
    changed = {}
    records = []
    seen = set()
    for row in index['profiles']:
        name = row['path']
        if name != Path(name).name or not name.endswith('.json') or name in seen:
            raise ValueError('Invalid or duplicate indexed profile path: ' + name)
        seen.add(name)
        path = model / 'launch-profiles' / name
        regular_file(path)
        before = path.read_bytes()
        if len(before) > MAX_PROFILE_BYTES:
            raise ValueError('Profile exceeds the finite generator input limit: ' + name)
        old = json.loads(before)
        if row['asset'] != old['asset'] or row['properties_sha256'] != old['properties_sha256']:
            raise ValueError('Index and profile disagree: ' + name)
        new = regenerate_profile(old, globals_)
        after = encoded(new)
        if len(after) > MAX_PROFILE_BYTES:
            raise ValueError('Generated profile exceeds the finite output limit: ' + name)
        if before != after:
            changed[path] = after
        row['properties_sha256'] = new['properties_sha256']
        manifest['sources']['launch-profiles/' + name] = sha(after)
        records.append(dict(asset=old['asset'],changed=before != after,
            before_sha256=sha(before),after_sha256=sha(after),bytes=len(after),
            rocket_sha256=digest(old['properties']['rocket']),
            property_changes=[key for key in ('body', 'precision', 'guidance')
                              if old['properties'][key] != new['properties'][key]]))
    provenance = dict(schema=SCHEMA, validated_build=NATIVE_BUILD, native_binary_sha256=NATIVE_SHA256,
        globals_sha256=digest(globals_), generator='scripts/regenerate_missile_profiles.py',
        generator_sha256=sha(Path(__file__).read_bytes()),
        loader_sha256=sha((Path(__file__).parent / 'missile_profile_loading.py').read_bytes()),
        scope='Outer rocket fin-cap scaling, authored precision, flat numeric CyMachMult loading; existing core and constructor histories retain their original provenance.')
    index['property_loading'] = deepcopy(provenance)
    index_raw = encoded(index)
    if index_path.read_bytes() != index_raw:
        changed[index_path] = index_raw
    manifest['sources']['launch-profiles/index.json'] = sha(index_raw)
    manifest['property_loading'] = provenance
    manifest_raw = encoded(manifest)
    if manifest_path.read_bytes() != manifest_raw:
        changed[manifest_path] = manifest_raw
    return changed, records


def atomic_replace(path, raw):
    """One owned temporary file; no baseline/profile-directory copies."""
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=path.parent, prefix=path.name + '.', suffix='.tmp', delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(raw)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument('--check', action='store_true')
    action.add_argument('--write', action='store_true')
    parser.add_argument('--model', type=Path, default=ROOT / 'scripts/missile_model')
    parser.add_argument('--gameparams', type=Path, default=ROOT / 'references/body-gameparams.blkx')
    parser.add_argument('--report', type=Path, help='Optional bounded summary under --output-budget-root')
    from benchmark_output import add_arguments, OutputBudget, DEFAULTS
    add_arguments(parser)
    args = parser.parse_args()
    globals_ = json.loads(args.gameparams.read_bytes())['shellBallisticsParams']
    changed, records = plan(args.model, globals_)
    if args.write:
        # All inputs and replacement payloads have already passed validation.
        # Profiles precede index/manifest; their digests reject partial updates.
        for path, raw in changed.items():
            atomic_replace(path, raw)
    summary = dict(schema=SCHEMA, action='write' if args.write else 'check', profiles=len(records),
        changed_profiles=sum(row['changed'] for row in records), changed_files=len(changed),
        planned_bytes=sum(map(len, changed.values())), records=records,
        native_build=NATIVE_BUILD, native_binary_sha256=NATIVE_SHA256,
        globals_source=str(args.gameparams), globals_source_sha256=sha(args.gameparams.read_bytes()),
        generator_sha256=sha(Path(__file__).read_bytes()),
        loader_sha256=sha((Path(__file__).parent / 'missile_profile_loading.py').read_bytes()))
    status = 0
    if args.report:
        limits = dict(DEFAULTS)
        for kind in limits:
            value = getattr(args, 'output_' + kind + '_mb')
            if value is not None:
                limits[kind] = int(value * 1_000_000)
        budget = OutputBudget(args.output_budget_root or args.report.parent,
                              'implementation-properties', limits, args.output_budget_reason)
        budget.json(args.report, summary)
        status = budget.finish()
    print(json.dumps({key: value for key, value in summary.items() if key != 'records'}))
    return status or (1 if args.check and changed else 0)


if __name__ == '__main__':
    raise SystemExit(main())
