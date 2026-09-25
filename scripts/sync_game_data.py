"""Refresh runtime aircraft data from one hash-verified datamine commit.

Uses only the standard library, both locally and in GitHub Actions. Downloads
are staged before publication; local edits cause an error rather than data loss.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import tempfile
import time
from urllib.error import HTTPError, URLError
from urllib.parse import quote
from urllib.request import Request, urlopen

ROOT = Path(__file__).resolve().parents[1]
FLIGHT = 'aces.vromfs.bin_u/gamedata/flightmodels'
GLOBALS = {
    'aces.vromfs.bin_u/config/gameparams.blkx': 'references/body-gameparams.blkx',
    'aces.vromfs.bin_u/config/gameplay.blkx': 'references/body-gameplay.blkx',
    'char.vromfs.bin_u/config/flightmodels.blkx': 'references/body-flightmodels-config.blkx',
    'char.vromfs.bin_u/config/modifications.blkx': 'references/aircraft-modifications/modifications.blkx',
}


def read_json(path):
    return json.loads(path.read_text(encoding='utf-8'))


def json_bytes(value):
    return (json.dumps(value, ensure_ascii=False, indent=2) + '\n').encode('utf-8')


def blob_sha(content):
    return hashlib.sha1(f'blob {len(content)}\0'.encode() + content).hexdigest()


class GitHubSource:
    def __init__(self, repository):
        if not re.fullmatch(r'[\w.-]+/[\w.-]+', repository):
            raise ValueError('Invalid source repository')
        self.repository = repository
        self.api = 'https://api.github.com/repos/' + repository
        self.trees = {}

    def request(self, url):
        headers = {'User-Agent': 'WT-FM-data-sync'}
        token = os.environ.get('GITHUB_TOKEN')
        if token and url.startswith('https://api.github.com/'):
            headers['Authorization'] = 'Bearer ' + token
        for attempt in range(4):
            try:
                with urlopen(Request(url, headers=headers), timeout=60) as response:
                    return response.read()
            except HTTPError as error:
                if error.code not in (429, 500, 502, 503, 504) or attempt == 3:
                    raise
            except (URLError, TimeoutError):
                if attempt == 3:
                    raise
            time.sleep(2 ** attempt)

    def resolve(self, ref):
        commit = json.loads(self.request(self.api + '/commits/' + quote(ref, safe='')))
        sha = commit['sha']
        if not re.fullmatch(r'[0-9a-f]{40}', sha):
            raise ValueError('Invalid upstream commit')
        return sha, commit['commit']['message'].splitlines()[0]

    def tree(self, sha, recursive=False):
        key = sha, recursive
        if key not in self.trees:
            data = json.loads(self.request(self.api + '/git/trees/' + sha + ('?recursive=1' if recursive else '')))
            if data.get('truncated'):
                raise ValueError('Upstream tree is truncated; refusing a partial update')
            self.trees[key] = data['tree']
        return self.trees[key]

    def entry(self, commit, path):
        sha = commit
        for part in path.split('/'):
            row = next((r for r in self.tree(sha) if r['path'] == part), None)
            if row is None:
                raise ValueError('Required upstream path is missing: ' + path)
            sha = row['sha']
        return dict(row, path=path)

    def inventory(self, commit):
        tree = self.tree(self.entry(commit, FLIGHT)['sha'], recursive=True)
        files = {}
        for row in tree:
            path = PurePosixPath(row['path'])
            if row['type'] != 'blob' or path.suffix != '.blkx':
                continue
            if '..' in path.parts or '\\' in str(path) or path.is_absolute():
                raise ValueError('Unsafe upstream path')
            if len(path.parts) == 1:
                target = 'references/prop-vehicles/' + path.name
            elif len(path.parts) == 2 and path.parts[0] == 'fm':
                target = 'references/jet-catalog/fm/' + path.name
            else:
                continue
            files[target] = dict(row, path=FLIGHT + '/' + str(path))
        if not any('/fm/' in p for p in files) or not any('/prop-vehicles/' in p for p in files):
            raise ValueError('Upstream aircraft inventory is empty')
        for source, target in GLOBALS.items():
            files[target] = self.entry(commit, source)
        return files

    def download(self, commit, path):
        return self.request(f'https://raw.githubusercontent.com/{self.repository}/{commit}/{quote(path)}')


@contextmanager
def sync_lock(root):
    work = root / '.data-sync'
    work.mkdir(exist_ok=True)
    lock = work / 'lock'
    try:
        lock.mkdir()
    except FileExistsError:
        raise RuntimeError('Another data update is running. If it crashed, remove .data-sync/lock after checking.')
    try:
        yield work
    finally:
        lock.rmdir()


def previous_hashes(root):
    version = root / 'references/data-version.json'
    if version.exists():
        return read_json(version)['files']
    # Bootstrap the transferred project's existing source manifests.
    result = {}
    fm = root / 'references/jet-catalog/fm-manifest.json'
    if fm.exists():
        result.update({'references/jet-catalog/fm/' + PurePosixPath(r['path']).name: r['sha']
                       for r in read_json(fm)['files']})
    vehicles = root / 'references/prop-vehicles/manifest.json'
    if vehicles.exists():
        result.update({'references/prop-vehicles/' + r['vehicle'] + '.blkx': r['sha']
                       for r in read_json(vehicles)['records']})
    return result


def check_local_edits(root, old, new):
    for name in set(old) | set(new):
        path = root / name
        if not path.exists():
            continue
        current = blob_sha(path.read_bytes())
        if name in old and current not in (old[name], new.get(name)):
            raise ValueError(f'Local edit preserved: {name}. Move your override aside before updating upstream data.')
        if name not in old and name in new and current != new[name] and ('/fm/' in name or '/prop-vehicles/' in name):
            raise ValueError('Untracked local source preserved: ' + name)


def publish(root, stage, names, removed):
    """Publish atomically even when staging and references use different mounts."""
    def replace_from(source, target):
        target.parent.mkdir(parents=True, exist_ok=True)
        fd, name = tempfile.mkstemp(prefix='.' + target.name + '.sync-', dir=target.parent)
        os.close(fd)
        temporary = Path(name)
        try:
            shutil.copy2(source, temporary)
            temporary.replace(target)
        finally:
            temporary.unlink(missing_ok=True)

    backups = stage / '_backup'
    completed = []
    try:
        for name in sorted(set(names) | set(removed)):
            target, source = root / name, stage / name
            if source.exists() and target.exists() and source.read_bytes() == target.read_bytes():
                continue
            backup = backups / name
            existed = target.exists()
            if existed:
                backup.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(target, backup)
            completed.append((target, backup, existed))
            if source.exists():
                replace_from(source, target)
            elif existed:
                target.unlink()
    except BaseException:
        for target, backup, existed in reversed(completed):
            if existed:
                replace_from(backup, target)
            elif target.exists():
                target.unlink()
        raise


def sync(root=ROOT, source=None, force=False, workers=8):
    config = read_json(root / 'references/data-sync-config.json')
    source = source or GitHubSource(config['repository'])
    with sync_lock(root) as work:
        stamp = work / 'last-check.json'
        if not force and stamp.exists():
            cached = read_json(stamp)
            if cached.get('config') == config and time.time() - cached['time'] < config['check_interval_hours'] * 3600:
                print('Game data checked recently; using local data.')
                return False
        commit, version = source.resolve(config['ref'])
        print(f'Checking datamine {version} ({commit[:12]})', flush=True)
        inventory = source.inventory(commit)
        hashes = {name: row['sha'] for name, row in inventory.items()}
        old = previous_hashes(root)
        check_local_edits(root, old, hashes)
        removed = set(old) - set(hashes)
        with tempfile.TemporaryDirectory(prefix='stage-', dir=work) as temp:
            stage = Path(temp)
            def fetch(item):
                name, row = item
                local = root / name
                data = local.read_bytes() if local.exists() else b''
                if blob_sha(data) != row['sha']:
                    data = source.download(commit, row['path'])
                if blob_sha(data) != row['sha']:
                    raise ValueError('Source hash mismatch: ' + name)
                if not isinstance(json.loads(data), dict):
                    raise ValueError('Expected a game data object: ' + name)
                target = stage / name
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(data)
            with ThreadPoolExecutor(max_workers=workers) as pool:
                for count, _ in enumerate(pool.map(fetch, inventory.items()), 1):
                    if count % 500 == 0:
                        print(f'Verified {count}/{len(inventory)} source files', flush=True)
            records = []
            fm_rows = []
            for name, row in sorted(inventory.items()):
                if '/prop-vehicles/' in name:
                    value = read_json(stage / name)
                    records.append(dict(vehicle=Path(name).stem, sha=row['sha'], fm=value.get('fmFile'), model=value.get('model')))
                elif '/fm/' in name:
                    fm_rows.append(row)
            modifications = 'references/aircraft-modifications/modifications.blkx'
            version_data = dict(schema=1, repository=config['repository'], ref=config['ref'], commit=commit,
                                version=version, files=dict(sorted(hashes.items())))
            generated = {
                'references/jet-catalog/fm-manifest.json': dict(commit=commit, files=fm_rows),
                'references/prop-vehicles/manifest.json': dict(commit=commit, records=records),
                'references/aircraft-modifications/manifest.json': dict(commit=commit,
                    path=inventory[modifications]['path'],
                    url=f"https://raw.githubusercontent.com/{config['repository']}/{commit}/{inventory[modifications]['path']}",
                    sha256=hashlib.sha256((stage / modifications).read_bytes()).hexdigest()),
                'references/data-version.json': version_data,
            }
            for name, value in generated.items():
                target = stage / name
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(json_bytes(value))
            names = set(inventory) | set(generated)
            changed = bool(removed) or any(not (root / p).exists() or (root / p).read_bytes() != (stage / p).read_bytes() for p in names)
            # Check again so edits made during downloads are not overwritten.
            check_local_edits(root, old, hashes)
            publish(root, stage, names, removed)
        stamp.write_bytes(json_bytes(dict(time=time.time(), config=config)))
        print(f'{"Updated" if changed else "Already current"}: {len(fm_rows)} FMs, {len(records)} vehicles; {version}.')
        return changed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--force', action='store_true', help='Check now, ignoring the six-hour check cache')
    parser.add_argument('--offline-ok', action='store_true', help='Launch with cached data only when the network is unavailable')
    args = parser.parse_args()
    from bootstrap_runtime import ensure
    ensure(ROOT)
    try:
        sync(force=args.force)
    except (HTTPError, URLError, TimeoutError) as error:
        if args.offline_ok and (ROOT / 'references/jet-catalog/fm-manifest.json').exists():
            print(f'Network unavailable; using existing data. Update error: {error}')
        else:
            raise


if __name__ == '__main__':
    main()
