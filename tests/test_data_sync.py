import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from sync_game_data import GLOBALS, blob_sha, json_bytes, publish, sync
from prop_catalog import asset_mismatch
import hashlib


class FakeSource:
    def __init__(self):
        self.commit = 'a' * 40
        self.payloads = {
            'references/jet-catalog/fm/test.blkx': json_bytes({'Mass': {'EmptyMass': 100}}),
            'references/prop-vehicles/test.blkx': json_bytes({'fmFile': 'fm/test.blk', 'model': 'test'}),
            **{target: json_bytes({'value': 1}) for target in GLOBALS.values()},
        }
        self.bad_hash = False

    def resolve(self, ref):
        return self.commit, 'test version'

    def inventory(self, commit):
        return {name: dict(path=name, sha=blob_sha(data), type='blob', size=len(data), mode='100644')
                for name, data in self.payloads.items()}

    def download(self, commit, path):
        return b'{}' if self.bad_hash else self.payloads[path]


class DataSyncTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        (self.root / 'references').mkdir()
        (self.root / 'references/data-sync-config.json').write_bytes(json_bytes(
            dict(repository='test/data', ref='master', check_interval_hours=6)))
        self.source = FakeSource()

    def run_sync(self, force=True):
        return sync(self.root, self.source, force=force, workers=2)

    def snapshot(self):
        return {str(p.relative_to(self.root)): p.read_bytes() for p in (self.root / 'references').rglob('*') if p.is_file()}

    def test_refresh_and_noop_are_deterministic(self):
        self.assertTrue(self.run_sync())
        original = self.snapshot()
        self.assertFalse(self.run_sync())
        self.assertEqual(original, self.snapshot())
        manifests = ['jet-catalog/fm-manifest.json', 'prop-vehicles/manifest.json',
                     'aircraft-modifications/manifest.json', 'data-version.json']
        for name in manifests:
            self.assertEqual(json.loads((self.root / 'references' / name).read_bytes())['commit'], self.source.commit)

    def test_hash_failure_leaves_everything_unchanged(self):
        self.run_sync()
        original = self.snapshot()
        self.source.payloads['references/jet-catalog/fm/test.blkx'] = b'{"changed":true}'
        self.source.bad_hash = True
        with self.assertRaisesRegex(ValueError, 'hash mismatch'):
            self.run_sync()
        self.assertEqual(original, self.snapshot())

    def test_local_fm_and_global_edits_are_preserved(self):
        self.run_sync()
        for name in ['references/jet-catalog/fm/test.blkx', 'references/body-gameplay.blkx']:
            with self.subTest(name=name):
                path = self.root / name
                original = path.read_bytes()
                path.write_bytes(b'{"custom":true}')
                with self.assertRaisesRegex(ValueError, 'Local edit preserved'):
                    self.run_sync()
                self.assertEqual(path.read_bytes(), b'{"custom":true}')
                path.write_bytes(original)

    def test_upstream_removals_and_additions(self):
        self.run_sync()
        old = 'references/jet-catalog/fm/test.blkx'
        self.source.payloads['references/jet-catalog/fm/new.blkx'] = self.source.payloads.pop(old)
        self.assertTrue(self.run_sync())
        self.assertFalse((self.root / old).exists())
        self.assertTrue((self.root / 'references/jet-catalog/fm/new.blkx').exists())

    def test_download_failure_is_transactional(self):
        original = self.snapshot()
        with patch.object(self.source, 'download', side_effect=TimeoutError('offline')):
            with self.assertRaises(TimeoutError):
                self.run_sync()
        self.assertEqual(original, self.snapshot())
        self.assertFalse((self.root / '.data-sync/lock').exists())

    def test_recent_check_skips_network(self):
        self.run_sync()
        with patch.object(self.source, 'resolve', side_effect=AssertionError('Network should not be used')):
            self.assertFalse(self.run_sync(force=False))

    def test_publish_failure_rolls_back(self):
        stage = self.root / 'stage'
        stage.mkdir()
        for name in ('a', 'b'):
            (self.root / name).write_text('old')
            (stage / name).write_text('new')
        replace = Path.replace
        def fail_second(path, target):
            if path == stage / 'b':
                raise OSError('simulated write failure')
            return replace(path, target)
        with patch.object(Path, 'replace', fail_second):
            with self.assertRaises(OSError):
                publish(self.root, stage, ['a', 'b'], [])
        for name in ('a', 'b'):
            self.assertEqual((self.root / name).read_text(), 'old')

    def test_stale_prop_assets_are_unavailable(self):
        raw = b'{"fm":1}'
        asset = dict(fm_sha256=hashlib.sha256(raw).hexdigest(), binary_sha256='binary')
        self.assertIsNone(asset_mismatch(raw, asset, asset))
        self.assertIn('regenerated', asset_mismatch(b'{}', asset, asset))
        mass = dict(asset, binary_sha256='different')
        self.assertIn('executable', asset_mismatch(raw, asset, mass))


if __name__ == '__main__':
    unittest.main()
