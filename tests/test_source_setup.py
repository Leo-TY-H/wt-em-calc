import hashlib
import io
from pathlib import Path
import sys
import tempfile
import unittest
import zipfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from bootstrap_runtime import install_archive
from repository_policy import is_source


class SourceSetupTests(unittest.TestCase):
    def archive(self, entries):
        stream = io.BytesIO()
        with zipfile.ZipFile(stream, 'w') as archive:
            for name, content in entries.items():
                archive.writestr(name, content)
        return stream.getvalue()

    def test_only_missing_runtime_assets_installed(self):
        content = self.archive({'repo/references/data-version.json': 'old',
                                'repo/references/jet-catalog/fm/test.blkx': 'fm',
                                'repo/README.md': 'not runtime',
                                'repo/scripts/example.py': 'not runtime'})
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'references').mkdir()
            (root / 'references/data-version.json').write_text('new')
            count = install_archive(content, root, hashlib.sha256(content).hexdigest())
            self.assertEqual(count, 1)
            self.assertEqual((root / 'references/data-version.json').read_text(), 'new')
            self.assertEqual((root / 'references/jet-catalog/fm/test.blkx').read_text(), 'fm')
            self.assertFalse((root / 'README.md').exists())
            self.assertFalse((root / 'scripts').exists())

    def test_checksum_failure_writes_nothing(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, 'checksum'):
                install_archive(b'corrupt', directory)
            self.assertEqual(list(Path(directory).iterdir()), [])

    def test_traversal_rejected_before_writing(self):
        content = self.archive({'repo/references/data-version.json': 'data',
                                'repo/../escaped': 'bad'})
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, 'Unsafe'):
                install_archive(content, directory, hashlib.sha256(content).hexdigest())
            self.assertEqual(list(Path(directory).iterdir()), [])

    def test_source_policy(self):
        for name in ('scripts/em_solver.py', 'tests/test_source_setup.py', 'app/app.js',
                     'app/vendor/plotly.min.js', '.github/workflows/sync-data.yml',
                     'Setup Windows.cmd', 'requirements-plotter.txt', 'Dockerfile'):
            self.assertTrue(is_source(name), name)
        for name in ('README.md', 'research/findings.py', 'references/model.json',
                     'app/api/meta', 'app/fonts/font.ttf', 'scripts/__pycache__/x.pyc',
                     'Launch EM Plotter.command'):
            self.assertFalse(is_source(name), name)


if __name__ == '__main__':
    unittest.main()
