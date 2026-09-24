"""Pin a private code/backend snapshot while another task edits the EM model."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def snapshot():
    # Read a stable set before publishing it. Never edit or rebuild the
    # workspace's compiled backend, even if another task has invalidated it.
    for _ in range(3):
        paths = sorted((ROOT/'scripts').glob('*.py'))
        content = {p.name:p.read_bytes() for p in paths}
        if all(p.read_bytes() == content[p.name] for p in paths):
            break
    else:
        raise RuntimeError('Equation files are changing continuously. Retry the altitude launcher.')
    digest = hashlib.sha256()
    for name, data in sorted(content.items()):
        digest.update(name.encode()); digest.update(data)
    parent = ROOT/'outputs/altitude-runtime'
    parent.mkdir(parents=True, exist_ok=True)
    destination = parent/digest.hexdigest()[:20]
    if not destination.exists():
        with tempfile.TemporaryDirectory(dir=parent) as temporary:
            stage = Path(temporary)/'snapshot';stage.mkdir()
            (stage/'scripts').mkdir()
            for name, data in content.items():
                (stage/'scripts'/name).write_bytes(data)
            for name in ('references','app'):
                (stage/name).symlink_to(ROOT/name, target_is_directory=True)
            for name in ('requirements.txt','requirements-plotter.txt'):
                shutil.copy2(ROOT/name, stage/name)
            if (ROOT/'.native_em/lib').is_dir():
                # Copy binaries, not a shared build directory. A stale copy
                # is rebuilt only inside this private snapshot at startup.
                (stage/'.native_em').mkdir()
                shutil.copytree(ROOT/'.native_em/lib', stage/'.native_em/lib')
                shutil.copy2(ROOT/'.native_em/manifest.json', stage/'.native_em/manifest.json')
            (stage/'snapshot.json').write_text(json.dumps(dict(source=str(ROOT),revision=digest.hexdigest())))
            try: stage.rename(destination)
            except FileExistsError: pass
    return destination


if __name__=='__main__':
    os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
    os.environ.setdefault('VECLIB_MAXIMUM_THREADS','1')
    path = ROOT if os.name == 'nt' else snapshot()
    os.environ.setdefault('WT_ALTITUDE_OUTPUT_DIR',str(ROOT/'outputs/altitude'))
    os.execv(sys.executable,[sys.executable,str(path/'scripts/altitude_server.py'),*sys.argv[1:]])
