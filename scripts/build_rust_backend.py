"""Build native Rust kernels with Cargo; no Python headers/link library needed."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

from rust_backend import ROOT, DIRECTORY, signature


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cargo', default=shutil.which('cargo') or str(Path.home()/'.cargo/bin/cargo'))
    args = parser.parse_args()
    subprocess.run([args.cargo, 'build', '--release', '--locked',
                    '--manifest-path', str(ROOT/'native/Cargo.toml')], check=True)
    name = ('wt_numeric.dll' if os.name == 'nt' else
            'libwt_numeric.dylib' if sys.platform == 'darwin' else 'libwt_numeric.so')
    source = ROOT/'native/target/release'/name
    DIRECTORY.mkdir(exist_ok=True)
    shutil.copy2(source, DIRECTORY/name)
    manifest = dict(signature=signature(), binary=name,
                    sha256=hashlib.sha256((DIRECTORY/name).read_bytes()).hexdigest())
    temporary = DIRECTORY/'manifest.tmp'
    temporary.write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    temporary.replace(DIRECTORY/'manifest.json')
    print('Built Rust numeric kernels:', DIRECTORY/name)


if __name__ == '__main__':
    main()
