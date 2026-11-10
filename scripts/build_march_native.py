"""Build the march chart's compiled loop (scripts/march_native) into .native_march/.

  python scripts/build_march_native.py [--force]

Same conventions as build_em_backend.py: Cython with strict floating point
(/O2 /fp:strict on Windows; -O3 -ffp-contract=off -fno-fast-math elsewhere),
a new library directory per build (a loaded DLL is never overwritten), an
import probe, then an atomic manifest. Docker and the Windows package build it
ahead of time; development checkouts build it on first use. If it cannot be
built the march chart runs its Python loop (same results, slower).
"""
import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import sysconfig
import uuid
from contextlib import contextmanager
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DIRECTORY = ROOT/'.native_march'
COMPILE_ARGS = ['/O2', '/fp:strict'] if os.name == 'nt' else ['-O3', '-ffp-contract=off', '-fno-fast-math']


@contextmanager
def build_lock():
    DIRECTORY.mkdir(exist_ok=True)
    with (DIRECTORY/'.build.lock').open('a+b') as lock:
        if os.name == 'nt':
            import msvcrt, time, errno
            lock.seek(0, 2)
            if lock.tell() == 0:
                lock.write(b'0');lock.flush()
            while True:
                lock.seek(0)
                try:
                    msvcrt.locking(lock.fileno(), msvcrt.LK_NBLCK, 1);break
                except OSError as error:
                    if error.errno not in (errno.EACCES, errno.EAGAIN, errno.EDEADLK):raise
                    time.sleep(.2)
            try:yield
            finally:
                lock.seek(0);msvcrt.locking(lock.fileno(), msvcrt.LK_UNLCK, 1)
        else:
            import fcntl
            fcntl.flock(lock, fcntl.LOCK_EX)
            try:yield
            finally:fcntl.flock(lock, fcntl.LOCK_UN)


def build(force=False):
    sys.path.insert(0, str(ROOT/'scripts'))
    import em_march_native as N
    expected = N.build_signature()
    if not force and N._manifest()[0] is not None:
        print('March native loop is up to date.');return
    import numpy
    from setuptools import setup, Extension
    from Cython.Build import cythonize
    library = 'lib/'+expected[:16]+'-'+uuid.uuid4().hex[:12]
    lib = DIRECTORY/library;lib.mkdir(parents=True)
    work = DIRECTORY/('src-'+expected[:16]);shutil.rmtree(work, ignore_errors=True);work.mkdir(parents=True)
    for path in N.native_sources():shutil.copy2(path, work/path.name)
    here = os.getcwd()
    try:
        os.chdir(work)
        ext = Extension(N.MODULE, [N.MODULE+'.pyx'], include_dirs=[numpy.get_include()], extra_compile_args=COMPILE_ARGS,
                        define_macros=[('NPY_NO_DEPRECATED_API', 'NPY_1_7_API_VERSION')])
        setup(name='wt-march-native', ext_modules=cythonize([ext], compiler_directives={'language_level': 3}, annotate=False),
              script_args=['build_ext', '--build-lib', str(lib), '--build-temp', 'build', '-q'])
    finally:
        os.chdir(here)
    shutil.rmtree(work, ignore_errors=True)
    name = N.MODULE+sysconfig.get_config_var('EXT_SUFFIX')
    probe = subprocess.run([sys.executable, '-c',
        'import importlib.util,sys; s=importlib.util.spec_from_file_location(sys.argv[1],sys.argv[2]); '
        'm=importlib.util.module_from_spec(s); s.loader.exec_module(m); assert hasattr(m,"CurveCorrection")',
        N.MODULE, str(lib/name)], capture_output=True, text=True)
    if probe.returncode:raise RuntimeError('March native import verification failed: '+probe.stderr[-2000:])
    manifest = dict(signature=expected, native_sha256=N.native_digest(), library=library,
                    binaries={name: hashlib.sha256((lib/name).read_bytes()).hexdigest()},
                    compiler='Cython; '+' '.join(COMPILE_ARGS))
    temporary = DIRECTORY/'manifest.tmp'
    temporary.write_text(json.dumps(manifest, indent=1)+'\n')
    temporary.replace(DIRECTORY/'manifest.json')
    print('March native loop ready: '+str(lib/name))


def main():
    ap = argparse.ArgumentParser(description=__doc__);ap.add_argument('--force', action='store_true')
    args = ap.parse_args()
    with build_lock():build(args.force)


if __name__ == '__main__':main()
