"""Select compiled, equation-identical modules before importing the FM ports."""
import hashlib
import json
import os
import subprocess
import sys
import platform
import time
from contextlib import contextmanager
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
MODULES=['component_assembly','component_stages','control_mixer','polar_f32','mach_cubic','polar_runtime',
         'air_state','wing_stages','wing_model','downwash','tail_model','body_dynamics',
         'primary_controls','jet_model','mass_model','kinematics','engine_supply',
         'structural_limits','jet_nozzle','wing_sweep','aero_helpers','aircraft_model',
         'instructor_reduced','instructor_protection','instructor_pitch_predictor',
         'instructor_autotrim','instructor_predictor_inputs','instructor_keyboard','instructor_settle',
         'piston_model','piston_compressor','piston_general','propeller_model','propeller_step',
         'propeller_general','turbine_general','rocket_general','propulsion_general','fuel_loading',
         'advanced_mass','prop_steady','em_operating']
DIRECTORY=ROOT/'.native_em'


@contextmanager
def build_lock():
    """Serialize rebuilds across worker processes on Windows and POSIX."""
    DIRECTORY.mkdir(exist_ok=True)
    with (DIRECTORY/'.build.lock').open('a+b') as lock:
        if os.name == 'nt':
            import msvcrt
            lock.seek(0, 2)
            if lock.tell() == 0:
                lock.write(b'0'); lock.flush()
            while True:
                lock.seek(0)
                try:
                    msvcrt.locking(lock.fileno(), msvcrt.LK_NBLCK, 1)
                    break
                except OSError as error:
                    import errno
                    if error.errno not in (errno.EACCES, errno.EAGAIN, errno.EDEADLK):
                        raise
                    time.sleep(.1)
            try:
                yield
            finally:
                lock.seek(0)
                msvcrt.locking(lock.fileno(), msvcrt.LK_UNLCK, 1)
        else:
            import fcntl
            fcntl.flock(lock, fcntl.LOCK_EX)
            try:
                yield
            finally:
                fcntl.flock(lock, fcntl.LOCK_UN)


def signature():
    digest=hashlib.sha256(b'cython-native-em-v1-no-fast-math')
    for name in MODULES:
        digest.update(name.encode());digest.update((ROOT/'scripts'/f'{name}.py').read_bytes())
    digest.update((ROOT/'scripts/build_em_backend.py').read_bytes())
    digest.update((ROOT/'scripts/em_compiled_blades.py').read_bytes())
    digest.update((ROOT/'scripts/em_compiled_propeller.py').read_bytes())
    digest.update((ROOT/'scripts/em_compiled_piston.py').read_bytes())
    digest.update((ROOT/'scripts/polar_model.py').read_bytes())
    digest.update(sys.version.encode())
    digest.update((sys.platform + platform.machine()).encode())
    return digest.hexdigest()


def activate():
    if os.environ.get('WT_EM_BACKEND')=='python':return 'python'
    expected=signature()
    def current():
        try:return json.loads((DIRECTORY/'manifest.json').read_text())['signature']==expected
        except (OSError,ValueError,KeyError):return False
    if not current():
        # Equation edits invalidate the compiled FM. Rebuild once before the
        # solver imports any FM module, so an ordinary restart never silently
        # turns every chart into a slow Python-only calculation.
        with build_lock():
            if not current():
                completed=subprocess.run([sys.executable,str(ROOT/'scripts'/'build_em_backend.py')],
                    cwd=str(ROOT),stdout=subprocess.DEVNULL,stderr=subprocess.PIPE,text=True)
                if completed.returncode:
                    raise RuntimeError('Compiled EM backend rebuild failed: '+completed.stderr[-2000:])
    try:
        manifest=json.loads((DIRECTORY/'manifest.json').read_text())
        if manifest['signature']!=expected:raise RuntimeError('Compiled EM backend signature mismatch after rebuild')
        path=str(DIRECTORY/'lib')
        # Spawned workers inherit sys.path, then their entry script may insert
        # scripts/ ahead of it. Restore precedence even when lib is present.
        while path in sys.path:sys.path.remove(path)
        sys.path.insert(0,path)
        return 'compiled'
    except (OSError,ValueError,KeyError) as error:
        raise RuntimeError('Compiled EM backend is unavailable') from error
