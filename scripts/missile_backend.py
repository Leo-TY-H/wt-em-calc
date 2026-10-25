"""Optional acceleration, bound only inside the isolated missile interpreter.

WT_MISSILE_BACKEND=reference keeps the full reference path. 'python' uses fast
session orchestration without an extension. 'auto' uses a valid local extension
when present; missing/stale binaries fall back to fast Python without a build.

WT_MISSILE_NATIVE selects the native flight model (scripts/missile_native:
motor, body substeps, speed cap and the four shared controllers) on top of a
non-reference backend: 'auto' (default) uses a verified available build, '0'
leaves it off, and '1' requires the module from scripts/build_missile_native.py.
Seeker, manager and session code stay in Python either way.
"""
import hashlib
import importlib
import importlib.util
import json
import os
from pathlib import Path
import sys
from types import FunctionType
from build_missile_backend import DIRECTORY, signature

NATIVE = False


def activate():
    global NATIVE
    mode = _activate_helpers()
    NATIVE = activate_native(mode)
    return mode


def activate_native(mode):
    """Bind the native flight model into the website's flight compositions."""
    setting = os.environ.get('WT_MISSILE_NATIVE', 'auto')
    if setting not in ('0', '1', 'auto'):
        raise ValueError('Unknown WT_MISSILE_NATIVE')
    if setting == '0':
        return False
    if mode == 'reference':
        if setting == '1':
            raise RuntimeError('The reference backend never uses the native flight model')
        return False
    from build_missile_native import load
    native = load(required=setting == '1')
    if native is None:
        return False

    # The native wrapper loads after helper activation, so its Python globals
    # were not visited by that pass. Reuse the verified state copier too.
    import copy
    native.deepcopy = sys.modules['_missile_accel'].deepcopy if mode == 'compiled' else copy.deepcopy

    import kernels
    import orientation_control
    import guidance_request
    import acceleration_control
    import propulsion_control
    import optical_point_flight
    import optical_point_controls
    # Compiled helpers (when active) are the bindings current callers hold.
    def bound(name, function):
        compiled = sys.modules.get('_missile_numeric_' + name)
        return getattr(compiled, function.__name__, function) if compiled is not None else function
    reference_acceleration = bound('acceleration_control', acceleration_control.update)
    reference_propulsion = propulsion_control.update

    def acceleration_update(p, aero, motor, state, request, measured, q, velocity, height, time, dt, *,
                            environment=None, matrix_velocity_frame=True):
        # Native code implements the default world values only.
        if environment:
            return reference_acceleration(p, aero, motor, state, request, measured, q, velocity, height, time, dt,
                                          environment=environment, matrix_velocity_frame=matrix_velocity_frame)
        return native.flight_acceleration(p, aero, motor, state, request, measured, q, velocity, height, time, dt,
                                          matrix_velocity_frame)

    def propulsion_update(p, state, time, position, velocity, tgo, closure, dt, *, environment=None):
        if environment:
            return reference_propulsion(p, state, time, position, velocity, tgo, closure, dt, environment=environment)
        return native.flight_propulsion(p, state, time, position, velocity, tgo, closure, dt)

    def flight_update(p, state, targets, dt, *, guidance_step=None):
        return native.flight_update(p, state, targets, dt,
                                    optical_point_controls.update if guidance_step is None else guidance_step)

    replacements = {kernels.motor_scalar: native.flight_motor,
                    orientation_control.update: native.flight_orientation,
                    guidance_request.request: native.flight_request,
                    acceleration_control.update: acceleration_update,
                    reference_acceleration: acceleration_update,
                    propulsion_control.update: propulsion_update,
                    optical_point_flight.update: flight_update,
                    bound('optical_point_flight', optical_point_flight.update): flight_update}
    # The defining modules keep their reference functions; only consumers
    # (e.g. optical_manager_controls, geometric_*_flight, missile_support and
    # their compiled counterparts) are rebound.
    defining = {'kernels', 'orientation_control', 'guidance_request', 'acceleration_control', 'propulsion_control',
                'optical_point_flight', '_missile_numeric_acceleration_control', '_missile_numeric_optical_point_flight'}
    scripts = Path(__file__).resolve().parent
    model = scripts/'missile_model'
    for name, module in list(sys.modules.items()):
        filename = getattr(module, '__file__', None)
        if not filename or name in defining:
            continue
        path = Path(filename).resolve()
        if path.parent != model and not name.startswith('_missile_numeric_') and not (path.parent == scripts and path.stem in
                {'missile_worker', 'missile_support', 'missile_initialization', 'missile_telemetry', 'missile_fast'}):
            continue
        for key, value in list(vars(module).items()):
            if callable(value):
                try:
                    replacement = replacements.get(value)
                except TypeError:
                    continue
                if replacement is not None:
                    setattr(module, key, replacement)
    return True


def _activate_helpers():
    mode = os.environ.get('WT_MISSILE_BACKEND', 'auto')
    if mode not in ('auto', 'python', 'reference', 'compiled'):
        raise ValueError('Unknown WT_MISSILE_BACKEND')
    if mode in ('python', 'reference'):
        return mode
    try:
        manifest = json.loads((DIRECTORY/'manifest.json').read_text())
        if manifest['signature'] != signature():
            raise ValueError('Compiled helper source/ABI mismatch')
        library = manifest['library']
        if Path(library).name != library:
            raise ValueError('Invalid compiled library directory')
        binaries = manifest['binaries']
        from missile_numeric_build import MODULES
        expected = {'_missile_accel', *('_missile_numeric_' + name for name in MODULES)}
        if {name.split('.')[0] for name in binaries} != expected or len(binaries) != len(expected):
            raise ValueError('Unexpected compiled missile modules')
        modules = {}
        # Verify every file before loading any code.
        for name, digest in binaries.items():
            if Path(name).name != name:
                raise ValueError('Invalid compiled helper path')
            path = DIRECTORY/library/name
            if hashlib.sha256(path.read_bytes()).hexdigest() != digest:
                raise ValueError('Compiled helper hash mismatch')
        for name in sorted(binaries, key=lambda n: (not n.startswith('_missile_accel.'), n)):
            key = name.split('.')[0]
            spec = importlib.util.spec_from_file_location(key, DIRECTORY/library/name)
            module = importlib.util.module_from_spec(spec)
            sys.modules[key] = module
            spec.loader.exec_module(module)
            modules[key] = module
        accelerator = modules['_missile_accel']
    except (OSError, ValueError, KeyError, TypeError, AttributeError, ImportError):
        if mode == 'compiled':
            raise RuntimeError('Compiled missile helpers unavailable; run scripts/build_missile_backend.py')
        return 'python'

    import copy
    import kernels
    import flight_session
    import state_binary32
    replacements = {kernels.f32: accelerator.f32, kernels.add: accelerator.add,
                    kernels.sub: accelerator.sub, kernels.mul: accelerator.mul,
                    kernels.div: accelerator.div, copy.deepcopy: accelerator.deepcopy,
                    flight_session.finite: accelerator.finite,
                    state_binary32.validate_flight_numbers: accelerator.validate_flight_numbers}
    scripts = Path(__file__).resolve().parent
    model = scripts/'missile_model'
    if os.environ.get('WT_MISSILE_NUMERIC', '1') != '0':
        for name in MODULES:
            reference = importlib.import_module(name)
            compiled = modules['_missile_numeric_' + name]
            # The imported reference already identifies its own definitions.
            # Parsing every source again allocates large temporary ASTs during
            # activation and leaves allocator arenas resident for short flights.
            for function, value in vars(reference).items():
                if isinstance(value, FunctionType) and value.__module__ == reference.__name__:
                    replacements[value] = getattr(compiled, function)
    # Avoid touching EM, standard-library modules, or unrelated interpreters.
    for module in list(sys.modules.values()):
        if module is accelerator:
            # Its fallback must remain the standard-library deepcopy, including
            # dataclasses and tuples; replacing it would recurse into itself.
            continue
        filename = getattr(module, '__file__', None)
        if not filename:
            continue
        path = Path(filename).resolve()
        if path.parent != model and path.parent != DIRECTORY/library and not (path.parent == scripts and path.stem in
                {'missile_worker', 'missile_support', 'missile_initialization',
                 'missile_telemetry', 'missile_fast'}):
            continue
        for key, value in list(vars(module).items()):
            if callable(value):
                try:
                    replacement = replacements.get(value)
                except TypeError:
                    continue
                if replacement is not None:
                    setattr(module, key, replacement)
    return 'compiled'
