"""Optional acceleration, bound only inside the isolated missile interpreter.

WT_MISSILE_BACKEND=reference keeps the full reference path. 'python' uses fast
session orchestration without an extension. 'auto' uses a valid local extension
when present; missing/stale binaries fall back to fast Python without a build.
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


def activate():
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
