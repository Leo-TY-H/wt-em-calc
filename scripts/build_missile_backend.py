"""Build optional exact arithmetic/copy helpers; no compiler needed at runtime."""
import hashlib
import json
import os
from pathlib import Path
import platform
import sys

ROOT = Path(__file__).resolve().parents[1]
DIRECTORY = ROOT / '.native_missile'
SOURCE = r'''
from copy import deepcopy as reference_copy
from libc.math cimport isfinite

cpdef double f32(double value):
    return <float>value

cpdef double add(double a, double b):
    cdef double x = <float>a, y = <float>b
    return <float>(x+y)

cpdef double sub(double a, double b):
    cdef double x = <float>a, y = <float>b
    return <float>(x-y)

cpdef double mul(double a, double b):
    cdef double x = <float>a, y = <float>b
    return <float>(x*y)

cpdef double div(double a, double b):
    cdef double x = <float>a, y = <float>b
    if y == 0.: raise ZeroDivisionError('float division by zero')
    return <float>(x/y)

cpdef object deepcopy(object value, object memo=None):
    cdef object kind = type(value), key, item, output, identity
    if kind is float or kind is int or kind is str or kind is bool or value is None or kind is bytes:
        return value
    if kind is not dict and kind is not list:
        return reference_copy(value, memo)
    if memo is None:
        memo = {}
    identity = id(value)
    if identity in memo:
        return memo[identity]
    if kind is dict:
        output = {}
        memo[identity] = output
        for key, item in (<dict>value).items():
            output[deepcopy(key, memo)] = deepcopy(item, memo)
    else:
        output = []
        memo[identity] = output
        for item in <list>value:
            output.append(deepcopy(item, memo))
    memo.setdefault(id(memo), []).append(value)
    return output

cdef void _finite(object value, object path) except *:
    if isinstance(value, float) and not isfinite(value):
        raise ValueError(path+' must be finite')
    if isinstance(value, dict):
        for key, item in value.items():
            _finite(item, path+'.'+key)
    elif isinstance(value, (list, tuple)):
        for index, item in enumerate(value):
            _finite(item, f'{path}[{index}]')

def finite(value, path='state'):
    _finite(value, path)

cdef bint _pid_slot(tuple path):
    if len(path)==5 and path[:1]==('guidance',) and path[1] in ('orientation','guidance') and path[2]=='pid':
        return type(path[3]) is int and 0<=path[3]<2 and type(path[4]) is int and 0<=path[4]<8
    return len(path)==4 and path[:3]==('guidance','propulsion','pid') and type(path[3]) is int and 0<=path[3]<8

cdef void _flight_numbers(object value, tuple path) except *:
    if isinstance(value, float) and not isfinite(value) and not _pid_slot(path):
        raise ValueError('state.'+'.'.join(map(str,path))+' must be finite')
    if isinstance(value, dict):
        for key, item in value.items():
            _flight_numbers(item, path+(key,))
    elif isinstance(value, (list, tuple)):
        for index, item in enumerate(value):
            _flight_numbers(item, path+(index,))

def validate_flight_numbers(state):
    _flight_numbers(state, ())
'''


def signature():
    from missile_numeric_build import MODULES
    digest = hashlib.sha256((SOURCE + sys.implementation.cache_tag + sys.platform +
                            platform.machine().lower()).encode())
    digest.update((ROOT / 'scripts/missile_numeric_build.py').read_bytes())
    digest.update(Path(__file__).read_bytes())
    for name in MODULES:
        digest.update((ROOT / 'scripts/missile_model' / (name + '.py')).read_bytes())
    return digest.hexdigest()


def build():
    from Cython.Build import cythonize
    from setuptools import Extension, setup
    DIRECTORY.mkdir(exist_ok=True)
    source = DIRECTORY / '_missile_accel.pyx'
    source.write_text(SOURCE, encoding='utf-8')
    library = DIRECTORY / ('lib-' + signature()[:16])
    flags = ['/O2', '/fp:strict'] if os.name == 'nt' else ['-O3', '-ffp-contract=off', '-fno-fast-math']
    from missile_numeric_build import MODULES, source as numeric_source
    model = ROOT / 'scripts/missile_model'
    specifications = [Extension('_missile_accel', [str(source)], extra_compile_args=flags)]
    for name in MODULES:
        target = DIRECTORY / ('_missile_numeric_' + name + '.pyx')
        target.write_text(numeric_source(name, model), encoding='utf-8')
        specifications.append(Extension('_missile_numeric_' + name, [str(target)], extra_compile_args=flags))
    extensions = cythonize(specifications,
                          compiler_directives={'language_level': 3, 'infer_types': False})
    setup(name='missile-exact-helpers', ext_modules=extensions,
          script_args=['build_ext', '--build-lib', str(library), '--build-temp', str(DIRECTORY/'objects')])
    binaries = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                for p in library.iterdir() if p.suffix in ('.pyd', '.so')}
    if len(binaries) != 1 + len(MODULES):
        raise RuntimeError('Missing compiled missile extensions')
    manifest = dict(signature=signature(), library=library.name, binaries=binaries,
                    compiler_flags=flags)
    temporary = DIRECTORY / 'manifest.tmp'
    temporary.write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    temporary.replace(DIRECTORY/'manifest.json')


if __name__ == '__main__':
    build()
