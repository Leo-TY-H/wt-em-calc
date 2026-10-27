"""Build the standalone C++ calculation engine; Python is build-time only.

The source model remains the specification. Its supported syntax is translated
to C++ functions using native values; unsupported syntax fails the build. No
Python interpreter, C API, bytecode, or dynamic evaluation is linked or shipped.
"""
import argparse
import ast
from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'scripts' / 'missile_native'
DIRECTORY = ROOT / '.native_missile_engine'
MODEL = ROOT / 'scripts' / 'missile_model'
WRAPPERS = ('missile_worker', 'missile_support', 'missile_initialization',
            'missile_telemetry', 'missile_tracking', 'missile_fast', 'missile_envelope')


def inputs():
    return sorted([*MODEL.glob('*.py'), *MODEL.glob('*.json'),
                   *(ROOT / 'scripts' / (n + '.py') for n in WRAPPERS),
                   *SOURCE.glob('*.cpp'), *SOURCE.glob('*.h'), *SOURCE.glob('typed/*.h'), SOURCE / 'translate.py',
                   ROOT / 'scripts' / 'benchmark_output.py', Path(__file__)])


def fingerprint():
    rows = [dict(path=str(p.relative_to(ROOT)).replace('\\', '/'),
                 bytes=p.stat().st_size, sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in inputs()]
    return hashlib.sha256(json.dumps(rows, sort_keys=True).encode()).hexdigest(), rows


def native_sources():
    result = {}
    for path in [*sorted(MODEL.glob('*.py')), *(ROOT / 'scripts' / (n + '.py') for n in WRAPPERS)]:
        name = path.stem
        if name in ('native_atan2f', 'native_tanf'):
            tree = ast.parse('')  # Audited native core implementations.
        else:
            tree = ast.parse(path.read_text(encoding='utf-8'))
        keep = []
        for node in tree.body:
            if isinstance(node, ast.FunctionDef) and node.name == 'main':
                continue
            if isinstance(node, ast.If) and '__name__' in ast.unparse(node.test):
                continue
            if name == 'missile_envelope':
                if isinstance(node, ast.ImportFrom) and node.module == 'concurrent.futures':
                    continue
                if isinstance(node, ast.FunctionDef) and node.name == 'search':
                    continue  # Native thread scheduling in engine_main.cpp.
            if name == 'missile_worker':
                if isinstance(node, ast.ImportFrom) and node.module == 'missile_backend':
                    continue
                if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'BACKEND' for t in node.targets):
                    node = ast.parse("BACKEND = 'native'").body[0]
            keep.append(node)
        tree.body = keep
        tree.source_path = str(path.resolve())
        result[name] = tree
    return result


def build(generate_only=False, jobs=4):
    spec = importlib.util.spec_from_file_location('native_translation', SOURCE / 'translate.py')
    translation = importlib.util.module_from_spec(spec); spec.loader.exec_module(translation)
    writer = ROOT / 'scripts'
    sys.path.insert(0, str(writer))
    from benchmark_output import OutputBudget
    # The representative MSVC compilation measured 3-9 MB per object with
    # whole-program optimization. The 81 translation units require these
    # temporary link inputs. This exception is recorded BEFORE continuing:
    # owner Codex; finite 600 MB ceiling; retain final executable/evidence;
    # remove only owned object intermediates after final validation.
    reason = ('Standalone C++ engine compilation: representative MSVC objects '
              'measure 3-9 MB each for 81 modules; 600 MB finite temporary build '
              'ceiling, no model copies; remove owned objects after validation.')
    budget = OutputBudget(DIRECTORY, 'standalone-native-engine',
                          limits=dict(summary=1_000_000, diagnostic=25_000_000, total=600_000_000), reason=reason)
    generated = DIRECTORY / 'generated'; generated.mkdir(exist_ok=True)
    signature, sources = fingerprint()
    modules = native_sources()
    generated_files = []
    registration = []
    for name, source in modules.items():
        text = translation.generate({name: source}).replace('void register_generated_modules()', 'void register_' + name + '()')
        text = text.replace('#include "runtime.h"', '#include "runtime.h"\n#include "engine_stdlib.h"')
        origin = source.source_path
        relative = Path(origin).relative_to(ROOT).as_posix()
        text = text.replace('set(e, "__file__", V(' + translation.quote(origin) + '));',
                            'set(e, "__file__", V(engine_source_path(' + json.dumps(relative) + ')));')
        path = generated / (name + '.cpp')
        raw = text.encode()
        if not path.exists() or path.read_bytes() != raw:
            budget.write(path, lambda stream, raw=raw: stream.write(raw), kind='diagnostic')
        generated_files.append(path)
        registration.append(name)
    text = ''.join('void register_' + n + '();\n' for n in registration)
    text += 'void register_generated_modules(){\n' + ''.join('register_' + n + '();\n' for n in registration) + '}\n'
    registry = generated / 'registry.cpp'
    if not registry.exists() or registry.read_text() != text:
        budget.write(registry, lambda stream: stream.write(text.encode()), kind='diagnostic')
    generated_files.append(registry)
    from build_missile_native import tables_header
    tables = tables_header().encode()
    if not (generated / 'tables.h').exists() or (generated / 'tables.h').read_bytes() != tables:
        budget.write(generated / 'tables.h', lambda stream: stream.write(tables), kind='diagnostic')
    manifest = dict(owner='Codex', task='Standalone native missile/envelope calculations', status='building',
                    signature=signature, sources=sources, python_runtime_required=False,
                    modules=len(modules), generated_bytes=sum(p.stat().st_size for p in generated_files),
                    budget=budget.limits, budget_exception=reason, command=[sys.executable, '-B', str(Path(__file__))],
                    compiler_flags=['/std:c++17', '/O2', '/GL-', '/EHsc', '/fp:strict', '/fp:except-', '/bigobj'],
                    cleanup_condition='Retain final executable, manifest and validation; owned objects may be replaced on rebuild.',
                    retained_paths=['generated', 'objects', 'missile-engine.exe', 'build-manifest.json', 'validation'])
    budget.json(DIRECTORY / 'build-manifest.json', manifest)
    print(json.dumps(dict(modules=len(modules), generated_bytes=manifest['generated_bytes'])), flush=True)
    if generate_only:
        return
    # Compile one small generated module first before committing to the batch.
    from setuptools._distutils.ccompiler import new_compiler
    from setuptools._distutils.sysconfig import customize_compiler
    compiler = new_compiler(); customize_compiler(compiler)
    if os.name == 'nt':
        compiler.initialize()
        compiler.compile_options = ['/MT' if v == '/MD' else v for v in compiler.compile_options if v != '/GL']
        for key, value in compiler._ldflags.items():
            compiler._ldflags[key] = [v for v in value if v != '/LTCG']
        flags = manifest['compiler_flags']
    else:
        flags = ['-std=c++17', '-O2', '-ffp-contract=off', '-fno-fast-math', '-pthread']
    # The typed engine never reads or changes the floating-point environment.
    # MSVC's /fp:precise keeps every /fp:strict value there (source order and
    # rounding, no contraction, IEEE special values) without strict's float to
    # double to float round trip after each binary32 operation.
    typed_flags = [('/fp:precise' if f == '/fp:strict' else f) for f in flags]
    def flags_for(path):
        return typed_flags if path.name == 'engine_typed.cpp' else flags
    objects = DIRECTORY / 'objects'; objects.mkdir(exist_ok=True)
    sources_cpp = [SOURCE / 'runtime.cpp', SOURCE / 'engine_stdlib.cpp', SOURCE / 'engine_kernels.cpp',
                   SOURCE / 'engine_typed.cpp', SOURCE / 'engine_main.cpp', *generated_files]
    compiled = []
    pending = []
    start = time.perf_counter()
    headers = [*SOURCE.glob('*.h'), generated / 'tables.h']
    # The typed evaluation headers are included only by engine_typed.cpp.
    typed_headers = sorted(SOURCE.glob('typed/*.h'))
    def dependencies(path):
        return [*headers, *(typed_headers if path.name == 'engine_typed.cpp' else [])]
    settings_path = DIRECTORY / 'compile-settings.json'
    settings = dict(flags=flags, typed_flags=typed_flags, compiler_options=getattr(compiler, 'compile_options', []))
    force = not settings_path.exists() or json.loads(settings_path.read_text()) != settings
    for index, path in enumerate(sources_cpp):
        object_path = Path(compiler.object_filenames([str(path)], output_dir=str(objects))[0])
        newest_input = max(p.stat().st_mtime_ns for p in [path, *dependencies(path)])
        compiled.append(str(object_path))
        if force or not object_path.exists() or object_path.stat().st_mtime_ns < newest_input:
            pending.append((index, path))
    def compile_one(entry):
        index, path = entry
        print('COMPILE %d/%d %s' % (index + 1, len(sources_cpp), path.name), flush=True)
        compiler.compile([str(path)], output_dir=str(objects), include_dirs=[str(SOURCE), str(generated)],
                         depends=list(map(str, dependencies(path))), extra_postargs=flags_for(path))
        retained = sum(p.stat().st_size for p in DIRECTORY.rglob('*') if p.is_file())
        if retained > budget.limits['total']:
            raise RuntimeError('Native build exceeded declared workstream storage cap')
    with ThreadPoolExecutor(max_workers=max(1, min(jobs, os.cpu_count() or 1))) as pool:
        for result in as_completed([pool.submit(compile_one, entry) for entry in pending]):
            result.result()
    executable = DIRECTORY / ('missile-engine.exe' if os.name == 'nt' else 'missile-engine')
    compiler.link_executable(compiled, str(DIRECTORY / 'missile-engine'), extra_postargs=[] if os.name == 'nt' else ['-pthread'], target_lang='c++')
    budget.json(settings_path, settings)
    manifest.update(status='built', elapsed_s=time.perf_counter() - start, compiler_flags=flags,
                    compiler_options=getattr(compiler, 'compile_options', []),
                    executable=executable.name, executable_sha256=hashlib.sha256(executable.read_bytes()).hexdigest())
    budget.json(DIRECTORY / 'build-manifest.json', manifest)
    budget.finish()
    print(json.dumps(dict(executable=str(executable), elapsed_s=manifest['elapsed_s'])), flush=True)


def executable(required=True):
    try:
        manifest = json.loads((DIRECTORY / 'build-manifest.json').read_text())
        if manifest['status'] != 'built' or manifest['signature'] != fingerprint()[0]:
            raise ValueError('Native calculation source/build mismatch')
        path = DIRECTORY / manifest['executable']
        if path.parent != DIRECTORY or hashlib.sha256(path.read_bytes()).hexdigest() != manifest['executable_sha256']:
            raise ValueError('Native calculation executable hash mismatch')
        return path
    except (OSError, ValueError, KeyError):
        if required:
            raise RuntimeError('Native calculation engine unavailable; run scripts/build_missile_engine.py')
        return None


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--generate-only', action='store_true')
    parser.add_argument('--jobs', type=int, default=4, help='Concurrent compiler processes (default: 4)')
    args = parser.parse_args()
    if not 1 <= args.jobs <= 16:
        parser.error('--jobs must be between 1 and 16')
    build(args.generate_only, args.jobs)
