"""Build browser engine from an already-current native translation, never regenerate it."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time

import build_missile_engine as native
from benchmark_output import OutputBudget

EMSDK = '4.0.10'
ROOT = native.ROOT
SOURCE = ROOT / 'scripts/missile_wasm'
BUILD = ROOT / '.wasm_missile_engine'
OUTPUT = ROOT / 'app/live'


def build(jobs=4, assertions=False):
    native.executable()
    compiler = shutil.which('emcc')
    if not compiler:
        raise RuntimeError('Emscripten 4.0.10 is required. Activate its environment, then rerun this command.')
    version = subprocess.check_output([compiler, '--version'], text=True)
    if EMSDK not in version.splitlines()[0]:
        raise RuntimeError('Expected Emscripten '+EMSDK+', got '+version.splitlines()[0])
    signature, inputs = native.fingerprint()
    wasm_inputs = {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
                   for p in sorted(SOURCE.glob('*')) if p.is_file()}
    wasm_inputs['scripts/build_missile_wasm.py'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    fingerprint = hashlib.sha256(json.dumps([signature, wasm_inputs, EMSDK, assertions], sort_keys=True).encode()).hexdigest()
    BUILD.mkdir(exist_ok=True); OUTPUT.mkdir(exist_ok=True)
    # Explicit bounded compiler-intermediate exception, separate from benchmark evidence.
    budget = OutputBudget(BUILD, 'live-missile-wasm-build',
                          limits=dict(summary=1_000_000, diagnostic=25_000_000, total=600_000_000),
                          reason='Owner live-missile: generated WASM objects for one build; 600 MB ceiling; retain binaries, remove owned intermediates after acceptance.')
    budget.json(BUILD / 'manifest.json', dict(status='building', native_signature=signature, wasm_fingerprint=fingerprint,
                emsdk=EMSDK, inputs=wasm_inputs, cleanup='Remove owned compiler objects after acceptance'))
    header = '\n'.join('#define '+key+' '+json.dumps(value) for key,value in
                       [('NATIVE_SIGNATURE',signature),('WASM_FINGERPRINT',fingerprint),('EMSDK_VERSION',EMSDK)])
    (BUILD / 'live_build.h').write_text(header+'\n')
    generated = native.DIRECTORY / 'generated'
    sources = sorted(generated.glob('*.cpp')) + [native.SOURCE / name for name in
              ('runtime.cpp','engine_stdlib.cpp','engine_kernels.cpp','engine_typed.cpp')] + [SOURCE / 'engine_wasm.cpp']
    flags = ['-std=c++17','-O2','-ffp-contract=off','-fno-fast-math','-fwasm-exceptions',
             '-I'+str(native.SOURCE),'-I'+str(generated),'-I'+str(BUILD)]
    began=time.perf_counter()
    dependency_hash=hashlib.sha256(b''.join(p.read_bytes() for p in sorted([*native.SOURCE.glob('*.h'),*native.SOURCE.glob('typed/*.h'),generated/'tables.h',BUILD/'live_build.h']))).hexdigest()
    def compile_one(pair):
        index, source=pair; target=BUILD / (str(index)+'-'+source.stem+'.o')
        stamp=target.with_suffix('.sha256')
        digest=hashlib.sha256(source.read_bytes()+json.dumps([flags,EMSDK,dependency_hash]).encode()).hexdigest()
        if not target.exists() or not stamp.exists() or stamp.read_text()!=digest:
            subprocess.run([compiler,*flags,'-c',str(source),'-o',str(target)],check=True)
            stamp.write_text(digest)
        if sum(p.stat().st_size for p in BUILD.iterdir() if p.is_file()) > 600_000_000:
            raise RuntimeError('WASM build exceeded its declared byte ceiling')
        return target
    with ThreadPoolExecutor(max_workers=jobs) as pool:
        objects=list(pool.map(compile_one,enumerate(sources)))
    link=[*map(str,objects),'-O2','-fwasm-exceptions','--no-entry','-sSTACK_SIZE=1MB','-sALLOW_MEMORY_GROWTH=1',
          '-sINITIAL_MEMORY=32MB','-sMODULARIZE=1','-sEXPORT_ES6=1','-sEXPORT_NAME=createMissileEngine',
          '-sENVIRONMENT=web,node','-sFILESYSTEM=1','-sFORCE_FILESYSTEM=1',
          '-sEXPORTED_RUNTIME_METHODS=FS,ccall,cwrap,UTF8ToString,HEAPF64',
          '-o',str(OUTPUT / 'missile-engine.mjs')]
    if assertions: link.extend(['-sSTACK_OVERFLOW_CHECK=2','-sASSERTIONS=2'])
    response=BUILD/'link.rsp';response.write_text('\n'.join(json.dumps(x.replace('\\','/')) for x in link))
    subprocess.run([compiler,'@'+str(response)],check=True)
    assets={}
    for name in ('missile-engine.mjs','missile-engine.wasm'):
        data=(OUTPUT/name).read_bytes()
        assets[name]=dict(sha256=hashlib.sha256(data).hexdigest(),bytes=len(data),brotli_bytes=int(subprocess.check_output(['node','-e',"const fs=require('fs'),z=require('zlib');console.log(z.brotliCompressSync(fs.readFileSync(process.argv[1])).length)",str(OUTPUT/name)],text=True)))
    if assets['missile-engine.wasm']['bytes'] >= 25*1024*1024:
        raise RuntimeError('WASM exceeds the 25 MiB Pages limit')
    manifest=dict(native_signature=signature,wasm_fingerprint=fingerprint,emsdk=EMSDK,
                  state_version=1,state_size=32,assets=assets,elapsed_s=time.perf_counter()-began,assertions=assertions)
    (OUTPUT/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    budget.json(BUILD/'manifest.json',dict(manifest,status='built'));budget.finish()
    print(json.dumps(manifest))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--jobs',type=int,default=4)
    parser.add_argument('--assertions',action='store_true')
    args=parser.parse_args()
    if not 1<=args.jobs<=16:parser.error('--jobs must be 1–16')
    build(args.jobs,args.assertions)
