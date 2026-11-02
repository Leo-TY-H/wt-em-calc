"""Build the current native translation in WSL without changing the Windows manifest."""
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import subprocess
import time
from benchmark_output import OutputBudget
import build_missile_engine as native


def main():
    native.executable()  # Validate the retained Windows translation first.
    root=native.ROOT;output=root/'.linux_missile_engine';output.mkdir(exist_ok=True)
    budget=OutputBudget(output,'live-linux-build',limits=dict(summary=1_000_000,diagnostic=25_000_000,total=600_000_000),
                        reason='Owner live-missile: one WSL native parity build, 600 MB finite compiler ceiling; retain executable and hash, remove owned objects after acceptance.')
    source=native.SOURCE;generated=native.DIRECTORY/'generated';began=time.perf_counter()
    signature=native.fingerprint()[0]
    budget.json(output/'manifest.json',dict(status='building',signature=signature,cleanup='Remove owned objects after acceptance'))
    sources=[source/name for name in ['runtime.cpp','engine_stdlib.cpp','engine_kernels.cpp','engine_typed.cpp','engine_main.cpp']]+sorted(generated.glob('*.cpp'))
    flags=['-std=c++17','-O2','-ffp-contract=off','-fno-fast-math','-pthread','-I'+str(source),'-I'+str(generated)]
    def compile_one(pair):
        i,path=pair;obj=output/(str(i)+'.o')
        subprocess.run(['g++',*flags,'-c',str(path),'-o',str(obj)],check=True)
        if sum(p.stat().st_size for p in output.iterdir() if p.is_file())>600_000_000:raise RuntimeError('Linux compiler ceiling exceeded')
        return str(obj)
    with ThreadPoolExecutor(max_workers=4) as pool:objects=list(pool.map(compile_one,enumerate(sources)))
    exe=output/'missile-engine';subprocess.run(['g++',*objects,'-pthread','-o',str(exe)],check=True)
    manifest=dict(status='built',signature=signature,bytes=exe.stat().st_size,sha256=hashlib.sha256(exe.read_bytes()).hexdigest(),elapsed_s=time.perf_counter()-began)
    budget.json(output/'manifest.json',manifest);budget.finish();print(json.dumps(manifest))


if __name__=='__main__':main()
