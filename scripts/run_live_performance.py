"""Time one job at a time; temporarily suspend only this worktree's reference corpus."""
import os
from pathlib import Path
import subprocess
import sys
import psutil

ROOT=Path(__file__).resolve().parents[1]


def main():
    suspended=[]
    try:
        for process in psutil.process_iter(['cmdline','name']):
            command=process.info['cmdline'] or []
            if any(arg.endswith('verify_missile_live.py') for arg in command) and '--reference' in command and Path(process.cwd()).resolve()==ROOT:
                for child in [process,*process.children(recursive=True)]:
                    if child.pid not in [p.pid for p in suspended]:child.suspend();suspended.append(child)
        print('Suspended owned reference corpus for isolated timing: '+str([p.pid for p in suspended]),flush=True)
        env=dict(os.environ,PLAYWRIGHT_BROWSERS_PATH=str(ROOT.parent/'tools/live-browser/browsers'))
        subprocess.run(['node',str(ROOT/'scripts/benchmark_missile_live_browser.cjs')],cwd=ROOT,env=env,check=True)
        subprocess.run([sys.executable,'-X','utf8',str(ROOT/'scripts/benchmark_live_envelope.py')],cwd=ROOT,env=env,check=True)
    finally:
        for process in reversed(suspended):
            try:process.resume()
            except psutil.NoSuchProcess:pass
        print('Reference corpus resumed.',flush=True)


if __name__=='__main__':main()
