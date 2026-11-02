"""Alternating Standard envelope timing and stream parity against the retained release."""
import hashlib
import json
import os
from pathlib import Path
import statistics
import subprocess
import time
from benchmark_output import OutputBudget

ROOT=Path(__file__).resolve().parents[1]


def run(executable,request):
    begin=time.perf_counter();process=subprocess.Popen([str(executable),'--root',str(ROOT),'--mode','envelope'],
        stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,text=True,encoding='utf-8',
        env=dict(os.environ,WT_ENVELOPE_WORKERS='4'),creationflags=subprocess.CREATE_NO_WINDOW if os.name=='nt' else 0)
    process.stdin.write(json.dumps(request));process.stdin.close();messages=[];rays=[];seen=0
    for line in process.stdout:
        seen+=len(line.encode());value=json.loads(line);kind=value['type']
        if kind=='error':raise RuntimeError(value['error'])
        if kind=='progress':continue
        data=value[kind]
        for key in ['elapsed_s','wall_s','workers']:data.pop(key,None)
        digest=hashlib.sha256(json.dumps(data,sort_keys=True,separators=(',',':')).encode()).hexdigest()
        (rays if kind=='ray' else messages).append(digest)
    code=process.wait();process.stdout.close()
    if code:raise RuntimeError('Envelope failed')
    return dict(seconds=time.perf_counter()-begin,rays=len(rays),bytes_drained=seen,
                result_sha256=hashlib.sha256(json.dumps([messages,sorted(rays)]).encode()).hexdigest())


def main():
    out=ROOT/'outputs/live-missile';budget=OutputBudget(out,'live-envelope-performance')
    before=ROOT.parent/'WT-FM-app/.native_missile_engine/missile-engine.exe';after=ROOT/'.native_missile_engine/missile-engine.exe'
    report=dict(workers=4,cases=[],source_baseline=str(before),source_candidate=str(after))
    try:
        for missile in ['us_aim9l_sidewinder','us_aim_120c_5']:
            request=dict(missile=missile,frame='target',launcher=dict(speed_mps=300.,platform='air'),target=dict(speed_mps=200.),layers_m=[5000.],quality='standard')
            case=dict(request=request,before=[],after=[]);report['cases'].append(case)
            for repetition in range(5):
                for name,exe in [('before',before),('after',after)]:
                    value=run(exe,request);case[name].append(value)
                    print(json.dumps(dict(missile=missile,repetition=repetition,build=name,**value)),flush=True)
                    budget.json(out/'envelope-performance.json',report)
            case['identical']=len({r['result_sha256'] for name in ['before','after'] for r in case[name]})==1
            for name in ['before','after']:
                values=[r['seconds'] for r in case[name]];case[name+'_median_s']=statistics.median(values);case[name+'_spread_s']=max(values)-min(values)
            change=case['after_median_s']-case['before_median_s'];spread=max(case['before_spread_s'],case['after_spread_s'])
            case['timing_conclusion']='unresolved within run-to-run spread' if abs(change)<=spread else 'faster' if change<0 else 'slower'
    finally:budget.json(out/'envelope-performance.json',report);budget.finish()
    if not all(c.get('identical') for c in report['cases']):raise SystemExit(1)


if __name__=='__main__':main()
