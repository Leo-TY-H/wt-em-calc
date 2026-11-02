"""Stream the release/candidate golden comparisons; retain only bounded evidence."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import time
from benchmark_output import OutputBudget
from verify_missile_live import Engine,ROOT,canonical_equal


def main():
    out=ROOT/'outputs/live-missile';budget=OutputBudget(out,'live-baseline');started=time.perf_counter()
    baseline=ROOT.parent/'WT-FM-app/.native_missile_engine/missile-engine.exe'
    candidate=ROOT/'.native_missile_engine/missile-engine.exe'
    pairs={mode:[Engine([str(exe),'--root',str(ROOT),'--mode',mode,'--batch']) for exe in [baseline,candidate]] for mode in ['evaluate','simulator']}
    stats=dict(evaluate_cases=0,evaluate_equal=0,simulator_cases=0,simulator_equal=0,failures=[],baseline_sha256=hashlib.sha256(baseline.read_bytes()).hexdigest(),candidate_sha256=hashlib.sha256(candidate.read_bytes()).hexdigest(),complete=False)
    index=json.loads((ROOT/'scripts/missile_model/launch-profiles/index.json').read_bytes())
    try:
        for profile in index['profiles']:
            for geometry in range(3):
                target=dict(position=[4000.,5000.,0.],velocity=[[200.,0.,0.],[-200.,0.,0.],[0.,0.,200.]][geometry],angles=[[0.,0.,0.],[180.,0.,0.],[90.,0.,0.]][geometry])
                req=dict(missile=profile['path'].removesuffix('.json'),launcher=dict(position=[0.,5000.,0.],velocity=[300.,0.,0.],angles=[0.,0.,0.]),target=target,early_exit=True)
                a,b=[engine.request(req) for engine in pairs['evaluate']];stats['evaluate_cases']+=1
                if canonical_equal(a,b):stats['evaluate_equal']+=1
                elif len(stats['failures'])<10:stats['failures'].append(dict(kind='evaluate',request=req,before=a,after=b))
        for missile in ['us_aim9l_sidewinder','us_aim_120c_5']:
            for geometry in range(3):
                req=dict(missile=missile,launcher=dict(position=[0.,5000.,0.],velocity=[300.,0.,0.],angles=[0.,0.,0.]),
                         target=dict(position=[4000.,5000.,0.],velocity=[[200.,0.,0.],[-200.,0.,0.],[0.,0.,200.]][geometry],angles=[[0.,0.,0.],[180.,0.,0.],[90.,0.,0.]][geometry]))
                for early in [True,False]:
                    for escape in [False,True]:
                        current=deepcopy(req);current['early_exit']=early
                        if escape:current['escape']=dict(load_g=9.,delay_s=1.)
                        a,b=[engine.request(current) for engine in pairs['simulator']];stats['simulator_cases']+=1
                        if canonical_equal(a,b):stats['simulator_equal']+=1
                        elif len(stats['failures'])<10:stats['failures'].append(dict(kind='simulator',request=current,changed_keys=[k for k in a if a[k]!=b[k]]))
        stats['complete']=True
    finally:
        for pair in pairs.values():
            for engine in pair:engine.close()
        stats['elapsed_s']=time.perf_counter()-started;budget.json(out/'baseline.json',stats);budget.finish()
    print(json.dumps(stats))
    if stats['evaluate_cases']!=stats['evaluate_equal'] or stats['simulator_cases']!=stats['simulator_equal']:raise SystemExit(1)


if __name__=='__main__':main()
