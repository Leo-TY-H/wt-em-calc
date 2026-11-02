"""Canonical JSON parity, using four processes and shared immutable input files."""
from concurrent.futures import ProcessPoolExecutor
import json
import os
import time
from benchmark_output import OutputBudget
from verify_missile_live import Engine,ROOT,MISSILES,logs,scenario

def check_missile(missile):
    os.environ['WT_MISSILE_ENGINE']='python';os.environ['WT_MISSILE_BACKEND']='reference'
    from missile_worker import simulate
    engine=Engine([str(ROOT/'.native_missile_engine/missile-engine.exe'),'--root',str(ROOT),'--mode','simulator','--batch'],dict(os.environ,WT_MISSILE_TYPED='compare',WT_MISSILE_TYPED_LOG='1'))
    result=dict(missile=missile,cases=0,equal=0,failures=[])
    try:
        for case,log in enumerate(logs()):
            req=scenario(missile,case,log);expected=simulate(req);actual=engine.request(req);result['cases']+=1
            if json.dumps(expected,sort_keys=True)==json.dumps(actual,sort_keys=True):result['equal']+=1
            elif len(result['failures'])<10:result['failures'].append(dict(case=case,request=req))
    finally:
        engine.close();result['compare_count']=engine.compare_count;result['compare_failures']=engine.compare_failures
    return result

def main():
    out=ROOT/'outputs/live-missile';budget=OutputBudget(out,'canonical-reference');began=time.perf_counter()
    result=dict(workers=4,cases=0,equal=0,compare_count=0,compare_failures=0,missiles=[])
    with ProcessPoolExecutor(max_workers=4) as pool:
        for row in pool.map(check_missile,MISSILES):
            result['missiles'].append(row)
            for key in ['cases','equal','compare_count','compare_failures']:result[key]+=row[key]
            budget.json(out/'canonical-reference.json',result)
            print(json.dumps({k:v for k,v in row.items() if k!='failures'}),flush=True)
    result['elapsed_s']=time.perf_counter()-began;result['complete']=True;budget.json(out/'canonical-reference.json',result);budget.finish()
    if result['cases']!=720 or result['equal']!=720 or result['compare_count']!=720 or result['compare_failures']:raise SystemExit(1)

if __name__=='__main__':main()
