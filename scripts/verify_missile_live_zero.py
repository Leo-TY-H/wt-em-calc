"""Zero steering against the retained release, across every corpus setup."""
from copy import deepcopy
import json
import os
os.environ['WT_MISSILE_ENGINE']='python'
os.environ['WT_MISSILE_BACKEND']='reference'
from missile_worker import quaternion
from benchmark_output import OutputBudget
from verify_missile_live import Engine,ROOT,MISSILES,logs,scenario,canonical_equal

def main():
    out=ROOT/'outputs/live-missile';budget=OutputBudget(out,'zero-input')
    engines=[Engine([str(base/'.native_missile_engine/missile-engine.exe'),'--root',str(ROOT),'--mode','simulator','--batch'],dict(os.environ,WT_MISSILE_TYPED='1')) for base in [ROOT.parent/'WT-FM-app',ROOT]]
    result=dict(cases=0,baseline_equal=0,zero_equal=0,quaternions_equal=0,failures=[])
    try:
        for missile in MISSILES:
            for case,log in enumerate(logs()):
                req=scenario(missile,case,log);del req['target_control']
                before,after=[e.request(req) for e in engines];controlled=deepcopy(req);controlled['target_control']=dict(load_g=9.,inputs=[])
                zero=engines[1].request(controlled);q0=quaternion(req['target']['angles']);all_q=True
                del zero['scenario']['target_control'];zero['model']['target_motion']='constant_velocity'
                for row in zero['trajectory']+[zero['final_body_sample']]:all_q&=row.pop('target_quaternion')==q0
                result['cases']+=1;result['baseline_equal']+=canonical_equal(before,after);result['zero_equal']+=canonical_equal(before,zero);result['quaternions_equal']+=all_q
                if (not canonical_equal(before,after) or not canonical_equal(before,zero) or not all_q) and len(result['failures'])<10:result['failures'].append(dict(missile=missile,case=case))
    finally:
        for engine in engines:engine.close()
        budget.json(out/'zero-input.json',result);budget.finish()
    print(json.dumps(result))
    if any(result[k]!=result['cases'] for k in ['baseline_equal','zero_equal','quaternions_equal']):raise SystemExit(1)

if __name__=='__main__':main()
