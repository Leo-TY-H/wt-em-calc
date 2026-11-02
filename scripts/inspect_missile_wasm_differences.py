"""Attribute the retained corpus's non-identical fields without storing documents."""
import json
from pathlib import Path
from benchmark_output import OutputBudget
from verify_missile_live import Engine,ROOT,MISSILES,logs,scenario,canonical_equal


def differences(a,b,path=''):
    if isinstance(a,dict) and isinstance(b,dict):
        for key in a.keys()|b.keys():
            if key not in a or key not in b:yield path+'.'+key,a.get(key),b.get(key)
            else:yield from differences(a[key],b[key],path+'.'+key)
    elif isinstance(a,list) and isinstance(b,list):
        if len(a)!=len(b):yield path+'.length',len(a),len(b)
        for x,y in zip(a,b):yield from differences(x,y,path+'[]')
    elif not canonical_equal(a,b):yield path,a,b


def main():
    out=ROOT/'outputs/live-missile';budget=OutputBudget(out,'wasm-differences')
    linux='/mnt/'+ROOT.drive[0].lower()+ROOT.as_posix()[2:]
    engines={'windows':Engine([str(ROOT/'.native_missile_engine/missile-engine.exe'),'--root',str(ROOT),'--mode','simulator','--batch']),
             'linux':Engine(['wsl','-d','Ubuntu','--',linux+'/.linux_missile_engine/missile-engine','--root',linux,'--mode','simulator','--batch'])}
    wasm=Engine(['node',str(ROOT/'scripts/missile_wasm/run_node.mjs'),str(ROOT)],wrapped=False)
    result={name:dict(cases=0,value_equal=0,different_documents=0,fields={},samples=[]) for name in engines}
    try:
        for missile in MISSILES:
            for case,log in enumerate(logs()):
                req=scenario(missile,case,log);start=scenario(missile,case,[]);start['duration']=req['duration'];start['target_control']=dict(load_g=req['target_control']['load_g'],inputs=[])
                handle=wasm.request({'start':start})['handle'];step=0;terminal=False
                try:
                    for count,az,el,roll in log+[[14400,0,0,0]]:
                        for _ in range(count):
                            step+=1
                            if wasm.request({'step':[handle,az,el,roll,int(step==req['target_control'].get('stop_after_steps'))]})['status']==1:terminal=True;break
                        if terminal:break
                    doc=wasm.request({'result':handle})
                finally:wasm.request({'free':handle})
                for name,engine in engines.items():
                    native=engine.request(req);record=result[name];record['cases']+=1;record['value_equal']+=native==doc
                    if canonical_equal(native,doc):continue
                    record['different_documents']+=1
                    for field,a,b in differences(native,doc):
                        stat=record['fields'].setdefault(field,dict(count=0,max_absolute=0.));stat['count']+=1
                        if isinstance(a,(int,float)) and isinstance(b,(int,float)):stat['max_absolute']=max(stat['max_absolute'],abs(a-b))
                        else:stat['non_numeric']=True
                        if len(record['samples'])<10:record['samples'].append(dict(missile=missile,case=case,field=field,native=a,wasm=b))
    finally:
        for engine in [*engines.values(),wasm]:engine.close()
        budget.json(out/'wasm-differences.json',result);budget.finish()
    print(json.dumps(result))


if __name__=='__main__':main()
