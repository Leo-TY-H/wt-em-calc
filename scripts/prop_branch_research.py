"""Native propeller branch probes using the complete configuration loader.

Prescribed healthy, connected running shaft states, high ground clearance,
48 Hz. These are executable observations, not independent model validation
or reachable whole-aircraft operating points.
"""
import json
import math
from collections import Counter
from pathlib import Path
from unicorn import UC_HOOK_CODE
from prop_config_native import PropConfigNative
from propeller_native import PropellerNative, STATE
from component_assembly import f32,mul

ROOT=Path(__file__).resolve().parents[1]

def main():
    census=json.loads((ROOT/'references/prop-native-config.json').read_text())
    catalog=json.loads((ROOT/'analysis/prop-catalog-mechanisms.json').read_text())
    names=list(catalog['proposed_feature_cover'])
    for row in census['aircraft']:
        if any(p['coaxial'] for p in row['propellers']) and row['aircraft'] not in names:names.append(row['aircraft'])
    reports=[];failures=[]
    for name in names:
        m=PropConfigNative();a=m.load_config(json.loads((ROOT/'references/jet-catalog/fm'/(name+'.blkx')).read_text()))
        config=m.describe(a);p=config['propellers'][0]
        inst=a+0x2491c;prop=a+0x21218+p['type_id']*0x370
        link=next(q for t in config['transmissions'] for q in t['propellers'] if q['index']==0)
        m.qword(STATE,inst);m.qword(STATE+8,prop)
        m.p={'pitch_min':p['pitch_min'],'reduction':link['ratio']}
        m.u.hook_add(UC_HOOK_CODE,lambda u,a,n,d:PropellerNative.blade_capture(m,u,a,n,d),begin=0x101a07790,end=0x101a07790)
        modes=([False] if p['manual'] or not p['automatic'] else [])+([True] if p['automatic'] else [])
        cases=[];call_counts=Counter();steps=0
        try:
            for automatic in modes:
                for speed in [0.,50.,150.,220.]:
                    state={'pitch':f32((p['pitch_min']+p['pitch_max'])/2)}
                    first=None
                    kw=dict(velocity=(speed,0.,0.),omega=mul(p['max_omega'],link['ratio']),
                            target_omega=p['max_omega'],auto=automatic,command=1.,dt=f32(1/48))
                    for j in range(48):
                        actual=PropellerNative.step(m,state,**kw)
                        if not all(math.isfinite(v) for v in actual['outputs']+actual['flow']+[actual['pitch']]):
                            raise ValueError('nonfinite propeller output')
                        steps+=1;call_counts[len(actual['blade_inputs'])]+=1
                        snapshot={k:v for k,v in actual.items() if k not in ['state','blade_inputs']}
                        snapshot['blade_evaluations']=len(actual['blade_inputs'])
                        if j==0:first=snapshot
                        state=actual['state']
                    cases.append(dict(automatic=automatic,speed=speed,shaft_omega=kw['omega'],first=first,after_48_steps=snapshot))
            reports.append(dict(aircraft=name,properties=p,reduction=link['ratio'],steps=steps,
                                blade_evaluation_counts=dict(sorted(call_counts.items())),cases=cases))
            print(name,'steps',steps,'blade calls',dict(call_counts),flush=True)
        except Exception as e:
            failures.append(dict(aircraft=name,error=str(e)));print(name,'FAILED',str(e),flush=True)
    report=dict(status='FAIL' if failures else 'PASS',binary_sha256=m.sha,scope=__doc__,
                total_steps=sum(x['steps'] for x in reports),aircraft=reports,failures=failures)
    (ROOT/'analysis/prop-branch-research.json').write_text(json.dumps(report,indent=2)+'\n')
    print(report['status'],report['total_steps'],len(reports),flush=True)
    if failures:raise SystemExit(1)

if __name__=='__main__':main()
