"""Native complete configuration-loader census for pinned fixed-wing props.

The original loader chooses types, native defaults, mounting and drivetrain
links. Successful loading is not an independent dynamics validation.
"""
import argparse
import hashlib
import json
from collections import Counter, defaultdict
from pathlib import Path
from prop_config_native import PropConfigNative
from collision_native import HEAP

ROOT=Path(__file__).resolve().parents[1]

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--cover',action='store_true')
    args=parser.parse_args()
    catalog=json.loads((ROOT/'analysis/prop-catalog-mechanisms.json').read_text())
    names=catalog['proposed_feature_cover'] if args.cover else [x['aircraft'] for x in catalog['aircraft']]
    expected={x['aircraft']:x['sha256'] for x in catalog['aircraft']}
    m=None;rows=[];failures=[];counts=defaultdict(Counter)
    for i,name in enumerate(names):
        if m is None or m.cursor-HEAP > 24000000:m=PropConfigNative()
        path=ROOT/'references/jet-catalog/fm'/(name+'.blkx');raw=path.read_bytes()
        assert hashlib.sha256(raw).hexdigest()==expected[name]
        try:
            a=m.load_config(json.loads(raw));result=m.describe(a)
            rows.append(dict(aircraft=name,**result))
            for p in result['propellers']:
                assert not p['cyclic'],name
            for key in ['governor','fast','airflow_solver','coaxial','manual','automatic','feathering','differential_pitch','pitch_1d_rows','pitch_2d_rows']:
                for value in set(str(p[key]) for p in result['propellers']):counts[key][value]+=1
            if args.cover or (i+1)%50==0:print(i+1,name,'loaded',flush=True)
        except Exception as e:
            failures.append(dict(aircraft=name,error=str(e)));print(name,'FAILED',str(e),flush=True)
            m=None
    report=dict(status='FAIL' if failures else 'PASS',binary_sha256=m.sha if m else None,
                commit=catalog['commit'],requested=len(names),loaded=len(rows),
                scope=__doc__,counts={k:dict(v) for k,v in counts.items()},aircraft=rows,failures=failures)
    suffix='-cover' if args.cover else ''
    output=ROOT/('analysis/prop-native-config'+suffix+'.json');output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ['aircraft','scope','counts']},indent=2))
    if failures:raise SystemExit(1)

if __name__=='__main__':main()
