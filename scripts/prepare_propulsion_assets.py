"""Versioned portable installation properties from the pinned native loader."""
import argparse,hashlib,json
from pathlib import Path
from prop_config_native import PropConfigNative
from propulsion_config import decode
ROOT=Path(__file__).resolve().parents[1]


def main():
    ap=argparse.ArgumentParser();ap.add_argument('names',nargs='*');args=ap.parse_args()
    census=json.loads((ROOT/'references/prop-native-config.json').read_text())
    out=ROOT/'references/prop-propulsion';out.mkdir(exist_ok=True)
    reports=[];failures=[]
    for row in census['aircraft']:
        name=row['aircraft']
        if args.names and name not in args.names:continue
        raw=(ROOT/'references/jet-catalog/fm'/(name+'.blkx')).read_bytes()
        try:
            m=PropConfigNative();p=decode(m,m.load_config(json.loads(raw)))
            result=dict(schema=1,aircraft=name,commit=census['commit'],binary_sha256=m.sha,
                fm_sha256=hashlib.sha256(raw).hexdigest(),properties=p,
                scope='Original property loading and value-only decoding. No emulated pointers at runtime; dynamics are independent Python/Cython ports.')
            path=out/(name+'.json');path.write_text(json.dumps(result,indent=2)+'\n')
            reports.append(dict(aircraft=name,sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
            print(name,'PASS',flush=True)
        except Exception as error:failures.append(dict(aircraft=name,error=repr(error)));print(name,repr(error),'FAIL',flush=True)
    (out/'manifest.json').write_text(json.dumps(dict(schema=1,commit=census['commit'],binary_sha256=census['binary_sha256'],records=reports,failures=failures),indent=2)+'\n')
    print('PASS' if not failures else 'FAIL',len(reports),'assets',len(failures),'failures')
    if failures:raise SystemExit(1)

if __name__=='__main__':main()
