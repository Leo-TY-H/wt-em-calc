"""Collect individually checked native mass artifacts without losing reruns."""
import hashlib,json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]


def main():
    records=[]
    for path in sorted((ROOT/'references/prop-mass').glob('*.json')):
        if path.name=='manifest.json':continue
        a=json.loads(path.read_text())
        assert a['consumer_checks']==4,(path,'incomplete consumer check')
        assert 'complete mass-property loader' in a['scope'],(path,'obsolete loader')
        fm=ROOT/'references/jet-catalog/fm'/(a['aircraft']+'.blkx')
        assert hashlib.sha256(fm.read_bytes()).hexdigest()==a['fm_sha256']
        records.append(dict(aircraft=a['aircraft'],resource=a['resource'],path=str(path.relative_to(ROOT)),
            sha256=hashlib.sha256(path.read_bytes()).hexdigest(),consumer_checks=a['consumer_checks'],
            binary_sha256=a['binary_sha256'],collision_stream_sha256=a['collision_provenance']['stream_sha256']))
    report=dict(status='PASS',artifact_count=len(records),fm_count=len({r['aircraft'] for r in records}),
        consumer_checks=sum(r['consumer_checks'] for r in records),records=records,
        scope='Manifest of individually passing artifacts, including corrected reruns. Complete native collision and mass-property loading, independent priority fuel allocation and intact mass consumer at 0/30/70/100% internal fuel. Geometry and FM versions separately pinned.')
    (ROOT/'references/prop-mass/manifest.json').write_text(json.dumps(report,indent=2)+'\n')
    print({k:v for k,v in report.items() if k!='records'})


if __name__=='__main__':main()
