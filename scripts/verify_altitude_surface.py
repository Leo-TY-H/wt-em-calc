"""Held-out actual trim checks along exported altitude contours."""
import argparse
import json
from pathlib import Path
from altitude_envelope import LevelSampler

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser();parser.add_argument('data');args=parser.parse_args()
    data=json.loads(Path(args.data).read_text());sampler=LevelSampler(data['settings']);rows=[]
    levels=data['contours']
    # Sample between contour vertices, not the mesh's original solved nodes.
    for contour in levels:
        if not contour['paths']:continue
        path=max(contour['paths'],key=len)
        if len(path)<4:continue
        for fraction in (.3,.7):
            index=min(len(path)-2,int((len(path)-1)*fraction))
            a,b=path[index:index+2];speed=(a[0]+b[0])/2;height=(a[1]+b[1])/2
            p=sampler(speed,height)
            error=abs(p['ps_mps']-contour['sep_mps']) if p['valid'] else None
            rows.append(dict(speed_kmh=speed,altitude_m=height,level_mps=contour['sep_mps'],valid=p['valid'],
                             reasons=p['reasons'],error_mps=error))
    # This is a sampled interpolation check, not a global proof. Retain all
    # failed probes verbatim instead of excluding them from the report.
    tolerance=data['sampling']['tolerance_mps']
    failures=[p for p in rows if not p['valid'] or p['error_mps']>tolerance]
    report=dict(status='FAIL' if failures else 'PASS',tolerance_mps=tolerance,rows=rows,failures=failures,
                max_error_mps=max((p['error_mps'] for p in rows if p['valid']),default=None))
    out=ROOT/'analysis/altitude-envelope';out.mkdir(parents=True,exist_ok=True)
    (out/'surface-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(report['status'],len(rows),'checks; maximum error',report['max_error_mps'],flush=True)
    assert not failures,failures


if __name__=='__main__':main()
