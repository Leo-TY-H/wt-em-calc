"""Automatic propeller envelope columns and exported physical-limit points."""
import json,time
from pathlib import Path
from em_solver import settings,TrimSolver
from em_sampling import sample_column

ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'analysis/prop-integration'


def main():
    records=[];points=[]
    for name in ['yak-3','bf-109f-4']:
        cfg=settings(dict(aircraft=[name],speed_min_kmh=300.,speed_max_kmh=420.,
            speed_samples=7,load_samples=5,sep_tolerance_mps=1.,surface_resolution=201))
        t=time.monotonic();column=sample_column((name,json.dumps(cfg),300.,None))
        boundary=column['boundary']
        assert boundary and boundary['valid'] and boundary.get('envelope_limit'),column['boundary_status']
        checked=[boundary]+column['sustained'];solver=TrimSolver(name,cfg)
        for point in checked:
            assert point['valid'] and abs(solver.point_value(point)['ps']-point['ps_mps'])<1e-9
            if point is not boundary:assert abs(point['ps_mps'])<.02
            points.append(dict(point,aircraft=name,settings={}))
        (OUT/(name+'-automatic-column.json')).write_text(json.dumps(column,indent=2)+'\n')
        records.append(dict(aircraft=name,seconds=time.monotonic()-t,
            boundary=boundary['envelope_limit'],sustained_roots=len(column['sustained']),
            points=len(column['points']),boundary_status=column['boundary_status']))
        print(records[-1],flush=True)
    (OUT/'boundary-current.json').write_text(json.dumps(points,indent=2)+'\n')
    (OUT/'boundary-validation.json').write_text(json.dumps(dict(status='PASS',records=records,
        scope='Two automatic 300 km/h envelope columns; every returned limit/root replays exactly.'),indent=2)+'\n')


if __name__=='__main__':main()
