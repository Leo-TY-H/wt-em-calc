"""Time cold trim solves and automatic envelope columns without profiling overhead."""
import json,time
from pathlib import Path
from em_solver import TrimSolver,settings
from em_sampling import sample_column
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'analysis/prop-performance'

def main():
    records=[];points=[]
    for name in ['yak-3','bf-109f-4']:
        modes={}
        for mode in ['optimized','automatic']:
            cfg=dict(aircraft=[name],engine_control_mode=mode)
            s=TrimSolver(name,cfg);t=time.monotonic();p=s.solve(360.,2.,exhaustive=False);elapsed=time.monotonic()-t
            assert p['valid'],p['reasons'];assert abs(s.point_value(p)['ps']-p['ps_mps'])<1e-9
            modes[mode]=dict(seconds=elapsed,ps_mps=p['ps_mps']);print(name,mode,modes[mode],flush=True)
        records.append(dict(aircraft=name,modes=modes,speedup=modes['optimized']['seconds']/modes['automatic']['seconds']))
        cfg=settings(dict(aircraft=[name],speed_min_kmh=300.,speed_max_kmh=420.,speed_samples=7,load_samples=5,sep_tolerance_mps=1.,surface_resolution=201))
        t=time.monotonic();column=sample_column((name,json.dumps(cfg),300.,None));elapsed=time.monotonic()-t
        boundary=column['boundary'];assert boundary and boundary['valid'] and boundary.get('envelope_limit'),column['boundary_status']
        s=TrimSolver(name,cfg)
        for p in [boundary]+column['sustained']:
            assert p['valid'] and abs(s.point_value(p)['ps']-p['ps_mps'])<1e-9
            if p is not boundary:assert abs(p['ps_mps'])<.02
            points.append(dict(p,aircraft=name,settings=dict(engine_control_mode='automatic')))
        (OUT/(name+'-automatic-column.json')).write_text(json.dumps(column,indent=2)+'\n')
        records[-1]['column']=dict(seconds=elapsed,boundary=boundary['envelope_limit'],points=len(column['points']),sustained_roots=len(column['sustained']))
        print(records[-1],flush=True)
        (OUT/'benchmark.json').write_text(json.dumps(dict(status='PASS',records=records),indent=2)+'\n')
        (OUT/'automatic-boundary-points.json').write_text(json.dumps(points,indent=2)+'\n')
if __name__=='__main__':main()
