"""Full-column regression inventory, preserving every failed state for recovery."""
import argparse,json,time,multiprocessing
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor,as_completed
from em_solver import settings
from em_sampling import sample_column

def one(task):
    name,mode,speed=task
    cfg=settings(dict(aircraft=[name],instructor=mode))
    col=sample_column((name,json.dumps(cfg,sort_keys=True),speed,None))
    path=Path('analysis/gap-resolution/columns');path.mkdir(exist_ok=True)
    (path/f'{name}-{int(mode)}-{speed:g}.json').write_text(json.dumps(col)+'\n')
    return dict(aircraft=name,instructor=mode,speed_kmh=speed,status=col['boundary_status'],
        boundary_load=col['boundary']['load_g'] if col['boundary'] else None,
        gap_count=len(col['numerical_gap_brackets']),gaps=col['numerical_gap_brackets'],
        unresolved_intervals=col['unresolved_load_intervals'],seconds=col['elapsed_s'])

def main():
    p=argparse.ArgumentParser();p.add_argument('--aircraft',default='f_16a_block_15_adf,saab_jas39c,f_16xl')
    p.add_argument('--speeds',default='150,250,400,480,490,500,700,900,1100,1300')
    p.add_argument('--modes',default='0,1');p.add_argument('--output',default='analysis/gap-resolution/column-audit.json');args=p.parse_args()
    tasks=[(n,bool(int(m)),float(v)) for n in args.aircraft.split(',') for m in args.modes.split(',') for v in args.speeds.split(',')]
    rows=[];start=time.monotonic()
    with ProcessPoolExecutor(max_workers=4,mp_context=multiprocessing.get_context('spawn')) as pool:
        for f in as_completed([pool.submit(one,t) for t in tasks]):
            r=f.result();rows.append(r)
            Path(args.output).write_text(json.dumps(dict(rows=rows,seconds=time.monotonic()-start),indent=2)+'\n')
            print(len(rows),'/',len(tasks),r['aircraft'],r['instructor'],r['speed_kmh'],r['status'],'gaps',r['gap_count'],flush=True)

if __name__=='__main__':main()
