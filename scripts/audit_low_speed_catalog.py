"""Shared-solver low-speed column audit across the complete supported jet catalog."""
import json,multiprocessing,time
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor,as_completed
from em_solver import AIRCRAFT,settings
from em_sampling import sample_column


def one(task):
    name,mode,speed=task
    cfg=settings(dict(aircraft=[name],instructor=mode))
    try:
        c=sample_column((name,json.dumps(cfg,sort_keys=True),speed,None))
        path=Path('analysis/sideslip-integration/catalog-columns')
        path.mkdir(exist_ok=True)
        (path/f'{name}-{int(mode)}-{speed:g}.json').write_text(json.dumps(c)+'\n')
        return dict(aircraft=name,instructor=mode,speed_kmh=speed,status=c['boundary_status'],
                    gap_count=len(c['numerical_gap_brackets']),gaps=c['numerical_gap_brackets'],
                    boundary_load=c['boundary']['load_g'] if c['boundary'] else None,seconds=c['elapsed_s'])
    except Exception as error:
        return dict(aircraft=name,instructor=mode,speed_kmh=speed,status='exception',error=repr(error))


def main():
    tasks=[(name,mode,250.) for name,info in AIRCRAFT.items() if info['supported'] for mode in [False,True]]
    rows=[];start=time.monotonic()
    with ProcessPoolExecutor(max_workers=6,mp_context=multiprocessing.get_context('spawn')) as pool:
        for future in as_completed([pool.submit(one,t) for t in tasks]):
            row=future.result();rows.append(row)
            Path('analysis/sideslip-integration/catalog-low-speed-audit.json').write_text(json.dumps(
                dict(rows=rows,seconds=time.monotonic()-start,total=len(tasks)),indent=2)+'\n')
            if len(rows)%20==0 or row['status'] not in ['verified limit','no feasible samples'] or row.get('gap_count'):
                print(len(rows),'/',len(tasks),row,flush=True)


if __name__=='__main__':main()
