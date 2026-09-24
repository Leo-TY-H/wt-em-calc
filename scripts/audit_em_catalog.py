"""Catalog-wide production boundary/interior audit, resumable per condition.

No aircraft-specific recovery. Full column mode additionally audits the dense
interpolation/gap machinery. Boundary mode covers 403 jets at three speeds.
"""
import argparse,json,time,multiprocessing,os
from concurrent.futures import ProcessPoolExecutor,as_completed
from pathlib import Path
from em_solver import TrimSolver,settings,AIRCRAFT
from em_sampling import sample_column

def run(task):
    name,speed,flaps,sweep,full=task;start=time.monotonic()
    try:
        cfg=settings(dict(aircraft=[name],flaps_percent=flaps,sweep_percent=sweep,sep_tolerance_mps=1.))
        if full:
            c=sample_column((name,json.dumps(cfg),speed,None))
            return dict(task=task,status=c['boundary_status'],boundary=c['boundary'],gaps=c['numerical_gap_brackets'],elapsed=time.monotonic()-start)
        s=TrimSolver(name,cfg);previous=None;points=[];boundary=None
        for load in [1.,2.,3.,4.05,5.4675,7.381125,9.964519,13.4521,18.16034,24.51646,33.0972,44.6813,60.3198,64.]:
            p=s.solve(speed,load,previous['solution'] if previous else None,exhaustive=False);points.append(p)
            if any(x in p['reasons'] for x in ['sweep unavailable','IAS limit','Mach limit']):break
            if not p['valid'] and previous:
                boundary=s.boundary(speed,previous,p)
                if boundary:break
                if p['stall_margin_deg']>1 and p['authority_margin']>.02 and max(p['wing_load_ratios'])<.98:continue
                break
            if p['valid']:previous=p
        status='verified limit' if boundary else 'unresolved' if previous else 'no feasible samples'
        if any(any(x in p['reasons'] for x in ['sweep unavailable','IAS limit','Mach limit']) for p in points):status='speed/sweep exclusion'
        return dict(task=task,status=status,boundary=boundary,points=points,elapsed=time.monotonic()-start)
    except Exception as exc:return dict(task=task,status='error',error=repr(exc),elapsed=time.monotonic()-start)

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',default='analysis/em-three-tasks/catalog-audit.jsonl');p.add_argument('--full',action='store_true');p.add_argument('--aircraft',default='');p.add_argument('--flaps',type=float,default=0);p.add_argument('--sweep',type=float,default=0);p.add_argument('--speeds',default='400,800,1200');p.add_argument('--retry-failed',action='store_true');a=p.parse_args()
    path=Path(a.output);previous={tuple(r['task']):r for r in map(json.loads,path.read_text().splitlines())} if path.exists() else {}
    seen={t for t,r in previous.items() if not a.retry_failed or r['status'] not in ['error','unresolved','unresolved numerical boundary']}
    names=a.aircraft.split(',') if a.aircraft else [name for name,item in AIRCRAFT.items() if item['supported']]
    tasks=[(name,float(speed),a.flaps,a.sweep,a.full) for name in names for speed in a.speeds.split(',')]
    todo=[t for t in tasks if t not in seen];counts={};start=time.monotonic()
    with ProcessPoolExecutor(max_workers=6,mp_context=multiprocessing.get_context('spawn')) as pool,path.open('a') as out:
        futures={pool.submit(run,t):t for t in todo}
        for i,f in enumerate(as_completed(futures)):
            row=f.result();out.write(json.dumps(row,separators=(',',':'))+'\n');out.flush()
            counts[row['status']]=counts.get(row['status'],0)+1
            if i%25==0:print(i+1,'/',len(todo),counts,'elapsed',round(time.monotonic()-start,1),flush=True)
    print('DONE',len(tasks),counts,flush=True)
if __name__=='__main__':main()
