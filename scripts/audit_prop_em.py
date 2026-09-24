"""Catalog-wide installed-mass and balanced-flight smoke checks.

Native differential kernel reports supply equation evidence separately. A
single equilibrium is not validation of an entire aircraft envelope.
"""
import argparse,json,math,time,traceback
from concurrent.futures import ProcessPoolExecutor,as_completed
from pathlib import Path
from em_solver import AIRCRAFT,TrimSolver
from prop_catalog import catalog

ROOT=Path(__file__).resolve().parents[1]


def probe(name,coupled):
    started=time.monotonic();row=dict(aircraft=name,fm_id=catalog()[name]['fm_id'])
    AIRCRAFT[name]['supported']=True
    try:
        s=TrimSolver(name,dict(aircraft=[name],altitude_m=1800.))
        speed=min(s.model['geometry']['strength']['ias']*.72,
                  math.sqrt(2*s.weight*1.5/(1.03*s.model['geometry']['area']*.40)))*3.6
        speed=max(180.,min(750.,speed));row['speed_kmh']=speed
        if not coupled:
            seed=s.engine.seed_controls(speed/3.6)
            s=s.with_prop_controls(seed['controls']);s.engine.no_resolved_seed=any(not g['candidates'] for g in seed['selections'])
        p=s.solve(speed,1.5,exhaustive=False)
        row.update(status='PASS' if p['valid'] else 'UNRESOLVED',point=p,
                   replay_ps_error=abs(s.point_value(p)['ps']-p['ps_mps']),mass=s.mass['mass'])
    except Exception as error:row.update(status='ERROR',error=repr(error),traceback=traceback.format_exc())
    row['seconds']=time.monotonic()-started
    return row


def main():
    ap=argparse.ArgumentParser();ap.add_argument('names',nargs='*');ap.add_argument('--coupled',action='store_true')
    ap.add_argument('--workers',type=int,default=4)
    ap.add_argument('--report',default='analysis/prop-integration/em-catalog-audit.json');args=ap.parse_args()
    selected={};missing=[]
    for name,row in catalog().items():
        if row.get('engine_count') is None:missing.append(dict(aircraft=name,fm_id=row['fm_id'],reason=row['reason']));continue
        selected.setdefault(row['fm_id'],name)
    names=args.names or list(selected.values());rows=[]
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        futures=[pool.submit(probe,name,args.coupled) for name in names]
        for future in as_completed(futures):
            row=future.result();rows.append(row)
            print(row['aircraft'],row['status'],round(row['seconds'],2),row.get('error',row.get('point',{}).get('reasons','')),flush=True)
            counts={k:sum(r['status']==k for r in rows) for k in ['PASS','UNRESOLVED','ERROR']}
            (ROOT/args.report).write_text(json.dumps(dict(counts=counts,complete=len(rows)==len(names),
                coupled_control_search=args.coupled,records=sorted(rows,key=lambda r:r['aircraft']),missing_geometry=missing,
                scope=__doc__),indent=2)+'\n')
    print(counts,flush=True)


if __name__=='__main__':main()
