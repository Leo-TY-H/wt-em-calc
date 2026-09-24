"""Current-runtime replay of every catalog smoke point, with fresh recovery.

Each prior valid point supplies only a numerical start and fixed delivered
controls. Canonically initialized propulsion and the complete current aircraft
equations determine acceptance. Previously unresolved cases get a fresh search.
"""
import json,time,traceback,hashlib
from concurrent.futures import ProcessPoolExecutor,as_completed
from pathlib import Path
from em_solver import TrimSolver
from audit_prop_em import probe

ROOT=Path(__file__).resolve().parents[1]


def replay(old):
    name=old['aircraft'];p=old.get('point')
    if not p or not p['propulsion']['period_frames']:return probe(name,False)
    started=time.monotonic();row=dict(aircraft=name,fm_id=old['fm_id'],speed_kmh=old['speed_kmh'])
    try:
        s=TrimSolver(name,dict(aircraft=[name],altitude_m=1800.)).with_prop_controls(p['propulsion']['controls'])
        result=s.solve(p['speed_kmh'],p['load_g'],p['solution'],exhaustive=False)
        row.update(status='PASS' if result['valid'] else 'UNRESOLVED',point=result,
            replay_ps_error=abs(s.point_value(result)['ps']-result['ps_mps']),mass=s.mass['mass'])
    except Exception as error:row.update(status='ERROR',error=repr(error),traceback=traceback.format_exc())
    row['seconds']=time.monotonic()-started;return row


def main():
    source=ROOT/'analysis/prop-integration/em-catalog-audit.json';old=json.loads(source.read_text());rows=[]
    assert old['complete']
    with ProcessPoolExecutor(max_workers=4) as pool:
        for future in as_completed([pool.submit(replay,row) for row in old['records']]):
            row=future.result();rows.append(row)
            print(row['aircraft'],row['status'],round(row['seconds'],2),row.get('error',row.get('point',{}).get('reasons')),flush=True)
            counts={k:sum(r['status']==k for r in rows) for k in ['PASS','UNRESOLVED','ERROR']}
            (ROOT/'analysis/prop-integration/em-current-catalog-validation.json').write_text(json.dumps(dict(
                counts=counts,complete=len(rows)==len(old['records']),records=rows,
                source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),scope=__doc__),indent=2)+'\n')
    print(counts,flush=True)


if __name__=='__main__':main()
