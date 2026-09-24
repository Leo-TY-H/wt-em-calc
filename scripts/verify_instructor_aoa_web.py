"""Full default plots through HTTP, including static Instructor transitions."""
import json,time,math
from pathlib import Path
import numpy as np
from em_solver import settings,TrimSolver
from em_sampling import speed_interpolate
from verify_em_runtime_web import fetch

ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'analysis/instructor-aoa'


def main():
    rows=[];failures=[]
    for name,mode in [('fa_18e_block_2',False),('fa_18e_block_2',True),('a_4b',True),('ariete',True)]:
        cfg=settings(dict(aircraft=[name],instructor=mode));start=time.monotonic()
        key=fetch('/api/jobs',cfg)['id'];last=0.
        while True:
            status=fetch('/api/jobs/'+key)
            if status['status'] in ('complete','error','cancelled'):break
            now=time.monotonic()
            if now-last>20:print(name,mode,status['status'],status.get('progress'),flush=True);last=now
            if now-start>1200:raise TimeoutError(status)
            time.sleep(.2)
        assert status['status']=='complete',status
        ready=time.monotonic()-start
        data=json.loads((ROOT/'outputs/em'/key/'data.json').read_text());a=data['aircraft'][0]
        chart=fetch('/api/jobs/'+key+'/chart.json')
        row=dict(aircraft=name,instructor=mode,job=key,ready_seconds=ready,compute_seconds=data['elapsed_s'],
            columns=len(a['columns']),unresolved=len(a['numerical_boundaries']),gaps=len(a['numerical_gaps']),
            boundary_statuses={s:sum(c['boundary_status']==s for c in a['columns']) for s in set(c['boundary_status'] for c in a['columns'])},
            low_speed_edge=a.get('low_speed_edge'),backend=data['backend'])
        if row['unresolved'] or row['gaps']:failures.append(dict(stage='unresolved/gaps',**row))
        solver=TrimSolver(name,cfg);holdouts=[];columns=a['columns']
        pairs=[(l,r) for l,r in zip(columns,columns[1:]) if l.get('boundary') and r.get('boundary')]
        for i in np.linspace(1,len(pairs)-2,12,dtype=int):
            left,right=pairs[i];speed=.61*left['speed_kmh']+.39*right['speed_kmh']
            cap=min(left['boundary']['load_g'],right['boundary']['load_g'])
            floor=max(c['lower_boundary']['load_g'] if c.get('lower_boundary') else 1. for c in [left,right])
            for fraction in [.23,.67,.9]:
                load=floor+fraction*(cap-floor);pred=float(speed_interpolate(columns,[speed],[load])[0,0])
                p=solver.solve(speed,load,left['boundary']['solution']);error=abs(p['ps_mps']-pred)
                entry=dict(speed_kmh=speed,load_g=load,valid=p['valid'],error_mps=error if math.isfinite(error) else None)
                if not p['valid'] or not math.isfinite(error) or error>cfg['sep_tolerance_mps']:failures.append(dict(stage='surface holdout',aircraft=name,instructor=mode,**entry))
                holdouts.append(entry)
        row['holdouts']=holdouts;row['max_ps_error_mps']=max((h['error_mps'] or 0.) for h in holdouts)
        if name=='fa_18e_block_2' and not mode:
            old=json.loads((ROOT/'outputs/em/5adb8d079d8c488b98e6/data.json').read_text())['aircraft'][0]
            before={c['speed_kmh']:c for c in old['columns']}
            deltas=[abs(c['boundary']['turn_dps']-before[c['speed_kmh']]['boundary']['turn_dps']) for c in columns
                if c.get('boundary') and c['speed_kmh'] in before and before[c['speed_kmh']].get('boundary')]
            row['off_max_boundary_change_dps']=max(deltas)
            if max(deltas)>1e-10:failures.append(dict(stage='off boundary changed',delta=max(deltas)))
        rows.append(row)
        (OUT/'web-validation.json').write_text(json.dumps(dict(rows=rows,failures=failures),indent=2)+'\n')
        print('COMPLETE',{k:v for k,v in row.items() if k not in ['holdouts','low_speed_edge']},flush=True)
    png=fetch('/api/jobs/'+rows[1]['job']+'/diagram.png',raw=True)
    assert png.startswith(b'\x89PNG')
    assert not failures,failures


if __name__=='__main__':main()
