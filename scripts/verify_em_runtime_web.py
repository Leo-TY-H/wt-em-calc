"""Fresh full-range HTTP calculations and on-demand scientific downloads."""
import json,time
from pathlib import Path
from urllib.request import Request,urlopen
from em_solver import settings

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'analysis/performance-pass'
URL='http://127.0.0.1:8765'


def fetch(path,body=None,raw=False):
    request=Request(URL+path,data=None if body is None else json.dumps(body).encode(),
                    headers={'Content-Type':'application/json'})
    with urlopen(request,timeout=90) as r:return r.read() if raw else json.load(r)


def main():
    rows=[]
    for mode in [False,True]:
        cfg=settings(dict(aircraft=['fa_18e_block_2'],instructor=mode))
        start=time.monotonic();key=fetch('/api/jobs',cfg)['id'];last=0.;fresh=None
        while True:
            status=fetch('/api/jobs/'+key)
            if fresh is None:fresh=status['status']!='complete'
            if status['status'] in ('complete','error','cancelled'):break
            now=time.monotonic()
            if now-last>20:
                print(mode,status['status'],status.get('progress'),flush=True);last=now
            if now-start>1200:raise TimeoutError(status)
            time.sleep(.2)
        assert status['status']=='complete',status
        ready=time.monotonic()-start;directory=ROOT/'outputs/em'/key
        before=list(directory.glob('diagram.*'))
        t=time.monotonic();chart=fetch('/api/jobs/'+key+'/chart.json');chart_seconds=time.monotonic()-t
        data=json.loads((directory/'data.json').read_text());a=data['aircraft'][0]
        assert not a['numerical_gaps'] and not a['numerical_boundaries'] and not a['instructor_unresolved_speeds_kmh']
        assert all(c['boundary_status']=='verified limit' for c in a['columns'])
        assert data['settings']['surface_resolution']==601
        assert (directory/'ready.json').is_file()
        t=time.monotonic();again=fetch('/api/jobs',cfg)
        assert again['id']==key and fetch('/api/jobs/'+key)['status']=='complete'
        cached_seconds=time.monotonic()-t
        row=dict(instructor=mode,job=key,fresh_calculation=fresh,compute_seconds=data['elapsed_s'],ready_seconds=ready,
            chart_fetch_seconds=chart_seconds,cached_request_seconds=cached_seconds,
            workers=data['workers'],backend=data['backend'],columns=len(a['columns']),
            points=len(a['points']),unresolved=0,gaps=0,
            figures_before_request=[p.name for p in before])
        rows.append(row);(OUT/'web-runtime-validation.json').write_text(json.dumps(dict(rows=rows),indent=2)+'\n')
        print('COMPLETE',row,flush=True)
    # Trigger downloads only after both interactive timing measurements.
    downloads=[];key=rows[0]['job']
    for extension in ['pdf','png','svg']:
        start=time.monotonic();payload=fetch('/api/jobs/'+key+'/diagram.'+extension,raw=True)
        assert (payload.startswith(b'%PDF') if extension=='pdf' else
                payload.startswith(b'\x89PNG') if extension=='png' else b'<svg' in payload[:1000])
        downloads.append(dict(format=extension,bytes=len(payload),seconds=time.monotonic()-start))
    assert not list((ROOT/'outputs/em'/key).glob('*.tmp.*'))
    (OUT/'web-runtime-validation.json').write_text(json.dumps(dict(rows=rows,downloads=downloads,failures=[]),indent=2)+'\n')
    print('DOWNLOADS',downloads,flush=True)


if __name__=='__main__':main()
