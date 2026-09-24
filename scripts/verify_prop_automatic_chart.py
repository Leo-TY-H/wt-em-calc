"""Exercise the actual default speed range and automatically solved envelope."""
import json,time
from urllib.request import urlopen,Request
from pathlib import Path
OUT=Path(__file__).resolve().parents[1]/'analysis/prop-performance'
URL='http://127.0.0.1:8766'
def fetch(path,body=None):
    with urlopen(Request(URL+path,data=None if body is None else json.dumps(body).encode(),headers={'Content-Type':'application/json'}),timeout=60) as r:return json.load(r)
if __name__=='__main__':
    cfg=dict(aircraft=['yak-3','bf-109f-4']);started=time.monotonic();key=fetch('/api/jobs',cfg)['id'];last=0
    while True:
        s=fetch('/api/jobs/'+key)
        if time.monotonic()-last>20:print(s['status'],s.get('progress'),flush=True);last=time.monotonic()
        if s['status'] in ['complete','cancelled','error']:break
        time.sleep(1)
    assert s['status']=='complete',s
    data=fetch('/api/jobs/'+key+'/data.json');rows=[]
    for a in data['aircraft']:
        good=[p for p in a['points'] if p['valid']];assert good
        assert all(p['propulsion']['engine_control_mode']=='automatic' and p['propulsion']['optimization']['evaluated']==0 for p in good)
        assert all(p['force_error_g']<=2e-4 and p['angular_error_rad_s2']<=5e-5 for p in good)
        rows.append(dict(aircraft=a['id'],valid=len(good),columns=len(a['columns']),boundary_statuses={str(c['boundary_status']):sum(c['boundary_status']==other['boundary_status'] for other in a['columns']) for c in a['columns']}))
    report=dict(status='PASS',job=key,seconds=time.monotonic()-started,settings=data['settings'],aircraft=rows)
    (OUT/'default-chart.json').write_text(json.dumps(report,indent=2)+'\n');print(report,flush=True)
