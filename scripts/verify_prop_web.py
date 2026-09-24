"""Local HTTP integration and exports for the propeller EM website."""
import argparse,json,time
from pathlib import Path
from urllib.request import Request,urlopen

ROOT=Path(__file__).resolve().parents[1];URL='http://127.0.0.1:8766'


def fetch(path,body=None,raw=False):
    r=Request(URL+path,data=None if body is None else json.dumps(body).encode(),headers={'Content-Type':'application/json'})
    with urlopen(r,timeout=60) as response:return response.read() if raw else json.load(response)


def main():
    global URL
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url',default=URL)
    parser.add_argument('--speed-min',type=float,default=300.)
    parser.add_argument('--speed-max',type=float,default=420.)
    parser.add_argument('--max-load',type=float,default=3.)
    parser.add_argument('--report',default='analysis/prop-integration/web-validation.json')
    args=parser.parse_args();URL=args.url
    meta=fetch('/api/meta');props={k:v for k,v in meta['aircraft'].items() if v['propulsion']!='jet'}
    assert props['yak-3']['supported'] and props['bf-109f-4']['supported']
    assert props['wyvern_s4']['propulsion']=='turboprop' and props['fr_1_fireball']['propulsion']=='mixed'
    assert not props['yak-3_test']['supported']
    # A bounded adaptive chart exercises the production HTTP/export path.
    # Physical stall-edge equations have their own independent native check.
    cfg=dict(aircraft=['yak-3','bf-109f-4'],speed_min_kmh=args.speed_min,speed_max_kmh=args.speed_max,max_load_g=args.max_load,
             speed_samples=7,load_samples=5,sep_tolerance_mps=1.,surface_resolution=201)
    start=time.monotonic();key=fetch('/api/jobs',cfg)['id'];last=0.
    while True:
        status=fetch('/api/jobs/'+key)
        if status['status'] in ['complete','error','cancelled']:break
        if time.monotonic()-last>20:print(status,flush=True);last=time.monotonic()
        if time.monotonic()-start>2400:raise TimeoutError(status)
        time.sleep(.5)
    assert status['status']=='complete',status
    data=fetch('/api/jobs/'+key+'/data.json');chart=fetch('/api/jobs/'+key+'/chart.json')
    for a in data['aircraft']:
        points=[p for p in a['points'] if p['valid']];assert points,a['id']
        assert all(p['propulsion']['phase_check']['checked'] for p in points)
        assert all(p['force_error_g']<=2e-4 and p['angular_error_rad_s2']<=5e-5 for p in points)
        assert all(not p['propulsion']['optimization']['global_optimum_certified'] for p in points)
    assert len(chart['aircraft'])==2
    csv=fetch('/api/jobs/'+key+'/samples.csv',raw=True).decode();assert 'propulsion' in csv.splitlines()[0]
    for extension,signature in [('svg',b'<svg'),('pdf',b'%PDF'),('png',b'\x89PNG')]:
        content=fetch('/api/jobs/'+key+'/diagram.'+extension,raw=True);assert signature in content[:1000]
    assert fetch('/api/jobs',cfg)['id']==key
    # Cancellation must interrupt an active propeller search promptly, without
    # replacing the completed comparison or leaving CPU workers behind.
    cancel=fetch('/api/jobs',dict(cfg,altitude_m=3200.,speed_samples=49))['id'];time.sleep(2.)
    queued=fetch('/api/jobs',dict(cfg,altitude_m=3600.,speed_samples=49))['id']
    fetch('/api/jobs/'+queued+'/cancel',{})
    assert fetch('/api/jobs/'+queued)['status']=='cancelled'
    at=time.monotonic();fetch('/api/jobs/'+cancel+'/cancel',{})
    while time.monotonic()-at<15.:
        state=fetch('/api/jobs/'+cancel)
        if state['status']=='cancelled':break
        time.sleep(.1)
    assert state['status']=='cancelled',state
    assert fetch('/api/meta')['latest']==key
    report=dict(status='PASS',job=key,settings=cfg,url=URL,seconds=time.monotonic()-start,cancel_seconds=time.monotonic()-at,
        propeller_variants=len(props),available=sum(p['supported'] for p in props.values()),
        checks=['Family metadata and missing-geometry exclusion',f'Fresh Yak-3/Bf 109 F-4 adaptive comparison, {args.speed_min}–{args.speed_max} km/h and 1–{args.max_load} g',
                'Cycle-mean trim tolerances','CSV and JSON contain engine controls','SVG/PNG/PDF exports',
                'Completed-job reuse','Queued cancellation is immediate','Cooperative cancellation preserves completed chart'])
    (ROOT/args.report).write_text(json.dumps(report,indent=2)+'\n');print(report,flush=True)


if __name__=='__main__':main()
