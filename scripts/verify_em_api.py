"""Exercise a real local calculation, exports, cache reuse and cancellation."""
import csv
import io
import json
import time
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import Request, urlopen

ROOT=Path(__file__).resolve().parents[1]
BASE='http://127.0.0.1:8765'


def request(path, data=None, raw=False):
    req=Request(BASE+path, data=None if data is None else json.dumps(data).encode(),
                headers={'Content-Type':'application/json'})
    with urlopen(req, timeout=10) as response:
        value=response.read()
    return value if raw else json.loads(value)


def terminal(key):
    states=[];deadline=time.monotonic()+180
    while time.monotonic()<deadline:
        job=request('/api/jobs/'+key)
        if job['status'] not in states:states.append(job['status'])
        if job['status'] in ('complete','cancelled','error'):return job,states
        time.sleep(.25)
    raise AssertionError('Job did not finish within 180 seconds')


def main():
    checks=[]
    latest_path=ROOT/'outputs/em/latest.json'
    previous_latest=latest_path.read_text() if latest_path.exists() else None
    # Unique mass makes this an uncached calculation on repeated test runs.
    config=dict(aircraft=['f_16a_block_15_adf'],altitude_m=4500.,fuel_percent=50.,
                afterburner=False,throttle=1.,trim_mode='fixed',fixed_trim=[0.,0.,0.],
                speed_samples=7,load_samples=5,speed_min_kmh=900.,speed_max_kmh=1100.,
                max_load_g=3.,sampling='regular',extra_mass_kg=100.+(time.time_ns()%1000000)/1000000.)
    key=request('/api/jobs',config)['id'];job,states=terminal(key)
    assert job['status']=='complete',job
    assert 'running' in states or 'exporting' in states
    data=request(f'/api/jobs/{key}/data.json')
    assert len(data['aircraft'][0]['points'])==35
    assert data['aircraft'][0]['valid_points']==35
    assert all('trim' not in p and 'sticks' not in p for p in data['aircraft'][0]['points'])
    chart=request(f'/api/jobs/{key}/chart.json')
    assert 'surface' not in chart['aircraft'][0] and 'component_forces' not in chart['aircraft'][0]['points'][0]
    detail=request(f'/api/jobs/{key}/point/f_16a_block_15_adf/0')
    assert detail==data['aircraft'][0]['points'][0]
    checks.append('compact chart and on-demand exact point detail; no stick/trim outputs')
    assert data['equations_fingerprint']
    checks.append('uncached dry/fixed-trim calculation: 35 valid points, progress and completion')
    rows=list(csv.DictReader(io.StringIO(request(f'/api/jobs/{key}/samples.csv',raw=True).decode())))
    assert len(rows)>=35
    assert request(f'/api/jobs/{key}/diagram.png',raw=True).startswith(b'\x89PNG')
    assert request(f'/api/jobs/{key}/diagram.pdf',raw=True).startswith(b'%PDF')
    assert b'<svg' in request(f'/api/jobs/{key}/diagram.svg',raw=True)
    checks.append('CSV/JSON/PNG/SVG/PDF exports')
    assert request('/api/jobs',config)['id']==key
    integer_style={k:int(v) if isinstance(v,float) and v.is_integer() else v for k,v in config.items()}
    integer_style['fixed_trim']=[0,0,0]
    assert request('/api/jobs',integer_style)['id']==key
    assert request('/api/jobs/'+key)['status']=='complete'
    checks.append('equivalent integer/float settings reuse completed result')
    cancelled=request('/api/jobs',dict(config,extra_mass_kg=config['extra_mass_kg']+1.,speed_samples=49,load_samples=33))['id']
    request(f'/api/jobs/{cancelled}/cancel',{})
    cancel_job,cancel_states=terminal(cancelled)
    assert cancel_job['status']=='cancelled',cancel_job
    assert request('/api/meta')['latest']==key
    checks.append('cancelled calculation retains last complete result')
    for invalid in [dict(altitude_m=-1),dict(speed_samples=7.5),dict(aircraft=[]),dict(throttle=float('nan'))]:
        try:request('/api/jobs',invalid)
        except HTTPError as error:assert error.code==400
        else:raise AssertionError('Invalid request accepted')
    checks.append('four invalid HTTP configurations rejected')
    report=dict(checks=checks,job=key,statuses=states,cancel_statuses=cancel_states,failures=[])
    (ROOT/'analysis/em-api-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    if previous_latest and json.loads(latest_path.read_text())['id']==key:latest_path.write_text(previous_latest)
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
