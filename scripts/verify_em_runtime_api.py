"""Real adaptive cancellation, new-pool recovery, entry exports and exact reuse."""
import argparse,json,time
from pathlib import Path
from urllib.request import Request,urlopen


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--base',default='http://127.0.0.1:8766')
    ap.add_argument('--report',default='analysis/runtime-20s/api-validation.json');args=ap.parse_args()
    def request(path,data=None):
        req=Request(args.base+path,data=None if data is None else json.dumps(data).encode(),headers={'Content-Type':'application/json'})
        with urlopen(req,timeout=30) as response:return json.load(response)
    def terminal(key):
        deadline=time.monotonic()+180;states=[]
        while time.monotonic()<deadline:
            job=request('/api/jobs/'+key)
            if job['status'] not in states:states.append(job['status'])
            if job['status'] in ('complete','cancelled','error'):return job,states
            time.sleep(.2)
        raise AssertionError('Calculation did not terminate')
    fuel=30.+(time.time_ns()%1000000)/1000000.
    # Wait until workers have started real adaptive work before cancelling.
    cancelled=request('/api/jobs',dict(aircraft=['yak-3'],fuel_percent=fuel))['id']
    time.sleep(3.)
    before=request('/api/jobs/'+cancelled);assert before['status']=='running',before
    t=time.monotonic();request('/api/jobs/'+cancelled+'/cancel',{})
    stopped,states=terminal(cancelled);cancel_s=time.monotonic()-t
    assert stopped['status']=='cancelled',stopped
    print('Cancelled active adaptive calculation in',cancel_s,flush=True)
    entries=[dict(id='entry_runtime_'+str(i),aircraft_id='f_16a_block_10_mod',settings=dict(instructor=bool(i),fuel_percent=fuel)) for i in range(2)]
    cfg=dict(entries=entries,speed_min_kmh=700.,speed_max_kmh=800.,speed_samples=7,load_samples=5,sep_tolerance_mps=1.)
    t=time.monotonic();key=request('/api/jobs',cfg)['id'];job,states=terminal(key)
    assert job['status']=='complete',job
    chart=request('/api/jobs/'+key+'/chart.json');ready_s=time.monotonic()-t
    assert [a['id'] for a in chart['aircraft']]==[e['id'] for e in entries]
    assert [a['settings']['instructor'] for a in chart['aircraft']]==[False,True]
    for a in chart['aircraft']:
        point=next(p for p in a['points'] if p['valid'])
        detailed=request(f'/api/jobs/{key}/point/{a["id"]}/{point["detail_index"]}')
        assert detailed['instructor_enabled']==a['settings']['instructor']
        assert 'sticks' not in detailed and 'trim' not in detailed
    print('Fresh Instructor comparison after cancellation in',ready_s,flush=True)
    cfg['entries']=[dict(entries[i%2],id='entry_reuse_'+str(i)) for i in range(8)]
    t=time.monotonic();cached=request('/api/jobs',cfg)['id'];job,cache_states=terminal(cached)
    assert job['status']=='complete',job
    chart=request('/api/jobs/'+cached+'/chart.json');cached_s=time.monotonic()-t
    assert len(chart['aircraft'])==8 and len(chart['cached_aircraft'])==8
    assert len({a['id'] for a in chart['aircraft']})==8
    for a in chart['aircraft']:
        point=next(p for p in a['points'] if p['valid'])
        assert request(f'/api/jobs/{cached}/point/{a["id"]}/{point["detail_index"]}')['instructor_enabled']==a['settings']['instructor']
    assert request('/api/jobs',cfg)['id']==cached
    report=dict(status='PASS',cancel_s=cancel_s,comparison_ready_s=ready_s,eight_reused_ready_s=cached_s,
        comparison_job=key,eight_job=cached,statuses=states,cached_statuses=cache_states,
        checks=['cancel active adaptive workers','fresh successful calculation after cancellation','independent Instructor entries',
                'eight exact reused physical results','all entry point endpoints','completed request reuse'])
    Path(args.report).write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))

if __name__=='__main__':main()
