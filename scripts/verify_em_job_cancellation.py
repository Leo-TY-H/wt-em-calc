"""A queued retry must survive completion of its cancelled predecessor."""
import json,threading
from pathlib import Path
import em_server as server


def main():
    old=threading.Event();old.set();new=threading.Event()
    key='cancellation-generation-check'
    config=server.settings(dict(aircraft=['yak-3']))
    server.JOBS[key]=dict(id=key,status='queued',cancel=new)
    server.run_job(key,config,old)
    assert server.JOBS[key]['status']=='queued' and server.JOBS[key]['cancel'] is new
    assert 'progress' not in server.JOBS[key]
    new.set();server.run_job(key,config,new)
    assert server.JOBS[key]['status']=='cancelled' and 'progress' not in server.JOBS[key]
    report=dict(status='PASS',checks=['Cancelled old generation cannot overwrite its queued replacement',
        'Cancelled queued generation exits before computation'])
    (Path(__file__).resolve().parents[1]/'analysis/prop-integration/cancellation-generation-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(report)


if __name__=='__main__':main()
