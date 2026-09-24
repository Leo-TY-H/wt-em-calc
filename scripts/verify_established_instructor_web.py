"""Serve and inspect established-Instructor full plots plus a new HTTP job."""
import json,time
from pathlib import Path
from urllib.request import Request,urlopen
from playwright.sync_api import sync_playwright
from em_plot import write_exports
from em_server import key_for,RUNTIME_FINGERPRINT
from em_solver import settings

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'analysis/instructor-history'
URL='http://127.0.0.1:8765'


def fetch(path,body=None):
    raw=None if body is None else json.dumps(body).encode()
    with urlopen(Request(URL+path,data=raw,headers={'Content-Type':'application/json'}),timeout=30) as r:
        return json.load(r)


def main():
    jobs=[];rows=[]
    for file in ['a4-established.json','f18-established.json']:
        data=json.loads((OUT/file).read_text());job=key_for(data['settings'])
        data.update(id=job,equations_fingerprint=RUNTIME_FINGERPRINT)
        write_exports(data,ROOT/'outputs/em'/job)
        assert fetch('/api/jobs',data['settings'])['id']==job
        assert fetch('/api/jobs/'+job)['status']=='complete'
        a=data['aircraft'][0]
        assert not a['numerical_gaps'] and not a['numerical_boundaries'] and not a['instructor_unresolved_speeds_kmh'],file
        assert all(pt['speed_kmh']<=a['speed_limit']['sample_speed_kmh'] for pt in a['points'])
        jobs.append((file,job))
        baseline='a4-complete-on.json' if a['id']=='a_4b' else 'f18-complete-on.json'
        old=json.loads((ROOT/'analysis/speed-boundary-fixes'/baseline).read_text())['aircraft'][0]
        old_roots={p['speed_kmh']:p for p in old['sustained']}
        common=[(p,old_roots[p['speed_kmh']]) for p in a['sustained'] if p['speed_kmh'] in old_roots]
        rows.append(dict(file=file,job=job,columns=len(a['columns']),
            verified=sum(c['boundary_status']=='verified limit' for c in a['columns']),
            empty=sum(c['boundary_status']=='no feasible samples' for c in a['columns']),
            gaps=len(a['numerical_gaps']),unresolved=len(a['numerical_boundaries']),
            sustained_common_speeds=len(common),
            max_sustained_turn_change_dps=max((abs(p['turn_dps']-q['turn_dps']) for p,q in common),default=None),
            elapsed_s=data['elapsed_s']))
    cfg=settings(dict(aircraft=['fa_18e_block_2'],instructor=True,speed_min_kmh=790.,speed_max_kmh=795.,speed_samples=9,load_samples=7))
    live=fetch('/api/jobs',cfg)['id'];started=time.monotonic()
    while time.monotonic()-started<180:
        status=fetch('/api/jobs/'+live)
        if status['status'] in ['complete','error','cancelled']:break
        time.sleep(.2)
    assert status['status']=='complete',status
    actual=fetch('/api/jobs/'+live+'/data.json')['aircraft'][0]
    assert not actual['numerical_gaps'] and not actual['numerical_boundaries'] and not actual['instructor_unresolved_speeds_kmh']
    errors=[]
    with sync_playwright() as p:
        browser=p.chromium.launch(channel='chrome',headless=True)
        page=browser.new_page(viewport={'width':1600,'height':1200})
        page.on('pageerror',lambda e:errors.append(str(e)))
        page.goto(URL,wait_until='networkidle');page.wait_for_function('state.meta !== null')
        assert page.locator('input[name=aircraft]:checked').count()==0
        assert 'established' in page.locator('#instructor-hint').text_content()
        for file,job in jobs:
            page.evaluate('id=>loadData(id,true)',job)
            page.wait_for_function('id=>state.data?.id===id',arg=job)
            page.wait_for_timeout(500)
            assert page.evaluate("$('chart')._fullLayout.dragmode==='pan' && $('chart')._fullLayout.yaxis.scaleratio===20")
            page.locator('#chart').screenshot(path=str(OUT/file.replace('.json','-browser.png')))
        page.get_by_role('button',name='Reset',exact=True).click()
        assert page.locator('input[name=aircraft]:checked').count()==0
        browser.close()
    report=dict(rows=rows,live_job=live,live_columns=len(actual['columns']),browser_errors=errors)
    (OUT/'web-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    assert not errors,errors
    print(json.dumps(report,indent=2),flush=True)


if __name__=='__main__':main()
