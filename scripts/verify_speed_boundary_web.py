"""HTTP/Chrome validation of default-range A-4 plots and generic redlines."""
import json,time
from pathlib import Path
from urllib.request import Request,urlopen
from playwright.sync_api import sync_playwright
from em_plot import write_exports
from em_server import key_for,RUNTIME_FINGERPRINT
from em_solver import settings

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'analysis/speed-boundary-fixes'
URL='http://127.0.0.1:8765'


def fetch(path,body=None):
    raw=None if body is None else json.dumps(body).encode()
    with urlopen(Request(URL+path,data=raw,headers={'Content-Type':'application/json'}),timeout=30) as r:
        return json.load(r)


def main():
    jobs=[]
    for file in ['a4-fixed-off.json','a4-complete-on.json','f18-complete-on.json']:
        data=json.loads((OUT/file).read_text());job=key_for(data['settings'])
        data.update(id=job,equations_fingerprint=RUNTIME_FINGERPRINT)
        write_exports(data,ROOT/'outputs/em'/job)
        assert fetch('/api/jobs',data['settings'])['id']==job
        assert fetch('/api/jobs/'+job)['status']=='complete'
        jobs.append((file,job))
    # Exercise the live calculation endpoint on a range completely above VNE.
    cfg=settings(dict(aircraft=['a_4b'],speed_min_kmh=1200.,speed_max_kmh=1300.))
    excluded=fetch('/api/jobs',cfg)['id'];started=time.monotonic()
    while time.monotonic()-started<60:
        status=fetch('/api/jobs/'+excluded)
        if status['status'] in ['complete','error','cancelled']:break
        time.sleep(.2)
    assert status['status']=='complete',status
    empty=fetch('/api/jobs/'+excluded+'/data.json')['aircraft'][0]
    assert not empty['points'] and all(c['boundary_status']=='speed limit' for c in empty['columns'])
    errors=[];rows=[]
    with sync_playwright() as p:
        browser=p.chromium.launch(channel='chrome',headless=True)
        page=browser.new_page(viewport={'width':1600,'height':1200})
        page.on('pageerror',lambda e:errors.append(str(e)))
        page.goto(URL,wait_until='networkidle');page.wait_for_function('state.meta !== null')
        assert page.locator('input[name=aircraft]:checked').count()==0
        for file,job in jobs:
            page.evaluate('id=>loadData(id,true)',job)
            page.wait_for_function('id=>state.data?.id===id',arg=job)
            page.wait_for_timeout(500)
            data=fetch('/api/jobs/'+job+'/data.json');a=data['aircraft'][0]
            assert all(pt['speed_kmh']<=a['speed_limit']['sample_speed_kmh'] for pt in a['points'])
            assert not a['numerical_gaps'] and not a['numerical_boundaries'] and not a['instructor_unresolved_speeds_kmh']
            assert page.evaluate("$('chart')._fullLayout.dragmode==='pan' && $('chart')._fullLayout.yaxis.scaleratio===20")
            if a['id']=='a_4b':
                vertical=[pt for pt in a['boundary'] if pt.get('edge_kind')=='VNE speed boundary']
                assert len(vertical)==2 and vertical[0]['speed_kmh']==vertical[1]['speed_kmh']==a['speed_limit']['speed_kmh']
                assert vertical[0]['turn_dps']>0 and vertical[1]['turn_dps']==0
                surface=a['surface']
                assert all(row[i] is None for i,v in enumerate(surface['x']) if v>a['speed_limit']['sample_speed_kmh'] for row in surface['z'])
                page.evaluate("""() => {const el=$('chart');const i=el.data.findIndex(t=>t.name?.endsWith(' boundary'));
                    const j=state.data.aircraft[0].boundary.findIndex(p=>p.edge_kind==='VNE speed boundary');
                    Plotly.Fx.hover(el,[{curveNumber:i,pointNumber:j}]);}""")
                text=page.locator('#chart .hoverlayer').text_content()
                assert 'VNE speed boundary' in text and 'Turn radius' in text and 'Boundary Ps' in text,text
                page.evaluate("Plotly.Fx.unhover($('chart'))")
            page.locator('#chart').screenshot(path=str(OUT/(file.replace('.json','-browser.png'))))
            rows.append(dict(file=file,job=job,columns=len(a['columns']),gaps=len(a['numerical_gaps']),
                unresolved=len(a['numerical_boundaries']),instructor_unresolved=len(a['instructor_unresolved_speeds_kmh']),speed_limit=a['speed_limit']))
        page.get_by_role('button',name='Reset',exact=True).click()
        assert page.locator('input[name=aircraft]:checked').count()==0
        browser.close()
    (OUT/'web-validation.json').write_text(json.dumps(dict(rows=rows,above_redline_job=excluded,
        above_redline_points=0,browser_errors=errors),indent=2)+'\n')
    assert not errors,errors
    print('HTTP/Chrome speed-boundary checks pass',json.dumps(rows),flush=True)


if __name__=='__main__':main()
