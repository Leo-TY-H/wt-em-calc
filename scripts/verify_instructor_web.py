"""Local HTTP calculation and isolated Chrome UI check for Instructor integration."""
import json,time
from pathlib import Path
from urllib.request import Request,urlopen
from playwright.sync_api import sync_playwright

ROOT=Path(__file__).resolve().parents[1];URL='http://127.0.0.1:8765'

def fetch(path,body=None):
    data=None if body is None else json.dumps(body).encode()
    with urlopen(Request(URL+path,data=data,headers={'Content-Type':'application/json'}),timeout=30) as r:return json.load(r)

def main():
    cfg=dict(aircraft=['f_16a_block_15_adf'],instructor=True,speed_min_kmh=400.,speed_max_kmh=700.,
        speed_samples=9,load_samples=7,sep_tolerance_mps=1.,surface_resolution=201)
    job=fetch('/api/jobs',cfg)['id'];started=time.monotonic()
    while time.monotonic()-started<240:
        status=fetch('/api/jobs/'+job)
        if status['status'] in ['complete','error','cancelled']:break
        time.sleep(.5)
    assert status['status']=='complete',status
    print('HTTP job',job,'complete',status.get('elapsed_s'),flush=True)
    errors=[];checks=[]
    with sync_playwright() as p:
        browser=p.chromium.launch(channel='chrome',headless=True)
        page=browser.new_page(viewport={'width':1440,'height':1100})
        page.on('pageerror',lambda e:errors.append(str(e)))
        page.goto(URL,wait_until='networkidle')
        page.wait_for_function('state.meta !== null')
        assert page.locator('input[name=aircraft]:checked').count()==0
        checks.append('Startup has no selected aircraft')
        assert page.locator('#speed-min').input_value()=='150'
        assert page.locator('#speed-max').input_value()=='1300'
        assert page.locator('#fuel').input_value()=='30'
        assert page.locator('#throttle').input_value()=='110'
        checks.append('Requested default condition retained')
        page.evaluate('id => loadData(id,true)',job)
        page.wait_for_selector('#chart.js-plotly-plot')
        page.wait_for_timeout(400)
        assert page.locator('#instructor').is_checked()
        assert 'stationary' in page.locator('#instructor-hint').inner_text()
        assert 'Interior controller reachability is not certified' in page.locator('#instructor-hint').inner_text()
        assert page.evaluate("state.data.aircraft[0].instructor_approximation.kind==='experimental coupled held-pitch boundary'")
        assert page.evaluate("$('chart')._fullLayout.dragmode==='pan'")
        assert page.evaluate("$('chart').data.some(t=>t.name?.endsWith(' boundary')&&t.line.dash==='solid')")
        assert page.evaluate("$('chart').data.filter(t=>t.name?.includes('Ps ')).every(t=>t.line.dash==='dash')")
        assert page.locator('#max-load').count()==0
        checks.append('New Instructor model rendered, labeled experimental; pan/line styles/automatic ceiling retained')
        page.evaluate("""() => {const el=$('chart');const i=el.data.findIndex(t=>t.name?.endsWith(' boundary'));const j=el.data[i].x.findIndex((x,j)=>x>500&&el.data[i].y[j]!=null);Plotly.Fx.hover(el,[{curveNumber:i,pointNumber:j}]);}""")
        assert 'Turn radius' in page.locator('#chart .hoverlayer').text_content()
        checks.append('Boundary hover includes turn radius')
        page.evaluate("Plotly.Fx.unhover($('chart'))")
        page.locator('#chart').screenshot(path=str(ROOT/'analysis/instructor-full/integrated-browser.png'))
        page.get_by_role('button',name='Reset',exact=True).click()
        assert page.locator('input[name=aircraft]:checked').count()==0
        checks.append('Reset deselects every aircraft')
        browser.close()
    report=dict(job=job,status=status,checks=checks,browser_errors=errors)
    (ROOT/'analysis/instructor-full/web-integration-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(report,flush=True)
    if errors:raise SystemExit(1)

if __name__=='__main__':main()
