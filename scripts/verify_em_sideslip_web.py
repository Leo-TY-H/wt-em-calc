"""Low-speed Instructor outline and retained plot UI, through HTTP and Chrome."""
import json,time,sys
from pathlib import Path
from urllib.request import Request,urlopen
from playwright.sync_api import sync_playwright

ROOT=Path(__file__).resolve().parents[1]
URL='http://127.0.0.1:8765'


def fetch(path,body=None):
    data=None if body is None else json.dumps(body).encode()
    with urlopen(Request(URL+path,data=data,headers={'Content-Type':'application/json'}),timeout=30) as response:
        return json.load(response)


def main():
    ariete='--ariete' in sys.argv
    config_file='ariete-0-config.json' if ariete else 'low-speed-on-config.json'
    cfg=json.loads((ROOT/'analysis/sideslip-integration'/config_file).read_text())
    expected_edge='stall-speed edge' if ariete else 'Instructor minimum-speed edge'
    prefix='ariete-' if ariete else ''
    job=fetch('/api/jobs',cfg)['id'];start=time.monotonic()
    while time.monotonic()-start<300:
        status=fetch('/api/jobs/'+job)
        if status['status'] in ['complete','error','cancelled']:break
        time.sleep(.5)
    assert status['status']=='complete',status
    errors=[]
    with sync_playwright() as p:
        browser=p.chromium.launch(channel='chrome',headless=True)
        page=browser.new_page(viewport={'width':1440,'height':1100})
        page.on('pageerror',lambda e:errors.append(str(e)))
        page.goto(URL,wait_until='networkidle');page.wait_for_function('state.meta !== null')
        assert page.locator('input[name=aircraft]:checked').count()==0
        page.evaluate('id=>loadData(id,true)',job)
        page.wait_for_selector('#chart.js-plotly-plot');page.wait_for_timeout(400)
        summary=page.evaluate("""() => state.data.aircraft.map(a=>({id:a.id,
            edge:a.low_speed_edge,gaps:a.numerical_gaps.length,unresolved:a.numerical_boundaries.length,
            instructorUnresolved:a.instructor_unresolved_speeds_kmh.length,
            vertical:a.boundary.filter(p=>p.vertical_edge)}))""")
        for row in summary:
            assert row['gaps']==row['unresolved']==row['instructorUnresolved']==0,row
            assert row['edge']['refined'] and row['edge']['kind']==expected_edge,row
            edge=row['vertical'];assert len(edge)==2 and edge[0]['speed_kmh']==edge[1]['speed_kmh'] and edge[0]['turn_dps']==0.
        assert page.evaluate("$('chart')._fullLayout.dragmode==='pan'")
        assert page.evaluate("$('chart')._fullLayout.yaxis.scaleratio===20")
        assert page.evaluate("$('chart').data.filter(t=>t.name?.endsWith(' boundary')).every(t=>t.line.dash==='solid'&&t.cliponaxis===false)")
        page.evaluate("""() => {const el=$('chart');const i=el.data.findIndex(t=>t.name?.endsWith(' boundary'));
            const j=state.data.aircraft[0].boundary.findIndex(p=>p.vertical_edge);Plotly.Fx.hover(el,[{curveNumber:i,pointNumber:j}]);}""")
        text=page.locator('#chart .hoverlayer').text_content()
        assert expected_edge in text and 'Turn radius' in text,text
        page.evaluate("Plotly.Fx.unhover($('chart'))")
        page.locator('#chart').screenshot(path=str(ROOT/f'analysis/sideslip-integration/{prefix}low-speed-browser.png'))
        page.get_by_role('button',name='Reset',exact=True).click()
        assert page.locator('input[name=aircraft]:checked').count()==0
        browser.close()
    report=dict(job=job,status=status,aircraft=summary,browser_errors=errors)
    (ROOT/f'analysis/sideslip-integration/{prefix}web-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print('Low-speed HTTP/Chrome validation',job,'errors',errors,flush=True)
    assert not errors


if __name__=='__main__':main()
