"""Browser checks against the local propeller integration test server."""
import argparse,json
from pathlib import Path
from playwright.sync_api import sync_playwright

ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'analysis/prop-integration'


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--url',default='http://127.0.0.1:8766')
    args=parser.parse_args()
    errors=[];checks=[]
    with sync_playwright() as p:
        browser=p.chromium.launch(channel='chrome',headless=True)
        page=browser.new_page(viewport={'width':1440,'height':1100},device_scale_factor=1)
        page.on('pageerror',lambda e:errors.append(str(e)))
        page.goto(args.url,wait_until='networkidle')
        assert page.locator('input[name=aircraft]:checked').count()==0
        assert page.locator('#altitude').is_disabled();checks.append('Empty on load')
        def select(name):
            page.locator('#aircraft-search').fill(name)
            page.locator(f'input[name=aircraft][value="{name}"]').check()
        select('yak-3');assert page.locator('#propeller-hint').is_visible()
        page.locator('#altitude').fill('1200')
        select('bf-109f-4');page.locator('#settings-aircraft').select_option('bf-109f-4')
        assert page.locator('#altitude').input_value()=='0'
        page.locator('#altitude').fill('3000');page.locator('#settings-aircraft').select_option('yak-3')
        assert page.locator('#altitude').input_value()=='1200'
        assert page.locator('#selected-aircraft input:checked').count()==2
        checks.append('Prop variants selectable, selected rows pinned, independent conditions retained')
        page.locator('#reset').click();assert page.locator('input[name=aircraft]:checked').count()==0
        select('f_16a_block_15_adf');assert page.locator('#propeller-hint').is_hidden()
        page.locator('#reset').click();page.locator('#aircraft-search').fill('yak-3_test')
        assert page.locator('input[value="yak-3_test"]').is_disabled()
        checks.append('Reset clears selection; jet controls unchanged; exact missing geometry stays unavailable')
        point=next(r for r in json.loads((OUT/'em-integration-points.json').read_text()) if r['aircraft']=='a5m4')
        aircraft=dict(id='a5m4',name='A5M4',engine=dict(policy='Settled installed propeller, closed radiators'),settings=dict(structural_limits=True))
        page.evaluate('({a,p})=>inspect(a,p)',dict(a=aircraft,p=point))
        content=page.locator('#point-content').inner_text()
        assert all(x in content for x in ['Engine RPM','Propeller pitch','Compressor stage','Fixed · deployed','Cycle mean trim'])
        checks.append('Actual prop point shows engine controls, fixed gear and cycle evidence')
        page.locator('#aircraft-search').fill('');select('yak-3')
        page.screenshot(path=str(OUT/'website-propeller-desktop.png'),full_page=True)
        completed=OUT/'web-validation.json'
        if completed.exists():
            key=json.loads(completed.read_text())['job']
            # Re-render the independently checked physics fixture with the
            # current contour generator; do not relabel its old cached job.
            rendered=OUT/'rendered-comparison/chart.json'
            if rendered.exists():
                page.route('**/api/jobs/'+key+'/chart.json',lambda route:route.fulfill(path=str(rendered),content_type='application/json'))
            page.evaluate('(id)=>loadData(id,true)',key)
            page.wait_for_function('(id)=>state.dataJob===id',arg=key,timeout=30000)
            assert page.locator('.metric').count()==2
            assert page.evaluate("$('chart')._fullLayout.dragmode==='pan'")
            # The HTTP fixture has an explicit 3 g ceiling and a 201-cell
            # surface. The UI requests its automatic ceiling and 601 cells;
            # loading that fixture must truthfully flag the different setup.
            assert page.locator('#stale').is_visible()
            checks.append('Completed prop comparison renders; cropped API settings correctly flagged as different from UI settings')
            page.screenshot(path=str(OUT/'website-propeller-comparison.png'),full_page=True)
            if rendered.exists():
                assert page.evaluate('state.data.aircraft.every(a=>a.contours.some(c=>c.level===10))')
                page.evaluate("state.view='yak-3';renderChart()")
                page.wait_for_function("$('chart').data.some(t=>t.type==='heatmap'&&t.zmin===-50&&t.zmax===50)")
                checks.append('Current propeller contour paths and ±50 m/s heatmap render from unchanged solved points')
        page.set_viewport_size({'width':390,'height':844})
        assert page.evaluate('document.documentElement.scrollWidth<=window.innerWidth+1')
        page.screenshot(path=str(OUT/'website-propeller-mobile.png'),full_page=True)
        checks.append('Mobile layout has no horizontal overflow')
        browser.close()
    report=dict(status='FAIL' if errors else 'PASS',checks=checks,browser_errors=errors)
    (OUT/'browser-validation.json').write_text(json.dumps(report,indent=2)+'\n');print(report)
    if errors:raise SystemExit(1)


if __name__=='__main__':main()
