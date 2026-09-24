"""Browser/API regression checks without touching the EM server."""
import argparse
import json
import time
from pathlib import Path
from playwright.sync_api import sync_playwright

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--job',required=True);parser.add_argument('--port',type=int,default=8766)
    args=parser.parse_args();url=f'http://127.0.0.1:{args.port}';checks=[];errors=[]
    out=ROOT/'analysis/altitude-envelope';out.mkdir(parents=True,exist_ok=True)
    with sync_playwright() as p:
        browser=p.chromium.launch(channel='chrome',headless=True)
        page=browser.new_page(viewport={'width':1500,'height':1050})
        page.on('pageerror',lambda error:errors.append(str(error)))
        page.goto(url,wait_until='networkidle');page.wait_for_function('state.meta!==null')
        assert page.locator('#calculate').is_disabled()
        page.locator('#search').fill('f_14a_early');page.locator('#aircraft').select_option('f_14a_early')
        assert page.locator('#sweep-field').is_visible()
        page.locator('#search').fill('yak-3');page.locator('#aircraft').select_option('yak-3')
        assert page.locator('#prop-hint').is_visible() and not page.locator('#sweep-field').is_visible()
        assert page.locator('#mode, #flaps').count()==0
        assert page.evaluate("!('instructor' in readConfig().conditions) && !('flaps_percent' in readConfig().conditions)")
        assert page.locator('#quality').input_value()=='preview'
        checks.append('catalog search, sweep/prop controls; no mode/flap input; fast default')
        page.evaluate('(key)=>loadResult(key)',args.job)
        page.wait_for_selector('#chart.js-plotly-plot')
        assert page.evaluate("document.getElementById('chart').data.some(t=>t.meta?.kind==='contour')")
        checks.append('real API result and contour paths rendered')
        canonical=page.evaluate('state.chart.settings')
        canonical['conditions'].update(instructor=True,flaps_percent=100,torque_gyro=True)
        normalized=page.request.post(url+'/api/altitude/jobs',data=canonical)
        assert normalized.ok and normalized.json()['id']==args.job
        checks.append('legacy Instructor/flap requests return the same clean calculation and cache key')
        page.reload(wait_until='networkidle')
        page.wait_for_selector('#chart.js-plotly-plot')
        assert page.evaluate('state.resultId')==args.job
        assert not page.locator('#stale').is_visible()
        # Restoring 1.1 throttle must not produce 110.00000000000001%, which
        # exceeds the HTML maximum and silently blocks the next submission.
        assert page.locator('#throttle').evaluate('(el)=>el.checkValidity()')
        checks.append('saved result URL restores chart and matching controls after reload')
        assert page.evaluate("state.route?.status==='complete'")
        assert page.evaluate("document.getElementById('chart').data.some(t=>t.meta?.kind==='route')")
        assert page.locator('#climb-form').count()==0
        assert page.evaluate("state.route.objective==='continuous_maximum_sep' && state.route.points[0].altitude_m===state.chart.settings.altitude_min_m && state.route.points.at(-1).altitude_m===state.chart.settings.altitude_max_m")
        page.locator('#climb-visible').uncheck()
        page.wait_for_function("!document.getElementById('chart').data.some(t=>t.meta?.kind==='route')")
        assert 'guide_visible=0' in page.locator('[data-export="diagram.svg"]').get_attribute('href')
        page.locator('#climb-visible').check()
        page.wait_for_function("document.getElementById('chart').data.some(t=>t.meta?.kind==='route')")
        with page.expect_download() as download:page.locator('#climb-download').click()
        download.value.save_as(str(out/'route-browser.csv'))
        assert (out/'route-browser.csv').read_text().startswith('altitude_m,speed_kmh,sep_mps,specific_power_w_kg')
        route_svg=page.request.get(url+page.locator('[data-export="diagram.svg"]').get_attribute('href'))
        assert route_svg.ok and 'Continuous best-SEP guide' in route_svg.text()
        checks.append('continuous full-range energy guide without time or target controls, toggle, CSV and matching figure export')
        page.locator('#samples').check();page.locator('#gaps').check()
        page.wait_for_function("document.getElementById('chart').data.some(t=>t.meta?.kind==='sample')")
        index=page.evaluate('state.chart.points.find(p=>p.valid).index')
        page.evaluate('(index)=>inspect(index)',index)
        page.wait_for_selector('.point-grid')
        assert 'Native SEP' in page.locator('#point-content').inner_text()
        checks.append('sample and unresolved overlays plus full point inspection')
        hover=page.evaluate('''()=>{const el=document.getElementById('chart'),i=el.data.findIndex(t=>t.meta?.kind==='contour');
          Plotly.Fx.hover(el,[{curveNumber:i,pointNumber:0}]);return el.data[i].hovertemplate;}''')
        assert 'SEP' in hover and 'Altitude' in hover
        checks.append('contour hover includes speed, altitude and SEP')
        page.locator('#samples').uncheck();page.locator('#gaps').uncheck()
        page.locator('#chart').screenshot(path=str(out/'chart.png'))
        for name in ['data.json','samples.csv','diagram.svg','diagram.png','diagram.pdf']:
            response=page.request.get(url+'/api/altitude/jobs/'+args.job+'/'+name)
            assert response.ok and len(response.body())>1000,(name,response.status)
        assert page.request.get(url+'/api/altitude/jobs/../../app.js').status==404
        invalid=page.request.post(url+'/api/altitude/jobs',data={'aircraft':'not-an-aircraft'})
        assert invalid.status==400
        foreign=page.request.post(url+'/api/altitude/jobs',data={},headers={'Origin':'https://example.invalid'})
        assert foreign.status==403
        checks.append('all five download formats and API validation/origin checks')
        page.locator('#fuel').fill('40');assert page.locator('#stale').is_visible()
        page.set_viewport_size({'width':390,'height':844})
        page.wait_for_function('document.documentElement.scrollWidth<=innerWidth+1')
        page.screenshot(path=str(out/'mobile.png'),full_page=True)
        checks.append('changed-settings notice and responsive mobile layout without overflow')
        # Submit through the visible form, then cancel the actual calculation.
        page.locator('#reset').click()
        page.locator('#search').fill('f_16a_block_15_adf');page.locator('#aircraft').select_option('f_16a_block_15_adf')
        page.locator('#fuel').fill('43');page.locator('#calculate').click()
        page.wait_for_function('state.job!==null')
        page.locator('#cancel').click()
        page.wait_for_function('!state.busy',timeout=30000)
        assert not page.locator('#error').is_visible(),page.locator('#error').text_content()
        checks.append('visible Calculate flow starts a real job and Cancel stops it')
        page.locator('#speed-min').fill('800');page.locator('#speed-max').fill('1200')
        page.locator('#altitude-min').fill('3000');page.locator('#altitude-max').fill('4000')
        page.locator('#calculate').click();page.wait_for_function('!state.busy',timeout=30000)
        assert not page.locator('#error').is_visible(),page.locator('#error').text_content()
        assert page.evaluate('state.chart.settings.altitude_max_m')==4000
        checks.append('persistent workers complete a fresh calculation after cancellation')
        assert not errors,errors
        browser.close()
    report=dict(status='PASS',checks=checks,browser_errors=errors,job=args.job)
    (out/'browser-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
