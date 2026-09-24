"""Browser checks for shared 2D/3D SEP data, view state and exports."""
import argparse
import json
import time
from pathlib import Path
from playwright.sync_api import sync_playwright


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--job',required=True);parser.add_argument('--port',type=int,default=8766)
    args=parser.parse_args();url=f'http://127.0.0.1:{args.port}';out=Path('analysis/altitude-envelope');checks=[];errors=[]
    with sync_playwright() as p:
        browser=p.chromium.launch(channel='chrome',headless=True)
        page=browser.new_page(viewport={'width':1500,'height':1050});requests=[]
        page.on('pageerror',lambda e:errors.append(str(e)))
        page.on('request',lambda r:requests.append((r.method,r.url)))
        page.goto(url+'/?job='+args.job,wait_until='networkidle');page.wait_for_selector('#chart.js-plotly-plot')
        assert page.locator('#view-2d').get_attribute('aria-pressed')=='true'
        assert page.locator('#climb-form').count()==0
        page.evaluate("Plotly.relayout(document.getElementById('chart'),{'xaxis.range':[500,1400],'yaxis.range':[1000,14000]})")
        t=time.monotonic();page.locator('#view-3d').click();page.evaluate('state.renderQueue')
        first=time.monotonic()-t
        assert page.evaluate("document.getElementById('chart').data[0].type==='surface'")
        assert page.evaluate("JSON.stringify(document.getElementById('chart').data[0].z)===JSON.stringify(state.surface.sep_mps)")
        assert page.evaluate("document.getElementById('chart').data[0].connectgaps===false")
        checks.append('3D uses the identical full-resolution checked grid and null masks, without a solver request')
        camera={'eye':{'x':1.9,'y':.8,'z':1.3},'up':{'x':0,'y':0,'z':1},'center':{'x':0,'y':0,'z':0}}
        page.evaluate("camera=>Plotly.relayout(document.getElementById('chart'),{'scene.camera':camera})",camera)
        page.locator('#samples').check();page.evaluate('state.renderQueue')
        assert page.evaluate("document.getElementById('chart').data.some(t=>t.type==='scatter3d' && t.meta?.kind==='sample')")
        assert abs(page.evaluate("document.getElementById('chart')._fullLayout.scene.camera.eye.x")-1.9)<1e-6
        page.evaluate("()=>{const chart=document.getElementById('chart'),trace=chart.data.find(t=>t.meta?.kind==='sample');chart.emit('plotly_click',{points:[{data:trace,customdata:trace.customdata[0]}]});}")
        page.wait_for_selector('.point-grid')
        page.locator('#samples').uncheck();page.evaluate('state.renderQueue')
        page.locator('#climb-visible').uncheck();page.evaluate('state.renderQueue')
        assert not page.evaluate("document.getElementById('chart').data.some(t=>t.meta?.kind==='route')")
        page.locator('#climb-visible').check();page.evaluate('state.renderQueue')
        checks.append('3D samples, operating-point click and best-SEP visibility work without losing the camera')
        page.locator('#view-2d').click();page.evaluate('state.renderQueue')
        assert page.evaluate("document.getElementById('chart').layout.xaxis.range")==[500,1400]
        t=time.monotonic();page.locator('#view-3d').click();page.evaluate('state.renderQueue');repeat=time.monotonic()-t
        assert abs(page.evaluate("document.getElementById('chart')._fullLayout.scene.camera.eye.x")-1.9)<1e-6
        page.locator('#reset-view').click();page.evaluate('state.renderQueue')
        assert abs(page.evaluate("document.getElementById('chart')._fullLayout.scene.camera.eye.x")-1.6)<1e-6
        page.locator('#gaps').check();page.evaluate('state.renderQueue')
        assert abs(page.evaluate("document.getElementById('chart')._fullLayout.scene.camera.eye.x")-1.6)<1e-6
        page.locator('#gaps').uncheck();page.evaluate('state.renderQueue')
        checks.append('2D zoom and 3D camera persist independently; reset stays reset on subsequent overlay changes')
        page.evaluate("()=>{for(const mode of ['2d','3d','2d','3d','2d'])document.getElementById('view-'+mode).click();}")
        page.evaluate('state.renderQueue')
        assert page.evaluate("state.view==='2d' && document.getElementById('chart').data[0].type==='scatter'")
        assert not any(method=='POST' for method,_ in requests)
        assert sum('/surface.json' in u for _,u in requests)==1
        checks.append('rapid view changes finish in the selected view; one surface fetch and no physics reruns')
        page.locator('#view-3d').click();page.evaluate('state.renderQueue')
        with page.expect_download() as download:page.locator('#surface-download').click()
        download.value.save_as(str(out/'surface-export.png'))
        assert (out/'surface-export.png').read_bytes()[:8]==b'\x89PNG\r\n\x1a\n'
        page.locator('#chart').screenshot(path=str(out/'surface-3d.png'))
        page.reload(wait_until='networkidle');page.wait_for_function("document.getElementById('chart').data?.[0].type==='surface'");page.evaluate('state.renderQueue')
        assert page.locator('#view-3d').get_attribute('aria-pressed')=='true'
        assert page.evaluate("state.route.points[0].altitude_m===state.chart.settings.altitude_min_m && state.route.points.at(-1).altitude_m===state.chart.settings.altitude_max_m")
        checks.append('3D PNG export, saved 3D URL restoration and full-altitude guide coverage')
        page.set_viewport_size({'width':390,'height':844});page.wait_for_function('document.documentElement.scrollWidth<=innerWidth+1')
        page.locator('#chart').screenshot(path=str(out/'surface-mobile.png'))
        checks.append('mobile 3D layout fits viewport without horizontal overflow')
        assert not errors,errors
        browser.close()
    report=dict(status='PASS',job=args.job,first_3d_switch_s=first,cached_3d_switch_s=repeat,checks=checks,javascript_errors=errors)
    (out/'surface-browser-validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))


if __name__=='__main__':main()
