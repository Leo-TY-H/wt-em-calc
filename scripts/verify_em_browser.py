"""Local browser interaction checks; never connects to game or third-party tabs."""
import json
from pathlib import Path
from playwright.sync_api import sync_playwright

ROOT=Path(__file__).resolve().parents[1]


def main():
    errors=[];checks=[]
    with sync_playwright() as p:
        browser=p.chromium.launch(channel='chrome',headless=True)
        page=browser.new_page(viewport={'width':1440,'height':1100},device_scale_factor=1)
        page.on('pageerror',lambda error:errors.append(str(error)))
        page.goto('http://127.0.0.1:8765',wait_until='networkidle')
        page.wait_for_selector('#chart.js-plotly-plot',timeout=30000)
        assert page.locator('.metric').count()==2;checks.append('saved comparison loaded')
        page.screenshot(path=str(ROOT/'outputs/em-plotter-desktop.png'),full_page=True)
        page.get_by_role('button',name='JAS39C',exact=True).click()
        page.wait_for_timeout(500)
        assert page.evaluate("document.getElementById('chart').data.some(t=>t.type==='heatmap')")
        checks.append('single-aircraft heatmap')
        page.locator('#show-samples').check();page.locator('#show-rejected').check()
        assert page.evaluate("document.getElementById('chart').data.some(t=>t.name==='JAS39C rejected')")
        checks.append('sample and rejected overlays')
        # Click a visible plotted sample through the real browser pointer.
        point=page.evaluate("""() => {const el=document.getElementById('chart');const t=el.data.find(t=>t.customdata?.some(c=>Array.isArray(c)&&c[1]==='grid'));const i=t.x.findIndex((x,i)=>x>800&&x<1100&&t.y[i]>4&&t.y[i]<8);const box=el.getBoundingClientRect(),f=el._fullLayout;return {x:box.x+f.xaxis._offset+f.xaxis.l2p(t.x[i]),y:box.y+f.yaxis._offset+f.yaxis.l2p(t.y[i])};}""")
        page.mouse.click(point['x'],point['y'])
        page.wait_for_function("document.getElementById('point-title').textContent.includes('JAS39C')",timeout=10000)
        assert 'JAS39C' in page.locator('#point-title').inner_text()
        assert 'COMPONENT FORCES' in page.locator('#point-content').inner_text();checks.append('point inspector')
        page.locator('#altitude').fill('4500');assert page.locator('#stale').is_visible();checks.append('changed settings flagged')
        assert page.locator('#trim-mode').count()==0
        assert page.locator('#max-load').count()==0
        assert 'Manual trim' not in page.locator('#point-content').inner_text()
        assert 'stick' not in page.locator('#point-content').inner_text().split('Gravity')[0].lower()
        checks.append('performance interface has no stick or trim settings/outputs')
        page.get_by_role('button',name='Reset',exact=True).click()
        assert page.locator('#altitude').input_value()=='0'
        assert page.locator('#fuel').input_value()=='30'
        assert page.locator('#speed-min').input_value()=='150'
        assert page.locator('#speed-max').input_value()=='1300'
        assert page.locator('#throttle').input_value()=='110' and page.locator('#afterburner').count()==0
        assert page.evaluate('readConfig().afterburner')
        checks.append('requested sea-level defaults and automatic boundary; no load-ceiling control')
        assert not page.locator('#sweep-field').is_visible()
        for aircraft in ['f_16a_block_15_adf','saab_jas39c']:
            page.locator(f'.aircraft-option:has(input[value="{aircraft}"])').click()
        page.locator('#aircraft-search').fill('f_14a_early')
        assert page.locator('.aircraft-option:visible').count()==1
        page.locator('.aircraft-option:visible').click()
        assert page.locator('#sweep-field').is_visible()
        page.locator('#sweep').focus();page.locator('#sweep').press('Home')
        for _ in range(37):page.locator('#sweep').press('ArrowRight')
        assert page.locator('#sweep').input_value()=='37'
        assert page.locator('#sweep-label').inner_text()=='37%'
        assert page.locator('#sweep-field select').count()==0
        assert 'best' not in page.locator('#sweep-field').inner_text().lower()
        assert 'unavailable' not in page.locator('#error').inner_text().lower()
        assert page.locator('#stale').is_visible()
        checks.append('searchable catalog; swing-wing selection reveals manual-only 0–100% sweep slider')
        page.locator('#aircraft-search').fill('')
        page.get_by_role('button',name='Reset',exact=True).click()
        assert not page.locator('#sweep-field').is_visible()
        page.locator('#calculate').click()
        page.wait_for_function("!document.getElementById('calculate').disabled",timeout=15000)
        assert not page.locator('#stale').is_visible();checks.append('calculate reuses saved default')
        page.locator('#altitude').fill('1100');page.locator('#calculate').click()
        page.wait_for_function("document.getElementById('progress-label').textContent!=='Preparing calculation…'",timeout=10000)
        page.locator('#cancel').click()
        page.wait_for_function("document.getElementById('footer-status').textContent.includes('cancelled')",timeout=10000)
        assert page.locator('.metric').count()==2;checks.append('cancel button preserves displayed results')
        page.get_by_role('button',name='Reset',exact=True).click()
        with page.expect_download() as download:page.get_by_text('CSV',exact=True).click()
        file=download.value;file.save_as(str(ROOT/'outputs/em-browser-download.csv'))
        assert (ROOT/'outputs/em-browser-download.csv').read_text().startswith('aircraft,kind,');checks.append('CSV download')
        page.get_by_role('button',name='Compare',exact=True).click();page.locator('#show-samples').uncheck();page.locator('#show-rejected').uncheck()
        # Save the normal completed state, not the cancellation-test notice.
        page.reload(wait_until='networkidle')
        page.wait_for_selector('#chart.js-plotly-plot',timeout=30000)
        page.wait_for_timeout(400)
        page.screenshot(path=str(ROOT/'outputs/em-plotter-desktop.png'),full_page=True)
        page.set_viewport_size({'width':390,'height':844});page.wait_for_timeout(500)
        assert page.evaluate('document.documentElement.scrollWidth<=window.innerWidth+1');checks.append('mobile has no horizontal overflow')
        page.screenshot(path=str(ROOT/'outputs/em-plotter-mobile.png'),full_page=True)
        browser.close()
    report=dict(checks=checks,browser_errors=errors)
    (ROOT/'analysis/em-browser-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    if errors:raise SystemExit(1)


if __name__=='__main__':main()
