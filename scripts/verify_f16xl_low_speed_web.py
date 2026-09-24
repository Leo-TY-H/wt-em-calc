"""Calculate a fresh default F-16XL with Instructor off in the real website."""
import json,time,urllib.request
from pathlib import Path
from playwright.sync_api import sync_playwright

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'analysis/f16xl-low-speed'
BASE='http://127.0.0.1:8765'


def get(path):
    with urllib.request.urlopen(BASE+path,timeout=30) as response:return response.read()


def main():
    errors=[];first_preview=None
    with sync_playwright() as p:
        browser=p.chromium.launch(channel='chrome',headless=True)
        page=browser.new_page(viewport={'width':1440,'height':1100})
        page.on('pageerror',lambda e:errors.append(str(e)))
        page.goto(BASE,wait_until='networkidle');page.wait_for_function('state.meta!==null')
        page.locator('#aircraft-search').fill('f_16xl')
        page.locator('[data-add="f_16xl"]').click()
        page.locator('#instructor').uncheck()
        page.locator('#quality').select_option('standard')
        cfg=page.evaluate('readConfig()')
        assert cfg['speed_min_kmh']==150. and cfg['speed_max_kmh']==1300.
        assert cfg['entries'][0]['settings']['instructor'] is False
        start=time.monotonic();page.locator('#calculate').click()
        page.wait_for_function('state.activeJob!==null||state.data!==null')
        key=page.evaluate('state.activeJob||state.dataJob')
        deadline=start+150
        while page.evaluate('state.running') and time.monotonic()<deadline:
            if first_preview is None and page.evaluate('!!state.data?.preview'):
                first_preview=time.monotonic()-start
            page.wait_for_timeout(250)
        assert not page.evaluate('state.running'),'F-16XL calculation did not finish'
        assert page.locator('#error').is_hidden(),page.locator('#error').inner_text()
        ready=time.monotonic()-start
        job=json.loads(get('/api/jobs/'+key))
        assert job['status']=='complete',job
        data=json.loads(get('/api/jobs/'+key+'/data.json'))
        chart=json.loads(get('/api/jobs/'+key+'/chart.json'))
        aircraft=data['aircraft'][0]
        assert aircraft['aircraft_id']=='f_16xl' and not aircraft['settings']['instructor']
        low=aircraft['columns'][0]
        assert low['speed_kmh']==150. and low['boundary_status']=='verified limit'
        assert any(p['load_g']==1. and p['valid'] for p in low['points'])
        gaps=[c for c in aircraft['columns'] if c['numerical_gap_brackets']]
        assert gaps and all(c['boundary_status']=='verified limit' for c in gaps)
        assert not page.evaluate('state.data.preview')
        assert page.locator('#stale').is_hidden()
        visible=chart['aircraft'][0]
        point=next(p for p in visible['points'] if p['valid'] and p['speed_kmh']==150. and p['load_g']==1.)
        detail=json.loads(get(f'/api/jobs/{key}/point/{visible["id"]}/{point["detail_index"]}'))
        assert detail['valid'] and detail['instructor_enabled'] is False
        assert len(get(f'/api/jobs/{key}/samples.csv'))>1000
        page.screenshot(path=str(OUT/'f16xl-off.png'),full_page=True)
        assert not errors,errors
        report=dict(status='PASS',id=key,ready_s=ready,first_preview_s=first_preview,
            solver_s=data['elapsed_s'],columns=len(aircraft['columns']),points=len(aircraft['points']),
            gap_speeds_kmh=[c['speed_kmh'] for c in gaps],settings=cfg,errors=errors,
            checks=['default Standard F-16XL Instructor off finishes', 'live chart preview',
                    '150 km/h level flight and upper boundary', 'numerical gaps retained',
                    'final chart, point endpoint and CSV export'])
        (OUT/'browser-validation.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2));browser.close()


if __name__=='__main__':main()
