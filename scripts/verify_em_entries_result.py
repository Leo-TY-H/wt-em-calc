"""Browser/API integration with a real F-16A SB/RB comparison."""
import csv,io,json,urllib.request
from pathlib import Path
from playwright.sync_api import sync_playwright
ROOT=Path(__file__).resolve().parents[1];BASE='http://127.0.0.1:8765'
def get(path):
    with urllib.request.urlopen(BASE+path) as r:return r.read()
def main():
    errors=[];checks=[]
    with sync_playwright() as p:
        b=p.chromium.launch(channel='chrome',headless=True);page=b.new_page(viewport={'width':1440,'height':1100})
        page.on('pageerror',lambda e:errors.append(str(e)))
        page.goto(BASE,wait_until='networkidle');page.wait_for_function('state.meta!==null')
        page.locator('#aircraft-search').fill('f_16a_block_10_mod');page.locator('[data-add="f_16a_block_10_mod"]').click()
        page.get_by_text('SB',exact=True).click()
        page.locator('[data-action="duplicate"]').click();page.get_by_text('RB',exact=True).click()
        page.locator('#quality').select_option('quick');page.locator('#speed-min').fill('700');page.locator('#speed-max').fill('800')
        page.locator('#calculate').click()
        page.wait_for_function('state.activeJob!==null||state.data!==null')
        key=page.evaluate('state.activeJob||state.dataJob')
        (ROOT/'analysis/aircraft-entries/api-job.json').write_text(json.dumps(dict(job=dict(id=key),config=page.evaluate('readConfig()')),indent=2))
        page.wait_for_function('state.data!==null||!$("error").hidden',timeout=240000)
        assert page.locator('#error').is_hidden(),page.locator('#error').inner_text()
        if page.evaluate('!!state.data.preview'):
            assert page.locator('[data-export="samples.csv"]').get_attribute('aria-disabled')=='true'
            checks.append('Live preview renders with entry identities and disables unfinished exports')
        page.wait_for_function('!state.running',timeout=240000)
        assert page.locator('#error').is_hidden(),page.locator('#error').inner_text()
        assert page.locator('#stale').is_hidden()
        data=json.loads(get(f'/api/jobs/{key}/chart.json'))
        assert len(data['aircraft'])==2 and all(a['aircraft_id']=='f_16a_block_10_mod' for a in data['aircraft'])
        assert [a['settings']['instructor'] for a in data['aircraft']]==[False,True]
        assert [a['settings']['torque_gyro'] for a in data['aircraft']]==[True,False]
        assert [a['name'].rsplit(' · ',1)[-1] for a in data['aircraft']]==['SB','RB']
        assert [a['id'] for a in data['aircraft']]==['entry_1','entry_2']
        checks.append('Calculate submits duplicate entries; final plot retains independent Instructor settings without a stale warning')
        for a in data['aircraft']:
            page.locator(f'[data-view="{a["id"]}"]').click()
            page.evaluate('(id)=>{const a=state.data.aircraft.find(a=>a.id===id);return inspect(a,a.points.find(p=>p.valid));}',a['id'])
            assert a['name'] in page.locator('#point-title').inner_text()
            assert page.locator('#error').is_hidden()
            point=next(p for p in a['points'] if p['valid'])
            detail=json.loads(get(f'/api/jobs/{key}/point/{a["id"]}/{point["detail_index"]}'))
            assert detail['instructor_enabled']==a['settings']['instructor']
        checks.append('Each entry tab and point inspector resolves its own exported detailed points')
        rows=list(csv.DictReader(io.StringIO(get(f'/api/jobs/{key}/samples.csv').decode())))
        assert {r['aircraft'] for r in rows}=={'entry_1','entry_2'}
        assert {r['aircraft_id'] for r in rows}=={'f_16a_block_10_mod'}
        svg=get(f'/api/jobs/{key}/diagram.svg')
        assert b'<svg' in svg and all(a['name'].encode() in svg for a in data['aircraft'])
        checks.append('CSV retains entry and aircraft identities; SVG exports both configurations')
        page.locator('[data-view="compare"]').click()
        page.locator('[data-entry="entry_2"] [data-action="configure"]').click()
        page.screenshot(path=str(ROOT/'analysis/aircraft-entries/comparison.png'),full_page=True)
        page.evaluate('(id)=>loadData(id,true)',key)
        assert page.locator('.aircraft-entry').count()==2 and page.locator('#stale').is_hidden()
        page.locator('[data-entry="entry_2"] [data-action="configure"]').click()
        assert page.locator('#flight-mode-rb').is_checked()
        page.evaluate("loadData('2df9747afec8cc80b492',true)")
        assert page.locator('.aircraft-entry').count()==2
        needs_mode_update=page.evaluate('configEntries(state.data.settings).some(e=>e.settings.instructor===e.settings.torque_gyro)')
        assert page.locator('#stale').is_visible()==needs_mode_update
        checks.append('New duplicate results and previous unique-aircraft results restore correctly')
        assert not errors,errors
        b.close()
    report=dict(id=key,checks=checks,errors=errors)
    (ROOT/'analysis/aircraft-entries/result-validation.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
if __name__=='__main__':main()
