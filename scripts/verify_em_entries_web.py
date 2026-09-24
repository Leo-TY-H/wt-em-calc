"""Real browser checks for search/add/configure and duplicate settings."""
import json
from pathlib import Path
from playwright.sync_api import sync_playwright
ROOT=Path(__file__).resolve().parents[1]
def main():
    errors=[];checks=[]
    with sync_playwright() as p:
        b=p.chromium.launch(channel='chrome',headless=True);page=b.new_page(viewport={'width':1440,'height':1100})
        page.on('pageerror',lambda e:errors.append(str(e)))
        page.goto('http://127.0.0.1:8765',wait_until='networkidle')
        page.wait_for_function('state.meta!==null')
        assert page.locator('.aircraft-entry').count()==0
        assert page.locator('#compare-instructor').count()==0
        assert page.locator('#speed-min').input_value()=='100'
        assert page.locator('#instructor, #torque-gyro').count()==0
        search=page.locator('#aircraft-search');search.fill('yak-3')
        page.locator('[data-add="yak-3"]').click()
        first=page.evaluate('state.editing')
        page.locator('#altitude').fill('1000');page.locator('#fuel').press('End')
        assert page.locator('#flight-mode-rb').is_checked()
        page.locator(f'[data-entry="{first}"] [data-action="duplicate"]').click()
        second=page.evaluate('state.editing')
        assert second!=first
        assert page.locator('#altitude').input_value()=='1000'
        assert page.locator('#fuel').input_value()=='100'
        assert page.locator('#flight-mode-rb').is_checked()
        page.get_by_text('SB',exact=True).click();page.locator('#altitude').fill('3000')
        page.locator(f'[data-entry="{first}"] [data-action="configure"]').click()
        assert page.locator('#altitude').input_value()=='1000'
        assert page.locator('#flight-mode-rb').is_checked()
        page.locator('[data-add="yak-3"]').click()
        third=page.evaluate('state.editing')
        assert page.locator('#altitude').input_value()=='0'
        assert page.locator('#fuel').input_value()=='30'
        assert page.locator('#flight-mode-rb').is_checked()
        config=page.evaluate('readConfig()')
        assert len(config['entries'])==3 and len(config['aircraft'])==1
        assert [e['settings']['altitude_m'] for e in config['entries']]==[1000,3000,0]
        assert [e['settings']['instructor'] for e in config['entries']]==[True,False,True]
        assert [e['settings']['torque_gyro'] for e in config['entries']]==[False,True,False]
        from em_solver import settings
        validated=settings(config)
        assert validated['speed_min_kmh']==100
        assert [e['settings']['torque_gyro'] for e in validated['entries']]==[False,True,False]
        checks.append('Search adds repeated aircraft; duplicate copies settings; new additions use defaults; edits remain independent')
        page.evaluate('(c)=>populate(c)',config)
        assert page.evaluate('(c)=>signature(c)===signature(readConfig())',config)
        page.locator(f'[data-entry="{second}"] [data-action="remove"]').click()
        assert page.evaluate('state.entries.length')==2
        page.locator(f'[data-entry="{third}"] [data-action="configure"]').click()
        page.locator(f'[data-entry="{third}"] [data-action="remove"]').click()
        assert page.evaluate('state.editing')==first
        assert page.locator('#altitude').input_value()=='1000'
        checks.append('Saved entry config round-trips; removing active or inactive entries preserves remaining settings')
        search.fill('no such plane xyz');assert page.locator('#search-empty').is_visible()
        assert page.locator('.aircraft-entry').count()==1
        search.fill('f_14a_early');page.locator('[data-add="f_14a_early"]').click()
        assert page.locator('#sweep-field').is_visible()
        assert not page.locator('#engine-control-field').is_visible()
        assert page.get_by_role('radiogroup',name='Flight mode').is_visible()
        # Native radio keyboard navigation must change both solver settings.
        page.locator('#flight-mode-rb').focus();page.keyboard.press('ArrowRight')
        assert page.locator('#flight-mode-sb').is_checked()
        page.locator('#apply-settings-to-all').click()
        assert all(not e['settings']['instructor'] and e['settings']['torque_gyro']
                   for e in page.evaluate('readConfig()')['entries'])
        checks.append('RB/SB couples Instructor and torque for propellers and jets, including keyboard input and apply-to-all; API validates the pairs')
        for _ in range(6):page.locator('[data-add="f_14a_early"]').click()
        assert page.locator('.aircraft-entry').count()==8
        assert page.locator('[data-add="f_14a_early"]').is_disabled()
        assert page.locator('[data-action="duplicate"]:enabled').count()==0
        checks.append('Search leaves added entries visible; aircraft-specific controls and eight-entry limit work')
        page.locator('#speed-min').fill('250')
        page.locator('#reset').click();assert page.locator('.aircraft-entry').count()==0
        assert page.locator('#speed-min').input_value()=='100'
        search.fill('yak-3');page.locator('[data-add="yak-3"]').click()
        assert page.locator('#flight-mode-rb').is_checked()
        page.locator('[data-action="duplicate"]').click();page.get_by_text('SB',exact=True).click()
        page.screenshot(path=str(ROOT/'analysis/aircraft-entries/desktop.png'),full_page=True)
        page.set_viewport_size({'width':390,'height':844})
        assert page.evaluate('document.documentElement.scrollWidth<=innerWidth')
        page.screenshot(path=str(ROOT/'analysis/aircraft-entries/mobile.png'),full_page=True)
        checks.append('Reset restores defaults; desktop/mobile layouts render without horizontal overflow')
        assert not errors,errors
        b.close()
    report=dict(checks=checks,errors=errors)
    (ROOT/'analysis/aircraft-entries/browser-validation.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))
if __name__=='__main__':main()
