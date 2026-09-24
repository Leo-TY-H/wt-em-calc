"""Local browser integration check for same-aircraft Instructor comparison."""
from playwright.sync_api import sync_playwright
import json
from pathlib import Path
with sync_playwright() as p:
    browser=p.chromium.launch(channel='chrome',headless=True)
    page=browser.new_page(viewport={'width':1440,'height':1100})
    errors=[]
    page.on('pageerror',lambda e:errors.append(str(e)))
    page.goto('http://127.0.0.1:8765',wait_until='networkidle')
    page.wait_for_function('state.meta!==null')
    name='f_16a_block_10_mod'
    page.locator('#aircraft-search').fill(name)
    page.locator(f'input[name=aircraft][value="{name}"]').check()
    page.locator('#compare-instructor').check()
    assert page.locator('#instructor').is_disabled()
    assert page.locator('input[name=aircraft]:not(:checked):enabled').count()==0
    page.locator('#fuel').fill('45')
    page.locator('#speed-min').fill('700');page.locator('#speed-max').fill('800')
    page.locator('#quality').select_option('quick')
    page.locator('#calculate').click()
    page.wait_for_function('state.data!==null || !$("error").hidden',timeout=240000)
    assert page.locator('#error').is_hidden(),page.locator('#error').inner_text()
    data=page.evaluate('state.data')
    assert len(data['aircraft'])==2
    assert len({a['id'] for a in data['aircraft']})==2
    assert {a['settings']['instructor'] for a in data['aircraft']}=={False,True}
    assert all(a['settings']['fuel_percent']==45 for a in data['aircraft'])
    assert not page.locator('#stale').is_visible()
    for a in data['aircraft']:
        page.locator(f'[data-view="{a["id"]}"]').click()
        page.evaluate('(a)=>inspect(a,a.points.find(p=>p.valid))',a)
        assert a['name'] in page.locator('#point-title').inner_text()
    key=page.evaluate('state.dataJob')
    page.evaluate('(key)=>loadData(key,true)',key)
    assert page.locator('#compare-instructor').is_checked()
    assert not page.locator('#stale').is_visible()
    page.locator('[data-view=compare]').click()
    page.screenshot(path=str(Path(__file__).resolve().parents[1]/'outputs/em-instructor-comparison.png'),full_page=True)
    page.locator('#compare-instructor').uncheck()
    assert page.locator('#stale').is_visible()
    assert page.locator('#instructor').is_enabled()
    page.locator('#reset').click()
    assert not page.locator('#compare-instructor').is_checked()
    assert page.locator('input[name=aircraft]:checked').count()==0
    assert not errors,errors
    print(json.dumps({'job':key,'checks':'Real adaptive comparison, chart tabs, both point endpoints, saved settings, stale warning and reset passed','errors':errors}))
    browser.close()
