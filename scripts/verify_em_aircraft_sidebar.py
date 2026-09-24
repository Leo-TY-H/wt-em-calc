"""Local-only browser checks for independent aircraft conditions and reset."""
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
        assert page.locator('input[name=aircraft]:checked').count()==0
        assert page.locator('#altitude').is_disabled()
        def select(name):
            page.locator('#aircraft-search').fill(name)
            page.locator(f'input[name=aircraft][value="{name}"]').check()
        select('f_16a_block_10_mod')
        page.locator('#altitude').fill('1000');page.locator('#fuel').press('End')
        page.locator('#throttle').press('Home');page.locator('#flaps').press('End');page.locator('#instructor').check()
        select('f_14a_early')
        page.locator('#settings-aircraft').select_option('f_14a_early')
        assert page.locator('#altitude').input_value()=='0'
        assert page.locator('#fuel').input_value()=='30'
        assert not page.locator('#instructor').is_checked()
        assert page.locator('#sweep-field').is_visible()
        page.locator('#altitude').fill('3000');page.locator('#sweep').press('End')
        page.locator('#settings-aircraft').select_option('f_16a_block_10_mod')
        assert page.locator('#altitude').input_value()=='1000'
        assert page.locator('#fuel').input_value()=='100'
        assert page.locator('#throttle').input_value()=='0'
        assert page.locator('#flaps').input_value()=='100'
        assert page.locator('#instructor').is_checked()
        assert not page.locator('#sweep-field').is_visible()
        config=page.evaluate('readConfig()')
        assert config['aircraft_settings']['f_16a_block_10_mod']['altitude_m']==1000
        assert config['aircraft_settings']['f_14a_early']['sweep_percent']==100
        assert config['aircraft_settings']['f_14a_early']['afterburner'] is True
        assert config['aircraft_settings']['f_16a_block_10_mod']['afterburner'] is False
        checks.append('Independent altitude, fuel, throttle/AB, flaps, Instructor and sweep survive aircraft switching')
        assert page.locator('#selected-aircraft input:checked').count()==2
        page.locator('input[name=aircraft][value="f_16a_block_10_mod"]').uncheck()
        assert page.locator('#settings-aircraft').input_value()=='f_14a_early'
        select('f_16a_block_10_mod');page.locator('#settings-aircraft').select_option('f_16a_block_10_mod')
        assert page.locator('#fuel').input_value()=='100'
        checks.append('Selected aircraft stay pinned; deselect/reselect retains conditions')
        page.locator('#reset').click()
        assert page.locator('input[name=aircraft]:checked').count()==0
        select('f_16a_block_10_mod')
        assert page.locator('#fuel').input_value()=='30' and page.locator('#flaps').input_value()=='0'
        assert page.locator('#altitude').input_value()=='0'
        checks.append('Reset clears selections and restores all aircraft defaults')
        page.evaluate("loadData('9815d98a6714d59b4cd3',true)")
        page.wait_for_selector('#chart.js-plotly-plot')
        assert page.locator('#flaps').input_value()=='100'
        assert not page.locator('#stale').is_visible()
        assert page.evaluate("$('chart')._fullLayout.dragmode==='pan'")
        scales=page.evaluate("({x:100*Math.abs($('chart')._fullLayout.xaxis._m),y:5*Math.abs($('chart')._fullLayout.yaxis._m)})")
        assert abs(scales['x']-scales['y'])<1e-8
        checks.append('Legacy saved plot restores effective conditions without stale warning; pan and equal axis units preserved')
        page.screenshot(path=str(ROOT/'analysis/em-three-tasks/sidebar.png'),full_page=True)
        api_result=ROOT/'analysis/em-three-tasks/api-validation.json'
        if api_result.exists():
            key=json.loads(api_result.read_text())['id']
            page.evaluate('(id)=>loadData(id,true)',key)
            page.wait_for_function('(id)=>state.dataJob===id',arg=key)
            page.locator('#settings-aircraft').select_option('f_16a_block_10_mod')
            assert page.locator('#altitude').input_value()=='1000' and page.locator('#instructor').is_checked()
            page.locator('#settings-aircraft').select_option('f_14a_early')
            assert page.locator('#altitude').input_value()=='3000' and page.locator('#sweep').input_value()=='50'
            assert not page.locator('#instructor').is_checked() and not page.locator('#stale').is_visible()
            page.screenshot(path=str(ROOT/'analysis/em-three-tasks/independent-conditions.png'),full_page=True)
            checks.append('New mixed-condition API result restores each aircraft correctly without stale warning')
        current=json.loads((ROOT/'analysis/em-three-tasks/recomputed.json').read_text())['id']
        page.evaluate('(id)=>loadData(id,true)',current)
        page.wait_for_function('(id)=>state.dataJob===id',arg=current)
        page.screenshot(path=str(ROOT/'analysis/em-three-tasks/refined-boundary.png'),full_page=True)
        assert not errors,errors
        b.close()
    print(json.dumps(dict(checks=checks,errors=errors),indent=2))
if __name__=='__main__':main()
