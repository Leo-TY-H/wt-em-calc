"""Per-aircraft engine management controls, old-result labels and request payloads."""
import argparse,json
from pathlib import Path
from playwright.sync_api import sync_playwright
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'analysis/prop-performance'

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--url',default='http://127.0.0.1:8766');args=ap.parse_args();errors=[]
    with sync_playwright() as p:
        browser=p.chromium.launch(channel='chrome',headless=True);page=browser.new_page(viewport={'width':1440,'height':1100})
        page.on('pageerror',lambda e:errors.append(str(e)));page.goto(args.url,wait_until='networkidle')
        def select(name):
            page.locator('#aircraft-search').fill(name);page.locator(f'input[name=aircraft][value="{name}"]').check()
        assert page.locator('input[name=aircraft]:checked').count()==0
        select('yak-3');assert page.locator('#engine-control-mode').input_value()=='automatic'
        page.locator('#engine-control-mode').select_option('optimized');select('bf-109f-4')
        page.locator('#settings-aircraft').select_option('bf-109f-4')
        assert page.locator('#engine-control-mode').input_value()=='automatic'
        page.locator('#settings-aircraft').select_option('yak-3')
        assert page.locator('#engine-control-mode').input_value()=='optimized'
        config=page.evaluate('readConfig()')
        assert config['aircraft_settings']['yak-3']['engine_control_mode']=='optimized'
        assert config['aircraft_settings']['bf-109f-4']['engine_control_mode']=='automatic'
        page.locator('#reset').click();assert page.locator('input[name=aircraft]:checked').count()==0
        select('yak-3');assert page.locator('#engine-control-mode').input_value()=='automatic'
        page.screenshot(path=str(OUT/'automatic-controls-desktop.png'),full_page=True)
        page.set_viewport_size({'width':390,'height':844})
        assert page.locator('#engine-control-field').is_visible()
        assert page.evaluate('document.documentElement.scrollWidth<=innerWidth')
        page.screenshot(path=str(OUT/'automatic-controls-mobile.png'),full_page=True)
        # Historic settings lacking the new field must retain their MEC meaning.
        old=json.loads((ROOT/'outputs/em/0215977e7f64113f6652/data.json').read_text())['settings']
        page.evaluate('(c)=>populate(c)',old)
        assert page.locator('#engine-control-mode').input_value()=='optimized'
        page.locator('#reset').click();select('f_16a_block_15_adf')
        assert page.locator('#engine-control-field').is_hidden()
        # The inspector shows the settled stage rather than initial stage zero.
        point=next(r for r in json.loads((OUT/'automatic-points.json').read_text()) if r['aircraft']=='p-47d-28')
        a=dict(id='p-47d-28',name='P-47',engine=dict(policy='Native automatic engine controls'),settings=dict(structural_limits=True))
        page.evaluate('({a,p})=>inspect(a,p)',dict(a=a,p=point))
        content=page.locator('#point-content').inner_text();assert 'Automatic' in content and 'Idealized manual' not in content
        assert not errors,errors;browser.close()
    report=dict(status='PASS',checks=['automatic default and reset','independent per-aircraft modes in API payload','legacy results retain manual label','jet selector hidden','desktop/mobile layout','automatic inspector'],errors=errors)
    (OUT/'browser-validation.json').write_text(json.dumps(report,indent=2)+'\n');print(report)
if __name__=='__main__':main()
