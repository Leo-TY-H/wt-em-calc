import json
from pathlib import Path
from playwright.sync_api import sync_playwright
out=Path(__file__).resolve().parents[1]/'analysis/prop-performance'
key=json.loads((out/'default-chart.json').read_text())['job'];errors=[]
with sync_playwright() as p:
 b=p.chromium.launch(channel='chrome',headless=True);page=b.new_page(viewport={'width':1440,'height':1100});page.on('pageerror',lambda e:errors.append(str(e)))
 page.goto('http://127.0.0.1:8765',wait_until='networkidle');page.evaluate('(id)=>loadData(id,true)',key)
 page.wait_for_function('(id)=>state.dataJob===id',arg=key,timeout=60000)
 assert page.locator('#engine-control-mode').input_value()=='automatic'
 assert page.locator('#stale').is_hidden()
 assert page.locator('.metric').count()==2
 page.locator('#engine-control-mode').select_option('optimized');assert page.locator('#stale').is_visible()
 page.locator('#engine-control-mode').select_option('automatic');assert page.locator('#stale').is_hidden()
 page.screenshot(path=str(out/'automatic-full-chart.png'),full_page=True)
 assert not errors,errors;b.close()
(out/'full-chart-browser.json').write_text(json.dumps(dict(status='PASS',job=key,checks=['complete chart rendered','automatic mode restored','selection order does not cause a false stale warning','engine mode changes mark the plot stale'],errors=errors),indent=2)+'\n')
print('PASS',key)
