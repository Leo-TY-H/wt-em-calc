"""Local rendered-chart checks for adaptive sampling and shared flight modes."""
import argparse,json
from pathlib import Path
from playwright.sync_api import sync_playwright
ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--out',required=True)
    parser.add_argument('--latest',default=str(ROOT/'outputs/em/latest.json'));args=parser.parse_args()
    out=Path(args.out);out.mkdir(parents=True,exist_ok=True);errors=[];checks=[]
    with sync_playwright() as p:
        browser=p.chromium.launch(channel='chrome',headless=True)
        page=browser.new_page(viewport={'width':1550,'height':1120})
        page.on('pageerror',lambda error:errors.append(str(error)))
        page.goto('http://127.0.0.1:8765',wait_until='networkidle')
        # The application intentionally opens an empty workspace. Load the
        # completed API fixture through its normal result-loading function.
        page.wait_for_function('state.meta !== null')
        job=json.loads(Path(args.latest).read_text())['id']
        page.evaluate('(id)=>loadData(id,true)',job)
        page.wait_for_selector('#chart.js-plotly-plot',timeout=30000)
        page.wait_for_function("document.getElementById('chart').data.some(t=>t.x?.length>5)")
        checks.append('completed API result loaded and rendered')
        buttons=page.locator('#chart-tabs button')
        if buttons.count()>1:buttons.nth(1).click()
        page.wait_for_function("document.getElementById('chart').data.some(t=>t.type==='heatmap')")
        count=page.evaluate("document.getElementById('chart').data.filter(t=>t.mode==='lines').length")
        assert count>3,count;checks.append('single-aircraft heatmap, boundary and SEP paths rendered')
        hover=page.evaluate('''()=>{
          const chart=document.getElementById('chart'),curve=chart.data.findIndex(t=>t.type==='heatmap');
          const trace=chart.data[curve];
          for(let j=0;j<trace.z.length;j++)for(let i=trace.x.length-1;i>=0;i--){
            if(trace.z[j][i]!==null && Number.isFinite(trace.customdata[j][i])){
              Plotly.Fx.hover(chart,[{curveNumber:curve,pointNumber:[j,i]}]);
              return {template:trace.hovertemplate,value:trace.customdata[j][i],speed:trace.x[i]};
            }
          }
          return null;
        }''')
        assert hover and 'Flaps' in hover['template'] and 'requested' in hover['template'],hover
        assert 0.<=hover['value']<=100.,hover
        page.wait_for_function("document.querySelector('.hoverlayer')?.textContent.includes('Flaps')")
        assert 'requested' in page.locator('.hoverlayer').text_content()
        checks.append('actual Plotly hover displays effective and requested flaps')
        page.locator('#chart').screenshot(path=str(out/'flap-hover.png'))
        page.evaluate("Plotly.Fx.unhover(document.getElementById('chart'))")
        page.locator('#chart').screenshot(path=str(out/'chart.png'))
        page.locator('#show-samples').check();page.locator('#show-rejected').check()
        page.wait_for_function("document.getElementById('chart').data.some(t=>t.mode==='markers')")
        checks.append('sample and rejection overlays remain available')
        page.locator('#show-samples').uncheck();page.locator('#show-rejected').uncheck()
        saved=page.evaluate('readConfig()')
        page.locator('#quality').select_option('quick')
        assert page.locator('#stale').is_visible()
        page.locator('#quality').select_option('fine')
        config=page.evaluate('readConfig()')
        assert config['speed_samples']==9 and config['sep_tolerance_mps']==.15
        assert 'km/h' in page.locator('#sampling-hint').inner_text()
        checks.append('Detailed uses adaptive nine-speed seed, reports both position targets, and flags changed settings')
        page.locator('#reset').click();assert page.locator('#speed-min').input_value()=='100'
        page.evaluate("addEntry('spitfire_ix_usa')")
        assert page.locator('#flight-mode-rb').is_checked()
        entry=page.evaluate('readConfig().entries[0].settings')
        assert entry['instructor'] and not entry['torque_gyro']
        page.locator('label:has(#flight-mode-sb)').click()
        entry=page.evaluate('readConfig().entries[0].settings')
        assert not entry['instructor'] and entry['torque_gyro']
        checks.append('100 km/h default and RB/SB linked settings')
        page.evaluate('(c)=>populate(c)',saved)
        page.set_viewport_size({'width':390,'height':844})
        page.wait_for_function('document.documentElement.scrollWidth<=innerWidth+1')
        checks.append('mobile viewport has no horizontal overflow')
        page.screenshot(path=str(out/'mobile.png'),full_page=True)
        assert not errors,errors;browser.close()
    report=dict(status='PASS',checks=checks,browser_errors=errors)
    (out/'browser.json').write_text(json.dumps(report,indent=2)+'\n');print(report)


if __name__=='__main__':main()
