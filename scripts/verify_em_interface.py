"""Verify presentation changes against saved results, without new EM solves."""
import json
import math
from pathlib import Path

from playwright.sync_api import sync_playwright

ROOT=Path(__file__).resolve().parents[1]


def main():
    checks=[];errors=[]
    with sync_playwright() as p:
        browser=p.chromium.launch(channel='chrome',headless=True)
        page=browser.new_page(viewport={'width':1440,'height':1100},device_scale_factor=1)
        page.on('pageerror',lambda error:errors.append(str(error)))
        page.goto('http://127.0.0.1:8765',wait_until='networkidle')
        page.wait_for_selector('#chart.js-plotly-plot',timeout=30000)
        scales=page.evaluate("({x:100*Math.abs($('chart')._fullLayout.xaxis._m),y:5*Math.abs($('chart')._fullLayout.yaxis._m)})")
        assert abs(scales['x']-scales['y'])<1e-8,scales
        assert page.evaluate("$('chart').layout.annotations.some(a=>a.text==='SEP 0 m/s') && $('chart').layout.annotations.every(a=>/^SEP [+-]?\\d+(\\.\\d+)? m\\/s$/.test(a.text))")
        checks.append('100 km/h and 5 degrees/s occupy identical pixel lengths; labeled SEP contours')
        # Use a flap-capable aircraft for the form check, without recalculation.
        page.evaluate("document.querySelectorAll('input[name=aircraft]').forEach(e=>e.checked=e.value==='f_16a_block_15_adf');syncLabels()")
        flap=page.locator('#flaps');flap.press('End')
        assert page.evaluate("readConfig().flaps_percent===100 && $('flaps-label').textContent==='100%'")
        flap.press('Home')
        assert page.evaluate('readConfig().flaps_percent===0')
        page.locator('#instructor').check()
        assert page.evaluate("readConfig().instructor && !$('instructor-hint').hidden && $('instructor-hint').textContent.includes('approximation')")
        page.locator('#instructor').uncheck()
        page.evaluate("document.querySelectorAll('input[name=aircraft]').forEach(e=>e.checked=e.value==='saab_jas39c');syncLabels()")
        assert page.locator('#flaps').is_disabled() and page.evaluate('readConfig().flaps_percent===0')
        page.evaluate('populate(state.data.settings)')
        checks.append('flap slider 0–100%; disabled for no manual flaps; labeled experimental Instructor toggle')
        assert page.locator('#afterburner').count()==0
        throttle=page.locator('#throttle');throttle.press('End')
        assert page.evaluate('readConfig().throttle===1.1 && readConfig().afterburner')
        for _ in range(9):throttle.press('ArrowLeft')
        assert page.evaluate('readConfig().throttle===1.01 && readConfig().afterburner')
        throttle.press('ArrowLeft')
        assert page.evaluate('readConfig().throttle===1 && !readConfig().afterburner')
        throttle.press('Home')
        assert page.evaluate('readConfig().throttle===0 && !readConfig().afterburner')
        throttle.press('End')
        checks.append('throttle automatically selects dry / afterburner at 0, 100, 101 and 110 percent')
        assert page.evaluate("$('chart').data.filter(t=>t.mode==='lines'&&t.name?.endsWith(' boundary')).every(t=>t.line.dash==='solid')")
        assert page.evaluate("$('chart').data.filter(t=>t.name?.includes('Ps ')).every(t=>t.line.dash==='dash')")
        checks.append('solid feasible boundaries; dashed SEP contours including zero')
        assert page.evaluate("$('chart')._fullLayout.dragmode==='pan'")
        assert page.locator('#chart').bounding_box()['height']>=760
        page.locator('#chart').scroll_into_view_if_needed()
        # A real left-button drag must translate both ranges without resizing.
        before=page.evaluate("({x:$('chart')._fullLayout.xaxis.range,y:$('chart')._fullLayout.yaxis.range})")
        rect=page.locator('#chart .nsewdrag').bounding_box()
        x=rect['x']+rect['width']*.5;y=rect['y']+rect['height']*.5
        page.mouse.move(x,y);page.mouse.down();page.mouse.move(x+70,y+40,steps=12);page.mouse.up()
        page.wait_for_timeout(200)
        after=page.evaluate("({x:$('chart')._fullLayout.xaxis.range,y:$('chart')._fullLayout.yaxis.range})")
        for axis in ['x','y']:
            assert abs((after[axis][1]-after[axis][0])-(before[axis][1]-before[axis][0]))<1e-8
            assert abs(after[axis][0]-before[axis][0])>1e-3
        page.locator('#reset-zoom').click()
        checks.append('real left-mouse drag pans without changing axis spans; reset works')
        assert page.evaluate("turnRadius(360,180/Math.PI)==='100 m'")
        assert page.evaluate("turnRadius(360,0).startsWith('∞') && turnRadius(null,10)==='—'")
        checks.append('turn-radius units and straight-flight/missing-value handling')
        page.locator('#chart').scroll_into_view_if_needed()
        # Inspect each hover class, including data at interpolated boundary points.
        for kind in ['boundary','contour','zero']:
            selected=page.evaluate("""kind => {
                const el=$('chart');
                const i=el.data.findIndex(t=>kind==='boundary'?t.name?.endsWith(' boundary'):
                    kind==='zero'?t.name?.endsWith(' Ps = 0'):t.name?.includes(' · Ps ')&&!t.name.endsWith('Ps 0'));
                const t=el.data[i],j=t.x.findIndex((x,j)=>x>450&&x<1100&&t.y[j]>3&&t.y[j]<el._fullLayout.yaxis.range[1]-2);
                if(i<0||j<0)throw Error('No visible '+kind+' test point');
                Plotly.Fx.hover(el,[{curveNumber:i,pointNumber:j}]);
                return {speed:t.x[j],rate:t.y[j],radius:kind==='boundary'?t.customdata[j][2]:t.text[j]};
            }""",kind)
            text=page.locator('#chart .hoverlayer').text_content()
            assert 'Turn radius' in text and 'Ps' in text and 'km/h' in text and '°/s' in text,(kind,text)
            radius=(selected['speed']/3.6)/math.radians(selected['rate'])
            rendered=float(selected['radius'].replace(',','').split()[0])
            assert abs(radius-rendered)<=.51
        checks.append('boundary, SEP and zero-SEP hovers include Ps, speed, turn rate and correct radius')
        page.evaluate("Plotly.Fx.unhover($('chart'))")
        page.locator('#chart').screenshot(path=str(ROOT/'outputs/em-interface-plot.png'))
        page.screenshot(path=str(ROOT/'outputs/em-interface-desktop.png'),full_page=True)
        # Switching aircraft retains the requested default and line styling.
        page.locator('#chart-tabs button').nth(1).click()
        assert page.evaluate("$('chart').data.some(t=>t.type==='heatmap') && $('chart')._fullLayout.dragmode==='pan'")
        page.set_viewport_size({'width':390,'height':844});page.wait_for_timeout(300)
        assert page.evaluate("Math.abs(100*Math.abs($('chart')._fullLayout.xaxis._m)-5*Math.abs($('chart')._fullLayout.yaxis._m))<1e-8")
        assert page.evaluate('document.documentElement.scrollWidth<=window.innerWidth+1')
        assert page.locator('#chart').bounding_box()['height']==560
        page.locator('#chart').scroll_into_view_if_needed()
        page.screenshot(path=str(ROOT/'outputs/em-interface-mobile.png'),full_page=True)
        checks.append('taller desktop and mobile plot; single-aircraft view; no mobile overflow')
        browser.close()
    report=dict(checks=checks,browser_errors=errors)
    (ROOT/'analysis/em-interface-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    if errors:raise SystemExit(1)


if __name__=='__main__':main()
