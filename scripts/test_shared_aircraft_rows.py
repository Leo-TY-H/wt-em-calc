"""Browser regression: engine-created aircraft must calculate after changing tools.

Requires Playwright and Chrome. Run with --url URL; --asset-dir app overrides
engine.js for a pre-deployment check. --expect-error reproduces the old release.
"""
import argparse,json,re
from pathlib import Path
from playwright.sync_api import sync_playwright

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url',default='https://neothunderism.pages.dev')
    parser.add_argument('--asset-dir',type=Path)
    parser.add_argument('--expect-error',action='store_true')
    args=parser.parse_args()
    with sync_playwright() as p:
        browser=p.chromium.launch(channel='chrome',headless=True,args=['--disk-cache-size=1000000'])
        context=browser.new_context(viewport=dict(width=1440,height=1100));page=context.new_page();errors=[]
        page.on('pageerror',lambda e:errors.append(str(e)))
        if args.asset_dir:
            page.route('**/engine.js',lambda route:route.fulfill(path=args.asset_dir/'engine.js',content_type='application/javascript'))
        def tool(name):
            page.locator('[aria-controls="nav-fm"]').click();page.locator('#'+name+'-tab').click()
        def rows():return page.evaluate('state.entries')
        def valid(entries):
            ids=[e['id'] for e in entries]
            assert len(set(ids))==len(ids) and all(re.fullmatch(r'entry_[A-Za-z0-9_-]{1,48}',i) for i in ids),ids
        try:
            page.goto(args.url,wait_until='domcontentloaded',timeout=60000)
            page.wait_for_function('state.meta && !document.querySelector("#calculate").disabled',timeout=180000)
            versions=page.evaluate('async()=>({latest:GameVersions.default,primary:(await GameVersions.load()).primary})')
            tool('engine');page.locator('#engine-search').fill('f_16a_block_10')
            page.locator('#engine-list [data-add="f_16a_block_10"]').click()
            tool('em');first=rows();assert len(first)==1
            if args.expect_error:
                with page.expect_response(lambda r:r.url.endswith('/api/jobs') and r.request.method=='POST',timeout=60000) as pending:
                    page.locator('#calculate').click()
                response=pending.value;body=response.json()
                assert response.status==400 and body.get('error')=='Aircraft entry IDs must be unique entry_ identifiers',(response.status,body)
                print(json.dumps(dict(reproduced=True,entries=first,status=response.status,message=body['error'])))
                return
            valid(first)
            page.locator('#selected-aircraft [data-action="configure"]').click()
            page.locator('#fuel').fill('47')
            page.locator('#fuel').dispatch_event('input')
            page.locator('#entry-version .version-search').fill(versions['primary'])
            page.locator('#entry-version [data-version="'+versions['primary']+'"]').click()
            page.wait_for_function('(v)=>state.entries[0].version===v && GameVersions.meta(v)!==null',arg=versions['primary'])
            page.locator('#entry-dialog-close').click()
            page.locator('#aircraft-search').fill('f_16a_block_10')
            page.locator('#aircraft-list [data-add="f_16a_block_10"]').click()
            before=rows();valid(before);assert len(before)==2
            tool('engine')
            engine_ids=page.locator('#engine-selected [data-entry]').evaluate_all('(els)=>els.map(e=>e.dataset.entry)')
            assert engine_ids==[e['id'] for e in before]
            page.locator('#engine-selected [data-action="duplicate"]').first.click()
            tool('em');after=rows();valid(after);assert len(after)==3 and after[:2]==before
            assert page.evaluate('(id)=>state.conditions[id].fuel_percent',first[0]['id'])==47
            page.locator('#selected-aircraft [data-action="duplicate"]').nth(1).click()
            after=rows();valid(after);assert len(after)==4
            tool('altitude');assert rows()==after
            tool('em');assert rows()==after
            page.locator('#speed-min').fill('600');page.locator('#speed-max').fill('900')
            page.locator('#calculate').click()
            page.wait_for_function('!state.running && state.data?.aircraft.length===4',timeout=240000)
            result=page.evaluate('''()=>({rows:state.data.aircraft.map(a=>({id:a.id,version:a.version_label,points:a.boundary.length,
              native:a.chart_solver_work.native.status})),entries:state.entries,default_build:GameVersions.default})''')
            assert len({r['id'] for r in result['rows']})==4
            assert {r['version'] for r in result['rows']}=={versions['latest'],versions['primary']},result
            assert all(r['points']>2 and r['native'].startswith('enabled') for r in result['rows']),result
            assert not errors,errors
            print(json.dumps(dict(passed=True,engine_to_em=True,duplication=True,altitude_roundtrip=True,
              settings_preserved=True,mixed_versions=True,**result)))
        finally:context.close();browser.close()

if __name__=='__main__':main()
