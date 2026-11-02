// Browser event integration and recoverable setup failures.
const assert=require('node:assert/strict'),path=require('node:path'),cp=require('node:child_process');
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'../../tools/live-browser/node_modules/playwright');
const root=path.resolve(__dirname,'..');
(async()=>{
  const browser=await chromium.launch({headless:true,args:['--enable-unsafe-swiftshader']});
  const page=await browser.newPage({viewport:{width:390,height:844}}),report={checks:[],errors:[]};
  page.on('pageerror',e=>report.errors.push(e.message));
  const idle=()=>page.waitForFunction(()=>!window.missileLive.active&&!window.missileSimulator.isBusy());
  const fly=async()=>{await page.evaluate(()=>window.missileLive.fly());assert(await page.evaluate(()=>window.missileLive.active),await page.locator('#missile-error').textContent());};
  const exported=()=>page.evaluate(async()=>JSON.parse(await (await fetch(document.getElementById('missile-download').href)).text()));
  try{
    await page.goto(process.env.LIVE_URL||'http://127.0.0.1:8799');await page.waitForFunction(()=>window.missileLive&&window.missileSimulator);
    await page.evaluate(()=>window.missileSimulator.load({missile:'us_aim9l_sidewinder',launcher:{position:[0,5000,0],velocity:[300,0,0],angles:[0,0,0]},target:{position:[12000,5000,0],velocity:[250,0,0],angles:[0,0,0]}}));
    await page.evaluate(async()=>{await window.missileLive.fly();window.missileLive.stop();});await idle();
    assert((await page.evaluate(()=>window.missileLive.info())).every(x=>x.open_handles===0));report.checks.push('Escape before first step cancels and frees');
    for(const event of ['blur','visibilitychange','tool-tab-change']){
      await fly();await page.keyboard.down('w');await page.waitForTimeout(100);
      await page.evaluate(event=>{
        if(event==='blur')window.dispatchEvent(new Event(event));
        else if(event==='visibilitychange'){Object.defineProperty(document,'hidden',{configurable:true,value:true});document.dispatchEvent(new Event(event));delete document.hidden;}
        else document.dispatchEvent(new CustomEvent(event,{detail:'em'}));
      },event);
      const paused=await page.evaluate(()=>window.missileLive.snapshot().time);await page.waitForTimeout(150);
      assert.equal(await page.evaluate(()=>window.missileLive.snapshot().time),paused);
      await page.keyboard.up('w');await page.keyboard.press('Space');await page.waitForTimeout(150);await page.keyboard.press('Escape');await idle();
      assert.deepEqual((await exported()).scenario.target_control.inputs.at(-1).slice(1),[0,0,0]);report.checks.push(event+' clears inputs and pauses');
    }
    await fly();const pad=page.locator('[data-code="KeyW"]'),bounds=await pad.boundingBox();
    await page.mouse.move(bounds.x+20,bounds.y+20);await page.mouse.down();await page.waitForTimeout(100);
    await page.mouse.move(10,200);await page.mouse.up();await page.waitForTimeout(100);await page.keyboard.press('Escape');await idle();
    const log=(await exported()).scenario.target_control.inputs;
    assert(log.some(row=>row[2]===1));assert.deepEqual(log.at(-1).slice(1),[0,0,0]);report.checks.push('touch pad capture releases outside button');
    await page.keyboard.press('r');await page.waitForFunction(()=>window.missileLive.active);await page.keyboard.press('Escape');await idle();report.checks.push('R restarts finished live results');
    await page.evaluate(async()=>{await window.missileLive.fly();window.missileLive.stop();});await idle();
    await page.keyboard.press('r');await page.waitForFunction(()=>window.missileLive.active);await page.keyboard.press('Escape');await idle();
    assert(await page.evaluate(()=>!document.getElementById('missile-chart').hidden||!document.getElementById('missile-setup-pane').hidden));
    report.checks.push('R remains available after pre-step cancellation restores previous live results');
    for(const fault of ['signature','hash','prepare']){
      const pattern=fault==='hash'?'**/api/missiles/inputs/*':'**/api/missiles/live/prepare';
      await page.route(pattern,async route=>{
        if(fault==='hash')return route.fulfill({status:200,body:'invalid bytes'});
        if(fault==='prepare')return route.fulfill({status:400,contentType:'application/json',body:JSON.stringify({error:'Injected preparation failure'})});
        const response=await route.fetch(),body=await response.json();body.engine_signature='invalid';await route.fulfill({response,json:body});
      });
      await page.evaluate(()=>window.missileLive.fly());await idle();
      const error=await page.locator('#missile-error').textContent();assert.match(error,fault==='signature'?/out of date/:fault==='hash'?/hash mismatch/:/preparation failure/);
      assert((await page.evaluate(()=>window.missileLive.info())).every(x=>x.open_handles===0));
      await page.unroute(pattern);report.checks.push(fault+' failure is visible and frees sessions');
    }
    await fly();await page.keyboard.press('Escape');await idle();report.checks.push('successful run after setup failures');
    // Two build labels use local fixture data; this verifies browser isolation,
    // not availability of deployed /v/<build> servers.
    await page.evaluate(()=>{
      const hooks=window.missileSimulator,prepare=hooks.prepareLive,request=GameVersions.request,handoff=hooks.showLiveFlights;
      window.buildRequests=[];
      hooks.prepareLive=async load=>{const prepared=await prepare(load),first=prepared.rows[0];
        prepared.rows=['fixture-a','fixture-b'].map(version=>({...first,version,rowKey:first.rowKey+'@'+version,prepared:{...first.prepared,input_set:first.prepared.input_set+version}}));
        prepared.submitted.missiles=prepared.rows.map(row=>row.rowKey);return prepared;};
      GameVersions.request=(url,options,version)=>{window.buildRequests.push(version);return request(url,options,'');};
      hooks.showLiveFlights=async(submitted,entries)=>{window.buildEntries=entries;};
      window.restoreBuildHooks=()=>{hooks.prepareLive=prepare;GameVersions.request=request;hooks.showLiveFlights=handoff;};
    });
    await fly();const activeBuilds=await page.evaluate(()=>window.missileLive.info());
    assert.equal(activeBuilds.filter(x=>x.build.startsWith('fixture-')&&x.open_handles===1).length,2);
    await page.waitForTimeout(100);await page.keyboard.press('Escape');await idle();
    const builds=await page.evaluate(()=>({requests:window.buildRequests,entries:window.buildEntries,info:window.missileLive.info()}));
    for(const version of ['fixture-a','fixture-b'])assert(builds.requests.includes(version));
    assert.equal(builds.entries.length,2);assert.deepEqual(builds.entries[0].document.trajectory,builds.entries[1].document.trajectory);
    assert(builds.info.every(x=>x.open_handles===0));await page.evaluate(()=>window.restoreBuildHooks());
    report.checks.push('two fixture build labels route inputs separately and isolate engine instances');
    assert.equal(await page.locator('.missile-live-canvas').count(),1);assert.deepEqual(report.errors,[]);
  }catch(error){report.failure=error.stack;throw error;}
  finally{
    await browser.close();
    const code="import sys,json;from pathlib import Path;sys.path.insert(0,'scripts');from benchmark_output import OutputBudget;b=OutputBudget(Path('outputs/live-missile'),'events');b.json(Path('outputs/live-missile/events.json'),json.load(sys.stdin));raise SystemExit(b.finish())";
    cp.execFileSync('python',['-X','utf8','-c',code],{cwd:root,input:JSON.stringify(report)});console.log(JSON.stringify(report));
  }
})().catch(error=>{console.error(error);process.exitCode=1;});
