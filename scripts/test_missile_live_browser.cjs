// Real-browser acceptance. External Playwright path is explicit, never installed by this test.
const assert=require('node:assert/strict'),path=require('node:path'),cp=require('node:child_process');
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'../../tools/live-browser/node_modules/playwright');
const runCount=Number(process.env.LIVE_BROWSER_RUNS||50),reportFile=process.env.LIVE_BROWSER_REPORT||'browser.json';
const root=path.resolve(__dirname,'..'),python=process.env.LIVE_PYTHON||path.resolve(root,'../WT-FM-Reconstruction/.venv/Scripts/python.exe');
function save(name,data,kind='summary'){
  const code="import sys;from pathlib import Path;sys.path.insert(0,'scripts');from benchmark_output import OutputBudget;b=OutputBudget(Path('outputs/live-missile'),'browser');b.write(Path('outputs/live-missile')/sys.argv[1],lambda f:f.write(sys.stdin.buffer.read()),kind=sys.argv[2]);raise SystemExit(b.finish())";
  cp.execFileSync(python,['-X','utf8','-c',code,name,kind],{cwd:root,input:data});
}
(async()=>{
  const browser=await chromium.launch({headless:true,args:['--enable-unsafe-swiftshader']});
  const page=await browser.newPage({viewport:{width:1440,height:1000},baseURL:process.env.LIVE_URL||'http://127.0.0.1:8799'}),errors=[],report={runs:0,replays:0,cameraSamples:0,handles:[],heap:[],errors};
  page.on('pageerror',error=>errors.push(error.message));
  try{
    await page.goto(process.env.LIVE_URL||'http://127.0.0.1:8799');await page.waitForFunction(()=>window.missileLive&&window.missileSimulator);
    assert(await page.evaluate(()=>!!document.createElement('canvas').getContext('webgl2')),'WebGL unavailable');
    assert(await page.locator('#missile-playback').isHidden());
    const meta=await (await page.request.get('/api/missiles/meta')).json();
    for(let i=0;i<runCount;i++){
      const missile=i===1?'us_aim_120c_5':'us_aim9l_sidewinder';
      const row=meta.missiles.find(m=>m.id===missile);const aircraft=row.aircraft_ids?.[0];
      await page.evaluate(({missile,aircraft})=>window.missileSimulator.load({missile,launcher:{position:[0,5000,0],velocity:[300,0,0],angles:[0,0,0],aircraft_id:aircraft},target:{position:[12000,5000,0],velocity:[50,0,0],angles:[0,0,0]}}),{missile,aircraft});
      await page.locator('#missile-live-start').click();
      await page.waitForFunction(()=>window.missileLive.active||(!window.missileSimulator.isBusy()&&!document.getElementById('missile-error').hidden),null,{timeout:120000});
      assert(await page.evaluate(()=>window.missileLive.active),await page.locator('#missile-error').textContent());
      assert(await page.locator('#missile-playback').isHidden());
      assert.equal((await page.evaluate(()=>window.missileLive.snapshot())).camera,'free');
      assert.equal(await page.locator('.missile-live-pad [data-code=KeyQ],.missile-live-pad [data-code=KeyE]').count(),0);
      if(i<2){
        await page.keyboard.press('c');
        assert.equal((await page.evaluate(()=>window.missileLive.snapshot())).camera,'target');
        let previousUp=null;
        await page.keyboard.down('s');
        for(let j=0;j<42;j++){
          await page.waitForTimeout(100);const state=await page.evaluate(()=>window.missileLive.snapshot());if(!state)break;
          assert.equal(state.camera,'target');
          assert([...state.cameraUp,...state.cameraPosition,...state.cameraDirection].every(Number.isFinite));
          assert(state.cameraUp[1]>0,'Camera horizon inverted');
          if(previousUp)assert(state.cameraUp.reduce((sum,v,k)=>sum+v*previousUp[k],0)>.9,'Camera snapped at pole');
          previousUp=state.cameraUp;report.cameraSamples++;
        }
        await page.keyboard.up('s');await page.keyboard.down('d');await page.waitForTimeout(500);await page.keyboard.up('d');
        const rolled=await page.evaluate(()=>window.missileLive.snapshot());assert(rolled.cameraUp[1]>0);
        await page.keyboard.press('c');assert.notEqual((await page.evaluate(()=>window.missileLive.snapshot())).camera,'target');
        await page.keyboard.press('c');assert.equal((await page.evaluate(()=>window.missileLive.snapshot())).camera,'free');
        await page.keyboard.press('c');
        if(i===0&&await page.evaluate(()=>window.missileLive.active))save('desktop.png',await page.screenshot(),'diagnostic');
        if(i===1&&await page.evaluate(()=>window.missileLive.active)){await page.setViewportSize({width:390,height:844});await page.waitForTimeout(100);save('phone.png',await page.screenshot(),'diagnostic');await page.setViewportSize({width:1440,height:1000});}
      }else await page.waitForTimeout(100);
      if(await page.evaluate(()=>window.missileLive.active))await page.keyboard.press('Escape');
      await page.waitForFunction(()=>!window.missileLive.active&&!window.missileSimulator.isBusy(),null,{timeout:120000});
      assert(await page.locator('#missile-playback').isVisible());
      const info=await page.evaluate(()=>window.missileLive.info());assert(info.every(x=>x.open_handles===0));report.handles.push(info.map(x=>x.open_handles));report.heap.push(info.reduce((n,x)=>n+x.heap_bytes,0));report.runs++;
      if(i<2){
        const doc=await page.evaluate(async()=>JSON.parse(await (await fetch(document.getElementById('missile-download').href)).text()));
        const scenario=structuredClone(doc.scenario);delete scenario.launcher.aircraft;
        const submitted=await (await page.request.post('/api/missiles/jobs',{data:scenario})).json();assert(submitted.id,JSON.stringify(submitted));let result;
        for(let tries=0;tries<600;tries++){const job=await (await page.request.get('/api/missiles/jobs/'+submitted.id)).json();if(job.status==='complete'){result=job.result;break;}assert.notEqual(job.status,'failed',job.error);await page.waitForTimeout(100);}
        assert(result);assert.equal(result.outcome.code,doc.outcome.code);report.replays++;
        report['replay_'+i]={outcome:doc.outcome.code,trajectory_identical:JSON.stringify(result.trajectory)===JSON.stringify(doc.trajectory)};
      }
    }
    if(runCount>=50)assert(report.heap[49]<=report.heap[4]+1_000_000,'Heap grew by more than 1 MB');
    assert.equal(await page.locator('.missile-live-canvas').count(),1);
    report.metrics=await page.evaluate(()=>window.missileLive.metrics);report.errors=errors;assert.deepEqual(errors,[]);
    save(reportFile,JSON.stringify(report));console.log(JSON.stringify({runs:report.runs,replays:report.replays,heap_delta:report.heap.at(-1)-report.heap[Math.min(4,report.heap.length-1)],errors}));
  }catch(error){report.failure=error.stack;save(reportFile,JSON.stringify(report));throw error;}
  finally{await browser.close();}
})().catch(error=>{console.error(error);process.exitCode=1;});
