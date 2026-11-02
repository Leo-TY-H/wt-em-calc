// PL-12A live preparation and replay with straight/canted aircraft nozzles.
const assert=require('node:assert/strict'),path=require('node:path'),cp=require('node:child_process');
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'../../tools/live-browser/node_modules/playwright');
const root=path.resolve(__dirname,'..'),python=process.env.LIVE_PYTHON||path.resolve(root,'../WT-FM-Reconstruction/.venv/Scripts/python.exe');
(async()=>{
  const browser=await chromium.launch({headless:true,args:['--enable-unsafe-swiftshader']});
  const page=await browser.newPage({baseURL:process.env.LIVE_URL||'http://127.0.0.1:8799'});
  const report={cases:[],error_count:0,errors:[]};
  page.on('pageerror',error=>{report.error_count++;if(report.errors.length<10)report.errors.push(error.message.slice(0,1000));});
  try{
    await page.goto('/');await page.waitForFunction(()=>window.missileLive&&window.missileSimulator);
    for(const aircraft of ['j_10c','j_15t','j_16']){
      await page.evaluate(aircraft=>window.missileSimulator.load({missile:'cn_pl12a',
        launcher:{aircraft_id:aircraft,position:[0,5000,0],velocity:[300,0,0],angles:[0,0,0]},
        target:{position:[12000,5000,0],velocity:[300,0,0],angles:[0,0,0]}}),aircraft);
      await page.locator('#missile-target-aircraft').selectOption(aircraft);
      await page.locator('#missile-live-start').click();
      await page.waitForFunction(()=>window.missileLive.active||(!window.missileSimulator.isBusy()&&!document.getElementById('missile-error').hidden),null,{timeout:120000});
      assert(await page.evaluate(()=>window.missileLive.active),await page.locator('#missile-error').textContent());
      await page.keyboard.down('s');await page.waitForTimeout(350);await page.keyboard.up('s');
      await page.keyboard.down('d');await page.waitForTimeout(350);await page.keyboard.up('d');
      await page.keyboard.press('Escape');
      await page.waitForFunction(()=>!window.missileLive.active&&!window.missileSimulator.isBusy(),null,{timeout:120000});
      const doc=await page.evaluate(async()=>JSON.parse(await (await fetch(document.getElementById('missile-download').href)).text()));
      assert.equal(doc.scenario.target_control.aircraft_profile.aircraft_id,aircraft);
      assert(doc.trajectory.every(row=>Number.isFinite(row.target_speed_mps)&&row.target_velocity.every(Number.isFinite)));
      assert(doc.scenario.target_control.inputs.some(row=>row[2]===1));
      assert(doc.scenario.target_control.inputs.some(row=>row[3]!==0));
      assert((await page.evaluate(()=>window.missileLive.info())).every(engine=>engine.open_handles===0));
      const scenario=structuredClone(doc.scenario);delete scenario.launcher.aircraft;
      const submitted=await (await page.request.post('/api/missiles/jobs',{data:scenario})).json();assert(submitted.id,JSON.stringify(submitted));
      let result;
      for(let tries=0;tries<600;tries++){
        const job=await (await page.request.get('/api/missiles/jobs/'+submitted.id)).json();
        if(job.status==='complete'){result=job.result;break;}
        assert.notEqual(job.status,'failed',job.error);await page.waitForTimeout(100);
      }
      assert(result);assert.equal(result.outcome.code,doc.outcome.code);assert.deepEqual(result.trajectory,doc.trajectory);
      report.cases.push({aircraft,missile:'cn_pl12a',source_sha256:doc.scenario.target_control.aircraft_profile.source_sha256,
        samples:doc.trajectory.length,elapsed_s:doc.elapsed_s,native_replay_identical:true,handles_freed:true});
    }
    assert.equal(report.error_count,0);
  }catch(error){report.failure=String(error.stack).slice(0,4000);throw error;}
  finally{
    await browser.close();
    const code="import sys,json;from pathlib import Path;sys.path.insert(0,'scripts');from benchmark_output import OutputBudget;b=OutputBudget(Path('outputs/live-missile'),'aircraft-nozzles');b.json(Path('outputs/live-missile/nozzle-browser.json'),json.load(sys.stdin));raise SystemExit(b.finish())";
    cp.execFileSync(python,['-X','utf8','-c',code],{cwd:root,input:JSON.stringify(report)});console.log(JSON.stringify(report));
  }
})().catch(error=>{console.error(error);process.exitCode=1;});
