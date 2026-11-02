const path=require('node:path'),cp=require('node:child_process'),assert=require('node:assert/strict');
const {chromium,firefox}=require(process.env.PLAYWRIGHT_MODULE||'../../tools/live-browser/node_modules/playwright');
const root=path.resolve(__dirname,'..'),python=path.resolve(root,'../WT-FM-Reconstruction/.venv/Scripts/python.exe');
const summary=[];
const describe=values=>{const sorted=values.slice().sort((a,b)=>a-b);return {median_ms:sorted[Math.floor(sorted.length/2)],p99_ms:sorted[Math.min(sorted.length-1,Math.floor(sorted.length*.99))]};};
async function chrome(throttle){
  const browser=await chromium.launch({headless:true,args:['--enable-unsafe-swiftshader']});
  try{
    const page=await browser.newPage({viewport:{width:1440,height:1000},baseURL:'http://127.0.0.1:8799'});
    await page.goto('/');await page.waitForFunction(()=>window.missileLive&&window.missileSimulator);
    const meta=await (await page.request.get('/api/missiles/meta')).json();
    const missile='us_aim_120c_5',aircraft=meta.missiles.find(m=>m.id===missile).aircraft_ids[0];
    await page.evaluate(({missile,aircraft})=>window.missileSimulator.load({missile,launcher:{position:[0,5000,0],velocity:[300,0,0],angles:[0,0,0],aircraft_id:aircraft},target:{position:[12000,5000,0],velocity:[2000,0,0],angles:[0,0,0]}}),{missile,aircraft});
    await page.waitForFunction(()=>document.querySelector('[data-add-missile="us_aim_120c_5"]'));
    await page.evaluate(()=>{for(let i=0;i<7;i++)document.querySelector('[data-add-missile="us_aim_120c_5"]').click();});
    const session=await page.context().newCDPSession(page);await session.send('Emulation.setCPUThrottlingRate',{rate:throttle});
    await page.locator('#missile-live-start').click();await page.waitForFunction(()=>window.missileLive.active,null,{timeout:120000});
    await page.waitForFunction(()=>window.missileLive.metrics.frames>=1000,null,{timeout:120000});
    assert.equal(await page.locator('.missile-live-hud span').count(),9);
    const m=await page.evaluate(()=>window.missileLive.metrics);await page.keyboard.press('Escape');
    await page.waitForFunction(()=>!window.missileLive.active&&!window.missileSimulator.isBusy(),null,{timeout:120000});
    assert.equal((await page.evaluate(()=>window.missileLive.info()))[0].open_handles,0);
    const row={browser:'Chromium',cpu_throttle:throttle,sessions:8,frames:m.frames,steps:m.steps,...describe(m.samples),per_step_ms:m.totalEngineMs/m.steps,download_ms:m.downloadMs,download_and_compile_ms:m.downloadAndCompileMs};summary.push(row);console.log(JSON.stringify(row));
  }finally{await browser.close();}
}
async function firefoxLoop(){
  const browser=await firefox.launch({headless:true});
  try{
    const page=await browser.newPage({baseURL:'http://127.0.0.1:8799'});await page.goto('/');
    const row=await page.evaluate(async()=>{
      const factory=(await import('/live/missile-engine.mjs')).default,module=await factory();
      const meta=await (await fetch('/api/missiles/meta')).json();
      const results=[];let initialized=false;
      const call=(name,type,types=[],args=[])=>module.ccall(name,type,types,args);
      for(const missile of ['us_aim9l_sidewinder','us_aim_120c_5','us_aim9b_sidewinder','us_aim9m_sidewinder','us_aim7f_sparrow','us_aim7m_sparrow','su_r_73','fr_mica_em']){
        const aircraft=meta.missiles.find(m=>m.id===missile).aircraft_ids?.[0];
        const request={missile,launcher:{position:[0,5000,0],velocity:[300,0,0],angles:[0,0,0],aircraft_id:aircraft},target:{position:[12000,5000,0],velocity:[2000,0,0],angles:[0,0,0]},target_control:{load_g:9}};
        const prepared=await (await fetch('/api/missiles/live/prepare',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(request)})).json();
        if(prepared.error)throw Error(prepared.error);
        for(const [relative,hash] of Object.entries(prepared.inputs)){const name='/eng/'+relative;module.FS.mkdirTree(name.slice(0,name.lastIndexOf('/')));module.FS.writeFile(name,new Uint8Array(await (await fetch('/api/missiles/inputs/'+hash)).arrayBuffer()));}
        if(!initialized){if(call('engine_init','number',['string'],['/eng'])<0)throw Error(call('last_error','string'));initialized=true;}
        let time=0,restarts=0,handle=null,steps=0;
        while(steps<14400){
          if(handle===null){handle=call('live_start','number',['string'],[JSON.stringify(prepared.config)]);if(handle<0)throw Error(call('last_error','string'));restarts++;}
          const begin=performance.now();let status=0;
          for(let j=0;j<24&&steps<14400;j++){status=call('live_step','number',Array(5).fill('number'),[handle,0,0,0,0]);steps++;if(status!==0)break;}
          time+=performance.now()-begin;
          if(status<0)throw Error(call('last_error','string'));if(status===1){call('live_free',null,['number'],[handle]);handle=null;}
        }
        if(handle!==null)call('live_free',null,['number'],[handle]);
        results.push({browser:'Firefox',missile,steps,engine_ms:time,per_step_ms:time/steps,restarts,info:JSON.parse(call('engine_info','string'))});
      }
      return results;
    });summary.push(...row);console.log(JSON.stringify(row));
  }finally{await browser.close();}
}
(async()=>{
  try{await chrome(1);await chrome(4);await firefoxLoop();}
  finally{
    const code="import sys;from pathlib import Path;sys.path.insert(0,'scripts');from benchmark_output import OutputBudget;b=OutputBudget(Path('outputs/live-missile'),'browser-performance');b.write(Path('outputs/live-missile/performance.json'),lambda f:f.write(sys.stdin.buffer.read()),kind='summary');raise SystemExit(b.finish())";
    cp.execFileSync(python,['-X','utf8','-c',code],{cwd:root,input:JSON.stringify(summary)});
  }
})().catch(error=>{console.error(error);process.exitCode=1;});
