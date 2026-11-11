const {test}=require('node:test'),assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm'),path=require('node:path');
const source=fs.readFileSync(path.join(__dirname,'../app/app.js'),'utf8');
const poll=source.slice(source.indexOf('async function poll(id){'),source.indexOf("$('scenario').addEventListener('submit'"));
async function run(job,age=3000){
  const loaded=[],timers=[],state={activeJob:'job',activeVersion:'2.56.0.47',jobStartedAt:-age,previewToken:null,meta:{aircraft:{}},jobDone:[],jobQueue:[]};
  const context={state,performance:{now:()=>0},api:async()=>job,loadData:async(...args)=>loaded.push(args),
    setRunning:value=>state.running=value,setTimeout:(f,ms)=>timers.push(ms),fmt:String,error:message=>{throw Error(message)},
    $:()=>({style:{setProperty(){}},textContent:''})};
  vm.runInNewContext(poll+';this.poll=poll;',context);await context.poll('job');return {state,loaded,timers};
}
test('exporting waits briefly for final data without downloading its stale preview',async()=>{
  const r=await run({status:'exporting',preview_revision:1});assert.deepEqual(r.loaded,[]);assert.deepEqual(r.timers,[100]);
});
test('running still displays progressive previews',async()=>{
  const r=await run({status:'running',preview_revision:1});assert.deepEqual(r.loaded,[['job',false,true]]);assert.deepEqual(r.timers,[400]);
});
test('short calculations can finish without downloading a large preview first',async()=>{
  const r=await run({status:'running',preview_revision:1},1000);assert.deepEqual(r.loaded,[]);assert.deepEqual(r.timers,[400]);
});
test('completed and stopped results load exactly once and finish',async()=>{
  for(const status of ['complete','stopped']){const r=await run({status,preview_revision:1});assert.deepEqual(r.loaded,[['job',false]]);assert.equal(r.state.activeJob,null);assert.equal(r.state.running,false);assert.deepEqual(r.timers,[]);}
});
