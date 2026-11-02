const assert=require('node:assert/strict');
const core=require('../app/missile-live-core.js');
let checks=0;
const test=(name,fn)=>{fn();checks++;};
const event=(code,extra={})=>({code,preventDefault(){this.prevented=true;},...extra});
test('codes, aliases, opposing keys, tap latch',()=>{
  const keys=new core.Keys();const e=event('KeyW');keys.down(e,true);keys.up(e);assert.equal(e.prevented,true);assert.deepEqual(keys.take(),[0,-1,0]);assert.deepEqual(keys.take(),[0,0,0]);
  keys.down(event('ArrowUp'),true);keys.down(event('KeyW'),true);keys.down(event('KeyS'),true);assert.deepEqual(keys.take(),[0,0,0]);
  keys.clear();for(const code of ['KeyS','KeyD'])keys.down(event(code),true);assert.deepEqual(keys.take(),[0,1,1]);
  keys.clear();for(const code of ['KeyQ','KeyE']){const e=event(code);assert.equal(keys.down(e,true),null);assert.equal(e.prevented,undefined);}assert.deepEqual(keys.take(),[0,0,0]);
});
test('repeats, modifiers, focus, inactive, restart',()=>{
  const keys=new core.Keys();for(const option of [{repeat:true},{ctrlKey:true},{metaKey:true},{altKey:true},{target:{closest:()=>true}}])assert.equal(keys.down(event('KeyW',option),true),null);
  assert.equal(keys.down(event('KeyW'),false),null);assert.equal(keys.down(event('KeyR'),false,true),'again');assert.equal(keys.down(event('KeyR'),true,false),null);
  keys.down(event('KeyW'),true);keys.take();keys.up(event('KeyW',{target:{closest:()=>true}}));assert.deepEqual(keys.take(),[0,0,0]);
});
test('pause clears clock, resume excludes elapsed pause',()=>{
  const clock=new core.Clock();let steps=0;clock.frame(0,()=>steps++);clock.frame(20,()=>steps++);clock.pause();clock.frame(100000,()=>steps++);clock.resume();clock.frame(200000,()=>steps++);assert.equal(steps,0);clock.frame(200002,()=>steps++);assert.equal(steps,1);
});
test('ten minutes, catchup limit, slow motion',()=>{
  for(const scale of [1,.5,.25]){const clock=new core.Clock();clock.scale=scale;let steps=0;for(let i=0;i<=36000;i++)clock.frame(i*1000/60,()=>{steps++;});assert.equal(steps,28800*scale);}
  const clock=new core.Clock();clock.frame(0,()=>{});clock.frame(100000,()=>{});assert.equal(clock.steps,4);clock.acc=10;const before=clock.steps;clock.frame(100000,()=>{});assert.equal(clock.steps-before,24);
});
test('input log is exactly the sampled controls',()=>{
  const log=[];for(const input of [[0,0,0],[1,1,-1],[1,1,-1],[0,0,0]])core.appendInput(log,...input);
  assert.deepEqual(log,[[1,0,0,0],[2,1,1,-1],[1,0,0,0]]);
});
test('quaternion sign and target interpolation',()=>{
  assert.deepEqual(core.slerp([0,0,0,1],[0,0,0,-1],.5),[0,0,0,1]);
  const rows=[{time_s:0,target:[0,0,0],target_quaternion:[0,0,0,1]},{time_s:1,target:[10,20,30],target_quaternion:[0,0,1,0]}];
  const frame=core.targetFrame(rows,.5);assert.deepEqual(frame.position,[5,10,15]);assert.ok(Math.abs(frame.quaternion[2]-Math.SQRT1_2)<1e-14);
});
test('per-build body and handoff preserve metadata',()=>{
  const body=core.prepareBody({missiles:['a@b'],launch_aircraft:{'a@b':'jet'},launcher:{position:[0,1,0]},target:{}},'a@b',()=> 'a',(state,id)=>({...state,aircraft_id:id}));
  assert.equal(body.missile,'a');assert.equal(body.launcher.aircraft_id,'jet');assert.equal(body.early_exit,false);assert.equal(body.missiles,undefined);
  const doc={scenario:{launcher:{}}};assert.equal(core.handoff({rowKey:'a@b',document:doc,aircraft:{id:'jet'}},'Label').scenario.label,'Label');assert.equal(doc.scenario.launcher.aircraft_id,'jet');
});
console.log(JSON.stringify({checks,status:'passed'}));
