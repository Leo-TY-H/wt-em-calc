const assert=require('node:assert/strict');
const {bindGestures,latestRenderer,stabilizeCamera,commitCamera}=require('../app/missile-plot.js');
class Events {
  listeners=new Map();
  addEventListener(name,fn){if(!this.listeners.has(name))this.listeners.set(name,new Set());this.listeners.get(name).add(fn);}
  removeEventListener(name,fn){this.listeners.get(name)?.delete(fn);}
  emit(name,event={}){for(const fn of this.listeners.get(name)||[])fn({target:{closest:()=>null},button:0,...event});}
}
const host=new Events(),node=new Events(),frames=new Map(),timers=new Map();let seq=0,ended=0,checks=0;
host.requestAnimationFrame=fn=>{frames.set(++seq,fn);return seq;};host.cancelAnimationFrame=id=>frames.delete(id);
host.setTimeout=fn=>{timers.set(++seq,fn);return seq;};host.clearTimeout=id=>timers.delete(id);
const flush=queue=>{const pending=[...queue.values()];queue.clear();pending.forEach(fn=>fn());};
const guard=bindGestures(node,()=>ended++,host);
function check(name,fn){fn();checks++;}
check('Release outside the plot settles once, after pointer and mouse up',()=>{
  node.emit('pointerdown');node.emit('mousedown');assert.ok(guard.active);
  host.emit('pointerup');host.emit('mouseup');assert.ok(guard.active);assert.equal(frames.size,1);
  flush(frames);assert.ok(!guard.active);assert.equal(ended,1);
});
check('Wheel bursts settle after the final event and do not interrupt a drag',()=>{
  node.emit('wheel');node.emit('wheel');assert.equal(timers.size,1);assert.ok(guard.active);
  node.emit('mousedown');flush(timers);flush(frames);assert.ok(guard.active);
  host.emit('mouseup');flush(frames);assert.ok(!guard.active);assert.equal(ended,2);
});
check('Cancel, blur, and pointercancel cannot leave the plot held',()=>{
  node.emit('mousedown');node.emit('wheel');guard.cancel();flush(timers);flush(frames);assert.ok(!guard.active);
  node.emit('pointerdown');host.emit('pointercancel');flush(frames);assert.ok(!guard.active);
  node.emit('mousedown');host.emit('blur');assert.ok(!guard.active);
});
check('Modebar clicks do not start a canvas gesture',()=>{
  node.emit('mousedown',{target:{closest:()=>({})}});node.emit('wheel',{target:{closest:()=>({})}});assert.ok(!guard.active);
});
check('Disposal removes event handlers and pending work',()=>{
  node.emit('wheel');guard.dispose();assert.equal(timers.size,0);assert.equal(frames.size,0);
  node.emit('mousedown');assert.ok(!guard.active);
});
check('Turntable restyles do not invoke the delayed camera mode setter',()=>{
  let delayed=0,original=0,current='orbit';
  const camera={view:{setMode(value){current=value;}},set mode(value){delayed++;current=value;},get mode(){return current;}};
  const scene={camera,id:'scene',fullSceneLayout:{camera:{up:{x:1,y:0,z:0}}},graphDiv:{layout:{scene:{camera:{up:{x:1,y:0,z:0}}}}},updateFx(mode){original++;camera.mode=mode;}};
  const node={_fullLayout:{scene:{_scene:scene}}};stabilizeCamera(node);const wrapped=scene.updateFx;stabilizeCamera(node);assert.equal(scene.updateFx,wrapped);
  for(let i=0;i<100;i++)scene.updateFx('turntable','closest');
  assert.equal(delayed,0);assert.equal(current,'turntable');assert.equal(camera.keyBindingMode,'rotate');
  assert.deepEqual(scene.fullSceneLayout.camera.up,{x:0,y:0,z:1});
  scene.updateFx('orbit','closest');assert.equal(original,1);assert.equal(delayed,1);
});
check('Release outside the canvas commits the live camera for the next replot',()=>{
  const camera={eye:{x:2,y:3,z:4},up:{x:0,y:0,z:1},center:{x:1,y:1,z:1}};
  const scene={getCamera:()=>camera,fullSceneLayout:{camera:null}},node={layout:{scene:{}},_fullLayout:{scene:{_scene:scene}}};
  commitCamera(node);assert.deepEqual(node.layout.scene.camera,camera);assert.deepEqual(scene.fullSceneLayout.camera,camera);
});
async function main(){
  const calls=[],releases=[];let simultaneous=0,maximum=0;
  const queue=latestRenderer(async(view,isCurrent)=>{
    calls.push(view);maximum=Math.max(maximum,++simultaneous);
    await new Promise(resolve=>releases.push(resolve));
    simultaneous--;calls.push(isCurrent()?'published '+view:'discarded '+view);
  });
  const a=queue.request('3d');await Promise.resolve();await Promise.resolve();
  const b=queue.request('xy'),c=queue.request('xz');releases.shift()();assert.equal(await a,false);
  assert.equal(await b,false);await Promise.resolve();releases.shift()();assert.equal(await c,true);
  assert.equal(maximum,1);assert.deepEqual(calls,['3d','discarded 3d','xz','published xz']);checks++;
  const cancelled=queue.request('zy');queue.cancel();assert.equal(await cancelled,false);checks++;
  let failed=true;const recovering=latestRenderer(async()=>{if(failed){failed=false;throw new Error('fixture');}});
  await assert.rejects(recovering.request('bad'),/fixture/);assert.equal(await recovering.request('good'),true);checks++;
  console.log(JSON.stringify({status:'passed',checks}));
}
main().catch(error=>{console.error(error);process.exitCode=1;});
