// Exercise the production playback helpers and animation loop without a GPU.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const visuals = require('../app/missile-models.js');
const missilePlot = require('../app/missile-plot.js');
const source = fs.readFileSync(path.join(__dirname, '../app/missiles.js'), 'utf8');
function fn(name) {
  const start = source.indexOf(`  function ${name}(`);
  const end = source.indexOf('\n  }', start);
  assert.ok(start >= 0 && end > start, name);
  return source.slice(start, end + 4);
}
const plain = value => JSON.parse(JSON.stringify(value));
const elements = new Map();
const el = id => {
  if (!elements.has(id)) elements.set(id, {value: '0', textContent: '', open: false});
  return elements.get(id);
};
const plotUpdates = [], cursorUpdates = [], scheduled = [];
const context = {el, console, visuals, window:{missilePlot},drawAxes(){}, Plotly: {
  restyle(node, data, indices) {plotUpdates.push(plain({data, indices,camera:node._fullLayout?.scene?.camera}));},
  relayout(node, data) {cursorUpdates.push(plain(data));}
}, requestAnimationFrame(fn) {scheduled.push(fn);}};
const helpers = source.slice(source.indexOf('  const totalTime='), source.indexOf('  function inspectFlight('));
const hover = source.slice(source.indexOf('  const trackingLabels='), source.indexOf('  // Telemetry quantities'))
  .replace('function flightHover(', 'function actualFlightHover(');
vm.runInNewContext(`'use strict';
  let flights=[],result=null,focusIndex=0,timeline=[],playbackTime=0,playing=false,lastFrame=0,hoverCalls=0,animationFrame=0,trajectoryReady=true,plotView='3d';
  const plotGestures={active:false};
  const missileName=id=>id,missileLabel=id=>id,escapeHtml=text=>String(text),baseId=id=>id;
  const viewAxes={'3d':[0,2,1],xz:[0,2],xy:[0,1],zy:[2,1]};
  const pause=()=>{playing=false;};
  ${hover}
  function flightHover(...args){hoverCalls++;return actualFlightHover(...args);}
  ${helpers}
  ${fn('projectedPositions')}
  ${fn('timeCursor')}
  ${fn('updateFrame')}
  ${fn('animate')}
  globalThis.test={playbackFrame,updateFrame,animate,
    setup(data){flights=data;result=data[0];focusIndex=0;timeline=[...new Set(data.flatMap(f=>f.trajectory.map(r=>r.time_s)))].sort((a,b)=>a-b);playbackTime=0;playing=false;lastFrame=0;hoverCalls=0;},
    frame(time){updateFrame(indexAt(timeline,time,null),time);},
    start(time=0,now=0){playbackTime=time;lastFrame=now;playing=true;},
    pause,hold(value){plotGestures.active=value;},
    state(){return {playbackTime,playing,hoverCalls};}
  };`, context);
const api = context.test;
function frozen(value) {
  if (value && typeof value === 'object') {Object.values(value).forEach(frozen);Object.freeze(value);}
  return value;
}
function flight(times = [0, .0625, .125], event = false) {
  return frozen({scenario: {missile: 'fixture', target: {position: [50, 10, 0], velocity: [20, 0, 0]}},
    elapsed_s: times.at(-1), outcome: {label: event ? 'Target proximity event' : 'Finished'},
    trajectory: times.map((time, i) => ({time_s: time, missile: [160*time, 10+32*time, 80*time],
      target: [50+20*time, 10, 0], kind: event && i === times.length-1 ? 'proximity_event' : undefined,
      tracking_state: 'observed', speed_mps: 180, mach: .6, traveled_distance_m: 180*time}))});
}
function setup(flights) {
  api.setup(flights);plotUpdates.length=0;cursorUpdates.length=0;scheduled.length=0;
  el('missile-view').value='3d';el('missile-time').value=0;el('missile-play-speed').value='1';
  el('missile-flight-data').open=false;el('missile-telemetry-chart').data=null;
  el('missile-chart')._fullLayout={scene:{camera:{eye:{x:0,y:0,z:1}}}};
}
function close(a,b) {assert.ok(Math.abs(a-b)<1e-10, `${a} != ${b}`);}
let passed=0;
function check(name, run) {run();passed++;}
check('Between samples the marker and growing trail meet at the interpolated position', () => {
  setup([flight()]);api.frame(.03125);
  const {data,indices}=plotUpdates.at(-1);
  assert.deepEqual(indices,[0,2,3]);assert.deepEqual(data.x,[[0,5],[5],[50.625]]);
  assert.deepEqual(data.y,[[0,2.5],[2.5],[0]]);assert.deepEqual(data.z,[[10,11],[11],[10]]);
  assert.equal(el('missile-time').value,0);close(api.state().playbackTime,.03125);
  assert.equal(plotUpdates.length,1);
});
check('Exact samples are not duplicated in the trail', () => {
  const f=flight();setup([f]);const frame=api.playbackFrame(f,.0625,'3d');
  assert.deepEqual(plain(frame.trail.x),[0,10]);assert.strictEqual(frame.sample,f.trajectory[1]);
});
check('1x animation produces distinct positions on 60Hz display frames', () => {
  setup([flight(Array.from({length:17},(_,i)=>i/16))]);api.start();const positions=[];
  for(let i=1;i<=60;i++){api.animate(i*1000/60);positions.push(plotUpdates.at(-1).data.x[1][0]);close(positions.at(-1),160*i/60);}
  assert.equal(new Set(positions.map(v=>v.toFixed(8))).size,60);
});
check('4x and 10x use the requested continuous playback time', () => {
  for(const speed of [4,10]){setup([flight([0,1,2])]);el('missile-play-speed').value=String(speed);api.start();api.animate(16);close(api.state().playbackTime,.016*speed);}
});
check('Pausing and resuming retain the fractional sample time', () => {
  setup([flight()]);api.start();api.animate(20);api.pause();api.animate(40);close(api.state().playbackTime,.02);
  api.start(api.state().playbackTime,100);api.animate(110);close(api.state().playbackTime,.03);
});
check('Mouse gestures keep the clock, trail and model moving on every frame', () => {
  setup([flight([0,1,2])]);api.start();api.animate(20);api.hold(true);
  api.animate(100);api.animate(200);close(api.state().playbackTime,.2);
  const count=plotUpdates.length;api.animate(300);assert.equal(plotUpdates.length,count+1);
  close(plotUpdates.at(-1).data.x[1][0],48);
  api.hold(false);api.animate(316);close(api.state().playbackTime,.316);
});
check('Each playback update preserves the current live camera before restyling', () => {
  setup([flight([0,1,2])]);const chart=el('missile-chart'),layout=chart._fullLayout.scene;
  chart.layout={scene:{camera:layout.camera}};
  let eye=1;layout._scene={fullSceneLayout:layout,getCamera:()=>({eye:{x:eye,y:2,z:3},center:{x:0,y:0,z:0},up:{x:0,y:0,z:1}})};
  api.hold(true);api.start();
  for(let i=1;i<=10;i++){
    eye=i;api.animate(i*16);
    assert.equal(plotUpdates.at(-1).camera.eye.x,i);assert.equal(chart.layout.scene.camera.eye.x,i);
    close(plotUpdates.at(-1).data.x[1][0],160*i*.016);
  }
  api.hold(false);
});
check('Backward seeking removes future points and returns to exact sample', () => {
  setup([flight()]);api.frame(.1);api.updateFrame(0);const update=plotUpdates.at(-1).data;
  assert.deepEqual(update.x[0],[0]);assert.deepEqual(update.x[1],[0]);close(api.state().playbackTime,0);
});
check('Sub-step proximity endpoint is approached without overshoot', () => {
  const f=flight([0,.0625,.091],true);setup([f]);api.frame(.08);
  close(plotUpdates.at(-1).data.x[1][0],12.8);assert.notEqual(api.playbackFrame(f,.08,'3d').sample.kind,'proximity_event');
  api.frame(1);close(plotUpdates.at(-1).data.x[1][0],14.56);close(api.state().playbackTime,.091);
  assert.equal(api.playbackFrame(f,1,'3d').sample.kind,'proximity_event');
});
check('A terminal event without a quaternion preserves the last body pose', () => {
  const f=plain(flight([0,.0625,.091],true)),q=[0,-Math.SQRT1_2,0,Math.SQRT1_2];
  f.trajectory[0].quaternion=[0,0,0,1];f.trajectory[1].quaternion=q;f.trajectory[2].quaternion=null;
  frozen(f);setup([f]);
  for(const t of [.08,.091,1])api.playbackFrame(f,t,'3d').quaternion.forEach((v,i)=>close(v,q[i]));
});
check('A finished missile stays fixed while a longer comparison continues', () => {
  setup([flight([0,.04],true),flight([0,.1,.2])]);api.frame(.15);
  const update=plotUpdates.at(-1);assert.deepEqual(update.indices,[0,1,3,4,5]);
  close(update.data.x[2][0],6.4);close(update.data.x[3][0],24);close(update.data.x[4][0],53);
});
check('Animation clamps to the final endpoint and stops scheduling', () => {
  setup([flight()]);api.start();api.animate(1000);
  close(api.state().playbackTime,.125);assert.equal(api.state().playing,false);assert.equal(scheduled.length,0);
});
check('All projections use the same fractional position', () => {
  const f=flight();setup([f]);
  for(const [view,expected] of Object.entries({xz:[5,2.5],xy:[5,11],zy:[2.5,11]})){
    el('missile-view').value=view;api.frame(.03125);const data=plotUpdates.at(-1).data;
    assert.deepEqual([data.x[1][0],data.y[1][0]],expected);assert.equal(data.z,undefined);
  }
});
check('Telemetry cursor follows continuous time, hover keeps recorded sample time', () => {
  setup([flight()]);el('missile-flight-data').open=true;el('missile-telemetry-chart').data=[];api.frame(.03125);
  close(cursorUpdates.at(-1).shapes[0].x0,.03125);
  assert.match(plotUpdates.at(-1).data.text[1][0],/0\.00 s after release/);
});
check('Immutable input and export data remain unchanged; hover is built once', () => {
  const f=flight();const before=JSON.stringify(f);setup([f]);
  for(let i=0;i<=120;i++)api.frame(i/1000);
  el('missile-view').value='xy';api.frame(.08);
  assert.equal(JSON.stringify(f),before);assert.equal(api.state().hoverCalls,f.trajectory.length);
});
check('Single-sample and duplicate-time trajectories remain finite', () => {
  const one=flight([0]);setup([one]);assert.deepEqual(plain(api.playbackFrame(one,.5,'3d').position),[0,10,0]);
  const duplicate=flight([0,0,.0625]);setup([duplicate]);api.frame(.02);close(plotUpdates.at(-1).data.x[1][0],3.2);
});
console.log(JSON.stringify({status:'passed',checks:passed,display_frames_per_second_tested:60,source:'app/missiles.js',browser_rendering_tested:false}));
