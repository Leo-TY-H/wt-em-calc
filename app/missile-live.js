import * as THREE from './vendor/three.module.min.js';
import {aircraftGeometry} from './missile-setup3d.js';

const core=window.missileLiveCore,instances=new Map(),keys=new core.Keys();
const $=id=>document.getElementById(id),colors=['#38c9d7','#b79aff','#91d477','#ee8eb6','#f0d367','#e0b0ff','#cfb296','#f08070'];
let compiled=null,factory=null,run=null,view=null;
const metrics={frames:0,steps:0,totalEngineMs:0,samples:[],detail_truncated:false};
const hex=buffer=>Array.from(new Uint8Array(buffer),v=>v.toString(16).padStart(2,'0')).join('');
async function assets(){
  if(!compiled)compiled=(async()=>{
    const began=performance.now();
    const [module,response]=await Promise.all([import('./live/missile-engine.mjs'),fetch('./live/missile-engine.wasm')]);
    if(!response.ok)throw Error('Live engine is unavailable. Build the WebAssembly assets.');
    factory=module.default;const binary=await WebAssembly.compileStreaming(response);
    metrics.downloadAndCompileMs=performance.now()-began;
    metrics.downloadMs=performance.getEntriesByName(new URL('./live/missile-engine.wasm',location.href).href).at(-1)?.duration??null;
    return binary;
  })().catch(error=>{compiled=null;throw error;});
  return compiled;
}
class LiveEngine {
  constructor(module,inputSet){this.module=module;this.inputSet=inputSet;this.dead=false;this.handles=new Set();}
  call(name,type,types=[],args=[]){
    if(this.dead)throw Error('Live engine stopped. Fly again to restart.');
    try{return this.module.ccall(name,type,types,args);}
    catch(error){if(error instanceof WebAssembly.RuntimeError||this.module.aborted)this.dead=true;throw error;}
  }
  info(){return JSON.parse(this.call('engine_info','string'));}
  error(){return this.call('last_error','string');}
  state(){const offset=this.call('live_state_ptr','number')>>>3;return Array.from(this.module.HEAPF64.slice(offset,offset+core.STATE_SIZE));}
  start(config){const h=this.call('live_start','number',['string'],[JSON.stringify(config)]);if(h<0)throw Error(this.error());this.handles.add(h);return {handle:h,state:this.state()};}
  step(h,input,stop){const status=this.call('live_step','number',Array(5).fill('number'),[h,...input,Number(stop)]);if(status<0)throw Error(this.error());return {status,state:this.state()};}
  result(h){const value=this.call('live_result','string',['number'],[h]);if(!value)throw Error(this.error());return JSON.parse(value);}
  free(h){try{if(!this.dead)this.call('live_free',null,['number'],[h]);}finally{this.handles.delete(h);}}
}
async function engineFor(version,prepared){
  const first=prepared[0].prepared;
  if(prepared.some(row=>row.prepared.input_set!==first.input_set||row.prepared.engine_signature!==first.engine_signature))throw Error('Game inputs changed during preparation. Fly again.');
  let engine=instances.get(version);
  if(engine&&(engine.dead||engine.inputSet!==first.input_set)){instances.delete(version);engine=null;}
  if(!engine){
    const wasm=await assets();let aborted=false,rejectInstantiation;
    const failed=new Promise((resolve,reject)=>{rejectInstantiation=reject;});
    const module=await Promise.race([failed,factory({instantiateWasm(imports,receive){
      WebAssembly.instantiate(wasm,imports).then(instance=>receive(instance,wasm)).catch(rejectInstantiation);return {};
    },onAbort(reason){aborted=true;if(engine)engine.dead=true;rejectInstantiation(Error(String(reason)));}})]);
    module.aborted=aborted;engine=new LiveEngine(module,first.input_set);
  }
  const manifest=Object.assign({},...prepared.map(row=>row.prepared.inputs));
  for(const [relative,digest] of Object.entries(manifest)){
    if(!/^(references\/missile-inputs|scripts\/missile_model)\/[a-zA-Z0-9_./-]+$/.test(relative)||relative.split('/').includes('..'))throw Error('Invalid engine input path');
    const response=await GameVersions.request('/api/missiles/inputs/'+digest,{},version);
    if(!response.ok)throw Error('Engine input unavailable');const data=await response.arrayBuffer();
    if(hex(await crypto.subtle.digest('SHA-256',data))!==digest)throw Error('Engine input hash mismatch');
    const name='/eng/'+relative;engine.module.FS.mkdirTree(name.slice(0,name.lastIndexOf('/')));engine.module.FS.writeFile(name,new Uint8Array(data));
  }
  if(!instances.has(version)){
    if(engine.call('engine_init','number',['string'],['/eng'])<0)throw Error(engine.error());
    instances.set(version,engine);
  }
  const info=engine.info();
  if(info.native_signature!==first.engine_signature)throw Error('The page is out of date. Reload.');
  if(info.state_version!==core.STATE_VERSION||info.state_size!==core.STATE_SIZE)throw Error('The page is out of date. Reload.');
  return engine;
}
function createView(){
  const pane=document.createElement('div');pane.className='missile-live-pane';pane.tabIndex=-1;pane.hidden=true;pane.setAttribute('aria-label','Live flight');
  const renderer=new THREE.WebGLRenderer({antialias:true});renderer.setClearColor(0x000000);renderer.domElement.className='missile-live-canvas';pane.append(renderer.domElement);
  const hud=document.createElement('div');hud.className='missile-live-hud';pane.append(hud);
  const actions=document.createElement('div');actions.className='missile-live-actions';pane.append(actions);
  const button=(label,fn)=>{const b=document.createElement('button');b.type='button';b.textContent=label;b.onclick=fn;actions.append(b);return b;};
  const pauseButton=button('Pause',()=>togglePause());button('Stop',()=>stop());button('Camera',()=>cycleCamera());
  for(const [label,scale] of [['1×',1],['½×',.5],['¼×',.25]])button(label,()=>{if(run)run.clock.scale=scale;});
  const pad=document.createElement('div');pad.className='missile-live-pad';pane.append(pad);
  for(const [label,code] of [['↑','KeyW'],['↶','KeyA'],['↓','KeyS'],['↷','KeyD'],['←','KeyQ'],['→','KeyE']]){
    const b=document.createElement('button');b.type='button';b.textContent=label;b.dataset.code=code;b.setAttribute('aria-label',({KeyW:'Pull up',KeyS:'Push down',KeyA:'Roll left',KeyD:'Roll right',KeyQ:'Turn left',KeyE:'Turn right'})[code]);pad.append(b);
    b.addEventListener('pointerdown',e=>{e.preventDefault();b.setPointerCapture(e.pointerId);keys.down({code},!!run);});
    for(const name of ['pointerup','pointercancel','lostpointercapture'])b.addEventListener(name,()=>keys.up({code}));
  }
  document.querySelector('.missile-plot').append(pane);
  const scene=new THREE.Scene(),camera=new THREE.PerspectiveCamera(50,1,1,2e6);
  scene.add(new THREE.HemisphereLight(0xffffff,0x303038,2));const sun=new THREE.DirectionalLight(0xffffff,1.8);sun.position.set(.6,1,.4);scene.add(sun);
  const grid=new THREE.GridHelper(500000,100,0x555555,0x222222);scene.add(grid);
  const orbit={yaw:1,pitch:.35,distance:1000},up=new THREE.Vector3(0,1,0);let drag=null;
  renderer.domElement.addEventListener('pointerdown',e=>{if(run?.camera==='free'){drag=[e.clientX,e.clientY];renderer.domElement.setPointerCapture(e.pointerId);}});
  renderer.domElement.addEventListener('pointermove',e=>{if(drag){orbit.yaw-=(e.clientX-drag[0])*.005;orbit.pitch=Math.max(-1.5,Math.min(1.5,orbit.pitch+(e.clientY-drag[1])*.005));drag=[e.clientX,e.clientY];}});
  for(const name of ['pointerup','pointercancel','lostpointercapture'])renderer.domElement.addEventListener(name,()=>{drag=null;});
  renderer.domElement.addEventListener('wheel',e=>{e.preventDefault();orbit.distance=Math.max(50,Math.min(1e6,orbit.distance*Math.exp(e.deltaY*.001)));},{passive:false});
  return {pane,renderer,hud,scene,camera,grid,orbit,up,pauseButton,size:'',objects:[]};
}
function mesh(model,color){const object=new THREE.Mesh(aircraftGeometry(model),new THREE.MeshLambertMaterial({color,side:THREE.DoubleSide}));view.scene.add(object);view.objects.push(object);return object;}
function trail(color){
  const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.BufferAttribute(new Float32Array(14402*3),3));geometry.setDrawRange(0,0);
  const object=new THREE.Line(geometry,new THREE.LineBasicMaterial({color}));object.frustumCulled=false;view.scene.add(object);view.objects.push(object);return object;
}
function addTrail(object,position){const count=object.geometry.drawRange.count;if(count>=14402)return;object.geometry.attributes.position.setXYZ(count,...position);object.geometry.attributes.position.needsUpdate=true;object.geometry.setDrawRange(0,count+1);}
function pause(){if(!run)return;run.clock.pause();keys.clear();view.pauseButton.textContent='Resume';}
function togglePause(){if(!run)return;if(run.clock.paused){run.clock.resume();view.pauseButton.textContent='Pause';}else pause();}
function stop(){if(!run)return;if(!run.clock.steps){void finish(true);return;}run.stopRequested=true;run.clock.resume();}
function cycleCamera(){if(!run)return;const choices=['target',...run.sessions.filter(s=>!s.terminal&&!s.error).map(s=>s.rowKey),'free'];run.camera=choices[(choices.indexOf(run.camera)+1)%choices.length];}
function render(alpha,dt){
  if(!run)return;
  const states=run.sessions.map(s=>core.interpolate(s.previous,s.current,alpha));
  const targetState=states[run.sessions.findIndex(s=>!s.error&&!s.terminal)>=0?run.sessions.findIndex(s=>!s.error&&!s.terminal):run.sessions.reduce((best,s,i)=>s.current[0]>run.sessions[best].current[0]?i:best,0)];
  const target=new THREE.Vector3(...targetState.slice(13,16)),q=new THREE.Quaternion(...targetState.slice(19,23));
  run.target.position.copy(target);run.target.quaternion.copy(q);
  for(let i=0;i<states.length;i++){const s=states[i],session=run.sessions[i];session.mesh.position.set(...s.slice(1,4));session.mesh.quaternion.set(...s.slice(7,11));}
  const camera=view.camera;
  if(run.camera==='target'){
    camera.up.set(0,1,0).applyQuaternion(q);camera.position.copy(target).add(new THREE.Vector3(-180,55,0).applyQuaternion(q));camera.lookAt(target.clone().add(new THREE.Vector3(100,0,0).applyQuaternion(q)));
  }else if(run.camera==='free'){
    const o=view.orbit;camera.up.set(0,1,0);camera.position.copy(target).add(new THREE.Vector3(Math.cos(o.yaw)*Math.cos(o.pitch),Math.sin(o.pitch),Math.sin(o.yaw)*Math.cos(o.pitch)).multiplyScalar(o.distance));camera.lookAt(target);
  }else{
    const index=run.sessions.findIndex(s=>s.rowKey===run.camera),session=run.sessions[index];
    if(!session||session.terminal||session.error){run.camera=run.sessions.find(s=>!s.terminal&&!s.error)?.rowKey||'target';return render(alpha,dt);}
    const position=new THREE.Vector3(...states[index].slice(1,4)),direction=target.clone().sub(position).normalize();
    view.up.addScaledVector(direction,-view.up.dot(direction));
    if(view.up.lengthSq()<1e-12)view.up.set(0,0,1).addScaledVector(direction,-direction.z);view.up.normalize();
    if(Math.abs(direction.y)<Math.cos(Math.PI/18)){
      const desired=new THREE.Vector3(0,1,0).addScaledVector(direction,-direction.y).normalize();
      const signed=Math.atan2(direction.dot(view.up.clone().cross(desired)),view.up.dot(desired));
      view.up.applyAxisAngle(direction,Math.max(-Math.PI*dt,Math.min(Math.PI*dt,signed)));
    }
    camera.up.copy(view.up);camera.position.copy(position).addScaledVector(direction,-80);camera.lookAt(target);
  }
  for(const obj of [run.target,...run.sessions.map(s=>s.mesh)])obj.scale.setScalar(Math.max(10,camera.position.distanceTo(obj.position)*.025));
  view.grid.position.set(Math.round(target.x/5000)*5000,0,Math.round(target.z/5000)*5000);
  const width=view.pane.clientWidth,height=view.pane.clientHeight,ratio=Math.min(devicePixelRatio||1,2),size=[width,height,ratio].join(':');
  if(!width||!height)return;if(size!==view.size){view.size=size;view.renderer.setPixelRatio(ratio);view.renderer.setSize(width,height,false);camera.aspect=width/height;camera.updateProjectionMatrix();}
  view.renderer.render(view.scene,camera);
  const velocity=targetState.slice(16,19),horizontal=Math.hypot(velocity[0],velocity[2]),climb=Math.atan2(velocity[1],horizontal)*180/Math.PI;
  run.heading.textContent=`${targetState[0].toFixed(2)} s · ${Math.abs(climb)>89.5?'—':((Math.atan2(velocity[2],velocity[0])*180/Math.PI+360)%360).toFixed(0)+'°'} · ${climb.toFixed(0)}° · ${run.input[0]||run.input[1]?run.load:1} g${run.clock.paused?' · Paused':''}`;
  states.forEach((s,i)=>{const session=run.sessions[i];if(session.error){session.chip.textContent=session.label+' · '+session.error;return;}
    const delta=s.slice(13,16).map((v,k)=>v-s[1+k]),range=Math.hypot(...delta),closing=range?delta.reduce((sum,v,k)=>sum+v*(s[4+k]-s[16+k]),0)/range:0;
    session.chip.textContent=`${session.label} · ${(range/1000).toFixed(2)} km · ${closing.toFixed(0)} m/s · ${s[11].toFixed(0)} m/s · M ${s[12].toFixed(2)} · ${['ACQ','TRK','IOG','COAST'][s[23]]||'—'}${session.terminal?' · Ended':''}`;});
}
function frame(now){
  if(!run)return;const currentRun=run,dt=Math.min(.1,(now-(run.renderTime||now))/1000);run.renderTime=now;
  let engineMs=0,steps=0;
  const alpha=run.clock.frame(now,()=>{
    const input=keys.take();run.input=input;core.appendInput(run.log,...input);
    const began=performance.now();
    for(const session of run.sessions){
      if(session.terminal||session.error)continue;
      try{steps++;const next=session.engine.step(session.handle,input,run.stopRequested);session.previous=session.current;session.current=next.state;session.terminal=next.status===1;addTrail(session.trail,next.state.slice(1,4));}
      catch(error){session.error=error.message;if(session.engine.dead)for(const other of run.sessions)if(other.engine===session.engine)other.error=error.message;}
    }
    engineMs+=performance.now()-began;
    const latest=run.sessions.reduce((a,b)=>a.current[0]>=b.current[0]?a:b);addTrail(run.targetTrail,latest.current.slice(13,16));
    if(run.sessions.every(s=>s.terminal||s.error)){void finish(false);return false;}return true;
  });
  if(!currentRun.clock.paused||steps){metrics.frames++;metrics.steps+=steps;metrics.totalEngineMs+=engineMs;if(metrics.samples.length<20000)metrics.samples.push(engineMs);else metrics.detail_truncated=true;}
  if(run===currentRun){try{render(alpha,dt);run.raf=requestAnimationFrame(frame);}catch(error){for(const session of run.sessions)session.error=error.message;void finish(false);}}
}
async function finish(cancelled){
  if(!run||run.finishing)return;const ended=run;ended.finishing=true;cancelAnimationFrame(ended.raf);keys.clear();
  const entries=[],failed=[...ended.failed];
  try{
    for(const session of ended.sessions){
      if(session.error)failed.push(session.label+': '+session.error);
      else if(!cancelled)try{entries.push({rowKey:session.rowKey,document:session.engine.result(session.handle),aircraft:session.aircraft});}catch(error){failed.push(session.label+': '+error.message);}
    }
    view.pane.hidden=true;run=null;
    await window.missileSimulator.showLiveFlights(ended.submitted,entries,failed,cancelled);
  }catch(error){window.missileSimulator.liveError(error.message);
  }finally{
    for(const session of ended.sessions)try{session.engine.free(session.handle);}catch{}
    for(const object of view.objects){view.scene.remove(object);object.geometry.dispose();object.material.dispose();}view.objects=[];
    run=null;window.missileSimulator.endLive();
  }
}
async function fly(){
  const hooks=window.missileSimulator;if(!hooks||hooks.isBusy())return;
  const load=Number($('missile-live-load').value);if(!Number.isFinite(load)||load<1.5||load>15){hooks.liveError('Target load must be between 1.5 and 15 g');return;}
  let started=[];
  try{
    const prepared=await hooks.prepareLive(load);if(!prepared)return;
    if(!prepared.rows.length){await hooks.showLiveFlights(prepared.submitted,[],prepared.failed,false);hooks.endLive();return;}
    await window.missileModels.load();if(!view)view=createView();
    const groups=new Map();for(const row of prepared.rows){const key=row.version||'';if(!groups.has(key))groups.set(key,[]);groups.get(key).push(row);}
    for(const [version,rows] of groups){
      try{const engine=await engineFor(version,rows);for(const row of rows)try{const launch=engine.start(row.prepared.config);started.push({...row,engine,handle:launch.handle,previous:launch.state,current:launch.state,aircraft:row.prepared.aircraft});}catch(error){prepared.failed.push(row.label+': '+error.message);}}
      catch(error){for(const row of rows)prepared.failed.push(row.label+': '+error.message);}
    }
    if(!started.length){await hooks.showLiveFlights(prepared.submitted,[],prepared.failed,false);hooks.endLive();return;}
    started.sort((a,b)=>prepared.submitted.missiles.indexOf(a.rowKey)-prepared.submitted.missiles.indexOf(b.rowKey));
    view.hud.replaceChildren();const heading=document.createElement('span');view.hud.append(heading);
    for(let i=0;i<started.length;i++){const session=started[i];session.mesh=mesh(window.missileModels.get(GameVersions.baseMissile(session.rowKey)),colors[i]);session.trail=trail(colors[i]);addTrail(session.trail,session.current.slice(1,4));session.chip=document.createElement('span');session.chip.style.color=colors[i];view.hud.append(session.chip);}
    run={sessions:started,submitted:prepared.submitted,failed:prepared.failed,clock:new core.Clock(),log:[],input:[0,0,0],load,heading,camera:'target',target:mesh(window.missileModels.getTarget(),'#ffa66b'),targetTrail:trail('#ffa66b')};
    addTrail(run.targetTrail,started[0].current.slice(13,16));keys.clear();view.pauseButton.textContent='Pause';view.pane.hidden=false;
    $('missile-setup-pane').hidden=true;$('missile-chart').hidden=true;view.pane.focus();render(0,0);run.raf=requestAnimationFrame(frame);
  }catch(error){for(const session of started)try{session.engine.free(session.handle);}catch{}
    if(run)cancelAnimationFrame(run.raf);run=null;keys.clear();
    if(view){view.pane.hidden=true;for(const object of view.objects){view.scene.remove(object);object.geometry.dispose();object.material.dispose();}view.objects=[];}
    hooks.liveError(error.message);hooks.endLive();
  }
}
window.addEventListener('keydown',event=>{const action=keys.down(event,!!run,!run&&window.missileSimulator?.isLiveResult()&&!window.missileSimulator?.isBusy());if(action==='pause')togglePause();else if(action==='stop')stop();else if(action==='camera')cycleCamera();else if(action==='again')void fly();else if(typeof action==='number'&&run)run.clock.scale=action;});
window.addEventListener('keyup',event=>keys.up(event));window.addEventListener('blur',pause);
document.addEventListener('visibilitychange',()=>{if(document.hidden)pause();});
document.addEventListener('tool-tab-change',event=>{if(event.detail!=='missile')pause();});
$('missile-live-start').addEventListener('click',()=>void fly());
window.missileLive={fly,pause,stop,metrics,snapshot:()=>run?{time:run.sessions[0].current[0],camera:run.camera,cameraUp:view.camera.up.toArray(),targetUp:new THREE.Vector3(0,1,0).applyQuaternion(run.target.quaternion).toArray()}:null,info:()=>[...instances].map(([build,engine])=>({build,...engine.info()})),get active(){return !!run;}};
