// Real asset integrity and display-only pose regression checks; no GPU required.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const models = require('../app/missile-models.js');
const close = (a,b) => assert.ok(Math.abs(a-b)<1e-6, `${a} != ${b}`);
const vector = (a,b) => a.forEach((v,i)=>close(v,b[i]));
const identity = [0,0,0,1], yaw90 = [0,-Math.SQRT1_2,0,Math.SQRT1_2];
let checks=0;
function check(name,fn) {fn();checks++;}

check('Nose, up and right preserve the game axes before Plotly projection',()=>{
  vector(models.rotate([1,0,0],identity),[1,0,0]);
  vector(models.rotate([1,0,0],yaw90),[0,0,1]);
  vector(models.rotate([0,1,0],[Math.SQRT1_2,0,0,Math.SQRT1_2]),[0,0,1]);
});
check('SLERP follows the short arc and is invariant to quaternion sign',()=>{
  const a={quaternion:identity}, b={quaternion:yaw90};
  const middle=models.attitude(a,b,.5);
  vector(models.rotate([1,0,0],middle),[Math.SQRT1_2,0,Math.SQRT1_2]);
  vector(models.attitude(a,{quaternion:yaw90.map(v=>-v)},.5),middle);
  vector(models.attitude(a,{quaternion:identity.map(v=>-v)},.5),identity);
});
check('Body orientation wins over velocity, including angle of attack',()=>{
  vector(models.attitude({quaternion:yaw90,velocity:[100,0,0]},null,0),yaw90);
  vector(models.attitude({quaternion:yaw90.map(v=>2*v)},null,0),yaw90);
});
check('An exact sub-step hit holds the last physical body attitude',()=>{
  const prior={quaternion:yaw90}, event={kind:'proximity_event',quaternion:null,velocity:null};
  vector(models.attitude(prior,event,.7),yaw90);
  vector(models.attitude(event,null,0,prior),yaw90);
});
check('Older exports without attitude use velocity or the next position',()=>{
  for(const direction of [[1,0,0],[0,0,1],[0,1,0],[-2,3,-4]]) {
    const q=models.attitude({velocity:direction},null,0);
    vector(models.rotate([1,0,0],q),direction.map(v=>v/Math.hypot(...direction)));
  }
  vector(models.rotate([1,0,0],models.attitude({missile:[1,2,3]},{missile:[1,2,4]},.2)),[0,0,1]);
});
const catalog=JSON.parse(fs.readFileSync(path.join(__dirname,'../app/missile-models.json')));
const targetAsset=JSON.parse(fs.readFileSync(path.join(__dirname,'../app/target-model.json')));
const launchers=JSON.parse(fs.readFileSync(path.join(__dirname,'../references/missile-launchers.json')));
check('Every installed mesh has finite, normalized geometry and valid faces',()=>{
  assert.equal(catalog.schema,1);
  for(const mesh of Object.values(catalog.models)) {
    assert.ok(mesh.length_m>.1 && mesh.length_m<25);
    const xs=mesh.vertices.map(v=>v[0]);close(Math.max(...xs)-Math.min(...xs),1);
    for(const v of mesh.vertices)assert.ok(v.length===3 && v.every(Number.isFinite));
    for(const t of mesh.triangles)assert.ok(t.length===3 && new Set(t).size===3 && t.every(i=>Number.isInteger(i)&&i>=0&&i<mesh.vertices.length));
  }
  for(const [id,name] of Object.entries(catalog.missiles)) {
    assert.ok(catalog.models[name],id);assert.ok(launchers.carriers[id]?.length,id);
  }
  for(const id of ['cn_pl12a','us_aim_120d','us_aim9l_sidewinder'])assert.ok(catalog.missiles[id]);
});
check('3D geometry is centered, rotated and projected without mutating assets',()=>{
  const mesh={vertices:[[1,0,0],[0,1,0],[0,0,1]]};
  const before=JSON.stringify(mesh);
  const shape=models.geometry(mesh,[10,20,30],yaw90,2,'3d');
  vector(shape.x,[10,10,8]);vector(shape.y,[32,30,30]);vector(shape.z,[20,22,20]);
  assert.equal(JSON.stringify(mesh),before);
});
check('All 2D footprints are the projected model triangles, each closed',()=>{
  const mesh=catalog.models[catalog.missiles.cn_pl12a];
  for(const view of ['xz','xy','zy']) {
    const shape=models.geometry(mesh,[100,200,300],yaw90,20,view);
    assert.equal(shape.x.length,mesh.triangles.length*5);
    for(let i=0;i<shape.x.length;i+=5) {
      close(shape.x[i],shape.x[i+3]);close(shape.y[i],shape.y[i+3]);
      assert.equal(shape.x[i+4],null);assert.equal(shape.y[i+4],null);
      assert.ok([0,1,2,3].every(k=>Number.isFinite(shape.x[i+k])&&Number.isFinite(shape.y[i+k])));
    }
  }
});
check('Zoom scaling bounds the screen marker even below physical size',()=>{
  const mesh={length_m:4};
  const chart={clientHeight:500,_fullLayout:{xaxis:{range:[0,1000],_length:1000},yaxis:{range:[0,500],_length:500}}};
  close(models.displayLength(mesh,chart,'xz',[[0,1000],[0,500],[0,100]]),40);
  chart._fullLayout.xaxis.range=[0,1];chart._fullLayout.yaxis.range=[0,1];
  close(models.displayLength(mesh,chart,'xz',[[0,1000],[0,500],[0,100]]),.08);
});
check('Mesh trace reverses winding for the plot Y/Z permutation',()=>{
  const trace=models.trace({triangles:[[0,1,2]]},'Missile','#123456','3d');
  assert.equal(trace.type,'mesh3d');assert.equal(trace.color,'#123456');
  assert.deepEqual([trace.i,trace.j,trace.k],[[0],[2],[1]]);
});
check('Panning in 3D does not change the marker scale',()=>{
  const scene={aspectratio:{x:1,y:1,z:1},xaxis:{range:[0,1000]},yaxis:{range:[0,1000]},zaxis:{range:[0,1000]},camera:{eye:{x:0,y:-1.6,z:.65},center:{x:0,y:0,z:0}}};
  const chart={clientHeight:500,_fullLayout:{scene}},bounds=[[0,1000],[0,1000],[0,1000]],mesh={length_m:15};
  const before=models.displayLength(mesh,chart,'3d',bounds);
  for(const key of ['x','y','z']){scene.camera.eye[key]+=100;scene.camera.center[key]+=100;}
  close(models.displayLength(mesh,chart,'3d',bounds),before);
});
check('F-16 ADF target geometry is valid and does not attract hover',()=>{
  assert.equal(targetAsset.id,'f_16a_block_15_adf');
  const m=targetAsset.model;assert.ok(m.length_m>14 && m.length_m<17);
  assert.equal(m.triangles.length,711);
  for(const v of m.vertices)assert.ok(v.length===3 && v.every(Number.isFinite));
  for(const face of m.triangles)assert.ok(face.every(i=>Number.isInteger(i)&&i>=0&&i<m.vertices.length));
  for(const view of ['3d','xz','xy','zy']){
    const trace=models.trace(m,'Target now','#ffd0ac',view,false);
    assert.equal(trace.hoverinfo,'skip');assert.equal(trace.hovertemplate,undefined);
    const g=models.geometry(m,[0,0,0],identity,20,view);
    assert.ok(g.x.concat(g.y,g.z||[]).every(v=>v===null||Number.isFinite(v)));
    if(view!=='3d'){
      assert.equal(g.x.length,m.triangles.length*5);assert.equal(trace.line.width,0);
      for(let i=0;i<g.x.length;i+=5){
        assert.equal(g.x[i+4],null);assert.equal(g.y[i+4],null);
        assert.ok((g.x[i+1]-g.x[i])*(g.y[i+2]-g.y[i])-(g.y[i+1]-g.y[i])*(g.x[i+2]-g.x[i])>=-1e-9);
      }
    }
  }
});
check('Target nose follows heading/climb, with stationary heading as fallback',()=>{
  vector(models.rotate([1,0,0],models.targetAttitude({velocity:[-200,0,0],angles:[0,0,0]})),[-1,0,0]);
  vector(models.rotate([1,0,0],models.targetAttitude({velocity:[100,100,0]})),[Math.SQRT1_2,Math.SQRT1_2,0]);
  vector(models.rotate([1,0,0],models.targetAttitude({velocity:[0,0,0],angles:[90,30,0]})),[0,.5,Math.sqrt(3)/2]);
});

async function main() {
  const source=fs.readFileSync(path.join(__dirname,'../app/missile-models.js'),'utf8');
  let requests=0;const context={window:{},console,fetch:async url=>{requests++;return {ok:true,json:async()=>url.includes('target-model')?targetAsset:catalog};}};
  vm.runInNewContext(source,context);
  await Promise.all([context.window.missileModels.load(),context.window.missileModels.load()]);
  assert.equal(requests,2);assert.equal(context.window.missileModels.get('us_aim_120d'),catalog.models[catalog.missiles.us_aim_120d]);
  assert.equal(context.window.missileModels.getTarget(),targetAsset.model);
  assert.equal(context.window.missileModels.get('unknown'),null);checks++;
  const warnings=[];const missing={window:{},console:{warn:text=>warnings.push(text)},fetch:async()=>({ok:false})};
  vm.runInNewContext(source,missing);await missing.window.missileModels.load();
  assert.equal(missing.window.missileModels.get('us_aim_120d'),null);assert.equal(missing.window.missileModels.getTarget(),null);assert.equal(warnings.length,2);checks++;
  const partial={window:{},console:{warn(){}},fetch:async url=>({ok:!url.includes('target-model'),json:async()=>catalog})};
  vm.runInNewContext(source,partial);await partial.window.missileModels.load();
  assert.ok(partial.window.missileModels.get('us_aim_120d'));assert.equal(partial.window.missileModels.getTarget(),null);checks++;
  console.log(JSON.stringify({status:'passed',checks,missiles:Object.keys(catalog.missiles).length,meshes:Object.keys(catalog.models).length}));
}
main().catch(error=>{console.error(error);process.exitCode=1;});
