const assert=require('node:assert/strict');
(async()=>{
  const THREE=await import('../app/vendor/three.module.min.js');
  const {TargetCamera,targetCameraFov}=await import('../app/missile-live-camera.js');
  let checks=0;
  const camera=new THREE.PerspectiveCamera(),target=new THREE.Vector3(12000,5000,0);
  const attitude=new THREE.Quaternion(),follow=new TargetCamera();
  follow.update(camera,target,attitude,0,0);
  assert(camera.position.x<target.x&&camera.position.y>target.y);
  assert(Math.abs(camera.position.distanceTo(target)-Math.hypot(17,4)*1.7)<1e-10);checks++;
  const original=camera.position.clone();
  for(let i=0;i<=360;i++){
    attitude.setFromAxisAngle(new THREE.Vector3(1,0,0),i*Math.PI/180);
    follow.update(camera,target,attitude,0,1/120);
    assert(camera.position.distanceTo(original)<1e-10,'Bank changed camera position');
    assert(camera.up.y>.999,'Camera followed aircraft roll');
  }checks++;
  for(const sign of [-1,1]){
    const loop=new TargetCamera();let previous=null;
    for(let i=0;i<=1440;i++){
      attitude.setFromAxisAngle(new THREE.Vector3(0,0,1),sign*i*Math.PI/360);
      loop.update(camera,target,attitude,sign,1/120);
      assert([...camera.position.toArray(),...camera.quaternion.toArray()].every(Number.isFinite));
      assert(camera.up.y>0);
      if(previous)assert(previous.angleTo(camera.quaternion)<.06,'Discontinuous loop camera');
      previous=camera.quaternion.clone();
    }
  }checks++;
  const poses=[];
  for(const rate of [30,60,144]){
    const f=new TargetCamera();f.update(camera,target,new THREE.Quaternion(),0,0);
    attitude.setFromAxisAngle(new THREE.Vector3(0,0,1),.3);
    for(let i=0;i<rate;i++)f.update(camera,target,attitude,1,1/rate);
    poses.push(camera.position.clone());
  }
  assert(poses.every(p=>p.distanceTo(poses[0])<1e-9),'Smoothing depends on frame rate');checks++;
  assert(Math.abs(targetCameraFov(1)-90)<1e-10);
  assert(targetCameraFov(16/9)<60&&targetCameraFov(.5)===90);checks++;
  const aim=new TargetCamera();aim.update(camera,target,new THREE.Quaternion(),0,0);
  aim.lookBy(300,-100);
  for(let i=0;i<120;i++)aim.update(camera,target,new THREE.Quaternion(),0,1/60);
  const aimed=camera.getWorldDirection(new THREE.Vector3());
  assert(aimed.y>.3&&aimed.z>.7,'Mouse did not look right and up');
  const turned=new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,1,0),-1.5);
  for(let i=0;i<120;i++)aim.update(camera,target,turned,0,1/60);
  assert(aimed.angleTo(camera.getWorldDirection(new THREE.Vector3()))<1e-6,'Aircraft turn overwrote mouse aim');
  aim.lookBy(100000,-100000);assert(Math.abs(aim.look.pitch-80*Math.PI/180)<1e-12);
  aim.lookBy(NaN,Infinity);assert(Number.isFinite(aim.look.yaw));
  aim.recenter();for(let i=0;i<240;i++)aim.update(camera,target,new THREE.Quaternion(),0,1/60);
  assert(camera.getWorldDirection(new THREE.Vector3()).x>.99,'Recenter did not restore following');checks++;
  console.log(JSON.stringify({checks,status:'passed',loopSamples:2882,rollSamples:361}));
})().catch(error=>{console.error(error);process.exitCode=1;});
