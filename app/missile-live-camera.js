import * as THREE from './vendor/three.module.min.js';

// Aircraft TPS reference: War-Thunder-Datamine, 546f90e, config/camera.blkx.
// This is a presentation approximation of the mouse-aim view, not Gaijin's
// unavailable controller. The keyboard still controls the target's attitude.
const clamp=THREE.MathUtils.clamp,limit=80*Math.PI/180;
const wrap=angle=>Math.atan2(Math.sin(angle),Math.cos(angle));
export class TargetCamera {
  constructor(){this.ready=false;this.yaw=0;this.pitch=0;this.bank=0;this.distance=17;}
  update(camera,target,attitude,pitchInput,dt){
    const forward=new THREE.Vector3(1,0,0).applyQuaternion(attitude);
    const horizontal=Math.hypot(forward.x,forward.z);
    // Keep heading through the pole; don't turn numerical noise into a 180° snap.
    const yaw=horizontal>Math.cos(limit)||!this.ready?Math.atan2(forward.z,forward.x):this.yaw;
    const pitch=clamp(Math.atan2(forward.y,horizontal),-limit,limit);
    const distance=pitchInput?14:17;
    if(!this.ready){this.yaw=yaw;this.pitch=pitch;this.distance=distance;this.ready=true;}
    const seconds=clamp(dt,0,.1),blend=-Math.expm1(-seconds/.1);
    this.yaw+=clamp(wrap(yaw-this.yaw)*blend,-Math.PI*seconds,Math.PI*seconds);
    this.pitch+=(pitch-this.pitch)*blend;
    this.distance+=(distance-this.distance)*-Math.expm1(-seconds/.04);
    const direction=new THREE.Vector3(Math.cos(this.yaw)*Math.cos(this.pitch),Math.sin(this.pitch),Math.sin(this.yaw)*Math.cos(this.pitch));
    const right=new THREE.Vector3().crossVectors(direction,new THREE.Vector3(0,1,0)).normalize();
    const up=new THREE.Vector3().crossVectors(right,direction).normalize();
    const bodyUp=new THREE.Vector3(0,1,0).applyQuaternion(attitude);
    const bank=.01*Math.atan2(bodyUp.dot(right),bodyUp.dot(up));
    this.bank+=(bank-this.bank)*blend;
    // Neutral (-17,4,0), 500 m look-ahead and 1.7 distance multiplier.
    // Use horizon-relative height, so rolling never swings the camera around the plane.
    camera.position.copy(target).addScaledVector(direction,-this.distance*1.7).addScaledVector(up,4*1.7);
    camera.up.copy(up).applyAxisAngle(direction,this.bank);
    camera.lookAt(target.clone().addScaledVector(direction,500));
  }
}

// Treat the reference's 90° FOV as horizontal on landscape displays; cap vertical
// FOV on narrow screens so the aircraft remains readable in portrait orientation.
export const targetCameraFov=aspect=>2*Math.atan(1/Math.max(1,aspect))*180/Math.PI;
