// The trajectory simulator's setup view (Leo, 2026-10-30): a three.js scene where the engagement is set by dragging.
// Plotly's 3D charts redraw the whole figure for every change, so this view has its own light scene: a ground grid at
// altitude 0, axes, both aircraft as the game's F-16A mesh at a constant size on screen, heading arrows, and a camera
// that turns, pans and zooms with a little inertia. Nothing limits a drag but the fields' own ranges. World metres map
// straight to three's axes: X right, Y = altitude up, Z toward the viewer (right-handed; headings run from +X to +Z).
import * as THREE from './vendor/three.module.min.js';

const FOV=36,MIN_DISTANCE=30,MAX_DISTANCE=3e6,MIN_ELEVATION=.03,MAX_ELEVATION=1.55,DAMPING=16;
const ARROW_PX=62,ALTITUDE_PX=30,TILT_PX=21,TILT_RATE=.9,GRID_CELLS=32;
const niceStep=value=>{const power=10**Math.floor(Math.log10(value)),m=value/power;return (m<1.5?1:m<3.5?2:m<7.5?5:10)*power;};
const degrees=value=>`${+value.toFixed(1)}°`;

window.missileSetup3d={create};

function aircraftGeometry(model) {
  if(!model) {
    // Without the game's mesh, a dart along +X of unit length.
    const cone=new THREE.ConeGeometry(.16,1,12);cone.rotateZ(-Math.PI/2);return cone.toNonIndexed();
  }
  const positions=[];
  for(const face of model.triangles)for(const index of face)positions.push(...model.vertices[index]);
  const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.Float32BufferAttribute(positions,3));
  geometry.computeVertexNormals();return geometry;
}

function create(container,{labels,colors,onCamera,onDrag}) {
  let renderer;
  try{renderer=new THREE.WebGLRenderer({antialias:true});}catch(error){console.warn(error);return null;}
  renderer.setPixelRatio(Math.min(window.devicePixelRatio||1,2));renderer.setClearColor(0x000000);
  const canvas=renderer.domElement;canvas.style.cssText='display:block;width:100%;height:100%;touch-action:none';container.append(canvas);
  const scene=new THREE.Scene(),camera=new THREE.PerspectiveCamera(FOV,1,10,1e7);
  scene.add(new THREE.HemisphereLight(0xffffff,0x303038,1.9));
  const sun=new THREE.DirectionalLight(0xffffff,1.7);sun.position.set(.6,1,.4);scene.add(sun);

  // Camera: the shown pose follows its goal, which gives every gesture the same soft finish.
  const view={target:new THREE.Vector3(2000,2500,0),azimuth:1.2,elevation:.42,distance:12000};
  const goal={target:view.target.clone(),azimuth:view.azimuth,elevation:view.elevation,distance:view.distance};
  const size={width:0,height:0};
  let state=null,framed=false,hover=null,drag=null,orbit=null,scheduled=false,lastFrame=0;
  const pointers=new Map();

  const line=(color,dashed=false)=>{
    const material=dashed?new THREE.LineDashedMaterial({color,dashSize:1,gapSize:1}):new THREE.LineBasicMaterial({color});
    const object=new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(),new THREE.Vector3()]),material);
    object.frustumCulled=false;scene.add(object);return object;
  };
  const setLine=(object,a,b)=>{const p=object.geometry.attributes.position;p.setXYZ(0,a.x,a.y,a.z);p.setXYZ(1,b.x,b.y,b.z);p.needsUpdate=true;object.computeLineDistances();};
  const solid=(geometry,color,opacity=1)=>{
    const mesh=new THREE.Mesh(geometry,new THREE.MeshBasicMaterial({color,transparent:opacity<1,opacity}));mesh.frustumCulled=false;scene.add(mesh);return mesh;
  };

  // Ground grid and axes: faded toward their ends, rebuilt only when the view's scale or centre moves a cell.
  const grid=new THREE.LineSegments(new THREE.BufferGeometry(),new THREE.LineBasicMaterial({vertexColors:true}));grid.frustumCulled=false;scene.add(grid);
  let gridKey='',gridStep=1000;
  function buildGrid() {
    const step=niceStep(view.distance/7),major=step*5,cx=Math.round(view.target.x/major)*major,cz=Math.round(view.target.z/major)*major;
    gridStep=step;const key=`${step}:${cx}:${cz}`;if(key===gridKey)return;gridKey=key;
    const positions=[],shades=[],reach=GRID_CELLS*step;
    const add=(a,b,c,shade)=>{positions.push(...a,...b,...b,...c);shades.push(0,0,0,shade,shade,shade,shade,shade,shade,0,0,0);};
    for(let i=-GRID_CELLS;i<=GRID_CELLS;i++) {
      const fade=(1-Math.abs(i)/GRID_CELLS)**1.5,x=cx+i*step,z=cz+i*step;
      const strength=value=>(Math.abs(value)<step/2?.55:Math.abs(value/major-Math.round(value/major))<1e-6?.2:.085)*fade;
      add([x,0,cz-reach],[x,0,cz],[x,0,cz+reach],strength(x));
      add([cx-reach,0,z],[cx,0,z],[cx+reach,0,z],strength(z));
    }
    grid.geometry.setAttribute('position',new THREE.Float32BufferAttribute(positions,3));
    grid.geometry.setAttribute('color',new THREE.Float32BufferAttribute(shades,3));
  }
  const altitudeAxis=line(0x77777f);

  const unit=new THREE.CylinderGeometry(1,1,1,10),cone=new THREE.ConeGeometry(1,1,16),ball=new THREE.SphereGeometry(1,20,14);
  const foot=new THREE.RingGeometry(.72,1,28);foot.rotateX(-Math.PI/2);
  const lineOfSight=line(0xc8c8d0,true),groundLine=line(0x66666e,true);
  const material=color=>new THREE.MeshLambertMaterial({color,side:THREE.DoubleSide});
  const aircraft={};
  for(const role of ['launch','target']) {
    const color=colors[role];
    const mesh=new THREE.Mesh(aircraftGeometry(null),material(color));mesh.frustumCulled=false;mesh.matrixAutoUpdate=false;scene.add(mesh);
    aircraft[role]={color,mesh,position:new THREE.Vector3(),forward:new THREE.Vector3(1,0,0),moving:false,
      shaft:solid(unit,color),head:solid(cone,color),tip:solid(ball,color,.5),
      up:solid(cone,color),down:solid(cone,color),tiltUp:solid(cone,color),tiltDown:solid(cone,color),tiltPoint:new THREE.Vector3(),drop:line(0x77777f,true),foot:solid(foot,color,.7),tipPoint:new THREE.Vector3(),gizmoPoint:new THREE.Vector3()};
  }

  // Screen labels are plain text over the canvas, kept crisp at any zoom.
  const pool=[];let poolUsed=0;
  const label=(text,x,y,className,color)=>{
    let node=pool[poolUsed];
    if(!node){node=document.createElement('div');labels.append(node);pool.push(node);}
    poolUsed++;
    if(node.dataset.text!==text){node.innerHTML=text;node.dataset.text=text;}
    if(node.className!==className)node.className=className;
    node.style.color=color||'';node.hidden=false;node.style.transform=`translate(${x.toFixed(1)}px,${y.toFixed(1)}px) translate(-50%,0)`;
  };
  const projected=new THREE.Vector3();
  function toScreen(point) {
    projected.copy(point).project(camera);
    return {x:(projected.x+1)/2*size.width,y:(1-projected.y)/2*size.height,visible:projected.z>-1&&projected.z<1};
  }
  const pixel=point=>2*camera.position.distanceTo(point)*Math.tan(FOV*Math.PI/360)/Math.max(1,size.height);

  function placeCamera() {
    const c=Math.cos(view.elevation);
    camera.position.set(view.target.x+view.distance*c*Math.cos(view.azimuth),view.target.y+view.distance*Math.sin(view.elevation),view.target.z+view.distance*c*Math.sin(view.azimuth));
    camera.up.copy(THREE.Object3D.DEFAULT_UP);camera.lookAt(view.target);
    camera.near=Math.max(1,view.distance/300);camera.far=view.distance*80+4e5;camera.updateProjectionMatrix();camera.updateMatrixWorld();
  }
  const along=new THREE.Vector3(),basisUp=new THREE.Vector3(),basisRight=new THREE.Vector3();
  function poseAircraft(a) {
    const scale=pixel(a.position);
    const length=46*scale;
    basisUp.set(0,1,0).addScaledVector(a.forward,-a.forward.y).normalize();basisRight.crossVectors(a.forward,basisUp);
    a.mesh.matrix.makeBasis(a.forward,basisUp,basisRight).scale(new THREE.Vector3(length,length,length)).setPosition(a.position);a.mesh.matrixWorldNeedsUpdate=true;
    const body=a.role==='target'&&(hover==='target-move'||drag?.key==='target-move');
    a.mesh.material.emissive.set(body?a.color:0x000000);a.mesh.material.emissiveIntensity=body?.35:0;
    const show=a.moving;
    for(const part of [a.shaft,a.head,a.tip,a.tiltUp,a.tiltDown])part.visible=show;
    if(show) {
      a.tipPoint.copy(a.position).addScaledVector(a.forward,ARROW_PX*scale);
      a.shaft.position.copy(a.position).addScaledVector(a.forward,(ARROW_PX-10)*scale/2);
      a.shaft.quaternion.setFromUnitVectors(THREE.Object3D.DEFAULT_UP,a.forward);a.shaft.scale.set(1.3*scale,(ARROW_PX-10)*scale,1.3*scale);
      a.head.position.copy(a.position).addScaledVector(a.forward,(ARROW_PX-5)*scale);
      a.head.quaternion.copy(a.shaft.quaternion);a.head.scale.set(5*scale,11*scale,5*scale);
      const hot=hover===`${a.role}-heading`||drag?.key===`${a.role}-heading`;
      a.tip.position.copy(a.tipPoint);a.tip.scale.setScalar((hot?9:7)*scale);a.tip.material.opacity=hot?.8:.4;
      // Elevation handle: a small double arrow standing just above the tip.
      const hotTilt=hover===`${a.role}-tilt`||drag?.key===`${a.role}-tilt`,tilt=(hotTilt?1.3:1)*scale;
      a.tiltPoint.copy(a.tipPoint);a.tiltPoint.y+=TILT_PX*scale;
      a.tiltUp.position.copy(a.tiltPoint);a.tiltUp.position.y+=5*tilt;a.tiltUp.scale.set(3.5*tilt,7*tilt,3.5*tilt);
      a.tiltDown.position.copy(a.tiltPoint);a.tiltDown.position.y-=5*tilt;a.tiltDown.scale.set(3.5*tilt,-7*tilt,3.5*tilt);
    }
    // Altitude gizmo: a small double arrow standing above the aircraft.
    const hot=hover===`${a.role}-altitude`||drag?.key===`${a.role}-altitude`,grow=hot?1.3:1;
    a.gizmoPoint.copy(a.position);a.gizmoPoint.y+=ALTITUDE_PX*scale;
    a.up.position.copy(a.gizmoPoint);a.up.position.y+=8*scale*grow;a.up.scale.set(5*scale*grow,10*scale*grow,5*scale*grow);
    a.down.position.copy(a.gizmoPoint);a.down.position.y-=8*scale*grow;a.down.scale.set(5*scale*grow,-10*scale*grow,5*scale*grow);
    along.set(a.position.x,0,a.position.z);setLine(a.drop,a.position,along);
    a.drop.material.dashSize=a.drop.material.gapSize=5*pixel(along.lerp(a.position,.5));
    a.foot.position.set(a.position.x,1,a.position.z);a.foot.scale.setScalar(6*pixel(a.foot.position));
  }
  function ticks() {
    // Axis numbers (km) at the grid's major lines, skipped where they would crowd.
    const major=niceStep(view.distance/3),limit=major*Math.ceil(view.distance*1.1/major),placed=[];
    const put=(point,text,className='missile-setup-tick',dy=4,gap=46)=>{
      const s=toScreen(point);if(!s.visible||s.x<0||s.y<0||s.x>size.width||s.y>size.height)return null;
      if(placed.some(p=>Math.hypot(p.x-s.x,p.y-s.y)<gap))return null;placed.push(s);label(text,s.x,s.y+dy,className);return s;
    };
    const km=value=>`${+(value/1000).toFixed(3)}`;
    const centre=[view.target.x,view.target.z],point=new THREE.Vector3();
    for(const axis of [0,1]) {
      const base=Math.round(centre[axis]/major)*major;let far=null;
      for(let i=-8;i<=8;i++) {
        const value=base+i*major;if(Math.abs(value-centre[axis])>limit)continue;
        point.set(axis===0?value:0,0,axis===0?0:value);
        if(put(point,km(value)))far=Math.max(far??value,value);
      }
      if(far!=null){point.set(axis===0?far+major*.5:0,0,axis===0?0:far+major*.5);put(point,axis===0?'X · km':'Z · km','missile-setup-axis');}
    }
    const top=Math.max(state?Math.max(state.launch.position[1],state.target.position[1])*1.25:0,major*2);
    setLine(altitudeAxis,point.set(0,0,0),along.set(0,top,0));
    const rise=niceStep(top/4);
    for(let value=rise;value<=top;value+=rise)put(point.set(0,value,0),km(value),'missile-setup-tick',-7,30);
    put(point.set(0,top,0),'Y = altitude · km','missile-setup-axis',-22,30);
  }
  function drawLabels() {
    poolUsed=0;ticks();
    if(state) {
      const L=aircraft.launch.position,T=aircraft.target.position,mid=along.copy(L).lerp(T,.5),s=toScreen(mid);
      const total=L.distanceTo(T),rise=T.y-L.y;
      if(s.visible)label(`${(total/1000).toFixed(2)} km${Math.abs(rise)>=1?` <span>${rise>0?'+':'−'}${Math.round(Math.abs(rise)).toLocaleString()} m</span>`:''}`,s.x,s.y-22,'missile-setup-distance');
      for(const [role,name] of [['launch','Launcher'],['target','Target']]) {
        const a=state[role],p=toScreen(aircraft[role].position);if(!p.visible)continue;
        const [heading,climb]=a.angles;
        label(`${name}<span>HDG ${degrees(heading)}${climb?` · EL ${degrees(climb)}`:''} · ${Math.round(a.position[1]).toLocaleString()} m</span>`,p.x,p.y+20,'missile-setup-aircraft',aircraft[role].color);
      }
    }
    for(let i=poolUsed;i<pool.length;i++)if(!pool[i].hidden)pool[i].hidden=true;
  }
  function render() {
    const width=container.clientWidth,height=container.clientHeight;
    if(!width||!height)return;
    if(width!==size.width||height!==size.height){size.width=width;size.height=height;renderer.setSize(width,height,false);camera.aspect=width/height;}
    placeCamera();buildGrid();
    for(const a of Object.values(aircraft)){a.mesh.visible=!!state;for(const part of [a.up,a.down,a.drop,a.foot])part.visible=!!state;if(state)poseAircraft(a);else for(const part of [a.shaft,a.head,a.tip,a.tiltUp,a.tiltDown])part.visible=false;}
    lineOfSight.visible=groundLine.visible=!!state;
    if(state) {
      const L=aircraft.launch.position,T=aircraft.target.position;
      setLine(lineOfSight,L,T);lineOfSight.material.dashSize=lineOfSight.material.gapSize=5*pixel(along.copy(L).lerp(T,.5));
      setLine(groundLine,along.set(L.x,0,L.z),basisUp.set(T.x,0,T.z));groundLine.material.dashSize=groundLine.material.gapSize=5*pixel(along.lerp(basisUp,.5));
    }
    renderer.render(scene,camera);drawLabels();
    const d=camera.position.clone().sub(view.target).normalize();onCamera?.({x:d.x,y:-d.z,z:d.y});
  }
  // Renders only when something changed; keeps going while the camera settles on its goal.
  function frame(now) {
    scheduled=false;const dt=Math.min(.05,(now-(lastFrame||now))/1000);lastFrame=now;
    const f=1-Math.exp(-DAMPING*dt);
    view.azimuth+=(goal.azimuth-view.azimuth)*f;view.elevation+=(goal.elevation-view.elevation)*f;
    view.distance*=(goal.distance/view.distance)**f;view.target.lerp(goal.target,f);
    render();
    const moving=Math.abs(goal.azimuth-view.azimuth)>1e-4||Math.abs(goal.elevation-view.elevation)>1e-4||Math.abs(goal.distance/view.distance-1)>1e-3||goal.target.distanceTo(view.target)>view.distance*1e-4;
    if(moving)invalidate();else lastFrame=0;
  }
  function invalidate(){if(!scheduled){scheduled=true;requestAnimationFrame(frame);}}

  // What is under the pointer: arrow tips, elevation and altitude gizmos first, then the target's body. The launcher stays above
  // the origin, so only its altitude and heading move.
  function pick(x,y) {
    if(!state)return null;
    let best=null;
    const consider=(key,point,radius,bias=1)=>{const s=toScreen(point);if(!s.visible)return;const d=Math.hypot(s.x-x,s.y-y);if(d<=radius&&(!best||d*bias<best.score))best={key,score:d*bias};};
    for(const role of ['launch','target']) {
      const a=aircraft[role];
      if(a.moving){consider(`${role}-heading`,a.tipPoint,14);consider(`${role}-tilt`,a.tiltPoint,12);}
      consider(`${role}-altitude`,a.gizmoPoint,15);
    }
    consider('target-move',aircraft.target.position,24,1.4);
    return best?.key||null;
  }
  const ndc=new THREE.Vector2(),ray=new THREE.Raycaster(),plane=new THREE.Plane(),hit=new THREE.Vector3(),normal=new THREE.Vector3();
  function pointerOn(x,y,planeNormal,through) {
    ndc.set(x/size.width*2-1,1-y/size.height*2);ray.setFromCamera(ndc,camera);
    plane.setFromNormalAndCoplanarPoint(planeNormal,through);
    if(!ray.ray.intersectPlane(plane,hit))return null;
    // A ray skimming the plane lands absurdly far away.
    return hit.distanceTo(through)>40*view.distance?null:hit;
  }
  function verticalNormal(){normal.set(camera.position.x-view.target.x,0,camera.position.z-view.target.z);if(normal.lengthSq()<1e-6)normal.set(1,0,0);return normal.normalize();}
  function startDrag(key,x,y) {
    const role=key.startsWith('launch')?'launch':'target',a=aircraft[role];
    const point=key.endsWith('altitude')?pointerOn(x,y,verticalNormal(),a.gizmoPoint):pointerOn(x,y,THREE.Object3D.DEFAULT_UP,a.position);
    // Grabbed where it was clicked: the aircraft does not snap to the pointer.
    const grab=point?(key.endsWith('altitude')?a.position.y-point.y:key==='target-move'?a.position.clone().sub(point):null):null;
    drag={key,role,grab,startY:y,startClimb:a.climb};goal.target.copy(view.target);goal.azimuth=view.azimuth;goal.elevation=view.elevation;goal.distance=view.distance;canvas.style.cursor=cursorOf(key,true);invalidate();
  }
  function moveDrag(x,y) {
    const a=aircraft[drag.role];
    if(drag.key==='target-move') {
      const point=pointerOn(x,y,THREE.Object3D.DEFAULT_UP,a.position);if(!point)return;
      onDrag({kind:'move',x:point.x+(drag.grab?.x||0),z:point.z+(drag.grab?.z||0)});
    } else if(drag.key.endsWith('heading')) {
      // In the tip's own horizontal plane, so a tilted arrow does not swing when pressed; elevation stays as it is.
      const point=pointerOn(x,y,THREE.Object3D.DEFAULT_UP,a.tipPoint);if(!point)return;
      onDrag({kind:'heading',role:drag.role,heading:Math.atan2(point.z-a.position.z,point.x-a.position.x)*180/Math.PI});
    } else if(drag.key.endsWith('tilt')) {
      // Up raises the nose, at about the rate the tip would follow the pointer; the heading stays as it is.
      onDrag({kind:'climb',role:drag.role,climb:drag.startClimb+(drag.startY-y)*TILT_RATE});
    } else {
      const point=pointerOn(x,y,verticalNormal(),a.gizmoPoint);if(!point)return;
      onDrag({kind:'altitude',role:drag.role,altitude:point.y+(drag.grab||0)});
    }
  }
  const cursorOf=(key,active)=>!key?(active?'grabbing':'default'):key==='target-move'?'move':key.endsWith('altitude')||key.endsWith('tilt')?'ns-resize':active?'grabbing':'grab';
  const local=event=>{const box=canvas.getBoundingClientRect();return [event.clientX-box.left,event.clientY-box.top];};
  function pan(dx,dy) {
    const scale=2*goal.distance*Math.tan(FOV*Math.PI/360)/Math.max(1,size.height),right=new THREE.Vector3().setFromMatrixColumn(camera.matrixWorld,0),up=new THREE.Vector3().setFromMatrixColumn(camera.matrixWorld,1);
    goal.target.addScaledVector(right,-dx*scale).addScaledVector(up,dy*scale);
  }
  canvas.addEventListener('contextmenu',event=>event.preventDefault());
  canvas.addEventListener('pointerdown',event=>{
    canvas.setPointerCapture(event.pointerId);const [x,y]=local(event);pointers.set(event.pointerId,[x,y]);
    if(pointers.size===2){drag=null;orbit={mode:'pinch',spread:null};return;}
    const key=event.button===0&&!event.shiftKey?pick(x,y):null;
    if(key){startDrag(key,x,y);moveDrag(x,y);return;}
    orbit={mode:event.button===0&&!event.shiftKey?'turn':'pan',x,y};canvas.style.cursor=orbit.mode==='pan'?'move':'grabbing';
  });
  canvas.addEventListener('pointermove',event=>{
    const [x,y]=local(event);
    if(pointers.has(event.pointerId))pointers.set(event.pointerId,[x,y]);
    if(drag){moveDrag(x,y);return;}
    if(orbit?.mode==='pinch'&&pointers.size===2) {
      const [a,b]=[...pointers.values()],spread=Math.hypot(a[0]-b[0],a[1]-b[1]),mx=(a[0]+b[0])/2,my=(a[1]+b[1])/2;
      if(orbit.spread){goal.distance=Math.min(MAX_DISTANCE,Math.max(MIN_DISTANCE,goal.distance*orbit.spread/Math.max(1,spread)));pan(mx-orbit.mx,my-orbit.my);invalidate();}
      Object.assign(orbit,{spread,mx,my});return;
    }
    if(orbit) {
      const dx=x-orbit.x,dy=y-orbit.y;orbit.x=x;orbit.y=y;
      if(orbit.mode==='pan')pan(dx,dy);
      else{goal.azimuth+=dx*.0065;goal.elevation=Math.min(MAX_ELEVATION,Math.max(MIN_ELEVATION,goal.elevation+dy*.0065));}
      invalidate();return;
    }
    const key=pick(x,y);if(key!==hover){hover=key;canvas.style.cursor=cursorOf(key,false);invalidate();}
  });
  const release=event=>{
    pointers.delete(event.pointerId);if(!pointers.size){drag=null;orbit=null;canvas.style.cursor=cursorOf(hover,false);invalidate();}
  };
  canvas.addEventListener('pointerup',release);canvas.addEventListener('pointercancel',release);
  canvas.addEventListener('pointerleave',()=>{if(!drag&&!orbit&&hover){hover=null;canvas.style.cursor='default';invalidate();}});
  // The wheel zooms about the point under the pointer.
  canvas.addEventListener('wheel',event=>{
    event.preventDefault();const [x,y]=local(event);
    const factor=Math.exp(Math.max(-120,Math.min(120,event.deltaY*(event.deltaMode===1?33:1)))*.0016),next=Math.min(MAX_DISTANCE,Math.max(MIN_DISTANCE,goal.distance*factor));
    const shrink=1-next/goal.distance;pan(-(x-size.width/2)*shrink,-(y-size.height/2)*shrink);goal.distance=next;invalidate();
  },{passive:false});
  canvas.addEventListener('dblclick',event=>{const [x,y]=local(event);if(!pick(x,y))fit();});

  // The view that holds both aircraft and the ground beneath them (the ground left out when it is far below them).
  function fit(immediate=false) {
    if(!state)return;
    const box=new THREE.Box3(),L=aircraft.launch.position,T=aircraft.target.position,across=Math.max(Math.hypot(T.x-L.x,T.z-L.z),1000);
    box.expandByPoint(L).expandByPoint(T);
    if(Math.max(L.y,T.y)<3*across){box.expandByPoint(new THREE.Vector3(L.x,0,L.z)).expandByPoint(new THREE.Vector3(T.x,0,T.z));}
    const sphere=box.getBoundingSphere(new THREE.Sphere()),aspect=(container.clientWidth||16)/(container.clientHeight||9),half=Math.min(FOV,2*Math.atan(Math.tan(FOV*Math.PI/360)*aspect)*180/Math.PI)*Math.PI/360;
    // Looking a little above the box's middle keeps the aircraft, not the ground, at the centre of the picture.
    goal.target.copy(sphere.center);goal.target.y=Math.max(goal.target.y,.68*Math.max(L.y,T.y));goal.distance=Math.max(MIN_DISTANCE,Math.max(sphere.radius,500)*1.2/Math.sin(half));
    goal.azimuth=1.2;goal.elevation=.34;
    if(immediate){view.target.copy(goal.target);view.distance=goal.distance;view.azimuth=goal.azimuth;view.elevation=goal.elevation;}
    invalidate();
  }
  return window.missileSetup3d.current={
    // launch and target: {position:[x,altitude,z] m, velocity:[x,y,z] m/s, angles:[heading,climb,0]}.
    update(next) {
      state=next;
      for(const role of ['launch','target']) {
        const a=aircraft[role],s=next[role],[heading,climb]=s.angles,h=heading*Math.PI/180,c=climb*Math.PI/180;
        a.role=role;a.position.set(s.position[0],s.position[1],s.position[2]);
        a.climb=climb;a.forward.set(Math.cos(c)*Math.cos(h),Math.sin(c),Math.cos(c)*Math.sin(h));a.moving=Math.hypot(...s.velocity)>0;
      }
      if(!framed&&container.clientWidth){framed=true;fit(true);}
      invalidate();
    },
    clear(){state=null;invalidate();},
    fit,
    setModel(model){const geometry=aircraftGeometry(model);for(const a of Object.values(aircraft)){a.mesh.geometry.dispose();a.mesh.geometry=geometry;}invalidate();},
    get dragging(){return !!drag;},
    // Screen position of a handle ('target-move', 'launch-altitude', 'target-heading', ...), for tests.
    screenOf(key){const a=aircraft[key.startsWith('launch')?'launch':'target'];return toScreen(key==='target-move'?a.position:key.endsWith('altitude')?a.gizmoPoint:key.endsWith('tilt')?a.tiltPoint:a.tipPoint);},
  };
}

