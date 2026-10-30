(() => {
  'use strict';
  let catalog = null, targetModel = null, loading = null;
  async function readAsset(url) {
    const response = await fetch(url);
    if (!response.ok) throw new Error('Trajectory mesh asset unavailable: '+url);
    const data = await response.json();
    if (data.schema !== 1) throw new Error('Unsupported trajectory mesh format: '+url);
    return data;
  }
  async function load() {
    if (!loading) loading = Promise.all([
      readAsset('./missile-models.json').then(data => { catalog = data; }).catch(error => console.warn(error.message)),
      readAsset('./target-model.json').then(data => { targetModel = data.model; }).catch(error => console.warn(error.message))
    ]);
    return loading;
  }
  const get = id => catalog?.models[catalog.missiles[id]] || null;
  const getTarget = () => targetModel;
  const normalized = q => {
    if (!Array.isArray(q) || q.length !== 4 || !q.every(Number.isFinite)) return null;
    const n = Math.hypot(...q); return n > 1e-12 ? q.map(v => v / n) : null;
  };
  function attitude(sample, next, fraction, previous) {
    const a = normalized(sample.quaternion) || normalized(previous?.quaternion), b = normalized(next?.quaternion);
    if (!a) {
      const direction = sample.velocity || (next ? next.missile.map((v, i) => v - sample.missile[i]) : [1, 0, 0]);
      const yaw = -Math.atan2(direction[2], direction[0]) / 2;
      const pitch = Math.atan2(direction[1], Math.hypot(direction[0], direction[2])) / 2;
      return [Math.sin(yaw)*Math.sin(pitch), Math.sin(yaw)*Math.cos(pitch), Math.cos(yaw)*Math.sin(pitch), Math.cos(yaw)*Math.cos(pitch)];
    }
    if (!b || !fraction) return a;
    let dot = a.reduce((sum, v, i) => sum + v * b[i], 0);
    const end = dot < 0 ? b.map(v => -v) : b; dot = Math.min(1, Math.abs(dot));
    if (dot > .9995) return normalized(a.map((v, i) => v + (end[i] - v) * fraction));
    const angle = Math.acos(dot), divisor = Math.sin(angle);
    return a.map((v, i) => (v*Math.sin((1-fraction)*angle) + end[i]*Math.sin(fraction*angle)) / divisor);
  }
  function rotate([vx, vy, vz], [x, y, z, w]) {
    const tx = 2*(y*vz-z*vy), ty = 2*(z*vx-x*vz), tz = 2*(x*vy-y*vx);
    return [vx+w*tx+y*tz-z*ty, vy+w*ty+z*tx-x*tz, vz+w*tz+x*ty-y*tx];
  }
  function targetAttitude(target) {
    let velocity = target.velocity;
    if (!velocity || Math.hypot(...velocity) < 1e-9) {
      const heading = (target.angles?.[0] || 0)*Math.PI/180, climb = (target.angles?.[1] || 0)*Math.PI/180;
      velocity = [Math.cos(climb)*Math.cos(heading),Math.sin(climb),Math.cos(climb)*Math.sin(heading)];
    }
    // The constant-velocity target, like the setup preview, has zero bank.
    return attitude({velocity},null,0);
  }
  function hull(points) {
    const sorted = points.slice().sort((a,b) => a[0]-b[0] || a[1]-b[1]);
    const cross = (o,a,b) => (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0]);
    const half = source => {
      const result = [];
      for (const p of source) { while (result.length >= 2 && cross(result.at(-2),result.at(-1),p) <= 0) result.pop(); result.push(p); }
      return result;
    };
    const lower = half(sorted), upper = half(sorted.slice().reverse());
    const outline = lower.slice(0,-1).concat(upper.slice(0,-1));
    return outline.length ? [...outline, outline[0]] : [];
  }
  function geometry(model, position, quaternion, length, view) {
    const axes = {'3d':[0,2,1],xz:[0,2],xy:[0,1],zy:[2,1]}[view];
    const points = model.vertices.map(vertex => {
      const p = rotate(vertex, quaternion);
      return axes.map(axis => position[axis] + p[axis] * length);
    });
    let projected = view === '3d' ? points : hull(points);
    if (view !== '3d' && model.project_faces) {
      // Consistent winding makes the filled triangle paths form a silhouette,
      // preserving the aircraft's wing/tail gaps instead of a convex outline.
      projected = model.triangles.flatMap(face => {
        const [a,b,c] = face.map(i=>points[i]);
        const cross = (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]);
        return cross > 0 ? [a,b,c,a,null] : [a,c,b,a,null];
      });
    }
    return Object.fromEntries(axes.map((_,i) => [['x','y','z'][i],projected.map(p => p === null ? null : p[i])]));
  }
  function trace(model, name, color, view, interactive=true) {
    const base = {name,showlegend:false,...(interactive?{hovertemplate:'%{meta}<extra></extra>'}:{hoverinfo:'skip'}),x:[],y:[]};
    if (view !== '3d') return {...base,type:'scatter',mode:'lines',fill:'toself',fillcolor:color,
      line:{color,width:model.project_faces?0:1},hoveron:'fills'};
    return {...base,type:'mesh3d',z:[],i:model.triangles.map(f=>f[0]),j:model.triangles.map(f=>f[2]),k:model.triangles.map(f=>f[1]),
      color,flatshading:true,lighting:{ambient:.65,diffuse:.8,specular:.25,roughness:.65},
      lightposition:{x:1000,y:-1000,z:2000}};
  }
  function displayLength(model, chart, view, bounds) {
    const layout = chart._fullLayout;
    const span = Math.max(100,...bounds.map(([low,high])=>high-low));
    let metersPerPixel = span / Math.max(400,chart.clientHeight || 0);
    if (view === '3d' && layout?.scene) {
      const scene = layout.scene, ratios = scene.aspectratio;
      if (ratios) metersPerPixel = Math.max(...['x','y','z'].map(axis =>
        Math.abs(scene[axis+'axis'].range[1]-scene[axis+'axis'].range[0]) / ratios[axis])) * 2 / Math.max(360,chart.clientHeight);
      const eye = scene.camera?.eye;
      const center = scene.camera?.center;
      if (eye) metersPerPixel *= Math.hypot(eye.x-(center?.x||0),eye.y-(center?.y||0),eye.z-(center?.z||0)) / 1.75;
    } else if (layout?.xaxis && layout?.yaxis) {
      metersPerPixel = Math.max(...['xaxis','yaxis'].map(axis =>
        Math.abs(layout[axis].range[1]-layout[axis].range[0]) / Math.max(1,layout[axis]._length)));
    }
    // These are plot markers: keep a small screen footprint even at close zoom.
    // Display size never changes simulation coordinates or hit geometry.
    return Math.max(1e-9,metersPerPixel*40);
  }
  const api = {load,get,getTarget,attitude,targetAttitude,rotate,hull,geometry,trace,displayLength};
  if (typeof module !== 'undefined') module.exports = api;
  else window.missileModels = api;
})();
