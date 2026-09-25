'use strict';
// Both views use the same checked surface. Never fill or triangulate its holes.
async function renderSurface(reset=false) {
  const id=state.resultId, token=++state.surfaceToken;
  if(reset)state.camera=null;
  if(!state.surface) {
    state.surfacePromise ??= request(base(id)+'/surface.json');
    let surface;
    try { surface=await state.surfacePromise; }
    catch(error) { if(id===state.resultId)state.surfacePromise=null;throw error; }
    if(id!==state.resultId || token!==state.surfaceToken || state.view!=='3d')return;
    state.surface=surface;
  }
  if(!state.surface)throw new Error('This saved plot has no surface. Recalculate to enable 3D.');
  const s=state.surface,c=state.chart.settings;
  let lo=Infinity,hi=-Infinity;
  for(const row of s.sep_mps)for(const z of row)if(Number.isFinite(z)){lo=Math.min(lo,z);hi=Math.max(hi,z);}
  if(!Number.isFinite(lo)){lo=-1;hi=1;}
  if(lo===hi){lo-=1;hi+=1;}
  const zero=Math.max(0,Math.min(1,-lo/(hi-lo)));
  const colors=lo>=0?[[0,'#244852'],[.45,'#299baa'],[1,'#bcf5d8']]:hi<=0?[[0,'#8d413b'],[.6,'#e59e70'],[1,'#bacad1']]:
    [[0,'#873e3d'],[zero*.55,'#e59e70'],[zero,'#acbdc7'],[zero+(1-zero)*.38,'#249aa8'],[1,'#bcf5d8']];
  const traces=[{type:'surface',name:'Specific excess power',x:s.speeds_kmh,
    y:s.altitudes_m.map(h=>s.speeds_kmh[0].map(()=>h)),z:s.sep_mps,
    colorscale:colors,cmin:lo,cmax:hi,connectgaps:false,meta:{kind:'surface'},
    lighting:{ambient:.72,diffuse:.78,specular:.16,roughness:.65,fresnel:.08},lightposition:{x:150,y:100,z:500},
    contours:{z:{show:true,start:Math.ceil(lo/c.contour_interval_mps)*c.contour_interval_mps,end:hi,
      size:Math.max(c.contour_interval_mps,Math.ceil((hi-lo)/25/c.contour_interval_mps)*c.contour_interval_mps),
      usecolormap:false,color:'#bdd8df',width:1,highlightcolor:'#ffffff'}},
    colorbar:{title:{text:'SEP · m/s',side:'right'},thickness:12,len:.7,x:1.01,tickfont:{size:10},outlinewidth:0},
    hovertemplate:'TAS %{x:.1f} km/h<br>Altitude %{y:.0f} m<br>SEP %{z:.1f} m/s<extra></extra>'}];
  const zeroContour=state.chart.contours.find(row=>row.sep_mps===0);
  if(zeroContour) {
    const xy=joined(zeroContour.paths);
    traces.push({type:'scatter3d',mode:'lines',...xy,z:xy.x.map(x=>x===null?null:0),
      line:{color:'#f0f6fa',width:5},name:'SEP = 0',meta:{kind:'contour'},connectgaps:false,
      hovertemplate:'SEP 0 m/s<br>TAS %{x:.1f} km/h<br>Altitude %{y:.0f} m<extra></extra>'});
  }
  if($('climb-visible').checked && state.route?.status==='complete') {
    const points=state.route.points;
    traces.push({type:'scatter3d',mode:'lines',name:'Energy guide',
      x:points.map(p=>p.speed_kmh),y:points.map(p=>p.altitude_m),z:points.map(p=>p.sep_mps),
      line:{color:'#f1cb64',width:7},meta:{kind:'route'},connectgaps:false,
      hovertemplate:'Energy guide<br>TAS %{x:.1f} km/h · %{y:.0f} m<br>SEP %{z:.1f} m/s<extra></extra>'});
  }
  if($('samples').checked) {
    const points=state.chart.points.filter(p=>p.valid);
    traces.push({type:'scatter3d',mode:'markers',name:'Solved samples',x:points.map(p=>p.speed_kmh),y:points.map(p=>p.altitude_m),z:points.map(p=>p.ps_mps),
      customdata:points.map(p=>[p.index]),marker:{size:2,color:'#e0edf1',opacity:.75},meta:{kind:'sample'},
      hovertemplate:'Solved sample<br>TAS %{x:.1f} km/h · %{y:.0f} m<br>SEP %{z:.2f} m/s<extra></extra>'});
  }
  if($('gaps').checked) {
    const gaps=state.chart.masked_cells.filter(c=>c.reason!=='physical boundary');
    traces.push({type:'scatter3d',mode:'markers',name:'Unresolved intervals on floor',
      x:gaps.map(c=>(c.bounds[0]+c.bounds[1])/2),y:gaps.map(c=>(c.bounds[2]+c.bounds[3])/2),z:gaps.map(()=>lo),
      text:gaps.map(c=>c.reason),marker:{size:3,color:'#ef977e',symbol:'x'},meta:{kind:'gap'},
      hovertemplate:'%{text}<br>TAS %{x:.1f} km/h · %{y:.0f} m<br>Shown on floor; SEP unavailable<extra></extra>'});
  }
  const axis=(title,range)=>({title:{text:title,font:{size:11}},range,gridcolor:'#2c4051',zerolinecolor:'#758897',
    showbackground:true,backgroundcolor:'#101d29',tickfont:{size:10},nticks:5,showspikes:false});
  const layout={paper_bgcolor:'#0e1721',font:{family:'-apple-system, BlinkMacSystemFont, Segoe UI, sans-serif',color:'#8ea7ba'},
    margin:{l:10,r:65,t:12,b:10},showlegend:false,uirevision:state.resultId+'3d',
    scene:{xaxis:axis('True airspeed · km/h',[c.speed_min_kmh,c.speed_max_kmh]),
      yaxis:axis('Altitude · m',[c.altitude_min_m,c.altitude_max_m]),zaxis:axis('SEP · m/s',[lo,hi]),
      aspectmode:'manual',aspectratio:{x:1.45,y:1.5,z:.9},dragmode:'orbit',
      camera:reset?defaultCamera():state.camera || defaultCamera()}};
  await Plotly.react($('chart'),traces,layout,{responsive:true,scrollZoom:true,displaylogo:false,modeBarButtonsToRemove:['toImage']});
  if(token!==state.surfaceToken || state.view!=='3d')return;
  $('chart').removeAllListeners('plotly_relayout');
  $('chart').on('plotly_relayout',event=>{if(event['scene.camera'])state.camera=event['scene.camera'];});
  $('chart').removeAllListeners('plotly_click');
  $('chart').on('plotly_click',event=>{
    const p=event.points[0];if(p.data.meta?.kind==='sample')inspect(p.customdata[0]);
    else if(['surface','contour'].includes(p.data.meta?.kind))inspectNearest(p.x,p.y);
  });
}
function defaultCamera(){return {eye:{x:1.6,y:1.7,z:1.1},up:{x:0,y:0,z:1},center:{x:0,y:0,z:-.08}};}
function inspectNearest(x,y) {
  const c=state.chart.settings,xr=c.speed_max_kmh-c.speed_min_kmh,yr=c.altitude_max_m-c.altitude_min_m;
  const candidates=state.chart.points.filter(q=>q.valid);
  const near=candidates.reduce((a,b)=>Math.hypot((a.speed_kmh-x)/xr,(a.altitude_m-y)/yr)<Math.hypot((b.speed_kmh-x)/xr,(b.altitude_m-y)/yr)?a:b,nullSafe(candidates));
  if(near.index!==undefined)inspect(near.index);
}
async function changeView(view) {
  if(state.view===view)return;
  if(state.chart && state.view==='2d') {
    const layout=$('chart').layout;
    if(layout?.xaxis?.range && layout?.yaxis?.range)
      state.planarRanges={x:[...layout.xaxis.range],y:[...layout.yaxis.range]};
  }
  state.view=view;state.surfaceToken++;
  const url=new URL(location.href);if(view==='3d')url.searchParams.set('view','3d');else url.searchParams.delete('view');history.replaceState(null,'',url);
  for(const mode of ['2d','3d'])$('view-'+mode).setAttribute('aria-pressed',String(mode===view));
  $('chart').setAttribute('aria-label',view==='3d'?'3D surface: airspeed, altitude and specific excess power':'Speed versus altitude with specific excess power contours');
  $('surface-download').hidden=view!=='3d' || !state.chart;
  try {await render();}catch(error){showError(error);}
}
