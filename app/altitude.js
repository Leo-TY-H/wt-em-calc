'use strict';
const $ = id => document.getElementById(id);
const state = {meta:null, chart:null, job:null, busy:false, config:null, token:0, pointToken:0, route:null, routeToken:0, view:'2d',surfaceToken:0};
const number = id => Number($(id).value);
const fmt = (v, digits=1) => Number.isFinite(v) ? v.toLocaleString(undefined,{maximumFractionDigits:digits}) : '—';
const base = id => `/api/altitude/jobs/${id}`;
const escapeHTML = value => String(value).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));

async function request(url, body) {
  const response = await fetch(url, body===undefined ? {} : {method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
  const data = await response.json();
  if(!response.ok) throw new Error(data.error || `Request failed (${response.status})`);
  return data;
}
function showError(error) { $('error').textContent=error.message || String(error); $('error').hidden=false; }
function readConfig() {
  return {aircraft:$('aircraft').value,speed_min_kmh:number('speed-min'),speed_max_kmh:number('speed-max'),
    altitude_min_m:number('altitude-min'),altitude_max_m:number('altitude-max'),quality:$('quality').value,
    contour_interval_mps:number('interval'),conditions:{fuel_percent:number('fuel'),throttle:number('throttle')/100,
      afterburner:number('throttle')>100,
      sweep_percent:$('sweep-field').hidden?0:number('sweep'),structural_limits:$('limits').checked}};
}
function catalog() {
  const selected=$('aircraft').value,query=$('search').value.toLocaleLowerCase().trim();
  const rows=Object.entries(state.meta.aircraft).filter(([id,a])=>a.supported && (id+' '+a.name).toLocaleLowerCase().includes(query));
  $('aircraft').replaceChildren();
  $('aircraft').add(new Option(rows.length?'Select an aircraft…':'No matching aircraft',''));
  for(const [id,a] of rows) $('aircraft').add(new Option(a.name,id));
  if(rows.some(([id])=>id===selected)) $('aircraft').value=selected;
  aircraftChanged();
}
function aircraftChanged() {
  const a=state.meta?.aircraft[$('aircraft').value];
  $('calculate').disabled=state.busy || !a;
  $('aircraft-info').textContent=a?a.name:'Select an aircraft to begin.';
  $('sweep-field').hidden=!a?.has_sweep;
  $('prop-hint').hidden=!(a?.propeller_count>0);
}
function markStale() { if(state.chart) $('stale').hidden=JSON.stringify(readConfig())===JSON.stringify(state.config); }
function busy(value) {
  state.busy=value; $('cancel').hidden=!value; $('progress').hidden=!value;
  $('calculate').disabled=value || !$('aircraft').value;
  $('calculate').textContent=value?'Calculating…':'Calculate altitude plot ↗';
  $('cancel').disabled=false;
}
async function calculate(event) {
  event?.preventDefault(); if(state.busy || !$('scenario').reportValidity()) return;
  const config=readConfig(), token=++state.token;
  if(config.speed_min_kmh>=config.speed_max_kmh || config.altitude_min_m>=config.altitude_max_m) return showError(new Error('Maximum speed and altitude must exceed their minimum values.'));
  $('error').hidden=true; busy(true); $('progress-label').textContent='Preparing aircraft…'; $('progress-detail').textContent='';
  state.job=null;
  try {
    const job=await request('/api/altitude/jobs',config); state.job=job.id;
    for(;;) {
      if(token!==state.token) return;
      const status=await request(base(job.id));
      if(status.status==='complete') { await loadResult(job.id,config); break; }
      if(status.status==='error') throw new Error(status.error);
      if(status.status==='cancelled') { $('progress-label').textContent='Calculation cancelled'; break; }
      const p=status.progress;
      $('progress-label').textContent=status.status==='exporting'?'Drawing contours…':p?.phase || 'Waiting for calculation…';
      $('progress-detail').textContent=p?`${fmt(p.samples,0)} samples · ${fmt(p.elapsed_s,0)} s elapsed`:'The altitude solver has its own calculation queue.';
      await new Promise(resolve=>setTimeout(resolve,200));
    }
  } catch(error) { showError(error); }
  finally { if(token===state.token) busy(false); }
}
async function loadResult(id, config=null) {
  const data=await request(base(id)+'/chart.json');
  if(!config) populate(data.settings);
  state.chart=data; state.resultId=id; state.config=config || readConfig();
  state.surface=null;state.surfacePromise=null;state.surfaceToken++;state.camera=null;state.planarRanges=null;
  $('surface-download').hidden=state.view!=='3d';
  state.routeToken++;state.route=data.energy_guide || null;
  const url=new URL(location.href);url.searchParams.set('job',id);history.replaceState(null,'',url);
  $('stale').hidden=true;
  for(const a of document.querySelectorAll('[data-export]')) { a.href=base(id)+'/'+a.dataset.export; a.setAttribute('aria-disabled','false'); a.setAttribute('download',a.dataset.export); }
  const valid=data.points.filter(p=>p.valid), gaps=data.points.filter(p=>p.category==='numerical gap');
  const unresolved=data.masked_cells.filter(c=>c.reason!=='physical boundary').length;
  const max=valid.length?Math.max(...valid.map(p=>p.ps_mps)):null;
  $('metrics').innerHTML=[[`${fmt(max)} m/s`,'Highest sampled SEP'],[fmt(valid.length,0),'Accepted operating points'],[fmt(gaps.length,0),'Unresolved trim samples'],[`${fmt(data.elapsed_s,1)} s`,'Calculation time']]
    .map(([value,label])=>`<div>${escapeHTML(value)}<span>${escapeHTML(label)}</span></div>`).join('');
  const c=data.settings;
  showRoute();
  $('summary').textContent=`${data.aircraft_name} · ${fmt(c.conditions.fuel_percent,0)}% fuel · ${fmt(c.conditions.throttle*100,0)}% throttle · ${fmt(c.altitude_min_m,0)}–${fmt(c.altitude_max_m,0)} m. ${unresolved} intervals remain unresolved.`;
  $('point-title').textContent='Select a contour or sample'; $('point-content').innerHTML='<p class="hint">Inspect the nearest solved point, its energy balance and trim residuals.</p>';
  await render(true); markStale();
}
function populate(config) {
  $('search').value=config.aircraft;catalog();$('aircraft').value=config.aircraft;aircraftChanged();
  const fields={'speed-min':'speed_min_kmh','speed-max':'speed_max_kmh','altitude-min':'altitude_min_m','altitude-max':'altitude_max_m','quality':'quality','interval':'contour_interval_mps'};
  for(const [id,key] of Object.entries(fields))$(id).value=config[key];
  const c=config.conditions;
  // Keep binary roundoff (1.1 * 100) below the number input's 110% maximum.
  $('fuel').value=c.fuel_percent;$('throttle').value=Number((c.throttle*100).toFixed(8));$('sweep').value=c.sweep_percent;
  $('limits').checked=c.structural_limits;
}
function joined(paths) {
  const x=[],y=[]; for(const path of paths) { for(const p of path) { x.push(p[0]); y.push(p[1]); } x.push(null); y.push(null); } return {x,y};
}
function updateExports() {
  if(!state.chart)return;
  const query=new URLSearchParams({guide_visible:$('climb-visible').checked && state.route?.status==='complete'?'1':'0'});
  for(const a of document.querySelectorAll('[data-export]'))
    a.href=base(state.resultId)+'/'+a.dataset.export+(a.dataset.export.startsWith('diagram.')?'?'+query:'');
}
function showRoute() {
  const r=state.route,ok=r?.status==='complete';
  $('climb-time').textContent=ok?`${fmt(r.peak.sep_mps)} m/s`:'—';
  $('climb-summary').textContent=ok?`Gold follows a continuous best-SEP speed schedule from ${fmt(r.points[0].altitude_m,0)} to ${fmt(r.points.at(-1).altitude_m,0)} m. Peak on the guide: ${fmt(r.peak.sep_mps)} m/s at ${fmt(r.peak.speed_kmh)} km/h and ${fmt(r.peak.altitude_m,0)} m.`:
    r?.reason || 'Calculate a plot to build the maximum-SEP guide.';
  const link=$('climb-download');link.setAttribute('aria-disabled',ok?'false':'true');
  if(state.routeURL)URL.revokeObjectURL(state.routeURL);
  if(ok) {
    const keys=['altitude_m','speed_kmh','sep_mps','specific_power_w_kg'];
    const csv=[keys.join(','),...r.points.map(p=>keys.map(k=>p[k]).join(','))].join('\n')+'\n';
    state.routeURL=URL.createObjectURL(new Blob([csv],{type:'text/csv'}));link.href=state.routeURL;link.download='maximum-sep-guide.csv';
  } else link.removeAttribute('href');
  updateExports();
}
function render(reset=false) {
  // Serialize Plotly updates so rapid toggles cannot leave an older view
  // painted over the most recent selection.
  state.renderQueue=(state.renderQueue || Promise.resolve()).catch(()=>{}).then(()=>draw(reset));
  return state.renderQueue;
}
async function draw(reset=false) {
  const data=state.chart; if(!data)return;
  if(state.view==='3d')return renderSurface(reset);
  if(reset)state.planarRanges=null;
  const traces=[],annotations=[];
  for(const contour of data.contours) {
    const level=contour.sep_mps,color=level>0?'#46c6d5':level<0?'#edaa80':'#e7f2fc';
    traces.push({type:'scatter',mode:'lines',...joined(contour.paths),name:`SEP ${fmt(level)} m/s`,
      line:{color,width:level===0?3:1.2,dash:level<0?'dash':'solid'},connectgaps:false,
      meta:{kind:'contour',sep:level},hovertemplate:`SEP ${fmt(level)} m/s<br>TAS %{x:.1f} km/h<br>Altitude %{y:.0f} m<extra></extra>`});
    const path=contour.paths.reduce((best,p)=>p.length>best.length?p:best,[]);
    if(path.length>5) { const p=path[Math.floor(path.length*.56)]; annotations.push({x:p[0],y:p[1],text:fmt(level,0),showarrow:false,font:{color,size:10},bgcolor:'#0e1721',borderpad:2}); }
  }
  if(data.settings.conditions.structural_limits) traces.push({type:'scatter',mode:'lines',name:'IAS / Mach limit',
    x:data.speed_limits.map(p=>p.speed_kmh),y:data.speed_limits.map(p=>p.altitude_m),line:{color:'#b481a6',width:1.5},
    meta:{kind:'limit'},hovertemplate:'IAS / Mach limit<br>TAS %{x:.1f} km/h<br>Altitude %{y:.0f} m<extra></extra>'});
  if($('climb-visible').checked && state.route?.status==='complete') {
    const points=state.route.points;
    traces.push({type:'scatter',mode:'lines',name:'Continuous best-SEP guide',
      x:points.map(p=>p.speed_kmh),y:points.map(p=>p.altitude_m),customdata:points.map(p=>p.sep_mps),
      line:{color:'#f1cb64',width:3},meta:{kind:'route'},connectgaps:false,
      hovertemplate:'Best-SEP guide<br>TAS %{x:.1f} km/h · %{y:.0f} m<br>SEP %{customdata:.1f} m/s<extra></extra>'});
    const ends=[points[0],points[points.length-1]];
    traces.push({type:'scatter',mode:'markers',name:'Guide endpoints',x:ends.map(p=>p.speed_kmh),y:ends.map(p=>p.altitude_m),
      marker:{size:8,color:'#f1cb64',symbol:['circle','triangle-up']},hoverinfo:'skip',meta:{kind:'route'}});
  }
  if($('samples').checked) {
    const samples=data.points.filter(p=>p.valid);
    traces.push({type:'scattergl',mode:'markers',name:'Solved samples',x:samples.map(p=>p.speed_kmh),y:samples.map(p=>p.altitude_m),
      customdata:samples.map(p=>[p.index,p.ps_mps,p.alpha_deg,p.mach,p.flaps_percent]),meta:{kind:'sample'},marker:{size:4,color:'#7ab0c2',opacity:.65},
      hovertemplate:'TAS %{x:.1f} km/h · %{y:.0f} m<br>SEP %{customdata[1]:.2f} m/s<br>AoA %{customdata[2]:.2f}° · Mach %{customdata[3]:.3f}<br>Flaps %{customdata[4]:.0f}%<extra></extra>'});
  }
  if($('gaps').checked) {
    const unresolved=data.masked_cells.filter(c=>c.reason!=='physical boundary');
    traces.push({type:'scattergl',mode:'markers',name:'Unresolved cells',x:unresolved.map(c=>(c.bounds[0]+c.bounds[1])/2),y:unresolved.map(c=>(c.bounds[2]+c.bounds[3])/2),
      customdata:unresolved.map(c=>c.reason),meta:{kind:'gap'},marker:{size:6,symbol:'x',color:'#e38677'},
      hovertemplate:'%{customdata}<br>TAS %{x:.1f} km/h · %{y:.0f} m<extra></extra>'});
  }
  if(!data.contours.some(c=>c.paths.length)) annotations.push({xref:'paper',yref:'paper',x:.5,y:.5,showarrow:false,
    text:data.sampling.accepted_cells?'No contour crossings in this range. Enable Samples to inspect SEP.':'No checked surface in this range. Enable Unresolved to inspect gaps.',font:{color:'#b6cad9',size:12}});
  const c=data.settings;
  const layout={paper_bgcolor:'#0e1721',plot_bgcolor:'#0e1721',font:{family:'-apple-system, BlinkMacSystemFont, Segoe UI, sans-serif',color:'#8ea7ba',size:11},
    margin:{l:64,r:20,t:24,b:58},showlegend:false,annotations,dragmode:'pan',hovermode:'closest',
    uirevision:reset?`${state.resultId}-${Date.now()}`:state.viewRevision,
    xaxis:{title:'True airspeed · km/h',range:state.planarRanges?.x || [c.speed_min_kmh,c.speed_max_kmh],gridcolor:'#213142',zeroline:false},
    yaxis:{title:'Altitude · m',range:state.planarRanges?.y || [c.altitude_min_m,c.altitude_max_m],gridcolor:'#213142',zeroline:false}};
  state.viewRevision=layout.uirevision;
  await Plotly.react($('chart'),traces,layout,{responsive:true,scrollZoom:true,displaylogo:false,modeBarButtonsToRemove:['select2d','lasso2d','toImage']});
  $('chart').removeAllListeners('plotly_relayout');
  $('chart').on('plotly_relayout',()=>{
    const layout=$('chart').layout;
    if(layout?.xaxis?.range && layout?.yaxis?.range)
      state.planarRanges={x:[...layout.xaxis.range],y:[...layout.yaxis.range]};
  });
  $('chart').removeAllListeners('plotly_click');
  $('chart').on('plotly_click',event=>{
    const p=event.points[0],kind=p.data.meta?.kind;
    if(kind==='sample') inspect(p.customdata[0]);
    else if(kind==='contour') {
      const xr=c.speed_max_kmh-c.speed_min_kmh,yr=c.altitude_max_m-c.altitude_min_m;
      const candidates=data.points.filter(q=>q.valid);
      const near=candidates.reduce((a,b)=>Math.hypot((a.speed_kmh-p.x)/xr,(a.altitude_m-p.y)/yr)<Math.hypot((b.speed_kmh-p.x)/xr,(b.altitude_m-p.y)/yr)?a:b,nullSafe(candidates));
      if(near)inspect(near.index);
    }
  });
}
function nullSafe(rows){ return rows[0] || {speed_kmh:Infinity,altitude_m:Infinity}; }
async function inspect(index) {
  const token=++state.pointToken,id=state.resultId;
  try {
    const p=await request(base(id)+`/point/${index}`); if(token!==state.pointToken || id!==state.resultId)return;
    $('point-title').textContent=`${fmt(p.speed_kmh)} km/h · ${fmt(p.altitude_m,0)} m — ${p.valid?'accepted trim':p.reasons.join(', ')}`;
    const items=[[`${fmt(p.ps_mps,3)} m/s`,'Native SEP'],[`${fmt(p.ps_continuous_mps,3)} m/s`,'Force-projection SEP'],[`${fmt(p.alpha_deg,3)}°`,'Angle of attack'],[fmt(p.mach,3),'Mach'],[`${fmt(p.ias_kmh)} km/h`,'Longitudinal IAS'],[`${fmt(p.flaps_percent)}%`,'Flaps'],[p.force_error_g?.toExponential(2)||'—','Force residual · g'],[p.angular_error_rad_s2?.toExponential(2)||'—','Angular residual · rad/s²']];
    $('point-content').innerHTML='<div class="point-grid">'+items.map(([v,l])=>`<div><span>${escapeHTML(l)}</span><strong>${escapeHTML(v)}</strong></div>`).join('')+'</div><details><summary>Full solved state</summary><pre></pre></details>';
    $('point-content').querySelector('pre').textContent=JSON.stringify(p,null,2);
  } catch(error){showError(error);}
}
$('scenario').addEventListener('submit',calculate);
$('scenario').addEventListener('input',markStale);
$('scenario').addEventListener('change',markStale);
$('search').addEventListener('input',()=>{catalog();markStale();});
$('aircraft').addEventListener('change',aircraftChanged);
$('reset').addEventListener('click',()=>{if(state.busy)return; HTMLFormElement.prototype.reset.call($('scenario')); catalog(); markStale();});
$('cancel').addEventListener('click',async()=>{if(!state.job)return;try{await request(base(state.job)+'/cancel',{});$('cancel').disabled=true;$('progress-label').textContent='Cancelling…';}catch(error){showError(error);}});
for(const id of ['samples','gaps'])$(id).addEventListener('change',()=>render());
$('climb-visible').addEventListener('change',()=>{updateExports();render();});
$('reset-view').addEventListener('click',()=>render(true));
for(const mode of ['2d','3d'])$('view-'+mode).addEventListener('click',()=>changeView(mode));
$('surface-download').addEventListener('click',async()=>{try{await Plotly.downloadImage($('chart'),{format:'png',filename:'sep-surface-3d',width:1600,height:1100});}catch(error){showError(error);}});
if(new URL(location.href).searchParams.get('view')==='3d')changeView('3d');
request('/api/altitude/meta').then(async meta=>{state.meta=meta;catalog();const job=new URL(location.href).searchParams.get('job');if(job&&/^[a-f0-9]{20}$/.test(job))await loadResult(job);}).catch(error=>showError(new Error('Start “Launch Altitude Plotter.command” to use this feature. '+error.message)));
