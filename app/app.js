'use strict';
const $ = id => document.getElementById(id);
const runtimeConfig=window.EM_CONFIG||{};
const apiBase=String(runtimeConfig.apiBase||'').replace(/\/$/,'');
const remoteSite=!['localhost','127.0.0.1',''].includes(location.hostname);
const state = {data:null, dataJob:null, activeJob:null, view:'compare', meta:null, running:false, conditions:{}, entries:[], nextEntry:1, editing:null, hiddenVehicles:new Set(), contourLevels:[100,0,-100,-200,-400], contourRequest:0};
// Accuracy drives refinement; all presets start with a small speed stencil.
const quality = {quick:[9,7,1.], standard:[9,9,.5], fine:[9,13,.15]};
const fmt = (v,d=1) => Number.isFinite(v) ? v.toLocaleString(undefined,{minimumFractionDigits:d,maximumFractionDigits:d}) : '—';
const turnRadius = (speed,rate) => !Number.isFinite(speed)||!Number.isFinite(rate) ? '—' :
  rate===0 ? '∞ (straight flight)' : fmt((speed/3.6)/(Math.abs(rate)*Math.PI/180),0)+' m';
const escapeText = s => String(s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const apiUrl=path=>path.startsWith('/api/')?(apiBase?apiBase+path:'.'+path):path;
async function api(path, options={}) {
  // Relative API paths work both at localhost / and at a GitHub Pages project
  // path such as /repository-name/.
  const response=await fetch(apiUrl(path), options); const body=await response.json();
  if (!response.ok) throw new Error(body.error || `Request failed (${response.status})`);
  return body;
}
function error(message) {$('error').textContent=message;$('error').hidden=!message;}
const conditionFields={altitude:'altitude_m',fuel:'fuel_percent','extra-mass':'extra_mass_kg',
  timestep:'timestep_hz',sweep:'sweep_percent',flaps:'flaps_percent'};
const conditionKeys=[...Object.values(conditionFields),'throttle','afterburner','trim_mode','trim_limit',
  'fixed_trim','instructor','instructor_model','engine_control_mode','torque_gyro'];
const entryColors=['#38c9d7','#ffa66b','#b79aff','#91d477','#ee8eb6','#f0d367','#79a7fa','#cfb296'];
const entryName=e=>state.meta.aircraft[e.aircraft_id].name;
const conditionsFrom=c=>Object.fromEntries(conditionKeys.map(k=>[k,c[k]??state.meta.defaults[k]]));
// Old saved combinations remain unchanged in results. Editing selects the
// paired mode matching their Instructor setting, including inactive entries.
const modeConditions=c=>{const values=conditionsFrom(c);return {...values,instructor_model:'steady',engine_control_mode:'automatic',torque_gyro:!values.instructor};};
const aircraftConditions=(name,c)=>{const values=modeConditions(c);if(!state.meta.aircraft[name].has_flaps)values.flaps_percent=0;return values;};
const flightModeLabel=c=>c.instructor!==c.torque_gyro?(c.instructor?'RB':'SB'):
  `Instructor ${c.instructor?'on':'off'} · torque/gyro ${c.torque_gyro?'on':'off'}`;
const resultConditions=a=>({torque_gyro:true,...(a.settings||{...state.data.settings,...state.data.settings.aircraft_settings?.[a.id]})});
const instructorEvidence=a=>resultConditions(a).instructor?
  a.instructor_approximation?.capability_status||'Experimental Instructor boundary; see the calculation method.':'';
function saveCondition(){
  if(!state.editing)return;
  state.conditions[state.editing]={...state.conditions[state.editing],
    ...Object.fromEntries(Object.entries(conditionFields).map(([id,k])=>[k,+$(id).value])),
    engine_control_mode:'automatic',instructor_model:$('instructor-model').value,
    throttle:+$('throttle').value/100,afterburner:+$('throttle').value>100,
    instructor:$('flight-mode-rb').checked,torque_gyro:$('flight-mode-sb').checked};
  if(!state.meta.aircraft[state.entries.find(e=>e.id===state.editing).aircraft_id].has_flaps)state.conditions[state.editing].flaps_percent=0;
}
function showCondition(id){
  state.editing=id||null;
  const c=id?(state.conditions[id]??=conditionsFrom(state.meta.defaults)):state.meta.defaults;
  if(id&&!state.meta.aircraft[state.entries.find(e=>e.id===id).aircraft_id].has_flaps)c.flaps_percent=0;
  for(const [field,key] of Object.entries(conditionFields))$(field).value=c[key];
  $('engine-control-mode').value='automatic';
  $('instructor-model').value='steady';
  $('throttle').value=c.throttle*100;
  $('flight-mode-rb').checked=c.instructor;$('flight-mode-sb').checked=!c.instructor;
  $('aircraft-condition').disabled=!id;
}
function configEntries(c){
  if(c.entries)return c.entries.map(e=>({...e,settings:{torque_gyro:true,...e.settings}}));
  const entries=c.aircraft.map((name,i)=>({id:`entry_${i+1}`,aircraft_id:name,
    settings:aircraftConditions(name,{engine_control_mode:'automatic',torque_gyro:true,...c,...c.aircraft_settings?.[name]})}));
  if(c.compare_instructor&&entries.length===1)return [false,true].map((enabled,i)=>({...entries[0],id:`entry_${i+1}`,settings:{...entries[0].settings,instructor:enabled}}));
  return entries;
}
function renderEntries(){
  // Move the single settings editor into the active card; never clone form IDs.
  const editor=$('aircraft-condition');$('condition-home').append(editor);
  $('selected-aircraft').innerHTML=state.entries.map((e,i)=>`<article class="aircraft-entry ${e.id===state.editing?'active':''}" data-entry="${escapeText(e.id)}" style="--entry-color:${entryColors[i]}">
    <div class="entry-heading"><span class="entry-number">${i+1}</span><strong>${escapeText(entryName(e))}</strong></div>
    <p class="entry-summary"></p><div class="entry-actions">
    <button type="button" data-action="configure" aria-expanded="${e.id===state.editing}" aria-label="${e.id===state.editing?'Done configuring':'Configure'} entry ${i+1}">${e.id===state.editing?'Done':'Configure'}</button>
    <button type="button" data-action="duplicate" aria-label="Duplicate entry ${i+1}" ${state.entries.length>=8?'disabled':''}>Duplicate</button>
    <button type="button" data-action="remove" aria-label="Remove entry ${i+1}">Remove</button></div>
    <div class="entry-editor"></div></article>`).join('');
  const active=[...$('selected-aircraft').children].find(e=>e.dataset.entry===state.editing);
  if(active)active.querySelector('.entry-editor').append(editor);
  $('entries-empty').hidden=!!state.entries.length;
  $('entry-count').textContent=`${state.entries.length} / 8`;
  $('apply-settings-to-all').disabled=state.entries.length<2;
  $('apply-settings-status').textContent='';
  refreshEntrySummaries();
}
function refreshEntrySummaries(){
  for(const row of $('selected-aircraft').children){
    const c=state.conditions[row.dataset.entry];
    row.querySelector('.entry-summary').textContent=`${fmt(c.altitude_m,0)} m · ${fmt(c.fuel_percent,0)}% fuel · ${fmt(c.throttle*100,0)}% power · ${flightModeLabel(c)}`;
  }
}
function addEntry(name,condition){
  if(state.entries.length>=8)return;
  saveCondition();
  let id;do{id=`entry_${state.nextEntry++}`;}while(state.entries.some(e=>e.id===id));
  state.entries.push({id,aircraft_id:name});
  state.conditions[id]=structuredClone(aircraftConditions(name,condition||state.meta.defaults));
  showCondition(id);renderEntries();syncLabels();
}
function readConfig() {
  saveCondition();
  const grid=quality[$('quality').value] || [state.data?.settings.speed_samples||25,state.data?.settings.load_samples||19,state.data?.settings.sep_tolerance_mps||.5];
  return {...state.meta.defaults,aircraft:[...new Set(state.entries.map(e=>e.aircraft_id))],compare_instructor:false,aircraft_settings:{},
    entries:state.entries.map(e=>({...e,settings:{...state.conditions[e.id],structural_limits:true}})),
    speed_min_kmh:+$('speed-min').value,speed_max_kmh:+$('speed-max').value,max_load_g:null,
    speed_samples:grid[0],load_samples:grid[1],sampling:'adaptive',sep_tolerance_mps:grid[2],surface_resolution:601,
    sep_contour_levels_mps:[...state.contourLevels],heatmap:$('show-heatmap').checked};
}
function populate(c) {
  state.conditions={};state.editing=null;
  state.contourLevels=[...(c.sep_contour_levels_mps||[100,0,-100,-200,-400])];renderContourControls();
  $('show-heatmap').checked=c.heatmap??false;
  state.entries=configEntries(c).map(e=>{state.conditions[e.id]=aircraftConditions(e.aircraft_id,e.settings);return {id:e.id,aircraft_id:e.aircraft_id};});
  $('speed-min').value=c.speed_min_kmh;$('speed-max').value=c.speed_max_kmh;
  let selected=Object.keys(quality).find(k=>quality[k][2]===c.sep_tolerance_mps&&c.sampling==='adaptive') ||
    Object.keys(quality).find(k=>quality[k][0]===c.speed_samples&&quality[k][1]===c.load_samples&&quality[k][2]===c.sep_tolerance_mps);
  if(!selected){selected='custom';if(!$('quality').querySelector('[value=custom]'))$('quality').add(new Option('Saved sampling','custom'));}
  $('quality').value=selected;showCondition(state.entries[0]?.id);renderEntries();syncLabels();
}
function signature(c){
  const canonical=v=>Array.isArray(v)?v.map(canonical):v&&typeof v==='object'?
    Object.fromEntries(Object.keys(v).sort().map(k=>[k,canonical(v[k])])):v;
  const shared=Object.fromEntries(Object.entries(c).filter(([k])=>!conditionKeys.includes(k)&&!['aircraft','aircraft_settings','compare_instructor','entries','sep_contour_levels_mps','heatmap'].includes(k)));
  // Instance IDs identify exports/inspection, not physical conditions. Preserve
  // list order and duplicate multiplicity so colors and entries match the plot.
  shared.entries=configEntries(c).map(e=>({aircraft_id:e.aircraft_id,settings:conditionsFrom(e.settings)}));
  return JSON.stringify(canonical(shared));
}
function syncLabels(){
  $('fuel-label').textContent=$('fuel').value+'%';$('throttle-label').textContent=fmt(+$('throttle').value,0)+'%';
  $('sweep-label').textContent=$('sweep').value+'%';
  const aircraft=state.meta?.aircraft[state.entries.find(e=>e.id===state.editing)?.aircraft_id];
  $('engine-control-field').hidden=!aircraft||!aircraft.propulsion||aircraft.propulsion==='jet';
  $('flaps-field').hidden=!aircraft||!aircraft.has_flaps;
  $('flaps').disabled=!aircraft||!aircraft.has_flaps;
  if(aircraft&&!aircraft.has_flaps)$('flaps').value=0;
  $('flaps-label').textContent=$('flaps').value+'%';
  $('instructor-model-field').hidden=!$('flight-mode-rb').checked;
  $('sweep-field').hidden=!aircraft?.has_sweep;
  syncAircraftMenu();
  const config=readConfig();refreshEntrySummaries();
  $('stale').hidden=!state.data||(signature(config)===signature(state.data.settings)&&!needsHeatmap());
}
function syncAircraftMenu(){
  const normalizeSearch=value=>value.toLowerCase().normalize('NFKD').replace(/[^a-z0-9]/g,'');
  const query=normalizeSearch($('aircraft-search').value);
  let visible=0;
  for(const row of $('aircraft-list').children){
    row.hidden=!normalizeSearch(row.dataset.search).includes(query);
    if(!row.hidden)visible++;
    row.querySelector('button').disabled=row.dataset.supported!=='true'||state.entries.length>=8;
  }
  $('search-empty').hidden=visible>0;
}
function populateAircraft(){
  const entries=Object.entries(state.meta.aircraft).sort(([,a],[,b])=>Number(b.supported)-Number(a.supported)||a.name.localeCompare(b.name));
  const family=a=>({piston:'Piston propeller',turboprop:'Turboprop',mixed:'Propeller + jet'})[a.propulsion]||(a.has_sweep?'Variable sweep':'Jet');
  $('aircraft-list').innerHTML=entries.map(([id,a])=>`<div class="aircraft-option" data-search="${escapeText(id+' '+a.name+' '+family(a))}" data-supported="${!!a.supported}" title="${escapeText(a.reason||'')}"><span><strong>${escapeText(a.name)}</strong><small>${escapeText(a.supported?family(a)+(a.experimental?' · Experimental':''):a.reason||'Model integration pending')}</small></span><button type="button" data-add="${escapeText(id)}" aria-label="Add ${escapeText(a.name)}" ${a.supported?'':'disabled'}>Add</button></div>`).join('');
  const ready=entries.filter(([,a])=>a.supported).length;
  $('catalog-status').textContent=`${ready} available aircraft`;
}
$('aircraft-search').addEventListener('input',syncAircraftMenu);
$('aircraft-search').addEventListener('keydown',e=>{if(e.key==='Enter')e.preventDefault();});
$('aircraft-list').addEventListener('click',e=>{const b=e.target.closest('[data-add]');if(b&&!b.disabled)addEntry(b.dataset.add);});
$('selected-aircraft').addEventListener('click',event=>{
  const button=event.target.closest('[data-action]');if(!button||button.disabled)return;
  const id=button.closest('[data-entry]').dataset.entry,entry=state.entries.find(e=>e.id===id);
  saveCondition();
  if(button.dataset.action==='duplicate'){addEntry(entry.aircraft_id,state.conditions[id]);return;}
  if(button.dataset.action==='remove'){
    const index=state.entries.findIndex(e=>e.id===id);state.entries.splice(index,1);delete state.conditions[id];
    if(state.editing===id)showCondition(state.entries[Math.min(index,state.entries.length-1)]?.id);
  }else showCondition(state.editing===id?null:id);
  renderEntries();syncLabels();
});
$('apply-settings-to-all').addEventListener('click',()=>{
  if(!state.editing||state.entries.length<2)return;
  saveCondition();
  const settings=state.conditions[state.editing];
  for(const entry of state.entries)state.conditions[entry.id]=structuredClone(settings);
  renderEntries();syncLabels();
  $('apply-settings-status').textContent=`Applied to all ${state.entries.length} aircraft.`;
});
function setRunning(value){state.running=value;$('calculate').disabled=value;$('cancel').hidden=!value;$('progress').hidden=!value;}
async function loadData(id,populateForm=false,preview=false){
  const sameResult=state.dataJob===id;
  const data=await api(`/api/jobs/${id}/${preview?'preview.json':'chart.json'}`);state.data=data;state.dataJob=id;
  if(!sameResult)state.hiddenVehicles.clear();
  if(populateForm)populate(data.settings);
  if(state.view!=='compare'&&!data.aircraft.some(a=>a.id===state.view))state.view='compare';
  $('chart-tabs').innerHTML='<button type="button" data-view="compare">Compare</button>'+data.aircraft.map(a=>`<button type="button" data-view="${escapeText(a.id)}">${escapeText(a.name)}</button>`).join('');
  for(const button of document.querySelectorAll('[data-view]')){
    button.disabled=button.dataset.view!=='compare'&&!data.aircraft.some(a=>a.id===button.dataset.view);
    button.classList.toggle('active',button.dataset.view===state.view);
  }
  renderChart();syncLabels();
  updateContourAvailability();
  const unresolved=data.aircraft.reduce((sum,a)=>sum+(a.numerical_boundaries?.length||0),0);
  const gaps=data.aircraft.reduce((sum,a)=>sum+(a.numerical_gaps?.length||0),0);
  $('footer-status').textContent=preview?'Preview · solved samples only · gaps, contours and endpoints are still being refined':`Calculated in ${fmt(data.elapsed_s,1)} s · Saved ${remoteSite?'on the server':'locally'}${unresolved?' · '+unresolved+' unresolved boundary points':''}${gaps?' · '+gaps+' equilibrium gaps':''}${unresolved||gaps?' · Enable Rejected for details':''}`;
  if(!sameResult){
    const hasPoints=data.aircraft.some(a=>a.points.some(p=>p.valid));
    $('point-title').textContent=hasPoints?'Select a point on the diagram':'No valid operating points';
    $('point-content').innerHTML=hasPoints?'':'<p class="muted">No valid operating points in this range.</p>';
    $('point-status').textContent='';
  }
  if(!preview&&JSON.stringify([...state.contourLevels].sort((a,b)=>a-b))!==
      JSON.stringify([...(data.settings.sep_contour_levels_mps||[])].sort((a,b)=>a-b)))await refreshContours();
}
function rgba(hex,opacity){return `rgba(${parseInt(hex.slice(1,3),16)},${parseInt(hex.slice(3,5),16)},${parseInt(hex.slice(5,7),16)},${opacity})`;}
function renderContourControls(){
  $('contour-levels').innerHTML=state.contourLevels.map(level=>`<button type="button" data-contour-remove="${level}" aria-label="Remove SEP ${level} m/s">${level>0?'+':''}${level} ×</button>`).join('');
}
function supportedContourLevels(){
  if(!state.data)return [];
  return state.contourLevels.filter(level=>state.data.aircraft.every(a=>{
    if(a.interpolation?.scope==='contours')return a.interpolation.checked_sep_levels_mps.includes(level);
    const [low,high]=a.interpolation?.checked_sep_range_mps||[-300,300];return low<=level&&level<=high;
  }));
}
function needsHeatmap(){return !!state.data&&$('show-heatmap').checked&&state.data.settings.heatmap===false;}
function updateContourAvailability(){
  const missing=state.data?state.contourLevels.filter(level=>!supportedContourLevels().includes(level)):[];
  $('contour-status').textContent=[missing.length?`Calculate to validate ${missing.map(v=>`${v>0?'+':''}${v}`).join(', ')} m/s.`:'',needsHeatmap()?'Calculate to add the heatmap.':''].filter(Boolean).join(' ');
  if(!state.data)return;
  for(const link of document.querySelectorAll('[data-export]')){
    const figure=link.dataset.export.startsWith('diagram.');
    const disabled=state.data.preview||(figure&&(missing.length>0||needsHeatmap()));
    const query=figure?'?levels='+encodeURIComponent(state.contourLevels.join(','))+'&heatmap='+($('show-heatmap').checked?'1':'0'):'';
    link.href=disabled?'#':apiUrl(`/api/jobs/${state.dataJob}/${link.dataset.export}${query}`);
    link.download=`WT-EM-${state.data.aircraft.map(a=>a.id).join('-vs-')}-${link.dataset.export}`;
    link.setAttribute('aria-disabled',disabled?'true':'false');
  }
}
async function refreshContours(){
  const token=++state.contourRequest,data=state.data;
  updateContourAvailability();
  if(!data)return;
  if(data.preview){renderChart();return;}
  const levels=supportedContourLevels();
  try{
    const payload=await api(`/api/jobs/${state.dataJob}/contours.json?levels=${encodeURIComponent(levels.join(','))}`);
    if(token!==state.contourRequest||data!==state.data)return;
    for(const a of data.aircraft)a.contours=payload.contours[a.id]||[];
    renderChart();
  }catch(e){if(token===state.contourRequest)error(e.message);}
}
function sustainedCurve(aircraft,data){
  // Both preview and completed curves follow the currently selected contour.
  const paths=aircraft.contours.filter(c=>c.level===0);
  return paths.length?{x:paths.flatMap(p=>[...p.x,null]),y:paths.flatMap(p=>[...p.y,null])}:aircraft.sustained_curve;
}
function envelopeTop(aircraft){
  const peak=Math.max(0,...aircraft.flatMap(a=>(a.boundary||[]).map(p=>Number.isFinite(p.turn_dps)?p.turn_dps:0)));
  return Math.max(10,Math.ceil(peak*1.05+1));
}
function contourLabelTrace(a,data){
  const x=[],y=[],labels=[],levels=[];
  const xmin=data.settings.speed_min_kmh,xmax=data.settings.speed_max_kmh;
  const ymax=envelopeTop([a]);
  for(const level of state.contourLevels){
    const paths=a.contours.filter(path=>path.level===level&&path.x.length>2);
    let best=null;
    for(const path of paths){
      let length=0;for(let i=1;i<path.x.length;i++)length+=Math.hypot((path.x[i]-path.x[i-1])/(xmax-xmin),(path.y[i]-path.y[i-1])/ymax);
      if(!best||length>best.length)best={path,length};
    }
    if(!best||best.length<.06)continue;
    let progress=0;const target=best.length*(data.aircraft.indexOf(a)%2?.65:.42);
    for(let i=1;i<best.path.x.length;i++){
      const step=Math.hypot((best.path.x[i]-best.path.x[i-1])/(xmax-xmin),(best.path.y[i]-best.path.y[i-1])/ymax);
      if(progress+step>=target){const t=step?(target-progress)/step:0;
        x.push(best.path.x[i-1]+t*(best.path.x[i]-best.path.x[i-1]));
        y.push(best.path.y[i-1]+t*(best.path.y[i]-best.path.y[i-1]));
        labels.push(`SEP ${level>0?'+':''}${level}`);levels.push(level);break;
      }progress+=step;
    }
  }
  return {type:'scatter',mode:'markers+text',x,y,text:labels,customdata:levels,textposition:'middle center',
    textfont:{color:a.color,size:11,family:'WTSymbols, Segoe UI, sans-serif'},marker:{size:12,color:'rgba(0,0,0,0)'},
    meta:{id:a.id,kind:'contour-label'},legendgroup:a.id,showlegend:false,visible:state.hiddenVehicles.has(a.id)?'legendonly':true,
    hovertemplate:`${a.name}<br>SEP %{customdata} m/s<extra></extra>`};
}
function projectedContourPoint(event,a,level){
  const chart=$('chart'),rect=chart.getBoundingClientRect(),plot=chart._fullLayout._size;
  const [xmin,xmax]=chart._fullLayout.xaxis.range,[ymin,ymax]=chart._fullLayout.yaxis.range;
  const cursor=[(event.event.clientX-rect.left-plot.l)/plot.w,1-(event.event.clientY-rect.top-plot.t)/plot.h];
  let best=null;
  for(const path of a.contours.filter(path=>path.level===level))for(let i=1;i<path.x.length;i++){
    const start=[(path.x[i-1]-xmin)/(xmax-xmin),(path.y[i-1]-ymin)/(ymax-ymin)];
    const end=[(path.x[i]-xmin)/(xmax-xmin),(path.y[i]-ymin)/(ymax-ymin)];
    const dx=end[0]-start[0],dy=end[1]-start[1],length=dx*dx+dy*dy;
    const t=length?Math.max(0,Math.min(1,((cursor[0]-start[0])*dx+(cursor[1]-start[1])*dy)/length)):0;
    const distance=(cursor[0]-start[0]-t*dx)**2+(cursor[1]-start[1]-t*dy)**2;
    if(!best||distance<best.distance)best={distance,speed:path.x[i-1]+t*(path.x[i]-path.x[i-1]),rate:path.y[i-1]+t*(path.y[i]-path.y[i-1])};
  }
  return best;
}
function inspectContour(a,level,point){
  const load=Math.hypot(1,(point.speed/3.6)*(point.rate*Math.PI/180)/9.8100004196167);
  $('point-title').textContent=`${a.name} · ${fmt(point.speed,1)} km/h · ${fmt(load,2)} g`;
  $('point-status').innerHTML='<span class="point-pill">Interpolated contour</span>';
  $('point-content').innerHTML=`<div class="point-grid"><div><h3>FLIGHT & ENERGY</h3><dl>
    <dt>Native-step Ps</dt><dd>${fmt(level,1)} m/s</dd><dt>Turn rate</dt><dd>${fmt(point.rate,3)} °/s</dd>
    <dt>Turn radius</dt><dd>${turnRadius(point.speed,point.rate)}</dd><dt>True airspeed</dt><dd>${fmt(point.speed,1)} km/h</dd>
    <dt>Load</dt><dd>${fmt(load,3)} g</dd></dl></div></div>`;
}
function renderChart(){
  if(!state.data)return;
  const data=state.data,displayed=data.aircraft.filter(a=>state.view==='compare'||a.id===state.view),traces=[];
  const showSamples=$('show-samples').checked,showRejected=$('show-rejected').checked;
  if($('show-heatmap').checked&&displayed.length===1&&displayed[0].heatmap){
    const a=displayed[0],limit=a.ps_color_limit_mps||300;traces.push({type:'heatmap',x:a.heatmap.x||data.speeds_kmh,y:a.heatmap.y,z:a.heatmap.z,zmin:-limit,zmax:limit,
      colorscale:[[0,'#98633d'],[.33,'#54453b'],[.5,'#1b2c3d'],[.67,'#245867'],[1,'#399f9a']],
      zsmooth:false,connectgaps:false,hoverongaps:false,name:a.name,meta:a.id,legendgroup:a.id,showlegend:false,visible:state.hiddenVehicles.has(a.id)?'legendonly':true,
      colorbar:{title:{text:'Ps · m/s',side:'right',font:{size:10}},x:.995,xanchor:'right',thickness:9,len:.55,tickfont:{size:10},outlinewidth:0},
      hovertemplate:`%{x:.0f} km/h · %{y:.2f}°/s<br>Interpolated Ps %{z:.1f} m/s<extra></extra>`});
  }
  for(const n of [2,4,6,9,12,16]){
    if(n>(data.plot_max_load_g||data.settings.max_load_g))continue;
    traces.push({type:'scatter',mode:'lines',x:data.speeds_kmh,y:data.speeds_kmh.map(v=>180/Math.PI*9.8100004196167*Math.sqrt(n*n-1)/(v/3.6)),
      line:{color:'rgba(133,158,185,.16)',width:1},hoverinfo:'skip',showlegend:false});
  }
  for(const a of displayed){
    for(const level of [...new Set(a.contours.map(c=>c.level))]){
      if(!state.contourLevels.includes(level))continue;
      if(level===0&&a.sustained?.length>1)continue;
      const x=[],y=[];for(const path of a.contours.filter(c=>c.level===level)){x.push(...path.x,null);y.push(...path.y,null);}
      traces.push({type:'scatter',mode:'lines',x,y,name:`${a.name} · Ps ${level}`,meta:{id:a.id,kind:'contour',level},legendgroup:a.id,visible:state.hiddenVehicles.has(a.id)?'legendonly':true,
        line:{color:rgba(a.color,level===0?.95:displayed.length===1?.75:.46),width:level===0?3.6:2.2,dash:'dash'},
        text:x.map((v,i)=>turnRadius(v,y[i])),
        hovertemplate:`${a.name}<br>Ps ${level>0?'+':''}${level} m/s<br>%{x:.0f} km/h · %{y:.2f}°/s<br>Turn radius %{text}<extra></extra>`,showlegend:false});
    }
    traces.push({type:'scatter',mode:'lines',x:a.boundary.map(p=>p.speed_kmh),y:a.boundary.map(p=>p.turn_dps),
      name:a.name,meta:a.id,legendgroup:a.id,showlegend:true,visible:state.hiddenVehicles.has(a.id)?'legendonly':true,connectgaps:false,cliponaxis:false,line:{color:rgba(a.color,.95),width:2.8,dash:'solid'},
      customdata:a.boundary.map(p=>[Number.isFinite(p.alpha_deg)?fmt(p.alpha_deg,2)+'°':'—',turnRadius(p.speed_kmh,p.turn_dps)]),
      hovertemplate:`${a.name}<br>AoA %{customdata[0]}<br>Radius %{customdata[1]}<br>Rate %{y:.2f}°/s<br>Speed %{x:.0f} km/h<extra></extra>`});
    if(showRejected&&a.numerical_boundaries?.length){
      traces.push({type:'scatter',mode:'markers',x:a.numerical_boundaries.map(p=>p.speed_kmh),y:a.numerical_boundaries.map(p=>p.turn_dps),
        name:a.name+' unresolved boundary',meta:a.id,legendgroup:a.id,showlegend:false,visible:state.hiddenVehicles.has(a.id)?'legendonly':true,marker:{size:8,symbol:'x',color:'#f6be6d'},
        hovertemplate:'Unresolved numerical boundary<br>Highest verified interior point; aircraft limit not established<extra></extra>'});
    }
    if(showRejected&&a.numerical_gaps?.length){
      traces.push({type:'scatter',mode:'markers',x:a.numerical_gaps.map(p=>p.speed_kmh),y:a.numerical_gaps.map(p=>p.turn_dps),
        customdata:a.numerical_gaps.map(p=>p.valid_side_loads),name:a.name+' equilibrium gaps',meta:a.id,legendgroup:a.id,showlegend:false,visible:state.hiddenVehicles.has(a.id)?'legendonly':true,
        marker:{size:7,symbol:'circle-open',color:'#f6be6d',line:{width:1.5}},
        hovertemplate:'Unresolved local equilibrium<br>Valid solutions bracket %{customdata[0]:.5f}–%{customdata[1]:.5f} g<br>This narrow interval remains masked<extra></extra>'});
    }
    if(state.contourLevels.includes(0)&&a.sustained?.length){
      const roots=new Map(a.sustained.map((p,i)=>[p.speed_kmh,{p,i}]));
      const curve=sustainedCurve(a,data);
      traces.push({type:'scatter',mode:curve?'lines':'lines+markers',x:curve?.x||data.speeds_kmh,y:curve?.y||data.speeds_kmh.map(v=>roots.get(v)?.p.turn_dps??null),
        name:a.name+' Ps = 0',meta:{id:a.id,kind:'contour',level:0},legendgroup:a.id,showlegend:false,visible:state.hiddenVehicles.has(a.id)?'legendonly':true,connectgaps:false,line:{color:a.color,width:4,dash:'dash'},marker:{size:4,color:a.color},
        customdata:curve?null:data.speeds_kmh.map(v=>roots.has(v)?[a.id,'root',roots.get(v).i]:null),
        text:(curve?.x||data.speeds_kmh).map((v,i)=>{const rate=curve?curve.y[i]:roots.get(v)?.p.turn_dps;return turnRadius(v,rate);}),
        hovertemplate:`${a.name} · ${data.preview?'preview contour':'refined'} Ps = 0 m/s<br>%{x:.0f} km/h · %{y:.2f}°/s<br>Turn radius %{text}<extra></extra>`});
    }
    const valid=a.points.map((p,i)=>({p,i})).filter(r=>r.p.valid);
    if(showSamples)traces.push({type:'scatter',mode:'markers',x:valid.map(r=>r.p.speed_kmh),y:valid.map(r=>r.p.turn_dps),
      marker:{size:showSamples?5:9,color:showSamples?rgba(a.color,.5):'rgba(0,0,0,0)'},name:a.name,meta:a.id,legendgroup:a.id,showlegend:false,visible:state.hiddenVehicles.has(a.id)?'legendonly':true,
      customdata:valid.map(r=>[a.id,'grid',r.i]),text:valid.map(r=>`${a.name} · ${fmt(r.p.load_g,2)} g<br>Ps ${fmt(r.p.ps_mps,1)} m/s`),
      hovertemplate:'%{text}<br>%{x:.0f} km/h · %{y:.2f}°/s<extra></extra>'});
    if(showRejected){
      const rejected=a.points.map((p,i)=>({p,i})).filter(r=>!r.p.valid);
      traces.push({type:'scatter',mode:'markers',x:rejected.map(r=>r.p.speed_kmh),y:rejected.map(r=>r.p.turn_dps),
        marker:{size:5,symbol:'x',color:rgba(a.color,.33)},name:a.name+' rejected',meta:a.id,legendgroup:a.id,showlegend:false,visible:state.hiddenVehicles.has(a.id)?'legendonly':true,
        customdata:rejected.map(r=>[a.id,'grid',r.i]),text:rejected.map(r=>r.p.reasons.join(', ')),
        hovertemplate:`${a.name} · rejected<br>%{text}<br>%{x:.0f} km/h · %{y:.2f}°/s<extra></extra>`});
    }
    traces.push(contourLabelTrace(a,data));
  }
  const ymax=envelopeTop(displayed);
  const layout={paper_bgcolor:'#121925',plot_bgcolor:'#121925',font:{family:'WTSymbols, -apple-system, BlinkMacSystemFont, Segoe UI, sans-serif',color:'#a7b7cb',size:11},
    margin:{l:52,r:18,t:Math.max(38,displayed.length*20),b:46},hovermode:'closest',hoverlabel:{bgcolor:'#202e40',bordercolor:'#40536b',font:{color:'#eff7ff',size:11}},
    xaxis:{title:{text:'True airspeed · km/h',standoff:6,font:{size:11}},range:[data.settings.speed_min_kmh,data.settings.speed_max_kmh],gridcolor:'#253144',zeroline:false,dtick:100,tickfont:{size:10},constrain:'domain'},
    yaxis:{title:{text:'Turn rate · °/s',standoff:6,font:{size:11}},range:[0,ymax],gridcolor:'#253144',zeroline:false,dtick:5,tickfont:{size:10}},
    legend:{orientation:'h',x:0,y:1.02,yanchor:'bottom',font:{size:10},bgcolor:'rgba(0,0,0,0)',traceorder:'normal',groupclick:'togglegroup'},
    uirevision:state.dataJob+'-'+state.view+'-'+showRejected,dragmode:'pan'};
  if(!$('chart').classList.contains('js-plotly-plot'))$('chart').innerHTML='';
  Plotly.react('chart',traces,layout,{responsive:true,displaylogo:false,scrollZoom:true,modeBarButtonsToRemove:['select2d','lasso2d','toImage']});
  $('chart').removeAllListeners?.('plotly_legendclick');
  $('chart').on('plotly_legendclick',event=>{
    const group=event.data[event.curveNumber]?.legendgroup;if(!group)return false;
    const hidden=state.hiddenVehicles.has(group);
    if(hidden)state.hiddenVehicles.delete(group);else state.hiddenVehicles.add(group);
    const indices=event.data.flatMap((trace,i)=>trace.legendgroup===group?[i]:[]);
    Plotly.restyle('chart',{visible:hidden?true:'legendonly'},indices);
    return false;
  });
  $('chart').removeAllListeners?.('plotly_click');
  $('chart').on('plotly_click', event=>{
    const p=event.points?.[0];if(!p)return;
    if(p.data.meta?.kind==='contour'||p.data.meta?.kind==='contour-label'){
      const a=data.aircraft.find(a=>a.id===p.data.meta.id);
      const level=p.data.meta.kind==='contour'?p.data.meta.level:p.customdata;
      const position=a&&projectedContourPoint(event,a,level);
      if(position)return inspectContour(a,level,position);
    }
    if(Array.isArray(p.customdata)&&p.customdata.length===3){
      const [id,kind,index]=p.customdata,a=data.aircraft.find(a=>a.id===id);if(a)return inspect(a,(kind==='root'?a.sustained:a.points)[index]);
    }
    const candidates=displayed.filter(a=>!p.data.meta||a.id===(p.data.meta.id||p.data.meta));let best=null;
    for(const a of candidates)for(const point of a.points){
      if(!point.valid&&!showRejected)continue;
      const d=((point.speed_kmh-p.x)/(data.settings.speed_max_kmh-data.settings.speed_min_kmh))**2+((point.turn_dps-p.y)/ymax)**2;
      if(!best||d<best.d)best={a,point,d};
    }
    if(best)inspect(best.a,best.point);
  });
}
async function inspect(a,p){
  const sequence=state.inspection=(state.inspection||0)+1;
  if(!p.component_forces){
    try {p=await api(`/api/jobs/${state.dataJob}/point/${encodeURIComponent(a.id)}/${p.detail_index}`);}
    catch(e){error(e.message);return;}
  }
  if(sequence!==state.inspection)return;
  $('point-title').textContent=`${a.name} · ${fmt(p.speed_kmh,0)} km/h · ${fmt(p.load_g,2)} g`;
  $('point-status').innerHTML=`<span class="point-pill ${p.valid?'':'rejected'}">${p.valid?'Converged & within bounds':'Rejected'}</span>`;
  const dl=rows=>'<dl>'+rows.map(([k,v])=>`<dt>${k}</dt><dd>${v}</dd>`).join('')+'</dl>';
  const forces=Object.entries(p.component_forces).filter(([k])=>!['parasite','chute'].includes(k));
  const names={left_wing:'Left wing',right_wing:'Right wing',left_hstab:(a.aircraft_id||a.id)==='saab_jas39c'?'Left canard':'Left tail',right_hstab:(a.aircraft_id||a.id)==='saab_jas39c'?'Right canard':'Right tail',vstab:'Vertical tail',fuselage:'Fuselage'};
  $('point-content').innerHTML=`<div class="point-grid"><div><h3>FLIGHT & ENERGY</h3>${dl([
    ['Native-step Ps',fmt(p.ps_mps,2)+' m/s'],['Continuous Ps',fmt(p.ps_continuous_mps,2)+' m/s'],['Turn rate',fmt(p.turn_dps,3)+' °/s'],['Turn radius',turnRadius(p.speed_kmh,p.turn_dps)],
    ['Angle of attack',fmt(p.alpha_deg,3)+'°'],['Bank',fmt(p.bank_deg,3)+'°'],['Longitudinal IAS',fmt(p.ias_kmh,0)+' km/h'],['Mach',fmt(p.mach,3)],
    ['Engine force',fmt(p.engine_force_n[0]/1000,2)+' kN'],
    ...(p.propulsion?[
      ['Engine RPM',p.propulsion.engine_rpm.map(v=>fmt(v,0)).join(' / ')],
      ['Propeller pitch',p.propulsion.propeller_pitch_deg.map(v=>fmt(v,2)+'°').join(' / ')],
      ['Propeller control',p.propulsion.controls.automatic.map((auto,i)=>auto?'Automatic':fmt(p.propulsion.controls.commands[i]/255*100,1)+'% manual').join(' / ')],
      ['Compressor stage',(p.propulsion.compressor_stages??p.propulsion.controls.gears).map(v=>v+1).join(' / ')],
      ['Radiators','Closed'],['Landing gear',p.gear_percent===100?'Fixed · deployed':'Retracted']]:[]),
    ...(state.meta.aircraft[a.aircraft_id||a.id]?.has_flaps?[['Flaps',fmt(p.flaps_percent??0,1)+'%']]:[]),
    ...(p.sweep_percent==null?[]:[['Fixed wing sweep',fmt(p.sweep_percent,0)+'%'],['Available sweep',p.sweep_available_percent.map(x=>fmt(x,1)).join('–')+'%']])])}</div><div><h3>LIMITS & CONVERGENCE</h3>${dl([
    ['Control authority margin',fmt(p.authority_margin*100,2)+'%'],
    ...(instructorEvidence(a)?[['Instructor evidence',escapeText(instructorEvidence(a))]]:[]),
    ...(p.prolonged_pull?[['Boundary definition','Stationary full-pull reference · selected power'],['Full-pull check',escapeText(p.prolonged_pull.status)],['Full capability',escapeText(p.prolonged_pull.capability_status||'Not certified by the stationary check')],['Native overload timer',fmt(p.prolonged_pull.overload_timer,4)]]:[]),
    ...(p.instructor_enabled&&p.instructor?.history_independent?[['Instructor','Steady AoA schedule'],['Effective upper wing-AoA limit',fmt(p.instructor.effective_angle_limits_deg[1],3)+'°'],['Adjusted wing AoA',p.instructor.adjusted_wing_angles_deg.map(v=>fmt(v,3)+'°').join(' / ')],['Instructor limiting constraint',escapeText(p.instructor.limiting)]]:[]),
    ...(p.instructor_enabled&&!p.instructor?.history_independent?[['Instructor',p.instructor?.delivered_pitch!==undefined?'Experimental recovered pitch limiter':escapeText(p.instructor?.model??'Experimental Instructor')],...(p.instructor?.delivered_pitch!==undefined?[['Required elevator',fmt(p.instructor.required_pitch*100,2)+'%'],[p.instructor.required_pilot?'Delivered elevator':'Full-pull available elevator',fmt(p.instructor.delivered_pitch*100,2)+'%']]:[]),['Instructor limiting constraint',escapeText(p.instructor?.limiting??'Not evaluated: aircraft balance or physical limit rejected this point')],...(p.instructor?.angle_limits_deg?[['Instructor upper wing-AoA target',fmt(p.instructor.angle_limits_deg[1],3)+'°'],['Adjusted wing AoA',p.instructor.adjusted_wing_angles_deg?.map(v=>fmt(v,3)+'°').join(' / ')??'—']]:[])]:[]),
    ['Force residual',p.force_error_g.toExponential(2)+' g'],['Angular residual',p.angular_error_rad_s2.toExponential(2)+' rad/s²'],
    ['History residual',p.history_error.toExponential(2)],['Stall margin',fmt(p.stall_margin_deg,2)+'°'],
    ...(p.propulsion?[
      ['Propulsion settling',p.propulsion.period_frames?`${p.propulsion.period_frames}-frame cycle`:p.propulsion.stationarity?`${p.propulsion.sample_frames}-frame stable output cycle`:'Unresolved'],
      ['Mean trim',p.propulsion.phase_check?.checked?'Complete aircraft outputs averaged':'Not certified'],
      ['Engine management','Automatic']]:[]),
    ['Solved sideslip',fmt(p.sideslip_deg??0,4)+'°'],
    ['Wing load / limit',p.wing_load_ratios.map(x=>fmt(x*100,1)).join(' / ')+'%'],['Flight mode',flightModeLabel(resultConditions(a))],['Propeller torque & gyro',resultConditions(a).torque_gyro?'On':'Off'],['Engine state',escapeText(a.engine.policy)]])}</div><div><h3>COMPONENT FORCES · kN</h3><table><thead><tr><th>Component</th><th>Forward</th><th>Up</th><th>Right</th></tr></thead><tbody>${forces.map(([k,v])=>`<tr><td>${names[k]||escapeText(k)}</td>${v.map(x=>`<td>${fmt(x/1000,2)}</td>`).join('')}</tr>`).join('')}<tr><td>Total, incl. engine</td>${p.force_n.map(x=>`<td>${fmt(x/1000,2)}</td>`).join('')}</tr></tbody></table></div></div>${p.reasons.length?`<div class="point-note">Excluded: ${escapeText(p.reasons.join('; '))}.</div>`:''}`;
}
async function poll(id){
  if(state.activeJob!==id)return;
  try{
    const job=await api(`/api/jobs/${id}`);const p=job.progress||{};
    if(job.status==='complete'){
      await loadData(id,false);setRunning(false);state.activeJob=null;return;
    }
    if(job.status==='error')throw new Error(job.error||'Calculation failed');
    if(job.status==='cancelled'){setRunning(false);state.activeJob=null;$('footer-status').textContent=state.data?.preview?'Calculation cancelled · incomplete preview retained; exports unavailable.':'Calculation cancelled. Previous results retained.';return;}
    if(job.preview_revision&&state.previewToken!==`${id}:${job.preview_revision}`){
      await loadData(id,false,true);state.previewToken=`${id}:${job.preview_revision}`;
    }
    const name=state.meta.aircraft[p.aircraft]?.name||'';
    $('progress-label').textContent=job.status==='exporting'?'Preparing diagram and result data…':
      p.phase?`${p.phase}${name?' · '+name:''}`:name?`Solving ${name} · ${fmt(p.speed_kmh,0)} km/h · ${fmt(p.load_g,2)} g`:'Queued for calculation…';
    const percent=p.total?Math.min(99,100*p.done/p.total):0;
    $('progress-fill').style.width=percent+'%';$('progress-count').textContent=p.total?`${fmt(p.done,Number.isInteger(p.done)?0:1)} / ${p.total} · ${fmt(p.elapsed_s||0,0)} s`:'';
    setTimeout(()=>poll(id),900);
  }catch(e){setRunning(false);state.activeJob=null;error(e.message);}
}
$('scenario').addEventListener('submit',async e=>{
  e.preventDefault();error('');if(state.running)return;
  if(state.meta.public_static)return error('Calculations require the local application. The public site currently provides the interface and aircraft catalog only.');
  if(!state.entries.length)return error('Add at least one aircraft.');
  if(+$('speed-min').value>=+$('speed-max').value)return error('Maximum speed must exceed minimum speed.');
  try{setRunning(true);$('progress-label').textContent='Preparing calculation…';$('progress-count').textContent='';$('progress-fill').style.width='0%';
    const job=await api('/api/jobs',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(readConfig())});state.activeJob=job.id;poll(job.id);
  }catch(e){setRunning(false);error(e.message);}
});
function formChanged(e){
  if(e.target.id==='aircraft-search')return;
  $('apply-settings-status').textContent='';
  saveCondition();
  syncLabels();
}
$('scenario').addEventListener('input',formChanged);$('scenario').addEventListener('change',formChanged);
$('reset').addEventListener('click',()=>{populate(state.meta.defaults);syncLabels();error('');});
$('cancel').addEventListener('click',async()=>{if(state.activeJob){$('cancel').disabled=true;try{await api(`/api/jobs/${state.activeJob}/cancel`,{method:'POST'});}catch(e){error(e.message);}finally{$('cancel').disabled=false;}}});
$('chart-tabs').addEventListener('click',event=>{
  const button=event.target.closest('[data-view]');if(!button)return;
  state.view=button.dataset.view;document.querySelectorAll('[data-view]').forEach(b=>b.classList.toggle('active',b===button));renderChart();
});
$('contour-levels').addEventListener('click',event=>{
  const button=event.target.closest('[data-contour-remove]');if(!button)return;
  state.contourLevels=state.contourLevels.filter(level=>level!==Number(button.dataset.contourRemove));
  renderContourControls();syncLabels();refreshContours();
});
$('contour-add').addEventListener('click',()=>{
  const input=$('contour-input'),level=Number(input.value);
  if(!input.value.trim()||!Number.isFinite(level)||Math.abs(level)>2000||state.contourLevels.includes(level)||state.contourLevels.length>=16){
    $('contour-status').textContent='Enter a unique level from -2000 to 2000 m/s (up to 16).';return;
  }
  state.contourLevels.push(level);state.contourLevels.sort((a,b)=>b-a);input.value='';
  renderContourControls();syncLabels();refreshContours();
});
$('contour-input').addEventListener('keydown',event=>{if(event.key==='Enter'){event.preventDefault();$('contour-add').click();}});
for(const id of ['show-samples','show-rejected'])$(id).addEventListener('change',renderChart);
$('show-heatmap').addEventListener('change',()=>{syncLabels();updateContourAvailability();renderChart();});
$('reset-zoom').addEventListener('click',()=>{if(state.data)Plotly.relayout('chart',{'xaxis.range':[state.data.settings.speed_min_kmh,state.data.settings.speed_max_kmh],'yaxis.range':[0,envelopeTop(state.data.aircraft.filter(a=>state.view==='compare'||a.id===state.view))]});});
document.querySelectorAll('[data-export]').forEach(a=>a.addEventListener('click',e=>{if(a.getAttribute('aria-disabled')==='true')e.preventDefault();}));
(async()=>{try{
  [state.meta]=await Promise.all([api('/api/meta'),document.fonts.load('14px WTSymbols','▄')]);
  state.meta.defaults={...state.meta.defaults,aircraft:[]};
  populateAircraft();populate(state.meta.defaults);
  if(state.meta.public_static){
    $('hosting-notice').hidden=false;
    $('runtime-label').innerHTML='<i></i> PUBLIC PREVIEW';
    $('runtime-footer').textContent='PUBLIC PREVIEW';
    $('footer-status').textContent='Calculations currently run in the local application.';
    $('calculate').disabled=true;$('calculate').textContent='Calculations require local app';
  }else if(remoteSite){
    $('runtime-label').innerHTML='<i></i> LIVE TEST';
    $('runtime-footer').textContent='LIVE TEST';
    $('footer-status').textContent='Calculations run on the server.';
  }
}catch(e){error('Could not load the plotter: '+e.message);}})();
