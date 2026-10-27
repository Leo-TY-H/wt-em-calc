(() => {
  'use strict';
  const el = id => document.getElementById(id);
  const root = el('missile-workspace');
  const initial = {missile:'us_aim9l_sidewinder',
    launch:{position:[0,5000,0],velocity:[300,0,0],angles:[0,0,0]},
    target:{position:[4000,5000,0],velocity:[200,0,0],angles:[0,0,0]}};
  let meta=null, loading=null, result=null, playing=false, lastFrame=0, playbackTime=0, downloadUrl=null, flightRevision=0;
  const activeJobs=new Set();
  let selectedMissiles=[], flights=[], resultColors=[], timeline=[], comparison=null, batchRunning=false, cancelRequested=false, focusIndex=0;
  const flightColors=['#38c9d7','#b79aff','#91d477','#ee8eb6','#f0d367','#79a7fa','#cfb296','#f08070'];
  const missileName=id=>meta?.missiles.find(m=>m.id===id)?.name||id;
  const missileLabel=id=>`${missileName(id)} [${id}]`;
  const REMOVE_ICON='<svg viewBox="0 0 16 16" width="14" height="14" aria-hidden="true" fill="none" stroke="currentColor" stroke-width="1.4" stroke-linejoin="round"><path d="M2.5 4.5h11M6 4.5v-2h4v2M4 4.5l.7 9h6.6l.7-9M6.7 7v4.5M9.3 7v4.5"/></svg>';
  const escapeHtml=text=>String(text).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
  // The launcher sits above the origin; the target is placed from it in cylindrical coordinates (Leo, 2026-10-26):
  // azimuth (0° = +X, 90° = +Z, as headings), horizontal distance in km and altitude.
  // Rounded below a micrometre so replayed shots read cleanly (not 158.29999999999998°).
  const cylindrical = offset => [+(Math.atan2(offset[2], offset[0]) * 180 / Math.PI).toFixed(10) + 0, +(Math.hypot(offset[0], offset[2]) / 1000).toFixed(9)];
  // Position fields in the same style as the motion fields: label above, unit inside the input.
  const unitField = (id, label, aria, unit, value, min, max) => `<label class="field">${label}<div class="input-unit"><input id="${id}" aria-label="${aria}" type="number" step="any" required value="${value}" min="${min}" max="${max}"><span>${unit}</span></div></label>`;
  const positionFields = role => {
    if(role==='launch')return `<div class="missile-motion">${unitField('missile-launch-position-1','Altitude','Launcher altitude','m',initial.launch.position[1],0.001,30000)}</div>`;
    const [azimuth,distance]=cylindrical(initial.target.position.map((v,i)=>v-initial.launch.position[i]));
    return `<div class="missile-motion">${unitField('missile-target-position-1','Altitude','Target altitude','m',initial.target.position[1],0.001,30000)}${unitField('missile-target-azimuth','Azimuth','Target azimuth','°',azimuth,-360,360)}${unitField('missile-target-distance','Horizontal distance','Target horizontal distance','km',distance,0,200)}</div>`;
  };
  const motion = role => `<div class="missile-motion">${[['speed','Speed','km/h',0,12470.76582],['course','Heading','°',-360,360],['climb',role==='launch'?'Launch angle':'Climb angle','°',-90,90]].map(([key,label,unit,min,max])=>`<label class="field">${label}<div class="input-unit"><input id="missile-${role}-${key}" aria-label="${role==='launch'?'Launcher':'Target'} ${label.toLowerCase()}" type="number" min="${min}" max="${max}" step="any" required><span>${unit}</span></div></label>`).join('')}</div>`;
  const stateFields = (role,title) => `<fieldset class="missile-step missile-${role}"><legend>${title}</legend>${positionFields(role)}${motion(role)}</fieldset>`;
  root.innerHTML = `<aside class="missile-sidebar"><form id="missile-form">
    <div class="section-heading"><span></span><button id="missile-reset" class="text-button" type="button">Reset all</button></div>
    <div class="missile-presets" role="group" aria-label="Starter scenarios">
      <button type="button" data-preset="chase" aria-pressed="true"><span aria-hidden="true">⇉</span>Tail chase</button>
      <button type="button" data-preset="head-on" aria-pressed="false"><span aria-hidden="true">⇄</span>Head-on</button>
      <button type="button" data-preset="crossing" aria-pressed="false"><span aria-hidden="true">↱</span>Crossing</button>
    </div><p id="missile-preset-note" class="hint"></p>
    <fieldset id="missile-picker" class="missile-step"><legend class="visually-hidden">Add missiles</legend>
      <input id="missile-search" type="search" placeholder="Search missiles…" autocomplete="off" aria-label="Search missiles" aria-controls="missile-list">
      <div id="missile-list" class="aircraft-list" aria-label="Available missiles"></div>
      <p id="missile-search-empty" class="hint" hidden>No missiles match your search.</p>
      <p id="missile-catalog-note" class="hint" role="status">Loading missiles…</p>
      <div id="missile-selected" class="missile-selected" aria-label="Selected missiles"></div>
      <button id="missile-retry" class="text-button" type="button" hidden>Retry loading missiles</button>
    </fieldset>
    ${stateFields('launch','Launcher aircraft')}
    ${stateFields('target','Target aircraft')}
    <div class="missile-run-panel">
    <p id="missile-ready" class="hint"></p>
    <button id="missile-run" class="primary-button" type="submit" disabled>Calculate</button>
    <button id="missile-cancel" class="secondary-button" type="button" hidden>Cancel simulation</button>
    <div id="missile-status" role="status" aria-live="polite"></div></div>
  </form></aside>
  <div class="missile-main">
    <div id="missile-error" class="error" role="alert" hidden></div>
    <div id="missile-stale" class="notice" hidden>Inputs changed. Run again to update the trajectories.</div>
    <section class="chart-card missile-plot-card"><div class="chart-toolbar"><div id="missile-legend" class="missile-legend"></div>
      <div class="mode-switch missile-views" role="radiogroup" aria-label="Plot view"><label><input type="radio" name="missile-plot-view" value="setup" checked><span>Setup</span></label><label><input type="radio" name="missile-plot-view" value="3d" disabled><span>3D</span></label><label><input type="radio" name="missile-plot-view" value="xz" disabled><span>Top</span></label><label><input type="radio" name="missile-plot-view" value="xy" disabled><span>Side</span></label><label><input type="radio" name="missile-plot-view" value="zy" disabled><span>Front</span></label></div>
      <select id="missile-view" hidden aria-hidden="true"><option value="3d">3D</option><option value="xz">Top</option><option value="xy">Side</option><option value="zy">Front</option></select></div>
    <div class="missile-plot">
      <div id="missile-setup-pane" class="missile-geometry-wrap"><div id="missile-geometry" role="img" aria-label="Starting positions and flight directions"></div><svg id="missile-axes" class="missile-axes" viewBox="0 0 72 72" aria-hidden="true"></svg><p id="missile-preview-note" class="missile-plot-message"></p></div>
      <div id="missile-chart" aria-label="Interactive 3D missile and target trajectories" hidden></div>
    </div>
    <div class="missile-playback"><button id="missile-play" type="button" disabled>Play</button><input id="missile-time" type="range" aria-label="Trajectory time" min="0" max="0" value="0" step="1" disabled><output id="missile-time-label">0.00 s</output><select id="missile-play-speed" aria-label="Playback speed"><option value="1">1×</option><option value="4" selected>4×</option><option value="10">10×</option></select><p id="missile-frame" class="missile-frame-readout"></p></div>
    <p id="missile-slice-note" class="hint missile-plot-note" hidden>2D slices project the full trajectories onto the selected plane.</p></section>
    <details id="missile-flight-data" class="missile-flight-data" open hidden><summary>Telemetry <span id="missile-flight-name"></span></summary>
      <p class="hint">Click a plotted point to inspect that missile and time. In Drag &amp; lift, solid lines show drag and dashed lines show lift.</p>
      <div class="missile-data-grid">
        <section class="chart-card"><label class="field missile-plot-choice" for="missile-propulsion-metric">Propulsion<select id="missile-propulsion-metric"><option value="thrust">Thrust · kN</option><option value="mass">Missile mass · kg</option><option value="propellant">Propellant consumed · kg</option></select></label><div id="missile-propulsion-legend" class="missile-plot-legend" aria-label="Missile colors"></div><div id="missile-propulsion-chart" class="missile-data-chart" aria-label="Propulsion over flight time"></div></section>
        <section class="chart-card"><label class="field missile-plot-choice" for="missile-aero-metric">Aerodynamics<select id="missile-aero-metric"><option value="forces">Drag &amp; lift · kN</option><option value="cd">Drag coefficient · Cd</option><option value="aoa">Angle of attack · degrees</option><option value="pressure">Dynamic pressure · kPa</option><option value="acceleration">Net acceleration · m/s²</option></select></label><div id="missile-aero-legend" class="missile-plot-legend" aria-label="Missile colors"></div><div id="missile-aero-chart" class="missile-data-chart" aria-label="Aerodynamics over flight time"></div></section>
        <section class="chart-card"><label class="field missile-plot-choice" for="missile-control-metric">Control<select id="missile-control-metric"><option value="load">Manoeuvre load · g</option><option value="look">Seeker look angle · degrees</option><option value="los">Line-of-sight rate · °/s</option><option value="turn">Turn rate · °/s</option></select></label><div id="missile-control-legend" class="missile-plot-legend" aria-label="Missile colors"></div><div id="missile-control-chart" class="missile-data-chart" aria-label="Control over flight time"></div></section>
      </div><p class="hint">Control values come from the saved motion: manoeuvre load is the acceleration across the flight path without gravity, the look angle is between the missile's nose and the target. Forces come from the final integration substep before each saved position. Lift and drag are force magnitudes; angle of attack is unsigned local-flow incidence. Net acceleration includes gravity. Missing values at release or the exact proximity event are left blank.</p>
    </details>
    <div class="exports"><span>EXPORT</span><a id="missile-download" aria-disabled="true">Flight data JSON</a></div>
    <details class="missile-details" hidden><summary>Model and current limitations</summary><p>Recovered profile baseline: War Thunder 2.59.0.34. Seekers begin with warm-up complete, respecting designation and angle limits. Radar missiles retain their lock-before-launch or lock-after-launch behavior. Ideal radar support moves from the launch position at the entered launch velocity; recovered inertial guidance remains active where configured. Target geometry is a point, so a proximity event does not assert aircraft damage. Full engagements and native collision timing are still being validated.</p><ul id="missile-model-details"></ul></details>
  </div>`;
  const apiBase=String((window.EM_CONFIG||{}).apiBase||'').replace(/\/$/,'');
  async function request(path, options={}) {
    const response=await fetch(apiBase?apiBase+path:'.'+path,options);
    const body=await response.json();
    if(!response.ok) throw new Error(body.error||`Request failed (${response.status})`);
    return body;
  }
  const showError = text => {el('missile-error').textContent=text;el('missile-error').hidden=!text;};
  const pause = () => {playing=false;el('missile-play').textContent='Play';};
  const toolTabs=[['em','em-tab','em-workspace'],['missile','missile-tab','missile-workspace'],['envelope','envelope-tab','envelope-workspace']].filter(([,tab,panel])=>el(tab)&&el(panel));
  function selectTab(name) {
    if(name===true||name===false)name=name?'missile':'em';
    for(const [key,tab,panel] of toolTabs) {
      const selected=key===name;el(panel).hidden=!selected;
      if(selected)el(tab).setAttribute('aria-current','page');else el(tab).removeAttribute('aria-current');
    }
    // The header group holding the open calculator is marked current (app.js runs its menus).
    for(const group of document.querySelectorAll('.nav-group'))group.classList.toggle('current',!!group.querySelector('[aria-current=page]'));
    if(name==='missile') {
      if(!meta) load();
      if(result) requestAnimationFrame(()=>{Plotly.Plots.resize(el('missile-chart'));resizeDataPlots();});
    } else {pause(); if(name==='em'&&el('chart').data) requestAnimationFrame(()=>Plotly.Plots.resize(el('chart')));}
    document.dispatchEvent(new CustomEvent('tool-tab-change',{detail:name}));
  }
  for(const [key,tab] of toolTabs) el(tab).addEventListener('click',()=>selectTab(key));
  async function load() {
    if(loading) return loading;
    loading=(async()=>{try{
      showError('');el('missile-retry').hidden=true;meta=await request('/api/missiles/meta');
      const unique=new Map();
      const rank=m=>m.id==='us_aim4g_falcon'?-1:m.id.length;
      for(const m of meta.missiles.filter(m=>!m.id.toLowerCase().split('_').includes('default')).sort((a,b)=>rank(a)-rank(b)||a.id.localeCompare(b.id))) {
        const name=m.name.trim().toLowerCase();if(!unique.has(name))unique.set(name,m);
      }
      meta.missiles=[...unique.values()];
      meta.missiles.sort((a,b)=>a.name.localeCompare(b.name,undefined,{numeric:true})||a.id.localeCompare(b.id));
      populateMissiles();renderSelected();busy(false);
    }catch(error){meta=null;busy(false);el('missile-retry').hidden=false;showError('Could not load missile configurations. Check that the simulator server is running, then retry. '+error.message);}
    finally{loading=null;}})();return loading;
  }
  // The target's position (m) from the cylindrical fields; the launcher is above the origin.
  function targetPosition() {
    const azimuth=radians(Number(el('missile-target-azimuth').value)),distance=Number(el('missile-target-distance').value)*1000;
    return [distance*Math.cos(azimuth),Number(el('missile-target-position-1').value),distance*Math.sin(azimuth)];
  }
  // Fills the cylindrical fields from a position relative to the launcher's ground point (x, altitude, z in m).
  function setTargetPosition(offset) {
    const [azimuth,distance]=cylindrical(offset);
    el('missile-target-azimuth').value=String(azimuth);el('missile-target-distance').value=String(distance);el('missile-target-position-1').value=String(offset[1]);
  }
  function inputs() {
    const config={missiles:[...selectedMissiles]};
    for(const role of ['launch','target']) {
      const [speed,heading,climb]=['speed','course','climb'].map(key=>Number(el(`missile-${role}-${key}`).value));
      const horizontal=speed/3.6*Math.cos(radians(climb));
      config[role==='launch'?'launcher':role]={
        position:role==='launch'?[0,Number(el('missile-launch-position-1').value),0]:targetPosition(),
        velocity:[horizontal*Math.cos(radians(heading)),speed/3.6*Math.sin(radians(climb)),horizontal*Math.sin(radians(heading))],
        angles:[heading,climb,0]};
    }
    return config;
  }
  const normalizeMissileSearch=value=>value.toLowerCase().normalize('NFKD').replace(/[^a-z0-9]/g,'');
  function populateMissiles() {
    el('missile-list').replaceChildren(...meta.missiles.map(item=>{
      const row=document.createElement('div');row.className='aircraft-option';
      const family=item.family==='optical'?'Optical / infrared':'Radar';
      row.dataset.search=item.id+' '+item.name+' '+family;
      const label=document.createElement('span'),name=document.createElement('strong'),detail=document.createElement('small'),code=document.createElement('code');
      name.textContent=item.name;code.textContent=item.id;detail.append(code);label.append(name,detail);label.title=missileLabel(item.id);
      const button=document.createElement('button');button.type='button';button.dataset.addMissile=item.id;
      button.textContent='Add';button.setAttribute('aria-label','Add '+missileLabel(item.id));
      row.append(label,button);return row;
    }));
    el('missile-catalog-note').hidden=true;
    filterMissiles();
  }
  function filterMissiles() {
    const query=normalizeMissileSearch(el('missile-search').value);let visible=0;
    for(const row of el('missile-list').children) {
      row.hidden=!normalizeMissileSearch(row.dataset.search).includes(query);
      if(!row.hidden)visible++;
    }
    el('missile-search-empty').hidden=!meta||visible>0;
    updateAddButton();
  }
  function updateAddButton() {
    for(const button of el('missile-list').querySelectorAll('[data-add-missile]')) {
      const added=selectedMissiles.includes(button.dataset.addMissile);
      button.disabled=batchRunning||added||selectedMissiles.length>=8;
      button.textContent=added?'Added':'Add';
    }
  }
  function renderSelected() {
    el('missile-selected').replaceChildren(...selectedMissiles.map((id,i)=>{
      const card=document.createElement('article');card.className='aircraft-entry';card.style.setProperty('--entry-color',flightColors[i]);
      card.innerHTML=`<div class="entry-heading"><span class="entry-number">${i+1}</span><strong>${escapeHtml(missileName(id))}</strong><code>${escapeHtml(id)}</code></div><div class="entry-actions"><button type="button" class="entry-icon" data-action="remove" title="Remove">${REMOVE_ICON}</button></div>`;
      const remove=card.querySelector('button');remove.disabled=batchRunning;remove.setAttribute('aria-label','Remove '+missileLabel(id));
      remove.addEventListener('click',()=>{selectedMissiles.splice(i,1);renderSelected();filterMissiles();changed();busy(false);});
      return card;
    }));
    updateAddButton();
  }
  el('missile-list').addEventListener('click',event=>{
    const button=event.target.closest('[data-add-missile]');
    if(!button||button.disabled)return;
    selectedMissiles.push(button.dataset.addMissile);renderSelected();filterMissiles();changed();busy(false);
  });
  const radians = angle => angle*Math.PI/180;
  const degrees = angle => angle*180/Math.PI;
  // A stationary aircraft (a surface-to-air launcher) keeps the heading and climb of its angles.
  function syncMotion(role,velocity=initial[role].velocity,angles=null) {
    const [x,y,z]=velocity.map(v=>v*3.6);
    const speed=Math.hypot(x,y,z);
    for(const [key,value] of Object.entries({speed,course:speed?degrees(Math.atan2(z,x)):angles?.[0]??0,climb:speed?degrees(Math.atan2(y,Math.hypot(x,z))):angles?.[1]??0})) {
      el(`missile-${role}-${key}`).value=Number(value.toFixed(10));
    }
  }
  function changed() {
    if(result)el('missile-stale').hidden=false;
    // One plot (Leo, 2026-10-27): a changed setup is shown as its starting geometry until the next run.
    if(plotView!=='setup')showView('setup');else updatePreview();
  }
  // The plot shows the setup's starting geometry or the calculated flight in one of four views.
  let plotView='setup',flightView='3d';
  function showPanes(view) {
    plotView=view;const setup=view==='setup';
    for(const input of root.querySelectorAll('input[name=missile-plot-view]'))input.checked=input.value===view;
    el('missile-setup-pane').hidden=!setup;el('missile-chart').hidden=setup;
    el('missile-slice-note').hidden=setup||view==='3d';
  }
  function showView(view) {
    showPanes(view);
    if(view==='setup'){updatePreview();return;}
    flightView=view;
    if(el('missile-view').value!==view){el('missile-view').value=view;el('missile-view').dispatchEvent(new Event('change'));}
    else if(el('missile-chart').data)Plotly.Plots.resize(el('missile-chart'));
  }
  for(const input of root.querySelectorAll('input[name=missile-plot-view]'))input.addEventListener('change',()=>{if(input.checked)showView(input.value);});
  function updatePreview() {
    const targetDistance=el('missile-target-distance');targetDistance.setCustomValidity('');
    const numericFields=[...el('missile-form').querySelectorAll('input[type=number]')];
    for(const role of ['launch','target'])el(`missile-${role}-speed`).setCustomValidity('');
    if(numericFields.every(field=>field.validity.valid)) {
      const config=inputs();
      for(const role of ['launch','target'])if(config[role==='launch'?'launcher':role].velocity.some(v=>Math.abs(v)>2000))el(`missile-${role}-speed`).setCustomValidity('Reduce speed: each world-axis velocity must be at most 7,200 km/h.');
      if(Math.hypot(...config.target.position.map((v,i)=>v-config.launcher.position[i]))<10)targetDistance.setCustomValidity('Place the target at least 10 m from the launcher aircraft.');
    }
    const invalid=numericFields.find(field=>!field.validity.valid),valid=!invalid;
    el('missile-ready').textContent=valid?'':`${invalid.getAttribute('aria-label')||'Setup'}: ${invalid.validationMessage}`;
    if(!valid) {
      if(el('missile-geometry').data)Plotly.purge(el('missile-geometry'));
      el('missile-axes').innerHTML='';
      el('missile-geometry').setAttribute('aria-label','Preview unavailable. Check the setup values.');
      el('missile-preview-note').textContent='Complete the numeric fields within their allowed ranges to see the preview.';
      return;
    }
    const {launcher:launch,target}=inputs(),offset=target.position.map((v,i)=>v-launch.position[i]);
    const distance=Math.hypot(...offset),altitude=offset[1];
    const format=value=>value.toLocaleString(undefined,{maximumFractionDigits:1});
    drawPreview(launch,target);
    el('missile-geometry').setAttribute('aria-label',`3D view of aircraft at release. Initial separation ${(distance/1000).toFixed(2)} kilometres. Target ${altitude===0?'at the same altitude':`${format(Math.abs(altitude))} meters ${altitude>0?'above':'below'} the launcher aircraft`}.`);
    el('missile-preview-note').textContent='';
  }
  // Starting geometry in 3D (Leo, 2026-10-26): the launcher, the target placed from it by azimuth, horizontal distance
  // and altitude, and both headings as arrows. World X, Z and altitude are the scene's x, y and z, as in the flight chart.
  // Labels (Leo, 2026-10-27: no overlapping text) go on screen where no line or other label is: each takes the side of
  // its point, of eight, whose box covers the least, and they are placed again when the view is turned.
  function drawPreview(launch,target) {
    const node=el('missile-geometry');if(!node.clientWidth)return;   // hidden tab: drawn once it is shown (ResizeObserver)
    const light='#dce5f0',grey='#8d9aae',cyan='#38c9d7',orange='#ffa66b';
    const scene=p=>[p[0],p[2],p[1]],L=scene(launch.position),T=scene(target.position),G=[T[0],T[1],L[2]];
    const horizontal=Math.hypot(T[0]-L[0],T[1]-L[1]),rise=T[2]-L[2],size=Math.max(horizontal,Math.abs(rise),300);
    const azimuth=Math.atan2(T[1]-L[1],T[0]-L[0]),deg=v=>`${+v.toFixed(1)}°`,turned=Math.abs(azimuth)>1e-3;
    const along=(a,b,t)=>a.map((v,i)=>v+(b[i]-v)*t),points=[L,T],traces=[],segments=[],labels=[];
    const W=node.clientWidth,H=node.clientHeight||260,narrow=W<560;
    const line=(ps,color,width,dash='solid')=>{points.push(...ps);for(let i=1;i<ps.length;i++)segments.push([ps[i-1],ps[i]]);
      traces.push({type:'scatter3d',mode:'lines',x:ps.map(p=>p[0]),y:ps.map(p=>p[1]),z:ps.map(p=>p[2]),line:{color,width,dash},hoverinfo:'skip',showlegend:false});};
    const label=(p,text,color,lines=[text])=>{points.push(p);labels.push({p,text,color,lines});};
    // Azimuth: the arc from the +X reference (dotted) to the target's bearing, in the launcher's horizontal plane,
    // drawn beyond the heading arrows.
    const r=.3*size;
    if(horizontal>0&&turned) {
      line([L,[L[0]+r*1.25,L[1],L[2]]],grey,2,'dot');
      const steps=Math.max(1,Math.ceil(Math.abs(azimuth)*180/Math.PI/3));
      line(Array.from({length:steps+1},(_,i)=>{const a=azimuth*i/steps;return [L[0]+r*Math.cos(a),L[1]+r*Math.sin(a),L[2]];}),light,2);
      label([L[0]+r*Math.cos(azimuth/2),L[1]+r*Math.sin(azimuth/2),L[2]],deg(azimuth*180/Math.PI),light);
    }
    // Horizontal distance, then the altitude difference and the line of sight.
    if(horizontal>0){line([L,G],light,2.5,'dash');label(along(L,G,.6),`${(horizontal/1000).toFixed(2)} km`,light);}
    if(rise){line([G,T],grey,2,'dot');label(along(G,T,.5),`${rise>0?'+':'−'}${Math.abs(rise).toLocaleString()} m`,grey);
      // Total separation (Leo, 2026-10-27) along the line of sight; level aircraft show only the horizontal distance.
      line([L,T],grey,1.5,'dash');label(along(L,T,.45),`${(Math.hypot(horizontal,rise)/1000).toFixed(2)} km total`,grey);}
    // Each aircraft with its heading (and climb) as an arrow of one size for both.
    for(const [state,p,color,name,climbName] of [[launch,L,cyan,'Launcher','launch angle'],[target,T,orange,'Target','climb']]) {
      const v=scene(state.velocity),speed=Math.hypot(...v),[heading,climb]=state.angles;
      const detail=`HDG ${deg(heading)}${climb?` · ${climbName} ${deg(climb)}`:''}`;
      traces.push({type:'scatter3d',mode:'markers',x:[p[0]],y:[p[1]],z:[p[2]],marker:{size:5,color},showlegend:false,
        hovertemplate:`${name}<br>${Math.round(p[2]).toLocaleString()} m · ${Math.round(speed*3.6).toLocaleString()} km/h<br>${detail[0].toUpperCase()+detail.slice(1)}<extra></extra>`});
      if(narrow)label(p,name,color);else label(p,`${name}<br><span style="font-size:11px;color:${grey}">${detail}</span>`,color,[name,detail]);
      if(!speed)continue;
      const u=v.map(c=>c/speed),length=.2*size,head=.05*size,end=p.map((c,i)=>c+u[i]*length);
      line([p,end],color,3);points.push(end.map((c,i)=>c+u[i]*head));segments.push([end,end.map((c,i)=>c+u[i]*head)]);
      traces.push({type:'cone',x:[end[0]],y:[end[1]],z:[end[2]],u:[u[0]*head],v:[u[1]*head],w:[u[2]*head],anchor:'tail',sizemode:'raw',sizeref:.7,
        colorscale:[[0,color],[1,color]],showscale:false,hoverinfo:'skip',lighting:{ambient:1,diffuse:0,specular:0,roughness:1,fresnel:0}});
    }
    // Seen from behind the launcher's left, 35° above, so the target's bearing and the horizontal plane both read;
    // a view turned by hand is kept and fitted as it is.
    const view=azimuth-2*Math.PI/3,elevation=35*Math.PI/180,E=1.73;
    const eye={x:E*Math.cos(elevation)*Math.cos(view),y:E*Math.cos(elevation)*Math.sin(view),z:E*Math.sin(elevation)};
    const bounds=[0,1,2].map(i=>[Math.min(...points.map(p=>p[i])),Math.max(...points.map(p=>p[i]))]);
    // Labels may sit on any side: the scene keeps a margin of the widest one all round.
    const context=(drawPreview.canvas||=document.createElement('canvas')).getContext('2d');context.font=`12px ${getComputedStyle(document.body).fontFamily}`;
    for(const l of labels){l.width=Math.max(...l.lines.map(t=>context.measureText(t).width))+4;l.height=15*l.lines.length;}
    const widest=Math.max(...labels.map(l=>l.width));
    const mx=Math.min(.3,(widest+14)/W),my=Math.min(.25,34/H),domain={x:[mx,1-mx],y:[my,1-my]};
    const viewport=W*(domain.x[1]-domain.x[0])/(H*(domain.y[1]-domain.y[0]));
    const fitted=sceneFit(bounds,{chart:node,floor:.05,pad:1.15,eye:previewCamera?.eye||eye,points,viewport}),axis=i=>{const c=(bounds[i][0]+bounds[i][1])/2,span=fitted.spans[i];return {range:[c-span/2,c+span/2],visible:false};};
    // Plotly's orthographic scene shows two scene units over the domain's height.
    // The axes indicator's corner (bottom right) is kept free of labels.
    const frame={W,H,domain,scale:H*(domain.y[1]-domain.y[0])/2,reserved:[W-AXES_SIZE-8,W,H-AXES_SIZE-8,H]};
    const place=camera=>placeLabels(labels,segments,bounds,fitted,camera.eye,frame);
    Plotly.react(node,traces,{paper_bgcolor:'#000000',margin:{l:0,r:0,t:0,b:0},showlegend:false,
      hoverlabel:{bgcolor:'#0c0c0f',font:{color:'#e3edf7',size:12}},
      scene:{domain,xaxis:axis(0),yaxis:axis(1),zaxis:axis(2),aspectmode:'manual',aspectratio:fitted.aspectratio,camera:previewCamera||fitted.camera,annotations:place(previewCamera||fitted.camera)}},
      {displayModeBar:false,responsive:true,scrollZoom:false});
    drawAxes((previewCamera||fitted.camera).eye);
    node.removeAllListeners?.('plotly_relayout');node.removeAllListeners?.('plotly_relayouting');
    node.on('plotly_relayouting',event=>{if(event['scene.camera'])drawAxes(event['scene.camera'].eye);});
    let placing=false;
    node.on('plotly_relayout',event=>{
      if(placing||!event['scene.camera'])return;
      // The turned view goes into the layout with its labels, or relaying them out would turn it back.
      previewCamera=event['scene.camera'];placing=true;drawAxes(previewCamera.eye);
      Plotly.relayout(node,{'scene.camera':previewCamera,'scene.annotations':place(previewCamera)}).finally(()=>{placing=false;});
    });
  }
  // Axes indicator (Leo, 2026-10-27): world X, Y (altitude) and Z as seen from the preview's camera, turning with it.
  const AXES_SIZE=72;
  function drawAxes(eye) {
    const horizontal=Math.hypot(eye.x,eye.y)||1e-9,length=Math.hypot(horizontal,eye.z),c=AXES_SIZE/2,arm=24;
    const right=[-eye.y/horizontal,eye.x/horizontal,0],up=[-eye.x*eye.z/(horizontal*length),-eye.y*eye.z/(horizontal*length),horizontal/length];
    const toward=[eye.x/length,eye.y/length,eye.z/length],dot=(a,b)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
    // World X, Z and altitude are the scene's x, y and z; the axis nearest the viewer is drawn last.
    const axes=[['X',[1,0,0],'#ef7d7a'],['Y',[0,0,1],'#86d38a'],['Z',[0,1,0],'#76a9f5']].map(([name,v,color])=>({name,color,x:dot(v,right),y:dot(v,up),depth:dot(v,toward)}))
      .sort((a,b)=>a.depth-b.depth);
    // Letters just beyond the tips, pushed apart where two foreshortened axes point the same way.
    for(const a of axes){const n=Math.max(Math.hypot(a.x,a.y),.35);a.lx=c+(arm+9)*a.x/n;a.ly=c-(arm+9)*a.y/n;}
    for(let pass=0;pass<4;pass++)for(const a of axes)for(const b of axes){if(a===b)continue;const dx=a.lx-b.lx,dy=a.ly-b.ly,d=Math.hypot(dx,dy);
      if(d<11){const push=(11-d)/2,ux=d>1e-6?dx/d:1,uy=d>1e-6?dy/d:0;a.lx+=ux*push;a.ly+=uy*push;b.lx-=ux*push;b.ly-=uy*push;}}
    el('missile-axes').innerHTML=`<circle cx="${c}" cy="${c}" r="2" fill="#8d9aae"/>`+axes.map(a=>{
      const x=c+arm*a.x,y=c-arm*a.y,lx=a.lx,ly=a.ly;
      return `<line x1="${c}" y1="${c}" x2="${x.toFixed(1)}" y2="${y.toFixed(1)}" stroke="${a.color}" stroke-width="2" stroke-linecap="round"/>`+
        `<text x="${lx.toFixed(1)}" y="${ly.toFixed(1)}" fill="${a.color}" font-size="11" text-anchor="middle" dominant-baseline="central">${a.name}</text>`;
    }).join('');
  }
  // Annotations for the preview's labels. Each label's box goes on the side of its point (of eight, diagonals first)
  // that covers the fewest drawn line pixels and no label placed before it; the aircraft are labelled first.
  function placeLabels(labels,segments,bounds,fitted,eye,frame) {
    const keys=['x','y','z'],centre=bounds.map(([low,high])=>(low+high)/2),k=frame.scale;
    const horizontal=Math.hypot(eye.x,eye.y)||1e-9,length=Math.hypot(horizontal,eye.z);
    const right=[-eye.y/horizontal,eye.x/horizontal,0],up=[-eye.x*eye.z/(horizontal*length),-eye.y*eye.z/(horizontal*length),horizontal/length];
    // Pixels from the scene domain's centre, y up.
    const screen=p=>{const q=p.map((v,i)=>fitted.aspectratio[keys[i]]*(v-centre[i])/fitted.spans[i]);return [k*q.reduce((s,v,i)=>s+v*right[i],0),k*q.reduce((s,v,i)=>s+v*up[i],0)];};
    const samples=[];
    for(const [a,b] of segments){const A=screen(a),B=screen(b),n=Math.max(1,Math.ceil(Math.hypot(B[0]-A[0],B[1]-A[1])/3));
      for(let i=0;i<=n;i++)samples.push([A[0]+(B[0]-A[0])*i/n,A[1]+(B[1]-A[1])*i/n]);}
    const cx=(frame.domain.x[0]+frame.domain.x[1])/2*frame.W,cy=(frame.domain.y[0]+frame.domain.y[1])/2*frame.H;
    const limits=[-cx+2,frame.W-cx-2,-cy+2,frame.H-cy-2],r=frame.reserved;
    // The reserved corner (screen px, y down) as a box already taken, in these coordinates (y up).
    const placed=r?[[r[0]-cx,r[1]-cx,frame.H-r[3]-cy,frame.H-r[2]-cy]]:[];
    // Close to the point first; a second, farther ring when everything near is taken.
    const sides=[9,26].flatMap((gap,ring)=>[[1,1],[-1,1],[1,-1],[-1,-1],[1,0],[-1,0],[0,1],[0,-1]].map(([dx,dy])=>[dx,dy,gap,ring*4]));
    const order=labels.map((l,i)=>i).sort((a,b)=>(labels[b].lines.length>1||/Launcher|Target/.test(labels[b].text))-(labels[a].lines.length>1||/Launcher|Target/.test(labels[a].text)));
    const out=[];
    for(const i of order) {
      const {p,text,color,width:w,height:h}=labels[i],s=screen(p);let best=null;
      for(const [dx,dy,gap,far] of sides) {
        const x0=dx>0?gap:dx<0?-gap-w:-w/2,y0=dy>0?gap:dy<0?-gap-h:-h/2,box=[s[0]+x0,s[0]+x0+w,s[1]+y0,s[1]+y0+h];
        let cost=far;
        for(const [x,y] of samples)if(x>box[0]-2&&x<box[1]+2&&y>box[2]-2&&y<box[3]+2)cost+=1;
        for(const b of placed)cost+=400*Math.max(0,Math.min(box[1],b[1])-Math.max(box[0],b[0]))*Math.max(0,Math.min(box[3],b[3])-Math.max(box[2],b[2]))/(w*h);
        cost+=40*(Math.max(0,limits[0]-box[0])+Math.max(0,box[1]-limits[1])+Math.max(0,limits[2]-box[2])+Math.max(0,box[3]-limits[3]))/Math.min(w,h);
        if(!best||cost<best.cost-1e-6)best={cost,dx,dy,gap,box};
      }
      placed.push(best.box);const {dx,dy,gap}=best;
      out[i]={x:p[0],y:p[1],z:p[2],text,showarrow:false,font:{color,size:12},xshift:dx*gap,yshift:dy*gap,
        xanchor:dx>0?'left':dx<0?'right':'center',yanchor:dy>0?'bottom':dy<0?'top':'middle',align:dx<0?'right':dx>0?'left':'center'};
    }
    return out;
  }
  // The preview's view once it has been turned by hand; a preset or Reset all turns it back.
  let previewCamera=null;
  function busy(value) {
    el('missile-run').disabled=value||!meta||!selectedMissiles.length;
    el('missile-cancel').hidden=!value;el('missile-reset').disabled=value;
    root.querySelectorAll('[data-preset]').forEach(button=>button.disabled=value);
    el('missile-selected').querySelectorAll('button').forEach(button=>button.disabled=value);
    updateAddButton();
  }
  el('missile-search').addEventListener('input',()=>filterMissiles());
  el('missile-search').addEventListener('keydown',event=>{if(event.key==='Enter')event.preventDefault();});
  el('missile-retry').addEventListener('click',load);
  el('missile-form').addEventListener('input',event=>{
    if(event.target.id==='missile-search')return;
    if(event.target.id.match(/^missile-(launch|target)-/)) {
      root.querySelectorAll('[data-preset]').forEach(button=>button.setAttribute('aria-pressed','false'));
      el('missile-preset-note').textContent='';
    }
    changed();
  });
  el('missile-form').addEventListener('invalid',event=>{
    const details=event.target.closest('details');if(details)details.open=true;
  },true);
  function applyPreset(preset) {
    const states=JSON.parse(JSON.stringify(initial));previewCamera=null;
    if(preset==='head-on'){states.target.velocity=[-200,0,0];states.target.angles=[180,0,0];}
    if(preset==='crossing'){states.target.velocity=[0,0,200];states.target.angles=[90,0,0];}
    for(const role of ['launch','target']) {
      if(role==='launch')el('missile-launch-position-1').value=states.launch.position[1];else setTargetPosition(states.target.position);
      syncMotion(role,states[role].velocity);
    }
    root.querySelectorAll('[data-preset]').forEach(button=>button.setAttribute('aria-pressed',String(button.dataset.preset===preset)));
    el('missile-preset-note').textContent='';
    changed();
  }
  root.querySelectorAll('[data-preset]').forEach(button=>button.addEventListener('click',()=>applyPreset(button.dataset.preset)));
  el('missile-reset').addEventListener('click',()=>{
    selectedMissiles=[];renderSelected();busy(false);
    el('missile-search').value='';filterMissiles();
    applyPreset('chase');if(!meta)load();
  });
  for(const role of ['launch','target'])syncMotion(role);
  updatePreview();
  // Load a complete scenario (SI units, launcher at any position) from another tool.
  window.missileSimulator={load(scenario){
    if(batchRunning)return false;
    selectTab('missile');
    selectedMissiles=[scenario.missile];renderSelected();filterMissiles();
    const origin=scenario.launcher.position;
    // Full precision: near an envelope boundary a rounded position can change the outcome.
    el('missile-launch-position-1').value=String(origin[1]);
    setTargetPosition(scenario.target.position.map((v,i)=>i===1?v:v-origin[i]));
    syncMotion('launch',scenario.launcher.velocity,scenario.launcher.angles);syncMotion('target',scenario.target.velocity,scenario.target.angles);
    root.querySelectorAll('[data-preset]').forEach(button=>button.setAttribute('aria-pressed','false'));
    el('missile-preset-note').textContent='Loaded from the launch envelope.';
    changed();busy(false);root.scrollIntoView?.({block:'start'});
    return true;
  }};
  new ResizeObserver(()=>updatePreview()).observe(el('missile-geometry'));
  el('missile-form').addEventListener('submit',async event=>{
    event.preventDefault();if(batchRunning||!selectedMissiles.length)return;
    batchRunning=true;cancelRequested=false;pause();showError('');busy(true);
    const submitted=inputs(),completed=[],failed=[];el('missile-status').textContent='Submitting engagement…';
    try {
      const runFlight=async i=>{
        const id=submitted.missiles[i];let activeJob=null;
        const {missiles,...shared}=submitted;
        el('missile-status').textContent=`Submitting ${i+1}/${missiles.length} · ${missileName(id)}…`;
        try {
          const job=await request('/api/missiles/jobs',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({...shared,missile:id})});activeJob=job.id;activeJobs.add(activeJob);
          if(cancelRequested)await request('/api/missiles/jobs/'+activeJob+'/cancel',{method:'POST'});
          for(;;) {
            const status=await request('/api/missiles/jobs/'+activeJob);
            if(status.status==='complete') {completed.push(status.result);break;}
            if(status.status==='failed')throw new Error(status.error);
            if(status.status==='cancelled'){cancelRequested=true;break;}
            const p=status.progress;el('missile-status').textContent=`${completed.length}/${missiles.length} complete · ${missileName(id)} · `+(p?`${p.time_s.toFixed(1)} s flown…`:'Waiting for simulation…');
            await new Promise(resolve=>setTimeout(resolve,200));
          }
        } catch(error) {
          failed.push(`${missileLabel(id)}: ${error.message}`);
          if(activeJob)try{await request('/api/missiles/jobs/'+activeJob+'/cancel',{method:'POST'});}catch{}
        }
        finally {activeJobs.delete(activeJob);}
      };
      let next=0;
      const concurrency=Math.min(submitted.missiles.length,Math.max(1,Math.min(4,meta.max_concurrent_jobs||1)));
      await Promise.all(Array.from({length:concurrency},async()=>{
        while(!cancelRequested&&next<submitted.missiles.length)await runFlight(next++);
      }));
      completed.sort((a,b)=>submitted.missiles.indexOf(a.scenario.missile)-submitted.missiles.indexOf(b.scenario.missile));
      if(completed.length){
        const {missiles,...shared}=submitted;
        comparison={requested_missiles:missiles,shared_conditions:shared,complete:completed.length===missiles.length,cancelled:cancelRequested,errors:failed};
        flights=completed;resultColors=flights.map(f=>flightColors[missiles.indexOf(f.scenario.missile)]);focusIndex=0;result=flights[0];await render();
      }
      el('missile-status').textContent=cancelRequested?`Comparison cancelled · ${completed.length}/${submitted.missiles.length} flights complete.`:failed.length?`${completed.length}/${submitted.missiles.length} flights complete.`:'';
      if(failed.length)showError(failed.join(' · '));
      el('missile-stale').hidden=completed.length===submitted.missiles.length&&JSON.stringify(inputs())===JSON.stringify(submitted);
      el('missile-stale').textContent=!completed.length?(flights.length?'No new flights completed. Previous results are still displayed.':'No flights completed. Run again to compare the selected missiles.'):completed.length<submitted.missiles.length?'Comparison incomplete. Only completed flights are shown; run again to compare all selected missiles.':'Inputs changed. Run again to update the trajectories.';
    }catch(error){showError(error.message);el('missile-status').textContent='Simulation did not complete.';}
    finally{activeJobs.clear();batchRunning=false;busy(false);}
  });
  el('missile-cancel').addEventListener('click',async()=>{
    cancelRequested=true;
    el('missile-status').textContent='Cancelling comparison…';
    await Promise.all([...activeJobs].map(async id=>{try{await request('/api/missiles/jobs/'+id+'/cancel',{method:'POST'});}catch(error){showError(error.message);}}));
  });
  let camera={eye:{x:.15,y:-1.6,z:1},up:{x:0,y:0,z:1},projection:{type:'orthographic'}},flightBounds=null;
  function sceneFit(bounds,{chart=el('missile-chart'),floor=.08,pad=1.08,eye=null,points=null,viewport=null}={}) {
    const raw=bounds.map(([low,high])=>high-low),largest=Math.max(100,...raw);
    const spans=raw.map(span=>Math.max(100,largest*floor,span)*pad);
    eye=eye||(raw[0]>=raw[1]?{x:.15,y:-1.6,z:.65}:{x:1.6,y:.15,z:.65});
    const horizontal=Math.hypot(eye.x,eye.y),length=Math.hypot(horizontal,eye.z);
    const right=[-eye.y/horizontal,eye.x/horizontal,0];
    const up=[-eye.x*eye.z/(horizontal*length),-eye.y*eye.z/(horizontal*length),horizontal/length];
    const ratio=spans.map(span=>span/largest);
    // On screen: the box's projected size, or that of the given points.
    const centre=bounds.map(([low,high])=>(low+high)/2),project=(basis,p)=>basis.reduce((sum,value,i)=>sum+value*ratio[i]*(p[i]-centre[i])/spans[i],0);
    const extent=basis=>points?Math.max(...points.map(p=>project(basis,p)))-Math.min(...points.map(p=>project(basis,p))):basis.reduce((sum,value,i)=>sum+Math.abs(value)*ratio[i],0);
    viewport=viewport||(chart.clientWidth||640)/(chart.clientHeight||360);
    const scale=Math.min(1.65*viewport/extent(right),1.45/extent(up));
    return {spans,aspectratio:Object.fromEntries(['x','y','z'].map((key,i)=>[key,ratio[i]*scale])),
      camera:{eye,up:{x:0,y:0,z:1},projection:{type:'orthographic'}}};
  }
  new ResizeObserver(()=>{
    if(el('missile-view').value==='3d'&&flightBounds&&el('missile-chart').data&&el('missile-chart').clientWidth)
      Plotly.relayout(el('missile-chart'),{'scene.aspectratio':sceneFit(flightBounds).aspectratio});
  }).observe(el('missile-chart'));
  // Seeker states as short codes (Leo, 2026-10-27): tracking (observed or coasting on the track), inertial guidance, acquiring.
  const trackingLabels={observed:'TRK',coasting:'TRK',inertial:'IOG',acquiring:'ACQ'};
  const formatValue=(value,unit='',digits=1)=>Number.isFinite(value)?`${value.toLocaleString(undefined,{maximumFractionDigits:digits,minimumFractionDigits:digits})}${unit?' '+unit:''}`:'Unavailable';
  function flightHover(row,role='missile',flight=result) {
    const range=Math.hypot(...row.missile.map((v,i)=>v-row.target[i]));
    const lines=[`<b>${role==='missile'?escapeHtml(missileLabel(flight.scenario.missile)):'Target'} · ${row.time_s.toFixed(2)} s after release</b>`,
      `Altitude: ${formatValue(row[role][1],'m')}`,`Target range: ${formatValue(range,'m')}`];
    if(role==='target') {
      lines.push(`Speed: ${formatValue(Math.hypot(...flight.scenario.target.velocity)*3.6,'km/h')}`);
    } else if(row.kind==='proximity_event') {
      lines.push('Target proximity event','Speed and forces unavailable at this exact event time');
    } else {
      const t=row.telemetry||{};
      lines.push(`Speed: ${formatValue(Number.isFinite(row.speed_mps)?row.speed_mps*3.6:null,'km/h')}`,
        `Mach: ${formatValue(row.mach,'',2)}`,`Travelled: ${formatValue(row.traveled_distance_m,'m')}`,
        `Thrust: ${formatValue(t.thrust_n,'N')}`,`Drag: ${formatValue(t.drag_n,'N')}`,`Lift: ${formatValue(t.lift_n,'N')}`,
        `Mass: ${formatValue(t.mass_kg,'kg',2)}`,`Angle of attack: ${formatValue(t.aoa_deg,'°',2)}`,
        trackingLabels[row.tracking_state]||'Tracking state unavailable');
    }
    return lines.join('<br>');
  }
  const propulsionMetrics={thrust:{unit:'kN',series:[['thrust_n','Thrust',.001,'#38c9d7']]},mass:{unit:'kg',series:[['mass_kg','Missile mass',1,'#38c9d7']]},propellant:{unit:'kg',series:[['propellant_used_kg','Propellant consumed',1,'#38c9d7']]}};
  const aeroMetrics={forces:{unit:'kN',series:[['drag_n','Drag',.001,'#ffa66b'],['lift_n','Lift',.001,'#a7a0ff']]},cd:{unit:'Cd',series:[['drag_coefficient','Drag coefficient',1,'#ffa66b']]},aoa:{unit:'degrees',series:[['aoa_deg','Angle of attack',1,'#ffa66b']]},pressure:{unit:'kPa',series:[['dynamic_pressure_pa','Dynamic pressure',.001,'#ffa66b']]},acceleration:{unit:'m/s²',series:[['acceleration_mps2','Net acceleration',1,'#ffa66b']]}};
  const controlMetrics={load:{unit:'g',series:[['load_g','Manoeuvre load',1,'#91d477']]},look:{unit:'degrees',series:[['look_deg','Seeker look angle',1,'#91d477']]},los:{unit:'°/s',series:[['los_rate_dps','Line-of-sight rate',1,'#91d477']]},turn:{unit:'°/s',series:[['turn_rate_dps','Turn rate',1,'#91d477']]}};
  const DATA_KINDS=['propulsion','aero','control'],dataMetrics={propulsion:propulsionMetrics,aero:aeroMetrics,control:controlMetrics};
  // Control quantities from each saved sample's motion (Leo, 2026-10-27); the engine records no fin or command outputs.
  const controlCache=new WeakMap();
  function controlSeries(flight) {
    if(controlCache.has(flight))return controlCache.get(flight);
    const rows=flight.trajectory,n=rows.length,G=9.80665,deg=180/Math.PI,out={load_g:[],look_deg:[],los_rate_dps:[],turn_rate_dps:[]};
    const sub=(a,b)=>a.map((v,i)=>v-b[i]),dot=(a,b)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2],norm=a=>Math.hypot(...a);
    const angle=(a,b)=>Math.acos(Math.max(-1,Math.min(1,dot(a,b)/(norm(a)*norm(b)||1))));
    rows.forEach((r,i)=>{
      // The nose from the attitude quaternion [x, y, z, w] (missile_model/control_frame.frame); the look angle is to the target.
      const q=r.quaternion,los=sub(r.target,r.missile);
      const nose=q?[2*(q[0]*q[0]+q[3]*q[3])-1,2*(q[3]*q[2]+q[1]*q[0]),2*(q[2]*q[0]-q[3]*q[1])]:null;
      out.look_deg.push(nose&&norm(los)>0?angle(nose,los)*deg:null);
      // Central differences: acceleration across the velocity (turn rate), less gravity (load), and the sight line's turn.
      const a=rows[Math.max(0,i-1)],b=rows[Math.min(n-1,i+1)],dt=b.time_s-a.time_s,v=r.velocity;
      if(!(dt>1e-6)||!a.velocity||!b.velocity||!v){out.load_g.push(null);out.turn_rate_dps.push(null);out.los_rate_dps.push(null);return;}
      const speed=norm(v),across=c=>{const k=dot(c,v)/(speed*speed||1);return c.map((x,j)=>x-k*v[j]);};
      const acceleration=sub(b.velocity,a.velocity).map(c=>c/dt);
      out.turn_rate_dps.push(speed?norm(across(acceleration))/speed*deg:null);
      out.load_g.push(norm(across(acceleration.map((c,j)=>j===1?c+G:c)))/G);
      out.los_rate_dps.push(angle(sub(b.target,b.missile),sub(a.target,a.missile))/dt*deg);
    });
    controlCache.set(flight,out);return out;
  }
  function timeCursor() {
    return [{type:'line',xref:'x',yref:'paper',x0:playbackTime,x1:playbackTime,y0:0,y1:1,line:{color:'#b8cada',width:1,dash:'dot'}}];
  }
  function resizeDataPlots() {
    if(el('missile-flight-data').open)for(const kind of DATA_KINDS)if(el(`missile-${kind}-chart`).data)Plotly.Plots.resize(el(`missile-${kind}-chart`));
  }
  const totalTime=()=>Math.max(...flights.map(f=>f.elapsed_s));
  function indexAt(rows,time,key='time_s') {
    let low=0,high=rows.length-1;
    while(low<high){const middle=Math.ceil((low+high)/2);if((key?rows[middle][key]:rows[middle])<=time)low=middle;else high=middle-1;}
    return low;
  }
  function inspectFlight(index) {
    focusIndex=index;result=flights[index];
    el('missile-legend').querySelectorAll('button').forEach((button,i)=>button.setAttribute('aria-pressed',String(i===index)));
    el('missile-model-details').replaceChildren(...result.model.limitations.map(text=>{const li=document.createElement('li');li.textContent=text;return li;}));
  }
  async function renderDataPlots() {
    if(!result||!el('missile-flight-data').open)return;
    await Promise.all(DATA_KINDS.map(async kind=>{
      const node=el(`missile-${kind}-chart`),key=el(`missile-${kind}-metric`).value;
      const metric=dataMetrics[kind][key];
      el(`missile-${kind}-legend`).replaceChildren(...flights.map((flight,i)=>{
        const label=document.createElement('span');label.textContent=`━ ${i+1} · ${missileName(flight.scenario.missile)}`;
        label.style.color=resultColors[i];label.title=missileLabel(flight.scenario.missile);return label;
      }));
      const traces=flights.flatMap((flight,i)=>metric.series.map(([field,name,scale],s)=>({type:'scatter',mode:'lines',name:`${i+1} · ${missileName(flight.scenario.missile)}`,
        legendgroup:String(i),showlegend:s===0,
        x:flight.trajectory.map(r=>r.time_s),y:kind==='control'?controlSeries(flight)[field].map(v=>Number.isFinite(v)?v*scale:null):flight.trajectory.map(r=>Number.isFinite(r.telemetry?.[field])?r.telemetry[field]*scale:null),
        customdata:flight.trajectory.map(r=>[i,r.time_s]),connectgaps:false,line:{color:resultColors[i],width:2,dash:s?'dash':'solid'},
        hovertemplate:`${escapeHtml(missileLabel(flight.scenario.missile))}<br>%{x:.3f} s<br>${name}: %{y:.3f} ${metric.unit}<extra></extra>`})));
      await Plotly.react(node,traces,{paper_bgcolor:'#000000',plot_bgcolor:'#000000',font:{color:'#dce5f0',size:11},
        margin:{l:62,r:18,t:30,b:55},xaxis:{title:'Time after release · s',gridcolor:'rgba(240,240,250,.09)',range:[0,totalTime()]},
        yaxis:{title:metric.unit,gridcolor:'rgba(240,240,250,.09)',rangemode:'tozero'},showlegend:false,
        hovermode:'x unified',shapes:timeCursor(),uirevision:flightRevision+':'+key,dragmode:'pan',modebar:{bgcolor:'rgba(0,0,0,0)',color:'#8a8a96',activecolor:'#f0f0fa'}},
        {responsive:true,displaylogo:false,scrollZoom:true,modeBarButtonsToRemove:['select2d','lasso2d'],toImageButtonOptions:{filename:'missile-comparison-'+kind+'-'+key,format:'png',scale:2}});
      node.removeAllListeners('plotly_click');
      node.on('plotly_click',event=>{const point=event.points?.[0]?.customdata;if(Array.isArray(point)){pause();inspectFlight(point[0]);updateFrame(indexAt(timeline,point[1],null));}});
    }));
  }
  el('missile-flight-data').addEventListener('toggle',()=>{if(el('missile-flight-data').open)renderDataPlots().catch(error=>showError(error.message));});
  for(const kind of DATA_KINDS)el(`missile-${kind}-metric`).addEventListener('change',()=>renderDataPlots().catch(error=>showError(error.message)));
  const viewAxes={ '3d':[0,2,1],xz:[0,2],xy:[0,1],zy:[2,1] };
  const axisNames=['X · m','Altitude · Y · m','Z · m'];
  function projectedPositions(positions,view=el('missile-view').value) {
    return Object.fromEntries(viewAxes[view].map((axis,i)=>[['x','y','z'][i],positions.map(p=>p[axis])]));
  }
  async function renderTrajectory() {
    const view=el('missile-view').value,is3d=view==='3d',type=is3d?'scatter3d':'scatter';
    const longest=flights.reduce((a,b)=>a.elapsed_s>b.elapsed_s?a:b);
    const trace=(flight,role,color)=>({type,mode:'lines',name:role==='missile'?missileLabel(flight.scenario.missile):'Target',...projectedPositions(role==='missile'?[]:flight.trajectory.map(r=>r[role]),view),text:role==='missile'?[]:undefined,line:{color,width:role==='target'?(is3d?2:1.5):(is3d?5:2.5),dash:role==='target'?'dash':'solid'},...(role==='target'?{hoverinfo:'skip',opacity:.3}:{hovertemplate:'%{text}<extra></extra>'})});
    const marker=(name,color,interactive=true)=>({type,mode:'markers',name,...projectedPositions([],view),marker:{color,size:is3d?6:10},...(interactive?{hovertemplate:'%{text}<extra></extra>'}:{hoverinfo:'skip'}),showlegend:false});
    const bounds=[0,2,1].map(i=>{
      const values=flights.flatMap(f=>f.trajectory.flatMap(r=>[r.missile[i],r.target[i]]));
      return [Math.min(...values),Math.max(...values)];
    });
    flightBounds=bounds;const fitted=sceneFit(bounds);camera=fitted.camera;
    const axis=(title,index)=>{const center=(bounds[index][0]+bounds[index][1])/2,span=fitted.spans[index];return {title:{text:index===2?'':title,font:{size:11}},range:[center-span/2,center+span/2],nticks:span<Math.max(...fitted.spans)*.2?3:6,tickfont:{size:10},tickangle:0,tickformat:',.0f',showspikes:false,gridcolor:'rgba(240,240,250,.09)',zerolinecolor:'#455568',color:'#a9b8ca',backgroundcolor:'#000000'};};
    const sliceAxis=(worldAxis)=>({title:{text:axisNames[worldAxis]},gridcolor:'rgba(240,240,250,.09)',zerolinecolor:'#455568',tickformat:',.0f',automargin:true});
    // Mouse logic as on the EM chart (Leo, 2026-10-27): drag pans (turns the 3D view), the wheel zooms, the toolbar resets.
    const layout={paper_bgcolor:'#000000',plot_bgcolor:'#000000',font:{color:'#dce5f0'},hoverlabel:{bgcolor:'#0c0c0f',bordercolor:'#38c9d7',font:{color:'#e3edf7',size:12}},showlegend:false,uirevision:flightRevision+':'+view,
      dragmode:is3d?'turntable':'pan',modebar:{bgcolor:'rgba(0,0,0,0)',color:'#8a8a96',activecolor:'#f0f0fa'},
      ...(is3d?{margin:{l:0,r:0,t:0,b:0},annotations:[{text:'Y = altitude · m',xref:'paper',yref:'paper',x:.015,y:.985,xanchor:'left',yanchor:'top',showarrow:false,font:{size:10,color:'#a9b8ca'}}],scene:{xaxis:axis('X · m',0),yaxis:axis('Z · m',1),zaxis:axis('Altitude · m',2),aspectmode:'manual',aspectratio:fitted.aspectratio,camera}}:
        {margin:{l:70,r:25,t:20,b:60},xaxis:{...sliceAxis(viewAxes[view][0]),constrain:'domain'},yaxis:{...sliceAxis(viewAxes[view][1]),scaleanchor:'x',scaleratio:1},hovermode:'closest'})};
    el('missile-slice-note').hidden=is3d;
    el('missile-chart').setAttribute('aria-label',is3d?'Interactive 3D missile and target trajectories':`Interactive 2D missile and target trajectories: ${axisNames[viewAxes[view][0]]} / ${axisNames[viewAxes[view][1]]}`);
    await Plotly.newPlot(el('missile-chart'),[...flights.map((f,i)=>trace(f,'missile',resultColors[i])),trace(longest,'target','#ffa66b'),...flights.map((f,i)=>marker(missileName(f.scenario.missile)+' now',resultColors[i])),marker('Target now','#ffd0ac',false)],
      layout,
      {responsive:true,displaylogo:false,scrollZoom:true,modeBarButtonsToRemove:['toImage','select2d','lasso2d']});
  }
  async function render() {
    flightRevision++;
    timeline=[...new Set(flights.flatMap(f=>f.trajectory.map(r=>r.time_s)))].sort((a,b)=>a-b);
    for(const input of root.querySelectorAll('input[name=missile-plot-view]'))input.disabled=false;
    showPanes(flightView);el('missile-view').value=flightView;
    el('missile-view').disabled=true;
    try{await renderTrajectory();}finally{el('missile-view').disabled=false;}
    el('missile-legend').replaceChildren(...flights.map((flight,i)=>{
      const button=document.createElement('button');button.type='button';button.textContent=`━ ${i+1} · ${missileName(flight.scenario.missile)}`;button.style.color=resultColors[i];button.title=missileLabel(flight.scenario.missile);
      button.addEventListener('click',()=>{inspectFlight(i);updateFrame(Number(el('missile-time').value));});return button;
    }));
    const targetLegend=document.createElement('span');targetLegend.textContent='┄ Target';targetLegend.style.color='#ffa66b';targetLegend.style.opacity='.45';el('missile-legend').append(targetLegend);
    el('missile-time').max=timeline.length-1;el('missile-time').value=0;el('missile-time').disabled=false;el('missile-play').disabled=false;
    inspectFlight(focusIndex);
    const single=comparison.requested_missiles.length===1;
    const exported=single?flights[0]:{format:'missile-comparison-v1',...comparison,flights};
    if(downloadUrl)URL.revokeObjectURL(downloadUrl);downloadUrl=URL.createObjectURL(new Blob([JSON.stringify(exported,null,2)],{type:'application/json'}));
    el('missile-download').href=downloadUrl;el('missile-download').download=single?result.scenario.missile+'-engagement.json':'missile-comparison.json';el('missile-download').setAttribute('aria-disabled','false');
    el('missile-flight-data').hidden=false;
    el('missile-flight-name').textContent=flights.length===1?missileName(result.scenario.missile):`${flights.length} missiles`;
    updateFrame(0);
    await renderDataPlots();
  }
  function updateFrame(index) {
    if(!result)return;const time=timeline[index],rows=flights.map(f=>f.trajectory[indexAt(f.trajectory,time)]),row=rows[focusIndex];
    el('missile-time').value=index;playbackTime=time;el('missile-time-label').textContent=time.toFixed(2)+' s';
    const finished=time>=result.elapsed_s;
    el('missile-frame').textContent=`${missileName(result.scenario.missile)} · `+(finished?`${result.outcome.label} at ${result.elapsed_s.toFixed(2)} s`:row.kind==='proximity_event'?'Target proximity event':(trackingLabels[row.tracking_state]||'—'));
    const targetPosition=result.scenario.target.position.map((v,i)=>v+result.scenario.target.velocity[i]*time);
    const positions=[...rows.map(r=>r.missile),targetPosition];
    const projected=Object.fromEntries(Object.entries(projectedPositions(positions)).map(([axis,values])=>[axis,values.map(v=>[v])]));
    const trails=flights.map(f=>f.trajectory.slice(0,indexAt(f.trajectory,time)+1));
    const trailPositions=trails.map(path=>projectedPositions(path.map(r=>r.missile)));
    const trailUpdate=Object.fromEntries(Object.keys(trailPositions[0]).map(axis=>[axis,trailPositions.map(p=>p[axis])]));
    Plotly.restyle(el('missile-chart'),{...trailUpdate,text:trails.map((path,i)=>path.map(r=>flightHover(r,'missile',flights[i])))},flights.map((_,i)=>i));
    Plotly.restyle(el('missile-chart'),{...projected,text:[...rows.map((r,i)=>[flightHover(r,'missile',flights[i])+(time>flights[i].elapsed_s?'<br>Flight ended · marker remains at the final position':'')]),[]]},positions.map((_,i)=>flights.length+1+i));
    if(el('missile-flight-data').open)for(const kind of DATA_KINDS)if(el(`missile-${kind}-chart`).data)Plotly.relayout(el(`missile-${kind}-chart`),{shapes:timeCursor()});
  }
  el('missile-time').addEventListener('input',()=>{pause();if(plotView==='setup')showView(flightView);updateFrame(Number(el('missile-time').value));});
  el('missile-view').addEventListener('change',async()=>{
    if(!result)return;
    pause();const index=Number(el('missile-time').value);el('missile-view').disabled=true;
    try{await renderTrajectory();updateFrame(index);}catch(error){showError(error.message);}finally{el('missile-view').disabled=false;}
  });
  el('missile-play').addEventListener('click',()=>{
    if(playing){pause();return;}if(!result)return;
    if(plotView==='setup')showView(flightView);
    if(Number(el('missile-time').value)>=timeline.length-1)updateFrame(0);
    playing=true;lastFrame=performance.now();el('missile-play').textContent='Pause';requestAnimationFrame(animate);
  });
  function animate(now) {
    if(!playing)return;const wanted=playbackTime+(now-lastFrame)/1000*Number(el('missile-play-speed').value);lastFrame=now;
    let index=Number(el('missile-time').value);while(index<timeline.length-1&&timeline[index+1]<=wanted)index++;
    updateFrame(index);playbackTime=wanted;
    if(index>=timeline.length-1)pause();else requestAnimationFrame(animate);
  }
})();
