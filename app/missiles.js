(() => {
  'use strict';
  const el = id => document.getElementById(id);
  const root = el('missile-workspace');
  const initial = {missile:'us_aim9l_sidewinder',duration:60,
    launch:{position:[0,5000,0],velocity:[300,0,0],angles:[0,0,0]},
    target:{position:[4000,5000,0],velocity:[200,0,0],angles:[0,0,0]}};
  let meta=null, loading=null, result=null, playing=false, lastFrame=0, playbackTime=0, downloadUrl=null, flightRevision=0;
  const activeJobs=new Set();
  let selectedMissiles=[], flights=[], resultColors=[], timeline=[], comparison=null, batchRunning=false, cancelRequested=false, focusIndex=0;
  const flightColors=['#38c9d7','#b79aff','#91d477','#ee8eb6','#f0d367','#79a7fa','#cfb296','#f08070'];
  const missileName=id=>meta?.missiles.find(m=>m.id===id)?.name||id;
  const missileLabel=id=>`${missileName(id)} [${id}]`;
  const escapeHtml=text=>String(text).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
  const positionFields = role => `<div class="missile-vector"><span>Aircraft position · m</span><div class="${role==='launch'?'missile-altitude-input':'three-inputs'}">${['Along X','Altitude · Y','Along Z'].map((axis,i)=>role==='launch'&&i!==1?'':`<label>${axis}<input id="missile-${role}-position-${i}" aria-label="${role==='launch'?'Launcher':'Target'} ${axis}" type="number" step="any" required value="${initial[role].position[i]}" ${i===1?'min="0.001" max="30000"':'min="-200000" max="200000"'}></label>`).join('')}</div></div>`;
  const motion = role => `<div class="missile-motion">${[['speed','Speed','km/h',0,12470.76582],['course','Travel heading','°',-360,360],['climb',role==='launch'?'Launch angle':'Climb angle','°',-90,90]].map(([key,label,unit,min,max])=>`<label class="field">${label}<div class="input-unit"><input id="missile-${role}-${key}" aria-label="${role==='launch'?'Launcher':'Target'} ${label.toLowerCase()}" type="number" min="${min}" max="${max}" step="any" required><span>${unit}</span></div></label>`).join('')}</div>`;
  const stateFields = (role,number,title,description) => `<fieldset class="missile-step missile-${role}"><legend><span class="missile-step-number">${number}</span>${title}</legend><p class="hint">${description}</p>${positionFields(role)}${motion(role)}
    <p class="hint">Travel heading sets movement: 0° = +X, 90° = +Z. ${role==='launch'?'Launch angle':'Climb angle'}: positive = climbing, 0° = level.</p>
    </fieldset>`;
  root.innerHTML = `<aside class="missile-sidebar"><form id="missile-form">
    <div class="section-heading"><span class="eyebrow">Engagement setup</span><button id="missile-reset" class="text-button" type="button">Reset all</button></div>
    <h2 class="missile-setup-title">Build your engagement</h2><p class="hint">Start with an example, then adjust the flight conditions.</p>
    <div class="missile-presets" role="group" aria-label="Starter scenarios">
      <button type="button" data-preset="chase" aria-pressed="true"><span aria-hidden="true">⇉</span>Tail chase</button>
      <button type="button" data-preset="head-on" aria-pressed="false"><span aria-hidden="true">⇄</span>Head-on</button>
      <button type="button" data-preset="crossing" aria-pressed="false"><span aria-hidden="true">↱</span>Crossing</button>
    </div><p id="missile-preset-note" class="hint">Target starts 4 km ahead, flying away at 720 km/h. Examples replace both aircraft states.</p>
    <fieldset id="missile-picker" class="missile-step"><legend>Add missiles <small>Up to eight entries</small></legend>
      <input id="missile-search" type="search" placeholder="Search missiles…" autocomplete="off" aria-label="Search missiles" aria-controls="missile-list">
      <div id="missile-list" class="aircraft-list" aria-label="Available missiles"></div>
      <p id="missile-search-empty" class="hint" hidden>No missiles match your search.</p>
      <p id="missile-catalog-note" class="hint" role="status">Loading missiles…</p>
      <div class="section-heading"><span class="eyebrow">Selected missiles</span><span id="missile-entry-count" class="hint">0 / 8</span></div>
      <p id="missile-entries-empty" class="hint">No missiles added.</p>
      <div id="missile-selected" class="missile-selected" aria-label="Selected missiles"></div>
      <p class="hint">All missiles launch at t = 0 from the same aircraft setup and fly independently toward the same target.</p>
      <details class="missile-timing"><summary id="missile-timing-title">Automatic release &amp; timing</summary><dl id="missile-timing-values"></dl><p class="hint">Times start at release (t = 0). Each motor uses its own clock; later stages may wait for flight conditions. Guidance ramps and seeker search gates remain separate.</p><p class="hint">Seeker warm-up is complete before release. The missile starts at the launcher’s position, aligned with its body. Aircraft-specific hardpoint offsets and ejector impulses are not supplied.</p></details>
      <button id="missile-retry" class="text-button" type="button" hidden>Retry loading missiles</button>
    </fieldset>
    <div class="missile-coordinate-guide"><strong>One shared world coordinate system</strong><p>The launcher starts at X = 0, Z = 0. +X points right in the top view, +Z points up, and +Y is altitude. Target X and Z locate it relative to the launcher's starting horizontal position; altitude is absolute.</p><p>Aircraft face their travel heading and vertical angle (launcher: launch angle; target: climb angle), with zero roll.</p></div>
    ${stateFields('launch',2,'Launcher aircraft','Aircraft state at the instant of release (t = 0). Missile release adjustments are applied automatically.')}
    ${stateFields('target',3,'Target aircraft','Flies in a straight line at constant velocity throughout the simulation.')}
    <fieldset class="missile-step"><legend><span class="missile-step-number">4</span>Simulation time</legend>
      <label class="field">Stop after<div class="input-unit"><input id="missile-duration" type="number" min="0.1" max="180" step="any" value="60" required><span>s</span></div></label>
      <p class="hint">Up to 180 seconds. The run may end earlier at interception or a modeled limit.</p>
    </fieldset>
    <div class="missile-run-panel">
    <p id="missile-ready" class="hint"></p>
    <button id="missile-run" class="primary-button" type="submit" disabled>Simulate engagement <span>↗</span></button>
    <button id="missile-cancel" class="secondary-button" type="button" hidden>Cancel simulation</button>
    <div id="missile-status" role="status" aria-live="polite"></div></div>
  </form></aside>
  <div class="missile-main">
    <div class="page-heading"><div><span class="eyebrow">Missile flight</span><h1>3D engagement simulator</h1></div><span class="status-badge">Experimental</span></div>
    <p class="missile-model-note">Set the launcher and target aircraft at release. Missile launch processing, ignition delays and guidance timing follow the selected game profile. Ideal target visibility and radar support retain geometric seeker limits.</p>
    <div id="missile-error" class="error" role="alert" hidden></div>
    <div id="missile-stale" class="notice" hidden>Inputs changed. Run again to update the trajectories.</div>
    <section class="missile-preview" aria-labelledby="missile-preview-title">
      <div class="missile-preview-heading"><div><span class="eyebrow">Setup preview</span><h2 id="missile-preview-title">Starting geometry</h2></div><span class="missile-preview-tag">Top view · X / Z</span></div>
      <svg id="missile-geometry" viewBox="0 0 640 200" role="img" aria-label="Starting positions and flight directions"></svg>
      <div class="missile-setup-stats"><div><span>Aircraft separation</span><strong id="missile-separation">—</strong></div><div><span>Target altitude difference</span><strong id="missile-altitude-difference">—</strong></div><div><span>Aircraft closing speed</span><strong id="missile-closing">—</strong></div></div>
      <p id="missile-preview-note" class="hint">Arrows show flight direction. Preview uses current inputs; it is not a simulated trajectory.</p>
    </section>
    <section class="chart-card"><div class="chart-toolbar"><div id="missile-legend" class="missile-legend"><span>━ Missile</span><span>━ Target</span></div><div class="missile-view-controls"><label for="missile-view">Plot view<select id="missile-view"><option value="3d">3D</option><option value="xz">2D slice · Top (X / Z)</option><option value="xy">2D slice · Side (X / altitude)</option><option value="zy">2D slice · Front (Z / altitude)</option></select></label><button id="missile-view-reset" type="button">Reset view</button></div></div>
    <p id="missile-slice-note" class="hint missile-hover-hint" hidden>2D slices project the full trajectories onto the selected plane.</p>
    <p class="hint missile-hover-hint">Hover over the missile or its flight path to inspect flight data.</p>
    <div id="missile-chart" aria-label="Interactive 3D missile and target trajectories"><div class="empty"><h2>Set up a missile flight</h2><p>Choose a starter scenario or adjust the setup,<br>then select <strong>Simulate engagement</strong> to see the 3D flight.</p><button id="missile-run-preview" class="secondary-button" type="submit" form="missile-form" disabled>Simulate engagement</button></div></div>
    <div class="missile-playback"><button id="missile-play" type="button" disabled>Play</button><input id="missile-time" type="range" aria-label="Trajectory time" min="0" max="0" value="0" step="1" disabled><output id="missile-time-label">0.00 s</output><select id="missile-play-speed" aria-label="Playback speed"><option value="1">1×</option><option value="4" selected>4×</option><option value="10">10×</option></select></div></section>
    <p id="missile-frame" class="missile-frame-readout"></p>
    <label id="missile-result-label" class="field missile-result-choice" hidden>Inspect missile result<select id="missile-result-select"></select></label>
    <div class="missile-metrics"><div class="missile-metric"><span>Outcome</span><strong id="missile-outcome">—</strong></div><div class="missile-metric"><span>Closest approach</span><strong id="missile-closest">—</strong></div><div class="missile-metric"><span>Elapsed time</span><strong id="missile-elapsed">—</strong></div></div>
    <details id="missile-flight-data" class="missile-flight-data" hidden><summary>Propulsion &amp; aerodynamic plots <span id="missile-flight-name"></span></summary>
      <p class="hint">Compare every missile using the same colors as the 3D view. Hover for values; click a plotted point to inspect that missile and time. In Drag &amp; lift, solid lines show drag and dashed lines show lift.</p>
      <div class="missile-data-grid">
        <section class="chart-card"><label class="field missile-plot-choice" for="missile-propulsion-metric">Propulsion<select id="missile-propulsion-metric"><option value="thrust">Thrust · kN</option><option value="mass">Missile mass · kg</option><option value="propellant">Propellant consumed · kg</option></select></label><div id="missile-propulsion-legend" class="missile-plot-legend" aria-label="Missile colors"></div><div id="missile-propulsion-chart" class="missile-data-chart" aria-label="Propulsion over flight time"></div></section>
        <section class="chart-card"><label class="field missile-plot-choice" for="missile-aero-metric">Aerodynamics<select id="missile-aero-metric"><option value="forces">Drag &amp; lift · kN</option><option value="cd">Drag coefficient · Cd</option><option value="aoa">Angle of attack · degrees</option><option value="pressure">Dynamic pressure · kPa</option><option value="acceleration">Net acceleration · m/s²</option></select></label><div id="missile-aero-legend" class="missile-plot-legend" aria-label="Missile colors"></div><div id="missile-aero-chart" class="missile-data-chart" aria-label="Aerodynamics over flight time"></div></section>
      </div><p class="hint">Forces come from the final integration substep before each saved position. Lift and drag are force magnitudes; angle of attack is unsigned local-flow incidence. Net acceleration includes gravity. Missing values at release or the exact proximity event are left blank.</p>
    </details>
    <div class="exports"><span>Export</span><a id="missile-download" aria-disabled="true">Flight data JSON</a></div>
    <details class="missile-details"><summary>Model and current limitations</summary><p>Recovered profile baseline: War Thunder 2.59.0.34. Seekers begin with warm-up complete, respecting designation and angle limits. Radar missiles retain their lock-before-launch or lock-after-launch behavior. Ideal radar support moves from the launch position at the entered launch velocity; recovered inertial guidance remains active where configured. Target geometry is a point, so a proximity event does not assert aircraft damage. Full engagements and native collision timing are still being validated.</p><ul id="missile-model-details"></ul></details>
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
  function selectTab(missile) {
    el('em-workspace').hidden=missile;root.hidden=!missile;
    for(const [id,selected] of [['em-tab',!missile],['missile-tab',missile]]) {
      el(id).setAttribute('aria-selected',String(selected));el(id).tabIndex=selected?0:-1;
    }
    if(missile) {
      if(!meta) load();
      if(result) requestAnimationFrame(()=>{Plotly.Plots.resize(el('missile-chart'));resizeDataPlots();});
    } else {pause(); if(el('chart').data) requestAnimationFrame(()=>Plotly.Plots.resize(el('chart')));}
  }
  el('em-tab').addEventListener('click',()=>selectTab(false));
  el('missile-tab').addEventListener('click',()=>selectTab(true));
  document.querySelector('.tool-tabs').addEventListener('keydown',event=>{
    if(!['ArrowLeft','ArrowRight','Home','End'].includes(event.key)) return;
    event.preventDefault();const missile=event.key==='End'||(event.key!=='Home'&&root.hidden);
    selectTab(missile);el(missile?'missile-tab':'em-tab').focus();
  });
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
      populateMissiles();renderSelected();updateTiming();busy(false);
    }catch(error){meta=null;busy(false);el('missile-retry').hidden=false;showError('Could not load missile configurations. Check that the simulator server is running, then retry. '+error.message);}
    finally{loading=null;}})();return loading;
  }
  function inputs() {
    const config={missiles:[...selectedMissiles],duration:Number(el('missile-duration').value)};
    for(const role of ['launch','target']) {
      const [speed,heading,climb]=['speed','course','climb'].map(key=>Number(el(`missile-${role}-${key}`).value));
      const horizontal=speed/3.6*Math.cos(radians(climb));
      config[role==='launch'?'launcher':role]={
        position:[0,1,2].map(i=>role==='launch'&&i!==1?0:Number(el(`missile-${role}-position-${i}`).value)),
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
      const label=document.createElement('span'),name=document.createElement('strong'),detail=document.createElement('small');
      name.textContent=missileLabel(item.id);detail.textContent=family;label.append(name,detail);
      const button=document.createElement('button');button.type='button';button.dataset.addMissile=item.id;
      button.textContent='Add';button.setAttribute('aria-label','Add '+missileLabel(item.id));
      row.append(label,button);return row;
    }));
    el('missile-catalog-note').textContent=`${meta.missiles.length} available missiles`;
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
      const row=document.createElement('div');row.className='missile-selected-row';
      const name=document.createElement('span');name.textContent=missileLabel(id);name.style.borderColor=flightColors[i];
      const remove=document.createElement('button');remove.type='button';remove.textContent='×';remove.disabled=batchRunning;
      remove.setAttribute('aria-label','Remove '+missileLabel(id));remove.title='Remove missile';
      remove.addEventListener('click',()=>{selectedMissiles.splice(i,1);renderSelected();filterMissiles();changed();busy(false);});
      row.append(name,remove);return row;
    }));
    el('missile-entry-count').textContent=`${selectedMissiles.length} / 8`;
    el('missile-entries-empty').hidden=selectedMissiles.length>0;
    updateTiming();updateAddButton();
  }
  el('missile-list').addEventListener('click',event=>{
    const button=event.target.closest('[data-add-missile]');
    if(!button||button.disabled)return;
    selectedMissiles.push(button.dataset.addMissile);renderSelected();filterMissiles();changed();busy(false);
  });
  function updateTiming() {
    const id=selectedMissiles.at(-1);
    el('missile-timing-title').textContent=id?'Release & timing · '+missileName(id):'Automatic release & timing';
    const timing=meta?.missiles.find(item=>item.id===id)?.timing;
    const list=el('missile-timing-values');list.replaceChildren();
    if(!timing){const note=document.createElement('dd');note.textContent=id?'Timing details need the updated local simulator server.':'Add a missile to see its timing details.';list.append(note);return;}
    const seconds=value=>Number(value.toFixed(3))+' s';
    const rows=[['Motor ignition delays',timing.motor_delays_s.map((delay,i)=>`Motor ${i+1}: ${seconds(delay)}`).join(' · ')],
      ['Guidance gain over time',timing.guidance_gain.map(point=>`${seconds(point.time_s)}: ${Math.round(point.gain*100)}%`).join(' → ')],
      ['Seeker search gate',`${seconds(timing.seeker_search_delay_s)}${timing.lock_after_launch?' · lock after launch':''}; geometry still applies`],
      ['Proximity arming delay',timing.proximity_delay_s===null?'No proximity fuze':seconds(timing.proximity_delay_s)+' + distance / altitude gates'],
      ['Seeker preparation',`${seconds(timing.warm_up_s)} warm-up completed before release`]];
    for(const [label,value] of rows){const dt=document.createElement('dt'),dd=document.createElement('dd');dt.textContent=label;dd.textContent=value;list.append(dt,dd);}
  }
  const radians = angle => angle*Math.PI/180;
  const degrees = angle => angle*180/Math.PI;
  function syncMotion(role,velocity=initial[role].velocity) {
    const [x,y,z]=velocity.map(v=>v*3.6);
    const speed=Math.hypot(x,y,z);
    for(const [key,value] of Object.entries({speed,course:speed?degrees(Math.atan2(z,x)):0,climb:speed?degrees(Math.atan2(y,Math.hypot(x,z))):0})) {
      el(`missile-${role}-${key}`).value=Number(value.toFixed(10));
    }
  }
  function changed() {
    if(result)el('missile-stale').hidden=false;
    updatePreview();
  }
  function updatePreview() {
    const targetX=el('missile-target-position-0');targetX.setCustomValidity('');
    const numericFields=[...el('missile-form').querySelectorAll('input[type=number]')];
    for(const role of ['launch','target'])el(`missile-${role}-speed`).setCustomValidity('');
    if(numericFields.every(field=>field.validity.valid)) {
      const config=inputs();
      for(const role of ['launch','target'])if(config[role==='launch'?'launcher':role].velocity.some(v=>Math.abs(v)>2000))el(`missile-${role}-speed`).setCustomValidity('Reduce speed: each world-axis velocity must be at most 7,200 km/h.');
      if(Math.hypot(...config.target.position.map((v,i)=>v-config.launcher.position[i]))<10)targetX.setCustomValidity('Place the target at least 10 m from the launcher aircraft.');
    }
    const invalid=numericFields.find(field=>!field.validity.valid),valid=!invalid;
    el('missile-ready').textContent=valid?(selectedMissiles.length?`${selectedMissiles.length} missile${selectedMissiles.length===1?'':'s'} ready · shared aircraft setup.`:'Add a missile to simulate.'):`${invalid.getAttribute('aria-label')||'Simulation time'}: ${invalid.validationMessage}`;
    if(!valid) {
      for(const id of ['missile-separation','missile-altitude-difference','missile-closing'])el(id).textContent='—';
      el('missile-geometry').replaceChildren();
      el('missile-geometry').setAttribute('aria-label','Preview unavailable. Check the setup values.');
      el('missile-preview-note').textContent='Complete the numeric fields within their allowed ranges to see the preview.';
      return;
    }
    const {launcher:launch,target}=inputs(),offset=target.position.map((v,i)=>v-launch.position[i]);
    const distance=Math.hypot(...offset),altitude=offset[1];
    const closing=distance?offset.reduce((sum,v,i)=>sum+v*(launch.velocity[i]-target.velocity[i]),0)/distance:0;
    const format=value=>value.toLocaleString(undefined,{maximumFractionDigits:1});
    el('missile-separation').textContent=format(distance)+' m';
    el('missile-altitude-difference').textContent=altitude===0?'Same altitude':`${format(Math.abs(altitude))} m ${altitude>0?'above':'below'}`;
    el('missile-closing').textContent=distance===0?'Same position':`${format(Math.abs(closing)*3.6)} km/h${closing<0?' · opening':closing>0?' · closing':''}`;
    const width=Math.max(240,el('missile-geometry').clientWidth||640),height=150;
    el('missile-geometry').setAttribute('viewBox',`0 0 ${width} ${height}`);
    const scale=Math.min((width-150)/Math.max(Math.abs(offset[0]),1),70/Math.max(Math.abs(offset[2]),1));
    const dx=offset[0]*scale,dz=offset[2]*scale;
    const points=[[width/2-dx/2,height/2+dz/2],[width/2+dx/2,height/2-dz/2]];
    const markers=[launch,target].map((state,i)=>{
      const [x,y]=points[i],color=i?'#ffa66b':'#38c9d7',name=i?'Target aircraft':'Launcher aircraft';
      const horizontal=Math.hypot(state.velocity[0],state.velocity[2]);
      const vx=horizontal?state.velocity[0]/horizontal*34:0,vz=horizontal?-state.velocity[2]/horizontal*34:0;
      return `<g fill="${color}">${horizontal?`<path d="M ${x} ${y} l ${vx} ${vz}" fill="none" stroke="${color}" stroke-width="2" marker-end="url(#missile-arrow-${i})"/>`:''}<circle cx="${x}" cy="${y}" r="5"/><text x="${x}" y="${y+(i?27:-16)}" text-anchor="middle">${name}</text></g>`;
    }).join('');
    el('missile-geometry').innerHTML=`<defs>${['#38c9d7','#ffa66b'].map((color,i)=>`<marker id="missile-arrow-${i}" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="5" markerHeight="5" orient="auto-start-reverse"><path d="M 0 0 L 10 5 L 0 10 z" fill="${color}"/></marker>`).join('')}<pattern id="missile-grid" width="32" height="25" patternUnits="userSpaceOnUse"><path d="M 32 0 H 0 V 25" fill="none" stroke="#223042" stroke-width=".6"/></pattern></defs><rect width="${width}" height="${height}" fill="url(#missile-grid)"/><path d="M ${points[0].join(' ')} L ${points[1].join(' ')}" stroke="#78879a" stroke-dasharray="5 5"/>${markers}<path d="M 24 42 V 20 M 24 42 H 48" stroke="#8d9aae"/><g fill="#8d9aae" font-size="10"><text x="15" y="14">+Z</text><text x="53" y="46">+X</text></g>`;
    el('missile-geometry').setAttribute('aria-label',`Top view of aircraft at release. Initial separation ${format(distance)} meters. Target ${altitude===0?'at the same altitude':`${format(Math.abs(altitude))} meters ${altitude>0?'above':'below'} the launcher aircraft`}.`);
    el('missile-preview-note').textContent='Arrows show aircraft travel direction. Positions are scaled to fit; altitude is shown below. Missile release adjustments are applied when you simulate.';
  }
  function busy(value) {
    for(const id of ['missile-run','missile-run-preview'])if(el(id))el(id).disabled=value||!meta||!selectedMissiles.length;
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
      el('missile-preset-note').textContent='Custom setup · choosing an example replaces both aircraft states.';
    }
    changed();
  });
  el('missile-form').addEventListener('invalid',event=>{
    const details=event.target.closest('details');if(details)details.open=true;
  },true);
  function applyPreset(preset) {
    const states=JSON.parse(JSON.stringify(initial));
    if(preset==='head-on'){states.target.velocity=[-200,0,0];states.target.angles=[180,0,0];}
    if(preset==='crossing'){states.target.velocity=[0,0,200];states.target.angles=[90,0,0];}
    for(const role of ['launch','target']) {
      for(let i=0;i<3;i++)if(role!=='launch'||i===1)el(`missile-${role}-position-${i}`).value=states[role].position[i];
      syncMotion(role,states[role].velocity);
    }
    root.querySelectorAll('[data-preset]').forEach(button=>button.setAttribute('aria-pressed',String(button.dataset.preset===preset)));
    el('missile-preset-note').textContent={chase:'Target starts 4 km ahead, flying away at 720 km/h.','head-on':'Target starts 4 km ahead, flying toward the launcher at 720 km/h.',crossing:'Target starts 4 km ahead, flying across the launcher’s path at 720 km/h.'}[preset]+' Examples replace both aircraft states.';
    changed();
  }
  root.querySelectorAll('[data-preset]').forEach(button=>button.addEventListener('click',()=>applyPreset(button.dataset.preset)));
  el('missile-reset').addEventListener('click',()=>{
    selectedMissiles=[];renderSelected();busy(false);
    el('missile-search').value='';filterMissiles();el('missile-duration').value=initial.duration;
    applyPreset('chase');updateTiming();if(!meta)load();
  });
  for(const role of ['launch','target'])syncMotion(role);
  updatePreview();
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
            const p=status.progress;el('missile-status').textContent=`${completed.length}/${missiles.length} complete · ${missileName(id)} · `+(p?`${p.time_s.toFixed(1)} / ${p.duration_s.toFixed(1)} s…`:'Waiting for simulation…');
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
      el('missile-status').textContent=cancelRequested?`Comparison cancelled · ${completed.length}/${submitted.missiles.length} flights complete.`:failed.length?`${completed.length}/${submitted.missiles.length} flights complete.`:'Simulation complete.';
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
  function sceneFit(bounds) {
    const raw=bounds.map(([low,high])=>high-low),largest=Math.max(100,...raw);
    const spans=raw.map(span=>Math.max(100,largest*.08,span)*1.08);
    const eye=raw[0]>=raw[1]?{x:.15,y:-1.6,z:.65}:{x:1.6,y:.15,z:.65};
    const horizontal=Math.hypot(eye.x,eye.y),length=Math.hypot(horizontal,eye.z);
    const right=[-eye.y/horizontal,eye.x/horizontal,0];
    const up=[-eye.x*eye.z/(horizontal*length),-eye.y*eye.z/(horizontal*length),horizontal/length];
    const ratio=spans.map(span=>span/largest);
    const extent=basis=>basis.reduce((sum,value,i)=>sum+Math.abs(value)*ratio[i],0);
    const chart=el('missile-chart'),viewport=(chart.clientWidth||640)/(chart.clientHeight||360);
    const scale=Math.min(1.65*viewport/extent(right),1.45/extent(up));
    return {spans,aspectratio:Object.fromEntries(['x','y','z'].map((key,i)=>[key,ratio[i]*scale])),
      camera:{eye,up:{x:0,y:0,z:1},projection:{type:'orthographic'}}};
  }
  new ResizeObserver(()=>{
    if(el('missile-view').value==='3d'&&flightBounds&&el('missile-chart').data&&el('missile-chart').clientWidth)
      Plotly.relayout(el('missile-chart'),{'scene.aspectratio':sceneFit(flightBounds).aspectratio});
  }).observe(el('missile-chart'));
  const trackingLabels={observed:'Target observed',coasting:'Coasting on the previous track',inertial:'Inertial guidance · seeker not tracking',acquiring:'Acquiring / searching for target'};
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
  function timeCursor() {
    return [{type:'line',xref:'x',yref:'paper',x0:playbackTime,x1:playbackTime,y0:0,y1:1,line:{color:'#b8cada',width:1,dash:'dot'}}];
  }
  function resizeDataPlots() {
    if(el('missile-flight-data').open)for(const kind of ['propulsion','aero'])if(el(`missile-${kind}-chart`).data)Plotly.Plots.resize(el(`missile-${kind}-chart`));
  }
  const totalTime=()=>Math.max(...flights.map(f=>f.elapsed_s));
  function indexAt(rows,time,key='time_s') {
    let low=0,high=rows.length-1;
    while(low<high){const middle=Math.ceil((low+high)/2);if((key?rows[middle][key]:rows[middle])<=time)low=middle;else high=middle-1;}
    return low;
  }
  function inspectFlight(index) {
    focusIndex=index;result=flights[index];el('missile-result-select').value=index;
    el('missile-outcome').textContent=result.outcome.label;
    el('missile-closest').textContent=result.closest_approach.distance_m.toFixed(2)+' m';
    el('missile-elapsed').textContent=result.elapsed_s.toFixed(2)+' s';
    el('missile-model-details').replaceChildren(...result.model.limitations.map(text=>{const li=document.createElement('li');li.textContent=text;return li;}));
  }
  el('missile-result-select').addEventListener('change',()=>{inspectFlight(Number(el('missile-result-select').value));updateFrame(Number(el('missile-time').value));});
  async function renderDataPlots() {
    if(!result||!el('missile-flight-data').open)return;
    await Promise.all(['propulsion','aero'].map(async kind=>{
      const node=el(`missile-${kind}-chart`),key=el(`missile-${kind}-metric`).value;
      const metric=(kind==='propulsion'?propulsionMetrics:aeroMetrics)[key];
      el(`missile-${kind}-legend`).replaceChildren(...flights.map((flight,i)=>{
        const label=document.createElement('span');label.textContent=`━ ${i+1} · ${missileName(flight.scenario.missile)}`;
        label.style.color=resultColors[i];label.title=missileLabel(flight.scenario.missile);return label;
      }));
      const traces=flights.flatMap((flight,i)=>metric.series.map(([field,name,scale],s)=>({type:'scatter',mode:'lines',name:`${i+1} · ${missileName(flight.scenario.missile)}`,
        legendgroup:String(i),showlegend:s===0,
        x:flight.trajectory.map(r=>r.time_s),y:flight.trajectory.map(r=>Number.isFinite(r.telemetry?.[field])?r.telemetry[field]*scale:null),
        customdata:flight.trajectory.map(r=>[i,r.time_s]),connectgaps:false,line:{color:resultColors[i],width:2,dash:s?'dash':'solid'},
        hovertemplate:`${escapeHtml(missileLabel(flight.scenario.missile))}<br>%{x:.3f} s<br>${name}: %{y:.3f} ${metric.unit}<extra></extra>`})));
      await Plotly.react(node,traces,{paper_bgcolor:'#1d2024',plot_bgcolor:'#1d2024',font:{color:'#dce5f0',size:11},
        margin:{l:62,r:18,t:30,b:55},xaxis:{title:'Time after release · s',gridcolor:'#393d43',range:[0,totalTime()]},
        yaxis:{title:metric.unit,gridcolor:'#393d43',rangemode:'tozero'},showlegend:false,
        hovermode:'x unified',shapes:timeCursor(),uirevision:flightRevision+':'+key},
        {responsive:true,displaylogo:false,toImageButtonOptions:{filename:'missile-comparison-'+kind+'-'+key,format:'png',scale:2}});
      node.removeAllListeners('plotly_click');
      node.on('plotly_click',event=>{const point=event.points?.[0]?.customdata;if(Array.isArray(point)){pause();inspectFlight(point[0]);updateFrame(indexAt(timeline,point[1],null));}});
    }));
  }
  el('missile-flight-data').addEventListener('toggle',()=>{if(el('missile-flight-data').open)renderDataPlots().catch(error=>showError(error.message));});
  for(const kind of ['propulsion','aero'])el(`missile-${kind}-metric`).addEventListener('change',()=>renderDataPlots().catch(error=>showError(error.message)));
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
    const axis=(title,index)=>{const center=(bounds[index][0]+bounds[index][1])/2,span=fitted.spans[index];return {title:{text:index===2?'':title,font:{size:11}},range:[center-span/2,center+span/2],nticks:span<Math.max(...fitted.spans)*.2?3:6,tickfont:{size:10},tickangle:0,tickformat:',.0f',showspikes:false,gridcolor:'#393d43',zerolinecolor:'#455568',color:'#a9b8ca',backgroundcolor:'#1d2024'};};
    const sliceAxis=(worldAxis)=>({title:{text:axisNames[worldAxis]},gridcolor:'#393d43',zerolinecolor:'#455568',tickformat:',.0f',automargin:true});
    const layout={paper_bgcolor:'#1d2024',plot_bgcolor:'#1d2024',font:{color:'#dce5f0'},hoverlabel:{bgcolor:'#25282d',bordercolor:'#38c9d7',font:{color:'#e3edf7',size:12}},showlegend:false,uirevision:flightRevision+':'+view,
      ...(is3d?{margin:{l:0,r:0,t:0,b:0},annotations:[{text:'Y = altitude · m',xref:'paper',yref:'paper',x:.015,y:.985,xanchor:'left',yanchor:'top',showarrow:false,font:{size:10,color:'#a9b8ca'}}],scene:{xaxis:axis('X · m',0),yaxis:axis('Z · m',1),zaxis:axis('Altitude · m',2),aspectmode:'manual',aspectratio:fitted.aspectratio,camera}}:
        {margin:{l:70,r:25,t:20,b:60},xaxis:{...sliceAxis(viewAxes[view][0]),constrain:'domain'},yaxis:{...sliceAxis(viewAxes[view][1]),scaleanchor:'x',scaleratio:1},hovermode:'closest'})};
    el('missile-slice-note').hidden=is3d;
    el('missile-chart').setAttribute('aria-label',is3d?'Interactive 3D missile and target trajectories':`Interactive 2D missile and target trajectories: ${axisNames[viewAxes[view][0]]} / ${axisNames[viewAxes[view][1]]}`);
    await Plotly.newPlot(el('missile-chart'),[...flights.map((f,i)=>trace(f,'missile',resultColors[i])),trace(longest,'target','#ffa66b'),...flights.map((f,i)=>marker(missileName(f.scenario.missile)+' now',resultColors[i])),marker('Target now','#ffd0ac',false)],
      layout,
      {responsive:true,displaylogo:false,modeBarButtonsToRemove:['toImage']});
  }
  async function render() {
    flightRevision++;
    timeline=[...new Set(flights.flatMap(f=>f.trajectory.map(r=>r.time_s)))].sort((a,b)=>a-b);
    el('missile-view').disabled=true;
    try{await renderTrajectory();}finally{el('missile-view').disabled=false;}
    el('missile-legend').replaceChildren(...flights.map((flight,i)=>{
      const button=document.createElement('button');button.type='button';button.textContent=`━ ${i+1} · ${missileName(flight.scenario.missile)}`;button.style.color=resultColors[i];button.title=missileLabel(flight.scenario.missile);
      button.addEventListener('click',()=>{inspectFlight(i);updateFrame(Number(el('missile-time').value));});return button;
    }));
    const targetLegend=document.createElement('span');targetLegend.textContent='┄ Target';targetLegend.style.color='#ffa66b';targetLegend.style.opacity='.45';el('missile-legend').append(targetLegend);
    el('missile-time').max=timeline.length-1;el('missile-time').value=0;el('missile-time').disabled=false;el('missile-play').disabled=false;
    el('missile-result-select').replaceChildren(...flights.map((f,i)=>{const option=document.createElement('option');option.value=i;option.textContent=`${i+1} · ${missileLabel(f.scenario.missile)} · ${f.elapsed_s.toFixed(2)} s`;return option;}));
    el('missile-result-label').hidden=false;inspectFlight(focusIndex);
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
    el('missile-frame').textContent=`${missileName(result.scenario.missile)} · `+(finished?`${result.outcome.label} at ${result.elapsed_s.toFixed(2)} s`:row.kind==='proximity_event'?'Target proximity event':`Sample ${row.time_s.toFixed(2)} s · ${(row.speed_mps*3.6).toFixed(1)} km/h · ${trackingLabels[row.tracking_state]||'Tracking unavailable'}`);
    const targetPosition=result.scenario.target.position.map((v,i)=>v+result.scenario.target.velocity[i]*time);
    const positions=[...rows.map(r=>r.missile),targetPosition];
    const projected=Object.fromEntries(Object.entries(projectedPositions(positions)).map(([axis,values])=>[axis,values.map(v=>[v])]));
    const trails=flights.map(f=>f.trajectory.slice(0,indexAt(f.trajectory,time)+1));
    const trailPositions=trails.map(path=>projectedPositions(path.map(r=>r.missile)));
    const trailUpdate=Object.fromEntries(Object.keys(trailPositions[0]).map(axis=>[axis,trailPositions.map(p=>p[axis])]));
    Plotly.restyle(el('missile-chart'),{...trailUpdate,text:trails.map((path,i)=>path.map(r=>flightHover(r,'missile',flights[i])))},flights.map((_,i)=>i));
    Plotly.restyle(el('missile-chart'),{...projected,text:[...rows.map((r,i)=>[flightHover(r,'missile',flights[i])+(time>flights[i].elapsed_s?'<br>Flight ended · marker remains at the final position':'')]),[]]},positions.map((_,i)=>flights.length+1+i));
    if(el('missile-flight-data').open)for(const kind of ['propulsion','aero'])if(el(`missile-${kind}-chart`).data)Plotly.relayout(el(`missile-${kind}-chart`),{shapes:timeCursor()});
  }
  el('missile-time').addEventListener('input',()=>{pause();updateFrame(Number(el('missile-time').value));});
  el('missile-view').addEventListener('change',async()=>{
    if(!result)return;
    pause();const index=Number(el('missile-time').value);el('missile-view').disabled=true;
    try{await renderTrajectory();updateFrame(index);}catch(error){showError(error.message);}finally{el('missile-view').disabled=false;}
  });
  el('missile-view-reset').addEventListener('click',()=>{if(result)Plotly.relayout(el('missile-chart'),el('missile-view').value==='3d'?{'scene.camera':camera}:{'xaxis.autorange':true,'yaxis.autorange':true});});
  el('missile-play').addEventListener('click',()=>{
    if(playing){pause();return;}if(!result)return;
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
