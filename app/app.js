'use strict';
const chartUsable=p=>p.chart_usable??p.valid;
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
  const pageLoad=path==='/api/meta?refresh=1';
  const readOnly=(options.method||'GET').toUpperCase()==='GET';
  const attempts=pageLoad?300:readOnly?4:1;
  let restarting=false;
  for(let attempt=0;attempt<attempts;attempt++){
    let response,body;
    try{
      response=await fetch(apiUrl(path),options);
      // Proxy outage responses may be HTML or have no body at all.
      if([502,503,504].includes(response.status)&&!pageLoad){
        throw Object.assign(new Error('The calculation server is temporarily unavailable.'),{retryable:true});
      }
      if(restarting&&[502,503,504].includes(response.status)){
        await new Promise(resolve=>setTimeout(resolve,2000));continue;
      }
      body=await response.json();
      if([502,503,504].includes(response.status)&&!(pageLoad&&body.code==='server_restarting')){
        throw Object.assign(new Error('The calculation server is temporarily unavailable.'),{retryable:true});
      }
    }catch(cause){
      if(cause.name==='AbortError')throw cause;
      const retryable=cause.retryable||cause instanceof TypeError||[502,503,504].includes(response?.status);
      if(!retryable)throw cause;
      if(readOnly&&attempt<(restarting?attempts:4)-1){
        await new Promise(resolve=>setTimeout(resolve,Math.min(4000,1000*2**Math.min(attempt,2))));continue;
      }
      throw Object.assign(new Error(readOnly?
        'Connection to the calculation server was interrupted. Please try again.':
        'Connection to the calculation server was interrupted. The request may have reached the server; its outcome could not be confirmed.'),{retryable:true,cause});
    }
    if(response.status===503&&body.code==='server_restarting'&&pageLoad){
      restarting=true;$('footer-status').textContent='Loading updated calculator…';
    }else if(!response.ok)throw new Error(body.error||`Request failed (${response.status})`);
    else {
      if(restarting)$('footer-status').textContent='Calculator ready.';
      return body;
    }
    await new Promise(resolve=>setTimeout(resolve,2000));
  }
  throw new Error('Calculator updates took too long to load. Please refresh the page.');
}
function error(message) {$('error').textContent=message;$('error').hidden=!message;}
const conditionFields={altitude:'altitude_m',fuel:'fuel_percent','extra-mass':'extra_mass_kg',
  sweep:'sweep_percent',flaps:'flaps_percent',
  'propeller-pitch':'propeller_pitch_percent','oil-radiator':'oil_radiator_percent',radiator:'radiator_percent'};
// Fixed browser model; diagnostic API settings remain independently available.
const fixedSolverConditions={trim_solver_mode:'stationary',mach_curve_mode:'auto',experimental_instantaneous_aero:false,experimental_lateral_trim:'full',timestep_hz:48};
const conditionKeys=[...Object.values(conditionFields),...Object.keys(fixedSolverConditions),'throttle','afterburner','trim_mode','trim_limit',
  'fixed_trim','default_ammunition','ammunition_vehicle','loadout','stores_released','instructor','instructor_model','instructor_authority_mode','engine_control_mode','aircraft_trim_mode','torque_gyro','roll_leveling','structural_limits','turn_response_mode','manual_engine_controls','propeller_pitch_auto'];
const entryColors=['#38c9d7','#ffa66b','#b79aff','#91d477','#ee8eb6','#f0d367','#79a7fa','#cfb296'];
const entryName=e=>state.meta.aircraft[e.aircraft_id].name;
// Catalog names end in the internal id ("Bf 109 F-4 [bf-109f-4]"): the page shows it apart, hover text keeps it.
const cleanName=n=>String(n).replace(/\s*\[[^\]]*\]/,'');
const nameCode=n=>(String(n).match(/\[([^\]]*)\]/)||[])[1]||'';
const sortName=n=>cleanName(n).replace(/^[^\p{L}\p{N}]+/u,'');
const conditionsFrom=c=>Object.fromEntries(conditionKeys.map(k=>[k,c[k]??({experimental_instantaneous_aero:false,experimental_lateral_trim:'full',mach_curve_mode:'auto'}[k]??state.meta.defaults[k])]));
const requestConditions=c=>{const values={...c,...fixedSolverConditions};delete values.mach_curve_mode;return values;};
// Preserve actual result settings for display and stale-result comparisons.
// Editing or copying settings applies the fixed browser model to every entry.
const modeConditions=c=>{const values=conditionsFrom(c);return {...values,...fixedSolverConditions,instructor_model:'steady',engine_control_mode:'quasi_steady',aircraft_trim_mode:'discrete',instructor_authority_mode:'direct',turn_response_mode:'settled',torque_gyro:!values.instructor};};
const engineModelLabel=c=>c.engine_control_mode==='quasi_steady'?(c.controls?.automatic?.some(auto=>!auto)||c.manual_engine_controls&&!c.propeller_pitch_auto?'Quasi-steady · manual pitch':'Quasi-steady · ideal governor'):'Dynamic propulsion';
const aircraftModelLabel=c=>c.aircraft_trim_mode==='quasi_steady'?'Quasi-steady aircraft trim':'Discrete aircraft trim';
const experimentLabel=p=>{const e=p.model_experiments;return e?`${e.instantaneous_aerodynamics?'Instantaneous aerodynamics':'Retained aerodynamic history'} · ${{full:'Full trim',zero_sideslip:'Zero sideslip, full roll/yaw balance',symmetric:'Ideal symmetric, pitch trim only'}[e.lateral_trim]}`:'Retained model';};
const instructorAuthorityLabel=c=>c.instructor_authority_mode==='direct'?'Direct equilibrium (experimental)':'Original automatic trim';
const ammunitionFor=(aircraft,vehicle)=>aircraft?.ammunition?.variants?.[vehicle]||aircraft?.ammunition;
function normalizeManualControls(values,aircraft){
  const caps=aircraft.manual_controls||{};
  if(['jet','rocket'].includes(aircraft.propulsion))values.manual_engine_controls=false;
  if(!caps.propeller_pitch)values.propeller_pitch_auto=true;
  else if(values.manual_engine_controls&&!caps.propeller_auto)values.propeller_pitch_auto=false;
  if(!caps.oil_radiator)values.oil_radiator_percent=0;
  if(!caps.radiator)values.radiator_percent=0;
  return values;
}
const aircraftConditions=(name,c)=>{const values=modeConditions(c);const aircraft=state.meta.aircraft[name];const ammo=aircraft.ammunition;if(!ammo?.variants?.[values.ammunition_vehicle])values.ammunition_vehicle=ammo?.vehicle_id||'';if(!ammunitionFor(aircraft,values.ammunition_vehicle)?.supported)values.default_ammunition=false;values.flaps_percent=Math.max(0,Math.min(100,Math.round(values.flaps_percent||0)));if(!aircraft.has_flaps)values.flaps_percent=0;if(['jet','rocket'].includes(aircraft.propulsion))values.engine_control_mode='automatic';if(!aircraft.loadouts?.supported||!values.loadout){values.loadout='';values.stores_released=false;}return normalizeManualControls(values,aircraft);};
const flightModeLabel=c=>c.instructor!==c.torque_gyro?(c.instructor?'RB':'SB'):
  `Instructor ${c.instructor?'on':'off'} · torque/gyro ${c.torque_gyro?'on':'off'}`;
const resultConditions=a=>({torque_gyro:true,...(a.settings||{...state.data.settings,...state.data.settings.aircraft_settings?.[a.id]})});
const instructorEvidence=a=>resultConditions(a).instructor?
  a.instructor_approximation?.capability_status||'Experimental Instructor boundary; see the calculation method.':'';
function saveCondition(){
  if(!state.editing)return;
  state.conditions[state.editing]={...state.conditions[state.editing],
    ...Object.fromEntries(Object.entries(conditionFields).map(([id,k])=>[k,+$(id).value])),
    default_ammunition:$('default-ammunition').checked,ammunition_vehicle:$('ammunition-vehicle').value,
    manual_engine_controls:$('manual-engine-controls').checked,propeller_pitch_auto:$('propeller-pitch-auto').checked,
    ...fixedSolverConditions,structural_limits:$('flutter-on').checked,
    throttle:+$('throttle').value/100,afterburner:+$('throttle').value>100,
    instructor:$('flight-mode-rb').checked,torque_gyro:$('flight-mode-sb').checked};
  if(!state.meta.aircraft[state.entries.find(e=>e.id===state.editing).aircraft_id].has_flaps)state.conditions[state.editing].flaps_percent=0;
}
function showCondition(id){
  state.editing=id||null;
  const c=id?(state.conditions[id]??=modeConditions(state.meta.defaults)):state.meta.defaults;
  if(id&&!state.meta.aircraft[state.entries.find(e=>e.id===id).aircraft_id].has_flaps)c.flaps_percent=0;
  for(const [field,key] of Object.entries(conditionFields))$(field).value=c[key];
  $('default-ammunition').checked=!!c.default_ammunition;
  $('manual-engine-controls').checked=!!c.manual_engine_controls;
  $('propeller-pitch-auto').checked=c.propeller_pitch_auto!==false;
  const aircraft=id?state.meta.aircraft[state.entries.find(e=>e.id===id).aircraft_id]:null;
  const variants=aircraft?.ammunition?.variants;
  $('ammunition-vehicle-field').hidden=!variants||Object.keys(variants).length<2;
  $('ammunition-vehicle').innerHTML=variants?Object.keys(variants).map(v=>`<option value="${escapeText(v)}">${escapeText(aircraft.vehicle_names?.find(n=>n.vehicle_id===v)?.display_name||v)}</option>`).join(''):'';
  if(variants)$('ammunition-vehicle').value=c.ammunition_vehicle||aircraft.ammunition.vehicle_id;
  showAmmunition(aircraft);
  showLoadout();
  $('throttle').value=c.throttle*100;
  $('flight-mode-rb').checked=c.instructor;$('flight-mode-sb').checked=!c.instructor;
  $('flutter-on').checked=c.structural_limits!==false;$('flutter-off').checked=c.structural_limits===false;
  $('aircraft-condition').disabled=!id;
}
function showAmmunition(aircraft){
  const ammo=ammunitionFor(aircraft,$('ammunition-vehicle').value);
  $('default-ammunition').disabled=!ammo?.supported;
  if(!ammo?.supported)$('default-ammunition').checked=false;
  $('ammunition-help').textContent=ammo?.supported?
    [ammo.gun_rounds?`${fmt(ammo.gun_rounds,0)} rounds`:'',ammo.countermeasures?`${fmt(ammo.countermeasures,0)} countermeasures`:''].filter(Boolean).join(' · ')||'No ammunition in the default loadout.':
    'Default ammunition is unavailable for this aircraft.';
  $('ammunition-status').hidden=!ammo?.supported;
  $('ammunition-status').textContent=ammo?.supported?($('default-ammunition').checked?
    `Adds ${fmt(ammo.mass_kg,2)} kg to aircraft mass.`:'Ammunition is excluded.'):'';
}
function configEntries(c){
  if(c.entries)return c.entries.map(e=>({...e,settings:{torque_gyro:true,...e.settings}}));
  const entries=c.aircraft.map((name,i)=>({id:`entry_${i+1}`,aircraft_id:name,
    settings:conditionsFrom({engine_control_mode:'automatic',torque_gyro:true,...c,...c.aircraft_settings?.[name]})}));
  if(c.compare_instructor&&entries.length===1)return [false,true].map((enabled,i)=>({...entries[0],id:`entry_${i+1}`,settings:{...entries[0].settings,instructor:enabled}}));
  return entries;
}
const COPY_ICON='<svg viewBox="0 0 16 16" width="14" height="14" aria-hidden="true" fill="none" stroke="currentColor" stroke-width="1.4" stroke-linejoin="round"><rect x="5.5" y="5.5" width="8" height="8"/><path d="M10.5 5.5v-3h-8v8h3"/></svg>';
const REMOVE_ICON='<svg viewBox="0 0 16 16" width="14" height="14" aria-hidden="true" fill="none" stroke="currentColor" stroke-width="1.4" stroke-linejoin="round"><path d="M2.5 4.5h11M6 4.5v-2h4v2M4 4.5l.7 9h6.6l.7-9M6.7 7v4.5M9.3 7v4.5"/></svg>';
const EDIT_ICON='<svg viewBox="0 0 16 16" width="14" height="14" aria-hidden="true" fill="none" stroke="currentColor" stroke-width="1.4" stroke-linejoin="round"><path d="M10.5 2.5l3 3-8 8H2.5v-3z"/><path d="M9 4l3 3"/></svg>';
function renderEntries(){
  // The single settings editor lives in the settings popup while an entry is open (Leo, 2026-10-26); never clone form IDs.
  const editor=$('aircraft-condition');(state.editing?$('entry-dialog-body'):$('condition-home')).append(editor);
  $('selected-aircraft').innerHTML=state.entries.map((e,i)=>`<article class="aircraft-entry ${e.id===state.editing?'active':''}" data-entry="${escapeText(e.id)}" style="--entry-color:${entryColors[i]}">
    <div class="entry-heading"><span class="entry-number">${i+1}</span><strong>${escapeText(cleanName(entryName(e)))}</strong><code>${escapeText(nameCode(entryName(e))||e.aircraft_id)}</code></div>
    <div class="entry-actions">
    <button type="button" class="entry-edit entry-icon" data-action="configure" aria-haspopup="dialog" aria-label="Edit settings of entry ${i+1}" title="Edit settings">${EDIT_ICON}</button>
    <button type="button" class="entry-icon" data-action="duplicate" aria-label="Duplicate entry ${i+1}" title="Duplicate" ${state.entries.length>=8?'disabled':''}>${COPY_ICON}</button>
    <button type="button" class="entry-icon" data-action="remove" aria-label="Remove entry ${i+1}" title="Remove">${REMOVE_ICON}</button></div></article>`).join('');
  $('apply-settings-to-all').disabled=state.entries.length<2;
  $('apply-settings-status').textContent='';
}
function addEntry(name,condition){
  if(state.entries.length>=8)return;
  saveCondition();
  let id;do{id=`entry_${state.nextEntry++}`;}while(state.entries.some(e=>e.id===id));
  state.entries.push({id,aircraft_id:name});
  state.conditions[id]=structuredClone(aircraftConditions(name,condition||state.meta.defaults));
  updateDefaultContours();
  // A new card stays closed; its settings open with Configure (Leo, 2026-10-26).
  showCondition(state.editing);renderEntries();syncLabels();
}
function readConfig() {
  saveCondition();
  const grid=quality[$('quality').value] || [state.data?.settings.speed_samples||25,state.data?.settings.load_samples||19,state.data?.settings.sep_tolerance_mps||.5];
  return {...requestConditions(state.meta.defaults),aircraft:[...new Set(state.entries.map(e=>e.aircraft_id))],compare_instructor:false,aircraft_settings:{},
    entries:state.entries.map(e=>({...e,settings:requestConditions(state.conditions[e.id])})),
    speed_min_kmh:+$('speed-min').value,speed_max_kmh:+$('speed-max').value,max_load_g:null,
    speed_samples:grid[0],load_samples:grid[1],sampling:'adaptive',sep_tolerance_mps:grid[2],surface_resolution:601,
    sep_contour_levels_mps:[...state.contourLevels],enable_exports:$('enable-exports').checked};
}
function populate(c) {
  state.conditions={};state.editing=null;
  state.customContours=c!==state.meta.defaults&&Array.isArray(c.sep_contour_levels_mps);
  state.contourLevels=[...(c.sep_contour_levels_mps||[100,0,-100,-200,-400])];renderContourControls();
  state.entries=configEntries(c).map(e=>{state.conditions[e.id]=aircraftConditions(e.aircraft_id,e.settings);return {id:e.id,aircraft_id:e.aircraft_id};});
  updateDefaultContours();
  $('speed-min').value=c.speed_min_kmh;$('speed-max').value=c.speed_max_kmh;
  // Exports are always prepared (Leo, 2026-10-23); the switch stays in the page, hidden.
  $('enable-exports').checked=true;
  let selected=Object.keys(quality).find(k=>quality[k][2]===c.sep_tolerance_mps&&c.sampling==='adaptive') ||
    Object.keys(quality).find(k=>quality[k][0]===c.speed_samples&&quality[k][1]===c.load_samples&&quality[k][2]===c.sep_tolerance_mps);
  if(!selected){selected='custom';if(!$('quality').querySelector('[value=custom]'))$('quality').add(new Option('Saved sampling','custom'));}
  $('quality').value=selected;showCondition(state.entries[0]?.id);renderEntries();syncLabels();
}
function syncLabels(){
  $('fuel-label').textContent=$('fuel').value+'%';$('throttle-label').textContent=fmt(+$('throttle').value,0)+'%';
  $('sweep-label').textContent=$('sweep').value+'%';
  const aircraft=state.meta?.aircraft[state.entries.find(e=>e.id===state.editing)?.aircraft_id];
  const caps=aircraft?.manual_controls||{};
  $('manual-engine-field').hidden=!aircraft||['jet','rocket'].includes(aircraft.propulsion);
  $('manual-engine-settings').hidden=!$('manual-engine-controls').checked;
  $('propeller-pitch-field').hidden=!caps.propeller_pitch;
  $('propeller-pitch-auto-field').hidden=!caps.propeller_auto;
  if(caps.propeller_pitch&&!caps.propeller_auto&&$('manual-engine-controls').checked)$('propeller-pitch-auto').checked=false;
  $('propeller-pitch').disabled=$('propeller-pitch-auto').checked||!caps.propeller_pitch;
  $('propeller-pitch-label').textContent=$('propeller-pitch-auto').checked?'Automatic':$('propeller-pitch').value+'%';
  $('propeller-pitch-name').textContent=caps.pitch_kind==='RPM governor'?'Propeller RPM setting':caps.pitch_kind==='Blade angle'?'Propeller blade-angle setting':'Propeller control';
  $('propeller-pitch-help').textContent=caps.pitch_kind==='Blade angle'?'Direct blade angle: higher percentages select finer pitch and can cause over-rev.':caps.pitch_kind==='RPM governor'?'Selects a target within the aircraft’s governor RPM range. Blade pitch adjusts to hold that RPM until a pitch stop is reached.':`Control: ${caps.pitch_kind||'unavailable'}. Each propeller uses its own control type.`;
  for(const id of ['oil-radiator','radiator']){
    $(id+'-field').hidden=!caps[id.replace('-','_')];
    $(id+'-label').textContent=$(id).value+'%';
  }
  $('radiator-name').textContent=caps.common_radiator?'Radiator (oil + engine)':'Radiator';
  showAmmunition(aircraft);
  $('rocket-model-help').hidden=aircraft?.propulsion!=='rocket';
  $('flaps-field').hidden=!aircraft||!aircraft.has_flaps;
  $('flaps').disabled=!aircraft||!aircraft.has_flaps;
  if(aircraft&&!aircraft.has_flaps)$('flaps').value=0;
  $('flaps-label').textContent=$('flaps').value+'%';
  for(const button of $('flap-modes').children){
    const percent=aircraft?.flaps?.modes[button.dataset.flapMode];
    button.disabled=percent==null||!aircraft?.has_flaps;
    button.setAttribute('aria-pressed',String(!button.disabled&&+$('flaps').value===Math.round(percent)));
    button.title=button.disabled?'Not available on this aircraft':`${Math.round(percent)}% flaps`;
  }
  $('flap-limit-help').textContent=flapLimitLabel(aircraft?.flaps,+$('flaps').value);
  $('sweep-field').hidden=!aircraft?.has_sweep;
  syncAircraftMenu();
  // readConfig also stores the open entry's settings. No "settings have changed" notice (Leo, 2026-10-26).
  readConfig();
}
function flapLimitLabel(profile,percent){
  if(!profile?.available||percent<=0)return '';
  const fraction=percent/100;
  const rows=profile.destruction_mps;
  let damage=rows[rows.length-1][1];
  if(fraction<=rows[0][0])damage=rows[0][1];
  else for(let i=1;i<rows.length;i++)if(fraction<=rows[i][0]){
    const [x0,y0]=rows[i-1],[x1,y1]=rows[i];damage=y0+(y1-y0)*(fraction-x0)/(x1-x0);break;
  }
  const automatic=(row,extension=fraction)=>{
    let [x0,x1,y0,y1]=row;
    if(x1<x0)[x0,x1,y0,y1]=[x1,x0,y1,y0];
    if(extension>y0)return 0;
    return extension<=y1?Infinity:Math.max(0,x0+(y0-extension)*(x1-x0)/(y0-y1));
  };
  const ias=Math.min(damage*3.6,automatic(profile.ias_curve)),mach=automatic(profile.mach_curve);
  let travel=Infinity;
  if(profile.deployment_rate_curve){
    let [x0,x1,y0,y1]=profile.deployment_rate_curve;
    if(x1<x0)[x0,x1,y0,y1]=[x1,x0,y1,y0];
    if(y1<=0)travel=y0<=0?0:Math.max(0,x0+y0*(x1-x0)/(y0-y1));
  }
  const deployIas=Math.min(travel,automatic(profile.ias_curve)),deployMach=automatic(profile.mach_curve);
  const split=deployIas<ias-.01||deployMach<mach-1e-6;
  const deployment=split?` Deployment: ${[Number.isFinite(deployIas)?`${fmt(deployIas,0)} km/h IAS`:null,Number.isFinite(deployMach)?`Mach ${fmt(deployMach,3)}`:null].filter(Boolean).join(' · ')}. The dashed line marks the deployment limit; higher speeds require prior deployment.`:'';
  return `Flap hold limit: ${fmt(ias,0)} km/h IAS${Number.isFinite(mach)?` · Mach ${fmt(mach,3)}`:''}. The diagram uses actual extension; automatic controls may reduce a larger flap request to this position. The diagram stops at the first applicable speed limit.${deployment}`;
}
$('flap-modes').addEventListener('click',event=>{
  const button=event.target.closest('[data-flap-mode]');
  if(!button||button.disabled||!state.editing)return;
  const aircraft=state.meta.aircraft[state.entries.find(e=>e.id===state.editing).aircraft_id];
  $('flaps').value=Math.round(aircraft.flaps.modes[button.dataset.flapMode]);
  syncLabels();
});
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
  // By name, the leading nation symbol ignored, so an aircraft's variants sit together (the one without a symbol first).
  const symbol=a=>Number(cleanName(a.name)!==sortName(a.name));
  const entries=Object.entries(state.meta.aircraft).sort(([,a],[,b])=>Number(b.supported)-Number(a.supported)||
    sortName(a.name).localeCompare(sortName(b.name),undefined,{numeric:true})||symbol(a)-symbol(b)||a.name.localeCompare(b.name));
  // The type stays searchable but is not shown (Leo, 2026-10-23).
  const family=a=>({piston:'Piston propeller',turboprop:'Turboprop',mixed:'Propeller + jet',rocket:'Rocket'})[a.propulsion]||(a.has_sweep?'Variable sweep':'Jet');
  $('aircraft-list').innerHTML=entries.map(([id,a])=>`<div class="aircraft-option" data-search="${escapeText(id+' '+a.name+' '+family(a))}" data-supported="${!!a.supported}" title="${escapeText(a.reason||'')}"><span><strong>${escapeText(cleanName(a.name))}</strong><small><code>${escapeText(nameCode(a.name)||id)}</code>${escapeText(a.supported?(a.experimental&&a.propulsion!=='piston'?' · Experimental':''):' · '+(a.reason||'Model integration pending'))}</small></span><button type="button" data-add="${escapeText(id)}" aria-label="Add ${escapeText(a.name)}" ${a.supported?'':'disabled'}>Add</button></div>`).join('');
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
    updateDefaultContours();
    if(state.editing===id)showCondition(null);
  }else{
    showCondition(id);renderEntries();syncLabels();
    const index=state.entries.findIndex(e=>e.id===id);
    $('entry-dialog-title').textContent=`${index+1} · ${cleanName(entryName(entry))}`;
    $('entry-dialog').style.setProperty('--entry-color',entryColors[index]);
    $('entry-dialog').showModal();return;
  }
  renderEntries();syncLabels();
});
// Settings apply as they change; closing the popup (✕, Escape or a click outside) just puts them away.
$('entry-dialog-close').addEventListener('click',()=>$('entry-dialog').close());
$('entry-dialog').addEventListener('click',event=>{if(event.target===$('entry-dialog'))$('entry-dialog').close();});
$('entry-dialog').addEventListener('close',()=>{saveCondition();showCondition(null);renderEntries();syncLabels();});
$('apply-settings-to-all').addEventListener('click',()=>{
  if(!state.editing||state.entries.length<2)return;
  saveCondition();
  const settings=state.conditions[state.editing];
  const source=state.entries.find(e=>e.id===state.editing).aircraft_id;
  // Loadouts belong to one aircraft (and armament variant); other aircraft keep a clean loadout.
  for(const entry of state.entries)state.conditions[entry.id]=structuredClone(aircraftConditions(entry.aircraft_id,
    entry.aircraft_id===source?settings:{...settings,loadout:'',stores_released:false}));
  renderEntries();syncLabels();
  $('apply-settings-status').textContent=`Applied to all ${state.entries.length} aircraft.`;
});
function setRunning(value){state.running=value;$('calculate').disabled=value;$('cancel').hidden=!value;$('cancel').disabled=false;$('progress').hidden=!value;}
async function loadData(id,populateForm=false,preview=false){
  const sameResult=state.dataJob===id;
  const data=await api(`/api/jobs/${id}/${preview?'preview.json':'chart.json'}`);state.data=data;state.dataJob=id;
  if(!sameResult)state.hiddenVehicles.clear();
  if(populateForm)populate(data.settings);
  if(state.view!=='compare'&&!data.aircraft.some(a=>a.id===state.view))state.view='compare';
  $('chart-tabs').innerHTML='<button type="button" data-view="compare">Compare</button>'+data.aircraft.map(a=>`<button type="button" data-view="${escapeText(a.id)}" title="${escapeText(a.name)}">${escapeText(cleanName(a.name))}</button>`).join('');
  for(const button of document.querySelectorAll('[data-view]')){
    button.disabled=button.dataset.view!=='compare'&&!data.aircraft.some(a=>a.id===button.dataset.view);
    button.classList.toggle('active',button.dataset.view===state.view);
  }
  renderChart();syncLabels();
  updateContourAvailability();
  const rpmNotices=data.aircraft.filter(a=>a.over_rev&&(a.over_rev.crossings.length||a.over_rev.over_rev_at_minimum||a.over_rev.unresolved_intervals_kmh.length));
  $('over-rev-status').textContent=rpmNotices.map(a=>{
    const r=a.over_rev;
    const detail=r.crossings.length?r.crossings.map(c=>`${fmt(c.speed_kmh,1)} km/h TAS · calculation stops at over-rev`).join('; '):
      r.over_rev_at_minimum?'Already over-revving at the minimum plotted speed; this entry was not solved.':'RPM limit could not be resolved; calculation stops at the last verified speed.';
    return `${cleanName(a.name)}: ${detail}`;
  }).join(' ');
  $('over-rev-status').hidden=!rpmNotices.length;
  const unresolved=data.aircraft.reduce((sum,a)=>sum+(a.numerical_boundaries?.length||0),0);
  const gaps=data.aircraft.reduce((sum,a)=>sum+(a.numerical_gaps?.length||0),0);
  const incomplete=data.aircraft.reduce((sum,a)=>sum+(a.columns||[]).filter(c=>c.coverage_complete===false||c.search_unresolved).length,0);
  const speedGaps=data.aircraft.reduce((sum,a)=>sum+(a.interpolation?.unresolved_speed_intervals?.length||0),0);
  const machUncertain=data.aircraft.reduce((sum,a)=>sum+(a.interpolation?.certified_boundary_intervals||[]).filter(c=>c.native_mach_uncertainty?.length).length,0);
  $('footer-status').textContent=preview?'Preview · solved samples only · gaps, contours and endpoints are still being refined':`${incomplete||speedGaps?'Partial result':'Calculated'} in ${fmt(data.elapsed_s,1)} s${incomplete?' · '+incomplete+' incomplete speed columns':''}${speedGaps?' · '+speedGaps+' unresolved speed intervals':''}${unresolved?' · '+unresolved+' unresolved boundary points':''}${gaps?' · '+gaps+' equilibrium gaps':''}${machUncertain?' · '+machUncertain+' Mach transition intervals with numerical uncertainty (hover boundary)':''}${unresolved||gaps||incomplete?' · Enable Rejected for details':''}`;
  if(data.calculation_stopped)$('footer-status').textContent=`Stopped after ${fmt(data.elapsed_s,1)} s · ${data.aircraft.reduce((n,a)=>n+a.points.length,0)} retained samples · Enable exports for partial JSON or CSV. Unfinished regions remain unresolved.`;
  if(!sameResult){
    const hasPoints=data.aircraft.some(a=>a.points.some(p=>chartUsable(p)));
    $('point-title').textContent=hasPoints?'Select a point on the diagram':((data.stationary_full_chart||data.stationary_baseline_chart)?'No chartable candidates found':'No valid operating points');
    $('point-content').innerHTML=hasPoints?'':((data.stationary_full_chart||data.stationary_baseline_chart)?'<p class="muted">Search unresolved; absence has not been established.</p>':'<p class="muted">No valid operating points in this range.</p>');
    $('point-status').textContent='';
    // Details stay out of view until a point is clicked, unless they explain an empty chart.
    $('inspector').hidden=hasPoints;
  }
  if(!preview&&JSON.stringify([...state.contourLevels].sort((a,b)=>a-b))!==
      JSON.stringify([...(data.settings.sep_contour_levels_mps||[])].sort((a,b)=>a-b)))await refreshContours();
}
function rgba(hex,opacity){return `rgba(${parseInt(hex.slice(1,3),16)},${parseInt(hex.slice(3,5),16)},${parseInt(hex.slice(5,7),16)},${opacity})`;}
function updateDefaultContours(){
  if(state.customContours)return;
  const props=state.entries.length>0&&state.entries.every(e=>['piston','turboprop','mixed'].includes(state.meta.aircraft[e.aircraft_id].propulsion));
  state.contourLevels=props?[20,10,0,-10,-20,-40,-80]:[100,0,-100,-200,-400];
  renderContourControls();
}
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
function updateContourAvailability(){
  const missing=state.data?state.contourLevels.filter(level=>!supportedContourLevels().includes(level)):[];
  $('contour-status').textContent=missing.length?`Calculate to validate ${missing.map(v=>`${v>0?'+':''}${v}`).join(', ')} m/s.`:'';
  if(!state.data)return;
  for(const link of document.querySelectorAll('[data-export]')){
    const figure=link.dataset.export.startsWith('diagram.');
    const disabled=!$('enable-exports').checked||(state.data.preview&&(!state.data.calculation_stopped||figure))||(figure&&missing.length>0);
    const query=figure?'?levels='+encodeURIComponent(state.contourLevels.join(',')):'';
    link.href=disabled?'#':apiUrl(`/api/jobs/${state.dataJob}/${link.dataset.export}${query}`);
    link.download=`WT-EM-${state.data.calculation_stopped?'partial-':''}${state.data.aircraft.map(a=>a.id).join('-vs-')}-${link.dataset.export}`;
    link.setAttribute('aria-disabled',disabled?'true':'false');
    link.title=!$('enable-exports').checked?'Enable exports to download this result':'';
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
function envelopeEnd(aircraft,data){
  // The speed axis ends just past the fastest drawn envelope, within the requested range.
  const fastest=Math.max(0,...aircraft.flatMap(a=>(a.boundary||[]).map(p=>Number.isFinite(p.speed_kmh)?p.speed_kmh:0)));
  if(!fastest)return data.settings.speed_max_kmh;
  return Math.min(data.settings.speed_max_kmh,Math.max(Math.ceil((fastest+40)/50)*50,data.settings.speed_min_kmh+100));
}
function envelopeTop(aircraft){
  const peak=Math.max(0,...aircraft.flatMap(a=>[...(a.boundary||[]),...(a.points||[]).filter(chartUsable)].map(p=>Number.isFinite(p.turn_dps)?p.turn_dps:0)));
  return Math.max(10,Math.ceil(peak*1.05+1));
}
function contourLabelTraces(displayed,data){
  // One label per aircraft and SEP level, placed together in screen space (Leo, 2026-10-26): a label never
  // overlaps another, and avoids the envelopes and every other contour where its own line leaves room.
  const chart=$('chart'),width=Math.max(300,(chart.clientWidth||1000)-80),height=Math.max(200,(chart.clientHeight||700)-54-Math.max(40,displayed.length*22));
  const x0=data.settings.speed_min_kmh,x1=envelopeEnd(displayed,data),ymax=envelopeTop(displayed),cell=6;
  const px=(x,y)=>[(x-x0)/(x1-x0)*width,(1-y/ymax)*height];
  const lines=new Map(),shown=displayed.filter(a=>!state.hiddenVehicles.has(a.id));
  const mark=(key,xs,ys)=>{
    for(let i=1;i<xs.length;i++){
      if(![xs[i-1],ys[i-1],xs[i],ys[i]].every(Number.isFinite))continue;
      const [ax,ay]=px(xs[i-1],ys[i-1]),[bx,by]=px(xs[i],ys[i]),steps=Math.ceil(Math.hypot(bx-ax,by-ay)/(cell/2))+1;
      for(let k=0;k<=steps;k++){const c=Math.floor((ax+(bx-ax)*k/steps)/cell)+','+Math.floor((ay+(by-ay)*k/steps)/cell);
        if(!lines.has(c))lines.set(c,new Set());lines.get(c).add(key);}
    }
  };
  for(const a of shown){
    const edge=(a.boundary||[]).filter(q=>Number.isFinite(q.speed_kmh)&&Number.isFinite(q.turn_dps));
    mark(a.id+'|envelope',edge.map(q=>q.speed_kmh),edge.map(q=>q.turn_dps));
    for(const path of a.contours)if(state.contourLevels.includes(path.level))mark(a.id+'|'+path.level,path.x,path.y);
  }
  const placed=[],inside=(x,y)=>x>=4&&y>=4&&x<=width-4&&y<=height-4;
  const crossings=(box,own)=>{let n=0;
    for(let cx=Math.floor(box[0]/cell);cx<=Math.floor(box[2]/cell);cx++)for(let cy=Math.floor(box[1]/cell);cy<=Math.floor(box[3]/cell);cy++){
      const keys=lines.get(cx+','+cy);if(keys)for(const key of keys)if(key!==own){n++;break;}
    }return n;};
  const result=new Map(displayed.map(a=>[a.id,{x:[],y:[],labels:[],levels:[]}]));
  // Levels in turn across aircraft, so every aircraft gets its labels before crowding sets in.
  for(const level of state.contourLevels)for(const [index,a] of shown.entries()){
    const text=`SEP ${level>0?'+':''}${level}`,w=text.length*6.6+6,h=15,own=a.id+'|'+level;
    let best=null;
    for(const path of a.contours.filter(q=>q.level===level&&q.x.length>2)){
      const pts=path.x.map((x,i)=>Number.isFinite(x)&&Number.isFinite(path.y[i])?px(x,path.y[i]):null);
      const along=[0];for(let i=1;i<pts.length;i++)along.push(along[i-1]+(pts[i]&&pts[i-1]?Math.hypot(pts[i][0]-pts[i-1][0],pts[i][1]-pts[i-1][1]):0));
      const total=along.at(-1);if(total<w*1.5)continue;
      for(let f=.08;f<=.92;f+=.02){
        const target=total*f,i=along.findIndex(v=>v>=target);if(i<1||!pts[i]||!pts[i-1])continue;
        const t=(target-along[i-1])/((along[i]-along[i-1])||1),cx=pts[i-1][0]+t*(pts[i][0]-pts[i-1][0]),cy=pts[i-1][1]+t*(pts[i][1]-pts[i-1][1]);
        const box=[cx-w/2,cy-h/2,cx+w/2,cy+h/2];
        if(!inside(box[0],box[1])||!inside(box[2],box[3]))continue;
        if(placed.some(q=>box[0]<q[2]+4&&q[0]<box[2]+4&&box[1]<q[3]+3&&q[1]<box[3]+3))continue;
        const near=placed.reduce((m,q)=>Math.min(m,Math.hypot((q[0]+q[2])/2-cx,(q[1]+q[3])/2-cy)),Infinity);
        // Fewest crossed lines first, then away from other labels, then near this aircraft's usual spot.
        const score=crossings(box,own)*10+Math.max(0,60-near)/6+Math.abs(f-(index%2?.62:.42))*8;
        if(!best||score<best.score)best={score,cx,cy};
      }
    }
    if(!best)continue;
    placed.push([best.cx-w/2,best.cy-h/2,best.cx+w/2,best.cy+h/2]);
    const r=result.get(a.id);r.x.push(x0+best.cx/width*(x1-x0));r.y.push((1-best.cy/height)*ymax);r.labels.push(text);r.levels.push(level);
  }
  return displayed.map(a=>{const r=result.get(a.id);return {type:'scatter',mode:'markers+text',x:r.x,y:r.y,text:r.labels,customdata:r.levels,textposition:'middle center',
    textfont:{color:a.color,size:11,family:'WTSymbols, Segoe UI, sans-serif',shadow:'0 0 2px #000, 0 0 3px #000, 0 0 5px #000, 0 0 7px #000'},marker:{size:12,color:'rgba(0,0,0,0)'},
    meta:{id:a.id,kind:'contour-label'},legendgroup:a.id,showlegend:false,visible:state.hiddenVehicles.has(a.id)?'legendonly':true,
    hovertemplate:`${a.name}<br>SEP %{customdata} m/s<extra></extra>`};});
}
function projectedContourPoint(event,a,level){
  return projectedPoint(event,a.contours.filter(path=>path.level===level));
}
function projectedPoint(event,paths){
  // The clicked position on the drawn lines, projected in screen coordinates.
  const chart=$('chart'),rect=chart.getBoundingClientRect(),plot=chart._fullLayout._size;
  const [xmin,xmax]=chart._fullLayout.xaxis.range,[ymin,ymax]=chart._fullLayout.yaxis.range;
  const cursor=[(event.event.clientX-rect.left-plot.l)/plot.w,1-(event.event.clientY-rect.top-plot.t)/plot.h];
  let best=null;
  for(const path of paths)for(let i=1;i<path.x.length;i++){
    if(![path.x[i-1],path.y[i-1],path.x[i],path.y[i]].every(Number.isFinite))continue;
    const start=[(path.x[i-1]-xmin)/(xmax-xmin),(path.y[i-1]-ymin)/(ymax-ymin)];
    const end=[(path.x[i]-xmin)/(xmax-xmin),(path.y[i]-ymin)/(ymax-ymin)];
    const dx=end[0]-start[0],dy=end[1]-start[1],length=dx*dx+dy*dy;
    const t=length?Math.max(0,Math.min(1,((cursor[0]-start[0])*dx+(cursor[1]-start[1])*dy)/length)):0;
    const distance=(cursor[0]-start[0]-t*dx)**2+(cursor[1]-start[1]-t*dy)**2;
    if(!best||distance<best.distance)best={distance,speed:path.x[i-1]+t*(path.x[i]-path.x[i-1]),rate:path.y[i-1]+t*(path.y[i]-path.y[i-1])};
  }
  return best;
}
function speedBoundaryAnnotations(a,data){
  const labels=a.speed_boundary_labels||[];
  const threshold=data.settings.speed_min_kmh+.75*(data.settings.speed_max_kmh-data.settings.speed_min_kmh);
  return labels.map(p=>({name:a.id,x:p.speed_kmh,y:p.dashed ? .5 : p.turn_dps,xref:'x',yref:p.dashed?'paper':'y',
    text:escapeText(p.text).replace(/\n/g,'<br>'),textangle:-90,showarrow:false,
    xanchor:p.speed_kmh>threshold?'right':'left',yanchor:'middle',xshift:p.speed_kmh>threshold?-7:7,
    font:{color:a.color,size:12,family:'WTSymbols, Segoe UI, sans-serif'},bgcolor:'rgba(0,0,0,.85)',
    visible:!state.hiddenVehicles.has(a.id),
    hovertext:`${escapeText(a.name)}<br>${escapeText(p.text).replace(/\n/g,'<br>')}<br>${fmt(p.speed_kmh,1)} km/h TAS${p.description?'<br>'+escapeText(p.description):''}`}));
}
function nearestSolved(a,speed,rate,showRejected){
  // Nearest on screen: distances in pixels of the current view.
  const layout=$('chart')._fullLayout,[x0,x1]=layout.xaxis.range,[y0,y1]=layout.yaxis.range;
  const sx=layout._size.w/(x1-x0),sy=layout._size.h/(y1-y0);
  let best=null;
  for(const point of [...a.points,...(a.sustained||[])]){
    if(!chartUsable(point)&&!showRejected)continue;
    const d=((point.speed_kmh-speed)*sx)**2+((point.turn_dps-rate)*sy)**2;
    if(!best||d<best.d)best={point,d};
  }
  return best?.point;
}
async function solveAt(a,position,origin,showRejected){
  // Lines are interpolated between solved states: solve the full model at the clicked speed and load.
  const sequence=state.inspection=(state.inspection||0)+1,job=state.dataJob;
  const load=Math.hypot(1,(position.speed/3.6)*(position.rate*Math.PI/180)/9.8100004196167);
  const clicked={...origin,speed:position.speed,rate:position.rate};
  $('inspector').hidden=false;
  $('point-title').textContent=`${cleanName(a.name)} · ${fmt(position.speed,0)} km/h · ${fmt(position.rate,2)} °/s`;
  $('point-status').innerHTML='<span class="point-pill">Solving the clicked point…</span>';
  $('point-content').innerHTML='';
  let point;
  try{point=await api(`/api/jobs/${job}/solve/${encodeURIComponent(a.id)}?speed_kmh=${position.speed}&load_g=${load}`);}
  catch(e){
    if(sequence!==state.inspection||job!==state.dataJob)return;
    // A server without point solves: the nearest solved state instead.
    const nearest=nearestSolved(a,position.speed,position.rate,showRejected);
    if(nearest)return inspect(a,nearest,{...clicked,nearest:true});
    error(e.message);return;
  }
  if(sequence!==state.inspection||job!==state.dataJob)return;
  return inspect(a,point,clicked);
}
const DATA_ICONS={
  csv:{width:24,height:24,path:'M3 4h18v3H3z M3 10h8v3H3z M13 10h8v3h-8z M3 16h8v3H3z M13 16h8v3h-8z'},
  json:{width:24,height:24,path:'M9 3H7.5C6.1 3 5 4.1 5 5.5V9c0 .8-.7 1.5-1.5 1.5H3v3h.5c.8 0 1.5.7 1.5 1.5v3.5C5 19.9 6.1 21 7.5 21H9v-2H7.5c-.3 0-.5-.2-.5-.5V15c0-1.2-.6-2.3-1.5-3 .9-.7 1.5-1.8 1.5-3V5.5c0-.3.2-.5.5-.5H9z M15 3h1.5C17.9 3 19 4.1 19 5.5V9c0 .8.7 1.5 1.5 1.5h.5v3h-.5c-.8 0-1.5.7-1.5 1.5v3.5c0 1.4-1.1 2.5-2.5 2.5H15v-2h1.5c.3 0 .5-.2.5-.5V15c0-1.2.6-2.3 1.5-3-.9-.7-1.5-1.8-1.5-3V5.5c0-.3-.2-.5-.5-.5H15z'}};
function downloadExport(name){
  const link=document.querySelector(`[data-export="${name}"]`);
  if(link.getAttribute('aria-disabled')==='true')return error('Downloads are available once the calculation has finished.');
  link.click();
}
function renderChart(){
  if(!state.data)return;
  const data=state.data,displayed=data.aircraft.filter(a=>state.view==='compare'||a.id===state.view),traces=[];
  const showSamples=$('show-samples').checked,showRejected=$('show-rejected').checked;
  // Analytic guides need their own drawing grid; trim columns can be sparse.
  const guideMin=data.settings.speed_min_kmh,guideMax=data.settings.speed_max_kmh;
  const guideSpeeds=Array.from({length:1201},(_,i)=>guideMin*Math.pow(guideMax/guideMin,i/1200));
  for(const n of [2,4,6,9,12,16]){
    if(n>(data.plot_max_load_g||data.settings.max_load_g))continue;
    traces.push({type:'scatter',mode:'lines',x:guideSpeeds,y:guideSpeeds.map(v=>180/Math.PI*9.8100004196167*Math.sqrt(n*n-1)/(v/3.6)),
      line:{color:'rgba(240,240,250,.12)',width:1,simplify:false},hoverinfo:'skip',showlegend:false});
  }
  for(const a of displayed){
    for(const level of [...new Set(a.contours.map(c=>c.level))]){
      if(!state.contourLevels.includes(level))continue;
      if(level===0&&a.sustained?.length>1)continue;
      const x=[],y=[];for(const path of a.contours.filter(c=>c.level===level)){x.push(...path.x,null);y.push(...path.y,null);}
      traces.push({type:'scatter',mode:'lines',x,y,name:`${a.name} · SEP ${level}`,meta:{id:a.id,kind:'contour',level},legendgroup:a.id,visible:state.hiddenVehicles.has(a.id)?'legendonly':true,
        line:{color:rgba(a.color,level===0?.95:displayed.length===1?.75:.46),width:level===0?3.6:2.2,dash:'dash'},
        text:x.map((v,i)=>turnRadius(v,y[i])),
        hovertemplate:`${a.name}<br>SEP ${level>0?'+':''}${level} m/s<br>%{x:.0f} km/h · %{y:.2f}°/s<br>Turn radius %{text}<extra></extra>`,showlegend:false});
    }
    // SEP jump lines: where SEP jumps along the turn (contours end at them); the hover names the cause
    for(const j of a.sep_jumps||[]){
      const why={supercharger:'Supercharger has no valid stage',propeller:'Propeller changes state'}[j.cause]||'SEP is discontinuous here';
      // A faint band under a dash-dot line, so jumps never read as contours (dashed) or the envelope (solid).
      traces.push({type:'scatter',mode:'lines',x:j.speed_kmh,y:j.turn_dps,meta:{id:a.id,kind:'sep jump band'},legendgroup:a.id,visible:state.hiddenVehicles.has(a.id)?'legendonly':true,
        line:{color:rgba(a.color,.22),width:11},hoverinfo:'skip',showlegend:false});
      traces.push({type:'scatter',mode:'lines',x:j.speed_kmh,y:j.turn_dps,name:`${a.name} · SEP jump`,meta:{id:a.id,kind:'sep jump'},legendgroup:a.id,visible:state.hiddenVehicles.has(a.id)?'legendonly':true,
        line:{color:rgba(a.color,.95),width:2,dash:'12px,4px,2px,4px'},
        customdata:j.speed_kmh.map((v,i)=>[fmt(j.sep_before_mps[i],1),fmt(j.sep_after_mps[i],1)]),
        hovertemplate:`${a.name}<br>SEP jump %{customdata[0]} → %{customdata[1]} m/s<br>${why}<br>%{x:.0f} km/h · %{y:.2f}°/s<extra></extra>`,showlegend:false});
    }
    traces.push({type:'scatter',mode:'lines',x:a.boundary.map(p=>p.speed_kmh),y:a.boundary.map(p=>p.turn_dps),
      name:cleanName(a.name),meta:{id:a.id,kind:'boundary'},legendgroup:a.id,showlegend:true,visible:state.hiddenVehicles.has(a.id)?'legendonly':true,connectgaps:false,cliponaxis:false,line:{color:rgba(a.color,.95),width:2.8,dash:'solid'},
      customdata:a.boundary.map(p=>[Number.isFinite(p.alpha_deg)?fmt(p.alpha_deg,2)+'°':'—',turnRadius(p.speed_kmh,p.turn_dps),p.at_plot_ceiling?'Search limit; aircraft limit not established':p.limit_label||p.edge_kind||'Calculated envelope; limit unavailable in this saved result']),
      hovertemplate:`${a.name}<br>%{customdata[2]}<br>AoA %{customdata[0]}<br>Radius %{customdata[1]}<br>Rate %{y:.2f}°/s<br>Speed %{x:.0f} km/h<extra></extra>`});
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
        name:a.name+' SEP 0',meta:{id:a.id,kind:'contour',level:0},legendgroup:a.id,showlegend:false,visible:state.hiddenVehicles.has(a.id)?'legendonly':true,connectgaps:false,line:{color:a.color,width:4,dash:'dash'},marker:{size:4,color:a.color},
        customdata:curve?null:data.speeds_kmh.map(v=>roots.has(v)?[a.id,'root',roots.get(v).i]:null),
        text:(curve?.x||data.speeds_kmh).map((v,i)=>{const rate=curve?curve.y[i]:roots.get(v)?.p.turn_dps;return turnRadius(v,rate);}),
        hovertemplate:`${a.name} · ${data.preview?'preview contour':'refined'} SEP 0 m/s<br>%{x:.0f} km/h · %{y:.2f}°/s<br>Turn radius %{text}<extra></extra>`});
    }
    const valid=a.points.map((p,i)=>({p,i})).filter(r=>chartUsable(r.p));
    if(showSamples)traces.push({type:'scatter',mode:'markers',x:valid.map(r=>r.p.speed_kmh),y:valid.map(r=>r.p.turn_dps),
      marker:{size:showSamples?5:9,color:showSamples?rgba(a.color,.5):'rgba(0,0,0,0)'},name:a.name,meta:a.id,legendgroup:a.id,showlegend:false,visible:state.hiddenVehicles.has(a.id)?'legendonly':true,
      customdata:valid.map(r=>[a.id,'grid',r.i]),text:valid.map(r=>`${a.name} · ${fmt(r.p.load_g,2)} g<br>SEP ${fmt(r.p.ps_mps,1)} m/s`),
      hovertemplate:'%{text}<br>%{x:.0f} km/h · %{y:.2f}°/s<extra></extra>'});
    if(showRejected){
      const rejected=a.points.map((p,i)=>({p,i})).filter(r=>!chartUsable(r.p));
      traces.push({type:'scatter',mode:'markers',x:rejected.map(r=>r.p.speed_kmh),y:rejected.map(r=>r.p.turn_dps),
        marker:{size:5,symbol:'x',color:rgba(a.color,.33)},name:a.name+((data.stationary_full_chart||data.stationary_baseline_chart)?' unresolved':' rejected'),meta:a.id,legendgroup:a.id,showlegend:false,visible:state.hiddenVehicles.has(a.id)?'legendonly':true,
        customdata:rejected.map(r=>[a.id,'grid',r.i]),text:rejected.map(r=>((data.stationary_full_chart||data.stationary_baseline_chart)?[r.p.chart_status,...r.p.reasons]:r.p.reasons).filter(Boolean).join(', ')),
        hovertemplate:`${a.name} · ${(data.stationary_full_chart||data.stationary_baseline_chart)?'unresolved':'rejected'}<br>%{text}<br>%{x:.0f} km/h · %{y:.2f}°/s<extra></extra>`});
    }
  }
  traces.push(...contourLabelTraces(displayed,data));
  const ymax=envelopeTop(displayed);
  const layout={paper_bgcolor:'#000',plot_bgcolor:'#000',font:{family:'WTSymbols, D-DIN, Bahnschrift, DIN Alternate, Segoe UI, sans-serif',color:'#b9b9c4',size:12},
    annotations:displayed.flatMap(a=>speedBoundaryAnnotations(a,data)),
    shapes:displayed.flatMap(a=>(a.speed_boundary_labels||[]).filter(p=>p.dashed).map(p=>({
      name:a.id,type:'line',xref:'x',yref:'paper',x0:p.speed_kmh,x1:p.speed_kmh,y0:0,y1:1,
      line:{color:rgba(a.color,.7),width:1.4,dash:'dash'},visible:!state.hiddenVehicles.has(a.id)}))),
    margin:{l:58,r:22,t:Math.max(40,displayed.length*22),b:54},hovermode:'closest',hoverlabel:{bgcolor:'#0c0c0f',bordercolor:'rgba(240,240,250,.35)',font:{color:'#eff7ff',size:12}},
    xaxis:{title:{text:'True airspeed (km/h)',standoff:8,font:{size:13}},range:[data.settings.speed_min_kmh,envelopeEnd(displayed,data)],gridcolor:'rgba(240,240,250,.09)',zeroline:false,dtick:100,tickfont:{size:12},constrain:'domain'},
    yaxis:{title:{text:'Turn rate (°/s)',standoff:8,font:{size:13}},range:[0,ymax],gridcolor:'rgba(240,240,250,.09)',zeroline:false,dtick:5,tickfont:{size:12}},
    legend:{orientation:'h',x:0,y:1.02,yanchor:'bottom',font:{size:12},bgcolor:'rgba(0,0,0,0)',traceorder:'normal',groupclick:'togglegroup',itemdoubleclick:false},
    uirevision:state.dataJob+'-'+state.view+'-'+showRejected,dragmode:'pan',modebar:{bgcolor:'rgba(0,0,0,0)',color:'#8a8a96',activecolor:'#f0f0fa'}};
  if(!$('chart').classList.contains('js-plotly-plot'))$('chart').innerHTML='';
  Plotly.react('chart',traces,layout,{responsive:true,displaylogo:false,scrollZoom:true,modeBarButtonsToRemove:['select2d','lasso2d'],
    // Downloads sit in the chart's own toolbar: Plotly's PNG of the current view, then the result's data files.
    toImageButtonOptions:{format:'png',scale:2,filename:'WT-EM-'+data.aircraft.map(a=>a.aircraft_id||a.id).join('-vs-')},
    modeBarButtonsToAdd:[{name:'Download samples (CSV)',icon:DATA_ICONS.csv,click:()=>downloadExport('samples.csv')},
                         {name:'Download full result (JSON)',icon:DATA_ICONS.json,click:()=>downloadExport('data.json')}]});
  $('chart').removeAllListeners?.('plotly_legendclick');
  $('chart').on('plotly_legendclick',event=>{
    const group=event.data[event.curveNumber]?.legendgroup;if(!group)return false;
    const hidden=state.hiddenVehicles.has(group);
    if(hidden)state.hiddenVehicles.delete(group);else state.hiddenVehicles.add(group);
    const indices=event.data.flatMap((trace,i)=>trace.legendgroup===group?[i]:[]);
    Plotly.restyle('chart',{visible:hidden?true:'legendonly'},indices);
    const visibility={};
    for(const key of ['annotations','shapes'])$('chart').layout[key]?.forEach((item,i)=>{
      if(item.name===group)visibility[`${key}[${i}].visible`]=hidden;
    });
    if(Object.keys(visibility).length)Plotly.relayout('chart',visibility);
    // Labels are placed against the shown aircraft only, so lay them out again.
    const labelIndices=event.data.flatMap((trace,i)=>trace.meta?.kind==='contour-label'?[i]:[]);
    const shownAircraft=state.data.aircraft.filter(a=>labelIndices.some(i=>event.data[i].meta.id===a.id));
    const placedLabels=new Map(contourLabelTraces(shownAircraft,state.data).map(t=>[t.meta.id,t]));
    const fresh=labelIndices.map(i=>placedLabels.get(event.data[i].meta.id));
    Plotly.restyle('chart',{x:fresh.map(t=>t.x),y:fresh.map(t=>t.y),text:fresh.map(t=>t.text),customdata:fresh.map(t=>t.customdata)},labelIndices);
    return false;
  });
  $('chart').removeAllListeners?.('plotly_click');
  $('chart').on('plotly_click', event=>{
    const p=event.points?.[0];if(!p)return;
    if(p.data.meta?.kind==='contour'||p.data.meta?.kind==='contour-label'){
      const a=data.aircraft.find(a=>a.id===p.data.meta.id);
      const level=p.data.meta.kind==='contour'?p.data.meta.level:p.customdata;
      const position=a&&projectedContourPoint(event,a,level);
      if(position)return solveAt(a,position,{level},showRejected);
    }
    if(p.data.meta?.kind==='boundary'){
      const a=data.aircraft.find(a=>a.id===p.data.meta.id);
      const position=a&&projectedPoint(event,[{x:a.boundary.map(q=>q.speed_kmh),y:a.boundary.map(q=>q.turn_dps)}]);
      if(position)return solveAt(a,position,{boundary:true},showRejected);
    }
    if(Array.isArray(p.customdata)&&p.customdata.length===3){
      const [id,kind,index]=p.customdata,a=data.aircraft.find(a=>a.id===id);if(a)return inspect(a,(kind==='root'?a.sustained:a.points)[index]);
    }
    const candidates=displayed.filter(a=>!p.data.meta||a.id===(p.data.meta.id||p.data.meta));let best=null;
    for(const a of candidates)for(const point of a.points){
      if(!chartUsable(point)&&!showRejected)continue;
      const d=((point.speed_kmh-p.x)/(data.settings.speed_max_kmh-data.settings.speed_min_kmh))**2+((point.turn_dps-p.y)/ymax)**2;
      if(!best||d<best.d)best={a,point,d};
    }
    if(best)inspect(best.a,best.point);
  });
}
const KGF=9.80665,METRIC_HP=735.49875;
function propellerPower(a,p){
  // Shaft power per engine (metric hp); propeller efficiency = thrust x airspeed / shaft power, for pure propeller aircraft.
  const power=p.propulsion.shaft_power_kw;if(!power?.length)return [];
  const rows=[['Engine power',power.map(kw=>fmt(kw*1000/METRIC_HP,0)).join(' / ')+' hp']];
  const total=power.reduce((sum,kw)=>sum+kw,0)*1000;
  if(['piston','turboprop'].includes(state.meta.aircraft[a.aircraft_id||a.id]?.propulsion)&&total>1)
    rows.push(['Propeller efficiency',fmt(p.engine_force_n[0]*p.speed_kmh/3.6/total*100,1)+'%']);
  return rows;
}
async function inspect(a,p,clicked){
  const sequence=state.inspection=(state.inspection||0)+1;
  if(!p.component_forces&&p.detail_index!=null){
    try {p=await api(`/api/jobs/${state.dataJob}/point/${encodeURIComponent(a.id)}/${p.detail_index}`);}
    catch(e){error(e.message);return;}
  }
  if(sequence!==state.inspection)return;
  $('inspector').hidden=false;
  $('point-title').textContent=`${cleanName(a.name)} · ${fmt(p.speed_kmh,0)} km/h · ${fmt(p.turn_dps,2)} °/s`;
  $('point-status').innerHTML=`<span class="point-pill ${chartUsable(p)?'':'rejected'}">${clicked?.nearest?'Nearest solved point':chartUsable(p)?'Converged & within bounds':p.status==='unresolved'?'Unresolved':'Rejected'}</span>`;
  const where=clicked&&`${clicked.boundary?'the envelope':`the SEP ${clicked.level>0?'+':''}${clicked.level} m/s contour`} at ${fmt(clicked.speed,1)} km/h · ${fmt(clicked.rate,2)} °/s`;
  const note=clicked?`<div class="point-note">Clicked ${where}. ${clicked.nearest?'The details are for the nearest solved point; this server cannot solve the clicked point.':'This state is solved there with the full model.'}</div>`:'';
  // A third row entry hides that row on the page; it stays in the markup (Leo, 2026-10-23).
  const dl=rows=>'<dl>'+rows.map(([k,v,hide])=>`<dt${hide?' hidden':''}>${k}</dt><dd${hide?' hidden':''}>${v}</dd>`).join('')+'</dl>';
  if(p.not_evaluated){$('point-content').innerHTML=dl([['Not calculated',escapeText(p.reasons.join(', '))]])+note;return;}
  if(!chartUsable(p)&&p.status==='unresolved'&&!p.component_forces){$('point-content').innerHTML=dl([['Calculation',escapeText(p.search_unresolved||p.chart_status||'No numerical candidate established')]])+note;return;}
  const forces=Object.entries(p.component_forces).filter(([k])=>!['parasite','chute'].includes(k));
  const names={left_wing:'Left wing',right_wing:'Right wing',left_hstab:(a.aircraft_id||a.id)==='saab_jas39c'?'Left canard':'Left tail',right_hstab:(a.aircraft_id||a.id)==='saab_jas39c'?'Right canard':'Right tail',vstab:'Vertical tail',fuselage:'Fuselage'};
  $('point-content').innerHTML=`<div class="point-grid"><div><h3>FLIGHT & ENERGY</h3>${dl([
    ['SEP',fmt(p.ps_mps,2)+' m/s'],
    ['Aircraft model',aircraftModelLabel(p),true],
    ['Model experiment',experimentLabel(p),true],
    ['Turn rate',fmt(p.turn_dps,3)+' °/s'],['Turn radius',turnRadius(p.speed_kmh,p.turn_dps)],
    ['Angle of attack',fmt(p.alpha_deg,3)+'°'],['Bank',fmt(p.bank_deg,3)+'°'],['Longitudinal IAS',fmt(p.ias_kmh,0)+' km/h'],['Mach',fmt(p.mach,3)],
    ['Thrust',fmt(p.engine_force_n[0]/KGF,0)+' kgf'],
    ...(p.propulsion?[
      ...propellerPower(a,p),
      ['Engine RPM',p.propulsion.engine_rpm.map(v=>fmt(v,0)).join(' / ')],
      ['Propeller pitch',p.propulsion.propeller_pitch_deg.map((v,i)=>fmt(v,2)+'°'+(Number.isFinite(p.propulsion.propeller_pitch_percent?.[i])?' / '+fmt(p.propulsion.propeller_pitch_percent[i],0)+'%':'')).join(' · ')],
      ['Propeller control',p.propulsion.controls.automatic.map((auto,i)=>auto?'Automatic':fmt(p.propulsion.controls.commands[i]/255*100,1)+'% manual').join(' / ')],
      ['Compressor stage',(p.propulsion.compressor_stages??p.propulsion.controls.gears).map(v=>v+1).join(' / ')],
      ['RPM limit',p.propulsion.engine_rpm_threshold.map(v=>fmt(v,0)).join(' / ')],
      ['Over-rev',p.propulsion.rpm_threshold_exceeded_engines.length?'Engine '+p.propulsion.rpm_threshold_exceeded_engines.map(i=>i+1).join(' / '):'No']]:[]),
    ...(p.sweep_percent==null?[]:[['Fixed wing sweep',fmt(p.sweep_percent,0)+'%'],['Available sweep',p.sweep_available_percent.map(x=>fmt(x,1)).join('–')+'%']])])}</div><div hidden><h3>LIMITS & CONVERGENCE</h3>${dl([
    ['Control authority margin',fmt(p.authority_margin*100,2)+'%'],
    ...(p.history_closure_required===false?[['Aerodynamic history','Not part of this experimental model']]:[]),
    ...(p.symmetry_projection?[['Roll/yaw balance','Assumed ideal support; physical control authority not established']]:[]),
    ['Flutter',resultConditions(a).structural_limits===false?'Off · structural limits disabled':'On · structural limits enforced'],
    ...(instructorEvidence(a)?[['Instructor evidence',escapeText(instructorEvidence(a))]]:[]),
    ...(resultConditions(a).instructor?[['Instructor control authority',instructorAuthorityLabel(resultConditions(a))]]:[]),
    ...(p.prolonged_pull?[['Boundary definition','Stationary full-pull reference · selected power'],['Full-pull check',escapeText(p.prolonged_pull.status)],['Full capability',escapeText(p.prolonged_pull.capability_status||'Not certified by the stationary check')],['Native overload timer',fmt(p.prolonged_pull.overload_timer,4)]]:[]),
    ...(p.instructor_enabled&&p.instructor?.history_independent?[['Instructor','Steady AoA schedule'],['Effective upper wing-AoA limit',fmt(p.instructor.effective_angle_limits_deg[1],3)+'°'],['Adjusted wing AoA',p.instructor.adjusted_wing_angles_deg.map(v=>fmt(v,3)+'°').join(' / ')],['Instructor limiting constraint',escapeText(p.instructor.limiting)]]:[]),
    ...(p.instructor_enabled&&!p.instructor?.history_independent?[['Instructor',p.instructor?.delivered_pitch!==undefined?'Experimental recovered pitch limiter':escapeText(p.instructor?.model??'Experimental Instructor')],...(p.instructor?.delivered_pitch!==undefined?[['Required elevator',fmt(p.instructor.required_pitch*100,2)+'%'],[p.instructor.required_pilot?'Delivered elevator':'Full-pull available elevator',fmt(p.instructor.delivered_pitch*100,2)+'%']]:[]),['Instructor limiting constraint',escapeText(p.instructor?.limiting??'Not evaluated: aircraft balance or physical limit rejected this point')],...(p.instructor?.angle_limits_deg?[['Instructor upper wing-AoA target',fmt(p.instructor.angle_limits_deg[1],3)+'°'],['Adjusted wing AoA',p.instructor.adjusted_wing_angles_deg?.map(v=>fmt(v,3)+'°').join(' / ')??'—']]:[])]:[]),
    ['Force residual',p.force_error_g.toExponential(2)+' g'],['Angular residual',p.angular_error_rad_s2.toExponential(2)+' rad/s²'],
    [p.aircraft_trim_mode==='quasi_steady'?'Aerodynamic state residual':'History residual',p.history_error.toExponential(2)],['Stall margin',fmt(p.stall_margin_deg,2)+'°'],
    ...(p.propulsion?[
      ['Propulsion state',p.propulsion.engine_control_mode==='quasi_steady'?(p.propulsion.converged?'Steady torque and flow equilibrium':'Unresolved'):p.propulsion.period_frames?`${p.propulsion.period_frames}-frame cycle`:p.propulsion.stationarity?`${p.propulsion.sample_frames}-frame output window`:'Unresolved'],
      ['Aircraft trim',p.propulsion.phase_check?.checked?(p.propulsion.engine_control_mode==='quasi_steady'?'Steady aircraft forces checked':'Complete aircraft outputs averaged'):'Not certified'],
      ['Engine model',engineModelLabel(p.propulsion)],
      ...(p.propulsion.engine_control_mode==='quasi_steady'?[
        ['Governor',p.propulsion.stationarity.governor_status.join(' / ')],
        ['Model assumption','Steady propulsion; fluctuations and transient peak loads omitted']]:[])]:[]),
    ['Solved sideslip',fmt(p.sideslip_deg??0,4)+'°'],
    ['Wing load / limit',p.wing_load_ratios.map(x=>fmt(x*100,1)).join(' / ')+'%'],['Flight mode',flightModeLabel(resultConditions(a))],['Propeller torque & gyro',resultConditions(a).torque_gyro?'On':'Off'],['Engine state',escapeText(a.engine.policy)]])}</div><div><h3>COMPONENT FORCES · kgf</h3><table><thead><tr><th>Component</th><th>Forward</th><th>Up</th><th>Right</th></tr></thead><tbody>${forces.map(([k,v])=>`<tr><td>${names[k]||escapeText(k)}</td>${v.map(x=>`<td>${fmt(x/KGF,0)}</td>`).join('')}</tr>`).join('')}<tr><td>Total, incl. engine</td>${p.force_n.map(x=>`<td>${fmt(x/KGF,0)}</td>`).join('')}</tr></tbody></table></div></div>${p.reasons.length?`<div class="point-note">Excluded: ${escapeText(p.reasons.join('; '))}.</div>`:''}${note}`;
}
async function poll(id){
  if(state.activeJob!==id)return;
  try{
    const job=await api(`/api/jobs/${id}`);const p=job.progress||{};
    if(job.status==='complete'||job.status==='stopped'){
      await loadData(id,false);setRunning(false);state.activeJob=null;return;
    }
    if(job.status==='error')throw new Error(job.error||'Calculation failed');
    if(job.status==='cancelled'){setRunning(false);state.activeJob=null;$('footer-status').textContent=state.data?.preview?'Calculation cancelled · incomplete preview retained; exports unavailable.':'Calculation cancelled. Previous results retained.';return;}
    if(job.preview_revision&&state.previewToken!==`${id}:${job.preview_revision}`){
      await loadData(id,false,true);state.previewToken=`${id}:${job.preview_revision}`;
    }
    const name=state.meta.aircraft[p.aircraft]?.name||'';
    $('progress-label').textContent=job.status==='stopping'?'Stopping calculation and retaining partial results…':job.status==='exporting'?(job.partial_result?'Preparing partial results…':'Preparing diagram and result data…'):
      p.phase?`${p.phase}${name?' · '+name:''}`:name?`Solving ${name} · ${fmt(p.speed_kmh,0)} km/h · ${fmt(p.load_g,2)} g`:'Queued for calculation…';
    const percent=p.total?Math.min(99,100*p.done/p.total):0;
    $('progress-fill').style.width=percent+'%';$('progress-count').textContent=p.total?`${fmt(p.done,Number.isInteger(p.done)?0:1)} / ${p.total} · ${fmt(p.elapsed_s||0,0)} s`:'';
    setTimeout(()=>poll(id),900);
  }catch(e){
    if(state.activeJob!==id)return;
    if(e.retryable){
      $('progress-label').textContent='Connection interrupted · reconnecting to your calculation…';
      setTimeout(()=>poll(id),5000);return;
    }
    setRunning(false);state.activeJob=null;error(e.message);
  }
}
$('scenario').addEventListener('submit',async e=>{
  e.preventDefault();error('');if(state.running)return;
  if(state.meta.public_static)return error('Calculations require the local application. The public site currently provides the interface and aircraft catalog only.');
  if(!state.entries.length)return error('Add at least one aircraft.');
  saveCondition();
  if(+$('speed-min').value>=+$('speed-max').value)return error('Maximum speed must exceed minimum speed.');
  try{setRunning(true);$('progress-label').textContent='Preparing calculation…';$('progress-count').textContent='';$('progress-fill').style.width='0%';
    const job=await api('/api/jobs',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(readConfig())});state.activeJob=job.id;poll(job.id);
  }catch(e){setRunning(false);error(e.message);}
});
// ---- Weapon loadouts: the game's presets and custom pylons (in-game editor rules) ----
const loadoutCache=new Map();
const loadoutKey=(aircraft,vehicle)=>`${aircraft}|${vehicle||''}`;
function fetchLoadouts(aircraft,vehicle){
  const key=loadoutKey(aircraft,vehicle);
  if(!loadoutCache.has(key)){
    const request=api(`/api/loadouts/${encodeURIComponent(aircraft)}?vehicle=${encodeURIComponent(vehicle||'')}`)
      .then(info=>{request.value=info;return info;},cause=>{loadoutCache.delete(key);throw cause;});
    loadoutCache.set(key,request);
  }
  return loadoutCache.get(key);
}
const cachedLoadouts=(aircraft,vehicle)=>loadoutCache.get(loadoutKey(aircraft,vehicle))?.value;
const optionOf=(editor,tier,id)=>editor?.tiers.find(t=>t.tier===+tier)?.options.find(o=>o.id===id);
// Without stores the loadout is either the default ammunition and countermeasures (default) or clean: nothing aboard
// (Leo, 2026-10-26; this replaces the "Include default ammunition" switch). Stores always come with the ammunition.
const BARE_LOADOUTS={'':'Ammunition and countermeasures only',__clean:'Clean'};
function ammunitionText(aircraft,vehicle){
  const a=ammunitionFor(aircraft,vehicle);
  return a?.supported?[a.gun_rounds?`${fmt(a.gun_rounds,0)} rounds`:'',a.countermeasures?`${fmt(a.countermeasures,0)} countermeasures`:'',`${fmt(a.mass_kg,0)} kg`].filter(Boolean).join(' · '):'';
}
function loadoutText(c,info){
  if(!c.loadout)return BARE_LOADOUTS[c.default_ammunition?'':'__clean'];
  if(typeof c.loadout==='string')return info?.presets.find(p=>p.id===c.loadout)?.label||c.loadout;
  const tiers=Object.entries(c.loadout.tiers||{});
  if(!tiers.length)return 'Custom · all pylons empty';
  const counts={};
  for(const [tier,id] of tiers){const o=optionOf(info?.editor,tier,id);const label=o?.label||id;counts[label]=(counts[label]||0)+1;}
  return 'Custom · '+Object.entries(counts).map(([label,n])=>n>1?`${n}× (${label})`:label).join(', ');
}
function loadoutTotals(loadout,info){
  if(!loadout||!info)return {mass:0,drag:0};
  if(typeof loadout==='string'){const p=info.presets.find(p=>p.id===loadout);return {mass:p?.mass_kg||0,drag:p?.drag_area_m2||0};}
  let mass=0,drag=0;
  for(const [tier,id] of Object.entries(loadout.tiers||{})){const o=optionOf(info.editor,tier,id);if(o){mass+=o.mass_kg;drag+=o.drag_area_m2;}}
  return {mass,drag};
}
const loadoutFigures=t=>`${fmt(t.mass,0)} kg · drag area ${fmt(t.drag,3)} m²`;
async function showLoadout(){
  const entry=state.entries.find(e=>e.id===state.editing);
  $('loadout-field').hidden=!entry;
  if(!entry)return;
  const aircraft=state.meta.aircraft[entry.aircraft_id],c=state.conditions[entry.id];
  const available=!!aircraft.loadouts?.supported,ammunition=!!ammunitionFor(aircraft,c.ammunition_vehicle)?.supported;
  // Without stores the choice is still between ammunition only and clean.
  $('loadout-edit').disabled=!available&&!ammunition;
  $('loadout-summary').textContent=loadoutText(c,cachedLoadouts(entry.aircraft_id,c.ammunition_vehicle));
  $('loadout-detail').textContent=c.loadout?'':c.default_ammunition?ammunitionText(aircraft,c.ammunition_vehicle):
    available||ammunition?'Nothing aboard':(aircraft.loadouts?.reason||'Loadouts are unavailable for this aircraft.');
  if(!available||!c.loadout)return;
  try{
    const info=await fetchLoadouts(entry.aircraft_id,c.ammunition_vehicle);
    if(state.editing!==entry.id)return;
    $('loadout-summary').textContent=loadoutText(c,info);
    $('loadout-detail').textContent=loadoutFigures(loadoutTotals(c.loadout,info))+(c.stores_released?' · stores released, racks stay':'');
  }catch(cause){$('loadout-detail').textContent='Could not load loadouts: '+cause.message;}
}
const loadoutDialog={entry:null,info:null,draft:null,pylon:null,notes:[]};
async function openLoadoutEditor(){
  const entry=state.entries.find(e=>e.id===state.editing);if(!entry)return;
  saveCondition();
  const c=state.conditions[entry.id],d=loadoutDialog;
  d.entry=entry.id;d.info=null;d.pylon=null;d.notes=[];
  $('loadout-title').textContent=cleanName(state.meta.aircraft[entry.aircraft_id].name);
  $('loadout-preset-list').innerHTML='<p class="hint">Loading loadouts…</p>';
  $('loadout-custom').hidden=true;$('loadout-apply').disabled=true;
  $('loadout-released').checked=!!c.stores_released;
  $('loadout-dialog').showModal();
  const aircraft=state.meta.aircraft[entry.aircraft_id];
  d.ammunition=ammunitionText(aircraft,c.ammunition_vehicle);
  try{d.info=aircraft.loadouts?.supported?await fetchLoadouts(entry.aircraft_id,c.ammunition_vehicle):{presets:[],editor:null};}
  catch(cause){$('loadout-preset-list').innerHTML=`<p class="hint">${escapeText('Could not load loadouts: '+cause.message)}</p>`;return;}
  if(d.entry!==entry.id)return;
  const loadout=c.loadout;
  if(!loadout)d.draft={preset:c.default_ammunition?'':'__clean',tiers:{}};
  else if(typeof loadout==='string')d.draft={preset:loadout,tiers:{...(d.info.presets.find(p=>p.id===loadout)?.tiers||{})}};
  else d.draft={preset:null,tiers:{...loadout.tiers}};
  renderLoadoutEditor();
}
function loadoutWeights(editor,tiers){
  // weaponryPresetsParams.getPresetWeightRestrictionText: editor masses, sides split at slot count / 2.
  const limits=editor.limits,center=Math.floor(editor.count/2);
  let left=0,right=0,middle=0;
  for(const [tier,id] of Object.entries(tiers)){
    const t=editor.tiers.find(t=>t.tier===+tier),o=t?.options.find(o=>o.id===id);if(!o)continue;
    if(t.center)middle+=o.editor_mass_kg;else if(+tier<center)left+=o.editor_mass_kg;else right+=o.editor_mass_kg;
  }
  const total=left+right+middle,problems=[];
  if(!Object.values(limits).every(v=>v<0)){
    if(Math.abs(left-right)>limits.disbalance)problems.push(`Left/right mass difference ${fmt(Math.abs(left-right),0)} kg exceeds ${fmt(limits.disbalance,0)} kg.`);
    for(const [value,key,text] of [[total,'total','Total pylon mass'],[left,'left','Left pylon mass'],[right,'right','Right pylon mass']])
      if(limits[key]>=0&&value>limits[key])problems.push(`${text} ${fmt(value,0)} kg exceeds ${fmt(limits[key],0)} kg.`);
  }
  return {left,right,middle,total,problems};
}
function choosePylon(tier,id){
  // editSlotInPreset: banned weapons leave, weapons banning this one leave, required partners are fitted,
  // and weapons that depended on the replaced one leave with it.
  const d=loadoutDialog,editor=d.info.editor,tiers={...d.draft.tiers},notes=[];
  const tierOf=slot=>editor.tiers.find(t=>t.slot===slot);
  const name=(t,id)=>optionOf(editor,t,id)?.label||id;
  const dropDependents=(slot,id)=>{
    for(const [t,other] of Object.entries(tiers)){
      const o=optionOf(editor,t,other);
      if(o?.dependent.some(([s,p])=>s===slot&&p===id)){notes.push(`Removed ${name(t,other)} from pylon ${+t+1}: it needs ${name(tierOf(slot)?.tier,id)}.`);delete tiers[t];}
    }
  };
  const target=editor.tiers.find(t=>t.tier===+tier);
  if(tiers[tier])dropDependents(target.slot,tiers[tier]);
  if(id){
    const option=optionOf(editor,tier,id);
    for(const [slot,banned] of option.banned){
      const t=tierOf(slot);
      if(t&&tiers[t.tier]===banned){notes.push(`Removed ${name(t.tier,banned)} from pylon ${t.tier+1}: it cannot be carried with ${option.label}.`);delete tiers[t.tier];dropDependents(slot,banned);}
    }
    for(const [t,other] of Object.entries(tiers)){
      const o=optionOf(editor,t,other);
      if(+t!==+tier&&o?.banned.some(([s,p])=>s===target.slot&&p===id)){notes.push(`Removed ${name(t,other)} from pylon ${+t+1}: it cannot be carried with ${option.label}.`);delete tiers[t];}
    }
    for(const [slot,need] of option.dependent){
      const t=tierOf(slot);
      if(t&&t.options.some(o=>o.id===need)&&tiers[t.tier]!==need){notes.push(`Fitted ${name(t.tier,need)} on pylon ${t.tier+1}: ${option.label} needs it.`);tiers[t.tier]=need;}
    }
    tiers[tier]=id;
  }else delete tiers[tier];
  d.draft={preset:null,tiers};d.notes=notes;
  renderLoadoutEditor();
}
function renderLoadoutEditor(){
  const d=loadoutDialog,info=d.info,editor=info.editor;
  const preset=(id,label,figures,extra='',disabled=false,title='')=>`<button type="button" class="loadout-preset" data-preset="${escapeText(id)}" aria-pressed="${d.draft.preset===id}" ${disabled?'disabled':''} title="${escapeText(title)}"><strong>${escapeText(label)}${extra}</strong><span>${escapeText(figures)}</span></button>`;
  // The game's own preset with no stores (e.g. "Empty") is the same as ammunition only, so it is not listed twice.
  $('loadout-preset-list').innerHTML=preset('',BARE_LOADOUTS[''],d.ammunition||'No ammunition data for this aircraft','',!d.ammunition)+
    preset('__clean',BARE_LOADOUTS.__clean,'Nothing aboard')+
    info.presets.filter(p=>p.error||p.mass_kg||p.drag_area_m2).map(p=>preset(p.id,p.label,p.error?'Unavailable':loadoutFigures({mass:p.mass_kg,drag:p.drag_area_m2}),
      '',!!p.error,p.error||'')).join('')+
    (editor?`<button type="button" class="loadout-preset" data-preset="__custom" aria-pressed="${d.draft.preset===null}"><strong>Custom</strong><span>${escapeText(d.draft.preset===null?loadoutFigures(loadoutTotals({tiers:d.draft.tiers},info)):'Start from the selected preset')}</span></button>`:'');
  $('loadout-custom').hidden=!editor;
  let problems=[];
  if(editor){
    const sides=Math.floor(editor.count/2);
    $('loadout-pylons').innerHTML=editor.tiers.map(t=>{
      const id=d.draft.tiers[t.tier],o=id?t.options.find(o=>o.id===id):null;
      const side=t.center?'centre':t.tier<sides?'left':'right';
      return `<button type="button" class="loadout-pylon ${o?'':'empty'}" data-pylon="${t.tier}" aria-expanded="${d.pylon===t.tier}" aria-label="Pylon ${t.tier+1}: ${escapeText(o?.label||'empty')}"><b>${t.tier+1}</b><span>${escapeText(o?.label||'Empty')}</span><em>${side}</em></button>`;
    }).join('');
    const t=editor.tiers.find(t=>t.tier===d.pylon);
    $('loadout-options').hidden=!t;
    if(t){
      const current=d.draft.tiers[t.tier]||'';
      const row=(id,label,figures,disabled,title,tag='')=>`<button type="button" class="loadout-option" data-option="${escapeText(id)}" aria-pressed="${current===id}" ${disabled?'disabled':''} title="${escapeText(title)}"><strong>${escapeText(label)}${tag}</strong><span>${escapeText(figures)}</span></button>`;
      $('loadout-options').innerHTML=row('','Empty','',false,'')+t.options.map(o=>row(o.id,o.label,o.error?'Unavailable':loadoutFigures({mass:o.mass_kg,drag:o.drag_area_m2}),!!o.error,o.error||'')).join('');
    }
    const w=loadoutWeights(editor,d.draft.tiers),L=editor.limits;
    problems=d.draft.preset===null?w.problems:[];
    const cell=(label,value,limit,over)=>`<div class="${over?'over':''}"><small>${label}</small><strong>${fmt(value,0)}${limit>=0?' / '+fmt(limit,0):''} kg</strong></div>`;
    $('loadout-weights').innerHTML=cell('Total pylon mass',w.total,L.total,L.total>=0&&w.total>L.total)+cell('Left',w.left,L.left,L.left>=0&&w.left>L.left)+
      cell('Right',w.right,L.right,L.right>=0&&w.right>L.right)+cell('Difference',Math.abs(w.left-w.right),L.disbalance,Math.abs(w.left-w.right)>L.disbalance&&!Object.values(L).every(v=>v<0));
  }
  $('loadout-notes').classList.toggle('problem',problems.length>0);
  $('loadout-notes').textContent=[...problems,...d.notes].join(' ')||(editor?'Limits follow the in-game loadout editor. Drop tanks are carried full.':'Drop tanks are carried full.');
  const bare=d.draft.preset in BARE_LOADOUTS;
  $('loadout-total').textContent=d.draft.preset===''?d.ammunition:d.draft.preset==='__clean'?'Nothing aboard':
    loadoutFigures(loadoutTotals(d.draft.preset===null?{tiers:d.draft.tiers}:d.draft.preset,info));
  $('loadout-released').disabled=bare;
  $('loadout-apply').disabled=problems.length>0;
}
$('loadout-edit').addEventListener('click',openLoadoutEditor);
$('loadout-preset-list').addEventListener('click',event=>{
  const button=event.target.closest('[data-preset]');if(!button||button.disabled||!loadoutDialog.info)return;
  const d=loadoutDialog,id=button.dataset.preset;d.notes=[];
  if(id==='__custom')d.draft={preset:null,tiers:{...d.draft.tiers}};
  else d.draft={preset:id,tiers:{...(d.info.presets.find(p=>p.id===id)?.tiers||{})}};
  renderLoadoutEditor();
});
$('loadout-pylons').addEventListener('click',event=>{
  const button=event.target.closest('[data-pylon]');if(!button)return;
  loadoutDialog.pylon=loadoutDialog.pylon===+button.dataset.pylon?null:+button.dataset.pylon;renderLoadoutEditor();
});
$('loadout-options').addEventListener('click',event=>{
  const button=event.target.closest('[data-option]');if(!button||button.disabled)return;
  choosePylon(loadoutDialog.pylon,button.dataset.option);
});
$('loadout-apply').addEventListener('click',()=>{
  const d=loadoutDialog,c=state.conditions[d.entry];if(!c||!d.draft)return;
  c.loadout=d.draft.preset in BARE_LOADOUTS?'':d.draft.preset!==null?d.draft.preset:{tiers:{...d.draft.tiers}};
  c.default_ammunition=d.draft.preset!=='__clean'&&!!d.ammunition;
  c.stores_released=!!c.loadout&&$('loadout-released').checked;
  if(state.editing===d.entry)$('default-ammunition').checked=c.default_ammunition;
  $('loadout-dialog').close();
  if(state.editing===d.entry)showLoadout();
  syncLabels();
});
function formChanged(e){
  if(e.target.id==='aircraft-search')return;
  if(e.target.id==='enable-exports'){updateContourAvailability();return;}
  $('apply-settings-status').textContent='';
  saveCondition();
  if(e.target.id==='ammunition-vehicle'&&state.editing){
    // Each armament variant has its own pylons and presets.
    state.conditions[state.editing].loadout='';state.conditions[state.editing].stores_released=false;showLoadout();
  }
  syncLabels();
}
$('scenario').addEventListener('input',formChanged);$('scenario').addEventListener('change',formChanged);
$('reset').addEventListener('click',()=>{populate(state.meta.defaults);syncLabels();error('');});
$('cancel').addEventListener('click',async()=>{if(state.activeJob){$('cancel').disabled=true;try{await api(`/api/jobs/${state.activeJob}/cancel`,{method:'POST'});$('progress-label').textContent='Stopping calculation and retaining partial results…';}catch(e){error(e.message);$('cancel').disabled=false;}}});
$('chart-tabs').addEventListener('click',event=>{
  const button=event.target.closest('[data-view]');if(!button)return;
  state.view=button.dataset.view;document.querySelectorAll('[data-view]').forEach(b=>b.classList.toggle('active',b===button));renderChart();
});
$('contour-levels').addEventListener('click',event=>{
  const button=event.target.closest('[data-contour-remove]');if(!button)return;
  state.customContours=true;
  state.contourLevels=state.contourLevels.filter(level=>level!==Number(button.dataset.contourRemove));
  renderContourControls();syncLabels();refreshContours();
});
$('contour-add').addEventListener('click',()=>{
  const input=$('contour-input'),level=Number(input.value);
  if(!input.value.trim()||!Number.isFinite(level)||Math.abs(level)>2000||state.contourLevels.includes(level)||state.contourLevels.length>=16){
    $('contour-status').textContent='Enter a unique level from -2000 to 2000 m/s (up to 16).';return;
  }
  state.customContours=true;state.contourLevels.push(level);state.contourLevels.sort((a,b)=>b-a);input.value='';
  renderContourControls();syncLabels();refreshContours();
});
$('contour-input').addEventListener('keydown',event=>{if(event.key==='Enter'){event.preventDefault();$('contour-add').click();}});
for(const id of ['show-samples','show-rejected'])$(id).addEventListener('change',renderChart);
document.querySelectorAll('[data-export]').forEach(a=>a.addEventListener('click',e=>{if(a.getAttribute('aria-disabled')==='true')e.preventDefault();}));
(async()=>{try{
  $('calculate').disabled=true;
  // Symbol glyphs are cosmetic; a missing font must not disable the calculator.
  [state.meta]=await Promise.all([api('/api/meta?refresh=1'),document.fonts?.load('14px WTSymbols','▄').catch(()=>[])]);
  $('calculate').disabled=false;
  state.meta.defaults=requestConditions({...state.meta.defaults,default_ammunition:true,aircraft:[]});
  populateAircraft();populate(state.meta.defaults);
  if(state.meta.public_static){
    $('hosting-notice').hidden=false;
    $('runtime-label').innerHTML='<i></i> PUBLIC PREVIEW';$('runtime-label').hidden=false;
    $('runtime-footer').textContent='PUBLIC PREVIEW';
    $('footer-status').textContent='Calculations currently run in the local application.';
    $('calculate').disabled=true;$('calculate').textContent='Calculations require local app';
  }else if(remoteSite){
    // No runtime label on the website or local copies (Leo, 2026-10-23); only the public preview shows one.
    $('footer-status').textContent='Calculations run on the server.';
  }
}catch(e){error('Could not load the plotter: '+e.message);}})();
// Header calculator menus (Leo, 2026-10-25): each group opens on hover, on a click of its name or with the down
// arrow, and closes on a choice, Escape, leaving it or a click elsewhere. missiles.js switches the calculators.
(()=>{
  const groups=[...document.querySelectorAll('.nav-group')],hover=matchMedia('(hover: hover)');
  const set=(group,open)=>{group.classList.toggle('open',open);group.querySelector('.nav-top').setAttribute('aria-expanded',String(open));};
  const closeAll=except=>groups.forEach(g=>{if(g!==except)set(g,false);});
  for(const group of groups){
    const top=group.querySelector('.nav-top'),items=[...group.querySelectorAll('.nav-item')];
    group.addEventListener('mouseenter',()=>{if(hover.matches){closeAll(group);set(group,true);}});
    group.addEventListener('mouseleave',()=>{if(hover.matches)set(group,false);});
    // With a mouse the menu is already open under the pointer; a tap toggles it.
    top.addEventListener('click',()=>{const open=hover.matches||!group.classList.contains('open');closeAll(group);set(group,open);});
    for(const item of items)item.addEventListener('click',()=>set(group,false));
    group.addEventListener('keydown',event=>{
      const at=items.indexOf(document.activeElement);
      if(event.key==='Escape'){set(group,false);top.focus();}
      else if(event.key==='ArrowDown'){set(group,true);items[at<0?0:Math.min(at+1,items.length-1)].focus();}
      else if(event.key==='ArrowUp'&&at>=0)(at===0?top:items[at-1]).focus();
      else return;
      event.preventDefault();
    });
    group.addEventListener('focusout',event=>{if(!group.contains(event.relatedTarget))set(group,false);});
  }
  document.addEventListener('click',event=>{if(!event.target.closest('.nav-group'))closeAll();});
})();
