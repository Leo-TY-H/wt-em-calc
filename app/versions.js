'use strict';
// Game version search (Leo, 2026-10-30): each aircraft and missile row picks its game build in its edit popup, and
// rows on different builds compare on one chart. Every other build has its own server behind /v/<build>/ on the same
// host; each runs the same release on that build's data, so a chart difference is a data difference. The server
// starts on first use. Without a version list (a local copy) the calculators behave as they always did.
window.GameVersions=(()=>{
  const base=String((window.EM_CONFIG||{}).apiBase||'').replace(/\/$/,'');
  const root=path=>base?base+path:'.'+path;
  const START_LIMIT_S=120;
  let list=null,history=null;
  const metas=new Map(),starts=new Map();

  const escapeHtml=s=>String(s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
  const enabled=()=>!!list;
  const build=version=>list?.builds.find(b=>b.id===version);
  // Identical builds share one server: the one holding their data.
  const server=version=>build(version)?.data_build||version;
  const isDefault=version=>!version||!list||server(version)===server(list.default);
  const url=(path,version)=>root(isDefault(version)?path:`/v/${encodeURIComponent(server(version))}${path}`);
  const channelName={live:'Live',dev:'Dev'};
  const label=version=>{
    const id=version||list?.default;
    const channel=channelName[build(id)?.channel];
    return channel?`${id} · ${channel}`:id||'';
  };
  // A row whose build couldn't compile it keeps its last working data, and says from which build.
  const rowLabel=(version,row)=>label(version)+(row?.data_version_used?` · data from ${row.data_version_used}`:'');
  // The solver and compiler follow one game build; any other major is that build's data under those rules.
  const rulesNote=version=>{
    const id=version||list?.default;
    return list&&id&&id.split('.').slice(0,2).join('.')!==list.rules?`Calculated with ${list.rules} game rules`:'';
  };

  async function load(){
    try{
      const response=await fetch(root('/api/versions'));
      if(!response.ok)return null;
      const value=await response.json();
      if(value.builds?.length&&value.default)list=value;
    }catch{list=null;}
    return list;
  }
  function loadHistory(){
    history??=fetch(root('/api/version-history')).then(r=>r.ok?r.json():null).catch(()=>null);
    return history;
  }
  // The meta of a build whose server has answered; null for the default build, which the page already holds.
  const meta=version=>isDefault(version)?null:metas.get(server(version))?.value||null;
  async function requestMeta(id){
    const response=await fetch(root(`/v/${encodeURIComponent(id)}/api/meta`));
    if([502,503,504].includes(response.status))throw Object.assign(new Error('Not running'),{starting:true});
    if(!response.ok)throw new Error(`Build ${id} did not answer (${response.status})`);
    return response.json();
  }
  // Resolves once the build's server answers, starting it if needed. onStatus reports the wait.
  function ensure(version,onStatus=()=>{}){
    if(isDefault(version))return Promise.resolve(null);
    const id=server(version),known=metas.get(id);
    if(known)return known;
    const waiting=(async()=>{
      try{const value=await requestMeta(id);waiting.value=value;return value;}
      catch(cause){if(!cause.starting&&!(cause instanceof TypeError))throw cause;}
      const started=await fetch(root(`/api/versions/${encodeURIComponent(id)}/start`),{method:'POST'});
      if(!started.ok)throw new Error((await started.json().catch(()=>({}))).error||`Build ${id} cannot start`);
      const until=performance.now()+START_LIMIT_S*1000;
      while(performance.now()<until){
        onStatus(`Starting ${id}…`);
        await new Promise(resolve=>setTimeout(resolve,2000));
        try{const value=await requestMeta(id);waiting.value=value;return value;}
        catch(cause){if(!cause.starting&&!(cause instanceof TypeError))throw cause;}
      }
      throw new Error(`Build ${id} did not start in time. Please try again.`);
    })();
    metas.set(id,waiting);
    waiting.catch(()=>metas.delete(id));
    return waiting;
  }
  // A server stopped while idle has to be asked again.
  const forget=version=>metas.delete(server(version));

  // ---- The search in an edit popup --------------------------------------------------------
  // file: the row's flight model or missile file name; kind: 'fm' or 'missile'.
  function picker(container,{file,kind,current,onPick}){
    if(!list){container.hidden=true;container.replaceChildren();return;}
    container.hidden=false;
    const noun=kind==='missile'?'file':'FM';
    container.innerHTML=`<label class="field" for="version-search-box">Game version</label>
      <input id="version-search-box" class="version-search" type="search" placeholder="Search game versions…" aria-label="Search game versions" autocomplete="off">
      <label class="check-row version-changed"><input type="checkbox"> Only where its ${noun} changed</label>
      <div class="version-list" role="listbox" aria-label="Game versions"></div>
      <small class="version-note" role="status" aria-live="polite"></small>`;
    const search=container.querySelector('.version-search'),changed=container.querySelector('.version-changed input'),
      rows=container.querySelector('.version-list'),note=container.querySelector('.version-note');
    let track=null,order=new Map(),selected=current||list.default;
    const render=()=>{
      const query=search.value.trim(),only=changed.checked&&!!track;
      const index=id=>order.get(id);
      const present=b=>{
        if(!track)return true;
        const i=index(b.id);
        return i>=track.changes[0]&&(track.removed==null||i<track.removed);
      };
      const shown=list.builds.filter(b=>(!query||b.id.includes(query))&&(!only||track.changes.includes(index(b.id))));
      rows.innerHTML=shown.map(b=>{
        const moved=track?.changes.includes(index(b.id))&&index(b.id)>track.changes[0],inBuild=present(b),usable=b.available&&inBuild;
        const state=!b.available?'Not built yet':!inBuild?'Not in this build':moved?`${noun} changed`:'';
        return `<button type="button" class="version-row${b.id===selected?' selected':''}" role="option" aria-selected="${b.id===selected}" data-version="${escapeHtml(b.id)}"${usable?'':' disabled'}>
          <span>${escapeHtml(label(b.id))}</span><small>${escapeHtml(state)}</small><time>${escapeHtml(b.date)}</time></button>`;
      }).join('')||'<p class="hint">No matching game version.</p>';
    };
    const say=text=>{note.textContent=text;};
    const show=value=>{selected=value;say(rulesNote(value));render();};
    container.querySelector('.version-changed').hidden=true;
    show(selected);
    rows.querySelector('.selected')?.scrollIntoView({block:'nearest'});
    search.addEventListener('input',render);
    changed.addEventListener('change',render);
    rows.addEventListener('click',async event=>{
      const row=event.target.closest('[data-version]');
      if(!row||row.disabled||row.dataset.version===selected)return;
      const version=row.dataset.version;
      try{
        say(`Loading ${version}…`);
        await ensure(version,say);
        const result=await onPick(version===list.default?undefined:version);
        if(typeof result==='string'){say(result);return;}
        show(version);
        if(result?.note)say(result.note);
      }catch(cause){say(cause.message);}
    });
    loadHistory().then(value=>{
      track=value?.[kind]?.[file]||null;
      if(!track||!container.contains(rows))return;
      order=new Map(value.builds.map((id,i)=>[id,i]));
      container.querySelector('.version-changed').hidden=false;
      render();
    });
    return {show,say};
  }

  // ---- Rows on different builds ------------------------------------------------------------
  // One request per build, in row order. Each server only sees its own rows, and the version stays on this page.
  function split(config){
    const groups=new Map();
    for(const entry of config.entries){
      const key=isDefault(entry.version)?'':server(entry.version);
      if(!groups.has(key))groups.set(key,{version:key||undefined,entries:[]});
      groups.get(key).entries.push({id:entry.id,aircraft_id:entry.aircraft_id,settings:entry.settings});
    }
    return [...groups.values()].map(group=>({version:group.version,config:{...config,entries:group.entries,
      aircraft:[...new Set(group.entries.map(e=>e.aircraft_id))]}}));
  }
  // The chart of all finished builds, drawn as one result: rows back in their order, with their row colours and numbers.
  function merge(parts,rows,colors){
    const order=new Map(rows.map((row,index)=>[row.id,index]));
    const aircraft=parts.flatMap(part=>part.data.aircraft.map(a=>({...a,game_version:part.version}))).sort((a,b)=>order.get(a.id)-order.get(b.id));
    for(const a of aircraft){
      const index=order.get(a.id);
      if(index==null)continue;
      a.color=colors[index]||a.color;
      a.name=a.name.replace(/ · \d+( · |$)/,` · ${index+1}$1`);
      if(parts.length>1||!isDefault(a.game_version)){a.version_label=a.game_version||list.default;a.name+=` · ${a.version_label}`;}
    }
    const merged={...parts[0].data,aircraft,elapsed_s:parts.reduce((sum,part)=>sum+(part.data.elapsed_s||0),0)};
    if(parts.length>1){
      merged.speeds_kmh=[...new Set(parts.flatMap(part=>part.data.speeds_kmh||[]))].sort((x,y)=>x-y);
      merged.preview=parts.some(part=>part.data.preview);
      merged.calculation_stopped=parts.some(part=>part.data.calculation_stopped);
      for(const key of Object.keys(merged))if(key.startsWith('plot_max')&&Number.isFinite(merged[key]))
        merged[key]=Math.max(...parts.map(part=>part.data[key]??-Infinity));
    }
    return merged;
  }
  // ---- Missile rows ------------------------------------------------------------------------
  // A row on another build is its own entry in the page's missile list, keyed "id@build", so every lookup by key and
  // the page's comparison work unchanged; requests send the plain id to that build's server.
  // A further row of one missile on one build (its launcher can differ) gets its own entry the same way: "id@build~2".
  const baseMissile=key=>String(key).split(/[@~]/)[0];
  const missileVersion=key=>String(key).split('~')[0].split('@')[1];
  function freeRow(meta,key,taken){
    const plain=String(key).split('~')[0];
    if(!taken.includes(plain))return plain;
    let n=2;
    while(taken.includes(`${plain}~${n}`))n++;
    const next=`${plain}~${n}`,row=meta.missiles.find(m=>m.id===plain);
    if(!meta.missiles.some(m=>m.id===next))meta.missiles.push({...row,id:next,base_id:row.base_id||plain,base_name:row.base_name||row.name});
    return next;
  }
  const missileMetas=new Map();
  function missileMeta(version){
    const id=server(version);
    if(!missileMetas.has(id)){
      const loaded=ensure(version).then(async()=>{
        const response=await fetch(url('/api/missiles/meta',version));
        if(!response.ok)throw new Error(`Build ${id} did not answer (${response.status})`);
        return response.json();
      });
      missileMetas.set(id,loaded);
      loaded.catch(()=>missileMetas.delete(id));
    }
    return missileMetas.get(id);
  }
  async function missileRow(meta,base,version){
    if(isDefault(version))return base;
    const key=`${base}@${server(version)}`;
    if(!meta.missiles.some(m=>m.id===key)){
      const other=await missileMeta(version),row=other.missiles.find(m=>m.id===base);
      if(!row)throw new Error('This missile is not in that build.');
      meta.missiles.push({...row,id:key,base_id:base,base_name:row.name,name:`${row.name} · ${server(version)}`});
      meta.aircraft={...other.aircraft,...meta.aircraft};
    }
    return key;
  }
  function versionDialog(){
    let dialog=document.getElementById('version-dialog');
    if(dialog)return dialog;
    dialog=document.createElement('dialog');dialog.id='version-dialog';dialog.className='entry-dialog';
    dialog.innerHTML=`<header class="entry-dialog-header"><div><span class="eyebrow"></span><h2></h2></div><button type="button" class="text-button" aria-label="Close">✕</button></header><div class="entry-dialog-body"><div class="version-picker"></div></div>`;
    dialog.querySelector('button').addEventListener('click',()=>dialog.close());
    dialog.addEventListener('click',event=>{if(event.target===dialog)dialog.close();});
    document.body.append(dialog);
    return dialog;
  }
  // onKey gets the row's new key; taken: the keys of all rows.
  function missileDialog({key,meta,color,taken,onKey}){
    const dialog=versionDialog(),base=baseMissile(key),row=meta.missiles.find(m=>m.id===key);
    dialog.querySelector('.eyebrow').textContent='MISSILE SETTINGS';
    dialog.querySelector('h2').textContent=row?.base_name||row?.name||base;
    dialog.style.setProperty('--entry-color',color);
    picker(dialog.querySelector('.version-picker'),{file:base,kind:'missile',current:missileVersion(key),onPick:async version=>{
      onKey(freeRow(meta,await missileRow(meta,base,version),taken.filter(other=>other!==key)));dialog.close();
    }});
    dialog.showModal();
  }
  // ---- What changed: the values that differ between builds of the same aircraft or missile ---
  // Values as the app reads them (an FM after its loader, so a reformatted file doesn't read as all changed).
  // Hidden by default: what can't change a chart (temperatures, autopilot, passport), and float round-off.
  const QUIET=/^(Autopilot|Passport)\b|\.Temperature\.|^Instructor\.MouseAim\b/;
  const valueCache=new Map();
  function values(kind,id,version){
    const key=`${kind}|${id}|${server(version||list.default)}`;
    if(!valueCache.has(key)){
      const loaded=fetch(url(`/api/build-values/${kind==='missile'?'missile':'aircraft'}/${encodeURIComponent(id)}`,version)).then(async response=>{
        if(!response.ok)throw new Error((await response.json().catch(()=>({}))).error||'Not available');
        return response.json();
      });
      valueCache.set(key,loaded);
      loaded.catch(()=>valueCache.delete(key));
    }
    return valueCache.get(key);
  }
  function leaves(value,prefix,out){
    if(Array.isArray(value))value.forEach((item,i)=>leaves(item,`${prefix}[${i}]`,out));
    else if(value&&typeof value==='object')for(const [key,item] of Object.entries(value))leaves(item,prefix?`${prefix}.${key}`:key,out);
    else out.set(prefix,value);
    return out;
  }
  const same=(a,b)=>a===b||typeof a==='number'&&typeof b==='number'&&Math.abs(a-b)<=1e-4*Math.max(Math.abs(a),Math.abs(b));
  const shownValue=v=>typeof v==='number'?String(+v.toPrecision(7)):String(v);
  function difference(a,b){
    const left=leaves(a,'',new Map()),right=leaves(b,'',new Map()),rows=[];let oneSided=0;
    for(const [key,value] of left){
      if(!right.has(key))oneSided++;
      else if(!same(value,right.get(key)))rows.push([key,value,right.get(key)]);
    }
    for(const key of right.keys())if(!left.has(key))oneSided++;
    return {rows,oneSided};
  }
  // rows: {id, kind ('fm' or 'missile'), version, name}, in row order. Each pair compares a row with the first row of its name.
  async function changes(container,rows){
    const token=container.dataset.token=String(+(container.dataset.token||0)+1);
    const groups=new Map();
    if(list)for(const row of rows){
      const group=groups.get(row.kind+row.id)||[];
      if(!group.some(other=>server(other.version||list.default)===server(row.version||list.default)))group.push(row);
      groups.set(row.kind+row.id,group);
    }
    const pairs=[...groups.values()].filter(group=>group.length>1).flatMap(group=>group.slice(1).map(other=>[group[0],other]));
    container.hidden=!pairs.length;
    if(!pairs.length){container.replaceChildren();return;}
    container.innerHTML='<h2>What changed</h2><p class="hint">Comparing values…</p>';
    const sections=await Promise.all(pairs.map(async ([a,b])=>{
      const from=a.version||list.default,to=b.version||list.default;
      try{
        const {rows:found,oneSided}=difference(await values(a.kind,a.id,a.version),await values(b.kind,b.id,b.version));
        const quiet=found.filter(row=>QUIET.test(row[0])),loud=found.filter(row=>!QUIET.test(row[0]));
        const table=list_=>`<table><thead><tr><th>Value</th><th>${escapeHtml(from)}</th><th>${escapeHtml(to)}</th></tr></thead><tbody>${list_.map(([key,x,y])=>`<tr><td>${escapeHtml(key)}</td><td>${escapeHtml(shownValue(x))}</td><td>${escapeHtml(shownValue(y))}</td></tr>`).join('')}</tbody></table>`;
        const notes=[quiet.length?`${quiet.length} temperature, autopilot and passport values also differ.`:'',oneSided?`${oneSided} values exist in only one of the two.`:''].filter(Boolean).join(' ');
        return `<section><h3>${escapeHtml(a.name)} · ${escapeHtml(from)} to ${escapeHtml(to)}</h3>${loud.length?table(loud):'<p class="hint">No flight values differ.</p>'}${notes?`<p class="hint">${escapeHtml(notes)}</p>`:''}${quiet.length?`<details><summary>Show the other ${quiet.length}</summary>${table(quiet)}</details>`:''}</section>`;
      }catch(cause){return `<section><h3>${escapeHtml(a.name)}</h3><p class="hint">${escapeHtml(cause.message)}</p></section>`;}
    }));
    if(container.dataset.token===token)container.innerHTML='<h2>What changed</h2>'+sections.join('');
  }
  return {load,enabled,url,label,rowLabel,rulesNote,meta,ensure,forget,picker,split,merge,loadHistory,isDefault,baseMissile,missileVersion,freeRow,missileRow,missileDialog,changes,
    get default(){return list?.default;},get dev(){return list?.builds.find(b=>b.channel==='dev')?.id;}};
})();
