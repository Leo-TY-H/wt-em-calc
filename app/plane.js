// Plane info (Leo, 2026-11-19): key mass, aerodynamic, limit and propulsion figures of up to eight aircraft at their
// settings, their flap speed curves (automatic schedule, critical speed) and a flap efficiency score (POST /api/plane,
// scripts/plane_info.py). Laid out like the engine page; uses app.js's catalog (state.meta), api(), name helpers and
// loadout editor.
(() => {
  'use strict';
  const el = id => document.getElementById(id);
  const root = el('plane-workspace');
  if (!root) return;
  const MAX = 8, defaults = {altitude: 0};
  const EMPTY = '<div class="empty"><span class="empty-glyph">⌁</span><h2>Add an aircraft and press Calculate</h2></div>';

  root.innerHTML = `<aside class="sidebar"><form id="plane-form" novalidate>
    <fieldset id="plane-picker"><legend class="visually-hidden">Add aircraft</legend>
      <input id="plane-search" type="search" placeholder="Search aircraft…" autocomplete="off" aria-label="Search aircraft" aria-controls="plane-list">
      <div id="plane-list" class="aircraft-list" aria-label="Available aircraft"></div>
      <p id="plane-search-empty" class="hint" hidden>No aircraft match your search.</p>
    </fieldset>
    <div id="plane-selected" aria-label="Added aircraft"></div>
    <label class="field" id="plane-altitude-field">Altitude<div class="input-unit"><input id="plane-altitude" type="number" min="0" max="30000" step="any" value="${defaults.altitude}" required aria-label="Altitude"><span>m</span></div></label>
    <div class="sidebar-actions"><p id="plane-ready" class="hint" hidden></p>
      <div class="action-row"><button id="plane-run" class="primary-button" type="submit" disabled>Calculate</button><button id="plane-reset" class="secondary-button reset-button" type="button">Reset</button></div></div>
  </form></aside>
  <main>
    <div id="plane-error" class="error" role="alert" hidden></div>
    <div id="plane-empty" class="plane-empty">${EMPTY}</div>
    <div id="plane-charts" class="plane-charts" hidden>
      <section class="chart-card"><h3>FLAP SPEEDS</h3><div id="plane-flaps-chart" class="plane-chart" aria-label="Flap schedule and critical speed"></div></section>
      <section class="chart-card"><h3>FLAP EFFICIENCY</h3><div class="mode-switch plane-setting" id="plane-setting" role="radiogroup" aria-label="Flap setting">${['Combat', 'Takeoff', 'Landing'].map(m =>
        `<label><input type="radio" name="plane-setting" value="${m.toLowerCase()}"><span>${m}</span></label>`).join('')}</div><div id="plane-score-chart" class="plane-chart" aria-label="Flap efficiency"></div></section>
    </div>
    <div id="plane-tables" class="plane-tables" hidden></div>
    <section id="plane-version-changes" class="version-changes" hidden></section>
  </main>
  <dialog id="plane-dialog" class="entry-dialog" aria-labelledby="plane-dialog-title"><header class="entry-dialog-header"><div><span class="eyebrow">AIRCRAFT SETTINGS</span><h2 id="plane-dialog-title"></h2></div><button type="button" id="plane-dialog-close" class="text-button" aria-label="Close settings">✕</button></header>
    <div class="entry-dialog-body"><div id="plane-version" class="version-picker" hidden></div><label class="field">Fuel load <output id="plane-fuel-label">30%</output><input id="plane-fuel" type="range" min="1" max="100" step="1" value="30"></label>
      <label class="field">Throttle <output id="plane-throttle-label">110%</output><input id="plane-throttle" type="range" min="0" max="110" step="1" value="110"></label>
      <div class="field loadout-field"><div class="loadout-field-heading"><span>Loadout</span><button type="button" id="plane-loadout-edit" class="text-button">Change…</button></div>
        <div id="plane-loadout-summary" class="loadout-summary"></div><small id="plane-loadout-detail"></small></div>
      <label id="plane-vehicle-field" class="field" hidden>Armament variant<select id="plane-vehicle"></select></label>
      <label class="field">Additional mass at CG<div class="input-unit"><input id="plane-mass" type="number" min="0" max="5000" step="any" value="0" aria-label="Additional mass at CG"><span>kg</span></div></label>
      <div class="divider"></div>
      <div class="apply-settings"><button id="plane-apply-all" type="button">Apply these settings to all added aircraft</button><span id="plane-apply-status" class="hint" role="status" aria-live="polite"></span></div></div></dialog>`;

  let entries = [], nextEntry = 1, results = new Map(), running = false, shown = null, editing = null;
  const catalog = () => (typeof state !== 'undefined' && state.meta) ? state.meta.aircraft : null;
  const showError = text => { el('plane-error').textContent = text; el('plane-error').hidden = !text; };
  const aircraftOf = entry => metaOf(entry.version)?.aircraft?.[entry.aircraft_id] ?? catalog()?.[entry.aircraft_id];
  const versionSuffix = e => window.GameVersions?.enabled() && (e.version || shown?.entries.some(x => x.version)) ? ` · ${e.version || GameVersions.default}` : '';
  const full = (aircraft_id, version) => noWep(aircraft_id, version) ? '100' : '110';
  const body = (entry, run) => ({aircraft: entry.aircraft_id, altitude_m: run.altitude, throttle: +entry.throttle, mass: entry.c});
  const key = (entry, run) => JSON.stringify({...body(entry, run), version: entry.version});

  // ---- Aircraft list and cards (as the engine page) ----------------------------------------
  function populate() {
    const aircraft = catalog();
    const symbol = a => Number(cleanName(a.name) !== sortName(a.name));
    const rows = Object.entries({...state.extraAircraft, ...aircraft}).filter(([, a]) => !a.bomb_plane).sort(([, a], [, b]) => Number(b.supported) - Number(a.supported) ||
      sortName(a.name).localeCompare(sortName(b.name), undefined, {numeric: true}) || symbol(a) - symbol(b) || a.name.localeCompare(b.name));
    el('plane-list').innerHTML = rows.map(([id, a]) => `<div class="aircraft-option" data-search="${escapeText(id + ' ' + a.name)}" data-supported="${!!a.supported}" title="${escapeText(a.supported ? '' : a.reason || '')}"><span><strong>${escapeText(cleanName(a.name))}</strong><small><code>${escapeText(nameCode(a.name) || id)}</code>${a.only_in ? escapeText(' · Only in ' + a.only_in) : ''}</small></span><button type="button" data-add="${escapeText(id)}"${a.only_in ? ` data-version="${escapeText(a.only_in)}"` : ''} aria-label="Add ${escapeText(a.name)}"${a.supported ? '' : ' disabled'}>Add</button></div>`).join('');
    filter(); renderEntries(); updateForm();
  }
  function filter() {
    const normalize = text => text.toLowerCase().normalize('NFKD').replace(/[^a-z0-9]/g, '');
    const query = normalize(el('plane-search').value); let visible = 0;
    for (const row of el('plane-list').children) {
      row.hidden = !normalize(row.dataset.search).includes(query); if (!row.hidden) visible++;
      row.querySelector('button').disabled = row.dataset.supported !== 'true' || entries.length >= MAX || running;
    }
    el('plane-list').hidden = !query || !visible;
    el('plane-search-empty').hidden = !query || visible > 0;
  }
  function renderEntries() {
    el('plane-selected').innerHTML = entries.map((e, i) => {
      const a = aircraftOf(e), name = a?.name || e.aircraft_id, off = running ? ' disabled' : '';
      return `<article class="aircraft-entry ${e.id === editing ? 'active' : ''}" data-entry="${e.id}" style="--entry-color:${entryColors[i]}">
        <div class="entry-heading"><span class="entry-number">${i + 1}</span><strong>${escapeText(cleanName(name))}</strong>${window.GameVersions?.enabled() ? `<code class="entry-version" title="${escapeText(GameVersions.rulesNote(e.version))}">${escapeText(GameVersions.rowLabel(e.version, aircraftOf(e)))}</code>` : ''}</div>
        <div class="entry-actions"><button type="button" class="entry-edit entry-icon" data-action="configure" aria-haspopup="dialog" aria-label="Edit settings of entry ${i + 1}" title="Edit settings"${off}>${EDIT_ICON}</button>
        <button type="button" class="entry-icon" data-action="duplicate" aria-label="Duplicate entry ${i + 1}" title="Duplicate"${entries.length >= MAX || running ? ' disabled' : ''}>${COPY_ICON}</button>
        <button type="button" class="entry-icon" data-action="remove" aria-label="Remove entry ${i + 1}" title="Remove"${off}>${REMOVE_ICON}</button></div></article>`;
    }).join('');
  }
  // c: the EM page's mass settings without flaps (the page covers every extension), normalized as app.js
  // aircraftConditions does: ammunition only where the aircraft has it, loadouts per aircraft.
  const FRESH = {throttle: null, c: null};
  function massSettings(aircraft_id, from, version) {
    const a = metaOf(version).aircraft[aircraft_id] ?? catalog()[aircraft_id], d = state.meta.defaults;
    const {fuel_percent = d.fuel_percent, extra_mass_kg = d.extra_mass_kg, loadout = '', stores_released = false,
      default_ammunition = d.default_ammunition, ammunition_vehicle = ''} = from || {};
    const c = {fuel_percent, extra_mass_kg, loadout, stores_released, default_ammunition, ammunition_vehicle};
    if (!a.ammunition?.variants?.[c.ammunition_vehicle]) c.ammunition_vehicle = a.ammunition?.vehicle_id || '';
    if (!ammunitionFor(a, c.ammunition_vehicle)?.supported) c.default_ammunition = false;
    if (!a.loadouts?.supported || !c.loadout) { c.loadout = ''; c.stores_released = false; }
    return c;
  }
  const addEntry = (aircraft_id, from = FRESH, version = from.version) => {
    if (entries.length >= MAX) return;
    const throttle = from.throttle ?? full(aircraft_id, version);
    // These rows also travel to the other calculators, whose API requires entry_ identifiers.
    let id; do { id = `entry_plane_${nextEntry++}`; } while (entries.some(entry => entry.id === id));
    entries.push({id, aircraft_id, ...(version ? {version} : {}), throttle, c: massSettings(aircraft_id, from.c && structuredClone(from.c), version)});
    renderEntries(); filter(); updateForm();
  };
  el('plane-list').addEventListener('click', event => {
    const button = event.target.closest('[data-add]'); if (button && !button.disabled) addEntry(button.dataset.add, FRESH, button.dataset.version);
  });
  el('plane-selected').addEventListener('click', event => {
    const button = event.target.closest('[data-action]'); if (!button || button.disabled || running) return;
    const index = entries.findIndex(e => e.id === button.closest('[data-entry]').dataset.entry), entry = entries[index];
    if (button.dataset.action === 'duplicate') return addEntry(entry.aircraft_id, entry);
    if (button.dataset.action === 'remove') { entries.splice(index, 1); renderEntries(); filter(); updateForm(); render(); return; }
    editing = entry.id; showSettings(); renderEntries();
    window.GameVersions?.picker(el('plane-version'), {file: aircraftOf(entry)?.fm_id || entry.aircraft_id, kind: 'fm', current: entry.version, onPick: version => setVersion(entry, version)});
    el('plane-dialog-title').textContent = `${index + 1} · ${cleanName(aircraftOf(entry)?.name || entry.aircraft_id)}`;
    el('plane-dialog').style.setProperty('--entry-color', entryColors[index]);
    el('plane-dialog').showModal();
  });

  // The row changes at once. The build's own values and checks follow when its data arrives, unless the row has moved on.
  async function setVersion(entry, version) {
    const previous = entry.version, current = () => entry.version === version && entries.includes(entry);
    const apply = value => { if (value) entry.version = value; else delete entry.version; renderEntries(); updateForm(); };
    const back = text => { if (current()) apply(previous); return text; };
    apply(version);
    try { await GameVersions.ensure(version); } catch (cause) { return back(cause.message); }
    if (!current()) return undefined;
    const row = metaOf(version).aircraft[entry.aircraft_id];
    if (!row) return back('This aircraft is not in that build.');
    if (!row.supported) return back('Not available in that build: ' + (row.reason || 'model integration pending'));
    entry.c = massSettings(entry.aircraft_id, entry.c, version);
    if (+entry.throttle > +full(entry.aircraft_id, version)) entry.throttle = full(entry.aircraft_id, version);
    const note = await dropMissingLoadout(row, entry.aircraft_id, entry.c, version);
    if (!current()) return undefined;
    if (editing === entry.id) showSettings();
    renderEntries(); updateForm();
    return {note: note || (row.data_version_used ? `That build couldn't compile this aircraft, so it uses data from ${row.data_version_used}.` : '')};
  }

  // ---- Settings popup: applies as it changes; ✕, Escape or a click outside puts it away ------
  function showSettings() {
    const entry = entries.find(e => e.id === editing), a = aircraftOf(entry), variants = a?.ammunition?.variants;
    el('plane-fuel').value = entry.c.fuel_percent; el('plane-mass').value = entry.c.extra_mass_kg;
    el('plane-vehicle-field').hidden = !variants || Object.keys(variants).length < 2;
    el('plane-vehicle').innerHTML = variants ? Object.keys(variants).map(v => `<option value="${escapeText(v)}">${escapeText(a.vehicle_names?.find(n => n.vehicle_id === v)?.display_name || v)}</option>`).join('') : '';
    if (variants) el('plane-vehicle').value = entry.c.ammunition_vehicle;
    el('plane-apply-all').disabled = entries.length < 2; el('plane-apply-status').textContent = '';
    showLoadoutSummary(entry);
    el('plane-throttle').max = full(entry.aircraft_id, entry.version); el('plane-throttle').value = entry.throttle;
    syncSettings();
  }
  function syncSettings() {
    el('plane-fuel-label').textContent = el('plane-fuel').value + '%';
    el('plane-throttle-label').textContent = el('plane-throttle').value + '%';
  }
  function saveSettings() {
    syncSettings();
    const entry = entries.find(e => e.id === editing); if (!entry) return;
    entry.throttle = el('plane-throttle').value;
    const mass = Number(el('plane-mass').value);
    Object.assign(entry.c, {fuel_percent: Number(el('plane-fuel').value),
      extra_mass_kg: el('plane-mass').checkValidity() && Number.isFinite(mass) ? mass : entry.c.extra_mass_kg});
    el('plane-apply-status').textContent = '';
  }
  for (const id of ['plane-fuel', 'plane-throttle', 'plane-mass']) el(id).addEventListener('input', saveSettings);
  el('plane-vehicle').addEventListener('change', () => {
    // Each armament variant has its own pylons and presets (as the EM page).
    const entry = entries.find(e => e.id === editing); if (!entry) return;
    entry.c = massSettings(entry.aircraft_id, {...entry.c, ammunition_vehicle: el('plane-vehicle').value, loadout: '', stores_released: false}, entry.version);
    showLoadoutSummary(entry);
  });
  async function showLoadoutSummary(entry) {
    const a = aircraftOf(entry), c = entry.c;
    const available = !!a.loadouts?.supported, ammunition = !!ammunitionFor(a, c.ammunition_vehicle)?.supported;
    el('plane-loadout-edit').disabled = !available && !ammunition;
    el('plane-loadout-summary').textContent = loadoutText(c, cachedLoadouts(entry.aircraft_id, c.ammunition_vehicle, entry.version));
    el('plane-loadout-detail').textContent = c.loadout ? '' : c.default_ammunition ? ammunitionText(a, c.ammunition_vehicle) :
      available || ammunition ? 'Nothing aboard' : (a.loadouts?.reason || 'Loadouts are unavailable for this aircraft.');
    if (!available || !c.loadout) return;
    try {
      const info = await fetchLoadouts(entry.aircraft_id, c.ammunition_vehicle, entry.version);
      if (editing !== entry.id) return;
      el('plane-loadout-summary').textContent = loadoutText(c, info);
      el('plane-loadout-detail').textContent = loadoutFigures(loadoutTotals(c.loadout, info)) + (c.stores_released ? ' · stores released, racks stay' : '');
    } catch (cause) { el('plane-loadout-detail').textContent = 'Could not load loadouts: ' + cause.message; }
  }
  el('plane-loadout-edit').addEventListener('click', () => {
    const entry = entries.find(e => e.id === editing); if (!entry) return;
    openLoadoutEditor({id: entry.id, aircraft_id: entry.aircraft_id, version: entry.version, c: entry.c, applied() { if (editing === entry.id) { showLoadoutSummary(entry); saveSettings(); } }});
  });
  el('plane-apply-all').addEventListener('click', () => {
    // As the EM page: every setting to every entry; a loadout and armament variant only to the same aircraft.
    const source = entries.find(e => e.id === editing); if (!source || entries.length < 2) return;
    saveSettings();
    for (const entry of entries) {
      if (entry === source) continue;
      const same = entry.aircraft_id === source.aircraft_id;
      entry.c = massSettings(entry.aircraft_id, same ? structuredClone(source.c) : {...source.c, loadout: '', stores_released: false, ammunition_vehicle: ''}, entry.version);
      entry.throttle = String(Math.min(+source.throttle, +full(entry.aircraft_id, entry.version)));
    }
    el('plane-apply-status').textContent = `Applied to all ${entries.length} aircraft.`;
  });
  el('plane-dialog-close').addEventListener('click', () => el('plane-dialog').close());
  el('plane-dialog').addEventListener('click', event => { if (event.target === el('plane-dialog')) el('plane-dialog').close(); });
  el('plane-dialog').addEventListener('close', () => { editing = null; renderEntries(); });
  el('plane-search').addEventListener('input', filter);
  el('plane-search').addEventListener('keydown', event => { if (event.key === 'Enter') event.preventDefault(); });

  // ---- Form -----------------------------------------------------------------------------
  function updateForm() {
    const input = el('plane-altitude'), invalid = !input.checkValidity();
    el('plane-ready').textContent = invalid && entries.length ? `Altitude: ${input.validationMessage}` : '';
    el('plane-ready').hidden = !el('plane-ready').textContent;
    el('plane-run').disabled = running || !catalog() || !entries.length || invalid;
  }
  el('plane-form').addEventListener('input', event => { if (event.target.type === 'number') updateForm(); });
  el('plane-reset').addEventListener('click', () => {
    if (running) return;
    el('plane-altitude').value = defaults.altitude;
    for (const e of entries) Object.assign(e, {throttle: full(e.aircraft_id, e.version), c: massSettings(e.aircraft_id, null, e.version)});
    el('plane-search').value = ''; renderEntries(); filter(); updateForm(); render();
  });

  // ---- Calculation: one request per entry, the button filling as they finish --------------
  const RUN_LABEL = el('plane-run').innerHTML;
  function busy(on) {
    running = on; const run = el('plane-run');
    run.innerHTML = on ? 'Calculating…' : RUN_LABEL; run.classList.toggle('calculating', on); run.style.setProperty('--progress', '0%');
    el('plane-reset').disabled = on; renderEntries(); filter(); updateForm();
  }
  el('plane-form').addEventListener('submit', async event => {
    event.preventDefault(); if (running || !entries.length) return;
    const wanted = [...entries], run = {altitude: Number(el('plane-altitude').value)};
    showError(''); busy(true);
    try {
      for (const [i, entry] of wanted.entries()) {
        const id = key(entry, run);
        if (!results.has(id)) results.set(id, await api('/api/plane', {method: 'POST', headers: {'Content-Type': 'application/json'},
          body: JSON.stringify(body(entry, run))}, entry.version));
        el('plane-run').style.setProperty('--progress', `${(i + 1) / wanted.length * 100}%`);
      }
      shown = {...run, entries: wanted.map(e => structuredClone(e))};
      render();
      window.GameVersions?.changes(el('plane-version-changes'), wanted.map(e => ({id: e.aircraft_id, kind: 'fm', version: e.version, name: cleanName(aircraftOf(e)?.name || e.aircraft_id)})));
    } catch (error) { showError(error.message); }
    finally { busy(false); }
  });

  // ---- Page -------------------------------------------------------------------------------
  // The calculated entries still on the page, each with its result and colour.
  function displayed() {
    if (!shown) return [];
    return shown.entries.map(e => {
      const index = entries.findIndex(x => x.id === e.id);
      return index < 0 ? null : {entry: e, index, color: entryColors[index], result: results.get(key(e, shown))};
    }).filter(d => d?.result);
  }
  const entryName = d => `${cleanName(d.result.name)} · ${d.index + 1}${versionSuffix(d.entry)}`;
  function render() {
    const list = displayed(), flapped = list.filter(d => d.result.flaps.available);
    el('plane-empty').hidden = !!list.length; el('plane-tables').hidden = !list.length; el('plane-charts').hidden = !flapped.length;
    for (const id of ['plane-flaps-chart', 'plane-score-chart']) if (!flapped.length && el(id).classList.contains('js-plotly-plot')) Plotly.purge(el(id));
    if (!list.length) return;
    renderTables(list);
    if (flapped.length) { renderFlaps(flapped); renderScore(flapped); }
  }

  // One table per section, a column per aircraft. Rows keep each aircraft's own order: a row only some aircraft have
  // goes after the row it follows in its own list.
  function cell(row) {
    if (!row) return '—';
    const number = v => Number.isFinite(v) ? fmt(v, row.digits) : '—';
    const text = typeof row.value === 'string' ? escapeText(row.value) : (Array.isArray(row.value) ? row.value.map(number).join(' / ') : number(row.value)) + (row.unit ? ' ' + escapeText(row.unit) : '');
    return text + (row.note ? ` <small>${escapeText(row.note)}</small>` : '');
  }
  function renderTables(list) {
    const titles = [...new Set(list.flatMap(d => d.result.sections.map(s => s.title)))];
    el('plane-tables').classList.toggle('wide', list.length > 2);
    el('plane-tables').innerHTML = titles.map(title => {
      const sections = list.map(d => d.result.sections.find(s => s.title === title)?.rows || []);
      const labels = [];
      for (const rows of sections) {
        let at = -1;
        for (const r of rows) { const i = labels.indexOf(r.label); if (i >= 0) at = i; else labels.splice(++at, 0, r.label); }
      }
      return `<section><h3>${escapeText(title)}</h3><div class="plane-table"><table><thead><tr><th></th>${list.map(d =>
        `<th style="--entry-color:${d.color}">${escapeText(entryName(d))}</th>`).join('')}</tr></thead><tbody>${labels.map(label =>
        `<tr><td>${escapeText(label)}</td>${sections.map(rows => `<td>${cell(rows.find(r => r.label === label))}</td>`).join('')}</tr>`).join('')}</tbody></table></div></section>`;
    }).join('');
  }

  const axisStyle = {gridcolor: 'rgba(240,240,250,.09)', zeroline: false, tickfont: {size: 12}, separatethousands: true};
  const layoutBase = (traces, revision) => ({paper_bgcolor: '#000', plot_bgcolor: '#000', font: {family: 'WTSymbols, D-DIN, Bahnschrift, DIN Alternate, Segoe UI, sans-serif', color: '#b9b9c4', size: 12},
    margin: {l: 64, r: 22, t: Math.max(40, Math.ceil(traces.filter(t => t.showlegend !== false).length / 2) * 22), b: 54}, hovermode: 'closest',
    hoverlabel: {bgcolor: '#0c0c0f', bordercolor: 'rgba(240,240,250,.35)', font: {color: '#f4f4fc', size: 12}},
    legend: {orientation: 'h', x: 0, y: 1.02, yanchor: 'bottom', font: {size: 12}, bgcolor: 'rgba(0,0,0,0)', itemdoubleclick: false},
    uirevision: revision, dragmode: 'pan', modebar: {bgcolor: 'rgba(0,0,0,0)', color: '#8a8a96', activecolor: '#f0f0fa'}});
  function draw(id, list, traces, layout, name) {
    const chart = el(id);
    Plotly.react(chart, traces, layout, {responsive: true, displaylogo: false, scrollZoom: true, modeBarButtonsToRemove: ['select2d', 'lasso2d'],
      toImageButtonOptions: {format: 'png', scale: 2, filename: `WT-${name}-` + list.map(d => d.entry.aircraft_id).join('-vs-')},
      modeBarButtonsToAdd: [{name: 'Download data (JSON)', icon: DATA_ICONS.json, click: download}]});
  }
  // A drawn line with a hover point every whole unit of `along` (0: x, 1: y), straight between its knots.
  function dense(points, along) {
    const out = [];
    points.forEach((p, i) => {
      out.push(p);
      const q = points[i + 1]; if (!q || q[along] === p[along]) return;
      const lo = Math.min(p[along], q[along]), hi = Math.max(p[along], q[along]), inner = [];
      for (let v = Math.floor(lo) + 1; v < hi; v++) { const t = (v - p[along]) / (q[along] - p[along]); inner.push([p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t]); }
      out.push(...(q[along] < p[along] ? inner.reverse() : inner));
    });
    return out;
  }
  // Flap speeds: extension against indicated airspeed, only the limits that apply (Leo, 2026-11-19). Holding an
  // extension: the critical speed solid, the automatic retraction dashed, each where it is the lower one; extending:
  // the speed where the flaps stop travelling out, dotted.
  // The IAS limits are the game's forward IAS; a Mach limit is placed at its level-flight IAS at the page's altitude.
  const LIMITS = {critical: ['Critical speed', 'solid', 1], retraction: ['Auto retraction', 'dash', 0], retraction_mach: ['Auto retraction by Mach', 'dashdot', 0],
    extension: ['Extension limit', 'dot', 1]};
  function renderFlaps(list) {
    const traces = [];
    for (const d of list) {
      const name = entryName(d);
      for (const segment of d.result.flaps.limits) {
        const [label, dash, along] = LIMITS[segment.kind], points = dense(segment.points, along), group = `${d.entry.id}-${segment.kind}`;
        traces.push({type: 'scatter', mode: 'lines', x: points.map(p => p[0]), y: points.map(p => p[1]), name: `${name} · ${label.toLowerCase()}`,
          legendgroup: group, showlegend: !traces.some(t => t.legendgroup === group), line: {color: d.color, width: 2.4, dash}, hoverlabel: {bordercolor: d.color},
          ...(segment.kind === 'retraction_mach' && {customdata: points.map(p => p[0] / d.result.flaps.ias_per_mach)}),
          hovertemplate: `%{y:.0f}% · ${segment.kind === 'retraction_mach' ? 'Mach %{customdata:.2f} · ' : ''}%{x:,.0f} km/h IAS<extra>${escapeText(name)} · ${label.toLowerCase()}</extra>`});
      }
    }
    const layout = layoutBase(traces, 'flaps');
    layout.xaxis = {...axisStyle, title: {text: `Forward IAS (km/h) · ${fmt(shown.altitude, 0)} m`, standoff: 8, font: {size: 13}}, rangemode: 'tozero'};
    layout.yaxis = {...axisStyle, title: {text: 'Flap extension (%)', standoff: 8, font: {size: 13}}, range: [0, 104]};
    draw('plane-flaps-chart', list, traces, layout, 'flap-speeds');
  }
  // Flap efficiency (Leo, 2026-11-19): lift gain over drag gain against true airspeed for the chosen flap setting, at
  // the EM diagram's full Mouse-aim pull, over the speeds where the setting can be held. Every setting is calculated,
  // so choosing another one only redraws.
  const PULL = {aoa: 'Instructor AoA', control: 'full stick', wing: 'wing limit', stall: 'stall'};
  function renderScore(list) {
    const has = mode => list.some(d => d.result.flaps.efficiency?.modes[mode]?.rows.some(r => Number.isFinite(r?.score)));
    const inputs = [...root.querySelectorAll('input[name=plane-setting]')];
    for (const input of inputs) input.disabled = !has(input.value);
    // The setting stays as chosen while any aircraft has it; else the first one available.
    if (!inputs.some(i => i.checked && !i.disabled)) { const first = inputs.find(i => !i.disabled); if (first) first.checked = true; }
    const mode = inputs.find(i => i.checked && !i.disabled)?.value, traces = [], signed = v => (v >= 0 ? '+' : '−') + fmt(Math.abs(v), 1) + '%';
    for (const d of list) {
      const curve = d.result.flaps.efficiency?.modes[mode], name = entryName(d); if (!curve) continue;
      const rows = curve.rows, solved = r => Number.isFinite(r?.score);
      if (!rows.some(solved)) continue;
      traces.push({type: 'scatter', mode: 'lines', x: d.result.flaps.efficiency.speeds, y: rows.map(r => solved(r) ? r.score : null), name: `${name} · ${fmt(curve.percent, 0)}%`,
        connectgaps: false, line: {color: d.color, width: 2.4}, hoverlabel: {bordercolor: d.color},
        customdata: rows.map(r => solved(r) ? [signed(r.lift_gain_percent), signed(r.drag_gain_percent), fmt(r.load_g, 2), fmt(r.clean_load_g, 2), PULL[r.limiting] || ''] : []),
        hovertemplate: `%{y:.3f} · %{x:,.0f} km/h<br>Lift %{customdata[0]} · drag %{customdata[1]}<br>%{customdata[2]} g · flaps up %{customdata[3]} g<br>%{customdata[4]}<extra>${escapeText(name)}</extra>`});
    }
    const layout = layoutBase(traces, 'score-' + mode);
    layout.xaxis = {...axisStyle, title: {text: `True airspeed (km/h) · ${fmt(shown.altitude, 0)} m`, standoff: 8, font: {size: 13}}};
    // The first view covers the middle 96% of the scores and zero: where the drag gain passes through zero the ratio
    // runs off without bound (zoom out for all of it).
    const scores = traces.flatMap(t => t.y.filter(Number.isFinite)).sort((p, q) => p - q);
    const low = Math.min(0, scores[Math.floor(.02 * (scores.length - 1))] ?? 0), high = Math.max(0, scores[Math.ceil(.98 * (scores.length - 1))] ?? 1), pad = .08 * (high - low || 1);
    layout.yaxis = {...axisStyle, title: {text: 'Lift gain / drag gain', standoff: 8, font: {size: 13}}, range: [low - (low < 0 ? pad : 0), high + pad], zeroline: true, zerolinecolor: 'rgba(240,240,250,.35)'};
    draw('plane-score-chart', list, traces, layout, 'flap-efficiency');
  }
  root.querySelectorAll('input[name=plane-setting]').forEach(input => input.addEventListener('change', () => {
    const list = displayed().filter(d => d.result.flaps.available); if (list.length) renderScore(list);
  }));
  function download() {
    const list = displayed(); if (!list.length) return;
    const link = document.createElement('a');
    link.href = URL.createObjectURL(new Blob([JSON.stringify(list.map(d => ({entry: d.index + 1, ...d.result})), null, 1)], {type: 'application/json'}));
    link.download = `WT-plane-${list.map(d => d.result.aircraft_id).join('-vs-')}.json`;
    link.click(); setTimeout(() => URL.revokeObjectURL(link.href), 1000);
  }

  // The catalog arrives with the EM page's start; the list fills once it is there.
  function whenReady() { if (catalog()) populate(); else setTimeout(whenReady, 300); }
  document.addEventListener('game-versions-ready', () => { if (catalog()) populate(); });
  document.addEventListener('tool-tab-leave', event => {
    if (event.detail === 'plane' && catalog())
      sidebarRows.aircraft = entries.map(({id, aircraft_id, version}) => ({id, aircraft_id, ...(version ? {version} : {})}));
  });
  const resize = () => { for (const id of ['plane-flaps-chart', 'plane-score-chart']) if (el(id).data && el(id).clientWidth) Plotly.Plots.resize(el(id)); };
  document.addEventListener('tool-tab-change', event => {
    if (event.detail !== 'plane') return;
    if (sidebarRows.aircraft && catalog()) {
      const previous = new Map(entries.map(e => [e.id, e]));
      entries = sidebarRows.aircraft.map(row => {
        const old = previous.get(row.id);
        if (old && old.version === row.version) return old;
        return {id: row.id, aircraft_id: row.aircraft_id, ...(row.version ? {version: row.version} : {}),
          throttle: String(Math.min(+(old?.throttle ?? 110), +full(row.aircraft_id, row.version))), c: massSettings(row.aircraft_id, old?.c, row.version)};
      });
      if (!entries.some(e => e.id === editing)) editing = null;
      renderEntries(); filter(); updateForm(); render();
    }
    requestAnimationFrame(resize);
  });
  new ResizeObserver(() => { if (!root.hidden) resize(); }).observe(el('plane-charts'));
  updateForm(); whenReady();
})();
