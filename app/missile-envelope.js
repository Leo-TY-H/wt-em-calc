(() => {
  'use strict';
  const el = id => document.getElementById(id);
  const root = el('envelope-workspace');
  if (!root) return;
  const apiBase = String((window.EM_CONFIG || {}).apiBase || '').replace(/\/$/, '');
  async function request(path, options = {}) {
    const response = await fetch(apiBase ? apiBase + path : '.' + path, options);
    const body = await response.json();
    if (!response.ok) throw new Error(body.error || `Request failed (${response.status})`);
    return body;
  }
  const REMOVE = '<svg viewBox="0 0 16 16" width="14" height="14" aria-hidden="true" fill="none" stroke="currentColor" stroke-width="1.4" stroke-linejoin="round"><path d="M2.5 4.5h11M6 4.5v-2h4v2M4 4.5l.7 9h6.6l.7-9M6.7 7v4.5M9.3 7v4.5"/></svg>';
  const escapeHtml = text => String(text).replace(/[&<>"']/g, c => ({'&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;'}[c]));
  // As Python's math.radians (x * (pi / 180)), so a replayed shot starts from the searched sample's exact numbers.
  const radians = a => a * (Math.PI / 180);
  // Base directions per layer; the search adds directions where the envelope bends, splits or ends.
  const qualities = {
    quick: {step: 30, edge: 1, label: 'Quick · 30° directions, edges to 1°'},
    standard: {step: 15, edge: .5, label: 'Standard · 15° directions, edges to 0.5°'},
    fine: {step: 10, edge: .25, label: 'Detailed · 10° directions, edges to 0.25°'}};
  const defaults = {missile: null, quality: 'standard',
    target: {speed: 1200},
    launcher: {speed: 1200},
    // No-escape zone (Leo, 2026-10-26): the target breaks away at 9 g the moment the missile is fired.
    escape: {on: false, load: 9, delay: 0},
    // One launcher (surface-to-air: target) altitude per calculation (Leo, 2026-10-27: one altitude at a time).
    altitude: 4000};
  // Launcher at centre (shooter-centred launch acceptable region, Leo 2026-10-26): the launcher keeps its heading
  // and the target, at its altitude, flies one of these ways; 'crossing' is a heading relative to the launcher's.
  const motions = {hot: {label: 'Toward me', text: 'Target flying toward you'}, cold: {label: 'Away from me', text: 'Target flying away from you'},
    same: {label: 'Same way as me', text: 'Target flying your heading'}, opposite: {label: 'Opposite way', text: 'Target flying the opposite heading'},
    crossing: {label: 'At a heading…', text: 'Target heading'}};
  // One colour per added missile, as on the trajectory page.
  const COLORS = ['#38c9d7', '#b79aff', '#91d477', '#ee8eb6', '#f0d367', '#e0b0ff', '#cfb296', '#f08070'], MAX_MISSILES = 8;
  const numberField = (id, label, unit, value, min, max, step = 'any') =>
    `<label class="field">${label}<div class="input-unit"><input id="${id}" type="number" min="${min}" max="${max}" step="${step}" value="${value}" required aria-label="${escapeHtml(label)}"><span>${unit}</span></div></label>`;

  // Laid out like the EM page (Leo, 2026-10-26): picker + card, plain sections, a sticky Calculate button that
  // fills as progress, the chart unboxed with the same mouse interaction and toolbar downloads, details below on click.
  root.innerHTML = `<aside class="missile-sidebar envelope-sidebar"><form id="envelope-form" novalidate>
    <fieldset id="envelope-picker"><legend class="visually-hidden">Add missiles</legend>
      <input id="envelope-search" type="search" placeholder="Search missiles…" autocomplete="off" aria-label="Search missiles" aria-controls="envelope-list">
      <div id="envelope-list" class="aircraft-list" aria-label="Available missiles"></div>
      <p id="envelope-search-empty" class="hint" hidden>No missiles match your search.</p>
      <p id="envelope-catalog-note" class="hint" role="status">Loading missiles…</p>
      <div id="envelope-selected" class="envelope-selected" aria-live="polite"></div>
      <button id="envelope-retry" class="text-button" type="button" hidden>Retry loading missiles</button>
    </fieldset>
    <span class="eyebrow">SCENARIO</span>
    <div class="mode-switch" role="radiogroup" aria-label="Launch platform" hidden><label><input id="envelope-platform-air" type="radio" name="envelope-platform" value="air" checked><span>Air-to-air</span></label><label><input id="envelope-platform-ground" type="radio" name="envelope-platform" value="ground"><span>Surface-to-air</span></label></div>
    <div class="mode-switch" role="radiogroup" aria-label="Plot centre"><label><input id="envelope-centre-target" type="radio" name="envelope-centre" value="target" checked><span>Target POV</span></label><label><input id="envelope-centre-launcher" type="radio" name="envelope-centre" value="launcher"><span>Launcher POV</span></label></div>
    <div id="envelope-motion" class="envelope-motion" hidden><label class="field">Target flies<select id="envelope-target-heading">${Object.entries(motions).map(([k, m]) => `<option value="${k}">${m.label}</option>`).join('')}</select></label>${numberField('envelope-crossing', 'Target heading', '°', 90, -180, 180)}<label class="field">Launcher gimbal limit<div class="input-unit"><input id="envelope-gimbal" type="number" min="1" max="180" step="any" placeholder="None" aria-label="Launcher gimbal limit"><span>±°</span></div></label></div>
    <p id="envelope-centre-note" class="hint" hidden>A ground launcher turns toward the target, so this is the target-centred envelope drawn around the launcher, with the target flying up the plot.</p>
    <div class="envelope-pair">${numberField('envelope-launcher-speed', 'Launcher speed', 'km/h', defaults.launcher.speed, 0, 7200)}${numberField('envelope-target-speed', 'Target speed', 'km/h', defaults.target.speed, 0, 7200)}</div>
    <label class="field"><span id="envelope-altitude-label">Altitude</span><div class="input-unit"><input id="envelope-altitude" type="number" min="50" max="25000" step="any" value="${defaults.altitude}" required aria-label="Altitude"><span>m</span></div></label>
    <span class="eyebrow">NO-ESCAPE ZONE</span>
    <label class="check-row envelope-nez-toggle"><input id="envelope-nez" type="checkbox"><span>Show the no-escape zone</span></label>
    <div id="envelope-nez-settings"><div class="envelope-pair">${numberField('envelope-escape-load', 'Escape load', 'g', defaults.escape.load, 1.5, 15)}${numberField('envelope-escape-delay', 'Reaction delay', 's', defaults.escape.delay, 0, 60)}</div>
</div>
    <span class="eyebrow">SEARCH</span>
    <label class="field">Resolution<select id="envelope-quality">${Object.entries(qualities).map(([k, q]) => `<option value="${k}"${k === defaults.quality ? ' selected' : ''}>${q.label}</option>`).join('')}</select></label>
    <div class="sidebar-actions">
      <p id="envelope-ready" class="hint" hidden></p>
      <div class="action-row"><button id="envelope-run" class="primary-button" type="submit" disabled>Calculate</button><button id="envelope-reset" class="secondary-button reset-button" type="button">Reset</button></div>
      <button id="envelope-cancel" class="secondary-button" type="button" hidden>Stop</button>
    </div>
  </form></aside>
  <div class="missile-main envelope-main">
    <div id="envelope-error" class="error" role="alert" hidden></div>
    <div class="envelope-view-controls" hidden><label><input id="envelope-show-rays" type="checkbox"> Directions</label><label><input id="envelope-show-samples" type="checkbox"> Samples</label></div>
    <div id="envelope-chart" aria-label="Missile launch envelope"><div class="empty"><span class="empty-glyph">◎</span><h2>Add a missile and press Calculate</h2></div></div>
    <section id="envelope-inspector" class="inspector envelope-inspector" aria-live="polite" hidden><div class="inspector-heading"><h2 id="envelope-inspect-title"></h2><button id="envelope-open-simulator" class="secondary-button" type="button" hidden>Fly this shot in the simulator ↗</button></div><div id="envelope-inspect"></div></section>
    <section class="envelope-table-card" hidden><span class="eyebrow">RANGES BY ALTITUDE</span><div class="envelope-table-wrap"><table id="envelope-table"><thead></thead><tbody></tbody></table></div></section>
    <div hidden><a id="envelope-download-json" aria-disabled="true">Envelope JSON</a><a id="envelope-download-csv" aria-disabled="true">Directions CSV</a></div>
    <footer><span></span><span id="envelope-status" role="status" aria-live="polite"></span></footer>
  </div>`;

  // Each added missile gets its own envelope run; the geometry helpers read the run in use (rays, submitted, jobInfo, cache).
  let meta = null, loading = null, selected = [], runs = [], job = null, cancelRequested = false;
  // Results are kept for each plot centre, so switching between them never recalculates.
  const runSets = {target: [], launcher: []}, statusText = {target: '', launcher: ''};
  // rays: searched against the target holding its course; escapeRays: against the target breaking away (no-escape zone).
  let rays = new Map(), escapeRays = new Map(), submitted = null, summary = null;
  let jobInfo = null, polling = false, renderTimer = null, lastRender = 0, downloadUrls = [], inspected = null, cache = null;

  const showError = text => { el('envelope-error').textContent = text; el('envelope-error').hidden = !text; };
  const value = id => Number(el(id).value);
  const km = m => m >= 10000 ? (m / 1000).toFixed(1) + ' km' : m >= 1000 ? (m / 1000).toFixed(2) + ' km' : Math.round(m) + ' m';
  const fmt = (v, d = 0) => Number.isFinite(v) ? v.toLocaleString(undefined, {maximumFractionDigits: d, minimumFractionDigits: d}) : '—';
  const missileName = id => meta?.missiles.find(m => m.id === id)?.name || id;
  const outcomes = {proximity_event: 'proximity fuze', lifetime: 'lifetime ended', distance_limit: 'distance limit',
    tracking_timeout: 'lost track (timeout)', ground: 'hit the ground', target_ground: 'target reached the ground', unreachable: 'out of energy to reach',
    time_limit: 'time limit', error: 'simulation error'};
  const outcomeLabel = code => outcomes[code] || String(code).replaceAll('_', ' ');
  const closestText = s => s.outcome === 'unreachable' ? `≤ ${km(s.closest_m)} so far` : km(s.closest_m);

  const layers = () => [value('envelope-altitude')];
  // Surface-to-air (Leo, 2026-10-26): the launcher stands on the ground and the altitude is the target's.
  const ground = () => el('envelope-platform-ground').checked;
  const centreOn = () => el('envelope-centre-launcher').checked ? 'launcher' : 'target';
  // The searched frame: a ground launcher turns toward the target, so around it the target frame is only redrawn.
  const frameOf = () => centreOn() === 'launcher' && !ground() ? 'launcher' : 'target';
  function platformControls() {
    const launcherSpeed = el('envelope-launcher-speed').closest('.field');
    launcherSpeed.hidden = ground(); el('envelope-launcher-speed').disabled = ground();
    el('envelope-altitude-label').textContent = ground() ? 'Target altitude' : 'Altitude';
    el('envelope-altitude').setAttribute('aria-label', el('envelope-altitude-label').textContent);
    const motion = frameOf() === 'launcher', crossing = el('envelope-target-heading').value === 'crossing';
    el('envelope-motion').hidden = !motion;
    el('envelope-crossing').closest('.field').hidden = !crossing; el('envelope-crossing').disabled = !motion || !crossing;
    el('envelope-gimbal').disabled = !motion;
    el('envelope-centre-note').hidden = !(centreOn() === 'launcher' && ground());
  }
  for (const id of ['envelope-platform-air', 'envelope-platform-ground', 'envelope-target-heading']) el(id).addEventListener('change', () => { platformControls(); updateForm(); });
  for (const id of ['envelope-centre-target', 'envelope-centre-launcher']) el(id).addEventListener('change', () => { platformControls(); updateForm(); showCentre(); });
  function escapeControls() {
    const on = el('envelope-nez').checked;
    el('envelope-nez-settings').hidden = !on;
    for (const id of ['envelope-escape-load', 'envelope-escape-delay']) el(id).disabled = !on;
  }
  el('envelope-nez').addEventListener('change', () => { escapeControls(); updateForm(); });
  function inputs() {
    const frame = frameOf(), rule = el('envelope-target-heading').value, gimbal = el('envelope-gimbal').value.trim();
    return {quality: el('envelope-quality').value, layers_m: layers(), frame,
      target: {speed_mps: value('envelope-target-speed') / 3.6, ...(frame === 'launcher' ? {heading: rule === 'crossing' ? value('envelope-crossing') : rule} : {})},
      launcher: ground() ? {platform: 'ground', speed_mps: 0} : {platform: 'air', speed_mps: value('envelope-launcher-speed') / 3.6,
        ...(frame === 'launcher' ? {gimbal_deg: gimbal ? Number(gimbal) : null} : {})},
      escape: el('envelope-nez').checked ? {load_g: value('envelope-escape-load'), delay_s: value('envelope-escape-delay')} : null};
  }
  function updateForm() {
    const fields = [...el('envelope-form').querySelectorAll('input[type=number]:not(:disabled)')];
    const invalid = fields.find(field => !field.checkValidity());
    const problem = invalid ? `${invalid.getAttribute('aria-label')}: ${invalid.validationMessage}` : '';
    // Only problems are shown; the button says the rest.
    el('envelope-ready').textContent = meta && selected.length && !job ? problem : '';
    el('envelope-ready').hidden = !el('envelope-ready').textContent;
    el('envelope-run').disabled = !!job || !meta || !selected.length || !!problem;
  }
  async function load() {
    if (loading) return loading;
    loading = (async () => {
      try {
        showError(''); el('envelope-retry').hidden = true;
        meta = await request('/api/missiles/meta');
        populate(); updateForm();
      } catch (error) {
        meta = null; el('envelope-retry').hidden = false;
        showError('Could not load missile configurations. Check that the simulator server is running, then retry. ' + error.message);
      } finally { loading = null; }
    })();
    return loading;
  }
  const normalize = text => text.toLowerCase().normalize('NFKD').replace(/[^a-z0-9]/g, '');
  function populate() {
    el('envelope-list').replaceChildren(...meta.missiles.map(item => {
      const row = document.createElement('div'); row.className = 'aircraft-option';
      const family = item.family === 'optical' ? 'Optical / infrared' : 'Radar';
      row.dataset.search = item.id + ' ' + item.name + ' ' + family;
      const label = document.createElement('span'), name = document.createElement('strong'), detail = document.createElement('small'), code = document.createElement('code');
      name.textContent = item.name; code.textContent = item.id; detail.append(code); label.append(name, detail); label.title = `${item.name} [${item.id}]`;
      const button = document.createElement('button'); button.type = 'button'; button.dataset.pick = item.id;
      button.setAttribute('aria-label', 'Add ' + item.name);
      row.append(label, button); return row;
    }));
    el('envelope-catalog-note').hidden = true;
    filter(); renderSelected();
  }
  function filter() {
    const query = normalize(el('envelope-search').value); let visible = 0;
    for (const row of el('envelope-list').children) { row.hidden = !normalize(row.dataset.search).includes(query); if (!row.hidden) visible++; }
    el('envelope-search-empty').hidden = !meta || visible > 0;
    for (const button of el('envelope-list').querySelectorAll('[data-pick]')) {
      const added = selected.includes(button.dataset.pick);
      button.textContent = added ? 'Added' : 'Add';
      button.disabled = !!job || added || selected.length >= MAX_MISSILES;
    }
  }
  function renderSelected() {
    el('envelope-selected').replaceChildren(...selected.map((id, i) => {
      const item = meta?.missiles.find(m => m.id === id), name = item?.name || id;
      const card = document.createElement('article'); card.className = 'aircraft-entry missile-entry'; card.style.setProperty('--entry-color', COLORS[i]);
      card.innerHTML = `<div class="entry-heading"><span class="entry-number">${i + 1}</span><strong>${escapeHtml(name)}</strong><code>${escapeHtml(id)}</code></div>
        <div class="entry-actions"><button type="button" class="entry-icon" data-remove="${escapeHtml(id)}" title="Remove" aria-label="Remove ${escapeHtml(name)}">${REMOVE}</button></div>`;
      card.querySelector('button').disabled = !!job;
      return card;
    }));
  }
  el('envelope-selected').addEventListener('click', event => {
    const button = event.target.closest('[data-remove]'); if (!button || job) return;
    selected = selected.filter(id => id !== button.dataset.remove); renderSelected(); filter(); updateForm();
  });
  el('envelope-list').addEventListener('click', event => {
    const button = event.target.closest('[data-pick]');
    if (!button || button.disabled) return;
    selected.push(button.dataset.pick); renderSelected(); filter(); updateForm();
  });
  el('envelope-search').addEventListener('input', filter);
  el('envelope-search').addEventListener('keydown', event => { if (event.key === 'Enter') event.preventDefault(); });
  el('envelope-retry').addEventListener('click', load);
  el('envelope-form').addEventListener('input', event => { if (event.target.id !== 'envelope-search') updateForm(); });
  el('envelope-form').addEventListener('change', event => { if (event.target.id !== 'envelope-search') updateForm(); });
  function reset() {
    for (const [id, v] of [['envelope-target-speed', defaults.target.speed], ['envelope-launcher-speed', defaults.launcher.speed], ['envelope-crossing', 90],
      ['envelope-escape-load', defaults.escape.load], ['envelope-escape-delay', defaults.escape.delay], ['envelope-altitude', defaults.altitude]]) el(id).value = v;
    el('envelope-platform-air').checked = true; el('envelope-target-heading').value = 'hot'; el('envelope-gimbal').value = ''; platformControls();
    el('envelope-nez').checked = defaults.escape.on; escapeControls();
    el('envelope-quality').value = defaults.quality; el('envelope-search').value = '';
    if (meta) filter();
    updateForm();
  }
  el('envelope-reset').addEventListener('click', () => { if (!job) reset(); });

  // ---- Calculation ----------------------------------------------------------------
  const RUN_LABEL = el('envelope-run').innerHTML;
  function busy(on) {
    el('envelope-cancel').hidden = !on; el('envelope-reset').disabled = on;
    for (const id of ['envelope-centre-target', 'envelope-centre-launcher']) el(id).disabled = on;
    const run = el('envelope-run');
    run.innerHTML = on ? 'Calculating…' : RUN_LABEL; run.classList.toggle('calculating', on); run.style.setProperty('--progress', '0%');
    filter(); renderSelected(); updateForm();
  }
  // The run whose results the geometry helpers read.
  function use(run) { rays = run.rays; escapeRays = run.escapes; submitted = run.submitted; jobInfo = run.jobInfo; summary = run.summary; cache = run.cache; }
  const received = run => run.rays.size + run.escapes.size;
  function within(run, fn) { use(run); try { return fn(); } finally { run.cache = cache; } }
  el('envelope-form').addEventListener('submit', async event => {
    event.preventDefault();
    if (job || el('envelope-run').disabled) return;
    const base = inputs(), centre = centreOn();
    runs = runSets[centre] = selected.map((missile, index) => ({missile, index, color: COLORS[index], submitted: {...base, missile}, rays: new Map(), escapes: new Map(), jobInfo: null, summary: null, cache: null}));
    showError(''); inspected = null; cancelRequested = false; el('envelope-status').textContent = '';
    renderInspector(); setExports(false); render().catch(error => showError(error.message));
    job = 'starting'; busy(true);
    const started = performance.now(), errors = [];
    for (const [k, run] of runs.entries()) {
      if (cancelRequested) break;
      try {
        const created = await request('/api/missiles/envelopes', {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(run.submitted)});
        job = created.id;
        const outcome = await poll(run, k);
        if (outcome === 'failed') errors.push(`${missileName(run.missile)}: ${run.jobInfo?.error || 'envelope calculation failed'}`);
      } catch (error) { errors.push(`${missileName(run.missile)}: ${error.message}`); }
    }
    job = null; el('envelope-cancel').disabled = false; busy(false);
    const directions = runs.reduce((n, run) => n + received(run), 0), sims = runs.reduce((n, run) => n + (run.summary?.simulations || 0), 0);
    el('envelope-status').textContent = statusText[centre] = cancelRequested ? `Stopped · ${fmt(directions)} directions kept`
      : `Calculated in ${fmt((performance.now() - started) / 1000, 1)} s · ${fmt(directions)} directions, ${fmt(sims)} simulations`;
    if (errors.length) showError(errors.join(' · '));
    await render(); setExports(directions > 0);
  });
  el('envelope-cancel').addEventListener('click', async () => {
    if (!job) return;
    cancelRequested = true; el('envelope-cancel').disabled = true;
    if (job !== 'starting') try { await request('/api/missiles/envelopes/' + job + '/cancel', {method: 'POST'}); } catch (error) { showError(error.message); }
  });
  // Follow one run to its end; the Calculate button fills over all runs.
  async function poll(run, k) {
    while (true) {
      let status;
      try { status = await request(`/api/missiles/envelopes/${job}?since=${received(run)}`); }
      catch (error) { await new Promise(r => setTimeout(r, 1500)); continue; }
      run.jobInfo = status;
      for (const ray of status.rays) (ray.motion === 'escape' ? run.escapes : run.rays).set(ray.key.join(','), ray);
      if (status.rays.length) run.cache = null;
      const p = status.progress, total = Math.max(p?.rays_total || status.rays_total || 1, received(run));
      el('envelope-run').style.setProperty('--progress', `${Math.min(99, 100 * (k + received(run) / total) / runs.length)}%`);
      if (status.rays.length) scheduleRender();
      if (['complete', 'failed', 'cancelled'].includes(status.status)) { run.summary = status.summary || null; return status.status; }
      await new Promise(r => setTimeout(r, 700));
    }
  }
  function scheduleRender() {
    if (renderTimer) return;
    const wait = Math.max(0, 1000 - (performance.now() - lastRender));
    renderTimer = setTimeout(async () => { renderTimer = null; lastRender = performance.now(); try { await render(); } catch (error) { showError(error.message); } }, wait);
  }

  // ---- Geometry ---------------------------------------------------------------------
  const wrap = a => { let x = ((a + 180) % 360 + 360) % 360 - 180; return x === -180 ? 180 : x; };
  function mirrored() { return jobInfo ? jobInfo.mirrored : true; }
  const GROUND_LAUNCH_M = 2;   // as scripts/missile_envelope.py
  const baseStep = () => jobInfo?.config?.azimuth_step_deg || qualities[submitted.quality].step;
  // Launcher and target states at release for a sample (mirrors scripts/missile_envelope.py states()).
  function states(c, layer, azimuth, distance) {
    const a = radians(azimuth);
    const direction = (h, p, s) => [s * Math.cos(radians(p)) * Math.cos(radians(h)), s * Math.sin(radians(p)), s * Math.cos(radians(p)) * Math.sin(radians(h))];
    if (c.frame === 'launcher') {
      // The launcher keeps flying along +X; the target sits off its nose (Python's _wrap, so replays match exactly).
      const rule = c.target.heading, raw = typeof rule === 'string' ? {hot: azimuth + 180, cold: azimuth, same: 0, opposite: 180}[rule] : rule;
      let h = raw % 360; if (h > 180) h -= 360; if (h <= -180) h += 360;
      return {launcher: {position: [0, layer, 0], velocity: direction(0, 0, c.launcher.speed_mps), angles: [0, 0, 0]},
        target: {position: [distance * Math.cos(a), layer, distance * Math.sin(a)], velocity: direction(h, 0, c.target.speed_mps), angles: [h, 0, 0]}};
    }
    const h = wrap(azimuth + 180), target = {position: [-distance * Math.cos(a), layer, -distance * Math.sin(a)], velocity: direction(0, 0, c.target.speed_mps), angles: [0, 0, 0]};
    if (c.launcher.platform === 'ground') {
      const p = Math.atan2(layer - GROUND_LAUNCH_M, distance) * (180 / Math.PI);   // as math.degrees
      return {launcher: {position: [0, GROUND_LAUNCH_M, 0], velocity: [0, 0, 0], angles: [h, p, 0]}, target};
    }
    return {launcher: {position: [0, layer, 0], velocity: direction(h, 0, c.launcher.speed_mps), angles: [h, 0, 0]}, target};
  }
  // Bearings name the run in use: from the target's nose, or off the launcher's nose (positive to its right, +Z).
  const launcherFrame = () => submitted?.frame === 'launcher';
  // Searched bearings need up to two decimals (edges to 0.25°), clicked ones one.
  const places = a => Math.abs(a - Math.round(a)) < 1e-9 ? 0 : Math.abs(a * 10 - Math.round(a * 10)) < 1e-6 ? 1 : 2;
  const bearingName = az => {
    const a = Math.abs(wrap(az));
    if (launcherFrame()) {
      const side = a > 0 && a < 180 ? (wrap(az) > 0 ? ' right' : ' left') : '';
      return `Target ${fmt(a, places(a))}°${side} off the launcher's nose${a === 0 ? ' (dead ahead)' : a === 180 ? ' (behind)' : ''}`;
    }
    const side = wrap(az) > 0 ? ' · +Z side' : wrap(az) < 0 ? ' · −Z side' : '';
    return `${fmt(a, places(a))}° from the target's nose${a === 0 ? ' (head-on)' : a === 180 ? ' (tail chase)' : a === 90 ? ' (beam)' + side : side}`;
  };
  const angleText = t => launcherFrame() ? `${fmt(Math.abs(t), 1)}°${Math.abs(t) > 0 && Math.abs(t) < 180 ? (t > 0 ? ' right' : ' left') : ''} off the nose`
    : `${fmt(Math.abs(t), 1)}° from the target's nose`;

  // ---- Envelope field ---------------------------------------------------------------
  // The region is where s > 0. Along each searched direction s is the signed distance
  // in log(r + 100 m) to the nearest boundary of its hit intervals; between
  // directions and between layers it is interpolated with monotone cubic (PCHIP)
  // splines, so the outline passes through every searched boundary and bends
  // smoothly between them without overshooting.
  // A direction without hits counts as EMPTY below zero, so the region closes most
  // of the way toward it; the search places such directions next to the edge.
  const LOG = r => Math.log(Math.max(r, 0) + 100);
  const CLAMP = 3, EMPTY = -.5;
  function signed(iv, L) {
    if (!iv.length) return EMPTY;
    let s = -Infinity;
    for (let i = 0; i < iv.length; i += 2) s = Math.max(s, Math.min(L - iv[i], iv[i + 1] - L));
    return Math.max(-CLAMP, Math.min(CLAMP, s));
  }
  function pchipSlope(h0, h1, d0, d1) {
    if (d0 * d1 <= 0) return 0;
    const w0 = 2 * h1 + h0, w1 = h1 + 2 * h0;
    return (w0 + w1) / (w0 / d0 + w1 / d1);
  }
  // Interpolate values ys at increasing positions xs; constant beyond the ends.
  function pchip(xs, ys, x) {
    const n = xs.length;
    if (n === 1 || x <= xs[0]) return ys[0];
    if (x >= xs[n - 1]) return ys[n - 1];
    let lo = 0, hi = n - 1;
    while (hi - lo > 1) { const m = (lo + hi) >> 1; if (xs[m] <= x) lo = m; else hi = m; }
    const k = lo, x0 = xs[k], x1 = xs[k + 1], h = x1 - x0, y0 = ys[k], y1 = ys[k + 1], d = (y1 - y0) / h;
    const m0 = k > 0 ? pchipSlope(x0 - xs[k - 1], h, (y0 - ys[k - 1]) / (x0 - xs[k - 1]), d) : d;
    const m1 = k + 2 < n ? pchipSlope(h, xs[k + 2] - x1, d, (ys[k + 2] - y1) / (xs[k + 2] - x1)) : d;
    const t = (x - x0) / h, t2 = t * t, t3 = t2 * t;
    return (2 * t3 - 3 * t2 + 1) * y0 + (t3 - 2 * t2 + t) * h * m0 + (-2 * t3 + 3 * t2) * y1 + (t3 - t2) * h * m1;
  }
  // Searched directions of each layer over the full circle, as flat log-interval arrays.
  function linesOf(source) {
    const mirror = mirrored(), byLayer = submitted.layers_m.map(() => []);
    for (const ray of source.values()) {
      const iv = ray.intervals.flatMap(i => [LOG(i.start_m), LOG(i.end_m)]);
      const az = wrap(ray.azimuth_deg);
      byLayer[ray.key[0]].push({az, ray, iv});
      if (mirror && az > 0 && az < 180) byLayer[ray.key[0]].push({az: -az, ray, iv});
    }
    return byLayer.map(list => {
      list.sort((a, b) => a.az - b.az);
      return {az: list.map(d => d.az), entries: list};
    });
  }
  // escape: the directions searched against the target breaking away, when the no-escape zone was requested.
  function field() {
    if (cache) return cache;
    let radius = 1000;
    for (const ray of rays.values()) if (ray.max_range_m) radius = Math.max(radius, ray.max_range_m);
    cache = {lines: linesOf(rays), escape: submitted.escape && escapeRays.size ? linesOf(escapeRays) : null, radius: radius * 1.07, polar: new Map()};
    return cache;
  }
  // The no-escape zone is where both searches hit: s = min(s straight, s escaping).
  const hasNez = li => !!field().escape?.[li]?.az.length && !!field().lines[li]?.az.length;
  function nezColumn(li, az, Ls) {
    const f = field(), a = columnValues(f.lines[li], az, Ls);
    if (!f.escape) return Ls.map(() => EMPTY);
    const b = columnValues(f.escape[li], az, Ls);
    return a.map((v, i) => Math.min(v, b[i]));
  }
  const intersect = (a, b) => {
    const out = [];
    for (const p of a) for (const q of b) { const s = Math.max(p.start_m, q.start_m), e = Math.min(p.end_m, q.end_m); if (e > s) out.push({start_m: s, end_m: e}); }
    return out.sort((x, y) => x.start_m - y.start_m);
  };
  // No-escape intervals (m) along one bearing: from the two searched directions when both lie on it, else from the drawn field.
  function nezIntervals(li, az) {
    const a = rayNear(li, az), b = rayNear(li, az, true);
    if (a && b) return intersect(a.intervals, b.intervals);
    if (!hasNez(li)) return null;
    const f = field(), L0 = LOG(0), L1 = LOG(f.radius), n = 600, Ls = Array.from({length: n}, (_, j) => L0 + (L1 - L0) * j / (n - 1));
    const s = nezColumn(li, az, Ls), unlog = L => Math.max(0, Math.exp(L) - 100), out = [];
    let start = null;
    for (let j = 0; j < n; j++) {
      const cross = () => Ls[j - 1] + (Ls[j] - Ls[j - 1]) * s[j - 1] / (s[j - 1] - s[j]);
      if (s[j] > 0 && start === null) start = j ? cross() : Ls[0];
      if (s[j] <= 0 && start !== null) { out.push({start_m: unlog(start), end_m: unlog(cross())}); start = null; }
    }
    if (start !== null) out.push({start_m: unlog(start), end_m: unlog(Ls.at(-1))});
    return out;
  }
  const nezText = iv => iv === null ? 'not searched yet' : iv.length ? iv.map(i => `${km(i.start_m)} – ${km(i.end_m)}`).join(', ') : 'none';
  // s along one bearing az (deg) of a layer at the log distances Ls.
  function columnValues(line, az, Ls) {
    if (!line.az.length) return Ls.map(() => EMPTY);
    if (line.az.length === 1) return Ls.map(L => signed(line.entries[0].iv, L));
    // PCHIP over the four directions around az; the others do not influence it.
    const n = line.az.length, x = line.az[0] + ((az - line.az[0]) % 360 + 360) % 360;
    let k = n - 1;
    if (x < line.az[n - 1]) { let lo = 0, hi = n - 1; while (hi - lo > 1) { const m = (lo + hi) >> 1; if (line.az[m] <= x) lo = m; else hi = m; } k = lo; }
    const xs = [], ivs = [];
    for (let i = k - 1; i <= k + 2; i++) {
      const j = ((i % n) + n) % n, turns = Math.floor(i / n);
      xs.push(line.az[j] + turns * 360); ivs.push(line.entries[j].iv);
    }
    return Ls.map(L => pchip(xs, ivs.map(iv => signed(iv, L)), x));
  }
  const layerValue = (line, r, az) => columnValues(line, az, [LOG(r)])[0];
  // s at one horizontal point and altitude, interpolated across layers.
  function volumeValue(values, altitude) {
    return pchip(submitted.layers_m, values, altitude);
  }
  // s of one layer on a polar grid: bearings -180..180 deg by step, distances
  // uniform in log(r + 100 m) out to the plot radius. Cached per layer; nez gives
  // the no-escape zone's s on the same grid.
  function polarField(li, nez = false) {
    const f = field(), id = nez ? 'nez' + li : li;
    if (f.polar.has(id)) return f.polar.get(id);
    let grid;
    if (nez) {
      const base = polarField(li);
      const columns = base.thetas.map((t, i) => { const e = columnValues(f.escape[li], t, base.Ls); return base.columns[i].map((v, j) => Math.min(v, e[j])); });
      grid = {...base, columns, z: base.Ls.map((_, j) => columns.map(col => col[j]))};
    } else {
      const step = submitted.layers_m.length > 8 ? 1 : .5, thetas = [], Ls = [];
      for (let i = 0; i <= 360 / step; i++) thetas.push(-180 + i * step);
      const L0 = LOG(0), L1 = LOG(f.radius), rows = 241;
      for (let j = 0; j < rows; j++) Ls.push(L0 + (L1 - L0) * j / (rows - 1));
      const columns = thetas.map(t => columnValues(f.lines[li], t, Ls));
      grid = {thetas, Ls, step, columns, z: Ls.map((_, j) => columns.map(col => col[j]))};
    }
    f.polar.set(id, grid);
    return grid;
  }
  const kmOf = r => r / 1000;
  const unlogKm = L => L === null ? null : Math.max(0, Math.exp(L) - 100) / 1000;
  // Marching squares: polylines (x, y arrays with null breaks) where z crosses 0.
  function outline(xs, ys, z) {
    const out = {x: [], y: []}, segments = [];
    const lerp = (a, b, va, vb) => a + (b - a) * va / (va - vb);
    for (let j = 0; j + 1 < ys.length; j++) for (let i = 0; i + 1 < xs.length; i++) {
      const v = [z[j][i], z[j][i + 1], z[j + 1][i + 1], z[j + 1][i]];
      const p = [[xs[i], ys[j]], [xs[i + 1], ys[j]], [xs[i + 1], ys[j + 1]], [xs[i], ys[j + 1]]];
      const cross = [];
      for (let e = 0; e < 4; e++) {
        const a = e, b = (e + 1) % 4;
        if ((v[a] > 0) !== (v[b] > 0)) cross.push([lerp(p[a][0], p[b][0], v[a], v[b]), lerp(p[a][1], p[b][1], v[a], v[b])]);
      }
      if (cross.length === 2) segments.push(cross);
      else if (cross.length === 4) { segments.push([cross[0], cross[1]]); segments.push([cross[2], cross[3]]); }
    }
    // Join segments that share end points into polylines.
    const key = p => `${p[0].toFixed(4)},${p[1].toFixed(5)}`, ends = new Map(), used = new Uint8Array(segments.length);
    segments.forEach((s, i) => { for (const p of s) { const k = key(p); if (!ends.has(k)) ends.set(k, []); ends.get(k).push(i); } });
    for (let i = 0; i < segments.length; i++) {
      if (used[i]) continue;
      used[i] = 1; const line = [segments[i][0], segments[i][1]];
      for (const forward of [true, false]) {
        for (;;) {
          const tip = forward ? line.at(-1) : line[0];
          const next = (ends.get(key(tip)) || []).find(j => !used[j]);
          if (next === undefined) break;
          used[next] = 1; const s = segments[next], other = key(s[0]) === key(tip) ? s[1] : s[0];
          if (forward) line.push(other); else line.unshift(other);
        }
      }
      for (const p of line) { out.x.push(p[0]); out.y.push(p[1]); }
      out.x.push(null); out.y.push(null);
    }
    return out;
  }
  // The layer's outline as polar polylines: theta in degrees, r in km.
  function polarOutline(li, nez = false) {
    const g = polarField(li, nez), o = outline(g.thetas, g.Ls, g.z), r = o.y.map(unlogKm);
    const text = o.x.map((t, i) => t === null ? null : `${nez ? 'No-escape zone · ' : ''}${angleText(t)} · ${fmt(r[i], 2)} km`);
    return {theta: o.x, r, text};
  }

  // ---- Rendering --------------------------------------------------------------------
  const font = 'WTSymbols, D-DIN, Bahnschrift, DIN Alternate, Segoe UI, sans-serif';
  const theme = {paper_bgcolor: '#000000', plot_bgcolor: '#000000', font: {color: '#e2e2ea', family: font},
    hoverlabel: {bgcolor: '#0c0c0f', bordercolor: 'rgba(240,240,250,.35)', font: {color: '#f4f4fc', size: 12, family: font}},
    modebar: {bgcolor: 'rgba(0,0,0,0)', color: '#8a8a96', activecolor: '#f0f0fa'}};
  const grid = 'rgba(240,240,250,.09)', tick = '#b9b9c4', cyan = '#38c9d7';
  // Polar axes: bearing from the target's nose clockwise from the top, range in km.
  const multi = () => runs.length > 1;
  const shownRuns = () => runs.filter(run => run.rays.size);
  const revision = view => view + ':' + centreOn() + JSON.stringify(runs.map(run => run.submitted));
  // What the plot is drawn around. A ground launch searched around the target is turned half a circle to put the
  // launcher at the centre, the target flying up the plot; its bearings still name the target's aspect.
  const sight = () => {
    const c = runs[0]?.submitted, turned = c?.frame === 'target' && centreOn() === 'launcher';
    return {launcher: c?.frame === 'launcher', centre: c?.frame === 'launcher' || turned ? 'launcher' : 'target', turn: turned ? 180 : 0};
  };
  // Top views on km axes: bearing 0 up, positive bearings to the right, after the view's turn.
  const plotXY = (r, t) => r == null || t == null ? [null, null] : [r * Math.sin(radians(t + sight().turn)), r * Math.cos(radians(t + sight().turn))];
  // Label of a bearing a (deg, 0..360 clockwise) in the run's own frame.
  const bearingTick = a => {
    const b = a > 180 ? 360 - a : a;
    return sight().launcher ? (b === 0 ? '0° ahead' : b === 180 ? '180° behind' : `${b}°`) : b === 0 ? '0° head-on' : b === 180 ? '180° tail' : `${b}°`;
  };
  function polarLayout(radius, revision, annotations = []) {
    const ticks = Array.from({length: 12}, (_, i) => i * 30);
    const label = bearingTick;
    return {...theme, margin: {l: 40, r: 40, t: 40, b: 40}, hovermode: 'closest', uirevision: revision, annotations: [...motionNotes(), ...annotations],
      legend: {orientation: 'h', x: 0, y: 1.02, yanchor: 'bottom', font: {color: tick, size: 12}, bgcolor: 'rgba(0,0,0,0)', groupclick: 'togglegroup',
        ...(multi() ? {} : {title: {text: runs[0]?.submitted.launcher.platform === 'ground' ? 'Target altitude' : 'Altitude', font: {color: tick, size: 12}}})},
      polar: {bgcolor: '#000000',
        angularaxis: {rotation: 90, direction: 'clockwise', tickmode: 'array', tickvals: ticks, ticktext: ticks.map(label),
          gridcolor: grid, linecolor: 'rgba(240,240,250,.2)', color: tick, tickfont: {size: 11}},
        radialaxis: {range: [0, kmOf(radius)], ticksuffix: ' km', tickformat: ',.0f', angle: 300, tickangle: 0, gridcolor: grid,
          linecolor: 'rgba(240,240,250,.2)', color: tick, tickfont: {size: 10}}}};
  }
  async function render() {
    if (!runs.length) return;
    renderTable();
    if (!shownRuns().length) {
      if (el('envelope-chart').data) Plotly.purge(el('envelope-chart'));
      el('envelope-chart').innerHTML = '<div class="empty"><span class="empty-glyph">◎</span><h2>Searching…</h2></div>';
      return;
    }
    await renderLayer();
    if (inspected) renderInspector();
  }
  // The target at the centre, or the launcher.
  function polarTarget(radius) {
    // No heading arrow (Leo, 2026-10-26): the 0° head-on label already gives the target's direction.
    if (sight().centre === 'launcher') {
      const air = sight().launcher;
      return [{type: 'scatterpolar', mode: 'markers', r: [0], theta: [0], marker: {symbol: air ? 'triangle-up' : 'square', size: 11, color: '#f4f4fc'},
        hovertemplate: (air ? 'Launcher, flying up the plot' : 'Launcher on the ground') + '<extra></extra>', showlegend: false}];
    }
    return [{type: 'scatterpolar', mode: 'markers', r: [0], theta: [0], marker: {symbol: 'diamond', size: 11, color: '#ffd0ac'},
        hovertemplate: 'Target<extra></extra>', showlegend: false}];
  }
  // The launching aircraft's gimbal limit as two dashed bearings (Leo, 2026-10-26: no shaded regions).
  function gimbalTrace(radius) {
    const g = runs[0]?.submitted.launcher.gimbal_deg;
    if (!sight().launcher || !g || g >= 180) return [];
    const R = kmOf(radius) * 1.3;
    return [{type: 'scatterpolar', mode: 'lines', r: [0, R, null, 0, R], theta: [g, g, null, 360 - g, 360 - g], line: {color: 'rgba(240,240,250,.35)', width: 1, dash: '4px,4px'},
      hovertemplate: `Launcher's gimbal limit (±${fmt(g, g % 1 ? 1 : 0)}°)<extra></extra>`, showlegend: false}];
  }
  // Around the launcher, a note in the top-right corner says how the target flies, with an arrow when its heading is fixed.
  function motionNotes() {
    const s = sight(), c = runs[0]?.submitted;
    if (s.centre !== 'launcher' || !c) return [];
    const rule = c.target.heading, speed = `${fmt(c.target.speed_mps * 3.6)} km/h`;
    let text, heading = null;
    if (!s.launcher) { text = `Target flying up the plot · ${speed}`; heading = 0; }
    else if (typeof rule === 'number') { text = `Target heading ${fmt(Math.abs(rule), rule % 1 ? 1 : 0)}°${rule % 180 ? (rule > 0 ? ' right' : ' left') + ' of yours' : ''} · ${speed}`; heading = rule; }
    else { text = `${motions[rule].text} · ${speed}`; heading = {same: 0, opposite: 180}[rule] ?? null; }
    const notes = [{xref: 'paper', yref: 'paper', x: 1, y: 1, xanchor: 'right', yanchor: 'top', showarrow: false, text, font: {size: 12, color: '#ffd0ac'}}];
    if (heading !== null) {
      const L = 34, dx = Math.sin(radians(heading)), dy = Math.cos(radians(heading));
      notes.push({xref: 'paper', yref: 'paper', x: 1, y: 1, xshift: -24 + L / 2 * dx, yshift: -44 + L / 2 * dy, ax: -L * dx, ay: L * dy, text: '',
        showarrow: true, arrowhead: 2, arrowsize: 1.2, arrowwidth: 1.6, arrowcolor: '#ffd0ac'});
    }
    return notes;
  }
  // Searched samples: place(ray, az, range_m) returns [a, b] (polar r km and theta, or x and y) or null.
  // customdata [layer, azimuth, run, 0, range m]: a click flies exactly that shot.
  function sampleTraces(filterRay, place, polar, run) {
    const hits = {a: [], b: [], t: [], c: []}, misses = {a: [], b: [], t: [], c: []};
    for (const ray of rays.values()) {
      if (!filterRay(ray)) continue;
      for (const az of mirrored() && ray.azimuth_deg > 0 && ray.azimuth_deg < 180 ? [ray.azimuth_deg, -ray.azimuth_deg] : [ray.azimuth_deg])
        for (const s of ray.samples) {
          const p = place(ray, az, s.range_m); if (!p) continue;
          const d = s.hit ? hits : misses;
          d.a.push(p[0]); d.b.push(p[1]); d.c.push([ray.key[0], az, run.index, 0, s.range_m]);
          d.t.push(`${s.hit ? 'Hit' : 'Miss · ' + escapeHtml(outcomeLabel(s.outcome))} at ${km(s.range_m)}<br>${fmt(ray.layer_m)} m · ${escapeHtml(bearingName(az))}${Number.isFinite(s.time_s) ? `<br>Ended ${fmt(s.time_s, 1)} s after release` : ''}`);
        }
    }
    return [[hits, '#91d477', 'Hit sample', 'circle'], [misses, '#f08070', 'Miss sample', 'x']].map(([d, color, name, symbol]) => ({
      type: polar ? 'scatterpolar' : 'scatter', mode: 'markers', name, ...(polar ? {r: d.a, theta: d.b} : {x: d.a, y: d.b}), text: d.t, customdata: d.c,
      hovertemplate: '%{text}<extra></extra>', marker: {size: 5, color, symbol, opacity: .9}, showlegend: false}));
  }
  // Searched hit intervals as radial segments along their bearings.
  function rayTrace(line) {
    const r = [], theta = [];
    for (const {az, ray} of line.entries) for (const i of ray.intervals) { r.push(kmOf(i.start_m), kmOf(i.end_m), null); theta.push(az, az, null); }
    return {type: 'scatterpolar', mode: 'lines', r, theta, hoverinfo: 'skip', line: {color: 'rgba(240,240,250,.35)', width: 1}, showlegend: false};
  }
  // The no-escape zone of one layer: a thin dashed outline in the layer's colour (no fill or dots: Leo, 2026-10-26).
  function nezTrace(li, run, color, group) {
    const o = polarOutline(li, true);
    return {trace: {type: 'scatterpolar', mode: 'lines', name: 'No-escape zone', legendgroup: group, r: o.r, theta: o.theta, text: o.text.map(t => t && runName(run) + t),
      connectgaps: false, line: {color, width: 1.4, dash: '5px,3px'}, hovertemplate: '%{text}<extra></extra>', showlegend: false}, outline: o};
  }
  const runName = run => multi() ? escapeHtml(missileName(run.missile)) + ' · ' : '';
  const outlineXY = o => o.r.map((r, i) => plotXY(r, o.theta[i]));
  async function renderLayer() {
    const li = 0, traces = []; let radius = 0; labelLines = []; pickLines = [];
    for (const run of shownRuns()) within(run, () => {
      const f = field(), line = f.lines[li]; radius = Math.max(radius, f.radius);
      if (!line?.az.length) return;
      const o = polarOutline(li), color = multi() ? run.color : cyan, name = missileName(run.missile), oxy = outlineXY(o);
      pickLines.push({group: run.missile, run: run.index, layer: li, x: oxy.map(q => q[0]), y: oxy.map(q => q[1])});
      traces.push({type: 'scatterpolar', mode: 'lines', name, legendgroup: run.missile, r: o.r, theta: o.theta, text: o.text.map(t => t && runName(run) + t), connectgaps: false,
        line: {color, width: 2.2}, hovertemplate: '%{text}<extra></extra>', showlegend: multi()});
      if (hasNez(li)) {
        const z = nezTrace(li, run, color, run.missile), zxy = outlineXY(z.outline);
        traces.push(z.trace);
        labelLines.push({group: run.missile, text: `${runName(run)}NEZ`, color, order: run.index, x: zxy.map(q => q[0]), y: zxy.map(q => q[1])});
      }
      if (el('envelope-show-rays').checked) traces.push(rayTrace(line));
      if (el('envelope-show-samples').checked) traces.push(...sampleTraces(ray => ray.key[0] === li, (ray, az, r) => [kmOf(r), az], true, run));
    });
    traces.unshift(...gimbalTrace(radius));
    traces.push(...polarTarget(radius));
    const layout = polarLayout(radius, revision('layer'));
    layout.showlegend = multi();
    await draw(traces, layout);
  }
  // Plotly's polar plots cannot pan or wheel-zoom, so the top views are drawn on equal-scaled km axes with polar
  // range rings and bearing spokes: same picture, the EM chart's mouse interaction. Bearing 0 (head-on) is up,
  // positive bearings (+Z side) to the right.
  function cartesianFromPolar(traces, layout) {
    const xy = plotXY;
    const out = [];
    for (const trace of traces) {
      if (trace.type === 'scatterpolar') {
        const {r, theta, ...rest} = trace, x = [], y = [];
        r.forEach((v, i) => { const [a, b] = xy(v, theta[i]); x.push(a); y.push(b); });
        out.push({...rest, type: 'scatter', x, y});
      } else out.push(trace);
    }
    const R = layout.polar.radialaxis.range[1], nice = [1, 2, 2.5, 5, 10, 20, 25, 50, 100, 200, 500];
    const step = nice.find(v => R / v <= 6) || R / 5, shapes = [], annotations = [...(layout.annotations || [])];
    const line = {color: 'rgba(240,240,250,.09)', width: 1};
    for (let k = step; k <= R + 1e-9; k += step) shapes.push({type: 'circle', xref: 'x', yref: 'y', x0: -k, x1: k, y0: -k, y1: k, line, layer: 'below'});
    for (let a = 0; a < 360; a += 30) {
      const [u, v] = xy(R, a);
      shapes.push({type: 'line', xref: 'x', yref: 'y', x0: 0, y0: 0, x1: u, y1: v, line, layer: 'below'});
      const [lx, ly] = xy(R * 1.07, a);
      annotations.push({x: lx, y: ly, xref: 'x', yref: 'y', showarrow: false, text: bearingTick(a),
        font: {size: 11, color: '#b9b9c4'}});
    }
    const {polar, ...rest} = layout, span = R * 1.16;
    const axis = {gridcolor: 'rgba(240,240,250,.09)', zeroline: false, color: '#b9b9c4', tickfont: {size: 12}, ticksuffix: ' km', range: [-span, span]};
    return {traces: out, layout: {...rest, shapes, annotations, margin: {l: 58, r: 22, t: rest.showlegend === false ? 30 : 50, b: 40},
      // Equal km scale kept by widening the x range, so the plot fills the width and pans freely.
      xaxis: {...axis, constrain: 'range'}, yaxis: {...axis, scaleanchor: 'x', scaleratio: 1, constrain: 'range'}}};
  }
  // Labels on the drawn outlines (the no-escape zone's), placed like the EM chart's SEP labels: slanted along their line,
  // never overlapping, clear of other outlines where the line leaves room, laid out again after zoom, pan or a legend toggle.
  // pickLines: the drawn outlines a click on the top view picks its missile from.
  let labelLines = [], labeling = false, pickLines = [];
  function placeLabels() {
    const node = el('envelope-chart'), L = node._fullLayout;
    if (!L?.xaxis || !node.data) return;
    const xa = L.xaxis, ya = L.yaxis, width = xa._length, height = ya._length, cell = 6;
    const shown = labelLines.filter(line => node.data.some(t => t.legendgroup === line.group && t.mode === 'lines' && t.visible !== 'legendonly'))
      .sort((a, b) => b.order - a.order);
    const px = (x, y) => [xa.l2p(x), ya.l2p(y)], lines = new Map();
    shown.forEach((line, key) => {
      for (let i = 1; i < line.x.length; i++) {
        if ([line.x[i - 1], line.y[i - 1], line.x[i], line.y[i]].some(v => v == null)) continue;
        const [ax, ay] = px(line.x[i - 1], line.y[i - 1]), [bx, by] = px(line.x[i], line.y[i]), steps = Math.ceil(Math.hypot(bx - ax, by - ay) / (cell / 2)) + 1;
        for (let k = 0; k <= steps; k++) {
          const c = Math.floor((ax + (bx - ax) * k / steps) / cell) + ',' + Math.floor((ay + (by - ay) * k / steps) / cell);
          if (!lines.has(c)) lines.set(c, new Set()); lines.get(c).add(key);
        }
      }
    });
    const placed = [], labels = [], inside = (x, y) => x >= 4 && y >= 4 && x <= width - 4 && y <= height - 4;
    const crossings = (box, own) => { let n = 0;
      for (let cx = Math.floor(box[0] / cell); cx <= Math.floor(box[2] / cell); cx++) for (let cy = Math.floor(box[1] / cell); cy <= Math.floor(box[3] / cell); cy++) {
        const keys = lines.get(cx + ',' + cy); if (keys) for (const key of keys) if (key !== own) { n++; break; }
      } return n; };
    // One label on each outline's farthest loop (the maximum range). The smaller loops near the target (minimum
    // range) get their own label only once zoomed in far enough that the loop spans MIN_LOOP_PX on screen.
    const MIN_LOOP_PX = 160, items = [];
    shown.forEach((line, key) => {
      const loops = []; let loop = [];
      line.x.forEach((x, i) => { if (x == null || line.y[i] == null) { if (loop.length > 1) loops.push(loop); loop = []; } else loop.push([x, line.y[i]]); });
      if (loop.length > 1) loops.push(loop);
      if (!loops.length) return;
      const reach = l => Math.max(...l.map(([x, y]) => Math.hypot(x, y))), main = loops.reduce((m, l) => reach(l) > reach(m) ? l : m);
      items.push({line, key, loop: main});
      for (const l of loops) {
        if (l === main) continue;
        const q = l.map(([x, y]) => px(x, y)), span = Math.max(Math.max(...q.map(v => v[0])) - Math.min(...q.map(v => v[0])), Math.max(...q.map(v => v[1])) - Math.min(...q.map(v => v[1])));
        if (span >= MIN_LOOP_PX) items.push({line, key, loop: l, minor: true});
      }
    });
    items.sort((a, b) => !!a.minor - !!b.minor);
    for (const {line, key, loop} of items) {
      const w = line.text.length * 6.6 + 6, h = 15;
      const pts = loop.map(([x, y]) => px(x, y));
      const along = [0]; for (let i = 1; i < pts.length; i++) along.push(along[i - 1] + (pts[i] && pts[i - 1] ? Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]) : 0));
      const total = along.at(-1); if (total < w * 1.5) continue;
      let best = null;
      for (let f = .04; f <= .96; f += .02) {
        const target = total * f, i = along.findIndex(v => v >= target); if (i < 1 || !pts[i] || !pts[i - 1]) continue;
        const t = (target - along[i - 1]) / ((along[i] - along[i - 1]) || 1), cx = pts[i - 1][0] + t * (pts[i][0] - pts[i - 1][0]), cy = pts[i - 1][1] + t * (pts[i][1] - pts[i - 1][1]);
        const ia = along.findIndex(v => v >= target + w / 2), ahead = pts[ia >= 0 ? ia : pts.length - 1], behind = pts[Math.max(0, along.findLastIndex(v => v <= target - w / 2))];
        let angle = ahead && behind ? Math.atan2(ahead[1] - behind[1], ahead[0] - behind[0]) * 180 / Math.PI : 0; if (angle > 90) angle -= 180; if (angle < -90) angle += 180;
        const c = Math.abs(Math.cos(radians(angle))), sn = Math.abs(Math.sin(radians(angle))), bw = w * c + h * sn, bh = w * sn + h * c;
        const box = [cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2];
        if (!inside(box[0], box[1]) || !inside(box[2], box[3])) continue;
        if (placed.some(q => box[0] < q[2] + 4 && q[0] < box[2] + 4 && box[1] < q[3] + 3 && q[1] < box[3] + 3)) continue;
        const near = placed.reduce((m, q) => Math.min(m, Math.hypot((q[0] + q[2]) / 2 - cx, (q[1] + q[3]) / 2 - cy)), Infinity);
        // Fewest crossed lines first, then away from other labels, then near this line's usual spot.
        const score = crossings(box, key) * 10 + Math.max(0, 60 - near) / 6 + Math.abs(f - (key % 2 ? .62 : .38)) * 8;
        if (!best || score < best.score) best = {score, cx, cy, box, angle};
      }
      if (!best) continue;
      placed.push(best.box);
      labels.push({envLabel: true, x: xa.p2l(best.cx), y: ya.p2l(best.cy), xref: 'x', yref: 'y', text: line.text, textangle: best.angle, showarrow: false,
        xanchor: 'center', yanchor: 'middle', bgcolor: 'rgba(0,0,0,.88)', borderpad: 1, font: {color: line.color, size: 11, family: font}});
    }
    labeling = true;
    Plotly.relayout(node, {annotations: [...(node.layout.annotations || []).filter(a => !a.envLabel), ...labels]}).finally(() => { labeling = false; });
  }
  async function draw(traces, layout) {
    if (layout.polar) ({traces, layout} = cartesianFromPolar(traces, layout));
    const node = el('envelope-chart');
    if (node.querySelector('.empty')) node.replaceChildren();
    // As on the EM chart: drag pans, the wheel zooms, tooltips take their line's colour, downloads sit in Plotly's toolbar.
    for (const trace of traces) {
      const color = typeof trace.line?.color === 'string' ? trace.line.color : typeof trace.marker?.color === 'string' ? trace.marker.color : null;
      if (color) trace.hoverlabel = {...trace.hoverlabel, bordercolor: color};
    }
    const icons = typeof DATA_ICONS === 'object' ? DATA_ICONS : null;
    const downloads = icons ? [{name: 'Download directions (CSV)', icon: icons.csv, click: () => el('envelope-download-csv').click()},
      {name: 'Download envelope (JSON)', icon: icons.json, click: () => el('envelope-download-json').click()}] : [];
    const shot = shotShape(layout);
    if (shot) layout = {...layout, shapes: [...(layout.shapes || []), shot]};
    await Plotly.react(node, traces, {...layout, dragmode: 'pan'}, {responsive: true, displaylogo: false, scrollZoom: true, modeBarButtonsToRemove: ['select2d', 'lasso2d'],
      toImageButtonOptions: {filename: runs.map(run => run.missile).join('-vs-') + '-envelope', format: 'png', scale: 2}, modeBarButtonsToAdd: downloads});
    if (labelLines.length) placeLabels();
    node.removeAllListeners?.('plotly_relayout'); node.removeAllListeners?.('plotly_restyle');
    node.on('plotly_relayout', event => {
      if (!labeling && labelLines.length && Object.keys(event).some(key => key.startsWith('xaxis') || key.startsWith('yaxis') || key === 'autosize')) placeLabels();
    });
    node.on('plotly_restyle', () => { if (labelLines.length) placeLabels(); });
    // A click anywhere in the plot area sets up a shot there (Leo, 2026-10-26). Plotly re-sends a click that did not
    // drag to the plot's drag surface, which is the only target taken here (not the legend or the toolbar).
    node.onclick = event => { if (event.target.closest?.('.nsewdrag')) pickShot(event); };
  }
  // The shot under a click: on a searched sample (when samples are shown) exactly that shot; elsewhere the clicked
  // bearing and distance (0.1°, 10 m) at the calculated altitude.
  function pickShot(event) {
    const node = el('envelope-chart'), L = node._fullLayout, xa = L?.xaxis, ya = L?.yaxis;
    if (!xa || !ya || !shownRuns().length) return;
    const box = node.getBoundingClientRect();
    const cx = event.clientX - box.left - xa._offset, cy = event.clientY - box.top - ya._offset, x = xa.p2l(cx), y = ya.p2l(cy);
    // Only a click on the sample itself (tooltips reach 20 px).
    const point = (node._hoverdata || []).find(p => Array.isArray(p.customdata) && Number.isFinite(p.customdata[4]) && runs[p.customdata[2]]
      && Math.hypot(xa.l2p(p.x) - cx, ya.l2p(p.y) - cy) <= 6);
    if (point) {
      const [li, az, index, , range] = point.customdata, run = runs[index];
      inspected = {run: index, layer: li, altitude: run.submitted.layers_m[li], azimuth: az, range};
    } else {
      const az = Math.round(wrap(Math.atan2(x, y) * 180 / Math.PI - sight().turn) * 10) / 10, range = Math.max(50, Math.round(Math.hypot(x, y) * 100) * 10);
      // The missile of the drawn outline nearest the click on screen, at the altitude shown.
      const shown = pickLines.filter(line => node.data.some(t => t.legendgroup === line.group && t.mode === 'lines' && t.visible !== 'legendonly'));
      let best = null;
      for (const line of shown) for (let i = 1; i < line.x.length; i++) {
        if ([line.x[i - 1], line.y[i - 1], line.x[i], line.y[i]].some(v => v == null)) continue;
        const ax = xa.l2p(line.x[i - 1]), ay = ya.l2p(line.y[i - 1]), bx = xa.l2p(line.x[i]), by = ya.l2p(line.y[i]), dx = bx - ax, dy = by - ay;
        const t = Math.max(0, Math.min(1, ((cx - ax) * dx + (cy - ay) * dy) / (dx * dx + dy * dy || 1))), d = Math.hypot(ax + t * dx - cx, ay + t * dy - cy);
        if (!best || d < best.d) best = {d, line};
      }
      const run = best ? runs[best.line.run] : shownRuns()[0];
      const li = best ? best.line.layer : 0;
      inspected = {run: run.index, layer: li, altitude: run.submitted.layers_m[li], azimuth: az, range};
    }
    renderInspector(); markShot();
  }
  // s of the envelope (escape: of the no-escape zone) at a shot: its layer, or across layers off them; null outside the calculated altitudes.
  function shotValue(li, az, range, altitude, escape = false) {
    const f = field(), layers = submitted.layers_m;
    const at = k => { const v = layerValue(f.lines[k], range, az); return escape ? (hasNez(k) ? Math.min(v, layerValue(f.escape[k], range, az)) : EMPTY) : v; };
    if (Math.abs(altitude - layers[li]) < 1e-6) return f.lines[li]?.az.length ? at(li) : null;
    if (altitude < layers[0] || altitude > layers.at(-1) || layers.some((_, k) => !f.lines[k]?.az.length)) return null;
    return volumeValue(layers.map((_, k) => at(k)), altitude);
  }
  // The inspected shot as a ring on the plot, where this view shows it.
  function shotShape(layout) {
    const run = inspected && runs[inspected.run]; if (!run || !layout) return null;
    const {azimuth: az, range} = inspected, [x, y] = plotXY(range / 1000, az);
    return {envShot: true, type: 'circle', xref: 'x', yref: 'y', xsizemode: 'pixel', ysizemode: 'pixel', xanchor: x, yanchor: y,
      x0: -7, x1: 7, y0: -7, y1: 7, line: {color: '#ffffff', width: 2}};
  }
  function markShot() {
    const node = el('envelope-chart'); if (!node.layout) return;
    const shot = shotShape(node.layout);
    Plotly.relayout(node, {shapes: [...(node.layout.shapes || []).filter(s => !s.envShot), ...(shot ? [shot] : [])]});
  }
  // The direction searched on a bearing in one layer (mirrored directions count on both sides); escape: against the escaping target.
  function rayNear(li, azimuth, escape = false) {
    const entry = entryNear(li, azimuth, escape);
    return entry && Math.abs(wrap(entry.az - azimuth)) < 1e-6 ? entry.ray : null;
  }
  // The searched direction nearest a bearing in one layer, as {az, ray}.
  function entryNear(li, azimuth, escape = false) {
    const line = escape ? field().escape?.[li] : field().lines[li];
    if (!line?.az.length) return null;
    let best = null;
    for (const entry of line.entries) { const d = Math.abs(wrap(entry.az - azimuth)); if (!best || d < best.d) best = {d, entry}; }
    return best.entry;
  }
  function renderTable() {
    const shown = shownRuns();
    root.querySelector('.envelope-table-card').hidden = !shown.length;
    if (!shown.length) return;
    const step = within(runs[0], baseStep), every = Math.ceil(30 / step - 1e-9) * step;
    const columns = Array.from({length: Math.floor(180 / every + 1e-9) + 1}, (_, i) => i * every);
    // A target crossing at a fixed heading is not symmetric: list the left side too.
    const sided = !within(runs[0], mirrored);
    if (sided) columns.unshift(...columns.slice(1, -1).map(a => -a).reverse());
    const name = !within(runs[0], launcherFrame) ? a => a === 0 ? 'Head-on' : a === 180 ? 'Tail' : a === 90 ? 'Beam' : `${fmt(a)}°`
      : a => a === 0 ? 'Ahead' : a === 180 ? 'Behind' : `${fmt(Math.abs(a))}°${sided ? (a > 0 ? ' right' : ' left') : ''}`;
    const nez = shown.some(run => run.submitted.escape);
    el('envelope-table').tHead.innerHTML = `<tr><th>${multi() ? 'Missile · altitude' : 'Altitude'}</th>${columns.map(a => `<th>${name(a)}<small>min – max${nez ? ' · NEZ' : ''}</small></th>`).join('')}</tr>`;
    // The no-escape zone of each searched bearing on a second line: where both searches hit.
    const nezCell = (li, a) => {
      if (!submitted.escape) return '';
      const wez = rayNear(li, a), esc = rayNear(li, a, true);
      if (!wez || !esc) return '<small class="envelope-nez">NEZ …</small>';
      const iv = intersect(wez.intervals, esc.intervals);
      return `<small class="envelope-nez">${iv.length ? `NEZ ${km(iv[0].start_m)} – ${km(iv.at(-1).end_m)}${iv.length > 1 ? ' *' : ''}` : 'no NEZ'}</small>`;
    };
    el('envelope-table').tBodies[0].innerHTML = shown.map(run => within(run, () => submitted.layers_m.map((alt, li) => `<tr><td>${runName(run)}${fmt(alt)} m</td>${columns.map(a => {
      const ray = rayNear(li, a);
      if (!ray) return '<td class="hint">…</td>';
      if (ray.blocked) return '<td class="hint">no launch</td>';
      if (ray.max_range_m === null) return '<td class="hint">no hit</td>';
      return `<td>${ray.intervals[0].min_at_search_limit ? '≤' : ''}${km(ray.min_range_m)} – ${km(ray.max_range_m)}${ray.intervals.length > 1 ? ' *' : ''}${nezCell(li, a)}</td>`;
    }).join('')}</tr>`).slice().reverse().join(''))).join('');
  }
  // The launch zone in words: the missile's own (the game's launch-zone angles) and the launcher's gimbal limit.
  function launchZoneText(run) {
    const zone = run.jobInfo?.plan?.limits?.launch_angle_deg, g = run.submitted.launcher.gimbal_deg, deg = v => `±${fmt(v, v % 1 ? 1 : 0)}°`;
    const parts = [];
    if (zone) parts.push(`the ${missileName(run.missile)} launches within ${deg(zone[0])} of the nose`);
    if (g) parts.push(`the launcher's gimbal limit is ${deg(g)}`);
    return parts.join(' and ') || 'the target is beyond the launch angle';
  }
  function renderInspector() {
    const box = el('envelope-inspect'), button = el('envelope-open-simulator');
    const run = inspected && runs[inspected.run];
    el('envelope-inspector').hidden = !run;
    if (!run) { box.replaceChildren(); button.hidden = true; return; }
    use(run);
    const c = submitted, {layer: li, azimuth: az, range, altitude} = inspected, onLayer = Math.abs(altitude - c.layers_m[li]) < 1e-6;
    el('envelope-inspect-title').textContent = `${multi() ? missileName(run.missile) + ' · ' : ''}${fmt(altitude)} m · ${bearingName(az)} · ${fmt(range / 1000, 2)} km`;
    button.hidden = typeof window.missileSimulator?.load !== 'function';
    // This shot: flown by the envelope search, or what the envelope expects between its searched shots.
    const exact = onLayer ? rayNear(li, az) : null, sample = exact?.samples.find(s => Math.abs(s.range_m - range) < .5);
    const entry = entryNear(li, az), ray = entry?.ray;
    let shot;
    if (sample) shot = `The envelope search flew this shot: ${sample.hit ? 'a hit' : `a miss (${outcomeLabel(sample.outcome)})`}.`;
    else if (ray?.blocked) shot = `Outside the launch zone, so the envelope launches nothing here: ${launchZoneText(run)}. The simulator flies it anyway.`;
    else {
      const s = shotValue(li, az, range, altitude), z = s > 0 && c.escape ? shotValue(li, az, range, altitude, true) : null;
      shot = s === null ? (onLayer ? 'This altitude has not finished yet.' : 'Outside the calculated altitudes, so the envelope has no prediction here.')
        : `Between the searched shots, the envelope expects a ${s > 0 ? 'hit' : 'miss'} here${z > 0 ? ', inside the no-escape zone' : ''}.`;
    }
    // The simulator flies constant-velocity targets only.
    if (shot.includes('no-escape')) shot += ' The simulator flies the target straight, without the break.';
    let details = '';
    if (ray && !ray.blocked) {
      const where = exact ? 'This searched direction' : `Nearest searched direction, ${onLayer ? '' : fmt(c.layers_m[li]) + ' m · '}${bearingName(entry.az)}`;
      const found = ray.max_range_m === null ? 'no searched distance produced a hit.' :
        `hits from ${ray.intervals[0].min_at_search_limit ? 'the closest searched distance, ' : ''}${km(ray.min_range_m)} to ${km(ray.max_range_m)}${ray.intervals.at(-1).max_at_search_limit ? ' (the search limit)' : ''}. Farthest hit flew ${fmt(ray.max_time_s, 1)} s.${ray.intervals.length > 1 ? ` Hits stop and resume ${ray.intervals.length - 1} time(s) in between.` : ''}`;
      const rows = ray.samples.map(s => `<tr${sample === s ? ' class="envelope-shot"' : ''}><td>${km(s.range_m)}</td><td class="${s.hit ? 'envelope-hit' : 'envelope-miss'}">${s.hit ? 'Hit' : 'Miss'}</td><td>${escapeHtml(outcomeLabel(s.outcome))}</td><td>${Number.isFinite(s.time_s) ? fmt(s.time_s, 2) + ' s' : '—'}</td><td>${Number.isFinite(s.closest_m) ? closestText(s) : '—'}</td></tr>`).join('');
      const zone = c.escape && onLayer ? `<p class="muted">No-escape zone on this bearing, where the missile hits whether the target holds its course or breaks away: ${nezText(nezIntervals(li, az))}.</p>` : '';
      details = `<p class="muted">${escapeHtml(where)}: ${found} ${ray.simulations} simulations searched ${km(ray.search[0])} – ${km(ray.search[1])}.</p>${zone}
      <div class="envelope-table-wrap"><table class="envelope-samples"><thead><tr><th>Launch distance</th><th>Result</th><th>Ended by</th><th>At</th><th>Closest approach</th></tr></thead><tbody>${rows}</tbody></table></div>`;
    }
    run.cache = cache;
    box.innerHTML = `<p>${escapeHtml(shot)}</p>${details}`;
  }
  // The simulator flies the inspected shot as it is set up here, until its outcome.
  el('envelope-open-simulator').addEventListener('click', () => {
    const run = inspected && runs[inspected.run]; if (!run) return;
    const c = run.submitted, s = states(c, inspected.altitude, inspected.azimuth, inspected.range);
    window.missileSimulator.load({missile: c.missile, launcher: s.launcher, target: s.target});
  });
  for (const id of ['envelope-show-rays', 'envelope-show-samples'])
    el(id).addEventListener('change', () => render().catch(error => showError(error.message)));
  // Show the results kept for the chosen centre. Around the launcher, a ground launch calculated around the target is
  // reused (turned half a circle) until one is calculated here.
  function showCentre() {
    const centre = centreOn();
    runs = centre === 'launcher' && !runSets.launcher.length && runSets.target[0]?.submitted.launcher.platform === 'ground' ? runSets.target : runSets[centre];
    inspected = null; renderInspector();
    el('envelope-status').textContent = runs === runSets[centre] ? statusText[centre] : statusText.target;
    if (!runs.length) {
      if (el('envelope-chart').data) Plotly.purge(el('envelope-chart'));
      el('envelope-chart').innerHTML = `<div class="empty"><span class="empty-glyph">◎</span><h2>Add a missile and press Calculate</h2></div>`;
      root.querySelector('.envelope-table-card').hidden = true; setExports(false); return;
    }
    render().then(() => setExports(shownRuns().length > 0)).catch(error => showError(error.message));
  }

  // ---- Export -----------------------------------------------------------------------
  function setExports(enabled) {
    for (const url of downloadUrls) URL.revokeObjectURL(url); downloadUrls = [];
    for (const id of ['envelope-download-json', 'envelope-download-csv']) { el(id).setAttribute('aria-disabled', String(!enabled)); el(id).removeAttribute('href'); }
    if (!enabled) return;
    const header = ['missile', 'target_motion', 'altitude_m', 'azimuth_deg', 'refinement_level', 'min_range_m', 'max_range_m', 'min_at_search_limit', 'max_at_search_limit', 'max_range_flight_time_s', 'hit_intervals_m', 'simulations'];
    const csvRows = [header.join(',')], envelopes = [];
    const sorted = source => [...source.values()].sort((a, b) => a.key[0] - b.key[0] || a.azimuth_deg - b.azimuth_deg);
    for (const run of shownRuns()) within(run, () => {
      const ordered = sorted(rays), escaping = sorted(escapeRays);
      envelopes.push({format: 'missile-envelope-v2', missile: run.missile, missile_name: missileName(run.missile), request: submitted,
        config: jobInfo?.config || null, mirrored: mirrored(), complete: jobInfo?.status === 'complete', summary, rays: ordered,
        ...(submitted.escape ? {escape_rays: escaping} : {}),
        conventions: {...(submitted.frame === 'launcher' ? {frame: 'Launcher at the origin flying level along +X and keeping that heading; each point is a target position at release',
          azimuth: 'Target bearing off the launcher nose; 0 = dead ahead, 180 = behind, positive toward +Z (the launcher\'s right)',
          target_heading: "'hot' toward the launcher, 'cold' away from it, 'same' along +X, 'opposite' along -X, or degrees from +X toward +Z",
          launch_zone: 'Bearings beyond the missile launch zone (config limits.launch_angle_deg) or the launcher gimbal limit are not launched at (rays marked blocked)',
          range: 'Horizontal distance in metres from the launcher at release', layer: 'Altitude of both aircraft (m)'}
          : {frame: 'Target at the origin flying level along +X; each point is a launcher position at release, heading toward the target',
          azimuth: 'Launcher bearing from the target nose; 0 = ahead of the target (head-on), 180 = behind it (tail chase), positive toward +Z',
          range: 'Horizontal distance in metres from the target at release', layer: 'Altitude of both aircraft (m); the launcher flies level, pointed at the target'}),
          hit: 'Native proximity-fuze event against the constant-velocity point target',
          ...(submitted.escape ? {escape_rays: 'Searched against a target that, after the reaction delay, turns level at the escape load factor and constant speed until the launch point lies dead astern, then flies straight on',
            no_escape_zone: 'Launch distances where both rays and escape_rays hit'} : {})}});
      for (const r of [...ordered, ...escaping]) csvRows.push([run.missile, r.motion || 'straight', r.layer_m, r.azimuth_deg, r.level, r.min_range_m ?? '', r.max_range_m ?? '', r.max_range_m === null ? '' : r.intervals[0].min_at_search_limit,
        r.max_range_m === null ? '' : r.intervals.at(-1).max_at_search_limit, r.max_time_s ?? '', r.intervals.map(i => `${Math.round(i.start_m)}-${Math.round(i.end_m)}`).join(' '), r.simulations].join(','));
    });
    const data = envelopes.length === 1 ? envelopes[0] : {format: 'missile-envelopes-v1', envelopes}, csv = csvRows.join('\n');
    const stem = runs.map(run => run.missile).join('-vs-');
    for (const [id, content, type, name] of [['envelope-download-json', JSON.stringify(data, null, 1), 'application/json', `${stem}-envelope.json`], ['envelope-download-csv', csv, 'text/csv', `${stem}-envelope.csv`]]) {
      const url = URL.createObjectURL(new Blob([content], {type})); downloadUrls.push(url);
      el(id).href = url; el(id).download = name;
    }
  }

  // ---- Tab integration --------------------------------------------------------------
  document.addEventListener('tool-tab-change', event => {
    if (event.detail !== 'envelope') return;
    if (!meta) load();
    requestAnimationFrame(() => { if (el('envelope-chart').data) Plotly.Plots.resize(el('envelope-chart')); });
  });
  new ResizeObserver(() => { if (!root.hidden && el('envelope-chart').data) Plotly.Plots.resize(el('envelope-chart')); }).observe(el('envelope-chart'));
  reset(); renderInspector();
})();
