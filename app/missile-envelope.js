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
  const escapeHtml = text => String(text).replace(/[&<>"']/g, c => ({'&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;'}[c]));
  const radians = a => a * Math.PI / 180;
  const qualities = {
    quick: {step: 30, tolerance: 0.04, label: 'Quick · 30° directions · ±2% range'},
    standard: {step: 15, tolerance: 0.02, label: 'Standard · 15° directions · ±1% range'},
    fine: {step: 7.5, tolerance: 0.01, label: 'Detailed · 7.5° directions · ±0.5% range'}};
  const defaults = {frame: 'target', missile: null, quality: 'standard',
    target: {altitude: 6000, speed: 900, heading: 'hot', customHeading: 90},
    launcher: {altitude: 6000, speed: 900, aim: 'nose', pitch: 0},
    layers: {from: 2000, to: 10000, count: 5}};
  const headingLabels = {hot: 'Toward the launcher (hot)', cold: 'Away from the launcher (cold)', same: 'Same heading as the launcher', opposite: 'Opposite to the launcher', custom: 'Fixed heading…'};
  const numberField = (id, label, unit, value, min, max, step = 'any') =>
    `<label class="field">${label}<div class="input-unit"><input id="${id}" type="number" min="${min}" max="${max}" step="${step}" value="${value}" required aria-label="${escapeHtml(label)}"><span>${unit}</span></div></label>`;

  root.innerHTML = `<aside class="missile-sidebar envelope-sidebar"><form id="envelope-form" novalidate>
    <div class="section-heading"><span class="eyebrow">ENVELOPE SETUP</span><button id="envelope-reset" class="text-button" type="button">Reset all</button></div>
    <h2 class="missile-setup-title">Where can it hit?</h2>
    <p class="hint">Searches launch distance in every direction and altitude layer with the same engagement simulation as the missile simulator.</p>
    <fieldset class="missile-step" id="envelope-picker"><legend><span class="missile-step-number">1</span>Missile</legend>
      <input id="envelope-search" type="search" placeholder="Search missiles…" autocomplete="off" aria-label="Search missiles" aria-controls="envelope-list">
      <div id="envelope-list" class="aircraft-list" aria-label="Available missiles"></div>
      <p id="envelope-search-empty" class="hint" hidden>No missiles match your search.</p>
      <p id="envelope-catalog-note" class="hint" role="status">Loading missiles…</p>
      <div id="envelope-selected" class="envelope-selected" aria-live="polite"><span class="hint">No missile selected.</span></div>
      <button id="envelope-retry" class="text-button" type="button" hidden>Retry loading missiles</button>
    </fieldset>
    <fieldset class="missile-step"><legend><span class="missile-step-number">2</span>Draw the envelope around</legend>
      <div class="mode-switch" role="radiogroup" aria-label="Envelope frame"><label><input type="radio" name="envelope-frame" value="target" checked><span>The target</span></label><label><input type="radio" name="envelope-frame" value="launcher"><span>The launcher</span></label></div>
      <p id="envelope-frame-note" class="hint"></p>
    </fieldset>
    <fieldset class="missile-step missile-target"><legend><span class="missile-step-number">3</span>Target aircraft</legend>
      <div data-frame="target">${numberField('envelope-target-altitude', 'Altitude', 'm', defaults.target.altitude, 50, 25000)}</div>
      ${numberField('envelope-target-speed', 'Speed', 'km/h', defaults.target.speed, 0, 7200)}
      <div data-frame="launcher"><label class="field">Heading<select id="envelope-target-heading">${Object.entries(headingLabels).map(([k, v]) => `<option value="${k}">${v}</option>`).join('')}</select></label>
        <div id="envelope-custom-heading-field" hidden>${numberField('envelope-target-custom-heading', 'Fixed heading', '°', defaults.target.customHeading, -360, 360)}</div></div>
      <p id="envelope-target-note" class="hint"></p>
    </fieldset>
    <fieldset class="missile-step missile-launch"><legend><span class="missile-step-number">4</span>Launcher aircraft</legend>
      <div data-frame="launcher">${numberField('envelope-launcher-altitude', 'Altitude', 'm', defaults.launcher.altitude, 50, 25000)}</div>
      ${numberField('envelope-launcher-speed', 'Speed', 'km/h', defaults.launcher.speed, 0, 7200)}
      <div data-frame="target"><label class="field">Attitude at launch<select id="envelope-launcher-aim"><option value="nose">Nose pointed at the target</option><option value="level">Level, heading toward the target</option></select></label></div>
      <div data-frame="launcher">${numberField('envelope-launcher-pitch', 'Launch angle', '°', defaults.launcher.pitch, -60, 60)}</div>
      <p id="envelope-launcher-note" class="hint"></p>
    </fieldset>
    <fieldset class="missile-step"><legend><span class="missile-step-number">5</span><span id="envelope-layers-title">Launcher altitudes</span></legend>
      <div class="envelope-layer-inputs">${numberField('envelope-layers-from', 'Lowest', 'm', defaults.layers.from, 50, 25000)}${numberField('envelope-layers-to', 'Highest', 'm', defaults.layers.to, 50, 25000)}${numberField('envelope-layers-count', 'Layers', '', defaults.layers.count, 1, 12, 1)}</div>
      <p id="envelope-layers-list" class="hint"></p>
    </fieldset>
    <fieldset class="missile-step"><legend><span class="missile-step-number">6</span>What counts as a hit</legend>
      <label class="field">Hit test<select id="envelope-criterion"><option value="kinematic" selected>Kinematic reach · energy-limited</option><option value="fuze">Proximity fuze event · as in the simulator</option></select></label>
      <div id="envelope-reach-field">${numberField('envelope-reach', 'Reach distance', 'm', '', 1, 2000).replace('required ', '')}</div>
      <p id="envelope-criterion-note" class="hint"></p>
    </fieldset>
    <fieldset class="missile-step"><legend><span class="missile-step-number">7</span>Resolution</legend>
      <label class="field">Search detail<select id="envelope-quality">${Object.entries(qualities).map(([k, q]) => `<option value="${k}"${k === defaults.quality ? ' selected' : ''}>${q.label}</option>`).join('')}</select></label>
      <p id="envelope-cost" class="hint"></p>
    </fieldset>
    <div class="missile-run-panel">
      <p id="envelope-ready" class="hint"></p>
      <button id="envelope-run" class="primary-button" type="submit" disabled>Calculate envelope <span>↗</span></button>
      <button id="envelope-cancel" class="secondary-button" type="button" hidden>Stop · keep finished directions</button>
      <div id="envelope-status" role="status" aria-live="polite"></div>
    </div>
  </form></aside>
  <div class="missile-main">
    <div class="page-heading"><div><span class="eyebrow">MISSILE KINEMATICS</span><h1>3D launch envelope</h1></div><span class="status-badge">EXPERIMENTAL</span></div>
    <p id="envelope-model-note" class="missile-model-note">Each direction is searched with complete engagement simulations: recovered propulsion, mass, aerodynamics, control limits and guidance. In the kinematic envelope a launch position is inside when the missile passes within the reach distance of the non-maneuvering target before its lifetime, distance limit or the ground ends the flight; the proximity fuze is switched off.</p>
    <div id="envelope-error" class="error" role="alert" hidden></div>
    <div id="envelope-stale" class="notice" hidden>Settings changed. Calculate again to update the envelope.</div>
    <div id="envelope-progress" class="progress-panel" aria-live="polite" hidden><div><span class="spinner"></span><strong id="envelope-progress-label">Starting…</strong><span id="envelope-progress-count"></span></div><div class="progress-track"><div id="envelope-progress-fill"></div></div></div>
    <section class="chart-card">
      <div class="chart-toolbar"><div id="envelope-legend" class="missile-legend envelope-legend"><span class="envelope-key-max">■ Maximum range</span><span class="envelope-key-min">■ Minimum range</span><span class="envelope-key-ref">◆ <span id="envelope-ref-name">Target</span></span></div>
        <div class="missile-view-controls">
          <label for="envelope-view">Plot view<select id="envelope-view"><option value="3d">3D envelope</option><option value="layer">Layer · top view</option><option value="profile">Side profile</option></select></label>
          <label id="envelope-layer-label" for="envelope-layer" hidden>Layer<select id="envelope-layer"></select></label>
          <label id="envelope-cut-label" for="envelope-cut" hidden>Cut through<select id="envelope-cut"></select></label>
          <label class="envelope-check"><input id="envelope-show-min" type="checkbox" checked> Minimum range</label>
          <label class="envelope-check"><input id="envelope-show-samples" type="checkbox"> Samples</label>
          <button id="envelope-view-reset" type="button">Reset view</button>
        </div></div>
      <p id="envelope-view-note" class="hint missile-hover-hint"></p>
      <div id="envelope-chart" aria-label="Missile launch envelope"><div class="empty"><span class="empty-glyph">◎</span><h2>No envelope yet</h2><p>Choose a missile and the flight conditions,<br>then select <strong>Calculate envelope</strong>.</p></div></div>
    </section>
    <div class="missile-metrics envelope-metrics">
      <div class="missile-metric"><span id="envelope-metric-front-label">Head-on maximum range</span><strong id="envelope-metric-front">—</strong></div>
      <div class="missile-metric"><span id="envelope-metric-rear-label">Tail-chase maximum range</span><strong id="envelope-metric-rear">—</strong></div>
      <div class="missile-metric"><span>Simulations</span><strong id="envelope-metric-sims">—</strong></div>
    </div>
    <section class="inspector envelope-inspector" aria-live="polite"><div class="inspector-heading"><div><span class="eyebrow">DIRECTION</span><h2 id="envelope-inspect-title">Select a point on the envelope</h2></div><button id="envelope-open-simulator" class="secondary-button" type="button" hidden>Fly this shot in the simulator ↗</button></div><div id="envelope-inspect"></div></section>
    <section class="envelope-table-card"><h2>Ranges by layer</h2><div class="envelope-table-wrap"><table id="envelope-table"><thead></thead><tbody><tr><td class="hint">Calculate an envelope to fill this table.</td></tr></tbody></table></div></section>
    <div class="exports"><span>EXPORT</span><a id="envelope-download-json" aria-disabled="true">Envelope JSON</a><a id="envelope-download-csv" aria-disabled="true">Boundaries CSV</a></div>
    <details class="missile-details"><summary>Method and limitations</summary>
      <p>Launch positions are sampled in horizontal directions around the reference aircraft, one ring per altitude layer. Along each direction the launch distance is bisected between hits and misses until the bracket is within the selected tolerance. Directions start from boundaries predicted by finished neighbours, then confirm them with simulations. Because hits and narrow misses can alternate near a boundary, each search also launches a little beyond its outermost misses and continues if it hits again; a gap between hits is reported only where a sample found it. Search distances are limited by the missile's distance limit plus the target's travel during the missile's lifetime.</p>
      <p>Every simulation uses the missile simulator's model: recovered release processing, motors and changing mass, aerodynamics, control limits, guidance and seeker geometry, with ideal target visibility and radar support. The kinematic envelope switches the proximity fuze off and counts a launch when the missile passes within the reach distance of the target (by default the missile's own fuze radius), so fuze arming does not shape it; the fuze test instead requires the native proximity event. The target flies straight and level at constant speed; an envelope against a maneuvering target is not computed. A miss is declared early only once every motor has burned out and even converting all altitude to speed could not bring the missile to the target before its lifetime or distance limit.</p>
      <p>Seeker look-angle and tracking limits still apply, as in the simulator. With the launcher pointed at the target they rarely matter, but in the launcher-centred frame they cut off directions far off the nose.</p>
      <p>When the setup is mirror-symmetric (left/right), one side is searched and mirrored. Hardpoint offsets, aircraft damage and countermeasures are not modeled.</p>
    </details>
  </div>`;

  let meta = null, loading = null, selected = null, job = null, rays = new Map(), submitted = null, summary = null;
  let jobInfo = null, polling = false, renderTimer = null, lastRender = 0, downloadUrls = [], inspected = null;

  const showError = text => { el('envelope-error').textContent = text; el('envelope-error').hidden = !text; };
  const frame = () => root.querySelector('input[name=envelope-frame]:checked').value;
  const value = id => Number(el(id).value);
  const km = m => m >= 10000 ? (m / 1000).toFixed(1) + ' km' : m >= 1000 ? (m / 1000).toFixed(2) + ' km' : Math.round(m) + ' m';
  const fmt = (v, d = 0) => Number.isFinite(v) ? v.toLocaleString(undefined, {maximumFractionDigits: d, minimumFractionDigits: d}) : '—';
  const missileName = id => meta?.missiles.find(m => m.id === id)?.name || id;
  const fuzeRadius = id => meta?.missiles.find(m => m.id === id)?.fuze_radius_m ?? null;
  const outcomes = {reached: 'reached the target', proximity_event: 'proximity fuze', lifetime: 'lifetime ended', distance_limit: 'distance limit',
    tracking_timeout: 'lost track (timeout)', ground: 'hit the ground', target_ground: 'target reached the ground', unreachable: 'out of energy to reach',
    time_limit: 'time limit', error: 'simulation error'};
  const outcomeLabel = code => outcomes[code] || String(code).replaceAll('_', ' ');
  const closestText = s => s.outcome === 'unreachable' ? `≤ ${km(s.closest_m)} so far` : km(s.closest_m);

  function layers() {
    const from = value('envelope-layers-from'), to = value('envelope-layers-to'), count = Math.round(value('envelope-layers-count'));
    if (!(count >= 1)) return [];
    if (count === 1) return [from];
    return [...new Set(Array.from({length: count}, (_, i) => Math.round(from + (to - from) * i / (count - 1))))].sort((a, b) => a - b);
  }
  function heading() {
    const rule = el('envelope-target-heading').value;
    return rule === 'custom' ? value('envelope-target-custom-heading') : rule;
  }
  const isMirrored = config => config.frame === 'target' || typeof config.target.heading === 'string' || ((config.target.heading % 180) + 180) % 180 === 0;
  function azimuthList(config) {
    const step = qualities[config.quality].step, n = Math.round(180 / step);
    return isMirrored(config) ? Array.from({length: n + 1}, (_, i) => i * step) : Array.from({length: 2 * n}, (_, i) => -180 + (i + 1) * step);
  }
  function inputs() {
    const f = frame(), q = el('envelope-quality').value;
    const config = {missile: selected, frame: f, quality: q, layers_m: layers(),
      launcher: {speed_mps: value('envelope-launcher-speed') / 3.6}, target: {speed_mps: value('envelope-target-speed') / 3.6}};
    config.criterion = el('envelope-criterion').value;
    if (config.criterion === 'kinematic' && el('envelope-reach').value !== '') config.reach_m = value('envelope-reach');
    if (f === 'target') { config.target.altitude_m = value('envelope-target-altitude'); config.launcher.aim = el('envelope-launcher-aim').value; }
    else { config.launcher.altitude_m = value('envelope-launcher-altitude'); config.launcher.pitch_deg = value('envelope-launcher-pitch'); config.target.heading = heading(); }
    return config;
  }
  function updateForm() {
    const f = frame();
    root.querySelectorAll('[data-frame]').forEach(node => { node.hidden = node.dataset.frame !== f; node.querySelectorAll('input,select').forEach(input => input.disabled = node.hidden); });
    el('envelope-custom-heading-field').hidden = f !== 'launcher' || el('envelope-target-heading').value !== 'custom';
    el('envelope-target-custom-heading').disabled = el('envelope-custom-heading-field').hidden;
    el('envelope-frame-note').textContent = f === 'target'
      ? 'The target sits at the centre. Each point of the envelope is a launcher position: bearing 0° is in front of the target (head-on), 180° behind it (tail chase).'
      : 'The launcher sits at the centre, flying along +X. Each point of the envelope is a target position: bearing 0° is straight ahead of the launcher.';
    el('envelope-target-note').textContent = f === 'target' ? 'Flies straight and level along +X at constant speed.' : 'Flies straight and level at constant speed; heading is set relative to where the target starts.';
    el('envelope-launcher-note').textContent = f === 'target' ? 'Launches toward the target from every sampled position. Its altitude is the layer altitude.' : 'Flies along +X at release. Target altitude is the layer altitude.';
    el('envelope-layers-title').textContent = f === 'target' ? 'Launcher altitudes' : 'Target altitudes';
    const kinematic = el('envelope-criterion').value === 'kinematic';
    el('envelope-reach-field').hidden = !kinematic; el('envelope-reach').disabled = !kinematic;
    const fuze = meta && selected ? fuzeRadius(selected) : null;
    el('envelope-reach').placeholder = fuze ? `${fmt(fuze, 1)} · fuze radius` : 'Fuze radius';
    el('envelope-criterion-note').textContent = kinematic
      ? "The fuze is switched off. A launch counts when the guided missile passes within the reach distance before it runs out of lifetime, distance or altitude, so the edge is set by motor, drag, lift and control limits. Leave the distance blank to use this missile's fuze radius."
      : "A launch counts only when the missile's proximity fuze fires, including its arming delay and distance. Near the edge, narrow misses can alternate with hits.";
    const list = layers();
    el('envelope-layers-list').textContent = list.length ? `Layers: ${list.map(v => fmt(v) + ' m').join(', ')}` : '';
    const config = inputs(), directions = list.length * azimuthList(config).length;
    el('envelope-cost').textContent = `${directions} directions${isMirrored(config) ? ' (left/right mirrored)' : ''} · roughly ${fmt(directions * 9)} simulations. Long-range radar missiles take several minutes.`;
    const fields = [...el('envelope-form').querySelectorAll('input[type=number]:not(:disabled)')];
    const invalid = fields.find(field => !field.checkValidity());
    let message = '';
    if (!meta) message = 'Loading missiles…';
    else if (!selected) message = 'Select a missile.';
    else if (invalid) message = `${invalid.getAttribute('aria-label')}: ${invalid.validationMessage}`;
    else if (value('envelope-layers-to') < value('envelope-layers-from')) message = 'Highest layer must be at or above the lowest.';
    else message = job ? `Calculating ${missileName(submitted?.missile || selected)} envelope…` : `${missileName(selected)} · ${directions} directions ready.`;
    el('envelope-ready').textContent = message;
    el('envelope-run').disabled = !!job || !meta || !selected || !!invalid || value('envelope-layers-to') < value('envelope-layers-from');
    if (submitted) el('envelope-stale').hidden = JSON.stringify(inputs()) === JSON.stringify(submitted);
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
      const label = document.createElement('span'), name = document.createElement('strong'), detail = document.createElement('small');
      name.textContent = `${item.name} [${item.id}]`; detail.textContent = family; label.append(name, detail);
      const button = document.createElement('button'); button.type = 'button'; button.dataset.pick = item.id;
      button.setAttribute('aria-label', 'Select ' + item.name);
      row.append(label, button); return row;
    }));
    el('envelope-catalog-note').textContent = `${meta.missiles.length} available missiles`;
    filter(); renderSelected();
  }
  function filter() {
    const query = normalize(el('envelope-search').value); let visible = 0;
    for (const row of el('envelope-list').children) { row.hidden = !normalize(row.dataset.search).includes(query); if (!row.hidden) visible++; }
    el('envelope-search-empty').hidden = !meta || visible > 0;
    for (const button of el('envelope-list').querySelectorAll('[data-pick]')) {
      button.textContent = button.dataset.pick === selected ? 'Selected' : 'Select';
      button.disabled = !!job || button.dataset.pick === selected;
    }
  }
  function renderSelected() {
    const box = el('envelope-selected');
    if (!selected) { box.innerHTML = '<span class="hint">No missile selected.</span>'; return; }
    const item = meta?.missiles.find(m => m.id === selected);
    box.innerHTML = `<div class="missile-selected-row"><span style="border-color:var(--cyan)">${escapeHtml(item ? `${item.name} [${item.id}]` : selected)}</span></div>`;
  }
  el('envelope-list').addEventListener('click', event => {
    const button = event.target.closest('[data-pick]');
    if (!button || button.disabled) return;
    selected = button.dataset.pick; renderSelected(); filter(); updateForm();
  });
  el('envelope-search').addEventListener('input', filter);
  el('envelope-search').addEventListener('keydown', event => { if (event.key === 'Enter') event.preventDefault(); });
  el('envelope-retry').addEventListener('click', load);
  el('envelope-form').addEventListener('input', event => { if (event.target.id !== 'envelope-search') updateForm(); });
  el('envelope-form').addEventListener('change', event => { if (event.target.id !== 'envelope-search') updateForm(); });
  function reset() {
    root.querySelector('input[name=envelope-frame][value=target]').checked = true;
    for (const [id, v] of [['envelope-target-altitude', defaults.target.altitude], ['envelope-target-speed', defaults.target.speed],
      ['envelope-target-custom-heading', defaults.target.customHeading], ['envelope-launcher-altitude', defaults.launcher.altitude],
      ['envelope-launcher-speed', defaults.launcher.speed], ['envelope-launcher-pitch', defaults.launcher.pitch],
      ['envelope-layers-from', defaults.layers.from], ['envelope-layers-to', defaults.layers.to], ['envelope-layers-count', defaults.layers.count]]) el(id).value = v;
    el('envelope-target-heading').value = defaults.target.heading; el('envelope-launcher-aim').value = defaults.launcher.aim;
    el('envelope-quality').value = defaults.quality; el('envelope-search').value = '';
    el('envelope-criterion').value = 'kinematic'; el('envelope-reach').value = '';
    if (meta) filter();
    updateForm();
  }
  el('envelope-reset').addEventListener('click', () => { if (!job) reset(); });

  // ---- Calculation ----------------------------------------------------------------
  function busy(value) {
    el('envelope-cancel').hidden = !value; el('envelope-reset').disabled = value;
    el('envelope-progress').hidden = !value;
    filter(); updateForm();
  }
  el('envelope-form').addEventListener('submit', async event => {
    event.preventDefault();
    if (job || el('envelope-run').disabled) return;
    showError(''); summary = null; rays = new Map(); inspected = null; jobInfo = null;
    submitted = inputs(); el('envelope-stale').hidden = true;
    renderInspector(); setExports(false); render().catch(error => showError(error.message));
    try {
      const created = await request('/api/missiles/envelopes', {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(submitted)});
      job = created.id; busy(true); poll();
    } catch (error) { showError(error.message); job = null; busy(false); }
  });
  el('envelope-cancel').addEventListener('click', async () => {
    if (!job) return;
    el('envelope-status').textContent = 'Stopping…';
    try { await request('/api/missiles/envelopes/' + job + '/cancel', {method: 'POST'}); } catch (error) { showError(error.message); }
  });
  async function poll() {
    if (polling) return; polling = true;
    const started = performance.now();
    try {
      while (job) {
        let status;
        try { status = await request(`/api/missiles/envelopes/${job}?since=${rays.size}`); }
        catch (error) { el('envelope-status').textContent = 'Connection interrupted · retrying…'; await new Promise(r => setTimeout(r, 1500)); continue; }
        jobInfo = status;
        for (const ray of status.rays) rays.set(ray.key.join(','), ray);
        const total = status.rays_total || 1, done = rays.size, p = status.progress;
        const elapsed = (performance.now() - started) / 1000;
        el('envelope-progress-label').textContent = status.status === 'queued' ? 'Waiting for the previous envelope…' : `Searching ${missileName(submitted.missile)} envelope…`;
        el('envelope-progress-count').textContent = `${done} / ${total} directions · ${fmt(p?.simulations || 0)} simulations · ${Math.floor(elapsed / 60)}:${String(Math.floor(elapsed % 60)).padStart(2, '0')}`;
        el('envelope-progress-fill').style.width = `${100 * done / total}%`;
        el('envelope-status').textContent = `${done} / ${total} directions finished.`;
        if (status.rays.length) scheduleRender();
        if (['complete', 'failed', 'cancelled'].includes(status.status)) {
          summary = status.summary || null;
          const kept = rays.size;
          if (status.status === 'complete') el('envelope-status').textContent = `Envelope complete · ${fmt(summary?.simulations)} simulations in ${fmt(summary?.elapsed_s, 0)} s.`;
          else if (status.status === 'cancelled') el('envelope-status').textContent = `Stopped · ${kept} / ${total} directions kept.`;
          else { el('envelope-status').textContent = `Envelope did not finish · ${kept} / ${total} directions kept.`; showError(status.error || 'Envelope calculation failed.'); }
          job = null; busy(false); await render(); setExports(kept > 0);
          break;
        }
        await new Promise(r => setTimeout(r, 900));
      }
    } finally { polling = false; }
  }
  function scheduleRender() {
    if (renderTimer) return;
    const wait = Math.max(0, 1200 - (performance.now() - lastRender));
    renderTimer = setTimeout(async () => { renderTimer = null; lastRender = performance.now(); try { await render(); } catch (error) { showError(error.message); } }, wait);
  }

  // ---- Geometry ---------------------------------------------------------------------
  const wrap = a => { let x = ((a + 180) % 360 + 360) % 360 - 180; return x === -180 ? 180 : x; };
  function config() { return submitted; }
  function azimuths() { return jobInfo?.azimuths_deg || azimuthList(submitted); }
  function mirrored() { return jobInfo ? jobInfo.mirrored : isMirrored(submitted); }
  function rayAt(layerIndex, azimuth) {
    const list = azimuths(), a = mirrored() ? Math.abs(wrap(azimuth)) : wrap(azimuth);
    const index = list.findIndex(v => Math.abs(v - a) < 1e-6);
    return index < 0 ? undefined : rays.get(`${layerIndex},${index}`);
  }
  function circle() {
    const step = qualities[submitted.quality].step, n = Math.round(360 / step);
    return Array.from({length: n + 1}, (_, i) => wrap(i * step));
  }
  const referenceAltitude = c => c.frame === 'target' ? c.target.altitude_m : c.launcher.altitude_m;
  // Launcher and target states at release for a sample (mirrors scripts/missile_envelope.py states()).
  function states(c, layer, azimuth, distance) {
    const a = radians(azimuth), offset = [distance * Math.cos(a), 0, distance * Math.sin(a)];
    const direction = (h, p, s) => [s * Math.cos(radians(p)) * Math.cos(radians(h)), s * Math.sin(radians(p)), s * Math.cos(radians(p)) * Math.sin(radians(h))];
    if (c.frame === 'target') {
      const altitude = c.target.altitude_m, h = wrap(azimuth + 180);
      const p = c.launcher.aim === 'nose' ? Math.atan2(altitude - layer, distance) * 180 / Math.PI : 0;
      return {launcher: {position: [0, layer, 0], velocity: direction(h, p, c.launcher.speed_mps), angles: [h, p, 0]},
        target: {position: [-offset[0], altitude, -offset[2]], velocity: direction(0, 0, c.target.speed_mps), angles: [0, 0, 0]}};
    }
    const p = c.launcher.pitch_deg, rule = c.target.heading;
    const h = wrap(typeof rule === 'string' ? {hot: azimuth + 180, cold: azimuth, same: 0, opposite: 180}[rule] : rule);
    return {launcher: {position: [0, c.launcher.altitude_m, 0], velocity: direction(0, p, c.launcher.speed_mps), angles: [0, p, 0]},
      target: {position: [offset[0], layer, offset[2]], velocity: direction(h, 0, c.target.speed_mps), angles: [h, 0, 0]}};
  }
  const bearingName = (c, az) => {
    const a = Math.abs(wrap(az)), side = wrap(az) > 0 ? ' · +Z side' : wrap(az) < 0 ? ' · −Z side' : '';
    if (c.frame === 'target') return `${fmt(a, 1)}° from the target's nose${a === 0 ? ' (head-on)' : a === 180 ? ' (tail chase)' : a === 90 ? ' (beam)' : side}`;
    return `${fmt(a, 1)}° off the launcher's nose${a === 0 ? ' (ahead)' : a === 180 ? ' (behind)' : side}`;
  };
  function rayText(c, li, az, ray) {
    const otherRole = c.frame === 'target' ? 'Launcher' : 'Target';
    const lines = [`<b>${otherRole} at ${fmt(c.layers_m[li])} m · ${escapeHtml(bearingName(c, az))}</b>`];
    if (!ray) lines.push('Not calculated yet');
    else if (ray.max_range_m === null) lines.push('No hit at any searched distance');
    else {
      lines.push(`Maximum range: ${km(ray.max_range_m)}${ray.intervals.at(-1).max_at_search_limit ? ' (search limit)' : ''} · flight ${fmt(ray.max_time_s, 1)} s`);
      lines.push(`Minimum range: ${ray.intervals[0].min_at_search_limit ? '≤ ' : ''}${km(ray.min_range_m)}`);
      if (ray.intervals.length > 1) lines.push(`${ray.intervals.length - 1} gap${ray.intervals.length > 2 ? 's' : ''} between hits`);
    }
    return lines.join('<br>');
  }

  // ---- Rendering --------------------------------------------------------------------
  const theme = {paper_bgcolor: '#121925', plot_bgcolor: '#121925', font: {color: '#dce5f0'}, hoverlabel: {bgcolor: '#172638', bordercolor: '#38c9d7', font: {color: '#e3edf5', size: 12}}};
  function radius(ray, kind) {
    if (!ray) return null;
    if (ray.max_range_m === null) return 0;
    return kind === 'max' ? ray.max_range_m : ray.min_range_m;
  }
  function viewControls() {
    const view = el('envelope-view').value;
    el('envelope-layer-label').hidden = view !== 'layer';
    el('envelope-cut-label').hidden = view !== 'profile';
    const c = submitted;
    el('envelope-view-note').textContent = !c ? '' : view === '3d'
      ? `Hover for ranges; click a point to inspect that direction. ${c.frame === 'target' ? 'Launcher' : 'Target'} positions are shown around the ${c.frame === 'target' ? 'target' : 'launcher'} (◆) flying along +X. Altitude may be stretched to keep the layers readable.`
      : view === 'layer' ? 'Top view of one altitude layer (X / Z, equal scale). The shaded ring is the envelope; click a point to inspect it.'
        : 'Vertical cut through the reference aircraft: horizontal distance along the chosen bearing (right) and its opposite (left), against altitude.';
  }
  function fillSelectors() {
    const c = submitted;
    const layerSelect = el('envelope-layer'), current = layerSelect.value;
    layerSelect.replaceChildren(...c.layers_m.map((alt, i) => { const o = document.createElement('option'); o.value = i; o.textContent = `${fmt(alt)} m`; return o; }));
    const ref = c.layers_m.reduce((best, alt, i) => Math.abs(alt - referenceAltitude(c)) < Math.abs(c.layers_m[best] - referenceAltitude(c)) ? i : best, 0);
    layerSelect.value = current !== '' && current < c.layers_m.length ? current : ref;
    const cutSelect = el('envelope-cut'), cut = cutSelect.value;
    const step = qualities[c.quality].step;
    const cuts = Array.from({length: Math.round(180 / step)}, (_, i) => i * step);
    cutSelect.replaceChildren(...cuts.map(a => { const o = document.createElement('option'); o.value = a; o.textContent = c.frame === 'target' ? `${fmt(a, 1)}° / ${fmt(wrap(a + 180), 1)}°${a === 0 ? ' · head-on / tail' : ''}` : `${fmt(a, 1)}° / ${fmt(wrap(a + 180), 1)}°${a === 0 ? ' · ahead / behind' : ''}`; return o; }));
    cutSelect.value = cuts.includes(Number(cut)) ? cut : 0;
  }
  async function render() {
    if (!submitted) return;
    fillSelectors(); viewControls(); renderTable(); renderMetrics();
    el('envelope-ref-name').textContent = submitted.frame === 'target' ? 'Target' : 'Launcher';
    const view = el('envelope-view').value;
    if (!rays.size) {
      if (el('envelope-chart').data) Plotly.purge(el('envelope-chart'));
      el('envelope-chart').innerHTML = '<div class="empty"><span class="empty-glyph">◎</span><h2>Searching…</h2><p>Directions appear here as soon as they finish.</p></div>';
      return;
    }
    if (view === '3d') await render3d(); else if (view === 'layer') await renderLayer(); else await renderProfile();
    if (inspected) renderInspector();
  }
  function bounds3d() {
    let extent = 1000;
    for (const ray of rays.values()) if (ray.max_range_m) extent = Math.max(extent, ray.max_range_m);
    return extent;
  }
  async function render3d() {
    const c = submitted, ring = circle(), showMin = el('envelope-show-min').checked;
    const grid = kind => c.layers_m.map((alt, li) => ring.map(az => { const r = radius(rayAt(li, az), kind); return r === null ? null : [r * Math.cos(radians(az)), r * Math.sin(radians(az)), alt]; }));
    const text = c.layers_m.map((alt, li) => ring.map(az => rayText(c, li, az, rayAt(li, az))));
    const custom = c.layers_m.map((alt, li) => ring.map(az => [li, az]));
    const surface = (points, name, color, opacity) => ({type: 'surface', name, showscale: false, opacity, hoverinfo: 'text', text, customdata: custom,
      hovertemplate: '%{text}<extra></extra>', colorscale: [[0, color], [1, color]], surfacecolor: points.map(row => row.map(() => 0)),
      x: points.map(row => row.map(p => p ? p[0] : null)), y: points.map(row => row.map(p => p ? p[1] : null)), z: points.map(row => row.map(p => p ? p[2] : null)),
      lighting: {ambient: .75, diffuse: .5, specular: .1, roughness: .9}, contours: {x: {highlight: false}, y: {highlight: false}, z: {highlight: false}}});
    const outer = grid('max'), inner = grid('min');
    const traces = [];
    if (c.layers_m.length > 1) {
      traces.push(surface(outer, 'Maximum range', '#38c9d7', .32));
      if (showMin) traces.push(surface(inner, 'Minimum range', '#ffa66b', .5));
    }
    // Layer rings, and flat annulus caps on the lowest and highest layers.
    const capRows = c.layers_m.length > 1 ? [0, c.layers_m.length - 1] : [0];
    for (const li of capRows) {
      const rows = [inner[li], outer[li]];
      traces.push({...surface(rows, 'Layer', '#38c9d7', c.layers_m.length > 1 ? .22 : .35), text: [text[li], text[li]], customdata: [custom[li], custom[li]]});
    }
    for (const [kind, color, show] of [['max', '#38c9d7', true], ['min', '#ffa66b', showMin]]) {
      if (!show) continue;
      const g = kind === 'max' ? outer : inner, xs = [], ys = [], zs = [], tx = [], cd = [];
      g.forEach((row, li) => { row.forEach((p, i) => { xs.push(p ? p[0] : null); ys.push(p ? p[1] : null); zs.push(p ? p[2] : null); tx.push(text[li][i]); cd.push(custom[li][i]); }); xs.push(null); ys.push(null); zs.push(null); tx.push(''); cd.push(null); });
      traces.push({type: 'scatter3d', mode: 'lines', name: kind === 'max' ? 'Maximum range' : 'Minimum range', x: xs, y: ys, z: zs, text: tx, customdata: cd,
        hovertemplate: '%{text}<extra></extra>', line: {color, width: kind === 'max' ? 5 : 3}, connectgaps: false});
    }
    if (el('envelope-show-samples').checked) traces.push(...sampleTraces3d());
    const extent = bounds3d(), ref = referenceAltitude(c), arrow = extent * .12;
    traces.push({type: 'scatter3d', mode: 'markers+text', name: c.frame === 'target' ? 'Target' : 'Launcher', x: [0], y: [0], z: [ref], text: [c.frame === 'target' ? 'Target' : 'Launcher'],
      textposition: 'top center', textfont: {color: '#ffe0c7', size: 11}, marker: {symbol: 'diamond', size: 6, color: '#ffd0ac'}, hoverinfo: 'skip'});
    traces.push({type: 'scatter3d', mode: 'lines', x: [0, arrow], y: [0, 0], z: [ref, ref], line: {color: '#ffd0ac', width: 4}, hoverinfo: 'skip', showlegend: false});
    traces.push({type: 'cone', x: [arrow], y: [0], z: [ref], u: [arrow * .3], v: [0], w: [0], anchor: 'tail', sizemode: 'absolute', sizeref: arrow * .35, showscale: false,
      colorscale: [[0, '#ffd0ac'], [1, '#ffd0ac']], hoverinfo: 'skip'});
    const alts = [...c.layers_m, ref], low = Math.min(...alts), high = Math.max(...alts);
    const span = Math.max(high - low, 1000), horizontal = 2 * extent * 1.08;
    const zRatio = Math.max(span / horizontal, .3), stretch = zRatio * horizontal / span;
    const node = el('envelope-chart'), viewport = (node.clientWidth || 800) / (node.clientHeight || 500);
    const fill = Math.min(1.9, 1.25 * Math.max(.75, Math.min(viewport, 1.6)) / (1 + zRatio * .6));
    const pad = (high - low) * .08 + 300;
    const axis = (title, range) => ({title: {text: title, font: {size: 11}}, range, tickformat: ',.0f', gridcolor: '#263244', zerolinecolor: '#455568', color: '#a9b8ca', backgroundcolor: '#121925', showspikes: false, tickfont: {size: 10}});
    const layout = {...theme, margin: {l: 0, r: 0, t: 0, b: 0}, showlegend: false, uirevision: 'envelope3d:' + JSON.stringify(submitted),
      annotations: [{text: stretch > 1.05 ? `Altitude stretched ×${fmt(stretch, 1)}` : 'Equal scale', xref: 'paper', yref: 'paper', x: .015, y: .985, xanchor: 'left', yanchor: 'top', showarrow: false, font: {size: 10, color: '#a9b8ca'}}],
      scene: {xaxis: axis('X · m', [-extent * 1.08, extent * 1.08]), yaxis: axis('Z · m', [-extent * 1.08, extent * 1.08]), zaxis: axis('Altitude · m', [Math.max(0, low - pad), high + pad]),
        aspectmode: 'manual', aspectratio: {x: fill, y: fill, z: zRatio * fill}, camera: {eye: {x: -1.15, y: -1.3, z: .8}, up: {x: 0, y: 0, z: 1}, projection: {type: 'orthographic'}}}};
    await draw(traces, layout);
  }
  function sampleTraces3d() {
    const c = submitted, hits = {x: [], y: [], z: [], t: []}, misses = {x: [], y: [], z: [], t: []};
    for (const ray of rays.values()) {
      const li = ray.key[0], alt = c.layers_m[li];
      for (const az of mirrored() && ray.azimuth_deg > 0 && ray.azimuth_deg < 180 ? [ray.azimuth_deg, -ray.azimuth_deg] : [ray.azimuth_deg]) {
        for (const s of ray.samples) {
          const target = s.hit ? hits : misses;
          target.x.push(s.range_m * Math.cos(radians(az))); target.y.push(s.range_m * Math.sin(radians(az))); target.z.push(alt);
          target.t.push(`${s.hit ? 'Hit' : 'Miss · ' + escapeHtml(outcomeLabel(s.outcome))} at ${km(s.range_m)}<br>${escapeHtml(bearingName(c, az))}${Number.isFinite(s.time_s) ? `<br>Ended ${fmt(s.time_s, 1)} s after release` : ''}`);
        }
      }
    }
    return [[hits, '#91d477', 'Hit sample'], [misses, '#f08070', 'Miss sample']].map(([d, color, name]) => ({type: 'scatter3d', mode: 'markers', name, x: d.x, y: d.y, z: d.z, text: d.t,
      hovertemplate: '%{text}<extra></extra>', marker: {size: 2.5, color, opacity: .9}}));
  }
  async function renderLayer() {
    const c = submitted, li = Number(el('envelope-layer').value), ring = circle(), showMin = el('envelope-show-min').checked;
    const pts = kind => ring.map(az => { const r = radius(rayAt(li, az), kind); return r === null ? [null, null] : [r * Math.cos(radians(az)), r * Math.sin(radians(az))]; });
    const text = ring.map(az => rayText(c, li, az, rayAt(li, az))), custom = ring.map(az => [li, az]);
    const outer = pts('max'), inner = pts('min');
    const traces = [{type: 'scatter', mode: 'lines', name: 'Maximum range', x: outer.map(p => p[0]), y: outer.map(p => p[1]), fill: 'toself', fillcolor: 'rgba(56,201,215,.18)',
      line: {color: '#38c9d7', width: 2.5}, text, customdata: custom, hovertemplate: '%{text}<extra></extra>', hoveron: 'points'}];
    traces.push({type: 'scatter', mode: 'lines', name: 'Minimum range', x: inner.map(p => p[0]), y: inner.map(p => p[1]), fill: 'toself', fillcolor: '#121925',
      line: {color: showMin ? '#ffa66b' : 'rgba(0,0,0,0)', width: 2}, text, customdata: custom, hovertemplate: '%{text}<extra></extra>', hoveron: 'points'});
    if (el('envelope-show-samples').checked) {
      const hits = {x: [], y: [], t: []}, misses = {x: [], y: [], t: []};
      for (const ray of rays.values()) {
        if (ray.key[0] !== li) continue;
        for (const az of mirrored() && ray.azimuth_deg > 0 && ray.azimuth_deg < 180 ? [ray.azimuth_deg, -ray.azimuth_deg] : [ray.azimuth_deg])
          for (const s of ray.samples) {
            const d = s.hit ? hits : misses;
            d.x.push(s.range_m * Math.cos(radians(az))); d.y.push(s.range_m * Math.sin(radians(az)));
            d.t.push(`${s.hit ? 'Hit' : 'Miss · ' + escapeHtml(outcomeLabel(s.outcome))} at ${km(s.range_m)}${Number.isFinite(s.time_s) ? ` · ended ${fmt(s.time_s, 1)} s` : ''}`);
          }
      }
      for (const [d, color, name, symbol] of [[hits, '#91d477', 'Hit', 'circle'], [misses, '#f08070', 'Miss', 'x']])
        traces.push({type: 'scatter', mode: 'markers', name, x: d.x, y: d.y, text: d.t, hovertemplate: '%{text}<extra></extra>', marker: {size: 6, color, symbol}});
    }
    const extent = bounds3d();
    traces.push({type: 'scatter', mode: 'markers+text', x: [0], y: [0], text: [c.frame === 'target' ? 'Target' : 'Launcher'], textposition: 'bottom center',
      marker: {symbol: 'diamond', size: 11, color: '#ffd0ac'}, textfont: {color: '#ffe0c7', size: 11}, hoverinfo: 'skip'});
    const layout = {...theme, margin: {l: 70, r: 25, t: 36, b: 60}, showlegend: false, hovermode: 'closest', uirevision: 'layer:' + JSON.stringify(submitted),
      xaxis: {title: {text: 'X · m'}, gridcolor: '#263244', zerolinecolor: '#455568', tickformat: ',.0f', range: [-extent * 1.1, extent * 1.1], constrain: 'domain'},
      yaxis: {title: {text: 'Z · m'}, gridcolor: '#263244', zerolinecolor: '#455568', tickformat: ',.0f', scaleanchor: 'x', scaleratio: 1, range: [-extent * 1.1, extent * 1.1]},
      annotations: [{x: extent * .14, y: 0, ax: 0, ay: 0, xref: 'x', yref: 'y', axref: 'x', ayref: 'y', showarrow: true, arrowhead: 2, arrowsize: 1.2, arrowwidth: 2, arrowcolor: '#ffd0ac', text: ''},
        {text: `${c.frame === 'target' ? 'Launcher' : 'Target'} altitude ${fmt(c.layers_m[li])} m · ${c.frame === 'target' ? 'target' : 'launcher'} at ${fmt(referenceAltitude(c))} m`, xref: 'paper', yref: 'paper', x: 0, y: 1.01, xanchor: 'left', yanchor: 'bottom', showarrow: false, font: {size: 10, color: '#a9b8ca'}}]};
    await draw(traces, layout);
  }
  async function renderProfile() {
    const c = submitted, cut = Number(el('envelope-cut').value), showMin = el('envelope-show-min').checked;
    const traces = [];
    for (const [az, sign] of [[cut, 1], [wrap(cut + 180), -1]]) {
      const rows = c.layers_m.map((alt, li) => ({alt, li, ray: rayAt(li, az)}));
      const maxPts = rows.map(r => ({x: sign * (radius(r.ray, 'max') ?? NaN), y: r.alt, t: rayText(c, r.li, az, r.ray), cd: [r.li, az]}));
      const minPts = rows.map(r => ({x: sign * (radius(r.ray, 'min') ?? NaN), y: r.alt, t: rayText(c, r.li, az, r.ray), cd: [r.li, az]}));
      const valid = p => Number.isFinite(p.x);
      const ring = [...maxPts.filter(valid), ...minPts.filter(valid).reverse()];
      if (ring.length) traces.push({type: 'scatter', mode: 'lines', x: [...ring.map(p => p.x), ring[0].x], y: [...ring.map(p => p.y), ring[0].y], fill: 'toself',
        fillcolor: 'rgba(56,201,215,.16)', line: {width: 0}, hoverinfo: 'skip'});
      traces.push({type: 'scatter', mode: 'lines+markers', name: 'Maximum range', x: maxPts.map(p => valid(p) ? p.x : null), y: maxPts.map(p => p.y), text: maxPts.map(p => p.t), customdata: maxPts.map(p => p.cd),
        hovertemplate: '%{text}<extra></extra>', line: {color: '#38c9d7', width: 2.5}, marker: {size: 6}});
      if (showMin) traces.push({type: 'scatter', mode: 'lines+markers', name: 'Minimum range', x: minPts.map(p => valid(p) ? p.x : null), y: minPts.map(p => p.y), text: minPts.map(p => p.t), customdata: minPts.map(p => p.cd),
        hovertemplate: '%{text}<extra></extra>', line: {color: '#ffa66b', width: 2}, marker: {size: 5}});
    }
    traces.push({type: 'scatter', mode: 'markers+text', x: [0], y: [referenceAltitude(c)], text: [c.frame === 'target' ? 'Target' : 'Launcher'], textposition: 'top center',
      marker: {symbol: 'diamond', size: 11, color: '#ffd0ac'}, textfont: {color: '#ffe0c7', size: 11}, hoverinfo: 'skip'});
    const label = a => c.frame === 'target' ? `${fmt(Math.abs(a), 1)}° from target nose` : `${fmt(Math.abs(a), 1)}° off launcher nose`;
    const layout = {...theme, margin: {l: 70, r: 25, t: 30, b: 60}, showlegend: false, hovermode: 'closest', uirevision: 'profile:' + JSON.stringify(submitted) + cut,
      xaxis: {title: {text: `← ${label(wrap(cut + 180))} · horizontal distance (m) · ${label(cut)} →`}, gridcolor: '#263244', zerolinecolor: '#455568', tickformat: ',.0f'},
      yaxis: {title: {text: 'Altitude · m'}, gridcolor: '#263244', zerolinecolor: '#455568', tickformat: ',.0f', rangemode: 'tozero'}};
    await draw(traces, layout);
  }
  async function draw(traces, layout) {
    const node = el('envelope-chart');
    if (node.querySelector('.empty')) node.replaceChildren();
    await Plotly.react(node, traces, layout, {responsive: true, displaylogo: false, toImageButtonOptions: {filename: 'missile-envelope', format: 'png', scale: 2}});
    node.removeAllListeners?.('plotly_click');
    node.on('plotly_click', event => {
      const point = event.points?.[0]; const cd = point?.customdata;
      if (Array.isArray(cd) && cd.length === 2 && Number.isFinite(cd[0])) { inspected = {layer: cd[0], azimuth: cd[1]}; renderInspector(); }
    });
  }
  function renderMetrics() {
    const c = submitted, ref = c.layers_m.reduce((best, alt, i) => Math.abs(alt - referenceAltitude(c)) < Math.abs(c.layers_m[best] - referenceAltitude(c)) ? i : best, 0);
    el('envelope-metric-front-label').textContent = c.frame === 'target' ? `Head-on maximum · ${fmt(c.layers_m[ref])} m` : `Maximum straight ahead · ${fmt(c.layers_m[ref])} m`;
    el('envelope-metric-rear-label').textContent = c.frame === 'target' ? `Tail-chase maximum · ${fmt(c.layers_m[ref])} m` : `Maximum behind · ${fmt(c.layers_m[ref])} m`;
    const value = az => { const ray = rayAt(ref, az); return !ray ? '—' : ray.max_range_m === null ? 'No hit' : km(ray.max_range_m); };
    el('envelope-metric-front').textContent = value(0); el('envelope-metric-rear').textContent = value(180);
    el('envelope-metric-sims').textContent = fmt(summary?.simulations ?? jobInfo?.progress?.simulations ?? [...rays.values()].reduce((s, r) => s + r.simulations, 0));
  }
  function renderTable() {
    const c = submitted, columns = [0, 45, 90, 135, 180].filter(a => azimuths().some(v => Math.abs(v - a) < 1e-6));
    const names = c.frame === 'target' ? {0: 'Head-on', 45: '45°', 90: 'Beam', 135: '135°', 180: 'Tail'} : {0: 'Ahead', 45: '45°', 90: 'Side', 135: '135°', 180: 'Behind'};
    el('envelope-table').tHead.innerHTML = `<tr><th>${c.frame === 'target' ? 'Launcher' : 'Target'} altitude</th>${columns.map(a => `<th>${names[a]}<small>min – max</small></th>`).join('')}</tr>`;
    el('envelope-table').tBodies[0].innerHTML = c.layers_m.map((alt, li) => `<tr><td>${fmt(alt)} m</td>${columns.map(a => {
      const ray = rayAt(li, a);
      if (!ray) return '<td class="hint">…</td>';
      if (ray.max_range_m === null) return '<td class="hint">no hit</td>';
      return `<td>${ray.intervals[0].min_at_search_limit ? '≤' : ''}${km(ray.min_range_m)} – ${km(ray.max_range_m)}</td>`;
    }).join('')}</tr>`).slice().reverse().join('');
  }
  function renderInspector() {
    const box = el('envelope-inspect'), button = el('envelope-open-simulator');
    if (!inspected || !submitted) { el('envelope-inspect-title').textContent = 'Select a point on the envelope'; box.innerHTML = '<p class="muted">Click the envelope to see that direction\'s searched distances, outcomes and flight times.</p>'; button.hidden = true; return; }
    const c = submitted, ray = rayAt(inspected.layer, inspected.azimuth);
    el('envelope-inspect-title').textContent = `${c.frame === 'target' ? 'Launcher' : 'Target'} at ${fmt(c.layers_m[inspected.layer])} m · ${bearingName(c, inspected.azimuth)}`;
    if (!ray) { box.innerHTML = '<p class="muted">This direction has not finished yet.</p>'; button.hidden = true; return; }
    const rows = ray.samples.map(s => `<tr><td>${km(s.range_m)}</td><td class="${s.hit ? 'envelope-hit' : 'envelope-miss'}">${s.hit ? 'Hit' : 'Miss'}</td><td>${escapeHtml(outcomeLabel(s.outcome))}</td><td>${Number.isFinite(s.time_s) ? fmt(s.time_s, 2) + ' s' : '—'}</td><td>${Number.isFinite(s.closest_m) ? closestText(s) : '—'}</td></tr>`).join('');
    const summaryText = ray.max_range_m === null ? 'No searched distance produced a hit.' :
      `Hits from ${ray.intervals[0].min_at_search_limit ? 'the closest searched distance, ' : ''}${km(ray.min_range_m)} to ${km(ray.max_range_m)}${ray.intervals.at(-1).max_at_search_limit ? ' (the search limit)' : ''}. Farthest hit flew ${fmt(ray.max_time_s, 1)} s.${ray.intervals.length > 1 ? ` Hits stop and resume ${ray.intervals.length - 1} time(s) in between.` : ''}`;
    const reach = jobInfo?.plan?.config?.reach_m;
    const test = submitted.criterion === 'fuze' ? 'A hit is a proximity-fuze event.' : `A hit passes within ${fmt(reach ?? NaN, 1)} m of the target (fuze off).`;
    box.innerHTML = `<p class="muted">${summaryText} ${ray.simulations} simulations searched ${km(ray.search[0])} – ${km(ray.search[1])}. ${test} Near a boundary, hits and narrow misses can alternate with very small changes in the launch.</p>
      <div class="envelope-table-wrap"><table class="envelope-samples"><thead><tr><th>Launch distance</th><th>Result</th><th>Ended by</th><th>At</th><th>Closest approach</th></tr></thead><tbody>${rows}</tbody></table></div>`;
    button.hidden = ray.max_range_m === null || typeof window.missileSimulator?.load !== 'function';
    button.title = submitted.criterion === 'kinematic' ? 'The simulator uses the proximity fuze, so a kinematic edge shot can end as a narrow miss there.' : '';
  }
  el('envelope-open-simulator').addEventListener('click', () => {
    const c = submitted, ray = inspected && rayAt(inspected.layer, inspected.azimuth);
    if (!ray || ray.max_range_m === null) return;
    const lastHit = ray.intervals.at(-1).last_hit_m;
    const s = states(c, c.layers_m[inspected.layer], inspected.azimuth, lastHit);
    window.missileSimulator.load({missile: c.missile, duration: Math.min(180, Math.ceil((ray.max_time_s || 60) + 5)), launcher: s.launcher, target: s.target});
  });
  for (const id of ['envelope-view', 'envelope-layer', 'envelope-cut', 'envelope-show-min', 'envelope-show-samples'])
    el(id).addEventListener('change', () => render().catch(error => showError(error.message)));
  el('envelope-view-reset').addEventListener('click', () => { const node = el('envelope-chart'); if (node.data) { Plotly.purge(node); render().catch(error => showError(error.message)); } });

  // ---- Export -----------------------------------------------------------------------
  function setExports(enabled) {
    for (const url of downloadUrls) URL.revokeObjectURL(url); downloadUrls = [];
    for (const id of ['envelope-download-json', 'envelope-download-csv']) { el(id).setAttribute('aria-disabled', String(!enabled)); el(id).removeAttribute('href'); }
    if (!enabled) return;
    const ordered = [...rays.values()].sort((a, b) => a.key[0] - b.key[0] || a.key[1] - b.key[1]);
    const data = {format: 'missile-envelope-v1', missile: submitted.missile, missile_name: missileName(submitted.missile), request: submitted,
      mirrored: mirrored(), azimuths_deg: azimuths(), complete: ordered.length === jobInfo?.rays_total, summary, rays: ordered,
      conventions: {frame: submitted.frame, azimuth: submitted.frame === 'target' ? 'Launcher bearing from the target nose; 0 = ahead of the target (head-on), positive toward +Z' : 'Target bearing from the launcher nose; positive toward +Z',
        range: 'Horizontal distance in metres from the reference aircraft at release', layer: submitted.frame === 'target' ? 'Launcher altitude (m)' : 'Target altitude (m)',
        hit: 'Native proximity event against the constant-velocity point target'}};
    const csv = [['layer_altitude_m', 'azimuth_deg', 'min_range_m', 'max_range_m', 'min_at_search_limit', 'max_at_search_limit', 'max_range_flight_time_s', 'hit_intervals', 'simulations'].join(',')]
      .concat(ordered.map(r => [r.layer_m, r.azimuth_deg, r.min_range_m ?? '', r.max_range_m ?? '', r.max_range_m === null ? '' : r.intervals[0].min_at_search_limit, r.max_range_m === null ? '' : r.intervals.at(-1).max_at_search_limit, r.max_time_s ?? '', r.intervals.length, r.simulations].join(','))).join('\n');
    for (const [id, content, type, name] of [['envelope-download-json', JSON.stringify(data, null, 1), 'application/json', `${submitted.missile}-envelope.json`], ['envelope-download-csv', csv, 'text/csv', `${submitted.missile}-envelope.csv`]]) {
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
