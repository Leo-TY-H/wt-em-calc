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
  // Base directions per layer; the search adds directions where the envelope bends, splits or ends.
  const qualities = {
    quick: {step: 30, edge: 1, label: 'Quick · 30° directions, edges to 1°'},
    standard: {step: 15, edge: .5, label: 'Standard · 15° directions, edges to 0.5°'},
    fine: {step: 10, edge: .25, label: 'Detailed · 10° directions, edges to 0.25°'}};
  const defaults = {missile: null, quality: 'standard',
    target: {altitude: 6000, speed: 900},
    launcher: {speed: 900, aim: 'nose'},
    layers: {from: 2000, to: 10000, count: 5}};
  const numberField = (id, label, unit, value, min, max, step = 'any') =>
    `<label class="field">${label}<div class="input-unit"><input id="${id}" type="number" min="${min}" max="${max}" step="${step}" value="${value}" required aria-label="${escapeHtml(label)}"><span>${unit}</span></div></label>`;

  root.innerHTML = `<aside class="missile-sidebar envelope-sidebar"><form id="envelope-form" novalidate>
    <div class="section-heading"><span class="eyebrow">ENVELOPE SETUP</span><button id="envelope-reset" class="text-button" type="button">Reset all</button></div>
    <h2 class="missile-setup-title">Where can it hit?</h2>
    <p class="hint">Searches the launch distance in every direction around the target, layer by layer, with the same engagement simulation as the missile simulator.</p>
    <fieldset class="missile-step" id="envelope-picker"><legend><span class="missile-step-number">1</span>Missile</legend>
      <input id="envelope-search" type="search" placeholder="Search missiles…" autocomplete="off" aria-label="Search missiles" aria-controls="envelope-list">
      <div id="envelope-list" class="aircraft-list" aria-label="Available missiles"></div>
      <p id="envelope-search-empty" class="hint" hidden>No missiles match your search.</p>
      <p id="envelope-catalog-note" class="hint" role="status">Loading missiles…</p>
      <div id="envelope-selected" class="envelope-selected" aria-live="polite"><span class="hint">No missile selected.</span></div>
      <button id="envelope-retry" class="text-button" type="button" hidden>Retry loading missiles</button>
    </fieldset>
    <fieldset class="missile-step missile-target"><legend><span class="missile-step-number">2</span>Target aircraft</legend>
      ${numberField('envelope-target-altitude', 'Altitude', 'm', defaults.target.altitude, 50, 25000)}
      ${numberField('envelope-target-speed', 'Speed', 'km/h', defaults.target.speed, 0, 7200)}
      <p class="hint">Sits at the centre of the envelope, flying straight and level along +X at constant speed.</p>
    </fieldset>
    <fieldset class="missile-step missile-launch"><legend><span class="missile-step-number">3</span>Launcher aircraft</legend>
      ${numberField('envelope-launcher-speed', 'Speed', 'km/h', defaults.launcher.speed, 0, 7200)}
      <label class="field">Attitude at launch<select id="envelope-launcher-aim"><option value="nose">Nose pointed at the target</option><option value="level">Level, heading toward the target</option></select></label>
      <p class="hint">Launches toward the target from every sampled position; its altitude is the layer altitude.</p>
    </fieldset>
    <fieldset class="missile-step"><legend><span class="missile-step-number">4</span>Launcher altitudes</legend>
      <div class="envelope-layer-inputs">${numberField('envelope-layers-from', 'Lowest', 'm', defaults.layers.from, 50, 25000)}${numberField('envelope-layers-to', 'Highest', 'm', defaults.layers.to, 50, 25000)}${numberField('envelope-layers-count', 'Layers', '', defaults.layers.count, 1, 25, 1)}</div>
      <p id="envelope-layers-list" class="hint"></p>
    </fieldset>
    <fieldset class="missile-step"><legend><span class="missile-step-number">5</span>Resolution</legend>
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
    <p class="missile-model-note">Each point of the envelope is a launcher position at release around the target: 0° is in front of the target (head-on), 180° behind it (tail chase). A launch counts as a hit when the missile's proximity fuze fires, exactly as in the missile simulator, so fuze arming and radius shape the envelope together with the motor, mass, aerodynamics, control limits, guidance and seeker limits.</p>
    <div id="envelope-error" class="error" role="alert" hidden></div>
    <div id="envelope-stale" class="notice" hidden>Settings changed. Calculate again to update the envelope.</div>
    <div id="envelope-progress" class="progress-panel" aria-live="polite" hidden><div><span class="spinner"></span><strong id="envelope-progress-label">Starting…</strong><span id="envelope-progress-count"></span></div><div class="progress-track"><div id="envelope-progress-fill"></div></div></div>
    <section class="chart-card">
      <div class="chart-toolbar"><div class="missile-legend envelope-legend"><span class="envelope-key-max">■ Hit region</span><span class="envelope-key-ref">◆ Target</span></div>
        <div class="missile-view-controls">
          <label for="envelope-view">Plot view<select id="envelope-view"><option value="3d">3D envelope</option><option value="layer">Layer · top view</option><option value="profile">Side profile</option></select></label>
          <label id="envelope-layer-label" for="envelope-layer" hidden>Layer<select id="envelope-layer"></select></label>
          <label id="envelope-cut-label" for="envelope-cut" hidden>Cut through<select id="envelope-cut"></select></label>
          <label class="envelope-check"><input id="envelope-show-rays" type="checkbox"> Directions</label>
          <label class="envelope-check"><input id="envelope-show-samples" type="checkbox"> Samples</label>
          <button id="envelope-view-reset" type="button">Reset view</button>
        </div></div>
      <p id="envelope-view-note" class="hint missile-hover-hint"></p>
      <div id="envelope-chart" aria-label="Missile launch envelope"><div class="empty"><span class="empty-glyph">◎</span><h2>No envelope yet</h2><p>Choose a missile and the flight conditions,<br>then select <strong>Calculate envelope</strong>.</p></div></div>
    </section>
    <div class="missile-metrics envelope-metrics">
      <div class="missile-metric"><span id="envelope-metric-front-label">Head-on maximum</span><strong id="envelope-metric-front">—</strong></div>
      <div class="missile-metric"><span id="envelope-metric-rear-label">Tail-chase maximum</span><strong id="envelope-metric-rear">—</strong></div>
      <div class="missile-metric"><span>Directions · simulations</span><strong id="envelope-metric-sims">—</strong></div>
    </div>
    <section class="inspector envelope-inspector" aria-live="polite"><div class="inspector-heading"><div><span class="eyebrow">DIRECTION</span><h2 id="envelope-inspect-title">Select a point on the envelope</h2></div><button id="envelope-open-simulator" class="secondary-button" type="button" hidden>Fly this shot in the simulator ↗</button></div><div id="envelope-inspect"></div></section>
    <section class="envelope-table-card"><h2>Ranges by layer</h2><div class="envelope-table-wrap"><table id="envelope-table"><thead></thead><tbody><tr><td class="hint">Calculate an envelope to fill this table.</td></tr></tbody></table></div></section>
    <div class="exports"><span>EXPORT</span><a id="envelope-download-json" aria-disabled="true">Envelope JSON</a><a id="envelope-download-csv" aria-disabled="true">Directions CSV</a></div>
    <details class="missile-details"><summary>Method and limitations</summary>
      <p>The target sits at the centre, flying along +X; each layer places the launcher at one altitude, heading toward the target. Along horizontal directions from the target, the launch distance is searched for the change between hits and misses with complete simulator runs, until each change is bracketed within the selected tolerance. Directions start on a regular grid. Between two finished neighbours another direction is added where one hits and the other does not, where their gaps between hits differ, or where the boundary bends, so the outline follows the envelope's edges closely while open sky costs little. New directions start from the boundaries their neighbours predict and confirm them with simulations.</p>
      <p>Every simulation uses the missile simulator's model: recovered release processing, motors and changing mass, aerodynamics, control limits, guidance and seeker geometry, with ideal target visibility and radar support, against a target flying straight and level at constant speed. A hit is the native proximity-fuze event, including its arming time and distance. Near a boundary, hits and narrow misses can alternate with very small changes in the launch; each search keeps the changes it brackets, and hits scattered closer than a few percent of the envelope's size do not steer where new directions go. A miss is declared early only once every motor has burned out and even converting all altitude to speed could not bring the missile to the target before its lifetime or distance limit.</p>
      <p>The drawn region interpolates smoothly between searched directions and between layers; the searched boundaries are exact simulation results (show them with <strong>Directions</strong>). Seeker look-angle and tracking limits apply as in the simulator; the launcher always heads toward the target, so they matter mainly for level launches at large height differences. The setup is mirror-symmetric (left/right), so one side is searched and mirrored. Hardpoint offsets, aircraft damage and countermeasures are not modeled.</p>
    </details>
  </div>`;

  let meta = null, loading = null, selected = null, job = null, rays = new Map(), submitted = null, summary = null;
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

  function layers() {
    const from = value('envelope-layers-from'), to = value('envelope-layers-to'), count = Math.round(value('envelope-layers-count'));
    if (!(count >= 1)) return [];
    if (count === 1) return [from];
    return [...new Set(Array.from({length: count}, (_, i) => Math.round(from + (to - from) * i / (count - 1))))].sort((a, b) => a - b);
  }
  function inputs() {
    return {missile: selected, quality: el('envelope-quality').value, layers_m: layers(),
      target: {speed_mps: value('envelope-target-speed') / 3.6, altitude_m: value('envelope-target-altitude')},
      launcher: {speed_mps: value('envelope-launcher-speed') / 3.6, aim: el('envelope-launcher-aim').value}};
  }
  function updateForm() {
    const list = layers();
    el('envelope-layers-list').textContent = list.length ? `Layers: ${list.map(v => fmt(v) + ' m').join(', ')}` : '';
    const q = qualities[el('envelope-quality').value], base = (180 / q.step + 1) * list.length;
    el('envelope-cost').textContent = `${base} starting directions (left/right mirrored); more are added where the envelope bends, splits or ends. Long-range radar missiles take longest.`;
    const fields = [...el('envelope-form').querySelectorAll('input[type=number]:not(:disabled)')];
    const invalid = fields.find(field => !field.checkValidity());
    let message = '';
    if (!meta) message = 'Loading missiles…';
    else if (!selected) message = 'Select a missile.';
    else if (invalid) message = `${invalid.getAttribute('aria-label')}: ${invalid.validationMessage}`;
    else if (value('envelope-layers-to') < value('envelope-layers-from')) message = 'Highest layer must be at or above the lowest.';
    else message = job ? `Calculating ${missileName(submitted?.missile || selected)} envelope…` : `${missileName(selected)} · ${list.length} layer${list.length > 1 ? 's' : ''} ready.`;
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
    for (const [id, v] of [['envelope-target-altitude', defaults.target.altitude], ['envelope-target-speed', defaults.target.speed],
      ['envelope-launcher-speed', defaults.launcher.speed], ['envelope-layers-from', defaults.layers.from], ['envelope-layers-to', defaults.layers.to],
      ['envelope-layers-count', defaults.layers.count]]) el(id).value = v;
    el('envelope-launcher-aim').value = defaults.launcher.aim;
    el('envelope-quality').value = defaults.quality; el('envelope-search').value = '';
    if (meta) filter();
    updateForm();
  }
  el('envelope-reset').addEventListener('click', () => { if (!job) reset(); });

  // ---- Calculation ----------------------------------------------------------------
  function busy(on) {
    el('envelope-cancel').hidden = !on; el('envelope-reset').disabled = on;
    el('envelope-progress').hidden = !on;
    filter(); updateForm();
  }
  el('envelope-form').addEventListener('submit', async event => {
    event.preventDefault();
    if (job || el('envelope-run').disabled) return;
    showError(''); summary = null; rays = new Map(); inspected = null; jobInfo = null; cache = null;
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
        if (status.rays.length) cache = null;
        const p = status.progress, total = Math.max(p?.rays_total || status.rays_total || 1, rays.size), done = rays.size;
        const elapsed = (performance.now() - started) / 1000;
        el('envelope-progress-label').textContent = status.status === 'queued' ? 'Waiting for the previous envelope…' : `Searching ${missileName(submitted.missile)} envelope…`;
        el('envelope-progress-count').textContent = `${done} / ${total} directions · ${fmt(p?.simulations || 0)} simulations · ${Math.floor(elapsed / 60)}:${String(Math.floor(elapsed % 60)).padStart(2, '0')}`;
        el('envelope-progress-fill').style.width = `${100 * done / total}%`;
        el('envelope-status').textContent = `${done} directions finished.`;
        if (status.rays.length) scheduleRender();
        if (['complete', 'failed', 'cancelled'].includes(status.status)) {
          summary = status.summary || null;
          if (status.status === 'complete') el('envelope-status').textContent = `Envelope complete · ${fmt(rays.size)} directions, ${fmt(summary?.simulations)} simulations in ${fmt(summary?.elapsed_s, 1)} s.`;
          else if (status.status === 'cancelled') el('envelope-status').textContent = `Stopped · ${rays.size} directions kept.`;
          else { el('envelope-status').textContent = `Envelope did not finish · ${rays.size} directions kept.`; showError(status.error || 'Envelope calculation failed.'); }
          job = null; busy(false); await render(); setExports(rays.size > 0);
          break;
        }
        await new Promise(r => setTimeout(r, 700));
      }
    } finally { polling = false; }
  }
  function scheduleRender() {
    if (renderTimer) return;
    const wait = Math.max(0, 1000 - (performance.now() - lastRender));
    renderTimer = setTimeout(async () => { renderTimer = null; lastRender = performance.now(); try { await render(); } catch (error) { showError(error.message); } }, wait);
  }

  // ---- Geometry ---------------------------------------------------------------------
  const wrap = a => { let x = ((a + 180) % 360 + 360) % 360 - 180; return x === -180 ? 180 : x; };
  function mirrored() { return jobInfo ? jobInfo.mirrored : true; }
  const baseStep = () => jobInfo?.config?.azimuth_step_deg || qualities[submitted.quality].step;
  // Launcher and target states at release for a sample (mirrors scripts/missile_envelope.py states()).
  function states(c, layer, azimuth, distance) {
    const a = radians(azimuth);
    const direction = (h, p, s) => [s * Math.cos(radians(p)) * Math.cos(radians(h)), s * Math.sin(radians(p)), s * Math.cos(radians(p)) * Math.sin(radians(h))];
    const altitude = c.target.altitude_m, h = wrap(azimuth + 180);
    const p = c.launcher.aim === 'nose' ? Math.atan2(altitude - layer, distance) * 180 / Math.PI : 0;
    return {launcher: {position: [0, layer, 0], velocity: direction(h, p, c.launcher.speed_mps), angles: [h, p, 0]},
      target: {position: [-distance * Math.cos(a), altitude, -distance * Math.sin(a)], velocity: direction(0, 0, c.target.speed_mps), angles: [0, 0, 0]}};
  }
  const bearingName = az => {
    const a = Math.abs(wrap(az)), side = wrap(az) > 0 ? ' · +Z side' : wrap(az) < 0 ? ' · −Z side' : '';
    return `${fmt(a, a % 1 ? 2 : 0)}° from the target's nose${a === 0 ? ' (head-on)' : a === 180 ? ' (tail chase)' : a === 90 ? ' (beam)' + side : side}`;
  };
  function rayText(ray, az) {
    const lines = [`<b>Launcher at ${fmt(ray.layer_m)} m · ${escapeHtml(bearingName(az))}</b>`];
    if (ray.max_range_m === null) lines.push('No hit at any searched distance');
    else {
      lines.push(`Maximum range: ${km(ray.max_range_m)}${ray.intervals.at(-1).max_at_search_limit ? ' (search limit)' : ''} · flight ${fmt(ray.max_time_s, 1)} s`);
      lines.push(`Minimum range: ${ray.intervals[0].min_at_search_limit ? '≤ ' : ''}${km(ray.min_range_m)}`);
      if (ray.intervals.length > 1) lines.push(`${ray.intervals.length - 1} gap${ray.intervals.length > 2 ? 's' : ''} between hits`);
    }
    return lines.join('<br>');
  }

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
  function field() {
    if (cache) return cache;
    const c = submitted, mirror = mirrored(), byLayer = c.layers_m.map(() => []);
    // Plot window: the hit region and the target, with a margin.
    const box = [0, 0, 0, 0];
    for (const ray of rays.values()) {
      const iv = ray.intervals.flatMap(i => [LOG(i.start_m), LOG(i.end_m)]);
      const az = wrap(ray.azimuth_deg);
      byLayer[ray.key[0]].push({az, ray, iv});
      if (mirror && az > 0 && az < 180) byLayer[ray.key[0]].push({az: -az, ray, iv});
      for (const a of mirror ? [az, -az] : [az]) for (const i of ray.intervals) for (const r of [i.start_m, i.end_m]) {
        const x = r * Math.cos(radians(a)), y = r * Math.sin(radians(a));
        box[0] = Math.min(box[0], x); box[1] = Math.max(box[1], x); box[2] = Math.min(box[2], y); box[3] = Math.max(box[3], y);
      }
    }
    const lines = byLayer.map(list => {
      list.sort((a, b) => a.az - b.az);
      return {az: list.map(d => d.az), entries: list};
    });
    const span = Math.max(1000, box[1] - box[0], box[3] - box[2]), pad = span * .07;
    let radius = 1000;
    for (const ray of rays.values()) if (ray.max_range_m) radius = Math.max(radius, ray.max_range_m);
    cache = {lines, window: {x: [box[0] - pad, box[1] + pad], y: [box[2] - pad, box[3] + pad]}, radius: radius * 1.07};
    return cache;
  }
  // s at horizontal distance r and bearing az (deg) in one layer.
  function layerValue(line, r, az) {
    if (!line.az.length) return EMPTY;
    const L = LOG(r);
    if (line.az.length === 1) return signed(line.entries[0].iv, L);
    // PCHIP over the four directions around az; the others do not influence it.
    const n = line.az.length, x = line.az[0] + ((az - line.az[0]) % 360 + 360) % 360;
    let k = n - 1;
    if (x < line.az[n - 1]) { let lo = 0, hi = n - 1; while (hi - lo > 1) { const m = (lo + hi) >> 1; if (line.az[m] <= x) lo = m; else hi = m; } k = lo; }
    const xs = [], ys = [];
    for (let i = k - 1; i <= k + 2; i++) {
      const j = ((i % n) + n) % n, turns = Math.floor(i / n);
      xs.push(line.az[j] + turns * 360); ys.push(signed(line.entries[j].iv, L));
    }
    return pchip(xs, ys, x);
  }
  // s at one horizontal point and altitude, interpolated across layers.
  function volumeValue(values, altitude) {
    const alts = submitted.layers_m;
    return pchip(alts, values, altitude);
  }
  // Square cells over the plot window, n along its longer side.
  function axes(win, n) {
    const h = Math.max(win.x[1] - win.x[0], win.y[1] - win.y[0]) / (n - 1);
    const steps = ([a, b]) => Array.from({length: Math.max(2, Math.ceil((b - a) / h) + 1)}, (_, i) => a + i * h);
    return {xs: steps(win.x), ys: steps(win.y)};
  }
  function layerGrid(line, win, n) {
    const {xs, ys} = axes(win, n);
    return {xs, ys, z: ys.map(y => xs.map(x => layerValue(line, Math.hypot(x, y), Math.atan2(y, x) * 180 / Math.PI)))};
  }
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
    const key = p => `${p[0].toFixed(3)},${p[1].toFixed(3)}`, ends = new Map(), used = new Uint8Array(segments.length);
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
  // Hoverable searched boundary points of one layer: [x, y, text, [layer, azimuth]].
  function boundaryPoints(line) {
    const pts = [];
    for (const {az, ray} of line.entries) {
      const a = radians(az), text = rayText(ray, az);
      for (const i of ray.intervals) for (const r of [i.start_m, i.end_m]) pts.push([r * Math.cos(a), r * Math.sin(a), text, [ray.key[0], az]]);
      if (!ray.intervals.length) pts.push([0, 0, text, [ray.key[0], az]]);
    }
    return pts;
  }

  // ---- Rendering --------------------------------------------------------------------
  const font = 'WTSymbols, D-DIN, Bahnschrift, DIN Alternate, Segoe UI, sans-serif';
  const theme = {paper_bgcolor: '#000000', plot_bgcolor: '#000000', font: {color: '#dce5f0', family: font},
    hoverlabel: {bgcolor: '#0c0c0f', bordercolor: '#38c9d7', font: {color: '#f0f0fa', size: 12, family: font}},
    modebar: {bgcolor: 'rgba(0,0,0,0)', color: '#8a8a96', activecolor: '#f0f0fa'}};
  const grid = 'rgba(240,240,250,.09)', zero = 'rgba(240,240,250,.3)', tick = '#b9b9c4', cyan = '#38c9d7', fill = 'rgba(56,201,215,.22)';
  const axis2d = title => ({title: {text: title}, gridcolor: grid, zerolinecolor: zero, color: tick, tickformat: ',.0f'});
  function viewControls() {
    const view = el('envelope-view').value;
    el('envelope-layer-label').hidden = view !== 'layer';
    el('envelope-cut-label').hidden = view !== 'profile';
    el('envelope-view-note').textContent = !submitted ? '' : view === '3d'
      ? 'Launcher positions around the target (◆), which flies along +X. Hover the layer outlines for ranges; click one to inspect that direction. Altitude may be stretched to keep the layers readable.'
      : view === 'layer' ? 'Top view of one launcher altitude (X / Z, equal scale). The shaded region is where a launch from there hits; click its outline to inspect a direction.'
        : 'Vertical cut through the target: horizontal distance along the chosen bearing (right) and its opposite (left), against launcher altitude.';
  }
  const nearestLayer = c => c.layers_m.reduce((best, alt, i) => Math.abs(alt - c.target.altitude_m) < Math.abs(c.layers_m[best] - c.target.altitude_m) ? i : best, 0);
  function fillSelectors() {
    const c = submitted;
    const layerSelect = el('envelope-layer'), current = layerSelect.value;
    layerSelect.replaceChildren(...c.layers_m.map((alt, i) => { const o = document.createElement('option'); o.value = i; o.textContent = `${fmt(alt)} m`; return o; }));
    layerSelect.value = current !== '' && current < c.layers_m.length ? current : nearestLayer(c);
    const cutSelect = el('envelope-cut'), cut = cutSelect.value, step = Math.min(15, baseStep());
    const cuts = Array.from({length: Math.round(180 / step)}, (_, i) => i * step);
    cutSelect.replaceChildren(...cuts.map(a => { const o = document.createElement('option'); o.value = a; o.textContent = `${fmt(a, 1)}° / ${fmt(wrap(a + 180), 1)}°${a === 0 ? ' · head-on / tail' : ''}`; return o; }));
    cutSelect.value = cuts.includes(Number(cut)) ? cut : 0;
  }
  async function render() {
    if (!submitted) return;
    fillSelectors(); viewControls(); renderTable(); renderMetrics();
    if (!rays.size) {
      if (el('envelope-chart').data) Plotly.purge(el('envelope-chart'));
      el('envelope-chart').innerHTML = '<div class="empty"><span class="empty-glyph">◎</span><h2>Searching…</h2><p>Directions appear here as soon as they finish.</p></div>';
      return;
    }
    const view = el('envelope-view').value;
    if (view === '3d') await render3d(); else if (view === 'layer') await renderLayer(); else await renderProfile();
    if (inspected) renderInspector();
  }
  const targetMarker = (x, y, extra = {}) => ({type: 'scatter', mode: 'markers+text', x: [x], y: [y], text: ['Target'], textposition: 'bottom center',
    marker: {symbol: 'diamond', size: 11, color: '#ffd0ac'}, textfont: {color: '#ffe0c7', size: 11}, hoverinfo: 'skip', ...extra});
  function sampleTraces(filterRay, place, is3d) {
    const hits = {x: [], y: [], z: [], t: []}, misses = {x: [], y: [], z: [], t: []};
    for (const ray of rays.values()) {
      if (!filterRay(ray)) continue;
      for (const az of mirrored() && ray.azimuth_deg > 0 && ray.azimuth_deg < 180 ? [ray.azimuth_deg, -ray.azimuth_deg] : [ray.azimuth_deg])
        for (const s of ray.samples) {
          const p = place(ray, az, s.range_m); if (!p) continue;
          const d = s.hit ? hits : misses;
          d.x.push(p[0]); d.y.push(p[1]); if (is3d) d.z.push(p[2]);
          d.t.push(`${s.hit ? 'Hit' : 'Miss · ' + escapeHtml(outcomeLabel(s.outcome))} at ${km(s.range_m)}<br>${escapeHtml(bearingName(az))}${Number.isFinite(s.time_s) ? `<br>Ended ${fmt(s.time_s, 1)} s after release` : ''}`);
        }
    }
    return [[hits, '#91d477', 'Hit sample', 'circle'], [misses, '#f08070', 'Miss sample', is3d ? 'x' : 'x']].map(([d, color, name, symbol]) => ({
      type: is3d ? 'scatter3d' : 'scatter', mode: 'markers', name, x: d.x, y: d.y, ...(is3d ? {z: d.z} : {}), text: d.t,
      hovertemplate: '%{text}<extra></extra>', marker: {size: is3d ? 2.5 : 5, color, symbol, opacity: .9}}));
  }
  function rayTraces(line, place, is3d) {
    const xs = [], ys = [], zs = [];
    for (const {az, ray} of line.entries) for (const i of ray.intervals) {
      const a = place(ray, az, i.start_m), b = place(ray, az, i.end_m);
      xs.push(a[0], b[0], null); ys.push(a[1], b[1], null); if (is3d) zs.push(a[2], b[2], null);
    }
    return {type: is3d ? 'scatter3d' : 'scatter', mode: 'lines', x: xs, y: ys, ...(is3d ? {z: zs} : {}), hoverinfo: 'skip', line: {color: 'rgba(240,240,250,.35)', width: 1}};
  }
  function markerTrace(points, is3d, altitude) {
    return {type: is3d ? 'scatter3d' : 'scatter', mode: 'markers', x: points.map(p => p[0]), y: points.map(p => p[1]),
      ...(is3d ? {z: points.map(() => altitude)} : {}), text: points.map(p => p[2]), customdata: points.map(p => p[3]),
      hovertemplate: '%{text}<extra></extra>', marker: {size: is3d ? 2.5 : 6, color: cyan, opacity: is3d ? .7 : .55}};
  }
  // Plotly shades where a constraint fails: '<' 0 shades the hit region s >= 0.
  const region = (x, y, z) => ({type: 'contour', x, y, z, contours: {type: 'constraint', operation: '<', value: 0},
    fillcolor: fill, line: {color: cyan, width: 2.2, smoothing: 1}, hoverinfo: 'skip', showscale: false, showlegend: false});
  async function renderLayer() {
    const c = submitted, li = Number(el('envelope-layer').value), f = field(), line = f.lines[li], win = f.window;
    const g = layerGrid(line, win, 321);
    const traces = [region(g.xs, g.ys, g.z)];
    const place = (ray, az, r) => [r * Math.cos(radians(az)), r * Math.sin(radians(az))];
    if (el('envelope-show-rays').checked) traces.push(rayTraces(line, place, false));
    if (el('envelope-show-samples').checked) traces.push(...sampleTraces(ray => ray.key[0] === li, place, false));
    traces.push(markerTrace(boundaryPoints(line).filter(p => p[0] || p[1]), false));
    traces.push(targetMarker(0, 0));
    const arrow = Math.max(win.x[1] - win.x[0], win.y[1] - win.y[0]) * .06;
    const layout = {...theme, margin: {l: 70, r: 25, t: 36, b: 60}, showlegend: false, hovermode: 'closest', uirevision: 'layer:' + JSON.stringify(submitted),
      xaxis: {...axis2d('X · m'), range: win.x, constrain: 'domain'},
      yaxis: {...axis2d('Z · m'), scaleanchor: 'x', scaleratio: 1, range: win.y, constrain: 'domain'},
      annotations: [{x: arrow, y: 0, ax: 0, ay: 0, xref: 'x', yref: 'y', axref: 'x', ayref: 'y', showarrow: true, arrowhead: 2, arrowsize: 1.2, arrowwidth: 2, arrowcolor: '#ffd0ac', text: ''},
        {text: `Launcher altitude ${fmt(c.layers_m[li])} m · target at ${fmt(c.target.altitude_m)} m`, xref: 'paper', yref: 'paper', x: 0, y: 1.01, xanchor: 'left', yanchor: 'bottom', showarrow: false, font: {size: 10, color: tick}}]};
    await draw(traces, layout);
  }
  async function renderProfile() {
    const c = submitted, cut = Number(el('envelope-cut').value), f = field(), extent = f.radius;
    const label = a => `${fmt(Math.abs(a), 1)}° from target nose`;
    const nx = 401, xs = Array.from({length: nx}, (_, i) => -extent + 2 * extent * i / (nx - 1));
    const along = x => [Math.abs(x), x >= 0 ? cut : wrap(cut + 180)];
    const perLayer = f.lines.map(line => xs.map(x => { const [r, az] = along(x); return layerValue(line, r, az); }));
    const traces = [];
    if (c.layers_m.length > 1) {
      const low = c.layers_m[0], high = c.layers_m.at(-1), ny = Math.min(241, 24 * c.layers_m.length + 1);
      const ys = Array.from({length: ny}, (_, j) => low + (high - low) * j / (ny - 1));
      traces.push(region(xs, ys, ys.map(y => xs.map((_, i) => volumeValue(perLayer.map(v => v[i]), y)))));
    } else {
      const segment = {x: [], y: []};
      xs.forEach((x, i) => { segment.x.push(perLayer[0][i] > 0 ? x : null); segment.y.push(c.layers_m[0]); });
      traces.push({type: 'scatter', mode: 'lines', x: segment.x, y: segment.y, line: {color: cyan, width: 6}, hoverinfo: 'skip'});
    }
    // Searched boundaries on the cut, where a direction lies on it.
    const pts = [];
    for (const ray of rays.values()) {
      for (const [az, sign] of [[cut, 1], [wrap(cut + 180), -1]]) {
        const match = Math.abs(wrap(ray.azimuth_deg) - az) < 1e-6 || (mirrored() && Math.abs(Math.abs(wrap(ray.azimuth_deg)) - Math.abs(az)) < 1e-6);
        if (!match) continue;
        for (const i of ray.intervals) for (const r of [i.start_m, i.end_m]) pts.push([sign * r, ray.layer_m, rayText(ray, az), [ray.key[0], az]]);
      }
    }
    if (el('envelope-show-samples').checked) traces.push(...sampleTraces(() => true, (ray, az, r) => {
      if (Math.abs(wrap(az) - cut) < 1e-6) return [r, ray.layer_m];
      if (Math.abs(wrap(az) - wrap(cut + 180)) < 1e-6) return [-r, ray.layer_m];
      return null;
    }, false));
    traces.push(markerTrace(pts, false));
    traces.push(targetMarker(0, c.target.altitude_m, {textposition: 'top center'}));
    // Show both sides of the cut only as far as hits reach.
    const reach = [0, 0];
    perLayer.forEach(v => v.forEach((s, i) => { if (s > 0) { reach[0] = Math.min(reach[0], xs[i]); reach[1] = Math.max(reach[1], xs[i]); } }));
    const margin = .07 * Math.max(1000, reach[1] - reach[0]);
    const layout = {...theme, margin: {l: 70, r: 25, t: 30, b: 60}, showlegend: false, hovermode: 'closest', uirevision: 'profile:' + JSON.stringify(submitted) + cut,
      xaxis: {...axis2d(`← ${label(wrap(cut + 180))} · horizontal distance (m) · ${label(cut)} →`), range: [reach[0] - margin, reach[1] + margin]},
      yaxis: {...axis2d('Launcher altitude · m'), rangemode: 'tozero'}};
    await draw(traces, layout);
  }
  async function render3d() {
    const c = submitted, f = field(), win = f.window, alts = c.layers_m;
    const traces = [];
    if (alts.length > 1) {
      const {xs, ys} = axes(win, 61);
      const perLayer = f.lines.map(line => ys.map(y => xs.map(x => layerValue(line, Math.hypot(x, y), Math.atan2(y, x) * 180 / Math.PI))));
      const levels = Math.min(41, 4 * (alts.length - 1) + 1), zs = Array.from({length: levels}, (_, k) => alts[0] + (alts.at(-1) - alts[0]) * k / (levels - 1));
      const X = [], Y = [], Z = [], value = [];
      for (const z of zs) for (let j = 0; j < ys.length; j++) for (let i = 0; i < xs.length; i++) {
        X.push(xs[i]); Y.push(ys[j]); Z.push(z); value.push(volumeValue(perLayer.map(g => g[j][i]), z));
      }
      traces.push({type: 'isosurface', x: X, y: Y, z: Z, value, isomin: 0, isomax: CLAMP, surface: {count: 1}, opacity: .38,
        caps: {x: {show: false}, y: {show: false}, z: {show: true, fill: 1}}, colorscale: [[0, cyan], [1, cyan]], showscale: false,
        lighting: {ambient: .75, diffuse: .5, specular: .1, roughness: .9}, hoverinfo: 'skip'});
    }
    // Layer outlines at full resolution, with hoverable searched boundaries.
    f.lines.forEach((line, li) => {
      if (!line.az.length) return;
      const g = layerGrid(line, win, 201), o = outline(g.xs, g.ys, g.z);
      traces.push({type: 'scatter3d', mode: 'lines', x: o.x, y: o.y, z: o.x.map(v => v === null ? null : alts[li]), hoverinfo: 'skip',
        line: {color: cyan, width: 4}, connectgaps: false});
      traces.push(markerTrace(boundaryPoints(line).filter(p => p[0] || p[1]), true, alts[li]));
      if (el('envelope-show-rays').checked) traces.push(rayTraces(line, (ray, az, r) => [r * Math.cos(radians(az)), r * Math.sin(radians(az)), alts[li]], true));
    });
    if (el('envelope-show-samples').checked) traces.push(...sampleTraces(() => true, (ray, az, r) => [r * Math.cos(radians(az)), r * Math.sin(radians(az)), ray.layer_m], true));
    const wx = win.x[1] - win.x[0], wy = win.y[1] - win.y[0], horizontal = Math.max(wx, wy);
    const ref = c.target.altitude_m, arrow = horizontal * .06;
    traces.push({type: 'scatter3d', mode: 'markers+text', x: [0], y: [0], z: [ref], text: ['Target'],
      textposition: 'top center', textfont: {color: '#ffe0c7', size: 11}, marker: {symbol: 'diamond', size: 6, color: '#ffd0ac'}, hoverinfo: 'skip'});
    traces.push({type: 'scatter3d', mode: 'lines', x: [0, arrow], y: [0, 0], z: [ref, ref], line: {color: '#ffd0ac', width: 4}, hoverinfo: 'skip'});
    traces.push({type: 'cone', x: [arrow], y: [0], z: [ref], u: [arrow * .3], v: [0], w: [0], anchor: 'tail', sizemode: 'absolute', sizeref: arrow * .35, showscale: false,
      colorscale: [[0, '#ffd0ac'], [1, '#ffd0ac']], hoverinfo: 'skip'});
    const all = [...alts, ref], low = Math.min(...all), high = Math.max(...all);
    const span = Math.max(high - low, 1000);
    const zRatio = Math.max(span / horizontal, .3), stretch = zRatio * horizontal / span;
    const node = el('envelope-chart'), viewport = (node.clientWidth || 800) / (node.clientHeight || 500);
    const fillRatio = Math.min(1.9, 1.25 * Math.max(.75, Math.min(viewport, 1.6)) / (1 + zRatio * .6));
    const pad = (high - low) * .08 + 300;
    const axis = (title, range) => ({title: {text: title, font: {size: 11}}, range, tickformat: ',.0f', gridcolor: grid, zerolinecolor: zero, color: tick, backgroundcolor: '#000000', showspikes: false, tickfont: {size: 10}});
    const layout = {...theme, margin: {l: 0, r: 0, t: 0, b: 0}, showlegend: false, uirevision: 'envelope3d:' + JSON.stringify(submitted),
      annotations: [{text: stretch > 1.05 ? `Altitude stretched ×${fmt(stretch, 1)}` : 'Equal scale', xref: 'paper', yref: 'paper', x: .015, y: .985, xanchor: 'left', yanchor: 'top', showarrow: false, font: {size: 10, color: tick}}],
      scene: {xaxis: axis('X · m', win.x), yaxis: axis('Z · m', win.y), zaxis: axis('Altitude · m', [Math.max(0, low - pad), high + pad]),
        aspectmode: 'manual', aspectratio: {x: fillRatio * wx / horizontal, y: fillRatio * wy / horizontal, z: zRatio * fillRatio},
        camera: {eye: {x: -1.15, y: -1.3, z: .8}, up: {x: 0, y: 0, z: 1}, projection: {type: 'orthographic'}}}};
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
  // The searched direction nearest a bearing in one layer (mirrored directions count on both sides).
  function rayNear(li, azimuth) {
    const line = field().lines[li];
    if (!line?.az.length) return null;
    let best = null;
    for (const entry of line.entries) { const d = Math.abs(wrap(entry.az - azimuth)); if (!best || d < best.d) best = {d, entry}; }
    return best && best.d < 1e-6 ? best.entry.ray : null;
  }
  function renderMetrics() {
    const c = submitted, ref = nearestLayer(c);
    el('envelope-metric-front-label').textContent = `Head-on maximum · launcher at ${fmt(c.layers_m[ref])} m`;
    el('envelope-metric-rear-label').textContent = `Tail-chase maximum · launcher at ${fmt(c.layers_m[ref])} m`;
    const shown = az => { const ray = rays.size ? rayNear(ref, az) : null; return !ray ? '—' : ray.max_range_m === null ? 'No hit' : km(ray.max_range_m); };
    el('envelope-metric-front').textContent = shown(0); el('envelope-metric-rear').textContent = shown(180);
    const sims = summary?.simulations ?? jobInfo?.progress?.simulations ?? [...rays.values()].reduce((s, r) => s + r.simulations, 0);
    el('envelope-metric-sims').textContent = rays.size ? `${fmt(rays.size)} · ${fmt(sims)}` : '—';
  }
  function renderTable() {
    const c = submitted, step = baseStep(), every = Math.ceil(30 / step - 1e-9) * step;
    const columns = Array.from({length: Math.floor(180 / every + 1e-9) + 1}, (_, i) => i * every);
    const name = a => a === 0 ? 'Head-on' : a === 180 ? 'Tail' : a === 90 ? 'Beam' : `${fmt(a)}°`;
    el('envelope-table').tHead.innerHTML = `<tr><th>Launcher altitude</th>${columns.map(a => `<th>${name(a)}<small>min – max</small></th>`).join('')}</tr>`;
    el('envelope-table').tBodies[0].innerHTML = c.layers_m.map((alt, li) => `<tr><td>${fmt(alt)} m</td>${columns.map(a => {
      const ray = rays.size ? rayNear(li, a) : null;
      if (!ray) return '<td class="hint">…</td>';
      if (ray.max_range_m === null) return '<td class="hint">no hit</td>';
      return `<td>${ray.intervals[0].min_at_search_limit ? '≤' : ''}${km(ray.min_range_m)} – ${km(ray.max_range_m)}${ray.intervals.length > 1 ? ' *' : ''}</td>`;
    }).join('')}</tr>`).slice().reverse().join('');
  }
  function renderInspector() {
    const box = el('envelope-inspect'), button = el('envelope-open-simulator');
    if (!inspected || !submitted) { el('envelope-inspect-title').textContent = 'Select a point on the envelope'; box.innerHTML = '<p class="muted">Click the envelope outline to see that direction\'s searched distances, outcomes and flight times.</p>'; button.hidden = true; return; }
    const c = submitted, ray = rayNear(inspected.layer, inspected.azimuth);
    el('envelope-inspect-title').textContent = `Launcher at ${fmt(c.layers_m[inspected.layer])} m · ${bearingName(inspected.azimuth)}`;
    if (!ray) { box.innerHTML = '<p class="muted">This direction has not finished yet.</p>'; button.hidden = true; return; }
    const rows = ray.samples.map(s => `<tr><td>${km(s.range_m)}</td><td class="${s.hit ? 'envelope-hit' : 'envelope-miss'}">${s.hit ? 'Hit' : 'Miss'}</td><td>${escapeHtml(outcomeLabel(s.outcome))}</td><td>${Number.isFinite(s.time_s) ? fmt(s.time_s, 2) + ' s' : '—'}</td><td>${Number.isFinite(s.closest_m) ? closestText(s) : '—'}</td></tr>`).join('');
    const summaryText = ray.max_range_m === null ? 'No searched distance produced a hit.' :
      `Hits from ${ray.intervals[0].min_at_search_limit ? 'the closest searched distance, ' : ''}${km(ray.min_range_m)} to ${km(ray.max_range_m)}${ray.intervals.at(-1).max_at_search_limit ? ' (the search limit)' : ''}. Farthest hit flew ${fmt(ray.max_time_s, 1)} s.${ray.intervals.length > 1 ? ` Hits stop and resume ${ray.intervals.length - 1} time(s) in between.` : ''}`;
    box.innerHTML = `<p class="muted">${summaryText} ${ray.simulations} simulations searched ${km(ray.search[0])} – ${km(ray.search[1])}. A hit is a proximity-fuze event. Near a boundary, hits and narrow misses can alternate with very small changes in the launch.</p>
      <div class="envelope-table-wrap"><table class="envelope-samples"><thead><tr><th>Launch distance</th><th>Result</th><th>Ended by</th><th>At</th><th>Closest approach</th></tr></thead><tbody>${rows}</tbody></table></div>`;
    button.hidden = ray.max_range_m === null || typeof window.missileSimulator?.load !== 'function';
  }
  el('envelope-open-simulator').addEventListener('click', () => {
    const c = submitted, ray = inspected && rayNear(inspected.layer, inspected.azimuth);
    if (!ray || ray.max_range_m === null) return;
    const s = states(c, c.layers_m[inspected.layer], inspected.azimuth, ray.intervals.at(-1).last_hit_m);
    window.missileSimulator.load({missile: c.missile, duration: Math.min(180, Math.ceil((ray.max_time_s || 60) + 5)), launcher: s.launcher, target: s.target});
  });
  for (const id of ['envelope-view', 'envelope-layer', 'envelope-cut', 'envelope-show-rays', 'envelope-show-samples'])
    el(id).addEventListener('change', () => render().catch(error => showError(error.message)));
  el('envelope-view-reset').addEventListener('click', () => { const node = el('envelope-chart'); if (node.data) { Plotly.purge(node); render().catch(error => showError(error.message)); } });

  // ---- Export -----------------------------------------------------------------------
  function setExports(enabled) {
    for (const url of downloadUrls) URL.revokeObjectURL(url); downloadUrls = [];
    for (const id of ['envelope-download-json', 'envelope-download-csv']) { el(id).setAttribute('aria-disabled', String(!enabled)); el(id).removeAttribute('href'); }
    if (!enabled) return;
    const ordered = [...rays.values()].sort((a, b) => a.key[0] - b.key[0] || a.azimuth_deg - b.azimuth_deg);
    const data = {format: 'missile-envelope-v2', missile: submitted.missile, missile_name: missileName(submitted.missile), request: submitted,
      config: jobInfo?.config || null, mirrored: mirrored(), complete: jobInfo?.status === 'complete', summary, rays: ordered,
      conventions: {frame: 'Target at the origin flying level along +X; each point is a launcher position at release, heading toward the target',
        azimuth: 'Launcher bearing from the target nose; 0 = ahead of the target (head-on), 180 = behind it (tail chase), positive toward +Z',
        range: 'Horizontal distance in metres from the target at release', layer: 'Launcher altitude (m)',
        hit: 'Native proximity-fuze event against the constant-velocity point target'}};
    const csv = [['launcher_altitude_m', 'azimuth_deg', 'refinement_level', 'min_range_m', 'max_range_m', 'min_at_search_limit', 'max_at_search_limit', 'max_range_flight_time_s', 'hit_intervals_m', 'simulations'].join(',')]
      .concat(ordered.map(r => [r.layer_m, r.azimuth_deg, r.level, r.min_range_m ?? '', r.max_range_m ?? '', r.max_range_m === null ? '' : r.intervals[0].min_at_search_limit,
        r.max_range_m === null ? '' : r.intervals.at(-1).max_at_search_limit, r.max_time_s ?? '', r.intervals.map(i => `${Math.round(i.start_m)}-${Math.round(i.end_m)}`).join(' '), r.simulations].join(','))).join('\n');
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
