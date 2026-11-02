// Engine ratings (Leo, 2026-10-30): each engine's temperatures under the game's thermal fatigue rules, for up to eight
// aircraft (GET /api/engine-ratings/<aircraft>, scripts/engine_ratings.py). Laid out like the Engine performance page;
// uses app.js's catalog (state.meta), api() and name helpers. Results load as aircraft are added.
// Pilot-direct tables (Leo, 2026-11-02): by setting (automatic and open radiators: what each fluid settles at, how long
// it lasts, how long until WEP is back) and by gauge reading (how long each temperature lasts, and how cool and for how
// long to get all of it back), with the time against temperature over the HUD's warning colours.
// Plots (Leo, 2026-11-01): radiator factor and drag coefficient against opening, and the heat index against throttle
// (GET /api/engine-ratings/<aircraft>/heat, loaded after the tables: it runs the engine page's solver, and carries the
// tables by setting).
(() => {
  'use strict';
  const el = id => document.getElementById(id);
  const root = el('ratings-workspace');
  if (!root) return;
  const MAX = 8, CAP = 7200;                                 // chart: up to 2 h
  const K = 273.1499938964844, K32 = Math.fround(273.15);  // the game's Celsius offset, f(273.15)
  const COLORS = ['#38c9d7', '#ffa66b'];                     // channel 0 (water or head), oil
  // The game's HUD colours by remaining endurance (config/hud.blk): <60 s red blinking (drawn faded), <120 s red,
  // <300 s amber, <900 s pale yellow. Held steady, the time left counts down from the time shown.
  const HUD = [[0, 60, '#ee1606', .45], [60, 120, '#ee1606', 1], [120, 300, '#eda918', 1], [300, 900, '#f6e69f', 1]];
  const FAMILY = {TurboProp: 'Turboprop'};
  const PORT = 'Port only: thermal input not checked against the game loader';

  root.innerHTML = `<aside class="sidebar"><form id="ratings-form" novalidate>
    <fieldset id="ratings-picker"><legend class="visually-hidden">Add aircraft</legend>
      <input id="ratings-search" type="search" placeholder="Search aircraft…" autocomplete="off" aria-label="Search aircraft" aria-controls="ratings-list">
      <div id="ratings-list" class="aircraft-list" aria-label="Available aircraft"></div>
      <p id="ratings-search-empty" class="hint" hidden>No aircraft match your search.</p>
    </fieldset>
    <div id="ratings-selected" aria-label="Added aircraft"></div>
  </form></aside>
  <main>
    <div id="ratings-error" class="error" role="alert" hidden></div>
    <div id="ratings-results"></div>
  </main>`;

  let entries = [], nextEntry = 1;
  const results = new Map(), failures = new Map();   // per aircraft: done or loading (null); failures are retried
  const heats = new Map(), heatFailures = new Map();  // the same, per aircraft
  const catalog = () => (typeof state !== 'undefined' && state.meta) ? state.meta.aircraft : null;
  const showError = text => { el('ratings-error').textContent = text; el('ratings-error').hidden = !text; };

  // ---- Formatting -----------------------------------------------------------------------------------------------
  // Limits: the largest one-decimal temperature whose game value (float32 C + f(273.15)) does not exceed the limit.
  function floorC(c) {
    const limit = c + K;
    let d = Math.floor(c * 10 + 1e-3) / 10;
    while (Math.fround(Math.fround(d) + K32) > limit) d = Math.round(d * 10 - 1) / 10;
    return d;
  }
  const one = v => v == null ? '—' : v.toFixed(1);
  function time(s) {
    if (s == null) return 'never';
    const t = Math.floor(s), h = Math.floor(t / 3600), m = Math.floor(t % 3600 / 60), x = String(t % 60).padStart(2, '0');
    return h ? `${h}:${String(m).padStart(2, '0')}:${x}` : `${m}:${x}`;
  }
  const lasts = s => s == null ? '∞' : time(s);
  const wait = s => s == null ? 'never' : time(Math.ceil(s - 1e-9));     // waits round up: never back too early
  const deg = c => floorC(c).toFixed(1);
  // The HUD colour on arrival at a steady temperature lasting s seconds (none from 15 min).
  function hud(s) {
    const band = s == null ? null : HUD.find(([lo, hi]) => s >= lo && s < hi);
    return band ? `<i class="ratings-hud" style="background:${band[2]};opacity:${band[3]}"></i>` : '';
  }
  const lastsCell = s => `${hud(s)}${lasts(s)}`;
  // Engine numbers, 1-based, runs joined: "Engines 1–4", "Engine 2", "Engines 1, 3".
  function engineLabel(list) {
    const runs = [];
    for (const n of list.map(i => i + 1)) { const last = runs.at(-1); if (last && n === last[1] + 1) last[1] = n; else runs.push([n, n]); }
    return `${list.length > 1 ? 'Engines' : 'Engine'} ${runs.map(([a, b]) => a === b ? a : `${a}–${b}`).join(', ')}`;
  }

  // ---- Aircraft list and cards ----------------------------------------------------------------------------------
  function populate() {
    const rows = Object.entries(catalog()).sort(([, a], [, b]) =>
      sortName(a.name).localeCompare(sortName(b.name), undefined, {numeric: true}) || a.name.localeCompare(b.name));
    el('ratings-list').innerHTML = rows.map(([id, a]) => `<div class="aircraft-option" data-search="${escapeText(id + ' ' + a.name)}" data-id="${escapeText(id)}"><span><strong>${escapeText(cleanName(a.name))}</strong><small><code>${escapeText(nameCode(a.name) || id)}</code></small></span><button type="button" data-add="${escapeText(id)}" aria-label="Add ${escapeText(a.name)}">Add</button></div>`).join('');
    filter(); renderEntries(); render();
  }
  function filter() {
    const normalize = text => text.toLowerCase().normalize('NFKD').replace(/[^a-z0-9]/g, '');
    const query = normalize(el('ratings-search').value); let visible = 0;
    for (const row of el('ratings-list').children) {
      row.hidden = !normalize(row.dataset.search).includes(query); if (!row.hidden) visible++;
      row.querySelector('button').disabled = entries.length >= MAX || entries.some(e => e.aircraft_id === row.dataset.id);
    }
    // Only search results are listed (as the other pages): nothing until something is typed.
    el('ratings-list').hidden = !query || !visible;
    el('ratings-search-empty').hidden = !query || visible > 0;
  }
  function renderEntries() {
    el('ratings-selected').innerHTML = entries.map((e, i) => {
      const a = catalog()?.[e.aircraft_id];
      return `<article class="aircraft-entry ratings-entry" data-entry="${e.id}" style="--entry-color:${entryColors[i]}">
        <div class="entry-heading"><span class="entry-number">${i + 1}</span><strong>${escapeText(cleanName(a?.name || e.aircraft_id))}</strong></div>
        <div class="entry-actions"><button type="button" class="entry-icon" data-action="remove" aria-label="Remove entry ${i + 1}" title="Remove">${REMOVE_ICON}</button></div></article>`;
    }).join('');
  }
  el('ratings-list').addEventListener('click', event => {
    const button = event.target.closest('[data-add]'); if (!button || button.disabled || entries.length >= MAX) return;
    entries.push({id: `ratings_${nextEntry++}`, aircraft_id: button.dataset.add});
    renderEntries(); filter(); load();
  });
  el('ratings-selected').addEventListener('click', event => {
    const button = event.target.closest('[data-action=remove]'); if (!button) return;
    entries = entries.filter(e => e.id !== button.closest('[data-entry]').dataset.entry);
    renderEntries(); filter(); render();
  });
  el('ratings-search').addEventListener('input', filter);
  el('ratings-search').addEventListener('keydown', event => { if (event.key === 'Enter') event.preventDefault(); });

  // ---- Loading: one request per aircraft, kept; a failed one is asked again on the next load ------------------
  async function load() {
    const wanted = [...new Set(entries.map(e => e.aircraft_id))].filter(k => !results.has(k));
    for (const k of wanted) { results.set(k, null); failures.delete(k); }
    const hot = [...new Set(entries.map(e => e.aircraft_id))].filter(id => !heats.has(id));
    for (const id of hot) { heats.set(id, null); heatFailures.delete(id); }
    render();
    await Promise.all([...wanted.map(async k => {
      try { results.set(k, await api(`/api/engine-ratings/${encodeURIComponent(k)}`)); }
      catch (error) { results.delete(k); failures.set(k, error.message); }
      render();
    }), ...hot.map(async id => {
      try { heats.set(id, await api(`/api/engine-ratings/${encodeURIComponent(id)}/heat`)); }
      catch (error) { heats.delete(id); heatFailures.set(id, error.message); }
      render();
    })]);
  }
  const heatStatus = id => heats.get(id) ? 'done' : heatFailures.has(id) && !heats.has(id) ? 'error' : 'pending';

  // ---- Results --------------------------------------------------------------------------------------------------
  const swatch = i => `<i class="ratings-swatch" style="background:${COLORS[i]}"></i>`;
  // A band of held temperatures as the gauge reads them (0.1 C, floored like every limit): above one band top, up to
  // the next (open-ended at the last load row).
  function range(above, to) {
    const lo = (Math.round(floorC(above) * 10) + 1) / 10;
    if (to == null) return `≥ ${lo.toFixed(1)}`;
    const hi = floorC(to);
    if (lo > hi + 1e-9) return '—';       // narrower than a gauge step
    return lo < hi ? `${lo.toFixed(1)}–${hi.toFixed(1)}` : hi.toFixed(1);
  }
  function continuousCell(c) {
    if (c.continuous.bound === 'load') return `≤ ${deg(c.continuous.max_c)}`;
    return c.continuous.bound === 'boiling' ? `&lt; ${one(c.boiling_c)}` : 'any';
  }
  // By gauge reading: one row per time to first damage (the band's shortest, at its top), both channels side by side;
  // full recovery: cool to the temperature shown or below for that long after using all of the band's time (the
  // longer of the channels).
  function gaugeTable(e) {
    const ch = e.channels, g = e.gauge || [];
    const times = [...new Set(g.flatMap(x => x ? x.bands.map(b => b.lasts_s) : []))].sort((a, b) => b - a);
    const band = (c, s) => g[c]?.bands.find(b => b.lasts_s === s);
    const back = times.length ? ch.length + 1 : 0;
    const head = `<tr><th colspan="${ch.length}">Gauge °C</th><th rowspan="2">Lasts</th>${back ? `<th colspan="${back}" class="ratings-group">Full recovery</th>` : ''}</tr>
      <tr>${ch.map((c, i) => `<th>${swatch(i)}${c.label}</th>`).join('')}${back ? `${ch.map((c, i) => `<th>${swatch(i)}${c.label} ≤</th>`).join('')}<th>Takes</th>` : ''}</tr>`;
    const rows = [`<tr>${ch.map((c, i) => `<${i ? 'td' : 'th'}>${continuousCell(c)}</${i ? 'td' : 'th'}>`).join('')}<td>∞</td>${back ? `<td colspan="${back}"></td>` : ''}</tr>`,
      ...times.map(s => {
        const bands = ch.map((c, i) => band(i, s)), takes = bands.filter(Boolean).map(b => b.recover_s);
        return `<tr>${bands.map((b, i) => `<${i ? 'td' : 'th'}>${b ? range(b.above_c, b.to_c) : '—'}</${i ? 'td' : 'th'}>`).join('')}<td>${lastsCell(s)}</td>${
          bands.map(b => `<td>${b?.recover_c != null ? deg(b.recover_c) : '—'}</td>`).join('')}<td>${wait(takes.includes(null) ? null : Math.max(...takes))}</td></tr>`;
      })];
    return `<table class="ratings-table ratings-gauge"><thead>${head}</thead><tbody>${rows.join('')}</tbody></table>`;
  }
  // By setting (heat sweep): automatic and open radiators. Without it, the automatic radiator at its RadiatorMode rows
  // (each channel at the top of its target band) with the sweep's status; a note while it loads or where nothing is
  // solved (nothing where the sweep is solved and drawn but gives no table).
  function settingsTables(e, heat, status) {
    const ch = e.channels;
    const where = status === 'done' ? `${heat.altitude_m.toLocaleString('en-US')} m · ${heat.speed_kmh} km/h` : '';
    const solved = solvedGroups(e, heat, status).length > 0;
    const head = (title, cols, hot) => `<tr><th>${title}</th>${cols.map(i => `<th>${swatch(i)}${ch[i].label} °C</th>`).join('')}<th>Lasts</th>${hot ? `<th>${hot} back</th>` : ''}</tr>`;
    const groups = status === 'done' ? heat.engines.filter(g => g.engines.some(i => e.engines.includes(i)) && g.tables) : [];
    if (groups.length) return groups.flatMap(g => g.tables.map(t => {
      const title = {auto: 'Auto radiator', open: 'Radiator open', fixed: 'Throttle'}[t.mode] + (groups.length > 1 ? ` · ${engineLabel(g.engines)}` : '');
      return `<table class="ratings-table ratings-settings"><thead>${head(title, ch.map((c, i) => i), t.hot)}</thead><tbody>${t.rows.map(r =>
        `<tr><th>${r.label}</th>${r.temps_c.map(v => `<td>${deg(v)}</td>`).join('')}<td>${lastsCell(r.lasts_s)}${r.boils ? ' · boils' : ''}</td>${t.hot ? `<td>${r.label === t.hot ? '—' : wait(r.back_s)}</td>` : ''}</tr>`).join('')}</tbody></table>`;
    })).join('') + `<p class="ratings-fluids">${where} TAS</p>`;
    const piston = ['Inline', 'Radial'].includes(e.family);
    const missing = !piston || solved ? '' : `<p class="ratings-fluids">Radiator open · ${status === 'done' ? `not solved at ${where}` : 'not available'}</p>`;
    const cols = ch.map((c, i) => i).filter(i => e.automatic?.some(a => a.held_c[i] != null));
    if (cols.length && status !== 'pending') return `<table class="ratings-table ratings-settings"><thead>${head('Auto radiator', cols)}</thead><tbody>${
      e.automatic.map(a => `<tr><th>${a.label}</th>${cols.map(i => `<td>${deg(a.held_c[i])}</td>`).join('')}<td>${lastsCell(a.seconds)}${a.boils ? ' · boils' : ''}</td></tr>`).join('')}</tbody></table>${missing}`;
    if (!piston || solved) return '';
    return `<p class="ratings-plot-note ratings-settings-note">${status === 'pending' ? 'Calculating…' : status === 'done' ? `Not solved at ${where}` : 'Not available'}</p>`;
  }
  // Heat groups of an engine with at least one solved throttle (the ones the heat chart draws).
  const solvedGroups = (e, heat, status) => status === 'done'
    ? heat.engines.filter(g => g.engines.some(i => e.engines.includes(i)) && g.points.some(p => p?.heat_c.some(v => v != null))) : [];
  function engineBlock(e, id, result, heat, status, charts) {
    const kind = [FAMILY[e.family] || e.family, ['Inline', 'Radial'].includes(e.family) ? (e.water_cooled ? 'water-cooled' : 'air-cooled') : null].filter(Boolean).join(' · ');
    const port = e.verified === false ? ` <abbr class="ratings-port" title="${PORT}${e.mode_rows_skipped ? ' · Mode rows not loadable' : ''}">P</abbr>` : '';
    if (e.excluded) return `<p class="ratings-excluded">${engineLabel(e.engines)} · ${escapeText(e.family && !e.excluded.startsWith(e.family) ? `${e.family} · ` : '')}${escapeText(e.excluded)}${port}</p>`;
    const fluids = e.channels.filter(c => c.thermostat_c != null || c.boiling_c != null).map(c =>
      `<span>${c.label} thermostat ${one(c.thermostat_c)} · boils ${one(c.boiling_c)}</span>`).join('');
    const settings = settingsTables(e, heat, status);
    return `<div class="ratings-engine"><h3>${engineLabel(e.engines)} · ${kind}${port}</h3>
      <div class="ratings-engine-body">${settings ? `<div class="ratings-tables">${settings}</div>` : ''}
      <div class="ratings-tables">${gaugeTable(e)}${fluids ? `<p class="ratings-fluids">${fluids}</p>` : ''}</div></div>${plotsRow(e, id, result, heat, status, charts)}</div>`;
  }
  // The plots row: time against held temperature; heat index against throttle (piston engines); radiator factor and
  // drag against opening (controllable radiators).
  function plotsRow(e, id, result, heat, status, charts) {
    const cells = [];
    if (e.channels.some(c => c.curve.length)) {
      cells.push(`<div class="ratings-plot" id="${id}-lasts" aria-label="Time to first damage against temperature"></div>`);
      charts.push(() => lastsChart(el(`${id}-lasts`), e));
    }
    // Groups with no solved throttle (e.g. over-rev at every throttle) draw no chart: the tables' note says so.
    const groups = solvedGroups(e, heat, status);
    if (groups.length) {
      cells.push(`<div class="ratings-plot" id="${id}-heat" aria-label="Heat index against throttle"></div>`);
      charts.push(() => heatChart(el(`${id}-heat`), e, heat, groups));
    }
    if (e.radiator) {
      cells.push(`<div class="ratings-plot" id="${id}-factor" aria-label="Radiator factor against opening"></div>`, `<div class="ratings-plot" id="${id}-drag" aria-label="Drag coefficient against opening"></div>`);
      charts.push(() => factorChart(el(`${id}-factor`), e), () => dragChart(el(`${id}-drag`), e, result.radiator_cd));
    }
    return cells.length ? `<div class="ratings-plots">${cells.join('')}</div>` : '';
  }
  // One section per aircraft, rebuilt only when its number or results change (each holds up to four charts per engine).
  const purge = node => { if (typeof Plotly !== 'undefined') for (const chart of node.querySelectorAll('.js-plotly-plot')) Plotly.purge(chart); };
  function section(entry, i, result, status, charts) {
    const heat = heats.get(entry.aircraft_id), hot = heatStatus(entry.aircraft_id);
    const a = catalog()?.[entry.aircraft_id];
    const heading = `<header class="ratings-aircraft-heading"><span class="entry-number">${i + 1}</span><h2>${escapeText(cleanName(a?.name || entry.aircraft_id))}</h2><code>${escapeText(entry.aircraft_id)}</code></header>`;
    if (status !== 'done') return `${heading}<p class="ratings-excluded">${status === 'error' ? 'Not available' : 'Calculating…'}</p>`;
    if (!result.engines.length) return `${heading}<p class="ratings-excluded">No engines</p>`;
    return heading + result.engines.map((e, j) => engineBlock(e, `ratings-chart-${entry.id}-${j}`, result, heat, hot, charts)).join('');
  }
  function render() {
    const host = el('ratings-results');
    showError([...new Set(entries.map(e => failures.get(e.aircraft_id)).filter(Boolean))].join(' · '));
    if (!entries.length) { purge(host); host.innerHTML = '<div class="empty"><span class="empty-glyph">⌁</span><h2>Add an aircraft</h2></div>'; return; }
    host.querySelector('.empty')?.remove();
    const kept = new Set(entries.map(e => e.id));
    for (const node of [...host.children]) if (!kept.has(node.dataset.entry)) { purge(node); node.remove(); }
    entries.forEach((entry, i) => {
      const k = entry.aircraft_id, result = results.get(k);
      const status = result ? 'done' : failures.has(k) && !results.has(k) ? 'error' : 'pending', stamp = `${i}|${k}|${status}|${heatStatus(k)}`;
      let node = host.querySelector(`[data-entry="${entry.id}"]`);
      if (node?.dataset.stamp !== stamp) {
        const fresh = document.createElement('section'), charts = [];
        Object.assign(fresh, {className: 'ratings-aircraft', innerHTML: section(entry, i, result, status, charts)});
        Object.assign(fresh.dataset, {entry: entry.id, stamp}); fresh.style.setProperty('--entry-color', entryColors[i]);
        if (node) { purge(node); node.replaceWith(fresh); } else host.append(fresh);
        node = fresh;
        if (typeof Plotly !== 'undefined') for (const draw of charts) draw();
      }
      if (host.children[i] !== node) host.insertBefore(node, host.children[i] || null);
    });
  }

  // ---- Charts ---------------------------------------------------------------------------------------------------
  const axisStyle = {gridcolor: 'rgba(240,240,250,.09)', zeroline: false, tickfont: {size: 11}, fixedrange: true};
  const CONFIG = {responsive: true, displaylogo: false, displayModeBar: false};
  const frame = () => ({paper_bgcolor: '#000', plot_bgcolor: '#000', font: {family: 'WTSymbols, D-DIN, Bahnschrift, DIN Alternate, Segoe UI, sans-serif', color: '#b9b9c4', size: 12},
    margin: {l: 56, r: 14, t: 10, b: 42}, hovermode: 'closest', showlegend: false, dragmode: false,
    hoverlabel: {bgcolor: '#0c0c0f', bordercolor: 'rgba(240,240,250,.35)', font: {color: '#f4f4fc', size: 12}}});
  const title = text => ({text, standoff: 6, font: {size: 12}});
  // How long each held temperature lasts (log time, up to 2 h), both channels on one temperature axis, over the HUD's
  // warning colours (the colour on arrival; held steady, the point falls through them at one second per second);
  // dashed at the continuous limits.
  function lastsChart(node, e) {
    const ch = e.channels.map((c, j) => [c, j]).filter(([c]) => c.curve.length);
    const all = ch.flatMap(([c]) => c.curve), xs = all.map(p => p[0]);
    const bottom = Math.min(60, ...all.map(p => p[1])) / 1.25, lo = Math.min(...xs), hi = Math.max(...xs), pad = Math.max(1, (hi - lo) * .04);
    const shapes = HUD.map(([a, b, color, opacity]) => ({type: 'rect', xref: 'paper', x0: 0, x1: 1, yref: 'y', y0: Math.max(a, bottom), y1: b,
      fillcolor: color, opacity: opacity * .22, line: {width: 0}, layer: 'below'}));
    const annotations = [];
    for (const [c, j] of ch) if (c.continuous.bound === 'load') {
      shapes.push({type: 'line', xref: 'x', yref: 'paper', x0: c.continuous.max_c, x1: c.continuous.max_c, y0: 0, y1: 1, line: {color: COLORS[j], width: 1, dash: 'dash'}, opacity: .55});
      annotations.push({x: c.continuous.max_c, y: 1, xref: 'x', yref: 'paper', xanchor: 'right', yanchor: 'top', showarrow: false, text: '∞ ', font: {color: COLORS[j], size: 12}});
    }
    const traces = ch.map(([c, j]) => ({type: 'scatter', mode: 'lines', x: c.curve.map(p => p[0]), y: c.curve.map(p => p[1]), line: {color: COLORS[j], width: 2},
      // Hover temperature floored like the tables (time does not rise with temperature, so this never overstates it).
      customdata: c.curve.map(p => [deg(p[0]), p[1] >= 36000 ? '≥ 10:00:00' : time(p[1])]), hoverlabel: {bordercolor: COLORS[j]},
      hovertemplate: `${c.label} %{customdata[0]} °C · %{customdata[1]}<extra></extra>`}));
    Plotly.react(node, traces, {...frame(), shapes, annotations, margin: {l: 56, r: 14, t: 10, b: 42},
      xaxis: {...axisStyle, range: [lo - pad, hi + pad], nticks: 8, title: title('°C')},
      yaxis: {...axisStyle, type: 'log', range: [Math.log10(bottom), Math.log10(CAP * 1.15)], title: title('Lasts'),
        tickvals: [60, 120, 300, 900, 1800, 3600, 7200], ticktext: ['1 min', '2 min', '5 min', '15 min', '30 min', '1 h', '2 h']}}, CONFIG);
  }

  // Against radiator opening: one line per controllable radiator (a common radiator moves both).
  function openingChart(node, e, values, yTitle, format) {
    const x = e.radiator.opening_percent;
    const traces = e.radiator.channels.map((c, n) => ({type: 'scatter', mode: 'lines', x, y: values(c, n), line: {color: COLORS[c], width: 2},
      hovertemplate: `${e.channels[c].label} %{y:${format}}<extra></extra>`}));
    Plotly.react(node, traces, {...frame(), hovermode: 'x unified', margin: {l: 64, r: 20, t: 10, b: 42},
      xaxis: {...axisStyle, range: [-1, 101], dtick: 25, hoverformat: '.0f', ticksuffix: '%', title: title('Radiator open')},
      yaxis: {...axisStyle, title: title(yTitle)}}, CONFIG);
  }
  const factorChart = (node, e) => openingChart(node, e, (c, n) => e.radiator.factor[n], 'Radiator factor', '.3f');
  // The solver adds Cd x opening (wing_model.drag_terms).
  const dragChart = (node, e, cd) => openingChart(node, e, c => e.radiator.opening_percent.map(x => cd[c] == null ? null : cd[c] * x / 100), 'Drag coefficient', '.5f');

  // Heat index against throttle, per channel; dashed lines at the continuous limits. Engines of this block with
  // different curves get one line each (dotted after the first). Unsolved throttles leave gaps; a solved point with no
  // solved neighbour is a dot.
  function heatChart(node, e, heat, groups) {
    const x = heat.throttle_percent, shapes = [], traces = [];
    groups.forEach((g, n) => e.channels.forEach((c, j) => {
      const label = groups.length > 1 ? `${c.label} (${engineLabel(g.engines)})` : c.label;
      const y = g.points.map(p => p?.heat_c[j] ?? null), lone = y.map((v, k) => v != null && y[k - 1] == null && y[k + 1] == null);
      traces.push({type: 'scatter', mode: 'lines+markers', x, y, line: {color: COLORS[j], width: 2, dash: n ? 'dot' : 'solid'},
        marker: {color: COLORS[j], size: lone.map(l => l ? 5 : 0)},
        // Temperatures floored as in the tables; the time from the engine held there from fresh.
        customdata: g.points.map(p => p?.heat_c[j] == null ? null : [Math.round(p.rpm), p.manifold_ata.toFixed(2), lasts(p.lasts_s?.[j]), deg(p.heat_c[j])]),
        hovertemplate: `${label} %{customdata[3]} °C · %{customdata[2]}${j ? '' : '<br>%{customdata[0]} rpm · %{customdata[1]} ata'}<extra></extra>`});
    }));
    e.channels.forEach((c, j) => { if (c.continuous.bound === 'load') shapes.push({type: 'line', xref: 'paper', x0: 0, x1: 1, yref: 'y', y0: c.continuous.max_c, y1: c.continuous.max_c,
      line: {color: COLORS[j], width: 1, dash: 'dash'}, opacity: .55}); });
    const where = `${heat.altitude_m.toLocaleString('en-US')} m · ${heat.speed_kmh} km/h TAS`;
    Plotly.react(node, traces, {...frame(), hovermode: 'x unified', shapes, margin: {l: 56, r: 20, t: 10, b: 42},
      xaxis: {...axisStyle, range: [-1, x.at(-1) + 1], hoverformat: '.0f', ticksuffix: '%', title: title(`Throttle % · ${where}`),
        tickvals: [0, 20, 40, 60, 80, 100, ...(x.at(-1) > 100 ? [x.at(-1)] : [])], ticktext: ['0', '20', '40', '60', '80', '100', 'WEP']},
      yaxis: {...axisStyle, title: title('Heat index °C')}}, CONFIG);
  }

  // The catalog arrives with the EM page's start; the list fills once it is there.
  function whenReady() { if (catalog()) populate(); else setTimeout(whenReady, 300); }
  document.addEventListener('tool-tab-change', event => {
    if (event.detail === 'ratings' && typeof Plotly !== 'undefined')
      requestAnimationFrame(() => { for (const chart of el('ratings-results').querySelectorAll('.js-plotly-plot')) Plotly.Plots.resize(chart); });
  });
  whenReady();
})();
