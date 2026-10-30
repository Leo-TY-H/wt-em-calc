// Engine ratings (Leo, 2026-10-30): each engine's temperature limits under the game's thermal fatigue rules, for up to
// eight aircraft (GET /api/engine-ratings/<aircraft>, scripts/engine_ratings.py). Laid out like the Engine performance
// page; uses app.js's catalog (state.meta), api() and name helpers. Results load as aircraft or ratings change.
(() => {
  'use strict';
  const el = id => document.getElementById(id);
  const root = el('ratings-workspace');
  if (!root) return;
  const MAX = 8, RATINGS = [30, 10, 5, 1], CAP = 36000;
  const K = 273.1499938964844, K32 = Math.fround(273.15);  // the game's Celsius offset, f(273.15)
  const COLORS = ['#38c9d7', '#ffa66b'];                     // channel 0 (water or head), oil
  // The game's HUD colours by remaining endurance (config/hud.blk): <60 s red blinking (drawn faded), <120 s red,
  // <300 s amber, <900 s pale yellow.
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
    <div class="field contour-settings"><span>Ratings · min</span><div id="ratings-levels" class="contour-levels"></div><div class="contour-add"><input id="ratings-input" type="number" min="0" max="600" step="any" aria-label="New rating in minutes" placeholder="New rating"><button id="ratings-add" type="button">Add</button></div><span id="ratings-status" class="hint" role="status"></span></div>
  </form></aside>
  <main>
    <div id="ratings-error" class="error" role="alert" hidden></div>
    <div id="ratings-results"></div>
  </main>`;

  let entries = [], nextEntry = 1, ratings = [...RATINGS];
  const results = new Map(), failures = new Map();   // results: done or loading (null); failures are retried
  const catalog = () => (typeof state !== 'undefined' && state.meta) ? state.meta.aircraft : null;
  const showError = text => { el('ratings-error').textContent = text; el('ratings-error').hidden = !text; };
  const key = aircraft_id => `${aircraft_id}?ratings=${ratings.join(',')}`;

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
  const curveTime = s => s >= CAP ? '≥ 10:00:00' : time(s);
  function limitCell(limit, channel) {
    if (limit.bound === 'load') return `≤ ${floorC(limit.max_c).toFixed(1)}`;
    return limit.bound === 'boiling' ? `boils ${one(channel.boiling_c)}` : '—';
  }
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
    filter(); renderEntries(); renderRatings(); render();
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

  // ---- Ratings: the EM page's SEP contour chips -----------------------------------------------------------------
  function renderRatings() {
    el('ratings-levels').innerHTML = ratings.map(m => `<button type="button" data-rating-remove="${m}" aria-label="Remove ${m} min rating">${m} ×</button>`).join('');
  }
  el('ratings-levels').addEventListener('click', event => {
    const button = event.target.closest('[data-rating-remove]'); if (!button) return;
    ratings = ratings.filter(m => m !== Number(button.dataset.ratingRemove));
    el('ratings-status').textContent = ''; renderRatings(); load();
  });
  el('ratings-add').addEventListener('click', () => {
    const input = el('ratings-input'), m = Number(input.value);
    if (!input.value.trim() || !Number.isFinite(m) || m <= 0 || m > 600 || ratings.includes(m) || ratings.length >= 8) {
      el('ratings-status').textContent = 'Up to 8 distinct ratings, above 0 and up to 600 min.'; return;
    }
    ratings.push(m); ratings.sort((a, b) => b - a); input.value = ''; el('ratings-status').textContent = '';
    renderRatings(); load();
  });
  el('ratings-input').addEventListener('keydown', event => { if (event.key === 'Enter') { event.preventDefault(); el('ratings-add').click(); } });

  // ---- Loading: one request per aircraft and ratings, kept; a failed one is asked again on the next load --------
  async function load() {
    const wanted = [...new Set(entries.map(e => key(e.aircraft_id)))].filter(k => !results.has(k));
    for (const k of wanted) { results.set(k, null); failures.delete(k); }
    render();
    await Promise.all(wanted.map(async k => {
      try { results.set(k, await api(`/api/engine-ratings/${encodeURIComponent(k.split('?')[0])}?${k.split('?')[1]}`)); }
      catch (error) { results.delete(k); failures.set(k, error.message); }
      render();
    }));
  }

  // ---- Results --------------------------------------------------------------------------------------------------
  function engineBlock(e, chartId) {
    const kind = [FAMILY[e.family] || e.family, ['Inline', 'Radial'].includes(e.family) ? (e.water_cooled ? 'water-cooled' : 'air-cooled') : null].filter(Boolean).join(' · ');
    const port = e.verified === false ? ` <abbr class="ratings-port" title="${PORT}${e.mode_rows_skipped ? ' · Mode rows not loadable' : ''}">P</abbr>` : '';
    if (e.excluded) return `<p class="ratings-excluded">${engineLabel(e.engines)} · ${escapeText(e.family && !e.excluded.startsWith(e.family) ? `${e.family} · ` : '')}${escapeText(e.excluded)}${port}</p>`;
    const ch = e.channels, swatch = i => `<i class="ratings-swatch" style="background:${COLORS[i]}"></i>`;
    const head = `<tr><th></th>${ch.map((c, i) => `<th>${swatch(i)}${c.label} °C</th>`).join('')}</tr>`;
    const rows = [['Continuous', c => c.continuous], ...e.channels[0].ratings.map((r, j) => [`${r.minutes} min`, c => c.ratings[j]])];
    const limits = `<table class="ratings-table"><thead>${head}</thead><tbody>${rows.map(([name, pick]) =>
      `<tr><th>${name}</th>${ch.map(c => `<td>${limitCell(pick(c), c)}</td>`).join('')}</tr>`).join('')}</tbody></table>`;
    const fluids = ch.filter(c => c.thermostat_c != null || c.boiling_c != null).map(c =>
      `<span>${c.label} thermostat ${one(c.thermostat_c)} · boils ${one(c.boiling_c)}</span>`).join('');
    // Recovery: each load counter's fill and empty times, and per channel the hottest temperatures at which it drains
    // at all and at its full rate.
    const below = c => c == null ? '—' : `≤ ${floorC(c).toFixed(1)}`;
    const recovery = e.buckets?.length ? `<table class="ratings-table ratings-recovery"><thead><tr><th></th><th></th>${ch.map((c, i) => `<th colspan="2">${swatch(i)}${c.label} °C</th>`).join('')}</tr>
      <tr><th>Fills in</th><th>Empties in</th>${ch.map(() => '<th>Drains</th><th>Full rate</th>').join('')}</tr></thead><tbody>${
      e.buckets.map(b => `<tr><th>${time(b.fill_s)}</th><td>${time(b.empty_s)}</td>${b.channels.map(t => `<td>${below(t.drains_c)}</td><td>${below(t.full_c)}</td>`).join('')}</tr>`).join('')}</tbody></table>` : '';
    // Automatic radiator: only the channels whose radiator it moves.
    const autoColumns = ch.map((c, i) => i).filter(i => e.automatic?.some(a => a.targets_c[i] != null));
    const auto = autoColumns.length ? `<table class="ratings-table ratings-radiator"><thead><tr><th>Throttle</th>${autoColumns.map(i => `<th>${ch[i].label} target</th>`).join('')}<th>Lasts</th></tr></thead><tbody>${
      e.automatic.map(a => `<tr><th>${a.label}</th>${autoColumns.map(i => `<td>${one(a.targets_c[i])}</td>`).join('')}<td>${time(a.seconds)}${a.boils ? ' · boils' : ''}</td></tr>`).join('')}</tbody></table>` : '';
    return `<div class="ratings-engine"><h3>${engineLabel(e.engines)} · ${kind}${port}</h3>
      <div class="ratings-engine-body"><div class="ratings-tables">${limits}${fluids ? `<p class="ratings-fluids">${fluids}</p>` : ''}${recovery}${auto}</div>
      <div class="ratings-chart" id="${chartId}" aria-label="Time to first damage against temperature"></div></div></div>`;
  }
  // One section per aircraft, rebuilt only when its number, ratings or result change (each holds up to four charts).
  const purge = node => { if (typeof Plotly !== 'undefined') for (const chart of node.querySelectorAll('.js-plotly-plot')) Plotly.purge(chart); };
  function section(entry, i, result, status, charts) {
    const a = catalog()?.[entry.aircraft_id];
    const heading = `<header class="ratings-aircraft-heading"><span class="entry-number">${i + 1}</span><h2>${escapeText(cleanName(a?.name || entry.aircraft_id))}</h2><code>${escapeText(entry.aircraft_id)}</code></header>`;
    if (status !== 'done') return `${heading}<p class="ratings-excluded">${status === 'error' ? 'Not available' : 'Calculating…'}</p>`;
    if (!result.engines.length) return `${heading}<p class="ratings-excluded">No engines</p>`;
    return heading + result.engines.map((e, j) => {
      const id = `ratings-chart-${entry.id}-${j}`; if (!e.excluded) charts.push([id, e]); return engineBlock(e, id);
    }).join('');
  }
  function render() {
    const host = el('ratings-results');
    showError([...new Set(entries.map(e => failures.get(key(e.aircraft_id))).filter(Boolean))].join(' · '));
    if (!entries.length) { purge(host); host.innerHTML = '<div class="empty"><span class="empty-glyph">⌁</span><h2>Add an aircraft</h2></div>'; return; }
    host.querySelector('.empty')?.remove();
    const kept = new Set(entries.map(e => e.id));
    for (const node of [...host.children]) if (!kept.has(node.dataset.entry)) { purge(node); node.remove(); }
    entries.forEach((entry, i) => {
      const k = key(entry.aircraft_id), result = results.get(k);
      const status = result ? 'done' : failures.has(k) && !results.has(k) ? 'error' : 'pending', stamp = `${i}|${k}|${status}`;
      let node = host.querySelector(`[data-entry="${entry.id}"]`);
      // While new ratings load, the shown result stays (dimmed): the page keeps its height and scroll position.
      const keep = node && status === 'pending' && node.dataset.index === String(i) && node.dataset.status !== 'pending';
      node?.classList.toggle('ratings-stale', !!keep);
      if (!keep && node?.dataset.stamp !== stamp) {
        const fresh = document.createElement('section'), charts = [];
        Object.assign(fresh, {className: 'ratings-aircraft', innerHTML: section(entry, i, result, status, charts)});
        Object.assign(fresh.dataset, {entry: entry.id, stamp, index: i, status}); fresh.style.setProperty('--entry-color', entryColors[i]);
        if (node) { purge(node); node.replaceWith(fresh); } else host.append(fresh);
        node = fresh;
        if (typeof Plotly !== 'undefined') for (const [id, e] of charts) chart(el(id), e);
      }
      if (host.children[i] !== node) host.insertBefore(node, host.children[i] || null);
    });
  }

  // Time to first damage (log) against temperature, one panel per channel sharing the time axis; dashed lines at the
  // continuous limits; the HUD's warning colours along the time axis.
  function chart(node, e) {
    const ch = e.channels.filter(c => c.curve.length);
    if (!ch.length) { node.remove(); return; }
    const gap = .1, width = (1 - gap * (ch.length - 1)) / ch.length;
    const lowest = Math.min(60, ...ch.flatMap(c => c.curve.map(p => p[1])));
    const axisStyle = {gridcolor: 'rgba(240,240,250,.09)', zeroline: false, tickfont: {size: 11}, fixedrange: true};
    const layout = {paper_bgcolor: '#000', plot_bgcolor: '#000', font: {family: 'WTSymbols, D-DIN, Bahnschrift, DIN Alternate, Segoe UI, sans-serif', color: '#b9b9c4', size: 12},
      margin: {l: 56, r: 14, t: 10, b: 42}, hovermode: 'closest', showlegend: false, dragmode: false,
      hoverlabel: {bgcolor: '#0c0c0f', bordercolor: 'rgba(240,240,250,.35)', font: {color: '#f4f4fc', size: 12}},
      yaxis: {...axisStyle, type: 'log', range: [Math.log10(lowest / 1.15), Math.log10(CAP * 1.15)],
        tickvals: [60, 300, 600, 1800, 3600, 36000], ticktext: ['1 min', '5 min', '10 min', '30 min', '1 h', '10 h']},
      shapes: HUD.filter(([, hi]) => hi > lowest / 1.15).map(([lo, hi, color, opacity]) => ({type: 'rect', xref: 'paper', xsizemode: 'pixel', xanchor: 0, x0: 0, x1: 5,
        yref: 'y', y0: Math.max(lo, lowest / 1.15), y1: hi, fillcolor: color, opacity, line: {width: 0}, layer: 'above'}))};
    const traces = ch.map((c, n) => {
      const axis = n ? String(n + 1) : '', xs = c.curve.map(p => p[0]), lo = xs[0], hi = xs.at(-1), pad = Math.max(.5, (hi - lo) * .04);
      const color = COLORS[e.channels.indexOf(c)];
      layout['xaxis' + axis] = {...axisStyle, domain: [n * (width + gap), n * (width + gap) + width], range: [lo - pad, hi + pad], nticks: 6,
        title: {text: `${c.label} °C`, standoff: 6, font: {size: 12}}, ...(n && {anchor: 'y'})};
      if (c.continuous.bound === 'load') layout.shapes.push({type: 'line', xref: 'x' + axis, yref: 'paper', x0: c.continuous.max_c, x1: c.continuous.max_c, y0: 0, y1: 1,
        line: {color: 'rgba(240,240,250,.35)', width: 1, dash: 'dash'}});
      return {type: 'scatter', mode: 'lines', x: xs, y: c.curve.map(p => p[1]), xaxis: 'x' + axis, yaxis: 'y', line: {color, width: 2},
        // Hover temperature floored like the table (time does not rise with temperature, so this never overstates it).
        customdata: c.curve.map(p => [floorC(p[0]).toFixed(1), curveTime(p[1])]), hoverlabel: {bordercolor: color},
        hovertemplate: `${c.label} %{customdata[0]} °C: %{customdata[1]}<extra></extra>`};
    });
    Plotly.react(node, traces, layout, {responsive: true, displaylogo: false, displayModeBar: false});
  }

  // The catalog arrives with the EM page's start; the list fills once it is there.
  function whenReady() { if (catalog()) populate(); else setTimeout(whenReady, 300); }
  document.addEventListener('tool-tab-change', event => {
    if (event.detail === 'ratings' && typeof Plotly !== 'undefined')
      requestAnimationFrame(() => { for (const chart of el('ratings-results').querySelectorAll('.js-plotly-plot')) Plotly.Plots.resize(chart); });
  });
  renderRatings(); whenReady();
})();
