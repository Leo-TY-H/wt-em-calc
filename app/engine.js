// Engine performance (Leo, 2026-10-28): shaft power, thrust, propeller efficiency, RPM, blade pitch and manifold
// pressure over altitude or true airspeed, at each aircraft's throttle setting, for up to eight aircraft. Straight flight with
// the nose on the flight path, fully upgraded (GET /api/engine/<aircraft>, scripts/engine_performance.py). Laid out
// like the EM page; uses app.js's catalog (state.meta), api() and name helpers.
(() => {
  'use strict';
  const el = id => document.getElementById(id);
  const root = el('engine-workspace');
  if (!root) return;
  const MAX = 8, PROPS = ['piston', 'turboprop', 'mixed'];
  const defaults = {altitude: 1000, speed: 300, axis: 'altitude', quantity: 'power'};
  // y: the plotted value of a sweep row (engine or propeller 1 where there are several; the details list them all).
  const QUANTITIES = {
    power: {label: 'Power', title: 'Shaft power (hp)', y: r => r.power_hp, digits: 0, unit: ' hp', props: true},
    thrust: {label: 'Thrust', title: 'Thrust (kgf)', y: r => r.thrust_kgf, digits: 0, unit: ' kgf'},
    efficiency: {label: 'Efficiency', title: 'Propeller efficiency (%)', y: r => r.efficiency, digits: 1, unit: '%', props: true},
    rpm: {label: 'RPM', title: 'Engine RPM', y: r => r.rpm?.[0], digits: 0, unit: ' rpm'},
    pitch: {label: 'Blade pitch', title: 'Blade angle (°)', y: r => r.pitch_deg?.[0], digits: 1, unit: '°', props: true},
    manifold: {label: 'Manifold', title: 'Manifold pressure (ata)', y: r => r.manifold_ata?.[0], digits: 2, unit: ' ata', piston: true}};
  const AXES = {altitude: {label: 'Altitude', x: r => r.altitude_m, title: 'Altitude (m)', unit: ' m'},
    speed: {label: 'Speed', x: r => r.speed_kmh, title: 'True airspeed (km/h)', unit: ' km/h'}};
  const GOVERNOR = {'target RPM': 'Holding RPM', 'minimum pitch': 'Fine pitch stop', 'maximum pitch': 'Coarse pitch stop',
    'fixed pitch': 'Fixed pitch', 'manual blade pitch': 'Manual blade angle'};
  const field = (id, label, unit, value, min, max) => `<label class="field" id="${id}-field">${label}<div class="input-unit"><input id="${id}" type="number" min="${min}" max="${max}" step="any" value="${value}" required aria-label="${label}"><span>${unit}</span></div></label>`;
  const radios = (name, items, checked, label) => `<div class="mode-switch" role="radiogroup" aria-label="${label}">${Object.entries(items).map(([k, q]) =>
    `<label><input type="radio" name="${name}" value="${k}"${k === checked ? ' checked' : ''}><span>${q.label}</span></label>`).join('')}</div>`;

  root.innerHTML = `<aside class="sidebar"><form id="engine-form" novalidate>
    <fieldset id="engine-picker"><legend class="visually-hidden">Add aircraft</legend>
      <input id="engine-search" type="search" placeholder="Search aircraft…" autocomplete="off" aria-label="Search aircraft" aria-controls="engine-list">
      <div id="engine-list" class="aircraft-list" aria-label="Available aircraft"></div>
      <p id="engine-search-empty" class="hint" hidden>No aircraft match your search.</p>
    </fieldset>
    <div id="engine-selected" aria-label="Added aircraft"></div>
    <div class="field">X axis${radios('engine-axis', AXES, defaults.axis, 'X axis')}</div>
    ${field('engine-speed', 'True airspeed', 'km/h', defaults.speed, 5, 1600)}
    ${field('engine-altitude', 'Altitude', 'm', defaults.altitude, 0, 18000)}
    <div class="sidebar-actions"><p id="engine-ready" class="hint" hidden></p>
      <div class="action-row"><button id="engine-run" class="primary-button" type="submit" disabled>Calculate</button><button id="engine-reset" class="secondary-button reset-button" type="button">Reset</button></div></div>
  </form></aside>
  <main>
    <div id="engine-error" class="error" role="alert" hidden></div>
    <section class="chart-card"><div class="chart-toolbar">${radios('engine-quantity', QUANTITIES, defaults.quantity, 'Plotted quantity')}</div>
      <div id="engine-chart" aria-label="Engine performance"><div class="empty"><span class="empty-glyph">⌁</span><h2>Add an aircraft and press Calculate</h2></div></div></section>
    <section id="engine-inspector" class="inspector" hidden><div class="inspector-heading"><h2 id="engine-point-title"></h2></div><div id="engine-point"></div></section>
  </main>
  <dialog id="engine-dialog" class="entry-dialog" aria-labelledby="engine-dialog-title"><header class="entry-dialog-header"><div><span class="eyebrow">AIRCRAFT SETTINGS</span><h2 id="engine-dialog-title"></h2></div><button type="button" id="engine-dialog-close" class="text-button" aria-label="Close settings">✕</button></header>
    <div class="entry-dialog-body"><label class="field">Throttle <output id="engine-throttle-label">110%</output><input id="engine-throttle" type="range" min="0" max="110" step="1" value="110"></label>
      <div id="engine-pitch-field"><label class="check-row"><input id="engine-pitch-auto" type="checkbox" checked> Automatic propeller control</label>
      <label class="field"><span id="engine-pitch-name">Propeller control</span> <output id="engine-pitch-label">Automatic</output><input id="engine-pitch" type="range" min="0" max="100" step="1" value="50"></label></div></div></dialog>`;

  let entries = [], nextEntry = 1, results = new Map(), running = false, shown = null, editing = null;
  const catalog = () => (typeof state !== 'undefined' && state.meta) ? state.meta.aircraft : null;
  const showError = text => { el('engine-error').textContent = text; el('engine-error').hidden = !text; };
  const value = id => Number(el(id).value);
  const checked = name => root.querySelector(`input[name=${name}]:checked`).value;
  const propeller = a => PROPS.includes(a?.propulsion);
  const pitchable = a => propeller(a) && !!a?.manual_controls?.propeller_pitch;
  const aircraftOf = entry => catalog()?.[entry.aircraft_id];
  const key = (entry, altitude, speed) => `${entry.aircraft_id}|${altitude}|${speed}|${entry.throttle}|${entry.pitch}`;

  // ---- Aircraft list and cards ----------------------------------------------------------
  function populate() {
    const aircraft = catalog();
    const symbol = a => Number(cleanName(a.name) !== sortName(a.name));
    const rows = Object.entries(aircraft).filter(([, a]) => a.propulsion !== 'rocket').sort(([, a], [, b]) => Number(b.supported) - Number(a.supported) ||
      sortName(a.name).localeCompare(sortName(b.name), undefined, {numeric: true}) || symbol(a) - symbol(b) || a.name.localeCompare(b.name));
    el('engine-list').innerHTML = rows.map(([id, a]) => `<div class="aircraft-option" data-search="${escapeText(id + ' ' + a.name)}" data-supported="${!!a.supported}" title="${escapeText(a.reason || '')}"><span><strong>${escapeText(cleanName(a.name))}</strong><small><code>${escapeText(nameCode(a.name) || id)}</code></small></span><button type="button" data-add="${escapeText(id)}" aria-label="Add ${escapeText(a.name)}"${a.supported ? '' : ' disabled'}>Add</button></div>`).join('');
    filter(); renderEntries(); updateForm();
  }
  function filter() {
    const normalize = text => text.toLowerCase().normalize('NFKD').replace(/[^a-z0-9]/g, '');
    const query = normalize(el('engine-search').value); let visible = 0;
    for (const row of el('engine-list').children) {
      row.hidden = !normalize(row.dataset.search).includes(query); if (!row.hidden) visible++;
      row.querySelector('button').disabled = row.dataset.supported !== 'true' || entries.length >= MAX || running;
    }
    el('engine-search-empty').hidden = visible > 0;
  }
  // Cards as on the EM page (Leo, 2026-10-28): edit, duplicate and remove icons; edit opens the settings popup, which
  // holds only what changes engine output: the throttle (as the EM page's slider) and the propeller control.
  function renderEntries() {
    el('engine-selected').innerHTML = entries.map((e, i) => {
      const a = aircraftOf(e), name = a?.name || e.aircraft_id, off = running ? ' disabled' : '';
      return `<article class="aircraft-entry ${e.id === editing ? 'active' : ''}" data-entry="${e.id}" style="--entry-color:${entryColors[i]}">
        <div class="entry-heading"><span class="entry-number">${i + 1}</span><strong>${escapeText(cleanName(name))}</strong><code>${escapeText(nameCode(name) || e.aircraft_id)}</code></div>
        <div class="entry-actions"><button type="button" class="entry-edit entry-icon" data-action="configure" aria-haspopup="dialog" aria-label="Edit settings of entry ${i + 1}" title="Edit settings"${off}>${EDIT_ICON}</button>
        <button type="button" class="entry-icon" data-action="duplicate" aria-label="Duplicate entry ${i + 1}" title="Duplicate"${entries.length >= MAX || running ? ' disabled' : ''}>${COPY_ICON}</button>
        <button type="button" class="entry-icon" data-action="remove" aria-label="Remove entry ${i + 1}" title="Remove"${off}>${REMOVE_ICON}</button></div></article>`;
    }).join('');
  }
  const addEntry = (aircraft_id, throttle = '110', pitch = 'auto') => {
    if (entries.length >= MAX) return;
    entries.push({id: `engine_${nextEntry++}`, aircraft_id, throttle, pitch}); renderEntries(); filter(); updateForm();
  };
  el('engine-list').addEventListener('click', event => {
    const button = event.target.closest('[data-add]'); if (button && !button.disabled) addEntry(button.dataset.add);
  });
  el('engine-selected').addEventListener('click', event => {
    const button = event.target.closest('[data-action]'); if (!button || button.disabled || running) return;
    const index = entries.findIndex(e => e.id === button.closest('[data-entry]').dataset.entry), entry = entries[index];
    if (button.dataset.action === 'duplicate') return addEntry(entry.aircraft_id, entry.throttle, entry.pitch);
    if (button.dataset.action === 'remove') { entries.splice(index, 1); renderEntries(); filter(); updateForm(); render(); return; }
    editing = entry.id; showSettings(); renderEntries();
    el('engine-dialog-title').textContent = `${index + 1} · ${cleanName(aircraftOf(entry)?.name || entry.aircraft_id)}`;
    el('engine-dialog').style.setProperty('--entry-color', entryColors[index]);
    el('engine-dialog').showModal();
  });

  // ---- Settings popup: applies as it changes; ✕, Escape or a click outside puts it away ------
  function showSettings() {
    const entry = entries.find(e => e.id === editing), kind = aircraftOf(entry)?.manual_controls?.pitch_kind;
    el('engine-throttle').value = entry.throttle;
    el('engine-pitch-field').hidden = !pitchable(aircraftOf(entry));
    el('engine-pitch-auto').checked = entry.pitch === 'auto';
    if (entry.pitch !== 'auto') el('engine-pitch').value = entry.pitch;
    el('engine-pitch-name').textContent = kind === 'RPM governor' ? 'Propeller RPM setting' : kind === 'Blade angle' ? 'Propeller blade-angle setting' : 'Propeller control';
    syncSettings();
  }
  function syncSettings() {
    const auto = el('engine-pitch-auto').checked;
    el('engine-throttle-label').textContent = el('engine-throttle').value + '%';
    el('engine-pitch').disabled = auto; el('engine-pitch-label').textContent = auto ? 'Automatic' : el('engine-pitch').value + '%';
  }
  for (const id of ['engine-throttle', 'engine-pitch-auto', 'engine-pitch']) el(id).addEventListener('input', () => {
    syncSettings();
    const entry = entries.find(e => e.id === editing); if (!entry) return;
    entry.throttle = el('engine-throttle').value;
    if (pitchable(aircraftOf(entry))) entry.pitch = el('engine-pitch-auto').checked ? 'auto' : el('engine-pitch').value;
  });
  el('engine-dialog-close').addEventListener('click', () => el('engine-dialog').close());
  el('engine-dialog').addEventListener('click', event => { if (event.target === el('engine-dialog')) el('engine-dialog').close(); });
  el('engine-dialog').addEventListener('close', () => { editing = null; renderEntries(); });
  el('engine-search').addEventListener('input', filter);
  el('engine-search').addEventListener('keydown', event => { if (event.key === 'Enter') event.preventDefault(); });

  // ---- Form -----------------------------------------------------------------------------
  function axisControls() {
    // Over altitude the sweep holds one speed; over speed, one altitude.
    const axis = checked('engine-axis');
    el('engine-speed-field').hidden = axis !== 'altitude'; el('engine-altitude-field').hidden = axis !== 'speed';
  }
  function updateForm() {
    const invalid = ['engine-speed', 'engine-altitude'].map(el).find(input => !input.checkValidity());
    el('engine-ready').textContent = invalid && entries.length ? `${invalid.getAttribute('aria-label')}: ${invalid.validationMessage}` : '';
    el('engine-ready').hidden = !el('engine-ready').textContent;
    el('engine-run').disabled = running || !catalog() || !entries.length || !!invalid;
  }
  root.querySelectorAll('input[name=engine-axis]').forEach(input => input.addEventListener('change', () => { axisControls(); render(); }));
  root.querySelectorAll('input[name=engine-quantity]').forEach(input => input.addEventListener('change', render));
  el('engine-form').addEventListener('input', event => { if (event.target.type === 'number') updateForm(); });
  el('engine-reset').addEventListener('click', () => {
    if (running) return;
    el('engine-speed').value = defaults.speed; el('engine-altitude').value = defaults.altitude;
    root.querySelector(`input[name=engine-axis][value=${defaults.axis}]`).checked = true;
    root.querySelector(`input[name=engine-quantity][value=${defaults.quantity}]`).checked = true;
    for (const e of entries) { e.throttle = '110'; e.pitch = 'auto'; }
    el('engine-search').value = ''; axisControls(); renderEntries(); filter(); updateForm(); render();
  });

  // ---- Calculation: one request per entry, the button filling as they finish --------------
  const RUN_LABEL = el('engine-run').innerHTML;
  function busy(on) {
    running = on; const run = el('engine-run');
    run.innerHTML = on ? 'Calculating…' : RUN_LABEL; run.classList.toggle('calculating', on); run.style.setProperty('--progress', '0%');
    el('engine-reset').disabled = on; renderEntries(); filter(); updateForm();
  }
  el('engine-form').addEventListener('submit', async event => {
    event.preventDefault(); if (running || !entries.length) return;
    const altitude = value('engine-altitude'), speed = value('engine-speed'), wanted = [...entries];
    showError(''); busy(true);
    try {
      for (const [i, entry] of wanted.entries()) {
        const id = key(entry, altitude, speed);
        if (!results.has(id)) results.set(id, await api(`/api/engine/${encodeURIComponent(entry.aircraft_id)}?altitude_m=${altitude}&speed_kmh=${speed}&throttle=${entry.throttle}&pitch=${entry.pitch}`));
        el('engine-run').style.setProperty('--progress', `${(i + 1) / wanted.length * 100}%`);
      }
      shown = {altitude, speed, entries: wanted.map(e => ({...e}))};
      el('engine-inspector').hidden = true; render();
    } catch (error) { showError(error.message); }
    finally { busy(false); }
  });

  // ---- Plot -------------------------------------------------------------------------------
  // The calculated entries still on the page, each with its result and colour.
  function displayed() {
    if (!shown) return [];
    return shown.entries.map(e => {
      const index = entries.findIndex(x => x.id === e.id);
      return index < 0 ? null : {entry: e, index, color: entryColors[index], result: results.get(key(e, shown.altitude, shown.speed))};
    }).filter(d => d?.result);
  }
  const available = (q, list) => list.some(d => (!QUANTITIES[q].props || propeller(aircraftOf(d.entry))) &&
    (!QUANTITIES[q].piston || d.result.stages.length) && d.result.modes.some(m => m.altitude.some(r => Number.isFinite(QUANTITIES[q].y(r)))));
  // Legend: aircraft · number, then the settings that differ from the defaults (110% throttle, automatic pitch).
  const entryName = d => `${cleanName(d.result.name)} · ${d.index + 1}${d.entry.throttle !== '110' ? ` · ${d.entry.throttle}%` : ''}${d.entry.pitch !== 'auto' ? ` · ${d.entry.pitch}% pitch` : ''}`;
  function render() {
    const list = displayed(), chart = el('engine-chart');
    for (const input of root.querySelectorAll('input[name=engine-quantity]')) input.disabled = !!list.length && !available(input.value, list);
    if (!list.length) {
      if (chart.classList.contains('js-plotly-plot')) Plotly.purge(chart);
      chart.innerHTML = '<div class="empty"><span class="empty-glyph">⌁</span><h2>Add an aircraft and press Calculate</h2></div>';
      el('engine-inspector').hidden = true; return;
    }
    let q = checked('engine-quantity');
    if (!available(q, list)) { q = 'thrust'; root.querySelector('input[name=engine-quantity][value=thrust]').checked = true; }
    const axis = checked('engine-axis'), quantity = QUANTITIES[q], x = AXES[axis], traces = [];
    for (const d of list) {
      const modes = d.result.modes;
      for (const mode of modes) {
        const rows = mode[axis], y = rows.map(r => r.solved ? quantity.y(r) ?? null : null);
        // An entry without this quantity (a jet's power) draws nothing and takes no legend row.
        if (!y.some(Number.isFinite)) continue;
        traces.push({type: 'scatter', mode: 'lines', x: rows.map(x.x), y,
          name: entryName(d), meta: {entry: d.entry.id, mode: mode.key}, line: {color: d.color, width: 2.4}, connectgaps: false,
          hoverlabel: {bordercolor: d.color}, customdata: rows.map(() => mode.label),
          hovertemplate: `%{customdata}<br>%{y:,.${quantity.digits}f}${quantity.unit}<br>%{x:,.0f}${x.unit}<extra></extra>`});
        // Supercharger gear changes over altitude: a mark where each new gear starts.
        if (axis === 'altitude' && d.result.stages.some(n => n > 1)) {
          const marks = rows.filter((r, i) => r.solved && i && rows[i - 1].solved && String(r.gear) !== String(rows[i - 1].gear));
          if (marks.length) traces.push({type: 'scatter', mode: 'markers', x: marks.map(x.x), y: marks.map(quantity.y), showlegend: false,
            meta: {entry: d.entry.id, mode: mode.key}, marker: {color: d.color, size: 9, symbol: 'diamond-open', line: {width: 2}},
            hoverlabel: {bordercolor: d.color}, customdata: marks.map(r => `${mode.label} · gear ${r.gear.join(' / ')}`),
            hovertemplate: `%{customdata}<br>%{x:,.0f}${x.unit}<extra></extra>`});
        }
      }
      if (q === 'rpm') traces.push({type: 'scatter', mode: 'lines', x: [Math.min(...d.result.modes[0][axis].map(x.x)), Math.max(...d.result.modes[0][axis].map(x.x))],
        y: [d.result.rpm_limit[0], d.result.rpm_limit[0]], showlegend: false, hoverlabel: {bordercolor: d.color},
        line: {color: rgba(d.color, .5), width: 1.2, dash: 'dot'}, hovertemplate: `RPM limit ${fmt(d.result.rpm_limit[0], 0)}<extra></extra>`});
    }
    const fixed = axis === 'altitude' ? `${fmt(shown.speed, 0)} km/h TAS` : `${fmt(shown.altitude, 0)} m`;
    const axisStyle = {gridcolor: 'rgba(240,240,250,.09)', zeroline: false, tickfont: {size: 12}, separatethousands: true};
    const layout = {paper_bgcolor: '#000', plot_bgcolor: '#000', font: {family: 'WTSymbols, D-DIN, Bahnschrift, DIN Alternate, Segoe UI, sans-serif', color: '#b9b9c4', size: 12},
      margin: {l: 64, r: 22, t: Math.max(40, Math.ceil(traces.filter(t => t.showlegend !== false).length / 2) * 22), b: 54}, hovermode: 'closest',
      hoverlabel: {bgcolor: '#0c0c0f', bordercolor: 'rgba(240,240,250,.35)', font: {color: '#f4f4fc', size: 12}},
      xaxis: {...axisStyle, title: {text: `${x.title} · ${fixed}`, standoff: 8, font: {size: 13}}, rangemode: 'tozero'},
      yaxis: {...axisStyle, title: {text: quantity.title, standoff: 8, font: {size: 13}}, rangemode: q === 'efficiency' || q === 'pitch' ? 'normal' : 'tozero'},
      legend: {orientation: 'h', x: 0, y: 1.02, yanchor: 'bottom', font: {size: 12}, bgcolor: 'rgba(0,0,0,0)', itemdoubleclick: false},
      uirevision: `${axis}-${q}`, dragmode: 'pan', modebar: {bgcolor: 'rgba(0,0,0,0)', color: '#8a8a96', activecolor: '#f0f0fa'}};
    if (!chart.classList.contains('js-plotly-plot')) chart.innerHTML = '';
    Plotly.react(chart, traces, layout, {responsive: true, displaylogo: false, scrollZoom: true, modeBarButtonsToRemove: ['select2d', 'lasso2d'],
      toImageButtonOptions: {format: 'png', scale: 2, filename: 'WT-engine-' + list.map(d => d.entry.aircraft_id).join('-vs-')},
      modeBarButtonsToAdd: [{name: 'Download data (CSV)', icon: DATA_ICONS.csv, click: () => download('csv')},
                            {name: 'Download data (JSON)', icon: DATA_ICONS.json, click: () => download('json')}]});
    chart.removeAllListeners?.('plotly_click');
    chart.on('plotly_click', event => {
      const p = event.points?.[0], meta = p?.data.meta; if (!meta) return;
      const d = list.find(item => item.entry.id === meta.entry), mode = d?.result.modes.find(m => m.key === meta.mode);
      const row = mode?.[axis].find(r => x.x(r) === p.x); if (row?.solved) inspect(d, mode, row);
    });
  }

  // ---- Details of a clicked point -------------------------------------------------------
  function inspect(d, mode, r) {
    const result = d.result, list = (values, digits, unit = '') => values?.length ? values.map(v => Number.isFinite(v) ? fmt(v, digits) + unit : '—').join(' / ') : null;
    const dl = rows => '<dl>' + rows.filter(([, v]) => v != null).map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join('') + '</dl>';
    const section = (title, rows) => rows.some(([, v]) => v != null) ? `<section><h3>${title}</h3>${dl(rows)}</section>` : '';
    const prop = propeller(aircraftOf(d.entry));
    el('engine-inspector').hidden = false;
    el('engine-point-title').textContent = `${cleanName(result.name)} · ${mode.label} · ${fmt(r.altitude_m, 0)} m · ${fmt(r.speed_kmh, 0)} km/h`;
    el('engine-point').innerHTML = `<div class="point-grid">${section('FLIGHT', [
      ['Altitude', fmt(r.altitude_m, 0) + ' m'], ['True airspeed', fmt(r.speed_kmh, 0) + ' km/h'], ['IAS', fmt(r.ias_kmh, 0) + ' km/h'], ['Mach', fmt(r.mach, 3)]])}${section('ENGINE', [
      ['Throttle', mode.label], ['Thrust', fmt(r.thrust_kgf, 0) + ' kgf'],
      ['Power', prop ? list(r.engine_power_hp, 0, ' hp') : null], ['RPM', list(r.rpm, 0)], ['RPM limit', list(result.rpm_limit, 0)],
      ['Manifold pressure', list(r.manifold_ata, 3, ' ata')], ['Supercharger gear', result.stages.some(n => n > 1) ? r.gear.join(' / ') : null]])}${prop ? section('PROPELLER', [
      ['Efficiency', Number.isFinite(r.efficiency) ? fmt(r.efficiency, 1) + '%' : null],
      ['Blade angle', r.pitch_deg.map((v, i) => fmt(v, 2) + '°' + (Number.isFinite(r.pitch_percent?.[i]) ? ' / ' + fmt(r.pitch_percent[i], 0) + '%' : '')).join(' · ')],
      ['Propeller control', d.entry.pitch === 'auto' ? 'Automatic' : d.entry.pitch + '% manual'],
      ['Governor', r.governor?.length ? r.governor.map(g => GOVERNOR[g] || g).join(' / ') : null],
      ['Tip Mach', list(r.tip_mach, 3)]]) : ''}</div>`;
  }

  // ---- Downloads ------------------------------------------------------------------------
  function download(kind) {
    const list = displayed(); if (!list.length) return;
    const join = v => Array.isArray(v) ? v.map(x => Number.isFinite(x) ? +x.toFixed(4) : x ?? '').join('/') : Number.isFinite(v) ? +v.toFixed(4) : v ?? '';
    const columns = ['altitude_m', 'speed_kmh', 'ias_kmh', 'mach', 'thrust_kgf', 'power_hp', 'engine_power_hp', 'efficiency', 'rpm', 'pitch_deg', 'pitch_percent', 'manifold_ata', 'gear', 'governor', 'tip_mach'];
    const text = kind === 'json' ? JSON.stringify(list.map(d => ({entry: d.index + 1, propeller_pitch: d.entry.pitch, ...d.result})), null, 1) :
      [['entry', 'aircraft', 'propeller_pitch', 'throttle', 'sweep', ...columns].join(','), ...list.flatMap(d => d.result.modes.flatMap(m => ['altitude', 'speed'].flatMap(sweep =>
        m[sweep].filter(r => r.solved).map(r => [d.index + 1, d.result.aircraft_id, d.entry.pitch, m.label, sweep, ...columns.map(c => join(r[c]))].join(',')))))].join('\n');
    const link = document.createElement('a');
    link.href = URL.createObjectURL(new Blob([text], {type: kind === 'json' ? 'application/json' : 'text/csv'}));
    link.download = `WT-engine-${list.map(d => d.result.aircraft_id).join('-vs-')}.${kind}`;
    link.click(); setTimeout(() => URL.revokeObjectURL(link.href), 1000);
  }

  // The catalog arrives with the EM page's start; the list fills once it is there.
  function whenReady() { if (catalog()) populate(); else setTimeout(whenReady, 300); }
  document.addEventListener('tool-tab-change', event => {
    if (event.detail === 'engine') requestAnimationFrame(() => { if (el('engine-chart').data) Plotly.Plots.resize(el('engine-chart')); });
  });
  new ResizeObserver(() => { if (!root.hidden && el('engine-chart').data) Plotly.Plots.resize(el('engine-chart')); }).observe(el('engine-chart'));
  axisControls(); updateForm(); whenReady();
})();
