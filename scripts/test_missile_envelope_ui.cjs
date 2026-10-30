// Executes the real UI controller with a small DOM/Plotly test double. No browser
// session, server, package installation, or persistent application test hooks.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const elements = new Map(), blobs = new Map();
let requests = 0, draws = 0;
class Element {
  constructor(id) {
    this.id = id; this.value = ''; this.checked = false; this.disabled = false;
    this.hidden = false; this.textContent = ''; this.listeners = {}; this.children = [];
    this.dataset = {}; this.attrs = {}; this.style = {setProperty() {}};
    this.classList = {toggle() {}};
  }
  set innerHTML(html) {
    this.html = html;
    if (this.id === 'envelope-workspace') for (const tag of html.matchAll(/<[^>]*\bid="([^"]+)"[^>]*>/g)) {
      const node = get(tag[1]); node.checked = /\bchecked\b/.test(tag[0]);
      node.value = /\bvalue="([^"]*)"/.exec(tag[0])?.[1] || '';
    }
  }
  get innerHTML() { return this.html || ''; }
  addEventListener(type, fn) { (this.listeners[type] ||= []).push(fn); }
  emit(type) { for (const fn of this.listeners[type] || []) fn({target: this}); }
  closest(selector) { return get(this.id + selector); }
  querySelector(selector) { return selector === '.empty' ? null : get(this.id + selector); }
  querySelectorAll() { return []; }
  setAttribute(name, value) { this.attrs[name] = value; }
  getAttribute(name) { return this.attrs[name]; }
  removeAttribute(name) { delete this.attrs[name]; delete this[name]; }
  replaceChildren(...children) { this.children = children; }
  append(...children) { this.children.push(...children); }
  on() {}
  removeAllListeners() {}
  get tHead() { return get(this.id + '-thead'); }
  get tBodies() { return [get(this.id + '-tbody')]; }
}
function get(id) { if (!elements.has(id)) elements.set(id, new Element(id)); return elements.get(id); }
const context = {console, Blob, setTimeout, clearTimeout, performance,
  document: {getElementById: get, addEventListener() {}, createElement: tag => new Element(tag)},
  window: {}, ResizeObserver: class { observe() {} }, requestAnimationFrame: fn => fn(),
  fetch: async () => { requests++; throw Error('A display toggle must not make a request'); },
  URL: {createObjectURL(blob) { const url = 'blob:' + blobs.size; blobs.set(url, blob); return url; }, revokeObjectURL() {}},
  Plotly: {async react(node, traces, layout) { node.data = traces; node.layout = layout; draws++; },
    purge(node) { delete node.data; }, async relayout() {}, Plots: {resize() {}}}};
const source = fs.readFileSync(path.join(__dirname, '../app/missile-envelope.js'), 'utf8');
const hook = `
globalThis.test = {
  setup(data) { runs = data; runSets.target = data; runSets.launcher = [{cache: {old: true}}]; },
  async render() { await render(); setExports(true); },
  view(run) { use(run); return {rays, escapeRays, field: field()}; },
  cacheOther() { return runSets.launcher[0].cache; },
  reset, showCentre
};
`;
vm.runInNewContext(source.replace(/\}\)\(\);\s*$/, hook + '})();'), context);
const interval = (a, b) => ({start_m: a, end_m: b, min_at_search_limit: false, max_at_search_limit: false});
function ray(angle, firstOnly = false, motion = 'straight') {
  return {key: [0, angle], layer_m: 1000, azimuth_deg: angle, motion, level: 0, search: [150, 20000],
    intervals: [interval(150, 900), interval(2300, 10000)], min_range_m: 150, max_range_m: 10000,
    min_time_s: 10, max_time_s: 15, simulations: 4,
    first_pass: {intervals: firstOnly ? [] : [interval(2300, 10000)], min_range_m: firstOnly ? null : 2300,
      max_range_m: firstOnly ? null : 10000, min_time_s: firstOnly ? null : 5, max_time_s: firstOnly ? null : 15,
      min_bracket_m: null, max_bracket_m: null, ragged_edge: false},
    with_energy: {intervals: [interval(150, 900), interval(2300, 7000)], min_range_m: 150, max_range_m: 7000,
      min_time_s: 10, max_time_s: 9, min_bracket_m: null, max_bracket_m: null, ragged_edge: false},
    first_pass_with_energy: {intervals: firstOnly ? [] : [interval(2300, 7000)], min_range_m: firstOnly ? null : 2300,
      max_range_m: firstOnly ? null : 7000, min_time_s: 5, max_time_s: 9, min_bracket_m: null, max_bracket_m: null, ragged_edge: false},
    samples: [{range_m: 150, hit: true, turn_back: true, outcome: 'proximity_event', time_s: 10, closest_m: 0},
      {range_m: 5000, hit: true, turn_back: false, outcome: 'proximity_event', time_s: 5, closest_m: 0}]};
}
function run(index, allReturn = false) {
  return {index, missile: 'fixture' + index, color: '#38c9d7', cache: null,
    submitted: {missile: 'fixture' + index, frame: 'target', quality: 'quick', layers_m: [1000],
      launcher: {platform: 'air', speed_mps: 250}, target: {speed_mps: 250}, escape: {load_g: 9, delay_s: 0}},
    rays: new Map([0, 90, 180].map(a => [String(a), ray(a, allReturn)])),
    escapes: new Map([0, 90, 180].map(a => [String(a), ray(a, allReturn, 'escape')])),
    jobInfo: {mirrored: true, status: 'complete', config: {azimuth_step_deg: 90}}, summary: {simulations: 12}};
}
async function settle() { for (let i = 0; i < 4; i++) await new Promise(resolve => setImmediate(resolve)); }
async function exported() { return JSON.parse(await blobs.get(get('envelope-download-json').href).text()); }
(async () => {
  const toggle = get('envelope-turn-back');
  const energy = get('envelope-terminal-energy');
  assert.equal(toggle.checked, true);
  assert.equal(energy.checked, false);
  const markup = get('envelope-workspace').innerHTML;
  assert.match(markup, /<label class="check-row envelope-nez-toggle"><input id="envelope-turn-back" type="checkbox" checked><span>Include turn-back hits<\/span><\/label>/);
  assert.equal((markup.match(/Include turn-back hits/g) || []).length, 1);
  assert.match(markup, /<label class="check-row envelope-nez-toggle"><input id="envelope-terminal-energy" type="checkbox"><span>Require terminal energy<\/span><\/label>/);
  const data = [run(0), run(1, true)]; context.test.setup(data);
  await context.test.render();
  const original = JSON.stringify(get('envelope-chart').data), raw = JSON.stringify([...data[0].rays]);
  const first = await exported(); assert.equal(first.envelopes[0].display.include_turn_back_hits, true);
  toggle.checked = false; toggle.emit('change'); await settle();
  assert.equal(get('envelope-error').textContent, '');
  assert.notEqual(JSON.stringify(get('envelope-chart').data), original);
  assert.equal(context.test.cacheOther(), null);
  const view = context.test.view(data[0]);
  assert.equal(view.rays.get('0').min_range_m, 2300);
  assert.equal(view.escapeRays.get('0').min_range_m, 2300);
  assert.equal(context.test.view(data[1]).rays.get('0').max_range_m, null);
  const filtered = await exported();
  assert.equal(filtered.envelopes[0].display.include_turn_back_hits, false);
  assert.equal(filtered.envelopes[0].rays[0].min_range_m, 2300);
  assert.equal(filtered.envelopes[0].rays[0].all_hits.min_range_m, 150);
  assert.equal(filtered.envelopes[0].escape_rays[0].min_range_m, 2300);
  assert.equal(filtered.envelopes[1].rays[0].max_range_m, null);
  const csv = await blobs.get(get('envelope-download-csv').href).text();
  assert.match(csv, /include_turn_back_hits/); assert.ok(csv.split('\n')[1].endsWith(',false,false'));
  assert.equal(JSON.stringify([...data[0].rays]), raw);
  toggle.checked = true; toggle.emit('change'); await settle();
  assert.equal(JSON.stringify(get('envelope-chart').data), original);
  energy.checked = true; energy.emit('change'); await settle();
  assert.equal(context.test.view(data[0]).rays.get('0').min_range_m, 150);
  assert.equal(context.test.view(data[0]).rays.get('0').max_range_m, 7000);
  assert.equal(context.test.view(data[0]).escapeRays.get('0').max_range_m, 7000);
  assert.equal((await exported()).envelopes[0].display.require_terminal_energy, true);
  toggle.checked = false; toggle.emit('change'); await settle();
  assert.equal(context.test.view(data[0]).rays.get('0').min_range_m, 2300);
  assert.equal(context.test.view(data[0]).rays.get('0').max_range_m, 7000);
  const legacy = run(2);
  for (const r of legacy.rays.values()) { delete r.with_energy; delete r.first_pass_with_energy; }
  assert.equal(context.test.view(legacy).rays.get('0').max_range_m, null);
  context.test.reset(); await settle();
  assert.equal(toggle.checked, true);
  assert.equal(energy.checked, false);
  assert.equal(JSON.stringify(get('envelope-chart').data), original);
  assert.equal(requests, 0);
  console.log(JSON.stringify({status: 'passed', draws, requests, checks: ['same toggle markup', 'default on', 'plot changes and restores',
    'multiple missiles', 'empty filtered region', 'NEZ filtered', 'JSON and CSV filtered', 'raw outcomes preserved', 'other centre cache invalidated',
    'four filter combinations', 'unknown energy excluded', 'energy default off', 'reset restores']}));
})().catch(error => { console.error(error); process.exitCode = 1; });
