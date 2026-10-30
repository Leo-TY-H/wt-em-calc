// Run both production controllers with a minimal DOM; exercise per-missile
// choices, disabled states and the actual submitted HTTP payloads.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const catalog = JSON.parse(fs.readFileSync(path.join(__dirname, '../references/missile-launchers.json')));
const ids = ['cn_pl12a', 'us_aim_120d', 'uk_sraam'];
const meta = {aircraft: catalog.aircraft, missiles: ids.map(id => ({id, name: id, family: catalog.families[id], aircraft_ids: catalog.carriers[id]}))};
function harness(kind) {
  const elements = new Map(), posts = [];
  const get = id => { if (!elements.has(id)) elements.set(id, new Element(id)); return elements.get(id); };
  class Element {
    constructor(id) {
      this.id = id; this.value = ''; this.checked = false; this.disabled = false; this.hidden = false;
      this.textContent = ''; this.listeners = {}; this.children = []; this.dataset = {}; this.attrs = {};
      this.style = {setProperty() {}}; this.classList = {toggle() {}}; this.clientWidth = 0;
      this.validity = {valid: true}; this.validationMessage = '';
    }
    set innerHTML(html) {
      this.html = html;
      if (this.id.endsWith('workspace')) for (const tag of html.matchAll(/<[^>]*\bid="([^"]+)"[^>]*>/g)) {
        const node = get(tag[1]); node.checked = /\bchecked\b/.test(tag[0]);
        node.value = /\bvalue="([^"]*)"/.exec(tag[0])?.[1] || '';
      }
    }
    get innerHTML() { return this.html || ''; }
    addEventListener(type, fn) { (this.listeners[type] ||= []).push(fn); }
    async emit(type) { for (const fn of this.listeners[type] || []) await fn({target: this, preventDefault() {}}); }
    closest(selector) { return get(this.id + selector); }
    querySelector(selector) { return selector === '.empty' ? null : get(this.id + selector); }
    querySelectorAll(selector) {
      if (selector === 'button,select') return this.children.flatMap(c => [c.querySelector('button'), ...c.children.flatMap(f => f.children)]);
      return [];
    }
    setAttribute(k, v) { this.attrs[k] = v; }
    getAttribute(k) { return this.attrs[k]; }
    removeAttribute(k) { delete this.attrs[k]; }
    toggleAttribute(k, on) { if (on) this.attrs[k] = ''; else delete this.attrs[k]; }
    replaceChildren(...children) { this.children = children; }
    append(...children) { this.children.push(...children); }
    setCustomValidity() {}
    checkValidity() { return true; }
    on() {}
    removeAllListeners() {}
    get tHead() { return get(this.id + '-thead'); }
    get tBodies() { return [get(this.id + '-tbody')]; }
  }
  const context = {console, Blob, performance, setTimeout, clearTimeout,
    document: {getElementById: get, createElement: tag => new Element(tag), querySelectorAll: () => [], addEventListener() {}, dispatchEvent() {}},
    window: {addEventListener(){},removeEventListener(){},requestAnimationFrame(){},cancelAnimationFrame(){},setTimeout,clearTimeout}, ResizeObserver: class {observe() {}}, requestAnimationFrame() {}, cancelAnimationFrame(){}, CustomEvent: class {},
    URL: {createObjectURL() {return 'blob:test';}, revokeObjectURL() {}},
    fetch: async (url, options) => {
      if (options?.method === 'POST') { posts.push(JSON.parse(options.body)); return {ok: true, json: async () => ({id: String(posts.length)})}; }
      if (url.endsWith('/meta')) return {ok: true, json: async () => meta};
      // A finished, empty result lets us inspect dispatch without calculating.
      const envelope = url.includes('/envelopes/');
      return {ok: true, json: async () => envelope ? {status: 'complete', rays: [], config: posts.at(-1)} : {status: 'failed', error: 'fixture end'}};
    },
    Plotly: {async react(node, data) {node.data = data;}, purge() {}, async relayout() {}, Plots: {resize() {}}}};
  vm.createContext(context);
  vm.runInContext(fs.readFileSync(path.join(__dirname, '../app/missile-launchers.js'), 'utf8'), context);
  vm.runInContext(fs.readFileSync(path.join(__dirname, '../app/missile-plot.js'), 'utf8'), context);
  const file = kind === 'missile' ? 'missiles.js' : 'missile-envelope.js';
  const source = fs.readFileSync(path.join(__dirname, '../app', file), 'utf8');
  const hook = kind === 'missile'
    ? `globalThis.test={setup(m,ids,saved={}){meta=m;selectedMissiles=ids;Object.assign(launcherAircraft,saved);renderSelected();busy(false);},busy,inputs};`
    : `globalThis.test={setup(m,ids,saved={}){meta=m;selected=ids;Object.assign(launcherAircraft,saved);renderSelected();updateForm();},busy(on){job=on?'fixture':null;busy(on);},inputs,
        traces(data){runs=data;return gimbalTrace(10000);}};`;
  vm.runInContext(source.replace(/\}\)\(\);\s*$/, hook + '})();'), context);
  return {context, get, posts};
}
(async () => {
  for (const kind of ['missile', 'envelope']) {
    const {context, get, posts} = harness(kind);
    context.test.setup(meta, ids);
    assert.equal(get(kind + '-run').disabled, true, 'Aircraft is required');
    const selectors = () => get(kind + '-selected').children.flatMap(card => card.children.flatMap(field => field.children));
    assert.equal(selectors().length, 2, 'Only radar missiles have an aircraft selector');
    assert.equal(get(kind + '-selected').children[2].children.length, 0, 'No selector for IR');
    let [pl, aim] = selectors();
    assert.deepEqual(pl.children.slice(1).map(o => o.value).sort(), [...catalog.carriers.cn_pl12a].sort());
    assert.ok(aim.children.some(o => o.value === 'f_15c_golden_eagle'));
    assert.ok(!pl.children.some(o => o.value === 'f_15c_golden_eagle'));
    pl.value = 'j_10c'; await pl.emit('change'); assert.equal(get(kind + '-run').disabled, true);
    aim.value = 'f_15c_golden_eagle'; await aim.emit('change'); assert.equal(get(kind + '-run').disabled, false);
    context.test.busy(true); assert.ok(selectors().every(s => s.disabled));
    context.test.busy(false); [pl, aim] = selectors();
    assert.equal(pl.value, 'j_10c'); assert.equal(aim.value, 'f_15c_golden_eagle');
    get('envelope-quality').value = 'quick';
    await get(kind + '-form').emit('submit');
    assert.equal(posts.length, 3);
    assert.deepEqual(posts.map(p => p.launcher.aircraft_id), ['j_10c', 'f_15c_golden_eagle', undefined]);
    assert.ok(!('aircraft_id' in posts[2].launcher));
    assert.deepEqual(posts.map(p => p.missile), ids);
    if (kind === 'envelope') {
      assert.ok(!get('envelope-workspace').innerHTML.includes('id="envelope-gimbal"'));
      get('envelope-centre-launcher').checked = true;
      const traces = context.test.traces([70, 60].map((g, i) => ({color: ['cyan', 'orange'][i],
        submitted: {frame: 'launcher', launcher: {platform: 'air', gimbal_deg: g, aircraft: {name: 'aircraft'+i}}}})));
      assert.deepEqual(Array.from(traces, t => t.theta[0]), [70, 60]);
      assert.deepEqual(Array.from(traces, t => t.line.color), ['cyan', 'orange']);
    }
    context.test.setup(meta, ['uk_sraam'], {uk_sraam: 'hunter_f6'});
    assert.equal(selectors().length, 0);
    assert.equal(get(kind + '-run').disabled, false, 'IR alone needs no aircraft choice');
    await get(kind + '-form').emit('submit');
    assert.equal(posts.length, 4);
    assert.equal(posts[3].missile, 'uk_sraam');
    assert.ok(!('aircraft_id' in posts[3].launcher), 'Discard saved IR aircraft choice');
  }
  console.log(JSON.stringify({status: 'passed', controllers: 2, posts: 8,
    checks: ['compatibility options', 'required selections', 'independent choices', 'busy state', 'selection persistence',
      'actual request payloads', 'per-aircraft plot limits', 'manual gimbal removed', 'mixed radar and IR selections',
      'IR alone without aircraft', 'saved IR selection discarded'], browser_rendering_tested: false}));
})().catch(error => {console.error(error); process.exitCode = 1;});
