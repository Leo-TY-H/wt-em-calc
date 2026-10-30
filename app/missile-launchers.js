(() => {
  'use strict';
  const choices = (meta, missile) => (meta?.missiles.find(m => m.id === missile)?.aircraft_ids || [])
    .filter(id => meta.aircraft[id])
    .map(id => ({id, ...meta.aircraft[id]}))
    .sort((a, b) => a.name.localeCompare(b.name, undefined, {numeric: true}) || a.id.localeCompare(b.id));
  const needsAircraft = (meta, missile) => meta?.missiles.find(m => m.id === missile)?.family === 'radar';
  const valid = (meta, missile, aircraft) => meta?.missiles.find(m => m.id === missile)?.family === 'optical' ||
    (needsAircraft(meta, missile) && choices(meta, missile).some(a => a.id === aircraft));
  function selector(meta, missile, aircraft, disabled, onChange) {
    const field = document.createElement('label'); field.className = 'field missile-launcher-field';
    field.textContent = 'Launcher aircraft';
    const select = document.createElement('select'), available = choices(meta, missile);
    select.required = true; select.disabled = disabled;
    select.setAttribute('aria-label', `Launcher aircraft for ${meta?.missiles.find(m => m.id === missile)?.name || missile}`);
    for (const item of [{id: '', name: available.length ? 'Select aircraft…' : 'No compatible aircraft available'}, ...available]) {
      const option = document.createElement('option'); option.value = item.id; option.textContent = item.name; option.title = item.id;
      select.append(option);
    }
    select.value = valid(meta, missile, aircraft) ? aircraft : '';
    select.addEventListener('change', () => onChange(select.value));
    field.append(select); return field;
  }
  // Clone the shared launch conditions; one selection must never leak into a
  // different missile in the same comparison.
  const launcher = (base, aircraft) => {
    const clean = {...base}; delete clean.aircraft; delete clean.aircraft_id;
    if (aircraft) clean.aircraft_id = aircraft;
    return clean;
  };
  const api = {choices, needsAircraft, valid, selector, launcher};
  if (typeof module !== 'undefined') module.exports = api;
  else window.missileLaunchers = api;
})();
