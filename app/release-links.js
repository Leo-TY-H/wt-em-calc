'use strict';
// The header shows no source or version link (Leo, 2026-10-22); the version
// stays in release.json. Local copies still link to the hosted website.
(async () => {
  const nav = document.getElementById('release-links');
  if (!nav || !['localhost', '127.0.0.1'].includes(location.hostname)) return;
  try {
    const response = await fetch('./release.json', {cache: 'no-cache'});
    if (!response.ok) throw new Error('Release information unavailable');
    const release = await response.json();
    const a = document.createElement('a');
    a.textContent = 'Website'; a.href = release.website; a.className = 'website-link';
    a.target = '_blank'; a.rel = 'noopener noreferrer';
    nav.replaceChildren(a);
  } catch {
    // Without metadata the header simply has no links.
  }
})();
