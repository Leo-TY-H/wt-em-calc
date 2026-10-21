'use strict';
(async () => {
  const nav = document.getElementById('release-links');
  if (!nav) return;
  try {
    const response = await fetch('./release.json', {cache: 'no-cache'});
    if (!response.ok) throw new Error('Release information unavailable');
    const release = await response.json();
    const tag = 'v' + release.version;
    const links = [
      ['Source', release.repository, 'source-link'],
      [tag, `${release.repository}/tree/${tag}`, 'release-version']
    ];
    if (['localhost', '127.0.0.1'].includes(location.hostname)) {
      links.unshift(['Website', release.website, 'website-link']);
    }
    nav.replaceChildren(...links.map(([label, href, className]) => {
      const a = document.createElement('a');
      a.textContent = label; a.href = href; a.className = className;
      a.target = '_blank'; a.rel = 'noopener noreferrer';
      return a;
    }));
  } catch {
    // Keep the working static release-page link if metadata cannot be loaded.
  }
})();
