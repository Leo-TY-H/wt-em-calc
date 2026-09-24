"""Refresh offline wiki names; retain exact vehicle IDs, including shared jet FMs."""
import html
import json
import re
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[1]
SOURCE = 'https://wiki.warthunder.com/aviation'
DESTINATION = ROOT / 'references/vehicle-names/wiki.json'


def tree_names(document):
    pattern = r'class="wt-tree_item(?: [^"]*)?" data-unit-id="([^"]+)".*?class="wt-tree_item-text">(.*?)</div>'
    return {key: html.unescape(re.sub('<[^>]+>', '', text)).replace('\xa0', ' ')
            for key, text in re.findall(pattern, document, re.S)}


def main():
    from aircraft_catalog import catalog
    document = urlopen(SOURCE, timeout=30).read().decode()
    names = tree_names(document)
    if len(names) < 1000:
        raise ValueError('Wiki aircraft tree format changed; refusing incomplete replacement')
    records = json.loads((ROOT / 'references/prop-vehicles/manifest.json').read_text())['records']
    wanted = set(catalog()) | {r['vehicle'] for r in records}
    sources = {key: SOURCE for key in names}
    missing = []

    def fetch(key):
        url = 'https://wiki.warthunder.com/unit/' + key
        try:
            page = urlopen(url, timeout=30).read().decode()
        except HTTPError as error:
            if error.code == 404:
                return key, None, url
            raise
        title = html.unescape(re.search(r'<title>(.*?)</title>', page, re.S)[1])
        if not title.endswith(' | War Thunder Wiki'):
            raise ValueError('Unexpected wiki page: ' + url)
        return key, title.removesuffix(' | War Thunder Wiki').replace('\xa0', ' '), url

    with ThreadPoolExecutor(max_workers=8) as pool:
        for key, name, url in pool.map(fetch, sorted(wanted - names.keys())):
            if name:
                names[key] = name
                sources[key] = url
            else:
                missing.append(key)
    result = dict(source=SOURCE, fetched_at=datetime.now(timezone.utc).isoformat(),
                  names=dict(sorted(names.items())), sources=sources, unlisted=missing)
    DESTINATION.parent.mkdir(parents=True, exist_ok=True)
    DESTINATION.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    print(f'{len(names)} wiki names; {len(missing)} IDs without a wiki page')


if __name__ == '__main__':
    main()
