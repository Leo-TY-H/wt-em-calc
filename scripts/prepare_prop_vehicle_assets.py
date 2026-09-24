"""Pin source vehicle-to-FM/model mappings for collision mass preparation.

Uses the same datamine commit as the existing FM catalog. Installed collision
assets remain separately hashed; matching a name does not prove patch equality.
"""
import concurrent.futures
import hashlib
import json
import time
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
COMMIT = json.loads((ROOT/'references/jet-catalog/fm-manifest.json').read_text())['commit']
REPO = 'https://api.github.com/repos/gszabi99/War-Thunder-Datamine'
OUT = ROOT/'references/prop-vehicles'


def request(url):
    for attempt in range(4):
        try:
            return urllib.request.urlopen(urllib.request.Request(url, headers={'User-Agent':'FM-reconstruction'}), timeout=60).read()
        except Exception:
            if attempt == 3: raise
            time.sleep(1 + attempt)


def main():
    OUT.mkdir(exist_ok=True, parents=True)
    tree = json.loads(request(REPO+'/git/trees/'+COMMIT))
    for name in ['aces.vromfs.bin_u', 'gamedata', 'flightmodels']:
        entry = next(x for x in tree['tree'] if x['path'] == name)
        tree = json.loads(request(entry['url']))
    rows = [r for r in tree['tree'] if r['path'].endswith('.blkx') and r['type']=='blob']
    def fetch(row):
        path = OUT/row['path']
        def blob_sha(data): return hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest()
        data = path.read_bytes() if path.exists() else b''
        if blob_sha(data) != row['sha']:
            data = request('https://raw.githubusercontent.com/gszabi99/War-Thunder-Datamine/'+COMMIT+'/aces.vromfs.bin_u/gamedata/flightmodels/'+row['path'])
            if blob_sha(data) != row['sha']: raise ValueError('Source hash mismatch: '+row['path'])
            path.write_bytes(data)
        value = json.loads(data)
        return dict(vehicle=path.stem, sha=row['sha'], fm=value.get('fmFile'), model=value.get('model'))
    results=[]
    with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:
        for result in pool.map(fetch, rows):
            results.append(result)
            if len(results)%100==0: print(len(results), '/', len(rows), flush=True)
    (OUT/'manifest.json').write_text(json.dumps(dict(commit=COMMIT, records=results), indent=2)+'\n')
    print('Pinned', len(results), 'vehicle mappings', flush=True)


if __name__=='__main__': main()
