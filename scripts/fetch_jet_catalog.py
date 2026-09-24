"""Download hash-verified FM inputs from the reconstruction's pinned commit."""
import hashlib
import json
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
from urllib.request import urlopen

ROOT=Path(__file__).resolve().parents[1]
DIRECTORY=ROOT/'references/jet-catalog'


def main():
    manifest=json.loads((DIRECTORY/'fm-manifest.json').read_text())
    destination=DIRECTORY/'fm';destination.mkdir(exist_ok=True)
    def fetch(item):
        path=destination/Path(item['path']).name
        if path.exists():content=path.read_bytes()
        else:
            url=f"https://raw.githubusercontent.com/gszabi99/War-Thunder-Datamine/{manifest['commit']}/{item['path']}"
            for attempt in range(4):
                try:
                    with urlopen(url,timeout=40) as response:content=response.read()
                    break
                except OSError:
                    if attempt==3:raise
                    time.sleep(.5*(attempt+1))
        digest=hashlib.sha1(f'blob {len(content)}\0'.encode()+content).hexdigest()
        if digest!=item['sha']:raise ValueError('Source hash mismatch: '+str(path))
        json.loads(content)
        if not path.exists():path.write_bytes(content)
        return path.name
    failures=[]
    with ThreadPoolExecutor(max_workers=12) as pool:
        futures={pool.submit(fetch,item):item['path'] for item in manifest['files']}
        for i,f in enumerate(as_completed(futures),1):
            try:f.result()
            except Exception as error:failures.append(dict(path=futures[f],error=str(error)))
            if i%100==0:print(f'{i}/{len(futures)} downloaded/verified',flush=True)
    print(json.dumps(dict(files=len(futures),failures=failures),indent=2))
    if failures:raise SystemExit(1)


if __name__=='__main__':main()
