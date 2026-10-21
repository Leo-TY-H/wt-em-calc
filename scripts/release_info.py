import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def release_info(root=ROOT):
    root = Path(root)
    info = json.loads((root / 'app/release.json').read_text(encoding='utf-8'))
    tag = 'v' + info['version']
    info.update(tag=tag, source_url=info['repository'] + '/tree/' + tag,
                release_url=info['repository'] + '/releases/tag/' + tag)
    build = root / 'build-info.json'
    if build.exists():
        info['commit'] = json.loads(build.read_text(encoding='utf-8'))['commit']
    return info
