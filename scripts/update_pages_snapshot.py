#!/usr/bin/env python3
"""Refresh the static aircraft catalog used by the GitHub Pages build."""

import argparse
import json
import os
from pathlib import Path
from urllib.request import urlopen


ROOT = Path(__file__).resolve().parents[1]
TARGET = ROOT / "app" / "api" / "meta"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--url",
        default="http://127.0.0.1:8765/api/meta",
        help="Metadata endpoint from a running local plotter",
    )
    parser.add_argument('--offline', action='store_true', help='Build directly from local data; no running server required')
    args = parser.parse_args()
    if args.offline:
        # Catalog generation needs no numerical kernels or native executable.
        os.environ.setdefault('WT_EM_BACKEND', 'python')
        from em_solver import AIRCRAFT, DEFAULTS
        data = dict(aircraft=dict(AIRCRAFT), defaults=DEFAULTS)
    else:
        with urlopen(args.url, timeout=30) as response:
            data = json.load(response)
    if not isinstance(data.get("aircraft"), dict) or not isinstance(data.get("defaults"), dict):
        raise ValueError("Metadata response is missing aircraft or defaults")
    data["latest"] = None
    data["public_static"] = True
    version = ROOT / 'references/data-version.json'
    if version.exists():
        data['game_data'] = {key: value for key, value in json.loads(version.read_text(encoding='utf-8')).items() if key != 'files'}
    TARGET.parent.mkdir(parents=True, exist_ok=True)
    temporary = TARGET.with_suffix(".tmp")
    temporary.write_text(json.dumps(data, separators=(",", ":")), encoding="utf-8")
    temporary.replace(TARGET)
    print(f"Wrote {len(data['aircraft']):,} aircraft to {TARGET.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
