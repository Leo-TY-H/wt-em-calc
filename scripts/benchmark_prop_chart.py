"""Reproducible full-range automatic prop chart timing and saved evidence."""
import argparse
import json
import time
from pathlib import Path
from em_solver import compute, settings

ROOT = Path(__file__).resolve().parents[1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--label', required=True)
    ap.add_argument('--aircraft', nargs='+', default=['yak-3'])
    ap.add_argument('--standard', action='store_true')
    args = ap.parse_args()
    out = ROOT / 'analysis/prop-speed'
    out.mkdir(exist_ok=True)
    cfg = settings(dict(aircraft=args.aircraft, speed_samples=17 if args.standard else 9,
                        load_samples=9 if args.standard else 7,
                        sep_tolerance_mps=.5 if args.standard else 1.))
    started = time.monotonic()
    def progress(p):
        if p.get('speed_kmh') is not None:
            print(json.dumps(p), flush=True)
    result = compute(cfg, progress=progress)
    result['benchmark_wall_seconds'] = time.monotonic() - started
    (out / (args.label + '.json')).write_text(json.dumps(result) + '\n')
    print('COMPLETE', args.label, result['benchmark_wall_seconds'], flush=True)


if __name__ == '__main__':
    main()
