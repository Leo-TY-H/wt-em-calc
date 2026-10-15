"""Reproducible sampled stationary candidate charts; no envelope interpolation.

Example: python scripts/em_stationary_chart.py --config settings.json
         --speeds 300 400 500 --loads 1 2 3 --output outputs/candidates
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
import time

import numpy as np


def clean(value):
    """Normalize export values once per shared object, retaining aliases."""
    memo={}
    def visit(item):
        if isinstance(item, (dict,list,tuple,np.ndarray)):
            identity=id(item)
            if identity in memo:return memo[identity]
            if isinstance(item,dict):
                result={};memo[identity]=result
                result.update((str(k),visit(v)) for k,v in item.items() if k!='_detail')
            else:
                result=[];memo[identity]=result
                result.extend(visit(v) for v in (item.tolist() if isinstance(item,np.ndarray) else item))
            return result
        if isinstance(item,np.generic):return visit(item.item())
        if isinstance(item,float) and not math.isfinite(item):return None
        return item
    return visit(value)


def render(record, destination):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import TwoSlopeNorm
    from matplotlib.lines import Line2D

    points = [p for row in record['results'] for p in row['stationary_candidates'] if p['local_valid']]
    values = [p['ps_mps'] for p in points]
    norm = TwoSlopeNorm(vmin=min([-1.] + values), vcenter=0., vmax=max([1.] + values))
    fig, ax = plt.subplots(figsize=(9, 6), layout='constrained')
    ax.scatter([p['speed_kmh'] for p in points], [p['turn_dps'] for p in points],
        c=[p['ps_mps'] for p in points], cmap='RdYlBu', norm=norm,
        s=54, edgecolors='#263248', linewidths=.4, zorder=3)
    missing = [r for r in record['results'] if not any(
        p['local_valid'] for p in r['stationary_candidates'])]
    ax.scatter([r['speed_kmh'] for r in missing], [math.degrees(9.8100004196167 *
        math.sqrt(r['load_g']**2 - 1.) / (r['speed_kmh']/3.6)) for r in missing],
        marker='x', c='#a3aab5', s=24, linewidths=.8, zorder=2)
    ax.set_title('Upright candidates only | bank within +/-90 degrees', fontsize=12)
    ax.set_xlabel('True airspeed (km/h)')
    ax.set_ylabel('Prescribed turn rate (degrees/s)')
    ax.grid(alpha=.18)
    ax.set_facecolor('#f8fafc')
    ax.set_ylim(bottom=-1)
    bar = fig.colorbar(plt.cm.ScalarMappable(norm=norm, cmap='RdYlBu'), ax=ax, shrink=.82)
    bar.set_label('Candidate discrete SEP (m/s)')
    config = record['settings']
    title = record['aircraft_name']
    subtitle = (f"Sea level | fuel {config['fuel_percent']:g}% | throttle {config['throttle']*100:g}% | "
                f"Instructor {'on' if config['instructor'] else 'off'} | "
                f"ammunition {'on' if config['default_ammunition'] else 'off'}")
    if config['altitude_m'] != 0:subtitle = subtitle.replace('Sea level', f"{config['altitude_m']:g} m")
    fig.suptitle(title + ' — sampled stationary results\n' + subtitle, fontsize=14)
    ax.legend(handles=[
        Line2D([], [], marker='o', linestyle='', color='#5679a8', label='Locally admitted candidate'),
        Line2D([], [], marker='x', linestyle='', color='#a3aab5', label='No admitted candidate found')],
        fontsize=8, loc='upper right')
    fig.supxlabel('Global root coverage, governor/aircraft selection and error bounds remain unresolved.\n'
                  'Crosses are not proof of absence. Points are samples, not a certified flight envelope.', fontsize=9)
    fig.savefig(destination.with_suffix('.png'), dpi=170)
    fig.savefig(destination.with_suffix('.svg'))
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--speeds', nargs='+', type=float, required=True)
    parser.add_argument('--loads', nargs='+', type=float, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--max-nfev', type=int, default=100)
    args = parser.parse_args()
    from em_solver import AIRCRAFT, BACKEND, TrimSolver, settings
    from em_stationary import REVISION, solve
    config = settings(dict(json.loads(Path(args.config).read_text()), trim_solver_mode='stationary'))
    if len(config['aircraft']) != 1:raise ValueError('One aircraft per diagnostic chart')
    if any(not math.isfinite(v) or v <= 0 for v in args.speeds):raise ValueError('Positive finite speeds required')
    if any(not math.isfinite(n) or n < 1 for n in args.loads):raise ValueError('Finite loads >= 1 required')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    root = Path(__file__).resolve().parents[1]
    manifest_path = root/'outputs/checkpoint-1-2026-10-11/manifest.json'
    manifest = json.loads(manifest_path.read_text())
    for path, expected in manifest['data_files'].items():
        if hashlib.sha256((root/'references'/path).read_bytes()).hexdigest() != expected:
            raise ValueError('Pinned data mismatch: ' + path)
    name = config['aircraft'][0]
    solver = TrimSolver(name, config)
    started = time.monotonic()
    record = dict(model_revision=REVISION, settings=solver.config, aircraft_name=AIRCRAFT[name]['name'],
        backend=BACKEND, python=sys.version, executable=sys.executable,
        data_manifest_sha256=hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
        verified_data_files=len(manifest['data_files']),
        source_hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (root/'scripts').glob('*.py')},
        argv=sys.argv, sampling='upright bank within +/-90 degrees; ordinary start plus continuation; zero attitude sideslip only',
        interpretation='Locally admitted samples only; global selection, error bounds and coverage unresolved',
        results=[])
    previous = {}
    for speed in sorted(set(args.speeds)):
        for load in sorted(set(args.loads)):
            ordinary = solve(solver, speed, load, initial=previous.get(load), exhaustive=False,
                             max_nfev=args.max_nfev)
            if ordinary['local_valid']:previous[load] = ordinary
            row = dict(ordinary)
            record['results'].append(row)
            record['elapsed_s'] = time.monotonic() - started
            args.output.with_suffix('.json').write_text(json.dumps(clean(record), indent=2, allow_nan=False)+'\n')
            print(f"{len(record['results'])}/{len(set(args.speeds))*len(set(args.loads))} "
                  f"{name} {speed:g} km/h {load:g} g: {row['local_candidate_count']} local candidates "
                  f"({record['elapsed_s']:.1f}s)", flush=True)
    with args.output.with_suffix('.csv').open('w', newline='') as stream:
        keys = ['speed_kmh','load_g','turn_dps','ps_mps','alpha_deg','bank_deg','sideslip_deg',
                'local_valid','status','force_error_g','angular_error_rad_s2','history_error','reasons']
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader()
        for row in record['results']:
            for point in row['stationary_candidates']:
                writer.writerow({k:point.get(k) for k in keys})
    render(record, args.output)
    print(args.output.with_suffix('.png'), flush=True)


if __name__ == '__main__':main()
