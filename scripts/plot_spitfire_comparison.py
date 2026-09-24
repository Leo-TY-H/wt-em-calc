"""Standalone evidence figure for the supplied Spitfire turn recordings."""
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from analyze_spitfire_flights import OUT
from body_dynamics import G


def main():
    telemetry = json.loads((OUT / 'telemetry.json').read_text())
    before = {r['file']:r['matched_rate_point'] for r in json.loads((OUT / 'matched-points.json').read_text())}
    after = {r['file']:r['matched_rate_point'] for r in json.loads((OUT / 'matched-points-octane.json').read_text())}
    edges = {r['file']:r['boundary'] for r in json.loads((OUT / 'boundaries-octane.json').read_text())}
    plt.rcParams.update({'font.size':10, 'axes.spines.top':False, 'axes.spines.right':False})
    fig, axes = plt.subplots(1, 3, figsize=(12.8, 4.5), sharey=True)
    for i, row in enumerate(telemetry):
        name = row['file']; measured = row['means']['Thrust, kgf']
        for point, color, marker, label in [(before[name], '#ba5847', 'x', 'Before upgrade fix'),
                                            (after[name], '#2274a5', 'o', '150-octane applied')]:
            axes[0].scatter(100. * (point['engine_force_n'][0] / float(G) / measured - 1.), i,
                c=color, marker=marker, s=65, label=label if i == 0 else None)
            axes[1].scatter(point['ps_mps'], i, c=color, marker=marker, s=65)
        axes[1].scatter(row['ps_fit_mps'], i, c='#222222', marker='s', s=30,
            label='Recorded energy trend' if i == 0 else None)
        axes[2].scatter(row['turn_dps'], i, c='#222222', marker='s', s=35,
            label='Recorded heading rate' if i == 0 else None)
        axes[2].scatter(edges[name]['turn_dps'], i, c='#2274a5', marker='D', s=45,
            label='Upgraded Instructor boundary' if i == 0 else None)
    axes[0].set_yticks(np.arange(2), ['Clean\n319.007 km/h TAS', 'Landing flaps\n252.527 km/h TAS'])
    axes[0].set_xlabel('Model minus recorded thrust (%)')
    axes[1].set_xlabel('Specific excess power, Ps (m/s)')
    axes[2].set_xlabel('Turn rate at recorded TAS (°/s)')
    for ax in axes:
        ax.set_ylim(1.5, -.5); ax.grid(axis='x', alpha=.18)
    for ax in axes[:2]:
        ax.axvline(0., c='#999999', lw=.8, ls='--', zorder=0)
    axes[0].legend(loc='upper left', bbox_to_anchor=(0., 1.22), frameon=False, fontsize=8)
    axes[1].legend(loc='upper left', bbox_to_anchor=(0., 1.22), frameon=False, fontsize=8)
    axes[2].legend(loc='upper left', bbox_to_anchor=(0., 1.22), frameon=False, fontsize=8)
    fig.suptitle('Spitfire Mk IX USSR · measured turns and the verified upgrade correction', fontsize=13, y=.98)
    fig.text(.02, .035, 'WEP, automatic controls, Instructor, 30% fuel; recorded total mass and altitude matched.\n'
        'Thrust and Ps prescribe the measured rate. Instructor boundary is independently solved; it is not a Ps = 0 curve.', fontsize=9)
    fig.subplots_adjust(left=.16, right=.985, bottom=.24, top=.73, wspace=.18)
    fig.savefig(OUT / 'comparison.png', dpi=170)
    fig.savefig(OUT / 'comparison.svg')


if __name__ == '__main__':
    main()
