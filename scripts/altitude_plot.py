"""Contour paths and portable exports for the independent altitude feature."""
import csv
import io
import math

import numpy as np
from matplotlib.figure import Figure
from matplotlib.tri import Triangulation
from matplotlib.collections import LineCollection


def contours(data):
    surface=data.get('surface')
    if not data['triangles'] and surface is None:
        return []
    if surface is not None:
        z=np.ma.masked_invalid(np.array(surface['sep_mps'],dtype=float))
        if not z.count():return []
    else:
        points = data['points']
        used = sorted({i for t in data['triangles'] for i in t})
        index = {old: new for new, old in enumerate(used)}
        tri = Triangulation([points[i]['speed_kmh'] for i in used],
                            [points[i]['altitude_m'] for i in used],
                            [[index[i] for i in t] for t in data['triangles']])
        z = np.array([points[i]['ps_mps'] for i in used])
    spacing = data['settings']['contour_interval_mps']
    lo, hi = math.ceil(float(z.min()) / spacing), math.floor(float(z.max()) / spacing)
    # Avoid an unbounded export for pathological configurations/intervals.
    stride = max(1, math.ceil(max(0, hi-lo) / 160))
    levels = sorted(set([i*spacing for i in range(lo, hi+1, stride)] + ([0.] if z.min() <= 0 <= z.max() else [])))
    if not levels or z.min() == z.max():
        return []
    fig = Figure(); ax = fig.subplots()
    if surface is not None:
        x=np.array(surface['speeds_kmh']);y=np.broadcast_to(np.array(surface['altitudes_m'])[:,None],x.shape)
        cs=ax.contour(x,y,z,levels=levels,corner_mask=False)
    else:cs = ax.tricontour(tri, z, levels=levels)
    rows = [dict(sep_mps=float(level), paths=[path.tolist() for path in paths if len(path) > 1])
            for level, paths in zip(cs.levels, cs.allsegs)]
    fig.clear()
    return rows


def chart_payload(data):
    fields = ('speed_kmh', 'altitude_m', 'ps_mps', 'ps_continuous_mps', 'valid', 'category',
              'reasons', 'alpha_deg', 'bank_deg', 'ias_kmh', 'mach', 'flaps_percent',
              'force_error_g', 'angular_error_rad_s2', 'history_error')
    return dict(schema=data['schema'], settings=data['settings'], aircraft_name=data['aircraft_name'],
                contours=data['contours'], speed_limits=data['speed_limits'], sampling=data['sampling'],
                energy_guide=data.get('energy_guide'),
                masked_cells=data['masked_cells'], elapsed_s=data['elapsed_s'], method=data['method'],
                points=[dict(index=i, **{k: p.get(k) for k in fields}) for i, p in enumerate(data['points'])])


def export_csv(data):
    stream = io.StringIO(newline='')
    fields = ['speed_kmh', 'altitude_m', 'load_g', 'ps_mps', 'ps_continuous_mps', 'valid',
              'category', 'reasons', 'ias_kmh', 'mach', 'alpha_deg', 'bank_deg',
              'sideslip_deg', 'flaps_percent', 'force_error_g', 'angular_error_rad_s2',
              'history_error', 'vertical_step_velocity_mps', 'altitude_correction']
    writer = csv.DictWriter(stream, fieldnames=fields)
    writer.writeheader()
    for p in data['points']:
        row = {key: p.get(key) for key in fields}
        row['reasons'] = '; '.join(p.get('reasons', []))
        writer.writerow(row)
    return stream.getvalue()


def export_figure(data, path):
    fig = Figure(figsize=(12, 8), layout='constrained')
    fig.get_layout_engine().set(rect=(0., 0., 1., .955))
    ax = fig.subplots()
    for contour in data['contours']:
        level = contour['sep_mps']; paths = contour['paths']
        color = '#076b79' if level > 0 else '#a55132' if level < 0 else '#142b40'
        ax.add_collection(LineCollection(paths, colors=color, linewidths=3.6 if level == 0 else 1.5,
                                        linestyles='solid' if level >= 0 else 'dashed'))
        for line in paths:
            if len(line) > 6:
                x, y = line[len(line)//2]
                ax.text(x, y, f'{level:g}', fontsize=8, color=color,
                        bbox=dict(facecolor='white', edgecolor='none', alpha=.8, pad=.7))
    if data['settings']['conditions']['structural_limits']:
        ax.plot([p['speed_kmh'] for p in data['speed_limits']],
                [p['altitude_m'] for p in data['speed_limits']], color='#ab688b', lw=1.2, label='IAS / Mach limit')
    guide=data.get('energy_guide')
    if guide and guide['status']=='complete':
        x=[];y=[]
        for p in guide['points']:
            if not p['connected_from_previous']:x.append(np.nan);y.append(np.nan)
            x.append(p['speed_kmh']);y.append(p['altitude_m'])
        ax.plot(x,y,color='#b08013',lw=2.4,label='Continuous best-SEP guide')
        ax.legend(loc='upper left',fontsize=8)
    c = data['settings']
    ax.set(xlim=(c['speed_min_kmh'], c['speed_max_kmh']), ylim=(c['altitude_min_m'], c['altitude_max_m']),
           xlabel='True airspeed (km/h)', ylabel='Altitude (m)',
           title=data['aircraft_name']+'\nSpeed–altitude · SEP contours (m/s) · bold at zero')
    ax.grid(alpha=.18)
    fig.savefig(path, dpi=180)
    fig.clear()
