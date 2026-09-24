"""Reduced, reproducible end-to-end EM plot benchmark across aircraft types."""
import argparse
import json
import time
from collections import Counter
from pathlib import Path

import numpy as np

from em_plot import enrich
from em_solver import BACKEND,compute,settings


CASES={
    'ki61_reported':dict(aircraft=['ki_61_1a_otsu_china'],speed_min_kmh=150.,speed_max_kmh=1300.),
    'p47_radial':dict(aircraft=['p-47d-28'],speed_min_kmh=150.,speed_max_kmh=1300.),
    'mosquito_twin':dict(aircraft=['mosquito_fb_mk6'],speed_min_kmh=150.,speed_max_kmh=1300.),
    'wyvern_turboprop':dict(aircraft=['wyvern_s4'],speed_min_kmh=150.,speed_max_kmh=1300.),
    'draken_delta':dict(aircraft=['saab_j35d'],speed_min_kmh=150.,speed_max_kmh=1300.),
    'f5e_seam':dict(aircraft=['f_5e'],speed_min_kmh=150.,speed_max_kmh=1300.),
    'f16xl_branch':dict(aircraft=['f_16xl'],speed_min_kmh=150.,speed_max_kmh=450.),
    'f104c_new':dict(aircraft=['f_104c'],speed_min_kmh=200.,speed_max_kmh=1300.),
    'tornado_new':dict(aircraft=['tornado_gr1'],speed_min_kmh=250.,speed_max_kmh=1300.),
    'fa18e_instructor':dict(aircraft=['fa_18e_block_2'],speed_min_kmh=300.,speed_max_kmh=1100.,instructor=True),
    'p38j_new':dict(aircraft=['p-38j'],speed_min_kmh=300.,speed_max_kmh=525.),
    'yak3_new':dict(aircraft=['yak-3_france'],speed_min_kmh=150.,speed_max_kmh=700.),
    'i153_biplane':dict(aircraft=['i-153_m62_zhukovskiy'],speed_min_kmh=100.,speed_max_kmh=1300.),
    'la5_radial':dict(aircraft=['la-5fn'],speed_min_kmh=100.,speed_max_kmh=1300.),
}
DEFAULT_CASES=['ki61_reported','p47_radial','mosquito_twin','wyvern_turboprop','draken_delta']


def benchmark(name,save_dir=None):
    # Match the application's Quick setting and its actual 601-column raster.
    preparing=time.monotonic()
    # Keep historical benchmark conditions independent of the UI's RB default.
    config=settings(dict({'instructor':False,**CASES[name]},
                         speed_samples=9,load_samples=7,sep_tolerance_mps=1.,surface_resolution=601))
    preparation=time.monotonic()-preparing
    progress_state={'phase':None,'last':0.}
    def report(item):
        now=time.monotonic();phase=item.get('phase')
        if phase!=progress_state['phase'] or now-progress_state['last']>10.:
            print('  progress',name,phase,item.get('done'),item.get('total'),
                  round(item.get('elapsed_s',0.),1),'s',flush=True)
            progress_state.update(phase=phase,last=now)
    start=time.monotonic();data=compute(config,progress=report);calculation=time.monotonic()-start
    aircraft=data['aircraft'][0];columns=aircraft['columns']
    if save_dir:
        path=Path(save_dir)/f'{name}.json';path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(json.dumps(data,separators=(',',':'),allow_nan=False))
    start=time.monotonic();enrich(data);render=time.monotonic()-start
    surface=aircraft['surface'];xs=np.asarray(surface['x']);z=np.asarray(surface['z'],dtype=float)
    verified=[c['speed_kmh'] for c in columns if c['boundary_status'] in ('verified limit','plot ceiling')]
    interior=(xs>min(verified))&(xs<max(verified)) if verified else np.zeros(len(xs),dtype=bool)
    blank=np.where(interior&~np.isfinite(z).any(axis=0))[0]
    gaps=[g for c in columns for g in c['numerical_gap_brackets']]
    # An interior-only count can report zero while the high-speed tail is
    # entirely missing. Report the sampled endpoint and trailing coverage too.
    endpoint=max(c['speed_kmh'] for c in columns)
    last_verified=max(verified) if verified else None
    endpoint_column=max(columns,key=lambda c:c['speed_kmh'])
    result=dict(case=name,aircraft=aircraft['id'],backend=BACKEND,settings=config,
        catalog_preparation_s=round(preparation,3),calculation_s=round(calculation,3),
        rendering_s=round(render,3),total_s=round(calculation+render,3),
        meets_20s_target=calculation+render<=20.,meets_60s_limit=calculation+render<60.,
        speed_columns=len(columns),boundary_statuses=dict(Counter(c['boundary_status'] for c in columns)),
        unresolved_speed_intervals=len(aircraft['interpolation']['unresolved_speed_intervals']),
        unresolved_load_intervals=sum(len(c['unresolved_load_intervals']) for c in columns),
        numerical_gap_intervals=len(gaps),max_gap_load_g=max((g['width_g'] for g in gaps),default=0.),
        rendered_speed_columns=len(xs),blank_rendered_columns_inside_verified_range=len(blank),
        blank_speeds_kmh=[float(xs[i]) for i in blank],valid_points=aircraft['valid_points'])
    result.update(sampled_endpoint_kmh=endpoint,last_verified_speed_kmh=last_verified,
        unverified_high_speed_span_kmh=endpoint-last_verified if last_verified is not None else None,
        endpoint_boundary_status=endpoint_column['boundary_status'],
        endpoint_valid_points=sum(p['valid'] for p in endpoint_column['points']))
    result['gap_reasons']=dict(Counter(reason for g in gaps for reason in g.get('reasons',[])))
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cases',default=','.join(DEFAULT_CASES))
    parser.add_argument('--output',default='analysis/em-global-benchmark.json')
    parser.add_argument('--save-data-dir')
    args=parser.parse_args()
    names=args.cases.split(',')
    if set(names)-set(CASES):parser.error('Unknown cases: '+','.join(sorted(set(names)-set(CASES))))
    rows=[]
    for name in names:
        print('Running',name,flush=True)
        row=benchmark(name,args.save_data_dir)
        rows.append(row)
        print(json.dumps(row),flush=True)
        path=Path(args.output);path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(json.dumps(dict(cases=rows),indent=2)+'\n')
    assert all(row['meets_60s_limit'] for row in rows),'An EM plot exceeded the 60-second limit; see the saved report'


if __name__=='__main__':main()
