"""Exact column interpolation, continuity across speed knots, and gap masks."""
import argparse
import json
from pathlib import Path

import numpy as np

from em_sampling import speed_interpolate
from em_surface import eligible


ROOT=Path(__file__).resolve().parents[1]
DEFAULTS=[
    'outputs/em/e1ed839357745d261eee/data.json',
    'outputs/em/772cc8b510d1b5d37f0e/data.json',
    'outputs/em/024c2105a62d19bb873b/data.json',
    'analysis/em-global-september22/rpm-fix/refined-data/mosquito_twin.json',
    'analysis/em-global-september22/rpm-fix/refined-data/wyvern_turboprop.json',
    'analysis/em-global-september22/rpm-fix/refined-data/draken_delta.json',
]


def gaps():
    columns=[]
    for speed in [200.,250.,300.,350.]:
        points=[dict(load_g=n,ps_mps=20.-n*n-speed*.04,alpha_deg=n**.5,
                     valid=n!=3.,force_error_g=0.) for n in [1.,2.,3.,4.,5.,6.]]
        columns.append(dict(speed_kmh=speed,points=points,boundary=points[-1],
                            lower_boundary=points[0],boundary_status='verified limit',
                            boundary_reason='stall'))
    loads=np.array([1.,1.5,2.,2.1,3.,3.9,4.,4.5,6.])
    actual=speed_interpolate(columns,[225.,250.,275.,300.,325.],loads)
    assert not np.isfinite(actual[(loads>2.)&(loads<4.)]).any(),'Rejected load gap was bridged'
    assert np.isfinite(actual[(loads<=2.)|(loads>=4.)]).all(),'Valid runs disappeared'


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('data',nargs='*')
    ap.add_argument('--report',default='analysis/em-contour-kinks-september22/continuity-validation.json')
    args=ap.parse_args();rows=[]
    for filename in args.data or DEFAULTS:
        data=json.loads((ROOT/filename).read_text())
        for aircraft in data['aircraft']:
            columns=aircraft['columns'];worst=0.;raw_error=0.;count=0;corners=[]
            for column in columns[1:-1]:
                if not eligible(column):continue
                lo=column['lower_boundary']['load_g'];hi=column['boundary']['load_g']
                if hi-lo<.01:continue
                loads=lo+(hi-lo)*np.array([.02,.1,.3,.5,.7,.9,.98])
                speed=column['speed_kmh'];eps=1e-7
                actual=speed_interpolate(columns,[speed-eps,speed,speed+eps],loads)
                good=np.isfinite(actual).all(axis=1)
                if good.any():
                    jump=float(np.max(abs(actual[good][:,[0,2]]-actual[good,1,None])))
                    if jump>=1e-5:
                        # Check continuity by convergence to the knot. Both
                        # a lift corner and narrowly bracketed native switches
                        # can have large slopes; a fixed-distance difference
                        # alone would impose an arbitrary derivative bound.
                        differences=[jump]
                        for delta in (1e-9,1e-11):
                            near=speed_interpolate(columns,[speed-delta,speed,speed+delta],loads)
                            common=good & np.isfinite(near).all(axis=1)
                            assert common.any(),(filename,speed,'no corner support')
                            differences.append(float(np.max(abs(near[common][:,[0,2]]-near[common,1,None]))))
                        assert differences[-1]<1e-5 and differences[-1]<differences[0]*.05,(filename,speed,differences)
                        corners.append(dict(speed_kmh=speed,steps_kmh=[1e-7,1e-9,1e-11],differences_mps=differences))
                    else:assert jump<1e-5,(filename,speed,jump)
                    worst=max(worst,jump);count+=int(good.sum())
                points=[p for p in column['points'] if p['valid'] and p.get('surface_sample',True)
                        and lo<=p['load_g']<=hi]
                values=speed_interpolate(columns,[speed],[p['load_g'] for p in points])[:,0]
                error=float(np.max(abs(values-[p['ps_mps'] for p in points])))
                assert np.isfinite(values).all() and error<1e-8,(filename,speed,error)
                raw_error=max(raw_error,error)
            row=dict(aircraft=aircraft['name'],data=filename,continuity_probes=count,
                     max_sided_difference_mps=worst,max_solved_sample_error_mps=raw_error,corner_limits=corners)
            rows.append(row);print(row,flush=True)
    gaps()
    Path(args.report).write_text(json.dumps(dict(status='PASS',cases=rows,
        rejected_load_gap_preserved=True),indent=2)+'\n')


if __name__=='__main__':main()
