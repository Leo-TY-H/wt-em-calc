"""Compare chart geometry when parallel adaptive sampling chooses new knots."""
import argparse
import gzip
import json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree


def cloud(paths,scale):
    """Sample existing segments only; never bridge a masked gap.

    Spacing <=0.1 tolerance units bounds nearest-sample error by 0.05.
    """
    chunks=[]
    for path in paths:
        vertices=np.asarray(path,dtype=float)/scale
        if not len(vertices):continue
        chunks.append(vertices)
        for a,b in zip(vertices,vertices[1:]):
            length=np.max(abs(b-a))
            if not np.isfinite(length):continue
            n=max(1,int(np.ceil(length/.1)))
            if n>1:chunks.append(a+(b-a)*np.arange(1,n)[:,None]/n)
    return np.concatenate(chunks) if chunks else np.empty((0,2))


def distance(paths_a,paths_b,scale):
    a,b=cloud(paths_a,scale),cloud(paths_b,scale)
    if not len(a) or not len(b):return 0. if len(a)==len(b) else None
    return float(max(cKDTree(b).query(a,p=np.inf)[0].max(),cKDTree(a).query(b,p=np.inf)[0].max()))


def outline(points):
    paths=[];current=[]
    for point in points:
        if point['turn_dps'] is None:
            if current:paths.append(current);current=[]
        else:current.append([point['speed_kmh'],point['turn_dps']])
    if current:paths.append(current)
    return paths


def compare(a,b):
    metadata=a['interpolation']
    scale=np.array([metadata['target_speed_kmh'],metadata['target_contour_dps']])
    levels=sorted({c['level'] for result in (a,b) for c in result['contours']})
    contours={str(level):distance(
        [list(zip(c['x'],c['y'])) for c in a['contours'] if c['level']==level],
        [list(zip(c['x'],c['y'])) for c in b['contours'] if c['level']==level],scale) for level in levels}
    sa,sb=a['surface'],b['surface']
    xs,ia,ib=np.intersect1d(sa['x'],sb['x'],return_indices=True)
    za,zb=np.asarray(sa['z'],dtype=float)[:,ia],np.asarray(sb['z'],dtype=float)[:,ib]
    changed=np.any(np.isfinite(za)!=np.isfinite(zb),axis=0)
    common=np.isfinite(za)&np.isfinite(zb)
    return dict(id=a['id'],tolerances=dict(speed_kmh=float(scale[0]),turn_dps=float(scale[1])),
        boundary_distance_in_tolerance_units=distance(outline(a['boundary']),outline(b['boundary']),scale),
        contour_distances_in_tolerance_units=contours,
        distance_sampling_uncertainty_units=.1,
        common_speed_knots=len(xs),mask_difference_speeds_kmh=xs[changed].tolist(),
        max_fraction_grid_sep_difference_mps=float(np.max(abs(za[common]-zb[common]))) if np.any(common) else None,
        note='Distances use the existing speed/turn display tolerances. Independent approximations can differ by two tolerance units. SEP at equal surface fractions is diagnostic, since physical loads can differ.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('first',type=Path);parser.add_argument('second',type=Path)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    with gzip.open(args.first,'rt') as f:a=json.load(f)
    with gzip.open(args.second,'rt') as f:b=json.load(f)
    results=[compare(x,next(y for y in b if y['id']==x['id'])) for x in a]
    text=json.dumps(results,indent=2)
    if args.output:args.output.write_text(text+'\n')
    print(text)


if __name__=='__main__':main()
