"""Check the rendered contour endpoints against the actual mesh edges."""
import argparse,json
from pathlib import Path
import numpy as np
from em_plot import enrich,contour_levels


def check(aircraft):
    surface=aircraft['surface'];xs=np.array(surface['x']);ys=np.array(surface['turn_dps'])
    z=np.array(surface['z'],dtype=float);paths=aircraft['contours'];checked=0;failures=[]
    # Contourpy intersects each mesh edge linearly. That intersection must
    # appear as a path endpoint, including Ps=0 and the uppermost mesh row.
    for side,row in [('lower',0),('upper',-1)]:
        for level in contour_levels(aircraft):
            ends=np.array([[p['x'][i],p['y'][i]] for p in paths if p['level']==level for i in (0,-1)])
            indices=np.flatnonzero(np.isfinite(z[row,:-1])&np.isfinite(z[row,1:])&
                ((z[row,:-1]-level)*(z[row,1:]-level)<0.))
            for j in indices:
                fraction=(level-z[row,j])/(z[row,j+1]-z[row,j])
                expected=np.array([xs[j]+fraction*(xs[j+1]-xs[j]),ys[row,j]+fraction*(ys[row,j+1]-ys[row,j])])
                error=float(np.min(np.max(abs(ends-expected),axis=1))) if len(ends) else None
                checked+=1
                if error is None or error>1e-7:failures.append(dict(side=side,level=level,expected=expected.tolist(),error=error))
    # Upper edge coverage is counted separately; a mask must not silently
    # defeat the intersection check by removing both edge values.
    outline={p['speed_kmh']:p for p in aircraft['boundary'] if not p.get('vertical_edge')}
    visible=np.array([outline.get(float(x),{}).get('turn_dps') is not None for x in xs])
    declared=np.zeros(len(xs),dtype=bool)
    solved=np.isin(xs,[c['speed_kmh'] for c in aircraft.get('columns',[])])
    certificates={tuple(c['speed_interval_kmh']):c for c in aircraft.get('interpolation',{}).get('certified_speed_interiors',[])}
    for lo,hi in aircraft.get('interpolation',{}).get('unresolved_speed_intervals',[]):
        mask=(xs>lo)&(xs<hi)&~solved
        assert mask.any(),('Unresolved speed interval lacks a mesh separator',lo,hi)
        certificate=certificates.get((lo,hi))
        if certificate:
            loads=np.asarray(surface['load_g'])[:,mask];bottom,top=certificate['load_interval_g']
            outside=(loads<bottom)|(loads>top)
            assert not np.isfinite(z[:,mask][outside]).any(),('Unverified load band was painted',lo,hi)
            assert np.isfinite(z[:,mask][~outside]).all(),('Certified common interior was erased',lo,hi)
        else:assert not np.isfinite(z[:,mask]).any(),('Unverified speed interval was painted',lo,hi)
        declared|=mask
    missing=np.flatnonzero(visible&(ys[-1]>ys[0]+1e-8)&~np.isfinite(z[-1])&~declared)
    return dict(aircraft=aircraft['id'],checked_intersections=checked,failures=failures,
        declared_masked_speed_vertices=int(declared.sum()),
        missing_upper_edge_vertices=len(missing),missing_upper_edge_speeds_kmh=xs[missing].tolist())


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('data',nargs='+');parser.add_argument('--report',required=True)
    args=parser.parse_args();rows=[]
    for filename in args.data:
        data=json.loads(Path(filename).read_text());enrich(data)
        rows.extend(dict(check(a),source=filename) for a in data['aircraft'])
    Path(args.report).write_text(json.dumps(rows,indent=2)+'\n')
    print([(r['aircraft'],r['checked_intersections'],len(r['failures']),r['missing_upper_edge_vertices']) for r in rows])
    assert all(not r['failures'] and not r['missing_upper_edge_vertices'] for r in rows)


if __name__=='__main__':main()
