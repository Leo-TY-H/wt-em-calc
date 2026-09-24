"""Verify a common interior independently of a discontinuous envelope edge."""
import math
import numpy as np


def values(left,right,speeds,loads):
    """Linear speed interpolation at fixed physical loads, inside both curves."""
    from em_sampling import column_at_load
    speeds=np.asarray(speeds);loads=np.asarray(loads)
    t=(speeds-left['speed_kmh'])/(right['speed_kmh']-left['speed_kmh'])
    a=column_at_load(left,loads.ravel()).reshape(loads.shape)
    b=column_at_load(right,loads.ravel()).reshape(loads.shape)
    return a*(1.-t)+b*t


def check_interior(task):
    """Keep only a contiguous load range passing new force/moment holdouts.

    A failed boundary-interpolation check says nothing about lower turns.
    Two independent interior speeds check a fixed-load interpolant without
    borrowing a slope or a normalized coordinate from the uncertain cap.
    No failed sample is filled or converted into a physical boundary.
    """
    from em_sampling import worker_solver,column_at_load,column_curve
    from em_accuracy import neighboring_loads,turn_tolerance,within_contour_band,visible_error
    name,config_json,_,pair=task
    solver=worker_solver(name,config_json);left,right=pair
    lo,hi=left['speed_kmh'],right['speed_kmh']
    bottom=max(c['lower_boundary']['load_g'] for c in pair)
    top=min(c['boundary']['load_g'] for c in pair)
    # Sparse load probes cannot certify across a known hole in either
    # endpoint curve. Restrict the certificate to their common continuous
    # run before choosing holdouts; rendering uses these same endpoint curves.
    for column in pair:
        prepared=column_curve(column)
        run=next((run for run in prepared[1] if run[0]<=bottom<=run[1]),None) if prepared else None
        if run is None:return None
        top=min(top,run[1])
    if top<=bottom+1e-5:return None
    # Stay just inside the common physical cap, with the usual plotted
    # turn-coordinate tolerance. This is not a new aircraft constraint.
    mid=(lo+hi)*.5
    top=float(neighboring_loads(mid,np.array([top]),.5*turn_tolerance(solver.config['sep_tolerance_mps']))[0,0])
    if top<=bottom+1e-5:return None
    u=np.unique(np.r_[np.linspace(0.,1.,max(9,min(13,solver.config['load_samples']))),.98,.995])
    turns=np.sqrt(bottom*bottom-1.)+u*(math.sqrt(top*top-1.)-math.sqrt(bottom*bottom-1.))
    loads=np.hypot(1.,turns);loads[0]=bottom;loads[-1]=top
    passed=np.ones(len(loads),bool);checks=[];points=[]
    for t in (1./3.,2./3.):
        speed=lo+t*(hi-lo);predicted=values(left,right,speed,loads)
        band=np.clip(neighboring_loads(speed,loads,turn_tolerance(solver.config['sep_tolerance_mps'])),bottom,top)
        nearby=values(left,right,speed,band)
        for j,n in enumerate(loads):
            guesses=[]
            for c in pair:
                good=[p for p in c['points'] if p['valid'] and p['load_g']<=c['boundary']['load_g']]
                good.sort(key=lambda p:p['load_g'])
                guesses.append([float(np.interp(n,[p['load_g'] for p in good],[p['solution'][k] for p in good])) for k in range(5)])
            guess=(np.asarray(guesses[0])*(1-t)+np.asarray(guesses[1])*t).tolist()
            guess[1]=math.degrees(math.acos(1./n))
            point=solver.solve(speed,float(n),guess,exhaustive=False,quick=True)
            if not point['valid'] and not point['converged']:
                # A quick corrector is only a predictor test. An interior
                # numerical failure gets the ordinary solver before it can
                # invalidate the entire common band; physical rejections keep
                # their meaning and are never retried into acceptance here.
                point=solver.solve(speed,float(n),guess,exhaustive=False)
                if not point['valid'] and not point['converged']:
                    point=solver.solve(speed,float(n),exhaustive=False)
            points.append(point)
            error=abs(point['ps_mps']-predicted[j]) if np.isfinite(predicted[j]) else float('inf')
            accurate=(error<=solver.config['sep_tolerance_mps'] or
                bool(within_contour_band(point['ps_mps'],nearby[j],solver.config['sep_tolerance_mps'])) or
                not bool(visible_error(point['ps_mps'],predicted[j])))
            ok=point['valid'] and np.isfinite(predicted[j]) and accurate
            passed[j]&=ok
            checks.append(dict(speed_kmh=speed,load_g=float(n),valid=point['valid'],error_mps=float(error),passed=bool(ok)))
    failed=np.flatnonzero(~passed);end=int(failed[0]) if len(failed) else len(loads)
    if end<5:return None
    return dict(speed_interval_kmh=[lo,hi],load_interval_g=[bottom,float(loads[end-1])],
        method='fixed-load linear speed interpolation with independent one-third/two-thirds aircraft trims',
        checks=checks,points=points)


def plot_values(columns,certificate,speeds,loads):
    pair=[next(c for c in columns if c['speed_kmh']==v) for v in certificate['speed_interval_kmh']]
    predicted=values(*pair,speeds,loads)
    bottom,top=certificate['load_interval_g']
    return np.where((loads>=bottom)&(loads<=top),predicted,np.nan)
