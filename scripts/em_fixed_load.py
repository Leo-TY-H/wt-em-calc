"""Batched PCHIP with the same contiguous-valid-run masks as scalar rows."""
import numpy as np
from scipy.interpolate import PchipInterpolator


def interpolate(columns,speeds,loads):
    from em_sampling import column_at_load
    speeds=np.atleast_1d(speeds);loads=np.atleast_1d(loads)
    values=np.array([column_at_load(c,loads) for c in columns]).T
    xs=np.array([c['speed_kmh'] for c in columns]);out=np.full((len(loads),len(speeds)),np.nan)
    masks=np.isfinite(values)
    for mask in np.unique(masks,axis=0):
        rows=np.flatnonzero(np.all(masks==mask,axis=1));indices=np.flatnonzero(mask)
        for run in np.split(indices,np.where(np.diff(indices)>1)[0]+1):
            if len(run)<2:continue
            query=np.flatnonzero((speeds>=xs[run[0]])&(speeds<=xs[run[-1]]))
            if len(query):
                out[np.ix_(rows,query)]=PchipInterpolator(xs[run],values[np.ix_(rows,run)],axis=1)(speeds[query])
    indices=np.searchsorted(xs,speeds);exact=indices<len(xs)
    exact[exact]=xs[indices[exact]]==speeds[exact]
    out[:,exact]=values[:,indices[exact]]
    return out
