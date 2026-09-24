"""Continuous speed refinement of a feasible discrete energy-climb route."""
import time
import numpy as np
from scipy.optimize import minimize


def refine(field,points,bounds,g):
    v=np.array([p['speed_kmh']/3.6 for p in points]);h=np.array([p['altitude_m'] for p in points])
    keep=[0]
    for i in range(1,len(v)-1):
        cross=(v[i]-v[keep[-1]])*(h[i+1]-h[i])-(v[i+1]-v[i])*(h[i]-h[keep[-1]])
        if abs(cross)>1e-7:keep.append(i)
    keep.append(len(v)-1);v=v[keep];h=h[keep]
    # Bound interactive work on a highly fragmented or very large surface.
    if len(v)>40 or len(v)<2:return None
    left=[];right=[];f0=[];f1=[]
    for i in range(len(h)-1):
        if h[i+1]>h[i]:
            fractions=(np.r_[h[i],field.h[(field.h>h[i])&(field.h<h[i+1])],h[i+1]]-h[i])/(h[i+1]-h[i])
        else:fractions=np.linspace(0,1,33)
        for a,b in zip(fractions,fractions[1:]):
            left.append(i);right.append(i+1);f0.append(a);f1.append(b)
    left=np.array(left);right=np.array(right);f0=np.array(f0);f1=np.array(f1)
    h0=h[left]+(h[right]-h[left])*f0;h1=h[left]+(h[right]-h[left])*f1;dh=h1-h0
    cache={};started=time.monotonic()
    def values(x):
        if time.monotonic()-started>1.5:raise TimeoutError
        key=x.tobytes()
        if key in cache:return cache[key]
        vs=np.r_[v[0],x];v0=vs[left]+(vs[right]-vs[left])*f0;v1=vs[left]+(vs[right]-vs[left])*f1;dv=v1-v0
        dt=np.zeros(len(v0));constraints=[]
        for f,weight in ((0.,1.),(.5,4.),(1.,1.)):
            vv=v0+dv*f;hh=h0+dh*f;ps=field.sample(vv,hh);de=dh+vv*dv/g
            constraints.extend([np.nan_to_num(ps,nan=-1e6),de-1e-8,
                                np.nan_to_num(.99999*vv*de-dh*ps,nan=-1e6)])
            dt+=weight*de/np.maximum(np.nan_to_num(ps,nan=1e-3),1e-3)/6
        result=(float(dt.sum()),np.concatenate(constraints),v0,v1,dt)
        cache.clear();cache[key]=result;return result
    try:
        fit=minimize(lambda x:values(x)[0],v[1:],method='SLSQP',bounds=[bounds]*(len(v)-1),
                     constraints=[dict(type='ineq',fun=lambda x:values(x)[1])],
                     options=dict(maxiter=80,ftol=1e-5))
        total,constraints,v0,v1,dt=values(fit.x)
    except (TimeoutError,ValueError,FloatingPointError):return None
    if not fit.success or constraints.min()<-1e-8 or total>=points[-1]['elapsed_s']:return None
    if not field.clear_edges(v0,h0,v1,h1).all():return None
    # Check more locations than the optimizer's quadrature/constraint nodes.
    for f in np.linspace(0,1,17):
        vv=v0+(v1-v0)*f;ps=field.sample(vv,h0+dh*f);de=dh+vv*(v1-v0)/g
        if not (np.isfinite(ps)&(ps>0)&(de>0)&(dh*ps<=vv*de+1e-6)).all():return None
    speed=np.r_[v0[0],v1];altitude=np.r_[h0[0],h1];elapsed=np.r_[0.,np.cumsum(dt)]
    power=field.sample(speed,altitude)
    return [dict(speed_kmh=float(vv*3.6),altitude_m=float(hh),elapsed_s=float(tt),sep_mps=float(pp),
                 energy_height_m=float(hh+vv*vv/(2*g))) for vv,hh,tt,pp in zip(speed,altitude,elapsed,power)]
