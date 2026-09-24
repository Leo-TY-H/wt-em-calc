"""Finite-time terminal vertical arcs, including negative-SEP zoom climbs."""
import numpy as np


def step(field,v,h,dh,g):
    # dh/dt=V and dV/dh=g/V * (Ps/V - 1): no instantaneous
    # constant-energy transfer, and Ps is allowed to become negative.
    def derivative(vv,hh):
        ps=field.sample(vv,hh)
        return g/np.maximum(vv,1.)*(ps/np.maximum(vv,1.)-1.),1/np.maximum(vv,1.)
    a,ta=derivative(v,h);b,tb=derivative(v+a*dh/2,h+dh/2)
    c,tc=derivative(v+b*dh/2,h+dh/2);d,td=derivative(v+c*dh,h+dh)
    end=v+dh*(a+2*b+2*c+d)/6;dt=dh*(ta+2*tb+2*tc+td)/6
    clear=field.clear_edges(v,np.full_like(v,h),end,np.full_like(v,h+dh),positive_only=False)
    valid=np.isfinite(end)&(end>0)&clear&np.isfinite(field.sample(end,h+dh))
    return np.where(valid,end,np.nan),dt


def terminal(field,heights,speed,distance,previous,start_index,base,g):
    """Compare finite-time zooms from graph states and the refined route.

    Subsample graph origins (not trajectory integration) to bound interactive
    work. All actual arcs use every intervening surface altitude interval.
    """
    n=len(speed);seeds={}
    for row in range(0,len(heights)-1,4):
        for col in range(0,n,4):
            index=row*n+col
            if np.isfinite(distance[index]):
                seeds.setdefault(float(heights[row]),[]).append((speed[col],distance[index],('graph',index)))
    start_v=speed[start_index]
    seeds.setdefault(float(heights[0]),[]).append((start_v,0.,('graph',start_index)))
    for i,p in enumerate(base or []):
        if p['altitude_m']<heights[-1]:
            seeds.setdefault(p['altitude_m'],[]).append((p['speed_kmh']/3.6,p['elapsed_s'],('base',i)))
    levels=np.unique(np.r_[heights,list(seeds)])
    vv=np.array([],dtype=float);tt=np.array([],dtype=float);origins=[]
    best=base[-1]['elapsed_s'] if base else np.inf
    for h,h1 in zip(levels,levels[1:]):
        additions=seeds.get(float(h),[])
        if additions:
            vv=np.r_[vv,[s[0] for s in additions]];tt=np.r_[tt,[s[1] for s in additions]]
            origins.extend((h,*s) for s in additions)
        vv,dt=step(field,vv,h,h1-h,g);tt+=dt
        keep=np.isfinite(vv)&(tt<best)
        vv=vv[keep];tt=tt[keep];origins=[o for o,k in zip(origins,keep) if k]
    if not len(tt):return None
    winner=int(np.argmin(tt));h0,v0,t0,origin=origins[winner]
    if tt[winner]>=best-1e-6:return None
    if origin[0]=='base':points=list(base[:origin[1]+1])
    else:
        indices=[];index=origin[1]
        while index>=0:
            indices.append(index)
            if index==start_index:break
            index=int(previous[index])
        indices.reverse();points=[]
        for index in indices:
            row,col=divmod(index,n);v=speed[col];h=heights[row]
            points.append(dict(speed_kmh=float(v*3.6),altitude_m=float(h),elapsed_s=float(distance[index]),
                               sep_mps=float(field.sample(v,h)),energy_height_m=float(h+v*v/(2*g))))
    v=np.array([v0]);t=t0
    path_heights=np.unique(np.r_[h0,field.h[(field.h>h0)&(field.h<heights[-1])],heights[-1]])
    for h,h1 in zip(path_heights,path_heights[1:]):
        v,dt=step(field,v,h,h1-h,g);t+=float(dt[0]);speed_value=float(v[0])
        if not np.isfinite(speed_value):return None
        points.append(dict(speed_kmh=speed_value*3.6,altitude_m=float(h1),elapsed_s=t,
                           sep_mps=float(field.sample(speed_value,h1)),energy_height_m=float(h1+speed_value**2/(2*g)),
                           phase='terminal zoom'))
    return points
