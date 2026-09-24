"""Continuous best-SEP speed schedule across the requested altitude range.

SEP is energy gain rate, not altitude gain rate. Join high-SEP operating
points through checked surface cells. This schedule has no clock, initial
speed or acceleration dynamics; it is not a flyable trajectory claim.
"""
import numpy as np
from altitude_climb import Surface,G

MODEL='Continuous best-SEP speed schedule over altitude; no time or ceiling objective. Uses level-flight SEP. Transitions, acceleration and pitch/climb-angle drag are not simulated.'


def energy_guide(data):
    result=dict(status='unavailable',objective='continuous_maximum_sep',model=MODEL,points=[])
    if not data.get('surface'):
        return dict(result,reason='Recalculate the plot to build the maximum-SEP guide.')
    field=Surface(data);rows=[];scores=[];parents=[]
    if abs(field.h[0]-data['settings']['altitude_min_m'])>.01 or abs(field.h[-1]-data['settings']['altitude_max_m'])>.01:
        return dict(result,reason='The checked surface does not span the full requested altitude range.')
    # Include every local maximum, both sides of every gap, and regular
    # connecting vertices. The candidate graph only uses whole checked cells.
    for z in field.z:
        good=np.isfinite(z)
        if not good.any():return dict(result,reason='No continuous guide spans this range: an altitude row has no checked SEP. Reduce the range or improve sampling.')
        left=np.r_[-np.inf,np.where(good,z,-np.inf)[:-1]]
        right=np.r_[np.where(good,z,-np.inf)[1:],-np.inf]
        peaks=good&(z>=left)&(z>=right)&((z>left)|(z>right))
        edges=good&(~np.r_[False,good[:-1]]|~np.r_[good[1:],False])
        regular=np.zeros(len(z),dtype=bool);regular[np.linspace(0,len(z)-1,49).astype(int)]=True
        rows.append(np.flatnonzero(good&(peaks|edges|regular)))
    scores.append(np.zeros(len(rows[0])))
    for i in range(1,len(rows)):
        a,b=np.meshgrid(rows[i-1],rows[i],indexing='ij')
        v0=field.x[i-1,a];v1=field.x[i,b]
        clear=field.clear_edges(v0,field.h[i-1],v1,field.h[i],positive_only=False)
        reward=.5*(field.z[i-1,a]+field.z[i,b])*(field.h[i]-field.h[i-1])
        # A negligible tie break keeps flat plateaus at a stable speed.
        value=scores[-1][:,None]+reward-1e-10*(v1-v0)**2
        value=np.where(clear,value,-np.inf)
        parent=np.argmax(value,axis=0);best=value[parent,np.arange(len(rows[i]))]
        if not np.isfinite(best).any():return dict(result,reason='No continuous guide spans the checked surface. Unresolved or excluded regions block the connection; reduce the range or improve sampling.')
        parents.append(parent);scores.append(best)
    selected=[int(np.argmax(scores[-1]))]
    for parent in reversed(parents):selected.append(int(parent[selected[-1]]))
    selected.reverse();points=[]
    for i,k in enumerate(selected):
        j=rows[i][k];ps=float(field.z[i,j])
        points.append(dict(speed_kmh=float(field.x[i,j]*3.6),altitude_m=float(field.h[i]),sep_mps=ps,
                           specific_power_w_kg=ps*G,connected_from_previous=i>0))
    # Drape the connecting guide over the field instead of drawing a 3D
    # chord through the surface when the preferred speed changes rapidly.
    knots=points;points=[knots[0]]
    for a,b in zip(knots,knots[1:]):
        steps=max(1,int(np.ceil(abs(b['speed_kmh']-a['speed_kmh'])/5)),
                  int(np.ceil((b['altitude_m']-a['altitude_m'])/25)))
        f=np.linspace(0,1,steps+1)[1:]
        v=(a['speed_kmh']+(b['speed_kmh']-a['speed_kmh'])*f)/3.6
        h=a['altitude_m']+(b['altitude_m']-a['altitude_m'])*f
        ps=field.sample(v,h)
        if not np.isfinite(ps).all():
            return dict(result,reason='A connecting segment leaves the checked surface; no continuous guide is available at this resolution.')
        points.extend(dict(speed_kmh=float(vv*3.6),altitude_m=float(hh),sep_mps=float(pp),
                           specific_power_w_kg=float(pp*G),connected_from_previous=True)
                      for vv,hh,pp in zip(v,h,ps))
    peak=max(points,key=lambda p:p['sep_mps'])
    return dict(result,status='complete',points=points,peak=peak,
                method='Connected speed schedule maximizing trapezoidal altitude-averaged SEP on a sampled graph. Includes every row-local maximum; all connecting cells must be checked. No claim of instantaneous global optimality between rows or dynamic flyability.')
