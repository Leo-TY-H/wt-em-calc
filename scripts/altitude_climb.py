"""Minimum-time non-descending climb on the checked SEP surface.

Energy approximation: d(h + V²/(2g))/dt = Ps(V,h). Each directed edge
charges for kinetic as well as potential energy and checks 0 <= hdot <= V.
Dijkstra finds the least-time route on this discrete graph. This uses the
level-flight SEP model; pitch transients and climb-angle-dependent lift/drag
are not available from this surface and are not claimed to be simulated.

Reference: NASA TM X-62292, https://ntrs.nasa.gov/citations/19740003707 .
"""
import math
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra
from body_dynamics import G as NATIVE_GRAVITY

G=float(NATIVE_GRAVITY)
MODEL='Minimum-time energy-model estimate; non-descending powered climb and finite-time terminal zoom, free arrival speed. Level-flight SEP; no pitch/turn transients or climb-angle drag correction.'


class Surface:
    def __init__(self,data):
        surface=data['surface']
        self.h=np.asarray(surface['altitudes_m'],dtype=float)
        self.x=np.asarray(surface['speeds_kmh'],dtype=float)/3.6
        self.z=np.asarray(surface['sep_mps'],dtype=float)
        self.mapping=surface.get('coordinate')
        # Recover the common speed coordinate without importing the FM or
        # requiring a particular atmosphere. x(h,j)=scale(h)*coordinate(j).
        self.scale=self.x[:,0]
        self.axis=np.asarray(self.mapping['axis']) if self.mapping else self.x[0]/self.scale[0]
        invalid=(~np.isfinite(self.z))|(self.z<=0)
        self.bad=np.pad(invalid.astype(int),((1,0),(1,0))).cumsum(0).cumsum(1)
        self.missing=np.pad((~np.isfinite(self.z)).astype(int),((1,0),(1,0))).cumsum(0).cumsum(1)

    def coordinate(self,v,h):
        if self.mapping:
            if self.mapping.get('kind')=='normalized':
                left=np.interp(h,self.h,np.asarray(self.mapping['left_speed_kmh'])/3.6)
                right=np.interp(h,self.h,np.asarray(self.mapping['right_speed_kmh'])/3.6)
                return (np.asarray(v)-left)/(right-left)
            left=np.interp(h,self.h,np.asarray(self.mapping['edge_speed_kmh'])/3.6)
            anchor=np.interp(h,self.h,np.asarray(self.mapping['anchor_speed_kmh'])/3.6)
            sound=np.interp(h,self.h,np.asarray(self.mapping['sound_kmh'])/3.6)
            return np.where(v<anchor,(v-left)/(anchor-left),1+(v-anchor)/sound)
        return np.asarray(v)/np.interp(h,self.h,self.scale)

    def sample(self,v,h):
        v,h=np.broadcast_arrays(np.asarray(v,dtype=float),np.asarray(h,dtype=float))
        m=self.coordinate(v,h)
        j=np.clip(np.searchsorted(self.axis,m,side='right')-1,0,len(self.axis)-2)
        i=np.clip(np.searchsorted(self.h,h,side='right')-1,0,len(self.h)-2)
        u=(m-self.axis[j])/(self.axis[j+1]-self.axis[j]);w=(h-self.h[i])/(self.h[i+1]-self.h[i])
        total=np.zeros_like(v);valid=(u>=-1e-10)&(u<=1+1e-10)&(w>=-1e-10)&(w<=1+1e-10)
        for di,dj,weight in ((0,0,(1-u)*(1-w)),(0,1,u*(1-w)),(1,0,(1-u)*w),(1,1,u*w)):
            value=self.z[i+di,j+dj]
            valid &= np.isfinite(value)|(np.abs(weight)<1e-12)
            total+=weight*np.where(np.isfinite(value),value,0.)
        return np.where(valid,total,np.nan)

    def clear_edges(self,v0,h0,v1,h1,positive_only=True):
        """Conservative whole-cell check: never jump an unsampled hole.

        Check every surface vertex in each edge's bounding cell rectangle,
        not just its quadrature points. Both coordinates are monotone inside
        one altitude interval of the rendered surface.
        """
        m0=self.coordinate(v0,h0);m1=self.coordinate(v1,h1)
        left=np.clip(np.searchsorted(self.axis,np.minimum(m0,m1)+1e-12,side='right')-1,0,len(self.axis)-1)
        right=np.clip(np.searchsorted(self.axis,np.maximum(m0,m1)-1e-12,side='left'),0,len(self.axis)-1)
        low=np.clip(np.searchsorted(self.h,h0+1e-8,side='right')-1,0,len(self.h)-1)
        high=np.clip(np.searchsorted(self.h,h1-1e-8,side='left'),0,len(self.h)-1)
        mask=self.bad if positive_only else self.missing
        count=mask[high+1,right+1]-mask[low,right+1]-mask[high+1,left]+mask[low,left]
        return count==0


def options(data,values=None):
    values={} if values is None else values
    if not isinstance(values,dict) or set(values)-{'start_altitude_m','start_speed_kmh','target_altitude_m'}:
        raise ValueError('Unknown climb-route setting')
    c=data['settings'];out=dict(start_altitude_m=c['altitude_min_m'],target_altitude_m=c['altitude_max_m'],start_speed_kmh=None)
    out.update(values)
    for key,lo,hi in [('start_altitude_m',c['altitude_min_m'],c['altitude_max_m']),
                      ('target_altitude_m',c['altitude_min_m'],c['altitude_max_m']),
                      ('start_speed_kmh',c['speed_min_kmh'],c['speed_max_kmh'])]:
        v=out[key]
        if key=='start_speed_kmh' and v is None:continue
        if isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or not lo<=v<=hi:
            raise ValueError(f'{key} must be between {lo:g} and {hi:g}')
        out[key]=float(v)
    if out['target_altitude_m']<=out['start_altitude_m']:
        raise ValueError('Target altitude must be above start altitude')
    return out


def climb_route(data,values=None,speed_nodes=241,refine=True):
    config=options(data,values);surface=data.get('surface')
    result=dict(status='unavailable',settings=config,model=MODEL,points=[],elapsed_s=None)
    if not surface:
        return dict(result,reason='Recalculate the plot to build a climb route.')
    field=Surface(data);c=data['settings'];h0=config['start_altitude_m'];hf=config['target_altitude_m']
    speed=np.linspace(c['speed_min_kmh']/3.6,c['speed_max_kmh']/3.6,speed_nodes)
    if config['start_speed_kmh'] is None:
        power=field.sample(speed,np.full_like(speed,h0))
        if not np.any(np.isfinite(power)):
            return dict(result,reason='No checked starting point at this altitude.')
        start=float(speed[np.nanargmax(power)])
    else:start=config['start_speed_kmh']/3.6
    result['start_speed_kmh']=start*3.6
    if not np.isfinite(field.sample(start,h0)):
        return dict(result,reason='Starting point is outside the checked surface. Choose another speed or altitude.')
    speed=np.unique(np.r_[speed,start]);heights=np.unique(np.r_[h0,field.h[(field.h>h0)&(field.h<hf)],hf])
    n=len(speed);v,h=np.meshgrid(speed,heights);node=np.arange(v.size).reshape(v.shape)
    power=field.sample(v,h);good=np.isfinite(power)&(power>0)
    sources=[];targets=[];costs=[]
    # Adjacent altitude rows plus level acceleration. A modest speed stencil
    # resolves acceleration/climb trade-offs without a costly physics rerun.
    offsets=sorted(set(range(-4,5))|{-32,-24,-16,-12,-8,8,12,16,24,32})
    for dy,dx in [(0,1)]+[(1,dx) for dx in offsets]:
        ya=slice(0,len(heights)-dy);yb=slice(dy,len(heights))
        xa=slice(max(0,-dx),min(n,n-dx));xb=slice(max(0,dx),min(n,n+dx))
        a=node[ya,xa].ravel();b=node[yb,xb].ravel()
        valid=good.ravel()[a]&good.ravel()[b]
        a=a[valid];b=b[valid]
        v0=v.ravel()[a];v1=v.ravel()[b];h0s=h.ravel()[a];h1=h.ravel()[b]
        clear=field.clear_edges(v0,h0s,v1,h1)
        a=a[clear];b=b[clear];v0=v0[clear];v1=v1[clear];h0s=h0s[clear];h1=h1[clear]
        dv=v1-v0;dh=h1-h0s;dt=np.zeros(len(a));valid=np.ones(len(a),dtype=bool)
        for f,weight in ((0.,1.),(.25,4.),(.5,2.),(.75,4.),(1.,1.)):
            vv=v0+dv*f;hh=h0s+dh*f;ps=field.sample(vv,hh)
            de=dh+vv*dv/G
            valid &= np.isfinite(ps)&(ps>0)&(de>0)&(dh*ps<=vv*de+1e-8)
            dt+=weight*de/np.where(ps>0,ps,1.)/12
        valid &= np.isfinite(dt)&(dt>0)
        sources.append(a[valid]);targets.append(b[valid]);costs.append(dt[valid])
    graph=coo_matrix((np.concatenate(costs),(np.concatenate(sources),np.concatenate(targets))),shape=(v.size,v.size)).tocsr()
    start_index=int(np.searchsorted(speed,start))
    distance,previous=dijkstra(graph,directed=True,indices=start_index,return_predecessors=True)
    final=node[-1];target=int(final[np.argmin(distance[final])])
    route=[];index=target if np.isfinite(distance[target]) else -1
    while index>=0:
        route.append(index)
        if index==start_index:break
        index=int(previous[index])
    route.reverse()
    points=[dict(speed_kmh=float(v.ravel()[i]*3.6),altitude_m=float(h.ravel()[i]),
                 elapsed_s=float(distance[i]),sep_mps=float(power.ravel()[i]),
                 energy_height_m=float(h.ravel()[i]+v.ravel()[i]**2/(2*G))) for i in route]
    improved=None
    if refine and points:
        from altitude_climb_refine import refine as refine_route
        improved=refine_route(field,points,(speed[0],speed[-1]),G)
        if improved is not None:points=improved
    from altitude_climb_zoom import terminal
    zoom=terminal(field,heights,speed,distance,previous,start_index,points,G)
    if zoom is not None:points=zoom
    if not points:
        return dict(result,reason='No continuous climb or terminal zoom reaches the target inside this checked plot. Try a lower target or a wider speed range.')
    points=[dict(p,phase=p.get('phase','powered climb')) for p in points]
    return dict(result,status='complete',points=points,elapsed_s=points[-1]['elapsed_s'],
                end_speed_kmh=points[-1]['speed_kmh'],resolution=dict(speed_nodes=n,altitude_rows=len(heights)),
                refined=bool(refine and improved is not None),
                terminal_zoom=zoom is not None,
                method='Shortest-time graph plus speed refinement and finite-time terminal zooms; checked energy balance and vertical-speed <= TAS.')
