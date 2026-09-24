"""Checked cubic SEP rows: concentrate native solves along one dimension.

The display surface is interpolated from independently checked speed curves.
Additional native solves between altitude rows check the second dimension.
Only the search/interpolation change; every accepted sample still passes the
shared aircraft solver's full equilibrium and physical-limit checks.
"""
import math
import time
from collections import OrderedDict
from concurrent.futures import wait, FIRST_COMPLETED

import numpy as np
from scipy.interpolate import PchipInterpolator,interp1d
from scipy.optimize import brentq
from altitude_envelope import LevelSampler, QUALITY
from air_state import speed_of_sound
from control_mixer import density_at_height

_SAMPLERS = OrderedDict()


def sampler(config):
    import json
    key = json.dumps(config,sort_keys=True)
    if key not in _SAMPLERS:
        _SAMPLERS[key] = LevelSampler(config)
        if len(_SAMPLERS)>2:_SAMPLERS.popitem(last=False)
    _SAMPLERS.move_to_end(key)
    return _SAMPLERS[key]


def row_values(row, mach):
    mach=np.asarray(mach);segments=row['segments']
    if not segments:return np.full(mach.shape,np.nan)
    if '_curves' not in row:
        bounds=np.array([s['bounds'] for s in segments])
        coefficients=np.array([[0.]*(4-len(s['coefficients']))+s['coefficients'] for s in segments])
        row['_curves']=(bounds,coefficients)
    bounds,coefficients=row['_curves']
    index=np.clip(np.searchsorted(bounds[:,0],mach,side='right')-1,0,len(segments)-1)
    x=mach-bounds[index,0];c=coefficients[index]
    return np.where((mach>=bounds[index,0])&(mach<=bounds[index,1]),((c[...,0]*x+c[...,1])*x+c[...,2])*x+c[...,3],np.nan)


def sample_row(task):
    config,height,known = task
    s=sampler(config); sound=float(speed_of_sound(height))*3.6
    tolerance=QUALITY[config['quality']]['tolerance']*.45
    # Tight SEP tolerances also need tighter boundary/curvature resolution.
    # A fixed 1 km/h stop made Detailed abandon otherwise valid intervals.
    min_speed_step={'preview':1.,'smooth':.25,'detailed':.05}[config['quality']]
    max_speed_depth={'preview':9,'smooth':11,'detailed':14}[config['quality']]
    low=config['speed_min_kmh']/sound;high=config['speed_max_kmh']/sound
    nodes=sorted(set(np.linspace(low,high,13).tolist()+
        [m for m in (.6,.75,.85,.9,.95,1.,1.05,1.1,1.2,1.4,1.6,1.8,2.,2.2) if low<m<high]))
    solved={p['speed_kmh']/sound:p for p in known}
    nodes=sorted(set(nodes)|set(solved))
    def at(mach):
        if mach not in solved:solved[mach]=s(mach*sound,height)
        return solved[mach]
    limit=at(high).get('speed_limit')
    if limit:
        edge=(limit['sample_speed_kmh']-1e-7)/sound
        if low<edge<high:nodes=sorted(set(nodes+[edge]))
    for m in reversed(nodes):at(m)
    retried=set()
    def recover(m,left,right):
        p=at(m)
        if (not p['valid'] and p.get('category')=='numerical gap' and
                m not in retried and hasattr(s,'recover') and left['valid'] and right['valid']):
            retried.add(m)
            retry=s.recover(m*sound,height,left,right)
            if retry['valid']:solved[m]=retry
        return solved[m]
    valid=[m for m in nodes if at(m)['valid']]
    for m in nodes:
        if at(m)['valid'] or not valid or not valid[0]<m<valid[-1]:continue
        i=np.searchsorted(valid,m)
        recover(m,at(valid[i-1]),at(valid[i]))
    pending=list(reversed(list(zip(nodes,nodes[1:]))));segments=[];masked=[];blocked=set()
    for depth in range(max_speed_depth):
        keys=sorted(solved);curves={};run=[]
        def commit():
            if len(run)<2:return
            curve=PchipInterpolator(run,[solved[m]['ps_mps'] for m in run])
            for i,m in enumerate(run[:-1]):curves[(m,run[i+1])]=curve.c[:,i].tolist()
        for m in keys:
            if solved[m]['valid']:run.append(m)
            else:commit();run=[]
        commit();next_pending=[]
        for lo,hi in pending:
            a,b=at(lo),at(hi)
            if not a['valid'] and not b['valid']:
                masked.append([lo,hi]);blocked.add((lo,hi));continue
            mid=(lo+hi)/2;p=recover(mid,a,b)
            coefficients=curves.get((lo,hi))
            if coefficients is not None and p['valid']:
                error=abs(p['ps_mps']-float(np.polyval(coefficients,mid-lo)))
                if error<=tolerance:
                    segments.append(dict(bounds=[lo,hi],coefficients=coefficients,error_mps=error))
                    continue
                # A sharp, resolved change in slope can spoil the cubic
                # derivative estimate. A checked linear segment preserves
                # continuity without inventing values across a failed trim.
                linear_error=abs(p['ps_mps']-(a['ps_mps']+b['ps_mps'])/2)
                if linear_error<=tolerance:
                    segments.append(dict(bounds=[lo,hi],coefficients=[(b['ps_mps']-a['ps_mps'])/(hi-lo),a['ps_mps']],error_mps=linear_error))
                    continue
            resolution=min_speed_step if a['valid'] and b['valid'] and p['valid'] else 1.
            if depth==max_speed_depth-1 or (hi-lo)*sound<resolution:
                masked.append([lo,hi]);continue
            next_pending.extend(((mid,hi),(lo,mid)))
        # A numerical failure at the edge may have hidden an entire valid
        # interval when both coarse endpoints initially failed. Revisit it
        # once refinement supplies nearby accepted seeds on the valid side.
        if depth<max_speed_depth-1 and hasattr(s,'recover'):
            for lo,hi in list(blocked):
                for m in (hi,lo):
                    p=at(m)
                    if p['valid'] or p.get('category')!='numerical gap' or m in retried:continue
                    near=sorted((p for p in solved.values() if p['valid']),key=lambda p:abs(p['speed_kmh']-m*sound))[:2]
                    if len(near)==2 and max(abs(p['speed_kmh']-m*sound) for p in near)<12:
                        retried.add(m)
                        method=getattr(s,'recover_edge',s.recover)
                        retry=method(m*sound,height,*sorted(near,key=lambda p:p['speed_kmh']))
                        if retry['valid']:solved[m]=retry
                if at(lo)['valid'] or at(hi)['valid']:
                    blocked.remove((lo,hi));masked.remove([lo,hi]);next_pending.append((lo,hi))
        pending=next_pending
        if not pending:break
    return dict(altitude_m=height,sound_kmh=sound,segments=sorted(segments,key=lambda a:a['bounds'][0]),
                points=list(solved.values()),masked=masked)


def sample_probes(task):
    config,height,machs=task[:3];s=sampler(config);sound=float(speed_of_sound(height))*3.6
    points={p['speed_kmh']/sound:p for p in task[3]} if len(task)>3 else {}
    points.update({float(m):s(float(m)*sound,height) for m in sorted(machs,reverse=True)})
    valid=[p for p in points.values() if p['valid']]
    if hasattr(s,'recover') and len(valid)>1:
        for m,p in points.items():
            if p['valid'] or p.get('category')!='numerical gap':continue
            lower=[q for q in valid if q['speed_kmh']<p['speed_kmh']]
            upper=[q for q in valid if q['speed_kmh']>p['speed_kmh']]
            if lower and upper:
                near=[max(lower,key=lambda q:q['speed_kmh']),min(upper,key=lambda q:q['speed_kmh'])]
            else:
                near=sorted(valid,key=lambda q:abs(q['speed_kmh']-p['speed_kmh']))[:2]
                if max(abs(q['speed_kmh']-p['speed_kmh']) for q in near)>80:continue
            near.sort(key=lambda q:q['speed_kmh'])
            q=s.recover(p['speed_kmh'],height,*near)
            if q['valid']:points[m]=q
    return [points[float(m)] for m in machs]


def altitude_breaks(config):
    """Do not smooth across native engine-table or atmosphere breakpoints."""
    lo,hi=config['altitude_min_m'],config['altitude_max_m']
    breaks={lo,hi}
    if lo<11000<hi:breaks.add(11000.)
    def visit(value):
        if not isinstance(value,dict):return
        thrust=value.get('ThrustMax')
        if isinstance(thrust,dict):
            breaks.update(float(v) for k,v in thrust.items() if k.startswith('Altitude_') and lo<v<hi)
        for child in value.values():
            if isinstance(child,dict):visit(child)
    aircraft=sampler(config).solver(lo)
    visit(aircraft.fm)
    if config['conditions']['structural_limits'] and hasattr(aircraft,'config'):
        from em_speed_limits import speed_limits
        def limits(h):return speed_limits(aircraft.fm,dict(aircraft.config,altitude_m=float(h)))
        grid=np.linspace(lo,hi,17)
        for kind in ('plot edge','IAS/Mach crossover'):
            def difference(h):
                value=limits(h)
                return value['sample_speed_kmh']-config['speed_max_kmh'] if kind=='plot edge' else value['vne_tas_kmh']-value['mne_tas_kmh']
            for a,b in zip(grid,grid[1:]):
                if difference(a)*difference(b)<0:
                    knot=float(brentq(difference,a,b,xtol=.001))
                    if kind=='plot edge' or limits(knot)['sample_speed_kmh']<config['speed_max_kmh']:
                        breaks.add(knot)
    return sorted(breaks)


def interpolate_altitude(altitude,z,heights,breaks,linear=False):
    """Interpolate columns only through contiguous valid row runs."""
    coordinate=-np.array([density_at_height(h) for h in altitude])
    target=-np.array([density_at_height(h) for h in heights])
    out=np.full((len(heights),z.shape[1]),np.nan)
    for lo,hi in zip(breaks,breaks[1:]):
        indices=np.flatnonzero((altitude>=lo)&(altitude<=hi))
        patterns,inverse=np.unique(np.isfinite(z[indices,:]).T,axis=0,return_inverse=True)
        for pattern_id,pattern in enumerate(patterns):
            columns=np.flatnonzero(inverse==pattern_id);valid=indices[pattern]
            for group in np.split(valid,np.flatnonzero(np.diff(valid)>1)+1):
                if len(group)<2:continue
                targets=np.flatnonzero((heights>=altitude[group[0]])&(heights<=altitude[group[-1]]))
                if not len(targets):continue
                curve=(interp1d(coordinate[group],z[np.ix_(group,columns)],axis=0,assume_sorted=True)
                       if linear else PchipInterpolator(coordinate[group],z[np.ix_(group,columns)],axis=0))
                out[np.ix_(targets,columns)]=curve(target[targets])
    return out


def domain(rows,heights):
    left=np.array([r['segments'][0]['bounds'][0] if r['segments'] else np.nan for r in rows])
    right=np.array([r['segments'][-1]['bounds'][1] if r['segments'] else np.nan for r in rows])
    density=-np.array([density_at_height(r['altitude_m']) for r in rows])
    target=-np.array([density_at_height(h) for h in heights])
    return left,right,np.interp(target,density,left),np.interp(target,density,right)


def normalized_values(rows,q,heights,breaks,linear=False):
    left,right,_,_=domain(rows,heights)
    values=np.array([row_values(r,np.where(q==0,lo,np.where(q==1,hi,lo+(hi-lo)*q))) for r,lo,hi in zip(rows,left,right)])
    return interpolate_altitude(np.array([r['altitude_m'] for r in rows]),values,heights,breaks,linear)


def altitude_values(rows, mach, heights, breaks,linear=False):
    """Follow both feasible speed edges instead of eroding altitude bands.

    Fixed Mach columns require BOTH endpoint altitudes to admit a speed,
    creating artificial stairs at the stall edge, redline and plot boundary.
    Interpolate fractional distance between the checked row edges instead.
    Additional native probes validate this mapping in each accepted band.
    """
    _,_,left,right=domain(rows,heights)
    mach=np.asarray(mach)
    out=np.full((len(heights),mach.shape[-1]),np.nan)
    for i,(lo,hi) in enumerate(zip(left,right)):
        local_mach=mach[i] if mach.ndim==2 else mach
        inside=(local_mach>=lo)&(local_mach<=hi)
        if not inside.any():continue
        # Evaluate the checked speed polynomials at the exact probe
        # coordinates. Regridding through 601 linearly spaced display
        # columns adds an error floor that altitude refinement cannot fix.
        q=(local_mach[inside]-lo)/(hi-lo)
        # PCHIP derivatives depend only on adjacent row secants. Retain
        # the neighbors needed for both slopes, clipped at physical knots.
        h=heights[i];alt=np.array([r['altitude_m'] for r in rows])
        band_lo=max(b for b in breaks if b<=h);band_hi=min(b for b in breaks if b>=h)
        if band_lo==band_hi:
            exact=np.flatnonzero(alt==h)
            if len(exact):out[i,inside]=row_values(rows[exact[0]],local_mach[inside])
            continue
        group=np.flatnonzero((alt>=band_lo)&(alt<=band_hi))
        k=np.searchsorted(alt[group],h)-1
        indices=group[max(0,k-1):min(len(group),k+3)]
        local=[rows[j] for j in indices]
        out[i,inside]=normalized_values(local,q,np.array([h]),[band_lo,band_hi],linear)[0]
    return out


def fixed_values(rows,mach,heights):
    """Linear density interpolation at fixed Mach, without edge warping.

    Near a numerical stall-boundary wiggle this is often more accurate than
    transporting the whole speed curve with that edge. It is only selected
    after independent native probes pass the same error check.
    """
    alt=np.array([r['altitude_m'] for r in rows]);mach=np.asarray(mach)
    out=np.full((len(heights),mach.shape[-1]),np.nan)
    density=-np.array([density_at_height(h) for h in alt])
    for i,h in enumerate(heights):
        j=int(np.clip(np.searchsorted(alt,h,side='right')-1,0,len(rows)-2))
        m=mach[i] if mach.ndim==2 else mach
        a=row_values(rows[j],m);b=row_values(rows[j+1],m)
        w=(-density_at_height(h)-density[j])/(density[j+1]-density[j])
        out[i]=a if abs(w)<1e-12 else b if abs(w-1)<1e-12 else (1-w)*a+w*b
    return out


def compute_rows(config,pool=None,stop=None,progress=None,cancelled=None):
    started=time.monotonic();point_map={};complete=0
    tolerance=QUALITY[config['quality']]['tolerance']
    # These checks compare the complete interpolated prediction to a native
    # solve, so they already include the speed-curve error. Reserve 10% for
    # rendering instead of charging that speed error a second time.
    check_tolerance=tolerance*.9
    def collect(points):
        for p in points:point_map[(p['speed_kmh'],p['altitude_m'])]=p
    def run(fn,tasks,phase):
        nonlocal complete
        if not tasks:return []
        if pool is None:
            out=[]
            for task in tasks:
                if cancelled and cancelled():raise InterruptedError('Calculation cancelled')
                out.append(fn(task))
            return out
        futures={pool.submit(fn,task):i for i,task in enumerate(tasks)};out=[None]*len(tasks)
        while futures:
            if cancelled and cancelled():
                stop.set()
                for f in futures:f.cancel()
                wait(futures)
                raise InterruptedError('Calculation cancelled')
            ready,_=wait(futures,timeout=.05,return_when=FIRST_COMPLETED)
            for f in ready:
                index=futures.pop(f)
                try:out[index]=f.result()
                except BaseException:
                    stop.set()
                    for pending in futures:pending.cancel()
                    wait(futures)
                    raise
                complete+=1
                if progress:progress(dict(samples=len(point_map),done=complete,total=complete+len(futures),
                                           phase=phase,elapsed_s=time.monotonic()-started))
        return out
    breaks=altitude_breaks(config)
    heights=sorted(set(h for lo,hi in zip(breaks,breaks[1:])
                       for h in np.linspace(lo,hi,max(3,math.ceil((hi-lo)/2000)+1))))
    rows=run(sample_row,[(config,h,[]) for h in heights],'Solving speed curves')
    for row in rows:collect(row['points'])
    rows.sort(key=lambda r:r['altitude_m']);bad=[];checked=[];probe_cache={};accepted={};bad_ranges={};linear_bands=set();fixed_bands=set();unresolved_checks=[];bad_details={}
    max_altitude_depth={'preview':5,'smooth':6,'detailed':7}[config['quality']]
    for depth in range(max_altitude_depth):
        tasks=[];checks=[]
        left=min(config['speed_min_kmh']/r['sound_kmh'] for r in rows)
        right=max(config['speed_max_kmh']/r['sound_kmh'] for r in rows)
        machs=np.array(sorted(set(np.linspace(left,right,15).tolist()+[m for m in (.85,.95,1.05,1.2) if left<m<right])))
        fractions=(.5,) if config['quality']=='preview' else (.25,.5,.75)
        intervals=[(lo,hi,lo['altitude_m']+(hi['altitude_m']-lo['altitude_m'])*f)
                   for lo,hi in zip(rows,rows[1:]) if (lo['altitude_m'],hi['altitude_m']) not in accepted for f in fractions]
        midpoints=np.array([h for _,_,h in intervals])
        if not len(midpoints):break
        predictions=altitude_values(rows,machs,midpoints,breaks)
        linear_predictions=altitude_values(rows,machs,midpoints,breaks,linear=True)
        fixed_predictions=fixed_values(rows,machs,midpoints)
        _,_,edge,right_edge=domain(rows,midpoints)
        boundary_probes=edge[:,None]+(right_edge-edge)[:,None]*np.array([.015,.1,.9,.985])
        extra_predictions=altitude_values(rows,boundary_probes,midpoints,breaks)
        extra_linear=altitude_values(rows,boundary_probes,midpoints,breaks,linear=True)
        extra_fixed=fixed_values(rows,boundary_probes,midpoints)
        for index,(lo,hi,h) in enumerate(intervals):
            local_machs=np.r_[machs,boundary_probes[index]]
            prediction=np.r_[predictions[index],extra_predictions[index]]
            linear_prediction=np.r_[linear_predictions[index],extra_linear[index]]
            fixed_prediction=np.r_[fixed_predictions[index],extra_fixed[index]]
            speed=local_machs*float(speed_of_sound(h))*3.6
            valid=np.isfinite(prediction)&(speed>=config['speed_min_kmh'])&(speed<=config['speed_max_kmh'])
            if valid.any():
                tasks.append((config,h,local_machs[valid].tolist()));checks.append((lo,hi,prediction[valid],linear_prediction[valid],fixed_prediction[valid]))
        missing=[t for t in tasks if (t[1],tuple(t[2])) not in probe_cache]
        for t,result in zip(missing,run(sample_probes,missing,'Checking altitude interpolation')):
            probe_cache[(t[1],tuple(t[2]))]=result
        probes=[probe_cache[(t[1],tuple(t[2]))] for t in tasks]
        bad=[];bad_ranges={};add=[];unresolved_checks=[];bad_details={};groups={}
        for task,(lo,hi,prediction,linear_prediction,fixed_prediction),points in zip(tasks,checks,probes):
            collect(points)
            candidates=[prediction,linear_prediction,fixed_prediction]
            errors=[[abs(p['ps_mps']-float(pred)) if p['valid'] and np.isfinite(pred) else math.inf
                     for p,pred in zip(points,predictions)] for predictions in candidates]
            groups.setdefault((lo['altitude_m'],hi['altitude_m']),[]).append((task,points,candidates,errors))
        for band,entries in groups.items():
            totals=[[e for _,_,_,errors in entries for e in errors[mode]] for mode in range(3)]
            # Prefer cubic when it passes. Otherwise choose the checked
            # alternative with the fewest failures, even for a local mask.
            mode=min(range(3),key=lambda m:(sum(e>check_tolerance for e in totals[m]),m if max(totals[m])<=check_tolerance else max(totals[m])))
            error=max(totals[mode])
            if mode==1:linear_bands.add(band)
            if mode==2:fixed_bands.add(band)
            if error<=check_tolerance:
                checked.append(error);accepted[band]=rows
                continue
            bad.append(band)
            for task,points,candidates,errors in entries:
                prediction=candidates[mode];errors=errors[mode]
                if max(errors)<=check_tolerance:continue
                bad_details[(*band,task[1])]=list(zip(task[2],points,prediction,errors))
                unresolved_checks.extend(dict(altitude_m=task[1],speed_kmh=p['speed_kmh'],valid=p['valid'],
                    actual_mps=p['ps_mps'],predicted_mps=float(pred),error_mps=e if math.isfinite(e) else None)
                    for p,pred,e in zip(points,prediction,errors) if e>check_tolerance)
            middle=sum(band)/2
            known=next((points for task,points,_,_ in entries if task[1]==middle),[])
            add.append((config,middle,known))
        if not bad:break
        if depth==max_altitude_depth-1:
            for band in bad:accepted[band]=rows
            break
        extra=run(sample_row,add,'Refining altitude curves')
        for row in extra:collect(row['points'])
        rows=sorted(rows+extra,key=lambda r:r['altitude_m'])
    # Localize remaining failures in speed. A failed edge probe must not
    # erase the tens of km/h up to its next (widely spaced) good neighbor.
    for _ in range(7):
        tasks=[];bands=[]
        for key,details in bad_details.items():
            band=key[:2];h=key[2];sound=float(speed_of_sound(h))*3.6
            entries=sorted(details,key=lambda d:d[0]);machs=[]
            for a,b in zip(entries,entries[1:]):
                if (a[3]<=check_tolerance)!=(b[3]<=check_tolerance) and (b[0]-a[0])*sound>.5:
                    machs.append((a[0]+b[0])/2)
            if machs:tasks.append((config,h,machs,[d[1] for d in details]));bands.append(key)
        if not tasks:break
        for task,key,points in zip(tasks,bands,run(sample_probes,tasks,'Checking local gap boundaries')):
            band=key[:2]
            collect(points)
            predictions=(fixed_values(accepted[band],np.array(task[2]),np.array([task[1]])) if band in fixed_bands else
                         altitude_values(accepted[band],np.array(task[2]),np.array([task[1]]),breaks,band in linear_bands))[0]
            errors=[abs(p['ps_mps']-pred) if p['valid'] and np.isfinite(pred) else math.inf for p,pred in zip(points,predictions)]
            checked.extend(e for e in errors if e<=check_tolerance)
            bad_details[key].extend(zip(task[2],points,predictions,errors))
    for key,details in bad_details.items():
        band=key[:2]
        entries=sorted(details,key=lambda d:d[0]);_,_,lo,hi=domain(accepted[band],np.array([key[2]]));width=hi[0]-lo[0]
        bad_ranges.setdefault(band,[]).extend([(max(0.,((entries[i-1][0] if i else lo[0])-lo[0])/width),
                           min(1.,((entries[i+1][0] if i+1<len(entries) else hi[0])-lo[0])/width))
                          for i,d in enumerate(entries) if d[3]>check_tolerance])
    # Dense rendering evaluates only the checked cubic curves; these nodes
    # are explicitly interpolation, not fabricated solved operating points.
    heights=np.unique(np.concatenate([np.linspace(a['altitude_m'],b['altitude_m'],9)
                                     for a,b in zip(rows,rows[1:])]))
    # The display grid follows BOTH checked edges. Keep each accepted
    # interpolation context fixed: refining its neighbor must not silently
    # change the curve whose probes already passed.
    q=np.unique(np.r_[np.linspace(0,1,501),np.linspace(0,.1,81),np.linspace(.9,1,81)])
    # The display grid must resolve the checked polynomials too; otherwise
    # linear contour extraction can reintroduce the same speed error floor.
    lo,hi,_,_=domain(rows,np.array([r['altitude_m'] for r in rows]))
    for _ in range(7):
        mid=(q[:-1]+q[1:])/2
        values=np.array([row_values(r,np.where(q==0,a,np.where(q==1,b,a+(b-a)*q))) for r,a,b in zip(rows,lo,hi)])
        middle=np.array([row_values(r,a+(b-a)*mid) for r,a,b in zip(rows,lo,hi)])
        error=abs(middle-(values[:,:-1]+values[:,1:])/2)
        refine=np.any(np.isfinite(error)&(error>tolerance*.08),axis=0)
        if not refine.any():break
        q=np.sort(np.r_[q,mid[refine]])
    if bad_ranges:q=np.unique(np.r_[q,[x for ranges in bad_ranges.values() for pair in ranges for x in pair]])
    z=np.full((len(heights),len(q)),np.nan)
    _,_,left,right=domain(rows,heights)
    for context_id in {id(context) for context in accepted.values()}:
        bands=[band for band,context in accepted.items() if id(context)==context_id]
        context=accepted[bands[0]]
        for mode in range(3):
            selected=[band for band in bands if (2 if band in fixed_bands else 1 if band in linear_bands else 0)==mode]
            if not selected:continue
            indices=np.flatnonzero(np.logical_or.reduce([(heights>=lo)&(heights<=hi) for lo,hi in selected]))
            if mode==2:
                mach=left[indices,None]+(right-left)[indices,None]*q
                z[indices]=fixed_values(context,mach,heights[indices])
            else:z[indices]=normalized_values(context,q,heights[indices],breaks,mode==1)
    sound=np.array([float(speed_of_sound(h))*3.6 for h in heights])
    # Empty rows still have finite plotting coordinates, with a masked SEP.
    left=np.where(np.isfinite(left),left,config['speed_min_kmh']/sound)
    right=np.where(np.isfinite(right),right,config['speed_max_kmh']/sound)
    speed=sound[:,None]*(left[:,None]+(right-left)[:,None]*q)
    z[(speed<config['speed_min_kmh'])|(speed>config['speed_max_kmh'])]=np.nan
    masked=[]
    for lo,hi in bad:
        interior=(heights>lo)&(heights<hi)
        for a,b in bad_ranges[(lo,hi)]:
            columns=((q>a)|(a==0))&((q<b)|(b==1))
            z[np.ix_(interior,columns)]=np.nan
            middle=int(np.argmin(abs(heights-(lo+hi)/2)))
            v0=np.interp(a,q,speed[middle]);v1=np.interp(b,q,speed[middle])
            masked.append(dict(bounds=[float(v0),float(v1),lo,hi],reason='altitude interpolation unresolved'))
    for row in rows:
        for lo,hi in row['masked']:
            masked.append(dict(bounds=[lo*row['sound_kmh'],hi*row['sound_kmh'],row['altitude_m'],row['altitude_m']],reason='speed interval unresolved'))
    return dict(points=list(point_map.values()),triangles=[],cells=[],masked_cells=masked,
                surface=dict(speeds_kmh=speed,altitudes_m=heights,sep_mps=z,
                             coordinate=dict(kind='normalized',axis=q,left_speed_kmh=sound*left,right_speed_kmh=sound*right)),
                sampling=dict(method='checked cubic speed curves with independent altitude probes',
                              tolerance_mps=tolerance,samples=len(point_map),altitude_rows=len(rows),
                              accepted_cells=sum(len(r['segments']) for r in rows),masked_cells=len(masked),
                              unresolved_checks=unresolved_checks,
                              max_checked_error_mps=max(checked+[s['error_mps'] for r in rows for s in r['segments']],default=None)))
