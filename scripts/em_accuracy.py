import numpy as np


def visible_error(actual,predicted,config=None):
    config=config or {}
    lower=np.minimum(actual,predicted);upper=np.maximum(actual,predicted)
    guard=8.*config.get('sep_tolerance_mps',.5)
    relevant=np.zeros(np.broadcast_shapes(np.shape(actual),np.shape(predicted)),dtype=bool)
    for level in config.get('sep_contour_levels_mps',[]):
        relevant|=(lower<=level+guard)&(upper>=level-guard)
    return relevant


def crossing_coordinates(coordinates,values,levels):
    coordinates=np.asarray(coordinates);values=np.asarray(values)
    result=[]
    for level in levels:
        for i in range(len(values)-1):
            a,b=values[i:i+2]
            if not np.isfinite([a,b]).all() or not min(a,b)<=level<=max(a,b):continue
            t=(level-a)/(b-a) if b!=a else .5
            result.append(float(coordinates[i]+t*(coordinates[i+1]-coordinates[i])))
    return sorted(set(result))


def surface_contour_loads(columns,speed,config):
    from em_sampling import speed_interpolate,load_coordinate,coordinate_load
    from em_surface import envelope_limits
    low,high=envelope_limits(columns,[speed])[0]
    if not np.isfinite([low,high]).all() or high<=low:return []
    coordinates=np.linspace(float(load_coordinate(low,high)),1.,129)
    loads=coordinate_load(coordinates,high)
    values=speed_interpolate(columns,[speed],loads)[:,0]
    return crossing_coordinates(loads,values,config['sep_contour_levels_mps'])


def coverage(config):
    return dict(scope='contours',
                checked_sep_levels_mps=list(config.get('sep_contour_levels_mps',[])))


def boundary_contour_speeds(columns,config):
    from scipy.optimize import brentq
    from em_surface import envelope_values,same_deployment_branch
    queries=set()
    for left,right in zip(columns,columns[1:]):
        if not all(c.get('boundary_status') in ('verified limit','plot ceiling') and
                   all(c.get(k) and c[k]['valid'] for k in ('lower_boundary','boundary'))
                   for c in (left,right)):continue
        if not same_deployment_branch([left,right]):continue
        lo,hi=left['speed_kmh'],right['speed_kmh']
        for side,key in enumerate(('lower_boundary','boundary')):
            a,b=left[key]['ps_mps'],right[key]['ps_mps']
            for level in config.get('sep_contour_levels_mps',[]):
                if not np.isfinite([a,b]).all() or (a-level)*(b-level)>=0.:continue
                try:
                    residual=lambda v:float(envelope_values(columns,[v])[0,side+2])-level
                    if config.get('trim_solver_mode')=='stationary':
                        from em_roots import checked_root
                        from em_accuracy_policy import ALLOWANCE
                        speed=checked_root(residual,lo,hi,residual_tolerance=float(ALLOWANCE),
                            xtol=.1*speed_tolerance(config))
                    else:speed=brentq(residual,lo,hi,xtol=1e-5)
                except ValueError:continue
                if min(speed-lo,hi-speed)>speed_tolerance(config):queries.add((lo,hi,float(speed)))
    return sorted(queries)


def turn_tolerance(sep_tolerance):


    return float(np.interp(sep_tolerance,[.15,.5,1.],[.01,.02,.04]))


def neighboring_loads(speed,loads,tolerance):
    rates=np.degrees(9.8100004196167*np.sqrt(np.maximum(0.,np.asarray(loads)**2-1.))/(speed/3.6))
    return np.hypot(1.,np.radians(np.maximum(0.,rates[:,None]+[-tolerance,tolerance]))*(speed/3.6)/9.8100004196167)


def within_contour_band(actual,neighbors,sep_tolerance):
    neighbors=np.asarray(neighbors)
    return (np.isfinite(neighbors).all(axis=-1)&
            (np.asarray(actual)>=np.min(neighbors,axis=-1)-sep_tolerance)&
            (np.asarray(actual)<=np.max(neighbors,axis=-1)+sep_tolerance))


def speed_tolerance(config):
    cells=float(np.interp(config['sep_tolerance_mps'],[.15,.5,1.],[.5,1.,2.]))
    return cells*(config['speed_max_kmh']-config['speed_min_kmh'])/(2*config['surface_resolution']-2)


def refinement_speed_tolerance(config):
    """Locate stationary transitions to the chart's horizontal tolerance."""
    return speed_tolerance(config)*(1. if config.get('trim_solver_mode')=='stationary' else .25)


def load_interval_resolved(speed,low,high,config):
    """An admitted interval narrower than the chart's vertical tolerance.

    This bounds placement of a transition, not its SEP amplitude or root
    coverage. Failed samples remain gaps and never enter an admitted interval.
    """
    if config.get('trim_solver_mode')!='stationary':return False
    rates=np.degrees(9.8100004196167*np.sqrt(np.maximum(0.,np.array([low,high])**2-1.))/(speed/3.6))
    return abs(rates[1]-rates[0])<=turn_tolerance(config['sep_tolerance_mps'])


def surface_band_values(columns,speed,loads,config):
    from em_sampling import speed_interpolate
    from em_surface import envelope_limits
    dv=speed_tolerance(config);dw=turn_tolerance(config['sep_tolerance_mps'])
    speeds=np.clip(np.array([speed-dv,speed,speed+dv]),columns[0]['speed_kmh'],columns[-1]['speed_kmh'])
    loads=np.atleast_1d(loads)
    rates=np.degrees(9.8100004196167*np.sqrt(np.maximum(0.,loads**2-1.))/(speed/3.6))
    turns=np.maximum(0.,rates[:,None]+[-dw,0.,dw])
    query=np.hypot(1.,np.radians(turns[:,:,None])*(speeds[None,None,:]/3.6)/9.8100004196167).reshape((-1,3))
    limits=envelope_limits(columns,speeds)
    query=np.clip(query,limits[None,:,0],limits[None,:,1])
    values=speed_interpolate(columns,speeds,query).reshape((len(loads),9))
    return values


def contour_check_points(columns,low,high,speed,loads,actual,predicted,config,longitudinal_knots=()):
    from scipy.optimize import brentq
    from em_sampling import speed_interpolate
    from em_surface import envelope_limits
    from em_accuracy_policy import comparison_tolerance
    tolerance=comparison_tolerance(config)
    if high-low<=4*speed_tolerance(config):return []
    candidates=np.flatnonzero(np.isfinite(actual)&np.isfinite(predicted)&
        (abs(actual-predicted)>tolerance)&visible_error(actual,predicted,config))
    if not len(candidates):return []
    boxes=surface_band_values(columns,speed,np.asarray(loads)[candidates],config)
    budget=np.max(abs(boxes-np.asarray(predicted)[candidates,None]),axis=1)+tolerance
    ratios=abs(np.asarray(actual)[candidates]-np.asarray(predicted)[candidates])/budget
    ratios=np.where(np.isfinite(ratios),ratios,0.)
    index=int(np.argmax(ratios))
    if ratios[index]<.65:return []
    chosen=candidates[index]
    target=min(config['sep_contour_levels_mps'],key=lambda level:abs(level-actual[chosen]))
    floor,cap=envelope_limits(columns,[speed])[0]
    if not np.isfinite([floor,cap]).all() or cap<=floor:return []
    fraction=float(np.clip((loads[chosen]-floor)/(cap-floor),0.,1.))
    def contour_load(probe):
        bottom,top=envelope_limits(columns,[probe])[0]
        if not np.isfinite([bottom,top]).all() or top<=bottom:return None
        grid=np.linspace(bottom,top,25)
        values=speed_interpolate(columns,[probe],grid)[:,0]-target
        crossings=[i for i in range(len(grid)-1) if np.isfinite(values[i:i+2]).all()
                   and values[i]*values[i+1]<=0.]
        if not crossings:return None
        near=bottom+fraction*(top-bottom)
        i=min(crossings,key=lambda i:abs((grid[i]+grid[i+1])*.5-near))
        residual=lambda n:float(speed_interpolate(columns,[probe],[n])[0,0])-target
        if config.get('trim_solver_mode')=='stationary':
            from em_roots import checked_root
            from em_accuracy_policy import ALLOWANCE
            return checked_root(residual,grid[i],grid[i+1],residual_tolerance=float(ALLOWANCE),xtol=2e-5)
        return brentq(residual,grid[i],grid[i+1],xtol=1e-7)
    points=[]
    for probe in (low+.25*(high-low),low+.75*(high-low)):
        load=contour_load(probe)
        if load is not None:points.append((probe,load))


    if longitudinal_knots:
        local=[c for c in columns if low<=c['speed_kmh']<=high]
        def body_speed(probe):
            n=contour_load(probe)
            if n is None:return float('nan')
            bottom,top=envelope_limits(columns,[probe])[0]
            u=(n-bottom)/(top-bottom);angles=[]
            for c in local:
                ps=sorted((p for p in c['points'] if p['valid']),key=lambda p:p['load_g'])
                query=c['lower_boundary']['load_g']+u*(c['boundary']['load_g']-c['lower_boundary']['load_g'])
                angles.append([np.interp(query,[p['load_g'] for p in ps],[p[k] for p in ps])
                               for k in ('alpha_deg','sideslip_attitude_deg')])
            a,b=[np.interp(probe,[c['speed_kmh'] for c in local],np.asarray(angles)[:,i]) for i in (0,1)]
            return probe*np.cos(np.radians(a))*np.cos(np.radians(b))
        a,b=body_speed(low),body_speed(high)
        for knot in longitudinal_knots:
            if not np.isfinite([a,b]).all() or (a-knot)*(b-knot)>=0.:continue
            try:
                if config.get('trim_solver_mode')=='stationary':
                    from em_roots import checked_root
                    probe=checked_root(lambda v:body_speed(v)-knot,low,high,
                        residual_tolerance=.25*speed_tolerance(config),xtol=.1*speed_tolerance(config))
                else:probe=brentq(lambda v:body_speed(v)-knot,low,high,xtol=.01)
            except ValueError:continue
            load=contour_load(probe)
            if load is not None and all(abs(probe-v)>.01 for v,_ in points):points.append((probe,load))
    return points
