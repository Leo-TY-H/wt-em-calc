"""Geometric checks for contours near steep, native trim transitions."""
import numpy as np


def visible_range(config=None):
    levels=(config or {}).get('sep_contour_levels_mps',[])
    return min([-300.,*levels]),max([300.,*levels])


def visible_error(actual,predicted,config=None):
    """Whether an error can affect any displayed SEP contour or heat color.

    Include the requested contour levels as well as the heatmap color range.
    """
    if config is not None and not config.get('heatmap',True):
        # Errors crossing a target matter even when both endpoints are far
        # from it. A small guard also keeps nearby shape changes in scope.
        lower=np.minimum(actual,predicted);upper=np.maximum(actual,predicted)
        guard=8.*config.get('sep_tolerance_mps',.5)
        relevant=np.zeros(np.broadcast_shapes(np.shape(actual),np.shape(predicted)),dtype=bool)
        for level in config['sep_contour_levels_mps']:
            relevant|=(lower<=level+guard)&(upper>=level-guard)
        return relevant
    low,high=visible_range(config)
    return (np.maximum(actual,predicted)>=low)&(np.minimum(actual,predicted)<=high)


def crossing_coordinates(coordinates,values,levels):
    """Locate every bracketed target; masked samples always separate branches.

    These coordinates only request independent solves. They are never used
    as solved knots or as evidence that a contour is accurate.
    """
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
    return dict(scope='surface' if config.get('heatmap',True) else 'contours',
                checked_sep_range_mps=list(visible_range(config)),
                checked_sep_levels_mps=list(config.get('sep_contour_levels_mps',[])))


def turn_tolerance(sep_tolerance):
    # At a 60 degree/s axis these are 0.1--0.4 pixels on a 600px plot.
    # Report this separately from the SEP tolerance; neither is a trim tolerance.
    return float(np.interp(sep_tolerance,[.15,.5,1.],[.01,.02,.04]))


def neighboring_loads(speed,loads,tolerance):
    rates=np.degrees(9.8100004196167*np.sqrt(np.maximum(0.,np.asarray(loads)**2-1.))/(speed/3.6))
    return np.hypot(1.,np.radians(np.maximum(0.,rates[:,None]+[-tolerance,tolerance]))*(speed/3.6)/9.8100004196167)


def within_contour_band(actual,neighbors,sep_tolerance):
    """A continuous predicted curve crosses the observed SEP within the band.

    Both endpoints must exist: this test cannot certify a masked equilibrium
    or extend an envelope. Raw SEP errors remain in the diagnostic evidence.
    """
    neighbors=np.asarray(neighbors)
    return (np.isfinite(neighbors).all(axis=-1)&
            (np.asarray(actual)>=np.min(neighbors,axis=-1)-sep_tolerance)&
            (np.asarray(actual)<=np.max(neighbors,axis=-1)+sep_tolerance))


def speed_tolerance(config):
    """Subpixel contour position allowance in the requested speed viewport."""
    cells=float(np.interp(config['sep_tolerance_mps'],[.15,.5,1.],[.5,1.,2.]))
    return cells*(config['speed_max_kmh']-config['speed_min_kmh'])/(2*config['surface_resolution']-2)


def surface_band_values(columns,speed,loads,config):
    """Sample a contour-position box in speed AND turn rate.

    A vertical-only band over-refines nearly vertical contours. The box is
    measured in plot coordinates; equilibrium tolerances are unchanged. All
    nine observations must exist, so this never certifies across masked gaps.
    """
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
    """Extra native holdouts where a central check nearly spends its budget.

    A cubic's largest error need not occur at the midpoint. Follow the same
    visible SEP level to both quarter speeds; using a fixed load instead can
    move the check outside the displayed range on a steep surface. These are
    query locations only: no interpolated point is accepted as an equilibrium.
    """
    from scipy.optimize import brentq
    from em_sampling import speed_interpolate
    from em_surface import envelope_limits
    tolerance=config['sep_tolerance_mps']
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
    low_ps,high_ps=visible_range(config)
    target=float(np.clip(actual[chosen],low_ps+.1*tolerance,high_ps-.1*tolerance))
    if not config.get('heatmap',True):
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
        return brentq(lambda n:float(speed_interpolate(columns,[probe],[n])[0,0])-target,
                      grid[i],grid[i+1],xtol=1e-7)
    points=[]
    for probe in (low+.25*(high-low),low+.75*(high-low)):
        load=contour_load(probe)
        if load is not None:points.append((probe,load))
    # Jet table axes use body-forward speed. Their slope breaks follow
    # curved lines in TAS/load coordinates. Predict their intersections with
    # this contour, then ask for independent native trims at those locations.
    # Approximate angles only place checks; they never supply plotted values.
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
            try:probe=brentq(lambda v:body_speed(v)-knot,low,high,xtol=.01)
            except ValueError:continue
            load=contour_load(probe)
            if load is not None and all(abs(probe-v)>.01 for v,_ in points):points.append((probe,load))
    return points
