import numpy as np
from functools import lru_cache
from scipy.interpolate import PchipInterpolator


def same_deployment_branch(columns):
    mach={tuple(c['mach_branch']) for c in columns if c.get('mach_branch') is not None}
    if len(mach)>1:return False
    threshold=float(np.float32(.05))*100.
    branches={tuple(c[k]['flaps_percent']>threshold for k in ('lower_boundary','boundary'))
              for c in columns if all(c.get(k) and 'flaps_percent' in c[k]
                                      for k in ('lower_boundary','boundary'))}
    return len(branches)<=1


def local_support(columns,left,valid):
    indices=[left,left+1]
    core=columns[left:left+2]
    if not same_deployment_branch(core):return indices
    for i in (left-1,left+2):
        if (0<=i<len(columns) and valid[i]
                and same_deployment_branch(core+[columns[i]])):
            indices.append(i)
    return sorted(indices)


def eligible(column):
    if (column.get('boundary_status') not in ('verified limit','plot ceiling') and
            not (column.get('boundary') or {}).get('native_branch_search')):return False
    if not all(column.get(k) and column[k]['valid'] for k in ('boundary','lower_boundary')):return False
    if any(column.get(k) for k in ('numerical_gap_brackets','interior_failures','unresolved_load_intervals')):return False
    low,high=column['lower_boundary']['load_g'],column['boundary']['load_g']
    if high-low<=1e-10:return True
    from em_sampling import column_curve
    prepared=column_curve(column)
    return bool(prepared and len(prepared[1])==1 and prepared[1][0][0]<=low+1e-10 and prepared[1][0][1]>=high-1e-10)


def _derivative(xs,values,i):
    if len(xs)==2:return (values[1]-values[0])/(xs[1]-xs[0])
    if i==0 or i==len(xs)-1:
        a,b,c=(0,1,2) if i==0 else (-1,-2,-3)
        h0=abs(xs[b]-xs[a]);h1=abs(xs[c]-xs[b])
        m0=(values[b]-values[a])/(xs[b]-xs[a]);m1=(values[c]-values[b])/(xs[c]-xs[b])
        d=((2*h0+h1)*m0-h0*m1)/(h0+h1)
        d=np.where(np.sign(d)!=np.sign(m0),0.,d)
        return np.where((np.sign(m0)!=np.sign(m1))&(abs(d)>3*abs(m0)),3*m0,d)
    h0=xs[i]-xs[i-1];h1=xs[i+1]-xs[i]
    m0=(values[i]-values[i-1])/h0;m1=(values[i+1]-values[i])/h1
    same=(m0*m1>0.);w1=2*h1+h0;w2=h1+2*h0
    with np.errstate(divide='ignore',invalid='ignore'):
        return np.where(same,(w1+w2)/(w1/m0+w2/m1),0.)


def pchip_pair(xs,values,left,speeds):
    xs=np.asarray(xs);values=np.asarray(values);h=xs[left+1]-xs[left]
    t=(np.asarray(speeds)-xs[left])/h;t2=t*t;t3=t2*t
    return ((2*t3-3*t2+1)*values[left]+(t3-2*t2+t)*h*_derivative(xs,values,left)
            +(-2*t3+3*t2)*values[left+1]+(t3-t2)*h*_derivative(xs,values,left+1))


class PreparedPchipPair:
    """Fixed speed-coordinate work shared by all load-inversion iterations."""
    def __init__(self,xs,left,speeds):
        self.xs=np.asarray(xs);self.left=left
        h=self.xs[left+1]-self.xs[left]
        t=(np.asarray(speeds)-self.xs[left])/h;t2=t*t;t3=t2*t
        self.weights=(2*t3-3*t2+1,(t3-2*t2+t)*h,-2*t3+3*t2,(t3-t2)*h)

    def __call__(self,values):
        values=np.asarray(values);left=self.left;a,b,c,d=self.weights
        return (a*values[left]+b*_derivative(self.xs,values,left)
                +c*values[left+1]+d*_derivative(self.xs,values,left+1))


def normalized_values(column,u):
    from em_sampling import column_curve,load_coordinate
    low,high=column['lower_boundary']['load_g'],column['boundary']['load_g']
    if high-low<=1e-10:return np.full(np.shape(u),column['boundary']['ps_mps'])
    start=float(load_coordinate(low,high))
    return column_curve(column)[1][0][2](np.clip(start+(1-start)*u,start,1.))


def angle_surface(support,sx,left,speeds,loads,bottom,top):
    from em_sampling import column_at_load
    curves=[];direct=[]
    for column in support:
        points=[p for p in column['points'] if p['valid'] and p.get('surface_sample',True)
                and column['lower_boundary']['load_g']<=p['load_g']<=column['boundary']['load_g']]
        if len(points)<4 or np.any(np.diff([p['alpha_deg'] for p in points])<=1e-8):return None
        cached=column.get('_angle_map')
        if cached is None:
            cached=ParametricCurve(points,column['boundary']['load_g'],_output_allowance(points));column['_angle_map']=cached
        curves.append(cached)
        direct.append(bool(cached.output_tolerance) and len({curve_branch(p) for p in points})==1)
    def states(u):
        return np.asarray([curve.load(curve.angles[0]+u*(curve.angles[-1]-curve.angles[0])) for curve in curves])
    interpolate=PreparedPchipPair(sx,left,speeds)
    def mapped(u):return interpolate(states(u))
    target=np.clip(loads,bottom,top);lo=np.zeros_like(target);hi=np.ones_like(target)
    tolerance=(min(2e-4,.25*min(float(np.min(curve.load_tolerance)) for curve in curves))
               if all(curve.output_tolerance for curve in curves) else 1e-10)
    u=np.sqrt(np.clip((target-bottom)/(top-bottom),0.,1.))
    for iteration in range(20):
        predicted=mapped(u);error=predicted-target
        if np.max(abs(error),initial=0.)<=tolerance:break
        lo=np.where(error<0.,u,lo);hi=np.where(error>0.,u,hi)
        before=np.maximum(0.,u-1e-5);after=np.minimum(1.,u+1e-5)
        slope=(mapped(after)-mapped(before))/(after-before)
        with np.errstate(divide='ignore',invalid='ignore'):trial=u-error/slope
        u=np.where(abs(error)<=tolerance,u,np.where(np.isfinite(trial)&(trial>=lo)&(trial<=hi),trial,(lo+hi)*.5))
    ns=states(u)
    # This coordinate already identifies alpha. On one stationary branch the
    # same parametric curve supplies power directly; inverting its load again
    # only duplicates work and introduces a second numerical error.
    powers=np.asarray([curve.power(curve.angles[0]+u*(curve.angles[-1]-curve.angles[0]))
        if known else column_at_load(c,np.clip(n,c['lower_boundary']['load_g'],c['boundary']['load_g']))
        for c,n,curve,known in zip(support,ns,curves,direct)])
    return interpolate(powers)


def fitted_interpolate(columns,speeds,loads):
    from em_sampling import column_at_load,load_coordinate,column_curve
    speeds=np.atleast_1d(speeds);loads=np.asarray(loads)
    grid=np.broadcast_to(loads[:,None],(len(loads),len(speeds))) if loads.ndim==1 else loads
    out=np.full(grid.shape,np.nan);covered=np.zeros(len(speeds),dtype=bool)
    xs=np.array([c['speed_kmh'] for c in columns]);valid=[eligible(c) for c in columns]
    covered[(speeds<xs[0])|(speeds>xs[-1])]=True
    right=np.searchsorted(xs,speeds)
    for j in np.unique(right):
        chosen=np.flatnonzero(right==j)
        exact=chosen[speeds[chosen]==xs[j]] if j<len(xs) else np.array([],dtype=int)
        for index in exact:out[:,index]=column_at_load(columns[j],grid[:,index]);covered[index]=True
        chosen=np.setdiff1d(chosen,exact,assume_unique=True)
        if not len(chosen) or not 0<j<len(xs):continue
        if not valid[j-1] or not valid[j]:
            if any(not any(p['valid'] for p in c['points']) for c in columns[j-1:j+1]):covered[chosen]=True
            continue
        indices=local_support(columns,j-1,valid)
        support=[columns[i] for i in indices];sx=xs[indices];v=speeds[chosen]
        limits=_envelope_curve(support)(v)[:,:2]
        bottom,top=limits.T;n=grid[:,chosen]
        inside=(n>=bottom[None,:]-1e-12)&(n<=top[None,:]+1e-12)&(top[None,:]>1.)
        with np.errstate(invalid='ignore',divide='ignore'):
            start=load_coordinate(bottom,top)
            u=(load_coordinate(np.clip(n,bottom,top),top)-start)/(1-start)


        regular=all(c['boundary_reason'] in ('wing force','control') for c in support)
        predicted=None if regular else angle_surface(support,sx,indices.index(j-1),v,n,bottom,top)
        if predicted is None:
            values=np.asarray([normalized_values(c,u) for c in support])
            predicted=pchip_pair(sx,values,indices.index(j-1),v)
        out[:,chosen]=np.where(inside,predicted,np.nan);covered[chosen]=True
    return out,covered


def envelope_limits(columns,speeds):
    return envelope_values(columns,speeds)[:,:2]


def mesh_limits(columns,speeds):
    speeds=np.asarray(speeds);out=np.full((len(speeds),2),np.nan);run=[]
    def finish():
        if not run:return
        xs=np.array([x for x,_ in run]);bounds=np.array([b for _,b in run])
        if len(run)>1:
            mask=(speeds>=xs[0])&(speeds<=xs[-1])
            out[mask]=PchipInterpolator(xs,bounds,axis=0)(speeds[mask])
        for x,b in run:out[speeds==x]=b
    for column in columns+[None]:
        points=([p for p in column['points'] if p['valid'] and p.get('surface_sample',True)]
                if column is not None else [])
        if points:
            loads=[p['load_g'] for p in points]
            run.append((column['speed_kmh'],[min(loads),max(loads)]))
        else:finish();run=[]
    return out


def envelope_values(columns,speeds):
    speeds=np.asarray(speeds)
    if not any(c.get('constraint_corner') for c in columns) or any(c.get('_coordinate_mapped') for c in columns):
        return _envelope_values(columns,speeds)
    result=np.full((len(speeds),4),np.nan)
    for chosen,support,query in coordinate_regions(columns,speeds):
        result[chosen]=_envelope_values(support,query)
    return result


def _envelope_curve(columns):
    xs=tuple(c['speed_kmh'] for c in columns)
    values=tuple((c['lower_boundary']['load_g'],c['boundary']['load_g'],
                  c['lower_boundary']['ps_mps'],c['boundary']['ps_mps']) for c in columns)
    # The replacement's repeated numerical probes and detached previews share
    # coefficients by value, without retaining mutable solver/column objects.
    if all('stationary_coordinates' in c['boundary'] for c in columns):
        return _prepared_envelope(xs,values)
    return PchipInterpolator(xs,values,axis=0)


@lru_cache(maxsize=1024)
def _prepared_envelope(xs,values):
    return PchipInterpolator(xs,values,axis=0)


def _envelope_values(columns,speeds):
    speeds=np.asarray(speeds);out=np.full((len(speeds),4),np.nan);run=[]
    def values(c):
        return [c['lower_boundary']['load_g'],c['boundary']['load_g'],
                c['lower_boundary']['ps_mps'],c['boundary']['ps_mps']]
    def finish(group):
        if len(group)<2:return
        xs=np.array([c['speed_kmh'] for c in group]);right=np.searchsorted(xs,speeds)
        for j in np.unique(right):
            if not 0<j<len(xs):continue
            mask=(right==j)
            support=local_support(group,j-1,[True]*len(group))
            out[mask]=_envelope_curve([group[i] for i in support])(speeds[mask])
    for c in columns+[None]:
        if (c is not None and (c.get('boundary_status') in ('verified limit','plot ceiling') or (c.get('boundary') or {}).get('native_branch_search')) and
                all(c.get(k) and c[k]['valid'] for k in ('lower_boundary','boundary'))):run.append(c)
        else:finish(run);run=[]
    for c in columns:
        if c.get('boundary') and c.get('lower_boundary'):out[speeds==c['speed_kmh']]=values(c)
    return out


class ParametricCurve:
    def __init__(self,points,top,output_tolerance=0.):
        from scipy.interpolate import CubicSpline
        self.top=top
        self.angles=np.array([p['alpha_deg'] for p in points])
        self.loads=np.array([p['load_g'] for p in points])
        self.load=CubicSpline(self.angles,self.loads)
        self.power=CubicSpline(self.angles,[p['ps_mps'] for p in points])


        c=self.load.c;h=np.diff(self.angles)
        with np.errstate(divide='ignore',invalid='ignore'):
            vertex=np.clip(-c[1]/(3*c[0]),0.,h)
        vertex=np.where(np.isfinite(vertex),vertex,0.)
        d=lambda t:(3*c[0]*t+2*c[1])*t+c[2]
        if np.min([d(np.zeros_like(h)),d(h),d(vertex)])<0.:
            self.load=PchipInterpolator(self.angles,self.loads)


        c=self.power.c;secants=np.diff([p['ps_mps'] for p in points])/h
        with np.errstate(divide='ignore',invalid='ignore'):
            vertex=np.clip(-c[1]/(3*c[0]),0.,h)
        vertex=np.where(np.isfinite(vertex),vertex,0.)
        derivatives=np.array([(3*c[0]*t+2*c[1])*t+c[2] for t in (np.zeros_like(h),h,vertex)])
        if np.any(derivatives*secants[None,:]<0.) or np.any((secants==0.)&np.any(derivatives!=0.,axis=0)):
            self.power=PchipInterpolator(self.angles,[p['ps_mps'] for p in points])
        self.slope=self.load.derivative()
        self.output_tolerance=output_tolerance
        self.load_tolerance=np.full(len(h),1e-12)
        if output_tolerance:
            # Bound the SEP effect of inverse error using extrema of the two
            # cubic derivatives on each segment. This stops at output accuracy
            # rather than twelve invisible decimal places of load factor.
            def derivative_bounds(curve):
                c=curve.c
                with np.errstate(divide='ignore',invalid='ignore'):
                    vertex=np.clip(-c[1]/(3*c[0]),0.,h)
                vertex=np.where(np.isfinite(vertex),vertex,0.)
                return np.array([(3*c[0]*t+2*c[1])*t+c[2] for t in (np.zeros_like(h),h,vertex)])
            minimum=np.maximum(0.,np.min(derivative_bounds(self.load),axis=0))
            maximum=np.max(abs(derivative_bounds(self.power)),axis=0)
            self.load_tolerance=np.divide(output_tolerance*minimum,maximum,
                out=np.full_like(minimum,np.inf),where=maximum>0.)

    def __call__(self,u):
        from em_sampling import coordinate_load
        shape=np.shape(u);n=np.asarray(coordinate_load(u,self.top)).ravel()
        indices=np.clip(np.searchsorted(self.loads,n,side='right')-1,0,len(self.loads)-2)
        lo=self.angles[indices].copy();hi=self.angles[indices+1].copy()
        alpha=lo+(hi-lo)*np.clip((n-self.loads[indices])/(self.loads[indices+1]-self.loads[indices]),0.,1.)
        for _ in range(16):
            error=self.load(alpha)-n
            closed=abs(error)<=self.load_tolerance[indices]
            flat=(self.load_tolerance[indices]==0.)
            if self.output_tolerance and np.any(flat):
                closed[flat]|=abs(self.power(hi[flat])-self.power(lo[flat]))<=self.output_tolerance
            if np.all(closed):break
            lo=np.where(error<0.,alpha,lo);hi=np.where(error>0.,alpha,hi)
            slope=self.slope(alpha)
            with np.errstate(divide='ignore',invalid='ignore'):trial=alpha-error/slope
            alpha=np.where(closed,alpha,np.where(np.isfinite(trial)&(trial>=lo)&(trial<=hi),
                                                trial,(lo+hi)*.5))
        return self.power(alpha).reshape(shape)


def _smooth_curve(points,top,output_tolerance=0.):
    from em_sampling import load_coordinate,interpolation_knots
    if len(points)>=4 and np.all(np.diff([p['alpha_deg'] for p in points])>1e-8):
        return ParametricCurve(points,top,output_tolerance)
    ux,uy=interpolation_knots(load_coordinate([p['load_g'] for p in points],top),[p['ps_mps'] for p in points])
    return PchipInterpolator(ux,uy)


class BranchCurve:
    def __init__(self,groups,top,output_tolerance=0.):
        from em_sampling import load_coordinate
        self.parts=[];self.transitions=[]
        previous=None
        for group in groups:
            lo,hi=[float(load_coordinate(p['load_g'],top)) for p in (group[0],group[-1])]
            curve=_smooth_curve(group,top,output_tolerance) if len(group)>1 else None
            self.parts.append((lo,hi,curve,group[0]['ps_mps']))
            if previous is not None:
                self.transitions.append((previous[0],lo,previous[1],group[0]['ps_mps']))
            previous=(hi,group[-1]['ps_mps'])
    def __call__(self,u):
        u=np.asarray(u);out=np.full(u.shape,np.nan)
        for lo,hi,curve,power in self.parts:
            mask=(u>=lo-1e-14)&(u<=hi+1e-14)
            out[mask]=curve(np.clip(u[mask],lo,hi)) if curve is not None else power
        for lo,hi,p0,p1 in self.transitions:
            mask=(u>lo)&(u<hi)
            out[mask]=p0+(p1-p0)*(u[mask]-lo)/(hi-lo)
        return out


def curve_branch(point):
    return (point.get('roll_leveling_branch'),
            tuple(point.get('propulsion',{}).get('stationarity',{}).get('governor_status',())))


def prepare_curve(points,top):
    # Preparation depends only on these numerical values. Refinement, boundary
    # checks and detached previews often ask for the same curve repeatedly.
    # Do not key on mutable point dictionaries or retain whole solver results.
    key=tuple((p['load_g'],p.get('alpha_deg',0.),p['ps_mps'],curve_branch(p)) for p in points)
    return _prepared_curve(key,top,_output_allowance(points))


def _output_allowance(points):
    if all('stationary_coordinates' in point for point in points):
        from em_accuracy_policy import ALLOWANCE
        return float(ALLOWANCE)
    return 0.


@lru_cache(maxsize=256)
def _prepared_curve(key,top,output_tolerance=0.):
    points=[dict(load_g=n,alpha_deg=a,ps_mps=p,branch=b) for n,a,p,b in key]
    groups=[]
    for point in points:
        if not groups or point['branch']!=groups[-1][-1]['branch']:groups.append([])
        groups[-1].append(point)
    return BranchCurve(groups,top,output_tolerance) if len(groups)>1 else _smooth_curve(points,top,output_tolerance)


def coordinate_regions(columns,speeds):
    corners=[c for c in columns if c.get('constraint_corner')]
    edges=[-np.inf]+[c['speed_kmh'] for c in corners]+[np.inf]
    for i,(lo,hi) in enumerate(zip(edges,edges[1:])):
        chosen=np.flatnonzero((speeds>=lo)&(speeds<=hi))
        if not len(chosen):continue
        support=[c for c in columns if lo<=c['speed_kmh']<=hi]
        if not support:continue
        left=i>0 and corners[i-1]['constraint_corner']['structural_side']=='right'
        right=i<len(corners) and corners[i]['constraint_corner']['structural_side']=='left'
        def coordinate(x):
            x=np.asarray(x)
            if left and right:return np.arcsin(np.clip(2*(x-lo)/(hi-lo)-1.,-1.,1.))
            if left:return np.sqrt(np.maximum(0.,x-lo))
            if right:return -np.sqrt(np.maximum(0.,hi-x))
            return x
        mapped=[dict(c,speed_kmh=float(coordinate(c['speed_kmh'])),_coordinate_mapped=True) for c in support]
        yield chosen,mapped,coordinate(speeds[chosen])
