"""Interpolation on verified envelopes, with the physical boundary included.

Only numerical coordinates change. Knots are the original balanced aircraft
states, and independent trim holdouts determine the sampling accuracy.
"""
import numpy as np
from scipy.interpolate import PchipInterpolator


def same_deployment_branch(columns):
    """Do not borrow a cubic slope across the native 5% flap bypass.

    The deployment policy can jump there. Independent equilibria on either
    side are still valid, but the cross-jump secant is not a derivative of
    either smooth branch. This affects interpolation support only.
    """
    threshold=float(np.float32(.05))*100.
    branches={tuple(c[k]['flaps_percent']>threshold for k in ('lower_boundary','boundary'))
              for c in columns if all(c.get(k) and 'flaps_percent' in c[k]
                                      for k in ('lower_boundary','boundary'))}
    return len(branches)<=1


def local_support(columns,left,valid):
    indices=[left,left+1]
    core=columns[left:left+2]
    for i in (left-1,left+2):
        if (0<=i<len(columns) and valid[i]
                and (not same_deployment_branch(core) or same_deployment_branch(core+[columns[i]]))):
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
    """PCHIP derivative, batched over different queried load coordinates."""
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
    """Evaluate one PCHIP speed interval without a per-query spline object."""
    xs=np.asarray(xs);values=np.asarray(values);h=xs[left+1]-xs[left]
    t=(np.asarray(speeds)-xs[left])/h;t2=t*t;t3=t2*t
    return ((2*t3-3*t2+1)*values[left]+(t3-2*t2+t)*h*_derivative(xs,values,left)
            +(-2*t3+3*t2)*values[left+1]+(t3-t2)*h*_derivative(xs,values,left+1))


def normalized_values(column,u):
    """Evaluate the already checked column at an envelope coordinate."""
    from em_sampling import column_curve,load_coordinate
    low,high=column['lower_boundary']['load_g'],column['boundary']['load_g']
    if high-low<=1e-10:return np.full(np.shape(u),column['boundary']['ps_mps'])
    start=float(load_coordinate(low,high))
    return column_curve(column)[1][0][2](np.clip(start+(1-start)*u,start,1.))



def angle_surface(support,sx,left,speeds,loads,bottom,top):
    """Fit normalized physical angle, invert only the final load mapping.

    Load reaches a maximum at stall; alpha remains a regular parameter.
    All SEP values use the already checked load curves at actual columns.
    """
    from em_sampling import column_at_load
    curves=[]
    for column in support:
        points=[p for p in column['points'] if p['valid'] and p.get('surface_sample',True)
                and column['lower_boundary']['load_g']<=p['load_g']<=column['boundary']['load_g']]
        if len(points)<4 or np.any(np.diff([p['alpha_deg'] for p in points])<=1e-8):return None
        cached=column.get('_angle_map')
        if cached is None:
            cached=ParametricCurve(points,column['boundary']['load_g']);column['_angle_map']=cached
        curves.append(cached)
    def states(u):
        return np.asarray([curve.load(curve.angles[0]+u*(curve.angles[-1]-curve.angles[0])) for curve in curves])
    def mapped(u):return pchip_pair(sx,states(u),left,speeds)
    target=np.clip(loads,bottom,top);lo=np.zeros_like(target);hi=np.ones_like(target)
    u=np.sqrt(np.clip((target-bottom)/(top-bottom),0.,1.))
    for iteration in range(20):
        predicted=mapped(u);error=predicted-target
        if np.max(abs(error),initial=0.)<1e-10:break
        lo=np.where(error<0.,u,lo);hi=np.where(error>0.,u,hi)
        before=np.maximum(0.,u-1e-5);after=np.minimum(1.,u+1e-5)
        slope=(mapped(after)-mapped(before))/(after-before)
        with np.errstate(divide='ignore',invalid='ignore'):trial=u-error/slope
        u=np.where(abs(error)<1e-10,u,np.where(np.isfinite(trial)&(trial>=lo)&(trial<=hi),trial,(lo+hi)*.5))
    ns=states(u)
    powers=np.asarray([column_at_load(c,np.clip(n,c['lower_boundary']['load_g'],c['boundary']['load_g'])) for c,n in zip(support,ns)])
    return pchip_pair(sx,powers,left,speeds)


def fitted_interpolate(columns,speeds,loads):
    """Return values plus the speed intervals covered by verified envelopes.

    A normalized load coordinate aligns the two solved boundaries across
    speed, including the collapsed 1 g endpoint. Gaps and uncertified limits
    are excluded; callers retain their explicit legacy/gap behavior there.
    """
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
        limits=PchipInterpolator(sx,[[c['lower_boundary']['load_g'],c['boundary']['load_g']] for c in support],axis=0)(v)
        bottom,top=limits.T;n=grid[:,chosen]
        inside=(n>=bottom[None,:]-1e-12)&(n<=top[None,:]+1e-12)&(top[None,:]>1.)
        with np.errstate(invalid='ignore',divide='ignore'):
            start=load_coordinate(bottom,top)
            u=(load_coordinate(np.clip(n,bottom,top),top)-start)/(1-start)
        # Angle regularization is needed near a lift maximum. Applying that
        # nonlinear mapping to an ordinary structural/control cap can bend a
        # smooth fixed-load SEP curve between already accurate columns. Keep
        # load as the coordinate when every supporting cap is non-stall;
        # both physical edges remain aligned and independently checked.
        regular=all(c['boundary_reason'] in ('wing force','control') for c in support)
        predicted=None if regular else angle_surface(support,sx,indices.index(j-1),v,n,bottom,top)
        if predicted is None:
            values=np.asarray([normalized_values(c,u) for c in support])
            predicted=pchip_pair(sx,values,indices.index(j-1),v)
        out[:,chosen]=np.where(inside,predicted,np.nan);covered[chosen]=True
    return out,covered


def envelope_limits(columns,speeds):
    """Lower/upper loads from the same contiguous verified support."""
    return envelope_values(columns,speeds)[:,:2]


def mesh_limits(columns,speeds):
    """Continuous mesh coordinates from sampled loads, without certifying edges.

    A failed upper-limit search can still contain valid interior SEP samples.
    Keep their display rows aligned with neighboring columns. These coordinates
    grant no feasibility: the SEP interpolator still masks rejected/unknown
    states, and the outline still requires independently verified limits.
    """
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


def _envelope_values(columns,speeds):
    """Verified boundary coordinates and SEP, independent of interior gaps."""
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
            out[mask]=PchipInterpolator(xs[support],[values(group[i]) for i in support],axis=0)(speeds[mask])
    for c in columns+[None]:
        if (c is not None and (c.get('boundary_status') in ('verified limit','plot ceiling') or (c.get('boundary') or {}).get('native_branch_search')) and
                all(c.get(k) and c[k]['valid'] for k in ('lower_boundary','boundary'))):run.append(c)
        else:finish(run);run=[]
    for c in columns:
        if c.get('boundary') and c.get('lower_boundary'):out[speeds==c['speed_kmh']]=values(c)
    return out


class ParametricCurve:
    """A checked cubic in angle, inverted to the requested physical load."""
    def __init__(self,points,top):
        from scipy.interpolate import CubicSpline
        self.top=top
        self.angles=np.array([p['alpha_deg'] for p in points])
        self.loads=np.array([p['load_g'] for p in points])
        self.load=CubicSpline(self.angles,self.loads)
        self.power=CubicSpline(self.angles,[p['ps_mps'] for p in points])
        # An unconstrained cubic must not invent a reversal or an extra branch
        # near maximum lift. Test the minimum derivative on every polynomial.
        c=self.load.c;h=np.diff(self.angles)
        with np.errstate(divide='ignore',invalid='ignore'):
            vertex=np.clip(-c[1]/(3*c[0]),0.,h)
        vertex=np.where(np.isfinite(vertex),vertex,0.)
        d=lambda t:(3*c[0]*t+2*c[1])*t+c[2]
        if np.min([d(np.zeros_like(h)),d(h),d(vertex)])<0.:
            self.load=PchipInterpolator(self.angles,self.loads)
        # Unrestricted global cubics can amplify rounding noise between two
        # almost identical native trim points. Keep a cubic only when its
        # entire interval is monotone in the direction of its endpoint data.
        c=self.power.c;secants=np.diff([p['ps_mps'] for p in points])/h
        with np.errstate(divide='ignore',invalid='ignore'):
            vertex=np.clip(-c[1]/(3*c[0]),0.,h)
        vertex=np.where(np.isfinite(vertex),vertex,0.)
        derivatives=np.array([(3*c[0]*t+2*c[1])*t+c[2] for t in (np.zeros_like(h),h,vertex)])
        if np.any(derivatives*secants[None,:]<0.) or np.any((secants==0.)&np.any(derivatives!=0.,axis=0)):
            self.power=PchipInterpolator(self.angles,[p['ps_mps'] for p in points])
        self.slope=self.load.derivative()

    def __call__(self,u):
        from em_sampling import coordinate_load
        shape=np.shape(u);n=np.asarray(coordinate_load(u,self.top)).ravel()
        indices=np.clip(np.searchsorted(self.loads,n,side='right')-1,0,len(self.loads)-2)
        lo=self.angles[indices].copy();hi=self.angles[indices+1].copy()
        alpha=lo+(hi-lo)*np.clip((n-self.loads[indices])/(self.loads[indices+1]-self.loads[indices]),0.,1.)
        for _ in range(16):
            error=self.load(alpha)-n
            if np.max(abs(error),initial=0.)<1e-12:break
            lo=np.where(error<0.,alpha,lo);hi=np.where(error>0.,alpha,hi)
            slope=self.slope(alpha)
            with np.errstate(divide='ignore',invalid='ignore'):trial=alpha-error/slope
            alpha=np.where(abs(error)<=1e-12,alpha,
                np.where(np.isfinite(trial)&(trial>=lo)&(trial<=hi),trial,(lo+hi)*.5))
        return self.power(alpha).reshape(shape)


def _smooth_curve(points,top):
    from em_sampling import load_coordinate,interpolation_knots
    if len(points)>=4 and np.all(np.diff([p['alpha_deg'] for p in points])>1e-8):
        return ParametricCurve(points,top)
    ux,uy=interpolation_knots(load_coordinate([p['load_g'] for p in points],top),[p['ps_mps'] for p in points])
    return PchipInterpolator(ux,uy)


class BranchCurve:
    """Separate continuous fits on either side of native helper switches.

    Two verified edge samples bound each discontinuity. Only that narrow
    bracket is drawn as a linear jump; no smooth cubic crosses the switch.
    """
    def __init__(self,groups,top):
        from em_sampling import load_coordinate
        self.parts=[];self.transitions=[]
        previous=None
        for group in groups:
            lo,hi=[float(load_coordinate(p['load_g'],top)) for p in (group[0],group[-1])]
            curve=_smooth_curve(group,top) if len(group)>1 else None
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


def prepare_curve(points,top):
    groups=[]
    for point in points:
        if not groups or point.get('roll_leveling_branch')!=groups[-1][-1].get('roll_leveling_branch'):groups.append([])
        groups[-1].append(point)
    return BranchCurve(groups,top) if len(groups)>1 else _smooth_curve(points,top)


def coordinate_regions(columns,speeds):
    """Split at solved constraint corners and regularize the structural side.

    Close to maximum lift, alpha departs from its critical value as the square
    root of the distance from the corner. Transform only interpolation speed;
    raw aircraft speed, force, load and trim data remain unchanged. The same
    independent native holdouts check this representation before it is used.
    """
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
