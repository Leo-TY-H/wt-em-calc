"""Numerical initializer for the unchanged healthy propulsion frame map.

The native drivetrain is a small coupled dynamical system. Solve its retained
state simultaneously instead of waiting for every slow governor/compressor
transient. This produces an initializer only: the normal frame-map recurrence
certificate and complete aircraft phase replay remain the acceptance gates.
Stochastic maps, active stops and non-attracting roots use ordinary integration.
"""
import math
import numpy as np
from component_assembly import f32
from propulsion_general import step


def propose(p,state,velocity,height,body_omega,cg,dt,nitro,torque_gyro,allow_long_map=True):
    """Return a locally attracting stationary proposal, or None.

    All frame outputs, including the finite-difference Jacobian, come from the
    original equations. No prescribed RPM, altered gain or time step is used.
    Only connected healthy piston graphs currently have a closed state adapter;
    other installation graphs retain the general time integrator.
    """
    if not p['transmissions'] or any(e['family'] not in (0,1) for e in p['engines']):return None
    if {l['index'] for t in p['transmissions'] for l in t['engines']}!=set(range(len(p['engines']))):return None
    paths=[];scales=[];bounds=[]
    for i,t in enumerate(state['transmissions']):
        limit=min(p['engines'][l['index']]['properties']['omega_limit']*l['inverse_ratio'] for l in p['transmissions'][i]['engines'])
        for k in ('omega','previous_omega'):
            paths.append(('transmissions',i,k));scales.append(300.);bounds.append((1.,2.*limit))
    for i,s in enumerate(state['propellers']):
        pp=p['propellers'][i]['properties']
        if pp['governor']:
            paths.append(('propellers',i,'pitch'));scales.append(1.);bounds.append((pp['pitch_min'],pp['pitch_max']))
            if not pp['governor_fast']:
                paths.append(('propellers',i,'governor_pitch'));scales.append(1.);bounds.append((-math.pi,pp['pitch_max']))
        for j in ((0,1) if pp['coaxial'] else (0,2)):
            paths.append(('propellers',i,'flow',j));scales.append(20.);bounds.append((-150.,150.) if j<2 else (0.,20.))
    for i,e in enumerate(state['engines']):
        paths.append(('engines',i,'regulator'));scales.append(1.);bounds.append((0.,1.))
        if p['engines'][i]['properties']['compressor_type']==3:
            paths.append(('engines',i,'turbo'));scales.append(100.);bounds.append((0.,math.inf))
    scales=np.asarray(scales);bounds=np.asarray(bounds)/scales[:,None]
    calls=0
    def advance(s):
        nonlocal calls
        calls+=1
        result=step(p,s,velocity,height,body_omega,cg,dt,s.get('seed',12345),nitro,torque_gyro=torque_gyro)
        if result['seed']!=s.get('seed',12345):raise ValueError('Stochastic native frame')
        return result
    def pack(s):
        return np.asarray([s[a][i][k] if len(path)==3 else s[a][i][k][path[3]]
                           for path in paths for a,i,k in [path[:3]]])/scales
    def unpack(x):
        result=dict(state,engines=[dict(e) for e in state['engines']],transmissions=[dict(t) for t in state['transmissions']],
                    propellers=[dict(s,flow=list(s['flow'])) for s in state['propellers']])
        for path,value in zip(paths,x*scales):
            row=result[path[0]][path[1]]
            if len(path)==3:row[path[2]]=f32(value)
            else:row[path[2]][path[3]]=f32(value)
        return result
    def evaluate(x):
        s=unpack(x);result=advance(s)
        if [e.get('gear',0) for e in result['engines']]!=[e.get('gear',0) for e in state['engines']]:raise ValueError('Compressor branch change')
        return pack(result)-pack(s),s
    def jacobian(x,r,delta=1e-4):
        matrix=np.empty((len(x),len(x)))
        rounded=pack(unpack(x))
        for i in range(len(x)):
            q=x.copy();q[i]+=delta
            actual_step=pack(unpack(q))[i]-rounded[i]
            matrix[:,i]=(evaluate(q)[0]-r)/actual_step
        return matrix
    try:
        # Resolve the directly assigned throttle and selected compressor gear
        # before treating the continuous retained fields as unknowns.
        state=advance(state);x=pack(state);r,best=evaluate(x);norm=float(np.linalg.norm(r))
        if norm<2e-8:return None  # Ordinary recurrence closes this cheaply.
        initial=x.copy();matrix=jacobian(x,r);iterations=0
        def rounded_root():
            # An initializer need not resolve a float32 map below its own
            # rounding increments. The subsequent unchanged native recurrence
            # or mean certificate decides whether it is a settled solution.
            return bool(np.all(abs(r)<=4*np.finfo(np.float32).eps*np.maximum(abs(x),.01)))
        for iteration in range(8):
            direction=np.linalg.lstsq(matrix,-r,rcond=1e-8)[0]
            maximum=max(abs(direction),default=0.)
            if maximum>.1:direction*=.1/maximum
            improved=False
            for scale in (1.,.5,.25):
                q=np.clip(x+scale*direction,bounds[:,0],bounds[:,1])
                if not np.isfinite(q).all():continue
                rr,ss=evaluate(q);nn=float(np.linalg.norm(rr))
                if nn<norm:
                    dx=q-x;dr=rr-r;den=float(dx.dot(dx))
                    if den:matrix+=np.outer(dr-matrix.dot(dx),dx)/den
                    x=q;r=rr;norm=nn;best=ss;iterations+=1;improved=True;break
            if rounded_root():break
            if not improved:
                if iteration>3:return None
                matrix=jacobian(x,r)
        if not rounded_root() or max(abs(x-initial),default=0.)>.2:return None
        # Check the actual local time-update map, including previous shaft RPM.
        # A Newton root of an unstable governor must not replace its limit cycle.
        radii=[]
        # A 0.001 shaft-coordinate step can cross the governor's native
        # rate limiter even at a stationary root. That secant describes a
        # saturated transient, not the local derivative. Resolve the smaller
        # neighborhood at three scales; ambiguous signs still use F^k below.
        for delta in (3e-4,1e-4,5e-5):
            matrix=jacobian(x,r,delta)
            radius=float(max(abs(np.linalg.eigvals(matrix+np.eye(len(x)))),default=0.))
            if not math.isfinite(radius) or radius>=1.005:return None
            radii.append(radius)
        # Reject an unresolved stability sign. A fixed .995 cutoff rejects
        # genuinely slow attracting governors, precisely those for which
        # direct equilibrium is useful. Bound the measured step-size scatter
        # as well as requiring each independent linearization to contract.
        radius=max(radii);scatter=radius-min(radii)
        long_map=None
        if radius+scatter>=1.:
            if not allow_long_map:return None
            # A one-frame finite difference can lose a weak decay rate in
            # float32 rounding. Differentiate several ORIGINAL frame updates
            # together: at a fixed point D(F^k)=(DF)^k, so the same attraction
            # has a resolvable margin without changing the game's time step.
            # A clearly unstable one-frame map was rejected above. Keep each
            # perturbed trajectory local; saturation onto a remote cycle must
            # never make an unstable stationary root appear attracting.
            for frames in (4,16,64,128):
                def mapped(q):
                    s=unpack(q)
                    for _ in range(frames):
                        s=advance(s)
                        if [e.get('gear',0) for e in s['engines']]!=[e.get('gear',0) for e in state['engines']]:
                            raise ValueError('Compressor branch change')
                        if max(abs(pack(s)-x),default=0.)>.05:raise ValueError('Nonlocal stability trial')
                    return pack(s)
                long_radii=[]
                for delta in (5e-4,3e-4,1.5e-4):
                    matrix=np.column_stack([(mapped(x+np.eye(len(x))[i]*delta)-
                        mapped(x-np.eye(len(x))[i]*delta))/(2*delta) for i in range(len(x))])
                    long_radii.append(float(max(abs(np.linalg.eigvals(matrix)),default=0.)))
                high=max(long_radii);spread=high-min(long_radii)
                if math.isfinite(high) and high+spread<1.:
                    long_map=dict(frames=frames,spectral_radii=long_radii,step_scatter=spread)
                    break
            if long_map is None:return None
        return best,dict(method='coupled native frame fixed-point initializer',frame_evaluations=calls,
                         newton_iterations=iterations,residual=norm,local_spectral_radius=radius,
                         stability_radii=radii,stability_step_scatter=scatter,multi_frame_stability=long_map)
    except (ValueError,OverflowError,ZeroDivisionError,np.linalg.LinAlgError):return None
