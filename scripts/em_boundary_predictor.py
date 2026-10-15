import math
import numpy as np
from em_pitch_response import PHYSICAL_REASONS


def coupled_limit(solver,speed,seed):
    from em_continuation import TrimCurve
    limits=sorted((p for p in seed if p.get('envelope_limit')),key=lambda p:p['speed_kmh'])
    if len(limits)!=2 or not limits[0]['speed_kmh']<speed<limits[1]['speed_kmh']:return None
    left,right=limits
    from em_mach_events import catalog,crossings
    if crossings(catalog(solver)['events'],left,right):return None
    kind=left['envelope_limit']['kind']
    if kind!=right['envelope_limit']['kind']:return None
    if any(p.get('sideslip_attitude_deg',0.) for p in limits):return None
    pass
    if kind=='control':
        def stop(p):
            return min((abs(c-b),i,j) for i,(c,bounds) in enumerate(zip(p['commands'],p['control_bounds']))
                for j,b in enumerate(bounds))[1:]
        if stop(left)!=stop(right):return None
    if (left['flaps_percent']>float(np.float32(.05))*100)!=(right['flaps_percent']>float(np.float32(.05))*100):return None
    t=(speed-left['speed_kmh'])/(right['speed_kmh']-left['speed_kmh'])
    curve=TrimCurve(solver,speed,left if t<.5 else right)
    x,load=curve.decode((1-t)*curve.state(left)+t*curve.state(right))
    initial=dict(left,speed_kmh=speed,solution=x.tolist(),load_g=load,alpha_deg=float(x[0]))
    point=curve.limit(initial,kind,max_iterations=6,max_refreshes=2,max_evaluations=12)
    if point is None:return None
    point['envelope_limit']['seed_method']='neighboring active-constraint continuation'
    return point,point,[point]


def continue_limit(solver,speed,seed):
    limits=sorted((p for p in seed if p.get('envelope_limit')),key=lambda p:p['speed_kmh'])
    if len(limits)!=2 or not limits[0]['speed_kmh']<speed<limits[1]['speed_kmh']:return None
    left,right=limits
    from em_mach_events import catalog,crossings
    if crossings(catalog(solver)['events'],left,right):return None
    def active(point):
        limit=point['envelope_limit']
        candidates={limit['kind'],*limit.get('competing_constraints',[])}


        margins={'wing force':1.-max(point['wing_load_ratios']),
                 'control':point['authority_margin'],
                 'Instructor pitch':(point.get('instructor') or {}).get('envelope_margin',float('inf'))}
        return {kind for kind in candidates if kind in margins and
                (kind==limit['kind'] or abs(margins[kind])<=2e-4)}
    common=active(left)&active(right)
    kind=next((kind for kind in ('Instructor pitch','wing force','control') if kind in common),None)
    if kind is None:return None
    if kind=='control':
        def stop(point):
            return min((abs(command-bound),axis,side)
                for axis,(command,bounds) in enumerate(zip(point['commands'],point['control_bounds']))
                for side,bound in enumerate(bounds))[1:]
        if stop(left)!=stop(right):return None
    if any(p.get('sideslip_attitude_deg',0.) for p in limits):return None
    if (left['flaps_percent']>float(np.float32(.05))*100)!=(right['flaps_percent']>float(np.float32(.05))*100):return None
    def margin(point):
        if kind=='wing force':return 1.-max(point['wing_load_ratios'])
        if kind=='control':return point['authority_margin']
        ctl=point.get('instructor')
        return ctl['envelope_margin'] if ctl and ctl['converged'] else None
    gradients=[];state_gradients=[];ps_gradients=[]
    for edge in limits:
        candidates=[p for p in seed if p['valid'] and p['speed_kmh']==edge['speed_kmh'] and
                    max(1.,edge['load_g']*.7)<p['load_g']<edge['load_g']-.005 and margin(p) is not None]
        if not candidates or margin(edge) is None:return None
        interior=min(candidates,key=lambda p:abs(p['load_g']-.9*edge['load_g']))
        width=edge['load_g']-interior['load_g']
        slope=(margin(edge)-margin(interior))/width
        if not slope<0:return None
        gradients.append(slope)
        state_gradients.append((np.asarray(edge['solution'])-interior['solution'])/width)
        ps_gradients.append(abs(edge['ps_mps']-interior['ps_mps'])/width)
    t=(speed-left['speed_kmh'])/(right['speed_kmh']-left['speed_kmh'])
    load=left['load_g']*(1-t)+right['load_g']*t
    guess=np.asarray(left['solution'])*(1-t)+np.asarray(right['solution'])*t
    direction=state_gradients[0]*(1-t)+state_gradients[1]*t
    slope=gradients[0]*(1-t)+gradients[1]*t
    ps_slope=max(ps_gradients)
    from em_accuracy import turn_tolerance
    rate_target=(1. if solver.config.get('trim_solver_mode')=='stationary' else .25)*turn_tolerance(solver.config['sep_tolerance_mps'])
    allowed=PHYSICAL_REASONS
    low=high=previous=None;observations=[]
    lower=max(1.00001,min(p['load_g'] for p in limits)*.8)
    from em_load_limits import ceiling
    upper=min(ceiling(solver,speed),max(p['load_g'] for p in limits)*1.2)
    if not lower<load<upper:return None
    for iteration in range(4):
        guess[1]=math.degrees(math.acos(1./load))
        from em_sampling import nearby_initial
        point=solver.solve(speed,load,nearby_initial(solver,speed,load,[*seed,*observations],guess.tolist()),exhaustive=False,quick=True)
        observations.append(point)
        if not point['converged'] or not set(point['reasons'])<=allowed:return None
        value=margin(point)
        if value is None:return None
        if point['valid']:
            if low is None or load>low['load_g']:low=point
        elif point['reasons']:
            if high is None or load<high['load_g']:high=point
        else:return None
        if low is not None and high is not None and low['load_g']<high['load_g']:
            from em_constraint_bracket import refine
            result=refine(solver,speed,low,high)
            if result:
                result['envelope_limit']['seed_method']='constraint slope continued from neighboring physical boundaries'
                result['envelope_limit']['continuation_observations']=len(observations)
                return low,result,observations
            return None
        if previous is not None:
            prior_n,prior_value=previous
            if abs(load-prior_n)>1e-6:
                measured=(value-prior_value)/(load-prior_n)
                if measured<0:slope=measured


        dwdn=math.degrees(9.8100004196167*load/math.sqrt(load*load-1.)/(speed/3.6))
        width=min(.8*rate_target/dwdn,.16*solver.config['sep_tolerance_mps']/max(ps_slope,1e-12))
        target=load-value/slope+(.5*width if point['valid'] else -.5*width)
        target=float(np.clip(target,lower,upper))
        if abs(target-load)<1e-7:return None
        guess=np.asarray(point['solution'])+direction*(target-load)
        previous=(load,value);load=target
    return None
