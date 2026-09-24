"""Safeguarded refinement of an observed, balanced physical limit crossing."""
import math
from em_pitch_response import PHYSICAL_REASONS


def refine(solver,speed,low,high):
    allowed=PHYSICAL_REASONS
    if not (low['valid'] and high['converged'] and high['reasons'] and
            set(high['reasons'])<=allowed and high['load_g']>low['load_g']):return None
    if 'post-stall' in high['reasons']:
        # Load is flat at maximum lift. A tiny load bracket can still span
        # a large angle/SEP range and cannot locate the stall endpoint.
        # Let boundary() solve the actual stall-angle equation with load free.
        return None
    # Keep actual balanced endpoints. Float32 controller command steps need
    # not contain an exact zero; their crossing is still a physical bracket.
    initial=(low['load_g'],high['load_g']);evaluations=0
    def margins(point):
        result={'control':point['authority_margin'],'stall':point['stall_margin_deg']/10.}
        pitch=point.get('pitch_response')
        if pitch is not None:result['pitch response']=pitch['margin']
        if solver.config['structural_limits']:result['wing force']=1.-max(point['wing_load_ratios'])
        if solver.config['instructor']:
            ctl=point.get('instructor')
            if ctl is not None:
                if not ctl['converged']:return None
                result['Instructor pitch']=ctl['envelope_margin']
                result['control']=ctl['margins']['control authority with auto trim']
            elif not set(point['reasons'])&(PHYSICAL_REASONS-{'Instructor pitch limit'}):return None
        return result
    lo_m,hi_m=margins(low),margins(high)
    if lo_m is None or hi_m is None:return None
    def rate(point):return math.degrees(9.8100004196167*math.sqrt(max(0.,point['load_g']**2-1.))/(speed/3.6))
    from em_accuracy import turn_tolerance
    rate_target=.25*turn_tolerance(solver.config['sep_tolerance_mps'])
    ps_target=.2*solver.config['sep_tolerance_mps']
    def closed():
        return (rate(high)-rate(low)<=rate_target and
                (high['load_g']-low['load_g']<=.0005 or abs(high['ps_mps']-low['ps_mps'])<=ps_target))
    # TOMS 748 uses the observed constraint curvature to shrink a balanced
    # bracket. Every function value remains a complete physical trim. A bad
    # chosen coordinate abandons this acceleration while retaining the newly
    # certified endpoints for the existing alternative-coordinate fallback.
    class ClosedBracket(Exception):pass
    class UnresolvedTrim(Exception):pass
    def constraint(load):
        nonlocal low,high,lo_m,hi_m,evaluations
        if closed():raise ClosedBracket()
        if load==low['load_g']:return min(lo_m.values())
        if load==high['load_g']:return min(hi_m.values())
        if not low['load_g']<load<high['load_g']:raise UnresolvedTrim()
        t=(load-low['load_g'])/(high['load_g']-low['load_g'])
        guess=[a*(1-t)+b*t for a,b in zip(low['solution'],high['solution'])]
        guess[1]=math.degrees(math.acos(1./load))
        if solver.is_prop and solver.engine.automatic:
            seeds=[p for p in (low,high) if p.get('_propulsion_seed')]
            if seeds:
                from em_data import clone
                solver.engine._warm=clone(min(seeds,key=lambda p:abs(p['load_g']-load))['_propulsion_seed'])
        point=solver.solve(speed,load,guess,exhaustive=False,quick=True);evaluations+=1
        values=margins(point) if point['converged'] and set(point['reasons'])<=allowed else None
        if values is None:raise UnresolvedTrim()
        if point['valid']:low,lo_m=point,values
        elif point['reasons']:high,hi_m=point,values
        else:raise UnresolvedTrim()
        return min(values.values())
    if min(lo_m.values())>0. and min(hi_m.values())<0. and not closed():
        from scipy.optimize import toms748
        try:toms748(constraint,low['load_g'],high['load_g'],xtol=1e-6,maxiter=6,disp=False)
        except (ClosedBracket,UnresolvedTrim,ValueError):pass
    for iteration in range(24):
        width=high['load_g']-low['load_g']
        if closed():
            active=[kind for kind,margin in hi_m.items() if margin<0.]
            kind='Instructor pitch' if 'Instructor pitch' in active else min(hi_m,key=hi_m.get)
            if hi_m[kind]>=0.:return None
            point=dict(low)
            axis=(2+min(range(3),key=lambda i:min(point['commands'][i]-point['control_bounds'][i][0],
                point['control_bounds'][i][1]-point['commands'][i]))) if kind=='control' else None
            point['envelope_limit']=dict(kind=kind,axis=axis,limiting_load_g=point['load_g'],
                limiting_load_interval_g=[point['load_g'],high['load_g']],
                constraint_residual=lo_m.get(kind),rejected_constraint_residual=hi_m[kind],
                evaluations=evaluations,initial_bracket_g=list(initial),
                active_constraints=active,
                turn_interval_dps=[rate(low),rate(high)],turn_target_dps=rate_target,
                ps_interval_mps=[low['ps_mps'],high['ps_mps']],ps_target_mps=ps_target,
                rejected_endpoint={key:high[key] for key in ('speed_kmh','load_g','solution','valid','converged',
                    'reasons','force_error_g','angular_error_rad_s2','history_error','stall_margin_deg','wing_load_ratios')},
                method='balanced physical constraint bracket')
            return point
        # Secant proposals accelerate smooth constraints. Safeguarding and
        # periodic bisection guarantee shrinkage across native discontinuities.
        f0=min(lo_m.values());f1=min(hi_m.values())
        fraction=f0/(f0-f1) if f0>0. and f1<0. and iteration%3!=2 else .5
        fraction=max(.1,min(.9,fraction));point=None;values=None
        # One chosen load can have a difficult governor phase or rounded
        # branch. The bracket permits other interior coordinates. Preserve
        # its balanced endpoints and try those before discarding it for a
        # much larger coupled aircraft/controller search.
        for quick in (True,False):
            for t in dict.fromkeys((fraction,.5,.25,.75)):
                load=low['load_g']+t*width
                guess=[a*(1-t)+b*t for a,b in zip(low['solution'],high['solution'])]
                guess[1]=math.degrees(math.acos(1./load))
                if solver.is_prop and solver.engine.automatic:
                    seeds=[p for p in (low,high) if p.get('_propulsion_seed')]
                    if seeds:
                        from em_data import clone
                        seed=min(seeds,key=lambda p:abs(p['load_g']-load))
                        solver.engine._warm=clone(seed['_propulsion_seed'])
                        if not quick:solver.engine.clear_cached_conditions()
                trial=solver.solve(speed,load,guess,exhaustive=False,quick=quick);evaluations+=1
                if trial['converged'] and set(trial['reasons'])<=allowed:
                    values=margins(trial)
                    if values is not None:point=trial;break
            if point is not None:break
        if point is None:return None
        if point['valid']:low,lo_m=point,values
        elif point['reasons']:high,hi_m=point,values
        else:return None
    return None
