"""Find a balanced controller bracket before a near-level active-limit solve.

Stall and Instructor limits can be separated by only millionths of a g at the
level-flight endpoint. A far rejected probe can cross both. Repeated coupled
constraint searches there are unnecessary: contract the regular turn coordinate
until the original balances certify the controller crossing. Failed probes only
guide search; they never serve as evidence of a physical limit.
"""
import math

def bracket(solver,speed,low,high):
    if not (low['valid'] and 1.<=low['load_g']<1.05 and high['load_g']>low['load_g'] and
            solver.config['instructor'] and low.get('instructor',{}).get('envelope_margin',1.)<.04):return None
    from em_constraint_bracket import refine
    left=low;upper=math.sqrt(high['load_g']**2-1.)
    for i in range(12):
        u=math.sqrt(max(0.,left['load_g']**2-1.));trial_u=(u+upper)*.5
        if trial_u-u<1e-7:return None
        guess=list(left['solution']);guess[1]+=math.degrees(math.atan(trial_u)-math.atan(u))
        p=solver.solve(speed,math.hypot(1.,trial_u),guess,exhaustive=False,quick=True)
        if (p['converged'] and p['reasons']==['Instructor pitch limit'] and
                p.get('instructor',{}).get('converged')):
            result=refine(solver,speed,left,p)
            if result:
                result['envelope_limit']['bracket_seed_method']='Contract turn coordinate using balanced aircraft before controller crossing'
                result['envelope_limit']['bracket_seed_evaluations']=i+1
                return result
            return None
        if p['valid']:left=p
        else:upper=trial_u
    return None


def level_response_edge(solver,left,right):
    """Retain a measured normal/reversed 1-g crossing at chart accuracy.

    A native piecewise moment derivative need not have a zero to optimize.
    Both sides must be balanced aircraft, close in attitude and controls, and
    differ only in the required normal pitch-response condition. Numerical
    failure is not an endpoint and never supplies a constraint sign.
    """
    import copy
    from em_accuracy import speed_tolerance
    bad=next((p for p in left['points'] if p['load_g']==1.),None)
    good=right.get('boundary')
    if not (bad and good and bad['converged'] and bad['reasons']==['reversed pitch response'] and
            good['valid'] and good['load_g']==1. and
            (bad.get('pitch_response') or {}).get('reversed') and
            (good.get('pitch_response') or {}).get('normal')):return None
    if not bad['speed_kmh']<good['speed_kmh']:return None
    view=copy.copy(solver);view._pitch_response_recovery=True
    view.__dict__.pop('_trim_predictor',None)
    target_speed=.25*speed_tolerance(solver.config)
    target_ps=.2*solver.config['sep_tolerance_mps']
    evaluations=0
    def nearby():
        return (abs(good['alpha_deg']-bad['alpha_deg'])<.25 and
                abs(good['bank_deg']-bad['bank_deg'])<.5 and
                max(abs(a-b) for a,b in zip(good['solution'][2:],bad['solution'][2:]))<.03)
    def closed():
        return (good['speed_kmh']-bad['speed_kmh']<=target_speed and
                abs(good['ps_mps']-bad['ps_mps'])<=target_ps)
    if not nearby():return None
    for _ in range(16):
        if closed():break
        speed=(bad['speed_kmh']+good['speed_kmh'])*.5
        guess=[(a+b)*.5 for a,b in zip(bad['solution'],good['solution'])]
        trial=view.solve(speed,1.,guess,exhaustive=False,quick=True);evaluations+=1
        if trial['valid'] and (trial.get('pitch_response') or {}).get('normal'):good=trial
        elif (trial['converged'] and trial['reasons']==['reversed pitch response'] and
                (trial.get('pitch_response') or {}).get('reversed')):bad=trial
        else:return None
        if not nearby():return None
    if not closed():return None
    result=dict(good)
    result['surface_sample']=True
    result['envelope_limit']=dict(kind='pitch response',axis=None,limiting_load_g=1.,
        method='Balanced normal/reversed pitch-response bracket in level-flight speed',
        limiting_speed_interval_kmh=[bad['speed_kmh'],good['speed_kmh']],
        speed_target_kmh=target_speed,ps_target_mps=target_ps,
        ps_interval_mps=[bad['ps_mps'],good['ps_mps']],
        constraint_residual=good['pitch_response']['margin'],
        rejected_constraint_residual=bad['pitch_response']['margin'],evaluations=evaluations,
        rejected_endpoint={k:bad[k] for k in ('speed_kmh','load_g','solution','valid','converged','reasons',
            'force_error_g','angular_error_rad_s2','history_error','pitch_response')})
    return result
