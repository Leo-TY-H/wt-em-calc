import copy
import numpy as np
from scipy.optimize import brentq, minimize_scalar

REASON='reversed pitch response'
UNRESOLVED='pitch response unresolved'
KINDS={'post-stall':'stall','control authority':'control',
       'wing force limit':'wing force','Instructor pitch limit':'Instructor pitch',
       'engine RPM limit':'engine RPM',
       REASON:'pitch response'}
PHYSICAL_REASONS=frozenset(KINDS)


def _history_closed(value):
    # Only the stationary conditional evaluator declares this contract. Keep
    # the existing nested-baseline response path unchanged. Conditional probes
    # use the same admitted history error as ordinary stationary points.
    if '_stationary_frozen_history' not in value:return True
    error=value.get('history_error',float('inf'))
    residual=np.asarray(value.get('stationary_angle_residual',[float('inf')]))
    return (np.isfinite(error) and 0.<=error<=2e-4
            and residual.shape==(2,) and np.isfinite(residual).all()
            and np.max(abs(residual))<=2e-4)


class _HistoryUnclosed(Exception):
    """A conditional history probe cannot supply a stationary moment."""


def response(solver,value):
    if '_pitch_response' in value:return value['_pitch_response']
    x=list(map(float,value['equilibrium_coordinates']));speed=value['speed'];load=value['equilibrium_load_g']
    sign=1. if solver.controls['invert_elevator'] else -1.
    scale=solver.weight*solver.fm['Length']
    samples=[]
    evaluate=solver.operating_point
    if (getattr(solver,'config',{}).get('trim_solver_mode')=='stationary'
            and not solver.config.get('experimental_instantaneous_aero',False)):
        from em_stationary_baseline import stationary_frozen_point
        core=solver.core() if callable(getattr(solver,'core',None)) else solver
        def evaluate(speed,load,q,**kwargs):
            return stationary_frozen_point(core,speed,load,q,
                history_initial=value.get('history_input',{}).get('wing_aoa'),**kwargs)
    for step in (.002,.0005):
        lo=x.copy();hi=x.copy()
        lo[3]=max(-1.,x[3]-step);hi[3]=min(1.,x[3]+step)


        a=evaluate(speed,load,lo,propulsion_sample=value['propulsion'])
        b=evaluate(speed,load,hi,propulsion_sample=value['propulsion'])
        if not _history_closed(a) or not _history_closed(b):
            result=dict(normal=False,reversed=False,unresolved=True,reason=UNRESOLVED,
                detail='Conditional wing history did not close',
                margin=float('nan'),gain_rad_s2=float('nan'),samples=samples,
                failed_step=step,method='Native steady pitch response unresolved')
            value['_pitch_response']=result
            return result
        width=hi[3]-lo[3]
        ma=a['result']['stored_moment'][2];mb=b['result']['stored_moment'][2]
        slope=sign*(mb-ma)/width


        rounding=8.*(abs(float(np.spacing(np.float32(ma))))+
                     abs(float(np.spacing(np.float32(mb)))))/width
        samples.append(dict(step=step,gain_rad_s2=float(slope/solver.mass['inertia'][2]),
                            margin=float(slope/scale),rounding=float(rounding/scale)))
        if abs(slope)>32.*rounding and abs(slope/scale)>.002:break
    last=samples[-1]
    normal=all(p['margin']>p['rounding'] for p in samples)
    reversed_response=all(p['margin']<-p['rounding'] for p in samples)
    result=dict(normal=normal,reversed=reversed_response,margin=last['margin'],
                gain_rad_s2=last['gain_rad_s2'],samples=samples,
                method='Native steady pitch moment derivative at fixed flight state; normal control sign')
    value['_pitch_response']=result
    return result


def recover(solver,value,*,detailed=False,refine=False):
    from em_trim_search import run_generator
    return run_generator(recover_attempt(solver,value,detailed=detailed,refine=refine))


def recover_attempt(solver,value,*,detailed=False,refine=False):
    from em_trim_search import Request
    if getattr(solver,'_pitch_response_recovery',False):return None
    x=list(map(float,value['equilibrium_coordinates']));speed=value['speed'];load=value['equilibrium_load_g']
    sign=1. if solver.controls['invert_elevator'] else -1.
    memo={}
    unavailable=set()
    def at(command):
        if command not in memo:
            q=x.copy();q[3]=command
            memo[command]=solver.operating_point(speed,load,q,propulsion_sample=value['propulsion'])
        if not _history_closed(memo[command]):
            unavailable.add(command)
            raise _HistoryUnclosed()
        return sign*float(memo[command]['rate_residual'][2])
    def available_at(command):
        try:return at(command)
        except _HistoryUnclosed:return float('nan')
    commands=sorted(set(np.linspace(-1.,1.,9).tolist()+[x[3]]))
    values=[available_at(command) for command in commands]


    valleys=[(commands[i-1],commands[i+1]) for i in range(1,len(commands)-1)
             if np.isfinite(values[i-1:i+2]).all()
             and values[i]<min(values[i-1],values[i+1]) and values[i]>=-5e-5]
    for lo,hi in valleys:
        try:fit=minimize_scalar(at,bounds=(lo,hi),method='bounded',options=dict(xatol=2e-6,maxiter=24))
        except _HistoryUnclosed:continue
        commands.append(float(fit.x))
    # A failed interior probe breaks the sampled bracket as well as the local
    # minimization. Never join its two closed neighbors across the missing state.
    commands=sorted(set(commands)|unavailable)
    values=[available_at(command) for command in commands]
    brackets=[(a,b) for a,b,fa,fb in zip(commands,commands[1:],values,values[1:])
              if np.isfinite([fa,fb]).all() and fa<=0.<=fb and fa<fb]
    brackets.sort(key=lambda pair:abs((pair[0]+pair[1])*.5-x[3]))
    view=copy.copy(solver);view._pitch_response_recovery=True
    view.__dict__.pop('_trim_predictor',None)
    for lo,hi in brackets:
        try:command,fit=brentq(at,lo,hi,xtol=2e-6,maxiter=24,full_output=True,disp=False)
        except _HistoryUnclosed:continue
        if not fit.converged:continue
        q=x.copy();q[3]=command
        point=(yield Request(view,speed*3.6,load,q,detailed=detailed,exhaustive=False,refine=refine))
        if point['converged'] and (point.get('pitch_response') or {}).get('normal') and 'post-stall' not in point['reasons']:
            point['pitch_branch_recovery']=dict(method='Normal-direction native pitch-moment bracket followed by full aircraft balance',
                rejected_solution=x,pitch_bracket=[lo,hi],evaluations=len(memo))
            point['evaluations']+=len(memo)
            return point
    return None
