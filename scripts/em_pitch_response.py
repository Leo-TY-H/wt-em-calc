"""Normal controllable-flight domain of the unmodified aircraft equations.

Pitch response is the change in actual pitch moment with delivered elevator,
at fixed attitude, velocity and angular rate. Native mixer inversion determines
its expected sign. This is control effectiveness, not static pitch stability.
Propeller samples retain the complete nonlinear aircraft/propulsion phases.
"""
import copy
import numpy as np
from scipy.optimize import brentq, minimize_scalar

REASON='reversed pitch response'
UNRESOLVED='pitch response unresolved'
KINDS={'post-stall':'stall','control authority':'control',
       'wing force limit':'wing force','Instructor pitch limit':'Instructor pitch',
       REASON:'pitch response'}
PHYSICAL_REASONS=frozenset(KINDS)


def response(solver,value):
    if '_pitch_response' in value:return value['_pitch_response']
    x=list(map(float,value['equilibrium_coordinates']));speed=value['speed'];load=value['equilibrium_load_g']
    sign=1. if solver.controls['invert_elevator'] else -1.
    scale=solver.weight*solver.fm['Length']
    samples=[]
    for step in (.002,.0005):
        lo=x.copy();hi=x.copy()
        lo[3]=max(-1.,x[3]-step);hi[3]=min(1.,x[3]+step)
        # Elevator does not enter the engine owner. Reuse these exact retained
        # phases at unchanged velocity/rates, then replay the full nonlinear
        # aircraft for each command. No mean-wash proxy or governor restart.
        a=solver.operating_point(speed,load,lo,propulsion_sample=value['propulsion'])
        b=solver.operating_point(speed,load,hi,propulsion_sample=value['propulsion'])
        width=hi[3]-lo[3]
        ma=a['result']['stored_moment'][2];mb=b['result']['stored_moment'][2]
        slope=sign*(mb-ma)/width
        # Estimate cancellation of the native float32 moment outputs. A sign
        # at this floor is numerical uncertainty, not a physical exclusion.
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
    """Bracket normal-direction pitch equilibria before full aircraft closure.

    A wrong-sign root is not cured by shrinking Newton steps. Search the
    bounded physical elevator coordinate for crossings in the normal direction,
    then rebalance every force and moment. No SEP ranking or aircraft constants.
    """
    if getattr(solver,'_pitch_response_recovery',False):return None
    x=list(map(float,value['equilibrium_coordinates']));speed=value['speed'];load=value['equilibrium_load_g']
    sign=1. if solver.controls['invert_elevator'] else -1.
    memo={}
    def at(command):
        if command not in memo:
            q=x.copy();q[3]=command
            memo[command]=solver.operating_point(speed,load,q,propulsion_sample=value['propulsion'])
        return sign*float(memo[command]['rate_residual'][2])
    commands=sorted(set(np.linspace(-1.,1.,9).tolist()+[x[3]]))
    values=[at(command) for command in commands]
    # Two close pitch roots can lie inside one coarse interval near a tail
    # lift maximum. Locate measured minima of the signed moment residual so
    # their negative-to-positive crossing is not missed by the seed grid.
    valleys=[(commands[i-1],commands[i+1]) for i in range(1,len(commands)-1)
             if values[i]<min(values[i-1],values[i+1]) and values[i]>=-5e-5]
    for lo,hi in valleys:
        fit=minimize_scalar(at,bounds=(lo,hi),method='bounded',options=dict(xatol=2e-6,maxiter=24))
        commands.append(float(fit.x))
    commands.sort();values=[at(command) for command in commands]
    brackets=[(a,b) for a,b,fa,fb in zip(commands,commands[1:],values,values[1:]) if fa<=0.<=fb and fa<fb]
    brackets.sort(key=lambda pair:abs((pair[0]+pair[1])*.5-x[3]))
    view=copy.copy(solver);view._pitch_response_recovery=True
    view.__dict__.pop('_trim_predictor',None)
    for lo,hi in brackets:
        try:command=brentq(at,lo,hi,xtol=2e-6,maxiter=24)
        except (ValueError,RuntimeError):continue
        q=x.copy();q[3]=command
        point=view.solve(speed*3.6,load,q,detailed=detailed,exhaustive=False,refine=refine)
        if point['converged'] and (point.get('pitch_response') or {}).get('normal') and 'post-stall' not in point['reasons']:
            point['pitch_branch_recovery']=dict(method='Normal-direction native pitch-moment bracket followed by full aircraft balance',
                rejected_solution=x,pitch_bracket=[lo,hi],evaluations=len(memo))
            point['evaluations']+=len(memo)
            return point
    return None
