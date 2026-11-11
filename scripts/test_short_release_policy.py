"""Chart policy checks: release threshold, unconditional released limits, native parity."""
import copy
import json
import math
import sys
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parent))

def threshold_checks():
    from wing_line import protection
    config=dict(instructor=True,structural_limits=True,instructor_protection=None)
    cases=[(.5,.1,'released',5.),(.5,.05,'released',10.),(.5,.04,'held',12.5),
           (.5,0.,'held',math.inf),(1.,.1,'held',math.inf)]
    for start,rate,state,seconds in cases:
        fm={'Instructor':dict(limitOverload=True,overloadTimeRange=[start,1.],overloadTimeRate=rate)}
        actual=protection(fm,config)
        assert actual==(state,seconds),(actual,state,seconds)
    for changed in (dict(instructor=False),dict(structural_limits=False)):
        assert protection({},dict(config,**changed))==(None,None)
    assert protection({'Instructor':{'limitOverload':False}},config)==(None,None)
    return len(cases)+3

def admission_checks():
    from em_solver import TrimSolver,settings
    import em_march_native as native
    from instructor_aoa import controller_limits
    module,status=native.load()
    assert module is not None,status
    solver=TrimSolver('f_16xl',settings(dict(aircraft=['f_16xl'],instructor=True,autotrim=True)))
    assert solver.instructor_protection=='released'
    free=copy.copy(solver);free.instructor_protection=None
    held=copy.copy(solver);held.instructor_protection='held'
    model=module.Model(solver);compiled=module.Instructor(model,solver)
    held_model=module.Model(held);held_compiled=module.Instructor(held_model,held)
    checks=0;cap_differences=0
    for speed in (400.,650.,680.,690.,700.,750.,900.,1200.):
        # Admission accepts trial states; compare the same state under each policy.
        x=[15.,80.,0.,.5,0.,16.,16.]
        value=model.record(speed/3.6,6.,x,False,True)
        released=controller_limits(solver,dict(value,instructor=None))
        unlimited=controller_limits(free,dict(value,instructor=None))
        protected=controller_limits(held,dict(value,instructor=None))
        fast=compiled.limits(dict(value,instructor=None),speed/3.6)
        fast_held=held_compiled.limits(dict(value,instructor=None),speed/3.6)
        for field in ('angle_limits_deg','effective_angle_limits_deg','control_bounds'):
            import numpy as np
            assert np.allclose(released[field],unlimited[field],rtol=0.,atol=1e-8),(speed,field)
            assert np.allclose(released[field],fast[field],rtol=0.,atol=1e-8),(speed,field,'native released')
            assert np.allclose(protected[field],fast_held[field],rtol=0.,atol=1e-8),(speed,field,'native held')
        assert protected['angle_limits_deg'][1]<=released['angle_limits_deg'][1]+1e-8
        cap_differences+=protected['angle_limits_deg'][1]<released['angle_limits_deg'][1]-1e-5
        checks+=1
    assert cap_differences>0,'Test must exercise a binding wing-protection reserve'
    return dict(states=checks,held_cap_differences=cap_differences)

if __name__=='__main__':
    print(json.dumps(dict(threshold_checks=threshold_checks(),admission=admission_checks(),passed=True)))
