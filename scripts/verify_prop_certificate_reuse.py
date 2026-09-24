"""A native engine certificate survives the search-to-final handoff intact."""
import copy,json,math
import numpy as np
from pathlib import Path
from em_solver import TrimSolver,settings
import prop_em


def main():
    solver=TrimSolver('p-63a-10',settings(dict(aircraft=['p-63a-10'])))
    engine=solver.engine
    original=prop_em.settled_cycle;calls=[]
    def recorded(*args,**kwargs):
        result=original(*args,**kwargs)
        calls.append(copy.deepcopy(result))
        calls[-1]['_input_state']=copy.deepcopy(kwargs.get('state'))
        return result
    prop_em.settled_cycle=recorded
    velocity=(218.94866943359375,7.034461498260498,0.)
    omega=(0.,0.,0.)
    try:
        search=engine.condition(velocity,omega,cycle_seconds=60.)
        assert search['converged'] and search.get('stationarity'),search
        assert search['cycle_samples'] is None
        count=len(calls)
        final=engine.condition(velocity,omega,cycle_seconds=20.,preserve_cycle_samples=True)
        assert len(calls)==count,'Final validation repeated a completed native settle'
        assert final['cycle_samples']==calls[-1]['cycle_samples']
        assert len(final['cycle_samples'])>1
        assert final['state']==search['state']==engine._warm
        for key in ('force','moment','wash','stationarity','simulated_seconds'):
            assert final[key]==search[key]==calls[-1][key],key
        # Display coordinates do not enter automatic engine controls. Native
        # rounded flow/rates and this branch's commands remain the cache key.
        display=engine.condition(velocity,omega,flaps=.3,speed=200.,cycle_seconds=180.,preserve_cycle_samples=True)
        assert len(calls)==count,'Unused display coordinates repeated the same automatic engine calculation'
        assert display==final
        rounded=engine.condition(tuple(math.nextafter(v,math.inf) for v in velocity),
            tuple(math.nextafter(w,math.inf) for w in omega),preserve_cycle_samples=True)
        assert len(calls)==count and rounded==final,'Sub-ulp flow/rate changes repeated an identical native calculation'
        changed=list(omega);changed[1]=float(np.nextafter(np.float32(0.),np.float32(1.)))
        engine.condition(velocity,tuple(changed),preserve_cycle_samples=True)
        assert len(calls)>count,'Distinct native rates reused a different input certificate'
        count=len(calls)
        # A different branch initializer must not reuse a predecessor's result.
        independent=engine.with_controls(engine.automatic_controls)
        assert not independent._certificates
        independent.condition(velocity,omega,canonical=True,preserve_cycle_samples=True)
        assert len(calls)>count
        # A new cold request starts without prior numerical results.
        engine.reset_search()
        assert not engine._certificates and engine._warm is None
        # Exhausting a search budget must preserve the failed trajectory.
        # The retry is a new observation of those native end states, not a
        # cached failure or another restart from the minimum blade angle.
        engine.reset_search()
        short=engine.condition(velocity,omega,cycle_seconds=.1,preserve_cycle_samples=True)
        assert not short['converged'] and engine._pending
        first_end=copy.deepcopy(short['state']);count=len(calls)
        resumed=engine.condition(velocity,omega,cycle_seconds=180.,preserve_cycle_samples=True)
        assert calls[count]['_input_state']==first_end,'Retry discarded the existing native trajectory'
        assert resumed['converged'] and resumed['continued_simulated_seconds']>resumed['simulated_seconds']
        assert not engine._pending
        engine.reset_search()
        report=dict(status='PASS',retained_phases=len(final['cycle_samples']),
            checks=['identical native means and phase samples','no repeated final settle',
                    'unused display coordinates preserve native-input identity','independent branch isolation',
                    'sub-ulp native inputs reuse exact phases; distinct float32 rates do not',
                    'empty numerical state after reset','failed native trajectory resumed and certified'])
        path=Path(__file__).resolve().parents[1]/'analysis/em-upstream-september23/certificate-reuse.json'
        path.write_text(json.dumps(report,indent=2)+'\n')
        print(report)
    finally:prop_em.settled_cycle=original


if __name__=='__main__':main()
