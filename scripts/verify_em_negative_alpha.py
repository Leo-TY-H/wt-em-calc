"""Full-flap negative-alpha roots and phase-independent propulsion means."""
import json
from pathlib import Path

import numpy as np

from em_solver import TrimSolver,settings
from prop_cycle import aligned_window


def main():
    rows=[]
    for torque in (False,True):
        solver=TrimSolver('spitfire_ix',settings(dict(aircraft=['spitfire_ix'],
            flaps_percent=100.,instructor=False,torque_gyro=torque)))
        for speed in (500.,600.,700.,750.):
            for load in (1.,1.1):
                point=solver.solve(speed,load,exhaustive=False)
                assert point['valid'],(speed,load,torque,point['reasons'])
                assert point['alpha_deg']<-6.
                assert point['flaps_percent']==100.
                assert point['force_error_g']<=2e-4 and point['angular_error_rad_s2']<=5e-5
                assert point['stall_margin_deg']>0 and point['authority_margin']>=-2e-7
                assert point['propulsion']['phase_check']['checked']
                rows.append(dict(speed_kmh=speed,load_g=load,torque_gyro=torque,
                    alpha_deg=point['alpha_deg'],force_error_g=point['force_error_g'],
                    angular_error_rad_s2=point['angular_error_rad_s2']))
    # A non-integer governor period makes fixed-duration means phase biased.
    # All accepted samples must still be consecutive, unmodified input frames.
    t=np.arange(1440);wave=np.sin(2*np.pi*t/11.8)
    outputs=np.zeros((len(t),11));outputs[:,0]=6000+500*wave
    outputs[:,3]=2000+200*wave;outputs[:,9]=20+.2*wave
    certificate=aligned_window(outputs,480,12)
    assert certificate is not None
    n=certificate['frames'];mean=outputs[-2*n:].mean(axis=0)
    assert abs(mean[0]-6000)<1. and abs(mean[9]-20)<.001
    # A quiet force channel cannot conceal a drifting moment or wash channel.
    for channel,drift in ((0,2.),(3,1.),(9,.01)):
        transient=outputs.copy();transient[:,channel]+=t*drift
        assert aligned_window(transient,480,12) is None,channel
    assert aligned_window(outputs[:100],480,12) is None
    report=dict(status='PASS',points=rows,aliased_cycle_mean=mean.tolist(),
        drifting_force_moment_wash_rejected=True,short_history_rejected=True)
    target=Path('analysis/em-spitfire-flaps-september22/negative-alpha-validation.json')
    target.parent.mkdir(parents=True,exist_ok=True)
    target.write_text(json.dumps(report,indent=2)+'\n')
    print('PASS',len(rows),'negative-alpha equilibria; cycle aliasing, drift and history checks')


if __name__=='__main__':main()
