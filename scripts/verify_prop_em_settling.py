"""Continue certified EM engine states through 120 seconds of native frames."""
import copy
import json
import math
from pathlib import Path

import numpy as np

from em_solver import TrimSolver, settings
from propulsion_general import step


def main():
    rows=[]
    cases=[('ki_61_1a_otsu_china',631.2476,7.),
           ('ki_61_1a_otsu_china',718.7471,8.),
           ('ki_61_1a_otsu_china',806.2467,2.),
           ('ki_61_1a_otsu_china',830.,2.),
           ('ki_61_1a_otsu_china',849.9,2.),
           ('p-47d-28',884.9964,1.),
           ('mosquito_fb_mk6',400.,3.),
           ('wyvern_s4',680.,5.)]
    for name,speed,load in cases:
        solver=TrimSolver(name,settings(dict(aircraft=[name])))
        point=solver.solve(speed,load,refine=True,detailed=True,exhaustive=False)
        assert point['valid'],(name,point['reasons'])
        original=point.pop('_detail');prop=original['propulsion']
        state=copy.deepcopy(prop['state']);phases=[];rpm=[]
        for frame in range(round(120./solver.dt)):
            state=step(solver.engine.properties,state,original['velocity'],solver.config['altitude_m'],
                original['geometry']['omega'],solver.mass['cog'],solver.dt,state['seed'],
                solver.mass['nitro_mass'],torque_gyro=solver.config['torque_gyro'])
            if frame>=round(100./solver.dt):
                phases.append(state['aggregate_force']+state['aggregate_moment']+
                              state['engine_angular_momentum']+state['engine_wash'])
                rpm.append([e['omega']*60./(2.*math.pi) for e in state['engines']])
        mean=np.asarray(phases).mean(axis=0).tolist()
        continued=dict(prop,state=state,force=mean[:3],moment=mean[3:6],
                       angular_momentum=mean[6:9],wash=mean[9:],cycle_samples=phases)
        view=copy.copy(solver);view.engine=copy.copy(solver.engine)
        view.engine.condition=lambda *args,**kwargs:continued
        actual=view.operating_point(speed/3.6,load,point['solution'])
        row=dict(aircraft=name,speed_kmh=speed,load_g=load,simulated_seconds=120.,
            sample_seconds=20.,sep_change_mps=abs(actual['ps']-point['ps_mps']),
            force_error_g=actual['force_error_g'],angular_error_rad_s2=float(max(abs(actual['rate_residual']))),
            maximum_rpm=np.max(rpm,axis=0).tolist(),certificate=prop.get('stationarity'))
        assert row['sep_change_mps']<.05,row
        assert row['force_error_g']<2e-4 and row['angular_error_rad_s2']<5e-5,row
        rows.append(row)
        print(name,round(row['sep_change_mps'],6),'m/s native continuation error',flush=True)
    report=dict(status='PASS',cases=rows,engine_health_policy='Intact; no damage modeled')
    path=Path(__file__).resolve().parents[1]/'analysis/em-global-september22/native-continuation.json'
    path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':main()
