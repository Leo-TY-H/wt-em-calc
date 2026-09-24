"""Stationary propulsion acceptance: reject dynamics, replay long horizons and trim."""
import copy,json
from pathlib import Path
import numpy as np
from em_solver import TrimSolver,settings
from prop_steady import saturated_key
from propulsion_general import step

OUT=Path('analysis/numerical-gap-budget')

def main():
    properties=dict(transmissions=[{}],propellers=[dict(properties=dict(pitch_min=.5,pitch_max=1.))])
    before=(12345,250.,250.,.5,.4,10.,0.,0.)
    outward=(12345,250.,250.,.5,.3,10.,0.,0.)
    assert saturated_key(properties,outward,before)==saturated_key(properties,before,before)
    assert saturated_key(properties,before,outward) is None
    for index in [0,1,2,3,5,6,7]:
        changed=list(outward);changed[index]+=.0001
        assert saturated_key(properties,tuple(changed),before)!=saturated_key(properties,outward,before)
    upper_before=(12345,250.,250.,1.,1.2,10.,0.,0.)
    upper_after=(12345,250.,250.,1.,1.3,10.,0.,0.)
    assert saturated_key(properties,upper_after,upper_before)==saturated_key(properties,upper_before,upper_before)
    assert saturated_key(properties,upper_before,upper_after) is None
    baseline=json.loads(Path('outputs/em/0fb0549d3f9ad62b2e69/data.json').read_text())['aircraft'][0]
    gap=min(baseline['columns'],key=lambda c:abs(c['speed_kmh']-245.624))
    failed=[p for p in gap['points'] if not p['valid']]
    cases=[('a6m2_zero',False,p['speed_kmh'],p['load_g'],p) for p in failed[::4]]
    cases += [(name,mode,speed,load,None) for name,mode,speed,load in [
        ('a6m2_zero',True,250.,2.),('a6m2_zero',False,341.24866373291013,3.),
        ('a6m2_zero',False,404.99821831054686,3.),('yak-3',False,350.,2.),
        ('bf-109f-4',False,350.,2.),('i-16_chung_28',False,350.,2.)]]
    report=[];native=[]
    for name,mode,speed,load,old in cases:
        cfg=settings(dict(aircraft=[name],instructor=mode));s=TrimSolver(name,cfg)
        p=s.solve(speed,load,old['solution'] if old else None)
        assert p['valid'],(name,speed,load,p['reasons'])
        v=s.point_value(p);prop=v['propulsion'];state=copy.deepcopy(prop['state'])
        kw=dict(velocity=v['velocity'],height=cfg['altitude_m'],body_omega=v['geometry']['omega'],cg=s.mass['cog'],dt=s.dt,nitro=s.mass['nitro_mass'])
        phase_rows=[]
        for i in range(round(60/s.dt)):
            state=step(s.engine.properties,state,**kw,seed=state['seed'])
            if i>=round(60/s.dt)-max(24,len(prop['cycle_samples'])):
                phase_rows.append(state['aggregate_force']+state['aggregate_moment']+state['engine_angular_momentum']+state['engine_wash'])
        mean=np.mean(phase_rows,axis=0).tolist()
        later=dict(prop,state=state,force=mean[:3],moment=mean[3:6],angular_momentum=mean[6:9],wash=mean[9:],cycle_samples=phase_rows)
        s.engine.condition=lambda *args,**kwargs:later
        long=s.operating_point(speed/3.6,load,p['solution'])
        error=abs(long['ps']-p['ps_mps'])
        row=dict(aircraft=name,instructor=mode,speed_kmh=speed,load_g=load,stationarity=prop.get('stationarity'),
                 late_ps_error_mps=error,late_force_error_g=long['force_error_g'],
                 late_angular_error_rad_s2=float(max(abs(long['rate_residual']))),
                 previous_ps_difference_mps=abs(p['ps_mps']-old['ps_mps']) if old else None)
        assert error<.01,row
        # The accepted point meets the usual thresholds. The longer, differently
        # phased window may differ slightly when the exact period is >24 frames.
        assert long['force_error_g']<3e-4 and row['late_angular_error_rad_s2']<1e-4,row
        report.append(row)
        if old and len(native)<2 or name in ('yak-3','bf-109f-4'):
            native.append(dict(p,aircraft=name,settings={k:v for k,v in cfg.items() if k!='aircraft'}))
        print(name,mode,speed,load,'60 s later Ps error',error,flush=True)
    OUT.joinpath('stationarity-validation.json').write_text(json.dumps(dict(status='PASS',rejection_cases=7,points=report),indent=2)+'\n')
    OUT.joinpath('native-points.json').write_text(json.dumps(native,indent=2)+'\n')

if __name__=='__main__':main()
