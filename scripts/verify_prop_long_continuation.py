"""Validate continued EM engine certificates against independent 180-second native continuations."""
import copy,json,math,time
from pathlib import Path
import numpy as np
from em_solver import TrimSolver,settings,ROOT
from propulsion_general import step


def main():
 cases=[('p-63a-10',788.6,1.,30.,True),('p-63a-10',862.4,1.,30.,True),
        ('p-63a-10',868.55,4.,30.,True),('p-38g',700.,3.,30.,True),
        ('b_25j_1',450.,3.,50.,True),('saab_j21a_1',550.,4.,30.,False),
        ('la-7',600.,5.,60.,False),('ki_61_1a_otsu_china',718.7,8.,0.,False),
        ('a7m2',111.69057615447734,1.,100.,False),('a7m2',190.,1.,100.,False),('a7m2',450.,5.,100.,False),
        ('mosquito_fb_mk6',400.,3.,0.,True),('wyvern_s4',680.,5.,0.,False),
        ('seafire_fr47',650.,4.,0.,False),('fw-190a-5_cannons',600.,2.,30.,True)]
 rows=[]
 for name,speed,load,flaps,rb in cases:
  started=time.monotonic();solver=TrimSolver(name,settings(dict(aircraft=[name],flaps_percent=flaps,instructor=rb,torque_gyro=not rb)))
  seed=None
  if name=='a7m2' and speed<120.:
   data=json.loads((ROOT/'analysis/em-upstream-september23/phase-root-trial/a7m_sb/data.json').read_text())
   candidates=[p for p in data['aircraft'][0]['points'] if p['valid'] and p['load_g']==1.]
   seed=min(candidates,key=lambda p:abs(p['speed_kmh']-speed))['solution']
  point=solver.solve(speed,load,seed,detailed=True,exhaustive=False)
  if not point['valid']:
   rows.append(dict(aircraft=name,speed_kmh=speed,load_g=load,status='solve failed',reasons=point['reasons']))
   print(rows[-1],flush=True);continue
  original=point.pop('_detail');prop=original['propulsion'];state=copy.deepcopy(prop['state']);phases=[]
  for frame in range(round(180./solver.dt)):
   state=step(solver.engine.properties,state,original['velocity'],solver.config['altitude_m'],
       original['geometry']['omega'],solver.mass['cog'],solver.dt,state['seed'],
       solver.mass['nitro_mass'],torque_gyro=solver.config['torque_gyro'])
   if frame>=round(100./solver.dt):
    phases.append(state['aggregate_force']+state['aggregate_moment']+state['engine_angular_momentum']+state['engine_wash'])
  mean=np.asarray(phases).mean(0).tolist()
  continued=dict(prop,state=state,force=mean[:3],moment=mean[3:6],angular_momentum=mean[6:9],wash=mean[9:],cycle_samples=phases,stationarity=None)
  view=copy.copy(solver);view.engine=copy.copy(solver.engine);view.engine.condition=lambda *args,**kw:continued
  actual=view.operating_point(speed/3.6,load,point['solution'])
  row=dict(aircraft=name,speed_kmh=speed,load_g=load,elapsed_s=time.monotonic()-started,
           phase_frames=len(prop['cycle_samples']),averaging=(prop.get('stationarity') or {}).get('window_alignment','exact'),
           sep_change_mps=abs(actual['ps']-point['ps_mps']),force_error_g=actual['force_error_g'],
           angular_error_rad_s2=float(max(abs(actual['rate_residual']))),
           certificate=prop.get('stationarity'),initializer=prop.get('numerical_initializer'))
  row['status']='PASS' if row['sep_change_mps']<.05 and row['force_error_g']<2e-4 and row['angular_error_rad_s2']<5e-5 else 'FAIL'
  rows.append(row);print(name,speed,row['status'],row['averaging'],round(row['sep_change_mps'],6),'m/s',flush=True)
 report=dict(status='PASS' if all(r['status']=='PASS' for r in rows) else 'FAIL',scope=__doc__,cases=rows)
 (ROOT/'analysis/em-upstream-september23/long-continuation-validation.json').write_text(json.dumps(report,indent=2)+'\n')
 assert report['status']=='PASS',[(r['aircraft'],r['speed_kmh'],r['status']) for r in rows if r['status']!='PASS']

if __name__=='__main__':main()
