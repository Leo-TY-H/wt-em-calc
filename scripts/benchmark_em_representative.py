"""Reduced aircraft-family benchmark; actual UI presets, cold numerical caches."""
import argparse,json,time
from collections import Counter
from pathlib import Path
import numpy as np
from em_solver import settings,compute,BACKEND
from em_plot import enrich
from em_sampling import _AIRCRAFT_CACHE,_COLUMN_CACHE
from em_workers import shutdown

# One representative per distinct numerical/propulsion regime. RB and SB are
# explicit, so changes in UI defaults cannot silently alter the benchmark.
CASES={
 'biplane':dict(aircraft='he51b1',mode='SB',reason='Low-speed biplane, fixed gear and simple propeller installation'),
 'inline':dict(aircraft='p-51d-30_usaaf_korea',mode='SB',reason='Single inline piston with automatic governor and torque'),
 'radial':dict(aircraft='f8f1b',mode='RB',reason='High-power radial piston and Instructor boundary'),
 'twin':dict(aircraft='do_335b_2',mode='RB',reason='Two piston drivetrains with different governor phases; reported aircraft'),
 'turboprop':dict(aircraft='wyvern_s4',mode='SB',reason='Turboprop and contra-rotating propellers'),
 'conventional_jet':dict(aircraft='mig-17',mode='RB',reason='Conventional swept-wing jet with Instructor'),
 'delta':dict(aircraft='mirage_3e',mode='SB',reason='Delta wing with nonlinear high-angle lift/drag'),
 'variable_sweep':dict(aircraft='f_14b',mode='RB',sweep_percent=50.,reason='Twin jet, variable geometry and native sweep availability'),
 'mixed':dict(aircraft='fr_1_fireball',mode='RB',reason='Mixed piston/jet propulsion, supported singleton family'),
 'heavy':dict(aircraft='halifax_mk3',mode='SB',reason='Four-engine heavy bomber; large inertia and four coupled propeller installations'),
 'spitfire_flaps':dict(aircraft='spitfire_ix_usa',mode='RB',flaps_percent=100.,reason='Reported full-flap negative-alpha regression'),
 'ki61':dict(aircraft='ki_61_1a_otsu_china',mode='SB',reason='Reported high-speed governor and missing-column regression'),
 'spitfire_flaps_sb':dict(aircraft='spitfire_ix_usa',mode='SB',flaps_percent=100.,reason='Original full-flap Instructor-off regression'),
}
REPRESENTATIVES=list(CASES)[:10]
PRESETS={'quick':(9,7,1.),'standard':(9,9,.5),'fine':(9,13,.15)}

def clean(value):
 if isinstance(value,dict):return {k:clean(v) for k,v in value.items() if not k.startswith('_')}
 if isinstance(value,(tuple,list)):return [clean(v) for v in value]
 if hasattr(value,'tolist'):return value.tolist()
 return value

def run_case(name,preset,out,limit):
 case=CASES[name];ns,nl,tol=PRESETS[preset]
 config=settings(dict(aircraft=[case['aircraft']],instructor=case['mode']=='RB',torque_gyro=case['mode']=='SB',
  speed_min_kmh=100.,speed_max_kmh=1800.,speed_samples=ns,load_samples=nl,sep_tolerance_mps=tol,
  flaps_percent=case.get('flaps_percent',0.),sweep_percent=case.get('sweep_percent',0.)))
 shutdown();_AIRCRAFT_CACHE.clear();_COLUMN_CACHE.clear();last={};start=time.monotonic();reported=[0.]
 def progress(p):
  last.update(p)
  if time.monotonic()-reported[0]>10.:
   print(name,p.get('phase'),p.get('done'),p.get('total'),p.get('speed_kmh'),round(time.monotonic()-start,1),flush=True);reported[0]=time.monotonic()
 row=dict(case=name,aircraft=case['aircraft'],reason=case['reason'],settings=config,backend=BACKEND)
 try:
  data=compute(config,progress=progress,cancelled=lambda:time.monotonic()-start>=limit)
 except InterruptedError:
  columns={v['speed_kmh']:v for key,v in _COLUMN_CACHE.items() if key[1]==case['aircraft']}
  (out/(name+'.partial.json')).write_text(json.dumps(clean(dict(columns=list(columns.values()))),separators=(',',':')))
  row.update(status='timeout',elapsed_s=time.monotonic()-start,last_progress=last,meets_20s=False,meets_60s=False)
  return row
 except Exception as error:
  row.update(status='error',error=repr(error),elapsed_s=time.monotonic()-start,last_progress=last,meets_20s=False,meets_60s=False)
  return row
 calculation=time.monotonic()-start
 (out/(name+'.json')).write_text(json.dumps(clean(data),separators=(',',':'),allow_nan=False))
 enrich(data);elapsed=time.monotonic()-start;a=data['aircraft'][0];cols=a['columns']
 from verify_em_contours import check as check_contours
 attachment=check_contours(a)
 (out/(name+'.contours.json')).write_text(json.dumps(attachment,indent=2)+'\n')
 z=np.asarray(a['surface']['z'],dtype=float);xs=np.asarray(a['surface']['x'])
 good=[c['speed_kmh'] for c in cols if c.get('boundary_status') in ('verified limit','plot ceiling')]
 interior=(xs>min(good))&(xs<max(good)) if good else np.zeros(len(xs),dtype=bool)
 for lo,hi in a.get('sweep_excluded_speeds_kmh',[]):interior&=~((xs>lo)&(xs<hi))
 excluded=[]
 for c in cols:
  level=next((p for p in c['points'] if p['load_g']==1.),None)
  if c['boundary_status']=='no feasible samples' and level and level['converged'] and level['reasons']==['Instructor pitch limit']:
   excluded.append(dict(speed_kmh=c['speed_kmh'],reason='balanced Instructor pitch exclusion at level flight',
    force_error_g=level['force_error_g'],angular_error_rad_s2=level['angular_error_rad_s2'],
    pitch_margin=level['instructor']['pitch_margin']))
 row.update(status='complete',calculation_s=calculation,elapsed_s=elapsed,meets_20s=elapsed<=20.,meets_60s=elapsed<60.,
  speed_columns=len(cols),points=len(a['points']),valid_points=a['valid_points'],
  boundary_statuses=dict(Counter(c['boundary_status'] for c in cols)),
  numerical_gaps=sum(len(c.get('numerical_gap_brackets',[])) for c in cols),
  unresolved_load_intervals=sum(len(c.get('unresolved_load_intervals',[])) for c in cols),
  unresolved_speed_intervals=len(a['interpolation']['unresolved_speed_intervals']),
  contour_edge_intersections=attachment['checked_intersections'],
  detached_contour_ends=len(attachment['failures']),missing_upper_edge_vertices=attachment['missing_upper_edge_vertices'],
  blank_rendered_columns=int(np.count_nonzero(interior&~np.isfinite(z).any(axis=0))),
  blank_speeds_kmh=xs[interior&~np.isfinite(z).any(axis=0)].tolist(),
  balanced_instructor_exclusions=excluded)
 return row

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--cases',default=','.join(REPRESENTATIVES));p.add_argument('--preset',choices=PRESETS,default='standard');p.add_argument('--out',required=True);p.add_argument('--limit',type=float,default=60.);args=p.parse_args()
 names=args.cases.split(',');assert set(names)<=set(CASES)
 out=Path(args.out);out.mkdir(parents=True,exist_ok=True);rows=[]
 for name in names:
  row=run_case(name,args.preset,out,args.limit);rows.append(row)
  (out/'results.json').write_text(json.dumps(dict(preset=args.preset,cases=rows),indent=2)+'\n')
  print('RESULT',name,row['status'],round(row['elapsed_s'],3),'s',flush=True)
 shutdown()
if __name__=='__main__':main()
