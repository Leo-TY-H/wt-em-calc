"""Cold, condition-stratified EM benchmarks; each case gets a new interpreter.

This suite includes reported regressions plus aircraft selected before timing.
Saved plots and numerical caches are never inputs. Compiled equation modules
and immutable game data are retained, as they are for a user's running server.
"""
import argparse,hashlib,json,os,subprocess,sys,time
from pathlib import Path

CASES={
 'p51k_rb':dict(aircraft='p-51k',reason='Reported default RB timeout; inline piston, clean'),
 'fw190_rb20':dict(aircraft='fw-190a-5_cannons',flaps_percent=20.,reason='Saved 277-second request; radial, partial flaps, lower control limit'),
 'fw190_rb30':dict(aircraft='fw-190a-5_cannons',flaps_percent=30.,reason='User-reported partial flap setting; higher negative-alpha/control demand'),
 'bf109_altitude':dict(aircraft='bf-109f-4',flaps_percent=30.,altitude_m=4000.,reason='Unseen inline piston; partial flaps and compressor/air-density change'),
 'la7_sb':dict(aircraft='la-7',mode='SB',flaps_percent=60.,reason='Unseen radial; torque and large requested flap position'),
 'p38_rb':dict(aircraft='p-38g',flaps_percent=30.,reason='Unseen twin piston; opposing propellers and partial flaps'),
 'seafire_sb':dict(aircraft='seafire_fr47',mode='SB',reason='Unseen contra-rotating piston installation'),
 'wellington_rb':dict(aircraft='wellington_mk1c',flaps_percent=70.,reason='Unseen heavy twin piston; large inertia and large requested flap position'),
 'me262_rb':dict(aircraft='me-262a-1a',reason='Unseen early twin jet, Instructor and conventional wing'),
 'hunter_sb':dict(aircraft='hunter_f1',mode='SB',flaps_percent=40.,reason='Unseen swept-wing jet; partial flaps without Instructor'),
 'mig21_altitude':dict(aircraft='mig-21_f13',altitude_m=8000.,reason='Unseen delta jet; altitude, afterburner and Instructor'),
 'f14_sweep':dict(aircraft='f_14b',sweep_percent=50.,reason='Retained variable-sweep family; native sweep exclusions'),
 'wyvern_sb':dict(aircraft='wyvern_s4',mode='SB',reason='Retained turboprop family and governor regression'),
 'fireball_rb':dict(aircraft='fr_1_fireball',reason='Retained mixed piston/jet family'),
 'he51_sb':dict(aircraft='he51b1',mode='SB',reason='Retained biplane, fixed gear and low-speed edges'),
 'f8f_rb':dict(aircraft='f8f1b',reason='Retained native Instructor rounding regression'),
 'spitfire_sb100':dict(aircraft='spitfire_ix_usa',mode='SB',flaps_percent=100.,reason='Reported full-flap negative-alpha regression'),
 'ki61_sb':dict(aircraft='ki_61_1a_otsu_china',mode='SB',reason='Reported high-speed automatic-governor regression'),
}

def worker(name,preset,out,limit):
 from em_solver import compute,settings
 from em_plot import enrich
 from em_sampling import _AIRCRAFT_CACHE,_COLUMN_CACHE,worker_solver
 from em_workers import shutdown,START_METHOD
 from benchmark_em_representative import PRESETS,clean
 from verify_em_contours import check
 import numpy as np
 case=CASES[name];ns,nl,tol=PRESETS[preset]
 overrides={k:v for k,v in case.items() if k not in ('aircraft','mode','reason')}
 rb=case.get('mode','RB')=='RB'
 config=settings(dict(overrides,aircraft=[case['aircraft']],instructor=rb,torque_gyro=not rb,
                      speed_samples=ns,load_samples=nl,sep_tolerance_mps=tol))
 shutdown();_AIRCRAFT_CACHE.clear();_COLUMN_CACHE.clear();worker_solver.cache_clear()
 assert not _AIRCRAFT_CACHE and not _COLUMN_CACHE
 sources={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(Path(__file__).absolute().parent.glob('*.py'))}
 t=time.monotonic();last=[0.];row=dict(case=name,reason=case['reason'],settings=config,source_sha256=sources,
     cache_policy='Fresh interpreter and worker pool; empty numerical caches; no saved-result reads',
     process_start_method=START_METHOD,
     native_map_initializer=os.environ.get('WT_EM_PROP_FIXEDPOINT','1')!='0',
     numerical_environment={k:v for k,v in os.environ.items() if k.startswith('WT_EM_')})
 def progress(p):
  if time.monotonic()-last[0]>10.:
   print(name,round(time.monotonic()-t,1),p.get('phase'),p.get('speed_kmh'),p.get('done'),p.get('total'),flush=True);last[0]=time.monotonic()
 try:
  data=compute(config,progress,cancelled=lambda:time.monotonic()-t>=limit)
  (out/'data.json').write_text(json.dumps(clean(data),separators=(',',':'),allow_nan=False))
  enrich(data);elapsed=time.monotonic()-t;a=data['aircraft'][0];cols=a['columns'];z=np.asarray(a['surface']['z'],float)
  xs=np.asarray(a['surface']['x']);good=[c['speed_kmh'] for c in cols if c.get('boundary')]
  interior=(xs>min(good))&(xs<max(good)) if good else np.zeros(len(xs),bool)
  for lo,hi in a.get('sweep_excluded_speeds_kmh',[]):interior&=~((xs>lo)&(xs<hi))
  contours=check(a)
  row.update(status='complete',elapsed_s=elapsed,meets_20s=elapsed<=20.,meets_60s=elapsed<60.,
   cached_aircraft=data.get('cached_aircraft'),cached_columns=data.get('cached_columns'),
   speed_columns=len(cols),points=len(a['points']),
   max_column_s=max(c.get('elapsed_s',0.) for c in cols),
   numerical_gaps=sum(len(c.get('numerical_gap_brackets',[])) for c in cols),
   unresolved_boundaries=sum(c['boundary_status']=='unresolved numerical boundary' for c in cols),
   unresolved_speed_intervals=a.get('interpolation',{}).get('unresolved_speed_intervals'),
   blank_rendered_columns=int(np.count_nonzero(interior&~np.isfinite(z).any(axis=0))),
   contour_checks=contours)
 except InterruptedError:
  (out/'partial.json').write_text(json.dumps(clean(dict(columns=list(_COLUMN_CACHE.values()))),separators=(',',':')))
  row.update(status='timeout',elapsed_s=time.monotonic()-t,meets_20s=False,meets_60s=False)
 finally:shutdown()
 (out/'result.json').write_text(json.dumps(row,indent=2)+'\n');print('RESULT',name,row['status'],row['elapsed_s'],flush=True)


def main():
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--cases',default=','.join(CASES));ap.add_argument('--preset',choices=['quick','standard','fine'],default='standard');ap.add_argument('--out',required=True);ap.add_argument('--limit',type=float,default=60.);ap.add_argument('--worker',action='store_true');args=ap.parse_args()
 names=args.cases.split(',');assert set(names)<=set(CASES);out=Path(args.out).absolute();out.mkdir(parents=True,exist_ok=True)
 if args.worker:
  assert len(names)==1;worker(names[0],args.preset,out,args.limit);return
 rows=[]
 for name in names:
  directory=out/name;directory.mkdir(exist_ok=True)
  if (directory/'data.json').exists():(directory/'data.json').unlink()
  if (directory/'result.json').exists():(directory/'result.json').unlink()
  child=subprocess.run([sys.executable,__file__,'--worker','--cases',name,'--preset',args.preset,'--out',str(directory),'--limit',str(args.limit)])
  if child.returncode:raise RuntimeError(f'{name}: benchmark subprocess failed ({child.returncode})')
  rows.append(json.loads((directory/'result.json').read_text()))
  (out/'results.json').write_text(json.dumps(dict(preset=args.preset,cases=rows),indent=2)+'\n')
if __name__=='__main__':main()
