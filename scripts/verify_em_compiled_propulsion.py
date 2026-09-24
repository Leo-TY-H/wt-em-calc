"""Compare compiled dispatch with saved, exact native-equation trajectories."""
import copy,json,time
from pathlib import Path
from em_backend import activate,ROOT
activate()
from prop_steady import settled_cycle

def main():
 source=ROOT/'analysis/em-upstream-september23/p-63a-10-788.6219-RB/propulsion-samples.json'
 cases=json.loads(source.read_text());rows=[]
 for i,sample in enumerate(cases):
  args,kwargs=copy.deepcopy(sample['inputs']);start=time.monotonic();actual=settled_cycle(*args,**kwargs);elapsed=time.monotonic()-start
  expected=sample['output']
  keys=('state','force','moment','angular_momentum','wash','converged','feasible','period_frames','stationarity','simulated_seconds')
  for key in keys:
   assert json.loads(json.dumps(actual.get(key)))==expected.get(key),(i,key)
  if expected.get('cycle_samples') is not None:assert actual['cycle_samples']==expected['cycle_samples'],(i,'native phases')
  rows.append(dict(case=i,frames=round(actual['simulated_seconds']*48),elapsed_s=elapsed))
 report=dict(status='PASS',scope=__doc__,cases=rows,total_s=sum(r['elapsed_s'] for r in rows))
 (ROOT/'analysis/em-upstream-september23/compiled-dispatch-validation.json').write_text(json.dumps(report,indent=2)+'\n')
 print('PASS',len(rows),'trajectories;',sum(r['frames'] for r in rows),'frames;',report['total_s'],'s')
if __name__=='__main__':main()
