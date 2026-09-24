"""All-jet Instructor trim gates through complete original snapshot consumer."""
import json,random
from pathlib import Path
from component_assembly import f32
from jet_catalog import catalog
from instructor_source import load
from primary_controls import selected_properties,authority_ranges
from verify_snapshot_delivery import DeliveryMachine
from instructor_settings import deliver_keyboard_commands


def main():
 n=DeliveryMachine();rng=random.Random(393899);failures=[];counts=dict(aircraft=0,updates=0,auto_retained_pitch=0,auto_retained_yaw=0)
 for name,item in catalog().items():
  if not item['supported']:continue
  fm=load(name);p=selected_properties(fm);ground=[fm['AvailableControls'].get('has'+k+'TrimGroundControl',False) for k in ['Aileron','Elevator','Rudder']]
  for i in range(4):
   v=lambda a,b:f32(rng.uniform(a,b))
   flags=dict(autotrim=bool(i&1),autotrim_allowed=bool(i&2),ground_trim=ground)
   snapshot=dict(commands=[v(-1,1) for _ in range(3)],throttle=v(0,1.1),afterburner=bool(i&1))
   state=dict(delivered=[v(-1,1) for _ in range(3)],trim_requested=[v(-1,1) for _ in range(3)],trim_actual=[v(-1,1) for _ in range(3)],throttle=v(0,1.1),afterburner=False,running=7,vtol=0.,reverse=0.)
   ranges=authority_ranges(p,v(30,700));dt=f32([1/120,1/60,1/48,1/30][i])
   a=n.delivery(p,snapshot,state,ranges,dt,**flags);e=deliver_keyboard_commands(p,snapshot,state,ranges,dt,**flags);counts['updates']+=1
   if i==3:
    counts['auto_retained_pitch']+=not p['trim_available'][1] and not ground[1]
    counts['auto_retained_yaw']+=not p['trim_available'][2] and not ground[2]
   if a!=e:failures.append(dict(case=[name,i],actual=a,expected=e))
  counts['aircraft']+=1
 report=dict(binary_sha256=n.sha,counts=counts,failures=failures[:12],failure_count=len(failures),scope='Complete original101a4e5f0 primary/jet delivery, prepared one engine/no transmission; all 403 FM trim/ground-trim options, both autotrim and permission bits. No scheduling or engine-count initializer claimed.')
 Path('analysis/instructor-full/delivery-validation.json').write_text(json.dumps(report,indent=2)+'\n');print(counts,'failures',len(failures))
 if failures:raise SystemExit(1)
if __name__=='__main__':main()
