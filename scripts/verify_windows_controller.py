"""Prepared Windows full-pitch updates, actuator trim and native restore checks.

Static updates compare with the readable keyboard controller using independently
called original Windows predictors. Moving updates additionally check every
observed trim slew and retention through the original snapshot restores. They
do not certify owner initialization, an engine trajectory or a chart boundary.
"""
import argparse
import json
import math
from pathlib import Path

from windows_instructor_controller import WindowsInstructorController
from instructor_native import BASE,OBJ,INPUT
from aircraft_catalog import catalog,load
from aircraft_model import prepare,at_sweep
from mass_model import aircraft_properties,evaluate
from jet_catalog import fuel_capacities
from prop_catalog import mass_state
from verify_instructor_keyboard import extract_state,extract_history,extract_result
from instructor_keyboard import keyboard_step
from windows_instructor_source import fixed_source


def differences(actual,expected,path=''):
    if isinstance(expected,dict):
        return [row for key,value in expected.items() for row in differences(actual[key],value,path+'.'+str(key))]
    if isinstance(expected,(list,tuple)):
        return [row for i,value in enumerate(expected) for row in differences(actual[i],value,path+f'[{i}]')]
    if actual==expected:return []
    return [dict(field=path,actual=actual,expected=expected,
                 delta=abs(actual-expected) if isinstance(expected,(int,float)) else None)]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--binary',type=Path,required=True)
    p.add_argument('--aircraft',default='f_16xl,saab_jas39c,f_14a_early,j6k1,go229_v3,fw_200c_1')
    p.add_argument('--all-aircraft',action='store_true')
    p.add_argument('--cases',default='0,1,2,3')
    p.add_argument('--moving-only',action='store_true')
    p.add_argument('--out',type=Path,default=Path('analysis/windows-instructor/windows-controller-validation.json'))
    a=p.parse_args();n=WindowsInstructorController(a.binary);rows=[];errors=[]
    names=sorted(k for k,v in catalog().items() if v['supported']) if a.all_aircraft else a.aircraft.split(',')
    cases=list(map(int,a.cases.split(',')))
    if not cases or any(case not in range(4) for case in cases):p.error('Cases must be in 0,1,2,3')
    for name in names:
        fm=load(name);base=prepare(fm)
        mass=mass_state(name,30.) if catalog()[name]['propulsion']!='jet' else evaluate(aircraft_properties(fm),[v*.3 for v in fuel_capacities(fm)])
        for moving in ((True,) if a.moving_only else (False,True)):
            for case in cases:
                sweep=[0.,0.,.4,1.][case] if catalog()[name]['has_sweep'] else 0.
                flaps=[0.,.3,.37,1.][case];model=at_sweep(base,sweep)
                n.setup_controller(model,mass,moving=moving,speed=[100.,250.,350.,180.][case],
                    alpha=[4.,18.,-8.,30.][case],flaps=flaps,sweep=sweep,height=[0.,1500.,9000.,1000.][case])
                n.u.mem_write(INPUT+2,bytes([case!=2]));n.u.mem_write(INPUT+0x29,b'\1')
                n.floats(INPUT+0x10,[0.]);n.floats(INPUT+0x44,[1.])
                # Full pitch disables all MouseAim output axes; these bytes
                # are enable flags (the native producer returns zero here).
                n.u.mem_write(INPUT+0x40,b'\0\0\0')
                n.u.mem_write(0x107d6fbc0,b'\1\1');n.u.mem_write(BASE+0x3658,bytes([1 if case%2 else 3]))
                n.u.mem_write(0x107d6fbf5,b'\1')
                commands=[.15,1.,-.05];n.floats(BASE+0x8514,commands);n.floats(OBJ+0xe0,[commands[1],commands[0],commands[2]])
                n.floats(BASE+0x87f4,[.12,-.14,.03]);n.floats(BASE+0xa290,[-.04,.08,-.01]);n.floats(BASE+0x3a24,[.07,-.06,.02])
                n.sync_windows_globals()
                state=extract_state(n,model,flaps);history=extract_history(n)
                # Separate oracle calls do not supply intermediates of the
                # full update under test. Restore its input record afterward.
                saved=bytes(n.u.mem_read(INPUT,0x1000))
                n.trace['predictors']=[]
                expected=keyboard_step(model,state,history,n.dt,predictor_cache={'fixed_source':fixed_source(model,state)},
                    predictor_backend=lambda kind,packed,model,source,old:n.predict(packed,old))
                expected_predictors=list(n.trace['predictors'])
                n.u.mem_write(INPUT,saved)
                try:
                    trace=n.step_windows();actual=extract_result(n)
                except Exception as error:
                    errors.append(dict(aircraft=name,moving=moving,case=case,error=repr(error)));continue
                checked={k:v for k,v in actual.items() if k!='trim_actual'} if moving else actual
                diff=differences(checked,{k:expected[k] for k in checked})
                row=dict(aircraft=name,moving=moving,case=case,configuration=dict(flaps=flaps,sweep=sweep),
                    differences=diff,actual=actual,expected=({k:expected[k] for k in checked}),
                    trim_update_count=len(trace['trim_updates']),
                    trim_slew_mismatches=sum(not r['equal'] for r in trace['trim_updates']),
                    restore_count=len(trace['restores']),
                    trim_restore_mismatches=sum(r['before']!=r['after'] for r in trace['restores']),
                    internal_actuator_time=sum(r['dt'] for r in trace['trim_updates']),
                    actor_services=sorted(set(trace['actor_calls'])),body_property_calls=sorted(set(trace.get('body_property_calls',[]))),
                    predictor_success=[r['success'] for r in trace['predictors']],
                    predictor_count_match=len(trace['predictors'])==len(expected_predictors),
                    protection=trace.get('protection'),
                    protection_polar_differences=differences(trace['protection']['polar'],fixed_source(model,state)['polar']) if 'protection' in trace else None,
                    angle_limits_expected=expected['diagnostics']['angle_limits'],
                    cos_dihedral=trace.get('cos_dihedral'),allocations=trace.get('allocations',[]),
                    predictor_input_differences=[differences(x['inputs'],y['inputs']) for x,y in zip(trace['predictors'],expected_predictors)],
                    finite=all(math.isfinite(v) for v in actual['commands']+actual['trim_actual']),
                    first_trim_updates=trace['trim_updates'][:3],last_trim_updates=trace['trim_updates'][-3:])
                rows.append(row)
                print(name,moving,case,'differences',len(diff),'trim updates',row['trim_update_count'],'restore mismatches',row['trim_restore_mismatches'],flush=True)
    report=dict(scope=__doc__,windows_sha256=n.windows.sha256,fixture_binary_sha256=n.sha,
        data_version=json.loads(Path('references/data-version.json').read_text())['version'],rows=rows,errors=errors,
        global_boundary_validated=False)
    a.out.parent.mkdir(parents=True,exist_ok=True);a.out.write_text(json.dumps(report,indent=2)+'\n')
    if errors or any(r['differences'] or any(r['predictor_input_differences']) or not r['predictor_count_match']
        or r['protection_polar_differences']
        or (r['protection'] is not None and r['protection']['angle_limits']!=r['angle_limits_expected'])
        or r['trim_slew_mismatches'] or r['trim_restore_mismatches'] or not r['finite'] for r in rows):raise SystemExit(1)


if __name__=='__main__':main()
