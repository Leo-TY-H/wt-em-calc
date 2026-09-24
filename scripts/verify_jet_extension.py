"""Catalog smoke tests and original-code force/moment checks for jet families."""
import json,math,random,time
from pathlib import Path
from em_solver import AIRCRAFT,TrimSolver
from aircraft_model import evaluate
from verify_aircraft_body_native import AircraftBodyNative
from component_assembly import f32


def main():
    start=time.monotonic();machine=AircraftBodyNative();records=[];failures=[]
    for name,metadata in AIRCRAFT.items():
        if not metadata['supported']:continue
        try:
            solver=TrimSolver(name,dict(aircraft=[name],sweep_percent=50.))
            point=solver.solve(850.,3.,detailed=True,exhaustive=False)
            d=point.pop('_detail');r=d['result'];g=d['geometry'];a=d['allocation']
            machine.call(solver.model,d['velocity'],g['omega'].tolist(),solver.mass,a['commands'],
                         0.,solver.dt,d['history_input'],ground_height=-1e6,quaternion=g['quaternion'])
            actual=machine.extend(r['engine_force'],r['engine_moment'],[0.]*3,[0.]*3,1.,solver.fm.get('ExtThrustBaseMult',1.),solver.dt)
            force=max(abs(x-y) for x,y in zip(actual['force'],r['force']))
            moment=max(abs(x-y) for x,y in zip(actual['moment'],r['stored_moment']))
            angular=max(abs(x-y)/inertia for x,y,inertia in zip(actual['moment'],r['stored_moment'],solver.mass['inertia']))
            row=dict(id=name,valid=point['valid'],converged=point['converged'],reasons=point['reasons'],
                     engines=len(solver.engine.units),mass_kg=solver.mass['mass'],force_error_n=force,
                     moment_error_nm=moment,angular_error_rad_s2=angular,ps_mps=point['ps_mps'])
            if force/solver.weight>2e-5 or angular>5e-5:
                failures.append(dict(id=name,stage='native',**{k:v for k,v in row.items() if k.endswith(('_n','_nm','_s2'))}))
            records.append(row)
        except Exception as error:
            failures.append(dict(id=name,stage='prepare/solve',error=repr(error)))
        if len(records)%25==0:print('Checked',len(records),'failures',len(failures),flush=True)
    report=dict(binary_sha256=machine.sha,records=records,failures=failures,elapsed_s=time.monotonic()-start,
        limitations='One 850 km/h, 3g operating point per enabled model, 50% fixed sweep. Native aerodynamic/body comparison shares prepared BLK adapters and supplied engine aggregates. Not a live-flight comparison or proof of all envelope branches.')
    Path('analysis/jet-extension-validation.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(dict(checked=len(records),failures=failures,elapsed_s=report['elapsed_s']),indent=2))
    if failures:raise SystemExit(1)


if __name__=='__main__':main()
