"""Compare supplied turn telemetry with the pinned EM model, without fitting it."""
import csv
import json
import math
from pathlib import Path
import numpy as np
from em_solver import TrimSolver, settings
from em_sampling import sample_column, worker_solver
from prop_catalog import mass_state
from body_dynamics import G as NATIVE_G

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'analysis/prop-flight-comparison'
G = 9.80665


def read(path):
    with path.open(encoding='utf-8-sig', newline='') as f:
        rows = list(csv.DictReader(f))
    return {k:np.array([float(r[k]) for r in rows]) for k in rows[0] if k}


def summarize(path):
    data = read(path)
    t = data['Time, s']
    assert np.all(np.diff(t)>0)
    duration = t[-1]-t[0]
    heading = np.degrees(np.unwrap(np.radians(data['Compass'])))
    rate = abs((heading[-1]-heading[0])/duration)
    means = {k:float(np.trapz(v,t)/duration) for k,v in data.items()}
    v = data['TAS, km/h']/3.6
    energy = data['Altitude, m']+v*v/(2*G)
    windows = []
    for start in np.arange(0., duration-2.+.0001, .5):
        stop = start+2.
        windows.append(abs(float(np.interp(stop,t,heading)-np.interp(start,t,heading)))/2.)
    return dict(file=path.name, samples=len(t), duration_s=float(duration), means=means,
        ranges={k:[float(v.min()),float(v.max())] for k,v in data.items()},
        heading_change_deg=float(heading[-1]-heading[0]), turn_dps=rate,
        turn_fit_dps=abs(float(np.polyfit(t,heading,1)[0])),
        two_second_rate_range_dps=[min(windows),max(windows)],
        ps_endpoint_mps=float((energy[-1]-energy[0])/duration),
        ps_fit_mps=float(np.polyfit(t,energy,1)[0]),
        equivalent_level_load_g=math.hypot(1.,math.radians(rate)*means['TAS, km/h']/3.6/G),
        instructor='22_32' not in path.name)


def brief(point):
    if point is None:return None
    keys=['speed_kmh','load_g','turn_dps','ps_mps','alpha_deg','bank_deg','sideslip_deg',
          'valid','converged','reasons','commands','force_n','engine_force_n','instructor',
          'propulsion','envelope_limit','solution']
    return {k:point[k] for k in keys if k in point}


def apply_recorded_mass(solver,row):
    extra=max(0.,row['means']['Total mass, kg']-solver.mass['mass'])
    if extra:
        # Test-only point payload: true ammunition positions remain unknown.
        replacement=mass_state(solver.name,30.,extra)
        solver.mass.clear();solver.mass.update(replacement)
        solver.weight=solver.mass['mass']*float(NATIVE_G)
    return extra


def main():
    OUT.mkdir(exist_ok=True)
    # The later deceleration recording is analyzed separately, not treated
    # as another sustained equilibrium by this historical comparison.
    records = [summarize(p) for p in sorted((ROOT/'FlightTestData').glob('*.csv')) if 'aoavsspeed' not in p.name]
    (OUT/'telemetry.json').write_text(json.dumps(records,indent=2)+'\n')
    results = []
    for row in records:
        name = row['file'].split('-2026_')[0]
        cfg=settings(dict(aircraft=[name],fuel_percent=30.,
            altitude_m=row['means']['Altitude, m'],instructor=row['instructor'],
            speed_samples=9,load_samples=7,sep_tolerance_mps=1.))
        # Seed the column worker with the identical explicitly matched mass.
        solver=worker_solver(name,json.dumps(cfg))
        extra=apply_recorded_mass(solver,row)
        speed=row['means']['TAS, km/h'];load=row['equivalent_level_load_g']
        point=solver.solve(speed,load,exhaustive=False,detailed=True)
        value=point.pop('_detail')
        fit=dict(telemetry=row,aircraft=name,settings=cfg,model_mass=solver.mass,
            mass_matching='30% fuel; measured additional mass at nominal FM CG for this comparison only; exact ammo positions/inertia unverified.',
            added_mass_kg=extra,
            matched_rate_point=brief(point),
            matched_rate_body_y_load_g=value['result']['force'][1]/(solver.mass['mass']*G))
        results.append(fit)
        (OUT/'comparison.json').write_text(json.dumps(results,indent=2)+'\n')
        print(name,row['file'], 'matched',point['valid'], 'alpha',point['alpha_deg'],
              'Ps',point['ps_mps'],'thrust kgf',point['engine_force_n'][0]/G,flush=True)
        column=sample_column((name,json.dumps(cfg),speed,None))
        fit['boundary_status']=column['boundary_status']
        fit['boundary']=brief(column['boundary'])
        fit['sustained']=[brief(p) for p in column['sustained']]
        (OUT/'comparison.json').write_text(json.dumps(results,indent=2)+'\n')
        print('boundary',brief(column['boundary']),flush=True)
        (OUT/(row['file']+'.column.json')).write_text(json.dumps(column)+'\n')


if __name__=='__main__':main()
