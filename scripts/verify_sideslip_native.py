"""Check diagnostic nonzero-sideslip solutions against original aero kernels."""
import json
from pathlib import Path
from probe_sideslip_equilibria import SlipSolver
from verify_aircraft_native import AircraftNative

def main():
    fixtures=json.loads(Path('analysis/equilibrium-gap-recovery/fixtures.json').read_text())['fixtures']
    found=json.loads(Path('analysis/gap-resolution/sideslip-extended.json').read_text())['rows'];native=AircraftNative();rows=[];failures=[]
    for row in found:
        p=row['solution']
        if not p:continue
        f=fixtures[row['index']];s=SlipSolver(f['aircraft'],f['settings']);s.beta=p['beta_input']
        v=s.operating_point(p['speed_kmh']/3.6,p['load_g'],p['solution']);a=v['result']
        got=native.call(s.model,v['velocity'],v['geometry']['omega'].tolist(),s.mass,v['allocation']['commands'],s.config['altitude_m'],s.dt,v['history_input'],
            flaps=v['flaps'],throttle=s.config['throttle'],ground_height=-1e6,quaternion=v['geometry']['quaternion'])
        for key,field in [('forces','component_forces'),('points','component_points')]:
            if got[key]!={k:a[field][k] for k in got[key]}:failures.append(dict(index=row['index'],field=key))
        if got['moment']!=a['raw_aero_moment']:failures.append(dict(index=row['index'],field='moment'))
        assert v['force_error_g']<=2e-4 and max(abs(v['rate_residual']))<=5e-5 and abs(v['kinematic']['velocity'][1])<=.005
        rows.append(dict(index=row['index'],aircraft=s.name,sideslip_deg=a['air']['beta'],force_error_g=v['force_error_g'],angular_error_rad_s2=float(max(abs(v['rate_residual'])))))
        print(s.name,a['air']['beta'],'failures',len(failures),flush=True)
    Path('analysis/gap-resolution/sideslip-native-validation.json').write_text(json.dumps(dict(rows=rows,failures=failures,binary_sha256=native.sha,scope='Different flight condition: stationary nonzero sideslip. Exact native component forces/points/moments, unchanged equilibrium tolerances; diagnostic only, not a zero-sideslip gap replacement.'),indent=2)+'\n')
    if failures:raise SystemExit(1)

if __name__=='__main__':main()
