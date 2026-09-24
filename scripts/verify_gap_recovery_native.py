"""Native evidence for current gaps, with original plot/equations preserved."""
import json,time
from pathlib import Path
from scipy.optimize import brentq
from em_solver import TrimSolver
from verify_aircraft_body_native import AircraftBodyNative


def main():
    root=Path('analysis/equilibrium-gap-recovery');fixtures=json.loads((root/'fixtures.json').read_text())['fixtures'];machine=AircraftBodyNative()
    rows=[];failures=[];start=time.monotonic()
    for index,f in enumerate(fixtures):
        s=TrimSolver(f['aircraft'],f['settings']);p=f['bad'];speed=p['speed_kmh']/3.6;load=p['load_g'];base=p['solution']
        def at(x):return s.operating_point(speed,load,x)
        if f['aircraft'] in ('f_16a_block_10','f_16a_block_15_adf'):
            def residual(alpha):return at([alpha,*base[1:]])['geometry']['omega'][0]
            threshold=brentq(residual,base[0]-.01,base[0]+.01,xtol=1e-14)
            states=[('roll-rate crossing '+str(delta),[threshold+delta,*base[1:]]) for delta in [-1e-7,1e-7]]
            cause='wing blend at zero required body roll rate'
        elif f['aircraft']=='saab_jas39c':
            d=at(base);tail=min(['left_hstab','right_hstab'],key=lambda k:abs(d['result']['tail']['effective_angles'][k]-50.))
            def residual(pitch):
                x=list(base);x[3]=pitch;return at(x)['result']['tail']['effective_angles'][tail]-50.
            threshold=brentq(residual,max(-1.,base[3]-.01),min(1.,base[3]+.01),xtol=1e-13)
            states=[]
            for delta in [-1e-5,1e-5]:
                x=list(base);x[3]=threshold+delta;states.append((tail+' 50-degree crossing '+str(delta),x))
            cause='canard polar at 50-degree effective angle'
        else:
            states=[('saved interior failure',base)];cause='no admissible root found; pitch-balance search obstruction'
            trace_path=root/f'pitch-trace-{index}.json'
            if trace_path.exists():
                trace=json.loads(trace_path.read_text())
                admissible=[row for row in trace['rows'] if row['admissible_other']]
                if admissible:
                    peak=max(admissible,key=lambda row:row['residual'][4])
                    states.append(('least-negative pitch residual on sampled force-balanced branch',peak['solution']))
        for label,x in states:
            d=at(x);r=d['result'];g=d['geometry'];allocation=d['allocation']
            native=machine.call(s.model,d['velocity'],g['omega'].tolist(),s.mass,allocation['commands'],s.config['altitude_m'],s.dt,
                d['history_input'],throttle=s.config['throttle'],ground_height=-1e6,quaternion=g['quaternion'])
            full=machine.extend(r['engine_force'],r['engine_moment'],[0.]*3,[0.]*3,1.,s.fm.get('ExtThrustBaseMult',1.),s.dt)
            component=max(abs(a-b) for key in native['forces'] for a,b in zip(native['forces'][key],r['component_forces'][key]))
            force=max(abs(a-b) for a,b in zip(full['force'],r['force']));moment=max(abs(a-b) for a,b in zip(full['moment'],r['stored_moment']))
            row=dict(fixture=index,aircraft=s.name,speed_kmh=p['speed_kmh'],load_g=load,label=label,cause=cause,
                solution=x,body_roll_rate=g['omega'][0],wing_blend=r['wing']['blend'],tail_angles=r['tail']['effective_angles'],
                force_residual_g=d['residual'][:2].tolist(),angular_residual_rad_s2=d['rate_residual'].tolist(),
                component_error_n=component,force_error_n=force,moment_error_nm=moment)
            rows.append(row)
            if force/s.weight>2e-5 or max(abs(a-b)/i for a,b,i in zip(full['moment'],r['stored_moment'],s.mass['inertia']))>5e-5:failures.append(row)
    report=dict(binary_sha256=machine.sha,rows=rows,failures=failures,elapsed_s=time.monotonic()-start,
        limitations='Native aerodynamic/body instructions with established prepared-property and libm adapters. Confirms local force jumps and matches failed F-16XL states plus available pitch-trace maxima; does not prove global nonexistence of every alternative equilibrium. No equations, tolerances, plot settings, interpolation or stored result changed.')
    (root/'native-validation.json').write_text(json.dumps(report,indent=2));print('Native states',len(rows),'failures',len(failures),'time',report['elapsed_s'])
    if failures:raise SystemExit(1)


if __name__=='__main__':main()
