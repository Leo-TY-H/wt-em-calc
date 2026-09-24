"""Original force-kernel checks around newly observed 12-degree gaps."""
import json
from pathlib import Path
from em_solver import TrimSolver,settings
from verify_aircraft_native import AircraftNative

def main():
    machine=AircraftNative();rows=[];failures=[]
    for name,speed in [('harrier_gr1',700.),('harrier_gr1',1100.),('mig_21_2000_iaf',1100.)]:
        col=json.loads(Path(f'analysis/gap-resolution/columns/{name}-0-{speed:g}.json').read_text())
        s=TrimSolver(name,settings(dict(aircraft=[name])))
        for gap in col['numerical_gap_brackets']:
            p=min((p for p in col['points'] if not p['valid']),key=lambda p:abs(p['load_g']-gap['load_g']))
            for alpha in [11.9999,12.0001]:
                x=list(p['solution']);x[0]=alpha;v=s.operating_point(speed/3.6,p['load_g'],x);a=v['result']
                native=machine.call(s.model,v['velocity'],v['geometry']['omega'].tolist(),s.mass,v['allocation']['commands'],
                    s.config['altitude_m'],s.dt,v['history_input'],flaps=v['flaps'],throttle=s.config['throttle'],ground_height=-1e6,quaternion=v['geometry']['quaternion'])
                for key,source in [('forces','component_forces'),('points','component_points')]:
                    if native[key]!={k:a[source][k] for k in native[key]}:failures.append(dict(aircraft=name,speed=speed,alpha=alpha,field=key,actual=native[key],expected={k:a[source][k] for k in native[key]}))
                if native['moment']!=a['raw_aero_moment']:failures.append(dict(aircraft=name,speed=speed,alpha=alpha,field='moment'))
                rows.append(dict(aircraft=name,speed=speed,alpha=alpha,forces=native['forces'],moment=native['moment'],residual=v['residual'].tolist()))
            print(name,speed,'failures',len(failures),flush=True)
    Path('analysis/gap-resolution/new-gap-native.json').write_text(json.dumps(dict(binary_sha256=machine.sha,rows=rows,failures=failures),indent=2)+'\n')
    if failures:raise SystemExit(1)

if __name__=='__main__':main()
