"""Original aerodynamic replay of boundaries and new numerical recovery states."""
import argparse,json
from pathlib import Path
from em_solver import TrimSolver
from verify_aircraft_native import AircraftNative
from verify_polar_machine_code import EXPECTED_BINARY_SHA256
from em_solver import ROOT


def main():
    ap=argparse.ArgumentParser();ap.add_argument('data',nargs='+');ap.add_argument('--report',required=True);args=ap.parse_args()
    binary=ROOT/'references/native'/('aces-'+EXPECTED_BINARY_SHA256)
    native=AircraftNative(binary if binary.exists() else None);rows=[];failures=[]
    for path in args.data:
        data=json.loads(Path(path).read_text())
        for a in data['aircraft']:
            s=TrimSolver(a.get('aircraft_id',a['id']),a['settings'])
            if s.is_prop:raise ValueError('Use verify_prop_em_native for phase-resolved propulsion')
            selected={}
            for p in a['points']:
                method=p.get('recovery_method')
                if method and (method not in selected or abs(p['sideslip_deg'])>abs(selected[method]['sideslip_deg'])):selected[method]=p
                crossing=p.get('continuation_native_switch')
                if crossing and 'native roll-leveling switch' not in selected:
                    selected['native roll-leveling switch']=p
                    for side in ('previous','opposite'):
                        point=s.solve(p['speed_kmh'],crossing[side+'_load_g'],crossing[side+'_solution'],exhaustive=False)
                        assert point['valid'],(a['id'],side,point['reasons'])
                        selected['native switch '+side]=point
            limits=[c['boundary'] for c in a['columns'] if c['boundary_status']=='verified limit']
            for index in sorted({0,len(limits)//2,len(limits)-1}):selected['boundary '+str(index)]=limits[index]
            for reason,p in selected.items():
                v=s.point_value(p);aero=v['result']
                got=native.call(s.model,v['velocity'],v['geometry']['omega'].tolist(),s.mass,v['allocation']['commands'],
                    s.config['altitude_m'],s.dt,v['history_input'],flaps=v['flaps'],throttle=s.config['throttle'],
                    ground_height=s.config['altitude_m']-1e6,quaternion=v['geometry']['quaternion'])
                for key,field in [('forces','component_forces'),('points','component_points')]:
                    if got[key]!={k:aero[field][k] for k in got[key]}:failures.append(dict(aircraft=s.name,reason=reason,field=key))
                if got['moment']!=aero['raw_aero_moment']:failures.append(dict(aircraft=s.name,reason=reason,field='moment'))
                assert abs(v['ps']-p['ps_mps'])<1e-9
                assert v['force_error_g']<=2e-4 and max(abs(v['rate_residual']))<=5e-5 and v['history_error']<=2e-4
                rows.append(dict(aircraft=s.name,selection=reason,speed_kmh=p['speed_kmh'],load_g=p['load_g'],sideslip_deg=p['sideslip_deg']))
            print(s.name,len(selected),'original aero replays',flush=True)
    report=dict(status='FAIL' if failures else 'PASS',rows=rows,failures=failures,binary_sha256=native.sha,
        scope='Exact original component forces, application points and raw aero moments; exported Ps and unchanged force/rate/history tolerances. Controller live-flight tracking is outside this test.')
    Path(args.report).write_text(json.dumps(report,indent=2)+'\n');assert not failures,failures
    print('PASS',len(rows),'selected states')

if __name__=='__main__':main()
