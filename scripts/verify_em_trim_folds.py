"""Check connected low-speed trim limits against independent trim and native code."""
import argparse,json
from pathlib import Path
import numpy as np
from em_solver import TrimSolver
from em_sampling import speed_interpolate,column_at_load
from verify_em_trim_contours import independently_continue
from verify_aircraft_body_native import AircraftBodyNative


def main():
    ap=argparse.ArgumentParser();ap.add_argument('data');ap.add_argument('--report',required=True);args=ap.parse_args()
    data=json.loads(Path(args.data).read_text());a=data['aircraft'][0];columns=a['columns'];solver=TrimSolver('f_16xl',a['settings'])
    low=[c for c in columns if c['speed_kmh']<178.7]
    assert low and all(c['boundary_reason']=='trim fold' for c in low)
    assert all(c['branch_selection']['limit_verified'] for c in low)
    assert not any(c['numerical_gap_brackets'] or c['interior_failures'] for c in columns)
    original=json.loads(Path('outputs/em/0bac2d3f13e0aff28d6f/data.json').read_text())['aircraft'][0]
    exclusions=[]
    for c in low:
        p=c['boundary'];e=p['envelope_limit'];assert p['valid'] and e['axis']==3
        assert e['command_bracket'][0]<p['solution'][3]<e['command_bracket'][1]
        assert all(p['load_g']-n>.0003 for n in e['bracket_loads_g'])
        old=next((q for q in original['columns'] if q['speed_kmh']==c['speed_kmh']),None)
        if old and old['numerical_gap_brackets']:
            n=np.mean(old['numerical_gap_brackets'][0]['valid_side_loads'])
            assert p['load_g']<n and np.isnan(column_at_load(c,[n])[0])
            assert any(q['valid'] and q['load_g']>p['load_g'] for q in c['branch_search_points'])
            exclusions.append(dict(speed_kmh=c['speed_kmh'],connected_limit_g=p['load_g'],former_gap_load_g=float(n)))
    holdouts=[]
    for speed in [153.,161.,169.,174.,177.]:
        left=max((c for c in columns if c['speed_kmh']<speed),key=lambda c:c['speed_kmh'])
        right=min((c for c in columns if c['speed_kmh']>speed),key=lambda c:c['speed_kmh'])
        cap=min(c['boundary']['load_g'] for c in (left,right))
        for fraction in [.5,.9,.98]:
            n=1+(cap-1)*fraction;p=independently_continue(solver,speed,n)
            predicted=float(speed_interpolate(columns,[speed],[n])[0,0]);error=abs(predicted-p['ps_mps'])
            assert np.isfinite(error) and error<.5,(speed,n,error)
            holdouts.append(dict(speed_kmh=speed,load_g=n,error_mps=error))
    machine=AircraftBodyNative();native=[]
    for c in low[::max(1,len(low)//5)]:
        p=c['boundary'];v=solver.point_value(p);r=v['result'];g=v['geometry']
        assert v['force_error_g']<2e-4 and max(abs(v['rate_residual']))<5e-5 and v['history_error']<2e-4
        got=machine.call(solver.model,v['velocity'],g['omega'].tolist(),solver.mass,v['allocation']['commands'],solver.config['altitude_m'],solver.dt,v['history_input'],flaps=v['flaps'],throttle=solver.config['throttle'],ground_height=-1e6,quaternion=g['quaternion'])
        full=machine.extend(r['engine_force'],r['engine_moment'],[0.]*3,[0.]*3,1.,solver.fm.get('ExtThrustBaseMult',1.),solver.dt)
        assert got['forces']=={k:r['component_forces'][k] for k in got['forces']}
        assert full['force']==r['force'] and full['moment']==r['stored_moment']
        native.append(dict(speed_kmh=p['speed_kmh'],load_g=p['load_g'],force_error_g=p['force_error_g'],angular_error_rad_s2=p['angular_error_rad_s2']))
    edges=[c for c in a['boundary_columns'] if 178.7<c['speed_kmh']<179.1]
    assert edges and all(c['boundary']['turn_dps']>16.6 for c in edges),'Spurious control-root notch'
    report=dict(status='PASS',data=args.data,fold_columns=len(low),native=native,independent_holdouts=holdouts,
                max_error_mps=max(p['error_mps'] for p in holdouts),excluded_disconnected_intervals=exclusions,
                remaining_feature='Real change in connected trim branch near 179 km/h retained; no smoothing across failed states')
    Path(args.report).write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2),flush=True)

if __name__=='__main__':main()
