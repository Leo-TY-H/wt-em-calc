"""Independent checks for the F-16XL trim-branch boundary discontinuity."""
import argparse,json
from pathlib import Path
import numpy as np
from em_solver import TrimSolver
from em_sampling import speed_interpolate,column_at_load
from verify_em_trim_contours import independently_continue
from verify_aircraft_body_native import AircraftBodyNative

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'analysis/f16xl-envelope'


def main():
    ap=argparse.ArgumentParser();ap.add_argument('data')
    ap.add_argument('--report',default=str(OUT/'validation.json'));args=ap.parse_args()
    aircraft=json.loads(Path(args.data).read_text())['aircraft'][0]
    solver=TrimSolver('f_16xl',aircraft['settings'])
    columns=aircraft['columns']
    outline=[c for c in aircraft['boundary_columns'] if 190.<=c['speed_kmh']<=350.]
    changes=np.diff([c['boundary']['load_g'] for c in outline])
    assert all(c['boundary_status']=='verified limit' for c in outline)
    assert min(changes)>-.0002, min(changes)
    recovered=next(c for c in columns if c['speed_kmh']==221.875)
    assert 2.55<recovered['boundary']['load_g']<2.56
    assert recovered['boundary']['commands'][1]>.9
    holdouts=[];native_points=[];failures=[]
    for speed in [216.3,220.1,224.5,230.2,235.9,236.49,238.3,250.2,280.6,335.4]:
        left=max((c for c in columns if c['speed_kmh']<speed),key=lambda c:c['speed_kmh'])
        right=min((c for c in columns if c['speed_kmh']>speed),key=lambda c:c['speed_kmh'])
        cap=min(c['boundary']['load_g'] for c in (left,right))
        for fraction in (.8,.94,.99):
            load=1.+(cap-1.)*fraction
            point=independently_continue(solver,speed,load)
            predicted=float(speed_interpolate(columns,[speed],[load])[0,0])
            error=abs(predicted-point['ps_mps'])
            holdouts.append(dict(speed_kmh=speed,load_g=load,error_mps=error))
            if not np.isfinite(error) or error>.5:failures.append(holdouts[-1])
        if speed in (220.1,236.49,250.2):native_points.append(point)
    gaps=[c for c in columns if c['numerical_gap_brackets']]
    folds=[c for c in columns if c['boundary_reason']=='trim fold']
    if folds:
        # The old holes are above the newly solved connected-branch limit.
        # Do not turn those disconnected equilibria into filled performance.
        assert not gaps
        assert all(c['branch_selection']['limit_verified'] for c in folds)
    else:
        assert {150.,158.984375,167.96875,172.4609375,176.953125}<={c['speed_kmh'] for c in gaps}
    for c in gaps:
        for gap in c['numerical_gap_brackets']:
            assert np.isnan(column_at_load(c,[sum(gap['valid_side_loads'])/2.])[0])
    # Match original aero and body assembly for recovered limits/interior states.
    for speed in [219.62890625,221.875,237.59765625,257.8125,293.75]:
        native_points.append(min(columns,key=lambda c:abs(c['speed_kmh']-speed))['boundary'])
    machine=AircraftBodyNative();native=[]
    for point in native_points:
        value=solver.point_value(point);aero=value['result'];g=value['geometry']
        assert value['force_error_g']<=2e-4 and max(abs(value['rate_residual']))<=5e-5
        assert value['history_error']<=2e-4 and abs(value['ps']-point['ps_mps'])<1e-9
        got=machine.call(solver.model,value['velocity'],g['omega'].tolist(),solver.mass,
            value['allocation']['commands'],solver.config['altitude_m'],solver.dt,value['history_input'],
            flaps=value['flaps'],throttle=solver.config['throttle'],ground_height=-1e6,quaternion=g['quaternion'])
        assembled=machine.extend(aero['engine_force'],aero['engine_moment'],[0.]*3,[0.]*3,1.,
            solver.fm.get('ExtThrustBaseMult',1.),solver.dt)
        assert got['forces']=={k:aero['component_forces'][k] for k in got['forces']}
        assert assembled['force']==aero['force'] and assembled['moment']==aero['stored_moment']
        native.append(dict(speed_kmh=point['speed_kmh'],load_g=point['load_g']))
    report=dict(status='FAIL' if failures else 'PASS',failures=failures,minimum_outline_load_step_g=float(min(changes)),
        boundary_221_875_load_g=recovered['boundary']['load_g'],holdouts=holdouts,
        max_holdout_error_mps=max(p['error_mps'] for p in holdouts),native_states=native,
        retained_gap_speeds_kmh=[c['speed_kmh'] for c in gaps],binary_sha256=machine.sha,
        trim_fold_speeds_kmh=[c['speed_kmh'] for c in folds],
        scope='Independent near-boundary holdouts and original aerodynamic/body force assembly. No global branch-completeness or live-flight claim.')
    Path(args.report).write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    assert not failures,failures


if __name__=='__main__':main()
