"""Compare original instructions on both sides of observed EM branch jumps."""
import json
from em_solver import TrimSolver, ROOT
from verify_aircraft_native import AircraftNative


def main():
    fixtures=json.loads((ROOT/'analysis/em-gap-fixtures.json').read_text())
    machine=AircraftNative();rows=[];failures=[]
    for case in fixtures['fixtures']:
        solver=TrimSolver(case['aircraft'],fixtures['settings'])
        for offset in case['alpha_offsets']:
            x=list(case['solution']);x[0]+=offset
            detail=solver.operating_point(case['speed_kmh']/3.6,case['load_g'],x)
            aero=detail['result']
            native=machine.call(solver.model,detail['velocity'],detail['geometry']['omega'].tolist(),
                solver.mass,detail['allocation']['commands'],solver.config['altitude_m'],solver.dt,
                detail['history_input'],throttle=solver.config['throttle'],ground_height=-1e6)
            expected={'forces':{k:aero['component_forces'][k] for k in native['forces']},
                      'points':{k:aero['component_points'][k] for k in native['points']},
                      'moment':aero['raw_aero_moment']}
            for key,value in expected.items():
                if native[key]!=value:failures.append(dict(aircraft=case['aircraft'],offset=offset,field=key))
            rows.append(dict(aircraft=case['aircraft'],speed_kmh=case['speed_kmh'],load_g=case['load_g'],
                alpha_offset=offset,alpha_deg=x[0],wing_blend=aero['wing']['blend'],
                tail_angles=aero['tail']['effective_angles'],component_forces=native['forces'],moment=native['moment']))
    report=dict(binary_sha256=machine.sha,cases=rows,failures=failures,
        limitations='Original aerodynamic instruction span with documented runtime adapters and scalar math hooks; out of ground effect. Confirms force-function branch changes, not nonexistence of every possible equilibrium or a periodic orbit.')
    (ROOT/'analysis/em-gap-native-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print('NATIVE BRANCH STATES',len(rows),'FAILURES',len(failures))
    if failures:raise SystemExit(1)


if __name__=='__main__':main()
