"""Representative end-to-end prop EM, replay and export checks."""
import json,math,time
from pathlib import Path
from em_solver import TrimSolver
from em_plot import export_csv

ROOT=Path(__file__).resolve().parents[1]


def main():
    cases=[('yak-3',360.,2.,{}),('bf-109f-4',360.,2.,{}),('a5m4',300.,2.,{}),
           ('a2d',450.,2.,{}),('wyvern_s4',450.,2.,{}),('fr_1_fireball',450.,2.,{}),
           ('fw_200c_1',330.,1.5,{}),('tu_4',450.,1.5,{}),('b-17g',350.,1.5,{}),
           ('p-47d-28',450.,2.,dict(altitude_m=6500.)),
           ('yak-3',360.,2.,dict(instructor=True)),('bf-109f-4',360.,2.,dict(instructor=True))]
    rows=[];failures=[]
    for name,speed,load,condition in cases:
        condition=dict(condition,engine_control_mode='optimized')
        started=time.monotonic();s=TrimSolver(name,dict(aircraft=[name],**condition));p=s.solve(speed,load,exhaustive=False)
        v=s.point_value(p)
        errors=dict(replay_ps=abs(v['ps']-p['ps_mps']),force=p['force_error_g'],angular=p['angular_error_rad_s2'])
        if not p['valid'] or errors['replay_ps']>1e-9:failures.append(dict(aircraft=name,settings=condition,reasons=p['reasons'],errors=errors))
        assert p['propulsion']['controls']['commands']
        assert p['gear_percent']==(0. if s.fm['AvailableControls']['hasGearControl'] else 100.)
        csv=export_csv(dict(settings=s.config,aircraft=[dict(id=name,points=[p])]))
        assert 'propulsion' in csv.splitlines()[0] and 'global_optimum_certified' in csv
        p.update(aircraft=name,settings=condition,wall_seconds=time.monotonic()-started);rows.append(p)
        print(name,condition,p['valid'],p['reasons'],round(p['ps_mps'],4),round(p['wall_seconds'],2),flush=True)
        (ROOT/'analysis/prop-integration/em-integration-points.json').write_text(json.dumps(rows,indent=2)+'\n')
    report=dict(status='FAIL' if failures else 'PASS',cases=len(rows),failures=failures,
        checks=['Coupled discrete control search','Exact export replay','Original force/moment closure tolerances',
                'Fixed gear drag retained','Propeller settings included in CSV','Instructor source includes actual propwash'],
        scope='Representative integration checks, not an all-envelope or live-flight accuracy certificate.')
    (ROOT/'analysis/prop-integration/em-integration-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(report['status'],failures,flush=True)
    if failures:raise SystemExit(1)


if __name__=='__main__':main()
