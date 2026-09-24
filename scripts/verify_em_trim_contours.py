"""Check plotted SEP against independent continuation from level flight."""
import argparse,json,math
from pathlib import Path
import numpy as np
from scipy.interpolate import PchipInterpolator
from em_solver import TrimSolver
from em_sampling import solve_level,speed_interpolate
from verify_aircraft_body_native import AircraftBodyNative

ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'analysis/f16xl-contour-jumps'


def independently_continue(solver,speed,load):
    point=solve_level(solver,speed)
    assert point['valid'],(speed,1.)
    for n in np.linspace(1.,load,max(2,math.ceil((load-1.)/.025)+1))[1:]:
        previous=point;solver.__dict__.pop('_trim_predictor',None)
        point=solver.solve(speed,float(n),previous['solution'],exhaustive=True)
        assert point['valid'],(speed,n,point['reasons'])
        assert max(abs(np.array(point['solution'])[2:]-np.array(previous['solution'])[2:]))<.2,(speed,n,'branch switch')
    return point


def main():
    ap=argparse.ArgumentParser();ap.add_argument('data');ap.add_argument('--report',default=str(OUT/'validation.json'));args=ap.parse_args()
    data=json.loads(Path(args.data).read_text());a=data['aircraft'][0]
    columns=a['columns'];solver=TrimSolver(a.get('aircraft_id',a['id']),a['settings'])
    failures=[];checks=[];native_points=[];topology=[]
    for level in (-100.,-50.,0.):
        paths=[path for path in a['contours'] if path['level']==level
               and sum(210.<x<450. for x in path['x'])>10]
        topology.append(dict(level_mps=level,paths=len(paths)))
        if len(paths)!=1:failures.append(dict(kind='discontinuous SEP contour',level=level,paths=len(paths)))
        for path in paths:
            pts=[(x,y) for x,y in zip(path['x'],path['y']) if 215.<x<440. and y>15.]
            if len(pts)<3:continue
            for i in sorted(set(np.linspace(1,len(pts)-2,10,dtype=int))):
                speed,turn=pts[i];load=math.hypot(1.,math.radians(turn)*(speed/3.6)/9.8100004196167)
                point=independently_continue(solver,speed,load);error=abs(point['ps_mps']-level)
                row=dict(kind='drawn SEP contour',speed_kmh=speed,load_g=load,level_mps=level,error_mps=error)
                checks.append(row)
                if error>.5:failures.append(row)
                if i in (sorted(set(np.linspace(1,len(pts)-2,10,dtype=int)))[2:4]):native_points.append(point)
    # Directly checked Ps=0 roots must form one uninterrupted branch in the
    # screenshot region, not separate upper/lower runs joined or simply hidden.
    roots=[c['sustained'] for c in columns if 200.<c['speed_kmh']<450.]
    assert all(len(points)==1 for points in roots),'Missing or competing sustained root'
    curve=a['sustained_curve'];runs=[];run=[]
    for x,y in zip(curve['x'],curve['y']):
        if x is None:
            if run:runs.append(run)
            run=[]
        elif 210.<x<450.:run.append((x,y))
    if run:runs.append(run)
    assert len(runs)==1,'Drawn sustained curve still disconnected'
    assert all(abs(y2-y1)<.15*max(.001,abs(x2-x1)) for (x1,y1),(x2,y2) in zip(runs[0],runs[0][1:]))
    for speed in (216.3,220.1,236.49,257.62,269.2,300.3,335.4):
        nearby=sorted(columns,key=lambda c:abs(c['speed_kmh']-speed))[:2]
        cap=min(c['boundary']['load_g'] for c in nearby)
        for fraction in (.8,.98):
            load=1.+(cap-1.)*fraction;point=independently_continue(solver,speed,load)
            prediction=float(speed_interpolate(columns,[speed],[load])[0,0]);error=abs(prediction-point['ps_mps'])
            row=dict(kind='selected power surface',speed_kmh=speed,load_g=load,error_mps=error)
            checks.append(row)
            if not np.isfinite(error) or error>.5:failures.append(row)
    at220=min(columns,key=lambda c:abs(c['speed_kmh']-221.875))
    assert at220['speed_kmh']==221.875 and 2.55<at220['boundary']['load_g']<2.56
    assert at220['boundary']['solution'][3]>.9
    assert all(c.get('branch_selection') for c in columns if 180.<c['speed_kmh']<360.)
    # Preserve evidence that the competing high-SEP root is balanced, but does
    # not match the independently traced branch at the same physical condition.
    old=json.loads((ROOT/'outputs/em/94cc51d06fd993d2b139/data.json').read_text())['aircraft'][0]
    alternative=max(next(c for c in old['columns'] if c['speed_kmh']==257.8125)['sustained'],key=lambda p:p['load_g'])
    normal=independently_continue(solver,alternative['speed_kmh'],alternative['load_g'])
    assert normal['solution'][3]>0. and alternative['solution'][3]<0.
    assert normal['ps_mps']<alternative['ps_mps']-1.
    native_points.extend((normal,alternative))
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
        native.append(dict(speed_kmh=point['speed_kmh'],load_g=point['load_g'],ps_mps=point['ps_mps']))
    report=dict(status='FAIL' if failures else 'PASS',data_path=str(Path(args.data).resolve()),
        equations_fingerprint=data.get('equations_fingerprint'),failures=failures,checks=checks,
        max_error_mps=max(p['error_mps'] for p in checks),contour_components=topology,native_states=native,
        competing_root=dict(speed_kmh=normal['speed_kmh'],load_g=normal['load_g'],
            connected_ps_mps=normal['ps_mps'],alternative_ps_mps=alternative['ps_mps']),
        binary_sha256=machine.sha,scope='Fixed small-load-step continuation independent of production adaptive trace; original aero/body replay. No global uniqueness or stability claim.')
    Path(args.report).write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2),flush=True)
    assert not failures,failures


if __name__=='__main__':main()
