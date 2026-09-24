"""Independent trims inside speed strips previously erased by boundary checks."""
import argparse,math
from pathlib import Path
import orjson
import numpy as np
from em_solver import TrimSolver
from em_speed_seam import plot_values
from em_accuracy import neighboring_loads,turn_tolerance,within_contour_band,visible_error
from verify_em_contours import check


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('data',nargs='+');ap.add_argument('--report',required=True)
    args=ap.parse_args();rows=[];failures=[]
    for filename in args.data:
        data=orjson.loads(Path(filename).read_bytes())
        for a in data['aircraft']:
            solver=TrimSolver(a.get('aircraft_id',a['id']),a['settings']);columns=a['columns'];samples=[]
            s=a['surface'];xs=np.asarray(s['x']);z=np.asarray(s['z'],float)
            solved=[c['speed_kmh'] for c in columns if c['boundary']]
            inside=(xs>min(solved))&(xs<max(solved))
            blank=xs[inside&~np.isfinite(z).any(axis=0)].tolist()
            unresolved=[c['speed_kmh'] for c in columns if c['boundary_status']=='unresolved numerical boundary']
            assert not blank and not unresolved,(a['name'],blank,unresolved)
            for cert in a['interpolation'].get('certified_speed_interiors',[]):
                lo,hi=cert['speed_interval_kmh'];bottom,top=cert['load_interval_g']
                near=min(columns,key=lambda c:abs(c['speed_kmh']-lo))
                for fraction in (.21,.73):
                    speed=lo+(hi-lo)*fraction
                    for f in (0.,.057,.341,.733,.947,.999):
                        load=bottom+(top-bottom)*f
                        seed=min((p for p in near['points'] if p['valid']),key=lambda p:abs(p['load_g']-load))
                        point=solver.solve(speed,load,seed['solution'],exhaustive=True)
                        actual=point['ps_mps'];pred=float(plot_values(columns,cert,np.array([speed]),np.array([[load]]))[0,0])
                        band=np.clip(neighboring_loads(speed,np.array([load]),turn_tolerance(a['settings']['sep_tolerance_mps'])),bottom,top)
                        nearby=plot_values(columns,cert,speed,band)[0]
                        error=abs(actual-pred)
                        accurate=(error<=a['settings']['sep_tolerance_mps'] or
                            bool(within_contour_band(actual,nearby,a['settings']['sep_tolerance_mps'])) or
                            not bool(visible_error(actual,pred)))
                        passed=point['valid'] and math.isfinite(pred) and accurate
                        row=dict(speed_kmh=speed,load_g=load,error_mps=error,valid=point['valid'],passed=passed,
                            force_error_g=point['force_error_g'],angular_error_rad_s2=point['angular_error_rad_s2'])
                        samples.append(row)
                        if not passed:failures.append(dict(aircraft=a['name'],**row))
            contour=check(a);assert not contour['failures'] and not contour['missing_upper_edge_vertices'],contour
            row=dict(aircraft=a['name'],instructor=a['settings']['instructor'],data=filename,blank_speed_columns=blank,
                unresolved_boundaries=unresolved,independent_holdouts=samples,contours=contour)
            rows.append(row);print(a['name'],'holdouts',len(samples),'blank columns',len(blank),'unresolved boundaries',len(unresolved),flush=True)
    Path(args.report).write_bytes(orjson.dumps(dict(status='FAIL' if failures else 'PASS',cases=rows,failures=failures),option=orjson.OPT_INDENT_2))
    assert not failures,failures

if __name__=='__main__':main()
