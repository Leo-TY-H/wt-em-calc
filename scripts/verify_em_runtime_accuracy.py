"""Independent speed/load holdouts and native-replay points for runtime changes."""
import argparse,json,math
from pathlib import Path
from em_solver import TrimSolver,ROOT
from em_sampling import speed_interpolate
from em_surface import envelope_limits,eligible
from em_accuracy import neighboring_loads,within_contour_band,visible_error,surface_band_values
from em_sideslip import recover_scalar_sideslip,recover_sideslip


def main():
    ap=argparse.ArgumentParser();ap.add_argument('data');ap.add_argument('--report',required=True);ap.add_argument('--regressions',nargs='*',default=[]);args=ap.parse_args()
    data=json.loads(Path(args.data).read_text());rows=[];failures=[];native=[]
    for a in data['aircraft']:
        name=a.get('aircraft_id',a['id']);solver=TrimSolver(name,a['settings'])
        columns=a['columns'];masks=a.get('interpolation',{}).get('unresolved_speed_intervals',[])
        spans=[(l,r) for l,r in zip(columns,columns[1:]) if all(eligible(c) for c in (l,r)) and
               not any(lo<l['speed_kmh']*.63+r['speed_kmh']*.37<hi for lo,hi in masks)]
        assert spans,(name,'No displayed, verified speed intervals to test')
        # Stratify over physical speed, not the number of adaptive columns.
        # A tiny discontinuity can contain many columns; sampling by index
        # overrepresented those masks and left too few visible holdouts.
        low,high=spans[0][0]['speed_kmh'],spans[-1][1]['speed_kmh'];selected=set()
        for fraction in (.08,.23,.41,.59,.77,.92):
            remaining=set(range(len(spans)))-selected
            if not remaining:break
            target=low+(high-low)*fraction
            selected.add(min(remaining,key=lambda i:abs(spans[i][0]['speed_kmh']*.63+spans[i][1]['speed_kmh']*.37-target)))
        indices=sorted(selected)
        queries=[]
        for index in indices:
            left,right=spans[index];speed=left['speed_kmh']*.63+right['speed_kmh']*.37
            bottom=max(c['lower_boundary']['load_g'] for c in (left,right));top=min(c['boundary']['load_g'] for c in (left,right))
            queries.extend((left,right,speed,bottom+(top-bottom)*fraction,fraction)
                           for fraction in (.013,.173,.517,.827,.971,.995))
        for filename in args.regressions:
            for row in json.loads(Path(filename).read_text()).get('failures',[]):
                if row['aircraft']!=name or not row['valid'] and not row.get('expect_recovered'):continue
                speed=row['speed_kmh'];load=row['load_g']
                left=max((c for c in columns if c['speed_kmh']<=speed),key=lambda c:c['speed_kmh'])
                right=min((c for c in columns if c['speed_kmh']>=speed),key=lambda c:c['speed_kmh'])
                if not any(abs(q[2]-speed)<1e-8 and abs(q[3]-load)<1e-8 for q in queries):
                    queries.append((left,right,speed,load,'prior failed holdout'))
        for left,right,speed,load,fraction in queries:
            predicted=float(speed_interpolate(columns,[speed],[load])[0,0])
            neighbors=[p for c in (left,right) for p in c['points'] if p['valid']]
            seed=min(neighbors,key=lambda p:abs(p['load_g']-load))
            point=solver.solve(speed,load,seed['solution'])
            if not point['valid']:
                point=recover_scalar_sideslip(solver,speed,load,point) or recover_sideslip(solver,speed,load,point,neighbors) or point
            error=abs(predicted-point['ps_mps']) if math.isfinite(predicted) else None
            position_target=a.get('interpolation',{}).get('target_contour_dps',0.)
            band=neighboring_loads(speed,[load],position_target)[0]
            limits=envelope_limits(columns,[speed])[0]
            if all(math.isfinite(v) for v in limits):band=band.clip(*limits)
            nearby=speed_interpolate(columns,[speed],band)[:,0]
            geometry=bool(within_contour_band(point['ps_mps'],nearby,data['settings']['sep_tolerance_mps']))
            if a.get('interpolation',{}).get('target_speed_kmh'):
                geometry=geometry or bool(within_contour_band(point['ps_mps'],surface_band_values(columns,speed,[load],data['settings'])[0],data['settings']['sep_tolerance_mps']))
            visible=bool(visible_error(point['ps_mps'],predicted)) if a.get('interpolation',{}).get('checked_sep_range_mps') else True
            masked=(speed not in {c['speed_kmh'] for c in columns} and any(
                lo<speed<hi for lo,hi in a.get('interpolation',{}).get('unresolved_speed_intervals',[])))
            passed=point['valid'] and error is not None and (not visible or error<=data['settings']['sep_tolerance_mps'] or position_target>0. and geometry)
            row=dict(aircraft=name,speed_kmh=speed,load_g=load,valid=point['valid'],error_mps=error,
                     displayed_range=visible,within_contour_tolerance=geometry,passed=None if masked else passed,
                     declared_speed_mask=masked,
                     predicted_mps=predicted,actual_mps=point['ps_mps'],reasons=point['reasons'],
                     force_error_g=point['force_error_g'],angular_error_rad_s2=point['angular_error_rad_s2'],
                     solution=point['solution'],fraction=fraction)
            rows.append(row)
            if not masked and not passed:failures.append(row)
            if fraction in (.995,'prior failed holdout'):print(name,'holdout speed',round(speed,2),flush=True)
        for c in [spans[i][0] for i in indices[::2]]:
            for point in [c['boundary'],*c['sustained'][:1]]:
                native.append(dict(point,aircraft=name,settings={k:v for k,v in a['settings'].items() if k!='aircraft'}))
        for point in a['points']:
            if point.get('recovery_method','').startswith('balanced scalar sideslip'):
                native.append(dict(point,aircraft=name,settings={k:v for k,v in a['settings'].items() if k!='aircraft'}));break
    masked=[r for r in rows if r['declared_speed_mask']]
    report=dict(holdouts=rows,failures=failures,masked_holdouts=masked,
                max_error_mps=max((r['error_mps'] or 0 for r in rows),default=0),target_mps=data['settings']['sep_tolerance_mps'])
    Path(args.report).write_text(json.dumps(report,indent=2)+'\n')
    Path(args.report).with_suffix('.points.json').write_text(json.dumps(native)+'\n')
    print('PASS' if not failures else 'FAIL',len(rows)-len(masked),'displayed independent holdouts;',
          len(masked),'holdouts inside declared speed masks; maximum raw Ps error',report['max_error_mps'],flush=True)
    assert not failures,failures

if __name__=='__main__':main()
