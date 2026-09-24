"""Regression checks for the automatic envelope and localized gap handling."""
import json
from pathlib import Path
import numpy as np
from em_solver import ROOT, settings
from em_sampling import speed_interpolate


def main():
    latest=json.loads((ROOT/'outputs/em/latest.json').read_text())
    data=json.loads((ROOT/'outputs/em'/latest['id']/'data.json').read_text())
    defaults=settings();failures=[];aircraft=[]
    expected=dict(altitude_m=0.,fuel_percent=30.,throttle=1.1,afterburner=True,
                  speed_min_kmh=150.,speed_max_kmh=1300.,max_load_g=None)
    for key,value in expected.items():
        if defaults[key]!=value or data['settings'][key]!=value:failures.append(dict(stage='default',setting=key))
    for a in data['aircraft']:
        columns=a['columns'];gaps=a.get('numerical_gaps',[])
        if any(c['boundary_status']!='verified limit' for c in columns):failures.append(dict(stage='physical_boundary',aircraft=a['id']))
        maximum=max(c['boundary']['load_g'] for c in columns)
        if maximum<=12.:failures.append(dict(stage='old_ceiling_remains',aircraft=a['id']))
        field=np.array(a['surface']['z'],dtype=float);empty=np.where(~np.isfinite(field).any(axis=0))[0].tolist()
        if empty:failures.append(dict(stage='blank_speed_columns',aircraft=a['id'],indices=empty))
        for gap in gaps:
            value=float(speed_interpolate(columns,[gap['speed_kmh']],[gap['load_g']])[0,0])
            if np.isfinite(value):failures.append(dict(stage='interpolated_over_gap',aircraft=a['id'],gap=gap))
            if gap['width_g']>.01:failures.append(dict(stage='coarse_gap',aircraft=a['id'],gap=gap))
            if any(abs(b-a)>.000201 for a,b in gap.get('edge_brackets_g',[])):
                failures.append(dict(stage='unrefined_gap_edge',aircraft=a['id'],gap=gap))
        # Every recorded structural limit must use the positive force bound.
        for c in columns:
            point=c['boundary'];limit=point.get('envelope_limit',{})
            if limit.get('kind')=='wing force' and not .9995<=max(point['wing_load_ratios'])<=1.:
                failures.append(dict(stage='wing_constraint',aircraft=a['id'],speed=c['speed_kmh']))
            good=[p for p in c['points'] if p['valid']]
            if good and min(p['ps_mps'] for p in good)<0<max(p['ps_mps'] for p in good):
                if any(abs(p['ps_mps'])<.02 for p in good) and not c['sustained']:
                    failures.append(dict(stage='discarded_sustained_solution',aircraft=a['id'],speed=c['speed_kmh']))
        aircraft.append(dict(id=a['id'],columns=len(columns),maximum_load_g=maximum,
                             empty_display_columns=len(empty),localized_gaps=len(gaps),
                             maximum_gap_width_g=max((g['width_g'] for g in gaps),default=0.)))
    report=dict(result=latest['id'],defaults=expected,aircraft=aircraft,failures=failures)
    (ROOT/'analysis/em-auto-envelope-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    if failures:raise SystemExit(1)


if __name__=='__main__':main()
