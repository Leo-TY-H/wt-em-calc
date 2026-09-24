"""Compare final complete boundaries and independently withheld interior states."""
import json,math
from pathlib import Path
import numpy as np
from em_solver import TrimSolver,settings
from em_sampling import speed_interpolate

ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'analysis/performance-pass'


def main():
    report=json.loads((OUT/'web-runtime-validation.json').read_text());rows=[];failures=[]
    for result in report['rows']:
        mode=int(result['instructor'])
        data=json.loads((ROOT/'outputs/em'/result['job']/'data.json').read_text());a=data['aircraft'][0]
        baseline=json.loads((OUT/f'baseline-full-{mode}.json').read_text())['aircraft'][0]
        before={c['speed_kmh']:c for c in baseline['columns']};deltas=[]
        for column in a['columns']:
            old=before.get(column['speed_kmh'])
            if old and column.get('boundary') and old.get('boundary'):
                deltas.append(dict(speed_kmh=column['speed_kmh'],turn_dps=abs(column['boundary']['turn_dps']-old['boundary']['turn_dps']),
                    alpha_deg=abs(column['boundary']['alpha_deg']-old['boundary']['alpha_deg'])))
        old_roots={p['speed_kmh']:p for p in baseline['sustained']}
        root_deltas=[abs(p['turn_dps']-old_roots[p['speed_kmh']]['turn_dps']) for p in a['sustained'] if p['speed_kmh'] in old_roots]
        # Interior equations do not require maximum keyboard input. Check the
        # aircraft directly at withheld speeds/loads under the plotted cap.
        solver=TrimSolver(a['id'],settings(dict(data['settings'],instructor=False)))
        columns=a['columns'];pairs=[(l,r) for l,r in zip(columns,columns[1:]) if l.get('boundary') and r.get('boundary')]
        holdouts=[]
        for index in np.linspace(1,len(pairs)-2,16,dtype=int):
            left,right=pairs[index];speed=.63*left['speed_kmh']+.37*right['speed_kmh']
            cap=min(left['boundary']['load_g'],right['boundary']['load_g'])
            for fraction in [.23,.61,.91]:
                load=1.+fraction*(cap-1.)
                predicted=float(speed_interpolate(columns,[speed],[load])[0,0])
                point=solver.solve(speed,load,left['boundary']['solution'])
                error=abs(point['ps_mps']-predicted)
                row=dict(speed_kmh=speed,load_g=load,error_mps=error if math.isfinite(error) else None,valid=point['valid'])
                holdouts.append(row)
                if not point['valid'] or not math.isfinite(error) or error>data['settings']['sep_tolerance_mps']:
                    failures.append(dict(stage='withheld surface',instructor=bool(mode),**row))
        maximum=max((d['turn_dps'] for d in deltas),default=0.)
        if maximum>.025:failures.append(dict(stage='boundary changed',instructor=bool(mode),max_turn_difference_dps=maximum))
        rows.append(dict(instructor=bool(mode),job=result['job'],boundary_comparisons=len(deltas),boundary_deltas=deltas,
            max_boundary_difference_dps=maximum,sustained_comparisons=len(root_deltas),
            max_sustained_difference_dps=max(root_deltas,default=0.),holdouts=holdouts,
            max_holdout_error_mps=max((p['error_mps'] or 0. for p in holdouts),default=0.)))
        print(mode,'boundary difference',maximum,'holdouts',len(holdouts),'failures',len(failures),flush=True)
    (OUT/'full-result-validation.json').write_text(json.dumps(dict(rows=rows,failures=failures),indent=2)+'\n')
    assert not failures,failures


if __name__=='__main__':main()
