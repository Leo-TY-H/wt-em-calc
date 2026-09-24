"""Describe the reduced suite against the supported local vehicle catalog."""
import argparse,json
from collections import Counter
from pathlib import Path
from aircraft_catalog import catalog
from benchmark_em_representative import CASES,REPRESENTATIVES


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--out',required=True);args=parser.parse_args()
    vehicles=catalog();supported={k:v for k,v in vehicles.items() if v['supported']}
    selected=[]
    for name in REPRESENTATIVES:
        case=CASES[name];v=supported[case['aircraft']]
        selected.append(dict(case=name,aircraft=case['aircraft'],name=v['name'],reason=case['reason'],mode=case['mode'],
            propulsion=v['propulsion'],engine_count=v['engine_count'],propeller_count=v.get('propeller_count',0),
            has_sweep=v['has_sweep'],has_flaps=v['has_flaps'],fuel_capacity_kg=v['fuel_capacity'],fm_id=v.get('fm_id',case['aircraft'])))
    result=dict(supported_vehicles=len(supported),propulsion_counts=dict(Counter(v['propulsion'] for v in supported.values())),
        engine_count_histogram=dict(Counter(v['engine_count'] for v in supported.values())),
        variable_sweep_count=sum(v['has_sweep'] for v in supported.values()),representatives=selected,
        regressions={k:v for k,v in CASES.items() if k not in REPRESENTATIVES},
        selection_policy='Mechanism and workload coverage selected before final timings; no aircraft-name branches in the solver. The mixed-propulsion singleton and a four-engine bomber prevent a fighter-only sample.',
        scope='Sea-level, 30% fuel, fully upgraded, healthy automatic engines; RB/SB split, clean wing except explicit full-flap regressions; 100–1800 km/h requested, native structural speed limits retained. Both Smooth and Detailed require independent accuracy checks. Finite representative coverage is not proof for every vehicle or flight condition.')
    Path(args.out).write_text(json.dumps(result,indent=2)+'\n');print(result['propulsion_counts'],len(selected),'representatives')


if __name__=='__main__':main()
