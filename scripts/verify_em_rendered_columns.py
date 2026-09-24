"""An unresolved upper edge must not erase directly solved interior samples."""
import json
from pathlib import Path

import numpy as np

from em_plot import enrich
from em_sampling import column_at_load


def main():
    root=Path(__file__).resolve().parents[1]
    data=json.loads((root/'analysis/em-global-september22/final-data/draken_delta.json').read_text())
    aircraft=data['aircraft'][0]
    columns=[c for c in aircraft['columns'] if c['boundary_status']=='verified limit']
    center=len(columns)//2;selected=columns[center-1:center+2]
    left,middle,right=selected
    aircraft['columns']=selected
    aircraft['boundary_columns']=[c for c in aircraft['boundary_columns']
                                  if c['speed_kmh'] in {p['speed_kmh'] for p in selected}]
    middle['boundary_status']='unresolved numerical boundary'
    for c in aircraft['boundary_columns']:
        if c['speed_kmh']==middle['speed_kmh']:c['boundary_status']='unresolved numerical boundary'
    aircraft['interpolation']['unresolved_speed_intervals']=[[left['speed_kmh'],right['speed_kmh']]]
    enrich(data)
    surface=aircraft['surface'];x=np.asarray(surface['x']);loads=np.asarray(surface['load_g'])
    z=np.asarray(surface['z'],dtype=float)
    actual=z[:,np.where(x==middle['speed_kmh'])[0][0]]
    expected=column_at_load(middle,loads);valid=np.isfinite(expected)
    assert valid.any() and np.isfinite(actual[valid]).all(),'Verified interior column was erased'
    assert np.max(abs(actual[valid]-expected[valid]))<1e-8
    open_interval=(x>left['speed_kmh'])&(x<right['speed_kmh'])&(x!=middle['speed_kmh'])
    assert open_interval.any() and not np.isfinite(z[:,open_interval]).any(),'Unchecked interpolation was filled'
    assert all(p['turn_dps'] is None for p in aircraft['boundary'] if p['speed_kmh']==middle['speed_kmh'])
    print('PASS: solved interior retained, unresolved upper edge and unchecked interpolation still marked')


if __name__=='__main__':main()
