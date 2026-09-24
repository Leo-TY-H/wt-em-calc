"""Pinned vehicle -> actual FM -> installed collision resource mapping."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]


def main():
    vehicles=json.loads((ROOT/'references/prop-vehicles/manifest.json').read_text())['records']
    resources=json.loads((ROOT/'analysis/prop-integration/collision-index.json').read_text())['resources']
    census=json.loads((ROOT/'references/prop-native-config.json').read_text())['aircraft']
    rows=[]
    for record in census:
        name=record['aircraft']
        referenced=[v for v in vehicles if (Path(v['fm']).stem if v['fm'] else v['vehicle'])==name]
        matches=referenced or [v for v in vehicles if v['vehicle']==name]
        assets=sorted({v['model']+'_collision' for v in matches if v['model'] and v['model']+'_collision' in resources})
        rows.append(dict(aircraft=name,vehicles=matches,assets=assets,referenced=bool(referenced)))
    (ROOT/'analysis/prop-integration/geometry-mapping.json').write_text(json.dumps(rows,indent=2)+'\n')
    print(len(rows),'records',sum(len(r['assets']) for r in rows),'geometries')
    print('Unresolved:',[r['aircraft'] for r in rows if not r['assets']])
    print('Unreferenced:',[r['aircraft'] for r in rows if not r['referenced']])

if __name__=='__main__':main()
