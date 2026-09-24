"""Physical holdouts and endpoint regression for the interactive sampler."""
import argparse,json,math
from pathlib import Path
import numpy as np
from em_solver import TrimSolver
from em_sampling import speed_interpolate

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'analysis/prop-interactive'

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--job');args=parser.parse_args()
    job=args.job or json.loads((OUT/'website-benchmark.json').read_text())['id']
    data=json.loads((ROOT/'outputs/em'/job/'data.json').read_text())
    old=json.loads((ROOT/'outputs/em/109d8bb30e5f27186606/data.json').read_text())
    prior={a['id']:a for a in old['aircraft']}
    native=[];checks=[];replays=[]
    for a in data['aircraft']:
        name=a['id'];solver=TrimSolver(name,a['settings']);columns=a['columns']
        stall=next(c for c in columns if c.get('level_stall_endpoint'))
        assert stall['boundary']['valid'] and stall['boundary']['turn_dps']==0.
        lo,hi=prior[name]['low_speed_edge']['bracket_kmh']
        assert lo<=stall['speed_kmh']<=hi,(name,'stall speed moved',stall['speed_kmh'],lo,hi)
        assert not [c['speed_kmh'] for c in columns if c['speed_kmh']>stall['speed_kmh'] and c['boundary_status']=='no feasible samples'],(name,'false low-speed holes returned')
        endpoint=next(c for c in columns if c.get('level_endpoint_bracket_kmh'))
        point=next(p for p in endpoint['sustained'] if p.get('sustained_endpoint'))
        assert point['valid'] and point['load_g']==1. and point['turn_dps']==0. and abs(point['ps_mps'])<.002
        curve=a['sustained_curve']
        assert any(x==point['speed_kmh'] and y==0. for x,y in zip(curve['x'],curve['y']))
        for p in [stall['boundary'],point,max(a['sustained'],key=lambda p:p['turn_dps'])]:
            value=solver.point_value(p)
            assert value['ps']==p['ps_mps'] and value['result']['force']==p['force_n']
            assert value['result']['stored_moment']==p['moment_nm']
            replays.append(dict(aircraft=name,speed_kmh=p['speed_kmh'],load_g=p['load_g'],exact=True))
            native.append(dict(p,aircraft=name,settings={k:v for k,v in a['settings'].items() if k!='aircraft'}))
        for speed,load in [(205.3,1.2),(277.7,1.7),(361.3,2.3),(452.9,3.4),(543.1,1.3),(610.7,4.7)]:
            predicted=float(speed_interpolate(columns,[speed],[load])[0,0])
            assert math.isfinite(predicted),(name,'masked holdout',speed,load)
            nearby=min((p for p in a['points'] if p['valid']),key=lambda p:abs(p['speed_kmh']-speed)+10*abs(p['load_g']-load))
            p=solver.solve(speed,load,nearby['solution'])
            error=abs(predicted-p['ps_mps'])
            checks.append(dict(aircraft=name,speed_kmh=speed,load_g=load,error_mps=error,valid=p['valid']))
            assert p['valid'] and error<=data['settings']['sep_tolerance_mps'],checks[-1]
        print(name,'endpoints, exact replay and off-grid checks passed',flush=True)
    (OUT/'chart-validation.json').write_text(json.dumps(dict(status='PASS',holdouts=checks,replays=replays),indent=2)+'\n')
    (OUT/'native-points.json').write_text(json.dumps(native,indent=2)+'\n')

if __name__=='__main__':main()
