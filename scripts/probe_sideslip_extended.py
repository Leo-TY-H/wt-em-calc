"""Broader stationary sideslip search over saved gaps; still diagnostic only."""
import json,time,multiprocessing
from concurrent.futures import ProcessPoolExecutor,as_completed
from pathlib import Path
from probe_sideslip_equilibria import SlipSolver

def one(pair):
    index,f=pair;s=SlipSolver(f['aircraft'],f['settings']);p=f['bad'];tries=0;best=None
    for magnitude in [.001,.003,.01,.03,.1,.3,1.,2.,3.,5.,8.,12.,20.,30.]:
        for beta in [magnitude,-magnitude]:
            s.beta=beta
            for key in ['bad','lower','upper']:
                q=s.solve(p['speed_kmh'],p['load_g'],f[key]['solution'],detailed=True,exhaustive=False);tries+=1
                if q['valid']:
                    best={k:v for k,v in q.items() if not k.startswith('_')};best['beta_input']=beta;best['sideslip_deg']=q['_detail']['result']['air']['beta'];break
            if best:break
        if best:break
    return dict(index=index,aircraft=s.name,speed=p['speed_kmh'],load=p['load_g'],solution=best,attempts=tries)

def main():
    fixtures=json.loads(Path('analysis/equilibrium-gap-recovery/fixtures.json').read_text())['fixtures'];rows=[];start=time.monotonic()
    with ProcessPoolExecutor(max_workers=4,mp_context=multiprocessing.get_context('spawn')) as pool:
        for f in as_completed([pool.submit(one,pair) for pair in enumerate(fixtures)]):
            r=f.result();rows.append(r)
            Path('analysis/gap-resolution/sideslip-extended.json').write_text(json.dumps(dict(rows=rows,seconds=time.monotonic()-start),indent=2)+'\n')
            print(r['index'],r['aircraft'],'found',bool(r['solution']),'sideslip',r['solution']['sideslip_deg'] if r['solution'] else None,'attempts',r['attempts'],flush=True)

if __name__=='__main__':main()
