"""Independent restart audit; does not change equations, limits or saved plots."""
import argparse,json,math,time
from pathlib import Path
from em_solver import TrimSolver


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--indices',default='');args=parser.parse_args()
    path=Path('analysis/equilibrium-gap-recovery');fixtures=json.loads((path/'fixtures.json').read_text())['fixtures']
    indices=list(map(int,args.indices.split(','))) if args.indices else range(len(fixtures))
    rows=[];start=time.monotonic()
    for index in indices:
        f=fixtures[index];s=TrimSolver(f['aircraft'],f['settings']);p=f['bad'];v,n=p['speed_kmh'],p['load_g']
        guesses=[f[k]['solution'] for k in ['bad','lower','upper']]
        for alpha in [-3.,0.,3.,8.,15.,22.,27.,30.,33.,35.,38.,41.]:
            for base in [p['solution'],[alpha,math.degrees(math.acos(1/n)),0.,0.,0.]]:
                x=list(base);x[0]=alpha;guesses.append(x)
        best=p;success=[]
        for guess in guesses:
            q=s.solve(v,n,guess,exhaustive=False)
            if q['valid']:success.append(q)
            if q['force_error_g']+10*q['angular_error_rad_s2']<best['force_error_g']+10*best['angular_error_rad_s2']:best=q
        row=dict(index=index,aircraft=s.name,speed_kmh=v,load_g=n,attempts=len(guesses),solutions=success,best=best)
        rows.append(row);(path/'restart-audit.json').write_text(json.dumps(dict(rows=rows,elapsed_s=time.monotonic()-start),indent=2))
        print(index,s.name,'valid',len(success),'F',best['force_error_g'],'alpha',best['alpha_deg'],flush=True)
    print('Elapsed',time.monotonic()-start)


if __name__=='__main__':main()
