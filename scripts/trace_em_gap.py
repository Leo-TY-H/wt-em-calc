"""Trace the unchanged four balance equations while sweeping physical alpha."""
import argparse,json,time
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from em_solver import TrimSolver


def main():
    parser=argparse.ArgumentParser();parser.add_argument('index',type=int);parser.add_argument('--count',type=int,default=201);args=parser.parse_args()
    root=Path('analysis/equilibrium-gap-recovery');f=json.loads((root/'fixtures.json').read_text())['fixtures'][args.index]
    s=TrimSolver(f['aircraft'],f['settings']);p=f['bad'];speed=p['speed_kmh']/3.6;load=p['load_g'];rows=[];start=time.monotonic()
    ranges=[(f['lower']['alpha_deg']-.2,f['upper']['alpha_deg']+.2)]
    for direction in [1,-1]:
        lo,hi=ranges[0];angles=np.linspace(lo,hi,args.count)[::direction];z=np.array(f['lower' if direction==1 else 'upper']['solution'][1:])
        for alpha in angles:
            def detail(z):return s.operating_point(speed,load,[alpha,*z])
            def fun(z):return detail(z)['residual'][1:]
            def jac(z):
                base=fun(z);columns=[]
                for i,h in enumerate([.002,.0001,.0001,.0001]):
                    dz=np.eye(4)[i]*h;columns.append((fun(z+dz)-base)/h)
                return np.column_stack(columns)
            r=least_squares(fun,z,jac=jac,bounds=([-15.,-1.,-1.,-1.],[89.7,1.,1.,1.]),x_scale=[30.,.2,.2,.2],max_nfev=60,xtol=1e-10,ftol=1e-10,gtol=1e-10)
            z=r.x;d=detail(z);a=d['result']
            rows.append(dict(direction=direction,alpha=float(alpha),solution=[float(alpha),*z.tolist()],residual=d['residual'].tolist(),
                valid_other=bool(max(abs(d['residual'][1:]))<=.0005),history_error=d['history_error'],stall_margin=d['stall_margin'],
                wing_blend=a['wing']['blend'],tail_angles=a['tail']['effective_angles'],ps=d['ps']))
    report=dict(fixture=args.index,aircraft=s.name,speed_kmh=p['speed_kmh'],load_g=load,rows=rows,elapsed_s=time.monotonic()-start)
    (root/f'trace-{args.index}.json').write_text(json.dumps(report,indent=2))
    good=[r for r in rows if r['valid_other']];best=min(rows,key=lambda r:sum(x*x for x in r['residual']))
    print('Rows',len(rows),'four-equation closures',len(good),'best',best,'time',report['elapsed_s'])
if __name__=='__main__':main()
