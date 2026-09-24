"""Check speed-continuation recovery against saved, actual failed grid points."""
import copy,json,time
from pathlib import Path
from em_solver import TrimSolver,settings
from em_speed_continuation import recover_from_speed

def main():
    rows=[];start=time.monotonic()
    for name,mode,speed in [('mig_21_2000_iaf',False,1100.),('mig_21_2000_iaf',True,700.),('harrier_gr1',False,700.),('harrier_gr1',False,1100.)]:
        col=json.loads(Path(f'analysis/gap-resolution/columns/{name}-{int(mode)}-{speed:g}.json').read_text())
        s=TrimSolver(name,settings(dict(aircraft=[name],instructor=mode)))
        physical=copy.copy(s);physical.config=dict(s.config,instructor=False,aircraft_settings={})
        for gap in col['numerical_gap_brackets']:
            p=min((p for p in col['points'] if not p['valid']),key=lambda p:abs(p['load_g']-gap['load_g']))
            seeds=[]
            for offset in [-20.,-5.,-1.,-.1,.1,1.,5.,20.]:
                q=physical.solve(speed+offset,p['load_g'],p['solution'],exhaustive=False)
                if q['valid']:seeds.append(q)
            r=recover_from_speed(s,speed,p['load_g'],seeds)
            rows.append(dict(aircraft=name,instructor=mode,speed=speed,gap=gap,bad=p,seed_count=len(seeds),recovered=r))
            Path('analysis/gap-resolution/speed-recovery.json').write_text(json.dumps(dict(rows=rows,seconds=time.monotonic()-start),indent=2)+'\n')
            print(name,mode,speed,'seeds',len(seeds),'recovered',bool(r),flush=True)

if __name__=='__main__':main()
