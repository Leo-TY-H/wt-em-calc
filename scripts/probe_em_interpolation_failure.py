"""Capture the exact sample coordinates behind an interpolation exception."""
import inspect,json,sys
from pathlib import Path
import numpy as np
import scipy.interpolate

original=scipy.interpolate.PchipInterpolator
def capture(x,y,*args,**kwargs):
    xx=np.asarray(x)
    if len(xx)>1 and np.any(np.diff(xx)<=0):
        f=inspect.currentframe().f_back
        loc=f.f_locals
        points=loc.get('valid',loc.get('good',[]))
        report=dict(function=f.f_code.co_name,line=f.f_lineno,x=xx.tolist(),
            speed=loc.get('speed'),top=loc.get('top'),
            points=points if isinstance(points,list) and all(isinstance(p,dict) for p in points) else None)
        Path('analysis/speed-boundary-fixes/interpolation-failure.json').write_text(json.dumps(report,indent=2)+'\n')
    return original(x,y,*args,**kwargs)
scipy.interpolate.PchipInterpolator=capture

if __name__=='__main__':
    from em_solver import compute,settings
    cfg=settings(dict(aircraft=['a_4b'],instructor='--on' in sys.argv,speed_samples=9,load_samples=7,sep_tolerance_mps=1.))
    print('reproducing',flush=True)
    compute(cfg)
