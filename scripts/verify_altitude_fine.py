"""Regressions for interpolation error floors and fine contour continuity."""
import json
from pathlib import Path
import numpy as np
import altitude_rows
from altitude_envelope import settings
from altitude_plot import contours
from verify_altitude import checked_surface


def main():
    checks=[]
    rows=[dict(altitude_m=h,segments=[dict(bounds=[.5,1.5],coefficients=[300000.,0.,0.])]) for h in [0,1000,2000,3000]]
    mach=np.array([.98765]);expected=300000*(mach[0]-.5)**2
    actual=altitude_rows.altitude_values(rows,mach,np.array([1500.]),[0.,3000.])[0,0]
    assert abs(actual-expected)<1e-7,(actual,expected)
    old_q=np.linspace(.5,1.5,601)
    old=np.interp(mach[0],old_q,300000*(old_q-.5)**2)
    assert abs(old-expected)>.075
    checks.append('exact curve evaluation removes the old fixed-grid error floor before altitude validation')
    # A valid sharp speed curve needs sub-1 km/h refinement, even though no
    # equilibrium is missing. The old hard stop deleted its first intervals.
    config=settings(dict(speed_min_kmh=500,speed_max_kmh=1000,altitude_max_m=1000,quality='detailed'))
    class Sharp:
        def __call__(self,v,h):return dict(speed_kmh=v,altitude_m=h,valid=True,category='valid',ps_mps=40*np.sqrt(v-500+.5))
    original=altitude_rows.sampler
    try:
        altitude_rows.sampler=lambda c:Sharp()
        row=altitude_rows.sample_row((config,0,[]))
    finally:altitude_rows.sampler=original
    assert not row['masked'],row['masked']
    checks.append('Detailed resolves valid speed curvature below the former 1 km/h cutoff')
    class HiddenEdge:
        def __call__(self,v,h):
            valid=v>=570 and abs(v-600)>1e-8
            return dict(speed_kmh=v,altitude_m=h,valid=valid,ps_mps=v-585,
                        category='valid' if valid else 'numerical gap' if v>=570 else 'physical limit')
        def recover(self,v,h,left,right):
            return dict(speed_kmh=v,altitude_m=h,valid=v>=570,ps_mps=v-585,
                        category='valid' if v>=570 else 'physical limit')
    config=settings(dict(speed_min_kmh=500,speed_max_kmh=900,altitude_max_m=1000,quality='detailed'))
    try:
        altitude_rows.sampler=lambda c:HiddenEdge()
        row=altitude_rows.sample_row((config,0,[]))
    finally:altitude_rows.sampler=original
    assert row['segments'][0]['bounds'][0]*row['sound_kmh']<575
    assert abs(altitude_rows.row_values(row,np.array([585/row['sound_kmh']]))[0])<1e-8
    checks.append('an initially failed coarse edge is revisited after accepted neighbors appear, restoring its valid zero contour')
    rows=[dict(altitude_m=h,segments=[dict(bounds=[lo,1.5],coefficients=[100.,200*lo,100*lo*lo])])
          for h,lo in [(0,.2),(1000,.35),(2000,.25)]]
    value=altitude_rows.fixed_values(rows,np.array([.5]),np.array([500.]))[0,0]
    assert abs(value-25)<1e-10
    assert np.isnan(altitude_rows.fixed_values(rows,np.array([.3]),np.array([500.]))[0,0])
    checks.append('fixed-Mach fallback preserves a known field across moving edges and never extrapolates across an excluded endpoint')
    for quality,tolerance in [('smooth',.5),('detailed',.15)]:
        config=settings(dict(speed_min_kmh=200,speed_max_kmh=1000,altitude_max_m=12000,quality=quality))
        def curved(v,h):return dict(valid=True,converged=True,reasons=[],ps_mps=64-((v-600)/40)**2-((h-6000)/600)**2)
        mesh=checked_surface(config,curved)
        zero=next(c for c in contours(dict(mesh,settings=config)) if c['sep_mps']==0)
        assert not mesh['masked_cells'],mesh['masked_cells']
        assert len(zero['paths'])==1 and np.linalg.norm(np.array(zero['paths'][0][0])-zero['paths'][0][-1])<1e-8
        error=max(abs(curved(v,h)['ps_mps']) for v,h in zero['paths'][0])
        assert error<tolerance,(quality,error)
        checks.append(f'{quality}: closed analytic contour remains continuous, maximum error {error:.6f} m/s below {tolerance} m/s')
    report=dict(status='PASS',checks=checks)
    Path('analysis/altitude-envelope/fine-sampling-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
