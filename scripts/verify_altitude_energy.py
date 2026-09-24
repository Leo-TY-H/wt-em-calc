"""Known analytic SEP ridges, masking and full-range guide checks."""
import json
from pathlib import Path
import numpy as np
from altitude_energy import energy_guide
from altitude_climb import Surface


def field():
    h=np.linspace(0,10000,51);v=np.linspace(100,300,201)
    best=140+.008*h
    z=80-(v[None,:]-best[:,None])**2/100
    return dict(settings=dict(speed_min_kmh=360.,speed_max_kmh=1080.,altitude_min_m=0.,altitude_max_m=10000.),
                surface=dict(altitudes_m=h,speeds_kmh=np.broadcast_to(v[None,:]*3.6,z.shape),sep_mps=z))


def main():
    checks=[];d=field();r=energy_guide(d)
    assert r['status']=='complete',r
    assert r['points'][0]['altitude_m']==0 and r['points'][-1]['altitude_m']==10000
    assert max(abs(p['speed_kmh']/3.6-(140+.008*p['altitude_m'])) for p in r['points'])<=.5
    assert all(abs(p['sep_mps']-80)<.01 for p in r['points'])
    checks.append('known sloping analytic maximum tracked across the entire plotted altitude range')
    d=field();d['surface']['sep_mps']-=200
    r=energy_guide(d);assert r['status']=='complete' and all(p['sep_mps']<0 for p in r['points'])
    assert max(abs(p['speed_kmh']/3.6-(140+.008*p['altitude_m'])) for p in r['points'])<=.5
    checks.append('negative SEP chooses the least energy loss; zero SEP is not a barrier or height objective')
    d=field();d['surface']['sep_mps'][25,:]=np.nan
    assert energy_guide(d)['status']=='unavailable'
    checks.append('missing altitude strip cannot be crossed or filled')
    d=field();d['surface']['sep_mps'][:,95:105]=np.nan
    r=energy_guide(d);assert r['status']=='complete'
    speeds=np.array([p['speed_kmh']/3.6 for p in r['points']])
    assert np.all(speeds<195) or np.all(speeds>204)
    f=Surface(d)
    for a,b in zip(r['points'],r['points'][1:]):
        assert f.clear_edges(a['speed_kmh']/3.6,a['altitude_m'],b['speed_kmh']/3.6,b['altitude_m'],positive_only=False)
    checks.append('connected branch selected when per-row maxima lie on opposite sides of a masked barrier')
    d=field();d['surface']['sep_mps'][0,:]=np.nan
    assert energy_guide(d)['status']=='unavailable'
    checks.append('guide never silently starts above the requested minimum altitude')
    report=dict(status='PASS',checks=checks)
    Path('analysis/altitude-envelope/energy-guide-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
