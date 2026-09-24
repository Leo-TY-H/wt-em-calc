"""Independent energy/time and obstacle checks for the climb planner."""
import json
from pathlib import Path
import numpy as np
from altitude_climb import climb_route,Surface,G


def field(power=10.):
    h=np.linspace(0,2000,65);v=np.linspace(100,200,81)
    return dict(settings=dict(speed_min_kmh=360.,speed_max_kmh=720.,altitude_min_m=0.,altitude_max_m=2000.),
                surface=dict(altitudes_m=h,speeds_kmh=np.broadcast_to(v[None,:]*3.6,(len(h),len(v))),
                             sep_mps=np.full((len(h),len(v)),power)))


def main():
    checks=[];d=field();r=climb_route(d,{'start_speed_kmh':360},speed_nodes=81)
    assert r['status']=='complete',r
    assert abs(r['elapsed_s']-200)<1e-6,r['elapsed_s']
    assert abs(r['end_speed_kmh']-360)<1e-5
    checks.append('constant SEP: exact 2,000 m / 10 m/s = 200 s optimum, without kinetic-energy advantage')
    # For constant power, time must equal TOTAL energy-height gain / power,
    # including kinetic energy. This independently checks every edge.
    r=climb_route(d,{'start_speed_kmh':540},speed_nodes=81)
    assert r['status']=='complete'
    expected=(2000+(r['end_speed_kmh']/3.6)**2/(2*G)-150**2/(2*G))/10
    assert abs(r['elapsed_s']-expected)<1e-6,(r['elapsed_s'],expected)
    for a,b in zip(r['points'],r['points'][1:]):
        de=(b['altitude_m']-a['altitude_m'])+((b['speed_kmh']/3.6)**2-(a['speed_kmh']/3.6)**2)/(2*G)
        assert abs((b['elapsed_s']-a['elapsed_s'])-de/10)<1e-6
        assert b['altitude_m']>=a['altitude_m'] and b['elapsed_s']>a['elapsed_s']
        assert (b['altitude_m']-a['altitude_m'])/(b['elapsed_s']-a['elapsed_s'])<=max(a['speed_kmh'],b['speed_kmh'])/3.6+1e-6
    checks.append('kinetic-energy trade charged correctly, positive time, no descent or instantaneous zoom')
    broken=field();broken['surface']['sep_mps'][32,:]=np.nan
    assert climb_route(broken,{'start_speed_kmh':360},speed_nodes=81)['status']=='unavailable'
    checks.append('one-row missing-data barrier cannot be jumped, including by long graph edges')
    # A narrow hole between quadrature nodes is caught by the whole-cell test.
    broken=field();broken['surface']['sep_mps'][1,21]=np.nan;s=Surface(broken)
    assert not s.clear_edges(np.array([100.]),np.array([0.]),np.array([150.]),np.array([31.25]))[0]
    checks.append('whole-cell mask test catches holes between integration probes')
    assert climb_route(field(-5),speed_nodes=81)['status']=='unavailable'
    checks.append('no route fabricated when power and starting kinetic energy cannot reach the target')
    # With negative SEP, sufficient initial kinetic energy still permits a
    # finite-time zoom. Compare against the independent closed-form integral.
    from scipy.optimize import brentq
    r=climb_route(field(-10),{'start_speed_kmh':720,'target_altitude_m':500},speed_nodes=81)
    assert r['status']=='complete' and r['terminal_zoom'],r
    def gain(v):return ((200**2-v*v)/2-10*(200-v)+100*np.log(210/(v+10)))/G
    end=brentq(lambda v:gain(v)-500,100,200)
    expected=((200-end)-10*np.log(210/(end+10)))/G
    assert abs(r['elapsed_s']-expected)<1e-4,(r['elapsed_s'],expected)
    assert abs(r['end_speed_kmh']/3.6-end)<1e-4
    checks.append('negative-SEP zoom matches closed-form time and final speed; kinetic energy is spent')
    changing=field();changing['surface']['sep_mps'][:]=15-.03*changing['surface']['altitudes_m'][:,None]
    r=climb_route(changing,{'start_speed_kmh':720,'target_altitude_m':1000},speed_nodes=81)
    assert r['status']=='complete' and r['points'][-1]['sep_mps']<0
    assert all(np.isfinite(p['elapsed_s']) for p in r['points'])
    checks.append('finite-time terminal arc crosses SEP=0 without an energy singularity or a fake ceiling')
    for options in [{'start_speed_kmh':True},{'target_altitude_m':float('nan')},{'start_altitude_m':1500,'target_altitude_m':1000},{'start_speed_kmh':800},{'extra':1}]:
        try:climb_route(d,options,speed_nodes=81)
        except ValueError:pass
        else:raise AssertionError(options)
    checks.append('nonfinite/out-of-range/reversed inputs rejected')
    report=dict(status='PASS',checks=checks)
    Path('analysis/altitude-envelope/climb-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
