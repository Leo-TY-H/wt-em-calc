"""Independent manufactured-surface and export checks for altitude contours."""
import csv
import io
import json
from pathlib import Path
import tempfile
import numpy as np
from altitude_envelope import settings, LevelSampler
import altitude_rows
from altitude_plot import contours, export_csv, export_figure

ROOT = Path(__file__).resolve().parents[1]


def checked_surface(config,evaluate,**kwargs):
    class ManufacturedSampler:
        def solver(self,height):
            from types import SimpleNamespace
            return SimpleNamespace(fm={})
        def __call__(self,v,h):
            p=evaluate(v,h)
            return dict(p,speed_kmh=v,altitude_m=h,category='valid' if p['valid'] else 'numerical gap')
    original=altitude_rows.sampler
    try:
        altitude_rows.sampler=lambda config:ManufacturedSampler()
        return altitude_rows.compute_rows(config,**kwargs)
    finally:altitude_rows.sampler=original


def main():
    checks = []
    config = settings(dict(speed_min_kmh=200, speed_max_kmh=1000, altitude_max_m=12000, quality='preview',
                           conditions={'instructor': False}))
    def evaluate(v, h):
        return dict(valid=True, converged=True, reasons=[], ps_mps=(v-600)/4+(h-6000)/100)
    mesh = checked_surface(config, evaluate)
    data = dict(mesh, settings=config, aircraft_name='Manufactured linear field', speed_limits=[])
    data['contours'] = contours(data)
    assert np.isfinite(mesh['surface']['sep_mps']).any() and not mesh['masked_cells']
    for c in data['contours']:
        # Check every contour vertex against the independent analytic field,
        # including negative and zero levels, using the requested tolerance.
        for path in c['paths']:
            assert max(abs(evaluate(x, y)['ps_mps']-c['sep_mps']) for x, y in path) < 1.
    assert any(c['sep_mps'] == 0 for c in data['contours'])
    assert any(c['sep_mps'] < 0 for c in data['contours'])
    # A linear field has exactly one continuous path per visible level.
    assert all(len(c['paths']) == 1 for c in data['contours'] if c['paths'])
    checks.append('analytic linear SEP levels within 1 m/s and continuous, including negative and zero')
    def curved(v, h):
        return dict(valid=True, converged=True, reasons=[], ps_mps=64-((v-600)/40)**2-((h-6000)/600)**2)
    curved_mesh = checked_surface(config, curved)
    curved_data = dict(curved_mesh, settings=config)
    zero = next(c for c in contours(curved_data) if c['sep_mps'] == 0)
    assert len(zero['paths']) == 1
    assert np.linalg.norm(np.array(zero['paths'][0][0])-zero['paths'][0][-1]) < 1e-8
    worst = max(abs(curved(x,y)['ps_mps']) for x,y in zero['paths'][0])
    assert worst < 1., worst
    checks.append('closed nonlinear zero-SEP envelope recovered with <1 m/s analytic contour error')
    def gap(v, h):
        p=evaluate(v,h)
        if 550 <= v <= 650:
            p.update(valid=False, converged=False, reasons=['trim did not converge'])
        return p
    broken = checked_surface(config, gap)
    x=broken['surface']['speeds_kmh'];z=broken['surface']['sep_mps']
    assert np.isnan(z[(x>=550)&(x<=650)]).all()
    assert broken['masked_cells']
    checks.append('failed trim strip stays masked; contours never bridge rejected samples')
    def sloping_edges(v,h):
        lo=250+h*.012;hi=800+h*.01;valid=lo<=v<=hi
        return dict(valid=valid,converged=valid,reasons=[] if valid else ['physical boundary'],
                    ps_mps=(v-lo-2)*(hi-2-v)*.001)
    fitted=checked_surface(config,sloping_edges)
    zero_edges=next(c for c in contours(dict(fitted,settings=config)) if c['sep_mps']==0)
    assert len(zero_edges['paths'])==2
    for path in zero_edges['paths']:
        assert {round(path[0][1]),round(path[-1][1])}=={0,12000}
        assert max(abs(sloping_edges(v,h)['ps_mps']) for v,h in path)<1.
    checks.append('zero contours 2 km/h inside both moving feasible edges stay continuous across the full altitude range')
    try:
        checked_surface(config, evaluate, cancelled=lambda: True)
        raise AssertionError('Cancellation ignored')
    except InterruptedError:
        checks.append('cooperative cancellation')
    invalid = [dict(altitude_max_m=float('nan')),dict(altitude_min_m=1000,altitude_max_m=500),
               dict(aircraft='unknown'),dict(speed_min_kmh=900,speed_max_kmh=800),
               dict(conditions={'altitude_m':3000}),dict(contour_interval_mps=0),dict(quality=[]),
               dict(conditions={'throttle':True}),dict(aircraft=[])]
    for bad in invalid:
        try: settings(bad)
        except ValueError: pass
        else: raise AssertionError(('invalid input accepted',bad))
    checks.append('invalid, nonfinite and misplaced physical inputs rejected')
    assert settings({'conditions':{'instructor':True,'flaps_percent':100,'torque_gyro':True}})==settings({'conditions':{'instructor':False,'flaps_percent':0,'torque_gyro':False}})
    checks.append('legacy mode/flap inputs normalize to the identical clean, Instructor-independent calculation')
    rows = list(csv.DictReader(io.StringIO(export_csv(data))))
    assert len(rows)==len(mesh['points']) and any(float(r['ps_mps'])<0 for r in rows)
    with tempfile.TemporaryDirectory() as directory:
        for extension in ['png','svg','pdf']:
            target=Path(directory)/('diagram.'+extension)
            export_figure(data,target)
            assert target.stat().st_size>1000
    checks.append('CSV retains all samples; SVG/PNG/PDF exports render')
    # The one-sided high-altitude correction previously rejected this
    # otherwise balanced state. Check actual native closure is retained.
    sample = LevelSampler(settings(dict(conditions={'instructor':False})))
    point=sample(1200.,16000.)
    assert point['valid'],point['reasons']
    assert point['force_error_g']<=2e-4 and point['angular_error_rad_s2']<=5e-5
    assert abs(point['vertical_step_velocity_mps'])<=.005
    assert not point['altitude_correction']
    checks.append('16 km F-16 1-g state closes with unchanged tolerances and native altitude correction active')
    report=dict(status='PASS',checks=checks,nonlinear_zero_error_mps=worst)
    out=ROOT/'analysis/altitude-envelope';out.mkdir(parents=True,exist_ok=True)
    (out/'verification.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
