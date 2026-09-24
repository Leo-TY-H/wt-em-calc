"""Same-aircraft comparison retains independent results and export identities."""
import csv
import io
from unittest.mock import patch
from em_solver import settings, compute
from em_plot import export_csv


def main():
    name='f_16a_block_10_mod'
    cfg=settings(dict(aircraft=[name],compare_instructor=True,sampling='regular',
                      speed_min_kmh=700,speed_max_kmh=800,speed_samples=7,
                      load_samples=5,max_load_g=2,aircraft_settings={name:dict(fuel_percent=45)}))
    assert settings(cfg)==cfg
    for update in [dict(aircraft=[name,'f_14a_early']),dict(compare_instructor='true'),
                   dict(aircraft_settings={name:dict(extra_mass_kg=10)})]:
        try:settings(dict(cfg,**update))
        except ValueError:pass
        else:raise AssertionError(update)
    # Use real independently solved points to exercise combination and export
    # without repeating a full expensive Instructor envelope sweep.
    from em_solver import TrimSolver, AIRCRAFT
    def sample(config,progress,cancelled):
        solver=TrimSolver(name,config)
        point=solver.solve(750,1.5)
        return dict(settings=config,speeds_kmh=[750],loads_g=[1.5],assumptions=[],elapsed_s=0,
                    aircraft=[dict(id=name,**AIRCRAFT[name],settings=solver.config,
                                   points=[point],sustained=[],valid_points=int(point['valid']))])
    with patch('em_solver.compute_regular',side_effect=sample):
        data=compute(cfg)
    off,on=data['aircraft']
    assert off['id']!=on['id'] and off['color']!=on['color']
    assert off['settings']['instructor'] is False and on['settings']['instructor'] is True
    assert off['settings']['fuel_percent']==on['settings']['fuel_percent']==45
    assert {k:v for k,v in off['settings'].items() if k!='instructor'}=={k:v for k,v in on['settings'].items() if k!='instructor'}
    rows=list(csv.DictReader(io.StringIO(export_csv(data))))
    assert len({r['aircraft'] for r in rows})==2
    assert {r['instructor_enabled'] for r in rows}=={'True','False'}
    with patch('em_solver.compute_regular',side_effect=AssertionError('Should cancel first')):
        try:compute(cfg,cancelled=lambda:True)
        except InterruptedError:pass
        else:raise AssertionError('Cancellation ignored')
    print('Comparison validation, real points, matched conditions, unique identities, CSV and cancellation passed')

if __name__=='__main__':main()
