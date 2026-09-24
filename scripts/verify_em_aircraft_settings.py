"""Independent physical conditions must match independent single-aircraft runs."""
import csv,io,json
from em_solver import settings,aircraft_settings,TrimSolver
from em_plot import export_csv
from em_sampling import sample_column

def main():
    ids=['f_16a_block_10_mod','f_14a_early']
    cfg=settings(dict(aircraft=ids,aircraft_settings={ids[0]:dict(altitude_m=1000,fuel_percent=70,throttle=.9,afterburner=False,flaps_percent=60,instructor=True),
        ids[1]:dict(altitude_m=3000,fuel_percent=20,throttle=1.1,afterburner=True,sweep_percent=50,structural_limits=False)}))
    assert settings(cfg)==cfg
    aircraft=[]
    for name in ids:
        c=aircraft_settings(cfg,name);independent=TrimSolver(name,c);combined=TrimSolver(name,cfg)
        for speed,load in [(500,1),(800,3),(1100,5)]:
            a=combined.solve(speed,load);b=independent.solve(speed,load)
            assert a==b,(name,speed,load)
        assert combined.mass==independent.mass
        aircraft.append(dict(id=name,settings=c,points=[a]))
    records=list(csv.DictReader(io.StringIO(export_csv(dict(settings=cfg,aircraft=aircraft)))))
    assert [float(r['altitude_m']) for r in records]==[1000,3000]
    assert [float(r['fuel_percent']) for r in records]==[70,20]
    # A global flap request must not wrongly reject a retracted-flap swing wing.
    c=settings(dict(cfg,flaps_percent=100,sweep_percent=80));assert aircraft_settings(c,ids[1])['flaps_percent']==0
    for overrides in [{ids[0]:{'speed_samples':25}},{ids[0]:{'fuel_percent':-1}},{ids[0]:{'instructor':'true'}},{'unknown':{}}]:
        try:settings(dict(cfg,aircraft_settings=overrides))
        except ValueError:pass
        else:raise AssertionError(overrides)
    print('6 independent/combined point comparisons, mass, CSV conditions, normalization and invalid override checks passed')
if __name__=='__main__':main()
