"""Entry validation, independent physical settings, previews, cache and exports."""
import copy,csv,io,json
from pathlib import Path
from unittest.mock import patch
from em_solver import settings,compute,AIRCRAFT,TrimSolver
from em_plot import export_csv
from em_sampling import _AIRCRAFT_CACHE

def main():
    name='f_16a_block_10_mod'
    entries=[dict(id=f'entry_{i+1}',aircraft_id=name,settings=dict(fuel_percent=30+i*5,instructor=bool(i%2))) for i in range(8)]
    cfg=settings(dict(entries=entries,speed_min_kmh=700,speed_max_kmh=800,speed_samples=7,load_samples=5,max_load_g=2,sampling='regular'))
    assert settings(cfg)==cfg
    for bad in [[],entries*2,[dict(entries[0],id='../bad')],[entries[0],entries[0]],
                [dict(entries[0],aircraft_id='unknown')],[dict(entries[0],settings={'fuel_percent':0})],
                [dict(entries[0],settings={'instructor':'true'})],[dict(entries[0],settings={'speed_samples':9})],
                [dict(entries[0],settings={'instructor':True,'extra_mass_kg':10})]]:
        try:settings(dict(cfg,entries=bad))
        except ValueError:pass
        else:raise AssertionError(bad)
    calls=[];previews=[]
    def sample(config,progress,cancelled):
        solver=TrimSolver(name,config);point=solver.solve(750,1.5)
        calls.append(solver.config)
        if progress:progress(dict(done=1,total=1,phase='Verified point'))
        return dict(settings=config,speeds_kmh=[750],loads_g=[1.5],assumptions=[],elapsed_s=0,
                    aircraft=[dict(id=name,**AIRCRAFT[name],settings=solver.config,
                                   points=[point],sustained=[],valid_points=int(point['valid']))])
    with patch('em_solver.compute_regular',side_effect=sample):
        result=compute(cfg,progress=lambda p:None,preview=previews.append)
    assert len({a['id'] for a in result['aircraft']})==8
    assert len({a['color'] for a in result['aircraft']})==8
    assert [a['settings']['fuel_percent'] for a in result['aircraft']]==list(range(30,70,5))
    assert [len(p['aircraft']) for p in previews]==list(range(1,8))
    assert len({r['aircraft'] for r in csv.DictReader(io.StringIO(export_csv(result)))})==8
    assert all(c['entries'] is None and c['aircraft']==[name] for c in calls)
    result['aircraft'][0]['points'][0]['solution'][0]+=1.
    assert previews[0]['aircraft'][0]['points'][0]['solution'][0]!=result['aircraft'][0]['points'][0]['solution'][0]
    assert previews[0]['aircraft'][0]['points']==previews[-1]['aircraft'][0]['points']
    with patch('em_solver.compute_regular',side_effect=AssertionError('Should cancel first')):
        try:compute(cfg,cancelled=lambda:True)
        except InterruptedError:pass
        else:raise AssertionError('Ignored cancellation')
    # Exercise the real complete-aircraft cache, supplying only the expensive
    # physical sampling result. Entry ID and display label must not affect reuse.
    _AIRCRAFT_CACHE.clear();samples=[]
    def adaptive(config,progress,cancelled,preview):
        data=sample(config,progress,cancelled)
        a=data['aircraft'][0];a['columns']=[dict(speed_kmh=750)]
        samples.append(copy.deepcopy(config))
        if preview:preview(copy.deepcopy(data))
        return data
    pair=dict(cfg,sampling='adaptive',entries=[entries[0],dict(entries[0],id='entry_copy')])
    with patch('em_sampling.compute_adaptive',side_effect=adaptive):
        cached=compute(pair,preview=lambda p:None)
        assert len(samples)==1 and cached['cached_aircraft']==['entry_copy']
        assert cached['aircraft'][0]['points']==cached['aircraft'][1]['points']
        changed=copy.deepcopy(pair);changed['entries'][1]['settings']['fuel_percent']=50
        compute(changed)
        assert len(samples)==2
    report=dict(checks=['eight independent real trim points','validation and normalization','eight unique result IDs and colors',
        'independent incremental preview snapshots','CSV identities','cancellation','exact physical cache reuse across duplicate IDs','changed settings invalidate physical cache'])
    Path('analysis/aircraft-entries/backend-validation.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))
if __name__=='__main__':main()
