"""Regression checks for numerical recovery without changing aircraft physics."""
import json,ast
from pathlib import Path
import numpy as np
from em_solver import TrimSolver,settings,ROOT
from em_sampling import load_coordinate
from em_recovery import recover_equilibrium


def main():
    rows=[]
    # Independent equilibria at adjacent loads provide real branch seeds. The
    # alternate coordinates must converge to the same interior performance.
    for name in ['f_16a_block_15_adf','saab_jas39c','a_6e_tram','f_14a_early','jaguar_a','su_25t']:
        s=TrimSolver(name,settings(dict(aircraft=[name])))
        left=s.solve(800,2.9);right=s.solve(800,3.1,left['solution'])
        direct=s.solve(800,3.,left['solution'])
        recovered=recover_equilibrium(s,800,3.,[left,right])
        assert direct['valid'] and recovered and recovered['valid'],name
        error=abs(direct['ps_mps']-recovered['ps_mps'])
        assert error<.002,(name,error)
        rows.append(dict(aircraft=name,direct_ps=direct['ps_mps'],recovered_ps=recovered['ps_mps'],error_mps=error))
    lo=np.nextafter(1.,2.);hi=np.nextafter(lo,2.)
    assert 0<load_coordinate(lo,10.)<load_coordinate(hi,10.)
    def operating_point(path):
        tree=ast.parse(Path(path).read_text())
        return ast.dump(next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='operating_point'))
    assert operating_point(ROOT/'scripts/em_solver.py')==operating_point(ROOT/'analysis/em-three-tasks/baseline-em_solver.py')
    # A genuine failed-search regression from the catalog audit. No aircraft
    # identity is consulted by production recovery; this is a saved test case.
    before=json.loads((ROOT/'analysis/em-three-tasks/marut-baseline-column.json').read_text())
    after=json.loads((ROOT/'analysis/em-three-tasks/marut-recovered-column.json').read_text())
    point=next(p for p in after['points'] if p.get('recovery_method'))
    assert not next(p for p in before['points'] if p['load_g']==point['load_g'])['valid']
    s=TrimSolver('marut_mk1',settings(dict(aircraft=['marut_mk1'],flaps_percent=100,sep_tolerance_mps=1.)))
    recovered=recover_equilibrium(s,800,point['load_g'],before['points'])
    assert recovered and recovered['valid']
    report=dict(interior_comparisons=rows,stable_near_level_coordinate=True,operating_point_unchanged=True,
                genuine_failed_point_recovered=dict(aircraft=s.name,load_g=point['load_g'],point=recovered))
    (ROOT/'analysis/em-three-tasks/recovery-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
if __name__=='__main__':main()
