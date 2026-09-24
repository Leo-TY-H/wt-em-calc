"""Exact comparison of compiled and readable full aircraft phase consumers."""
import ast
import hashlib
import importlib.util
import json
from pathlib import Path
import time

import numpy as np
from em_solver import TrimSolver,ROOT


def encode(value):
    def convert(item):
        if isinstance(item,np.ndarray):return item.tolist()
        if isinstance(item,np.generic):return item.item()
        raise TypeError(type(item))
    return json.dumps(value,default=convert,sort_keys=True,separators=(',',':'))


def main():
    base=ROOT/'analysis/em-upstream-september23'
    source=ROOT/'scripts/em_operating.py'
    spec=importlib.util.spec_from_file_location('readable_em_operating',source)
    reference=importlib.util.module_from_spec(spec);spec.loader.exec_module(reference)
    original=ast.parse((base/'operating-point-before-extraction.py').read_text()).body[0]
    current=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.FunctionDef))
    assert isinstance(current.body[0],ast.ImportFrom) and current.body[0].module=='em_solver'
    current.body.pop(0)
    assert ast.dump(original)==ast.dump(current),'Aircraft equations changed during extraction'
    rows=[]
    for suite,case in [('initial-level-trial',c) for c in ('p63_rb','b25_rb','j21_sb','wyvern_sb','fireball_rb','f84_rb')]+[
            ('reported-regressions','spitfire_sb100')]:
        aircraft=json.loads((base/suite/case/'data.json').read_text())['aircraft'][0]
        solver=TrimSolver(aircraft['id'],aircraft['settings'])
        if solver.is_prop:solver.engine.force_canonical=True
        columns=[c for c in aircraft['columns'] if c.get('boundary') and c['boundary']['valid']]
        for index in (len(columns)//3,2*len(columns)//3):
            point=columns[index]['boundary'];speed=point['speed_kmh']/3.6;load=point['load_g'];x=point['solution']
            expected=reference.operating_point(solver,speed,load,x,phase_certificate=True)
            text=encode(expected)
            actual=solver.operating_point(speed,load,x,phase_certificate=True)
            assert encode(actual)==text,(case,point['speed_kmh'],'complete phases')
            changed=list(x);changed[0]+=.002;changed[3]+=.0002
            mean=expected['propulsion']
            expected_proxy=reference.operating_point(solver,speed,load,changed,propulsion_override=mean)
            proxy=solver.operating_point(speed,load,changed,propulsion_override=mean)
            assert encode(proxy)==encode(expected_proxy),(case,point['speed_kmh'],'frozen direction')
            timings={}
            for label,function in [('readable',reference.operating_point),('compiled',type(solver).operating_point)]:
                start=time.monotonic()
                for _ in range(5):function(solver,speed,load,x,phase_certificate=True)
                timings[label]=time.monotonic()-start
            rows.append(dict(case=case,speed_kmh=point['speed_kmh'],load_g=load,
                output_sha256=hashlib.sha256(text.encode()).hexdigest(),timings=timings))
        print(case,'exact phase and frozen-direction comparisons PASS',flush=True)
    report=dict(status='PASS',unchanged_equation_ast=True,cases=rows,
        note='Saved conditions are accuracy inputs only. These are microbenchmarks, not cold plot timings.')
    (base/'operating-compiled-validation.json').write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':main()
