"""Check complete Mach/stall preparation using actual aircraft runtime properties."""
import json,random
from pathlib import Path
from component_assembly import f32
from polar_runtime import PolarMachine,make_runtime,evaluate
from polar_model import make_polar
from mach_cubic import all_coefficients


def main():
    machine=PolarMachine();rng=random.Random(980016);failures=[];count=0;setup=0;old_error=0.;runtimes={}
    for name in ['f_16a_block_15_adf','saab_jas39c']:
        fm=json.loads(Path(f'references/fm-2.59.0.13/{name}.blkx').read_text());runtimes[name]={}
        for component in ['WingPlane','HorStabPlane','VerStabPlane','FuselagePlane']:
            plane=fm['Aerodynamics'][component]
            area=sum(v for k,v in plane['Areas'].items() if k!='Aileron')
            for key,props in plane.items():
                if not(key=='Polar' or key.startswith('FlapsPolar')):continue
                runtime=make_runtime(props,plane['Span'],area);setup+=1
                if machine.coefficients(runtime)!=all_coefficients(runtime):
                    failures.append(dict(stage='cubic_setup',aircraft=name,component=component,polar=key))
                runtimes[name][component+'/'+key]=runtime
                knots=[f32(x) for row in runtime['mach'] for x in row[:2]]
                machs=[0.,.3,.6,.8,.9,1.,1.1,1.4,1.8,2.2]+knots+[f32(rng.uniform(0,2.5)) for _ in range(40)]
                for mach in machs:
                    for cm in [0.,.5,1.,1.5]:
                        actual=machine.evaluate(runtime,mach,cm);expected=evaluate(runtime,mach,cm);count+=1
                        differences={k:[actual[k],expected[k]] for k in actual if actual[k]!=expected[k]}
                        if differences:failures.append(dict(aircraft=name,component=component,polar=key,mach=mach,cy_mult=cm,fields=differences))
                    old=make_polar(props,plane['Span'],area,mach)
                    actual=machine.evaluate(runtime,mach)
                    old_error=max(old_error,max(abs(actual[k]-old[k]) for k in actual))
    report=dict(binary_sha256=machine.sha,coefficient_setup_calls=setup,mach_constructor_calls=count,
                failures=failures,comparison='Exact numerical float equality for all 24 output fields',
                earlier_normalized_hermite_max_field_difference=old_error,
                limitations='Independent adjugate cubic setup and Mach evaluation compared with original setup/inverse/evaluator. Datamine-to-property layout is statically traced, not original BLK loader execution. Evaluation covers configured advanced Mach mode; no whole timestep claim.')
    Path('analysis/polar-runtime-validation.json').write_text(json.dumps(report,indent=2))
    Path('analysis/prepared-polar-properties.json').write_text(json.dumps(dict(binary_sha256=machine.sha,aircraft=runtimes),indent=2))
    print(json.dumps({k:v for k,v in report.items() if k!='failures'},indent=2));print('FAILURES',len(failures));print(json.dumps(failures[:3],indent=2))
    if failures:raise SystemExit(1)


if __name__=='__main__':main()
