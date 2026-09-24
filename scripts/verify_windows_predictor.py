"""Compare complete Windows reduced predictors with the readable port.

Reports float differences explicitly; Windows and Mach-O code generation can
differ in operation grouping. Prepared intact sources, not a live boundary.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path

from windows_instructor_native import WindowsInstructorNative
from instructor_native import BASE, OWNER
from aircraft_catalog import catalog, load
from aircraft_model import prepare, at_sweep
from mass_model import aircraft_properties, evaluate
from jet_catalog import fuel_capacities
from prop_catalog import mass_state
from verify_instructor_keyboard import extract_state
from instructor_keyboard import fixed_source
from instructor_predictor_inputs import pack_pitch_inputs
from instructor_pitch_predictor import pitch_predictor, unpack_inputs
from instructor_autotrim import autotrim_predictor


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--binary', type=Path, required=True)
    p.add_argument('--aircraft', default='f_16xl,j6k1,go229_v3,il-10,fw_200c_1,saab_jas39c,f_14a_early')
    p.add_argument('--all-aircraft', action='store_true')
    p.add_argument('--out', type=Path, default=Path('analysis/windows-instructor/windows-predictor-validation.json'))
    args = p.parse_args()
    n = WindowsInstructorNative(args.binary)
    names = [k for k, v in catalog().items() if v['supported']] if args.all_aircraft else args.aircraft.split(',')
    counts = Counter(); rows = []; errors = []; examples = []; adapters = set()
    for name in names:
        fm = load(name)
        mass = (evaluate(aircraft_properties(fm), [v*.3 for v in fuel_capacities(fm)])
                if catalog()[name]['propulsion']=='jet' else mass_state(name, 30.))
        for case in range(4):
            sweep = [0., .4, .7, 1.][case] if catalog()[name]['has_sweep'] else 0.
            model = at_sweep(prepare(fm), sweep)
            n.setup(model, mass, speed=[100.,200.,300.,250.][case], alpha=[4.,10.,18.,-8.][case],
                    flaps=[0.,.3,1.,0.][case], sweep=sweep, height=[0.,1500.,6000.,9000.][case], mode_lane=True)
            n.floats(OWNER+0x25b58, [35. if case%2 else 0., 12. if case%2 else 0.])
            source = extract_state(n, model, 0.)
            source['predictor']['f'].update({o:n.read(BASE+o, 1)[0] for o in (0x5328,0x79b8,0x79bc)})
            fixed = fixed_source(model, source)
            inputs = [(0, fixed['auto_inputs'])]
            for angle, accel in [(-12., -.4), (25., .7)]:
                packed = pack_pitch_inputs(**source['wrapper'], target_angle=angle, target_acceleration=accel,
                    body_pitch_rate=source['pitch_rate'], angle_bounds=fixed['tail_bounds'], axis_weights=[0.,0.,0.],
                    quaternion=source['quaternion'], world_velocity=source['world_velocity'])
                inputs.append((1, packed))
            for mode, packed in inputs:
                history = [0., 0., False]
                trace = []
                expected = (pitch_predictor if mode else autotrim_predictor)(model, unpack_inputs(packed), source['predictor'], history, trace)
                try:
                    actual = n.predict(packed, history)
                except Exception as error:
                    errors.append(dict(aircraft=name,case=case,mode=mode,error=repr(error)))
                    continue
                counts['mode_'+str(mode)] += 1
                counts['exact' if actual==expected else 'different'] += 1
                if not mode: counts['mode_0_port' if trace else 'mode_0_machine_backend'] += 1
                diff = [abs(a-b) for a,b in zip(actual['output'], expected['output'])]
                history_diff = [abs(float(a)-float(b)) for a,b in zip(actual['history'],expected['history'])]
                record = dict(aircraft=name, case=case, mode=mode, success_equal=actual['success']==expected['success'],
                    max_command_difference=max(diff[1:3]), max_angle_difference=diff[0],
                    max_force_relative_difference=max(diff[i]/max(1.,abs(actual['output'][i]),abs(expected['output'][i])) for i in range(3,13)),
                    history_angle_difference=history_diff[0], history_force_difference=history_diff[1],
                    history_written_equal=actual['history'][2]==expected['history'][2],
                    finite=all(math.isfinite(x) for x in actual['output']+actual['history']))
                rows.append(record)
                if actual!=expected and (len(examples)<12 or not record['success_equal']):
                    examples.append(dict(**record, actual=actual, expected=expected))
                adapters.update(n.windows_calls)
        print(name, dict(counts), 'execution errors', len(errors), flush=True)
    version = json.loads(Path('references/data-version.json').read_text(encoding='utf-8'))
    decisions_equal = all(r['success_equal'] and r['history_written_equal'] for r in rows)
    summary = dict(success_differences=sum(not r['success_equal'] for r in rows),
        history_write_differences=sum(not r['history_written_equal'] for r in rows),
        max_command_difference=max((r['max_command_difference'] for r in rows),default=0.),
        max_angle_difference=max((r['max_angle_difference'] for r in rows),default=0.))
    report = dict(windows_sha256=n.windows.sha256,fixture_binary_sha256=n.sha,
        data_version=version['version'],data_commit=version['commit'],summary=summary,
        counts=dict(counts),expected_calls=len(names)*12,aircraft_count=len(names),errors=errors,rows=rows,examples=examples,
        adapters=sorted(adapters),scope=__doc__,global_boundary_validated=False)
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print('report',args.out,flush=True)
    if errors or not decisions_equal or len(rows)!=len(names)*12 or any(not r['finite'] for r in rows):raise SystemExit(1)


if __name__=='__main__':main()
