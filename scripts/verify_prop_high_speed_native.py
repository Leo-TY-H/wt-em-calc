"""Probe intact high-speed AEC states against the pinned original owner.

The binary argument is still checked by AircraftNative's original SHA guard.
No game installation or guard is changed. Thermal/fuel lifetime adapters are
the existing native harness boundaries; this is not a live-game replay.
"""
import argparse
import copy
import json
import math
from pathlib import Path

from em_backend import activate
activate()
from em_solver import TrimSolver, settings
from propulsion_general import step
from propulsion_general_native import PropulsionGeneralNative
from verify_propulsion_general import differences
import verify_aircraft_native
from macho_scan import MachO


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--binary',required=True)
    ap.add_argument('--output',default='analysis/em-global-september22/rpm-investigation/automatic-high-speed-native.json')
    args=ap.parse_args()
    verify_aircraft_native.MachO=lambda:MachO(args.binary)
    root=Path(__file__).resolve().parents[1]
    native=PropulsionGeneralNative()
    native.configure(json.loads((root/'references/jet-catalog/fm/ki_61_1a_otsu.blkx').read_text()))
    solver=TrimSolver('ki_61_1a_otsu_china',settings(dict(aircraft=['ki_61_1a_otsu_china'])))
    rows=[]
    for speed,load in [(806.2467,2.),(830.,2.),(849.9,2.)]:
        point=solver.solve(speed,load,exhaustive=False,detailed=True)
        assert point['valid'], point['reasons']
        assert point['propulsion']['rpm_threshold_exceeded_engines']
        detail=point.pop('_detail')
        prop=detail['propulsion']
        for automatic in [False,True]:
            native.u.mem_write(0x107d6fc10,bytes([automatic,1]))
            state=copy.deepcopy(prop['state']);rpms=[];pitches=[];failures=[]
            for frame in range(24):
                kw=dict(velocity=detail['velocity'],height=solver.config['altitude_m'],
                    body_omega=detail['geometry']['omega'],cg=solver.mass['cog'],
                    dt=solver.dt,seed=state['seed'],nitro=solver.mass['nitro_mass'],
                    torque_gyro=solver.config['torque_gyro'])
                actual=native.step(state,**kw)
                expected=step(solver.engine.properties,state,**kw)
                diff=differences(actual,expected)
                if diff:failures.append(dict(frame=frame,differences=diff))
                rpms.append([e['omega']*60/(2*math.pi) for e in actual['engines']])
                pitches.append([p['pitch']*180/math.pi for p in actual['propellers']])
                # Carry original outputs independently, retaining commands.
                state=dict(state,**{k:v for k,v in actual.items() if k not in ('engines','propellers')},
                    engines=[dict(s,**r) for s,r in zip(state['engines'],actual['engines'])],
                    propellers=[dict(s,**r) for s,r in zip(state['propellers'],actual['propellers'])])
            row=dict(speed_kmh=speed,load_g=load,automatic_management_global=automatic,
                converged=prop['converged'],point_reasons=point['reasons'],native_rpm=rpms,
                native_pitch_deg=pitches,frames=24,failures=failures)
            rows.append(row)
            print(speed,automatic,'rpm',rpms[-1],'pitch',pitches[-1],'differences',len(failures),flush=True)
    report=dict(binary_sha256=native.sha,cases=rows)
    Path(args.output).write_text(json.dumps(report,indent=2)+'\n')
    assert not any(row['failures'] for row in rows)


if __name__=='__main__':main()
