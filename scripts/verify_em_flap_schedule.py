"""Check the prescribed-flap EM assumption through full aerodynamic consumers.

This tests a user-selected modeling condition, not a native actuator rule.
The game's actuator and aerodynamic kernels retain their separate native tests.
No chart or interpolated surface is produced here.
"""
import argparse
import json
from pathlib import Path
from em_solver import TrimSolver,ROOT,BACKEND
from component_assembly import f32
from aircraft_model import evaluate
from verify_em_physics_sources import verify


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--report',required=True);args=ap.parse_args()
    rows=[]
    # Cover automatic retraction, a swept wing, unavailable flap controls and
    # conventional trailing-edge flaps. Include speeds above deployment limits.
    for name in ['f_16xl','saab_jas39c','f_14b','f_86em_greece']:
        for percentage in [0.,30.,100.]:
            s=TrimSolver(name,dict(aircraft=[name],instructor=False,flaps_percent=percentage,sweep_percent=50. if name=='f_14b' else 0.))
            for speed in [80.,180.,350.]:
                v=s.operating_point(speed,3.,[2.,70.,0.,.05,0.])
                assert v['flaps']==f32(percentage/100.),(name,speed,percentage,v['flaps'])
                # Re-evaluate the physical consumer independently of the EM
                # selection path, prescribing exactly the requested deployment.
                expected=evaluate(s.model,v['velocity'],v['geometry']['omega'].tolist(),s.mass,v['allocation']['commands'],
                    s.config['altitude_m'],s.dt,v['result']['history'],oil_radiator=0.,flaps=f32(percentage/100.),gear=s.gear,
                    throttle=s.config['throttle'],height_agl=1e6,engine_vectors=s.engine.vectors(v['velocity'][0],f32(percentage/100.)),
                    engine_spin_factor=f32(s.config['throttle']),quaternion=v['geometry']['quaternion'],torque_gyro=False)
                assert expected['component_forces']==v['result']['component_forces'],(name,speed,percentage)
                rows.append(dict(aircraft=name,speed_mps=speed,flaps_percent=percentage,force_n=v['result']['force']))
    # The Instructor on/off setting must not change the prescribed devices.
    s=TrimSolver('f_16xl',dict(aircraft=['f_16xl'],instructor=True,flaps_percent=100.))
    v=s.operating_point(350.,3.,[2.,70.,0.,.05,0.])
    assert v['flaps']==1.
    report=dict(status='PASS',backend=BACKEND,scope=__doc__,operating_points=len(rows)+1,rows=rows,physics_sources=verify())
    Path(args.report).write_text(json.dumps(report,indent=2)+'\n')
    print('PASS:',len(rows)+1,'full operating points; unchanged protected kernels')


if __name__=='__main__':main()
