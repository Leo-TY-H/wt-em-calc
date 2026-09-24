"""Conditional engine-kernel samples, not an aircraft performance solver."""
import json
from pathlib import Path
import piston_model as piston
from component_assembly import f32, mul

ROOT=Path(__file__).resolve().parents[1]


def main():
    rows=[]
    for name in ['yak-3','bf-109f-4']:
        model=piston.prepare(json.loads((ROOT/'references/fm-2.59.0.13'/(name+'.blkx')).read_text()))
        for boost in ([False] if name=='yak-3' else [False,True]):
            rpm=2700 if name=='yak-3' or boost else 2500
            w=mul(f32(rpm),piston.RAD_PER_RPM);throttle=f32(1.1 if boost else 1.)
            for velocity in [0.,100.,180.]:
                for height in [0.,1000.,2000.,3000.,5000.,7000.,9000.]:
                    inlet=piston.inlet_pressure(height,velocity,model['ram_recovery'])
                    comp=piston.compressor(model,w,throttle,inlet,1/48,gear=None,regulator=-1,afterburner=boost)
                    torque=mul(piston.rpm_torque(model,w,throttle,1.,boost,comp['gear']),comp['multiplier'])
                    interval=([piston.div(mul(inlet,.25),model['mixer_scale']),piston.div(inlet,model['mixer_scale'])]
                              if model['mixer_type']==2 else None)
                    rows.append(dict(aircraft=name,afterburner=boost,rpm=rpm,height_m=height,body_u_mps=velocity,
                                     gear=comp['gear'],manifold=comp['manifold'],
                                     nominal_engine_diagnostic_hp=piston.div(mul(torque,w),746.),
                                     unpenalized_mixture_interval_raw=interval))
    scope=('Conditional engine-kernel diagnostics, not propeller thrust or aircraft performance. '
           'Prescribed RPM at conventional full-power governor targets, reset-to-equilibrium compressor, '
           'mixture multiplier=1, caller torque multiplier=1 and healthy mechanical multiplier=1. '
           'No assertion of drivetrain equilibrium or globally best RPM. Native horsepower conversion is 746 W. '
           'Mixture interval describes the numerical consumer: intersect with permitted delivered commands; '
           'it is not a UI percentage recommendation.')
    (ROOT/'analysis/piston-nominal-diagnostics.json').write_text(json.dumps(dict(scope=scope,rows=rows),indent=2)+'\n')
    print('Saved',len(rows),'conditional engine samples')


if __name__=='__main__':main()
