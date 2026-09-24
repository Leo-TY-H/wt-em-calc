"""Check the float32 scalar polar ports against every native polar branch."""
import hashlib,json,random
from pathlib import Path
from macho_scan import MachO
from component_assembly import f32
from polar_model import make_polar,round_polar
from polar_f32 import calc_cl,calc_cd,calc_c
from verify_polar_machine_code import Kernel,EXPECTED_BINARY_SHA256


def main():
    binary=MachO()
    if hashlib.sha256(binary.data).hexdigest()!=EXPECTED_BINARY_SHA256:raise ValueError('Binary changed; remap instruction addresses first')
    kernel=Kernel(binary);rng=random.Random(16390311);failures=[];count=0
    for name in ['f_16a_block_15_adf','saab_jas39c']:
        fm=json.loads(Path('references/fm-2.59.0.13/'+name+'.blkx').read_text())
        for component in ['WingPlane','HorStabPlane','VerStabPlane','FuselagePlane']:
            plane=fm['Aerodynamics'][component];area=sum(plane['Areas'].values())
            props=plane.get('Polar',plane.get('FlapsPolar0'))
            for mach in [.2,.8,1.,1.8]:
                p=round_polar(make_polar(props,plane['Span'],area,mach,cy_mult=f32(rng.uniform(.5,1.5))))
                angles=[f32(rng.uniform(-180,180)) for _ in range(100)]
                angles.extend(p[k] for k in ['aoaCritH','aoaCritL','aoaLineH','aoaLineL','maxDistAng'])
                angles.extend([-180.,-140.,-90.,-40.,0.,40.,90.,140.,180.])
                for a in angles:
                    angle=f32(rng.uniform(-180,180));cladd=f32(rng.uniform(-.5,.5));cdscale=f32(rng.uniform(-2,2))
                    cases=[(0x10198c4d0,[a],[calc_cl(p,a)]),(0x10198c440,[a],[calc_cd(p,a)]),
                           (0x10198c320,[a,angle,cladd,cdscale],calc_c(p,a,angle,cladd,cdscale))]
                    for entry,args,expected in cases:
                        actual=kernel.call(entry,p,args);count+=1
                        if actual!=expected:failures.append(dict(aircraft=name,component=component,mach=mach,args=args,entry=hex(entry),actual=actual,expected=expected))
    report=dict(binary_sha256=EXPECTED_BINARY_SHA256,kernel_calls=count,failures=failures,
                comparison='Exact numerical float32 equality',limitations='Prepared runtime polar inputs. Host libm sin/sincos rounded to float32 replaces target library calls. NaN payloads and signed-zero bits are not compared.')
    Path('analysis/polar-f32-validation.json').write_text(json.dumps(report,indent=2))
    print('KERNEL CALLS',count,'FAILURES',len(failures));print(json.dumps(failures[:5],indent=2))
    if failures:raise SystemExit(1)


if __name__=='__main__':main()
