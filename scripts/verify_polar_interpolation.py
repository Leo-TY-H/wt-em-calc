"""Differentially check original property blending and nonlinear flap selection."""
import json,random
from pathlib import Path
from component_assembly import f32
from polar_runtime import PolarMachine,interpolate,flap_polar


def same(a,b):
    return all(a[k]==b[k] for k in ['base','mode','combined','mach']) and all(x==y and inv==inv2 and v==v2 for (x,inv,v),(y,inv2,v2) in zip(a['cm'],b['cm'])) and len(a['cm'])==len(b['cm'])


def main():
    machine=PolarMachine();rng=random.Random(166390);counts=dict(interpolation=0,flap_selector=0);failures=[]
    saved=json.loads(Path('analysis/prepared-polar-properties.json').read_text())['aircraft']
    for name,r in saved.items():
        a,b=[r['WingPlane/FlapsPolar'+str(i)] for i in range(2)]
        for i in range(200):
            k=f32([0.,.5,1.][i] if i<3 else rng.uniform(0,1))
            actual=machine.interpolate(a,b,k);expected=interpolate(a,b,k);counts['interpolation']+=1
            if not same(actual,expected):failures.append(dict(stage='interpolation',aircraft=name,k=k,actual=actual,expected=expected))
            family=[(0.,a),(1.,b)] if i<100 else [(0.,a),(f32(.37),b),(f32(.91),a)]
            f=f32([0.,1.,1.2][i] if i<3 else rng.uniform(0,1.2))
            actual=machine.flap(family,f);expected=flap_polar(family,f);counts['flap_selector']+=1
            if not same(actual,expected):failures.append(dict(stage='flap',aircraft=name,f=f,actual=actual,expected=expected))
    report=dict(binary_sha256=machine.sha,calls=counts,failures=failures,
                limitations='All original arithmetic and branches run unchanged. memcpy/memmove use byte-copy hooks. The independent port includes its own double adjugate and float cubic setup. Covers both selected aircraft plus synthetic three-knot flap families.')
    Path('analysis/polar-interpolation-validation.json').write_text(json.dumps(report,indent=2));print('CALLS',counts,'FAILURES',len(failures));print(json.dumps(failures[:1],indent=2))
    if failures:raise SystemExit(1)


if __name__=='__main__':main()
