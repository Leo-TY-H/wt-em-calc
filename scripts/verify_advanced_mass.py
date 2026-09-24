"""Original complete AdvancedMass wrapper/consumer on prepared intact records."""
import json,random
from pathlib import Path
from component_assembly import f32
from advanced_mass import evaluate
from verify_mass_model import MassMachine

def main():
    m=MassMachine();r=random.Random(193300);failures=[];count=0
    def val(a,b):return f32(r.uniform(a,b))
    for i in range(2000):
        p=dict(advanced_mass=True,empty=val(2000,3000),oil=val(0,100),crew_mass=90.,configured_cog=[val(-2,2) for _ in range(3)],normalized_inertia=[val(1,10) for _ in range(3)],payload_affects_cog=bool(i%2),cog_y_limits=[-1.,1.])
        fuel=[val(0,100) for _ in range(i%12)];parts=[dict(mass=val(0,500),position=[val(-7,7) for _ in range(3)],specific_inertia=[val(0,4) for _ in range(3)],fuel_tank=j if j<len(fuel) else None) for j in range(i%12+3)]
        kw=dict(mass_parts=parts,fuel_by_tank=fuel,payloads=[dict(mass=val(0,100),position=[val(-4,4) for _ in range(3)]) for _ in range(i%9)],nitro=val(0,20),payload_cg_scale=val(0,2),payload_inertia_scale=val(0,2),inertia_modifiers=[val(.5,2) for _ in range(3)])
        a=m.evaluate(p,fuel,**kw);e=evaluate(p,fuel,**kw);count+=1
        d={k:dict(actual=v,expected=e[k]) for k,v in a.items() if v!=e[k]}
        if d:failures.append(dict(case=i,diff=d));break
    report=dict(status='FAIL' if failures else 'PASS',binary_sha256=m.sha,cases=count,failures=failures,scope='Complete original101998ad0/101998bf0 intact AdvancedMass=true return. Prepared component mass, specific inertia, position and fuel-tank links, payload records and frozen fuel. Bf109F4 enables this branch. No actual collision-mesh-to-mass-part producer or damaged/missing parts; independent math must not be mistaken for captured Bf CG/inertia.')
    Path('analysis/advanced-mass-validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
    if failures:raise SystemExit(1)
if __name__=='__main__':main()
