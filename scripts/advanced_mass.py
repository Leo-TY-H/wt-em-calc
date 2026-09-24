"""Intact AdvancedMass branch101998bf0 with prepared mesh/part records.

The part producer needs collision mesh geometry; FM mass fractions alone do
not supply positions or dimensions. mass_parts_native.py now executes that
producer on the installed Bf geometry (verify_mesh_mass.py); this independent
module remains the consumer. Never substitute legacy MomentOfInertia while
AdvancedMass is enabled.
"""
from component_assembly import f32,add,mul,sub
from mass_model import fuel_sum,payload_sums

def evaluate(properties,fuel_by_system,mass_parts,payloads=(),fuel_by_tank=(),nitro=0.,payload_cg_scale=None,payload_inertia_scale=None,inertia_modifiers=(1.,1.,1.)):
    fuel=fuel_sum(fuel_by_system);pm,pf,pi=payload_sums(payloads)
    empty,oil,crew,nitro=map(f32,[properties['empty'],properties['oil'],properties['crew_mass'],nitro])
    core=add(add(add(add(oil,empty),crew),nitro),fuel);mass=add(core,pm)
    tanks=list(map(f32,fuel_by_tank));first=list(pf);parts=[]
    for part in mass_parts:
        i=part.get('fuel_tank');m=add(part['mass'],tanks[i] if i is not None and i<len(tanks) else 0.)
        point=list(map(f32,part['position']));parts.append((part,m,point))
        first=[add(a,mul(m,b)) for a,b in zip(first,point)]
    inv=f32(1./mass) if abs(mass)>f32(4e-19) else 0.;cg=[mul(a,inv) for a in first];inertia=list(pi)
    for part,m,point in parts:
        x,y,z=[mul(sub(a,b),sub(a,b)) for a,b in zip(point,cg)];ix,iy,iz=map(f32,part.get('specific_inertia',[0.,0.,0.]))
        terms=[mul(m,add(z,add(ix,y))),mul(m,add(iy,add(x,z))),mul(add(add(x,y),iz),m)]
        inertia=[a+b for a,b in zip(inertia,terms)]
    lo,hi=properties.get('cog_y_limits',[-2147440000.,2147440000.]);cg[1]=min(max(cg[1],f32(lo)),f32(hi))
    return dict(mass=add(add(add(add(add(fuel,empty),oil),crew),nitro),pm),consumer_mass=mass,cog=cg,inertia=inertia,payload_mass=pm,payload_inertia=pi,fuel_mass=fuel)
