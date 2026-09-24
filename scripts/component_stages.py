"""Scalar ports of additional instruction slices; inputs use raw internal xyz.

Prepared coefficients already include Mach, controls, coefficient rotation, etc.
Dynamic pressures/areas below are explicit inputs, not computed from freestream
speed automatically. No claim of a complete flight update.
"""
from component_assembly import f32,add,sub,mul

def local_flow(position,cog,velocity,omega,disturbance):
    r=[sub(f32(position[i]),f32(cog[i])) for i in range(3)]
    v,w,d=([f32(x) for x in z] for z in [velocity,omega,disturbance])
    return [add(add(sub(d[0],mul(w[1],r[2])),mul(w[2],r[1])),v[0]),
            add(sub(add(mul(w[0],r[2]),d[1]),mul(w[2],r[0])),v[1]),
            add(sub(d[2],mul(w[0],r[1])),add(mul(w[1],r[0]),v[2]))]

def horizontal_application_point(base,cx,cy,cd_add,cm0,cm1):
    cx=add(f32(cx),f32(cd_add));cy=f32(cy)
    b=add(mul(f32(cm1),cy),f32(cm0))
    den=add(mul(cy,cy),mul(cx,cx))
    inv=f32(1/den) if den>f32(4e-19) else 0.0
    k=mul(inv,b)
    return [sub(f32(base[0]),mul(cy,k)),sub(f32(base[1]),mul(cx,k)),f32(base[2])]

def secondary_forces(coeff,q_tail,q_fuse,areas,health,vertical_area_add,vertical_area_scale,vertical_lift_scale=1.0):
    """Recovered final scaling of hstab halves, vertical surface, and fuselage.

    `areas` holds runtime values, not raw configuration Areas assumed equivalent.
    `q_tail` is shared by hstab and vstab in the detailed path (see report).
    Vertical area scale is 1 when flag +0x817c is set, otherwise its prepared value.
    """
    c={n:[f32(x) for x in v] for n,v in coeff.items()}
    a={n:f32(v) for n,v in areas.items()};h={n:f32(v) for n,v in health.items()}
    qt,qf=f32(q_tail),f32(q_fuse)
    al=add(mul(h['left_main'],a['left_main']),mul(a['left_elevator'],h['left_elevator']))
    ar=add(mul(h['right_elevator'],a['right_elevator']),mul(a['right_main'],h['right_main']))
    av=mul(add(mul(h['rudder'],a['rudder']),add(f32(vertical_area_add),mul(mul(h['v_main'],1.5),a['v_main']))),f32(vertical_area_scale))
    fq=mul(mul(-qf,h['fuselage']),a['fuselage'])
    vs=f32(float(f32(vertical_lift_scale))*float(qt)*float(c['vstab'][1]))
    return {'left_hstab':[mul(mul(-qt,al),c['left_hstab'][0]),mul(mul(c['left_hstab'][1],qt),al),0.0],
            'right_hstab':[mul(mul(-qt,ar),c['right_hstab'][0]),mul(mul(c['right_hstab'][1],qt),ar),0.0],
            'vstab':[mul(mul(c['vstab'][0],-qt),av),0.0,mul(av,vs)],
            'fuselage':[mul(c['fuselage'][0],fq),0.0,mul(mul(c['fuselage'][1],qf),mul(h['fuselage'],a['fuselage']))]}


def runtime_secondary_areas(fm):
    """Direct assignments recovered from 0x101a3adbb..0x101a3ae3a."""
    a=fm['Aerodynamics'];h=a['HorStabPlane']['Areas'];v=a['VerStabPlane']['Areas']
    return dict(fuselage=f32(a['FuselagePlane']['Areas']['Main']),
                left_main=mul(f32(h['Main']),.5),left_elevator=mul(f32(h['Elevator']),.5),
                right_main=mul(f32(h['Main']),.5),right_elevator=mul(f32(h['Elevator']),.5),
                v_main=f32(v['Main']),rudder=f32(v['Rudder']))
