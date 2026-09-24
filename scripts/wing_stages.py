"""Recovered wing area and coupled coefficient/application-point stages.

These use prepared coefficients in the game's stored frame, before later wing
force ordering/height corrections. No full flight-state evaluator is implied.
"""
from component_assembly import f32, add, sub, mul


def effective_areas(areas, health):
    """Each side: [inner, middle, outer, aileron]; aileron is already included."""
    return [add(add(mul(a[2],h[2]),add(mul(a[1],h[1]),mul(a[0],h[0]))),
                mul(add(h[3],-1.0),a[3])) for a,h in zip(areas,health)]


def coupled_points(cx, cy, cy_reference, cy_add, weights, dt, roll_rate,
                   area, span, positions, cm):
    """Port coefficient mixing and both points from 0x106c6021a..6049e.

    The paired reference Cy values come from additional polar evaluations.
    weights include dynamic pressure, effective area, arm and inertia terms.
    """
    delta = add(mul(sub(cy[0],cy_reference[0]),weights[0]),
                mul(sub(cy[1],cy_reference[1]),weights[1]))
    omega = f32(roll_rate)
    ratio = f32(mul(delta,dt)/-omega) if abs(omega)>f32(4e-19) else 0.0
    blend = min(1.0,max(0.0,add(ratio,-1.0)))
    corrected = [add(mul(sub(r,c),blend),add(c,d))
                 for c,r,d in zip(cy,cy_reference,cy_add)]
    chord = f32(area/span)
    points = []
    for x,y,p,(cm0,cm1) in zip(cx,corrected,positions,cm):
        denom=add(mul(y,y),mul(x,x))
        inv=f32(1/denom) if denom>f32(4e-19) else 0.0
        k=mul(inv,mul(add(mul(cm1,y),cm0),chord))
        points.append([sub(p[0],mul(y,k)),sub(p[1],mul(x,k)),p[2]])
    return dict(blend=blend,cy=corrected,points=points)
