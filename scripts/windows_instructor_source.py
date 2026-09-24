"""Independent Windows rounding for the prepared Instructor wing polar.

143149ebd..143149eca computes (critical - (slope*line + offset))/denominator.
The pinned Mac constructor computes ((critical-offset) - slope*line)/denominator.
They are algebraically equivalent but can differ after float32 rounding.
This supplies source calculations, never captured native intermediates.
"""
from component_assembly import f32,add,sub,mul
from instructor_keyboard import fixed_source as portable_source


def fixed_source(model,state):
    result=portable_source(model,state)
    p=dict(result['polar'])
    distance=sub(p['aoaCritH'],p['aoaLineH']);denominator=mul(distance,distance)
    slope=mul(p['clLineCoeff'],p['cyMult'])
    numerator=sub(p['cyCritH'],add(mul(p['aoaLineH'],slope),p['cl0']))
    p['parabCyCoeffH']=f32(numerator/denominator) if denominator>f32(4e-19) else 0.
    return dict(result,polar=p)
