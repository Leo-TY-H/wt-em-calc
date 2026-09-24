"""Independent polynomial setup: float matrix, double adjugate, float output.

The four constraints are P'(a)=0, P'(b)=s, P(a)=low, P(b)=high.
The game constructs a global-Mach power basis, NOT normalized Hermite values.
Its matrix entries are rounded before inversion, including double-computed M³.
"""
from component_assembly import f32,add,mul


def det3(m):
    a,b,c=m
    return (a[0]*(b[1]*c[2]-b[2]*c[1])-a[1]*(b[0]*c[2]-b[2]*c[0]))+a[2]*(b[0]*c[1]-b[1]*c[0])


def coefficients(row,index):
    a,b,high,slope,_=map(f32,row[:5]);low=0. if index==5 else 1.
    columns=[[0.,0.,1.,1.],[1.,1.,a,b],
             [mul(a,2.),mul(b,2.),mul(a,a),mul(b,b)],
             [mul(mul(a,a),3.),mul(mul(b,b),3.),f32(a*a*a),f32(b*b*b)]]
    matrix=list(map(list,zip(*columns)))
    cofactors=[[(-1. if (i+j)%2 else 1.)*det3([[v for jj,v in enumerate(r) if jj!=j]
                    for ii,r in enumerate(matrix) if ii!=i]) for j in range(4)] for i in range(4)]
    determinant=sum(matrix[0][j]*cofactors[0][j] for j in range(4))
    if abs(determinant)<f32(1e-15):return [0.,0.,0.,low]
    # The original inverse scales its double cofactors before rounding to float.
    inverse=[[f32(cofactors[j][i]*(1./determinant)) for j in range(4)] for i in range(4)]
    return [add(add(mul(r[3],high),mul(r[1],slope)),mul(r[2],low)) for r in inverse]


def all_coefficients(runtime):return [coefficients(row,i) for i,row in enumerate(runtime['mach'])]
