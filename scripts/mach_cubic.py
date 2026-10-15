from component_assembly import f32,add,mul
from decimal import Decimal,localcontext
from functools import lru_cache
from math import tanh,log


def native_inverse(m):
    """Column-major inverse with the native binary64 operation ordering.

    The input and output matrices are binary32. Do not regroup the minors or
    determinant: narrow Mach intervals amplify even binary64 rounding changes.
    Recovered from aces.exe matrix inverse 0x142ffbd30 (2026-09-29 build).
    """
    m0,m1,m2,m3,m4,m5,m6,m7,m8,m9,m10,m11,m12,m13,m14,m15=m
    d0=m15*m10-m14*m11
    d1=m15*m9-m13*m11
    d2=m14*m9-m13*m10
    d3=m15*m8-m12*m11
    d4=m14*m8-m12*m10
    d5=m13*m8-m12*m9
    d6=m10*m5-m9*m6
    d7=m9*m4-m8*m5
    d8=m10*m4-m8*m6
    d9=m11*m4-m8*m7
    d10=m11*m5-m9*m7
    d11=m11*m6-m10*m7
    d12=m14*m4-m12*m6
    d13=m14*m5-m13*m6
    d14=m13*m4-m12*m5
    d15=m5*m15-m13*m7
    d16=m4*m15-m12*m7
    d17=m15*m6-m14*m7
    adj=[
        (d2*m7+d0*m5)-d1*m6,
        d1*m2-(d0*m1+d2*m3),
        (d13*m3+d17*m1)-d15*m2,
        d10*m2-(d11*m1+d6*m3),
        d3*m6-(d0*m4+d4*m7),
        (d4*m3+m0*d0)-d3*m2,
        d16*m2-(d17*m0+d12*m3),
        (d8*m3+d11*m0)-d9*m2,
        (d5*m7+d1*m4)-d3*m5,
        d3*m1-(d1*m0+d5*m3),
        (d14*m3+d15*m0)-d16*m1,
        d9*m1-(d10*m0+d7*m3),
        d4*m5-(d2*m4+d5*m6),
        (d5*m2+d2*m0)-d4*m1,
        d12*m1-(d13*m0+d14*m2),
        (d7*m2+d6*m0)-d8*m1,
    ]
    determinant=(adj[4]*m1+adj[12]*m3)+(adj[8]*m2+adj[0]*m0)
    if abs(determinant)<f32(1e-15):return None
    inverse_determinant=1./determinant
    return [f32(v*inverse_determinant) for v in adj]


def coefficients(row,index):
    a,b,high,slope,_=map(f32,row[:5]);low=0. if index==5 else 1.
    columns=[[0.,0.,1.,1.],[1.,1.,a,b],
             [mul(a,2.),mul(b,2.),mul(a,a),mul(b,b)],
             [mul(mul(a,a),3.),mul(mul(b,b),3.),f32(a*a*a),f32(b*b*b)]]
    inverse=native_inverse([v for column in columns for v in column])
    if inverse is None:return [0.,0.,0.,low]
    return [add(add(mul(inverse[12+i],high),mul(inverse[4+i],slope)),mul(inverse[8+i],low))
            for i in range(4)]


def all_coefficients(runtime):return [coefficients(row,i) for i,row in enumerate(runtime['mach'])]


def hermite_value(row,mach,index):
    """Hermite transition in local coordinates, evaluated in binary64.

    This is the endpoint-constrained approximation, not native arithmetic.
    Retain the authored interval, zero entry slope, exit slope and outer clamp.
    An inconsistent authored exit clamp may still create a genuine branch jump.
    """
    a,b,high,slope,limit=row[:5]
    low=0. if index==5 else 1.
    if mach<a:return low
    if mach>b:
        value=high+(mach-b)*slope
        return min(value,limit) if slope>=0. else max(value,limit)
    if b<=a:return low
    t=(mach-a)/(b-a)
    # Avoid subtracting large global-Mach polynomial terms. The local cubic
    # has values of the same order as the endpoint values and scaled slope.
    return low+(high-low)*(t*t*(3.-2.*t))+(b-a)*slope*(t*t*(t-1.))


@lru_cache(maxsize=4096)
def native_local_coefficients(row):
    """Exactly re-express the stored native cubic about the interval midpoint.

    Decimal.from_float retains the *actual* binary32 coefficients and Mach
    endpoints. High-precision setup avoids losing their small residual/bias
    while translating the global polynomial. Runtime evaluation needs only
    binary64 Horner operations on u in [-1,1]. No endpoint refit is performed.
    """
    with localcontext() as context:
        context.prec=80
        a,b=map(Decimal.from_float,row[:2])
        c0,c1,c2,c3=map(Decimal.from_float,row[5:])
        center=(a+b)/2;radius=(b-a)/2
        if radius<=0:raise ValueError('Local native cubic needs an increasing Mach interval')
        return tuple(float(v) for v in (
            ((c3*center+c2)*center+c1)*center+c0,
            radius*((3*c3*center+2*c2)*center+c1),
            radius*radius*(3*c3*center+c2),
            radius*radius*radius*c3))


JOIN_FRACTION=.20


def stable_value(row,mach,index,local=None):
    """Declared native pieces evaluated in binary64, without refitting joins.

    Stored coefficients and comparison/equality ownership are unchanged. The
    caller rounds the returned multiplier once to binary32. Degenerate native
    intervals retain their guarded global polynomial at any owned equality.
    """
    a,b,high,slope,limit,c0,c1,c2,c3=row
    if mach<a:return 0. if index==5 else 1.
    if mach>b:
        value=high+(mach-b)*slope
        return min(value,limit) if slope>=0. else max(value,limit)
    if b<=a:return ((c3*mach+c2)*mach+c1)*mach+c0
    if local is None:local=native_local_coefficients(tuple(row))
    radius=(b-a)*.5
    u=(mach-(a+radius))/radius
    c0,c1,c2,c3=local
    return ((c3*u+c2)*u+c1)*u+c0


def join_scale(row):
    """Tanh scale; JOIN_FRACTION specifies the 10%-to-90% transition width."""
    return (row[1]-row[0])*JOIN_FRACTION/log(9.)


def approximation_knots(row):
    """Sample both sides and the center of each tanh transition."""
    a,b=row[:2];scale=join_scale(row)
    return tuple(sorted({(a+b)*.5,*[x+k*scale for x in (a,b) for k in (-3.,-1.,0.,1.,3.)]}))


def continuous_value(row,mach,index,local=None):
    """Stable native cubic blended by tanh functions centered at native jumps.

    Only ill-conditioned rows use this evaluator. Unlike the previous one-sided
    joins, the transition straddles each endpoint and is half complete there.
    The stored native polynomial is unchanged; corrections decay exponentially
    away from the jumps. Native exterior arithmetic resumes at saturated weights.
    """
    a,b,high,slope,limit=row[:5]
    low=0. if index==5 else 1.
    scale=join_scale(row)
    entry=.5*(1.+tanh((mach-a)/scale))
    exit=.5*(1.+tanh((mach-b)/scale))
    if entry==0.:return low
    if exit==1.:
        outer=add(mul(f32(mach-b),slope),high)
        return min(outer,limit) if slope>=0. else max(outer,limit)
    if local is None:local=native_local_coefficients(tuple(row))
    radius=(b-a)*.5
    u=(mach-(a+radius))/radius
    c0,c1,c2,c3=local
    value=((c3*u+c2)*u+c1)*u+c0
    endpoint=min(high,limit) if slope>=0. else max(high,limit)
    tangent=slope if (high<limit if slope>=0. else high>limit) else 0.
    if mach<=b:outer=endpoint+(mach-b)*tangent
    else:
        outer=high+(mach-b)*slope
        outer=min(outer,limit) if slope>=0. else max(outer,limit)
    joined=low+entry*(value-low)
    return joined+exit*(outer-joined)


def needs_continuous_approximation(row,index):
    a,b,high,slope,limit,*c=row
    if not 0.<a<b:return False
    scale=max(1.,abs(high),abs((b-a)*slope))
    # Cancellation (lost binary32 significance) is the only eligibility test.
    # Endpoint mismatches and deliberate, well-conditioned jumps are not enough.
    return sum(abs(v)*b**i for i,v in enumerate(c))>16384.*scale
