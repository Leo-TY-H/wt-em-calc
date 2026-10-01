import struct
from rust_backend import load as _load_rust, assembly as _rust_assembly

_rust = _load_rust(__file__)

def f32(x):return struct.unpack('<f',struct.pack('<f',x))[0]
def add(a,b):return f32(a+b)
def sub(a,b):return f32(a-b)
def mul(a,b):return f32(a*b)

NAMES=('left_wing','right_wing','left_hstab','right_hstab','vstab','fuselage','chute')

def assemble_force(forces):
    l,r,h,j,v,b,c,p=([f32(x) for x in forces[n]] for n in (*NAMES,'parasite'))
    x=add(add(add(v[0],add(j[0],add(h[0],add(add(l[0],c[0]),r[0])))),b[0]),p[0])
    y=add(add(add(add(add(add(l[1],c[1]),r[1]),j[1]),add(v[1],h[1])),p[1]),b[1])
    z=add(add(add(b[2],add(p[2],add(r[2],l[2]))),c[2]),v[2])
    return [x,y,z]

def assemble_moment(forces,positions,cog):
    f={n:[f32(x) for x in forces[n]] for n in NAMES}
    r={n:[sub(f32(positions[n][i]),f32(cog[i])) for i in range(3)] for n in NAMES}
    def term(n,fi,ri):return mul(f[n][fi],r[n][ri])
    def paired(fi,ri):
        wings=add(term('right_wing',fi,ri),term('left_wing',fi,ri))
        tails=add(term('left_hstab',fi,ri),term('right_hstab',fi,ri))
        main=add(wings,tails)
        extra=add(term('fuselage',fi,ri),term('vstab',fi,ri))
        return add(add(main,extra),term('chute',fi,ri))

    pos_w=add(term('right_wing',1,2),term('left_wing',1,2))
    pos_h=add(term('right_hstab',1,2),term('left_hstab',1,2))
    pos=add(add(term('chute',1,2),add(term('fuselage',1,2),term('vstab',1,2))),add(pos_h,pos_w))
    neg_w=add(term('right_wing',2,1),term('left_wing',2,1))
    neg_h=add(term('right_hstab',2,1),term('left_hstab',2,1))
    neg=add(term('chute',2,1),add(add(term('fuselage',2,1),term('vstab',2,1)),add(neg_h,neg_w)))
    return [sub(pos,neg),sub(paired(2,0),paired(0,2)),sub(paired(0,1),paired(1,0))]


def limit_aerodynamic_force(force,mass,y_scale=1.0,allow_balance_change=True):
    import math
    x,y,z=force
    if not all(math.isfinite(v) for v in force):raise ValueError('Only finite inputs reconstructed here')
    y*=1.0 if allow_balance_change else f32(y_scale)
    limit=mul(mul(f32(9.81),f32(mass)),50.0)
    magnitude=math.sqrt((z*z+x*x)+y*y)
    if magnitude>limit:
        scale=limit/magnitude
        return [x*scale,y*scale,z*scale]
    return [x,y,z]


# Bind complete calls directly; the pure bodies remain reference fallbacks.
_reference_assemble_force=assemble_force
_reference_assemble_moment=assemble_moment
if _rust is not None:
    if _rust._python:
        from rust_backend import bind_native
        globals()['assemble_force']=bind_native(_rust,_reference_assemble_force,0)
        globals()['assemble_moment']=bind_native(_rust,_reference_assemble_moment,1)
    else:
        def _portable_force(forces):
            result=_rust_assembly(_rust,(x for n in (*NAMES,'parasite') for x in forces[n]))
            return _reference_assemble_force(forces) if result is None else result
        def _portable_moment(forces,positions,cog):
            values=[x for n in NAMES for x in forces[n]]
            values.extend(positions[n][i] for n in NAMES for i in range(3))
            values.extend(cog)
            result=_rust_assembly(_rust,values,moment=True)
            return _reference_assemble_moment(forces,positions,cog) if result is None else result
        globals()['assemble_force']=_portable_force
        globals()['assemble_moment']=_portable_moment
