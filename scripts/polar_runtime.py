"""Runtime polar preparation and independent float32 evaluation.

Mach cubic coefficients retain the original matrix construction and global-Mach
power basis. Preparation and evaluation operate without a game binary.
"""
import math
import struct
from component_assembly import f32, add, sub, mul
from control_mixer import prepare_rows, curve
from polar_model import FIELDS
from mach_cubic import all_coefficients

DATA=0x220000000; STACK=0x220010000; STOP=0x220020000
CONST_KEYS=['lineClCoeff','AfterCritParabAngle','AfterCritDeclineCoeff',
            'AfterCritMaxDistanceAngle','CxAfterCoeff','ClAfterCritLow',
            'ClAfterCritHigh','Cl0','alphaCritHigh','alphaCritLow',
            'ClCritHigh','ClCritLow','CdMin']




def pack_runtime(r,address=DATA):
    data=bytearray(0x1e0)
    struct.pack_into('<16f',data,0,*r['base'])
    struct.pack_into('<I',data,0x40,r['mode'])
    for i,row in enumerate(r['mach']):struct.pack_into('<9f',data,0x44+36*i,*row)
    data[0x140]=r['combined']
    start=address+0x160;end=start+16*len(r['cm'])
    if len(r['cm'])>8:raise ValueError('Too many Cm rows')
    struct.pack_into('<3Q',data,0x148,start,end,address+0x1e0)
    for i,(x,inv,v) in enumerate(r['cm']):struct.pack_into('<4f',data,0x160+16*i,x,inv,*v)
    return bytes(data)


def unpack_runtime(data,address=DATA):
    start,end,_=struct.unpack_from('<3Q',data,0x148)
    cm=[]
    for off in range(start-address,end-address,16):
        x,inv,a,b=struct.unpack_from('<4f',data,off);cm.append((x,inv,[a,b]))
    return dict(base=list(struct.unpack_from('<16f',data)),mode=struct.unpack_from('<I',data,0x40)[0],
                mach=[list(struct.unpack_from('<9f',data,0x44+36*i)) for i in range(7)],combined=bool(data[0x140]),cm=cm)


def make_runtime(props,span,area,machine=None):
    """Statically traced adapter with independently reconstructed cubic setup."""
    span,area=f32(span),f32(area)
    aspect=f32(mul(span,span)/area) if abs(area)>f32(4e-19) else 0.
    base=[mul(aspect,f32(props['OswaldsEfficiencyNumber']))]
    base.extend(f32(props[k]) for k in CONST_KEYS);base.extend([span,area])
    rows=[]
    for i in range(1,8):
        rows.append([f32(props[k+str(i)]) for k in ['MachCrit','MachMax','MultMachMax','MultLineCoeff','MultLimit']]+[0.]*4)
    cm=[]
    if 'ClToCmByMach' in props:
        cm=[(0.,props['ClToCmByMach'])]
    else:
        for i in range(8):
            if 'ClToCmByMach'+str(i) in props:
                x,*v=props['ClToCmByMach'+str(i)];cm.append((x,v))
    runtime=dict(base=base,mode=props.get('MachFactor',0),combined=bool(props.get('CombinedCl',True)),mach=rows,
                 cm=prepare_rows(cm or [(0,[0,0])]))
    for row,c in zip(rows,all_coefficients(runtime) if machine is None else machine.coefficients(runtime)):row[5:]=c
    return runtime




def interpolate(a,b,k,machine=None):
    """Property interpolation before Mach evaluation; retains a's Cm knot grid."""
    k=f32(k);choose=a if k<.5 else b
    lerp=lambda x,y:add(mul(sub(y,x),k),x)
    rows=[[lerp(x,y) for x,y in zip(ra[:5],rb[:5])]+[0.]*4 for ra,rb in zip(a['mach'],b['mach'])]
    cm=prepare_rows([(x,[lerp(u,v) for u,v in zip(values,curve(b['cm'],x,2))]) for x,_,values in a['cm']])
    out=dict(base=[lerp(x,y) for x,y in zip(a['base'],b['base'])],mode=choose['mode'],combined=choose['combined'],mach=rows,cm=cm)
    for row,c in zip(rows,all_coefficients(out) if machine is None else machine.coefficients(out)):row[5:]=c
    return out


def flap_polar(properties,flaps,machine=None):
    """Select/interpolate (flap knot,runtime polar) pairs as 0x1019e20d0.

    Nonnegative flap input; polar shape uses .75*sqrt(f)+.25*f, CdMin uses f.
    The empty-family default is deliberately unsupported by this selected-jet adapter.
    """
    if not properties:raise ValueError('A polar family is required')
    flaps=f32(flaps)
    if flaps<0:raise ValueError('Flap blending input must be nonnegative')
    shape=add(mul(f32(math.sqrt(flaps)),.75),mul(flaps,.25))
    def bracket(x):
        if x<=properties[0][0]:return properties[0],properties[0],0.
        for a,b in zip(properties,properties[1:]):
            if x<b[0]:return a,b,mul(sub(x,a[0]),f32(1/sub(b[0],a[0])))
        return properties[-1],properties[-1],0.
    a,b,k=bracket(shape)
    if a is b:
        import copy
        result=copy.deepcopy(a[1])
    else:result=interpolate(a[1],b[1],k,machine)
    a,b,k=bracket(flaps)
    cd=a[1]['base'][13]
    if a is not b:
        cd=add(mul(sub(b[1]['base'][13],cd),k),cd)
    result['base'][13]=cd
    return result


def mach_value(runtime,mach,index):
    a,b,high,slope,limit,c0,c1,c2,c3=runtime['mach'][index]
    if mach<a:return 0. if index==5 else 1.
    if mach>b:
        value=add(mul(sub(mach,b),slope),high)
        return min(value,limit) if slope>=0 else max(value,limit)
    return add(mul(add(mul(add(mul(c3,mach),c2),mach),c1),mach),c0)


def evaluate(runtime,mach,cy_mult=1.):
    """Float32 port of original Mach constructor, including stall preparation."""
    mach,cy_mult=f32(mach),f32(cy_mult);b=runtime['base']
    lam,slope,parab,decline,maxdist,cdafter,clafterl,clafterh,cl0,ah,al,ch,cl,cd,span,area=b
    ch,cl=mul(ch,cy_mult),mul(cl,cy_mult)
    ind=f32(f32(1/math.pi)/lam) if abs(mul(lam,f32(math.pi)))>f32(4e-19) else 0.
    focus,cm0,cm1,kq,clkq=0.,0.,0.,1.,1.
    if runtime['mode']==3:
        factors=[mach_value(runtime,mach,i) for i in range(7)]
        cd=mul(cd,factors[0])
        slope=mul(slope,sub(add(factors[0],1.),factors[1]) if runtime['combined'] else factors[1])
        original_cl0=cl0;cl0=mul(cl0,factors[6])
        ch=add(original_cl0,mul(sub(ch,cl0),factors[2]))
        cl=add(original_cl0,mul(sub(cl,cl0),factors[2]))
        ah,al=mul(ah,factors[3]),mul(al,factors[3])
        ind=mul(ind,factors[4]);focus=factors[5]
        cm0,cm1=curve(runtime['cm'],mach,2)
    elif runtime['mode'] in (1,2):
        mm=min(mach,f32(.9)) if runtime['mode']==1 else mach
        value=abs(sub(1.,mul(mm,mm)))
        if runtime['mode']==1:value=min(value,1.)
        kq=f32(1/max(f32(math.pow(value,f32(.3))),f32(.2)));clkq=f32(math.sqrt(kq))
    elif runtime['mode']!=0:raise ValueError('Unknown Mach mode')
    effective_slope=mul(slope,cy_mult)
    inv=f32(1/effective_slope) if abs(effective_slope)>f32(4e-19) else 0.
    dh=sub(ch,cl0);dl=sub(cl,cl0)
    lineh=sub(mul(add(inv,inv),dh),ah)
    low=mul(dl,inv);linel=sub(add(low,low),al)
    if linel>lineh:linel=lineh=mul(add(linel,lineh),.5)
    highden=mul(sub(ah,lineh),sub(ah,lineh));lowden=mul(sub(linel,al),sub(linel,al))
    ph=f32(sub(dh,mul(effective_slope,lineh))/highden) if highden>f32(4e-19) else 0.
    pl=f32(add(sub(cl0,cl),mul(effective_slope,linel))/lowden) if lowden>f32(4e-19) else 0.
    values=[cl0,cd,ind,slope,ch,cl,ah,al,lineh,linel,ph,pl,focus,cm0,cm1,parab,decline,maxdist,cdafter,clafterl,clafterh,kq,clkq,cy_mult]
    return dict(zip(FIELDS,values))
