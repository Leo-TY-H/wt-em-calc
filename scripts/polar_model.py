"""Scalar reconstruction of the recovered polar kernel, not a complete aircraft FM.

Reference: DagorEngine 75723669297e48e200a0dc67b18c1629e0975daf,
gamePhys/common/polares.cpp; kernel matched against local aces instructions.
Angles are degrees. Geometry, flow, controls, damage and force summation belong
to the caller and are deliberately not replaced by this module.
"""
import math
import struct

FIELDS = ('cl0 cd0 indCoeff clLineCoeff cyCritH cyCritL aoaCritH aoaCritL '
          'aoaLineH aoaLineL parabCyCoeffH parabCyCoeffL aerCenterOffset '
          'clToCm0 clToCm1 parabAngle declineCoeff maxDistAng cdAfterCoeff '
          'clAfterCritL clAfterCritH kq clKq cyMult').split()


def safe_div(a, b):
    return a / b if abs(b) > 4e-19 else 0.0


def mach_multiplier(props, mach, index):
    """Advanced mode: constant, cubic Hermite transition, bounded linear tail."""
    low = 0.0 if index == 6 else 1.0
    a, b = props[f'MachCrit{index}'], props[f'MachMax{index}']
    high = props[f'MultMachMax{index}']
    slope, limit = props[f'MultLineCoeff{index}'], props[f'MultLimit{index}']
    if mach < a:
        return low
    if mach > b:
        value = high + slope * (mach - b)
        return min(value, limit) if slope >= 0 else max(value, limit)
    if a == b:
        # Public source's singular-matrix fallback stores low in coefficient 3.
        return low * mach ** 3
    t = (mach - a) / (b - a)
    return (2*t**3-3*t**2+1)*low + (-2*t**3+3*t**2)*high + (t**3-t**2)*(b-a)*slope


def table_value(props, mach):
    points = sorted((v for k,v in props.items() if k.startswith('ClToCmByMach') and len(v)==3), key=lambda p:p[0])
    if not points:
        return props.get('ClToCmByMach', [0.0,0.0])
    if mach <= points[0][0]:return points[0][1:]
    for a,b in zip(points,points[1:]):
        if mach <= b[0]:
            t=safe_div(mach-a[0],b[0]-a[0])
            return [a[i]+t*(b[i]-a[i]) for i in [1,2]]
    return points[-1][1:]


def make_polar(props, span, area, mach=0.0, cy_mult=1.0):
    """Published Mach model; span/area must be the loader's polar references.

    Do not substitute one component's damaged area for the reference area.
    Runtime interpolation and defaults of the aircraft loader are separate.
    """
    cl=props['Cl0']; ind=safe_div(1.0,math.pi*props['OswaldsEfficiencyNumber']*safe_div(span*span,area))
    p=dict(cl0=cl,cd0=props['CdMin'],indCoeff=ind,clLineCoeff=props['lineClCoeff'],
           cyCritH=props['ClCritHigh']*cy_mult,cyCritL=props['ClCritLow']*cy_mult,
           aoaCritH=props['alphaCritHigh'],aoaCritL=props['alphaCritLow'],
           aerCenterOffset=0.0,clToCm0=0.0,clToCm1=0.0,
           parabAngle=props['AfterCritParabAngle'],declineCoeff=props['AfterCritDeclineCoeff'],
           maxDistAng=props['AfterCritMaxDistanceAngle'],cdAfterCoeff=props['CxAfterCoeff'],
           clAfterCritL=props['ClAfterCritLow'],clAfterCritH=props['ClAfterCritHigh'],kq=1.0,clKq=1.0,cyMult=cy_mult)
    mode=props.get('MachFactor',0)
    if mode == 3:
        f={i:mach_multiplier(props,mach,i) for i in range(1,8)}
        p.update(cd0=p['cd0']*f[1],clLineCoeff=p['clLineCoeff']*((1+f[1]-f[2]) if props.get('CombinedCl',True) else f[2]),
                 cl0=cl*f[7],cyCritH=cl+(p['cyCritH']-cl*f[7])*f[3],cyCritL=cl+(p['cyCritL']-cl*f[7])*f[3],
                 aoaCritH=p['aoaCritH']*f[4],aoaCritL=p['aoaCritL']*f[4],indCoeff=ind*f[5],aerCenterOffset=f[6])
        p['clToCm0'],p['clToCm1']=table_value(props,mach)
    elif mode in [1,2]:
        mm=min(mach,0.9) if mode==1 else mach
        x=abs(1-mm*mm)
        if mode==1:x=min(x,1.0)
        p['kq']=1/max(x**0.3,0.2);p['clKq']=math.sqrt(p['kq'])
    elif mode != 0:
        raise ValueError(f'Unknown MachFactor {mode}')
    p['aoaLineH']=2*safe_div(p['cyCritH']-p['cl0'],p['clLineCoeff']*cy_mult)-p['aoaCritH']
    p['aoaLineL']=2*safe_div(p['cyCritL']-p['cl0'],p['clLineCoeff']*cy_mult)-p['aoaCritL']
    if p['aoaLineL'] > p['aoaLineH']:
        p['aoaLineL']=p['aoaLineH']=0.5*(p['aoaLineL']+p['aoaLineH'])
    p['parabCyCoeffH']=safe_div(p['cyCritH']-(p['cl0']+p['aoaLineH']*p['clLineCoeff']*cy_mult),(p['aoaCritH']-p['aoaLineH'])**2)
    p['parabCyCoeffL']=safe_div(-p['cyCritL']+(p['cl0']+p['aoaLineL']*p['clLineCoeff']*cy_mult),(p['aoaCritL']-p['aoaLineL'])**2)
    return p


def calc_cl(p,a):
    if p['aoaLineL'] <= a <= p['aoaLineH']:
        return p['cl0']+p['clLineCoeff']*a*p['cyMult']
    s=1 if a-p['aoaLineH']+0.01>=0 else -1
    crit=p['aoaCritH' if s>0 else 'aoaCritL']
    after=p['clAfterCritH' if s>0 else 'clAfterCritL']
    cy=p['cyCritH' if s>0 else 'cyCritL']
    if s*(a-crit)<=0:
        return cy-s*p['parabCyCoeffH' if s>0 else 'parabCyCoeffL']*(crit-a)**2
    max_ang=max(40,p['maxDistAng'])
    if s*a<=max_ang:
        if s*a<=p['maxDistAng']:
            da=a-crit
            if s*da<p['parabAngle']:return cy-s*p['declineCoeff']*da*da
            h=after*math.sin(math.pi*0.0125*p['maxDistAng'])
            max_da=s*(p['maxDistAng']-p['parabAngle'])-crit
            need=cy-s*p['declineCoeff']*p['parabAngle']**2-h
            return h+safe_div(need,max_da**2)*(s*p['maxDistAng']-a)**2
        return after*math.sin(math.pi*0.0125*s*a)
    if s*a<=140:
        local_sign=s;local_aoa=s*a
        if s*a>90:local_sign=-s;local_aoa=180-local_aoa
        correction=math.sin(math.pi*0.0125*max_ang)-math.sin(math.pi*0.5+math.pi*0.01*max(max_ang-40,0))
        return s*after*(s*correction*(1-(s*a-40)/100)+math.sin(math.pi*0.5+math.pi*0.01*(local_aoa-40))*local_sign)
    return s*after*math.sin(math.pi*0.0125*(180-s*a))*(-s)


def calc_cd(p,a):
    linear=p['cl0']+p['clLineCoeff']*a
    s=1 if a>=0 else -1
    crit=p['aoaCritH' if s>0 else 'aoaCritL']
    cd=p['cd0']+linear*linear*p['indCoeff']+p['cdAfterCoeff']*max(s*(a-crit),0)
    return min(cd,0.15+p['cyCritH']*abs(math.sin(math.radians(a))))


def calc_c(p,a,angle,cl_add=0.0,cd_coeff=1.0):
    cd=calc_cd(p,a)*cd_coeff;cl=calc_cl(p,a)+cl_add
    sn=math.sin(math.radians(angle));cs=math.cos(math.radians(angle))
    return ((cd*cs-cl*sn)*p['kq'],(cl*cs+cd*sn)*p['clKq'])


def pack_polar(p):
    return struct.pack('<24f',*(p[k] for k in FIELDS))


def round_polar(p):
    return dict(zip(FIELDS,struct.unpack('<24f',pack_polar(p))))
