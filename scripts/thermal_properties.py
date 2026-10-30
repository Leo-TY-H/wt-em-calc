"""Independent finite-input translation of thermal construction and calibration.

Native scalar expf, logf and powf are supplied dependencies. Constructor values
below come from 0x142fc5432..0x142fc55f7, not inferred configuration meanings.
"""
import struct
from thermal_core import f,add,sub,mul,div
EPS=f(4e-19);K=f(273.15)
def safe(a,b):return div(a,b) if abs(b)>EPS else 0.
def density(z,sea_density=f(1.225),cutoff=18300.):
    z=f(z);x=min(z,cutoff)
    v=add(mul(add(mul(add(mul(add(mul(x,2.287190065873038e-19),-5.83556030646186e-14),x),3.5311800150594763e-09),x),-9.593870345270261e-05),x),1.)
    return div(mul(mul(cutoff,sea_density),v),max(z,cutoff))
def append(rows,x,payload):
    if rows:
        d=sub(x,rows[-1][0])
        if d<=EPS:return
        rows[-1][1]=div(1.,d)
    rows.append([x,0.,*payload])
def initial_slots():
    values=[.15,.15,.01,.01,.1,.1,0.,1.,1.,1.,1.,30.5,.05285409092903137,20.5,.047745924443006516,
            .3,.3,2.677441358566284,-2.177441358566284,2.677441358566284,-2.177441358566284,
            .1,.1,.2,.2,0.,0.,0.,0.,.4,.2,.02,.2,0.,0.,373.1499938964844,633.1500244140625,.05,.05,
            363.1499938964844,378.1499938964844,1/180,.6666666865348816,40.,40.]
    return {0x144+4*j:f(v) for j,v in enumerate(values)}

def solve_exponent(base,ambient,last,ratio,target,start,powf):
    """Native bounded Newton/finite-difference calibration, load direction only.

    The output is changed only on convergence. Exhausting ten iterations or a
    near-zero derivative preserves the original exponent, not the last iterate.
    """
    q=mul(sub(last,ambient),base)
    source=add(mul(sub(last,ambient),1.3),ambient)
    heat=div(q,sub(source,last));source=mul(source,heat)
    lower,upper=-10.,10.;exponent=start
    for _ in range(10):
        pw=powf(ratio,exponent);cool=mul(pw,base)
        current=safe(add(mul(cool,ambient),source),add(cool,heat))
        pw=powf(ratio,add(exponent,.1))
        adjacent=safe(add(mul(pw,mul(ambient,base)),source),add(mul(pw,base),heat))
        derivative=mul(sub(adjacent,current),10.)
        if abs(derivative)<f(2**-23):return start
        error=sub(current,target)
        if mul(derivative,error)>0.:upper=exponent
        else:lower=exponent
        step=div(-error,derivative);exponent=add(exponent,step)
        if lower!=-10. and upper!=10. and (exponent<lower or exponent>upper):exponent=mul(add(lower,upper),.5)
        else:exponent=min(upper,max(lower,exponent))
        if abs(step)<f(.0025):return exponent
    return start

def construct(data,special,water,powf,expf,logf,ambient=f(288.1600036621094),global_mult=1.,sea_density=f(1.225),density_cutoff=18300.):
    p=initial_slots();modes=[];temps=[];mode_rows=[];load_rows=[[],[]];loads=[];rad=[]
    rho_at=lambda z:density(z,sea_density,density_cutoff)
    get=lambda name,default:f(data.get(name,default))
    p[0x1e8]=mul(p[0x1e8],global_mult);p[0x1ec]=mul(p[0x1ec],global_mult)
    for c,prefix in enumerate(('Water','Oil')):
        p[0x144+4*c]=get('NoIgnition'+prefix+'HeatProducingFactor',.15)
        p[0x1c8+4*c]=get(prefix+'AirDensityPower',0.)
        if special and c==0:continue
        power=get(prefix+'RadiatorEffectPower',.3);p[0x180+4*c]=power
        a=expf(mul(power,-.1));b=expf(-power)
        scale=safe(sub(1.,get(prefix+'MinRadiatorThermalConductionCoeff',.5)),sub(b,a))
        p[0x18c+8*c]=scale;p[0x188+8*c]=sub(1.,mul(b,scale))
        p[0x198+4*c]=get(prefix+'MinThermostatThermalConductionCoeff',.1)
        p[0x1a0+4*c]=get(prefix+'MinLevelThermalConductionCoeff',.2)
        for j,v in enumerate(data.get(prefix+'ThermalConductionCrossCoeffTable',[.4,.2,.02,.2])):p[0x1a8+16*c+4*j]=f(v)
        p[0x1d0+4*c]=add(get(prefix+'BoilingTemperature',150. if c==0 else 360.),K)
        p[0x1d8+4*c]=safe(1.,get(prefix+'ThermostatInertia',20.))
        p[0x1e0+4*c]=add(get(prefix+'ThermostatSetPoint',90. if c==0 else 105.),K)
    no_flow=0 if special else int(data.get('NoFlowMode',0));p[0x15c]=struct.unpack('<f',struct.pack('<I',no_flow&0xffffffff))[0]
    legacy=data.get('DegradationTime_0_10',[180.,150.]);modern=data.get('DegradationTime_10_5',[div(x,global_mult) for x in legacy])
    p[0x1e8]=safe(1.,f(modern[0]));p[0x1ec]=mul(sub(safe(1.,f(modern[1])),p[0x1e8]),600.)
    for c,prefix in enumerate(('Water','Oil')):p[0x1f0+c*4]=add(get('HalfHealth'+prefix+'TemperatureToleranceDecrease',20.),get('HalfHealth'+prefix+'TemperatureToleranceDecrease',20.))
    calibration=ambient if special else add(ambient,.5)
    anchors=[None,None]
    for j in range(6):
        if 'Mode'+str(j) not in data:break
        block=data['Mode'+str(j)];omega=mul(block.get('RPM',2400+200*j),.10471975827217102);alt=f(block.get('Altitude',1000.))
        coordinate=omega if special else mul(omega,block.get('ManifoldPressure',add(1.,mul(.2,j))))
        append(mode_rows,coordinate,[j]);tt=[add(f(block.get(prefix+'Temperature',add(363.15+20*c,mul(10.,j)))),K) for c,prefix in enumerate(('Water','Oil'))]
        flags=[int(tt[c]>p[0x1e0+4*c]) for c in range(2)]
        modes.append([omega,alt,0.,0.,0.,0.,*flags]);temps.append(tt)
        for c in range(2):
            if anchors[c] is None and (flags[c] or (c==0 and not water)):anchors[c]=(safe(1.,sub(tt[c],calibration)),rho_at(alt))
    inertia=[1.,1.]
    for c in range(2):
        base,rho=anchors[c] if anchors[c] is not None else (0.,0.)
        exponent=p[0x1c8+4*c]
        if special and modes:
            last=temps[-1][c];delta=sub(last,ambient);q=mul(delta,base);ratio=safe(rho_at(10000.),rho)
            coefficient=mul(powf(ratio,exponent),base)
            target=safe(add(add(q,last),mul(coefficient,ambient)),add(coefficient,1.))
            if abs(exponent)>f(2**-23):exponent=solve_exponent(base,ambient,last,ratio,target,exponent,powf)
            p[0x1c8+4*c]=exponent
            d=mul(delta,1.3)
            inertia[c]=div(mul(add(sub(ambient,last),d),add(q,delta)),mul(d,q))
        p[0x14c+4*c]=safe(base,powf(rho,exponent))
        prefix=('Water','Oil')[c]
        p[0x160+4*c]=get(prefix+'FlowEffectPower',1.);p[0x168+4*c]=get(prefix+'PropFlowSpeedCoeff',1.)
    for j,mode in enumerate(modes):
        effective_speed=0.
        for c in range(2):
            if anchors[c] is None:continue
            prefix=('Water','Oil')[c];temp=temps[j][c]
            thermostat=1. if mode[6+c] or (c==0 and not water) else p[0x198+4*c]
            heat=mul(mul(mul(thermostat,p[0x14c+4*c]),sub(temp,calibration)),powf(rho_at(mode[1]),p[0x1c8+4*c]))
            delta=mul(sub(temp,ambient),1.3) if special else 700.
            denominator=add(sub(ambient,temp),delta) if special else sub(add(700.,ambient),temp)
            heat=div(heat,denominator);mode[2+c*2]=heat;mode[3+c*2]=delta
            if not special and j==no_flow:
                if c==0 or anchors[0] is None:effective_speed=mul(get('CoolingEffectiveAirSpeed',77.77777862548828),.2777777910232544)
                noflow=add(get(prefix+'TemperatureNoFlow',add(temp,30.)),K)
                conduction=p[0x14c+4*c];ct=mul(noflow,conduction);ca=mul(conduction,ambient)
                source=mul(add(delta,ambient),heat)
                scale=safe(add(mul(noflow,heat),sub(ct,add(source,ca))),sub(ct,ca))
                p[0x170+8*c]=scale
                log=logf(max(f(1e-9),safe(.5,mul(temp,scale))))
                p[0x174+8*c]=f(-safe(log,powf(effective_speed,p[0x160+4*c])))
    for c,prefix in enumerate(('Water','Oil')):p[0x154+4*c]=mul(safe(1.,get(prefix+'TemperatureInertia',10.)),inertia[c])
    for j in range(8):
        if 'Load'+str(j) not in data:break
        block=data['Load'+str(j)]
        for c,prefix in enumerate(('Water','Oil')):append(load_rows[c],add(f(block.get(prefix+'Temperature',add(363.15+20*c,mul(10.,j)))),K),[j])
        default=1e9 if j==0 else f(900./(3.**(j-1)))
        loads.append([safe(1.,f(block.get('WorkTime',default))),safe(1.,f(block.get('RecoverTime',1e-9 if j==0 else default/3.)))])
    if not special:
        for j in range(6):
            if 'RadiatorMode'+str(j) not in data:break
            block=data['RadiatorMode'+str(j)]
            append(rad,f(block.get('Throttle',0.)),[add(f(block.get(prefix+'Temperature',add(363.15+20*c,mul(10.,j)))),K) for c,prefix in enumerate(('Water','Oil'))])
    return dict(mode_count=len(modes),modes=modes,has_oil=int(bool(data.get('HasOil',True))),load_count=len(loads),loads=loads,
                slots={hex(k):v for k,v in p.items()},mode_rows=mode_rows,water_rows=load_rows[0],oil_rows=load_rows[1],radiator_rows=rad)
