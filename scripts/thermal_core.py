"""Recovered .91 finite-input thermal endurance arithmetic, not a full engine model.

Addresses: rates 0x1449e6440; integration and health 0x1449ea6b6/0x1449ea916.
Inputs are loaded runtime tables, never interpretations of raw config names.
"""
import bisect, struct
def f(x): return struct.unpack('<f',struct.pack('<f',x))[0]
def add(a,b): return f(f(a)+f(b))
def sub(a,b): return f(f(a)-f(b))
def mul(a,b): return f(f(a)*f(b))
def div(a,b): return f(f(a)/f(b))
def equal_bits(a,b):
    if isinstance(a,dict):return a.keys()==b.keys() and all(equal_bits(a[k],b[k]) for k in a)
    if isinstance(a,(list,tuple)):return len(a)==len(b) and all(equal_bits(x,y) for x,y in zip(a,b))
    if isinstance(a,float) or isinstance(b,float):return struct.pack('<f',a)==struct.pack('<f',b)
    return a==b

def rates(rows, work, recover, temperature, state_count=None):
    n=len(work) if state_count is None else state_count
    out=[0.]*8
    if not rows:return out
    t=f(temperature)
    if len(rows)==1 or t<=rows[0][0]: point=rows[0][2]; fraction=0.; endpoint=True
    elif t>=rows[-1][0]: point=rows[-1][2]; fraction=0.; endpoint=True
    else:
        k=bisect.bisect_left([r[0] for r in rows],t)
        point=rows[k][2]; fraction=mul(sub(t,rows[k-1][0]),rows[k-1][1]); endpoint=False
    if endpoint and point!=0:point=len(work)
    point=min(point,n)
    for i in range(point):out[i]=add(out[i],work[i])
    if point<n:out[point]=add(out[point],mul(work[point],fraction))
    for i in range(point+1,len(work)):out[i]=sub(out[i],recover[i])
    if point<n:out[point]=add(out[point],mul(add(fraction,-1.),recover[point]))
    return out

def endurance_step(tables,work,recover,temperatures,counters,health,shifts,dt,damage_enabled=True):
    dt=f(dt); health=f(health); out=[list(x) for x in counters]; worst=0.; channel=2
    allrates=[]
    for c,t in enumerate(temperatures):
        effective=add(t,f(float(f(shifts[c]))*(1.-float(health))))
        rr=rates(tables[c],work,recover,effective); allrates.append(rr)
        for i in range(len(work)):
            value=add(out[c][i],mul(rr[i],dt))
            if value>1. and rr[i]>worst:worst=rr[i];channel=c
            out[c][i]=min(1.,max(0.,value))
    return out,worst,channel,allrates

def damage(health,worst,dt,base,slope,enabled=True):
    if worst<=0. or not enabled:return f(health)
    excess=max(0.,add(worst,f(-1./600.)))
    return max(0.,sub(health,mul(add(mul(excess,slope),base),dt)))
