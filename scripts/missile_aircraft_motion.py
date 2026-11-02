"""Reduced point-mass aircraft; all units SI. Shared Python/native/WASM source."""
import math
from copy import deepcopy


def validate_profile(p):
    if not isinstance(p,dict) or p.get('version')!=1: raise ValueError('Unsupported target aircraft profile')
    for key in ['aircraft_id','name','source_sha256']:
        if not isinstance(p.get(key),str) or not 1<=len(p[key])<=160: raise ValueError('Invalid aircraft identity')
    bounds=[['mass_kg',100.,1000000.],['area_m2',1.,2000.],['fuel_fraction',0.,1.],['throttle',1.,1.1],
            ['positive_g',1.,100.],['negative_g',-100.,0.],['vne_mps',50.,2000.],['mne',.2,5.],
            ['roll_rate_deg_s',1.,360.],['roll_reference_mps',1.,1000.],['response_s',.05,2.]]
    for key,lo,hi in bounds:
        v=p.get(key)
        if isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or not lo<=v<=hi:
            raise ValueError('Invalid target aircraft parameter: '+key)
    for key,width,maximum in [['atmosphere',3,64],['aero',5,256]]:
        rows=p.get(key)
        if not isinstance(rows,list) or not 2<=len(rows)<=maximum: raise ValueError('Invalid aircraft table: '+key)
        previous=-1.
        for row in rows:
            if not isinstance(row,list) or len(row)!=width: raise ValueError('Invalid aircraft table row')
            for v in row:
                if isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or abs(v)>100000.:
                    raise ValueError('Invalid aircraft table value')
            if row[0]<=previous: raise ValueError('Aircraft table must be increasing')
            previous=row[0]
            if key=='atmosphere' and (row[1]<=0 or row[2]<=0): raise ValueError('Invalid atmosphere')
            if key=='aero' and (row[1]<0 or row[2]<0 or row[3]>=0 or row[4]<=0): raise ValueError('Invalid aerodynamic polar')
    speeds=p.get('speeds'); thrust=p.get('thrust')
    if not isinstance(speeds,list) or not 2<=len(speeds)<=64: raise ValueError('Invalid thrust speeds')
    previous=-1.
    for v in speeds:
        if isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or not previous<v<=2000.:
            raise ValueError('Invalid thrust speed')
        previous=v
    if not isinstance(thrust,list) or len(thrust)!=len(p['atmosphere']): raise ValueError('Invalid thrust heights')
    for row in thrust:
        if not isinstance(row,list) or len(row)!=len(speeds): raise ValueError('Invalid thrust row')
        for v in row:
            if isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or not 0<=v<=10000000.:
                raise ValueError('Invalid thrust')
    if p['atmosphere'][0][0]!=0 or p['atmosphere'][-1][0]!=30000 or speeds[0]!=0 or speeds[-1]<1000 or p['aero'][0][0]!=0 or p['aero'][-1][0]<p['mne']:
        raise ValueError('Incomplete aircraft table domain')
    return deepcopy(p)


def bracket(rows,x):
    i=0
    while i+2<len(rows) and x>rows[i+1][0]: i+=1
    return i,max(0.,min(1.,(x-rows[i][0])/(rows[i+1][0]-rows[i][0])))


def lookup(rows,x):
    i,t=bracket(rows,x)
    return [rows[i][k]+(rows[i+1][k]-rows[i][k])*t for k in range(1,len(rows[i]))]


def forces(m,p,v,u,load):
    from missile_live_motion import dot
    data=m['aircraft']; speed=math.sqrt(dot(v,v)); height=max(0.,min(30000.,p[1]))
    rho,sound=lookup(data['atmosphere'],height)
    cd0,k,clmin,clmax=lookup(data['aero'],speed/sound)
    qarea=.5*rho*speed*speed*data['area_m2']; weight=data['mass_kg']*9.81
    lift=max(max(data['negative_g']*weight,qarea*clmin),min(min(data['positive_g']*weight,qarea*clmax),load*weight))
    cl=lift/max(qarea,.000001); drag=qarea*(cd0+k*cl*cl)
    i,t=bracket(data['atmosphere'],height); j=0
    while j+2<len(data['speeds']) and speed>data['speeds'][j+1]: j+=1
    f=max(0.,min(1.,(speed-data['speeds'][j])/(data['speeds'][j+1]-data['speeds'][j])))
    a=data['thrust'][i][j]+(data['thrust'][i][j+1]-data['thrust'][i][j])*f
    b=data['thrust'][i+1][j]+(data['thrust'][i+1][j+1]-data['thrust'][i+1][j])*f
    thrust=a+(b-a)*t
    acceleration=[(thrust-drag)/data['mass_kg']*v[n]/max(speed,.000001)+lift/data['mass_kg']*u[n] for n in range(3)]
    acceleration[1]-=9.81
    return dict(acceleration=acceleration,load_g=lift/weight,thrust=thrust,drag=drag,speed=speed,
                outside=speed<25. or speed>1000. or speed/sound>data['mne'] or speed*math.sqrt(rho/1.225)>data['vne_mps'] or p[1]>30000.)


def advance(m,old,new,el,roll):
    from missile_live_motion import dot,cross,unit,frame_quaternion,series_sin_cos
    data=m['aircraft']; dt=(new-old)/4.; p=m['p']; v=m['dynamic_velocity']; u=m['u']
    start_p=p[:]; start_v=v[:]; start_q=frame_quaternion(m['f'],u,m['r'])
    for substep in range(4):
        f=unit(v); r=cross(f,u); rho,sound=lookup(data['atmosphere'],max(0.,min(30000.,p[1])))
        ias=math.sqrt(dot(v,v))*math.sqrt(rho/1.225)
        effectiveness=min(1.,ias/max(data['roll_reference_mps'],1.),max(data['roll_reference_mps'],1.)/max(ias,1.))
        angle=roll*math.radians(data['roll_rate_deg_s'])*effectiveness*dt
        sn,cs=series_sin_cos(angle); u=[cs*u[n]+sn*r[n] for n in range(3)]
        demand=m['command_g'] if el>0 else -min(m['command_g'],-data['negative_g']) if el<0 else 1.
        m['requested_g']+=(demand-m['requested_g'])*(dt/(data['response_s']+dt))
        first=forces(m,p,v,u,m['requested_g']); a=first['acceleration']
        vm=[v[n]+a[n]*dt*.5 for n in range(3)]; pm=[p[n]+v[n]*dt*.5 for n in range(3)]
        fm=unit(vm); um=unit([u[n]-dot(u,fm)*fm[n] for n in range(3)])
        middle=forces(m,pm,vm,um,m['requested_g']); a=middle['acceleration']
        p=[p[n]+vm[n]*dt for n in range(3)]; v=[v[n]+a[n]*dt for n in range(3)]
        f=unit(v); u=unit([u[n]-dot(u,f)*f[n] for n in range(3)])
        m['outside']=m['outside'] or first['outside'] or middle['outside']
    r=cross(f,u); m['p']=p; m['dynamic_velocity']=v; m['f']=f; m['u']=u; m['r']=r
    end=forces(m,p,v,u,m['requested_g']); m['actual_g']=end['load_g']; m['outside']=m['outside'] or end['outside']
    m['segments'].append(dict(kind='aircraft',t0=old,t1=new,p0=start_p,p1=p,v0=start_v,v1=v,q0=start_q,q1=frame_quaternion(f,u,r)))
    m['last_time']=new
