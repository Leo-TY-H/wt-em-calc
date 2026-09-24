"""General jet-nozzle force/moment kernel, 1019ec630 / 1019f0fc0 / 1019f19e0.

Angles/curves use the loader's float32 degree conversion. Engine instances sum
nozzles in float32; callers promote each completed engine aggregate to double.
"""
import math
from component_assembly import f32,add,sub,mul
from control_mixer import prepare_rows,curve
from body_dynamics import accumulate_nozzle

RAD=f32(.01745329238474369)


def direction_basis(direction,direction2):
    def vector(angles):
        yaw,pitch=[mul(f32(a),RAD) for a in angles]
        sy,cy=f32(math.sin(yaw)),f32(math.cos(yaw));sp,cp=f32(math.sin(pitch)),f32(math.cos(pitch))
        return [mul(cy,cp),sp,mul(-sy,cp)]
    def cross(a,b):
        return [sub(mul(a[1],b[2]),mul(a[2],b[1])),sub(mul(a[2],b[0]),mul(a[0],b[2])),sub(mul(a[0],b[1]),mul(a[1],b[0]))]
    def unit(v,fallback,order=(0,1,2)):
        a,b,c=order;squared=add(add(mul(v[a],v[a]),mul(v[b],v[b])),mul(v[c],v[c]))
        if squared<.0001:return fallback
        inv=f32(1/f32(math.sqrt(squared)))
        return [mul(x,inv) for x in v]
    forward=vector(direction);up=unit(cross(vector(direction2),forward),[0.,0.,0.],(2,0,1))
    side=unit(cross(forward,up),[0.,0.,0.])
    return [forward,up,side]


def prepare(config):
    def rows(key,width,angular=False):
        found=[]
        for i in range(8):
            if key+str(i) in config:
                x,*values=config[key+str(i)];found.append((mul(f32(x),f32(1/3.6)),values))
        if not found:found=[(0.,config.get(key,[0.]*width))]
        return prepare_rows([(x,[mul(f32(v),RAD) if angular else f32(v) for v in vs]) for x,vs in found])
    weights=config.get('NozzleAnglesAxesWeights',{});ranges=config.get('NozzleAnglesRanges',{})
    defaults=[[1.,0.,1.,0.,0.],[0.,1.,0.,1.,1.]]
    axes=['Ailerons','Elevator','Rudder','Vtol','Reverse']
    result=dict(position=list(map(f32,config.get('Position',[0.,0.,0.]))),
        ratio=f32(config.get('ThrustRatio',1.)),maximum=f32(config.get('ThrustMax',2147440000.)),
        basis=direction_basis(config.get('Direction',[0.,0.]),config.get('Direction2',[-f32(math.pi/2),0.])),
        main_controls=bool(config.get('HasThrustVecMainControlsMode',False)),tip=bool(config.get('TipPosition',False)),
        weights=[[f32(weights.get(k,{}).get(axis,d)) for axis,d in zip(axes,default)] for k,default in zip(['Horizontal','Vertical'],defaults)],
        ranges=[[mul(f32(v),RAD) for v in ranges.get(k,[-180.,180.])] for k in ['Horizontal','Vertical']],
        flaps=list(map(f32,config.get('FlapsToThrust',[0.,1.,1.,1.]))))
    for short,key,width,angle in [('roll_angle','AileronsToThrustDeflection',3,True),('pitch_angle','ElevatorToThrustDeflection',3,True),
        ('yaw_angle','RudderToThrustDeflection',3,True),('vtol_angle','VtolToThrustDeflection',2,True),('reverse_angle','ReverseToThrustDeflection',2,True),
        ('roll_thrust','AileronsToThrust',3,False),('pitch_thrust','ElevatorToThrust',3,False),('yaw_thrust','RudderToThrust',3,False),
        ('airbrake','AirbrakeToThrust',2,False),('vtol','VtolToThrust',2,False),('reverse','ReverseToThrust',2,False)]:result[short]=rows(key,width,angle)
    return result


def control_value(values,command):
    lo,center,hi=values
    return add(mul(sub(hi,center) if command>=0 else sub(center,lo),command),center)


def evaluate(nozzles,thrust,cg,ias=0.,flaps=0.,airbrake=0.,vtol=0.,reverse=0.,sticks=(0.,0.,0.),main_controls=False,tip_axes=(False,False,False)):
    thrust,ias,flaps,airbrake,vtol,reverse=map(f32,(thrust,ias,flaps,airbrake,vtol,reverse))
    force=[0.]*3;moment=[0.]*3;angles=[];individual=[]
    for p in nozzles:
        roll,pitch,yaw=[f32(v) if not p['tip'] or enabled else 0. for v,enabled in zip(sticks,tip_axes)]
        enabled=not p['main_controls'] or main_controls
        r=control_value(curve(p['roll_angle'],ias,3),roll) if enabled else 0.
        e=-control_value(curve(p['pitch_angle'],ias,3),pitch) if enabled else 0.
        y=control_value(curve(p['yaw_angle'],ias,3),yaw) if enabled else 0.
        v0,v1=curve(p['vtol_angle'],ias,2);v=add(mul(sub(v1,v0),vtol),v0)
        r0,r1=curve(p['reverse_angle'],ias,2);reverse_angle=sub(mul(sub(r0,r1),reverse),r0)
        angle=[]
        for weights,limits in zip(p['weights'],p['ranges']):
            wr,we,wy,wv,wb=weights
            a=sub(mul(we,e),add(mul(wy,y),mul(wr,r)))
            a=add(sub(mul(wb,reverse_angle),mul(wv,v)),a)
            angle.append(min(limits[1],max(limits[0],a)))
        sh,ch=f32(math.sin(angle[0])),f32(math.cos(angle[0]));sv,cv=-f32(math.sin(angle[1])),f32(math.cos(angle[1]))
        b0,b1,b2=p['basis']
        direction=[add(mul(cv,add(mul(b2[i],sh),mul(b0[i],ch))),mul(sv,b1[i])) for i in [0,1]]
        direction.append(add(mul(b1[2],sv),mul(add(mul(b2[2],sh),mul(b0[2],ch)),cv)))
        x0,y0,x1,y1=p['flaps']
        if x1<x0:x0,x1,y0,y1=x1,x0,y1,y0
        blend=y0 if flaps<=x0 else y1 if flaps>=x1 else add(y0,f32(mul(sub(flaps,x0),sub(y1,y0))/sub(x1,x0)))
        capped=min(mul(mul(blend,thrust),p['ratio']),p['maximum'])
        multiplier=1.
        if enabled:
            rl,rc,rh=curve(p['roll_thrust'],ias,3);el,ec,eh=curve(p['pitch_thrust'],ias,3);yl,yc,yh=curve(p['yaw_thrust'],ias,3)
            rd=mul(sub(rh,rc) if roll>=0 else sub(rc,rl),roll)
            ed=mul(sub(eh,ec) if pitch>=0 else sub(ec,el),pitch)
            yd=mul(sub(yc,yh) if yaw>=0 else sub(yl,yc),yaw)
            multiplier=sub(add(add(add(add(ec,rd),ed),add(rc,1.)),yc),yd)
        for key,command in [('airbrake',airbrake),('vtol',vtol),('reverse',reverse)]:
            a,b=curve(p[key],ias,2);multiplier=add(add(a,multiplier),mul(sub(b,a),command))
        old_force=force;old_moment=moment
        force,moment=accumulate_nozzle(direction,p['position'],cg,capped,multiplier,force,moment)
        individual.append(dict(direction=direction,thrust=mul(multiplier,capped),angles=angle));angles.append(angle)
    return dict(force=force,moment=moment,angles=angles,nozzles=individual)
