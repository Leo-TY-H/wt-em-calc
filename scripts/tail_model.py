"""Recovered secondary-surface stages in the detailed aircraft update.

All vectors use the executable's raw xyz convention. Angles are degrees unless
explicitly named radians. The wake uses the executable's inline trigonometric
approximation rather than substituting a conventional Euler rotation.
"""
import math
from component_assembly import f32, add, sub, mul
from component_stages import local_flow, horizontal_application_point, secondary_forces
from downwash import scalar_stage

RAD=f32(0.01745329238474369)
DEG=f32(57.2957763671875)
PI=f32(math.pi)


def div(a,b):
    return f32(a/b)


def guarded_div(a,b):
    return div(a,b) if abs(b)>f32(4e-19) else 0.0


def inline_half_sincos(angle):
    """0x106c612b9..613b5, finite-angle lane of the SIMD polynomial."""
    h=mul(angle,.5)
    n=math.trunc(add(math.copysign(.5,h),mul(angle,f32(.31830987334251404))))
    r=add(mul(f32(n),f32(-1.5707963705062866)),h)
    r2=mul(r,r)
    c=add(mul(add(mul(add(mul(r2,f32(-.0013602249091491103)),f32(.04165669530630112)),r2),f32(-.4999990165233612)),r2),1.)
    s=add(mul(add(mul(add(mul(r2,f32(-.0001950727018993348)),f32(.00833207555115223)),r2),f32(-.16666652262210846)),mul(r2,r)),r)
    sn,cs=(s,c) if n%2==0 else (c,s)
    return (-sn if n&2 else sn),(-cs if (n+1)&2 else cs)


def wake_quaternion(alpha_degrees,beta_degrees,dihedral_degrees):
    sa,ca=inline_half_sincos(sub(PI,mul(beta_degrees,RAD)))
    sb,cb=inline_half_sincos(mul(alpha_degrees,RAD))
    sc,cc=inline_half_sincos(mul(dihedral_degrees,RAD))
    return [add(mul(mul(cc,sa),sb),mul(mul(sc,ca),cb)),
            add(mul(mul(cc,sa),cb),mul(mul(sc,ca),sb)),
            sub(mul(mul(cc,ca),sb),mul(mul(sc,sa),cb)),
            sub(mul(mul(cc,ca),cb),mul(mul(sc,sa),sb))]


def wake_projection(wing_point,tail_point,alpha_degrees,beta_degrees,dihedral_degrees):
    """Exact operation order of 0x106c613b5..615a1."""
    x,y,z,w=wake_quaternion(alpha_degrees,beta_degrees,dihedral_degrees)
    dx,dy,dz=[sub(tail_point[i],wing_point[i]) for i in range(3)]
    xx=add(mul(x,x),mul(x,x));xy=mul(add(x,x),y)
    zw=mul(mul(z,-2.),w);xz=mul(add(z,z),x)
    yw=mul(mul(y,-2.),w)
    base=add(add(mul(w,w),mul(w,w)),-1.)
    px=add(add(mul(add(xz,yw),dz),mul(sub(xy,zw),dy)),mul(add(xx,base),dx))
    yz=mul(add(z,z),y);xw=mul(mul(x,-2.),w)
    py=add(mul(dy,add(base,add(mul(y,y),mul(y,y)))),add(mul(dx,add(zw,xy)),mul(dz,sub(yz,xw))))
    pz=add(mul(dz,add(base,add(mul(z,z),mul(z,z)))),add(mul(dy,add(xw,yz)),mul(dx,sub(xz,yw))))
    return [px,py,pz]


def wrap_degrees(value):
    value=f32(math.fmod(value,360.))
    return add(value,-360. if value>=180. else 360. if value < -180. else 0.)


def legacy_downwash(mode,tail_flows,vertical_axial,aspect,distance,alpha,current_cl,
                    previous_cl,engine_flow=(0.,0.),engine_count=1):
    """106c60e31..6109f: legacy modes 0/1, retaining previous wing CL."""
    angles=[]
    if mode==1:
        attenuation=1./(1.+float(f32(distance))*.04)**2
        factor=f32(attenuation*div(mul(vertical_axial,f32(1/math.pi)),aspect))
        angle=mul(f32(alpha),RAD);sn=f32(math.sin(angle));cs=f32(math.cos(angle))
    elif mode!=0:raise ValueError('Unknown legacy downwash mode')
    for i,flow in enumerate(tail_flows):
        x,y=flow[:2]
        if mode==1:
            induced=mul(f32(current_cl[i]),factor)
            x=add(mul(sn,induced),x);y=add(mul(cs,induced),y)
            if engine_count==1:
                x+=engine_flow[0]
                y+=engine_flow[1]*.36000001430511475*(-1 if i==0 else 1)
        angles.append(mul(f32(math.atan2(y,x)),-DEG))
    return dict(angles=angles,projection=[],correction=[],previous_cl=list(previous_cl))


def type2_downwash(span,area,sweep,taper,dihedral,wing_points,tail_points,
                   velocity,omega_x,alpha_degrees,beta_degrees,current_cl,
                   previous_cl,dt,coefficient,tail_flows,travel_cap=1.,engine_flow=(0.,0.),clockwise=False):
    """Entire normal jet branch 0x106c60fa0..618d9 (no propeller wash).

    Previous wing CL is updated even if the tail lies upstream of the wake.
    Return left/right local flow angles after downwash, plus detailed geometry.
    """
    span,area,sweep,taper,dihedral,dt,coefficient=map(f32,(span,area,sweep,taper,dihedral,dt,coefficient))
    speed2=f32(velocity[2]*velocity[2]+(velocity[0]*velocity[0]+velocity[1]*velocity[1]))
    invspeed=guarded_div(1.,speed2)
    invhalf=guarded_div(2.,span)
    shape=add(mul(f32(math.exp(mul(taper,f32(-.35)))),f32(-1.2)),f32(1.56))
    amp=mul(mul(mul(area,add(f32(math.sin(mul(sweep,RAD))),1.)),shape),f32(-46.2))
    invspan2=div(1.,mul(span,span));invdt=div(1.,dt)
    angles=[];projections=[];corrections=[]
    for i in range(2):
        projected=wake_projection(wing_points[i],tail_points[i],alpha_degrees,beta_degrees,dihedral if i==0 else -dihedral)
        x,y,z=projected;delta=0.
        if x>0:
            delta=scalar_stage(x,(y,z),invspeed,f32(travel_cap),omega_x,f32(current_cl[i]),mul(sub(previous_cl[i],current_cl[i]),invdt),invhalf,mul(f32(-2.5),invhalf),amp,coefficient,invspan2)
        axial,swirl=engine_flow
        side=1 if (i==0)==clockwise else -1
        local=mul(f32(math.atan2(tail_flows[i][1]+swirl*(side*.36000001430511475),tail_flows[i][0]+axial)),-DEG)
        angles.append(wrap_degrees(add(delta,local)));projections.append(projected);corrections.append(delta)
    return dict(angles=angles,projection=projections,correction=corrections,previous_cl=list(map(f32,current_cl)))


def history_increment(current_degrees,previous_degrees,scale,flow_inertia,dt,vertical=False):
    """Faithful raw-degree delta wrapped at a numerical 2*pi, as implemented.

    The executable does not convert these stored degree-valued angles to radians
    before applying fmodf(delta, 2*pi). This unusual detail is deliberate here.
    """
    delta=f32(math.fmod(sub(current_degrees,previous_degrees),f32(2*math.pi)))
    if delta>PI:delta=add(delta,f32(-2*math.pi))
    elif delta < -PI:delta=add(delta,f32(2*math.pi))
    if vertical:return mul(f32(flow_inertia),div(mul(f32(scale),-delta),f32(dt)))
    return div(mul(mul(delta,f32(scale)),f32(flow_inertia)),f32(dt))


def damage_drag_multiplier(main_area,control_area,main_health,control_health,
                           curve=((.3,.5),(1.,0.,0.,0.),(1.,0.,0.,0.),4.),area_add=0.,main_area_scale=1.):
    """Tail damage coefficient 0x106c61a97..61bb7 / 61cc2..61dd2.

    Curve is a runtime-global input: ((threshold, split), cubic_low, cubic_high,
    gain). Defaults are the analyzed binary image, before runtime overrides.
    Supply 1.5*main_area and area_add for the vertical surface.
    """
    if main_health>=1. and control_health>=1.:return 1.
    m=add(mul(mul(main_health,main_area_scale),main_area),area_add)
    main_area=mul(main_area,main_area_scale)
    e=mul(control_health,control_area)
    total=add(add(area_add,control_area),main_area)
    surviving=add(m,e)
    multiplier=guarded_div(total,surviving)
    ratio=guarded_div(add(add(m,control_area),mul(sub(e,control_area),f32(.33))),total)
    thresholds,low,high,gain=curve
    if ratio>=f32(thresholds[0]):
        a,b,c,d=map(f32,low if ratio<f32(thresholds[1]) else high)
        poly=add(mul(add(mul(add(mul(d,ratio),c),ratio),b),ratio),sub(a,1.))
        multiplier=mul(multiplier,add(mul(poly,f32(gain)),1.))
    return multiplier


def fuselage_drag_multiplier(area,health,curve):
    if health>=1.:return 1.
    factor=div(1.,health) if abs(mul(area,health))>f32(4e-19) else 0.
    ratio=health if abs(area)>f32(4e-19) else 0.
    thresholds,low,high,gain=curve
    if ratio>=f32(thresholds[0]):
        a,b,c,d=map(f32,low if ratio<f32(thresholds[1]) else high)
        poly=add(mul(add(mul(add(mul(d,ratio),c),ratio),b),ratio),sub(a,1.))
        factor=mul(factor,add(mul(poly,f32(gain)),1.))
    return factor


def secondary_coefficients(polars,controls,local_angles,body_angles,previous_angles,
                           incidence,flow_inertia,dt,areas,health,
                           convert_aoa=False,horizontal_bias=0.,history_scales=(1.,1.),
                           vertical_control_scale=1.,vertical_area_add=0.,
                           damage_curve=((.3,.5),(1.,0.,0.,0.),(1.,0.,0.,0.),4.),
                           fuselage_damage_curve=((.3,.5),(1.,0.,0.,0.),(1.,0.,0.,0.),1.),
                           coefficient_evaluator=None):
    """All secondary effective angles, rotated coefficients and Cd additions.

    controls supplies three complete mixer output records: left_hstab,
    right_hstab, vstab. local_angles is left/right wake-adjusted angle plus
    vertical/fuselage -atan2(z,x) in degrees. Points are handled separately.
    coefficient_evaluator(p, alpha, rotation, cl_add, cd_multiplier) is injectable
    to use the original polar machine code; default is the reconstructed kernel.
    """
    if coefficient_evaluator is None:
        from polar_f32 import calc_c
        coefficient_evaluator=calc_c
    hinc=history_increment(body_angles[0],previous_angles[0],history_scales[0],flow_inertia['hstab'],dt)
    # The fuselage shares the vertical angle derivative but has its own inertia.
    rate=history_increment(body_angles[1],previous_angles[1],history_scales[1],1.,dt,True)
    vinc=mul(f32(flow_inertia['vstab']),rate)
    finc=mul(rate,f32(flow_inertia['fuselage']))
    coeff={};angles={};multipliers={}
    ib=add(f32(horizontal_bias),f32(incidence['hstab']))
    for name,i,side in [('left_hstab',0,'left'),('right_hstab',1,'right')]:
        c=list(map(f32,controls[name]));hc=f32(health[side+'_elevator'])
        a=add(add(add(mul(c[1],hc),ib),f32(local_angles[i])),hinc)
        d=damage_drag_multiplier(areas[side+'_main'],areas[side+'_elevator'],health[side+'_main'],hc,damage_curve)
        cx,cy=coefficient_evaluator(polars['hstab'],a,f32(body_angles[0]) if convert_aoa else a,mul(c[2],hc),d)
        coeff[name]=[add(f32(cx),mul(c[3],hc)),f32(cy)];angles[name]=a;multipliers[name]=d
    c=list(map(f32,controls['vstab']));hc=f32(health['rudder'])
    a=add(sub(sub(f32(local_angles[2]),f32(incidence['vstab'])),mul(mul(f32(vertical_control_scale),c[1]),hc)),vinc)
    d=damage_drag_multiplier(areas['v_main'],areas['rudder'],health['v_main'],hc,damage_curve,vertical_area_add,1.5)
    cx,cy=coefficient_evaluator(polars['vstab'],a,-f32(body_angles[1]) if convert_aoa else a,mul(c[2],-hc),d)
    coeff['vstab']=[add(f32(cx),mul(c[3],hc)),f32(cy)];angles['vstab']=a;multipliers['vstab']=d
    a=add(finc,f32(local_angles[3]));d=fuselage_drag_multiplier(areas['fuselage'],health['fuselage'],fuselage_damage_curve)
    coeff['fuselage']=list(map(f32,coefficient_evaluator(polars['fuselage'],a,-f32(body_angles[1]) if convert_aoa else a,0.,d)))
    angles['fuselage']=a;multipliers['fuselage']=d
    return dict(coefficients=coeff,effective_angles=angles,drag_multipliers=multipliers,previous_angles=list(map(f32,body_angles)))


def secondary_points(base_points,polars,coefficients):
    out={}
    for name,p in base_points.items():
        polar=polars['hstab' if 'hstab' in name else name]
        cx,cy=coefficients[name]
        point=horizontal_application_point([p[0],p[1] if 'hstab' in name else p[2],p[2] if 'hstab' in name else p[1]],cx,cy,0.,polar['clToCm0'],polar['clToCm1'])
        out[name]=point if 'hstab' in name else [point[0],point[2],point[1]]
    return out



def horizontal_flows(points,cog,velocity,omega,disturbance):
    v,w,d=[list(map(f32,x)) for x in [velocity,omega,disturbance]]
    out=[]
    for i,p in enumerate(points):
        x,y,z=[sub(p[j],f32(cog[j])) for j in range(3)]
        out.append([add(sub(add(v[0],d[0]),mul(w[1],z)),mul(w[2],y)),
                    sub(add(mul(w[0],z),add(d[1],v[1])),mul(w[2],x)),
                    add(sub(add(d[2],v[2]),mul(w[0],y)),mul(w[1],x)) if i==0 else sub(add(add(d[2],v[2]),mul(w[1],x)),mul(w[0],y))])
    return out


def fuselage_flow(point,cog,velocity,omega):
    v,w=[list(map(f32,x)) for x in [velocity,omega]]
    x,y,z=[sub(point[j],f32(cog[j])) for j in range(3)]
    return [add(v[0],sub(mul(y,w[2]),mul(z,w[1]))),
            add(v[1],sub(mul(z,w[0]),mul(x,w[2]))),
            add(sub(mul(x,w[1]),mul(y,w[0])),v[2])]

def secondary_model(properties,state,wing,controls,coefficient_evaluator=None):
    """Prepared-state secondary surfaces for F-16/Gripen jets, finite inputs.

    properties: polars, span/area/sweep/taper/dihedral, downwash_coefficient,
      arms {hstab,vstab,fuselage}, incidence, flow_inertia, areas; optional scales.
    state: velocity, omega, cog, density, dt, body_angles, previous_angles,
      previous_wing_cl, disturbance (one vertical-surface sample), health.
    wing: points [left,right] before moment-equivalent shifts, cl [left,right].
    controls: complete mixer outputs for left_hstab/right_hstab/vstab.
    No aircraft control/instructor substitution or engine model is performed.
    """
    p,s=properties,state;polar=p['polars'];base={}
    for name in ['hstab','vstab','fuselage']:
        arm=list(map(f32,p['arms'][name]));arm[0]=add(arm[0],polar[name]['aerCenterOffset'])
        if name=='hstab':base['left_hstab']=arm;base['right_hstab']=[arm[0],arm[1],-arm[2]]
        else:base[name]=arm
    flows={n:local_flow(v,s['cog'],s['velocity'],s['omega'],s.get('disturbance',(0.,0.,0.)) if n!='fuselage' else (0.,0.,0.)) for n,v in base.items()}
    flows['left_hstab'],flows['right_hstab']=horizontal_flows([base['left_hstab'],base['right_hstab']],s['cog'],s['velocity'],s['omega'],s.get('disturbance',(0.,0.,0.)))
    flows['fuselage']=fuselage_flow(base['fuselage'],s['cog'],s['velocity'],s['omega'])
    washed,axial,swirl=propeller_tail_flow(flows['vstab'],s.get('engine_wash',(0.,0.)),p.get('slipstream_distance',5.72),s.get('torque_gyro',True))
    flows['vstab']=washed
    tail_flows=[flows['left_hstab'],flows['right_hstab']]
    if p.get('downwash_type',2)==2:
        horizontal_wash=(axial,swirl) if p.get('engine_count',1)==1 else (0.,0.)
        wake=type2_downwash(p['span'],p['area'],p['sweep'],p['taper'],p['dihedral'],wing['points'],[base['left_hstab'],base['right_hstab']],s['velocity'],s['omega'][0],*s['body_angles'],wing['cl'],s['previous_wing_cl'],s['dt'],p['downwash_coefficient'],tail_flows,p.get('travel_cap',1.),horizontal_wash,p.get('clockwise',False))
    else:
        wake=legacy_downwash(p['downwash_type'],tail_flows,washed[0],p['downwash_aspect'],p['slipstream_distance'],s['body_angles'][0],wing['cl'],s['previous_wing_cl'],(axial,swirl),p['engine_count'])
    angles=wake['angles']+[mul(f32(math.atan2(add(flows['vstab'][2],mul(f32(swirl),-.5)),flows['vstab'][0])),-DEG),mul(f32(math.atan2(flows['fuselage'][2],flows['fuselage'][0])),-DEG)]
    c=secondary_coefficients(polar,controls,angles,s['body_angles'],s['previous_angles'],p['incidence'],p['flow_inertia'],s['dt'],p['areas'],s['health'],p.get('convert_aoa',False),p.get('horizontal_bias',0.),p.get('history_scales',(1.,1.)),p.get('vertical_control_scale',1.),p.get('vertical_area_add',0.),p.get('damage_curve',((.3,.5),(1.,0.,0.,0.),(1.,0.,0.,0.),4.)),p.get('fuselage_damage_curve',((.3,.5),(1.,0.,0.,0.),(1.,0.,0.,0.),1.)),coefficient_evaluator)
    v=flows['vstab'];f=flows['fuselage'];rhohalf=mul(f32(s['density']),.5)
    qt=mul(add(add(mul(v[1],v[1]),mul(v[0],v[0])),mul(v[2],v[2])),rhohalf)
    qf=mul(rhohalf,add(add(mul(f[2],f[2]),mul(f[0],f[0])),mul(f[1],f[1])))
    c.update(points=secondary_points(base,polar,c['coefficients']),forces=secondary_forces(c['coefficients'],qt,qf,p['areas'],s['health'],p.get('vertical_area_add',0.),p.get('vertical_area_scale',1.),p.get('vertical_lift_scale',1.)),flows=flows,wake=wake,q_tail=qt,q_fuselage=qf)
    return c


def runtime_flow_inertia(configured_inertia,arm_x,configured_cog_x):
    """Loader scalar 0x1019e2aaf..2ad3; CoG is Mass.CenterOfGravity at load."""
    return mul(add(mul(sub(f32(arm_x),f32(configured_cog_x)),f32(.006306306459009647)),f32(.7)),f32(configured_inertia))


def aircraft_secondary_properties(fm,polars):
    """Selected-aircraft scalar geometry adapter; prepared Mach polars supplied."""
    from component_stages import runtime_secondary_areas
    ad=fm['Aerodynamics'];wing=ad['WingPlane'];cx=fm['Mass']['CenterOfGravity'][0]
    names=dict(hstab='HorStabPlane',vstab='VerStabPlane',fuselage='FuselagePlane')
    return dict(polars=polars,span=f32(wing['Span']),area=f32(sum(wing['Areas'][k] for k in ['LeftIn','LeftMid','LeftOut','RightIn','RightMid','RightOut'])),
                sweep=f32(wing['SweptAngle']),taper=f32(wing['TaperRatio']),dihedral=f32(wing.get('VAngle',0.)),
                downwash_coefficient=f32(wing['DownwashCoeff']),travel_cap=1.,
                engine_count=sum(1 for k in fm if k.startswith('Engine') and k[6:].isdigit()),
                arms={n:list(map(f32,ad[k]['Arm'])) for n,k in names.items()},
                incidence={n:f32(ad[k].get('Angle',0.)) for n,k in names.items()},
                flow_inertia={n:runtime_flow_inertia(ad[k].get('FlowInertia',0.),ad[k]['Arm'][0],cx) for n,k in names.items()},
                areas=runtime_secondary_areas(fm),slipstream_distance=f32(ad['VerStabPlane'].get('SlipStreamDistance',5.72)),clockwise=ad['HorStabPlane'].get('ClockWiseAOA',False))


def propeller_tail_flow(flow,wash,distance,torque_gyro=True):
    """106c60a89..60c48: double attenuation, transverse-flow suppression."""
    axial_source,swirl_source=map(f32,wash)
    if axial_source==0.:return list(flow),0.,0.
    attenuation=1./(1.+float(f32(distance))*.04)**2
    axial=axial_source*attenuation
    cross=f32(math.sqrt(add(mul(flow[2],flow[2]),mul(flow[1],flow[1]))))
    gain=max(1.+float(cross)*-1.5/max(float(flow[0])+axial,.2),0.)
    axial*=gain
    if abs(axial)<1.1920928955078125e-7:axial=0.
    out=list(flow);out[0]=f32(float(flow[0])+axial)
    swirl=0.
    if torque_gyro and swirl_source!=0.:
        gate=min(float(mul(out[0],out[0]))*.0011,1.) if out[0]>=0 else 0.
        swirl=(gain*attenuation)*(gate*swirl_source)
        if abs(swirl)<1.1920928955078125e-7:swirl=0.
    return out,axial,swirl
