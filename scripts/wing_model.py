"""Detailed-update wing production in the executable's stored frame.

The evaluator starts with runtime polars and processed control-mixer outputs.
It does not map pilot commands, create Mach polars, or update rigid-body state.
Finite inputs only. Ports preserve the recovered float32 operation grouping;
polar evaluation itself uses the independently checked scalar polar model.
"""
import math
from component_assembly import f32, add, sub, mul
from polar_f32 import calc_c, calc_cl
from wing_stages import coupled_points, effective_areas

EPS=f32(4e-19)
DEG=f32(57.2957763671875)
RAD=f32(0.01745329238474369)


def div(a,b):
    return f32(a/b) if abs(b)>EPS else 0.0


def lookup(points,x):
    if not points:return 0.0
    if x<=points[0][0]:return points[0][1]
    for a,b in zip(points,points[1:]):
        if x<=b[0]:return add(a[1],mul(sub(b[1],a[1]),mul(sub(x,a[0]),div(1.,sub(b[0],a[0])))))
    return points[-1][1]


def selected_geometry(fm):
    """Wing property-loader adapter. Arm.x and shift ordinates use S/b."""
    w=fm['Aerodynamics']['WingPlane'];a=w['Areas'];cfg=w['Arm']
    areas=[[f32(a[s+k]) for k in ['In','Mid','Out']]+[f32(a['Aileron'])]
           for s in ['Left','Right']]
    area=add(add(add(add(add(areas[0][1],areas[0][0]),areas[0][2]),areas[1][0]),areas[1][1]),areas[1][2])
    chord=div(area,f32(w['Span']))
    arm=[mul(f32(cfg['Arm'][0]),chord),*map(f32,cfg['Arm'][1:])]
    shifts={k:[mul(f32(x),chord) for x in cfg.get(k,[.15,0.])]
            for k in ['FlapsShift','GearShift','AirbrakesShift','ElevonShift']}
    # 1019e32c0 visits numbered rows in order and drops non-increasing knots.
    # Unnumbered float4 goes through1019e35e0 as two (knot,value) pairs;
    # an unnumbered scalar is a fallback constant.
    points=[]
    for i in range(10):
        key='AoaShiftAdd'+str(i)
        if key in cfg:
            x,y=cfg[key];x=f32(x)
            if not points or sub(x,points[-1][0])>EPS:points.append([x,mul(f32(y),chord)])
    if not points:
        value=cfg.get('AoaShiftAdd',0.)
        if isinstance(value,list) and len(value)==4:
            for x,y in [value[:2],value[2:]]:
                x=f32(x)
                if not points or sub(x,points[-1][0])>EPS:points.append([x,mul(f32(y),chord)])
        elif isinstance(value,(int,float)):points=[[0.,mul(f32(value),chord)]]
        else:raise ValueError('Unsupported AoaShiftAdd source type')
    return dict(span=f32(w['Span']),area=area,areas=areas,arm=arm,shifts=shifts,
                incidence=f32(w['Angle']),dihedral=f32(w['VAngle']),
                sine_aos=f32(cfg.get('SineAosMultiplier',1.)),v_focus=f32(cfg.get('VFocusMultiplier',0.)),
                aoa_shift=mul(f32(cfg.get('AoaShift',.6)),chord),aoa_shift_add=points,
                use_spin_loss=w.get('UseSpinLoss',False),
                spin_loss=[f32(w.get('SpinCdloss',-1.)),f32(w.get('SpinClloss',-1.))])


def base_points(g,polars,previous_aoa,beta,flaps=(0.,0.),gear=(0.,0.),
                airbrake=(0.,0.),pitch=0.,modifier_x=0.,health=None,damage_arm=(0.,0.)):
    """Live moving wing centers, 0x106c5e018..e34b / f37d..f672.

    gear and airbrake are prepared normalized/health-weighted runtime fractions.
    Angle history is the preceding wing effective AoA, including incidence.
    """
    sn=f32(math.sin(mul(f32(beta),RAD)));sv=f32(math.sin(mul(g['dihedral'],RAD)))
    result=[];health=health or [[1.]*4 for _ in range(2)]
    for i,p in enumerate(polars):
        old=f32(previous_aoa[i]);lo=p['aoaCritL'];hi=p['aoaCritH']
        excess=max(sub(lo,old),sub(old,hi))
        blend=min(1.,max(0.,min(float(excess)*.3,float(abs(old))*.0111)))
        flap=min(float(f32(flaps[i]))*3.,1.)
        x=float(add(p['aerCenterOffset'],g['arm'][0]))+float(modifier_x)
        x+=(1.-blend)*g['shifts']['FlapsShift'][0]*flap
        x+=mul(g['shifts']['AirbrakesShift'][0],f32(airbrake[i]))
        x+=mul(g['shifts']['GearShift'][0],f32(gear[i]))
        x+=g['aoa_shift']*blend
        x+=mul(abs(f32(pitch)),g['shifts']['ElevonShift'][0])
        x+=lookup(g['aoa_shift_add'],excess)
        y=float(mul(g['shifts']['AirbrakesShift'][1],f32(airbrake[i])))+g['arm'][1]
        y+=mul(g['shifts']['GearShift'][1],f32(gear[i]))
        y+=mul(f32(pitch),g['shifts']['ElevonShift'][1])
        y+=g['shifts']['FlapsShift'][1]*flap
        first=add(mul(sub(1.,f32(health[i][0])),f32(damage_arm[0])),1.)
        armz=mul(add(mul(sub(f32(health[i][2]),1.),f32(damage_arm[1])),first),g['arm'][2])
        z=mul(add(mul(g['sine_aos'],sn),f32(1.3 if i==0 else -1.3)),armz)
        result.append([-f32(x),sub(mul(mul(abs(z),sv),g['v_focus']),f32(y)),z])
    return result


def local_inputs(side,position,cog,velocity,omega,disturbance,dihedral,
                 engine_x=0.,engine_swirl=0.):
    """Wing-specific velocity construction, including two engine-flow factors."""
    v=[f32(x) for x in velocity];w=[f32(x) for x in omega]
    d=[f32(x) for x in disturbance];r=[sub(f32(x),f32(y)) for x,y in zip(position,cog)]
    sn=f32(math.sin(mul(f32(dihedral),RAD)));cs=f32(math.cos(mul(f32(dihedral),RAD)))
    k=.07000000029802322 if side==0 else .07
    ux=f32(float(add(v[0],d[0]))+float(engine_x)*k)
    y,z=add(v[1],d[1]),add(v[2],d[2])
    uy=sub(mul(cs,y),mul(sn,z)) if side==0 else add(mul(cs,y),mul(sn,z))
    rz=sub(mul(r[0],w[1]),mul(w[0],r[1]))
    uz=add(add(rz,mul(sn,y)),mul(cs,z)) if side==0 else add(rz,sub(mul(cs,z),mul(sn,y)))
    flow=[add(ux,sub(mul(r[1],w[2]),mul(r[2],w[1]))),
          add(uy,sub(mul(r[2],w[0]),mul(r[0],w[2]))),
          uz]
    swirl=float(engine_swirl)*k*(1 if side==0 else -1)
    local_y=f32(float(flow[1])+swirl)
    reference_y=f32(float(uy)+float(engine_swirl)*.07000000029802322*(1 if side==0 else -1))
    alpha=mul(f32(math.atan2(local_y,flow[0])),-DEG) if flow[0] or local_y else -0.
    alpha_ref=mul(f32(math.atan2(reference_y,ux)),DEG) if ux or reference_y else 0.
    speed2=add(mul(flow[1],flow[1]),add(mul(flow[2],flow[2]),mul(flow[0],flow[0])))
    return dict(flow=flow,alpha=alpha,reference_angle=alpha_ref,speed_squared=speed2)


def coefficients(p,flow,control,incidence,flap,health=1.,body_aoa=0.,convert_aoa=False,cd_factor=1.,extra_cd=0.):
    """Three native polar calls followed by additive CL/Cx controls.

    Mixer CL is added AFTER coefficient rotation here, unlike horizontal tail.
    The reference angle excludes rotational velocity but retains engine flow.
    """
    control=[f32(x) for x in control];health=f32(health)
    shift=mul(mul(add(control[4],control[1]),health),add(mul(f32(flap),f32(-.1)),1.))
    rotation=add(flow['alpha'],shift)
    angle=add(rotation,f32(incidence))
    reference=add(sub(shift,flow['reference_angle']),f32(incidence))
    rot=f32(body_aoa) if convert_aoa else rotation
    normal=[f32(x) for x in calc_c(p,angle,rot,0.,cd_factor)]
    ref=[f32(x) for x in calc_c(p,reference,sub(reference,f32(incidence)),0.,cd_factor)]
    dragrot=f32(body_aoa) if convert_aoa else sub(angle,f32(incidence))
    drag=[f32(x) for x in calc_c(p,angle,dragrot,0.,cd_factor)]
    if isinstance(extra_cd,dict):
        e=extra_cd
        local=add(e['gear'],mul(control[3],health))
        common=add(e['radiator'],e['central_gear'])
        common=add(e['oil_radiator'],common)
        total=add(local,common)
        total=add(total,e['fuselage'])
        total=add(e['airbrake'],total)
        total=add(total,e['bomb_bay'])
    else:total=add(mul(control[3],health),f32(extra_cd))
    cdadd=mul(total,p['kq'])
    return dict(cx=add(drag[0],cdadd),cy=normal[1],cy_reference=ref[1],
                cy_add=mul(control[2],health),aoa=angle,reference_aoa=reference,
                raw_cl=f32(calc_cl(p,angle)))


def quantized_fraction(x):
    """Actuated device fraction is truncated to an 8-bit value in this caller."""
    return mul(float(min(255,max(0,int(mul(f32(x),255.))))),f32(1/255))


def drag_terms(fm,gear_position=0.,airbrake_position=0.,bomb_bay_position=0.,
               airbrake_health=(1.,1.),gear_present=(True,True),central_gear_present=True,
               gear_animated=(True,True),central_gear_animated=True,radiator=0.,oil_radiator=0.):
    """Aerodynamics device drag assigned to each wing before q and area.

    Animation flags are actual component configuration flags from FM owner
    +c4/+134/+1a4. A non-animated present gear contributes its full configured Cd.
    Radiator values are owner-helper outputs (engine averages), not throttle.
    """
    a=fm['Aerodynamics'];gear=quantized_fraction(gear_position)
    air_count=float(min(255,max(0,int(mul(f32(airbrake_position),255.)))))
    door_count=float(min(255,max(0,int(mul(f32(bomb_bay_position),255.)))))
    result=[]
    for i in range(2):
        air=mul(mul(f32(airbrake_health[i]),f32(1/255)),air_count)
        result.append(dict(gear=mul(f32(a.get('GearCd',0.)) if gear_present[i] else 0.,gear if gear_animated[i] else 1.),
                           central_gear=mul(f32(a.get('GearCentralCd',0.)) if central_gear_present else 0.,gear if central_gear_animated else 1.),
                           radiator=mul(f32(a.get('RadiatorCd',0.)),f32(radiator)),
                           oil_radiator=mul(f32(a.get('OilRadiatorCd',0.)),f32(oil_radiator)),
                           fuselage=f32(a.get('FuseCd',0.)),
                           airbrake=mul(air,f32(a.get('AirbrakeCd',0.))),
                           bomb_bay=mul(mul(f32(a.get('BombBayCd',0.)),f32(1/255)),door_count)))
    return result


def postprocess_xy(xy,aoa,crit,cy_add,q,body_q,dt,height,ground_length,
                   ordering=True,spin=0.,spin_increment=.2,spin_cap_input=0.,
                   rudder_health=1.,yaw_command=0.,yaw_rate=0.,
                   use_spin_loss=False,spin_agl=0.,spin_loss=(0.,0.)):
    """All branches from 0x106c605a1 / 64e16 through 606ca.

    ordering selects the caller branch, not a game-mode name. The opposite
    branch implements positive-stall/spin correction and tail scale outputs.
    """
    xy=[list(x) for x in xy];spin=f32(spin)
    gyroscopic_scale=1.;vertical_scale=1.;vertical_control_scale=1.
    if ordering:
        l=sub(xy[0][1],mul(f32(cy_add[0]),f32(q[0])))
        r=sub(xy[1][1],mul(f32(cy_add[1]),f32(q[1])))
        swap=(l<r) if aoa[0]>aoa[1] else (l>r)
        if swap:xy[0][1],xy[1][1]=xy[1][1],xy[0][1]
        spin=0.
    else:
        threshold=mul(add(f32(crit[0]),f32(crit[1])),.5)
        maxaoa=max(aoa)
        if maxaoa<=threshold:spin=0.
        else:
            delta=sub(maxaoa,threshold)
            growth=min(1.,max(0.,mul(delta,f32(.1))))
            cap=add(mul(f32(spin_cap_input),f32(.4)),f32(1.2))
            spin=min(add(spin,mul(mul(f32(spin_increment),f32(dt)),growth)),cap)
            if rudder_health>0. and abs(yaw_command)>.5 and float(f32(yaw_command))*float(yaw_rate)>0.:
                spin=max(0.,add(spin,mul(mul(f32(-.3),f32(dt)),growth)))
            d=min(15.,max(1.,mul(delta,f32(.8))))
            gyroscopic_scale=max(1.-float(delta)*.2,.2)/float(d)
            vertical_control_scale=add(mul(d,f32(.035)),1.)
            upper=mul(add(f32(crit[0]),f32(crit[1])),.75)
            if maxaoa>=upper:vertical_scale=f32(.3)
            else:vertical_scale=add(1.,div(mul(sub(maxaoa,threshold),sub(f32(.3),1.)),sub(upper,threshold)))
            if use_spin_loss:
                s=float(mul(mul(f32(body_q),d),spin))*float(min(1.,max(0.,mul(f32(spin_agl),f32(.1)))))
                positive=[s*.019999999552965164*float(f32(v)) for v in spin_loss]
                negative=[s*k*float(f32(v)) for k,v in zip([.25,.10000000149011612],spin_loss)]
                high,low=(0,1) if aoa[0]>aoa[1] else (1,0)
                xy[low]=[f32(v+a) for v,a in zip(xy[low],positive)]
                xy[high]=[f32(v-a) for v,a in zip(xy[high],negative)]
    h=max(float(height),0.);limit=float(f32(ground_length))*.4
    if h<limit:
        gain=(h*-.2)/limit+1.2
        xy=[[f32(float(v)*gain) for v in p] for p in xy]
    return dict(xy=xy,spin=spin,gyroscopic_scale=gyroscopic_scale,
                vertical_area_scale=vertical_scale,vertical_control_scale=vertical_control_scale)


def evaluate(g,polars,velocity,omega,cog,controls,density,dt,previous_aoa,
             beta=0.,body_aoa=0.,flaps=(0.,0.),health=None,disturbance=None,
             gear=(0.,0.),airbrake=(0.,0.),pitch=0.,modifier_x=0.,damage_arm=(0.,0.),
             engine_x=0.,engine_swirl=0.,cd_factor=(1.,1.),extra_cd=(0.,0.),
             convert_aoa=False,inertia_x=1.,height=1e6,ground_length=0.,
             ordering=True,spin=0.,spin_increment=.2,spin_cap_input=0.,
             rudder_health=1.,yaw_command=0.,spin_agl=0.):
    """Compose wing geometry, local flow, polars, points and final forces.

    Coefficients, shifts and force corrections are all explicit. Returned tail
    scales are consumed by other component branches; do not silently drop them.
    """
    health=health or [[1.]*4 for _ in range(2)]
    disturbance=disturbance or [[0.]*3 for _ in range(2)]
    base=base_points(g,polars,previous_aoa,beta,flaps,gear,airbrake,pitch,modifier_x,health,damage_arm)
    flows=[local_inputs(i,base[i],cog,velocity,omega,disturbance[i],g['dihedral'],engine_x,engine_swirl) for i in range(2)]
    c=[coefficients(polars[i],flows[i],controls[i],g['incidence'],flaps[i],health[i][3],body_aoa,convert_aoa,cd_factor[i],extra_cd[i]) for i in range(2)]
    areas=effective_areas(g['areas'],health)
    q=[mul(v['speed_squared'],mul(f32(density),.5)) for v in flows]
    sv=f32(math.sin(mul(g['dihedral'],RAD)));cv=f32(math.cos(mul(g['dihedral'],RAD)))
    sb=f32(math.sin(mul(f32(beta),RAD)));cb=abs(f32(math.cos(mul(f32(beta),RAD))))
    weights=[div(mul(mul(mul(q[i],areas[i]),cv),base[i][2]),f32(inertia_x)) for i in range(2)]
    coupled=coupled_points([x['cx'] for x in c],[x['cy'] for x in c],[x['cy_reference'] for x in c],
                           [x['cy_add'] for x in c],weights,f32(dt),omega[0],g['area'],g['span'],base,
                           [[p['clToCm0'],p['clToCm1']] for p in polars])
    drag=[mul(-q[i],c[i]['cx']) for i in range(2)]
    lift=[mul(coupled['cy'][i],q[i]) for i in range(2)]
    xy=[[mul(drag[i],cb),mul(lift[i],cv)] for i in range(2)]
    bv=list(map(f32,velocity));body_q=mul(add(add(mul(bv[0],bv[0]),mul(bv[1],bv[1])),mul(bv[2],bv[2])),mul(f32(density),.5))
    corrected=postprocess_xy(xy,[v['aoa'] for v in c],[p['aoaCritH'] for p in polars],
                             [v['cy_add'] for v in c],q,body_q,dt,height,ground_length,
                             ordering,spin,spin_increment,spin_cap_input,rudder_health,yaw_command,
                             omega[1],g['use_spin_loss'],spin_agl,g['spin_loss'])
    z=[sub(mul(drag[0],sb),mul(lift[0],sv)),add(mul(lift[1],sv),mul(drag[1],sb))]
    forces=[[mul(v,areas[i]) for v in corrected['xy'][i]+[z[i]]] for i in range(2)]
    return dict(forces=forces,points=coupled['points'],base_points=base,flow=flows,coefficients=c,
                q=q,areas=areas,blend=coupled['blend'],cy=coupled['cy'],postprocess=corrected)
