import math
from component_assembly import f32,add,sub,mul
from instructor_protection import divide,inverse_cl
from instructor_pitch_predictor import prepare_geometry,deflection,RAD
from instructor_reduced import inverse_rotated_cl,tail_command,control_iteration,tail_flow,legacy_wake_factor
from polar_f32 import calc_c,calc_cl
from control_mixer import axis_limits


def roll_balance(c,ip,state,target,wing_angle,plain_limits):
    g=c['geometry'];f=state['f'];p,q=c['polars'];t=c['tail']
    areas=[add(add(s[1],s[0]),s[2]) for s in g['areas']]
    left,right=areas;tl,tr=c['tail_areas'];cg=f.get(0x5328,0.)
    positive=left>=right
    def slope(limits):
        hi,center,lo=limits[0]
        return sub(hi,center) if positive else -sub(lo,center)
    al=mul(c['ail_gain'],slope(c['limits']['Ailerons']))
    ar=mul(c['ail_gain'],slope(plain_limits['Ailerons']))
    el=slope(c['limits']['Elevator']);er=slope(plain_limits['Elevator'])
    sn=f32(math.sin(mul(g['dihedral'],RAD)))
    zl=sub(mul(c['wing_z'][0],c['cos_dihedral']),add(mul(c['wing_y'][0],sn),cg))
    zr=sub(add(mul(c['wing_y'][1],sn),mul(c['wing_z'][1],c['cos_dihedral'])),cg)
    bl=mul(p['clLineCoeff'],left);br=mul(q['clLineCoeff'],right)
    wl=mul(bl,al);wr=mul(br,ar)
    wing_sum=add(bl,br)
    wing_lever=max(add(mul(br,zr),mul(bl,zl)),f32(1e-6))
    wing_control=sub(mul(wr,zr),mul(wl,zl))
    inverse_q=divide(1.,divide(c['q_area'][0],left))
    forcing=add(add(mul(ip[0x5c],inverse_q),mul(mul(zr,right),q['cl0'])),
                mul(mul(zl,left),p['cl0']))
    tail_zl=sub(f[0x6f28],cg);tail_zr=add(f[0x6f28],cg)
    dl=mul(mul(mul(c['tail_gain'],tl),t['clLineCoeff']),el)
    dr=mul(mul(mul(c['tail_gain'],tr),t['clLineCoeff']),er)
    tail_ratio=divide(add(mul(t['clLineCoeff'],tl),mul(t['clLineCoeff'],tr)),
        max(sub(mul(mul(t['clLineCoeff'],tl),tail_zl),
                mul(mul(t['clLineCoeff'],tr),tail_zr)),f32(1e-6)))
    wing_ratio=divide(wing_sum,wing_lever)
    denominator=add(sub(sub(mul(wing_control,wing_ratio),
        mul(add(mul(dr,tail_zr),mul(dl,tail_zl)),tail_ratio)),add(wr,dr)),add(wl,dl))
    numerator=add(mul(sub(mul(sub(mul(tail_zl,tl),mul(tail_zr,tr)),tail_ratio),c['tail_area']),t['cl0']),
        add(mul(sub(target,p['cl0']),add(left,right)),
            add(mul(wing_ratio,forcing),mul(target,c['tail_area']))))
    roll=max(-1.,min(1.,divide(numerator,denominator)))
    return roll,[add(mul(al,roll),wing_angle),sub(wing_angle,mul(ar,roll))]


def autotrim_predictor(model,ip,state,history=(0.,0.,False),trace=None,*,
                       lift_relaxation=1.,lift_iterations=10,lift_bisection=False,
                       equilibrium_state=None):
    if ip[0]!=0 or ip[4]!=1. or ip[0x30]!=1. or ip[0x34]!=0.:
        raise ValueError('Mode-0 one-g horizontal auto trim only')
    c=prepare_geometry(model,ip,state);f=state['f'];flags=state['flags'];g=c['geometry']
    areas=[add(add(s[1],s[0]),s[2]) for s in g['areas']];area=add(areas[0],areas[1])
    asymmetric=(abs(ip[0x5c])>f32(.01)
        or abs(sub(*areas))>mul(area,f32(.01))
        or abs(sub(*c['tail_areas']))>mul(c['tail_area'],f32(.01))
        or abs(f.get(0x5328,0.))>f32(.1))
    plain_limits={k:axis_limits(model['controls'][k],[False]*3,ip[0x44],
        mul(f32(math.sqrt(f32(ip[0x44]/state['rho0']))),ip[0x38]))
        for k in ('Ailerons','Elevator')} if asymmetric else None
    roll=0.
    convert=flags[0x7c0a];invert=flags[0x7c54];ail=model['controls']['Ailerons'];elev=model['controls']['Elevator']
    angle,tail_force=history[:2];current=0.;unmet=0.;axial=swirl=0.
    direct=equilibrium_state is not None
    if direct:angle,tail_force,current=map(f32,equilibrium_state)
    rates=c['limits']['Elevator'][1];center=rates[1];positive=sub(rates[0],center);negative=sub(rates[2],center)
    level_scale=1. if flags[0x7c08] else divide(1.,state['balance_multiplier'])
    level_scale=mul(level_scale,c['flight_path_cos'])
    wash_speed=add(mul(ip[0x48],f32(.07)),ip[0x38]);wash_speed=max(wash_speed,1.) if wash_speed>=0. else min(wash_speed,-1.)
    washq=mul(mul(wash_speed,wash_speed),mul(ip[0x44],.5))
    sum_q_area=mul(washq,area)
    def flow(a):
        nonlocal axial,swirl
        working=sub(a,g['incidence']);sn=f32(math.sin(mul(working,RAD)));cs=f32(math.cos(mul(working,RAD)))
        vx=mul(cs,c['speed']);vy=mul(sn,-c['speed'])
        if ip[0x48]!=0.:
            wash=mul(ip[0x48],c['wash_attenuation'])
            k=max(add(f32(mul(abs(vy),-1.5)/max(add(wash,vx),f32(.2))),1.),0.)
            axial=mul(wash,k)
            if abs(axial)<f32(1.1920928955078125e-7):axial=0.
            vx=add(axial,vx)
            if ip[0x68] and not flags[0x8471]:
                spin=min(mul(mul(vx,vx),f32(.0011)),1.) if vx>=0. else 0.
                swirl=mul(mul(mul(k,c['wash_attenuation']),spin),ip[0x4c])
                if abs(swirl)<f32(1.1920928955078125e-7):swirl=0.
        return working,sn,cs,mul(mul(add(mul(vy,vy),mul(vx,vx)),.5),ip[0x44])
    for index in range(1 if direct else 10):
        command=-current if invert else current
        a=deflection([0.,command,0.],[True,False,False],c['limits']['Ailerons'])
        e=deflection([0.,command,0.],[True,False,False],c['limits']['Elevator'])
        cladd=mul(mul(a,c['ail_sens']),ail['cl'][int(a<0.)])
        bias=sub(mul(e,-elev['wing_aoa']),mul(a,c['ail_gain']))
        wing_angle=add(angle,bias)
        wing_pair=[wing_angle,wing_angle]
        if direct:
            if asymmetric:
                roll,wing_pair=roll_balance(c,ip,state,
                    mul(calc_cl(c['polars'][0],wing_angle),c['polars'][0]['clKq']),wing_angle,plain_limits)
            wing_ok=all(p['aoaCritL']<=wa<=p['aoaCritH'] for p,wa in zip(c['polars'],wing_pair))

        ad=deflection([0.,current,0.],[True,False,False],c['limits']['Ailerons'])
        drag_control=mul(mul(ad,-c['ail_gain']),add(mul(ip[0x6c],f32(-.1)),1.))
        extra=[mul(drag_control,f[0x79b8 if drag_control>=0. else 0x79bc]),
               mul(ip[0x78] if flags[0x7fa3] else 1.,f[0x7ca8]),mul(ip[0x80],f[0x7cb4]),mul(ip[0x84],f[0x7cb8])]
        other_drag=add(add(extra[3],extra[1]),add(extra[2],extra[0]))
        for moment_index in range(1 if direct else 10):
            if lift_bisection:
                low=max(p['aoaCritL'] for p in c['polars'])
                high=min(p['aoaCritH'] for p in c['polars'])
                wing_angle=low;angle=sub(wing_angle,bias);wing_ok=False
            for lift_index in range(1 if direct else lift_iterations):
                if lift_bisection:
                    wing_pair=[wing_angle,wing_angle]
                    if asymmetric:
                        roll,wing_pair=roll_balance(c,ip,state,
                            mul(calc_cl(c['polars'][0],wing_angle),c['polars'][0]['clKq']),wing_angle,plain_limits)
                working,sn,cs,dynamic=flow(angle);taq=mul(dynamic,c['tail_area'])
                vstab_drag=mul(dynamic,c['vstab_drag_factor'])
                required,tail_ok=inverse_rotated_cl(c['tail'],divide(tail_force,taq),0.,convert)
                tc=calc_c(c['tail'],required,0.)
                eng=add(mul(cs,ip[0x54]),mul(sn,ip[0x50]))
                other_normal=add(mul(add(c['parasite_drag'],vstab_drag),sn),add(mul(taq,sub(mul(cs,tc[1]),mul(sn,tc[0]))),eng))
                required_wing=sub(mul(ip[4],mul(level_scale,ip[0x2c])),mul(other_normal,ip[0x30]))
                pol=[calc_c(p,wa,working if convert else sub(wa,g['incidence'])) for p,wa in zip(c['polars'],wing_pair)]
                cls=[add(p[1],cladd) for p in pol]
                drag=[];lift=[]
                for side,p in enumerate(pol):
                    cd=add(add(add(f[0x7cc0],p[0]),mul(ip[0x7c],f[0x7cb0])),other_drag)
                    drag.append(mul(cd,c['negative_q_area'][side]))
                    lift.append(mul(mul(c['q_area'][side],cls[side]),c['cos_dihedral']))
                wings=[add(drag[1],drag[0]),add(lift[1],lift[0])]
                error=sub(required_wing,mul(add(mul(sn,wings[0]),mul(cs,wings[1])),ip[0x30]))
                if direct:break
                if lift_bisection:


                    if abs(error)<20.:
                        wing_ok=True
                        break
                    if lift_index==0:
                        low_error=error;wing_angle=high
                    elif lift_index==1:
                        if low_error*error>0.:break
                        wing_angle=f32((low+high)*.5)
                    else:
                        if low_error*error>0.:low=wing_angle;low_error=error
                        else:high=wing_angle
                        wing_angle=f32((low+high)*.5)
                    angle=sub(wing_angle,bias)
                    continue
                target=sub(divide(add(required_wing,mul(error,f32(.01)) if abs(error)>=20. else 0.),mul(mul(c['cos_dihedral'],sum_q_area),ip[0x30])),cladd)
                next_wing,wing_ok=inverse_cl(c['polars'][0],target)
                wing_angle=(next_wing if lift_relaxation==1. else
                            add(wing_angle,mul(sub(next_wing,wing_angle),f32(lift_relaxation))))
                wing_pair=[wing_angle,wing_angle]
                if asymmetric:roll,wing_pair=roll_balance(c,ip,state,target,wing_angle,plain_limits)
                angle=sub(wing_angle,bias)
                if abs(error)<20.:break

            k0=mul(add(mul(c['polars'][0]['clToCm1'],cls[0]),c['polars'][0]['clToCm0']),c['polar_area_over_span'])
            inv0=f32(1./add(mul(cls[0],cls[0]),mul(pol[0][0],pol[0][0])))
            shiftx0=mul(mul(k0,cls[0]),inv0);shifty0=mul(inv0,mul(k0,pol[0][0]))
            inv1=f32(1./add(mul(cls[1],cls[1]),mul(pol[1][0],pol[1][0])))
            k1=mul(inv1,mul(add(mul(c['polars'][1]['clToCm1'],cls[1]),c['polars'][1]['clToCm0']),c['polar_area_over_span']))
            shiftx1=mul(k1,cls[1]);shifty1=mul(k1,pol[1][0])
            mx=mul(sub(c['moment_wing_x'][0],add(shiftx0,ip[0x1c])),lift[0])
            mx=add(mx,mul(c['tail_lever'],tail_force))
            mx=add(mx,mul(sub(c['moment_wing_x'][1],add(shiftx1,ip[0x1c])),lift[1]))
            my=add(ip[0x64],add(mul(vstab_drag,c['vstab_y']),mul(c['fuse_drag'],c['fuse_y'])))
            my=add(my,mul(sub(c['moment_wing_y'][0],add(ip[0x20],shifty0)),drag[0]))
            my=add(my,mul(sub(c['moment_wing_y'][1],add(shifty1,ip[0x20])),drag[1]))
            unmet=sub(my,mx)
            if not state['new_balance'] and flags[0x7c08]:
                unmet=add(unmet,mul(f32(math.sin(mul(abs(working),RAD))),mul(add(wings[1],tail_force),.5)))
            if trace is not None:trace.append(dict(stage='balance',outer=index,moment=moment_index,lift=lift_index,angle=angle,wing_angle=wing_angle,working=working,wing_force=wings,tail_force=tail_force,error=error,unmet=unmet))
            if direct or abs(unmet)<20.:break
            tail_force=add(tail_force,mul(unmet,divide(1.,c['tail_lever'])))
        td=add(mul(positive if command>=0. else -negative,command),center)
        target=add(divide(tail_force,taq),mul(c['tail_cl'][int(td<0.)],td))
        required,tail_final_ok=inverse_rotated_cl(c['tail'],target,working if convert else 0.,convert)
        tail_drag=mul(mul(dynamic,-c['tail_area']),calc_c(c['tail'],required,working if convert else required)[0])
        flow_angle=tail_flow(downwash_type=g['downwash_type'],
            legacy_factor=legacy_wake_factor(c['wash_attenuation'],add(mul(cs,c['speed']),axial),c['legacy_aspect']) if g['downwash_type']==1 else 0.,
            span=g['span'],area=area,sweep=g['sweep'],taper=g['taper'],dihedral=g['dihedral'],
            working_alpha=working,wing_angle=angle,wing_polars=c['polars'],wing_x=c['wing_x'],wing_y=c['wing_y'],wing_z=c['wing_z'],
            tail_point=[f[o] for o in [0x6f20,0x6f24,0x6f28]],tail_polar_offset=c['tail']['aerCenterOffset'],
            coefficient=g['downwash_coefficient'],pitch_rate=0.,tail_lever=c['tail_flow_lever'],speed=c['speed'],sin_angle=sn,cos_angle=cs,
            engine_count=state['engine_count'],clockwise=flags[0x6f3c],axial_wash=axial,swirl_wash=swirl,
            tail_area_delta=sub(*c['tail_areas']),inverse_tail_area=1./c['tail_area'])
        cmd=tail_command(flow_angle=flow_angle,required_angle=required,flap_incidence=ip[0x70],tail_incidence=f[0x6f1c],
            area_sensitivity_scale=c['area_sensitivity_scale'],has_sensitivity=c['has_sensitivity'],center=center,
            positive_delta=positive,negative_delta=negative,inverted=invert)
        proposed=cmd['command']
        command_error=sub(proposed,current)
        if cmd['saturated']:
            effective=add(sub(add(ip[0x70],f[0x6f1c]),mul(cmd['deflection'],c['tail_gain'])),flow_angle)
            actual=calc_c(c['tail'],effective,working if convert else effective)[1]
            residual=add(mul(taq,sub(mul(c['tail_cl'][int(proposed<0.)],proposed),actual)),tail_force)
            unmet=add(unmet,mul(residual,c['tail_lever']))
        result=control_iteration(index,current,proposed,saturated=cmd['saturated'],
            aileron_effect_range=mul(abs(sub(c['limits']['Ailerons'][1][0],c['limits']['Ailerons'][1][2])),c['ail_gain']),
            elevon_effect_range=mul(abs(sub(rates[0],rates[2])),elev['wing_aoa']),direct_lift_authority=max(c['tail_cl']))
        if trace is not None:trace.append(dict(stage='command',outer=index,current=current,required=required,flow=flow_angle,proposed=proposed,unmet=unmet))
        if not direct:current=result['command']
        if result['done']:break
    dx=add(add(vstab_drag,wings[0]),add(c['parasite_drag'],tail_drag));ly=add(wings[1],tail_force)
    aero=[sub(mul(cs,dx),mul(sn,ly)),add(mul(cs,ly),mul(dx,sn)),0.]
    engine=[sub(mul(cs,ip[0x50]),mul(sn,ip[0x54])),add(mul(cs,ip[0x54]),mul(sn,ip[0x50])),0.]
    ok=wing_ok and tail_ok and tail_final_ok and abs(unmet)<20. and not cmd['saturated']
    if direct:
        return dict(output=[working,current,roll],success=ok,
                    equilibrium=dict(lift_error_n=error,moment_error_nm=unmet,command_error=command_error),
                    saturated=cmd['saturated'])
    return dict(output=[working,current,roll,*aero,*engine,mul(ip[0x2c],-ip[0x34]),mul(ip[0x2c],-c['flight_path_cos']),0.,unmet],
                equilibrium=dict(lift_error_n=error,moment_error_nm=unmet),
                success=ok,history=[angle,tail_force,True] if ok else list(history))
