"""Independent mode-1 Instructor predictor, 101a5cac0.

Research port for the intact, fixed-wing keyboard path: input mode=1,
input+c=0 and input+28=0. It consumes source FM properties/state and the
wrapper's input struct; no native stack intermediate is an input. The exposed
preparation dictionary uses stack offsets to support instruction-level audits.
This is the Instructor's approximation, not the detailed aerodynamic model.
"""
import math
import struct
from component_assembly import f32,add,sub,mul
from control_mixer import axis_limits,curve
from polar_runtime import evaluate as mach_polar,flap_polar
from polar_f32 import calc_c
from instructor_protection import divide
from instructor_reduced import inverse_rotated_cl,pitch_balance,tail_command,control_iteration,tail_flow,legacy_wake_factor
from wing_sweep import select

RAD=f32(.01745329238474369)
EPS=f32(4e-19)


def deflection(commands,inverted,limits):
    """1019e5cf0: prepared control-table evaluation, without aero scaling."""
    values=[]
    for command,inv,(positive,center,negative) in zip(commands,inverted,limits):
        slope=sub(center,positive) if command>=0. else sub(negative,center)
        value=sub(center,mul(slope,command))
        values.append(-value if inv else value)
    return add(values[2],add(values[1],values[0]))


def unpack_inputs(data):
    values={off:struct.unpack_from('<f',data,off)[0] for off in range(4,0x8c,4)}
    values.update({off:bool(data[off]) for off in [0xc,0x28,0x68]})
    values[0]=struct.unpack_from('<I',data)[0]
    return values


def prepare_geometry(model,ip,state):
    """Prepare geometry/polars/tables from source fields, no native locals.

    state.f contains the documented runtime FM scalar offsets read by this
    function. State is intact; normalization fields and auxiliary drag remain
    explicit because they are runtime values, not necessarily datamine values.
    """
    if ip[0] not in (0,1) or ip[0xc] or ip[0x28]:
        raise ValueError('Only intact mode-0/1 predictor with prescribed pitch rate')
    # The nested predictor iterations change history, not these prepared
    # inputs. Keep exact input bits/values: no Mach, speed or angle rounding.
    # Returned geometry/coefficient dictionaries are read-only to predictors.
    # Forces, demanded acceleration and predictor histories do not enter
    # this preparation. Key every input actually read below, so changing a
    # command does not rebuild identical Mach polars and geometry each tick.
    key=(tuple(ip[k] for k in (0x34,0x38,0x40,0x44,0x48,0x6c,0x74,0x78,0x7c,0x88)),tuple(sorted(state['f'].items())),
         tuple(sorted(state['flags'].items())),state['balance_multiplier'],state['rho0'])
    memo=model.setdefault('_instructor_geometry_cache',{})
    if key in memo:return memo[key]
    f=state['f'];flags=state['flags'];c={}
    wing=select(model['wing_family'],ip[0x74]);g=wing['geometry']
    if g['downwash_type'] not in (0,1,2):raise ValueError('Unknown reduced downwash type')
    flap=ip[0x6c] if flags[0x7fa6] else 0.
    factor=state['balance_multiplier'] if flags[0x7c08] else 1.
    flap_key=(ip[0x74],flap)
    flap_cache=model.setdefault('_predictor_flap_cache',{})
    if flap_key not in flap_cache:
        if len(flap_cache)>=64:flap_cache.clear()
        flap_cache[flap_key]=flap_polar(wing['polars'],flap)
    runtime=flap_cache[flap_key]
    polars=[mach_polar(runtime,ip[0x40],factor) for _ in range(2)]
    for p,off in zip(polars,[0x843c,0x8440]):p['indCoeff']=mul(divide(1.,f[off]),p['indCoeff'])
    tail=mach_polar(model['polars']['HorStabPlane'][0][1],ip[0x40])
    fuselage=mach_polar(model['polars']['FuselagePlane'][0][1],ip[0x40])
    vstab=mach_polar(model['polars']['VerStabPlane'][0][1],ip[0x40])
    areas=[add(add(side[1],side[0]),side[2]) for side in g['areas']]
    tail_areas=[add(f[0x8424],f[0x8420]),add(f[0x8428],f[0x842c])]
    elevator_area=add(f[0x842c],f[0x8424])
    tail_area=add(tail_areas[1],tail_areas[0])
    sn=f32(math.sin(mul(g['dihedral'],RAD)));cs=f32(math.cos(mul(g['dihedral'],RAD)))
    flap_shift=min(mul(ip[0x6c],3.),1.) if flags[0x7fa6] else 0.
    x=add(mul(g['shifts']['FlapsShift'][0],flap_shift),g['arm'][0])
    y=add(mul(flap_shift,g['shifts']['FlapsShift'][1]),g['arm'][1])
    dx=add(mul(g['shifts']['GearShift'][0],ip[0x78]),mul(g['shifts']['AirbrakesShift'][0],ip[0x7c]))
    dy=add(mul(ip[0x7c],g['shifts']['AirbrakesShift'][1]),mul(ip[0x78],g['shifts']['GearShift'][1]))
    # The left and right expressions have different addition groupings.
    wx=[add(add(polars[0]['aerCenterOffset'],x),dx),add(add(x,dx),polars[1]['aerCenterOffset'])]
    wz=[mul(g['arm'][2],f32(1.3)),mul(f32(-1.3),g['arm'][2])]
    wy=[sub(mul(mul(g['v_focus'],sn),abs(wz[0])),add(y,dy)),
        sub(mul(abs(wz[1]),mul(g['v_focus'],sn)),add(y,dy))]
    lever=add(sub(f[0x6f20],f[0x5320]),tail['aerCenterOffset'])
    speed=max(ip[0x38],1.) if ip[0x38]>=0. else min(ip[0x38],-1.)
    speed2=mul(speed,speed);q=mul(mul(.5,speed2),ip[0x44])
    wash_speed=add(mul(ip[0x48],f32(.07)),ip[0x38])
    wash_speed=max(wash_speed,1.) if wash_speed>=0. else min(wash_speed,-1.)
    washq=mul(mul(wash_speed,wash_speed),mul(ip[0x44],.5))
    fuse_drag=mul(mul(mul(mul(ip[0x44],calc_c(fuselage,0.,0.)[0]),mul(speed2,-.5)),f[0x18a0]),f[0x83fc])
    vstab_area=add(mul(f[0x18d4],mul(f[0x8430],1.5)),f32(.2))
    vstab_area=add(mul(f[0x8434],f[0x18d8]),vstab_area)
    vstab_drag=mul(q,mul(vstab_area,-calc_c(vstab,0.,0.)[0]))
    tail_sens=curve(model['controls']['Elevator']['sensitivity_curve'],ip[0x40],1)[0]
    ail_sens=curve(model['controls']['Ailerons']['sensitivity_curve'],ip[0x40],1)[0]
    elevator=model['controls']['Elevator'];aileron=model['controls']['Ailerons']
    tail_gain=mul(elevator['sensitivity'],tail_sens)
    sensitivity_den=mul(max(elevator_area,f32(.01)),tail_gain)
    ias=mul(f32(math.sqrt(f32(ip[0x44]/state['rho0']))),ip[0x38])
    limits={k:axis_limits(model['controls'][k],[True,False,False],ip[0x44],ias) for k in ['Ailerons','Elevator']}
    c.update(polars=polars,tail=tail,geometry=g,limits=limits,legacy_aspect=runtime['base'][0],
             speed=speed,wing_x=wx,wing_y=wy,wing_z=wz,tail_areas=tail_areas,
             tail_gain=tail_gain,tail_cl=[mul(x,tail_sens) for x in elevator['cl']],
             ail_gain=mul(aileron['sensitivity'],ail_sens),ail_sens=-ail_sens,
             tail_area=tail_area,tail_flow_lever=lever,
             tail_lever=sub(lever,tail['clToCm1']),
             area_sensitivity_scale=mul(elevator_area,f32(1./sensitivity_den)) if sensitivity_den else float('inf'),
             has_sensitivity=abs(sensitivity_den)>EPS,flight_path_cos=f32(math.sqrt(sub(1.,mul(ip[0x34],ip[0x34])))),
             q_area=[mul(washq,a) for a in areas],negative_q_area=[mul(-a,washq) for a in areas],
             cos_dihedral=cs,polar_area_over_span=f32(f[0x8438]/g['span']),
             moment_wing_x=[-add(f[0x5320],v) for v in wx],moment_wing_y=[sub(v,f[0x5324]) for v in wy],
             fuse_drag=fuse_drag,vstab_drag=vstab_drag,vstab_drag_factor=mul(vstab_area,-calc_c(vstab,0.,0.)[0]),
             fuse_y=sub(f[0x7334],f[0x5324]),vstab_y=sub(f[0x712c],f[0x5324]),
             parasite_drag=add(sub(mul(ip[0x88],f[0x7cc4]),mul(f[0x8448],washq)),fuse_drag),
             wash_attenuation=f32(1./mul(add(mul(f[0x7140],f32(.04)),1.),add(mul(f[0x7140],f32(.04)),1.))))
    if len(memo)>=512:memo.clear()
    memo[key]=c
    return c


def prepare_mode1(model,ip,state):
    if ip[0]!=1:raise ValueError('Expected mode 1')
    return prepare_geometry(model,ip,state)


def pitch_predictor(model,ip,state,history=(0.,0.,False),trace=None):
    """Return all 13 native output floats, success and conditionally updated history.

    Original mode-1 call sites can ignore the success boolean and still use the
    returned pitch command. Last-pass relaxation is retained, including forces
    evaluated at the preceding iterate. No alternative equilibrium root is used.
    """
    c=prepare_mode1(model,ip,state);f=state['f'];flags=state['flags'];g=c['geometry']
    convert=flags[0x7c0a];invert=flags[0x7c54];ail=model['controls']['Ailerons'];elev=model['controls']['Elevator']
    current=0.;unmet=0.;working=sub(ip[4],g['incidence'])
    sn=f32(math.sin(mul(working,RAD)));cs=f32(math.cos(mul(working,RAD)))
    vx=mul(cs,c['speed']);vy=mul(sn,-c['speed']);axial=swirl=0.
    qvx=vx
    if ip[0x48]!=0.:
        wash=mul(ip[0x48],c['wash_attenuation'])
        k=max(add(f32(mul(abs(vy),-1.5)/max(add(wash,vx),f32(.2))),1.),0.)
        axial=mul(wash,k)
        if abs(axial)<f32(1.1920928955078125e-7):axial=0.
        qvx=add(vx,ip[0x48])  # Deliberately unattenuated in this mode-1 branch.
        if ip[0x68] and not flags[0x8471]:
            spin=min(mul(mul(qvx,qvx),f32(.0011)),1.) if qvx>=0. else 0.
            swirl=mul(mul(mul(k,c['wash_attenuation']),spin),ip[0x4c])
            if abs(swirl)<f32(1.1920928955078125e-7):swirl=0.
    dynamic=mul(mul(add(mul(vy,vy),mul(qvx,qvx)),.5),ip[0x44])
    taq=mul(dynamic,c['tail_area'])
    rates=c['limits']['Elevator'][1];center=rates[1];positive=sub(rates[0],center);negative=sub(rates[2],center)
    products=[mul(ip[0x78] if flags[0x7fa3] else 1.,f[0x7ca8]),mul(ip[0x80],f[0x7cb4]),
              mul(ip[0x84],f[0x7cb8]),mul(ip[0x7c],f[0x7cb0])]
    extra=add(add(products[3],products[1]),add(products[2],products[0]))
    vstab_drag=c['vstab_drag']
    for index in range(10):
        command=-current if invert else current
        a=deflection([0.,command,0.],[True,False,False],c['limits']['Ailerons'])
        e=deflection([0.,command,0.],[True,False,False],c['limits']['Elevator'])
        cladd=mul(mul(a,c['ail_sens']),ail['cl'][int(a<0.)])
        bias=sub(mul(e,-elev['wing_aoa']),mul(a,c['ail_gain']))
        angle=add(bias,ip[4])
        left=calc_c(c['polars'][0],angle,working if convert else sub(angle,g['incidence']))
        # Native right wing always rotates by its deflected angle here.
        right=calc_c(c['polars'][1],angle,sub(angle,g['incidence']))
        other=add(mul(sub(c['fuse_y'],ip[0x20]),c['fuse_drag']),mul(vstab_drag,c['vstab_y']))
        balance_inputs=dict(cd=[left[0],right[0]],cl=[left[1],right[1]],control_cl=cladd,
            extra_cd=[extra,extra],fuselage_cd=f[0x7cc0],q_area=c['q_area'],negative_q_area=c['negative_q_area'],
            cos_dihedral=c['cos_dihedral'],cm0=[p['clToCm0'] for p in c['polars']],cm1=[p['clToCm1'] for p in c['polars']],
            polar_area_over_span=c['polar_area_over_span'],wing_x=c['moment_wing_x'],wing_y=c['moment_wing_y'],
            other_moment=other,engine_pitch_moment=ip[0x64],pitch_inertia=state['pitch_inertia'],target_acceleration=ip[8],
            weight=ip[0x2c],flight_path_cos=c['flight_path_cos'],reference_x=ip[0x1c],tail_lever=c['tail_lever'],working_alpha=working,
            legacy_balance_shift=not state['new_balance'] and flags[0x7c08])
        balance=pitch_balance(**balance_inputs)
        # 101a5f655: updated only AFTER this pass's moment balance. The next
        # pass therefore consumes the new flow pressure, not the initial one.
        vstab_drag=mul(dynamic,c['vstab_drag_factor'])
        td=add(mul(positive if command>=0. else -negative,command),center)
        target=add(divide(balance['tail_force'],taq),mul(c['tail_cl'][int(td<0.)],td))
        required,converged=inverse_rotated_cl(c['tail'],target,working if convert else 0.,convert)
        required=min(max(required,ip[0x14]),ip[0x18])
        tail_cd=calc_c(c['tail'],required,working if convert else required)[0]
        tail_drag=mul(mul(dynamic,-c['tail_area']),tail_cd)
        flow=tail_flow(downwash_type=g['downwash_type'],
            legacy_factor=legacy_wake_factor(c['wash_attenuation'],qvx,c['legacy_aspect']) if g['downwash_type']==1 else 0.,
            span=g['span'],area=add(*reversed([add(add(s[1],s[0]),s[2]) for s in g['areas']])),
            sweep=g['sweep'],taper=g['taper'],dihedral=g['dihedral'],working_alpha=working,wing_angle=ip[4],
            wing_polars=c['polars'],wing_x=c['wing_x'],wing_y=c['wing_y'],wing_z=c['wing_z'],
            tail_point=[f[o] for o in [0x6f20,0x6f24,0x6f28]],tail_polar_offset=c['tail']['aerCenterOffset'],
            coefficient=g['downwash_coefficient'],pitch_rate=ip[0x10],tail_lever=c['tail_flow_lever'],speed=c['speed'],sin_angle=sn,cos_angle=cs,
            engine_count=state['engine_count'],clockwise=flags[0x6f3c],axial_wash=axial,swirl_wash=swirl,
            tail_area_delta=sub(*c['tail_areas']),inverse_tail_area=1./c['tail_area'])
        command_result=tail_command(flow_angle=flow,required_angle=required,flap_incidence=ip[0x70],tail_incidence=f[0x6f1c],
            area_sensitivity_scale=c['area_sensitivity_scale'],has_sensitivity=c['has_sensitivity'],center=center,
            positive_delta=positive,negative_delta=negative,inverted=invert)
        proposed=command_result['command']
        if command_result['saturated']:
            incidence=add(ip[0x70],f[0x6f1c])
            effective=add(sub(incidence,mul(command_result['deflection'],c['tail_gain'])),flow)
            actual_cl=calc_c(c['tail'],effective,working if convert else effective)[1]
            residual=add(mul(taq,sub(mul(c['tail_cl'][int(proposed<0.)],proposed),actual_cl)),balance['tail_force'])
            unmet=add(unmet,mul(residual,c['tail_lever']))
        result=control_iteration(index,current,proposed,saturated=command_result['saturated'],
            aileron_effect_range=mul(abs(sub(c['limits']['Ailerons'][1][0],c['limits']['Ailerons'][1][2])),c['ail_gain']),
            elevon_effect_range=mul(abs(sub(rates[0],rates[2])),elev['wing_aoa']),direct_lift_authority=max(c['tail_cl']))
        if trace is not None:trace.append(dict(index=index,current=current,angle=angle,wing=balance['wing_force'],tail=balance['tail_force'],
            target_cl=target,required=required,flow=flow,command=proposed,done=result['done'],unmet=unmet,
            tail_drag=tail_drag,parasite_drag=c['parasite_drag'],vstab_drag=vstab_drag,balance_inputs=balance_inputs))
        current=result['command']
        if result['done']:break
    dx=add(add(vstab_drag,balance['wing_force'][0]),add(c['parasite_drag'],tail_drag))
    ly=add(balance['wing_force'][1],balance['tail_force'])
    aero=[sub(mul(cs,dx),mul(sn,ly)),add(mul(cs,ly),mul(dx,sn)),0.]
    engine=[sub(mul(cs,ip[0x50]),mul(sn,ip[0x54])),add(mul(cs,ip[0x54]),mul(sn,ip[0x50])),0.]
    weight=mul(ip[4],ip[0x2c])
    auxiliary=[mul(weight,-ip[0x34]),mul(weight,-c['flight_path_cos']),0.]
    ok=converged and abs(unmet)<20. and not command_result['saturated']
    return dict(output=[working,current,0.,*aero,*engine,*auxiliary,unmet],success=ok,
                history=[ip[4],balance['tail_force'],True] if ok else list(history))
