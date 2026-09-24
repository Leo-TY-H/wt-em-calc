"""Composition of recovered clean F-16/JAS39C force and moment equations.

Shared Realistic/Simulator physics; no instructor, contact or damaged-airframe
branches. Commands are the final FM1694/98/9c values, AFTER the separately
recovered stick/trim/authority/actuator stages. Histories and oil-radiator state
are explicit. This composition is not itself a full-native-timestep oracle.
"""
from component_assembly import f32,add,mul,assemble_force,assemble_moment
from control_mixer import selected_aircraft_properties,prepare_rows,curve,mix
from polar_runtime import make_runtime,flap_polar,evaluate as mach_polar
from air_state import cache
from wing_model import selected_geometry,evaluate as wings,drag_terms,quantized_fraction
from tail_model import aircraft_secondary_properties,secondary_model
from body_dynamics import preprocess_angular_rate,parasite_force,limit_aerodynamic_force,limit_total_moment,accumulate_nozzle,compose_force,realistic_engine_scale,gyroscopic_moment
from wing_area_normalization import intact_polars
from jet_model import prepare as prepare_jet,steady
from wing_sweep import prepare as prepare_wings,select as select_wing
from aero_helpers import roll_leveling


def prepare(fm):
    family=prepare_wings(fm);geometry=family[0][1]['geometry']
    if 'WingPlane' not in fm['Aerodynamics']:
        fm=dict(fm,Aerodynamics=dict(fm['Aerodynamics'],WingPlane=fm['Aerodynamics']['WingPlaneSweep0']))
    ad=fm['Aerodynamics'];polars={'WingPlane':family[0][1]['polars']}
    for name in ['HorStabPlane','VerStabPlane','FuselagePlane']:
        plane=ad[name];areas=plane['Areas'];span=f32(plane['Span'])
        if name=='VerStabPlane':
            # 1019e2de0: polar preparation differs from the force-area formula.
            span=mul(span,f32(1.4142135381698608));area=add(add(mul(f32(areas['Rudder']),1.5),f32(areas['Main'])),f32(.2))
        else:area=f32(sum(map(f32,areas.values())))
        family=[]
        for key,p in plane.items():
            if key=='Polar' or key.startswith('FlapsPolar'):
                family.append((f32(p.get('Flaps',0.)),make_runtime(p,span,area)))
        polars[name]=sorted(family,key=lambda x:x[0])
    flap_rows=[]
    for i in range(16):
        block=ad.get('Flaps'+str(i))
        if block:flap_rows.append((block['Flaps'],[block[k] for k in ['FlapsPolarBlending','FlapsAnimation','Stab','Slats']]))
    return dict(fm=fm,geometry=geometry,polars=polars,wing_family=prepare_wings(fm),sweep=0.,flaps=prepare_rows(flap_rows),
                controls={k:selected_aircraft_properties(ad[k]) for k in ['Ailerons','Elevator','Rudder']},
                engine=prepare_jet(fm['EngineType0']['Main']) if fm['EngineType0']['Main']['Type']=='Jet' else None)


def at_sweep(model,position):
    """An independent condition cache for each prepared sweep, shared constants."""
    position=f32(position);memo=model.setdefault('_sweep_cache',{})
    if position not in memo:
        if len(memo)>=256:memo.clear()
        wing=select_wing(model['wing_family'],position)
        memo[position]={k:v for k,v in model.items() if not k.startswith('_')}
        memo[position].update(sweep=position,geometry=wing['geometry'],polars=dict(model['polars'],WingPlane=wing['polars']))
    return memo[position]


def condition_properties(model,mach,flaps):
    """Cache immutable prepared coefficients at their exact native inputs.

    No quantization: neighboring float32 Mach values retain separate entries.
    Callers copy the secondary-property outer dictionary before adding state.
    """
    key=(mach,f32(flaps));memo=model.setdefault('_condition_cache',{})
    if key not in memo:
        if len(memo)>=256:memo.clear()
        values=curve(model['flaps'],key[1],4) if model['flaps'] else [key[1],key[1],0.,0.]
        wing_runtime=flap_polar(model['polars']['WingPlane'],values[0])
        wing=mach_polar(wing_runtime,mach)
        secondary={n:mach_polar(model['polars'][k][0][1],mach) for n,k in
                   [('hstab','HorStabPlane'),('vstab','VerStabPlane'),('fuselage','FuselagePlane')]}
        tp=aircraft_secondary_properties(model['fm'],secondary);g=model['geometry']
        tp.update({k:g[k] for k in ['span','area','sweep','taper','dihedral','downwash_coefficient','downwash_type']})
        tp['downwash_aspect']=wing_runtime['base'][0]
        memo[key]=(values,wing,tp)
    return memo[key]


def replay_propulsion(model, result, mass_state, engine_vectors, engine_angular_momentum,torque_gyro=True):
    """Reassemble an EM phase with exactly identical aero inputs and history.

    The caller must match the complete flight condition, controls, propwash
    and incoming aerodynamic history. EM uses zero external forces/moments
    and unit external thrust multiplier. Keep every native assembly rounding
    and moment clamp; never average propulsion before this consumer.
    """
    mass=mass_state['mass']
    engine_force,engine_moment=[list(map(float,v)) for v in engine_vectors]
    scale=realistic_engine_scale(1.,model['fm'].get('ExtThrustBaseMult',1.))
    force=compose_force(limit_aerodynamic_force(result['raw_aero_force'],mass),
                        (0.,0.,0.),engine_force,scale)
    moment=[((result['raw_aero_moment'][i]+result['helper_moment'][i])+0.)+engine_moment[i] for i in range(3)]
    gyro=gyroscopic_moment(engine_angular_momentum,result['omega_for_flow'],result['wing']['postprocess']['gyroscopic_scale']) if torque_gyro else [0.,0.,0.]
    moment=limit_total_moment([x+y for x,y in zip(moment,gyro)],mass)
    return dict(result,force=force,stored_moment=moment,engine_force=engine_force,engine_moment=engine_moment)


def evaluate(model,velocity,stored_omega,mass_state,commands,height,dt,history,
             *,oil_radiator,flaps=0.,throttle=1.1,gear=0.,airbrake=0.,cockpit_door=0.,
             attachment_drag_area=0.,external_force=(0.,0.,0.),external_moment=(0.,0.,0.),
             ext_thrust_mult=1.,height_agl=1e6,ground_effect_height=None,scalar_thrust=None,engine_spin_factor=None,
             engine_vectors=None,quaternion=(0.,0.,0.,1.),engine_wash=(0.,0.),torque_gyro=True,engine_angular_momentum=(0.,0.,0.)):
    """Return every component, total non-gravitational body F and stored M.

    velocity is cached body air velocity (float32 values promoted to double).
    mass_state is mass_model.evaluate output. Required history has wing_aoa,
    wing_cl, body_angles and spin. A steady-trim solve must converge these states
    and actuator/engine equilibria, rather than resetting them on each call.
    oil_radiator is the engine-owner's prepared mean opening, not a Cd value.
    ground_effect_height is worldY-FM6cfc; height_agl is the separate spin height
    worldY-max(ground,water). Omission assumes they coincide at this location.
    scalar_thrust optionally supplies the engine output in newtons; omitted
    selects a nominal target-RPM value, which does not include upstream healthy
    RPM modulation or the small rounded-update fixed-point band. The returned
    'engine' dictionary describes that nominal diagnostic; force assembly uses
    scalar_thrust when supplied. Coupled/replay checks supply the recovered
    engine wrapper's actual force. See engine_supply.py.
    engine_vectors supplies an explicit (force, moment) parent aggregate,
    including an ensemble mean. It bypasses scalar/nozzle re-evaluation and
    returns engine=None. Mutually exclusive with scalar_thrust.
    """
    if engine_vectors is not None and scalar_thrust is not None:
        raise ValueError('Supply either engine vectors or scalar thrust')
    if ground_effect_height is None:ground_effect_height=height_agl
    if engine_spin_factor is None:
        if scalar_thrust is not None or engine_vectors is not None:raise ValueError('Transient engine requires its explicit spin factor')
        engine_spin_factor=f32(throttle)  # healthy running equilibrium, 1019f39b0 -> engine+a4
    fm=model['fm'];ad=fm['Aerodynamics'];g=model['geometry'];v=list(map(f32,velocity))
    air=cache(v,height);omega=preprocess_angular_rate(stored_omega,air['ias_u'])
    cog=mass_state['cog'];inertia=mass_state['inertia'];mass=mass_state['mass'];commands=list(map(f32,commands));dt=f32(dt)
    flap_values,wing_polar,secondary_properties=condition_properties(model,air['mach'],flaps)
    blend,animation,stab_bias,slats=flap_values
    def control(name,inverted,bias=0.):return mix(model['controls'][name],commands,inverted,bias,air['density'],air['ias_u'],air['mach'],False)
    tc=dict(left_hstab=control('Elevator',[True,False,False]),right_hstab=control('Elevator',[False,False,True]),vstab=control('Rudder',[False,False,False]))
    # 106c5e79f..e860 passes world altitude as xmm0 to the standalone wing-bias
    # helper, whose 2D table evaluator treats it as the outer density axis.
    # The later full mixer uses actual density. Preserve this native difference.
    def wing_bias(name,inverted):
        if model['controls'][name]['wing_aoa']==0.:return 0.
        return mix(model['controls'][name],commands,inverted,0.,f32(height),air['ias_u'],air['mach'],False)[4]
    bias=add(mul(add(wing_bias('Elevator',[True,False,False]),wing_bias('Elevator',[False,False,True])),.5),wing_bias('Rudder',[False,False,False]))
    wc=[control('Ailerons',[True,False,False],bias),control('Ailerons',[False,False,False],bias)]
    device_drag=drag_terms(fm,gear_position=gear,airbrake_position=airbrake,oil_radiator=oil_radiator)
    swirl=f32(engine_wash[1])*.5 if torque_gyro else 0.
    if abs(swirl)<1.1920928955078125e-7:swirl=0.
    wing=wings(g,intact_polars(wing_polar,g['areas']),v,omega,cog,wc,air['density'],dt,history['wing_aoa'],
               beta=air['beta'],body_aoa=air['alpha'],flaps=(blend,blend),gear=(quantized_fraction(gear),)*2,airbrake=(quantized_fraction(airbrake),)*2,
               pitch=commands[1],extra_cd=device_drag,inertia_x=inertia[0],height=ground_effect_height,ground_length=fm['Length'],
               ordering=False,convert_aoa=fm.get('ConvertAoa',False),spin=history['spin'],spin_increment=f32(.2),spin_cap_input=engine_spin_factor,yaw_command=commands[2],spin_agl=height_agl,
               engine_x=f32(engine_wash[0]),engine_swirl=swirl)
    tp=dict(secondary_properties)
    tp.update(horizontal_bias=stab_bias,history_scales=(1.,1.),vertical_control_scale=wing['postprocess']['vertical_control_scale'],
              # 106c628c9: UseSpinLoss bypasses the separate stalled-tail area loss.
              vertical_area_scale=1. if g['use_spin_loss'] else wing['postprocess']['vertical_area_scale'],
              vertical_lift_scale=1.,vertical_area_add=f32(.2),convert_aoa=fm.get('ConvertAoa',False))
    health={n:1. for n in ['left_main','right_main','left_elevator','right_elevator','v_main','rudder','fuselage']}
    ts=dict(velocity=v,omega=omega,cog=cog,density=air['density'],dt=dt,body_angles=[air['alpha'],air['beta']],
            previous_angles=history['body_angles'],previous_wing_cl=history['wing_cl'],health=health,engine_wash=engine_wash,torque_gyro=torque_gyro)
    tail=secondary_model(tp,ts,dict(points=wing['base_points'],cl=[c['raw_cl'] for c in wing['coefficients']]),tc)
    forces=dict(tail['forces']);points=dict(tail['points'])
    for i,name in enumerate(['left_wing','right_wing']):forces[name]=wing['forces'][i];points[name]=wing['points'][i]
    forces['chute']=[0.,0.,0.];points['chute']=[0.,0.,0.]
    forces['parasite']=parasite_force(v,air['density'],ad.get('CockpitDoorCd',0.),cockpit_door,attachment_drag_area)
    raw_force=assemble_force(forces);raw_moment=assemble_moment(forces,points,cog)
    # The longitudinal balance helper is behind the opposite update branch;
    # it is inactive in this detailed pre/post-stall evaluation, even when set.
    helper=[roll_leveling(air['alpha'],v[0],quaternion,mass) if fm.get('RollLeveling',True) else 0.,0.,0.]
    aero_force=limit_aerodynamic_force(raw_force,mass)
    if engine_vectors is None:
        if model['engine'] is None:raise ValueError('Propeller aircraft require propulsion owner force/moment, wash and angular momentum inputs')
        engine=steady(model['engine'],height,v[0],throttle)
        thrust=engine['thrust'] if scalar_thrust is None else f32(scalar_thrust)
        nozzle=fm['Engine0']['Nozzle0']
        if any(nozzle['Direction']):raise ValueError('Only selected forward nozzles supported in composition')
        capped=min(mul(thrust,f32(nozzle['ThrustRatio'])),f32(nozzle['ThrustMax']))
        engine_force,engine_moment=accumulate_nozzle([1.,0.,0.],nozzle['Position'],cog,capped)
    else:
        engine=None
        engine_force,engine_moment=[list(map(float,vector)) for vector in engine_vectors]
    scale=realistic_engine_scale(ext_thrust_mult,fm.get('ExtThrustBaseMult',1.))
    total_force=compose_force(aero_force,external_force,engine_force,scale)
    total_moment=[((raw_moment[i]+helper[i])+float(f32(external_moment[i])))+engine_moment[i] for i in range(3)]
    gyro=gyroscopic_moment(engine_angular_momentum,omega,wing['postprocess']['gyroscopic_scale']) if torque_gyro else [0.,0.,0.]
    total_moment=[x+y for x,y in zip(total_moment,gyro)]
    total_moment=limit_total_moment(total_moment,mass)
    next_history=dict(wing_aoa=[c['aoa'] for c in wing['coefficients']],wing_cl=tail['wake']['previous_cl'],
                      body_angles=tail['previous_angles'],spin=wing['postprocess']['spin'])
    return dict(force=total_force,stored_moment=total_moment,component_forces=forces,component_points=points,
                raw_aero_force=raw_force,raw_aero_moment=raw_moment,helper_moment=helper,engine_force=engine_force,engine_moment=engine_moment,
                air=air,omega_for_flow=omega,wing=wing,tail=tail,history=next_history,engine=engine,
                flap_mapping=flap_values,controls=dict(wing=wc,tail=tc))
