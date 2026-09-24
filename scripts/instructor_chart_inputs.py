"""Native controller inputs shared by the experimental chart approximation."""
import json
from pathlib import Path
from tail_model import aircraft_secondary_properties
from component_assembly import f32
from control_mixer import curve
from instructor_settings import restored_wing_normalization
from instructor_protection import gain_reference_speed
from polar_runtime import flap_polar
from jet_catalog import engines as installed_engines
from body_dynamics import realistic_engine_scale

GAMEPLAY=json.loads((Path(__file__).resolve().parents[1]/'references/body-gameplay.blkx').read_text())['instructor']


def source_state(solver,value):
    """Build original controller source fields from the detailed operating point.

    Clean intact aircraft, no payload, gear/airbrake/radiators retracted. All
    inputs are derived independently; executable memory is only a test oracle.
    """
    model=solver.model;fm=model['fm'];ad=fm['Aerodynamics'];g=model['geometry'];mass=solver.mass
    tp=aircraft_secondary_properties(fm,{});air=value['result']['air'];flaps=value['flaps']
    mapped=curve(model['flaps'],f32(flaps),4) if model['flaps'] else [f32(flaps),f32(flaps),0.,0.]
    f={0x18a0:1.,0x18d4:1.,0x18d8:1.,0x8448:0.}
    for off,v in zip([0x5320,0x5324,0x5328],mass['cog']):f[off]=f32(v)
    for off,v in zip([0x6f20,0x6f24,0x6f28],tp['arms']['hstab']):f[off]=f32(v)
    for off,v in [(0x7334,tp['arms']['fuselage'][1]),(0x712c,tp['arms']['vstab'][1]),(0x7140,tp['slipstream_distance']),(0x6f1c,tp['incidence']['hstab'])]:f[off]=f32(v)
    for key,off in dict(fuselage=0x83fc,left_main=0x8420,left_elevator=0x8424,right_main=0x8428,right_elevator=0x842c,v_main=0x8430,rudder=0x8434).items():f[off]=f32(tp['areas'][key])
    for off,key in [(0x7cc4,'CockpitDoorCd'),(0x7ca8,'GearCd'),(0x7cb4,'RadiatorCd'),(0x7cb8,'OilRadiatorCd'),(0x7cb0,'AirbrakeCd'),(0x7cc0,'FuseCd')]:f[off]=f32(ad.get(key,0.))
    f[0x79b8],f[0x79bc]=model['controls']['Ailerons']['cd'];f.update(restored_wing_normalization(g['areas']))
    flags={0x7fa6:bool(fm['AvailableControls']['hasFlapsControl']),0x7fa3:bool(fm['AvailableControls']['hasGearControl']),
           0x7c08:bool(fm.get('AllowModsToChangeLongidutialBalance',True)),0x7c0a:bool(fm.get('ConvertAoa',False)),
           0x7c54:bool(fm['InvertElevator']),0x8471:False,0x6f3c:tp['clockwise']}
    # Resolving installed engines recursively copies their complete FM blocks.
    # The count is immutable for this prepared aircraft, not an operating-state
    # quantity. Do that work once, outside the boundary/interior iterations.
    if '_instructor_engine_count' not in model:
        model['_instructor_engine_count']=len(installed_engines(fm))
    predictor=dict(f=f,flags=flags,pitch_inertia=mass['inertia'][2],engine_count=model['_instructor_engine_count'],balance_multiplier=1.,new_balance=True,rho0=f32(1.225))
    props=dict(GAMEPLAY['propsDefault']);props.update(fm.get('Instructor',{}))
    prop=value.get('propulsion')
    vectors=(prop['force'],prop['moment']) if prop is not None else solver.engine.vectors(value['velocity'][0],flaps)
    wrapper=dict(mass=f32(mass['mass']),tas=air['tas'],speed_squared=air['speed_squared'],mach=air['mach'],height=float(solver.config['altitude_m']),
        # Owner wash fields are float32 even when propulsion is represented
        # by a finite-cycle mean. Input packing already rounds these values;
        # expose the same original source-field representation to the audit.
        engine_wash=list(map(f32,prop['wash'])) if prop is not None else [0.,0.],engine_force=vectors[0],engine_moment=vectors[1],engine_scale=realistic_engine_scale(1.,fm.get('ExtThrustBaseMult',1.)),
        torque_gyro=solver.config['torque_gyro'],flap_blend=mapped[0],flap_health=[1.,1.],flap_incidence=mapped[2],sweep=model['sweep'],
        gear_fraction=value.get('gear',0.),gear_available=[True,True],airbrake_fraction=0.,airbrake_health=[1.,1.],oil_radiator=0.,water_radiator=0.,cockpit_door=0.,parameter_pointer=0,gameplay_pointer=0)
    commands=value['allocation']['commands'];snapshot=list(commands)
    if solver.controls['invert_elevator']:snapshot[1]=-snapshot[1]
    return dict(predictor=predictor,wrapper=wrapper,properties=props,
        time_constants=[f32(.05)]*3,linear_rates=[f32(.03)]*3,
        trim_requested=[0.]*3,trim_actual=[0.]*3,trim_cache=[0.]*3,autotrim_enabled=True,default_autotrim=True,
        indicated_airspeed=air['ias_u'],longitudinal_speed=value['velocity'][0],rho0=f32(1.225),axis_enabled=[True]*3,authority_scale=1.,full_control_loss=True,
        asymmetric_authority=False,elevator_state=snapshot[1],delivered=list(commands),wing_angles=value['result']['history']['wing_aoa'],
        # 106c5cadf..cb46 writes FM8218 from the mapped flap blend (healthy
        # left/right mean), not from the raw device position.
        wing_runtime=flap_polar(model['polars']['WingPlane'],mapped[0]),wing_area=g['area'],dihedral=g['dihedral'],strength=g['strength']['force'],
        tail_area_pair=[f32(ad['HorStabPlane']['Areas'][k]) for k in ['Main','Elevator']],overload_enabled=True,
        force_advanced=bool(props.get('MouseAim',{}).get('forceAdvanced',False)),world_velocity=[float(value['speed']),0.,0.],
        world_acceleration=value['kinematic']['world_acceleration'],quaternion=value['geometry']['quaternion'],
        pitch_rate=float(value['geometry']['omega'][2]),stored_yaw_rate=float(value['geometry']['omega'][1]),
        reference_speed=gain_reference_speed(fm['Mass'].get('Takeoff',0.),flap_polar(model['wing_family'][0][1]['polars'],0.)),
        requested=[0.,1.,0.],command_cache_enabled=True,recovery_enabled=True,recovery_suppressed=False)

