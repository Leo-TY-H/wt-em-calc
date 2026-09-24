"""Exhaustive declared-field inventory, not a claim of complete FM reconstruction.

Every scalar/list element is assigned a subsystem, including inactive fields.
Native defaults and owner-supplied records are separate from declared fields.
"""
import hashlib
import json
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
REF = ROOT/'references/fm-2.59.0.13'
BALANCE = {'AllowModsToChangeLongidutialBalance','RollLeveling'}
CONTROL = {'AileronEffectiveSpeed','RudderEffectiveSpeed','ElevatorsEffectiveSpeed',
           'AileronPowerLoss','RudderPowerLoss','ElevatorPowerLoss','AlphaAileronMin',
           'AlphaRudderMin','AlphaElevatorMin','AllowStrongControlsRestrictions',
           'InvertElevator','RudderSens','ElevatorSens','AileronSens','AileronMaxDv',
           'ElevatorMaxDv','RudderMaxDv','VneControl','AileronAngles','ElevatorAngles',
           'RudderAngles','RudderInterceptor','Elevon'}
ACTUATORS = {'FlapsRadiator','GearActuatorSpeed','WingActuatorSpeed','CockpitDoorSpeedOpen',
             'CockpitDoorSpeedClose','CockpitDoorBlockSpeed','AirBrakeSpeed','BayDoorSpeed',
             'BombLauncherSpeed','SweepWingActuatorSpeed','dvFlapsIn','dvFlapsOut',
             'maxChuteSpeed','minChuteSpeed','chuteRipSpeed','FlapsAngle','VSlats',
             'SlatsRelAoa','FlapsToSlats','MaxSpeedFlaps','MinSpeedFlaps'}
GEOMETRY = {'Length','SweptWingAngle','WingTaperRatio','Wingspan','StabWidth','FinHeight',
            'WingAngle','StabAngle','KeelAngle','Areas','Focus'}
LIMITS = {'Vne','VneMach','VneCockpitDoor','CockpitOpenedDoorBreakSpeed',
          'WingWaveMassRel','WingSpringDampJointMult'}
REFERENCE = {'MaxSpeedNearGround','MaxSpeedAtAltitude','MinimalSpeed','CriticalSpeed'}


def leaves(obj, path=()):
    if isinstance(obj,dict) and obj:
        for k,v in obj.items(): yield from leaves(v,path+(k,))
    elif isinstance(obj,list) and obj:
        for i,v in enumerate(obj): yield from leaves(v,path+(str(i),))
    else: yield '.'.join(path),obj


def domain(path):
    parts=path.split('.');top=parts[0]
    if top in BALANCE:return 'balance_and_mode_helpers'
    if top.startswith('Arcade') or top in {'Autopilot','MouseAim','FlyByWire'}:return 'automatic_control_and_modes'
    if top in CONTROL or top=='AvailableControls':return 'pilot_controls_and_trim'
    if top in ACTUATORS:return 'devices_and_actuators'
    if top in GEOMETRY:return 'airframe_geometry_and_legacy_adapter'
    if top in LIMITS:return 'structural_limits_and_flex'
    if top in REFERENCE or top=='Passport':return 'reference_and_helper_parameters'
    if top in {'MomentOfInertia','Crew','Mass'}:return 'mass_cg_inertia_fuel_supply'
    if top=='Gear':return 'ground_and_water_contact'
    if top=='SelfSealingTanks':return 'damage_and_fire'
    if top=='IgnoreErrors':return 'loader_metadata'
    if top.startswith('Transmission'):return 'transmission_rpm_and_inertia'
    if top.startswith('Propeller') or 'Propellor' in parts:
        if 'Polar' in parts:return 'blade_polar'
        if 'Governor' in parts or any(x.startswith('Governor') or x.startswith('ThrottleRPMAuto') for x in parts):return 'governor_and_prop_controls'
        if 'Damage' in parts:return 'damage_and_fire'
        if 'Mass' in parts or 'InertiaMomentCoeff' in parts:return 'propeller_rotating_inertia'
        return 'propeller_geometry_flow_and_controls'
    if top.startswith('Engine'):
        if 'Temperature' in parts:return 'thermal_and_cooling'
        if 'FireExtinguisher' in parts:return 'damage_and_fire'
        if 'Compressor' in parts:return 'compressor_and_ram'
        if 'Mixer' in parts:return 'mixture'
        if 'Afterburner' in parts:return 'wep_and_boost'
        if 'Controls' in parts or 'AutoThrottle' in parts:return 'engine_command_delivery'
        if 'Nozzle0' in parts or 'External' in parts:return 'engine_mount_and_output_routing'
        return 'piston_torque_supply_and_lifecycle'
    if top=='Aerodynamics':
        if len(parts)>1 and parts[1] in {'Ailerons','Elevator','Rudder'}:return 'control_surface_mixing'
        if len(parts)>1 and (parts[1].startswith('Flaps') or parts[1] in {'GearCd','GearCentralCd','RadiatorCd','OilRadiatorCd','AirbrakeCd','CockpitDoorCd','BombBayCd','chuteCx'}):return 'devices_and_actuators'
        return 'airframe_polars_flow_stall_and_wake'
    raise ValueError('Unclassified FM field: '+path)


def main():
    jets={n:dict(leaves(json.loads((REF/(n+'.blkx')).read_text()))) for n in ['f_16a_block_15_adf','saab_jas39c']}
    result=dict(scope=__doc__,schema_version=1,aircraft={},native_default_evidence={
        'AllowModsToChangeLongidutialBalance':dict(default=True,loader='101a3797a..101a3798e'),
        'RollLeveling':dict(default=True,loader='101a379ef..101a37a03'),
        'legacy_propeller_AirFlowSolver':dict(default=False,loader='101a0636c..101a0637d'),
        'propeller_CombinedCl':dict(default=True,loader='10198ce00;101a02fe0'),
        'propeller_missing_Oswald':dict(default='55.5 / geometry aspect input',loader='101a02fe0'),
        'legacy_propeller_Mach6':dict(default=[0,0,0,0,0],loader='10198cdd4..10198cdde'),
        'legacy_propeller_Mach7':dict(default=[0,1,1,0,1],loader='10198cde8..10198cdf6')},
        external_inputs=['Aircraft mesh-associated mass records: Bf default intact geometry is decoded; other aircraft and live pose remain separate (mesh-mass-validation.json)',
                         'Actual ammunition/loadout mass, positions and parasite drag',
                         'Runtime difficulty and modification multipliers',
                         'Ground trim settings, command delivery and engine/control histories',
                         'Atmosphere, air-relative body motion, attitude, terrain clearance and timestep'])
    for name in ['yak-3','bf-109f-4']:
        path=REF/(name+'.blkx');fm=json.loads(path.read_text());entries=[]
        for key,value in leaves(fm):
            comparisons={n:('same' if v[key]==value and type(v[key]) is type(value) else 'different')
                         if key in v else 'path_absent' for n,v in jets.items()}
            entries.append(dict(path=key,value=value,domain=domain(key),same_path_comparison_to_jets=comparisons))
        result['aircraft'][name]=dict(sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            schema='modern' if 'EngineType0' in fm else 'legacy',
            effective_balance_flags={k:fm.get(k,True) for k in sorted(BALANCE)},
            advanced_mass=fm['Mass']['AdvancedMass'],
            field_count=len(entries),domain_counts=dict(sorted(Counter(e['domain'] for e in entries).items())),fields=entries)
    (ROOT/'analysis/propeller-fm-audit.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({n:{k:v for k,v in a.items() if k!='fields'} for n,a in result['aircraft'].items()},indent=2))


if __name__=='__main__':main()
