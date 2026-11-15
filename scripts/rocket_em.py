import math
import re
from functools import lru_cache
from component_assembly import f32,add,mul
from rocket_general import step
from engine_supply import fuel_properties,available_fuel
from jet_nozzle import prepare as prepare_nozzle,evaluate as nozzle_vectors
from control_mixer import density_at_height


def prepare(instance,booster=False):
    main=instance['Main']
    if main['Type']!='Rocket':raise ValueError('Rocket engine required')
    if instance.get('Booster',False) and not booster:raise ValueError('Auxiliary rocket boosters remain disabled')
    if instance.get('AutoThrottle',{}).get('HasContorller',False):
        raise ValueError('Rocket automatic throttle controller is not supported')
    required=('RPMMin','RPMMax','Thrust','ThrottleBoost','FuelConsumptionOnWEP')
    if any(not isinstance(main.get(k),(int,float)) or not math.isfinite(main[k]) for k in required):
        raise ValueError('Incomplete finite rocket properties')
    if not 0<=main['RPMMin']<=main['RPMMax'] or main['RPMMax']<=0 or main['Thrust']<=0 or main['FuelConsumptionOnWEP']<0:
        raise ValueError('Invalid rocket operating range')

    p=dict(max_omega=mul(f32(main['RPMMax']),f32(.10471975803375244)),
        shaft_min=mul(f32(main['RPMMin']),f32(.10471975803375244)),
        throttle_boost=f32(main['ThrottleBoost']),consumption=[0.,0.,0.,f32(main['FuelConsumptionOnWEP'])])


    return dict(properties=p,maximum_direct_thrust=f32(main['Thrust']),axis=[1.,0.,0.],position=[0.,0.,0.])


class RocketUnit:
    def __init__(self,model,mass,config,instance,booster=False):
        self.config,self.mass,self.booster=config,mass,booster
        self.properties=prepare(instance,booster)
        if booster:
            # The rocket step (aces.exe 0x142fa3b83) pushes along the engine block's Vector/Direction at its Position
            # and reads no Nozzle section or flap term.
            from jet_catalog import engine_mount
            self.nozzles=[prepare_nozzle(engine_mount(instance,flaps=False))]
        else:self.nozzles=[prepare_nozzle(instance[k]) for k in sorted(
            (k for k in instance if re.fullmatch(r'Nozzle\d+',k)),key=lambda k:int(k[6:]))]
        if not self.nozzles:raise ValueError('Rocket requires a resolved engine mount')
        main=instance['Main'];system=int(main.get('FuelSystemNum',0))
        fuel=mass['fuel_by_system']
        if not 0<=system<len(fuel):raise ValueError('Rocket fuel system is missing')
        # A lit booster keeps the throttle the ignition set, 1.1 (aces.exe 0x142fa5d1e), whatever the pilot's.
        throttle=f32(1.1) if booster else f32(min(config['throttle'],main.get('MaxThrMult',1.1)))

        running=7 if throttle>0 and fuel[system]>0 else 0
        self.scalar=step(self.properties,dict(throttle=throttle,running=running))
        self.thrust=self.scalar['force'][0]
        self.consumption=self.scalar['consumption']
        fp=fuel_properties(model['fm']['Mass'],system);dt=f32(1/config['timestep_hz'])
        supplied=available_fuel(fp,fuel[system],min(fp['capacity'],fuel[system]),1.,dt)
        if mul(self.consumption,dt)>supplied:
            raise ValueError('Rocket fuel-flow-limited operation is outside the supplied steady model')
        self.rho=density_at_height(f32(config['altitude_m']))
        self.summary=dict(policy='Rocket booster lit; fixed fuel load' if booster else 'Supplied stationary rocket; fixed fuel load',period_steps=1,samples=1,
            propulsion='rocket',thrust_n=self.thrust,fuel_consumption_kg_s=self.consumption,
            fuel_system=system,effective_throttle=throttle,running=bool(running),
            fuel_time_at_current_throttle_s=fuel[system]/self.consumption if self.consumption>0 else None,
            note='Fuel time is inventory divided by current flow, not a simulated burn trajectory; no jet thrust-table or afterburner multiplier')

    @lru_cache(maxsize=2048)
    def vectors(self,body_u,flaps=0.):
        ias=mul(f32(body_u),f32(math.sqrt(f32(self.rho/f32(1.225)))))
        result=nozzle_vectors(self.nozzles,self.thrust,self.mass['cog'],ias,flaps=flaps)
        return result['force'],result['moment']


def booster_units(model,mass,config,boosters):
    """Lit booster engines as RocketUnits. Boosters of equal thrust share one unit whose nozzles each carry that
    thrust (jet_nozzle.evaluate applies thrust x ratio per nozzle), up to the compiled loop's 16 nozzles a unit."""
    groups=[]
    for _,instance in boosters:
        unit=RocketUnit(model,mass,config,instance,booster=True)
        group=next((g for g in groups if g.thrust==unit.thrust and len(g.nozzles)+len(unit.nozzles)<=16),None)
        if group is None:
            unit.summary=dict(unit.summary,engines=1);groups.append(unit);continue
        group.nozzles=[*group.nozzles,*unit.nozzles];group.consumption=add(group.consumption,unit.consumption)
        group.summary=dict(group.summary,thrust_n=add(group.summary['thrust_n'],unit.thrust),
                           fuel_consumption_kg_s=group.consumption,engines=group.summary['engines']+1)
    return groups
