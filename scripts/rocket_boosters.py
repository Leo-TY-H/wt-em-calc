"""Rocket boosters: FM engines with Booster:true (Me 262 C-1a/C-2b, Mirage SEPR, JATO/RATO pods).

Game rules (aces.exe, pinned build):
- Engine props+1 = Booster, props+2 = External (loader 0x142faa250). Boosters are off at spawn
  (spawn 0x140e3ffe1 hot-starts only non-booster engines) and burn only after ID_IGNITE_BOOSTERS
  (control bit FM+0x362c -> 0x142f93180), which instant-starts every booster with its throttle fixed
  at 1.1 (0x142fa5d1e), whatever the pilot's throttle and the manual engine controls.
- Rocket step 0x142fa3b83: thrust = Main.Thrust (the ThrustMax tables are not read; no altitude or
  speed term), fuel flow = 1.1 * FuelConsumptionOnWEP from the engine's own Main.FuelSystemNum, zero
  thrust once that system is empty: rocket_general.step with throttle 1.1.
- An External engine runs only while the module that names it (payload fmModule itemType 'engine')
  is attached (0x1414151d1); the module's 'fuelTank' items make their tanks present (0x141415049),
  full at spawn like drop tanks (0x1411204d0).
"""
from jet_catalog import engines as fm_engines


def seed_state(p, velocity, height, nitro, controls):
    """prop_steady.initial_state for propeller-aircraft controls, with the boosters in controls['boosters']
    (FM engine indices) lit: running (state 7) at the ignition throttle 1.1, as 0x142fa5b30 starts them."""
    from component_assembly import f32
    from prop_steady import initial_state
    native = {k: v for k, v in controls.items() if k != 'boosters'}
    seed = initial_state(p, velocity, height, nitro=nitro, **dict(native, engine_control_mode='automatic'))
    for i in controls.get('boosters', ()):
        if p['engines'][i]['family'] != 3:
            raise ValueError('Booster engine ' + str(i) + ' is not a rocket')
        seed['engines'][i].update(running=7, throttle=f32(1.1), effective_throttle=f32(1.1))
    return seed


def system_fill(fm, capacities, percent):
    """Internal fuel per system at spawn (unit fuel setup 0x140ea2290): systems that feed a non-booster engine
    hold the fuel setting's share; a booster-only system (Me 262 C rocket propellant) is full on a ground start."""
    from component_assembly import f32
    main = {int(e['Main'].get('FuelSystemNum', 0)) for _, e in fm_engines(fm) if not e.get('Booster', False)}
    return [f32(c * percent / 100.) if s in main else f32(c) for s, c in enumerate(capacities)]


def propellant_tanks(fm):
    """FM tank indices of booster-only fuel systems (tanks a booster burns and no other engine does)."""
    from mass_model import tank_configuration
    boosters = {int(e['Main'].get('FuelSystemNum', 0)) for _, e in booster_engines(fm)}
    main = {int(e['Main'].get('FuelSystemNum', 0)) for _, e in fm_engines(fm) if not e.get('Booster', False)}
    return {t['index'] for t in tank_configuration(fm) if int(t['system']) in boosters - main}


def booster_systems(fm):
    """Fuel systems only boosters burn."""
    boosters = {int(e['Main'].get('FuelSystemNum', 0)) for _, e in booster_engines(fm)}
    main = {int(e['Main'].get('FuelSystemNum', 0)) for _, e in fm_engines(fm) if not e.get('Booster', False)}
    return boosters - main


def scale_propellant(fm, fuel_by_system, percent):
    """Jets: the booster propellant setting applied to booster-only systems (100% leaves them as loaded)."""
    from component_assembly import f32
    if percent == 100.:
        return fuel_by_system
    systems = booster_systems(fm)
    return [f32(f * percent / 100.) if s in systems else f for s, f in enumerate(fuel_by_system)]


def scale_tank_propellant(fm, tanks, fill, percent):
    """Propeller aircraft: the same on fuel_loading.initial's per-tank fill; system totals follow."""
    from component_assembly import f32, add
    systems = booster_systems(fm)
    by_tank = [f32(f * percent / 100.) if int(t['system']) in systems else f for t, f in zip(tanks, fill['fuel_by_tank'])]
    by_system = list(fill['fuel_by_system'])
    for s in systems:
        if s < len(by_system):
            total = f32(max(fill['reservoir'][s], 0.) * percent / 100.)
            for t, f in zip(tanks, by_tank):
                if int(t['system']) == s and f > 0.:
                    total = add(total, f)
            by_system[s] = total
    return dict(fill, fuel_by_tank=by_tank, fuel_by_system=by_system)


def booster_engines(fm):
    """(FM engine index, resolved instance) of every rocket booster engine."""
    return [(int(key[6:]), e) for key, e in fm_engines(fm)
            if e.get('Booster', False) and e.get('Main', {}).get('Type') == 'Rocket']


def present(fm, attached=()):
    """Booster engines fitted: built in (not External), or switched on by an attached module."""
    attached = set(attached)
    return [(i, e) for i, e in booster_engines(fm) if not e.get('External', False) or i in attached]


def fueled(fm, fuel_by_system, attached=()):
    """Fitted boosters whose fuel system holds propellant (the game zeroes thrust below 0.01 kg)."""
    return [(i, e) for i, e in present(fm, attached)
            if 0 <= int(e['Main'].get('FuelSystemNum', 0)) < len(fuel_by_system)
            and fuel_by_system[int(e['Main'].get('FuelSystemNum', 0))] >= .01]


def summary(fm, fuel_by_system, attached=(), on=False):
    """What the chart reports: boosters fitted, propellant and burn time at the game's flow."""
    active = fueled(fm, fuel_by_system, attached)
    if not present(fm, attached):
        return None
    flows = {}
    for _, e in active:  # each system's flow at the lit throttle 1.1
        system = int(e['Main'].get('FuelSystemNum', 0))
        flows[system] = flows.get(system, 0.) + 1.1 * float(e['Main'].get('FuelConsumptionOnWEP', 0.))
    burn = min((fuel_by_system[s] / f for s, f in flows.items() if f > 0.), default=0.)
    lit = bool(on and active)
    return dict(on=lit, engines=[i for i, _ in present(fm, attached)], ready=[i for i, _ in active],
                thrust_n=sum(float(e['Main']['Thrust']) for _, e in active) if lit else 0.,
                propellant_kg=sum(fuel_by_system[s] for s in flows), flow_kg_s=sum(flows.values()) if lit else 0.,
                burn_time_s=burn,
                policy=('Rocket boosters lit: full rated thrust at any speed and altitude, independent of the throttle, '
                        'until the propellant runs out' if lit else 'Rocket boosters off (as in the game until ignited)'))


def capability(fm, profile=None):
    """Catalog flag for the 'Booster on' option, or None without booster engines.
    builtin: a fitted booster with internal propellant; default: always-fitted booster modules (common weapons);
    loadout: some loadout fits or fuels a booster."""
    boosters = booster_engines(fm)
    if not boosters:
        return None
    from mass_model import tank_configuration
    from jet_catalog import fuel_capacities
    try:
        internal = fuel_capacities(fm)
    except ValueError:
        internal = []
    tanks = {t['index']: int(t['system']) for t in tank_configuration(fm)}
    systems = {i: int(e['Main'].get('FuelSystemNum', 0)) for i, e in boosters}
    external = {i for i, e in boosters if e.get('External', False)}
    builtin = any(i not in external and systems[i] < len(internal) and internal[systems[i]] > 0 for i, _ in boosters)
    default = loadout = False
    if profile and profile.get('supported'):
        from aircraft_loadouts import module_links
        common = set(profile.get('common', ()))
        for k, item in enumerate(profile['items']):
            if item['k'] != 's' or item.get('c') != 'payload':
                continue
            links = module_links(item['w']) or {}
            fits = any(i in external for i in links.get('engine', ()))
            fuels = any(tanks.get(t) == systems[i] for t in links.get('fueltank', ()) for i, _ in boosters if i not in external)
            if fits or fuels:
                loadout = True
                default = default or (k in common and fits)
    if not (builtin or loadout):
        return None
    return dict(builtin=builtin, default=default, loadout=loadout, engines=len(boosters))
