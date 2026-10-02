"""Preliminary aircraft search domains.

The padded force estimate is predetermined from aircraft data and conditions.
It has no dependency on engine settling, trim solutions or search history.
It is a search-domain assumption, not a physical-limit certificate.
"""
import math
from collections import OrderedDict

REVISION = 'predetermined-forces-v2'
FORCE_MARGIN = 1.25
LOAD_MARGIN_G = .25


def engine_data(solver):
    """Prepare immutable source properties once, without creating an engine
    simulator. The estimator can run on a context with no engine object.
    """
    if hasattr(solver,'_region_engine_data'):return solver._region_engine_data
    if solver.is_prop:
        from prop_catalog import assets
        p=assets(solver.name)[0]
        data=dict(p,engines=[dict(e,properties=dict(e['properties'],amplitude=[0.,0.,0.]))
                            for e in p['engines']])
    else:
        import re
        from jet_catalog import engines
        from jet_model import prepare
        from jet_nozzle import prepare as nozzle
        data=[]
        for _,e in engines(solver.fm):
            if e.get('Booster',False):continue
            main=e['Main'];kind=main['Type']
            if kind not in ('Jet','Rocket'):continue
            data.append(dict(kind=kind,main=main,boost=e.get('Afterburner',{}),
                jet=prepare(main) if kind=='Jet' else None,
                nozzles=[nozzle(e[k]) for k in sorted(e) if re.fullmatch(r'Nozzle\d+',k)]))
    solver._region_engine_data=data
    return data


def direct_thrust(solver,speed,density):
    """Nominal RPM from the throttle table, then direct thrust/nozzle lookup.
    No scalar_update, fuel-flow iteration, cycle averaging or time stepping.
    """
    from control_mixer import curve
    from jet_model import table,mode
    from jet_nozzle import evaluate
    from engine_commands import afterburner_command
    total=0.
    for engine in engine_data(solver):
        main=engine['main'];throttle=solver.config['throttle']
        if engine['kind']=='Jet':
            jet=engine['jet']
            rpm=curve(jet['throttle'],min(1.,throttle/jet['throttle_scale']),1)[0]
            # Include table knots below the requested RPM as well as its value;
            # the nominal engine need not sit exactly on that RPM in the model.
            multiplier=max([mode(jet,rpm)[0]]+[r[2][0] for r in jet['modes'] if r[0]<=rpm])
            # Body-axis inflow can be below TAS at incidence. Bound each
            # piecewise-linear table factor at endpoints and enclosed knots.
            speeds=[0.,speed]+[v for v in jet['speed'] if 0.<v<speed]
            boost=engine['boost']
            ab=afterburner_command(solver.config['afterburner'],boost.get('Type',0),boost.get('IsControllable',True))
            values=[table(jet,density,v) for v in speeds]
            thrust=max(t for t,_,_,_ in values)*max(0.,multiplier)
            if ab:thrust*=max(1.,max(b for _,b,_,_ in values))
        else:
            throttle=min(throttle,main.get('MaxThrMult',1.1))
            fraction=min(1.,throttle/(1.1 if main['ThrottleBoost']>1. else 1.))
            rpm=main['RPMMin']+(main['RPMMax']-main['RPMMin'])*fraction
            thrust=main['Thrust']*rpm/main['RPMMax'] if throttle>0. else 0.
        # Requested and clean flap settings account for automatic retraction.
        total+=max(math.hypot(*evaluate(engine['nozzles'],thrust,solver.mass['cog'],
            ias=speed*math.sqrt(density/1.225),flaps=flaps)['force']) for flaps in {0.,solver.flaps})
    return total


def propulsion_estimate(solver, speed, density):
    """No propeller equilibrium or aircraft trim: nominal engine torque and
    ideal static disc thrust allow for thrust at large body incidence. Sum disc
    areas once (including coaxial pairs on the same disc), and retain direct
    turbine thrust. Static thrust is intentionally not replaced by P / TAS.
    """
    if not solver.is_prop:
        return direct_thrust(solver,speed,density)
    from prop_steady import initial_state
    from prop_quasisteady import engine_output
    from propulsion_general import target_omega
    properties=engine_data(solver)
    controls=dict(throttle=solver.config['throttle'],afterburner=solver.config['afterburner'],
                  engine_control_mode='automatic')
    nitro=solver.mass['nitro_mass']
    state=initial_state(properties,(speed,0.,0.),solver.config['altitude_m'],nitro=nitro,**controls)
    power=direct=0.
    for engine,seed in zip(properties['engines'],state['engines']):
        if engine['family']==3:continue  # Optional boosters are disabled in trim.
        target=target_omega(engine['properties'],seed,nitro)
        outputs=[]
        for fraction in (.8,1.,1.1):
            s=dict(seed,omega=max(1.,target*fraction))
            if engine['family'] in (2,5):s['turbo']=engine['turbine']['max_omega']
            result=engine_output(engine,s,(speed,0.,0.),solver.config['altitude_m'],nitro,solver.mass['cog'])
            outputs.append((max(0.,result['torque']*s['omega']),math.hypot(*result.get('force',(0.,0.,0.)))))
        power+=max(p for p,_ in outputs)
        direct+=max(t for _,t in outputs)
    area=sum(math.pi*p['properties']['diameter']**2/4. for p in properties['propellers'])
    return (2.*density*area*power**2)**(1./3.)+direct


def preliminary(solver, speed):
    """Cached per-speed domain from selected mass, Mach, flaps and structure.
    Missing/nonfinite input leaves the explicitly configured outer domain in
    force. Cache identity includes all mutable inputs used by this estimate.
    """
    if (not solver.config.get('aircraft_search_region',False) or
            solver.config.get('reference_load_cap',False) or solver.config.get('sampling')=='regular'):return None
    key=(float(speed),solver.weight,solver.mass.get('nitro_mass',0.),solver.flaps,solver.config['altitude_m'],
         solver.config['structural_limits'],solver.config['throttle'],solver.config['afterburner'])
    if not hasattr(solver,'_aircraft_regions'):solver._aircraft_regions=OrderedDict()
    cache=solver._aircraft_regions
    if key in cache:
        cache.move_to_end(key)
        return cache[key]
    from aircraft_model import condition_properties
    from air_state import cache as air_cache
    from component_assembly import f32
    v=speed/3.6
    if not math.isfinite(v) or v<=0. or not solver.weight>0.:return None
    air=air_cache([f32(v),0.,0.],solver.config['altitude_m'])
    q=.5*air['density']*v*v
    geometry=solver.model['geometry']
    # Flaps may retract automatically. Enclose both requested and clean lift
    # capacity rather than using an already-trimmed deployment as a prerequisite.
    estimates=[]
    for flaps in sorted({0.,solver.flaps}):
        _,wing,secondary=condition_properties(solver.model,air['mach'],flaps)
        def peak(p):return max(abs(p['cyCritH']),abs(p['cyCritL']))*abs(p['clKq'])
        areas=secondary['areas'];polars=secondary['polars']
        wing_force=q*geometry['area']*peak(wing)
        structural=math.inf
        if solver.config['structural_limits']:
            angle=min(80.,max(0.,wing['aoaCritH']+abs(geometry['incidence'])))
            structural=2.*max(geometry['strength']['force'])/math.cos(math.radians(angle))
        tail=q*(sum(areas[k] for k in ('left_main','right_main','left_elevator','right_elevator'))*peak(polars['hstab'])+
            (1.5*areas['v_main']+areas['rudder']+.2)*peak(polars['vstab']))
        fuselage=q*areas['fuselage']*peak(polars['fuselage'])
        estimates.append((min(wing_force,structural),tail,fuselage))
    try:thrust=propulsion_estimate(solver,v,air['density'])
    except (ValueError,ArithmeticError):thrust=math.nan
    forces=[max(e[i] for e in estimates) for i in range(3)]+[thrust]
    note=None
    if all(math.isfinite(x) and x>=0. for x in forces):
        estimate=sum(forces)/solver.weight
        limit=max(1.1,FORCE_MARGIN*estimate+LOAD_MARGIN_G)
        note=dict(revision=REVISION,estimated_load_g=estimate,initial_load_g=limit,
            force_margin=FORCE_MARGIN,load_margin_g=LOAD_MARGIN_G,certifies_exclusion=False,predetermined=True,
            basis='preliminary Mach/flap lifting surfaces, selected mass, wing strength and propulsion',
            force_estimates_g=dict(zip(('wing','tail','fuselage','engine'),(x/solver.weight for x in forces))))
    cache[key]=note
    if len(cache)>512:cache.popitem(last=False)
    return note


def search_ceiling(solver,speed):
    note=preliminary(solver,speed)
    if note is None:return math.inf
    return note['initial_load_g']


def annotate(solver,column):
    speed=column['speed_kmh']
    region=preliminary(solver,speed)
    if region is not None:
        from em_load_limits import ceiling,annotate as annotate_limit
        annotate_limit(solver,column)
        cap=ceiling(solver,speed)
        column['aircraft_search_region']=dict(region,search_ceiling_g=cap,
            verified_boundary=column.get('boundary_status')=='verified limit')
        if column.get('boundary_status')=='no feasible samples':
            column['coverage_complete']=False
            column['searched_load_interval_g']=[1.,cap]
    return column
