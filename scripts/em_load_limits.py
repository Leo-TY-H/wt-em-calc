import math

SPEED_KMH=300.
LOAD_G=8.
LOWER_SPEED_KMH=200.
LOWER_LOAD_G=3.6
GLOBAL_LOAD_G=64.
ANCHOR_MARGIN_G=.05


def configured_ceiling(solver,speed):
    limit=solver.config.get('max_load_g') or 64.
    if not getattr(solver,'chart_search',False):return limit
    limit=min(limit,solver.config.get('global_load_cap_g',GLOBAL_LOAD_G))
    if solver.config.get('reference_load_cap',False):
        from em_reference_envelope import reference_ceiling
        return min(limit,reference_ceiling(speed))
    if not solver.config.get('low_speed_load_cap',True):return limit
    if speed>=SPEED_KMH:return limit
    limit=min(limit,LOAD_G)
    if speed<=LOWER_SPEED_KMH:limit=min(limit,LOWER_LOAD_G)
    for anchor in getattr(solver,'chart_load_anchors',[]):
        if speed<anchor['speed_kmh']:
            limit=min(limit,max(1.1,anchor['upper_load_g']))
    return limit


def ceiling(solver,speed):
    limit=configured_ceiling(solver,speed)
    if (getattr(solver,'chart_search',False) and solver.config.get('aircraft_search_region',False)
            and not solver.config.get('reference_load_cap',False)):
        from em_aircraft_region import search_ceiling
        limit=min(limit,search_ceiling(solver,speed))
    return limit


def description(solver,speed):
    if not getattr(solver,'chart_search',False):return None
    limit=ceiling(solver,speed)
    if (solver.config.get('aircraft_search_region',False) and
            not solver.config.get('reference_load_cap',False)):
        from em_aircraft_region import preliminary
        region=preliminary(solver,speed)
        if region is not None and limit<configured_ceiling(solver,speed):
            return dict(region,load_g=limit,physical_limit=False,anchors=[],
                global_load_cap_g=solver.config.get('global_load_cap_g',GLOBAL_LOAD_G))
    if solver.config.get('reference_load_cap',False):
        from em_reference_envelope import reference_values, TOLERANCE_G, TOLERANCE_DPS, REFERENCE_CONDITIONS
        base,reference=reference_values(speed)
        basis='Frozen I-153 / BI / F-16XL SB flutter-on composite + min(2 deg/s, 1 g)'
        if limit<reference:basis='user-selected global load search cap'
        if solver.config.get('max_load_g')==limit:basis='user-selected plot load ceiling'
        return dict(load_g=limit,physical_limit=False,basis=basis,
            reference_load_g=base,tolerance_g=TOLERANCE_G,tolerance_dps=TOLERANCE_DPS,
            applied_margin_g=reference-base,frozen=True,
            reference_conditions=REFERENCE_CONDITIONS,anchors=[])
    low_speed=solver.config.get('low_speed_load_cap',True) and speed<SPEED_KMH
    lower_speed=low_speed and speed<=LOWER_SPEED_KMH
    anchors=[a for a in getattr(solver,'chart_load_anchors',[]) if low_speed and speed<a['speed_kmh'] and max(1.1,a['upper_load_g'])<=limit+1e-10]
    basis='user-selected global load search cap'
    if low_speed and limit==LOAD_G:basis='user-selected 8 g low-speed search cap'
    if lower_speed and limit==LOWER_LOAD_G:basis='user-selected 3.6 g low-speed search cap'
    if anchors:basis='monotonic low-speed envelope assumption'
    if solver.config.get('max_load_g')==limit:basis='user-selected plot load ceiling'
    return dict(load_g=limit,below_speed_kmh=SPEED_KMH if low_speed and not lower_speed else None,
        at_or_below_speed_kmh=LOWER_SPEED_KMH if lower_speed else None,physical_limit=False,
        global_load_cap_g=solver.config.get('global_load_cap_g',GLOBAL_LOAD_G),basis=basis,
        anchors=anchors)


def anchor_from_columns(columns,speed):
    known=[c for c in columns if c.get('boundary_status')=='verified limit' and c.get('boundary') and c['boundary']['valid']]
    if not known:return None
    peak=max(known,key=lambda c:c['boundary']['load_g'])
    rising=sorted((c for c in known if c['speed_kmh']<=peak['speed_kmh']),key=lambda c:c['speed_kmh'])
    if any(b['boundary']['load_g']<a['boundary']['load_g']-.005 for a,b in zip(rising,rising[1:])):return None
    higher=[c for c in rising if c['speed_kmh']>speed]
    if not higher:return None
    c=min(higher,key=lambda c:c['speed_kmh'])
    bracket=c.get('boundary_bracket_g') or [c['boundary']['load_g']]
    return dict(speed_kmh=c['speed_kmh'],upper_load_g=max(bracket)+ANCHOR_MARGIN_G,
        observed_peak_speed_kmh=peak['speed_kmh'],assumption=True)


def excluded_point(solver,speed,load,initial=None):
    if not getattr(solver,'chart_search',False) or load<=ceiling(solver,speed)+1e-10:return None
    x=list(initial) if initial is not None else solver.initial_guess(speed/3.6,load)
    return dict(speed_kmh=float(speed),load_g=float(load),valid=False,converged=False,
        reasons=['chart load search limit'],not_evaluated=True,evaluations=0,search_work={},
        search_limit=description(solver,speed),solution=x,alpha_deg=x[0],bank_deg=x[1],
        sideslip_deg=solver.sideslip_attitude_deg,sideslip_attitude_deg=solver.sideslip_attitude_deg,
        force_error_g=math.inf,angular_error_rad_s2=math.inf,history_error=math.inf,
        stall_margin_deg=math.inf,negative_stall_margin_deg=math.inf,authority_margin=math.inf,
        wing_load_ratios=[0.,0.],bounded_search_stationary=False,instructor=None,
        instructor_enabled=solver.config['instructor'],structural_flags=[],pitch_response=None,
        turn_dps=math.degrees(9.8100004196167*math.sqrt(max(0.,load*load-1.))/(speed/3.6)),
        ps_mps=None,ps_continuous_mps=None,component_forces={},propulsion={})


def annotate(solver,column):
    if column is None or 'speed_kmh' not in column:return column
    note=description(solver,column['speed_kmh'])
    if note is not None:
        column['load_search_limit']=note
        p=column.get('boundary')
        if p and p['valid'] and abs(p['load_g']-note['load_g'])<1e-8:
            column['boundary_status']='plot ceiling'
            column['boundary_reason']='chart load search limit'
    return column
