import math
from air_state import cache,speed_of_sound
from wing_sweep import prepare,select,schedule,available
from flap_model import profile as flap_profile, limits as flap_limits


def sweep_excluded_kmh(fm,config):
    """TAS intervals of the chart speed range where the selected fixed wing sweep is unavailable."""
    if len(prepare(fm))<2:return []
    rows=schedule(fm);position=config['sweep_percent']/100.
    scale=float(speed_of_sound(config['altitude_m']))*3.6
    low,high=config['speed_min_kmh']/scale,config['speed_max_kmh']/scale
    knots=[low,high]+[row[0] for row in rows if low<row[0]<high]
    for (xa,_,va),(xb,_,vb) in zip(rows,rows[1:]):
        for a,b in zip(va,vb):
            if a!=b:
                x=xa+(position-a)*(xb-xa)/(b-a)
                if max(low,xa)<x<min(high,xb):knots.append(x)
    knots=sorted(set(knots));excluded=[]
    for a,b in zip(knots,knots[1:]):
        lo,hi=available(fm,rows,(a+b)*.5)
        if lo-1e-7<=position<=hi+1e-7:continue
        if excluded and abs(excluded[-1][1]-a*scale)<1e-6:excluded[-1][1]=b*scale
        else:excluded.append([a*scale,b*scale])
    return excluded


def sweep_cutoff_kmh(fm,config):
    """The speed above which the selected fixed sweep stays unavailable up to the end of the chart range
    (drawn as a speed boundary, Leo 2026-10-19), or None."""
    excluded=sweep_excluded_kmh(fm,config)
    if not excluded or excluded[-1][1]<config['speed_max_kmh']-1e-6 or excluded[-1][0]<=config['speed_min_kmh']+1e-6:return None
    return excluded[-1][0]


def speed_limits(fm,config):
    strength=select(prepare(fm),config['sweep_percent']/100.)['geometry']['strength']


    factor=cache([1.,0.,0.],config['altitude_m'])['ias_u']
    vne=strength['ias']/factor*3.6
    mne=strength['mach']*speed_of_sound(config['altitude_m'])*3.6
    if not all(math.isfinite(x) and x>0 for x in (vne,mne)):
        raise ValueError('Aircraft speed redlines must be positive and finite')
    structural=bool(config['structural_limits'])
    candidates=[dict(kind='VNE',speed_kmh=vne,enforced=structural),
                dict(kind='Mach limit',speed_kmh=mne,enforced=structural)]
    flaps=flap_limits(flap_profile(fm),config.get('flaps_percent',0.))
    if flaps:
        candidates.append(dict(kind='Flap IAS limit',
            speed_kmh=flaps['destruction_ias_kmh']/factor,enforced=structural))
        if flaps['automatic_ias_kmh'] is not None:
            candidates.append(dict(kind='Flap automatic IAS limit',
                speed_kmh=flaps['automatic_ias_kmh']/factor,enforced=True))
        if flaps['automatic_mach'] is not None:
            candidates.append(dict(kind='Flap automatic Mach limit',
                speed_kmh=flaps['automatic_mach']*speed_of_sound(config['altitude_m'])*3.6,enforced=True))
    cutoff=sweep_cutoff_kmh(fm,config)
    if cutoff is not None:
        # The selected wing sweep is unavailable above this speed (game sweep schedule): a speed boundary.
        candidates.append(dict(kind='Wing sweep limit',speed_kmh=cutoff,enforced=True))
    active=[c for c in candidates if c['enforced']]
    # Each worker receives the same precomputed aircraft-specific RPM cutoff.
    rpm=(config.get('_over_rev_limits') or {}).get(config['aircraft'][0]) if len(config.get('aircraft',[]))==1 else None
    if rpm:
        candidate=dict(rpm,enforced=True)
        candidates.append(candidate);active.append(candidate)
    selected=min(active or candidates,key=lambda c:c['speed_kmh'])
    limit=selected['speed_kmh']


    inside=max(0.,limit-.0036)
    deployment=[]
    if flaps:
        if flaps['deployment_ias_kmh'] is not None:
            deployment.append(flaps['deployment_ias_kmh']/factor)
        if flaps['deployment_mach'] is not None:
            deployment.append(flaps['deployment_mach']*speed_of_sound(config['altitude_m'])*3.6)
    return dict(speed_kmh=limit,sample_speed_kmh=inside,
        kind=selected['kind'],vne_ias_kmh=strength['ias']*3.6,
        vne_tas_kmh=vne,mne_tas_kmh=mne,mne=strength['mach'],
        enforced=bool(active),flaps=flaps,candidates=candidates,
        deployment_speed_kmh=min(deployment,default=None),
        convention='Total-speed IAS converted to TAS; lowest enforced wing, Mach, flap damage or automatic flap-extension limit; native pointwise checks retained')


def excluded_column(solver, speed):
    if not hasattr(solver,'plot_speed_limits'):
        solver.plot_speed_limits=speed_limits(solver.fm,solver.config)
    limit=solver.plot_speed_limits
    if not limit['enforced'] or speed<=limit['sample_speed_kmh']:return None
    return dict(speed_kmh=float(speed),points=[],boundary=None,sustained=[],load_checks=[],
        boundary_bracket_g=None,boundary_reason=limit['kind'],boundary_status='speed limit',
        lower_boundary=None,numerical_gap_brackets=[],interior_failures=[],unresolved_load_intervals=[],
        elapsed_s=0.,mass=solver.mass,engine=solver.engine.summary)


def excluded_point(solver, speed, load):
    column=excluded_column(solver,speed)
    if column is None:return None
    return dict(speed_kmh=float(speed),load_g=float(load),valid=False,converged=False,
        reasons=[column['boundary_reason']],not_evaluated=True,evaluations=0,
        turn_dps=math.degrees(9.8100004196167*math.sqrt(max(0.,load*load-1.))/(speed/3.6)),
        ps_mps=None,ps_continuous_mps=None,component_forces={},
        flaps_percent=solver.config.get('flaps_percent',0.),flaps_requested_percent=solver.config.get('flaps_percent',0.),
        instructor_enabled=solver.config['instructor'])
