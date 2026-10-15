import math
from air_state import cache,speed_of_sound
from wing_sweep import prepare,select
from flap_model import profile as flap_profile, limits as flap_limits


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
