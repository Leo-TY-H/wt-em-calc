"""Ideal aircraft support for the website's recovered radar flight composition.

Aircraft motion and perfect sensor pointing are scenario assumptions. The
provider manager, inertial estimate, seeker and controls remain recovered code.
"""
from copy import deepcopy

from flight_session import digest
from geometric_radar import update as geometric_seeker, BOUND
from kernels import f32, sub
from optical_point_flight import update as flight_update
from radar_provider_controls import update as provider_controls
from radar_provider_properties import extend
from missile_initialization import radar_properties


def profile_with_support(profile):
    """Validate provider fields using the saved native channel, retaining policy."""
    from pathlib import Path
    out = deepcopy(profile)
    p = out['properties']
    launcher = radar_properties()[Path(out['asset']).name]
    ordinary = deepcopy(p['guidance'])
    # Exported launch profiles already contain the geometric distance override.
    # The provider loader checks ordinary channel semantics. Supply its saved
    # native mode channel for that check, then retain the original policy seeker.
    ordinary['seeker']['modes']['distance'] = deepcopy(launcher['modes']['distance'])
    expanded = extend(p['rocket'], ordinary, launcher_properties=launcher)
    expanded['seeker'] = p['guidance']['seeker']
    p['guidance'] = expanded
    out['properties_sha256'] = digest(p)
    return out


def ideal_context(launch, target, time, source_mask):
    """A present source at prescribed pose, with one current designation record.

    Support follows the launch position/velocity with constant attitude. Records
    are stamped at the current tick; relative velocity is relative to that source.
    Authored datalink eligibility still belongs to the native helper. There is no
    channel contention, clutter, signature loss or support outage in this policy.
    """
    origin = [f32(a+v*time) for a,v in zip(launch['position'],launch['velocity'])]
    velocity = list(map(f32,launch['velocity']))
    pose = dict(position=origin, quaternion=launch['quaternion'][:])
    source = next((s for s in (6,3,5,2,1,4,7,8,9,0) if source_mask & (1<<s)), None)
    records = [] if source is None else [dict(source=source, source_tag=0, target_id=-1, time=f32(time),
        origin_override=False, point_valid=True, velocity_valid=True, flag48=True, priority=True,
        origin=origin[:], point=target['position'][:], direction=[0.,0.,0.],
        relative_velocity=[sub(a,b) for a,b in zip(target['velocity'],velocity)])]
    return dict(provider=dict(available=lambda:True, pose=lambda:deepcopy(pose), velocity=lambda:velocity[:]),
                records=records, channel_counts=(0,0), lag_limit=f32(.2),
                source_direction=lambda kind,t,direction: list(direction), observation_range=True)


def supported_radar_step(launch, *, immutable_properties=False):
    """Return the transient step callback carried through frame/interaction copies."""
    launch = deepcopy(launch)
    prepared_source = None
    prepared_guidance = None

    def step(p, state, targets, dt):
        nonlocal prepared_source, prepared_guidance
        if p.get('policy') != 'geometric_radar_v1' or len(targets) != 1:
            raise ValueError('Ideal radar support requires one geometric-policy target')
        if not immutable_properties or prepared_source is not p['guidance']:
            guidance_properties = deepcopy(p['guidance'])
            # Inertial projection reaches the designation helper during coast
            # modes. Only these fixed policy parameters are prepared here.
            designation = guidance_properties['seeker']['modes']['designation']
            for name in ('distance','doppler'):
                axis = designation[name]
                axis.update(minimum=0. if name=='distance' else -BOUND, maximum=BOUND,
                            unambiguous=2*BOUND, signal_width_min=max(1.,axis['signal_width_min']))
            if immutable_properties:
                prepared_source, prepared_guidance = p['guidance'], guidance_properties
        else:
            guidance_properties = prepared_guidance

        def guidance(unused, history, old, new, elapsed, clocks, points, **kwargs):
            context = ideal_context(launch, points[0]['scene']['new_target'], new[13],
                                    guidance_properties['manager']['designation_mask'])
            return provider_controls(guidance_properties,history,old,new,elapsed,clocks,points,None,context,
                                     seeker_step=geometric_seeker,**kwargs)
        return flight_update(p,state,targets,dt,guidance_step=guidance)
    return step
