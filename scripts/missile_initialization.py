"""Website launch policy composed from recovered seeker and handoff routines.

The user starts with a ready launcher, at its first acquisition observation.
No unspecified prelaunch aircraft maneuver or filter history is fabricated.
Designation is ideal support, but authored source masks and angular limits apply.
Radar initialization uses two ready updates of the prescribed linear motion.
"""
from copy import deepcopy
from functools import lru_cache
import json
from pathlib import Path
from missile_inputs import directory as input_directory
import struct

from kernels import f32, sub
from frame_session import STEP
from geometric_optical_observation import observe
from optical_modes import update
from optical_prelaunch import initial, constructed_guidance, properties
from radar_activation import world_to_body
from seeker_designation import SOURCE_NAMES
from radar_activation import activate as activate_radar
from radar_prelaunch import update as update_radar_launcher, seeker_update as update_radar_seeker, body_frame
from radar_modes import update as radar_modes
from radar_handoff import constructed_guidance as radar_handoff
from launcher_inertial import initial as inertial_initial
from geometric_radar import measure as radar_measure, BOUND


@lru_cache(maxsize=1)
def radar_properties():
    return json.loads((input_directory() / 'radar-launch-properties.json').read_text())['properties']


def radar_ready(asset, guidance, template, body, target):
    """Native launcher activation, ready updates and constructor handoff.

    Warm-up is complete before the two supplied 48 Hz prelaunch intervals.
    Both actors follow their entered constant velocities over those intervals,
    with constant attitude. Signal/range/Doppler rejection is excluded by the
    website policy; source masks and head geometry remain native. No prediction
    preview or aircraft-associated-unit lookup is requested.
    """
    p = deepcopy(radar_properties()[Path(asset).name])
    p['modes']['shared']['gate_rate'] = BOUND
    for name in ('distance', 'doppler'):
        p['modes'][name].update(minimum=0. if name=='distance' else -BOUND,
                               maximum=BOUND, width=0., limit_timeout=BOUND)
        for designated in (p['seeker'], p['modes']['designation']):
            designated[name].update(minimum=0. if name=='distance' else -BOUND,
                                    maximum=BOUND, unambiguous=2*BOUND,
                                    signal_width_min=max(1., designated[name]['signal_width_min']))
    source = next((s for s in (6, 3, 5, 2, 1, 4, 7, 8, 9, 0) if p['primary_mask'] & (1 << s)), None)

    def packet(t):
        position = [f32(a+v*t) for a,v in zip(body['position'],body['velocity'])]
        return [*position,*body['quaternion'],*body['velocity'],0.,0.,0.,f32(t)]

    def records(t):
        if source is None: return []
        return [dict(source=source, source_tag=0, target_id=-1, time=f32(t),
                     origin_override=False, point_valid=True, velocity_valid=True,
                     flag48=True, priority=True, origin=[0.,0.,0.], direction=[0.,0.,0.],
                     point=[f32(a+v*t) for a,v in zip(target['position'],target['velocity'])],
                     relative_velocity=[sub(a,b) for a,b in zip(target['velocity'],body['velocity'])])]

    start = body_frame(packet(-2*STEP))
    reset = radar_modes(p['modes'], template['seeker'], body_frame(packet(-3*STEP)),
                        start, lambda _: {}, 0., mode=0)
    state = dict(phase=0, activation_time=-float.fromhex('0x1.fffffep127'), strength=0.,
                 association_flag=False, seeker=reset['state'], inertial=inertial_initial(),
                 output=dict(half_angle=0., direction=[1.,0.,0.], range=-1., phase=-1,
                             strength=0., use_target_id=False, associated_id=-1))
    state = activate_radar(p, state, start, records(-2*STEP))['state']
    state.update(phase=0x50 if p['lock_after_launch'] else 0x30, activation_time=f32(-2*STEP))
    observations = []

    def seeker(request, seeker_state, inertial_state):
        old, new = request['old_frame'], request['new_frame']
        t = new['time']
        point = dict(position=[f32(a+v*t) for a,v in zip(target['position'],target['velocity'])],
                     velocity=list(map(f32,target['velocity'])), quaternion=target['quaternion'])
        targets = [dict(id=123, scene=dict(new_target=point))]
        light = dict(present=True, frame=[1.,0.,0.,0.,1.,0.,0.,0.,1.,*new['position']], velocity=body['velocity'])
        def measure(temporary):
            answer = radar_measure(guidance['seeker'], temporary, old, new, light, targets)
            observations.append(answer)
            return answer
        return update_radar_seeker(p, request, seeker_state, measure)

    for before, after in ((-2*STEP,-STEP),(-STEP,0.)):
        observations.clear()
        result = update_radar_launcher(p,state,packet(before),packet(after),records(after),seeker)
        state = result['state']
    handed = radar_handoff(state, template, lock_after_launch=p['lock_after_launch'])
    ready = bool(result['calls'] and result['calls'][-1]['answer']['outputs']['tracking'])
    diagnostics = dict(policy='radar_ready_handoff', tracking=state['phase']==0x40,
                       observation_available=observations[-1]['available'] if observations else None,
                       ready_for_launch=ready,
                       lock_after_launch=p['lock_after_launch'], launcher_phase=state['phase'],
                       designation_source=SOURCE_NAMES[source] if source is not None else None,
                       inertial_valid=state['inertial']['valid'],
                       angles_rad=deepcopy(handed['seeker']['angles']),
                       loss_reason=(observations[-1]['loss_reason'] if observations else
                                    None if ready else 'designation_geometry' if source is not None else 'no_designation'),
                       history='Warm-up complete; two prelaunch updates with constant velocity and attitude.')
    return handed, diagnostics


def optical_ready(rocket, guidance, template, body, target):
    """Acquire once at the supplied pose, then use the native constructor copy.

    Warm-up is assumed complete. Acquisition uses mode 4, including its tighter
    lock-angle bound, rather than injecting a mode-8 track at arbitrary angles.
    Initial acquisition reseeds filter rate just as the recovered routine does.
    Failed acquisition still hands off the actual head state for in-flight search.
    """
    p = properties(rocket, guidance)  # Reject unsupported launcher families.
    launcher = initial()
    q = body['quaternion']
    ss = launcher['seeker']
    # Reset in the supplied body frame. Constructor +X is not world boresight
    # for a rotated launch, and must not leak from a reference trajectory.
    reset = update(p['seeker'], ss['state'], q, -2*STEP, -STEP,
                   lambda _: {}, mode=0, opaque_state=ss['opaque_state'])
    ss = {key: reset[key] for key in ('state', 'opaque_state')}
    raw = ss['opaque_state']
    mask = p['seeker']['designation_mask']
    source = next((s for s in (6, 3, 5, 2, 1, 4, 7, 8, 9, 0) if mask & (1 << s)), None)
    designation = None
    if source is not None:
        # The pointing routine uses direction ratios; a world displacement is
        # sufficient. No rate history exists at first acquisition.
        delta = [sub(a, b) for a, b in zip(target['position'], body['position'])]
        designation = dict(source=source, direction=world_to_body(q, delta), angular_rate=[0., 0., 0.])
    observations = []

    def measurement(temporary):
        os = dict(temporary, reject_time=struct.unpack('<f', bytes(raw[24:28]))[0],
                  target_id=struct.unpack('<i', bytes(raw[28:32]))[0])
        measured = observe(p['observation'], os, q, body['position'],
                           [dict(position=target['position'], auxiliary=[0., 0., 0.], id=123)],
                           dt=STEP, new_time=0.)
        observations.append(measured)
        changed = raw[:]
        changed[24:28] = struct.pack('<f', measured['state']['reject_time'])
        changed[28:32] = struct.pack('<i', measured['state']['target_id'])
        if measured['accepted']: changed[0] = 0
        return dict(available=measured['accepted'], measurement=measured['measurement'],
                    distance=measured['state']['distance'], strength=measured['strength'],
                    flag=measured['flag'], count=measured['reported_target_count'],
                    auxiliary=measured['auxiliary'], opaque_state=changed,
                    refresh_los_cache=not temporary['los_cache_valid'] and measured['state']['los_cache_valid'])

    acquired = update(p['seeker'], ss['state'], q, -STEP, 0., measurement,
                      mode=4, opaque_state=raw, designation=designation,
                      los_check_timeout=p['los_check_timeout'])
    launcher['seeker'] = {key: acquired[key] for key in ('state', 'opaque_state')}
    tracking = bool(acquired['outputs']['tracking'])
    launcher.update(phase=0x40 if tracking else 0x30, activation_time=f32(-STEP))
    state = constructed_guidance(launcher, template)
    diagnostics = dict(policy='optical_ready_acquisition', tracking=tracking,
                       observation_available=observations[-1]['accepted'],
                       designation_source=SOURCE_NAMES[source] if source is not None else None,
                       angles_rad=deepcopy(state['seeker']['state']['angles']),
                       loss_reason=observations[-1]['loss_reason'],
                       history='First acquisition observation at launch pose; warm-up complete.')
    return state, diagnostics
