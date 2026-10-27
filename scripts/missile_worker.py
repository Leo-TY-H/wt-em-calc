"""Isolated website engagement worker; stdin request, NDJSON progress/result.

The UI supplies launcher-aircraft state at release. A coincident, body-aligned
mount supplies the world release record to the recovered launch processing.
Legacy API requests with `launch` retain their explicit post-release meaning.
Seekers use ready prelaunch initialization and recovered handoff; motor,
guidance, acquisition and proximity clocks start at release, not at ignition.
"""
if __name__ == '__main__':
    import os as _os
    if _os.environ.get('WT_MISSILE_ENGINE', 'native') != 'python':
        import json as _json
        import subprocess as _subprocess
        import sys as _sys
        from missile_engine import command as _command
        try:
            _request = _json.load(_sys.stdin)
            if isinstance(_request, dict):
                _request['_parent_pid'] = _os.getpid()
                _request['_supervisor_pid'] = _os.getppid()
            _status = _subprocess.run(_command('simulator'), input=_json.dumps(_request),
                                      text=True, encoding='utf-8').returncode
        except (RuntimeError, OSError, ValueError) as _error:
            print(_json.dumps(dict(type='error', error=f'{type(_error).__name__}: {_error}')), flush=True)
            _status = 1
        raise SystemExit(_status)

from copy import deepcopy
import json
import math
from pathlib import Path
import sys

MODEL = Path(__file__).with_name('missile_model')
sys.path.insert(0, str(MODEL))
from body_launch import construct, initialize as launch_body
from flight_session import FlightSession, linear_point
from frame_session import FrameSession, FORMAT as FRAME_FORMAT, STEP
from flight_limits import distance_setting
from guidance_launch import properties as launch_properties, initialize as launch_guidance
from interaction_session import InteractionSession, PointHistory
from kernels import f32, add
from missile_initialization import optical_ready, radar_ready
from missile_support import profile_with_support, supported_radar_step
from missile_tracking import describe as describe_tracking
import missile_telemetry
from missile_inputs import directory as input_directory
from missile_fast import FastInteractionSession
from missile_backend import activate

BACKEND = activate()
if BACKEND != 'reference':
    InteractionSession = FastInteractionSession


def quaternion(angles):
    heading, pitch, roll = [math.radians(x) / 2 for x in angles]
    def product(a, b):
        x,y,z,w=a; X,Y,Z,W=b
        return [w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
                w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z]
    return [f32(v) for v in product(product([0,-math.sin(heading),0,math.cos(heading)],
                          [0,0,math.sin(pitch),math.cos(pitch)]),
                          [math.sin(roll),0,0,math.cos(roll)])]


ESCAPE_GRAVITY = 9.81
ESCAPE_LOAD_G = (1.5, 15.)
ESCAPE_DELAY_S = (0., 60.)
# A simulation runs until an outcome (Leo, 2026-10-26): the longest missile lifetime is 180 s and expires just
# after it, so this default limit is never the one reached.
RUN_LIMIT_S = 300.


def escape_motion(target, escape, origin):
    """Target path of the no-escape zone: a level break away from the launch point.

    The target holds its course for escape['delay_s'], then turns at
    escape['load_g'] the shorter way until origin (the launch point) lies directly
    behind it, and flies straight on. The turn is level and coordinated: rate
    g sqrt(n^2 - 1) / V, banked acos(1 / n) into the turn. Speed and altitude stay
    constant, so the path has a closed form; target_point() evaluates it at any
    time after release.
    """
    position, velocity = list(target['position']), list(target['velocity'])
    if velocity[1] != 0: raise ValueError('An escaping target must fly level')
    delay, load = float(escape['delay_s']), float(escape['load_g'])
    if not ESCAPE_DELAY_S[0] <= delay <= ESCAPE_DELAY_S[1] or not ESCAPE_LOAD_G[0] <= load <= ESCAPE_LOAD_G[1]:
        raise ValueError('Escape load factor or delay out of range')
    def wrapped(angle):   # (-pi, pi]
        angle = math.fmod(angle, 2 * math.pi)
        if angle > math.pi: angle -= 2 * math.pi
        if angle <= -math.pi: angle += 2 * math.pi
        return angle
    speed = math.sqrt(velocity[0] * velocity[0] + velocity[2] * velocity[2])
    turn_point = [p + v * delay for p, v in zip(position, velocity)]
    heading = math.atan2(velocity[2], velocity[0])
    turn = wrapped(math.atan2(turn_point[2] - origin[2], turn_point[0] - origin[0]) - heading)   # head-on: toward +Z
    sine, cosine = math.sin(heading), math.cos(heading)
    rate = radius = 0.; end = delay; exit_point = turn_point[:]; run = velocity[:]; exit_heading = heading
    if speed > 0 and turn != 0:
        rate = math.copysign(ESCAPE_GRAVITY * math.sqrt(load * load - 1.) / speed, turn)
        radius = speed / rate
        sign = math.copysign(1., turn)
        def ahead(tau):
            """Turn still needed tau seconds into the turn until origin lies dead astern."""
            h = heading + rate * tau
            x, z = turn_point[0] + radius * (math.sin(h) - sine), turn_point[2] - radius * (math.cos(h) - cosine)
            return sign * wrapped(math.atan2(z - origin[2], x - origin[0]) - h)
        # First crossing in 1-degree steps of heading (a jump across +-180 degrees is no
        # crossing), then bisection. Without one in a full circle (origin close to the
        # turn), the turn ends on the bearing seen from its start.
        step = math.pi / 180. / abs(rate)
        duration = turn / rate; previous = ahead(0.)
        for k in range(1, 361):
            value = ahead(k * step)
            if value <= 0. and previous < math.pi / 2:
                low, high = (k - 1) * step, k * step
                for _ in range(60):
                    middle = (low + high) / 2
                    if ahead(middle) > 0.: low = middle
                    else: high = middle
                duration = high
                break
            previous = value
        end = delay + duration
        exit_heading = heading + rate * (end - delay)
        exit_point = [turn_point[0] + radius * (math.sin(exit_heading) - sine), turn_point[1],
                      turn_point[2] - radius * (math.cos(exit_heading) - cosine)]
        run = [speed * math.cos(exit_heading), 0., speed * math.sin(exit_heading)]
    return dict(kind='escape', position=position, velocity=velocity, quaternion=quaternion(target['angles']),
                load_g=load, delay_s=delay, speed=speed, start=delay, end=end, heading=heading, sine=sine, cosine=cosine,
                rate=rate, radius=radius, roll=math.copysign(math.degrees(math.acos(1. / load)), turn),
                turn_point=turn_point, exit_point=exit_point, run=run,
                run_quaternion=quaternion([math.degrees(exit_heading), 0., 0.]))


def target_point(motion, time):
    """Target position, velocity and attitude at time: linear_point() or an escape_motion() path."""
    if motion.get('kind') != 'escape': return linear_point(motion, time)
    if time <= motion['start']:
        position = [p + v * time for p, v in zip(motion['position'], motion['velocity'])]
        velocity, attitude = motion['velocity'], motion['quaternion']
    elif time < motion['end']:
        heading = motion['heading'] + motion['rate'] * (time - motion['start'])
        point, radius = motion['turn_point'], motion['radius']
        position = [point[0] + radius * (math.sin(heading) - motion['sine']), point[1],
                    point[2] - radius * (math.cos(heading) - motion['cosine'])]
        velocity = [motion['speed'] * math.cos(heading), 0., motion['speed'] * math.sin(heading)]
        attitude = quaternion([math.degrees(heading), 0., motion['roll']])
    else:
        delta = time - motion['end']
        position = [p + v * delta for p, v in zip(motion['exit_point'], motion['run'])]
        velocity, attitude = motion['run'], motion['run_quaternion']
    return dict(position=[f32(x) for x in position], velocity=[f32(x) for x in velocity],
                quaternion=[f32(x) for x in attitude])


def validate(request):
    if not isinstance(request, dict): raise ValueError('Expected a scenario object')
    index = json.loads((input_directory() / 'launch-profiles/index.json').read_text())
    profiles = {p['path'].removesuffix('.json'): p for p in index['profiles']}
    asset = request.get('missile')
    if not isinstance(asset, str) or asset not in profiles: raise ValueError('Select a supported missile')
    def number(value, label, low, high):
        if isinstance(value, bool) or not isinstance(value, (int,float)) or not math.isfinite(value) or not low <= value <= high:
            raise ValueError(f'{label} must be a finite number between {low} and {high}')
        return float(value)
    if 'launcher' in request and 'launch' in request:
        raise ValueError('Supply launcher aircraft state or legacy post-release launch state, not both')
    launch_key = 'launcher' if 'launcher' in request else 'launch'
    clean = dict(missile=asset)
    for role in (launch_key, 'target'):
        data = request.get(role)
        if not isinstance(data, dict): raise ValueError(f'{role} state is required')
        clean[role] = {}
        for name, bound in (('position', 200000), ('velocity', 2000), ('angles', 360)):
            values = data.get(name)
            if not isinstance(values, list) or len(values) != 3: raise ValueError(f'{role} {name} needs three components')
            clean[role][name] = [number(v, f'{role} {name}', -bound, bound) for v in values]
        if not 0 < clean[role]['position'][1] <= 30000: raise ValueError(f'{role} altitude must be above 0 and at most 30000 m')
        if not -90 <= clean[role]['angles'][1] <= 90: raise ValueError(f'{role} pitch must be between -90 and 90 degrees')
    clean['duration'] = number(request.get('duration', RUN_LIMIT_S), 'Duration', .1, RUN_LIMIT_S)
    early_exit = request.get('early_exit', True)
    if not isinstance(early_exit, bool): raise ValueError('Early-exit setting must be boolean')
    clean['early_exit'] = early_exit
    if math.dist(clean[launch_key]['position'], clean['target']['position']) < 10:
        raise ValueError('Initial launcher/target separation must be at least 10 m')
    escape = request.get('escape')
    if escape is not None:
        if not isinstance(escape, dict): raise ValueError('Escape settings must be an object')
        if clean['target']['velocity'][1] != 0: raise ValueError('An escaping target must fly level')
        clean['escape'] = dict(load_g=number(escape.get('load_g'), 'Escape load factor', *ESCAPE_LOAD_G),
                               delay_s=number(escape.get('delay_s', 0.), 'Escape delay', *ESCAPE_DELAY_S))
    return clean


def initial_frame(profile, launch, target=None, *, launcher_aircraft=False):
    if target is not None and profile['family']=='radar':
        profile = profile_with_support(profile)
    flight = FlightSession(profile)
    p = flight.properties
    args = profile['release_inputs']
    record = dict(position=launch['position'], velocity=launch['velocity'],
                  quaternion=quaternion(launch['angles']), omega=[0.,0.,0.], time=0.)
    body = construct(record)
    template = args['constructed_guidance']
    initialization = dict(policy='cold_acquisition', tracking=False)
    if target is not None and flight.family == 'optical':
        template, initialization = optical_ready(p['rocket'], p['guidance'], template, body, target)
    elif target is not None and flight.family == 'radar':
        target_body = dict(target, quaternion=quaternion(target['angles']))
        template, initialization = radar_ready(profile['asset'], p['guidance'], template, body, target_body)
    seed = 123
    guidance = launch_guidance(launch_properties(p['rocket'], p['guidance']),
                              template, body, seed)
    limit = distance_setting(p['rocket'])
    # Constructor/seeker handoff consumes the aircraft record BEFORE the native
    # release adjustment. In aircraft mode, apply authored start speed, spread
    # and delay-dependent transverse velocity processing exactly once. Aircraft
    # support continues to use the original input, never the processed missile.
    launched = launch_body(p['rocket'], body, seed, 0x12345678,
                           launch_time=0., clock_time=0., launch_mode=2 if launcher_aircraft else 0,
                           distance_limit=limit)
    flight.state = dict(body=launched['body'], guidance=guidance['state'], controls=deepcopy(args['controls']))
    frame = FrameSession(dict(format=FRAME_FORMAT, flight=flight.checkpoint(),
                             iteration_limit=15, expiry_flag=False, lost_tracking_clock=0.,
                             limits=dict(launch_time=0., lifetime=launched['lifetime'],
                                         distance_limit=limit, origin=body['position'][:])))
    frame.website_initialization = initialization
    frame.website_release = dict(input_state='launcher_aircraft' if launcher_aircraft else 'post_release',
        mount='coincident_body_aligned' if launcher_aircraft else None,
        launch_seed=seed, position=launched['body']['position'][:],
        velocity_mps=launched['body']['velocity'][:], quaternion=launched['body']['quaternion'][:],
        motor_clocks_s=launched['body']['clocks'][:])
    return frame


def closest_segment(m0, m1, t0, t1):
    r = [a-b for a,b in zip(m0,t0)]
    d = [(b-a)-(v-u) for a,b,u,v in zip(m0,m1,t0,t1)]
    squared = sum(x*x for x in d)
    fraction = min(1., max(0., -sum(a*b for a,b in zip(r,d))/squared)) if squared else 0.
    return math.sqrt(sum((a+fraction*b)**2 for a,b in zip(r,d))), fraction


def proximity_endpoint(event, old, new, target_at):
    """Presentation endpoint from the recovered event, without invented dynamics.

    Native integration retains its completed body sample even when the event
    lies earlier inside that interval. Keep that sample separately; no velocity,
    attitude or tracking history is inferred at the intermediate event time.
    The website's 48 Hz visits and default collision tolerance bracket this time.
    """
    time = event['time']
    if not old['time_s'] <= time <= new['time_s']:
        raise ValueError('Proximity timestamp lies outside the completed website flight interval')
    return dict(kind='proximity_event', time_s=time, missile=event['position'][:],
                target=target_at(time)['position'], velocity=None, speed_mps=None,
                quaternion=None, tracking=None, observation_available=None,
                tracking_state='event', loss_reason=None, mach=None, telemetry=None)


def simulate(request, progress=lambda _: None, cancelled=lambda: False):
    # Share the envelope's conservative miss certificate. This local import
    # avoids a module-initialization cycle with the envelope's lazy worker.
    from missile_envelope import unreachable, _burn_end
    config = validate(request)
    profile = json.loads((input_directory() / 'launch-profiles' / (config['missile']+'.json')).read_text())
    launcher_aircraft = 'launcher' in config
    launch = config['launcher' if launcher_aircraft else 'launch']
    frame = initial_frame(profile, launch, config['target'], launcher_aircraft=launcher_aircraft)
    initialization = frame.website_initialization
    session = InteractionSession.from_frame(frame, clock=0., origin=frame.website_release['position'],
                                            enabled=True, guidance_flag=False)
    escape = config.get('escape')
    target_motion = (escape_motion(config['target'], escape, launch['position']) if escape is not None else
                     dict(kind='constant_velocity', position=config['target']['position'],
                          velocity=config['target']['velocity'], quaternion=quaternion(config['target']['angles'])))
    straight_from = 0. if escape is None else target_motion['end']
    straight_velocity = config['target']['velocity'] if escape is None else target_motion['run']
    burn_end = _burn_end(frame.flight.properties['guidance']['motors'])
    limits = frame.limits
    deadline = limits['launch_time'] + limits['lifetime']
    reach_radius = float(session.props.get('radius', 0.) or 0.) + 30.
    def target(t): return target_point(target_motion, t)
    def targets(old, next_time):
        new = target(next_time)
        if frame.flight.family == 'optical': return [dict(position=new['position'], auxiliary=[0.,0.,0.], id=123)]
        return [dict(id=123, unit=1, unit_type=1, scene=dict(old_target=target(old['time']), new_target=new))]
    radar_step = (supported_radar_step(dict(launch, quaternion=quaternion(launch['angles'])),
                                     immutable_properties=BACKEND != 'reference')
                  if frame.flight.family=='radar' else None)
    history = PointHistory(lambda t: target(t)['position'], lambda t: target(t)['quaternion'])
    rows = []; transitions = []; traveled_distance = 0.
    inertial_enabled = bool(frame.flight.properties['rocket']['guidance'].get('inertialNavigation',False))
    tracking_status = describe_tracking(frame.flight.family,session.frame.flight.state['guidance']['manager'],
                                       inertial_enabled,initialization=initialization)
    seeker_transitions = [dict(time_s=0.,**tracking_status)]
    best = dict(distance_m=math.dist(frame.website_release['position'],config['target']['position']), time_s=0.)
    tracked_before = initialization['tracking']
    if tracked_before: transitions.append(dict(time_s=0., tracking=True))
    def sample():
        body = session.frame.flight.state['body']
        return dict(time_s=body['time'], missile=body['position'][:], target=target(body['time'])['position'],
                    velocity=body['velocity'][:], speed_mps=math.sqrt(sum(v*v for v in body['velocity'])),
                    quaternion=body['quaternion'][:], mach=missile_telemetry.mach(body),
                    traveled_distance_m=traveled_distance, **tracking_status)
    rows.append(dict(sample(),telemetry=missile_telemetry.initial(frame.flight.properties)))
    outcome = dict(code='time_limit', label='Time limit reached — unresolved')
    count = math.ceil(config['duration'] / STEP)
    for i in range(count):
        if cancelled(): raise InterruptedError('Simulation cancelled')
        old = sample(); now = add(old['time_s'], STEP)
        before = deepcopy(session.frame.flight.state['body'])
        controls = session.frame.flight.state['controls'][:]
        result = session.advance(now, STEP, targets, collision_time=now, point_history=history,
                                 flight_step=radar_step)
        previous_tracking_status = tracking_status
        tracking_status = describe_tracking(frame.flight.family,session.frame.flight.state['guidance']['manager'],
            inertial_enabled,guidance=result['frame']['updates'][-1]['guidance'])
        new = sample()
        event = result.get('event')
        endpoint = proximity_endpoint(event,old,new,target) if event else new
        traveled_distance += math.dist(old['missile'], endpoint['missile'])
        endpoint['traveled_distance_m'] = traveled_distance
        distance, fraction = closest_segment(old['missile'],endpoint['missile'],old['target'],endpoint['target'])
        if distance < best['distance_m']:
            best = dict(distance_m=distance,time_s=old['time_s']+fraction*(endpoint['time_s']-old['time_s']))
        if not event and new['tracking'] != tracked_before:
            transitions.append(dict(time_s=new['time_s'], tracking=new['tracking']))
            tracked_before = new['tracking']
        terminal = False
        if event:
            outcome = dict(code='proximity_event', label='Target proximity event', event=event); terminal=True
        elif new['missile'][1] <= 0:
            outcome = dict(code='ground', label='Missile reached ground'); terminal=True
        elif new['target'][1] <= 0:
            outcome = dict(code='target_ground', label='Target reached ground — scenario ended'); terminal=True
        elif session.frame.expiry_flag:
            expiry = result['frame']['expiry']
            code = 'lifetime' if expiry['time_expired'] else 'distance_limit' if expiry['distance_expired'] else 'tracking_timeout'
            labels = dict(lifetime='Missile lifetime expired',distance_limit='Missile distance limit reached',tracking_timeout='Geometric tracking timeout')
            outcome = dict(code=code,label=labels[code]); terminal=True
        elif config['early_exit'] and i % 6 == 5 and new['time_s'] >= straight_from:
            body = session.frame.flight.state['body']
            if (all(c >= end + 1e-3 for c, end in zip(body['clocks'], burn_end))
                    and unreachable(new['missile'], new['speed_mps'], new['target'], straight_velocity,
                                    deadline - new['time_s'] + 2 * STEP, reach_radius,
                                    limits['origin'], limits['distance_limit'])):
                outcome = dict(code='unreachable', label='Miss — target is unreachable'); terminal=True
        if event:
            seeker_transitions.append(dict(time_s=endpoint['time_s'],tracking=None,
                observation_available=None,tracking_state='event',loss_reason=None))
        elif tracking_status != previous_tracking_status:
            seeker_transitions.append(dict(time_s=new['time_s'],**tracking_status))
        if i % 3 == 2 or terminal or i == count-1:
            new['telemetry'] = missile_telemetry.completed(session.frame.flight.properties,
                before, controls, result['frame']['updates'][-1], STEP)
            if event:
                new['traveled_distance_m'] = old['traveled_distance_m'] + math.dist(old['missile'],new['missile'])
            if event and rows[-1]['time_s']==endpoint['time_s']: rows[-1]=endpoint
            else: rows.append(endpoint)
        if i % 24 == 23: progress(dict(time_s=now, duration_s=config['duration'], fraction=min(1.,now/config['duration'])))
        if terminal: break
    return dict(scenario=config, release=frame.website_release, trajectory=rows, tracking_transitions=transitions,
                seeker_transitions=seeker_transitions,
                closest_approach=best, outcome=outcome, elapsed_s=rows[-1]['time_s'],
                simulation_time_s=new['time_s'], final_body_sample=new,
                model=dict(profile_build='2.59.0.34', property_loading=profile.get('property_loading'), policy=frame.flight.policy,
                           step_s=STEP, launch_state=frame.website_release['input_state'], seeker_initialization=initialization['policy'],
                           time_origin='release', units=dict(position='m', velocity='m/s', angles='degrees', time='s'),
                           initialization=initialization,
                           radar_support='ideal_recovered_provider' if frame.flight.family=='radar' else None,
                           component_provenance='2.59.0.34 core and 2.59.0.38 shared arithmetic, with scoped 2.59.0.87 projectile-loader corrections; whole-build equivalence unproven',
                           target_motion='constant_velocity' if escape is None else dict(escape, kind='level_break_away'),
                           geometry='point_target',
                           telemetry=dict(force_sampling='Final integration substep; force_time_s records its evaluation time. Thrust is the actual propulsion update held over that step.',
                               aerodynamic_forces='Magnitudes of pre-fin translational drag/lift including body perturbation scaling; Cd is the pre-fin drag coefficient.',
                               aoa='Unsigned local-flow incidence to the aerodynamic forward axis, 0–180 degrees.',
                               acceleration='Magnitude of integrated net acceleration, including gravity; not aerodynamic load factor.',
                               missing='No integrated forces at release; no dynamics inferred at an intermediate proximity event.',
                               distance='Sum of 48 Hz missile position segments, truncated to the event position when present.'),
                           trajectory_endpoint='Recovered proximity-event position/time when present; completed body sample retained separately.',
                           early_miss_termination='After motor burnout and any target turn, stop if the conservative energy/lifetime/distance bound excludes every future hit. Closest approach covers the simulated portion. Disable with early_exit=false.',
                           limitations=['Whole engagements are not yet validated against live-game flights.',
                                        ('Launcher mount is coincident and aligned with the aircraft; aircraft-specific hardpoint offsets, ejector impulses and angular rates are not supplied.'
                                         if launcher_aircraft else 'Legacy launch inputs are the missile state after release; release speed/spread is not applied again.'),
                                        ('Optical launch assumes a ready seeker at its first acquisition observation; prior tracking history is unspecified.'
                                         if frame.flight.family=='optical' else
                                         'Radar support follows the launch position and velocity with constant attitude; aircraft sensor hardware is idealized.'),
                                        'Point proximity events do not model aircraft damage or physical mesh impact.',
                                        'Collision events are currently consumed in the same visit; native outer-loop timing is not connected.']))


def main():
    try:
        request = json.load(sys.stdin)
        result = simulate(request, lambda p: print(json.dumps(dict(type='progress', progress=p)), flush=True))
        print(json.dumps(dict(type='result', result=result), allow_nan=False), flush=True)
    except Exception as error:
        print(json.dumps(dict(type='error', error=f'{type(error).__name__}: {error}')), flush=True)
        raise SystemExit(1)


if __name__ == '__main__': main()
