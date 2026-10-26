"""Launch-envelope search over the website's recovered missile engagement model.

Every sample is the website engagement (the same session, target, support and
terminal rules as missile_worker.simulate) against a constant-velocity point
target. A launch is a hit when the missile's proximity fuze fires, exactly as in
the simulator tab, so fuze arming and radius shape the envelope together with
the motor, mass, drag, lift, control limits, guidance and seeker limits.

The envelope is drawn around the target, which flies straight and level along
+X. Air-to-air: layers are altitudes shared by both aircraft; the launcher is
always at the target's altitude, flying level and pointed at it, so the missile
leaves heading straight at the target and lofts only by its own guidance.
Surface-to-air: the launcher stands still on the ground (GROUND_LAUNCH_M) with
the missile pointed at the target, and layers are target altitudes
(Leo, 2026-10-26). Azimuth is the launcher's bearing from the target's nose
(0 = head-on, 180 = tail chase).

frame='launcher' (air-to-air only) is the shooter-centred launch acceptable
region: the launcher flies level along +X at the layer altitude and keeps that
heading, and the target, at the same altitude, sits at a bearing off the
launcher's nose (azimuth; positive toward +Z, the launcher's right). The target
flies level toward the launcher ('hot'), away from it ('cold'), along +X
('same'), along -X ('opposite'), or at a fixed heading in degrees relative to
the launcher's. Were the launcher to turn toward the target, this view would
only be the target frame turned 180 degrees; keeping the heading shows the
off-boresight limits: the missile's launch zone (launch_angles), an optional
gimbal limit of the launching aircraft, and the seeker's own limits in flight.
Bearings outside the launch zone are not launched at (and not simulated).

Each layer is sampled along horizontal rays from the aircraft at the centre.
Along a ray the launch distance is searched for its hit/miss transitions; the
nearest and farthest hit boundaries are the minimum and maximum launch ranges.
Rays start on a regular azimuth grid (Planner). Between two finished neighbours
another ray is added where one hits and the other does not, down to the edge
step, and where their gaps between hits differ or their boundaries bend, down to
the detail step. Boundary changes are judged against the envelope's own size, so the
outline is resolved where it bends or ends while directions without hits, and
scattered hits close to the target, cost little.

No-escape zone (Leo, 2026-10-26): with config['escape'] a second, independent
search runs against a target that breaks away at launch (missile_worker.
escape_motion: after the reaction delay a level, constant-speed turn at the escape
load factor until the launch point lies dead astern, then straight on), in either
frame. Its rays carry motion='escape'. The no-escape zone is where both searches
hit: the target cannot escape by holding its course or by breaking away.

Only presentation work is removed from the flight loop. A miss may be declared
early once every motor has burned out, the target flies straight, and an energy
bound proves that the missile cannot reach the target's future path before its
lifetime or distance limit; `early_exit=False` runs every flight to its modeled
terminal condition.
"""
from concurrent.futures import FIRST_COMPLETED, ProcessPoolExecutor, wait
import json
import math
import os
import time

STEP_GRAVITY = 9.81
SLANT_MINIMUM_M = 150.
HORIZONTAL_MINIMUM_M = 50.
PLATFORMS = ('air', 'ground')
FRAMES = ('target', 'launcher')
HEADINGS = ('hot', 'cold', 'same', 'opposite')
GROUND_LAUNCH_M = 2.                # surface-to-air launch height above flat ground
RANGE_CAP_M = 195000.
COARSE_RATIO = 1.6
INTERIOR = 4.
SHAPE = 5.
MATCH = 2.5
MAX_LAYERS = 25
ESCAPE_DEFAULT_G = 9.               # default escape load factor (Leo, 2026-10-26)
QUALITY = dict(quick=dict(azimuth_step_deg=30., detail_step_deg=3.75, edge_step_deg=1., tolerance=.03, tolerance_m=50.),
               standard=dict(azimuth_step_deg=15., detail_step_deg=1.875, edge_step_deg=.5, tolerance=.015, tolerance_m=30.),
               fine=dict(azimuth_step_deg=10., detail_step_deg=1.25, edge_step_deg=.25, tolerance=.0075, tolerance_m=15.))

_worker = None
_profiles = {}


def _missile_worker():
    """Import the website engagement module lazily (one model per process)."""
    global _worker
    if _worker is None:
        import missile_worker
        _worker = missile_worker
    return _worker


def _profile(missile):
    if missile not in _profiles:
        worker = _missile_worker()
        _profiles[missile] = json.loads((worker.input_directory() / 'launch-profiles' / (missile + '.json')).read_text())
    return _profiles[missile]


def profiles():
    from missile_inputs import directory
    index = json.loads((directory() / 'launch-profiles/index.json').read_text())
    return {p['path'].removesuffix('.json'): p for p in index['profiles']}


def validate(request):
    if not isinstance(request, dict): raise ValueError('Expected an envelope request object')
    def number(value, label, low, high):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
            raise ValueError(f'{label} must be a finite number between {low:g} and {high:g}')
        return float(value)
    missile = request.get('missile')
    if not isinstance(missile, str) or missile not in profiles(): raise ValueError('Select a supported missile')
    frame = request.get('frame', 'target')
    if frame not in FRAMES: raise ValueError("Frame must be 'target' or 'launcher'")
    launcher = request.get('launcher') or {}; target = request.get('target') or {}
    if not isinstance(launcher, dict) or not isinstance(target, dict): raise ValueError('Launcher and target settings are required')
    platform = launcher.get('platform', 'air')
    if platform not in PLATFORMS: raise ValueError("Launcher platform must be 'air' or 'ground'")
    if frame == 'launcher' and platform != 'air': raise ValueError('The launcher-centred envelope is air-to-air only')
    clean = dict(missile=missile, frame=frame,
                 launcher=dict(platform=platform,
                               speed_mps=0. if platform == 'ground' else number(launcher.get('speed_mps'), 'Launcher speed', 0., 2000.)),
                 target=dict(speed_mps=number(target.get('speed_mps'), 'Target speed', 0., 2000.)))
    if frame == 'launcher':
        heading = target.get('heading', 'hot')
        if isinstance(heading, str):
            if heading not in HEADINGS: raise ValueError('Unknown target heading rule')
        else: heading = number(heading, 'Target heading', -180., 180.)
        clean['target']['heading'] = heading
        # Optional off-nose limit of the launching aircraft's sensor (Leo, 2026-10-26).
        gimbal = launcher.get('gimbal_deg')
        clean['launcher']['gimbal_deg'] = None if gimbal is None else number(gimbal, 'Launcher gimbal limit', 1., 180.)
    layers = request.get('layers_m')
    if not isinstance(layers, list) or not 1 <= len(layers) <= MAX_LAYERS:
        raise ValueError(f'Supply between 1 and {MAX_LAYERS} layer altitudes')
    clean['layers_m'] = sorted({number(v, 'Layer altitude', 50., 25000.) for v in layers})
    quality = QUALITY.get(request.get('quality', 'standard'))
    if quality is None: raise ValueError('Unknown envelope quality')
    step = number(request.get('azimuth_step_deg', quality['azimuth_step_deg']), 'Azimuth step', 2.5, 45.)
    if abs(180. / step - round(180. / step)) > 1e-9: raise ValueError('Azimuth step must divide 180 degrees')
    clean['azimuth_step_deg'] = step
    clean['detail_step_deg'] = number(request.get('detail_step_deg', min(step, quality['detail_step_deg'])), 'Detail step', .1, step)
    clean['edge_step_deg'] = number(request.get('edge_step_deg', min(clean['detail_step_deg'], quality['edge_step_deg'])),
                                    'Edge step', .1, clean['detail_step_deg'])
    clean['tolerance'] = number(request.get('tolerance', quality['tolerance']), 'Relative tolerance', .002, .2)
    clean['tolerance_m'] = number(request.get('tolerance_m', quality['tolerance_m']), 'Absolute tolerance', 5., 1000.)
    clean['max_range_m'] = number(request.get('max_range_m', RANGE_CAP_M), 'Maximum range', 1000., RANGE_CAP_M)
    clean['early_exit'] = bool(request.get('early_exit', True))
    escape = request.get('escape')
    if escape is not None and not isinstance(escape, dict): raise ValueError('Escape settings must be an object')
    clean['escape'] = None if escape is None else dict(
        load_g=number(escape.get('load_g', ESCAPE_DEFAULT_G), 'Escape load factor', 1.5, 15.),
        delay_s=number(escape.get('delay_s', 0.), 'Escape reaction delay', 0., 60.))
    return clean


def mirrored(config):
    """Left/right symmetric setups are searched on one side and mirrored.

    Target frame: the target flies along +X and the launcher always heads toward
    it, so every envelope is symmetric about the target's track. Launcher frame:
    symmetric about the launcher's track unless the target crosses it at a
    fixed heading.
    """
    if config['frame'] == 'target': return True
    heading = config['target']['heading']
    return isinstance(heading, str) or heading % 180. == 0.


def azimuths(config):
    """Base azimuth grid; refinement adds rays between these."""
    step = config['azimuth_step_deg']
    count = round(180. / step)
    if mirrored(config): return [i * step for i in range(count + 1)]
    return [-180. + i * step for i in range(1, 2 * count + 1)]


def _direction(heading, pitch, speed):
    h, p = math.radians(heading), math.radians(pitch)
    return [speed * math.cos(p) * math.cos(h), speed * math.sin(p), speed * math.cos(p) * math.sin(h)]


def _wrap(angle):
    angle = math.fmod(angle, 360.)
    if angle > 180.: angle -= 360.
    if angle <= -180.: angle += 360.
    return angle


def states(config, layer, azimuth, distance, escape=False):
    """Launcher and target states at release for one envelope sample (SI, degrees).

    The launcher is always released above the world origin, as in the missile
    simulator tab. The model computes in binary32 world coordinates, so near a
    boundary the outcome can depend on where the same geometry is placed;
    a shared placement lets the simulator tab replay any envelope sample exactly.
    With escape the target dict also carries the escape settings for evaluate().
    """
    a = math.radians(azimuth)
    if config['frame'] == 'launcher':
        # Both at the layer altitude; the launcher keeps flying along +X whatever the target's bearing.
        launcher = dict(position=[0., layer, 0.], velocity=_direction(0., 0., config['launcher']['speed_mps']), angles=[0., 0., 0.])
        rule = config['target']['heading']
        heading = _wrap(dict(hot=azimuth + 180., cold=azimuth, same=0., opposite=180.)[rule] if isinstance(rule, str) else rule)
        target = dict(position=[distance * math.cos(a), layer, distance * math.sin(a)],
                      velocity=_direction(heading, 0., config['target']['speed_mps']), angles=[heading, 0., 0.])
        if escape: target['escape'] = dict(config['escape'])
        return launcher, target
    target = dict(position=[-distance * math.cos(a), layer, -distance * math.sin(a)],
                  velocity=_direction(0., 0., config['target']['speed_mps']), angles=[0., 0., 0.])
    if escape: target['escape'] = dict(config['escape'])
    heading = _wrap(azimuth + 180.)
    if config['launcher']['platform'] == 'ground':
        # Stationary on the ground, the missile pointed at the target.
        pitch = math.degrees(math.atan2(layer - GROUND_LAUNCH_M, distance))
        launcher = dict(position=[0., GROUND_LAUNCH_M, 0.], velocity=[0., 0., 0.], angles=[heading, pitch, 0.])
    else:
        # Both aircraft at the layer altitude; the launcher flies level, pointed at the target.
        launcher = dict(position=[0., layer, 0.], velocity=_direction(heading, 0., config['launcher']['speed_mps']),
                        angles=[heading, 0., 0.])
    return launcher, target


def vertical_offset(config, layer):
    """Altitude of the launcher above the target: zero air-to-air, below it surface-to-air."""
    if config['launcher']['platform'] == 'ground':
        return GROUND_LAUNCH_M - layer
    return 0.


def _intervals(a, b, c, low, high):
    """Subintervals of [low, high] where a t^2 + b t + c <= 0."""
    roots = []
    if abs(a) > 1e-12:
        disc = b * b - 4 * a * c
        if disc >= 0:
            s = math.sqrt(disc)
            roots = sorted(((-b - s) / (2 * a), (-b + s) / (2 * a)))
    elif abs(b) > 1e-12:
        roots = [-c / b]
    cuts = [low] + [r for r in roots if low < r < high] + [high]
    out = []
    for x, y in zip(cuts, cuts[1:]):
        m = (x + y) / 2
        if a * m * m + b * m + c <= 0: out.append((x, y))
    for r in roots:
        if low <= r <= high: out.append((r, r))
    return out


def unreachable(position, speed, target_position, target_velocity, remaining, radius, origin, distance_limit):
    """True when no admissible future missile path can meet the target point.

    After burnout, thrust is zero and gravity is conservative while drag only
    removes energy; lift is normal to the flow. Speed is therefore bounded by
    sqrt(v^2 + 2 g h) with h >= 0 above flat ground. The missile must also stay
    within its distance limit of the launch origin. The bound carries margins.
    """
    bound = math.sqrt(speed * speed + 2. * STEP_GRAVITY * max(position[1], 0.)) * 1.02 + 5.
    d = [t - p for t, p in zip(target_position, position)]
    v2 = sum(v * v for v in target_velocity); dv = sum(x * v for x, v in zip(d, target_velocity))
    # |d + V tau| <= U tau + R  (both sides nonnegative for tau >= 0)
    reach = _intervals(v2 - bound * bound, 2. * (dv - bound * radius), sum(x * x for x in d) - radius * radius, 0., remaining)
    if not reach: return True
    if distance_limit <= 0: return False
    e = [t - o for t, o in zip(target_position, origin)]
    limit = distance_limit + radius + bound * .05
    inside = _intervals(v2, 2. * sum(x * v for x, v in zip(e, target_velocity)), sum(x * x for x in e) - limit * limit, 0., remaining)
    return not any(max(a0, b0) <= min(a1, b1) for a0, a1 in reach for b0, b1 in inside)


def _burn_end(motors):
    ends = []
    for motor in motors:
        if isinstance(motor, dict):
            delay = motor.get('delay', 0.); durations = [i.get('duration', 0.) for i in motor.get('impulses', [])]
        else:
            delay = motor.delay; durations = [i.duration for i in motor.impulses]
        ends.append(delay + sum(durations))
    return ends


def evaluate(missile, launcher, target, early_exit=True, reach_m=None):
    """One website engagement reduced to its outcome. Physics path = simulate().

    The envelope always leaves reach_m unset: a hit is the native proximity
    event. With reach_m (development comparisons only) the proximity fuze is
    disabled (it only consumes collision candidates; flight dynamics are
    unchanged) and the flight ends 'reached' once the missile passes within
    reach_m of the target.

    A target with 'escape' settings breaks away at launch (missile_worker.
    escape_motion); release and seeker setup see its state at release.
    """
    w = _missile_worker()
    from frame_session import STEP
    from kernels import add
    profile = _profile(missile)
    escape = target.get('escape')
    if escape is not None: target = dict(position=target['position'], velocity=target['velocity'], angles=target['angles'])
    frame = w.initial_frame(profile, launcher, target, launcher_aircraft=True)
    session = w.InteractionSession.from_frame(frame, clock=0., origin=frame.website_release['position'],
                                              enabled=reach_m is None, guidance_flag=False)
    if escape is None:
        motion = dict(kind='constant_velocity', position=target['position'], velocity=target['velocity'],
                      quaternion=w.quaternion(target['angles']))
        straight, straight_velocity = 0., target['velocity']
    else:
        # The energy bound needs straight-line target motion: after the turn only.
        motion = w.escape_motion(target, escape, launcher['position'])
        straight, straight_velocity = motion['end'], motion['run']
    def target_at(t): return w.target_point(motion, t)
    optical = frame.flight.family == 'optical'
    def targets(old, next_time):
        new = target_at(next_time)
        if optical: return [dict(position=new['position'], auxiliary=[0., 0., 0.], id=123)]
        return [dict(id=123, unit=1, unit_type=1, scene=dict(old_target=target_at(old['time']), new_target=new))]
    radar_step = (w.supported_radar_step(dict(launcher, quaternion=w.quaternion(launcher['angles'])),
                                         immutable_properties=w.BACKEND != 'reference')
                  if frame.flight.family == 'radar' else None)
    history = w.PointHistory(lambda t: target_at(t)['position'], lambda t: target_at(t)['quaternion'])
    limits = frame.limits
    deadline = limits['launch_time'] + limits['lifetime']
    count = math.ceil((limits['lifetime'] + 1.) / STEP)
    radius = (reach_m if reach_m is not None else float(session.props.get('radius', 0.) or 0.)) + 30.
    burn_end = _burn_end(frame.flight.properties['guidance']['motors'])
    body = session.frame.flight.state['body']
    best = (math.dist(frame.website_release['position'], target['position']), 0.)
    previous = (body['position'][:], target_at(body['time'])['position'], body['time'])
    outcome = 'time_limit'; end_time = body['time']; event_time = None
    for i in range(count):
        now = add(body['time'], STEP)
        result = session.advance(now, STEP, targets, collision_time=now, point_history=history, flight_step=radar_step)
        body = session.frame.flight.state['body']
        event = result.get('event')
        if event: point, at = event['position'], event['time']
        else: point, at = body['position'], body['time']
        target_point = target_at(at)['position']
        distance, fraction = w.closest_segment(previous[0], point, previous[1], target_point)
        if distance < best[0]: best = (distance, previous[2] + fraction * (at - previous[2]))
        previous = (point[:], target_point, at)
        end_time = at
        if reach_m is not None and best[0] <= reach_m:
            outcome = 'reached'; event_time = best[1]; break
        if event:
            outcome = 'proximity_event'; event_time = at; break
        if body['position'][1] <= 0: outcome = 'ground'; break
        if target_at(body['time'])['position'][1] <= 0: outcome = 'target_ground'; break
        if session.frame.expiry_flag:
            expiry = result['frame']['expiry']
            outcome = 'lifetime' if expiry['time_expired'] else 'distance_limit' if expiry['distance_expired'] else 'tracking_timeout'
            break
        if early_exit and i % 6 == 5 and body['time'] >= straight and all(c >= e + 1e-3 for c, e in zip(body['clocks'], burn_end)):
            speed = math.sqrt(sum(v * v for v in body['velocity']))
            if unreachable(body['position'], speed, target_at(body['time'])['position'], straight_velocity,
                           deadline - body['time'] + 2 * STEP, radius, limits['origin'], limits['distance_limit']):
                outcome = 'unreachable'; break
    return dict(hit=outcome in ('proximity_event', 'reached'), outcome=outcome, time_s=end_time if event_time is None else event_time,
                event_time_s=event_time,
                closest_m=best[0], closest_time_s=best[1], steps=i + 1, family=frame.flight.family,
                lifetime_s=limits['lifetime'], distance_limit_m=limits['distance_limit'])


def missile_limits(missile):
    """Lifetime and distance limit used to bound the search, from a real frame."""
    w = _missile_worker()
    frame = w.initial_frame(_profile(missile), dict(position=[0., 5000., 0.], velocity=[250., 0., 0.], angles=[0., 0., 0.]),
                            dict(position=[5000., 5000., 0.], velocity=[-250., 0., 0.], angles=[180., 0., 0.]),
                            launcher_aircraft=True)
    rocket = frame.flight.properties['rocket']
    fuze = float(rocket.get('proximityFuse', {}).get('radius', 0.) or 0.) if rocket.get('hasProximityFuse', False) else 0.
    return dict(lifetime_s=frame.limits['lifetime'], distance_limit_m=frame.limits['distance_limit'], family=frame.flight.family,
                fuze_radius_m=fuze, launch_angle_deg=launch_angles(rocket, frame.flight.family))


def launch_angles(rocket, family):
    """Launch-zone half-angles off the launcher's nose (yaw, pitch) in degrees.

    aces.exe 2.59.0.86: every guided launcher loads guidance.launchAngleMax
    (default 90/90 for optical launchers 0x143ec3d00, 0/0 for radar launchers
    0x1452b9570) and its seeker's lockAngleMax (default 5, 0x14332b300). The
    launch-zone query the HUD reads (launcher properties vtable +0x78: radar
    0x1452ba6d0, optical 0x143ec52d0) reports max(launchAngleMax, lockAngleMax)
    per axis for radar launchers and min(launchAngleMax, lockAngleMax) for
    optical ones. So a radar missile launches within its seeker's lock angle
    unless launchAngleMax widens it (R-77-1, RVV-SD, MICA-EM, PL-12: 180/90),
    and an optical one within the smaller of the two.
    """
    guidance = rocket.get('guidance', {})
    if family == 'radar':
        seeker = guidance.get('radarSeeker', {})
        default = [0., 0.]
    else:
        seeker = guidance.get('irSeeker', guidance.get('opticalSeeker', {}))
        default = [90., 90.]
    lock = float(seeker.get('lockAngleMax', 5.))
    launch = [float(v) for v in guidance.get('launchAngleMax', default)]
    if family == 'radar': return [max(a, lock) for a in launch]
    return [min(a, lock) for a in launch]


def launch_yaw_limit(config, limits):
    """Largest target bearing off the nose (deg) that can be launched at, or None without a limit.

    In the launcher frame both aircraft fly level at the same altitude, so the
    target's pitch off the nose is zero and only the yaw limit applies: the
    missile's launch zone and the launching aircraft's gimbal limit. In the
    target frame the launcher points at the target.
    """
    if config['frame'] != 'launcher': return None
    limit = limits['launch_angle_deg'][0]
    gimbal = config['launcher']['gimbal_deg']
    if gimbal is not None and gimbal < limit: limit = gimbal
    return limit


def _middle(a, b):
    """Bisection point: geometric across wide brackets, arithmetic otherwise."""
    return math.sqrt(a * b) if a > 0 and b > 1.5 * a else (a + b) / 2


class Ray:
    """Search one horizontal ray for its hit intervals; a generator of range batches."""

    def __init__(self, key, layer, azimuth, low, high, tolerance, tolerance_m, level=0, escape=False):
        self.key, self.layer, self.azimuth = key, layer, azimuth
        self.low, self.high = low, high
        self.tolerance, self.tolerance_m = tolerance, tolerance_m
        self.level = level
        self.escape = escape   # searched against the escaping target (no-escape zone)
        self.cell = None
        self.samples = {}
        self.generator = None
        self.result = None
        self.simulations = 0
        self.started = self.finished = None
        self.seed_source = None
        self.blocked = False   # outside the launch zone: no launch, nothing to simulate

    def width(self, r): return max(self.tolerance_m, self.tolerance * r)

    def coarse(self, ratio=COARSE_RATIO):
        grid = [self.low]
        r = max(self.low, 300.)
        while r < self.high:
            grid.append(r); r *= ratio
        grid.append(self.high)
        return grid

    def fresh(self, values):
        out = []
        for r in sorted(min(self.high, max(self.low, r)) for r in values):
            if all(abs(r - s) > self.width(r) / 4 for s in list(self.samples) + out): out.append(r)
        return out

    def search(self, seed):
        def run(values):
            values = self.fresh(values)
            if values:
                outcomes = yield values
                self.samples.update(zip(values, outcomes))
        if self.blocked: return self.finish()
        hits = lambda: sorted(r for r, o in self.samples.items() if o['hit'])
        out = inward = down = up = .1
        descend = ascend = bottom = None   # inner probes of the predicted maximum and minimum
        # Inside a region without hits (Grid seed 'empty') the range grid is sparser.
        empty = seed is not None and seed.get('source') == 'empty'
        if empty: seed = None
        if seed is not None:
            # Bracket both predicted boundaries; probe the middle when the prediction is loose.
            low = max(seed['low'], self.low); high = max(seed['high'], low); spread = seed['spread']
            spread_low = seed.get('spread_low', spread)
            # Expansion steps start at twice each boundary's probe spread.
            out = down = seed.get('step', max(2 * spread, .1))
            inward = up = seed.get('step_low', out)
            grow = math.exp(spread)
            probes = [high / grow, high * grow]
            descend = high / grow
            at_limit = seed['at_limit'] or low <= self.low * 1.05
            # Minimum ranges are predicted in log(r + 100 m); probe their spread there.
            if at_limit: probes.append(self.low)
            else:
                probes += [_unlog(_log(low) - spread_low), _unlog(_log(low) + spread_low)]; bottom, ascend = probes[-2:]
            if not at_limit and spread > .12: probes.append(_middle(low, high))
            yield from run(probes + seed.get('extra', []))
            if not hits() and high > low * 1.5:
                yield from run([_middle(low, high)] + [low * (high / low) ** f for f in (.25, .75)])
        # A ray next to an empty neighbour that finds no hit near its live neighbour's
        # interval is empty: the empty neighbour already searched its whole range.
        if not hits() and not (seed is not None and seed.get('edge')):
            yield from run(self.coarse(COARSE_RATIO ** 2 if empty else COARSE_RATIO))
        if not hits(): return self.finish()
        while True:
            # Expand from the hits until both boundaries are bracketed. When both
            # probes of a predicted boundary miss far from any hit, step from it
            # toward the hits, so a boundary that moved, or an interval separate
            # from the hits found so far, is bracketed before bisecting.
            h = hits(); misses = [r for r, o in self.samples.items() if not o['hit']]
            batch = []
            if not any(r > h[-1] for r in misses):
                if h[-1] < self.high - self.width(self.high) / 4:
                    batch.append(min(self.high, max(h[-1] * (1 + out), h[-1] + 4000. * out))); out *= 2
            if descend is not None:
                if h[-1] >= descend or descend <= h[-1] * (1 + 2 * down): descend = None
                else:
                    descend = max(self.low, descend / (1 + down)); batch.append(descend); down *= 2
            if not any(r < h[0] for r in misses):
                if h[0] > self.low + self.width(self.low) / 4:
                    # Short flights are cheap: test the closest searched distance at once.
                    batch += [max(self.low, h[0] / (1 + inward)), self.low]; inward *= 2
            if ascend is not None:
                # Hits closer than the predicted minimum (scattered short-range hits) do not end the step.
                above = [r for r in h if r >= bottom]
                if not above or above[0] <= ascend or ascend * (1 + 2 * up) >= above[0]: ascend = None
                else:
                    ascend = min(self.high, ascend * (1 + up)); batch.append(ascend); up *= 2
            batch = self.fresh(batch)
            if not batch: break
            yield from run(batch)
        # Near a boundary, narrow misses and hits can alternate; the bisection keeps
        # the transitions it brackets. The minimum and maximum ranges and the start
        # of the farthest interval are bisected to the tolerance (but not below a
        # quarter of the maximum range's), the edges of other gaps between hits to
        # INTERIOR times the tolerance of the maximum range.
        while True:
            rs = sorted(self.samples); h = hits()
            inner = max([r for r in rs if r < h[-1] and not self.samples[r]['hit']], default=None)
            floor = self.width(h[-1]) / 4
            batch = [_middle(a, b) for a, b in zip(rs, rs[1:])
                     if self.samples[a]['hit'] != self.samples[b]['hit']
                     and b - a > (max(self.width((a + b) / 2), floor) if b == h[0] or a == h[-1] or a == inner
                                  else INTERIOR * self.width(h[-1]))]
            if not batch: break
            yield from run(batch)
        return self.finish()

    def finish(self):
        rs = sorted(self.samples)
        intervals = []
        for i, r in enumerate(rs):
            if not self.samples[r]['hit']: continue
            start = (rs[i - 1] + r) / 2 if i else r
            if intervals and intervals[-1]['last'] == rs[i - 1]:
                intervals[-1].update(last=r); continue
            intervals.append(dict(start=start, first=r, last=r))
        for item in intervals:
            j = rs.index(item['last'])
            item['end'] = (item['last'] + rs[j + 1]) / 2 if j + 1 < len(rs) else item['last']
            item['open_low'] = item['first'] == rs[0]
            item['open_high'] = j + 1 == len(rs)
        samples = [dict(range_m=r, **{k: self.samples[r][k] for k in ('hit', 'outcome', 'time_s', 'closest_m')}) for r in rs]
        self.result = dict(key=list(self.key), motion='escape' if self.escape else 'straight',
                           layer_m=self.layer, azimuth_deg=self.azimuth, level=self.level, search=[self.low, self.high],
                           intervals=[dict(start_m=i['start'], end_m=i['end'], first_hit_m=i['first'], last_hit_m=i['last'],
                                           min_at_search_limit=i['open_low'], max_at_search_limit=i['open_high']) for i in intervals],
                           samples=samples, simulations=self.simulations, seed=self.seed_source)
        if intervals:
            first, last = intervals[0], intervals[-1]
            self.result.update(min_range_m=first['start'], max_range_m=last['end'],
                               min_bracket_m=[rs[rs.index(first['first']) - 1] if not first['open_low'] else None, first['first']],
                               max_bracket_m=[last['last'], rs[rs.index(last['last']) + 1] if not last['open_high'] else None],
                               max_time_s=self.samples[last['last']]['time_s'], min_time_s=self.samples[first['first']]['time_s'])
        else:
            self.result.update(min_range_m=None, max_range_m=None)
        if self.blocked: self.result['blocked'] = True
        return self.result


MAX_RAYS = 1200


def plan(config, limits, escape=False):
    """Base ray grid with horizontal search bounds for every layer and azimuth.

    The missile cannot fly farther than its distance limit from the launch
    point (or a generous Mach 9 over its lifetime when there is none), and the
    target moves at most speed x lifetime: no hit lies beyond their sum.
    Ray keys are (layer index, azimuth in units of the finest refinement step);
    `cell` is (layer index, base column) for the seeding Grid.
    """
    slant = limits['distance_limit_m'] if limits['distance_limit_m'] > 0 else 3000. * limits['lifetime_s']
    slant += config['target']['speed_mps'] * limits['lifetime_s'] + 2000.
    unit = finest_step(config)
    yaw_limit = launch_yaw_limit(config, limits)
    rays = []
    for li, layer in enumerate(config['layers_m']):
        dh = abs(vertical_offset(config, layer))
        # A small horizontal offset avoids a vertical launch straight at the target.
        low = max(HORIZONTAL_MINIMUM_M, math.sqrt(max(SLANT_MINIMUM_M ** 2 - dh * dh, 0.)))
        high = min(config['max_range_m'], math.sqrt(max(slant * slant - dh * dh, 0.)))
        for ai, azimuth in enumerate(azimuths(config)):
            ray = Ray((li, round(azimuth / unit)), layer, azimuth, low, max(high, low), config['tolerance'], config['tolerance_m'],
                      escape=escape)
            ray.cell = (li, ai)
            ray.blocked = outside(yaw_limit, azimuth)
            rays.append(ray)
    if len(rays) > MAX_RAYS:
        raise ValueError(f'Too many search directions ({len(rays)}); use fewer layers or a coarser azimuth step (limit {MAX_RAYS})')
    return rays


def outside(yaw_limit, azimuth):
    """Whether a ray's bearing lies outside the launch zone (yaw_limit None: no limit)."""
    return yaw_limit is not None and abs(_wrap(azimuth)) > yaw_limit + 1e-9


def finest_step(config):
    """Largest halving of the azimuth step that is at most the edge step."""
    unit = config['azimuth_step_deg']
    while unit > config['edge_step_deg'] * (1 + 1e-9): unit /= 2.
    return unit


LOG_OFFSET_M = 100.


def _log(r): return math.log(r + LOG_OFFSET_M)


def _unlog(x): return max(0., math.exp(x) - LOG_OFFSET_M)


class Grid:
    """Fixed seeding plan: which finished base rays predict each base ray's boundaries.

    The middle layer's rays nearest 0 and 180 degrees start from a coarse range
    grid. Along azimuth, each ray follows the neighbour one step closer to the
    nearest of those origins (and the one beyond it, for extrapolation). Other
    layers also follow the same azimuth one layer closer to the middle, scaled
    by that layer's ratio at the chain neighbour (parallelogram rule). A ray
    starts only when all of its parents have finished, so the samples searched,
    and therefore the envelope, do not depend on worker timing. Keys are cells
    (layer index, base column).
    """

    def __init__(self, rays, columns, circular, angles):
        self.rays, self.columns, self.circular = rays, columns, circular
        self.layers = 1 + max(key[0] for key in rays)
        self.middle = (self.layers - 1) // 2
        front = min(range(columns), key=lambda i: abs(angles[i]))
        back = max(range(columns), key=lambda i: abs(angles[i]))
        self.origins = [front] if back == front else [front, back]
        self.parents = {key: self._parents(key) for key in rays}

    def _step(self, a):
        """Chain neighbour one step closer to the nearest origin, or None at an origin."""
        if a in self.origins: return None
        best = None
        for origin in self.origins:
            for direction in (-1, 1):
                distance = ((origin - a) * direction) % self.columns if self.circular else (origin - a) * direction
                if distance <= 0: continue
                if best is None or distance < best[0]: best = (distance, direction)
        if best is None: return None
        return (a + best[1]) % self.columns if self.circular else a + best[1]

    def _parents(self, key):
        l, a = key
        chain = self._step(a); beyond = self._step(chain) if chain is not None else None
        toward = None if l == self.middle else (l + (1 if l < self.middle else -1))
        out = dict(chain=None if chain is None else (l, chain), beyond=None if beyond is None else (l, beyond),
                   layer=None if toward is None else (toward, a),
                   corner=None if toward is None or chain is None else (toward, chain))
        if toward is not None and chain is None:
            further = toward + (1 if toward < self.middle else -1) if toward != self.middle else None
            out['beyond_layer'] = None if further is None else (further, a)
        return out

    def ready(self, key):
        return all(self.rays[p].result is not None for p in self.parents[key].values() if p is not None)

    def depth(self, key, memo=None):
        """Longest parent chain, for a start order that keeps chains moving."""
        memo = {} if memo is None else memo
        if key not in memo:
            parents = [p for p in self.parents[key].values() if p is not None]
            memo[key] = 0 if not parents else 1 + max(self.depth(p, memo) for p in parents)
        return memo[key]

    def finished(self, key):
        ray = self.rays.get(key) if key is not None else None
        if ray is None or ray.result is None or ray.result['max_range_m'] is None: return None
        return lobe(ray.result, ray.tolerance)

    def seed(self, key):
        """Predicted boundaries from this cell's parents, or None for a coarse search."""
        p = {name: self.finished(cell) for name, cell in self.parents[key].items()}
        estimates = []   # (log low, log high, at_limit, base spread)
        if p['chain'] and p.get('beyond'):
            one, two = p['chain'], p['beyond']
            estimates.append((2 * one[0] - two[0], 2 * one[1] - two[1], one[2], .04))
        if p['layer'] and p['chain'] and p['corner']:
            across, same, corner = p['layer'], p['chain'], p['corner']
            estimates.append((across[0] + same[0] - corner[0], across[1] + same[1] - corner[1], across[2], .04))
        if p['layer'] and p.get('beyond_layer'):
            one, two = p['layer'], p['beyond_layer']
            estimates.append((2 * one[0] - two[0], 2 * one[1] - two[1], one[2], .06))
        if not estimates:
            for name, spread in (('chain', .1), ('layer', .15)):
                if p.get(name): estimates.append((*p[name], spread))
        parents = [list(self.rays[k].key) for k in self.parents[key].values() if k is not None]
        # Inside a region without hits the range grid is searched more sparsely.
        if not estimates: return dict(source='empty', parents=parents) if parents else None
        lows = sorted(e[0] for e in estimates); highs = sorted(e[1] for e in estimates)
        median = lambda xs: (xs[(len(xs) - 1) // 2] + xs[len(xs) // 2]) / 2
        base = min(e[3] for e in estimates)
        spread = max(base, (highs[-1] - highs[0]) / 2 + .02)
        spread_low = max(base + .04, (lows[-1] - lows[0]) / 2 + .04)
        limits = sum(e[2] for e in estimates)
        return dict(low=_unlog(median(lows)), high=_unlog(median(highs)), at_limit=limits * 2 > len(estimates),
                    spread=min(spread, .3), spread_low=min(spread_low, .5), source='neighbours', parents=parents)


class Planner:
    """Deterministic ray schedule: the base grid (seeded by Grid) and its refinements.

    Once two neighbouring rays of a layer have both finished, refine() decides
    from their results alone whether a ray halfway between them is needed, and
    between() seeds it from those two rays only. Rays are only ever inserted
    between finished neighbours, so the rays searched, and therefore the
    envelope, do not depend on worker timing or start order. With escape every
    ray is searched against the escaping target (an independent schedule).
    """

    def __init__(self, config, limits, escape=False):
        self.config = config
        self.escape = escape
        self.unit = finest_step(config)
        self.yaw_limit = launch_yaw_limit(config, limits)
        self.circular = not mirrored(config)
        self.period = round(360. / self.unit)
        angles = azimuths(config)
        base = plan(config, limits, escape)
        self.rays = {ray.key: ray for ray in base}
        self.cells = {ray.cell: ray for ray in base}
        self.grid = Grid(self.cells, len(angles), self.circular, angles)
        memo = {}
        self.priority = {ray.key: (0, self.grid.depth(ray.cell, memo), ray.key) for ray in base}
        self.children = {}
        for cell in self.cells:
            for parent in self.grid.parents[cell].values():
                if parent is not None: self.children.setdefault(parent, []).append(cell)
        self.lines = {}
        for ray in base: self.lines.setdefault(ray.key[0], []).append(ray.key[1])
        for line in self.lines.values(): line.sort()
        self.queue = [ray.key for ray in base if self.grid.ready(ray.cell)]
        self.queued = {self.rays[key].cell for key in self.queue}   # base cells released so far
        self.seeds = {}
        self.spans = {}   # refinement ray key -> (left key, right key) it was added between
        self.done = 0

    def total(self): return len(self.rays)

    def complete(self): return self.done == len(self.rays)

    def ready(self, count):
        """Up to count (ray, seed) pairs that may start now, in a fixed priority order."""
        if count <= 0 or not self.queue: return []
        self.queue.sort(key=lambda k: self.priority[k])
        out = []
        for key in self.queue[:count]:
            ray = self.rays[key]
            seed = self.seeds.pop(key) if ray.cell is None else self.grid.seed(ray.cell)
            out.append((ray, seed))
        self.queue = self.queue[count:]
        return out

    def finish(self, key, result):
        ray = self.rays[key]
        ray.result = result
        self.done += 1
        if ray.cell is not None:
            for cell in self.children.get(ray.cell, []):
                if cell not in self.queued and self.grid.ready(cell):
                    self.queued.add(cell); self.queue.append(self.cells[cell].key)
        # A ray added between two neighbours is 'smooth' when it confirms their
        # interpolation; then neither half needs another ray below the detail step.
        smooth = self.matched(ray) if key in self.spans else None
        layer, position = key
        line = self.lines[layer]
        i = line.index(position)
        pairs = []   # (left, right) in increasing azimuth; a circular wrap keeps that order.
        if self.circular or i > 0: pairs.append((self.rays[(layer, line[(i - 1) % len(line)])], ray))
        if self.circular or i + 1 < len(line): pairs.append((ray, self.rays[(layer, line[(i + 1) % len(line)])]))
        for left, right in pairs:
            if left.result is not None and right.result is not None: self._split(left, right, smooth)

    def matched(self, ray):
        """Whether a refinement ray's boundaries lie within MATCH tolerances of its parents' mean."""
        results = [self.rays[k].result for k in self.spans[ray.key]] + [ray.result]
        scale = self.scale(*results)
        x, y, z = [structure(r, SHAPE * self.config['tolerance'] * scale) for r in results]
        limit = MATCH * self.config['tolerance'] * scale
        return len(x) == len(y) == len(z) and all(abs(c[0] - (a[0] + b[0]) / 2) <= limit and abs(c[1] - (a[1] + b[1]) / 2) <= limit
                                                    for a, b, c in zip(x, y, z))

    def scale(self, *results):
        """Comparison length: the farthest boundary among the rays compared.

        Boundary changes are judged against the size the envelope is drawn at,
        so the scatter of very short minimum ranges does not draw refinement.
        """
        return max([self.config['tolerance_m']] + [r['max_range_m'] for r in results if r['max_range_m'] is not None])

    def _split(self, left, right, smooth):
        gap = right.key[1] - left.key[1]
        if self.circular: gap %= self.period
        if gap <= 1 or not self.refine(left.result, right.result, gap * self.unit, smooth): return
        position = left.key[1] + gap // 2
        if self.circular and position > self.period // 2: position -= self.period
        layer = left.key[0]
        key = (layer, position)
        ray = Ray(key, left.layer, position * self.unit, left.low, left.high, left.tolerance, left.tolerance_m,
                  level=max(left.level, right.level) + 1, escape=self.escape)
        ray.blocked = outside(self.yaw_limit, ray.azimuth)
        line = self.lines[layer]
        index = 0
        while index < len(line) and line[index] < position: index += 1
        line.insert(index, position)
        self.rays[key] = ray
        self.spans[key] = (left.key, right.key)
        self.seeds[key] = self.between(left, right)
        self.priority[key] = (1, ray.level, key)
        self.queue.append(key)

    def refine(self, a, b, gap, smooth):
        """Whether two finished neighbours gap degrees apart need a ray between them.

        Where only one of them hits, rays are added down to the edge step. Where
        the gaps between hits differ or the boundaries bend, they are added down
        to the detail step: between two base rays when a boundary moves by more
        than SHAPE tolerances, otherwise when the ray last added between them
        did not confirm its neighbours' interpolation (smooth is False; True
        when it did, None for two base rays).
        """
        c = self.config
        scale = self.scale(a, b)
        x, y = structure(a, SHAPE * c['tolerance'] * scale), structure(b, SHAPE * c['tolerance'] * scale)
        if not (x or y): return False
        if not (x and y): return True
        if gap <= c['detail_step_deg'] * (1 + 1e-9): return False
        if len(x) != len(y): return True
        if smooth is not None: return not smooth
        limit = SHAPE * c['tolerance'] * scale
        return any(abs(p[0] - q[0]) > limit or abs(p[1] - q[1]) > limit for p, q in zip(x, y))

    def between(self, left, right):
        """Seed for the ray halfway between two finished neighbours."""
        tolerance = self.config['tolerance']
        merge = SHAPE * tolerance * self.scale(left.result, right.result)
        x, y = [[[_log(r) for r in item] for item in structure(ray.result, merge)] for ray in (left, right)]
        live = [s for s in (x, y) if s]
        limited = [s[0][0] <= _log(left.low) * (1 + 1e-12) for s in live]
        parents = [list(left.key), list(right.key)]
        if len(live) == 2 and len(x) == len(y):
            # Interpolate; the probes bracket the prediction by a quarter of the neighbours' difference.
            spread = min(.3, max(.45 * tolerance, abs(x[-1][1] - y[-1][1]) / 4))
            spread_low = min(.5, max(.45 * tolerance, abs(x[0][0] - y[0][0]) / 4))
            gaps = [_unlog((p[1] + q[1] + p2[0] + q2[0]) / 4) for p, p2, q, q2 in zip(x, x[1:], y, y[1:])]
            return dict(low=_unlog((x[0][0] + y[0][0]) / 2), high=_unlog((x[-1][1] + y[-1][1]) / 2), at_limit=all(limited),
                        spread=spread, spread_low=spread_low, step=max(2 * spread, 2 * tolerance),
                        step_low=max(2 * spread_low, 2 * tolerance), extra=gaps, source='between', parents=parents)
        # An edge (one side has no hit) or a change in the gaps: start from the live
        # neighbours' outer bounds and probe inside each of their gaps.
        gaps = [_unlog((p[1] + q[0]) / 2) for s in live for p, q in zip(s, s[1:])]
        return dict(low=_unlog(min(s[0][0] for s in live)), high=_unlog(max(s[-1][1] for s in live)), at_limit=any(limited),
                    spread=.06, spread_low=.1, edge=len(live) == 1, extra=gaps, source='edge' if len(live) == 1 else 'gaps',
                    parents=parents)


def structure(result, merge):
    """Hit intervals as [start, end] in metres, as refinement compares them.

    Gaps narrower than merge metres are closed and intervals narrower than it
    dropped, keeping the farthest one, so scattered hits at short range
    or narrow misses near an edge do not count as changes of the envelope.
    """
    out = []
    for item in result['intervals']:
        if out and item['start_m'] - out[-1][1] < merge: out[-1][1] = item['end_m']
        else: out.append([item['start_m'], item['end_m']])
    return [s for s in out if s[1] - s[0] >= merge] or out[-1:]


def lobe(result, tolerance):
    """(log start, log end, start at the search limit) of a ray's hits, as structure() compares them."""
    s = structure(result, SHAPE * tolerance * result['max_range_m'])
    return (_log(s[0][0]), _log(s[-1][1]), s[0][0] <= result['search'][0] * (1 + 1e-9))


def _safe_evaluate(missile, launcher, target, early_exit):
    try:
        return evaluate(missile, launcher, target, early_exit)
    except Exception as error:  # A failed sample is reported, never counted as a hit.
        return dict(hit=False, outcome='error', error=f'{type(error).__name__}: {error}', time_s=None, closest_m=None)


def planners(config, limits):
    """The envelope's schedules: the straight-flying target, and the escaping one when requested."""
    return [Planner(config, limits)] + ([Planner(config, limits, True)] if config['escape'] is not None else [])


def search(config, *, workers=None, emit=lambda kind, data: None, cancelled=lambda: False, executor=None):
    """Run the full envelope search; emit('ray', result) as each ray completes.

    Rays start as soon as a Planner releases them; both schedules share the
    workers, and the result is the same for any number of workers.
    """
    started = time.perf_counter()
    limits = missile_limits(config['missile'])
    schedules = planners(config, limits)
    total = lambda: sum(p.total() for p in schedules)
    emit('plan', dict(config=config, limits=limits, rays_total=total()))
    workers = workers or default_workers()
    own = executor is None
    pool = executor or ProcessPoolExecutor(max_workers=workers, initializer=_missile_worker)
    capacity = workers + 2
    pending = {}   # future -> (planner, ray, index)
    batches = {}   # (escape, ray key) -> outcomes being collected
    counters = dict(simulations=0, errors=0, early=0)
    active = set()

    def submit(ray, planner, values):
        batches[ray.escape, ray.key] = [None] * len(values)
        for i, r in enumerate(values):
            launcher, target = states(config, ray.layer, ray.azimuth, r, ray.escape)
            pending[pool.submit(_safe_evaluate, config['missile'], launcher, target, config['early_exit'])] = (planner, ray, i)
        ray.simulations += len(values); counters['simulations'] += len(values)

    def advance(ray, planner, outcomes):
        try:
            values = next(ray.generator) if outcomes is None else ray.generator.send(outcomes)
        except StopIteration:
            ray.finished = time.perf_counter()
            result = dict(ray.result, wall_s=ray.finished - ray.started,
                          errors=sum(s['outcome'] == 'error' for s in ray.result['samples']))
            active.discard((ray.escape, ray.key))
            planner.finish(ray.key, result)
            emit('ray', result)
            emit('progress', dict(rays_done=sum(p.done for p in schedules), rays_total=total(),
                                  simulations=counters['simulations'], elapsed_s=time.perf_counter() - started))
            return
        submit(ray, planner, values)

    try:
        while not all(p.complete() for p in schedules):
            if cancelled(): raise InterruptedError('Envelope calculation cancelled')
            for planner in schedules:
                for ray, seed in planner.ready(capacity - len(active)):
                    active.add((ray.escape, ray.key))
                    ray.seed_source = seed and dict(seed)
                    ray.generator = ray.search(seed)
                    ray.started = time.perf_counter()
                    advance(ray, planner, None)
            if not pending:
                if not all(p.complete() for p in schedules): raise RuntimeError('Envelope search plan has no ready direction')
                continue
            done, _ = wait(list(pending), timeout=.5, return_when=FIRST_COMPLETED)
            for future in done:
                planner, ray, index = pending.pop(future)
                outcome = future.result()
                counters['errors'] += outcome['outcome'] == 'error'
                counters['early'] += outcome['outcome'] == 'unreachable'
                batch = batches[ray.escape, ray.key]
                batch[index] = outcome
                if all(o is not None for o in batch):
                    advance(ray, planner, batches.pop((ray.escape, ray.key)))
    finally:
        if own: pool.shutdown(wait=False, cancel_futures=True)
    return dict(format='missile-envelope-v2', config=config, limits=limits, mirrored=mirrored(config),
                azimuths_deg=azimuths(config), rays=[p.rays[key].result for p in schedules for key in sorted(p.rays)],
                simulations=counters['simulations'], sample_errors=counters['errors'], early_misses=counters['early'],
                elapsed_s=time.perf_counter() - started, workers=workers)


def default_workers():
    configured = os.environ.get('WT_ENVELOPE_WORKERS')
    if configured: return max(1, min(61, int(configured)))
    return max(1, min(16, (os.cpu_count() or 2) - 1))
