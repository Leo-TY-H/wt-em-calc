"""Launch-envelope search over the website's recovered missile engagement model.

Every sample is the website engagement (the same session, target, support and
terminal rules as missile_worker.simulate) against a constant-velocity point
target. Two hit tests are available:

criterion='kinematic'  (default) The kinematic envelope: the proximity fuze is
                       switched off and a launch counts when the guided missile
                       passes within reach_m of the target before its lifetime,
                       distance limit or the ground ends the flight. Fuze arming
                       and timing do not shape it; motor, mass, drag, lift,
                       control limits and guidance do. reach_m defaults to the
                       missile's proximity-fuze radius.
criterion='fuze'       A native proximity event, exactly as the simulator tab.

The envelope is sampled in cylindrical coordinates around a reference aircraft:
one horizontal ray per (layer altitude, azimuth). Along each ray the horizontal
launch distance is searched for its hit/miss transitions; the nearest and
farthest hit boundaries are the minimum and maximum launch ranges.

frame='target'    Target at the origin, flying level along +X. Layers are
                  launcher altitudes. Azimuth is the launcher's bearing from the
                  target's nose (0 = head-on, 180 = tail chase). The launcher
                  flies toward the target, nose on (aim='nose') or level.
frame='launcher'  Launcher at the origin, flying along +X. Layers are target
                  altitudes. Azimuth is the target's bearing off the launcher's
                  nose. The target flies level: toward the launcher ('hot'),
                  away ('cold'), along +X ('same'), along -X ('opposite') or a
                  fixed world heading in degrees.

Only presentation work is removed from the flight loop. A miss may be declared
early once every motor has burned out and an energy bound proves that the missile
cannot reach the target's future path before its lifetime or distance limit;
`early_exit=False` runs every flight to its modeled terminal condition.
"""
from concurrent.futures import FIRST_COMPLETED, ProcessPoolExecutor, wait
import json
import math
import os
import time

STEP_GRAVITY = 9.81
SLANT_MINIMUM_M = 150.
HORIZONTAL_MINIMUM_M = 50.
RANGE_CAP_M = 195000.
COARSE_RATIO = 1.6
SEED_SPREAD = .06
EXPLORE = (.05, .15)
DEFAULT_REACH_M = 5.
HEADINGS = ('hot', 'cold', 'same', 'opposite')
QUALITY = dict(quick=dict(azimuth_step_deg=30., tolerance=.04, tolerance_m=100.),
               standard=dict(azimuth_step_deg=15., tolerance=.02, tolerance_m=50.),
               fine=dict(azimuth_step_deg=7.5, tolerance=.01, tolerance_m=25.))

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
    if frame not in ('target', 'launcher'): raise ValueError("Frame must be 'target' or 'launcher'")
    launcher = request.get('launcher') or {}; target = request.get('target') or {}
    if not isinstance(launcher, dict) or not isinstance(target, dict): raise ValueError('Launcher and target settings are required')
    clean = dict(missile=missile, frame=frame,
                 launcher=dict(speed_mps=number(launcher.get('speed_mps'), 'Launcher speed', 0., 2000.)),
                 target=dict(speed_mps=number(target.get('speed_mps'), 'Target speed', 0., 2000.)))
    if frame == 'target':
        clean['target']['altitude_m'] = number(target.get('altitude_m'), 'Target altitude', 50., 25000.)
        aim = launcher.get('aim', 'nose')
        if aim not in ('nose', 'level'): raise ValueError("Launcher aim must be 'nose' or 'level'")
        clean['launcher']['aim'] = aim
    else:
        clean['launcher']['altitude_m'] = number(launcher.get('altitude_m'), 'Launcher altitude', 50., 25000.)
        clean['launcher']['pitch_deg'] = number(launcher.get('pitch_deg', 0.), 'Launch angle', -60., 60.)
        heading = target.get('heading', 'hot')
        if isinstance(heading, str):
            if heading not in HEADINGS: raise ValueError('Unknown target heading rule')
        else:
            heading = number(heading, 'Target heading', -360., 360.)
        clean['target']['heading'] = heading
    layers = request.get('layers_m')
    if not isinstance(layers, list) or not 1 <= len(layers) <= 12: raise ValueError('Supply between 1 and 12 layer altitudes')
    clean['layers_m'] = sorted({number(v, 'Layer altitude', 50., 25000.) for v in layers})
    quality = QUALITY.get(request.get('quality', 'standard'))
    if quality is None: raise ValueError('Unknown envelope quality')
    step = number(request.get('azimuth_step_deg', quality['azimuth_step_deg']), 'Azimuth step', 2.5, 45.)
    if abs(180. / step - round(180. / step)) > 1e-9: raise ValueError('Azimuth step must divide 180 degrees')
    clean['azimuth_step_deg'] = step
    clean['tolerance'] = number(request.get('tolerance', quality['tolerance']), 'Relative tolerance', .002, .2)
    clean['tolerance_m'] = number(request.get('tolerance_m', quality['tolerance_m']), 'Absolute tolerance', 5., 1000.)
    clean['max_range_m'] = number(request.get('max_range_m', RANGE_CAP_M), 'Maximum range', 1000., RANGE_CAP_M)
    clean['early_exit'] = bool(request.get('early_exit', True))
    criterion = request.get('criterion', 'kinematic')
    if criterion not in ('kinematic', 'fuze'): raise ValueError("Hit test must be 'kinematic' or 'fuze'")
    clean['criterion'] = criterion
    reach = request.get('reach_m')
    clean['reach_m'] = None if criterion == 'fuze' or reach is None else number(reach, 'Reach distance', 1., 2000.)
    return clean


def mirrored(config):
    """Left/right symmetric setups are searched on one side and mirrored."""
    if config['frame'] == 'target': return True
    heading = config['target']['heading']
    return isinstance(heading, str) or heading % 180. == 0.


def azimuths(config):
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


def states(config, layer, azimuth, distance):
    """Launcher and target states at release for one envelope sample (SI, degrees).

    The launcher is always released above the world origin, as in the missile
    simulator tab. The model computes in binary32 world coordinates, so near a
    boundary the outcome can depend on where the same geometry is placed;
    a shared placement lets the simulator tab replay any envelope sample exactly.
    """
    a = math.radians(azimuth)
    offset = [distance * math.cos(a), 0., distance * math.sin(a)]
    if config['frame'] == 'target':
        altitude = config['target']['altitude_m']
        target = dict(position=[-offset[0], altitude, -offset[2]], velocity=_direction(0., 0., config['target']['speed_mps']),
                      angles=[0., 0., 0.])
        heading = _wrap(azimuth + 180.)
        pitch = math.degrees(math.atan2(altitude - layer, distance)) if config['launcher']['aim'] == 'nose' else 0.
        launcher = dict(position=[0., layer, 0.], velocity=_direction(heading, pitch, config['launcher']['speed_mps']),
                        angles=[heading, pitch, 0.])
    else:
        pitch = config['launcher']['pitch_deg']
        launcher = dict(position=[0., config['launcher']['altitude_m'], 0.],
                        velocity=_direction(0., pitch, config['launcher']['speed_mps']), angles=[0., pitch, 0.])
        rule = config['target']['heading']
        heading = _wrap(dict(hot=azimuth + 180., cold=azimuth, same=0., opposite=180.)[rule] if isinstance(rule, str) else rule)
        target = dict(position=[offset[0], layer, offset[2]], velocity=_direction(heading, 0., config['target']['speed_mps']),
                      angles=[heading, 0., 0.])
    return launcher, target


def vertical_offset(config, layer):
    """Altitude of the other aircraft above the reference aircraft."""
    reference = config['target']['altitude_m'] if config['frame'] == 'target' else config['launcher']['altitude_m']
    return layer - reference


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

    With reach_m, the proximity fuze is disabled (it only consumes collision
    candidates; flight dynamics are unchanged) and the flight ends 'reached'
    once the missile passes within reach_m of the target.
    """
    w = _missile_worker()
    from frame_session import STEP
    from kernels import add
    profile = _profile(missile)
    frame = w.initial_frame(profile, launcher, target, launcher_aircraft=True)
    session = w.InteractionSession.from_frame(frame, clock=0., origin=frame.website_release['position'],
                                              enabled=reach_m is None, guidance_flag=False)
    motion = dict(kind='constant_velocity', position=target['position'], velocity=target['velocity'],
                  quaternion=w.quaternion(target['angles']))
    def target_at(t): return w.linear_point(motion, t)
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
        if early_exit and i % 6 == 5 and all(c >= e + 1e-3 for c, e in zip(body['clocks'], burn_end)):
            speed = math.sqrt(sum(v * v for v in body['velocity']))
            if unreachable(body['position'], speed, target_at(body['time'])['position'], target['velocity'],
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
                fuze_radius_m=fuze)


def _middle(a, b):
    """Bisection point: geometric across wide brackets, arithmetic otherwise."""
    return math.sqrt(a * b) if a > 0 and b > 1.5 * a else (a + b) / 2


class Ray:
    """Search one horizontal ray for its hit intervals; a generator of range batches."""

    def __init__(self, key, layer, azimuth, low, high, tolerance, tolerance_m):
        self.key, self.layer, self.azimuth = key, layer, azimuth
        self.low, self.high = low, high
        self.tolerance, self.tolerance_m = tolerance, tolerance_m
        self.samples = {}
        self.generator = None
        self.result = None
        self.simulations = 0
        self.started = self.finished = None
        self.seed_source = None

    def width(self, r): return max(self.tolerance_m, self.tolerance * r)

    def coarse(self):
        grid = [self.low]
        r = max(self.low, 300.)
        while r < self.high:
            grid.append(r); r *= COARSE_RATIO
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
        hits = lambda: sorted(r for r, o in self.samples.items() if o['hit'])
        spread = SEED_SPREAD
        if seed is not None:
            # Bracket both predicted boundaries; probe the middle when the prediction is loose.
            low = max(seed['low'], self.low); high = max(seed['high'], low); spread = seed['spread']
            grow = math.exp(spread)
            probes = [high / grow, high * grow]
            at_limit = seed['at_limit'] or low <= self.low * 1.05
            # Minimum ranges are predicted in log(r + 100 m); probe the same spread there.
            spread_low = seed.get('spread_low', spread)
            probes += [self.low] if at_limit else [_unlog(_log(low) - spread_low), _unlog(_log(low) + spread_low)]
            if not at_limit and spread > .12: probes.append(_middle(low, high))
            yield from run(probes)
            if not hits() and high > low * 1.5:
                yield from run([_middle(low, high)] + [low * (high / low) ** f for f in (.25, .75)])
        if not hits(): yield from run(self.coarse())
        if not hits(): return self.finish()
        out = inward = max(2 * spread, .1)
        explored = set()
        while True:
            while True:
                h = hits(); misses = [r for r, o in self.samples.items() if not o['hit']]
                batch = []
                if h[-1] < self.high - self.width(self.high) / 4 and not any(r > h[-1] for r in misses):
                    batch.append(min(self.high, max(h[-1] * (1 + out), h[-1] + 4000. * out))); out *= 2
                if h[0] > self.low + self.width(self.low) / 4 and not any(r < h[0] for r in misses):
                    # Short flights are cheap: test the closest searched distance at once.
                    batch += [max(self.low, h[0] / (1 + inward)), self.low]; inward *= 2
                batch = self.fresh(batch)
                if not batch: break
                yield from run(batch)
            while True:
                rs = sorted(self.samples)
                batch = [_middle(a, b) for a, b in zip(rs, rs[1:])
                         if self.samples[a]['hit'] != self.samples[b]['hit'] and b - a > self.width((a + b) / 2)]
                if not batch: break
                yield from run(batch)
            # Near a boundary, narrow misses and hits can alternate. Look a little
            # beyond each outer miss once; resume the search if hits continue there.
            # One probe follows the long maximum-range flights, two the short
            # minimum-range ones. Decisions use only hit/miss, so early miss
            # detection cannot change which launches are searched.
            h = hits(); misses = sorted(r for r, o in self.samples.items() if not o['hit'])
            above = [r for r in misses if r > h[-1]]; below = [r for r in misses if r < h[0]]
            probes = []
            if above and above[0] not in explored:
                explored.add(above[0]); probes.append(min(self.high, above[0] * (1 + EXPLORE[0])))
            if below and below[-1] not in explored:
                explored.add(below[-1]); probes += [max(self.low, _unlog(_log(below[-1]) - f)) for f in EXPLORE]
            probes = self.fresh(probes)
            if not probes: break
            before = len(hits())
            yield from run(probes)
            if len(hits()) == before: break
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
        self.result = dict(key=self.key, layer_m=self.layer, azimuth_deg=self.azimuth, search=[self.low, self.high],
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
        return self.result


MAX_RAYS = 640


def plan(config, limits):
    """Ray list with horizontal search bounds for every layer and azimuth.

    The missile cannot fly farther than its distance limit from the launch
    point (or a generous Mach 9 over its lifetime when there is none), and the
    target moves at most speed x lifetime: no hit lies beyond their sum.
    """
    slant = limits['distance_limit_m'] if limits['distance_limit_m'] > 0 else 3000. * limits['lifetime_s']
    slant += config['target']['speed_mps'] * limits['lifetime_s'] + 2000.
    rays = []
    for li, layer in enumerate(config['layers_m']):
        dh = abs(vertical_offset(config, layer))
        # A small horizontal offset avoids a vertical launch straight at the target.
        low = max(HORIZONTAL_MINIMUM_M, math.sqrt(max(SLANT_MINIMUM_M ** 2 - dh * dh, 0.)))
        high = min(config['max_range_m'], math.sqrt(max(slant * slant - dh * dh, 0.)))
        for ai, azimuth in enumerate(azimuths(config)):
            rays.append(Ray((li, ai), layer, azimuth, low, max(high, low), config['tolerance'], config['tolerance_m']))
    if len(rays) > MAX_RAYS:
        raise ValueError(f'Too many search directions ({len(rays)}); use fewer layers or a coarser azimuth step (limit {MAX_RAYS})')
    return rays


def _grid_distance(a, b, columns, circular):
    da = abs(a[1] - b[1])
    if circular: da = min(da, columns - da)
    return abs(a[0] - b[0]) * 1.5 + da


LOG_OFFSET_M = 100.


def _log(r): return math.log(r + LOG_OFFSET_M)


def _unlog(x): return max(0., math.exp(x) - LOG_OFFSET_M)


class Grid:
    """Fixed seeding plan: which finished rays predict each ray's boundaries.

    The middle layer's rays nearest 0 and 180 degrees start from a coarse range
    grid. Along azimuth, each ray follows the neighbour one step closer to the
    nearest of those origins (and the one beyond it, for extrapolation). Other
    layers also follow the same azimuth one layer closer to the middle, scaled
    by that layer's ratio at the chain neighbour (parallelogram rule). A ray
    starts only when all of its parents have finished, so the samples searched,
    and therefore the envelope, do not depend on worker timing.
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
        r = ray.result
        return (_log(r['min_range_m']), _log(r['max_range_m']), r['intervals'][0]['min_at_search_limit'])

    def seed(self, ray):
        """Predicted boundaries from this ray's parents, or None for a coarse search."""
        p = {name: self.finished(key) for name, key in self.parents[ray.key].items()}
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
        if not estimates: return None
        lows = sorted(e[0] for e in estimates); highs = sorted(e[1] for e in estimates)
        median = lambda xs: (xs[(len(xs) - 1) // 2] + xs[len(xs) // 2]) / 2
        base = min(e[3] for e in estimates)
        spread = max(base, (highs[-1] - highs[0]) / 2 + .02)
        spread_low = max(base + .04, (lows[-1] - lows[0]) / 2 + .04)
        limits = sum(e[2] for e in estimates)
        return dict(low=_unlog(median(lows)), high=_unlog(median(highs)), at_limit=limits * 2 > len(estimates),
                    spread=min(spread, .3), spread_low=min(spread_low, .5), source='neighbours',
                    parents=[list(k) for k in self.parents[ray.key].values() if k is not None])


def _safe_evaluate(missile, launcher, target, early_exit, reach_m=None):
    try:
        return evaluate(missile, launcher, target, early_exit, reach_m)
    except Exception as error:  # A failed sample is reported, never counted as a hit.
        return dict(hit=False, outcome='error', error=f'{type(error).__name__}: {error}', time_s=None, closest_m=None)


def search(config, *, workers=None, emit=lambda kind, data: None, cancelled=lambda: False, executor=None):
    """Run the full envelope search; emit('ray', result) as each ray completes.

    Rays start as soon as their parents (see Grid) have finished, deepest
    chains first; the result is the same for any number of workers.
    """
    started = time.perf_counter()
    limits = missile_limits(config['missile'])
    if config['criterion'] == 'kinematic' and config['reach_m'] is None:
        # Default reach: the missile's own lethal (proximity-fuze) radius.
        config = dict(config, reach_m=limits['fuze_radius_m'] or DEFAULT_REACH_M)
    reach = config['reach_m'] if config['criterion'] == 'kinematic' else None
    rays = {ray.key: ray for ray in plan(config, limits)}
    angles = azimuths(config)
    emit('plan', dict(config=config, limits=limits, rays_total=len(rays)))
    grid = Grid(rays, len(angles), not mirrored(config), angles)
    memo = {}
    depth = {key: grid.depth(key, memo) for key in rays}
    workers = workers or default_workers()
    own = executor is None
    pool = executor or ProcessPoolExecutor(max_workers=workers, initializer=_missile_worker)
    capacity = workers + 2
    pending = {}   # future -> (ray, index)
    batches = {}   # ray key -> outcomes being collected
    counters = dict(simulations=0, errors=0, early=0)
    waiting = set(rays)

    def submit(ray, values):
        batches[ray.key] = [None] * len(values)
        for i, r in enumerate(values):
            launcher, target = states(config, ray.layer, ray.azimuth, r)
            pending[pool.submit(_safe_evaluate, config['missile'], launcher, target, config['early_exit'], reach)] = (ray, i)
        ray.simulations += len(values); counters['simulations'] += len(values)

    def advance(ray, outcomes):
        try:
            values = next(ray.generator) if outcomes is None else ray.generator.send(outcomes)
        except StopIteration:
            ray.finished = time.perf_counter()
            ray.result = dict(ray.result, wall_s=ray.finished - ray.started,
                              errors=sum(s['outcome'] == 'error' for s in ray.result['samples']))
            emit('ray', ray.result)
            emit('progress', dict(rays_done=sum(r.result is not None for r in rays.values()), rays_total=len(rays),
                                  simulations=counters['simulations'], elapsed_s=time.perf_counter() - started))
            return
        submit(ray, values)

    def start(key):
        waiting.discard(key)
        ray = rays[key]
        seed = grid.seed(ray)
        ray.seed_source = seed and dict(seed)
        ray.generator = ray.search(seed)
        ray.started = time.perf_counter()
        advance(ray, None)

    try:
        while pending or waiting:
            if cancelled(): raise InterruptedError('Envelope calculation cancelled')
            # Rays nearest the origins start first; seeds never depend on this order.
            ready = sorted((key for key in waiting if grid.ready(key)), key=lambda k: (depth[k], k))
            for key in ready:
                if len(pending) >= capacity and pending: break
                start(key)
            if not pending:
                if waiting and not any(grid.ready(key) for key in waiting): raise RuntimeError('Envelope search plan has no ready direction')
                continue
            done, _ = wait(list(pending), timeout=.5, return_when=FIRST_COMPLETED)
            for future in done:
                ray, index = pending.pop(future)
                outcome = future.result()
                counters['errors'] += outcome['outcome'] == 'error'
                counters['early'] += outcome['outcome'] == 'unreachable'
                batches[ray.key][index] = outcome
                if all(o is not None for o in batches[ray.key]):
                    advance(ray, batches.pop(ray.key))
    finally:
        if own: pool.shutdown(wait=False, cancel_futures=True)
    return dict(format='missile-envelope-v1', config=config, limits=limits, mirrored=mirrored(config),
                azimuths_deg=angles, rays=[rays[key].result for key in sorted(rays)],
                simulations=counters['simulations'], sample_errors=counters['errors'], early_misses=counters['early'],
                elapsed_s=time.perf_counter() - started, workers=workers)


def default_workers():
    configured = os.environ.get('WT_ENVELOPE_WORKERS')
    if configured: return max(1, min(61, int(configured)))
    return max(1, min(16, (os.cpu_count() or 2) - 1))
