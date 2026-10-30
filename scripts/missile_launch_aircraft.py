"""Aircraft compatibility and launch-time sensor geometry for the web services.

The flight model still receives ideal tracking. These checks do not change
in-flight support, seeker behavior, missile dynamics or aircraft performance.
"""
from copy import deepcopy
from functools import lru_cache
import json
import math
from pathlib import Path
from missile_inputs import directory as input_directory
from missile_envelope import launch_angles

CATALOG = Path(__file__).resolve().parents[1] / 'references/missile-launchers.json'


@lru_cache(maxsize=1)
def catalog():
    return json.loads(CATALOG.read_bytes())


def compatible(missile):
    data = catalog()
    return [ident for ident in data['carriers'].get(missile, [])
            if data['families'][missile] != 'radar' or data['aircraft'][ident]['radars']]


@lru_cache(maxsize=256)
def infrared_launch_angles(missile):
    """The same authored lock/launch half-angles used by envelope searches.

    Optical seeker yaw and elevation are independently clamped to lockAngleMax
    before launch; angleMax is the wider in-flight tracking limit. A narrower
    authored launchAngleMax further restricts each axis.
    """
    profile = json.loads((input_directory()/ 'launch-profiles' / (missile + '.json')).read_bytes())
    return tuple(launch_angles(profile['properties']['rocket'], profile['family']))


def _selection(request):
    if not isinstance(request, dict): raise ValueError('Expected a scenario object')
    launcher = request.get('launcher')
    if not isinstance(launcher, dict): raise ValueError('Launcher settings are required')
    missile = request.get('missile')
    if not isinstance(missile, str) or missile not in catalog()['carriers']:
        raise ValueError('Select a supported missile')
    # Infrared launches use the missile's own seeker/launch limits. A saved
    # aircraft selection is irrelevant and must not impose a radar restriction.
    if catalog()['families'][missile] == 'optical': return None
    ident = launcher.get('aircraft_id')
    if not isinstance(ident, str) or ident not in compatible(missile):
        raise ValueError('Select a compatible launcher aircraft for this missile')
    aircraft = catalog()['aircraft'][ident]
    update = catalog().get('compatibility_updates', {}).get(missile, {}).get(ident)
    return dict(id=ident, name=aircraft['name'], radars=deepcopy(aircraft['radars']),
                radar_limited=catalog()['families'][missile] == 'radar',
                source_commit=update['source_commit'] if update else catalog()['source_commit'],
                **(dict(compatibility_source=deepcopy(update)) if update else {}),
                policy='launch_time_only')


def _vector(value, label):
    if (not isinstance(value, (list, tuple)) or len(value) != 3 or
        any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in value)):
        raise ValueError(f'{label} must contain three finite numbers')
    return value


def bearing(launcher, target):
    """LOS in aircraft axes: world +X forward, +Y up, +Z right.

    Inverse of missile_worker.quaternion: yaw(-Y), pitch(+Z), roll(+X).
    Radar elevation is measured above the body horizontal plane.
    """
    position = _vector(launcher.get('position'), 'Launcher position')
    other = _vector(target.get('position'), 'Target position')
    h, p, r = map(math.radians, _vector(launcher.get('angles', [0, 0, 0]), 'Launcher angles'))
    x, y, z = (b - a for a, b in zip(position, other))
    x, z = math.cos(h)*x + math.sin(h)*z, -math.sin(h)*x + math.cos(h)*z
    x, y = math.cos(p)*x + math.sin(p)*y, -math.sin(p)*x + math.cos(p)*y
    y, z = math.cos(r)*y + math.sin(r)*z, -math.sin(r)*y + math.cos(r)*z
    return math.degrees(math.atan2(z, x)), math.degrees(math.atan2(y, math.hypot(x, z)))


def within_radar(aircraft, azimuth, elevation):
    eps = 1e-9  # Inclusive authored boundary, allowing only roundoff in transforms.
    return any(r['azimuth_deg'][0]-eps <= azimuth <= r['azimuth_deg'][1]+eps and
               r['elevation_deg'][0]-eps <= elevation <= r['elevation_deg'][1]+eps
               for r in aircraft['radars'])


def attach(config, aircraft):
    """Restore launch provenance after native validators strip presentation data."""
    if aircraft is not None:
        config['launcher'].update(aircraft_id=aircraft['id'], aircraft=deepcopy(aircraft))
    else:
        config['launcher'].pop('aircraft_id', None)
        config['launcher'].pop('aircraft', None)
    return config


def trajectory(request):
    aircraft = _selection(request)
    target = request.get('target')
    if not isinstance(target, dict): raise ValueError('Target settings are required')
    azimuth, elevation = bearing(request['launcher'], target)
    if aircraft is None:
        yaw_limit, pitch_limit = infrared_launch_angles(request['missile'])
        if abs(azimuth) > yaw_limit + 1e-9 or abs(elevation) > pitch_limit + 1e-9:
            raise ValueError(f"Target is outside this missile's IR lock/launch limits "
                             f"(azimuth {azimuth:.2f}°, elevation {elevation:.2f}°; "
                             f"limits ±{yaw_limit:g}° / ±{pitch_limit:g}°)")
    if aircraft is not None and aircraft['radar_limited'] and not within_radar(aircraft, azimuth, elevation):
        raise ValueError(f"Target is outside {aircraft['name']}'s radar launch limits "
                         f"(azimuth {azimuth:.2f}°, elevation {elevation:.2f}°)")
    return attach(deepcopy(request), aircraft)


def envelope(request):
    if not isinstance(request, dict): raise ValueError('Expected an envelope request object')
    launcher = request.get('launcher')
    if not isinstance(launcher, dict): raise ValueError('Launcher settings are required')
    clean = deepcopy(request)
    # The existing surface launcher mode has no aircraft radar.
    if launcher.get('platform', 'air') == 'ground':
        clean['launcher'].pop('aircraft', None)
        clean['launcher'].pop('aircraft_id', None)
        return clean, None
    aircraft = _selection(request)
    # All air envelope shots are level at equal altitude. In target POV each
    # aircraft points at the target; in launcher POV its heading stays fixed.
    limit = None
    if aircraft is not None and aircraft['radar_limited']:
        if not within_radar(aircraft, 0, 0):
            raise ValueError('This aircraft radar cannot support a level forward launch')
        level = [r for r in aircraft['radars'] if r['elevation_deg'][0] <= 0 <= r['elevation_deg'][1]]
        # The current pinned data has symmetric azimuth limits. Fail explicitly
        # if a future catalog requires a different envelope representation.
        if any(abs(r['azimuth_deg'][0] + r['azimuth_deg'][1]) > 1e-9 for r in level):
            raise ValueError('Asymmetric aircraft radar azimuth limits need envelope support')
        limit = max(r['azimuth_deg'][1] for r in level)
    clean['launcher']['gimbal_deg'] = limit if request.get('frame', 'target') == 'launcher' else None
    return attach(clean, aircraft), aircraft
