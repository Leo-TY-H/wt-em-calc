"""Composite carriers that fly a 'bomb plane' (Mistel 2: Fw 190 A-8 on a Ju 88 G-1 Mistel).

Game rule (installed aces.exe 2.61.0.17; the pinned 2.59 executable predates the Mistel):
- The carrier vehicle names its bomb plane (bombPlaneName, unit loader 0x140dd8804); the bomb plane is a separate
  aircraft unit with its own FM, spawned and joined at the hook node (0x140e3b470).
- While joined, only the bomb plane's FM is integrated (full step 0x14318f220); the carrier takes a reduced step
  with no motion integration (0x14318ee70) and is moved with the bomb plane (unit update 0x140defb00). The pilot's
  stick, flap and gear commands drive the bomb plane (0x140dfcd90), and its engines get the mean of the carrier's
  throttles clamped to 1.1; propeller pitch, mixture and radiators are not copied, so they stay automatic. The bomb
  plane's own controller (Instructor, limits) runs.
- The carrier's mass, CG and drag reach the bomb plane only when fmConstants/useAllPayloads is true (payload gate
  0x14318d09b); the game data sets it false, so the carrier adds nothing.
- Releasing the store (releasesBombPlane) unlinks them (0x140e3c5a0); the carrier then flies its own FM with the
  store spent (no mass, no drag).
"""
from functools import lru_cache
import orjson

from input_paths import REFERENCES


@lru_cache(maxsize=None)
def bomb_plane(name):
    """The bomb plane a carrier vehicle names, or None."""
    path = REFERENCES / 'prop-vehicles' / (name + '.blkx')
    if not path.is_file():
        return None
    value = orjson.loads(path.read_bytes()).get('bombPlaneName')
    return value if isinstance(value, str) and value else None


@lru_cache(maxsize=1)
def uses_all_payloads():
    """fmConstants/useAllPayloads of the game build (default false)."""
    path = REFERENCES / 'body-gameplay.blkx'
    if not path.is_file():
        return False
    return (orjson.loads(path.read_bytes()).get('fmConstants') or {}).get('useAllPayloads') is True


def carried(name, config):
    """The bomb plane flying the chart: when the carrier's loadout holds the store that releases it and the stores are
    not released. None otherwise (the carrier flies its own FM)."""
    plane = bomb_plane(name)
    if plane is None or not config.get('loadout') or config.get('stores_released'):
        return None
    from aircraft_loadouts import resolve, _weapon_document
    loadout = resolve(name, config.get('ammunition_vehicle', ''), config['loadout'], released=False)
    if not any(item['k'] == 's' and (_weapon_document(item['w']) or {}).get('releasesBombPlane') for item in loadout['items']):
        return None
    if uses_all_payloads():
        raise ValueError('This game build adds the carrier to the bomb plane as a payload (fmConstants/useAllPayloads), '
                         'which the calculator does not model yet')
    from aircraft_catalog import catalog
    row = catalog().get(plane)
    if not row or not row.get('supported'):
        raise ValueError('The bomb plane ' + plane + ' is unavailable: ' + str((row or {}).get('reason', 'not in the catalog')))
    return plane


@lru_cache(maxsize=None)
def is_bomb_plane(vehicle):
    """A vehicle the game spawns only as another aircraft's bomb plane (vehicle blk bombPlane, e.g. ju_88g_1_mistel):
    not selectable on its own, so the aircraft pickers leave it out; carriers still fly its FM."""
    path = REFERENCES / 'prop-vehicles' / (vehicle + '.blkx')
    return path.is_file() and orjson.loads(path.read_bytes()).get('bombPlane') is True


def mark_bomb_planes(aircraft):
    """Catalog rows for the page: bomb planes carry bomb_plane=True (copies; the solver catalog is untouched)."""
    return {name: dict(row, bomb_plane=True) if is_bomb_plane(row.get('vehicle_id') or name) else row
            for name, row in aircraft.items()}


def flight(name, config):
    """(aircraft, settings) whose flight model flies a chart of `name` with `config`: the bomb plane while joined."""
    plane = carried(name, config)
    return (plane, flight_settings(config, plane)) if plane else (name, config)


def flight_settings(config, plane):
    """The bomb plane's own settings while joined: the carrier's flight conditions, no stores of its own, automatic
    engine controls (only the throttle is passed on)."""
    return dict(config, aircraft=[plane], loadout='', stores_released=False, default_ammunition=False, ammunition_vehicle='',
                manual_engine_controls=False, propeller_pitch_auto=True, radiator_percent=0., oil_radiator_percent=0.,
                boosters=False, booster_fuel_percent=100.)
