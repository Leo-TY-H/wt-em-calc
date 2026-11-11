"""Curated aircraft/vehicle exclusions, independent of downloaded FM assets."""
from functools import lru_cache
from pathlib import Path
import orjson

from input_paths import REFERENCES, policy
EXCLUSIONS = policy('aircraft-exclusions.json')
INPUT_BUILD = REFERENCES / 'input-build.json'


@lru_cache(maxsize=1)
def _policy():
    return orjson.loads(EXCLUSIONS.read_bytes())


def exclusions():
    return _policy()['aircraft']


@lru_cache(maxsize=1)
def input_build():
    """Record of a hosted compile: which items keep an older version, and which didn't build."""
    return orjson.loads(INPUT_BUILD.read_bytes()) if INPUT_BUILD.is_file() else {}


def excluded_vehicles():
    # Catalog keys can be FM IDs; a removed FM need not exclude its vehicle
    # when that playable vehicle is represented by a different, shared FM.
    return _policy().get('vehicle_variants', {})
