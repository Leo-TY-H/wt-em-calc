"""Curated aircraft/vehicle exclusions, independent of downloaded FM assets."""
from functools import lru_cache
from pathlib import Path
import orjson

EXCLUSIONS = Path(__file__).resolve().parents[1] / 'references/aircraft-exclusions.json'


@lru_cache(maxsize=1)
def _policy():
    return orjson.loads(EXCLUSIONS.read_bytes())


def exclusions():
    return _policy()['aircraft']


def excluded_vehicles():
    # Catalog keys can be FM IDs; a removed FM need not exclude its vehicle
    # when that playable vehicle is represented by a different, shared FM.
    return _policy().get('vehicle_variants', {})
