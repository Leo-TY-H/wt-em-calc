"""Game fuel tank layout from the FM and the collision model's tank parts.

Used when no game-produced prop-mass asset exists for a vehicle's collision
model. Only legacy-mass FMs qualify: their mass state reads the tank layout
alone, while AdvancedMass also needs each collision part's mesh mass record.
Game producer (aces.exe 2.59.0.86, 0x143033200-0x143034c89): one slot per
tankN_dm node at the rank of N, tank<rank+1>_* keys, capacities summed in node
order, empty slots share MaxFuelMass minus that sum, then external tanks.
build_fuel_layout.py checks this against every game-produced asset.
"""
import json
import re
from functools import lru_cache
from pathlib import Path
from component_assembly import f32, add, sub, mul

TANK_PARTS = Path(__file__).resolve().parents[1] / 'references/collision-fuel-tanks.json'


def _value(value):
    while isinstance(value, list):
        value = value[0]
    return value


def declared_tanks(fm):
    rows = {}
    for key, value in fm['Mass'].get('Parts', {}).items():
        match = re.fullmatch(r'tank(\d+)_(capacity|system|external|priority)', key)
        if match:
            rows.setdefault(int(match[1]), {})[match[2]] = _value(value)
    return rows


def layout(fm, parts):
    """Tanks and fuel systems for the tankN_dm numbers `parts`, in collision node order."""
    mass = fm['Mass']
    if not parts or len(parts) > 16 or len(set(parts)) != len(parts):
        raise ValueError('Unsupported collision fuel tank parts')
    separate = bool(_value(mass.get('SeparateFuelTanks', False)))
    # Pooled fuel ignores the FM tank keys: every part is an empty system-0 tank.
    declared = declared_tanks(fm) if separate else {}
    if 'MaxFuelMass1' in mass and not separate or \
            any(f'MaxFuelMass{int(row.get("system", 0))}' not in mass for row in declared.values()):
        raise ValueError('Fuel tank system without a fuel capacity')
    rank = {n: i for i, n in enumerate(sorted(parts))}
    slots = [dict(index=i, capacity=0., system=0, priority=0, external=False) for i in range(16)]
    count, total, empty = 0, {}, {}
    for n in parts:
        row = declared.get(rank[n] + 1, {})
        if row.get('external'):
            continue
        capacity, system = f32(row.get('capacity', 0.)), int(row.get('system', 0))
        total[system] = add(total.get(system, 0.), capacity)
        if capacity == 0.:
            empty[system] = empty.get(system, 0) + 1
        slots[rank[n]].update(capacity=capacity, system=system, priority=int(row.get('priority', 0)))
        count += 1
    for t in slots[:count]:
        if t['capacity'] == 0.:
            s = t['system']
            share = f32(1. / empty[s]) if empty.get(s) else 0.
            t['capacity'] = mul(sub(f32(mass.get(f'MaxFuelMass{s}', 0.)), total.get(s, 0.)), share)
    for n in range(1, 17):
        row = declared.get(n, {})
        if row.get('external') and f32(row.get('capacity', 0.)) > 0.:
            slots[n - 1].update(capacity=f32(row['capacity']), system=int(row.get('system', 0)),
                                priority=int(row.get('priority', 0)), external=True)
            count += 1
    tanks = slots[:count]
    systems = []
    for system in range(max(t['system'] for t in tanks) + 1):
        capacity = external = 0.
        priorities = 1
        for t in tanks:
            if t['system'] == system:
                capacity = add(capacity, t['capacity'])
                if t['external']:
                    external = add(external, t['capacity'])
                priorities = max(priorities, t['priority'] + 1)
        systems.append(dict(capacity=capacity, external_capacity=external,
                            reservoir_capacity=f32(mass.get(f'FuelAccumulatorCapacity{system}', 0.)),
                            priority_count=priorities))
    return tanks, systems


@lru_cache(maxsize=1)
def tank_parts():
    return json.loads(TANK_PARTS.read_text()) if TANK_PARTS.exists() else dict(resources={}, unavailable=[])


def asset(fm, resource):
    """Mass asset for a legacy-mass FM whose collision model has no game-produced asset."""
    if resource in tank_parts()['unavailable']:
        raise ValueError('The game files have no collision model for this vehicle')
    if fm['Mass'].get('AdvancedMass', False):
        raise ValueError('Advanced mass needs collision part masses, which are not generated for this model')
    row = tank_parts()['resources'].get(resource)
    if row is None:
        raise ValueError('Fuel tank layout of this model is unavailable')
    tanks, systems = layout(fm, row['tank_parts'])
    return dict(resource=resource, producer='fuel-layout', tanks=tanks, fuel_systems=systems,
                records=[], damage_records=[])
