"""Finite enumeration of self-consistent governed requests at fixed body flow.

Pitch and wake do not enter engine_output. Engine stage affects target_omega
only through discrete boost selection; eligible stages are finite. Enumerating
those requests and replaying the actual engine selector at each assigned speed
therefore covers requests, not torque/wake equilibria or aircraft trims.
"""
import itertools
import math

from em_manual_controls import manual_rpm_governor
from prop_quasisteady import engine_output
from propulsion_general import delivered_boost, target_omega


def governed_links(properties, seed, shaft):
    return [link for link in shaft['propellers']
            if properties['propellers'][link['index']]['properties']['governor']
            and (seed['propellers'][link['index']]['auto']
                 or manual_rpm_governor(properties['propellers'][link['index']]['properties']))]


def requests(properties, seed, shaft, links, engines, nitro):
    target = max((target_omega(properties['engines'][link['index']]['properties'],
                              engines[link['index']], nitro) for link in shaft['engines']), default=0.)
    boost = any(properties['engines'][link['index']]['properties']['boost_controllable']
                and delivered_boost(properties['engines'][link['index']]['properties'],
                                    engines[link['index']], nitro) for link in shaft['engines'])
    desired = []
    for link in links:
        index = link['index']
        pp = properties['propellers'][index]['properties']
        state = seed['propellers'][index]
        value = target
        if manual_rpm_governor(pp):
            maximum = pp['boost_omega'] if boost else pp['max_omega']
            value = (min(max(value, pp['min_omega']), maximum) if state['auto'] else
                     pp['min_omega'] + state['command'] * (maximum - pp['min_omega']))
        desired.append(value * pp['reduction'] / link['ratio'])
    return desired


def enumerate_speeds(properties, seed, shaft_index, velocity, height, nitro, cg):
    """Seed must use the declared nominal auxiliary engine fields, not a warm state.

    The caller prepares turbine gas-generator targets before this operation.
    Exceptions remain model/adapter gaps; they are not converted into absence.
    """
    shaft = properties['transmissions'][shaft_index]
    links = governed_links(properties, seed, shaft)
    if not links:
        return dict(governed=False, candidates=[], excluded=[], request_coverage_complete=True)
    indices = [link['index'] for link in shaft['engines']]
    stages = []
    for index in indices:
        engine = properties['engines'][index]
        # Turbine/rocket output retains its seed gear. Automatic pistons choose
        # an authored stage, or stage zero on the explicit no-winner branch.
        stages.append(range(max(1, len(engine['properties']['stages'])))
                      if engine['family'] in (0, 1) else [seed['engines'][index]['gear']])
    possible = set()
    for gears in itertools.product(*stages):
        states = [dict(state) for state in seed['engines']]
        for index, gear in zip(indices, gears):states[index]['gear'] = gear
        # Every consistent shared request is in this union, including cases
        # whose other hypothetical stage requests are incompatible.
        possible.update(requests(properties, seed, shaft, links, states, nitro))
    if not all(math.isfinite(speed) for speed in possible):
        raise ValueError('Nonfinite governed request; source-model calculation unresolved')
    candidates, excluded = [], []
    for speed in sorted(possible):
        states = [dict(state) for state in seed['engines']]
        for link in shaft['engines']:
            index = link['index']
            state = dict(states[index], omega=speed * link['ratio'])
            states[index] = engine_output(properties['engines'][index], state, velocity, height, nitro, cg)
        actual = requests(properties, seed, shaft, links, states, nitro)
        record = dict(shaft_speed=speed, actual_requests=actual,
                      engine_gears=[states[index]['gear'] for index in indices],
                      engines={index: states[index] for index in indices})
        if all(value == speed for value in actual):candidates.append(record)
        else:excluded.append(record)
    return dict(governed=True, candidates=candidates, excluded=excluded,
                request_coverage_complete=True,
                equilibrium_coverage_complete=False)
