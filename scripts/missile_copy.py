"""Fast snapshot copies for the website's mostly plain-data state graphs.

Retain deepcopy's memo/alias/cycle contract. Subclasses, tuples and custom
objects delegate to the standard implementation with the same memo.
"""
from copy import deepcopy as reference_copy
import math
from missile_model.state_binary32 import validate_flight_numbers as reference_validator

_ATOMIC = frozenset((float, int, str, bool, bytes, type(None)))


def deepcopy(value, memo=None):
    kind = type(value)
    if kind in _ATOMIC:
        return value
    if kind is not dict and kind is not list:
        return reference_copy(value, memo)
    if memo is None:
        memo = {}
    identity = id(value)
    if identity in memo:
        return memo[identity]
    if kind is dict:
        output = {}
        memo[identity] = output
        for key, item in value.items():
            output[key if type(key) in _ATOMIC else deepcopy(key, memo)] = (
                item if type(item) in _ATOMIC else deepcopy(item, memo))
    else:
        output = []
        memo[identity] = output
        for item in value:
            output.append(item if type(item) in _ATOMIC else deepcopy(item, memo))
    # Match copy._keep_alive for custom callbacks that create transient objects.
    memo.setdefault(id(memo), []).append(value)
    return output


# Ordinary states contain only finite floats. Avoid constructing a path tuple
# for every leaf; unusual special-word states use the original slot checks.

def _finite_graph(value):
    if isinstance(value,float):
        return math.isfinite(value)
    if isinstance(value,dict):
        values=value.values()
    elif isinstance(value,(list,tuple)):
        values=value
    else:
        return True
    for item in values:
        if isinstance(item,float):
            if not math.isfinite(item):return False
        elif isinstance(item,(dict,list,tuple)) and not _finite_graph(item):
            return False
    return True


def validate_flight_numbers(state):
    if not _finite_graph(state):
        return reference_validator(state)
