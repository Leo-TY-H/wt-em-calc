"""Engine command availability from native1019f7a40.

This is the delivered command bit, not a thrust multiplier. The intact-engine
scope excludes the separate type-1 activation damage side effect; that side
effect does not change this bit's assignment.
"""


def afterburner_command(requested, boost_type, controllable):
    return bool(requested and ((boost_type > 0 and controllable) or boost_type in (4, 8, 10)))
