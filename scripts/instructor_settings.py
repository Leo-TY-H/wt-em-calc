from component_assembly import mul
from functools import lru_cache
import json
from pathlib import Path


@lru_cache(maxsize=1)
def payload_multipliers():
    """Authored RB/SB base and assistance multipliers; no aircraft fitting."""
    root=Path(__file__).resolve().parents[1]/'references'
    difficulty=json.loads((root/'body-gameparams.blkx').read_text())['difficulty_settings']['baseDifficulty']
    constants=json.loads((root/'body-gameplay.blkx').read_text())['fmConstants']
    # The supported battle difficulties share the base. Assistance is selected
    # separately by control-snapshot bit 0, not by torque/gyro or manual trim.
    base=difficulty['medium']['payloadCoGMultiplier']
    if base!=difficulty['hard']['payloadCoGMultiplier']:
        raise ValueError('RB/SB payload base multipliers require an explicit battle-difficulty setting')
    return base,constants.get('instructorPayloadCogOffsetMult',1.)


def payload_scales(snapshot_flags,base_multiplier,instructor_multiplier):
    scale=mul(instructor_multiplier if snapshot_flags&1 else 1.,base_multiplier)
    return [scale,scale]


def trim_retained(manual_trim,ground_trim,snapshot_flags,autotrim_allowed):
    auto=bool(autotrim_allowed and snapshot_flags&1)
    return [bool(manual_trim[i] or ground_trim[i] or (i!=0 and auto)) for i in range(3)]


def deliver_keyboard_commands(properties,snapshot,state,ranges,dt,*,autotrim=True,
                              autotrim_allowed=True,ground_trim=(False,False,False)):
    from control_snapshots import deliver_selected_jet_commands
    p=dict(properties,trim_available=trim_retained(properties['trim_available'],ground_trim,int(autotrim),autotrim_allowed))
    return deliver_selected_jet_commands(p,snapshot,state,ranges,dt)


def restored_wing_normalization(areas,health=((1.,1.,1.),(1.,1.,1.))):
    from component_assembly import add
    from instructor_protection import divide
    totals=[];ratios=[]
    for a,h in zip(areas,health):
        nominal=add(add(a[1],a[0]),a[2])
        actual=add(mul(a[2],h[2]),add(mul(a[1],h[1]),mul(a[0],h[0])))
        totals.append(actual);ratios.append(mul(actual,divide(1.,nominal)))
    return {0x8438:add(totals[1],totals[0]),0x843c:ratios[0],0x8440:ratios[1]}
