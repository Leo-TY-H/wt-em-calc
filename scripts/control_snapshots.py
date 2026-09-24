"""Exact control-owner boolean and command-history selection, with raw records.

Snapshot offsets are relative to an 0xee0-byte record. Simulation state starts
at FM39e8; delivered state at FM2b08. Preserve the entire snapshot on transfer.
"""
import struct

SIZE=0xee0
TICK=0x2c
GENERATION=0x818
FLAGS=0xb50


def owner_uses_history(unit_flags58,unit_flags5c,unit_2f20,has_fa0,fa0_flags1d8):
    """1050d4e30, Unit+100 interface virtual+18 -> FM8470.

    The name describes its observable control-copy behavior. Raw unit bitfield
    semantics remain explicit rather than assuming this is an instructor flag.
    """
    if unit_flags58&0x100:
        return unit_flags5c&0x8001!=0x8001 or unit_2f20==0
    return bool(has_fa0 and not fa0_flags1d8&0x80)


def control_flags(autotrim,torque_gyro,complex_engine):
    """101a4d4ad..d4d2. Getter virtual+20 (Unit58 bit100) gates writing.

    Settings1e = autotrim && isAutotrimAllowed; settings28 =
    Torque_N_Gyro_Effects || its mandatory option; settings27 = ComplexEManagement.
    Context offsets21/19/56 supply these three booleans, respectively.
    """
    return int(bool(autotrim)) | (int(bool(torque_gyro))<<1) | (int(bool(complex_engine))<<2)


def select_snapshot(records,delivered,tick,prune=False,start_hint=0,generation=0):
    """101a51a10, exact hold/selection and queue-pruning behavior.

    Returns (new delivered record, remaining records, returned index). There is
    no interpolation. A generation mismatch suppresses copying but does not
    suppress pruning. With an earlier-than-first tick, the first record is used.
    """
    if len(delivered)!=SIZE or any(len(r)!=SIZE for r in records):raise ValueError('Snapshot size')
    start=max(0,start_hint);n=len(records)
    if start>=n:return delivered,list(records),-1
    index=start
    for following in range(start+1,n):
        if struct.unpack_from('<i',records[following],TICK)[0]>tick:
            index=following-1;break
    else:
        if struct.unpack_from('<i',records[start],TICK)[0]<tick:index=n-1
    if records[index][GENERATION]==generation:delivered=records[index]
    if prune and index>0:return delivered,list(records[index:]),0
    return delivered,list(records),index
def deliver_selected_jet_commands(properties,snapshot,state,ranges,dt):
    """101a4e5f0 clean selected single-jet/no-transmission consumer.

    Manual aerodynamic trim, no start/stop requests or autopilot. AB command
    and throttle are separate delivered fields. Unavailable VTOL/reverse
    controls retain their old position (zero after intact initialization).
    The no-transmission jump bypasses the later propeller/radiator block.
    """
    from primary_controls import delivered_commands
    out=dict(state)
    out['delivered']=delivered_commands(properties,snapshot['commands'],state['delivered'],ranges,dt)
    out['trim_requested']=[x if available else 0. for x,available in zip(state['trim_requested'],properties['trim_available'])]
    out['trim_actual']=[x if available else 0. for x,available in zip(state['trim_actual'],properties['trim_available'])]
    out['ab_indicator']=state['afterburner'] and state['running']==7
    out['afterburner']=bool(snapshot['afterburner']);out['throttle']=snapshot['throttle']
    return out
