"""Recovered Instructor owner scheduling and gates, research only.

The raw owner fields are explicit. This does not guess network/spawn state or
replace the controller and aircraft with a static angle cap. Instruction spans
belong to 104f6af60 in the pinned executable.
"""
import math
from component_assembly import f32,mul


def prediction_count(*,world_time,physics_dt,current_tick,previous_interval,time_scale):
    """104f6b0da..b14c, 104f701d0..23f and 104f703f3..70466.

    FM4ea8 is the float physics timestep; FM4eac is passed by its raw field
    role, not asserted to be render dt. Finite positive dt, bounded signed
    ticks and nonnegative source intervals are the supported domain.
    ROUNDSS immediate 0xa means round toward +infinity, not nearest.
    """
    dt=f32(physics_dt)
    frequency=f32(1./dt)
    now_tick=int(float(world_time)/dt)
    count=max(now_tick-int(current_tick),0)
    threshold=math.ceil(mul(f32(.7),frequency))
    capped=count>=threshold
    if capped:
        base=math.ceil(mul(frequency,f32(.11)))
        cap=math.ceil(mul(max(f32(time_scale),1.),f32(base)))
        age=math.ceil(mul(f32(previous_interval),frequency))
        count=min(cap,max(cap,1)+age-1)
    return dict(count=count,now_tick=now_tick,threshold=threshold,capped=capped,
                final_dt=mul(f32(count+1),dt))


def recovery_suppressed(*,unit_flags108,config332,config330,unit4314):
    """104f70202..247, input+9 producer; raw field identities retained."""
    return bool(config332 or config330) if unit_flags108&6 else bool(unit4314)


def dispatch_enabled(*,selector,difficulty309,unit_flags108,config332,config330,
                     unit4314,longitudinal_speed,speed_threshold,has_fm6ef0):
    """104f7070a..708b4, orbiting-off input+29 producer.

    FM6ef0 is kept as an explicit provider-presence predicate. UI side effects
    after this decision do not alter this byte or the restored pilot commands.
    """
    if selector!=2 and not difficulty309:return False
    suppressed=recovery_suppressed(unit_flags108=unit_flags108,config332=config332,
        config330=config330,unit4314=unit4314)
    if suppressed and f32(longitudinal_speed)<=f32(speed_threshold):return False
    return not has_fm6ef0


def prediction_calls(current_tick,count,physics_dt):
    """Original loop70504..705bc then final call708b4..708ee.

    Every loop iteration selects that tick's held snapshot, restores the same
    pilot request, runs Instructor with input29=false and then advances physics.
    The final call uses updated caller flags and dt=(count+1)*physics_dt. It
    does not immediately run another physics step in this owner span.
    """
    dt=f32(physics_dt);calls=[]
    for tick in range(current_tick,current_tick+max(0,count)):
        calls.extend([dict(stage='select_snapshot',tick=tick),
                      dict(stage='controller',dt=dt,dispatch=False),
                      dict(stage='physics',tick=tick,dt=dt,produce_primary=False)])
    calls.append(dict(stage='final_controller',dt=mul(f32(count+1),dt)))
    return calls


def produces_primary_snapshot(*,phase,has_owner,owner_flags108):
    """101a3ea7e..eab4: boolean physics phase and owner interface flag gate.

    The owner's look-ahead loop passes phase=0. Therefore it does NOT run the
    next-command actuator/snapshot producer101a4b9a0, even when the owner exists.
    Authority, held-command delivery, engines and physical prediction still run.
    """
    return bool(phase and has_owner and owner_flags108&2)
