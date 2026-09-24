"""Recovered owner clock and real-physics catch-up scheduling, research only.

These are separate from Instructor look-ahead scheduling. Runtime profile,
queue, frame time and FM interval inputs remain explicit; none is fitted to an
EM boundary. Finite times, positive float32 dt and bounded signed ticks only.
"""
import math
from component_assembly import f32, mul


def clock_source(*,has_network,network_predicate_dd50,network_predicate_a620,
                 has_world400,world400_5e9):
    """104e59ce0. The two network predicate identities remain raw offsets."""
    if has_network and not network_predicate_dd50 and not network_predicate_a620:
        return 'network+70'
    if has_world400 and world400_5e9 == 1:
        return 'world400+638'
    return 'world+2d0'


def catchup_schedule(*,world_time,physics_dt,current_tick,previous_interval,
                     time_scale,controls_enabled,newest_control_tick=None,
                     fm1610=True,reference_time=0.):
    """Whole 104fb9de0 arithmetic and selected state writes.

    Unlike prediction_count, catch-up has a now_tick+1 endpoint and multiplies
    the time scale BEFORE rounding the .11-second budget. The game-authored
    catch-up budget is a scheduling rule, not an aircraft-performance cap.
    The newest record's tick is used as stored; queue sorting is not inferred.
    `previous_interval` retains the older research API name for FM+4eac:
    constructor/getter/setter layout matches maxTimeDeferredControls in the
    bundled Dagor source. It is not a measured render-frame interval. The
    base constructor sets .35, but aircraft initialization can override it.
    """
    dt = f32(physics_dt)
    frequency = f32(1. / dt)
    interval_ticks = max(math.ceil(mul(f32(previous_interval), frequency)), 0)
    threshold = math.ceil(mul(f32(.7), frequency))
    budget = math.ceil(mul(mul(max(f32(time_scale), 1.), f32(.11)), frequency))
    now_tick = int(float(world_time) / dt)
    target = now_tick + 1
    start = int(current_tick)
    leaped = target - start >= threshold
    retained_flag = bool(fm1610)
    if leaped:
        start = now_tick - (max(budget, 1) + interval_ticks) + 2
        retained_flag = False
    minimum_target = now_tick + 1 - interval_ticks
    record_target = minimum_target if newest_control_tick is None else int(newest_control_tick) + 1
    if controls_enabled:
        target = max(min(target, record_target), minimum_target)
    if target == minimum_target:
        retained_flag = False
    end = min(start + budget, target)
    start_time = float(start) * float(dt)
    return dict(now_tick=now_tick,interval_ticks=interval_ticks,threshold=threshold,
                budget=budget,start_tick=start,end_tick=end,
                minimum_target=minimum_target,leaped=leaped,fm1610=retained_flag,
                multiple_steps=start+1<end,start_time=start_time,
                at_or_after_reference=start_time>=reference_time)


def needs_controller(*,now_tick,newest_control_tick):
    """104f6b317..34c: queue gate for the selected owner look-ahead branch."""
    return newest_control_tick is None or int(newest_control_tick) < int(now_tick)


def needs_publication(*,now_tick,newest_control_tick):
    """104f6b5f8..64f: publication's comparison is !=, not the above <."""
    return newest_control_tick is None or int(newest_control_tick) != int(now_tick)
