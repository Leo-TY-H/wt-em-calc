"""Skip invariant full controller cycles before overload targets begin moving."""
from component_assembly import f32


def periodic_timer_prefix(records, timer, step, floor, budget):
    """Return exact rounded timer additions spanning complete repeated cycles.

    All recorded controller fields except the overload timer must repeat bit
    for bit for three cycles. The caller requires a fixed timestep ring and a
    fixed authority factor. Below floor, the timer cannot affect the target.
    Whole-cycle skipping leaves every other history in the same native phase.
    """
    period=0
    for candidate in (2,3,4,8):
        if len(records)<3*candidate:
            continue
        same=True
        for i in range(2*candidate):
            a=records[-1-i][0];b=records[-1-i-candidate][0]
            if a[:-3]+a[-2:] != b[:-3]+b[-2:]:
                same=False
                break
        if same:
            period=candidate
            break
    if not period:
        return 0,timer
    skipped=0
    while skipped+period<=budget:
        trial=timer
        for _ in range(period):
            next_timer=min(1.,f32(trial+step))
            if next_timer>=floor or next_timer==trial:
                return skipped,timer
            trial=next_timer
        timer=trial
        skipped+=period
    return skipped,timer
