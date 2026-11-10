def profile(fm,config=None):
    out=dict(experimental=True, kind='steady AoA schedule',
        input='native critical-angle targets and established adjusted wing AoA',
        correction='native settled wing-angle adjustment, rate feedback and reduced forward moment balance; full aircraft trim enforces control authority',
        branch='aircraft equilibrium below positive stall; same AoA constraint at boundary and interior',
        trim=(('Trim-independent endpoints evaluated directly; otherwise simultaneous one-g equilibrium with original recovery; fixed trim bounds pilot authority')),
        authority_mode=(config or {}).get('instructor_authority_mode','native'),
        power='selected engine power; native thrust force, moment and propwash retained',
        history='static schedule; transient overload reserve/release omitted',
        physical_limits='positive-AoA stall, strength, control authority and configured speed limits retained',
        capability_status='Algebraic steady effective AoA schedule (approximation). Native rate feedback, elevator compression and physical limits retained; transient overshoot, delay and control history omitted.',
        validated_live=False)
    if config is None:return out
    # Leo 2026-11-10 (wing_line.py): with a wing protection state the wing breaking load is a chart line and the
    # protection is held or released on the chart by its release start (never written as inf)
    import math
    from wing_line import protection as wing_protection
    state,seconds=wing_protection(fm,config)
    if state is None:return out
    out.update(history='static schedule; wing protection held or released by its release start (10 s threshold)',
        physical_limits='positive-AoA stall, control authority and configured speed limits retained; the wing breaking load is charted as a line',
        capability_status='Algebraic steady effective AoA schedule (approximation). Native rate feedback, elevator compression and stall, authority and speed limits retained; transient overshoot, delay and control history omitted.',
        wing_protection=dict(state=state,release_start_s=None if math.isinf(seconds) else seconds))
    return out


def controller_limits(solver,value,speed=None):
    from instructor_steady_aoa import controller_limits as schedule_limits
    return schedule_limits(solver,value,speed)
