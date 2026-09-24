"""Experimental static Instructor approximation on detailed aircraft equilibria."""


def profile(fm):
    return dict(experimental=True, kind='effective AoA limiter',
        input='native critical-angle targets and established adjusted wing AoA',
        correction='native rate feedback and reduced predictor force/moment balance at delivered elevator',
        branch='aircraft equilibrium below positive stall; same AoA constraint at boundary and interior',
        trim='free trim allocation; native retained-trim permission is not reproduced',
        power='selected engine power; native thrust force, moment and propwash retained',
        history='static schedule; transient overload reserve/release omitted',
        physical_limits='positive-AoA stall, strength, control authority and configured speed limits retained',
        capability_status='EXPERIMENTAL: static AoA approximation; boundary and interior permission unvalidated',
        validated_live=False)


def controller_limits(solver,value,speed=None):
    from instructor_aoa import controller_limits as effective_limits
    return effective_limits(solver,value,speed)

