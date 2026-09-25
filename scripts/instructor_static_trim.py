"""Independent one-g auto-trim solve for each static operating condition.

The native predictor's nested force/moment iterations are algebraic solves,
not elapsed controller time. Always start from the same numerical guess;
never retain a previous speed's trim or use the balanced-turn elevator as trim.
"""
from component_assembly import add, sub, mul, f32
from instructor_autotrim import autotrim_predictor
from instructor_pitch_predictor import unpack_inputs, prepare_geometry


def equilibrium_valid(auto):
    """The native success flag alone does not test lift-loop convergence."""
    residual=auto['equilibrium']
    return (auto['success'] and abs(residual['lift_error_n'])<20.
            and abs(residual['moment_error_nm'])<20.)


def trim_independent_authority(controls, ranges):
    """Whether unknown bounded trim can change any delivered-control endpoint.

    At full authority the native trim mixer maps stick -1/+1 to -1/+1 for
    every trim in [-1, 1]. Its continuous monotone map covers that whole
    interval. Unavailable trim is identically zero. Thus these axes need no
    auto-trim root to certify reachability; compressed axes still do.
    """
    return all(not available or (lo == -1. and hi == 1.)
               for available, (lo, hi) in zip(controls['trim_available'], ranges))


def static_trim(solver, state, fixed):
    cache = solver.__dict__.setdefault('_static_instructor_trim_cache', {})
    packed = fixed['auto_inputs']
    # Source mass/geometry/flags are immutable for one TrimSolver. Packed inputs
    # include Mach, TAS, engine force/moment/wash, flaps, gear and altitude.
    key = (packed, fixed['rudder_trim'])
    if key in cache:
        return cache[key]
    ip = unpack_inputs(packed)
    predictor = state['predictor']
    geometry = prepare_geometry(solver.model, ip, predictor)
    areas = [add(add(s[1], s[0]), s[2]) for s in geometry['geometry']['areas']]
    asymmetric = (abs(ip[0x5c]) > f32(.01)
        or abs(sub(*areas)) > mul(add(*areas), f32(.01))
        or abs(sub(*geometry['tail_areas'])) > mul(geometry['tail_area'], f32(.01))
        or abs(predictor['f'].get(0x5328, 0.)) > f32(.1))
    # Never make the app execute an installed game binary. The asymmetric
    # predictor remains a research-only native backend, not a static app port.
    if asymmetric:
        result = dict(success=False, trim=[0., 0., 0.],
                      reason='Asymmetric one-g auto trim is not ported')
    else:
        auto = autotrim_predictor(solver.model, ip, predictor, (0., 0., False))
        method='native iteration'
        if not equilibrium_valid(auto):
            # Native ten-pass inverse lift iteration can oscillate between
            # pre-stall AoAs, including a false success with a huge lift error.
            # Re-solve the SAME algebraic equations from the SAME zero guess.
            # This damps numerical updates, not flight/control transients.
            for relaxation,iterations in ((.5,64),(.25,128)):
                auto=autotrim_predictor(solver.model,ip,predictor,(0.,0.,False),
                    lift_relaxation=relaxation,lift_iterations=iterations)
                method='relaxed algebraic lift solve'
                if equilibrium_valid(auto):break
        if not equilibrium_valid(auto):
            auto=autotrim_predictor(solver.model,ip,predictor,(0.,0.,False),
                lift_iterations=64,lift_bisection=True)
            method='bracketed algebraic lift solve'
        success=bool(equilibrium_valid(auto))
        result = dict(success=success,method=method,equilibrium=auto['equilibrium'],
            trim=[auto['output'][2], auto['output'][1], fixed['rudder_trim']],
            reason=None if success else 'One-g auto-trim equilibrium unresolved',
            one_g_alpha_deg=auto['output'][0])
    if len(cache) >= 2048:
        cache.clear()
    cache[key] = result
    return result
