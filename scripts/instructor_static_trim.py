from instructor_pitch_predictor import unpack_inputs


def trim_independent_authority(controls, ranges):
    return all(not available or (lo == -1. and hi == 1.)
               for available, (lo, hi) in zip(controls['trim_available'], ranges))


def static_trim(solver, state, fixed):
    cache = solver.__dict__.setdefault('_static_instructor_trim_cache', {})
    packed = fixed['auto_inputs']
    predictor = state['predictor']
    from instructor_trim_equilibrium import REVISION, solve
    # Packed inputs do not include every predictor dependency (for example,
    # loaded lateral CG and predictor flags). A shared cache must include those
    # dependencies as well as the model representation and wrapper convention.
    constants = (tuple(sorted(predictor['f'].items())),
        tuple(sorted(predictor['flags'].items())),
        tuple((k,predictor[k]) for k in ('pitch_inertia','engine_count',
            'balance_multiplier','new_balance','rho0')))
    key = (packed, fixed['rudder_trim'], constants, id(solver.model),
        solver.model.get('mach_curve_mode','native'), REVISION,
        solver.config.get('instructor_authority_mode','native'))
    if key in cache:
        return cache[key]
    ip = unpack_inputs(packed)
    corrected = solve(solver.model, ip, predictor)
    result = dict(corrected,trim=list(corrected['trim']))
    result['trim'][2] = fixed['rudder_trim']
    if len(cache) >= 2048:
        cache.clear()
    cache[key] = result
    return result
