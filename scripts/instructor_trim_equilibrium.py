"""The retained three-equation one-g Instructor authority relation.

The source predictor supplies both residuals and admission guards. Numerical
correction is shared with the stationary aircraft and propulsion equations;
there is no alternative finite-iteration model after a failed correction.

A failed correction is restarted, with the same equations, corrector settings
and admission test, from its returned wing angle and tail force with the
elevator command placed on each side of zero (RESTART_COMMANDS, in order).
The first correction starts at command exactly 0, where the control travel
has a kink: one-sided derivative samples taken there can describe the wrong
side of a root that exists (Rafale C at 5,000 m, 899-909 km/h). The first
admitted restart is returned; when none is admitted the first correction is
returned as before. Counts after a restart:
  evaluations                         distinct predictor states evaluated by
                                      the whole solve (seeds, first
                                      correction and every restart);
  correction.nfev / .evaluations /    objective samples / all samples /
    .derivative_evaluations           derivative samples summed over the
                                      first correction and every restart;
  correction.restart_attempts         restarts run (0, 1 or 2);
  correction.restart_command          command the returned restart started
                                      from, None when the first is returned.
Termination, residual, closure ratio and coordinates describe the returned
correction. The native twin (march_native/instr.pxi trim_solve) follows this
operation for operation.
"""
import numpy as np

from instructor_autotrim import autotrim_predictor
from instructor_pitch_predictor import prepare_geometry

REVISION = 'steady-authority-moment-initialization-v5'
RESTART_COMMANDS = (-.01, .01)


def solve(model, ip, state):
    c = prepare_geometry(model, ip, state)
    weight = max(abs(ip[0x2c]), 1.)
    moment_scale = weight * max(abs(c['tail_lever']), 1.)
    p = c['polars'][0]
    cl = weight / max(sum(c['q_area']) * c['cos_dihedral'], 1.)
    slope = p['clLineCoeff'] * p['clKq']
    angle = (cl - p['cl0'] * p['clKq']) / slope if abs(slope) > 1e-9 else 0.
    angle = min(p['aoaCritH'], max(p['aoaCritL'], angle))
    calls = 0
    memo = {}

    def evaluate(x):
        nonlocal calls
        key = tuple(map(float, x))
        if key not in memo:
            calls += 1
            auto = autotrim_predictor(model, ip, state,
                equilibrium_state=(x[0], x[1] * weight, x[2]))
            eq = auto['equilibrium']
            residual = np.array([eq['lift_error_n'] / weight,
                eq['moment_error_nm'] / moment_scale, eq['command_error']])
            # These known signs and saturation state guide local stencils only.
            # They do not claim to enumerate every inverse-polar/source guard.
            memo[key] = dict(residual=residual, auto=auto,
                proposal_branch=(x[0]>=0.,x[1]>=0.,x[2]>=0.,auto['saturated']))
        return memo[key]

    # These are representation bounds, not a newly assumed physical domain.
    # The predictor consumes finite binary32 angle/tail-force/command values.
    # Only the existing |command|<=1 admission rule supplies a physical bound.
    finite32 = np.nextafter(float(np.finfo(np.float32).max), 0.)
    lower = [-finite32, -finite32/weight, -1.]
    upper = [finite32, finite32/weight, 1.]
    initial = [angle, 0., 0.]
    if abs(c['tail_lever']) > 4e-19:
        seed = evaluate(initial)['auto']
        tail = seed['equilibrium']['moment_error_nm'] / c['tail_lever'] / weight
        # Use the native pitch-moment update as an initial guess. A saturated
        # command adds an unmet-authority term, so it cannot seed this balance.
        # Keep the simultaneous equations and their admission gates unchanged.
        if not seed['saturated'] and np.isfinite(tail) and lower[1] <= tail <= upper[1]:
            initial[1] = tail
            seeded = evaluate(initial)['auto']
            lift_slope = sum(c['q_area']) * c['cos_dihedral'] * slope * p['cyMult']
            # The new tail force also carries lift, especially on canards.
            # Re-estimate wing angle once before the simultaneous correction.
            if not seeded['saturated'] and abs(lift_slope) > 1e-9:
                adjusted = angle + seeded['equilibrium']['lift_error_n'] / lift_slope
                if np.isfinite(adjusted):
                    initial[0] = min(p['aoaCritH'], max(p['aoaCritL'], adjusted))
    # Defer this import: compiled Instructor wrappers import this Python module
    # during backend activation, before all aircraft modules are installed.
    from em_stationary_corrector import Corrector
    from em_trim_numerics import UnevaluableTrial

    def correct(start):
        corrector = Corrector(evaluate, (lower,upper), [.002,2e-5,.0002],
            branch=lambda value:value['proposal_branch'],
            scale=[10.,1.,.2], step_limit=[8.,.3,.4],
            residual_scale=[20./weight,20./moment_scale,2e-6])
        fit = corrector.solve(start, max_nfev=120,
                              numerical_target=None, reuse_jacobian=True)
        value = evaluate(fit.x)
        auto = value['auto']
        eq = auto['equilibrium']
        success = (auto['success'] and abs(eq['lift_error_n']) < 20.
            and abs(eq['moment_error_nm']) < 20. and abs(eq['command_error']) < 2e-6
            and abs(auto['output'][1]) <= 1. and np.isfinite(fit.x).all())
        return fit, value, success

    fit, value, success = correct(initial)
    nfev, samples = fit.nfev, fit.evaluations
    restart_command, restart_attempts = None, 0
    if not success and np.isfinite(fit.x).all():
        first = fit.x
        for command in RESTART_COMMANDS:
            restart_attempts += 1
            try:
                again = correct([first[0], first[1], command])
            except UnevaluableTrial:
                # Only the first sample of a correction can raise this (later
                # unevaluable trials are skipped inside it): one objective
                # sample was spent and this restart is not admitted.
                nfev += 1
                samples += 1
                continue
            nfev += again[0].nfev
            samples += again[0].evaluations
            if again[2]:
                fit, value, success = again
                restart_command = command
                break
    auto = value['auto']
    eq = auto['equilibrium']
    return dict(success=bool(success), method='shared correction of simultaneous one-g relation',
        equilibrium=eq, trim=[auto['output'][2], auto['output'][1], 0.],
        one_g_alpha_deg=auto['output'][0], evaluations=calls,
        reason=None if success else 'One-g equilibrium correction unresolved',
        coordinates=list(map(float,fit.x)),
        coordinate_names=['predictor_angle_deg','tail_force_over_weight','elevator_command'],
        search_bounds=[lower,upper],
        search_bound_scope='Finite binary32 predictor states and retained command bound; no coverage proof',
        local_admission=bool(success),coverage_complete=False,
        correction=dict(termination=fit.termination,nfev=nfev,
            evaluations=samples,derivative_evaluations=samples-nfev,
            residual_max=float(np.max(abs(value['residual']),initial=0.)),
            return_metric=fit.return_metric,closure_ratio=fit.closure_ratio,
            numerical_target_reached=fit.success,
            restart_command=restart_command,restart_attempts=restart_attempts),
        revision=REVISION)
