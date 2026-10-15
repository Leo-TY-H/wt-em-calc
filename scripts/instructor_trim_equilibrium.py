"""The retained three-equation one-g Instructor authority relation.

The source predictor supplies both residuals and admission guards. Numerical
correction is shared with the stationary aircraft and propulsion equations;
there is no alternative finite-iteration model after a failed correction.
"""
import numpy as np

from instructor_autotrim import autotrim_predictor
from instructor_pitch_predictor import prepare_geometry

REVISION = 'steady-authority-shared-correction-v3'


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
    # Defer this import: compiled Instructor wrappers import this Python module
    # during backend activation, before all aircraft modules are installed.
    from em_stationary_corrector import Corrector
    corrector = Corrector(evaluate, (lower,upper), [.002,2e-5,.0002],
        branch=lambda value:value['proposal_branch'],
        scale=[10.,1.,.2], step_limit=[8.,.3,.4],
        residual_scale=[20./weight,20./moment_scale,2e-6])
    fit = corrector.solve([angle,0.,0.], max_nfev=120,
                          numerical_target=None, reuse_jacobian=True)
    value = evaluate(fit.x)
    auto = value['auto']
    eq = auto['equilibrium']

    success = (auto['success'] and abs(eq['lift_error_n']) < 20.
        and abs(eq['moment_error_nm']) < 20. and abs(eq['command_error']) < 2e-6
        and abs(auto['output'][1]) <= 1. and np.isfinite(fit.x).all())
    return dict(success=bool(success), method='shared correction of simultaneous one-g relation',
        equilibrium=eq, trim=[auto['output'][2], auto['output'][1], 0.],
        one_g_alpha_deg=auto['output'][0], evaluations=calls,
        reason=None if success else 'One-g equilibrium correction unresolved',
        coordinates=list(map(float,fit.x)),
        coordinate_names=['predictor_angle_deg','tail_force_over_weight','elevator_command'],
        search_bounds=[lower,upper],
        search_bound_scope='Finite binary32 predictor states and retained command bound; no coverage proof',
        local_admission=bool(success),coverage_complete=False,
        correction=dict(termination=fit.termination,nfev=fit.nfev,
            evaluations=fit.evaluations,derivative_evaluations=fit.derivative_evaluations,
            residual_max=float(np.max(abs(value['residual']),initial=0.)),
            return_metric=fit.return_metric,closure_ratio=fit.closure_ratio,
            numerical_target_reached=fit.success),
        revision=REVISION)
