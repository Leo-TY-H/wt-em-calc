"""Bounded local correction of a declared stationary branch.

The evaluator owns equations and admission. This module only proposes numerical
coordinates: neither optimizer termination nor a small residual proves a root,
physical admission, branch completeness, or absence. Every proposed coordinate
is evaluated with the selected model; projection never substitutes a residual.
"""
from types import SimpleNamespace
import math

import numpy as np
from scipy.optimize import lsq_linear

from em_cancellation import check as check_cancel
from em_trim_numerics import DerivativeUnavailable, UnevaluableTrial, jacobian
from em_trim_work import check_deadline

# Try both sides and several lengths before using a cross-branch secant as a
# predictor. Such a secant is never a derivative or coverage certificate.
FACTORS = (1., .5, 1.5, 2., .25, -.5, -1., -1.5, -2., .1,
           3., 4., 5., -.1, .05, -.05, .01, -.01)
NUMERICAL_TARGET = 1e-7
# Production stops at the declared componentwise equation gates. Diagnostics
# can still request a stricter raw target, including exact zero.
PRODUCTION_NUMERICAL_TARGET = None
MERIT_WINDOW = 5
SUFFICIENT_DECREASE = 1e-4
MODEL_AGREEMENT = .1
MERIT_ROUNDOFF = 32. * float(np.finfo(float).eps)
PROGRESS_WINDOW = 6
PROGRESS_FRACTION = .001


class Corrector:
    def __init__(self, evaluate, bounds, steps, *, branch=lambda v: (),
                 project=None, scale=None, step_limit=None, residual_scale=None, dependencies=None):
        self.evaluate, self.branch, self.project = evaluate, branch, project
        self.lower, self.upper = [np.asarray(v, dtype=float) for v in bounds]
        self.steps = np.asarray(steps, dtype=float)
        self.scale = (np.ones(len(self.steps)) if scale is None
                      else np.asarray(scale, dtype=float))
        self.step_limit = (self.scale.copy() if step_limit is None
                           else np.asarray(step_limit, dtype=float))
        self.residual_scale = (None if residual_scale is None
                               else np.array(residual_scale, dtype=float, copy=True))
        self.dependencies = dependencies
        arrays = (self.lower, self.upper, self.steps, self.scale, self.step_limit)
        if (self.steps.ndim != 1 or any(v.shape != self.steps.shape for v in arrays)
                or any(not np.isfinite(v).all() for v in arrays)
                or np.any(self.lower >= self.upper)
                or any(np.any(v <= 0.) for v in (self.steps, self.scale, self.step_limit))):
            raise ValueError('Correction requires finite ordered bounds and positive coordinate scales')
        if self.residual_scale is not None and (self.residual_scale.ndim != 1
                or not np.isfinite(self.residual_scale).all()
                or np.any(self.residual_scale <= 0.)):
            raise ValueError('Residual scales must be a finite positive vector')
        self.evaluations = 0

    def _weighted(self, residual):
        residual = np.asarray(residual, dtype=float)
        if self.residual_scale is None:return residual
        if residual.shape != self.residual_scale.shape:
            raise ValueError('Residual scales must match the equation vector')
        return residual / self.residual_scale

    def sample(self, x):
        check_cancel()
        check_deadline()
        self.evaluations += 1
        value = self.evaluate(x)
        if not np.isfinite(value['residual']).all():
            raise UnevaluableTrial('Nonfinite stationary residual; correction unresolved')
        return value

    def inside(self, x):
        return bool(np.all(x >= self.lower) and np.all(x <= self.upper))

    def _matrix(self, x, base, evaluate, steps, *, numerical_target=NUMERICAL_TARGET):
        magnitude = np.abs(base['residual'])
        required = (np.zeros(magnitude.shape, dtype=bool) if numerical_target is None else
                    magnitude > 0. if numerical_target == 0. else magnitude >= numerical_target)
        if self.residual_scale is not None:
            required = required | (np.abs(self._weighted(base['residual'])) > 1.)
        return jacobian(x, base, evaluate, self.branch, steps, FACTORS,
            inside=self.inside, actual_step=True,
            prefer_informative=True, fallback_steps=self.steps, required_rows=required,
            dependencies=self.dependencies,
            project=(lambda q: self.project(q, base)) if self.project else None)

    def jacobian(self, x):
        x = np.asarray(x, dtype=float)
        return self._matrix(x, self.sample(x), self.sample, self.steps)

    def _direction(self, matrix, residual, x, radius, *, tolerance=1e-10):
        """Newton first; solve the constrained model when a bound is active.

        Coordinate scales condition columns; declared equation scales condition
        rows and define the merit used by globalization. Returned residuals and
        Jacobians retain their original units.
        The trust box limits physical steps independently of those scales.
        A constrained least-squares direction can move tangentially along an
        active physical or trust bound. Radially shrinking the Newton vector
        would unnecessarily suppress movement in the other coordinates.
        """
        cap = radius * self.step_limit
        lo = np.maximum(self.lower - x, -cap)
        hi = np.minimum(self.upper - x, cap)
        residual = self._weighted(residual)
        if self.residual_scale is not None:
            matrix = matrix / self.residual_scale[:, None]
        delta = self.scale * np.linalg.lstsq(matrix * self.scale, -residual, rcond=None)[0]
        if np.any(delta < lo) or np.any(delta > hi):
            free = hi > lo
            delta = lo.copy()
            if np.any(free):
                fit = lsq_linear(matrix[:, free] * self.scale[free],
                    -residual - matrix[:, ~free] @ delta[~free],
                    bounds=(lo[free] / self.scale[free], hi[free] / self.scale[free]),
                    method='bvls', tol=tolerance, max_iter=20)
                delta[free] = fit.x * self.scale[free]
        return delta

    def solve(self, initial, *, max_nfev=120, max_evaluations=None,
              max_iterations=None, numerical_target=NUMERICAL_TARGET, trace=None,
              reuse_jacobian=False, predictor=None):
        """Use one Newton-first, projected trust-box correction trajectory.

        Significant weighted native norm improvement with model agreement of
        at least 0.1 is tried first. If every damped Newton trial fails that
        test, compare measured progress from current derivative probes and
        coordinate-model proposals together. If none supplies at least 0.1 of
        the coupled trial's predicted progress, that trial may cross a rounded merit barrier
        only when its norm is strictly below the maximum of the last five
        accepted norms minus 1e-4 times its positive predicted decrease. Neutral
        moves and revisits to those five states are rejected. The finite window
        expires older high references. When equation scales are supplied, the
        returned state minimizes (maximum absolute scaled residual, scaled L2)
        over all samples; otherwise it minimizes the original L2 norm. This
        retention rule cannot admit a physical state or relax the raw target.
        Numerical closure also requires every explicitly scaled row to be at
        or below its declared scale, which may be stricter than the raw target.
        ``numerical_target`` defaults to 1e-7; zero requests exact raw objective
        zero, for example when the evaluator supplies constraint violations.
        None uses only the supplied componentwise residual scales. Such
        production correction stops unresolved after six linearizations without
        a 0.1 percent improvement in its best scaled closure ratio. It need not
        chase roundoff on an unreachable trial. Explicit diagnostic targets
        retain their requested precision and work budget.
        Optional Jacobian reuse applies a scaled Broyden update after accepted same-branch
        steps. A poor prediction refreshes the derivative before robust recovery.
        An incoming predictor is a numerical hint, never an evaluated state.

        Bad predictions contract the trust box to the actually attempted step
        and adapt local derivative lengths independently. Nominal source-aware
        stencils remain available when local differences round to zero in an
        unresolved equation, even if another equation changes smoothly. Only
        coordinates with representable movement justify adapting their stencil;
        every accepted displacement, including a damped Newton step, supplies
        that evidence. When all coupled trials fail, their smallest evaluated
        projected displacement refreshes participating stencils after the complete
        current-box comparison, even if a coordinate proposal improves. Such
        progress does not validate the failed coupled model. Stencil refresh
        alone does not contract the trust box, and a one-coordinate improvement
        preserves the other lengths. A local
        complete coordinate poll supplies proposals when the coupled model
        stalls. Its bounded one-coordinate least-squares steps use the current
        weighted model, independently of derivative lengths. Unavailable or
        stationary columns use the existing two-sided stencil proposals. All
        projected proposals are tried in descending model-predicted decrease;
        ties and uninformative proposals preserve axis/sign order. Prediction
        only orders work within the existing budget; actual merit decides.
        All coordinates compete on evaluated merit before choosing a point; a tiny
        improvement in a derivative probe cannot exclude the coordinate model.
        A significantly improved current derivative
        candidate competes with an improving model trial before choosing the
        next linearization point; older best states and rejected Newton trials
        cannot bypass the acceptance policy. Empirical progress reconciles the trust box with
        its successful displacement. Norm changes within a conservative
        binary64 rounding allowance (32 eps times the compared norm) do not
        count as progress, and every iteration checks representable movement.
        None of these proposals certifies smoothness, coverage or feasibility.

        ``max_nfev`` counts the initial sample and trial/poll evaluations.
        Derivative probes have a separate explicit total-evaluation ceiling;
        its default grants one nominal stencil allowance per objective sample.
        Informative-stencil refinements consume that same total allowance.
        ``max_iterations`` defaults to ``max_nfev`` and bounds linearizations,
        including rounds that only promote an existing derivative sample.
        Global cancellation/work allowances apply to every actual evaluation.
        Stagnation and exhausted allowances return the best sampled candidate
        with success=False, even if a local optimizer would call them success.
        ``trace``, when supplied, receives serializable dictionaries. No trace
        payloads or trajectory collection are created when it is absent.
        """
        if isinstance(max_nfev, bool) or int(max_nfev) != max_nfev or max_nfev < 1:
            raise ValueError('max_nfev must be a positive integer')
        max_nfev = int(max_nfev)
        if max_iterations is None:max_iterations = max_nfev
        if (isinstance(max_iterations, bool) or int(max_iterations) != max_iterations
                or max_iterations < 1):
            raise ValueError('max_iterations must be a positive integer')
        max_iterations = int(max_iterations)
        if max_evaluations is None:
            max_evaluations = max_nfev * (1 + len(self.steps) * len(FACTORS))
        if (isinstance(max_evaluations, bool) or int(max_evaluations) != max_evaluations
                or max_evaluations < 1):
            raise ValueError('max_evaluations must be a positive integer')
        if trace is not None and not callable(trace):
            raise TypeError('trace must be callable')
        row_target = numerical_target is None
        if row_target and self.residual_scale is None:
            raise ValueError('Componentwise stopping requires residual scales')
        if not row_target and (isinstance(numerical_target, (bool, np.bool_, str, bytes, complex, np.complexfloating)) or
                not np.isscalar(numerical_target)):
            raise ValueError('numerical_target must be a finite nonnegative number')
        try:
            if not row_target:
                if numerical_target < 0.:raise ValueError()
                converted_target = float(numerical_target)
                if converted_target == 0. and numerical_target != 0.:raise ValueError()
                numerical_target = converted_target
        except (TypeError, ValueError, OverflowError):
            raise ValueError('numerical_target must be a finite nonnegative number') from None
        if not row_target and (not math.isfinite(numerical_target) or numerical_target < 0.):
            raise ValueError('numerical_target must be a finite nonnegative number')
        x = np.asarray(initial, dtype=float)
        if x.shape != self.steps.shape or not np.isfinite(x).all():
            raise ValueError('Initial correction coordinates must be finite and match the bounds')
        x = np.clip(x, self.lower, self.upper)
        start_evaluations = self.evaluations
        used = iterations = contractions = polls = accepted = unavailable = 0
        radius = 1.
        best_x = best_value = None
        best_norm = float('inf')
        return_x = return_value = return_key = None
        matrix = matrix_x = None
        matrix_value = None
        matrix_exact = False
        jacobian_reuses = jacobian_refreshes = 0
        stencil = self.steps.copy()
        recent = []
        nonmonotone_steps = 0
        progress_ratio, progress_iteration = math.inf, 0
        message, termination = 'Correction budget exhausted', 'objective_budget'

        class Closed(Exception):
            pass

        class Budget(Exception):
            pass

        def emit(event, **details):
            trace(dict(event=event, iteration=iterations, nfev=used,
                evaluations=self.evaluations-start_evaluations,
                radius=float(radius), **details))

        def sample(q, *, objective=False, kind='derivative'):
            nonlocal used, best_x, best_value, best_norm, unavailable
            nonlocal return_x, return_value, return_key
            if self.evaluations - start_evaluations >= max_evaluations:
                raise Budget('evaluation_budget')
            if objective:
                if used >= max_nfev:raise Budget('objective_budget')
                used += 1
            try:value = self.sample(q)
            except UnevaluableTrial as error:
                unavailable += 1
                if trace is not None:
                    emit('unavailable', candidate_kind=kind, x=q.tolist(), reason=str(error))
                raise
            residual = value['residual']
            weighted = self._weighted(residual)
            norm = math.hypot(*weighted)
            improved = best_x is None or norm < best_norm
            if improved:
                best_x, best_value, best_norm = q.copy(), value, norm
            key = ((float(np.max(abs(weighted), initial=0.)), norm)
                   if self.residual_scale is not None else (norm,))
            retained = return_x is None or key < return_key
            if retained:
                return_x, return_value, return_key = q.copy(), value, key
            if trace is not None:
                emit('sample', candidate_kind=kind, x=q.tolist(),
                    residual=np.asarray(residual).tolist(), norm=norm,
                    raw_norm=math.hypot(*residual), best=improved, retained=retained)
            raw_max = np.max(abs(residual), initial=0.)
            target_met = (True if row_target else
                          raw_max == 0. if numerical_target == 0. else raw_max < numerical_target)
            if (target_met
                    and (self.residual_scale is None or key[0] <= 1.)):
                # Include derivative probes: they are genuine evaluated states,
                # and a later model step must not erase a numerically closed one.
                best_x, best_value, best_norm = q.copy(), value, norm
                return_x, return_value, return_key = q.copy(), value, key
                raise Closed()
            return value

        def proposal(q, base):
            nonlocal unavailable
            q = np.clip(q, self.lower, self.upper)
            try:
                if self.project is not None:q = np.asarray(self.project(q, base), dtype=float)
            except UnevaluableTrial as error:
                unavailable += 1
                if trace is not None:
                    emit('unavailable', candidate_kind='projection', x=q.tolist(), reason=str(error))
                return None
            if not self.inside(q) or not np.isfinite(q).all() or np.array_equal(q, x):
                return None
            return q

        def remember(q, norm):
            recent.append((tuple(q), norm))
            if len(recent)>MERIT_WINDOW:del recent[0]

        def merit(value):
            return math.hypot(*self._weighted(value['residual']))

        def significant_decrease(before, after):
            # This is a norm-comparison rounding allowance, not an equation or
            # physical tolerance. A finite norm also improves an overflowed one.
            return bool(after < before and (not math.isfinite(before)
                or before-after > MERIT_ROUNDOFF*max(before, after)))

        def representable_step():
            floor = 8. * np.spacing(np.maximum(1., abs(x)))
            return bool(np.any(radius * self.step_limit > floor))

        def adapt(displacement, *, contract):
            nonlocal radius, stencil, contractions
            floor = 8. * np.spacing(np.maximum(1., abs(x)))
            # An unsuccessful tiny Newton move says nothing about the much
            # larger original trust box. Contract at the scale actually tried.
            size = float(np.max(abs(displacement)/self.step_limit, initial=0.))
            if contract:
                radius = .5 * min(radius, size if size>0. else radius)
                contractions += 1
            if size>0.:
                # A one-axis empirical promotion gives no new length evidence
                # for the other axes. Preserve their source sensitivity instead
                # of collapsing every untouched stencil to its rounding floor.
                moved = abs(displacement) >= floor
                stencil[moved] = np.minimum(self.steps[moved],
                    np.maximum(.25*abs(displacement[moved]), floor[moved]))

        def accept(q, trial, kind, *, ratio=None, predicted=None,
                   reference=None, relaxed=False, empirical=False):
            nonlocal x, value, accepted, nonmonotone_steps, radius
            nonlocal matrix, matrix_x, matrix_value, matrix_exact
            displacement=q-x
            if reuse_jacobian and matrix is not None:
                if (kind == 'newton' and not relaxed and not empirical
                        and self.branch(value) == self.branch(trial)):
                    scaled = displacement / self.scale
                    length = float(scaled @ scaled)
                    if length > 0.:
                        change = trial['residual']-value['residual']-matrix@displacement
                        matrix = matrix + np.outer(change, scaled / length) / self.scale
                        matrix_x, matrix_value, matrix_exact = q.copy(), trial, False
                else:
                    matrix = matrix_x = matrix_value = None
                    matrix_exact = False
            adapt(displacement, contract=False)
            if empirical:
                size=float(np.max(abs(displacement)/self.step_limit, initial=0.))
                radius=min(1., max(radius, size))
            x, value = q.copy(), trial
            accepted += 1
            nonmonotone_steps += int(relaxed)
            remember(x, merit(value))
            if trace is not None:
                emit('accepted', candidate_kind=kind, x=x.tolist(),
                    residual=np.asarray(value['residual']).tolist(), ratio=ratio,
                    predicted_decrease=predicted, merit_reference=reference,
                    nonmonotone=relaxed)

        return_metric = 'scaled_linf_then_l2' if self.residual_scale is not None else 'raw_l2'
        if trace is not None:
            emit('configuration', residual_scale=(None if self.residual_scale is None
                else self.residual_scale.tolist()), merit_metric='scaled_l2',
                return_metric=return_metric, numerical_target=numerical_target,
                minimum_model_agreement=MODEL_AGREEMENT,
                merit_roundoff_multiplier=32.)
        try:
            value = sample(x, objective=True, kind='initial')
            remember(x, merit(value))
            if reuse_jacobian and predictor is not None:
                hint = np.asarray(predictor['matrix'], dtype=float)
                anchor = np.asarray(predictor['x'], dtype=float)
                if (hint.shape == (len(value['residual']), len(x)) and anchor.shape == x.shape
                        and np.isfinite(hint).all() and np.isfinite(anchor).all()
                        and np.max(abs(anchor-x)/self.step_limit, initial=0.) <= 1.
                        and predictor['branch'] == self.branch(value)):
                    matrix, matrix_x, matrix_value = hint.copy(), x.copy(), value
            while used < max_nfev:
                check_cancel()
                check_deadline()
                if row_target:
                    if return_key[0] < progress_ratio * (1. - PROGRESS_FRACTION):
                        progress_ratio, progress_iteration = return_key[0], iterations
                    elif iterations - progress_iteration >= PROGRESS_WINDOW:
                        message = 'No useful componentwise progress; correction unresolved'
                        termination = 'stagnation'
                        break
                if not representable_step():
                    message = 'No representable local trust step; correction unresolved'
                    termination = 'stagnation'
                    break
                if iterations >= max_iterations:raise Budget('iteration_budget')
                iterations += 1
                residual = value['residual']
                norm = merit(value)
                reference = max(v[1] for v in recent)
                stencil = np.maximum(stencil, 8. * np.spacing(np.maximum(1., abs(x))))
                if trace is not None:
                    emit('iteration', x=x.tolist(), residual=np.asarray(residual).tolist(),
                        stencil=stencil.tolist(), merit_reference=reference)
                sampled_candidate = None
                def probe(q):
                    nonlocal sampled_candidate
                    trial=sample(q)
                    trial_norm=merit(trial)
                    if sampled_candidate is None or trial_norm<sampled_candidate[2]:
                        sampled_candidate=(q.copy(),trial,trial_norm)
                    return trial
                try:
                    reused = (reuse_jacobian and matrix is not None and matrix_x is not None
                              and np.array_equal(matrix_x, x) and np.isfinite(matrix).all())
                    if reused:
                        jacobian_reuses += 1
                    else:
                        matrix = self._matrix(x, value, probe, stencil,
                                              numerical_target=numerical_target)
                        matrix_x, matrix_value, matrix_exact = x.copy(), value, True
                        jacobian_refreshes += 1
                    delta = self._direction(matrix, residual, x, radius,
                        tolerance=1e-5 if row_target else 1e-10)
                except (DerivativeUnavailable, np.linalg.LinAlgError):
                    matrix = matrix_x = matrix_value = None
                    matrix_exact = False
                    delta = np.zeros_like(x)
                if trace is not None:
                    emit('model', x=x.tolist(), direction=delta.tolist(),
                        matrix_available=matrix is not None, stencil=stencil.tolist(),
                        jacobian=None if matrix is None else matrix.tolist())

                moved = False
                tried = set()
                relaxed = None
                failed_displacement = None
                failed_size = math.inf
                for factor in (() if matrix is None else (1., .5, .25, .1)):
                    q = proposal(x + factor * delta, value)
                    if q is None or tuple(q) in tried:continue
                    tried.add(tuple(q))
                    try:trial = sample(q, objective=True, kind='newton')
                    except UnevaluableTrial:continue
                    trial_norm = merit(trial)
                    predicted = norm - math.hypot(*self._weighted(residual + matrix @ (q - x)))
                    ratio = (norm - trial_norm) / predicted if predicted > 0. else 0.
                    significant = significant_decrease(norm, trial_norm)
                    improves = significant and ratio >= MODEL_AGREEMENT
                    crossing = (trial_norm > norm and predicted > 0.
                        and significant_decrease(reference, trial_norm)
                        and trial_norm < reference-SUFFICIENT_DECREASE*predicted
                        and not any(tuple(q)==p for p,_ in recent))
                    if trace is not None:
                        emit('trial', candidate_kind='newton', x=q.tolist(),
                            residual=np.asarray(trial['residual']).tolist(), factor=factor,
                            predicted_decrease=predicted, ratio=ratio,
                            merit_reference=reference, improves=improves,
                            significant_decrease=significant,
                            nonmonotone_eligible=crossing)
                    if not improves:
                        # Only a completed native evaluation supplies model
                        # disagreement evidence. Projection may change both
                        # the size and axes of the attempted displacement.
                        displacement=q-x
                        size=float(np.max(abs(displacement)/self.step_limit, initial=0.))
                        if 0. < size < failed_size:
                            failed_displacement,failed_size=displacement.copy(),size
                        if relaxed is None and crossing:relaxed=(q,trial,predicted,ratio)
                        continue
                    if sampled_candidate is not None and significant_decrease(trial_norm, sampled_candidate[2]):
                        # This sample already used the original evaluator. A
                        # merely improving Newton trial must not delay a much
                        # better native point until a later model failure.
                        accept(sampled_candidate[0],sampled_candidate[1],
                               'sampled_best',reference=reference,empirical=True)
                        moved = True
                        break
                    if ratio > .75 and factor == 1.:
                        radius = min(1., 2. * radius)
                    accept(q, trial, 'newton', ratio=ratio, predicted=predicted,
                           reference=reference)
                    moved = True
                    break
                if moved:continue

                if reuse_jacobian and reused:
                    # A stale linear model must not trigger coordinate polls or
                    # alter the trust box. Refresh it at this same actual state.
                    matrix = matrix_x = matrix_value = None
                    matrix_exact = False
                    continue

                sampled_improves = (sampled_candidate is not None
                    and significant_decrease(norm, sampled_candidate[2]))

                # A failed coupled direction does not invalidate every model
                # direction in this trust box. Minimize the weighted linear
                # least-squares model on each coordinate before contracting it
                # or choosing a nonmonotone step. A stencil establishes a
                # column; its length is not an optimization step length.
                polls += 1
                poll_candidate = None
                proposals = []
                weighted = self._weighted(residual)
                weighted_matrix = (None if matrix is None else matrix
                    if self.residual_scale is None else matrix/self.residual_scale[:,None])
                for axis, step in enumerate(np.minimum(radius*self.step_limit,stencil)):
                    length = None
                    if weighted_matrix is not None:
                        column = weighted_matrix[:,axis]
                        column_norm = math.hypot(*column)
                        if 0. < column_norm < math.inf:
                            # Normalization avoids squaring the column norm;
                            # this is -J_j.T r / (J_j.T J_j).
                            candidate = -float((column/column_norm) @ weighted)/column_norm
                            if not math.isnan(candidate):
                                cap=radius*self.step_limit[axis]
                                length=float(np.clip(candidate,
                                    max(self.lower[axis]-x[axis],-cap),
                                    min(self.upper[axis]-x[axis],cap)))
                    modeled = length is not None and length != 0.
                    for displacement in ((length,) if modeled else (step,-step)):
                        q=x.copy();q[axis]+=displacement
                        q=proposal(q,value)
                        if q is None or tuple(q) in tried:continue
                        tried.add(tuple(q))
                        kind='coordinate_model' if modeled else 'poll'
                        predicted=(norm-math.hypot(*self._weighted(residual+matrix@(q-x)))
                                   if modeled else None)
                        proposals.append((q,axis,kind,modeled,predicted))
                # Projection can move more than the named coordinate. Rank the
                # complete projected model, not the unprojected scalar optimum.
                # Stable sorting preserves axis/sign order for ties; absent or
                # undefined predictions rank with zero rather than suppressing
                # empirical proposals. Every sample still passes native merit.
                proposals.sort(key=lambda item:(-item[4]
                    if item[4] is not None and not math.isnan(item[4]) else 0.))
                for q,axis,kind,modeled,predicted in proposals:
                    try:trial=sample(q,objective=True,kind=kind)
                    except UnevaluableTrial:continue
                    trial_norm=merit(trial)
                    ratio=((norm-trial_norm)/predicted if predicted is not None and predicted>0.
                            else 0. if modeled else None)
                    improves=(significant_decrease(norm,trial_norm)
                               and (not modeled or ratio>=MODEL_AGREEMENT))
                    if trace is not None:
                        emit('trial',candidate_kind=kind,x=q.tolist(),
                             residual=np.asarray(trial['residual']).tolist(),axis=axis,
                             predicted_decrease=predicted,ratio=ratio,
                             merit_reference=reference,improves=improves,
                             significant_decrease=significant_decrease(norm,trial_norm),
                             nonmonotone_eligible=False)
                    if improves and (poll_candidate is None or trial_norm<poll_candidate[2]):
                        poll_candidate=(q,trial,trial_norm,kind,ratio,predicted)
                # All usable progress at this linearization competes by the
                # same measured norm, irrespective of how it was proposed.
                # An unevaluated coordinate model cannot justify a decrease;
                # an evaluated one must not be bypassed just because its
                # displacement differs from a derivative stencil.
                current = ((*sampled_candidate,'sampled_best',None,None)
                           if sampled_improves else None)
                if poll_candidate is not None and (current is None
                        or significant_decrease(current[2],poll_candidate[2])):
                    current=poll_candidate
                preferred = (current is not None and (relaxed is None
                    or norm-current[2] >= MODEL_AGREEMENT*relaxed[2]))
                if trace is not None:
                    emit('current_progress',candidate_kind=None if current is None else current[3],
                        measured_decrease=0. if current is None else norm-current[2],
                        coupled_predicted_decrease=None if relaxed is None else relaxed[2],
                        preferred=preferred)
                if relaxed is not None and not preferred:
                    q,trial,predicted,ratio=relaxed
                    # Keep a barrier crossing bounded by its actual movement;
                    # do not interpret an increase as agreement with the model.
                    radius=min(radius,float(np.max(abs(q-x)/self.step_limit)))
                    accept(q,trial,'newton',ratio=ratio,predicted=predicted,
                           reference=reference,relaxed=True)
                    continue

                if poll_candidate is None:
                    adapt(delta, contract=True)
                    if trace is not None:
                        emit('contracted', candidate_kind='newton', direction=delta.tolist(),
                            stencil=stencil.tolist(), merit_reference=reference)

                if failed_displacement is not None:
                    # A useful coordinate step does not repair a misleading
                    # coupled stencil in another axis. Refresh from the
                    # smallest actually tested coupled step only after all
                    # same-box proposals have competed. Keep trust contraction
                    # and unobserved-axis preservation independent of this.
                    adapt(failed_displacement, contract=False)
                    if trace is not None:
                        emit('stencil_refreshed',candidate_kind='newton',
                            direction=failed_displacement.tolist(),
                            stencil=stencil.tolist(),merit_reference=reference)

                # Only this linearization's actual eligible proposals enter
                # current. Older best states and rejected low-agreement trials
                # remain return candidates without bypassing acceptance.
                if current is not None:
                    q,trial,_,kind,ratio,predicted=current
                    accept(q,trial,kind,ratio=ratio,predicted=predicted,
                           reference=reference,empirical=True)
                    continue
        except Closed:
            message, termination = 'Numerical correction target reached', 'numerical_target'
        except Budget as error:
            termination = error.args[0]
            message = ('Total correction evaluation budget exhausted' if termination == 'evaluation_budget'
                       else 'Correction iteration budget exhausted' if termination == 'iteration_budget'
                       else 'Correction budget exhausted')

        # Do not perform unbudgeted final probes or present a stale Jacobian as
        # belonging to the returned best point. The evaluator replays that point
        # for physical admission; consumers may request fresh diagnostics later.
        current_matrix = matrix_exact and matrix_x is not None and np.array_equal(matrix_x, return_x)
        residual = return_value['residual']
        diagnostic = matrix if current_matrix else np.full((len(residual), len(return_x)), np.nan)
        success = termination == 'numerical_target'
        closure_ratio = None if self.residual_scale is None else return_key[0]
        merit_norm = merit(return_value)
        if trace is not None:
            emit('termination', termination=termination, success=success,
                x=return_x.tolist(), residual=np.asarray(residual).tolist(),
                return_metric=return_metric, return_score=list(return_key),
                merit_norm=merit_norm, closure_ratio=closure_ratio,
                accepted_steps=accepted, nonmonotone_steps=nonmonotone_steps,
                contractions=contractions, coordinate_polls=polls,
                unevaluable_trials=unavailable)
        return SimpleNamespace(x=return_x, fun=residual, jac=diagnostic,
            value=return_value,
            predictor=(dict(matrix=matrix.copy(),x=matrix_x.copy(),branch=self.branch(matrix_value))
                if matrix is not None and matrix_x is not None and matrix_value is not None
                and np.isfinite(matrix).all() else None),
            jacobian_reuses=jacobian_reuses, jacobian_refreshes=jacobian_refreshes,
            numerical_target=numerical_target,
            residual_scale=None if self.residual_scale is None else self.residual_scale.copy(),
            return_metric=return_metric, return_score=return_key,
            closure_ratio=closure_ratio, merit_norm=merit_norm,
            jacobian_current=current_matrix, message=message, termination=termination,
            status=1 if success else 0, success=success, nfev=used,
            evaluations=self.evaluations-start_evaluations,
            derivative_evaluations=self.evaluations-start_evaluations-used,
            iterations=iterations, iteration_limit=max_iterations,
            accepted_steps=accepted, nonmonotone_steps=nonmonotone_steps,
            contractions=contractions,
            coordinate_polls=polls, unevaluable_trials=unavailable, trust_radius=radius)
