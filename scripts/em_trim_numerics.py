import numpy as np
from em_trim_work import charge


class UnevaluableTrial(ValueError):
    """A declared trial leaves the evaluator's available branch/domain.

    Local correctors may reject an off-root proposal carrying this explicit
    classification. Programming errors, arbitrary arithmetic exceptions,
    cancellation, and work-limit exceptions remain unrelated and propagate.
    """


class DerivativeUnavailable(Exception):
    def __init__(self, axis):
        self.axis = axis
        super().__init__('No same-branch derivative for coordinate '+str(axis))


def jacobian(x, base, evaluate, branch, steps, factors, *, inside=None, project=None, event=None, actual_step=False, allow_secant=True, prefer_informative=False, fallback_steps=None, required_rows=None, dependencies=None):
    """Sample complete branch-aware columns, preserving rounded observability.

    ``required_rows`` marks equations still outside the caller's closure target.
    If a shorter adaptive stencil changes only other rows, compare one declared
    nominal stencil. Use that complete same-branch column only if it restores
    sensitivity in a missing required row; never combine rows from two probes.
    All actual probes share the existing work/evaluation allowances.
    """
    columns = []
    displacements = []
    event_columns = []
    key = branch(base)
    extra = event(base) if event is not None else None
    required = None if required_rows is None else np.asarray(required_rows, dtype=bool)
    if required is not None and required.shape != np.shape(base['residual']):
        raise ValueError('Required derivative rows must match the equation vector')
    if dependencies is not None:
        dependencies=np.asarray(dependencies,dtype=bool)
        if dependencies.shape != (len(base['residual']),len(x)):
            raise ValueError('Derivative dependencies must match equations and coordinates')
    for axis, h in enumerate(steps):
        matched = False
        last = None
        zero = None
        seen = set()
        nominal = None
        lengths = [factor*h for factor in factors]
        if fallback_steps is not None and fallback_steps[axis] > h:
            # The trust box controls accepted movement, not observability of a
            # rounded evaluator. A local stencil may be wholly inside a cell;
            # the declared nominal lengths can still supply a useful proposal.
            nominal = fallback_steps[axis]
            lengths.extend(factor*nominal for factor in factors)
        def probe(length, *, comparison=False):
            q = np.asarray(x).copy();q[axis] += length
            try:
                if project is not None:q = project(q)
            except UnevaluableTrial:
                return None
            if inside is not None and not inside(q):return None
            step = q[axis]-x[axis] if actual_step else length
            if not step:return None
            if prefer_informative:
                # Distinct nominal lengths can project to the same actual
                # coordinate. Charge/evaluate that coordinate only once.
                coordinate = tuple(q)
                if coordinate in seen:return None
                seen.add(coordinate)
            if comparison:charge('nominal_sensitivity_comparisons')
            charge('derivative_samples')
            try:value = evaluate(q)
            except UnevaluableTrial:return None
            column = (value['residual']-base['residual'])/step
            if not np.isfinite(column).all():return None
            candidate = (column, (event(value)-extra)/step if event is not None else None, (q-x)/step)
            return candidate, branch(value) == key

        nominal_prioritized = False
        for index, length in enumerate(lengths):
            sampled = probe(length)
            if sampled is None:continue
            candidate, same_branch = sampled
            column = candidate[0]
            informative = bool(np.any(column)) or (event is not None and bool(np.any(candidate[1])))
            if prefer_informative and not informative and nominal is not None and not nominal_prioritized:
                # A rounded-zero local sample gives no reason to spend all
                # eighteen local lengths before checking the source stencil.
                # Keep the remaining branch-aware lengths if this one fails.
                lengths.insert(index+1, factors[0]*nominal)
                nominal_prioritized = True
            if not prefer_informative or informative or last is None:last = candidate
            if not same_branch:continue
            if prefer_informative and not informative:
                zero = candidate
                continue
            if (prefer_informative and nominal is not None and abs(length) < nominal
                    and required is not None):
                missing = required & (column == 0.)
                if dependencies is not None:
                    # Projection can move additional coordinates (e.g. bank
                    # on a native Mach face); include all of their dependencies.
                    missing = missing & np.any(dependencies[:,candidate[2] != 0.],axis=1)
                if np.any(missing):
                    # Smooth nuisance rows must not hide a rounded-zero row
                    # that still needs closure. One full nominal comparison is
                    # enough to establish restored sensitivity; an unrelated
                    # zero row cannot replace a valid local column.
                    comparison = probe(factors[0]*nominal, comparison=True)
                    if (comparison is not None and comparison[1]
                            and np.any(missing & (comparison[0][0] != 0.))):
                        candidate = comparison[0]
                        charge('nominal_sensitivity_restorations')
            columns.append(candidate[0])
            displacements.append(candidate[2])
            if event is not None:event_columns.append(candidate[1])
            matched = True
            break
        if not matched:
            # A zero within a rounded cell is not evidence that the coordinate
            # is irrelevant. Prefer an observed nonzero secant when permitted;
            # retain a genuine zero column if every available sample agrees.
            candidate = last if allow_secant and last is not None else zero
            if candidate is None:raise DerivativeUnavailable(axis)
            if candidate is not zero:charge('branch_secant_predictors')
            columns.append(candidate[0])
            displacements.append(candidate[2])
            if event is not None:event_columns.append(candidate[1])
    matrix = np.column_stack(columns)
    if event is not None:matrix = np.vstack((matrix, event_columns))
    if project is not None:
        # A same-Mach projection can move more than the perturbed coordinate.
        # Solve the complete stencil instead of treating it as axis aligned.
        directions=np.column_stack(displacements)
        try:matrix=np.linalg.solve(directions.T,matrix.T).T
        except np.linalg.LinAlgError:raise DerivativeUnavailable(-1)
    if not np.isfinite(matrix).all():raise DerivativeUnavailable(-1)
    return matrix
