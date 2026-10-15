import time
from em_trim_work import Work, WorkLimit, current, use


def mark_unresolved_lower_loads(column):
    """A numerical level-flight failure does not establish a physical floor."""
    bottom = column.get('lower_boundary')
    if not bottom or bottom['load_g'] <= 1.:return
    failed = [p for p in column.get('points', [])
              if 1. <= p['load_g'] < bottom['load_g'] and not p['valid']
              and (not (p.get('converged') or p.get('bounded_search_stationary'))
                   or any(r in ('propulsion did not settle','propulsion cycle unresolved')
                          for r in p.get('reasons', [])))]
    if not failed:return
    low = min(p['load_g'] for p in failed)
    unresolved = column.setdefault('unresolved_load_intervals', [])
    if not any(a <= low and b >= bottom['load_g'] for a,b in unresolved):
        unresolved.append([low, bottom['load_g']])
        unresolved.sort(key=lambda pair:pair[0])
    column.update(coverage_complete=False, lower_boundary_status='unresolved numerical boundary')
    bottom.pop('_checked_probe_lower_boundary', None)


def bounded_column(task, evaluate, *, certificate=False):
    from em_sampling import worker_solver
    parent = current()

    if parent is not None and getattr(parent, 'column', None) == task[:3]:return evaluate()
    solver = worker_solver(task[0], task[1])
    from numbers import Real
    from em_mach_events import discontinuous_column
    if isinstance(task[2],Real):
        excluded=discontinuous_column(solver,task[2])
        if excluded is not None:return None if certificate else excluded


    limits = getattr(solver, '_column_search_limits', {})
    work = Work(limits, parent=parent, collect=True)
    work.column = task[:3]
    started = time.monotonic()
    with use(work):
        try:column = evaluate()
        except WorkLimit as failure:
            owner=failure.owner
            while owner is not None and owner is not work:owner=owner.parent
            if owner is None:raise


            if certificate:return None
            points = sorted((dict(p, surface_sample=False) for p in work.points.values()
                             if p['speed_kmh'] == task[2]), key=lambda p:p['load_g'])


            column = dict(speed_kmh=task[2], points=points, boundary=None, lower_boundary=None,
                sustained=[], load_checks=[], boundary_bracket_g=None,
                boundary_reason='Numerical search allowance exhausted',
                boundary_status='unresolved numerical boundary',
                numerical_gap_brackets=[], interior_failures=[],
                unresolved_load_intervals=[(1., solver.config['max_load_g'] or 64.)],
                elapsed_s=time.monotonic()-started, mass=solver.mass, engine=solver.engine.summary,
                search_unresolved=failure.kind+' allowance exhausted', coverage_complete=False)
    if column is not None:
        if not certificate:mark_unresolved_lower_loads(column)
        column['search_work'] = dict(work.counts)
    return column
