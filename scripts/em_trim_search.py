from dataclasses import dataclass

from em_cancellation import check as check_cancel
from em_trim_work import Work, WorkLimit, current, use
from em_trim_numerics import DerivativeUnavailable


@dataclass
class Request:
    solver: object
    speed: float
    load: float
    initial: object = None
    detailed: bool = False
    exhaustive: bool = True
    refine: bool = False
    quick: bool = False

    def attempt(self):
        return self.solver._solve_attempt(self.speed, self.load, self.initial,
            detailed=self.detailed, exhaustive=self.exhaustive, refine=self.refine, quick=self.quick)


def run_generator(generator, request=None):
    stack = [generator]
    requests = [request]
    work = current()
    answer = None
    try:
        while stack:
            check_cancel()
            if work is not None:work.active = requests[-1]
            try:
                request = stack[-1].send(answer)
            except StopIteration as completed:
                stack.pop()
                requests.pop()
                answer = completed.value
            else:
                if not isinstance(request, Request):
                    raise TypeError('A trim corrector must yield a recovery Request')
                if work is not None:work.charge('recovery_requests')
                stack.append(request.attempt())
                requests.append(request)
                answer = None
        return answer
    finally:
        for pending in reversed(stack):pending.close()


def solve(solver, speed, load, initial=None, detailed=False, exhaustive=True, refine=False, quick=False):
    request = Request(solver, speed, load, initial, detailed, exhaustive, refine, quick)


    # Stop repeatedly differentiating an unchanging rejected iterate. Retain
    # the best full-physics state and explicitly report an incomplete search;
    # this allowance never certifies a physical limit or an infeasible region.
    defaults = ({'stagnant_aircraft_evaluations':192}
                if getattr(solver,'is_prop',False) and solver.engine.quasi_steady else {})
    limits = getattr(solver, '_search_limits', defaults)
    work = Work(limits, parent=current())
    with use(work):
        try:
            point = run_generator(request.attempt(), request)
        except (WorkLimit, DerivativeUnavailable) as exhausted:
            if isinstance(exhausted, WorkLimit) and exhausted.owner is not work:raise
            retained = work.best.get(id(request))
            if retained is None:raise
            _, owner, x, value = retained


            with use(work.parent):
                point = owner.certify(speed, load, x, value, detailed=detailed,
                                      evaluations=work.counts['aircraft_evaluations'])
            point['search_unresolved'] = (exhausted.kind+' allowance exhausted'
                if isinstance(exhausted, WorkLimit) else str(exhausted))
    point['search_work'] = dict(work.counts)
    return point
