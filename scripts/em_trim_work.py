from collections import Counter
from contextlib import contextmanager
from contextvars import ContextVar
import time

import numpy as np

_current = ContextVar('trim_work', default=None)


class WorkLimit(Exception):
    def __init__(self, owner, kind):
        self.owner, self.kind = owner, kind
        super().__init__('Unresolved numerical search: '+kind+' allowance exhausted')


class Work:
    def __init__(self, limits=None, parent=None, collect=False):
        self.limits = dict(limits or {})
        self.parent = parent
        self.counts = Counter()
        self.best = {}
        self.progress = {}
        # Requests leave run_generator's stack before this Work is discarded.
        # Retain their identities for as long as best/progress retain their ids.
        self.request_owners = {}
        self.active = None
        self.events = []
        self.points = {} if collect else None
        self.deadline = time.monotonic()+self.limits.get('seconds', float('inf'))

    def check_deadline(self):
        owner = self
        while owner is not None:
            if time.monotonic() >= owner.deadline:raise WorkLimit(owner, 'elapsed time')
            owner = owner.parent

    def charge(self, kind):
        self.check_deadline()
        chain = []
        owner = self
        while owner is not None:
            if owner.counts[kind] >= owner.limits.get(kind, float('inf')):
                raise WorkLimit(owner, kind)
            chain.append(owner)
            owner = owner.parent
        for owner in chain:owner.counts[kind] += 1

    def observe(self, solver, speed, load, x, value, frozen=False):
        if frozen or self.active is None:return
        request = self.active
        if solver is not request.solver or load != request.load or speed != request.speed/3.6:return
        residual = value['residual']
        if not np.isfinite(residual).all():return
        propulsion = value.get('propulsion')


        rank = (bool(propulsion and not propulsion['converged']), float(np.linalg.norm(residual)))
        key = id(request)
        owner = self.request_owners.setdefault(key, request)
        if owner is not request:raise RuntimeError('Trim work request identity collision')
        if key not in self.best or rank < self.best[key][0]:
            self.best[key] = (rank, solver, list(x), value)
        allowance = self.limits.get('stagnant_aircraft_evaluations')
        if allowance is not None:
            if (not rank[0] and value['force_error_g'] <= 2e-4 and
                    np.max(abs(value['rate_residual']),initial=0.) <= 5e-5 and
                    value['history_error'] <= 2e-4):return
            count = self.counts['aircraft_evaluations']
            previous = self.progress.get(key)
            if (previous is None or rank[0] < previous[0][0] or
                    rank[0] == previous[0][0] and rank[1] < previous[0][1]*(1.-1e-3)):
                self.progress[key] = (rank,count)
            elif count-previous[1] >= allowance:
                raise WorkLimit(self,'stagnant aircraft evaluations')


def current():return _current.get()


@contextmanager
def use(work):
    token = _current.set(work)
    try:yield work
    finally:_current.reset(token)


def charge(kind):
    work = _current.get()
    if work is not None:work.charge(kind)


def observe(solver, speed, load, x, value, frozen=False):
    work = _current.get()
    if work is not None:work.observe(solver, speed, load, x, value, frozen)


def check_deadline():
    work = _current.get()
    if work is not None:work.check_deadline()


def record_point(point):
    work = _current.get()
    while work is not None:
        if work.points is not None:
            key = (point['speed_kmh'], point['load_g'], point['sideslip_attitude_deg'])
            old = work.points.get(key)
            rank = lambda p:(not p['valid'], p['force_error_g']+p['angular_error_rad_s2'])
            if old is None or rank(point) < rank(old):
                work.points[key] = {k:v for k,v in point.items() if k != '_detail'}
        work = work.parent
