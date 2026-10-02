"""Concurrent solves with a deterministic order for numerical decisions.

Only the issued prefix is accepted. Children may be submitted immediately after
their parent is accepted, without waiting for a whole refinement round.
"""
from collections import deque
from concurrent.futures import wait, FIRST_COMPLETED


class OrderedWork:
    def __init__(self):
        self.items = deque()

    def submit(self, context, future=None, value=None):
        self.items.append((context, future, value))

    def __bool__(self):
        return bool(self.items)

    def __len__(self):
        return len(self.items)

    def take(self, timeout=.25):
        if not self.items:
            return []
        first = self.items[0][1]
        if first is not None and not first.done():
            wait((first,), timeout=timeout, return_when=FIRST_COMPLETED)
        accepted = []
        while self.items:
            context, future, value = self.items[0]
            if future is not None and not future.done():
                break
            # Keep failed futures available to cancellation/cleanup callers.
            value = future.result() if future is not None else value
            self.items.popleft()
            accepted.append((context, value))
        return accepted

    def cancel(self):
        for _, future, _ in self.items:
            if future is not None:
                future.cancel()
