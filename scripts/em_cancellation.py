"""Cooperative cancellation shared with this calculation's worker processes."""
_event=None


def initialize(event):
    global _event
    _event=event


def check():
    if _event is not None and _event.is_set():raise InterruptedError('Calculation cancelled')
