"""Stop numerical roots at an explicitly checked physical residual target."""
from scipy.optimize import brentq


def checked_root(function,low,high,*,residual_tolerance,accept=None,**options):
    class ClosedRoot(Exception):
        def __init__(self,x):self.x=x
    def evaluate(x):
        value=function(x)
        if abs(value)<=residual_tolerance and (accept is None or accept(x)):
            raise ClosedRoot(x)
        return value
    try:return brentq(evaluate,low,high,**options)
    except ClosedRoot as closed:return closed.x
