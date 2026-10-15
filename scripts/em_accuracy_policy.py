"""User-approved operating-point SEP error allowance.

This bounds numerical SEP error against an identified intended equilibrium.
It is separate from equation closure, physical admission and chart interpolation.
The approved 0.05 m/s allowance is exact and independent of chart interpolation
tolerance. Nothing in this policy establishes root coverage.
"""
from fractions import Fraction
from numbers import Real


REVISION = 'sep-budget-0.05-v2'
ALLOWANCE = Fraction(1, 20)
FORMULA = '0.05 m/s'


def comparison_tolerance(config):
    """Chart interpolation budget plus both evaluated samples' allowances."""
    tolerance=config['sep_tolerance_mps']
    return tolerance+(2.*float(ALLOWANCE) if config.get('trim_solver_mode')=='stationary' else 0.)


def sep_error_budget(chart_tolerance):
    """Return the exact approved budget for a positive finite chart tolerance."""
    if isinstance(chart_tolerance, bool) or not isinstance(chart_tolerance, Real):
        raise ValueError('Chart SEP tolerance must be a positive finite real number')
    try:
        tolerance = Fraction(chart_tolerance)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError('Chart SEP tolerance must be a positive finite real number') from error
    if tolerance <= 0:
        raise ValueError('Chart SEP tolerance must be a positive finite real number')
    return ALLOWANCE
