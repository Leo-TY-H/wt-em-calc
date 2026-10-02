"""Force/moment closure independent of aircraft reporting and solver history.

The inputs are numerical arrays and fixed mass/time parameters. Native frame
evaluation and certificate construction remain at the operating-point boundary.
"""
import math
from dataclasses import dataclass

import numpy as np
from body_dynamics import angular_acceleration


@dataclass(frozen=True, slots=True)
class TrimParameters:
    mass: float
    weight: float
    inertia: tuple
    dt: float


def closure(parameters, speed, force, processed_omega, stored_moment,
            omega, forward, up, lateral, normal, side, turn_rate):
    acceleration = np.asarray(angular_acceleration(
        processed_omega, parameters.inertia, stored_moment))
    rate = (processed_omega + acceleration * parameters.dt - omega) / parameters.dt
    tangential = force.dot(forward) / parameters.mass
    lateral_acceleration = (speed / parameters.dt + tangential) * math.tan(turn_rate * parameters.dt)
    required = parameters.weight * up + parameters.mass * lateral_acceleration * lateral
    force_error = (force - required) / parameters.weight
    residual = np.array([force_error.dot(normal), force_error.dot(side), *(rate / .1)])
    return residual, rate
