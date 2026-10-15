"""Force/moment closure independent of aircraft reporting and solver history.

The inputs are numerical arrays and fixed mass/time parameters. Native frame
evaluation and certificate construction remain at the operating-point boundary.
"""
import math
from dataclasses import dataclass

import numpy as np
from body_dynamics import angular_acceleration
from component_assembly import f32


def turn_quaternion(alpha_deg, bank_deg, sideslip_deg=0.):
    """Attitude is independent of speed, damping and turn-rate closure."""
    alpha, bank = map(math.radians, (alpha_deg, bank_deg))
    quaternion = [f32(math.sin(bank*.5)*math.cos(alpha*.5)),
                  f32(-math.sin(bank*.5)*math.sin(alpha*.5)),
                  f32(math.cos(bank*.5)*math.sin(alpha*.5)),
                  f32(math.cos(bank*.5)*math.cos(alpha*.5))]
    if sideslip_deg:
        beta = math.radians(sideslip_deg)
        sx, cx = math.sin(bank*.5), math.cos(bank*.5)
        sy, cy = math.sin(beta*.5), math.cos(beta*.5)
        sz, cz = math.sin(alpha*.5), math.cos(alpha*.5)
        quaternion = list(map(f32, [sx*cy*cz+cx*sy*sz, cx*sy*cz-sx*cy*sz,
                                    cx*cy*sz+sx*sy*cz, cx*cy*cz-sx*sy*sz]))
    return quaternion


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
