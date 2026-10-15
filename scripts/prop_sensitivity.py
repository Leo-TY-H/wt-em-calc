"""Implicit propulsion sensitivities for local aircraft derivative proposals.

Use only to propose derivative samples. The accepted aircraft candidate must
still pass the ordinary complete propulsion and aircraft solve/certificate.
"""
import copy

import numpy as np
from scipy.linalg import lu_factor, lu_solve

from prop_equilibrium import solve, RESIDUAL_TOLERANCE


class SensitivityUnavailable(ValueError):
    pass


class PropulsionSensitivity:
    def __init__(self, properties, velocity, height, body_omega, cg, dt, nitro,
                 controls, torque_gyro=True, state=None, *, result=None):
        self.properties=properties
        self.fixed=(tuple(cg),dt,nitro,copy.deepcopy(controls),torque_gyro)
        self.condition=np.r_[velocity,body_omega,height].astype(float)
        if not np.isfinite(self.condition).all():
            raise SensitivityUnavailable('nonfinite flight condition')
        self.result=(result if result is not None else solve(properties,velocity,height,body_omega,
            *self.fixed,state=state,_retain_linearization=True))
        if not self.result['converged'] or not self.result['feasible'] or '_linearization' not in self.result:
            raise SensitivityUnavailable('no converged propulsion linearization')
        self.state=copy.deepcopy(self.result['state'])
        linearization=self.result['_linearization']
        if (not np.array_equal(linearization['condition'],self.condition) or
                linearization['fixed']!=self.fixed):
            raise SensitivityUnavailable('linearization does not match a supported flight condition')
        if self.result['overspeed_engines']:
            raise SensitivityUnavailable('unsupported overspeed propulsion state')
        self.z=linearization['coordinates'].copy()
        self.bounds=linearization['bounds']
        matrix=linearization['matrix']
        self.condition_number=float(np.linalg.cond(matrix))
        if not np.isfinite(matrix).all() or not np.isfinite(self.condition_number) or self.condition_number>1e8:
            raise SensitivityUnavailable('ill-conditioned propulsion linearization')
        self.factorization=lu_factor(matrix)
        base=self.problem(self.condition)
        self.residual,self.outputs,self.branch=base['sample_numeric'](self.z)
        self.frames=base['calls']()
        if (not np.isfinite(self.outputs).all() or not np.isfinite(self.residual).all() or
                np.max(abs(self.residual),initial=0.)>RESIDUAL_TOLERANCE):
            raise SensitivityUnavailable('retained propulsion state needs a full refresh')
        # G_x changes only the flight condition, keeping all propulsion unknowns
        # fixed. No perturbed condition runs a nested equilibrium solve.
        steps=np.array([.01,.01,.01,1e-4,1e-4,1e-4,.1])
        columns=[]
        for axis,h in enumerate(steps):
            samples=[]
            for sign in (1.,-1.):
                condition=self.condition.copy();condition[axis]+=sign*h
                problem=self.problem(condition)
                residual,_,branch=problem['sample_numeric'](self.z)
                self.frames+=problem['calls']()
                if branch!=self.branch or not np.isfinite(residual).all():
                    raise SensitivityUnavailable('branch transition in propulsion derivative')
                samples.append(residual)
            columns.append((samples[0]-samples[1])/(2*h))
        self.state_derivatives=lu_solve(self.factorization,-np.column_stack(columns))
        if not np.isfinite(self.state_derivatives).all():
            raise SensitivityUnavailable('nonfinite propulsion derivative')

    def problem(self, condition):
        return solve(self.properties,tuple(condition[:3]),float(condition[6]),tuple(condition[3:6]),
            *self.fixed,state=self.state,_prepare_only=True)

    def predict(self, velocity, height, body_omega):
        condition=np.r_[velocity,body_omega,height].astype(float)
        if not np.isfinite(condition).all():
            raise SensitivityUnavailable('nonfinite flight condition')
        delta=self.state_derivatives@(condition-self.condition)
        candidate=self.z+delta
        if np.max(abs(delta),initial=0.)>.02 or np.any(candidate<self.bounds[0]) or np.any(candidate>self.bounds[1]):
            raise SensitivityUnavailable('outside local propulsion neighborhood')
        problem=self.problem(condition)
        residual,outputs,branch=problem['sample_numeric'](candidate)
        self.frames+=problem['calls']()
        if branch!=self.branch or not np.isfinite(outputs).all() or not np.isfinite(residual).all():
            raise SensitivityUnavailable('branch transition or nonfinite propulsion frame')
        if np.max(abs(residual),initial=0.)>RESIDUAL_TOLERANCE:
            raise SensitivityUnavailable('propulsion prediction needs a full refresh')
        return dict(coordinates=candidate,outputs=outputs,residual=residual,branch=branch)
