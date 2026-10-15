"""Deterministic coupled wake relaxation after the inexpensive iteration fails.

Integrate the residual field from the same seed, rather than selecting a root
from unrelated starting guesses. This is a numerical homotopy, not a simulation
of physical wake dynamics. Only the original algebraic residual certifies it.
"""
import numpy as np
import math
from scipy.integrate import RK45
from scipy.optimize import least_squares, brentq


def recover(evaluate, initial, q, flows, bounds, tolerance):
    if not flows:
        raise ValueError('Induced flow did not settle during governor branch search')
    trial = initial.copy()
    lower, upper = (np.asarray(b)[flows] for b in bounds)

    # Some windmilling wakes have only axial flow: the other wake equation is
    # already closed throughout the path. Follow that one-dimensional path
    # from the original seed to its first restoring crossing. Reject this
    # reduction immediately if another wake component becomes active.
    value=np.asarray(evaluate(trial))
    if np.isfinite(value).all():
        axis=int(np.argmax(abs(value[flows])))
        j=flows[axis];other=[k for k in flows if k!=j]
        if other and np.max(abs(value[other]),initial=0.)<=tolerance:
            def scalar(z):
                trial[:]=initial;trial[j]=z
                r=np.asarray(evaluate(trial))
                if not np.isfinite(r).all() or np.max(abs(r[other]),initial=0.)>tolerance:
                    raise ValueError('Wake leaves the scalar path')
                return float(r[j])
            left=float(initial[j]);low=float(value[j])
            end=float(bounds[1 if low>0. else 0][j])
            count=max(1,int(math.ceil(abs(end-left)/.05)))
            discontinuous=False
            try:
                for right in np.linspace(left,end,count+1)[1:]:
                    high=scalar(float(right))
                    if low*high<=0.:
                        root=brentq(scalar,min(left,right),max(left,right),xtol=1e-12,rtol=1e-12)
                        scalar(root);r=np.asarray(evaluate(trial))
                        if np.max(abs(r[flows]),initial=0.)<=tolerance:
                            q[:]=trial
                            return r
                        # A sign jump is not an algebraic root. Confirm that
                        # shrinking the bracket does not shrink its residual
                        # jump, then retain an unresolved branch immediately.
                        h=max(1e-10,abs(root)*1e-10)
                        outer=(scalar(root-h),scalar(root+h))
                        inner=(scalar(root-h*.1),scalar(root+h*.1))
                        discontinuous=(outer[0]*outer[1]<0. and inner[0]*inner[1]<0.
                            and min(map(abs,inner))>tolerance
                            and abs(inner[1]-inner[0])>.9*abs(outer[1]-outer[0]))
                        break
                    left,low=float(right),high
            except (ValueError,RuntimeError):pass
            if discontinuous:
                raise ValueError('Discontinuous induced-flow crossing during governor branch search')
            trial[:]=initial

    # Slowly contracting wakes need more iterations, not a new trajectory.
    # Replay the cheap deterministic iteration before using the coupled method.
    # The ordinary 80-step success path remains untouched.
    previous_flow=[0.]*len(flows);previous_residual=[0.]*len(flows)
    previous_error=math.inf;relaxation=1.
    for iteration in range(256):
        value=np.asarray(evaluate(trial))
        if not np.isfinite(value).all():raise ValueError('Nonfinite governor branch sample')
        error=max((abs(value[j]) for j in flows),default=0.)
        if error<=tolerance:
            q[:]=trial
            return value
        if error>previous_error*1.05:relaxation=max(.125,relaxation*.5)
        previous_error=error
        for k,j in enumerate(flows):
            current=float(trial[j]);r=float(value[j]);update=relaxation*r
            if iteration:
                difference=r-previous_residual[k]
                if abs(difference)>1e-14:
                    secant=-r*(current-previous_flow[k])/difference
                    if abs(secant)<=2.*abs(update):update=secant
            trial[j]=min(bounds[1][j],max(bounds[0][j],current+update))
            previous_flow[k]=current;previous_residual[k]=r
    trial[:]=initial

    def residual(z):
        trial[flows] = np.clip(z, lower, upper)
        value = np.asarray(evaluate(trial))
        if not np.isfinite(value).all():
            raise ValueError('Nonfinite governor branch sample')
        return value

    last_polish=None
    def polish(z, value):
        nonlocal last_polish
        error=np.max(abs(value[flows]),initial=0.)
        if error > .02:return None
        # A stalled least-squares basin does not improve when retried at every
        # integration step. Retry after actual residual or state progress.
        if (last_polish is not None and error>.5*last_polish[1] and
                np.max(abs(z-last_polish[0]),initial=0.)<.25):return None
        last_polish=(np.array(z,copy=True),error)
        fit = least_squares(lambda v:residual(v)[flows],np.clip(z,lower,upper),
                            bounds=(lower,upper),max_nfev=12,
                            ftol=1e-12,xtol=1e-12,gtol=1e-12,diff_step=1e-5)
        candidate=np.where(fit.x-lower<1e-6,lower,np.where(upper-fit.x<1e-6,upper,fit.x))
        value = residual(candidate)
        if (np.max(abs(value[flows]),initial=0.) <= tolerance and
                np.max(abs(candidate-z),initial=0.) <= .05 and
                np.max(np.real(np.linalg.eigvals(fit.jac)),initial=-np.inf) < 0.):
            q[:] = trial
            return value
        return None

    # A local attracting root near the last iterate preserves its basin. Large
    # jumps or unstable roots must instead follow the residual field from seed.
    value = residual(q[flows])
    closed = polish(q[flows].copy(),value)
    if closed is not None:return closed

    # Adaptive coupled steps avoid the componentwise secant stagnation seen
    # around stalled-blade wake transitions. Bounded work and the evaluator's
    # cancellation/work accounting apply to every stage.
    integrator = RK45(lambda t, z: residual(z)[flows], 0., initial[flows], 256.,
                      rtol=1e-8, atol=tolerance*.05, max_step=2.)
    for _ in range(1024):
        value = residual(integrator.y)
        if np.max(abs(value[flows]), initial=0.) <= tolerance:
            q[:] = trial
            return value
        closed = polish(integrator.y,value)
        if closed is not None:return closed
        if integrator.status != 'running':break
        integrator.step()
    raise ValueError('Induced flow did not settle during governor branch search')
