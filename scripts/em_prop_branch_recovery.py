"""Bounded pitch/wake seed discovery, never governor selection or absence.

Each probe freezes one aircraft/request condition. Original joint equations
must correct every nominated seed. Multiple wake branches and unsampled pitch
intervals remain unproved even when a candidate is recovered.
"""
import itertools
import math

import numpy as np
from scipy.optimize import brentq, minimize_scalar

from em_cancellation import check as check_cancel
from em_trim_work import charge
from prop_equilibrium import RESIDUAL_TOLERANCE, pitch_limits
from prop_governor_evaluator import prepare as prepare_evaluators

REVISION = 'bounded-pitch-wake-discovery-v2'
PITCH_SAMPLES = 65
TOTAL_SAMPLES = 260
CORRECTIONS = 8
WAKE_NFEV = 24
CORRECTION_NFEV = 40


class ProbeUnavailable(ValueError):
    pass


class Budget:
    def __init__(self, samples=TOTAL_SAMPLES):
        self.remaining = samples
        self.corrections = CORRECTIONS

    def sample(self):
        check_cancel()
        if self.remaining <= 0:raise ProbeUnavailable('shared pitch probe budget')
        self.remaining -= 1
        charge('stationary_branch_probes')


def warnings(problem, fit):
    """Cheap local Schur-complement hints from the normal fit's Jacobian.

    Frozen-attitude derivatives are neither coupled fold certificates nor
    global governor classifications. No warning changes physical admission.
    """
    prep=problem.prepared(problem.attitude(fit.x))[0]
    matrix=np.asarray(fit.jac);result=[]
    p=problem.solver.engine.properties
    for col,path in enumerate(problem.paths,problem.prop_start):
        if path[0]!='pitch':continue
        if getattr(problem,'governor_saturation',False) and not 0.<=fit.x[col]<=1.:
            result.append(dict(shaft=path[1],governor_coordinate=float(fit.x[col]),
                warning='saturated governor; pitch-only slope is inapplicable'))
            continue
        shaft=path[1];indices={l['index'] for l in p['transmissions'][shaft]['propellers']}
        flows=[(j,q) for j,q in enumerate(problem.paths,problem.prop_start)
               if q[0]=='flow' and q[1] in indices]
        flowcols=[j for j,_ in flows]
        flowrows=[problem.prop_start+problem.rows.index(prep['paths'].index(q)) for _,q in flows]
        torque=problem.prop_start+problem.rows.index(prep['paths'].index(('shaft',shaft)))
        row=dict(shaft=shaft,pitch_fraction=float(fit.x[col]),net_torque_scaled=float(fit.fun[torque]),
                 scope='local frozen-attitude derivative; not a fold certificate')
        try:
            block=matrix[np.ix_(flowrows,flowcols)]
            condition=float(np.linalg.cond(block)) if flows else 1.
            row['wake_jacobian_condition']=condition if math.isfinite(condition) else None
            if condition>1e10:raise np.linalg.LinAlgError('ill-conditioned wake')
            slope=float(matrix[torque,col]-(matrix[torque,flowcols] @
                np.linalg.solve(block,matrix[flowrows,col]) if flows else 0.))
            row['net_torque_pitch_slope']=slope
            if abs(slope)<1e-12:row['warning']='flat torque curve'
            else:
                step=-row['net_torque_scaled']/slope
                row['predicted_pitch_correction']=step
                if not 0.<=fit.x[col]+step<=1.:row['warning']='linear prediction leaves pitch interval'
                elif abs(step)>.1:row['warning']='weak torque slope; large predicted pitch correction'
            if min(fit.x[col],1.-fit.x[col])<1e-6:row['pitch_endpoint']=True
        except np.linalg.LinAlgError:row['warning']='wake branch derivative unavailable'
        result.append(row)
    return result


def scan(evaluate, initial, bounds, torque_row, pitch_col, flows, *,
         budget=None, collapsed=False, reverse=False, use_cache=True):
    """Inspect one wake continuation with an immutable, deterministic seed.

    The finite mesh is discovery only. Even successful brackets are provisional:
    unobserved discontinuities/other wake roots prevent a coverage claim.
    """
    budget = budget or Budget()
    initial = np.asarray(initial, dtype=float).copy()
    cache, failures, probes, roots, extrema = {}, {}, [], [], []
    calls = 0
    count = 0
    row = dict(revision=REVISION, probes=probes, roots=roots, extrema=extrema,
               unresolved_intervals=[], coverage_complete=False)

    def at(fraction):
        nonlocal count, calls
        fraction = float(fraction)
        check_cancel()
        if use_cache and fraction in cache:return cache[fraction]
        if fraction in failures:raise ProbeUnavailable(failures[fraction])
        if count >= PITCH_SAMPLES:raise ProbeUnavailable('shaft pitch probe budget')
        budget.sample();count += 1
        q = initial.copy();q[pitch_col] = fraction

        def residual(z):
            nonlocal calls
            check_cancel();charge('stationary_branch_kernel_evaluations');calls += 1
            q[flows] = z
            value = evaluate(q)
            if not np.isfinite(value).all():raise ProbeUnavailable('nonfinite propeller probe')
            return value

        try:
            if flows:
                from em_stationary_corrector import Corrector
                fit = Corrector(lambda z:dict(residual=residual(z)[flows]),
                    (np.asarray(bounds[0])[flows], np.asarray(bounds[1])[flows]),
                    np.full(len(flows),1e-5), scale=np.ones(len(flows)),
                    step_limit=np.full(len(flows),.3),
                    residual_scale=np.full(len(flows),1e-6)).solve(initial[flows], max_nfev=WAKE_NFEV)
                value = residual(fit.x)
            else:value = residual([])
            error = float(max(abs(value[flows]), default=0.))
            if error > 1e-6:raise ProbeUnavailable('wake did not close')
            result = (float(value[torque_row]), q.copy())
            cache[fraction] = result
            probes.append(dict(pitch_fraction=fraction, net_torque_scaled=result[0],
                wake_residual=error, wake_coordinates=q[flows].tolist(),
                wake_identity='fixed trial seed; uniqueness unproved'))
            return result
        except (ValueError, ArithmeticError, np.linalg.LinAlgError) as error:
            failures[fraction] = str(error)
            raise ProbeUnavailable(str(error)) from error

    def nominate(f, bracket, kind):
        r, q = at(f)
        if abs(r) > RESIDUAL_TOLERANCE:return
        # Deduplicate numerical seeds, not distinct certified roots.
        if any(abs(root['pitch_fraction']-f)<1e-8 for root in roots):return
        roots.append(dict(pitch_fraction=float(f), bracket=list(bracket), kind=kind,
                          net_torque_scaled=r, coordinates=q.tolist()))

    def interval_error(lo, hi, error):
        row['unresolved_intervals'].append(dict(interval=[lo, hi], reason=str(error)))

    grid = sorted(set([0.] if collapsed else
                      np.linspace(0., 1., 9).tolist()+[float(initial[pitch_col])]))
    for f in reversed(grid) if reverse else grid:
        try:
            r, _ = at(f)
            if abs(r) <= RESIDUAL_TOLERANCE:nominate(f, [f, f], 'near zero probe')
        except (ValueError, ArithmeticError) as error:interval_error(f, f, error)

    def crossings(nodes):
        for lo, hi in zip(nodes, nodes[1:]):
            if lo not in cache or hi not in cache:continue
            if any(lo <= f <= hi for f in failures):continue
            a, b = cache[lo][0], cache[hi][0]
            if a*b >= 0.:continue
            # Do not repeat a bracket already containing a nominated seed.
            if any(lo <= r['pitch_fraction'] <= hi for r in roots):continue
            try:
                f = brentq(lambda f:at(f)[0], lo, hi, xtol=1e-8, maxiter=24)
                nominate(f, [lo, hi], 'restoring crossing hint' if a>b else 'repelling crossing hint')
                if not any(lo <= r['pitch_fraction'] <= hi for r in roots):
                    interval_error(lo, hi, 'crossing has no closed torque root; possible discontinuity')
            except (ValueError, ArithmeticError, RuntimeError) as error:interval_error(lo, hi, error)

    # Preserve obvious crossings even if a later extremum exhausts the budget.
    crossings(grid)
    for lo, mid, hi in zip(grid, grid[1:], grid[2:]):
        if any(f not in cache for f in (lo,mid,hi)):continue
        a,b,c = (cache[f][0] for f in (lo,mid,hi))
        if (b-a)*(c-b) >= 0.:continue
        sign = 1. if b<a else -1.
        try:
            fit = minimize_scalar(lambda f:sign*at(f)[0], bounds=(lo,hi), method='bounded',
                                  options=dict(xatol=1e-5,maxiter=24))
            r,_ = at(fit.x)
            extrema.append(dict(pitch_fraction=float(fit.x),net_torque_scaled=r,
                                kind='minimum' if sign>0 else 'maximum'))
            if not fit.success:interval_error(lo, hi, 'extremum refinement budget')
            nominate(fit.x, [lo,hi], 'near zero extremum')
            crossings(sorted(set([lo,float(fit.x),hi])))
        except (ValueError, ArithmeticError) as error:interval_error(lo, hi, error)
    row.update(pitch_samples=count,kernel_evaluations=calls,
               failed_probes=[dict(pitch_fraction=f,reason=r) for f,r in sorted(failures.items())])
    roots.sort(key=lambda r:r['pitch_fraction'])
    return row


def discover(problem, x, *, budget=None):
    """Nominate joint seeds from frozen pitch/wake branches; do not trim aircraft.

    The finite scan is explicitly incomplete. A seed is not an admitted state,
    a selected governor branch, or proof of existence or absence.
    """
    budget = budget or Budget()
    diagnostics = dict(revision=REVISION, status='unresolved', trigger='propulsion nonclosure',
                       shafts=[], corrections=[], coverage_complete=False,
                       governor_selection_certified=False)
    seeds = []
    problem.residual(x)
    attitude = problem.attitude(x)
    prep, requests, velocity, rates = problem.prepared(attitude)
    # The problem can reuse an exact propulsion sample while preparation's
    # scratch snapshot belongs to a later derivative probe.
    frame = problem.last['propulsion']['state']
    p = problem.solver.engine.properties
    q = problem.last['propulsion']['_equilibrium_coordinates'].copy()
    targets = [s['omega'] for s in frame['transmissions']]
    spans, scales, pitch_intervals = {}, [], {}
    for i, shaft in enumerate(p['transmissions']):
        scales.append(max(1000.,sum(abs(p['engines'][l['index']]['properties'].get('torque_base',1000.)*l['ratio'])
                                    for l in shaft['engines'])))
        if ('pitch',i) not in prep['paths'] or not requests[i]['governed']:continue
        links = [l for l in shaft['propellers'] if p['propellers'][l['index']]['properties']['governor']]
        intervals = [pitch_limits(p['propellers'][l['index']]['properties'],velocity,rates,targets[i]*l['ratio'])
                     for l in links]
        spans[i] = max((hi-lo for lo,hi in intervals),default=0.)
        pitch_intervals[i] = [list(v) for v in intervals]
    if not spans:
        diagnostics['reason']='No governed pitch unknown';return seeds, diagnostics
    evaluators,_ = prepare_evaluators(p,frame,prep['paths'],targets,spans,scales,
        velocity,problem.solver.config['altitude_m'],rates,problem.solver.mass['cog'],pitch_limits)
    options=[]
    for shaft in spans:
        check_cancel()
        si=prep['paths'].index(('shaft',shaft));pi=prep['paths'].index(('pitch',shaft))
        indices={l['index'] for l in p['transmissions'][shaft]['propellers']}
        flows=[j for j,path in enumerate(prep['paths']) if path[0]=='flow' and path[1] in indices]
        row=scan(evaluators[shaft],q,prep['bounds'],si,pi,flows,budget=budget,collapsed=spans[shaft]==0.)
        row.update(shaft=shaft,request_index=problem.request_indices[shaft],
                   requested_omega=targets[shaft],pitch_intervals_rad=pitch_intervals[shaft],torque_scale_nm=scales[shaft])
        for probe in row['probes']:probe['net_torque_nm']=probe['net_torque_scaled']*scales[shaft]
        diagnostics['shafts'].append(row)
        options.append([(pi,flows,np.array(r['coordinates'])) for r in row['roots']])
    total=math.prod(len(v) for v in options)
    diagnostics['candidate_combinations']=total
    allowance=budget.corrections
    diagnostics['combination_budget_exhausted']=total>allowance
    for combination in itertools.islice(itertools.product(*options),allowance):
        check_cancel()
        seed=q.copy()
        for pi,flows,state in combination:
            seed[pi]=state[pi];seed[flows]=state[flows]
        start=np.array(x,copy=True);start[problem.prop_start:]=seed[problem.active]
        seeds.append(start)
    if seeds:diagnostics['status']='discovered_seeds'
    return seeds, diagnostics


def recover(problem, x, *, detailed=False, budget=None, conditional=False):
    """Correct nominated seeds through the shared method and common admission."""
    budget = budget or Budget()
    seeds, diagnostics = discover(problem, x, budget=budget)
    diagnostics['conditional_aircraft'] = conditional
    candidates = []
    for start in seeds:
        check_cancel();charge('stationary_branch_corrections')
        budget.corrections-=1
        try:
            offset=problem.history_start if conditional else 0
            def expand(z):
                full=start.copy();full[offset:]=z;return full
            inner_scales=(np.asarray(problem.correction_residual_scales[offset:])
                          if conditional else None)
            fit=problem.correct(start,offset=offset,max_nfev=CORRECTION_NFEV,
                                residual_scale=inner_scales)
            point=problem.finish(expand(fit.x),detailed=detailed)
            point['internal_residual_norm']=float(np.linalg.norm(point['stationary_residual'][problem.history_start:]))
            point['internal_closed']=bool(problem.last['propulsion']['converged'] and
                np.all(abs(np.asarray(point['stationary_residual'][problem.history_start:]))<=
                       np.asarray(problem.correction_residual_scales[problem.history_start:])))
            point['numerical_message']=str(fit.message)
            point['correction_diagnostics']={name:getattr(fit,name,None) for name in
                ('termination','success','nfev','evaluations','derivative_evaluations',
                 'iterations','accepted_steps','contractions','coordinate_polls','trust_radius',
                 'return_metric','return_score','closure_ratio','merit_norm')}
            point['branch_recovery_candidate']=True
            candidates.append(point)
            diagnostics['corrections'].append(dict(local_valid=point['local_valid'],internal_closed=point['internal_closed'],reasons=point['reasons'],
                force_error_g=point['force_error_g'],angular_error_rad_s2=point['angular_error_rad_s2']))
        except (ValueError,ArithmeticError,np.linalg.LinAlgError) as error:
            diagnostics['corrections'].append(dict(error=str(error)))
    if any(p['local_valid'] for p in candidates):diagnostics['status']='recovered_candidates'
    elif conditional and any(p['internal_closed'] for p in candidates):diagnostics['status']='recovered_internal_candidates'
    return candidates,diagnostics
