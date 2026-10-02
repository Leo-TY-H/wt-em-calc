import math
import os
from collections import OrderedDict
from copy import deepcopy
import numpy as np
from scipy.optimize import OptimizeResult, least_squares
from component_assembly import f32
from propulsion_general import target_omega, delivered_boost
from prop_quasisteady import frame as step, REVISION
from propeller_general import local_flow
from prop_steady import initial_state
from em_cancellation import check as check_cancel
from jet_model import scalar_update
from control_mixer import density_at_height
from piston_model import div
from prop_equilibrium_blocks import variable_blocks, grouped_jacobian
from em_trim_work import charge


RESIDUAL_TOLERANCE = 1e-5


EARLY_STOP_ENABLED = os.environ.get('WT_EM_PROP_EARLY_STOP', '1') == '1'
EARLY_STOP_TOLERANCE = 1e-6


def _fit_equilibrium(evaluate, initial, **options):
    if not EARLY_STOP_ENABLED:
        return least_squares(evaluate, initial, **options)

    class ClosedEquilibrium(Exception):
        def __init__(self, x, residual):
            self.x = np.array(x, copy=True)
            self.residual = np.array(residual, copy=True)

    def checked(x):
        residual = evaluate(x)
        if np.isfinite(residual).all() and np.max(np.abs(residual), initial=0.) <= EARLY_STOP_TOLERANCE:
            raise ClosedEquilibrium(x, residual)
        return residual

    try:
        return least_squares(checked, initial, **options)
    except ClosedEquilibrium as closed:
        return OptimizeResult(x=closed.x, fun=closed.residual, residual_stop=True)


def pitch_limits(p, velocity, body_omega, shaft_omega):
    lo, hi = p['pitch_min'], p['pitch_max']
    if not p['governor']:
        return lo, lo
    local = local_flow(tuple(p['basis']), tuple(p['position']), tuple(velocity), tuple(body_omega))[0]
    neutral = (math.pi / 2 if abs(shaft_omega) < 1e-5 else
        math.atan2(local[0], p['neutral_radius'] * shaft_omega) - p['mean_twist']
        - p['polar']['base'][8] / p['polar']['base'][1] * math.pi / 180)
    floor = neutral - .12
    lo = floor if p['governor_fast'] and p['governor'] == 6 else max(lo, floor)
    if p['governor'] == 8:
        lo = max(lo, p['pitch_min'] + .05235988)
    return min(lo, hi), hi


def _regime_fit(evaluate, x, paths, bounds, blocks, targets, modes, full_jacobian=None):
    fixed={};omit=[]
    for j,path in enumerate(paths):
        if path[0]!='pitch':continue
        shaft=paths.index(('shaft',path[1]));mode=modes[path[1]]
        fixed[shaft if mode=='target' else j]=(targets[path[1]]/300. if mode=='target' else float(mode))
        omit.append(j)
    if any(not bounds[0][j]<=v<=bounds[1][j] for j,v in fixed.items()):return None
    active=[j for j in range(len(x)) if j not in fixed]
    rows=[j for j in range(len(x)) if j not in omit]
    base=x.copy()
    for j,value in fixed.items():base[j]=value
    lo=bounds[0][active].copy();hi=bounds[1][active].copy()
    for k,j in enumerate(active):
        if paths[j][0]=='pitch':lo[k]=0.;hi[k]=1.
    def expand(z):
        q=base.copy();q[active]=z;return q
    def residual(z):return evaluate(expand(z))[rows]
    def derivative(z):
        matrix=(full_jacobian(expand(z),2e-5) if full_jacobian is not None else
                grouped_jacobian(evaluate,expand(z),bounds,blocks,2e-5))
        return matrix[np.ix_(rows,active)]
    fit=_fit_equilibrium(residual,np.clip(base[active],lo,hi),jac=derivative,bounds=(lo,hi),
                         max_nfev=45,ftol=1e-10,xtol=2e-10,gtol=1e-10)
    fit.x=expand(fit.x)
    return fit


def solve(p, velocity, height, body_omega, cg, dt, nitro, controls, torque_gyro=True, state=None, *, _prepare_only=False, _retain_linearization=False):
    native_controls = dict(controls, engine_control_mode='automatic')
    seed = initial_state(p, velocity, height, nitro=nitro, **native_controls)
    if state is not None:
        for group in ('engines', 'propellers', 'transmissions'):
            for s, old in zip(seed[group], state[group]):
                for key in ('omega', 'turbo', 'pitch', 'flow'):
                    if key in old:
                        s[key] = old[key]
    paths, scales, lower, upper, initial = [], [], [], [], []
    governed = []
    shaft_targets = []
    torque_scales = []
    def variable(path, value, scale, lo, hi):
        paths.append(path); scales.append(scale)
        lower.append(lo / scale); upper.append(hi / scale); initial.append(value / scale)
    for i, t in enumerate(p['transmissions']):
        links = [link for link in t['propellers'] if p['propellers'][link['index']]['properties']['governor']]
        governed.append(links)
        target = max((target_omega(p['engines'][l['index']]['properties'], seed['engines'][l['index']], nitro)
                      * l['inverse_ratio'] for l in t['engines']), default=300.)
        shaft_targets.append(target)
        limit = min((p['engines'][l['index']]['properties']['omega_limit'] * l['inverse_ratio']
                     for l in t['engines']), default=max(300., target))
        variable(('shaft', i), seed['transmissions'][i]['omega'] if state else target, 300., 1., 2. * limit)
        torque_scales.append(max(1000., sum(abs(p['engines'][l['index']]['properties'].get('torque_base', 1000.)
                                               * l['ratio']) for l in t['engines'])))
        if links:
            pp = p['propellers'][links[0]['index']]['properties']
            lo, hi = pitch_limits(pp, velocity, body_omega, target * links[0]['ratio'])
            fraction = ((seed['propellers'][links[0]['index']]['pitch'] - lo) / (hi - lo)
                        if state and hi > lo else .5)
            variable(('pitch', i), min(1., max(0., fraction)), 1., -.25, 1.25)
    for i, prop in enumerate(p['propellers']):


        seed['propellers'][i]['flow']=list(seed['propellers'][i]['flow'])
        seed['propellers'][i]['flow'][2 if prop['properties']['coaxial'] else 1]=0.
        for j in ((0, 1) if prop['properties']['coaxial'] else (0, 2)):
            variable(('flow', i, j), seed['propellers'][i]['flow'][j], 20., -150. if j < 2 else 0., 150. if j < 2 else 20.)
    for i, e in enumerate(p['engines']):
        ep = e['properties']; es = seed['engines'][i]
        if e['family'] in (2, 5):
            normalized = min(div(es['throttle'], f32(1.1 if ep['throttle_boost'] > 1. else 1.)), 1.)
            scalar = scalar_update(e['turbine'], density_at_height(f32(height)), velocity[0],
                e['turbine']['max_omega'], normalized, dt,
                afterburner=delivered_boost(ep, es, nitro), health=1., running=2)
            key, limit = ('turbo', ep['turbo_allowed']) if e['family'] == 5 else ('omega', ep['omega_limit'])
            es[key] = f32(min(max(scalar['target_omega'], 0.), limit))
    scales = np.asarray(scales)
    bounds = (np.asarray(lower), np.asarray(upper))
    calls = 0
    last = None
    evaluations = OrderedDict()

    def evaluate(x):
        nonlocal calls, last
        check_cancel()
        # SciPy and the grouped derivative both request the base point. Only
        # exact coordinates within this solve may reuse a deterministic frame.
        # Prepared samplers expose mutable state, so they retain fresh frames.
        key = None if _prepare_only else np.asarray(x, dtype=np.float64).tobytes()
        if key is not None and key in evaluations:
            residual, last = evaluations[key]
            evaluations.move_to_end(key)
            charge('engine_evaluation_reuse')
            return residual.copy()
        charge('engine_frames')
        calls += 1
        s = dict(seed, engines=[dict(e, regulator=-1.) for e in seed['engines']],
                 transmissions=[dict(t) for t in seed['transmissions']],
                 propellers=[dict(q, flow=list(q['flow'])) for q in seed['propellers']])
        fractions = {}
        for path, value in zip(paths, x * scales):
            kind, i = path[:2]
            if kind == 'shaft':s['transmissions'][i].update(omega=float(value), previous_omega=float(value))
            elif kind == 'pitch':fractions[i] = value
            elif kind == 'flow':s['propellers'][i]['flow'][path[2]] = float(value)
        for i, t in enumerate(p['transmissions']):
            for link in t['engines']:
                j = link['index']; ep = p['engines'][j]['properties']; es = s['engines'][j]
                es['omega'] = s['transmissions'][i]['omega'] * link['ratio']
            for link in t['propellers']:
                j = link['index']; pp = p['propellers'][j]['properties']
                lo, hi = pitch_limits(pp, velocity, body_omega, s['transmissions'][i]['omega'] * link['ratio'])
                pitch = lo + min(1., max(0., fractions.get(i, 0.))) * (hi - lo)
                s['propellers'][j].update(pitch=pitch, governor_pitch=pitch)
        r = step(p, s, velocity, height, body_omega, cg, dt, s['seed'], nitro, torque_gyro=torque_gyro)
        if r['seed'] != s['seed']:
            raise ValueError('Quasi-steady propulsion requires deterministic healthy engine outputs')
        torques, targets = [], []
        for i, t in enumerate(p['transmissions']):
            torque = sum(r['engines'][l['index']]['torque'] * l['ratio'] for l in t['engines'])
            load = sum(r['propellers'][l['index']]['outputs'][18] * l['ratio'] for l in t['propellers'])

            if any(r['engines'][l['index']]['friction'] for l in t['engines']) or any(
                    r['propellers'][l['index']]['outputs'][19] for l in t['propellers']):
                raise ValueError('Quasi-steady shaft friction is not supported')
            torques.append(torque - load)
            target = max((target_omega(p['engines'][l['index']]['properties'], r['engines'][l['index']], nitro)
                          for l in t['engines']), default=0.)
            boost = any(p['engines'][l['index']]['properties']['boost_controllable'] and
                        delivered_boost(p['engines'][l['index']]['properties'], r['engines'][l['index']], nitro)
                        for l in t['engines'])
            desired = []
            for link in governed[i]:
                pp = p['propellers'][link['index']]['properties']
                value = target
                if pp['governor'] in (1, 2):
                    value = min(max(value, pp['min_omega']), pp['boost_omega'] if boost else pp['max_omega'])
                desired.append(value * pp['reduction'] / link['ratio'])
            if desired and max(desired) - min(desired) > 1e-4 * max(1., max(desired)):
                raise ValueError('Conflicting ideal governor targets on a shared shaft')
            targets.append(max(desired, default=shaft_targets[i]))
        residual = []
        for path, value, scale in zip(paths, x * scales, scales):
            kind, i = path[:2]
            if kind == 'shaft':error = torques[i] / torque_scales[i]
            elif kind == 'pitch':
                rpm_error = (s['transmissions'][i]['omega'] - targets[i]) / 300.
                error = value - min(1., max(0., value + rpm_error))
            elif kind == 'flow':error = (r['propellers'][i]['equilibrium_flow'][path[2]] - s['propellers'][i]['flow'][path[2]]) / scale
            residual.append(error)

        for i, row in enumerate(r['propellers']):
            row.update(pitch=s['propellers'][i]['pitch'], governor_pitch=s['propellers'][i]['pitch'])
        for i, row in enumerate(r['transmissions']):
            row.update(omega=s['transmissions'][i]['omega'], previous_omega=s['transmissions'][i]['omega'])
        last = (r, torques, targets, fractions)
        residual = np.asarray(residual)
        if key is not None:
            evaluations[key] = residual.copy(), last
            if len(evaluations) > 8:evaluations.popitem(last=False)
        return residual

    retained_jacobian=None
    def jacobian(x, step_size=2e-4):
        nonlocal retained_jacobian
        matrix=grouped_jacobian(evaluate, x, bounds, blocks, step_size)
        if _retain_linearization:retained_jacobian=(x.copy(),matrix.copy())
        return matrix

    blocks = variable_blocks(p, paths)

    if _prepare_only:
        def sample_numeric(x):
            residual=evaluate(np.asarray(x))
            frame=last[0]
            outputs=np.asarray(frame['aggregate_force']+frame['aggregate_moment']+
                frame['engine_angular_momentum']+frame['engine_wash'])
            modes=tuple('fixed pitch' if not links else 'minimum pitch' if last[3][i]<=2e-5
                else 'maximum pitch' if last[3][i]>=1.-2e-5 else 'target RPM'
                for i,links in enumerate(governed))
            branch=(modes,tuple(e.get('gear',0) for e in frame['engines']))
            return residual,outputs,branch
        def sample(x, exact=False):
            q = np.array(x, copy=True)
            residual = evaluate(q)
            if exact:


                for i, links in enumerate(governed):
                    if links and 2e-5 < last[3][i] < 1.-2e-5:
                        q[paths.index(('shaft', i))] = last[2][i] / 300.
                residual = evaluate(q)
            result = _result(p, state, controls, governed, last, residual, calls, 0)
            result['_equilibrium_coordinates'] = q
            return residual, result
        return dict(initial=np.clip(initial, *bounds), bounds=bounds, paths=paths,
                    evaluate=evaluate, sample=sample, scales=scales, seed=seed,
                    snapshot=lambda:last,calls=lambda:calls,jacobian=jacobian,sample_numeric=sample_numeric)

    x = np.clip(initial, *bounds)
    early_stops = 0
    best = None


    evaluate(x)
    modes={i:'target' for i,links in enumerate(governed) if links}
    for _ in range(3):
        targets=list(last[2])
        fit=_regime_fit(evaluate,x,paths,bounds,blocks,targets,modes,jacobian if _retain_linearization else None)
        if fit is None:break
        early_stops+=bool(getattr(fit,'residual_stop',False))
        residual=evaluate(fit.x);error=float(max(abs(residual),default=0.))
        if best is None or error<best[0]:best=(error,fit.x.copy())
        if error<=RESIDUAL_TOLERANCE:break
        changed=False
        for j,path in enumerate(paths):
            if path[0]!='pitch':continue
            i=path[1]
            if modes[i]=='target' and (fit.x[j]<2e-5 or fit.x[j]>1.-2e-5):
                modes[i]=0 if fit.x[j]<.5 else 1;changed=True
        x=fit.x.copy()
        if not changed:break


    for fraction in (None, .2, .8):
        if best is not None and best[0]<=RESIDUAL_TOLERANCE:break
        trial = x.copy()
        if fraction is not None:
            for j, path in enumerate(paths):
                if path[0] == 'pitch':trial[j] = fraction
                elif path[0] == 'shaft':trial[j] = np.clip(shaft_targets[path[1]] / 300., bounds[0][j], bounds[1][j])
        fit = _fit_equilibrium(evaluate, trial, jac=jacobian, bounds=bounds, max_nfev=45,
                            ftol=1e-10, xtol=2e-8, gtol=1e-10)
        early_stops += bool(getattr(fit, 'residual_stop', False))
        residual = evaluate(fit.x)
        snapped = fit.x.copy()
        for j, path in enumerate(paths):
            if path[0] == 'pitch':
                i = path[1]
                rpm_error = last[0]['transmissions'][i]['omega'] - last[2][i]
                if abs(snapped[j]) < .002 and rpm_error < 0:snapped[j] = 0.
                elif abs(snapped[j] - 1.) < .002 and rpm_error > 0:snapped[j] = 1.
        if not np.array_equal(snapped, fit.x):
            fit = _fit_equilibrium(evaluate, snapped, jac=jacobian, bounds=bounds, max_nfev=20,
                                ftol=1e-10, xtol=2e-8, gtol=1e-10)
            early_stops += bool(getattr(fit, 'residual_stop', False))
            residual = evaluate(fit.x)
        error = float(max(abs(residual), default=0.))

        for delta in (2e-5, 2e-6):
            if error <= RESIDUAL_TOLERANCE:break
            for _ in range(4):
                direction = np.linalg.lstsq(jacobian(fit.x, delta), -residual, rcond=1e-9)[0]
                if max(abs(direction), default=0.) > .02:break
                improved = False
                for factor in (1., .5, .25):
                    q = np.clip(fit.x + factor * direction, *bounds); rr = evaluate(q)
                    ee = float(max(abs(rr), default=0.))
                    if ee < error:
                        fit.x, residual, error = q, rr, ee; improved = True; break
                if not improved or error <= RESIDUAL_TOLERANCE:break
        if best is None or error < best[0]:best = (error, fit.x.copy())
        if error <= RESIDUAL_TOLERANCE:break
    error, x = best


    evaluate(x)
    final_modes={i:0 if last[3][i]<=2e-5 else 1 if last[3][i]>=1.-2e-5 else 'target'
                 for i,links in enumerate(governed) if links}
    if any(mode=='target' and x[paths.index(('shaft',i))]*300.!=last[2][i]
           for i,mode in final_modes.items()):
        exact=_regime_fit(evaluate,x,paths,bounds,blocks,list(last[2]),final_modes,jacobian if _retain_linearization else None)
        if exact is not None:
            x=exact.x;early_stops+=bool(getattr(exact,'residual_stop',False))
    residual = evaluate(x)
    linearization=None
    if _retain_linearization:
        # A fitting Jacobian belongs to its exact iterate. Refresh it when the
        # final root differs; never silently reuse a tangent from another state.
        if retained_jacobian is None or not np.array_equal(retained_jacobian[0],x):
            jacobian(x,2e-5)
        residual=evaluate(x)
        linearization=dict(coordinates=x.copy(),matrix=retained_jacobian[1],
            bounds=tuple(b.copy() for b in bounds),paths=tuple(paths),
            condition=np.r_[velocity,body_omega,height],
            fixed=(tuple(cg),dt,nitro,deepcopy(controls),torque_gyro))
    result=_result(p, state, controls, governed, last, residual, calls, early_stops)
    if linearization is not None:result['_linearization']=linearization
    return result


def _result(p, state, controls, governed, last, residual, calls, early_stops):
    r, torques, targets, fractions = last
    error = float(np.max(np.abs(residual), initial=0.))
    converged = bool(np.isfinite(residual).all() and error <= RESIDUAL_TOLERANCE)
    rpm_errors=[r['transmissions'][i]['omega']-targets[i] for i in range(len(governed))]


    converged = converged and all(not links or fractions[i]<=2e-5 or fractions[i]>=1.-2e-5
        or abs(rpm_errors[i])<=1e-10*max(1.,abs(targets[i])) for i,links in enumerate(governed))
    row = r['aggregate_force'] + r['aggregate_moment'] + r['engine_angular_momentum'] + r['engine_wash']
    converged = converged and all(math.isfinite(v) for v in row)
    stopped = [i for i, t in enumerate(r['transmissions']) if t['omega'] <= 1.]
    diagnostics = dict(method='quasi-steady ideal governor equilibrium', model_revision=REVISION,
        force_model='continuous steady blade polar; normalized Hermite Mach curves', residual=error,
        residual_early_stop_enabled=EARLY_STOP_ENABLED, residual_early_stops=early_stops,
        frame_evaluations=calls, shaft_torque_residual_nm=torques,
        shaft_target_rpm=[v * 60 / (2 * math.pi) for v in targets],
        shaft_rpm_error=[v * 60 / (2 * math.pi) for v in rpm_errors],
        induced_flow_residual_mps=[[a-b for a,b in zip(q['equilibrium_flow'],q['flow'])] for q in r['propellers']],
        governor_status=[('fixed pitch' if not governed[i] else 'minimum pitch' if fractions[i] <= 2e-5 else
                          'maximum pitch' if fractions[i] >= 1 - 2e-5 else 'target RPM') for i in range(len(governed))])
    return dict(state=r, force=row[:3], moment=row[3:6], angular_momentum=row[6:9], wash=row[9:],
        converged=converged, feasible=converged and not stopped, stopped_shafts=stopped,
        overspeed_engines=[i for i, e in enumerate(r['engines']) if e['omega'] > p['engines'][i]['properties']['omega_limit'] * (1 + 1e-6)],
        period_frames=None, cycle_samples=[row] if converged else None, window_relative_change=None,
        simulated_seconds=0., controls=controls, canonical_initialization=state is None, stationarity=diagnostics)
