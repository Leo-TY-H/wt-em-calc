"""Stationary point equations behind the unchanged baseline chart sampler.

`valid` identifies a locally admitted point usable by the chart. Global proof
flags remain separate. Sampling and PCHIP interpolation are supplied by
em_sampling/em_plot, not a second mesh.
"""
import copy
import itertools
from contextlib import contextmanager
from contextvars import ContextVar
from functools import wraps

import numpy as np

from em_solver import TrimSolver
from em_stationary import JointProblem, prepare, solve as stationary_solve

_presentation_views = ContextVar('stationary_presentation_views', default=frozenset())


def stationary_frozen_point(core, speed, load, x, *args, history_initial=None, **kwargs):
    """Close only wing history while the aircraft and propulsion stay fixed.

    Pitch-response proposals retain their prescribed propulsion sample. Their
    history relation is the same explicit stationary relation as JointProblem,
    rather than the old capped history iteration. Explicit-history calls from
    this evaluator bypass the dispatcher, so every native sample is charged
    normally and there is no recursive conditional solve.
    """
    if core.config.get('trim_solver_mode') == 'off':
        raise ValueError('Correction is off; conditional history correction is unavailable')
    native = getattr(core, 'native_loop', None)
    if native is not None:
        handled, value = native.frozen_point(core, speed, load, x, args, kwargs, history_initial)
        if handled:return value
    if (core.config.get('trim_solver_mode') != 'stationary'
            or core.config.get('experimental_instantaneous_aero',False)
            or kwargs.get('stationary_angles') is not None):
        return core.operating_point(speed,load,x,*args,**kwargs)
    from em_stationary_corrector import Corrector, PRODUCTION_NUMERICAL_TARGET
    propulsion = kwargs.get('propulsion_sample', args[0] if args else None)
    key = (speed, load, getattr(core, 'sideslip_attitude_deg', 0.))
    previous = getattr(core, '_stationary_frozen_predictor', None)
    nearby = (previous is not None and previous['key'] == key
        and previous['propulsion'] is propulsion
        and np.max(abs(np.asarray(x)-previous['attitude']) /
                   np.array([8.,20.,.4,.4,.4])) <= 1.)
    initial = (previous['angles'] if nearby else
               history_initial if history_initial is not None else [x[0]]*2)
    memo={}
    def sample(angles):
        key=tuple(angles)
        if key not in memo:
            value=core.operating_point(speed,load,x,*args,
                **dict(kwargs,stationary_angles=list(angles)))
            memo[key]=dict(value,residual=value['stationary_angle_residual']/10.,
                           _aircraft_value=value)
        return memo[key]
    fit=Corrector(sample,([-1000.]*2,[1000.]*2),[.002]*2,
        branch=core.derivative_branch,scale=[10.]*2,step_limit=[8.]*2,
        residual_scale=[2e-5]*2).solve(initial, reuse_jacobian=True,
            numerical_target=PRODUCTION_NUMERICAL_TARGET,
            predictor=previous['predictor'] if nearby else None)
    value=dict(fit.value['_aircraft_value'])
    core._stationary_frozen_predictor = dict(key=key, propulsion=propulsion,
        attitude=np.asarray(x).copy(), angles=fit.x.copy(), predictor=fit.predictor)
    value['_stationary_frozen_history']=dict(coordinates=fit.x.tolist(),
        residual=fit.fun.tolist(),termination=fit.termination,nfev=fit.nfev,
        evaluations=fit.evaluations,numerical_target_reached=fit.success,
        scope='Conditional history closure at fixed aircraft coordinates and propulsion; no aircraft admission')
    return value


class BaselineSolver(TrimSolver):
    # Let the unchanged load sampler establish the requested cap or an actual
    # rejection bracket first. A separate all-face trace before ordinary load
    # sampling duplicates that work and can chase limits beyond the chart cap.
    pretrace_boundary = False
    # These are search bounds. Only the bank endpoints are physical admission.
    trim_bounds = ((-180., -90., -1., -1., -1.), (180., 90., 1., 1., 1.))

    def core(self):
        # Cache only admitted continuation seeds owned by this configured solver.
        # Every reuse is corrected/replayed; failed searches are never cached.
        if not hasattr(self,'_stationary_seeds'):
            from collections import OrderedDict
            self._stationary_seeds=OrderedDict()
        if not hasattr(self,'_stationary_predictors'):
            from collections import OrderedDict
            self._stationary_predictors=OrderedDict()
        core = copy.copy(self)
        core.__class__ = TrimSolver
        core._sideslip_solvers = {}
        core.chart_search = False
        return core

    def sample(self, result):
        from em_stationary_surface import chart_point
        from em_trim_work import record_point
        row = chart_point(result, self.config['sep_tolerance_mps'])
        # The selected state's complete physical record is already at the row
        # root. Keep lightweight candidate identities for selection reporting;
        # do not copy the same forces, controller and propulsion three more
        # times into each chart point and every subsequent preview/export.
        fields=('speed_kmh','load_g','solution','stationary_coordinates',
            'stationary_coordinate_names','stationary_request_indices',
            'sideslip_attitude_deg','source_branch','local_valid','converged',
            'reasons','ps_mps','turn_dps','force_error_g','angular_error_rad_s2',
            'history_error','equilibrium_coverage_complete')
        summaries={}
        def brief(point):
            identity=id(point)
            if identity not in summaries:
                summaries[identity]={key:point[key] for key in fields if key in point}
            return summaries[identity]
        for key in ('stationary_candidates','selected_candidates'):
            if key in row:row[key]=[brief(point) for point in row[key]]
        if 'chart_candidate' in row:row['chart_candidate']=brief(row['chart_candidate'])
        row['valid'] = row['chart_usable']
        row['validity_scope'] = 'numerical equilibrium and physical admission'
        # Normalize once in the point's worker, rather than traversing every
        # old point again for every progressive preview in the coordinator.
        from em_stationary_chart import clean
        detail=row.get('_detail')
        row=clean(row)
        if detail is not None:row['_detail']=detail
        record_point(row)
        return row

    def solve(self, speed_kmh, load, initial=None, detailed=False, exhaustive=True,
              refine=False, quick=False):
        result=stationary_solve(self.core(), speed_kmh, load, initial,
            detailed, exhaustive, refine, quick)
        if 'stationary_coordinates' not in result:
            self.unresolved('stationary sample has no established candidate')
        return self.sample(result)

    def _solve_attempt(self, *args, **kwargs):
        # Some baseline recovery routines drive a generator instead of solve().
        # They must not bypass the stationary sample evaluator.
        if False:yield
        return self.solve(*args, **kwargs)

    @staticmethod
    def unresolved(reason):
        from em_trim_work import WorkLimit, current
        if current() is not None:raise WorkLimit(current(), reason)
        raise ValueError('Unresolved numerical calculation: '+reason)

    def operating_point(self, speed, load, x, *args, **kwargs):
        """Conditional history/shaft closure for baseline boundary correctors.

        Trial aircraft coordinates stay prescribed. The inner equations are
        precisely the same JointProblem equations used at a sample point.
        Legacy frozen-governor/limit-cycle options cannot replace them.
        """
        core = self.core()
        if (kwargs.get('propulsion_sample') is not None
                or kwargs.get('propulsion_override') is not None
                or args and args[0] is not None):
            # Retain the declared pitch-response test at a fixed propulsion
            # sample. This is a derivative probe, not a new admitted trim;
            # certify() always reevaluates the coupled stationary state.
            return stationary_frozen_point(core,speed,load,x,*args,**kwargs)
        requests = prepare(core, speed, load, x)[1] if self.is_prop else []
        branches = itertools.product(*[range(len(r['candidates'])) if r['governed']
                                       else [0] for r in requests])
        trials = [];recovery_records=[];pending_recovery=[]
        from em_prop_branch_recovery import Budget, recover
        budget=Budget()
        def retain(problem,branch,q):
            residual=problem.value_at(q)[0][problem.history_start:]
            closed=(np.all(abs(residual)<=problem.correction_residual_scales[problem.history_start:])
                    and problem.last['history_error']<=2e-4
                    and (not self.is_prop or problem.last['propulsion']['converged']))
            trials.append((not closed,float(np.linalg.norm(residual)),branch,problem,q.copy()))
        for branch in branches:
            problem = JointProblem(core, speed * 3.6, load, x, branch)
            q = problem.initial.copy()
            # Do not clip trial aircraft coordinates: callers own their bounds.
            q[:problem.history_start] = [x[i] for i in problem.attitude_indices]
            start = problem.history_start
            if start < len(q):
                governed=any(path[0]=='pitch' for path in getattr(problem,'paths',()))
                fit = problem.correct(q, offset=start,
                    residual_scale=problem.correction_residual_scales[start:],warm_start=True)
                q[start:] = fit.x
                retain(problem,branch,q)
                if governed and not problem.last['propulsion']['converged']:
                    pending_recovery.append((problem,branch,q.copy()))
            else:retain(problem,branch,q)
        # Complete normal conditional correction on every request before seed
        # discovery. An already closed inner state does not need another search
        # merely because an earlier request/start failed; no absence is inferred.
        if not any(not row[0] for row in trials):
            for problem,branch,q in pending_recovery:
                extra,record=recover(problem,q,budget=budget,conditional=True)
                recovery_records.append(record)
                for point in extra:
                    retain(problem,branch,np.asarray(point['stationary_coordinates']))
        if not trials:
            # No request at this trial is not a proof about a whole column.
            self.unresolved('stationary request unresolved')
        _, _, branch, problem, q = min(trials, key=lambda row:(row[0],row[1],row[2]))
        value = dict(problem.value_at(q)[1])
        value['_stationary_recovery']=recovery_records
        value['_stationary_trials'] = [dict(coordinates=z.tolist(), requests=b)
                                      for _, _, b, _, z in trials]
        if problem.symmetric:
            # Embed the reduced three-equation model in the baseline's five
            # coordinate interface. Neutral lateral controls are model constraints.
            value['residual'] = value['residual'].copy()
            value['residual'][2:4] = [x[2], x[4]]
        return value

    def certify(self, speed_kmh, load, x, final, *, evaluations=0, detailed=False,
                bound_stationary=False):
        trials = final.get('_stationary_trials')
        if trials is None:
            final = self.operating_point(speed_kmh/3.6, load, x)
            trials = final['_stationary_trials']
        points = []
        for trial in trials:
            problem = JointProblem(self.core(), speed_kmh, load, x, trial['requests'])
            point = problem.finish(np.asarray(trial['coordinates']), detailed=detailed)
            points.append(point)
        point = min(points, key=lambda p:(not p['local_valid'], tuple(p['stationary_coordinates'])))
        return self.sample(dict(point, stationary_candidates=points, stationary_attempts=[],
            propeller_branch_recovery=final.get('_stationary_recovery',[])))

    def point_value(self, point):
        return self.core().point_value(point)

    def direct_boundary(self, speed, interior):
        from em_stationary_boundary import find_boundary
        if not hasattr(self,'stationary_boundary_evidence'):
            self.stationary_boundary_evidence=[]
        return find_boundary(self, speed, interior,evidence=self.stationary_boundary_evidence)

    def boundary(self,speed_kmh,low,high,continuation=True,scalar_fallback=True,
                 candidate_kinds=None,max_iterations=24,trial_cycle_seconds=None):
        """Keep the chart's bracket protocol on the shared stationary method."""
        mode=self.config.get('trim_solver_mode')
        if mode == 'off':
            raise ValueError('Correction is off; boundary searches are unavailable')
        if mode != 'stationary':
            return super().boundary(speed_kmh,low,high,continuation,scalar_fallback,
                candidate_kinds,max_iterations,trial_cycle_seconds)
        from em_stationary_boundary import from_bracket
        angle=low.get('sideslip_attitude_deg',self.sideslip_attitude_deg)
        view=self if angle==self.sideslip_attitude_deg else self.at_sideslip(angle)
        if high.get('sideslip_attitude_deg',angle)!=angle:
            high=view.solve(speed_kmh,high['load_g'],high['solution'],exhaustive=False)
        if not hasattr(self,'stationary_boundary_evidence'):
            self.stationary_boundary_evidence=[]
        return from_bracket(view,speed_kmh,low,high,candidate_kinds=candidate_kinds,
            max_iterations=max_iterations,continuation=continuation,
            evidence=self.stationary_boundary_evidence)

    def continue_boundary(self,speed_kmh,low,high):
        if self.config.get('trim_solver_mode') == 'nested':
            return super().continue_boundary(speed_kmh,low,high)
        return self.boundary(speed_kmh,low,high,continuation=True)


class AxialReference:
    """Keep the baseline manual-RPM scan, evaluating stationary shaft equations."""
    def __init__(self, solver):
        self.solver = solver
        self.properties = solver.engine.properties

    def condition(self, velocity, rates, canonical=True):
        if any(rates) or any(velocity[1:]) or velocity[0]<=0.:
            raise ValueError('The chart RPM reference requires positive axial flow and zero rotation')
        try:
            return self.solver.operating_point(velocity[0], 1., [0.]*5)['propulsion']
        except (ValueError, ArithmeticError, np.linalg.LinAlgError):
            return dict(converged=False)


def dictionaries(value):
    """Walk shared records once; neither alter settings nor copy native objects."""
    seen, pending = set(), [value]
    while pending:
        item = pending.pop()
        if not isinstance(item, (dict, list, tuple)) or id(item) in seen:continue
        seen.add(id(item))
        if isinstance(item, dict):
            yield item
            # Rendered numeric grids and contour arrays contain no equilibrium
            # records. Walking millions of their scalars on every preview costs
            # more than correcting ordinary points.
            pending.extend(v for k, v in item.items() if k not in ('surface','contours')
                and (not isinstance(k,str) or not k.startswith('_')))
        elif item and isinstance(item[0],(dict,list,tuple)):pending.extend(item)


def public_result(data):
    # A comparison may contain legacy entries; preserve their reporting contract.
    targets=([a for a in data['aircraft'] if a.get('settings',{}).get('trim_solver_mode')!='nested']
             if 'aircraft' in data else [data])
    for target in targets:
        for row in dictionaries(target):
            if 'valid' in row and 'load_g' in row and 'speed_kmh' in row:
                usable = bool(row.get('chart_usable', row.get('local_valid', row['valid']))
                              and row.get('local_valid', True))
                row.update(valid=usable, chart_usable=usable,
                           status='admitted' if usable else 'unresolved',
                           validity_scope='numerical equilibrium and physical admission',
                           absence_certified=False, equilibrium_coverage_complete=False,
                           aircraft_selection_certified=False, governor_selection_certified=False)
            if 'boundary_status' in row:
                row['boundary_status_scope'] = 'numerically located boundary of the sampled branch'
    if 'coverage_complete' in data:data['coverage_complete']=False
    for aircraft in data.get('aircraft', []):
        if aircraft.get('settings',{}).get('trim_solver_mode')=='nested':continue
        aircraft['local_candidate_points'] = sum(p.get('chart_usable', False) for p in aircraft['points'])
        aircraft['valid_points'] = sum(bool(p['valid']) for p in aircraft['points'])
        if 'interpolation' in aircraft:
            aircraft['interpolation']['certification_scope'] = 'baseline numerical interpolation checks; no global equilibrium/selection certificate'
    return data


@contextmanager
def chart_view(data):
    """Normalize saved chart records to the same public/local validity policy."""
    if data.get('stationary_baseline_chart'):
        public_result(data)
    yield


def presentation(function):
    @wraps(function)
    def wrapped(data, *args, **kwargs):
        active = _presentation_views.get()
        if not data.get('stationary_baseline_chart') or active or data.get('_stationary_presentation_ready'):
            return function(data, *args, **kwargs)
        token = _presentation_views.set(active | {id(data)})
        try:
            with chart_view(data):result = function(data, *args, **kwargs)
        finally:_presentation_views.reset(token)
        if isinstance(result, dict) and result is not data:public_result(result)
        return result
    return wrapped


def compute(config, progress=None, cancelled=None, preview=None):
    from em_sampling import compute_cached
    from em_solver import aircraft_settings
    if any(aircraft_settings(config,name)['trim_solver_mode']!='stationary' for name in config['aircraft']):
        raise ValueError('Mixed stationary/legacy charts require separate runs')
    def finish(data):
        # The sampler supplies detached previews and clones its completed
        # cache. Point records have already been normalized by sample(); only
        # chart metadata belongs here. Rewalking that graph on every preview
        # serialized ordinary chart work in the coordinator.
        data=dict(data,stationary_baseline_chart=True,
            method='Baseline adaptive speed/load sampling and PCHIP; stationary sample equations',
            validation='Stationary numerical equilibrium and physical admission; baseline interpolation checks')
        data['assumptions']=[*data['assumptions'],
            'Baseline interpolation of admitted stationary points; unresolved searches remain gaps.']
        data['coverage_complete']=False
        for aircraft in data['aircraft']:
            aircraft['local_candidate_points']=sum(p.get('chart_usable',False) for p in aircraft['points'])
            aircraft['valid_points']=sum(bool(p['valid']) for p in aircraft['points'])
            if 'interpolation' in aircraft:
                aircraft['interpolation']['certification_scope']='baseline numerical interpolation checks; no global equilibrium/selection certificate'
                from em_accuracy_policy import ALLOWANCE,comparison_tolerance
                aircraft['interpolation'].update(numerical_sep_allowance_mps=float(ALLOWANCE),
                    comparison_tolerance_mps=comparison_tolerance(config),
                    inverse_interpolation_sep_allowance_mps=float(ALLOWANCE))
            for column in aircraft.get('columns',[]):
                column['boundary_status_scope']='numerically located boundary of the sampled branch'
        data['_stationary_presentation_ready']=True
        return data
    if config['sampling']!='adaptive':
        from em_solver import compute_regular
        return finish(compute_regular(config, progress, cancelled))
    return finish(compute_cached(config, progress, cancelled,
        (lambda data:preview(finish(data))) if preview else None))
