"""Joint stationary closure with continuation-first numerical selection.

Return a physically checked ordinary candidate without searching again after
success. Bounded alternate starts remain failure recovery, not a global census.
"""
import itertools
import math
import time
from numbers import Integral

import numpy as np

from component_assembly import f32
from em_cancellation import check as check_cancel
from em_operating import flight_condition
from em_trim_numerics import UnevaluableTrial
from em_stationary_corrector import PRODUCTION_NUMERICAL_TARGET
from prop_admission import evaluate as admission
from prop_equilibrium import RESIDUAL_TOLERANCE, solve as prepare_propulsion
from prop_requests import enumerate_speeds
from prop_quasisteady import propeller_condition
from propulsion_general import fixed_air

REVISION = 'joint-stationary-bounded-governor-v8'
STABLE_POLAR_REVISION = 'stored-mode3-binary64-v1'


class BranchUnavailable(UnevaluableTrial):
    """A search branch is unavailable; not a proof of aircraft absence."""


def braking_rule(solver):
    """S10 (Leo, 2026-10-20; WT_MARCH_S10=0 or WT_MARCH_S9=0 removes it, WT_MARCH_S10_BRAKE=0 this rule alone, for
    attributing chart changes to it): under the march chart's admission rule
    (solver.soft_admission) a propeller state whose net thrust is negative under power is rejected ('propeller braking')."""
    import os
    return (bool(getattr(solver, 'soft_admission', False)) and os.environ.get('WT_MARCH_S10', '1') != '0'
            and os.environ.get('WT_MARCH_S9', '1') != '0' and os.environ.get('WT_MARCH_S10_BRAKE', '1') != '0')


def propeller_braking(properties, state):
    """S10: the net thrust of the shafts' propellers (each propeller's force along its own axis) is negative while
    their engines deliver power (the sum of engine torque x shaft speed is positive). The native Finisher.braking."""
    thrust = power = 0.
    for i, shaft in enumerate(properties['transmissions']):
        for link in shaft['propellers']:
            index = link['index']
            force = state['propellers'][index]['outputs']
            basis = properties['propellers'][index]['properties']['basis']
            for k in range(3):thrust += force[k]*basis[k]
        for link in shaft['engines']:
            engine = state['engines'][link['index']]
            power += engine.get('torque', 0.)*engine['omega']
    return thrust < 0. and power > 0.


def prepare(solver, speed, load, attitude):
    velocity, _, geometry, _ = flight_condition(solver, speed, load, attitude)
    velocity = tuple(map(f32, velocity))
    rates = tuple(map(f32, geometry['omega']))
    engine = solver.engine
    p = engine.properties
    prep = prepare_propulsion(p, velocity, solver.config['altitude_m'], rates,
        solver.mass['cog'], engine.dt, solver.mass['nitro_mass'],
        engine.fixed_controls or engine.automatic_controls, solver.config['torque_gyro'],
        _prepare_only=True, _stationary=True,
        _governor_saturation=solver.config.get('stationary_governor_mode','pitch_stops')=='pitch_stops')
    requests = [enumerate_speeds(p, prep['seed'], i, velocity,
        solver.config['altitude_m'], solver.mass['nitro_mass'], solver.mass['cog'])
        for i in range(len(p['transmissions']))]
    return prep, requests, velocity, rates


class JointProblem:
    def __init__(self, solver, speed_kmh, load, attitude, request_indices=()):
        self.solver, self.speed, self.load = solver, speed_kmh / 3.6, load
        self.request_indices = tuple(request_indices)
        self.cache = {}
        self.prop_cache = {}
        self.aircraft_cache = {}
        self.governor_saturation=solver.config.get('stationary_governor_mode','pitch_stops')=='pitch_stops'
        self.calls = 0
        self.last = None
        self._last_key = None
        self.instantaneous = solver.config.get('experimental_instantaneous_aero', False)
        self.symmetric = solver.config.get('experimental_lateral_trim') == 'symmetric'
        self.attitude_indices = [0, 1, 3] if self.symmetric else list(range(5))
        self.history_start = len(self.attitude_indices)
        self.prop_start = self.history_start + (0 if self.instantaneous else 2)
        # Bank bounds are the declared physical admission rule. Other bounds
        # are candidate-search boxes, not domain exclusions.
        self.lower = [[-180., -90., -1., -1., -1.][i] for i in self.attitude_indices]
        self.upper = [[180., 90., 1., 1., 1.][i] for i in self.attitude_indices]
        self.steps = [[.002, .002, .0002, .0002, .0002][i] for i in self.attitude_indices]
        self.coordinate_scales = [[.01, .01, .0001, .0001, .0001][i] for i in self.attitude_indices]
        self.coordinate_names = [['alpha_deg','bank_deg','aileron','elevator','rudder'][i] for i in self.attitude_indices]
        self.initial = [attitude[i] for i in self.attitude_indices]
        self.history_domain = None
        if not self.instantaneous:
            # Numerical search limits only. Source/domain proof construction
            # is deliberately outside this frequently created production path.
            self.lower += [-1000., -1000.]
            self.upper += [1000., 1000.]
            self.steps += [.002, .002]
            self.coordinate_scales += [.01, .01]
            self.coordinate_names += ['left_wing_history_deg', 'right_wing_history_deg']
            self.initial += [attitude[0], attitude[0]]
        self.active, self.rows = [], []
        if solver.is_prop:
            prep, requests, _, _ = self.prepared(attitude)
            self.active = [j for j, path in enumerate(prep['paths'])
                           if path[0] != 'shaft' or not requests[path[1]]['governed']]
            self.rows = [j for j, path in enumerate(prep['paths']) if path[0] != 'pitch']
            self.paths = [prep['paths'][j] for j in self.active]
            for j in self.active:
                path = prep['paths'][j]
                # One coordinate spans all three governor regimes: below zero
                # it changes RPM at minimum pitch, [0,1] changes pitch at the
                # request, above one changes RPM at maximum pitch. No extra
                # shaft unknown or complementarity equation is introduced.
                if path[0]=='pitch' and self.governor_saturation:
                    shaft=prep['paths'].index(('shaft',path[1]))
                    self.lower.append(prep['bounds'][0][shaft]-prep['bounds'][1][shaft])
                    self.upper.append(1.+prep['bounds'][1][shaft])
                else:
                    self.lower.append(0. if path[0] == 'pitch' else prep['bounds'][0][j])
                    self.upper.append(1. if path[0] == 'pitch' else prep['bounds'][1][j])
                self.initial.append(.3 if path[0] == 'pitch' else
                                    (1. if path[2] < 2 else .15) if path[0] == 'flow'
                                    else prep['initial'][j])
                self.steps.append(.00001)
                self.coordinate_scales.append(.0001)
                self.coordinate_names.append(':'.join(map(str,
                    ('governor',path[1]) if path[0]=='pitch' and self.governor_saturation else path)))
        self.initial = np.clip(self.initial, self.lower, self.upper)
        # Coordinate conditioning belongs to the equation definition. Point,
        # conditional and boundary correction use the same physical scales.
        self.correction_scales = [[10., 30., .2, .2, .2][i] for i in self.attitude_indices]
        self.correction_scales += [10.] * (self.prop_start-self.history_start)
        self.correction_scales += [1.] * (len(self.initial)-self.prop_start)
        self.correction_step_limits = [[8., 20., .4, .4, .4][i] for i in self.attitude_indices]
        self.correction_step_limits += [8.] * (self.prop_start-self.history_start)
        self.correction_step_limits += [.3] * (len(self.initial)-self.prop_start)
        # Row conditioning follows the unchanged componentwise closure gates:
        # aircraft angular residuals are rate/.1; history residuals are deg/10.
        # These are numerical comparison scales, not new admission tolerances.
        self.correction_residual_scales = ([2e-4] * 2
            + [5e-4] * (self.history_start-2)
            + [2e-5] * (self.prop_start-self.history_start)
            + [RESIDUAL_TOLERANCE] * (len(self.initial)-self.prop_start))
        # Shaft/wake equations depend on alpha, bank and propulsion, never on
        # control commands or retained wing histories. This only suppresses
        # unnecessary nominal-stencil comparisons; actual samples stay intact.
        self.derivative_dependencies = np.ones((len(self.initial),len(self.initial)),dtype=bool)
        self.derivative_dependencies[self.prop_start:,2:self.prop_start] = False

    def attitude(self, x):
        out = [0.] * 5
        for i, value in zip(self.attitude_indices, x[:self.history_start]):out[i] = value
        return out

    def prepared(self, attitude):
        # Continuation varies speed, load and sideslip, hence rotor inflow and
        # prescribed rotation, even when attitude is unchanged. The evaluator owns this dependency;
        # callers must not have to invalidate the cache in the right order.
        # Controls and wing histories do not enter body flow or propeller
        # kinematics. Their derivative probes share this exact preparation.
        key = (self.speed, self.load, self.solver.sideslip_attitude_deg, *attitude[:2])
        if key not in self.cache:
            if len(self.cache) >= 8:self.cache.clear()
            self.cache[key] = prepare(self.solver, self.speed, self.load, attitude)
        return self.cache[key]

    def propulsion_at(self, x):
        """Evaluate only shaft torque and wake, independent of controls/history."""
        inner, prop = [], None
        attitude = self.attitude(x)
        if self.solver.is_prop:
            prep, requests, _, _ = self.prepared(attitude)
            q = prep['initial'].copy()
            assigned = {}
            for i, request in enumerate(requests):
                if request['governed']:
                    if i >= len(self.request_indices):
                        raise BranchUnavailable('Governed request branch is missing from the candidate')
                    index = self.request_indices[i]
                    if (isinstance(index, bool) or not isinstance(index, Integral)
                            or index < 0 or index >= len(request['candidates'])):
                        raise BranchUnavailable('Governed request branch changed or is inconsistent')
                    assigned[i] = request['candidates'][index]['shaft_speed']
                    q[prep['paths'].index(('shaft', i))] = assigned[i] / 300.
            q[self.active] = x[self.prop_start:]
            if self.governor_saturation:
                for j,path in enumerate(self.paths,self.prop_start):
                    if path[0]!='pitch':continue
                    shaft=path[1];coordinate=float(x[j]);target=assigned[shaft]
                    omega=target+300.*(coordinate if coordinate<0. else coordinate-1. if coordinate>1. else 0.)
                    index=prep['paths'].index(('shaft',shaft))
                    if not 300.*prep['bounds'][0][index]<=omega<=300.*prep['bounds'][1][index]:
                        raise BranchUnavailable('Governed shaft outside its speed search domain')
                    assigned[shaft]=omega;q[index]=omega/300.
                    q[prep['paths'].index(path)]=min(1.,max(0.,coordinate))
            key=(self.speed,self.load,self.solver.sideslip_attitude_deg,*attitude[:2],
                 tuple(self.request_indices),tuple(q),tuple(assigned.items()))
            if key not in self.prop_cache:
                if len(self.prop_cache)>=32:self.prop_cache.clear()
                self.prop_cache[key]=prep['sample'](q, requested_speeds=assigned)
            rr, prop = self.prop_cache[key]
            inner = rr[self.rows]
        return inner,prop

    def seed_propulsion(self, initial):
        """Close the small propulsion block once for an ordinary cold start."""
        from em_stationary_corrector import Corrector
        offset=self.prop_start
        def sample(z):
            q=np.array(initial,copy=True);q[offset:]=z
            residual,prop=self.propulsion_at(q)
            return dict(residual=residual,propulsion=prop)
        fit=Corrector(sample,(self.lower[offset:],self.upper[offset:]),self.steps[offset:],
            scale=self.correction_scales[offset:],step_limit=self.correction_step_limits[offset:],
            residual_scale=self.correction_residual_scales[offset:]).solve(
                np.asarray(initial)[offset:],max_nfev=20,numerical_target=PRODUCTION_NUMERICAL_TARGET,
                reuse_jacobian=True)
        q=np.array(initial,copy=True)
        if fit.success:q[offset:]=fit.x
        return q

    def residual(self, x):
        check_cancel()
        self.calls += 1
        attitude=self.attitude(x)
        inner,prop=self.propulsion_at(x)
        key=(self.speed,self.load,self.solver.sideslip_attitude_deg,*attitude)
        if key not in self.aircraft_cache:
            if len(self.aircraft_cache)>=8:self.aircraft_cache.clear()
            self.aircraft_cache[key]={}
        value = self.solver.operating_point(self.speed, self.load, attitude,
            propulsion_sample=prop,
            stationary_angles=None if self.instantaneous else x[self.history_start:self.prop_start],
            _search=True,_evaluation_cache=self.aircraft_cache[key])
        # Report physical pitch independently of the reduced governor
        # coordinate. Only the explicit exact-RPM diagnostic treats its stops
        # as domain boundaries; normal saturation continues through them.
        value['_stationary_pitch_fractions'] = {
            path[1]:min(1.,max(0.,float(x[j]))) for j,path in enumerate(getattr(self,'paths',()),self.prop_start)
            if path[0]=='pitch'}
        self.last = value
        aircraft = value['residual'][[0,1,4]] if self.symmetric else value['residual']
        history = [] if self.instantaneous else value['stationary_angle_residual'] / 10.
        residual = np.r_[aircraft, history, inner]
        self._last_key = self.sample_key(x)
        self._last_residual = residual
        return residual

    def sample_key(self, x):
        return (self.speed, self.load, self.solver.sideslip_attitude_deg,
                tuple(self.request_indices), tuple(map(float, x)))

    def value_at(self, x):
        """Reuse only the exact last evaluated state at the same condition."""
        if self._last_key != self.sample_key(x):self.residual(x)
        return self._last_residual, self.last

    def jacobian(self, x):
        return self.corrector(x).jacobian(x)

    def sample(self, x):
        residual = self.residual(x)
        return dict(self.last, residual=residual, _stationary_value=self.last,
                    _stationary_residual=residual)

    def derivative_branch(self, value):
        key=self.solver.derivative_branch(value)
        if self.solver.config.get('mach_curve_mode')=='stable':
            from em_mach_events import stable_piece_key
            # Stable polynomial arithmetic removes the need for same-bin Mach
            # stencils. Formula faces and aerodynamic pieces remain distinct.
            return (None,*key[1:],stable_piece_key(self.solver,value['result']['air']['mach']))
        return key

    def project(self, x, base):
        from em_mach_events import project_state, rough_mach
        mach = base['result']['air']['mach']
        if not rough_mach(self.solver, mach):return x
        bank = self.attitude_indices.index(1)
        attitude = project_state(self.solver, self.speed, self.load, self.attitude(x), mach,
                                 bank_bounds=(self.lower[bank],self.upper[bank]))
        q = x.copy()
        q[:self.history_start] = attitude[self.attitude_indices]
        return q

    def corrector(self, initial, *, offset=0, residual_scale=None):
        """The same local method also closes conditional histories/shafts."""
        if self.solver.config['trim_solver_mode'] == 'off':
            raise ValueError('Correction is off; evaluate the explicit trial state instead')
        from em_stationary_corrector import Corrector
        def expand(z):
            q = np.asarray(initial).copy();q[offset:] = z
            return q
        def evaluate(z):
            value = self.sample(expand(z))
            value['residual'] = value['residual'][offset:]
            return value
        return Corrector(evaluate, (self.lower[offset:], self.upper[offset:]),
            self.steps[offset:], branch=self.derivative_branch,
            dependencies=self.derivative_dependencies[offset:,offset:],
            project=(lambda q, base:self.project(q, base)) if offset==0 else None,
            scale=self.correction_scales[offset:], step_limit=self.correction_step_limits[offset:],
            residual_scale=(self.correction_residual_scales[offset:]
                            if residual_scale is None else residual_scale))

    def correct(self, initial, *, offset=0, max_nfev=120, trace=None, residual_scale=None,
                numerical_target=PRODUCTION_NUMERICAL_TARGET, warm_start=False):
        native=getattr(self.solver,'native_loop',None)
        if native is not None:
            handled,fit=native.joint_correct(self,initial,offset=offset,max_nfev=max_nfev,trace=trace,
                residual_scale=residual_scale,numerical_target=numerical_target,warm_start=warm_start)
            if handled:return fit
        cache = getattr(getattr(self, 'solver', None), '_stationary_predictors', None)
        key = (None if cache is None else (id(self.solver.engine), self.speed,
            self.solver.sideslip_attitude_deg, tuple(self.coordinate_names),
            tuple(self.request_indices), offset))
        previous = cache.get(key) if cache is not None else None
        initial = np.asarray(initial).copy()
        if previous is not None and warm_start:
            anchor = previous['coordinates']
            if (len(anchor) == len(initial) and np.max(abs(anchor-initial)/
                    self.correction_step_limits, initial=0.) <= 1.):
                initial[self.history_start:] = anchor[self.history_start:]
        fit = self.corrector(initial, offset=offset, residual_scale=residual_scale).solve(
            initial[offset:], max_nfev=max_nfev, trace=trace,
            numerical_target=numerical_target, reuse_jacobian=True,
            fast_stagnation=offset==0,
            predictor=None if previous is None else previous['predictor'])
        q = initial.copy();q[offset:] = fit.x
        # The returned sample was already evaluated, even when a derivative
        # probe or an earlier iterate won. Restore its owned physical record.
        if '_stationary_value' in fit.value:
            self.last = fit.value['_stationary_value']
            self._last_residual = fit.value['_stationary_residual']
            self._last_key = self.sample_key(q)
        if cache is not None and fit.predictor is not None:
            cache[key] = dict(predictor=fit.predictor, coordinates=q.copy())
            cache.move_to_end(key)
            while len(cache) > 64:cache.popitem(last=False)
        return fit

    def finish(self, x, *, detailed=False):
        residual, value = self.value_at(x)
        restrictions = []
        requests = []
        attitude = self.attitude(x)
        if self.solver.is_prop:
            prep, requests, velocity, rates = self.prepared(attitude)
            air, sound = fixed_air(velocity, self.solver.config['altitude_m'])
            p = self.solver.engine.properties
            for i, shaft in enumerate(p['transmissions']):
                state = value['propulsion']['state']
                for link in shaft['propellers']:
                    index = link['index']
                    pp = p['propellers'][index]['properties']
                    rotor = state['propellers'][index]
                    row = admission(pp, rotor['flow'], rotor['pitch'],
                        state['transmissions'][i]['omega'] * link['ratio'], air['density'], sound,
                        propeller_condition(pp, velocity, rates))
                    restrictions.append(dict(row, propeller=index, shaft=i))
        point = self.solver.certify(self.speed * 3.6, self.load, attitude, value,
                                    evaluations=self.calls, detailed=detailed)
        if abs(point['sideslip_deg']) > 2.:
            point['reasons'].append('sideslip limit')
        if any(not row['admitted'] for row in restrictions):
            point['reasons'].append('propeller admission')
        if (self.solver.is_prop and braking_rule(self.solver)
                and propeller_braking(self.solver.engine.properties, value['propulsion']['state'])):
            point['reasons'].append('propeller braking')
        point['local_valid'] = not point['reasons']
        point.update(valid=point['local_valid'],
            status='admitted' if point['local_valid'] else 'unresolved',
            validity_scope='Numerical stationary point with original physical admission',
            selection_status='numerical_candidate' if point['local_valid'] else 'unresolved',
            model_revision=REVISION,
            stationary_coordinates=list(map(float, x)),
            stationary_coordinate_names=self.coordinate_names,
            stationary_coordinate_scales=self.coordinate_scales,
            stationary_correction_residual_scales=self.correction_residual_scales,
            stationary_history_domain=self.history_domain,
            source_branch=(self.derivative_branch(value)[1:],
                           tuple(point.get('compressor_stages', ())), tuple(self.request_indices)),
            stationary_request_indices=list(self.request_indices),
            stationary_governor_mode='pitch_stops' if self.governor_saturation else 'exact_rpm',
            stationary_residual=residual.tolist(), propeller_admission=restrictions,
            request_records=requests, equilibrium_coverage_complete=False,
            governor_selection_certified=False, aircraft_selection_certified=False,
            numerical_error_certified=False, absence_certified=False,
            search_unresolved=None if point['local_valid'] else 'No locally admitted stationary point',
            search_bounds=[self.lower, self.upper])
        from em_model_experiments import description
        point.update(model_experiments=description(self.solver.config),
                     history_closure_required=not self.instantaneous,
                     symmetry_projection=value['result'].get('symmetry_projection'))
        point['mach_curve_mode']=self.solver.config.get('mach_curve_mode','native')
        if point['mach_curve_mode']=='stable':
            point['mach_curve_revision']=STABLE_POLAR_REVISION
        return point


def replay(solver, point, *, detailed=False):
    from em_model_experiments import description
    expected=description(solver.config)
    if solver.is_prop and point.get('stationary_governor_mode','exact_rpm')!=solver.config.get('stationary_governor_mode','pitch_stops'):
        raise ValueError('Saved point uses a different governor model; an explicit new correction is required')
    if point.get('model_experiments',dict(expected,instantaneous_aerodynamics=False,lateral_trim='full')) != expected:
        raise ValueError('Saved point uses different experimental model settings')
    mode=solver.config.get('mach_curve_mode','native')
    saved=point.get('mach_curve_mode')
    if ((saved is not None and saved!=mode) or
            (mode=='stable' and (saved!='stable' or point.get('mach_curve_revision')!=STABLE_POLAR_REVISION))):
        raise ValueError('Saved point uses different Mach arithmetic; an explicit new correction is required')
    view = solver.at_sideslip(point.get('sideslip_attitude_deg', 0.))
    problem = JointProblem(view, point['speed_kmh'], point['load_g'], point['solution'],
                           point.get('stationary_request_indices', ()))
    return problem.finish(np.asarray(point['stationary_coordinates']), detailed=detailed)


def remember_seed(solver, point):
    """Retain an admitted state, never a failed search or an admission shortcut."""
    if not point.get('local_valid'):return
    cache=solver._stationary_seeds
    key=(id(solver.engine),id(solver.model),point['sideslip_attitude_deg'],
         tuple(point['stationary_coordinate_names']),tuple(point['stationary_request_indices']),
         point['speed_kmh'],point['load_g'])
    cache[key]={name:(list(point[name]) if isinstance(point[name],list) else point[name]) for name in ('solution','stationary_coordinate_names',
        'stationary_coordinates','stationary_request_indices','speed_kmh','load_g')}
    cache.move_to_end(key)
    while len(cache)>64:cache.popitem(last=False)


def nearby_seed(solver, speed, load, slip):
    """Choose one local full-state hint; the ordinary corrector decides its use."""
    candidates=[]
    for key,point in solver._stationary_seeds.items():
        if key[:3] != (id(solver.engine),id(solver.model),slip):continue
        dv=abs(point['speed_kmh']-speed)/max(1.,speed)
        dn=abs(point['load_g']-load)/max(1.,load)
        if dv<=.25 and dn<=1.:candidates.append((dv*dv+dn*dn,point))
    if not candidates:return None
    point=min(candidates,key=lambda item:item[0])[1]
    from em_sampling import continuation_initial
    attitude=list(point['solution'])
    nominal=lambda n:math.degrees(math.acos(1./max(1.,n)))
    attitude[1]+=nominal(load)-nominal(point['load_g'])
    return continuation_initial(solver,point,attitude)


def solve(solver, speed_kmh, load, initial=None, detailed=False, exhaustive=True,
          refine=False, quick=False, *, max_nfev=120):
    started = time.monotonic()
    if not hasattr(solver, '_stationary_predictors'):
        from collections import OrderedDict
        solver._stationary_predictors = OrderedDict()
    if not hasattr(solver, '_stationary_seeds'):
        from collections import OrderedDict
        solver._stationary_seeds = OrderedDict()
    guess = None
    extended = exhaustive and not quick

    def ordinary_guess():
        nonlocal guess
        if guess is None:guess = list(solver.initial_guess(speed_kmh / 3.6, load))
        return guess

    def starts(slip):
        if initial is not None:
            supplied = list(initial['solution']) if isinstance(initial, dict) else list(initial[:5])
            yield supplied, initial, False
        cached=nearby_seed(solver,speed_kmh,load,slip)
        if cached is not None:yield list(cached['solution']),cached,False
        ordinary = ordinary_guess()
        yield ordinary, None, False
        if extended:
            # Try both ordinary aircraft starts before changing internal
            # pitch/wake seeds. Repeated seeds are skipped by the shared set.
            if solver.is_prop:
                if initial is not None:yield supplied, initial, True
                yield ordinary, None, True
            yield [ordinary[0], -ordinary[1], *ordinary[2:]], None, True

    candidates, attempts = [], []
    recovery_records = []
    recovery_budget = None

    def preference(point):
        return (0 if point['sideslip_attitude_deg'] == 0. else 1,
                abs(point['sideslip_deg']), -point['ps_mps'],
                tuple(point['stationary_coordinates']))

    def result():
        admitted = [p for p in candidates if p['local_valid']]
        for point in admitted:remember_seed(solver,point)
        pool = admitted or candidates
        def failed_preference(point):
            residual=np.asarray(point.get('stationary_residual',[float('inf')]))
            scales=np.asarray(point.get('stationary_correction_residual_scales',np.ones(len(residual))))
            error=float(np.max(abs(residual)/scales,initial=0.))
            return (not point.get('converged',False),error if math.isfinite(error) else float('inf'),preference(point))
        representative = min(pool, key=preference if admitted else failed_preference) if pool else dict(
            speed_kmh=float(speed_kmh), load_g=float(load), valid=False, local_valid=False,
            converged=False, reasons=['stationary search unresolved'], solution=ordinary_guess(),
            status='unresolved', absence_certified=False, ps_mps=None,
            model_revision=REVISION, search_unresolved='No candidate established; not an absence proof')
        output = dict(representative)
        output.update(stationary_candidates=candidates, stationary_attempts=attempts,
            propeller_branch_recovery=recovery_records,
            local_candidate_count=len(admitted), selected_candidates=[representative] if admitted else [],
            selection_status='numerical_selected' if admitted else 'unresolved',
            selection_scope='First admitted continuation or ordinary candidate; bounded failure recovery, without global selection certification',
            seconds=time.monotonic() - started)
        return output

    slip_trials = [solver.sideslip_attitude_deg]
    # Finite fallback sampling is explicitly incomplete; the reported beta
    # limit is enforced independently of these attitude-side search seeds.
    # sideslip_recovery=False is the zero-sideslip policy of the march chart.
    if (extended and solver.sideslip_attitude_deg == 0. and solver.config.get('experimental_lateral_trim','full')=='full'
            and solver.config.get('sideslip_recovery',True)):
        slip_trials += [-.5, .5, -1., 1., -2., 2.]
    for slip in slip_trials:
        view = solver.at_sideslip(slip)
        # Complete ordinary correction before considering another propeller
        # branch. A failed first seed must not trigger discovery ahead of a
        # valid supplied/ordinary continuation seed on the same request.
        pending_recovery, closed_requests, tried = {}, set(), set()
        for attitude, saved, alternate_seeds in starts(slip):
            try:
                # The first request uses this problem's own preparation. Do
                # not repeat engine/request preparation before construction.
                problem = JointProblem(view, speed_kmh, load, attitude)
                requests = problem.prepared(attitude)[1] if view.is_prop else []
                branches = list(itertools.product(*[
                    range(len(r['candidates'])) if r['governed'] else [0] for r in requests]))
                preferred = tuple(saved.get('stationary_request_indices', ())) if isinstance(saved, dict) else ()
                if (all(isinstance(index, Integral) and not isinstance(index, bool) for index in preferred)
                        and preferred in branches):
                    preferred = tuple(map(int, preferred))
                    branches.remove(preferred);branches.insert(0, preferred)
                if not branches:
                    attempts.append(dict(initial=attitude, sideslip=slip,
                        error='No consistent request at this trial attitude; aircraft coverage unresolved'))
            except (ValueError, ArithmeticError) as error:
                attempts.append(dict(initial=attitude, sideslip=slip, error=str(error)))
                continue
            for branch_index, branch in enumerate(branches):
                if branch_index:
                    problem = JointProblem(view, speed_kmh, load, attitude, branch)
                else:problem.request_indices = branch

                def seeds():
                    if (isinstance(saved, dict) and tuple(saved.get('stationary_request_indices',()))==branch
                            and len(saved.get('stationary_coordinates', [])) == len(problem.initial)
                            and saved.get('stationary_coordinate_names',problem.coordinate_names)==problem.coordinate_names):
                        yield np.clip(saved['stationary_coordinates'], problem.lower, problem.upper)
                    else:yield problem.initial.copy()
                    if alternate_seeds and view.is_prop:
                        for fraction, axial in ((.05, -.5), (.85, 2.)):
                            seed = problem.initial.copy()
                            for j, path in enumerate(problem.paths, problem.prop_start):
                                if path[0] == 'pitch':seed[j] = fraction
                                elif path[0] == 'flow' and path[2] < 2:seed[j] = axial
                            yield seed

                for seed in seeds():
                    identity = (branch, tuple(map(float,seed)))
                    if identity in tried:continue
                    tried.add(identity)
                    try:
                        governed = any(path[0] == 'pitch' for path in getattr(problem,'paths',()))
                        # Quick chart probes get a dimension-scaled local
                        # correction budget. Failure remains unresolved and the
                        # sampler can recover/continue the branch; do not spend
                        # a full difficult-root budget on every rejected trial.
                        budget=min(max_nfev,4*len(seed)+1) if quick else max_nfev
                        if (view.is_prop and saved is None and not alternate_seeds
                                and hasattr(problem,'seed_propulsion') and np.array_equal(seed,problem.initial)):
                            seed=problem.seed_propulsion(seed)
                        fit = problem.correct(seed, max_nfev=budget,
                            warm_start=not isinstance(saved, dict) and np.array_equal(seed, problem.initial))
                        point = problem.finish(fit.x, detailed=detailed)
                        if governed:
                            if problem.last['propulsion']['converged']:
                                closed_requests.add(branch)
                            else:
                                score = float(np.linalg.norm(fit.fun[problem.prop_start:]))
                                previous = pending_recovery.get(branch)
                                if previous is None or score < previous[0]:
                                    pending_recovery[branch] = (score, problem, fit)
                        point['numerical_message'] = str(fit.message)
                        point['correction_diagnostics'] = {
                            name:getattr(fit,name,None) for name in
                            ('termination','success','nfev','evaluations','derivative_evaluations',
                             'iterations','accepted_steps','contractions','coordinate_polls','trust_radius',
                             'return_metric','return_score','closure_ratio','merit_norm','numerical_target',
                             'jacobian_reuses','jacobian_refreshes')}
                        candidates.append(point)
                        attempts.append(dict(initial=seed.tolist(), sideslip=slip,
                            requests=list(branch), calls=problem.calls,
                            local_valid=point['local_valid'], reasons=point['reasons']))
                        if point['local_valid']:return result()
                        if not extended and point.get('converged'):
                            from em_pitch_response import PHYSICAL_REASONS
                            if point['reasons'] and set(point['reasons'])<=PHYSICAL_REASONS:
                                # A local load probe already has the balanced
                                # rejection needed to locate its branch limit.
                                # Alternate-root discovery is an explicit task.
                                return result()
                    except (ValueError, ArithmeticError, np.linalg.LinAlgError) as error:
                        attempts.append(dict(initial=seed.tolist(), sideslip=slip,
                                             requests=list(branch), error=str(error)))
        if pending_recovery and not quick:
            from em_prop_branch_recovery import Budget, recover, warnings
            if recovery_budget is None:recovery_budget = Budget()
            for branch in sorted(set(pending_recovery)-closed_requests):
                _, problem, fit = pending_recovery[branch]
                try:
                    try:hints=warnings(problem,fit)
                    except (ValueError,ArithmeticError,np.linalg.LinAlgError) as error:
                        hints=[dict(warning='derivative unavailable',reason=str(error))]
                    extra, record = recover(problem,fit.x,detailed=detailed,budget=recovery_budget)
                    candidates.extend(extra)
                    recovery_records.append(dict(record,sideslip=slip,requests=list(branch),early_warnings=hints))
                    good=[p for p in extra if p['local_valid']]
                    if good:return result()
                except (ValueError,ArithmeticError,np.linalg.LinAlgError) as error:
                    recovery_records.append(dict(status='unresolved',reason=str(error),
                        sideslip=slip,requests=list(branch),coverage_complete=False))
    return result()
