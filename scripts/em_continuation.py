import math

import numpy as np

from em_cancellation import check as check_cancel


class TrimCurve:

    scale = np.array([10., 30., .3, .3, .3, 10.])
    difference = np.array([.002, .002, .0002, .0002, .0002, .0002]) / scale
    # Numerical placement of a chart section in scaled coordinates. This is
    # not an equilibrium equation or a physical boundary.
    stationary_plane_tolerance = 1e-5

    def __init__(self, solver, speed, seed=None):
        self.solver = solver
        self.speed = float(speed)
        self.stationary = (solver.config.get('trim_solver_mode') == 'stationary'
                           and hasattr(solver, 'core'))
        self._stationary_seed = seed


        if not self.stationary and solver.is_prop and solver.engine.automatic:
            self.solver = solver.with_prop_controls(
                solver.engine.fixed_controls or solver.engine.automatic_controls)
            from em_data import clone
            if seed is not None and seed.get('_propulsion_seed') is not None:
                self.solver.engine._reference = clone(seed['_propulsion_seed'])
            self.solver.engine._warm = clone(self.solver.engine._reference)
            self.solver.engine.force_canonical = False
        self.matrix = None
        self.matrix_at = None
        self.branch = None
        self.evaluations = 0
        self.last_correction = None
        self.full_phase = True
        self.prescribed_load = None

    @classmethod
    def encode(cls, solution, load):
        u = math.sqrt(max(0., (load - 1.) * (load + 1.)))
        z = np.r_[solution, u] / cls.scale
        z[1] -= math.degrees(math.atan(u)) / cls.scale[1]
        return z

    def decode(self, z):
        q = z * self.scale
        x = q[:5].copy()
        x[1] += math.degrees(math.atan(q[5]))


        load = self.prescribed_load if self.prescribed_load is not None else math.hypot(1., q[5])
        return x, load

    def state(self, point):
        return self.encode(point['solution'], point['load_g'])

    def inside(self, z):
        from em_load_limits import ceiling
        x, load = self.decode(z)
        lo, hi = self.solver.trim_bounds
        return (z[5] >= 0. and z[5] <= 6.4 and load<=ceiling(self.solver,self.speed)+1e-10 and
                bool(np.all(x >= lo) and np.all(x <= hi)))

    def same_mach_state(self, z, mach, normal=None):
        pass


        from em_solver import turn_geometry, turn_direction
        from air_state import cache, world_to_body_air
        from component_assembly import f32
        x, load = self.decode(z)
        speed = self.speed / 3.6
        angle = math.radians(x[0])
        air = cache([f32(speed * math.cos(angle)), f32(-speed * math.sin(angle)), 0.],
            self.solver.config['altitude_m'])
        step = max(abs(float(np.spacing(np.float32(x[1])))), 1e-7)
        compensate = None
        if normal is not None and abs(normal[1]) > 1e-12:
            compensate = 2 + int(np.argmax(abs(normal[2:5])))
            if abs(normal[compensate]) < 1e-10:
                return z
        for offset in (0., 1., -1., 2., -2., 4., -4., 8., -8.):
            q = z.copy()
            q[1] += offset * step / self.scale[1]
            if compensate is not None:


                q[compensate] -= normal[1] * (q[1] - z[1]) / normal[compensate]
            trial_x, _ = self.decode(q)
            if not self.inside(q):
                continue
            geometry = turn_geometry(trial_x[0], trial_x[1], speed, load, self.solver.dt,
                air['ias_u'], self.solver.sideslip_attitude_deg, turn_direction(self.solver.config))
            velocity = world_to_body_air(geometry['quaternion'], [speed, 0., 0.])
            if cache(velocity, self.solver.config['altitude_m'])['mach'] == mach:
                return q
        return z

    def value(self, z, frozen=None, retain_phases=False):
        check_cancel()
        x, load = self.decode(z)
        if frozen is None:
            self.evaluations += 1
        return self.solver.operating_point(self.speed / 3.6, load, x,
            propulsion_override=frozen if not retain_phases else None,
            propulsion_sample=frozen if retain_phases else None,
            cycle_seconds=20., phase_certificate=self.full_phase)

    @staticmethod
    def closed(value):


        return (np.isfinite(value['residual']).all() and np.isfinite(value['rate_residual']).all() and
                value['force_error_g'] + value['force_mean_uncertainty_g'] <= 2e-4 and
                max(abs(value['rate_residual']) + value['angular_mean_uncertainty_rad_s2']) <= 5e-5 and
                value['history_error'] <= 2e-4)

    def jacobian(self, z, value, coupled=False, event=None):
        frozen = (value['propulsion'] if self.solver.is_prop and not coupled and not (
            self.solver.engine.quasi_steady and self.solver.config.get('aircraft_trim_mode')=='quasi_steady') else None)


        retain_phases = False
        base = self.value(z, frozen, retain_phases) if frozen is not None else value
        from em_trim_numerics import jacobian, DerivativeUnavailable
        from em_mach_events import rough_mach
        mach=base['result']['air']['mach']
        try:
            return jacobian(z, base, lambda q:self.value(q, frozen, retain_phases),
                self.solver.derivative_branch, self.difference,
                (1., -1., .5, -.5, 2., -2., .1, -.1, .01, -.01), inside=self.inside,
                project=(lambda q:self.same_mach_state(q,mach)) if event is not None or rough_mach(self.solver,mach) else None,
                event=event, actual_step=True)
        except DerivativeUnavailable as failure:
            self.last_correction = dict(outcome='derivative branch unavailable', axis=failure.axis)
            return None


    def correct(self, predicted, normal=None, *, detailed=False, event=None, event_tolerance=1e-6,
                max_iterations=12, max_refreshes=3, max_evaluations=None, extend_settling=True,
                prescribed_load=None):
        predicted = np.asarray(predicted, dtype=float)
        self.prescribed_load = prescribed_load
        normal = np.asarray(normal if normal is not None else np.eye(6)[5], dtype=float)
        length = np.linalg.norm(normal)
        if not length or not self.inside(predicted):
            return None
        normal = normal / length
        if self.stationary:
            return self._stationary_correct(predicted, normal, event=event,
                event_tolerance=event_tolerance, max_iterations=max_iterations,
                max_evaluations=max_evaluations, prescribed_load=prescribed_load)
        self.full_phase = not self.solver.is_prop
        if self.solver.is_prop:
            self.solver.engine.force_canonical = False
        z = predicted.copy()
        value = self.value(z)
        matrix = self.matrix.copy() if (event is None and self.matrix is not None and
            self.branch == self.solver.derivative_branch(value) and
            np.linalg.norm(z - self.matrix_at) < .5) else None
        extra = (lambda q, v: float(normal.dot(q - predicted))) if event is None else (lambda q, v: event(v))
        tolerance = 1e-8 if event is None else event_tolerance
        refreshes = 0
        settling_extended = False
        started = self.evaluations - 1
        for iteration in range(max_iterations + 1):
            self.last_correction = dict(iterations=iteration, refreshes=refreshes,
                force_error_g=value['force_error_g'],
                angular_error_rad_s2=float(max(abs(value['rate_residual']))),
                force_uncertainty_g=value['force_mean_uncertainty_g'],
                angular_uncertainty_rad_s2=float(max(value['angular_mean_uncertainty_rad_s2'])))
            plane = extra(z, value)
            if self.closed(value) and abs(plane) < tolerance and not self.full_phase:


                x, load = self.decode(z)
                value = self.solver.operating_point(self.speed / 3.6, load, x,
                    cycle_seconds=20., phase_certificate=True)
                self.evaluations += 1
                self.full_phase = True
                plane = extra(z, value)
                if value['propulsion']['converged'] and not self.closed(value):
                    from em_data import clone
                    self.solver.engine._reference = clone(value['propulsion']['state'])
                    self.solver.engine.force_canonical = True
                    value = self.value(z)
                    plane = extra(z, value)
                    matrix = None
            needs_settling = (self.solver.is_prop and (not value['propulsion']['converged'] or
                value['force_mean_uncertainty_g'] > 1e-4 or
                max(value['angular_mean_uncertainty_rad_s2']) > 2.5e-5))
            if (value['force_error_g'] < 2e-4 and max(abs(value['rate_residual'])) < 5e-5
                    and abs(plane) < tolerance and needs_settling and not settling_extended and extend_settling):


                from em_data import clone
                self.solver.engine._reference = clone(value['propulsion']['state'])
                self.solver.engine._warm = clone(value['propulsion']['state'])
                x, load = self.decode(z)
                value = self.solver.operating_point(self.speed / 3.6, load, x,
                    cycle_seconds=180., phase_certificate=True)
                self.evaluations += 1
                self.full_phase = True
                if value['propulsion']['converged']:
                    self.solver.engine.search_cycle_seconds = max(
                        self.solver.engine.search_cycle_seconds,
                        min(180., value['propulsion'].get('simulated_seconds', 20.)))
                settling_extended = True
                matrix = None
                plane = extra(z, value)
            if self.closed(value) and abs(plane) < tolerance:
                x, load = self.decode(z)
                point = self.solver.certify(self.speed, load, x, value,
                    evaluations=self.evaluations - started, detailed=detailed)
                self.last_correction['reasons'] = point['reasons']
                point['trim_algorithm'] = 'scaled predictor-corrector continuation'
                if matrix is not None and event is None:
                    self.matrix = matrix.copy()
                    self.matrix_at = z.copy()
                    self.branch = self.solver.derivative_branch(value)
                if point['valid'] and point.get('_propulsion_seed') is not None:
                    from em_data import clone
                    self.solver.engine._reference = clone(point['_propulsion_seed'])
                return point
            if (iteration == max_iterations or
                    max_evaluations is not None and self.evaluations - started >= max_evaluations):
                return None
            if matrix is None:
                if refreshes >= max_refreshes:
                    return None
                matrix = self.jacobian(z, value,
                    coupled=refreshes == 2 and not (self.full_phase and event is not None), event=event)
                refreshes += 1
                if matrix is None:
                    return None
            residual = np.r_[value['residual'], plane]
            system = np.vstack((matrix, normal)) if event is None else matrix
            try:
                delta = np.linalg.solve(system, -residual)
            except np.linalg.LinAlgError:
                return None
            if not np.isfinite(delta).all():
                return None
            delta /= max(1., float(np.max(abs(delta))) / .5)
            advanced = False
            for factor in (1., .5, .25):
                q = z + factor * delta
                q = self.same_mach_state(q, value['result']['air']['mach'],
                    normal=normal if event is None else None)
                if not self.inside(q):
                    continue
                trial = self.value(q)
                r = np.r_[trial['residual'], extra(q, trial)]
                if np.linalg.norm(r) < np.linalg.norm(residual):
                    dx = q - z
                    denominator = float(dx.dot(dx))
                    if (denominator > 1e-20 and self.solver.derivative_branch(trial) ==
                            self.solver.derivative_branch(value)):
                        difference = trial['residual'] - value['residual'] if event is None else r - residual
                        matrix += np.outer(difference -
                            matrix.dot(dx), dx) / denominator


                    z, value = q, trial
                    advanced = True
                    break
            if not advanced:
                matrix = None
        return None

    def _stationary_correct(self, predicted, normal, *, event, event_tolerance,
                            max_iterations, max_evaluations, prescribed_load):
        """Retain the baseline continuation plane, share joint correction.

        The six-coordinate chart parameterization is a sampling policy. Its
        history/shaft unknowns stay in JointProblem, and it must not invoke
        the legacy continuation's frozen governor, Broyden or settling paths.
        """
        from em_stationary_boundary import Curve
        attitude, load = self.decode(predicted)
        seed = dict(self._stationary_seed or {}, solution=attitude.tolist(),
                    load_g=load, speed_kmh=self.speed)
        if event is None:
            def constraint(value):
                state = self.encode(value['equilibrium_coordinates'],
                                    value['equilibrium_load_g'])
                return float(normal.dot(state-predicted))
            constraint.stationary_plane_normal=np.asarray(normal)
            constraint.stationary_plane_scale=self.scale
            tolerance = self.stationary_plane_tolerance
        else:
            constraint, tolerance = event, event_tolerance
        curve = getattr(self, '_stationary_curve', None)
        if curve is None:
            curve = self._stationary_curve = Curve(self.solver, self.speed)
        previous_calls = curve.calls
        curve.unverified_candidates.clear()
        point = curve.correct(seed, load=prescribed_load,
            event=constraint if prescribed_load is None else None,
            max_nfev=max(1, 4*max_iterations), max_evaluations=max_evaluations,
            event_tolerance=tolerance)
        self.evaluations += curve.calls-previous_calls
        self.last_correction = getattr(curve, 'last_correction', None)
        if curve.unverified_candidates:
            self.solver.__dict__.setdefault('stationary_boundary_evidence', []).extend(
                curve.unverified_candidates)
        if point is not None:
            point['trim_algorithm'] = 'shared stationary correction of baseline continuation plane'
            if point['valid']:
                self._stationary_seed = point
        return point

    def limit(self, initial, kind, **options):
        axis = None
        if kind == 'stall':
            event = lambda v: (v['stall_margin'] - .002) / 10.
        elif kind == 'wing force':
            event = lambda v: max(v['maximum_wing_load_ratios']) - .99995
        elif kind == 'control':
            _, axis, side = min((abs(command - bound), i, j)
                for i, (command, bounds) in enumerate(zip(initial['commands'], initial['control_bounds']))
                for j, bound in enumerate(bounds))
            def event(v):
                bounds = (self.solver.instructor_at(v, self.speed / 3.6, v['equilibrium_load_g'])['control_bounds']
                    if self.solver.config['instructor'] else v['allocation']['bounds'])
                return (1. if side == 0 else -1.) * (v['allocation']['commands'][axis] - bounds[axis][side]) - 3e-5
        elif kind == 'Instructor pitch':
            def event(v):
                ctl = self.solver.instructor_at(v, self.speed / 3.6, v['equilibrium_load_g'])
                return ctl['envelope_margin'] - 2e-6 if ctl['converged'] else float('nan')
        else:
            return None
        point = self.correct(self.state(initial), event=event,
            event_tolerance=2e-5 if kind == 'stall' else 2e-7 if kind == 'Instructor pitch' else 1e-6, **options)
        if point is None or not point['valid']:
            return None
        if (abs(point['alpha_deg']-initial['alpha_deg'])>1.5 or
                max(abs(a-b) for a,b in zip(point['solution'][2:],initial['solution'][2:]))>.2):
            return None
        point['envelope_limit'] = dict(kind=kind, axis=axis + 2 if axis is not None else None,
            limiting_load_g=point['load_g'], evaluations=point['evaluations'],
            constraint_residual=(point['stall_margin_deg']-.002)/10. if kind=='stall' else
                max(point['wing_load_ratios'])-.99995 if kind=='wing force' else
                point['instructor']['envelope_margin']-2e-6 if kind=='Instructor pitch' else
                point['authority_margin']-3e-5,
            method='simultaneous force, moment and active-constraint corrector')
        return point

    def at_load(self, load, initial=None, **options):
        guess = initial if initial is not None else self.solver.initial_guess(self.speed / 3.6, load)
        return self.correct(self.encode(guess, load), np.eye(6)[5], prescribed_load=float(load), **options)

    def between(self, left, right, fraction=.5, **options):
        a, b = self.state(left), self.state(right)
        point = self.correct(a + fraction * (b - a), b - a, **options)
        if point is None or not left['load_g'] < point['load_g'] < right['load_g']:
            return None
        if (left['alpha_deg'] < right['alpha_deg'] and
                not left['alpha_deg'] < point['alpha_deg'] < right['alpha_deg']):
            return None
        if np.linalg.norm(self.state(point) - (a + fraction * (b - a))) > max(.025, .5 * np.linalg.norm(b - a)):
            return None
        return point
