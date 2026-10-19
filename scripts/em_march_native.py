"""Compiled evaluate-and-correct loop of the march chart (speed plan S2/S3).

The native module (scripts/march_native, built by build_march_native.py into
.native_march/) reproduces the stationary evaluation bit for bit and the
corrections to rounding. It is a per-solver option: ``attach(solver)`` sets
``solver.native_loop``; the production code checks that attribute at five
places (Curve.correct, TrimSolver.operating_point, TrimSolver.instructor_at,
JointProblem.correct, stationary_frozen_point) and uses this module only for
the calls it covers. Solvers without the attribute (the baseline chart, the
point API) never reach it. Copies and sideslip views of an attached solver
share it.

Covered (everything else stays on the Python path, unchanged):
- Curve.correct with exactly one of alpha / load / elevator / the direct SEP
  event, and the stall, wing-force and non-Instructor control events, at zero
  sideslip on a supported solver (stationary trim, full model, discrete
  aircraft trim, stable Mach mode; jets, rockets and the ported propeller and
  piston-engine subset);
- explicit-history operating points, frozen-propulsion history closures,
  production point corrections (JointProblem.correct at offset 0) and the
  Instructor admission.

Safety:
- the module is used only while the Python sources it was verified against
  are unchanged (march_native/verified.json: hash of the transliterated
  modules and of the native sources) and the built binary matches its
  manifest; otherwise every call stays Python (same chart, slower) and
  ``status()`` says why;
- native event corrections additionally require the Curve.events source the
  event residuals were transliterated from (march_native/events.sha256);
- an unexpected native error in an event correction sends that call to Python.

``WT_MARCH_NATIVE=0`` disables the module. ``WT_MARCH_NATIVE_VERIFY=1`` runs
the shadow checks (native and Python finish rows, explicit operating points,
Instructor admission and event corrections compared; the Python results are
returned) used by verify_march_native.py.
"""
from __future__ import annotations

import ast
import hashlib
import json
import os
import sys
import sysconfig
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT/'scripts'/'march_native'
DIRECTORY = ROOT/'.native_march'
MODULE = 'march_native'
STAMP = SOURCE/'verified.json'
# Python modules the native code transliterates or calls into (pxi headers);
# the verified source closure adds their top-level imports.
ROOTS = ('polar_f32', 'wing_model', 'wing_stages', 'tail_model', 'component_stages', 'downwash', 'component_assembly',
         'body_dynamics', 'air_state', 'aero_helpers', 'control_mixer', 'primary_controls', 'em_operating', 'em_solver',
         'jet_nominal', 'jet_model', 'jet_nozzle', 'rocket_em', 'kinematics', 'em_stationary_corrector',
         'em_trim_numerics', 'em_stationary_boundary', 'em_stationary', 'em_stationary_surface', 'em_stationary_baseline',
         'em_stationary_chart', 'instructor_chart_inputs', 'windows_instructor_source', 'instructor_keyboard',
         'instructor_protection', 'instructor_predictor_inputs', 'instructor_pitch_predictor', 'instructor_aoa_balance',
         'instructor_reduced', 'instructor_autotrim', 'instructor_trim_equilibrium', 'instructor_static_trim',
         'instructor_steady_aoa', 'instructor_aoa', 'instructor_envelope', 'prop_equilibrium', 'prop_quasisteady',
         'piston_general', 'piston_compressor', 'prop_requests', 'prop_admission', 'em_pitch_response', 'polar_runtime',
         'polar_model', 'mach_cubic', 'flap_model', 'wing_sweep', 'engine_supply', 'structural_limits',
         'em_model_experiments', 'em_accuracy_policy', 'aircraft_model', 'mass_model')
_STATE = dict(module=None, status=None, events=None)


# ----- source identity --------------------------------------------------------
def _text(path):
    return path.read_bytes().replace(b'\r\n', b'\n')


def native_sources():
    return sorted(p for p in SOURCE.iterdir() if p.suffix in ('.pyx', '.pxi'))


def native_digest():
    digest = hashlib.sha256(b'march-native-sources-v1')
    for path in native_sources():
        digest.update(path.name.encode());digest.update(_text(path))
    return digest.hexdigest()


def closure(roots=ROOTS):
    """ROOTS plus their top-level imports within scripts/ (sorted names)."""
    folder = ROOT/'scripts';seen = set();todo = list(roots)
    while todo:
        name = todo.pop();path = folder/(name+'.py')
        if name in seen or not path.is_file() or name.startswith('em_march'):continue
        seen.add(name)
        for node in ast.parse(_text(path).decode('utf-8')).body:
            if isinstance(node, ast.ImportFrom) and node.module and not node.level:todo.append(node.module.split('.')[0])
            elif isinstance(node, ast.Import):todo.extend(a.name.split('.')[0] for a in node.names)
    return sorted(seen)


def python_digest(modules):
    digest = hashlib.sha256(b'march-native-python-v1')
    for name in modules:
        path = ROOT/'scripts'/(name+'.py')
        digest.update(name.encode());digest.update(_text(path) if path.is_file() else b'<missing>')
    return digest.hexdigest()


def stamp_record(verification):
    modules = closure()
    return dict(native_sha256=native_digest(), python_modules=modules, python_sha256=python_digest(modules),
                verification=verification)


# ----- build / load -------------------------------------------------------------
def build_signature():
    """Identity of a build: native sources, compiler flags, interpreter and platform."""
    import platform
    from build_march_native import COMPILE_ARGS
    digest = hashlib.sha256(native_digest().encode())
    digest.update(' '.join(COMPILE_ARGS).encode());digest.update(sys.version.encode())
    digest.update((sys.platform+platform.machine()).encode())
    return digest.hexdigest()


def _manifest():
    """(library directory, manifest) of a valid build, or (None, reason)."""
    suffix = sysconfig.get_config_var('EXT_SUFFIX')
    portable = DIRECTORY/'portable.json'
    try:
        if portable.exists():
            manifest = json.loads(portable.read_text())
            if manifest['native_sha256'] != native_digest() or manifest['abi'] != sys.implementation.cache_tag:
                return None, 'packaged native module does not match this source'
            library = DIRECTORY/'lib'
        else:
            manifest = json.loads((DIRECTORY/'manifest.json').read_text())
            if manifest['signature'] != build_signature():return None, 'native build is stale'
            library = (DIRECTORY/manifest['library']).resolve()
            if not library.is_relative_to(DIRECTORY.resolve()):return None, 'invalid native library path'
        name = MODULE+suffix
        if set(manifest['binaries']) != {name}:return None, 'native build has unexpected files'
        if hashlib.sha256((library/name).read_bytes()).hexdigest() != manifest['binaries'][name]:
            return None, 'native binary does not match its manifest'
        return library, manifest
    except (OSError, ValueError, KeyError, TypeError):
        return None, 'native module not built'


def load(build=True):
    """The compiled module (or None) and a status string; decided once per process."""
    if _STATE['status'] is not None:return _STATE['module'], _STATE['status']
    module, status = None, None
    verifying = os.environ.get('WT_MARCH_NATIVE_VERIFY') == '1'
    if os.environ.get('WT_MARCH_NATIVE', '1') == '0':status = 'disabled by WT_MARCH_NATIVE=0'
    # WT_MARCH_NATIVE_UNSTAMPED=1: development builds before re-verification (never in releases).
    if status is None and not verifying and os.environ.get('WT_MARCH_NATIVE_UNSTAMPED') != '1':
        try:
            stamp = json.loads(STAMP.read_text())
            if stamp['native_sha256'] != native_digest():status = 'disabled: native sources changed since verification'
            elif python_digest(stamp['python_modules']) != stamp['python_sha256']:
                status = 'disabled: model sources changed since native verification (run verify_march_native.py)'
        except (OSError, ValueError, KeyError, TypeError):
            status = 'disabled: no native verification record'
    if status is None:
        library, manifest = _manifest()
        if library is None and build and not (DIRECTORY/'portable.json').exists():
            built = _build()
            library, manifest = _manifest()
            if library is None and built:manifest = built
        if library is None:status = 'unavailable: '+str(manifest)
    if status is None:
        import importlib.util
        try:
            spec = importlib.util.spec_from_file_location(MODULE, library/(MODULE+sysconfig.get_config_var('EXT_SUFFIX')))
            module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
            status = 'enabled (verify mode)' if verifying else 'enabled'
        except (ImportError, OSError) as error:
            module, status = None, 'unavailable: '+type(error).__name__+': '+str(error)[:120]
    _STATE.update(module=module, status=status)
    if module is None and status != 'disabled by WT_MARCH_NATIVE=0':
        print('March chart: compiled loop '+status+'; using the Python loop (same results, slower).', file=sys.stderr, flush=True)
    return module, status


def _build():
    """Build once in a subprocess (development checkouts); returns a failure reason or None."""
    import subprocess
    try:
        done = subprocess.run([sys.executable, str(ROOT/'scripts'/'build_march_native.py')], cwd=str(ROOT),
                              stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True, timeout=1800)
    except (OSError, subprocess.SubprocessError) as error:
        return 'build failed: '+str(error)[:200]
    return None if done.returncode == 0 else 'build failed: '+done.stderr[-300:]


def status():
    return load(build=False)[1] if _STATE['status'] is None else _STATE['status']


def events_supported():
    """Native event corrections need the Curve.events source they were transliterated from."""
    if _STATE['events'] is None:
        import inspect
        import em_stationary_boundary as B
        digest = hashlib.sha256(inspect.getsource(B.Curve.events).encode()).hexdigest()
        expected = (SOURCE/'events.sha256').read_text().strip()
        codes = {}
        if digest == expected:
            import types
            for const in B.Curve.events.__code__.co_consts:
                if not isinstance(const, types.CodeType):continue
                if const.co_name == '<lambda>' and 'stall_margin' in const.co_consts:codes['stall'] = const
                elif const.co_name == '<lambda>' and 'maximum_wing_load_ratios' in const.co_consts:codes['wing force'] = const
                elif const.co_name == 'control':codes['control'] = const
            if set(codes) != {'stall', 'wing force', 'control'}:codes = {}
        _STATE['events'] = codes
    return _STATE['events']


# ----- per-solver option ----------------------------------------------------------
def supported(solver):
    cfg = solver.config
    return (cfg.get('trim_solver_mode') == 'stationary' and not cfg.get('experimental_instantaneous_aero', False)
            and cfg.get('experimental_lateral_trim', 'full') == 'full' and cfg.get('aircraft_trim_mode', 'discrete') == 'discrete'
            and solver.model.get('mach_curve_mode') == 'stable' and getattr(solver, 'sideslip_attitude_deg', 0.) == 0.)


def attach(solver, verify=None):
    """Give ``solver`` (and its later copies) the compiled loop; returns the status string."""
    module, state = load()
    if module is None:return state
    if getattr(solver, 'native_loop', None) is None:
        solver.native_loop = NativeLoop(module, verify=os.environ.get('WT_MARCH_NATIVE_VERIFY') == '1' if verify is None else verify)
    return state


def detach(solver):
    solver.__dict__.pop('native_loop', None)


class NativeLoop:
    """Native objects of one solver model, built on first use (the S2/S3 install() hooks)."""

    def __init__(self, module, verify=False):
        self.m = module
        self.verify = bool(verify)
        self.entry = None       # (CurveCorrection, FrozenPoint) or False when unsupported
        self.instructor = None  # native Instructor, False when unavailable
        self.unsupported = {}
        self.stats = dict(native_corrections=0, python_corrections=0, native_events=0, python_events=0, native_frozen=0,
                          python_frozen=0, native_points=0, python_points=0, native_point_solves=0, python_point_solves=0,
                          native_instructor=0, python_instructor=0, native_event_errors={}, event_kinds={})

    # ----- native objects ---------------------------------------------------
    def native(self, solver):
        if not supported(solver):return None
        if self.entry is None:
            s2 = self.m
            model = s2.Model(solver)
            cc = s2.CurveCorrection(model)
            self.entry = False
            if solver.is_prop:
                try:
                    pm = s2.PropModel(solver)
                except s2.PropUnsupported as error:
                    self.unsupported['propulsion'] = str(error)[:120]
                    return None
                from em_stationary import JointProblem
                layout = JointProblem(solver.core() if callable(getattr(solver, 'core', None)) else solver, 300., 1., [5., 0., 0., 0., 0.])
                pm.set_joint_layout(layout.active, layout.rows)
                cc.set_prop_layout(pm, layout.coordinate_names, layout.lower, layout.upper, layout.initial)
            cc.finisher = s2.Finisher(model, solver)
            if solver.is_prop:
                try:
                    cc.finisher.set_prop(pm, cc.layout)
                except s2.PropUnsupported as error:
                    self.unsupported['finish'] = str(error)[:120];cc.finisher = None
            cc.verify = int(self.verify)
            self.entry = (cc, s2.FrozenPoint(model))
            if cc.finisher is not None:
                cc.finisher.loop_stats = self.stats
                # Lean Instructor admission (S5); verify mode keeps the shadow-checked Python route.
                if not self.verify:cc.finisher.instructor = self.instructor_for(solver)
        return self.entry or None

    def instructor_for(self, solver):
        if self.instructor is None:
            self.instructor = False
            entry = self.native(solver)
            if entry is not None and solver.config.get('instructor'):
                try:
                    self.instructor = self.m.Instructor(entry[0].model, solver)
                except self.m.InstructorUnsupported as error:
                    self.unsupported['instructor'] = str(error)[:120]
        return self.instructor or None

    # ----- hooks: each returns (handled, value) -------------------------------
    def correct(self, curve, seed, *, alpha=None, elevator=None, event=None, events=None, load=None, max_nfev=40,
                max_evaluations=None, event_tolerance=2e-6):
        from em_stationary import BranchUnavailable
        from em_stationary_boundary import TrialUnavailable
        import numpy as np
        stats = self.stats
        plain = (events is None and curve.speed_interval is None and not curve.free_load
                 and seed.get('sideslip_attitude_deg', 0.) == 0.)
        code = self.event_code(curve, event) if event is not None and plain else None
        if (code is not None and alpha is None and elevator is None and load is None
                and getattr(event, 'stationary_plane_normal', None) is None):
            entry = self.native(curve.solver)
            if entry is not None:
                stats['native_events'] += 1
                kinds = stats['event_kinds'];kinds[str(code[0])] = kinds.get(str(code[0]), 0)+1
                if self.verify:return True, self.verified_event(curve, entry, seed, code, max_nfev, max_evaluations,
                                                               event_tolerance, event)
                try:
                    return True, self.m.correct(entry[0], curve, seed, max_nfev=max_nfev, max_evaluations=max_evaluations,
                                                event_tolerance=event_tolerance, event_kind=code)
                except (TrialUnavailable, BranchUnavailable, np.linalg.LinAlgError):
                    return True, None
                except (ValueError, RuntimeError, ArithmeticError) as error:
                    # Unexpected native failure (e.g. a full memo): this call goes to Python.
                    errors = stats['native_event_errors'];key = type(error).__name__+': '+str(error)[:60]
                    errors[key] = errors.get(key, 0)+1
        identity = getattr(event, 'stationary_identity', None) if event is not None else None
        sep = identity is not None and identity[0] == 'direct discrete SEP'
        if event is not None and not sep:stats['python_events'] += 1
        single = (alpha is not None)+(load is not None)+(elevator is not None)+(event is not None) == 1
        if single and (event is None or sep) and plain:
            entry = self.native(curve.solver)
            if entry is not None:
                stats['native_corrections'] += 1
                try:
                    return True, self.m.correct(entry[0], curve, seed, alpha=alpha, load=load, max_nfev=max_nfev,
                                                max_evaluations=max_evaluations, event_tolerance=event_tolerance,
                                                event_target=identity[1] if sep else None, elevator=elevator)
                except (TrialUnavailable, BranchUnavailable, np.linalg.LinAlgError):
                    return True, None
        stats['python_corrections'] += 1
        return False, None

    def event_code(self, curve, event):
        """(1,0,0) stall, (2,0,0) wing force, (3, axis, side) control without the Instructor; else None."""
        codes = events_supported()
        if not codes:return None
        code = getattr(event, '__code__', None)
        if code is None:return None
        if code is codes['stall'] and event.__defaults__ is None:return (1, 0, 0)
        if code is codes['wing force'] and event.__defaults__ is None:return (2, 0, 0)
        if code is codes['control'] and not curve.solver.config.get('instructor'):
            d = event.__defaults__
            if d is not None and len(d) == 2 and d[0] in (0, 1, 2) and d[1] in (0, 1):return (3, d[0], d[1])
        return None

    def operating_point(self, solver, speed, load, x, args, kwargs):
        angles = kwargs.get('stationary_angles')
        propulsion = kwargs.get('propulsion_sample')
        allowed = {'propulsion_sample', 'stationary_angles', '_search', '_evaluation_cache', 'canonical_propulsion'}
        if (angles is not None and not args and set(kwargs) <= allowed and not kwargs.get('canonical_propulsion', False)
                and (propulsion is None) != bool(getattr(solver, 'is_prop', False))):
            entry = self.native(solver)
            if entry is not None:
                from em_trim_work import charge, observe
                self.stats['native_points'] += 1
                if self.verify:return False, self.verified_point(entry, solver, speed, load, x, angles, kwargs, propulsion)
                charge('aircraft_evaluations')
                value = entry[0].model.value(speed, load, x, angles, bool(kwargs.get('_search', False)), propulsion)
                observe(solver, speed, load, x, value, False)
                return True, value
        self.stats['python_points'] += 1
        return False, None

    def frozen_point(self, core, speed, load, x, args, kwargs, history_initial):
        propulsion = kwargs.get('propulsion_sample', args[0] if args else None)
        plain = (not args or args[0] is None) and set(kwargs) <= {'propulsion_sample', 'canonical_propulsion'}
        if (plain and (propulsion is None) != bool(getattr(core, 'is_prop', False))
                and core.config.get('trim_solver_mode') == 'stationary'
                and not core.config.get('experimental_instantaneous_aero', False)):
            entry = self.native(core)
            if entry is not None:
                self.stats['native_frozen'] += 1
                return True, entry[1].solve(core, speed, load, x, propulsion, history_initial)
        self.stats['python_frozen'] += 1
        return False, None

    def joint_correct(self, problem, initial, *, offset, max_nfev, trace, residual_scale, numerical_target, warm_start):
        if (offset == 0 and trace is None and residual_scale is None and numerical_target is None
                and getattr(problem.solver, 'sideslip_attitude_deg', 0.) == 0.):
            entry = self.native(problem.solver)
            if entry is not None:
                self.stats['native_point_solves'] += 1
                return True, self.m.joint_correct(entry[0], problem, initial, max_nfev, warm_start)
        self.stats['python_point_solves'] += 1
        return False, None

    def instructor_at(self, solver, value, speed, load):
        inst = self.instructor_for(solver)
        if inst is not None:
            try:
                if self.verify:return True, self.verified_instructor(inst, solver, value, speed, load)
                out = inst.limits(value, speed)
                self.stats['native_instructor'] += 1
                return True, out
            except self.m.InstructorUnsupported as error:
                reasons = self.stats.setdefault('python_instructor_reasons', {})
                reasons[str(error)] = reasons.get(str(error), 0)+1
        self.stats['python_instructor'] += 1
        return False, None

    # ----- shadow checks (WT_MARCH_NATIVE_VERIFY=1) ---------------------------
    def mismatch(self, kind, diffs, decision=False, example=None):
        row = self.stats.setdefault('verify', {}).setdefault(kind, dict(checked=0, mismatched=0, decision=0, worst={}, examples=[]))
        row['checked'] += 1
        if not diffs:return
        row['mismatched'] += 1;row['decision'] += bool(decision)
        for path, a, b in diffs:
            key = ''.join(ch for ch in path if not ch.isdigit())
            d = abs(a-b) if isinstance(a, float) and isinstance(b, float) else float('inf')
            if d != d:d = float('inf')
            if key not in row['worst'] or d > row['worst'][key]:row['worst'][key] = d
        if example is not None and len(row['examples']) < 5:row['examples'].append(example)

    def verified_point(self, entry, solver, speed, load, x, angles, kwargs, propulsion):
        """Native and Python explicit-history evaluations compared exactly (no work charged
        for the native one); the Python path then evaluates as usual."""
        native = entry[0].model.value(speed, load, x, angles, bool(kwargs.get('_search', False)), propulsion)
        from em_operating import operating_point
        python = operating_point(solver, speed, load, x, **kwargs)
        diffs = leaf_diffs(plain(python), plain(native))
        self.mismatch('operating_point', diffs, example=dict(speed=speed, load=load, diffs=[list(map(str, d)) for d in diffs[:6]]))
        return None

    def verified_instructor(self, inst, solver, value, speed, load):
        from instructor_envelope import controller_limits
        if value.get('instructor') is not None:return value['instructor']
        inst.python_trim = False
        native_value = dict(value);native_value['instructor'] = None
        python_value = dict(value);python_value['instructor'] = None
        reference = controller_limits(solver, python_value, speed)
        try:candidate = inst.limits(native_value, speed)
        except self.m.InstructorUnsupported:
            value['instructor'] = reference;return reference
        diffs = leaf_diffs(plain(reference), plain(candidate))
        decisions = [(r['converged'], r['envelope_margin'] < -2e-7, r['margins']['control authority with auto trim'] < -2e-7)
                     for r in (reference, candidate)]
        self.mismatch('instructor', diffs, decision=decisions[0] != decisions[1],
                      example=dict(speed=speed, load=load, diffs=[list(map(str, d)) for d in diffs[:6]]))
        value['instructor'] = reference
        return reference

    def verified_event(self, curve, entry, seed, code, max_nfev, max_evaluations, event_tolerance, event):
        """Native and Python event corrections on the same seed (agree to rounding: decisions
        must match); the Python result is returned."""
        from em_stationary import BranchUnavailable
        from em_stationary_boundary import TrialUnavailable
        from em_trim_work import Work, use
        import numpy as np
        w = Work({}, parent=None)
        saved = dict(curve.predictors)
        with use(w):
            try:native = self.m.correct(entry[0], curve, seed, max_nfev=max_nfev, max_evaluations=max_evaluations,
                                        event_tolerance=event_tolerance, event_kind=code)
            except (TrialUnavailable, BranchUnavailable, np.linalg.LinAlgError):native = None
        curve.predictors.clear();curve.predictors.update(saved)
        try:python = curve._correct(seed, event=event, max_nfev=max_nfev, max_evaluations=max_evaluations,
                                    event_tolerance=event_tolerance)
        except (TrialUnavailable, BranchUnavailable, np.linalg.LinAlgError):python = None
        a = None if python is None else plain(python)
        b = None if native is None else plain(native)
        if (a is None) != (b is None):
            self.mismatch('event', [('presence', float(a is None), float(b is None))], decision=True)
            return python
        diffs = []
        if a is not None:
            for p in (a, b):p.pop('evaluations', None)
            diffs = leaf_diffs(a, b)
        decision = a is not None and (a['valid'] != b['valid'] or sorted(a['reasons']) != sorted(b['reasons']))
        self.mismatch('event', diffs, decision=decision, example=dict(speed=curve.speed, code=list(code),
                                                                      diffs=[list(map(str, d)) for d in diffs[:6]]))
        return python


def plain(x):
    """JSON-normalized copy (numpy scalars and arrays to Python values) for exact comparison."""
    return json.loads(json.dumps(x, default=lambda o: o.tolist() if hasattr(o, 'tolist') else str(o)))


def leaf_diffs(a, b, path=''):
    out = []
    if isinstance(a, dict) and isinstance(b, dict):
        if set(a) != set(b):return [(path+'{keys}', 0., 1.)]
        for k in a:out += leaf_diffs(a[k], b[k], path+'/'+k)
        return out
    if isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):return [(path+'[len]', float(len(a)), float(len(b)))]
        for i, (u, v) in enumerate(zip(a, b)):out += leaf_diffs(u, v, path+'[%d]' % i)
        return out
    if type(a) != type(b) or a != b:
        if isinstance(a, float) and isinstance(b, float) and a != a and b != b:return out
        return [(path, a, b)]
    return out


def summary(loops):
    """Counters of the NativeLoops of one chart (one per solver model)."""
    def add(into, values, worst=False):
        for k, v in values.items():
            if isinstance(v, dict):add(into.setdefault(k, {}), v, worst=worst or k == 'worst')
            elif isinstance(v, list):into[k] = (into.get(k, [])+v)[:5]
            elif isinstance(v, (int, float)) and not isinstance(v, bool):
                into[k] = max(into.get(k, v), v) if worst else into.get(k, 0)+v
            else:into[k] = v
    out = dict(status=status())
    for loop in loops:
        add(out, loop.stats)
        if loop.entry:
            cc = loop.entry[0]
            for k, v in cc.counters().items():
                if isinstance(v, (int, float)):out['curve_'+k] = out.get('curve_'+k, 0)+v
        if loop.unsupported:out.setdefault('unsupported', {}).update(loop.unsupported)
    return out


# ----- S5: the native march chart (march, tracer and column workers on native states) ---------------
def engine(solver):
    """The native march engine of an attached solver, or None (then the Python march runs, as S4)."""
    if os.environ.get('WT_MARCH_NATIVE_LOOP', '1') == '0':return None
    loop = getattr(solver, 'native_loop', None)
    if loop is None or loop.verify:return None
    if not hasattr(loop.m, 'MarchEngine'):return None
    try:
        return loop.m.MarchEngine(solver, loop)
    except loop.m.NativeMarchUnsupported as error:
        loop.unsupported['march'] = str(error)[:120]
        return None


def column_preamble(solver, speed):
    """em_sampling.sample_with_history before the worker: per-column solver state, then the speed-limit
    exclusion (returned as the column, as there)."""
    from em_workers import prepare_job
    from em_speed_limits import excluded_column
    prepare_job()
    if hasattr(solver, 'direct_boundary'):solver.stationary_boundary_evidence = []
    solver.chart_load_anchors = []
    solver._sideslip_solvers = {}
    solver.instructor_boundaries = {}
    solver.instructor_trim_entries = {}
    for key in ('_trim_predictor', '_minimum_instructor_speed', '_static_instructor_trim_cache'):
        solver.__dict__.pop(key, None)
    excluded = excluded_column(solver, speed)
    if excluded is not None:return excluded
    solver.__dict__.pop('_interior_curve', None)
    if solver.is_prop and solver.engine.automatic:
        # The legacy engine search state; the stationary propulsion never reads it.
        solver.engine._task_seed = None
        solver.engine.reset_search()
        solver.__dict__.pop('_trim_predictor', None)
    return None


def discontinuous(solver, speed):
    """em_column_work.bounded_column's Mach-discontinuity column (with the sample_with_history tail)."""
    from em_mach_events import discontinuous_column
    column = discontinuous_column(solver, speed)
    if column is None:return None
    return post_column(solver, column)


def post_column(solver, column):
    """em_sampling.sample_with_history after the worker: boundary evidence, load-limit and Mach annotation."""
    from em_load_limits import annotate
    from em_mach_events import annotate as annotate_mach
    if hasattr(solver, 'direct_boundary'):
        retained = [];seen = set()
        for evidence in [*column.get('stationary_boundary_evidence', ()), *getattr(solver, 'stationary_boundary_evidence', ())]:
            if id(evidence) in seen:continue
            seen.add(id(evidence));retained.append(evidence)
        column['stationary_boundary_evidence'] = retained
    return annotate_mach(solver, annotate(solver, column))


def march_record(solver, col, points, samples, boundary, level):
    """em_march.column_record + march_worker's region annotation + bounded_column + sample_with_history tail."""
    from em_aircraft_region import annotate as annotate_region
    from em_load_limits import annotate
    from em_mach_events import annotate as annotate_mach
    record = dict(speed_kmh=float(col.speed), points=points, boundary=boundary, lower_boundary=level,
                  boundary_status=col.raw_status, boundary_reason=col.raw_reason,
                  boundary_bracket_g=None, sustained=[], load_checks=[], numerical_gap_brackets=[], interior_failures=[],
                  unresolved_load_intervals=[], mass=solver.mass, engine=solver.engine.summary, boundary_probe=True,
                  lower_boundary_searched=False, elapsed_s=col.elapsed,
                  s1_march=dict(trials=col.ntrials, reason=col.march_reason, trial_log=col.trial_log,
                                samples=[[round(p['alpha_deg'], 4), round(p['load_g'], 5), round(p['ps_mps'], 3)]
                                         for p in samples]))
    record['s3_path'] = col.s3_path
    if col.s3_flags:record['s3_flags'] = col.s3_flags
    if col.region:annotate_region(solver, record)
    record['search_work'] = col.search_work
    if hasattr(solver, 'direct_boundary'):record['stationary_boundary_evidence'] = list(col.evidence_list)
    return annotate_mach(solver, annotate(solver, record))


def merge_stats(engine_stats):
    """Book the native march's counters into em_march.STATS / RESCUE_STATS (the chart's march report)."""
    import em_march as M
    stats = dict(engine_stats)
    M.STATS['columns'] += stats.pop('columns', 0)
    M.STATS['predictors_offered'] += stats.pop('predictors_offered', 0)
    for key, value in stats.pop('s3_paths', {}).items():
        M.RESCUE_STATS['s3_paths'][key] = M.RESCUE_STATS['s3_paths'].get(key, 0)+value
    for key in ('s3_face', 's3_bisect', 's3_unresolved', 'bisect_trials', 'face_trials', 'behind_face', 'fold_within_tolerance'):
        M.RESCUE_STATS[key] = M.RESCUE_STATS.get(key, 0)+stats.pop(key, 0)
    if 'rescue_errors' in stats:
        errors = M.RESCUE_STATS.setdefault('rescue_errors', {})
        for key, value in stats.pop('rescue_errors').items():errors[key] = errors.get(key, 0)+value
    M.STATS['native_march'] = stats
