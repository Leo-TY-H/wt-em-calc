"""The default chart solver (S4): alpha-march envelope plus direct SEP curves.

For each aircraft (``compute``):
1. envelope: ``em_march.MarchTracer`` places speed columns (sweep, corners,
   edges, withheld midpoint checks); every column is an alpha march from level
   flight to the first limit reached (Leo, 2026-10-17); under the wing rule
   (wing_line.py, Leo 2026-11-10) the wing breaking load is no limit but a
   charted line, and a second envelope pass (secondary_pass) solves the
   Instructor's other wing protection state for the dashed line;
2. SEP curves: ``em_march_curves.MarchCurveSolver`` in column mode on the
   verified columns, with withheld exact checks of every knot interval;
3. the compiled loop (``em_march_native``) where it covers the call.

Policy and tolerances (speed plan, Leo 2026-10-17/18): zero prescribed
sideslip (``sideslip_recovery`` False); envelope tolerance 0.15 deg/s at the
standard 0.5 m/s chart precision (scaled with the chart's turn tolerance for
other precisions); curve tolerance 3x the chart curve tolerance; physical
admission unchanged except the wing force under the wing rule (a line, not a
rejection); every drawn state is an admitted full-model state.

What the chart shows:
- the outline is PCHIP in limiting load through verified boundary columns,
  split wherever the trace has a Mach seam, an unresolved speed, an edge or a
  missing column; a smooth line is never drawn across an unresolved speed.
  S7: where the columns carry their exact boundary slopes, the outline is the
  cubic Hermite through loads and slopes, and between two columns on different
  limiting faces the two faces continued from their columns (slope and
  curvature) up to where they meet (the corner), the interpolant the tracer's
  withheld checks validate (run_loads, boundary_interval);
- SEP contours are the solved curve components, drawn only between their
  knots and never across an unresolved knot interval; only the calculated
  levels are available (other levels need a new calculation). S7: a contour is
  the cubic Hermite in n^2-1 through its knots and their exact slopes (the
  slope of each knot's own SEP level set; PCHIP slopes where none was solved),
  the interpolant the curve solver's withheld position checks validate;
  a level-flight end knot (no slope exists at n = 1) takes the slope of the
  parabola through its interval with the neighbour's slope.

Fallback (``eligible``): configurations outside the validated domain use the
baseline sampler for that aircraft, and so does any aircraft on which the
march raises or finds no verified boundary at all. The reason is recorded in
the chart (``chart_solver_fallback``). ``chart_solver='baseline'`` selects
the baseline for every aircraft.
"""
from __future__ import annotations

import math
import os
import time

import numpy as np

COLORS = ('#38c9d7', '#ffa66b')
# S9 (WT_MARCH_S9=0 restores S8): the S9 fixes of the march chart (native: tracer, curves, sliding; here: bookkeeping)
S9 = os.environ.get('WT_MARCH_S9', '1') != '0' and os.environ.get('WT_MARCH_S8', '1') != '0'
# S10 (WT_MARCH_S10=0 restores S9): contours that turn back drawn as they are (fold pieces), high-speed edges drawn vertical
S10 = S9 and os.environ.get('WT_MARCH_S10', '1') != '0'
# S15 (WT_MARCH_S15=0 restores S14): the forward-IAS edge drawn from the native trace along the limit itself (ias_trace),
# every state within the limit inside the outline; where a chart's forward-IAS end has no drawn edge, a vertical one
def s15_on():
    return S10 and all(os.environ.get(k, '1') != '0' for k in ('WT_MARCH_S11', 'WT_MARCH_S12', 'WT_MARCH_S13', 'WT_MARCH_S14',
                                                              'WT_MARCH_S15'))


# S16 (WT_MARCH_S16=0 restores S15): a level flight that over-revs starts the march (in a turn the propeller turns slower):
# the engine RPM edge leans and is drawn from the native trace along the limit (rpm_trace), as the forward-IAS edge
def s16_on():
    return s15_on() and os.environ.get('WT_MARCH_S16', '1') != '0'
# Over-rev (Leo 2026-10-26, option C; WT_OVERREV=0 restores S16): an engine RPM over RPMMaxAllowed is no chart limit;
# the chart carries where over-rev starts (overrev_lines) and, on every over-rev row, contour and outline point, the
# seconds the game gives that engine (overrev.py)
from overrev import RULE as OVERREV
# Wing line (Leo 2026-11-10; WT_WING_LINE=0 restores the wing cut): the wing breaking load is a charted line, not a limit
# (column['wing_line'], aircraft['wing_line']); SEP contours are split on it (past_wing); the Instructor's other wing
# protection state is a second envelope pass drawn dashed (secondary_envelope, secondary_boundary) (wing_line.py)
import wing_line as WING
CURVE_WIDE_SLIDE = int(os.environ.get('WT_OVERREV_CURVE_SLIDE', '28'))   # MarchEngine.wide_slide bits for the curve engine
ENVELOPE_TOLERANCE_DPS = .15   # at the standard chart precision (turn tolerance 0.02 deg/s)
CURVE_SCALE = 3.
COLUMN_SPACING_KMH = 80.
MAX_STEP_KMH = 50.
REVISION = 'alpha-march-chart-v1'


def eligible(condition):
    """(True, None) when the march chart covers this aircraft condition, else (False, reason).

    S8: manual engine controls (the march's own engine RPM rule draws the over-rev edges), every chart load cap (the
    march's load ceiling), every Mach curve mode and any wing sweep (stretches where the selected sweep is unavailable
    are cut out of the speed range: chart_segments) are in the domain; what stays outside are the diagnostic and
    experimental model modes."""
    c = condition
    checks = [
        (c['trim_solver_mode'] == 'stationary', 'nested or uncorrected trim'),
        (c['sampling'] == 'adaptive', 'regular diagnostic sampling'),
        (c['stationary_execution'] == 'baseline', 'diagnostic stationary execution mode'),
        (not c['experimental_instantaneous_aero'] and c['experimental_lateral_trim'] == 'full', 'model experiment'),
        (c['aircraft_trim_mode'] == 'discrete' and c['turn_response_mode'] == 'settled', 'trim or turn-response mode'),
        (c['stationary_governor_mode'] == 'pitch_stops', 'governor mode other than pitch stops'),
    ]
    for ok, reason in checks:
        if not ok:return False, reason
    return True, None


def envelope_tolerance(config):
    from em_accuracy import turn_tolerance
    # The ratio first, so the standard precision gives exactly 0.15 deg/s.
    return ENVELOPE_TOLERANCE_DPS*(turn_tolerance(config['sep_tolerance_mps'])/turn_tolerance(.5))


SWEEP_GUARD_KMH = .0036   # a sweep-limit segment end lies this far inside the available speeds (as the speed limit's sample)


def chart_segments(solver):
    """(segments, speed limit, Mach seams, sweep gaps) of the march chart's speed range: the chart range up to the
    enforced speed limit, split at the native Mach discontinuities (seams, 'mach_seam' ends) and, S8, around every
    stretch where the selected wing sweep is unavailable ('sweep_limit' ends: the game's sweep schedule runs on
    total-speed Mach, so the edge is vertical; S5 made a stretch reaching the end of the range the speed limit
    'Wing sweep limit'). Each segment is traced on its own, so a vertical edge stands at every sweep-limit end."""
    from em_speed_limits import speed_limits, sweep_excluded_kmh, march_domain
    from em_mach_events import solver_regions
    config = solver.config
    lo, hi = float(config['speed_min_kmh']), float(config['speed_max_kmh'])
    limit = speed_limits(solver.fm, config)
    end, hint = 'range_end', None
    if limit['enforced'] and getattr(solver, 'soft_admission', False):
        # S8: vertical edges only at the Mach and sweep limits; a forward-IAS limit is judged on each state (the range
        # runs on past its zero-alpha speed, the 'hint' where the top segment's trace starts)
        hi, end, hint = march_domain(limit, hi)
    elif limit['enforced'] and limit['sample_speed_kmh'] < hi:
        hi, end = limit['sample_speed_kmh'], 'speed_limit'
    seams = sorted((a, b) for a, b in (r['speed_interval_kmh'] for r in solver_regions(solver)) if b > lo and a < hi)
    gaps = sorted([a-SWEEP_GUARD_KMH, b+SWEEP_GUARD_KMH] for a, b in sweep_excluded_kmh(solver.fm, config)
                  if b > lo and a < hi and b < float(config['speed_max_kmh'])-1e-6)
    blocks = sorted([(a, b, 'mach_seam') for a, b in seams]+[(a, b, 'sweep_limit') for a, b in gaps])
    out, start, kind = [], lo, 'range_end'
    for a, b, why in blocks:
        if a > start:out.append(dict(speed_kmh=[start, a], ends=[kind, why]))
        if b > start:start, kind = b, why
    if hi > start:out.append(dict(speed_kmh=[start, hi], ends=[kind, end]))
    for s in out:
        if hint is not None and s['speed_kmh'][0] < hint < s['speed_kmh'][1]:s['start_kmh'] = hint
    return out, limit, seams, [list(g) for g in gaps]


def compute(config, progress=None, cancelled=None, preview=None):
    """Chart data with the march chart where eligible and the baseline sampler elsewhere."""
    from em_solver import aircraft_settings, compute as compute_chart
    started = time.monotonic()
    names = list(config['aircraft'])
    conditions = {name: aircraft_settings(config, name) for name in names}
    rows, fallback = {}, {}
    for index, name in enumerate(names):
        ok, reason = eligible(conditions[name])
        if not ok:
            fallback[name] = 'outside the march chart domain: '+reason
            continue
        if cancelled and cancelled():raise InterruptedError('Calculation cancelled')

        def report(p, index=index):
            if progress:progress(dict(p, done=index+p.get('fraction', 0.), total=len(names), elapsed_s=time.monotonic()-started))

        def show(row):
            if preview is None:return
            snapshot = assemble(config, [*rows.values(), row], started, preview=True)
            preview(snapshot)
        try:
            rows[name] = march_aircraft(name, conditions[name], index, report, cancelled, show)
        except (InterruptedError, KeyboardInterrupt, MemoryError):
            raise
        except Exception as error:  # never a wrong chart: this aircraft uses the baseline sampler
            import traceback
            traceback.print_exc()
            fallback[name] = 'march chart failed: '+type(error).__name__+': '+str(error)[:200]
            continue
        if not any(c.get('boundary_status') in ('verified limit', 'plot ceiling') for c in rows[name]['columns']):
            # S8: a chart whose speed limit lies below its speed range (e.g. flaps that the selected wing sweep retracts)
            # is empty by that limit, not a march failure
            limit = (rows[name].get('speed_limit') or {}).get('speed_kmh')
            if limit is not None and limit <= conditions[name]['speed_min_kmh'] and os.environ.get('WT_MARCH_S8', '1') != '0':
                rows[name]['chart_empty_reason'] = rows[name]['speed_limit']['kind']
                continue
            fallback[name] = 'march chart found no verified boundary'
            rows.pop(name)
    baseline = None
    if fallback:
        subset = [n for n in names if n in fallback]
        work = dict(config, chart_solver='baseline', aircraft=subset,
                    aircraft_settings={n: config['aircraft_settings'][n] for n in subset if n in config.get('aircraft_settings', {})})
        done = [*rows.values()]

        def report_baseline(p):
            if progress:progress(dict(p, phase='Baseline sampler · '+p.get('phase', 'Solving')))

        def preview_baseline(data):
            if preview is not None:preview(merge(config, data, done, started, preview=True))
        baseline = compute_chart(work, report_baseline if progress else None, cancelled,
                                 preview_baseline if preview else None)
        for a in baseline['aircraft']:
            a['chart_solver'] = 'baseline';a['chart_solver_fallback'] = fallback[a['id']]
    data = merge(config, baseline, list(rows.values()), started) if baseline is not None else assemble(config, list(rows.values()), started)
    return data


def merge(config, baseline, rows, started, preview=False):
    """Baseline-sampler data with march-chart aircraft rows added (config order)."""
    if baseline is None:return assemble(config, rows, started, preview=preview)
    by_id = {a['id']: a for a in baseline['aircraft']}
    by_id.update({r['id']: r for r in rows})
    out = dict(baseline, settings=config, aircraft=[by_id[n] for n in config['aircraft'] if n in by_id])
    if rows:
        out['method'] = 'Alpha-march envelope and direct SEP curves; baseline adaptive sampler where noted per aircraft'
        out['assumptions'] = list(dict.fromkeys([*baseline.get('assumptions', []), *ASSUMPTIONS]))
        out['speeds_kmh'] = sorted({v for a in out['aircraft'] for v in
                                    ([c['speed_kmh'] for c in a['columns']] if 'columns' in a else baseline.get('speeds_kmh', []))})
    for i, a in enumerate(out['aircraft']):a['color'] = COLORS[i % 2]
    out['elapsed_s'] = time.monotonic()-started
    if preview:out['preview'] = True
    return out


ASSUMPTIONS = [
    'Envelope boundary: the first physical limit reached when alpha is increased from level flight at each speed (alpha march); unresolved speeds remain gaps, never absence proofs',
    'Zero prescribed sideslip at every solved state; the 2 degree actual-sideslip admission limit and all other physical admission checks unchanged',
    'SEP curves solved directly at the selected levels with withheld exact checks between knots; other levels need a new calculation',
    'Chart tolerances: envelope 0.15 deg/s at standard precision; SEP curves 3x the chart curve tolerance',
]


def assemble(config, rows, started, preview=False):
    from em_solver import BACKEND
    data = dict(settings=config, aircraft=list(rows), sampling='alpha march', backend=BACKEND, workers=1, loads_g=[],
                method='Alpha-march envelope and direct SEP curves; stationary sample equations',
                assumptions=['Full-real manual aerodynamic trim', 'Constant fuel and intact components',
                             'Still air; out of ground effect; retractable gear and airbrakes stowed; fixed propeller-aircraft gear retains its drag',
                             'Fixed flap extension; speed domain ends at the selected extension’s automatic IAS/Mach limit or intact-flap damage threshold; flap travel and damage transients omitted',
                             'Normal controllable flight: native pitch response must retain the normal elevator direction',
                             *ASSUMPTIONS],
                validation='Reconstructed kernels have native-code comparisons; this EM solver has not been validated against live flight.',
                speeds_kmh=sorted({c['speed_kmh'] for a in rows for c in a['columns']}),
                elapsed_s=time.monotonic()-started, cached_columns=0, cached_aircraft=[],
                stationary_baseline_chart=True, coverage_complete=False, _stationary_presentation_ready=True,
                chart_revision=REVISION)
    for i, a in enumerate(data['aircraft']):a['color'] = COLORS[i % 2]
    if config.get('global_load_cap_g') is not None:
        data['assumptions'].append(f"Additional global search ceiling: {config['global_load_cap_g']:g} g; not a proved physical limit.")
    if OVERREV and any(a.get('overrev_lines') is not None for a in rows):
        data['assumptions'].append(OVERREV_ASSUMPTION)
    if any((a.get('envelope_trace') or {}).get('wing_line') for a in rows):
        data['assumptions'].append(WING_ASSUMPTION)
    if preview:data['preview'] = True
    return data


OVERREV_ASSUMPTION = ('Engine over-rev (RPM above RPMMaxAllowed) is charted, not a limit; a line marks where it starts. With '
                      'Engine_Overheat on (Realistic, Simulator) the game stops an over-revving engine after the seconds '
                      'given (steady RPM, fresh engine) and adds its over-rev torque, as charted')
WING_ASSUMPTION = ('The wing breaking load (wing load ratio 1) is charted as a line, not a limit: beyond it the game breaks '
                   'an intact wing after about 0.47 / (ratio - 1) s (0.3/(ratio-1) s, then a chance of (ratio-1)/8 per 48 Hz '
                   'step). Instructor wing protection: where its release starts after 10 s or more the protected limit is '
                   'the envelope and the released limit is dashed above it; otherwise the released limit is the envelope '
                   'and the protected limit is dashed inside')


# ----- one aircraft --------------------------------------------------------------
def march_aircraft(name, condition, index, report, cancelled, show):
    import em_march as M
    import em_march_curves as C
    import em_march_native as N
    import em_sampling as S
    from em_solver import settings
    from em_stationary_baseline import BaselineSolver
    from em_trim_work import Work, use
    started = time.monotonic()
    cfg = settings(dict(condition, sideslip_recovery=False))
    # Nothing carries over from an earlier chart in this process.
    M.reset();S.worker_solver.cache_clear()
    columns_done = [0]

    def on_column(tracer, phase, speed, why):
        if cancelled and cancelled():raise InterruptedError('Calculation cancelled')
        if phase == 'done':
            columns_done[0] += 1
            report(dict(phase=f'Envelope · alpha march · {columns_done[0]} speeds', aircraft=name, speed_kmh=float(speed),
                        fraction=min(.6, .6*columns_done[0]/80.)))
    tracer = M.MarchTracer(name, cfg, turn_tolerance=envelope_tolerance(cfg), max_step_kmh=MAX_STEP_KMH, on_column=on_column)
    solver = S.worker_solver(name, tracer.cj)
    curve_solver = BaselineSolver(name, cfg)
    # S8: the chart's admission rule for soft limits (engine RPM, forward IAS): hard checks also on soft-rejected states
    # (WT_MARCH_S8=0: the S7 rule, every soft limit a hard one)
    import os
    if os.environ.get('WT_MARCH_S8', '1') != '0':
        for s in (solver, curve_solver):s.soft_admission = True
    # S5: no cyclic garbage collection while the chart builds its rows. A collection walks every live
    # object (the loaded aircraft data included), and the many row dicts would trigger several; the
    # chart creates no reference cycles that need collecting before it returns.
    import gc
    collecting = gc.isenabled()
    gc.disable()
    extra = []   # the solvers march_solvers adds (the second envelope pass of the Instructor's other wing protection state)
    try:
        return march_solvers(name, index, cfg, tracer, solver, curve_solver, report, cancelled, show, started, extra)
    finally:
        for s in (solver, curve_solver, *extra):
            N.detach(s);s.__dict__.pop('soft_admission', None)
        if collecting:gc.enable()


def march_solvers(name, index, cfg, tracer, solver, curve_solver, report, cancelled, show, started, extra=None):
    import em_march as M
    import em_march_curves as C
    import em_march_native as N
    from em_trim_work import Work, use
    native = N.attach(solver)
    loops = [solver.native_loop] if getattr(solver, 'native_loop', None) is not None else []
    total = Work({});t0 = time.process_time();w0 = time.perf_counter()
    # S5: the march, its column workers and the tracer on native trial states where the compiled
    # module covers this solver; otherwise the Python march (em_march, the reference) as in S4.
    engine = N.engine(solver)
    if engine is None:
        # S8: the soft-limit rule is native only; the Python march (the rollback reference) keeps the S7 rule
        for s in (solver, curve_solver):s.__dict__.pop('soft_admission', None)
    if engine is not None:
        native_tracer =solver.native_loop.m.NTracer(engine, name, cfg, tracer.cj, tracer.ttol, tracer.vtol, tracer.place_tol,
                                                     tracer.max_step, tracer.max_depth, tracer.march_tol, tracer.on_column)
        with use(total):
            envelope = native_tracer.run()
            envelope['columns'] = [native_tracer.record(c) for c in envelope['columns']]
        N.merge_stats(engine.stats)
    else:
        with use(total):envelope = tracer.run()
    envelope_cpu = time.process_time()-t0;envelope_wall = time.perf_counter()-w0
    envelope_work = dict(total.counts)
    # Wing line (Leo 2026-11-10): every verified column's wing-line point. Native columns carry theirs (NTracer.record,
    # the wing event); a column without one (a production-fallback column) takes the interpolation between its samples
    if WING.rule(solver):
        for c in envelope['columns']:
            if tracer.ok(c) and 'wing_line' not in c:
                point = WING.column_point(c)
                if point:c['wing_line'] = point
    verified = curve_columns(envelope, tracer)
    row = aircraft_row(name, index, cfg, solver, envelope, None, tracer)
    show(row)
    # whether the Instructor's other wing protection state moves the boundary anywhere (one evaluation per column;
    # columns whose evaluation fails are skipped and counted: never a reason to lose the chart)
    w0 = time.perf_counter();scan_failures = []
    with use(Work({})):binds = protection_scan(solver, envelope['columns'], scan_failures) if engine is not None else False
    scan_wall = time.perf_counter()-w0
    N.attach(curve_solver)
    if getattr(curve_solver, 'native_loop', None) is not None:loops.append(curve_solver.native_loop)
    curves = C.MarchCurveSolver(name, cfg, solver=curve_solver, columns=verified,
                                policy=C.CurvePolicy(tolerance_scale=CURVE_SCALE, column_min_spacing_kmh=COLUMN_SPACING_KMH))
    count = [0]

    def check():
        if cancelled and cancelled():raise InterruptedError('Calculation cancelled')
        count[0] += 1
        if count[0] % 25 == 0:report(dict(phase='SEP curves', aircraft=name, fraction=min(.99, .6+.4*count[0]/2000.)))
    curves.check = check
    t0 = time.process_time();w0 = time.perf_counter()
    # S5: the curve solver on native trial states where the compiled module covers it.
    curve_engine = N.engine(curve_solver) if engine is not None else None
    if curve_engine is not None:curve_engine.known_caps = engine.ceilings
    # Over-rev follow-up: the wide sliding search only in the curve solver's own rescues and 1 g solves, not in the level
    # flight of its level-flight path marches (S16's path rule judges contour states against that path)
    if curve_engine is not None and OVERREV:curve_engine.wide_slide &= CURVE_WIDE_SLIDE
    if verified and curve_engine is not None:
        native_curves = curve_solver.native_loop.m.NCurves(curve_engine, curves)
        native_curves.check = check
        if s16_on():
            native_curves.set_outline(*curve_outline(cfg, row, tracer))
            if engine is not None:native_curves.edge_knots = list(native_tracer.edge_rows.values())
            native_curves.rpm_feet = [min(e['speed_kmh']) for e in envelope['events'] if e['kind'] == 'rpm edge']
            native_curves.lf_rows = level_flights(envelope)
        result = native_curves.run(verified)
    else:
        result = curves.run() if verified else None
    curve_cpu = time.process_time()-t0;curve_wall = time.perf_counter()-w0
    if result is not None and str(result.get('status', '')).startswith('exhausted'):
        raise RuntimeError('SEP curve budget exhausted: '+result['status'])
    row = aircraft_row(name, index, cfg, solver, envelope, result, tracer)
    # Leo 2026-11-10: the envelope of the Instructor's other wing protection state (the dashed line), envelope only. A
    # failure of this pass only drops the dashed line (never a baseline fallback of the chart)
    secondary = dict(ran=False, reason='compiled march unavailable')
    if engine is not None:
        try:
            secondary = secondary_pass(name, cfg, tracer, solver, binds, loops, extra if extra is not None else [],
                                       report, cancelled)
        except (InterruptedError, KeyboardInterrupt, MemoryError):
            raise
        except Exception as error:
            secondary = dict(ran=False, reason='failed: '+type(error).__name__+': '+str(error)[:160])
    if secondary.get('ran'):row['secondary_envelope'] = secondary.pop('envelope')
    secondary.update(scan_wall_s=scan_wall, scan_failures=len(scan_failures), **({'scan_failure': scan_failures[0]} if scan_failures else {}))
    ee, ce = envelope_work.get('aircraft_evaluations', 0), (result or {}).get('work', {}).get('aircraft_evaluations', 0)
    row['chart_solver_work'] = dict(evaluations=ee+ce, envelope_evaluations=ee, curve_evaluations=ce,
                                    envelope_cpu_s=envelope_cpu, curve_cpu_s=curve_cpu, elapsed_s=time.monotonic()-started,
                                    envelope_wall_s=envelope_wall, curve_wall_s=curve_wall,
                                    columns=len(envelope['columns']), work_by_role=envelope['work_by_role'],
                                    march=M.stats(), native=N.summary(loops), native_status=native,
                                    curve_status=(result or {}).get('status'),
                                    **({'secondary': secondary} if WING.RULE else {}))
    if engine is not None:
        timers = dict(envelope=dict(solve_s=engine.t_solve, finish_s=engine.t_finish, rows_s=engine.t_rows, point_s=engine.t_point, python_s=engine.t_python, ledger_s=engine.t_ledger, tangent_s=engine.t_tangent))
        if curve_engine is not None:
            timers['curves'] = dict(solve_s=curve_engine.t_solve, finish_s=curve_engine.t_finish, rows_s=curve_engine.t_rows, point_s=curve_engine.t_point, python_s=curve_engine.t_python, ledger_s=curve_engine.t_ledger, tangent_s=curve_engine.t_tangent)
            row['chart_solver_work']['curve_native'] = {k: v for k, v in curve_engine.stats.items()
                                                        if isinstance(v, (int, float)) and not isinstance(v, bool)}
        f = engine.fin_timers()
        timers['envelope'].update(f)
        row['chart_solver_work']['native_timers'] = timers
        row['chart_solver_work']['native_python'] = dict(envelope=engine.python_counts(),
                                                         curves=curve_engine.python_counts() if curve_engine is not None else None)
    return row


def protection_scan(solver, columns, failures=None):
    """Leo 2026-11-10: True where the Instructor's other wing protection state (wing_line.py) moves the boundary of a
    verified column; every such column gets column['protection'] (wing_line.protection_limits at its boundary state:
    one evaluation, no admission). The two states differ only where the cap can be released (released_limit_deg above
    held_limit_deg). A 'held' chart: the column ends on the Instructor's limit and the game's release timer runs at
    that state. A 'released' chart: the cap lies below the boundary state's AoA (inside the limit the upper Instructor
    margin moves exactly by the change of the limit; 0.05 deg slack). A column whose evaluation fails is skipped
    (its error text appended to ``failures``)."""
    state = getattr(solver, 'instructor_protection', None)
    if not (WING.RULE and state):return False
    binds = False
    for c in columns:
        if not ok(c):continue
        ctl = c['boundary'].get('instructor') or {}
        if not ctl.get('converged') or not ctl.get('effective_angle_limits_deg') or not ctl.get('adjusted_wing_angles_deg'):continue
        try:
            lim = WING.protection_limits(solver, c['boundary'])
        except (InterruptedError, MemoryError):
            raise
        except Exception as error:
            if failures is not None:failures.append('%g km/h: %s: %s' % (c['speed_kmh'], type(error).__name__, str(error)[:120]))
            continue
        if not all(isinstance(v, bool) or math.isfinite(v) for v in lim.values()):continue
        c['protection'] = lim
        d = lim.get('released_limit_deg', lim['free_limit_deg'])-lim['held_limit_deg']
        if d <= 1e-6:continue
        if state == 'held':binds |= c.get('boundary_reason') == 'Instructor pitch' and bool(lim['releasable'])
        else:
            # the upper margin of the admission: with the tailplane cap where it is the smaller term (the readout
            # effective_angle_limits_deg is the acceleration margin alone)
            upper = ctl['effective_angle_limits_deg'][1]
            if ctl.get('tail_cap_limiting'):upper = max(ctl['adjusted_wing_angles_deg'])+ctl['tail_cap_margin_deg']
            binds |= d > upper-max(ctl['adjusted_wing_angles_deg'])-.05
    return bool(binds)


SECONDARY_COLUMN = ('speed_kmh', 'boundary_status', 'boundary_reason', 'boundary_slope', 'boundary_slope_kind', 'wing_line')
SECONDARY_POINT = ('speed_kmh', 'load_g', 'turn_dps', 'ps_mps', 'alpha_deg', 'wing_load_ratios', 'envelope_limit', 'valid')


def secondary_pass(name, cfg, tracer, solver, binds, loops, extra, report, cancelled):
    """Leo 2026-11-10: the envelope of the Instructor's other wing protection state (wing_line.py): a second,
    envelope-only native trace on a fresh solver of the same settings with instructor_protection forced to the other
    state (the native Instructor is built once per solver, so the state is never flipped on a live one). Returns
    dict(ran=False, reason) or dict(ran=True, state, wall_s, columns, evaluations, envelope): 'envelope' is the row's
    'secondary_envelope' (slim columns, the trace's run data and its unresolved speeds: what runs() and run_loads read).
    Not run without a protection state, where the protection is never released, or where the other state moves no
    column's boundary (protection_scan). The new solver is appended to ``extra`` (the caller detaches it)."""
    import em_march as M
    import em_march_native as N
    import em_sampling as S
    from em_solver import settings
    from em_trim_work import Work, use
    state = getattr(solver, 'instructor_protection', None)
    if state is None:return dict(ran=False, reason='no wing protection')
    # Leo 2026-11-10: no second Instructor line at all (neither 'after N s' nor 'first N s'); WT_INSTRUCTOR_SECOND_LINE=1 draws it
    if os.environ.get('WT_INSTRUCTOR_SECOND_LINE', '0') != '1':return dict(ran=False, reason='second line not drawn')
    # Leo 2026-11-10: no 'Instructor limit after N s' line; only short-release aircraft get the dashed protection line
    if state == 'held' or math.isinf(solver.protection_release_s):return dict(ran=False, reason='protection held: no released line')
    if not binds:return dict(ran=False, reason='protection never binds')
    other = 'released' if state == 'held' else 'held'
    cfg2 = settings(dict(cfg, instructor_protection=other))

    def on_column(tracer, phase, speed, why):
        if cancelled and cancelled():raise InterruptedError('Calculation cancelled')
    tracer2 = M.MarchTracer(name, cfg2, turn_tolerance=tracer.ttol, max_step_kmh=MAX_STEP_KMH, on_column=on_column)
    solver2 = S.worker_solver(name, tracer2.cj);extra.append(solver2)
    solver2.soft_admission = True
    N.attach(solver2)
    engine2 = N.engine(solver2)
    if engine2 is None:return dict(ran=False, reason='compiled march unavailable')
    loops.append(solver2.native_loop)
    report(dict(phase='Instructor wing protection line', aircraft=name, fraction=.99))
    work = Work({});w0 = time.perf_counter()
    with use(work):
        native_tracer = solver2.native_loop.m.NTracer(engine2, name, cfg2, tracer2.cj, tracer2.ttol, tracer2.vtol,
                                                      tracer2.place_tol, tracer2.max_step, tracer2.max_depth,
                                                      tracer2.march_tol, tracer2.on_column)
        env2 = native_tracer.run()
        columns = sorted((native_tracer.record(c) for c in env2['columns']), key=lambda c: c['speed_kmh'])
    slim = [dict({k: c[k] for k in SECONDARY_COLUMN if c.get(k) is not None},
                 boundary={k: c['boundary'].get(k) for k in SECONDARY_POINT} if c.get('boundary') else None) for c in columns]
    trace = dict(segments=env2['segments'], mach_seams_kmh=env2['mach_seams_kmh'], sweep_gaps_kmh=env2.get('sweep_gaps_kmh', []),
                 events=env2['events'],
                 # (the traced forward-IAS and engine RPM edges: this state's outline ends at its own corner, drawn_runs)
                 **{k: env2[k] for k in ('ias_trace', 'rpm_trace') if env2.get(k)})
    return dict(ran=True, state=other, wall_s=time.perf_counter()-w0, columns=len(slim),
                evaluations=work.counts.get('aircraft_evaluations', 0),
                envelope=dict(state=other, columns=slim, envelope_trace=trace,
                              interpolation=dict(unresolved_speed_intervals=unresolved_intervals(env2, slim))))


def level_flights(envelope):
    """S16: (speed, level-flight point) of every envelope column with one (NCurves.lf_rows: the level flight the chart's
    propeller-state rule starts from)."""
    out = []
    for c in envelope['columns']:
        p = next((p for p in c.get('points', []) if abs(p['load_g']-1.) <= 1e-9 and p.get('stationary_coordinates') is not None), None)
        if p is not None:out.append((float(c['speed_kmh']), p))
    return out


def curve_outline(cfg, row, tracer):
    """S16: (regions, speed unit, turn unit) of the drawn outline for the curve solver (NCurves.set_outline): the outline
    as the chart draws it (outline, from the envelope alone) as closed regions (outline_regions); a contour state more
    than one unit (the speed tolerance, the envelope's turn tolerance) outside them is no knot, and a piece drawn that
    far outside is split."""
    return outline_regions(outline(dict(settings=cfg), row)), float(tracer.vtol), float(tracer.ttol)


def outline_regions(boundary):
    """S16: the closed regions of a drawn outline (its runs, closed down to level flight). Runs the outline splits at a
    numerical gap or a Mach seam (the run before does not end on level flight and the next does not start there) are one
    region, joined straight across the gap: a gap is no absence proof, so a contour may pass it below that line."""
    runs, run = [], []
    for p in boundary+[dict(turn_dps=None)]:
        if p['turn_dps'] is not None:
            run.append((float(p['speed_kmh']), float(p['turn_dps'])));continue
        if run:runs.append(run)
        run = []
    joined = []
    for run in runs:
        if joined and joined[-1][-1][1] != 0. and run[0][1] != 0.:joined[-1] = joined[-1]+run
        else:joined.append(list(run))
    for run in joined:
        if run[-1][1] != 0.:run.append((run[-1][0], 0.))
        if run[0][1] != 0.:run.insert(0, (run[0][0], 0.))
    return joined


def curve_columns(envelope, tracer):
    """The verified columns the curve solver starts from. S15: not the column just inside the traced forward-IAS edge's
    corner (it draws the outline's end; the curve solver's column choice stays as before); S16: nor the engine RPM
    edge's."""
    return [c for c in envelope['columns'] if tracer.ok(c) and c.get('s1_role') not in ('ias corner', 'rpm corner')]


def aircraft_row(name, index, cfg, solver, envelope, curves, tracer):
    from em_solver import AIRCRAFT
    from em_sampling import outline_column
    from em_speed_limits import speed_limits
    from em_accuracy import turn_tolerance, speed_tolerance, coverage
    from em_mach_events import discontinuity_regions, aircraft_catalog
    from aircraft_catalog import load
    from aircraft_description import description
    from instructor_envelope import profile
    columns = sorted(envelope['columns'], key=lambda c: c['speed_kmh'])
    points = [p for c in columns for p in c['points']]
    mach = aircraft_catalog(load(name), cfg)
    gaps = unresolved_intervals(envelope, columns)
    components = [compact_component(c) for c in (curves or {}).get('components', [])]
    levels = coverage(cfg)['checked_sep_levels_mps']
    first = next((c for c in columns if c.get('mass')), None)
    return dict(AIRCRAFT[name], id=name, color=COLORS[index % 2], settings=cfg, aircraft_model=description(cfg),
                chart_solver='march', points=points, columns=columns,
                boundary_columns=[outline_column(c) for c in columns],
                instructor_approximation=profile(load(name), cfg) if cfg['instructor'] else None,
                instructor_unresolved_speeds_kmh=[], sweep_excluded_speeds_kmh=[],
                low_speed_edge=low_speed_edge(cfg, envelope, columns, tracer),
                speed_limit=speed_limits(solver.fm, cfg), sustained=[],
                mass=first['mass'] if first else solver.mass, engine=first['engine'] if first else solver.engine.summary,
                valid_points=sum(bool(p.get('valid')) for p in points), converged_points=sum(bool(p.get('converged')) for p in points),
                local_candidate_points=sum(bool(p.get('chart_usable')) for p in points),
                curve_components=components,
                curve_knots=[k for comp in (curves or {}).get('components', []) for k in comp['knots']],
                # S16 (Leo 2026-10-24): the SEP jump lines (NCurves.sep_jumps), drawn dotted; contours end at them
                **({'sep_jumps': curves['sep_jumps']} if curves and curves.get('sep_jumps') else {}),
                # Over-rev (Leo 2026-10-26): where an engine starts to over-rev
                **({'overrev_lines': overrev_lines(envelope, columns, (curves or {}).get('sep_jumps'))}
                   if overrev_row(solver, points) else {}),
                # Wing line (Leo 2026-11-10): the Instructor's wing protection state of this chart (wing_line.protection)
                **({'wing_protection': dict(state=solver.instructor_protection, threshold_s=WING.HOLD_S,
                                            release_start_s=None if math.isinf(solver.protection_release_s)
                                            else float(solver.protection_release_s))}
                   if getattr(solver, 'instructor_protection', None) else {}),
                # The complete curve solver record (knots, withheld checks, ends) for exports and replay.
                curves=None if curves is None else {k: v for k, v in curves.items() if k not in ('seeds',)},
                envelope_trace=dict(segments=envelope['segments'], mach_seams_kmh=envelope['mach_seams_kmh'],
                                    sweep_gaps_kmh=envelope.get('sweep_gaps_kmh', []),
                                    soft_limits=bool(getattr(solver, 'soft_admission', False)),
                                    # the wing rule was on: the wing breaking load is a line (columns' 'wing_line')
                                    **({'wing_line': WING.rule(solver)} if WING.RULE else {}),
                                    events=envelope['events'], checks=len(envelope['checks']),
                                    speed_limit=envelope['speed_limit'], turn_tolerance_dps=tracer.ttol,
                                    placement_tolerance_kmh=tracer.place_tol,
                                    **({'ias_trace': envelope['ias_trace']} if envelope.get('ias_trace') else {}),
                                    **({'rpm_trace': envelope['rpm_trace']} if envelope.get('rpm_trace') else {}),
                                    **({'overrev_lines': envelope['overrev_lines']} if envelope.get('overrev_lines') else {})),
                interpolation=dict(target_mps=cfg['sep_tolerance_mps'], target_contour_dps=turn_tolerance(cfg['sep_tolerance_mps']),
                                   target_speed_kmh=speed_tolerance(cfg), scope='contours', checked_sep_levels_mps=levels,
                                   unresolved_speed_intervals=gaps, mach_transition_intervals=[], mach_events=mach['events'],
                                   native_discontinuities=discontinuity_regions(mach, cfg),
                                   mach_approximations=mach.get('approximations', []),
                                   certified_speed_interiors=[], certified_boundary_intervals=[],
                                   envelope_turn_tolerance_dps=tracer.ttol, curve_tolerance_scale=CURVE_SCALE,
                                   # S9: only stretches the curve solver could not resolve; a contour leaving the envelope
                                   # (outcome 'excluded', both ends located) is listed apart (WT_MARCH_S9=0: S8's list)
                                   curve_unresolved_intervals=[dict(level=c['level'], interval_kmh=u['interval_kmh'],
                                                                    **({'outcome': u.get('outcome'), 'reason': str(u.get('reason'))[:160]}
                                                                       if S9 else {}))
                                                               for c in components for u in c['unresolved_intervals']
                                                               if not (S9 and u.get('outcome') == 'excluded')],
                                   **({'curve_exit_intervals': [dict(level=c['level'], interval_kmh=u['interval_kmh'])
                                                                for c in components for u in c['unresolved_intervals']
                                                                if u.get('outcome') == 'excluded']} if S9 else {}),
                                   # S10: a fold piece whose refinement did not close is not drawn (an unresolved stretch)
                                   **({'curve_unresolved_folds': [dict(level=c['level'], interval_kmh=[min(k[0] for k in c['fold']),
                                                                                                         max(k[0] for k in c['fold'])])
                                                                  for c in components if c.get('fold_unresolved')]} if S10 else {}),
                                   certification_scope='admitted full-model states; withheld exact checks; no global coverage certificate'))


def compact_component(comp):
    # S7: knots are [speed, load, Ps, left slope, right slope] (dw/dV per km/h, w = n^2-1; None where unavailable)
    return dict(level=comp['level'], segment_kmh=comp['segment_kmh'],
                knots=[[k['speed_kmh'], k['load_g'], k.get('ps_mps')]+(list(k['s7_w_slope']) if k.get('s7_w_slope') else [])
                       for k in comp['knots']],
                unresolved_intervals=[dict(interval_kmh=list(u['interval_kmh']), reason=str(u.get('reason'))[:120],
                                           **({'outcome': u.get('outcome')} if S9 and u.get('outcome') else {}))
                                      for u in comp.get('unresolved_intervals', [])],
                start=(comp.get('start') or {}).get('kind'), end=(comp.get('end') or {}).get('kind'),
                **({'steps': [list(u['interval_kmh']) for u in comp['steps']]} if comp.get('steps') else {}),
                # S10: the fold piece where the contour turns back in speed: knots by load, [speed, load, Ps, dV/dw]
                **({'fold': [[k['speed_kmh'], k['load_g'], k.get('ps_mps'), k.get('fold_dv_dw')] for k in comp['fold']['knots']]}
                   if comp.get('fold') else {}),
                **({'fold_unresolved': True} if (comp.get('fold') or {}).get('unresolved') else {}),
                # S16: where a piece meets an SEP jump line: [direction, speed, load, the speed of the knot drawn from]
                **({'jump_ends': [[j['direction'], j['speed_kmh'], j['load_g'], j['from_kmh']] for j in comp['jump_ends']]}
                   if comp.get('jump_ends') else {}),
                # over-rev: each knot's highest engine RPM against its RPMMaxAllowed (fold knots apart)
                **({'rpm': [k.get('engine_rpm_ratio') for k in comp['knots']]}
                   if any(k.get('engine_rpm_ratio') is not None for k in comp['knots']) else {}),
                **({'fold_rpm': [k.get('engine_rpm_ratio') for k in comp['fold']['knots']]}
                   if comp.get('fold') and any(k.get('engine_rpm_ratio') is not None for k in comp['fold']['knots']) else {}))


def ok(c):
    return c.get('boundary_status') in ('verified limit', 'plot ceiling') and bool(c.get('boundary'))


def unresolved_intervals(envelope, columns):
    """Speed intervals the trace could not resolve: between the neighbouring verified columns."""
    good = [c['speed_kmh'] for c in columns if ok(c)]
    out = []
    for e in envelope['events']:
        if e['kind'] == 'unresolved':
            v = e['speed_kmh']
            lo = max((x for x in good if x < v), default=None);hi = min((x for x in good if x > v), default=None)
            if lo is not None and hi is not None:out.append([lo, hi])
        elif e['kind'] == 'segment unresolved':
            out.append(list(e['speed_kmh']))
    for a, b, c in zip(columns, columns[1:], columns[2:]):
        if ok(a) and ok(c) and not ok(b) and b.get('boundary_status') != 'no feasible samples':out.append([a['speed_kmh'], c['speed_kmh']])
    return sorted({tuple(x): x for x in out}.values())


def low_speed_edge(cfg, envelope, columns, tracer):
    good = [c for c in columns if ok(c)]
    if not good:return None
    first = good[0]
    below = [c for c in columns if c['speed_kmh'] < first['speed_kmh']]
    edge = next((e for e in envelope['events'] if e['kind'] == 'edge' and abs(max(e['speed_kmh'])-first['speed_kmh']) < 1e-9
                 or e['kind'] == 'edge' and abs(min(e['speed_kmh'])-first['speed_kmh']) < 1e-9), None)
    clipped = abs(first['speed_kmh']-cfg['speed_min_kmh']) < 1e-8
    kind = 'speed-range edge' if clipped else 'low-speed feasibility edge'
    # S8: the first column at a segment start where the selected wing sweep becomes available
    sweep = any(s['ends'][0] == 'sweep_limit' and abs(s['speed_kmh'][0]-first['speed_kmh']) < 1e-9
                for s in envelope.get('segments') or ())
    if sweep:kind = 'Wing sweep limit'
    reason, load = first.get('boundary_reason'), first['boundary']['load_g']
    if not clipped and not sweep and below and below[-1].get('boundary_status') == 'no feasible samples':
        if reason == 'stall' and load <= 1.01:kind = 'stall-speed edge'
        elif reason == 'Instructor pitch' and load <= 1.01:kind = 'Instructor pitch minimum-speed edge'
        elif OVERREV and numerical_level(below[-1]):kind = UNRESOLVED_LEVEL_EDGE
    return dict(speed_kmh=first['speed_kmh'], kind=kind, bracket_kmh=[below[-1]['speed_kmh'], first['speed_kmh']] if below else None,
                refined=edge is not None, boundary_verified=True, method='alpha-march speed bracket',
                turn_dps=first['boundary']['turn_dps'], lower_load_g=1.,
                note='Vertical outline only; Ps exists only where a balanced equilibrium was solved')


# Over-rev follow-up: a chart's low-speed end where level flight below it is only numerically unresolved (no solve closes;
# no physical rejection) is no feasibility limit (Bf 109 F-4 RB at 1,500 m, 100% pitch: 217 km/h at 4,088 rpm)
UNRESOLVED_LEVEL_EDGE = 'unresolved level-flight edge'
NUMERICAL_REASONS = frozenset(('trim did not converge', 'propulsion did not settle', 'propulsion cycle unresolved',
                               'vertical step did not close', 'pitch response unresolved', 'Instructor boundary unresolved'))


def numerical_level(column):
    """The column's level flight was rejected for numerical reasons only (a correction or sub-solve that did not close)."""
    v = column['speed_kmh']
    level = next((p for p in column.get('points', []) if abs(p['load_g']-1.) <= 1e-9 and abs(p['speed_kmh']-v) <= 1e-9), None)
    reasons = set((level or {}).get('reasons') or ())
    return bool(reasons) and reasons <= NUMERICAL_REASONS


# ----- presentation (em_plot dispatches march aircraft here) -----------------------
def turn_rate(speed, load):
    return np.degrees(9.8100004196167*np.sqrt(np.maximum(0., np.asarray(load)**2-1.))/(np.asarray(speed)/3.6))


def runs(aircraft):
    """Runs of neighbouring verified columns that may be joined by one smooth outline piece."""
    columns = sorted(aircraft['columns'], key=lambda c: c['speed_kmh'])
    trace = aircraft.get('envelope_trace') or {}
    seams = trace.get('mach_seams_kmh', [])+trace.get('sweep_gaps_kmh', [])
    events = trace.get('events', [])
    gaps = [tuple(x) for x in aircraft.get('interpolation', {}).get('unresolved_speed_intervals', [])]
    # S8: segment ends (Mach seams, wing-sweep limits) are never drawn across
    ends = sorted({v for s in (trace.get('segments') or []) for v in s['speed_kmh']})
    out, run = [], []
    for c in columns:
        if not ok(c):
            if run:out.append(run)
            run = [];continue
        if run:
            a, b = run[-1]['speed_kmh'], c['speed_kmh']
            cut = (any(a <= x+1e-9 and b >= y-1e-9 or a < x < b or a < y < b for x, y in seams)
                   or any(a+1e-9 < v < b-1e-9 for v in ends)
                   or any(e['kind'] == 'edge' and a <= min(e['speed_kmh']) and b >= max(e['speed_kmh']) for e in events)
                   or any(e['kind'] == 'unresolved' and a < e['speed_kmh'] < b for e in events)
                   or any(lo < b and hi > a for lo, hi in gaps))
            if cut:
                out.append(run);run = []
        run.append(c)
    if run:out.append(run)
    return out


def face_load(v0, n0, s0, k0, v):
    """One limiting face beyond its column: load, slope and curvature there (a quadratic; the native face_load)."""
    return n0+(v-v0)*(s0+.5*k0*(v-v0))


def corner_vertex(va, na, sa, ka, vb, nb, sb, kb):
    """Speed of the corner of an interval whose ends lie on different limiting faces: where the two faces continued
    from their columns (face_load) meet, or None unless face a lies below face b at va and above it at vb (the
    boundary is the lower face, the first limit along alpha). The native corner_vertex (the same 60 bisection steps)."""
    if not all(np.isfinite(x) for x in (sa, sb, ka, kb)):return None
    tol = 1e-3*max(abs(na), abs(nb))
    fa, fb = na-face_load(vb, nb, sb, kb, va), face_load(va, na, sa, ka, vb)-nb
    if fa > tol or fb < -tol:return None
    if fa > 0.:return va    # within 0.1% in load of an end the corner is that end
    if fb < 0.:return vb
    lo, hi = va, vb
    for _ in range(60):
        mid = .5*(lo+hi)
        if face_load(va, na, sa, ka, mid)-face_load(vb, nb, sb, kb, mid) > 0.:hi = mid
        else:lo = mid
    return .5*(lo+hi)


def boundary_interval(va, na, sa, ka, pa, vb, nb, sb, kb, pb, corner, exact, v):
    """The S7 outline's load on [va, vb] (vectorized over v; the native interval_load): on one limiting face the cubic
    Hermite through loads and slopes (sa right of va, sb left of vb); across a face change the two faces up to their
    corner where both slopes are exact and the faces meet, else the Hermite with the PCHIP slopes pa, pb (no overshoot)."""
    v = np.asarray(v, dtype=float)
    if corner:
        vc = corner_vertex(va, na, sa, ka, vb, nb, sb, kb) if exact else None
        if vc is not None:
            return np.where(v <= vc, face_load(va, na, sa, ka, v), face_load(vb, nb, sb, kb, v))
        sa, sb = pa, pb
    h = vb-va;t = (v-va)/h;t2 = t*t;t3 = t2*t
    return (2*t3-3*t2+1)*na+(t3-2*t2+t)*h*sa+(-2*t3+3*t2)*nb+(t3-t2)*h*sb


def face_of(column):
    """The limiting face of a verified column: its boundary reason with the control axis or the engine (the native
    tracer's face_of)."""
    limit = (column.get('boundary') or {}).get('envelope_limit') or {}
    if limit.get('axis') is not None:return '%s:%s' % (column.get('boundary_reason'), limit['axis'])
    engine = (limit.get('engine_rpm_limit') or {}).get('engine')
    if engine is not None:return '%s:%s' % (column.get('boundary_reason'), engine)
    return column.get('boundary_reason')


def run_loads(run, xs):
    """Loads of one outline run at speeds xs: the S7 interpolant (boundary_interval, with the native tracer's
    interval_slopes rules) where its columns carry boundary slopes, PCHIP through the loads otherwise (S5/S6)."""
    from scipy.interpolate import PchipInterpolator
    speeds = np.array([c['speed_kmh'] for c in run], dtype=float);loads = np.array([c['boundary']['load_g'] for c in run], dtype=float)
    if len(run) == 1:return np.full(len(xs), loads[0])
    if not any(c.get('boundary_slope') for c in run):return PchipInterpolator(speeds, loads)(xs)
    pchip = pchip_slopes(speeds, loads)
    left = np.array([(c.get('boundary_slope') or [None, None])[0] for c in run], dtype=float)
    right = np.array([(c.get('boundary_slope') or [None, None])[1] for c in run], dtype=float)
    faces = [face_of(c) for c in run]
    exact_left, exact_right = np.isfinite(left), np.isfinite(right)
    # a missing exact slope: the same face's slope at the neighbouring column on that side (the native interval_slopes)
    fill_left, fill_right = left.copy(), right.copy()
    for k in range(len(run)):
        if not exact_right[k] and k >= 1 and faces[k-1] == faces[k] and exact_right[k-1]:fill_right[k] = right[k-1]
        if not exact_left[k] and k+1 < len(run) and faces[k+1] == faces[k] and exact_left[k+1]:fill_left[k] = left[k+1]
    exact_left, exact_right = np.isfinite(fill_left), np.isfinite(fill_right)
    left = np.where(exact_left, fill_left, pchip);right = np.where(exact_right, fill_right, pchip)
    out = np.empty(len(xs))
    i = np.clip(np.searchsorted(speeds, xs, side='right')-1, 0, len(run)-2)
    for k in np.unique(i).tolist():
        ka = (left[k]-right[k-1])/(speeds[k]-speeds[k-1]) if k >= 1 and faces[k-1] == faces[k] else 0.
        kb = (left[k+2]-right[k+1])/(speeds[k+2]-speeds[k+1]) if k+2 < len(run) and faces[k+2] == faces[k+1] else 0.
        sel = i == k
        out[sel] = boundary_interval(speeds[k], loads[k], right[k], ka, pchip[k], speeds[k+1], loads[k+1], left[k+1], kb,
                                     pchip[k+1], faces[k] != faces[k+1], bool(exact_right[k] and exact_left[k+1]), xs[sel])
    return out


def outline(data, aircraft):
    """Boundary outline in the production chart format (dense interpolant of limiting load per run, run_loads)."""
    cfg = data['settings']
    pieces = runs(aircraft)
    nx = 2*cfg.get('surface_resolution', 601)-1
    if data.get('preview'):nx = min(nx, 241)
    grid = np.linspace(cfg['speed_min_kmh'], cfg['speed_max_kmh'], nx)
    boundary = []
    # S8: a vertical edge where the selected wing sweep becomes unavailable inside the chart range (a segment end of
    # kind 'sweep_limit'; the low-speed edge and the speed limit draw their own)
    segments = (aircraft.get('envelope_trace') or {}).get('segments') or []
    low = (aircraft.get('low_speed_edge') or {}).get('speed_kmh')
    starts = {s['speed_kmh'][0] for s in segments if s['ends'][0] == 'sweep_limit'}
    stops = {s['speed_kmh'][1] for s in segments if s['ends'][1] == 'sweep_limit'}
    S15 = s15_on()
    trace = (aircraft.get('envelope_trace') or {}).get('ias_trace') if S15 else None
    if trace is not None and trace.get('status') != 'traced':trace = None
    if s16_on():
        # S16: the traced engine RPM edge (where both edges are traced, the one whose foot comes first)
        rpm = (aircraft.get('envelope_trace') or {}).get('rpm_trace')
        if rpm is not None and rpm.get('status') == 'traced' and (trace is None or rpm['foot_kmh'] < trace['foot_kmh']):
            trace = rpm
    # S16: where the engine RPM edge's trace failed, S8's edge: vertical where level flight first over-revs (the 'rpm
    # edge' foot, else the last column before the columns whose level flight over-revs). The columns' lower edges past it
    # are no guide to the iso-line, which can bulge back between them (He 51 A-1 SB: 47.6 deg/s at 448 km/h, 44.9 at 458)
    rpm_foot = None
    if s16_on():
        rpm = (aircraft.get('envelope_trace') or {}).get('rpm_trace')
        if rpm is not None and rpm.get('status') != 'traced':
            rpm_foot = next((max(e['speed_kmh']) for e in (aircraft.get('envelope_trace') or {}).get('events', [])
                             if e['kind'] == 'rpm edge'), math.nan)
    # S15: the traced edge closes the last run between the trace's start column and its corner
    host = None if trace is None else max((k for k, run in enumerate(pieces) if run[0]['speed_kmh'] <= trace['corner']['speed_kmh']+1e-3
                                           and run[-1]['speed_kmh'] >= trace['start_kmh']-1e-9), default=None)
    # runs wholly past the corner stand on the edge itself (the native trace checked that their columns end on the limit)
    skip = set() if host is None else {k for k, run in enumerate(pieces) if k > host
                                       and run[0]['speed_kmh'] > trace['corner']['speed_kmh']+1e-3}
    for k, run in enumerate(pieces):
        iso = None
        if k in skip:continue
        if k == host:
            # S15: the run that reaches the traced forward-IAS edge ends at its corner (columns past it stand on the
            # edge itself) and the edge closes it down to level flight
            # (a column within 0.001 km/h past it stands on the corner itself: an edge ending at the chart's speed range
            # reaches the range end only to the trace's tolerance)
            kept = [c for c in run if c['speed_kmh'] <= trace['corner']['speed_kmh']+1e-3]
            if kept:
                run = kept
                iso = traced_edge(aircraft, trace)
        # (S16: the run that reaches columns whose level flight over-revs ends at the foot when the RPM trace failed)
        cut = None
        if rpm_foot is not None:
            band = [c for c in run if c.get('lower_edge_kind') == 'engine RPM limit']
            below = [c['speed_kmh'] for c in run if band and c['speed_kmh'] < band[0]['speed_kmh'] and not c.get('lower_edge')]
            if below:cut = rpm_foot if below[-1] <= rpm_foot <= band[0]['speed_kmh'] else below[-1]
        speeds = np.array([c['speed_kmh'] for c in run])
        exact = {c['speed_kmh']: c for c in run}
        xs = np.unique(np.r_[grid[(grid > speeds[0]) & (grid < speeds[-1])], speeds])
        # S8: a discontinuity of the boundary (a column interval narrower than the speed tolerance whose error bound
        # fails: a 'step' event) is drawn vertical at its middle: no grid speeds inside, the middle twice
        steps = [e['speed_kmh'] for e in (aircraft.get('envelope_trace') or {}).get('events', [])
                 if e['kind'] == 'step' and speeds[0] <= e['speed_kmh'][0] and e['speed_kmh'][1] <= speeds[-1]]
        for lo, hi in steps:xs = xs[~((xs > lo) & (xs < hi))]
        loads = run_loads(run, xs)
        rates = turn_rate(xs, loads)
        if steps:
            add = []
            for lo, hi in steps:
                mid = .5*(lo+hi)
                add += [(mid, float(exact[lo]['boundary']['turn_dps']) if lo in exact else float(np.interp(lo, xs, rates))),
                        (mid, float(exact[hi]['boundary']['turn_dps']) if hi in exact else float(np.interp(hi, xs, rates)))]
            order = sorted([(x, 0, t) for x, t in zip(xs.tolist(), rates.tolist())]+[(x, 1+k, t) for k, (x, t) in enumerate(add)])
            xs = np.array([q[0] for q in order]);rates = np.array([q[2] for q in order])
        if cut is not None:
            keep = xs <= cut+1e-9
            xs, rates = xs[keep], rates[keep]
            if not len(xs) or xs[-1] < cut-1e-9:
                xs = np.r_[xs, cut];rates = np.r_[rates, turn_rate(np.array([cut]), run_loads(run, np.array([cut])))]
        index = np.searchsorted(speeds, xs)
        if run[0]['speed_kmh'] in starts and run[0]['speed_kmh'] != low:
            boundary.append(dict(speed_kmh=run[0]['speed_kmh'], turn_dps=0., at_plot_ceiling=False,
                                 edge_kind='Wing sweep limit', vertical_edge=True))
        for x, t, j in zip(xs.tolist(), rates.tolist(), index.tolist()):
            c = exact.get(x)
            if c is not None:
                p = dict(speed_kmh=x, turn_dps=float(c['boundary']['turn_dps']),
                         at_plot_ceiling=c['boundary_status'] == 'plot ceiling')
                if c.get('load_search_limit'):p['load_search_limit'] = c['load_search_limit']
            else:
                left, right = run[max(0, j-1)], run[min(len(run)-1, j)]
                p = dict(speed_kmh=x, turn_dps=t,
                         at_plot_ceiling=left['boundary_status'] == 'plot ceiling' and right['boundary_status'] == 'plot ceiling')
            boundary.append(p)
        if run[-1]['speed_kmh'] in stops:
            boundary[-1].update(edge_kind='Wing sweep limit', vertical_edge=True)
            boundary.append(dict(speed_kmh=run[-1]['speed_kmh'], turn_dps=0., at_plot_ceiling=False,
                                 edge_kind='Wing sweep limit', vertical_edge=True))
        # (S15: with a traced edge no run draws the S14 leaning edge: a run cut before the host is no edge of the limit)
        leaning = iso if iso is not None else [] if trace is not None else ias_edge(aircraft, run, grid)
        if cut is not None and boundary:
            boundary[-1].update(edge_kind=RPM_EDGE, vertical_edge=True)
            leaning = [dict(speed_kmh=float(cut), turn_dps=0., at_plot_ceiling=False, edge_kind=RPM_EDGE, vertical_edge=True)]
        boundary.extend(leaning)
        if S10 and not leaning:
            # S10: where level flight is physically rejected just above a run's last column (a high-speed edge: the
            # propellers brake, an engine over-revs at 1 g), the chart ends there: a vertical edge down to level flight.
            # S15: also past a forward-IAS limit with no edge drawn (no column landed past it and the trace failed)
            top = run[-1]['speed_kmh']
            # (S15: located by its columns even where the edge's own column stayed unresolved)
            ias = lambda e: 'forward-IAS' in str(e.get('reason'))+str(e.get('located_by'))
            edge = next((e for e in (aircraft.get('envelope_trace') or {}).get('events', []) if e['kind'] == 'edge'
                         and (e.get('status') == 'no feasible samples' or S15 and 'forward-IAS' in str(e.get('located_by')))
                         and abs(min(e['speed_kmh'])-top) < 1e-9
                         and max(e['speed_kmh']) > top+1e-9 and (S15 or 'forward-IAS' not in str(e.get('reason')))), None)
            if edge is not None and boundary and boundary[-1]['speed_kmh'] == top:
                # (S15: level flight that ends before reaching the limit, e.g. over-revving at 1 g, is no forward-IAS edge)
                early = 'level flight ends before the limit' in str(((aircraft.get('envelope_trace') or {}).get('ias_trace') or {}).get('status'))
                kind = ('Propeller braking edge' if 'propeller braking' in str(edge.get('reason')) else
                        forward_ias_kind(aircraft)+' (forward IAS) speed boundary' if ias(edge) and not early
                        else 'High-speed feasibility edge')
                boundary[-1].update(edge_kind=kind, vertical_edge=True)
                boundary.append(dict(speed_kmh=top, turn_dps=0., at_plot_ceiling=False, edge_kind=kind, vertical_edge=True))
        if k < len(pieces)-1 and not all(j in skip for j in range(k+1, len(pieces))):
            # An explicit gap: never a smooth line to the next run.
            nxt = pieces[k+1][0]['speed_kmh'];here = run[-1]['speed_kmh']
            seam = any(here <= x+1e-9 and nxt >= y-1e-9 for x, y in (aircraft.get('envelope_trace') or {}).get('mach_seams_kmh', []))
            gap = dict(speed_kmh=.5*(here+nxt), turn_dps=None, at_plot_ceiling=False)
            if seam:gap['edge_kind'] = 'Native Mach discontinuity'
            boundary.append(gap)
    edge = aircraft.get('low_speed_edge')
    if edge and boundary:
        at = next((i for i, p in enumerate(boundary) if p['speed_kmh'] == edge['speed_kmh'] and p['turn_dps'] is not None), None)
        if at is not None:
            boundary.insert(at, dict(speed_kmh=edge['speed_kmh'], turn_dps=0., at_plot_ceiling=False,
                                     edge_kind=edge['kind'], vertical_edge=True))
            boundary[at+1].update(edge_kind=edge['kind'], vertical_edge=True)
    redline = aircraft.get('speed_limit')
    if redline and 'vertical_limit' in redline and (aircraft.get('envelope_trace') or {}).get('soft_limits'):
        # S8: only a Mach or sweep limit is a vertical edge (a forward-IAS limit leans: ias_edge)
        vertical = redline['vertical_limit']
        redline = None if vertical is None else dict(vertical, enforced=True, sample_speed_kmh=max(0., vertical['speed_kmh']-.0036))
    if redline and redline['enforced'] and pieces:
        sample = redline['sample_speed_kmh'];column = next((c for c in pieces[-1] if c['speed_kmh'] == sample), None)
        if column is not None and cfg['speed_min_kmh'] <= redline['speed_kmh'] <= cfg['speed_max_kmh']:
            at = next((i for i, p in enumerate(boundary) if p['speed_kmh'] == sample and p['turn_dps'] is not None), None)
            if at is not None:
                common = dict(speed_kmh=redline['speed_kmh'], sample_speed_kmh=sample, at_plot_ceiling=False,
                              vertical_edge=True, edge_kind=redline['kind']+' speed boundary')
                # S15: down to the traced forward-IAS edge where it reaches this limit (below it the states are over the
                # forward-IAS limit), else to level flight
                foot = (trace['corner']['turn_dps'] if trace is not None and trace['corner']['speed_kmh'] >= sample-.01
                        else 0.)
                boundary[at+1:at+1] = [dict(common, turn_dps=column['boundary']['turn_dps']), dict(common, turn_dps=foot)]
    return boundary


def ias_edge(aircraft, run, grid):
    """S8: the leaning forward-IAS edge of one outline run: where level flight is over the lowest forward-IAS limit
    (wing VNE, flap retraction or damage by IAS) the columns carry their lower edge (the first state inside the limit
    along alpha) and its exact slope; the edge runs from the last column's lower edge back down through the others to
    level flight at the speed the trace located ('ias edge' event), the speed as the cubic Hermite in n^2-1 (vertical
    at level flight; PCHIP in load on the chart grid for records without slopes). [] without one."""
    from scipy.interpolate import PchipInterpolator
    lows = [c for c in run if c.get('lower_edge')]
    if not lows:return []
    kind = (aircraft.get('speed_limit') or {}).get('forward_ias_limits') or []
    kind = min(kind, key=lambda c: c['speed_kmh'])['kind'] if kind else 'Forward IAS limit'
    # S16: an engine RPM lower edge (level flight over-revs) has its own foot event ('rpm edge') and name
    rpm = s16_on() and lows[0].get('lower_edge_kind') == 'engine RPM limit'
    if rpm:lows = [c for c in lows if c.get('lower_edge_kind') == 'engine RPM limit']
    v = [c['speed_kmh'] for c in lows];n = [c['lower_edge']['load_g'] for c in lows]
    slopes = [c.get('lower_edge_slope') for c in lows]
    events = (aircraft.get('envelope_trace') or {}).get('events', [])
    one = next((e for e in events if e['kind'] == ('rpm edge' if rpm else 'ias edge') and min(e['speed_kmh']) <= v[0]+1e-9), None)
    if one is not None and max(one['speed_kmh']) < v[0]-1e-9:
        v.insert(0, float(max(one['speed_kmh'])));n.insert(0, 1.);slopes.insert(0, None)
    xs = np.unique(np.r_[grid[(grid > v[0]) & (grid < v[-1])], v])
    w = np.asarray(n, dtype=float)**2-1.
    # S14 (WT_MARCH_S14=0: S13): an edge not monotone in w is drawn as V(w) on its monotone stretch from level flight
    # (the native tracer's lower_split judges that stretch) and through the remaining columns in load as before
    m = len(v)
    if S10 and all(os.environ.get(k, '1') != '0' for k in ('WT_MARCH_S11', 'WT_MARCH_S12', 'WT_MARCH_S13', 'WT_MARCH_S14')):
        m = 1
        while m < len(v) and w[m] > w[m-1]:m += 1
    if len(v) > 1 and any(s is not None for s in slopes[:m]) and (np.all(np.diff(w) > 0.) if m == len(v) else m >= 2):
        # the edge's exact slopes: the speed along the limit as the cubic Hermite V(w), w = n^2-1, with dV/dw = 1/slope
        # at the columns and 0 at level flight (where the edge leaves the 1 g line vertically); the native tracer's
        # lower_split judges this drawing. Sampled in w (with the columns) instead of on the speed grid.
        d = [0. if k == 0 and n[0] == 1. else (1./s if s is not None and s > 0. else None) for k, s in enumerate(slopes[:m])]
        wm, vm = w[:m], np.asarray(v[:m], dtype=float)
        dl, dr = hermite_slopes(wm, vm, d, d)
        wq = np.unique(np.r_[np.linspace(wm[0], wm[-1], 121), wm])
        xs = hermite(wm, vm, dl, dr, wq);loads = np.sqrt(1.+np.maximum(wq, 0.))
        if not s15_on():
            # S15: kept in w order (an edge that turns back in speed was drawn as a zigzag in speed order)
            order = np.argsort(xs, kind='stable');xs, loads = xs[order], loads[order]
        if m < len(v):
            tail = np.unique(np.r_[grid[(grid > v[m-1]) & (grid < v[-1])], v[m:]])
            xs = np.r_[xs, tail];loads = np.r_[loads, PchipInterpolator(v[m-1:], n[m-1:])(tail)]
    else:
        loads = PchipInterpolator(v, n)(xs) if len(v) > 1 else np.array(n)
    rates = turn_rate(xs, loads)
    common = dict(at_plot_ceiling=False, edge_kind=RPM_EDGE if rpm else kind+' (forward IAS) speed boundary', leaning_edge=True)
    out = [dict(common, speed_kmh=float(x), turn_dps=float(t)) for x, t in zip(xs[::-1].tolist(), rates[::-1].tolist())]
    if n[0] > 1.:out.append(dict(common, speed_kmh=float(v[0]), turn_dps=0.))
    return out


RPM_EDGE = 'Engine RPM limit (over-rev) speed boundary'   # S16: the leaning engine RPM edge


def forward_ias_kind(aircraft):
    """The lowest forward-IAS limit's kind (VNE, flap damage or flap retraction by IAS)."""
    kinds = (aircraft.get('speed_limit') or {}).get('forward_ias_limits') or []
    return min(kinds, key=lambda c: c['speed_kmh'])['kind'] if kinds else 'Forward IAS limit'


def traced_edge(aircraft, trace):
    """S15: the forward-IAS edge from the native trace (NTracer.ias_trace): speed and w = n^2-1 along the limit's
    iso-line as cubic Hermites in body alpha through its knots and their slopes (missing slopes as hermite_slopes; the
    drawing the native split rule judged), from the corner where it meets the first hard limit down to level flight, in
    that order (where level flight has negative body alpha the edge turns back to lower speeds before it leans out: the
    over-limit band lies outside the outline)."""
    knots = trace['knots']
    a = np.array([k[4] for k in knots], dtype=float)
    v = np.array([k[0] for k in knots], dtype=float);w = np.array([k[1] for k in knots], dtype=float)**2-1.
    vl, vr = hermite_slopes(a, v, [k[2] for k in knots], [k[2] for k in knots])
    wl, wr = hermite_slopes(a, w, [k[3] for k in knots], [k[3] for k in knots])
    aq = np.unique(np.r_[np.linspace(a[0], a[-1], 241), a])[::-1]
    xs = hermite(a, v, vl, vr, aq);wq = hermite(a, w, wl, wr, aq)
    rates = turn_rate(xs, np.sqrt(1.+np.maximum(wq, 0.)))
    # (S16: the engine RPM edge, traced the same way)
    kind = RPM_EDGE if trace.get('limit') == 'engine RPM' else forward_ias_kind(aircraft)+' (forward IAS) speed boundary'
    common = dict(at_plot_ceiling=False, edge_kind=kind, leaning_edge=True)
    out = [dict(common, speed_kmh=float(x), turn_dps=float(t)) for x, t in zip(xs.tolist(), rates.tolist())]
    if knots[0][1] > 1.+1e-9:
        # S16: an engine RPM edge whose iso-line came back to its foot speed above level flight (NTracer.rpm_return):
        # vertical from there down to level flight
        out.append(dict(common, speed_kmh=float(knots[0][0]), turn_dps=0.))
    else:
        out[-1]['turn_dps'] = 0.
    return out


def pchip_slopes(x, y):
    """The PCHIP derivatives at the knots (PchipInterpolator(x, y).derivative()(x))."""
    from scipy.interpolate import PchipInterpolator
    if len(x) < 2:return np.zeros(len(x))
    return PchipInterpolator(x, y).derivative()(x)


def hermite_slopes(x, y, left, right):
    """(left, right) knot slopes with the missing ones (None/NaN) taken from PCHIP; an end knot in level flight
    (y = w = 0, no exact slope there) takes the slope of the parabola through both ends of its interval with the
    neighbour's slope. The rule of the native fill_slopes."""
    dl = np.array([np.nan if v is None else v for v in left], dtype=float)
    dr = np.array([np.nan if v is None else v for v in right], dtype=float)
    missing = ~(np.isfinite(dl) & np.isfinite(dr))
    if missing.any():
        n = len(x)
        lend = n >= 2 and y[0] == 0. and not np.isfinite(dl[0]) and not np.isfinite(dr[0])
        rend = n >= 2 and y[-1] == 0. and not np.isfinite(dl[-1]) and not np.isfinite(dr[-1])
        d = pchip_slopes(x, y)
        both = ~np.isfinite(dl) & ~np.isfinite(dr)
        dl[both] = d[both];dr[both] = d[both]
        dl = np.where(np.isfinite(dl), dl, dr);dr = np.where(np.isfinite(dr), dr, dl)
        if lend:
            dl[0] = 2.*(y[1]-y[0])/(x[1]-x[0])-dl[1];dr[0] = dl[0]
        if rend:
            dr[-1] = 2.*(y[-1]-y[-2])/(x[-1]-x[-2])-dr[-2];dl[-1] = dr[-1]
    return dl, dr


def hermite(x, y, dl, dr, v):
    """Piecewise cubic Hermite through (x, y): on [x[k], x[k+1]] the slopes dr[k] and dl[k+1]; vectorized over v
    (the native hermite_eval)."""
    x = np.asarray(x, dtype=float);y = np.asarray(y, dtype=float);v = np.asarray(v, dtype=float)
    if len(x) == 1:return np.full(v.shape, y[0])
    i = np.clip(np.searchsorted(x, v, side='right')-1, 0, len(x)-2)
    h = x[i+1]-x[i];t = (v-x[i])/h;t2 = t*t;t3 = t2*t
    return (2*t3-3*t2+1)*y[i]+(t3-2*t2+t)*h*dr[i]+(-2*t3+3*t2)*y[i+1]+(t3-t2)*h*dl[i+1]


def curve_interpolant(knots):
    """w(V) of compact curve knots: the S7 Hermite where the knots carry slopes, PCHIP otherwise (S5/S6 records)."""
    from scipy.interpolate import PchipInterpolator
    v = np.array([k[0] for k in knots], dtype=float);w = np.array([k[1]**2-1. for k in knots], dtype=float)
    if all(len(k) < 5 for k in knots):return PchipInterpolator(v, w)
    dl, dr = hermite_slopes(v, w, [k[3] if len(k) >= 5 else None for k in knots], [k[4] if len(k) >= 5 else None for k in knots])
    return lambda x: hermite(v, w, dl, dr, x)


def polyline(component, samples=200):
    """Drawable pieces of one compact curve component: the curve interpolant in n^2-1 through its knots
    (curve_interpolant), split at unresolved knot intervals (as em_march_curves.polyline)."""
    knots = component['knots']
    if not knots:return []
    if len(knots) == 1:
        return [[(knots[0][0], float(turn_rate(knots[0][0], knots[0][1])))]]+(
            [fold_path(component['fold'])] if component.get('fold') and not component.get('fold_unresolved') else [])
    v = np.array([k[0] for k in knots], dtype=float)
    f = curve_interpolant(knots);xs = np.unique(np.r_[np.linspace(v[0], v[-1], samples), v])
    turns = turn_rate(xs, np.sqrt(1.+np.maximum(np.asarray(f(xs), dtype=float), 0.)))
    open_ = [tuple(u['interval_kmh']) for u in component.get('unresolved_intervals', [])]
    inside = np.zeros(len(xs), dtype=bool)
    for lo, hi in open_:inside |= (xs > lo) & (xs < hi)
    # S8: a discontinuity of the level set (a knot interval narrower than the speed tolerance whose error bound fails)
    # is drawn as a vertical step at its middle
    steps = [tuple(u) for u in component.get('steps', [])]
    if steps:
        keep = np.ones(len(xs), dtype=bool)
        extra = []
        for lo, hi in steps:
            keep &= ~((xs > lo) & (xs < hi))
            mid = .5*(lo+hi)
            extra += [(mid, float(np.interp(lo, xs, turns))), (mid, float(np.interp(hi, xs, turns)))]
        pts = sorted([(x, 0, t) for x, t in zip(xs[keep].tolist(), turns[keep].tolist())]
                     + [(x, 1+k, t) for k, (x, t) in enumerate(extra)])
        xs = np.array([q[0] for q in pts]);turns = np.array([q[2] for q in pts])
        inside = np.zeros(len(xs), dtype=bool)
        for lo, hi in open_:inside |= (xs > lo) & (xs < hi)
    pieces, piece = [], []
    for x, t, cut in zip(xs.tolist(), turns.tolist(), inside.tolist()):
        if cut:
            if piece:pieces.append(piece)
            piece = [];continue
        piece.append((x, t))
    if piece:pieces.append(piece)
    # S16: a piece that meets an SEP jump line ends on it (the point where its side's SEP on the line equals the level)
    for direction, speed, load, frm in component.get('jump_ends', []):
        ends = [(abs((p[-1] if direction > 0 else p[0])[0]-frm), k) for k, p in enumerate(pieces) if p]
        if not ends:continue
        gap, k = min(ends)
        if gap > 1e-6:continue
        point = (float(speed), float(turn_rate(speed, load)))
        if direction > 0:pieces[k].append(point)
        else:pieces[k].insert(0, point)
    if component.get('fold') and not component.get('fold_unresolved'):pieces.append(fold_path(component['fold']))
    return pieces


def fold_path(knots, samples=40):
    """S10: the drawn fold piece of a contour that turns back in speed (NCurves.check_fold): the speed as the cubic
    Hermite V(w) through the fold knots [speed, load, Ps, dV/dw] in w = n^2-1 (PCHIP slopes where none was solved), so
    the line turns back smoothly (dV/dw = 0 at the turn); points in w order, from the lower load to the upper."""
    ks = sorted(knots, key=lambda k: k[1])
    w = np.array([k[1]**2-1. for k in ks], dtype=float);v = np.array([k[0] for k in ks], dtype=float)
    if len(ks) == 1:return [(float(v[0]), float(turn_rate(v[0], ks[0][1])))]
    dl, dr = hermite_slopes(w, v, [k[3] for k in ks], [k[3] for k in ks])
    wq = np.unique(np.r_[np.linspace(w[0], w[-1], samples+1), w])
    vq = hermite(w, v, dl, dr, wq)
    tq = turn_rate(vq, np.sqrt(1.+np.maximum(wq, 0.)))
    return [(float(a), float(b)) for a, b in zip(vq.tolist(), tq.tolist())]


def contours(aircraft, levels=None):
    """Contour paths of the calculated levels (``levels``: the subset to draw)."""
    checked = set(aircraft.get('interpolation', {}).get('checked_sep_levels_mps', []))
    wanted = checked if levels is None else checked & {float(v) for v in levels}
    out = []
    # Wing line (Leo 2026-11-10): a path is split where it crosses the drawn wing line; the pieces above carry past_wing
    at = wing_turns(aircraft)
    for comp in aircraft.get('curve_components', []):
        if comp['level'] not in wanted:continue
        pieces = polyline(comp)
        fold = len(pieces)-1 if comp.get('fold') and not comp.get('fold_unresolved') else -1
        for i, piece in enumerate(pieces):
            if len(piece) > 1:
                path = dict(level=float(comp['level']), x=[p[0] for p in piece], y=[p[1] for p in piece])
                if OVERREV:contour_overrev(path, comp, i == fold)
                out += [path] if at is None else split_past_wing(path, at)
    return out


def load_of(speed, turn):
    """The load of a turn rate (deg/s) at a speed (km/h): the inverse of turn_rate."""
    w = np.radians(np.asarray(turn, dtype=float))*np.asarray(speed, dtype=float)/3.6/9.8100004196167
    return np.sqrt(1.+w*w)


def overrev_points(path, ratio):
    """Over-rev: a drawn path's per-point 'percent_over' (RPM over RPMMaxAllowed, %) and 'overrev_s' (seconds the game
    gives the engine at that RPM, overrev.seconds_to_stop), None where within the limit; nothing where all are."""
    from overrev import seconds_to_stop
    over = np.isfinite(ratio) & (ratio > 1.+1e-6)
    if not over.any():return
    path['percent_over'] = [round(100.*(r-1.), 3) if o else None for r, o in zip(ratio.tolist(), over.tolist())]
    path['overrev_s'] = [seconds_to_stop(r) if o else None for r, o in zip(ratio.tolist(), over.tolist())]


def contour_overrev(path, comp, fold):
    """Over-rev: a contour path's over-rev points from its knots' RPM ratios (compact_component 'rpm', 'fold_rpm'):
    linear in speed between knots (a fold piece: in load)."""
    if fold:
        pairs = sorted((k[1], r) for k, r in zip(comp['fold'], comp.get('fold_rpm') or []) if r is not None)
        x = load_of(path['x'], path['y'])
    else:
        pairs = sorted((k[0], r) for k, r in zip(comp['knots'], comp.get('rpm') or []) if r is not None)
        x = np.asarray(path['x'], dtype=float)
    if not pairs:return
    overrev_points(path, np.interp(x, [a for a, _ in pairs], [r for _, r in pairs]))


def overrev_row(solver, points):
    """Over-rev: the chart row carries the over-rev record (a propeller aircraft under the rule); every point row gets
    its own (overrev.annotate)."""
    if not (OVERREV and getattr(solver, 'is_prop', False) and getattr(solver, 'soft_admission', False)):return False
    from overrev import annotate
    for p in points:annotate(p)
    return True


def iso_path(knots, samples=240):
    """An over-rev line from the native trace (NTracer.overrev_trace): speed and w = n^2-1 as cubic Hermites in body
    alpha through its knots [speed, load, dV/dalpha, dw/dalpha, alpha] (missing slopes as hermite_slopes), from level
    flight to where it meets the envelope's boundary."""
    a = np.array([k[4] for k in knots], dtype=float)
    v = np.array([k[0] for k in knots], dtype=float);w = np.array([k[1] for k in knots], dtype=float)**2-1.
    vl, vr = hermite_slopes(a, v, [k[2] for k in knots], [k[2] for k in knots])
    wl, wr = hermite_slopes(a, w, [k[3] for k in knots], [k[3] for k in knots])
    aq = np.unique(np.r_[np.linspace(a[0], a[-1], samples+1), a])
    xs = hermite(a, v, vl, vr, aq);wq = hermite(a, w, wl, wr, aq)
    rates = turn_rate(xs, np.sqrt(1.+np.maximum(wq, 0.)))
    if knots[0][1] <= 1.+1e-9:rates[0] = 0.
    return [float(x) for x in xs], [float(t) for t in rates]


OVERREV_NEAR_DPS = 2.   # a column's crossing within this of a drawn line at its speed lies on that line
OVERREV_SIDE = .005     # a column sample within this of RPMMaxAllowed (as a fraction) lies on either side of it


def column_rows(column):
    """Over-rev: a verified column's admitted samples with an RPM ratio, by load."""
    from overrev import rpm_ratio
    v = column['speed_kmh']
    return sorted((p for p in column.get('points', []) if abs(p['speed_kmh']-v) <= 1e-9 and p.get('valid')
                   and rpm_ratio(p) is not None), key=lambda p: p['load_g'])


def column_crossings(column):
    """Over-rev: where a verified column's admitted samples cross RPMMaxAllowed: (load, turn rate of the sample below,
    of the sample above), the load linear in rpm / limit between them."""
    from overrev import rpm_ratio
    rows = column_rows(column)
    out = []
    for a, b in zip(rows, rows[1:]):
        ra, rb = rpm_ratio(a)-1., rpm_ratio(b)-1.
        if (ra > 0.) != (rb > 0.) and b['load_g'] > a['load_g']:
            out.append((a['load_g']+(b['load_g']-a['load_g'])*ra/(ra-rb), a['turn_dps'], b['turn_dps']))
    return out


def turns_at(speed, line):
    """The turn rates where a drawn line passes a speed (it can pass one more than once)."""
    xs, ts = line['speed_kmh'], line['turn_dps']
    return [t0 if x1 == x0 else t0+(t1-t0)*(speed-x0)/(x1-x0)
            for x0, x1, t0, t1 in zip(xs, xs[1:], ts, ts[1:]) if min(x0, x1)-1e-9 <= speed <= max(x0, x1)+1e-9]


def on_lines(speed, crossing, lines):
    """A column's crossing lies on a drawn line: the line passes between its two samples (or within OVERREV_NEAR_DPS)."""
    _, lo, hi = crossing
    return any(lo-OVERREV_NEAR_DPS <= t <= hi+OVERREV_NEAR_DPS for line in lines for t in turns_at(speed, line))


def on_chart(line, columns):
    """Over-rev follow-up: a traced line piece lies on the chart's own propeller states: at every verified column inside
    its speed span the column's samples either side of it (by turn rate) are not both on one side of RPMMaxAllowed. The
    propeller can be bistable (the game's over-rev torque holds a branch over the limit), and the trace can follow a
    branch the chart's columns are not on (Bf 109 F-4 RB at 100% pitch, 1,500 m: an iso-line at 665-700 km/h where the
    chart's level flight runs 10-18% over)."""
    from overrev import rpm_ratio
    xs = line['speed_kmh']
    for c in columns:
        v = c['speed_kmh']
        if not min(xs)+1e-6 < v < max(xs)-1e-6:continue
        rows = column_rows(c)
        for t in turns_at(v, line):
            below = [p for p in rows if p['turn_dps'] <= t];above = [p for p in rows if p['turn_dps'] >= t]
            if not below or not above:continue
            ra, rb = rpm_ratio(below[-1])-1., rpm_ratio(above[0])-1.
            if min(ra, rb) > OVERREV_SIDE or max(ra, rb) < -OVERREV_SIDE:return False
    return True


def jump_pieces(jumps):
    """Over-rev follow-up: the stretches of the SEP jump lines (NCurves.sep_jumps) where the propeller jumps across
    RPMMaxAllowed (the engine RPM over it on one side, within it on the other): there over-rev starts at the jump."""
    out = []
    for j in jumps or []:
        rb, ra = j.get('rpm_ratio_before'), j.get('rpm_ratio_after')
        if not rb or not ra:continue
        piece = []
        for v, t, b, a in zip(j['speed_kmh'], j['turn_dps'], rb, ra):
            if b is not None and a is not None and (b > 1.) != (a > 1.):
                piece.append((v, t))
                continue
            if len(piece) >= 2:out.append(piece)
            piece = []
        if len(piece) >= 2:out.append(piece)
    return out


def overrev_lines(envelope, columns, jumps=None):
    """Over-rev (option C): the lines where an engine starts to over-rev (RPM = RPMMaxAllowed) inside the envelope, each
    {'speed_kmh', 'turn_dps', 'rpm_limit', 'engine', 'traced', 'kind', 'stop_after_s'}:
    - 'iso': the native trace's iso-lines from level flight (NTracer.overrev_trace; in pieces where they jump), each kept
      where it lies on the chart's own propeller states (on_chart);
    - 'jump': the SEP jump lines' stretches where the propeller jumps across the limit (jump_pieces);
    - 'link': where level flight itself jumps across the limit between two columns (the trace's 'level_jump_kmh'), a
      straight piece from there at 1 g to the nearest end of such a jump line (traced False: approximate);
    - 'columns': straight pieces through the verified columns' own crossings that no other line explains
      (column_crossings; traced False: approximate).
    'stop_after_s': [[percent over RPMMaxAllowed, seconds until the game stops the engine]]."""
    from overrev import report_table
    table = report_table()
    good = sorted((c for c in columns if ok(c)), key=lambda c: c['speed_kmh'])
    limit = next((float(l) for c in good for p in c.get('points', [])
                  for l in [((p.get('propulsion') or {}).get('engine_rpm_threshold') or [None])[0]] if l), None)
    lines = []
    for rec in envelope.get('overrev_lines') or []:
        if rec.get('status') != 'traced' or len(rec.get('knots') or []) < 2:continue
        # (where the iso-line jumps, the propeller changing state across the limit, it is drawn in pieces)
        pieces, piece = [], [rec['knots'][0]]
        for k0, k1 in zip(rec['knots'], rec['knots'][1:]):
            if any(abs(k0[4]-a) <= 1e-9 and abs(k1[4]-b) <= 1e-9 for a, b in rec.get('jumps_alpha_deg') or []):
                pieces.append(piece);piece = []
            piece.append(k1)
        pieces.append(piece)
        for piece in pieces:
            if len(piece) < 2:continue
            xs, ts = iso_path(piece)
            line = dict(speed_kmh=xs, turn_dps=ts, rpm_limit=rec.get('rpm_limit'), engine=rec.get('engine'), traced=True,
                        kind='iso', stop_after_s=table)
            if on_chart(line, good):lines.append(line)
            else:rec.setdefault('off_chart_pieces_kmh', []).append([xs[0], xs[-1]])
    pieces = jump_pieces(jumps)
    for piece in pieces:
        lines.append(dict(speed_kmh=[float(v) for v, _ in piece], turn_dps=[float(t) for _, t in piece], rpm_limit=limit,
                          engine=None, traced=True, kind='jump', stop_after_s=table))
    for rec in envelope.get('overrev_lines') or []:
        if not rec.get('level_jump_kmh') or not pieces:continue
        vm = .5*sum(rec['level_jump_kmh'])
        width = abs(rec['edge_kmh'][1]-rec['edge_kmh'][0]) if rec.get('edge_kmh') else 0.
        # (the jump line's end nearest the level-flight jump, within the columns' spacing of it)
        end = min((p for piece in pieces for p in (min(piece, key=lambda q: q[1]),)), key=lambda q: abs(q[0]-vm))
        if abs(end[0]-vm) <= max(width, 1e-6) and end[1] > 0.:
            lines.append(dict(speed_kmh=[vm, float(end[0])], turn_dps=[0., float(end[1])], rpm_limit=limit, engine=None,
                              traced=False, kind='link', stop_after_s=table))
    drawn = list(lines)
    open_, done = [], []
    for c in good:
        v = c['speed_kmh']
        loads = [x[0] for x in column_crossings(c) if not on_lines(v, x, drawn)]
        if open_ and len(loads) == len(open_):
            for path, n in zip(open_, loads):path.append((v, n))
        else:
            done += open_
            open_ = [[(v, n)] for n in loads]
    done += open_
    for path in done:
        if len(path) < 2:continue
        lines.append(dict(speed_kmh=[float(v) for v, _ in path], turn_dps=[float(turn_rate(v, n)) for v, n in path],
                          rpm_limit=limit, engine=None, traced=False, kind='columns', stop_after_s=table))
    return lines


def outline_overrev(aircraft):
    """Over-rev: the outline points' over-rev record (overrev_points), the verified columns' RPM ratios linear in
    speed: their boundary states' for points above level flight, their level flights' for points on it."""
    from overrev import rpm_ratio
    good = sorted((c for c in aircraft.get('columns', []) if ok(c)), key=lambda c: c['speed_kmh'])
    top = [(c['speed_kmh'], rpm_ratio(c['boundary'])) for c in good if rpm_ratio(c['boundary']) is not None]
    level = [(c['speed_kmh'], rpm_ratio(p)) for c in good for p in c.get('points', [])
             if abs(p['load_g']-1.) <= 1e-9 and abs(p['speed_kmh']-c['speed_kmh']) <= 1e-9 and rpm_ratio(p) is not None]
    points = [p for p in aircraft.get('boundary', []) if p.get('turn_dps') is not None]
    if not points or not top:return
    v = np.array([p['speed_kmh'] for p in points], dtype=float)
    r = np.interp(v, [a for a, _ in top], [b for _, b in top])
    if level:
        flat = np.array([p['turn_dps'] == 0. for p in points])
        r = np.where(flat, np.interp(v, [a for a, _ in level], [b for _, b in level]), r)
    record = {}
    overrev_points(record, r)
    for k, p in enumerate(points):
        if record.get('percent_over') and record['percent_over'][k] is not None:
            p['percent_over'] = record['percent_over'][k];p['overrev_s'] = record['overrev_s'][k]


# ----- wing line and Instructor wing protection (Leo 2026-11-10, wing_line.py) -----------------------
def chart_grid(data):
    """The chart's dense speed grid (as outline's)."""
    cfg = data['settings']
    nx = 2*cfg.get('surface_resolution', 601)-1
    if data.get('preview'):nx = min(nx, 241)
    return np.linspace(cfg['speed_min_kmh'], cfg['speed_max_kmh'], nx)


def edge_trace(aircraft):
    """The traced forward-IAS or engine RPM edge that closes the outline's last run (outline's S15/S16 choice), or None."""
    trace = aircraft.get('envelope_trace') or {}
    edge = trace.get('ias_trace') if s15_on() else None
    if edge is not None and edge.get('status') != 'traced':edge = None
    rpm = trace.get('rpm_trace') if s16_on() else None
    if rpm is not None and rpm.get('status') == 'traced' and (edge is None or rpm['foot_kmh'] < edge['foot_kmh']):edge = rpm
    return edge


def drawn_runs(aircraft):
    """runs(aircraft) as outline draws them: the run that reaches a traced edge (edge_trace) ends at the edge's corner
    (the columns past it stand on the edge itself) and the runs wholly past the corner are left out."""
    pieces, edge = runs(aircraft), edge_trace(aircraft)
    if edge is None:return pieces
    corner = edge['corner']['speed_kmh']+1e-3
    host = max((k for k, run in enumerate(pieces) if run[0]['speed_kmh'] <= corner
                and run[-1]['speed_kmh'] >= edge['start_kmh']-1e-9), default=None)
    if host is None:return pieces
    out = []
    for k, run in enumerate(pieces):
        if k > host and run[0]['speed_kmh'] > corner:continue
        out.append(([c for c in run if c['speed_kmh'] <= corner] or run) if k == host else run)
    return out


def boundary_ratio(column):
    """A column's wing load ratio (the highest of wing_load_ratios) at its boundary state; NaN where it carries none."""
    ratios = (column.get('boundary') or {}).get('wing_load_ratios')
    return float(max(ratios)) if ratios else math.nan


def wing_knot(column):
    """The wing-line load of a verified column (column['wing_line']), kept between its lower edge (level flight over a
    soft limit: the states below it lie outside the outline) and its boundary; None without a record."""
    load = (column.get('wing_line') or {}).get('load_g')
    if load is None or not math.isfinite(load):return None
    low = max(1., float((column.get('lower_edge') or {}).get('load_g') or 1.))
    return min(max(float(load), low), float(column['boundary']['load_g']))


def wing_outline(data, aircraft):
    """The drawn wing line (aircraft['wing_line']): where the wing load ratio is 1 inside the outline. Per drawn outline
    run and stretch of neighbouring columns with a wing-line point: PCHIP in load through the points, never above the
    outline's own load (run_loads). Where the neighbouring column of the run has no point and the boundary's wing load
    ratio passes 1 between the two, the line starts on the boundary at the speed where that ratio (linear in speed) is
    1. Points dict(speed_kmh, load_g, turn_dps, break_s_at_boundary: the game's expected seconds to break the wing
    held at the boundary above, wing_line.break_seconds, None where the boundary ratio is not above 1); a
    dict(speed_kmh, turn_dps=None) between stretches. Rebuilt from the columns on every call."""
    from scipy.interpolate import PchipInterpolator
    grid = chart_grid(data)
    out = []
    for run in drawn_runs(aircraft):
        speeds = np.array([c['speed_kmh'] for c in run], dtype=float)
        ratios = np.array([boundary_ratio(c) for c in run], dtype=float)
        known = np.isfinite(ratios)
        marks = [wing_knot(c) for c in run]
        k = 0
        while k < len(run):
            if marks[k] is None:
                k += 1;continue
            j = k
            while j+1 < len(run) and marks[j+1] is not None:j += 1
            knots = [(float(speeds[i]), marks[i]) for i in range(k, j+1)]
            for near, far in ((k, k-1), (j, j+1)):
                if not 0 <= far < len(run) or not ratios[far] <= 1. < ratios[near]:continue
                v = float(speeds[far]+(speeds[near]-speeds[far])*(1.-ratios[far])/(ratios[near]-ratios[far]))
                if abs(v-speeds[near]) < 1e-9:continue
                point = (v, float(run_loads(run, np.array([v]))[0]))
                if far < near:knots.insert(0, point)
                else:knots.append(point)
            k = j+1
            if len(knots) < 2:continue
            kv = np.array([q[0] for q in knots], dtype=float);kn = np.array([q[1] for q in knots], dtype=float)
            xs = np.unique(np.r_[grid[(grid > kv[0]) & (grid < kv[-1])], kv])
            loads = np.maximum(1., np.minimum(PchipInterpolator(kv, kn)(xs), run_loads(run, xs)))
            rates = turn_rate(xs, loads)
            top = np.interp(xs, speeds[known], ratios[known]) if known.any() else np.full(len(xs), np.nan)
            if not (np.isfinite(loads).all() and np.isfinite(rates).all()):continue
            if out:out.append(dict(speed_kmh=float(.5*(out[-1]['speed_kmh']+xs[0])), turn_dps=None))
            out += [dict(speed_kmh=float(x), load_g=float(n), turn_dps=float(t),
                         break_s_at_boundary=WING.break_seconds(r) if r == r else None)
                    for x, n, t, r in zip(xs.tolist(), loads.tolist(), rates.tolist(), top.tolist())]
    return out


def outline_wing(aircraft):
    """Wing line: the outline points held past the wing's breaking load get 'wing_load_ratio' (the verified columns'
    boundary ratios, linear in speed inside each run) and 'wing_break_s' (wing_line.break_seconds); points with a ratio
    at or below 1, gaps, leaning edges and the feet of vertical edges get nothing (the twin of outline_overrev)."""
    points = [p for p in aircraft.get('boundary', []) if p.get('turn_dps') is not None and not p.get('leaning_edge')
              and not (p.get('vertical_edge') and p['turn_dps'] == 0.)]
    for run in runs(aircraft):
        pairs = [(c['speed_kmh'], boundary_ratio(c)) for c in run]
        pairs = [(v, r) for v, r in pairs if r == r]
        if not pairs:continue
        for p in points:
            v = p.get('sample_speed_kmh', p['speed_kmh'])
            if not run[0]['speed_kmh']-1e-9 <= v <= run[-1]['speed_kmh']+1e-9:continue
            r = float(np.interp(v, [a for a, _ in pairs], [b for _, b in pairs]))
            s = WING.break_seconds(r)
            if s is not None:p['wing_load_ratio'] = r;p['wing_break_s'] = float(s)


def wing_turns(aircraft):
    """The drawn wing line as a function: speeds (array) -> its turn rate (linear between the drawn points of each
    stretch, NaN where no stretch covers the speed); None without a line."""
    stretches, piece = [], []
    for p in (aircraft.get('wing_line') or [])+[dict(turn_dps=None)]:
        if p.get('turn_dps') is not None:
            piece.append((p['speed_kmh'], p['turn_dps']));continue
        if len(piece) >= 2:stretches.append((np.array([q[0] for q in piece], dtype=float), np.array([q[1] for q in piece], dtype=float)))
        piece = []
    if not stretches:return None

    def at(x):
        x = np.asarray(x, dtype=float);out = np.full(x.shape, np.nan)
        for xs, ts in stretches:
            sel = (x >= xs[0]) & (x <= xs[-1])
            if sel.any():out[sel] = np.interp(x[sel], xs, ts)
        return out
    return at


def split_past_wing(path, at):
    """A contour path in pieces by the side of the wing line (``at``: wing_turns): the pieces above it carry
    past_wing=True. Neighbouring pieces share the crossing point, which lies on the drawn line (bisection along the
    path's segment); where the line ends between two points (no line on one side) the cut is that point itself. The
    over-rev record ('percent_over', 'overrev_s') is sliced with the path, the crossing taking its neighbour's value
    in each piece. Pieces of fewer than 2 points are dropped; a path wholly on or below the line is returned as it is."""
    x = np.asarray(path['x'], dtype=float);y = np.asarray(path['y'], dtype=float)
    d = y-at(x)
    past = np.isfinite(d) & (d > 1e-9)
    if not past.any():return [path]
    extras = [k for k in ('percent_over', 'overrev_s') if k in path]

    def start(flag):
        piece = dict(level=path['level'], x=[], y=[], **{k: [] for k in extras})
        if flag:piece['past_wing'] = True
        return piece

    def add(piece, px, py, i):
        piece['x'].append(float(px));piece['y'].append(float(py))
        for k in extras:piece[k].append(path[k][i])
    out = [];piece = start(bool(past[0]))
    for i in range(len(x)):
        if i and past[i] != past[i-1]:
            if np.isfinite(d[i-1]) and np.isfinite(d[i]):
                lo, hi, t = 0., 1., min(1., max(0., d[i-1]/(d[i-1]-d[i])))
                for _ in range(50):
                    mid = .5*(lo+hi)
                    f = y[i-1]+mid*(y[i]-y[i-1])-float(at(np.array([x[i-1]+mid*(x[i]-x[i-1])]))[0])
                    if f != f:break     # the line ends inside this segment: the linear estimate
                    if (f > 1e-9) == bool(past[i-1]):lo = mid
                    else:hi = mid
                else:
                    t = .5*(lo+hi)
                cx, cy = x[i-1]+t*(x[i]-x[i-1]), y[i-1]+t*(y[i]-y[i-1])
                add(piece, cx, cy, i-1)
                out.append(piece);piece = start(bool(past[i]))
                add(piece, cx, cy, i)
            elif np.isfinite(d[i]):
                # no line at the point before: the piece above starts on that point
                out.append(piece);piece = start(bool(past[i]))
                add(piece, x[i-1], y[i-1], i-1)
            else:
                # no line at this point: the piece above ends on it
                add(piece, x[i], y[i], i)
                out.append(piece);piece = start(bool(past[i]))
        add(piece, x[i], y[i], i)
    out.append(piece)
    return [p for p in out if len(p['x']) > 1]


def secondary_outline(data, aircraft):
    """The dashed line of the Instructor's other wing protection state (aircraft['secondary_boundary']) from the second
    envelope pass (aircraft['secondary_envelope'], secondary_pass): the outline interpolant (run_loads) of its own
    runs, kept only where it differs from the chart's boundary by more than the envelope's turn tolerance (with one
    coincident point either side). A 'held' chart (the line is the limit after the release) also needs the game's
    release timer to run at the boundary there (the columns' protection['releasable'], linear in speed). Points
    dict(speed_kmh, load_g, turn_dps, and wing_break_s where the line's own states are past the wing's breaking load);
    a dict(speed_kmh, turn_dps=None) between stretches. Removed where nothing remains or no second pass ran."""
    aircraft.pop('secondary_boundary', None)
    second = aircraft.get('secondary_envelope')
    if not second:return
    protection = aircraft.get('wing_protection') or {}
    primary = protection.get('state') or ('released' if second.get('state') == 'held' else 'held')
    tolerance = (aircraft.get('envelope_trace') or {}).get('turn_tolerance_dps', ENVELOPE_TOLERANCE_DPS)
    grid = chart_grid(data)
    own = runs(aircraft)
    gate = None
    if primary == 'held':
        marks = sorted((c['speed_kmh'], float(bool(c['protection'].get('releasable'))))
                       for c in aircraft.get('columns', []) if ok(c) and c.get('protection'))
        if marks:gate = [m[0] for m in marks], [m[1] for m in marks]
    points = []
    for run in drawn_runs(second):
        if len(run) < 2:continue
        speeds = np.array([c['speed_kmh'] for c in run], dtype=float)
        xs = np.unique(np.r_[grid[(grid > speeds[0]) & (grid < speeds[-1])], speeds])
        loads = run_loads(run, xs)
        base = np.full(len(xs), np.nan)
        for other in own:
            sel = (xs >= other[0]['speed_kmh']) & (xs <= other[-1]['speed_kmh'])
            if sel.any():base[sel] = run_loads(other, xs[sel])
        rates = turn_rate(xs, loads)
        with np.errstate(invalid='ignore'):
            differ = np.isfinite(base) & np.isfinite(rates) & (np.abs(rates-turn_rate(xs, base)) > tolerance)
        if gate is not None:differ &= np.interp(xs, gate[0], gate[1]) >= .5
        keep = differ.copy();keep[1:] |= differ[:-1];keep[:-1] |= differ[1:]
        keep &= np.isfinite(loads) & np.isfinite(rates)
        pairs = [(c['speed_kmh'], boundary_ratio(c)) for c in run]
        pairs = [(v, r) for v, r in pairs if r == r]
        ratio = np.interp(xs, [a for a, _ in pairs], [b for _, b in pairs]) if pairs else np.full(len(xs), np.nan)
        stretch = []
        for x, n, t, r, on in zip(xs.tolist()+[None], loads.tolist()+[None], rates.tolist()+[None], ratio.tolist()+[None],
                                  keep.tolist()+[False]):
            if on:
                s = WING.break_seconds(r) if r == r else None
                stretch.append(dict(speed_kmh=float(x), load_g=float(n), turn_dps=float(t), **({} if s is None else {'wing_break_s': float(s)})))
                continue
            if len(stretch) >= 2:
                if points:points.append(dict(speed_kmh=float(.5*(points[-1]['speed_kmh']+stretch[0]['speed_kmh'])), turn_dps=None))
                points += stretch
            stretch = []
    if not points:return
    seconds = protection.get('release_start_s')
    aircraft['secondary_boundary'] = dict(state=second.get('state'), seconds=seconds, label=WING.secondary_label(primary, seconds),
                                          points=points)


def enrich(data, aircraft):
    aircraft['boundary'] = outline(data, aircraft)
    if OVERREV and aircraft.get('overrev_lines') is not None:outline_overrev(aircraft)
    # Wing line (Leo 2026-11-10), before the contours (they are split on the drawn line)
    line = wing_outline(data, aircraft) if any(c.get('wing_line') for c in aircraft.get('columns', [])) else []
    if line:aircraft['wing_line'] = line
    else:aircraft.pop('wing_line', None)
    if (aircraft.get('envelope_trace') or {}).get('wing_line'):outline_wing(aircraft)
    secondary_outline(data, aircraft)
    aircraft['contours'] = contours(aircraft, data['settings'].get('sep_contour_levels_mps'))
    zeros = [p for p in aircraft['contours'] if p['level'] == 0.]
    aircraft['sustained_curve'] = dict(x=[x for p in zeros for x in [*p['x'], None]], y=[y for p in zeros for y in [*p['y'], None]])
    aircraft['numerical_boundaries'] = []
    aircraft['numerical_gaps'] = []
    aircraft['display_discontinuities'] = []
    return aircraft
