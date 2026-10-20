"""The default chart solver (S4): alpha-march envelope plus direct SEP curves.

For each aircraft (``compute``):
1. envelope: ``em_march.MarchTracer`` places speed columns (sweep, corners,
   edges, withheld midpoint checks); every column is an alpha march from level
   flight to the first limit reached (Leo, 2026-10-17);
2. SEP curves: ``em_march_curves.MarchCurveSolver`` in column mode on the
   verified columns, with withheld exact checks of every knot interval;
3. the compiled loop (``em_march_native``) where it covers the call.

Policy and tolerances (speed plan, Leo 2026-10-17/18): zero prescribed
sideslip (``sideslip_recovery`` False); envelope tolerance 0.15 deg/s at the
standard 0.5 m/s chart precision (scaled with the chart's turn tolerance for
other precisions); curve tolerance 3x the chart curve tolerance; physical
admission unchanged; every drawn state is an admitted full-model state.

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
    if preview:data['preview'] = True
    return data


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
    try:
        return march_solvers(name, index, cfg, tracer, solver, curve_solver, report, cancelled, show, started)
    finally:
        for s in (solver, curve_solver):
            N.detach(s);s.__dict__.pop('soft_admission', None)
        if collecting:gc.enable()


def march_solvers(name, index, cfg, tracer, solver, curve_solver, report, cancelled, show, started):
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
    verified = [c for c in envelope['columns'] if tracer.ok(c)]
    row = aircraft_row(name, index, cfg, solver, envelope, None, tracer)
    show(row)
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
    if verified and curve_engine is not None:
        native_curves = curve_solver.native_loop.m.NCurves(curve_engine, curves)
        native_curves.check = check
        result = native_curves.run(verified)
    else:
        result = curves.run() if verified else None
    curve_cpu = time.process_time()-t0;curve_wall = time.perf_counter()-w0
    if result is not None and str(result.get('status', '')).startswith('exhausted'):
        raise RuntimeError('SEP curve budget exhausted: '+result['status'])
    row = aircraft_row(name, index, cfg, solver, envelope, result, tracer)
    ee, ce = envelope_work.get('aircraft_evaluations', 0), (result or {}).get('work', {}).get('aircraft_evaluations', 0)
    row['chart_solver_work'] = dict(evaluations=ee+ce, envelope_evaluations=ee, curve_evaluations=ce,
                                    envelope_cpu_s=envelope_cpu, curve_cpu_s=curve_cpu, elapsed_s=time.monotonic()-started,
                                    envelope_wall_s=envelope_wall, curve_wall_s=curve_wall,
                                    columns=len(envelope['columns']), work_by_role=envelope['work_by_role'],
                                    march=M.stats(), native=N.summary(loops), native_status=native,
                                    curve_status=(result or {}).get('status'))
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
                # The complete curve solver record (knots, withheld checks, ends) for exports and replay.
                curves=None if curves is None else {k: v for k, v in curves.items() if k not in ('seeds',)},
                envelope_trace=dict(segments=envelope['segments'], mach_seams_kmh=envelope['mach_seams_kmh'],
                                    sweep_gaps_kmh=envelope.get('sweep_gaps_kmh', []),
                                    soft_limits=bool(getattr(solver, 'soft_admission', False)),
                                    events=envelope['events'], checks=len(envelope['checks']),
                                    speed_limit=envelope['speed_limit'], turn_tolerance_dps=tracer.ttol,
                                    placement_tolerance_kmh=tracer.place_tol),
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
                   if comp.get('fold') else {}))


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
    return dict(speed_kmh=first['speed_kmh'], kind=kind, bracket_kmh=[below[-1]['speed_kmh'], first['speed_kmh']] if below else None,
                refined=edge is not None, boundary_verified=True, method='alpha-march speed bracket',
                turn_dps=first['boundary']['turn_dps'], lower_load_g=1.,
                note='Vertical outline only; Ps exists only where a balanced equilibrium was solved')


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
    for k, run in enumerate(pieces):
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
        boundary.extend(ias_edge(aircraft, run, grid))
        if k < len(pieces)-1:
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
                boundary[at+1:at+1] = [dict(common, turn_dps=column['boundary']['turn_dps']), dict(common, turn_dps=0.)]
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
    v = [c['speed_kmh'] for c in lows];n = [c['lower_edge']['load_g'] for c in lows]
    slopes = [c.get('lower_edge_slope') for c in lows]
    events = (aircraft.get('envelope_trace') or {}).get('events', [])
    one = next((e for e in events if e['kind'] == 'ias edge' and min(e['speed_kmh']) <= v[0]+1e-9), None)
    if one is not None and max(one['speed_kmh']) < v[0]-1e-9:
        v.insert(0, float(max(one['speed_kmh'])));n.insert(0, 1.);slopes.insert(0, None)
    xs = np.unique(np.r_[grid[(grid > v[0]) & (grid < v[-1])], v])
    w = np.asarray(n, dtype=float)**2-1.
    if len(v) > 1 and any(s is not None for s in slopes) and np.all(np.diff(w) > 0.):
        # the edge's exact slopes: the speed along the limit as the cubic Hermite V(w), w = n^2-1, with dV/dw = 1/slope
        # at the columns and 0 at level flight (where the edge leaves the 1 g line vertically); the native tracer's
        # lower_split judges this drawing. Sampled in w (with the columns) instead of on the speed grid.
        d = [0. if k == 0 and n[0] == 1. else (1./s if s is not None and s > 0. else None) for k, s in enumerate(slopes)]
        dl, dr = hermite_slopes(w, np.asarray(v, dtype=float), d, d)
        wq = np.unique(np.r_[np.linspace(w[0], w[-1], 121), w])
        xs = hermite(w, np.asarray(v, dtype=float), dl, dr, wq);loads = np.sqrt(1.+np.maximum(wq, 0.))
        order = np.argsort(xs, kind='stable');xs, loads = xs[order], loads[order]
    else:
        loads = PchipInterpolator(v, n)(xs) if len(v) > 1 else np.array(n)
    rates = turn_rate(xs, loads)
    common = dict(at_plot_ceiling=False, edge_kind=kind+' (forward IAS) speed boundary', leaning_edge=True)
    out = [dict(common, speed_kmh=float(x), turn_dps=float(t)) for x, t in zip(xs[::-1].tolist(), rates[::-1].tolist())]
    if n[0] > 1.:out.append(dict(common, speed_kmh=float(v[0]), turn_dps=0.))
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
            [fold_path(component['fold'])] if component.get('fold') else [])
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
    if component.get('fold'):pieces.append(fold_path(component['fold']))
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
    for comp in aircraft.get('curve_components', []):
        if comp['level'] not in wanted:continue
        for piece in polyline(comp):
            if len(piece) > 1:out.append(dict(level=float(comp['level']), x=[p[0] for p in piece], y=[p[1] for p in piece]))
    return out


def enrich(data, aircraft):
    aircraft['boundary'] = outline(data, aircraft)
    aircraft['contours'] = contours(aircraft, data['settings'].get('sep_contour_levels_mps'))
    zeros = [p for p in aircraft['contours'] if p['level'] == 0.]
    aircraft['sustained_curve'] = dict(x=[x for p in zeros for x in [*p['x'], None]], y=[y for p in zeros for y in [*p['y'], None]])
    aircraft['numerical_boundaries'] = []
    aircraft['numerical_gaps'] = []
    aircraft['display_discontinuities'] = []
    return aircraft
