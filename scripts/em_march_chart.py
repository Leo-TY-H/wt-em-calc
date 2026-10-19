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
  missing column; a smooth line is never drawn across an unresolved speed;
- SEP contours are the solved curve components, drawn only between their
  knots and never across an unresolved knot interval; only the calculated
  levels are available (other levels need a new calculation).

Fallback (``eligible``): configurations outside the validated domain use the
baseline sampler for that aircraft, and so does any aircraft on which the
march raises or finds no verified boundary at all. The reason is recorded in
the chart (``chart_solver_fallback``). ``chart_solver='baseline'`` selects
the baseline for every aircraft.
"""
from __future__ import annotations

import math
import time

import numpy as np

COLORS = ('#38c9d7', '#ffa66b')
ENVELOPE_TOLERANCE_DPS = .15   # at the standard chart precision (turn tolerance 0.02 deg/s)
CURVE_SCALE = 3.
COLUMN_SPACING_KMH = 80.
MAX_STEP_KMH = 50.
REVISION = 'alpha-march-chart-v1'


def eligible(condition):
    """(True, None) when the march chart covers this aircraft condition, else (False, reason)."""
    c = condition
    checks = [
        (c['trim_solver_mode'] == 'stationary', 'nested or uncorrected trim'),
        (c['sampling'] == 'adaptive', 'regular diagnostic sampling'),
        (c['stationary_execution'] == 'baseline', 'diagnostic stationary execution mode'),
        (not c['experimental_instantaneous_aero'] and c['experimental_lateral_trim'] == 'full', 'model experiment'),
        (c['aircraft_trim_mode'] == 'discrete' and c['turn_response_mode'] == 'settled', 'trim or turn-response mode'),
        (c['mach_curve_mode'] == 'stable', 'Mach curve mode other than stable'),
        (c['stationary_governor_mode'] == 'pitch_stops', 'governor mode other than pitch stops'),
        (not c['manual_engine_controls'], 'manual engine controls (over-rev boundaries)'),
        (c['max_load_g'] is None and not c['reference_load_cap'] and not c['low_speed_load_cap']
         and c['aircraft_search_region'] and c['global_load_cap_g'] == 64., 'non-default chart load caps'),
    ]
    for ok, reason in checks:
        if not ok:return False, reason
    from em_sampling import sweep_speed_intervals
    name = c['aircraft'][0]
    if sweep_speed_intervals(name, c):return False, 'speeds where the selected wing sweep is unavailable'
    return True, None


def envelope_tolerance(config):
    from em_accuracy import turn_tolerance
    # The ratio first, so the standard precision gives exactly 0.15 deg/s.
    return ENVELOPE_TOLERANCE_DPS*(turn_tolerance(config['sep_tolerance_mps'])/turn_tolerance(.5))


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
    try:
        return march_solvers(name, index, cfg, tracer, solver, curve_solver, report, cancelled, show, started)
    finally:
        for s in (solver, curve_solver):N.detach(s)


def march_solvers(name, index, cfg, tracer, solver, curve_solver, report, cancelled, show, started):
    import em_march as M
    import em_march_curves as C
    import em_march_native as N
    from em_trim_work import Work, use
    native = N.attach(solver)
    loops = [solver.native_loop] if getattr(solver, 'native_loop', None) is not None else []
    total = Work({});t0 = time.process_time()
    # S5: the march, its column workers and the tracer on native trial states where the compiled
    # module covers this solver; otherwise the Python march (em_march, the reference) as in S4.
    engine = N.engine(solver)
    if engine is not None:
        native_tracer = solver.native_loop.m.NTracer(engine, name, cfg, tracer.cj, tracer.ttol, tracer.vtol, tracer.place_tol,
                                                     tracer.max_step, tracer.max_depth, tracer.march_tol, tracer.on_column)
        with use(total):
            envelope = native_tracer.run()
            envelope['columns'] = [native_tracer.record(c) for c in envelope['columns']]
        N.merge_stats(engine.stats)
    else:
        with use(total):envelope = tracer.run()
    envelope_cpu = time.process_time()-t0
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
    t0 = time.process_time()
    result = curves.run() if verified else None
    curve_cpu = time.process_time()-t0
    if result is not None and str(result.get('status', '')).startswith('exhausted'):
        raise RuntimeError('SEP curve budget exhausted: '+result['status'])
    row = aircraft_row(name, index, cfg, solver, envelope, result, tracer)
    ee, ce = envelope_work.get('aircraft_evaluations', 0), (result or {}).get('work', {}).get('aircraft_evaluations', 0)
    row['chart_solver_work'] = dict(evaluations=ee+ce, envelope_evaluations=ee, curve_evaluations=ce,
                                    envelope_cpu_s=envelope_cpu, curve_cpu_s=curve_cpu, elapsed_s=time.monotonic()-started,
                                    columns=len(envelope['columns']), work_by_role=envelope['work_by_role'],
                                    march=M.stats(), native=N.summary(loops), native_status=native,
                                    curve_status=(result or {}).get('status'))
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
                                   curve_unresolved_intervals=[dict(level=c['level'], interval_kmh=u['interval_kmh'])
                                                               for c in components for u in c['unresolved_intervals']],
                                   certification_scope='admitted full-model states; withheld exact checks; no global coverage certificate'))


def compact_component(comp):
    return dict(level=comp['level'], segment_kmh=comp['segment_kmh'],
                knots=[[k['speed_kmh'], k['load_g'], k.get('ps_mps')] for k in comp['knots']],
                unresolved_intervals=[dict(interval_kmh=list(u['interval_kmh']), reason=str(u.get('reason'))[:120])
                                      for u in comp.get('unresolved_intervals', [])],
                start=(comp.get('start') or {}).get('kind'), end=(comp.get('end') or {}).get('kind'))


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
    reason, load = first.get('boundary_reason'), first['boundary']['load_g']
    if not clipped and below and below[-1].get('boundary_status') == 'no feasible samples':
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
    seams = trace.get('mach_seams_kmh', [])
    events = trace.get('events', [])
    gaps = [tuple(x) for x in aircraft.get('interpolation', {}).get('unresolved_speed_intervals', [])]
    out, run = [], []
    for c in columns:
        if not ok(c):
            if run:out.append(run)
            run = [];continue
        if run:
            a, b = run[-1]['speed_kmh'], c['speed_kmh']
            cut = (any(a <= x+1e-9 and b >= y-1e-9 or a < x < b or a < y < b for x, y in seams)
                   or any(e['kind'] == 'edge' and a <= min(e['speed_kmh']) and b >= max(e['speed_kmh']) for e in events)
                   or any(e['kind'] == 'unresolved' and a < e['speed_kmh'] < b for e in events)
                   or any(lo < b and hi > a for lo, hi in gaps))
            if cut:
                out.append(run);run = []
        run.append(c)
    if run:out.append(run)
    return out


def outline(data, aircraft):
    """Boundary outline in the production chart format (dense PCHIP of limiting load per run)."""
    from scipy.interpolate import PchipInterpolator
    cfg = data['settings']
    pieces = runs(aircraft)
    nx = 2*cfg.get('surface_resolution', 601)-1
    if data.get('preview'):nx = min(nx, 241)
    grid = np.linspace(cfg['speed_min_kmh'], cfg['speed_max_kmh'], nx)
    boundary = []
    for k, run in enumerate(pieces):
        speeds = np.array([c['speed_kmh'] for c in run])
        exact = {c['speed_kmh']: c for c in run}
        xs = np.unique(np.r_[grid[(grid > speeds[0]) & (grid < speeds[-1])], speeds])
        if len(run) > 1:
            f = PchipInterpolator(speeds, [c['boundary']['load_g'] for c in run])
            loads = f(xs)
        else:loads = np.array([run[0]['boundary']['load_g']])
        rates = turn_rate(xs, loads)
        for i, (x, t) in enumerate(zip(xs, rates)):
            c = exact.get(float(x))
            j = int(np.searchsorted(speeds, x))
            if c is not None:
                p = dict(speed_kmh=float(x), turn_dps=float(c['boundary']['turn_dps']),
                         at_plot_ceiling=c['boundary_status'] == 'plot ceiling')
                if c.get('load_search_limit'):p['load_search_limit'] = c['load_search_limit']
            else:
                left, right = run[max(0, j-1)], run[min(len(run)-1, j)]
                p = dict(speed_kmh=float(x), turn_dps=float(t),
                         at_plot_ceiling=left['boundary_status'] == 'plot ceiling' and right['boundary_status'] == 'plot ceiling')
            boundary.append(p)
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
    if redline and redline['enforced'] and pieces:
        sample = redline['sample_speed_kmh'];column = next((c for c in pieces[-1] if c['speed_kmh'] == sample), None)
        if column is not None and cfg['speed_min_kmh'] <= redline['speed_kmh'] <= cfg['speed_max_kmh']:
            at = next((i for i, p in enumerate(boundary) if p['speed_kmh'] == sample and p['turn_dps'] is not None), None)
            if at is not None:
                common = dict(speed_kmh=redline['speed_kmh'], sample_speed_kmh=sample, at_plot_ceiling=False,
                              vertical_edge=True, edge_kind=redline['kind']+' speed boundary')
                boundary[at+1:at+1] = [dict(common, turn_dps=column['boundary']['turn_dps']), dict(common, turn_dps=0.)]
    return boundary


def polyline(component, samples=200):
    """Drawable pieces of one compact curve component: PCHIP in n^2-1 through its knots,
    split at unresolved knot intervals (as em_march_curves.polyline)."""
    from scipy.interpolate import PchipInterpolator
    knots = component['knots']
    if not knots:return []
    if len(knots) == 1:return [[(knots[0][0], float(turn_rate(knots[0][0], knots[0][1])))]]
    v = np.array([k[0] for k in knots]);w = np.array([k[1]**2-1. for k in knots])
    f = PchipInterpolator(v, w);xs = np.unique(np.r_[np.linspace(v[0], v[-1], samples), v])
    open_ = [tuple(u['interval_kmh']) for u in component.get('unresolved_intervals', [])]
    pieces, piece = [], []
    for x in xs:
        if any(lo < x < hi for lo, hi in open_):
            if piece:pieces.append(piece)
            piece = [];continue
        piece.append((float(x), float(turn_rate(x, math.sqrt(1.+max(float(f(x)), 0.))))))
    if piece:pieces.append(piece)
    return pieces


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
