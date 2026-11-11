"""The wing breaking load on the chart: a line, not a limit; the Instructor's wing protection (Leo 2026-11-10).

One rule for every control mode: the chart boundary is what a sustained turn with full pitch held reaches first
(Instructor AoA limit, control authority, stall, pitch response, the IAS and Mach limits). The wing force (highest
wing_load_ratios above 1) is no admission failure and no march event; the load where that ratio is 1 is charted as the
line 'Wing breaks (wing load)' wherever it lies inside the boundary.

Game rule for an intact wing (aces.exe, 48 Hz main step), e = ratio - 1 > 0: a damage counter fills in 0.3/e s, then
the wing breaks on each step with probability e/8 (mean wait 8/(48 e) = 1/(6 e) s). Expected time to break
``break_seconds`` = 0.3/e + 1/(6 e) s (ratio 1.02: 23.3 s, 1.05: 9.3 s, 1.10: 4.7 s).

Instructor wing protection (instructor_protection.angle_targets): while its overload timer is at or below
overloadTimeRange[0] the high AoA limit is capped at reserve x the wing-strength AoA (the critical angle where the wing
cannot be overloaded). The timer rises by overloadTimeRate per second only while the strength AoA is inside the base
limit and the predicted AoA is above onset x the strength AoA, so the release starts overloadTimeRange[0] /
overloadTimeRate seconds into such a pull (defaults 0.9 / 0.45 = 2 s; never where the range starts at 1 or above), and
never where the strength AoA is not inside the base limit. Two chart states (``protection``, solver attribute
instructor_protection, read by instructor_aoa.controller_limits and its native twin):

- 'held'     the cap everywhere it applies (overload timer 0);
- 'released' the post-release limit everywhere, without checking the in-game release conditions (chart policy).

The boundary uses 'held' where the release starts after HOLD_S = 10 s (or never), else 'released'; the chart
draws the other state as a dashed line where the two differ. Without the Instructor, with Flutter (structural_limits)
off or with limitOverload false there is no protection state.

The wing rule follows the march chart's admission rule (solver.soft_admission, as overrev.rule): the baseline sampler
and the Python march keep the wing cut. WT_WING_LINE=0 restores the deployed chart exactly (wing force a limit, no
protection state: the Instructor limit with the overload reserve omitted).
"""
import math
import os

RULE = os.environ.get('WT_WING_LINE', '1') != '0'
# Leo 2026-11-10 (later the same day): the wing's breaking load stays the drawn limit in every mode, also where the
# achievable pull is above it. WT_WING_AS_LINE=1 brings back the wing as a line with the boundary past it.
LINE = os.environ.get('WT_WING_AS_LINE', '0') == '1'
HOLD_S = 10.               # latest release start (s) for the released chart boundary
G = 9.8100004196167


def rule(solver):
    """The wing is a line, not a cut: Flutter on and the Instructor on, under the march chart's admission rule
    (solver.soft_admission). Without the Instructor nothing limits a full pull near the wing, so the wing stays the
    boundary there (Leo 2026-11-10: the F-104A full-real envelope ran to 22 g)."""
    return (RULE and LINE and bool(solver.config['structural_limits']) and bool(solver.config.get('instructor'))
            and bool(getattr(solver, 'soft_admission', False)))


def break_seconds(ratio):
    """Expected seconds until the game breaks an intact wing held at wing load ``ratio`` (> 1); None at or below 1."""
    if ratio is None:return None
    e = float(ratio)-1.
    if not e > 1e-9:return None
    return .3/e + 1./(6.*e)


def protection(fm, config):
    """(state, release start in s) of the Instructor's wing protection for one aircraft and its conditions: state
    'held' | 'released' (config['instructor_protection'] when set, else by the release start against HOLD_S), release
    start math.inf where the protection is never released; (None, None) without protection (Instructor off, Flutter
    off, limitOverload false, WT_WING_LINE=0)."""
    if not (RULE and config['instructor'] and config['structural_limits']):return None, None
    from instructor_chart_inputs import GAMEPLAY
    props = dict(GAMEPLAY['propsDefault']);props.update(fm.get('Instructor', {}))
    if not props.get('limitOverload', True):return None, None
    rate = float(props.get('overloadTimeRate', .45));start = float(list(props.get('overloadTimeRange', [.9, 1.]))[0])
    release_s = math.inf if rate <= 0. or start >= 1. else max(0., start/rate)
    state = config.get('instructor_protection') or ('held' if release_s > HOLD_S else 'released')
    return state, release_s


def secondary_label(primary_state, seconds):
    """Legend and hover name of the dashed line of the other protection state."""
    timed = seconds is not None and math.isfinite(seconds)
    n = '%g' % round(seconds, 1) if timed else ''
    if primary_state == 'held':return 'Instructor limit after '+n+' s' if timed else 'Instructor limit without wing protection'
    return 'Instructor wing protection, first '+n+' s' if timed else 'Instructor wing protection'


def column_point(column):
    """The wing-line point of one march column by linear interpolation in load between its two admitted samples
    around wing load ratio 1 (the fallback and cross-check of the native wing event); None without a crossing."""
    v = column['speed_kmh']
    rows = sorted((p for p in column.get('points') or () if p.get('valid') and p.get('wing_load_ratios')
                   and abs(p['speed_kmh']-v) <= 1e-9), key=lambda p:p['load_g'])
    for a, b in zip(rows, rows[1:]):
        ra, rb = max(a['wing_load_ratios']), max(b['wing_load_ratios'])
        if not (ra <= 1. < rb and b['load_g'] > a['load_g']):continue
        t = (1.-ra)/(rb-ra);n = a['load_g']+t*(b['load_g']-a['load_g'])
        numbers = all(isinstance(p.get('ps_mps'), (int, float)) and math.isfinite(p['ps_mps']) for p in (a, b))
        return dict(speed_kmh=float(v), load_g=n, turn_dps=math.degrees(G*math.sqrt(max(0., n*n-1.))/(v/3.6)),
                    ps_mps=a['ps_mps']+t*(b['ps_mps']-a['ps_mps']) if numbers else None,
                    alpha_deg=a['alpha_deg']+t*(b['alpha_deg']-a['alpha_deg']), wing_load_ratio=1., method='interpolated')
    return None


def protection_limits(solver, row):
    """Both protection states at one chart row (one evaluation of its state, no admission): the high AoA limit with
    the cap (held_limit_deg), without it (free_limit_deg), of the 'released' chart state (released_limit_deg: always the free
    limit, irrespective of the game's release trigger) and whether the game's release timer runs at
    this state (releasable: instructor_protection.angle_targets' trigger)."""
    import numpy as np
    from em_stationary import JointProblem
    from instructor_chart_inputs import source_state
    from windows_instructor_source import fixed_source
    from instructor_protection import angle_targets, predicted_wing_angles
    core = solver.core() if callable(getattr(solver, 'core', None)) else solver
    core = core.at_sideslip(row.get('sideslip_attitude_deg', 0.))
    problem = JointProblem(core, row['speed_kmh'], row['load_g'], row['solution'], row.get('stationary_request_indices', ()))
    value = problem.value_at(np.asarray(row['stationary_coordinates']))[1]
    core.complete_reporting(value)
    state = source_state(core, value, constant_cache=getattr(core, '_effective_aoa_constants', None))
    fixed = fixed_source(core.model, state)
    adjusted = predicted_wing_angles(state['wing_angles'], [0., 0.], state['delivered'][0], state['delivered'][1],
        fixed['sensitivity'], core.model['controls']['Elevator']['wing_aoa'])['adjusted']
    common = dict(rho=fixed['rho'], speed_squared=state['wrapper']['speed_squared'], area=state['wing_area'],
        dihedral=state['dihedral'], strength=state['strength'], mass=state['wrapper']['mass'],
        tail_area_pair=state['tail_area_pair'], timer=0., dt=0., mode_lane=True)
    held = angle_targets(fixed['polar'], state['properties'], adjusted, overload_enabled=True, **common)
    free = angle_targets(fixed['polar'], state['properties'], adjusted, overload_enabled=False, **common)
    return dict(held_limit_deg=float(held['angle_limits'][1]), free_limit_deg=float(free['angle_limits'][1]),
                released_limit_deg=float(free['angle_limits'][1]),
                releasable=bool(held['overload_trigger']))
