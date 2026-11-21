"""The website's altitude envelope (Leo, 2026-10-30): SEP of 1 g level flight over true airspeed and altitude, for the
EM page's aircraft entries and with its settings. This replaces the earlier altitude plotter.

Every sample is the EM plotter's own state at 1 g, on a solver set up as a march chart sets up its own (soft admission,
no sideslip recovery). Since 2026-11-20 one compiled march engine serves all the rows of an entry (moved to each
row's altitude), and a row's states are continued from one another by that engine's own level-flight step, the
correction an EM chart takes from the level flight of the speed beside (`Envelope.row_steps`): 0.4 to 0.7 ms a state.
The point solver starts the first row, and is asked where the step finds nothing and no limit explains it. Without
the compiled engine the rows are the point solver's (`row_points`, about 1 ms a state). This module only decides
where to solve; it changes nothing in the solver.

Rows of constant altitude, each marched outward in speed from the best-SEP speed of the row beside it until level
flight ends. A row's end is the last state before a limit: the engine keeps each state's margin to every limit, the
search aims where the falling margin reaches 0, and that limit names the end. Within a row, knots are added where the
curve through the others misses one (towards the stall above all), jumps in SEP or thrust are closed in on, and where
two states of the propulsion exist at one speed the row takes the one with more thrust. Rows are halved where the row
between two others differs from what they predict, in SEP or in its ends, so supercharger gears and the corners of
the limits get rows of their own. Between rows the chart interpolates along lines of the same fraction of each row's
speed span, so contours run all the way to the edges of the envelope. A clicked point is solved afresh.

The entry's control mode decides the slow edge (Leo, 2026-11-20): Mouse aim ends at the Instructor's AoA cap,
Realistic and Full real at the stall.

Above 9,900 m the game takes a little vertical speed off every upward-moving step (kinematics.
altitude_velocity_correction). A level state that ends its step a hair upward gets that correction and fails the
vertical-step check, though nothing about it is wrong: half of all states above about 10,100 m. As the earlier plotter
did, such a state is moved to the side that does not ascend (settle), which changes its SEP by under 0.01 m/s. Inside a
row, where no limit is near, the state counts as it stands, which saves the second solve.
"""
import csv
import io
import math
import time

import contourpy
import numpy as np
from scipy.interpolate import Akima1DInterpolator

from em_entries import COLORS, flight_mode
from em_solver import AIRCRAFT, MAX_ALTITUDE_M, settings
from overrev import rpm_ratio

ALTITUDES = (0., float(MAX_ALTITUDE_M))    # the solver's own altitude range (em_solver.settings)
DEFAULT_TOP = 10000.        # the page starts at 0-10,000 m (Leo, 2026-10-30)
# Sampling preset by SEP tolerance (m/s): spacing of the base rows and the closest rows (m), of the speeds in a row and
# between the two states bracketing an end of a row (km/h), and how far a row's end may miss its neighbours' (km/h).
SAMPLING = {1.: (2000., 500., 50., 2., 4.), .5: (1000., 125., 30., 1., 2.), .15: (500., 62.5, 15., .25, 1.)}
SEED = ('solution', 'stationary_coordinate_names', 'stationary_coordinates', 'stationary_request_indices')
NOT_CLOSED = 'vertical step did not close'
NUDGES_DEG = (2e-5, 1e-4, 5e-4, 2e-3)   # angle of attack taken off to land on the side that does not ascend
MISSES = 2                  # rejected speeds in a row that end a march outward (twice that with no row beside it)
HARD_LIMITS = frozenset(('IAS limit', 'Mach limit'))    # nothing faster can pass these
NUMERICAL = frozenset(('trim did not converge', 'propulsion did not settle', 'propulsion cycle unresolved', NOT_CLOSED,
                       'pitch response unresolved', 'Instructor boundary unresolved', 'no candidate'))
BRIDGED_SAMPLES = 3         # a gap of this many speed steps with only numerical rejections is interpolated across
# What ends level flight, most telling first (a stalled state usually fails to trim as well), in the EM chart's words.
LIMITS = (('post-stall', 'Stall'), ('Instructor pitch limit', 'AoA limiter (Instructor)'),
          ('control authority', 'Control surface authority'), ('wing force limit', 'Structure (wing load)'),
          ('IAS limit', 'VNE'), ('Mach limit', 'Mach limit'), ('sweep unavailable', 'Selected wing sweep unavailable'),
          ('flap IAS limit', 'Flap IAS limit'), ('flap automatic retraction', 'Flap automatic retraction'),
          ('engine RPM limit', 'Engine RPM limit'), ('reversed pitch response', 'Elevator effectiveness (pitch-response reversal)'),
          ('propeller admission', 'Propeller limit'), ('propeller braking', 'Propeller braking'),
          ('propeller shaft stopped', 'Propeller shaft stopped'))
UNRESOLVED = 'Level flight unresolved beyond here'
# One compiled engine for all the rows of an entry, moved to each row's altitude (MarchEngine.move, as the engine page's
# level-flight curves since 2026-11-19), instead of a solver and engine built per row (4 to 6 ms each).
SHARED_ENGINE = True
# A point's row as the solver made it: normalizing every field for export took about a third of a solved point's time,
# and a row here reads eight of them.
LEAN_ROWS = True
# Past a speed where the solve found no state, and between a solved speed and such a one, only the solve continued from
# the given state is tried. The point solver's further starts (its kept seeds, the trim guess) take three to four times
# as long to fail there, and those failures were half of a chart's time. Each end found this way is confirmed by one
# complete solve just beyond it.
QUICK_FAILS = True
# Rows marched with the EM chart's own level-flight step (the march engine's prescribed-load correction from the state
# beside, as Column.level_from): the same equations and admission checks as the point solver, at about half its time
# for a state and a tenth for a speed without one. The point solver starts a row with nothing to continue from,
# confirms each end, and lands the states of the vertical-step rule. Needs SHARED_ENGINE (states are continued from
# the row beside).
NATIVE_STEPS = True
STEP_NFEV, STEP_EVALS = 12, 40      # the march's own budget for a level-flight correction (MARCH_NFEV, MARCH_EVALS)
ROBUST_NFEV, ROBUST_EVALS = 30, 120 # its budget for a correction taken again (column.pxi), used where the first does not close
UPWARD_FROM_M = 9900.               # kinematics.altitude_velocity_correction applies above this altitude
# A row's curve is checked between its knots and given more of them where it is off (row_steps): a knot the curve
# through the others misses by more than REFINE tolerances gets a state halfway to each neighbour, and so does the
# stretch next to each end of the row and each side of a jump (SEP falls away steeply towards the stall: with the
# grid's knots alone the curve was up to five tolerances off there). Not closer than REFINE_GAP km/h.
REFINE, REFINE_GAP, REFINE_ROUNDS = 1.5, 2., 6
# Where SEP jumps between two speeds, two propulsion states meet, and over a band of speeds both can exist: a march keeps
# the one it came in on. The chart takes the one with more thrust (Leo, 2026-10-21 for the EM chart's braking states;
# the engine page's curves since 2026-11-19): each side's state is continued across the jump, in steps of at most
# CARRY km/h (halved down to the bracket where a step finds nothing or less thrust), for as long as it has more
# thrust than what the row has there, by more than PREFER of it.
MORE_THRUST = True
CARRY, PREFER, CARRY_ROUNDS = 5., 1e-4, 4
JUMP_THRUST = .5        # of the SEP tolerance: an unexplained step in thrust worth this much SEP is looked into
# Propeller aircraft: other states are also looked for where no jump shows them (row_steps' `discover`), at these
# fractions of a row's states.
DISCOVER = True
DISCOVER_AT = (.1, .3, .5, .7, .9)
# What ends a row, read from its last state: the march engine keeps every state's margin to each limit (1 far, 0 on
# it). The limit whose margin, continued along the row from the state before, reaches 0 within END_REACH brackets of
# the end is what stops level flight there; then no solve beyond the end is needed to name it. Engine RPM is no limit
# of level flight here (over-rev is a charted line).
MARGIN_LABELS = {'Instructor pitch':'AoA limiter (Instructor)', 'control':'Control surface authority', 'stall':'Stall',
                 'pitch response':'Elevator effectiveness (pitch-response reversal)', 'wing force':'Structure (wing load)',
                 'IAS limit':'VNE', 'propeller braking':'Propeller braking'}
END_REACH = 3.
# The end search aims at that limit instead of halving the bracket: 0.9 of the way to where the margin reaches 0, until
# that is within END_EXACT brackets. Ends found this way move smoothly from row to row (ends known to a bracket made
# the rows between two others look wrong, and so asked for more rows). On the fast side the first probe is the
# aircraft's speed limit itself (em_speed_limits), for the same reason.
END_EXACT = .25
# A speed limit (VNE, Mach) rejects a state at once: that end is closed in on to this part of the bracket, so it too moves
# smoothly from row to row.
HARD_FINE = 1./32.
# The check of a row against the two beside it counts a SEP difference by how far it would move a contour: where SEP
# changes fast along the row (towards the stall, and towards the speed limit, where it falls by hundreds of m/s), a
# difference smaller than SEP's change over the end tolerance (2 km/h at Smooth) is within what the ends themselves are
# held to. Without it those stretches asked for rows down to the closest spacing all the way up (a jet: 110-165 rows).
STEEP_KIND = True
# Between rows (Blend): a jump within JUMP_REACH speed steps of one in the row beside is the same jump line; within a
# jump lines the rows are cut at the same Mach numbers, MACH_STEP apart.
JUMP_REACH = 2.5
JUMP_END = 2.
# The row check (Envelope.mismatch): a miss counts as 1/ROW_SMOOTH of itself where SEP bends smoothly with altitude,
# which it does where the curve through the next rows out as well comes within SMOOTH of the straight line's miss.
ROW_SMOOTH, SMOOTH = 1., .25     # 1: off. At 3 it saved 6 of 99 rows on the F-4E and left the F-15A 1.4 tolerances off
MISMATCH_LOG = None     # a list here collects every row check (diagnostics): height, row gap, SEP term, where, end and jump terms
MACH_LINES = False      # tried 2026-11-21: bands that open at an end as a Mach line enters the span asked for more rows, not fewer
MACH_STEP = .05
# A wider speed grid for wide envelopes (the preset's step times a whole number, for about GRID_STATES steps a row) was
# tried 2026-11-21 at 25: a quarter fewer states for jets, but up to 1.4 tolerances off against the reference rows.
GRID_STATES = 0
SEED_EVERY = 3                      # of a row's states, how many are kept as starts for clicked points (`Envelope.seeds`)
SPEED_END, ALTITUDE_END = 'End of the speed range', 'End of the altitude range'


def request(values):
    """The job's settings: the EM page's entries and plot settings, plus this chart's altitude range."""
    values = dict(values)
    low, high = values.pop('altitude_min_m', ALTITUDES[0]), values.pop('altitude_max_m', DEFAULT_TOP)
    choose = bool(values.pop('auto_contours', False))
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in (low, high)):
        raise ValueError('Altitude range must be numbers')
    if not ALTITUDES[0] <= low < high <= ALTITUDES[1] or high-low < 100.:
        raise ValueError(f'Altitude range must rise at least 100 m, within 0 to {ALTITUDES[1]:,.0f} m')
    if not values.get('entries'):raise ValueError('Add at least one aircraft')
    return dict(settings(values), kind='altitude', altitude_min_m=float(low), altitude_max_m=float(high), auto_contours=choose)


def level_chart(name, condition, height):
    """A march chart's solver at this altitude, with the compiled evaluator."""
    import em_march_native
    from em_stationary_baseline import BaselineSolver
    chart = BaselineSolver(name, settings(dict(condition, altitude_m=float(height), sideslip_recovery=False)))
    chart.soft_admission = True
    em_march_native.attach(chart)
    return chart


def solver_at(name, condition, height):
    """A march chart's solver at this altitude and its compiled point solve (the Python one if there is no compiled)."""
    import em_march_native
    chart = level_chart(name, condition, height)
    engine = em_march_native.engine(chart)
    return chart, engine.get_point_solver().solve_row if engine is not None else chart.solve


def _no_guess(*args, **kwargs):
    raise ValueError('only the continued solve is tried here')


def continued_only(chart, solve, speed, seed):
    """`level_point` from this state alone: no kept seeds, no trim guess. None unless that start gives a state or is
    rejected for a physical reason (the point solver stops there in any case)."""
    from collections import OrderedDict
    kept = getattr(chart, '_stationary_seeds', None)
    chart._stationary_seeds = OrderedDict();chart.initial_guess = _no_guess
    try:return level_point(chart, solve, speed, seed)
    finally:
        del chart.initial_guess
        if kept is None:del chart._stationary_seeds
        else:chart._stationary_seeds = kept


def condition_of(config, entry):
    """The solver settings of one entry. Its control mode decides the slow edge (Leo, 2026-11-20: "users reserve the
    option to use instructor boundary or true stall"): Mouse aim ends level flight at the Instructor's AoA cap,
    Realistic and Full real at the wing's stall."""
    base = {k:v for k, v in config.items() if k not in ('kind', 'altitude_min_m', 'altitude_max_m', 'auto_contours')}
    return dict(base, **dict(entry['settings'], entries=None, aircraft=[entry['aircraft_id']], aircraft_settings={},
                             compare_instructor=False))


def seed_of(point):
    return {k:point[k] for k in SEED if k in point}


def level_point(chart, solve, speed, seed):
    """The EM point at 1 g, or None where no candidate is established."""
    from em_sampling import continuation_initial
    try:return solve(speed, 1., continuation_initial(chart, seed, list(seed['solution'])) if seed else None)
    except ValueError:return None


def ends_upward(point):
    return bool(point) and point.get('reasons') == [NOT_CLOSED] and bool(point.get('altitude_correction'))


def settle(chart, solve, speed, point):
    """A state rejected only for ending its step upward, moved to the side that does not ascend, where it gets every
    other check as well (the rejected state skips the pitch-response and Instructor checks)."""
    if not ends_upward(point):return point
    base = seed_of(point)
    for nudge in NUDGES_DEG:
        state = list(base['solution']);state[0] -= nudge
        retry = level_point(chart, solve, speed, dict(base, solution=state))
        if retry and (retry.get('valid') or retry.get('reasons') != [NOT_CLOSED]):return retry
    return point


def limit_label(misses):
    """The limit behind a row's end: of the rejected states next to it, the nearest with a physical reason."""
    for reasons in misses:
        for reason, label in LIMITS:
            if reason in reasons:return label
    return UNRESOLVED


def makima_at(xs, ys, v):
    """The modified Akima curve through a few knots at one place: scipy's Akima1DInterpolator(method='makima') in plain
    arithmetic (building that object for every knot checked took two fifths of a chart's time)."""
    n = len(xs)
    if n < 3:return float(np.interp(v, xs, ys))
    m = [(ys[i+1]-ys[i])/(xs[i+1]-xs[i]) for i in range(n-1)]
    first, last = 2.*m[0]-m[1], 2.*m[-1]-m[-2]
    m = [2.*first-m[0], first]+m+[last, 2.*last-m[-1]]       # slopes continued two intervals past each end
    j = min(max(next((i for i in range(n-1) if v <= xs[i+1]), n-2), 0), n-2)

    def slope(i):
        a, b, c, d = m[i:i+4]
        f1, f2 = abs(d-c)+.5*abs(d+c), abs(b-a)+.5*abs(b+a)
        return (f1*b+f2*c)/(f1+f2) if f1+f2 > 1e-12*max(abs(a), abs(b), abs(c), abs(d), 1e-300) else .5*(d+a)
    h = xs[j+1]-xs[j];t = (v-xs[j])/h;t0, t1 = slope(j), slope(j+1)
    return ((2.*t**3-3.*t**2+1.)*ys[j]+(t**3-2.*t**2+t)*h*t0+(-2.*t**3+3.*t**2)*ys[j+1]+(t**3-t**2)*h*t1)


class Row:
    """One altitude's solved speeds, its ends, and SEP along it: one curve for each piece between two jumps."""
    def __init__(self, height, samples, left, right, labels, gaps, spacing, jumps):
        self.height, self.left, self.right, self.labels, self.gaps = height, left, right, labels, gaps
        self.samples = samples      # by speed, the rejected ones too
        # The curve runs through the ends and the states well inside them: the bisection's close states add only rounding.
        knots = [v for v, s in sorted(samples.items()) if s['valid'] and (v in (left, right) or s.get('knot') or left+spacing < v < right-spacing)]
        self.jumps = sorted(j for j in jumps if knots and knots[0] < j < knots[-1])
        self.pieces = []            # (first knot, last knot, curve, slope)
        for k in range(len(self.jumps)+1):
            low = self.jumps[k-1] if k else -math.inf;high = self.jumps[k] if k < len(self.jumps) else math.inf
            xs = [v for v in knots if low < v < high];ys = [samples[v]['ps_mps'] for v in xs]
            # Not a shape-preserving curve: those flatten the top of SEP to the best state solved, and the ceiling sits there.
            if len(xs) > 2:
                curve = Akima1DInterpolator(xs, ys, method='makima');slope = curve.derivative()
            elif xs:
                curve = lambda v, x=xs, y=ys:np.interp(v, x, y)
                slope = lambda v, k=(ys[-1]-ys[0])/max(xs[-1]-xs[0], 1e-9):np.full(np.shape(v), k)
            else:continue
            self.pieces.append((xs[0], xs[-1], curve, slope))
        self.jumps = [.5*(a[1]+b[0]) for a, b in zip(self.pieces, self.pieces[1:])]
        self.steps = [abs(float(b[2](b[0]))-float(a[2](a[1]))) for a, b in zip(self.pieces, self.pieces[1:])]     # SEP across each jump
        fine = np.linspace(left, right, max(2, int((right-left)/2.)+1))
        values = self.sep(fine);i = int(np.nanargmax(values))
        self.best_speed, self.best_sep = float(fine[i]), float(values[i])
        if 0 < i < len(fine)-1 and not any(fine[i-1] < j < fine[i+1] for j in self.jumps):
            # The top of the parabola through the three best, so the line does not step with the grid above.
            a, b, c = values[i-1:i+2];curve = a-2.*b+c
            if curve < 0.:
                self.best_speed += .5*(a-c)/curve*(fine[1]-fine[0]);self.best_sep = float(self.sep(self.best_speed))

    def piece_of(self, speed):
        return sum(j < speed for j in self.jumps)

    def in_piece(self, k, speeds, slope=False):
        """SEP (or its slope along speed) of piece k at these speeds, each taken no further than the piece's own states."""
        low, high, curve, steep = self.pieces[min(k, len(self.pieces)-1)]
        speeds = np.asarray(speeds, dtype=float);values = np.array((steep if slope else curve)(np.clip(speeds, low, high)), dtype=float)
        for a, b in self.gaps:values[(speeds > a) & (speeds < b)] = math.nan
        return values

    def sep(self, speeds, slope=False):
        speeds = np.asarray(speeds, dtype=float);out = np.empty(speeds.shape)
        pieces = np.searchsorted(self.jumps, speeds) if self.jumps else np.zeros(speeds.shape, dtype=int)
        flat, index = out.reshape(-1), pieces.reshape(-1);given = speeds.reshape(-1)
        for k in set(index.tolist()):flat[index == k] = self.in_piece(k, given[index == k], slope)
        return out if out.shape else float(out)

    def slope(self, speeds):
        return self.sep(speeds, slope=True)

    def at_fractions(self, fractions):
        return self.sep(np.clip(self.left+fractions*(self.right-self.left), self.left, self.right))

    def by_a_jump(self, fractions, width):
        """Which of these fractions of the span lie within this width (km/h) of a jump in SEP."""
        speeds = self.left+fractions*(self.right-self.left);near = np.zeros(speeds.shape, dtype=bool)
        for jump in self.jumps:near |= np.abs(speeds-jump) < width
        return near


class Blend:
    """SEP between solved rows (Leo, 2026-11-21: jump lines traced across rows, fewer rows for jets).

    Jump lines: a jump found in neighbouring rows within `reach` km/h is one line (a track). Every row is cut at every
    track (a row a track does not reach is cut where the track ends, which changes nothing there), and rows are only
    ever blended part with part: no value is taken across a jump, and contours run up to it from both sides, as on
    the EM chart.

    Mach lines (MACH_LINES): between two jump lines every row is cut again at the same Mach numbers, MACH_STEP apart.
    What bends SEP along speed (the drag rise, the thrust tables) sits at a Mach number, while a row's ends move with
    indicated speed: lines of the same fraction of the whole span cut across those bends, and every one of them then
    looked like a kink between two rows. Within a band between two Mach lines the same fraction is the same Mach
    number; the bands next to an end or a jump line run to it, so the surface still meets them exactly.

    A segment is one such band of one part; it has the same index in every row, and zero width where its Mach lines lie
    outside the part."""
    def __init__(self, rows, reach):
        from air_state import speed_of_sound
        self.rows = rows = sorted(rows, key=lambda r:r.height)
        self.heights = np.array([r.height for r in rows])
        self.sound = np.array([float(speed_of_sound(r.height)) for r in rows])*3.6      # km/h
        tracks, open_ = [], []      # {row index: speed}
        for i, row in enumerate(rows):
            free, now = list(row.jumps), []
            for d, t, j in sorted((abs(t[i-1]-j), k, j) for k, t in enumerate(open_) for j in row.jumps):
                if d > reach or j not in free or i in open_[t]:continue
                open_[t][i] = j;free.remove(j);now.append(open_[t])
            for j in free:
                track = {i:j};tracks.append(track);now.append(track)
            open_ = now
        tracks.sort(key=lambda t:sorted(t.values())[len(t)//2])
        self.tracks = tracks
        machs = []
        if MACH_LINES:
            low = min(r.left/a for r, a in zip(rows, self.sound));high = max(r.right/a for r, a in zip(rows, self.sound))
            machs = [k*MACH_STEP for k in range(int(math.floor(low/MACH_STEP))+1, int(math.ceil(high/MACH_STEP)))]
        self.bands = len(machs)+1
        self.parts = len(tracks)+1
        bounds = np.empty((len(rows), self.parts*self.bands+1))
        for i, row in enumerate(rows):
            cuts = []
            for t in tracks:
                at = t[i] if i in t else t[min(t, key=lambda k:abs(k-i))]
                cuts.append(min(max(at, row.left, cuts[-1] if cuts else row.left), row.right))
            # a row's own jumps stay where they are: a line continued from another row never passes one
            for k, t in enumerate(tracks):
                if i in t:
                    cuts[k] = t[i]
                    for m in range(k):cuts[m] = min(cuts[m], t[i])
                    for m in range(k+1, len(cuts)):cuts[m] = max(cuts[m], t[i])
            cuts = [row.left]+cuts+[row.right]
            for part in range(self.parts):
                lo, hi = cuts[part], cuts[part+1]
                bounds[i, part*self.bands:(part+1)*self.bands] = [lo]+[min(max(m*self.sound[i], lo), hi) for m in machs]
            bounds[i, -1] = row.right
        self.bounds = bounds
        self.segments = self.parts*self.bands
        self.breaks = [part*self.bands for part in range(1, self.parts)]       # the bounds that are jump lines

    def reading(self, i, segments, fractions):
        """Row i's SEP at these fractions of these segments."""
        row = self.rows[i];lo, hi = self.bounds[i, segments], self.bounds[i, segments+1]
        speeds = lo+fractions*(hi-lo);out = np.empty(speeds.shape);parts = segments//self.bands
        for part in set(parts.tolist()):
            piece = row.piece_of(.5*(self.bounds[i, part*self.bands]+self.bounds[i, (part+1)*self.bands]))
            out[parts == part] = row.in_piece(piece, speeds[parts == part])
        return out

    def place(self, i, speeds):
        """(segments, fractions) of these speeds in row i."""
        cuts = self.bounds[i]
        segments = np.clip(np.searchsorted(cuts, speeds, side='right')-1, 0, self.segments-1)
        # a speed on a jump line belongs to the side its states are on; one on any other line to either
        width = cuts[segments+1]-cuts[segments]
        return segments, np.where(width > 1e-9, (speeds-cuts[segments])/np.where(width > 1e-9, width, 1.), 0.)

    def columns(self):
        """The surface's columns, one list for each stretch between jump lines: (segments, fractions)."""
        groups, group = [], None
        for k in range(self.segments):
            if k in self.breaks or group is None:
                group = [[], []];groups.append(group)
            width = float((self.bounds[:, k+1]-self.bounds[:, k]).max())
            if width <= 1e-9:continue
            fractions = np.linspace(0., 1., min(801, max(2, int(width/2.5)+1)))
            if group[0]:fractions = fractions[1:]       # the line it shares with the segment before
            group[0] += [k]*len(fractions);group[1] += fractions.tolist()
        return [(np.array(g[0]), np.array(g[1])) for g in groups if len(g[0]) > 1]

    def surface(self, spacing):
        """For each stretch between jump lines, speed and SEP on a grid of columns by altitude: the solved rows and
        rows blended between them every `spacing` m."""
        places = []     # (lower row, upper row, share of the upper)
        for i in range(len(self.rows)-1):
            n = max(1, int(math.ceil((self.heights[i+1]-self.heights[i])/spacing-1e-9)))
            places += [(i, i+1, k/n) for k in range(n)]
        places.append((len(self.rows)-1, len(self.rows)-1, 0.))
        heights = np.array([(1.-t)*self.heights[a]+t*self.heights[b] for a, b, t in places])
        groups = []
        for segments, fractions in self.columns():
            speed = np.array([self.bounds[i, segments]+fractions*(self.bounds[i, segments+1]-self.bounds[i, segments]) for i in range(len(self.rows))])
            sep = np.array([self.reading(i, segments, fractions) for i in range(len(self.rows))])
            groups.append(dict(altitude_m=heights, speed_kmh=np.array([(1.-t)*speed[a]+t*speed[b] for a, b, t in places]),
                               sep_mps=np.array([sep[a] if t == 0. else (1.-t)*sep[a]+t*sep[b] for a, b, t in places])))
        return dict(groups=groups)

    def at(self, speed, height):
        """SEP at one place, as the surface holds it (None outside the rows)."""
        if not self.heights[0] <= height <= self.heights[-1]:return None
        b = int(min(max(np.searchsorted(self.heights, height), 1), len(self.rows)-1)) if len(self.rows) > 1 else 0
        a = max(b-1, 0);t = 0. if a == b else (height-self.heights[a])/(self.heights[b]-self.heights[a])
        cuts = (1.-t)*self.bounds[a]+t*self.bounds[b]
        if not cuts[0] <= speed <= cuts[-1]:return None
        k = int(min(max(np.searchsorted(cuts, speed, side='right')-1, 0), self.segments-1))
        while k < self.segments-1 and cuts[k+1]-cuts[k] <= 1e-9:k += 1
        segments, fractions = np.array([k]), np.array([(speed-cuts[k])/max(cuts[k+1]-cuts[k], 1e-9)])
        value = (1.-t)*self.reading(a, segments, fractions)
        if t:value = value+t*self.reading(b, segments, fractions)
        return float(value[0])


class Envelope:
    """One aircraft entry's rows."""
    def __init__(self, config, entry, check):
        self.name, self.condition, self.check = entry['aircraft_id'], condition_of(config, entry), check
        self.low, self.high = float(config['speed_min_kmh']), float(config['speed_max_kmh'])
        self.bottom, self.top = config['altitude_min_m'], config['altitude_max_m']
        self.sep_tolerance = min(SAMPLING, key=lambda t:abs(t-config.get('sep_tolerance_mps', .5)))
        self.spacing, self.closest, self.step, self.bracket, self.end_tolerance = SAMPLING[self.sep_tolerance]
        self.speeds = np.arange(self.low, self.high, self.step).tolist()+[self.high]
        self.solves = 0
        self.chart_at = None
        self.point_solves = {}      # the point solver's calls by what asked for them (row_steps)
        self.carried = self.discovered = 0      # rows where the more-thrust rule changed states; where another state was found

    def solver(self, height):
        """The chart of this altitude and its point solve."""
        if not SHARED_ENGINE:return solver_at(self.name, self.condition, height)
        if self.chart_at is None:
            from engine_performance import level_charts_of
            self.chart_at = level_charts_of(level_chart(self.name, self.condition, height), lean=LEAN_ROWS)[1]
        return self.chart_at(height)

    def row(self, height, near, predicted=None, base=False):
        """The row at this altitude, or None if it holds no level flight. near: the row beside it, for seeds and where to
        start; predicted: where its two ends are expected; base: a row of the base grid (not one between two others)."""
        chart, solve = self.solver(height)
        self.check()
        engine = getattr(solve, 'native', None) if NATIVE_STEPS else None
        if engine is not None:return self.row_steps(chart, solve, engine, height, near, predicted, base or near is None)
        return self.row_points(chart, solve, height, near, predicted)

    def seeds(self, rows):
        """The solved states of these rows as starts for a clicked point's solve."""
        out = []
        for row in rows:
            solved = [s for s in sorted(row.samples.values(), key=lambda s:s['speed_kmh']) if s['valid']]
            for k, s in enumerate(solved):
                seed = s.get('seed')
                # A state's full record takes half as long as the state did: every third one, the two ends of the row
                # and the states next to a jump in SEP (a click there must start on the right side of it).
                wanted = k % SEED_EVERY == 0 or k == len(solved)-1 or any(abs(s['speed_kmh']-j) <= 2.*self.bracket for j in row.jumps)
                if seed is None and wanted and s.get('sid') is not None and not s['unchecked']:
                    try:seed = seed_of(row.engine.state_row(s['sid']))
                    except RuntimeError:seed = None
                if seed:out.append(dict(seed, speed_kmh=s['speed_kmh'], altitude_m=s['altitude_m'], sliding=bool(s.get('sliding'))))
        return out

    def row_steps(self, chart, solve, engine, height, near, predicted, base=False):
        """`row` by the march engine's level-flight step."""
        solve.here()        # the shared engine at this altitude
        grid, seen = self.speeds, {}
        upward = height >= UPWARD_FROM_M
        ratio = []          # indicated per true airspeed at this altitude

        # The engine's level step (with the sliding state at a blade-polar jump, as a chart column takes it); a build
        # without it has the plain correction.
        level = getattr(engine, 'level_step', None)
        if level is None:level = lambda sid, v, nfev, evals, slide=True:engine.debug_level(sid, v, nfev, evals)

        def sample(v, sid, valid, reasons, unchecked=False):
            s = dict(speed_kmh=v, altitude_m=height, valid=bool(valid or unchecked), unchecked=unchecked, reasons=list(reasons) or ['no candidate'], sid=sid)
            if s['valid']:
                info = engine.debug_info(sid)
                s.update(ps_mps=info['ps'], alpha_deg=info['alpha'], mach=info['mach'], thrust=info['thrust'], ias_kmh=None)
                pairs = [a/b for a, b in zip(info['rpm'], info['rpm_limit']) if b]
                if pairs:s['rpm_ratio'] = max(pairs)
                if info['slide']:s['sliding'] = True
            return s

        def seed_for(origin):
            if origin is None:return None
            if 'seed' not in origin:
                try:origin['seed'] = seed_of(engine.state_row(origin['sid'])) if origin.get('sid') is not None else None
                except RuntimeError:origin['seed'] = None
            return origin['seed']

        def state_id(p):
            rows = engine.rows
            for i in range(len(rows)-1, max(-1, len(rows)-200), -1):
                if rows[i] is p:return i
            return None

        def point(v, origin=None, why='start'):
            """The point solver's state at v: continued from the origin's state, then its other starts."""
            self.solves += 1;self.point_solves[why] = self.point_solves.get(why, 0)+1
            p = level_point(chart, solve, v, seed_for(origin))
            if ends_upward(p):
                self.solves += 1;p = settle(chart, solve, v, p)
            if p is None:return dict(sample(v, None, False, ()), complete=True)
            s = dict(sample(v, state_id(p), bool(p.get('valid')), p.get('reasons') or ()), complete=True, seed=seed_of(p))
            if s['valid'] and s['sid'] is None:
                s.update(ps_mps=float(p['ps_mps']), alpha_deg=p.get('alpha_deg'), mach=p.get('mach'), thrust=None)
            if s['valid'] and not ratio and p.get('ias_kmh'):ratio.append(float(p['ias_kmh'])/v)
            return s

        def landed(v, sid):
            """A state rejected only for ending its step upward, moved to the side that does not ascend (`settle`)."""
            alpha = engine.debug_info(sid)['sol'][0]
            for nudge in NUDGES_DEG:
                self.solves += 1
                q = engine.debug_correct(sid, v, 'load', 1., alpha=alpha-nudge)[0]
                if q >= 0:
                    info = engine.debug_info(q)
                    if info['valid'] or info['reasons'] != [NOT_CLOSED]:return sample(v, q, info['valid'], info['reasons'])
            return sample(v, sid, False, [NOT_CLOSED])

        def step(origin, v, full=False, robust=False):
            """The level-flight state at v continued from the origin sample's state. robust: where the correction does
            not close, again with the march's larger budget (for a speed where a state is expected: past an end it only
            takes longer to fail)."""
            if origin.get('sid') is None:return point(v, origin, 'no state id')
            self.solves += 1
            # the sliding state only with the larger budget: a first correction that does not close is tried regular again
            out = level(origin['sid'], v, STEP_NFEV, STEP_EVALS, bool(origin.get('sliding')))
            if out is None and robust:
                self.solves += 1
                out = level(origin['sid'], v, ROBUST_NFEV, ROBUST_EVALS)
            if out is None:return dict(sample(v, None, False, ()), weak=not robust)   # weak: the larger budget was not tried
            sid, valid, _, reasons = out
            if not valid and upward and reasons == [NOT_CLOSED]:
                return landed(v, sid) if full else sample(v, sid, False, reasons, unchecked=True)
            return sample(v, sid, valid, reasons)

        def twin_of(v):
            if near is None or getattr(near, 'engine', None) is not engine:return None
            return min((t for u, t in near.samples.items() if t['valid'] and abs(u-v) <= self.step), key=lambda t:abs(t['speed_kmh']-v), default=None)

        def nearest(v):
            solved = [u for u, t in seen.items() if t['valid']]
            return seen[min(solved, key=lambda u:abs(u-v))] if solved else None

        def at(v, origin=None, full=False, robust=False):
            """Inside the row (full False) a state failing only the vertical-step check counts as it stands."""
            if v in seen and not (robust and seen[v].get('weak')):
                if not (full and seen[v]['unchecked']):return seen[v]
                seen[v] = landed(v, seen[v]['sid'])
                return seen[v]
            origin = origin or nearest(v)
            seen[v] = step(origin, v, full, robust) if origin is not None else point(v, None, 'from rest')
            return seen[v]

        def reach(v, way):
            """(km/h from the state at v, going this way, to the limit it is closing in on, at the rate its margin falls
            along the row; that limit's label), or None if no margin is falling."""
            state = seen[v]
            if state.get('sid') is None:return None
            inner = min((u for u, t in seen.items() if t['valid'] and t.get('sid') is not None and 1e-9 < (v-u)*way <= self.step),
                        key=lambda u:abs(u-v), default=None)
            if inner is None:return None
            margins, before = engine.debug_info(state['sid'])['margins'], engine.debug_info(seen[inner]['sid'])['margins']
            best = None
            for kind, label in MARGIN_LABELS.items():
                m, m0 = margins.get(kind), before.get(kind)
                if m is None or m0 is None or not 0. <= m < m0:continue
                distance = m*abs(v-inner)/(m0-m)
                if best is None or distance < best[0]:best = (distance, label)
            return best

        def numerical(s):
            return not s['valid'] and NUMERICAL >= set(s['reasons'])

        def fresh(v):
            """The state at a speed with nothing of this row to continue from: from the row beside's state there, then
            the point solver (from that state, or from rest)."""
            twin = twin_of(v)
            if twin is not None:
                seen[v] = step(twin, v, robust=True)
                if not numerical(seen[v]):return seen[v]
            seen[v] = point(v, twin, 'row start')
            return seen[v]

        from em_speed_limits import speed_limits
        limits = speed_limits(chart.fm, chart.config)
        top_speed = float(limits['speed_kmh']) if limits['enforced'] else None

        # Start from the best-SEP speed of the row beside; failing that, look across its span (or the whole range).
        start = None
        if near is not None:
            i = min(range(len(grid)), key=lambda k:abs(grid[k]-near.best_speed))
            if fresh(grid[i])['valid']:start = i
        if start is None:
            lo, hi = (near.left-3.*self.step, near.right+3.*self.step) if near is not None else (self.low, self.high)
            start = next((i for i, v in enumerate(grid) if lo <= v <= hi and (seen[v] if v in seen else fresh(v))['valid']), None)
        if start is None:return None

        def march(i, way, expected):
            """Outward along the grid until level flight has clearly ended: the last solved index and the rejected speeds past it."""
            outer, rejected, allowed = i, [], MISSES if near is not None else 2*MISSES
            while 0 <= i+way < len(grid):
                i += way;v = grid[i];twin = twin_of(v)
                if rejected:
                    # Past a rejected speed: from the row beside's state at this speed, if it has one (then the point
                    # solver where that finds nothing), or continued from the outermost state of this row.
                    s = at(v, twin or seen[grid[outer]])
                    if numerical(s) and (twin is not None or near is None):s = seen[v] = point(v, twin or seen[grid[outer]], 'past a rejected speed')
                else:
                    s = at(v, seen[grid[outer]], robust=twin is not None)
                    if numerical(s) and twin is not None:s = seen[v] = step(twin, v, robust=True)      # the row beside solved this speed
                    if numerical(s) and outer == i-way:
                        # A step too long for the correction? Again from halfway.
                        halfway = at(.5*(v+grid[outer]), seen[grid[outer]])
                        if halfway['valid']:s = seen[v] = step(halfway, v)
                    if numerical(s) and near is None:s = seen[v] = point(v, seen[grid[outer]], 'first row')
                if s['valid']:outer, rejected = i, []
                else:
                    rejected.append(v)
                    if way > 0 and HARD_LIMITS & set(s['reasons']):break
                    if len(rejected) >= allowed and (expected is None or (v-expected)*way >= 0.):break
            return outer, rejected

        def end(i, way, rejected, expected):
            """The fastest or slowest level state past grid index i, within the bracket, and what stops it there."""
            # The outermost state must pass every check; if it does not, the one inside it is the start.
            while not at(grid[i], full=True)['valid']:
                rejected = [grid[i]]+rejected;i -= way
                if not 0 <= i < len(grid) or grid[i] not in seen:return None, None
            if not rejected:return grid[i], SPEED_END
            good, tries = grid[i], 3
            if way > 0 and top_speed is not None and good < top_speed < rejected[0]:expected = top_speed
            probes = [expected] if expected is not None and min(good, rejected[0]) < expected < max(good, rejected[0]) else []
            first, trusted, bridges = bool(probes), True, 2

            def width():
                return self.bracket*(HARD_FINE if HARD_LIMITS & set(seen[rejected[0]]['reasons']) else 1.)
            while True:
                while abs(rejected[0]-good) > width():
                    limit, gap = reach(good, way) if trusted else None, abs(rejected[0]-good)
                    if limit is not None and limit[0] <= END_EXACT*self.bracket:
                        # The limit should be right here: a state just past it must not exist. If one does, the margin
                        # turned before reaching 0 (Bf 109 F-4 at 70% pitch, 5,617 m: level flight goes on 23 km/h
                        # below where the stall margin first pointed), and the search goes on by halving.
                        v = good+way*max(2.*limit[0], .25*self.bracket)
                        if (rejected[0]-v)*way > 0.:
                            s = at(v, seen[good], full=True, robust=True)
                            if s['valid']:good, trusted = v, False
                            else:rejected = [v]+rejected
                            continue
                    if probes:v = probes.pop()
                    elif limit is not None and .9*limit[0] < .5*gap:v = good+way*.9*limit[0]      # just inside that limit
                    else:v = .5*(good+rejected[0])
                    s = at(v, seen[good], full=True, robust=limit is None or abs(v-good) < limit[0])
                    if s['valid']:good = v
                    else:rejected = [v]+rejected
                    if first:
                        # The neighbours' end is usually close: look just past it next.
                        first = False;w = v+(way if s['valid'] else -way)*2.*self.bracket
                        if min(good, rejected[0]) < w < max(good, rejected[0]):probes.append(w)
                if seen[rejected[0]].get('weak'):
                    # The speed just beyond was rejected by the small budget only: with the larger one. If it has a state
                    # after all, the other speeds rejected that way count for nothing, and the search goes on with the
                    # larger budget throughout.
                    s = at(rejected[0], seen[good], full=True, robust=True)
                    if s['valid']:
                        good, trusted = rejected[0], False
                        rejected = [v for v in rejected[1:] if not seen[v].get('weak')]
                        while not rejected:
                            v = min(max(good+way*self.step, self.low), self.high)
                            if v == good:return good, SPEED_END
                            s = at(v, seen[good], full=True, robust=True)
                            if s['valid']:good = v
                            else:rejected = [v]
                        continue
                limit = reach(good, way) if numerical(seen[rejected[0]]) else None
                if limit is not None and limit[0] <= END_REACH*self.bracket:return good, limit[1]
                if numerical(seen[rejected[0]]) and not seen[rejected[0]].get('complete'):
                    # The correction found nothing just beyond the end, and no limit is closing in on the last state: the
                    # point solver, with all its starts, confirms the end (and names what stops level flight there). If
                    # it has a state, the end lies beyond.
                    s = seen[rejected[0]] = point(rejected[0], seen[good], 'end confirmed')
                    if s['valid']:
                        self.point_solves['end moved'] = self.point_solves.get('end moved', 0)+1
                        good, rejected = rejected[0], rejected[1:]
                        while not rejected:
                            v = min(max(good+way*self.step, self.low), self.high)
                            if v == good:return good, SPEED_END
                            s = seen[v] = point(v, seen[good], 'beyond a moved end')
                            if s['valid']:good = v
                            else:rejected = [v]
                        continue
                if numerical(seen[rejected[0]]) and bridges:
                    # No limit ends the row here: the states only stop (Do 17 Z-2: its propellers have no steady state
                    # over a few km/h at some altitudes). Inside a row such a stretch is drawn across if it is narrow
                    # (BRIDGED_SAMPLES steps); so the row goes on if level flight is found again within that distance,
                    # looked for from rest at a few speeds.
                    bridges -= 1;found = None
                    for offset in (4.*self.bracket, .25*self.step, .5*self.step, self.step, 2.*self.step, BRIDGED_SAMPLES*self.step):
                        v = good+way*offset
                        if not self.low <= v <= self.high:break
                        if v not in seen or not (seen[v]['valid'] or seen[v].get('complete')):seen[v] = point(v, None, 'beyond a hole')
                        if seen[v]['valid']:
                            found = v;break
                    if found is not None:
                        good, rejected = found, [u for u in rejected if (u-found)*way > 0.]
                        while not rejected:
                            v = min(max(good+way*self.step, self.low), self.high)
                            if v == good:return good, SPEED_END
                            s = at(v, seen[good], full=True, robust=True)
                            if s['valid']:good = v
                            else:rejected = [v]
                        trusted = True
                        continue
                # The bracket has closed on a state that only failed to solve: that is no limit, so look just beyond it.
                beyond = rejected[0]+way*2.*self.bracket
                if not numerical(seen[rejected[0]]) or not tries or len(rejected) < 2 or (rejected[1]-beyond)*way <= 0.:break
                tries -= 1
                if beyond not in seen:seen[beyond] = point(beyond, seen[good], 'just beyond')
                if not seen[beyond]['valid']:break
                good, rejected = beyond, [v for v in rejected if (v-beyond)*way > 0.]
            for v in rejected[:3]:
                # What stops it: the point solver's reasons at the nearest rejected speeds, until one is physical.
                if limit_label([seen[v]['reasons']]) != UNRESOLVED:break
                if not seen[v].get('complete'):seen[v] = point(v, seen[good], 'label')
            return good, limit_label([seen[v]['reasons'] for v in rejected])

        expected = predicted or (None, None)
        upper, rejected = march(start, 1, expected[1]);right, right_label = end(upper, 1, rejected, expected[1])
        lower, rejected = march(start, -1, expected[0]);left, left_label = end(lower, -1, rejected, expected[0])
        if left is None or right is None or not left <= right:return None
        # States rejected inside the span: one more try from the far side; what stays numerical and narrow is bridged.
        for v in sorted(v for v, s in seen.items() if not s['valid'] and left < v < right):
            faster = min((u for u, s in seen.items() if u > v and s['valid']), default=None)
            if faster is not None:
                s = step(seen[faster], v, robust=True)
                if s['valid']:seen[v] = s
        # SEP jumps between speeds where the propeller or the engine changes branch: found by what the slopes on either
        # side leave unexplained, and closed in on, so contours and the best-climb line do not step with the speed grid.
        spacing = .25*self.step

        def find_jumps():
            """Where SEP, or thrust, steps between two neighbouring states by more than the slopes on either side explain:
            closed in on down to the bracket. Thrust counts by what it is worth in SEP (its change times the speed, in
            m/s): a weaker state's SEP can climb steeply towards the speed where it ends, and its slope then explained
            the step to the other state (B-17G at 4,740 m: 2 m/s at 252 km/h, with a band of two states below it)."""
            jumps = []
            for _ in range(9):
                knots = [v for v in sorted(seen) if seen[v]['valid'] and left+spacing < v < right-spacing]
                between, jumps = [], []
                for k in range(1, len(knots)-2):
                    p, a, b, q = knots[k-1:k+3]

                    def unexplained(key):
                        yp, ya, yb, yq = (seen[u].get(key) for u in (p, a, b, q))
                        if None in (yp, ya, yb, yq):return 0.
                        rise = yb-ya
                        return min(abs(rise-(ya-yp)/(a-p)*(b-a)), abs(rise-(yq-yb)/(q-b)*(b-a)))
                    if (unexplained('ps_mps') > 2.*self.sep_tolerance*(1.+abs(seen[a]['ps_mps'])/20.)
                            or unexplained('thrust')*.5*(a+b)/3.6 > JUMP_THRUST*self.sep_tolerance):
                        if b-a <= self.bracket:jumps.append(.5*(a+b))
                        else:between.append((.5*(a+b), a, b))
                progress = False
                for v, a, b in between:
                    if v in seen:continue
                    # from the state beside it; where that one does not reach (its state ends before), from the other
                    first, second = (a, b) if v-a <= b-v else (b, a)
                    state = at(v, seen[first])
                    if not state['valid']:
                        other = step(seen[second], v)
                        if other['valid']:state = seen[v] = other
                    progress = progress or state['valid']
                if not progress:break
            return jumps

        def more(new, old):
            return new is not None and old is not None and new > old+PREFER*max(abs(new), abs(old), 1e-3)

        def carry(origin, way):
            """The state at this speed continued to the row's next states this way, in steps of at most CARRY km/h, and
            on from each where it has more thrust than the state there (which it replaces, with the steps between).
            Where it ends between two of them, the row's state from beyond is continued back to it, and the carried
            steps are kept if they have more thrust than that. Whether anything changed."""
            current, changed = seen[origin], False
            if current.get('thrust') is None or current.get('sid') is None:return False
            while True:
                v = current['speed_kmh']
                knot = min((u for u, t in seen.items() if t['valid'] and (u-v)*way > 1e-9 and left <= u <= right), key=lambda u:abs(u-v), default=None)
                if knot is None:break
                state, trail, reach = current, [], CARRY
                while (knot-state['speed_kmh'])*way > 1e-9:
                    at_ = state['speed_kmh']
                    target = knot if abs(knot-at_) <= reach*(1.+1e-9) else at_+way*reach
                    following = step(state, target)
                    if not following['valid'] or following.get('thrust') is None:
                        if reach <= self.bracket:break
                        reach *= .5;continue
                    state, reach = following, min(2.*reach, CARRY)
                    if target != knot:trail.append(following)
                if state['speed_kmh'] == knot:
                    if not more(state['thrust'], seen[knot].get('thrust')):break      # the same state, or a weaker one
                    kept = trail+[state]
                else:
                    # it ends inside: the row's state continued back to just beyond where it ends
                    if not trail:break
                    beyond = trail[-1]['speed_kmh']+way*self.bracket
                    back = step(seen[knot], beyond) if (knot-beyond)*way > 1e-9 else seen[knot]
                    if not back['valid'] or not more(trail[-1]['thrust'], back.get('thrust')):break
                    kept = trail+[back]
                for u in [u for u in seen if (u-v)*way > 1e-9 and (knot-u)*way > 1e-9]:del seen[u]
                for t in kept:
                    t['knot'] = True;seen[t['speed_kmh']] = t
                changed = True
                if state['speed_kmh'] != knot:break
                current = state
            return changed

        def prefer_thrust(jumps):
            """The more-thrust rule at these jumps: each side's state carried across. The row's jumps afterwards."""
            if not MORE_THRUST:return jumps
            for _ in range(CARRY_ROUNDS):
                changed = False
                for jump in jumps:
                    for way in (1, -1):
                        origin = (max if way > 0 else min)((u for u, t in seen.items() if t['valid'] and (jump-u)*way > 0.), default=None)
                        if origin is not None and carry(origin, way):changed = True
                if not changed:break
                self.carried += 1
                jumps = find_jumps()
            return jumps

        def off(value, true):
            return abs(value-true) > REFINE*self.sep_tolerance*(1.+abs(true)/20.)

        def refine(jumps):
            """More knots where the curve through the row's knots is off (REFINE). The knots the curve through the others
            missed: where the row bends more than a smooth curve explains."""
            settled, suspects = set(), set()      # stretches next to an end or a jump that their midpoint has confirmed
            for _ in range(REFINE_ROUNDS):
                knots = [v for v in sorted(seen) if seen[v]['valid'] and (v in (left, right) or seen[v].get('knot') or left+spacing < v < right-spacing)]
                # The curve is one piece between jumps: each piece is checked by itself.
                pieces, piece = [], []
                for v in knots:
                    if piece and any(piece[-1] < j < v for j in jumps):
                        pieces.append(piece);piece = []
                    piece.append(v)
                pieces.append(piece)
                wanted = set()
                for xs in pieces:
                    if len(xs) < 2:continue
                    ys = [seen[u]['ps_mps'] for u in xs]
                    for k in range(1, len(xs)-1):
                        lo, hi = max(0, k-3), k+4
                        if off(makima_at(xs[lo:k]+xs[k+1:hi], ys[lo:k]+ys[k+1:hi], xs[k]), ys[k]):
                            suspects.add(xs[k])
                            wanted.update(.5*(a+b) for a, b in ((xs[k-1], xs[k]), (xs[k], xs[k+1])) if b-a > 2.*REFINE_GAP)
                    for a, b, lo, hi in ((xs[0], xs[1], 0, 5), (xs[-2], xs[-1], max(0, len(xs)-5), len(xs))):
                        if b-a <= 2.*REFINE_GAP or (a, b) in settled:continue
                        middle = .5*(a+b);was = makima_at(xs[lo:hi], ys[lo:hi], middle)
                        state = at(middle);state['knot'] = True
                        if not state['valid'] or not off(was, state['ps_mps']):settled.add((a, middle) if a == xs[0] else (middle, b))
                wanted = [v for v in wanted if v not in seen]
                for v in wanted:at(v)['knot'] = True
                if not wanted and not any(b-a > 2.*REFINE_GAP and (a, b) not in settled for xs in pieces if len(xs) > 1
                                          for a, b in ((xs[0], xs[1]), (xs[-2], xs[-1]))):break
            return suspects

        def discover(suspects):
            """Other states of the propulsion that a march along the row does not meet (a propeller's second state over
            a band of speeds begins without a jump on the state the march is on): looked for where the row bends
            (each state beside a suspect knot carried across it), at a few speeds from the row beside's states there,
            and in the rows of the base grid from rest. One with more thrust is taken and carried both ways."""
            changed = False
            for v in sorted(suspects):
                for way in (1, -1):
                    origin = (max if way > 0 else min)((u for u, t in seen.items() if t['valid'] and (v-u)*way > 1e-9), default=None)
                    if origin is not None and carry(origin, way):changed = True
            solved = [u for u in sorted(seen) if seen[u]['valid'] and left < u < right and seen[u].get('thrust') is not None]
            if not solved:return changed
            chosen = sorted({solved[min(len(solved)-1, int(f*len(solved)))] for f in DISCOVER_AT})
            for v in chosen:
                if v not in seen or not seen[v]['valid']:continue
                found = []
                twin = twin_of(v)
                if twin is not None and twin.get('sid') is not None:found.append(step(twin, v))
                if base:found.append(point(v, None, 'discovery'))
                for state in found:
                    if state['valid'] and more(state.get('thrust'), seen[v].get('thrust')):
                        state['knot'] = True;seen[v] = state
                        carry(v, 1);carry(v, -1);changed = True
            return changed

        jumps = prefer_thrust(find_jumps())
        suspects = refine(jumps)
        if MORE_THRUST and DISCOVER and chart.is_prop and discover(suspects):
            self.discovered += 1
            jumps = prefer_thrust(find_jumps())
            refine(jumps)
        solved = sorted(v for v, s in seen.items() if s['valid'] and left <= v <= right)
        if not ratio:
            # indicated airspeed of the row's states: one full record gives the ratio at this altitude
            first = next((seen[v] for v in solved if seen[v].get('sid') is not None and not seen[v]['unchecked']), None)
            try:record = engine.state_row(first['sid']) if first else None
            except RuntimeError:record = None
            if record and record.get('ias_kmh'):ratio.append(float(record['ias_kmh'])/first['speed_kmh'])
        for v in solved:
            if seen[v].get('ias_kmh') is None and ratio:seen[v]['ias_kmh'] = ratio[0]*v
        gaps = []
        for a, b in zip(solved, solved[1:]):
            between = [seen[v]['reasons'] for v in seen if a < v < b]
            if between and (b-a > BRIDGED_SAMPLES*self.step or not all(NUMERICAL >= set(r) for r in between)):gaps.append((a, b))
        row = Row(height, seen, left, right, (left_label, right_label), gaps, spacing, jumps)
        row.engine = engine
        return row

    def row_points(self, chart, solve, height, near, predicted):
        """`row` by the point solver alone (no compiled engine, or NATIVE_STEPS off)."""
        grid, seen = self.speeds, {}

        def nearest_seed(v):
            for source in (seen, near.samples if near else {}):
                solved = [u for u, s in source.items() if s['valid']]
                if solved:return source[min(solved, key=lambda u:abs(u-v))]['seed']
            return None

        def number(value):
            return None if value is None else float(value)

        def store(v, p, unchecked, quick=False):
            seen[v] = sample = dict(speed_kmh=v, altitude_m=height, valid=bool(p and (p.get('valid') or unchecked)), unchecked=unchecked,
                                    reasons=list((p or {}).get('reasons') or ['no candidate']))
            if sample['valid']:
                sample.update(ps_mps=float(p['ps_mps']), alpha_deg=number(p.get('alpha_deg')), ias_kmh=number(p.get('ias_kmh')),
                              mach=number(p.get('mach')), seed=seed_of(p))
                ratio = rpm_ratio(p)
                if ratio is not None:sample['rpm_ratio'] = ratio
            elif quick and p is None:sample['quick'] = True     # only the continued solve was tried
            return sample

        def at(v, seed=None, full=False, quick=False):
            """Inside the row (full False) a state failing only the vertical-step check counts as it stands. quick: only
            the solve continued from the seed (`continued_only`); a speed rejected that way is solved completely when
            asked for again without it."""
            if v in seen and (not full or not seen[v]['unchecked']) and (quick or not seen[v].get('quick')):return seen[v]
            self.solves += 1
            first = seed or nearest_seed(v)
            quick = quick and QUICK_FAILS and first is not None
            p = (continued_only if quick else level_point)(chart, solve, v, first)
            if full and ends_upward(p):
                self.solves += 1;p = settle(chart, solve, v, p)
            return store(v, p, ends_upward(p) and not full, quick)

        # Start from the best-SEP speed of the row beside; failing that, look across its span (or the whole range).
        start = None
        if near is not None:
            i = min(range(len(grid)), key=lambda k:abs(grid[k]-near.best_speed))
            if at(grid[i])['valid']:start = i
        if start is None:
            lo, hi = (near.left-3.*self.step, near.right+3.*self.step) if near is not None else (self.low, self.high)
            start = next((i for i, v in enumerate(grid) if lo <= v <= hi and at(v)['valid']), None)
        if start is None:return None

        def numerical(s):
            return not s['valid'] and NUMERICAL >= set(s['reasons'])

        def march(i, way, expected):
            """Outward along the grid until level flight has clearly ended: the last solved index and the rejected speeds past it."""
            outer, rejected, allowed = i, [], MISSES if near is not None else 2*MISSES
            while 0 <= i+way < len(grid):
                i += way
                twin = None if near is None else min((t for u, t in near.samples.items() if t['valid'] and abs(u-grid[i]) <= self.step),
                                                     key=lambda t:abs(t['speed_kmh']-grid[i]), default=None)
                if rejected and near is not None and QUICK_FAILS:
                    # Past a rejected speed: continued from the row beside's state at this speed, or from the outermost
                    # of this row. Where the row beside has a state and that start finds nothing, the complete solve.
                    s = at(grid[i], twin['seed'] if twin else seen[grid[outer]]['seed'], quick=True)
                    if twin and s.get('quick'):s = at(grid[i], twin['seed'])
                else:
                    s = at(grid[i])
                    if numerical(s) and twin:
                        # The row beside solved this speed: again from its state.
                        del seen[grid[i]];s = at(grid[i], twin['seed'])
                if numerical(s) and outer == i-way:
                    # A step too long for the solve? Again from halfway.
                    halfway = at(.5*(grid[i]+grid[outer]), seen[grid[outer]]['seed'])
                    if halfway['valid']:
                        del seen[grid[i]];s = at(grid[i], halfway['seed'])
                if s['valid']:outer, rejected = i, []
                else:
                    rejected.append(grid[i])
                    if way > 0 and HARD_LIMITS & set(s['reasons']):break
                    if len(rejected) >= allowed and (expected is None or (grid[i]-expected)*way >= 0.):break
            return outer, rejected

        def end(i, way, rejected, expected):
            """The fastest or slowest level state past grid index i, within the bracket, and what stops it there."""
            # The outermost state must pass every check; if it does not, the one inside it is the start.
            while not at(grid[i], full=True)['valid']:
                rejected = [grid[i]]+rejected;i -= way
                if not 0 <= i < len(grid) or grid[i] not in seen:return None, None
            if not rejected:return grid[i], SPEED_END
            good, tries = grid[i], 3
            probes = [expected] if expected is not None and min(good, rejected[0]) < expected < max(good, rejected[0]) else []
            first = bool(probes)
            while True:
                while abs(rejected[0]-good) > self.bracket:
                    v = probes.pop() if probes else .5*(good+rejected[0])
                    s = at(v, seen[good]['seed'], full=True, quick=True)
                    if s['valid']:good = v
                    else:rejected = [v]+rejected
                    if first:
                        # The neighbours' end is usually close: look just past it next.
                        first = False;w = v+(way if s['valid'] else -way)*2.*self.bracket
                        if min(good, rejected[0]) < w < max(good, rejected[0]):probes.append(w)
                if seen[rejected[0]].get('quick') and at(rejected[0], seen[good]['seed'], full=True)['valid']:
                    # The complete solve has a state where the continued one found none: the end lies beyond it.
                    good, rejected = rejected[0], rejected[1:]
                    continue
                # The bracket has closed on a state that only failed to solve: that is no limit, so look just beyond it.
                beyond = rejected[0]+way*2.*self.bracket
                if not numerical(seen[rejected[0]]) or not tries or len(rejected) < 2 or (rejected[1]-beyond)*way <= 0.:break
                tries -= 1
                if not at(beyond, seen[good]['seed'], full=True)['valid']:break
                good, rejected = beyond, [v for v in rejected if (v-beyond)*way > 0.]
            for v in rejected[:3]:
                # What stops it: the complete solve's reasons of the nearest rejected speeds, until one is physical.
                if limit_label([seen[v]['reasons']]) != UNRESOLVED:break
                if seen[v].get('quick'):at(v, seen[good]['seed'], full=True)
            return good, limit_label([seen[v]['reasons'] for v in rejected])

        expected = predicted or (None, None)
        upper, rejected = march(start, 1, expected[1]);right, right_label = end(upper, 1, rejected, expected[1])
        lower, rejected = march(start, -1, expected[0]);left, left_label = end(lower, -1, rejected, expected[0])
        if left is None or right is None or not left <= right:return None
        # States rejected inside the span: one more try from the far side; what stays numerical and narrow is bridged.
        for v in sorted(v for v, s in seen.items() if not s['valid'] and left < v < right):
            faster = min((u for u, s in seen.items() if u > v and s['valid']), default=None)
            if faster is not None:
                self.solves += 1;p = level_point(chart, solve, v, seen[faster]['seed'])
                if p and p.get('valid'):store(v, p, False)
        # SEP jumps between speeds where the propeller or the engine changes branch: found by what the slopes on either
        # side leave unexplained, and closed in on, so contours and the best-climb line do not step with the speed grid.
        spacing, jumps = .25*self.step, []
        for _ in range(9):
            knots = [v for v in sorted(seen) if seen[v]['valid'] and left+spacing < v < right-spacing]
            between, jumps = [], []
            for k in range(1, len(knots)-2):
                p, a, b, q = knots[k-1:k+3]
                ya, yb = seen[a]['ps_mps'], seen[b]['ps_mps']
                rise, below, above = yb-ya, (ya-seen[p]['ps_mps'])/(a-p)*(b-a), (seen[q]['ps_mps']-yb)/(q-b)*(b-a)
                if min(abs(rise-below), abs(rise-above)) > 2.*self.sep_tolerance*(1.+abs(ya)/20.):
                    if b-a <= self.bracket:jumps.append(.5*(a+b))
                    else:between.append(.5*(a+b))
            if not between:break
            for v in between:
                if v not in seen:at(v, seen[min(knots, key=lambda u:abs(u-v))]['seed'])
        solved = sorted(v for v, s in seen.items() if s['valid'] and left <= v <= right)
        gaps = []
        for a, b in zip(solved, solved[1:]):
            between = [seen[v]['reasons'] for v in seen if a < v < b]
            if between and (b-a > BRIDGED_SAMPLES*self.step or not all(NUMERICAL >= set(r) for r in between)):gaps.append((a, b))
        return Row(height, seen, left, right, (left_label, right_label), gaps, spacing, jumps)

    def mismatch(self, below, middle, above, beyond=()):
        """How far a row is from what the rows on either side predict (Blend): SEP (m/s, kinder where SEP is far from
        zero, as contours lie further apart there, or changes fast along the row), and its two ends and its jumps
        (km/h, against their own tolerance). beyond: the next rows out, below and above. Where the curve through
        those as well passes the row (SMOOTH of the straight line's miss), SEP bends smoothly with altitude there, and
        what is left between the rows once this one is in is a quarter of the miss: it counts as 1/ROW_SMOOTH of it.
        A kink (a critical altitude, a table's corner) is missed by both, and counts in full."""
        outer = [r for r in beyond if r is not None]
        ordered = sorted([below, middle, above]+outer, key=lambda r:r.height)
        blend = Blend(ordered, JUMP_REACH*self.step)
        lower, here, upper = (ordered.index(r) for r in (below, middle, above))
        others = [i for i in range(len(ordered)) if i != here]
        t = (middle.height-below.height)/(above.height-below.height)
        speeds = np.array(sorted(u for u, state in middle.samples.items() if state['valid'] and middle.left <= u <= middle.right))
        worst, where = 0., None
        if speeds.size:
            segments, fractions = blend.place(here, speeds)
            values = blend.reading(here, segments, fractions)
            scale = 1.+np.abs(values)/20.
            if STEEP_KIND:scale = np.maximum(scale, np.abs(middle.slope(speeds))*self.end_tolerance/self.sep_tolerance)
            read = {i:blend.reading(i, segments, fractions) for i in others}
            errors = np.abs((1.-t)*read[lower]+t*read[upper]-values)/scale
            if len(others) > 2 and ROW_SMOOTH > 1.:
                curve = 0.
                for i in others:
                    weight = 1.
                    for k in others:
                        if k != i:weight *= (middle.height-ordered[k].height)/(ordered[i].height-ordered[k].height)
                    curve = curve+weight*read[i]
                errors = np.where(np.abs(curve-values)/scale <= SMOOTH*errors, errors/ROW_SMOOTH, errors)
            # A jump that not all three rows have (it ends between them, or is too small to be found in one) is no line
            # to read along: the states next to it are left out, as the ones next to every jump were before.
            ending = False
            for track in blend.tracks:
                if not all(i in track for i in (lower, here, upper)):
                    for i in (lower, here, upper):
                        if i in track:
                            errors[np.abs(speeds-track[i]) < self.step] = math.nan
                            row = ordered[i];size = row.steps[min(range(len(row.jumps)), key=lambda k:abs(row.jumps[k]-track[i]))]
                            # a jump worth more than JUMP_END tolerances ends between these rows: more rows, to find where
                            ending = ending or size > JUMP_END*self.sep_tolerance*(1.+abs(float(row.sep(track[i])))/20.)
            if np.isfinite(errors).any():
                worst = float(np.nanmax(errors));where = float(speeds[int(np.nanargmax(errors))])
            if ending:worst = max(worst, 2.*self.sep_tolerance)
        # the ends, and the jumps all three rows have
        moved = [abs((1.-t)*blend.bounds[lower, k]+t*blend.bounds[upper, k]-blend.bounds[here, k]) for k in (0, blend.segments)]
        moved += [abs((1.-t)*track[lower]+t*track[upper]-track[here]) for track in blend.tracks if all(i in track for i in (lower, here, upper))]
        if MISMATCH_LOG is not None:MISMATCH_LOG.append((middle.height, above.height-below.height, worst/self.sep_tolerance, where, [round(m/self.end_tolerance, 2) for m in moved]))
        return max(worst, self.sep_tolerance*max(moved)/self.end_tolerance)

    def rows(self, report=None):
        heights = np.arange(self.bottom, self.top, self.spacing).tolist()+[self.top]
        rows, near, empty = [], None, None
        self.finished = rows        # what a stopped calculation keeps

        def ahead(height):
            if len(rows) < 2:return (near.left, near.right) if near is not None else None
            a, b = rows[-2], rows[-1];f = (height-b.height)/(b.height-a.height)
            return b.left+f*(b.left-a.left), b.right+f*(b.right-a.right)
        for height in heights:
            row = self.row(height, near, ahead(height), base=True)
            if report:report(.4*len(rows)/len(heights))
            if row is None:
                if near is None:continue
                empty = height;break
            rows.append(row);near = row
            if len(rows) == 1 and GRID_STATES:
                wider = int(round((row.right-row.left)/(self.step*GRID_STATES)))
                if wider > 1:
                    self.step *= wider;self.speeds = np.arange(self.low, self.high, self.step).tolist()+[self.high]
        if not rows:return rows
        # The top of level flight, if it lies inside the range.
        if empty is not None:
            while empty-near.height > self.closest:
                middle = .5*(empty+near.height);row = self.row(middle, near, (near.left, near.right))
                if row is None:empty = middle
                else:rows.append(row);near = row
        pairs = list(zip(rows, rows[1:]));solved = 0
        while pairs:
            below, above = pairs.pop()
            if above.height-below.height <= self.closest*(1.+1e-9):continue
            middle = self.row(.5*(below.height+above.height), below, (.5*(below.left+above.left), .5*(below.right+above.right)))
            solved += 1
            if report:report(.4+.6*solved/(solved+len(pairs)+1))
            if middle is None:continue
            rows.append(middle)
            lower = max((r for r in rows if r.height < below.height), key=lambda r:r.height, default=None)
            higher = min((r for r in rows if r.height > above.height), key=lambda r:r.height, default=None)
            if self.mismatch(below, middle, above, (lower, higher)) > self.sep_tolerance:
                pairs += [(below, middle), (middle, above)]
        rows.sort(key=lambda r:r.height)
        return rows


def surface_of(rows, reach, spacing):
    """SEP between the rows, on a grid for each segment between jump lines (Blend.surface)."""
    return Blend(rows, reach).surface(spacing)


def round_step(value):
    """The nearest of 1, 2, 5 x 10^n on a log scale."""
    return min((m*10.**n for m in (1., 2., 5.) for n in range(-1, 4)), key=lambda s:abs(math.log(s/value)))


def contour_levels(aircraft):
    """SEP levels for what these aircraft actually reach (Leo, 2026-10-30): even steps from 0 up to just under the best
    SEP (a higher one would only be a sliver), about five of them, and two wider ones below 0, where SEP falls away fast
    at high speed and lines bunch. Levels are shared, so a much slower aircraft in the comparison adds a few of its own
    rather than being left with no line but 0."""
    peaks = [max(a['best_climb']['ps_mps']) for a in aircraft if a['best_climb']['ps_mps']]
    if not peaks:return None
    peak = max(peaks);low = min(min(a['best_climb']['ps_mps']) for a in aircraft if a['best_climb']['ps_mps'])
    step = round_step(peak/5. if peak > 0. else -low/5.)
    if peak <= 0.:
        # All above the ceiling: from the best SEP down.
        first = math.floor(peak/step)
        return sorted(({0.}|{(first-k)*step for k in range(5)}), reverse=True)
    levels = {k*step for k in range(1, int(.9*peak/step)+1)}|{0., -2.*step, -5.*step}
    for own in peaks:
        if own > 0. and sum(0. < level <= .9*own for level in levels) < 2:
            small = round_step(own/3.)
            levels |= {k*small for k in range(1, int(.9*own/small)+1)}
    return sorted(levels, reverse=True)


def contours_of(surface, levels):
    paths = []
    for part in surface['groups']:
        heights, speed = part['altitude_m'], part['speed_kmh']
        if len(heights) < 2:continue
        z = np.ma.masked_invalid(part['sep_mps'])
        if z.count() < 4 or not z.max() > z.min():continue
        contour = contourpy.contour_generator(x=np.arange(speed.shape[1], dtype=float), y=np.arange(len(heights), dtype=float), z=z, name='mpl2014', corner_mask=False)
        for level in levels:
            for line in contour.lines(level)[0]:
                if len(line) < 2:continue
                # a vertex lies on a grid line: between two columns of one row, or two rows of one column
                c = np.clip(np.floor(line[:, 0]).astype(int), 0, speed.shape[1]-2);r = np.clip(np.floor(line[:, 1]).astype(int), 0, len(heights)-2)
                u, w = line[:, 0]-c, line[:, 1]-r
                x = (1.-w)*((1.-u)*speed[r, c]+u*speed[r, c+1])+w*((1.-u)*speed[r+1, c]+u*speed[r+1, c+1])
                paths.append(dict(level=float(level), x=np.round(x, 3).tolist(), y=np.round((1.-w)*heights[r]+w*heights[r+1], 2).tolist()))
    return paths


def outline_of(rows, top):
    """The envelope's edge: up the slow side, across the top, down the fast side, back along the bottom."""
    def point(row, side):
        speed, label = (row.left, row.labels[0]) if side == 0 else (row.right, row.labels[1])
        return dict(speed_kmh=speed, turn_dps=row.height, altitude_m=row.height, limit_label=label,
                    alpha_deg=(row.samples.get(speed) or {}).get('alpha_deg'))
    slow, fast = [point(row, 0) for row in rows], [point(row, 1) for row in reversed(rows)]
    if rows[-1].height >= top-1e-6:fast[0] = dict(fast[0], limit_label=ALTITUDE_END)
    return slow+fast+[dict(slow[0], limit_label=ALTITUDE_END)]


def over_rev_lines(rows):
    """Where an engine goes past RPMMaxAllowed with rising speed in each row: the EM chart's over-rev line."""
    from overrev import report_table
    lines, line = [], None
    for row in rows:
        solved = [row.samples[v] for v in sorted(row.samples) if row.samples[v]['valid'] and row.samples[v].get('rpm_ratio') is not None]
        crossing = next((a['speed_kmh']+(1.-a['rpm_ratio'])*(b['speed_kmh']-a['speed_kmh'])/(b['rpm_ratio']-a['rpm_ratio'])
                         for a, b in zip(solved, solved[1:]) if a['rpm_ratio'] <= 1. < b['rpm_ratio']), None)
        if crossing is None:
            line = None;continue
        if line is None:
            line = dict(speed_kmh=[], turn_dps=[], stop_after_s=report_table());lines.append(line)
        line['speed_kmh'].append(round(crossing, 2));line['turn_dps'].append(row.height)
    return [l for l in lines if len(l['speed_kmh']) > 1]


def entry_label(entry, index):
    condition = entry['settings'];mode = flight_mode(condition)
    if not condition['structural_limits']:mode += ' · Flutter off'
    if condition.get('loadout'):
        from aircraft_loadouts import display_label
        mode += ' · '+display_label(entry['aircraft_id'], condition['ammunition_vehicle'], condition['loadout'], condition['stores_released'])
    return f"{AIRCRAFT[entry['aircraft_id']]['name']} · {index+1} · {mode}"


def calculate_entry(config, entry, index, check, report=None):
    started = time.monotonic()
    envelope = Envelope(config, entry, check)
    stopped = None
    try:rows = envelope.rows(report)
    except InterruptedError as error:
        stopped, rows = error, sorted(getattr(envelope, 'finished', ()), key=lambda r:r.height)
    item = dict(id=entry['id'], aircraft_id=entry['aircraft_id'], name=entry_label(entry, index), color=COLORS[index],
                settings=entry['settings'], chart_solver='altitude', engine=dict(policy=''), sustained=[], overrev_lines=[],
                contours=[], boundary=[], points=[], best_climb=dict(speed_kmh=[], altitude_m=[], ps_mps=[]),
                interpolation=dict(checked_sep_range_mps=[-2000., 2000.]), rows=len(rows), solves=envelope.solves,
                point_solves=dict(envelope.point_solves), more_thrust_rows=envelope.carried, discovered_rows=envelope.discovered)
    seeds = []
    if rows:
        solved = [s for row in rows for s in sorted(row.samples.values(), key=lambda s:s['speed_kmh']) if s['valid']]
        item.update(surface=surface_of(rows, JUMP_REACH*envelope.step, envelope.closest), boundary=outline_of(rows, envelope.top), overrev_lines=over_rev_lines(rows),
                    points=[dict({k:s.get(k) for k in ('speed_kmh', 'altitude_m', 'valid', 'ps_mps', 'alpha_deg', 'ias_kmh', 'mach')},
                                 turn_dps=s['altitude_m'], load_g=1., reasons=[]) for s in solved],
                    best_climb=dict(speed_kmh=[round(r.best_speed, 2) for r in rows], altitude_m=[r.height for r in rows],
                                    ps_mps=[round(r.best_sep, 3) for r in rows]))
        seeds = envelope.seeds(rows)
    item['elapsed_s'] = time.monotonic()-started
    if stopped is not None:
        # The rows solved so far, if they make a chart.
        stopped.unfinished = (item, seeds) if len(rows) > 1 else None
        raise stopped
    return item, seeds


def calculate(config, progress=None, cancelled=None):
    """(data, seeds by entry). Stopped: the error carries the entries finished so far and the rows of the current one, as .partial."""
    started = time.monotonic();entries = config['entries'];aircraft, seeds = [], {}

    def check():
        if cancelled and cancelled():raise InterruptedError('Calculation cancelled')
    data = dict(kind='altitude', settings=config, aircraft=aircraft, speeds_kmh=[], loads_g=[], assumptions=[], elapsed_s=0.,
                method='Native 1 g level-flight point solve on adaptive altitude rows')
    try:
        for index, entry in enumerate(entries):
            check()

            def report(fraction, index=index):
                if progress:progress(dict(done=index+min(fraction, .99), total=len(entries), phase=f"Entry {index+1} of {len(entries)}"))
            report(0.)
            item, seeds[entry['id']] = calculate_entry(config, entry, index, check, report)
            aircraft.append(item);data['elapsed_s'] = time.monotonic()-started
    except InterruptedError as error:
        if getattr(error, 'unfinished', None):
            aircraft.append(error.unfinished[0]);seeds[entries[len(aircraft)-1]['id']] = error.unfinished[1]
        data['elapsed_s'] = time.monotonic()-started;error.partial = (draw_contours(data), seeds)
        raise
    return draw_contours(data), seeds


def draw_contours(data):
    """The chart's levels, its own once all rows are in unless the page has set them, and their lines."""
    config = data['settings']
    levels = (contour_levels(data['aircraft']) if config.get('auto_contours') else None) or config['sep_contour_levels_mps']
    data['settings'] = dict(config, sep_contour_levels_mps=levels)
    for a in data['aircraft']:
        if 'surface' in a:a['contours'] = contours_of(a['surface'], levels)
    return data


def prepared(data, seeds, started_at):
    """The result record the job routes read (em_plot.prepare_exports' shape)."""
    chart = {k:v for k, v in data.items() if k != 'aircraft'}
    chart['aircraft'] = [{k:v for k, v in a.items() if k != 'surface'} for a in data['aircraft']]
    chart['calculation_elapsed_s'] = chart['elapsed_s'];chart['elapsed_s'] = time.monotonic()-started_at
    return dict(data=data, chart=chart, points=seeds, exports_ready=True)


def contour_paths(data, levels):
    return {a['id']:contours_of(a['surface'], levels) if 'surface' in a else [] for a in data['aircraft']}


def clicked_point(data, seeds, entry, speed, height):
    """The full EM point at a clicked speed and altitude, in the report format of an EM chart's clicked point, and its
    solver. Where the point solver finds no state, the chart's own way to it: level flight continued from the nearest
    regular state of the chart by the engine's level step, which takes the sliding state at a blade-polar jump (the
    point solver has none for a governing propeller)."""
    chosen = next(e for e in data['settings']['entries'] if e['id'] == entry)
    chart, _ = solver_at(chosen['aircraft_id'], condition_of(data['settings'], chosen), height)
    distance = lambda p:((p['speed_kmh']-speed)/speed)**2+((p['altitude_m']-height)/2000.)**2
    near = min(seeds, key=distance) if seeds else None
    point = settle(chart, chart.solve, speed, level_point(chart, chart.solve, speed, near))
    if (point is None or not point.get('valid')) and NATIVE_STEPS:
        stepped = stepped_point(chart, [p for p in seeds if not p.get('sliding')], speed, height, distance)
        if stepped is not None:point = stepped
    if point is None:
        # No candidate established: a gap at this state, not an absence proof.
        point = dict(speed_kmh=speed, load_g=1., status='unresolved', valid=False, chart_usable=False,
                     reasons=['stationary search unresolved'], search_unresolved='No level-flight state was established here')
    return point, chart


def stepped_point(chart, seeds, speed, height, distance, reach=5.):
    """Level flight at this speed continued from the nearest of these states in steps of at most `reach` km/h, or None."""
    import em_march_native
    engine = em_march_native.engine(chart)
    if engine is None or not hasattr(engine, 'level_step') or not seeds:return None
    solve = engine.get_point_solver().solve_row
    for near in sorted(seeds, key=distance)[:3]:
        start = level_point(chart, solve, near['speed_kmh'], near)
        if not start or not start.get('valid'):continue
        sid = next((i for i in range(len(engine.rows)-1, -1, -1) if engine.rows[i] is start), None)
        if sid is None:continue
        v, way = near['speed_kmh'], 1. if speed > near['speed_kmh'] else -1.
        while sid is not None and abs(speed-v) > 1e-9:
            v = speed if abs(speed-v) <= reach else v+way*reach
            out = engine.level_step(sid, v, STEP_NFEV, STEP_EVALS, False) or engine.level_step(sid, v, ROBUST_NFEV, ROBUST_EVALS)
            sid = out[0] if out and out[1] else None
        if sid is not None:
            try:return engine.state_row(sid)
            except RuntimeError:return None
    return None


def export_csv(data):
    out = io.StringIO();fields = ('aircraft', 'aircraft_id', 'aircraft_name', 'altitude_m', 'speed_kmh', 'ps_mps', 'alpha_deg', 'ias_kmh', 'mach')
    writer = csv.DictWriter(out, fieldnames=fields);writer.writeheader()
    for aircraft in data['aircraft']:
        for p in aircraft['points']:
            writer.writerow(dict({k:p.get(k, '') for k in fields}, aircraft=aircraft['id'], aircraft_id=aircraft['aircraft_id'], aircraft_name=aircraft['name']))
    return out.getvalue()
