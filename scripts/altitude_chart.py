"""The website's altitude envelope (Leo, 2026-10-30): SEP of 1 g level flight over true airspeed and altitude, for the
EM page's aircraft entries and with its settings. This replaces the earlier altitude plotter.

Every sample is the EM plotter's own point at 1 g, solved by the compiled point solver (em_march_native), on a solver
built per altitude as a march chart builds its own (soft admission, no sideslip recovery): about 1 ms a point. This
module only decides where to solve; it changes nothing in the solver.

Rows of constant altitude, each marched outward in speed from the best-SEP speed of the row beside it until level
flight ends, its two ends found by bisection and named by what rejected the state beyond them. Rows are halved where
the row between two others differs from what they predict, in SEP or in its ends, so supercharger gears and the corners
of the limits get rows of their own. Between rows the chart interpolates along lines of the same fraction of each
row's speed span, so contours run all the way to the edges of the envelope. A clicked point is solved afresh.

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
from em_solver import AIRCRAFT, settings
from overrev import rpm_ratio

ALTITUDES = (0., 18000.)    # the solver's own altitude range (em_solver.settings)
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
SPEED_END, ALTITUDE_END = 'End of the speed range', 'End of the altitude range'


def request(values):
    """The job's settings: the EM page's entries and plot settings, plus this chart's altitude range."""
    values = dict(values)
    low, high = values.pop('altitude_min_m', ALTITUDES[0]), values.pop('altitude_max_m', DEFAULT_TOP)
    choose = bool(values.pop('auto_contours', False))
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in (low, high)):
        raise ValueError('Altitude range must be numbers')
    if not ALTITUDES[0] <= low < high <= ALTITUDES[1] or high-low < 100.:
        raise ValueError('Altitude range must rise at least 100 m, within 0 to 18,000 m')
    if not values.get('entries'):raise ValueError('Add at least one aircraft')
    return dict(settings(values), kind='altitude', altitude_min_m=float(low), altitude_max_m=float(high), auto_contours=choose)


def solver_at(name, condition, height):
    """A march chart's solver at this altitude and its compiled point solve (the Python one if there is no compiled)."""
    import em_march_native
    from em_stationary_baseline import BaselineSolver
    chart = BaselineSolver(name, settings(dict(condition, altitude_m=float(height), sideslip_recovery=False)))
    chart.soft_admission = True
    em_march_native.attach(chart)
    engine = em_march_native.engine(chart)
    return chart, engine.get_point_solver().solve_row if engine is not None else chart.solve


def condition_of(config, entry):
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


class Row:
    """One altitude's solved speeds, its ends, and SEP along it."""
    def __init__(self, height, samples, left, right, labels, gaps, spacing, jumps):
        self.height, self.left, self.right, self.labels, self.gaps, self.jumps = height, left, right, labels, gaps, jumps
        self.samples = samples      # by speed, the rejected ones too
        # The curve runs through the ends and the states well inside them: the bisection's close states add only rounding.
        knots = [v for v, s in sorted(samples.items()) if s['valid'] and (v in (left, right) or left+spacing < v < right-spacing)]
        values = [samples[v]['ps_mps'] for v in knots]
        # Not a shape-preserving curve: those flatten the top of SEP to the best state solved, and the ceiling sits there.
        if len(knots) > 2:self.sep = Akima1DInterpolator(knots, values, method='makima')
        else:self.sep = lambda v, x=knots, y=values:np.interp(v, x, y)
        fine = np.linspace(left, right, max(2, int((right-left)/2.)+1))
        values = self.sep(fine);i = int(np.argmax(values))
        self.best_speed, self.best_sep = float(fine[i]), float(values[i])
        if 0 < i < len(fine)-1:
            # The top of the parabola through the three best, so the line does not step with the grid above.
            a, b, c = values[i-1:i+2];curve = a-2.*b+c
            if curve < 0.:
                self.best_speed += .5*(a-c)/curve*(fine[1]-fine[0]);self.best_sep = float(self.sep(self.best_speed))

    def at_fractions(self, fractions):
        speeds = np.clip(self.left+fractions*(self.right-self.left), self.left, self.right);values = np.array(self.sep(speeds), dtype=float)
        for low, high in self.gaps:values[(speeds > low) & (speeds < high)] = math.nan
        return values

    def by_a_jump(self, fractions, width):
        """Which of these fractions of the span lie within this width (km/h) of a jump in SEP."""
        speeds = self.left+fractions*(self.right-self.left);near = np.zeros(speeds.shape, dtype=bool)
        for jump in self.jumps:near |= np.abs(speeds-jump) < width
        return near


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

    def row(self, height, near, predicted=None):
        """The row at this altitude, or None if it holds no level flight. near: the row beside it, for seeds and where to
        start; predicted: where its two ends are expected."""
        chart, solve = solver_at(self.name, self.condition, height)
        self.check()
        grid, seen = self.speeds, {}

        def nearest_seed(v):
            for source in (seen, near.samples if near else {}):
                solved = [u for u, s in source.items() if s['valid']]
                if solved:return source[min(solved, key=lambda u:abs(u-v))]['seed']
            return None

        def store(v, p, unchecked):
            seen[v] = sample = dict(speed_kmh=v, altitude_m=height, valid=bool(p and (p.get('valid') or unchecked)), unchecked=unchecked,
                                    reasons=(p or {}).get('reasons') or ['no candidate'])
            if sample['valid']:
                sample.update(ps_mps=p['ps_mps'], alpha_deg=p.get('alpha_deg'), ias_kmh=p.get('ias_kmh'), mach=p.get('mach'), seed=seed_of(p))
                ratio = rpm_ratio(p)
                if ratio is not None:sample['rpm_ratio'] = ratio
            return sample

        def at(v, seed=None, full=False):
            """Inside the row (full False) a state failing only the vertical-step check counts as it stands."""
            if v in seen and (not full or not seen[v]['unchecked']):return seen[v]
            self.solves += 1
            p = level_point(chart, solve, v, seed or nearest_seed(v))
            if full and ends_upward(p):
                self.solves += 1;p = settle(chart, solve, v, p)
            return store(v, p, ends_upward(p) and not full)

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
                i += way;s = at(grid[i])
                if numerical(s) and near is not None:
                    # The row beside solved this speed: again from its state.
                    twin = min((t for u, t in near.samples.items() if t['valid'] and abs(u-grid[i]) <= self.step), key=lambda t:abs(t['speed_kmh']-grid[i]), default=None)
                    if twin:
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
                    s = at(v, seen[good]['seed'], full=True)
                    if s['valid']:good = v
                    else:rejected = [v]+rejected
                    if first:
                        # The neighbours' end is usually close: look just past it next.
                        first = False;w = v+(way if s['valid'] else -way)*2.*self.bracket
                        if min(good, rejected[0]) < w < max(good, rejected[0]):probes.append(w)
                # The bracket has closed on a state that only failed to solve: that is no limit, so look just beyond it.
                beyond = rejected[0]+way*2.*self.bracket
                if not numerical(seen[rejected[0]]) or not tries or len(rejected) < 2 or (rejected[1]-beyond)*way <= 0.:break
                tries -= 1
                if not at(beyond, seen[good]['seed'], full=True)['valid']:break
                good, rejected = beyond, [v for v in rejected if (v-beyond)*way > 0.]
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

    def mismatch(self, below, middle, above):
        """How far a row is from what the rows on either side predict: SEP (m/s, kinder where SEP is far from zero, as
        contours lie further apart there, and leaving out the jumps, which move with altitude), and its two ends (km/h,
        against their own tolerance)."""
        kept = [s for s in middle.samples.values() if s['valid'] and middle.left <= s['speed_kmh'] <= middle.right]
        fractions = np.array([(s['speed_kmh']-middle.left)/max(middle.right-middle.left, 1e-6) for s in kept])
        values = np.array([s['ps_mps'] for s in kept])
        errors = np.abs(.5*(below.at_fractions(fractions)+above.at_fractions(fractions))-values)/(1.+np.abs(values)/20.)
        errors = errors[np.isfinite(errors) & ~(below.by_a_jump(fractions, self.step) | middle.by_a_jump(fractions, self.step) | above.by_a_jump(fractions, self.step))]
        return max(float(errors.max()) if errors.size else 0.,
                   self.sep_tolerance*abs(.5*(below.left+above.left)-middle.left)/self.end_tolerance,
                   self.sep_tolerance*abs(.5*(below.right+above.right)-middle.right)/self.end_tolerance)

    def rows(self, report=None):
        heights = np.arange(self.bottom, self.top, self.spacing).tolist()+[self.top]
        rows, near, empty = [], None, None
        self.finished = rows        # what a stopped calculation keeps

        def ahead(height):
            if len(rows) < 2:return (near.left, near.right) if near is not None else None
            a, b = rows[-2], rows[-1];f = (height-b.height)/(b.height-a.height)
            return b.left+f*(b.left-a.left), b.right+f*(b.right-a.right)
        for height in heights:
            row = self.row(height, near, ahead(height))
            if report:report(.4*len(rows)/len(heights))
            if row is None:
                if near is None:continue
                empty = height;break
            rows.append(row);near = row
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
            if self.mismatch(below, middle, above) > self.sep_tolerance:
                pairs += [(below, middle), (middle, above)]
        rows.sort(key=lambda r:r.height)
        return rows


def surface_of(rows):
    """SEP on a grid of altitude rows by fraction of each row's speed span (0 at its slow end, 1 at its fast end)."""
    span = max(r.right-r.left for r in rows)
    fractions = np.linspace(0., 1., min(801, max(61, int(span/2.5)+1)))
    return dict(fraction=fractions, altitude_m=np.array([r.height for r in rows]), left_kmh=np.array([r.left for r in rows]),
                right_kmh=np.array([r.right for r in rows]), sep_mps=np.array([r.at_fractions(fractions) for r in rows]))


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
    heights = surface['altitude_m']
    if len(heights) < 2:return []
    z = np.ma.masked_invalid(surface['sep_mps'])
    if z.count() < 4 or not z.max() > z.min():return []
    contour = contourpy.contour_generator(x=surface['fraction'], y=heights, z=z, name='mpl2014', corner_mask=False)
    paths = []
    for level in levels:
        for segment in contour.lines(level)[0]:
            if len(segment) < 2:continue
            left, right = np.interp(segment[:, 1], heights, surface['left_kmh']), np.interp(segment[:, 1], heights, surface['right_kmh'])
            paths.append(dict(level=float(level), x=np.round(left+segment[:, 0]*(right-left), 3).tolist(), y=np.round(segment[:, 1], 2).tolist()))
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
                interpolation=dict(checked_sep_range_mps=[-2000., 2000.]), rows=len(rows), solves=envelope.solves)
    seeds = []
    if rows:
        solved = [s for row in rows for s in sorted(row.samples.values(), key=lambda s:s['speed_kmh']) if s['valid']]
        item.update(surface=surface_of(rows), boundary=outline_of(rows, envelope.top), overrev_lines=over_rev_lines(rows),
                    points=[dict({k:s.get(k) for k in ('speed_kmh', 'altitude_m', 'valid', 'ps_mps', 'alpha_deg', 'ias_kmh', 'mach')},
                                 turn_dps=s['altitude_m'], load_g=1., reasons=[]) for s in solved],
                    best_climb=dict(speed_kmh=[round(r.best_speed, 2) for r in rows], altitude_m=[r.height for r in rows],
                                    ps_mps=[round(r.best_sep, 3) for r in rows]))
        seeds = [dict(s['seed'], speed_kmh=s['speed_kmh'], altitude_m=s['altitude_m']) for s in solved]
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
    """The full EM point at a clicked speed and altitude, in the report format of an EM chart's clicked point, and its solver."""
    chosen = next(e for e in data['settings']['entries'] if e['id'] == entry)
    chart, _ = solver_at(chosen['aircraft_id'], condition_of(data['settings'], chosen), height)
    near = min(seeds, key=lambda p:((p['speed_kmh']-speed)/speed)**2+((p['altitude_m']-height)/2000.)**2) if seeds else None
    point = settle(chart, chart.solve, speed, level_point(chart, chart.solve, speed, near))
    if point is None:
        # No candidate established: a gap at this state, not an absence proof.
        point = dict(speed_kmh=speed, load_g=1., status='unresolved', valid=False, chart_usable=False,
                     reasons=['stationary search unresolved'], search_unresolved='No level-flight state was established here')
    return point, chart


def export_csv(data):
    out = io.StringIO();fields = ('aircraft', 'aircraft_id', 'aircraft_name', 'altitude_m', 'speed_kmh', 'ps_mps', 'alpha_deg', 'ias_kmh', 'mach')
    writer = csv.DictWriter(out, fieldnames=fields);writer.writeheader()
    for aircraft in data['aircraft']:
        for p in aircraft['points']:
            writer.writerow(dict({k:p.get(k, '') for k in fields}, aircraft=aircraft['id'], aircraft_id=aircraft['aircraft_id'], aircraft_name=aircraft['name']))
    return out.getvalue()
