"""Adaptive, checked surface over (altitude, true airspeed) for the Engine performance page's slice sliders.

The page draws curves on a 1 m / 1 km/h grid and must redraw them at once while a slider moves (Leo, 2026-11-18: "the
1/1/1 is the rule for the ui. in the back end you can do some high accuracy adaptive resolution"; "there should not be
any delay when i slide the slider"). Solving every grid point is exact but takes hours for a full range, so the surface
is built from solved rows only where the grid needs them:

  row      one altitude (a whole metre): solved states at whole km/h, split into pieces of one regime each (the caller's
           signature: supercharger gear and power-law branch, governor state, ...). Where neighbouring knots differ in
           regime or one is not a result, the interval is halved down to neighbouring whole km/h, so a piece ends
           exactly where its regime does. Inside a piece (smooth, one regime) an interval is halved until the solved
           midpoint is what the curve through the knots around it gives (`piece_value`), within the tolerance.
  surface  rows at whole metres. Between two rows of the same layout a curve is drawn by `ends` and `value`: each piece
           end follows the line through that end in the rows around (altitude as a function of speed, so an end that
           sweeps across the range near the altitude where its regime appears is followed too); a value is the curve
           through the rows around at the same speed (near a moving end, where only one side has the regime at that
           speed, continued from that side's rows). An interval is halved until the solved middle row is reproduced:
           every piece end within EDGE km/h, every value within the tolerance. Rows of different layouts are never
           blended: such an interval is halved down to neighbouring metres, with single solved points where one regime
           appears at an end of the range, else with rows solved only as far as their layout.

Nothing here knows the flight model: `solver_at(h)` gives f(x) -> None or (signature, values) for one altitude.
engine_performance.py supplies it; app/engine.js mirrors `ends`, `value` and `piece_value` to draw the curves.
"""
import heapq
from bisect import bisect_left, bisect_right

TOLERANCE = .0005   # of each plotted quantity's range over the surface (the page's sweep tolerance since 2026-10-29)
ROW_SHARE = .5      # a row's own share of the tolerance; the rest is for the blend between rows
EDGE = 1.           # km/h: how far a piece end between two rows may lie from the solved one
ROW_INTERVALS = 12  # a row's first intervals
MID_INTERVALS = 12  # the same for a row whose pieces the rows around predict (spans)
ROW_BUDGET = 600    # most solved points in one row beyond which only regime changes are still located
MAX_ROWS = 1500     # most rows; past it the remaining intervals keep their unchecked blend
SPREAD = .25        # a curve's outer knot lies at least this share of the interval away (else noise is amplified)
SOFT = 2            # km/h: how closely a row's knots close in on a corner the evaluator names (a result's third item)
CHECK = 1.          # the most a solved middle row may be off (in tolerances) for its interval to stand
SHARP = 2.          # a sharp bend: three neighbouring speeds whose middle one lies this many tolerances off the line of the others
STEP = 40.          # a step inside a regime: this many row tolerances within 2 km/h, 5 times the slope beside (`stepped`)


def through(xs, ys, x):
    """The polynomial through the points (2 to 4 of them) at x."""
    total = 0.
    for i, xi in enumerate(xs):
        term = ys[i]
        for j, xj in enumerate(xs):
            if j != i:term *= (x - xj) / (xi - xj)
        total += term
    return total


class Piece:
    __slots__ = ('sig', 'x', 'y', 'kept')

    def __init__(self, sig, count):
        self.sig, self.x, self.y = sig, [], [[] for _ in range(count)]
        self.kept = None   # in a finished row: {speed: every curve there}, as the checks of the rows around ask again


class Row:
    __slots__ = ('h', 'pieces', 'topo', 'solved', 'modes')

    def __init__(self, h, pieces, solved):
        self.h, self.pieces, self.solved = h, pieces, solved
        for p in pieces:p.kept = {}
        # How the interval above this row reads each piece near its (first, last) end: see `value`.
        self.modes = None
        # The layout two rows must share to be blended: each piece's regime and whether it continues the one before.
        self.topo = tuple((p.sig, i > 0 and p.x[0] == pieces[i - 1].x[-1] + 1) for i, p in enumerate(pieces))


def basis(xs, x):
    """The weights of the points' values in the polynomial through them at x (`through` for several curves at once)."""
    out = []
    for i, xi in enumerate(xs):
        w = 1.
        for j, xj in enumerate(xs):
            if j != i:w *= (x - xj) / (xi - xj)
        out.append(w)
    return out


def stencil(xs, x, extend=False):
    """The knots a piece's curves are read from at speed x, as (their indices, how the curve is taken from them): 'at'
    one knot, the 'chord' of two (past an end: `extend`), or the cubic 'through' the two knots around x and one more
    on each side (a parabola next to an end; an outer knot at least SPREAD of the interval away)."""
    n = len(xs)
    if n == 1:return (0,), 'at'
    if x <= xs[0] or x >= xs[-1]:
        if x <= xs[0]:
            if x == xs[0] or not extend:return (0,), 'at'
            i, j = 0, 1
            while j < n - 1 and xs[j] - xs[0] < 4:j += 1
        else:
            if x == xs[-1] or not extend:return (n - 1,), 'at'
            i, j = n - 2, n - 1
            while i > 0 and xs[-1] - xs[i] < 4:i -= 1
        return (i, j), 'chord'
    j = bisect_right(xs, x);i = j - 1
    if x == xs[i]:return (i,), 'at'
    reach = (xs[j] - xs[i]) * SPREAD
    a = i - 1
    while a >= 0 and xs[i] - xs[a] < reach:a -= 1
    b = j + 1
    while b < n and xs[b] - xs[j] < reach:b += 1
    return (a, i, j, b if b < n else -1), 'through'


def taken(xs, ys, x, found):
    """One curve's value from a `stencil`."""
    index, how = found
    if how == 'at':return ys[index[0]]
    if how == 'chord':
        i, j = index;y0, y1 = ys[i], ys[j]
        return None if y0 is None or y1 is None else y0 + (y1 - y0) * (x - xs[i]) / (xs[j] - xs[i])
    a, i, j, b = index;y0, y1 = ys[i], ys[j]
    if y0 is None or y1 is None:return None
    px, py = [xs[i], xs[j]], [y0, y1]
    if a >= 0 and ys[a] is not None:px.insert(0, xs[a]);py.insert(0, ys[a])
    if b >= 0 and ys[b] is not None:px.append(xs[b]);py.append(ys[b])
    return through(px, py, x)


def piece_value(p, k, x, extend=False):
    """The piece's curve k at speed x: the cubic through the two knots around x and one more on each side (a parabola
    next to an end; an outer knot at least SPREAD of the interval away). extend: continued straight past the piece's
    ends, along the chord of its last 4 km/h or more."""
    return taken(p.x, p.y[k], x, stencil(p.x, x, extend))


def piece_values(p, x, extend=False):
    """Every curve of the piece at speed x (`piece_value` for each, the knots looked up once)."""
    found = stencil(p.x, x, extend)
    index, how = found
    if how == 'at':return [ys[index[0]] for ys in p.y]
    if how == 'chord':return [taken(p.x, ys, x, found) for ys in p.y]
    xs = p.x;index = [k for k in index if k >= 0];weights = basis([xs[k] for k in index], x)
    out = []
    for ys in p.y:
        total = 0.
        for k, w in zip(index, weights):
            y = ys[k]
            if y is None:
                total = taken(xs, ys, x, found)
                break
            total += w * y
        out.append(total)
    return out


class RowBuilder:
    """The row at altitude h over the whole speeds lo..hi, in two stages: `layout` (the piece ends located, nothing
    refined: enough to tell which regimes the row has and where) and `row` (the knots refined too). f(x, again=False, start=None)
    -> None (no result) or (signature, values); again: solve a speed that gave no result once more if a result nearer
    to it than before can now start it; start: a solved speed whose state is continued to x, which gives a result only
    where that state is preferred to what x has (`prefer`). hints: speeds to solve first, in this order, where piece ends are expected
    (from inside each piece outward): an end found there is not searched for by halving, which would solve several
    states that are not results (the costly ones in level flight). spans: the (first, last) speeds the pieces are
    expected to cover; the row's first knots outside them are left out for the same reason. allow(values) -> None or
    what each curve of a solved point may be off besides its tolerance (a quotient of two curves where its divisor is
    next to nothing: the point solver's own accuracy shows there, and no knot would ever be enough)."""

    def __init__(self, f, h, lo, hi, scales, hints=(), spans=None, intervals=ROW_INTERVALS, budget=ROW_BUDGET, allow=None):
        self.f, self.h, self.lo, self.hi, self.scales, self.budget, self.vals = f, h, lo, hi, scales, budget, {}
        self.allow = allow
        self.crossed = set()
        for x in hints:
            if lo <= x <= hi:self.at(x)
        n = max(1, min(intervals if spans is None else min(intervals, MID_INTERVALS), hi - lo))
        knots = {lo + round((hi - lo) * i / n) for i in range(n + 1)}
        if spans is not None:
            # An expected end that is not there (the speed past it is a result of the same kind as the one before it:
            # the piece goes on): the knots beyond it stay in, or the row would end where it was expected to.
            def same(a, b):
                ra, rb = self.vals.get(a), self.vals.get(b)
                return ra is not None and rb is not None and ra[0] == rb[0]
            grown = []
            for first, last in spans:
                if same(round(first) - 1, round(first)):first = lo
                if same(round(last), round(last) + 1):last = hi
                grown.append((first, last))
            knots = {x for x in knots if any(first <= x <= last for first, last in grown)}
        knots = sorted(knots | set(self.vals))
        for a, b in zip(knots, knots[1:]):self.locate(a, b)
        self.creep()

    def at(self, x):
        if x not in self.vals:self.vals[x] = self.f(x)
        return self.vals[x]

    def locate(self, a, b):
        """Halve [a, b] wherever its ends differ in kind, down to neighbouring speeds."""
        ra, rb = self.at(a), self.at(b)
        if b - a <= 1 or (None if ra is None else ra[0]) == (None if rb is None else rb[0]):return
        m = (a + b) // 2
        self.at(m);self.locate(a, m);self.locate(m, b)

    def creep(self):
        """A speed without a result next to one with a result is solved again from that neighbour, and so on outward
        while that gives results: whether a state exists must not depend on the order the row was solved in (a solve
        started far away can fail where the state next to it continues)."""
        vals, f, lo, hi = self.vals, self.f, self.lo, self.hi
        for x in sorted(x for x in vals if vals[x] is not None):
            for step in (-1, 1):
                u = x + step
                if not lo <= u <= hi or u not in vals or vals[u] is not None:continue
                r = f(u, True)
                while r is not None:
                    vals[u] = r
                    u += step
                    if not lo <= u <= hi or vals.get(u) is not None:break
                    r = vals[u] = f(u, True) if u in vals else f(u)

    def layout(self):
        return Row(self.h, assemble(self.vals, len(self.scales), self.scales, self.allow), sum(r is not None for r in self.vals.values()))

    def prefer(self):
        """Where the row's curves step between two neighbouring speeds, the state of each side is continued across the
        step (f(x, start=u)), 1 km/h at a time, for as long as it is the preferred one of two states that coexist
        there (the evaluator's rule: more thrust). Two propeller states coexist over a band of speeds (blades stalled
        or not), and which one a solve finds depends on where it started: after this the step lies where the preferred
        state ends, however the row was solved. Returns whether any speed changed."""
        vals, f, lo, hi, changed = self.vals, self.f, self.lo, self.hi, False
        pieces = assemble(vals, len(self.scales), self.scales, self.allow)
        for a, b in zip(pieces, pieces[1:]):
            left, right = a.x[-1], b.x[0]
            if right != left + 1 or continues(a, b, self.scales):continue
            key = (left, vals[left][1], vals[right][1])
            if key in self.crossed:continue
            self.crossed.add(key)
            for u, step in ((left, 1), (right, -1)):
                w = u + step
                while lo <= w <= hi:
                    r = f(w, start=u)
                    if r is None:break
                    vals[w] = r;changed = True
                    u, w = w, w + step
        return changed

    def row(self):
        while True:
            self.refine()
            if not self.prefer():break
        return self.layout()

    def refine(self):
        vals, scales, accepted = self.vals, self.scales, set()
        while True:
            pieces = assemble(vals, len(scales))
            heap = [(-(b - a), a, b, p) for p in pieces for a, b in zip(p.x, p.x[1:]) if b - a > 1 and (a, b) not in accepted]
            if not heap or len(vals) >= self.budget:break
            heapq.heapify(heap)
            while heap and len(vals) < self.budget:
                _, a, b, p = heapq.heappop(heap)
                m = (a + b) // 2
                expected = piece_values(p, m)
                r = self.at(m)
                if r is None or r[0] != p.sig:
                    # Another regime (or no result) inside what looked like one piece: locate its ends and start over.
                    self.locate(a, m);self.locate(m, b);self.creep()
                    break
                extra = self.allow(r[1]) if self.allow is not None else None
                ok = all((ye is None and ym is None) or (ye is not None and ym is not None and abs(ym - ye) <= s + (extra[k] if extra else 0.))
                         for k, (s, ye, ym) in enumerate(zip(scales, expected, r[1])))
                i = bisect_left(p.x, m)
                p.x.insert(i, m)
                for ys, y in zip(p.y, r[1]):ys.insert(i, y)
                for c, d in ((a, m), (m, b)):
                    # A half stands when the midpoint was right, unless the evaluator says its two ends lie either
                    # side of a corner (their third items differ): a corner away from the midpoint leaves the
                    # midpoint right and the curve between the knots wrong, so the knots close in on it.
                    if ok and (d - c <= SOFT or len(vals[c]) < 3 or len(vals[d]) < 3 or vals[c][2] == vals[d][2]):accepted.add((c, d))
                    elif d - c > 1:heapq.heappush(heap, (-(d - c), c, d, p))


def assemble(vals, count, scales=None, allow=None):
    """The pieces of a row's solved speeds: runs of one signature. scales (a row's tolerances): a run is also cut where
    its curves step between two neighbouring speeds (`stepped`): a step the signature does not know of (a propeller
    shaft passing a jump of its blade polar: the B-17's thrust steps by hundreds of kgf along such a line), which the
    rows around then follow as they follow a regime's end."""
    pieces, current = [], None
    for x in sorted(vals):
        r = vals[x]
        if r is None:
            current = None
            continue
        if current is None or current.sig != r[0]:
            current = Piece(r[0], count);pieces.append(current)
        current.x.append(x)
        for ys, y in zip(current.y, r[1]):ys.append(y)
    if scales is None:return pieces
    out = []
    for p in pieces:
        start = 0
        for j in stepped(p, scales, allow) + [len(p.x) - 1]:
            part = Piece(p.sig, count)
            part.x = p.x[start:j + 1];part.y = [ys[start:j + 1] for ys in p.y]
            out.append(part);start = j + 1
    return out


def stepped(p, scales, allow=None):
    """The knots of a piece after which its curves step. Over two neighbouring 1 km/h intervals a curve changes by
    more than STEP tolerances and by more than 5 times what it changes per km/h just before and just after them; the
    cut is at the larger of the two changes. Two intervals, because a change narrower than a km/h falls into one
    interval or onto the speed between two, depending on where between whole speeds it lies, which moves with every
    metre of altitude: looked for in one interval alone, the same step was found in one row and missed in the next."""
    xs, n, found = p.x, len(p.x), set()
    for j in range(1, n - 1):
        if xs[j] - xs[j - 1] != 1 or xs[j + 1] - xs[j] != 1:continue
        extra = allow([ys[j] for ys in p.y]) if allow is not None else None
        for k, (ys, s) in enumerate(zip(p.y, scales)):
            a, b, c = ys[j - 1], ys[j], ys[j + 1]
            if a is None or b is None or c is None or s <= 0. or abs(c - a) <= STEP * (s + (extra[k] if extra else 0.)):continue
            beside = []
            if j >= 2 and xs[j - 1] - xs[j - 2] <= 2 and ys[j - 2] is not None:beside.append(abs(a - ys[j - 2]) / (xs[j - 1] - xs[j - 2]))
            if j + 2 < n and xs[j + 2] - xs[j + 1] <= 2 and ys[j + 2] is not None:beside.append(abs(ys[j + 2] - c) / (xs[j + 2] - xs[j + 1]))
            if beside and abs(c - a) > 5. * max(beside):
                found.add(j - 1 if abs(b - a) >= abs(c - b) else j)
                break
    return sorted(found)


def blend(ya, yb, w):
    return None if ya is None or yb is None else ya + (yb - ya) * w


def end(rows, i, h, pi, last):
    """One end (the last speed, or the first) of piece pi between rows[i] and rows[i + 1] (same layout) at altitude h,
    a real number. Where the end moves, it follows the curve through this end in the rows around, taken as altitude
    over speed (an end leaving the edge of the range runs along the altitude axis there)."""
    A, B = rows[i], rows[i + 1];at = -1 if last else 0
    xa, xb = A.pieces[pi].x[at], B.pieces[pi].x[at]
    if xa == xb:return float(xa)
    if abs(xb - xa) < 3:return xa + (xb - xa) * (h - A.h) / (B.h - A.h)
    way, reach, topo = (1 if xb > xa else -1), abs(xb - xa) * SPREAD, A.topo
    px, ph = [xa, xb], [A.h, B.h]
    r = i - 1
    while r >= 0 and rows[r].topo == topo:
        d = (xa - rows[r].pieces[pi].x[at]) * way
        if d <= 0:break
        if d >= reach:
            px.insert(0, rows[r].pieces[pi].x[at]);ph.insert(0, rows[r].h)
            break
        r -= 1
    r = i + 2
    while r < len(rows) and rows[r].topo == topo:
        d = (rows[r].pieces[pi].x[at] - xb) * way
        if d <= 0:break
        if d >= reach:
            px.append(rows[r].pieces[pi].x[at]);ph.append(rows[r].h)
            break
        r += 1
    if len(px) == 2:return xa + (xb - xa) * (h - A.h) / (B.h - A.h)
    lo, hi = float(xa), float(xb)
    for _ in range(40):
        mid = .5 * (lo + hi)
        if through(px, ph, mid) < h:lo = mid
        else:hi = mid
    return .5 * (lo + hi)


def ends(rows, i, h, pi):
    """Piece pi between rows[i] and rows[i + 1] at altitude h: its first and last speed (real numbers). A piece that
    continues the one before starts 1 km/h after that one's end, as in the rows."""
    first = end(rows, i, h, pi - 1, True) + 1. if rows[i].topo[pi][1] else end(rows, i, h, pi, False)
    return first, end(rows, i, h, pi, True)


def reading(rows, i, h, pi, x, first, last, modes=None):
    """Which rows give piece pi's curves at altitude h (between rows[i] and rows[i + 1], same layout) and speed x, and
    where each is read: ([(row, speed), ...] for the curve over altitude through them, the two rows' own places or
    None), or (None, (piece, speed, extend) of each of the two rows) where the curves are taken straight between the
    two rows. first, last: the piece's ends at h (`ends`).

    The rows are the two around h and one more on each side (at least SPREAD of the interval away) that have this
    piece there; where only one of the two has it, next to a moving end, that side's rows (two or three).

    Each row is read at the same speed x, or, near an end whose mode is 1, at the same distance from that end (fading
    back to the same speed over twice the distance the end moves between the two rows). Which reading suits an end is
    not known beforehand: a curve shaped by its distance from where its regime ends (SEP next to the stall, the
    supercharger's power law next to its flat-rated pressure) wants the second, a regime that merely appears there the
    first. `reproduced` takes for each end the reading that gives the solved middle row."""
    A, B = rows[i], rows[i + 1];a, b = A.pieces[pi], B.pieces[pi];gap, topo = B.h - A.h, A.topo
    if modes is None:modes = A.modes
    mode = modes[pi] if modes else (0, 0)
    fade_first = 2. * abs(b.x[0] - a.x[0]) if mode[0] else 0.
    fade_last = 2. * abs(b.x[-1] - a.x[-1]) if mode[1] else 0.
    def place(r):
        """Where row r is read, or None when its piece does not reach there."""
        p = r.pieces[pi];at = x
        if fade_last > 0. and last - x < fade_last:at += (p.x[-1] - last) * (1. - max(last - x, 0.) / fade_last)
        if fade_first > 0. and x - first < fade_first:at += (p.x[0] - first) * (1. - max(x - first, 0.) / fade_first)
        if at < p.x[0] - .5 or at > p.x[-1] + .5:return None
        return min(max(at, p.x[0]), p.x[-1])
    def beyond(r, step, origin, reach):
        """The first row from index r on (in direction step) of this layout at least `reach` from altitude `origin`
        with its place there: (index, place), or (-1, None) when the layout or the piece ends first."""
        while 0 <= r < len(rows) and rows[r].topo == topo:
            at = place(rows[r])
            if at is None:break
            if abs(rows[r].h - origin) >= reach:return r, at
            r += step
        return -1, None
    at_a, at_b = place(A), place(B)
    if at_a is not None and at_b is not None:
        nodes = [(A, at_a), (B, at_b)]
        r, at = beyond(i - 1, -1, A.h, gap * SPREAD)
        if r >= 0:nodes.insert(0, (rows[r], at))
        r, at = beyond(i + 2, 1, B.h, gap * SPREAD)
        if r >= 0:nodes.append((rows[r], at))
        return nodes, ((a, at_a, False), (b, at_b, False))
    if at_b is not None:
        r, at = beyond(i + 2, 1, B.h, gap * SPREAD)
        if r < 0:return None, ((a, x, True), (b, at_b, False))
        nodes = [(B, at_b), (rows[r], at)]
        r, at = beyond(r + 1, 1, rows[r].h, (rows[r].h - B.h) * SPREAD)
        if r >= 0:nodes.append((rows[r], at))
        return nodes, None
    if at_a is not None:
        r, at = beyond(i - 1, -1, A.h, gap * SPREAD)
        if r < 0:return None, ((a, at_a, False), (b, x, True))
        nodes = [(rows[r], at), (A, at_a)]
        r, at = beyond(r - 1, -1, rows[r].h, (A.h - rows[r].h) * SPREAD)
        if r >= 0:nodes.insert(0, (rows[r], at))
        return nodes, None
    return None, ((a, x, True), (b, x, True))


def value(rows, i, h, pi, k, x, first, last, modes=None):
    """Curve k of piece pi at altitude h (between rows[i] and rows[i + 1], same layout) and speed x: see `reading`."""
    A, B = rows[i], rows[i + 1]
    nodes, two = reading(rows, i, h, pi, x, first, last, modes)
    if nodes is not None:
        ys = [piece_value(r.pieces[pi], k, at) for r, at in nodes]
        if not any(y is None for y in ys):return through([r.h for r, _ in nodes], ys, h)
        if two is None:return None
    (pa, xa, ea), (pb, xb, eb) = two
    return blend(piece_value(pa, k, xa, ea), piece_value(pb, k, xb, eb), (h - A.h) / (B.h - A.h))


def zone(rows, i, h, pi, first, last, mode):
    """The whole speeds lo..hi at which `reading` gives piece pi's curves between rows[i] and rows[i + 1] from the
    same rows, each read at the speed itself (no end's reading applies there and every row has the piece), with those
    rows' pieces and their weights at altitude h: (lo, hi, pieces, weights). Most knots of a row check lie there."""
    A, B = rows[i], rows[i + 1];a, b = A.pieces[pi], B.pieces[pi];gap, topo = B.h - A.h, A.topo
    lo, hi = max(a.x[0], b.x[0]), min(a.x[-1], b.x[-1])
    if mode[0]:lo = max(lo, first + 2. * abs(b.x[0] - a.x[0]))
    if mode[1]:hi = min(hi, last - 2. * abs(b.x[-1] - a.x[-1]))
    nodes, reach = [A, B], gap * SPREAD
    r = i - 1
    while r >= 0 and rows[r].topo == topo:
        p = rows[r].pieces[pi];lo, hi = max(lo, p.x[0]), min(hi, p.x[-1])
        if A.h - rows[r].h >= reach:
            nodes.insert(0, rows[r])
            break
        r -= 1
    r = i + 2
    while r < len(rows) and rows[r].topo == topo:
        p = rows[r].pieces[pi];lo, hi = max(lo, p.x[0]), min(hi, p.x[-1])
        if rows[r].h - B.h >= reach:
            nodes.append(rows[r])
            break
        r += 1
    return lo, hi, [r.pieces[pi] for r in nodes], basis([r.h for r in nodes], h)


def values(rows, i, h, pi, x, first, last, modes=None, zones=None):
    """Every curve of piece pi there (`value` for each, the rows and knots looked up once). zones: a dict kept over
    the calls for one pair of rows and altitude, for `zone`."""
    A, B = rows[i], rows[i + 1]
    if zones is not None:
        mode = modes if modes is not None else A.modes
        mode = tuple(mode[pi]) if mode else (0, 0)
        z = zones.get((pi, mode))
        if z is None:z = zones[pi, mode] = zone(rows, i, h, pi, first, last, mode)
        if z[0] <= x <= z[1]:
            columns = []
            for p in z[2]:
                c = p.kept.get(x)
                if c is None:c = p.kept[x] = piece_values(p, x)
                columns.append(c)
            out, weights = [], z[3]
            for k in range(len(columns[0])):
                total = 0.
                for w, c in zip(weights, columns):
                    y = c[k]
                    if y is None:break
                    total += w * y
                else:
                    out.append(total)
                    continue
                break
            else:return out
    nodes, two = reading(rows, i, h, pi, x, first, last, modes)
    count = len(A.pieces[pi].y)
    if nodes is not None:
        columns = []
        for r, at in nodes:
            p = r.pieces[pi];kept = p.kept
            if kept is None:columns.append(piece_values(p, at))
            else:
                if at not in kept:kept[at] = piece_values(p, at)
                columns.append(kept[at])
        weights = basis([r.h for r, _ in nodes], h)
    out = []
    for k in range(count):
        if nodes is not None:
            ys = [c[k] for c in columns]
            if None not in ys:
                total = 0.
                for w, y in zip(weights, ys):total += w * y
                out.append(total)
                continue
            if two is None:
                out.append(None)
                continue
        (pa, xa, ea), (pb, xb, eb) = two
        out.append(blend(piece_value(pa, k, xa, ea), piece_value(pb, k, xb, eb), (h - A.h) / (B.h - A.h)))
    return out


def continues(left, right, scales):
    """Whether the curves run on across the boundary between two neighbouring pieces (a corner, not a step): the
    right piece starts where the left one's last chord leads, within twice the tolerance."""
    for k, s in enumerate(scales):
        ya, yb = left.y[k][-1], right.y[k][0]
        if ya is None or yb is None:
            if ya is not yb:return False
            continue
        slope = 0.
        if len(left.x) > 1 and left.y[k][-2] is not None:slope = (ya - left.y[k][-2]) / (left.x[-1] - left.x[-2])
        if abs(yb - ya - slope * (right.x[0] - left.x[-1])) > 2. * s and abs(yb - ya) > 2. * s:return False
    return True


def reproduced(rows, i, M, scales, soft=None, allow=None):
    """Whether the solved row M is what rows[i] and rows[i + 1] give at its altitude: (ok, (worst value error over its
    scale, curve, speed), worst end error or None for another layout, the reading of each piece end that gives M
    best). An end counts where a curve stops or steps; where the curves only turn a corner (`continues`) its place
    matters through the values alone, so M's knots are compared with whichever piece is drawn at their speed. soft: a
    speed up to which the first piece's start is not held to EDGE (where results begin there depends on how far the
    solver gets, not on the aircraft).

    Where a curve stops (next to states that are not results) or steps, its end may lie EDGE km/h off, so its values within
    2 km/h of that end may be off by what the curve changes over EDGE there: level-flight SEP falls steeply at the
    stall, and its very first value depends on where within a km/h the stall lies, which no row spacing settles.
    The same holds within 2 km/h of a sharp bend inside a piece (`bends`): a propeller's thrust turns upward like a
    square root where its blades leave the stall, by tens of tolerances within a km/h, and where within a km/h that
    happens moves with every metre of altitude; such a bend is placed within EDGE km/h like an end."""
    A, B = rows[i], rows[i + 1]
    worst, edge = (0., None, None), 0.
    if A.topo != B.topo or M.topo != A.topo:return False, worst, None, None
    ok, modes, zones = True, [], {}
    spans = [ends(rows, i, M.h, pi) for pi in range(len(M.pieces))]
    corner = [pi > 0 and M.topo[pi][1] and continues(M.pieces[pi - 1], M.pieces[pi], scales) for pi in range(len(M.pieces))]
    def error(pi, y, x, first, last, trial, slack=None):
        """The largest error over its scale of piece pi's curves at speed x against the solved values y (inf where
        one of the two has no value); slack: what each curve may be off besides."""
        out = (0., None, None)
        drawn = values(rows, i, M.h, pi, x, first, last, trial, zones)
        if allow is not None:
            extra = allow(y)
            if extra is not None:slack = extra if slack is None else [u + w for u, w in zip(slack, extra)]
        for k, s in enumerate(scales):
            ym, yp = y[k], drawn[k]
            if ym is None and yp is None:continue
            if ym is None or yp is None:return (float('inf'), k, x)
            off = abs(yp - ym)
            if slack is not None:off = max(0., off - slack[k])
            if s > 0. and off / s > out[0]:out = (off / s, k, x)
        return out
    def steepness(m, at):
        """What each curve of piece m changes over EDGE km/h at its first (at 0) or last (at -1) knot."""
        if len(m.x) < 2:return [0.] * len(scales)
        a, b = (0, 1) if at == 0 else (-2, -1)
        return [0. if ys[a] is None or ys[b] is None else EDGE * abs(ys[b] - ys[a]) / (m.x[b] - m.x[a]) for ys in m.y]
    def bends(m):
        """For each knot of piece m, what its curves may be off there because the piece bends sharply within 2 km/h
        (None elsewhere): the most the curve changes over EDGE km/h next to the knot."""
        xs, n, marks = m.x, len(m.x), []
        for j in range(1, n - 1):
            if xs[j] - xs[j - 1] != 1 or xs[j + 1] - xs[j] != 1:continue
            for ys, sc in zip(m.y, scales):
                a, b, c = ys[j - 1], ys[j], ys[j + 1]
                if a is not None and b is not None and c is not None and sc > 0. and abs(a - b - b + c) > SHARP * sc:
                    marks.append(j)
                    break
        if not marks:return None
        out = [None] * n
        for j in marks:
            lo, hi = j, j
            while lo > 0 and xs[j] - xs[lo - 1] <= 2:lo -= 1
            while hi < n - 1 and xs[hi + 1] - xs[j] <= 2:hi += 1
            for q in range(lo, hi + 1):
                if out[q] is not None:continue
                slack = []
                for ys in m.y:
                    most = 0.
                    for r in (q - 1, q + 1):
                        if 0 <= r < n and ys[r] is not None and ys[q] is not None:
                            most = max(most, EDGE * abs(ys[r] - ys[q]) / abs(xs[r] - xs[q]))
                    slack.append(most)
                out[q] = slack
        return out
    # The ends next to states that are not results: (piece, first or last) -> the slack of the values near them.
    loose = {}
    for pi, m in enumerate(M.pieces):
        if (M.topo[pi][1] and not corner[pi]) or \
                (not M.topo[pi][1] and (pi > 0 or A.pieces[0].x[0] != B.pieces[0].x[0] or m.x[0] != A.pieces[0].x[0])):
            loose[pi, 0] = steepness(m, 0)
        if (pi + 1 == len(M.pieces) and (A.pieces[pi].x[-1] != B.pieces[pi].x[-1] or m.x[-1] != A.pieces[pi].x[-1])) or \
                (pi + 1 < len(M.pieces) and not (M.topo[pi + 1][1] and corner[pi + 1])):
            loose[pi, -1] = steepness(m, -1)
    for pi, m in enumerate(M.pieces):
        first, last = spans[pi]
        # A step inside a regime (the piece before has the same signature) may lie 2 EDGE off: the speed it falls on
        # is solved from one side or the other, so the rows themselves place it within 1 km/h only.
        if not corner[pi] and not (pi == 0 and soft is not None and max(first, m.x[0]) <= soft):
            edge = max(edge, abs(first - m.x[0]) / (2. if M.topo[pi][1] and M.pieces[pi - 1].sig == m.sig else 1.))
        if not (pi + 1 < len(M.pieces) and corner[pi + 1]):
            edge = max(edge, abs(last - m.x[-1]) / (2. if pi + 1 < len(M.pieces) and M.topo[pi + 1][1] and M.pieces[pi + 1].sig == m.sig else 1.))
        a, b = A.pieces[pi], B.pieces[pi]
        inside = [j for j, x in enumerate(m.x) if first - .5 <= x < last + .5]
        # For each end that moves between the two rows, the reading that gives M's knots near it best.
        modes.append([0, 0])
        for e, at in ((0, 0), (1, -1)):
            move = abs(b.x[at] - a.x[at])
            if not move:continue
            near = [j for j in inside if abs(m.x[j] - m.x[at]) <= max(4., 3. * move)]
            trial = [list(t) for t in modes] + [[0, 0]] * (len(M.pieces) - len(modes))
            errors = []
            for choice in (0, 1):
                trial[pi][e] = choice
                errors.append(max((error(pi, [ys[j] for ys in m.y], m.x[j], first, last, trial)[0] for j in near), default=0.))
            modes[pi][e] = int(errors[1] < errors[0])
    for pi, m in enumerate(M.pieces):
        bent = bends(m)
        # At the middle row's own solved speeds: its values there are exact.
        for j, x in enumerate(m.x):
            drawn = next((q for q in (pi, pi - 1, pi + 1) if 0 <= q < len(spans) and spans[q][0] - .5 <= x < spans[q][1] + .5), None)
            if drawn is None:continue
            # A knot drawn in the neighbouring piece lies next to an end placed a fraction of a km/h off: compared
            # across a corner, left to the end's own check across a step.
            if drawn != pi and not corner[max(pi, drawn)]:continue
            slack = loose.get((pi, 0)) if x - m.x[0] <= 2 else None
            if slack is None and m.x[-1] - x <= 2:slack = loose.get((pi, -1))
            if bent is not None and bent[j] is not None:slack = bent[j] if slack is None else [max(u, w) for u, w in zip(slack, bent[j])]
            found = error(drawn, [ys[j] for ys in m.y], x, *spans[drawn], modes, slack)
            if found[0] == float('inf'):ok = False
            elif found[0] > worst[0]:worst = found
    return ok and edge <= EDGE and worst[0] <= CHECK, worst, edge, modes


def appearing(A, B):
    """Where rows A and B differ by one piece at an end of the range: (speed to probe, the regime there in the row that
    has the piece, which row has it), else None."""
    for X, Y, has in ((A, B, 'upper'), (B, A, 'lower')):
        if len(Y.topo) != len(X.topo) + 1 or not X.topo:continue
        sx, sy = [t[0] for t in X.topo], [t[0] for t in Y.topo]
        if sy[1:] == sx and Y.topo[1][1]:return Y.pieces[0].x[0], sy[0], has
        if sy[:-1] == sx and Y.topo[-1][1]:return Y.pieces[-1].x[-1], sy[-1], has
    return None


def build_surface(solver_at, h_lo, h_hi, x_lo, x_hi, scales, base=None, max_rows=MAX_ROWS, cancelled=None, log=None, soft=None,
                  allow=None, also=()):
    """Rows over the whole metres h_lo..h_hi and speeds x_lo..x_hi, halved until every solved middle row is reproduced.
    solver_at(h) -> f(x) for that altitude; scales: the tolerance of each curve; soft: see `reproduced`; allow: see
    RowBuilder; also: altitudes to have rows at from the start (the two metres either side of a change of layout the
    caller knows of, which then is not searched for). Returns (rows, stats)."""
    row_scales = [s * ROW_SHARE for s in scales]
    def builder(h, hints=(), spans=None):return RowBuilder(solver_at(h), h, x_lo, x_hi, row_scales, hints, spans, allow=allow)
    def row_at(h, hints=(), spans=None):return builder(h, hints, spans).row()
    def expected(i, h):
        """Where the rows around altitude h put its pieces: (the speeds either side of each piece end, from inside the
        piece outward; each piece's first and last speed), or ((), None) between rows of different layouts."""
        if rows[i].topo != rows[i + 1].topo:return (), None
        hints, spans = [], []
        for pi in range(len(rows[i].pieces)):
            first, last = ends(rows, i, h, pi)
            hints += [round(first) + 1, round(first), round(first) - 1, round(last) - 1, round(last), round(last) + 1]
            spans.append((first, last))
        return hints, spans
    span = h_hi - h_lo
    n = max(1, min(base or max(4, min(12, -(-span // 1000))), span)) if span else 0
    hs = sorted({h_lo + round(span * i / n) for i in range(n + 1)} | {h for h in also if h_lo <= h <= h_hi}) if n else [h_lo]
    rows = [row_at(h) for h in hs]
    # Intervals between rows of different layouts first (a regime appears there, often with a step in the curves, and
    # locating it takes few solves), then the widest: what the row or time limit leaves open is then the least harm.
    def urgency(a, b):
        i = bisect_left(hs, a)
        return (rows[i].topo == rows[i + 1].topo, -(b - a), a, b)
    heap = [urgency(a, b) for a, b in zip(hs, hs[1:]) if b - a > 1]
    heapq.heapify(heap)
    stats = dict(layout_rows=0, value_rows=0, edge_rows=0, probes=0, layouts=0, unchecked=0)
    probed = set()
    def insert(row):
        i = bisect_left(hs, row.h)
        rows.insert(i, row);hs.insert(i, row.h)
    def push(a, b):
        if b - a > 1:heapq.heappush(heap, urgency(a, b))
    while heap:
        if len(rows) >= max_rows or (cancelled is not None and cancelled()):
            stats['unchecked'] = len(heap)
            break
        _, _, ha, hb = heapq.heappop(heap)
        i = bisect_left(hs, ha)
        A, B = rows[i], rows[i + 1]
        spot = appearing(A, B) if A.topo != B.topo and (ha, hb) not in probed else None
        if spot is not None:
            # One regime appears at an end of the range between these rows: find the altitude with single solved
            # points there, then solve only the two rows either side of it.
            x, sig, has = spot
            lo, hi = ha, hb
            while hi - lo > 1:
                mid = (lo + hi) // 2
                r = solver_at(mid)(x);stats['probes'] += 1
                if (r is not None and r[0] == sig) == (has == 'upper'):hi = mid
                else:lo = mid
            for h in (lo, hi):
                if h not in (ha, hb):insert(row_at(h))
            for a, b in ((ha, lo), (hi, hb)):
                probed.add((a, b));push(a, b)
            continue
        if A.topo != B.topo:
            # Rows of different layouts: halve the interval with rows solved only as far as their layout, down to
            # neighbouring metres; a third layout on the way is kept as a row and its two sides go back in the queue.
            around = [x for r in (A, B) for p in r.pieces for e in (p.x[0], p.x[-1]) for x in (e, e - 1, e + 1)]
            reach = [(min(p.x[0] for r in (A, B) for p in r.pieces), max(p.x[-1] for r in (A, B) for p in r.pieces))] \
                if A.pieces and B.pieces else None
            lo, hi, below, above, third = ha, hb, None, None, None
            steps = any(now[1] and now[0] == before[0] for r in (A, B) for before, now in zip(r.topo, r.topo[1:]))
            while hi - lo > 1:
                mid = (lo + hi) // 2
                made = builder(mid, around, reach);stats['layouts'] += 1
                # A step inside a regime shows only between refined knots: such rows are solved in full here.
                found = (made.row() if steps else made.layout()).topo
                if found == A.topo:lo, below = mid, made
                elif found == B.topo:hi, above = mid, made
                else:
                    third = made
                    break
            stats['layout_rows'] += 1
            marks = [ha]
            for made in (below, third, above):
                if made is not None:
                    insert(made.row());marks.append(made.h)
            marks.append(hb)
            for a, b in zip(marks, marks[1:]):
                if third is not None or b - a > 1:push(a, b)
            continue
        hm = (ha + hb) // 2
        M = row_at(hm, *expected(i, hm))
        ok, worst, edge, modes = reproduced(rows, i, M, scales, soft, allow)
        A.modes = M.modes = modes
        insert(M)
        if log is not None:log.append((ha, hb, ok, worst, edge, len(M.pieces)))
        if ok:continue
        stats['layout_rows' if edge is None else 'edge_rows' if edge > EDGE else 'value_rows'] += 1
        if edge is None and M.topo not in (A.topo, B.topo):
            # A layout between two rows of one layout: its sides are different-layout intervals (handled above).
            push(ha, hm);push(hm, hb)
            continue
        push(ha, hm);push(hm, hb)
    stats.update(rows=len(rows), points=sum(r.solved for r in rows), knots=sum(len(p.x) for r in rows for p in r.pieces))
    return rows, stats


def covering(row, x):
    """The row's piece at speed x, or None."""
    return next((p for p in row.pieces if p.x[0] <= x <= p.x[-1]), None)


def plain(rows, i, h, k, x):
    """Curve k at altitude h and speed x between rows[i] and rows[i + 1] of different layouts, which have no checked
    blend (only where a surface stopped at its row or time limit before such an interval was down to neighbouring
    metres): straight between the two rows where both have a result at x, else the nearer row's own."""
    A, B = rows[i], rows[i + 1];pa, pb = covering(A, x), covering(B, x)
    if pa is not None and pb is not None:
        ya, yb = piece_value(pa, k, x), piece_value(pb, k, x)
        return None if ya is None or yb is None else ya + (yb - ya) * (h - A.h) / (B.h - A.h)
    near = pa if h - A.h <= B.h - h else pb
    return None if near is None else piece_value(near, k, x)


def layout(rows, hs, h):
    """How altitude h is drawn: (i, None) on row i itself, (i, 'plain') between rows i and i + 1 of different layouts
    (`plain`), else (i, [(first, last), ...]): between rows i and i + 1, with each piece's ends."""
    i = max(0, min(len(rows) - 1, bisect_right(hs, h) - 1))
    if rows[i].h == h or i + 1 >= len(rows):return i, None
    A, B = rows[i], rows[i + 1]
    if A.topo != B.topo:return i, 'plain'
    return i, [ends(rows, i, h, pi) for pi in range(len(A.pieces))]


def curve(rows, hs, h, k):
    """The drawn curve k at altitude h: [(x, y), ...] per piece, at whole speeds, as app/engine.js draws it."""
    import math
    i, spans = layout(rows, hs, h)
    if spans is None:return [[(x, piece_value(p, k, x)) for x in range(p.x[0], p.x[-1] + 1)] for p in rows[i].pieces]
    if spans == 'plain':
        out, current = [], None
        both = [p for r in rows[i:i + 2] for p in r.pieces]
        for x in range(min((p.x[0] for p in both), default=0), max((p.x[-1] for p in both), default=-1) + 1):
            near = covering(rows[i], x) if h - rows[i].h <= rows[i + 1].h - h else covering(rows[i + 1], x)
            if near is None and (covering(rows[i], x) is None or covering(rows[i + 1], x) is None):
                current = None
                continue
            if current is None:
                current = [];out.append(current)
            current.append((x, plain(rows, i, h, k, x)))
        return out
    return [[(x, value(rows, i, h, pi, k, x, first, last)) for x in range(math.ceil(first - .5), math.ceil(last + .5))]
            for pi, (first, last) in enumerate(spans)]


def point(rows, hs, h, k, x):
    """The drawn curve k at altitude h and whole speed x, or None outside every piece (the altitude view's samples)."""
    i, spans = layout(rows, hs, h)
    if spans is None:
        return next((piece_value(p, k, x) for p in rows[i].pieces if p.x[0] <= x <= p.x[-1]), None)
    if spans == 'plain':return plain(rows, i, h, k, x)
    for pi, (first, last) in enumerate(spans):
        if first - .5 <= x < last + .5:return value(rows, i, h, pi, k, x, first, last)
    return None
