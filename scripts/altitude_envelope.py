"""Checked speed/altitude SEP surface from unchanged 1-g aircraft equilibria.

All accepted samples pass TrimSolver's original force, moment, history,
propulsion and physical-limit checks. Instructor is not part of this calculation.
Interpolation is checked at extra speed and altitude samples. Unresolved intervals are masked, never
converted into a stall or a ceiling. No EM sampler/cache is used or modified.
"""
import altitude_runtime  # Must precede the equation imports.
import math
import time
from collections import OrderedDict, deque

import numpy as np
from em_solver import AIRCRAFT, AIRCRAFT_SETTINGS, TrimSolver, settings as em_settings
from em_speed_limits import speed_limits

QUALITY = {
    'preview': dict(tolerance=1., depth=4, budget=6000),
    'smooth': dict(tolerance=.5, depth=6, budget=12000),
    'detailed': dict(tolerance=.15, depth=7, budget=20000),
}
DEFAULTS = dict(aircraft='f_16a_block_15_adf', speed_min_kmh=100., speed_max_kmh=1600.,
                altitude_min_m=0., altitude_max_m=16000., quality='preview',
                contour_interval_mps=25., conditions={})
PHYSICAL = {'post-stall', 'control authority', 'wing force limit', 'IAS limit',
            'Mach limit', 'sweep unavailable', 'Instructor pitch limit', 'speed redline'}


def settings(values=None):
    if values is not None and not isinstance(values, dict):
        raise ValueError('Expected settings object')
    result = dict(DEFAULTS, **(values or {}))
    if set(result) - set(DEFAULTS):
        raise ValueError('Unknown altitude plot setting')
    name = result['aircraft']
    if not isinstance(name, str) or name not in AIRCRAFT:
        raise ValueError('Select an aircraft')
    if not isinstance(result['quality'], str) or result['quality'] not in QUALITY:
        raise ValueError('Unknown sampling quality')
    for key, lo, hi in [('altitude_min_m', 0, 17900), ('altitude_max_m', 100, 18000),
                        ('speed_min_kmh', 100, 2500), ('speed_max_kmh', 200, 2600),
                        ('contour_interval_mps', 1, 100)]:
        v = result[key]
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or not lo <= v <= hi:
            raise ValueError(f'{key} must be between {lo} and {hi}')
        result[key] = float(v)
    if result['altitude_max_m'] <= result['altitude_min_m']:
        raise ValueError('Maximum altitude must exceed minimum altitude')
    if result['speed_max_kmh'] <= result['speed_min_kmh']:
        raise ValueError('Maximum speed must exceed minimum speed')
    conditions = result['conditions']
    allowed = AIRCRAFT_SETTINGS - {'altitude_m'}
    if not isinstance(conditions, dict) or set(conditions) - allowed:
        raise ValueError('Unknown aircraft condition; altitude belongs to the plot range')
    # Legacy saved/requested modes normalize to the same clean level-flight
    # calculation. These controls do not belong to an altitude SEP diagram.
    resolved = em_settings(dict(conditions, aircraft=[name], instructor=False,
                                flaps_percent=0., torque_gyro=False))
    result['conditions'] = {k: resolved[k] for k in sorted(allowed)}
    return result


def category(point):
    if point['valid']:
        return 'valid'
    if point.get('converged') and set(point['reasons']) <= PHYSICAL:
        return 'physical limit'
    return 'numerical gap'


def public_point(point):
    return {k: v for k, v in point.items() if not k.startswith('_')}


class LevelSampler:
    def __init__(self, config):
        self.config = config
        self.solvers = OrderedDict()
        self.neighbors = {}
        self.redlines = {}

    def solver(self, height):
        if height not in self.solvers:
            c = dict(self.config['conditions'], aircraft=[self.config['aircraft']], altitude_m=height)
            self.solvers[height] = TrimSolver(self.config['aircraft'], c)
            # Bound retained native engine/controller state on large plots.
            if len(self.solvers) > 128:
                self.solvers.popitem(last=False)
        self.solvers.move_to_end(height)
        return self.solvers[height]

    def __call__(self, speed, height):
        s = self.solver(height)
        if height not in self.redlines:
            self.redlines[height] = speed_limits(s.fm, s.config)
        redline = self.redlines[height]
        if redline['enforced'] and speed > redline['sample_speed_kmh']:
            return dict(speed_kmh=speed, altitude_m=height, load_g=1., valid=False,
                        converged=True, reasons=['speed redline'], category='physical limit',
                        ps_mps=None, speed_limit=redline)
        near = self.neighbors.get(height, [])
        seed = min(near, key=lambda p: abs(p['speed_kmh'] - speed))['solution'] if near else None
        predictor = getattr(s, '_trim_predictor', None)
        if predictor and abs(speed-predictor[0][0]) < .15*speed:
            # A nearby Jacobian is only a search direction. The existing
            # safeguarded Newton search and native acceptance checks decide
            # whether the next point is actually an equilibrium.
            s._trim_predictor = ((speed,s.sideslip_attitude_deg),predictor[1],1.)
        p = s.solve(speed, 1., seed, exhaustive=False,quick=True)
        if not p['converged'] and not p.get('bounded_search_stationary'):
            guess = s.initial_guess(speed / 3.6, 1.)
            guess[0] *= .5
            retry = s.solve(speed, 1., guess, exhaustive=False,quick=True)
            if retry['valid'] or (retry['converged'] and not p['converged']):
                p = retry
        if p['converged'] and p['reasons'] == ['vertical step did not close'] and p['altitude_correction']:
            # The native high-altitude correction is one-sided at vy=0.
            # Try nearby rounded equilibria on its nonascending side, with
            # ALL normal acceptance checks intact. Never erase a rejection.
            p = nonascending_recheck(s, speed, p)
        if p['valid']:
            near.append(dict(speed_kmh=speed, solution=p['solution']))
            self.neighbors[height] = near
        p = public_point(p)
        p.update(altitude_m=height, category=category(p))
        return p

    def recover_edge(self,speed,height,left,right):
        # Boundary exploration also encounters genuinely stalled states.
        # One neighboring seed is enough to recover missed valid intervals;
        # an exhaustive search at every excluded point is prohibitively slow.
        return self.recover(speed,height,left,right,thorough=False)

    def recover(self, speed, height, left, right,thorough=True):
        """Retry an interior numerical hole with a bracketed trim seed.

        The quick search can stop at a poor local direction. Interpolating
        two accepted nearby trims and rebuilding its Jacobian is a search
        repair only; native equilibrium/physical acceptance is unchanged.
        """
        s=self.solver(height)
        fraction=(speed-left['speed_kmh'])/(right['speed_kmh']-left['speed_kmh'])
        seed=(1-fraction)*np.array(left['solution'])+fraction*np.array(right['solution'])
        guesses=(seed.tolist(),left['solution'],right['solution']) if thorough else (seed.tolist(),)
        for guess in guesses:
            s._trim_predictor=None
            p=s.solve(speed,1.,guess,exhaustive=False,quick=False)
            if p['converged'] and p['reasons']==['vertical step did not close'] and p['altitude_correction']:
                p=nonascending_recheck(s,speed,p)
            if p['valid']:break
        if thorough and not p['valid'] and not s.is_prop:
            s._trim_predictor=None
            p=s.solve(speed,1.,p['solution'],exhaustive=True,quick=False)
            if p['converged'] and p['reasons']==['vertical step did not close'] and p['altitude_correction']:
                p=nonascending_recheck(s,speed,p)
        if p['valid']:
            self.neighbors.setdefault(height,[]).append(dict(speed_kmh=speed,solution=p['solution']))
            p['altitude_recovery']='full search from neighboring accepted trims'
        p=public_point(p);p.update(altitude_m=height,category=category(p))
        return p


def nonascending_recheck(solver, speed, point):
    """Find a seed just inside the existing force tolerance below vy=0.

    The Newton target is -2e-6 g (100 times smaller than the force tolerance).
    It only chooses an initial guess. The unchanged solver then independently
    accepts/rejects the complete native state, including vertical closure.
    """
    x = np.array(point['solution'], dtype=float)
    target = np.array([-2e-6, 0., 0., 0., 0.])
    for _ in range(4):
        value = solver.operating_point(speed / 3.6, 1., x)
        predictor = getattr(solver,'_trim_predictor',None)
        if predictor and _ == 0:
            matrix = predictor[1]
        else:
            columns = []
            for axis, step in enumerate((.002, .002, .0002, .0002, .0002)):
                trial = x.copy(); trial[axis] += step
                q = solver.operating_point(speed / 3.6, 1., trial)
                columns.append((q['residual']-value['residual']) / step)
            matrix = np.column_stack(columns)
        delta = np.linalg.lstsq(matrix, target-value['residual'], rcond=None)[0]
        if not np.all(np.isfinite(delta)) or np.max(abs(delta)) > .02:
            break
        x += delta
        if any(x < np.array(solver.trim_bounds[0])) or any(x > np.array(solver.trim_bounds[1])):
            break
        retry = solver.solve(speed, 1., x.tolist(), exhaustive=False,quick=True)
        if retry['valid']:
            retry['altitude_recovery'] = 'balanced nonascending native rounding branch'
            return retry
    return point


def adaptive_mesh(config, evaluate, progress=None, cancelled=None):
    """Checked quadtree with a conforming final triangular mesh.

    Adjacent leaves share every sampled edge vertex, including finer ones;
    this prevents cracks or independently drawn contour fragments at T joins.
    The budget limits work, never loosens the requested error check.
    """
    policy = QUALITY[config['quality']]
    points, lookup, accepted, masked = [], {}, [], []
    done = 0
    started = time.monotonic()
    pending = deque()

    def at(v, h):
        key = (v, h)
        if key in lookup:
            return lookup[key]
        if cancelled and cancelled():
            raise InterruptedError('Calculation cancelled')
        if len(points) >= policy['budget']:
            raise OverflowError('sample budget')
        p = evaluate(v, h)
        p.update(speed_kmh=v, altitude_m=h)
        if p['valid'] and not math.isfinite(p['ps_mps']):
            p.update(valid=False, reasons=['nonfinite SEP'], category='numerical gap')
        p.setdefault('category', category(p))
        lookup[key] = len(points)
        points.append(p)
        if progress and (len(points) % 8 == 0 or len(points) == 1):
            progress(dict(samples=len(points), done=done, total=done+len(pending)+1, phase='Solving level flight',
                          altitude_m=h, elapsed_s=time.monotonic() - started))
        return len(points) - 1

    def cell(x0, x1, y0, y1, xdepth, ydepth):
        xm, ym = (x0 + x1) / 2, (y0 + y1) / 2
        bounds = [x0, x1, y0, y1]
        try:
            ids = [[at(x, y) for x in (x0, xm, x1)] for y in (y0, ym, y1)]
        except OverflowError:
            masked.append(dict(bounds=bounds, reason='sample budget'))
            return
        samples = [points[i] for row in ids for i in row]
        good = [p['valid'] for p in samples]
        split_x = split_y = True
        if all(good):
            z = np.array([[points[i]['ps_mps'] for i in row] for row in ids])
            expected = [(z[0, 0] + z[0, 2]) / 2, (z[2, 0] + z[2, 2]) / 2,
                        (z[0, 0] + z[2, 0]) / 2, (z[0, 2] + z[2, 2]) / 2,
                        (z[0, 0] + z[0, 2] + z[2, 0] + z[2, 2]) / 4]
            observed = [z[0, 1], z[2, 1], z[1, 0], z[1, 2], z[1, 1]]
            # Also resolve bilinear saddle curvature before triangulation.
            error = max(max(abs(a-b) for a, b in zip(expected, observed)),
                        abs(z[0, 0] - z[0, 2] - z[2, 0] + z[2, 2]) / 4)
            if error <= policy['tolerance']:
                accepted.append(dict(bounds=bounds, center=ids[1][1], error_mps=float(error)))
                return
            reason = 'interpolation unresolved'
            xerror = max(abs(z[j,1]-(z[j,0]+z[j,2])/2) for j in range(3))
            yerror = max(abs(z[1,i]-(z[0,i]+z[2,i])/2) for i in range(3))
            saddle = abs(z[0,0]-z[0,2]-z[2,0]+z[2,2])/4
            # Transonic changes often need speed refinement alone. Avoid
            # multiplying expensive altitude solves when that axis is smooth.
            if yerror < policy['tolerance']*.25 and saddle < policy['tolerance']*.5:
                split_y = False
            elif xerror < policy['tolerance']*.25 and saddle < policy['tolerance']*.5:
                split_x = False
        else:
            reason = 'numerical gap' if any(p['category'] == 'numerical gap' for p in samples) else 'physical boundary'
            if not any(good):
                masked.append(dict(bounds=bounds, reason=reason))
                return
            flags = np.array(good).reshape(3,3)
            if np.all(flags == flags[0,:]):
                split_y = False
            elif np.all(flags == flags[:,0,None]):
                split_x = False
        split_x = split_x and xdepth < policy['depth']
        split_y = split_y and ydepth < policy['depth']
        if not split_x and not split_y:
            masked.append(dict(bounds=bounds, reason=reason))
            return
        xx = [(x0,xm),(xm,x1)] if split_x else [(x0,x1)]
        yy = [(y0,ym),(ym,y1)] if split_y else [(y0,y1)]
        for ya,yb in yy:
            for xa,xb in xx:
                pending.append((xa,xb,ya,yb,xdepth+int(split_x),ydepth+int(split_y)))

    xs = np.linspace(config['speed_min_kmh'], config['speed_max_kmh'], 9).tolist()
    ys = np.linspace(config['altitude_min_m'], config['altitude_max_m'], 7).tolist()
    for y0, y1 in zip(ys, ys[1:]):
        # Start each band at high speed to seed low-AoA level equilibria.
        for x0, x1 in reversed(list(zip(xs, xs[1:]))):
            pending.append((x0, x1, y0, y1, 0, 0))
    while pending:
        cell(*pending.popleft())
        done += 1
    horizontal, vertical = {}, {}
    for i, p in enumerate(points):
        horizontal.setdefault(p['altitude_m'], []).append((p['speed_kmh'], i))
        vertical.setdefault(p['speed_kmh'], []).append((p['altitude_m'], i))
    for group in (horizontal, vertical):
        for row in group.values():
            row.sort()
    triangles, cells = [], []
    for c in accepted:
        x0, x1, y0, y1 = c['bounds']
        edge = ([i for x, i in horizontal[y0] if x0 <= x < x1] +
                [i for y, i in vertical[x1] if y0 <= y < y1] +
                [i for x, i in reversed(horizontal[y1]) if x0 < x <= x1] +
                [i for y, i in reversed(vertical[x0]) if y0 < y <= y1])
        if not all(points[i]['valid'] for i in edge):
            masked.append(dict(bounds=c['bounds'], reason='edge check unresolved'))
            continue
        # Finer neighbors can expose curvature missed by a coarse edge.
        corners = [points[lookup[(x, y)]]['ps_mps'] for x, y in [(x0,y0),(x1,y0),(x0,y1),(x1,y1)]]
        edge_error = 0.
        for i in edge:
            p = points[i]; u = (p['speed_kmh']-x0)/(x1-x0); v = (p['altitude_m']-y0)/(y1-y0)
            estimate = corners[0]*(1-u)*(1-v)+corners[1]*u*(1-v)+corners[2]*(1-u)*v+corners[3]*u*v
            edge_error = max(edge_error, abs(estimate-p['ps_mps']))
        if edge_error > policy['tolerance']:
            masked.append(dict(bounds=c['bounds'], reason='edge interpolation unresolved'))
            continue
        c['error_mps'] = max(c['error_mps'], edge_error)
        cells.append(c)
        triangles.extend([[c['center'], a, b] for a, b in zip(edge, edge[1:] + edge[:1])])
    return dict(points=points, triangles=triangles, cells=cells, masked_cells=masked,
                sampling=dict(tolerance_mps=policy['tolerance'], max_depth=policy['depth'],
                              sample_budget=policy['budget'], samples=len(points),
                              accepted_cells=len(cells), masked_cells=len(masked),
                              max_checked_error_mps=max((c['error_mps'] for c in cells), default=None)))


def compute(values=None, progress=None, cancelled=None, pool=None, stop=None):
    config = settings(values)
    started = time.monotonic()
    sampler = LevelSampler(config)
    from altitude_rows import compute_rows
    data = compute_rows(config, pool, stop, progress, cancelled)
    # Independently draw the analytic IAS/Mach redline at dense altitudes.
    s = sampler.solver(config['altitude_min_m'])
    limits = [dict(altitude_m=float(h), **speed_limits(s.fm, dict(s.config, altitude_m=float(h))))
              for h in np.linspace(config['altitude_min_m'], config['altitude_max_m'], 129)]
    surface=data['surface']
    if config['conditions']['structural_limits']:
        for i,h in enumerate(surface['altitudes_m']):
            limit=speed_limits(s.fm,dict(s.config,altitude_m=float(h)))['sample_speed_kmh']
            surface['sep_mps'][i,surface['speeds_kmh'][i]>limit]=np.nan
    data.update(schema='altitude-sep-v2', settings=config, aircraft_name=AIRCRAFT[config['aircraft']]['name'],
                speed_limits=limits, elapsed_s=time.monotonic()-started,
                method='Straight-flight 1-g trimmed operating points; native finite-step energy-height SEP. '
                       'Nonzero SEP describes energy gain/loss at the prescribed horizontal flight condition, not a steady climb. '
                       'Small bank needed to balance asymmetric forces is retained. '
                       'Native high-altitude correction and all existing equilibrium limits remain active. '
                       'Instructor-independent, retracted flaps, torque/gyro disabled. '
                       'Checked cubic speed curves and independent altitude probes; unresolved areas stay masked. '
                       'No global ceiling or complete envelope certification; fixed fuel and intact fully upgraded aircraft.')
    from altitude_energy import energy_guide
    data['energy_guide']=energy_guide(data)
    data['elapsed_s']=time.monotonic()-started
    return data
