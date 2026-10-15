"""Adaptive full-chart presentation of stationary numerical candidates.

Checked interpolation is a numerical diagnostic, not root coverage, physical
absence, or certified governor/aircraft selection. Inconclusive cells stay
masked. Neither isolated failures nor a chart search ceiling define feasibility.
"""
import math
import time

import numpy as np
from em_accuracy_policy import sep_error_budget

REVISION = 'stationary-adaptive-candidate-chart-v1'
MAX_DEPTH = 3


def chart_point(result, tolerance):
    """Keep separated candidate branches out of a single-valued contour map."""
    selected = result.get('selection_status') == 'numerical_selected'
    supplied = result.get('selected_candidates' if selected else 'stationary_candidates', [])
    points = [p for p in supplied if p.get('local_valid', False)]
    # A production selection names one admitted point. An invalid/missing
    # selection must not be replaced silently by a different recovered root.
    if selected and (len(supplied) != 1 or len(points) != 1):points = []
    row = dict(result)
    if selected and points:
        row.update(points[0])
        # Point fields come from the chosen state; retain the complete search
        # record and its numerical-selection label separately.
        for key in ('stationary_candidates', 'stationary_attempts', 'propeller_branch_recovery',
                    'local_candidate_count', 'selected_candidates', 'selection_status',
                    'selection_scope', 'seconds'):
            if key in result:row[key] = result[key]
    row.setdefault('turn_dps', math.degrees(9.8100004196167 *
        math.sqrt(max(0., row['load_g']**2-1.)) / (row['speed_kmh']/3.6)))
    row.setdefault('flaps_percent', 0.)
    row.setdefault('flaps_requested_percent', 0.)
    row.update(chart_usable=False, valid=False, status='unresolved',
        equilibrium_coverage_complete=False, aircraft_selection_certified=False,
        governor_selection_certified=False)
    row['chart_status'] = 'no locally admitted candidate'
    row['point_sep_error_budget_mps'] = float(sep_error_budget(tolerance))
    row['point_sep_error_certified'] = False
    # Only nearby replays on the same source branch are grouped. Their allowed
    # SEP spread uses the numerical allowance; spatial interpolation still uses
    # the chart's separate tolerance and unchanged sampler checks.
    grouping_budget = row['point_sep_error_budget_mps']
    row['chart_grouping_sep_budget_mps'] = grouping_budget
    if not points:return row
    first = points[0] if selected else min(points, key=lambda p: tuple(p['stationary_coordinates']))
    spread = max(p['ps_mps'] for p in points)-min(p['ps_mps'] for p in points)
    # This identifies numerically nearby replays for plotting only. It does not
    # merge exact roots or remove any candidate from the underlying result.
    scales = np.array(first.get('stationary_coordinate_scales',
                      [.01, .01, .0001, .0001, .0001, .01, .01] +
                      [.0001] * (len(first['stationary_coordinates'])-7)))
    same = all(len(p['stationary_coordinates']) == len(scales) and
        np.max(np.abs(np.asarray(p['stationary_coordinates'])-
                      first['stationary_coordinates'])/scales) <= 1.
        for p in points)
    same = same and all(p.get('source_branch') == first.get('source_branch') and
                       p['sideslip_attitude_deg'] == first['sideslip_attitude_deg'] for p in points)
    row.update(chart_sep_spread_mps=spread, chart_candidate=first,
        chart_usable=bool(same and spread <= grouping_budget),
        valid=bool(same and spread <= grouping_budget),
        status='admitted' if same and spread <= grouping_budget else 'unresolved',
        chart_status=('numerically selected equilibrium' if selected else 'nearby local candidates')
                     if same and spread <= grouping_budget
                     else 'multiple numerical candidate branches')
    return row


def cell_decision(rows, tolerance):
    """Nine-point interpolation check; does not certify the cell interior."""
    if not all(r['chart_usable'] for r in rows):return False, None
    branches = [r['chart_candidate'].get('source_branch') for r in rows]
    if any(b != branches[0] for b in branches[1:]):return False, None
    z = [r['chart_candidate']['ps_mps'] for r in rows]
    estimates = [(z[0]+z[1])/2, (z[1]+z[2])/2,
                 (z[2]+z[3])/2, (z[3]+z[0])/2, sum(z[:4])/4]
    error = max(abs(a-b) for a,b in zip(estimates, z[4:]))
    return error <= tolerance, error


class Surface:
    def __init__(self, solver, progress=None, cancelled=None):
        self.solver, self.progress, self.cancelled = solver, progress, cancelled
        self.points, self.keys, self.cells = [], {}, []
        self.started = time.monotonic()
        self.ceilings = {}
        self.tolerance = solver.config['sep_tolerance_mps']

    def ceiling(self, speed):
        if speed not in self.ceilings:
            from em_aircraft_region import preliminary
            c = self.solver.config
            top = min(c.get('max_load_g') or 64., c['global_load_cap_g'])
            estimate = preliminary(self.solver, speed)
            if estimate is not None:top = min(top, estimate['initial_load_g'])
            self.ceilings[speed] = dict(load_g=max(1.1, top), estimate=estimate,
                certifies_exclusion=False, meaning='chart search extent only')
        return self.ceilings[speed]['load_g']

    def at(self, speed, fraction):
        key = float(speed), float(fraction)
        if key in self.keys:return self.keys[key]
        if self.cancelled and self.cancelled():raise InterruptedError('Calculation cancelled')
        from em_stationary import solve
        cap = self.ceiling(speed)
        load = math.sqrt(1.+fraction*fraction*(cap*cap-1.))
        usable = [p for p in self.points if p['chart_usable']]
        seed = min(usable, key=lambda p: abs(p['chart_fraction']-fraction)+
                   abs(p['speed_kmh']-speed)/max(speed,100.))['chart_candidate'] if usable else None
        result = solve(self.solver, speed, load, initial=seed, exhaustive=False)
        row = chart_point(result, self.tolerance)
        # Search at a known nonzero sideslip can be added by the point solver;
        # this chart's finite zero-sideslip search is never a global preference proof.
        row.update(speed_kmh=float(speed), chart_fraction=float(fraction))
        return self.record(row)

    def record(self, row):
        speed, fraction = row['speed_kmh'], row['chart_fraction']
        key = float(speed), float(fraction)
        if key in self.keys:raise ValueError('Stationary point evaluated twice')
        index = len(self.points)
        self.keys[key] = index
        self.points.append(row)
        if self.progress:self.progress(dict(done=len(self.points), total=max(len(self.points)+1,
            self.solver.config['speed_samples']*self.solver.config['load_samples']),
            aircraft=self.solver.name, speed_kmh=speed, load_g=row['load_g'],
            phase='Stationary chart: adaptive candidate/interpolation checks',
            elapsed_s=time.monotonic()-self.started))
        return index

    def cell(self, lo, hi, bottom, top, depth):
        mid, half = (lo+hi)/2., (bottom+top)/2.
        coords = [(lo,bottom),(hi,bottom),(hi,top),(lo,top),
                  (mid,bottom),(hi,half),(mid,top),(lo,half),(mid,half)]
        # A center trial probes a possible interior branch even when the four
        # corner searches failed. Unsearched interiors remain unresolved.
        corners = [self.at(*q) for q in coords[:4]]
        center = self.at(mid, half)
        if not any(self.points[i]['chart_usable'] for i in corners+[center]):
            self.cells.append(dict(corners=corners, center=center, checked=False,
                depth=depth, reason='no usable sampled branch; interior unresolved'))
            return
        indices = corners+[self.at(*q) for q in coords[4:8]]+[center]
        passed, error = cell_decision([self.points[i] for i in indices], self.tolerance)
        if passed or depth >= MAX_DEPTH:
            self.cells.append(dict(corners=corners, samples=indices, center=center,
                checked=bool(passed), depth=depth, sampled_sep_error_mps=error,
                reason='sampled interpolation check passed' if passed else
                       'branch/admission/interpolation unresolved at refinement limit'))
        else:
            for a,b,c,d in ((lo,mid,bottom,half),(mid,hi,bottom,half),
                            (lo,mid,half,top),(mid,hi,half,top)):
                self.cell(a,b,c,d,depth+1)

    def grid(self):
        c = self.solver.config
        from em_speed_limits import speed_limits
        limits = speed_limits(self.solver.fm, c)
        high = min(c['speed_max_kmh'], limits['sample_speed_kmh']) if limits['enforced'] else c['speed_max_kmh']
        if high <= c['speed_min_kmh']:
            return limits, [], []
        speeds = np.linspace(c['speed_min_kmh'], high, c['speed_samples'])
        fractions = np.linspace(0., 1., c['load_samples'])
        return limits, speeds, fractions

    def result(self, limits, speeds):
        return dict(points=self.points, cells=self.cells, speed_limit=limits,
            plot_speed_interval_kmh=[float(speeds[0]),float(speeds[-1])] if len(speeds) else [],
            search_extents=[dict(speed_kmh=v, **r) for v,r in sorted(self.ceilings.items())])

    def run(self):
        limits, speeds, fractions = self.grid()
        for speed in speeds:
            for fraction in fractions:self.at(float(speed), float(fraction))
        for lo,hi in zip(speeds,speeds[1:]):
            for bottom,top in zip(fractions,fractions[1:]):
                self.cell(float(lo),float(hi),float(bottom),float(top),0)
        result=self.result(limits,speeds)
        result['execution']=dict(method='serial_cells',workers=1)
        return result


def compute(config, progress=None, cancelled=None, preview=None):
    from em_solver import AIRCRAFT, TrimSolver, aircraft_settings
    started = time.monotonic()
    data = dict(settings=config, stationary_full_chart=True, model_revision=REVISION,
        aircraft=[], speeds_kmh=[], loads_g=[], method='adaptive stationary candidate chart',
        validation='Provisional numerical interpolation; global selection and physical margins uncertified',
        assumptions=['Upright bank within +/-90 degrees; forward-flow propellers including blade stall; all power signs',
                     'Chart speed/load extents are not domain-wide absence proofs',
                     'Unresolved or separated numerical branches remain masked',
                     'Zero attitude sideslip candidate search; global preference unresolved'],
        max_refinement_depth=MAX_DEPTH, coverage_complete=False, selection_certified=False)
    for index,name in enumerate(config['aircraft']):
        condition = aircraft_settings(config, name)
        if condition['trim_solver_mode'] != 'stationary':
            raise ValueError('Mixed stationary/legacy charts require separate runs')
        solver = TrimSolver(name, condition)
        if condition['stationary_execution']=='columns':
            from em_stationary_columns import ColumnSurface
            surface=ColumnSurface(solver,progress,cancelled)
        else:surface=Surface(solver,progress,cancelled)
        from em_model_experiments import description, label
        experiment=label(condition)
        def aircraft_row(result):
            return dict(result, id=name, name=AIRCRAFT[name]['name']+(' · '+experiment if experiment else ''),
                color=['#38c9d7','#ffa66b'][index % 2], settings=condition,
                model_experiments=description(condition),
                mass=solver.mass, engine=solver.engine.summary,
                boundary=[], sustained=[], numerical_gaps=[],
                valid_points=0, local_candidate_points=sum(p['local_valid'] for p in result['points']),
                converged_points=sum(p['converged'] for p in result['points']))
        if preview and condition['stationary_execution']=='columns':
            last_preview=[0.,-1]
            def publish():
                now=time.monotonic()
                if len(surface.points)==last_preview[1] or now-last_preview[0]<3.:return
                if not surface.points:return
                from em_stationary_chart import clean
                snapshot=clean(dict(data,preview=True,elapsed_s=now-started,
                    aircraft=[*data['aircraft'],aircraft_row(surface.partial())]))
                snapshot['speeds_kmh']=sorted({p['speed_kmh'] for a in snapshot['aircraft'] for p in a['points']})
                enrich(snapshot)
                preview(snapshot)
                last_preview[:]=[now,len(surface.points)]
            surface.on_update=publish
        result = surface.run()
        data['aircraft'].append(aircraft_row(result))
    data['speeds_kmh'] = sorted({p['speed_kmh'] for a in data['aircraft'] for p in a['points']})
    data['elapsed_s'] = time.monotonic()-started
    from em_stationary_chart import clean
    data = clean(data)
    enrich(data)
    if preview:preview(data)
    return data


def mesh(aircraft):
    import matplotlib.tri as tri
    points = aircraft['points']
    triangles = []
    for cell in aircraft['cells']:
        if not cell['checked']:continue
        # Use all nine checked nodes. Eight local triangles avoid extrapolating
        # across an unchecked neighbor, edge or masked component.
        ids = cell['samples']
        ring = [ids[i] for i in (0,4,1,5,2,6,3,7)]
        triangles.extend([ids[8],a,b] for a,b in zip(ring,ring[1:]+ring[:1]))
    if not triangles:return None
    used = sorted({i for t in triangles for i in t})
    lookup = {i:j for j,i in enumerate(used)}
    triangulation = tri.Triangulation([points[i]['speed_kmh'] for i in used],
        [points[i]['turn_dps'] for i in used], [[lookup[i] for i in t] for t in triangles])
    return triangulation, np.array([points[i]['chart_candidate']['ps_mps'] for i in used])


def contours(data, levels=None):
    import matplotlib.pyplot as plt
    output = {}
    for a in data['aircraft']:
        shape = mesh(a)
        paths = []
        if shape:
            triangulation,z = shape
            selected = sorted(levels if levels is not None else data['settings']['sep_contour_levels_mps'])
            selected = [v for v in selected if z.min() <= v <= z.max()]
            if selected:
                fig,ax = plt.subplots()
                lines = ax.tricontour(triangulation,z,levels=selected)
                for level, segments in zip(lines.levels, lines.allsegs):
                    paths.extend(dict(level=float(level),x=seg[:,0].tolist(),y=seg[:,1].tolist(),
                        certified=False) for seg in segments if len(seg)>1)
                plt.close(fig)
        output[a['id']] = paths
    return output


def enrich(data):
    paths = contours(data)
    admitted = [p for a in data['aircraft'] for p in a['points'] if p.get('chart_usable')]
    data['plot_max_turn'] = max(10., math.ceil(max((p['turn_dps'] for p in admitted),default=0.)*1.08+1.))
    data['plot_max_load_g'] = max([1.1]+[p['load_g'] for p in admitted])
    for a in data['aircraft']:
        a['contours'] = paths[a['id']]
        a['checked_cells'] = sum(c['checked'] for c in a['cells'])
        a['unresolved_cells'] = len(a['cells'])-a['checked_cells']
        a['sustained_curve'] = dict(x=[],y=[])
    return data


def export_figure(data, path, selected=None, levels=None, format=None):
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection
    paths = contours(data, levels)
    fig,ax = plt.subplots(figsize=(11,7), layout='constrained')
    for a in data['aircraft']:
        if selected is not None and a['id'] != selected:continue
        shape = mesh(a)
        if shape:
            t,z = shape
            polygons = [list(zip(t.x[ids],t.y[ids])) for ids in t.triangles]
            ax.add_collection(PolyCollection(polygons, facecolors=a['color'], alpha=.10, edgecolors='none'))
        for line in paths[a['id']]:
            level = line['level']
            ax.plot(line['x'],line['y'],color=a['color'] if level==0 else '#435065',
                    linewidth=2.5 if level==0 else 1.,linestyle='-' if level==0 else '--')
            if len(line['x']) > 5:
                i=len(line['x'])//2
                ax.text(line['x'][i],line['y'][i],f'SEP {level:g}',fontsize=8,
                    bbox=dict(facecolor='white',edgecolor='none',alpha=.75,pad=1))
        p=[r for r in a['points'] if r.get('chart_usable')]
        ax.scatter([r['speed_kmh'] for r in p],[r['turn_dps'] for r in p],s=3,
                   color=a['color'],alpha=.4,label=a['name'])
    config=data['settings']
    x=np.linspace(config['speed_min_kmh'],config['speed_max_kmh'],1000)
    for n in (2,4,6,9,12,16):
        if n<=data['plot_max_load_g']:
            y=np.degrees(9.8100004196167*np.sqrt(n*n-1)/(x/3.6))
            ax.plot(x,y,color='#a0a8b3',linewidth=.5,alpha=.4)
    ax.set(xlim=(config['speed_min_kmh'],config['speed_max_kmh']),ylim=(0,data['plot_max_turn']),
        xlabel='True airspeed (km/h)',ylabel='Turn rate (degrees/s)')
    ax.grid(alpha=.15)
    ax.legend(loc='upper right',fontsize=9)
    fig.suptitle('Energy–maneuverability — provisional stationary chart',fontsize=14)
    ax.set_title('Upright bank | shaded cells passed sampled interpolation checks',fontsize=10)
    fig.supxlabel('Unshaded regions are unresolved or outside the chart search extent.\n'
        'Contours describe numerical candidates; global selection, coverage and physical-margin accuracy remain uncertified.',fontsize=9)
    fig.savefig(path,dpi=180,format=format)
    plt.close(fig)
