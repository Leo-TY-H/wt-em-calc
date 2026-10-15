"""Baseline column execution infrastructure applied to stationary equations.

Uses the existing process pool, job epochs, cached worker solvers, cancellation
and OrderedWork. The legacy five-variable boundary/governor routines are not
called: local candidates cannot certify a selected root or a physical boundary.
"""
import json
import math
import time
from collections import defaultdict

from em_ordered_work import OrderedWork
from em_stationary_surface import Surface, cell_decision, chart_point, MAX_DEPTH

REVISION = 'stationary-baseline-column-execution-v1'


def seed_record(row):
    """Send coordinates, not megabytes of reporting, to a continuation task."""
    p=row['chart_candidate']
    return dict(speed_kmh=row['speed_kmh'],chart_fraction=row['chart_fraction'],
                candidate={k:p[k] for k in ('solution','stationary_coordinates',
                    'stationary_request_indices','sideslip_attitude_deg','model_experiments') if k in p})


def nearest(seeds, speed, fraction):
    return min(seeds,key=lambda p:(abs(p['chart_fraction']-fraction)+
        abs(p['speed_kmh']-speed)/max(speed,100.),p['speed_kmh'],p['chart_fraction'])) if seeds else None


def sample_column(task):
    """One worker walks requested loads in order, carrying full stationary state."""
    from em_workers import prepare_job
    prepare_job()
    from em_sampling import worker_solver
    from em_stationary import solve
    from em_cancellation import check
    solver=worker_solver(task['aircraft'],task['config_json'])
    # The baseline worker constructor enables heuristic chart admission. The
    # stationary chart uses these bounds for search only, never for exclusion.
    solver.chart_search=False
    solver._sideslip_solvers={}
    solver.instructor_boundaries={}
    solver.instructor_trim_entries={}
    for key in ('_trim_predictor','_minimum_instructor_speed','_static_instructor_trim_cache'):
        solver.__dict__.pop(key,None)
    seeds=list(task['seeds'])
    rows=[]
    for fraction, load in task['requests']:
        check()
        seed=nearest(seeds,task['speed_kmh'],fraction)
        result=solve(solver,task['speed_kmh'],load,
                     initial=seed['candidate'] if seed else None,exhaustive=False)
        row=chart_point(result,solver.config['sep_tolerance_mps'])
        row.update(speed_kmh=task['speed_kmh'],chart_fraction=fraction)
        rows.append(row)
        if row['chart_usable']:seeds.append(seed_record(row))
    return rows


class ColumnSurface(Surface):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.column_tasks=0
        self.reused_points=0
        self.on_update=None

    def check_cancel(self):
        if self.cancelled and self.cancelled():raise InterruptedError('Calculation cancelled')

    def batch(self,coordinates,pool,worker_cancel):
        """Freeze inputs for a refinement round, then accept in speed order.

        Completion timing and worker count cannot choose continuation seeds.
        Every speed task owns its mutable per-column history. All previously
        sampled coordinates are reused without another solve.
        """
        self.check_cancel()
        unique=sorted(set((float(v),float(f)) for v,f in coordinates))
        groups=defaultdict(list)
        for speed,fraction in unique:
            if (speed,fraction) in self.keys:
                self.reused_points+=1
                continue
            cap=self.ceiling(speed)
            load=math.sqrt(1.+fraction*fraction*(cap*cap-1.))
            groups[speed].append((fraction,load))
        if not groups:return
        config_json=json.dumps(self.solver.config,sort_keys=True)
        available=[seed_record(p) for p in self.points if p['chart_usable']]
        work=OrderedWork()
        try:
            for speed,requests in sorted(groups.items()):
                self.check_cancel()
                # Each request's nearest established state plus same-column
                # states suffice for nearest-neighbor continuation. No roots
                # are deleted from the point records or certified selected.
                selected={(p['speed_kmh'],p['chart_fraction']):p for p in available if p['speed_kmh']==speed}
                for fraction,_ in requests:
                    seed=nearest(available,speed,fraction)
                    if seed:selected[(seed['speed_kmh'],seed['chart_fraction'])]=seed
                task=dict(aircraft=self.solver.name,config_json=config_json,
                          speed_kmh=speed,requests=requests,
                          seeds=[selected[k] for k in sorted(selected)])
                work.submit(speed,future=pool.submit(sample_column,task))
                self.column_tasks+=1
            done=0
            reported=0.
            while work:
                self.check_cancel()
                for speed,rows in work.take():
                    for row in rows:self.record(row)
                    done+=1
                if self.on_update:self.on_update()
                now=time.monotonic()
                if self.progress and now-reported>=1.:
                    self.progress(dict(done=done,total=len(groups),aircraft=self.solver.name,
                        phase='Stationary speed columns: solving loads and checking interpolation',
                        sampled_points=len(self.points),elapsed_s=now-self.started))
                    reported=now
        except BaseException:
            worker_cancel.set()
            work.cancel()
            raise

    @staticmethod
    def coordinates(cell):
        lo,hi,bottom,top,depth=cell
        mid,half=(lo+hi)/2.,(bottom+top)/2.
        return [(lo,bottom),(hi,bottom),(hi,top),(lo,top),
                (mid,bottom),(hi,half),(mid,top),(lo,half),(mid,half)]

    def run(self):
        from em_workers import process_pool,WORKERS
        limits,speeds,fractions=self.grid()
        self.partial=lambda:self.result(limits,speeds)
        if not len(speeds):return self.result(limits,speeds)
        with process_pool() as (pool,worker_cancel):
            self.batch(((v,f) for v in speeds for f in fractions),pool,worker_cancel)
            pending=[(float(lo),float(hi),float(bottom),float(top),0)
                     for lo,hi in zip(speeds,speeds[1:])
                     for bottom,top in zip(fractions,fractions[1:])]
            while pending:
                self.check_cancel()
                coords=[self.coordinates(cell) for cell in pending]
                self.batch((q for c in coords for q in c[:4]+c[8:]),pool,worker_cancel)
                active=[]
                for cell,c in zip(pending,coords):
                    corners=[self.keys[q] for q in c[:4]]
                    center=self.keys[c[8]]
                    if not any(self.points[i]['chart_usable'] for i in corners+[center]):
                        self.cells.append(dict(corners=corners,center=center,checked=False,
                            depth=cell[4],reason='no usable sampled branch; interior unresolved'))
                    else:active.append((cell,c))
                self.batch((q for _,c in active for q in c[4:8]),pool,worker_cancel)
                pending=[]
                for cell,c in active:
                    indices=[self.keys[q] for q in c]
                    passed,error=cell_decision([self.points[i] for i in indices],self.tolerance)
                    lo,hi,bottom,top,depth=cell
                    if passed or depth>=MAX_DEPTH:
                        self.cells.append(dict(corners=indices[:4],samples=indices,center=indices[8],
                            checked=bool(passed),depth=depth,sampled_sep_error_mps=error,
                            reason='sampled interpolation check passed' if passed else
                                   'branch/admission/interpolation unresolved at refinement limit'))
                    else:
                        mid,half=(lo+hi)/2.,(bottom+top)/2.
                        pending.extend((a,b,c,d,depth+1) for a,b,c,d in
                            ((lo,mid,bottom,half),(mid,hi,bottom,half),
                             (lo,mid,half,top),(mid,hi,half,top)))
        result=self.result(limits,speeds)
        by_speed=defaultdict(list)
        for point in self.points:by_speed[point['speed_kmh']].append(point)
        result.update(columns=[dict(speed_kmh=v,points=sorted(rows,key=lambda p:p['load_g']),
            boundary=None,lower_boundary=None,sustained=[],boundary_status='global selection unresolved',
            boundary_reason='Stationary candidate columns; no physical envelope boundary certified',
            coverage_complete=False) for v,rows in sorted(by_speed.items())],
            execution=dict(method='columns',revision=REVISION,workers=WORKERS,
                column_tasks=self.column_tasks,reused_point_requests=self.reused_points,
                deterministic_acceptance=True,point_solver='joint stationary equations'))
        return result
