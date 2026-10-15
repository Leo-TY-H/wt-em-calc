"""Local stationary branch boundaries before baseline load sampling.

Alpha parameterization survives a load turning point. Every returned point is
replayed through stationary admission. This is numerical branch continuation,
not a census of disconnected trims or a global absence/selection certificate.
"""
import math

import numpy as np

from em_accuracy import turn_tolerance
from em_cancellation import check as check_cancel
from em_load_limits import ceiling
from em_pitch_response import PHYSICAL_REASONS, KINDS
from em_trim_numerics import UnevaluableTrial


class TrialUnavailable(UnevaluableTrial):
    """A conditional trial has no usable residual, not an absence proof."""


class Curve:
    def __init__(self, solver, speed, *, speed_interval=None, free_load=False):
        if solver.config.get('trim_solver_mode') == 'off':
            raise ValueError('Correction is off; boundary searches are unavailable')
        self.solver, self.speed = solver, float(speed)
        self.speed_interval = None if speed_interval is None else tuple(map(float,speed_interval))
        self.free_load=bool(free_load)
        if self.free_load and self.speed_interval is None:
            raise ValueError('Two-parameter continuation requires a speed interval')
        if self.speed_interval is not None and not (0.<self.speed_interval[0]<self.speed_interval[1]):
            raise ValueError('Level continuation requires an ordered positive speed interval')
        self.cap = ceiling(solver,self.speed_interval[1] if self.free_load else speed)
        self.indices = ([0, 1, 3] if solver.config.get('experimental_lateral_trim') == 'symmetric'
                        else list(range(5)))
        self.rows = [0, 1, 4] if len(self.indices) == 3 else list(range(5))
        self.calls = 0
        self.unverified_candidates = []
        self.predictors = {}

    def retain(self, point, requirement, **details):
        """Keep an admitted candidate without promoting boundary verification."""
        self.unverified_candidates.append(dict(point=point, boundary_verified=False,
            missing_requirement=requirement, coverage_complete=False, **details))

    def retain_event(self, point, kind, axis, seed, requirement, **interval):
        """Preserve an admitted offset-event witness, never an envelope label."""
        witness=dict(point)
        witness.pop('envelope_limit',None)
        self.retain(witness,requirement,
            active_event=dict(kind=kind,axis=axis,
                scope='Numerically closed offset active event and native admission only',
                correction=dict(getattr(self,'last_correction',{}))),
            continuation_seed={key:seed[key] for key in ('speed_kmh','load_g','alpha_deg',
                'sideslip_attitude_deg','stationary_request_indices') if key in seed},
            branch_connectivity_certified=False,earliest_event_certified=False,
            global_boundary_certified=False,**interval)

    def _solver_at(self, point, base=None):
        """Bind every conditional equation to the seed's prescribed sideslip."""
        base=self.solver if base is None else base
        current=getattr(base,'sideslip_attitude_deg',0.)
        angle=point.get('sideslip_attitude_deg',current)
        return base if angle==current else base.at_sideslip(angle)

    def correct(self, seed, *, alpha=None, elevator=None, event=None, events=None, load=None,
                max_nfev=40, max_evaluations=None, event_tolerance=2e-6):
        from em_stationary import BranchUnavailable
        try:
            return self._correct(seed,alpha=alpha,elevator=elevator,event=event,events=events,load=load,
                max_nfev=max_nfev,max_evaluations=max_evaluations,event_tolerance=event_tolerance)
        except (TrialUnavailable,BranchUnavailable,np.linalg.LinAlgError):
            return None

    def _correct(self, seed, *, alpha=None, elevator=None, event=None, events=None, load=None,
                 max_nfev=40, max_evaluations=None, event_tolerance=2e-6):
        """Solve balance plus one prescribed coordinate or active constraint."""
        check_cancel()
        s = self._solver_at(seed)
        level=self.speed_interval is not None
        extra_count=2 if self.free_load else 1
        constraints=sum(v is not None for v in (alpha,elevator,event,load))
        if self.free_load:
            if constraints or events is None or len(events)!=2:
                raise ValueError('Two-parameter continuation requires exactly two events')
        elif events is not None or constraints!=1:
            raise ValueError('Prescribe exactly one boundary coordinate or event')
        if level and load is not None:
            raise ValueError('Level-speed continuation already prescribes one-g load')
        x = np.array(seed['solution'], dtype=float)
        n = 1. if level and not self.free_load else seed['load_g']
        if alpha is not None:
            # A nonzero turn predictor avoids the level-flight derivative zero.
            if not level:n = min(self.cap, max(1.001, n + .15*(alpha-x[0])))
            x[0] = alpha
        if load is None and (not level or self.free_load):
            # Active events also solve for turn. At one-g, dload/dturn is zero;
            # give this same correction a nondegenerate initial predictor.
            # Prescribed load and one-g speed continuation keep their values.
            n=min(self.cap,max(1.001,n))
        if elevator is not None:x[3]=elevator
        if load is not None:n = load
        x[1] += math.degrees(math.acos(1./n)-math.acos(1./max(1.,seed['load_g'])))
        # Use the same joint equations as stationary point solves. Histories and
        # shafts remain unknowns here, avoiding a nested inner solve per probe.
        problem=None
        if hasattr(s,'core'):
            from em_stationary import JointProblem
            problem=JointProblem(s.core(),self.speed,n,x,seed.get('stationary_request_indices',()))
            initial=problem.initial.copy()
            if seed.get('stationary_coordinate_names')==problem.coordinate_names:
                initial=np.asarray(seed['stationary_coordinates']).copy()
                initial[:problem.history_start]=x[self.indices]
            bounds=(problem.lower,problem.upper)
            differences=problem.steps
            scale=problem.correction_scales
            limits=problem.correction_step_limits
            residual_scales=np.asarray(problem.correction_residual_scales).copy()
        else:
            initial=x[self.indices]
            bounds=(np.asarray(s.trim_bounds[0])[self.indices],np.asarray(s.trim_bounds[1])[self.indices])
            differences=np.array([.002,.002,.0002,.0002,.0002])[self.indices]
            scale=np.array([10.,30.,.2,.2,.2])[self.indices]
            limits=np.array([8.,20.,.4,.4,.4])[self.indices]
            residual_scales=np.array([2e-4,2e-4,5e-4,5e-4,5e-4])[self.rows]
        turn=math.sqrt(max(0.,n*n-1.));turn_upper=math.sqrt(self.cap**2-1.)
        parameters=([turn,seed.get('speed_kmh',self.speed)] if self.free_load else
                    [seed.get('speed_kmh',self.speed)] if level else [turn])
        parameter_lower=([0.,self.speed_interval[0]] if self.free_load else
                         [self.speed_interval[0]] if level else [0.])
        parameter_upper=([turn_upper,self.speed_interval[1]] if self.free_load else
                         [self.speed_interval[1]] if level else [turn_upper])
        z = np.r_[initial,parameters]
        lower = np.r_[bounds[0],parameter_lower]
        upper = np.r_[bounds[1],parameter_upper]
        if np.any(upper <= lower):return None
        z = np.clip(z, lower+1e-10, upper-1e-10)
        fixed=(0 if alpha is not None else self.indices.index(3) if elevator is not None
               else len(z)-1 if load is not None else None)
        if alpha is not None:z[0]=alpha
        if elevator is not None:z[self.indices.index(3)]=elevator
        if load is not None:z[-1]=math.sqrt(max(0.,load*load-1.))
        if np.any(z<lower) or np.any(z>upper):return None
        free=[i for i in range(len(z)) if i!=fixed]
        rows=list(range(len(z)-(fixed is not None)))
        def expand(y):
            q=z.copy();q[free]=y
            return q
        memo = {}

        def condition(q):
            request=(math.hypot(1.,q[-2]) if self.free_load else 1. if level else
                     load if load is not None else math.hypot(1.,q[-1]))
            return request,q[-1] if level else self.speed

        def value(q):
            key = tuple(q)
            if key not in memo:
                check_cancel()
                request,speed=condition(q)
                if problem is not None:
                    problem.load=request;problem.speed=speed/3.6
                    residual=problem.residual(q[:-extra_count]);v=problem.last
                    attitude=np.asarray(problem.attitude(q[:-extra_count]))
                else:
                    attitude=x.copy();attitude[self.indices]=q[:-extra_count]
                    if len(self.indices)==3:attitude[2]=attitude[4]=0.
                    v=s.operating_point(speed/3.6,request,attitude)
                    residual=v['residual'][self.rows]
                # Event callbacks may originate before the recovered seed was
                # selected. Carry the evaluated condition into those callbacks.
                v=dict(v,sideslip_attitude_deg=getattr(s,'sideslip_attitude_deg',0.))
                extra = ([f(v) for f in events] if self.free_load else
                         [(attitude[0]-alpha)/10. if alpha is not None else
                         attitude[3]-elevator if elevator is not None else
                         (math.hypot(1.,q[-1])-load)/10. if load is not None else event(v)])
                if not np.isfinite(extra).all() or not np.isfinite(residual).all():raise TrialUnavailable()
                memo[key] = (np.r_[residual, extra], v, attitude, request)
                self.calls += 1
            return memo[key]

        steps = np.r_[differences,[.0002,.002] if self.free_load else [.002] if level else [.0002]]
        parameter_scales=[5.,10.] if self.free_load else [10.] if level else [5.]
        parameter_limits=[5.,30.] if self.free_load else [30.] if level else [5.]
        from em_stationary_corrector import Corrector, PRODUCTION_NUMERICAL_TARGET
        def sample(y):
            r,v,_,_ = value(expand(y))
            return dict(v,residual=r[rows])
        def project(y,base):
            q=expand(y)
            # The trial's load belongs to its own geometry. Projection only
            # moves free attitude coordinates and never the prescribed alpha.
            problem.load,speed=condition(q);problem.speed=speed/3.6
            q[:-extra_count]=problem.project(q[:-extra_count],base)
            return q[free]
        corrector=Corrector(sample,(lower[free],upper[free]),steps[free],
            branch=problem.derivative_branch if problem is not None else lambda v:(),
            project=project if problem is not None else None,scale=np.r_[scale,parameter_scales][free],
            step_limit=np.r_[limits,parameter_limits][free],
            # Equation rows differ from free-coordinate indices when alpha,
            # elevator or load is eliminated. Remove the fixed event equation,
            # never a physical balance equation at the same coordinate index.
            residual_scale=np.r_[residual_scales,[event_tolerance]*extra_count][rows])
        if not np.isfinite(value(z)[0]).all():return None
        # Exact coordinate elimination is retained; only the common correction
        # implementation owns stencils, globalization and local refinement.
        # Only coordinate constraints share a predictor across calls. Arbitrary
        # event functions can change equations, so reuse is local to that solve.
        key=(fixed, tuple(free), tuple(seed.get('stationary_request_indices',())),
             getattr(s,'sideslip_attitude_deg',0.)) if fixed is not None else None
        plane=getattr(event,'stationary_plane_normal',None)
        if plane is not None and not level:
            key=('plane',tuple(free),tuple(seed.get('stationary_request_indices',())),
                 getattr(s,'sideslip_attitude_deg',0.))
        predictor=self.predictors.get(key) if key is not None else None
        if predictor is not None and plane is not None:
            # Only the sampling plane changes. Retain the balanced aircraft's
            # derivative model and replace that row with its analytic gradient.
            # Corrector still checks the new branch/state and refreshes a poor
            # prediction before recovery.
            gradient=np.zeros(len(z));units=event.stationary_plane_scale
            gradient[:len(self.indices)]=plane[self.indices]/units[self.indices]
            gradient[-1]=plane[5]/units[5]-plane[1]/units[1]*math.degrees(1./(1.+z[-1]**2))
            predictor=dict(predictor,matrix=predictor['matrix'].copy())
            predictor['matrix'][-1]=gradient[free]
        fit=corrector.solve(z[free],max_nfev=max_nfev,max_evaluations=max_evaluations,
            numerical_target=PRODUCTION_NUMERICAL_TARGET,reuse_jacobian=True,
            predictor=predictor)
        if key is not None and fit.predictor is not None:self.predictors[key]=fit.predictor
        z=expand(fit.x)
        r,v,x,n = value(z)
        self.last_correction=dict(force_error_g=v['force_error_g'],
            angular_error_rad_s2=float(max(abs(v['rate_residual']))),
            residual_max=float(max(abs(r[:-extra_count]))),
            constraint_residual=float(max(abs(r[-extra_count:]))))
        # Continuation is subject to the same balance gates as an ordinary
        # point. A second raw-norm target wastes work below model resolution.
        closed=(np.isfinite(r).all() and v['force_error_g'] <= 2e-4
                and max(abs(v['rate_residual'])) <= 5e-5
                and np.all(abs(r[:-extra_count]) <= residual_scales)
                and max(abs(r[-extra_count:])) <= event_tolerance)
        if problem is not None:
            # Replay, including restoring the load if the last call was a
            # derivative probe. Prepared propulsion depends on load and attitude.
            problem.load=n;problem.speed=z[-1]/3.6 if level else self.speed/3.6
            point=problem.finish(z[:-extra_count]);point['selected_candidates']=[]
            p=s.sample(dict(point,stationary_candidates=[point],stationary_attempts=[]))
        else:p = s.certify(z[-1] if level else self.speed,n,x,v)
        if not closed:
            if p.get('valid'):
                self.retain(p, 'Boundary correction did not meet its balance or constraint tolerance',
                            correction=dict(self.last_correction))
            return None
        if not p['converged']:return None
        if self.free_load and n>ceiling(s,p['speed_kmh']):
            if p.get('valid'):
                self.retain(p,'Admitted corner exceeds the chart load ceiling at its corrected speed')
            return None
        return p

    def events(self, point):
        s = self._solver_at(point)
        events = [('stall', None, lambda v:(v['stall_margin']-.002)/10.)]
        for name,fraction in zip(point.get('stationary_coordinate_names',()),
                                 point.get('stationary_coordinates',())):
            if not name.startswith('pitch:'):continue
            shaft=int(name.split(':')[1])
            for side in (0,1):
                def pitch_stop(v,shaft=shaft,side=side):
                    fraction=v['_stationary_pitch_fractions'][shaft]
                    return fraction if side==0 else 1.-fraction
                pitch_stop.stationary_distance=abs(fraction-side)
                pitch_stop.stationary_limit=dict(shaft=shaft,side='minimum' if side==0 else 'maximum')
                events.append(('propeller pitch',None,pitch_stop))
        if point.get('stationary_governor_mode')=='pitch_stops':
            propulsion=point.get('propulsion') or {}
            regulated=set()
            if getattr(s,'is_prop',False):
                for name,coordinate in zip(point.get('stationary_coordinate_names',()),
                                           point.get('stationary_coordinates',())):
                    if name.startswith('governor:') and 0.<coordinate<1.:
                        shaft=s.engine.properties['transmissions'][int(name.split(':')[1])]
                        regulated.update(link['index'] for link in shaft['engines'])
            for engine,(rpm,limit) in enumerate(zip(propulsion.get('engine_rpm',()),
                                                   propulsion.get('engine_rpm_threshold',()))):
                # A pitch stop changes the governor regime. The engine's
                # physical RPM limit, when reached, is an actual chart face.
                omega_limit=limit*math.pi/30.
                def rpm_limit(v,engine=engine,limit=omega_limit):
                    return 1.-v['propulsion']['state']['engines'][engine]['omega']/limit-5e-6
                # Target-RPM regulation makes this event locally constant.
                # Try variable physical faces before shooting that flat row.
                rpm_limit.stationary_distance=float('inf') if engine in regulated else abs(1.-rpm/limit)
                rpm_limit.stationary_engine_limit=dict(engine=engine,rpm_limit=limit)
                events.append(('engine RPM',None,rpm_limit))
        if s.config['structural_limits']:
            events.append(('wing force',None,lambda v:1.-max(v['maximum_wing_load_ratios'])-5e-5))
        if s.config['instructor']:
            def pitch(v):
                ctl=self._solver_at(v,s).instructor_at(v,v.get('speed',self.speed/3.6),v['equilibrium_load_g'])
                return ctl['envelope_margin']-2e-6 if ctl['converged'] else float('nan')
            events.append(('Instructor pitch',None,pitch))
        if point.get('pitch_response') is not None:
            from em_pitch_response import response
            events.append(('pitch response',None,lambda v:response(self._solver_at(v,s),v)['margin']-2e-6))
        axes = [1] if len(self.indices)==3 else range(3)
        for axis in axes:
            for side in (0,1):
                def control(v,axis=axis,side=side):
                    bounds=(self._solver_at(v,s).instructor_at(v,v.get('speed',self.speed/3.6),v['equilibrium_load_g'])['control_bounds']
                            if s.config['instructor'] else v['allocation']['bounds'])
                    return (1. if side==0 else -1.)*(v['allocation']['commands'][axis]-bounds[axis][side])-3e-5
                commands=point.get('commands')
                bounds=(point.get('instructor') or {}).get('control_bounds',point.get('control_bounds'))
                if commands is not None and bounds is not None:
                    control.stationary_distance=abs(commands[axis]-bounds[axis][side])
                events.append(('control',axis+2,control))
        return events

    def active(self, point, extent):
        for kind,axis,event in self.nearest_events(point):
            p=self.correct(point,event=event)
            if p is None or not p['valid']:continue
            if (point['alpha_deg']-1e-5 <= p['alpha_deg'] <= point['alpha_deg']+extent and
                    p['load_g'] >= point['load_g']-2e-5):
                return mark(p,kind,axis=axis,method='stationary balance and active constraint',
                            **event_details(event))
            else:
                self.retain_event(p,kind,axis,point,
                    'Admitted active event is outside the current continuation extent; branch connection and preceding limits unresolved',
                    requested_alpha_interval_deg=[point['alpha_deg']-1e-5,point['alpha_deg']+extent],
                    requested_minimum_load_g=point['load_g']-2e-5)
        return None

    def nearest_events(self,point):
        """Prioritize nearby physical faces; every result still passes all guards.

        This is local branch recovery, not global event ordering. The caller
        checks the intervening branch before publishing an edge.
        """
        margins={'stall':abs(point.get('stall_margin_deg',float('inf')))/10.,
            'wing force':abs(1.-max(point.get('wing_load_ratios',[float('inf')]))),
            'Instructor pitch':abs((point.get('instructor') or {}).get('envelope_margin',float('inf'))),
            'pitch response':abs((point.get('pitch_response') or {}).get('margin',float('inf'))),
            'control':abs(point.get('authority_margin',float('inf')))}
        return sorted(self.events(point),key=lambda item:
            getattr(item[2],'stationary_distance',margins.get(item[0],float('inf'))))


def event_details(event):
    limit=getattr(event,'stationary_limit',None)
    if limit is not None:return {'propeller_pitch_limit':limit}
    limit=getattr(event,'stationary_engine_limit',None)
    return {'engine_rpm_limit':limit} if limit is not None else {}


def mark(point,kind,**details):
    point=dict(point)
    point['envelope_limit']=dict(kind=kind,axis=details.pop('axis',None),limiting_load_g=point['load_g'],
        certificate_scope='local numerical branch boundary; global coverage unresolved',**details)
    return point


def bracket(curve,low,high):
    """Refine the first observed rejection, checking all physical guards."""
    crossed={KINDS[r] for r in high['reasons'] if r in KINDS}
    for kind,axis,event in curve.nearest_events(low):
        if kind not in crossed:continue
        p=curve.correct(low,event=event)
        if p is None or not p['valid']:continue
        if low['alpha_deg']<=p['alpha_deg']<=high['alpha_deg']:
            return mark(p,kind,axis=axis,method='stationary balance and active constraint')
        else:
            curve.retain_event(p,kind,axis,low,
                'Admitted active event is outside the observed physical bracket; branch connection and preceding limits unresolved',
                requested_alpha_interval_deg=[low['alpha_deg'],high['alpha_deg']])
    for _ in range(28):
        if not high['reasons'] or not set(high['reasons'])<=PHYSICAL_REASONS:
            curve.retain(low, 'Upper bracket endpoint is not a purely physical rejection')
            return None
        if (abs(high['turn_dps']-low['turn_dps'])<turn_tolerance(curve.solver.config['sep_tolerance_mps']) and
                abs(high['ps_mps']-low['ps_mps'])<curve.solver.config['sep_tolerance_mps']):
            kinds=sorted({KINDS[r] for r in high['reasons']})
            return mark(low,kinds[0],method='stationary fixed-alpha physical rejection bracket',
                active_constraints=kinds,alpha_bracket_deg=[low['alpha_deg'],high['alpha_deg']],
                limiting_load_interval_g=sorted([low['load_g'],high['load_g']]),
                rejected_endpoint={k:high[k] for k in ('load_g','alpha_deg','reasons','force_error_g','angular_error_rad_s2')})
        p=curve.correct(low,alpha=(low['alpha_deg']+high['alpha_deg'])*.5)
        if p is None:
            curve.retain(low, 'Physical boundary bracket correction failed; limiting face and earlier maximum remain unresolved')
            return None
        if p['valid']:low=p
        else:high=p
    curve.retain(low, 'Physical boundary bracket refinement budget exhausted')
    return None


def peak(curve,left,middle,right,*interior):
    """Contract a sampled maximum bracket without a smooth scalar fit.

    Exact load ties remain a plateau of candidates. They never choose which
    half to discard, and a broad plateau cannot manufacture a localized fold.
    All corrections use the original equilibrium equations. The final bracket
    is local numerical evidence, not a bound on unobserved extrema or rounding.
    """
    points=sorted({p['alpha_deg']:p for p in (left,middle,right,*interior)}.values(),
                  key=lambda p:p['alpha_deg'])

    def retain(requirement):
        top=max(p['load_g'] for p in points)
        candidates=[p for p in points if p['load_g']==top]
        details=dict(alpha_interval_deg=[points[0]['alpha_deg'],points[-1]['alpha_deg']],
                     sampled_load_interval_g=[min(p['load_g'] for p in points),top],
                     sampled_peak_candidates=len(candidates),numerical_error_certified=False)
        for point in candidates:curve.retain(point,requirement,**details)

    for iteration in range(33):
        check_cancel()
        largest=max(p['load_g'] for p in points)
        tops=[p for p in points if p['load_g']==largest]
        first,last=tops[0],tops[-1]
        below=[p for p in points if p['alpha_deg']<first['alpha_deg'] and p['load_g']<largest]
        above=[p for p in points if p['alpha_deg']>last['alpha_deg'] and p['load_g']<largest]
        if not below or not above:
            retain('Balanced lower-load neighbors on both sides of maximum unresolved')
            return None
        low,high=below[-1],above[0]
        neighborhood=[low,*tops,high]
        turns=[p['turn_dps'] for p in neighborhood]
        powers=[p['ps_mps'] for p in neighborhood]
        # Localize chart outputs, not an invisible alpha decimal. Multiple tied
        # maxima still describe a plateau, not a unique localized fold.
        if (len(tops)==1 and
                max(turns)-min(turns)<turn_tolerance(curve.solver.config['sep_tolerance_mps']) and
                max(powers)-min(powers)<curve.solver.config['sep_tolerance_mps']):
            center=.5*(first['alpha_deg']+last['alpha_deg'])
            top=min(tops,key=lambda p:abs(p['alpha_deg']-center))
            return mark(top,'trim fold',method='stationary fixed-alpha sampled maximum bracket',
                alpha_bracket_deg=[low['alpha_deg'],high['alpha_deg']],
                neighboring_loads_g=[low['load_g'],high['load_g']],
                sampled_plateau_alpha_deg=[first['alpha_deg'],last['alpha_deg']],
                global_maximum_certified=False,numerical_error_certified=False,
                localization_scope='Sampled local bracket; unsampled extrema and rounding error unresolved')
        if iteration==32:
            retain('Load-maximum localization budget exhausted; plateau or unsampled segment unresolved')
            return None
        # Bisect the wider gap outside the complete set of tied best samples.
        # On a plateau this approaches its two edges, preserving its width.
        gaps=((low,first),(last,high))
        a,b=max(gaps,key=lambda pair:pair[1]['alpha_deg']-pair[0]['alpha_deg'])
        alpha=.5*(a['alpha_deg']+b['alpha_deg'])
        if alpha in (a['alpha_deg'],b['alpha_deg']):
            retain('Load-maximum bracket cannot contract at floating-point resolution')
            return None
        seed=min(points,key=lambda p:abs(p['alpha_deg']-alpha))
        point=curve.correct(seed,alpha=alpha)
        if point is None or not point['valid']:
            retain('Load-maximum localization correction failed or encountered a physical rejection')
            return None
        points.append(point)
        points.sort(key=lambda p:p['alpha_deg'])


def finish_edge(curve,history,edge):
    """A physical face after the load maximum must not hide that maximum."""
    low=history[-1]
    mid=curve.correct(low,alpha=(low['alpha_deg']+edge['alpha_deg'])*.5)
    if mid is None:
        curve.retain(edge, 'Midpoint verification correction failed')
        return None
    if not mid['valid']:
        replacement=bracket(curve,low,mid)
        if replacement is None:curve.retain(edge, 'Earlier rejected midpoint has no verified limiting bracket')
        return replacement
    near_alpha=max(mid['alpha_deg'],edge['alpha_deg']-.01)
    near=mid if near_alpha==mid['alpha_deg'] else curve.correct(mid,alpha=near_alpha)
    if near is None or not near['valid']:
        curve.retain(edge, 'Interior-side boundary verification failed')
        return None
    if near['load_g']>edge['load_g'] or mid['load_g']>edge['load_g']:
        maximum=peak(curve,history[-2] if len(history)>1 else low,mid,edge,near,low)
        if maximum is None:
            curve.retain(edge, 'Physical face is admitted; intervening load maximum remains unresolved')
        return maximum
    if low['load_g']>edge['load_g']:
        curve.retain(edge, 'Earlier branch point has greater load; intervening maximum unresolved')
        return None
    return edge


def find_boundary(solver,speed,interior, *, evidence=None):
    """Follow one admitted branch and retain unresolved candidate evidence."""
    if not interior['valid']:return None
    curve=Curve(solver,speed)
    try:return follow_branch(curve,interior)
    finally:
        if evidence is not None:evidence.extend(curve.unverified_candidates)


def from_bracket(solver,speed,low,high, *, candidate_kinds=None,max_iterations=24,
                 continuation=True,evidence=None):
    """Consume a sampler bracket through the shared stationary equations only."""
    if not low.get('valid') or high['load_g']<=low['load_g']:return None
    curve=Curve(solver,speed)
    search_cap=curve.cap
    curve.cap=min(curve.cap,high['load_g'])
    recovered_branch=None
    try:
        slip=low.get('sideslip_attitude_deg',getattr(solver,'sideslip_attitude_deg',0.))
        if high.get('sideslip_attitude_deg',slip)!=slip:
            curve.retain(low,'Boundary endpoints have different sideslip conditions')
            return None
        if (low.get('stationary_request_indices') is not None and
                high.get('stationary_request_indices') is not None and
                low['stationary_request_indices']!=high['stationary_request_indices']):
            curve.retain(low,'Boundary endpoints do not identify the same requested-RPM branch')
            return None
        # A sampler's coarse upper trial can overshoot several constraints.
        # Contract that interval with cheap fixed-load corrections before
        # solving every augmented active face. A failed correction only shrinks
        # the search interval; it never supplies a physical rejection.
        if continuation and hasattr(solver,'solve') and not high.get('converged'):
            # Hard pitch bounds cannot produce a balanced rejected endpoint.
            # Close the nearest such face before bisecting failed load solves.
            for kind,axis,event in curve.nearest_events(low):
                if kind!='propeller pitch':continue
                if candidate_kinds is not None and kind not in candidate_kinds:continue
                if event.stationary_distance>.05:continue
                edge=curve.correct(low,event=event)
                if edge is not None and edge['valid'] and low['load_g']<=edge['load_g']<=high['load_g']:
                    edge=mark(edge,kind,axis=axis,method='stationary requested-RPM pitch bound',
                              **event_details(event))
                    result=finish_edge(curve,[low],edge)
                    if result is not None:return result
            view=curve._solver_at(low)
            upper=math.sqrt(max(0.,high['load_g']**2-1.))
            for _ in range(12):
                u=math.sqrt(max(0.,low['load_g']**2-1.))
                middle=(u+upper)*.5
                if middle in (u,upper):break
                from em_accuracy import load_interval_resolved
                if load_interval_resolved(speed,math.hypot(1.,u),math.hypot(1.,upper),solver.config):break
                guess=list(low['solution'])
                guess[1]+=math.degrees(math.atan(middle)-math.atan(u))
                from em_sampling import continuation_initial
                trial=view.solve(speed,math.hypot(1.,middle),continuation_initial(view,low,guess),exhaustive=False,quick=True)
                if trial['converged'] and trial['reasons'] and set(trial['reasons'])<=PHYSICAL_REASONS:
                    high=trial
                    break
                if trial['valid']:low=trial
                else:upper=middle
        if (high.get('converged') and 'post-stall' in high.get('reasons',())
                and set(high['reasons'])<=PHYSICAL_REASONS):
            # A rejected descending lift-curve root does not bound a different
            # admitted ascending root at the same load. Check that load from
            # the admitted branch before using the rejection as an upper bound.
            alternate=curve.correct(low,load=high['load_g'],max_nfev=12)
            if alternate is not None and alternate['valid']:
                recovered_branch=dict(load_g=high['load_g'],
                    rejected_alpha_deg=high['alpha_deg'],admitted_alpha_deg=alternate['alpha_deg'],
                    original_load_bracket_g=[low['load_g'],high['load_g']],
                    method='Admitted continuation at the rejected root load')
                curve.cap=search_cap
                low=alternate
        if recovered_branch is not None:
            result=follow_branch(curve,low)
        elif (high.get('converged') and high.get('reasons') and
                set(high['reasons'])<=PHYSICAL_REASONS and high['alpha_deg']>low['alpha_deg']):
            # Both endpoints are already balanced. Interpolate their physical
            # margin with stationary point correction before shooting a new
            # augmented active-event system. No numerical failure forms a face.
            result=None
            if hasattr(solver,'solve'):
                from em_constraint_bracket import refine
                result=refine(curve._solver_at(low),speed,low,high)
            if result is None:
                edge=bracket(curve,low,high)
                result=finish_edge(curve,[low],edge) if edge is not None else None
            if result is None and continuation:result=follow_branch(curve,low)
        elif continuation or 'trim fold' in (candidate_kinds or ()):
            # Continuation already owns bounded active-face recovery when a
            # balanced next point is unavailable. Do not shoot the same full
            # face set here and then repeat it from the identical seed. A fold
            # hint also needs branch continuation; it has no active-face row.
            result=follow_branch(curve,low)
        else:
            result=None
            predicted=high.get('continuation_seed',low)
            if not low['load_g']<=predicted.get('load_g',0.)<=high['load_g']:predicted=low
            candidates=curve.nearest_events(predicted)
            expected=high.get('continuation_limit_kind')
            if expected is not None:
                # An interpolated hint carries old physical margins. Those
                # stale distances must not prioritize six unrelated control
                # stops ahead of the observed neighboring active constraint.
                candidates=sorted(candidates,key=lambda item:item[0]!=expected)
            for kind,axis,event in candidates:
                if candidate_kinds is not None and kind not in candidate_kinds:continue
                point=curve.correct(predicted,event=event,max_nfev=max_iterations)
                if point is None and predicted is not low:
                    point=curve.correct(low,event=event,max_nfev=max_iterations)
                if point is None or not point['valid']:continue
                if (low['load_g']<=point['load_g']<=high['load_g'] and point['alpha_deg']>=low['alpha_deg']):
                    edge=mark(point,kind,axis=axis,method='stationary bracket active constraint',
                              **event_details(event))
                    result=finish_edge(curve,[low],edge)
                    if result is not None:break
                else:
                    curve.retain_event(point,kind,axis,low,
                        'Admitted active event is outside the requested load bracket or direction; branch connection and preceding limits unresolved',
                        requested_load_interval_g=[low['load_g'],high['load_g']],
                        requested_minimum_alpha_deg=low['alpha_deg'])
        if result is None:return None
        kind=result.get('envelope_limit',{}).get('kind')
        upper=search_cap if recovered_branch is not None else high['load_g']
        # A localized fold found before a requested physical face is the
        # branch endpoint. A face hint must not discard that earlier maximum.
        if (not low['load_g']<=result['load_g']<=upper or kind is None or
                candidate_kinds is not None and kind not in candidate_kinds and kind!='trim fold'):
            curve.retain(result,'Localized candidate does not satisfy the requested boundary bracket or kind')
            return None
        if recovered_branch is not None:
            result['envelope_limit']['rejected_branch_recovery']=recovered_branch
        return result
    finally:
        if evidence is not None:evidence.extend(curve.unverified_candidates)


def level_edge(solver,low_speed,column,low_point=None,kind=None):
    """Balance one-g flight and an active constraint with speed as an unknown."""
    high=column.get('boundary')
    if not high or not high.get('valid') or low_speed>=column['speed_kmh']:return None
    kind=kind or high.get('envelope_limit',{}).get('kind')
    if kind not in ('stall','control','Instructor pitch','pitch response','propeller pitch'):return None
    angle=high.get('sideslip_attitude_deg',getattr(solver,'sideslip_attitude_deg',0.))
    view=solver.at_sideslip(angle) if angle!=getattr(solver,'sideslip_attitude_deg',0.) else solver
    # Reuse the observed level-flight branch before solving a joint speed/event
    # system. The accepted edge is bracketed by balanced physical states at
    # chart resolution, not by failed optimizations or a tiny event norm.
    if low_point is not None and low_point.get('load_g')==1.:
        from em_accuracy import speed_tolerance
        good=next((p for p in column.get('points',()) if p['load_g']==1. and p['valid']),None)
        if good is None and high['load_g']==1.:good=high
        if good is not None:
            bad=low_point;left=float(low_speed)
            for _ in range(16):
                physical=(bad.get('converged') and bad.get('reasons') and
                    set(bad['reasons'])<=PHYSICAL_REASONS)
                if not physical:break
                if (good['speed_kmh']-bad['speed_kmh']<=speed_tolerance(view.config)
                        and abs(good['ps_mps']-bad['ps_mps'])<=view.config['sep_tolerance_mps']):
                    active=sorted({KINDS[r] for r in bad['reasons']})
                    point=mark(good,active[0],method='stationary balanced level-speed physical bracket',
                        active_constraints=active,limiting_speed_interval_kmh=[bad['speed_kmh'],good['speed_kmh']],
                        rejected_endpoint={k:bad[k] for k in ('speed_kmh','load_g','reasons','force_error_g','angular_error_rad_s2')})
                    point['surface_sample']=True
                    return point
                speed=(left+good['speed_kmh'])*.5
                if speed in (left,good['speed_kmh']):break
                trial=view.solve(speed,1.,good,exhaustive=False,quick=True)
                if trial['valid']:good=trial
                else:bad=trial;left=speed
    curve=Curve(view,column['speed_kmh'],speed_interval=(low_speed,column['speed_kmh']))
    answers=[]
    try:
        for candidate,axis,event in curve.events(high):
            if candidate!=kind:continue
            point=curve.correct(high,event=event,max_nfev=60)
            if point is not None and point['valid']:
                answers.append(mark(point,kind,axis=axis,method='stationary one-g speed and active constraint',
                                    **event_details(event)))
        if not answers:return None
        edge=max(answers,key=lambda p:p['speed_kmh'])
        for point in answers:
            if point is not edge:curve.retain(point,'Competing level-speed constraint awaits branch localization')
        middle=.5*(edge['speed_kmh']+column['speed_kmh'])
        interior=curve.correct(edge,event=lambda v:(v['speed']*3.6-middle)/10.,max_nfev=40)
        if interior is None or not interior['valid']:
            curve.retain(edge,'Level-speed boundary interior verification unresolved')
            return None
        edge['surface_sample']=True
        edge['envelope_limit'].update(limiting_speed_interval_kmh=[low_speed,column['speed_kmh']],
            interior_speed_kmh=interior['speed_kmh'],global_boundary_certified=False)
        return edge
    finally:
        solver.__dict__.setdefault('stationary_boundary_evidence',[]).extend(curve.unverified_candidates)


def held_pitch(solver,held,speed,initial):
    """Close the held-pitch event without a second aircraft optimizer."""
    curve=Curve(solver,speed);curve.cap=min(curve.cap,math.sqrt(2.))
    seed=dict(solution=list(initial[:5]),load_g=float(initial[5]),speed_kmh=speed,
              stationary_request_indices=[0]*len(solver.engine.properties['transmissions']) if solver.is_prop else [])
    def event(value):
        control=held.controller(value,speed/3.6)
        return control['margin']*10. if control['converged'] else float('nan')
    try:
        point=curve.correct(seed,event=event,max_nfev=70,event_tolerance=2e-5)
        return np.r_[point['solution'],point['load_g']] if point is not None and point['valid'] else None
    finally:
        solver.__dict__.setdefault('stationary_boundary_evidence',[]).extend(curve.unverified_candidates)


def follow_branch(curve,interior):
    solver=curve.solver
    if curve.cap<=interior['load_g']+1e-8:return None
    history=[interior];step=3.;direct_seed=None
    for _ in range(64):
        low=history[-1]
        target=min(solver.trim_bounds[1][0],low['alpha_deg']+step)
        if target<=low['alpha_deg']:
            curve.retain(low, 'Continuation reached an angle search bound; no physical boundary established')
            return None
        high=curve.correct(low,alpha=target)
        unresolved=(high is not None and not high['valid'] and
                    (not high['reasons'] or not set(high['reasons'])<=PHYSICAL_REASONS))
        if unresolved:
            # Only the declared chart faces can form this physical bracket.
            # Other rejections, including failed closure, remain unresolved;
            # every later proposal still has to pass full local admission.
            curve.retain(low,'Continuation trial has no usable physical boundary bracket',
                rejected_endpoint={k:high[k] for k in ('load_g','alpha_deg','reasons',
                    'force_error_g','angular_error_rad_s2','stationary_residual') if k in high})
        if high is None or unresolved:
            # Hard control bounds can prevent a balanced rejected trial. Solve
            # all active faces directly; each answer must pass every other guard.
            # Their equations depend on low, not the attempted alpha step.
            # With low unchanged, halving the step only narrows acceptance;
            # retrying the identical face/cap corrections cannot add a candidate.
            if low is not direct_seed:
                direct_seed=low
                edge=curve.active(low,step)
                if edge is not None:
                    return finish_edge(curve,history,edge)
                cap=curve.correct(low,load=curve.cap)
                if (cap is not None and cap['valid'] and low['alpha_deg']<=cap['alpha_deg']<=target):
                    # Search/plot caps are not physical limits; the sampler labels them.
                    return cap
            step*=.5
            if step<.01:
                curve.retain(low, 'Continuation stalled; the last admitted interior point is not a localized boundary')
                return None
            continue
        if not high['valid']:
            edge=bracket(curve,low,high)
            return finish_edge(curve,history,edge) if edge is not None else None
        if high['load_g']<low['load_g']:
            if len(history)<2:
                curve.retain(low, 'Initial continuation direction decreases load; preceding maximum unresolved')
                curve.retain(high, 'Admitted decreasing-load branch point; preceding maximum unresolved')
                return None
            return peak(curve,history[-2],low,high)
        history.append(high)
        step=min(3.,step*1.5)
    curve.retain(history[-1], 'Continuation budget exhausted; boundary remains unlocalized')
    return None
