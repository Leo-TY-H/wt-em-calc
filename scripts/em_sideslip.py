import copy

MAX_SIDESLIP_DEG=2.
SEARCH_ANGLES=(.001,.003,.01,.03,.1,.3,1.,1.5)


def recover_local_sideslip(solver, speed, load, failed):
    """Try nearby balanced branches before expensive independent restarts.

    Near zero roll rate, the native wing-coupling blend can make the
    coordinated residual poorly conditioned. A small sideslip perturbation
    supplies a different starting branch. The quick corrector still certifies
    the complete aircraft, propulsion and Instructor with the usual tolerances.
    Failure here leaves the existing exhaustive recovery available.
    """
    if not solver.config.get('sideslip_recovery',True):return None  # zero-sideslip policy (march chart)
    if solver.config.get('experimental_lateral_trim','full')!='full':return None
    if (not solver.is_prop or not solver.engine.automatic or failed['converged'] or
            failed['force_error_g'] > .015 or failed['angular_error_rad_s2'] > .008 or
            failed['stall_margin_deg'] <= 1. or failed['authority_margin'] <= .02 or
            max(failed['wing_load_ratios']) >= .98):
        return None
    origin = failed.get('sideslip_attitude_deg', 0.)
    attempts = 0
    for magnitude in SEARCH_ANGLES[:4]:
        candidates = []
        for sign in (1., -1.):
            angle = origin + sign*magnitude
            if abs(angle) > MAX_SIDESLIP_DEG:continue
            point = solver.at_sideslip(angle).solve(speed, load, failed['solution'],
                                                   exhaustive=False, quick=True)
            attempts += 1
            if point['valid'] and abs(point['sideslip_deg']) <= MAX_SIDESLIP_DEG:
                candidates.append(point)
        if candidates:
            point = min(candidates, key=lambda p:(abs(p['sideslip_deg']), -p['ps_mps']))
            point['recovery_method'] = 'balanced local sideslip correction'
            point['sideslip_recovery'] = dict(max_abs_deg=MAX_SIDESLIP_DEG, attempts=attempts,
                policy='zero sideslip preferred; nearby fully certified correction; not a global optimum')
            return point
    return None


def needs_sideslip_search(point):
    if point.get('model_experiments',{}).get('lateral_trim','full')!='full':return False
    if point['valid']:return False
    if not point['converged']:return True
    if set(point['reasons'])=={'control authority'}:return True
    ctl=point.get('instructor')
    return bool(set(point['reasons'])=={'Instructor command limit'} and ctl and
                ctl['converged'] and ctl['limiting']=='control authority with auto trim')


def _stationary_fixed_alpha(solver,speed,load,failed,starts):
    """Discover fixed-alpha/free-beta candidates with the shared joint method.

    Sideslip is an added coordinate, replacing the prescribed alpha coordinate.
    Histories and shafts remain explicit joint unknowns. Search starts are
    proposals only; the common evaluator admits every returned aircraft state.
    """
    import itertools
    import numpy as np
    from em_cancellation import check as check_cancel
    from em_stationary import JointProblem, prepare
    from em_stationary_corrector import Corrector
    if solver.config.get('trim_solver_mode') == 'off':
        raise ValueError('Correction is off; sideslip correction is unavailable')
    candidates=[];attempts=[]
    for alpha,beta in starts:
        check_cancel()
        view=solver.core() if hasattr(solver,'core') else copy.copy(solver)
        view.sideslip_attitude_deg=beta
        view._sideslip_solvers={}
        attitude=list(failed['solution']);attitude[0]=alpha
        try:
            requests=prepare(view,speed/3.6,load,attitude)[1] if view.is_prop else []
            branches=itertools.product(*[range(len(r['candidates'])) if r['governed'] else [0]
                                         for r in requests])
            for branch in branches:
                check_cancel()
                problem=JointProblem(view,speed,load,attitude,branch)
                initial=problem.initial.copy()
                if (failed.get('stationary_coordinate_names')==problem.coordinate_names
                        and tuple(failed.get('stationary_request_indices',()))==branch):
                    initial=np.asarray(failed['stationary_coordinates'],dtype=float).copy()
                    initial[:problem.history_start]=np.asarray(attitude)[problem.attitude_indices]
                initial[0]=alpha
                def expand(z):
                    view.sideslip_attitude_deg=float(z[-1])
                    q=initial.copy();q[1:]=z[:-1]
                    return q
                def sample(z):return problem.sample(expand(z))
                def project(z,base):
                    q=problem.project(expand(z),base)
                    return np.r_[q[1:],z[-1]]
                correction=Corrector(sample,
                    (np.r_[problem.lower[1:],-1.95],np.r_[problem.upper[1:],1.95]),
                    np.r_[problem.steps[1:],.002],branch=problem.derivative_branch,
                    project=project,scale=np.r_[problem.correction_scales[1:],.2],
                    step_limit=np.r_[problem.correction_step_limits[1:],.2],
                    residual_scale=problem.correction_residual_scales)
                fit=correction.solve(np.r_[initial[1:],beta],max_nfev=80)
                point=problem.finish(expand(fit.x))
                point['numerical_message']=fit.message
                candidates.append(point)
                attempts.append(dict(alpha=alpha,sideslip=beta,requests=list(branch),
                    local_valid=point['local_valid'],termination=fit.termination,
                    nfev=fit.nfev,evaluations=fit.evaluations))
                if not point['local_valid'] or abs(point['sideslip_deg'])>MAX_SIDESLIP_DEG:continue
                result=dict(point,stationary_candidates=list(candidates),stationary_attempts=list(attempts),
                    selected_candidates=[],recovery_method='shared fixed-alpha sideslip correction',
                    sideslip_recovery=dict(max_abs_deg=MAX_SIDESLIP_DEG,attempts=len(attempts),
                        policy='Finite fixed-alpha/free-beta candidate search; selection and coverage unresolved'))
                return solver.sample(result) if hasattr(solver,'sample') else result
        except (ValueError,ArithmeticError,np.linalg.LinAlgError) as error:
            attempts.append(dict(alpha=alpha,sideslip=beta,error=str(error)))
    # Preserve unsuccessful discovery evidence without promoting it to absence.
    failed.setdefault('stationary_sideslip_candidates',[]).extend(candidates)
    failed.setdefault('stationary_sideslip_attempts',[]).extend(attempts)
    return None


def recover_fixed_alpha(solver,speed_kmh,load,failed):
    if solver.config.get('trim_solver_mode') == 'off':
        raise ValueError('Correction is off; sideslip correction is unavailable')
    if solver.config.get('experimental_lateral_trim','full')!='full':return None
    import numpy as np
    from scipy.optimize import least_squares
    if failed['converged'] or failed['force_error_g']>.015 or failed['angular_error_rad_s2']>.008:return None
    if solver.config.get('trim_solver_mode') == 'stationary':
        alpha=failed['solution'][0]
        return _stationary_fixed_alpha(solver,speed_kmh,load,failed,
            ((alpha,0.),(alpha,.2),(alpha,-.2),
             (round(alpha,2)+.0001,.2),(round(alpha,2)-.0001,-.2),
             (round(alpha,2)+.0001,1.),(round(alpha,2)-.0001,-1.)))
    alpha=failed['solution'][0];view=copy.copy(solver)
    view.instructor_boundaries={};view.instructor_trim_entries={};view._sideslip_solvers={}
    def value(z,frozen=None):
        view.sideslip_attitude_deg=float(z[4])
        return view.operating_point(speed_kmh/3.6,load,[alpha,*z[:4]],propulsion_override=frozen)
    def fun(z):return value(z)['residual']
    def jac(z):
        base=value(z)
        frozen=base['propulsion'] if solver.is_prop and solver.engine.automatic and base['propulsion']['converged'] else None
        if frozen is not None:base=value(z,frozen)
        columns=[]
        for i,h in enumerate([.002,.0002,.0002,.0002,.002]):
            for factor in (1.,.5,-.5,2.,-1.,.1,-.1):
                q=z.copy();q[i]+=h*factor;v=value(q,frozen)
                if solver.derivative_branch(v)==solver.derivative_branch(base):break
            columns.append((v['residual']-base['residual'])/(h*factor))
        return np.column_stack(columns)
    for alpha_offset,beta in ((0.,0.),(0.,.2),(0.,-.2),
            (round(failed['solution'][0],2)+.0001-failed['solution'][0],.2),
            (round(failed['solution'][0],2)-.0001-failed['solution'][0],-.2),
            (round(failed['solution'][0],2)+.0001-failed['solution'][0],1.),
            (round(failed['solution'][0],2)-.0001-failed['solution'][0],-1.)):
        alpha=failed['solution'][0]+alpha_offset
        fit=least_squares(fun,[*failed['solution'][1:],beta],jac=jac,
            bounds=([-15.,-1.,-1.,-1.,-1.95],[89.7,1.,1.,1.,1.95]),
            x_scale=[30.,.2,.2,.2,.2],max_nfev=20,ftol=1e-10,xtol=1e-9,gtol=1e-10)
        v=value(fit.x)
        if v['force_error_g']>2e-4 or max(abs(v['rate_residual']))>5e-5 or v['history_error']>2e-4:continue
        actual=solver.at_sideslip(float(fit.x[4]))
        point=actual.solve(speed_kmh,load,[alpha,*fit.x[:4]],exhaustive=False)
        if point['valid'] and abs(point['sideslip_deg'])<=MAX_SIDESLIP_DEG:
            point['recovery_method']='balanced fixed-alpha sideslip correction'
            point['sideslip_recovery']=dict(max_abs_deg=MAX_SIDESLIP_DEG,
                policy='zero sideslip preferred; checked bounded correction; not a global optimum')
            return point
    return None


def recover_scalar_sideslip(solver,speed,load,failed):
    if not solver.config.get('sideslip_recovery',True):return None  # zero-sideslip policy (march chart)
    if solver.config.get('trim_solver_mode') == 'off':
        raise ValueError('Correction is off; sideslip correction is unavailable')
    if solver.config.get('experimental_lateral_trim','full')!='full':return None
    import math
    import numpy as np
    from scipy.optimize import brentq
    if failed['converged'] or failed['force_error_g']>.05 or failed['angular_error_rad_s2']>.02:return None
    if solver.config.get('trim_solver_mode') == 'stationary':
        alpha=failed['solution'][0]
        # Beta is corrected jointly rather than solving another inner system
        # at every scalar bracket probe. Nearby alpha seeds remain discovery.
        return _stationary_fixed_alpha(solver,speed,load,failed,
            ((alpha,0.),(round(alpha,2)+.0001,.03),(round(alpha,2)-.0001,-.03)))
    view=copy.copy(solver)
    for alpha in (failed['solution'][0],round(failed['solution'][0],2)+.0001,round(failed['solution'][0],2)-.0001):
        known={};accepted=None;root_search=False;checked=set()
        def certify(beta):
            nonlocal accepted
            if beta in checked:return
            checked.add(beta)
            z,residual=known[beta]
            if abs(residual)>2e-4:return
            point=solver.at_sideslip(beta).solve(speed,load,[alpha,*z],exhaustive=False)
            if point['valid'] and abs(point['sideslip_deg'])<=MAX_SIDESLIP_DEG:
                point['recovery_method']='balanced scalar sideslip correction'
                point['sideslip_recovery']=dict(max_abs_deg=MAX_SIDESLIP_DEG,
                    policy='zero sideslip preferred; checked bounded correction; not a global optimum')
                accepted=point
        def at(beta):
            nonlocal accepted
            if beta in known:return known[beta]
            view.sideslip_attitude_deg=float(beta)
            z=(min(known.items(),key=lambda item:abs(item[0]-beta))[1][0].copy()
               if known else np.array(failed['solution'][1:]))
            def value(q,frozen=None):return view.operating_point(speed/3.6,load,[alpha,*q],propulsion_override=frozen)
            v=value(z)
            for iteration in range(8):
                residual=v['residual'][1:]
                if abs(residual[0])<2e-5 and max(abs(v['rate_residual']))<5e-6:break
                frozen=v['propulsion'] if solver.is_prop and solver.engine.automatic and v['propulsion']['converged'] else None
                base=value(z,frozen) if frozen is not None else v
                cols=[]
                for i,h in enumerate((.002,.0002,.0002,.0002)):
                    q=z.copy();q[i]+=h;vv=value(q,frozen)
                    cols.append((vv['residual'][1:]-base['residual'][1:])/h)
                delta=np.linalg.lstsq(np.column_stack(cols),-residual,rcond=None)[0]
                delta/=max(1.,float(np.max(abs(delta)/[10.,.2,.2,.2])))
                advanced=False
                for factor in (1.,.5,.25,.1):
                    q=np.clip(z+factor*delta,[-15.,-1.,-1.,-1.],[89.7,1.,1.,1.]);vv=value(q)
                    if np.linalg.norm(vv['residual'][1:])<np.linalg.norm(residual):
                        z,v=q,vv;advanced=True;break
                if not advanced:break
            if abs(v['residual'][1])>2e-4 or max(abs(v['rate_residual']))>5e-5 or v['history_error']>2e-4:
                raise ValueError('Auxiliary balance unresolved')
            known[beta]=(z,float(v['residual'][0]))
            if not root_search:certify(beta)
            return known[beta]
        for beta in (0.,.03,-.03,.3,-.3,1.,-1.,1.95,-1.95):
            try:at(beta)
            except ValueError:continue
            if accepted:return accepted
            ordered=sorted(known)
            for left,right in zip(ordered,ordered[1:]):
                if known[left][1]*known[right][1]>0:continue


                root_search=True
                try:
                    root=brentq(lambda b:at(b)[1],left,right,xtol=1e-6,maxiter=20)
                except (ValueError,RuntimeError):continue
                finally:root_search=False
                certify(root)
                if accepted:return accepted
    return None


def recover_sideslip(solver,speed_kmh,load,failed,neighbors=()):
    if not solver.config.get('sideslip_recovery',True):return None  # zero-sideslip policy (march chart)
    if solver.config.get('experimental_lateral_trim','full')!='full':return None
    if not needs_sideslip_search(failed):
        return None

    if set(failed['reasons']) & {'IAS limit','Mach limit','sweep unavailable'}:
        return None
    if failed['converged']:


        for da in (0.,.00001,-.00001,.0001,-.0001):
            for db in (.00001,-.00001,.0001,-.0001,.001,-.001):
                initial=list(failed['solution']);initial[0]+=da;initial[1]+=db
                result=solver.solve(speed_kmh,load,initial,exhaustive=False)
                if result['valid']:
                    result['recovery_method']='balanced authority rounding-branch search'
                    return result
    good=[p for p in neighbors if p['valid']]
    seeds=[failed['solution']]
    for side in (sorted((p for p in good if p['load_g']<load),key=lambda p:-p['load_g']),
                 sorted((p for p in good if p['load_g']>load),key=lambda p:p['load_g'])):
        if side and side[0]['solution'] not in seeds:seeds.append(side[0]['solution'])


    if not solver.config['autotrim']:
        physical=solver
    else:
        physical=getattr(solver,'_physical_sideslip_recovery',None)
        if physical is None:
            physical=copy.copy(solver)
            physical.config=dict(solver.config,instructor=False,autotrim=False,aircraft_settings={})
            physical._sideslip_solvers={};physical.instructor_boundaries={}
            solver._physical_sideslip_recovery=physical
    attempts=0;last_failed=0.

    def probe(angle,starts):
        nonlocal attempts
        view=physical.at_sideslip(angle)
        for start in starts:
            point=view.solve(speed_kmh,load,start,exhaustive=False)
            attempts+=1


            accepted=point['valid'] or (solver.config['autotrim'] and
                                       set(point['reasons'])=={'control authority'})
            if accepted and abs(point['sideslip_deg'])<=MAX_SIDESLIP_DEG:
                return point
        return None

    for magnitude in SEARCH_ANGLES:
        candidates=[]
        for sign in (1.,-1.):
            candidate=probe(sign*magnitude,seeds)
            if candidate:candidates.append(candidate)
        for candidate in sorted(candidates,key=lambda p:(abs(p['sideslip_deg']),-p['ps_mps'])):


            lo,hi=last_failed,magnitude;sign=1. if candidate['sideslip_attitude_deg']>0 else -1.
            for _ in range(5):
                if hi-lo<=.001:break
                mid=(lo+hi)*.5
                trial=probe(sign*mid,[candidate['solution'],*seeds])
                if trial:hi=mid;candidate=trial
                else:lo=mid
            actual=solver.at_sideslip(candidate['sideslip_attitude_deg'])
            result=actual.solve(speed_kmh,load,candidate['solution'],exhaustive=False)
            if result['valid'] and abs(result['sideslip_deg'])<=MAX_SIDESLIP_DEG:
                result['recovery_method']='balanced small-sideslip equilibrium'
                result['sideslip_recovery']=dict(max_abs_deg=MAX_SIDESLIP_DEG,attempts=attempts,
                    policy='zero sideslip preferred; smallest successful searched magnitude; not a global optimum')
                return result
        last_failed=magnitude
    return None


def recover_sideslip_boundary(solver,speed_kmh,low,high):
    if not solver.config.get('sideslip_recovery',True):return None  # zero-sideslip policy (march chart)
    if solver.config.get('experimental_lateral_trim','full')!='full':return None
    # Sideslip cannot make the selected wing sweep available (as in recover_sideslip).
    if 'sweep unavailable' in high.get('reasons',()):return None
    for magnitude in SEARCH_ANGLES:
        candidates=[]
        for sign in (1.,-1.):
            view=solver.at_sideslip(sign*magnitude)
            left=view.solve(speed_kmh,low['load_g'],low['solution'],exhaustive=False)
            if not left['valid'] or abs(left['sideslip_deg'])>MAX_SIDESLIP_DEG:continue
            right=view.solve(speed_kmh,high['load_g'],high['solution'],exhaustive=False)
            limit=view.boundary(speed_kmh,left,right,continuation=False)
            if (limit and limit['valid'] and limit.get('envelope_limit') and
                limit['load_g']>=low['load_g']-1e-7 and abs(limit['sideslip_deg'])<=MAX_SIDESLIP_DEG):
                candidates.append(limit)
        if candidates:
            result=min(candidates,key=lambda p:(abs(p['sideslip_deg']),-p['load_g']))
            result['recovery_method']='small-sideslip active-constraint solve'
            return result
    return None
