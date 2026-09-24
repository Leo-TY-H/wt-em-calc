"""Select the pre-stall trim branch by continuation from level flight.

A balanced root is not necessarily connected to normal trimmed flight. Power
is an output, never a criterion for choosing between different trim roots.
"""
import math
import numpy as np
from em_pitch_response import PHYSICAL_REASONS, KINDS


def first_pitch_response_limit(solver,speed,level,boundary,points):
    """Do not join normal-flight components through observed control reversal."""
    if not level or not boundary or not level['valid'] or level['load_g']!=1.:return None
    rejected=sorted((p for p in points if p['converged'] and p['reasons']==['reversed pitch response']
                     and level['load_g']<p['load_g']<boundary['load_g']),key=lambda p:p['load_g'])
    if not rejected:return None
    high=rejected[0]
    low=max((p for p in points if p['valid'] and level['load_g']<=p['load_g']<high['load_g']),
            key=lambda p:p['load_g'],default=level)
    limit=solver.boundary(speed,low,high,continuation=False,candidate_kinds={'pitch response'},max_iterations=8)
    if not limit or limit['load_g']>=boundary['load_g']-1e-5:return None
    limit['envelope_limit']['selection']='First observed loss of normal pitch response from level flight'
    return limit


def connected_limit(solver,speed,level,neighbor_points=None):
    """Trace balanced states to a verified limit without crossing control roots.

    Step sizes constrain only the numerical predictor/corrector. Every point
    and the final physical limit retain the original acceptance tolerances.
    An unresolved interval returns no branch certificate, not a made-up edge.
    """
    if not level['valid'] or level['load_g']!=1.:return None
    points=[level];rejected=[];seams=[];step=.08
    def unresolved():
        # Preserve the verified connected interior. Never fall back to a
        # different root's stall/control boundary when this trace cannot close.
        return dict(points=points,boundary=points[-1],rejected=rejected,seams=seams,unresolved=True,
                    method='verified level-flight continuation; upper limit unresolved')
    def cross_native_switch(last,failed):
        # The original roll-leveling helper switches at +/-12 degrees. Its
        # removal can require a discontinuous aileron correction even on an
        # otherwise nearby trim branch. A universal control-step limit would
        # incorrectly truncate that branch. Solve across the actual switch,
        # then re-solve the failed load from that independently balanced seed.
        if not solver.fm.get('RollLeveling',True) or not last['valid']:return None
        from em_branch_limit import fixed_alpha
        for alpha in (-12.,12.):
            if not alpha-.03<last['alpha_deg']<alpha:continue
            if not last['load_g']<failed['load_g']<last['load_g']+.02:continue
            # A corrected point already on the other branch is stronger
            # evidence than another solve at an arbitrary offset angle.
            opposite=(failed if failed['valid'] and failed.get('roll_leveling_branch')!=last.get('roll_leveling_branch')
                      else fixed_alpha(solver,speed,alpha+.002,last))
            if not opposite or not opposite['valid']:continue
            if opposite.get('roll_leveling_branch')==last.get('roll_leveling_branch'):continue
            candidate=(failed if opposite is failed else
                       solver.solve(speed,failed['load_g'],opposite['solution'],exhaustive=False,quick=True))
            gap=False
            if not candidate['valid'] and opposite['load_g']>failed['load_g']:
                # The failed load lies between balanced states on opposite
                # sides of the switch. Keep it masked and continue from the
                # independently verified state above it.
                candidate=opposite;gap=True
            if not candidate['valid'] or candidate.get('roll_leveling_branch')!=opposite.get('roll_leveling_branch'):continue
            change=abs(np.asarray(candidate['solution'])-np.asarray(last['solution']))
            if (change[0]>.25 or change[1]>2. or change[3]>.08 or change[4]>.15 or
                    abs(candidate['ps_mps']-last['ps_mps'])>2.):continue
            candidate['continuation_native_switch']=dict(alpha_deg=alpha,
                method='balanced states across the native roll-leveling switch',failed_load_resolved=not gap,
                previous_load_g=last['load_g'],previous_solution=last['solution'],
                opposite_load_g=opposite['load_g'],opposite_solution=opposite['solution'])
            if gap:
                marker=dict(failed,native_seam_marker=True,surface_sample=True)
                seams.append(dict(lower=last,upper=candidate,failed=marker))
            return candidate
        return None
    def cross_numerical_seam(last,failed):
        """Resume only across a tiny, balanced gap in a native piecewise branch.

        Cold starts can reach the same root from above when a one-sided Newton
        search gets trapped at a polar seam. Both sides must independently
        close the aircraft equations and have nearly identical control states.
        The failed load remains masked; it is never treated as an equilibrium.
        """
        if not (last['stall_margin_deg']>.1 and last['authority_margin']>.02 and
                (not solver.config['structural_limits'] or max(last['wing_load_ratios'])<.98)):
            return None
        native=cross_native_switch(last,failed)
        if native:return native
        # Search just above the failed load first. Near 1 g, a native polar
        # seam can be only a few ten-thousandths of a g wide; a fixed .01 g
        # jump changes the controls enough to fail the same-branch check.
        nearby=[p for p in (neighbor_points or []) if p['valid'] and
                abs(p['speed_kmh']-speed)<max(2.,speed*.02) and
                p['alpha_deg']>last['alpha_deg']+.005 and
                p['load_g']>last['load_g']]
        for distance in (.0002,.0005,.001,.002,.005,.01,.02):
            if last['load_g']+distance<=failed['load_g']:continue
            load=last['load_g']+distance
            candidate=solver.solve(speed,load,exhaustive=False)
            if not candidate['valid'] and nearby:
                neighbor=min(nearby,key=lambda p:abs(p['load_g']-load))
                candidate=solver.solve(speed,load,neighbor['solution'],exhaustive=False)
            if not candidate['valid']:continue
            if (candidate['stall_margin_deg']<=.1 or candidate['authority_margin']<=.02 or
                    solver.config['structural_limits'] and max(candidate['wing_load_ratios'])>=.98):continue
            state_change=np.abs(np.asarray(candidate['solution'])-np.asarray(last['solution']))
            if (state_change[0]>.25 or state_change[1]>2. or
                    state_change[2]>.15 or state_change[3]>.08 or state_change[4]>.15 or
                    abs(candidate['ps_mps']-last['ps_mps'])>2.):continue
            # The original failed solve lies strictly between these verified
            # equilibria. Keep it as a display and interpolation separator.
            if not last['load_g']<failed['load_g']<candidate['load_g']:continue
            marker=dict(failed,native_seam_marker=True,surface_sample=True)
            seams.append(dict(lower=last,upper=candidate,failed=marker))
            return candidate
        return None
    for _ in range(240):
        last=points[-1]
        u=math.sqrt(max(0.,last['load_g']**2-1.))
        n=math.hypot(1.,u+step)
        if n>64.:return unresolved()
        guess=np.array(last['solution'])
        if len(points)>1:
            # Do not turn a native Mach/roll-helper command jump into a
            # derivative. Near-equal loads on different float32 branches can
            # otherwise predict enormous command changes and force hundreds
            # of shrinking corrections through an ordinary interior region.
            previous=next((p for p in reversed(points[:-1]) if p['mach']==last['mach']
                           and p.get('roll_leveling_branch')==last.get('roll_leveling_branch')),None)
            if previous is not None:
                previous_u=math.sqrt(max(0.,previous['load_g']**2-1.))
                guess+=(guess-np.array(previous['solution']))*step/(u-previous_u)
        guess[1]=math.degrees(math.atan(u+step))+last['bank_deg']-math.degrees(math.atan(u))
        # Near a known active constraint, solve that equation on this local
        # branch before spending iterations outside the physical envelope.
        pitch_near=solver.config['instructor'] and last.get('instructor',{}).get('envelope_margin',1.)<.04
        near=pitch_near or last['stall_margin_deg']<.6 or last['authority_margin']<.04
        wing_near=solver.config['structural_limits'] and max(last['wing_load_ratios'])>.97
        near=near or wing_near
        response_scale=max((level.get('pitch_response') or {}).get('margin',0.),1e-8)
        response_margin=(last.get('pitch_response') or {}).get('margin',response_scale)
        response_near=response_margin<.15*response_scale
        near=near or response_near
        if step<.00015 and rejected:
            resumed=cross_numerical_seam(last,rejected[-1])
            if resumed:
                points.append(resumed);step=.08
                continue
        # Evaluate the corrector before classifying the crossed constraint.
        # A controller can reject abruptly while its previous margin is still
        # large. That observed balanced rejection is a useful bracket even
        # when the near-limit heuristic did not predict it.
        point=solver.solve(speed,n,guess.tolist(),exhaustive=False,quick=near)
        if near:
            # Establish an actual crossing before asking a six-variable
            # optimizer to find a constraint from an unbalanced hint. This is
            # particularly important at the singular 1 g end of a turn.
            if point['valid']:near=response_near
        physical=(point is not None and point['converged'] and point['reasons'] and
            set(point['reasons'])<=PHYSICAL_REASONS)
        if physical or near or step<.00015:
            # A shrinking load step cannot regularize maximum lift or an
            # actuator stop. Close the active equation with load free as soon
            # as the nearby corrector fails. Several limits can be close at
            # once; choosing only control hid a nearer stall on some aircraft.
            distances={}
            if pitch_near:distances['Instructor pitch']=last['instructor']['envelope_margin']/.04
            if last['authority_margin']<.04:distances['control']=last['authority_margin']/.04
            if last['stall_margin_deg']<.6:distances['stall']=last['stall_margin_deg']/.6
            if wing_near:distances['wing force']=(1.-max(last['wing_load_ratios']))/.03
            if response_near:distances['pitch response']=response_margin/(.15*response_scale)
            if physical:
                for reason,kind in KINDS.items():
                    if reason in point['reasons']:distances.setdefault(kind,0.)
            if 'stall' in distances or 'control' in distances:
                # Close both competing equations. Near a lift maximum the
                # remaining elevator travel can be substantial at the last
                # load-corrected point, although its stop occurs before stall.
                distances.setdefault('stall',last['stall_margin_deg']/.6)
                distances.setdefault('control',last['authority_margin']/.04)
            if not distances and step<.00015:
                # All physical constraints still have room. A shrinking
                # corrector is then a possible trim fold, not evidence of a
                # remote stall/control limit. Test its regular coordinate
                # before a near-1-g active-constraint search becomes singular.
                from em_trim_fold import trim_fold
                limit=trim_fold(solver,speed,last)
                if limit:
                    points=[p for p in points if p['load_g']<limit['load_g']]+[limit]
                    return dict(points=points,boundary=limit,rejected=rejected,seams=seams,
                        method='continuous force/moment equilibria from level flight to a trim fold')
            if not distances:distances['stall']=1.
            kind=min(distances,key=distances.get)
            hint=dict(last,load_g=n,solution=np.clip(guess,*solver.trim_bounds).tolist(),converged=False,reasons=[],
                      continuation_limit_kind=kind)
            if point is not None and point['converged'] and point['reasons']:hint=point
            limit=solver.boundary(speed,last,hint,continuation=False,
                                  candidate_kinds=set(distances),max_iterations=8)
            if (limit and limit['load_g']>=last['load_g']-.004
                    and limit['load_g']-last['load_g']<max(.3,last['load_g']*.1)
                    and max(abs(np.array(limit['solution'])[2:]-np.array(last['solution'])[2:]))<.2
                    and abs(limit['alpha_deg']-last['alpha_deg'])<2.):
                points=[p for p in points if p['load_g']<limit['load_g']]+[limit]
                return dict(points=points,boundary=limit,rejected=rejected,seams=seams,
                    method='continuous force/moment equilibria from level flight')
            if step<.00015:
                from em_trim_fold import trim_fold
                limit=trim_fold(solver,speed,last)
                if limit:
                    points=[p for p in points if p['load_g']<limit['load_g']]+[limit]
                    return dict(points=points,boundary=limit,rejected=rejected,seams=seams,
                        method='continuous force/moment equilibria from level flight to a trim fold')
                return unresolved()
        solver.__dict__.pop('_trim_predictor',None)
        if point is None:
            point=solver.solve(speed,n,guess.tolist(),exhaustive=False)
        control_step=max(abs(np.array(point['solution'])[2:]-np.array(last['solution'])[2:]))
        angle_step=abs(point['alpha_deg']-last['alpha_deg'])
        prediction_error=max(abs(np.array(point['solution'])[2:]-guess[2:]))
        if not point['valid'] or control_step>=.15:
            resumed=cross_native_switch(last,point)
            if resumed:
                rejected.append(point);points.append(resumed);step=.08
                continue
        # At very small load steps the allowed native force residual is larger
        # than the step itself. Do not demand a sub-tolerance predictor match;
        # still require the full balance and a tightly continuous corrected state.
        mach_quantum=max(abs(float(np.spacing(np.float32(p['mach'])))) for p in (last,point))
        rounded_mach=point['mach']!=last['mach'] and abs(point['mach']-last['mach'])<=4*mach_quantum
        # TAS is fixed in this column. Quaternion/air-cache rounding can
        # select neighboring native Mach polynomials with different moments.
        # A fully balanced, tightly nearby correction across those branches
        # need not meet a smooth predictor's error estimate. Shrinking the
        # step below that rounding noise stalls continuation unnecessarily.
        close_correction=rounded_mach or prediction_error<.015 or step<=.001 and control_step<.03 and angle_step<.15
        if point['valid'] and control_step<.15 and angle_step<1.5 and (len(points)<2 or close_correction):
            points.append(point)
            # Control and angle predictor errors determine the next step.
            # A fixed eight-percent load ceiling forced dozens of redundant
            # correctors through almost linear high-speed trim branches.
            # Keep the same continuity tests, shrinking on every failed step.
            growth=min(1.8,.8*.15/max(control_step,1e-8),
                       .8*1.5/max(angle_step,1e-8),
                       math.sqrt(.8*.015/max(0. if rounded_mach else prediction_error,1e-8)))
            step=min(max(.12,point['load_g']*.4),step*max(.5,growth))
        else:
            rejected.append(point);step*=.5
    return unresolved()
