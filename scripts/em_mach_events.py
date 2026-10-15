"""Mach-polar approximation metadata, events and float32 correction proposals.

Event detection follows the selected polar evaluation; trim acceptance is unchanged.
"""
import os
import numpy as np

ENABLED = os.environ.get('WT_EM_MACH_AWARE', '1') != '0'
# Heuristics select extra sampling/projection work, never relaxed acceptance.
EVENT_RELATIVE_JUMP = 1e-4
CANCELLATION_RATIO = 16384.
# A large condition number only requests cheap probes. A region is withheld
# only when repeated native adjacent-bin jumps dwarf the polynomial's change.
ADJACENT_RELATIVE_JUMP = .01
MIN_JUMP_PROBES = 4


def configure_model(model,mode):
    from polar_runtime import approximate_mach,stabilize_mach
    if mode not in ('native','continuous','stable'):raise ValueError('Unknown Mach curve mode')
    families=list(model['polars'].values())
    families.extend(wing['polars'] for _,wing in model['wing_family'])
    for family in families:
        for _,runtime in family:
            runtime.pop('stable_mach',None)
            runtime.pop('continuous_mach',None)
            if mode=='continuous':approximate_mach(runtime)
            elif mode=='stable':stabilize_mach(runtime)
    model['mach_curve_mode']=mode
    for key in ('_condition_cache','_condition_flap_cache','_sweep_cache'):model.pop(key,None)
    return model


def cancellation_jumps(runtime,index):
    from polar_runtime import mach_value
    a,b,high,slope,limit,*c=runtime['mach'][index]
    evidence=[]
    for value in np.unique(np.asarray(np.linspace(a,b,19)[1:-1],dtype=np.float32)):
        x=float(value)
        y=float(np.nextafter(np.float32(x),np.float32(np.inf)))
        if not a<x<y<b:continue
        left,right=mach_value(runtime,x,index),mach_value(runtime,y,index)
        # Binary64 estimates only the expected local variation; all evidence
        # and all aerodynamic evaluations retain native binary32 arithmetic.
        derivative=c[1]+2.*c[2]*x+3.*c[3]*x*x
        dx=y-x
        expected=abs(derivative*dx+(c[2]+3.*c[3]*x)*dx*dx+c[3]*dx**3)
        jump=abs(right-left)
        if jump>max(ADJACENT_RELATIVE_JUMP*max(1.,abs(high),abs(left),abs(right)),16.*expected):
            evidence.append(dict(mach_pair=[x,y],values=[float(left),float(right)],
                                 expected_change=expected))
    return evidence if len(evidence)>=MIN_JUMP_PROBES else []


def model_catalog(model, flaps):
    from polar_runtime import flap_polar, mach_value
    from mach_cubic import approximation_knots,JOIN_FRACTION,join_scale
    from control_mixer import curve
    values=curve(model['flaps'],flaps,4) if model['flaps'] else [flaps]*4
    runtimes=[('WingPlane',flap_polar(model['polars']['WingPlane'],values[0]))]
    runtimes.extend((name,model['polars'][name][0][1])
                   for name in ('HorStabPlane','VerStabPlane','FuselagePlane'))
    events={};rough=[];discontinuities=[];approximations=[];projection=[];pieces=[]
    for component,runtime in runtimes:
        if runtime['mode']!=3:continue
        for index,row in enumerate(runtime['mach']):
            pieces.append(tuple(row[:5]))
            if index==6 and runtime['base'][8]==0.:continue
            a,b,high,slope,limit,*coefficients=row
            if not 0.<a<b:continue
            approximated=bool(runtime.get('continuous_mach') and runtime['continuous_mach'][index])
            if approximated:
                approximations.append(dict(component=component,multiplier=index,
                    mach_interval=[float(a),float(b)],method='Recentered native cubic with tanh blends centered at native jumps',
                    join_fraction=JOIN_FRACTION,join_width_definition='10%-90% weight width / native Mach interval',
                    tanh_scale=join_scale(row),mach_knots=list(approximation_knots(row)),
                    endpoints=[float(mach_value(runtime,a,index)),float(mach_value(runtime,b,index))],
                    local_coefficients=list(runtime['continuous_mach'][index]),
                    authored_endpoints=[0. if index==5 else 1.,float(high)],
                    authored_slopes=[0.,float(slope)],limit=float(limit)))
                width=4.*join_scale(row)
                projection.append((float(a-width),float(b+width)))
                # These joins are continuous by construction. An adjacent-f32
                # difference on a steep join is not evidence of a formula jump.
                continue
            cancellation=sum(abs(c)*b**i for i,c in enumerate(coefficients))
            if runtime.get('stable_mach') is None and cancellation>CANCELLATION_RATIO*max(1.,abs(high)):
                rough.append((float(a),float(b)))
                evidence=cancellation_jumps(runtime,index)
                if evidence:
                    discontinuities.append(dict(mach_interval=[float(a),float(b)],
                        component=component,multiplier=index,kind='Repeated native Mach quantization jumps',
                        probes=evidence,probe_count=17,
                        coverage='Interpolation unresolved within the transition; not a physical flight exclusion'))
            for mach,side in ((a,'start'),(b,'end')):
                before=float(np.nextafter(np.float32(mach),np.float32(-np.inf)))
                after=float(np.nextafter(np.float32(mach),np.float32(np.inf)))
                left=mach_value(runtime,before if side=='start' else mach,index)
                right=mach_value(runtime,mach if side=='start' else after,index)
                if abs(right-left)<=EVENT_RELATIVE_JUMP*max(1.,abs(left),abs(right)):continue
                event=events.setdefault((float(mach),side),dict(mach=float(mach),side=side,components=[]))
                event['components'].append(dict(component=component,multiplier=index,
                    left_value=float(left),right_value=float(right)))
    return dict(events=[events[k] for k in sorted(events)],rough_intervals=sorted(set(rough)),
                discontinuities=discontinuities,approximations=approximations,
                projection_intervals=sorted(set(projection)),polynomial_pieces=pieces)


def stable_piece_key(solver,mach):
    """Protect declared mode-3 guards, without requiring an exact Mach bin.

    This is a local stencil classification, not a complete source-branch proof.
    Equality remains owned by the cubic, as in the evaluator.
    """
    key=[]
    for a,b,high,slope,limit in catalog(solver,force=True).get('polynomial_pieces',()):
        if mach<a:key.append('low')
        elif mach>b:
            value=high+(mach-b)*slope
            clamped=value>=limit if slope>=0. else value<=limit
            key.append('clamped' if clamped else 'linear')
        else:key.append('cubic')
    return tuple(key)


def discontinuity_regions(info,config):
    """Guard formula jumps and unresolved quantization bands in plotted TAS."""
    from air_state import speed_of_sound
    scale=3.6*speed_of_sound(config['altitude_m'])
    guard=.0036  # Also used by the existing one-sided Mach-event sampling.
    regions=[]
    entries=[(d['mach_interval'],d) for d in info.get('discontinuities',[])]
    entries.extend(([e['mach'],e['mach']],dict(kind='Native Mach formula jump',mach_event=e))
                   for e in info['events'])
    for (a,b),evidence in sorted(entries,key=lambda e:e[0][0]):
        lo,hi=a*scale-guard,b*scale+guard
        if regions and lo<=regions[-1]['speed_interval_kmh'][1]:
            regions[-1]['speed_interval_kmh'][1]=max(hi,regions[-1]['speed_interval_kmh'][1])
            regions[-1]['evidence'].append(evidence)
        else:regions.append(dict(speed_interval_kmh=[lo,hi],evidence=[evidence]))
    return regions


def overlaps(regions,lo,hi):
    return any(lo<b and hi>a for a,b in (r['speed_interval_kmh'] for r in regions))


def solver_regions(solver):
    info=catalog(solver);altitude=solver.config['altitude_m']
    cached=getattr(solver,'_mach_discontinuity_regions',None)
    if cached is None or cached[0] is not info or cached[1]!=altitude:
        cached=(info,altitude,discontinuity_regions(info,solver.config))
        solver._mach_discontinuity_regions=cached
    return cached[2]


def discontinuous_column(solver,speed):
    region=next((r for r in solver_regions(solver) if r['speed_interval_kmh'][0]<speed<r['speed_interval_kmh'][1]),None)
    if region is None:return None
    return dict(speed_kmh=float(speed),points=[],boundary=None,lower_boundary=None,
        sustained=[],load_checks=[],boundary_bracket_g=None,
        boundary_status='native discontinuity',boundary_reason='Native Mach discontinuity; interpolation unresolved',
        numerical_gap_brackets=[],interior_failures=[],unresolved_load_intervals=[],
        discontinuity=region,not_evaluated=True,coverage_complete=False,search_work={},
        elapsed_s=0.,mass=solver.mass,engine=solver.engine.summary)


def catalog(solver,*,force=False):
    if (not ENABLED and not force) or not hasattr(solver,'model'):return dict(events=[],rough_intervals=[])
    key=(id(solver.model),float(solver.flaps),solver.model.get('mach_curve_mode','native'))
    cached=getattr(solver,'_mach_catalog',None)
    if cached is None or cached[0]!=key:
        cached=(key,model_catalog(solver.model,solver.flaps));solver._mach_catalog=cached
    return cached[1]


def aircraft_catalog(fm,config):
    if not ENABLED:return dict(events=[],rough_intervals=[])
    from aircraft_model import prepare,at_sweep
    from component_assembly import f32
    model=at_sweep(prepare(fm),config['sweep_percent']/100.)
    configure_model(model,config.get('mach_curve_mode','native'))
    return model_catalog(model,f32(config['flaps_percent']/100.))


def left_of(mach,event):
    return mach<event['mach'] if event['side']=='start' else mach<=event['mach']


def crossings(events,left,right):
    if not left or not right or 'mach' not in left or 'mach' not in right:return []
    return [e for e in events if left_of(left['mach'],e) and not left_of(right['mach'],e)]


def annotate(solver,column):
    if column is None:return column
    events=catalog(solver)['events']
    if events and all(column.get(k) and 'mach' in column[k] for k in ('lower_boundary','boundary')):
        column['mach_branch']=[sum(not left_of(column[k]['mach'],e) for e in events)
                               for k in ('lower_boundary','boundary')]
    return column


def rough_mach(solver,mach):
    info=catalog(solver)
    # Body rotation can move evaluated Mach by a binary32 bin at fixed TAS.
    # Hold that bin for derivative proposals across repaired intervals and tails;
    # accepted states still use the ordinary aerodynamic/residual evaluator.
    return any(a<=mach<=b for a,b in info['rough_intervals']+info.get('projection_intervals',[]))


def project_state(solver,speed,load,x,mach,*,bank_bounds=None):
    """Small bank adjustments find the same *evaluated* Mach bin, if attainable.

    Used for derivative and corrector proposals. The coupled stencil accounts
    for its actual displacement, and acceptance uses native residuals.
    """
    from em_operating import flight_condition
    x=np.asarray(x)
    low,high=(solver.trim_bounds[0][1],solver.trim_bounds[1][1]) if bank_bounds is None else bank_bounds
    if flight_condition(solver,speed,load,x)[1]['mach']==mach:return x
    step=max(abs(float(np.spacing(np.float32(x[1])))),1e-7)
    for offset in (1.,-1.,2.,-2.,4.,-4.,8.,-8.,16.,-16.):
        q=x.copy();q[1]+=offset*step
        if not low<=q[1]<=high:continue
        if flight_condition(solver,speed,load,q)[1]['mach']==mach:return q
    return x


def speed_knots(info,config):
    # A sentinel separates interpolation support on either side, even when
    # the plotting raster is wider than the discontinuity.
    knots=[v for r in discontinuity_regions(info,config)
            for a,b in [r['speed_interval_kmh']] for v in (a,(a+b)*.5,b)]
    from air_state import speed_of_sound
    scale=3.6*speed_of_sound(config['altitude_m'])
    # A steep continuous interval still needs explicit sampling, even when it
    # is narrower than the initial speed grid. Do not classify it as a gap.
    knots.extend(m*scale for r in info.get('approximations',[])
                 for m in r['mach_knots'])
    return sorted(set(knots))


def transition_width(config):
    from em_accuracy import speed_tolerance
    return max(.01,min(.05,.1*speed_tolerance(config)))


def certificate(solver,pair,sample):
    """Certify a short native formula transition by independent one-sided trims."""
    left,right=[c['boundary'] for c in pair]
    if not all(p and p.get('valid') for p in (left,right)):return None
    events=crossings(catalog(solver)['events'],left,right)
    if len(events)!=1:return None
    event=events[0]
    if left.get('roll_leveling_branch')!=right.get('roll_leveling_branch'):return None
    if right['speed_kmh']-left['speed_kmh']>transition_width(solver.config):return None
    # Bracket using measured Mach, not nominal TAS alone.
    a,b=left,right;points={p['speed_kmh']:p for p in (a,b)}
    from air_state import speed_of_sound
    target=max(.0005,8.*float(np.spacing(np.float32(event['mach'])))*3.6*speed_of_sound(solver.config['altitude_m']))
    for _ in range(10):
        if b['speed_kmh']-a['speed_kmh']<=target:break
        speed=(a['speed_kmh']+b['speed_kmh'])*.5
        p=sample(speed,a,b)
        if (p is None or not p.get('valid') or
                p.get('roll_leveling_branch')!=left.get('roll_leveling_branch')):return None
        points[speed]=p
        if left_of(p['mach'],event):a=p
        else:b=p
    else:return None
    # Validate interpolation on each one-sided outer piece, retaining its spread.
    from em_accuracy import turn_tolerance
    tolerance=turn_tolerance(solver.config['sep_tolerance_mps'])
    spread=[];uncertainty=[]
    for lo,hi in ((left,a),(b,right)):
        side_points=[lo,hi];side_errors=[]
        for fraction in (1./3.,2./3.) if lo['speed_kmh']<hi['speed_kmh'] else ():
            speed=lo['speed_kmh']+(hi['speed_kmh']-lo['speed_kmh'])*fraction
            p=sample(speed,lo,hi)
            if (p is None or not p.get('valid') or
                    p.get('roll_leveling_branch')!=lo.get('roll_leveling_branch') or
                    left_of(p['mach'],event)!=left_of(lo['mach'],event)):return None
            error=abs(p['turn_dps']-((1-fraction)*lo['turn_dps']+fraction*hi['turn_dps']))
            spread.append(error)
            side_errors.append(error);side_points.append(p)
            points[speed]=p
        if max(side_errors,default=0.)>tolerance:
            # Every retained point is independently balanced. Do not claim that
            # interpolation through quantized native values meets a smoothness
            # tolerance: retain the observed spread explicitly instead.
            uncertainty.append(dict(speed_interval_kmh=[lo['speed_kmh'],hi['speed_kmh']],
                sampled_turn_interval_dps=[min(p['turn_dps'] for p in side_points),max(p['turn_dps'] for p in side_points)],
                interpolation_error_dps=max(side_errors),rigorous_bound=False))
    return dict(speed_interval_kmh=[left['speed_kmh'],right['speed_kmh']],
        transition_speed_interval_kmh=[a['speed_kmh'],b['speed_kmh']],
        transition_kind='Mach-polar boundary transition',mach_event=event,
        method='native Mach event bracket with independently balanced one-sided samples',
        points=[points[v] for v in sorted(points)],
        one_sided_error_dps=max(spread,default=0.),
        one_sided_interpolation_verified=not uncertainty,
        native_mach_uncertainty=uncertainty,
        evaluated_mach_interval=[a['mach'],b['mach']])
