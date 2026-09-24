"""Discrete propulsion controls compared using complete aircraft equilibria.

This is a numerical search, not a proof of a global optimum. Every score comes
from a re-trimmed, settled installation; maximum free-stream thrust is only a
starting point. Discrete stage/automatic branches get separate starts.
"""
import copy


def key(controls):
    return (tuple(controls['commands']),tuple(controls['automatic']),tuple(controls['gears']))


def replace_group(controls,selection,local):
    result=copy.deepcopy(controls)
    for i,j in enumerate(selection['engine_indices']):result['gears'][j]=local['gears'][i]
    for i,j in enumerate(selection['propeller_indices']):
        result['commands'][j]=local['commands'][i]
        result['automatic'][j]=local['automatic'][i]
    return result


def solve(solver,speed_kmh,load,initial=None,detailed=False,exhaustive=True,refine=False,controls_hint=None):
    seed=solver.engine.seed_controls(speed_kmh/3.6);memo={};failures=[]
    def evaluate(controls,start):
        k=key(controls)
        if k not in memo:
            view=solver.with_prop_controls(controls)
            view.engine.no_resolved_seed=any(not g['candidates'] for g in seed['selections'])
            try:
                p=view.solve(speed_kmh,load,start,detailed=detailed,
                             exhaustive=exhaustive and k==key(seed['controls']),refine=refine)
            except (ValueError,ArithmeticError) as error:
                failures.append(dict(controls=controls,reason=str(error)));return None
            memo[k]=p
        return memo[k]
    def rank(p):
        if p['valid']:
            refined=p['force_error_g']<=1e-6 and p['angular_error_rad_s2']<=2e-6
            return (4 if not refine or refined else 3,p['ps_mps'],0.)
        if p['converged'] and not set(p['reasons'])-{'post-stall','control authority','wing force limit','Instructor pitch limit'}:
            violation=max(0.,-p['stall_margin_deg']/10.,-p['authority_margin'],
                max(p['wing_load_ratios'])-1. if solver.config['structural_limits'] else 0.,
                -p['instructor']['envelope_margin'] if p.get('instructor') else 0.)
            return (2,-violation,p['ps_mps'])
        # Excess power from an unbalanced iterate is not performance. Use its
        # residual only to choose a numerical recovery seed.
        return (1,-max(p['force_error_g']/2e-4,p['angular_error_rad_s2']/5e-5,p['history_error']/2e-4),0.)
    # A boundary supplies an already balanced installation. Keep that root
    # available while testing other settings instead of losing it to a failed
    # first solve initialized with a different propeller command.
    best=evaluate(controls_hint,initial) if controls_hint is not None else None
    nominal=evaluate(seed['controls'],best['solution'] if best is not None else initial)
    if nominal is not None and (best is None or rank(nominal)>rank(best)):best=nominal
    if best is None:raise ValueError('The propeller seed could not produce an aircraft equilibrium')
    # Coordinate sweeps cover every drivetrain; exact coupled scoring includes
    # propwash, reaction, gyroscopic load and the drag of all trim deflections.
    sweeps=0;improved=True
    while improved and sweeps<3:
        improved=False;sweeps+=1
        for group in seed['selections']:
            baseline=best['propulsion']['controls'];branches={}
            for row in group['candidates']:
                c=row['controls'];branch=(tuple(c['gears']),tuple(c['automatic']))
                branches.setdefault(branch,[]).append(c)
            trials=[]
            for rows in branches.values():
                # Native nominal-thrust ordering supplies three distinct starts
                # per stage/mode. Local command refinement below uses actual Ps.
                trials.extend(rows[:3])
            for c in trials:
                p=evaluate(replace_group(baseline,group,c),best['solution'])
                if p is not None and rank(p)>rank(best):best=p;improved=True
            for stride in (8,4,2,1):
                while True:
                    if not best['converged']:break
                    was_valid=best['valid']
                    changed=False;controls=best['propulsion']['controls']
                    indices=group['propeller_indices']
                    if any(controls['automatic'][j] for j in indices):break
                    if not any(solver.engine.properties['propellers'][j]['manual'] and
                               solver.engine.properties['propellers'][j]['governor']!=0 for j in indices):break
                    for delta in (-stride,stride):
                        c=copy.deepcopy(controls)
                        if not all(0<=c['commands'][j]+delta<=255 for j in indices):continue
                        for j in indices:c['commands'][j]+=delta
                        p=evaluate(c,best['solution'])
                        if p is not None and rank(p)>rank(best):best=p;changed=True;improved=True
                    if not changed or not was_valid:break
    recovery_calls=0
    if exhaustive and not best['valid']:
        view=solver.with_prop_controls(best['propulsion']['controls'])
        view.engine.no_resolved_seed=any(not g['candidates'] for g in seed['selections'])
        recovered=view.solve(speed_kmh,load,best['solution'],detailed=detailed,exhaustive=True,refine=refine)
        recovery_calls=recovered['evaluations']
        if rank(recovered)>rank(best):best=recovered
    best['propulsion']['optimization']=dict(
        method='Stage/mode multistart and integer command coordinate search on re-trimmed Ps',
        evaluated=len(memo),coordinate_sweeps=sweeps,search_complete=not improved,
        global_optimum_certified=False,nominal_search=seed['counts'],failures=failures,
        radiators='closed',optional_rocket_boosters='off')
    best['evaluations']=sum(p['evaluations'] for p in memo.values())+recovery_calls
    return best


def boundary(solver,speed_kmh,low,high):
    """Bracket a physical limit with independently optimized equilibria.

    Discrete control changes need not form a differentiable sixth equation.
    A failed propulsion/trim solve never serves as the excluded endpoint.
    """
    physical={'post-stall':'stall','control authority':'control','wing force limit':'wing force',
              'Instructor pitch limit':'Instructor pitch'}
    if not low['valid']:return None
    samples=0
    # A fixed-control active-constraint solve gives a close starting point;
    # its acceptance is rechecked after the full propulsion optimization.
    controls=low['propulsion']['controls'];seen=set()
    for _ in range(4):
        if key(controls) in seen:break
        seen.add(key(controls));fixed=solver.with_prop_controls(controls)
        left=fixed.solve(speed_kmh,low['load_g'],low['solution'],exhaustive=False)
        right=fixed.solve(speed_kmh,high['load_g'],high['solution'],exhaustive=False)
        if not left['valid']:break
        guess=fixed.boundary(speed_kmh,left,right,continuation=False)
        if not guess:
            # Move the numerical starting bracket toward the constraint with
            # this already selected control set. Re-optimizing every failed
            # midpoint is expensive and gives unbalanced states no meaning.
            # These midpoints supply seeds only: acceptance still requires the
            # full active-constraint equations and subsequent control search.
            for attempt in range(9):
                if right['load_g']-left['load_g']<.001:break
                middle=fixed.solve(speed_kmh,(left['load_g']+right['load_g'])*.5,
                                   left['solution'],exhaustive=False)
                if middle['valid']:left=middle
                else:right=middle
                if attempt in (3,6,8):
                    guess=fixed.boundary(speed_kmh,left,right,continuation=False)
                    if guess:break
        if not guess or not low['load_g']-1e-7<=guess['load_g']<=high['load_g']:break
        # Interior plotting tolerances can accept an almost-balanced control
        # setting whose true stall edge is slightly below this load. Tighten
        # the numerical solve when comparing settings at an active constraint.
        p=solve(solver,speed_kmh,guess['load_g'],guess['solution'],exhaustive=False,refine=True,
                controls_hint=guess['propulsion']['controls']);samples+=1
        if p['valid']:
            kind=guess['envelope_limit']['kind']
            margin={'stall':p['stall_margin_deg'],'control':p['authority_margin'],
                    'wing force':1.-max(p['wing_load_ratios']),
                    'Instructor pitch':p['instructor']['envelope_margin'] if p.get('instructor') else float('inf')}.get(kind,float('inf'))
            target,tolerance=(.002,.0002) if kind=='stall' else (3e-5,5e-6) if kind=='control' else (2e-6,1e-6) if kind=='Instructor pitch' else (5e-5,1e-5)
            if abs(margin-target)<=tolerance:
                p['envelope_limit']=dict(guess['envelope_limit'],
                    method='Active aircraft constraint rechecked after discrete propulsion optimization',
                    constraint_residual=margin-target,propulsion_refinements=samples,global_optimum_certified=False)
                return p
            low=p;controls=p['propulsion']['controls']
        elif p['converged'] and p['reasons'] and not set(p['reasons'])-physical.keys():high=p;break
        else:break
    if not high['converged'] or not high['reasons'] or set(high['reasons'])-physical.keys():return None
    for _ in range(15):
        if high['load_g']-low['load_g']<=.0005:
            result=dict(low);kind=physical[high['reasons'][0]]
            result['envelope_limit']=dict(kind=kind,axis=None,limiting_load_g=low['load_g'],
                limiting_load_interval_g=[low['load_g'],high['load_g']],evaluations=samples,
                method='Balanced physical bracket with re-optimized discrete propulsion controls',
                global_optimum_certified=False)
            return result
        n=(low['load_g']+high['load_g'])*.5
        p=solver.solve(speed_kmh,n,low['solution'],exhaustive=False);samples+=1
        if p['valid']:low=p
        elif p['converged'] and p['reasons'] and not set(p['reasons'])-physical.keys():high=p
        else:return None
    return None
