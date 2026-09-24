"""Exact held-source Instructor recurrence, shared by reference and compiled backends."""
import copy,math,struct
from component_assembly import f32
from control_mixer import curve,density_at_height
from primary_controls import authority_ranges,steady_commands
from instructor_settings import trim_retained
from instructor_keyboard import keyboard_step
from instructor_protection import predicted_wing_angles,authority_factor_run,history_repeats

def settled_controller(model,controls,state,dt,*,seed=0,max_iterations=None,entry=None,accelerate_history=True,search_only=False,defer_cycles=False):
    """Fixed-point iteration of the actual keyboard controller at a held state.

    Wing prediction history uses the same operating point. Protection timers
    and authority advance through their native float32 recurrences until they
    stop changing. Auto-trim/predictor/command histories are iterated, never
    read from the native oracle. search_only endpoint estimates are numerical
    initializers; solve() independently certifies accepted roots without them.
    """
    from em_solver import command_allocation,DEFAULTS
    state=copy.deepcopy(state);ail=model['controls']['Ailerons']
    automatic_budget=max_iterations is None
    if automatic_budget:
        rate=float(state['properties'].get('overloadTimeRate',.45))
        max_iterations=96 if search_only else max(96,min(12000,int(math.ceil((max(1./rate if rate>0 else 0.,80.)+2.)/dt))))
    predictor_cache={}
    sensitivity=f32(curve(ail['sensitivity_curve'],state['wrapper']['mach'],1)[0]*ail['sensitivity'])
    adjusted=predicted_wing_angles(state['wing_angles'],[0.,0.],*state['delivered'][:2],sensitivity,model['controls']['Elevator']['wing_aoa'])['adjusted']
    history=dict(autotrim=[float(seed),0.,False],predictors=[[0.,0.,False],[0.,0.,False]],angles=adjusted,
        overload_timer=0.,authority_factor=1.,recovery=[0.,float(seed>0),0.],last_commands=[0.,float(seed>0),0.],
        dt_samples=[dt]*5,dt_index=0,dt_sum=f32(dt*5))
    if entry is not None:
        history['autotrim']=list(entry['history'])
        for key in ['trim_requested','trim_actual','trim_cache']:state[key]=list(entry['trim'])
        # The initializer supplies a starting state only. Every slow state
        # subsequently advances by the original elapsed-time recurrence.
        history['overload_timer']=f32(entry.get('overload_timer',0.))
        history['authority_factor']=f32(entry.get('authority_factor',1.))
    previous=None;error=float('inf');out=None;records=[];period=0
    ground=[model['fm']['AvailableControls'].get('has'+k+'TrimGroundControl',False) for k in ['Aileron','Elevator','Rudder']]
    effective=dict(controls,trim_available=trim_retained(controls['trim_available'],ground,1,True))
    ias=f32(state['longitudinal_speed']*float(f32(math.sqrt(f32(density_at_height(f32(state['wrapper']['height']))/state['rho0'])))))
    ranges=authority_ranges(controls,ias,elevator_state=state['elevator_state'])
    force_scale=max(1.,state['wrapper']['mass']*9.8100004196167)
    repeat_period=0
    ticks=0;accelerated_ticks=0;accelerated_factor_ticks=0;calls=0
    exact_states={};accelerated_cycle_ticks=0
    # This held observation keeps effective controls, delivered commands and
    # authority ranges invariant. Only native auto trim changes the inverse
    # allocation. Cache exact trim bytes (including signed zero), rather than
    # recomputing the same roll/yaw sticks and bounds on every controller tick.
    allocation_cache={}
    while ticks<max_iterations:
        iteration=ticks;ticks+=1;calls+=1
        out=keyboard_step(model,state,history,dt,predictor_cache=predictor_cache)
        timer=out['history']['overload_timer'];oldtimer=history['overload_timer']
        factor=out['history']['authority_factor'];oldfactor=history['authority_factor']
        if search_only:
            # Numerical initializer ONLY. solve() replays every accepted root
            # with native-rate histories and actual delivered force closure.
            if timer!=oldtimer:out['history']['overload_timer']=1. if timer>oldtimer else 0.
            if factor!=oldfactor:out['history']['authority_factor']=1. if factor>oldfactor else f32(.2)
            timer=out['history']['overload_timer'];factor=out['history']['authority_factor']
        trim=list(out['trim_requested'])
        # Roll/yaw are allocated only to maintain the specified coordinated
        # turn; full pitch stays +1 throughout the solve.
        allocation_key=struct.pack('<3d',*trim)
        if allocation_key not in allocation_cache:
            if len(allocation_cache)>=64:allocation_cache.clear()
            allocation_cache[allocation_key]=command_allocation(effective,state['delivered'],ranges,dict(DEFAULTS,trim_mode='fixed',fixed_trim=trim))
        allocation=allocation_cache[allocation_key]
        state['requested']=[allocation['sticks'][0],1.,allocation['sticks'][2]]
        state.update(trim_requested=trim,trim_actual=trim,trim_cache=out['trim_cache'],time_constants=out['time_constants'],linear_rates=out['linear_rates'])
        # Force histories are in newtons, commands dimensionless, angles in
        # degrees. Compare forces relative to weight; an ulp of a 100 kN tail
        # force must not be mistaken for a large control-command oscillation.
        histories=[out['history']['autotrim'],*out['history']['predictors']]
        flat=[*out['commands'],*trim,*out['trim_cache'],*state['requested'],*out['time_constants'],*out['linear_rates'],
            *[x for p in histories for x in [p[0],p[1]/force_scale,float(p[2])]],*out['history']['angles'],
            *out['history']['recovery'],*out['history']['last_commands'],out['history']['overload_timer'],out['history']['authority_factor'],out['history']['dt_sum']]
        if previous is not None:error=max(abs(a-b) for a,b in zip(flat,previous))
        previous=flat;history=out['history']
        current_delivered=steady_commands(effective,out['commands'],trim,ranges)
        records.append((flat,current_delivered,allocation['bounds']))
        records=records[-24:]
        # Once all predictor, trim, angle and response histories repeat,
        # authority adaptation is a scalar recurrence at this held source.
        # In inactive recovery, previous command/filter values are overwritten;
        # in cached active recovery, require those values to be invariant too.
        # Replay every rounded factor update, then run the complete controller
        # for the last skipped tick to restore its exact cache/filter outputs.
        fixed_inputs=(len(records)>=2 and records[-1][0][3:29]==records[-2][0][3:29]
                      and records[-1][0][-1]==records[-2][0][-1])
        stable_recovery=(not out['diagnostics']['recovery_active'] or
            state['command_cache_enabled'] and records[-1][0][:3]+records[-1][0][29:35]==records[-2][0][:3]+records[-2][0][29:35]) if len(records)>=2 else False
        if accelerate_history and fixed_inputs and stable_recovery and factor!=oldfactor and timer==oldtimer and all(x==dt for x in history['dt_samples']):
            diag=out['diagnostics'];args=(diag['peak'],diag['critical_high'],diag['reference'],dt,[f32(.0125),f32(.05)])
            if automatic_budget:
                endpoint=1. if factor>oldfactor else f32(.2)
                estimate=int(math.ceil(abs((endpoint-factor)/(factor-oldfactor))))+16
                max_iterations=max(max_iterations,min(2000000,ticks+estimate*2))
            factor,prior,skipped=authority_factor_run(factor,*args,max_iterations-ticks)
            ticks+=skipped
            if skipped:
                history['authority_factor']=prior
                history['dt_index']=(history['dt_index']+skipped-1)%len(history['dt_samples'])
                out=keyboard_step(model,state,history,dt,predictor_cache=predictor_cache);calls+=1
                history=out['history'];accelerated_factor_ticks+=skipped
                assert history['authority_factor']==factor
                state.update(trim_requested=list(out['trim_requested']),trim_actual=list(out['trim_requested']),
                    trim_cache=out['trim_cache'],time_constants=out['time_constants'],linear_rates=out['linear_rates'])
                previous=None;records=[];exact_states.clear();error=float('inf');continue
        # Before the overload interpolation band the timer does not affect
        # commands. If every other recurrence field is exactly unchanged,
        # replay just its original float32 additions up to the next event.
        # This preserves elapsed ticks and every intermediate timer rounding;
        # it never substitutes an endpoint or crosses a target-angle change.
        floor=min(map(f32,state['properties'].get('overloadTimeRange',[.9,1.])))
        if (accelerate_history and timer>oldtimer and timer<floor and factor==oldfactor
                and all(x==dt for x in history['dt_samples'])):
            from em_controller_cycle import periodic_timer_prefix
            skipped,advanced=periodic_timer_prefix(records,timer,
                f32(f32(state['properties'].get('overloadTimeRate',.45))*dt),floor,max_iterations-ticks)
            if skipped:
                timer=advanced;ticks+=skipped;accelerated_ticks+=skipped
                history['overload_timer']=timer;flat[-3]=timer
                history['dt_index']=(history['dt_index']+skipped)%len(history['dt_samples'])
        if (accelerate_history and len(records)>=2 and records[-1][0][:-3]+records[-1][0][-2:]==records[-2][0][:-3]+records[-2][0][-2:]
            and timer>oldtimer and timer<floor and factor==oldfactor
            and all(x==dt for x in history['dt_samples'])):
            step=f32(f32(state['properties'].get('overloadTimeRate',.45))*dt);skipped=0
            while ticks<max_iterations:
                next_timer=min(1.,f32(timer+step))
                if next_timer>=floor or next_timer==timer:break
                timer=next_timer;ticks+=1;skipped+=1
            if skipped:
                history['overload_timer']=timer;flat[-3]=timer
                history['dt_index']=(history['dt_index']+skipped)%len(history['dt_samples'])
                accelerated_ticks+=skipped
        if error<2e-6 and iteration>=3 and timer==oldtimer and factor==oldfactor:period=1;break
        # Native nested auto-trim iterations can have a repeatable cycle even
        # when every delivered surface is identical (e.g. pitch at its stop).
        # Accept only phase-INVARIANT delivery over three complete repeats.
        # No force averaging, altered commands or relaxed closure is involved.
        repeat_period=0
        for candidate in [2,3,4,8]:
            if len(records)<3*candidate:continue
            # This is a predicate, not a diagnostic maximum. Stop as soon as
            # any field disproves repetition; unsettled histories otherwise
            # spent most of their runtime comparing the rest of 24 snapshots.
            recurrence=history_repeats(records,candidate,2e-6)
            if not recurrence:continue
            if not repeat_period:repeat_period=candidate
            if all(abs(a-b)<2e-6 for rec in records[-3*candidate:] for a,b in zip(rec[1],current_delivered)):
                period=candidate;break
        if period:break
        if defer_cycles and repeat_period and timer==oldtimer and factor==oldfactor:break
        # A rejected delivery cycle cannot settle by replaying the same full
        # state thousands of times. Skip only exact whole cycles, preserving
        # the original observation budget and its final phase. Include every
        # mutable controller field, raw force histories, and the dt ring. Byte
        # comparison also distinguishes signed zero. The convergence window
        # must repeat too, so this cannot skip a later acceptance decision.
        if accelerate_history and not search_only and ticks>=24:
            raw=[*[x for name in ('requested','trim_requested','trim_actual','trim_cache','time_constants','linear_rates') for x in state[name]],
                 *history['autotrim'],*[x for p in history['predictors'] for x in p],
                 *history['angles'],history['overload_timer'],history['authority_factor'],
                 *history['recovery'],*history['last_commands'],*history['dt_samples'],history['dt_index'],history['dt_sum']]
            key=struct.pack('%sd'%len(raw),*raw)
            prior=exact_states.get(key)
            if prior is not None:
                window=[x for rec in records for part in (rec[0],rec[1],*[bounds for bounds in rec[2]]) for x in part]
                oldwindow=[x for rec in prior[1] for part in (rec[0],rec[1],*[bounds for bounds in rec[2]]) for x in part]
                if struct.pack('%sd'%len(window),*window)==struct.pack('%sd'%len(oldwindow),*oldwindow):
                    cycle=ticks-prior[0]
                    skipped=((max_iterations-ticks)//cycle)*cycle
                    ticks+=skipped;accelerated_cycle_ticks+=skipped
            if len(exact_states)>=128:exact_states.clear()
            exact_states[key]=(ticks,tuple(records))
    # A real update confirms stationarity, including endpoint timer/factor.
    check=keyboard_step(model,state,history,dt,predictor_cache=predictor_cache)
    drift=max(abs(a-b) for a,b in zip(check['commands']+check['trim_requested'],out['commands']+out['trim_requested']))
    slow_drift={key:check['history'][key]-history[key] for key in ['overload_timer','authority_factor']}
    drift=max(drift,*map(abs,slow_drift.values()))
    delivered=steady_commands(effective,check['commands'],check['trim_requested'],ranges)
    delivery_drift=max(abs(a-b) for a,b in zip(delivered,records[-1][1]))
    cycle_ok=period>1 and delivery_drift<2e-6 and not any(slow_drift.values())
    if cycle_ok:
        # All phases must supply the required roll/yaw and pitch authority.
        allocation['bounds']=[[max(rec[2][i][0] for rec in records[-period:]),min(rec[2][i][1] for rec in records[-period:])] for i in range(3)]
    sign=-1. if controls['invert_elevator'] else 1.
    margin=sign*(delivered[1]-state['delivered'][1])
    return dict(margin=margin,delivered=delivered,trim=check['trim_requested'],commands=check['commands'],
        entry_condition=copy.deepcopy(entry),slow_history_drift=slow_drift,
        elapsed_s=None if search_only else ticks*dt,history_exhausted=ticks==max_iterations,
        history_time_validated=not search_only,
        replay_ticks=ticks,controller_calls=calls,accelerated_timer_ticks=accelerated_ticks,
        accelerated_factor_ticks=accelerated_factor_ticks,
        accelerated_cycle_ticks=accelerated_cycle_ticks,
        converged=(period==1 and error<2e-6 and drift<2e-6 and not any(slow_drift.values())) or cycle_ok,
        drift=drift,history_error=error,iterations=ticks,
        history_period=period,delivery_drift=delivery_drift,delivery_cycle=[rec[1] for rec in records[-period:]] if period else [],
        repeat_period=repeat_period,repeat_delivery_cycle=[rec[1] for rec in records[-repeat_period:]] if repeat_period else [],
        repeat_control_bounds=[[max(rec[2][i][0] for rec in records[-repeat_period:]),min(rec[2][i][1] for rec in records[-repeat_period:])] for i in range(3)] if repeat_period else [],
        recovery_active=check['diagnostics']['recovery_active'],angle_limits=check['diagnostics']['angle_limits'],
        autotrim_success=check['diagnostics']['autotrim']['success'],state=state,history=history,result=check,
        lateral_margin=min(min(state['delivered'][i]-allocation['bounds'][i][0],allocation['bounds'][i][1]-state['delivered'][i]) for i in [0,2]),
        control_bounds=allocation['bounds'],
        authority_margin=min(min(d-lo,hi-d) for d,(lo,hi) in zip(state['delivered'],allocation['bounds'])),
        lateral_error=max(abs(delivered[i]-state['delivered'][i]) for i in [0,2]),
        lateral_authority=all(allocation['bounds'][i][0]-2e-7<=state['delivered'][i]<=allocation['bounds'][i][1]+2e-7 for i in [0,2]))
