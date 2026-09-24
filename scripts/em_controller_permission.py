"""Replay every phase before granting periodic Instructor permission."""
import copy,math
from component_assembly import f32
from control_mixer import density_at_height
from primary_controls import authority_ranges,steady_commands
from instructor_settings import trim_retained
from instructor_keyboard import keyboard_step


def phase_diagnostics(aircraft,settled):
    from em_solver import command_allocation,DEFAULTS
    period=max(settled['history_period'],settled['repeat_period'])
    if period<=1:return [settled['result']['diagnostics']]
    state=copy.deepcopy(settled['state']);history=copy.deepcopy(settled['history'])
    controls=aircraft.controls;model=aircraft.model
    ground=[model['fm']['AvailableControls'].get('has'+k+'TrimGroundControl',False)
            for k in ('Aileron','Elevator','Rudder')]
    effective=dict(controls,trim_available=trim_retained(controls['trim_available'],ground,1,True))
    ias=f32(state['longitudinal_speed']*float(f32(math.sqrt(f32(
        density_at_height(f32(state['wrapper']['height']))/state['rho0'])))))
    ranges=authority_ranges(controls,ias,elevator_state=state['elevator_state'])
    expected=settled['repeat_delivery_cycle'] or settled['delivery_cycle']
    diagnostics=[];cache={}
    for index in range(period):
        result=(settled['result'] if index==0 else
                keyboard_step(model,state,history,aircraft.dt,predictor_cache=cache))
        trim=list(result['trim_requested'])
        delivered=steady_commands(effective,result['commands'],trim,ranges)
        if min(max(abs(a-b) for a,b in zip(delivered,phase)) for phase in expected)>2e-6:
            return None
        diagnostics.append(result['diagnostics'])
        allocation=command_allocation(effective,state['delivered'],ranges,
                                      dict(DEFAULTS,trim_mode='fixed',fixed_trim=trim))
        state['requested']=[allocation['sticks'][0],1.,allocation['sticks'][2]]
        state.update(trim_requested=trim,trim_actual=trim,trim_cache=result['trim_cache'],
                     time_constants=result['time_constants'],linear_rates=result['linear_rates'])
        history=result['history']
    return diagnostics
