"""Recovered primary-control authority and full-real trim/actuators.

Core authority/actuator/delivery stages match 6,000 original-function calls;
see primary-controls-validation.json for prepared-input and libm boundaries.

Axes in this public interface are roll, pitch, yaw. Values feeding the mixer
are normalized commands, not geometric deflection degrees. Caller/runtime
flags and crew-state authority are explicit; do not infer them from Arcade/RB.
"""
import math
from component_assembly import f32,add,sub,mul


def selected_properties(fm):
    a,e,r=[fm['Aerodynamics'][k] for k in ['Ailerons','Elevator','Rudder']]
    def ratio(x,y):return f32(x/y) if abs(y)>f32(4e-19) else 0.
    roll_common=add(add(add(f32(a['AnglesRoll'][1]),f32(a['AnglesRoll'][0])),f32(e['AnglesRoll'][0])),f32(e['AnglesRoll'][1]))
    yaw_common=add(*map(f32,e['AnglesYaw']))
    negative=[ratio(add(f32(r['AnglesRoll'][0]),roll_common),add(roll_common,f32(r['AnglesRoll'][1]))),
              ratio(add(add(f32(e['AnglesPitch'][0]),f32(a['AnglesPitch'][0])),f32(r['AnglesPitch'][0])),
                    add(add(f32(e['AnglesPitch'][1]),f32(a['AnglesPitch'][1])),f32(r['AnglesPitch'][1]))),
              ratio(add(add(f32(a['AnglesYaw'][0]),yaw_common),f32(r['AnglesYaw'][0])),add(add(yaw_common,f32(a['AnglesYaw'][1])),f32(r['AnglesYaw'][1])))]
    speed=lambda v:mul(f32(v),f32(.2777777910232544))
    ac=fm['AvailableControls']
    return dict(effective_speed=[[speed(fm['AileronEffectiveSpeed'])]*2,list(map(speed,fm['ElevatorsEffectiveSpeed'])),[speed(fm['RudderEffectiveSpeed'])]*2],
                power=[[f32(fm['AileronPowerLoss'])]*2,list(map(f32,fm['ElevatorPowerLoss'])),[f32(fm['RudderPowerLoss'])]*2],
                minimum=list(map(f32,[fm['AlphaAileronMin'],fm['AlphaElevatorMin'],fm['AlphaRudderMin']])),negative=negative,
                strong=fm['AllowStrongControlsRestrictions'],invert_elevator=fm['InvertElevator'],
                max_rate=list(map(f32,[fm['AileronMaxDv'],fm['ElevatorMaxDv'],fm['RudderMaxDv']])),
                trim_rate=[f32(ac['dv'+k+'Trim']) for k in ['Aileron','Elevator','Rudder']],
                trim_available=[ac['has'+k+'TrimControl'] for k in ['Aileron','Elevator','Rudder']])


def authority_ranges(p,speed,axis_enabled=(True,True,True),authority_scale=1.,full_loss=True,asymmetric=False,elevator_state=0.):
    """0x101a4adb0 with the normal r13=0 caller; returns three [low,high].

    full_loss is global107d6fba0+55; asymmetric is global107d6fc18+15.
    axis_enabled corresponds to FM8520/21/22 (autopilot-off flags, not damage);
    authority_scale is FM16a8 (pilot control-strength degradation).
    """
    speed=f32(speed);negative=p['negative'] if asymmetric else [1.]*3
    def power_limit(axis,side):
        effective=p['effective_speed'][axis][side]
        if speed<=effective:return 1.
        denominator=speed if full_loss else add(mul(sub(effective,speed),.5),speed)
        ratio=f32(effective/denominator) if abs(denominator)>f32(4e-19) else 0.
        return max(p['minimum'][axis],f32(math.pow(ratio,p['power'][axis][side])))
    roll=power_limit(0,0);yaw=power_limit(2,0)
    rr=[max(-1.,mul(-roll,negative[0])),roll] if axis_enabled[0] else [-1.,1.]
    yr=[max(-1.,mul(-yaw,negative[2])),yaw]
    if axis_enabled[1]:
        pr=[-power_limit(1,0),power_limit(1,1)] if full_loss else [-1.,1.]
        if abs(sub(*p['effective_speed'][1]))<f32(1e-6) and abs(sub(*p['power'][1]))<f32(1e-6):
            pr[0]=max(-1.,mul(-pr[1],negative[1]))
        if not p['strong']:
            if p['invert_elevator'] != (f32(elevator_state)>=0.):pr=list(yr)
            pr=[add(mul(pr[0],.5),-.5),add(mul(pr[1],.5),.5)]
    else:pr=[-1.,1.]
    if not axis_enabled[2]:yr=[-1.,1.]
    return [[mul(x,f32(authority_scale)) for x in r] for r in [rr,pr,yr]]


def trimmed_targets(sticks,trim,ranges):
    result=[]
    for command,t,(lo,hi) in zip(sticks,trim,ranges):
        command=min(1.,max(-1.,f32(command)));t=f32(t)
        value=lo if command<=-1. else hi if command>=1. else add(mul(add(mul(command,.5),.5),sub(hi,lo)),lo)
        product=mul(value,t);sign=1. if product>0. else -1. if product<0. else 0.
        result.append(add(mul(sub(1.,mul(abs(t),sign)),value),t))
    return result


def actuator_step(p,sticks,trim_requested,trim_actual,old_commands,ranges,dt,time_constants,linear_rates,
                  axis_enabled=(True,True,True),authority_scale=1.):
    """Full-model 0x101a4b230. Returns command states and actuated trim.

    time_constants=FM84fc/8500/8504, linear_rates=FM8508/850c/8510.
    old_commands are FM39f4/3a1c/3a20 (pitch/yaw before output clamping).
    """
    dt=f32(dt);trim=[]
    for i in range(3):
        step=mul(mul(f32(authority_scale),dt),p['trim_rate'][i])
        t,old=f32(trim_requested[i]),f32(trim_actual[i]);lo,hi=sub(old,step),add(old,step)
        trim.append(min(hi,max(lo,t)) if axis_enabled[i] else 0.)
    target=trimmed_targets(sticks,trim,ranges);state=[]
    for i,((lo,hi),old,t) in enumerate(zip(ranges,map(f32,old_commands),target)):
        factor=f32(math.exp(f32(-dt/f32(time_constants[i]))))
        delta=mul(sub(old,t),factor);smoothed=add(delta,t);linear=mul(f32(linear_rates[i]),dt)
        smoothed=max(sub(smoothed,linear),t) if delta>=0. else min(add(smoothed,linear),t)
        limit=mul(mul(mul(.5,dt),sub(hi,lo)),p['max_rate'][i])
        state.append(min(add(old,limit),max(sub(old,limit),smoothed)))
    return dict(trim=trim,state=state,commands=[state[0],min(1.,max(-1.,state[1])),min(1.,max(-1.,state[2]))],targets=target)


def delivered_commands(p,simulation_commands,previous,ranges,dt):
    """Full-model 0x101a4e450: simulation snapshot -> FM1694/98/9c.

    InvertElevator converts both the preceding and output pitch command.
    The caller's simulation snapshot/copy timing remains outside this helper.
    """
    result=[]
    for i,(desired,old,(lo,hi)) in enumerate(zip(simulation_commands,previous,ranges)):
        old=f32(old);desired=f32(desired)
        if i==1 and p['invert_elevator']:old=-old
        step=mul(add(mul(mul(p['max_rate'][i],.5),sub(hi,lo)),f32(.03)),f32(dt))
        value=min(add(old,step),max(sub(old,step),desired))
        result.append(-value if i==1 and p['invert_elevator'] else value)
    return result


def sensitivity_parameters(sensitivity):
    """0x101aabe30 after the aircraft-class provider selects three sensitivities.

    Each user sensitivity is normalized 0..1; actual inputs clamp at endpoints.
    This is command response smoothing, independent of aerodynamic power loss.
    """
    times=[];rates=[]
    for value in map(f32,sensitivity):
        if value<=0.:t,r=f32(.75),f32(.02)
        elif value>=1.:t,r=f32(.05),f32(.03)
        else:t,r=add(mul(f32(-.7),value),f32(.75)),add(mul(value,f32(.01)),f32(.02))
        times.append(t);rates.append(r)
    return dict(time_constants=times,linear_rates=rates)


def steady_commands(p,sticks,trim,ranges):
    """Selected intact-jet equilibrium, after availability and final inversion.

    Aerodynamic trim is normalized, with no independent geometric trim angle.
    Caller supplies supported normalized trim values and reachable ranges.
    This limit does not model simulation snapshot timing or crew degradation.
    """
    applied=[f32(t) if available else 0. for t,available in zip(trim,p['trim_available'])]
    commands=trimmed_targets(sticks,applied,ranges)
    commands[1:]=[min(1.,max(-1.,v)) for v in commands[1:]]
    if p['invert_elevator']:commands[1]=-commands[1]
    return commands
