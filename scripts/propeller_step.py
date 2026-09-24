"""Independent intact, connected type-2/type-8 fighter propeller update.

Selected noncyclic, noncoaxial geometry; no ground-screen augmentation.
Original 101a08de0 order: old-pitch blade load, new flow, governor, outputs.
"""
import math
from component_assembly import f32,add,sub,mul
from piston_model import div
from propeller_model import blade_forces


def sign(x):return 1. if x>0 else -1. if x<0 else 0.
def clamp(x,lo,hi):return min(max(x,f32(lo)),f32(hi))
def tiny(x):return 0. if abs(x)<f32(1.1920929e-7) else x


def step(p,state,velocity=(100.,0.,0.),body_omega=(0.,0.,0.),cg=(0.,0.,0.),
         omega=180.,previous_omega=None,target_omega=270.,command=1.,auto=False,
         density=1.225,sound_speed=340.,dt=1/60,afterburner=False,torque_gyro=True):
    omega,previous_omega,target_omega,command,density,sound_speed,dt=map(f32,
        [omega,omega if previous_omega is None else previous_omega,target_omega,command,density,sound_speed,dt])
    v=list(map(f32,velocity));w=list(map(f32,body_omega));r=p['position']
    local=[add(sub(mul(r[1],w[2]),mul(r[2],w[1])),v[0]),
           add(sub(mul(r[2],w[0]),mul(r[0],w[2])),v[1]),
           add(sub(mul(r[0],w[1]),mul(r[1],w[0])),v[2])]
    transverse_sq=add(mul(local[2],local[2]),mul(local[1],local[1]))
    transverse=f32(math.sqrt(transverse_sq))
    old_flow=state.get('flow',[0.,0.,0.]);pitch=f32(state.get('pitch',p['pitch_min']))
    gp=f32(state.get('governor_pitch',pitch))
    args=[omega,pitch,add(mul(add(old_flow[1],old_flow[0]),.5),local[0]),transverse,0.,0.,density,sound_speed,mul(old_flow[2],.25)]
    blade=blade_forces(p,*args);thrust=blade['thrust'];torque=blade['torque']
    disc=mul(mul(p['diameter'],p['diameter']),f32(.7853981852531433))
    radicand=add(div(add(thrust,thrust),mul(density,disc)),mul(sign(local[0]),mul(local[0],local[0])))
    flow=tiny(sub(mul(sign(radicand),f32(math.sqrt(abs(radicand)))),local[0]))
    flow=clamp(tiny(mul(sign(thrust),flow)),-150.,150.)
    axial=add(mul(flow,.5),local[0])
    flux=mul(mul(f32(math.sqrt(add(mul(axial,axial),transverse_sq))),disc),density)
    halfdiam=mul(p['diameter'],.5)
    divisor=mul(mul(halfdiam,halfdiam),mul(flux,.5))
    swirl=clamp(tiny(div(torque,divisor)) if abs(divisor)>f32(4e-19) else 0.,0.,20.)
    direction=1. if p['direction']==0 else -1.
    reaction=mul(add(torque,mul(div(sub(omega,previous_omega),dt),p['inertia'])),direction)
    momentum=mul(mul(omega,p['inertia']),direction)
    arm=[sub(x,y) for x,y in zip(r,map(f32,cg))]
    moment=[0.,sub(0.,mul(arm[2],thrust)),mul(arm[1],thrust)]
    inv_ratio=div(1.,p['reduction']);engine_omega=mul(inv_ratio,omega);previous_engine=mul(inv_ratio,previous_omega)
    # The slow governors use the raw squared-speed gains. GovernorSpeed
    # subsequently controls the lag between the governor and blade pitch.
    gain=mul(f32(.0001),p['reduction']);derivative=mul(p['reduction'],f32(.004))
    neutral=(f32(math.pi/2) if abs(omega)<f32(1e-5) else
             add(sub(f32(math.atan2(local[0],mul(mul(p['diameter'],.375),omega))),p['mean_twist']),
                 mul(div(p['polar']['base'][8],p['polar']['base'][1]),f32(-.01745329238474369))))
    if p['governor']==2 or auto:
        if p['governor']==2:
            maximum=p['boost_omega'] if afterburner else p['max_omega']
            if auto:command=clamp(div(sub(target_omega,p['min_omega']),sub(maximum,p['min_omega'])),0.,1.)
            target=add(mul(sub(maximum,p['min_omega']),command),p['min_omega'])
        else:target=target_omega
        now_sq=mul(engine_omega,engine_omega)
        error=sub(mul(gain,sub(mul(target,target),now_sq)),mul(derivative,sub(now_sq,mul(previous_engine,previous_engine))))
        limit=.35 if p['governor']==2 else 1.
        gp=sub(gp,mul(clamp(error,-limit,limit),dt))
        gp=min(max(gp,add(neutral,f32(-.12))),p['pitch_max'])
        if p['governor']==8:gp=min(max(gp,add(p['pitch_min'],f32(.05235988))),p['pitch_max'])
        pitch=clamp(add(pitch,mul(mul(p['governor_speed'],dt),clamp(mul(sub(gp,pitch),f32(.6)),-1.,1.))),p['pitch_min'],p['pitch_max'])
    else:
        gp=add(mul(sub(p['pitch_min'],p['pitch_max']),command),p['pitch_max'])
        delta=sub(gp,pitch);rate=mul(mul(p['governor_speed'],dt),sign(delta))
        pitch=clamp(add(pitch,min(rate,delta) if delta>=0. else max(rate,delta)),p['pitch_min'],p['pitch_max'])
    out=[0.]*40;out[0]=thrust;out[18]=torque
    out[20]=reaction if torque_gyro else 0.;out[23:26]=moment
    out[26]=momentum if torque_gyro else 0.
    normalized=mul(p['inv_max_omega'],engine_omega);out[29]=mul(normalized,normalized) if transverse==0. else 0.
    out[30]=p['inertia'];out[31]=command if p['governor']==2 or (auto and p['pitch_command_report']) else clamp(div(sub(p['pitch_max'],pitch),sub(p['pitch_max'],p['pitch_min'])),0.,1.)
    out[32]=flow;out[33]=swirl
    return dict(pitch=pitch,governor_pitch=gp,flow=[flow,0.,swirl],outputs=out,blade_inputs=[args])
