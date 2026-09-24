"""Constrained propulsion equilibrium under ideal cooling; not aircraft trim.

Native blade/inflow and pressure/torque math; operating roots are numerical.
Enumerate delivered 8-bit prop commands and legal compressor stages. Healthy
mechanical multiplier is one for the search; exact transient replay is separate.
"""
import json,math
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from component_assembly import f32,mul
from piston_model import inlet_pressure,compressor,rpm_torque,mixture,div,RAD_PER_RPM
from propulsion_model import prepare,target_omega
from propeller_step import step as prop_step
from air_state import cache,speed_of_sound


def mixture_command(ep,inlet):
    if ep['mixer_type']==0:return .5
    # The snapshot consumer quantizes mixture in .005 steps up to1.275.
    valid=[mul(i,f32(.005)) for i in range(1,256) if mixture(ep,inlet,mul(i,f32(.005)))['multiplier']==1. and not mixture(ep,inlet,mul(i,f32(.005)))['requires_stop']]
    if not valid:raise ValueError('No unpenalized delivered mixture')
    return valid[len(valid)//2]


def condition(p,speed,height,boost):
    ep=p['engine'];u=f32(speed);h=f32(height);inlet=inlet_pressure(h,u,ep['ram_recovery'])
    return dict(velocity=[u,0.,0.],height=h,throttle=f32(1.1 if boost else 1.),afterburner=boost,
                inlet=inlet,mixture=mixture_command(ep,inlet),density=cache([u,0.,0.],h)['density'],sound=speed_of_sound(h))


def nominal_engine(p,c,omega,gear):
    ep=p['engine'];r=compressor(ep,omega,c['throttle'],c['inlet'],1/60,gear=gear,regulator=-1.,afterburner=c['afterburner'])
    return mul(rpm_torque(ep,omega,c['throttle'],1.,c['afterburner'],gear),r['multiplier']),r


def solve(p,c,gear,command=None,automatic=False,initial=None):
    pp=p['prop'];fixed_rpm=pp['governor']==2 or automatic
    cmd=1. if command is None else command
    if fixed_rpm:
        omega=(target_omega(p['engine'],c) if automatic else f32(pp['min_omega']+f32((pp['max_omega']-pp['min_omega'])*cmd)))
        if omega<150.:return None
        lower,upper=pp['pitch_min'],pp['pitch_max'];initial=initial or [.65,15.,5.]
    else:
        pitch=f32(pp['pitch_max']+f32((pp['pitch_min']-pp['pitch_max'])*cmd));lower,upper=150.,p['engine']['omega_limit'];initial=initial or [270.,15.,5.]
    bounds=([lower,-149.,0.],[upper,149.,20.]);initial=np.maximum(np.minimum(initial,np.array(bounds[1])-1e-7),np.array(bounds[0])+1e-7)
    def values(x):
        w=omega if fixed_rpm else f32(x[0]);b=f32(x[0]) if fixed_rpm else pitch
        r=prop_step(pp,dict(pitch=b,governor_pitch=b,flow=[f32(x[1]),0.,f32(x[2])]),velocity=c['velocity'],omega=mul(w,pp['reduction']),density=c['density'],sound_speed=c['sound'],command=cmd,auto=automatic,target_omega=target_omega(p['engine'],c),afterburner=c['afterburner'])
        tq,engine=nominal_engine(p,c,w,gear)
        return r,tq,engine,w,b
    def residual(x):
        r,tq,_,_,_=values(x)
        return [(tq-mul(r['outputs'][18],pp['reduction']))/5000.,(r['flow'][0]-x[1])/30.,(r['flow'][2]-x[2])/10.]
    fit=least_squares(residual,initial,bounds=bounds,diff_step=2e-4,xtol=2e-8,ftol=2e-8,gtol=2e-8,max_nfev=65)
    r,tq,engine,w,b=values(fit.x);error=max(abs(v) for v in residual(fit.x))
    if error>3e-5:return None
    # A governed interior RPM root must also remain inside the governor's
    # neutral-pitch bounds. Check its actual one-step pitch update.
    if fixed_rpm and abs(r['pitch']-b)>2e-6:return None
    power=tq*w;thrust=r['outputs'][0]
    return dict(command=cmd,automatic=automatic,gear=gear,rpm=w/RAD_PER_RPM,pitch_deg=b*180/math.pi,
                thrust_N=thrust,shaft_power_W=power,efficiency=thrust*c['velocity'][0]/power if power else None,
                root_residual=error,root=fit.x.tolist(),omega=w,pitch=b,flow=r['flow'],regulator=engine['regulator'],mixture=c['mixture'])


def roots(p,c,gear,command=None,automatic=False,last=None):
    fixed=p['prop']['governor']==2 or automatic
    starts=([last] if last is not None else [])+([[x,15.,5.] for x in [.36,.6,.85,1.05,1.28]] if fixed else [[x,15.,5.] for x in [160.,220.,280.,310.]])
    rows=[]
    for initial in starts:
        row=solve(p,c,gear,command,automatic,initial)
        if row and not any(abs(row['omega']-old['omega'])<.02 and abs(row['pitch']-old['pitch'])<.0001 for old in rows):rows.append(row)
    return rows


def best(p,c):
    candidates=[];counts=dict(commands=0,accepted_roots=0,multiple_root_commands=0)
    for gear in range(len(p['engine']['stages'])):
        last=None
        for i in range(256):
            command=mul(float(i),f32(1/255));found=roots(p,c,gear,command,last=last);counts['commands']+=1
            if found:last=max(found,key=lambda x:x['thrust_N'])['root']
            candidates.extend(found);counts['accepted_roots']+=len(found);counts['multiple_root_commands']+=len(found)>1
        found=roots(p,c,gear,automatic=True);counts['commands']+=1
        candidates.extend(found);counts['accepted_roots']+=len(found)
    if not candidates:raise ValueError('No accepted operating root')
    winner=max(candidates,key=lambda x:x['thrust_N'])
    autos=[x for x in candidates if x['automatic']]
    return winner,max(autos,key=lambda x:x['thrust_N']) if autos else None,counts


def main():
    rows=[]
    for n in ['yak-3','bf-109f-4']:
        p=prepare(json.loads(Path('references/fm-2.59.0.13/'+n+'.blkx').read_text()))
        for h in [0.,3000.,6000.]:
            for u in [50.,100.,170.]:
                c=condition(p,u,h,n=='bf-109f-4');win,auto,counts=best(p,c)
                row=dict(aircraft=n,speed_mps=u,height_m=h,best=win,automatic_prop_with_best_gear=auto,search=counts);rows.append(row)
                print(n,h,u,round(win['thrust_N'],1),round(win['rpm'],1),win['command'],win['gear'],flush=True)
                Path('analysis/prop-performance.json').write_text(json.dumps(dict(scope='Maximum nominal axial propulsion among converged roots using multiple starting guesses for all256 delivered prop commands, each legal compressor gear and automatic prop mode. Full throttle (Yak1/Bf1.1WEP), ideal unpenalized quantized mixture, closed radiators, normal positive shaft-speed branch150rad/s through RPMMaxAllowed. Unit healthy mechanical multiplier for search. Does not include overspeed beyond RPMMaxAllowed, full-aircraft trim/drag optimization, or proof that every nonlinear operating root was found. Exact timestep replay is separate.',rows=rows),indent=2)+'\n')
if __name__=='__main__':main()
