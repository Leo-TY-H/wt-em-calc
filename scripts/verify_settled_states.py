"""Verify control fixed points and healthy high-power rounded RPM equilibria."""
import copy,json,random
from pathlib import Path
from component_assembly import f32,mul
from primary_controls import (selected_properties,authority_ranges,actuator_step,
                              delivered_commands,sensitivity_parameters,steady_commands)
from verify_primary_controls import Controls
from verify_engine_supply import EngineSupplyMachine
from engine_supply import selected_properties as engine_properties,fuel_properties,wrapper_step
from jet_model import prepare,prepare_nozzle,steady


def main():
    c=Controls();e=EngineSupplyMachine();rng=random.Random(193194);dt=f32(1/48)
    failures=[];count=dict(control_steps=0,control_fixed_points=0,engine_steps=0,engine_fixed_points=0,engine_two_cycles=0);examples=[]
    for name in ['f_16a_block_15_adf','saab_jas39c']:
        fm=json.loads(Path('references/fm-2.59.0.13/'+name+'.blkx').read_text());p=selected_properties(fm)
        for i in range(12):
            speed=[100.,250.,450.,650.][i%4];sensitivity=[.1,.5,1.][i//4]
            response=sensitivity_parameters([sensitivity]*3);times=response['time_constants'];rates=response['linear_rates']
            ranges=authority_ranges(p,speed);sticks=[f32(rng.uniform(-.8,.8)) for _ in range(3)]
            trim=[f32(rng.uniform(-.4,.4)) if avail else 0. for avail in p['trim_available']]
            ns=ps=dict(trim=[0.]*3,state=[0.]*3,commands=[0.]*3);nd=pd=[0.]*3
            settled=False
            for tick in range(1,2001):
                old=copy.deepcopy([ns,nd])
                nd=c.delivery(p,ns['commands'],nd,ranges,dt);pd=delivered_commands(p,ps['commands'],pd,ranges,dt)
                ns=c.actuator(p,sticks,trim,ns['trim'],ns['state'],ranges,dt,times,rates,[True]*3,1.)
                ps=actuator_step(p,sticks,trim,ps['trim'],ps['state'],ranges,dt,times,rates)
                count['control_steps']+=1
                if nd!=pd or ns!={k:ps[k] for k in ns}:
                    failures.append(dict(stage='controls',aircraft=name,i=i,tick=tick));break
                if [ns,nd]==old:
                    settled=True;target=steady_commands(p,sticks,trim,ranges)
                    if nd!=target:failures.append(dict(stage='control_fixed_point',actual=nd,expected=target))
                    count['control_fixed_points']+=1;break
            if not settled:failures.append(dict(stage='control_did_not_settle',aircraft=name,i=i))
        ep=engine_properties(fm['EngineType0']);fp=fuel_properties(fm['Mass']);jet=prepare(fm['EngineType0']['Main']);nozzle=prepare_nozzle(fm['Engine0']['Nozzle0'])
        for throttle in [.5,1.,1.1]:
            ns=ps=dict(omega=mul(.6,jet['max_omega']),health=1.,cylinders=25,mechanical=1.,extra_amplitude=0.,
                       torque=0.,friction=0.,throttle=f32(throttle),running=7,afterburner=throttle>1.,vtol=0.,reverse=0.,
                       rpm_limit_scale=1.,elapsed=100.,inactive_elapsed=0.,stop_reason=0)
            nseed=pseed=123457;quiet=0;cycle_history=[]
            for tick in range(1,4001):
                old=ns['omega'];args=(ep,jet,nozzle,fp);tail=([250.,0.,0.],4500.,fm['Mass']['CenterOfGravity'],dt)
                na=e.wrapper(*args,ns,*tail,nseed,2000.,fp['capacity']);pa=wrapper_step(*args,ps,*tail,pseed,2000.,fp['capacity'])
                count['engine_steps']+=1
                if na!=pa:failures.append(dict(stage='engine',aircraft=name,throttle=throttle,tick=tick));break
                ns,nseed=na['state'],na['seed'];ps,pseed=pa['state'],pa['seed']
                cycle_history.append((ns['omega'],ns['mechanical'],nseed,na['force'][0]))
                quiet=quiet+1 if old==ns['omega'] and ns['mechanical']==1. else 0
                period=1 if quiet>=10 else 2 if len(cycle_history)>=20 and all(cycle_history[-i]==cycle_history[-i-2] for i in range(1,19)) else 0
                if period:
                    count['engine_fixed_points' if period==1 else 'engine_two_cycles']+=1
                    nominal=steady(jet,4500.,250.,throttle)
                    cycle=cycle_history[-period:];mean_thrust=sum(x[3] for x in cycle)/period
                    examples.append(dict(aircraft=name,throttle=throttle,steps=tick,
                        period_steps=period,cycle_omega=[x[0] for x in cycle],cycle_thrust=[x[3] for x in cycle],
                        actual_omega=ns['omega'],target_omega=na['target_omega'],actual_thrust=na['force'][0],
                        nominal_thrust=nominal['thrust'],mean_thrust=mean_thrust,relative_mean_difference=mean_thrust/nominal['thrust']-1.))
                    break
            else:failures.append(dict(stage='engine_did_not_settle',aircraft=name,throttle=throttle))
    report=dict(binary_sha256=e.sha,dt=dt,counts=count,examples=examples,failures=failures,
        limitations='Fixed supplied speed/height and authority/command snapshots, full health and constant fuel. Original actuator/delivery and full engine wrapper iterate independently of ports. Checks control fixed points and selected quiet mechanical regime, including a sustained two-step F16 afterburner cycle verified for20 repeats. Does not solve aircraft force/moment equilibrium. Rounded RPM can stop below mathematical target or cycle around it; nominal steady() is explicitly a diagnostic. Low-power random modulation requires stateful replay or a defined statistical averaging policy.')
    Path('analysis/settled-state-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(count,'FAILURES',len(failures));print(json.dumps(failures[:3],indent=2));print(json.dumps(examples,indent=2))
    if failures:raise SystemExit(1)

if __name__=='__main__':main()
