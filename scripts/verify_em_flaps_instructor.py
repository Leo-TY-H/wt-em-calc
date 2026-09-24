"""Native flap integration and unchanged-equilibrium Instructor checks.

Instructor tests validate the advertised approximation, not the game controller.
"""
import json, math, struct, time
from pathlib import Path
from unicorn.x86_const import UC_X86_REG_RDI,UC_X86_REG_RSI,UC_X86_REG_RDX
from verify_wing_actuators import Actuators
from verify_component_assembly import BASE
from verify_aircraft_body_native import AircraftBodyNative
from aircraft_model import prepare,at_sweep,evaluate
from mass_model import aircraft_properties,evaluate as mass_evaluate
from jet_catalog import catalog,load,fuel_capacities
from component_assembly import f32
from component_assembly import mul
from wing_actuators import flap_mechanism_bounds,settled_flaps
from em_solver import TrimSolver,settings
from verify_general_nozzles import Nozzles
from engine_supply import mechanical_multiplier,available_fuel
import numpy as np

ROOT=Path(__file__).resolve().parents[1]


def native_bounds(machine,fm,mach,ias,sweep):
    m=machine;m.reset();p=fm.get('AvailableControls',{}).get('flapsLimits',{})
    m.u.reg_write(UC_X86_REG_RDI,BASE);m.u.reg_write(UC_X86_REG_RSI,2);m.u.reg_write(UC_X86_REG_RDX,1)
    m.qword(BASE+0x6ed8,BASE+0x10000)
    m.floats(BASE+0x8464,[mach,ias]);m.floats(BASE+0x16a4,[sweep]);m.floats(BASE+0x87c4,[sweep])
    m.floats(BASE+0x8834,[0.,1.])
    m.floats(BASE+0x7d20,p.get('mechRangeOnGround',[0.,1.])+[p.get('mechLockMachNumber',100.),p.get('mechLockIas',2147440000.)])
    indices={'gear':0,'sweep':1,'flaps':2,'airbrake':3,'bombBay':4,'chute':5,'vtol':6}
    m.u.mem_write(BASE+0x7d30,struct.pack('<i',indices.get(p.get('secondaryMech'),-1)))
    m.floats(BASE+0x7d34,p.get('secondaryMechRange',[0.,1.])+p.get('secondaryMechDependentRange',[0.,1.])+[p.get('forcedSecondaryMechValue',-1.)])
    m.u.mem_write(BASE+0x7d48,bytes([p.get('applySecondaryMechRequiredValue',True)]))
    m.run(0x101a4df10,0x101a4e17b,3000)
    return tuple(m.read_xmm(0)[:2])


def main():
    start=time.monotonic();act=Actuators();limits=Actuators();body=AircraftBodyNative();failures=[];counts={};examples=[]
    def check(stage,a,b,**where):
        counts[stage]=counts.get(stage,0)+1
        if a!=b:failures.append(dict(stage=stage,actual=a,expected=b,**where))
    for name,meta in catalog().items():
        if not meta['supported']:continue
        fm=load(name)
        for sweep in [0.,.5,1.]:
            for mach,ias in [(.3,100.),(.8,250.),(1.3,400.)]:
                a=native_bounds(limits,fm,mach,ias,sweep);b=flap_mechanism_bounds(fm,mach,ias,sweep)
                check('native_mechanism_range',a,b,aircraft=name,sweep=sweep,mach=mach)
        # Prepared geometry + original aero execution: all supported jets, two
        # nonzero deployed states (where controls exist), no solver interpolation.
        model=at_sweep(prepare(fm),0.)
        mass=mass_evaluate(aircraft_properties(fm),[f32(x*.3) for x in fuel_capacities(fm)])
        for request in [.25,1.]:
            flaps=settled_flaps(fm,request,.3,95.,0.)
            # Independently execute the original actuator on its held target.
            bounds=native_bounds(limits,fm,.3,95.,0.)
            if fm['AvailableControls']['hasFlapsControl'] and 'FlapsAxis' in fm['Aerodynamics']:
                a=act.flap(fm,request,flaps,.3,95.,f32(1/48),bounds)
                check('native_held_flaps',a['requested'],flaps,aircraft=name,request=request)
                check('native_flap_fixed_point',a['actual'],flaps,aircraft=name,request=request)
            hist=dict(wing_aoa=[0.,0.],body_angles=[0.,0.],wing_cl=[0.,0.],spin=0.)
            args=(model,[100.,-8.,0.],[.01,.02,.03],mass,[.03,.2,-.02],1000.,f32(1/48),hist)
            kw=dict(flaps=flaps,throttle=1.1)
            a=body.call(*args,**kw,ground_height=-1e6)
            b=evaluate(*args,**kw,oil_radiator=0.,height_agl=1001000.,engine_vectors=([0.]*3,[0.]*3),engine_spin_factor=f32(1.1))
            check('native_component_forces',a['forces'],{k:b['component_forces'][k] for k in a['forces']},aircraft=name,request=request)
            check('native_component_points',a['points'],{k:b['component_points'][k] for k in a['points']},aircraft=name,request=request)
            check('native_aero_moment',a['moment'],b['raw_aero_moment'],aircraft=name,request=request)
        if len(failures)>20:break
    # Both modes start at the same condition independently: neither commands
    # nor force/energy calculations may depend on the Instructor switch.
    for name in ['f_16a_block_15_adf','saab_jas39c','f_14a_early']:
        for flap in [0.,25.,100.]:
            off=TrimSolver(name,dict(aircraft=[name],flaps_percent=flap))
            on=TrimSolver(name,dict(aircraft=[name],flaps_percent=flap,instructor=True))
            for speed,load_g in [(400.,2.),(700.,4.),(1000.,10.)]:
                a=off.solve(speed,load_g);b=on.solve(speed,load_g)
                for key in ['solution','force_n','moment_nm','component_forces','ps_mps','flaps_percent']:
                    check('instructor_unchanged_'+key,a[key],b[key],aircraft=name,flap=flap,speed=speed,load=load_g)
                check('instructor_validity',b['valid'],a['valid'] and b['instructor']['converged'] and b['instructor']['margin']>=0,aircraft=name)
                examples.append(dict(aircraft=name,speed=speed,load_g=load_g,flaps_request=flap,actual_flaps=a['flaps_percent'],ps_mps=a['ps_mps'],valid_off=a['valid'],valid_on=b['valid']))
    nozzles=Nozzles()
    for name in ['buccaneer_s1','f_104a','f-8e_fn','dh_110']:
        solver=TrimSolver(name,dict(aircraft=[name]));speed=120.
        for flaps in [0.,.25,1.]:
            forces=[];moments=[]
            for unit in solver.engine.units:
                force=[];moment=[];ias=mul(speed,f32(math.sqrt(f32(unit.rho/f32(1.225)))))
                for state in unit.phase_states(speed):
                    health,_,_=mechanical_multiplier(unit.ep,state['omega'],state['health'],state['cylinders'],state['mechanical'],0.,state['torque'],state['friction'],unit.dt,state['_seed'])
                    throttle=state.get('effective_throttle',solver.config['throttle'])
                    a=nozzles.call(unit.jet,unit.nozzles,unit.rho,speed,state['omega'],min(f32(throttle/unit.jet['throttle_scale']),1.),unit.dt,solver.mass['cog'],ias,
                        flaps,0.,0.,0.,[0.]*3,False,[False]*3,afterburner=True,health=health,
                        fuel_available=available_fuel(unit.fp,unit.system_fuel,unit.accumulator,1.,unit.dt),shaft_fraction=mul(f32(state['omega']),unit.ep['inverse_omega']))
                    force.append(a['force']);moment.append(a['moment'])
                forces.append(np.mean(force,axis=0));moments.append(np.mean(moment,axis=0))
            check('native_flap_engine_assembly',solver.engine.vectors(speed,flaps),(np.sum(forces,axis=0).tolist(),np.sum(moments,axis=0).tolist()),aircraft=name,flaps=flaps)
    for bad in [dict(flaps_percent=-1),dict(flaps_percent=101),dict(flaps_percent=True),dict(flaps_percent=float('nan')),dict(instructor='yes')]:
        try:settings(bad);failures.append(dict(stage='bad_input_accepted',input=str(bad)))
        except ValueError:counts['invalid_settings_rejected']=counts.get('invalid_settings_rejected',0)+1
    report=dict(counts=counts,failures=failures,examples=examples,elapsed_s=time.monotonic()-start,
        binary_sha256=body.sha,limitations='Native mechanisms and aero use prepared properties and existing harness hooks. Instructor tests validate only the documented stationary upper-boundary construction, not native controller equivalence. Flap transitions, tear-off and forced-sweep deployment sequencing excluded.')
    (ROOT/'analysis/em-flaps-instructor-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(counts=counts,failure_count=len(failures),failures=failures[:5],elapsed_s=report['elapsed_s']),indent=2),flush=True)
    if failures:raise SystemExit(1)


if __name__=='__main__':main()
