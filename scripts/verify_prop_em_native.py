"""Original propulsion, airframe and airborne return at exported prop EM points.

Native loaded propulsion properties and actual geometry-backed mass feed the
independent EM solver. Each settled phase is then replayed through original
propulsion, detailed aero, body assembly and airborne integration. Frozen
fuel/thermal health and prepared airframe properties remain explicit boundaries.
"""
import argparse,json,math,struct
from pathlib import Path
import numpy as np
from unicorn import UC_HOOK_CODE
from unicorn.x86_const import UC_X86_REG_RSP,UC_X86_REG_RIP
from em_solver import AIRCRAFT,TrimSolver,ROOT
from propulsion_general_native import PropulsionGeneralNative
from propulsion_general import step
from verify_propulsion_general import differences
from verify_coupled_airborne import CoupledAirborne
from aircraft_model import evaluate
from kinematics import airborne_step,altitude_velocity_correction
from body_dynamics import G
from aircraft_upgrades_native import apply as apply_native_upgrades


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--points',default='analysis/prop-integration/coupled-first-probes.json')
    ap.add_argument('--report',default='analysis/prop-integration/em-native-validation.json');args=ap.parse_args()
    points=json.loads((ROOT/args.points).read_text());failures=[];counts={};records=[]
    def check(stage,actual,expected,where,tolerance=0.):
        counts[stage]=counts.get(stage,0)+1
        if tolerance:
            bad=abs(actual-expected)>tolerance
        else:bad=actual!=expected
        if bad:failures.append(dict(stage=stage,actual=actual,expected=expected,where=where,tolerance=tolerance))
    for point in points:
        name=point['aircraft'];AIRCRAFT[name]['supported']=True
        s=TrimSolver(name,dict(aircraft=[name],**point.get('settings',{})))
        value=s.point_value(point);p=value['propulsion'];geom=value['geometry'];speed=point['speed_kmh']/3.6
        period=len(p.get('cycle_samples') or [])
        if not period:raise ValueError('Native equilibrium check needs settled phase samples')
        check('export_replay_ps',value['ps'],point['ps_mps'],name,1e-9)
        engine=PropulsionGeneralNative();engine.configure(json.loads((ROOT/'references/jet-catalog/fm'/(AIRCRAFT[name]['fm_id']+'.blkx')).read_text()))
        apply_native_upgrades(engine,name)
        check('portable_properties',json.loads(json.dumps(engine.config)),s.engine.properties,name)
        m=CoupledAirborne()
        def powf(u,a,size,data):
            m.xmm(0,[math.pow(m.read_xmm(0)[0],m.read_xmm(1)[0])])
            sp=u.reg_read(UC_X86_REG_RSP);ret=struct.unpack('<Q',u.mem_read(sp,8))[0]
            u.reg_write(UC_X86_REG_RSP,sp+8);u.reg_write(UC_X86_REG_RIP,ret)
        m.u.hook_add(UC_HOOK_CODE,powf,begin=0x106e6199f,end=0x106e6199f)
        state=p.get('phase_start_state',p['state']);energies=[];rows=[];angular_rates=[];history=value['history_input']
        for phase in range(period):
            where=dict(aircraft=name,phase=phase)
            kw=dict(velocity=value['velocity'],height=s.config['altitude_m'],body_omega=geom['omega'].tolist(),cg=s.mass['cog'],dt=s.dt,nitro=s.mass['nitro_mass'],seed=state['seed'])
            original=engine.step(state,**kw);expected=step(s.engine.properties,state,**kw)
            check('propulsion_owner',differences(original,expected),{},where);state=expected
            row=original['aggregate_force']+original['aggregate_moment']+original['engine_angular_momentum']+original['engine_wash'];rows.append(row)
            m.initial_position=[0.,s.config['altitude_m'],0.];m.initial_quaternion=geom['quaternion']
            m.world_velocity=[speed,0.,0.];m.collision_radius=10.;m.tick=10;m.flex=[[0.,0.],[0.,0.]]
            aero_args=[s.model,value['velocity'],geom['omega'].tolist(),s.mass,value['allocation']['commands'],s.config['altitude_m'],s.dt,history]
            actual=m.call(*aero_args,throttle=s.config['throttle'],flaps=value['flaps'],gear=value['gear'],engine_wash=original['engine_wash'],
                          ground_height=s.config['altitude_m']-1e6,full_update=True)
            expected=evaluate(*aero_args,oil_radiator=0.,throttle=s.config['throttle'],flaps=value['flaps'],gear=value['gear'],height_agl=1e6,
                engine_vectors=(original['aggregate_force'],original['aggregate_moment']),engine_spin_factor=s.config['throttle'],
                engine_wash=original['engine_wash'],engine_angular_momentum=original['engine_angular_momentum'],quaternion=geom['quaternion'])
            check('component_forces',actual['forces'],{k:expected['component_forces'][k] for k in actual['forces']},where)
            check('aero_history',actual['history'],expected['history'],where)
            history=actual['history']
            assembly=m.extend(original['aggregate_force'],original['aggregate_moment'],[0.]*3,[0.]*3,1.,s.fm.get('ExtThrustBaseMult',1.),s.dt,
                              engine_angular_momentum=original['engine_angular_momentum'])
            check('body_force',assembly['force'],expected['force'],where);check('body_moment',assembly['moment'],expected['stored_moment'],where)
            m.integrate();actual=m.finish()
            predicted=airborne_step(m.initial_position,m.world_velocity,geom['quaternion'],expected['omega_for_flow'],expected['force'],expected['stored_moment'],s.mass['mass'],s.mass['inertia'],s.dt)
            predicted['velocity'][1]=altitude_velocity_correction(predicted['position'][1],predicted['velocity'][1],s.dt)
            for k in ['position','velocity','omega','quaternion','angular_acceleration','world_acceleration']:check('airborne_'+k,actual[k],predicted[k],where)
            error=max(abs(a-b) for a,b in zip(actual['omega'],geom['omega']))/s.dt
            angular_rates.append(actual['omega'])
            energy=((actual['position'][1]-s.config['altitude_m'])+(sum(v*v for v in actual['velocity'])-speed*speed)/(2*float(G)))/s.dt
            energies.append(energy)
        check('period_output_mean',np.mean(rows,axis=0).tolist(),p['force']+p['moment']+p['angular_momentum']+p['wash'],name)
        check('native_mean_angular_closure',max(abs(a-b) for a,b in zip(np.mean(angular_rates,axis=0),geom['omega']))/s.dt,0.,name,5e-5)
        check('native_mean_ps',sum(energies)/period,point['ps_mps'],name,.001)
        records.append(dict(aircraft=name,period_frames=p['period_frames'],sample_frames=period,
                            stationarity=p.get('stationarity'),ps_mps=point['ps_mps'],native_mean_ps=sum(energies)/period))
    report=dict(status='FAIL' if failures else 'PASS',counts=counts,records=records,failures=failures,scope=__doc__)
    (ROOT/args.report).write_text(json.dumps(report,indent=2)+'\n');print(report['status'],counts,flush=True)
    if failures:print(json.dumps(failures[:2],indent=2));raise SystemExit(1)


if __name__=='__main__':main()
