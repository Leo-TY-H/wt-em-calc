"""Replay complete selected governor windows; check sampled phases in native code.

Captures the actual initialization of a fresh solve, including warm-start state.
No saved-point cold-start assumption and no phase shifted final-state replay.
"""
import argparse,copy,json,math,struct
from pathlib import Path
import numpy as np
from em_solver import TrimSolver,ROOT,evaluate
from component_assembly import f32
from prop_steady import initial_state
from propulsion_general import step
from propulsion_general_native import PropulsionGeneralNative
from verify_propulsion_general import differences
from aircraft_upgrades_native import apply as apply_native_upgrades
from verify_polar_machine_code import EXPECTED_BINARY_SHA256
import prop_em
import verify_aircraft_native
from macho_scan import MachO
from unicorn import UC_HOOK_CODE
from unicorn.x86_const import UC_X86_REG_RSP,UC_X86_REG_RIP


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('data',nargs='+');parser.add_argument('--report',required=True)
    args=parser.parse_args()
    binary=ROOT/'references/native'/('aces-'+EXPECTED_BINARY_SHA256)
    verify_aircraft_native.MachO=lambda path=None:MachO(path or binary)
    original=prop_em.settled_cycle
    def capture(*values,**options):
        supplied=copy.deepcopy(options.get('state'))
        result=original(*values,**options)
        result['_replay_input']=(values,supplied)
        return result
    prop_em.settled_cycle=capture
    reports=[];airframe_native=verify_aircraft_native.AircraftNative(binary)
    # Radial-aircraft tail flow calls the imported powf routine. Supply the
    # same float32 libm adapter used by verify_prop_em_native; this fixture
    # does not run the game's dynamic linker. Native aircraft code is intact.
    def powf(u,address,size,data):
        airframe_native.xmm(0,[math.pow(airframe_native.read_xmm(0)[0],airframe_native.read_xmm(1)[0])])
        sp=u.reg_read(UC_X86_REG_RSP);ret=struct.unpack('<Q',u.mem_read(sp,8))[0]
        u.reg_write(UC_X86_REG_RSP,sp+8);u.reg_write(UC_X86_REG_RIP,ret)
    airframe_native.u.hook_add(UC_HOOK_CODE,powf,begin=0x106e6199f,end=0x106e6199f)
    for filename in args.data:
        data=json.loads(Path(filename).read_text())
        for aircraft in data['aircraft']:
            name=aircraft.get('aircraft_id',aircraft['id']);solver=TrimSolver(name,aircraft['settings'])
            if not solver.is_prop:continue
            candidates=[p for p in aircraft['points'] if p['valid']]
            # Long observed cycles exercise the revised period search, while
            # the highest-speed level state covers the original flap failure.
            longest=max(candidates,key=lambda p:p['propulsion']['sample_frames'])
            level=max((p for p in candidates if p['load_g']==1.),key=lambda p:p['speed_kmh'])
            native=PropulsionGeneralNative()
            native.configure(json.loads((ROOT/'references/jet-catalog/fm'/(aircraft['fm_id']+'.blkx')).read_text()))
            apply_native_upgrades(native,name)
            assert json.loads(json.dumps(native.config))==solver.engine.properties
            branch=next((p for p in candidates if p.get('native_branch_search')),None)
            recovered=next((p for p in candidates if p.get('instructor_recheck')),None)
            residual_mean=next((p for p in candidates if (p['propulsion'].get('stationarity') or {}).get('window_alignment')=='aircraft force/moment error budget'),None)
            selected={(p['speed_kmh'],p['load_g']):p for p in (longest,level,branch,recovered,residual_mean) if p is not None}
            for saved in selected.values():
                point=solver.solve(saved['speed_kmh'],saved['load_g'],saved['solution'],exhaustive=False,detailed=True)
                assert point['valid'],(name,point['reasons'])
                value=point['_detail'];prop=value['propulsion'];rows=prop['cycle_samples']
                assert rows
                supplied,state=prop['_replay_input'];properties,velocity,height,omega,cg,dt,nitro,controls=supplied
                initial=initial_state(properties,velocity,height,nitro=nitro,**controls)
                if state is None:state=initial
                else:
                    for engine,s,c in zip(properties['engines'],state['engines'],initial['engines']):
                        for k in ('throttle','afterburner','mixture','automatic_turbo','automatic_mixture','automatic_compressor'):s[k]=c[k]
                        if engine['properties']['manual_compressor'] and not c['automatic_compressor']:s['gear']=c['gear']
                    for s,c in zip(state['propellers'],initial['propellers']):s['command']=c['command'];s['auto']=c['auto']
                frames=round(prop['simulated_seconds']/dt);start=frames-len(rows)
                selected=set(np.linspace(start,frames-1,min(9,len(rows)),dtype=int).tolist())
                replay=[];checked=0
                for frame in range(frames):
                    kw=dict(velocity=velocity,height=height,body_omega=omega,cg=cg,dt=dt,nitro=nitro,seed=state.get('seed',12345),torque_gyro=solver.config['torque_gyro'])
                    expected=step(properties,state,**kw)
                    if frame in selected:
                        actual=native.step(state,**kw);diff=differences(actual,expected)
                        assert not diff,(name,frame,diff)
                        checked+=1
                    state=expected
                    if frame>=start:replay.append(state['aggregate_force']+state['aggregate_moment']+state['engine_angular_momentum']+state['engine_wash'])
                assert replay==rows,(name,'Complete retained window did not replay exactly')
                assert [sum(r[i] for r in replay)/len(replay) for i in range(11)]==prop['force']+prop['moment']+prop['angular_momentum']+prop['wash']
                # Replay the aerodynamic consumer at every retained phase,
                # with its actual incoming history. Check selected phases in
                # the original executable, including native branch edges.
                history=value['history_input'];forces=[];moments=[];aero_checks=0
                checked_phases=set(np.linspace(0,len(rows)-1,min(9,len(rows)),dtype=int).tolist())
                for index,row in enumerate(rows):
                    aero_args=(solver.model,value['velocity'],value['geometry']['omega'].tolist(),solver.mass,
                        value['allocation']['commands'],solver.config['altitude_m'],solver.dt,history)
                    options=dict(flaps=value['flaps'],gear=value['gear'],throttle=solver.config['throttle'],
                        quaternion=value['geometry']['quaternion'],engine_wash=row[9:],torque_gyro=solver.config['torque_gyro'])
                    expected=evaluate(*aero_args,**options,oil_radiator=0.,height_agl=1e6,
                        engine_vectors=(row[:3],row[3:6]),engine_angular_momentum=row[6:9],
                        engine_spin_factor=f32(solver.config['throttle']))
                    if index in checked_phases:
                        actual=airframe_native.call(*aero_args,**options,ground_height=solver.config['altitude_m']-1e6)
                        assert actual['forces']=={k:expected['component_forces'][k] for k in actual['forces']},(name,index,'native aerodynamic forces')
                        assert actual['moment']==expected['raw_aero_moment'],(name,index,'native aerodynamic moment')
                        assert actual['history']==expected['history'],(name,index,'native aerodynamic history')
                        aero_checks+=1
                    forces.append(expected['force']);moments.append(expected['stored_moment']);history=expected['history']
                assert [sum(r[i] for r in forces)/len(forces) for i in range(3)]==value['result']['force']
                assert [sum(r[i] for r in moments)/len(moments) for i in range(3)]==value['result']['stored_moment']
                if (prop.get('stationarity') or {}).get('window_alignment')=='aircraft force/moment error budget':
                    half=len(rows)//2
                    force_delta=np.linalg.norm(np.mean(forces[:half],axis=0)-np.mean(forces[half:],axis=0))/solver.weight
                    angular_delta=abs(np.mean(moments[:half],axis=0)-np.mean(moments[half:],axis=0))/solver.mass['inertia']
                    assert value['force_mean_uncertainty_g']>=force_delta
                    assert np.all(value['angular_mean_uncertainty_rad_s2']>=angular_delta)
                    assert value['force_error_g']+value['force_mean_uncertainty_g']<=2e-4
                    assert max(abs(value['rate_residual'])+value['angular_mean_uncertainty_rad_s2'])<=5e-5
                reports.append(dict(aircraft=name,speed_kmh=point['speed_kmh'],load_g=point['load_g'],
                    phase_frames=len(rows),native_phase_checks=checked,native_aero_checks=aero_checks,
                    complete_aircraft_mean_exact=True,complete_portable_window_exact=True,
                    stationarity=prop.get('stationarity'),force_error_g=point['force_error_g'],
                    angular_error_rad_s2=point['angular_error_rad_s2'],saved_sep_difference_mps=abs(point['ps_mps']-saved['ps_mps'])))
                print(name,point['speed_kmh'],'PASS',len(rows),'exact replayed frames;',checked,'native phase checks',flush=True)
    Path(args.report).write_text(json.dumps(dict(status='PASS',binary_sha256=EXPECTED_BINARY_SHA256,
        scope=__doc__,cases=reports),indent=2)+'\n')


if __name__=='__main__':main()
