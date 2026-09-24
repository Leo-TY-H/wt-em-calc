"""Solve conditional full-pitch balanced roots using original Windows predictors.

Research only: the entry controller history and actual trim are explicit, held
query parameters, not independently optimized aircraft coordinates. Root-query
order never advances elapsed time or retains a failed auto-trim output. This
does not establish which histories are attainable or replay moving owner trim.
"""
import argparse
import copy
import json
from pathlib import Path
import struct

from em_solver import TrimSolver, settings, command_allocation
from component_assembly import f32
from instructor_chart_inputs import source_state
from instructor_keyboard import fixed_source, keyboard_step
from instructor_protection import predicted_wing_angles
from instructor_settings import trim_retained
from primary_controls import steady_commands
from windows_instructor_native import WindowsInstructorNative
from instructor_native import BASE, OWNER, HEAP


class BalancedProbe:
    def __init__(self, native, name, config, entry_trim, timer, autotrim_history):
        self.native = native
        self.solver = TrimSolver(name, settings(dict(config, aircraft=[name],instructor=False,torque_gyro=False)))
        self.trim = list(map(f32, [0.,entry_trim,0.]))
        self.timer = f32(timer)
        self.autotrim_history = list(autotrim_history)
        self.points = {}
        native.setup(self.solver.model,self.solver.mass,flaps=self.solver.flaps,
                     sweep=self.solver.model['sweep'],height=self.solver.config['altitude_m'],mode_lane=True)

    def query(self, speed, load):
        key = float(speed), float(load)
        if key in self.points:return self.points[key]
        solver = self.solver
        earlier = [r for (v,_),r in self.points.items() if v==speed and r['point'].get('converged')]
        initial = min(earlier,key=lambda r:abs(r['point']['load_g']-load))['point']['solution'] if earlier else None
        point = solver.solve(speed,load,initial,exhaustive=False,refine=True)
        if not point['converged']:
            row = dict(point=point,status='physical solve unresolved',residual=None)
            self.points[key] = row
            return row
        value = solver.point_value(point)
        state = source_state(solver,value)
        for name in ('trim_requested','trim_actual','trim_cache'):state[name]=list(self.trim)
        state['wrapper'].update(parameter_pointer=0x107d6fba0,gameplay_pointer=HEAP+0x22b8)
        n = self.native
        for off,v in state['predictor']['f'].items():n.floats(BASE+off,[v])
        for off,v in state['predictor']['flags'].items():n.u.mem_write(BASE+off,bytes([bool(v)]))
        n.doubles(BASE+0x5358,[state['predictor']['pitch_inertia']])
        n.u.mem_write(OWNER+0x25f78,struct.pack('<I',state['predictor']['engine_count']))
        n.floats(0x107d6fbb4,[state['predictor']['balance_multiplier']])
        n.u.mem_write(0x107d6fbb8,bytes([state['predictor']['new_balance']]))
        fixed = fixed_source(solver.model,state)
        ground = [solver.fm['AvailableControls'].get('has'+axis+'TrimGroundControl',False) for axis in ('Aileron','Elevator','Rudder')]
        controls = dict(solver.controls,trim_available=trim_retained(solver.controls['trim_available'],ground,1,True))
        allocation = command_allocation(controls,state['delivered'],fixed['ranges'],
            dict(solver.config,trim_mode='fixed',fixed_trim=self.trim))
        state['requested'] = [allocation['sticks'][0],1.,allocation['sticks'][2]]
        adjusted = predicted_wing_angles(state['wing_angles'],[0.,0.],*state['delivered'][:2],fixed['sensitivity'],
            solver.model['controls']['Elevator']['wing_aoa'])['adjusted']
        dt = solver.dt
        history = dict(autotrim=list(self.autotrim_history),predictors=[[0.,0.,False],[0.,0.,False]],
            angles=adjusted,overload_timer=self.timer,authority_factor=1.,recovery=[0.,1.,0.],last_commands=[0.,1.,0.],
            dt_samples=[dt]*5,dt_index=0,dt_sum=f32(dt*5))
        before = copy.deepcopy(history)
        backend = lambda kind, packed, model, source, old: n.predict(packed,old)
        result = keyboard_step(solver.model,state,history,dt,predictor_backend=backend)
        assert history==before
        endpoints = steady_commands(controls,result['commands'],result['trim_actual'],fixed['ranges'])
        endpoint = endpoints[1]
        sign = -1. if solver.controls['invert_elevator'] else 1.
        residual = (endpoint-state['delivered'][1])*sign
        row = dict(point=point,status='evaluated',residual=residual,required_pitch=state['delivered'][1],
            full_pitch_endpoint=endpoint,requested_trim=result['trim_requested'],actual_trim=result['trim_actual'],
            delivered_residuals=[a-b for a,b in zip(endpoints,state['delivered'])],
            autotrim_success=result['diagnostics']['autotrim']['success'],
            autotrim_history_out=result['history']['autotrim'],controller_pitch=result['commands'][1],
            angle_targets=result['diagnostics']['angle_limits'],overload_timer_after=result['history']['overload_timer'],
            recovery_active=result['diagnostics']['recovery_active'],history_in=before,
            limitation='Actual trim is the explicit entry value. Moving owner/actuator elapsed time is not inferred.')
        self.points[key] = row
        return row

    def roots(self,speed,loads):
        samples = [self.query(speed,float(load)) for load in loads]
        roots = []; unresolved = []
        for left,right in zip(samples,samples[1:]):
            if left['residual'] is None or right['residual'] is None:
                unresolved.append([left['point']['load_g'],right['point']['load_g']]);continue
            if left['residual']*right['residual']>0:continue
            lo,hi = left,right
            resolved = True
            for _ in range(30):
                if hi['point']['load_g']-lo['point']['load_g']<1e-5:break
                mid = self.query(speed,(lo['point']['load_g']+hi['point']['load_g'])/2)
                if mid['residual'] is None:
                    unresolved.append([lo['point']['load_g'],hi['point']['load_g']]);resolved=False;break
                if mid['residual']*lo['residual']<=0:hi=mid
                else:lo=mid
            best = min((lo,hi),key=lambda r:abs(r['residual']))
            roots.append(dict(load_bracket=[lo['point']['load_g'],hi['point']['load_g']],
                endpoint_residuals=[lo['residual'],hi['residual']],best=best,
                balanced_full_pitch=bool(resolved and hi['point']['load_g']-lo['point']['load_g']<1e-5
                    and best['point']['valid'] and max(map(abs,best['delivered_residuals']))<2e-5)))
        return dict(roots=roots,unresolved_intervals=unresolved,samples=samples,
            scope='All sign-changing intervals on this sample grid were refined; tangencies or narrower components are not excluded. This is not a global-maximum certificate.')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--binary',type=Path,required=True)
    p.add_argument('--aircraft',default='j6k1');p.add_argument('--speed',type=float,default=771.03162178)
    p.add_argument('--flaps',type=float,default=30.);p.add_argument('--trims',default='0,-0.9374316930770874')
    p.add_argument('--loads',default='5,6,7,8,9,10,11,12,13')
    p.add_argument('--timer',type=float,default=0.)
    p.add_argument('--out',type=Path,default=Path('analysis/windows-instructor/windows-balanced-roots.json'))
    args=p.parse_args();n=WindowsInstructorNative(args.binary);runs=[]
    for trim in map(float,args.trims.split(',')):
        probe=BalancedProbe(n,args.aircraft,dict(flaps_percent=args.flaps),trim,args.timer,[0.,0.,False])
        result=probe.roots(args.speed,list(map(float,args.loads.split(','))))
        runs.append(dict(entry_trim=trim,entry_timer=args.timer,physical_solver_configuration=probe.solver.config,**result))
        print('trim',trim,'roots',[(r['best']['point']['load_g'],r['balanced_full_pitch']) for r in result['roots']],flush=True)
    version=json.loads(Path('references/data-version.json').read_text(encoding='utf-8'))
    report=dict(aircraft=args.aircraft,speed_kmh=args.speed,flaps_percent=args.flaps,
        data_version=version['version'],data_commit=version['commit'],windows_sha256=n.windows.sha256,
        fixture_binary_sha256=n.sha,
        controller_configuration=dict(instructor=True,full_pitch=1.,torque_gyro=False,
            flaps_fixed=True,configuration_switches=False),
        definition='Maximum balanced turn with full pitch input; this report resolves only explicit conditional roots.',
        scope=__doc__,global_boundary_validated=False,runs=runs)
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')


if __name__=='__main__':main()
