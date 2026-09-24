"""Density helper and complete mixer executions using selected aircraft tables.

This validates the mixer for aircraft-derived prepared tables. The loader itself
does not execute; raw BLK-to-runtime table preparation remains statically traced.
"""
import json, random, struct
from pathlib import Path
from unicorn.x86_const import *
from component_assembly import f32, mul
from control_mixer import density_at_height, selected_aircraft_properties, mix
from verify_control_mixer import Mixer
from verify_component_assembly import BASE, FRAME
from macho_scan import MachO


def main():
    machine=Mixer();m=MachO();failures=[];examples=[];rng=random.Random(391600)
    for va,size in [(0x101988000,0x1000),(0x107d6f000,0x1000)]:
        machine.u.mem_map(va,size);machine.u.mem_write(va,m.read(va,size))
    density_calls,mixer_calls=0,0
    for i in range(300):
        h=f32([-500,0,2000,6000,10000,18300,25000][i%7] if i<21 else rng.uniform(-500,30000))
        sp=FRAME-8;stop=BASE+0x2f000
        machine.u.mem_write(sp,struct.pack('<Q',stop));machine.u.reg_write(UC_X86_REG_RSP,sp)
        machine.xmm(0,[h]);machine.u.emu_start(0x1019881d0,stop,count=100)
        actual,expected=machine.read_xmm(0)[0],density_at_height(h);density_calls+=1
        if actual!=expected:failures.append(dict(stage='density',h=h,actual=actual,expected=expected))
    for name in ['f_16a_block_15_adf','saab_jas39c']:
        fm=json.loads(Path(f'references/fm-2.59.0.13/{name}.blkx').read_text())
        for component in ['Ailerons','Elevator','Rudder']:
            props=selected_aircraft_properties(fm['Aerodynamics'][component])
            for i in range(100):
                h=f32(rng.uniform(0,14000));speed=f32(rng.uniform(0,600));mach=f32(rng.uniform(0,2))
                command=[f32(rng.uniform(-1,1)) for _ in range(3)]
                args=(props,command,[bool(i&1),bool(i&2),bool(i&4)],0.,density_at_height(h),speed,mach,bool(i&8))
                actual,expected=machine.call(*args),mix(*args);mixer_calls+=1
                if actual!=expected:failures.append(dict(stage='mixer',aircraft=name,component=component,actual=actual,expected=expected))
            if component=='Elevator':
                for height in [0,6000,10000]:
                    # Raw mixer commands: upstream InvertElevator intentionally
                    # not applied, so these are not pilot-stick response claims.
                    args=(props,[0.,1.,0.],[True,False,False],0.,density_at_height(f32(height)),mul(900.,f32(1/3.6)),f32(.8),False)
                    values=machine.call(*args);mixer_calls+=1
                    if values!=mix(*args):failures.append(dict(stage='example',aircraft=name,height=height))
                    examples.append(dict(aircraft=name,height=height,raw_pitch_command=1.,speed_table_kmh=900,mach=.8,output=values))
    report=dict(binary_sha256=machine.sha,density_function_calls=density_calls,mixer_function_calls=mixer_calls,
                failures=failures,examples=examples,
                limitations='Static loader-inspired table preparation; loader not executed. Mixer uses prepared runtime command values, not pilot stick inputs. Density helper uses binary image defaults for rho0 and altitude ceiling.')
    Path('analysis/aircraft-controls-validation.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k!='failures'},indent=2))
    print('FAILURES',len(failures));print(json.dumps(failures[:4],indent=2))
    if failures:raise SystemExit(1)


if __name__=='__main__':main()
