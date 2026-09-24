"""Whole mode-0/1 checks for legacy-wake aircraft, or the supported fleet.

Prepared healthy sources and prescribed engine forces/wash, not live aircraft
or a turn-boundary validation. Native intermediates are diagnostics only.
The existing asymmetric mode-0 machine-code backend is counted separately:
its comparison tests fixture integration, not an independent reconstruction.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import random
import sys
from unicorn import UC_HOOK_CODE
from unicorn.x86_const import UC_X86_REG_RSI, UC_X86_REG_RDX, UC_X86_REG_RCX, UC_X86_REG_RBX

if '--compiled' in sys.argv:
    from em_backend import activate
    assert activate()=='compiled'
from aircraft_catalog import catalog, load
from aircraft_model import prepare
from component_assembly import f32
from instructor_native import InstructorNative, BASE, OWNER, INPUT, OUTPUT, OBJ
from instructor_pitch_predictor import pitch_predictor, unpack_inputs
from instructor_autotrim import autotrim_predictor
from verify_instructor_pitch_predictor import source_state
from mass_model import aircraft_properties, evaluate
from jet_catalog import fuel_capacities
from prop_catalog import mass_state


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',type=Path,default=Path('analysis/windows-instructor/legacy-wake-validation.json'))
    parser.add_argument('--compiled',action='store_true')
    parser.add_argument('--all-aircraft',action='store_true',help='Check all supported catalog aircraft, not just legacy wake and controls')
    parser.add_argument('--cases',type=int,choices=range(1,9),default=8,help='Number of prescribed scenarios per aircraft')
    args=parser.parse_args()
    native=InstructorNative();rng=random.Random(24092026)
    counts=Counter();failures=[];pending={};case=None;rows=[]
    def hook(u,address,size,data):
        if address==0x101a5cac0:
            pointer=u.reg_read(UC_X86_REG_RSI)
            ip=unpack_inputs(bytes(u.mem_read(pointer,0xa0)))
            hist=u.reg_read(UC_X86_REG_RCX)
            history=native.read(hist,2)+[bool(u.mem_read(hist+8,1)[0])]
            state=source_state(native,pointer)
            state['f'].update({o:native.read(BASE+o,1)[0] for o in (0x5328,0x79b8,0x79bc)})
            trace=[]
            expected=(pitch_predictor if ip[0] else autotrim_predictor)(native.model,ip,state,history,trace)
            pending.update(expected=expected,output=u.reg_read(UC_X86_REG_RDX),history=hist,
                           trace=trace,inputs=ip,mode=ip[0])
        elif address==0x101a6080a:
            hist=pending['history']
            actual=dict(output=native.read(pending['output'],13),success=bool(u.reg_read(UC_X86_REG_RBX)&255),
                        history=native.read(hist,2)+[bool(u.mem_read(hist+8,1)[0])])
            counts['mode_'+str(pending['mode'])]+=1
            if pending['mode']==0:
                # The translated symmetric branch always records its balance
                # and command passes. The native fallback does not emit trace.
                counts['mode_0_port' if pending['trace'] else 'mode_0_machine_backend']+=1
            counts['native_success' if actual['success'] else 'native_failure']+=1
            if actual!=pending['expected']:
                failures.append(dict(case=case,mode=pending['mode'],actual=actual,expected=pending['expected'],
                                     inputs=pending['inputs'],trace=pending['trace']))
            pending.clear()
    for address in (0x101a5cac0,0x101a6080a):
        native.u.hook_add(UC_HOOK_CODE,hook,begin=address,end=address)
    coverage=Counter();selected=[]
    for name,row in catalog().items():
        if not row['supported']:continue
        fm=load(name);model=prepare(fm)
        types=sorted({wing['geometry']['downwash_type'] for _,wing in model['wing_family']})
        coverage[str(types)]+=1
        if args.all_aircraft or types!=[2] or name in ('f_16xl','j6k1','saab_jas39c'):
            selected.append((name,row,fm,model,types))
    for name,row,fm,model,types in selected:
        mass=(evaluate(aircraft_properties(fm),fuel_by_system=[f32(v*.3) for v in fuel_capacities(fm)])
              if row['propulsion']=='jet' else mass_state(name,30.))
        for index in range(args.cases):
            case=[name,index]
            native.setup(model,mass,speed=[70.,120.,200.,300.][index%4],alpha=[-8.,4.,18.,32.][index%4],
                         flaps=[0.,.3,1.,0.][index%4],height=[0.,1500.,6000.,9000.][index%4])
            native.floats(OWNER+0x25b58,[f32(35. if index%2 else 0.),f32(12. if index%2 else 0.)])
            native.doubles(OWNER+0x25a48,[rng.uniform(1000,50000),rng.uniform(-200,200),0.])
            native.doubles(OWNER+0x25a60,[0.,0.,rng.uniform(-1000,1000)])
            native.doubles(BASE+0x15f0,[rng.uniform(-.8,.8)])
            native.u.mem_write(BASE+0x3658,bytes([2 if index%2 else 0]))
            native.u.mem_write(BASE+0x6f3c,bytes([int(index>=4)]))
            native.floats(INPUT,[-20.,20.]);native.floats(INPUT+16,[0.,0.,0.])
            for angle,accel in [(-12.,-.4),(25.,.7)]:
                native.floats(OUTPUT+0x100,[0.,0.]);native.u.mem_write(OUTPUT+0x108,b'\0')
                native.invoke(0x101a60b50,[BASE,0x107d6fba0,INPUT,INPUT+16,OUTPUT,OUTPUT+0x100],[angle,accel])
            native.floats(OBJ+0x15c,[rng.uniform(-5,10),rng.uniform(-5000,5000)])
            native.invoke(0x101a60830,[BASE,0x107d6fba0,OUTPUT,OBJ+0x15c],[1.])
        rows.append(dict(aircraft=name,downwash_types=types,propulsion=row['propulsion']))
        print(name,dict(counts),'differences',len(failures),flush=True)
    assert not pending, 'Native predictor did not reach its return hook'
    assert counts['mode_1']==len(rows)*args.cases*2, dict(counts)
    assert counts['mode_0']==len(rows)*args.cases, dict(counts)
    report=dict(binary_sha256=native.sha,backend='compiled' if args.compiled else 'python',
                all_aircraft=args.all_aircraft,cases_per_aircraft=args.cases,
                counts=dict(counts),catalog_wake_coverage=dict(coverage),
                aircraft=rows,failure_count=len(failures),failures=failures[:12],
                scope=__doc__,global_boundary_validated=False)
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    if failures:raise SystemExit(1)


if __name__=='__main__':main()
