"""Whole independent mode-1 predictor versus original instructions.

The only Python inputs captured from native execution are source FM scalar
fields, global parameters and the public input/history structs at entry. No
native predictor stack intermediate or output is used to calculate expected
results. Stack diagnostics are compared after the independent result exists.
"""
import json,random,struct,sys
from pathlib import Path
from unicorn import UC_HOOK_CODE
from unicorn.x86_const import *
from instructor_native import InstructorNative,BASE,OWNER,ARENA
from instructor_pitch_predictor import pitch_predictor,prepare_mode1,unpack_inputs
from component_assembly import f32
from aircraft_model import prepare
from jet_catalog import catalog,fuel_capacities
from instructor_source import load
from mass_model import aircraft_properties,evaluate


def source_state(n,ipaddr):
    fields=[0x843c,0x8440,0x8420,0x8424,0x8428,0x842c,0x6f20,0x6f24,0x6f28,0x5320,0x5324,
            0x18a0,0x83fc,0x18d4,0x8430,0x8434,0x18d8,0x8438,0x7334,0x712c,0x8448,0x7cc4,
            0x7140,0x7ca8,0x7cb4,0x7cb8,0x7cb0,0x7cc0,0x6f1c]
    flags=[0x7fa6,0x7c08,0x7c0a,0x7c54,0x8471,0x7fa3,0x6f3c]
    param=n.read(ipaddr+0x90,1,'Q')[0]
    return dict(f={o:n.read(BASE+o,1)[0] for o in fields},flags={o:bool(n.u.mem_read(BASE+o,1)[0]) for o in flags},
                pitch_inertia=n.read(BASE+0x5358,1,'d')[0],engine_count=n.read(OWNER+0x25f78,1,'I')[0],
                balance_multiplier=n.read(param+0x14,1)[0],new_balance=bool(n.u.mem_read(param+0x18,1)[0]),rho0=n.read(0x107d6f790,1)[0])


def main(limit=None,names=None,stress=False,report_path=None):
    n=InstructorNative();rng=random.Random(200926);current={};case=None;failures=[];count=0;preparation_failures=[];cases=0
    def hook(u,a,size,data):
        nonlocal count
        if a==0x101a5cac0:
            ipaddr=u.reg_read(UC_X86_REG_RSI);ip=unpack_inputs(bytes(u.mem_read(ipaddr,0xa0)))
            if ip[0]!=1:return
            out=u.reg_read(UC_X86_REG_RDX);hist=u.reg_read(UC_X86_REG_RCX)
            history=n.read(hist,2)+[bool(u.mem_read(hist+8,1)[0])]
            state=source_state(n,ipaddr);trace=[]
            expected=pitch_predictor(n.model,ip,state,history,trace)
            current.update(expected=expected,out=out,hist=hist,trace=trace,ip=ip,context=prepare_mode1(n.model,ip,state),iterations=[])
        elif a==0x101a5e15e:
            if not current:return
            bp=u.reg_read(UC_X86_REG_RBP)
            def scalar(off):return n.read(bp-off,1)[0]
            c=current['context'];expected={
                0x2c8:c['polar_area_over_span'],0x2c0:c['area_sensitivity_scale'],0x5c0:c['flight_path_cos'],
                0x3bc:c['tail_flow_lever'],0x520:c['tail_lever'],0x2b8:c['fuse_drag'],0x430:c['vstab_drag'],0x264:c['parasite_drag'],
                0x3cc:c['moment_wing_y'][0],0x7a0:c['moment_wing_y'][1],0x480:c['moment_wing_x'][0],0x5b0:c['moment_wing_x'][1],
                0x2b4:c['wing_x'][0],0x3b0:c['wing_x'][1],0x234:c['wing_y'][0],0x490:c['wing_y'][1],
                0x450:c['wing_z'][0],0x540:c['wing_z'][1],0x790:c['tail_gain'],0x630:c['ail_gain'],0x600:c['ail_sens'],
                0x510:c['tail_area'],0x4f0:c['speed'],0x2a4:c['wash_attenuation']}
            if (u.reg_read(UC_X86_REG_RBX)&0xffffffff)==0:
                bad={hex(o):[scalar(o),v] for o,v in expected.items() if scalar(o)!=v}
                if bad:preparation_failures.append(dict(case=case,fields=bad))
        elif a==0x101a6059e:
            if not current:return
            bp=u.reg_read(UC_X86_REG_RBP)
            current['iterations'].append(dict(index=u.reg_read(UC_X86_REG_RBX)&0xffffffff,
                current=n.read(bp-0x340,1)[0],angle=n.read(bp-0x268,1)[0],wing=n.read(bp-0x3f0,2),tail=n.read(bp-0x2e0,1)[0],
                required=n.read(bp-0x3d8,1)[0],command=n.read_xmm(4)[0],unmet=n.read_xmm(7)[0]))
            current['iterations'][-1].update(tail_drag=n.read(bp-0x360,1)[0],parasite_drag=n.read(bp-0x264,1)[0],vstab_drag=n.read(bp-0x430,1)[0])
        elif a==0x101a6080a:
            if not current:return
            h=current['hist'];actual=dict(output=n.read(current['out'],13),success=bool(u.reg_read(UC_X86_REG_RBX)&255),
                history=n.read(h,2)+[bool(u.mem_read(h+8,1)[0])])
            count+=1
            if actual!=current['expected']:
                failures.append(dict(case=case,input=current['ip'],actual=actual,expected=current['expected'],native_iterations=current['iterations'],port_iterations=current['trace']))
            current.clear()
    handles=[n.u.hook_add(UC_HOOK_CODE,hook,begin=a,end=a) for a in [0x101a5cac0,0x101a5e15e,0x101a6059e,0x101a6080a]]
    for name,item in catalog().items():
        if not item['supported']:continue
        if names and name not in names:continue
        fm=load(name);model=prepare(fm);mass=evaluate(aircraft_properties(fm),fuel_by_system=[f32(v*.3) for v in fuel_capacities(fm)])
        for i in range(2 if stress else 4):
            case=[name,i];speed=[120.,300.,180.,430.][i];alpha=[5.,25.,-8.,40.][i]
            if stress:
                n.setup(model,mass,speed=rng.uniform(45,620),alpha=rng.uniform(-30,50),flaps=rng.random(),sweep=rng.random(),height=rng.uniform(0,18000))
                n.floats(OWNER+0x25b58,[rng.uniform(-25,100),rng.uniform(-15,15)])
                n.doubles(OWNER+0x25a48,[rng.uniform(-10000,200000),rng.uniform(-10000,10000),rng.uniform(-10000,10000)])
                n.doubles(OWNER+0x25a60,[rng.uniform(-20000,20000) for _ in range(3)])
                n.floats(BASE+0x5320,[f32(mass['cog'][j]+rng.uniform(-.25,.25)) for j in range(3)])
                n.floats(BASE+0x843c,[rng.uniform(.8,1.2),rng.uniform(.8,1.2)])
                n.floats(BASE+0x8448,[rng.uniform(0.,1.)])
                n.floats(BASE+0x16a0,[rng.random()])
                n.u.mem_write(BASE+0x7fa3,bytes([i%2]));n.u.mem_write(BASE+0x8471,bytes([i%2]))
                n.u.mem_write(BASE+0x6f3c,bytes([i%2]));n.u.mem_write(BASE+0x3658,bytes([2 if i==0 else 0]))
                for off in [0x2b50,0x2b58]:n.floats(BASE+off,[rng.random()])
                # The count>1 predictor branch uses atan2f and excludes the
                # single-engine asymmetric wash. Provide a second complete
                # prepared engine record so original wrapper getters can run.
                if i==1:
                    second=ARENA+0xa0000
                    n.u.mem_write(second,bytes(n.u.mem_read(BASE+0x58000,0x1000)))
                    n.qword(OWNER+0x25f88,second)
                    n.u.mem_write(OWNER+0x25f78,struct.pack('<I',2))
            else:
                n.setup(model,mass,speed=speed,alpha=alpha,flaps=[0.,0.,.37,1.][i],sweep=[0.,0.,.4,1.][i],height=[0.,0.,2500.,7000.][i])
            n.u.mem_write(0x107d6fbb8,bytes([i%2]))
            n.doubles(BASE+0x15f0,[f32(rng.uniform(-.8,.8))])
            n.step();cases+=1
        if cases%100==0:print(cases,count,'calls; failures',len(failures),'preparation',len(preparation_failures),flush=True)
        if limit and cases>=limit:break
    for h in handles:n.u.hook_del(h)
    report=dict(binary_sha256=n.sha,cases=cases,predictor_calls=count,failures=failures[:8],failure_count=len(failures),
                preparation_failures=preparation_failures[:8],preparation_failure_count=len(preparation_failures),
                scope='Complete independent intact mode-1 keyboard reduced predictor, all 13 output fields, return status and conditional history writeback. Source FM state and input structs are prepared fixtures; no native stack intermediate is used as a port input.')
    report['stress']=stress
    report['prepared_engine_counts']=[1,2] if stress else [1]
    Path(report_path or 'analysis/instructor-full/pitch-predictor'+('-stress' if stress else '')+'-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(cases,count,'failures',len(failures),'preparation',len(preparation_failures),flush=True)
    if failures or preparation_failures:raise SystemExit(1)


if __name__=='__main__':
    args=[a for a in sys.argv[1:] if a!='--stress']
    main(int(args[0]) if args and args[0].isdigit() else None,
         args[0].split(',') if args and not args[0].isdigit() else None,stress='--stress' in sys.argv)
