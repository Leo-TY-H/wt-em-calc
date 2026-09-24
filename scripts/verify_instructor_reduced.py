"""Differential tests for recovered reduced-predictor stages, not a full port."""
import json,random
from pathlib import Path
from unicorn import UC_HOOK_CODE
from unicorn.x86_const import *
from instructor_native import InstructorNative,BASE,ARENA,OUTPUT
from instructor_reduced import inverse_rotated_cl,pitch_balance,tail_command,control_iteration,type2_tail_flow
from component_assembly import f32,mul,sub
from aircraft_model import prepare
from mass_model import aircraft_properties,evaluate
from jet_catalog import catalog,fuel_capacities
from instructor_source import load
from polar_runtime import evaluate as mach_polar
from polar_model import pack_polar,FIELDS


def verify_inverse(n):
    rng=random.Random(260920);failures=[];count=0;valid=0;clamps=0
    for name,item in catalog().items():
        if not item['supported']:continue
        model=prepare(load(name))
        for component in ['WingPlane','HorStabPlane']:
            for mach in [.4,1.1]:
                p=mach_polar(model['polars'][component][0][1],mach)
                low=mul(p['clKq'],p['cyCritL']);high=mul(p['clKq'],p['cyCritH'])
                targets=[low,high,f32((low+high)*.5)]+[f32(rng.uniform(low,high)) for _ in range(3)]
                for target in targets:
                    for convert in [False,True]:
                        angle=f32(rng.choice([-20.,-5.,0.,5.,20.]));initial=f32(-123.456)
                        n.u.mem_write(ARENA+0x8000,pack_polar(p));n.floats(OUTPUT,[initial])
                        n.invoke(0x10198c900,[ARENA+0x8000,int(convert),OUTPUT],[target,angle])
                        actual=[n.read(OUTPUT,1)[0],bool(n.u.reg_read(UC_X86_REG_RAX)&255)]
                        expected=list(inverse_rotated_cl(p,target,angle,convert,initial))
                        count+=1;valid+=actual[1];clamps+=actual[0] in [p['aoaCritH'],p['aoaCritL']]
                        if actual!=expected:failures.append(dict(case=[name,component,mach,target,angle,convert],actual=actual,expected=expected))
    report=dict(binary_sha256=n.sha,cases=count,converged=valid,critical_clamps=clamps,
                failures=failures[:20],failure_count=len(failures),
                scope='Independent rotated-polar inverse and native 15-step Newton algorithm; prepared polars.')
    Path('analysis/instructor-full/rotated-inverse-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print('rotated inverse',count,'failures',len(failures),flush=True)
    assert not failures,failures[:2]


def verify_balance(n):
    failures=[];counts=dict(captures=0,legacy_shift=0,tail_command=0,iteration=0,relaxed_last_pass=0,type2_wake=0);pending={};case=None
    def capture(u,a,size,data):
        bp=u.reg_read(UC_X86_REG_RBP);p=u.reg_read(UC_X86_REG_R15)
        def local(off):return n.read(bp-off,1)[0]
        if a==0x101a5f4cf:
            left=n.read(bp-0x2a0,2);right=n.read_xmm(0)[:2]
            pending['inputs']=dict(cd=[left[0],right[0]],cl=[left[1],right[1]],control_cl=local(0x280),
                extra_cd=[local(0x2f0),n.read_xmm(2)[0]],fuselage_cd=local(0x390),
                q_area=n.read(bp-0x7d0,2),negative_q_area=n.read(bp-0x7e0,2),cos_dihedral=local(0x26c),
                cm0=[local(0x80c),local(0x86c)],cm1=[local(0x808),local(0x868)],polar_area_over_span=local(0x2c8),
                wing_x=[local(0x480),local(0x5b0)],wing_y=[local(0x3cc),local(0x7a0)],
                other_moment=n.read_xmm(9)[0],engine_pitch_moment=n.read(p+0x64,1)[0],
                pitch_inertia=n.read(BASE+0x5358,1,'d')[0],target_acceleration=n.read(p+8,1)[0],
                weight=n.read(p+0x2c,1)[0],flight_path_cos=local(0x5c0),reference_x=n.read(p+0x1c,1)[0],
                tail_lever=local(0x520),working_alpha=local(0x300),
                legacy_balance_shift=n.u.mem_read(0x107d6fbb8,1)==b'\0' and n.u.mem_read(BASE+0x7c08,1)==b'\1')
        elif a==0x101a5f7f3 and 'inputs' in pending:
            inputs=pending.pop('inputs');expected=pitch_balance(**inputs)
            actual=dict(wing_force=n.read(bp-0x3f0,2),tail_force=local(0x2e0))
            counts['captures']+=1;counts['legacy_shift']+=inputs['legacy_balance_shift']
            if actual!=expected:
                failures.append(dict(case=case,actual=actual,expected=expected,inputs=inputs))
        elif a==0x101a60402:
            if 'wake' in pending:
                expected=pending.pop('wake');actual=n.read_xmm(0)[0];counts['type2_wake']+=1
                if actual!=expected:failures.append(dict(stage='type2_wake',case=case,actual=actual,expected=expected,inputs=pending.get('wake_inputs')))
            inputs=dict(flow_angle=n.read_xmm(0)[0],required_angle=local(0x3d8),
                flap_incidence=n.read(p+0x70,1)[0],tail_incidence=n.read(BASE+0x6f1c,1)[0],
                area_sensitivity_scale=local(0x2c0),has_sensitivity=bool(n.read(bp-0x910,1,'I')[0]),
                center=local(0x3c8),positive_delta=local(0x4d0),negative_delta=local(0x350),
                inverted=n.u.mem_read(BASE+0x7c54,1)!=b'\0')
            pending['tail']=tail_command(**inputs)
        elif a==0x101a604e3:
            actual=dict(command=n.read_xmm(4)[0],saturated=bool(u.reg_read(UC_X86_REG_R14)&1),deflection=n.read_xmm(1)[0])
            expected=pending.pop('tail');counts['tail_command']+=1
            if actual!=expected:failures.append(dict(stage='tail_command',case=case,actual=actual,expected=expected))
        elif a==0x101a6059e:
            index=u.reg_read(UC_X86_REG_RBX)&0xffffffff
            pending['iteration']=control_iteration(index,local(0x340),n.read_xmm(4)[0],
                saturated=bool(u.reg_read(UC_X86_REG_R14)&1),
                aileron_effect_range=mul(abs(sub(local(0x754),local(0x74c))),local(0x630)),
                elevon_effect_range=mul(abs(sub(local(0x5e4),local(0x5dc))),n.read(BASE+0x7ab8,1)[0]),
                direct_lift_authority=local(0x8e0))
            counts['relaxed_last_pass']+=index==9 and pending['iteration']['command']!=n.read_xmm(4)[0]
        elif a in [0x101a5e15e,0x101a60649] and 'iteration' in pending:
            actual=dict(done=a==0x101a60649,command=n.read_xmm(2 if a==0x101a60649 else 1)[0])
            expected=pending.pop('iteration');counts['iteration']+=1
            if actual!=expected:failures.append(dict(stage='iteration',case=case,actual=actual,expected=expected))
        elif a==0x101a5fb8c:
            owner=n.read(BASE+0x55a0,1,'Q')[0]
            inputs=dict(span=local(0x730),area=local(0x530),sweep=local(0x71c),taper=local(0x718),dihedral=local(0x714),
                working_alpha=local(0x300),wing_angle=local(0x250),
                wing_polars=[dict(zip(FIELDS,n.read(bp-off,24))) for off in [0x840,0x8a0]],
                wing_x=[local(0x2b4),local(0x3b0)],wing_y=[local(0x234),local(0x490)],wing_z=[local(0x450),local(0x540)],
                tail_point=n.read(BASE+0x6f20),tail_polar_offset=local(0xa30),coefficient=local(0x6b8),
                pitch_rate=n.read_xmm(4)[0],tail_lever=local(0x3bc),speed=local(0x4f0),sin_angle=local(0x214),cos_angle=local(0x218),
                engine_count=n.read(owner+0x25f78,1,'I')[0],clockwise=n.u.mem_read(BASE+0x6f3c,1)!=b'\0',
                axial_wash=n.read(bp-0x440,1,'d')[0],swirl_wash=n.read(bp-0x438,1,'d')[0],
                tail_area_delta=n.read(bp-0x658,1,'d')[0],inverse_tail_area=n.read(bp-0x580,1,'d')[0])
            pending['wake']=type2_tail_flow(**inputs);pending['wake_inputs']=inputs
    handles=[n.u.hook_add(UC_HOOK_CODE,capture,begin=a,end=a) for a in
             [0x101a5f4cf,0x101a5f7f3,0x101a60402,0x101a604e3,0x101a6059e,0x101a5e15e,0x101a60649,0x101a5fb8c]]
    for index,(name,item) in enumerate(catalog().items()):
        if not item['supported']:continue
        fm=load(name);model=prepare(fm);mass=evaluate(aircraft_properties(fm),fuel_by_system=[f32(v*.3) for v in fuel_capacities(fm)])
        for i in range(2):
            case=[name,i];n.setup(model,mass,speed=[120.,300.][i],alpha=[5.,25.][i])
            n.u.mem_write(0x107d6fbb8,bytes([i]))
            n.step()
        if index%75==0:print('balance',index,counts,'failures',len(failures),flush=True)
    for h in handles:n.u.hook_del(h)
    report=dict(binary_sha256=n.sha,counts=counts,failures=failures[:12],failure_count=len(failures),
        scope='Mode-1 reduced pitch balance, tail-command inversion and outer iteration at each reached stage. Full native predictor supplies prepared geometry, polar evaluations and wake; these tests are not a complete independent predictor.')
    Path('analysis/instructor-full/reduced-balance-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(counts,'failures',len(failures),failures[:1]);assert not failures


if __name__=='__main__':
    n=InstructorNative();verify_inverse(n);verify_balance(n)
