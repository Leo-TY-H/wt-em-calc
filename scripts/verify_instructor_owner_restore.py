"""Owner's final restore span, with original FM snapshot copy/restore.

Only actor transform publication is replaced by an explicit recording callback.
The original dynamic aircraft restore, memcpy, trim publication and snapshot
destructor execute. Check selected controller-relevant state, not every byte
of a spawned game actor.
"""
import json,random,struct
from pathlib import Path
from unicorn import UC_HOOK_CODE
from unicorn.x86_const import *
from instructor_native import InstructorNative,BASE,OWNER,OBJ,ARENA,STACK
from instructor_source import load
from aircraft_model import prepare
from mass_model import aircraft_properties,evaluate
from jet_catalog import fuel_capacities
from component_assembly import f32


def main():
    n=InstructorNative();u=n.u;rng=random.Random(926316);fails=[];cases=0;publication=[];bp=STACK-0x1000
    # 101a461c0 restores the body block; 104f70965 restores the separate
    # delivered record, then 104f70983 overwrites only its trim with final cache.
    fields={
        'position':(BASE+0x1558,3,'d',True),'quaternion':(BASE+0x1570,4,'f',True),
        'velocity':(BASE+0x15b0,3,'d',True),'acceleration':(BASE+0x15c8,3,'d',True),
        'stored_rates':(BASE+0x15e0,3,'d',True),'body_air':(BASE+0x1618,3,'d',True),
        'wing_angles':(BASE+0x1678,2,'f',True),'delivered_commands':(BASE+0x1694,3,'f',True),
        'requested_commands':(BASE+0x8514,3,'f',False),'requested_trim':(BASE+0x87f4,3,'f',False),
        'actual_trim':(BASE+0xa290,3,'f',False),'trim_cache':(BASE+0x3a24,3,'f',False),
        'response':(BASE+0x84fc,6,'f',False),'scalar_air_cache':(BASE+0x845c,4,'f',False),
        'engine_force':(OWNER+0x25a48,3,'d',False),'engine_moment':(OWNER+0x25a60,3,'d',False),
        'engine_wash':(OWNER+0x25b58,2,'f',False),'dispatch_cache':(OBJ+0xe0,3,'f',False),
        'recovery_history':(OBJ+0x130,3,'f',False),'angle_history':(OBJ+0x148,2,'f',False),
        'authority_overload_history':(OBJ+0x150,2,'f',False),
        'simulation_commands':(BASE+0x39f4,3,'f',False)}
    def publisher(u,a,size,data):
        assert u.reg_read(UC_X86_REG_ESI)==1 and u.reg_read(UC_X86_REG_ECX)==0 and u.reg_read(UC_X86_REG_R8D)==1
        publication.append(n.read(u.reg_read(UC_X86_REG_RDX),3));n.return_call()
    handle=u.hook_add(UC_HOOK_CODE,publisher,begin=0x104f712b0,end=0x104f712b0)
    names=['f_16a_block_15_adf','saab_jas39c','f_14a_early','mig_23m','harrier_gr3','kfir_c7']
    for name in names:
        fm=load(name);model=prepare(fm);mass=evaluate(aircraft_properties(fm),[f32(x*.3) for x in fuel_capacities(fm)])
        for i in range(12):
            n.setup(model,mass,speed=120+i*20,alpha=i*2,flaps=[0.,.37,1.][i%3],sweep=[0.,.4,1.][i%3],height=3000.)
            actor=n.empty_payload_actor();n.qword(actor+0x2ef0,BASE)
            # Initialize the actual owner snapshot, then copy with its original
            # snapshot producer, not an independently invented serial format.
            snap=bp-0x1580;n.invoke(0x101a23100,[snap]);n.invoke(0x101a33b10,[BASE,snap,0])
            original={k:n.read(p,count,kind) for k,(p,count,kind,restore) in fields.items()}
            saved_record=bytes(u.mem_read(BASE+0x2b08,0xee0));u.mem_write(bp-0x2460,saved_record)
            for key,(p,count,kind,restore) in fields.items():
                values=[rng.uniform(-.5,.5) for _ in range(count)]
                if kind=='d':n.doubles(p,values)
                else:n.floats(p,values)
            changed={k:n.read(p,count,kind) for k,(p,count,kind,restore) in fields.items()}
            # Separate delivered-command record is restored even though the
            # simulation command record and final request/filter state survive.
            n.floats(BASE+0x2b14,[.3,.4,.5]);n.floats(BASE+0x2b44,[-.7,-.8,-.9])
            u.reg_write(UC_X86_REG_R12,actor);u.reg_write(UC_X86_REG_RDI,BASE)
            u.reg_write(UC_X86_REG_RBP,bp);u.reg_write(UC_X86_REG_RSP,bp-0x4000)
            u.emu_start(0x104f708ee,0x104f709b3,count=1000000)
            assert u.reg_read(UC_X86_REG_RIP)==0x104f709b3
            for key,(p,count,kind,restore) in fields.items():
                actual=n.read(p,count,kind);expected=(original if restore else changed)[key]
                if actual!=expected:fails.append(dict(case=[name,i],field=key,actual=actual,expected=expected))
            expected_record=bytearray(saved_record)
            struct.pack_into('<3f',expected_record,0x3c,*changed['trim_cache'])
            actual_record=bytes(u.mem_read(BASE+0x2b08,0xee0))
            if actual_record!=bytes(expected_record):fails.append(dict(case=[name,i],field='delivered_record',offsets=[hex(j) for j,(a,e) in enumerate(zip(actual_record,expected_record)) if a!=e]))
            if publication[-1]!=list(map(f32,original['position'])):fails.append(dict(case=[name,i],field='published_position'))
            cases+=1
    u.hook_del(handle)
    report=dict(binary_sha256=n.sha,cases=cases,fields_per_case=len(fields),failure_count=len(fails),failures=fails[:15],
        restored_fields=[k for k,v in fields.items() if v[-1]],retained_fields=[k for k,v in fields.items() if not v[-1]],
        delivered_record='Full original saved record restored; trim at offset3c replaced by final FM3a24 cache.',
        scope='Original snapshot producer and owner restore span104f708ee..709b3, including original dynamic FM restore and cleanup, six prepared intact jet fixtures/72 cases. Actor transform publication104f712b0 is an explicit recording callback; no physics stepping or live/network initialization. Per-engine internals not independently compared by this test.')
    Path('analysis/instructor-full/owner-restore-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print('cases',cases,'fields',len(fields),'failures',len(fails),fails[:3])
    if fails:raise SystemExit(1)


if __name__=='__main__':main()
