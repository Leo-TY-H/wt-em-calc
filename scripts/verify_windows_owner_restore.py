"""Execute the original Windows owner's final restore and trim publication.

Prepared intact aircraft and empty actor-owned providers. Only actor-transform
publication is a recording callback. Original snapshot construction, FM copy,
restore, delivered-record memcpy, trim publication and cleanup execute.
This tests selected fields, not owner scheduling or game initialization.
"""
import argparse
import json
from pathlib import Path
import random
import struct

from unicorn import UC_HOOK_CODE
from unicorn.x86_const import *
from windows_instructor_controller import WindowsInstructorController,ACTOR
from instructor_native import BASE,OWNER,OBJ,ARENA
from aircraft_catalog import catalog,load
from aircraft_model import prepare,at_sweep
from mass_model import aircraft_properties,evaluate
from jet_catalog import fuel_capacities
from prop_catalog import mass_state
from component_assembly import f32


FIELDS={
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


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--binary',type=Path,required=True)
    p.add_argument('--out',type=Path,default=Path('analysis/windows-instructor/windows-owner-restore-validation.json'))
    a=p.parse_args();n=WindowsInstructorController(a.binary);u=n.u;failures=[];rng=random.Random(9262026)
    publications=[];cases=0;frame=ARENA+0xb0000
    def publish(u,address,size,data):
        assert u.reg_read(UC_X86_REG_RCX)==ACTOR and u.reg_read(UC_X86_REG_RDX)&255==1
        assert u.reg_read(UC_X86_REG_R9)==0 and u.mem_read(u.reg_read(UC_X86_REG_RSP)+0x28,1)==b'\1'
        publications.append(n.read(u.reg_read(UC_X86_REG_R8),3));n.return_call()
    u.hook_add(UC_HOOK_CODE,publish,begin=0x140e5d280,end=0x140e5d280)
    names=['f_16xl','saab_jas39c','f_14a_early','go229_v3','j6k1','fw_200c_1']
    for name in names:
        fm=load(name);base=prepare(fm)
        mass=mass_state(name,30.) if catalog()[name]['propulsion']!='jet' else evaluate(aircraft_properties(fm),[v*.3 for v in fuel_capacities(fm)])
        for case in range(8):
            sweep=[0.,.4,1.,0.][case%4] if catalog()[name]['has_sweep'] else 0.
            n.setup_controller(at_sweep(base,sweep),mass,speed=120+case*20,alpha=case*2,flaps=[0.,.3,1.][case%3],sweep=sweep,height=3000.)
            snap=frame+0x1e0
            n.invoke_windows(0x142faefb0,(snap,));n.invoke_windows(0x142fd3980,(BASE,snap,0))
            original={k:n.read(ptr,count,kind) for k,(ptr,count,kind,restore) in FIELDS.items()}
            saved=bytes(u.mem_read(BASE+0x2b08,0xee0));u.mem_write(frame+0x1730,saved)
            for ptr,count,kind,restore in FIELDS.values():
                values=[rng.uniform(-.5,.5) for _ in range(count)]
                (n.doubles if kind=='d' else n.floats)(ptr,values)
            changed={k:n.read(ptr,count,kind) for k,(ptr,count,kind,restore) in FIELDS.items()}
            n.floats(BASE+0x2b14,[.3,.4,.5]);n.floats(BASE+0x2b44,[-.7,-.8,-.9])
            u.reg_write(UC_X86_REG_RSI,ACTOR);u.reg_write(UC_X86_REG_RSP,frame)
            u.emu_start(0x140e53e75,0x140e53f56,count=1000000)
            if u.reg_read(UC_X86_REG_RIP)!=0x140e53f56:raise RuntimeError('Owner span did not finish')
            for key,(ptr,count,kind,restore) in FIELDS.items():
                actual=n.read(ptr,count,kind);expected=(original if restore else changed)[key]
                if actual!=expected:failures.append(dict(aircraft=name,case=case,field=key,actual=actual,expected=expected))
            expected_record=bytearray(saved);struct.pack_into('<3f',expected_record,0x3c,*changed['trim_cache'])
            if bytes(u.mem_read(BASE+0x2b08,0xee0))!=bytes(expected_record):failures.append(dict(aircraft=name,case=case,field='complete delivered record'))
            if publications[-1]!=list(map(f32,original['position'])):failures.append(dict(aircraft=name,case=case,field='published position'))
            cases+=1
        print(name,cases,'cases',len(failures),'failures',flush=True)
    report=dict(scope=__doc__,windows_sha256=n.windows.sha256,cases=cases,fields_per_case=len(FIELDS),
        failures=failures,owner_span=['0x140e53e75','0x140e53f56'],
        restored_fields=[k for k,v in FIELDS.items() if v[-1]],retained_fields=[k for k,v in FIELDS.items() if not v[-1]],
        delivered_record='Full saved record restored, then trim at +3c replaced by final trim cache.',
        global_boundary_validated=False)
    a.out.parent.mkdir(parents=True,exist_ok=True);a.out.write_text(json.dumps(report,indent=2)+'\n')
    if failures:raise SystemExit(1)


if __name__=='__main__':main()
