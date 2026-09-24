"""Independent mode-0 predictor versus original complete instructions.

The independent result consumes only source FM properties and entry structs.
Native inner-iteration captures are diagnostics, never port inputs.
"""
import json,sys,random
from pathlib import Path
from unicorn import UC_HOOK_CODE
from unicorn.x86_const import *
from instructor_native import InstructorNative,BASE,OWNER,INPUT,OUTPUT,OBJ
from instructor_pitch_predictor import unpack_inputs
from instructor_autotrim import autotrim_predictor
from verify_instructor_pitch_predictor import source_state
from aircraft_model import prepare
from jet_catalog import catalog,fuel_capacities
from instructor_source import load
from mass_model import aircraft_properties,evaluate
from component_assembly import f32


def main(names=None,report_path=None):
    n=InstructorNative();rng=random.Random(209263);current={};failures=[];skipped=[];counts={'calls':0,'success':0,'native_failure':0};case=None
    def hook(u,a,size,data):
        if a==0x101a5cac0:
            ipaddr=u.reg_read(UC_X86_REG_RSI);ip=unpack_inputs(bytes(u.mem_read(ipaddr,0xa0)))
            if ip[0]!=0:return
            out=u.reg_read(UC_X86_REG_RDX);hist=u.reg_read(UC_X86_REG_RCX);history=n.read(hist,2)+[bool(u.mem_read(hist+8,1)[0])]
            state=source_state(n,ipaddr)
            state['f'].update({o:n.read(BASE+o,1)[0] for o in [0x5328,0x79b8,0x79bc]})
            trace=[]
            try:expected=autotrim_predictor(n.model,ip,state,history,trace)
            except ValueError as exc:
                skipped.append(dict(case=case,reason=str(exc)));return
            current.update(out=out,hist=hist,expected=expected,trace=trace,native=[])
        elif a==0x101a5f2ea:
            if not current:return
            bp=u.reg_read(UC_X86_REG_RBP)
            current['native'].append(dict(stage='balance',angle=n.read_xmm(8)[0],working=n.read(bp-0x300,1)[0],
                wing_angle=n.read(bp-0x3e0,1)[0],wing_force=list(reversed(n.read(bp-0x3f0,2))),tail_force=n.read(bp-0x2e0,1)[0],unmet=n.read_xmm(4)[0]))
        elif a==0x101a6059e:
            if not current:return
            bp=u.reg_read(UC_X86_REG_RBP)
            current['native'].append(dict(stage='command',current=n.read(bp-0x340,1)[0],required=n.read(bp-0x3d8,1)[0],proposed=n.read_xmm(4)[0],unmet=n.read_xmm(7)[0]))
        elif a==0x101a6080a:
            if not current:return
            h=current['hist'];actual=dict(output=n.read(current['out'],13),success=bool(u.reg_read(UC_X86_REG_RBX)&255),history=n.read(h,2)+[bool(u.mem_read(h+8,1)[0])])
            counts['calls']+=1;counts['success' if actual['success'] else 'native_failure']+=1
            if actual!=current['expected']:
                if len(failures)<12:failures.append(dict(case=case,actual=actual,expected=current['expected'],native=current['native'],port=current['trace']))
                counts['failures']=counts.get('failures',0)+1
            current.clear()
    handles=[n.u.hook_add(UC_HOOK_CODE,hook,begin=a,end=a) for a in [0x101a5cac0,0x101a5f2ea,0x101a6059e,0x101a6080a]]
    for name,item in catalog().items():
        if not item['supported'] or names and name not in names:continue
        fm=load(name);model=prepare(fm);mass=evaluate(aircraft_properties(fm),fuel_by_system=[f32(v*.3) for v in fuel_capacities(fm)])
        for i in range(4):
            case=[name,i];n.setup(model,mass,speed=[120.,300.,180.,430.][i],flaps=[0.,0.,.37,1.][i],sweep=[0.,0.,.4,1.][i],height=[0.,0.,2500.,7000.][i])
            n.floats(OBJ+0x15c,[rng.uniform(-5,10),rng.uniform(-15000,15000)])
            n.invoke(0x101a60830,[BASE,0x107d6fba0,OUTPUT,OBJ+0x15c],[1.])
        if counts['calls']%100==0:print(counts,flush=True)
    for h in handles:n.u.hook_del(h)
    report=dict(binary_sha256=n.sha,counts=counts,failures=failures,skipped=skipped,
        scope='Complete intact symmetric mode-0 one-g predictor; source state and entry structs only. Other branches explicitly excluded, not approximated.')
    Path(report_path or 'analysis/instructor-full/autotrim-predictor-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(counts,'skipped',len(skipped),flush=True)
    if counts.get('failures'):raise SystemExit(1)

if __name__=='__main__':main(sys.argv[1].split(',') if len(sys.argv)>1 else None)
