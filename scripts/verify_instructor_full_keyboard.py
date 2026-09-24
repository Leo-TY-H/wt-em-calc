"""Complete native keyboard-path executions, with explicit fixture limits.

Tests a general controller path across the supported jet catalog; it is not a
game-state capture, independent predictor validation, or settled EM boundary.
"""
import json
import math
from pathlib import Path
from unicorn import UC_HOOK_CODE
from unicorn.x86_const import UC_X86_REG_RSI
from instructor_native import InstructorNative, OBJ, INPUT, BASE
from instructor_protection import recovery_reference, protected_pitch_command, recovery_filter
from component_assembly import f32
from aircraft_model import prepare
from mass_model import aircraft_properties,evaluate
from jet_catalog import catalog,fuel_capacities
from instructor_source import load


def main():
    n=InstructorNative();failures=[];counts={};providers=set();probes=[];trace={}
    def check(kind,actual,expected,case):
        counts[kind]=counts.get(kind,0)+1
        if actual!=expected:failures.append(dict(kind=kind,case=case,actual=actual,expected=expected))
    def capture(u,a,size,data):
        if a==0x101a9594d:trace['after']=n.read(BASE+0x8514)
        else:
            p=u.reg_read(UC_X86_REG_RSI)
            trace.update(path='advanced' if a==0x101a98590 else 'simple',
                         enabled=list(u.mem_read(p+0x38,3)),before=n.read(BASE+0x8514))
    for a in [0x101a98590,0x101a9b7d0,0x101a9594d]:n.u.hook_add(UC_HOOK_CODE,capture,begin=a,end=a)
    supported=[name for name,p in catalog().items() if p['supported']]
    for index,name in enumerate(supported):
        fm=load(name);model=prepare(fm);mass=evaluate(aircraft_properties(fm),fuel_by_system=[f32(v*.3) for v in fuel_capacities(fm)])
        for case in range(4):
            key=[name,case];trace.clear();alpha=[4.,30.,-12.,18.][case]
            n.setup(model,mass,speed=[100.,250.,350.,180.][case],alpha=alpha,height=[0.,6000.,12000.,1000.][case])
            actor=n.empty_payload_actor();n.qword(actor+0x2ef0,BASE)
            commands=list(map(f32,[.15,-1. if case<2 else 1.,-.05]))
            n.floats(BASE+0x8514,commands);n.floats(OBJ+0xe0,[commands[1],commands[0],commands[2]])
            n.u.mem_write(OBJ+0x48,bytes([case%2]));n.floats(INPUT+0x44,[1.])
            n.u.mem_write(INPUT+0x29,b'\1');n.u.mem_write(INPUT+0xd,bytes([case>=2]))
            # Execute actual owner helper, rather than asserting its outcome.
            n.invoke(0x104f70ef0,[actor,INPUT+0x40,INPUT+0x41,INPUT+0x42])
            check('owner_keyboard_flags',list(n.u.mem_read(INPUT+0x40,3)),[0,0,0],key)
            try:a=n.step()
            except Exception as exc:
                failures.append(dict(kind='native_execution',case=key,error=str(exc)));continue
            check('mouseaim_invoked',bool(trace.get('after')),True,key)
            check('mouseaim_disabled',trace.get('enabled'),[0,0,0],key)
            check('keyboard_output_preserved',trace.get('after'),trace.get('before'),key)
            final=n.read(BASE+0x8514)
            check('final_commands_finite',all(math.isfinite(x) for x in final),True,key)
            check('two_pitch_predictors',len(a.get('predictor_inputs',[])),2,key)
            ref=recovery_reference(a['pitch_clamp_inputs']['critical_high'],n.read(BASE+0x1678,2),n.properties['critMult'],
                force_advanced=bool(case%2),mode_lane=case>=2)
            check('recovery_reference',a['pitch_clamp_inputs']['recovery_reference'],ref,key)
            check('clamp_port',a['pitch_clamp'],protected_pitch_command(**a['pitch_clamp_inputs']),key)
            if 'recovery_output' in a:
                check('recovery_port',a['recovery_output'],recovery_filter(**a['recovery_inputs']),key)
            if name in ['f_16a_block_15_adf','saab_jas39c','f_14a_early','mig_23m','harrier_gr3']:
                probes.append(dict(case=key,alpha=alpha,trace=dict(trace),final=final,recovery_active='recovery_output' in a,
                                   reference=ref,angle_limits=a['angle_limits']))
            providers.update(n.calls+n.extra_calls)
        if index%50==0:print('aircraft',index+1,'/',len(supported),'failures',len(failures),flush=True)
    report=dict(binary_sha256=n.sha,aircraft=len(supported),counts=counts,failures=failures[:30],failure_count=len(failures),
        probes=probes,adapters=sorted(providers),
        scope='Native main update, full-keyboard owner helper, both MouseAim paths and their snapshot/restore. Prescribed intact zero-payload actor and zero-time world. Differential checks cover reference, clamp, recovery and final disabled-axis preservation.',
        not_validated=['Spawned aircraft initialization and remaining caller mode identities','Reduced predictor independent port',
                       'MouseAim enabled-axis trajectory accuracy','Closed-loop aircraft reachability/stability','Production Instructor boundary'])
    Path('analysis/instructor-full/full-keyboard-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(counts,'failures',len(failures),failures[:3]);assert not failures


if __name__=='__main__':main()
