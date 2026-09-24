"""Native auto-trim payload-setting slice followed by mass-consumer comparisons.

Explicit prepared payloads, not a claim about a spawned aircraft's ammunition.
The producer and consumer are executed separately; owner payload enumeration
and network snapshot publication are not substituted with invented results.
"""
import json,random
from pathlib import Path
from unicorn.x86_const import *
from macho_scan import MachO
from component_assembly import f32
from verify_primary_controls import Controls,BASE
from verify_component_assembly import FRAME
from verify_mass_model import MassMachine
from mass_model import aircraft_properties,evaluate
from instructor_settings import payload_scales,restored_wing_normalization
from jet_catalog import catalog,fuel_capacities
from instructor_source import load
from aircraft_model import prepare,at_sweep


class SettingsMachine(Controls):
    def __init__(self):
        super().__init__();m=MachO()
        for address in [0x101a3f000,0x101a34000]:
            self.u.mem_map(address,0x1000);self.u.mem_write(address,m.read(address,0x1000))
    def scales(self,flags,base,instructor):
        self.reset();self.u.mem_write(BASE+0x3658,bytes([flags]))
        self.floats(BASE+0x1001c,[base]);self.floats(0x107d6fc74,[instructor])
        self.u.reg_write(UC_X86_REG_RDI,BASE);self.u.reg_write(UC_X86_REG_RSI,BASE+0x10000)
        self.u.reg_write(UC_X86_REG_RDX,42);self.u.reg_write(UC_X86_REG_RSP,FRAME)
        self.u.emu_start(0x101a3f8c0,0x101a3db30,count=100)
        assert self.u.reg_read(UC_X86_REG_RIP)==0x101a3db30
        assert self.u.reg_read(UC_X86_REG_ESI)==42
        return self.read3(BASE+0x54c8)[:2]

    def normalization(self,areas):
        self.reset();self.floats(BASE+0x8400,areas[0]+areas[1]);self.floats(BASE+0x18a4,[1.]*7)
        self.u.reg_write(UC_X86_REG_RBX,BASE)
        self.u.emu_start(0x101a34302,0x101a34430,count=200)
        return {o:self.read3(BASE+o)[0] for o in [0x8438,0x843c,0x8440]}


def main():
    n=SettingsMachine();m=MassMachine();rng=random.Random(393898);failures=[];counts=dict(producer=0,mass=0,normalization=0);examples=[]
    for i in range(1200):
        args=[i%256,f32(rng.uniform(0,1.5)),f32(rng.uniform(0,2.))]
        a=n.scales(*args);e=payload_scales(*args);counts['producer']+=1
        if a!=e:failures.append(dict(stage='producer',inputs=args,actual=a,expected=e))
    for name in ['f_16a_block_15_adf','saab_jas39c']:
        fm=load(name);p=aircraft_properties(fm);fuel=[f32(x*.3) for x in fuel_capacities(fm)]
        for i in range(100):
            # Exercise both CG formula options, including the source option.
            p['payload_affects_cog']=bool(i%2)
            payloads=[dict(mass=f32(rng.uniform(1,200)),position=[f32(rng.uniform(-4,4)) for _ in range(3)]) for _ in range(i%6)]
            modes=[]
            for flags in [2,3]:
                ns=n.scales(flags,.2,.25);ps=payload_scales(flags,.2,.25)
                a=m.evaluate(p,fuel,payloads=payloads,payload_cg_scale=ns[0],payload_inertia_scale=ns[1])
                e=evaluate(p,fuel,payloads=payloads,payload_cg_scale=ps[0],payload_inertia_scale=ps[1]);counts['mass']+=1
                bad={k:dict(actual=v,expected=e[k]) for k,v in a.items() if v!=e[k]}
                if bad:failures.append(dict(stage='mass',case=[name,i,flags],fields=bad))
                modes.append(dict(scale=ns[0],**a))
            assert modes[0]['mass']==modes[1]['mass']
            if not payloads:assert modes[0]['cog']==modes[1]['cog'] and modes[0]['inertia']==modes[1]['inertia']
            if i in [0,1,2]:examples.append(dict(aircraft=name,payload_affects_cog=p['payload_affects_cog'],payloads=payloads,modes=modes))
    for name,item in catalog().items():
        if not item['supported']:continue
        model=prepare(load(name))
        for sweep in [0.,.37,.73,1.]:
            areas=at_sweep(model,sweep)['geometry']['areas']
            a=n.normalization(areas);e=restored_wing_normalization(areas);counts['normalization']+=1
            if a!=e:failures.append(dict(stage='normalization',case=[name,sweep],actual=a,expected=e))
    report=dict(binary_sha256=n.sha,counts=counts,failures=failures,examples=examples,
        scope='Intact restored wing-area/normalization slice101a34302..34430, four sweep settings across all403 jets. Original101a3f8c0 through entry101a3db30 compared with independent scale producer; separate complete101998ad0/101998bf0 mass wrapper/consumer comparisons using producer outputs. Prepared intact nonadvanced mass, explicit payloads, fixed fuel. No loadout initialization or live mode capture.')
    Path('analysis/instructor-full/settings-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(counts,'failures',len(failures))
    if failures:raise SystemExit(1)

if __name__=='__main__':main()
