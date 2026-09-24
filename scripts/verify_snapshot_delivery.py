"""Complete original primary/engine snapshot delivery for selected clean jets."""
import json,random,struct
from pathlib import Path
from unicorn.x86_const import *
from macho_scan import MachO
from verify_engine_supply import EngineSupplyMachine,FM,ENGINE,PROP,OWN,SEED
from verify_jet_model import DATA
from primary_controls import selected_properties,authority_ranges
from control_snapshots import deliver_selected_jet_commands
from component_assembly import f32

OWNER=DATA+0x40000


class DeliveryMachine(EngineSupplyMachine):
    def __init__(self):
        super().__init__();m=MachO()
        for a,n in [(0x101a4e000,0x2000),(0x101a17000,0x1000),(0x101a56000,0x1000)]:
            self.u.mem_map(a,n);self.u.mem_write(a,m.read(a,n))
        self.u.mem_map(OWNER,0x30000)
    def delivery(self,p,snapshot,state,ranges,dt,auxiliary=(0.,0.),*,autotrim=False,autotrim_allowed=True,ground_trim=(False,False,False)):
        self.u.mem_write(DATA,bytes(0x10000));self.u.mem_write(FM,bytes(0xc000));self.u.mem_write(OWNER,bytes(0x30000))
        self.write(ENGINE,'2Q',PROP,OWN);self.write(PROP,'B',2);self.write(PROP+0x1e4,'I',3)
        self.write(PROP+0x249,'2B',1,1);self.write(ENGINE+0x1c,'B',state['running'])
        self.write(ENGINE+0xa4,'f',state['throttle']);self.write(ENGINE+0xbc,'B',state['afterburner'])
        self.write(ENGINE+0xc4,'2f',state['vtol'],state['reverse'])
        self.write(OWNER+0x25f78,'I',1);self.write(OWNER+0x25f80,'Q',ENGINE);self.write(OWNER+0x25934,'i',-1)
        self.write(FM+0x55a0,'Q',OWNER);self.write(FM+0x8528,'I',1);self.write(FM+0x2f58,'I',1)
        self.write(FM+0x8470,'B',1);self.write(FM+0x7c0c,'3f',*p['max_rate']);self.write(FM+0x7c54,'B',bool(p['invert_elevator']))
        for i,off in enumerate([0x7fb0,0x7fb4,0x7fb2]):self.write(FM+off,'B',bool(p['trim_available'][i]))
        self.write(FM+0x3658,'B',int(autotrim));self.write(0x107d6fbc0,'B',int(autotrim_allowed))
        for i,off in enumerate([0x7fb1,0x7fb5,0x7fb3]):self.write(FM+off,'B',int(ground_trim[i]))
        self.write(FM+0x2b14,'3f',*snapshot['commands']);self.write(FM+0x1694,'3f',*state['delivered'])
        self.write(FM+0x87f4,'3f',*state['trim_requested']);self.write(FM+0xa290,'3f',*state['trim_actual'])
        self.write(FM+0x2f68,'f',snapshot['throttle']);self.write(FM+0x2f88,'B',snapshot['afterburner'])
        self.write(FM+0x2f90,'2f',*auxiliary)
        self.write(SEED,'I',991337);self.write(SEED+0x100,'6f',*(ranges[0]+ranges[2]+ranges[1]))
        for reg,a in [(UC_X86_REG_RDI,FM),(UC_X86_REG_RSI,SEED),(UC_X86_REG_RDX,SEED+0x100),
                      (UC_X86_REG_RCX,71),(UC_X86_REG_R8,SEED+0x200)]:self.u.reg_write(reg,a)
        self.xmm(0,dt);self.run(0x101a4e5f0)
        result=dict(state,delivered=self.read(FM+0x1694,3),trim_requested=self.read(FM+0x87f4,3),
                    trim_actual=self.read(FM+0xa290,3),ab_indicator=bool(self.u.mem_read(FM+0x559d,1)[0]),
                    afterburner=bool(self.u.mem_read(ENGINE+0xbc,1)[0]),throttle=self.read(ENGINE+0xa4,1)[0],
                    vtol=self.read(ENGINE+0xc4,1)[0],reverse=self.read(ENGINE+0xc8,1)[0])
        if struct.unpack('<I',self.u.mem_read(SEED,4))[0]!=991337:raise AssertionError('Unexpected random draw')
        return result


def main():
    m=DeliveryMachine();rng=random.Random(193195);failures=[];counts=0
    def val(a,b):return f32(rng.uniform(a,b))
    for name in ['f_16a_block_15_adf','saab_jas39c']:
        fm=json.loads(Path('references/fm-2.59.0.13/'+name+'.blkx').read_text());p=selected_properties(fm)
        for i in range(1000):
            ranges=authority_ranges(p,val(50,700));dt=f32(rng.choice([1/30,1/48,1/60,1/120]))
            snapshot=dict(commands=[val(-1,1) for _ in range(3)],throttle=val(0,1.1),afterburner=bool(i%2))
            state=dict(delivered=[val(-1,1) for _ in range(3)],trim_requested=[val(-1,1) for _ in range(3)],
                       trim_actual=[val(-1,1) for _ in range(3)],throttle=val(0,1.1),afterburner=bool(i%3),running=7,
                       vtol=0.,reverse=0.)
            args=(p,snapshot,state,ranges,dt)
            a=m.delivery(*args,auxiliary=(val(0,1),val(0,1)));e=deliver_selected_jet_commands(*args);counts+=1
            if a!=e:failures.append(dict(aircraft=name,i=i,actual=a,expected=e))
    report=dict(binary_sha256=m.sha,complete_calls=counts,failures=failures,
        limitations='Full101a4e5f0 normal returns, all original selected callees, no executed hooks. Prepared full-real manual-trim snapshot and single engine/no transmission. No start/stop/autopilot requests. Unavailable VTOL/reverse setters are exercised with nonzero requests and retain zero states. Covers delivery and trim availability, not input producer or queue publication chronology.')
    Path('analysis/snapshot-delivery-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print('CALLS',counts,'FAILURES',len(failures));print(json.dumps(failures[:3],indent=2))
    if failures:raise SystemExit(1)

if __name__=='__main__':main()
