"""Original complete piston force wrapper with prepared intact running state."""
import json,struct,math
from pathlib import Path
from unicorn import UC_HOOK_CODE
from unicorn.x86_const import *
from propeller_native import PropellerNative
from verify_piston_model import PistonMachine,DATA,PROP,STATE,FM,STACK,STOP,OUT
from piston_wrapper import properties,step
from component_assembly import f32
from piston_model import inlet_pressure

ENGINE=STATE-0x10
OWN=DATA+0x7000
SEED=DATA+0x8000


class PistonNative(PropellerNative):
    def __init__(self):
        super().__init__();self.u.mem_map(DATA,0x31000)
        self.loader=object.__new__(PistonMachine);self.loader.u=self.u
        self.u.hook_add(UC_HOOK_CODE,self.pow_hook,begin=0x106e6199f,end=0x106e6199f)
    def pow_hook(self,u,a,n,d):
        self.calls.append(hex(a));self.xmm(0,[math.pow(self.read_xmm(0)[0],self.read_xmm(1)[0])])
        sp=u.reg_read(UC_X86_REG_RSP);ret=struct.unpack('<Q',u.mem_read(sp,8))[0]
        u.reg_write(UC_X86_REG_RSP,sp+8);u.reg_write(UC_X86_REG_RIP,ret)
    def configure_piston(self,p):
        self.loader.load(p);l=self.loader
        l.write(ENGINE,'2Q',PROP,OWN);l.write(ENGINE+0x10,'3B',1,1,1)
        for off,val in [(0xc,p['base_hp']),(0x154,p['omega_limit']),(0x16c,p['engine_inertia']),(0x1c0,p['ram_recovery'])]:l.write(PROP+off,'f',val)
        l.write(PROP+0x180,'3f',*p['amplitude']);l.write(PROP+0x244,'I',p['cylinders'])
        l.write(PROP+0x251,'B',p['manual_compressor'])
        l.write(PROP+0x24a,'B',p['boost_controllable']);l.write(PROP+0x1f8,'I',len(p['rpm_targets']))
        for i,row in enumerate(p['rpm_targets']):l.write(PROP+0x1fc+i*8,'2f',*row)
        l.write(PROP+0x268,'Q',OWN+0x100);l.write(PROP+0x278,'I',1);l.write(OWN+0x100,'3f',0.,0.,1.1)
        l.write(PROP+0x284,'8f',0.,1.1,1.,1.1,0.,1.,0.,1.)
        l.write(PROP+0x2a4,'4f',*p['consumption'])
        f=p['fuel'];l.write(FM+0x5158,'3f',f['minimal_load'],f['accumulator_flow'],f['engine_flow'])
        l.write(FM+0x53e4,'3f',200.,0.,f['capacity'])
        l.write(0x107d6fbea,'B',1);l.write(0x107d6fc10,'2B',0,1)
        self.ep=p
    def piston(self,s,velocity=(100.,0.,0.),height=0.,dt=1/60,seed=12345,torque_multiplier=1.,prepare_only=False):
        l=self.loader;self.configure_piston(self.ep);p=self.ep;self.calls=[]
        l.write(ENGINE+0x1c,'B',7);l.write(ENGINE+0x28,'f',s['omega']);l.write(ENGINE+0x30,'f',1.)
        l.write(ENGINE+0x38,'f',s.get('mechanical',1.));l.write(ENGINE+0x40,'f',s.get('regulator',-1.))
        l.write(ENGINE+0x58,'f',1.);l.write(ENGINE+0x6c,'I',p['cylinders'])
        l.write(ENGINE+0xa4,'f',s.get('throttle',1.));l.write(ENGINE+0xac,'f',s.get('mixture',.5))
        l.write(ENGINE+0xb0,'2I',3,s.get('gear',0));l.write(ENGINE+0xbc,'B',s.get('afterburner',False))
        l.write(ENGINE+0x124,'f',s.get('friction',0.));l.write(ENGINE+0x138,'f',s.get('extra_amplitude',0.))
        l.write(SEED,'I',seed);l.write(SEED+0x10,'3f',*velocity)
        for reg,val in [(UC_X86_REG_RDI,ENGINE),(UC_X86_REG_RSI,FM),(UC_X86_REG_RDX,SEED+0x10),(UC_X86_REG_RCX,SEED),(UC_X86_REG_R8,1),(UC_X86_REG_R9,ENGINE+0x94)]:self.u.reg_write(reg,val)
        for i,val in enumerate([dt,height,inlet_pressure(height,velocity[0],p['ram_recovery']),0.,0.,0.,1.,torque_multiplier]):self.xmm(i,[val])
        if prepare_only:return
        l.run(0x1019f3b80)
        return self.piston_result()
    def piston_result(self):
        l=self.loader
        return dict(torque=l.read(ENGINE+0x120)[0],friction=l.read(ENGINE+0x124)[0],regulator=l.read(ENGINE+0x40)[0],gear=struct.unpack('<I',self.u.mem_read(ENGINE+0xb4,4))[0],
                    throttle_ratio=l.read(ENGINE+0x3c)[0],potential_manifold=l.read(ENGINE+0x128)[0],manifold=l.read(ENGINE+0x12c)[0],
                    mechanical=l.read(ENGINE+0x38)[0],seed=struct.unpack('<I',self.u.mem_read(SEED,4))[0],extra_amplitude=l.read(ENGINE+0x138)[0],
                    consumption=l.read(ENGINE+0x94)[0],effective_throttle=l.read(ENGINE+0x14)[0],force=l.read(ENGINE+0x27c,3),moment=l.read(ENGINE+0x288,3),running=self.u.mem_read(ENGINE+0x1c,1)[0])


if __name__=='__main__':
    m=PistonNative()
    for n in ['yak-3','bf-109f-4']:
        p=properties(json.loads(Path('references/fm-2.59.0.13/'+n+'.blkx').read_text()));m.configure_piston(p)
        s=dict(omega=270.,throttle=1.,mixture=.5)
        a=m.piston(s);e=step(p,s);print(n,a);print('diff',[(k,a[k],e[k]) for k in e if a[k]!=e[k]])
