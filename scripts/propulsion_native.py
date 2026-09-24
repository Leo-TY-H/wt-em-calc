"""Native complete one-engine/one-propeller transmission, fixed fuel/health."""
import json,struct
from pathlib import Path
from unicorn import UC_HOOK_CODE
from unicorn.x86_const import *
from piston_native import PistonNative,ENGINE,FM,SEED
from propeller_native import STATE,GROUND,BASE
from component_assembly import f32
from air_state import cache,speed_of_sound

TRAN=BASE+0x9000
TP=BASE+0xa000
CTX=BASE+0xb000
LISTS=BASE+0xc000


class PropulsionNative(PistonNative):
    def __init__(self):
        super().__init__()
        for a in [0x1019f7b50,0x1019f77a0,0x1019f7fc0]:
            self.u.hook_add(UC_HOOK_CODE,self.freeze,begin=a,end=a)
    def freeze(self,u,a,n,d):
        self.calls.append(hex(a));sp=u.reg_read(UC_X86_REG_RSP);ret=struct.unpack('<Q',u.mem_read(sp,8))[0]
        u.reg_write(UC_X86_REG_RSP,sp+8);u.reg_write(UC_X86_REG_RIP,ret)
    def propulsion(self,p,s,velocity=(100.,0.,0.),height=0.,body_omega=(0.,0.,0.),cg=(0.,0.,0.),dt=1/60,seed=12345,torque_gyro=True,prepare_only=False):
        self.configure(p['prop']);self.ep=p['engine']
        air=cache(velocity,height)
        self.step(s.get('prop',{}),velocity=velocity,body_omega=body_omega,cg=cg,omega=0.,command=s.get('command',1.),auto=s.get('auto',False),prepare_only=True)
        self.piston(dict(s.get('engine',{}),omega=s['omega']),velocity=velocity,height=height,dt=dt,seed=seed,prepare_only=True)
        l=self.loader
        l.write(TRAN,'Q',TP);l.write(TRAN+8,'2f',s['omega'],s.get('previous_omega',s['omega']));l.write(TRAN+0x10,'f',1.);l.write(TRAN+0x18,'f',1.)
        l.write(TP,'II2f',1,0,1.,1.);l.write(TP+0x38,'II2f',1,0,p['prop']['reduction'],p['inverse_reduction'])
        l.write(TP+0xb0,'B',p['auto_inertia']);l.write(TP+0xb4,'f',p['acceleration']);l.write(TP+0xb8,'B',p['correct_link'])
        l.write(CTX+8,'Q',FM);l.write(CTX+0x10,'9f',*velocity,*body_omega,*cg)
        l.write(CTX+0x34,'4B',torque_gyro,1,0,0);l.write(CTX+0x3c,'2f',air['density'],speed_of_sound(height))
        l.write(CTX+0x58,'f',1.)
        l.write(CTX+0x60,'Q',LISTS);l.write(LISTS,'Q',ENGINE)
        l.write(CTX+0x70,'Q',LISTS+0x10);l.write(LISTS+0x10,'Q',STATE)
        l.write(CTX+0x80,'Q',LISTS+0x20)
        l.write(CTX+0x90,'Q',LISTS+0x30);l.write(LISTS+0x30,'QI',GROUND,4)
        l.write(FM+0x1560,'d',height);l.write(FM+0x1618,'3d',*velocity);l.write(FM+0x16b8,'d',1.)
        l.write(FM+0x5320,'3f',*cg)
        for reg,a in [(UC_X86_REG_RDI,TRAN),(UC_X86_REG_RSI,CTX),(UC_X86_REG_RDX,SEED),(UC_X86_REG_RCX,SEED+0x200)]:self.u.reg_write(reg,a)
        self.xmm(0,[dt]);self.calls=[];self.blade_inputs=[]
        if prepare_only:return
        l.run(0x101a10db0)
        return self.result()
    def result(self):
        l=self.loader
        return dict(omega=l.read(TRAN+8)[0],previous_omega=l.read(TRAN+0xc)[0],outputs=l.read(TRAN+0x70,13),
                    engine=self.piston_result(),prop=dict(pitch=l.read(STATE+0x10)[0],governor_pitch=l.read(STATE+0x24)[0],flow=l.read(STATE+0x28,3),
                    outputs=l.read(STATE+0x60,40),blade_inputs=self.blade_inputs),elapsed=l.read(ENGINE+0x20)[0])


if __name__=='__main__':
    from propeller_native import properties as prop_properties
    from piston_wrapper import properties as engine_properties
    from piston_model import div
    m=PropulsionNative()
    for n in ['yak-3','bf-109f-4']:
        fm=json.loads(Path('references/fm-2.59.0.13/'+n+'.blkx').read_text());pp=prop_properties(fm)
        p=dict(prop=pp,engine=engine_properties(fm),inverse_reduction=div(1.,pp['reduction']),auto_inertia=n=='yak-3',correct_link=n=='yak-3',acceleration=1. if n=='yak-3' else 4.)
        print(n,m.propulsion(p,dict(omega=270.,auto=n=='bf-109f-4',prop=dict(pitch=.7))))
