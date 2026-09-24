"""Native full propeller-aircraft snapshot delivery on selected manual branch."""
from unicorn.x86_const import *
from prop_owner_native import PropOwnerNative
from propulsion_native import TRAN,TP
from propeller_native import STATE,PROPS
from piston_native import ENGINE,FM,SEED,PROP
from verify_aircraft_native import OWNER

class PropCommandsNative(PropOwnerNative):
    def delivery(self,cp,pp,snapshot,state,ranges,dt):
        s=dict(omega=270.,prop=dict(pitch=.5),engine=state['engine'],command=state.get('command',1.))
        self.propulsion(pp,s,prepare_only=True);l=self.loader
        self.u.mem_write(OWNER,bytes(0x28000))
        l.write(OWNER+0x25f78,'I',1);l.write(OWNER+0x25f80,'Q',ENGINE);l.write(OWNER+0x26090,'Q',TRAN);l.write(OWNER+0x26008,'Q',STATE)
        l.write(OWNER+0x25934,'I',0);l.write(PROP+0x249,'B',1);l.write(PROP+0x24b,'7B',1,1,1,0,pp['engine']['mixer_type']==2,1,pp['engine']['manual_compressor'])
        l.write(PROPS+0x369,'B',pp['prop']['auto_allowed'] if pp['prop']['governor']==2 else 0)
        l.write(FM+0x55a0,'Q',OWNER);l.write(FM+0x8528,'I',1);l.write(FM+0x2f58,'I',1)
        l.write(FM+0x8470,'B',1);l.write(FM+0x3658,'B',6);l.write(FM+0x6ef0,'Q',SEED+0x600)
        l.write(FM+0x7c0c,'3f',*cp['max_rate']);l.write(FM+0x7c54,'B',cp['invert_elevator'])
        for i,off in enumerate([0x7fb0,0x7fb4,0x7fb2]):l.write(FM+off,'B',cp['trim_available'][i])
        l.write(FM+0x2b14,'3f',*snapshot['commands']);l.write(FM+0x1694,'3f',*state['delivered'])
        l.write(FM+0x87f4,'3f',*state['trim_requested']);l.write(FM+0xa290,'3f',*state['trim_actual'])
        l.write(FM+0x2f5c,'B',snapshot['auto']);l.write(FM+0x2f68,'4f',snapshot['throttle'],snapshot['command'],snapshot['radiator'],snapshot['oil_radiator'])
        l.write(FM+0x2f78,'2I',3,snapshot['gear']);l.write(FM+0x2f80,'f',snapshot['mixture']);l.write(FM+0x2f88,'B',snapshot['afterburner'])
        l.write(SEED+0x100,'6f',*(ranges[0]+ranges[2]+ranges[1]))
        for reg,a in [(UC_X86_REG_RDI,FM),(UC_X86_REG_RSI,SEED),(UC_X86_REG_RDX,SEED+0x100),(UC_X86_REG_RCX,71),(UC_X86_REG_R8,SEED+0x200)]:self.u.reg_write(reg,a)
        self.xmm(0,[dt]);self.calls=[];l.run(0x101a4e5f0)
        e=dict(state['engine'],throttle=l.read(ENGINE+0xa4)[0],afterburner=bool(self.u.mem_read(ENGINE+0xbc,1)[0]),mixture=l.read(ENGINE+0xac)[0],gear=int.from_bytes(self.u.mem_read(ENGINE+0xb4,4),'little'),regulator=l.read(ENGINE+0x40)[0])
        return dict(delivered=l.read(FM+0x1694,3),trim_requested=l.read(FM+0x87f4,3),trim_actual=l.read(FM+0xa290,3),engine=e,command=l.read(STATE+0x48)[0],auto=bool(self.u.mem_read(STATE+0x4d,1)[0]),radiator=l.read(ENGINE+0x9c)[0],oil_radiator=l.read(ENGINE+0xa0)[0])
