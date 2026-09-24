"""Complete original 101a155c0 owner with one connected piston drivetrain."""
import struct
from unicorn import UC_HOOK_CODE
from unicorn.x86_const import *
from propulsion_native import PropulsionNative,TRAN,STATE
from piston_native import ENGINE,FM,SEED
from verify_aircraft_native import OWNER

class PropOwnerNative(PropulsionNative):
    def __init__(self):
        super().__init__();self.u.hook_add(UC_HOOK_CODE,self.zero,begin=0x106e61351,end=0x106e61351)
    def zero(self,u,a,n,d):
        self.calls.append(hex(a));u.mem_write(u.reg_read(UC_X86_REG_RDI),bytes(u.reg_read(UC_X86_REG_RSI)))
        sp=u.reg_read(UC_X86_REG_RSP);ret=struct.unpack('<Q',u.mem_read(sp,8))[0]
        u.reg_write(UC_X86_REG_RSP,sp+8);u.reg_write(UC_X86_REG_RIP,ret)
    def owner(self,p,s,velocity=(100.,0.,0.),height=0.,body_omega=(0.,0.,0.),cg=(0.,0.,0.),dt=1/60,seed=12345,torque_gyro=True):
        self.propulsion(p,s,velocity,height,body_omega,cg,dt,seed,torque_gyro,prepare_only=True);l=self.loader
        self.u.mem_write(OWNER,bytes(0x28000))
        # RB/SB engineControl: force, vortex and screen enabled; Arcade tangential floor disabled.
        l.write(0x107d6fbea,'5B',1,1,1,0,1)
        l.write(STATE+0x34,'f',1000.)  # Prepared free-air surface-distance history.
        for off,val in [(0x25a78,1),(0x25f78,1),(0x26000,1),(0x26088,1),(0x25b70,1),(0x25934,0)]:l.write(OWNER+off,'I',val)
        for off,val in [(0x25f80,ENGINE),(0x26008,STATE),(0x26090,TRAN)]:l.write(OWNER+off,'Q',val)
        l.write(OWNER+0x25b68,'f',1.);l.write(OWNER+0x25b74,'16f',*([0.,1.,0.,1000.]*4))
        l.write(OWNER+0x25a48,'6d',31.,-47.,12.,111.,333.,-222.);l.write(OWNER+0x25b40,'3d',71.,29.,11.)
        l.write(FM+0x15e0,'3d',*body_omega);l.write(FM+0x55a0,'Q',OWNER)
        l.write(FM+0x3658,'B',2 if torque_gyro else 0);l.write(FM+0x8470,'B',1)
        l.write(FM+0x8494,'f',height);l.write(FM+0x531c,'f',3000.)
        for reg,a in [(UC_X86_REG_RDI,OWNER),(UC_X86_REG_RSI,FM),(UC_X86_REG_RDX,SEED),(UC_X86_REG_RCX,0),(UC_X86_REG_R8,SEED+0x200)]:self.u.reg_write(reg,a)
        self.xmm(0,[dt]);l.run(0x101a155c0);r=self.result()
        r.update(aggregate_force=self.read(OWNER+0x25a48,3,'d'),aggregate_moment=self.read(OWNER+0x25a60,3,'d'),engine_angular_momentum=self.read(OWNER+0x25b40,3,'d'),propeller_force=self.read(OWNER+0x25a7c,3),engine_wash=self.read(OWNER+0x25b58,2),shake=self.read(OWNER+0x25b60,1)[0])
        return r
