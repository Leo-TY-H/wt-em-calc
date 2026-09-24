"""Original complete running piston wrapper on actual loaded properties."""
import struct
from prop_config_native import PropConfigNative
from piston_config import loaded_properties
from piston_model import inlet_pressure
from engine_supply import fuel_properties
from propulsion_config import decode


class PistonGeneralNative(PropConfigNative):
    def configure_engine(self,fm,index=0):
        self.config=self.load_config(fm)
        self.instance=self.config+0x7710+index*0x19b0
        type_id=self.read(self.instance,1,'I')[0]
        self.properties=self.config+8+type_id*0x770
        self.p=loaded_properties(self,self.properties)
        self.description=decode(self,self.config)['engines'][index]
        self.engine=self.alloc(0x600);self.fm=self.alloc(0x9000);self.seed=self.alloc(64)
        self.qword(self.engine,self.properties);self.qword(self.engine+8,self.instance)
        self.qword(self.engine+0x320,self.properties+0x578);self.qword(self.engine+0x328,self.instance+0x28)
        self.u.mem_write(self.engine+0x10,b'\x01\x01\x01');self.u.mem_write(self.fm+0x8470,b'\x01')
        self.u.mem_write(self.fm+0x3658,b'\x04')
        self.u.mem_write(0x107d6fbea,b'\x01');self.u.mem_write(0x107d6fc10,b'\x00\x01')
        system=self.read(self.instance+0x1c,1,'I')[0];fuel=fuel_properties(fm['Mass'],system)
        self.floats(self.fm+0x5158+28*system,[fuel['minimal_load'],fuel['accumulator_flow'],fuel['engine_flow']])
        self.floats(self.fm+0x53e4+12*system,[10000.,0.,fuel['capacity']])

    def piston(self,s,velocity=(100.,0.,0.),height=0.,dt=1/48,seed=12345,torque_multiplier=1.,nitro=0.,cg=(0.,0.,0.)):
        e=self.engine;p=self.p
        def f(offset,values):self.floats(e+offset,values)
        def u(offset,value):self.u.mem_write(e+offset,struct.pack('<I',value))
        self.u.mem_write(e+0x1c,b'\x07');f(0x28,[s['omega']]);f(0x30,[1.])
        f(0x38,[s.get('mechanical',1.)]);f(0x40,[s.get('regulator',-1.),s.get('turbo',0.),1.])
        f(0x58,[1.]);u(0x6c,p['cylinders']);f(0x98,[s.get('reservoir',p['reservoir_capacity'])])
        f(0xa4,[s.get('throttle',1.)]);f(0xac,[s.get('mixture',.5)])
        self.u.mem_write(e+0xcf,bytes([s.get('automatic_mixture',False)]))
        self.u.mem_write(self.fm+0x2f5c+60*self.description['control_group'],bytes([16 if s.get('automatic_compressor',False) else 0]))
        u(0xb0,3);u(0xb4,s.get('gear',0));self.u.mem_write(e+0xbc,bytes([s.get('afterburner',False)]))
        f(0xc0,[s.get('turbo_command',1.)]);self.u.mem_write(e+0xcc,bytes([s.get('automatic_turbo',True)]))
        f(0x120,[s.get('torque',0.),s.get('friction',0.)]);f(0x138,[s.get('extra_amplitude',0.)])
        self.u.mem_write(self.seed,struct.pack('<I',seed));self.floats(self.seed+16,velocity)
        self.floats(self.fm+0x531c,[nitro])
        self.floats(self.fm+0x5320,cg)
        for i,value in enumerate([dt,height,inlet_pressure(height,velocity[0],p['ram_recovery']),0.,0.,0.,1.,torque_multiplier]):self.xmm(i,[value])
        self.run(0x1019f3b80,[e,self.fm,self.seed+16,self.seed,1,e+0x94])
        result={k:self.read(e+o,1)[0] for k,o in dict(torque=0x120,friction=0x124,regulator=0x40,throttle_ratio=0x3c,
            potential_manifold=0x128,manifold=0x12c,mechanical=0x38,extra_amplitude=0x138,consumption=0x94,effective_throttle=0x14).items()}
        result.update(gear=self.read(e+0xb4,1,'I')[0],seed=self.read(self.seed,1,'I')[0],
                      force=self.read(e+0x27c,3),moment=self.read(e+0x288,3),running=self.u.mem_read(e+0x1c,1)[0])
        if p['compressor_type']==3 or p['family']==5:result.update(turbo=self.read(e+0x44,1)[0],turbo_command=self.read(e+0xc0,1)[0])
        if p['family'] in (2,3):result['omega']=self.read(e+0x28,1)[0]
        return result
