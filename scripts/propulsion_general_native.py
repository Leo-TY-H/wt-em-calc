"""Original complete propulsion owner on the actual loaded installation graph."""
import struct
from unicorn import UC_HOOK_CODE
from prop_config_native import PropConfigNative
from propulsion_config import decode
from engine_supply import fuel_properties


class PropulsionGeneralNative(PropConfigNative):
    def configure(self,fm):
        self.config_address=self.load_config(fm);self.config=decode(self,self.config_address)
        self.owner=self.alloc(0x28000);self.fm=self.alloc(0xb000);self.seed=self.alloc(0x400)
        self.engine_addresses=[self.alloc(0x600) for _ in self.config['engines']]
        self.prop_addresses=[self.alloc(0x100) for _ in self.config['propellers']]
        self.trans_addresses=[self.alloc(0x100) for _ in self.config['transmissions']]
        for address in [0x1019f7b50,0x1019f77a0,0x1019f7fc0]:
            self.u.hook_add(UC_HOOK_CODE,lambda u,a,n,d:self.ret(),begin=address,end=address)
        self.u.mem_write(0x107d6fbea,b'\x01\x01\x01\x00\x01')
        self.u.mem_write(0x107d6fc10,b'\x00\x01')
        for off,n in [(0x25a78,len(self.engine_addresses)),(0x25f78,len(self.engine_addresses)),
                      (0x26000,len(self.prop_addresses)),(0x26088,len(self.trans_addresses)),(0x25b70,1)]:self.uint(self.owner+off,n)
        for off,addresses in [(0x25f80,self.engine_addresses),(0x26008,self.prop_addresses),(0x26090,self.trans_addresses)]:
            for i,address in enumerate(addresses):self.qword(self.owner+off+8*i,address)
        self.floats(self.owner+0x25b68,[1.]);self.floats(self.owner+0x25b74,[0.,1.,0.,1000.]*4)
        self.qword(self.fm+0x55a0,self.owner);self.u.mem_write(self.fm+0x8470,b'\x01')
        self.doubles(self.fm+0x16b8,[1.])
        for e in self.config['engines']:
            linked=next((t['index'] for t in self.config['transmissions'] if any(l['index']==e['index'] for l in t['engines'])),0xffffffff)
            self.uint(self.owner+0x25934+8*e['index'],linked)
            system=e['fuel_system'];fuel=fuel_properties(fm['Mass'],system)
            self.floats(self.fm+0x5158+28*system,[fuel['minimal_load'],fuel['accumulator_flow'],fuel['engine_flow']])
            self.floats(self.fm+0x53e4+12*system,[10000.,0.,fuel['capacity']])
        for a in self.prop_addresses:self.run(0x101a08760,[a])

    def uint(self,a,v):self.u.mem_write(a,struct.pack('<I',v))

    def step(self,state,velocity=(100.,0.,0.),height=0.,body_omega=(0.,0.,0.),cg=(0.,0.,0.),dt=1/48,seed=12345,nitro=0.,torque_gyro=True,prepare_only=False):
        ca=self.config_address
        for e,s,a in zip(self.config['engines'],state['engines'],self.engine_addresses):
            self.u.mem_write(a,bytes(0x600));p=e['properties']
            self.qword(a,ca+8+e['type_id']*0x770);self.qword(a+8,ca+0x7710+e['index']*0x19b0)
            self.qword(a+0x320,ca+8+e['type_id']*0x770+0x578);self.qword(a+0x328,ca+0x7710+e['index']*0x19b0+0x28)
            self.u.mem_write(a+0x10,b'\x01\x01\x01');self.u.mem_write(a+0x1c,bytes([s.get('running',7)]))
            self.floats(a+0x14,[s.get('effective_throttle',0.)])
            self.floats(a+0x20,[s.get('elapsed',0.)]);self.floats(a+0x28,[s.get('omega',270.),0.,1.])
            self.floats(a+0x38,[s.get('mechanical',1.)]);self.floats(a+0x40,[s.get('regulator',-1.),s.get('turbo',0.),1.])
            self.floats(a+0x58,[1.]);self.uint(a+0x6c,p['cylinders']);self.floats(a+0x98,[s.get('reservoir',p['reservoir_capacity'])])
            self.floats(a+0xa4,[s.get('throttle',1.)]);self.floats(a+0xac,[s.get('mixture',.5)])
            self.u.mem_write(a+0xcf,bytes([s.get('automatic_mixture',False)]))
            self.u.mem_write(self.fm+0x2f5c+60*e['control_group'],bytes([16 if s.get('automatic_compressor',False) else 0]))
            self.uint(a+0xb0,3);self.uint(a+0xb4,s.get('gear',0));self.u.mem_write(a+0xbc,bytes([s.get('afterburner',False)]))
            self.floats(a+0xc0,[s.get('turbo_command',1.)]);self.u.mem_write(a+0xcc,bytes([s.get('automatic_turbo',True)]))
            self.floats(a+0x120,[s.get('torque',0.),s.get('friction',0.)])
        for p,s,a in zip(self.config['propellers'],state['propellers'],self.prop_addresses):
            self.u.mem_write(a+0x10,bytes(0xf0));self.qword(a,ca+0x2491c+p['index']*0x38)
            self.qword(a+8,ca+0x21218+p['type_id']*0x370);pitch=s.get('pitch',p['properties']['pitch_min'])
            self.floats(a+0x10,[pitch]);self.floats(a+0x24,[s.get('governor_pitch',pitch),*s.get('flow',[0.,0.,0.]),1000.])
            self.floats(a+0x48,[s.get('command',1.)]);self.u.mem_write(a+0x4d,bytes([s.get('auto',False)]))
        for t,s,a in zip(self.config['transmissions'],state['transmissions'],self.trans_addresses):
            self.u.mem_write(a,bytes(0x100));self.qword(a,ca+0x24ca8+t['index']*0xc8)
            self.floats(a+8,[s['omega'],s.get('previous_omega',s['omega']),1.,0.,1.])
        self.uint(self.seed,seed);self.doubles(self.fm+0x1560,[height]);self.doubles(self.fm+0x1618,velocity)
        self.doubles(self.fm+0x15e0,body_omega);self.floats(self.fm+0x5320,cg)
        self.u.mem_write(self.fm+0x3658,bytes([6 if torque_gyro else 4]))
        self.floats(self.fm+0x8494,[height]);self.floats(self.fm+0x531c,[nitro])
        if prepare_only:return
        self.xmm(0,[dt]);self.run(0x101a155c0,[self.owner,self.fm,self.seed,0,self.seed+0x200])
        es=[];ps=[];ts=[]
        for e,a in zip(self.config['engines'],self.engine_addresses):
            result={k:self.read(a+o,1)[0] for k,o in dict(omega=0x28,elapsed=0x20,torque=0x120,friction=0x124,regulator=0x40,
                throttle_ratio=0x3c,potential_manifold=0x128,manifold=0x12c,mechanical=0x38,extra_amplitude=0x138,
                consumption=0x94,effective_throttle=0x14).items()}
            result.update(gear=self.read(a+0xb4,1,'I')[0],force=self.read(a+0x27c),moment=self.read(a+0x288),running=self.u.mem_read(a+0x1c,1)[0])
            if e['properties']['compressor_type']==3:result.update(turbo=self.read(a+0x44,1)[0],turbo_command=self.read(a+0xc0,1)[0])
            es.append(result)
        for a in self.prop_addresses:ps.append(dict(pitch=self.read(a+0x10,1)[0],governor_pitch=self.read(a+0x24,1)[0],flow=self.read(a+0x28),outputs=self.read(a+0x60,40)))
        for a in self.trans_addresses:ts.append(dict(omega=self.read(a+8,1)[0],previous_omega=self.read(a+12,1)[0],outputs=self.read(a+0x70,13)))
        return dict(engines=es,propellers=ps,transmissions=ts,seed=self.read(self.seed,1,'I')[0],
            aggregate_force=self.read(self.owner+0x25a48,3,'d'),aggregate_moment=self.read(self.owner+0x25a60,3,'d'),
            engine_angular_momentum=self.read(self.owner+0x25b40,3,'d'),engine_wash=self.read(self.owner+0x25b58,2),
            per_engine_force=[self.read(self.owner+0x25a7c+12*i) for i in range(len(es))])
