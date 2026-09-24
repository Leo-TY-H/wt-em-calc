"""Original complete propulsion configuration loader, used as a research oracle.

101a11ef0 resolves engine/propeller types, instances and transmissions itself.
Only source DataBlock access, allocation/string services and host math are
adapted. This is not an independent property implementation or a flight step.
"""
import math
import re
from unicorn import UC_HOOK_CODE
from mass_parts_native import MassPartsNative


class PropConfigNative(MassPartsNative):
    def __init__(self):
        super().__init__()
        self.u.hook_add(UC_HOOK_CODE, self.sincos, begin=0x106e613e1, end=0x106e613e1)

    def sincos(self, u, a, n, data):
        self.services[hex(a)] += 1
        angle = self.read_xmm(0)[0]
        self.xmm(0, [math.sin(angle), math.cos(angle)])
        self.ret()

    def load_config(self, fm):
        address = self.alloc(0x26000)
        source = {k:v for k,v in fm.items()
                  if re.fullmatch(r'(Engine(Type)?|Propeller(Type)?|Transmission)\d+', k)}
        block = self.block(source)
        self.run(0x101a11ef0, [address, block, 1, self.allocator])
        return address

    def describe(self, a):
        def uint(p): return self.read(p,1,'I')[0]
        def flt(p): return self.read(p,1)[0]
        def flag(p): return bool(self.u.mem_read(p,1)[0])
        engines=[]
        for i in range(uint(a+0x7708)):
            p=a+0x7710+i*0x19b0; type_id=uint(p)
            t=a+8+type_id*0x770
            engines.append(dict(index=i,type_id=type_id,family=self.u.mem_read(t,1)[0],
                                position=self.read(p+4),axis=self.read(p+0x10),
                                carburetor=uint(t+0x2b4),mixture=uint(t+0x18c),compressor=uint(t+0x194),
                                compressor_stages=uint(t+0x198)+1,exact_altitudes=flag(t+0x1c),
                                boost_type=uint(t+0x1e4),torque_base=flt(t+0x168)))
        props=[]
        for i in range(uint(a+0x24918)):
            p=a+0x2491c+i*0x38;type_id=uint(p)
            t=a+0x21218+type_id*0x370
            props.append(dict(index=i,type_id=type_id,basis=self.read(p+4,9),
                              position=self.read(p+0x28),rotation=uint(p+0x34),
                              governor=uint(t+0x344),fast=flag(t+0x348),
                              airflow_solver=flag(t+0x25c),coaxial=flag(t+0x228),
                              manual=flag(t+0x368),automatic=flag(t+0x35c),
                              feathering=flag(t+0x36a),cyclic=flag(t+0x36b),
                              differential_pitch=flag(t+0x36c),
                              torque_gyro_always=flag(t+0x35d),force_always=flag(t+0x35e),
                              pitch_min=flt(t+0x2d8),pitch_max=flt(t+0x2dc),
                              pitch_2d_rows=uint(t+0x2f0),pitch_1d_rows=uint(t+0x330),
                              pitch_feather=flt(t+0x338),governor_speed=flt(t+0x340),
                              min_omega=flt(t+0x34c),max_omega=flt(t+0x350),
                              boost_omega=flt(t+0x358),critical_ias=flt(t+0x360),
                              diameter=flt(t+0x2c8),inertia=flt(t+0x2d0)))
        transmissions=[]
        for i in range(uint(a+0x24ca0)):
            p=a+0x24ca8+i*0xc8
            transmissions.append(dict(index=i,
                engines=[dict(index=uint(p+4+j*12),ratio=flt(p+8+j*12),inverse_ratio=flt(p+12+j*12)) for j in range(uint(p))],
                propellers=[dict(index=uint(p+0x3c+j*28),ratio=flt(p+0x40+j*28),inverse_ratio=flt(p+0x44+j*28),
                                 pitch_source=uint(p+0x48+j*28)) for j in range(uint(p+0x38))],
                auto_inertia=flag(p+0xb0),acceleration=flt(p+0xb4),correct_link=flag(p+0xb8)))
        return dict(engine_type_count=uint(a),propeller_type_count=uint(a+0x21210),
                    engines=engines,propellers=props,transmissions=transmissions)


if __name__ == '__main__':
    import json
    from pathlib import Path
    for name in ['yak-3', 'bf-109f-4']:
        m = PropConfigNative()
        a = m.load_config(json.loads(Path('references/fm-2.59.0.13/'+name+'.blkx').read_text()))
        print(name, json.dumps(m.describe(a),indent=2))
