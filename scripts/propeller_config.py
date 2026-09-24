"""Portable propeller properties decoded from the pinned native loader.

This is an offline configuration boundary, not an independent BLK loader.
The returned dictionaries contain values only; no emulated pointers or native
execution are required by the runtime propeller model.
"""
from polar_runtime import unpack_runtime


def loaded_properties(machine, address, instance, reduction):
    m=machine
    def f(offset):return m.read(address+offset,1)[0]
    def u(offset):return m.read(address+offset,1,'I')[0]
    def b(offset):return bool(m.u.mem_read(address+offset,1)[0])
    polar=unpack_runtime(bytes(m.u.mem_read(address,0x1e0)),address)
    p=dict(polar=polar,radius=f(0x1e0),blades=u(0x1e4),
           twist=m.read(address+0x1e8,4),mean_twist=f(0x1f8),
           width=m.read(address+0x1fc,4),projected_width=f(0x20c),
           thrust_deflection=f(0x21c),max_deflection=f(0x220),momentum_scale=f(0x224),
           coaxial=b(0x228),pitch_damping=m.read(address+0x22c,4),
           yaw_damping=m.read(address+0x23c,4),damping_speed=m.read(address+0x24c,4),
           iterative=b(0x25c),shake=m.read(address+0x2b0,4),
           inertia_coefficient=f(0x2c0),mass=f(0x2c4),diameter=f(0x2c8),
           neutral_radius=f(0x2cc),inertia=f(0x2d0),inverse_inertia=f(0x2d4),
           pitch_min=f(0x2d8),pitch_max=f(0x2dc),feather_pitch=f(0x338),
           aoa0=f(0x33c),governor_speed=f(0x340),governor=u(0x344),
           governor_fast=b(0x348),min_omega=f(0x34c),max_omega=f(0x350),
           inv_max_omega=f(0x354),boost_omega=f(0x358),auto_allowed=b(0x35c),
           torque_gyro_always=b(0x35d),force_always=b(0x35e),critical_ias=f(0x360),
           manual_allowed=b(0x368),pitch_command_report=b(0x369),
           feather_allowed=b(0x36a),cyclic=b(0x36b),differential_pitch=b(0x36c),
           basis=m.read(instance+4,9),position=m.read(instance+0x28,3),
           direction=m.read(instance+0x34,1,'I')[0],reduction=reduction)
    count=u(0x270);pointer=m.read(address+0x260,1,'Q')[0]
    p['airflow']=[m.read(pointer+i*20,5) for i in range(count)]
    count=u(0x2f0);pointer=m.read(address+0x2e0,1,'Q')[0]
    p['active_pitch_2d']=bool(count) and all(m.read(pointer+i*32+24,1,'I')[0] for i in range(count))
    p['pitch_1d_count']=u(0x330)
    return p
