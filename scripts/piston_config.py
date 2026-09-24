"""Value-only piston properties at the original configuration-loader boundary."""
from component_assembly import mul


def loaded_properties(m,address):
    def f(offset):return m.read(address+offset,1)[0]
    def u(offset):return m.read(address+offset,1,'I')[0]
    def b(offset):return bool(m.u.mem_read(address+offset,1)[0])
    p={k:f(o) for k,o in dict(base_hp=0xc,min_throttle=0x14,max_throttle=0x18,
        shaft_min=0x140,max_omega=0x144,afterburner_omega=0x148,omega_limit=0x154,
        inverse_omega=0x15c,compressor_reference=0x164,torque_base=0x168,engine_inertia=0x16c,
        mixer_scale=0x190,compressor_sq=0x1b8,max_ata=0x1bc,ram_recovery=0x1c0,
        boost_ata_ratio=0x1c4,compressor_zero=0x1c8,turbo_min=0x1cc,turbo_max=0x1d0,
        turbo_allowed=0x1d8,inverse_time=0x1e0,throttle_boost=0x1e8,afterburner_boost=0x1ec,
        reservoir_capacity=0x2b8).items()}
    p.update(family=m.u.mem_read(address,1)[0],exact_altitudes=b(0x1c),ata_enabled=b(0x160),
             mixer_type=u(0x18c),compressor_type=u(0x194),boost_type=u(0x1e4),
             cylinders=u(0x244),manual_compressor=b(0x251),boost_controllable=b(0x24a),carburetor=u(0x2b4),
             amplitude=m.read(address+0x180,3),consumption=m.read(address+0x2a4,4))
    p['stages']=[{k:f(o+4*i) for k,o in dict(critical=0x20,power=0x40,boost=0x60,
        flat=0x80,flat_power=0xa0,curvature=0xc0,ceiling=0xe0,ceiling_factor=0x100,
        pressure_boost=0x120).items()} for i in range(u(0x198)+1)]
    pointer=m.read(address+0x1a0,1,'Q')[0]
    p['ata']=[(m.read(pointer+i*12,1)[0],m.read(pointer+i*12+4,1)[0],m.read(pointer+i*12+8,1)) for i in range(u(0x1b0))]
    p['rpm_targets']=[m.read(address+0x1fc+8*i,2) for i in range(u(0x1f8))]
    return p
