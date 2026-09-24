"""Value-only turbine and nozzle properties from original configuration."""


def rows(m,a,width):
    ptr=m.read(a,1,'Q')[0];count=m.read(a+0x10,1,'I')[0];size=4*(2+width)
    return [[*m.read(ptr+i*size,2),m.read(ptr+i*size+8,width)] for i in range(count)]


def turbine(m,a):
    def grid(off):return m.read(m.read(a+off,1,'Q')[0],m.read(a+off+0x10,1,'I')[0])
    density=grid(0);speed=grid(0x20);ptr=m.read(a+0x50,1,'Q')[0]
    modes=[];mp=m.read(a+0x98,1,'Q')[0]
    for i in range(m.read(a+0xa8,1,'I')[0]):
        x,inv=m.read(mp+12*i,2);index=m.read(mp+12*i+8,1,'I')[0]
        modes.append([x,inv,m.read(a+0xb4+20*index,5)])
    throttle=[];tp=m.read(a+0x80,1,'Q')[0]
    for i in range(m.read(a+0x90,1,'I')[0]):
        x,inv=m.read(tp+12*i,2);index=m.read(tp+12*i+8,1,'I')[0]
        throttle.append([x,inv,[modes[index][0]]])
    return dict(density=density,speed=speed,cells=[[m.read(ptr+16*(i*len(speed)+j),4) for j in range(len(speed))] for i in range(len(density))],
        bases=m.read(a+0x38,4),tau=m.read(a+0x68,1)[0],max_omega=m.read(a+0x70,1)[0],
        throttle_scale=m.read(a+0x74,1)[0],consumption=m.read(a+0x78,1)[0],torque_zero=m.read(a+0x7c,1)[0],
        modes=modes,throttle=throttle)


def nozzles(m,instance):
    result=[]
    for i in range(m.read(instance+0x28,1,'I')[0]):
        a=instance+0x30+i*0x198
        p=dict(basis=[m.read(a+j*12) for j in range(3)],position=m.read(a+0x24),ratio=m.read(a+0x30,1)[0],
            maximum=m.read(a+0x34,1)[0],main_controls=bool(m.u.mem_read(a+0x38,1)[0]),tip=bool(m.u.mem_read(a+0x190,1)[0]),
            weights=[m.read(a+0xc8+j*20,5) for j in range(2)],ranges=[m.read(a+0xb8+j*8,2) for j in range(2)],flaps=m.read(a+0x138,4))
        for off,key,width in [(0x40,'roll_angle',3),(0x58,'pitch_angle',3),(0x70,'yaw_angle',3),(0x88,'vtol_angle',2),(0xa0,'reverse_angle',2),
            (0xf0,'roll_thrust',3),(0x108,'pitch_thrust',3),(0x120,'yaw_thrust',3),(0x148,'airbrake',2),(0x160,'vtol',2),(0x178,'reverse',2)]:p[key]=rows(m,a+off,width)
        result.append(p)
    return result
