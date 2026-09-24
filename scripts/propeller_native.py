"""Prepared intact fighter propeller adapter for original 101a08de0.

This executes the full native inflow/governor/load routine. It is an oracle,
not an independent reconstruction. Static loader and context inputs remain
explicit adapters; blade aerodynamics execute without substitution.
"""
import json
import math
import struct
from pathlib import Path
from unicorn import UC_HOOK_CODE
from unicorn.x86_const import *
from verify_aircraft_native import AircraftNative, BASE, FRAME, END
from component_assembly import f32, add, sub, mul
from piston_model import div
from polar_runtime import pack_runtime
from propeller_model import properties

STATE=BASE+0x1000
PROPS=BASE+0x2000
INSTANCE=BASE+0x3000
INPUT=BASE+0x4000
GROUND=BASE+0x5000
TABLE=BASE+0x6000




class PropellerNative(AircraftNative):
    def __init__(self):
        super().__init__()
        self.trace=[];self.blade_inputs=[]
        self.u.hook_add(UC_HOOK_CODE,self.blade_capture,begin=0x101a07790,end=0x101a07790)

    def blade_capture(self,u,a,n,data):
        self.blade_inputs.append([self.read_xmm(i)[0] for i in range(8)]+self.read(u.reg_read(UC_X86_REG_RSP)+8,1))

    def configure(self,p):
        u=self.u;u.mem_write(BASE,bytes(0x10000))
        self.qword(STATE,INSTANCE);self.qword(STATE+8,PROPS)
        u.mem_write(PROPS,pack_runtime(p['polar'],PROPS))
        self.floats(PROPS+0x1e0,[p['radius']]);u.mem_write(PROPS+0x1e4,struct.pack('<I',p['blades']))
        self.floats(PROPS+0x1e8,p['twist']+[p['mean_twist']]+p['width']+[p['projected_width']])
        self.floats(PROPS+0x220,[.95,1.])
        self.floats(PROPS+0x24c,[0.,1.,0.,1.])
        self.qword(PROPS+0x260,TABLE);u.mem_write(PROPS+0x270,struct.pack('<I',2))
        self.floats(TABLE,[-1000.,.0005,0.,0.,1.,1000.,0.,0.,0.,1.])
        self.floats(PROPS+0x278,[0.,1.,f32(2147440000.*f32(.2777778)),1.])
        self.floats(PROPS+0x28c,[-2147440000.,2147440000.,.1,div(f32(2.3025851),4.5),18.,.005])
        self.floats(PROPS+0x2c0,[p['inertia_coefficient'],p['mass'],p['diameter'],mul(p['diameter'],.375),p['inertia'],p['inverse_inertia'],p['pitch_min'],p['pitch_max']])
        self.floats(PROPS+0x338,[p['feather_pitch'],p['aoa0'],p['governor_speed']])
        u.mem_write(PROPS+0x344,struct.pack('<IB',p['governor'],p['governor_fast']))
        self.floats(PROPS+0x34c,[p['min_omega'],p['max_omega'],p['inv_max_omega'],p['boost_omega']])
        u.mem_write(PROPS+0x35c,bytes([p['auto_allowed'],0,1]))
        self.floats(PROPS+0x360,[p['critical_ias'],25.]);u.mem_write(PROPS+0x368,bytes([1,p['pitch_command_report']]))
        self.floats(INSTANCE+4,[1.,0.,0.,0.,1.,0.,0.,0.,1.]+p['position'])
        u.mem_write(INSTANCE+0x34,struct.pack('<I',p['direction']))
        self.qword(0x107615358,BASE+0x6e000)
        self.p=p

    def step(self,state,velocity=(100.,0.,0.),body_omega=(0.,0.,0.),cg=(0.,0.,0.),
             omega=180.,previous_omega=None,target_omega=270.,command=1.,auto=False,
             density=1.225,sound_speed=340.,dt=1/60,afterburner=False,torque_gyro=True,
             trace=False,prepare_only=False):
        u=self.u;self.calls=[];self.blade_inputs=[]
        u.mem_write(STATE+0x10,bytes(0xf0))
        if isinstance(state,bytes):u.mem_write(STATE+0x10,state)
        else:
            self.floats(STATE+0x10,[state.get('pitch',self.p['pitch_min'])])
            self.floats(STATE+0x24,[state.get('governor_pitch',state.get('pitch',self.p['pitch_min'])),*state.get('flow',[0.,0.,0.])])
        self.floats(STATE+0x48,[command]);u.mem_write(STATE+0x4d,bytes([auto]))
        self.floats(INPUT,[*velocity,*body_omega,*cg])
        u.mem_write(INPUT+0x24,bytes([torque_gyro,1,0,0,0]))
        self.floats(INPUT+0x2c,[density,sound_speed,1000.])
        self.qword(INPUT+0x38,GROUND);self.qword(INPUT+0x40,4)
        self.floats(GROUND,[0.,1.,0.,1000.]*4)
        self.floats(INPUT+0x48,[target_omega]);u.mem_write(INPUT+0x4c,b'\x01')
        self.floats(INPUT+0x50,[omega,omega if previous_omega is None else previous_omega,self.p['reduction'],div(1.,self.p['reduction'])])
        # Engine lifecycle 7 is mapped to governor lifecycle 2 by 10720821c.
        u.mem_write(INPUT+0x60,bytes([afterburner]));u.mem_write(INPUT+0x64,struct.pack('<I',2))
        if prepare_only:return
        self.qword(FRAME,END);u.reg_write(UC_X86_REG_RSP,FRAME)
        u.reg_write(UC_X86_REG_RDI,STATE);u.reg_write(UC_X86_REG_RSI,INPUT);self.xmm(0,[dt])
        self.trace=[]
        hook=u.hook_add(UC_HOOK_CODE,lambda u,a,n,d:self.trace.append(a),begin=0x101a08de0,end=0x101a0ed60) if trace else None
        try:u.emu_start(0x101a08de0,END,count=100000)
        finally:
            if hook:u.hook_del(hook)
        if u.reg_read(UC_X86_REG_RIP)!=END:raise RuntimeError('Propeller did not return')
        return dict(state=bytes(u.mem_read(STATE+0x10,0xf0)),pitch=self.read(STATE+0x10,1)[0],governor_pitch=self.read(STATE+0x24,1)[0],
                    flow=self.read(STATE+0x28),outputs=self.read(STATE+0x60,40),blade_inputs=self.blade_inputs)


if __name__=='__main__':
    m=PropellerNative()
    for name in ['yak-3','bf-109f-4']:
        p=properties(json.loads(Path('references/fm-2.59.0.13/'+name+'.blkx').read_text()))
        m.configure(p);r=m.step({},trace=True)
        print(name,{k:v for k,v in r.items() if k!='state'})
        lines=Path('analysis/disassembly/0000000101a08de0.asm').read_text().splitlines()
        seen=set(m.trace)
        Path('analysis/'+name+'-propeller-executed.asm').write_text('\n'.join(l for l in lines if int(l[:16],16) in seen)+'\n')
