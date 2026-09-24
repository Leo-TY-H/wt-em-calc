"""Original Windows Instructor update in an explicit prepared free-air fixture.

Not a spawned game actor or complete owner scheduler. The inherited source
fixture supplies intact FM properties and held propulsion inputs. Actor-owned
collections are empty, optional external components absent, and the two
control-history/publication predicates true. Collision handling is disabled
through its original native gate for this contact-free research fixture.
No controller, actuator, body integration or snapshot arithmetic is replaced.
"""
import json
from pathlib import Path
import struct

from unicorn import UC_HOOK_CODE, UC_HOOK_MEM_WRITE
from unicorn.x86_const import *
from windows_instructor_native import WindowsInstructorNative, TLS
from instructor_native import BASE, OBJ, INPUT, OUTPUT, STACK, STOP, ARENA
from instructor_mouseaim_settings import configured_fields
from component_assembly import mul, add, sub
from body_dynamics import G
from instructor_pitch_predictor import unpack_inputs
from polar_model import FIELDS as POLAR_FIELDS

PARAMETERS=0x147985ce8
MOUSE=0x147985a20
INSTRUCTOR=0x1479861b8
ACTOR=ARENA+0x80000
ALLOC_HEAP=0x240000000
ALLOC_SIZE=0x1000000


class WindowsInstructorController(WindowsInstructorNative):
    def __init__(self,binary):
        super().__init__(binary)
        self.trace=dict(predictors=[],trim_updates=[],restores=[],actor_calls=[])
        self._predictor=None;self._restore=None;self._actuator_dt=0.
        self.u.mem_map(ALLOC_HEAP,ALLOC_SIZE);self.allocation_cursor=0
        self.qword(0x147e5b490,TLS+0x4000)
        self.qword(TLS+0x5018,TLS+0x6100)
        self.u.mem_write(TLS+0x6100,b'\xc3')
        self.u.hook_add(UC_HOOK_CODE,self.allocate,begin=TLS+0x6100,end=TLS+0x6100)
        for address in (0x1430b06c0,0x1430b6204,0x1430aae60,0x142fd4180,0x142fd4663,0x143094a17,0x143094c82):
            self.u.hook_add(UC_HOOK_CODE,self.observe,begin=address,end=address)
        self.u.hook_add(UC_HOOK_MEM_WRITE,self.observe_trim,begin=BASE+0xa294,end=BASE+0xa297)
        # These are explicit interface values, not recovered game constructors.
        # b8/c0 return {data:null, count:0}; b0/c8/d0 return absent components.
        # 10/18 select control-history consumption and snapshot flag writing.
        definitions={0xb8:(0x8000,'48c70200000000c74208000000004889d0c3'),
                     0xc0:(0x8020,'48c70200000000c74208000000004889d0c3'),
                     0xb0:(0x8040,'31c0c3'),0xc8:(0x8060,'31c0c3'),0xd0:(0x8080,'31c0c3'),
                     0x10:(0x80a0,'b801000000c3'),0x18:(0x80c0,'b801000000c3')}
        self.actor_services={TLS+offset:slot for slot,(offset,_) in definitions.items()}
        for slot,(offset,code) in definitions.items():
            self.qword(TLS+0x7000+slot,TLS+offset)
            self.u.mem_write(TLS+offset,bytes.fromhex(code))
            self.u.hook_add(UC_HOOK_CODE,self.observe,begin=TLS+offset,end=TLS+offset)

    def allocate(self,u,address,size,data):
        # Explicit scratch allocator for original temporary MouseAim arrays.
        # No allocation result carries computed controller or physics values.
        requested=u.reg_read(UC_X86_REG_RDX);amount=(max(1,requested)+15)&~15
        if self.allocation_cursor+amount>ALLOC_SIZE:raise RuntimeError('Fixture allocation limit exceeded')
        result=ALLOC_HEAP+self.allocation_cursor;self.allocation_cursor+=amount
        u.mem_write(result,bytes(amount));u.reg_write(UC_X86_REG_RAX,result)
        self.trace.setdefault('allocations',[]).append(requested)
        self.return_call()

    def sync_windows_globals(self):
        """Copy prepared parameter fields; keep atmospheric globals separate.

        Common parameter loads retain the reviewed source-record offsets.
        Named Instructor loader143099fa0 and MouseAim loader142f98c60 locate
        the independent Windows global groups. Neither is an actor initializer.
        """
        u=self.u
        u.mem_write(PARAMETERS,bytes(u.mem_read(0x107d6fba0,0x100)))
        # Only Instructor fields, not adjacent Windows atmosphere variables.
        u.mem_write(INSTRUCTOR+0x14,bytes(u.mem_read(0x107e49914,0x31)))
        self.floats(0x1479861b4,[G]);self.floats(0x1479861c0,[1.225,18300.])
        gameplay=json.loads((Path(__file__).resolve().parents[1]/'references/body-gameplay.blkx').read_text())
        for off,data in configured_fields(gameplay['mouseAim']):u.mem_write(MOUSE+off,data)
        u.mem_write(MOUSE+0x17c,b'\0') # same constant-gain research profile
        self.qword(0x147e61fc0,1);u.mem_write(0x147e5f370,b'\0') # profiler registered, disabled
        u.mem_write(PARAMETERS+0x73,b'\0') # contact-free fixture; native collision gate

    def windows_provider(self,u,address,size,data):
        if getattr(self,'_predictor',None) is None and address in (0x142fdae80,0x143140ce0):
            # A detailed-body call must not read a dead reduced-predictor stack.
            previous=self.windows_input
            self.windows_input=TLS+0x9000
            self.floats(self.windows_input+0x74,[self.model['sweep']])
            self.trace.setdefault('body_property_calls',[]).append(hex(address))
            try:super().windows_provider(u,address,size,data)
            finally:self.windows_input=previous
        else:super().windows_provider(u,address,size,data)

    def setup_controller(self,model,mass,*,moving=False,**conditions):
        super().setup(model,mass,mode_lane=True,**conditions)
        self._predictor=None;self._restore=None;self._actuator_dt=0.
        self.allocation_cursor=0
        # Native flap mapper142fc1a40 uses separate value/index arrays. The
        # inherited Mac fixture replaces that provider and leaves them empty.
        # Supply prepared authored rows so Windows executes its own lookup.
        values,knots=TLS+0xa000,TLS+0xc000
        if len(model['flaps'])>512:raise ValueError('Flap table exceeds fixture storage')
        props=BASE+0x6ef8
        self.qword(props+0xce0,values);self.qword(props+0xcf8,knots)
        self.u.mem_write(props+0xcf0,struct.pack('<II',len(model['flaps']),len(model['flaps'])))
        self.u.mem_write(props+0xd08,struct.pack('<II',len(model['flaps']),len(model['flaps'])))
        for index,(knot,inverse,row) in enumerate(model['flaps']):
            self.floats(values+index*16,row)
            self.u.mem_write(knots+index*12,struct.pack('<ffI',knot,inverse,index))
        self.sync_windows_globals()
        self.qword(BASE+8,(BASE-ACTOR)&((1<<64)-1))
        self.qword(ACTOR,TLS+0x7000);self.qword(ACTOR+0x2ef0,BASE)
        # Original FM vtable. Inertia getter142fd2a50 occupies slot30.
        self.qword(BASE,0x146c83ed8)
        self.floats(BASE+0x4ea8,[self.dt if moving else 0.])
        self.moving=moving

    def invoke_windows(self,address,args=(),floats=None,*,count=20000000):
        u=self.u;sp=STACK-8;self.qword(sp,STOP);u.reg_write(UC_X86_REG_RSP,sp)
        for reg,value in zip((UC_X86_REG_RCX,UC_X86_REG_RDX,UC_X86_REG_R8,UC_X86_REG_R9),args):u.reg_write(reg,value)
        for index,value in (floats or {}).items():self.xmm(index,[value])
        try:u.emu_start(address,STOP,count=count)
        except Exception as error:
            raise RuntimeError('Windows update stopped at '+hex(u.reg_read(UC_X86_REG_RIP))) from error
        if u.reg_read(UC_X86_REG_RIP)!=STOP:raise RuntimeError('Windows update instruction budget exceeded')

    def step_windows(self):
        self.trace=dict(predictors=[],trim_updates=[],restores=[],actor_calls=[])
        self.windows_calls.clear()
        self.invoke_windows(0x1430933b0,(OBJ,INPUT,0,OUTPUT),{2:self.dt})
        return self.trace

    def trim_state(self):
        return dict(requested=self.read(BASE+0x87f4,3),actual=self.read(BASE+0xa290,3),cache=self.read(BASE+0x3a24,3))

    def observe(self,u,address,size,data):
        if address==0x143094a17:
            self.trace['cos_dihedral']=self.read_xmm(0)[0];return
        if address==0x143094c82:
            sp=u.reg_read(UC_X86_REG_RSP)
            self.trace['protection']=dict(polar=dict(zip(POLAR_FIELDS,self.read(sp+0x110,24))),
                angle_limits=[self.read_xmm(9)[0],self.read(sp+0x90,1)[0]])
            return
        if address in self.actor_services:
            self.trace['actor_calls'].append(hex(self.actor_services[address]));return
        if address==0x1430b06c0:
            self.windows_input=u.reg_read(UC_X86_REG_RDX)
            self._predictor=dict(mode=self.read(self.windows_input,1,'I')[0],
                output=u.reg_read(UC_X86_REG_R8),history=u.reg_read(UC_X86_REG_R9),
                inputs=unpack_inputs(bytes(u.mem_read(self.windows_input,0xa0))))
        elif address==0x1430b6204:
            row=self._predictor
            if row is not None:
                self.trace['predictors'].append(dict(mode=row['mode'],success=bool(u.reg_read(UC_X86_REG_RAX)&255),
                    inputs=row['inputs'],output=self.read(row['output'],13),history=self.read(row['history'],2)+[bool(u.mem_read(row['history']+8,1)[0])]))
                self._predictor=None
        elif address==0x1430aae60:self._actuator_dt=self.read_xmm(1)[0]
        elif address==0x142fd4180:self._restore=self.trim_state()
        elif address==0x142fd4663:
            self.trace['restores'].append(dict(before=self._restore,after=self.trim_state()))
            self._restore=None

    def observe_trim(self,u,access,address,size,value,data):
        # Only the actuator's pitch-trim store, not clears from availability.
        if u.reg_read(UC_X86_REG_RIP)!=0x1430ab0e5:return
        old=self.read(BASE+0xa294,1)[0];request=self.read(BASE+0x87f8,1)[0]
        step=mul(mul(self.read(BASE+0x16a8,1)[0],self._actuator_dt),self.read(BASE+0x8060,1)[0])
        expected=min(add(old,step),max(sub(old,step),request)) if u.mem_read(BASE+0x8521,1)[0] else 0.
        actual=struct.unpack('<f',struct.pack('<I',value&0xffffffff))[0]
        self.trace['trim_updates'].append(dict(dt=self._actuator_dt,before=old,request=request,after=actual,
            expected=expected,equal=actual==expected))
