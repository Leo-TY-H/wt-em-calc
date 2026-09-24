"""Offline Instructor oracle for the pinned local executable.

Research harness, not the production envelope. Input structs are prepared, not
captured from a running game. AircraftNative's explicitly listed property and
libm adapters remain in force. Every further substituted call is recorded.
"""
import json, math, struct
from pathlib import Path
from unicorn import UC_HOOK_CODE
from unicorn.x86_const import *
from verify_aircraft_native import AircraftNative, BASE, OWNER, CONFIG
from component_assembly import f32,mul
from body_dynamics import G
from primary_controls import selected_properties
from polar_runtime import pack_runtime, flap_polar
from wing_sweep import prepare as prepare_wings, select as select_wing
from verify_wing_sweep import SweepMachine
from instructor_protection import gain_reference_speed

ARENA=0x220000000
OBJ=ARENA; INPUT=ARENA+0x1000; OUTPUT=ARENA+0x2000
STACK=ARENA+0x20000; STOP=ARENA+0x30000
HEAP=ARENA+0x40000


class InstructorNative(AircraftNative):
    def hook(self,u,a,size,data):
        if a==0x1019e20d0 and ARENA <= u.reg_read(UC_X86_REG_RDI) < ARENA+0x100000:
            # Reduced predictor's stack wing header has already selected sweep.
            # AircraftNative's detailed-FM adapter otherwise uses model.sweep.
            out=u.reg_read(UC_X86_REG_RSI)
            wing=select_wing(self.wings,self.read(self.predictor_input+0x74,1)[0])
            u.mem_write(out,pack_runtime(flap_polar(wing['polars'],self.read_xmm(0)[0]),out))
            self.calls.append(hex(a));self.return_call();return
        super().hook(u,a,size,data)

    def __init__(self,binary_path=None):
        super().__init__(binary_path=binary_path)
        self.u.mem_map(ARENA,0x100000)
        self.extra_calls=[]; self.capture_stop=None; self.captured={}
        for va in [0x106e6133f,0x106e61549,0x106e614bf,0x106e61465,0x106e6199f,0x106e618d3,0x106e613e1,0x106e61351,0x106e618c1,
                   0x10001ef90,0x10001ee10,0x10001f030,0x101a39980,ARENA+0x31000]:
            self.u.hook_add(UC_HOOK_CODE,self.extra_hook,begin=va,end=va)
        self.u.hook_add(UC_HOOK_CODE,self.capture_controller,begin=0x101a939a4,end=0x101a939a4)
        self.u.hook_add(UC_HOOK_CODE,self.capture_controller,begin=0x101a60b50,end=0x101a60b50)
        self.u.hook_add(UC_HOOK_CODE,self.capture_controller,begin=0x101a94b54,end=0x101a94b54)
        self.u.hook_add(UC_HOOK_CODE,self.capture_controller,begin=0x101a93b3d,end=0x101a93b3d)
        self.u.hook_add(UC_HOOK_CODE,self.capture_controller,begin=0x101a94d8b,end=0x101a94d8b)
        self.u.hook_add(UC_HOOK_CODE,self.capture_controller,begin=0x101a9517e,end=0x101a9517e)
        self.u.hook_add(UC_HOOK_CODE,self.capture_predictor_entry,begin=0x101a5cac0,end=0x101a5cac0)

    def capture_predictor_entry(self,u,a,size,data):
        self.predictor_input=u.reg_read(UC_X86_REG_RSI)

    def return_call(self):
        u=self.u;sp=u.reg_read(UC_X86_REG_RSP)
        ret=struct.unpack('<Q',u.mem_read(sp,8))[0]
        u.reg_write(UC_X86_REG_RSP,sp+8);u.reg_write(UC_X86_REG_RIP,ret)

    def extra_hook(self,u,a,size,data):
        self.extra_calls.append(hex(a))
        if a==0x106e61549:self.xmm(0,[math.cos(self.read_xmm(0)[0])])
        elif a==0x106e614bf:self.xmm(0,[math.atan(self.read_xmm(0)[0])]) # atanf, verified import
        elif a==0x106e61465:self.xmm(0,[math.acos(self.read_xmm(0)[0])]) # acosf, verified import
        elif a==0x106e613e1:
            angle=self.read_xmm(0)[0];self.xmm(0,[math.sin(angle),math.cos(angle)]) # Darwin __sincosf_stret
        elif a==0x106e61351:u.mem_write(u.reg_read(UC_X86_REG_RDI),bytes(u.reg_read(UC_X86_REG_RSI))) # __bzero
        elif a==0x106e618c1:
            dest=u.reg_read(UC_X86_REG_RDI);u.mem_write(dest,bytes(u.mem_read(u.reg_read(UC_X86_REG_RSI),u.reg_read(UC_X86_REG_RDX))))
            u.reg_write(UC_X86_REG_RAX,dest) # memcpy
        elif a==0x106e6199f:self.xmm(0,[math.pow(self.read_xmm(0)[0],self.read_xmm(1)[0])])
        elif a==0x106e618d3:
            n=u.reg_read(UC_X86_REG_RDX);pattern=bytes(u.mem_read(u.reg_read(UC_X86_REG_RSI),16))
            u.mem_write(u.reg_read(UC_X86_REG_RDI),(pattern*((n+15)//16))[:n])
        elif a==0x106e6133f:pass # Darwin stack-page probe; stack is already mapped.
        elif a==0x10001ee10:
            # Temporary allocator getter. Prepared property adapter below does
            # not allocate; original constructors still initialize their structs.
            u.reg_write(UC_X86_REG_RAX,HEAP)
        elif a in [0x10001ef90,0x10001f030,ARENA+0x31000]:pass
        elif a==0x101a39980:
            # The sweep selector is independently validated. Keep the prepared
            # tables out of the predictor's stack locals and copy only its header.
            p=select_wing(self.wings,self.read_xmm(0)[0])
            SweepMachine.pack(self,p,HEAP+0x10000)
            out=u.reg_read(UC_X86_REG_RSI)
            u.mem_write(out,bytes(u.mem_read(HEAP+0x10000,0xd8)))
            for off in [0x98,0xb0,0xc8]:self.qword(out+off,HEAP+0x3000)
        else:raise RuntimeError('Unknown Instructor adapter '+hex(a))
        self.return_call()

    def capture_controller(self,u,a,size,data):
        if a==0x101a939a4:
            bp=u.reg_read(UC_X86_REG_RBP)
            self.captured.update(angle_limits=self.read_xmm(11)[:2],
                predicted_wing_angles=[self.read(bp-0x33b8,1)[0],self.read(bp-0x3520,1)[0]],
                overload_timer=self.read(OBJ+0x154,1)[0],
                wing_polar=list(struct.unpack('<24f',u.mem_read(bp-0x35e0,96))))
        elif a==0x101a60b50:
            self.captured.setdefault('predictor_inputs',[]).append(dict(
                target_angle=self.read_xmm(0)[0],target_acceleration=self.read_xmm(1)[0]))
        elif a==0x101a94b54:
            bp=u.reg_read(UC_X86_REG_RBP)
            self.captured['predictor_return_reached']=True
            self.captured['predictor_commands']=[self.read(bp-0x36b4,1)[0],self.read(bp-0x3680,1)[0]]
            self.captured['pitch_clamp_inputs']=dict(
                predictor_commands=self.captured['predictor_commands'],trim=self.read(BASE+0x87f8,1)[0],
                authority_inverse=[self.read(bp-0x3540,1)[0],self.read(bp-0x3460,1)[0]],
                authority_factor=self.read(OBJ+0x150,1)[0],requested=self.read(BASE+0x8518,1)[0],
                predicted_adjusted=[self.read(bp-0x33c0,1)[0],self.read(bp-0x33c4,1)[0]],
                adjustment_offsets=[self.read(bp-0x3454,1)[0],self.read(bp-0x345c,1)[0]],
                critical_high=self.read(bp-0x33e4,1)[0],recovery_reference=self.read(bp-0x3458,1)[0],
                dt=self.read(bp-0x3400,1)[0],adaptation_rates=self.read(0x107209df0,2))
        elif a==0x101a94d8b:
            bp=u.reg_read(UC_X86_REG_RBP)
            self.captured['pitch_clamp']=dict(command_bounds=self.read(bp-0x33a0,2),
                command=self.read(BASE+0x8518,1)[0],authority_factor=self.read(OBJ+0x150,1)[0])
            saved=u.mem_read(INPUT+0x29,1)!=b'\0'
            self.captured['recovery_inputs']=dict(
                commands=[self.read(OBJ+off,1)[0] for off in [0xe4,0xe0,0xe8]] if saved else self.read(BASE+0x8514),
                current_commands=self.read(BASE+0x8514),
                history=[self.read(OBJ+off,1)[0] for off in [0x134,0x130,0x138]],
                command_bounds=[[self.read(bp-0x3550,1)[0],self.read(bp-0x33bc,1)[0]],self.read(bp-0x33a0,2),
                                [self.read(bp-0x3510,1)[0],self.read(bp-0x3470,1)[0]]],
                peak=self.read_xmm(7)[0],reference=self.read_xmm(9)[0],critical_high=self.read_xmm(8)[0],
                current_wing_peak=self.read(bp-0x34a0,1)[0],stored_yaw_rate=self.read(BASE+0x15e8,1,'d')[0],
                dt=self.read(bp-0x3400,1)[0],max_cvt_angle=self.read(0x107e4992c,1)[0],smoothness=self.read(0x107e49930,1)[0])
        elif a==0x101a9517e:
            bp=u.reg_read(UC_X86_REG_RBP)
            self.captured['recovery_output']=dict(commands=self.read(BASE+0x8514),
                history=[self.read(OBJ+off,1)[0] for off in [0x134,0x130,0x138]],
                command_bounds=[self.read(bp-0x3408,2),self.read(bp-0x33a0,2),
                                [self.read(bp-0x3510,1)[0],self.read_xmm(3)[0]]],
                pitch_mix=self.read_xmm(10)[0],axis_mix=self.read_xmm(9)[0])
        elif a==0x101a93b3d:self.captured['angle_rate']=self.read_xmm(7,'2d')[0]
        if a==self.capture_stop:u.emu_stop()

    def invoke(self,address,args=(),xmm=(),stop=None,count=1000000):
        u=self.u; self.captured={}; self.capture_stop=stop
        u.reg_write(UC_X86_REG_RSP,STACK-8);self.qword(STACK-8,STOP)
        for reg,v in zip([UC_X86_REG_RDI,UC_X86_REG_RSI,UC_X86_REG_RDX,UC_X86_REG_RCX,UC_X86_REG_R8,UC_X86_REG_R9],args):u.reg_write(reg,v)
        for i,v in enumerate(xmm):self.xmm(i,[v])
        try:u.emu_start(address,STOP,count=count)
        except Exception as exc:
            raise RuntimeError('Instructor stopped at '+hex(u.reg_read(UC_X86_REG_RIP))+
                '; SP '+hex(u.reg_read(UC_X86_REG_RSP))+'; adapters '+str(self.extra_calls[-12:])) from exc
        pc=u.reg_read(UC_X86_REG_RIP)
        if pc!=(stop or STOP):raise RuntimeError('Instructor did not finish: '+hex(pc))
        return dict(self.captured)

    def setup(self,model,mass,speed=250.,alpha=10.,commands=(0.,-.1,0.),
              flaps=0.,height=0.,dt=1/48,sweep=0.,mode_lane=False,reference_speed_override=None):
        """Prescribed healthy state. NOT a validated game-state initializer.

        dt initializes the controller call and its interval ring, not FM4ea8.
        Moving prediction tests must also prepare the physics timestep, scene
        providers and owner timing. A zero FM timestep skips inner prediction.
        """
        alpha=math.radians(alpha);v=[f32(speed*math.cos(alpha)),f32(-speed*math.sin(alpha)),0.]
        hist=dict(wing_aoa=[math.degrees(alpha)]*2,body_angles=[math.degrees(alpha),0.],wing_cl=[0.,0.],spin=0.)
        super().call(model,v,[0.,0.,0.],mass,list(commands),height,dt,hist,flaps=flaps)
        u=self.u;u.mem_write(ARENA,bytes(0x100000));self.extra_calls=[]
        # Original reset 101a3c6a0 (also called by the FM constructor) sets
        # these values. Leaving all three zero incorrectly selects the
        # alternate body model in 101a3f900 and erases its angular velocity.
        # These are reset defaults, not a captured live difficulty/profile;
        # owner code may change them. Moving fixtures must state that scope.
        self.floats(BASE+0x84e0,[5.])
        u.mem_write(BASE+0x84ec,b'\1')
        self.qword(BASE+0xa29c,0x786000b0)
        self.wings=prepare_wings(model['fm']); wing=select_wing(self.wings,sweep)
        # AircraftNative prepares every installed engine. The Instructor's
        # engine helper dereferences each one's property pointer, including
        # before the reduced predictor. Initialize all records, not only #0.
        # This remains an explicit non-VTOL property fixture.
        engine_count=self.read(OWNER+0x25f78,1,'I')[0]
        for i in range(engine_count):
            engine=self.read(OWNER+0x25f80+8*i,1,'Q')[0]
            self.qword(engine,HEAP+0x1000)
        self.qword(0x107615120,HEAP+0x2000)
        self.qword(HEAP,HEAP+0x3000)
        self.qword(HEAP+0x3000,HEAP+0x4000);self.qword(HEAP+0x4040,ARENA+0x31000)
        self.floats(0x107d6f794,[G]);self.floats(0x107d6f790,[1.225])
        self.invoke(0x101a974e0)
        gameplay=json.loads(Path('references/body-gameplay.blkx').read_text())
        from instructor_mouseaim_settings import configured_fields,aircraft_property_fields
        for off,data in configured_fields(gameplay['mouseAim']):u.mem_write(0x107d70000+off,data)
        self.invoke(0x101a91f60,[OBJ])
        gp=gameplay['instructor']
        for off,key in [(0x24,'ailDegToPredict'),(0x28,'elevonDegToPredict'),(0x30,'instructorSmoothness'),
                        (0x34,'instructorAlphaPrediction'),(0x38,'instructorCritAngleK'),
                        (0x3c,'constPitchPropCoeffBase'),(0x40,'constPitchDerrCoeffBase')]:
            self.floats(0x107e49900+off,[gp[key]])
        u.mem_write(0x107e49944,bytes([gp['reducedFlightModelLoop']]))
        p=selected_properties(model['fm']); e=p['effective_speed'];pw=p['power'];mn=p['minimum']
        for off,values in [(0x7c18,[e[0][0],e[2][0],*e[1]]),(0x7c28,[pw[0][0],pw[2][0],*pw[1]]),
                           (0x7c38,[mn[0],mn[2],mn[1]]),(0x7c48,p['negative']),
                           (0x7c0c,p['max_rate']),(0x8058,[p['trim_rate'][i] for i in [0,2,1]])]:self.floats(BASE+off,values)
        # Research drivers resolve typed duplicate booleans with
        # instructor_source.load before reaching this prepared-state fixture.
        u.mem_write(BASE+0x7c44,bytes([bool(p['strong'])]));u.mem_write(BASE+0x7c54,bytes([bool(p['invert_elevator'])]))
        self.floats(BASE+0x16a8,[1.]);u.mem_write(BASE+0x8470,b'\x01');u.mem_write(BASE+0x7faa,b'\x01\x01\x01')
        u.mem_write(BASE+0x8520,b'\x01\x01\x01')
        self.floats(BASE+0x8514,[0.,-1.,0.])
        self.floats(BASE+0x81b4,wing['geometry']['strength']['force'])
        self.floats(BASE+0x16a4,[sweep])
        u.mem_write(BASE+0x8218,pack_runtime(flap_polar(model['polars']['WingPlane'],flaps),BASE+0x8218))
        self.floats(BASE+0x8254,[wing['geometry']['area']])
        # Packed wing polar overlaps its normalization fields, as in the FM.
        self.floats(BASE+0x8438,[wing['geometry']['area']])
        # Predictor and optional load-factor target read the undivided property
        # areas as well as the intact per-side runtime areas used by full aero.
        for key,span_off,area_off,fields in [
            ('HorStabPlane',0x6f18,0x6f2c,['Main','Elevator']),
            ('VerStabPlane',0x7120,0x7134,['Main','Rudder'])]:
            plane=model['fm']['Aerodynamics'][key]
            self.floats(BASE+span_off,[plane['Span']])
            self.floats(BASE+area_off,[plane['Areas'][field] for field in fields])
        reference=gain_reference_speed(model['fm']['Mass'].get('Takeoff',0.),
            flap_polar(select_wing(self.wings,0.)['polars'],0.))
        self.floats(BASE+0x7f8c,[reference if reference_speed_override is None else reference_speed_override])
        u.mem_write(BASE+0x7fa6,bytes([model['fm']['AvailableControls']['hasFlapsControl']]))
        self.floats(BASE+0x84e4,[1.,1.])
        self.floats(0x107d6fba4,[1.]);self.floats(0x107d6fbb4,[1.])
        u.mem_write(0x107d6fbb8,b'\x01');u.mem_write(0x107d6fbc2,b'\x01')
        self.qword(OBJ,BASE);self.qword(OBJ+8,ARENA+0x4000)
        self.qword(OBJ+0x40,BASE) # Embedded MouseAim controller's aircraft pointer.
        self.qword(0x1083acc20,1);u.mem_write(0x107f124c0,b'\0') # Registered profiler ID; profiling off.
        self.qword(ARENA+0x4000,ARENA+0x5000)
        u.mem_write(ARENA+0x4008,struct.pack('<II',5,0));self.floats(ARENA+0x4010,[dt*5]);self.floats(ARENA+0x5000,[dt]*5)
        props=dict(gp['propsDefault']);props.update(model['fm'].get('Instructor',{}))
        # Constructor copied authored global MouseAim defaults. The aircraft
        # loader only overwrites present names; its defaults differ from the
        # global propsDefault loader for partial/legacy property blocks.
        for off,data in aircraft_property_fields(props.get('MouseAim',{})):
            u.mem_write(OBJ+0x48+off-0x110,data)
        self.floats(OBJ+0xec,props['critMult']);u.mem_write(OBJ+0xf4,bytes([props['limitOverload']]))
        self.floats(OBJ+0xf8,props['overloadMult']);self.floats(OBJ+0x108,[props['overloadTimeRate'],*props['overloadTimeRange']])
        u.mem_write(OBJ+0x114,bytes([props.get('limitLoadfactor',False)]))
        self.floats(OBJ+0x118,[mul(f32(x),G) for x in props.get('loadFactorLimit',[-5.,12.])])
        self.floats(OBJ+0x120,props.get('constPitchDerrCoeffMultByClLinearRatio',[0.,1.,.01,2.]))
        self.floats(OBJ+0x148,self.read(BASE+0x1678,2));self.floats(OBJ+0x150,[1.,0.])
        for off in [1,7,8,0xc,0x28]:u.mem_write(INPUT+off,b'\x01')
        u.mem_write(INPUT+0xd,bytes([mode_lane]));self.floats(INPUT+0x10,[1.])
        self.floats(0x107d70080,[.8]);u.mem_write(0x107d7017c,b'\x00')
        self.dt=dt
        self.properties=props

    def step(self,stop=None):
        return self.invoke(0x101a92670,[OBJ,INPUT,OUTPUT],[self.dt],stop=stop)

    def empty_payload_actor(self):
        """Actual owner interfaces for an explicit zero-payload research fixture.

        FM back-offset points at Unit+0x100, whose real vtable is installed by
        104f4aa84. Payload getter 1050ed810 reads the collection through +fd0;
        the fixture intentionally has no payload provider. It is not a claim
        about the equipment state of a spawned aircraft. No getter is hooked.
        The embedded snapshot interface is initialized from 1050b9663..9684.
        """
        actor=ARENA+0x80000
        self.qword(actor+0x100,0x107cb1430)
        self.qword(BASE+8,(BASE-actor-0x100)&((1<<64)-1))
        self.qword(BASE,0x1078da710)
        self.qword(actor+0x16b0,0x107cb5898);self.qword(actor+0x16b8,actor)
        self.u.mem_write(actor+0x16c0,b'\xff\xff')
        # Snapshot restoration reads simulation time through 104e59ce0 even
        # when no actor flags changed. Supply an explicit zero-time world:
        # no replay/network clock, no alternate +400 clock provider. Execute
        # the original getter and restore callback rather than bypass either.
        self.qword(0x107f5fdf8,ARENA+0x90000)
        return actor


if __name__=='__main__':
    from aircraft_model import prepare
    from mass_model import aircraft_properties,evaluate
    fm=json.loads(Path('references/jet-catalog/fm/f_16a_block_15_adf.blkx').read_text())
    mass=evaluate(aircraft_properties(fm),fuel_by_system=[1000.])
    n=InstructorNative();n.setup(prepare(fm),mass)
    print(n.step(stop=0x101a939a4))
    print('adapters',sorted(set(n.extra_calls)))
