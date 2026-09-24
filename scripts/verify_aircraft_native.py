"""Contiguous detailed-aircraft aerodynamic execution against a Python port.

Prepared FM state and runtime property adapters are explicit boundaries.
This stops before propulsion/gravity and is not a complete native timestep.
"""
import hashlib,json,math,struct,random
from pathlib import Path
from unicorn import Uc,UC_ARCH_X86,UC_MODE_64,UC_HOOK_CODE,UC_HOOK_MEM_INVALID
from unicorn.x86_const import *
from macho_scan import MachO
from verify_component_assembly import AssemblySlice,BASE,FRAME,FORCE_OFFSETS,POINT_OFFSETS
from verify_control_mixer import Mixer
from verify_polar_machine_code import EXPECTED_BINARY_SHA256
from component_assembly import f32,mul,add
from polar_runtime import pack_runtime,flap_polar
from air_state import cache
from tail_model import aircraft_secondary_properties
from aircraft_model import prepare,evaluate

OWNER=BASE+0x30000;CONFIG=BASE+0x5a000;PARAM=BASE+0x5b000;END=BASE+0x6f000

class AircraftNative:
    floats=AssemblySlice.floats;xmm=AssemblySlice.xmm;read_xmm=AssemblySlice.read_xmm;table=Mixer.table
    def __init__(self,binary_path=None):
        m=MachO(binary_path) if binary_path else MachO();self.sha=hashlib.sha256(m.data).hexdigest()
        assert self.sha==EXPECTED_BINARY_SHA256
        self.u=Uc(UC_ARCH_X86,UC_MODE_64)
        for s in m.segments:
            if s['name'] in ['__PAGEZERO','__LINKEDIT']:continue
            self.u.mem_map(s['va'],s['size']);self.u.mem_write(s['va'],m.data[s['offset']:s['offset']+s['filesize']])
        self.u.mem_map(BASE,0x80000)
        for va in [0x100233d00,0x106e615f1,0x106e614b9,0x106e614b3,0x106e61c1b,0x106e6164b,0x106e6149b,
                   0x101a27d80,0x1019e20d0]:
            self.u.hook_add(UC_HOOK_CODE,self.hook,begin=va,end=va)
        self.u.hook_add(UC_HOOK_MEM_INVALID,self.invalid)
        self.calls=[]
        for a in [0x106c60c95,0x106c6191a,0x106c62d3a]:self.u.hook_add(UC_HOOK_CODE,self.capture,begin=a,end=a)
    def capture(self,u,a,size,data):
        if a==0x106c62d3a:
            zx=self.read_xmm(6,'2d');self.debug['raw_force']=[zx[1],self.read_xmm(2,'2d')[0],zx[0]];return
        if a==0x106c6191a:
            self.debug.update(controls=[self.read(FRAME-0xb68,5),self.read(FRAME-0xa68,5)],control_input=self.read(FRAME-0xbf8,9),wake_angles=self.read(FRAME-0x4a0,2));return
        self.debug=dict(wing_cl=[self.read(FRAME-0x4b0,1)[0],self.read(FRAME-0x4a0,1)[0]],base_points=[self.read(FRAME-0x650),self.read(FRAME-0x640)],extra_area=self.read(FRAME-0x5f8,1),hpolar=self.read(FRAME-0xc58,24),vpolar=self.read(FRAME-0x3f0,24),history_scales=[self.read(FRAME-0x828,1),self.read(FRAME-0x6c0,1)],disturbance=self.read(FRAME-0x490,2)+self.read(FRAME-0x480,1))
    def invalid(self,u,access,address,size,value,data):
        raise RuntimeError('Unmapped '+hex(address)+' at '+hex(u.reg_read(UC_X86_REG_RIP)))
    def qword(self,a,v):self.u.mem_write(a,struct.pack('<Q',v))
    def doubles(self,a,v):self.u.mem_write(a,struct.pack('<'+'d'*len(v),*v))
    def read(self,a,n=3,kind='f'):return list(struct.unpack('<'+str(n)+kind,self.u.mem_read(a,struct.calcsize(kind)*n)))
    def hook(self,u,a,size,data):
        self.calls.append(hex(a));x=self.read_xmm(0)[0]
        if a==0x100233d00:
            self.floats(u.reg_read(UC_X86_REG_RDI),[math.sin(x)]);self.floats(u.reg_read(UC_X86_REG_RSI),[math.cos(x)])
        elif a==0x106e615f1:self.xmm(0,[math.exp(x)])
        elif a==0x106e614b9:self.xmm(0,[math.atan2(x,self.read_xmm(1)[0])])
        elif a==0x106e614b3:
            value=math.atan2(self.read_xmm(0,'2d')[0],self.read_xmm(1,'2d')[0])
            u.reg_write(UC_X86_REG_XMM0,int.from_bytes(struct.pack('<2d',value,0.),'little'))
        elif a==0x106e61c1b:self.xmm(0,[math.sin(x)])
        elif a==0x106e6164b:self.xmm(0,[math.fmod(x,self.read_xmm(1)[0])])
        elif a==0x106e6149b:self.xmm(0,[math.asin(x)])
        elif a==0x101a27d80:
            from control_mixer import curve
            # Empty-table branch 101a27e68..e80 forwards the flap fraction to
            # blend/animation, with zero stabilizer/slat bias (legacy jets).
            self.floats(u.reg_read(UC_X86_REG_RSI),curve(self.model['flaps'],x,4) if self.model['flaps'] else [x,x,0.,0.])
        elif a==0x1019e20d0:
            out=u.reg_read(UC_X86_REG_RSI);u.mem_write(out,pack_runtime(flap_polar(self.model['polars']['WingPlane'],x),out))
        else:raise RuntimeError('Unexpected provider hook '+hex(a))
        sp=u.reg_read(UC_X86_REG_RSP);ret=struct.unpack('<Q',u.mem_read(sp,8))[0]
        u.reg_write(UC_X86_REG_RSP,sp+8);u.reg_write(UC_X86_REG_RIP,ret)
    def mixer_properties(self,address,p):
        for i in range(3):
            self.floats(address+8*i,p['angles'][i]);self.table(address+0x18+24*i,p['tables_1d'][i]);self.table(address+0x60+24*i,p['tables_2d'][i],True)
        self.floats(address+0xa8,[p['sensitivity']]);self.table(address+0xb0,p['sensitivity_curve']);self.table(address+0xc8,p['arcade_curve'])
        self.floats(address+0xe0,p['cl']+p['cd']+[p['wing_aoa']])
    def call(self,model,v,w,mass,commands,height,dt,history,flaps=0.,gear=0.,airbrake=0.,throttle=1.1,ground_height=0.,full_update=False,quaternion=(0.,0.,0.,1.),engine_wash=(0.,0.),torque_gyro=True):
        self.model=model;self.calls=[];u=self.u;u.mem_write(BASE,bytes(0x80000));self.cursor=BASE+0x60000
        fm=model['fm'];ad=fm['Aerodynamics'];g=model['geometry'];air=cache(v,height)
        for n in range(16):self.xmm(n,[0.])
        self.qword(0x107615358,BASE+0x6e000)
        self.qword(BASE+0x6ed8,CONFIG);self.qword(BASE+0x55a0,OWNER)
        self.floats(OWNER+0x25b58,engine_wash);u.mem_write(BASE+0x3658,bytes([2 if torque_gyro else 0]))
        self.doubles(BASE+0x1558,[0.,height,0.]);self.floats(BASE+0x1570,quaternion)
        for offset,key,default in [(0x7c08,'AllowModsToChangeLongidutialBalance',True),(0x7c09,'RollLeveling',True),(0x7c0a,'ConvertAoa',False)]:
            u.mem_write(BASE+offset,bytes([bool(fm.get(key,default))]))
        self.doubles(BASE+0x15b0,v);self.doubles(BASE+0x1618,v);self.doubles(BASE+0x15e0,w)
        self.floats(BASE+0x8454,[air[k] for k in ['alpha','beta','tas','speed_squared','mach','ias_u']])
        self.floats(BASE+0x5308,[mass['mass']]);self.floats(BASE+0x5320,mass['cog']);self.doubles(BASE+0x5348,mass['inertia'])
        self.floats(BASE+0x1694,commands);self.floats(BASE+0x1678,history['wing_aoa']);self.floats(BASE+0x1680,history['body_angles']);self.floats(BASE+0x1688,history['wing_cl']);self.floats(BASE+0x1690,[history['spin']])
        self.qword(BASE+0x1860,0xffffffffffffffff);self.floats(BASE+0x18a0,[1.]*19)
        self.floats(BASE+0x8400,g['areas'][0]+g['areas'][1]);self.floats(BASE+0x843c,[1.,1.])
        self.floats(BASE+0x8138,[g['span'],g['incidence'],*g['arm'],g['sweep'],g['taper'],g['dihedral']])
        from control_mixer import curve
        blend=curve(model['flaps'],f32(flaps),4)[0] if model['flaps'] else f32(flaps)
        u.mem_write(BASE+0x8218,pack_runtime(flap_polar(model['polars']['WingPlane'],blend),BASE+0x8218))
        for off,key in [(0x8188,'FlapsShift'),(0x8190,'AirbrakesShift'),(0x8198,'GearShift'),(0x81a0,'ElevonShift')]:self.floats(BASE+off,g['shifts'][key])
        self.floats(BASE+0x8174,[g['sine_aos'],g['v_focus']]);u.mem_write(BASE+0x817c,bytes([g['use_spin_loss']]));self.floats(BASE+0x8180,g['spin_loss']);self.floats(BASE+0x81a8,[g['aoa_shift']])
        self.qword(BASE+0x81c8,self.cursor);u.mem_write(BASE+0x81d8,struct.pack('<I',len(g['aoa_shift_add'])))
        for i,(x,y) in enumerate(g['aoa_shift_add']):
            inv=f32(1/f32(g['aoa_shift_add'][i+1][0]-x)) if i+1<len(g['aoa_shift_add']) else 0.
            self.floats(self.cursor+12*i,[x,inv,y])
        self.cursor+=max(16,len(g['aoa_shift_add'])*12)
        u.mem_write(BASE+0x81ac,struct.pack('<I',g['downwash_type']));self.floats(BASE+0x81b0,[g['downwash_coefficient']]);self.floats(BASE+0x8254,[g['area']])
        self.floats(BASE+0x7530,[fm['Length']]);self.floats(BASE+0x6cfc,[ground_height]);self.floats(BASE+0x8494,[f32(height-ground_height)])
        for off,val in [(0x2b50,gear),(0x2b58,airbrake),(0x2b60,flaps)]:self.floats(BASE+off,[val])
        for off in [0xc4,0x134,0x1a4]:u.mem_write(CONFIG+off,b'\x01')
        self.floats(BASE+0x7ca8,[ad.get(k,0.) for k in ['GearCd','GearCentralCd','AirbrakeCd','RadiatorCd','OilRadiatorCd','BombBayCd','FuseCd','CockpitDoorCd']])
        self.floats(BASE+0x7c8c,[0.,1.,0.,1.])
        tp=aircraft_secondary_properties(fm,{})
        self.floats(BASE+0x7140,[tp['slipstream_distance']]);u.mem_write(BASE+0x6f3c,bytes([tp['clockwise']]))
        for off,key in [(0x6f40,'HorStabPlane'),(0x7148,'VerStabPlane'),(0x7350,'FuselagePlane')]:
            u.mem_write(BASE+off,pack_runtime(model['polars'][key][0][1],BASE+off))
        for name,off in [('hstab',0x6f1c),('vstab',0x7124),('fuselage',0x732c)]:
            self.floats(BASE+off,[tp['incidence'].get(name,0.),*tp['arms'][name]])
        for name,off in [('hstab',0x6f34),('vstab',0x713c),('fuselage',0x7344)]:self.floats(BASE+off,[tp['flow_inertia'][name]])
        for key,off in dict(fuselage=0x83fc,left_main=0x8420,left_elevator=0x8424,right_main=0x8428,right_elevator=0x842c,v_main=0x8430,rudder=0x8434).items():self.floats(BASE+off,[tp['areas'][key]])
        for off,key in [(0x78d0,'Ailerons'),(0x79c8,'Elevator'),(0x7ac0,'Rudder')]:self.mixer_properties(BASE+off,model['controls'][key])
        self.floats(PARAM+4,[1.]);self.floats(PARAM+0x14,[1.]);u.mem_write(PARAM+0x18,b'\x01');u.mem_write(PARAM+0x22,b'\x01');self.floats(PARAM+0x40,[-1.])
        self.floats(0x107d6fc54,[1.])
        # One healthy running engine, with its current afterburner/throttle request.
        engine_count=tp['engine_count'] or 1
        u.mem_write(OWNER+0x25f78,struct.pack('<I',engine_count))
        for i in range(engine_count):
            address=BASE+0x58000+0x200*i
            self.qword(OWNER+0x25f80+8*i,address);u.mem_write(address+0x1c,b'\x07');self.floats(address+0x58,[1.]);self.floats(address+0xa4,[throttle])
        u.reg_write(UC_X86_REG_RSP,FRAME+8);self.qword(FRAME+8,END)
        for reg,val in [(UC_X86_REG_RDI,BASE),(UC_X86_REG_RSI,0),(UC_X86_REG_RDX,int(full_update)),(UC_X86_REG_RCX,0),(UC_X86_REG_R8,PARAM)]:u.reg_write(reg,val)
        self.xmm(0,[dt])
        # A previously cached full-function translation can cross emu_start's
        # until address. Enforce this audit boundary even after an inner
        # prediction has executed the same original function to its return.
        pause=u.hook_add(UC_HOOK_CODE,lambda machine,address,size,data:machine.emu_stop(),
                         begin=0x106c6331e,end=0x106c6331e)
        try:u.emu_start(0x106c5c7d0,0x106c6331e,count=200000)
        except Exception:
            print('PC',hex(u.reg_read(UC_X86_REG_RIP)),'RECENT_HOOKS',self.calls[-8:]);raise
        finally:u.hook_del(pause)
        if u.reg_read(UC_X86_REG_RIP)!=0x106c6331e:raise RuntimeError('Did not reach moment end')
        return dict(debug=self.debug,forces={k:self.read(FRAME-v) for k,v in FORCE_OFFSETS.items()},points={k:self.read(FRAME-v) for k,v in POINT_OFFSETS.items()},moment=[self.read_xmm(2,'2d')[0],*self.read_xmm(0,'2d')],capped_force=self.read(FRAME-0x4e0,2,'d')+self.read(FRAME-0x4d0,1,'d'),omega=self.read(BASE+0x15e0,3,'d'),history=dict(wing_aoa=self.read(BASE+0x1678,2),body_angles=self.read(BASE+0x1680,2),wing_cl=self.read(BASE+0x1688,2),spin=self.read(BASE+0x1690,1)[0]))

def main():
    from mass_model import aircraft_properties,evaluate as mass_eval
    from body_dynamics import limit_aerodynamic_force
    m=AircraftNative();rng=random.Random(16003919);failures=[];cases=0;errors={};providers=set();coverage=dict(force_cap=0,spin_active=0,angular_cap=0,ground_effect=0,chained_history=0)
    for n in ['f_16a_block_15_adf','saab_jas39c']:
        fm=json.loads(Path('references/fm-2.59.0.13/'+n+'.blkx').read_text());model=prepare(fm)
        ah=bh=None
        for i in range(1000):
            mass=mass_eval(aircraft_properties(fm),fuel_by_system=[f32(rng.uniform(100.,2000.))])
            v=list(map(f32,[rng.uniform(40,700),rng.uniform(-150,70),rng.uniform(-60,60)]))
            w=[rng.uniform(-.6,.6) for _ in range(3)];commands=[f32(rng.uniform(-1,1)) for _ in range(3)]
            height=f32(rng.uniform(0,17000));dt=f32(rng.choice([1/30,1/60,1/120]));ground=f32(height-rng.uniform(0,100))
            hist=dict(wing_aoa=[f32(rng.uniform(-10,45)) for _ in range(2)],body_angles=[f32(rng.uniform(-10,45)),f32(rng.uniform(-20,20))],wing_cl=[f32(rng.uniform(-.5,1.5)) for _ in range(2)],spin=f32(rng.uniform(0,1.5)))
            kw=dict(flaps=f32(rng.random()) if n.startswith('f_16') else 0.,gear=f32(rng.random()),airbrake=f32(rng.random()),throttle=f32(rng.uniform(.1,1.1)))
            if 800<=i<900:
                # Extreme and exact-axis inputs supplement ordinary random states.
                boundary=i-800;speed=[.1,1.,40.,250.,700.,1200.,2000.][boundary%7]
                alpha=math.radians([-80,-30,0,12,25,45,80][(boundary//7)%7]);beta=math.radians([-35,0,35][boundary%3])
                v=list(map(f32,[speed*math.cos(alpha)*math.cos(beta),-speed*math.sin(alpha),speed*math.cos(alpha)*math.sin(beta)]))
                w=[0.,0.,0.] if boundary%2 else [6.,-7.,8.]
                commands=[[0.,0.,0.],[1.,1.,1.],[-1.,-1.,-1.]][boundary%3]
                height=f32([0.,1000.,18300.,22000.][boundary%4]);ground=f32(height-[0.,1.,.4*fm['Length'],100.][(boundary//4)%4])
                kw.update(gear=[0.,1.,127/255,128/255][boundary%4],airbrake=[0.,1.][boundary%2])
            if i>=900:
                phase=(i-900)/100.;height=1000.;ground=0.;v=list(map(f32,[250.,-40.+20.*math.sin(phase*12.),8.*math.cos(phase*6.)]));w=[.1,-.05,.15];commands=[.1,f32(-.3+.2*math.sin(phase*12.)),0.]
                kw.update(flaps=0.,gear=0.,airbrake=0.);dt=f32(1/60)
                if i>900:hist=ah;coverage['chained_history']+=1
            args=[model,v,w,mass,commands,height,dt,hist]
            a=m.call(*args,ground_height=ground,**kw)
            providers.update(m.calls)
            if i>900:args[-1]=bh
            b=evaluate(*args,oil_radiator=0.,height_agl=f32(height-ground),ground_effect_height=height-ground,**kw)
            ah=a['history'];bh=b['history']
            comparisons=[('forces.'+k,x,b['component_forces'][k]) for k,x in a['forces'].items()]+[('points.'+k,x,b['component_points'][k]) for k,x in a['points'].items()]
            capped=limit_aerodynamic_force(b['raw_aero_force'],mass['mass'])
            comparisons += [('moment',a['moment'],b['raw_aero_moment']),('raw_force',a['debug']['raw_force'],b['raw_aero_force']),('capped_force',a['capped_force'],capped),('omega',a['omega'],b['omega_for_flow'])]
            coverage['force_cap']+=capped!=b['raw_aero_force'];coverage['spin_active']+=a['history']['spin']>0.;coverage['angular_cap']+=sum(x*x for x in w)>25.;coverage['ground_effect']+=height-ground<.4*fm['Length']
            comparisons += [('history.'+k,x if isinstance(x,list) else [x],b['history'][k] if isinstance(x,list) else [b['history'][k]]) for k,x in a['history'].items()]
            mismatch=[]
            for k,x,y in comparisons:
                errors[k]=max(errors.get(k,0.),max(abs(p-q) for p,q in zip(x,y)))
                if x!=y:mismatch.append(dict(field=k,actual=x,expected=y))
            if mismatch:
                failures.append(dict(aircraft=n,index=i,velocity=v,omega=w,mass=mass,commands=commands,height=height,dt=dt,history=hist,ground=ground,devices=kw,mismatches=mismatch,debug=a['debug']))
            cases+=1
    report=dict(binary_sha256=m.sha,seed=16003919,span=['0x106c5c7d0','0x106c6331e'],cases=cases,coverage=coverage,substituted_calls=sorted(providers),max_absolute_errors=errors,failing_cases=len(failures),failures=failures[:12],limitations='Prepared intact full-real FM state. Native prologue, wing, wake, secondary surfaces, raw force/moment assembly and aerodynamic force cap. Flap mapping/property selector and scalar math substituted. Chained cases propagate aerodynamic history only; kinematics and engine/control state remain prescribed. Does not execute primary actuators, engine force, gravity or full timestep.')
    Path('analysis/aircraft-native-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='failures'},indent=2))
    if failures:raise SystemExit(1)

if __name__=='__main__':main()
