"""Original orientation helper and contiguous airborne integration checks."""
import json,math,random,struct
from pathlib import Path
from unicorn import UC_HOOK_CODE
from unicorn.x86_const import *
from verify_aircraft_body_native import AircraftBodyNative
from verify_aircraft_native import BASE,FRAME,PARAM
from component_assembly import f32
from kinematics import orientation_increment,airborne_step,altitude_velocity_correction
from aircraft_model import prepare,evaluate
from mass_model import aircraft_properties,evaluate as mass_eval

class KinematicsNative(AircraftBodyNative):
    def __init__(self):
        super().__init__()
        self.u.hook_add(UC_HOOK_CODE,self.allocator_provider,begin=0x10001ee10,end=0x10001ee10)
    def allocator_provider(self,u,address,size,data):
        # 10001ee10 returns the address of a thread-local allocator pointer.
        # No allocation occurs before the no-contact integration stop.
        self.qword(BASE+0x6d100,BASE+0x6d200);u.reg_write(UC_X86_REG_RAX,BASE+0x6d100)
        sp=u.reg_read(UC_X86_REG_RSP);ret=struct.unpack('<Q',u.mem_read(sp,8))[0]
        u.reg_write(UC_X86_REG_RSP,sp+8);u.reg_write(UC_X86_REG_RIP,ret)
    def orientation(self,q,angles):
        self.floats(BASE+0x1570,q)
        for i,x in enumerate(angles):self.xmm(i,[x])
        self.u.reg_write(UC_X86_REG_RDI,BASE+0x1570);self.u.reg_write(UC_X86_REG_RSP,FRAME)
        self.qword(FRAME,BASE+0x6f000)
        self.u.emu_start(0x10198bd30,BASE+0x6f000,count=1000)
        return self.read(BASE+0x1570,4)
    def altitude_correction(self,height,upward_velocity,dt,arcade_boost=False):
        self.doubles(BASE+0x1560,[height]);self.doubles(BASE+0x15b8,[upward_velocity])
        self.floats(FRAME-0x5c0,[dt]);self.u.mem_write(PARAM+0x10,bytes([arcade_boost]))
        self.u.reg_write(UC_X86_REG_RBX,BASE);self.u.reg_write(UC_X86_REG_RBP,FRAME)
        self.u.reg_write(UC_X86_REG_RSI,PARAM)
        self.u.emu_start(0x106c64da5,0x106c64fac,count=300)
        if self.u.reg_read(UC_X86_REG_RIP)!=0x106c64fac:raise RuntimeError('Altitude boundary not reached')
        return self.read(BASE+0x15b8,1,'d')[0]
    def integrate(self,sanity_factor=1.):
        self.u.reg_write(UC_X86_REG_R15,PARAM+0x1000)
        self.u.mem_write(PARAM+0x1010,struct.pack('<I',1));self.floats(PARAM+0x1014,[sanity_factor])
        self.u.emu_start(0x106c63e7f,0x106c649ed,count=5000)
        if self.u.reg_read(UC_X86_REG_RIP)!=0x106c649ed:raise RuntimeError('Integration boundary not reached')
        return dict(position=self.read(BASE+0x1558,3,'d'),velocity=self.read(BASE+0x15b0,3,'d'),
                    quaternion=self.read(BASE+0x1570,4),omega=self.read(BASE+0x15e0,3,'d'),
                    angular_acceleration=self.read(BASE+0x15f8,3,'d'),world_acceleration=self.read(BASE+0x15c8,3,'d'),
                    reset=bool(self.u.mem_read(BASE+0x88e0,1)[0]))

def main():
    machine=KinematicsNative();rng=random.Random(16003921);failures=[];counts={};errors={}
    def check(kind,a,b):
        counts[kind]=counts.get(kind,0)+1
        err=max(abs(x-y) for x,y in zip(a,b));errors[kind]=max(errors.get(kind,0.),err)
        if a!=b:failures.append(dict(kind=kind,actual=a,expected=b))
    for i in range(2000):
        q=[f32(rng.uniform(-1,1)) for _ in range(4)];angles=[f32(rng.uniform(-360,360)) for _ in range(3)]
        check('orientation',machine.orientation(q,angles),orientation_increment(q,angles))
    heights=[9900.,9900.+1e-8,9900.+1e-4,9900.-1e-8,15999.99999,16000.,16000.00001]
    for i in range(2000):
        h=heights[i] if i<len(heights) else rng.uniform(0.,30000.)
        v=[0.,1e-8,-1e-8,.001][i%4] if i%2 else rng.uniform(-200.,200.)
        dt=f32(rng.choice([1/30,1/48,1/60,1/120]));boost=bool(i%2)
        check('altitude_correction',[machine.altitude_correction(h,v,dt,boost)],
              [altitude_velocity_correction(h,v,dt,arcade_boost=boost)])
    for n in ['f_16a_block_15_adf','saab_jas39c']:
        fm=json.loads(Path('references/fm-2.59.0.13/'+n+'.blkx').read_text());model=prepare(fm)
        for i in range(500):
            mass=mass_eval(aircraft_properties(fm),fuel_by_system=[1000.]);v=[250.,-20.,0.];w=[rng.uniform(-2,2) for _ in range(3)];dt=f32(rng.choice([1/30,1/48,1/60,1/120]))
            hist=dict(wing_aoa=[5.,5.],body_angles=[5.,0.],wing_cl=[.5,.5],spin=0.)
            args=[model,v,w,mass,[0.,0.,0.],1000.,dt,hist];a=machine.call(*args);b=evaluate(*args,oil_radiator=0.,height_agl=1000.)
            q=[rng.uniform(-1,1) for _ in range(4)];norm=math.sqrt(sum(x*x for x in q));q=[f32(x/norm) for x in q]
            machine.floats(BASE+0x1570,q)
            machine.extend(b['engine_force'],b['engine_moment'],[0.]*3,[0.]*3,1.,fm.get('ExtThrustBaseMult',1.),dt)
            factor=f32([1.,1e-7,100.][i%3])
            actual=machine.integrate(factor);expected=airborne_step([0.,1000.,0.],v,q,b['omega_for_flow'],b['force'],b['stored_moment'],mass['mass'],mass['inertia'],dt,sanity_factor=factor)
            for k in expected:
                check('integration.'+k,[actual[k]] if k=='reset' else actual[k],[expected[k]] if k=='reset' else expected[k])
    report=dict(binary_sha256=machine.sha,counts=counts,max_absolute_errors=errors,failures=failures[:12],failure_count=len(failures),limitations='Full original orientation helper, contiguous selected-jet airborne integration through106c649ed, and separate altitude velocity correction. Prepared engine vectors and thread-local allocator-pointer provider. No allocation/contact branches executed; intervening pose-restoration and contact step not included.')
    Path('analysis/kinematics-validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
    if failures:raise SystemExit(1)

if __name__=='__main__':main()
