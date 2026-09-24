"""Differential execution of body gravity, rotation, cap, Euler and integration."""
import json,math,random,struct
from pathlib import Path
from unicorn import UC_HOOK_CODE
from unicorn.x86_const import *
from macho_scan import MachO
from component_assembly import f32
from verify_component_stages import Stages,BASE,FRAME
from body_dynamics import gravity_body,rotate_acceleration_to_world,limit_total_moment,limit_aerodynamic_force,angular_acceleration,integrate_translation,compose_force,detailed_path,realistic_engine_scale,nozzle_direction_from_sincos,accumulate_nozzle,gyroscopic_moment,preprocess_angular_rate,parasite_force,compose_moment,airborne_linear_acceleration

class BodyMachine(Stages):
    def __init__(self):
        super().__init__();m=MachO()
        for va,size in [(0x106c5c000,0x2000),(0x106c64000,0x3000),(0x10198b000,0x1000),(0x101a3f000,0x1000),(0x1019f1000,0x2000),(0x101a17000,0x1000)]:
            self.u.mem_map(va,size);self.u.mem_write(va,m.read(va,size))
    def doubles(self,addr,values):self.u.mem_write(addr,struct.pack('<'+'d'*len(values),*values))
    def xd(self,n,values):self.u.reg_write(UC_X86_REG_XMM0+n,int.from_bytes(struct.pack('<2d',*(list(values)+[0,0])[:2]),'little'))
    def rd(self,addr,n=3):return list(struct.unpack('<'+'d'*n,self.u.mem_read(addr,n*8)))
    def rate_preprocess(self,w,ias):
        self.reset();self.doubles(BASE+0x15e0,w);self.floats(BASE+0x8468,[ias])
        self.u.emu_start(0x106c5ce04,0x106c5cea1,count=100)
        return self.rd(BASE+0x15e0)
    def parasite(self,v,rho,doorcd,door,attachments,intact):
        self.reset();self.doubles(BASE+0x1618,v);self.floats(FRAME-0x4c0,[f32(f32(rho)*.5)])
        self.u.reg_write(UC_X86_REG_R13,0x900)
        self.u.emu_start(0x106c6051d,0x106c60575,count=100)
        self.floats(BASE+0x7cc4,[doorcd]);self.floats(BASE+0x8448,[attachments]);self.floats(BASE+0x16a0,[door])
        self.u.mem_write(BASE+0x1864,bytes([intact]));self.xmm(2,[1])
        self.u.emu_start(0x106c626a7,0x106c627a8,count=150)
        return self.read3(FRAME-0x910)
    def gravity(self,q,mass):
        self.reset();self.floats(BASE+0x1570,q);self.xmm(11,[f32(f32(9.81)*f32(mass))])
        self.u.emu_start(0x106c6383d,0x106c638e2,count=200)
        return [*self.read_xmm(2,'2d'),self.read_xmm(1,'2d')[0]]
    def rotation(self,q,a):
        self.reset();self.floats(BASE+0x1570,q);self.xd(11,a[:2]);self.xd(12,[a[2]])
        self.u.emu_start(0x106c641ea,0x106c64348,count=200)
        return self.rd(BASE+0x15c8)
    def cap(self,t,mass):
        self.reset();self.doubles(FRAME-0x490,t[:2]);self.doubles(FRAME-0x480,[t[2]])
        self.xmm(11,[f32(f32(9.81)*f32(mass))])
        self.u.emu_start(0x106c63e09,0x106c63e7f,count=100)
        return self.rd(FRAME-0x4f0,2)+self.rd(FRAME-0x510,1)
    def force_cap(self,f,mass):
        self.reset();self.xd(6,[f[2],f[0]]);self.xd(0,[f[1]]);self.xmm(8,[f32(f32(9.81)*f32(mass))]);self.u.reg_write(UC_X86_REG_R12,0)
        self.u.emu_start(0x106c62dca,0x106c62ea1,count=100)
        return self.rd(FRAME-0x4e0,2)+[self.read_xmm(5,'2d')[0]]
    def angular(self,w,i,t,c):
        self.reset();self.doubles(BASE+0x15e0,w);self.doubles(BASE+0x5348,i)
        self.doubles(FRAME-0x4f0,[t[0]]);self.doubles(FRAME-0x560,[t[1]]);self.doubles(FRAME-0x510,[t[2]])
        self.xd(3,[c[0]]);self.xd(5,[c[1]]);self.xd(9,[c[2]]);self.xd(4,[f32(4e-19)])
        self.u.emu_start(0x106c63ffd,0x106c640fc,count=200)
        return self.rd(BASE+0x15f8)
    def translation(self,p,v,a,dt):
        self.reset();self.doubles(BASE+0x1558,p)
        self.xd(10,[v[0]]);self.xd(11,v[1:]);self.xd(9,[a[0]]);self.xd(8,a[1:])
        self.doubles(FRAME-0x480,[dt,0]);self.doubles(FRAME-0x7a0,[(dt*dt)*.5]);self.doubles(FRAME-0x5d0,[(dt*dt)*.5,dt]);self.doubles(FRAME-0x490,[dt,dt])
        self.u.emu_start(0x106c64411,0x106c644be,count=120)
        return self.rd(BASE+0x1558),self.rd(BASE+0x15b0)
    def compose(self,a,e,t,s):
        self.reset();self.doubles(FRAME-0x4f0,t[:2]);self.doubles(FRAME-0x4e0,a[:2]);self.doubles(FRAME-0x4c0,list(map(f32,e[:2])));self.doubles(FRAME-0x4b0,[f32(e[2])]);self.doubles(FRAME-0x510,[f32(s),f32(s)]);self.doubles(FRAME-0x500,[t[2]*f32(s)]);self.xd(2,[a[2]])
        self.u.emu_start(0x106c6374a,0x106c6377a,count=50)
        self.u.emu_start(0x106c637d7,0x106c637fb,count=50)
        return self.rd(FRAME-0x4e0,2)+self.rd(FRAME-0x4d0,1)
    def moment_compose(self,a,e,t,g):
        self.reset();self.xd(7,a[:2]);self.xd(6,[a[2]])
        self.doubles(FRAME-0x490,list(map(f32,e[:2])));self.doubles(FRAME-0x500,[f32(e[2])])
        self.u.emu_start(0x106c634da,0x106c634ea,count=30)
        self.u.mem_write(BASE+0x55a0,struct.pack('<Q',BASE));self.doubles(BASE+0x25a60,t)
        self.u.emu_start(0x106c63616,0x106c6366f,count=50)
        self.xd(0,g[:2]);self.xd(1,[g[2]])
        self.u.emu_start(0x106c63708,0x106c63730,count=30)
        return self.rd(FRAME-0x490,2)+self.rd(FRAME-0x480,1)
    def linear(self,force,gravity,mass):
        self.reset();self.xmm(0,[mass]);self.xd(13,gravity[:2]);self.xd(14,[gravity[2]])
        self.u.emu_start(0x106c63bbc,0x106c63c32,count=50)
        self.xd(11,gravity[:2]);self.xd(12,[gravity[2]])
        self.doubles(FRAME-0x4e0,force[:2]);self.doubles(FRAME-0x4d0,[force[2]])
        self.u.emu_start(0x106c641c6,0x106c641ea,count=30)
        return list(self.read_xmm(11,'2d'))+[self.read_xmm(12,'2d')[0]]
    def baseline_tail_scales(self,velocity,intact):
        self.reset();self.doubles(BASE+0x15b0,velocity)
        self.u.reg_write(UC_X86_REG_R14,BASE+0x10000);self.u.reg_write(UC_X86_REG_R13,0x900 if intact else 0)
        self.xmm(11,[velocity[0]])
        self.u.emu_start(0x106c5d4ea,0x106c5db75,count=100)
        return [self.read3(FRAME-0x6c0)[0],self.rd(FRAME-0x6f0,1)[0],self.read3(FRAME-0x828)[0],self.rd(FRAME-0x510,1)[0]],self.rd(BASE+0x15b0)
    def dispatch(self,flags,player):
        self.reset();self.u.reg_write(UC_X86_REG_R13,BASE);self.u.mem_write(BASE+0x84ec,bytes([bool(player)]));self.u.mem_write(BASE+0xa29c,struct.pack('<I',flags))
        choices=[]
        def hook(uc,address,size,data):
            if address in [0x101a3f97e,0x101a3f99c]:choices.append(address==0x101a3f99c);uc.emu_stop()
        h=self.u.hook_add(UC_HOOK_CODE,hook)
        self.u.emu_start(0x101a3f94f,0x101a3f99c+1,count=50);self.u.hook_del(h)
        return choices[0]
    def nozzle(self,direction,position,cg,thrust,mult,force,moment):
        self.reset();self.floats(BASE+0x160,force);self.floats(BASE+0x16c,moment)
        self.u.reg_write(UC_X86_REG_R12,BASE+0x1000);self.floats(BASE+0x1024,position)
        self.u.reg_write(UC_X86_REG_R14,BASE+0x2000);self.floats(BASE+0x204c,cg)
        self.u.reg_write(UC_X86_REG_R13,BASE+0x3000);self.floats(BASE+0x3000,direction)
        self.xmm(3,[mult]);self.xmm(0,[thrust])
        self.u.emu_start(0x1019f1f2b,0x1019f1fe2,count=100)
        return self.read3(BASE+0x160),self.read3(BASE+0x16c)
    def nozzle_direction(self,basis,sa,ca,sb,cb):
        self.reset();b0,b1,b2=basis
        self.xmm(0,[b0[2]]);self.xmm(3,[b1[2]]);self.xmm(8,[b2[2]])
        self.xmm(10,b0[:2]);self.xmm(9,b1[:2]);self.xmm(5,b2[:2])
        self.xmm(6,[sa]);self.xmm(7,[ca]);self.xmm(2,[-sb]);self.xmm(1,[cb])
        self.u.emu_start(0x1019f1606,0x1019f164f,count=60)
        return self.read3(BASE)
    def gyro(self,h,w):
        self.reset();self.u.reg_write(UC_X86_REG_RDI,BASE+0x1000);self.u.reg_write(UC_X86_REG_RSI,BASE);self.u.reg_write(UC_X86_REG_RDX,BASE+0x2000)
        self.doubles(BASE+0x25b40,h);self.doubles(BASE+0x2000,w)
        self.u.emu_start(0x101a17a44,0x101a17a96,count=50)
        return self.rd(BASE+0x1000)
    def realistic_scale(self,ext,base):
        self.reset();self.floats(BASE+0x84e4,[ext]);self.floats(BASE+0x7dfc,[base]);self.floats(BASE+0x7e24,[-1])
        self.u.reg_write(UC_X86_REG_R14,BASE+0x10000);self.floats(BASE+0x10004,[1]);self.floats(BASE+0x84e8,[1])
        self.u.emu_start(0x106c6347e,0x106c63616,count=150)
        return self.read_xmm(1)[0]
    def quaternion_step(self,q,w,dt):
        self.reset();self.floats(BASE+0x1570,q);self.u.reg_write(UC_X86_REG_RDI,BASE+0x1570)
        rad2deg=f32(57.2957763671875)
        for n,x in enumerate([-w[1]*dt,-w[2]*dt,w[0]*dt]):self.xmm(n,[f32(f32(x)*rad2deg)])
        self.u.reg_write(UC_X86_REG_RSP,FRAME-0x1000);self.u.mem_write(FRAME-0x1000,struct.pack('<Q',BASE+0x2f000))
        self.u.emu_start(0x10198bd30,BASE+0x2f000,count=400)
        return list(struct.unpack('<4f',self.u.mem_read(BASE+0x1570,16)))

def main():
    machine=BodyMachine();rng=random.Random(20260919);errors=[];counts={};maxerr={}
    def check(n,a,b):
        counts[n]=counts.get(n,0)+1
        if a!=b:errors.append(dict(stage=n,actual=a,expected=b))
    def vals(n,scale):return [rng.uniform(-scale,scale) for _ in range(n)]
    for k in range(600):
        q=vals(4,1);norm=math.sqrt(sum(x*x for x in q));q=[f32(x/norm) for x in q]
        mass=f32(rng.uniform(100,40000));a=vals(3,1e5);t=vals(3,1e9)
        check('gravity',machine.gravity(q,mass),gravity_body(q,mass))
        check('world_rotation',machine.rotation(q,a),rotate_acceleration_to_world(q,a))
        check('moment_cap',machine.cap(t,mass),limit_total_moment(t,mass))
        check('aerodynamic_force_cap',machine.force_cap(t,mass),limit_aerodynamic_force(t,mass))
        w=vals(3,4);inertia=vals(3,50000);c=vals(3,1e5)
        wp=vals(3,10);ias=rng.uniform(-1000,30000)
        check('angular_rate_preprocess',machine.rate_preprocess(wp,ias),preprocess_angular_rate(wp,ias))
        pv=vals(3,1200) if k>=3 else ([0,0,0] if k==0 else [10**(-8-k),0,0])
        pargs=(pv,rng.uniform(.1,1.3),rng.uniform(0,2),rng.uniform(0,1),rng.uniform(0,2),bool(k%2))
        check('parasite_force',machine.parasite(*pargs),parasite_force(*pargs))
        if k<3:inertia[k]=0
        check('angular_acceleration',machine.angular(w,inertia,t,c),angular_acceleration(w,inertia,t,c))
        p=vals(3,1e6);v=vals(3,1200);dt=f32(rng.uniform(.001,.05))
        check('translation',machine.translation(p,v,a,dt),integrate_translation(p,v,a,dt))
        e=vals(3,3000);s=f32(rng.uniform(.5,2))
        check('force_compose',machine.compose(a,e,t,s),compose_force(a,e,t,s))
        check('moment_compose',machine.moment_compose(a,e,t,c),compose_moment(a,e,t,c))
        check('airborne_linear_acceleration',machine.linear(a,t,mass),airborne_linear_acceleration(a,t,mass))
        check('realistic_tail_helper_bypass',machine.baseline_tail_scales(v,bool(k%2)),([1.,1.,1.,0.],v))
        direction=vals(3,1);pos=vals(3,10);cg=vals(3,2)
        f=vals(3,1e5);mom=vals(3,1e5);thrust=rng.uniform(0,2e5);mult=rng.uniform(-1,2)
        check('nozzle_force_moment',machine.nozzle(direction,pos,cg,thrust,mult,f,mom),accumulate_nozzle(direction,pos,cg,thrust,mult,f,mom))
        basis=[vals(3,1) for _ in range(3)];a0,a1=vals(2,math.pi);trig=list(map(f32,[math.sin(a0),math.cos(a0),math.sin(a1),math.cos(a1)]))
        check('nozzle_direction',machine.nozzle_direction(basis,*trig),nozzle_direction_from_sincos(basis,*trig))
        h=vals(3,1e5)
        check('gyroscopic_moment',machine.gyro(h,w),gyroscopic_moment(h,w))
        ext=rng.uniform(0,3);basemult=rng.uniform(0,4)
        check('realistic_engine_scale',machine.realistic_scale(ext,basemult),realistic_engine_scale(ext,basemult))
    for flags in [0,0x200000,0x1000000,0x1200000,0x786000b0,0xffffffff]:
        for player in [False,True]:check('dispatcher',machine.dispatch(flags,player),detailed_path(flags,player))
    probes=[]
    for axis in range(3):
        w=[0,0,0];w[axis]=1
        q=machine.quaternion_step([0,0,0,1],w,.01)
        probes.append(dict(stored_omega=w,dt=.01,quaternion=q))
        expected=[0.,0.,0.,math.cos(.005)];expected[axis]=-math.sin(.005)
        err=max(abs(x-y) for x,y in zip(q,expected));counts['quaternion_sign_probe']=counts.get('quaternion_sign_probe',0)+1
        if err>2e-7:errors.append(dict(stage='quaternion_sign_probe',actual=q,expected=expected))
    result=dict(binary_sha256=machine.sha,slice_executions=sum(counts.values()),counts=counts,failures=errors,quaternion_probes=probes,comparison='Exact numeric equality except quaternion sign probes (2e-7 absolute tolerance).',limitations='Finite synthetic prepared stages. No full body timestep. Nozzle direction accepts target sincos results; scalar thrust and control schedules are validated separately. Engine scale is tested with arcadeBoost=false, wepOverspeed=1.')
    Path('analysis/body-dynamics-validation.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k!='failures'},indent=2));print('FAILURES',len(errors));print(json.dumps(errors[:6],indent=2))
    if errors:raise SystemExit(1)
if __name__=='__main__':main()
