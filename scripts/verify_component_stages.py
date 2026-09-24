"""Differential tests for local flow, application-point shift and final scaling."""
import json,random,struct
from pathlib import Path
from unicorn.x86_const import *
from macho_scan import MachO
from component_assembly import f32
from component_stages import local_flow,horizontal_application_point,secondary_forces
from verify_component_assembly import AssemblySlice,BASE,FRAME

class Stages(AssemblySlice):
    def __init__(self):
        super().__init__();m=MachO()
        for va,size in [(0x106c60000,0x2000),(0x1071e4000,0x30000)]:
            self.u.mem_map(va,size);self.u.mem_write(va,m.read(va,size))
    def reset(self):
        self.u.mem_write(BASE,bytes(0x30000))
        for n in range(16):self.xmm(n,[0])
        self.u.reg_write(UC_X86_REG_RBX,BASE);self.u.reg_write(UC_X86_REG_RBP,FRAME)
    def read3(self,addr):return list(struct.unpack('<3f',self.u.mem_read(addr,12)))
    def flow(self,p,c,v,w,d):
        self.reset();self.floats(BASE+0x5320,c)
        self.u.mem_write(BASE+0x15e0,struct.pack('<3d',*w));self.u.mem_write(BASE+0x1618,struct.pack('<3d',*v))
        self.xmm(0,p[:2]);self.xmm(1,[p[2]]);self.xmm(8,d[:2]);self.xmm(7,[d[2]])
        self.u.emu_start(0x106c609f0,0x106c60a89,count=300)
        return self.read3(FRAME-0x688)
    def point(self,p,cx,cy,cdadd,cm0,cm1):
        self.reset();self.u.reg_write(UC_X86_REG_R15,BASE+0x10000)
        self.floats(BASE+0x10034,[cm0,cm1]);self.floats(FRAME-0x7f0,p)
        self.floats(FRAME-0x490,[cdadd,0,0,0]);self.xmm(0,[cx,cy])
        self.u.emu_start(0x106c61c10,0x106c61c9d,count=300)
        return self.read3(FRAME-0x768)
    def forces(self,c,qt,qf,a,h,extra,proj,vs):
        self.reset()
        for key,off in dict(fuselage=0x83fc,left_main=0x8420,left_elevator=0x8424,right_main=0x8428,right_elevator=0x842c,v_main=0x8430,rudder=0x8434).items():self.floats(BASE+off,[a[key]])
        for key,off in dict(fuselage=0x18a0,left_main=0x18c4,left_elevator=0x18c8,right_main=0x18cc,right_elevator=0x18d0,v_main=0x18d4,rudder=0x18d8).items():self.floats(BASE+off,[h[key]])
        self.floats(FRAME-0x5f8,[extra]);self.floats(FRAME-0x9d0,[proj]);self.floats(FRAME-0x4c0,[qf])
        self.floats(FRAME-0x490,[c['left_hstab'][0]]);self.floats(FRAME-0x480,[c['right_hstab'][0]]);self.floats(FRAME-0x4a0,[c['vstab'][0]])
        self.xmm(0,[c['fuselage'][0]]);self.xmm(9,[f32(c['fuselage'][1]*qf)])
        self.xmm(12,[qt]);self.xmm(13,[f32(c['left_hstab'][1]*qt)]);self.xmm(14,[f32(c['right_hstab'][1]*qt)])
        self.xmm(1,[f32(float(vs)*float(qt)*float(c['vstab'][1]))])
        self.u.reg_write(UC_X86_REG_XMM11,int.from_bytes(struct.pack('<4I',*([0x80000000]*4)),'little'))
        self.u.emu_start(0x106c627a8,0x106c62914,count=300)
        return {n:self.read3(FRAME-o) for n,o in dict(fuselage=0x718,left_hstab=0x7d0,right_hstab=0x7c0,vstab=0x7b0).items()}

def main():
    m=Stages();rng=random.Random(390016);errors=[];counts=dict(local_flow=0,horizontal_point=0,secondary_force=0)
    def vals(n,scale):return [f32(rng.uniform(-scale,scale)) for _ in range(n)]
    def check(name,actual,expected):
        counts[name]+=1
        if actual!=expected:errors.append(dict(stage=name,actual=actual,expected=expected))
    for i in range(500):
        p,c,v,w,d=vals(3,10),vals(3,2),vals(3,1000),vals(3,2),vals(3,20)
        check('local_flow',m.flow(p,c,v,w,d),local_flow(p,c,v,w,d))
        args=[p,*vals(5,2)]
        if i==0:args=[p,0,0,0,1,1]
        check('horizontal_point',m.point(*args),horizontal_application_point(*args))
        coeff={n:vals(2,2) for n in ['left_hstab','right_hstab','vstab','fuselage']}
        a={n:abs(x) for n,x in zip(['fuselage','left_main','left_elevator','right_main','right_elevator','v_main','rudder'],vals(7,10))}
        h={n:abs(vals(1,1)[0]) for n in a}
        qt,qf=map(abs,vals(2,60000));extra=abs(vals(1,1)[0]);proj=abs(vals(1,1)[0]);vs=abs(vals(1,2)[0])
        args=(coeff,qt,qf,a,h,extra,proj,vs)
        check('secondary_force',m.forces(*args),secondary_forces(*args))
    report=dict(binary_sha256=m.sha,slice_executions=sum(counts.values()),counts=counts,failures=errors,comparison='Exact float32 output equality',limitations='Synthetic prepared inputs. Does not validate upstream coefficient/control/downwash generation, runtime area initialization or pressure input provenance.')
    Path('analysis/component-stages-validation.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k!='failures'},indent=2));print('FAILURES',len(errors));print(json.dumps(errors[:5],indent=2))
    if errors:raise SystemExit(1)
if __name__=='__main__':main()
