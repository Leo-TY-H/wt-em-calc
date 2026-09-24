"""Execute force/moment instruction slices with explicit synthetic component inputs.

This validates assembly of already-computed forces, not their upstream generation.
All points/forces use the executable's stored coordinate convention. Inputs are
rounded to float32; moment products/sums are float32 before promotion to doubles.
"""
import hashlib,json,math,random,struct
from pathlib import Path
from unicorn import Uc,UC_ARCH_X86,UC_MODE_64
from unicorn.x86_const import *
from macho_scan import MachO
from component_assembly import assemble_force,assemble_moment
from verify_polar_machine_code import EXPECTED_BINARY_SHA256

BASE=0x210000000; FRAME=0x210020000
FORCE_OFFSETS={'left_wing':0x738,'right_wing':0x728,'left_hstab':0x7d0,'right_hstab':0x7c0,'vstab':0x7b0,'fuselage':0x718,'parasite':0x910}
POINT_OFFSETS={'left_wing':0x8a8,'right_wing':0x898,'left_hstab':0x768,'right_hstab':0x758,'vstab':0x930,'fuselage':0x920}
NAMES=list(POINT_OFFSETS)+['chute']

def f32(x):return struct.unpack('<f',struct.pack('<f',x))[0]
def vec32(v):return [f32(x) for x in v]
def cross(a,b):return [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]
def raw_moment(forces,points,cog):
    terms=[cross(forces[n],vec32([points[n][i]-cog[i] for i in range(3)])) for n in NAMES]
    return [sum(t[i] for t in terms) for i in range(3)]

class AssemblySlice:
    def __init__(self):
        m=MachO()
        self.sha=hashlib.sha256(m.data).hexdigest()
        if self.sha!=EXPECTED_BINARY_SHA256:raise ValueError('Binary changed; remap instruction addresses first')
        self.u=Uc(UC_ARCH_X86,UC_MODE_64)
        self.u.mem_map(0x106c62000,0x2000);self.u.mem_write(0x106c62000,m.read(0x106c62000,0x2000))
        self.u.mem_map(BASE,0x30000)
    def floats(self,addr,v):self.u.mem_write(addr,struct.pack('<'+'f'*len(v),*v))
    def xmm(self,n,v):self.u.reg_write(UC_X86_REG_XMM0+n,int.from_bytes(struct.pack('<4f',*(list(v)+[0]*4)[:4]),'little'))
    def read_xmm(self,n,fmt='4f'):return struct.unpack('<'+fmt,self.u.reg_read(UC_X86_REG_XMM0+n).to_bytes(16,'little'))
    def state(self,forces,points,cog):
        self.u.mem_write(BASE,bytes(0x30000))
        for n in range(16):self.xmm(n,[0])
        self.u.reg_write(UC_X86_REG_RBX,BASE);self.u.reg_write(UC_X86_REG_RBP,FRAME)
        for name,offset in FORCE_OFFSETS.items():self.floats(FRAME-offset,forces.get(name,[0,0,0]))
        for name,offset in POINT_OFFSETS.items():self.floats(FRAME-offset,points[name])
        self.floats(BASE+0x5320,cog);self.floats(BASE+0x8074,points['chute'])
    def moment(self,forces,points,cog):
        self.state(forces,points,cog)
        self.xmm(14,[forces['right_wing'][0]])
        self.xmm(1,[forces['left_wing'][0]])
        self.xmm(4,[forces['left_hstab'][0],forces['right_hstab'][0]])
        self.xmm(13,[forces['right_wing'][1]])
        self.xmm(11,[forces['vstab'][0]])
        self.floats(FRAME-0x480,[forces['left_wing'][1]])
        self.floats(FRAME-0x490,[forces['chute'][2],0,0,0])
        self.floats(FRAME-0x4b0,[forces['chute'][0],forces['chute'][1],0,0])
        self.floats(FRAME-0x4c0,[forces['chute'][1],0,0,0])
        self.u.emu_start(0x106c6300b,0x106c6331e,count=1000)
        return [self.read_xmm(2,'2d')[0],*self.read_xmm(0,'2d')]
    def force(self,forces,points,cog):
        self.state(forces,points,cog)
        self.xmm(8,[forces['left_wing'][2]])
        self.xmm(10,[forces['right_wing'][2]])
        self.xmm(9,[forces['fuselage'][2]])
        self.xmm(3,[forces['vstab'][2]])
        self.xmm(15,forces['chute'][:2])
        self.xmm(12,[forces['chute'][2]])
        self.floats(FRAME-0x5b0,[forces['parasite'][2],0,0,0])
        self.u.emu_start(0x106c62c45,0x106c62d3a,count=1000)
        zx=self.read_xmm(6,'2d');y=self.read_xmm(2,'2d')[0]
        return [zx[1],y,zx[0]]

def main():
    machine=AssemblySlice();rng=random.Random(160039);failures=[];max_errors={'force':0,'moment':0};cases=0
    def check(label,forces,points,cog):
        nonlocal cases
        forces={n:vec32(forces.get(n,[0]*3)) for n in NAMES+['parasite']}
        points={n:vec32(points.get(n,[0]*3)) for n in NAMES};cog=vec32(cog)
        for kind,expected in [('moment',assemble_moment(forces,points,cog)),('force',assemble_force(forces))]:
            actual=getattr(machine,kind)(forces,points,cog)
            error=max(abs(a-b) for a,b in zip(actual,expected));max_errors[kind]=max(max_errors[kind],error)
            scale=max(1,sum(sum(abs(x) for x in v) for v in forces.values()))
            if kind=='moment':scale*=max(1,max(abs(points[n][i]-cog[i]) for n in NAMES for i in range(3)))
            if actual!=expected:failures.append(dict(case=label,kind=kind,actual=actual,expected=expected,error=error))
        cases+=1
    for n in NAMES+['parasite']:
        for axis in range(3):
            f=[0,0,0];f[axis]=1
            check(n+str(axis),{n:f},{name:[2,3,5] for name in NAMES},[.5,-.25,1])
    for i in range(500):
        check(str(i),{n:[rng.uniform(-1e5,1e5) for _ in range(3)] for n in NAMES+['parasite']},{n:[rng.uniform(-10,10) for _ in range(3)] for n in NAMES},[rng.uniform(-2,2) for _ in range(3)])
    report=dict(binary_sha256=machine.sha,cases=cases,slice_executions=cases*2,max_absolute_error=max_errors,failures=failures,limitations='Synthetic already-computed component forces/points. Excludes upstream force generation, total-force limiter, helper moments, propulsion/gravity/contact. Raw internal frame; moments are F cross (position-CoG). Parasite force has no arm term in this block; chute does.')
    Path('analysis/component-assembly-validation.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k!='failures'},indent=2));print('FAILURES',len(failures));print(json.dumps(failures[:8],indent=2))
    if failures:raise SystemExit(1)
if __name__=='__main__':main()
