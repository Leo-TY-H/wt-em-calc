"""Execute the original nozzle-loader basis arithmetic with DataBlock/libm hooks."""
import json,math,random,struct
from pathlib import Path
from unicorn import UC_HOOK_CODE
from unicorn.x86_const import *
from verify_general_nozzles import Nozzles
from verify_jet_model import DATA,STACK
from macho_scan import MachO
from jet_nozzle import direction_basis
from component_assembly import f32

class Basis(Nozzles):
    def __init__(self):
        super().__init__();m=MachO();self.u.mem_map(0x1019ec000,0x3000);self.u.mem_write(0x1019ec000,m.read(0x1019ec000,0x3000))
        self.u.mem_map(0x102e6e000,0x1000);self.u.hook_add(UC_HOOK_CODE,self.reader,begin=0x102e6e4b0,end=0x102e6e4b0)
    def reader(self,u,address,size,data):
        values=self.inputs.pop(0)
        if values is None:
            values=struct.unpack('<2f',u.mem_read(u.reg_read(UC_X86_REG_RDX),8))
        u.reg_write(UC_X86_REG_XMM0,int.from_bytes(struct.pack('<2f',*values),'little'))
        sp=u.reg_read(UC_X86_REG_RSP);ret=struct.unpack('<Q',u.mem_read(sp,8))[0]
        u.reg_write(UC_X86_REG_RSP,sp+8);u.reg_write(UC_X86_REG_RIP,ret)
    def call(self,a,b):
        self.inputs=[a,b];self.u.mem_write(STACK,bytes(0x10000));self.u.mem_write(DATA+0x9000,bytes(0x1000))
        for reg,value in [(UC_X86_REG_RSP,STACK+0x7000),(UC_X86_REG_RBP,STACK+0x8000),(UC_X86_REG_RBX,DATA+0x9008),(UC_X86_REG_R12,0),(UC_X86_REG_R14,DATA+0x100)]:self.u.reg_write(reg,value)
        self.u.emu_start(0x1019ec9e8,0x1019ecc4f,count=10000)
        if self.u.reg_read(UC_X86_REG_RIP)!=0x1019ecc4f:raise RuntimeError('Loader fragment failed')
        v=self.read(DATA+0x9008,9);return [v[i:i+3] for i in [0,3,6]]

def main():
    rng=random.Random(19819);m=Basis();cases=[];fails=[]
    for path in Path('references/jet-catalog/fm').glob('*.blkx'):
        fm=json.loads(path.read_text())
        for key,engine in fm.items():
            if key.startswith('Engine') and isinstance(engine,dict):
                for name,nozzle in engine.items():
                    if name.startswith('Nozzle') and isinstance(nozzle,dict):cases.append((nozzle.get('Direction'),nozzle.get('Direction2')))
    cases+= [([rng.uniform(-180,180),rng.uniform(-90,90)],[rng.uniform(-180,180),rng.uniform(-90,90)]) for _ in range(1000)]
    for i,(a,b) in enumerate(cases):
        actual=m.call(a,b);expected=direction_basis(a or [0.,0.],b or [-f32(math.pi/2),0.])
        if actual!=expected:fails.append(dict(i=i,a=a,b=b,actual=actual,expected=expected))
    report=dict(cases=len(cases),failures=fails,binary_sha256=m.sha,scope='Original loader basis fragment. DataBlock Point2 getter and external sincosf only are hooked; includes catalog directions and random orientation/degeneracy inputs.')
    Path('analysis/nozzle-basis-validation.json').write_text(json.dumps(report,indent=2));print('CASES',len(cases),'FAILURES',len(fails));print(json.dumps(fails[:2],indent=2))
    if fails:raise SystemExit(1)
if __name__=='__main__':main()
