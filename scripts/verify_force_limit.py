"""Execute actual post-sum force limiter and runtime secondary-area assignments."""
import json,random,struct
from pathlib import Path
from unicorn.x86_const import *
from macho_scan import MachO
from component_assembly import f32,limit_aerodynamic_force
from component_stages import runtime_secondary_areas
from verify_component_stages import Stages,BASE,FRAME

class MoreStages(Stages):
    def __init__(self):
        super().__init__();m=MachO()
        for va,size in [(0x106c66000,0x1000),(0x107d6f000,0x1000),(0x101a3a000,0x1000)]:
            self.u.mem_map(va,size);self.u.mem_write(va,m.read(va,size))
    def xd(self,n,values):self.u.reg_write(UC_X86_REG_XMM0+n,int.from_bytes(struct.pack('<2d',*values),'little'))
    def limit(self,force,mass,scale,allow):
        self.reset();self.u.reg_write(UC_X86_REG_R14,BASE+0x10000);self.u.reg_write(UC_X86_REG_R12,0)
        self.u.reg_write(UC_X86_REG_RCX,0x7ff0000000000000);self.u.reg_write(UC_X86_REG_RDX,0x7fffffffffffffff)
        self.floats(BASE+0x5308,[mass]);self.floats(BASE+0x10014,[scale]);self.u.mem_write(BASE+0x7c08,bytes([allow]))
        self.xd(6,[force[2],force[0]]);self.xd(2,[force[1],0])
        self.u.emu_start(0x106c62d3a,0x106c62ea1,count=1000)
        xy=struct.unpack('<2d',self.u.mem_read(FRAME-0x4e0,16))
        return [*xy,self.read_xmm(5,'2d')[0]]
    def areas(self,fm):
        self.reset();a=fm['Aerodynamics']
        self.floats(BASE+0x733c,[a['FuselagePlane']['Areas']['Main']]);self.floats(BASE+0x6f2c,[a['HorStabPlane']['Areas']['Main'],a['HorStabPlane']['Areas']['Elevator']]);self.floats(BASE+0x7134,[a['VerStabPlane']['Areas']['Main'],a['VerStabPlane']['Areas']['Rudder']])
        self.u.reg_write(UC_X86_REG_R15,BASE+0x6ef8);self.u.reg_write(UC_X86_REG_RBX,BASE+0x8138)
        self.u.emu_start(0x101a3adbb,0x101a3ae3a,count=1000)
        return {n:struct.unpack('<f',self.u.mem_read(BASE+off,4))[0] for n,off in dict(fuselage=0x83fc,left_main=0x8420,left_elevator=0x8424,right_main=0x8428,right_elevator=0x842c,v_main=0x8430,rudder=0x8434).items()}

def main():
    m=MoreStages();rng=random.Random(16939);failures=[];area_results={};count=0
    for i in range(500):
        args=([f32(rng.uniform(-1e7,1e7)) for _ in range(3)],f32(rng.uniform(500,30000)),f32(rng.uniform(.5,2)),i%2==0)
        actual=m.limit(*args);expected=limit_aerodynamic_force(*args);count+=1
        if actual!=expected:failures.append(dict(stage='force_limit',actual=actual,expected=expected))
    for n in ['f_16a_block_15_adf','saab_jas39c']:
        fm=json.loads(Path('references/fm-2.59.0.13/'+n+'.blkx').read_text());actual=m.areas(fm);expected=runtime_secondary_areas(fm);area_results[n]=actual;count+=1
        if actual!=expected:failures.append(dict(stage='area_initialization',actual=actual,expected=expected))
    result=dict(binary_sha256=m.sha,slice_executions=count,failures=failures,area_results=area_results,comparison='Exact output equality',limitations='Force limiter tests use finite inputs only. Area initialization skips wing-geometry interpolation, which occurs earlier in the function.')
    Path('analysis/force-limit-validation.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
    if failures:raise SystemExit(1)
if __name__=='__main__':main()
