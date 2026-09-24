"""Differential tests of wing areas, coupled Cy and moment-equivalent points."""
import json, random, struct
from pathlib import Path
from unicorn.x86_const import *
from component_assembly import f32, add
from wing_stages import effective_areas, coupled_points
from macho_scan import MachO
from verify_component_stages import Stages
from verify_component_assembly import BASE, FRAME


class Wings(Stages):
    def __init__(self):
        super().__init__();m=MachO()
        self.u.mem_map(0x106c5c000,0x1000)
        self.u.mem_write(0x106c5c000,m.read(0x106c5c000,0x1000))

    def areas(self,areas,health):
        self.reset()
        for i in range(2):
            self.floats(BASE+0x8400+16*i,areas[i])
            self.floats(BASE+0x18a4+16*i,health[i])
        self.u.emu_start(0x106c5ccba,0x106c5cdb7,count=200)
        return [self.read3(FRAME-0x8f0)[0],self.read3(FRAME-0x880)[0]]

    def points(self,cx,cy,reference,extra,weights,dt,omega,area,span,positions,cm):
        self.reset()
        for off,values in [(0x620,[cy[0]]),(0x510,[cy[1]]),
                           (0x530,[reference[0]]),(0x4f0,[reference[1]]),
                           (0x5b0,[weights[0]]),(0x560,[weights[1]]),
                           (0x550,[add(cy[0],extra[0])]),(0x5d0,[add(cy[1],extra[1])]),
                           (0x5c0,[dt]),(0x4e0,[cx[0]]),(0x500,[cx[1]]),
                           (0x650,positions[0]),(0x640,positions[1]),
                           (0xb1c,cm[0]),(0xabc,cm[1])]:
            self.floats(FRAME-off,values)
        self.floats(BASE+0x8254,[area]);self.floats(BASE+0x8138,[span])
        self.floats(FRAME-0x480,[1.,0,0,0]);self.floats(FRAME-0x490,[1.,0,0,0])
        self.u.mem_write(BASE+0x15e0,struct.pack('<d',omega))
        self.u.emu_start(0x106c6021a,0x106c6049e,count=400)
        return dict(points=[self.read3(FRAME-0x8a8),self.read3(FRAME-0x898)],
                    cy=[self.read3(FRAME-0x6b0)[0],self.read_xmm(4)[0]])


def main():
    m=Wings();rng=random.Random(163941);failures=[]
    def vals(n,a=-2,b=2):return [f32(rng.uniform(a,b)) for _ in range(n)]
    for i in range(500):
        a=[vals(4,0,10) for _ in range(2)];h=[vals(4,0,1) for _ in range(2)]
        actual,expected=m.areas(a,h),effective_areas(a,h)
        if actual!=expected:failures.append(dict(stage='areas',case=i,actual=actual,expected=expected))
        args=[vals(2),vals(2),vals(2),vals(2),vals(2),vals(1,.001,.1)[0],
              rng.uniform(-2,2),vals(1,10,50)[0],vals(1,5,15)[0],
              [vals(3) for _ in range(2)],[vals(2) for _ in range(2)]]
        if i%10==0:args[6]=0.0
        if i%31==0:args[:4]=[[0.,0.] for _ in range(4)]
        actual,expected=m.points(*args),coupled_points(*args)
        expected.pop('blend')
        if actual!=expected:failures.append(dict(stage='points',case=i,actual=actual,expected=expected))
    report=dict(binary_sha256=m.sha,slice_executions=1000,failures=failures,
                comparison='Exact float32 output equality',
                limitations='Prepared rotated coefficients, reference coefficients, weighting factors and base points. Does not verify their producers or the later force-ordering/height corrections.')
    Path('analysis/wing-stages-validation.json').write_text(json.dumps(report,indent=2))
    print('SLICE EXECUTIONS',1000,'FAILURES',len(failures));print(json.dumps(failures[:5],indent=2))
    if failures:raise SystemExit(1)


if __name__=='__main__':main()
