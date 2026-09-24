"""Execute the authentic flap actuator and mechanism-limit helper blocks."""
import json,random,struct
from pathlib import Path
from unicorn import UC_HOOK_CODE
from unicorn.x86_const import *
from macho_scan import MachO
from verify_wing_model import WingMachine
from verify_component_assembly import BASE,FRAME
from component_assembly import f32,mul
from wing_actuators import flap_step


class Actuators(WingMachine):
    def __init__(self):
        super().__init__();m=MachO()
        self.u.mem_map(0x101a4b000,0x4000);self.u.mem_write(0x101a4b000,m.read(0x101a4b000,0x4000))
        for a in [0x101a4cfde,0x101a4cc2a]:self.u.hook_add(UC_HOOK_CODE,self.stop_hook,begin=a,end=a)

    def flap(self,fm,requested,actual,mach,ias,dt,bounds):
        self.reset();self.u.reg_write(UC_X86_REG_RDI,BASE)
        self.u.mem_write(BASE+0x7fa6,bytes([fm['AvailableControls']['hasFlapsControl']]))
        self.qword(BASE+0x6ed8,BASE+0x10000)
        self.floats(BASE+0x87dc,[requested]);self.floats(BASE+0x3a40,[actual])
        self.floats(BASE+0x8464,[mach,ias]);self.floats(BASE+0x8834,bounds)
        self.floats(BASE+0x7d20,[0.,1.,100.,2147440000.]);self.u.mem_write(BASE+0x7d30,struct.pack('<i',-1))
        self.floats(BASE+0x7fec,fm['dvFlapsOut']);self.floats(BASE+0x7ffc,fm['dvFlapsIn'])
        self.floats(BASE+0x800c,fm.get('flapsLimByMach',[.5,.7,1.,1.]));self.floats(BASE+0x801c,fm.get('flapsLimByIas',[0.,3000.,1.,1.]))
        axis=fm['Aerodynamics']['FlapsAxis']
        for i,k in enumerate(['Retracted','Combat','Takeoff','Landing']):
            self.u.mem_write(BASE+0x7bb8+i*8,bytes([axis[k]['Presents']]))
            self.floats(BASE+0x7bbc+i*8,[axis[k]['Flaps']])
        self.floats(FRAME-0x60,[dt]);self.floats(FRAME-0x70,[mul(ias,f32(3.6))])
        self.xmm(6,[mach]);self.xmm(7,[mul(ias,f32(3.6))])
        self.run(0x101a4cbdb,0x101a4cfde if fm['AvailableControls']['hasFlapsControl'] else 0x101a4cc2a,3000)
        return dict(requested=self.single(BASE+0x87dc),actual=self.single(BASE+0x3a40))


def main():
    m=Actuators();rng=random.Random(1603904);fails=[];count=0
    for name in ['f_16a_block_15_adf','saab_jas39c']:
        fm=json.loads(Path('references/fm-2.59.0.13/'+name+'.blkx').read_text())
        for j in range(1000):
            args=[f32(rng.uniform(0,1)),f32(rng.uniform(0,1)),f32(rng.uniform(0,1.5)),f32(rng.uniform(0,400)),f32(rng.uniform(.001,.1))]
            bounds=(0.,1.) if j%2 else sorted([f32(rng.uniform(0,1)),f32(rng.uniform(0,1))])
            a=m.flap(fm,*args,bounds);b=flap_step(fm,*args,bounds);count+=1
            if a!=b:fails.append(dict(aircraft=name,args=args,bounds=bounds,actual=a,expected=b))
    report=dict(binary_sha256=m.sha,cases=count,failures=fails,
                limitations='Actual actuator block and default mechanism-limit helper execute natively. Supplied mechanism range may represent damage/external restriction; its producer is not covered. Simulation-state copy/interpolation into FM2b60 not exercised.')
    Path('analysis/wing-actuators-validation.json').write_text(json.dumps(report,indent=2))
    print('CASES',count,'FAILURES',len(fails));print(json.dumps(fails[:5],indent=2))
    if fails:raise SystemExit(1)


if __name__=='__main__':main()
