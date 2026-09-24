"""Original full engine function with angled/vectoring/multiple prepared nozzles.

Only the external libm sincosf import is hooked. No aerodynamic or thrust code
is replaced. Property-loader parity is a separate obligation.
"""
import json,math,random,struct
from pathlib import Path
from unicorn import UC_HOOK_CODE
from unicorn.x86_const import *
from verify_jet_model import JetMachine,DATA
from jet_model import prepare as prepare_jet,scalar_update
from jet_nozzle import prepare,evaluate
from component_assembly import f32,mul
from control_mixer import density_at_height


class Nozzles(JetMachine):
    def __init__(self):
        super().__init__();self.u.mem_map(0x106e61000,0x1000)
        self.u.hook_add(UC_HOOK_CODE,self.sincos,begin=0x106e613e1,end=0x106e613e1)
    def sincos(self,u,address,size,data):
        angle=struct.unpack('<f',u.reg_read(UC_X86_REG_XMM0).to_bytes(16,'little')[:4])[0]
        u.reg_write(UC_X86_REG_XMM0,int.from_bytes(struct.pack('<2f',math.sin(angle),math.cos(angle)),'little'))
        sp=u.reg_read(UC_X86_REG_RSP);ret=struct.unpack('<Q',u.mem_read(sp,8))[0]
        u.reg_write(UC_X86_REG_RSP,sp+8);u.reg_write(UC_X86_REG_RIP,ret)
    def call(self,jet,nozzles,rho,speed,omega,throttle,dt,cg,ias,flaps,airbrake,vtol,reverse,sticks,main_controls,tip_axes,
             afterburner=True,health=1.,fuel_available=1e6,shaft_fraction=1.):
        self.load(jet);engine=DATA+0x7000;state=DATA+0x8000;base=DATA+0x9000
        self.write(engine,'2Q',DATA,base);self.write(base,'I',len(nozzles));address=DATA+0xc000
        for i,p in enumerate(nozzles):
            prop=base+8+i*0x198
            self.write(prop,'9f',*(v for b in p['basis'] for v in b))
            self.write(prop+0x24,'5f',*p['position'],p['ratio'],p['maximum'])
            self.write(prop+0x38,'B',p['main_controls']);self.write(prop+0x190,'B',p['tip'])
            self.write(prop+0xb8,'4f',*(v for r in p['ranges'] for v in r));self.write(prop+0xc8,'10f',*(v for r in p['weights'] for v in r))
            self.write(prop+0x138,'4f',*p['flaps'])
            for offset,key in [(0x40,'roll_angle'),(0x58,'pitch_angle'),(0x70,'yaw_angle'),(0x88,'vtol_angle'),(0xa0,'reverse_angle'),
                 (0xf0,'roll_thrust'),(0x108,'pitch_thrust'),(0x120,'yaw_thrust'),(0x148,'airbrake'),(0x160,'vtol'),(0x178,'reverse')]:
                rows=p[key];self.write(prop+offset,'Q',address);self.write(prop+offset+0x10,'I',len(rows))
                for x,inv,values in rows:
                    self.write(address,'f'*(2+len(values)),x,inv,*values);address+=4*(2+len(values))
        self.write(state,'3f',rho,speed,ias);self.write(state+0x10,'2fIf',omega,shaft_fraction,2,throttle)
        self.write(state+0x20,'2B',afterburner,main_controls);self.write(state+0x24,'7f',*sticks,flaps,airbrake,vtol,reverse)
        self.write(state+0x40,'3B',*tip_axes);self.write(state+0x44,'5f',health,fuel_available,*cg)
        self.u.reg_write(UC_X86_REG_RDI,engine);self.u.reg_write(UC_X86_REG_RSI,state);self.xmm(0,dt);self.run(0x1019f19e0)
        return dict(force=self.read(engine+0x160,3),moment=self.read(engine+0x16c,3),angles=[self.read(engine+0xe0+8*i,2) for i in range(len(nozzles))])


def main():
    m=Nozzles();rng=random.Random(194502);fails=[];count=0
    fm=json.loads(Path('references/fm-2.59.0.13/f_16a_block_15_adf.blkx').read_text());jet=prepare_jet(fm['EngineType0']['Main'])
    configs=[]
    for path in Path('references/jet-catalog/fm').glob('*.blkx'):
        raw=json.loads(path.read_text())
        for key,engine in raw.items():
            if key.startswith('Engine') and not key.startswith('EngineType') and isinstance(engine,dict):
                typ=raw.get('EngineType'+str(engine.get('Type',0)),engine)
                if typ.get('Main',{}).get('Type')=='Jet':
                    for k,v in engine.items():
                        if k.startswith('Nozzle') and isinstance(v,dict):configs.append((path.stem,k,v))
    for i in range(600):
        chosen=rng.sample(configs,1+i%4);ps=[prepare(p) for _,_,p in chosen]
        if i%2:
            for p in ps:
                p['main_controls']=bool(i%3);p['tip']=bool(i%5)
                # Prepared curve probes independently cover sign, interpolation,
                # range clamping and additive primary thrust modulation.
                for key,width in [('roll_angle',3),('pitch_angle',3),('yaw_angle',3),('roll_thrust',3),('pitch_thrust',3),('yaw_thrust',3)]:
                    p[key]=[(0.,f32(1/500),[f32(rng.uniform(-.4,.4)) for _ in range(width)]),(500.,0.,[f32(rng.uniform(-.4,.4)) for _ in range(width)])]
        val=lambda a,b:f32(rng.uniform(a,b))
        rho=density_at_height(val(0,15000));speed=val(0,700);omega=mul(val(.5,1.1),jet['max_omega']);throttle=1.;dt=f32(1/48)
        cg=[val(-2,2) for _ in range(3)];ias=val(0,650);flaps,airbrake,vtol,reverse=[val(0,1) for _ in range(4)]
        sticks=[val(-1,1) for _ in range(3)];main_controls=bool(i&2);tip_axes=[bool(i&k) for k in [1,2,4]]
        a=m.call(jet,ps,rho,speed,omega,throttle,dt,cg,ias,flaps,airbrake,vtol,reverse,sticks,main_controls,tip_axes)
        scalar=scalar_update(jet,rho,speed,omega,throttle,dt,afterburner=True)
        e=evaluate(ps,scalar['thrust'],cg,ias,flaps,airbrake,vtol,reverse,sticks,main_controls,tip_axes);e={k:e[k] for k in a}
        count+=1
        if a!=e:fails.append(dict(i=i,configs=[n+':'+k for n,k,_ in chosen],actual=a,expected=e))
    report=dict(cases=count,failures=fails,binary_sha256=m.sha,scope='Complete original scalar/nozzle engine; 1–4 prepared nozzles, real catalog and synthetic control tables. External sincosf hook; property loader not executed.')
    Path('analysis/general-nozzle-validation.json').write_text(json.dumps(report,indent=2));print('CASES',count,'FAILURES',len(fails));print(json.dumps(fails[:2],indent=2))
    if fails:raise SystemExit(1)
if __name__=='__main__':main()
