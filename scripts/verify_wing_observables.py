"""Complete original wing-flex helper and airborne observable producer."""
import json,random,struct
from pathlib import Path
from unicorn.x86_const import *
from verify_structural_limits import StrengthMachine
from verify_primary_controls import BASE,OUT
from macho_scan import MachO
from component_assembly import f32
from wing_observables import spring_step,wing_observables

class ObservableMachine(StrengthMachine):
 def __init__(self):
  super().__init__();m=MachO()
  for a,n in [(0x101a90000,0x1000),(0x101a2e000,0x1000)]:
   self.u.mem_map(a,n);self.u.mem_write(a,m.read(a,n))
 def spring(self,k,c,limit,x,v,forcing,dt):
  self.reset();self.floats(OUT,[k,c,limit,x,v]);self.xmm(0,[forcing]);self.xmm(1,[dt])
  self.u.reg_write(UC_X86_REG_RDI,OUT);self.call(0x101a907a0)
  return dict(position=self.read3(OUT+0xc)[0],velocity=self.read3(OUT+0xc)[1],result=self.read_xmm(0)[0])
 def observables(self,args,enabled):
  q,empty,frac,mults,strength,wy,ty,mx,ix,arm,mass,ias,vne,dt,state=args
  self.reset();self.u.mem_write(BASE+0x6ed8,struct.pack('<Q',BASE+0x14000))
  for off,vals in [(0x1570,q),(0x4f98,[empty]),(0x7c9c,[frac,*mults]),(0x81b4,[*strength,vne]),
                   (0x8148,[arm]),(0x5308,[mass]),(0x8468,[ias]),(0xa2f8,state[0]),(0xa30c,state[1])]:self.floats(BASE+off,vals)
  self.u.mem_write(BASE+0x5348,struct.pack('<d',ix));self.u.mem_write(OUT+0x22,bytes([enabled[1]]))
  self.u.mem_write(0x107d6fe90,bytes([enabled[0]]));self.floats(0x107d6fe9c,[1.])
  self.floats(0x107d6f794,[9.81]);self.floats(0x107d6fe70,[.88,1.,.9,1.15])
  self.u.reg_write(UC_X86_REG_RDI,BASE);self.u.reg_write(UC_X86_REG_RSI,OUT)
  for i,x in enumerate([*wy,ty,mx,dt]):self.xmm(i,[x])
  self.call(0x101a2e350)
  return dict(wing_wave=list(struct.unpack('<2f',self.u.mem_read(BASE+0x8a1c,8))),
              flex_state=[list(struct.unpack('<2f',self.u.mem_read(BASE+o,8))) for o in [0xa2f8,0xa30c]],
              overspeed_wave=self.read3(BASE+0x8a24)[0])

def main():
 m=ObservableMachine();r=random.Random(91504);errors=[];counts=dict(spring=0,observables=0,chained=0)
 def f(a,b):return f32(r.uniform(a,b))
 for i in range(2000):
  args=(f(0,1000),f(0,100),f(.2,2),f(-2,2),f(-10,10),f(-100,100),f32(r.choice([0.,1/480,1/48,1/60,-.001])))
  a=m.spring(*args);p=spring_step(*args);e=dict(position=p['position'],velocity=p['velocity'],result=p['position']);counts['spring']+=1
  if a!=e:errors.append(dict(stage='spring',args=args,actual=a,expected=e))
  args=([f(-1,1) for _ in range(4)],f(500,15000),f(.01,.4),[f(.01,1),f(.001,.05)],[-f(1e4,8e5),f(1e4,1e6)],
        [f(-1e6,1e6) for _ in range(2)],f(-1e6,1e6),f(-1e5,1e5),r.uniform(1e3,1e6),f(1,10),f(500,20000),f(10,600),f(200,500),
        f32(r.choice([1/30,1/48,1/60,1/120])),[[f(-1,1),f(-10,10)] for _ in range(2)])
  enabled=(i%3!=0,i%2!=0);a=m.observables(args,enabled);e=wing_observables(*args,spring_enabled=enabled[0],flutter_enabled=enabled[1]);counts['observables']+=1
  if a!=e:errors.append(dict(stage='observables',args=args,enabled=enabled,actual=a,expected=e))
 native=port=[[0.,0.],[0.,0.]]
 for i in range(600):
  args=([0.,0.,0.,1.],7850.,.125,[.5,.005],[-370000.,740000.],[40000.,41000.],85000.,10.,30000.,3.,10000.,350.,f32(1555/3.6),f32(1/48),native)
  a=m.observables(args,(True,True));e=wing_observables(*args[:-1],port);counts['chained']+=1
  native=a['flex_state'];port=e['flex_state']
  if a!=e:errors.append(dict(stage='chained',actual=a,expected=e))
 report=dict(binary_sha256=m.sha,counts=counts,failures=errors[:12],failure_count=len(errors),limitations='Complete original spring helper and airborne indicator producer101a2e350 with no hooks inside either function. Prepared inertia/forces/attitude and wing-flex properties. Both spring enable states and flutter settings tested; no-ground-contact only. Finite numerical equality.')
 Path('analysis/wing-observables-validation.json').write_text(json.dumps(report,indent=2)+'\n');print(counts,'FAILURES',len(errors));print(json.dumps(errors[:2],indent=2))
 if errors:raise SystemExit(1)
if __name__=='__main__':main()
