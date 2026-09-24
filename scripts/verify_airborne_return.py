"""Complete selected aerodynamic/body-function return with empty airborne contacts.

Owner production is still outside this harness. Engine vectors are supplied at
an existing pause. Native contact, pose restoration and observable code execute.
"""
import json,math,random,struct
from pathlib import Path
from unicorn import UC_HOOK_CODE
from unicorn.x86_const import *
from verify_kinematics import KinematicsNative
from verify_aircraft_native import BASE,FRAME,PARAM,CONFIG,END
from aircraft_model import prepare,evaluate
from mass_model import aircraft_properties,evaluate as mass_eval
from component_assembly import f32,mul
from body_dynamics import gravity_body,limit_aerodynamic_force
from kinematics import airborne_step,altitude_velocity_correction,restore_small_pose_change
from structural_limits import wing_load_ratios
from wing_observables import wing_observables

class AirborneReturn(KinematicsNative):
 def __init__(self):
  super().__init__();self.visits={};self.active_stop=None
  self.u.hook_add(UC_HOOK_CODE,self.configure,begin=0x106c5c7d0,end=0x106c5c7d0)
  for a in [0x101a29000,0x101a290d6,0x101a2dd2e,0x101a2e341,0x101a2e350,0x106c64d5e]:
   self.u.hook_add(UC_HOOK_CODE,self.visit,begin=a,end=a)
  for a in [0x106c6331e,0x106c63e7f,0x106c649ed,END]:
   self.u.hook_add(UC_HOOK_CODE,self.stop,begin=a,end=a)
 def stop(self,u,a,size,data):
  if a==self.active_stop:u.emu_stop()
 def call(self,*args,**kw):
  self.active_stop=0x106c6331e;return super().call(*args,**kw)
 def extend(self,*args,**kw):
  self.active_stop=0x106c63e7f;return super().extend(*args,**kw)
 def integrate(self,*args,**kw):
  self.active_stop=0x106c649ed;return super().integrate(*args,**kw)
 def visit(self,u,a,size,data):self.visits[hex(a)]=self.visits.get(hex(a),0)+1
 def configure(self,u,a,size,data):
  f=self.model['fm'];strength=f['Aerodynamics']['WingPlane']['Strength']
  self.qword(BASE,0x1078da710);self.qword(BASE+8,(-0x10000)&0xffffffffffffffff)
  self.doubles(BASE+0x1558,self.initial_position);self.doubles(BASE+0x1580,self.initial_position)
  self.floats(BASE+0x1570,self.initial_quaternion);self.floats(BASE+0x1598,self.initial_quaternion)
  self.qword(PARAM+0x1008,PARAM+0x1100);self.floats(PARAM+0x1168,[self.collision_radius])
  self.u.mem_write(PARAM+0x1010,struct.pack('<I',1));self.floats(PARAM+0x1014,[1.])
  u.reg_write(UC_X86_REG_RCX,PARAM+0x1000);u.reg_write(UC_X86_REG_ESI,self.tick)
  # Empty nearby-object list, known terrain height and no ground contact.
  self.u.mem_write(BASE+0x6ce4,b'\x01');self.u.mem_write(0x107d6fc13,b'\x01')
  self.u.mem_write(0x107d6fc9c,b'\x00');self.u.mem_write(BASE+0x8470,b'\x01')
  self.floats(0x107d6fc18+0xcc,[1e-6,1e-8]);self.floats(0x107d6fc18+0x28c,[1.])
  self.floats(BASE+0x4f98,[f['Mass']['EmptyMass']]);self.floats(BASE+0x7c9c,[mul(f32(f['WingWaveMassRel']),.5),*f['WingSpringDampJointMult']])
  self.floats(BASE+0x81b4,[*strength['CritOverload'],mul(f32(strength['VNE']),f32(1/3.6)),strength['MNE']])
  self.floats(BASE+0x8970,[-1.,-1.]);self.floats(0x107d6f794,[9.81]);self.u.mem_write(0x107d6fe90,b'\x01')
  self.floats(0x107d6fe9c,[1.]);self.floats(0x107d6fe70,[.88,1.,.9,1.15])
  self.floats(BASE+0xa2f8,self.flex[0]);self.floats(BASE+0xa30c,self.flex[1])
 def finish(self):
  self.active_stop=END
  self.u.emu_start(0x106c649ed,END,count=30000)
  if self.u.reg_read(UC_X86_REG_RIP)!=END:raise RuntimeError('Full normal return not reached')
  return dict(position=self.read(BASE+0x1558,3,'d'),velocity=self.read(BASE+0x15b0,3,'d'),
              quaternion=self.read(BASE+0x1570,4),omega=self.read(BASE+0x15e0,3,'d'),
              angular_acceleration=self.read(BASE+0x15f8,3,'d'),world_acceleration=self.read(BASE+0x15c8,3,'d'),
              wing_load=self.read(BASE+0x8970,2),wing_wave=self.read(BASE+0x8a1c,2),overspeed_wave=self.read(BASE+0x8a24,1)[0],
              flex_state=[self.read(BASE+o,2) for o in [0xa2f8,0xa30c]],
              reaction_force=self.read(BASE+0x8898,3,'d'),reaction_moment=self.read(BASE+0x88c8,3,'d'))

def main():
 m=AirborneReturn();r=random.Random(19416);failures=[];count=0;coverage=dict(pose_restore=0,altitude_velocity_change=0)
 def fv(a,b):return f32(r.uniform(a,b))
 for name in ['f_16a_block_15_adf','saab_jas39c']:
  f=json.loads(Path('references/fm-2.59.0.13/'+name+'.blkx').read_text());model=prepare(f);strength=f['Aerodynamics']['WingPlane']['Strength']
  native_flex=port_flex=[[0.,0.],[0.,0.]]
  for i in range(300):
   mass=mass_eval(aircraft_properties(f),fuel_by_system=[fv(100,2000)])
   height=fv(1000,21000);v=[fv(50,700),fv(-100,100),fv(-30,30)];omega=[r.uniform(-.3,.3) for _ in range(3)]
   dt=f32(1e-6 if i%15==0 else r.choice([1/30,1/48,1/60,1/120]))
   q=[r.uniform(-1,1) for _ in range(4)];norm=math.sqrt(sum(x*x for x in q));q=[f32(x/norm) for x in q]
   p=[r.uniform(-1e4,1e4),height,r.uniform(-1e4,1e4)];m.initial_position=p;m.initial_quaternion=q;m.collision_radius=fv(2,20);m.tick=i+1;m.flex=native_flex
   hist=dict(wing_aoa=[0.,0.],body_angles=[0.,0.],wing_cl=[0.,0.],spin=0.)
   args=[model,v,omega,mass,[fv(-1,1) for _ in range(3)],height,dt,hist]
   native_aero=m.call(*args,full_update=True);aero=evaluate(*args,oil_radiator=0.)
   m.extend(aero['engine_force'],aero['engine_moment'],[0.]*3,[0.]*3,1.,f.get('ExtThrustBaseMult',1.),dt)
   m.integrate();a=m.finish()
   k=airborne_step(p,v,q,aero['omega_for_flow'],aero['force'],aero['stored_moment'],mass['mass'],mass['inertia'],dt)
   pose=restore_small_pose_change(p,q,k['position'],k['quaternion']);k.update(position=pose['position'],quaternion=pose['quaternion']);coverage['pose_restore']+=pose['restored']
   vy=altitude_velocity_correction(k['position'][1],k['velocity'][1],dt);coverage['altitude_velocity_change']+=vy!=k['velocity'][1];k['velocity'][1]=vy
   gravity=gravity_body(q,mass['mass']);wy=[aero['component_forces'][n][1] for n in ['left_wing','right_wing']]
   wave=wing_observables(k['quaternion'],f['Mass']['EmptyMass'],mul(f32(f['WingWaveMassRel']),.5),f['WingSpringDampJointMult'],strength['CritOverload'],wy,gravity[1],0.,mass['inertia'][0],model['geometry']['arm'][2],mass['mass'],aero['air']['ias_u'],mul(f32(strength['VNE']),f32(1/3.6)),dt,port_flex)
   e={key:k[key] for key in ['position','velocity','quaternion','omega','angular_acceleration','world_acceleration']}
   e.update(wave);e.update(wing_load=wing_load_ratios(wy,strength['CritOverload']),reaction_force=gravity,reaction_moment=[0.]*3)
   if limit_aerodynamic_force(aero['raw_aero_force'],mass['mass'])!=aero['raw_aero_force']:e['wing_load']=[-1.,-1.]
   bad=[dict(field=key,actual=a[key],expected=e[key]) for key in e if a[key]!=e[key]]
   if bad:failures.append(dict(aircraft=name,index=i,mismatches=bad))
   native_flex=a['flex_state'];port_flex=e['flex_state'];count+=1
 report=dict(binary_sha256=m.sha,cases=count,coverage=coverage,native_visits=m.visits,failure_count=len(failures),failures=failures[:8],limitations='Prepared selected full-real FM, with full-update bit and normal aerodynamic/body function return. Native contact routine enabled; known ground-height cache, explicit bounding radius, and empty nearby collision records select its genuine no-contact path. Original wing observable producer executes. Scalar math/property adapters and thread-local allocator-pointer getter remain hooks; engine vectors are supplied at raw-aero pause. Chained flex only; prescribed air state/controls/fuel, not a complete owner-driven timestep or live flight replay. Contact/terrain production outside airborne domain is not validated.')
 Path('analysis/airborne-return-validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
 if failures:raise SystemExit(1)
if __name__=='__main__':main()
