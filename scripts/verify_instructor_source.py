"""Original typed boolean search, then keyboard parity on corrected Kfir data."""
import json,random,struct
from pathlib import Path
from unicorn.x86_const import *
from instructor_native import InstructorNative,BASE,OBJ,INPUT,ARENA,STACK
from instructor_source import load
from jet_catalog import load as old_load,fuel_capacities
from aircraft_model import prepare,at_sweep
from mass_model import aircraft_properties,evaluate
from component_assembly import f32
from instructor_keyboard import keyboard_step
from verify_instructor_keyboard import extract_state,extract_history,extract_result


def main():
 n=InstructorNative();rng=random.Random(399803);fails=[];counts=dict(boolean_search=0,keyboard=0);examples=[]
 block=ARENA+0xb0000;table=block+0x1000;default=block+0x800;bp=STACK-0x1000
 for i in range(1000):
  rows=[(rng.randrange(4),rng.choice([9,9,3]),rng.randrange(2)) for _ in range(i%11)]
  requested=rng.randrange(5);fallback=i%2;dynamic=bool(i%2)
  n.u.mem_write(block,bytes(0x3000));n.u.mem_write(default,bytes([fallback]))
  raw=b''.join(struct.pack('<II',name|(kind<<24),value) for name,kind,value in rows)
  if dynamic:
   n.qword(block+0x18,block+0x900);n.qword(block+0x900,table);n.u.mem_write(block+0x14,struct.pack('<I',0xffffffff))
  else:n.qword(block,table-0x90)
  n.u.mem_write(block+0xc,struct.pack('<H',len(rows)))
  if raw:n.u.mem_write(table,raw)
  n.qword(bp-0x30,default);n.qword(bp-0x38,block)
  for reg,v in [(UC_X86_REG_EAX,requested),(UC_X86_REG_RCX,block),(UC_X86_REG_RDX,default),(UC_X86_REG_RBP,bp)]:n.u.reg_write(reg,v)
  n.u.emu_start(0x102e6de25,0x102e6dd8e,count=2000)
  assert n.u.reg_read(UC_X86_REG_RIP)==0x102e6dd8e
  actual=n.u.mem_read(n.u.reg_read(UC_X86_REG_RDX),1)[0]&1
  match=next((r for r in rows if r[0]==requested),None);expected=match[2] if match and match[1]==9 else fallback
  counts['boolean_search']+=1
  if actual!=expected:fails.append(dict(stage='boolean',rows=rows,requested=requested,actual=actual,expected=expected))
 for name in ['kfir_c10_colombia','kfir_c2','kfir_c7','kfir_canard']:
  fm=load(name);base=prepare(fm);mass=evaluate(aircraft_properties(fm),[f32(v*.3) for v in fuel_capacities(fm)])
  examples.append(dict(aircraft=name,source=old_load(name)['InvertElevator'],resolved=fm['InvertElevator']))
  for i in range(8):
   flaps=f32([0.,.37,1.,0.][i%4]);sweep=f32([0.,.4,1.,.7][i%4]);model=at_sweep(base,sweep)
   n.setup(model,mass,speed=[100.,250.,350.,180.][i%4],alpha=[4.,30.,-12.,18.][i%4],flaps=flaps,sweep=sweep,height=1000.,mode_lane=True)
   actor=n.empty_payload_actor();n.qword(actor+0x2ef0,BASE)
   n.u.mem_write(INPUT+2,bytes([i!=2]));n.u.mem_write(INPUT+0x29,bytes([i>=4]));n.floats(INPUT+0x10,[0.]);n.floats(INPUT+0x44,[1.])
   n.u.mem_write(0x107d6fbc0,b'\1\1');n.u.mem_write(BASE+0x3658,b'\3');n.u.mem_write(0x107d6fbf5,b'\1');n.u.mem_write(OBJ+0x48,bytes([i%2]))
   commands=[f32(.15),-1. if i%4<2 else 1.,f32(-.05)];n.floats(BASE+0x8514,commands);n.floats(OBJ+0xe0,[commands[1],commands[0],commands[2]])
   n.invoke(0x104f70ef0,[actor,INPUT+0x40,INPUT+0x41,INPUT+0x42]);n.floats(BASE+0x84fc,[.2,.3,.4]);n.floats(BASE+0x8508,[.03,.02,.01])
   assert n.u.mem_read(BASE+0x7c54,1)==b'\0'
   state=extract_state(n,model,flaps);history=extract_history(n);e=keyboard_step(model,state,history,n.dt);n.step();a=extract_result(n);counts['keyboard']+=1
   bad={k:dict(actual=v,expected=e[k]) for k,v in a.items() if v!=e[k]}
   if bad:fails.append(dict(stage='keyboard',case=[name,i],fields=bad))
 report=dict(binary_sha256=n.sha,counts=counts,examples=examples,failures=fails,
  scope='Original named-getter parameter-search/type-selection102e6de25..6dd8e after an explicit name ID lookup; both inline/dynamic typed parameter storage, first matching key, bool type9/default behavior. Corrected source booleans then full keyboard command/history parity on four Kfir records. Original whole-aircraft loader/parser not executed. Production normalization now uses the same first-occurrence boolean rule.')
 Path('analysis/instructor-full/source-validation.json').write_text(json.dumps(report,indent=2)+'\n');print(counts,'failures',len(fails),fails[:1])
 if fails:raise SystemExit(1)

if __name__=='__main__':main()
