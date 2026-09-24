"""Original owner getter, flag packer and complete command-history consumer."""
import json,random,struct
from pathlib import Path
from unicorn import UC_HOOK_CODE
from unicorn.x86_const import *
from macho_scan import MachO
from verify_primary_controls import Controls,BASE,END
from verify_component_assembly import FRAME
from control_snapshots import SIZE,TICK,GENERATION,FLAGS,owner_uses_history,control_flags,select_snapshot


class SnapshotMachine(Controls):
    def __init__(self):
        super().__init__();m=MachO()
        for va,size in [(0x101a51000,0x1000),(0x1050d4000,0x1000)]:
            self.u.mem_map(va,size);self.u.mem_write(va,m.read(va,size))
        for va in [0x106e618c1,0x106e618c7]:self.u.hook_add(UC_HOOK_CODE,self.copy_hook,begin=va,end=va)
    def copy_hook(self,u,address,size,data):
        dest,src,n=[u.reg_read(r) for r in [UC_X86_REG_RDI,UC_X86_REG_RSI,UC_X86_REG_RDX]]
        u.mem_write(dest,bytes(u.mem_read(src,n)));u.reg_write(UC_X86_REG_RAX,dest)
        sp=u.reg_read(UC_X86_REG_RSP);ret=struct.unpack('<Q',u.mem_read(sp,8))[0]
        u.reg_write(UC_X86_REG_RSP,sp+8);u.reg_write(UC_X86_REG_RIP,ret)
    def owner(self,a,b,c,d,e):
        self.reset();self.u.mem_write(BASE+0x58,struct.pack('<II',a,b));self.u.mem_write(BASE+0x2f20,struct.pack('<I',c))
        self.u.mem_write(BASE+0xfa0,struct.pack('<Q',BASE+0x10000 if d else 0));self.u.mem_write(BASE+0x101d8,bytes([e]))
        self.u.reg_write(UC_X86_REG_RDI,BASE+0x100);self.call(0x1050d4e30)
        return bool(self.u.reg_read(UC_X86_REG_RAX)&255)
    def flags(self,a,b,c):
        self.reset();self.u.mem_write(FRAME-0xc0,struct.pack('<Q',BASE+0x10000))
        for off,val in [(0x21,a),(0x19,b),(0x56,c)]:self.u.mem_write(BASE+0x10000+off,bytes([val]))
        self.u.emu_start(0x101a4d4ad,0x101a4d4d2,count=100)
        return self.u.mem_read(BASE+0x4538,1)[0]
    def select(self,records,delivered,tick,prune,start_hint,generation):
        self.reset();buf=BASE+0x11000
        self.u.mem_write(BASE+0x48c8,struct.pack('<Q',buf));self.u.mem_write(BASE+0x48d8,struct.pack('<I',len(records)))
        if records:self.u.mem_write(buf,b''.join(records))
        self.u.mem_write(BASE+0x2b08,delivered)
        for reg,value in [(UC_X86_REG_RDI,BASE),(UC_X86_REG_ESI,tick&0xffffffff),(UC_X86_REG_EDX,int(prune)),
                          (UC_X86_REG_ECX,start_hint&0xffffffff),(UC_X86_REG_R8D,generation)]:self.u.reg_write(reg,value)
        self.call(0x101a51a10)
        n=struct.unpack('<I',self.u.mem_read(BASE+0x48d8,4))[0];index=self.u.reg_read(UC_X86_REG_EAX)
        if index&0x80000000:index-=0x100000000
        return bytes(self.u.mem_read(BASE+0x2b08,SIZE)),[bytes(self.u.mem_read(buf+i*SIZE,SIZE)) for i in range(n)],index


def main():
    rng=random.Random(193188);m=SnapshotMachine();fails=[];counts=dict(owner=0,flags=0,history=0)
    for i in range(1200):
        args=[rng.randrange(512),rng.choice([0,1,0x8000,0x8001,0xffff]),rng.randrange(3),bool(i%2),rng.randrange(256)]
        actual=m.owner(*args);expected=owner_uses_history(*args);counts['owner']+=1
        if actual!=expected:fails.append(dict(stage='owner',args=args))
        args=[bool(i&(1<<j)) for j in range(3)]
        actual=m.flags(*args);expected=control_flags(*args);counts['flags']+=1
        if actual!=expected:fails.append(dict(stage='flags',args=args,actual=actual,expected=expected))
        ticks=sorted(rng.sample(range(-20,60),rng.randrange(9)));records=[]
        for tick in ticks:
            raw=bytearray(rng.getrandbits(SIZE*8).to_bytes(SIZE,'little'))
            struct.pack_into('<i',raw,TICK,tick);raw[GENERATION]=rng.randrange(3);records.append(bytes(raw))
        delivered=bytes([0x55])*SIZE
        args=(records,delivered,rng.randrange(-25,65),bool(i%2),rng.randrange(-3,10),rng.randrange(3))
        actual=m.select(*args);expected=select_snapshot(*args);counts['history']+=1
        if actual!=expected:fails.append(dict(stage='history',ticks=ticks,options=args[2:],actual_index=actual[2],expected_index=expected[2],actual_count=len(actual[1]),expected_count=len(expected[1])))
    report=dict(binary_sha256=m.sha,counts=counts,failures=fails,
                limitations='Original complete owner getter and snapshot consumer plus flag packing slice. memcpy/memmove hooks copy exact bytes. All snapshot bytes and queue contents compared. Network scheduling, producer publication and callbacks are not replayed.')
    Path('analysis/control-snapshots-validation.json').write_text(json.dumps(report,indent=2));print(counts,'FAILURES',len(fails));print(json.dumps(fails[:5],indent=2))
    if fails:raise SystemExit(1)

if __name__=='__main__':main()
