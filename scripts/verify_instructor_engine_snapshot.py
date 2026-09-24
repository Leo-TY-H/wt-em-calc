"""Whole original owner engine-restore loop versus independently copied state."""
import json,random,struct
from pathlib import Path
from instructor_native import InstructorNative,ARENA
from instructor_engine_snapshot import restore_jet_engines
from component_assembly import f32


def main():
    n=InstructorNative();u=n.u;rng=random.Random(9201534);failures=[];calls=0;engine_records=0
    owner=ARENA+0x40000;snapshot=ARENA+0x80000;dynamic=ARENA+0x83000;enginebase=ARENA+0x85000;propbase=ARENA+0x90000
    for case in range(1000):
        ne=[1,2,4,5,8][case%5];ns=case%9;engines=[];props=[];records=[]
        u.mem_write(owner,bytes(0x26100));u.mem_write(snapshot,bytes(0x4000))
        u.mem_write(owner+0x25f78,struct.pack('<i',ne));u.mem_write(snapshot+0x3e0,struct.pack('<i',ns))
        if ns>=5:n.qword(snapshot+0x3e8,dynamic)
        for j in range(max(ne,ns)):
            s=bytearray(rng.getrandbits(8) for _ in range(0xb8))
            for off in list(range(4,0x20,4))+list(range(0x24,0x6c,4))+[0x70,0x7c,0x80,0x84,0x88,0x8c,0x98,0xa0,0xa8,0xac,0xb0,0xb4]:
                struct.pack_into('<f',s,off,f32(rng.uniform(-100,100)))
            capacity=rng.randrange(9);s[0x6c]=rng.randrange(capacity+1);s[0x6d]=rng.randrange(capacity+1)
            for off in [0x94,0x9c]:struct.pack_into('<i',s,off,rng.randrange(4))
            e=bytearray(rng.getrandbits(8) for _ in range(0x400));p=bytearray(0x300);p[0]=2
            struct.pack_into('<i',p,0x244,rng.randrange(1,32));struct.pack_into('<i',p,0x2b4,case%4);struct.pack_into('<f',p,0x2b8,f32(rng.uniform(0,100)))
            struct.pack_into('<Q',e,0,propbase+j*0x400);struct.pack_into('<I',e,0x2b8,capacity)
            struct.pack_into('<f',e,0xe4,f32(rng.choice([0.,1e-20,rng.uniform(-10,10)])))
            if j<ne:
                engines.append(bytes(e));props.append(bytes(p));n.qword(owner+0x25f80+j*8,enginebase+j*0x800)
                u.mem_write(enginebase+j*0x800,bytes(e));u.mem_write(propbase+j*0x400,bytes(p))
            if j<ns:
                records.append(bytes(s));u.mem_write((dynamic if ns>=5 else snapshot+0x3e4)+j*0xb8,bytes(s))
        expected=restore_jet_engines(engines,props,records)
        n.invoke(0x101a15340,[owner,snapshot])
        for j,wanted in enumerate(expected):
            actual=bytes(u.mem_read(enginebase+j*0x800,0x400))
            if actual!=wanted:failures.append(dict(case=case,engine=j,owner_count=ne,snapshot_count=ns,
                byte_offsets=[hex(i) for i,(a,e) in enumerate(zip(actual,wanted)) if a!=e]))
        calls+=1;engine_records+=min(ne,ns)
    report=dict(binary_sha256=n.sha,owner_calls=calls,restored_engine_records=engine_records,failure_count=len(failures),failures=failures[:12],
        scope='Complete original101a15340 engine restore loop and1019f9f20/1019eb400 children; all1024 bytes per prepared engine compared, including unchanged fields. Native bzero import has explicit zeroing adapter. One/two/four/five/eight engine owners, inline/dynamic snapshots and count mismatches. Finite raw type2-engine records; thermal histories restored, not thermally simulated. No snapshot producer or whole owner-frame replay.')
    Path('analysis/instructor-full/engine-snapshot-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print('owner calls',calls,'engine records',engine_records,'failures',len(failures),failures[:2])
    if failures:raise SystemExit(1)


if __name__=='__main__':main()
