"""Find direct-displacement references, validating candidate instruction operands.

This does not prove absence of indirect/base-adjusted references or object type.
"""
import argparse,json,struct
from capstone import Cs,CS_ARCH_X86,CS_MODE_64,CS_OP_MEM
from macho_scan import MachO
from pathlib import Path

def refs(m,offsets,lo=None,hi=None):
    s=next(s for s in m.sections if s['name']=='__text');start=max(s['va'],lo or s['va']);end=min(s['va']+s['size'],hi or s['va']+s['size']);data=m.read(start,end-start);functions=set()
    for off in offsets:
        pat=struct.pack('<I',off);pos=0
        while True:
            pos=data.find(pat,pos)
            if pos<0:break
            functions.add(m.function(start+pos));pos+=1
    cs=Cs(CS_ARCH_X86,CS_MODE_64);cs.detail=True;result=[]
    for fn in sorted(functions):
        for ins in cs.disasm(m.read(fn,m.function_end(fn)-fn),fn):
            if any(op.type==CS_OP_MEM and op.mem.disp in offsets for op in ins.operands):
                result.append(dict(function=hex(fn),address=hex(ins.address),mnemonic=ins.mnemonic,operands=ins.op_str))
    return result
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('offsets',nargs='+',type=lambda x:int(x,0));p.add_argument('--lo',type=lambda x:int(x,0));p.add_argument('--hi',type=lambda x:int(x,0));p.add_argument('--out',required=True);a=p.parse_args()
    r=refs(MachO(),a.offsets,a.lo,a.hi);Path(a.out).write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
