"""Locate RIP-relative address references, validating whole containing functions."""
import argparse,json,re,struct
from pathlib import Path
from capstone import Cs,CS_ARCH_X86,CS_MODE_64,CS_OP_MEM
from capstone.x86 import X86_REG_RIP
from macho_scan import MachO,disassemble


def refs(m,targets):
    sec=next(s for s in m.sections if s['name']=='__text');base=sec['va'];data=m.read(base,sec['size']);functions=set()
    # RIP ModR/M, four-byte displacement, and possible instruction immediate.
    for match in re.finditer(rb'(?=[\x05\x0d\x15\x1d\x25\x2d\x35\x3d](....))',data,re.S):
        pos=match.start();target=base+pos+5+struct.unpack('<i',match.group(1))[0]
        if any(target+immediate in targets for immediate in [0,1,2,4]):functions.add(m.function(base+pos))
    cs=Cs(CS_ARCH_X86,CS_MODE_64);cs.detail=True;result=[]
    for fn in sorted(functions):
        for ins in cs.disasm(m.read(fn,m.function_end(fn)-fn),fn):
            for op in ins.operands:
                if op.type==CS_OP_MEM and op.mem.base==X86_REG_RIP:
                    target=ins.address+ins.size+op.mem.disp
                    if target in targets:result.append(dict(function=hex(fn),address=hex(ins.address),target=hex(target),instruction=ins.mnemonic+' '+ins.op_str))
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('targets',nargs='+',type=lambda x:int(x,0));p.add_argument('--out',required=True);p.add_argument('--disassemble',action='store_true');a=p.parse_args()
    m=MachO();r=refs(m,set(a.targets));Path(a.out).write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
    if a.disassemble:
        for fn in sorted({int(x['function'],0) for x in r}):disassemble(m,fn,Path('analysis/disassembly'))
